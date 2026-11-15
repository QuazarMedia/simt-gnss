package app.simt.gnss

import android.location.GnssMeasurement
import android.location.GnssMeasurementsEvent
import android.location.GnssNavigationMessage
import android.location.GnssStatus
import android.os.Build
import android.util.Log
import app.simt.gnss.core.correction.RtkBaseStore
import app.simt.gnss.core.correction.SsrCorrections
import app.simt.gnss.core.filter.DopplerSmoother
import app.simt.gnss.core.filter.GnssInterEpochKalmanFilter
import app.simt.gnss.core.filter.HatchFilter
import app.simt.gnss.core.filter.KalmanCoast
import app.simt.gnss.core.filter.receiverClockStepMeters
import app.simt.gnss.core.model.GnssSignalBand
import app.simt.gnss.core.model.PositionResult
import app.simt.gnss.core.model.SatelliteEphemeris
import app.simt.gnss.core.model.SatelliteMeasurement
import app.simt.gnss.core.parser.SbasNavMessageParser
import app.simt.gnss.core.solver.GnssPositionSolver.Accuracy
import app.simt.gnss.core.utils.GnssMathUtils
import app.simt.gnss.core.utils.GnssMathUtils.SPEED_OF_LIGHT
import kotlin.math.*

/**
 * Orchestrates GNSS measurement processing and position computation.
 *
 * @param suppressSbas When `true`, SBAS (WAAS, EGNOS, MSAS, GAGAN, SDCM, …) is completely bypassed:
 *   SBAS L1 navigation messages are not decoded, SBAS satellites are not used as ranging sources,
 *   and SBAS fast/long-term/iono-grid corrections are never applied. Intended for dual-frequency
 *   (L1 + L5 / E5) devices, where the L1/L5 iono-free combination is substantially more accurate
 *   than SBAS grid corrections; skipping SBAS also avoids the CPU/battery cost of decoding the
 *   250 bps SBAS message stream when it cannot improve the solution.
 */
internal class GnssMeasurementProcessor(
    private val suppressSbas: Boolean = false,
) {
    init {
        AndroidGnssPlatform.install()
    }

    private val lock = Any()
    private val hatchFilter = HatchFilter()
    private val dopplerSmoother = DopplerSmoother()

    /**
     * Previous epoch's `fullBiasNanos`, or `null` before one exists. Kept as a `Long`: the value is
     * ~1.4×10¹⁸ ns and a `Double` difference at that magnitude is quantized to 256 ns (≈ 77 m).
     * [lastBiasNanos] holds the sub-nanosecond part, which is already a `Double` in the platform API.
     */
    private var lastFullBiasNanos: Long? = null
    private var lastBiasNanos: Double = 0.0

    /**
     * Hardware clock discontinuity count at the previous epoch, or `null` before one exists. A
     * change means the clock was rebased, so [lastFullBiasNanos] belongs to a different clock and
     * must not be differenced.
     */
    private var lastHardwareClockDiscontinuityCount: Int? = null

    internal fun getStoredEphemerisForTest(
        constellationType: Int,
        svid: Int,
    ): SatelliteEphemeris? = synchronized(lock) { ephemerisStore.get(constellationType to svid) }

    internal fun getLastKnownLeapSecondsForTest(): Int = synchronized(lock) { ingest.lastKnownLeapSeconds }

    internal fun getKalmanCoastForTest(): KalmanCoast? = synchronized(lock) { interEpochKalman.coastReport() }

    /** SSR (State Space Representation) corrections from IGS RTS NTRIP stream. */
    val ssrStore = SsrCorrections.Store()

    /** RTK reference-station observations + position from an OSR (MSM) NTRIP stream. */
    val rtkBaseStore = RtkBaseStore()

    private val pppState = PppState()
    private val interEpochKalman = GnssInterEpochKalmanFilter()

    private val ephemerisStore = GnssEphemerisStore()
    private val ingest = GnssNavMessageIngest(ephemerisStore, suppressSbas)
    private val ssr = SsrCorrectionPipeline(ssrStore)
    private val tdcp = TdcpMemory(ephemerisStore, interEpochKalman)
    private val rtk = GnssRtkSolver(ephemerisStore, ingest, pppState, interEpochKalman, tdcp, rtkBaseStore)
    private val absSolver = GnssAbsoluteSolver(ephemerisStore, ingest, ssr, pppState, interEpochKalman, tdcp)
    private var smoothingMode: SmoothingMode = SmoothingMode.CARRIER_HATCH

    fun setSmoothingMode(mode: SmoothingMode) =
        synchronized(lock) {
            if (smoothingMode == mode) return@synchronized
            smoothingMode = mode
            hatchFilter.reset()
            dopplerSmoother.reset()
            interEpochKalman.reset()
        }

    internal fun hasActivePppOrSsrState(gpsTimeNanos: Long = 0L): Boolean =
        synchronized(lock) {
            pppState.getLastPositionEcef(currentTimeNanos = gpsTimeNanos) != null ||
                ssrStore.hasValidCorrections
        }

    internal fun resetSession(
        preserveEphemeris: Boolean = true,
        clearCorrections: Boolean = true,
    ) = synchronized(lock) {
        pppState.reset()
        interEpochKalman.reset()
        hatchFilter.reset()
        dopplerSmoother.reset()
        tdcp.clear()
        rtk.rtkAmbiguityMemory.clear()
        lastFullBiasNanos = null
        lastBiasNanos = 0.0
        tdcp.epochClockStepMeters = 0.0
        lastHardwareClockDiscontinuityCount = null

        if (clearCorrections) {
            ssrStore.clear()
            rtk.baseStore.clear()
            ingest.clearSbasCorrections()
        }
        if (!preserveEphemeris) {
            ephemerisStore.clearEphemerides()
            ingest.clearNavigationState()
        }
    }

    /**
     * Seed position for the next epoch, plus the ambiguity and zenith-wet-delay holders a
     * sequential PPP filter would carry. The single-epoch solve does not estimate either
     * (review #85): a phase row with its own free ambiguity adds nothing, and a free wet
     * delay then trades off against height and clock.
     */
    internal class PppState {
        /** Float ambiguity state (meters), keyed by constellation + SVID; not integer-resolved. */
        val carrierAmbiguities = mutableMapOf<Pair<Int, Int>, Double>()
        var zenithWetDelay: Double = 0.1
        var lastX: Double? = null
        var lastY: Double? = null
        var lastZ: Double? = null
        var lastTimeNanos: Long = 0

        fun hasLastPosition(): Boolean = lastX != null

        fun getLastPositionEcef(
            currentTimeNanos: Long = 0L,
            maxAgeNanos: Long = PPP_SEED_MAX_AGE_NANOS,
        ): Triple<Double, Double, Double>? {
            val x = lastX ?: return null
            val y = lastY ?: return null
            val z = lastZ ?: return null
            if (currentTimeNanos > 0L &&
                lastTimeNanos > 0L &&
                (currentTimeNanos < lastTimeNanos || currentTimeNanos - lastTimeNanos > maxAgeNanos)
            ) {
                return null
            }
            return Triple(x, y, z)
        }

        fun updatePosition(
            x: Double,
            y: Double,
            z: Double,
            timeNanos: Long,
        ) {
            lastX = x
            lastY = y
            lastZ = z
            lastTimeNanos = timeNanos
        }

        fun resetAmbiguity(satKey: Pair<Int, Int>) {
            carrierAmbiguities.remove(satKey)
        }

        fun reset() {
            carrierAmbiguities.clear()
            zenithWetDelay = 0.1
            lastX = null
            lastY = null
            lastZ = null
            lastTimeNanos = 0
        }
    }

    /**
     * Process a GNSS measurements event and extract high-quality satellite data.
     *
     * @param selectedConstellationTypes Android `GnssStatus.CONSTELLATION_*` integers that
     *   the caller wants to include. Measurements whose `constellationType` is not in this
     *   set are dropped before any filtering / solving — saving work and preventing the
     *   disabled constellations from influencing the position, the satellite counts or the
     *   diagnostics. When `null`, no constellation-level filtering is applied (useful for
     *   tests and the legacy "process everything" behaviour).
     */
    fun processMeasurements(
        event: GnssMeasurementsEvent,
        gnssStatus: GnssStatus?,
        selectedConstellationTypes: Set<Int>? = null,
    ): ProcessedMeasurements =
        synchronized(lock) {
            val processedMeasurements = mutableListOf<SatelliteMeasurement>()
            val measurements = event.measurements
            val clock = event.clock

            // A hardware clock discontinuity means the receiver's timing solution jumped — a rebase,
            // a suspend, a leap-second step. The previous epoch's correction no longer describes
            // anything, so any carried smoother state is built on a difference that is not the one
            // being measured, and every signal would have to detect its own slip to escape it. Reset
            // once, here, on the platform's own signal, rather than letting each filter discover it.
            // The count is compared rather than its value used: the platform gives the number of
            // discontinuities since boot and has no sentinel for "none", so a change is the only
            // evidence available.
            if (lastHardwareClockDiscontinuityCount != null &&
                clock.hardwareClockDiscontinuityCount != lastHardwareClockDiscontinuityCount
            ) {
                hatchFilter.reset()
                dopplerSmoother.reset()
                lastFullBiasNanos = null
                tdcp.clear()
                rtk.rtkAmbiguityMemory.clear()
            }
            lastHardwareClockDiscontinuityCount = clock.hardwareClockDiscontinuityCount

            // One step per epoch. fullBias is differenced in Long; adding it to biasNanos in Double
            // first would quantize the step onto a 256 ns grid before the smoothers ever see it.
            val hasFullBias = Build.VERSION.SDK_INT >= Build.VERSION_CODES.N && clock.hasFullBiasNanos()
            val biasNanos = if (hasFullBias && clock.hasBiasNanos()) clock.biasNanos else 0.0
            val clockStepMeters =
                if (hasFullBias) {
                    receiverClockStepMeters(lastFullBiasNanos, clock.fullBiasNanos, lastBiasNanos, biasNanos)
                } else {
                    0.0
                }
            tdcp.epochClockStepMeters = clockStepMeters
            if (hasFullBias) {
                lastFullBiasNanos = clock.fullBiasNanos
                lastBiasNanos = biasNanos
            } else {
                lastFullBiasNanos = null
                lastBiasNanos = 0.0
            }

            hatchFilter.beginNewMeasurementsEpoch()
            dopplerSmoother.beginNewMeasurementsEpoch()

            // GPS−UTC leap seconds: prefer the receiver-reported value (survives future leap
            // events); retain the last reported value across epochs where hasLeapSecond() is false.
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N && clock.hasLeapSecond() && clock.leapSecond > 0) {
                ingest.lastKnownLeapSeconds = clock.leapSecond
            }
            val epochLeapSeconds = ingest.lastKnownLeapSeconds.toDouble()

            // Compute GPS time from GnssClock fields (nanoseconds since GPS epoch)
            val gpsTimeNanos =
                if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N && clock.hasFullBiasNanos()) {
                    clock.timeNanos - clock.fullBiasNanos - (if (clock.hasBiasNanos()) clock.biasNanos.toLong() else 0L)
                } else {
                    // Fallback: approximate from system clock (UTC − GPS epoch + leap seconds)
                    ingest.currentSystemGpsTimeNanos(ingest.lastKnownLeapSeconds)
                }

            // Cache GPS TOW for GLONASS tb conversion in processNavigationMessage
            ingest.lastGpsTimeOfWeekSeconds = (gpsTimeNanos / 1e9).mod(GnssMathUtils.SECONDS_IN_WEEK)
            ingest.hasGpsTimeOfWeek = true

            val satelliteInfo = mutableMapOf<Pair<Int, Int>, Pair<Double, Double>>()
            gnssStatus?.let { status ->
                for (i in 0 until status.satelliteCount) {
                    val constellationType = status.getConstellationType(i)
                    val svid = status.getSvid(i)
                    val elevation = status.getElevationDegrees(i).toDouble()
                    val azimuth = status.getAzimuthDegrees(i).toDouble()
                    satelliteInfo[constellationType to svid] = elevation to azimuth
                }
            }

            for (measurement in measurements) {
                val constellationType = measurement.constellationType
                // Drop measurements from constellations the user has disabled in settings.
                if (selectedConstellationTypes != null &&
                    constellationType !in selectedConstellationTypes
                ) {
                    continue
                }
                // On dual-frequency (L1 + L5/E5) devices, SBAS is bypassed — its GEO ranging
                // signals offer no accuracy benefit over the L1/L5 iono-free combination and
                // processing them wastes battery.
                if (suppressSbas && constellationType == GnssStatus.CONSTELLATION_SBAS) {
                    continue
                }
                val svid = measurement.svid
                val satKey = constellationType to svid

                val statusAngles = satelliteInfo[satKey]
                val hasStatusElevation = statusAngles != null
                val (elevationRaw, azimuthRaw) = statusAngles ?: (0.0 to 0.0)
                // GnssStatus often lags the first measurement epochs; 0° elevation would drop all SVs.
                val elevationDegrees =
                    if (hasStatusElevation) {
                        elevationRaw
                    } else {
                        PLACEHOLDER_ELEVATION_WHEN_STATUS_UNKNOWN_DEG
                    }
                val azimuthDegrees = if (hasStatusElevation) azimuthRaw else 0.0

                // Lowered thresholds for better urban performance (weighting handles quality)
                if (measurement.cn0DbHz < 25.0) continue
                if (hasStatusElevation && elevationRaw < 15.0) continue

                val effectiveFreqHz =
                    if (!measurement.hasCarrierFrequencyHz() ||
                        !measurement.carrierFrequencyHz.isFinite() ||
                        measurement.carrierFrequencyHz <= 0f
                    ) {
                        when (constellationType) {
                            GnssStatus.CONSTELLATION_GLONASS -> GnssMathUtils.GLONASS_G1_NOMINAL_FREQUENCY_HZ
                            GnssStatus.CONSTELLATION_BEIDOU -> GnssMathUtils.B1I_FREQUENCY_HZ
                            else -> GnssMathUtils.L1_FREQUENCY_HZ
                        }
                    } else {
                        measurement.carrierFrequencyHz.toDouble()
                    }
                val signalBand = GnssSignalBand.fromFrequencyHz(effectiveFreqHz, constellationType)
                val isL5 = signalBand == GnssSignalBand.L5_E5A_B2A

                if (Build.VERSION.SDK_INT < Build.VERSION_CODES.N || !clock.hasFullBiasNanos()) continue

                // Keep the large part of the receive time in Long: timeNanos − fullBiasNanos
                // ≈ 1.4e18 ns exceeds Double's 2^53 integer range (ULP ≈ 256 ns ≈ 77 m), so a
                // Double sum quantizes per-measurement timeOffsetNanos differences onto that
                // grid. The week/day modulo is taken in Long inside the seam and only the
                // small fractional terms are applied in Double afterwards.
                val receiveTimeWholeNanos = clock.timeNanos - clock.fullBiasNanos
                val receiveTimeFractionNanos =
                    measurement.timeOffsetNanos - (if (clock.hasBiasNanos()) clock.biasNanos else 0.0)

                val codeSigmaMeters =
                    (measurement.receivedSvTimeUncertaintyNanos * (SPEED_OF_LIGHT / 1e9))
                        .coerceIn(0.0, GnssMathUtils.MAX_PSEUDORANGE_SIGMA_METERS)
                val pseudorangeMeters =
                    computeRawPseudorangeMeters(
                        constellationType = constellationType,
                        measurementState = measurement.state,
                        receiveTimeWholeGpstNanos = receiveTimeWholeNanos,
                        receiveTimeFractionNanos = receiveTimeFractionNanos,
                        receivedSvTimeNanos = measurement.receivedSvTimeNanos,
                        leapSeconds = epochLeapSeconds,
                    ) ?: continue

                val adrMeters = measurement.accumulatedDeltaRangeMeters
                val adrState = measurement.accumulatedDeltaRangeState
                // Smoother state is per *signal*, not per satellite: L1 and L5 of one SV have
                // unrelated ADR origins and pseudorange baselines, so sharing one filter entry
                // mixes the bands (perpetual resets or meter-level cross-band bias).
                val signalKey =
                    Triple(
                        constellationType,
                        svid,
                        (effectiveFreqHz / 1e6).toInt(),
                    )
                val smoothedPr =
                    when (smoothingMode) {
                        SmoothingMode.CARRIER_HATCH ->
                            hatchFilter.smooth(
                                signalKey = signalKey,
                                rawPseudorange = pseudorangeMeters,
                                currentAdr = adrMeters,
                                adrState = adrState,
                                // The platform's own range rate for this signal, in the same hardware
                                // clock domain as the carrier delta. The filter needs it to tell a
                                // cycle slip apart from a satellite simply flying past: a carrier
                                // delta of several hundred metres per second is ordinary, and the
                                // filter used to treat it as a slip.
                                epochTimeNanos = gpsTimeNanos,
                                pseudorangeRateMetersPerSecond = measurement.pseudorangeRateMetersPerSecond,
                                pseudorangeRateUncertaintyMetersPerSecond = measurement.pseudorangeRateUncertaintyMetersPerSecond,
                                // The pseudorange above is clock-corrected and the carrier is not, so
                                // the filter tracks fullBiasNanos per signal to subtract the exact
                                // cumulative clock step even across brief signal occlusions.
                                clockStepMeters = clockStepMeters,
                                codeSigmaMeters = codeSigmaMeters,
                                fullBiasNanos = if (hasFullBias) clock.fullBiasNanos else null,
                                biasNanos = biasNanos,
                            )
                        SmoothingMode.DOPPLER ->
                            dopplerSmoother.smooth(
                                signalKey = signalKey,
                                rawPseudorange = pseudorangeMeters,
                                pseudorangeRateMetersPerSecond = measurement.pseudorangeRateMetersPerSecond,
                                pseudorangeRateUncertaintyMetersPerSecond = measurement.pseudorangeRateUncertaintyMetersPerSecond,
                                epochTimeNanos = gpsTimeNanos,
                                // Same reason: track fullBiasNanos per signal so the prediction
                                // stays in the pseudorange's clock domain across brief occlusions.
                                clockStepMeters = clockStepMeters,
                                fullBiasNanos = if (hasFullBias) clock.fullBiasNanos else null,
                                biasNanos = biasNanos,
                            )
                    }
                val smoothingEpochs =
                    when (smoothingMode) {
                        SmoothingMode.CARRIER_HATCH -> hatchFilter.currentSmoothingCount(signalKey)
                        SmoothingMode.DOPPLER -> dopplerSmoother.currentSmoothingCount(signalKey).coerceAtMost(DOPPLER_EFFECTIVE_MAX_SMOOTHING_EPOCHS)
                    }

                val wavelength = SPEED_OF_LIGHT / effectiveFreqHz
                val carrierPhaseMeters = if (hasValidAdr(adrState)) adrMeters else null

                processedMeasurements.add(
                    SatelliteMeasurement(
                        constellationType = constellationType,
                        svid = svid,
                        pseudorangeMeters = smoothedPr,
                        rawPseudorangeMeters = pseudorangeMeters,
                        carrierFrequencyHz = effectiveFreqHz,
                        cn0DbHz = measurement.cn0DbHz,
                        elevationDegrees = elevationDegrees,
                        azimuthDegrees = azimuthDegrees,
                        accumulatedDeltaRangeMeters = measurement.accumulatedDeltaRangeMeters,
                        pseudorangeUncertaintyMeters = codeSigmaMeters,
                        isL5 = isL5,
                        constellationName = getConstellationName(constellationType),
                        wavelengthMeters = wavelength,
                        carrierPhaseMeters = carrierPhaseMeters,
                        adrState = adrState,
                        pseudorangeRateMetersPerSecond = measurement.pseudorangeRateMetersPerSecond,
                        pseudorangeRateUncertaintyMetersPerSecond = measurement.pseudorangeRateUncertaintyMetersPerSecond,
                        smoothingEpochs = smoothingEpochs,
                        signalBand = signalBand,
                    ),
                )

                // Critical PPP Fix: Handle Cycle Slips
                if ((adrState and GnssMeasurement.ADR_STATE_CYCLE_SLIP) != 0 ||
                    (adrState and GnssMeasurement.ADR_STATE_RESET) != 0
                ) {
                    Log.d(TAG, "Cycle Slip/Reset detected for SVID: ${measurement.svid} (Constellation: $constellationType). Resetting ambiguity.")
                    pppState.resetAmbiguity(satKey)
                }
            }
            return ProcessedMeasurements(processedMeasurements, gpsTimeNanos)
        }

    /**
     * Forms the raw pseudorange from receiver and satellite times, or returns null when the
     * measurement is unusable.
     *
     * Decoded satellite time is required: without it `receivedSvTimeNanos` may contain only a
     * sub-millisecond code-phase offset, making the pseudorange wrong by hundreds of km.
     * GLONASS HALs flag decoded time via the GLO-specific STATE_GLO_TOD_DECODED/KNOWN bits —
     * the STATE_TOW_* bits are never set for GLONASS per the GnssMeasurement contract, so
     * requiring them unconditionally silently dropped the whole constellation.
     *
     * Timescales: `receivedSvTimeNanos` is GPST time-of-week for GPS/QZSS/Galileo/SBAS, BDT
     * time-of-week for BeiDou (GPST − 14 s), and **GLONASS time-of-day** (UTC(SU) + 3 h) for
     * GLONASS. The GPST receiver time is shifted into the satellite timescale before the
     * modulo difference — without the GLONASS shift the difference is off by 3 h − leap ≈
     * 10,782 s and every GLONASS pseudorange came out negative and was culled.
     *
     * Precision: [receiveTimeWholeGpstNanos] must be the exact Long `timeNanos − fullBiasNanos`
     * (≈1.4e18 ns exceeds Double's 2^53 integer range; a Double sum quantizes onto a 256 ns ≈
     * 77 m grid, turning per-measurement timeOffsetNanos differences into differential range
     * error). The week/day modulo is taken in Long; only the small [receiveTimeFractionNanos]
     * (timeOffsetNanos − biasNanos) is applied in Double on the sub-week remainder.
     */
    internal fun computeRawPseudorangeMeters(
        constellationType: Int,
        measurementState: Int,
        receiveTimeWholeGpstNanos: Long,
        receiveTimeFractionNanos: Double,
        receivedSvTimeNanos: Long,
        leapSeconds: Double,
    ): Double? {
        if ((measurementState and GnssMeasurement.STATE_MSEC_AMBIGUOUS) != 0) return null

        val isGlonass = constellationType == GnssStatus.CONSTELLATION_GLONASS
        val hasCodeLock =
            (
                measurementState and
                    (
                        GnssMeasurement.STATE_CODE_LOCK or
                            GnssMeasurement.STATE_2ND_CODE_LOCK or
                            GnssMeasurement.STATE_GAL_E1BC_CODE_LOCK or
                            GnssMeasurement.STATE_GAL_E1C_2ND_CODE_LOCK
                    )
            ) != 0

        val towDecoded = !isGlonass && (measurementState and GnssMeasurement.STATE_TOW_DECODED) != 0
        val towKnown = !isGlonass && (measurementState and GnssMeasurement.STATE_TOW_KNOWN) != 0 && hasCodeLock
        val gloTodDecoded =
            isGlonass &&
                (
                    (measurementState and GnssMeasurement.STATE_GLO_TOD_DECODED) != 0 ||
                        (measurementState and GnssMeasurement.STATE_GLO_TOD_KNOWN) != 0
                )
        if (!towDecoded && !towKnown && !gloTodDecoded) return null

        val unitNanos = if (isGlonass) GnssMathUtils.NANOS_PER_DAY else GnssMathUtils.NANOS_PER_WEEK

        // Exact sub-unit remainder in Long (< 6.05e14 — exactly representable in Double),
        // then the small Double terms.
        var wholeModuloNanos = receiveTimeWholeGpstNanos % unitNanos
        if (wholeModuloNanos < 0) wholeModuloNanos += unitNanos
        val glonassShiftNanos =
            if (isGlonass) (GnssMathUtils.GLONASS_UTC_OFFSET_SECONDS - leapSeconds) * 1e9 else 0.0
        val receiverTimeInSvTimescaleNanos = wholeModuloNanos.toDouble() + receiveTimeFractionNanos + glonassShiftNanos

        // For BeiDou, Android delivers `receivedSvTimeNanos` in the BDT time system, which
        // lags GPS by 14 seconds; convert BDT → GPST by adding 14 s to the transmit time,
        // otherwise every BeiDou pseudorange is biased by 14 s × c ≈ 4.2e9 m.
        val bdtToGpstOffsetNanos =
            if (constellationType == GnssStatus.CONSTELLATION_BEIDOU) {
                GnssMathUtils.BDT_GPST_OFFSET_SECONDS * 1e9
            } else {
                0.0
            }
        val transmitTimeNanos = receivedSvTimeNanos.toDouble() + bdtToGpstOffsetNanos

        var prNanos = receiverTimeInSvTimescaleNanos - transmitTimeNanos
        if (prNanos < 0) prNanos += unitNanos
        if (prNanos > unitNanos / 2.0) prNanos -= unitNanos

        val pseudorangeMeters = (prNanos / 1e9) * SPEED_OF_LIGHT

        // Sanity check: valid pseudoranges span ~15,000 km (low MEO) to ~42,000 km
        // (SBAS/QZSS GEO at low elevation); anything outside indicates a decoding error.
        if (pseudorangeMeters <= 0.0 || pseudorangeMeters.isNaN() ||
            pseudorangeMeters < MIN_REASONABLE_PSEUDORANGE_METERS ||
            pseudorangeMeters > MAX_REASONABLE_PSEUDORANGE_METERS
        ) {
            return null
        }
        return pseudorangeMeters
    }

    fun computePosition(
        measurements: List<SatelliteMeasurement>,
        initialLat: Double = 0.0,
        initialLon: Double = 0.0,
        initialAlt: Double = 0.0,
        usePpp: Boolean = true,
        gpsTimeNanos: Long = 0L,
        accuracy: Accuracy = Accuracy.HIGH,
        commitState: Boolean = true,
    ): PositionResult? = synchronized(lock) { absSolver.solve(measurements, initialLat, initialLon, initialAlt, usePpp, gpsTimeNanos, accuracy, commitState) }

    fun computeRtkPosition(
        measurements: List<SatelliteMeasurement>,
        gpsTimeNanos: Long,
        initialLat: Double = 0.0,
        initialLon: Double = 0.0,
        initialAlt: Double = 0.0,
    ): PositionResult? = synchronized(lock) { rtk.solve(measurements, gpsTimeNanos, initialLat, initialLon, initialAlt) }

    fun processNavigationMessage(message: GnssNavigationMessage) = synchronized(lock) { ingest.processNavigationMessage(message) }

    fun importExternalEphemeris(
        records: List<SatelliteEphemeris>,
        gpsTimeOfWeekSeconds: Double,
    ): Int = synchronized(lock) { ephemerisStore.importExternalEphemeris(records, gpsTimeOfWeekSeconds) }

    internal fun describeFreshEphemerisCoverage(measurements: List<SatelliteMeasurement>): String = synchronized(lock) { ephemerisStore.describeFreshEphemerisCoverage(measurements) }

    internal fun describeStoredEphemerides(): String = synchronized(lock) { ephemerisStore.describeStoredEphemerides() }

    internal fun decodeGpsL1CaSubframe(
        satKey: Pair<Int, Int>,
        data: ByteArray,
        parityAttested: Boolean,
    ): ByteArray? = synchronized(lock) { ingest.decodeGpsL1CaSubframe(satKey, data, parityAttested) }

    internal fun recordGpsCnavMt30Isc(
        svid: Int,
        data: ByteArray,
    ) = synchronized(lock) { ingest.recordGpsCnavMt30Isc(svid, data) }

    internal fun codePseudorangesForIonosphereFreeMeters(
        satKey: Pair<Int, Int>,
        l1Meters: Double,
        l5Meters: Double,
        ephemeris: SatelliteEphemeris?,
        upperBandHz: Double =
            if (satKey.first == GnssStatus.CONSTELLATION_BEIDOU) {
                GnssMathUtils.B1I_FREQUENCY_HZ
            } else {
                GnssMathUtils.L1_FREQUENCY_HZ
            },
        lowerBandHz: Double = GnssMathUtils.L5_FREQUENCY_HZ,
    ): GnssNavMessageIngest.IfCodeAdjustment = synchronized(lock) { ingest.codePseudorangesForIonosphereFreeMeters(satKey, l1Meters, l5Meters, ephemeris, upperBandHz, lowerBandHz) }

    internal fun getSbasLongTermCorrection(
        svid: Int,
        ephemerisIode: Int,
    ): SbasNavMessageParser.SbasLongTermCorrection? = synchronized(lock) { ingest.getSbasLongTermCorrection(svid, ephemerisIode) }

    internal fun groupDelaySecondsForMeasurement(
        ephemeris: SatelliteEphemeris,
        constellationType: Int,
        carrierFrequencyHz: Double,
        ionosphereFree: Boolean,
        clockExcludesGroupDelay: Boolean,
        lowerBandHz: Double = GnssMathUtils.L5_FREQUENCY_HZ,
    ): Double = measurementGroupDelaySeconds(ephemeris, constellationType, carrierFrequencyHz, ionosphereFree, clockExcludesGroupDelay, lowerBandHz)

    internal fun galileoGroupDelaySeconds(
        ephemeris: SatelliteEphemeris,
        carrierFrequencyHz: Double,
        ionosphereFree: Boolean,
        lowerBandHz: Double = GnssMathUtils.L5_FREQUENCY_HZ,
    ): Double = galileoSignalGroupDelaySeconds(ephemeris, carrierFrequencyHz, ionosphereFree, lowerBandHz)

    internal fun getConstellationForNavigationMessageType(type: Int): Int = navigationMessageConstellationType(type)

    internal fun isCarrierHalfCycleAmbiguous(adrState: Int): Boolean = app.simt.gnss.isCarrierHalfCycleAmbiguous(adrState)

    companion object {
        private const val TAG = "GnssMeasurementProcessor"
        private const val DOPPLER_EFFECTIVE_MAX_SMOOTHING_EPOCHS = 9

        /**
         * When [android.location.GnssStatus] has not yet reported this SV, elevation defaults to 0° in the
         * Android API. Using a modest placeholder keeps the SV in the solution while C/N0 and WLS geometry
         * (recomputed each iteration) dominate; iono uses this azimuth/elev until status arrives.
         */
        private const val PLACEHOLDER_ELEVATION_WHEN_STATUS_UNKNOWN_DEG = 20.0

        /** Minimum reasonable pseudorange in meters (~15,000 km — low-elevation GEO/MEO). */
        private const val MIN_REASONABLE_PSEUDORANGE_METERS = 15_000_000.0

        /** Maximum reasonable pseudorange in meters (~42,000 km — SBAS/QZSS GEO at low elevation). */
        private const val MAX_REASONABLE_PSEUDORANGE_METERS = 42_000_000.0

        private const val PPP_SEED_MAX_AGE_NANOS = 30_000_000_000L
        private const val MAX_PPP_SEED_DEVIATION_METERS = 50_000.0
        private const val MIN_IF_FREQUENCY_SEPARATION_HZ = 250_000_000.0
    }
}
