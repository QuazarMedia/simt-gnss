package app.simt.gnss

import android.location.GnssStatus
import android.util.Log
import app.simt.gnss.core.correction.AtmosphericCorrection
import app.simt.gnss.core.correction.RtkBaseStore
import app.simt.gnss.core.filter.GnssInterEpochKalmanFilter
import app.simt.gnss.core.model.GnssSolutionMode
import app.simt.gnss.core.model.PositionResult
import app.simt.gnss.core.model.SatelliteEphemeris
import app.simt.gnss.core.model.SatelliteMeasurement
import app.simt.gnss.core.orbit.SatellitePositionCalculator
import app.simt.gnss.core.orbit.TransmitTiming
import app.simt.gnss.core.parser.RtcmMsmParser
import app.simt.gnss.core.solver.RtkAmbiguityMemory
import app.simt.gnss.core.solver.RtkDoubleDifferenceSolver
import app.simt.gnss.core.utils.GnssMathUtils
import app.simt.gnss.core.utils.GnssMathUtils.SPEED_OF_LIGHT
import kotlin.math.*

/**
 * RTK double-difference solver: matches rover measurements against the base-station
 * observations held in the supplied [RtkBaseStore], forms code/carrier
 * double-differences and solves for the rover position, with ambiguity memory and
 * accuracy estimation.
 */
internal class GnssRtkSolver(
    private val ephemerisStore: GnssEphemerisStore,
    private val ingest: GnssNavMessageIngest,
    private val pppState: GnssMeasurementProcessor.PppState,
    private val interEpochKalman: GnssInterEpochKalmanFilter,
    private val tdcp: TdcpMemory,
    internal val baseStore: RtkBaseStore,
) {
    internal val rtkAmbiguityMemory = RtkAmbiguityMemory()

    /**
     * Transmit timing for an RTK observation. There is no receiver-clock estimate here, so the
     * rotation interval is PR/c + Δt_sv. The same Δt_sv is removed from the transmit epoch.
     */
    internal data class RtkTiming(
        val timing: TransmitTiming,
        val satelliteClockSeconds: Double,
    )

    internal fun rtkTransmitTiming(
        ephemeris: SatelliteEphemeris,
        receptionTimeSeconds: Double,
        pseudorangeMeters: Double,
    ): RtkTiming {
        val approxFlight =
            (pseudorangeMeters / SPEED_OF_LIGHT).coerceIn(MIN_TRAVEL_TIME_SECONDS, MAX_TRAVEL_TIME_SECONDS)
        val satelliteClockSeconds =
            SatellitePositionCalculator.getSatelliteClockCorrection(
                ephemeris,
                receptionTimeSeconds - approxFlight,
                includeGroupDelay = false,
            )
        return RtkTiming(
            timing =
                SatellitePositionCalculator.transmitTiming(
                    receptionTimeSeconds = receptionTimeSeconds,
                    pseudorangeMeters = pseudorangeMeters,
                    receiverClockBiasMeters = 0.0,
                    satelliteClockSeconds = satelliteClockSeconds,
                ),
            satelliteClockSeconds = satelliteClockSeconds,
        )
    }

    /**
     * RTK double-differenced position from base-station OSR observations held in [baseStore].
     *
     * For each rover measurement that has a fresh matching base observation and a usable broadcast
     * ephemeris, the satellite ECEF position is computed and a double-difference observation is formed
     * (see [RtkDoubleDifferenceSolver]). Differencing cancels satellite clock/orbit, receiver clocks,
     * and short-baseline atmosphere, yielding a decimeter-class float fix relative to the known base.
     * Returns null when there is no fresh base data, too few matched satellites, or the solve fails —
     * callers should then fall back to [computePosition].
     */
    internal data class RtkMatchedObservation(
        val meas: SatelliteMeasurement,
        val baseObs: RtcmMsmParser.BaseObservation,
        val satEcef: DoubleArray,
        val baseSatEcef: DoubleArray,
        val roverClockSeconds: Double,
        val baseClockSeconds: Double,
        val baseTropoMeters: Double,
        val baseIonoL1Meters: Double,
        val baseElevationDeg: Double,
        val initialRoverElevationDeg: Double,
        val initialRoverAzimuthDeg: Double,
    )

    internal fun rtkCarrierSamples(matched: List<RtkMatchedObservation>): List<RtkAmbiguityMemory.Sample> =
        matched.map { entry ->
            val meas = entry.meas
            RtkAmbiguityMemory.Sample(
                constellationType = meas.constellationType,
                svid = meas.svid,
                adrMeters = meas.accumulatedDeltaRangeMeters,
                adrState = meas.adrState,
                rateMetersPerSecond = meas.pseudorangeRateMetersPerSecond,
                rateUncertaintyMetersPerSecond = meas.pseudorangeRateUncertaintyMetersPerSecond,
                halfCycleAmbiguous = isCarrierHalfCycleAmbiguous(meas.adrState),
                bandMhz = (meas.carrierFrequencyHz / 1e6).roundToInt(),
                baseLockTimeMillis = entry.baseObs.lockTimeMillis,
            )
        }

    internal fun ecefDistanceMeters(
        a: DoubleArray,
        b: DoubleArray,
    ): Double {
        val dx = a[0] - b[0]
        val dy = a[1] - b[1]
        val dz = a[2] - b[2]
        return sqrt(dx * dx + dy * dy + dz * dz)
    }

    internal fun buildRtkCodeInputs(
        matched: List<RtkMatchedObservation>,
        roverEcef: DoubleArray,
        roverLat: Double,
        roverLon: Double,
        roverAlt: Double,
        dayOfYear: Int,
        gpsTimeMs: Long,
        applyIono: Boolean,
    ): List<RtkDoubleDifferenceSolver.SatelliteInput> =
        matched.map { entry ->
            val meas = entry.meas
            val (roverEl, roverAz) =
                GnssMathUtils.ecefLosToElevationAzimuth(
                    receiverX = roverEcef[0],
                    receiverY = roverEcef[1],
                    receiverZ = roverEcef[2],
                    receiverLatDeg = roverLat,
                    receiverLonDeg = roverLon,
                    satelliteX = entry.satEcef[0],
                    satelliteY = entry.satEcef[1],
                    satelliteZ = entry.satEcef[2],
                )
            val roverTropo = AtmosphericCorrection.computeTroposphericDelay(roverEl, roverAlt, roverLat, dayOfYear)
            val freqScale =
                if (meas.carrierFrequencyHz > 0.0) {
                    (GnssMathUtils.L1_FREQUENCY_HZ / meas.carrierFrequencyHz).pow(2)
                } else {
                    1.0
                }
            val roverIono =
                if (applyIono) {
                    AtmosphericCorrection.computeIonosphericDelay(
                        roverEl,
                        roverAz,
                        roverLat,
                        roverLon,
                        gpsTimeMs,
                        ingest.ionoAlpha,
                        ingest.ionoBeta,
                    ) * freqScale
                } else {
                    0.0
                }
            val baseIono = if (applyIono) entry.baseIonoL1Meters * freqScale else 0.0
            val codeSigma =
                GnssMathUtils
                    .pseudorangeSigmaFromElevationAndCn0(roverEl, meas.cn0DbHz)
                    .coerceIn(GnssMathUtils.MIN_PSEUDORANGE_SIGMA_METERS, GnssMathUtils.MAX_PSEUDORANGE_SIGMA_METERS)
            RtkDoubleDifferenceSolver.SatelliteInput(
                satelliteEcef = entry.satEcef,
                baseSatelliteEcef = entry.baseSatEcef,
                roverObservationMeters =
                    RtkDoubleDifferenceSolver.withoutSatelliteClock(
                        meas.rawPseudorangeMeters,
                        entry.roverClockSeconds,
                    ) - roverTropo - roverIono,
                baseObservationMeters =
                    RtkDoubleDifferenceSolver.withoutSatelliteClock(
                        entry.baseObs.pseudorangeMeters,
                        entry.baseClockSeconds,
                    ) - entry.baseTropoMeters - baseIono,
                elevationDeg = roverEl,
                sigmaMeters = codeSigma,
                constellationType = meas.constellationType,
                bandMhz = (meas.carrierFrequencyHz / 1e6).roundToInt(),
            )
        }

    internal fun buildRtkCarrierInputs(
        matched: List<RtkMatchedObservation>,
        roverEcef: DoubleArray,
        roverLat: Double,
        roverLon: Double,
        roverAlt: Double,
        dayOfYear: Int,
        gpsTimeMs: Long,
        applyIono: Boolean,
    ): List<RtkDoubleDifferenceSolver.CarrierSatelliteInput> {
        val inputs = ArrayList<RtkDoubleDifferenceSolver.CarrierSatelliteInput>(matched.size)
        for (entry in matched) {
            val meas = entry.meas
            val roverCarrier = meas.carrierPhaseMeters ?: continue
            val baseCarrier = entry.baseObs.carrierPhaseMeters ?: continue
            if (meas.wavelengthMeters <= 0.0 || isCarrierHalfCycleAmbiguous(meas.adrState)) continue

            val (roverEl, roverAz) =
                GnssMathUtils.ecefLosToElevationAzimuth(
                    receiverX = roverEcef[0],
                    receiverY = roverEcef[1],
                    receiverZ = roverEcef[2],
                    receiverLatDeg = roverLat,
                    receiverLonDeg = roverLon,
                    satelliteX = entry.satEcef[0],
                    satelliteY = entry.satEcef[1],
                    satelliteZ = entry.satEcef[2],
                )
            val roverTropo = AtmosphericCorrection.computeTroposphericDelay(roverEl, roverAlt, roverLat, dayOfYear)
            val freqScale =
                if (meas.carrierFrequencyHz > 0.0) {
                    (GnssMathUtils.L1_FREQUENCY_HZ / meas.carrierFrequencyHz).pow(2)
                } else {
                    1.0
                }
            val roverIono =
                if (applyIono) {
                    AtmosphericCorrection.computeIonosphericDelay(
                        roverEl,
                        roverAz,
                        roverLat,
                        roverLon,
                        gpsTimeMs,
                        ingest.ionoAlpha,
                        ingest.ionoBeta,
                    ) * freqScale
                } else {
                    0.0
                }
            val baseIono = if (applyIono) entry.baseIonoL1Meters * freqScale else 0.0
            val codeSigma =
                GnssMathUtils
                    .pseudorangeSigmaFromElevationAndCn0(roverEl, meas.cn0DbHz)
                    .coerceIn(GnssMathUtils.MIN_PSEUDORANGE_SIGMA_METERS, GnssMathUtils.MAX_PSEUDORANGE_SIGMA_METERS)
            val sinEl = sin(Math.toRadians(roverEl)).coerceAtLeast(0.05)
            val carrierSigma = (CARRIER_PHASE_BASE_SIGMA_METERS / sinEl).coerceIn(CARRIER_PHASE_BASE_SIGMA_METERS, 0.05)
            inputs.add(
                RtkDoubleDifferenceSolver.CarrierSatelliteInput(
                    satelliteEcef = entry.satEcef,
                    baseSatelliteEcef = entry.baseSatEcef,
                    roverCodeMeters =
                        RtkDoubleDifferenceSolver.withoutSatelliteClock(
                            meas.rawPseudorangeMeters,
                            entry.roverClockSeconds,
                        ) - roverTropo - roverIono,
                    baseCodeMeters =
                        RtkDoubleDifferenceSolver.withoutSatelliteClock(
                            entry.baseObs.pseudorangeMeters,
                            entry.baseClockSeconds,
                        ) - entry.baseTropoMeters - baseIono,
                    roverCarrierMeters =
                        RtkDoubleDifferenceSolver.withoutSatelliteClock(
                            roverCarrier,
                            entry.roverClockSeconds,
                        ) - roverTropo + roverIono,
                    baseCarrierMeters =
                        RtkDoubleDifferenceSolver.withoutSatelliteClock(
                            baseCarrier,
                            entry.baseClockSeconds,
                        ) - entry.baseTropoMeters + baseIono,
                    wavelengthMeters = meas.wavelengthMeters,
                    elevationDeg = roverEl,
                    codeSigmaMeters = codeSigma,
                    carrierSigmaMeters = carrierSigma,
                    constellationType = meas.constellationType,
                    svid = meas.svid,
                ),
            )
        }
        return inputs
    }

    internal fun solve(
        measurements: List<SatelliteMeasurement>,
        gpsTimeNanos: Long,
        initialLat: Double = 0.0,
        initialLon: Double = 0.0,
        initialAlt: Double = 0.0,
    ): PositionResult? {
        if (gpsTimeNanos <= 0L) return null
        val basePosition = baseStore.getBasePosition() ?: return null
        val baseEcef = doubleArrayOf(basePosition.xMeters, basePosition.yMeters, basePosition.zMeters)

        val pppLast =
            pppState.getLastPositionEcef(currentTimeNanos = gpsTimeNanos)?.takeIf { last ->
                if (initialLat != 0.0 || initialLon != 0.0) {
                    val (sx, sy, sz) = GnssMathUtils.latLonAltToEcef(initialLat, initialLon, initialAlt)
                    ecefDistanceMeters(doubleArrayOf(last.first, last.second, last.third), doubleArrayOf(sx, sy, sz)) <= 50_000.0
                } else {
                    true
                }
            }
        val hasSeed = pppLast != null || initialLat != 0.0 || initialLon != 0.0
        val seedRover =
            pppLast?.let { doubleArrayOf(it.first, it.second, it.third) }
                ?: if (initialLat != 0.0 || initialLon != 0.0) {
                    val (sx, sy, sz) = GnssMathUtils.latLonAltToEcef(initialLat, initialLon, initialAlt)
                    doubleArrayOf(sx, sy, sz)
                } else {
                    baseEcef
                }
        if (hasSeed && ecefDistanceMeters(baseEcef, seedRover) > GnssMathUtils.MAX_RTK_BASELINE_METERS) {
            Log.d(TAG, "RTK: seed baseline exceeds ${GnssMathUtils.MAX_RTK_BASELINE_METERS}m — falling back to absolute")
            return null
        }

        val (baseLat, baseLon, baseAlt) = GnssMathUtils.ecefToLatLonAlt(baseEcef[0], baseEcef[1], baseEcef[2])
        val (roverSeedLat, roverSeedLon, roverSeedAlt) =
            GnssMathUtils.ecefToLatLonAlt(seedRover[0], seedRover[1], seedRover[2])
        val gpsTimeMs = gpsTimeNanos / 1_000_000L
        val dayOfYear: Int =
            run {
                val unixSeconds = GnssMathUtils.GPS_EPOCH_UNIX_SECONDS + (gpsTimeNanos / 1_000_000_000L)
                val utcCalendar = java.util.Calendar.getInstance(java.util.TimeZone.getTimeZone("UTC"))
                utcCalendar.timeInMillis = unixSeconds * 1000L
                utcCalendar.get(java.util.Calendar.DAY_OF_YEAR)
            }

        val gpstSeconds = (gpsTimeNanos / 1e9) % GnssMathUtils.SECONDS_IN_WEEK
        val matched = ArrayList<RtkMatchedObservation>()
        val usedSatKeys = HashSet<Pair<Int, Int>>()
        val sortedMeasurements =
            measurements.sortedWith(
                compareBy<SatelliteMeasurement>(
                    { !effectiveSignalBand(it).isUpperBand },
                    { upperBandPreferenceRank(it.constellationType, effectiveSignalBand(it)) },
                    { -it.carrierFrequencyHz },
                ),
            )

        for (meas in sortedMeasurements) {
            val satKey = meas.constellationType to meas.svid
            if (satKey in usedSatKeys) continue // one observation per satellite
            val ephemeris = ephemerisStore.get(satKey)?.takeIf { ephemerisStore.isUsableEphemerisLocked(it, gpstSeconds) } ?: continue
            // GLONASS MSM carries a day/time-of-day epoch (not a GPS TOW), so its base epoch can't
            // be aligned to the rover here — skip it for RTK (it's the hardest constellation for
            // RTK regardless, and is already excluded first in the absolute fallback).
            if (meas.constellationType == GnssStatus.CONSTELLATION_GLONASS) continue
            val baseObs =
                baseStore.getObservation(meas.constellationType, meas.svid, meas.carrierFrequencyHz) ?: continue

            val currentTimeSeconds =
                if (meas.constellationType == GnssStatus.CONSTELLATION_BEIDOU) {
                    (gpstSeconds - GnssMathUtils.BDT_GPST_OFFSET_SECONDS).mod(GnssMathUtils.SECONDS_IN_WEEK)
                } else {
                    gpstSeconds
                }

            // Rover satellite position at the rover's transmit time. The receiver clock cancels
            // in t_rx − PR/c; Δt_sv still has to come off (review #88). No rover-clock
            // estimate exists on this path, so the rotation interval is PR/c + Δt_sv.
            val roverTiming = rtkTransmitTiming(ephemeris, currentTimeSeconds, meas.pseudorangeMeters)
            val (tx, ty, tz) =
                SatellitePositionCalculator.calculateSatellitePosition(ephemeris, roverTiming.timing.transmitTimeSeconds)
            val satEcef =
                SatellitePositionCalculator.rotateEcefForEarthRotationDuringTransit(
                    tx,
                    ty,
                    tz,
                    roverTiming.timing.rotationSeconds.coerceIn(MIN_TRAVEL_TIME_SECONDS, MAX_TRAVEL_TIME_SECONDS),
                ).let { doubleArrayOf(it.first, it.second, it.third) }

            // Base satellite position at the BASE observation's epoch. The base stream lags the
            // rover by network latency; over that interval the satellite moves ~km along the line
            // of sight, so reusing the rover-epoch position for the base range would corrupt the
            // double difference (kilometre-level residuals). The MSM epoch is already in the
            // constellation's timescale (BDT for BeiDou, GPST otherwise), matching currentTimeSeconds.
            val baseEpochSeconds = baseObs.epochTimeMs / 1000.0
            if (abs(GnssMathUtils.wrapToHalfWeekSeconds(baseEpochSeconds - currentTimeSeconds)) > MAX_BASE_ROVER_EPOCH_DIFF_SECONDS) {
                continue
            }
            val baseTravel = rtkTransmitTiming(ephemeris, baseEpochSeconds, baseObs.pseudorangeMeters)
            val (btx, bty, btz) =
                SatellitePositionCalculator.calculateSatellitePosition(ephemeris, baseTravel.timing.transmitTimeSeconds)
            val baseSatEcef =
                SatellitePositionCalculator.rotateEcefForEarthRotationDuringTransit(
                    btx,
                    bty,
                    btz,
                    baseTravel.timing.rotationSeconds.coerceIn(MIN_TRAVEL_TIME_SECONDS, MAX_TRAVEL_TIME_SECONDS),
                ).let { doubleArrayOf(it.first, it.second, it.third) }

            val (roverElevationDeg, roverAzimuthDeg) =
                GnssMathUtils.ecefLosToElevationAzimuth(
                    receiverX = seedRover[0],
                    receiverY = seedRover[1],
                    receiverZ = seedRover[2],
                    receiverLatDeg = roverSeedLat,
                    receiverLonDeg = roverSeedLon,
                    satelliteX = satEcef[0],
                    satelliteY = satEcef[1],
                    satelliteZ = satEcef[2],
                )
            val (baseElevationDeg, baseAzimuthDeg) =
                GnssMathUtils.ecefLosToElevationAzimuth(
                    receiverX = baseEcef[0],
                    receiverY = baseEcef[1],
                    receiverZ = baseEcef[2],
                    receiverLatDeg = baseLat,
                    receiverLonDeg = baseLon,
                    satelliteX = baseSatEcef[0],
                    satelliteY = baseSatEcef[1],
                    satelliteZ = baseSatEcef[2],
                )
            if (roverElevationDeg < RTK_MIN_ELEVATION_DEG || baseElevationDeg < RTK_MIN_ELEVATION_DEG) continue

            val baseTropoMeters =
                AtmosphericCorrection.computeTroposphericDelay(baseElevationDeg, baseAlt, baseLat, dayOfYear)
            val baseIonoL1Meters =
                AtmosphericCorrection.computeIonosphericDelay(
                    baseElevationDeg,
                    baseAzimuthDeg,
                    baseLat,
                    baseLon,
                    gpsTimeMs,
                    ingest.ionoAlpha,
                    ingest.ionoBeta,
                )

            matched.add(
                RtkMatchedObservation(
                    meas = meas,
                    baseObs = baseObs,
                    satEcef = satEcef,
                    baseSatEcef = baseSatEcef,
                    roverClockSeconds = roverTiming.satelliteClockSeconds,
                    baseClockSeconds = baseTravel.satelliteClockSeconds,
                    baseTropoMeters = baseTropoMeters,
                    baseIonoL1Meters = baseIonoL1Meters,
                    baseElevationDeg = baseElevationDeg,
                    initialRoverElevationDeg = roverElevationDeg,
                    initialRoverAzimuthDeg = roverAzimuthDeg,
                ),
            )
            usedSatKeys.add(satKey)
        }

        if (matched.size < RtkDoubleDifferenceSolver.MIN_SATELLITES) {
            Log.d(TAG, "RTK: only ${matched.size} matched base+ephemeris satellites — cannot form double differences")
            return null
        }

        val seedBaselineMeters = ecefDistanceMeters(baseEcef, seedRover)
        val applyIonoInitial = hasSeed && seedBaselineMeters > RTK_IONO_BASELINE_THRESHOLD_METERS
        var codeInputs =
            buildRtkCodeInputs(
                matched = matched,
                roverEcef = seedRover,
                roverLat = roverSeedLat,
                roverLon = roverSeedLon,
                roverAlt = roverSeedAlt,
                dayOfYear = dayOfYear,
                gpsTimeMs = gpsTimeMs,
                applyIono = applyIonoInitial,
            )

        // Always compute the robust code double-difference float solution (decimeter-class).
        var floatSolution = RtkDoubleDifferenceSolver.solve(baseEcef, codeInputs, seedRover) ?: return null
        if (ecefDistanceMeters(baseEcef, floatSolution.roverEcef) > GnssMathUtils.MAX_RTK_BASELINE_METERS) {
            Log.d(TAG, "RTK: solved baseline exceeds ${GnssMathUtils.MAX_RTK_BASELINE_METERS}m — falling back to absolute")
            return null
        }

        // Sanity guard: a healthy code DD has sub-metre-to-few-metre residuals. If the post-fit
        // rms is implausibly large after outlier rejection — e.g. a bad reference satellite that
        // corrupts every double difference and cannot be excluded — the DD is untrustworthy this
        // epoch, so abandon RTK and let the caller fall back to the absolute fix rather than
        // returning a garbage position.
        if (floatSolution.residualRms > RTK_MAX_ACCEPTABLE_FLOAT_RMS_METERS) {
            Log.d(TAG, "RTK float rms=${"%.1f".format(floatSolution.residualRms)}m exceeds sanity limit — falling back to absolute")
            return null
        }

        var (floatLat, floatLon, floatAlt) =
            GnssMathUtils.ecefToLatLonAlt(
                floatSolution.roverEcef[0],
                floatSolution.roverEcef[1],
                floatSolution.roverEcef[2],
            )
        var floatBaselineMeters = ecefDistanceMeters(baseEcef, floatSolution.roverEcef)
        var applyIonoRefined = floatBaselineMeters > RTK_IONO_BASELINE_THRESHOLD_METERS
        if (abs(floatAlt - roverSeedAlt) > 5.0 || applyIonoRefined != applyIonoInitial) {
            codeInputs =
                buildRtkCodeInputs(
                    matched = matched,
                    roverEcef = floatSolution.roverEcef,
                    roverLat = floatLat,
                    roverLon = floatLon,
                    roverAlt = floatAlt,
                    dayOfYear = dayOfYear,
                    gpsTimeMs = gpsTimeMs,
                    applyIono = applyIonoRefined,
                )
            floatSolution = RtkDoubleDifferenceSolver.solve(baseEcef, codeInputs, floatSolution.roverEcef) ?: return null
            if (ecefDistanceMeters(baseEcef, floatSolution.roverEcef) > GnssMathUtils.MAX_RTK_BASELINE_METERS) {
                Log.d(TAG, "RTK: refined baseline exceeds ${GnssMathUtils.MAX_RTK_BASELINE_METERS}m — falling back to absolute")
                return null
            }
            if (floatSolution.residualRms > RTK_MAX_ACCEPTABLE_FLOAT_RMS_METERS) {
                Log.d(TAG, "RTK float rms=${"%.1f".format(floatSolution.residualRms)}m exceeds sanity limit — falling back to absolute")
                return null
            }
            val refinedLla =
                GnssMathUtils.ecefToLatLonAlt(
                    floatSolution.roverEcef[0],
                    floatSolution.roverEcef[1],
                    floatSolution.roverEcef[2],
                )
            floatLat = refinedLla.first
            floatLon = refinedLla.second
            floatAlt = refinedLla.third
            floatBaselineMeters = ecefDistanceMeters(baseEcef, floatSolution.roverEcef)
            applyIonoRefined = floatBaselineMeters > RTK_IONO_BASELINE_THRESHOLD_METERS
        }

        val carrierInputs =
            buildRtkCarrierInputs(
                matched = matched,
                roverEcef = floatSolution.roverEcef,
                roverLat = floatLat,
                roverLon = floatLon,
                roverAlt = floatAlt,
                dayOfYear = dayOfYear,
                gpsTimeMs = gpsTimeMs,
                applyIono = applyIonoRefined,
            )

        // Then attempt to upgrade to an integer-fixed (centimeter) solution. This is purely
        // additive: solveFixed returns null unless the LAMBDA ratio test passes AND the fixed
        // position agrees with the float, so a marginal or wrong fix can never replace the float.
        val carrierSamples = rtkCarrierSamples(matched)
        val float3dSigmaMeters =
            sqrt(
                (
                    floatSolution.covariance[0][0] +
                        floatSolution.covariance[1][1] +
                        floatSolution.covariance[2][2]
                ).coerceAtLeast(0.0),
            )
        val maxFixFloatSep = max(1.5, 4.0 * float3dSigmaMeters)
        val rawFixedSolution =
            if (carrierInputs.size >= RtkDoubleDifferenceSolver.MIN_FIX_SATELLITES) {
                var carried: List<RtkDoubleDifferenceSolver.AmbiguityState>? = null
                val fixed =
                    RtkDoubleDifferenceSolver.solveFixed(
                        baseEcef = baseEcef,
                        inputs = carrierInputs,
                        initialRover = floatSolution.roverEcef,
                        floatRoverEcef = floatSolution.roverEcef,
                        priors = rtkAmbiguityMemory.applicablePriors(carrierSamples, gpsTimeNanos),
                        onFloat = { carried = it },
                        maxFixFloatSeparationMeters = maxFixFloatSep,
                    )
                carried?.let { rtkAmbiguityMemory.remember(it, carrierSamples, gpsTimeNanos) }
                fixed
            } else {
                null
            }
        val fixedSolution =
            rawFixedSolution?.takeIf {
                ecefDistanceMeters(baseEcef, it.roverEcef) <= GnssMathUtils.MAX_RTK_BASELINE_METERS
            }

        val roverEcef: DoubleArray
        val residualRms: Double
        val covariance: Array<DoubleArray>
        val usedCount: Int
        val degreesOfFreedom: Int
        val refSigma: Double
        val mode: String
        if (fixedSolution != null) {
            roverEcef = fixedSolution.roverEcef
            residualRms = fixedSolution.residualRms
            covariance = fixedSolution.covariance
            usedCount = fixedSolution.usedSatelliteCount
            degreesOfFreedom = fixedSolution.degreesOfFreedom
            refSigma = carrierInputs.map { it.carrierSigmaMeters }.average().coerceAtLeast(0.001)
            mode = "FIXED ratio=${"%.1f".format(fixedSolution.ratio)}"
        } else {
            roverEcef = floatSolution.roverEcef
            residualRms = floatSolution.residualRms
            covariance = floatSolution.covariance
            usedCount = floatSolution.usedSatelliteCount
            degreesOfFreedom = floatSolution.degreesOfFreedom
            refSigma = codeInputs.map { it.sigmaMeters }.average().coerceAtLeast(0.1)
            mode = "FLOAT"
        }

        val (lat, lon, alt) = GnssMathUtils.ecefToLatLonAlt(roverEcef[0], roverEcef[1], roverEcef[2])
        pppState.updatePosition(roverEcef[0], roverEcef[1], roverEcef[2], gpsTimeNanos)

        val accuracy =
            computeRtkAccuracy(
                covariance = covariance,
                lat = lat,
                lon = lon,
                refSigmaMeters = refSigma,
                residualRms = residualRms,
                degreesOfFreedom = degreesOfFreedom,
                isFloatMode = fixedSolution == null,
            )
        val rtkSigma0 =
            if (fixedSolution == null && (degreesOfFreedom <= 0 || residualRms == 0.0)) {
                refSigma
            } else {
                residualRms.coerceAtLeast(RTK_ACCURACY_SIGMA0_FLOOR_METERS)
            }
        val rtkVarianceFactor = (rtkSigma0 / refSigma).pow(2).coerceAtLeast(1.0)
        val rtkPosCovariance =
            Array(3) { ri ->
                DoubleArray(3) { cj ->
                    rtkVarianceFactor * covariance[ri][cj]
                }
            }
        val doppler =
            tdcp.dopplerVelocity(
                measurements = measurements,
                receiverX = roverEcef[0],
                receiverY = roverEcef[1],
                receiverZ = roverEcef[2],
                clockBiases = DoubleArray(1) { 0.0 },
                clockGroupToIndex = emptyMap(),
                clockGroupBySatKey = emptyMap(),
                gpstSeconds = gpstSeconds,
            )
        interEpochKalman.syncToExternalFix(
            positionEcef = Triple(roverEcef[0], roverEcef[1], roverEcef[2]),
            positionCovarianceEcef = rtkPosCovariance,
            timeNanos = gpsTimeNanos,
            velocityEcef = doppler?.let { Triple(it.velocityX, it.velocityY, it.velocityZ) },
            velocityCovarianceEcef = doppler?.covarianceEcef,
        )
        tdcp.clear()
        val speedAndBearing = if (doppler == null) null else tdcp.speedAndBearingDegrees(lat, lon)
        Log.d(
            TAG,
            "RTK $mode: sats=$usedCount hAcc=${"%.2f".format(accuracy.horizontalMeters)}m rms=${"%.2f".format(residualRms)}m ${baseStore.describe()}",
        )

        return PositionResult(
            latitude = lat,
            longitude = lon,
            altitude = alt,
            horizontalAccuracyMeters = accuracy.horizontalMeters.toFloat(),
            verticalAccuracyMeters = accuracy.verticalMeters.toFloat(),
            timestamp = System.currentTimeMillis(),
            gdop = accuracy.pdop,
            pdop = accuracy.pdop,
            hdop = accuracy.hdop,
            vdop = accuracy.vdop,
            tdop = 0.0,
            residualsRms = residualRms,
            satelliteCount = measurements.distinctBy { it.constellationType to it.svid }.size,
            usedInFixCount = usedCount,
            solutionMode = if (fixedSolution != null) GnssSolutionMode.RTK_FIXED else GnssSolutionMode.RTK_FLOAT,
            speedMetersPerSecond = speedAndBearing?.first,
            bearingDegrees = speedAndBearing?.second,
        )
    }

    internal data class RtkAccuracy(
        val horizontalMeters: Double,
        val verticalMeters: Double,
        val hdop: Double,
        val vdop: Double,
        val pdop: Double,
    )

    /**
     * Derives horizontal/vertical accuracy and DOP from the double-difference position covariance.
     *
     * The reported accuracy is **a-posteriori**, matching the absolute solver: the geometric DOP
     * (formal ENU sigma normalized by the a-priori observation sigma) is scaled by the *realized*
     * post-fit residual RMS, not the assumed measurement noise. This keeps the accuracy honest — a
     * wrong integer fix, a bad base decode, or a long baseline all blow up the residuals and inflate
     * the reported accuracy, instead of advertising a tight circle around a position that did not
     * actually improve. (A purely formal covariance would always look tight regardless of realized
     * error, which biases the best-candidate selection toward overconfident fixes.)
     */
    internal fun computeRtkAccuracy(
        covariance: Array<DoubleArray>,
        lat: Double,
        lon: Double,
        refSigmaMeters: Double,
        residualRms: Double,
        degreesOfFreedom: Int = 1,
        isFloatMode: Boolean = false,
    ): RtkAccuracy {
        val latRad = Math.toRadians(lat)
        val lonRad = Math.toRadians(lon)
        val sinLat = sin(latRad)
        val cosLat = cos(latRad)
        val sinLon = sin(lonRad)
        val cosLon = cos(lonRad)
        val r =
            arrayOf(
                doubleArrayOf(-sinLon, cosLon, 0.0),
                doubleArrayOf(-sinLat * cosLon, -sinLat * sinLon, cosLat),
                doubleArrayOf(cosLat * cosLon, cosLat * sinLon, sinLat),
            )
        val qEnu = Array(3) { DoubleArray(3) }
        for (i in 0 until 3) {
            for (j in 0 until 3) {
                var sum = 0.0
                for (k in 0 until 3) {
                    for (l in 0 until 3) {
                        sum += r[i][k] * covariance[k][l] * r[j][l]
                    }
                }
                qEnu[i][j] = sum
            }
        }
        val varE = qEnu[0][0].coerceAtLeast(0.0)
        val varN = qEnu[1][1].coerceAtLeast(0.0)
        val varU = qEnu[2][2].coerceAtLeast(0.0)
        val refSigma = refSigmaMeters.coerceAtLeast(1e-3)
        // Geometric dilution: formal ENU sigma divided by the a-priori observation sigma.
        val hdop = sqrt(varE + varN) / refSigma
        val vdop = sqrt(varU) / refSigma
        val pdop = sqrt(varE + varN + varU) / refSigma

        // A-posteriori unit-weight sigma from the realized residuals (honest scale), floored so a
        // genuine carrier fix can still report a few cm but never an unrealistic sub-cm circle.
        // A 0-DOF code float has identically zero residuals by algebra, so keep the a-priori
        // code sigma rather than advertising 2 cm accuracy from 4 satellites.
        val sigma0 =
            if (isFloatMode && (degreesOfFreedom <= 0 || residualRms == 0.0)) {
                refSigma
            } else {
                residualRms.coerceAtLeast(RTK_ACCURACY_SIGMA0_FLOOR_METERS)
            }

        return RtkAccuracy(
            horizontalMeters =
                (sigma0 * hdop).coerceIn(
                    GnssMathUtils.MIN_HORIZONTAL_ACCURACY.toDouble(),
                    GnssMathUtils.MAX_HORIZONTAL_ACCURACY.toDouble(),
                ),
            verticalMeters =
                (sigma0 * vdop).coerceIn(
                    GnssMathUtils.MIN_HORIZONTAL_ACCURACY.toDouble(),
                    (GnssMathUtils.MAX_HORIZONTAL_ACCURACY * GnssMathUtils.VERTICAL_ACCURACY_FACTOR).toDouble(),
                ),
            hdop = hdop,
            vdop = vdop,
            pdop = pdop,
        )
    }

    companion object {
        private const val TAG = "GnssMeasurementProcessor"

        /** Zenith one-sigma carrier-phase noise (m) for RTK; scaled by 1/sin(elevation) per satellite. */
        private const val CARRIER_PHASE_BASE_SIGMA_METERS = 0.003

        /** Minimum geometric elevation angle (degrees) for both base and rover in RTK. */
        private const val RTK_MIN_ELEVATION_DEG = 10.0

        /** Baseline threshold (m) above which differential Klobuchar ionosphere is applied in RTK. */
        private const val RTK_IONO_BASELINE_THRESHOLD_METERS = 10_000.0

        /** Max base/rover epoch offset (s) tolerated for a double difference; guards against stale base data. */
        private const val MAX_BASE_ROVER_EPOCH_DIFF_SECONDS = 30.0

        /**
         * Code double-difference post-fit residual RMS above which the RTK fix is rejected (and the
         * caller falls back to absolute positioning). Healthy code DD is sub-metre to a few metres;
         * a much larger value means a bad/uncorrectable satellite (often the reference) corrupted the
         * solution this epoch.
         */
        private const val RTK_MAX_ACCEPTABLE_FLOAT_RMS_METERS = 15.0

        /**
         * Floor for the a-posteriori unit-weight sigma in RTK accuracy. Lets a genuine carrier fix
         * report a few cm while preventing an unrealistically optimistic sub-cm circle on consumer
         * hardware.
         */
        private const val RTK_ACCURACY_SIGMA0_FLOOR_METERS = 0.02

        /** Minimum reasonable pseudorange in meters (~15,000 km — low-elevation GEO/MEO). */
        private const val MIN_REASONABLE_PSEUDORANGE_METERS = 15_000_000.0

        /** Maximum reasonable pseudorange in meters (~42,000 km — SBAS/QZSS GEO at low elevation). */
        private const val MAX_REASONABLE_PSEUDORANGE_METERS = 42_000_000.0

        /**
         * Lower clamp for travel time used during satellite-position / clock refinement. Travel
         * times below ~50 ms would imply a physically impossible SV distance (< 15,000 km) and
         * typically indicate a runaway receiver-clock estimate during early WLS iterations.
         */
        private const val MIN_TRAVEL_TIME_SECONDS =
            MIN_REASONABLE_PSEUDORANGE_METERS / SPEED_OF_LIGHT

        /** Upper clamp for travel time (~117 ms, corresponding to ~35,000 km). */
        private const val MAX_TRAVEL_TIME_SECONDS =
            MAX_REASONABLE_PSEUDORANGE_METERS / SPEED_OF_LIGHT
    }
}
