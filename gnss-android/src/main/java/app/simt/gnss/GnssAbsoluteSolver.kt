package app.simt.gnss

import android.location.GnssStatus
import android.util.Log
import app.simt.gnss.core.correction.AtmosphericCorrection
import app.simt.gnss.core.correction.SsrCorrectionApplicator
import app.simt.gnss.core.filter.GnssInterEpochKalmanFilter
import app.simt.gnss.core.model.PositionResult
import app.simt.gnss.core.model.SatelliteEphemeris
import app.simt.gnss.core.model.SatelliteMeasurement
import app.simt.gnss.core.orbit.SatellitePositionCalculator
import app.simt.gnss.core.solver.GnssPositionSolver
import app.simt.gnss.core.solver.GnssPositionSolver.Accuracy
import app.simt.gnss.core.utils.GnssMathUtils
import app.simt.gnss.core.utils.GnssMathUtils.SPEED_OF_LIGHT
import kotlin.math.*

/**
 * Absolute (broadcast/PPP) position solver: the single-epoch WLS solve with
 * RAIM outlier rejection, SSR orbit/clock/code-bias application, ionosphere and
 * troposphere modeling, DOP/covariance accuracy, and the inter-epoch Kalman and
 * Doppler state updates.
 */
internal class GnssAbsoluteSolver(
    private val ephemerisStore: GnssEphemerisStore,
    private val ingest: GnssNavMessageIngest,
    private val ssr: SsrCorrectionPipeline,
    private val pppState: GnssMeasurementProcessor.PppState,
    private val interEpochKalman: GnssInterEpochKalmanFilter,
    private val tdcp: TdcpMemory,
) {
    internal fun solve(
        measurements: List<SatelliteMeasurement>,
        initialLat: Double = 0.0,
        initialLon: Double = 0.0,
        initialAlt: Double = 0.0,
        usePpp: Boolean = true,
        gpsTimeNanos: Long = 0L,
        accuracy: Accuracy = Accuracy.HIGH,
        commitState: Boolean = true,
    ): PositionResult? {
        if (measurements.size < 4) return null

        val nowTow = (gpsTimeNanos / 1e9) % GnssMathUtils.SECONDS_IN_WEEK
        val usableEphemerisBySat = HashMap<Pair<Int, Int>, SatelliteEphemeris>()
        for (meas in measurements) {
            val satKey = meas.constellationType to meas.svid
            if (!usableEphemerisBySat.containsKey(satKey)) {
                ephemerisStore.get(satKey)
                    ?.takeIf { ephemerisStore.isUsableEphemerisLocked(it, nowTow) }
                    ?.let { usableEphemerisBySat[satKey] = it }
            }
        }

        val groupedBySv = measurements.groupBy { it.constellationType to it.svid }
        // Use a mutable list that we can modify during RAIM iteration
        var activeMeasurements = mutableListOf<SatelliteMeasurement>()
        val ifCorrectedSatKeys = mutableSetOf<Pair<Int, Int>>()
        // IF-combined SVs whose satellite clock must EXCLUDE the broadcast group delay
        // (LNAV/BGD path: the per-signal biases cancel in the IF combination).
        val ifClockWithoutGroupDelaySatKeys = mutableSetOf<Pair<Int, Int>>()
        val ssrCodeBiasAppliedSatKeys = mutableSetOf<Pair<Int, Int>>()
        val ifPairComponentsBySat = HashMap<Pair<Int, Int>, IfPairComponents>()

        // Initial filtering and IF combination
        initialFilteringAndIfCombination(
            groupedBySv,
            usableEphemerisBySat,
            ssr,
            ingest,
            usePpp,
            activeMeasurements,
            ifCorrectedSatKeys,
            ifClockWithoutGroupDelaySatKeys,
            ssrCodeBiasAppliedSatKeys,
            ifPairComponentsBySat,
        )

        // RAIM Iteration Loop
        // We will try up to 3 times to remove outliers (or until we run out of satellites)
        // Max excluded satellites = 3 to avoid excessive computation
        var excludedCount = 0
        val maxExclusions = 3

        if (!pruneMeasurementsWithoutEphemeris(activeMeasurements, usableEphemerisBySat)) {
            return null
        }

        // RAIM may strip SVs across iterations; constellation set is recomputed inside the
        // loop. The exclusion budget (maxExclusions) is enforced in the RAIM branch below,
        // so the loop is bounded solely by having enough satellites to solve.
        while (activeMeasurements.size >= 4) {
            pruneSingleSatelliteConstellations(activeMeasurements)
            if (activeMeasurements.size < 4) break

            // --- Standard WLS / PPP Logic Start ---

            // All SVs now have ephemeris (approximate-geometry-only SVs were filtered out above)
            val maxWlsIterations = GnssMathUtils.MAX_WLS_ITERATIONS
            val positionConvergenceThresholdMeters = GnssMathUtils.CONVERGENCE_THRESHOLD_METERS
            val clockConvergenceThresholdMeters = GnssMathUtils.CONVERGENCE_THRESHOLD_METERS * GnssMathUtils.SPEED_OF_LIGHT / 1e6
            val raimResidualThresholdMeters = GnssMathUtils.RAIM_RESIDUAL_THRESHOLD_METERS
            val minHorizontalAccuracyMeters = GnssMathUtils.MIN_HORIZONTAL_ACCURACY
            val maxHorizontalAccuracyMeters = GnssMathUtils.MAX_HORIZONTAL_ACCURACY

            // Day-of-year (1..366 GPST-based) used for Niell seasonal amplitude in the
            // hydrostatic mapping function. Derive from GPS epoch so troposphere modelling
            // stays aligned across hemispheres and seasons.
            val dayOfYear: Int =
                run {
                    val unixSeconds = GnssMathUtils.GPS_EPOCH_UNIX_SECONDS + (gpsTimeNanos / 1_000_000_000L)
                    val utcCalendar = java.util.Calendar.getInstance(java.util.TimeZone.getTimeZone("UTC"))
                    utcCalendar.timeInMillis = unixSeconds * 1000L
                    utcCalendar.get(java.util.Calendar.DAY_OF_YEAR)
                }

            val pppSeed =
                if (usePpp) {
                    val candidate = pppState.getLastPositionEcef(currentTimeNanos = gpsTimeNanos)
                    if (candidate != null && (initialLat != 0.0 || initialLon != 0.0)) {
                        val (initX, initY, initZ) = GnssMathUtils.latLonAltToEcef(initialLat, initialLon, initialAlt)
                        val dx = candidate.first - initX
                        val dy = candidate.second - initY
                        val dz = candidate.third - initZ
                        if (sqrt(dx * dx + dy * dy + dz * dz) > MAX_PPP_SEED_DEVIATION_METERS) null else candidate
                    } else {
                        candidate
                    }
                } else {
                    null
                }
            var (currentX, currentY, currentZ) =
                pppSeed ?: GnssMathUtils.latLonAltToEcef(initialLat, initialLon, initialAlt)
            var coarsePositionEstimate =
                (pppSeed == null && initialLat == 0.0 && initialLon == 0.0) ||
                    (currentX * currentX + currentY * currentY + currentZ * currentZ < COARSE_ECEF_RADIUS_SQ_THRESHOLD)

            // Handle multi-constellation clock biases (ISB), splitting BeiDou B3I-calibrated vs
            // uncalibrated BDS-3 (B1C/B2a) into two clock groups when both sub-groups have >= 2 SVs.
            val clockGroupBySatKey = buildClockGroupBySatKey(activeMeasurements, ifPairComponentsBySat, ssrCodeBiasAppliedSatKeys)
            val clockGroupIds = clockGroupBySatKey.values.distinct().sorted()
            val numClockStates = clockGroupIds.size
            if (activeMeasurements.size < 3 + numClockStates) break
            val clockGroupToIndex = clockGroupIds.withIndex().associate { it.value to it.index }

            val clockBiases = DoubleArray(numClockStates) { 0.0 }
            var lastResidualsRms = 0.0
            // A-posteriori variance-of-unit-weight (reduced chi-square), dimensionless. Scales the
            // metric formal covariance for honest accuracy/Kalman-R reporting; 1.0 = fit matches
            // the a-priori measurement model. Floored at 1.0 so we never report better than formal.
            var lastVarianceFactor = 1.0
            var lastRedundancyAllRows = 0

            var maxResidual = 0.0
            var maxResidualIdx = -1
            var maxStandardizedResidual = 0.0
            var maxStandardizedResidualIdx = -1

            // WLS Iterations
            var wlsConverged = false
            var designMatrix: Array<DoubleArray>? = null
            var observationVector: DoubleArray? = null
            var solutionWrapper: GnssPositionSolver.WlsSolution? = null
            // Unweighted geometry normal-matrix inverse for true (dimensionless) DOPs.
            var dopMatrix: Array<DoubleArray>? = null

            // Per-SV cache of satellite position / clock and atmospheric model terms, reused across
            // iterations while the receiver estimate barely moves. Scoped to this RAIM pass so a
            // re-solve after an outlier exclusion starts clean.
            val svModelCache = HashMap<Pair<Int, Int>, SvModelTerms>()

            for (iter in 0 until maxWlsIterations) {
                val (currentLat, currentLon, currentAlt) = GnssMathUtils.ecefToLatLonAlt(currentX, currentY, currentZ)
                val observerLat = sanitizeObserverLatitudeDegrees(currentLat, initialLat)
                val observerLon = sanitizeObserverLongitudeDegrees(currentLon, initialLon)
                val observerAlt = sanitizeObserverAltitudeMeters(currentAlt, initialAlt)

                // Single-epoch phase rows each own a free ambiguity, so they do not inform
                // position, and a free zenith wet delay then inflates height and clock
                // (review #85). The troposphere stays fully modelled. Sequential PPP is
                // what would make the phase informative.
                val n = activeMeasurements.size
                val m = 3 + numClockStates

                designMatrix = Array(n) { DoubleArray(m) }
                observationVector = DoubleArray(n)
                val weights = DoubleArray(n)
                // Per-row a-priori measurement sigma (σ_i). Parallel to [weights]; used by
                // the standardized-residual RAIM check after the WLS solve converges.
                val measurementSigmas = DoubleArray(n)

                for (i in activeMeasurements.indices) {
                    val meas = activeMeasurements[i]
                    val satKey = meas.constellationType to meas.svid
                    val ephemeris = usableEphemerisBySat[satKey] ?: continue

                    val gpstSeconds = (gpsTimeNanos / 1e9) % GnssMathUtils.SECONDS_IN_WEEK
                    val currentTimeSeconds = if (meas.constellationType == GnssStatus.CONSTELLATION_BEIDOU) gpstSeconds - GnssMathUtils.BDT_GPST_OFFSET_SECONDS else gpstSeconds
                    val ssrPair = ssr.getApplicableSsrPair(meas.constellationType, meas.svid, ephemeris, usePpp, leapSeconds = ingest.lastKnownLeapSeconds)

                    val clockGroupId = clockGroupBySatKey[satKey] ?: meas.constellationType
                    val currentConstellationBias =
                        clockBiases[clockGroupToIndex[clockGroupId] ?: 0]

                    // Per-SV model terms (satellite ECEF + clock, ionosphere, troposphere) depend only
                    // on the ephemeris/clock-bias and the observer geometry, which converge within the
                    // first WLS iterations. They are insensitive to sub-metre changes in the receiver
                    // estimate (the satellite position does not depend on it at all; iono/tropo gradients
                    // are < 1 mm per metre), so once the receiver settles within
                    // WLS_MODEL_REUSE_DISTANCE_METERS of where they were last evaluated we reuse them —
                    // skipping the per-iteration Kepler solve and Klobuchar evaluation. The
                    // line-of-sight, range, relativistic (Shapiro) and residual below are still rebuilt
                    // every iteration from the current receiver position.
                    val cachedModel = if (!coarsePositionEstimate) svModelCache[satKey] else null
                    val reusableModel =
                        cachedModel?.takeIf {
                            abs(currentX - it.refX) <= GnssMathUtils.WLS_MODEL_REUSE_DISTANCE_METERS &&
                                abs(currentY - it.refY) <= GnssMathUtils.WLS_MODEL_REUSE_DISTANCE_METERS &&
                                abs(currentZ - it.refZ) <= GnssMathUtils.WLS_MODEL_REUSE_DISTANCE_METERS &&
                                abs(currentConstellationBias - it.refClockBias) <= GnssMathUtils.WLS_MODEL_REUSE_DISTANCE_METERS
                        }

                    val satX: Double
                    val satY: Double
                    val satZ: Double
                    val iterElevationDeg: Double
                    val iterAzimuthDeg: Double
                    val dryDelay: Double
                    val ionoDelayL1: Double
                    val satClockCorr: Double

                    if (reusableModel != null) {
                        satX = reusableModel.satX
                        satY = reusableModel.satY
                        satZ = reusableModel.satZ
                        iterElevationDeg = reusableModel.elevationDeg
                        iterAzimuthDeg = reusableModel.azimuthDeg
                        dryDelay = reusableModel.dryDelay
                        ionoDelayL1 = reusableModel.ionoDelayL1
                        satClockCorr = reusableModel.satClockMeters
                    } else {
                        // The pseudorange and currentTimeSeconds share the receiver clock, so it
                        // cancels in the transmit epoch (t = t_rx − PR/c − Δt_sv). The flight
                        // time that rotates ECEF keeps the estimated clock: (PR − b̂)/c + Δt_sv.
                        // One clock evaluation at t_rx − PR/c is enough; af1 across Δt_sv is noise.
                        val approxFlight =
                            (meas.pseudorangeMeters / SPEED_OF_LIGHT)
                                .coerceIn(MIN_TRAVEL_TIME_SECONDS, MAX_TRAVEL_TIME_SECONDS)
                        val satelliteClockSeconds =
                            SatellitePositionCalculator.getSatelliteClockCorrection(
                                ephemeris,
                                currentTimeSeconds - approxFlight,
                                includeGroupDelay = false,
                            )
                        val transmitTiming =
                            SatellitePositionCalculator.transmitTiming(
                                receptionTimeSeconds = currentTimeSeconds,
                                pseudorangeMeters = meas.pseudorangeMeters,
                                receiverClockBiasMeters = currentConstellationBias,
                                satelliteClockSeconds = satelliteClockSeconds,
                            )
                        val travelTime =
                            transmitTiming.rotationSeconds.coerceIn(MIN_TRAVEL_TIME_SECONDS, MAX_TRAVEL_TIME_SECONDS)
                        val txTime = transmitTiming.transmitTimeSeconds
                        val (tx, ty, tz) =
                            SatellitePositionCalculator.calculateSatellitePosition(
                                ephemeris,
                                txTime,
                            )
                        val (rx, ry, rz) =
                            SatellitePositionCalculator.rotateEcefForEarthRotationDuringTransit(
                                transmitX = tx,
                                transmitY = ty,
                                transmitZ = tz,
                                transitTimeSeconds = travelTime,
                            )
                        // Apply SSR orbit correction if available. Gated on the broadcast
                        // ephemeris IOD and atomic orbit+clock pairing (with usePpp).
                        val ssrOrbit = ssrPair?.first
                        val (computedSatX, computedSatY, computedSatZ) =
                            if (ssrOrbit != null) {
                                val vel =
                                    SsrCorrectionApplicator.computeSatelliteVelocity(
                                        ephemeris,
                                        txTime,
                                        Triple(tx, ty, tz),
                                        SatellitePositionCalculator::calculateSatellitePosition,
                                    )
                                val dtSsr =
                                    ssr.polynomialAgeSeconds(
                                        epochTime = ssrOrbit.epochTime,
                                        updateIntervalSeconds = ssrOrbit.updateIntervalSeconds,
                                        timescale = ssrOrbit.epochTimescale,
                                        gpstSeconds = gpstSeconds,
                                        currentTimeSeconds = currentTimeSeconds,
                                        leapSeconds = ingest.lastKnownLeapSeconds,
                                    )
                                val beidouGeo =
                                    meas.constellationType == GnssStatus.CONSTELLATION_BEIDOU &&
                                        (meas.svid in 1..5 || meas.svid in 59..63)
                                SsrCorrectionApplicator.applyOrbitCorrection(
                                    rx,
                                    ry,
                                    rz,
                                    vel.first,
                                    vel.second,
                                    vel.third,
                                    ssrOrbit.deltaRadial + ssrOrbit.dotRadial * dtSsr,
                                    ssrOrbit.deltaAlongTrack + ssrOrbit.dotAlongTrack * dtSsr,
                                    ssrOrbit.deltaCrossTrack + ssrOrbit.dotCrossTrack * dtSsr,
                                    nonRotatingVelocity = beidouGeo,
                                )
                            } else {
                                // Apply SBAS long-term orbit correction if available
                                // (GPS only, IODE-matched to this broadcast ephemeris)
                                val sbasLtc =
                                    if (meas.constellationType == GnssStatus.CONSTELLATION_GPS) {
                                        ingest.getSbasLongTermCorrection(meas.svid, ephemeris.iode)
                                    } else {
                                        null
                                    }
                                if (sbasLtc != null) {
                                    Triple(rx + sbasLtc.deltaX, ry + sbasLtc.deltaY, rz + sbasLtc.deltaZ)
                                } else {
                                    Triple(rx, ry, rz)
                                }
                            }
                        satX = computedSatX
                        satY = computedSatY
                        satZ = computedSatZ

                        val (computedElevationDeg, computedAzimuthDeg) =
                            GnssMathUtils.ecefLosToElevationAzimuth(
                                receiverX = currentX,
                                receiverY = currentY,
                                receiverZ = currentZ,
                                receiverLatDeg = currentLat,
                                receiverLonDeg = currentLon,
                                satelliteX = satX,
                                satelliteY = satY,
                                satelliteZ = satZ,
                            )
                        iterElevationDeg = computedElevationDeg
                        iterAzimuthDeg = computedAzimuthDeg

                        dryDelay =
                            if (coarsePositionEstimate) {
                                0.0
                            } else {
                                AtmosphericCorrection.computeTroposphericDelay(
                                    iterElevationDeg,
                                    observerAlt,
                                    observerLat,
                                    dayOfYear,
                                )
                            }

                        // Single-frequency ionosphere: Klobuchar with the broadcast α/β (and the
                        // geomagnetic-latitude step), scaled per carrier below. Galileo SVs use
                        // Klobuchar too: the previous "NeQuick-G" implementation substituted an
                        // invented climatology for the CCIR maps/MODIP grid the broadcast aᵢ
                        // coefficients are fitted against, making its output unvalidated and
                        // regionally worse than Klobuchar — a real NeQuick-G needs those tables.
                        ionoDelayL1 =
                            if (coarsePositionEstimate || ifCorrectedSatKeys.contains(satKey)) {
                                0.0
                            } else {
                                AtmosphericCorrection.computeIonosphericDelay(iterElevationDeg, iterAzimuthDeg, observerLat, observerLon, gpsTimeNanos / 1_000_000L, ingest.ionoAlpha, ingest.ionoBeta)
                            }

                        val clockTimeSec = transmitTiming.transmitTimeSeconds
                        // Satellite clock WITHOUT the broadcast group delay; the
                        // signal-specific term is applied below. GPS/BeiDou: none on the
                        // LNAV ionosphere-free path, plain TGD for CNAV/BeiDou, γ·TGD
                        // on one frequency. Galileo (OS SIS ICD §5.1.5): the clock's own
                        // BGD on E1, and for an I/NAV clock the E5a and E1/E5a formulas
                        // that convert it onto the F/NAV reference.
                        var clockMeters =
                            SatellitePositionCalculator.getSatelliteClockCorrection(
                                ephemeris,
                                clockTimeSec,
                                includeGroupDelay = false,
                            ) * SPEED_OF_LIGHT
                        val groupDelaySeconds =
                            measurementGroupDelaySeconds(
                                ephemeris = ephemeris,
                                constellationType = meas.constellationType,
                                carrierFrequencyHz = meas.carrierFrequencyHz,
                                ionosphereFree = satKey in ifCorrectedSatKeys,
                                clockExcludesGroupDelay = satKey in ifClockWithoutGroupDelaySatKeys || satKey in ssrCodeBiasAppliedSatKeys,
                                lowerBandHz = ifPairComponentsBySat[satKey]?.fLowHz ?: GnssMathUtils.L5_FREQUENCY_HZ,
                            )
                        if (groupDelaySeconds.isFinite() && groupDelaySeconds != 0.0) {
                            clockMeters -= groupDelaySeconds * SPEED_OF_LIGHT
                        }
                        // Apply SSR clock correction if available (atomic orbit+clock pairing)
                        val ssrClock = ssrPair?.second
                        if (ssrClock != null) {
                            val dtSsr =
                                ssr.polynomialAgeSeconds(
                                    epochTime = ssrClock.epochTime,
                                    updateIntervalSeconds = ssrClock.updateIntervalSeconds,
                                    timescale = ssrClock.epochTimescale,
                                    gpstSeconds = gpstSeconds,
                                    currentTimeSeconds = currentTimeSeconds,
                                    leapSeconds = ingest.lastKnownLeapSeconds,
                                )
                            clockMeters = SsrCorrectionApplicator.applyClockCorrection(clockMeters, ssrClock.c0, ssrClock.c1, ssrClock.c2, dtSsr)
                            val hrClock =
                                if (ssrClock.iodSsr >= 0) {
                                    ssr.store.getHighRateClock(meas.constellationType, meas.svid, ssrClock.iodSsr)
                                } else {
                                    null
                                }
                            if (hrClock != null) {
                                val dtHr =
                                    abs(
                                        ssr.polynomialAgeSeconds(
                                            epochTime = hrClock.epochTime,
                                            updateIntervalSeconds = hrClock.updateIntervalSeconds,
                                            timescale = hrClock.epochTimescale,
                                            gpstSeconds = gpstSeconds,
                                            currentTimeSeconds = currentTimeSeconds,
                                            leapSeconds = ingest.lastKnownLeapSeconds,
                                        ),
                                    )
                                if (dtHr <= max(30.0, 2.0 * hrClock.updateIntervalSeconds)) {
                                    clockMeters += hrClock.deltaClockMeters
                                }
                            }
                        } else {
                            // SBAS fast (PRC) and long-term clock corrections are an
                            // *alternative* to SSR, not additive: both correct the same
                            // broadcast clock error, so stacking them on an SSR clock
                            // would double-correct (mirrors the orbit-side precedence).
                            clockMeters += ingest.getSbasFastCorrectionMeters(meas.constellationType, meas.svid)
                            clockMeters += ingest.getSbasLongTermClockCorrectionMeters(meas.constellationType, meas.svid, ephemeris.iode)
                        }
                        satClockCorr = clockMeters

                        if (!coarsePositionEstimate) {
                            svModelCache[satKey] =
                                SvModelTerms(
                                    refX = currentX,
                                    refY = currentY,
                                    refZ = currentZ,
                                    refClockBias = currentConstellationBias,
                                    satX = satX,
                                    satY = satY,
                                    satZ = satZ,
                                    satClockMeters = satClockCorr,
                                    ionoDelayL1 = ionoDelayL1,
                                    dryDelay = dryDelay,
                                    elevationDeg = iterElevationDeg,
                                    azimuthDeg = iterAzimuthDeg,
                                )
                        }
                    }

                    val range = sqrt((satX - currentX).pow(2) + (satY - currentY).pow(2) + (satZ - currentZ).pow(2))
                    if (range < 1e-6) continue

                    val dx = (satX - currentX) / range
                    val dy = (satY - currentY) / range
                    val dz = (satZ - currentZ) / range

                    // Code observation: partial derivatives of PR w.r.t. receiver position
                    // dPR/d(rec) = -(sat - rec)/range, so negate the unit LOS vector
                    designMatrix[i][0] = -dx
                    designMatrix[i][1] = -dy
                    designMatrix[i][2] = -dz
                    val constellationIdx = clockGroupToIndex[clockGroupId] ?: 0
                    designMatrix[i][3 + constellationIdx] = 1.0

                    // Klobuchar returns the slant delay at the GPS L1 frequency. Scale to the
                    // measurement's actual carrier by (f_L1/f)² so that single-frequency non-L1
                    // SVs (L5/E5a-only, or GLONASS L1) are corrected at the right magnitude.
                    // L1/E1 scales by about 1. IF-combined SVs have ionoDelayL1 = 0.
                    val ionoDelay =
                        if (meas.carrierFrequencyHz > 0.0) {
                            ionoDelayL1 * (GnssMathUtils.L1_FREQUENCY_HZ / meas.carrierFrequencyHz).pow(2)
                        } else {
                            ionoDelayL1
                        }
                    val relCorr =
                        AtmosphericCorrection.computeRelativisticCorrection(
                            receiverX = currentX,
                            receiverY = currentY,
                            receiverZ = currentZ,
                            satelliteX = satX,
                            satelliteY = satY,
                            satelliteZ = satZ,
                        )

                    val currentClockBias = currentConstellationBias
                    // Observation residual: PR + satClockCorr - range - sagnac - clockBias - tropo - iono - shapiro
                    // Shapiro delay increases measured PR, so subtract it
                    val pMeasured = meas.pseudorangeMeters + satClockCorr - range - currentClockBias - dryDelay - ionoDelay - relCorr
                    observationVector[i] = pMeasured

                    // Observation variance model: combine Android's `receivedSvTimeUncertaintyNanos × c`
                    // (a *time-tagging* floor, typically <1 m) with the physical elevation-/C/N₀-dependent
                    // code noise model σ² = (a² + b²/sin²el) · 10^(−(CN₀−CN₀_ref)/10). On IF-combined
                    // rows, propagate each band's physical sigma through the IF variance factor.
                    val sigmaElevationDeg = if (coarsePositionEstimate) COARSE_SIGMA_ELEVATION_DEG else iterElevationDeg
                    val ifComponents = ifPairComponentsBySat[satKey]
                    val sigmaCode =
                        if (ifComponents != null) {
                            val sigmaAndroidHigh =
                                ifComponents.sigmaAndroidHighMeters.coerceIn(
                                    GnssMathUtils.MIN_PSEUDORANGE_SIGMA_METERS,
                                    GnssMathUtils.MAX_PSEUDORANGE_SIGMA_METERS,
                                )
                            val sigmaAndroidLow =
                                ifComponents.sigmaAndroidLowMeters.coerceIn(
                                    GnssMathUtils.MIN_PSEUDORANGE_SIGMA_METERS,
                                    GnssMathUtils.MAX_PSEUDORANGE_SIGMA_METERS,
                                )
                            val sigmaHigh =
                                max(
                                    sigmaAndroidHigh,
                                    GnssMathUtils.pseudorangeSigmaFromElevationAndCn0(
                                        sigmaElevationDeg,
                                        ifComponents.cn0HighDbHz,
                                        ifComponents.smoothingHighEpochs,
                                    ),
                                )
                            val sigmaLow =
                                max(
                                    sigmaAndroidLow,
                                    GnssMathUtils.pseudorangeSigmaFromElevationAndCn0(
                                        sigmaElevationDeg,
                                        ifComponents.cn0LowDbHz,
                                        ifComponents.smoothingLowEpochs,
                                    ),
                                )
                            val f1Sq = ifComponents.fHighHz * ifComponents.fHighHz
                            val f2Sq = ifComponents.fLowHz * ifComponents.fLowHz
                            val denom = abs(f1Sq - f2Sq)
                            (sqrt(f1Sq * f1Sq * sigmaHigh * sigmaHigh + f2Sq * f2Sq * sigmaLow * sigmaLow) / denom)
                                .coerceIn(
                                    GnssMathUtils.MIN_PSEUDORANGE_SIGMA_METERS,
                                    GnssMathUtils.MAX_PSEUDORANGE_SIGMA_METERS,
                                )
                        } else {
                            val sigmaAndroid =
                                meas.pseudorangeUncertaintyMeters.coerceIn(
                                    GnssMathUtils.MIN_PSEUDORANGE_SIGMA_METERS,
                                    GnssMathUtils.MAX_PSEUDORANGE_SIGMA_METERS,
                                )
                            val sigmaModel =
                                GnssMathUtils.pseudorangeSigmaFromElevationAndCn0(
                                    sigmaElevationDeg,
                                    meas.cn0DbHz,
                                    meas.smoothingEpochs,
                                )
                            max(sigmaAndroid, sigmaModel).coerceIn(
                                GnssMathUtils.MIN_PSEUDORANGE_SIGMA_METERS,
                                GnssMathUtils.MAX_PSEUDORANGE_SIGMA_METERS,
                            )
                        }
                    val sigma =
                        if (ssrPair != null) {
                            hypot(sigmaCode, SSR_ORBIT_CLOCK_SIGMA_METERS)
                        } else {
                            GnssMathUtils.observationSigmaMeters(
                                sigmaCode,
                                ephemerisStore.ephemerisAgeSeconds(ephemeris, currentTimeSeconds),
                            )
                        }
                    measurementSigmas[i] = sigma
                    weights[i] = 1.0 / sigma.pow(2)
                }

                // Intermediate iterations only need the state update (deltaX); the covariance is
                // computed once at convergence (below). Skipping it here avoids an O(m³) inverse
                // on every iteration.
                val solution = GnssPositionSolver.solve(designMatrix, observationVector, weights, accuracy = accuracy, needCovariance = false) ?: break // Matrix singular?
                val deltaX = solution.deltaX

                currentX += deltaX[0]
                currentY += deltaX[1]
                currentZ += deltaX[2]

                for (j in 0 until numClockStates) {
                    clockBiases[j] += deltaX[3 + j]
                }

                val posDelta = sqrt(deltaX[0].pow(2) + deltaX[1].pow(2) + deltaX[2].pow(2))
                val maxClockDelta = (3 until deltaX.size.coerceAtMost(3 + numClockStates)).maxOfOrNull { abs(deltaX[it]) } ?: 0.0
                val converged = !coarsePositionEstimate && posDelta < positionConvergenceThresholdMeters && maxClockDelta < clockConvergenceThresholdMeters

                if (!converged) {
                    coarsePositionEstimate = posDelta > COARSE_POS_DELTA_THRESHOLD_METERS
                    if (iter == 0 || iter == maxWlsIterations - 1) {
                        val ephCount = activeMeasurements.count { m -> ephemerisStore.get(m.constellationType to m.svid)?.let(ephemerisStore::isFreshEphemerisLocked) == true }
                        Log.d(
                            TAG,
                            "WLS iter=$iter posDelta=${"%.1f".format(posDelta)}m clockDelta=${"%.1f".format(maxClockDelta)}m sats=${activeMeasurements.size} ephSats=$ephCount",
                        )
                    }
                    continue
                }

                wlsConverged = true

                // Converged: now compute the covariance once for DOP / accuracy / the Kalman
                // measurement noise, and the standardized-residual RAIM gate. Re-solving the same
                // (geometry-converged) system with the covariance is one extra factorization per
                // accepted fix, far cheaper than computing it on every iteration.
                val convergedSolution = GnssPositionSolver.solve(designMatrix, observationVector, weights, accuracy = accuracy, needCovariance = true) ?: break
                solutionWrapper = convergedSolution

                // DOP is defined on the UNWEIGHTED geometry: Q = (G^T G)^-1 over the code
                // rows' position+clock columns. The weighted covariance (1/sigma^2 weights)
                // is metric (m^2) — reporting its diagonals as "DOP" published sigma*DOP
                // values (and arbitrary ones once phase rows entered the normal matrix).
                dopMatrix =
                    run {
                        val mGeo = 3 + numClockStates
                        val normal = Array(mGeo) { DoubleArray(mGeo) }
                        for (i in 0 until activeMeasurements.size) {
                            if (weights[i] <= 0.0) continue
                            for (r in 0 until mGeo) {
                                val gr = designMatrix[i][r]
                                if (gr == 0.0) continue
                                for (c in r until mGeo) {
                                    normal[r][c] += gr * designMatrix[i][c]
                                }
                            }
                        }
                        for (r in 0 until mGeo) {
                            for (c in 0 until r) normal[r][c] = normal[c][r]
                        }
                        GnssMathUtils.invertSquareMatrix(normal)
                    }

                // Post-fit residuals: residual = b − A · deltaX (observation mismatch after correction).
                var sumSqResiduals = 0.0
                var validCodeRows = 0
                maxResidual = 0.0
                maxResidualIdx = -1
                maxStandardizedResidual = 0.0
                maxStandardizedResidualIdx = -1

                // Standardized residual uses the leverage h_ii of each code row.
                //   h_ii = w_i · aᵢᵀ Q aᵢ,   Q = (AᵀWA)⁻¹  (convergedSolution.covarianceMatrix)
                // z_i = |r_i| / (σ_i · √(1 − h_ii))
                // This auto-scales with each SV's expected noise and its geometric redundancy —
                // replacing the old fixed RAIM_RESIDUAL_THRESHOLD_METERS = 8 m absolute gate.
                val qCov = convergedSolution.covarianceMatrix
                for (i in 0 until activeMeasurements.size) {
                    if (weights[i] <= 0.0) continue
                    validCodeRows++
                    var predicted = 0.0
                    for (j in 0 until m) {
                        predicted += designMatrix[i][j] * deltaX[j]
                    }
                    val res = abs(observationVector[i] - predicted)
                    sumSqResiduals += res.pow(2)

                    if (res > maxResidual) {
                        maxResidual = res
                        maxResidualIdx = i
                    }

                    var aQa = 0.0
                    for (j in 0 until m) {
                        var qAi = 0.0
                        for (k in 0 until m) {
                            qAi += qCov[j][k] * designMatrix[i][k]
                        }
                        aQa += designMatrix[i][j] * qAi
                    }
                    val hii = (weights[i] * aQa).coerceIn(0.0, GnssMathUtils.RAIM_LEVERAGE_MAX)
                    val redundancy = sqrt((1.0 - hii).coerceAtLeast(1e-6))
                    val sigmaI = measurementSigmas[i].coerceAtLeast(GnssMathUtils.MIN_PSEUDORANGE_SIGMA_METERS)
                    val zi = res / (sigmaI * redundancy)
                    if (zi > maxStandardizedResidual) {
                        maxStandardizedResidual = zi
                        maxStandardizedResidualIdx = i
                    }
                }
                lastResidualsRms = if (validCodeRows > 0) sqrt(sumSqResiduals / validCodeRows) else 0.0

                // A-posteriori variance factor (reduced chi-square). The covariance is metric
                // (weights are 1/σ²), so the reported accuracy is sqrt(factor) × formal-sigma:
                // factor ≈ 1 when the fit matches the noise model, > 1 when residuals exceed it.
                //
                // The residual sum and the degrees of freedom must refer to the SAME system:
                // all n rows (code + phase) against dof = n − m. Dividing the code-only sum by
                // validCodeRows − m pinned the factor at 1.0 whenever phase rows were present
                // (m includes the ZWD and one ambiguity per phase row, so validCodeRows − m ≤ 0
                // in the default PPP mode) — reported accuracy never inflated on bad epochs and
                // the Kalman measurement covariance under-weighted them.
                var sumWeightedSqResidualsAllRows = 0.0
                var validRowsAll = 0
                for (i in 0 until n) {
                    if (weights[i] <= 0.0) continue
                    validRowsAll++
                    var predicted = 0.0
                    for (j in 0 until m) {
                        predicted += designMatrix[i][j] * deltaX[j]
                    }
                    val res = observationVector[i] - predicted
                    sumWeightedSqResidualsAllRows += weights[i] * res * res
                }
                val redundancyAllRows = validRowsAll - m
                lastRedundancyAllRows = redundancyAllRows
                lastVarianceFactor =
                    if (redundancyAllRows > 0) {
                        (sumWeightedSqResidualsAllRows / redundancyAllRows).coerceAtLeast(1.0)
                    } else {
                        1.0
                    }

                val ephCount = activeMeasurements.count { m -> ephemerisStore.get(m.constellationType to m.svid)?.let(ephemerisStore::isFreshEphemerisLocked) == true }
                Log.d(
                    TAG,
                    "WLS iter=$iter posDelta=${"%.1f".format(
                        posDelta,
                    )}m clockDelta=${"%.1f".format(
                        maxClockDelta,
                    )}m rms=${"%.1f".format(lastResidualsRms)}m maxRes=${"%.1f".format(maxResidual)}m sats=${activeMeasurements.size} ephSats=$ephCount",
                )
                break
            } // End WLS Loop

            if (wlsConverged && solutionWrapper != null) {
                // RAIM Check — **normalized** (standardized) post-fit residuals.
                //
                // Prefer the standardized residual z_i when leverages are computable (covariance
                // available): a SV is rejected when z_i = |r_i| / (σ_i · √(1 − h_ii)) exceeds
                // RAIM_STANDARDIZED_RESIDUAL_SIGMA. This correctly down-weights low-elevation SVs
                // (which have larger σ_i) and penalises high-leverage observations (small
                // √(1−h_ii)). Fall back to the raw metric gate only when we could not compute
                // standardized residuals (e.g. degenerate covariance).
                val standardizedRaimAvailable = maxStandardizedResidualIdx >= 0
                val raimTrippedStandardized =
                    standardizedRaimAvailable &&
                        lastRedundancyAllRows > 0 &&
                        activeMeasurements.size - 1 >= 4 &&
                        maxStandardizedResidual > GnssMathUtils.RAIM_STANDARDIZED_RESIDUAL_SIGMA
                val raimTrippedAbsolute =
                    !standardizedRaimAvailable &&
                        lastRedundancyAllRows > 0 &&
                        activeMeasurements.size - 1 >= 4 &&
                        maxResidual > raimResidualThresholdMeters

                if ((raimTrippedStandardized || raimTrippedAbsolute) && excludedCount < maxExclusions) {
                    val excludeIdx =
                        if (raimTrippedStandardized) maxStandardizedResidualIdx else maxResidualIdx
                    val candidateAfterExclusion = activeMeasurements.toMutableList()
                    val excludedMeas = candidateAfterExclusion.removeAt(excludeIdx)
                    pruneSingleSatelliteConstellations(candidateAfterExclusion)
                    val remainingClockGroups = buildClockGroupBySatKey(candidateAfterExclusion, ifPairComponentsBySat, ssrCodeBiasAppliedSatKeys).values.distinct().size
                    if (candidateAfterExclusion.size >= 4 && candidateAfterExclusion.size >= 3 + remainingClockGroups) {
                        val why =
                            if (raimTrippedStandardized) {
                                "standardized z=${"%.2f".format(maxStandardizedResidual)}σ (r=${"%.1f".format(maxResidual)}m)"
                            } else {
                                "absolute residual ${"%.1f".format(maxResidual)}m"
                            }
                        Log.w(TAG, "RAIM: Outlier detected! Excluding SVID ${excludedMeas.svid} ($why). Retrying...")
                        activeMeasurements = candidateAfterExclusion
                        excludedCount++
                        continue
                    }
                }

                // If we get here, either no outlier or the exclusion cap was reached.
                // Carrier ambiguities stay float (see [PppState] kdoc): undifferenced /
                // ionosphere-free ambiguities are not single-wavelength integers without
                // external satellite phase-bias products, so no integer fixing is attempted.

                // Create Result — inter-epoch CV Kalman on ECEF position (design remark #2)
                val wlsEcef = Triple(currentX, currentY, currentZ)
                val gpstSeconds = (gpsTimeNanos / 1e9) % GnssMathUtils.SECONDS_IN_WEEK
                val doppler =
                    tdcp.dopplerVelocity(
                        activeMeasurements,
                        currentX,
                        currentY,
                        currentZ,
                        clockBiases,
                        clockGroupToIndex,
                        clockGroupBySatKey,
                        gpstSeconds,
                    )
                val displacement =
                    if (commitState && gpsTimeNanos > 0L) {
                        tdcp.tdcpDisplacement(
                            activeMeasurements,
                            clockBiases,
                            clockGroupToIndex,
                            clockGroupBySatKey,
                            gpstSeconds,
                            gpsTimeNanos,
                        )
                    } else {
                        if (commitState) tdcp.clear()
                        null
                    }
                val (outX, outY, outZ) =
                    if (commitState && gpsTimeNanos > 0L) {
                        // Measurement covariance for the Kalman = a-posteriori variance factor ×
                        // the metric formal covariance (dimensionally correct m²), instead of the
                        // previous residualsRms² × covariance which double-counted the noise scale.
                        val rMeas =
                            Array(3) { ri ->
                                DoubleArray(3) { cj ->
                                    lastVarianceFactor * solutionWrapper.covarianceMatrix[ri][cj]
                                }
                            }
                        interEpochKalman.fuse(
                            wlsEcef,
                            rMeas,
                            gpsTimeNanos,
                            velocityEcef = doppler?.let { Triple(it.velocityX, it.velocityY, it.velocityZ) },
                            velocityCovarianceM2PerS2 = doppler?.covarianceEcef,
                            relativeDisplacementEcef = displacement?.let { Triple(it.deltaX, it.deltaY, it.deltaZ) },
                            relativeDisplacementCovarianceM2 = displacement?.covarianceEcef,
                        )
                    } else {
                        if (commitState) interEpochKalman.reset()
                        wlsEcef
                    }

                val (lat, lon, alt) = GnssMathUtils.ecefToLatLonAlt(outX, outY, outZ)
                if (commitState) {
                    if (gpsTimeNanos > 0L) {
                        tdcp.rememberTdcp(
                            activeMeasurements,
                            outX,
                            outY,
                            outZ,
                            clockBiases,
                            clockGroupToIndex,
                            clockGroupBySatKey,
                            gpstSeconds,
                            gpsTimeNanos,
                        )
                    }
                    pppState.updatePosition(outX, outY, outZ, gpsTimeNanos)
                }

                val dopsAndAccuracy = computeDopsAndAccuracy(solutionWrapper.covarianceMatrix, dopMatrix, lat, lon, lastVarianceFactor, numClockStates)

                // Accepted fixes keep the WLS accuracy. A coasted epoch publishes the
                // prediction, so its accuracy is the predicted covariance, and at least
                // the horizontal gap to the measurement that was rejected.
                val coast = if (commitState) interEpochKalman.coastReport() else null
                val covHorizontalAccuracy: Float
                val covVerticalAccuracy: Float
                if (coast == null) {
                    covHorizontalAccuracy = dopsAndAccuracy[5].toFloat()
                    covVerticalAccuracy = dopsAndAccuracy[6].toFloat()
                } else {
                    val (varE, varN, varU) = rotateCovarianceToEnu(coast.positionCovariance, lat, lon)
                    val innovH =
                        GnssMathUtils.ecefDeltaHorizontalMeters(
                            coast.innovationX,
                            coast.innovationY,
                            coast.innovationZ,
                            lat,
                            lon,
                        )
                    covHorizontalAccuracy = max(sqrt(varE + varN), innovH).toFloat()
                    covVerticalAccuracy = sqrt(varU).toFloat()
                }

                val gpsTimeOfWeekSeconds =
                    if (gpsTimeNanos > 0L) {
                        (gpsTimeNanos / 1_000_000_000.0) % GnssMathUtils.SECONDS_IN_WEEK
                    } else {
                        (System.currentTimeMillis() / 1000.0 - GnssMathUtils.GPS_EPOCH_UNIX_SECONDS + GnssMathUtils.GPS_UTC_LEAP_SECONDS) % GnssMathUtils.SECONDS_IN_WEEK
                    }
                val ephemerisSigmaCeiling =
                    activeMeasurements.maxOfOrNull { m ->
                        val eph = ephemerisStore.get(m.constellationType to m.svid) ?: return@maxOfOrNull 0.0
                        if (ssr.getApplicableSsrPair(m.constellationType, m.svid, eph, usePpp, leapSeconds = ingest.lastKnownLeapSeconds) != null) {
                            return@maxOfOrNull SSR_ORBIT_CLOCK_SIGMA_METERS
                        }
                        val timeSeconds =
                            if (m.constellationType == GnssStatus.CONSTELLATION_BEIDOU) {
                                gpsTimeOfWeekSeconds - GnssMathUtils.BDT_GPST_OFFSET_SECONDS
                            } else {
                                gpsTimeOfWeekSeconds
                            }
                        GnssMathUtils.broadcastEphemerisRangeSigmaMeters(ephemerisStore.ephemerisAgeSeconds(eph, timeSeconds))
                    } ?: 0.0

                val horizontalAccuracyFloor = minHorizontalAccuracyMeters
                val verticalAccuracyFloor = minHorizontalAccuracyMeters
                // The ceiling only stops the clamp from hiding uncertainty already computed above.
                val coastHorizontal = if (coast == null) 0f else covHorizontalAccuracy
                val coastVertical = if (coast == null) 0f else covVerticalAccuracy
                val effectiveMaxHorizontal = maxHorizontalAccuracyMeters + ephemerisSigmaCeiling.toFloat() + coastHorizontal
                val effectiveMaxVertical =
                    maxHorizontalAccuracyMeters * GnssMathUtils.VERTICAL_ACCURACY_FACTOR +
                        ephemerisSigmaCeiling.toFloat() +
                        coastVertical

                val horizontalAcc = covHorizontalAccuracy.coerceIn(horizontalAccuracyFloor, effectiveMaxHorizontal)
                val verticalAcc = covVerticalAccuracy.coerceIn(verticalAccuracyFloor, effectiveMaxVertical)
                val speedAndBearing =
                    if (doppler == null) {
                        null
                    } else if (commitState) {
                        tdcp.speedAndBearingDegrees(lat, lon)
                    } else {
                        tdcp.speedAndBearingFromVelocityEcef(
                            doppler.velocityX,
                            doppler.velocityY,
                            doppler.velocityZ,
                            lat,
                            lon,
                        )
                    }

                return PositionResult(
                    latitude = lat,
                    longitude = lon,
                    altitude = alt,
                    horizontalAccuracyMeters = horizontalAcc,
                    verticalAccuracyMeters = verticalAcc,
                    timestamp = System.currentTimeMillis(),
                    gdop = dopsAndAccuracy[0],
                    pdop = dopsAndAccuracy[1],
                    hdop = dopsAndAccuracy[2],
                    vdop = dopsAndAccuracy[3],
                    tdop = dopsAndAccuracy[4],
                    residualsRms = lastResidualsRms,
                    // Distinct physical satellites observed this epoch. `measurements` holds one row
                    // per signal/frequency, so a dual-frequency SV (L1+L5) would otherwise be
                    // double-counted; collapse to unique (constellation, svid) pairs.
                    satelliteCount = measurements.distinctBy { it.constellationType to it.svid }.size,
                    usedInFixCount = activeMeasurements.size,
                    speedMetersPerSecond = speedAndBearing?.first,
                    bearingDegrees = speedAndBearing?.second,
                )
            } else {
                // Convergence failed
                Log.d(
                    TAG,
                    "WLS did not converge after $maxWlsIterations iterations — lastRms=${"%.1f".format(lastResidualsRms)}m maxRes=${"%.1f".format(maxResidual)}m sats=${activeMeasurements.size}",
                )
                break // Stop iterating, return null
            }
        }

        return null
    }

    companion object {
        private const val TAG = "GnssMeasurementProcessor"
        private const val COARSE_ECEF_RADIUS_SQ_THRESHOLD = 3.0e13
        private const val COARSE_POS_DELTA_THRESHOLD_METERS = 100_000.0
        private const val COARSE_SIGMA_ELEVATION_DEG = 45.0
        private const val SSR_ORBIT_CLOCK_SIGMA_METERS = 0.2
    }
}
