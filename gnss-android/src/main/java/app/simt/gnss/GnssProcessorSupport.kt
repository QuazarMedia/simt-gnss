package app.simt.gnss

import android.location.GnssMeasurement
import android.location.GnssNavigationMessage
import android.location.GnssStatus
import android.util.Log
import app.simt.gnss.core.model.Constellation
import app.simt.gnss.core.model.GnssSignalBand
import app.simt.gnss.core.model.SatelliteEphemeris
import app.simt.gnss.core.model.SatelliteMeasurement
import app.simt.gnss.core.utils.GnssMathUtils
import app.simt.gnss.core.utils.GnssMathUtils.SPEED_OF_LIGHT
import kotlin.math.*

/**
 * Shared helpers for the GNSS processor collaborators: band preference, clock-group
 * construction, DOP/accuracy derivation, observer sanitization, group-delay
 * conventions and constellation naming.
 */
private const val TAG = "GnssMeasurementProcessor"

private const val BEIDOU_BDS3_CLOCK_GROUP = -GnssStatus.CONSTELLATION_BEIDOU

/** Minimum reasonable pseudorange in meters (~15,000 km — low-elevation GEO/MEO). */
internal const val MIN_REASONABLE_PSEUDORANGE_METERS = 15_000_000.0

/** Maximum reasonable pseudorange in meters (~42,000 km — SBAS/QZSS GEO at low elevation). */
internal const val MAX_REASONABLE_PSEUDORANGE_METERS = 42_000_000.0

/**
* Lower clamp for travel time used during satellite-position / clock refinement. Travel
* times below ~50 ms would imply a physically impossible SV distance (< 15,000 km) and
* typically indicate a runaway receiver-clock estimate during early WLS iterations.
*/
internal const val MIN_TRAVEL_TIME_SECONDS =
    MIN_REASONABLE_PSEUDORANGE_METERS / SPEED_OF_LIGHT

/** Upper clamp for travel time (~117 ms, corresponding to ~35,000 km). */
internal const val MAX_TRAVEL_TIME_SECONDS =
    MAX_REASONABLE_PSEUDORANGE_METERS / SPEED_OF_LIGHT

internal const val PPP_SEED_MAX_AGE_NANOS = 30_000_000_000L
internal const val MAX_PPP_SEED_DEVIATION_METERS = 50_000.0
internal const val MIN_IF_FREQUENCY_SEPARATION_HZ = 250_000_000.0

internal class IfPairComponents(
    val fHighHz: Double,
    val fLowHz: Double,
    val sigmaAndroidHighMeters: Double,
    val cn0HighDbHz: Double,
    val smoothingHighEpochs: Int,
    val sigmaAndroidLowMeters: Double,
    val cn0LowDbHz: Double,
    val smoothingLowEpochs: Int,
)

internal fun navigationMessageConstellationType(type: Int): Int =
    when (type) {
        GnssNavigationMessage.TYPE_GPS_L1CA,
        GnssNavigationMessage.TYPE_GPS_L2CNAV,
        GnssNavigationMessage.TYPE_GPS_L5CNAV,
        -> GnssStatus.CONSTELLATION_GPS
        GnssNavigationMessage.TYPE_QZS_L1CA -> GnssStatus.CONSTELLATION_QZSS
        GnssNavigationMessage.TYPE_IRN_L5CA,
        GnssNavigationMessage.TYPE_IRN_L5,
        GnssNavigationMessage.TYPE_IRN_L1,
        -> GnssStatus.CONSTELLATION_IRNSS
        GnssNavigationMessage.TYPE_GAL_F,
        GnssNavigationMessage.TYPE_GAL_I,
        -> GnssStatus.CONSTELLATION_GALILEO
        GnssNavigationMessage.TYPE_GLO_L1CA -> GnssStatus.CONSTELLATION_GLONASS
        GnssNavigationMessage.TYPE_BDS_CNAV1,
        GnssNavigationMessage.TYPE_BDS_CNAV2,
        GnssNavigationMessage.TYPE_BDS_D1,
        GnssNavigationMessage.TYPE_BDS_D2,
        -> GnssStatus.CONSTELLATION_BEIDOU
        0x0101 -> GnssStatus.CONSTELLATION_SBAS // TYPE_SBAS_L1CA (not yet in all SDK versions)
        else -> {
            // Some chipsets report SBAS nav messages with constellation type = SBAS (2)
            // but use an unrecognized message type. Check if the SVID falls in the SBAS
            // PRN range (120–158) and treat as SBAS.
            GnssStatus.CONSTELLATION_UNKNOWN
        }
    }

internal fun sanitizeObserverLatitudeDegrees(
    candidateLatitudeDegrees: Double,
    fallbackLatitudeDegrees: Double,
): Double {
    val fallback = if (fallbackLatitudeDegrees.isFinite()) fallbackLatitudeDegrees.coerceIn(-90.0, 90.0) else 0.0
    return if (candidateLatitudeDegrees.isFinite()) {
        candidateLatitudeDegrees.coerceIn(-90.0, 90.0)
    } else {
        fallback
    }
}

internal fun sanitizeObserverLongitudeDegrees(
    candidateLongitudeDegrees: Double,
    fallbackLongitudeDegrees: Double,
): Double {
    val rawLongitude = if (candidateLongitudeDegrees.isFinite()) candidateLongitudeDegrees else fallbackLongitudeDegrees
    val normalizedLongitude = ((rawLongitude + 180.0) % 360.0 + 360.0) % 360.0 - 180.0
    return if (normalizedLongitude == -180.0) 180.0 else normalizedLongitude
}

internal fun sanitizeObserverAltitudeMeters(
    candidateAltitudeMeters: Double,
    fallbackAltitudeMeters: Double,
): Double {
    val fallback = if (fallbackAltitudeMeters.isFinite()) fallbackAltitudeMeters.coerceIn(-500.0, 20_000.0) else 0.0
    return if (candidateAltitudeMeters.isFinite()) {
        candidateAltitudeMeters.coerceIn(-500.0, 20_000.0)
    } else {
        fallback
    }
}

internal fun hasValidAdr(state: Int): Boolean {
    return (state and GnssMeasurement.ADR_STATE_VALID) != 0 &&
        (state and GnssMeasurement.ADR_STATE_CYCLE_SLIP) == 0 &&
        (state and GnssMeasurement.ADR_STATE_RESET) == 0
}

/**
 * True when the chipset reports the carrier half-cycle ambiguity status AND it is not
 * resolved: the accumulated phase may be offset by λ/2, so the carrier must not feed
 * integer ambiguity fixing. Chipsets that don't report the status (HALF_CYCLE_REPORTED
 * unset, incl. pre-API-28 HALs that never set either bit) are accepted — rejecting them
 * would disable RTK fixing on the whole legacy fleet.
 */
internal fun isCarrierHalfCycleAmbiguous(adrState: Int): Boolean =
    (adrState and GnssMeasurement.ADR_STATE_HALF_CYCLE_REPORTED) != 0 &&
        (adrState and GnssMeasurement.ADR_STATE_HALF_CYCLE_RESOLVED) == 0

/**
 * Group delay subtracted from the broadcast clock, in seconds.
 *
 * GPS and BeiDou keep the previous convention: no delay on an LNAV/BGD ionosphere-free
 * clock, plain TGD for BeiDou and for an IF row, and γ·TGD on a single frequency.
 * Galileo follows OS SIS ICD §5.1.5. An I/NAV clock on the E1/E5a ionosphere-free
 * path still adjusts, by BGD(E1,E5b) − BGD(E1,E5a), because that offset does not
 * cancel in the combination.
 */
internal fun measurementGroupDelaySeconds(
    ephemeris: SatelliteEphemeris,
    constellationType: Int,
    carrierFrequencyHz: Double,
    ionosphereFree: Boolean,
    clockExcludesGroupDelay: Boolean,
    lowerBandHz: Double = GnssMathUtils.L5_FREQUENCY_HZ,
): Double {
    if (clockExcludesGroupDelay) return 0.0
    if (constellationType == GnssStatus.CONSTELLATION_GALILEO) {
        return galileoSignalGroupDelaySeconds(ephemeris, carrierFrequencyHz, ionosphereFree, lowerBandHz)
    }
    if (constellationType == GnssStatus.CONSTELLATION_BEIDOU) {
        if (ionosphereFree) return 0.0
        return when (GnssSignalBand.fromFrequencyHz(carrierFrequencyHz, Constellation.BEIDOU)) {
            GnssSignalBand.B1I -> if (ephemeris.tgd.isFinite()) ephemeris.tgd else 0.0
            GnssSignalBand.E5B_B2I -> if (ephemeris.beidouTgd2Seconds.isFinite()) ephemeris.beidouTgd2Seconds else 0.0
            else -> 0.0
        }
    }
    val tgdSeconds = ephemeris.tgd
    if (!tgdSeconds.isFinite() || tgdSeconds == 0.0) return 0.0
    if (ionosphereFree || carrierFrequencyHz <= 0.0) {
        return tgdSeconds
    }
    return tgdSeconds * (GnssMathUtils.L1_FREQUENCY_HZ / carrierFrequencyHz).pow(2)
}

/**
 * Galileo OS SIS ICD §5.1.5. γ = (f_E1 / f)².
 * E1 uses the clock's own BGD. E5a with an F/NAV clock is γ·BGD(E1,E5a); with an
 * I/NAV clock it is BGD(E1,E5b) + (γ − 1)·BGD(E1,E5a). E5b with an I/NAV clock is
 * γ·BGD(E1,E5b); with an F/NAV clock it is BGD(E1,E5a) + (γ − 1)·BGD(E1,E5b).
 * The E1/E5a ionosphere-free combination is 0 for F/NAV and BGD(E1,E5b) − BGD(E1,E5a)
 * for I/NAV; E1/E5b is 0 for I/NAV and BGD(E1,E5a) − BGD(E1,E5b) for F/NAV.
 * A record that only carries [SatelliteEphemeris.tgd] is treated as F/NAV BGD(E1,E5a).
 */
internal fun galileoSignalGroupDelaySeconds(
    ephemeris: SatelliteEphemeris,
    carrierFrequencyHz: Double,
    ionosphereFree: Boolean,
    lowerBandHz: Double = GnssMathUtils.L5_FREQUENCY_HZ,
): Double {
    val legacy =
        !ephemeris.galileoInavClock &&
            ephemeris.galileoBgdE1E5aSeconds == 0.0 &&
            ephemeris.galileoBgdE1E5bSeconds == 0.0
    val bgdA = if (legacy) ephemeris.tgd else ephemeris.galileoBgdE1E5aSeconds
    val bgdB = if (legacy) ephemeris.tgd else ephemeris.galileoBgdE1E5bSeconds
    if (!bgdA.isFinite() || !bgdB.isFinite()) return 0.0
    if (ionosphereFree) {
        val lowerBand = GnssSignalBand.fromFrequencyHz(lowerBandHz, Constellation.GALILEO)
        return if (lowerBand == GnssSignalBand.E5B_B2I) {
            if (ephemeris.galileoInavClock) 0.0 else bgdA - bgdB
        } else {
            if (ephemeris.galileoInavClock) bgdB - bgdA else 0.0
        }
    }
    val band = GnssSignalBand.fromFrequencyHz(carrierFrequencyHz, Constellation.GALILEO)
    val gamma =
        if (carrierFrequencyHz > 0.0) {
            (GnssMathUtils.L1_FREQUENCY_HZ / carrierFrequencyHz).pow(2)
        } else {
            1.0
        }
    return when (band) {
        GnssSignalBand.L5_E5A_B2A ->
            if (ephemeris.galileoInavClock) bgdB + (gamma - 1.0) * bgdA else gamma * bgdA
        GnssSignalBand.E5B_B2I ->
            if (ephemeris.galileoInavClock) gamma * bgdB else bgdA + (gamma - 1.0) * bgdB
        else ->
            (if (ephemeris.galileoInavClock) bgdB else bgdA) * gamma
    }
}

internal fun effectiveSignalBand(meas: SatelliteMeasurement): GnssSignalBand {
    val fromFreq = GnssSignalBand.fromFrequencyHz(meas.carrierFrequencyHz, meas.constellationType)
    return if (fromFreq != GnssSignalBand.UNKNOWN) fromFreq else meas.signalBand
}

internal fun upperBandPreferenceRank(
    constellationType: Int,
    band: GnssSignalBand,
): Int =
    if (constellationType == GnssStatus.CONSTELLATION_BEIDOU) {
        when (band) {
            GnssSignalBand.B1I -> 0
            GnssSignalBand.L1_E1_B1C -> 1
            else -> Int.MAX_VALUE
        }
    } else {
        when (band) {
            GnssSignalBand.L1_E1_B1C -> 0
            GnssSignalBand.B1I -> 1
            else -> Int.MAX_VALUE
        }
    }

internal fun lowerBandPreferenceRank(band: GnssSignalBand): Int =
    when (band) {
        GnssSignalBand.L5_E5A_B2A -> 0
        GnssSignalBand.E5B_B2I -> 1
        GnssSignalBand.L2 -> 2
        GnssSignalBand.B3I -> 3
        else -> Int.MAX_VALUE
    }

internal fun pruneSingleSatelliteConstellations(measurements: MutableList<SatelliteMeasurement>) {
    val countsByConstellation = measurements.groupingBy { it.constellationType }.eachCount()
    measurements.removeAll { (countsByConstellation[it.constellationType] ?: 0) < 2 }
}

internal fun isUncalibratedBeidouBds3(
    meas: SatelliteMeasurement,
    ifPair: IfPairComponents?,
    ssrCodeBiasApplied: Boolean = false,
): Boolean {
    if (ssrCodeBiasApplied) return false
    if (meas.constellationType != GnssStatus.CONSTELLATION_BEIDOU) return false
    if (ifPair != null) {
        val upperBand = GnssSignalBand.fromFrequencyHz(ifPair.fHighHz, Constellation.BEIDOU)
        val lowerBand = GnssSignalBand.fromFrequencyHz(ifPair.fLowHz, Constellation.BEIDOU)
        return upperBand == GnssSignalBand.L1_E1_B1C || lowerBand == GnssSignalBand.L5_E5A_B2A
    }
    val band = effectiveSignalBand(meas)
    return band == GnssSignalBand.L1_E1_B1C || band == GnssSignalBand.L5_E5A_B2A
}

internal fun buildClockGroupBySatKey(
    measurements: List<SatelliteMeasurement>,
    ifPairComponentsBySat: Map<Pair<Int, Int>, IfPairComponents>,
    ssrCodeBiasAppliedSatKeys: Set<Pair<Int, Int>> = emptySet(),
): Map<Pair<Int, Int>, Int> {
    var beidouCalibratedCount = 0
    var beidouUncalibratedCount = 0
    for (meas in measurements) {
        if (meas.constellationType != GnssStatus.CONSTELLATION_BEIDOU) continue
        val satKey = meas.constellationType to meas.svid
        if (isUncalibratedBeidouBds3(meas, ifPairComponentsBySat[satKey], satKey in ssrCodeBiasAppliedSatKeys)) {
            beidouUncalibratedCount++
        } else {
            beidouCalibratedCount++
        }
    }
    val splitBeidouClock = beidouCalibratedCount >= 2 && beidouUncalibratedCount >= 2
    return measurements.associate { meas ->
        val satKey = meas.constellationType to meas.svid
        val group =
            if (splitBeidouClock && isUncalibratedBeidouBds3(meas, ifPairComponentsBySat[satKey], satKey in ssrCodeBiasAppliedSatKeys)) {
                BEIDOU_BDS3_CLOCK_GROUP
            } else {
                meas.constellationType
            }
        satKey to group
    }
}

/**
 * Computes DOP values and covariance-derived accuracy.
 * Returns [gdop, pdop, hdop, vdop, tdop, horizontalAccuracyMeters, verticalAccuracyMeters].
 *
 * DOPs come from the UNWEIGHTED geometry inverse [dopMatrix] (dimensionless, classical
 * definition); accuracy comes from the metric weighted covariance scaled by the
 * a-posteriori variance factor. When the geometry inverse is unavailable (degenerate
 * normal matrix) the DOPs are reported as 0 rather than substituting metric sigmas.
 */
internal fun computeDopsAndAccuracy(
    weightedCovariance: Array<DoubleArray>,
    dopMatrix: Array<DoubleArray>?,
    lat: Double,
    lon: Double,
    aPosterioriVarianceFactor: Double,
    numClockStates: Int = 1,
): DoubleArray {
    var gdop = 0.0
    var pdop = 0.0
    var hdop = 0.0
    var vdop = 0.0
    var tdop = 0.0
    if (dopMatrix != null) {
        pdop = sqrt(dopMatrix[0][0] + dopMatrix[1][1] + dopMatrix[2][2])
        var clockTraceSum = 0.0
        for (k in 3 until (3 + numClockStates).coerceAtMost(dopMatrix.size)) {
            clockTraceSum += dopMatrix[k][k]
        }
        tdop = sqrt(clockTraceSum)
        gdop = sqrt(dopMatrix[0][0] + dopMatrix[1][1] + dopMatrix[2][2] + clockTraceSum)
        val (dopVarE, dopVarN, dopVarU) = rotateCovarianceToEnu(dopMatrix, lat, lon)
        hdop = sqrt(dopVarE + dopVarN)
        vdop = sqrt(dopVarU)
    }

    // Accuracy = sqrt(varianceFactor) x formal sigma. The WLS weights are 1/sigma^2, so
    // the weighted covariance is the metric formal covariance in m^2; the dimensionless
    // a-posteriori factor (>= 1) scales it for the realized fit.
    val (varE, varN, varU) = rotateCovarianceToEnu(weightedCovariance, lat, lon)
    val sigma0 = sqrt(aPosterioriVarianceFactor)
    val horizontalAccuracy = sigma0 * sqrt(varE + varN)
    val verticalAccuracy = sigma0 * sqrt(varU)

    return doubleArrayOf(gdop, pdop, hdop, vdop, tdop, horizontalAccuracy, verticalAccuracy)
}

/** Rotates the position 3x3 block of [covariance] into ENU; returns (varE, varN, varU). */
internal fun rotateCovarianceToEnu(
    covariance: Array<DoubleArray>,
    lat: Double,
    lon: Double,
): Triple<Double, Double, Double> {
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
    val enu = DoubleArray(3)
    for (i in 0 until 3) {
        var sum = 0.0
        for (k in 0 until 3) {
            for (l in 0 until 3) {
                sum += r[i][k] * covariance[k][l] * r[i][l]
            }
        }
        enu[i] = sum.coerceAtLeast(0.0)
    }
    return Triple(enu[0], enu[1], enu[2])
}

internal fun getConstellationName(type: Int): String =
    when (type) {
        GnssStatus.CONSTELLATION_GPS -> "GPS"
        GnssStatus.CONSTELLATION_GLONASS -> "GLONASS"
        GnssStatus.CONSTELLATION_GALILEO -> "GALILEO"
        GnssStatus.CONSTELLATION_BEIDOU -> "BEIDOU"
        GnssStatus.CONSTELLATION_QZSS -> "QZSS"
        GnssStatus.CONSTELLATION_IRNSS -> "NAVIC"
        GnssStatus.CONSTELLATION_SBAS -> "SBAS"
        else -> "UNKNOWN"
    }

/**
 * Cached per-SV model terms reused across WLS iterations (see [GnssMathUtils.WLS_MODEL_REUSE_DISTANCE_METERS]).
 * [refX]/[refY]/[refZ]/[refClockBias] record the receiver state at which the terms were last
 * evaluated; the cache is consulted only while the receiver estimate stays within the reuse radius.
 */
internal class SvModelTerms(
    val refX: Double,
    val refY: Double,
    val refZ: Double,
    val refClockBias: Double,
    val satX: Double,
    val satY: Double,
    val satZ: Double,
    val satClockMeters: Double,
    val ionoDelayL1: Double,
    val dryDelay: Double,
    val elevationDeg: Double,
    val azimuthDeg: Double,
)

/**
 * Initial filtering and IF combination: builds the RAIM-iteration working set
 * from the raw per-SV measurement groups (ephemeris gating, SSR code biases,
 * IF pseudorange combination). Mutates the caller's five working collections;
 * mechanical move of the former inline body of [GnssAbsoluteSolver.solve].
 */
internal fun initialFilteringAndIfCombination(
    groupedBySv: Map<Pair<Int, Int>, List<SatelliteMeasurement>>,
    usableEphemerisBySat: Map<Pair<Int, Int>, SatelliteEphemeris>,
    ssr: SsrCorrectionPipeline,
    ingest: GnssNavMessageIngest,
    usePpp: Boolean,
    activeMeasurements: MutableList<SatelliteMeasurement>,
    ifCorrectedSatKeys: MutableSet<Pair<Int, Int>>,
    ifClockWithoutGroupDelaySatKeys: MutableSet<Pair<Int, Int>>,
    ssrCodeBiasAppliedSatKeys: MutableSet<Pair<Int, Int>>,
    ifPairComponentsBySat: HashMap<Pair<Int, Int>, IfPairComponents>,
) {
    for ((satKey, svMeasurements) in groupedBySv) {
        val ephemerisForSv = usableEphemerisBySat[satKey]
        val ssrPairForSv =
            ephemerisForSv?.let {
                ssr.getApplicableSsrPair(satKey.first, satKey.second, it, usePpp, leapSeconds = ingest.lastKnownLeapSeconds)
            }
        val upperCandidates =
            svMeasurements
                .filter { m ->
                    m.carrierFrequencyHz.isFinite() &&
                        m.carrierFrequencyHz > 0.0 &&
                        effectiveSignalBand(m).isUpperBand
                }.sortedWith(
                    compareBy<SatelliteMeasurement>(
                        { upperBandPreferenceRank(satKey.first, effectiveSignalBand(it)) },
                        { -it.cn0DbHz },
                    ),
                )
        val lowerCandidates =
            svMeasurements
                .filter { m ->
                    m.carrierFrequencyHz.isFinite() &&
                        m.carrierFrequencyHz > 0.0 &&
                        effectiveSignalBand(m).isLowerBand
                }.sortedWith(
                    compareBy<SatelliteMeasurement>(
                        { lowerBandPreferenceRank(effectiveSignalBand(it)) },
                        { -it.cn0DbHz },
                    ),
                )

        var selectedPair: Pair<SatelliteMeasurement, SatelliteMeasurement>? = null
        for (upper in upperCandidates) {
            val lower =
                lowerCandidates.firstOrNull { cand ->
                    abs(upper.carrierFrequencyHz - cand.carrierFrequencyHz) >= MIN_IF_FREQUENCY_SEPARATION_HZ
                }
            if (lower != null) {
                selectedPair = upper to lower
                break
            }
        }

        if (selectedPair != null) {
            val (upper, lower) = selectedPair
            val f1 = upper.carrierFrequencyHz
            val f2 = lower.carrierFrequencyHz

            val bHigh =
                if (ssrPairForSv != null) {
                    ssr.store.getCodeBiasForBand(satKey.first, satKey.second, effectiveSignalBand(upper))
                } else {
                    null
                }
            val bLow =
                if (ssrPairForSv != null) {
                    ssr.store.getCodeBiasForBand(satKey.first, satKey.second, effectiveSignalBand(lower))
                } else {
                    null
                }

            val l1CodeForIf: Double
            val l5CodeForIf: Double
            if (bHigh != null && bLow != null) {
                // RTCM 10403.3 §3.5.12.3 (DF383): PR_corr = PR_measured + CodeBias
                l1CodeForIf = upper.pseudorangeMeters + bHigh.bias
                l5CodeForIf = lower.pseudorangeMeters + bLow.bias
                ifClockWithoutGroupDelaySatKeys.add(satKey)
                ssrCodeBiasAppliedSatKeys.add(satKey)
            } else {
                val ifAdjustment =
                    ingest.codePseudorangesForIonosphereFreeMeters(
                        satKey = satKey,
                        l1Meters = upper.pseudorangeMeters,
                        l5Meters = lower.pseudorangeMeters,
                        ephemeris = ephemerisForSv,
                        upperBandHz = f1,
                        lowerBandHz = f2,
                    )
                l1CodeForIf = ifAdjustment.l1Meters
                l5CodeForIf = ifAdjustment.l5Meters
                if (!ifAdjustment.clockIncludesGroupDelay) {
                    ifClockWithoutGroupDelaySatKeys.add(satKey)
                }
            }

            // Pseudorange IF Combination
            val f1Sq = f1 * f1
            val f2Sq = f2 * f2
            val denom = f1Sq - f2Sq
            val prIf = (f1Sq * l1CodeForIf - f2Sq * l5CodeForIf) / denom

            // Carrier Phase IF Combination (if available)
            val cpIf =
                upper.carrierPhaseMeters?.let { cp1 ->
                    lower.carrierPhaseMeters?.let { cp2 ->
                        (f1Sq * cp1 - f2Sq * cp2) / denom
                    }
                }

            val sigma1 = upper.pseudorangeUncertaintyMeters
            val sigma2 = lower.pseudorangeUncertaintyMeters
            val sigmaIf = sqrt((f1Sq * f1Sq * sigma1 * sigma1 + f2Sq * f2Sq * sigma2 * sigma2) / (denom * denom))

            ifPairComponentsBySat[satKey] =
                IfPairComponents(
                    fHighHz = f1,
                    fLowHz = f2,
                    sigmaAndroidHighMeters = upper.pseudorangeUncertaintyMeters,
                    cn0HighDbHz = upper.cn0DbHz,
                    smoothingHighEpochs = upper.smoothingEpochs,
                    sigmaAndroidLowMeters = lower.pseudorangeUncertaintyMeters,
                    cn0LowDbHz = lower.cn0DbHz,
                    smoothingLowEpochs = lower.smoothingEpochs,
                )

            activeMeasurements.add(
                upper.copy(
                    pseudorangeMeters = prIf,
                    carrierPhaseMeters = cpIf,
                    pseudorangeUncertaintyMeters = sigmaIf,
                ),
            )
            ifCorrectedSatKeys.add(satKey)
        } else {
            val singleMeas = upperCandidates.firstOrNull() ?: lowerCandidates.firstOrNull() ?: svMeasurements.first()
            val bSingle =
                if (ssrPairForSv != null) {
                    ssr.store.getCodeBiasForBand(satKey.first, satKey.second, effectiveSignalBand(singleMeas))
                } else {
                    null
                }
            if (bSingle != null) {
                ssrCodeBiasAppliedSatKeys.add(satKey)
                activeMeasurements.add(singleMeas.copy(pseudorangeMeters = singleMeas.pseudorangeMeters + bSingle.bias))
            } else {
                activeMeasurements.add(singleMeas)
            }
        }
    }
}

/**
 * Post-IF pruning: drops measurements without usable ephemeris and single-
 * satellite constellations, logging removals. Returns false when fewer than 4
 * usable measurements remain (caller returns null). Mechanical move of the
 * former inline body of [GnssAbsoluteSolver.solve].
 */
internal fun pruneMeasurementsWithoutEphemeris(
    activeMeasurements: MutableList<SatelliteMeasurement>,
    usableEphemerisBySat: Map<Pair<Int, Int>, SatelliteEphemeris>,
): Boolean {
    // Remove measurements that lack usable ephemeris — without satellite clock corrections
    // (~100–500 km per SV), these measurements destroy the WLS solution.
    // Uses extended validity (24h Keplerian / 6h GLONASS) so positioning works offline.
    val preEphemerisCount = activeMeasurements.size
    activeMeasurements.removeAll { measurement ->
        (measurement.constellationType to measurement.svid) !in usableEphemerisBySat
    }
    if (activeMeasurements.size < preEphemerisCount) {
        Log.d(TAG, "Filtered ${preEphemerisCount - activeMeasurements.size} measurements without ephemeris, kept ${activeMeasurements.size}")
    }
    pruneSingleSatelliteConstellations(activeMeasurements)
    if (activeMeasurements.size < 4) {
        Log.d(TAG, "Not enough measurements with ephemeris (${activeMeasurements.size}/4 minimum) — cannot solve")
        return false
    }
    return true
}
