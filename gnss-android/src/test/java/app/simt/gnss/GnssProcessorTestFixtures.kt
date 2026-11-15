package app.simt.gnss

import android.location.GnssStatus
import android.os.SystemClock
import app.simt.gnss.core.correction.AtmosphericCorrection
import app.simt.gnss.core.model.GnssSignalBand
import app.simt.gnss.core.model.SatelliteEphemeris
import app.simt.gnss.core.model.SatelliteMeasurement
import app.simt.gnss.core.orbit.SatellitePositionCalculator
import app.simt.gnss.core.solver.GnssPositionSolver
import app.simt.gnss.core.utils.GnssMathUtils
import org.junit.Assert.*
import kotlin.math.pow
import kotlin.math.sqrt

/**
 * Shared synthetic-measurement builders for the GnssMeasurementProcessor test
 * family. Extracted verbatim from GnssMeasurementProcessorTest (they were its
 * private helpers); [computePositionWithAccuracyFallback] takes the processor
 * under test explicitly since it no longer closes over a class field.
 */

internal const val SYNTHETIC_GPS_TIME_NANOS = 345_600_000_000_000L

internal fun computePositionWithAccuracyFallback(
    processor: GnssMeasurementProcessor,
    measurements: List<SatelliteMeasurement>,
    initialLat: Double,
    initialLon: Double,
    initialAlt: Double,
    usePpp: Boolean = false,
) = processor.computePosition(
    measurements = measurements,
    initialLat = initialLat,
    initialLon = initialLon,
    initialAlt = initialAlt,
    usePpp = usePpp,
    gpsTimeNanos = SYNTHETIC_GPS_TIME_NANOS,
    accuracy = GnssPositionSolver.Accuracy.HIGH,
) ?: processor.computePosition(
    measurements = measurements,
    initialLat = initialLat,
    initialLon = initialLon,
    initialAlt = initialAlt,
    usePpp = usePpp,
    gpsTimeNanos = SYNTHETIC_GPS_TIME_NANOS,
    accuracy = GnssPositionSolver.Accuracy.MEDIUM,
)

/**
 * Creates a synthetic measurement consistent with SatellitePositionCalculator.getSatellitePositionApproximation.
 * The pseudorange is computed from the approximate satellite position to the given receiver ECEF position.
 */
internal fun createConsistentMeasurement(
    svid: Int,
    elevDeg: Double,
    azDeg: Double,
    recX: Double,
    recY: Double,
    recZ: Double,
    observerLat: Double,
    observerLon: Double,
    observerAlt: Double,
    clockBiasMeters: Double = 0.0,
    noiseMeters: Double = 0.0,
): SatelliteMeasurement {
    val (satX, satY, satZ) =
        SatellitePositionCalculator.getSatellitePositionApproximation(
            GnssStatus.CONSTELLATION_GPS,
            elevDeg,
            azDeg,
            observerLat,
            observerLon,
            observerAlt,
        )
    val range = sqrt((satX - recX) * (satX - recX) + (satY - recY) * (satY - recY) + (satZ - recZ) * (satZ - recZ))
    return SatelliteMeasurement(
        constellationType = GnssStatus.CONSTELLATION_GPS,
        svid = svid,
        pseudorangeMeters = range + clockBiasMeters + noiseMeters,
        carrierFrequencyHz = 1.57542e9,
        cn0DbHz = 40.0,
        elevationDegrees = elevDeg,
        azimuthDegrees = azDeg,
        accumulatedDeltaRangeMeters = null,
        pseudorangeUncertaintyMeters = 1.0,
        isL5 = false,
    )
}

internal fun buildSyntheticSkyEpoch(
    constellationType: Int,
    lat: Double,
    lon: Double,
    alt: Double,
    gpsTimeNanos: Long,
    carrierFrequencyHz: Double = GnssMathUtils.L1_FREQUENCY_HZ,
    clockBiasMeters: Double = 0.0,
    maxSatellites: Int = Int.MAX_VALUE,
    applyAtmosphericAndRelativistic: Boolean = true,
): Pair<List<SatelliteEphemeris>, List<SatelliteMeasurement>> {
    val gpstTow = (gpsTimeNanos / 1e9) % GnssMathUtils.SECONDS_IN_WEEK
    val toe =
        if (constellationType == GnssStatus.CONSTELLATION_BEIDOU) {
            gpstTow - GnssMathUtils.BDT_GPST_OFFSET_SECONDS
        } else {
            gpstTow
        }
    val (recX, recY, recZ) = GnssMathUtils.latLonAltToEcef(lat, lon, alt)
    val receipt = SystemClock.elapsedRealtime()
    val unixSeconds = GnssMathUtils.GPS_EPOCH_UNIX_SECONDS + (gpsTimeNanos / 1_000_000_000L)
    val utcCalendar = java.util.Calendar.getInstance(java.util.TimeZone.getTimeZone("UTC"))
    utcCalendar.timeInMillis = unixSeconds * 1000L
    val dayOfYear = utcCalendar.get(java.util.Calendar.DAY_OF_YEAR)

    val ephemerides = ArrayList<SatelliteEphemeris>()
    val measurements = ArrayList<SatelliteMeasurement>()
    var svid = 1
    for (raanDeg in 0 until 360 step 30) {
        for (meanDeg in listOf(0.0, 90.0, 180.0, 270.0)) {
            val eph =
                SatelliteEphemeris(
                    constellationType = constellationType,
                    svid = svid,
                    toe = toe,
                    toc = toe,
                    sqrtA = sqrt(26_560_000.0),
                    eccentricity = 0.0,
                    i0 = Math.toRadians(55.0),
                    omega0 = Math.toRadians(raanDeg.toDouble()),
                    omega = 0.0,
                    m0 = Math.toRadians(meanDeg),
                    lastUpdateTime = receipt,
                )
            val rough = SatellitePositionCalculator.calculateSatellitePosition(eph, toe)
            val roughRange =
                sqrt(
                    (rough.first - recX) * (rough.first - recX) +
                        (rough.second - recY) * (rough.second - recY) +
                        (rough.third - recZ) * (rough.third - recZ),
                )
            val flight = roughRange / GnssMathUtils.SPEED_OF_LIGHT
            val transmit = SatellitePositionCalculator.calculateSatellitePosition(eph, toe - flight)
            val rotated =
                SatellitePositionCalculator.rotateEcefForEarthRotationDuringTransit(
                    transmit.first,
                    transmit.second,
                    transmit.third,
                    flight,
                )
            val (elev, az) =
                GnssMathUtils.ecefLosToElevationAzimuth(
                    receiverX = recX,
                    receiverY = recY,
                    receiverZ = recZ,
                    receiverLatDeg = lat,
                    receiverLonDeg = lon,
                    satelliteX = rotated.first,
                    satelliteY = rotated.second,
                    satelliteZ = rotated.third,
                )
            if (elev >= 20.0 && measurements.size < maxSatellites) {
                val range =
                    sqrt(
                        (rotated.first - recX) * (rotated.first - recX) +
                            (rotated.second - recY) * (rotated.second - recY) +
                            (rotated.third - recZ) * (rotated.third - recZ),
                    )
                val dryDelay =
                    if (applyAtmosphericAndRelativistic) {
                        AtmosphericCorrection.computeTroposphericDelay(elev, alt, lat, dayOfYear)
                    } else {
                        0.0
                    }
                val ionoDelayL1 =
                    if (applyAtmosphericAndRelativistic) {
                        AtmosphericCorrection.computeIonosphericDelay(elev, az, lat, lon, gpsTimeNanos / 1_000_000L, null, null)
                    } else {
                        0.0
                    }
                val ionoDelay = ionoDelayL1 * (GnssMathUtils.L1_FREQUENCY_HZ / carrierFrequencyHz).pow(2)
                val relCorr =
                    if (applyAtmosphericAndRelativistic) {
                        AtmosphericCorrection.computeRelativisticCorrection(recX, recY, recZ, rotated.first, rotated.second, rotated.third)
                    } else {
                        0.0
                    }
                ephemerides += eph
                measurements +=
                    SatelliteMeasurement(
                        constellationType = constellationType,
                        svid = svid,
                        pseudorangeMeters = range + clockBiasMeters + dryDelay + ionoDelay + relCorr,
                        carrierFrequencyHz = carrierFrequencyHz,
                        cn0DbHz = 42.0,
                        elevationDegrees = elev,
                        azimuthDegrees = az,
                        accumulatedDeltaRangeMeters = null,
                        pseudorangeUncertaintyMeters = 0.5,
                        isL5 = GnssSignalBand.fromFrequencyHz(carrierFrequencyHz, constellationType) == GnssSignalBand.L5_E5A_B2A,
                    )
            }
            svid++
        }
    }
    return ephemerides to measurements
}

internal fun ecefDistanceMeters(
    lat1: Double,
    lon1: Double,
    alt1: Double,
    lat2: Double,
    lon2: Double,
    alt2: Double,
): Double {
    val a = GnssMathUtils.latLonAltToEcef(lat1, lon1, alt1)
    val b = GnssMathUtils.latLonAltToEcef(lat2, lon2, alt2)
    val dx = a.first - b.first
    val dy = a.second - b.second
    val dz = a.third - b.third
    return sqrt(dx * dx + dy * dy + dz * dz)
}
