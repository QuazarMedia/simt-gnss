package app.simt.gnss

import android.location.GnssStatus
import android.os.SystemClock
import app.simt.gnss.core.correction.AtmosphericCorrection
import app.simt.gnss.core.model.AdrState
import app.simt.gnss.core.model.GnssSolutionMode
import app.simt.gnss.core.model.SatelliteEphemeris
import app.simt.gnss.core.model.SatelliteMeasurement
import app.simt.gnss.core.orbit.SatellitePositionCalculator
import app.simt.gnss.core.parser.RtcmMsmParser
import app.simt.gnss.core.utils.GnssMathUtils
import org.junit.Assert.*
import org.junit.Before
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sqrt

@RunWith(RobolectricTestRunner::class)
class GnssRtkProcessorTest {
    private lateinit var processor: GnssMeasurementProcessor

    @Before
    fun setUp() {
        processor = GnssMeasurementProcessor()
    }

    private data class RtkFixture(
        val processor: GnssMeasurementProcessor,
        val roverMeasurements: List<SatelliteMeasurement>,
        val roverLat: Double,
        val roverLon: Double,
        val roverAlt: Double,
        val gpsTimeNanos: Long,
    )

    private fun buildRtkFixture(
        baseLat: Double,
        baseLon: Double,
        baseAlt: Double,
        roverLat: Double,
        roverLon: Double,
        roverAlt: Double,
        constellationType: Int = GnssStatus.CONSTELLATION_GPS,
        carrierFrequencyHz: Double = 1_575_420_000.0,
        gpsTimeNanos: Long = SYNTHETIC_GPS_TIME_NANOS,
        baseEpochOffsetSeconds: Double = 0.0,
        includeTroposphere: Boolean = false,
        includeIonosphere: Boolean = false,
        includeCarrier: Boolean = true,
        maxSatellites: Int = 24,
    ): RtkFixture {
        val rtkProcessor = GnssMeasurementProcessor()
        val (baseX, baseY, baseZ) = GnssMathUtils.latLonAltToEcef(baseLat, baseLon, baseAlt)
        val (roverX, roverY, roverZ) = GnssMathUtils.latLonAltToEcef(roverLat, roverLon, roverAlt)
        val gpstSeconds = (gpsTimeNanos / 1e9) % GnssMathUtils.SECONDS_IN_WEEK
        val constellationTimeSeconds =
            if (constellationType == GnssStatus.CONSTELLATION_BEIDOU) {
                (gpstSeconds - GnssMathUtils.BDT_GPST_OFFSET_SECONDS).mod(GnssMathUtils.SECONDS_IN_WEEK)
            } else {
                gpstSeconds
            }
        val baseTimeSeconds = (constellationTimeSeconds + baseEpochOffsetSeconds).mod(GnssMathUtils.SECONDS_IN_WEEK)
        val dayOfYear =
            run {
                val unixSeconds = GnssMathUtils.GPS_EPOCH_UNIX_SECONDS + (gpsTimeNanos / 1_000_000_000L)
                val utcCalendar = java.util.Calendar.getInstance(java.util.TimeZone.getTimeZone("UTC"))
                utcCalendar.timeInMillis = unixSeconds * 1000L
                utcCalendar.get(java.util.Calendar.DAY_OF_YEAR)
            }
        val gpsTimeMs = gpsTimeNanos / 1_000_000L
        val receipt = SystemClock.elapsedRealtime()
        val ephemerides = ArrayList<SatelliteEphemeris>()
        var svid = 1
        for (raanDeg in 0 until 360 step 30) {
            for (meanDeg in listOf(0.0, 90.0, 180.0, 270.0)) {
                ephemerides +=
                    SatelliteEphemeris(
                        constellationType = constellationType,
                        svid = svid,
                        toe = constellationTimeSeconds,
                        toc = constellationTimeSeconds,
                        sqrtA = sqrt(26_560_000.0),
                        eccentricity = 0.0,
                        i0 = Math.toRadians(55.0),
                        omega0 = Math.toRadians(raanDeg.toDouble()),
                        omega = 0.0,
                        m0 = Math.toRadians(meanDeg),
                        lastUpdateTime = receipt,
                    )
                svid++
            }
        }
        rtkProcessor.importExternalEphemeris(ephemerides, gpstSeconds)
        rtkProcessor.rtkBaseStore.setBasePosition(baseX, baseY, baseZ, referenceStationId = 100)

        val wavelength = GnssMathUtils.SPEED_OF_LIGHT / carrierFrequencyHz
        val freqScale = (GnssMathUtils.L1_FREQUENCY_HZ / carrierFrequencyHz).pow(2)
        val roverClockMeters = 25.0
        val baseClockMeters = -11.0
        val baseObservations = ArrayList<RtcmMsmParser.BaseObservation>()
        val roverMeasurements = ArrayList<SatelliteMeasurement>()

        for (eph in ephemerides) {
            if (roverMeasurements.size >= maxSatellites) break
            val roughRover = SatellitePositionCalculator.calculateSatellitePosition(eph, constellationTimeSeconds)
            val flightRover =
                sqrt(
                    (roughRover.first - roverX).pow(2) +
                        (roughRover.second - roverY).pow(2) +
                        (roughRover.third - roverZ).pow(2),
                ) / GnssMathUtils.SPEED_OF_LIGHT
            val txRover = SatellitePositionCalculator.calculateSatellitePosition(eph, constellationTimeSeconds - flightRover)
            val rotRover =
                SatellitePositionCalculator.rotateEcefForEarthRotationDuringTransit(
                    txRover.first,
                    txRover.second,
                    txRover.third,
                    flightRover,
                )
            val (roverEl, roverAz) =
                GnssMathUtils.ecefLosToElevationAzimuth(
                    receiverX = roverX,
                    receiverY = roverY,
                    receiverZ = roverZ,
                    receiverLatDeg = roverLat,
                    receiverLonDeg = roverLon,
                    satelliteX = rotRover.first,
                    satelliteY = rotRover.second,
                    satelliteZ = rotRover.third,
                )

            val roughBase = SatellitePositionCalculator.calculateSatellitePosition(eph, baseTimeSeconds)
            val flightBase =
                sqrt(
                    (roughBase.first - baseX).pow(2) +
                        (roughBase.second - baseY).pow(2) +
                        (roughBase.third - baseZ).pow(2),
                ) / GnssMathUtils.SPEED_OF_LIGHT
            val txBase = SatellitePositionCalculator.calculateSatellitePosition(eph, baseTimeSeconds - flightBase)
            val rotBase =
                SatellitePositionCalculator.rotateEcefForEarthRotationDuringTransit(
                    txBase.first,
                    txBase.second,
                    txBase.third,
                    flightBase,
                )
            val (baseEl, baseAz) =
                GnssMathUtils.ecefLosToElevationAzimuth(
                    receiverX = baseX,
                    receiverY = baseY,
                    receiverZ = baseZ,
                    receiverLatDeg = baseLat,
                    receiverLonDeg = baseLon,
                    satelliteX = rotBase.first,
                    satelliteY = rotBase.second,
                    satelliteZ = rotBase.third,
                )
            if (roverEl < 15.0 || baseEl < 15.0) continue

            val roverGeomRange =
                sqrt(
                    (rotRover.first - roverX).pow(2) +
                        (rotRover.second - roverY).pow(2) +
                        (rotRover.third - roverZ).pow(2),
                )
            val baseGeomRange =
                sqrt(
                    (rotBase.first - baseX).pow(2) +
                        (rotBase.second - baseY).pow(2) +
                        (rotBase.third - baseZ).pow(2),
                )
            val roverTropo =
                if (includeTroposphere) {
                    AtmosphericCorrection.computeTroposphericDelay(roverEl, roverAlt, roverLat, dayOfYear)
                } else {
                    0.0
                }
            val baseTropo =
                if (includeTroposphere) {
                    AtmosphericCorrection.computeTroposphericDelay(baseEl, baseAlt, baseLat, dayOfYear)
                } else {
                    0.0
                }
            val roverIono =
                if (includeIonosphere) {
                    AtmosphericCorrection.computeIonosphericDelay(roverEl, roverAz, roverLat, roverLon, gpsTimeMs) * freqScale
                } else {
                    0.0
                }
            val baseIono =
                if (includeIonosphere) {
                    AtmosphericCorrection.computeIonosphericDelay(baseEl, baseAz, baseLat, baseLon, gpsTimeMs) * freqScale
                } else {
                    0.0
                }

            val roverN = (1000 + eph.svid * 7).toDouble()
            val baseN = (1000 + eph.svid * 5).toDouble()
            val roverPr = roverGeomRange + roverClockMeters + roverTropo + roverIono
            val basePr = baseGeomRange + baseClockMeters + baseTropo + baseIono
            val roverCp = if (includeCarrier) roverGeomRange + roverClockMeters + roverTropo - roverIono + roverN * wavelength else null
            val baseCp = if (includeCarrier) baseGeomRange + baseClockMeters + baseTropo - baseIono + baseN * wavelength else null

            baseObservations.add(
                RtcmMsmParser.BaseObservation(
                    constellationType = constellationType,
                    svid = eph.svid,
                    carrierFrequencyHz = carrierFrequencyHz,
                    pseudorangeMeters = basePr,
                    carrierPhaseMeters = baseCp,
                    cnrDbHz = 45.0,
                    epochTimeMs = (baseTimeSeconds * 1000.0).toLong(),
                    signalId = 2,
                    lockTimeMillis = 60_000L,
                    referenceStationId = 100,
                ),
            )
            roverMeasurements.add(
                SatelliteMeasurement(
                    constellationType = constellationType,
                    svid = eph.svid,
                    pseudorangeMeters = roverPr,
                    carrierPhaseMeters = roverCp,
                    wavelengthMeters = wavelength,
                    carrierFrequencyHz = carrierFrequencyHz,
                    cn0DbHz = 45.0,
                    elevationDegrees = 20.0, // intentional placeholder before GnssStatus arrives
                    azimuthDegrees = 0.0,
                    accumulatedDeltaRangeMeters = roverCp,
                    pseudorangeUncertaintyMeters = 0.5,
                    isL5 = false,
                    adrState = AdrState.VALID,
                ),
            )
        }
        assertTrue("need >= 4 matched satellites, had ${roverMeasurements.size}", roverMeasurements.size >= 4)
        rtkProcessor.rtkBaseStore.putObservations(baseObservations)
        return RtkFixture(rtkProcessor, roverMeasurements, roverLat, roverLon, roverAlt, gpsTimeNanos)
    }

    @Test
    fun computeRtkPosition_rejectsBaselinesExceeding50Km() {
        // 0.8 deg longitude at 47 deg lat ≈ 60.8 km (> 50 km cap).
        val fixture =
            buildRtkFixture(
                baseLat = 47.0,
                baseLon = 8.0,
                baseAlt = 400.0,
                roverLat = 47.0,
                roverLon = 8.8,
                roverAlt = 400.0,
            )

        // Seeded > 50 km -> immediately rejected.
        assertNull(
            fixture.processor.computeRtkPosition(
                fixture.roverMeasurements,
                fixture.gpsTimeNanos,
                initialLat = fixture.roverLat,
                initialLon = fixture.roverLon,
                initialAlt = fixture.roverAlt,
            ),
        )
        // Cold seed (0, 0) solves from baseEcef -> post-solve > 50 km -> still rejected!
        assertNull(
            fixture.processor.computeRtkPosition(
                fixture.roverMeasurements,
                fixture.gpsTimeNanos,
            ),
        )
    }

    @Test
    fun computeRtkPosition_recoversAltitudeAcross500mVerticalBaselineWithTroposphericDelay() {
        val fixture =
            buildRtkFixture(
                baseLat = 47.0,
                baseLon = 8.0,
                baseAlt = 100.0,
                roverLat = 47.002,
                roverLon = 8.002,
                roverAlt = 600.0,
                includeTroposphere = true,
            )

        // Cold seed (baseEcef): float refines altitude, re-evaluates rover tropo at 600 m, and fixes cm-accurate.
        val result = fixture.processor.computeRtkPosition(fixture.roverMeasurements, fixture.gpsTimeNanos)
        assertNotNull(result)
        requireNotNull(result)
        assertEquals(GnssSolutionMode.RTK_FIXED, result.solutionMode)
        assertTrue(
            "expected altitude within 5 cm of 600.0 m (tropo-corrected), was ${result.altitude} m",
            abs(result.altitude - 600.0) < 0.05,
        )
    }

    @Test
    fun computeRtkPosition_appliesDifferentialIonosphereAbove10Km() {
        // ~20 km baseline (> 10 km threshold, <= 50 km cap) with differential Klobuchar ionosphere.
        val fixture =
            buildRtkFixture(
                baseLat = 47.0,
                baseLon = 8.0,
                baseAlt = 300.0,
                roverLat = 47.15,
                roverLon = 8.10,
                roverAlt = 450.0,
                includeTroposphere = true,
                includeIonosphere = true,
            )

        val result =
            fixture.processor.computeRtkPosition(
                fixture.roverMeasurements,
                fixture.gpsTimeNanos,
                initialLat = fixture.roverLat,
                initialLon = fixture.roverLon,
                initialAlt = fixture.roverAlt,
            )
        assertNotNull(result)
        requireNotNull(result)
        assertEquals(GnssSolutionMode.RTK_FIXED, result.solutionMode)
        assertTrue(
            "expected altitude within 5 cm of 450.0 m on 20 km baseline, was ${result.altitude} m",
            abs(result.altitude - 450.0) < 0.05,
        )
    }

    @Test
    fun computeRtkPosition_matchesBeidouObservationsAcrossGpsWeekBoundary() {
        // gpstSeconds = 5.0 s -> BDT = (5.0 - 14.0) mod 604800 = 604,791.0 s.
        // Base epoch is 1.0 s earlier (604,790.0 s BDT), and also test straddling 0.0 s BDT.
        val fixture =
            buildRtkFixture(
                baseLat = 47.0,
                baseLon = 8.0,
                baseAlt = 400.0,
                roverLat = 47.001,
                roverLon = 8.001,
                roverAlt = 420.0,
                constellationType = GnssStatus.CONSTELLATION_BEIDOU,
                carrierFrequencyHz = 1_561_098_000.0,
                gpsTimeNanos = 5_000_000_000L,
                baseEpochOffsetSeconds = -1.0,
            )

        val result = fixture.processor.computeRtkPosition(fixture.roverMeasurements, fixture.gpsTimeNanos)
        assertNotNull("BeiDou observations across GPS week rollover must match", result)
        requireNotNull(result)
        assertEquals(GnssSolutionMode.RTK_FIXED, result.solutionMode)
        assertTrue(abs(result.altitude - 420.0) < 0.05)

        // Also test gpstSeconds = 15.0 s (BDT = 1.0 s) vs base at BDT = 604,799.0 s (-2.0 s across BDT week rollover).
        val bdtRolloverFixture =
            buildRtkFixture(
                baseLat = 47.0,
                baseLon = 8.0,
                baseAlt = 400.0,
                roverLat = 47.001,
                roverLon = 8.001,
                roverAlt = 420.0,
                constellationType = GnssStatus.CONSTELLATION_BEIDOU,
                carrierFrequencyHz = 1_561_098_000.0,
                gpsTimeNanos = 15_000_000_000L,
                baseEpochOffsetSeconds = -2.0,
            )
        val bdtRolloverResult =
            bdtRolloverFixture.processor.computeRtkPosition(
                bdtRolloverFixture.roverMeasurements,
                bdtRolloverFixture.gpsTimeNanos,
            )
        assertNotNull("BeiDou observations across BDT week rollover must match", bdtRolloverResult)
    }

    @Test
    fun computeRtkPosition_fourSatelliteCodeFloatDoesNotClaimTwoCmAccuracy() {
        val fixture =
            buildRtkFixture(
                baseLat = 47.0,
                baseLon = 8.0,
                baseAlt = 400.0,
                roverLat = 47.001,
                roverLon = 8.001,
                roverAlt = 405.0,
                includeCarrier = false,
                maxSatellites = 4,
            )

        val result = fixture.processor.computeRtkPosition(fixture.roverMeasurements, fixture.gpsTimeNanos)
        assertNotNull(result)
        requireNotNull(result)
        assertEquals(GnssSolutionMode.RTK_FLOAT, result.solutionMode)
        assertTrue(
            "0-DOF 4-SV code float must not claim 2 cm accuracy, was ${result.horizontalAccuracyMeters} m",
            result.horizontalAccuracyMeters > 0.2f,
        )
    }

    @Test
    fun rtkToAbsoluteTransition_syncsKalmanStateWithoutCoasting_andPopulatesDopplerVelocityBearing() {
        val locALat = 47.0
        val locALon = 8.0
        val locAAlt = 400.0
        val t1Nanos = SYNTHETIC_GPS_TIME_NANOS

        // Location B is ~220 m NE of Location A
        val locBLat = 47.0015
        val locBLon = 8.0020
        val locBAlt = 415.0
        val t2Nanos = t1Nanos + GnssMathUtils.NANOS_PER_SECOND
        val t3Nanos = t2Nanos + GnssMathUtils.NANOS_PER_SECOND

        val rtkProcessor = GnssMeasurementProcessor()
        val (ephA, measA) =
            buildSyntheticSkyEpoch(
                constellationType = GnssStatus.CONSTELLATION_GPS,
                lat = locALat,
                lon = locALon,
                alt = locAAlt,
                gpsTimeNanos = t1Nanos,
            )
        val tow1 = (t1Nanos / 1e9) % GnssMathUtils.SECONDS_IN_WEEK
        rtkProcessor.importExternalEphemeris(ephA, tow1)

        // Epoch 1 (t = 100 s): absolute fix at Location A warms up interEpochKalman at A.
        val fixA =
            requireNotNull(
                rtkProcessor.computePosition(
                    measurements = measA,
                    initialLat = locALat,
                    initialLon = locALon,
                    initialAlt = locAAlt,
                    usePpp = false,
                    gpsTimeNanos = t1Nanos,
                ),
            )
        assertTrue(ecefDistanceMeters(locALat, locALon, locAAlt, fixA.latitude, fixA.longitude, fixA.altitude) < 1.0)

        // Epoch 2 (t = 101 s): RTK fix at Location B with known velocity (12 m/s E, 5 m/s N -> 13 m/s, ~67.38 deg)
        val (baseX, baseY, baseZ) = GnssMathUtils.latLonAltToEcef(locALat, locALon, locAAlt)
        val (roverX, roverY, roverZ) = GnssMathUtils.latLonAltToEcef(locBLat, locBLon, locBAlt)
        val tow2 = (t2Nanos / 1e9) % GnssMathUtils.SECONDS_IN_WEEK
        val latRad = Math.toRadians(locBLat)
        val lonRad = Math.toRadians(locBLon)
        val sinLat = kotlin.math.sin(latRad)
        val cosLat = kotlin.math.cos(latRad)
        val sinLon = kotlin.math.sin(lonRad)
        val cosLon = kotlin.math.cos(lonRad)
        val vEast = 12.0
        val vNorth = 5.0
        val vxRx = vEast * (-sinLon) + vNorth * (-sinLat * cosLon)
        val vyRx = vEast * cosLon + vNorth * (-sinLat * sinLon)
        val vzRx = vNorth * cosLat
        val rxClockDriftMps = 2.5

        val unixSeconds2 = GnssMathUtils.GPS_EPOCH_UNIX_SECONDS + (t2Nanos / 1_000_000_000L)
        val utcCalendar2 = java.util.Calendar.getInstance(java.util.TimeZone.getTimeZone("UTC"))
        utcCalendar2.timeInMillis = unixSeconds2 * 1000L
        val dayOfYear2 = utcCalendar2.get(java.util.Calendar.DAY_OF_YEAR)
        val gpsTimeMs2 = t2Nanos / 1_000_000L
        val wavelength = GnssMathUtils.SPEED_OF_LIGHT / GnssMathUtils.L1_FREQUENCY_HZ

        rtkProcessor.rtkBaseStore.setBasePosition(baseX, baseY, baseZ, referenceStationId = 100)
        val baseObs2 = ArrayList<RtcmMsmParser.BaseObservation>()
        val roverMeas2 = ArrayList<SatelliteMeasurement>()
        for (eph in ephA) {
            val roughR = SatellitePositionCalculator.calculateSatellitePosition(eph, tow2)
            val flightR =
                sqrt((roughR.first - roverX).pow(2) + (roughR.second - roverY).pow(2) + (roughR.third - roverZ).pow(2)) /
                    GnssMathUtils.SPEED_OF_LIGHT
            val txTimeR = tow2 - flightR
            val txR = SatellitePositionCalculator.calculateSatellitePosition(eph, txTimeR)
            val velR = SatellitePositionCalculator.satelliteEcefVelocity(eph, txTimeR)
            val rotR = SatellitePositionCalculator.rotateEcefForEarthRotationDuringTransit(txR.first, txR.second, txR.third, flightR)
            val rotVelR = SatellitePositionCalculator.rotateEcefForEarthRotationDuringTransit(velR.first, velR.second, velR.third, flightR)
            val satDrift = SatellitePositionCalculator.satelliteClockDriftMetersPerSecond(eph, txTimeR)
            val (elR, azR) =
                GnssMathUtils.ecefLosToElevationAzimuth(roverX, roverY, roverZ, locBLat, locBLon, rotR.first, rotR.second, rotR.third)

            val roughB = SatellitePositionCalculator.calculateSatellitePosition(eph, tow2)
            val flightB =
                sqrt((roughB.first - baseX).pow(2) + (roughB.second - baseY).pow(2) + (roughB.third - baseZ).pow(2)) /
                    GnssMathUtils.SPEED_OF_LIGHT
            val txB = SatellitePositionCalculator.calculateSatellitePosition(eph, tow2 - flightB)
            val rotB = SatellitePositionCalculator.rotateEcefForEarthRotationDuringTransit(txB.first, txB.second, txB.third, flightB)
            val (elB, azB) =
                GnssMathUtils.ecefLosToElevationAzimuth(baseX, baseY, baseZ, locALat, locALon, rotB.first, rotB.second, rotB.third)
            if (elR < 15.0 || elB < 15.0) continue

            val rangeR = sqrt((rotR.first - roverX).pow(2) + (rotR.second - roverY).pow(2) + (rotR.third - roverZ).pow(2))
            val rangeB = sqrt((rotB.first - baseX).pow(2) + (rotB.second - baseY).pow(2) + (rotB.third - baseZ).pow(2))
            val tropoR = AtmosphericCorrection.computeTroposphericDelay(elR, locBAlt, locBLat, dayOfYear2)
            val tropoB = AtmosphericCorrection.computeTroposphericDelay(elB, locAAlt, locALat, dayOfYear2)
            val ionoR = AtmosphericCorrection.computeIonosphericDelay(elR, azR, locBLat, locBLon, gpsTimeMs2, null, null)
            val ionoB = AtmosphericCorrection.computeIonosphericDelay(elB, azB, locALat, locALon, gpsTimeMs2, null, null)

            val losX = (rotR.first - roverX) / rangeR
            val losY = (rotR.second - roverY) / rangeR
            val losZ = (rotR.third - roverZ) / rangeR
            val prr =
                losX * (rotVelR.first - vxRx) +
                    losY * (rotVelR.second - vyRx) +
                    losZ * (rotVelR.third - vzRx) +
                    rxClockDriftMps -
                    satDrift

            val nR = (1000 + eph.svid * 7).toDouble()
            val nB = (1000 + eph.svid * 5).toDouble()
            val cpR = rangeR + 15.0 + tropoR - ionoR + nR * wavelength
            val cpB = rangeB - 9.0 + tropoB - ionoB + nB * wavelength

            baseObs2 +=
                RtcmMsmParser.BaseObservation(
                    constellationType = GnssStatus.CONSTELLATION_GPS,
                    svid = eph.svid,
                    carrierFrequencyHz = GnssMathUtils.L1_FREQUENCY_HZ,
                    pseudorangeMeters = rangeB - 9.0 + tropoB + ionoB,
                    carrierPhaseMeters = cpB,
                    cnrDbHz = 45.0,
                    epochTimeMs = (tow2 * 1000.0).toLong(),
                    signalId = 2,
                    lockTimeMillis = 60_000L,
                    referenceStationId = 100,
                )
            roverMeas2 +=
                SatelliteMeasurement(
                    constellationType = GnssStatus.CONSTELLATION_GPS,
                    svid = eph.svid,
                    pseudorangeMeters = rangeR + 15.0 + tropoR + ionoR,
                    carrierPhaseMeters = cpR,
                    wavelengthMeters = wavelength,
                    carrierFrequencyHz = GnssMathUtils.L1_FREQUENCY_HZ,
                    cn0DbHz = 45.0,
                    elevationDegrees = elR,
                    azimuthDegrees = azR,
                    accumulatedDeltaRangeMeters = cpR,
                    pseudorangeUncertaintyMeters = 0.5,
                    adrState = AdrState.VALID,
                    pseudorangeRateMetersPerSecond = prr,
                    pseudorangeRateUncertaintyMetersPerSecond = 0.1,
                )
        }
        assertTrue("Need >= 6 matched SVs for RTK + Doppler, got ${roverMeas2.size}", roverMeas2.size >= 6)
        rtkProcessor.rtkBaseStore.putObservations(baseObs2)

        val rtkFix =
            requireNotNull(
                rtkProcessor.computeRtkPosition(
                    measurements = roverMeas2,
                    gpsTimeNanos = t2Nanos,
                    initialLat = locBLat,
                    initialLon = locBLon,
                    initialAlt = locBAlt,
                ),
            )
        assertEquals(GnssSolutionMode.RTK_FIXED, rtkFix.solutionMode)
        assertTrue(ecefDistanceMeters(locBLat, locBLon, locBAlt, rtkFix.latitude, rtkFix.longitude, rtkFix.altitude) < 0.05)

        val expectedSpeed = kotlin.math.hypot(vEast, vNorth)
        val expectedBearing = Math.toDegrees(kotlin.math.atan2(vEast, vNorth))
        val rtkSpeed = requireNotNull(rtkFix.speedMetersPerSecond) { "RTK fix must populate speedMetersPerSecond" }
        val rtkBearing = requireNotNull(rtkFix.bearingDegrees) { "RTK fix must populate bearingDegrees" }
        assertEquals(expectedSpeed, rtkSpeed, 0.5)
        assertEquals(expectedBearing, rtkBearing, 2.0)

        // Epoch 3 (t = 102 s, dt = 1 s < 5 s): RTK drops out and falls back to computePosition.
        // Rover moved 1 s along (vEast, vNorth) from Location B to Location C. Because
        // interEpochKalman was synced to B with Doppler velocity, this must NOT coast or pull back
        // toward Location A!
        rtkProcessor.rtkBaseStore.clear()
        val (locCLat, locCLon, locCAlt) = GnssMathUtils.ecefToLatLonAlt(roverX + vxRx, roverY + vyRx, roverZ + vzRx)
        val (cX, cY, cZ) = GnssMathUtils.latLonAltToEcef(locCLat, locCLon, locCAlt)
        val tow3 = (t3Nanos / 1e9) % GnssMathUtils.SECONDS_IN_WEEK
        val unixSeconds3 = GnssMathUtils.GPS_EPOCH_UNIX_SECONDS + (t3Nanos / 1_000_000_000L)
        val utcCalendar3 = java.util.Calendar.getInstance(java.util.TimeZone.getTimeZone("UTC"))
        utcCalendar3.timeInMillis = unixSeconds3 * 1000L
        val dayOfYear3 = utcCalendar3.get(java.util.Calendar.DAY_OF_YEAR)
        val gpsTimeMs3 = t3Nanos / 1_000_000L
        val measC3 =
            ephA.mapNotNull { eph ->
                val rough = SatellitePositionCalculator.calculateSatellitePosition(eph, tow3)
                val flight =
                    sqrt((rough.first - cX).pow(2) + (rough.second - cY).pow(2) + (rough.third - cZ).pow(2)) /
                        GnssMathUtils.SPEED_OF_LIGHT
                val tx = SatellitePositionCalculator.calculateSatellitePosition(eph, tow3 - flight)
                val rot = SatellitePositionCalculator.rotateEcefForEarthRotationDuringTransit(tx.first, tx.second, tx.third, flight)
                val (el, az) = GnssMathUtils.ecefLosToElevationAzimuth(cX, cY, cZ, locCLat, locCLon, rot.first, rot.second, rot.third)
                if (el < 20.0) return@mapNotNull null
                val range = sqrt((rot.first - cX).pow(2) + (rot.second - cY).pow(2) + (rot.third - cZ).pow(2))
                val dry = AtmosphericCorrection.computeTroposphericDelay(el, locCAlt, locCLat, dayOfYear3)
                val iono = AtmosphericCorrection.computeIonosphericDelay(el, az, locCLat, locCLon, gpsTimeMs3, null, null)
                val rel = AtmosphericCorrection.computeRelativisticCorrection(cX, cY, cZ, rot.first, rot.second, rot.third)
                SatelliteMeasurement(
                    constellationType = GnssStatus.CONSTELLATION_GPS,
                    svid = eph.svid,
                    pseudorangeMeters = range + dry + iono + rel,
                    carrierFrequencyHz = GnssMathUtils.L1_FREQUENCY_HZ,
                    cn0DbHz = 42.0,
                    elevationDegrees = el,
                    azimuthDegrees = az,
                    accumulatedDeltaRangeMeters = null,
                    pseudorangeUncertaintyMeters = 0.5,
                )
            }
        val fallbackFix =
            requireNotNull(
                rtkProcessor.computePosition(
                    measurements = measC3,
                    initialLat = locCLat,
                    initialLon = locCLon,
                    initialAlt = locCAlt,
                    usePpp = false,
                    gpsTimeNanos = t3Nanos,
                ),
            )
        assertNull("Fallback after RTK must not trip Kalman coast gate against old pre-RTK state", rtkProcessor.getKalmanCoastForTest())
        val fallbackDistToC = ecefDistanceMeters(locCLat, locCLon, locCAlt, fallbackFix.latitude, fallbackFix.longitude, fallbackFix.altitude)
        assertTrue(
            "Fallback after RTK must stay at Location C within < 1.0 m (was $fallbackDistToC m)",
            fallbackDistToC < 1.0,
        )
    }
}
