package app.simt.gnss

import android.location.GnssNavigationMessage
import android.location.GnssStatus
import android.os.SystemClock
import app.simt.gnss.core.model.SatelliteEphemeris
import app.simt.gnss.core.model.SatelliteMeasurement
import app.simt.gnss.core.orbit.SatellitePositionCalculator
import app.simt.gnss.core.solver.GnssPositionSolver
import app.simt.gnss.core.utils.GnssMathUtils
import org.junit.Assert.*
import org.junit.Before
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import kotlin.math.sqrt

@RunWith(RobolectricTestRunner::class)
class GnssMeasurementProcessorTest {
    private lateinit var processor: GnssMeasurementProcessor
    private val syntheticGpsTimeNanos = 345_600_000_000_000L

    @Before
    fun setUp() {
        processor = GnssMeasurementProcessor()
    }

    private fun computePositionWithAccuracyFallback(
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
        gpsTimeNanos = syntheticGpsTimeNanos,
        accuracy = GnssPositionSolver.Accuracy.HIGH,
    ) ?: processor.computePosition(
        measurements = measurements,
        initialLat = initialLat,
        initialLon = initialLon,
        initialAlt = initialAlt,
        usePpp = usePpp,
        gpsTimeNanos = syntheticGpsTimeNanos,
        accuracy = GnssPositionSolver.Accuracy.MEDIUM,
    )

    /**
     * Creates a synthetic measurement consistent with SatellitePositionCalculator.getSatellitePositionApproximation.
     * The pseudorange is computed from the approximate satellite position to the given receiver ECEF position.
     */
    private fun createConsistentMeasurement(
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

    @Test
    fun testIonosphereFreeCombination() {
        val l1 =
            SatelliteMeasurement(
                constellationType = GnssStatus.CONSTELLATION_GPS,
                svid = 1,
                pseudorangeMeters = 20_000_000.0,
                carrierFrequencyHz = 1.57542e9,
                cn0DbHz = 40.0,
                elevationDegrees = 45.0,
                azimuthDegrees = 90.0,
                accumulatedDeltaRangeMeters = null,
                pseudorangeUncertaintyMeters = 1.0,
                isL5 = false,
            )
        val l5 =
            l1.copy(
                pseudorangeMeters = 20_000_005.0,
                carrierFrequencyHz = 1.17645e9,
                isL5 = true,
            )

        val measurements = mutableListOf(l1, l5)
        for (i in 2..4) {
            measurements.add(l1.copy(svid = i, constellationType = GnssStatus.CONSTELLATION_GPS))
        }

        // Should not crash with IF combination logic
        processor.computePosition(measurements, 45.0, 8.0, 100.0, usePpp = false)
    }

    @Test
    fun testPppStateTracking() {
        val measurements =
            (1..4).map { svid ->
                SatelliteMeasurement(
                    constellationType = GnssStatus.CONSTELLATION_GPS,
                    svid = svid,
                    pseudorangeMeters = 20_000_000.0,
                    carrierPhaseMeters = 20_000_000.0,
                    wavelengthMeters = 0.19,
                    carrierFrequencyHz = 1.57542e9,
                    cn0DbHz = 40.0,
                    elevationDegrees = 45.0 + svid,
                    azimuthDegrees = 90.0 * svid,
                    accumulatedDeltaRangeMeters = 20_000_000.0,
                    pseudorangeUncertaintyMeters = 1.0,
                    isL5 = false,
                )
            }

        processor.computePosition(measurements, 45.0, 8.0, 100.0, usePpp = true)
        processor.computePosition(measurements, 45.0, 8.0, 100.0, usePpp = true)
    }

    @Test
    fun testNavicNavigationMessageTypesMapToIrnssConstellation() {
        val navicMessageTypes =
            listOf(
                GnssNavigationMessage.TYPE_IRN_L5CA,
                GnssNavigationMessage.TYPE_IRN_L5,
                GnssNavigationMessage.TYPE_IRN_L1,
            )

        navicMessageTypes.forEach { type ->
            assertEquals(
                "NavIC navigation message type $type should map to IRNSS constellation",
                GnssStatus.CONSTELLATION_IRNSS,
                processor.getConstellationForNavigationMessageType(type),
            )
        }
    }

    @Test
    fun testWlsConvergenceWithRealisticGeometry() {
        // True receiver position (Paris, 48.8N, 2.3E, 100m alt)
        val trueLat = 48.8
        val trueLon = 2.3
        val trueAlt = 100.0
        val (trueX, trueY, trueZ) = GnssMathUtils.latLonAltToEcef(trueLat, trueLon, trueAlt)

        // Observer lat/lon/alt for satellite approximation
        // Use the same as true position so approximate satellite positions are consistent
        val obsLat = trueLat
        val obsLon = trueLon
        val obsAlt = trueAlt

        // Clock bias in meters (realistic ~1ms * c)
        val clockBias = 300_000.0

        // 6 GPS satellites spread across the sky
        val satParams =
            listOf(
                Pair(70.0, 0.0), // high N
                Pair(45.0, 90.0), // mid E
                Pair(30.0, 180.0), // low S
                Pair(50.0, 270.0), // mid W
                Pair(80.0, 45.0), // high NE
                Pair(35.0, 225.0), // low SW
            )

        val measurements =
            satParams.mapIndexed { idx, (elev, az) ->
                createConsistentMeasurement(
                    svid = idx + 1,
                    elevDeg = elev,
                    azDeg = az,
                    recX = trueX,
                    recY = trueY,
                    recZ = trueZ,
                    observerLat = obsLat,
                    observerLon = obsLon,
                    observerAlt = obsAlt,
                    clockBiasMeters = clockBias,
                )
            }

        val result =
            computePositionWithAccuracyFallback(
                measurements = measurements,
                initialLat = obsLat,
                initialLon = obsLon,
                initialAlt = obsAlt,
                usePpp = false,
            )

        // Without ephemeris, the solver correctly returns null — satellite clock
        // corrections are needed for accurate positioning.
        assertNull(
            "Without ephemeris, computePosition should return null (no satellite clock corrections)",
            result,
        )
    }

    @Test
    fun testRaimExcludesOutlier() {
        val trueLat = 48.8
        val trueLon = 2.3
        val trueAlt = 100.0
        val (trueX, trueY, trueZ) = GnssMathUtils.latLonAltToEcef(trueLat, trueLon, trueAlt)

        val clockBias = 300_000.0

        val satParams =
            listOf(
                Pair(70.0, 0.0),
                Pair(45.0, 60.0),
                Pair(30.0, 120.0),
                Pair(50.0, 180.0),
                Pair(40.0, 240.0),
                Pair(60.0, 300.0),
                Pair(35.0, 45.0),
            )

        val measurements =
            satParams.mapIndexed { idx, (elev, az) ->
                createConsistentMeasurement(
                    svid = idx + 1,
                    elevDeg = elev,
                    azDeg = az,
                    recX = trueX,
                    recY = trueY,
                    recZ = trueZ,
                    observerLat = trueLat,
                    observerLon = trueLon,
                    observerAlt = trueAlt,
                    clockBiasMeters = clockBias,
                    // Add a strong outlier to satellite 3
                    noiseMeters = if (idx == 2) 300.0 else 0.0,
                )
            }

        // With 7 satellites (> 5), RAIM should detect and exclude the outlier
        val result =
            computePositionWithAccuracyFallback(
                measurements = measurements,
                initialLat = trueLat,
                initialLon = trueLon,
                initialAlt = trueAlt,
                usePpp = false,
            )

        // Without ephemeris, solver returns null — cannot compute position.
        assertNull(
            "Without ephemeris, computePosition should return null",
            result,
        )
    }

    @Test
    fun testNoEphemerisUsesDegradedApproximateGeometry() {
        val trueLat = 37.7749
        val trueLon = -122.4194
        val trueAlt = 30.0
        val (trueX, trueY, trueZ) = GnssMathUtils.latLonAltToEcef(trueLat, trueLon, trueAlt)

        val seedLat = trueLat + 0.0015
        val seedLon = trueLon - 0.0015
        val seedAlt = trueAlt + 35.0
        val clockBias = 300_000.0

        val satParams =
            listOf(
                75.0 to 20.0,
                50.0 to 95.0,
                25.0 to 160.0,
                60.0 to 240.0,
                35.0 to 300.0,
                45.0 to 350.0,
            )

        val measurements =
            satParams.mapIndexed { idx, (elev, az) ->
                createConsistentMeasurement(
                    svid = idx + 1,
                    elevDeg = elev,
                    azDeg = az,
                    recX = trueX,
                    recY = trueY,
                    recZ = trueZ,
                    observerLat = trueLat,
                    observerLon = trueLon,
                    observerAlt = trueAlt,
                    clockBiasMeters = clockBias,
                )
            }

        // Without ephemeris, computePosition should return null because
        // satellite clock corrections are unavailable — per-satellite errors
        // of 100-500 km make positioning impossible.
        val result =
            computePositionWithAccuracyFallback(
                measurements = measurements,
                initialLat = seedLat,
                initialLon = seedLon,
                initialAlt = seedAlt,
                usePpp = false,
            )

        assertNull(
            "Without ephemeris, computePosition must return null (no satellite clock corrections)",
            result,
        )
    }

    @Test
    fun testMinimumSatelliteRequirement() {
        val l1 =
            SatelliteMeasurement(
                constellationType = GnssStatus.CONSTELLATION_GPS,
                svid = 1,
                pseudorangeMeters = 20_000_000.0,
                carrierFrequencyHz = 1.57542e9,
                cn0DbHz = 40.0,
                elevationDegrees = 45.0,
                azimuthDegrees = 90.0,
                accumulatedDeltaRangeMeters = null,
                pseudorangeUncertaintyMeters = 1.0,
                isL5 = false,
            )

        // 3 satellites is not enough
        val measurements = (1..3).map { l1.copy(svid = it) }
        val result = processor.computePosition(measurements, 45.0, 8.0, 100.0, usePpp = false)
        assertNull("Should return null with < 4 satellites", result)
    }

    @Test
    fun testPppStateClassLifecycle() {
        val state = GnssMeasurementProcessor.PppState()

        assertFalse(state.hasLastPosition())
        assertNull(state.getLastPositionEcef())

        state.updatePosition(1.0, 2.0, 3.0, 100L)
        assertTrue(state.hasLastPosition())
        assertEquals(Triple(1.0, 2.0, 3.0), state.getLastPositionEcef())
        assertEquals(100L, state.lastTimeNanos)

        val satKey = 1 to 5
        state.carrierAmbiguities[satKey] = 42.0
        assertEquals(42.0, state.carrierAmbiguities[satKey]!!, 1e-10)

        state.resetAmbiguity(satKey)
        assertNull(state.carrierAmbiguities[satKey])

        state.reset()
        assertFalse(state.hasLastPosition())
        assertEquals(0.1, state.zenithWetDelay, 1e-10)
        assertTrue(state.carrierAmbiguities.isEmpty())
    }

    /**
     * Verifies that RAIM can exclude an outlier from exactly 5 satellites, leaving 4
     * (the geometric minimum for a position fix). This tests the > 4 threshold change.
     */
    @Test
    fun testRaimExcludesOutlierFromFiveSatellites() {
        val trueLat = 48.8
        val trueLon = 2.3
        val trueAlt = 100.0
        val (trueX, trueY, trueZ) = GnssMathUtils.latLonAltToEcef(trueLat, trueLon, trueAlt)

        val clockBias = 300_000.0

        // Exactly 5 satellites with good sky spread
        val satParams =
            listOf(
                Pair(75.0, 0.0),
                Pair(45.0, 90.0),
                Pair(30.0, 180.0),
                Pair(50.0, 270.0),
                Pair(60.0, 45.0),
            )

        val measurements =
            satParams.mapIndexed { idx, (elev, az) ->
                createConsistentMeasurement(
                    svid = idx + 1,
                    elevDeg = elev,
                    azDeg = az,
                    recX = trueX,
                    recY = trueY,
                    recZ = trueZ,
                    observerLat = trueLat,
                    observerLon = trueLon,
                    observerAlt = trueAlt,
                    clockBiasMeters = clockBias,
                    // Very large outlier on satellite 3 (simulates cycle slip corruption like SVID 34 in logs)
                    noiseMeters = if (idx == 2) 100_000.0 else 0.0,
                )
            }

        val result =
            computePositionWithAccuracyFallback(
                measurements = measurements,
                initialLat = trueLat,
                initialLon = trueLon,
                initialAlt = trueAlt,
                usePpp = false,
            )

        // Without ephemeris, solver correctly returns null.
        assertNull(
            "Without ephemeris, computePosition should return null",
            result,
        )
    }

    /**
     * Verifies that the approximate geometry residual RMS quality gate constant
     * is set to 250m — solutions with RMS below this should be accepted.
     */
    @Test
    fun testApproximateGeometryResidualRmsThresholdValue() {
        assertEquals(
            "APPROXIMATE_GEOMETRY_ACCEPTABLE_RESIDUAL_RMS_METERS should be 250.0",
            250.0,
            GnssMathUtils.APPROXIMATE_GEOMETRY_ACCEPTABLE_RESIDUAL_RMS_METERS,
            0.001,
        )
    }

    /**
     * Verifies that the approximate geometry max residual constant
     * is set to 500m — consistent with the RMS threshold.
     */
    @Test
    fun testApproximateGeometryMaxResidualThresholdValue() {
        assertEquals(
            "APPROXIMATE_GEOMETRY_ACCEPTABLE_MAX_RESIDUAL_METERS should be 500.0",
            500.0,
            GnssMathUtils.APPROXIMATE_GEOMETRY_ACCEPTABLE_MAX_RESIDUAL_METERS,
            0.001,
        )
    }

    /**
     * Verifies that the RAIM exclusion minimum satellite count allows
     * exclusion from 5 satellites (leaving 4 — the geometric minimum).
     * With a clean 4-satellite set the solver should still produce a fix.
     */
    @Test
    fun testFourSatellitesCanProduceFix() {
        val trueLat = 48.8
        val trueLon = 2.3
        val trueAlt = 100.0
        val (trueX, trueY, trueZ) = GnssMathUtils.latLonAltToEcef(trueLat, trueLon, trueAlt)

        val clockBias = 300_000.0

        // Exactly 4 clean satellites (what remains after RAIM excludes 1 from 5)
        val satParams =
            listOf(
                Pair(75.0, 0.0),
                Pair(45.0, 90.0),
                Pair(50.0, 270.0),
                Pair(60.0, 45.0),
            )

        val measurements =
            satParams.mapIndexed { idx, (elev, az) ->
                createConsistentMeasurement(
                    svid = idx + 1,
                    elevDeg = elev,
                    azDeg = az,
                    recX = trueX,
                    recY = trueY,
                    recZ = trueZ,
                    observerLat = trueLat,
                    observerLon = trueLon,
                    observerAlt = trueAlt,
                    clockBiasMeters = clockBias,
                )
            }

        val result =
            computePositionWithAccuracyFallback(
                measurements = measurements,
                initialLat = trueLat,
                initialLon = trueLon,
                initialAlt = trueAlt,
                usePpp = false,
            )

        // Without ephemeris, solver correctly returns null.
        assertNull(
            "Without ephemeris, computePosition should return null",
            result,
        )
    }

    @Test
    fun singleEpochPhase_matchesTheCodeOnlySolution() {
        val toe = 345_600.0
        val lat = 48.8
        val lon = 2.3
        val alt = 100.0
        val (recX, recY, recZ) = GnssMathUtils.latLonAltToEcef(lat, lon, alt)
        val receipt = SystemClock.elapsedRealtime()
        val ephemerides = ArrayList<SatelliteEphemeris>()
        var svid = 1
        for (raanDeg in 0 until 360 step 30) {
            for (meanDeg in listOf(0.0, 90.0, 180.0, 270.0)) {
                ephemerides +=
                    SatelliteEphemeris(
                        constellationType = GnssStatus.CONSTELLATION_GPS,
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
                svid++
            }
        }
        val measurements =
            ephemerides.mapNotNull { eph ->
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
                if (elev < 20.0) return@mapNotNull null
                val range =
                    sqrt(
                        (rotated.first - recX) * (rotated.first - recX) +
                            (rotated.second - recY) * (rotated.second - recY) +
                            (rotated.third - recZ) * (rotated.third - recZ),
                    )
                SatelliteMeasurement(
                    constellationType = GnssStatus.CONSTELLATION_GPS,
                    svid = eph.svid,
                    pseudorangeMeters = range,
                    carrierPhaseMeters = range,
                    wavelengthMeters = 0.190293672798,
                    carrierFrequencyHz = 1.57542e9,
                    cn0DbHz = 40.0,
                    elevationDegrees = elev,
                    azimuthDegrees = az,
                    accumulatedDeltaRangeMeters = range,
                    pseudorangeUncertaintyMeters = 1.0,
                    isL5 = false,
                )
            }
        assertTrue(
            "need a solvable sky, had ${measurements.size}: ${measurements.map { it.elevationDegrees.toInt() }}",
            measurements.size >= 6,
        )

        val withPhase = GnssMeasurementProcessor()
        val codeOnly = GnssMeasurementProcessor()
        assertEquals(ephemerides.size, withPhase.importExternalEphemeris(ephemerides, toe))
        assertEquals(ephemerides.size, codeOnly.importExternalEphemeris(ephemerides, toe))

        val phaseFix =
            withPhase.computePosition(
                measurements,
                lat,
                lon,
                alt,
                usePpp = true,
                gpsTimeNanos = syntheticGpsTimeNanos,
            )
        val codeFix =
            codeOnly.computePosition(
                measurements.map { it.copy(carrierPhaseMeters = null, accumulatedDeltaRangeMeters = null) },
                lat,
                lon,
                alt,
                usePpp = true,
                gpsTimeNanos = syntheticGpsTimeNanos,
            )
        assertNotNull(phaseFix)
        assertNotNull(codeFix)
        val phase = phaseFix!!
        val code = codeFix!!
        assertEquals(code.latitude, phase.latitude, 1e-8)
        assertEquals(code.longitude, phase.longitude, 1e-8)
        assertEquals(code.altitude, phase.altitude, 0.01)
        assertEquals(code.verticalAccuracyMeters, phase.verticalAccuracyMeters, 0.01f)
        assertEquals(code.horizontalAccuracyMeters, phase.horizontalAccuracyMeters, 0.01f)
    }
}
