package app.simt.gnss.core.location
import app.simt.gnss.core.model.Constellation
import app.simt.gnss.core.model.PositionResult
import app.simt.gnss.core.model.SatelliteMeasurement
import app.simt.gnss.core.solver.GnssPositionSolver
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertSame
import org.junit.Assert.assertTrue
import org.junit.Test

class RawGnssPositionFallbackResolverTest {
    @Test
    fun resolve_retriesCodeOnlyWhenPppStyleSolveFails() {
        val measurements = gpsMeasurements(count = 5)
        val expected = sampleResult(latitude = 1.0)
        val attempts = mutableListOf<String>()

        val result =
            RawGnssPositionFallbackResolver.resolve(
                measurements = measurements,
                initialLat = 48.8,
                initialLon = 2.3,
                initialAlt = 100.0,
                gpsTimeNanos = 123L,
                primaryAccuracy = GnssPositionSolver.Accuracy.HIGH,
                fallbackConstellationExclusions = intArrayOf(Constellation.GLONASS),
                positionComputer =
                    RawGnssPositionComputer { candidateMeasurements, _, _, _, usePpp, _, accuracy ->
                        attempts += attemptKey(candidateMeasurements, usePpp, accuracy)
                        if (!usePpp && accuracy == GnssPositionSolver.Accuracy.HIGH && candidateMeasurements.size == measurements.size) {
                            expected
                        } else {
                            null
                        }
                    },
            )

        assertSame(expected, result)
        assertEquals(
            listOf(
                "GPS:ppp:HIGH:5",
                "GPS:ppp:MEDIUM:5",
                "GPS:code:HIGH:5",
            ),
            attempts,
        )
    }

    @Test
    fun resolve_fallsBackToSingleConstellationCodeOnlySolve() {
        val gps = gpsMeasurements(count = 4)
        val galileo = constellationMeasurements(Constellation.GALILEO, startSvid = 11, count = 3)
        val allMeasurements = gps + galileo
        val expected = sampleResult(latitude = 2.0)
        val attempts = mutableListOf<String>()

        val result =
            RawGnssPositionFallbackResolver.resolve(
                measurements = allMeasurements,
                initialLat = 48.8,
                initialLon = 2.3,
                initialAlt = 100.0,
                gpsTimeNanos = 123L,
                primaryAccuracy = GnssPositionSolver.Accuracy.HIGH,
                fallbackConstellationExclusions = intArrayOf(Constellation.GLONASS),
                positionComputer =
                    RawGnssPositionComputer { candidateMeasurements, _, _, _, usePpp, _, accuracy ->
                        attempts += attemptKey(candidateMeasurements, usePpp, accuracy)
                        val onlyGps = candidateMeasurements.all { it.constellationType == Constellation.GPS }
                        if (onlyGps && !usePpp && accuracy == GnssPositionSolver.Accuracy.HIGH && candidateMeasurements.size == 4) {
                            expected
                        } else {
                            null
                        }
                    },
            )

        assertSame(expected, result)
        assertTrue(attempts.contains("GPS+GAL:ppp:HIGH:7"))
        assertTrue(attempts.contains("GPS+GAL:code:MEDIUM:7"))
        assertTrue(attempts.contains("GPS:code:HIGH:4"))
    }

    @Test
    fun resolve_fallsBackToLargestMultiConstellationSubsetBeforeSingleConstellation() {
        val gps = gpsMeasurements(count = 3)
        val galileo = constellationMeasurements(Constellation.GALILEO, startSvid = 11, count = 3)
        val beidou = constellationMeasurements(Constellation.BEIDOU, startSvid = 21, count = 2)
        val qzss = constellationMeasurements(Constellation.QZSS, startSvid = 31, count = 1)
        val allMeasurements = gps + galileo + beidou + qzss
        val expected = sampleResult(latitude = 3.0)
        val attempts = mutableListOf<String>()

        val result =
            RawGnssPositionFallbackResolver.resolve(
                measurements = allMeasurements,
                initialLat = 48.8,
                initialLon = 2.3,
                initialAlt = 100.0,
                gpsTimeNanos = 123L,
                primaryAccuracy = GnssPositionSolver.Accuracy.HIGH,
                fallbackConstellationExclusions = intArrayOf(Constellation.GLONASS),
                positionComputer =
                    RawGnssPositionComputer { candidateMeasurements, _, _, _, usePpp, _, accuracy ->
                        attempts += attemptKey(candidateMeasurements, usePpp, accuracy)
                        val usesTopTwoConstellations =
                            candidateMeasurements.size == 6 &&
                                candidateMeasurements.all {
                                    it.constellationType == Constellation.GPS ||
                                        it.constellationType == Constellation.GALILEO
                                }
                        if (usesTopTwoConstellations && !usePpp && accuracy == GnssPositionSolver.Accuracy.HIGH) {
                            expected
                        } else {
                            null
                        }
                    },
            )

        assertSame(expected, result)
        assertTrue(attempts.any { it.endsWith(":ppp:HIGH:9") })
        assertTrue(
            attempts.any { attempt ->
                attempt.endsWith(":code:HIGH:6") &&
                    attempt.contains("GPS") &&
                    attempt.contains("GAL")
            },
        )
    }

    @Test
    fun resolve_stopsCombinatorialFallbackOnceBudgetExhausted() {
        val gps = gpsMeasurements(count = 4)
        val galileo = constellationMeasurements(Constellation.GALILEO, startSvid = 11, count = 3)
        val allMeasurements = gps + galileo
        val attempts = mutableListOf<String>()

        // First clock read (after the full-set attempt) establishes the deadline at t=0; every later
        // read is well past the 1.5 s budget, so the combinatorial fallback stages are skipped.
        var clockCalls = 0
        val clock = { if (clockCalls++ == 0) 0L else 10_000_000_000L }

        val result =
            RawGnssPositionFallbackResolver.resolve(
                measurements = allMeasurements,
                initialLat = 48.8,
                initialLon = 2.3,
                initialAlt = 100.0,
                gpsTimeNanos = 123L,
                primaryAccuracy = GnssPositionSolver.Accuracy.HIGH,
                fallbackConstellationExclusions = intArrayOf(Constellation.GLONASS),
                positionComputer =
                    RawGnssPositionComputer { candidateMeasurements, _, _, _, usePpp, _, accuracy ->
                        attempts += attemptKey(candidateMeasurements, usePpp, accuracy)
                        // Never solves: without a budget this would try every fallback combination.
                        null
                    },
                fallbackBudgetMs = 1_500L,
                nowNanos = clock,
            )

        assertNull(result)
        // The full-set attempt always runs (it is not charged against the fallback budget)...
        assertTrue(attempts.contains("GPS+GAL:ppp:HIGH:7"))
        // ...but the budget stops the search before the single-constellation GPS-only solve it would
        // otherwise reach.
        assertTrue(attempts.none { it == "GPS:code:HIGH:4" })
    }

    private fun gpsMeasurements(count: Int): List<SatelliteMeasurement> =
        constellationMeasurements(
            constellationType = Constellation.GPS,
            startSvid = 1,
            count = count,
        )

    private fun constellationMeasurements(
        constellationType: Int,
        startSvid: Int,
        count: Int,
    ): List<SatelliteMeasurement> =
        (0 until count).map { index ->
            SatelliteMeasurement(
                constellationType = constellationType,
                svid = startSvid + index,
                pseudorangeMeters = 20_000_000.0 + index,
                carrierFrequencyHz = 1.57542e9,
                cn0DbHz = 40.0,
                elevationDegrees = 45.0,
                azimuthDegrees = (index * 45).toDouble(),
                accumulatedDeltaRangeMeters = null,
                pseudorangeUncertaintyMeters = 1.0,
            )
        }

    private fun sampleResult(latitude: Double) =
        PositionResult(
            latitude = latitude,
            longitude = 2.3,
            altitude = 100.0,
            horizontalAccuracyMeters = 6f,
            verticalAccuracyMeters = 9f,
            timestamp = 123L,
            satelliteCount = 4,
            usedInFixCount = 4,
        )

    private fun attemptKey(
        measurements: List<SatelliteMeasurement>,
        usePpp: Boolean,
        accuracy: GnssPositionSolver.Accuracy,
    ): String {
        val label =
            measurements
                .map { it.constellationType }
                .distinct()
                .sorted()
                .joinToString("+") { constellationType ->
                    when (constellationType) {
                        Constellation.GPS -> "GPS"
                        Constellation.GALILEO -> "GAL"
                        Constellation.GLONASS -> "GLO"
                        Constellation.BEIDOU -> "BDS"
                        Constellation.QZSS -> "QZS"
                        else -> constellationType.toString()
                    }
                }
        val mode = if (usePpp) "ppp" else "code"
        return "$label:$mode:${accuracy.name}:${measurements.size}"
    }
}
