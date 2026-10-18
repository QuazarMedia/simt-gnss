package app.simt.gnss.core.solver

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Pins the per-(constellation, band) partitioning of RTK double differences: differential
 * inter-system biases (rover vs base hardware delays) cancel only when both satellites of a
 * DD belong to the same system and band.
 */
class RtkPartitionedDoubleDifferenceTest {
    private val rover = doubleArrayOf(4_000_000.0, 3_000_000.0, 3_500_000.0)
    private val base = doubleArrayOf(4_000_010.0, 3_000_005.0, 3_500_003.0)

    private fun distance(
        a: DoubleArray,
        b: DoubleArray,
    ): Double {
        val dx = a[0] - b[0]
        val dy = a[1] - b[1]
        val dz = a[2] - b[2]
        return sqrt(dx * dx + dy * dy + dz * dz)
    }

    /** Satellites spread over the sky at GPS-like radius. */
    private fun satellitePositions(
        count: Int,
        offsetDeg: Double,
    ): List<DoubleArray> =
        (0 until count).map { i ->
            val az = Math.toRadians(offsetDeg + i * (360.0 / count))
            val el = Math.toRadians(35.0 + 8.0 * i)
            val r = 26_560_000.0
            doubleArrayOf(
                r * cos(el) * cos(az),
                r * cos(el) * sin(az),
                r * sin(el),
            )
        }

    /**
     * Build exact observations with a per-constellation differential inter-system bias on
     * the rover side (phone vs geodetic base hardware delays).
     */
    private fun inputs(
        constellation: Int,
        sats: List<DoubleArray>,
        roverIsbMeters: Double,
        baseElevations: Double = 40.0,
    ): List<RtkDoubleDifferenceSolver.SatelliteInput> =
        sats.mapIndexed { i, sat ->
            RtkDoubleDifferenceSolver.SatelliteInput(
                satelliteEcef = sat,
                baseSatelliteEcef = sat,
                roverObservationMeters = distance(sat, rover) + roverIsbMeters,
                baseObservationMeters = distance(sat, base),
                elevationDeg = baseElevations + i,
                sigmaMeters = 1.0,
                constellationType = constellation,
                bandMhz = 1575,
            )
        }

    @Test
    fun crossConstellationIsb_cancelsWithPartitionedReferences() {
        // Constellation A unbiased, constellation B with a 2.5 m differential code ISB.
        // With per-constellation references the ISB cancels inside every DD and the rover
        // is recovered exactly; a single global reference would leak 2.5 m into every
        // cross-constellation row and pull the solution away by a metre-class error.
        val all =
            inputs(constellation = 1, sats = satellitePositions(4, 0.0), roverIsbMeters = 0.0) +
                inputs(constellation = 3, sats = satellitePositions(4, 45.0), roverIsbMeters = 2.5)

        val solution = RtkDoubleDifferenceSolver.solve(base, all, base.copyOf())
        assertNotNull(solution)
        val err = distance(solution!!.roverEcef, rover)
        assertTrue("ISB must cancel within partitions (error was $err m)", err < 0.01)
    }

    @Test
    fun singleConstellation_exactObservations_recoverRoverExactly() {
        val solution =
            RtkDoubleDifferenceSolver.solve(
                base,
                inputs(constellation = 1, sats = satellitePositions(5, 10.0), roverIsbMeters = 0.0),
                base.copyOf(),
            )
        assertNotNull(solution)
        assertEquals(0.0, distance(solution!!.roverEcef, rover), 1e-3)
    }

    @Test
    fun twoLoneConstellations_cannotFormEnoughDoubleDifferences() {
        // 2 + 2 satellites → one DD row per partition = 2 rows < 3 unknowns → must refuse
        // rather than difference across systems.
        val all =
            inputs(constellation = 1, sats = satellitePositions(2, 0.0), roverIsbMeters = 0.0) +
                inputs(constellation = 3, sats = satellitePositions(2, 90.0), roverIsbMeters = 1.0)
        assertNull(RtkDoubleDifferenceSolver.solve(base, all, base.copyOf()))
    }

    // ------------------------------------------------------------------ LAMBDA edge cases

    @Test
    fun lambda_floatExactlyOnIntegers_isAcceptedNotRejected() {
        // Float ambiguities sitting exactly on integers are the strongest possible fix;
        // the previous code reported ratio = 0 and rejected it.
        val floatAmb = doubleArrayOf(3.0, -2.0, 7.0, 1.0)
        val q =
            Array(4) { i ->
                DoubleArray(4) { j -> if (i == j) 1e-4 else 0.0 }
            }
        val result = LambdaAmbiguityResolver.resolve(floatAmb, q)
        assertNotNull(result)
        assertTrue("A perfect fix must pass the ratio test", result!!.accepted)
        for (i in 0 until 4) assertEquals(floatAmb[i], result.fixedAmbiguities[i], 0.0)
    }
}
