package app.simt.gnss.core.solver

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.sqrt

class DopplerVelocitySolverTest {
    @Test
    fun knownVelocityAndClockDrift_areRecovered() {
        val velocity = doubleArrayOf(10.0, -3.0, 1.0)
        val clockDrift = 2.5
        val observations =
            directions().mapIndexed { index, los ->
                val satVelocity = doubleArrayOf(100.0 * index, -40.0, 20.0)
                val satClock = 0.1 * index
                observation(los, satVelocity, satClock, velocity, clockDrift, sigma = 0.2)
            }
        val solved = DopplerVelocitySolver.solve(observations)!!
        assertEquals(velocity[0], solved.velocityX, 1e-6)
        assertEquals(velocity[1], solved.velocityY, 1e-6)
        assertEquals(velocity[2], solved.velocityZ, 1e-6)
        assertEquals(clockDrift, solved.clockDriftMetersPerSecond, 1e-6)
        assertTrue(solved.varianceX > 0.0)
    }

    @Test
    fun oneCorruptRate_isExcludedAndTheVelocityStands() {
        val velocity = doubleArrayOf(4.0, 0.0, -2.0)
        val clockDrift = -1.0
        val sky =
            listOf(
                doubleArrayOf(1.0, 1.0, 1.0),
                doubleArrayOf(1.0, -1.0, 1.0),
                doubleArrayOf(-1.0, 1.0, 1.0),
                doubleArrayOf(-1.0, -1.0, 1.0),
                doubleArrayOf(1.0, 1.0, -1.0),
                doubleArrayOf(1.0, -1.0, -1.0),
                doubleArrayOf(-1.0, 1.0, -1.0),
                doubleArrayOf(-1.0, -1.0, -0.2),
            ).map { unit(it) }
        val observations =
            sky.mapIndexed { index, los ->
                val satVelocity = doubleArrayOf(0.0, 50.0, -30.0)
                observation(los, satVelocity, satClock = 0.0, velocity, clockDrift, sigma = 0.1)
                    .let { obs ->
                        if (index == 0) obs.copy(pseudorangeRateMetersPerSecond = obs.pseudorangeRateMetersPerSecond + 80.0) else obs
                    }
            }
        val solved = DopplerVelocitySolver.solve(observations)!!
        assertEquals(velocity[0], solved.velocityX, 1e-4)
        assertEquals(velocity[1], solved.velocityY, 1e-4)
        assertEquals(velocity[2], solved.velocityZ, 1e-4)
        assertEquals(clockDrift, solved.clockDriftMetersPerSecond, 1e-4)
    }

    @Test
    fun fewerThanFiveRates_returnsNull() {
        val velocity = doubleArrayOf(0.0, 0.0, 0.0)
        val observations = directions().take(4).map { observation(it, doubleArrayOf(0.0, 0.0, 0.0), 0.0, velocity, 0.0, 0.2) }
        assertNull(DopplerVelocitySolver.solve(observations))
    }

    private fun directions(): List<DoubleArray> =
        listOf(
            doubleArrayOf(1.0, 1.0, 1.0),
            doubleArrayOf(1.0, -1.0, 1.0),
            doubleArrayOf(-1.0, 1.0, 1.0),
            doubleArrayOf(-1.0, -1.0, 1.0),
            doubleArrayOf(1.0, 0.2, -0.4),
            doubleArrayOf(-0.3, 1.0, -0.5),
        ).map { unit(it) }

    private fun observation(
        los: DoubleArray,
        satVelocity: DoubleArray,
        satClock: Double,
        receiverVelocity: DoubleArray,
        clockDrift: Double,
        sigma: Double,
    ): DopplerVelocitySolver.Observation {
        var geometric = 0.0
        for (i in 0..2) geometric += los[i] * (satVelocity[i] - receiverVelocity[i])
        val rate = geometric + clockDrift - satClock
        return DopplerVelocitySolver.Observation(
            losX = los[0],
            losY = los[1],
            losZ = los[2],
            satelliteVelocityX = satVelocity[0],
            satelliteVelocityY = satVelocity[1],
            satelliteVelocityZ = satVelocity[2],
            satelliteClockDriftMetersPerSecond = satClock,
            pseudorangeRateMetersPerSecond = rate,
            sigmaMetersPerSecond = sigma,
        )
    }

    private fun unit(v: DoubleArray): DoubleArray {
        val n = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
        return doubleArrayOf(v[0] / n, v[1] / n, v[2] / n)
    }
}
