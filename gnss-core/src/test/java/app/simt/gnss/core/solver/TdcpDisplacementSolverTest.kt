package app.simt.gnss.core.solver

import app.simt.gnss.core.filter.carrierDeltaMatchesPseudorangeRate
import app.simt.gnss.core.utils.GnssMathUtils.SPEED_OF_LIGHT
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.sqrt

class TdcpDisplacementSolverTest {
    @Test
    fun knownDisplacement_clockStepAndSatelliteClock_areRecovered() {
        val delta = doubleArrayOf(0.4, -0.2, 0.1)
        val receiverClock = 0.3
        val solved = TdcpDisplacementSolver.solve(sky(delta, receiverClock, slipMeters = 0.0), CLOCK_STEP)!!
        assertEquals(delta[0], solved.deltaX, 1e-6)
        assertEquals(delta[1], solved.deltaY, 1e-6)
        assertEquals(delta[2], solved.deltaZ, 1e-6)
        assertTrue(solved.varianceX > 0.0)
    }

    @Test
    fun slipBelowTheDopplerFloor_isExcludedAndTheDisplacementStands() {
        val delta = doubleArrayOf(0.4, -0.2, 0.1)
        val rateDt = 50.0
        assertTrue(
            carrierDeltaMatchesPseudorangeRate(
                currentAdrMeters = rateDt + SLIP_METERS,
                previousAdrMeters = 0.0,
                dtSeconds = 1.0,
                rateMetersPerSecond = rateDt,
                rateUncertaintyMetersPerSecond = 0.05,
            ) == true,
        )
        val solved = TdcpDisplacementSolver.solve(sky(delta, receiverClock = 0.3, slipMeters = SLIP_METERS), CLOCK_STEP)!!
        assertEquals(delta[0], solved.deltaX, 1e-4)
        assertEquals(delta[1], solved.deltaY, 1e-4)
        assertEquals(delta[2], solved.deltaZ, 1e-4)
    }

    @Test
    fun dopplerSlipFloor_rejectsAHalfMetreCarrierDisagreement() {
        assertFalse(
            carrierDeltaMatchesPseudorangeRate(
                currentAdrMeters = 2.0,
                previousAdrMeters = 0.0,
                dtSeconds = 1.0,
                rateMetersPerSecond = 0.0,
                rateUncertaintyMetersPerSecond = 0.05,
            )!!,
        )
    }

    @Test
    fun fewerThanFiveSignals_returnsNull() {
        assertNull(TdcpDisplacementSolver.solve(sky(doubleArrayOf(0.0, 0.0, 0.0), 0.0, 0.0).take(4), CLOCK_STEP))
    }

    private fun sky(
        delta: DoubleArray,
        receiverClock: Double,
        slipMeters: Double,
    ): List<TdcpDisplacementSolver.Satellite> {
        val directions =
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
        return directions.mapIndexed { index, los ->
            var along = 0.0
            for (i in 0..2) along += los[i] * delta[i]
            val geometric = 50.0
            val satelliteClockMeters = 0.15
            val compensated = (-along + receiverClock) + geometric - satelliteClockMeters
            val adrDelta = compensated + CLOCK_STEP + if (index == 0) slipMeters else 0.0
            TdcpDisplacementSolver.Satellite(
                losX = los[0],
                losY = los[1],
                losZ = los[2],
                previousRangeMeters = 20_000_000.0,
                currentRangeMeters = 20_000_000.0 + geometric,
                previousAdrMeters = 1_000.0,
                currentAdrMeters = 1_000.0 + adrDelta,
                previousSatelliteClockSeconds = 0.0,
                currentSatelliteClockSeconds = satelliteClockMeters / SPEED_OF_LIGHT,
            )
        }
    }

    private fun unit(v: DoubleArray): DoubleArray {
        val n = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
        return doubleArrayOf(v[0] / n, v[1] / n, v[2] / n)
    }

    private companion object {
        const val CLOCK_STEP = 12.0
        const val SLIP_METERS = 0.30
    }
}
