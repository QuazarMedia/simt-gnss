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

    @Test
    fun oneCycleSlipOnHighLeverageSatellite_isCaughtByLeverageNormalizedResidual() {
        // 6-SV constellation: SV 0 at zenith (z=1.0), SVs 1..3 at z=0.2 (120 deg spacing),
        // SVs 4..5 at z=0.7 (180 deg spacing). Analytical leverage of SV 0 is h_00 = 7/12 ≈ 0.583,
        // so a 1-cycle L1 slip (0.19 m) yields raw post-fit residual (1 - 7/12) * 0.19 ≈ 0.079 m < 0.10 m,
        // escaping an unnormalized check, while leverage-normalized |r_0|/sqrt(1 - h_00) ≈ 0.123 m > 0.10 m
        // catches and excludes SV 0, recovering the true displacement.
        val delta = doubleArrayOf(0.35, -0.15, 0.25)
        val receiverClock = 0.2
        val oneCycleSlipMeters = 0.19
        val rLow = sqrt(1.0 - 0.2 * 0.2)
        val rHigh = sqrt(1.0 - 0.7 * 0.7)
        val directions =
            listOf(
                doubleArrayOf(0.0, 0.0, 1.0),
                doubleArrayOf(rLow, 0.0, 0.2),
                doubleArrayOf(rLow * kotlin.math.cos(Math.toRadians(120.0)), rLow * kotlin.math.sin(Math.toRadians(120.0)), 0.2),
                doubleArrayOf(rLow * kotlin.math.cos(Math.toRadians(240.0)), rLow * kotlin.math.sin(Math.toRadians(240.0)), 0.2),
                doubleArrayOf(rHigh, 0.0, 0.7),
                doubleArrayOf(-rHigh, 0.0, 0.7),
            ).map { unit(it) }

        val satellites =
            directions.mapIndexed { index, los ->
                var along = 0.0
                for (i in 0..2) along += los[i] * delta[i]
                val geometric = 40.0
                val satelliteClockMeters = 0.1
                val compensated = (-along + receiverClock) + geometric - satelliteClockMeters
                val adrDelta = compensated + CLOCK_STEP + if (index == 0) oneCycleSlipMeters else 0.0
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

        val solved = TdcpDisplacementSolver.solve(satellites, CLOCK_STEP)!!
        assertEquals(delta[0], solved.deltaX, 1e-6)
        assertEquals(delta[1], solved.deltaY, 1e-6)
        assertEquals(delta[2], solved.deltaZ, 1e-6)
        assertTrue(solved.varianceX >= TdcpDisplacementSolver.MIN_DISPLACEMENT_VARIANCE_M2)
        assertEquals(solved.varianceX, solved.covarianceEcef[0][0], 1e-12)
        assertEquals(solved.covarianceEcef[0][1], solved.covarianceEcef[1][0], 1e-12)
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
