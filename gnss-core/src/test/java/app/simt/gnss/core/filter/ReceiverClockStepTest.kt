package app.simt.gnss.core.filter

import app.simt.gnss.core.utils.GnssMathUtils
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The receiver clock step, pinned against scenarios whose answers are known rather than against the
 * arithmetic. Inverting the sign makes the smoothers worse, and differencing `fullBiasNanos` in
 * `Double` quantizes a real refit onto a 256 ns grid, so both failures have a test that misses by
 * tens of metres if they regress.
 */
class ReceiverClockStepTest {
    @Test
    fun noPreviousEpoch_returnsZero() {
        assertEquals(0.0, receiverClockStepMeters(null, 123_456L, 0.0, 0.0), 0.0)
    }

    @Test
    fun aGrowingCorrection_yieldsAPositiveStep() {
        assertTrue(
            "a growing correction must yield a positive step",
            receiverClockStepMeters(0L, 0L, 0.0, REFIT_STEP_NANOS) > 0.0,
        )
    }

    @Test
    fun aOneSecondCorrectionGrows_exactlyTheSpeedOfLight() {
        assertEquals(
            GnssMathUtils.SPEED_OF_LIGHT,
            receiverClockStepMeters(0L, 1_000_000_000L, 0.0, 0.0),
            1e-3,
        )
    }

    @Test
    fun reversingTheEpochs_negatesTheStep() {
        val forward = receiverClockStepMeters(0L, 0L, 1_000.0, 1_000.0 + REFIT_STEP_NANOS)
        val backward = receiverClockStepMeters(0L, 0L, 1_000.0 + REFIT_STEP_NANOS, 1_000.0)
        assertEquals(forward, -backward, 1e-6)
    }

    @Test
    fun aRealisticRefitStep_isAboutOneHundredMetres() {
        val step = receiverClockStepMeters(0L, 0L, 0.0, REFIT_STEP_NANOS)
        assertEquals("a 334 ns refit must be ~100 m of range", 100.0, step, 1.0)
    }

    @Test
    fun fullBiasAtGpsEpochScale_keepsASubUlpStep() {
        // |fullBias| ≈ 1.4e18 ns. Double's ulp there is 256 ns, so a 334 ns refit differenced in
        // Double comes back as 256 ns — 23 m short of the 100 m the smoothers have to remove.
        // The Long difference must keep the whole 334 ns, bias fraction included.
        val base = -1_400_000_000_000_000_000L
        val wholeNanos = 334L
        val fractionNanos = REFIT_STEP_NANOS - wholeNanos
        val step =
            receiverClockStepMeters(
                previousFullBiasNanos = base,
                currentFullBiasNanos = base + wholeNanos,
                previousBiasNanos = 0.0,
                currentBiasNanos = fractionNanos,
            )
        val expected = REFIT_STEP_NANOS / 1e9 * GnssMathUtils.SPEED_OF_LIGHT
        assertEquals(expected, step, 1e-4)

        val doubleDiffNanos = (base + wholeNanos).toDouble() - base.toDouble()
        val doubleDiffMeters = doubleDiffNanos / 1e9 * GnssMathUtils.SPEED_OF_LIGHT
        assertTrue(
            "the Double difference of these two fullBias values must be the case this test exists " +
                "to reject (it was ${"%.2f".format(doubleDiffMeters)} m against ${"%.2f".format(expected)} m)",
            kotlin.math.abs(doubleDiffMeters - expected) > 10.0,
        )
    }

    private companion object {
        /** Exactly 100 m of range at the speed of light. */
        const val REFIT_STEP_NANOS = 333.5640952
    }
}
