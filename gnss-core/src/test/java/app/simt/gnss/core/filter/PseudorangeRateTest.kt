package app.simt.gnss.core.filter

import app.simt.gnss.core.utils.GnssMathUtils
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The gate both smoothers share, asserted directly so the rules are pinned where they live rather
 * than only through the behaviour of whichever filter a change happened to touch.
 */
class PseudorangeRateTest {
    @Test
    fun aRateWithAPlausibleUncertainty_isReliable() {
        assertTrue(
            isPseudorangeRateReliable(
                rateMetersPerSecond = -820.0,
                rateUncertaintyMetersPerSecond = 0.05,
            ),
        )
    }

    @Test
    fun aStationarySatelliteWithARealUncertainty_isReliable() {
        // A rate of exactly zero is only rejected because of its uncertainty. Zero is the one value
        // a missing report shares with a real measurement, and the uncertainty is the only thing
        // that tells them apart.
        assertTrue(
            isPseudorangeRateReliable(
                rateMetersPerSecond = 0.0,
                rateUncertaintyMetersPerSecond = 0.02,
            ),
        )
    }

    @Test
    fun anAbsentRate_isNotReliable() {
        // The platform rate field has no "has rate" flag and no NaN sentinel, so an absent rate arrives as
        // 0.0 with a 0.0 uncertainty. Reading that pair as "the satellite is not moving" would make
        // HatchFilter reset on every epoch of every satellite — the failure this gate guards.
        assertFalse(
            "an unreported rate must not be trusted",
            isPseudorangeRateReliable(
                rateMetersPerSecond = 0.0,
                rateUncertaintyMetersPerSecond = 0.0,
            ),
        )
    }

    @Test
    fun aNonFiniteValue_isNotReliable() {
        assertFalse(isPseudorangeRateReliable(Double.NaN, 0.05))
        assertFalse(isPseudorangeRateReliable(-820.0, Double.NaN))
        assertFalse(isPseudorangeRateReliable(Double.POSITIVE_INFINITY, 0.05))
    }

    @Test
    fun anImplausibleUncertainty_isNotReliable() {
        // Above the module ceiling the rate is a number without a meaning attached, and predicting
        // with it would be worse than not predicting.
        assertFalse(
            isPseudorangeRateReliable(
                rateMetersPerSecond = -820.0,
                rateUncertaintyMetersPerSecond = GnssMathUtils.DOPPLER_MAX_RATE_UNCERTAINTY_MPS + 0.1,
            ),
        )
    }

    @Test
    fun anImplausibleMagnitude_isNotReliable() {
        // A receiver that has lost lock still reports a rate, and that rate is noise. The ceiling is
        // above anything a real line of sight produces, so it only rejects garbage.
        assertFalse(
            isPseudorangeRateReliable(
                rateMetersPerSecond = GnssMathUtils.DOPPLER_MAX_ABS_RATE_MPS + 1.0,
                rateUncertaintyMetersPerSecond = 0.05,
            ),
        )
    }
}
