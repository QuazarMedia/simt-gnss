package app.simt.gnss.core.filter

import app.simt.gnss.core.utils.GnssMathUtils
import kotlin.math.abs

/**
 * Whether a measurement's pseudorange rate can be trusted to predict how far the range to that
 * satellite will move between two epochs.
 *
 * This is the one gate both smoothers need, and it is here rather than in either of them because the
 * answer has to be the same in both. [DopplerSmoother] uses the rate to *predict* the next
 * pseudorange and [HatchFilter] uses it to decide whether a carrier delta is physically possible;
 * if the two disagreed about which rates were usable, a signal could be smoothed by one filter and
 * declared slipped by the other in the same epoch, which is not a decision any observer could reason
 * about.
 *
 * The magnitude and uncertainty limits are why this is more than a bare `isFinite()`: a receiver
 * that has lost lock often still reports a rate, and that rate is meaningless.
 *
 * **On the zero-uncertainty rule.** The platform measurement stores the rate and its
 * 1-sigma as plain `double` fields with no "has rate" flag and no `NaN` sentinel, so a receiver that
 * has nothing to report hands over `0.0` for both — which is indistinguishable, by construction,
 * from a satellite whose range rate genuinely happens to be zero. The uncertainty is what separates
 * them: no tracking loop reports exactly zero 1-sigma for a Doppler measurement, so a reported
 * uncertainty of `0.0` is a missing value. Reading the pair as "the satellite is stationary" is not
 * a harmless reading — a static receiver still sees hundreds of metres per second of satellite
 * motion, so [HatchFilter] would declare every one of those epochs slipped and reset, which is the
 * failure this whole check exists to prevent, and [DopplerSmoother] would blend its output towards a
 * prediction frozen at the previous epoch. Erring towards "not reported" is the safe direction for
 * both: a signal with no usable rate falls back to the flag- and code-based detectors rather than to
 * a confidently wrong answer.
 *
 * Pure and top-level so it is testable without a [HatchFilter] or a `Recomposer`.
 */
internal fun isPseudorangeRateReliable(
    rateMetersPerSecond: Double,
    rateUncertaintyMetersPerSecond: Double,
): Boolean =
    rateMetersPerSecond.isFinite() &&
        rateUncertaintyMetersPerSecond.isFinite() &&
        rateUncertaintyMetersPerSecond > 0.0 &&
        rateUncertaintyMetersPerSecond <= GnssMathUtils.DOPPLER_MAX_RATE_UNCERTAINTY_MPS &&
        abs(rateMetersPerSecond) <= GnssMathUtils.DOPPLER_MAX_ABS_RATE_MPS

/**
 * Whether a carrier delta agrees with the pseudorange rate over [dtSeconds].
 *
 * Null when the rate cannot be used, which is not a slip: the caller decides whether an untested
 * delta may pass. False when `|ΔADR − rate·Δt|` exceeds the #83 tolerance. The clock step is not
 * part of this residual; the carrier and the rate share the hardware clock.
 */
fun carrierDeltaMatchesPseudorangeRate(
    currentAdrMeters: Double,
    previousAdrMeters: Double,
    dtSeconds: Double,
    rateMetersPerSecond: Double,
    rateUncertaintyMetersPerSecond: Double,
): Boolean? {
    if (dtSeconds <= 0.0 || dtSeconds > GnssMathUtils.DOPPLER_MAX_DT_SECONDS) return null
    if (!isPseudorangeRateReliable(rateMetersPerSecond, rateUncertaintyMetersPerSecond)) return null
    val tolerance =
        maxOf(
            GnssMathUtils.ADR_DOPPLER_SLIP_FLOOR_METERS,
            GnssMathUtils.ADR_DOPPLER_SLIP_SIGMA * rateUncertaintyMetersPerSecond * dtSeconds,
        )
    val residual = abs((currentAdrMeters - previousAdrMeters) - rateMetersPerSecond * dtSeconds)
    return residual <= tolerance
}
