package app.simt.gnss.core.filter

import app.simt.gnss.core.utils.GnssMathUtils

/**
 * The change in the receiver clock correction between two epochs, as a range.
 *
 * `fullBiasNanos` and `biasNanos` are the correction the platform applies to the hardware clock to
 * produce the GPS time the pseudorange is measured against. The carrier and the pseudorange rate
 * are not corrected, so a smoother that predicts from them and blends against the pseudorange is
 * off by `c·Δ(fullBias + bias)` every epoch. That step is common to every satellite, but each
 * signal blends it with its own smoothing weight, so what reaches the solution is per-satellite.
 *
 * `fullBiasNanos` is about 1.4×10¹⁸ ns. A `Double` at that magnitude has a 256 ns ulp (≈ 77 m), so
 * the difference has to be taken in `Long` and only the sub-nanosecond `biasNanos` term in
 * `Double`. The same trap is why receive time stays in `Long` (#79).
 *
 * Positive when the correction grows. Callers subtract it: the correction was removed from the
 * pseudorange, so a prediction built from an uncorrected rate or carrier is too high by this much.
 *
 * @param previousFullBiasNanos the previous epoch's `fullBiasNanos`, or `null` when there is no
 *   previous epoch (session start, or the first epoch after a hardware-clock discontinuity).
 * @return the step in metres, or zero when there is no previous epoch.
 */
fun receiverClockStepMeters(
    previousFullBiasNanos: Long?,
    currentFullBiasNanos: Long,
    previousBiasNanos: Double,
    currentBiasNanos: Double,
): Double {
    val previous = previousFullBiasNanos ?: return 0.0
    val deltaNanos = (currentFullBiasNanos - previous).toDouble() + (currentBiasNanos - previousBiasNanos)
    return deltaNanos / GnssMathUtils.NANOS_PER_SECOND * GnssMathUtils.SPEED_OF_LIGHT
}
