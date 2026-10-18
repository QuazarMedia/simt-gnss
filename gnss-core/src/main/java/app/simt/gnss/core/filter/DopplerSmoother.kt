package app.simt.gnss.core.filter

import app.simt.gnss.core.utils.GnssMathUtils
import kotlin.math.abs

/**
 * Pseudorange smoothing using Doppler (pseudorange-rate) instead of carrier phase.
 * Useful when carrier phase lock is frequently interrupted (e.g., GNSS duty cycling).
 *
 * State is keyed **per signal** — `(constellation, svid, frequency band)`. With a per-satellite
 * key, the second frequency of a dual-band satellite always saw `dt == 0` within an epoch
 * (never smoothed) while the first was predicted from the *other* band's pseudorange baseline,
 * cross-contaminating the bands by their iono/inter-signal offset (meters).
 */
class DopplerSmoother {
    private val smoothedPseudoranges = mutableMapOf<Triple<Int, Int, Int>, Double>()
    private val lastEpochNanos = mutableMapOf<Triple<Int, Int, Int>, Long>()
    private val lastUpdatedEpoch = mutableMapOf<Triple<Int, Int, Int>, Long>()
    private var measurementEpoch: Long = 0L

    /**
     * Call once at the start of each measurement epoch to evict stale state.
     * @see [HatchFilter.beginNewMeasurementsEpoch]
     */
    fun beginNewMeasurementsEpoch() {
        measurementEpoch++
        val minKeep = measurementEpoch - GnssMathUtils.SMOOTHER_MAX_STALE_EPOCHS
        val staleKeys =
            smoothedPseudoranges.keys.filter { key ->
                (lastUpdatedEpoch[key] ?: Long.MIN_VALUE) < minKeep
            }
        staleKeys.forEach { signalKey ->
            smoothedPseudoranges.remove(signalKey)
            lastEpochNanos.remove(signalKey)
            lastUpdatedEpoch.remove(signalKey)
        }
    }

    /**
     * Smooth one signal's pseudorange for one epoch.
     *
     * @param clockStepMeters the receiver clock correction's change since the previous epoch, in
     *   metres of range, from [receiverClockStepMeters]. Subtracted from the prediction, because
     *   the pseudorange is corrected by the clock and the reported rate is not.
     */
    fun smooth(
        signalKey: Triple<Int, Int, Int>,
        rawPseudorange: Double,
        pseudorangeRateMetersPerSecond: Double,
        pseudorangeRateUncertaintyMetersPerSecond: Double,
        epochTimeNanos: Long,
        clockStepMeters: Double,
    ): Double {
        val rateIsReliable =
            isPseudorangeRateReliable(
                rateMetersPerSecond = pseudorangeRateMetersPerSecond,
                rateUncertaintyMetersPerSecond = pseudorangeRateUncertaintyMetersPerSecond,
            )

        val previousSmoothed = smoothedPseudoranges[signalKey]
        val previousEpoch = lastEpochNanos[signalKey]

        val smoothed =
            if (rateIsReliable && previousSmoothed != null && previousEpoch != null) {
                val dtSeconds = (epochTimeNanos - previousEpoch).toDouble() / GnssMathUtils.NANOS_PER_SECOND.toDouble()

                if (dtSeconds > 0.0 && dtSeconds <= GnssMathUtils.DOPPLER_MAX_DT_SECONDS) {
                    // The clock step, not the geometry. `pseudorangeRateMetersPerSecond` is the rate
                    // of the *uncorrected* range, and `previousSmoothed` came from a pseudorange the
                    // platform had already corrected by `C(k-1)`, so without this term the prediction
                    // sits a whole correction step above the observation and the blend charges the
                    // difference to the raw pseudorange as if the receiver's code were noisy. A refit
                    // of 334 ns is 100 m: 20 m of real code noise behind a 100 m prediction error
                    // produces an estimate three times *worse* than no smoothing at all, which is how
                    // this smoother ended up adding error rather than removing it.
                    //
                    // This also repairs the gate below. It compares the candidate against the raw
                    // pseudorange, and until now that comparison was between two quantities in
                    // different clock domains, so a step inside the 150 m threshold passed as noise
                    // and a step outside it threw away a perfectly good estimate — per signal, so the
                    // satellites rejected at different epochs and handed the estimator a decorrelated
                    // spread where a common-mode term belongs. Same domain on both sides is the fix;
                    // the gate itself is unchanged.
                    val predicted =
                        previousSmoothed + pseudorangeRateMetersPerSecond * dtSeconds - clockStepMeters
                    val alpha = GnssMathUtils.DOPPLER_SMOOTHING_ALPHA
                    val candidate = alpha * rawPseudorange + (1.0 - alpha) * predicted

                    if (abs(candidate - rawPseudorange) <= GnssMathUtils.DOPPLER_MAX_RAW_DEVIATION_METERS) {
                        candidate
                    } else {
                        rawPseudorange
                    }
                } else {
                    rawPseudorange
                }
            } else {
                rawPseudorange
            }

        smoothedPseudoranges[signalKey] = smoothed
        lastEpochNanos[signalKey] = epochTimeNanos
        lastUpdatedEpoch[signalKey] = measurementEpoch
        return smoothed
    }

    fun reset() {
        smoothedPseudoranges.clear()
        lastEpochNanos.clear()
        lastUpdatedEpoch.clear()
        measurementEpoch = 0L
    }
}
