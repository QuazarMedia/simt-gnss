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
    private val smoothingCount = mutableMapOf<Triple<Int, Int, Int>, Int>()
    private val lastEpochNanos = mutableMapOf<Triple<Int, Int, Int>, Long>()
    private val lastUpdatedEpoch = mutableMapOf<Triple<Int, Int, Int>, Long>()
    private val lastFullBiasNanos = mutableMapOf<Triple<Int, Int, Int>, Long>()
    private val lastBiasNanos = mutableMapOf<Triple<Int, Int, Int>, Double>()
    private var measurementEpoch: Long = 0L

    fun currentSmoothingCount(signalKey: Triple<Int, Int, Int>): Int = smoothingCount[signalKey] ?: 1

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
        staleKeys.forEach { removeSignalKey(it) }
    }

    private fun removeSignalKey(signalKey: Triple<Int, Int, Int>) {
        smoothedPseudoranges.remove(signalKey)
        smoothingCount.remove(signalKey)
        lastEpochNanos.remove(signalKey)
        lastUpdatedEpoch.remove(signalKey)
        lastFullBiasNanos.remove(signalKey)
        lastBiasNanos.remove(signalKey)
    }

    /**
     * Smooth one signal's pseudorange for one epoch.
     *
     * @param clockStepMeters fallback receiver clock step in metres when [fullBiasNanos] is null.
     * @param fullBiasNanos current epoch's `fullBiasNanos`, tracked per [signalKey] so signals
     *   returning from brief occlusion subtract the exact cumulative clock step since last seen.
     * @param biasNanos sub-nanosecond receiver clock bias paired with [fullBiasNanos].
     */
    fun smooth(
        signalKey: Triple<Int, Int, Int>,
        rawPseudorange: Double,
        pseudorangeRateMetersPerSecond: Double,
        pseudorangeRateUncertaintyMetersPerSecond: Double,
        epochTimeNanos: Long,
        clockStepMeters: Double = 0.0,
        fullBiasNanos: Long? = null,
        biasNanos: Double = 0.0,
    ): Double {
        val rateIsReliable =
            isPseudorangeRateReliable(
                rateMetersPerSecond = pseudorangeRateMetersPerSecond,
                rateUncertaintyMetersPerSecond = pseudorangeRateUncertaintyMetersPerSecond,
            )

        val previousSmoothed = smoothedPseudoranges[signalKey]
        val previousEpoch = lastEpochNanos[signalKey]
        val effectiveClockStepMeters =
            if (fullBiasNanos != null) {
                receiverClockStepMeters(
                    lastFullBiasNanos[signalKey],
                    fullBiasNanos,
                    lastBiasNanos[signalKey] ?: 0.0,
                    biasNanos,
                )
            } else {
                clockStepMeters
            }

        var usedSmoothed = false
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
                        previousSmoothed + pseudorangeRateMetersPerSecond * dtSeconds - effectiveClockStepMeters
                    val alpha = GnssMathUtils.DOPPLER_SMOOTHING_ALPHA
                    val candidate = alpha * rawPseudorange + (1.0 - alpha) * predicted

                    if (abs(candidate - rawPseudorange) <= GnssMathUtils.DOPPLER_MAX_RAW_DEVIATION_METERS) {
                        usedSmoothed = true
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

        smoothingCount[signalKey] = if (usedSmoothed) (smoothingCount[signalKey] ?: 1) + 1 else 1
        smoothedPseudoranges[signalKey] = smoothed
        lastEpochNanos[signalKey] = epochTimeNanos
        lastUpdatedEpoch[signalKey] = measurementEpoch
        if (fullBiasNanos != null) {
            lastFullBiasNanos[signalKey] = fullBiasNanos
            lastBiasNanos[signalKey] = biasNanos
        } else {
            lastFullBiasNanos.remove(signalKey)
            lastBiasNanos.remove(signalKey)
        }
        return smoothed
    }

    fun reset() {
        smoothedPseudoranges.clear()
        smoothingCount.clear()
        lastEpochNanos.clear()
        lastUpdatedEpoch.clear()
        lastFullBiasNanos.clear()
        lastBiasNanos.clear()
        measurementEpoch = 0L
    }
}
