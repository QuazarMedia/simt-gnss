package app.simt.gnss.core.filter
import app.simt.gnss.core.model.AdrState
import app.simt.gnss.core.utils.GnssMathUtils
import kotlin.math.abs
import kotlin.math.sqrt

/**
 * Implements Hatch filter for carrier-smoothed code.
 *
 * Filter state is keyed **per signal** — `(constellation, svid, frequency band)` — not per
 * satellite. On dual-frequency devices the L1 and L5 measurements of one satellite are
 * independent observables with unrelated ADR origins; sharing one filter state mixes their
 * ADR deltas (a chimera containing the arbitrary cross-band ADR offset), which either trips
 * the consistency check every epoch (silently disabling smoothing) or injects meter-level
 * cross-band bias into the smoothed pseudoranges.
 */
class HatchFilter {
    private val smoothedPseudoranges = mutableMapOf<Triple<Int, Int, Int>, Double>()
    private val smoothingCount = mutableMapOf<Triple<Int, Int, Int>, Int>()
    private val lastAdr = mutableMapOf<Triple<Int, Int, Int>, Double>()
    private val lastRaw = mutableMapOf<Triple<Int, Int, Int>, Double>()
    private val lastEpochNanos = mutableMapOf<Triple<Int, Int, Int>, Long>()
    private val lastUpdatedEpoch = mutableMapOf<Triple<Int, Int, Int>, Long>()
    private var measurementEpoch: Long = 0L

    /**
     * Call once at the start of each measurement epoch to age out signals
     * that have not appeared for [GnssMathUtils.SMOOTHER_MAX_STALE_EPOCHS] epochs.
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
        lastAdr.remove(signalKey)
        lastRaw.remove(signalKey)
        lastEpochNanos.remove(signalKey)
        lastUpdatedEpoch.remove(signalKey)
    }

    /**
     * Smoothes raw pseudorange using Accumulated Delta Range (ADR).
     *
     * @param signalKey (constellation, svid, carrier frequency in MHz)
     * @param epochTimeNanos receiver-clock time of this measurement, used only to turn the reported
     *   pseudorange rate into a predicted range change over [GnssMathUtils.SMOOTHER_MAX_STALE_EPOCHS]
     *   -style epochs. Monotonic within a session, which is all the prediction needs.
     * @param pseudorangeRateMetersPerSecond the platform's own range rate for this signal.
     * @param pseudorangeRateUncertaintyMetersPerSecond its 1-sigma, which widens the slip tolerance.
     * @param clockStepMeters the receiver clock correction's change since the previous epoch, in
     *   metres of range, from [receiverClockStepMeters]. Subtracted from the propagation and from
     *   the code-consistency residual, because the pseudorange is clock-corrected and the carrier is
     *   not.
     * @param codeSigmaMeters reported code uncertainty, in metres. Widens the consistency limit
     *   above [GnssMathUtils.ADR_CODE_CONSISTENCY_THRESHOLD_METERS]; zero keeps that floor.
     */
    fun smooth(
        signalKey: Triple<Int, Int, Int>,
        rawPseudorange: Double,
        currentAdr: Double?,
        adrState: Int,
        epochTimeNanos: Long,
        pseudorangeRateMetersPerSecond: Double,
        pseudorangeRateUncertaintyMetersPerSecond: Double,
        clockStepMeters: Double,
        codeSigmaMeters: Double = 0.0,
    ): Double {
        val hasValidAdr = currentAdr != null && (adrState and AdrState.VALID) != 0

        if (!hasValidAdr) {
            removeSignalKey(signalKey)
            return rawPseudorange
        }

        val prevSmoothed = smoothedPseudoranges[signalKey]
        val prevAdr = lastAdr[signalKey]
        val prevRaw = lastRaw[signalKey]
        val count = smoothingCount[signalKey] ?: 0

        // The seconds since this signal was last seen, which is the interval the Doppler prediction
        // below is integrated over. Zero or negative means the caller went backwards or repeated an
        // epoch, and no prediction over a non-positive interval is meaningful.
        val dtSeconds =
            lastEpochNanos[signalKey]?.let { previousNanos ->
                (epochTimeNanos - previousNanos).toDouble() / GnssMathUtils.NANOS_PER_SECOND.toDouble()
            }?.takeIf { it > 0.0 && it <= GnssMathUtils.DOPPLER_MAX_DT_SECONDS }

        // Cycle slip / rebase detection. Three independent sources, because each one covers a case
        // the others cannot see:
        //
        // 1. Android's CYCLE_SLIP flag. The receiver's own hardware-domain judgement, and the only
        //    source that can resolve a *sub-wavelength* slip — one L1 wavelength is 19 cm, which no
        //    threshold below can separate from carrier noise.
        // 2. Android's RESET flag — the accumulation base was re-established, so the inter-epoch ADR
        //    delta is meaningless regardless of magnitude (a rebase whose apparent delta is below the
        //    numeric thresholds would otherwise inject up to ~30 m of one-shot bias).
        // 3. Doppler residual. A carrier delta is geometry, and the platform reports the matching
        //    geometry as the pseudorange rate in the same clock domain, so the delta must equal
        //    rate × dt. Whatever is left over is a slip. This is the test that replaced a fixed
        //    |ΔADR| > 100 m "large jump" check, which was not measuring slips: it was reasoning about
        //    how far the *receiver* travels in a second and applying it to how far the *satellite*
        //    moves. Satellite range rates reach several hundred metres per second for a receiver that
        //    is not moving at all, so on a static watch that threshold was crossed on 88 % of epochs
        //    and the filter reset before it ever smoothed — the smoother was inert in production, not
        //    merely inaccurate.
        //
        // The rate test is skipped, rather than failed, when the receiver has no usable rate. There
        // is no per-signal physical prediction to judge the delta against in that case, and
        // resetting anyway is the behaviour this change is removing; the consistency check below and
        // the flags still apply.
        // The predicted range change, and the reason the clock step is absent here even though
        // neither the carrier nor the rate knows anything about the clock: the carrier delta
        // and the reported rate are both pure geometry, so they agree with each other
        // whatever the clock did, and this residual stays zero across a refit. That is the
        // property to keep — it is what makes the detector immune to the one fault it was
        // most likely to inherit, and it is why the step is subtracted in the two places
        // below rather than folded in here. Null means the rate could not be used, which is
        // not a slip.
        val dopplerSlip =
            if (prevAdr != null && dtSeconds != null) {
                carrierDeltaMatchesPseudorangeRate(
                    currentAdrMeters = currentAdr,
                    previousAdrMeters = prevAdr,
                    dtSeconds = dtSeconds,
                    rateMetersPerSecond = pseudorangeRateMetersPerSecond,
                    rateUncertaintyMetersPerSecond = pseudorangeRateUncertaintyMetersPerSecond,
                ) == false
            } else {
                false
            }

        // Consistency between the carrier delta and the code delta. Kept as the detector for signals
        // with no usable rate, and because it is the only test that reacts to a carrier or code
        // outlier that leaves both epochs' *change* intact.
        //
        // The clock step has to come out of this residual, and until it did this check was measuring
        // the receiver's clock rather than the satellites: a 334 ns refit moves every code
        // pseudorange by 100 m while leaving every carrier delta untouched, so the residual was 100 m
        // of pure receiver clock — three times the 30 m threshold, on every epoch of a refitting
        // receiver, for every satellite at once. The filter reset continuously and never smoothed,
        // which is the same failure as the one the Doppler detector replaced, arrived at from the
        // opposite direction. (Which is why the threshold is metres rather than centimetres even now:
        // it is a blunt instrument whose input is only comparable after the common-mode term is out,
        // and the remaining common-mode residue is the receiver clock's own sub-nanosecond wobble.)
        val codeResidualMeters =
            if (prevAdr != null && prevRaw != null) {
                abs((currentAdr - prevAdr - clockStepMeters) - (rawPseudorange - prevRaw))
            } else {
                null
            }
        val codeSigma = codeSigmaMeters.coerceIn(0.0, GnssMathUtils.MAX_PSEUDORANGE_SIGMA_METERS)
        val consistencyLimit =
            maxOf(
                GnssMathUtils.ADR_CODE_CONSISTENCY_THRESHOLD_METERS,
                GnssMathUtils.ADR_CODE_CONSISTENCY_SIGMA * sqrt(2.0) * codeSigma,
            )

        val isCycleSlip =
            (adrState and AdrState.CYCLE_SLIP) != 0 ||
                (adrState and AdrState.RESET) != 0 ||
                dopplerSlip ||
                (codeResidualMeters != null && codeResidualMeters > consistencyLimit)

        val smoothed =
            if (prevSmoothed != null && prevAdr != null && !isCycleSlip) {
                val k = (count + 1).toDouble().coerceAtMost(GnssMathUtils.HATCH_FILTER_MAX_SMOOTHING)
                val weight = 1.0 / k
                // The propagation subtracts the clock step, and this is the piece of the filter most likely to look
                // like an error on review — so the reasoning, including the earlier reasoning that was
                // wrong, is worth writing down.
                //
                // `prevSmoothed` was produced from a pseudorange the platform had already corrected by
                // `C(k-1)`. `currentAdr - prevAdr` is a *geometric* range change and carries no clock
                // term at all. The prediction therefore starts a whole correction step above where the
                // observation will be, and the blend charges that step to the raw pseudorange as if the
                // receiver's code were noisy. At 334 ns of refit that is 100 m of invented error on a
                // row whose purpose is to suppress invented error.
                //
                // Leaving it out was previously argued to be safe on the grounds that `c · Δe` is
                // common-mode across all satellites, and that the estimator's per-constellation clock
                // state absorbs it. The first half is true. The conclusion was not, for one specific
                // reason: common-mode absorbs a common *value*, and this is not a common value once the
                // filter has been running. Each signal carries its own smoothing age `k`, and the
                // propagated clock component is `(1 − 1/k) · (−c · C(k-1))` — so a signal on its third
                // epoch and a signal on its ninetieth blend the same two common-mode numbers with
                // different weights, and what reaches the position solution is satellite-dependent by
                // exactly that difference. A common-mode term applied at different strengths is
                // per-satellite bias. That is the whole of the correction, and it is why the term is
                // taken out here per signal rather than relied upon downstream.
                weight * rawPseudorange +
                    (1.0 - weight) * (prevSmoothed + (currentAdr - prevAdr) - clockStepMeters)
            } else {
                smoothingCount[signalKey] = 0 // Reset convergence on cycle slip
                rawPseudorange
            }

        smoothingCount[signalKey] = (smoothingCount[signalKey] ?: 0) + 1
        smoothedPseudoranges[signalKey] = smoothed
        lastAdr[signalKey] = currentAdr
        lastRaw[signalKey] = rawPseudorange
        lastEpochNanos[signalKey] = epochTimeNanos
        lastUpdatedEpoch[signalKey] = measurementEpoch
        return smoothed
    }

    fun reset() {
        smoothedPseudoranges.clear()
        smoothingCount.clear()
        lastAdr.clear()
        lastRaw.clear()
        lastEpochNanos.clear()
        lastUpdatedEpoch.clear()
        measurementEpoch = 0L
    }
}
