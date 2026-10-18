package app.simt.gnss.core.solver
import app.simt.gnss.core.filter.carrierDeltaMatchesPseudorangeRate
import app.simt.gnss.core.filter.isPseudorangeRateReliable
import app.simt.gnss.core.model.AdrState
import app.simt.gnss.core.utils.GnssMathUtils

/**
 * Double-difference ambiguities carried from one RTK epoch to the next.
 *
 * The value stored is the previous posterior, plus a small process noise so the variance cannot
 * lock at zero. A satellite whose carrier was not tested against the previous epoch — a slip, a
 * missing rate, a half-cycle, an invalid ADR, or a gap outside (0, 5] seconds — contributes no prior.
 */
class RtkAmbiguityMemory {
    data class Sample(
        val constellationType: Int,
        val svid: Int,
        val adrMeters: Double?,
        val adrState: Int,
        val rateMetersPerSecond: Double,
        val rateUncertaintyMetersPerSecond: Double,
        val halfCycleAmbiguous: Boolean,
    )

    private var epochNanos: Long = 0L
    private val previousAdr = HashMap<Pair<Int, Int>, Double>()
    private var states: List<RtkDoubleDifferenceSolver.AmbiguityState> = emptyList()

    fun clear() {
        epochNanos = 0L
        previousAdr.clear()
        states = emptyList()
    }

    fun applicablePriors(
        samples: List<Sample>,
        timeNanos: Long,
    ): List<RtkDoubleDifferenceSolver.AmbiguityState> {
        val dtSeconds = (timeNanos - epochNanos) / 1e9
        if (dtSeconds <= 0.0 || dtSeconds > GnssMathUtils.DOPPLER_MAX_DT_SECONDS || states.isEmpty()) {
            return emptyList()
        }
        val rejected = HashSet<Pair<Int, Int>>()
        val seen = HashSet<Pair<Int, Int>>()
        for (sample in samples) {
            val key = sample.constellationType to sample.svid
            seen += key
            val adr = sample.adrMeters
            val previous = previousAdr[key]
            if (adr == null || previous == null || !carrierUsable(sample)) {
                rejected += key
                continue
            }
            if (carrierDeltaMatchesPseudorangeRate(
                    currentAdrMeters = adr,
                    previousAdrMeters = previous,
                    dtSeconds = dtSeconds,
                    rateMetersPerSecond = sample.rateMetersPerSecond,
                    rateUncertaintyMetersPerSecond = sample.rateUncertaintyMetersPerSecond,
                ) != true
            ) {
                rejected += key
            }
        }
        return states.filter { state ->
            val sat = state.constellationType to state.satSvid
            val ref = state.constellationType to state.refSvid
            sat in seen && ref in seen && sat !in rejected && ref !in rejected
        }
    }

    fun remember(
        floatStates: List<RtkDoubleDifferenceSolver.AmbiguityState>,
        samples: List<Sample>,
        timeNanos: Long,
    ) {
        val trusted = HashSet<Pair<Int, Int>>()
        val nextAdr = HashMap<Pair<Int, Int>, Double>()
        for (sample in samples) {
            val adr = sample.adrMeters
            if (adr == null || !adr.isFinite() || !carrierUsable(sample)) continue
            val key = sample.constellationType to sample.svid
            nextAdr[key] = adr
            if (isPseudorangeRateReliable(sample.rateMetersPerSecond, sample.rateUncertaintyMetersPerSecond)) {
                trusted += key
            }
        }
        states =
            floatStates
                .filter { state ->
                    (state.constellationType to state.satSvid) in trusted &&
                        (state.constellationType to state.refSvid) in trusted
                }.map { state ->
                    state.copy(varianceCycles2 = state.varianceCycles2 + PROCESS_NOISE_CYCLES2)
                }
        previousAdr.clear()
        previousAdr.putAll(nextAdr)
        epochNanos = timeNanos
    }

    private fun carrierUsable(sample: Sample): Boolean {
        val adr = sample.adrMeters
        return adr != null &&
            adr.isFinite() &&
            !sample.halfCycleAmbiguous &&
            (sample.adrState and AdrState.VALID) != 0 &&
            (sample.adrState and AdrState.CYCLE_SLIP) == 0 &&
            (sample.adrState and AdrState.RESET) == 0
    }

    private companion object {
        /** About 0.01 cycle. Stops a posterior of zero from freezing the next epoch's integer. */
        const val PROCESS_NOISE_CYCLES2 = 1e-4
    }
}
