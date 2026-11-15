package app.simt.gnss.core.solver
import app.simt.gnss.core.filter.carrierDeltaMatchesPseudorangeRate
import app.simt.gnss.core.filter.isPseudorangeRateReliable
import app.simt.gnss.core.model.AdrState
import app.simt.gnss.core.utils.GnssMathUtils
import kotlin.math.roundToInt

/**
 * Double-difference ambiguities carried from one RTK epoch to the next.
 *
 * The value stored is the previous posterior, plus a small process noise so the variance cannot
 * lock at zero. A satellite whose carrier was not tested against the previous epoch — a slip, a
 * missing rate, a half-cycle, an invalid ADR, a base lock-time reset, or a gap outside (0, 5]
 * seconds — contributes no prior.
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
        val bandMhz: Int = 0,
        val baseLockTimeMillis: Long = -1L,
    )

    private data class SignalKey(
        val constellationType: Int,
        val svid: Int,
        val bandMhz: Int,
    )

    private var epochNanos: Long = 0L
    private val previousAdr = HashMap<SignalKey, Double>()
    private val previousBaseLockTimeMillis = HashMap<SignalKey, Long>()
    private var states: List<RtkDoubleDifferenceSolver.AmbiguityState> = emptyList()

    fun clear() {
        epochNanos = 0L
        previousAdr.clear()
        previousBaseLockTimeMillis.clear()
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
        val rejected = HashSet<SignalKey>()
        val seen = HashSet<SignalKey>()
        for (sample in samples) {
            val key = SignalKey(sample.constellationType, sample.svid, sample.bandMhz)
            seen += key
            val prevBaseLock = previousBaseLockTimeMillis[key]
            val currBaseLock = sample.baseLockTimeMillis
            if (prevBaseLock != null &&
                prevBaseLock >= 0L &&
                currBaseLock >= 0L &&
                (currBaseLock == 0L || currBaseLock < prevBaseLock)
            ) {
                rejected += key
                continue
            }
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

        val result = ArrayList<RtkDoubleDifferenceSolver.AmbiguityState>(states.size)
        val byPartitionAndRef =
            states.groupBy {
                Triple(it.constellationType, (it.wavelengthMeters * 1e5).roundToInt(), it.refSvid)
            }
        for ((key, groupStates) in byPartitionAndRef) {
            val constellation = key.first
            val oldRefSvid = key.third
            val bandMhz = bandMhzForWavelength(groupStates.first().wavelengthMeters)
            val refKey = resolveKey(seen, constellation, oldRefSvid, bandMhz)
            val refUsable = refKey != null && refKey !in rejected
            if (refUsable) {
                for (state in groupStates) {
                    val satKey = resolveKey(seen, constellation, state.satSvid, bandMhz)
                    if (satKey != null && satKey !in rejected) {
                        result += state
                    }
                }
            } else {
                // Old reference satellite dropped out or slipped: pivot surviving priors to one of the
                // surviving satellites so the remaining DD ambiguities remain usable without losing lock.
                val surviving =
                    groupStates.filter { state ->
                        val satKey = resolveKey(seen, constellation, state.satSvid, bandMhz)
                        satKey != null && satKey !in rejected
                    }
                if (surviving.size >= 2) {
                    val pivot = surviving.first()
                    for (state in surviving) {
                        if (state.satSvid == pivot.satSvid) continue
                        result +=
                            state.copy(
                                refSvid = pivot.satSvid,
                                cycles = state.cycles - pivot.cycles,
                                varianceCycles2 = state.varianceCycles2 + pivot.varianceCycles2,
                            )
                    }
                }
            }
        }
        return result
    }

    fun remember(
        floatStates: List<RtkDoubleDifferenceSolver.AmbiguityState>,
        samples: List<Sample>,
        timeNanos: Long,
    ) {
        val trusted = HashSet<SignalKey>()
        val nextAdr = HashMap<SignalKey, Double>()
        val nextBaseLock = HashMap<SignalKey, Long>()
        for (sample in samples) {
            val adr = sample.adrMeters
            if (adr == null || !adr.isFinite() || !carrierUsable(sample)) continue
            val key = SignalKey(sample.constellationType, sample.svid, sample.bandMhz)
            nextAdr[key] = adr
            if (sample.baseLockTimeMillis >= 0L) {
                nextBaseLock[key] = sample.baseLockTimeMillis
            }
            if (isPseudorangeRateReliable(sample.rateMetersPerSecond, sample.rateUncertaintyMetersPerSecond)) {
                trusted += key
            }
        }
        states =
            floatStates
                .filter { state ->
                    val bandMhz = bandMhzForWavelength(state.wavelengthMeters)
                    val satTrusted = resolveKey(trusted, state.constellationType, state.satSvid, bandMhz) != null
                    val refTrusted = resolveKey(trusted, state.constellationType, state.refSvid, bandMhz) != null
                    satTrusted && refTrusted
                }.map { state ->
                    state.copy(varianceCycles2 = state.varianceCycles2 + PROCESS_NOISE_CYCLES2)
                }
        previousAdr.clear()
        previousAdr.putAll(nextAdr)
        previousBaseLockTimeMillis.clear()
        previousBaseLockTimeMillis.putAll(nextBaseLock)
        epochNanos = timeNanos
    }

    private fun bandMhzForWavelength(wavelengthMeters: Double): Int =
        if (wavelengthMeters > 0.0) {
            (GnssMathUtils.SPEED_OF_LIGHT / wavelengthMeters / 1e6).roundToInt()
        } else {
            0
        }

    private fun resolveKey(
        keys: Set<SignalKey>,
        constellationType: Int,
        svid: Int,
        bandMhz: Int,
    ): SignalKey? {
        val exact = SignalKey(constellationType, svid, bandMhz)
        if (exact in keys) return exact
        val unkeyed = SignalKey(constellationType, svid, 0)
        return if (unkeyed in keys) unkeyed else null
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
