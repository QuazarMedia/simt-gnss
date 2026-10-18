package app.simt.gnss.core.correction
import app.simt.gnss.core.model.Constellation
import app.simt.gnss.core.parser.RtcmMsmParser
import app.simt.gnss.core.platform.GnssPlatform
import kotlin.math.roundToInt

/**
 * Thread-safe store of the latest RTK reference (base) station data: its surveyed ECEF position
 * (from RTCM 1005/1006) and its raw code/carrier observations (from MSM4–7), keyed by
 * (constellation, svid, frequency band). RTK double-differencing reads from this store each epoch
 * to difference the rover's measurements against the base's.
 *
 * Observations must be very fresh (base streams ≈1 Hz); the surveyed position changes only when the
 * base is reconfigured, so it is allowed a much longer lifetime. Stale entries are ignored, and the
 * store reports [hasFreshData] so the solver can fall back to absolute positioning when the base
 * link is down.
 */
class RtkBaseStore(
    /** Monotonic clock source (ms); injectable for testing. */
    private val nowMs: () -> Long = { GnssPlatform.timeSource.elapsedRealtimeMs() },
) {
    /** Base observations older than this are ignored (≈1 Hz stream + latency margin). */
    val maxObservationAgeMs: Long = 10_000L

    /** A surveyed base position older than this is ignored. */
    private val maxPositionAgeMs: Long = 120_000L

    private val lock = Any()

    data class BasePosition(
        val xMeters: Double,
        val yMeters: Double,
        val zMeters: Double,
        val receivedAtMs: Long,
    )

    private data class StoredObservation(
        val observation: RtcmMsmParser.BaseObservation,
        val receivedAtMs: Long,
    )

    private var basePosition: BasePosition? = null
    private val observations = HashMap<Key, StoredObservation>()

    /**
     * Latest MSM epoch seen per constellation. The base receiver clock cancels in a double
     * difference only when both base observations share one epoch: mixing epochs leaves
     * c·(dt_b(t1) − dt_b(t2)) — 0.3–3 m at ordinary oscillator drift, ~300 km if the base
     * applies a millisecond clock jump. Only current-epoch observations are served.
     */
    private val latestEpochMsByConstellation = HashMap<Int, Long>()

    /** Key = constellation, svid, frequency rounded to the nearest MHz (the carrier band). */
    private data class Key(val constellationType: Int, val svid: Int, val frequencyMHz: Int)

    /**
     * Preference rank (lower = better) for matching the phone's tracked signal within a band.
     * Geodetic bases broadcast several signals per band (e.g. GPS 1C and 1W); differencing
     * the rover's C/A observation against a base P(Y)/Z-tracking observation leaves the
     * satellite-specific P1−C1 bias (±0.3–0.6 m) in every double difference, so the
     * C/A-class signal must win regardless of MSM mask order.
     */
    private fun signalPreferenceRank(
        constellationType: Int,
        signalId: Int,
    ): Int =
        when (constellationType) {
            Constellation.GPS,
            Constellation.QZSS,
            Constellation.GLONASS,
            ->
                when (signalId) {
                    2 -> 0 // 1C — what phones track
                    30, 31, 32 -> 1 // L1C family
                    else -> 5 // 1P/1W and others
                }
            Constellation.GALILEO ->
                when (signalId) {
                    2 -> 0 // E1 C
                    5 -> 1 // E1 B+C
                    4 -> 2 // E1 B
                    6 -> 3 // E1 A+B+C
                    else -> 5
                }
            Constellation.BEIDOU ->
                when (signalId) {
                    2 -> 0 // B1I (2I)
                    3 -> 1
                    4 -> 2
                    else -> 5
                }
            else -> 5
        }

    private fun keyFor(
        constellationType: Int,
        svid: Int,
        frequencyHz: Double,
    ): Key? {
        if (frequencyHz <= 0.0) return null
        return Key(constellationType, svid, (frequencyHz / 1e6).roundToInt())
    }

    fun setBasePosition(
        xMeters: Double,
        yMeters: Double,
        zMeters: Double,
    ) = synchronized(lock) {
        basePosition = BasePosition(xMeters, yMeters, zMeters, nowMs())
    }

    /** Returns the surveyed base position, or null if none/stale. */
    fun getBasePosition(): BasePosition? =
        synchronized(lock) {
            basePosition?.takeIf { nowMs() - it.receivedAtMs < maxPositionAgeMs }
        }

    /** Stores a batch of base observations from one decoded MSM epoch. Unmapped-frequency obs are skipped. */
    fun putObservations(batch: List<RtcmMsmParser.BaseObservation>) =
        synchronized(lock) {
            val now = nowMs()
            for (obs in batch) {
                val key = keyFor(obs.constellationType, obs.svid, obs.carrierFrequencyHz) ?: continue
                val currentEpoch = latestEpochMsByConstellation[obs.constellationType]
                if (currentEpoch == null || currentEpoch != obs.epochTimeMs) {
                    // A new epoch supersedes every earlier observation of this constellation.
                    latestEpochMsByConstellation[obs.constellationType] = obs.epochTimeMs
                    observations.entries.removeAll {
                        it.key.constellationType == obs.constellationType &&
                            it.value.observation.epochTimeMs != obs.epochTimeMs
                    }
                }
                val existing = observations[key]
                if (existing != null && existing.observation.epochTimeMs == obs.epochTimeMs &&
                    signalPreferenceRank(obs.constellationType, existing.observation.signalId) <=
                    signalPreferenceRank(obs.constellationType, obs.signalId)
                ) {
                    continue // keep the better (phone-trackable) signal already stored
                }
                observations[key] = StoredObservation(obs, now)
            }
        }

    /** Returns the fresh, current-epoch base observation matching a rover measurement's band, or null. */
    fun getObservation(
        constellationType: Int,
        svid: Int,
        frequencyHz: Double,
    ): RtcmMsmParser.BaseObservation? =
        synchronized(lock) {
            val key = keyFor(constellationType, svid, frequencyHz) ?: return null
            val stored = observations[key] ?: return null
            if (nowMs() - stored.receivedAtMs >= maxObservationAgeMs) return null
            // Serve only observations from the constellation's latest epoch: a satellite that
            // dropped out of the newest base epoch must not be served from an older one next
            // to fresh ones (the base clock difference between epochs corrupts the DD).
            if (stored.observation.epochTimeMs != latestEpochMsByConstellation[constellationType]) return null
            stored.observation
        }

    /** True when a fresh base position and at least [minObservations] fresh, current-epoch observations exist. */
    fun hasFreshData(minObservations: Int = 4): Boolean =
        synchronized(lock) {
            if (basePosition?.let { nowMs() - it.receivedAtMs < maxPositionAgeMs } != true) {
                return false
            }
            val now = nowMs()
            observations.values.count {
                now - it.receivedAtMs < maxObservationAgeMs &&
                    it.observation.epochTimeMs == latestEpochMsByConstellation[it.observation.constellationType]
            } >= minObservations
        }

    fun evictStale() =
        synchronized(lock) {
            val now = nowMs()
            observations.entries.removeAll { now - it.value.receivedAtMs >= maxObservationAgeMs }
            basePosition?.let { if (now - it.receivedAtMs >= maxPositionAgeMs) basePosition = null }
        }

    fun clear() =
        synchronized(lock) {
            basePosition = null
            observations.clear()
            latestEpochMsByConstellation.clear()
        }

    fun describe(): String =
        synchronized(lock) {
            val now = nowMs()
            val freshObs = observations.values.count { now - it.receivedAtMs < maxObservationAgeMs }
            val hasPos = basePosition?.let { now - it.receivedAtMs < maxPositionAgeMs } == true
            "RTK base: position=${if (hasPos) "fresh" else "none"} observations=$freshObs"
        }
}
