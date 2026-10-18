package app.simt.gnss.core.correction
import app.simt.gnss.core.platform.GnssPlatform

/**
 * Data model for IGS Real-Time Service (RTS) State Space Representation (SSR) corrections.
 *
 * SSR corrections are delta values applied to broadcast ephemeris orbit and clock parameters
 * to improve them to precise-product quality. They also include code and phase biases.
 *
 * Unlike OSR (Observation Space Representation, e.g. RTK), SSR corrections are valid globally
 * and do not require a nearby reference station.
 */
object SsrCorrections {
    /** Maximum age of SSR corrections before they are considered stale. */
    const val MAX_AGE_MS = 120_000L // 2 minutes

    /** Maximum age for bias corrections (less time-critical than orbit/clock). */
    const val MAX_BIAS_AGE_MS = 300_000L // 5 minutes

    /**
     * Per-satellite orbit correction in radial/along-track/cross-track frame.
     * Applied as delta to broadcast ephemeris satellite position.
     *
     * RTCM3 messages: 1060 (GPS), 1066 (GLONASS), 1243 (Galileo), 1261 (BeiDou)
     */
    data class OrbitCorrection(
        val constellationType: Int,
        val svid: Int,
        /** Radial correction (meters) — positive = away from earth center. */
        val deltaRadial: Double,
        /** Along-track correction (meters) — positive = in velocity direction. */
        val deltaAlongTrack: Double,
        /** Cross-track correction (meters) — positive = right of velocity. */
        val deltaCrossTrack: Double,
        /** Radial rate correction (m/s). */
        val dotRadial: Double = 0.0,
        /** Along-track rate correction (m/s). */
        val dotAlongTrack: Double = 0.0,
        /** Cross-track rate correction (m/s). */
        val dotCrossTrack: Double = 0.0,
        /** IOD (Issue of Data) linking to broadcast ephemeris set. */
        val iod: Int,
        /** SSR epoch time (GPS seconds of week). */
        val epochTime: Double,
        /** SSR update interval (seconds). 0 means the rate reference is the epoch itself. */
        val updateIntervalSeconds: Double = 0.0,
        /** GnssPlatform.timeSource.elapsedRealtimeMs() at reception. */
        val receivedAtMs: Long,
    ) {
        fun ageMs(): Long = GnssPlatform.timeSource.elapsedRealtimeMs() - receivedAtMs

        fun isValid(): Boolean = ageMs() < MAX_AGE_MS
    }

    /**
     * Per-satellite clock correction (high-rate).
     * Applied as delta to broadcast clock polynomial.
     *
     * RTCM3 messages: combined in 1060/1066/1243/1261 (orbit+clock)
     */
    data class ClockCorrection(
        val constellationType: Int,
        val svid: Int,
        /** Clock correction C0 (meters). */
        val c0: Double,
        /** Clock correction C1 (m/s). */
        val c1: Double = 0.0,
        /** Clock correction C2 (m/s²). */
        val c2: Double = 0.0,
        /** IOD linking to broadcast ephemeris. */
        val iod: Int,
        /** SSR epoch time (GPS seconds of week). */
        val epochTime: Double,
        /** SSR update interval (seconds). 0 means the rate reference is the epoch itself. */
        val updateIntervalSeconds: Double = 0.0,
        /** GnssPlatform.timeSource.elapsedRealtimeMs() at reception. */
        val receivedAtMs: Long,
    ) {
        fun ageMs(): Long = GnssPlatform.timeSource.elapsedRealtimeMs() - receivedAtMs

        fun isValid(): Boolean = ageMs() < MAX_AGE_MS
    }

    /**
     * Per-satellite code bias correction for a specific signal.
     * RTCM3 messages: 1059 (GPS), 1065 (GLONASS), 1242 (Galileo), 1260 (BeiDou)
     */
    data class CodeBias(
        val constellationType: Int,
        val svid: Int,
        /** Signal identifier (RTCM signal index). */
        val signalId: Int,
        /** Code bias correction (meters). */
        val bias: Double,
        val receivedAtMs: Long,
    ) {
        fun isValid(): Boolean = (GnssPlatform.timeSource.elapsedRealtimeMs() - receivedAtMs) < MAX_BIAS_AGE_MS
    }

    /**
     * Per-satellite phase bias correction for a specific signal.
     * RTCM3 messages: 1265 (GPS), 1266 (GLONASS), 1267 (Galileo), 1270 (BeiDou)
     */
    data class PhaseBias(
        val constellationType: Int,
        val svid: Int,
        /** Signal identifier. */
        val signalId: Int,
        /** Phase bias correction (meters). */
        val bias: Double,
        /** Integer indicator: 0 = float, 1 = fixed. */
        val integerIndicator: Int = 0,
        val receivedAtMs: Long,
    ) {
        fun isValid(): Boolean = (GnssPlatform.timeSource.elapsedRealtimeMs() - receivedAtMs) < MAX_BIAS_AGE_MS
    }

    /**
     * Thread-safe store of the latest SSR corrections keyed by (constellationType, svid).
     *
     * Fallback strategy:
     * - If a correction is stale (older than MAX_AGE_MS), it is ignored
     * - If no corrections are available at all, the solver runs with broadcast-only (existing behavior)
     * - If IOD mismatch between SSR and broadcast ephemeris, SSR is not applied for that SV
     */
    class Store {
        private val lock = Any()
        private val orbits = mutableMapOf<Pair<Int, Int>, OrbitCorrection>()
        private val clocks = mutableMapOf<Pair<Int, Int>, ClockCorrection>()
        private val codeBiases = mutableMapOf<Triple<Int, Int, Int>, CodeBias>() // (const, svid, signalId)
        private val phaseBiases = mutableMapOf<Triple<Int, Int, Int>, PhaseBias>()

        @Volatile
        var lastUpdateMs: Long = 0L
            private set

        /** Whether any valid SSR corrections are available. */
        val hasValidCorrections: Boolean
            get() =
                synchronized(lock) {
                    orbits.values.any { it.isValid() } || clocks.values.any { it.isValid() }
                }

        fun putOrbit(correction: OrbitCorrection) =
            synchronized(lock) {
                orbits[correction.constellationType to correction.svid] = correction
                lastUpdateMs = GnssPlatform.timeSource.elapsedRealtimeMs()
            }

        fun putClock(correction: ClockCorrection) =
            synchronized(lock) {
                clocks[correction.constellationType to correction.svid] = correction
                lastUpdateMs = GnssPlatform.timeSource.elapsedRealtimeMs()
            }

        fun putCodeBias(bias: CodeBias) =
            synchronized(lock) {
                codeBiases[Triple(bias.constellationType, bias.svid, bias.signalId)] = bias
                lastUpdateMs = GnssPlatform.timeSource.elapsedRealtimeMs()
            }

        fun putPhaseBias(bias: PhaseBias) =
            synchronized(lock) {
                phaseBiases[Triple(bias.constellationType, bias.svid, bias.signalId)] = bias
                lastUpdateMs = GnssPlatform.timeSource.elapsedRealtimeMs()
            }

        /**
         * Get valid orbit correction for a satellite, or null if stale/unavailable.
         * @param iod Broadcast ephemeris IOD to check consistency; pass -1 to skip check.
         */
        fun getOrbit(
            constellationType: Int,
            svid: Int,
            iod: Int = -1,
        ): OrbitCorrection? =
            synchronized(lock) {
                val c = orbits[constellationType to svid] ?: return null
                if (!c.isValid()) return null
                if (iod >= 0 && c.iod != iod) return null
                c
            }

        fun getClock(
            constellationType: Int,
            svid: Int,
            iod: Int = -1,
        ): ClockCorrection? =
            synchronized(lock) {
                val c = clocks[constellationType to svid] ?: return null
                if (!c.isValid()) return null
                if (iod >= 0 && c.iod != iod) return null
                c
            }

        fun getCodeBias(
            constellationType: Int,
            svid: Int,
            signalId: Int,
        ): CodeBias? =
            synchronized(lock) {
                codeBiases[Triple(constellationType, svid, signalId)]?.takeIf { it.isValid() }
            }

        fun getPhaseBias(
            constellationType: Int,
            svid: Int,
            signalId: Int,
        ): PhaseBias? =
            synchronized(lock) {
                phaseBiases[Triple(constellationType, svid, signalId)]?.takeIf { it.isValid() }
            }

        /** Remove all stale corrections. */
        fun evictStale() =
            synchronized(lock) {
                orbits.entries.removeAll { !it.value.isValid() }
                clocks.entries.removeAll { !it.value.isValid() }
                codeBiases.entries.removeAll { !it.value.isValid() }
                phaseBiases.entries.removeAll { !it.value.isValid() }
            }

        /** Clear all corrections (e.g., on connection loss). */
        fun clear() =
            synchronized(lock) {
                orbits.clear()
                clocks.clear()
                codeBiases.clear()
                phaseBiases.clear()
                lastUpdateMs = 0L
            }

        /** Summary for logging. */
        fun describe(): String =
            synchronized(lock) {
                val validOrbits = orbits.values.count { it.isValid() }
                val validClocks = clocks.values.count { it.isValid() }
                val validCodeBias = codeBiases.values.count { it.isValid() }
                val validPhaseBias = phaseBiases.values.count { it.isValid() }
                "SSR: orbits=$validOrbits clocks=$validClocks codeBias=$validCodeBias phaseBias=$validPhaseBias"
            }
    }
}
