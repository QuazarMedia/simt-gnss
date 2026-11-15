package app.simt.gnss.core.correction

import app.simt.gnss.core.model.Constellation
import app.simt.gnss.core.model.GnssSignalBand
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

    /** Timescale of [OrbitCorrection.epochTime] and [ClockCorrection.epochTime]. */
    enum class EpochTimescale {
        GPST,
        BDT,
        GLONASS_TOD,
    }

    fun defaultEpochTimescale(constellationType: Int): EpochTimescale =
        when (constellationType) {
            Constellation.BEIDOU -> EpochTimescale.BDT
            Constellation.GLONASS -> EpochTimescale.GLONASS_TOD
            else -> EpochTimescale.GPST
        }

    /**
     * Ordered RTCM 10403.3 SSR 5-bit Tracking Mode / Signal Indicator IDs (0..31) matching each
     * tracked [GnssSignalBand] per constellation. Note: SSR 5-bit signal IDs differ from MSM
     * 1-indexed signal IDs.
     */
    fun ssrCodeBiasSignalIds(
        constellationType: Int,
        band: GnssSignalBand,
    ): IntArray =
        when (constellationType) {
            Constellation.GPS ->
                when (band) {
                    GnssSignalBand.L1_E1_B1C -> GPS_L1_SIGNAL_IDS
                    GnssSignalBand.L2 -> GPS_L2_SIGNAL_IDS
                    GnssSignalBand.L5_E5A_B2A -> GPS_L5_SIGNAL_IDS
                    else -> EMPTY_SIGNAL_IDS
                }
            Constellation.GLONASS ->
                when (band) {
                    GnssSignalBand.L1_E1_B1C -> GLO_G1_SIGNAL_IDS
                    GnssSignalBand.L2 -> GLO_G2_SIGNAL_IDS
                    else -> EMPTY_SIGNAL_IDS
                }
            Constellation.GALILEO ->
                when (band) {
                    GnssSignalBand.L1_E1_B1C -> GAL_E1_SIGNAL_IDS
                    GnssSignalBand.L5_E5A_B2A -> GAL_E5A_SIGNAL_IDS
                    GnssSignalBand.E5B_B2I -> GAL_E5B_SIGNAL_IDS
                    else -> EMPTY_SIGNAL_IDS
                }
            Constellation.QZSS ->
                when (band) {
                    GnssSignalBand.L1_E1_B1C -> QZS_L1_SIGNAL_IDS
                    GnssSignalBand.L2 -> QZS_L2_SIGNAL_IDS
                    GnssSignalBand.L5_E5A_B2A -> QZS_L5_SIGNAL_IDS
                    else -> EMPTY_SIGNAL_IDS
                }
            Constellation.BEIDOU ->
                when (band) {
                    GnssSignalBand.B1I -> BDS_B1I_SIGNAL_IDS
                    GnssSignalBand.B3I -> BDS_B3I_SIGNAL_IDS
                    GnssSignalBand.E5B_B2I -> BDS_B2I_SIGNAL_IDS
                    GnssSignalBand.L1_E1_B1C -> BDS_B1C_SIGNAL_IDS
                    GnssSignalBand.L5_E5A_B2A -> BDS_B2A_SIGNAL_IDS
                    else -> EMPTY_SIGNAL_IDS
                }
            else -> EMPTY_SIGNAL_IDS
        }

    private val EMPTY_SIGNAL_IDS = IntArray(0)
    private val GPS_L1_SIGNAL_IDS = intArrayOf(0, 1, 2, 17, 18, 19, 3, 4)
    private val GPS_L2_SIGNAL_IDS = intArrayOf(5, 8, 7, 6, 9, 10, 11, 12, 13)
    private val GPS_L5_SIGNAL_IDS = intArrayOf(15, 16, 14)
    private val GLO_G1_SIGNAL_IDS = intArrayOf(0, 1)
    private val GLO_G2_SIGNAL_IDS = intArrayOf(2, 3)
    private val GAL_E1_SIGNAL_IDS = intArrayOf(1, 2, 3, 4, 0)
    private val GAL_E5A_SIGNAL_IDS = intArrayOf(6, 7, 5)
    private val GAL_E5B_SIGNAL_IDS = intArrayOf(9, 10, 8)
    private val QZS_L1_SIGNAL_IDS = intArrayOf(0, 2, 3, 1)
    private val QZS_L2_SIGNAL_IDS = intArrayOf(4, 5, 6)
    private val QZS_L5_SIGNAL_IDS = intArrayOf(8, 9, 7)
    private val BDS_B1I_SIGNAL_IDS = intArrayOf(0, 1, 2)
    private val BDS_B3I_SIGNAL_IDS = intArrayOf(3, 4, 5)
    private val BDS_B2I_SIGNAL_IDS = intArrayOf(6, 7, 8)
    private val BDS_B1C_SIGNAL_IDS = intArrayOf(9, 10, 11)
    private val BDS_B2A_SIGNAL_IDS = intArrayOf(12, 13, 14)

    /**
     * Per-satellite orbit correction in radial/along-track/cross-track frame.
     * Applied as delta to broadcast ephemeris satellite position.
     *
     * RTCM3 messages: 1057/1060 (GPS), 1063/1066 (GLONASS), 1240/1243 (Galileo),
     * 1246/1249 (QZSS), 1258/1261 (BeiDou), and IGS 4076 orbit/combined subtypes.
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
        /**
         * SSR epoch time (seconds of week, or seconds of GLONASS day when
         * [epochTimescale] is [EpochTimescale.GLONASS_TOD]).
         */
        val epochTime: Double,
        /** SSR update interval (seconds). 0 means the rate reference is the epoch itself. */
        val updateIntervalSeconds: Double = 0.0,
        /** GnssPlatform.timeSource.elapsedRealtimeMs() at reception. */
        val receivedAtMs: Long,
        /** Header IOD SSR linking separate orbit, clock, and high-rate clock messages (-1 if unspecified). */
        val iodSsr: Int = -1,
        /** Timescale of [epochTime]. */
        val epochTimescale: EpochTimescale = defaultEpochTimescale(constellationType),
    ) {
        fun ageMs(): Long = GnssPlatform.timeSource.elapsedRealtimeMs() - receivedAtMs

        fun isValid(): Boolean = ageMs() < MAX_AGE_MS
    }

    /**
     * Per-satellite clock correction (high-rate).
     * Applied as delta to broadcast clock polynomial.
     *
     * RTCM3 messages: 1058/1060/1062 (GPS), 1064/1066/1068 (GLONASS), 1241/1243/1245 (Galileo),
     * 1247/1249/1251 (QZSS), 1259/1261/1263 (BeiDou), and IGS 4076 clock/combined subtypes.
     */
    data class ClockCorrection(
        val constellationType: Int,
        val svid: Int,
        /** Clock correction C0 (meters). */
        val c0: Double,
        /** Clock correction C1 (m/s). */
        val c1: Double = 0.0,
        /** Clock correction C2 (m/s^2). */
        val c2: Double = 0.0,
        /** IOD linking to broadcast ephemeris (-1 when decoded from a separate clock message before orbit arrival). */
        val iod: Int,
        /**
         * SSR epoch time (seconds of week, or seconds of GLONASS day when
         * [epochTimescale] is [EpochTimescale.GLONASS_TOD]).
         */
        val epochTime: Double,
        /** SSR update interval (seconds). 0 means the rate reference is the epoch itself. */
        val updateIntervalSeconds: Double = 0.0,
        /** GnssPlatform.timeSource.elapsedRealtimeMs() at reception. */
        val receivedAtMs: Long,
        /** Header IOD SSR linking separate orbit, clock, and high-rate clock messages (-1 if unspecified). */
        val iodSsr: Int = -1,
        /** Timescale of [epochTime]. */
        val epochTimescale: EpochTimescale = defaultEpochTimescale(constellationType),
    ) {
        fun ageMs(): Long = GnssPlatform.timeSource.elapsedRealtimeMs() - receivedAtMs

        fun isValid(): Boolean = ageMs() < MAX_AGE_MS
    }

    /**
     * Per-satellite high-rate clock delta (RTCM 1062/1068/1245/1251/1263 and IGS 4076 subtype *4).
     * Additive to the base [ClockCorrection] (`C0 + C1*dt + C2*dt^2 + deltaClockMeters`).
     */
    data class HighRateClockCorrection(
        val constellationType: Int,
        val svid: Int,
        /** High-rate clock delta (meters). */
        val deltaClockMeters: Double,
        /** Header IOD SSR linking to the base [ClockCorrection] (-1 if unspecified). */
        val iodSsr: Int = -1,
        /** SSR epoch time (seconds of week, or seconds of GLONASS day when [epochTimescale] is [EpochTimescale.GLONASS_TOD]). */
        val epochTime: Double,
        /** SSR update interval (seconds). 0 means the epoch itself. */
        val updateIntervalSeconds: Double = 0.0,
        /** Timescale of [epochTime]. */
        val epochTimescale: EpochTimescale = defaultEpochTimescale(constellationType),
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
        private val highRateClocks = mutableMapOf<Pair<Int, Int>, HighRateClockCorrection>()
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

        fun putHighRateClock(correction: HighRateClockCorrection) =
            synchronized(lock) {
                highRateClocks[correction.constellationType to correction.svid] = correction
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

        fun getHighRateClock(
            constellationType: Int,
            svid: Int,
            iodSsr: Int = -1,
        ): HighRateClockCorrection? =
            synchronized(lock) {
                val c = highRateClocks[constellationType to svid] ?: return null
                if (!c.isValid()) return null
                if (c.iodSsr < 0) return null
                if (iodSsr >= 0 && c.iodSsr != iodSsr) return null
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

        /**
         * Returns the first valid [CodeBias] matching [band] in preference order for
         * [constellationType], or null when no matching code bias is stored/fresh.
         */
        fun getCodeBiasForBand(
            constellationType: Int,
            svid: Int,
            band: GnssSignalBand,
        ): CodeBias? =
            synchronized(lock) {
                for (signalId in ssrCodeBiasSignalIds(constellationType, band)) {
                    val bias = codeBiases[Triple(constellationType, svid, signalId)]?.takeIf { it.isValid() }
                    if (bias != null) return@synchronized bias
                }
                return@synchronized null
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
                highRateClocks.entries.removeAll { !it.value.isValid() }
                codeBiases.entries.removeAll { !it.value.isValid() }
                phaseBiases.entries.removeAll { !it.value.isValid() }
            }

        /** Clear all corrections (e.g., on connection loss). */
        fun clear() =
            synchronized(lock) {
                orbits.clear()
                clocks.clear()
                highRateClocks.clear()
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
