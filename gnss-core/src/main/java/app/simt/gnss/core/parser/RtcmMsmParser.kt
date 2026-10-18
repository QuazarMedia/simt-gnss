package app.simt.gnss.core.parser
import app.simt.gnss.core.model.Constellation
import app.simt.gnss.core.utils.GnssMathUtils
import app.simt.gnss.core.utils.GnssMathUtils.SPEED_OF_LIGHT

/**
 * Decoder for RTCM3 **MSM** (Multiple Signal Message) types 4–7, which carry a reference
 * (base) station's raw code and carrier-phase observations — the Observation Space
 * Representation (OSR) data used for RTK double-differencing.
 *
 * Supported message numbers (constellation × MSM type):
 * GPS 1074/1077, GLONASS 1084/1087, Galileo 1094/1097, SBAS 1104/1107, QZSS 1114/1117,
 * BeiDou 1124/1127, NavIC 1134/1137. MSM5/6 share the same layout and are decoded too.
 *
 * Layout follows RTCM 10403.3 §3.5 (and the RTKLIB reference decoder): a common MSM header
 * (station id, epoch, satellite mask, signal mask, cell mask) followed by per-satellite rough
 * ranges and per-cell fine pseudorange / phase / lock / half-cycle / CNR fields. MSM6/7 use the
 * higher-resolution field widths; MSM5/7 additionally carry phase-range-rate fields, which are
 * decoded past but not retained (not needed for code/carrier double-differencing).
 */
object RtcmMsmParser {
    private const val RANGE_MS = SPEED_OF_LIGHT * 0.001 // metres per millisecond of range
    private const val P2_10 = 1.0 / (1L shl 10)
    private const val P2_24 = 1.0 / (1L shl 24)
    private const val P2_29 = 1.0 / (1L shl 29)
    private const val P2_31 = 1.0 / (1L shl 31)
    private const val P2_4 = 1.0 / (1L shl 4)

    private const val ROUGH_RANGE_INVALID = 255
    private const val FINE_PR_INVALID_LOW = -(1 shl 14) // MSM4/5: -2^14
    private const val FINE_PR_INVALID_HIGH = -(1 shl 19) // MSM6/7: -2^19
    private const val FINE_PH_INVALID_LOW = -(1 shl 21) // MSM4/5: -2^21
    private const val FINE_PH_INVALID_HIGH = -(1 shl 23) // MSM6/7: -2^23

    /** One base-station observation for a single (satellite, signal) cell. */
    data class BaseObservation(
        val constellationType: Int,
        val svid: Int,
        /** Carrier frequency (Hz), or 0.0 if the signal could not be mapped to a frequency. */
        val carrierFrequencyHz: Double,
        val pseudorangeMeters: Double,
        /** Carrier-phase range (metres), or null when invalid or carrying a half-cycle ambiguity. */
        val carrierPhaseMeters: Double?,
        val cnrDbHz: Double,
        /** GNSS epoch of this observation (MSM header time, ms). Used to time-match against the rover. */
        val epochTimeMs: Long,
        /** RTCM MSM signal id — identifies the tracked signal within the band (e.g. GPS 2 = 1C, 4 = 1W). */
        val signalId: Int = 0,
    )

    data class MsmObservations(
        val referenceStationId: Int,
        val epochTimeMs: Long,
        val observations: List<BaseObservation>,
    )

    fun isSupported(messageType: Int): Boolean {
        val msmType = messageType % 10
        if (msmType !in 4..7) return false
        return constellationForGroup(messageType - msmType) != null
    }

    /**
     * Decode an MSM payload (the RTCM3 message body, starting at the 12-bit message number).
     * Returns null if the message is not a supported MSM type or is malformed.
     */
    fun parse(payload: ByteArray): MsmObservations? {
        val reader = BitReader(payload)
        if (payload.size < 10) return null

        val messageType = reader.u(12).toInt()
        val msmType = messageType % 10
        if (msmType !in 4..7) return null
        val constellation = constellationForGroup(messageType - msmType) ?: return null

        val referenceStationId = reader.u(12).toInt()
        val epochTimeMs = reader.u(30)
        reader.skip(1 + 3 + 7 + 2 + 2 + 1 + 3) // multi-message, IODS, reserved, clock/ext-clock, smoothing

        val satelliteNumbers = (0 until 64).mapNotNull { if (reader.u(1) == 1L) it + 1 else null }
        val signalIndices = (0 until 32).mapNotNull { if (reader.u(1) == 1L) it + 1 else null }
        val nSat = satelliteNumbers.size
        val nSig = signalIndices.size
        if (nSat == 0 || nSig == 0 || nSat * nSig > 64) return null // RTCM caps cells at 64

        val cellPresent = Array(nSat) { BooleanArray(nSig) }
        for (i in 0 until nSat) {
            for (j in 0 until nSig) {
                cellPresent[i][j] = reader.u(1) == 1L
            }
        }

        val highRes = msmType == 6 || msmType == 7
        val withRates = msmType == 5 || msmType == 7

        // --- Satellite data ---
        val roughRangeInt = IntArray(nSat) { reader.u(8).toInt() }
        val extendedInfo = if (withRates) IntArray(nSat) { reader.u(4).toInt() } else IntArray(nSat)
        val roughRangeMod = IntArray(nSat) { reader.u(10).toInt() }
        if (withRates) repeat(nSat) { reader.skip(14) } // rough phase-range-rate (unused)

        // --- Signal data (per cell, satellite-major order) ---
        val cellOrder =
            buildList {
                for (i in 0 until nSat) for (j in 0 until nSig) if (cellPresent[i][j]) add(i to j)
            }
        val nCell = cellOrder.size

        val finePr = IntArray(nCell) { reader.s(if (highRes) 20 else 15).toInt() }
        val finePh = IntArray(nCell) { reader.s(if (highRes) 24 else 22).toInt() }
        repeat(nCell) { reader.skip(if (highRes) 10 else 4) } // lock-time indicator (unused for now)
        val halfCycle = IntArray(nCell) { reader.u(1).toInt() }
        val cnr = IntArray(nCell) { reader.u(if (highRes) 10 else 6).toInt() }
        if (withRates) repeat(nCell) { reader.skip(15) } // fine phase-range-rate (unused)

        if (!reader.withinBounds()) return null

        val finePrScale = if (highRes) P2_29 else P2_24
        val finePhScale = if (highRes) P2_31 else P2_29
        val cnrScale = if (highRes) P2_4 else 1.0
        val finePrInvalid = if (highRes) FINE_PR_INVALID_HIGH else FINE_PR_INVALID_LOW
        val finePhInvalid = if (highRes) FINE_PH_INVALID_HIGH else FINE_PH_INVALID_LOW

        val observations = ArrayList<BaseObservation>(nCell)
        for (cell in 0 until nCell) {
            val (satIdx, sigIdx) = cellOrder[cell]
            if (roughRangeInt[satIdx] == ROUGH_RANGE_INVALID) continue
            if (finePr[cell] == finePrInvalid) continue

            val roughMs = roughRangeInt[satIdx] + roughRangeMod[satIdx] * P2_10
            val pseudorangeMeters = (roughMs + finePr[cell] * finePrScale) * RANGE_MS

            val carrierPhaseMeters =
                if (finePh[cell] == finePhInvalid || halfCycle[cell] == 1) {
                    null
                } else {
                    (roughMs + finePh[cell] * finePhScale) * RANGE_MS
                }

            val satelliteNumber = satelliteNumbers[satIdx]
            val signalId = signalIndices[sigIdx]
            observations.add(
                BaseObservation(
                    constellationType = constellation,
                    svid = svidFor(constellation, satelliteNumber),
                    carrierFrequencyHz = frequencyHz(constellation, signalId, extendedInfo[satIdx], withRates),
                    pseudorangeMeters = pseudorangeMeters,
                    carrierPhaseMeters = carrierPhaseMeters,
                    cnrDbHz = cnr[cell] * cnrScale,
                    epochTimeMs = epochTimeMs,
                    signalId = signalId,
                ),
            )
        }

        return MsmObservations(referenceStationId, epochTimeMs, observations)
    }

    private fun constellationForGroup(group: Int): Int? =
        when (group) {
            1070 -> Constellation.GPS
            1080 -> Constellation.GLONASS
            1090 -> Constellation.GALILEO
            1100 -> Constellation.SBAS
            1110 -> Constellation.QZSS
            1120 -> Constellation.BEIDOU
            1130 -> Constellation.IRNSS
            else -> null
        }

    /** Maps an RTCM satellite number to the platform SVID convention. */
    private fun svidFor(
        constellation: Int,
        satelliteNumber: Int,
    ): Int =
        when (constellation) {
            Constellation.QZSS -> 192 + satelliteNumber // RTCM 1..10 -> PRN 193..202
            Constellation.SBAS -> 119 + satelliteNumber // RTCM 1..39 -> PRN 120..158
            else -> satelliteNumber
        }

    /**
     * Maps an RTCM signal id (1-based index into the 32-bit signal mask) to a carrier frequency.
     * Covers the L1/E1/B1, L2/E5b/B2b/B3 and L5/E5a/B2a bands across GPS, Galileo and BeiDou; GLONASS
     * is FDMA so its L1/L2 frequency is derived from the per-satellite channel (available in MSM5/7
     * extended info). Returns 0.0 for unmapped signals, which the RTK matcher then ignores.
     */
    private fun frequencyHz(
        constellation: Int,
        signalId: Int,
        extendedInfo: Int,
        hasExtendedInfo: Boolean,
    ): Double {
        val l1 = 1_575_420_000.0
        val l2 = 1_227_600_000.0
        val l5 = 1_176_450_000.0
        val e5b = 1_207_140_000.0
        val e6 = 1_278_750_000.0
        val b1i = 1_561_098_000.0
        val b3 = 1_268_520_000.0
        return when (constellation) {
            Constellation.GPS, Constellation.QZSS ->
                when (signalId) {
                    2, 3, 4, 30, 31, 32 -> l1
                    8, 9, 10, 15, 16, 17 -> l2
                    22, 23, 24 -> l5
                    else -> 0.0
                }
            Constellation.GALILEO ->
                when (signalId) {
                    2, 3, 4, 5, 6 -> l1 // E1
                    8, 9, 10 -> e6
                    14, 15, 16 -> e5b
                    22, 23, 24 -> l5 // E5a
                    else -> 0.0
                }
            Constellation.BEIDOU ->
                when (signalId) {
                    2, 3, 4 -> b1i
                    8, 9, 10 -> b3
                    14, 15, 16 -> e5b // B2b shares the E5b frequency
                    22, 23, 24 -> l5 // B2a shares the E5a frequency
                    30, 31, 32 -> l1 // B1C
                    else -> 0.0
                }
            Constellation.GLONASS -> {
                if (!hasExtendedInfo) return 0.0
                val channel = extendedInfo - 7 // DF419: channel + 7, range -7..+6
                when (signalId) {
                    2, 3 -> 1_602_000_000.0 + channel * 562_500.0 // G1
                    8, 9 -> 1_246_000_000.0 + channel * 437_500.0 // G2
                    else -> 0.0
                }
            }
            else -> 0.0
        }
    }

    /** Sequential MSB-first bit reader over an RTCM payload. */
    private class BitReader(private val data: ByteArray) {
        private var bitPos = 0

        fun u(bits: Int): Long {
            val value = GnssMathUtils.getBitsLong(data, bitPos, bits)
            bitPos += bits
            return value
        }

        fun s(bits: Int): Long {
            val value = GnssMathUtils.getSignedBitsLong(data, bitPos, bits)
            bitPos += bits
            return value
        }

        fun skip(bits: Int) {
            bitPos += bits
        }

        /** True while the read cursor is still inside the payload (no overrun). */
        fun withinBounds(): Boolean = bitPos <= data.size * 8
    }
}
