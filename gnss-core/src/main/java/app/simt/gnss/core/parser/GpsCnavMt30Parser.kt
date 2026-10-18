package app.simt.gnss.core.parser

import app.simt.gnss.core.utils.GnssMathUtils
import kotlin.math.pow

/**
 * Parses GPS **CNAV Message Type 30** (Clock, Iono & Group Delay) inter-signal corrections
 * from raw navigation-message payloads for GPS L5 CNAV and L2 CNAV
 * (`GnssNavigationMessage.TYPE_GPS_L5CNAV` / `TYPE_GPS_L2CNAV`).
 *
 * Bit layout matches **IS-GPS-705** MT30 and the reference implementation in GPSTk
 * `CNavISC::loadData` (message type at bit 14, ISC fields 13 bits each, scale 2⁻³⁵ s).
 *
 * The raw bit stream is packed **MSB-first** in big-endian bytes, consistent with
 * [GnssMathUtils.getBits] as used for GPS LNAV subframes.
 */
object GpsCnavMt30Parser {
    /** 13-bit pattern `0b1_0000_0000_0000` meaning the parameter is not available (IS-GPS-705 / GPSTk). */
    const val ISC_13BIT_NOT_AVAILABLE: Int = 0x1000

    private const val MESSAGE_TYPE_CLOCK_IONO_GROUP_DELAY: Int = 30

    /** Minimum bits through end of ISC(L5Q5) field: 179 + 13 - 1 = 191 → 24 bytes. */
    private const val MIN_DATA_BITS = 192

    private val TWO_POW_MINUS_35: Double = 2.0.pow(-35)

    /**
     * Inter-signal corrections for **L1 C/A** and **L5 I5** (seconds, broadcast sign).
     * Apply as **pseudorange meters += iscSeconds * c** before the ionosphere-free combination
     * when both values are present (IS-GPS-705 dual-frequency user algorithm).
     */
    data class L1L5IscSeconds(
        val iscL1CaSeconds: Double,
        val iscL5I5Seconds: Double,
    )

    /**
     * If [data] holds a decoded CNAV frame with message type 30 and both ISC(L1CA) and ISC(L5I5)
     * are marked available, returns parsed corrections; otherwise `null`.
     */
    fun parseL1L5IscIfComplete(data: ByteArray): L1L5IscSeconds? {
        if (data.isEmpty() || data.size * 8 < MIN_DATA_BITS) return null
        if (GnssMathUtils.getBits(data, 14, 6) != MESSAGE_TYPE_CLOCK_IONO_GROUP_DELAY) return null

        val iscL1 = readIscSecondsOrNull(data, 140) ?: return null
        val iscL5 = readIscSecondsOrNull(data, 166) ?: return null
        return L1L5IscSeconds(iscL1CaSeconds = iscL1, iscL5I5Seconds = iscL5)
    }

    private fun readIscSecondsOrNull(
        data: ByteArray,
        startBit: Int,
    ): Double? {
        if (GnssMathUtils.getBits(data, startBit, 13) == ISC_13BIT_NOT_AVAILABLE) return null
        val rawSigned = GnssMathUtils.getSignedBitsLong(data, startBit, 13)
        return rawSigned * TWO_POW_MINUS_35
    }
}
