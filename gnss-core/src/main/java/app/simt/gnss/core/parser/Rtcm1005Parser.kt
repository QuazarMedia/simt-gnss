package app.simt.gnss.core.parser

import app.simt.gnss.core.utils.GnssMathUtils

/**
 * Decoder for RTCM3 message types 1005 and 1006 — the Stationary Antenna Reference Point, i.e. the
 * base station's surveyed ECEF coordinates. RTK double-differencing solves for the rover relative to
 * this known point, so a fresh base position is required before any RTK fix can be formed.
 *
 * Layout (RTCM 10403.3 §3.5): 1005 is a fixed 152-bit message; 1006 appends a 16-bit antenna height
 * but carries the same X/Y/Z fields, which is all this decoder needs. Coordinates are 38-bit signed
 * integers with a resolution of 0.0001 m.
 */
object Rtcm1005Parser {
    private const val MESSAGE_BITS = 152
    private const val COORDINATE_RESOLUTION_METERS = 1e-4

    // Absolute bit offsets of the X/Y/Z fields within the message payload.
    private const val ECEF_X_BIT = 34
    private const val ECEF_Y_BIT = 74
    private const val ECEF_Z_BIT = 114
    private const val COORDINATE_BITS = 38

    data class StationPosition(
        val referenceStationId: Int,
        val xMeters: Double,
        val yMeters: Double,
        val zMeters: Double,
    )

    fun isSupported(messageType: Int): Boolean = messageType == 1005 || messageType == 1006

    /** Decode a 1005/1006 payload (starting at the 12-bit message number), or null if malformed. */
    fun parse(payload: ByteArray): StationPosition? {
        if (payload.size * 8 < MESSAGE_BITS) return null
        val messageType = GnssMathUtils.getBits(payload, 0, 12)
        if (!isSupported(messageType)) return null

        val referenceStationId = GnssMathUtils.getBits(payload, 12, 12)
        val x = GnssMathUtils.getSignedBitsLong(payload, ECEF_X_BIT, COORDINATE_BITS) * COORDINATE_RESOLUTION_METERS
        val y = GnssMathUtils.getSignedBitsLong(payload, ECEF_Y_BIT, COORDINATE_BITS) * COORDINATE_RESOLUTION_METERS
        val z = GnssMathUtils.getSignedBitsLong(payload, ECEF_Z_BIT, COORDINATE_BITS) * COORDINATE_RESOLUTION_METERS
        return StationPosition(referenceStationId, x, y, z)
    }
}
