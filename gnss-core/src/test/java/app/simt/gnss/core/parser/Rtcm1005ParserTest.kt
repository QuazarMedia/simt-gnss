package app.simt.gnss.core.parser

import app.simt.gnss.core.utils.GnssMathUtils
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.roundToLong

class Rtcm1005ParserTest {
    @Test
    fun isSupported_recognizes1005And1006Only() {
        assertTrue(Rtcm1005Parser.isSupported(1005))
        assertTrue(Rtcm1005Parser.isSupported(1006))
        org.junit.Assert.assertFalse(Rtcm1005Parser.isSupported(1077))
        org.junit.Assert.assertFalse(Rtcm1005Parser.isSupported(1004))
    }

    @Test
    fun parse_decodes1005StationCoordinatesRoundTrip() {
        val xMeters = 4_278_387.1234
        val yMeters = 601_233.5678
        val zMeters = 4_638_360.9012

        val payload = ByteArray(19) // 152 bits
        GnssMathUtils.packBits(payload, 0, 12, 1005) // message type
        GnssMathUtils.packBits(payload, 12, 12, 512) // reference station id
        writeSigned(payload, 34, 38, (xMeters / 1e-4).roundToLong())
        writeSigned(payload, 74, 38, (yMeters / 1e-4).roundToLong())
        writeSigned(payload, 114, 38, (zMeters / 1e-4).roundToLong())

        val result = Rtcm1005Parser.parse(payload)
        requireNotNull(result)
        assertEquals(512, result.referenceStationId)
        assertEquals(xMeters, result.xMeters, 1e-3)
        assertEquals(yMeters, result.yMeters, 1e-3)
        assertEquals(zMeters, result.zMeters, 1e-3)
    }

    @Test
    fun parse_handlesNegativeCoordinate() {
        val zMeters = -4_638_360.9012
        val payload = ByteArray(19)
        GnssMathUtils.packBits(payload, 0, 12, 1006)
        writeSigned(payload, 114, 38, (zMeters / 1e-4).roundToLong())

        val result = Rtcm1005Parser.parse(payload)
        requireNotNull(result)
        assertEquals(zMeters, result.zMeters, 1e-3)
    }

    @Test
    fun parse_returnsNullForShortPayload() {
        assertNull(Rtcm1005Parser.parse(ByteArray(10)))
    }

    /** Writes a two's-complement signed value of [bits] width at [startBit], MSB-first. */
    private fun writeSigned(
        data: ByteArray,
        startBit: Int,
        bits: Int,
        value: Long,
    ) {
        val mask = (1L shl bits) - 1
        val encoded = value and mask
        for (i in 0 until bits) {
            val bit = ((encoded shr (bits - 1 - i)) and 1L).toInt()
            if (bit == 1) {
                val idx = startBit + i
                data[idx / 8] = (data[idx / 8].toInt() or (1 shl (7 - idx % 8))).toByte()
            }
        }
    }
}
