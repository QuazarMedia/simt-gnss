package app.simt.gnss.core.parser

import app.simt.gnss.core.utils.GnssMathUtils
import org.junit.Assert.*
import org.junit.Test
import kotlin.math.pow

class GpsCnavMt30ParserTest {
    @Test
    fun parseL1L5IscIfComplete_wrongMessageType_returnsNull() {
        val buf = ByteArray(40)
        GnssMathUtils.packBits(buf, 14, 6, 10)
        assertNull(GpsCnavMt30Parser.parseL1L5IscIfComplete(buf))
    }

    @Test
    fun parseL1L5IscIfComplete_message30_zeroIsc_returnsZeros() {
        val buf = ByteArray(40)
        GnssMathUtils.packBits(buf, 14, 6, 30)
        // ISC fields default 0 — not equal to NOT_AVAILABLE pattern
        val isc = GpsCnavMt30Parser.parseL1L5IscIfComplete(buf)
        assertNotNull(isc)
        assertEquals(0.0, isc!!.iscL1CaSeconds, 1e-45)
        assertEquals(0.0, isc.iscL5I5Seconds, 1e-45)
    }

    @Test
    fun parseL1L5IscIfComplete_l1Unavailable_returnsNull() {
        val buf = ByteArray(40)
        GnssMathUtils.packBits(buf, 14, 6, 30)
        GnssMathUtils.packBits(buf, 140, 13, GpsCnavMt30Parser.ISC_13BIT_NOT_AVAILABLE)
        GnssMathUtils.packBits(buf, 166, 13, 0)
        assertNull(GpsCnavMt30Parser.parseL1L5IscIfComplete(buf))
    }

    @Test
    fun parseL1L5IscIfComplete_scaledPositiveL1() {
        val buf = ByteArray(40)
        GnssMathUtils.packBits(buf, 14, 6, 30)
        // +1 in 13-bit two's complement
        GnssMathUtils.packBits(buf, 140, 13, 1)
        GnssMathUtils.packBits(buf, 166, 13, 0)
        val isc = GpsCnavMt30Parser.parseL1L5IscIfComplete(buf)
        assertNotNull(isc)
        assertEquals(2.0.pow(-35), isc!!.iscL1CaSeconds, 1e-45)
        assertEquals(0.0, isc.iscL5I5Seconds, 1e-45)
    }

    @Test
    fun parseL1L5IscIfComplete_dataTooShort_returnsNull() {
        assertNull(GpsCnavMt30Parser.parseL1L5IscIfComplete(ByteArray(20)))
    }
}
