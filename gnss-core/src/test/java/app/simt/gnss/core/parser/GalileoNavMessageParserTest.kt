package app.simt.gnss.core.parser
import app.simt.gnss.core.model.Constellation
import app.simt.gnss.core.utils.GnssMathUtils
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.PI
import kotlin.math.pow

/**
 * Unit tests for [GalileoNavMessageParser]. Pages are built in the documented Android
 * `GnssNavigationMessage` TYPE_GAL_I format — two 114-bit page parts (even then odd) in 29
 * bytes, with the 128-bit I/NAV word split as Data(1/2) = word bits 1–112 and Data(2/2) =
 * word bits 113–128 — and field placement follows Galileo OS SIS ICD Issue 2.1 §4.3.5.
 */
class GalileoNavMessageParserTest {
    /** Builds a contiguous 128-bit I/NAV word with the 6-bit word type at word bits 0-5. */
    private fun newWord(wordType: Int): ByteArray = ByteArray(16).also { GnssMathUtils.packBits(it, 0, 6, wordType) }

    /** Packs a value of up to 32 bits at a word-relative offset. */
    private fun packLong(
        data: ByteArray,
        startBit: Int,
        length: Int,
        value: Long,
    ) {
        for (i in 0 until length) {
            val bitIdx = startBit + i
            val byteIdx = bitIdx / 8
            val bitOffset = 7 - (bitIdx % 8)
            val bit = ((value shr (length - 1 - i)) and 1L).toInt()
            if (byteIdx < data.size) {
                val current = data[byteIdx].toInt()
                val cleared = current and (1 shl bitOffset).inv()
                data[byteIdx] = (cleared or (bit shl bitOffset)).toByte()
            }
        }
    }

    private fun copyBit(
        src: ByteArray,
        srcBit: Int,
        dst: ByteArray,
        dstBit: Int,
    ) {
        val bit = (src[srcBit / 8].toInt() shr (7 - srcBit % 8)) and 1
        GnssMathUtils.packBits(dst, dstBit, 1, bit)
    }

    /**
     * Wraps a 128-bit I/NAV word into the 29-byte Android page pair: even part = even/odd(0)
     * + pageType(0) + word bits 1-112 at flat 2-113; odd part = even/odd(1) + pageType(0) +
     * word bits 113-128 at flat 116-131.
     */
    private fun page(
        word: ByteArray,
        alertPage: Boolean = false,
    ): ByteArray {
        val out = ByteArray(29)
        for (i in 0 until 112) copyBit(word, i, out, 2 + i)
        GnssMathUtils.packBits(out, 114, 1, 1)
        if (alertPage) GnssMathUtils.packBits(out, 115, 1, 1)
        for (i in 0 until 16) copyBit(word, 112 + i, out, 116 + i)
        return out
    }

    private fun pages(vararg words: ByteArray): Map<Int, ByteArray> = words.withIndex().associate { (i, w) -> (i + 1) to page(w) }

    @Test
    fun parse_withAllWordsZero_returnsAllZeroEphemerisWithMatchingIodNav() {
        // All-zero payload → IODnav = 0 everywhere (consistent), healthy flags, all values 0.
        val eph =
            GalileoNavMessageParser.parseGalileoEphemeris(
                svid = 5,
                words = pages(newWord(1), newWord(2), newWord(3), newWord(4), newWord(5)),
            )
        assertNotNull("Parser must accept an all-zero consistent page set", eph)
        assertEquals(Constellation.GALILEO, eph!!.constellationType)
        assertEquals(5, eph.svid)
        assertEquals(0, eph.week)
        assertEquals(0.0, eph.toe, 0.0)
        assertEquals(0.0, eph.toc, 0.0)
        assertEquals(0.0, eph.af0, 0.0)
    }

    @Test
    fun parse_missingWord_returnsNull() {
        val words = pages(newWord(1), newWord(2), newWord(3), newWord(4), newWord(5)).minus(3)
        assertNull(GalileoNavMessageParser.parseGalileoEphemeris(svid = 1, words = words))
    }

    @Test
    fun parse_inconsistentIodNav_returnsNull() {
        // IODnav = 5 in W1 but IODnav = 10 in W2. Parser must reject the mixed set.
        val w1 = newWord(1).also { GnssMathUtils.packBits(it, 6, 10, 5) }
        val w2 = newWord(2).also { GnssMathUtils.packBits(it, 6, 10, 10) }
        val w3 = newWord(3).also { GnssMathUtils.packBits(it, 6, 10, 5) }
        val w4 = newWord(4).also { GnssMathUtils.packBits(it, 6, 10, 5) }
        assertNull(GalileoNavMessageParser.parseGalileoEphemeris(svid = 1, words = pages(w1, w2, w3, w4, newWord(5))))
    }

    @Test
    fun parse_word1_t0eAndSqrtA_decodedWithCorrectScale() {
        // ICD: t_oe unsigned 14 bits at word bits 16..29, scale 60 s; √A unsigned 32 bits at
        // word bits 94..125 (crossing the Data1/Data2 boundary), scale 2^-19 m^(1/2).
        val t0eRaw = 5_760 // 5760 × 60 = 345_600 s (midweek)
        val sqrtARaw = (5440.625 * 2.0.pow(19)).toLong()

        val w1 =
            newWord(1).also {
                GnssMathUtils.packBits(it, 6, 10, 42) // IODnav = 42
                GnssMathUtils.packBits(it, 16, 14, t0eRaw)
                packLong(it, 94, 32, sqrtARaw)
            }
        val w2 = newWord(2).also { GnssMathUtils.packBits(it, 6, 10, 42) }
        val w3 = newWord(3).also { GnssMathUtils.packBits(it, 6, 10, 42) }
        val w4 = newWord(4).also { GnssMathUtils.packBits(it, 6, 10, 42) }

        val eph = GalileoNavMessageParser.parseGalileoEphemeris(svid = 1, words = pages(w1, w2, w3, w4, newWord(5)))!!
        assertEquals("t0e scale (60 s)", t0eRaw * 60.0, eph.toe, 1e-9)
        assertEquals("√A scale (2^-19)", sqrtARaw * 2.0.pow(-19), eph.sqrtA, 1e-9)
    }

    @Test
    fun parse_word4_tocAndAf0_decodedIndependentlyFromToe() {
        // Galileo uses distinct t_oe (Word 1) and t_oc (Word 4); set them ~10 min apart.
        val t0eRaw = 5_760
        val t0cRaw = 5_770
        val af0Raw = 12_345L // scale 2^-34

        val w1 =
            newWord(1).also {
                GnssMathUtils.packBits(it, 6, 10, 7)
                GnssMathUtils.packBits(it, 16, 14, t0eRaw)
            }
        val w2 = newWord(2).also { GnssMathUtils.packBits(it, 6, 10, 7) }
        val w3 = newWord(3).also { GnssMathUtils.packBits(it, 6, 10, 7) }
        val w4 =
            newWord(4).also {
                GnssMathUtils.packBits(it, 6, 10, 7)
                GnssMathUtils.packBits(it, 54, 14, t0cRaw)
                packLong(it, 68, 31, af0Raw)
            }

        val eph = GalileoNavMessageParser.parseGalileoEphemeris(svid = 3, words = pages(w1, w2, w3, w4, newWord(5)))!!
        assertEquals("toe must come from W1", t0eRaw * 60.0, eph.toe, 1e-9)
        assertEquals("toc must come from W4 (not defaulted to toe)", t0cRaw * 60.0, eph.toc, 1e-9)
        assertEquals("af0 scale (2^-34)", af0Raw * 2.0.pow(-34), eph.af0, 1e-12)
        assertTrue("toc and toe must actually differ in this test", eph.toc != eph.toe)
    }

    @Test
    fun parse_word5_weekAndBgdE1E5b_decoded() {
        // The I/NAV clock pairs with BGD(E1,E5b) at word bits 57..66 — NOT BGD(E1,E5a) at
        // 47..56, which belongs with the F/NAV clock. Both are set to different values so a
        // regression to the E5a field fails.
        val week = 1234
        val bgdE5aRaw = 77
        val bgdE5bRaw = -100

        val w5 =
            newWord(5).also {
                GnssMathUtils.packBits(it, 47, 10, bgdE5aRaw and 0x3FF)
                GnssMathUtils.packBits(it, 57, 10, bgdE5bRaw and 0x3FF)
                GnssMathUtils.packBits(it, 73, 12, week)
            }

        val eph =
            GalileoNavMessageParser.parseGalileoEphemeris(
                svid = 1,
                words = pages(newWord(1), newWord(2), newWord(3), newWord(4), w5),
            )!!
        assertEquals("Week comes from W5 word bits 73..84", week, eph.week)
        assertEquals("tgd must be BGD(E1,E5b)", bgdE5bRaw * 2.0.pow(-32), eph.tgd, 1e-15)
        assertEquals(bgdE5aRaw * 2.0.pow(-32), eph.galileoBgdE1E5aSeconds, 1e-15)
        assertEquals(bgdE5bRaw * 2.0.pow(-32), eph.galileoBgdE1E5bSeconds, 1e-15)
        assertTrue(eph.galileoInavClock)
    }

    @Test
    fun parse_word2_m0AndOmega0_signedAndScaled() {
        val m0Target = -PI / 2.0
        val m0Raw = (m0Target / (2.0.pow(-31) * PI)).toLong()
        val omega0Target = PI / 4.0
        val omega0Raw = (omega0Target / (2.0.pow(-31) * PI)).toLong()

        val w1 =
            newWord(1).also {
                GnssMathUtils.packBits(it, 6, 10, 1)
                packLong(it, 30, 32, m0Raw and 0xFFFFFFFFL)
            }
        val w2 =
            newWord(2).also {
                GnssMathUtils.packBits(it, 6, 10, 1)
                packLong(it, 16, 32, omega0Raw and 0xFFFFFFFFL)
            }
        val w3 = newWord(3).also { GnssMathUtils.packBits(it, 6, 10, 1) }
        val w4 = newWord(4).also { GnssMathUtils.packBits(it, 6, 10, 1) }

        val eph = GalileoNavMessageParser.parseGalileoEphemeris(svid = 1, words = pages(w1, w2, w3, w4, newWord(5)))!!
        assertEquals("M0 sign and scale", m0Target, eph.m0, 1e-6)
        assertEquals("Ω₀ sign and scale", omega0Target, eph.omega0, 1e-6)
    }

    @Test
    fun parse_fieldsCrossingTheDataBoundary_recovered() {
        // Every field extending past I/NAV word bit 112 lives partly in Data(2/2) of the odd
        // page part: √A (W1), IDOT (W2), Crs (W3), af1 + af2 (W4).
        val sqrtARaw = 2_852_716_549L
        val iDotRaw = -77L
        val crsRaw = -1781L
        val af1Raw = -54_321L
        val af2Raw = -17L
        val iod = 42

        val w1 =
            newWord(1).also {
                GnssMathUtils.packBits(it, 6, 10, iod)
                packLong(it, 94, 32, sqrtARaw)
            }
        val w2 =
            newWord(2).also {
                GnssMathUtils.packBits(it, 6, 10, iod)
                packLong(it, 112, 14, iDotRaw and 0x3FFF)
            }
        val w3 =
            newWord(3).also {
                GnssMathUtils.packBits(it, 6, 10, iod)
                packLong(it, 104, 16, crsRaw and 0xFFFF)
            }
        val w4 =
            newWord(4).also {
                GnssMathUtils.packBits(it, 6, 10, iod)
                packLong(it, 99, 21, af1Raw and 0x1FFFFF)
                packLong(it, 120, 6, af2Raw and 0x3F)
            }

        val eph = GalileoNavMessageParser.parseGalileoEphemeris(svid = 11, words = pages(w1, w2, w3, w4, newWord(5)))!!
        assertEquals(sqrtARaw * 2.0.pow(-19), eph.sqrtA, 1e-9)
        assertEquals(iDotRaw * 2.0.pow(-43) * PI, eph.iDot, 1e-18)
        assertEquals(crsRaw * 2.0.pow(-5), eph.crs, 1e-9)
        assertEquals(af1Raw * 2.0.pow(-46), eph.af1, 1e-18)
        assertEquals(af2Raw * 2.0.pow(-59), eph.af2, 1e-21)
    }

    @Test
    fun parse_alertPage_rejected() {
        val words =
            mapOf(
                1 to page(newWord(1), alertPage = true),
                2 to page(newWord(2)),
                3 to page(newWord(3)),
                4 to page(newWord(4)),
                5 to page(newWord(5)),
            )
        assertNull(GalileoNavMessageParser.parseGalileoEphemeris(svid = 1, words = words))
    }

    @Test
    fun parse_bufferWithoutPagePairFraming_rejected() {
        // No contiguous-buffer guessing: a 17-byte buffer (or one without the even/odd
        // framing flags) has unknown layout and must be rejected, not mis-decoded.
        val words =
            mapOf(
                1 to ByteArray(17),
                2 to page(newWord(2)),
                3 to page(newWord(3)),
                4 to page(newWord(4)),
                5 to page(newWord(5)),
            )
        assertNull(GalileoNavMessageParser.parseGalileoEphemeris(svid = 1, words = words))
    }

    @Test
    fun parse_unhealthyE1Signal_rejected() {
        // E1B-HS != 0 (word bits 69-70) → satellite must not be used for ranging.
        val w5Unhealthy = newWord(5).also { GnssMathUtils.packBits(it, 69, 2, 1) }
        assertNull(
            GalileoNavMessageParser.parseGalileoEphemeris(
                svid = 2,
                words = pages(newWord(1), newWord(2), newWord(3), newWord(4), w5Unhealthy),
            ),
        )
        // E1B-DVS = 1 (word bit 72, "working without guarantee") → likewise rejected.
        val w5NoGuarantee = newWord(5).also { GnssMathUtils.packBits(it, 72, 1, 1) }
        assertNull(
            GalileoNavMessageParser.parseGalileoEphemeris(
                svid = 2,
                words = pages(newWord(1), newWord(2), newWord(3), newWord(4), w5NoGuarantee),
            ),
        )
    }
}
