package app.simt.gnss.core.parser
import app.simt.gnss.core.model.Constellation
import app.simt.gnss.core.utils.GnssMathUtils
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Test
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.pow

class GpsNavMessageParserTest {
    // ------------------------------------------------------------------
    // ICD-layout encoder. Fields are placed word-by-word per IS-GPS-200
    // Figure 20-1 (1-indexed bit-in-word, 30-bit words with parity bits
    // left in place) — deliberately NOT reusing the parser's offsets, so
    // a layout error in the parser cannot cancel out in a round trip.
    // ------------------------------------------------------------------

    /** Places [length] (≤ 24) bits of [value] at 1-indexed [bitInWord] of 1-indexed [word]. */
    private fun put(
        sf: ByteArray,
        word: Int,
        bitInWord: Int,
        length: Int,
        value: Long,
    ) {
        require(length <= 24)
        val masked = (value and ((1L shl length) - 1)).toInt()
        GnssMathUtils.packBits(sf, (word - 1) * 30 + (bitInWord - 1), length, masked)
    }

    /** 32-bit two-word field: 8 MSBs in bits 17-24 of [msbWord], 24 LSBs in bits 1-24 of the next word. */
    private fun put32(
        sf: ByteArray,
        msbWord: Int,
        value: Long,
    ) {
        val u = value and 0xFFFF_FFFFL
        put(sf, msbWord, 17, 8, u ushr 24)
        put(sf, msbWord + 1, 1, 24, u and 0xFF_FFFF)
    }

    private fun assertScaled(
        expected: Double,
        actual: Double,
    ) = assertEquals(expected, actual, abs(expected) * 1e-12 + 1e-18)

    // Reference raw field values (close to a real GPS ephemeris); low-order
    // bits are deliberately non-zero so dropped LSBs are caught, and several
    // fields are negative to exercise sign extension.
    private val week = 246L
    private val iode = 46L
    private val tgdRaw = -22L
    private val tocRaw = 13050L // toc = 208800 s
    private val af2Raw = 0L
    private val af1Raw = -11L
    private val af0Raw = -116618L
    private val crsRaw = -1781L
    private val deltaNRaw = 12318L
    private val m0Raw = 478_553_701L
    private val cucRaw = -1010L
    private val eRaw = 78_168_185L
    private val cusRaw = 7592L
    private val sqrtARaw = 2_702_021_591L // > 2^31: exercises unsigned 32-bit handling
    private val toeRaw = 13050L
    private val cicRaw = 18L
    private val omega0Raw = -979_252_543L
    private val cisRaw = -41L
    private val i0Raw = 656_964_035L
    private val crcRaw = 7233L
    private val omegaRaw = -1_717_986_916L
    private val omegaDotRaw = -22432L
    private val iDotRaw = 67L

    private fun buildSubframe1(iodcLsb: Long = iode): ByteArray =
        ByteArray(38).also { sf ->
            put(sf, 2, 20, 3, 1) // HOW subframe ID
            put(sf, 3, 1, 10, week)
            put(sf, 3, 23, 2, 0) // IODC 2 MSBs
            put(sf, 7, 17, 8, tgdRaw)
            put(sf, 8, 1, 8, iodcLsb) // IODC 8 LSBs
            put(sf, 8, 9, 16, tocRaw)
            put(sf, 9, 1, 8, af2Raw)
            put(sf, 9, 9, 16, af1Raw)
            put(sf, 10, 1, 22, af0Raw)
        }

    private fun buildSubframe2(iodeValue: Long = iode): ByteArray =
        ByteArray(38).also { sf ->
            put(sf, 2, 20, 3, 2)
            put(sf, 3, 1, 8, iodeValue)
            put(sf, 3, 9, 16, crsRaw)
            put(sf, 4, 1, 16, deltaNRaw)
            put32(sf, 4, m0Raw) // M0: w4 b17-24 + w5 b1-24
            put(sf, 6, 1, 16, cucRaw)
            put32(sf, 6, eRaw) // e: w6 b17-24 + w7 b1-24
            put(sf, 8, 1, 16, cusRaw)
            put32(sf, 8, sqrtARaw) // sqrtA: w8 b17-24 + w9 b1-24
            put(sf, 10, 1, 16, toeRaw)
        }

    private fun buildSubframe3(iodeValue: Long = iode): ByteArray =
        ByteArray(38).also { sf ->
            put(sf, 2, 20, 3, 3)
            put(sf, 3, 1, 16, cicRaw)
            put32(sf, 3, omega0Raw) // Omega0: w3 b17-24 + w4 b1-24
            put(sf, 5, 1, 16, cisRaw)
            put32(sf, 5, i0Raw) // i0: w5 b17-24 + w6 b1-24
            put(sf, 7, 1, 16, crcRaw)
            put32(sf, 7, omegaRaw) // omega: w7 b17-24 + w8 b1-24
            put(sf, 9, 1, 24, omegaDotRaw)
            put(sf, 10, 1, 8, iodeValue)
            put(sf, 10, 9, 14, iDotRaw)
        }

    @Test
    fun parseGpsEphemeris_recoversIcdPlacedReferenceValues() {
        val eph = GpsNavMessageParser.parseGpsEphemeris(7, buildSubframe1(), buildSubframe2(), buildSubframe3())

        assertNotNull(eph)
        eph!!
        assertEquals(Constellation.GPS, eph.constellationType)
        assertEquals(7, eph.svid)
        assertEquals(week.toInt(), eph.week)

        assertScaled(tgdRaw * 2.0.pow(-31), eph.tgd)
        assertScaled(tocRaw * 16.0, eph.toc)
        assertScaled(af2Raw * 2.0.pow(-55), eph.af2)
        assertScaled(af1Raw * 2.0.pow(-43), eph.af1)
        assertScaled(af0Raw * 2.0.pow(-31), eph.af0)

        assertScaled(crsRaw * 2.0.pow(-5), eph.crs)
        assertScaled(deltaNRaw * 2.0.pow(-43) * PI, eph.deltaN)
        assertScaled(m0Raw * 2.0.pow(-31) * PI, eph.m0)
        assertScaled(cucRaw * 2.0.pow(-29), eph.cuc)
        assertScaled(eRaw * 2.0.pow(-33), eph.eccentricity)
        assertScaled(cusRaw * 2.0.pow(-29), eph.cus)
        assertScaled(sqrtARaw * 2.0.pow(-19), eph.sqrtA)
        assertScaled(toeRaw * 16.0, eph.toe)

        assertScaled(cicRaw * 2.0.pow(-29), eph.cic)
        assertScaled(omega0Raw * 2.0.pow(-31) * PI, eph.omega0)
        assertScaled(cisRaw * 2.0.pow(-29), eph.cis)
        assertScaled(i0Raw * 2.0.pow(-31) * PI, eph.i0)
        assertScaled(crcRaw * 2.0.pow(-5), eph.crc)
        assertScaled(omegaRaw * 2.0.pow(-31) * PI, eph.omega)
        assertScaled(omegaDotRaw * 2.0.pow(-43) * PI, eph.omegaDot)
        assertScaled(iDotRaw * 2.0.pow(-43) * PI, eph.iDot)
    }

    @Test
    fun parseGpsEphemeris_sanityCheck_sqrtAAndEccentricityArePhysical() {
        // Catches parity-bit contamination: a corrupted sqrtA/e lands far outside
        // physical GPS orbit values even when individual bits look plausible.
        val eph = GpsNavMessageParser.parseGpsEphemeris(7, buildSubframe1(), buildSubframe2(), buildSubframe3())!!
        assertEquals(5153.68, eph.sqrtA, 0.5) // GPS semi-major axis ≈ 26,560 km
        assertEquals(0.0091, eph.eccentricity, 0.0005)
    }

    @Test
    fun parseGpsEphemeris_mismatchedIode_returnsNull() {
        // SF2 from a newer data set than SF1/SF3 → must be rejected (IS-GPS-200 §20.3.3.4.1).
        assertNull(GpsNavMessageParser.parseGpsEphemeris(7, buildSubframe1(), buildSubframe2(iodeValue = 47), buildSubframe3()))
        // IODC LSBs disagree with IODE.
        assertNull(GpsNavMessageParser.parseGpsEphemeris(7, buildSubframe1(iodcLsb = 45), buildSubframe2(), buildSubframe3()))
    }

    @Test
    fun parseGpsEphemeris_zeroFilledSubframes_stillParses() {
        // Legacy behaviour: all-zero subframes are internally consistent (IODE 0 == IODC 0).
        val eph = GpsNavMessageParser.parseGpsEphemeris(1, ByteArray(38), ByteArray(38), ByteArray(38))
        assertNotNull(eph)
        assertEquals(Constellation.GPS, eph!!.constellationType)
        assertEquals(1, eph.svid)
    }

    @Test
    fun parseKlobucharCoefficients_recoversIcdPlacedReferenceValues() {
        // IS-GPS-200 Fig. 20-1 sheet 8 — word 3: dataId(2) svId(6) α0(8) α1(8);
        // word 4: α2(8) α3(8) β0(8); word 5: β1(8) β2(8) β3(8).
        val sf4 = ByteArray(38)
        put(sf4, 2, 20, 3, 4)
        put(sf4, 3, 1, 2, 1) // data ID
        put(sf4, 3, 3, 6, 56) // SV ID 56 = page 18
        put(sf4, 3, 9, 8, 12) // α0
        put(sf4, 3, 17, 8, 22) // α1
        put(sf4, 4, 1, 8, -1) // α2
        put(sf4, 4, 9, 8, -2) // α3
        put(sf4, 4, 17, 8, 47) // β0
        put(sf4, 5, 1, 8, 8) // β1
        put(sf4, 5, 9, 8, -3) // β2
        put(sf4, 5, 17, 8, -6) // β3

        val result = GpsNavMessageParser.parseKlobucharCoefficients(sf4)
        assertNotNull(result)
        val (alpha, beta) = result!!

        assertScaled(12 * 2.0.pow(-30), alpha[0])
        assertScaled(22 * 2.0.pow(-27), alpha[1])
        assertScaled(-1 * 2.0.pow(-24), alpha[2])
        assertScaled(-2 * 2.0.pow(-24), alpha[3])
        assertScaled(47 * 2.0.pow(11), beta[0])
        assertScaled(8 * 2.0.pow(14), beta[1])
        assertScaled(-3 * 2.0.pow(16), beta[2])
        assertScaled(-6 * 2.0.pow(16), beta[3])
    }

    @Test
    fun parseKlobucharCoefficients_wrongPage_returnsNull() {
        val sf4 = ByteArray(38)
        put(sf4, 3, 3, 6, 55) // SV ID 55 = page 17, not the iono page
        assertNull(GpsNavMessageParser.parseKlobucharCoefficients(sf4))
    }
}
