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

class BeidouNavMessageParserTest {
    // ------------------------------------------------------------------
    // ICD transmit-chain encoder (BDS-SIS-ICD-B1I §5.1.3): information
    // bits → two BCH(15,11,1) codewords per word → bit-by-bit interleave.
    // The parser implements the inverse; if its de-interleave, BCH check
    // or split-field reconstruction is wrong, these tests fail.
    // ------------------------------------------------------------------

    private fun bchRemainder(cw: Int): Int {
        var r = cw
        for (i in 14 downTo 4) {
            if ((r shr i) and 1 == 1) r = r xor (0b10011 shl (i - 4))
        }
        return r and 0xF
    }

    private fun bchEncode(info11: Int): Int {
        val shifted = (info11 and 0x7FF) shl 4
        return shifted or bchRemainder(shifted)
    }

    /** t[0]=cw1[0], t[1]=cw2[0], t[2]=cw1[1], … per ICD §5.1.3. */
    private fun interleave(
        cw1: Int,
        cw2: Int,
    ): Int {
        var w = 0
        for (k in 14 downTo 0) {
            w = (w shl 1) or ((cw1 shr k) and 1)
            w = (w shl 1) or ((cw2 shr k) and 1)
        }
        return w
    }

    /** Encodes an information-layout subframe into the as-broadcast (interleaved) form. */
    private fun encodeInterleaved(info: ByteArray): ByteArray {
        val out = ByteArray(38)
        val w0Direct = GnssMathUtils.getBits(info, 0, 15)
        val w0Info = GnssMathUtils.getBits(info, 15, 11)
        GnssMathUtils.packBits(out, 0, 30, (w0Direct shl 15) or bchEncode(w0Info))
        for (w in 1 until 10) {
            val info1 = GnssMathUtils.getBits(info, w * 30, 11)
            val info2 = GnssMathUtils.getBits(info, w * 30 + 11, 11)
            GnssMathUtils.packBits(out, w * 30, 30, interleave(bchEncode(info1), bchEncode(info2)))
        }
        return out
    }

    /** Encodes into the decode layout (already de-interleaved, parity in place) — HAL variant. */
    private fun encodeDeinterleaved(info: ByteArray): ByteArray {
        val out = ByteArray(38)
        val w0Direct = GnssMathUtils.getBits(info, 0, 15)
        val w0Info = GnssMathUtils.getBits(info, 15, 11)
        GnssMathUtils.packBits(out, 0, 30, (w0Direct shl 15) or bchEncode(w0Info))
        for (w in 1 until 10) {
            val info1 = GnssMathUtils.getBits(info, w * 30, 11)
            val info2 = GnssMathUtils.getBits(info, w * 30 + 11, 11)
            val cw1 = bchEncode(info1)
            val cw2 = bchEncode(info2)
            val word = ((cw1 ushr 4) shl 19) or ((cw2 ushr 4) shl 8) or ((cw1 and 0xF) shl 4) or (cw2 and 0xF)
            GnssMathUtils.packBits(out, w * 30, 30, word)
        }
        return out
    }

    /** Places [value] (masked to the total width) across MSB-first (start, length) segments. */
    private fun putSplit(
        buf: ByteArray,
        value: Long,
        vararg segments: Pair<Int, Int>,
    ) {
        val total = segments.sumOf { it.second }
        var remaining = total
        val masked = value and ((1L shl total) - 1)
        for ((start, length) in segments) {
            remaining -= length
            GnssMathUtils.packBits(buf, start, length, ((masked shr remaining) and ((1L shl length) - 1)).toInt())
        }
    }

    private fun assertScaled(
        expected: Double,
        actual: Double,
    ) = assertEquals(expected, actual, abs(expected) * 1e-12 + 1e-18)

    // Reference raw values: realistic BDS MEO (sqrtA ≈ 5282.9, e ≈ 0.005), odd low
    // bits to catch dropped LSBs, mixed signs for two's-complement reconstruction.
    private val week = 810L
    private val sow1 = 345_600L
    private val tocRaw = 43_200L // ×8 = 345,600 s
    private val tgd1Raw = -34L // ×0.1 ns
    private val af2Raw = -3L
    private val af0Raw = -123_457L
    private val af1Raw = 8_123L
    private val deltaNRaw = 7_321L
    private val cucRaw = -23_457L
    private val m0Raw = -1_234_567_891L
    private val eRaw = 42_949_673L // ×2^-33 ≈ 0.005
    private val cusRaw = 31_415L
    private val crcRaw = 9_241L
    private val crsRaw = -3_217L
    private val sqrtARaw = 2_769_748_541L // ×2^-19 ≈ 5282.88
    private val toeRaw = 43_200L // 17 bits: 2 MSBs in SF2, 15 LSBs in SF3
    private val i0Raw = 654_321_987L
    private val cicRaw = -777L
    private val omegaDotRaw = -45_678L
    private val cisRaw = 555L
    private val iDotRaw = -2_345L
    private val omega0Raw = 987_654_321L
    private val omegaRaw = -1_456_789_123L

    private fun infoHeader(
        buf: ByteArray,
        fraId: Int,
        sow: Long,
    ) {
        putSplit(buf, 0b11100010010L, 0 to 11) // preamble
        putSplit(buf, fraId.toLong(), 15 to 3)
        putSplit(buf, sow, 18 to 8, 30 to 12)
    }

    private fun buildSf1Info(
        satH1: Long = 0L,
        sow: Long = sow1,
    ): ByteArray =
        ByteArray(38).also { b ->
            infoHeader(b, 1, sow)
            putSplit(b, satH1, 42 to 1)
            putSplit(b, 1L, 43 to 5) // AODC
            putSplit(b, 2L, 48 to 4) // URAI
            putSplit(b, week, 60 to 13)
            putSplit(b, tocRaw, 73 to 9, 90 to 8)
            putSplit(b, tgd1Raw, 98 to 10)
            putSplit(b, af2Raw, 214 to 11)
            putSplit(b, af0Raw, 225 to 7, 240 to 17)
            putSplit(b, af1Raw, 257 to 5, 270 to 17)
            putSplit(b, 1L, 287 to 5) // AODE
        }

    private fun buildSf2Info(sow: Long = sow1 + 6): ByteArray =
        ByteArray(38).also { b ->
            infoHeader(b, 2, sow)
            putSplit(b, deltaNRaw, 42 to 10, 60 to 6)
            putSplit(b, cucRaw, 66 to 16, 90 to 2)
            putSplit(b, m0Raw, 92 to 20, 120 to 12)
            putSplit(b, eRaw, 132 to 10, 150 to 22)
            putSplit(b, cusRaw, 180 to 18)
            putSplit(b, crcRaw, 198 to 4, 210 to 14)
            putSplit(b, crsRaw, 224 to 8, 240 to 10)
            putSplit(b, sqrtARaw, 250 to 12, 270 to 20)
            putSplit(b, toeRaw shr 15, 290 to 2)
        }

    private fun buildSf3Info(sow: Long = sow1 + 12): ByteArray =
        ByteArray(38).also { b ->
            infoHeader(b, 3, sow)
            putSplit(b, toeRaw and 0x7FFF, 42 to 10, 60 to 5)
            putSplit(b, i0Raw, 65 to 17, 90 to 15)
            putSplit(b, cicRaw, 105 to 7, 120 to 11)
            putSplit(b, omegaDotRaw, 131 to 11, 150 to 13)
            putSplit(b, cisRaw, 163 to 9, 180 to 9)
            putSplit(b, iDotRaw, 189 to 13, 210 to 1)
            putSplit(b, omega0Raw, 211 to 21, 240 to 11)
            putSplit(b, omegaRaw, 251 to 11, 270 to 21)
        }

    private fun assertReferenceEphemeris(eph: app.simt.gnss.core.model.SatelliteEphemeris) {
        assertEquals(Constellation.BEIDOU, eph.constellationType)
        assertEquals(week.toInt(), eph.week)
        assertScaled(tocRaw * 8.0, eph.toc)
        assertScaled(toeRaw * 8.0, eph.toe)
        assertScaled(tgd1Raw * 0.1 * 1e-9, eph.tgd)
        assertScaled(af2Raw * 2.0.pow(-66), eph.af2)
        assertScaled(af0Raw * 2.0.pow(-33), eph.af0)
        assertScaled(af1Raw * 2.0.pow(-50), eph.af1)
        assertScaled(deltaNRaw * 2.0.pow(-43) * PI, eph.deltaN)
        assertScaled(cucRaw * 2.0.pow(-31), eph.cuc)
        assertScaled(m0Raw * 2.0.pow(-31) * PI, eph.m0)
        assertScaled(eRaw * 2.0.pow(-33), eph.eccentricity)
        assertScaled(cusRaw * 2.0.pow(-31), eph.cus)
        assertScaled(crcRaw * 2.0.pow(-6), eph.crc)
        assertScaled(crsRaw * 2.0.pow(-6), eph.crs)
        assertScaled(sqrtARaw * 2.0.pow(-19), eph.sqrtA)
        assertScaled(i0Raw * 2.0.pow(-31) * PI, eph.i0)
        assertScaled(cicRaw * 2.0.pow(-31), eph.cic)
        assertScaled(omegaDotRaw * 2.0.pow(-43) * PI, eph.omegaDot)
        assertScaled(cisRaw * 2.0.pow(-31), eph.cis)
        assertScaled(iDotRaw * 2.0.pow(-43) * PI, eph.iDot)
        assertScaled(omega0Raw * 2.0.pow(-31) * PI, eph.omega0)
        assertScaled(omegaRaw * 2.0.pow(-31) * PI, eph.omega)
    }

    @Test
    fun parse_interleavedBroadcastForm_recoversReferenceValues() {
        val eph =
            BeidouNavMessageParser.parseBeidouEphemeris(
                svid = 6,
                sf1Raw = encodeInterleaved(buildSf1Info()),
                sf2Raw = encodeInterleaved(buildSf2Info()),
                sf3Raw = encodeInterleaved(buildSf3Info()),
            )
        assertNotNull("Parser must decode the as-broadcast interleaved form", eph)
        assertEquals(6, eph!!.svid)
        assertReferenceEphemeris(eph)
    }

    @Test
    fun parse_deinterleavedHalForm_alsoAccepted() {
        val eph =
            BeidouNavMessageParser.parseBeidouEphemeris(
                svid = 9,
                sf1Raw = encodeDeinterleaved(buildSf1Info()),
                sf2Raw = encodeDeinterleaved(buildSf2Info()),
                sf3Raw = encodeDeinterleaved(buildSf3Info()),
            )
        assertNotNull("Parser must accept HALs that deliver de-interleaved words", eph)
        assertReferenceEphemeris(eph!!)
    }

    @Test
    fun parse_orbitIsPhysical() {
        val eph =
            BeidouNavMessageParser.parseBeidouEphemeris(
                svid = 6,
                sf1Raw = encodeInterleaved(buildSf1Info()),
                sf2Raw = encodeInterleaved(buildSf2Info()),
                sf3Raw = encodeInterleaved(buildSf3Info()),
            )!!
        assertEquals(5282.88, eph.sqrtA, 0.05) // BDS MEO semi-major axis ≈ 27,906 km
        assertEquals(0.005, eph.eccentricity, 0.0005)
    }

    @Test
    fun parse_corruptedBch_returnsNull() {
        val sf2 = encodeInterleaved(buildSf2Info())
        sf2[10] = (sf2[10].toInt() xor 0x10).toByte() // flip one bit inside word 3
        assertNull(
            BeidouNavMessageParser.parseBeidouEphemeris(
                svid = 6,
                sf1Raw = encodeInterleaved(buildSf1Info()),
                sf2Raw = sf2,
                sf3Raw = encodeInterleaved(buildSf3Info()),
            ),
        )
    }

    @Test
    fun parse_sowDiscontinuity_returnsNull() {
        // SF3 from a later frame (sow + 30) must be rejected: it can belong to a
        // different data set than SF1/SF2.
        assertNull(
            BeidouNavMessageParser.parseBeidouEphemeris(
                svid = 6,
                sf1Raw = encodeInterleaved(buildSf1Info()),
                sf2Raw = encodeInterleaved(buildSf2Info()),
                sf3Raw = encodeInterleaved(buildSf3Info(sow = sow1 + 42)),
            ),
        )
    }

    @Test
    fun parse_unhealthySatellite_returnsNull() {
        assertNull(
            BeidouNavMessageParser.parseBeidouEphemeris(
                svid = 6,
                sf1Raw = encodeInterleaved(buildSf1Info(satH1 = 1L)),
                sf2Raw = encodeInterleaved(buildSf2Info()),
                sf3Raw = encodeInterleaved(buildSf3Info()),
            ),
        )
    }

    @Test
    fun parse_allZeroSubframes_rejected() {
        // Zero words pass BCH trivially but produce a non-physical orbit (sqrtA = 0)
        // and FraID 0 — must not yield an ephemeris.
        assertNull(BeidouNavMessageParser.parseBeidouEphemeris(6, ByteArray(38), ByteArray(38), ByteArray(38)))
    }
}
