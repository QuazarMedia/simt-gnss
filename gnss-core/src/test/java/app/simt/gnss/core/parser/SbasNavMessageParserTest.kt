package app.simt.gnss.core.parser
import app.simt.gnss.core.model.Constellation
import app.simt.gnss.core.utils.GnssMathUtils
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class SbasNavMessageParserTest {
    // -------------------------------------------------------------------------
    //  Helpers — pack known values into a 250-bit (32-byte) SBAS frame
    // -------------------------------------------------------------------------

    private fun newFrame(messageType: Int): ByteArray {
        val data = ByteArray(32)
        // Preamble (bits 0–7): 0x53 (standard SBAS preamble)
        data[0] = 0x53
        // Message type (bits 8–13): 6 bits
        GnssMathUtils.packBits(data, 8, 6, messageType)
        return data
    }

    /** Pack a two's-complement signed value into the frame. */
    private fun packSigned(
        data: ByteArray,
        startBit: Int,
        length: Int,
        value: Int,
    ) {
        val unsigned = if (value >= 0) value else value + (1 shl length)
        GnssMathUtils.packBits(data, startBit, length, unsigned)
    }

    // -------------------------------------------------------------------------
    //  getMessageType
    // -------------------------------------------------------------------------

    @Test
    fun getMessageType_extractsCorrectType() {
        for (mt in intArrayOf(0, 1, 2, 5, 9, 18, 24, 25, 26, 63)) {
            val data = newFrame(mt)
            assertEquals("MT $mt", mt, SbasNavMessageParser.getMessageType(data))
        }
    }

    // -------------------------------------------------------------------------
    //  MT 9 — GEO Navigation Message
    // -------------------------------------------------------------------------

    @Test
    fun parseMt9_extractsPositionAndClockAtKnownBitOffsets() {
        val data = newFrame(9)
        // t0 (bits 22–34, 13 bits unsigned, LSB = 16 s) → set raw = 100 → t0 = 1600 s
        GnssMathUtils.packBits(data, 22, 13, 100)
        // X_G (bits 39–68, 30 bits signed, LSB = 0.08 m) → +1000 → 80.0 m
        packSigned(data, 39, 30, 1000)
        // Y_G (bits 69–98, 30 bits signed, LSB = 0.08 m) → -500 → -40.0 m
        packSigned(data, 69, 30, -500)
        // Z_G (bits 99–123, 25 bits signed, LSB = 0.4 m) → +200 → 80.0 m
        packSigned(data, 99, 25, 200)
        // X_G_dot (bits 124–140, 17 bits signed, LSB = 0.000625 m/s) → +10
        packSigned(data, 124, 17, 10)
        // Y_G_dot (bits 141–157, 17 bits signed, LSB = 0.000625 m/s) → -10
        packSigned(data, 141, 17, -10)
        // Z_G_dot (bits 158–175, 18 bits signed, LSB = 0.004 m/s) → +5
        packSigned(data, 158, 18, 5)
        // X_G_ddot (bits 176–185, 10 bits signed, LSB = 0.0000125 m/s²) → +3
        packSigned(data, 176, 10, 3)
        // Y_G_ddot (bits 186–195, 10 bits signed, LSB = 0.0000125 m/s²) → -3
        packSigned(data, 186, 10, -3)
        // Z_G_ddot (bits 196–205, 10 bits signed, LSB = 0.0000625 m/s²) → +2
        packSigned(data, 196, 10, 2)
        // aGf0 (bits 206–217, 12 bits signed, LSB = 2⁻³¹ s) → +100
        packSigned(data, 206, 12, 100)
        // aGf1 (bits 218–225, 8 bits signed, LSB = 2⁻⁴⁰ s/s) → -5
        packSigned(data, 218, 8, -5)

        val eph = SbasNavMessageParser.parseMt9GeoNavigation(135, data)
        assertNotNull(eph)
        eph!!

        assertEquals(Constellation.SBAS, eph.constellationType)
        assertEquals(135, eph.svid)
        assertTrue(eph.isSbas)

        // t0 = 100 * 16 = 1600 s
        assertEquals(1600.0, eph.sbasT0, 1e-9)
        // Position
        assertEquals(1000 * 0.08, eph.sbasPosX, 1e-9)
        assertEquals(-500 * 0.08, eph.sbasPosY, 1e-9)
        assertEquals(200 * 0.4, eph.sbasPosZ, 1e-9)
        // Velocity
        assertEquals(10 * 0.000625, eph.sbasVelX, 1e-12)
        assertEquals(-10 * 0.000625, eph.sbasVelY, 1e-12)
        assertEquals(5 * 0.004, eph.sbasVelZ, 1e-12)
        // Acceleration
        assertEquals(3 * 0.0000125, eph.sbasAccX, 1e-15)
        assertEquals(-3 * 0.0000125, eph.sbasAccY, 1e-15)
        assertEquals(2 * 0.0000625, eph.sbasAccZ, 1e-15)
        // Clock
        assertEquals(100 * Math.pow(2.0, -31.0), eph.sbasAgf0, 1e-18)
        assertEquals(-5 * Math.pow(2.0, -40.0), eph.sbasAgf1, 1e-18)
        // af0/af1 should mirror SBAS clock fields
        assertEquals(eph.sbasAgf0, eph.af0, 1e-18)
        assertEquals(eph.sbasAgf1, eph.af1, 1e-18)
    }

    @Test
    fun parseMt9_negativeSatellitePosition_correctSignExtension() {
        val data = newFrame(9)
        // X_G = -1 (all ones in 30-bit two's complement) → -0.08 m
        packSigned(data, 39, 30, -1)
        // Y_G = minimum negative: -(2^29) = -536870912
        val minNeg30 = -(1 shl 29)
        packSigned(data, 69, 30, minNeg30)

        val eph = SbasNavMessageParser.parseMt9GeoNavigation(120, data)
        assertNotNull(eph)
        assertEquals(-0.08, eph!!.sbasPosX, 1e-9)
        assertEquals(minNeg30 * 0.08, eph.sbasPosY, 1e-3)
    }

    @Test
    fun parseMt9_uraDoNotUse_returnsNull() {
        // URA index 15 (bits 35–38) = "do not use for ranging" (DO-229 §A.4.4.11).
        val data = newFrame(9)
        GnssMathUtils.packBits(data, 35, 4, 15)
        packSigned(data, 39, 30, 1000)
        assertNull(SbasNavMessageParser.parseMt9GeoNavigation(135, data))
    }

    // -------------------------------------------------------------------------
    //  MT 1 — PRN Mask
    // -------------------------------------------------------------------------

    @Test
    fun parsePrnMask_singleGpsPrn_mapsCorrectlyWithIodp() {
        val data = newFrame(1)
        // Set PRN 5 active: bit at 13 + 5 = 18; IODP (bits 224–225) = 2
        GnssMathUtils.packBits(data, 18, 1, 1)
        GnssMathUtils.packBits(data, 224, 2, 2)
        val mask = SbasNavMessageParser.parsePrnMask(data)
        assertNotNull(mask)
        assertEquals(2, mask!!.iodp)
        assertEquals(1, mask.maskToPrn.size)
        assertEquals(5, mask.maskToPrn[0])
    }

    @Test
    fun parsePrnMask_multipleGpsPrns_assignsSequentialMaskIndices() {
        val data = newFrame(1)
        // Set PRN 1 (bit 14), PRN 3 (bit 16), PRN 10 (bit 23) active
        GnssMathUtils.packBits(data, 14, 1, 1)
        GnssMathUtils.packBits(data, 16, 1, 1)
        GnssMathUtils.packBits(data, 23, 1, 1)
        val mask = SbasNavMessageParser.parsePrnMask(data)
        assertNotNull(mask)
        assertEquals(3, mask!!.maskToPrn.size)
        assertEquals(1, mask.maskToPrn[0]) // mask index 0 → PRN 1
        assertEquals(3, mask.maskToPrn[1]) // mask index 1 → PRN 3
        assertEquals(10, mask.maskToPrn[2]) // mask index 2 → PRN 10
    }

    @Test
    fun parsePrnMask_sbasPrnRange_mapsCorrectly() {
        val data = newFrame(1)
        // Set PRN 120 active: bit at 13 + 120 = 133
        GnssMathUtils.packBits(data, 133, 1, 1)
        val mask = SbasNavMessageParser.parsePrnMask(data)
        assertNotNull(mask)
        assertEquals(1, mask!!.maskToPrn.size)
        assertEquals(120, mask.maskToPrn[0])
    }

    @Test
    fun parsePrnMask_emptyMask_returnsEmptyMap() {
        val data = newFrame(1)
        val mask = SbasNavMessageParser.parsePrnMask(data)
        assertNotNull(mask)
        assertTrue(mask!!.maskToPrn.isEmpty())
    }

    // -------------------------------------------------------------------------
    //  MT 2–5 — Fast Corrections
    // -------------------------------------------------------------------------

    @Test
    fun parseFastCorrections_mt2_extractsPrcsUdreisAndIodp() {
        val data = newFrame(2)
        // IODP at bits 16–17 → 3
        GnssMathUtils.packBits(data, 16, 2, 3)
        // PRC for satellite 0: bits 18–29, 12 bits signed, LSB = 0.125 m → set +16 → 2.0 m
        packSigned(data, 18, 12, 16)
        // UDREI for satellite 0: bits 174–177, 4 bits → set 3
        GnssMathUtils.packBits(data, 174, 4, 3)
        // PRC for satellite 1: bits 30–41 → set -8 → -1.0 m
        packSigned(data, 30, 12, -8)
        // UDREI for satellite 1: bits 178–181 → set 14 (not monitored)
        GnssMathUtils.packBits(data, 178, 4, 14)

        val corrections = SbasNavMessageParser.parseFastCorrections(data, 2)
        assertNotNull(corrections)
        assertEquals(13, corrections!!.size)
        // MT 2 → baseIndex = 0
        assertEquals(0, corrections[0].prnMaskIndex)
        assertEquals(16 * 0.125, corrections[0].prc, 1e-9)
        assertEquals(3, corrections[0].udrei)
        assertEquals("IODP must qualify the corrections", 3, corrections[0].iodp)

        assertEquals(1, corrections[1].prnMaskIndex)
        assertEquals(-8 * 0.125, corrections[1].prc, 1e-9)
        assertEquals(14, corrections[1].udrei)
    }

    @Test
    fun parseFastCorrections_mt3_baseIndexIs13() {
        val data = newFrame(3)
        val corrections = SbasNavMessageParser.parseFastCorrections(data, 3)
        assertNotNull(corrections)
        assertEquals(13, corrections!![0].prnMaskIndex)
    }

    @Test
    fun parseFastCorrections_mt5_baseIndexIs39() {
        val data = newFrame(5)
        val corrections = SbasNavMessageParser.parseFastCorrections(data, 5)
        assertNotNull(corrections)
        assertEquals(39, corrections!![0].prnMaskIndex)
    }

    @Test
    fun parseFastCorrections_invalidMessageType_returnsNull() {
        val data = newFrame(9)
        assertNull(SbasNavMessageParser.parseFastCorrections(data, 1))
        assertNull(SbasNavMessageParser.parseFastCorrections(data, 6))
    }

    // -------------------------------------------------------------------------
    //  MT 24 — Mixed Fast + Long-term Corrections
    // -------------------------------------------------------------------------

    @Test
    fun parseMt24_extractsFastCorrectionsWithBlockIdAndIodp() {
        val data = newFrame(24)
        // IODP at bits 110–111 → 1; Block ID at bits 112–113 → 2 → baseIndex = 26
        GnssMathUtils.packBits(data, 110, 2, 1)
        GnssMathUtils.packBits(data, 112, 2, 2)
        // PRC for satellite 0: bits 14–25, 12 bits signed → +24 → 3.0 m
        packSigned(data, 14, 12, 24)
        // UDREI for satellite 0: bits 86–89 → 5
        GnssMathUtils.packBits(data, 86, 4, 5)

        val result = SbasNavMessageParser.parseMt24MixedCorrections(data)
        assertNotNull(result)
        assertEquals(6, result!!.fastCorrections.size)
        assertEquals(26, result.fastCorrections[0].prnMaskIndex)
        assertEquals(24 * 0.125, result.fastCorrections[0].prc, 1e-9)
        assertEquals(5, result.fastCorrections[0].udrei)
        assertEquals(1, result.fastCorrections[0].iodp)
    }

    @Test
    fun parseMt24_longTermHalfWithVelocityCode0_parsesMaskNumberIodeAndIodp() {
        val data = newFrame(24)
        // Velocity code at bit 120 → 0 (no velocity)
        GnssMathUtils.packBits(data, 120, 1, 0)
        // First half at base = 121:
        //   PRN mask number (6 bits) at 121 → 5 (a mask SLOT, not a PRN)
        GnssMathUtils.packBits(data, 121, 6, 5)
        //   IODE (8 bits) at 127 → 46
        GnssMathUtils.packBits(data, 127, 8, 46)
        //   delta-X (9 bits signed) at 135 → +4 → 0.5 m
        packSigned(data, 135, 9, 4)
        //   delta-Y (9 bits signed) at 144 → -2 → -0.25 m
        packSigned(data, 144, 9, -2)
        //   delta-Z (9 bits signed) at 153 → +1 → 0.125 m
        packSigned(data, 153, 9, 1)
        //   delta-af0 (10 bits signed) at 162 → +10
        packSigned(data, 162, 10, 10)
        // Section IODP at 120 + 103 = 223 → 2
        GnssMathUtils.packBits(data, 223, 2, 2)

        val result = SbasNavMessageParser.parseMt24MixedCorrections(data)
        assertNotNull(result)
        val ltcs = result!!.longTermCorrections
        assertTrue(ltcs.isNotEmpty())
        assertEquals(5, ltcs[0].prnMaskNumber)
        assertEquals(46, ltcs[0].iode)
        assertEquals(2, ltcs[0].iodp)
        assertEquals(4 * 0.125, ltcs[0].deltaX, 1e-9)
        assertEquals(-2 * 0.125, ltcs[0].deltaY, 1e-9)
        assertEquals(1 * 0.125, ltcs[0].deltaZ, 1e-9)
        assertEquals(10 * Math.pow(2.0, -31.0), ltcs[0].deltaAf0, 1e-18)
    }

    // -------------------------------------------------------------------------
    //  MT 25 — Long-term Corrections
    // -------------------------------------------------------------------------

    @Test
    fun parseLongTermCorrections_velocityCode0_parsesTwoHalvesPerSection() {
        val data = newFrame(25)
        // Section 1 (bit 14): velocity code = 0
        GnssMathUtils.packBits(data, 14, 1, 0)
        // First half at base = 15:
        //   PRN mask number (6 bits) at 15 → 3
        GnssMathUtils.packBits(data, 15, 6, 3)
        //   IODE (8 bits) at 21 → 101
        GnssMathUtils.packBits(data, 21, 8, 101)
        //   delta-X (9 bits signed) at 29 → +8 → 1.0 m
        packSigned(data, 29, 9, 8)
        //   delta-Y (9 bits signed) at 38 → -4 → -0.5 m
        packSigned(data, 38, 9, -4)
        //   delta-Z (9 bits signed) at 47 → +2 → 0.25 m
        packSigned(data, 47, 9, 2)
        //   delta-af0 (10 bits signed) at 56 → +20
        packSigned(data, 56, 10, 20)
        // Section 1 IODP at 14 + 103 = 117 → 1
        GnssMathUtils.packBits(data, 117, 2, 1)

        // Second half at base = 66:
        //   PRN mask number (6 bits) at 66 → 7
        GnssMathUtils.packBits(data, 66, 6, 7)
        //   delta-X at 80 → -1
        packSigned(data, 80, 9, -1)

        // Section 2 (bit 120): velocity code = 0
        GnssMathUtils.packBits(data, 120, 1, 0)
        // First half at base = 121:
        //   PRN mask number (6 bits) at 121 → 12
        GnssMathUtils.packBits(data, 121, 6, 12)
        //   delta-X at 135 → +3
        packSigned(data, 135, 9, 3)
        // Section 2 IODP at 223 → 3
        GnssMathUtils.packBits(data, 223, 2, 3)

        val corrections = SbasNavMessageParser.parseLongTermCorrections(data)
        assertNotNull(corrections)
        // Up to 4 corrections: 2 halves × 2 sections
        assertTrue("Expected at least 3 corrections", corrections!!.size >= 3)

        // Check first half of section 1
        val c0 = corrections[0]
        assertEquals(3, c0.prnMaskNumber)
        assertEquals(101, c0.iode)
        assertEquals("Section 1 corrections carry the section IODP", 1, c0.iodp)
        assertEquals(8 * 0.125, c0.deltaX, 1e-9)
        assertEquals(-4 * 0.125, c0.deltaY, 1e-9)
        assertEquals(2 * 0.125, c0.deltaZ, 1e-9)
        assertEquals(20 * Math.pow(2.0, -31.0), c0.deltaAf0, 1e-18)

        // Check second half of section 1
        val c1 = corrections[1]
        assertEquals(7, c1.prnMaskNumber)
        assertEquals(-1 * 0.125, c1.deltaX, 1e-9)

        // Check first half of section 2 — carries its own section IODP
        val c2 = corrections.first { it.prnMaskNumber == 12 }
        assertEquals(3 * 0.125, c2.deltaX, 1e-9)
        assertEquals(3, c2.iodp)
    }

    @Test
    fun parseLongTermCorrections_velocityCode1_skipsSection() {
        val data = newFrame(25)
        // Section 1: velocity code = 1 (not implemented, should be skipped)
        GnssMathUtils.packBits(data, 14, 1, 1)
        // Section 2: velocity code = 0 with PRN mask number = 0 (should be skipped)
        GnssMathUtils.packBits(data, 120, 1, 0)
        GnssMathUtils.packBits(data, 121, 6, 0) // mask number = 0 → unused slot

        val corrections = SbasNavMessageParser.parseLongTermCorrections(data)
        assertNotNull(corrections)
        // Section 1 skipped (vel code 1), section 2 halves have mask number 0 → empty
        assertTrue(corrections!!.isEmpty())
    }
}
