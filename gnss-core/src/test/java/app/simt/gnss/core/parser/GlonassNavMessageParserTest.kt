package app.simt.gnss.core.parser

import app.simt.gnss.core.utils.GnssMathUtils
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Test
import kotlin.math.abs
import kotlin.math.pow

class GlonassNavMessageParserTest {
    // ------------------------------------------------------------------
    // ICD-layout encoder. GLONASS ICD 5.1 numbers string bits 85..1 from
    // left (idle, bit 85) to right (parity). The Android buffer is the
    // same 85 bits MSB-first, so flat bit = 85 − ICD bit. Fields are
    // placed by their ICD bit numbers from Table 4.5 — deliberately NOT
    // using the parser's flat offsets, so layout bugs cannot round-trip.
    // ------------------------------------------------------------------

    /** Places [length] bits of [value] with the field's MSB at ICD bit [icdMsbBit]. */
    private fun putIcd(
        str: ByteArray,
        icdMsbBit: Int,
        length: Int,
        value: Long,
    ) {
        val flatStart = 85 - icdMsbBit
        GnssMathUtils.packBits(str, flatStart, length, (value and ((1L shl length) - 1)).toInt())
    }

    /** Sign-magnitude field (GLONASS convention): sign at [icdMsbBit], |raw| in the remaining bits. */
    private fun putIcdSignMagnitude(
        str: ByteArray,
        icdMsbBit: Int,
        length: Int,
        raw: Long,
    ) {
        putIcd(str, icdMsbBit, 1, if (raw < 0) 1L else 0L)
        putIcd(str, icdMsbBit - 1, length - 1, abs(raw))
    }

    private fun assertScaled(
        expected: Double,
        actual: Double,
    ) = assertEquals(expected, actual, abs(expected) * 1e-12 + 1e-18)

    // Reference raw values: realistic GLONASS state (radius ≈ 25,475 km),
    // odd low bits to catch dropped LSBs, mixed signs for sign-magnitude.
    private val xnRaw = 20_480_001L // ×2^-11 km ≈ +10,000.0005 km
    private val ynRaw = -30_720_003L // ≈ −15,000.0015 km
    private val znRaw = 36_864_005L // ≈ +18,000.0024 km
    private val xnDotRaw = -524_295L // ×2^-20 km/s ≈ −0.50001 km/s
    private val ynDotRaw = 2_831_155L // ≈ +2.7 km/s
    private val znDotRaw = -1_992_301L // ≈ −1.9 km/s
    private val xnAccRaw = -3L
    private val ynAccRaw = 0L
    private val znAccRaw = 1L
    private val gammaRaw = -5L // ×2^-40
    private val tauRaw = -107_374L // ×2^-30 s; getSignMagnitude(−) → τn ≈ −1.0e-4 s
    private val tb = 36L // ×15 min Moscow time = 09:00 MSK

    private fun buildString1(): ByteArray =
        ByteArray(11).also { s ->
            putIcd(s, 84, 4, 1) // m = string 1
            putIcd(s, 78, 2, 1) // P1 (after the TWO unused bits at ICD 80-79)
            putIcd(s, 76, 5, 11) // tk hours
            putIcd(s, 71, 6, 15) // tk minutes
            putIcd(s, 65, 1, 1) // tk 30 s flag
            putIcdSignMagnitude(s, 64, 24, xnDotRaw)
            putIcdSignMagnitude(s, 40, 5, xnAccRaw)
            putIcdSignMagnitude(s, 35, 27, xnRaw)
        }

    private fun buildString2(bn: Long = 0L): ByteArray =
        ByteArray(11).also { s ->
            putIcd(s, 84, 4, 2) // m = string 2
            putIcd(s, 80, 3, bn) // Bn
            putIcd(s, 77, 1, 1) // P2
            putIcd(s, 76, 7, tb)
            putIcdSignMagnitude(s, 64, 24, ynDotRaw)
            putIcdSignMagnitude(s, 40, 5, ynAccRaw)
            putIcdSignMagnitude(s, 35, 27, ynRaw)
        }

    private fun buildString3(): ByteArray =
        ByteArray(11).also { s ->
            putIcd(s, 84, 4, 3) // m = string 3
            putIcd(s, 80, 1, 1) // P3
            putIcdSignMagnitude(s, 79, 11, gammaRaw)
            putIcd(s, 67, 2, 0) // p (2 bits, ICD 67-66)
            putIcd(s, 65, 1, 0) // ln healthy
            putIcdSignMagnitude(s, 64, 24, znDotRaw)
            putIcdSignMagnitude(s, 40, 5, znAccRaw)
            putIcdSignMagnitude(s, 35, 27, znRaw)
        }

    private fun buildString4(): ByteArray =
        ByteArray(11).also { s ->
            putIcd(s, 84, 4, 4) // m = string 4
            putIcdSignMagnitude(s, 80, 22, tauRaw)
        }

    private fun strings(bn: Long = 0L): Map<Int, ByteArray> = mapOf(1 to buildString1(), 2 to buildString2(bn), 3 to buildString3(), 4 to buildString4())

    @Test
    fun parseGlonassEphemeris_recoversIcdPlacedReferenceValues() {
        // GPS TOW shortly after tb on GPS day 2: tb = 09:00 MSK = 06:00 UTC = 21618 s GPS-of-day.
        val gpsTow = 2 * 86400.0 + 22_000.0
        val eph = GlonassNavMessageParser.parseGlonassEphemeris(svid = 5, strings = strings(), gpsTimeOfWeekSeconds = gpsTow, leapSeconds = 18)

        assertNotNull(eph)
        eph!!
        val posScale = 2.0.pow(-11) * 1e3
        val velScale = 2.0.pow(-20) * 1e3
        val accScale = 2.0.pow(-30) * 1e3

        assertScaled(xnRaw * posScale, eph.gloPosX)
        assertScaled(ynRaw * posScale, eph.gloPosY)
        assertScaled(znRaw * posScale, eph.gloPosZ)
        assertScaled(xnDotRaw * velScale, eph.gloVelX)
        assertScaled(ynDotRaw * velScale, eph.gloVelY)
        assertScaled(znDotRaw * velScale, eph.gloVelZ)
        assertScaled(xnAccRaw * accScale, eph.gloAccX)
        assertScaled(ynAccRaw * accScale, eph.gloAccY)
        assertScaled(znAccRaw * accScale, eph.gloAccZ)
        assertScaled(gammaRaw * 2.0.pow(-40), eph.gloGammaN)
        assertScaled(tauRaw * 2.0.pow(-30), eph.gloTauN)
        // Clock convention: af0 = −τn, af1 = +γn.
        assertScaled(-(tauRaw * 2.0.pow(-30)), eph.af0)
        assertScaled(gammaRaw * 2.0.pow(-40), eph.af1)
        // tb = 09:00 MSK → 06:00 UTC → +18 leap s → GPS seconds-of-day 21618 on GPS day 2.
        assertEquals(2 * 86400.0 + 21_618.0, eph.gloTbGpsTow, 1e-9)
    }

    @Test
    fun parseGlonassEphemeris_orbitRadiusSanity() {
        // A one- or two-bit layout shift destroys the position magnitude; the
        // physical radius is the cheap invariant that catches it.
        val eph = GlonassNavMessageParser.parseGlonassEphemeris(5, strings(), 2 * 86400.0 + 22_000.0, 18)!!
        val rKm = Math.sqrt(eph.gloPosX * eph.gloPosX + eph.gloPosY * eph.gloPosY + eph.gloPosZ * eph.gloPosZ) / 1e3
        assertEquals(25_475.0, rKm, 5.0)
    }

    @Test
    fun parseGlonassEphemeris_unhealthySatellite_returnsNull() {
        // Bn MSB set = malfunction → must be rejected.
        assertNull(GlonassNavMessageParser.parseGlonassEphemeris(5, strings(bn = 0b100), 2 * 86400.0 + 22_000.0, 18))
    }

    @Test
    fun parseGlonassEphemeris_zeroStrings_rejectedByRadiusGate() {
        val zero = mapOf(1 to ByteArray(11), 2 to ByteArray(11), 3 to ByteArray(11), 4 to ByteArray(11))
        assertNull(GlonassNavMessageParser.parseGlonassEphemeris(5, zero, 0.0, 18))
    }

    @Test
    fun parseGlonassEphemeris_missingString_returnsNull() {
        val incomplete = mapOf(1 to buildString1(), 2 to buildString2(), 3 to buildString3())
        assertNull(GlonassNavMessageParser.parseGlonassEphemeris(5, incomplete, 0.0, 18))
    }
}
