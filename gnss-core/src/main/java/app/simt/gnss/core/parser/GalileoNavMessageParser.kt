package app.simt.gnss.core.parser
import app.simt.gnss.core.model.Constellation
import app.simt.gnss.core.model.SatelliteEphemeris
import app.simt.gnss.core.platform.GnssPlatform
import app.simt.gnss.core.utils.GnssMathUtils.getBits
import app.simt.gnss.core.utils.GnssMathUtils.getBitsLong
import app.simt.gnss.core.utils.GnssMathUtils.getSignedBits
import app.simt.gnss.core.utils.GnssMathUtils.getSignedBitsLong
import kotlin.math.PI
import kotlin.math.pow

/**
 * Parser for Galileo I/NAV Navigation Messages (E1-B / E5b-I).
 *
 * Decodes the four ephemeris words (Word Types 1–4) plus Word Type 5 (iono, BGDs, health,
 * reference week) to produce a [SatelliteEphemeris]. Word Type 6 (UTC / GPS-Galileo time offset)
 * is not required for ranging and is ignored here.
 *
 * **Data layout** (Galileo OS SIS ICD Issue 2.1, §4.3.5; Android `GnssNavigationMessage`
 * TYPE_GAL_I): Android delivers the nominal page as **two page parts** — even then odd,
 * 2 × 114 bits packed MSB-first into 29 bytes. The even part carries `even/odd(1) +
 * pageType(1) + Data(1/2) = word bits 1–112` at flat bits 0–113; the odd part carries
 * `even/odd(1) + pageType(1) + Data(2/2) = word bits 113–128` at flat bits 114–131 (then
 * OSNMA/SAR/CRC). [extractInavWord] splices Data(1/2) ∥ Data(2/2) into the contiguous
 * 128-bit I/NAV word that all field offsets below are relative to. Reading the delivered
 * buffer as if it were contiguous would absorb the odd part's two header bits into every
 * field that extends past word bit 112 (√A, IDOT, Crs, af1, af2) and drop their LSBs.
 */
object GalileoNavMessageParser {
    /**
     * Returns the contiguous 128-bit I/NAV word (16 bytes) for an Android TYPE_GAL_I payload,
     * or null if the buffer is malformed or is an alert page (page type 1 — no nav data).
     *
     * The GnssNavigationMessage contract delivers the nominal page as two page parts (even
     * then odd, 2 × 114 bits in 29 bytes); the even/odd framing flags are required. There is
     * deliberately no "contiguous buffer" fallback: a buffer without the documented framing
     * has unknown layout, and guessing produced fields shifted by the odd-part header bits.
     */
    internal fun extractInavWord(data: ByteArray): ByteArray? {
        if (data.size < 29) return null
        val evenOddFirst = getBits(data, 0, 1)
        val evenOddSecond = getBits(data, 114, 1)
        if (evenOddFirst != 0 || evenOddSecond != 1) return null // malformed page pair
        if (getBits(data, 1, 1) != 0 || getBits(data, 115, 1) != 0) return null // alert page
        val word = ByteArray(16)
        copyBits(data, 2, word, 0, 112)
        copyBits(data, 116, word, 112, 16)
        return word
    }

    private fun copyBits(
        src: ByteArray,
        srcStart: Int,
        dst: ByteArray,
        dstStart: Int,
        count: Int,
    ) {
        for (i in 0 until count) {
            val s = srcStart + i
            val bit = (src[s / 8].toInt() shr (7 - (s % 8))) and 1
            if (bit != 0) {
                val d = dstStart + i
                dst[d / 8] = (dst[d / 8].toInt() or (1 shl (7 - (d % 8)))).toByte()
            }
        }
    }

    fun parseGalileoEphemeris(
        svid: Int,
        words: Map<Int, ByteArray>,
    ): SatelliteEphemeris? {
        return try {
            // Splice each page pair into its contiguous 128-bit word; offsets below are
            // word-relative (word type at bits 0-5, first content bit at 6).
            val w1 = words[1]?.let(::extractInavWord) ?: return null
            val w2 = words[2]?.let(::extractInavWord) ?: return null
            val w3 = words[3]?.let(::extractInavWord) ?: return null
            val w4 = words[4]?.let(::extractInavWord) ?: return null
            val w5 = words[5]?.let(::extractInavWord) ?: return null

            // --- Word Type 1: IODnav + t0e + M0 + e + √A (Ephemeris 1/4) ------------------------
            val iodNavW1 = getBits(w1, 6, 10)
            val toe = getBits(w1, 16, 14) * 60.0
            val m0 = getSignedBitsLong(w1, 30, 32) * 2.0.pow(-31) * PI
            val e = getBitsLong(w1, 62, 32) * 2.0.pow(-33)
            val sqrtA = getBitsLong(w1, 94, 32) * 2.0.pow(-19)

            // --- Word Type 2: IODnav + Ω₀ + i₀ + ω + IDOT (Ephemeris 2/4) -----------------------
            val iodNavW2 = getBits(w2, 6, 10)
            val omega0 = getSignedBitsLong(w2, 16, 32) * 2.0.pow(-31) * PI
            val i0 = getSignedBitsLong(w2, 48, 32) * 2.0.pow(-31) * PI
            val omega = getSignedBitsLong(w2, 80, 32) * 2.0.pow(-31) * PI
            val iDot = getSignedBits(w2, 112, 14) * 2.0.pow(-43) * PI

            // --- Word Type 3: IODnav + Ω̇ + Δn + Cuc + Cus + Crc + Crs + SISA (Ephemeris 3/4) ---
            val iodNavW3 = getBits(w3, 6, 10)
            val omegaDot = getSignedBits(w3, 16, 24) * 2.0.pow(-43) * PI
            val deltaN = getSignedBits(w3, 40, 16) * 2.0.pow(-43) * PI
            val cuc = getSignedBits(w3, 56, 16) * 2.0.pow(-29)
            val cus = getSignedBits(w3, 72, 16) * 2.0.pow(-29)
            val crc = getSignedBits(w3, 88, 16) * 2.0.pow(-5)
            val crs = getSignedBits(w3, 104, 16) * 2.0.pow(-5)

            // --- Word Type 4: IODnav + SVID + Cic + Cis + t0c + af0 + af1 + af2 (Ephem. 4/4) ----
            val iodNavW4 = getBits(w4, 6, 10)
            val cic = getSignedBits(w4, 22, 16) * 2.0.pow(-29)
            val cis = getSignedBits(w4, 38, 16) * 2.0.pow(-29)
            val toc = getBits(w4, 54, 14) * 60.0
            val af0 = getSignedBitsLong(w4, 68, 31) * 2.0.pow(-34)
            val af1 = getSignedBits(w4, 99, 21) * 2.0.pow(-46)
            val af2 = getSignedBits(w4, 120, 6) * 2.0.pow(-59)

            // --- Word Type 5: BGDs, signal health, Galileo week number --------------------------
            // The I/NAV clock (af0/af1/af2 from Word 4) is referenced to the E1/E5b pair
            // (OS SIS ICD §5.1.5). E1 uses BGD(E1,E5b) at word bits 58-67. E5a and the
            // E1/E5a ionosphere-free combination also need BGD(E1,E5a) at bits 48-57.
            val bgdE1E5a = getSignedBits(w5, 47, 10) * 2.0.pow(-32)
            val bgdE1E5b = getSignedBits(w5, 57, 10) * 2.0.pow(-32)
            val week = getBits(w5, 73, 12)

            // Signal health (word bits 69-70 E1B-HS, 73 E1B-DVS, 1-indexed 70-71/74): any
            // non-OK status means the E1 signal must not be used for ranging.
            val e1bHealthStatus = getBits(w5, 69, 2)
            val e1bDataValidity = getBits(w5, 72, 1)
            if (e1bHealthStatus != 0 || e1bDataValidity != 0) return null

            // IODnav must match across Words 1–4 for a consistent ephemeris set
            // (Galileo OS SIS ICD §4.3.5 Note 1). Pages collected across boundary issues would
            // otherwise stitch fields from different ephemerides.
            if (iodNavW1 != iodNavW2 || iodNavW1 != iodNavW3 || iodNavW1 != iodNavW4) {
                return null
            }

            SatelliteEphemeris(
                constellationType = Constellation.GALILEO,
                svid = svid,
                week = week,
                iode = iodNavW1,
                toe = toe,
                toc = toc,
                af0 = af0,
                af1 = af1,
                af2 = af2,
                m0 = m0,
                deltaN = deltaN,
                eccentricity = e,
                sqrtA = sqrtA,
                omega0 = omega0,
                i0 = i0,
                omega = omega,
                omegaDot = omegaDot,
                iDot = iDot,
                cuc = cuc,
                cus = cus,
                crc = crc,
                crs = crs,
                cic = cic,
                cis = cis,
                tgd = bgdE1E5b,
                galileoBgdE1E5aSeconds = bgdE1E5a,
                galileoBgdE1E5bSeconds = bgdE1E5b,
                galileoInavClock = true,
                lastUpdateTime = GnssPlatform.timeSource.elapsedRealtimeMs(),
            )
        } catch (e: Exception) {
            null
        }
    }
}
