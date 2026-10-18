package app.simt.gnss.core.parser
import app.simt.gnss.core.model.Constellation
import app.simt.gnss.core.model.SatelliteEphemeris
import app.simt.gnss.core.platform.GnssPlatform
import app.simt.gnss.core.utils.GnssMathUtils.getBits
import app.simt.gnss.core.utils.GnssMathUtils.getBitsLong
import app.simt.gnss.core.utils.GnssMathUtils.packBits
import kotlin.math.PI
import kotlin.math.pow

/**
 * Parser for BeiDou D1 navigation messages (B1I MEO/IGSO), subframes 1–3.
 *
 * **Transport format** (BDS-SIS-ICD-B1I §5.1.3): each 30-bit word carries 22 information
 * bits protected by two BCH(15,11,1) codes. Word 1 transmits its first 15 bits (Pre + Rev)
 * directly followed by one BCH(15,11) block; words 2–10 split their 22 information bits
 * into two 11-bit groups, BCH-encode each to 15 bits, and **interleave the two codewords
 * bit-by-bit**. [prepareSubframe] undoes the interleaving, verifies every BCH parity, and
 * produces the canonical decode layout used by the field map below: per word, 22
 * information bits followed by 8 parity bits (word 1: 26 + 4) — the same buffer convention
 * as RTKLIB's `decode_bds_d1`.
 *
 * Because every multi-bit parameter is split across word boundaries by this structure, all
 * wide fields below are read as two segments and recombined.
 */
object BeidouNavMessageParser {
    /** BCH(15,11,1) generator polynomial g(x) = x⁴ + x + 1. */
    private const val BCH_GENERATOR = 0b10011

    private fun bchRemainder(codeword15: Int): Int {
        var r = codeword15
        for (i in 14 downTo 4) {
            if ((r shr i) and 1 == 1) r = r xor (BCH_GENERATOR shl (i - 4))
        }
        return r and 0xF
    }

    /**
     * De-interleaves one transmitted word (words 2–10): transmitted bits alternate between
     * the two BCH codewords (t[0]=cw1[0], t[1]=cw2[0], …). Returns the decode-layout word
     * `info1(11) | info2(11) | parity1(4) | parity2(4)`, or null if either BCH check fails.
     */
    private fun deinterleaveWord(word: Int): Int? {
        var cw1 = 0
        var cw2 = 0
        for (k in 0 until 15) {
            cw1 = (cw1 shl 1) or ((word shr (29 - 2 * k)) and 1)
            cw2 = (cw2 shl 1) or ((word shr (28 - 2 * k)) and 1)
        }
        if (bchRemainder(cw1) != 0 || bchRemainder(cw2) != 0) return null
        return ((cw1 ushr 4) shl 19) or ((cw2 ushr 4) shl 8) or ((cw1 and 0xF) shl 4) or (cw2 and 0xF)
    }

    /** BCH check of a word already in decode layout (HALs that deliver de-interleaved words). */
    private fun isValidDecodedWord(word: Int): Boolean {
        val cw1 = ((word ushr 19) shl 4) or ((word ushr 4) and 0xF)
        val cw2 = (((word ushr 8) and 0x7FF) shl 4) or (word and 0xF)
        return bchRemainder(cw1) == 0 && bchRemainder(cw2) == 0
    }

    /**
     * Validates and normalises one 38-byte subframe (10 × 30-bit words, MSB-first) into the
     * decode layout. Tries the as-broadcast interleaved interpretation first; if its BCH
     * checks fail, falls back to treating the words as already de-interleaved (some HALs
     * deliver decoded words). Returns null when neither interpretation passes — corrupted
     * data must not produce an ephemeris.
     */
    private fun prepareSubframe(raw: ByteArray): ByteArray? {
        if (raw.size < 38) return null
        val words = IntArray(10) { getBits(raw, it * 30, 30) }
        // Word 1: bits 0-14 direct, bits 15-29 one non-interleaved BCH block.
        if (bchRemainder(words[0] and 0x7FFF) != 0) return null

        val out = IntArray(10)
        out[0] = words[0]
        var interleavedOk = true
        for (w in 1 until 10) {
            val decoded = deinterleaveWord(words[w])
            if (decoded == null) {
                interleavedOk = false
                break
            }
            out[w] = decoded
        }
        if (!interleavedOk) {
            for (w in 1 until 10) {
                if (!isValidDecodedWord(words[w])) return null
                out[w] = words[w]
            }
        }

        val buf = ByteArray(38)
        for (w in 0 until 10) packBits(buf, w * 30, 30, out[w])
        return buf
    }

    /** Reads a field split across two segments of the decode buffer (MSB segment first). */
    private fun u2(
        sf: ByteArray,
        start1: Int,
        len1: Int,
        start2: Int,
        len2: Int,
    ): Long = (getBitsLong(sf, start1, len1) shl len2) or getBitsLong(sf, start2, len2)

    private fun s2(
        sf: ByteArray,
        start1: Int,
        len1: Int,
        start2: Int,
        len2: Int,
    ): Long {
        val value = u2(sf, start1, len1, start2, len2)
        val total = len1 + len2
        return if ((value shr (total - 1)) and 1L == 1L) value - (1L shl total) else value
    }

    private fun s1(
        sf: ByteArray,
        start: Int,
        len: Int,
    ): Long {
        val value = getBitsLong(sf, start, len)
        return if ((value shr (len - 1)) and 1L == 1L) value - (1L shl len) else value
    }

    /**
     * Decodes raw D1 subframes 1–3 into a [SatelliteEphemeris].
     *
     * Returns null when: BCH validation fails, the FraIDs are not 1/2/3, the three SOWs are
     * not consecutive (+6 s — which guarantees all three subframes belong to the same frame
     * and therefore the same data set), the satellite is flagged unhealthy (SatH1), or the
     * decoded orbit is non-physical.
     */
    fun parseBeidouEphemeris(
        svid: Int,
        sf1Raw: ByteArray,
        sf2Raw: ByteArray,
        sf3Raw: ByteArray,
    ): SatelliteEphemeris? {
        return try {
            val sf1 = prepareSubframe(sf1Raw) ?: return null
            val sf2 = prepareSubframe(sf2Raw) ?: return null
            val sf3 = prepareSubframe(sf3Raw) ?: return null

            if (getBits(sf1, 15, 3) != 1 || getBits(sf2, 15, 3) != 2 || getBits(sf3, 15, 3) != 3) return null

            // SOW continuity: subframes 2 and 3 follow subframe 1 by exactly 6 s within one
            // frame. Subframes collected from different frames (stale storage) are rejected,
            // which also enforces a consistent data set (same AODC/AODE).
            val sow1 = u2(sf1, 18, 8, 30, 12)
            val sow2 = u2(sf2, 18, 8, 30, 12)
            val sow3 = u2(sf3, 18, 8, 30, 12)
            if (sow2 != sow1 + 6 || sow3 != sow2 + 6) return null

            // --- Subframe 1: health, clock, group delay (BDS-SIS-ICD-B1I §5.2.4) ---
            val satH1 = getBits(sf1, 42, 1)
            if (satH1 != 0) return null // unhealthy / in maintenance
            val week = getBits(sf1, 60, 13)
            val toc = u2(sf1, 73, 9, 90, 8) * 8.0
            val tgd1 = s1(sf1, 98, 10) * 0.1 * 1e-9
            val af2 = s1(sf1, 214, 11) * 2.0.pow(-66)
            val af0 = s2(sf1, 225, 7, 240, 17) * 2.0.pow(-33)
            val af1 = s2(sf1, 257, 5, 270, 17) * 2.0.pow(-50)

            // --- Subframe 2: ΔN, Cuc, M0, e, Cus, Crc, Crs, √A, toe MSBs ---
            val deltaN = s2(sf2, 42, 10, 60, 6) * 2.0.pow(-43) * PI
            val cuc = s2(sf2, 66, 16, 90, 2) * 2.0.pow(-31)
            val m0 = s2(sf2, 92, 20, 120, 12) * 2.0.pow(-31) * PI
            val e = u2(sf2, 132, 10, 150, 22) * 2.0.pow(-33)
            val cus = s1(sf2, 180, 18) * 2.0.pow(-31)
            val crc = s2(sf2, 198, 4, 210, 14) * 2.0.pow(-6)
            val crs = s2(sf2, 224, 8, 240, 10) * 2.0.pow(-6)
            val sqrtA = u2(sf2, 250, 12, 270, 20) * 2.0.pow(-19)
            val toeMsb = getBitsLong(sf2, 290, 2)

            // --- Subframe 3: toe LSBs, i0, Cic, Ω̇, Cis, IDOT, Ω0, ω ---
            val toeLsb = u2(sf3, 42, 10, 60, 5)
            val toe = ((toeMsb shl 15) or toeLsb) * 8.0
            val i0 = s2(sf3, 65, 17, 90, 15) * 2.0.pow(-31) * PI
            val cic = s2(sf3, 105, 7, 120, 11) * 2.0.pow(-31)
            val omegaDot = s2(sf3, 131, 11, 150, 13) * 2.0.pow(-43) * PI
            val cis = s2(sf3, 163, 9, 180, 9) * 2.0.pow(-31)
            val iDot = s2(sf3, 189, 13, 210, 1) * 2.0.pow(-43) * PI
            val omega0 = s2(sf3, 211, 21, 240, 11) * 2.0.pow(-31) * PI
            val omega = s2(sf3, 251, 11, 270, 21) * 2.0.pow(-31) * PI

            // Physical plausibility: BDS MEO √A ≈ 5282.6, GEO/IGSO ≈ 6493.4 m^½.
            if (sqrtA < 5000.0 || sqrtA > 6600.0) return null
            if (e < 0.0 || e > 0.05) return null
            if (toc >= 604800.0 || toe >= 604800.0) return null

            SatelliteEphemeris(
                constellationType = Constellation.BEIDOU,
                svid = svid,
                week = week,
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
                tgd = tgd1,
                lastUpdateTime = GnssPlatform.timeSource.elapsedRealtimeMs(),
            )
        } catch (e: Exception) {
            null
        }
    }
}
