package app.simt.gnss.core.parser
import app.simt.gnss.core.model.Constellation
import app.simt.gnss.core.model.SatelliteEphemeris
import app.simt.gnss.core.platform.GnssPlatform
import app.simt.gnss.core.utils.GnssMathUtils.getBits
import app.simt.gnss.core.utils.GnssMathUtils.getSignedBits

/**
 * Parser for SBAS (Satellite-Based Augmentation System) L1 C/A navigation messages.
 *
 * SBAS satellites (WAAS, EGNOS, MSAS, GAGAN, SDCM, etc.) broadcast 250-bit messages at 250 bps
 * on the GPS L1 frequency (1575.42 MHz). Each message contains:
 *  - Preamble (8 bits)
 *  - Message type (6 bits)
 *  - 212 data bits (content varies by message type)
 *  - CRC-24Q (24 bits)
 *
 * This parser handles:
 *  - **MT 1**: PRN mask — maps mask slots to satellite PRNs (with IODP).
 *  - **MT 9**: GEO Navigation Message — ECEF position/velocity/acceleration for SBAS ranging.
 *  - **MT 2–5, 24**: Fast Corrections — clock corrections for GPS satellites.
 *  - **MT 25**: Long-term Corrections — orbit + clock corrections for GPS satellites.
 *
 * Issue-of-data discipline (DO-229 §2.1.1.4): fast and long-term corrections reference
 * satellites by their **slot in the MT1 mask**, so they may only be applied through a mask
 * with the matching IODP, and long-term corrections additionally carry the IODE of the
 * broadcast ephemeris they were computed against.
 *
 * Reference: RTCA DO-229E / ICAO Annex 10 Vol I, SBAS L1 message definitions.
 */
object SbasNavMessageParser {
    /**
     * Extracts the 6-bit message type from an SBAS 250-bit frame.
     * Bits 8–13 (after the 8-bit preamble).
     *
     * @param data Raw message bytes (at least 32 bytes for 250 bits)
     * @return Message type (0–63)
     */
    fun getMessageType(data: ByteArray): Int = getBits(data, 8, 6)

    /**
     * Parses an SBAS MT 9 (GEO Navigation Message) into a [SatelliteEphemeris].
     *
     * MT 9 provides the SBAS satellite's own ECEF position, velocity, and acceleration
     * at a reference time t0, plus clock bias (aGf0) and drift (aGf1). The GEO position
     * at any time t is computed by second-order Taylor expansion:
     *   X(t) = X0 + Vx·(t−t0) + 0.5·Ax·(t−t0)²
     *
     * Bit layout (250-bit frame, after 8-bit preamble + 6-bit MT):
     *   spare    : bits 14–21  (8 bits, reserved)
     *   t0       : bits 22–34  (13 bits, unsigned, LSB = 16 s)
     *   URA      : bits 35–38  (4 bits)
     *   X_G      : bits 39–68  (30 bits, signed, LSB = 0.08 m)
     *   Y_G      : bits 69–98  (30 bits, signed, LSB = 0.08 m)
     *   Z_G      : bits 99–123 (25 bits, signed, LSB = 0.4 m)
     *   X_G_dot  : bits 124–140 (17 bits, signed, LSB = 0.000625 m/s)
     *   Y_G_dot  : bits 141–157 (17 bits, signed, LSB = 0.000625 m/s)
     *   Z_G_dot  : bits 158–175 (18 bits, signed, LSB = 0.004 m/s)
     *   X_G_ddot : bits 176–185 (10 bits, signed, LSB = 0.0000125 m/s²)
     *   Y_G_ddot : bits 186–195 (10 bits, signed, LSB = 0.0000125 m/s²)
     *   Z_G_ddot : bits 196–205 (10 bits, signed, LSB = 0.0000625 m/s²)
     *   aGf0     : bits 206–217 (12 bits, signed, LSB = 2⁻³¹ s)
     *   aGf1     : bits 218–225 (8 bits, signed, LSB = 2⁻⁴⁰ s/s)
     *
     * Reference: RTCA DO-229E Table A-7, verified against RTKLIB decode_sbstype9().
     *
     * @param svid PRN of the SBAS satellite (120–158)
     * @param data Raw 250-bit message bytes
     * @return Parsed ephemeris, or null on error
     */
    fun parseMt9GeoNavigation(
        svid: Int,
        data: ByteArray,
    ): SatelliteEphemeris? {
        return try {
            // URA index 15 = "do not use for ranging" (DO-229 §A.4.4.11).
            if (getBits(data, 35, 4) == 15) return null

            // t0 in seconds of the day, LSB = 16 s (8 spare bits at 14–21)
            val t0Raw = getBits(data, 22, 13)
            val t0 = t0Raw * 16.0

            // Position (ECEF, meters)
            val xg = getSignedBits(data, 39, 30) * 0.08
            val yg = getSignedBits(data, 69, 30) * 0.08
            val zg = getSignedBits(data, 99, 25) * 0.4

            // Velocity (ECEF, m/s)
            val xgDot = getSignedBits(data, 124, 17) * 0.000625
            val ygDot = getSignedBits(data, 141, 17) * 0.000625
            val zgDot = getSignedBits(data, 158, 18) * 0.004

            // Acceleration (ECEF, m/s²)
            val xgDdot = getSignedBits(data, 176, 10) * 0.0000125
            val ygDdot = getSignedBits(data, 186, 10) * 0.0000125
            val zgDdot = getSignedBits(data, 196, 10) * 0.0000625

            // Clock correction
            val agf0 = getSignedBits(data, 206, 12) * Math.pow(2.0, -31.0)
            val agf1 = getSignedBits(data, 218, 8) * Math.pow(2.0, -40.0)

            SatelliteEphemeris(
                constellationType = Constellation.SBAS,
                svid = svid,
                isSbas = true,
                sbasT0 = t0,
                sbasPosX = xg,
                sbasPosY = yg,
                sbasPosZ = zg,
                sbasVelX = xgDot,
                sbasVelY = ygDot,
                sbasVelZ = zgDot,
                sbasAccX = xgDdot,
                sbasAccY = ygDdot,
                sbasAccZ = zgDdot,
                sbasAgf0 = agf0,
                sbasAgf1 = agf1,
                af0 = agf0,
                af1 = agf1,
                lastUpdateTime = GnssPlatform.timeSource.elapsedRealtimeMs(),
            )
        } catch (e: Exception) {
            null
        }
    }

    /**
     * Result of parsing SBAS fast corrections (MT 2–5, 24).
     * Each entry maps a satellite PRN mask index to a pseudorange correction (PRC) in meters
     * and a range rate correction (RRC) in m/s.
     */
    data class SbasFastCorrection(
        val prnMaskIndex: Int,
        val prc: Double,
        val udrei: Int,
        /** IODP of the PRN mask these corrections index into; apply only through a matching mask. */
        val iodp: Int,
        val receivedElapsedMs: Long = GnssPlatform.timeSource.elapsedRealtimeMs(),
    )

    /**
     * Parses SBAS MT 2–5 fast correction messages.
     *
     * Each MT 2–5 message carries fast corrections for 13 satellites identified by their
     * PRN mask index (set by MT 1). The corrections include:
     *  - IODF (Issue of Data, Fast Corrections): 2 bits at bits 14–15
     *  - IODP (Issue of Data, PRN mask): 2 bits at bits 16–17
     *  - 13 × PRC (Pseudorange Correction): 12 bits signed, LSB = 0.125 m, starting at bit 18
     *  - 13 × UDREI: 4 bits unsigned, starting at bit 174
     *
     * Reference: RTCA DO-229E Table A-1, verified against RTKLIB decode_sbstype2().
     *
     * @param data Raw 250-bit message bytes
     * @param messageType Message type (2, 3, 4, or 5) — determines which block of 13 PRN mask indices
     * @return List of fast corrections, or null on error
     */
    fun parseFastCorrections(
        data: ByteArray,
        messageType: Int,
    ): List<SbasFastCorrection>? {
        if (messageType !in 2..5) return null
        return try {
            val corrections = mutableListOf<SbasFastCorrection>()
            val baseIndex = (messageType - 2) * 13
            // IODF at bits 14–15, IODP at bits 16–17
            // 13 PRCs start at bit 18, each 12 bits (stride 12)
            // 13 UDREIs start at bit 174 (= 18 + 13×12), each 4 bits (stride 4)
            val iodp = getBits(data, 16, 2)
            for (i in 0 until 13) {
                val prc = getSignedBits(data, 18 + i * 12, 12) * 0.125
                val udrei = getBits(data, 174 + i * 4, 4)
                corrections.add(
                    SbasFastCorrection(
                        prnMaskIndex = baseIndex + i,
                        prc = prc,
                        udrei = udrei,
                        iodp = iodp,
                    ),
                )
            }
            corrections
        } catch (e: Exception) {
            null
        }
    }

    /**
     * Result of parsing SBAS MT 24 mixed fast/long-term corrections.
     */
    data class SbasMt24Result(
        val fastCorrections: List<SbasFastCorrection>,
        val longTermCorrections: List<SbasLongTermCorrection>,
    )

    /**
     * Parses SBAS MT 24 (mixed fast/long-term satellite error corrections).
     *
     * MT 24 combines 6 fast corrections with one long-term correction half-message:
     *   6 × PRC     : bits 14–85  (12 bits signed each, LSB = 0.125 m)
     *   6 × UDREI   : bits 86–109 (4 bits unsigned each)
     *   IODP        : bits 110–111 (2 bits)
     *   Block ID    : bits 112–113 (2 bits) — identifies which block of 13 satellites
     *   IODF        : bits 114–115 (2 bits)
     *   Spare       : bits 116–119 (4 bits)
     *   Long-term correction half-message: bits 120–224 (same as one half of MT 25)
     *
     * The 6 fast corrections apply to PRN mask indices 13×block + 0..5.
     *
     * Reference: RTCA DO-229E Table A-4, verified against RTKLIB decode_sbstype24().
     *
     * @param data Raw 250-bit message bytes
     * @return Parsed fast + long-term corrections, or null on error
     */
    fun parseMt24MixedCorrections(data: ByteArray): SbasMt24Result? {
        return try {
            val block = getBits(data, 112, 2)
            val fastIodp = getBits(data, 110, 2)
            val fastCorrections = mutableListOf<SbasFastCorrection>()

            // 6 fast corrections: PRCs at bits 14+i*12, UDREIs at bits 86+i*4
            val baseIndex = 13 * block
            for (i in 0 until 6) {
                val prc = getSignedBits(data, 14 + i * 12, 12) * 0.125
                val udrei = getBits(data, 86 + i * 4, 4)
                fastCorrections.add(
                    SbasFastCorrection(
                        prnMaskIndex = baseIndex + i,
                        prc = prc,
                        udrei = udrei,
                        iodp = fastIodp,
                    ),
                )
            }

            // Long-term correction half-message at bit 120 (same 106-bit format as one MT 25
            // section: velocity code + two 51-bit halves + IODP).
            val longTermCorrections = parseLongTermSection(data, sectionStart = 120)

            SbasMt24Result(
                fastCorrections = fastCorrections,
                longTermCorrections = longTermCorrections,
            )
        } catch (e: Exception) {
            null
        }
    }

    /**
     * Result of parsing SBAS long-term corrections (MT 25).
     *
     * [prnMaskNumber] is the 1-based slot in the MT1 PRN mask — NOT a PRN. The consumer must
     * translate it through a mask whose IODP equals [iodp], and apply the correction only to
     * the broadcast ephemeris whose IODE equals [iode] (DO-229 §2.1.1.4.9).
     */
    data class SbasLongTermCorrection(
        val prnMaskNumber: Int,
        val iode: Int,
        val iodp: Int,
        val deltaX: Double,
        val deltaY: Double,
        val deltaZ: Double,
        val deltaAf0: Double,
        val receivedElapsedMs: Long = GnssPlatform.timeSource.elapsedRealtimeMs(),
    )

    /**
     * Parses SBAS MT 25 long-term satellite error corrections (velocity code = 0 variant).
     *
     * MT 25 contains two independent "half-message" sections at bits 14–118 and 120–224.
     * Each section starts with a velocity code bit. For velocity code = 0 (no velocity):
     *   Each section = vel_code(1) + 2 × half(51 bits) + IODP(2)
     *
     * Each half (51 bits, velocity code = 0):
     *   PRN mask #  : 6 bits
     *   IODE        : 8 bits (Issue of Data, Ephemeris)
     *   delta-X     : 9 bits signed, LSB = 0.125 m
     *   delta-Y     : 9 bits signed, LSB = 0.125 m
     *   delta-Z     : 9 bits signed, LSB = 0.125 m
     *   delta-af0   : 10 bits signed, LSB = 2⁻³¹ s
     *
     * Reference: RTCA DO-229E Table A-5, verified against RTKLIB decode_longcorr0/decode_sbstype25.
     *
     * @param data Raw 250-bit message bytes
     * @return List of long-term corrections, or null on error
     */
    fun parseLongTermCorrections(data: ByteArray): List<SbasLongTermCorrection>? {
        return try {
            // Two independent 106-bit half-message sections at bits 14 and 120.
            parseLongTermSection(data, 14) + parseLongTermSection(data, 120)
        } catch (e: Exception) {
            null
        }
    }

    /**
     * Parses one 106-bit long-term section (velocity code + two 51-bit halves + IODP).
     * Velocity code = 1 (single correction with velocity terms) is not implemented; such a
     * section contributes nothing rather than mis-parsed values.
     */
    private fun parseLongTermSection(
        data: ByteArray,
        sectionStart: Int,
    ): List<SbasLongTermCorrection> {
        if (getBits(data, sectionStart, 1) != 0) return emptyList()
        val iodp = getBits(data, sectionStart + 103, 2)
        val corrections = mutableListOf<SbasLongTermCorrection>()
        for (block in 0 until 2) {
            val base = sectionStart + 1 + block * 51
            val prnMaskNumber = getBits(data, base, 6)
            if (prnMaskNumber == 0) continue // unused slot
            val iode = getBits(data, base + 6, 8)
            val dx = getSignedBits(data, base + 14, 9) * 0.125
            val dy = getSignedBits(data, base + 23, 9) * 0.125
            val dz = getSignedBits(data, base + 32, 9) * 0.125
            val daf0 = getSignedBits(data, base + 41, 10) * Math.pow(2.0, -31.0)
            corrections.add(
                SbasLongTermCorrection(
                    prnMaskNumber = prnMaskNumber,
                    iode = iode,
                    iodp = iodp,
                    deltaX = dx,
                    deltaY = dy,
                    deltaZ = dz,
                    deltaAf0 = daf0,
                ),
            )
        }
        return corrections
    }

    /** Parsed SBAS MT 1 PRN mask: mask slot (0-based) → PRN, qualified by its IODP. */
    data class SbasPrnMask(
        val iodp: Int,
        val maskToPrn: Map<Int, Int>,
    )

    /**
     * Parses SBAS MT 1 (PRN Mask) which maps SBAS "mask index" slots to actual satellite PRNs.
     *
     * Layout (after preamble + MT = bit 14):
     *   PRN mask : 210 bits (bits 14–223), one bit per PRN (1–210)
     *   IODP     : bits 224–225 (2 bits)
     *
     * PRN assignments:
     *   1–37   = GPS PRN 1–37
     *   38–61  = GLONASS slot 1–24
     *   62–119 = Reserved
     *   120–138 = SBAS GEO PRN 120–138
     *   remaining = future use
     *
     * @param data Raw 250-bit message bytes
     * @return The IODP-qualified mask, or null on error
     */
    fun parsePrnMask(data: ByteArray): SbasPrnMask? {
        return try {
            val maskToPrn = mutableMapOf<Int, Int>()
            var maskIndex = 0
            for (prn in 1..210) {
                if (getBits(data, 13 + prn, 1) == 1) {
                    maskToPrn[maskIndex] = prn
                    maskIndex++
                }
            }
            SbasPrnMask(iodp = getBits(data, 224, 2), maskToPrn = maskToPrn)
        } catch (e: Exception) {
            null
        }
    }
}
