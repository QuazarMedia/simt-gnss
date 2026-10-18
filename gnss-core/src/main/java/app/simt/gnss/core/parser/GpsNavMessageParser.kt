package app.simt.gnss.core.parser
import app.simt.gnss.core.model.Constellation
import app.simt.gnss.core.model.SatelliteEphemeris
import app.simt.gnss.core.platform.GnssPlatform
import app.simt.gnss.core.utils.GnssMathUtils.getBits
import app.simt.gnss.core.utils.GnssMathUtils.getBitsLong
import app.simt.gnss.core.utils.GnssMathUtils.getSignedBits
import kotlin.math.PI
import kotlin.math.pow

/**
 * Parser for GPS Navigation Messages (Subframes 1, 2, 3).
 */
object GpsNavMessageParser {
    /**
     * Reads a 32-bit LNAV field that spans two words: its 8 MSBs sit in bits 17–24 of one
     * word and its 24 LSBs in bits 1–24 of the next (IS-GPS-200 Figure 20-1). The subframe
     * buffer keeps each word's 6 parity bits in place, so the two halves are 14 bits apart —
     * a contiguous 32-bit read would swallow the parity bits into the middle of the value
     * and drop the 6 true LSBs.
     */
    private fun getTwoWordFieldUnsigned(
        sf: ByteArray,
        msbStartBit: Int,
    ): Long {
        val msb = getBitsLong(sf, msbStartBit, 8)
        val lsb = getBitsLong(sf, msbStartBit + 14, 24)
        return (msb shl 24) or lsb
    }

    private fun getTwoWordFieldSigned(
        sf: ByteArray,
        msbStartBit: Int,
    ): Long {
        val value = getTwoWordFieldUnsigned(sf, msbStartBit)
        return if ((value and 0x8000_0000L) != 0L) value - 0x1_0000_0000L else value
    }

    /**
     * Decodes raw subframes into a SatelliteEphemeris object.
     *
     * Returns null when the IODC/IODE issue numbers of the three subframes disagree
     * (IS-GPS-200 §20.3.3.4.1): during a data-set cutover the stored subframes can belong
     * to different ephemerides, and a stitched ephemeris yields km-level orbit errors.
     */
    fun parseGpsEphemeris(
        svid: Int,
        sf1: ByteArray,
        sf2: ByteArray,
        sf3: ByteArray,
    ): SatelliteEphemeris? {
        return try {
            // IODC = SF1 word 3 bits 23-24 (2 MSBs) + word 8 bits 1-8; IODE = SF2/SF3 word 3/10 bits 1-8.
            val iodcLsb = getBits(sf1, 210, 8)
            val iodeSf2 = getBits(sf2, 60, 8)
            val iodeSf3 = getBits(sf3, 270, 8)
            if (iodeSf2 != iodeSf3 || iodeSf2 != iodcLsb) return null

            // SV health: SF1 word 3 bits 17-22 (IS-GPS-200 §20.3.3.3.1.4). Any non-zero value
            // means some or all nav data is bad — satellites in maintenance broadcast flagged
            // data that can be hundreds of meters off.
            if (getBits(sf1, 76, 6) != 0) return null

            // Subframe 1: Clock correction and health (IS-GPS-200 Table 20-I)
            val week = getBits(sf1, 60, 10)
            val tgd = getSignedBits(sf1, 196, 8) * 2.0.pow(-31) // Group delay differential
            val toc = getBits(sf1, 218, 16) * 16.0 // Clock data reference time
            val af2 = getSignedBits(sf1, 240, 8) * 2.0.pow(-55) // SV clock drift rate
            val af1 = getSignedBits(sf1, 248, 16) * 2.0.pow(-43) // SV clock drift
            val af0 = getSignedBits(sf1, 270, 22) * 2.0.pow(-31) // SV clock bias

            // Subframe 2: Ephemeris parameters (IS-GPS-200 Table 20-II)
            val crs = getSignedBits(sf2, 68, 16) * 2.0.pow(-5)
            val deltaN = getSignedBits(sf2, 90, 16) * 2.0.pow(-43) * PI
            val m0 = getTwoWordFieldSigned(sf2, 106) * 2.0.pow(-31) * PI
            val cuc = getSignedBits(sf2, 150, 16) * 2.0.pow(-29)
            val eccentricity = getTwoWordFieldUnsigned(sf2, 166) * 2.0.pow(-33)
            val cus = getSignedBits(sf2, 210, 16) * 2.0.pow(-29)
            val sqrtA = getTwoWordFieldUnsigned(sf2, 226) * 2.0.pow(-19)
            val toe = getBits(sf2, 270, 16) * 16.0 // Time of ephemeris (CORRECTED: from sf2, not sf1)

            // Subframe 3: More ephemeris parameters (IS-GPS-200 Table 20-III)
            val cic = getSignedBits(sf3, 60, 16) * 2.0.pow(-29)
            val omega0 = getTwoWordFieldSigned(sf3, 76) * 2.0.pow(-31) * PI
            val cis = getSignedBits(sf3, 120, 16) * 2.0.pow(-29)
            val i0 = getTwoWordFieldSigned(sf3, 136) * 2.0.pow(-31) * PI
            val crc = getSignedBits(sf3, 180, 16) * 2.0.pow(-5)
            val omega = getTwoWordFieldSigned(sf3, 196) * 2.0.pow(-31) * PI
            val omegaDot = getSignedBits(sf3, 240, 24) * 2.0.pow(-43) * PI
            val iDot = getSignedBits(sf3, 278, 14) * 2.0.pow(-43) * PI

            SatelliteEphemeris(
                constellationType = Constellation.GPS,
                svid = svid,
                week = week,
                iode = iodeSf2,
                toe = toe,
                toc = toc,
                af0 = af0,
                af1 = af1,
                af2 = af2,
                m0 = m0,
                deltaN = deltaN,
                eccentricity = eccentricity,
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
                tgd = tgd,
                lastUpdateTime = GnssPlatform.timeSource.elapsedRealtimeMs(),
            )
        } catch (e: Exception) {
            null
        }
    }

    /**
     * Decodes GPS Subframe 4, Page 18 into Klobuchar ionospheric coefficients.
     * Returns a Pair of DoubleArrays (alpha, beta).
     */
    fun parseKlobucharCoefficients(sf4: ByteArray): Pair<DoubleArray, DoubleArray>? {
        return try {
            // Data ID (bits 61-62) and SV ID (bits 63-68)
            // Page 18 has SV ID = 56
            val svId = getBits(sf4, 62, 6)
            if (svId != 56) return null

            // IS-GPS-200 Fig. 20-1 sheet 8 — word 3: dataId(2) svId(6) α0(8) α1(8);
            // word 4: α2(8) α3(8) β0(8); word 5: β1(8) β2(8) β3(8). Each 30-bit word
            // keeps its 6 parity bits in the buffer, so word 4 data starts at bit 90
            // and word 5 data at bit 120 (not contiguous after α1).
            val alpha = DoubleArray(4)
            alpha[0] = getSignedBits(sf4, 68, 8) * 2.0.pow(-30)
            alpha[1] = getSignedBits(sf4, 76, 8) * 2.0.pow(-27)
            alpha[2] = getSignedBits(sf4, 90, 8) * 2.0.pow(-24)
            alpha[3] = getSignedBits(sf4, 98, 8) * 2.0.pow(-24)

            val beta = DoubleArray(4)
            beta[0] = getSignedBits(sf4, 106, 8) * 2.0.pow(11)
            beta[1] = getSignedBits(sf4, 120, 8) * 2.0.pow(14)
            beta[2] = getSignedBits(sf4, 128, 8) * 2.0.pow(16)
            beta[3] = getSignedBits(sf4, 136, 8) * 2.0.pow(16)

            alpha to beta
        } catch (e: Exception) {
            null
        }
    }
}
