package app.simt.gnss.core.parser
import app.simt.gnss.core.model.Constellation
import app.simt.gnss.core.model.SatelliteEphemeris
import app.simt.gnss.core.platform.GnssPlatform
import app.simt.gnss.core.utils.GnssMathUtils.getBits
import app.simt.gnss.core.utils.GnssMathUtils.getSignMagnitude
import kotlin.math.floor
import kotlin.math.pow

/**
 * Parser for GLONASS L1OF navigation messages (strings 1-4).
 *
 * Bit layout follows GLONASS ICD Edition 5.1 (2008) / RTKLIB convention.
 * Each string is 85 bits in 11 bytes (MSB-first). Bit numbering is 0-indexed.
 *
 * - Bit 0: idle
 * - Bits 1-4: string number (m)
 * - Bits 5+: navigation data
 */
object GlonassNavMessageParser {
    private const val TAG = "GlonassNavParser"

    /**
     * Parse immediate ephemeris from GLONASS strings 1-4.
     *
     * @param svid GLONASS SV id from Android HAL
     * @param strings Map of string number (1-4) → raw 11-byte data
     * @param gpsTimeOfWeekSeconds Current GPS TOW when this message set was collected
     * @param leapSeconds GPS-UTC leap second count
     */
    fun parseGlonassEphemeris(
        svid: Int,
        strings: Map<Int, ByteArray>,
        gpsTimeOfWeekSeconds: Double,
        leapSeconds: Int = 18,
    ): SatelliteEphemeris? {
        val str1 = strings[1] ?: return null
        val str2 = strings[2] ?: return null
        val str3 = strings[3] ?: return null
        val str4 = strings[4] ?: return null

        return try {
            // --- String 1: X state vector, tk ---
            // GLONASS ICD 5.1 Table 4.5 (flat bit = 85 − ICD bit; ICD numbers right-to-left):
            // Bit 0: idle, Bits 1-4: m, Bits 5-6: unused (2 bits, ICD 80-79), Bits 7-8: P1 (ICD 78-77)
            // Bits 9-13: tk hours (5 bits), ICD bits 76-72
            val tkH = getBits(str1, 9, 5)
            // Bits 14-19: tk minutes (6 bits), ICD bits 71-66
            val tkM = getBits(str1, 14, 6)
            // Bit 20: tk 30-second mark, ICD bit 65
            val tkS = getBits(str1, 20, 1) * 30
            // Bits 21-44: xn'(tb) velocity X (24 bits sign-mag, ×2^-20 km/s), ICD bits 64-41
            val xnDot = getSignMagnitude(str1, 21, 24) * 2.0.pow(-20) * 1e3 // → m/s
            // Bits 45-49: xn''(tb) acceleration X (5 bits sign-mag, ×2^-30 km/s²), ICD bits 40-36
            val xnDotDot = getSignMagnitude(str1, 45, 5) * 2.0.pow(-30) * 1e3 // → m/s²
            // Bits 50-76: xn(tb) position X (27 bits sign-mag, ×2^-11 km), ICD bits 35-9
            val xn = getSignMagnitude(str1, 50, 27) * 2.0.pow(-11) * 1e3 // → m

            // --- String 2: Y state vector, Bn, tb ---
            // Bits 5-7: Bn health (3 bits), ICD bits 78-80
            val bn = getBits(str2, 5, 3)
            // Bit 8: P2, ICD bit 77
            // Bits 9-15: tb (7 bits, × 15 minutes in Moscow time), ICD bits 70-76
            val tb = getBits(str2, 9, 7) // 0-95, each unit = 15 min
            // Bits 16-20: spare (5 bits)
            // Bits 21-44: yn'(tb) velocity Y (24 bits sign-mag), ICD bits 41-64
            val ynDot = getSignMagnitude(str2, 21, 24) * 2.0.pow(-20) * 1e3
            // Bits 45-49: yn''(tb) acceleration Y (5 bits sign-mag), ICD bits 36-40
            val ynDotDot = getSignMagnitude(str2, 45, 5) * 2.0.pow(-30) * 1e3
            // Bits 50-76: yn(tb) position Y (27 bits sign-mag), ICD bits 9-35
            val yn = getSignMagnitude(str2, 50, 27) * 2.0.pow(-11) * 1e3

            // --- String 3: Z state vector, γn ---
            // Bit 5: P3, ICD bit 80
            // Bits 6-16: γn(tb) frequency offset (11 bits sign-mag, ×2^-40), ICD bits 79-69
            val gammaN = getSignMagnitude(str3, 6, 11) * 2.0.pow(-40)
            // Bit 17: spare, ICD bit 68; Bits 18-19: p (2 bits), ICD bits 67-66; Bit 20: ln health, ICD bit 65
            // Bits 21-44: zn'(tb) velocity Z (24 bits sign-mag), ICD bits 64-41
            val znDot = getSignMagnitude(str3, 21, 24) * 2.0.pow(-20) * 1e3
            // Bits 45-49: zn''(tb) acceleration Z (5 bits sign-mag), ICD bits 40-36
            val znDotDot = getSignMagnitude(str3, 45, 5) * 2.0.pow(-30) * 1e3
            // Bits 50-76: zn(tb) position Z (27 bits sign-mag), ICD bits 35-9
            val zn = getSignMagnitude(str3, 50, 27) * 2.0.pow(-11) * 1e3

            // --- String 4: τn (clock bias), Δτn ---
            // Bits 5-26: τn(tb) clock bias (22 bits sign-mag, ×2^-30 s), ICD bits 59-80
            val tauN = getSignMagnitude(str4, 5, 22) * 2.0.pow(-30)
            // Bits 27-31: Δτn L1-L2 delay (5 bits sign-mag, ×2^-30 s), ICD bits 54-58
            // Bits 32-36: En age of data (5 bits), ICD bits 49-53

            // Sanity: position should be reasonable for GLONASS orbit (~25,510 km from center)
            val rKm = Math.sqrt(xn * xn + yn * yn + zn * zn) / 1e3
            GnssPlatform.logger.d(TAG, "svid=$svid rKm=${"%.0f".format(rKm)} tb=$tb bn=$bn")
            if (rKm < 20000 || rKm > 30000) {
                GnssPlatform.logger.w(TAG, "svid=$svid radius ${"%.0f".format(rKm)}km out of range — rejecting")
                return null
            }

            // Health check: Bn bit 2 set means malfunction
            if ((bn and 0x4) != 0) return null

            // Convert tb to GPS Time of Week
            val tbGpsTow = convertTbToGpsTow(tb, gpsTimeOfWeekSeconds, leapSeconds)

            SatelliteEphemeris(
                constellationType = Constellation.GLONASS,
                svid = svid,
                lastUpdateTime = GnssPlatform.timeSource.elapsedRealtimeMs(),
                isGlonass = true,
                gloPosX = xn,
                gloPosY = yn,
                gloPosZ = zn,
                gloVelX = xnDot,
                gloVelY = ynDot,
                gloVelZ = znDot,
                gloAccX = xnDotDot,
                gloAccY = ynDotDot,
                gloAccZ = znDotDot,
                gloTauN = tauN,
                gloGammaN = gammaN,
                gloTbGpsTow = tbGpsTow,
                gloBn = bn,
                af0 = -tauN, // clock bias (GLONASS sign convention: negated)
                af1 = gammaN,
            )
        } catch (e: Exception) {
            GnssPlatform.logger.w(TAG, "svid=$svid parse exception: ${e.message}")
            null
        }
    }

    /**
     * Convert GLONASS tb (7-bit, ×15 min from midnight Moscow time) to GPS TOW.
     */
    private fun convertTbToGpsTow(
        tb: Int,
        gpsTimeOfWeekSeconds: Double,
        leapSeconds: Int,
    ): Double {
        val tbMoscowSeconds = tb * 900.0 // seconds of day in Moscow time
        val tbUtcSeconds = tbMoscowSeconds - 10800.0 // UTC = Moscow - 3h
        val tbGpsSecondsOfDay = tbUtcSeconds + leapSeconds

        // Determine which GPS day of the week the observation falls on
        val currentGpsDay = floor(gpsTimeOfWeekSeconds / 86400.0)
        var tbGpsTow = currentGpsDay * 86400.0 + tbGpsSecondsOfDay

        // Handle day rollover (Moscow day may differ from GPS day)
        val dt = gpsTimeOfWeekSeconds - tbGpsTow
        if (dt > 43200.0) {
            tbGpsTow += 86400.0
        } else if (dt < -43200.0) {
            tbGpsTow -= 86400.0
        }

        // Clamp to week
        if (tbGpsTow < 0) tbGpsTow += 604800.0
        if (tbGpsTow >= 604800.0) tbGpsTow -= 604800.0

        return tbGpsTow
    }
}
