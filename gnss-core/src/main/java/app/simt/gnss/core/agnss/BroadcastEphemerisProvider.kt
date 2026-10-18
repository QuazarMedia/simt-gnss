package app.simt.gnss.core.agnss
import app.simt.gnss.core.model.Constellation
import app.simt.gnss.core.model.SatelliteEphemeris
import app.simt.gnss.core.platform.GnssPlatform
import app.simt.gnss.core.utils.GnssMathUtils
import java.io.BufferedReader

/**
 * Parses broadcast ephemeris from RINEX 3 mixed navigation file content.
 *
 * The download itself is left to the caller; this parser only turns the file content it is handed
 * into [SatelliteEphemeris] records.
 */
object BroadcastEphemerisProvider {
    private const val TAG = "BroadcastEphProv"

    /** GPS/QZSS, Galileo and BeiDou records span 8 RINEX lines; GLONASS spans 4. */
    private const val KEPLERIAN_RECORD_LINES = 8
    private const val GLONASS_RECORD_LINES = 4

    /** Retain ephemeris within this many seconds for offline propagation (24 hours). */
    private const val MAX_EPHEMERIS_AGE_SECONDS = 86400.0 // 24 hours

    /**
     * Galileo SV health is a bit field (RINEX 3.04 Table A8): E1-B bits 0–2, E5a bits 3–5,
     * E5b bits 6–8. An I/NAV record (data-source bits 0, 2 or 9) is rejected when E1-B is
     * flagged; an F/NAV record (bits 1 or 8) when E5a is flagged. A record with no source
     * bits is rejected when any of those nine bits is set.
     */
    internal fun galileoRecordUnhealthy(
        dataSources: Int,
        health: Int,
    ): Boolean {
        val e1b = health and 0x7
        val e5a = health and 0x38
        val inav = (dataSources and (1 or 4 or 512)) != 0
        val fnav = (dataSources and (2 or 256)) != 0
        if (!inav && !fnav) return (health and 0x1FF) != 0
        return (inav && e1b != 0) || (fnav && e5a != 0)
    }

    /**
     * Data-source bit 9 marks an I/NAV clock and bit 8 an F/NAV clock (RINEX 3.04).
     * When both are set the F/NAV clock wins: it is the reference the E5a and
     * ionosphere-free formulas already assume.
     */
    internal fun galileoClockIsInav(dataSources: Int): Boolean {
        val fnavClock = dataSources and (1 shl 8) != 0
        val inavClock = dataSources and (1 shl 9) != 0
        return inavClock && !fnavClock
    }

    /**
     * Parses a RINEX 3 mixed navigation file handed over as one complete [String].
     *
     * Prefer [parseRinex3MixedNav] with a [BufferedReader] or a [Sequence]. A full day's
     * broadcast navigation file is several megabytes of text; materialising it as a single
     * `String` costs that (plus roughly as much again to build it) and `String.lines()` then
     * allocates a second list holding every line. The streaming overloads keep only the
     * current record in memory.
     */
    fun parseRinex3MixedNav(
        content: String,
        gpsTimeOfWeekSeconds: Double,
    ): List<SatelliteEphemeris> = parseRinex3MixedNav(content.lineSequence(), gpsTimeOfWeekSeconds)

    /**
     * Parses a RINEX 3 mixed navigation stream line by line. The reader is consumed lazily and
     * is not closed by this function — the caller owns it.
     */
    fun parseRinex3MixedNav(
        reader: BufferedReader,
        gpsTimeOfWeekSeconds: Double,
    ): List<SatelliteEphemeris> = parseRinex3MixedNav(reader.lineSequence(), gpsTimeOfWeekSeconds)

    /**
     * Parses RINEX 3 mixed navigation file lines and returns ephemeris records.
     * Handles GPS (G), GLONASS (R), Galileo (E), and BeiDou (C) satellite systems.
     *
     * Lines are consumed lazily; at most one record (8 lines for the Keplerian systems, 4 for
     * GLONASS) is held at a time.
     *
     * @param lines Raw RINEX 3 mixed navigation file lines
     * @param gpsTimeOfWeekSeconds Current GPS time of week in seconds (for age filtering)
     * @return Deduplicated, age-filtered list of ephemeris records
     */
    fun parseRinex3MixedNav(
        lines: Sequence<String>,
        gpsTimeOfWeekSeconds: Double,
    ): List<SatelliteEphemeris> {
        val results = mutableListOf<SatelliteEphemeris>()
        val iterator = lines.iterator()

        // Skip header (ends with "END OF HEADER"). When it is absent the iterator is drained
        // and no record is parsed, matching the previous index-based scan.
        while (iterator.hasNext()) {
            if (iterator.next().contains("END OF HEADER")) break
        }

        val now = GnssPlatform.timeSource.elapsedRealtimeMs()
        var totalRecords = 0
        var skippedRecords = 0

        while (iterator.hasNext()) {
            val line = iterator.next()
            if (line.isBlank()) continue

            when (line[0]) {
                'G' -> {
                    totalRecords++
                    val eph =
                        readRecord(iterator, line, KEPLERIAN_RECORD_LINES)
                            ?.let { parseGpsRecord(it, now, gpsTimeOfWeekSeconds) }
                    if (eph != null) results.add(eph) else skippedRecords++
                }
                'E' -> {
                    totalRecords++
                    val eph =
                        readRecord(iterator, line, KEPLERIAN_RECORD_LINES)
                            ?.let { parseGalileoRecord(it, now, gpsTimeOfWeekSeconds) }
                    if (eph != null) results.add(eph) else skippedRecords++
                }
                'R' -> {
                    totalRecords++
                    val eph =
                        readRecord(iterator, line, GLONASS_RECORD_LINES)
                            ?.let { parseGlonassRecord(it, now, gpsTimeOfWeekSeconds) }
                    if (eph != null) results.add(eph) else skippedRecords++
                }
                'C' -> {
                    totalRecords++
                    val eph =
                        readRecord(iterator, line, KEPLERIAN_RECORD_LINES)
                            ?.let { parseBeidouRecord(it, now, gpsTimeOfWeekSeconds) }
                    if (eph != null) results.add(eph) else skippedRecords++
                }
                else -> Unit
            }
        }

        GnssPlatform.logger.d(TAG, "RINEX scan: $totalRecords total records, $skippedRecords skipped (age/health/parse), ${results.size} kept, gpsTow=${"%.0f".format(gpsTimeOfWeekSeconds)}")

        // Deduplicate: keep the record whose orbit reference time is closest to NOW.
        // (Sorting GLONASS by `toe` previously kept the file's OLDEST record — GLONASS
        // records carry toe = 0 and are referenced by tb instead.)
        fun referenceTow(eph: SatelliteEphemeris): Double = if (eph.isGlonass) eph.gloTbGpsTow else eph.toe

        fun ageSeconds(eph: SatelliteEphemeris): Double {
            var dt = gpsTimeOfWeekSeconds - referenceTow(eph)
            if (dt > 302400.0) dt -= 604800.0
            if (dt < -302400.0) dt += 604800.0
            return kotlin.math.abs(dt)
        }

        val deduped =
            results
                .groupBy { it.constellationType to it.svid }
                .mapValues { (_, ephs) ->
                    ephs.minWithOrNull(
                        compareBy<SatelliteEphemeris>(::ageSeconds).thenBy { if (it.galileoInavClock) 1 else 0 },
                    ) ?: ephs.first()
                }
                .values
                .toList()

        // Filter by age. GLONASS state vectors are only propagatable for a short window
        // around tb (the broadcast force model holds for ±15 min, tb refreshes every
        // 30 min) — a Keplerian-style multi-hour window would feed the RK4 integrator
        // states it cannot extrapolate.
        val filtered =
            deduped.filter { eph ->
                val maxAge = if (eph.isGlonass) GnssMathUtils.GLONASS_MAX_PROPAGATION_SECONDS else MAX_EPHEMERIS_AGE_SECONDS
                ageSeconds(eph) <= maxAge
            }
        GnssPlatform.logger.d(TAG, "Age filter: ${deduped.size} deduped → ${filtered.size} within validity of current time")
        return filtered
    }

    /**
     * Reads a record's continuation lines. [firstLine] is the system-character line already
     * consumed by the caller; the remaining [size] − 1 lines are pulled from [iterator].
     * Returns null when the stream ends (or the record is short) before it is complete.
     */
    private fun readRecord(
        iterator: Iterator<String>,
        firstLine: String,
        size: Int,
    ): List<String>? {
        val record = ArrayList<String>(size)
        record.add(firstLine)
        while (record.size < size) {
            if (!iterator.hasNext()) return null
            record.add(iterator.next())
        }
        return record
    }

    // ---- GPS/QZSS RINEX record (8 lines) ----
    private fun parseGpsRecord(
        record: List<String>,
        now: Long,
        gpsTow: Double,
    ): SatelliteEphemeris? {
        return try {
            val l0 = record[0]
            val svid = l0.substring(1, 3).trim().toInt()
            val af0 = parseRinexDouble(l0, 23, 19)
            val af1 = parseRinexDouble(l0, 42, 19)
            val af2 = parseRinexDouble(l0, 61, 19)

            val l1 = record[1]
            val iode = parseRinexDouble(l1, 4, 19).toInt() // IODE — gates SSR application
            val crs = parseRinexDouble(l1, 23, 19)
            val deltaN = parseRinexDouble(l1, 42, 19) // already in rad/s
            val m0 = parseRinexDouble(l1, 61, 19) // rad

            val l2 = record[2]
            val cuc = parseRinexDouble(l2, 4, 19)
            val eccentricity = parseRinexDouble(l2, 23, 19)
            val cus = parseRinexDouble(l2, 42, 19)
            val sqrtA = parseRinexDouble(l2, 61, 19)

            val l3 = record[3]
            val toe = parseRinexDouble(l3, 4, 19)
            val cic = parseRinexDouble(l3, 23, 19)
            val omega0 = parseRinexDouble(l3, 42, 19) // OMEGA0 (rad)
            val cis = parseRinexDouble(l3, 61, 19)

            val l4 = record[4]
            val i0 = parseRinexDouble(l4, 4, 19) // rad
            val crc = parseRinexDouble(l4, 23, 19)
            val omega = parseRinexDouble(l4, 42, 19) // omega (rad)
            val omegaDot = parseRinexDouble(l4, 61, 19) // rad/s

            val l5 = record[5]
            val iDot = parseRinexDouble(l5, 4, 19) // rad/s
            val week = parseRinexDouble(l5, 42, 19).toInt()

            val l6 = record[6]
            val health = parseRinexDouble(l6, 23, 19).toInt()
            if (health != 0) return null
            val tgd = parseRinexDouble(l6, 42, 19)

            val l7 = record[7]
            val toc = toe // RINEX stores epoch which maps to toc

            SatelliteEphemeris(
                constellationType = Constellation.GPS,
                svid = svid,
                week = week,
                iode = iode,
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
                lastUpdateTime = now,
            )
        } catch (e: Exception) {
            null
        }
    }

    // ---- Galileo RINEX record (8 lines, same structure as GPS) ----
    private fun parseGalileoRecord(
        record: List<String>,
        now: Long,
        gpsTow: Double,
    ): SatelliteEphemeris? {
        return try {
            val l0 = record[0]
            val svid = l0.substring(1, 3).trim().toInt()
            val af0 = parseRinexDouble(l0, 23, 19)
            val af1 = parseRinexDouble(l0, 42, 19)
            val af2 = parseRinexDouble(l0, 61, 19)

            val l1 = record[1]
            val iodNav = parseRinexDouble(l1, 4, 19).toInt() // IODnav — gates SSR application
            val crs = parseRinexDouble(l1, 23, 19)
            val deltaN = parseRinexDouble(l1, 42, 19)
            val m0 = parseRinexDouble(l1, 61, 19)

            val l2 = record[2]
            val cuc = parseRinexDouble(l2, 4, 19)
            val eccentricity = parseRinexDouble(l2, 23, 19)
            val cus = parseRinexDouble(l2, 42, 19)
            val sqrtA = parseRinexDouble(l2, 61, 19)

            val l3 = record[3]
            val toe = parseRinexDouble(l3, 4, 19)
            val cic = parseRinexDouble(l3, 23, 19)
            val omega0 = parseRinexDouble(l3, 42, 19)
            val cis = parseRinexDouble(l3, 61, 19)

            val l4 = record[4]
            val i0 = parseRinexDouble(l4, 4, 19)
            val crc = parseRinexDouble(l4, 23, 19)
            val omega = parseRinexDouble(l4, 42, 19)
            val omegaDot = parseRinexDouble(l4, 61, 19)

            val l5 = record[5]
            val iDot = parseRinexDouble(l5, 4, 19)
            val dataSources = parseRinexDouble(l5, 23, 19).toInt()
            val week = parseRinexDouble(l5, 42, 19).toInt()

            val l6 = record[6]
            val health = parseRinexDouble(l6, 23, 19).toInt()
            if (galileoRecordUnhealthy(dataSources, health)) return null
            // BROADCAST ORBIT - 6: BGD(E1,E5a) then BGD(E1,E5b). The stored clock's own
            // reference goes in tgd; both delays are kept for the per-signal formula.
            val bgdE1E5a = parseRinexDouble(l6, 42, 19)
            val bgdE1E5b = parseRinexDouble(l6, 61, 19)
            val inavClock = galileoClockIsInav(dataSources)
            val tgd = if (inavClock) bgdE1E5b else bgdE1E5a

            SatelliteEphemeris(
                constellationType = Constellation.GALILEO,
                svid = svid,
                week = week,
                iode = iodNav,
                toe = toe,
                toc = toe,
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
                galileoBgdE1E5aSeconds = bgdE1E5a,
                galileoBgdE1E5bSeconds = bgdE1E5b,
                galileoInavClock = inavClock,
                lastUpdateTime = now,
            )
        } catch (e: Exception) {
            null
        }
    }

    // ---- GLONASS RINEX record (4 lines) ----
    private fun parseGlonassRecord(
        record: List<String>,
        now: Long,
        gpsTow: Double,
    ): SatelliteEphemeris? {
        return try {
            val l0 = record[0]
            val svid = l0.substring(1, 3).trim().toInt()
            // RINEX GLONASS line 0: epoch (= tb, UTC), -TauN, +GammaN, message frame time (tk, unused)
            val tauN = -parseRinexDouble(l0, 23, 19) // RINEX stores -TauN, we want +TauN
            val gammaN = parseRinexDouble(l0, 42, 19)

            // Line 1: X(km), Xdot(km/s), Xdotdot(km/s²), health
            val l1 = record[1]
            val posX = parseRinexDouble(l1, 4, 19) * 1e3 // km → m
            val velX = parseRinexDouble(l1, 23, 19) * 1e3 // km/s → m/s
            val accX = parseRinexDouble(l1, 42, 19) * 1e3 // km/s² → m/s²
            val health = parseRinexDouble(l1, 61, 19).toInt()

            // Line 2: Y, Ydot, Ydotdot, frequency number
            val l2 = record[2]
            val posY = parseRinexDouble(l2, 4, 19) * 1e3
            val velY = parseRinexDouble(l2, 23, 19) * 1e3
            val accY = parseRinexDouble(l2, 42, 19) * 1e3

            // Line 3: Z, Zdot, Zdotdot, age of ephemeris
            val l3 = record[3]
            val posZ = parseRinexDouble(l3, 4, 19) * 1e3
            val velZ = parseRinexDouble(l3, 23, 19) * 1e3
            val accZ = parseRinexDouble(l3, 42, 19) * 1e3

            // Unhealthy satellite
            if (health != 0) return null

            // tb comes from the record's EPOCH timestamp (UTC), not from the third float on
            // line 0 — that field is the message frame time tk, up to ±15 min away from tb
            // (and week-seconds in some RINEX minor versions). Using tk as tb mislabeled the
            // state vector's reference time, propagating from the wrong instant.
            val year = l0.substring(4, 8).trim().toInt()
            val month = l0.substring(9, 11).trim().toInt()
            val day = l0.substring(12, 14).trim().toInt()
            val hour = l0.substring(15, 17).trim().toInt()
            val minute = l0.substring(18, 20).trim().toInt()
            val second = l0.substring(21, 23).trim().toInt()
            val epochCalendar =
                java.util.GregorianCalendar(java.util.TimeZone.getTimeZone("UTC")).apply {
                    clear()
                    set(year, month - 1, day, hour, minute, second)
                }
            val epochUnixSeconds = epochCalendar.timeInMillis / 1000.0
            val epochGpsSeconds = epochUnixSeconds - GnssMathUtils.GPS_EPOCH_UNIX_SECONDS + GnssMathUtils.GPS_UTC_LEAP_SECONDS
            var tbGpsTow = epochGpsSeconds % 604800.0
            if (tbGpsTow < 0) tbGpsTow += 604800.0

            SatelliteEphemeris(
                constellationType = Constellation.GLONASS,
                svid = svid,
                lastUpdateTime = now,
                isGlonass = true,
                gloPosX = posX,
                gloPosY = posY,
                gloPosZ = posZ,
                gloVelX = velX,
                gloVelY = velY,
                gloVelZ = velZ,
                gloAccX = accX,
                gloAccY = accY,
                gloAccZ = accZ,
                gloTauN = tauN,
                gloGammaN = gammaN,
                gloTbGpsTow = tbGpsTow,
                gloBn = health,
                af0 = -tauN, // clock bias (GLONASS convention)
                af1 = gammaN,
            )
        } catch (e: Exception) {
            null
        }
    }

    // ---- BeiDou RINEX record (8 lines, Keplerian like GPS) ----
    private fun parseBeidouRecord(
        record: List<String>,
        now: Long,
        gpsTow: Double,
    ): SatelliteEphemeris? {
        return try {
            val l0 = record[0]
            val svid = l0.substring(1, 3).trim().toInt()
            val af0 = parseRinexDouble(l0, 23, 19)
            val af1 = parseRinexDouble(l0, 42, 19)
            val af2 = parseRinexDouble(l0, 61, 19)

            val l1 = record[1]
            val crs = parseRinexDouble(l1, 23, 19)
            val deltaN = parseRinexDouble(l1, 42, 19)
            val m0 = parseRinexDouble(l1, 61, 19)

            val l2 = record[2]
            val cuc = parseRinexDouble(l2, 4, 19)
            val eccentricity = parseRinexDouble(l2, 23, 19)
            val cus = parseRinexDouble(l2, 42, 19)
            val sqrtA = parseRinexDouble(l2, 61, 19)

            val l3 = record[3]
            val toe = parseRinexDouble(l3, 4, 19)
            val cic = parseRinexDouble(l3, 23, 19)
            val omega0 = parseRinexDouble(l3, 42, 19)
            val cis = parseRinexDouble(l3, 61, 19)

            val l4 = record[4]
            val i0 = parseRinexDouble(l4, 4, 19)
            val crc = parseRinexDouble(l4, 23, 19)
            val omega = parseRinexDouble(l4, 42, 19)
            val omegaDot = parseRinexDouble(l4, 61, 19)

            val l5 = record[5]
            val iDot = parseRinexDouble(l5, 4, 19)
            val week = parseRinexDouble(l5, 42, 19).toInt()

            val l6 = record[6]
            val satH1 = parseRinexDouble(l6, 23, 19).toInt()
            if (satH1 != 0) return null
            val tgd = parseRinexDouble(l6, 42, 19)

            SatelliteEphemeris(
                constellationType = Constellation.BEIDOU,
                svid = svid,
                week = week,
                toe = toe, // keep in BDT
                toc = toe,
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
                lastUpdateTime = now,
            )
        } catch (e: Exception) {
            null
        }
    }

    /**
     * Parse a RINEX double value from a fixed-width field.
     * RINEX uses 'D' or 'd' for exponent notation instead of 'E'.
     */
    private fun parseRinexDouble(
        line: String,
        start: Int,
        length: Int,
    ): Double {
        if (start >= line.length) return 0.0
        val end = minOf(start + length, line.length)
        val field = line.substring(start, end).trim()
        if (field.isEmpty()) return 0.0
        return field.replace('D', 'E').replace('d', 'e').toDouble()
    }
}
