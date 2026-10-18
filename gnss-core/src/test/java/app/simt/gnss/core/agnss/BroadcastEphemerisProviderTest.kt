package app.simt.gnss.core.agnss

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Test
import java.time.OffsetDateTime
import java.time.ZoneOffset
import java.util.Locale

class BroadcastEphemerisProviderTest {
    /** Formats one 19-character RINEX 3 float field with a D exponent. */
    private fun f(v: Double): String = String.format(Locale.US, "%19.12E", v).replace("E", "D")

    private fun gpsTow(
        year: Int,
        month: Int,
        day: Int,
        hour: Int,
        minute: Int,
    ): Double {
        val unix = OffsetDateTime.of(year, month, day, hour, minute, 0, 0, ZoneOffset.UTC).toEpochSecond()
        return ((unix - 315_964_800L + 18L) % 604_800L).toDouble()
    }

    /** One RINEX 3 GLONASS record (4 lines) with a frame time deliberately ≠ the epoch. */
    private fun glonassRecord(
        svid: Int,
        hour: Int,
        minute: Int,
        frameTimeSecondsOfDay: Double,
    ): String =
        buildString {
            append("R")
            append(String.format(Locale.US, "%02d", svid))
            append(" 2026 06 11 ")
            append(String.format(Locale.US, "%02d %02d 00", hour, minute))
            append(f(-1.86e-4)) // -TauN
            append(f(0.0)) // GammaN
            append(f(frameTimeSecondsOfDay)) // message frame time tk — NOT tb
            append('\n')
            append("    ").append(f(11_000.0)).append(f(-0.5)).append(f(0.0)).append(f(0.0)).append('\n') // X, health=0
            append("    ").append(f(15_000.0)).append(f(2.7)).append(f(0.0)).append(f(1.0)).append('\n') // Y, FCN
            append("    ").append(f(18_000.0)).append(f(-1.9)).append(f(0.0)).append(f(0.0)).append('\n') // Z, age
        }

    private fun rinex(records: String): String =
        "     3.04           N: GNSS NAV DATA    M: MIXED            RINEX VERSION / TYPE\n" +
            "                                                            END OF HEADER\n" +
            records

    @Test
    fun glonassRecord_tbComesFromTheEpochLine_notTheFrameTime() {
        // Epoch 06:15 UTC; frame time field set 900 s later. tb must derive from the epoch:
        // using tk as tb mislabels the state vector's reference time.
        val epochTow = gpsTow(2026, 6, 11, 6, 15)
        val content = rinex(glonassRecord(svid = 5, hour = 6, minute = 15, frameTimeSecondsOfDay = 6 * 3600.0 + 15 * 60.0 + 900.0))

        val records = BroadcastEphemerisProvider.parseRinex3MixedNav(content, gpsTimeOfWeekSeconds = epochTow + 600.0)
        assertEquals(1, records.size)
        val eph = records[0]
        assertEquals(5, eph.svid)
        assertEquals("tb must equal the record epoch (UTC + leap) as GPS TOW", epochTow, eph.gloTbGpsTow, 1e-6)
        assertEquals(11_000_000.0, eph.gloPosX, 1e-3)
        assertEquals(2_700.0, eph.gloVelY, 1e-9)
    }

    @Test
    fun glonassDedup_keepsTheRecordClosestToNow() {
        // Two records for the same SV at 05:45 and 06:15; "now" at 06:25 → 06:15 wins.
        // (The previous dedup sorted by toe, which GLONASS records carry as 0 — keeping
        // whichever happened to come first, i.e. the file's oldest.)
        val content =
            rinex(
                glonassRecord(svid = 5, hour = 5, minute = 45, frameTimeSecondsOfDay = 0.0) +
                    glonassRecord(svid = 5, hour = 6, minute = 15, frameTimeSecondsOfDay = 0.0),
            )
        val now = gpsTow(2026, 6, 11, 6, 25)
        val records = BroadcastEphemerisProvider.parseRinex3MixedNav(content, gpsTimeOfWeekSeconds = now)
        assertEquals(1, records.size)
        assertEquals(gpsTow(2026, 6, 11, 6, 15), records[0].gloTbGpsTow, 1e-6)
    }

    @Test
    fun glonassRecord_outsidePropagationValidity_isDropped() {
        // tb three hours old: the RK4 force model cannot extrapolate that far — the record
        // must be filtered rather than fed to the integrator.
        val content = rinex(glonassRecord(svid = 7, hour = 3, minute = 15, frameTimeSecondsOfDay = 0.0))
        val now = gpsTow(2026, 6, 11, 6, 15)
        val records = BroadcastEphemerisProvider.parseRinex3MixedNav(content, gpsTimeOfWeekSeconds = now)
        assertEquals(0, records.size)
    }

    @Test
    fun glonassRecord_unhealthy_isDropped() {
        val record =
            buildString {
                append("R05 2026 06 11 06 15 00")
                append(f(-1.86e-4)).append(f(0.0)).append(f(0.0)).append('\n')
                append("    ").append(f(11_000.0)).append(f(-0.5)).append(f(0.0)).append(f(4.0)).append('\n') // health = 4
                append("    ").append(f(15_000.0)).append(f(2.7)).append(f(0.0)).append(f(1.0)).append('\n')
                append("    ").append(f(18_000.0)).append(f(-1.9)).append(f(0.0)).append(f(0.0)).append('\n')
            }
        val now = gpsTow(2026, 6, 11, 6, 20)
        assertEquals(0, BroadcastEphemerisProvider.parseRinex3MixedNav(rinex(record), now).size)
    }

    @Test
    fun gpsRecord_stillParsesAfterGlonassChanges() {
        // Sanity: an 8-line GPS record with toe near now survives the shared filter path.
        val now = 345_600.0
        val toe = now - 600.0
        val gps =
            buildString {
                append("G07 2026 06 11 00 00 00").append(f(1.0e-4)).append(f(0.0)).append(f(0.0)).append('\n')
                append("    ").append(f(46.0)).append(f(-55.65)).append(f(4.4e-9)).append(f(0.7)).append('\n')
                append("    ").append(f(-1.9e-6)).append(f(0.0091)).append(f(7.6e-6)).append(f(5153.683)).append('\n')
                append("    ").append(f(toe)).append(f(3.7e-8)).append(f(-0.456)).append(f(-7.6e-8)).append('\n')
                append("    ").append(f(0.961)).append(f(226.03)).append(f(-0.8)).append(f(-8.0e-9)).append('\n')
                append("    ").append(f(4.0e-10)).append(f(1.0)).append(f(2414.0)).append(f(0.0)).append('\n')
                append("    ").append(f(2.0)).append(f(0.0)).append(f(-1.0e-8)).append(f(46.0)).append('\n')
                append("    ").append(f(now)).append(f(4.0)).append('\n')
            }
        val records = BroadcastEphemerisProvider.parseRinex3MixedNav(rinex(gps), now)
        assertEquals(1, records.size)
        assertNotNull(records[0])
        assertEquals(7, records[0].svid)
        assertEquals(toe, records[0].toe, 1e-6)
        assertEquals(46, records[0].iode)
    }

    private fun keplerRecord(
        system: Char,
        svid: Int,
        toe: Double,
        now: Double,
        line5Field2: Double,
        health: Double,
        bgdE1E5a: Double = -1.0e-8,
        bgdE1E5b: Double = 0.0,
    ): String =
        buildString {
            append(system)
            append(String.format(Locale.US, "%02d", svid))
            append(" 2026 06 11 00 00 00").append(f(1.0e-4)).append(f(0.0)).append(f(0.0)).append('\n')
            append("    ").append(f(46.0)).append(f(-55.65)).append(f(4.4e-9)).append(f(0.7)).append('\n')
            append("    ").append(f(-1.9e-6)).append(f(0.0091)).append(f(7.6e-6)).append(f(5153.683)).append('\n')
            append("    ").append(f(toe)).append(f(3.7e-8)).append(f(-0.456)).append(f(-7.6e-8)).append('\n')
            append("    ").append(f(0.961)).append(f(226.03)).append(f(-0.8)).append(f(-8.0e-9)).append('\n')
            append("    ").append(f(4.0e-10)).append(f(line5Field2)).append(f(2414.0)).append(f(0.0)).append('\n')
            append("    ").append(f(2.0)).append(f(health)).append(f(bgdE1E5a)).append(f(bgdE1E5b)).append('\n')
            append("    ").append(f(now)).append(f(4.0)).append('\n')
        }

    @Test
    fun unhealthyGpsAndBeidou_areNotImported() {
        val now = 345_600.0
        val toe = now - 600.0
        val unhealthyGps = keplerRecord('G', 20, toe, now, line5Field2 = 1.0, health = 63.0)
        val unhealthyBeidou = keplerRecord('C', 47, toe, now, line5Field2 = 0.0, health = 1.0)
        val healthyBeidou = keplerRecord('C', 6, toe, now, line5Field2 = 0.0, health = 0.0)
        assertEquals(0, BroadcastEphemerisProvider.parseRinex3MixedNav(rinex(unhealthyGps), now).size)
        assertEquals(0, BroadcastEphemerisProvider.parseRinex3MixedNav(rinex(unhealthyBeidou), now).size)
        assertEquals(1, BroadcastEphemerisProvider.parseRinex3MixedNav(rinex(healthyBeidou), now).size)
    }

    @Test
    fun galileoHealth_rejectsTheSignalsTheRecordActuallyCarries() {
        val now = 345_600.0
        val toe = now - 600.0
        // I/NAV (data-source bit 0) with E1-B health 130, the E14/E18 pattern.
        val inavBad = keplerRecord('E', 14, toe, now, line5Field2 = 1.0, health = 130.0)
        // F/NAV (bit 1) with E5a health 16.
        val fnavBad = keplerRecord('E', 18, toe, now, line5Field2 = 2.0, health = 16.0)
        // I/NAV with only E5b flagged (bit 7 = 128). E1-B is healthy, so the record stays.
        val inavE5bOnly = keplerRecord('E', 11, toe, now, line5Field2 = 1.0, health = 128.0)
        assertEquals(0, BroadcastEphemerisProvider.parseRinex3MixedNav(rinex(inavBad), now).size)
        assertEquals(0, BroadcastEphemerisProvider.parseRinex3MixedNav(rinex(fnavBad), now).size)
        assertEquals(1, BroadcastEphemerisProvider.parseRinex3MixedNav(rinex(inavE5bOnly), now).size)
    }

    @Test
    fun galileoClock_storesBothBgdsAndPrefersFnavWhenToesMatch() {
        val now = 345_600.0
        val toe = now - 600.0
        val bgdA = 1.0e-9
        val bgdB = 2.4e-9
        // Data-source bit 9 (512): I/NAV clock. Listed first so a first-wins dedupe would keep it.
        val inav =
            keplerRecord('E', 11, toe, now, line5Field2 = 512.0, health = 0.0, bgdE1E5a = bgdA, bgdE1E5b = bgdB)
        val fnav =
            keplerRecord('E', 11, toe, now, line5Field2 = 256.0, health = 0.0, bgdE1E5a = bgdA, bgdE1E5b = bgdB)
        val preferred = BroadcastEphemerisProvider.parseRinex3MixedNav(rinex(inav + fnav), now).single()
        assertEquals(false, preferred.galileoInavClock)
        assertEquals(bgdA, preferred.tgd, 0.0)
        assertEquals(bgdA, preferred.galileoBgdE1E5aSeconds, 0.0)
        assertEquals(bgdB, preferred.galileoBgdE1E5bSeconds, 0.0)

        val onlyInav = BroadcastEphemerisProvider.parseRinex3MixedNav(rinex(inav), now).single()
        assertEquals(true, onlyInav.galileoInavClock)
        assertEquals(bgdB, onlyInav.tgd, 0.0)
    }
}
