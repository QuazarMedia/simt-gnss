package app.simt.gnss

import android.location.GnssNavigationMessage
import android.location.GnssStatus
import app.simt.gnss.core.model.SatelliteEphemeris
import app.simt.gnss.core.utils.GnssMathUtils
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import org.robolectric.shadows.ShadowSystemClock
import java.time.Duration
import kotlin.math.abs

@RunWith(RobolectricTestRunner::class)
class GnssEphemerisImportTest {
    private val now = 345_600.0

    private fun gpsEphemeris(
        toe: Double,
        lastUpdateTime: Long,
    ) = SatelliteEphemeris(
        constellationType = GnssStatus.CONSTELLATION_GPS,
        svid = 7,
        toe = toe,
        sqrtA = 5153.7,
        lastUpdateTime = lastUpdateTime,
    )

    @Test
    fun import_comparesDataRecency_notReceiptWallClock() {
        val processor = GnssMeasurementProcessor()

        // Fresh data (toe 10 min old), decoded long ago.
        assertEquals(1, processor.importExternalEphemeris(listOf(gpsEphemeris(toe = now - 600.0, lastUpdateTime = 0L)), now))

        // Staler data (toe ~1.4 h old) with a NEWER receipt timestamp — the previous
        // wall-clock comparison let every periodic import overwrite fresher data.
        assertEquals(
            "older orbit data must not replace fresher data despite newer receipt time",
            0,
            processor.importExternalEphemeris(listOf(gpsEphemeris(toe = now - 5_000.0, lastUpdateTime = 999_999L)), now),
        )

        // Genuinely fresher data replaces.
        assertEquals(1, processor.importExternalEphemeris(listOf(gpsEphemeris(toe = now - 60.0, lastUpdateTime = 1L)), now))
    }

    @Test
    fun import_handlesWeekRolloverInRecencyComparison() {
        val processor = GnssMeasurementProcessor()
        val nowNearRollover = 100.0 // just after the GPS week boundary

        // toe just before the rollover (last week) = 110 s old — fresh.
        assertEquals(1, processor.importExternalEphemeris(listOf(gpsEphemeris(toe = 604_790.0, lastUpdateTime = 0L)), nowNearRollover))
        // toe 2 h before the rollover — staler, must not replace.
        assertEquals(0, processor.importExternalEphemeris(listOf(gpsEphemeris(toe = 597_600.0, lastUpdateTime = 5L)), nowNearRollover))
    }

    @Test
    fun import_beidouRecency_accountsForBdtGpst14sOffset() {
        val processor = GnssMeasurementProcessor()
        val gpsTow = 345_600.0
        // Stale-in-GPST record: BDT toe = gpsTow - 14 + 609 = 346,195 (609 s from gpsTow in GPST,
        // even though |345,600 - 346,195| = 595 s without the offset).
        val furtherInGpst =
            SatelliteEphemeris(
                constellationType = GnssStatus.CONSTELLATION_BEIDOU,
                svid = 6,
                toe = 346_195.0,
                sqrtA = 5282.6,
                lastUpdateTime = 0L,
            )
        // Closer-in-GPST record: BDT toe = gpsTow - 14 - 600 = 344,986 (600 s from gpsTow in GPST,
        // even though |345,600 - 344,986| = 614 s without the offset).
        val closerInGpst = furtherInGpst.copy(toe = 344_986.0, lastUpdateTime = 1L)

        assertEquals(1, processor.importExternalEphemeris(listOf(furtherInGpst), gpsTow))
        assertEquals(
            "BeiDou toe=344,986 (600 s old in GPST) must replace toe=346,195 (609 s away in GPST)",
            1,
            processor.importExternalEphemeris(listOf(closerInGpst), gpsTow),
        )
    }

    private fun putGlonassIcd(
        str: ByteArray,
        icdMsbBit: Int,
        length: Int,
        value: Long,
    ) {
        val flatStart = 85 - icdMsbBit
        GnssMathUtils.packBits(str, flatStart, length, (value and ((1L shl length) - 1)).toInt())
    }

    private fun putGlonassSignMag(
        str: ByteArray,
        icdMsbBit: Int,
        length: Int,
        raw: Long,
    ) {
        putGlonassIcd(str, icdMsbBit, 1, if (raw < 0) 1L else 0L)
        putGlonassIcd(str, icdMsbBit - 1, length - 1, abs(raw))
    }

    private fun glonassNavMessage(
        svid: Int,
        stringNumber: Int,
        data: ByteArray,
    ): GnssNavigationMessage {
        val ctor = GnssNavigationMessage::class.java.getDeclaredConstructor()
        ctor.isAccessible = true
        return ctor.newInstance().apply {
            val cls = GnssNavigationMessage::class.java
            cls.getMethod("setType", Int::class.javaPrimitiveType).invoke(this, GnssNavigationMessage.TYPE_GLO_L1CA)
            cls.getMethod("setSvid", Int::class.javaPrimitiveType).invoke(this, svid)
            cls.getMethod("setSubmessageId", Int::class.javaPrimitiveType).invoke(this, stringNumber)
            cls.getMethod("setMessageId", Int::class.javaPrimitiveType).invoke(this, 1)
            cls.getMethod("setData", ByteArray::class.java).invoke(this, data)
            cls.getMethod("setStatus", Int::class.javaPrimitiveType).invoke(this, GnssNavigationMessage.STATUS_PARITY_PASSED)
        }
    }

    private fun glonassString1(
        tkHours: Long,
        tkMinutes: Long,
    ): ByteArray =
        ByteArray(11).also { s ->
            putGlonassIcd(s, 84, 4, 1)
            putGlonassIcd(s, 76, 5, tkHours)
            putGlonassIcd(s, 71, 6, tkMinutes)
            putGlonassIcd(s, 65, 1, 0)
            putGlonassSignMag(s, 64, 24, -524_295L)
            putGlonassSignMag(s, 40, 5, -3L)
            putGlonassSignMag(s, 35, 27, 20_480_001L)
        }

    private fun glonassString2(tbIndex: Long): ByteArray =
        ByteArray(11).also { s ->
            putGlonassIcd(s, 84, 4, 2)
            putGlonassIcd(s, 80, 3, 0)
            putGlonassIcd(s, 76, 7, tbIndex)
            putGlonassSignMag(s, 64, 24, 2_831_155L)
            putGlonassSignMag(s, 40, 5, 0L)
            putGlonassSignMag(s, 35, 27, -30_720_003L)
        }

    private fun glonassString3(): ByteArray =
        ByteArray(11).also { s ->
            putGlonassIcd(s, 84, 4, 3)
            putGlonassSignMag(s, 79, 11, -5L)
            putGlonassIcd(s, 65, 1, 0)
            putGlonassSignMag(s, 64, 24, -1_992_301L)
            putGlonassSignMag(s, 40, 5, 1L)
            putGlonassSignMag(s, 35, 27, 36_864_005L)
        }

    private fun glonassString4(): ByteArray =
        ByteArray(11).also { s ->
            putGlonassIcd(s, 84, 4, 4)
            putGlonassSignMag(s, 80, 22, -107_374L)
        }

    @Test
    fun glonassNavigation_tbChangeOnString2EvictsPriorFrameStrings() {
        val processor = GnssMeasurementProcessor()

        // Cold-start before processMeasurements: Strings 1..4 arrive for tb = 36 (09:00 MSK)
        // with tk = 09:05 MSK (within 30 min of both tb = 36 and tb = 38).
        processor.processNavigationMessage(glonassNavMessage(5, 2, glonassString2(tbIndex = 36L)))
        processor.processNavigationMessage(glonassNavMessage(5, 1, glonassString1(tkHours = 9L, tkMinutes = 5L)))
        processor.processNavigationMessage(glonassNavMessage(5, 3, glonassString3()))
        processor.processNavigationMessage(glonassNavMessage(5, 4, glonassString4()))

        val initialEph = processor.getStoredEphemerisForTest(GnssStatus.CONSTELLATION_GLONASS, 5)
        assertNotNull("GLONASS ephemeris should be decoded on cold start before processMeasurements", initialEph)
        val initialTbTow = initialEph!!.gloTbGpsTow

        // String 2 arrives with new tb = 38 (09:30 MSK): old Strings 1/3/4 MUST be evicted
        // so the old frame cannot stitch with the new String 2.
        processor.processNavigationMessage(glonassNavMessage(5, 2, glonassString2(tbIndex = 38L)))
        val afterNewStr2Only = processor.getStoredEphemerisForTest(GnssStatus.CONSTELLATION_GLONASS, 5)!!
        assertEquals(
            "Updating String 2 with a new tb must evict old Strings 1/3/4 instead of stitching",
            initialTbTow,
            afterNewStr2Only.gloTbGpsTow,
            1e-9,
        )

        // Once the new frame's Strings 1, 3, 4 arrive, the new tb = 38 ephemeris is stored.
        processor.processNavigationMessage(glonassNavMessage(5, 1, glonassString1(tkHours = 9L, tkMinutes = 35L)))
        processor.processNavigationMessage(glonassNavMessage(5, 3, glonassString3()))
        processor.processNavigationMessage(glonassNavMessage(5, 4, glonassString4()))
        val updatedEph = processor.getStoredEphemerisForTest(GnssStatus.CONSTELLATION_GLONASS, 5)!!
        val expectedShift = 1800.0
        val actualShift = (updatedEph.gloTbGpsTow - initialTbTow + GnssMathUtils.SECONDS_IN_WEEK) % GnssMathUtils.SECONDS_IN_WEEK
        assertEquals(expectedShift, actualShift, 1e-6)
    }

    @Test
    fun glonassNavigation_evictsStringsOlderThan30Seconds() {
        val processor = GnssMeasurementProcessor()

        processor.processNavigationMessage(glonassNavMessage(8, 1, glonassString1(tkHours = 9L, tkMinutes = 5L)))
        processor.processNavigationMessage(glonassNavMessage(8, 2, glonassString2(tbIndex = 36L)))
        processor.processNavigationMessage(glonassNavMessage(8, 3, glonassString3()))

        // Advance elapsedRealtime past the 30 s frame window before String 4 arrives.
        ShadowSystemClock.advanceBy(Duration.ofSeconds(31))
        processor.processNavigationMessage(glonassNavMessage(8, 4, glonassString4()))

        assertEquals(
            "Strings older than 30 s must be evicted rather than stitched with String 4",
            null,
            processor.getStoredEphemerisForTest(GnssStatus.CONSTELLATION_GLONASS, 8),
        )
    }
}
