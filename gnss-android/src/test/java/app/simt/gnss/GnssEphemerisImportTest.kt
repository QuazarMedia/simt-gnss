package app.simt.gnss

import android.location.GnssStatus
import app.simt.gnss.core.model.SatelliteEphemeris
import org.junit.Assert.assertEquals
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner

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
}
