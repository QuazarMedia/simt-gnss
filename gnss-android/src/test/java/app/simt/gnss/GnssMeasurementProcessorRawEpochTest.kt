package app.simt.gnss

import android.location.GnssMeasurement
import android.location.GnssStatus
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner

/**
 * Tests of [GnssMeasurementProcessor.computeRawPseudorangeMeters] — pseudorange formation
 * from receiver/satellite times. Pins the per-constellation time-state gating and timescale
 * handling: GPST week modulo for GPS, BDT shift for BeiDou, and the GLONASS day modulo in
 * the UTC(SU)+3h timescale.
 */
@RunWith(RobolectricTestRunner::class)
class GnssMeasurementProcessorRawEpochTest {
    private val speedOfLight = 299_792_458.0
    private val processor = GnssMeasurementProcessor()
    private val leapSeconds = 18.0

    /** Receiver GPST: 4 days + 12 h into the week → GPST time-of-day = 43,200 s. */
    private val receiverGpstWholeNanos = 388_800L * 1_000_000_000L

    private fun pseudorange(
        constellation: Int,
        state: Int,
        receivedSvTimeNanos: Long,
    ): Double? =
        processor.computeRawPseudorangeMeters(
            constellationType = constellation,
            measurementState = state,
            receiveTimeWholeGpstNanos = receiverGpstWholeNanos,
            receiveTimeFractionNanos = 0.0,
            receivedSvTimeNanos = receivedSvTimeNanos,
            leapSeconds = leapSeconds,
        )

    @Test
    fun gps_withTowDecoded_formsPseudorangeFromWeekModulo() {
        // Transmit time = GPST TOW − 70 ms → pseudorange = 0.07 × c ≈ 20,985,472 m.
        val travelSeconds = 0.07
        val txNanos = ((388_800.0 - travelSeconds) * 1e9).toLong()
        val pr = pseudorange(GnssStatus.CONSTELLATION_GPS, GnssMeasurement.STATE_TOW_DECODED, txNanos)
        assertNotNull("GPS measurement with TOW_DECODED must survive", pr)
        assertEquals(travelSeconds * speedOfLight, pr!!, 1.0)
    }

    @Test
    fun glonass_withGloTodDecoded_formsPseudorangeInGlonassTimescale() {
        // GLONASS time-of-day at reception = GPST tod − leap + 3 h = 43,200 − 18 + 10,800
        // = 53,982 s. Transmit 75 ms earlier → pseudorange = 0.075 × c ≈ 22,484,434 m.
        // Before the fix this measurement was dropped twice over: the TOW-state gate never
        // accepted STATE_GLO_TOD_DECODED, and the missing timescale shift made the
        // pseudorange negative.
        val travelSeconds = 0.075
        val gloTodRxSeconds = 43_200.0 - leapSeconds + 10_800.0
        val txNanos = ((gloTodRxSeconds - travelSeconds) * 1e9).toLong()
        val pr = pseudorange(GnssStatus.CONSTELLATION_GLONASS, GnssMeasurement.STATE_GLO_TOD_DECODED, txNanos)
        assertNotNull("GLONASS measurement with GLO_TOD_DECODED must survive the state gate", pr)
        assertEquals(
            "GLONASS pseudorange must be formed in the UTC(SU)+3h day timescale",
            travelSeconds * speedOfLight,
            pr!!,
            1.0,
        )
    }

    @Test
    fun glonass_withGloTodKnown_alsoAccepted() {
        val travelSeconds = 0.075
        val txNanos = ((43_200.0 - leapSeconds + 10_800.0 - travelSeconds) * 1e9).toLong()
        val pr = pseudorange(GnssStatus.CONSTELLATION_GLONASS, GnssMeasurement.STATE_GLO_TOD_KNOWN, txNanos)
        assertNotNull(pr)
        assertEquals(travelSeconds * speedOfLight, pr!!, 1.0)
    }

    @Test
    fun glonass_dayRollover_wrapsCorrectly() {
        // Receiver just after GLONASS midnight, satellite transmit just before it:
        // GPST tod 13,218 s → GLONASS tod = 13,218 − 18 + 10,800 = 23,999.99... wait —
        // choose GPST so GLONASS tod = 30 ms past midnight: gpst tod = 18 − 10,800 + 86,400.03
        // → use explicit values: GLONASS tod_rx = 0.030 s, transmit at 86,399.955 s (previous day).
        val gpstNanos = ((4L * 86_400L + 18L - 10_800L + 86_400L) * 1_000_000_000L) + 30_000_000L // GLO tod = 0.030 s
        val txNanos = (86_399.955 * 1e9).toLong() // 75 ms travel across the day boundary
        val pr =
            processor.computeRawPseudorangeMeters(
                constellationType = GnssStatus.CONSTELLATION_GLONASS,
                measurementState = GnssMeasurement.STATE_GLO_TOD_DECODED,
                receiveTimeWholeGpstNanos = gpstNanos,
                receiveTimeFractionNanos = 0.0,
                receivedSvTimeNanos = txNanos,
                leapSeconds = leapSeconds,
            )
        assertNotNull("Day rollover must wrap, not cull, the pseudorange", pr)
        assertEquals(0.075 * speedOfLight, pr!!, 1.0)
    }

    @Test
    fun beidou_transmitTimeShiftedFromBdtToGpst() {
        // BDT lags GPST by 14 s: a BDS satellite transmitting 80 ms before reception reports
        // receivedSvTimeNanos = GPST TOW − 14 s − 0.08 s.
        val travelSeconds = 0.08
        val txNanos = ((388_800.0 - 14.0 - travelSeconds) * 1e9).toLong()
        val pr = pseudorange(GnssStatus.CONSTELLATION_BEIDOU, GnssMeasurement.STATE_TOW_DECODED, txNanos)
        assertNotNull(pr)
        assertEquals(travelSeconds * speedOfLight, pr!!, 1.0)
    }

    @Test
    fun glonass_withoutDecodedTime_isDropped() {
        // Code lock alone is not enough — receivedSvTimeNanos holds only a code-phase offset.
        assertNull(pseudorange(GnssStatus.CONSTELLATION_GLONASS, GnssMeasurement.STATE_CODE_LOCK, 1_000_000L))
    }

    @Test
    fun gps_withoutTow_isDropped() {
        assertNull(pseudorange(GnssStatus.CONSTELLATION_GPS, GnssMeasurement.STATE_CODE_LOCK, 500_000L))
    }

    @Test
    fun implausiblePseudorange_isCulled() {
        // Transmit time equal to receive time → zero range → outside the physical window.
        val txNanos = (388_800.0 * 1e9).toLong()
        assertNull(pseudorange(GnssStatus.CONSTELLATION_GPS, GnssMeasurement.STATE_TOW_DECODED, txNanos))
    }

    @Test
    fun fullScaleGpsTime_keepsSubMeterPrecision() {
        // A realistic full GPS time (~1.4e18 ns; week ~2370) with a 25 ns per-measurement
        // timeOffset. A Double-summed receive time quantizes onto a 256 ns ≈ 77 m grid at
        // this magnitude; the Long-modulo path must keep the pseudorange exact.
        val week = 2_370L
        val tow = 345_600L
        val wholeNanos = (week * 604_800L + tow) * 1_000_000_000L + 7L // odd ns to defeat round grids
        val fractionNanos = 25.0
        val travelNanos = 70_000_000L
        val txNanos = tow * 1_000_000_000L + 7L - travelNanos

        val pr =
            processor.computeRawPseudorangeMeters(
                constellationType = GnssStatus.CONSTELLATION_GPS,
                measurementState = GnssMeasurement.STATE_TOW_DECODED,
                receiveTimeWholeGpstNanos = wholeNanos,
                receiveTimeFractionNanos = fractionNanos,
                receivedSvTimeNanos = txNanos,
                leapSeconds = leapSeconds,
            )
        assertNotNull(pr)
        val expected = (travelNanos + fractionNanos) / 1e9 * speedOfLight
        assertEquals("pseudorange must be exact to the centimetre at full GPS time", expected, pr!!, 0.01)
    }
}
