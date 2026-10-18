package app.simt.gnss

import android.location.GnssStatus
import app.simt.gnss.core.model.SatelliteEphemeris
import app.simt.gnss.core.orbit.SatellitePositionCalculator
import app.simt.gnss.core.utils.GnssMathUtils
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import kotlin.math.abs
import kotlin.math.pow

/**
 * Pins the group-delay (TGD/BGD) conventions fixed in the review:
 * - the ionosphere-free LNAV/BGD path must NOT adjust the codes and must pair with a clock
 *   that excludes the group delay (the per-signal biases cancel identically in the IF
 *   combination — previously +TGD·c on L5 plus the −TGD clock left ≈ −2.26·c·TGD per SV);
 * - the satellite clock exposes the group-delay term separately so single-frequency non-L1
 *   users can apply the γ-scaled value.
 */
@RunWith(RobolectricTestRunner::class)
class GnssGroupDelayConventionTest {
    private val processor = GnssMeasurementProcessor()
    private val tgdSeconds = 1.2e-8

    private fun ephemeris(constellation: Int) =
        SatelliteEphemeris(
            constellationType = constellation,
            svid = 7,
            toe = 345_600.0,
            toc = 345_600.0,
            af0 = 1.0e-4,
            sqrtA = 5153.7,
            eccentricity = 0.0, // → zero relativistic term, exact clock comparison below
            tgd = tgdSeconds,
            lastUpdateTime = 0L,
        )

    @Test
    fun ifCombination_lnavFallback_leavesCodesUnadjusted_andDropsClockGroupDelay() {
        val adjustment =
            processor.codePseudorangesForIonosphereFreeMeters(
                satKey = GnssStatus.CONSTELLATION_GPS to 7,
                l1Meters = 2.0e7,
                l5Meters = 2.0e7 + 3.0,
                ephemeris = ephemeris(GnssStatus.CONSTELLATION_GPS),
            )
        assertEquals("LNAV fallback must not add TGD to L1", 2.0e7, adjustment.l1Meters, 0.0)
        assertEquals("LNAV fallback must not add TGD to L5", 2.0e7 + 3.0, adjustment.l5Meters, 0.0)
        assertFalse(
            "IF observable pairs with the clock WITHOUT group delay on the LNAV path",
            adjustment.clockIncludesGroupDelay,
        )
    }

    @Test
    fun ifCombination_galileoBgdPath_sameConvention() {
        val adjustment =
            processor.codePseudorangesForIonosphereFreeMeters(
                satKey = GnssStatus.CONSTELLATION_GALILEO to 11,
                l1Meters = 2.4e7,
                l5Meters = 2.4e7 + 2.0,
                ephemeris = ephemeris(GnssStatus.CONSTELLATION_GALILEO),
            )
        assertEquals(2.4e7, adjustment.l1Meters, 0.0)
        assertEquals(2.4e7 + 2.0, adjustment.l5Meters, 0.0)
        assertFalse(adjustment.clockIncludesGroupDelay)
    }

    @Test
    fun ifCombination_beidou_keepsTgd1Clock() {
        val adjustment =
            processor.codePseudorangesForIonosphereFreeMeters(
                satKey = GnssStatus.CONSTELLATION_BEIDOU to 6,
                l1Meters = 2.2e7,
                l5Meters = 2.2e7 + 4.0,
                ephemeris = ephemeris(GnssStatus.CONSTELLATION_BEIDOU),
            )
        assertEquals(2.2e7, adjustment.l1Meters, 0.0)
        assertEquals(2.2e7 + 4.0, adjustment.l5Meters, 0.0)
        assertTrue("BDS keeps the B1I-referenced TGD1 clock convention", adjustment.clockIncludesGroupDelay)
    }

    @Test
    fun satelliteClock_groupDelayTermIsSeparable() {
        val eph = ephemeris(GnssStatus.CONSTELLATION_GPS)
        val time = 345_700.0
        val withTgd = SatellitePositionCalculator.getSatelliteClockCorrection(eph, time, includeGroupDelay = true)
        val withoutTgd = SatellitePositionCalculator.getSatelliteClockCorrection(eph, time, includeGroupDelay = false)
        assertEquals("clock(false) − clock(true) must equal TGD exactly", tgdSeconds, withoutTgd - withTgd, 1e-18)
        // Default behaviour unchanged for existing callers:
        assertEquals(
            withTgd,
            SatellitePositionCalculator.getSatelliteClockCorrection(eph, time),
            0.0,
        )
    }

    @Test
    fun galileoInavClock_convertsE5aAndIonosphereFreeOntoTheFnavReference() {
        val c = 299_792_458.0
        val mismatchMeters = 0.14
        val bgdA = 1.0e-9
        val bgdB = bgdA + mismatchMeters / c
        val inav =
            ephemeris(GnssStatus.CONSTELLATION_GALILEO).copy(
                tgd = bgdB,
                galileoBgdE1E5aSeconds = bgdA,
                galileoBgdE1E5bSeconds = bgdB,
                galileoInavClock = true,
            )
        val e1Hz = GnssMathUtils.L1_FREQUENCY_HZ
        val e5aHz = 1_176_450_000.0
        val gamma = (e1Hz / e5aHz).pow(2)

        val e1 = processor.galileoGroupDelaySeconds(inav, e1Hz, ionosphereFree = false)
        assertEquals(bgdB, e1, 1e-18)
        assertEquals(mismatchMeters, (e1 - bgdA) * c, 1e-6)

        val e5a = processor.galileoGroupDelaySeconds(inav, e5aHz, ionosphereFree = false)
        assertEquals(bgdB + (gamma - 1.0) * bgdA, e5a, 1e-15)
        val e5aErrorMeters = abs(e5a - gamma * bgdB) * c
        assertEquals(0.11, e5aErrorMeters, 0.01)

        val ionoFree = processor.galileoGroupDelaySeconds(inav, e1Hz, ionosphereFree = true)
        assertEquals(mismatchMeters, ionoFree * c, 1e-6)

        val fnav = inav.copy(galileoInavClock = false, tgd = bgdA)
        assertEquals(bgdA, processor.galileoGroupDelaySeconds(fnav, e1Hz, ionosphereFree = false), 1e-18)
        assertEquals(gamma * bgdA, processor.galileoGroupDelaySeconds(fnav, e5aHz, ionosphereFree = false), 1e-15)
        assertEquals(0.0, processor.galileoGroupDelaySeconds(fnav, e1Hz, ionosphereFree = true), 0.0)

        val legacy = ephemeris(GnssStatus.CONSTELLATION_GALILEO)
        assertEquals(tgdSeconds, processor.galileoGroupDelaySeconds(legacy, e1Hz, ionosphereFree = false), 1e-18)
        assertEquals(0.0, processor.galileoGroupDelaySeconds(legacy, e1Hz, ionosphereFree = true), 0.0)

        val gps = ephemeris(GnssStatus.CONSTELLATION_GPS)
        assertEquals(
            tgdSeconds * gamma,
            processor.groupDelaySecondsForMeasurement(
                gps,
                GnssStatus.CONSTELLATION_GPS,
                e5aHz,
                ionosphereFree = false,
                clockExcludesGroupDelay = false,
            ),
            1e-15,
        )
        assertEquals(
            0.0,
            processor.groupDelaySecondsForMeasurement(
                gps,
                GnssStatus.CONSTELLATION_GPS,
                e5aHz,
                ionosphereFree = true,
                clockExcludesGroupDelay = true,
            ),
            0.0,
        )
    }
}
