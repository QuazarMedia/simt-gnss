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
    fun ifCombination_beidouB1iB2i_removesTgd1AndTgd2_matchesB3iClock() {
        val c = 299_792_458.0
        val tgd1 = 1.2e-8
        val tgd2 = -0.8e-8
        val eph = ephemeris(GnssStatus.CONSTELLATION_BEIDOU).copy(tgd = tgd1, beidouTgd2Seconds = tgd2)
        val fB1i = GnssMathUtils.B1I_FREQUENCY_HZ
        val fB2i = GnssMathUtils.B2I_FREQUENCY_HZ
        val b3iClockRange = 22_000_000.0
        // Observed B1I and B2I pseudoranges carry B3I-referenced TGD1/TGD2 plus 1/f² ionosphere.
        val ionoB1i = 12.0
        val ionoB2i = ionoB1i * (fB1i / fB2i).pow(2)
        val pB1i = b3iClockRange + c * tgd1 + ionoB1i
        val pB2i = b3iClockRange + c * tgd2 + ionoB2i

        val adjustment =
            processor.codePseudorangesForIonosphereFreeMeters(
                satKey = GnssStatus.CONSTELLATION_BEIDOU to 6,
                l1Meters = pB1i,
                l5Meters = pB2i,
                ephemeris = eph,
                upperBandHz = fB1i,
                lowerBandHz = fB2i,
            )
        assertEquals(pB1i - c * tgd1, adjustment.l1Meters, 1e-9)
        assertEquals(pB2i - c * tgd2, adjustment.l5Meters, 1e-9)
        assertFalse("B1I+B2I IF aligns observables to B3I clock (no extra TGD)", adjustment.clockIncludesGroupDelay)

        val pIf = (fB1i * fB1i * adjustment.l1Meters - fB2i * fB2i * adjustment.l5Meters) / (fB1i * fB1i - fB2i * fB2i)
        assertEquals("IF combination must recover B3I clock range with zero residual bias", b3iClockRange, pIf, 1e-6)
    }

    @Test
    fun ifCombination_beidouB1iB2a_andB1cB2a_andSingleFreq_applyB3iConventions() {
        val c = 299_792_458.0
        val tgd1 = 1.5e-8
        val tgd2 = -0.7e-8
        val eph = ephemeris(GnssStatus.CONSTELLATION_BEIDOU).copy(tgd = tgd1, beidouTgd2Seconds = tgd2)

        // B1I + B2a: removes c*TGD1 from B1I only, 0 from B2a -> 2.313*c*TGD1 cancels cleanly.
        val fB1i = GnssMathUtils.B1I_FREQUENCY_HZ
        val fB2a = GnssMathUtils.L5_FREQUENCY_HZ
        val b3iRange = 21_500_000.0
        val ionoB1i = 9.0
        val ionoB2a = ionoB1i * (fB1i / fB2a).pow(2)
        val b1iB2aAdj =
            processor.codePseudorangesForIonosphereFreeMeters(
                satKey = GnssStatus.CONSTELLATION_BEIDOU to 19,
                l1Meters = b3iRange + c * tgd1 + ionoB1i,
                l5Meters = b3iRange + ionoB2a,
                ephemeris = eph,
                upperBandHz = fB1i,
                lowerBandHz = fB2a,
            )
        assertFalse(b1iB2aAdj.clockIncludesGroupDelay)
        val pIfB1iB2a = (fB1i * fB1i * b1iB2aAdj.l1Meters - fB2a * fB2a * b1iB2aAdj.l5Meters) / (fB1i * fB1i - fB2a * fB2a)
        assertEquals(b3iRange, pIfB1iB2a, 1e-6)

        // B1C + B2a: D1 TGD1/TGD2 do NOT apply to B1C or B2a.
        val b1cB2aAdj =
            processor.codePseudorangesForIonosphereFreeMeters(
                satKey = GnssStatus.CONSTELLATION_BEIDOU to 20,
                l1Meters = 2.2e7,
                l5Meters = 2.2e7 + 4.0,
                ephemeris = eph,
                upperBandHz = GnssMathUtils.L1_FREQUENCY_HZ,
                lowerBandHz = fB2a,
            )
        assertEquals(2.2e7, b1cB2aAdj.l1Meters, 0.0)
        assertEquals(2.2e7 + 4.0, b1cB2aAdj.l5Meters, 0.0)
        assertFalse(b1cB2aAdj.clockIncludesGroupDelay)

        // Single-frequency BeiDou bands: B1I -> TGD1, B2I -> TGD2, B3I/B1C/B2a -> 0.0
        assertEquals(
            tgd1,
            processor.groupDelaySecondsForMeasurement(eph, GnssStatus.CONSTELLATION_BEIDOU, fB1i, ionosphereFree = false, clockExcludesGroupDelay = false),
            1e-18,
        )
        assertEquals(
            tgd2,
            processor.groupDelaySecondsForMeasurement(eph, GnssStatus.CONSTELLATION_BEIDOU, GnssMathUtils.B2I_FREQUENCY_HZ, ionosphereFree = false, clockExcludesGroupDelay = false),
            1e-18,
        )
        assertEquals(
            0.0,
            processor.groupDelaySecondsForMeasurement(eph, GnssStatus.CONSTELLATION_BEIDOU, GnssMathUtils.B3I_FREQUENCY_HZ, ionosphereFree = false, clockExcludesGroupDelay = false),
            0.0,
        )
        assertEquals(
            0.0,
            processor.groupDelaySecondsForMeasurement(eph, GnssStatus.CONSTELLATION_BEIDOU, GnssMathUtils.L1_FREQUENCY_HZ, ionosphereFree = false, clockExcludesGroupDelay = false),
            0.0,
        )
        assertEquals(
            0.0,
            processor.groupDelaySecondsForMeasurement(eph, GnssStatus.CONSTELLATION_BEIDOU, fB2a, ionosphereFree = false, clockExcludesGroupDelay = false),
            0.0,
        )
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

    @Test
    fun galileoE5b_singleFrequencyAndE1E5bIf_followIcd515() {
        val bgdA = 1.0e-9
        val bgdB = 2.4e-9
        val inav =
            ephemeris(GnssStatus.CONSTELLATION_GALILEO).copy(
                tgd = bgdB,
                galileoBgdE1E5aSeconds = bgdA,
                galileoBgdE1E5bSeconds = bgdB,
                galileoInavClock = true,
            )
        val fnav = inav.copy(galileoInavClock = false, tgd = bgdA)
        val e1Hz = GnssMathUtils.L1_FREQUENCY_HZ
        val e5aHz = GnssMathUtils.L5_FREQUENCY_HZ
        val e5bHz = GnssMathUtils.E5B_FREQUENCY_HZ
        val gammaE5b = (e1Hz / e5bHz).pow(2)

        // Single-frequency E5b
        assertEquals(gammaE5b * bgdB, processor.galileoGroupDelaySeconds(inav, e5bHz, ionosphereFree = false), 1e-15)
        assertEquals(bgdA + (gammaE5b - 1.0) * bgdB, processor.galileoGroupDelaySeconds(fnav, e5bHz, ionosphereFree = false), 1e-15)

        // E1 + E5b IF with I/NAV clock: I/NAV clock is E1/E5b IF reference -> 0.0 bias, clockIncludesGroupDelay = false
        val e1E5bInavAdj =
            processor.codePseudorangesForIonosphereFreeMeters(
                satKey = GnssStatus.CONSTELLATION_GALILEO to 11,
                l1Meters = 2.4e7,
                l5Meters = 2.4e7 + 2.0,
                ephemeris = inav,
                upperBandHz = e1Hz,
                lowerBandHz = e5bHz,
            )
        assertFalse(e1E5bInavAdj.clockIncludesGroupDelay)
        assertEquals(0.0, processor.galileoGroupDelaySeconds(inav, e1Hz, ionosphereFree = true, lowerBandHz = e5bHz), 0.0)
        assertEquals(
            0.0,
            processor.groupDelaySecondsForMeasurement(
                inav,
                GnssStatus.CONSTELLATION_GALILEO,
                e1Hz,
                ionosphereFree = true,
                clockExcludesGroupDelay = !e1E5bInavAdj.clockIncludesGroupDelay,
                lowerBandHz = e5bHz,
            ),
            0.0,
        )

        // E1 + E5a IF with I/NAV clock: still adjusts by bgdB - bgdA
        val e1E5aInavAdj =
            processor.codePseudorangesForIonosphereFreeMeters(
                satKey = GnssStatus.CONSTELLATION_GALILEO to 11,
                l1Meters = 2.4e7,
                l5Meters = 2.4e7 + 2.0,
                ephemeris = inav,
                upperBandHz = e1Hz,
                lowerBandHz = e5aHz,
            )
        assertTrue(e1E5aInavAdj.clockIncludesGroupDelay)
        assertEquals(bgdB - bgdA, processor.galileoGroupDelaySeconds(inav, e1Hz, ionosphereFree = true, lowerBandHz = e5aHz), 1e-18)
        assertEquals(
            bgdB - bgdA,
            processor.groupDelaySecondsForMeasurement(
                inav,
                GnssStatus.CONSTELLATION_GALILEO,
                e1Hz,
                ionosphereFree = true,
                clockExcludesGroupDelay = !e1E5aInavAdj.clockIncludesGroupDelay,
                lowerBandHz = e5aHz,
            ),
            1e-18,
        )
    }

    @Test
    fun gpsCnavMt30Isc_appliesToL1L5Only_andLeavesL1L2Unadjusted() {
        val cnavPayload = ByteArray(40)
        GnssMathUtils.packBits(cnavPayload, 14, 6, 30)
        val iscL1Raw = 100 // +100 * 2^-35 s
        val iscL5Raw = -160 // -160 * 2^-35 s
        GnssMathUtils.packBits(cnavPayload, 140, 13, iscL1Raw and 0x1FFF)
        GnssMathUtils.packBits(cnavPayload, 166, 13, iscL5Raw and 0x1FFF)

        processor.recordGpsCnavMt30Isc(svid = 7, data = cnavPayload)

        val gpsEph = ephemeris(GnssStatus.CONSTELLATION_GPS)
        val c = GnssMathUtils.SPEED_OF_LIGHT
        val expectedL1IscMeters = iscL1Raw * 2.0.pow(-35) * c
        val expectedL5IscMeters = iscL5Raw * 2.0.pow(-35) * c

        // L1 + L5 applies ISC_L1CA and ISC_L5I5 with clockIncludesGroupDelay = true
        val l1L5Adj =
            processor.codePseudorangesForIonosphereFreeMeters(
                satKey = GnssStatus.CONSTELLATION_GPS to 7,
                l1Meters = 2.0e7,
                l5Meters = 2.0e7 + 3.0,
                ephemeris = gpsEph,
                upperBandHz = GnssMathUtils.L1_FREQUENCY_HZ,
                lowerBandHz = GnssMathUtils.L5_FREQUENCY_HZ,
            )
        assertEquals(2.0e7 + expectedL1IscMeters, l1L5Adj.l1Meters, 1e-9)
        assertEquals(2.0e7 + 3.0 + expectedL5IscMeters, l1L5Adj.l5Meters, 1e-9)
        assertTrue(l1L5Adj.clockIncludesGroupDelay)

        // L1 + L2 (1227.60 MHz) must NOT apply ISC_L5I5 or ISC_L1CA; TGD cancels in IS-GPS-200 L1/L2 IF
        val l1L2Adj =
            processor.codePseudorangesForIonosphereFreeMeters(
                satKey = GnssStatus.CONSTELLATION_GPS to 7,
                l1Meters = 2.0e7,
                l5Meters = 2.0e7 + 5.0,
                ephemeris = gpsEph,
                upperBandHz = GnssMathUtils.L1_FREQUENCY_HZ,
                lowerBandHz = 1_227_600_000.0,
            )
        assertEquals(2.0e7, l1L2Adj.l1Meters, 0.0)
        assertEquals(2.0e7 + 5.0, l1L2Adj.l5Meters, 0.0)
        assertFalse(l1L2Adj.clockIncludesGroupDelay)
    }
}
