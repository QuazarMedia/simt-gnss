package app.simt.gnss.core.correction

import app.simt.gnss.core.utils.GnssMathUtils
import org.junit.Assert.*
import org.junit.Test

class AtmosphericCorrectionTest {
    @Test
    fun testComputeTroposphericDelay() {
        // Sea level, 90 deg elevation
        val delay90 = AtmosphericCorrection.computeTroposphericDelay(90.0, 0.0)
        // Standard Saastamoinen at sea level is ~2.3m
        assertTrue("Zenith tropo delay at sea level should be 2.0-2.5m, was $delay90", delay90 in 2.0..2.5)

        // Sea level, 10 deg elevation (should be much higher due to mapping function)
        val delay10 = AtmosphericCorrection.computeTroposphericDelay(10.0, 0.0)
        assertTrue("Low elevation tropo should be > 5x zenith", delay10 > delay90 * 5)
        assertTrue("Low elevation tropo should be < 30m", delay10 < 30.0)

        // High altitude, 90 deg elevation (should be lower than sea level)
        val delayHigh = AtmosphericCorrection.computeTroposphericDelay(90.0, 5000.0)
        assertTrue("High altitude tropo should be less than sea level", delayHigh < delay90)
    }

    @Test
    fun testComputeTroposphericDelayHandlesAbsurdAltitudeWithoutThrowing() {
        val absurdHigh = AtmosphericCorrection.computeTroposphericDelay(35.0, 4.972484658008615E7, 48.8)
        val absurdLow = AtmosphericCorrection.computeTroposphericDelay(35.0, -4.972484658008615E7, 48.8)

        assertTrue("Absurd high altitude tropo should stay finite", absurdHigh.isFinite())
        assertTrue("Absurd low altitude tropo should stay finite", absurdLow.isFinite())
        assertTrue("Absurd high altitude tropo should stay bounded", absurdHigh in 0.1..60.0)
        assertTrue("Absurd low altitude tropo should stay bounded", absurdLow in 0.1..60.0)
    }

    @Test
    fun testDryWetZenithDelaySeparation() {
        val alt = 0.0

        val dryZenith = AtmosphericCorrection.computeDryZenithDelay(alt)
        val wetZenith = AtmosphericCorrection.computeWetZenithDelay(alt)
        val totalDelay = AtmosphericCorrection.computeTroposphericDelay(90.0, alt)

        // Dry zenith delay at sea level should be ~2.3m (hydrostatic)
        assertTrue("Dry zenith delay should be 2.0-2.5m, was $dryZenith", dryZenith in 2.0..2.5)

        // Wet zenith delay at sea level should be ~0.05-0.15m
        assertTrue("Wet zenith delay should be > 0, was $wetZenith", wetZenith > 0)
        assertTrue("Wet zenith delay should be < 0.3m, was $wetZenith", wetZenith < 0.3)

        // At zenith (90 deg), mapping is ~1.0, so dry + wet ≈ total
        val summed = dryZenith + wetZenith
        assertEquals("Dry + wet zenith should ≈ total at 90deg", totalDelay, summed, 0.05)
    }

    @Test
    fun testNiellMappingDryWetDiffer() {
        // Dry and wet mapping should be different (they use different Niell coefficients)
        val (dryMap, wetMap) = AtmosphericCorrection.computeTroposphericMapping(30.0)
        assertNotEquals("Dry and wet mapping should differ", dryMap, wetMap, 1e-6)

        // Both should be > 1 for non-zenith elevation
        assertTrue("Dry mapping at 30deg should be > 1", dryMap > 1.0)
        assertTrue("Wet mapping at 30deg should be > 1", wetMap > 1.0)

        // At zenith, mapping should be ~1.0
        val (dryZenith, wetZenith) = AtmosphericCorrection.computeTroposphericMapping(90.0)
        assertEquals("Dry mapping at zenith should be ~1.0", 1.0, dryZenith, 0.01)
        assertEquals("Wet mapping at zenith should be ~1.0", 1.0, wetZenith, 0.01)

        // At very low elevation, mapping should be large
        val (dryLow, wetLow) = AtmosphericCorrection.computeTroposphericMapping(5.0)
        assertTrue("Dry mapping at 5deg should be > 5", dryLow > 5.0)
    }

    @Test
    fun niellDryMapping_isSeasonal_northAndSouthHemispheresOppositePhase() {
        // Mid-latitude 45°: day 28 of year → dry A/B/C use average coefficients (phase = cos(0) = 1 → correction − amp).
        // Relative to July (doy ≈ 210) in the northern hemisphere, mapping should differ.
        val elevation = 15.0 // non-zenith so seasonal amplitude actually matters
        val (dryJan, _) = AtmosphericCorrection.computeTroposphericMapping(elevation, 45.0, dayOfYear = 28)
        val (dryJul, _) = AtmosphericCorrection.computeTroposphericMapping(elevation, 45.0, dayOfYear = 210)
        assertNotEquals(
            "Niell dry mapping at NH 45° must differ between January and July",
            dryJan,
            dryJul,
            1e-6,
        )

        // Southern hemisphere should flip the seasonal offset by half a year.
        val (drySouthJan, _) = AtmosphericCorrection.computeTroposphericMapping(elevation, -45.0, dayOfYear = 28)
        val (drySouthJul, _) = AtmosphericCorrection.computeTroposphericMapping(elevation, -45.0, dayOfYear = 210)
        val nhSwing = dryJul - dryJan
        val shSwing = drySouthJul - drySouthJan
        assertTrue(
            "NH and SH seasonal swings must point in opposite directions ($nhSwing vs $shSwing)",
            nhSwing * shSwing < 0,
        )
    }

    @Test
    fun niellDryMapping_equatorHasMinimalSeasonalAmplitude() {
        // At latitude 15° (below the first Niell lat knot 15°), seasonal amplitude amp arrays are all zero,
        // so January vs July must produce an identical dry mapping value.
        val elevation = 15.0
        val (dryJanEq, _) = AtmosphericCorrection.computeTroposphericMapping(elevation, 10.0, dayOfYear = 28)
        val (dryJulEq, _) = AtmosphericCorrection.computeTroposphericMapping(elevation, 10.0, dayOfYear = 210)
        assertEquals("Seasonal amplitude at tropics should be negligible", dryJanEq, dryJulEq, 1e-9)
    }

    @Test
    fun niellDryMapping_missingDayOfYear_fallsBetweenSeasonalExtremes() {
        val elevation = 20.0
        val latitude = 45.0
        val (dryNoDoy, wetNoDoy) = AtmosphericCorrection.computeTroposphericMapping(elevation, latitude, dayOfYear = null)
        val (dryJan, wetJan) = AtmosphericCorrection.computeTroposphericMapping(elevation, latitude, dayOfYear = 28)
        val (dryJul, wetJul) = AtmosphericCorrection.computeTroposphericMapping(elevation, latitude, dayOfYear = 210)
        // Null doy → pure averages → dry mapping must sit strictly between the two phase extremes.
        val low = minOf(dryJan, dryJul)
        val high = maxOf(dryJan, dryJul)
        assertTrue(
            "Averages-only mapping ($dryNoDoy) must be between seasonal extremes [$low, $high]",
            dryNoDoy in low..high,
        )
        // Wet mapping has no seasonal term.
        assertEquals(wetNoDoy, wetJan, 1e-12)
        assertEquals(wetNoDoy, wetJul, 1e-12)
    }

    @Test
    fun niellDryMapping_zenithIsOneRegardlessOfSeason() {
        val (dryJanZenith, _) = AtmosphericCorrection.computeTroposphericMapping(90.0, 45.0, dayOfYear = 28)
        val (dryJulZenith, _) = AtmosphericCorrection.computeTroposphericMapping(90.0, 45.0, dayOfYear = 210)
        assertEquals(1.0, dryJanZenith, 1e-3)
        assertEquals(1.0, dryJulZenith, 1e-3)
    }

    @Test
    fun testComputeIonosphericDelayWithAzimuth() {
        val timestamp = 1737800000000L

        // Azimuth should matter: north vs south produce different IPP locations
        val delayNorth = AtmosphericCorrection.computeIonosphericDelay(45.0, 0.0, 45.0, 8.0, timestamp)
        val delaySouth = AtmosphericCorrection.computeIonosphericDelay(45.0, 180.0, 45.0, 8.0, timestamp)
        val delayEast = AtmosphericCorrection.computeIonosphericDelay(45.0, 90.0, 45.0, 8.0, timestamp)

        // All should be positive and reasonable
        assertTrue("North delay should be > 0", delayNorth > 0)
        assertTrue("South delay should be > 0", delaySouth > 0)
        assertTrue("East delay should be > 0", delayEast > 0)

        // North and south should produce different delays at mid-latitudes
        // (IPP shifts to higher vs lower latitude)
        assertNotEquals("North and South azimuths should give different delays", delayNorth, delaySouth, 0.01)
    }

    @Test
    fun testComputeIonosphericDelayBasic() {
        val timestamp = 1737800000000L

        // Test with default coefficients and azimuth = 90 (east)
        val delayDefault = AtmosphericCorrection.computeIonosphericDelay(45.0, 90.0, 45.0, 8.0, timestamp)
        assertTrue("Default iono delay should be > 0", delayDefault > 0)
        assertTrue("Default iono delay should be < 100m", delayDefault < 100.0)

        // Test with explicit coefficients
        val alpha = doubleArrayOf(0.1e-07, 0.1e-07, -0.5e-07, -0.1e-06)
        val beta = doubleArrayOf(0.1e+06, 0.1e+05, -0.1e+06, -0.4e+06)
        val delayCustom = AtmosphericCorrection.computeIonosphericDelay(45.0, 90.0, 45.0, 8.0, timestamp, alpha, beta)
        assertTrue("Custom iono delay should be > 0", delayCustom > 0)

        // Low elevation should have higher delay (oblique factor)
        val delayLow = AtmosphericCorrection.computeIonosphericDelay(10.0, 90.0, 45.0, 8.0, timestamp)
        assertTrue("Low elevation iono should be > high elevation", delayLow > delayDefault)
    }

    @Test
    fun testComputeRelativisticCorrection() {
        val rho = 20_200_000.0
        val rE = 6_378_137.0
        // Collinear ECEF (equator, +X): geocentric radii match textbook simplified geometry
        val corr =
            AtmosphericCorrection.computeRelativisticCorrection(
                receiverX = rE,
                receiverY = 0.0,
                receiverZ = 0.0,
                satelliteX = rE + rho,
                satelliteY = 0.0,
                satelliteZ = 0.0,
            )

        // Shapiro delay for typical GPS range should be ~1-3 cm
        assertTrue("Shapiro delay should be > 0.5cm (was $corr)", corr > 0.005)
        assertTrue("Shapiro delay should be < 5cm (was $corr)", corr < 0.05)
    }

    @Test
    fun testComputeSagnacCorrection() {
        // Typical receiver ECEF near Europe
        val receiverX = 4_201_000.0
        val receiverY = 172_000.0

        // Typical MEO satellite ECEF
        val satX = -15_600_000.0
        val satY = 20_180_000.0

        val corr = sagnacCorrectionMeters(receiverX, receiverY, satX, satY)

        // Typical GNSS Earth-rotation range correction should be on the order of meters to a few tens of meters
        assertTrue("Sagnac correction magnitude should be realistic (was $corr)", kotlin.math.abs(corr) in 1.0..60.0)

        // Swapping receiver and satellite X/Y terms should negate the correction
        val swapped = sagnacCorrectionMeters(satX, satY, receiverX, receiverY)
        assertEquals("Sagnac correction should be antisymmetric", -corr, swapped, 1e-6)
    }

    /** IS-GPS-200 first-order Sagnac term — kept test-side; production rotates satellite ECEF instead. */
    private fun sagnacCorrectionMeters(
        receiverX: Double,
        receiverY: Double,
        satelliteX: Double,
        satelliteY: Double,
    ): Double = (GnssMathUtils.OMEGA_EARTH / GnssMathUtils.SPEED_OF_LIGHT) * (satelliteX * receiverY - satelliteY * receiverX)
}
