package app.simt.gnss.core.utils

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.abs
import kotlin.math.hypot
import kotlin.math.pow

class GnssMathUtilsTest {
    @Test
    fun cn0WeightFactor_referenceCn0_isUnity() {
        val factor = GnssMathUtils.cn0WeightFactor(GnssMathUtils.CN0_REFERENCE_DB_HZ)
        assertEquals(1.0, factor, 1e-12)
    }

    @Test
    fun cn0WeightFactor_higherCn0_increasesWeight() {
        val low = GnssMathUtils.cn0WeightFactor(30.0)
        val high = GnssMathUtils.cn0WeightFactor(45.0)
        assertTrue("Higher C/N0 should increase measurement weight", high > low)
    }

    @Test
    fun cn0WeightFactor_matchesSpecifiedModel() {
        val factorAt20 = GnssMathUtils.cn0WeightFactor(20.0)
        val factorAt40 = GnssMathUtils.cn0WeightFactor(40.0)
        val factorAt50 = GnssMathUtils.cn0WeightFactor(50.0)

        assertEquals(0.01, factorAt20, 1e-12)
        assertEquals(1.0, factorAt40, 1e-12)
        assertEquals(10.0, factorAt50, 1e-9)
    }

    // --- Bit extraction tests ---

    private fun packUnsigned32(value: Long): ByteArray {
        val data = ByteArray(4)
        for (i in 0..3) data[i] = ((value shr (24 - 8 * i)) and 0xFF).toByte()
        return data
    }

    @Test
    fun getBitsLong_32bitUnsigned_extractsValueExceedingIntMax() {
        val raw = 2_702_549_965L // typical GPS sqrtA raw encoding (> Int.MAX_VALUE)
        val data = packUnsigned32(raw)

        assertEquals(raw, GnssMathUtils.getBitsLong(data, 0, 32))
    }

    @Test
    fun getBitsLong_32bitUnsigned_gpsSqrtA_producesCorrectOrbit() {
        // GPS semi-major axis ≈ 26,559,710 m → sqrtA ≈ 5153.6 m^(1/2)
        val expectedSqrtA = 5153.6
        val raw = (expectedSqrtA * 2.0.pow(19)).toLong()
        val data = packUnsigned32(raw)

        val sqrtA = GnssMathUtils.getBitsLong(data, 0, 32) * 2.0.pow(-19)
        val semiMajorAxis = sqrtA * sqrtA

        assertTrue(
            "Semi-major axis should be ~26,560 km, got ${semiMajorAxis / 1000} km",
            semiMajorAxis > 26_000_000 && semiMajorAxis < 27_000_000,
        )
    }

    @Test
    fun getSignedBitsLong_32bitSigned_negativeValue() {
        // m0 = -1.0 rad → raw = round(-1.0 / (2^-31 * π)) = -685,712,602
        val signedRaw = -685_712_602L
        val unsigned = signedRaw + (1L shl 32)
        val data = packUnsigned32(unsigned)

        val extracted = GnssMathUtils.getSignedBitsLong(data, 0, 32)
        assertEquals(signedRaw, extracted)

        val m0 = extracted * 2.0.pow(-31) * Math.PI
        assertTrue("m0 should be approximately -1.0 rad, got $m0", abs(m0 + 1.0) < 0.01)
    }

    @Test
    fun getSignedBitsLong_32bitSigned_positiveValue() {
        val signedRaw = 685_712_602L
        val data = packUnsigned32(signedRaw)

        val extracted = GnssMathUtils.getSignedBitsLong(data, 0, 32)
        assertEquals(signedRaw, extracted)
    }

    @Test
    fun getBits_sub32bit_remainsCorrect() {
        // 10-bit all-ones: 1023
        val data = byteArrayOf(0xFF.toByte(), 0xC0.toByte())

        assertEquals(1023, GnssMathUtils.getBits(data, 0, 10))
        assertEquals(1023L, GnssMathUtils.getBitsLong(data, 0, 10))
    }

    @Test
    fun getSignedBits_sub32bit_remainsCorrect() {
        // 16-bit signed: -1 (all 1s) = 0xFFFF
        val data = byteArrayOf(0xFF.toByte(), 0xFF.toByte())

        assertEquals(-1, GnssMathUtils.getSignedBits(data, 0, 16))
        assertEquals(-1L, GnssMathUtils.getSignedBitsLong(data, 0, 16))
    }

    @Test
    fun getBitsLong_withOffset_extractsCorrectly() {
        // 0x5A = 0b_0101_1010 → MSB-first bits: 0,1,0,1,1,0,1,0
        val data = byteArrayOf(0x5A, 0xFF.toByte())

        // Extract 4 bits starting at bit 2: bits [2,3,4,5] = 0,1,1,0 = 6
        assertEquals(6L, GnssMathUtils.getBitsLong(data, 2, 4))
        assertEquals(6, GnssMathUtils.getBits(data, 2, 4))
    }

    // -----------------------------------------------------------------------------------------
    //  Constellation time-system offsets
    // -----------------------------------------------------------------------------------------

    @Test
    fun bdtGpstOffset_equalsIcdValue() {
        // BDS epoch is 2006-01-01 UTC, at which point GPS time was 14 seconds ahead of UTC.
        // This 14 s offset is fixed by the BDS ICD.
        assertEquals(14.0, GnssMathUtils.BDT_GPST_OFFSET_SECONDS, 0.0)
    }

    // -----------------------------------------------------------------------------------------
    //  ECEF ↔ LLA round-trip: positioning accuracy directly depends on this conversion being
    //  invertible to within millimetres. Regressions here show up as systematic horizontal
    //  bias after the WLS solver converges.
    // -----------------------------------------------------------------------------------------

    @Test
    fun latLonAltEcef_roundTrip_matchesWithin1mm() {
        data class Point(val lat: Double, val lon: Double, val alt: Double, val label: String)
        val samples =
            listOf(
                Point(0.0, 0.0, 0.0, "equator prime meridian"),
                Point(48.8, 2.3, 100.0, "Paris"),
                Point(-33.86, 151.21, 58.0, "Sydney"),
                Point(64.14, -21.94, 7.0, "Reykjavík"),
                Point(89.9, 45.0, 3000.0, "near north pole"),
                Point(-89.9, -120.0, -50.0, "near south pole"),
            )
        for (p in samples) {
            val (x, y, z) = GnssMathUtils.latLonAltToEcef(p.lat, p.lon, p.alt)
            val (lat, lon, alt) = GnssMathUtils.ecefToLatLonAlt(x, y, z)
            assertEquals("${p.label}: lat", p.lat, lat, 1e-8)
            assertEquals("${p.label}: lon", p.lon, lon, 1e-8)
            // 0.001 m = 1 mm — well inside WLS convergence noise.
            assertEquals("${p.label}: alt", p.alt, alt, 0.001)
        }
    }

    @Test
    fun ecefLosToElevationAzimuth_satelliteDirectlyOverhead_returnsNinetyDegrees() {
        val (x, y, z) = GnssMathUtils.latLonAltToEcef(lat = 45.0, lon = 10.0, alt = 0.0)
        // Place the SV 20 000 km "above" the receiver along the geodetic up direction.
        val upFactor = 1.0 + 20_000_000.0 / kotlin.math.sqrt(x * x + y * y + z * z)
        val (elev, _) =
            GnssMathUtils.ecefLosToElevationAzimuth(
                receiverX = x,
                receiverY = y,
                receiverZ = z,
                receiverLatDeg = 45.0,
                receiverLonDeg = 10.0,
                satelliteX = x * upFactor,
                satelliteY = y * upFactor,
                satelliteZ = z * upFactor,
            )
        // Note: geodetic up ≠ geocentric up (due to WGS84 flattening), so strict 90° is only
        // exact at the equator / poles. Mid-latitude deviation is ≤ 0.2°.
        assertTrue(
            "Zenith elevation should be > 89° at 45°N, was $elev",
            elev > 89.0 && elev <= 90.0,
        )
    }

    @Test
    fun ecefLosToElevationAzimuth_satelliteToNorthAbove_returnsNorthAzimuthAndPositiveElevation() {
        // Receiver at equator on prime meridian; at lat=0 local up ≈ +X, north ≈ +Z, east ≈ +Y.
        // Put the SV 1 Mm up and 2 Mm north → elevation atan2(1, 2) ≈ 26.565°, azimuth 0 (due north).
        val rx = 6_378_137.0
        val (elev, az) =
            GnssMathUtils.ecefLosToElevationAzimuth(
                receiverX = rx,
                receiverY = 0.0,
                receiverZ = 0.0,
                receiverLatDeg = 0.0,
                receiverLonDeg = 0.0,
                satelliteX = rx + 1_000_000.0,
                satelliteY = 0.0,
                satelliteZ = 2_000_000.0,
            )
        assertEquals("Elevation should match atan2(1, 2) ≈ 26.565°", 26.565, elev, 0.1)
        assertTrue("Azimuth should point due north (≈0°), was $az", az < 0.5 || az > 359.5)
    }

    // --- P1-A: Pseudorange sigma from elevation + C/N0 ---

    @Test
    fun pseudorangeSigma_atZenithWithReferenceCn0_equalsSqrtAsquaredPlusBsquared() {
        // At 90° elevation, sin(E) = 1, so base variance = a² + b² (reference C/N0 keeps scale at 1)
        val a = GnssMathUtils.PSEUDORANGE_NOISE_ZENITH_A_METERS
        val b = GnssMathUtils.PSEUDORANGE_NOISE_ELEVATION_B_METERS
        val expected = kotlin.math.sqrt(a * a + b * b)
        val sigma = GnssMathUtils.pseudorangeSigmaFromElevationAndCn0(90.0, GnssMathUtils.CN0_REFERENCE_DB_HZ)
        assertEquals(expected, sigma, 1e-9)
    }

    @Test
    fun pseudorangeSigma_lowerElevationHasHigherSigma() {
        val sigmaHigh = GnssMathUtils.pseudorangeSigmaFromElevationAndCn0(60.0, 40.0)
        val sigmaMid = GnssMathUtils.pseudorangeSigmaFromElevationAndCn0(20.0, 40.0)
        val sigmaLow = GnssMathUtils.pseudorangeSigmaFromElevationAndCn0(5.0, 40.0)
        assertTrue("σ(20°) > σ(60°) required, got $sigmaMid and $sigmaHigh", sigmaMid > sigmaHigh)
        assertTrue("σ(5°) > σ(20°) required, got $sigmaLow and $sigmaMid", sigmaLow > sigmaMid)
    }

    @Test
    fun pseudorangeSigma_lowerCn0ScalesSigmaByCn0Factor() {
        // 10 dB below reference → variance scaled by 10 → σ scaled by sqrt(10).
        val sigmaRef = GnssMathUtils.pseudorangeSigmaFromElevationAndCn0(30.0, GnssMathUtils.CN0_REFERENCE_DB_HZ)
        val sigmaDown10 = GnssMathUtils.pseudorangeSigmaFromElevationAndCn0(30.0, GnssMathUtils.CN0_REFERENCE_DB_HZ - 10.0)
        assertEquals(sigmaRef * kotlin.math.sqrt(10.0), sigmaDown10, 1e-6)
    }

    @Test
    fun pseudorangeSigma_handlesExtremeElevationWithoutNaN() {
        val sigmaAboveZenith = GnssMathUtils.pseudorangeSigmaFromElevationAndCn0(180.0, 35.0)
        val sigmaBelowHorizon = GnssMathUtils.pseudorangeSigmaFromElevationAndCn0(-10.0, 35.0)
        assertTrue("Sigma must be finite above zenith", sigmaAboveZenith.isFinite() && sigmaAboveZenith > 0.0)
        assertTrue("Sigma must be finite below horizon", sigmaBelowHorizon.isFinite() && sigmaBelowHorizon > 0.0)
    }

    @Test
    fun raimStandardizedThreshold_isFourSigma() {
        // The normalized-residual RAIM test must stay at 4σ (guards against accidental relaxation).
        assertEquals(4.0, GnssMathUtils.RAIM_STANDARDIZED_RESIDUAL_SIGMA, 1e-12)
        assertTrue(
            "Leverage clamp must be strictly below 1",
            GnssMathUtils.RAIM_LEVERAGE_MAX in 0.9..0.999_999,
        )
    }

    @Test
    fun broadcastEphemerisRangeSigma_followsTheMeasuredAgingKnots() {
        assertEquals(0.0, GnssMathUtils.broadcastEphemerisRangeSigmaMeters(0.0), 1e-9)
        assertEquals(0.6, GnssMathUtils.broadcastEphemerisRangeSigmaMeters(2.0 * 3600.0), 1e-9)
        assertEquals(50.0, GnssMathUtils.broadcastEphemerisRangeSigmaMeters(4.0 * 3600.0), 1e-9)
        assertEquals(162.0, GnssMathUtils.broadcastEphemerisRangeSigmaMeters(6.0 * 3600.0), 1e-9)
        assertEquals(280.0, GnssMathUtils.broadcastEphemerisRangeSigmaMeters(12.0 * 3600.0), 1e-9)
        assertEquals(534.75, GnssMathUtils.broadcastEphemerisRangeSigmaMeters(24.0 * 3600.0), 1e-9)
        assertEquals(25.3, GnssMathUtils.broadcastEphemerisRangeSigmaMeters(3.0 * 3600.0), 1e-9)
        assertEquals(
            GnssMathUtils.broadcastEphemerisRangeSigmaMeters(4.0 * 3600.0),
            GnssMathUtils.broadcastEphemerisRangeSigmaMeters(-4.0 * 3600.0),
            0.0,
        )

        val fresh = GnssMathUtils.observationSigmaMeters(5.0, 0.0)
        val agedFourHours = GnssMathUtils.observationSigmaMeters(5.0, 4.0 * 3600.0)
        assertEquals(5.0, fresh, 1e-9)
        assertEquals(hypot(5.0, 50.0), agedFourHours, 1e-9)
        val weightRatio = (fresh / agedFourHours).pow(2)
        assertTrue("a 4 h ephemeris must not keep a fresh satellite's weight", weightRatio < 0.02)
    }

    @Test
    fun ecefDeltaHorizontalMeters_atEquator_isTheEastNorthMagnitude() {
        assertEquals(40.0, GnssMathUtils.ecefDeltaHorizontalMeters(0.0, 40.0, 0.0, 0.0, 0.0), 1e-6)
        assertEquals(0.0, GnssMathUtils.ecefDeltaHorizontalMeters(40.0, 0.0, 0.0, 0.0, 0.0), 1e-6)
    }
}
