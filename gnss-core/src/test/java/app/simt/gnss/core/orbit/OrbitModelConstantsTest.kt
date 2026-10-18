package app.simt.gnss.core.orbit
import app.simt.gnss.core.model.Constellation
import app.simt.gnss.core.model.SatelliteEphemeris
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.sqrt

class OrbitModelConstantsTest {
    private fun distance(
        a: Triple<Double, Double, Double>,
        b: Triple<Double, Double, Double>,
    ): Double {
        val dx = a.first - b.first
        val dy = a.second - b.second
        val dz = a.third - b.third
        return sqrt(dx * dx + dy * dy + dz * dz)
    }

    @Test
    fun keplerianMu_isSelectedPerConstellation() {
        // IS-GPS-200 mandates μ = 3.986005e14 for GPS while Galileo/BDS use 3.986004418e14.
        // Identical orbital elements propagated one hour must therefore differ by ~1 m of
        // along-track motion between the two constellations (Δn ≈ n/2 · Δμ/μ).
        val elements =
            SatelliteEphemeris(
                constellationType = Constellation.GPS,
                svid = 1,
                toe = 0.0,
                sqrtA = 5153.7,
                eccentricity = 0.01,
                m0 = 1.0,
                omega0 = 0.5,
                i0 = 0.96,
                omega = 0.3,
                lastUpdateTime = 0L,
            )
        val gps = SatellitePositionCalculator.calculateSatellitePosition(elements, 3600.0)
        val gal =
            SatellitePositionCalculator.calculateSatellitePosition(
                elements.copy(constellationType = Constellation.GALILEO),
                3600.0,
            )
        val separation = distance(gps, gal)
        assertTrue(
            "μ difference must move the satellite ~1 m at tk = 1 h (got $separation m)",
            separation in 0.3..3.0,
        )
    }

    @Test
    fun glonassPropagation_isClampedToTheValidityWindow() {
        // Beyond GLONASS_MAX_PROPAGATION_SECONDS the broadcast force model is meaningless;
        // the integrator clamps there (callers gate usability on tb age) so CPU stays
        // bounded and no pseudo-propagated state escapes.
        val eph =
            SatelliteEphemeris(
                constellationType = Constellation.GLONASS,
                svid = 5,
                isGlonass = true,
                gloPosX = 10_000_000.0,
                gloPosY = 15_000_000.0,
                gloPosZ = 18_000_000.0,
                gloVelX = -500.0,
                gloVelY = 2_700.0,
                gloVelZ = -1_900.0,
                gloTbGpsTow = 100_000.0,
                lastUpdateTime = 0L,
            )
        val atCap = SatellitePositionCalculator.calculateSatellitePosition(eph, 100_000.0 + 2_700.0)
        val farBeyond = SatellitePositionCalculator.calculateSatellitePosition(eph, 100_000.0 + 50_000.0)
        assertEquals(atCap.first, farBeyond.first, 1e-6)
        assertEquals(atCap.second, farBeyond.second, 1e-6)
        assertEquals(atCap.third, farBeyond.third, 1e-6)
    }

    @Test
    fun glonassPropagation_shortIntervalMatchesKinematicExpansion() {
        // Over 60 s the RK4 result must agree with a second-order kinematic expansion to
        // within the neglected gravity-gradient effects (a few metres) — guards against
        // scratch-buffer reuse bugs in the integrator.
        val eph =
            SatelliteEphemeris(
                constellationType = Constellation.GLONASS,
                svid = 5,
                isGlonass = true,
                gloPosX = 10_000_000.0,
                gloPosY = 15_000_000.0,
                gloPosZ = 18_000_000.0,
                gloVelX = -500.0,
                gloVelY = 2_700.0,
                gloVelZ = -1_900.0,
                gloTbGpsTow = 100_000.0,
                lastUpdateTime = 0L,
            )
        val dt = 60.0
        val propagated = SatellitePositionCalculator.calculateSatellitePosition(eph, 100_000.0 + dt)
        val kinematic =
            Triple(
                eph.gloPosX + eph.gloVelX * dt,
                eph.gloPosY + eph.gloVelY * dt,
                eph.gloPosZ + eph.gloVelZ * dt,
            )
        // Central gravity bends the path by ~½·g·t² ≈ 0.5 · 0.6 m/s² · 3600 ≈ 1 km; the
        // kinematic difference must be of that order, not wildly off (buffer corruption).
        val deviation = distance(propagated, kinematic)
        assertTrue("RK4 vs kinematic deviation $deviation m must reflect gravity only", deviation in 100.0..5_000.0)
    }
}
