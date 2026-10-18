package app.simt.gnss.core.correction

import app.simt.gnss.core.utils.GnssMathUtils
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.sqrt

class SsrCorrectionApplicatorTest {
    private val r = 26_560_000.0 // GPS orbit radius (m)

    /** Displacement (broadcast − corrected) produced by a unit delta on one RAC axis. */
    private fun axisDisplacement(
        pos: DoubleArray,
        velEcef: DoubleArray,
        dRadial: Double,
        dAlong: Double,
        dCross: Double,
    ): DoubleArray {
        val (cx, cy, cz) =
            SsrCorrectionApplicator.applyOrbitCorrection(
                pos[0],
                pos[1],
                pos[2],
                velEcef[0],
                velEcef[1],
                velEcef[2],
                dRadial,
                dAlong,
                dCross,
            )
        return doubleArrayOf(pos[0] - cx, pos[1] - cy, pos[2] - cz)
    }

    private fun dot(
        a: DoubleArray,
        b: DoubleArray,
    ) = a[0] * b[0] + a[1] * b[1] + a[2] * b[2]

    private fun norm(a: DoubleArray) = sqrt(dot(a, a))

    @Test
    fun alongTrackCorrection_followsEcefVelocity() {
        val pos = doubleArrayOf(r, 0.0, 0.0)
        val vEcef = doubleArrayOf(0.0, 0.0, 3000.0)
        val eA = axisDisplacement(pos, vEcef, 0.0, 1.0, 0.0)
        assertEquals(0.0, eA[0], 1e-7)
        assertEquals(0.0, eA[1], 1e-7)
        assertEquals(1.0, eA[2], 1e-7)
    }

    @Test
    fun nonRotatingVelocity_isTheGeoException() {
        // A GEO's ECEF velocity can be ~0. The non-rotating velocity adds ω⊕ × r,
        // which is what IGS SSR specifies for that case only.
        val pos = doubleArrayOf(r, 0.0, 0.0)
        val vEcef = doubleArrayOf(0.0, 0.0, 3000.0)
        val (cx, cy, cz) =
            SsrCorrectionApplicator.applyOrbitCorrection(
                pos[0],
                pos[1],
                pos[2],
                vEcef[0],
                vEcef[1],
                vEcef[2],
                deltaRadial = 0.0,
                deltaAlongTrack = 1.0,
                deltaCrossTrack = 0.0,
                nonRotatingVelocity = true,
            )
        val eA = doubleArrayOf(pos[0] - cx, pos[1] - cy, pos[2] - cz)
        val omegaCrossR = GnssMathUtils.OMEGA_EARTH * r
        val vInertial = doubleArrayOf(0.0, omegaCrossR, 3000.0)
        val vMag = norm(vInertial)
        assertEquals(vInertial[1] / vMag, eA[1], 1e-6)
        val ecef = axisDisplacement(pos, vEcef, 0.0, 1.0, 0.0)
        val frameDelta =
            sqrt((eA[0] - ecef[0]) * (eA[0] - ecef[0]) + (eA[1] - ecef[1]) * (eA[1] - ecef[1]) + (eA[2] - ecef[2]) * (eA[2] - ecef[2]))
        assertTrue(frameDelta > 0.4)
    }

    @Test
    fun ssrPolynomialAge_isMeasuredFromTheMiddleOfTheUpdateInterval() {
        val age = SsrCorrectionApplicator.ssrPolynomialAgeSeconds(100.0, 80.0, updateIntervalSeconds = 10.0)
        assertEquals(15.0, age, 1e-9)
        assertEquals(20.0, SsrCorrectionApplicator.ssrPolynomialAgeSeconds(100.0, 80.0, 0.0), 1e-9)
    }

    @Test
    fun radialCorrection_onCircularOrbit_pointsAlongPositionVector() {
        // Circular orbit: ECEF velocity ⊥ position, so the triad's radial axis equals r̂.
        val pos = doubleArrayOf(r, 0.0, 0.0)
        val vEcef = doubleArrayOf(0.0, 3000.0, 0.0)

        val eR = axisDisplacement(pos, vEcef, 1.0, 0.0, 0.0)
        assertEquals(1.0, eR[0], 1e-7)
        assertEquals(0.0, eR[1], 1e-7)
        assertEquals(0.0, eR[2], 1e-7)
    }

    @Test
    fun racAxes_formRightHandedOrthonormalTriad() {
        // Generic (slightly eccentric) geometry: position not ⊥ velocity.
        val pos = doubleArrayOf(15_000_000.0, 18_000_000.0, 12_000_000.0)
        val vEcef = doubleArrayOf(-1500.0, 1200.0, 3100.0)

        val eR = axisDisplacement(pos, vEcef, 1.0, 0.0, 0.0)
        val eA = axisDisplacement(pos, vEcef, 0.0, 1.0, 0.0)
        val eC = axisDisplacement(pos, vEcef, 0.0, 0.0, 1.0)

        assertEquals(1.0, norm(eR), 1e-7)
        assertEquals(1.0, norm(eA), 1e-7)
        assertEquals(1.0, norm(eC), 1e-7)
        assertEquals(0.0, dot(eR, eA), 1e-7)
        assertEquals(0.0, dot(eR, eC), 1e-7)
        assertEquals(0.0, dot(eA, eC), 1e-7)
        // Right-handed: eR × eA = eC? With eR = eA × eC, (eA × eC) × eA = eC ✓ — verify numerically.
        val cross =
            doubleArrayOf(
                eR[1] * eA[2] - eR[2] * eA[1],
                eR[2] * eA[0] - eR[0] * eA[2],
                eR[0] * eA[1] - eR[1] * eA[0],
            )
        assertEquals(eC[0], cross[0], 1e-7)
        assertEquals(eC[1], cross[1], 1e-7)
        assertEquals(eC[2], cross[2], 1e-7)
    }

    @Test
    fun clockCorrection_polynomialExtrapolation() {
        val corrected = SsrCorrectionApplicator.applyClockCorrection(10.0, c0 = 0.5, c1 = 0.01, c2 = -0.001, dtSsr = 2.0)
        assertEquals(10.0 + 0.5 + 0.02 - 0.004, corrected, 1e-12)
    }

    @Test
    fun wrapToHalfWeekSeconds_handlesWeekRollover() {
        // Correction at TOW 604,790, used at TOW 5 of the next week → true age +15 s.
        assertEquals(15.0, GnssMathUtils.wrapToHalfWeekSeconds(5.0 - 604_790.0), 1e-7)
        // Symmetric case.
        assertEquals(-15.0, GnssMathUtils.wrapToHalfWeekSeconds(604_790.0 - 5.0), 1e-7)
        // Small differences unchanged.
        assertEquals(42.0, GnssMathUtils.wrapToHalfWeekSeconds(42.0), 1e-7)
    }
}
