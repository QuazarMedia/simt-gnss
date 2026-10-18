package app.simt.gnss.core.correction

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

/**
 * Compares [AtmosphericCorrection.computeIonosphericDelay] against an independent,
 * ICD-faithful reimplementation of the Klobuchar algorithm (IS-GPS-200 §20.3.3.5.2.5),
 * including the geomagnetic-latitude step (φ_m) the production code previously skipped.
 */
class KlobucharIcdReferenceTest {
    private val c = 299_792_458.0

    /** Independent IS-GPS-200 §20.3.3.5.2.5 implementation; all angles in semicircles. */
    private fun icdKlobucharMeters(
        elevationDeg: Double,
        azimuthDeg: Double,
        latDeg: Double,
        lonDeg: Double,
        gpsTodSeconds: Double,
        alpha: DoubleArray,
        beta: DoubleArray,
    ): Double {
        val e = Math.toRadians(elevationDeg).coerceAtLeast(0.05) / PI
        val azRad = Math.toRadians(azimuthDeg)
        val psi = 0.0137 / (e + 0.11) - 0.022
        val phiI = (latDeg / 180.0 + psi * cos(azRad)).coerceIn(-0.416, 0.416)
        val lamI = lonDeg / 180.0 + psi * sin(azRad) / cos(phiI * PI)
        val phiM = phiI + 0.064 * cos((lamI - 1.617) * PI)

        var t = 43_200.0 * lamI + gpsTodSeconds
        t %= 86_400.0
        if (t < 0) t += 86_400.0

        var amp = alpha[0] + alpha[1] * phiM + alpha[2] * phiM * phiM + alpha[3] * phiM * phiM * phiM
        if (amp < 0) amp = 0.0
        var per = beta[0] + beta[1] * phiM + beta[2] * phiM * phiM + beta[3] * phiM * phiM * phiM
        if (per < 72_000.0) per = 72_000.0

        val x = 2.0 * PI * (t - 50_400.0) / per
        val f = 1.0 + 16.0 * (0.53 - e) * (0.53 - e) * (0.53 - e)
        val tIono =
            if (abs(x) < 1.57) {
                5e-9 + amp * (1.0 - x * x / 2.0 + x * x * x * x / 24.0)
            } else {
                5e-9
            }
        return tIono * c * f
    }

    // Broadcast-like coefficients with strong latitude dependence, so a missing φ_m
    // step produces a clearly measurable error.
    private val alpha = doubleArrayOf(1.4e-8, 1.49e-8, -5.96e-8, -1.192e-7)
    private val beta = doubleArrayOf(1.43e5, 1.6384e4, -1.966e5, -4.588e5)

    private fun production(
        elevationDeg: Double,
        azimuthDeg: Double,
        latDeg: Double,
        lonDeg: Double,
        gpsTodSeconds: Long,
    ): Double =
        AtmosphericCorrection.computeIonosphericDelay(
            elevationDegrees = elevationDeg,
            azimuthDegrees = azimuthDeg,
            lat = latDeg,
            lon = lonDeg,
            timestamp = gpsTodSeconds * 1000L, // whole seconds → integer-division safe
            alpha = alpha,
            beta = beta,
        )

    @Test
    fun matchesIcdReference_acrossGeometries() {
        // Daytime epochs at varied elevation/azimuth/latitude/longitude, including
        // longitudes where cos((λ−1.617)π) ≈ ±1 (maximum geomagnetic shift).
        val cases =
            listOf(
                doubleArrayOf(20.0, 210.0, 40.0, -100.0, 75_600.0),
                doubleArrayOf(45.0, 90.0, 47.0, 8.0, 43_200.0),
                doubleArrayOf(10.0, 315.0, -33.0, 151.0, 50_400.0),
                doubleArrayOf(60.0, 180.0, 12.0, 77.0, 39_600.0),
                doubleArrayOf(35.0, 45.0, 64.0, -21.0, 61_200.0),
            )
        for (case in cases) {
            val (el, az, lat, lon, tod) = case
            val expected = icdKlobucharMeters(el, az, lat, lon, tod, alpha, beta)
            val actual = production(el, az, lat, lon, tod.toLong())
            assertEquals(
                "el=$el az=$az lat=$lat lon=$lon tod=$tod",
                expected,
                actual,
                1e-9 + expected * 1e-12,
            )
        }
    }

    @Test
    fun geomagneticLatitudeStep_changesDaytimeDelay() {
        // Two pierce points at the same geodetic latitude and same local time but at
        // longitudes with opposite geomagnetic shift must give different delays; an
        // implementation evaluating AMP/PER at the geodetic latitude returns equal values.
        val lonA = 1.617 * 180.0 // cos term = +1
        val lonB = lonA - 180.0 // cos term = −1
        // Equal local time t: t = 43200·λ + tod → compensate tod by the λ difference.
        val todA = 45_000.0
        val todB = todA + 43_200.0 * ((lonA - lonB) / 180.0)
        val a = production(45.0, 0.0, 30.0, lonA, todA.toLong())
        val b = production(45.0, 0.0, 30.0, lonB, todB.toLong())
        assertTrue(
            "Delays must differ once AMP/PER are evaluated at the geomagnetic latitude (a=$a b=$b)",
            abs(a - b) > 0.05,
        )
    }

    private operator fun DoubleArray.component4(): Double = this[3]

    private operator fun DoubleArray.component5(): Double = this[4]
}
