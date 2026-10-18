package app.simt.gnss.core.correction

import app.simt.gnss.core.utils.GnssMathUtils.MU
import app.simt.gnss.core.utils.GnssMathUtils.SPEED_OF_LIGHT
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.ln
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Models for atmospheric delays (Troposphere, Ionosphere).
 *
 * Tropospheric mapping uses the Niell (1996) hydrostatic and wet mapping functions with
 * **latitude-dependent** coefficients (linear interpolation between 15° and 75° |φ|),
 * per Niell and standard GNSS texts (e.g. Springer Handbook of GNSS, Table 9.2–9.3).
 */
object AtmosphericCorrection {
    private const val MAX_MAPPING = 20.0
    private const val MIN_TROPOSPHERIC_ALTITUDE_METERS = -500.0
    private const val MAX_TROPOSPHERIC_ALTITUDE_METERS = 20_000.0
    private const val DEFAULT_RECEIVER_LATITUDE_DEGREES = 45.0
    private const val DEFAULT_RECEIVER_LONGITUDE_DEGREES = 0.0
    private const val DEFAULT_ELEVATION_DEGREES = 90.0

    /** Niell latitude nodes (degrees, Northern table; Southern uses |φ|). */
    private val NIELL_LATITUDE_NODES = doubleArrayOf(15.0, 30.0, 45.0, 60.0, 75.0)

    /**
     * Hydrostatic (dry) mapping coefficients a_h — *average* values (Niell 1996 Table 9.2).
     * Seasonal amplitude is added via [NIELL_DRY_A_AMP] / [NIELL_DRY_B_AMP] / [NIELL_DRY_C_AMP]
     * and the DoY phase factor `cos(2π(DoY − 28)/365.25)` (Southern hemisphere uses +183 days).
     */
    private val NIELL_DRY_A_AVG =
        doubleArrayOf(
            1.2769934e-3,
            1.2683230e-3,
            1.2465397e-3,
            1.2196049e-3,
            1.2045996e-3,
        )

    private val NIELL_DRY_B_AVG =
        doubleArrayOf(
            2.9153695e-3,
            2.9152299e-3,
            2.9288445e-3,
            2.9022565e-3,
            2.9024912e-3,
        )

    private val NIELL_DRY_C_AVG =
        doubleArrayOf(
            62.610505e-3,
            62.837393e-3,
            63.721774e-3,
            63.824265e-3,
            64.258455e-3,
        )

    /** Hydrostatic (dry) seasonal amplitude coefficients a_h_amp (Niell 1996 Table 9.2). */
    private val NIELL_DRY_A_AMP =
        doubleArrayOf(
            0.0,
            1.2709626e-5,
            2.6523662e-5,
            3.4000452e-5,
            4.1202191e-5,
        )

    private val NIELL_DRY_B_AMP =
        doubleArrayOf(
            0.0,
            2.1414979e-5,
            3.0160779e-5,
            7.2562722e-5,
            11.723375e-5,
        )

    private val NIELL_DRY_C_AMP =
        doubleArrayOf(
            0.0,
            9.0128400e-5,
            4.3497037e-5,
            84.795348e-5,
            170.37206e-5,
        )

    /** Wet mapping coefficients a_w, b_w, c_w (Niell 1996 wet, handbook Table 9.3 style). */
    private val NIELL_WET_A =
        doubleArrayOf(
            5.75717e-4,
            5.67921e-4,
            5.80219e-4,
            5.67921e-4,
            5.75717e-4,
        )

    private val NIELL_WET_B =
        doubleArrayOf(
            1.023462e-3,
            1.540905e-3,
            1.4275268e-3,
            1.540905e-3,
            1.023462e-3,
        )

    private val NIELL_WET_C =
        doubleArrayOf(
            5.163468e-2,
            4.6946e-2,
            4.3472961e-2,
            4.6946e-2,
            5.163468e-2,
        )

    /**
     * Linear interpolation of [values] at |latitudeDegrees|, clamped to [15°, 75°].
     */
    private fun interpolateNiellCoefficient(
        latitudeDegrees: Double,
        values: DoubleArray,
    ): Double {
        val x = abs(sanitizeLatitudeDegrees(latitudeDegrees)).coerceIn(15.0, 75.0)
        val nodes = NIELL_LATITUDE_NODES
        if (x <= nodes[0]) return values[0]
        if (x >= nodes[4]) return values[4]
        var i = 0
        while (i < 4 && x > nodes[i + 1]) {
            i++
        }
        val t = (x - nodes[i]) / (nodes[i + 1] - nodes[i])
        return values[i] + t * (values[i + 1] - values[i])
    }

    private fun sanitizeLatitudeDegrees(latitudeDegrees: Double): Double =
        if (latitudeDegrees.isFinite()) {
            latitudeDegrees.coerceIn(-90.0, 90.0)
        } else {
            DEFAULT_RECEIVER_LATITUDE_DEGREES
        }

    private fun sanitizeLongitudeDegrees(longitudeDegrees: Double): Double {
        val rawLongitude = if (longitudeDegrees.isFinite()) longitudeDegrees else DEFAULT_RECEIVER_LONGITUDE_DEGREES
        val wrappedLongitude = ((rawLongitude + 180.0) % 360.0 + 360.0) % 360.0 - 180.0
        return if (wrappedLongitude == -180.0) 180.0 else wrappedLongitude
    }

    private fun sanitizeAltitudeMeters(altitudeMeters: Double): Double =
        if (altitudeMeters.isFinite()) {
            altitudeMeters.coerceIn(MIN_TROPOSPHERIC_ALTITUDE_METERS, MAX_TROPOSPHERIC_ALTITUDE_METERS)
        } else {
            0.0
        }

    private fun sanitizeElevationDegrees(elevationDegrees: Double): Double =
        if (elevationDegrees.isFinite()) {
            elevationDegrees.coerceIn(0.0, DEFAULT_ELEVATION_DEGREES)
        } else {
            DEFAULT_ELEVATION_DEGREES
        }

    /**
     * Niell continued-fraction mapping function: m(el) = (1 + a/(1+b/(1+c))) / (sin(el) + a/(sin(el)+b/(sin(el)+c)))
     */
    private fun niellMapping(
        sinElev: Double,
        a: Double,
        b: Double,
        c: Double,
    ): Double {
        val num = 1.0 + a / (1.0 + b / (1.0 + c))
        val den = sinElev + a / (sinElev + b / (sinElev + c))
        return num / den
    }

    /**
     * Calculates the tropospheric mapping function components using Niell (1996) with
     * **latitude-interpolated** dry and wet coefficients (|latitude| 15°–75°).
     *
     * When [dayOfYear] is non-null, the hydrostatic (dry) coefficients receive the Niell 1996
     * seasonal amplitude correction:
     *   a_h(φ, DoY) = a_avg(φ) − a_amp(φ) · cos(2π · (DoY − DoY₀) / 365.25)
     * with DoY₀ = 28 in the Northern hemisphere and DoY₀ = 28 + 182.625 in the Southern
     * hemisphere (half a year offset). Wet mapping has no seasonal term in Niell 1996.
     *
     * Returns Pair(dryMapping, wetMapping).
     */
    fun computeTroposphericMapping(
        elevationDegrees: Double,
        latitudeDegrees: Double = 45.0,
        dayOfYear: Int? = null,
    ): Pair<Double, Double> {
        val elevRad = Math.toRadians(sanitizeElevationDegrees(elevationDegrees))
        if (elevRad < 0.01) return MAX_MAPPING to MAX_MAPPING

        val sinElev = sin(elevRad)
        val safeLatitude = sanitizeLatitudeDegrees(latitudeDegrees)

        val dryAAvg = interpolateNiellCoefficient(safeLatitude, NIELL_DRY_A_AVG)
        val dryBAvg = interpolateNiellCoefficient(safeLatitude, NIELL_DRY_B_AVG)
        val dryCAvg = interpolateNiellCoefficient(safeLatitude, NIELL_DRY_C_AVG)

        val (dryA, dryB, dryC) =
            if (dayOfYear != null) {
                val dryAAmp = interpolateNiellCoefficient(safeLatitude, NIELL_DRY_A_AMP)
                val dryBAmp = interpolateNiellCoefficient(safeLatitude, NIELL_DRY_B_AMP)
                val dryCAmp = interpolateNiellCoefficient(safeLatitude, NIELL_DRY_C_AMP)
                // Southern hemisphere offsets the phase by half a year (January is austral summer).
                val doyOffset = if (safeLatitude >= 0.0) 28.0 else 28.0 + 182.625
                val phase = cos(2.0 * Math.PI * (dayOfYear.coerceIn(1, 366) - doyOffset) / 365.25)
                Triple(
                    dryAAvg - dryAAmp * phase,
                    dryBAvg - dryBAmp * phase,
                    dryCAvg - dryCAmp * phase,
                )
            } else {
                Triple(dryAAvg, dryBAvg, dryCAvg)
            }

        val wetA = interpolateNiellCoefficient(safeLatitude, NIELL_WET_A)
        val wetB = interpolateNiellCoefficient(safeLatitude, NIELL_WET_B)
        val wetC = interpolateNiellCoefficient(safeLatitude, NIELL_WET_C)

        val dryMapping = niellMapping(sinElev, dryA, dryB, dryC).coerceAtMost(MAX_MAPPING)
        val wetMapping = niellMapping(sinElev, wetA, wetB, wetC).coerceAtMost(MAX_MAPPING)
        return dryMapping to wetMapping
    }

    /**
     * Computes the Saastamoinen dry zenith hydrostatic delay in meters.
     * Does NOT include the wet component (water vapor) so it can be used
     * independently in PPP mode where the solver estimates the wet delay.
     */
    fun computeDryZenithDelay(
        altitudeMeters: Double,
        latitudeDegrees: Double = 45.0,
    ): Double {
        val safeAltitude = sanitizeAltitudeMeters(altitudeMeters)
        val safeLatitude = sanitizeLatitudeDegrees(latitudeDegrees)
        val p = 1013.25 * (1.0 - 2.2557e-5 * safeAltitude).pow(5.2559)
        val latRad = Math.toRadians(safeLatitude)
        val fLat = 1.0 - 0.00266 * cos(2.0 * latRad) - 0.00028 * safeAltitude / 1000.0
        val delay = 0.002277 * p / fLat
        return if (delay.isFinite()) delay.coerceAtLeast(0.0) else 0.0
    }

    /**
     * Computes the Saastamoinen wet zenith delay in meters.
     * Uses a standard atmosphere model with 50% relative humidity.
     */
    fun computeWetZenithDelay(altitudeMeters: Double): Double {
        val safeAltitude = sanitizeAltitudeMeters(altitudeMeters)
        val t = (288.15 - 0.0065 * safeAltitude).coerceAtLeast(150.0)
        val h = 0.5
        val e = 6.11 * 10.0.pow(7.5 * (t - 273.15) / (t - 35.85)) * h
        val delay = 0.002277 * (1255.0 / t + 0.05) * e
        return if (delay.isFinite()) delay.coerceAtLeast(0.0) else 0.0
    }

    /**
     * Computes the total tropospheric delay (dry + wet) mapped to the slant direction.
     * Used in non-PPP mode where the solver does not estimate wet delay separately.
     */
    fun computeTroposphericDelay(
        elevationDegrees: Double,
        altitudeMeters: Double,
        latitudeDegrees: Double = 45.0,
        dayOfYear: Int? = null,
    ): Double {
        val safeElevation = sanitizeElevationDegrees(elevationDegrees)
        val safeAltitude = sanitizeAltitudeMeters(altitudeMeters)
        val safeLatitude = sanitizeLatitudeDegrees(latitudeDegrees)
        val dryZenith = computeDryZenithDelay(safeAltitude, safeLatitude)
        val wetZenith = computeWetZenithDelay(safeAltitude)

        val elevRad = Math.toRadians(safeElevation)
        if (elevRad < 0.01) {
            return (dryZenith * MAX_MAPPING + wetZenith * MAX_MAPPING).coerceAtMost(60.0)
        }

        val (dryMapping, wetMapping) = computeTroposphericMapping(safeElevation, safeLatitude, dayOfYear)
        val delay = dryZenith * dryMapping + wetZenith * wetMapping
        val minDelay = (dryZenith + wetZenith).coerceAtLeast(0.1).coerceAtMost(60.0)
        val safeDelay = if (delay.isFinite()) delay else minDelay
        return safeDelay.coerceIn(minDelay, 60.0)
    }

    /**
     * Calculates ionospheric delay using the Klobuchar model (IS-GPS-200 Section 20.3.3.5.2.5).
     *
     * @param elevationDegrees Satellite elevation angle in degrees.
     * @param azimuthDegrees Satellite azimuth angle in degrees (required for IPP computation).
     * @param lat Receiver geodetic latitude in degrees.
     * @param lon Receiver geodetic longitude in degrees.
     * @param timestamp GPS time in milliseconds.
     * @param alpha Klobuchar alpha coefficients (from broadcast, or null for defaults).
     * @param beta Klobuchar beta coefficients (from broadcast, or null for defaults).
     */
    fun computeIonosphericDelay(
        elevationDegrees: Double,
        azimuthDegrees: Double,
        lat: Double,
        lon: Double,
        timestamp: Long,
        alpha: DoubleArray? = null,
        beta: DoubleArray? = null,
    ): Double {
        val minElevRad = 0.05
        val safeElevation = sanitizeElevationDegrees(elevationDegrees)
        val safeLatitude = sanitizeLatitudeDegrees(lat)
        val safeLongitude = sanitizeLongitudeDegrees(lon)
        val elevRad = Math.toRadians(safeElevation).coerceAtLeast(minElevRad)

        // Default Klobuchar Model coefficients if not provided
        val a = alpha ?: doubleArrayOf(0.1397e-07, 0.1490e-07, -0.5960e-07, -0.1192e-06)
        val b = beta ?: doubleArrayOf(0.1045e+06, 0.1638e+05, -0.1966e+06, -0.4588e+06)

        // Receiver position in semi-circles
        val phi = safeLatitude / 180.0
        val lam = safeLongitude / 180.0
        val azRad = Math.toRadians(azimuthDegrees)

        // Earth-centered angle (semi-circles)
        val psi = 0.0137 / (elevRad / Math.PI + 0.11) - 0.022

        // Ionospheric pierce point (IPP) latitude and longitude (semi-circles)
        val phiI = (phi + psi * cos(azRad)).coerceIn(-0.416, 0.416)
        val lamI = lam + psi * sin(azRad) / cos(phiI * Math.PI)

        // Geomagnetic latitude of the IPP (IS-GPS-200 §20.3.3.5.2.5, all in semicircles):
        // the broadcast α/β polynomials are fitted against φ_m, not the geodetic IPP
        // latitude — skipping this 0.064-semicircle (≈11.5°) shift changes AMP/PER by up
        // to ~40% regionally (1–4 m of slant iono error on single-frequency pseudoranges).
        val phiM = phiI + 0.064 * cos((lamI - 1.617) * Math.PI)

        val t = ((43200.0 * lamI + (timestamp / 1000) % 86400) % 86400 + 86400) % 86400
        val amp = (a[0] + a[1] * phiM + a[2] * phiM.pow(2) + a[3] * phiM.pow(3)).coerceAtLeast(0.0)
        val per = (b[0] + b[1] * phiM + b[2] * phiM.pow(2) + b[3] * phiM.pow(3)).coerceAtLeast(72000.0)

        val x = 2.0 * Math.PI * (t - 50400.0) / per
        val ionoDelay =
            if (abs(x) < 1.57) {
                (5e-9 + amp * (1.0 - x.pow(2) / 2.0 + x.pow(4) / 24.0)) * SPEED_OF_LIGHT
            } else {
                5e-9 * SPEED_OF_LIGHT
            }

        val obliqueFactor = 1.0 + 16.0 * (0.53 - elevRad / Math.PI).pow(3)
        return ionoDelay * obliqueFactor
    }

    /**
     * Shapiro (relativistic) range correction using **geocentric** receiver and satellite ECEF positions
     * (standard form (2μ/c²) ln((r_R + r_S + ρ)/(r_R + r_S − ρ)) with r_R, r_S geocentric radii and ρ geometric range).
     */
    fun computeRelativisticCorrection(
        receiverX: Double,
        receiverY: Double,
        receiverZ: Double,
        satelliteX: Double,
        satelliteY: Double,
        satelliteZ: Double,
    ): Double {
        val c = SPEED_OF_LIGHT
        val rRec = sqrt(receiverX * receiverX + receiverY * receiverY + receiverZ * receiverZ)
        val rSat = sqrt(satelliteX * satelliteX + satelliteY * satelliteY + satelliteZ * satelliteZ)
        val dx = satelliteX - receiverX
        val dy = satelliteY - receiverY
        val dz = satelliteZ - receiverZ
        val rho = sqrt(dx * dx + dy * dy + dz * dz)
        if (rho < 1.0 || rRec < 1.0 || rSat < 1.0) return 0.0
        val num = rSat + rRec + rho
        val den = rSat + rRec - rho
        if (den <= 0.0 || !num.isFinite() || !den.isFinite()) return 0.0
        return (2.0 * MU / (c * c)) * ln(num / den)
    }
}
