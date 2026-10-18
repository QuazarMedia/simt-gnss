package app.simt.gnss.core.utils
import app.simt.gnss.core.model.AdrState
import kotlin.math.*

/**
 * Utility functions for GNSS coordinate conversions and bit-level parsing.
 */
object GnssMathUtils {
    /**
     * Wraps a time-of-week difference to the nearest half week (±302,400 s). Differences of
     * two TOW values straddling a GPS week rollover otherwise jump by ±604,800 s, which blows
     * up any rate × dt extrapolation (e.g. SSR dot terms, ephemeris age).
     */
    fun wrapToHalfWeekSeconds(dt: Double): Double =
        when {
            dt > 302400.0 -> dt - 604800.0
            dt < -302400.0 -> dt + 604800.0
            else -> dt
        }

    /**
     * Line-of-sight one-sigma, meters, for a broadcast ephemeris at |t − toe| seconds.
     * Knots are the 2025-10-04 GPS broadcast: p95 of (radial − clock) plus one quarter of
     * the p95 3-D orbit error. At two hours that is 0.6 m, so a fresh fix is unchanged.
     * Past 24 h the last segment continues. The clock itself stays within 2 m at 24 h;
     * the orbit is what grows.
     */
    fun broadcastEphemerisRangeSigmaMeters(ageSeconds: Double): Double {
        val hours = abs(ageSeconds) / 3600.0
        if (hours <= 0.0) return 0.0
        val last = EPHEMERIS_AGE_HOURS.lastIndex
        if (hours >= EPHEMERIS_AGE_HOURS[last]) {
            val slope =
                (ephemerisRangeSigmaAt(last) - ephemerisRangeSigmaAt(last - 1)) /
                    (EPHEMERIS_AGE_HOURS[last] - EPHEMERIS_AGE_HOURS[last - 1])
            return ephemerisRangeSigmaAt(last) + slope * (hours - EPHEMERIS_AGE_HOURS[last])
        }
        var upper = 1
        while (EPHEMERIS_AGE_HOURS[upper] < hours) upper++
        val lower = upper - 1
        val span = EPHEMERIS_AGE_HOURS[upper] - EPHEMERIS_AGE_HOURS[lower]
        val fraction = (hours - EPHEMERIS_AGE_HOURS[lower]) / span
        val low = ephemerisRangeSigmaAt(lower)
        return low + fraction * (ephemerisRangeSigmaAt(upper) - low)
    }

    /** Code sigma and broadcast-orbit sigma added in quadrature. Age 0 returns [sigmaCodeMeters]. */
    fun observationSigmaMeters(
        sigmaCodeMeters: Double,
        ephemerisAgeSeconds: Double,
    ): Double = hypot(sigmaCodeMeters, broadcastEphemerisRangeSigmaMeters(ephemerisAgeSeconds))

    private val EPHEMERIS_AGE_HOURS = doubleArrayOf(0.0, 2.0, 4.0, 6.0, 12.0, 24.0)

    /** p95 of radial-minus-clock, meters, at [EPHEMERIS_AGE_HOURS]. */
    private val EPHEMERIS_RANGE_P95_METERS = doubleArrayOf(0.0, 0.3, 35.0, 100.0, 128.0, 265.0)

    /** p95 of the 3-D orbit error, meters, at [EPHEMERIS_AGE_HOURS]. */
    private val EPHEMERIS_ORBIT_P95_METERS = doubleArrayOf(0.0, 1.2, 60.0, 248.0, 608.0, 1079.0)

    private fun ephemerisRangeSigmaAt(index: Int): Double = EPHEMERIS_RANGE_P95_METERS[index] + EPHEMERIS_ORBIT_P95_METERS[index] / 4.0

    /**
     * Gauss-Jordan inverse with partial pivoting for small square matrices (DOP geometry,
     * m ≤ ~9). Returns null when the matrix is singular or contains non-finite values.
     */
    fun invertSquareMatrix(matrix: Array<DoubleArray>): Array<DoubleArray>? {
        val n = matrix.size
        val a = Array(n) { r -> matrix[r].copyOf() }
        val inv = Array(n) { r -> DoubleArray(n) { c -> if (r == c) 1.0 else 0.0 } }
        for (col in 0 until n) {
            var pivot = col
            for (r in col + 1 until n) {
                if (kotlin.math.abs(a[r][col]) > kotlin.math.abs(a[pivot][col])) pivot = r
            }
            val pivotValue = a[pivot][col]
            if (!pivotValue.isFinite() || kotlin.math.abs(pivotValue) < 1e-12) return null
            if (pivot != col) {
                val ta = a[pivot]
                a[pivot] = a[col]
                a[col] = ta
                val ti = inv[pivot]
                inv[pivot] = inv[col]
                inv[col] = ti
            }
            val scale = 1.0 / a[col][col]
            for (c in 0 until n) {
                a[col][c] *= scale
                inv[col][c] *= scale
            }
            for (r in 0 until n) {
                if (r == col) continue
                val factor = a[r][col]
                if (factor == 0.0) continue
                for (c in 0 until n) {
                    a[r][c] -= factor * a[col][c]
                    inv[r][c] -= factor * inv[col][c]
                }
            }
        }
        return inv
    }

    const val SPEED_OF_LIGHT = 299792458.0
    const val MU = 3.986004418e14 // WGS84/GTRF/CGCS2000 gravitational constant (Galileo, BeiDou, Shapiro term)
    const val GPS_MU = 3.986005e14 // IS-GPS-200 §20.3.3.4.3.3 — GPS/QZSS/IRNSS broadcast orbits are fitted against this value
    const val OMEGA_EARTH = 7.2921151467e-5

    /** |lat| above which ecefToLatLonAlt uses the z-based (pole-safe) altitude branch. */
    const val POLAR_ALTITUDE_SWITCH_RADIANS = 1.17 // ≈ 67°
    const val SECONDS_IN_WEEK = 604800.0
    const val SECONDS_IN_DAY = 86400.0
    const val NANOS_PER_SECOND = 1_000_000_000L
    const val NANOS_PER_WEEK = (SECONDS_IN_WEEK * NANOS_PER_SECOND).toLong()
    const val NANOS_PER_DAY = (SECONDS_IN_DAY * NANOS_PER_SECOND).toLong()
    const val EPHEMERIS_STALENESS_MS = 4 * 3600 * 1000L // 4 hours

    /** How long a received Keplerian record stays usable. Orbit error is weighted by |t − toe|, not by this clock. */
    const val EPHEMERIS_EXTENDED_VALIDITY_MS = 24 * 3600 * 1000L // 24 hours

    /** GLONASS state-vector propagation accumulates RK4 errors faster; cap at 6 hours */
    const val GLONASS_EPHEMERIS_EXTENDED_VALIDITY_MS = 6 * 3600 * 1000L // 6 hours

    /** SBAS GEO ephemeris (MT 9) is broadcast every ~120s; second-order Taylor expansion
     *  diverges quickly for GEO satellites — cap at 600 seconds (10 minutes). */
    const val SBAS_EPHEMERIS_EXTENDED_VALIDITY_MS = 600 * 1000L // 10 minutes

    /** SBAS fast corrections (MT 2–5 PRC) are valid for 6–120s depending on UDRE degradation.
     *  Use 18 s as a conservative timeout (typical SBAS update rate = 6 s, ×3 for margin). */
    const val SBAS_FAST_CORRECTION_STALENESS_MS = 18 * 1000L

    /** SBAS long-term corrections (MT 25) change slowly; valid for 120 seconds. */
    const val SBAS_LONG_TERM_CORRECTION_STALENESS_MS = 120 * 1000L

    /** SBAS ionospheric grid delays (MT 26) change on the order of minutes; 300 s validity. */
    const val SBAS_IONO_GRID_STALENESS_MS = 300 * 1000L
    const val MAX_WLS_ITERATIONS = 6
    const val CONVERGENCE_THRESHOLD_METERS = 0.01

    /**
     * Receiver-state movement (metres, and metres-equivalent clock bias) below which the per-SV
     * satellite position / clock and ionospheric / tropospheric model terms are reused across WLS
     * iterations instead of being recomputed. The satellite position does not depend on the receiver
     * position at all, and the iono/tropo gradients are well under 1 mm per metre, so reusing these
     * terms within this radius perturbs the converged fix by far less than a millimetre while
     * skipping the per-iteration Kepler solve and Klobuchar evaluation. Kept well below the
     * code-noise floor ([MIN_PSEUDORANGE_SIGMA_METERS]) so it cannot bias the solution.
     */
    const val WLS_MODEL_REUSE_DISTANCE_METERS = 0.5
    const val APPROXIMATE_GEOMETRY_MAX_WLS_ITERATIONS = 12
    const val APPROXIMATE_GEOMETRY_CONVERGENCE_THRESHOLD_METERS = 2.0
    const val APPROXIMATE_GEOMETRY_CLOCK_CONVERGENCE_THRESHOLD_METERS = 300.0
    const val APPROXIMATE_GEOMETRY_ACCEPTABLE_RESIDUAL_RMS_METERS = 250.0
    const val APPROXIMATE_GEOMETRY_ACCEPTABLE_MAX_RESIDUAL_METERS = 500.0
    const val L5_FREQUENCY_THRESHOLD_HZ = 1.2e9

    /**
     * GPS L1 / Galileo E1 carrier frequency (Hz). The Klobuchar, NeQuick-G and SBAS-grid ionosphere
     * models all return the slant delay referenced to this frequency; single-frequency measurements
     * on other bands are scaled by (L1_FREQUENCY_HZ / f)² since the ionospheric delay ∝ 1/f².
     */
    const val L1_FREQUENCY_HZ = 1_575_420_000.0
    const val HORIZONTAL_ACCURACY_BASE = 0.2f
    const val HORIZONTAL_ACCURACY_FACTOR = 0.5f
    const val MIN_HORIZONTAL_ACCURACY = 0.05f
    const val MAX_HORIZONTAL_ACCURACY = 50.0f
    const val APPROXIMATE_GEOMETRY_MIN_HORIZONTAL_ACCURACY = 25.0f
    const val APPROXIMATE_GEOMETRY_MAX_HORIZONTAL_ACCURACY = 150.0f
    const val VERTICAL_ACCURACY_FACTOR = 1.5f
    const val GPS_UTC_LEAP_SECONDS = 18L

    /** GPS epoch (Jan 6, 1980 00:00 UTC) in Unix seconds */
    const val GPS_EPOCH_UNIX_SECONDS = 315964800L
    const val BDT_GPST_OFFSET_SECONDS = 14.0
    const val GLONASS_UTC_OFFSET_SECONDS = 10800.0 // Moscow time = UTC + 3 hours
    const val GLONASS_MU = 3.9860044180e14 // PZ-90 gravitational constant (m³/s²)
    const val GLONASS_AE = 6378136.0 // PZ-90 equatorial radius (m)
    const val GLONASS_J2 = 1.0826257e-3 // Second zonal harmonic
    const val GLONASS_OMEGA_E = 7.2921151467e-5 // Earth rotation rate (rad/s)
    const val GLONASS_RK4_STEP = 60.0 // RK4 integration step (s)

    /**
     * Maximum |t − tb| for GLONASS state-vector propagation. The broadcast force model
     * (constant luni-solar acceleration at tb, J2-only) is specified for ±15 min and tb
     * refreshes every 30 min; 45 min covers one missed broadcast with margin. Beyond this
     * the RK4 extrapolation accumulates km-class error.
     */
    const val GLONASS_MAX_PROPAGATION_SECONDS = 2_700.0
    const val RAIM_RESIDUAL_THRESHOLD_METERS = 8.0
    const val APPROXIMATE_GEOMETRY_RAIM_RESIDUAL_THRESHOLD_METERS = 120.0
    const val CN0_REFERENCE_DB_HZ = 40.0
    const val PHASE_VARIANCE_M2 = 1e-4 // (0.01m)^2 = 1 cm phase noise variance

    /**
     * Hatch smoothing window cap (epochs ≈ seconds at 1 Hz). Single-frequency code and
     * carrier diverge at twice the iono rate, so the classic guidance caps near 100 s —
     * the previous 200 accumulated a ~4-8 m steady-state bias under ordinary daytime
     * ionosphere, correlated across satellites.
     */
    const val HATCH_FILTER_MAX_SMOOTHING = 100.0

    /**
     * How far a carrier delta may depart from the change the pseudorange rate predicts, as an
     * absolute floor in metres, before it is called a cycle slip.
     *
     * Replaces a fixed 100 m threshold on `|ΔADR|`, which was not a slip detector at all: it was a
     * guess about how far a *receiver* can travel in one second, applied to how far a *satellite*
     * moves. GPS range rates reach several hundred metres per second for a receiver that is not
     * moving at all, so the threshold was crossed on 88 % of epochs of a reference walk and the
     * Hatch filter never smoothed a single update.
     *
     * The floor is not zero because the detector's own noise is not zero. Replaying a reference
     * capture, `|ΔADR − PRR·Δt|` has a median of 2 cm and a 95th percentile of 7 cm, but a maximum
     * of 34 cm over 2 055 updates — multipath and scintillation both move the carrier and the rate
     * together and not always identically. A floor below that maximum buys sensitivity at the price
     * of resetting a healthy filter, which is the failure this change exists to remove.
     *
     * It also means this detector is deliberately a *multi-cycle* slip detector. One L1 wavelength
     * is 19 cm and one L5 wavelength is 24 cm, so neither is separable from the 34 cm tail above.
     * Sub-wavelength slips are the receiver's call and arrive as
     * [AdrState.CYCLE_SLIP], which is checked independently and
     * which the hardware produces in the same clock domain as the rate used here. This threshold
     * answers the question the flags cannot: whether the carrier delta is consistent with the range
     * rate at all.
     */
    const val ADR_DOPPLER_SLIP_FLOOR_METERS = 0.5

    /**
     * How many standard deviations of the reported pseudorange-rate uncertainty the carrier delta may
     * miss the rate's prediction by, on top of [ADR_DOPPLER_SLIP_FLOOR_METERS].
     *
     * The rate's contribution to the prediction is `σ_rate · Δt` metres, so the tolerance scales with
     * how poorly the receiver knows its own rate rather than being a second fixed number. Six sigma
     * is chosen so that on hardware reporting a plausible rate uncertainty — a few cm/s, which gives
     * a few centimetres at the 1 Hz epochs this pipeline runs at — the absolute floor is the binding
     * constraint and the detector is insensitive to the exact reported σ; the term only takes over on
     * hardware whose rate uncertainty is large enough to matter, which is exactly where a fixed
     * floor would have produced false slips.
     */
    const val ADR_DOPPLER_SLIP_SIGMA = 6.0

    const val ADR_CODE_CONSISTENCY_THRESHOLD_METERS = 30.0

    /**
     * Code-carrier consistency is widened with the reported code uncertainty, above
     * [ADR_CODE_CONSISTENCY_THRESHOLD_METERS]. The two observables are independent, so the
     * difference has variance 2·σ² and the limit is this many of those standard deviations.
     * A fixed 30 m rejects a healthy update once code noise itself is ~10 m.
     */
    const val ADR_CODE_CONSISTENCY_SIGMA = 5.0
    const val MIN_PSEUDORANGE_SIGMA_METERS = 0.5
    const val MAX_PSEUDORANGE_SIGMA_METERS = 50.0

    /**
     * Jin–Wang / Euler–Goad elevation-dependent code noise model:
     *   σ²_PR(el, C/N₀) = (a² + b²/sin²(el)) · 10^(−(C/N₀ − C/N₀_ref)/10)
     * The [a] term dominates at zenith (clean signal, ~0.3 m one-sigma code noise); [b] models
     * multipath + elevation-scaled noise and kicks in at low elevation. Values below are
     * well-established in the literature for consumer-grade GNSS on Android hardware; tighter
     * than survey-grade tuning (which would use a=0.1, b=0.3) but realistic for typical SNR.
     */
    const val PSEUDORANGE_NOISE_ZENITH_A_METERS: Double = 0.3
    const val PSEUDORANGE_NOISE_ELEVATION_B_METERS: Double = 0.5

    /** If Android's reported `receivedSvTimeUncertaintyNanos × c` is below this, treat as a
     *  time-tagging lower bound only; always apply the [PSEUDORANGE_NOISE_ZENITH_A_METERS] /
     *  [PSEUDORANGE_NOISE_ELEVATION_B_METERS] floor for weighting. */
    const val PSEUDORANGE_ANDROID_SIGMA_TRUST_METERS: Double = 0.2

    /**
     * Normalized (standardized) RAIM threshold in σ units. After solving WLS we compute
     *   z_i = |r_i| / (σ_i · √(1 − h_ii))
     * where h_ii is the leverage of measurement i (diag of the hat matrix) and σ_i is the
     * a-priori measurement sigma. Threshold 4.0 corresponds to a one-sided 99.997% rejection
     * rate under Gaussian residuals — balances false-alarm vs miss probability for a moderately
     * large set of candidate observations.
     */
    const val RAIM_STANDARDIZED_RESIDUAL_SIGMA: Double = 4.0

    /** Minimum leverage safety floor to avoid division blow-up when h_ii ≈ 1 (redundancy=0). */
    const val RAIM_LEVERAGE_MAX: Double = 0.999
    const val DOPPLER_SMOOTHING_ALPHA = 0.2
    const val DOPPLER_MAX_DT_SECONDS = 5.0
    const val DOPPLER_MAX_RATE_UNCERTAINTY_MPS = 3.0
    const val DOPPLER_MAX_ABS_RATE_MPS = 1500.0
    const val DOPPLER_MAX_RAW_DEVIATION_METERS = 150.0
    const val MIN_MEASUREMENT_EPOCHS = 5
    const val MAX_MAPPING_FUNCTION_VALUE = 20.0

    /** Drop Hatch / Doppler smoother state for SVs not seen for this many measurement epochs (~2 min @ 1 Hz). */
    const val SMOOTHER_MAX_STALE_EPOCHS: Long = 120

    // --- Inter-epoch Kalman (ECEF position + velocity) ---

    /** Re-initialize the inter-epoch filter if the GPS-time gap exceeds this (nanoseconds). */
    const val KALMAN_RESET_GAP_NANOS: Long = 5_000_000_000L

    /**
     * Maximum allowed **normalized** innovation (Mahalanobis distance, sqrt of χ²_3) between WLS
     * and predicted position before the filter is discarded and re-initialised. 4.0 ≈ 99.9%
     * confidence for a 3-DOF chi-square, which rejects genuine outliers / carrier-phase cycle
     * slips while still admitting noisy fixes. (Previously compared against 75 m, which—combined
     * with typical P+R ≈ 50 m² diagonal—required ~530 m innovations to trip, i.e. effectively
     * disabled.)
     */
    const val KALMAN_MAX_INNOVATION_SIGMA: Double = 4.0

    /**
     * Safety net in metres: even if the Mahalanobis gate passes (e.g. P and R overestimated),
     * reject epochs whose Euclidean innovation exceeds this value. Guards against measurement
     * covariance blow-ups that mask large jumps.
     */
    const val KALMAN_MAX_INNOVATION_METERS: Double = 500.0

    /**
     * Consecutive innovation-gate rejections after which the Kalman filter re-initializes from
     * the measurements. Below this, gated epochs are treated as outliers and rejected (the
     * filter coasts on its prediction); at/above it, the persistent disagreement means the
     * filter state itself is wrong (e.g. after an undetected displacement) and following the
     * measurements is the lesser error.
     */
    const val KALMAN_MAX_CONSECUTIVE_REJECTIONS: Int = 3

    /**
     * Continuous **white acceleration** power spectral density per axis (m²/s⁴ per Hz) used to build
     * discrete Q for the constant-velocity model; ~0.5–1.0 is typical for handheld motion.
     */
    const val KALMAN_ACCEL_NOISE_DENSITY_M2_S3: Double = 0.3

    /** Initial velocity variance per axis after first fix (m²/s²). */
    const val KALMAN_INITIAL_VEL_VAR_M2_S2: Double = 100.0

    /**
     * Clamp estimated speed magnitude (m/s) to avoid unrealistic coasting after bad
     * geometry. 120 m/s (~430 km/h) covers high-speed rail and small aircraft; the
     * reject-don't-adopt innovation gate is the actual outlier defense, so the clamp
     * only needs to bound true divergence (the previous 55 m/s silently broke the
     * filter on trains, re-initializing nearly every epoch).
     */
    const val KALMAN_MAX_SPEED_MPS: Double = 120.0

    fun cn0WeightFactor(cn0DbHz: Double): Double {
        val exponent = (cn0DbHz - CN0_REFERENCE_DB_HZ) / 10.0
        return 10.0.pow(exponent)
    }

    /**
     * Physical model for pseudorange one-sigma noise (meters) given satellite elevation and
     * C/N₀. Combines the elevation-dependent code-noise term with a C/N₀ scaling. The result
     * is *always* at least [PSEUDORANGE_NOISE_ZENITH_A_METERS] — no zero-variance downweighting.
     *
     * The floor applied by callers should be `max(androidReportedSigma, this)` so Android's
     * time-tagging uncertainty never inflates weights below the physical code-noise limit.
     */
    fun pseudorangeSigmaFromElevationAndCn0(
        elevationDegrees: Double,
        cn0DbHz: Double,
    ): Double {
        val el = elevationDegrees.coerceIn(0.1, 90.0)
        val sinEl = sin(Math.toRadians(el)).coerceAtLeast(1e-3)
        val a = PSEUDORANGE_NOISE_ZENITH_A_METERS
        val b = PSEUDORANGE_NOISE_ELEVATION_B_METERS
        val baseVar = a * a + (b * b) / (sinEl * sinEl)
        // C/N₀ scaling: 10 dB higher C/N₀ = ×0.1 variance = /√10 sigma; 10 dB lower = ×10 variance.
        val cn0Scale = 10.0.pow(-(cn0DbHz - CN0_REFERENCE_DB_HZ) / 10.0)
        return sqrt(baseVar * cn0Scale)
    }

    /**
     * Converts WGS84 Geodetic coordinates to ECEF (X, Y, Z).
     */
    fun latLonAltToEcef(
        lat: Double,
        lon: Double,
        alt: Double,
    ): Triple<Double, Double, Double> {
        val a = 6378137.0
        val e2 = 6.69437999014e-3
        val latRad = Math.toRadians(lat)
        val lonRad = Math.toRadians(lon)
        val n = a / sqrt(1 - e2 * sin(latRad).pow(2))
        val x = (n + alt) * cos(latRad) * cos(lonRad)
        val y = (n + alt) * cos(latRad) * sin(lonRad)
        val z = (n * (1 - e2) + alt) * sin(latRad)
        return Triple(x, y, z)
    }

    /**
     * Converts ECEF (X, Y, Z) to WGS84 Geodetic coordinates.
     */
    fun ecefToLatLonAlt(
        x: Double,
        y: Double,
        z: Double,
    ): Triple<Double, Double, Double> {
        val a = 6378137.0
        val e2 = 6.69437999014e-3
        val b = a * sqrt(1 - e2)
        val ep2 = (a * a - b * b) / (b * b)
        val p = sqrt(x * x + y * y)
        val theta = atan2(a * z, b * p)
        val lat =
            atan2(
                z + ep2 * b * sin(theta).pow(3),
                p - e2 * a * cos(theta).pow(3),
            )
        val lon = atan2(y, x)
        val n = a / sqrt(1 - e2 * sin(lat).pow(2))
        // alt = p/cos(lat) − N is singular toward the poles (cos→0, p→0 yields −N at the
        // exact pole); use the z-based form in the last ~23° of latitude where it is the
        // numerically stable branch.
        val alt =
            if (abs(lat) > POLAR_ALTITUDE_SWITCH_RADIANS) {
                z / sin(lat) - n * (1 - e2)
            } else {
                p / cos(lat) - n
            }
        return Triple(Math.toDegrees(lat), Math.toDegrees(lon), alt)
    }

    /**
     * Converts receiver->satellite LOS in ECEF into topocentric elevation/azimuth (degrees).
     */
    fun ecefLosToElevationAzimuth(
        receiverX: Double,
        receiverY: Double,
        receiverZ: Double,
        receiverLatDeg: Double,
        receiverLonDeg: Double,
        satelliteX: Double,
        satelliteY: Double,
        satelliteZ: Double,
    ): Pair<Double, Double> {
        val dx = satelliteX - receiverX
        val dy = satelliteY - receiverY
        val dz = satelliteZ - receiverZ
        val (east, north, up) = ecefDeltaToEnu(dx, dy, dz, receiverLatDeg, receiverLonDeg)

        val horizontal = sqrt(east * east + north * north)
        val elevationDeg = Math.toDegrees(atan2(up, horizontal))

        var azimuthDeg = Math.toDegrees(atan2(east, north))
        if (azimuthDeg < 0.0) azimuthDeg += 360.0

        return elevationDeg to azimuthDeg
    }

    /** ECEF displacement to topocentric east, north, up, in meters. */
    fun ecefDeltaToEnu(
        dx: Double,
        dy: Double,
        dz: Double,
        latDeg: Double,
        lonDeg: Double,
    ): Triple<Double, Double, Double> {
        val latRad = Math.toRadians(latDeg)
        val lonRad = Math.toRadians(lonDeg)
        val sinLat = sin(latRad)
        val cosLat = cos(latRad)
        val sinLon = sin(lonRad)
        val cosLon = cos(lonRad)
        val east = -sinLon * dx + cosLon * dy
        val north = -sinLat * cosLon * dx - sinLat * sinLon * dy + cosLat * dz
        val up = cosLat * cosLon * dx + cosLat * sinLon * dy + sinLat * dz
        return Triple(east, north, up)
    }

    /** Horizontal magnitude of an ECEF displacement, meters. */
    fun ecefDeltaHorizontalMeters(
        dx: Double,
        dy: Double,
        dz: Double,
        latDeg: Double,
        lonDeg: Double,
    ): Double {
        val (east, north, _) = ecefDeltaToEnu(dx, dy, dz, latDeg, lonDeg)
        return sqrt(east * east + north * north)
    }

    /**
     * Extracts an unsigned integer from a bit stream as [Long].
     * Supports bit widths up to 63 without overflow.
     */
    fun getBitsLong(
        data: ByteArray,
        startBit: Int,
        length: Int,
    ): Long {
        var value = 0L
        for (i in 0 until length) {
            val bitIdx = startBit + i
            val byteIdx = bitIdx / 8
            val bitOffset = 7 - (bitIdx % 8)
            if (byteIdx < data.size) {
                val bit = ((data[byteIdx].toInt() shr bitOffset) and 1).toLong()
                value = (value shl 1) or bit
            }
        }
        return value
    }

    /**
     * Extracts an unsigned integer from a bit stream as [Int].
     * Safe for bit widths up to 31. For 32-bit unsigned fields, use [getBitsLong].
     */
    fun getBits(
        data: ByteArray,
        startBit: Int,
        length: Int,
    ): Int = getBitsLong(data, startBit, length).toInt()

    /**
     * Extracts a two's-complement signed integer from a bit stream as [Long].
     * Supports bit widths up to 63.
     */
    fun getSignedBitsLong(
        data: ByteArray,
        startBit: Int,
        length: Int,
    ): Long {
        var value = getBitsLong(data, startBit, length)
        val msbMask = 1L shl (length - 1)
        if ((value and msbMask) != 0L) value -= (1L shl length)
        return value
    }

    /**
     * Extracts a two's-complement signed integer from a bit stream as [Int].
     * Safe for bit widths up to 32. For wider fields, use [getSignedBitsLong].
     */
    fun getSignedBits(
        data: ByteArray,
        startBit: Int,
        length: Int,
    ): Int = getSignedBitsLong(data, startBit, length).toInt()

    /**
     * Extracts a sign-magnitude encoded value (GLONASS convention).
     * Bit at [startBit] is the sign (0=positive, 1=negative),
     * remaining [length]-1 bits are the unsigned magnitude.
     */
    fun getSignMagnitude(
        data: ByteArray,
        startBit: Int,
        length: Int,
    ): Double {
        val sign = getBits(data, startBit, 1)
        val magnitude = getBitsLong(data, startBit + 1, length - 1).toDouble()
        return if (sign != 0) -magnitude else magnitude
    }

    /**
     * Packs bits into a ByteArray at the specified bit offset.
     */
    fun packBits(
        data: ByteArray,
        startBit: Int,
        length: Int,
        value: Int,
    ) {
        for (i in 0 until length) {
            val bitIdx = startBit + i
            val byteIdx = bitIdx / 8
            val bitOffset = 7 - (bitIdx % 8)
            val bit = (value shr (length - 1 - i)) and 1
            if (byteIdx < data.size) {
                if (bit == 1) {
                    data[byteIdx] = (data[byteIdx].toInt() or (1 shl bitOffset)).toByte()
                } else {
                    data[byteIdx] = (data[byteIdx].toInt() and (1 shl bitOffset).inv()).toByte()
                }
            }
        }
    }
}
