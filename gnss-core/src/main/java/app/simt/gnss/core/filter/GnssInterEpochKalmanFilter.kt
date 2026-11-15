package app.simt.gnss.core.filter

import app.simt.gnss.core.utils.GnssMathUtils
import kotlin.math.abs
import kotlin.math.sqrt

/**
 * A rejected measurement: the published point is the prediction, [positionCovariance] is the
 * predicted position block of P, and the innovation is the ECEF gap to the measurement that
 * was not used.
 */
data class KalmanCoast(
    val positionCovariance: Array<DoubleArray>,
    val innovationX: Double,
    val innovationY: Double,
    val innovationZ: Double,
)

/**
 * **Inter-epoch** extended state (ECEF position + velocity) Kalman filter that fuses successive
 * weighted-least-squares fixes. Smooths epoch-to-epoch noise and improves behavior under marginal
 * geometry vs. a purely memoryless WLS output each epoch.
 *
 * State **x** = `[rx, ry, rz, vx, vy, vz]` (meters, m/s). **Constant-velocity** prediction with
 * isotropic **white acceleration** process noise per axis. Measurements are **WLS position** with
 * covariance **R** (3×3) derived from the solver’s position block and residual RMS.
 *
 * Clock and ambiguity states remain **inside** the per-epoch WLS; this filter does not carry ISBs
 * across time (see GNSS feature plan / code review design remark #2).
 */
class GnssInterEpochKalmanFilter {
    private val x = DoubleArray(STATE_DIM)
    private val p = Array(STATE_DIM) { DoubleArray(STATE_DIM) }
    private var initialized = false
    private var hasPreviousEpoch = false
    private var lastTimeNanos: Long = 0L
    private var coast: KalmanCoast? = null

    /** Non-null only when the last [fuse] rejected the measurement and published the prediction. */
    fun coastReport(): KalmanCoast? = coast

    /**
     * Consecutive epochs rejected by the innovation gate. A single gated epoch is an outlier
     * to be *rejected* (coast on the prediction); only persistent disagreement means the
     * filter state is wrong and must re-initialize from the measurements.
     */
    private var consecutiveRejections = 0

    /** White acceleration PSD per axis (m²/s⁴ · s → discrete Q); tuned for pedestrian / handheld. */
    private var accelNoiseDensity = GnssMathUtils.KALMAN_ACCEL_NOISE_DENSITY_M2_S3

    fun reset() {
        initialized = false
        hasPreviousEpoch = false
        lastTimeNanos = 0L
        consecutiveRejections = 0
        coast = null
        x.fill(0.0)
        for (row in p) row.fill(0.0)
    }

    /**
     * Synchronizes the filter state to an external fix (for example an RTK solution) so a
     * subsequent fallback to absolute positioning predicts from the external position and
     * velocity instead of fusing against a stale pre-RTK state.
     */
    fun syncToExternalFix(
        positionEcef: Triple<Double, Double, Double>,
        positionCovarianceEcef: Array<DoubleArray>,
        timeNanos: Long,
        velocityEcef: Triple<Double, Double, Double>? = null,
        velocityCovarianceEcef: Array<DoubleArray>? = null,
    ) {
        val (px, py, pz) = positionEcef
        if (!px.isFinite() || !py.isFinite() || !pz.isFinite()) {
            reset()
            return
        }
        val rPos =
            Array(3) { i ->
                DoubleArray(3) { j -> safePositionCovEntry(positionCovarianceEcef, i, j, isDiagonal = i == j) }
            }
        initFromMeasurement(px, py, pz, rPos, timeNanos)
        coast = null
        if (velocityEcef != null) {
            val (vx, vy, vz) = velocityEcef
            if (vx.isFinite() && vy.isFinite() && vz.isFinite()) {
                x[3] = vx
                x[4] = vy
                x[5] = vz
                val rVel = velocityCovarianceEcef?.let { sanitizeCovariance3x3(it, minDiag = 1e-6) }
                if (rVel != null) {
                    for (i in 0 until 3) {
                        for (j in 0 until 3) {
                            p[3 + i][3 + j] = rVel[i][j]
                        }
                    }
                }
                clipVelocity()
            }
        }
    }

    /**
     * Fuses a new WLS ECEF fix. Returns **filtered** ECEF position (meters).
     *
     * State is 9D `[r_k, v_k, r_{k-1}]` so TDCP relative displacements (`r_k - r_{k-1}`) update
     * through `H = [I_3, 0_3, -I_3]` without re-injecting stale anchor bias as an absolute fix.
     *
     * @param wlsEcef unconverged WLS position for this epoch
     * @param rMeasurement 3×3 measurement covariance (m²); typically `σ₀² × covWls[0..2,0..2]`
     * @param timeNanos GPS time of epoch (nanoseconds); must be strictly increasing for prediction
     */
    fun fuse(
        wlsEcef: Triple<Double, Double, Double>,
        rMeasurement: Array<DoubleArray>,
        timeNanos: Long,
        velocityEcef: Triple<Double, Double, Double>? = null,
        velocityVarianceM2PerS2: Triple<Double, Double, Double>? = null,
        velocityCovarianceM2PerS2: Array<DoubleArray>? = null,
        relativeDisplacementEcef: Triple<Double, Double, Double>? = null,
        relativeDisplacementCovarianceM2: Array<DoubleArray>? = null,
    ): Triple<Double, Double, Double> {
        val (zx, zy, zz) = wlsEcef

        // A non-finite measurement cannot be fused or adopted; coast on the current state
        // without mutating P or running predict-less displacement updates.
        if (!zx.isFinite() || !zy.isFinite() || !zz.isFinite()) {
            return if (initialized) {
                rememberCoast(0.0, 0.0, 0.0)
                Triple(x[0], x[1], x[2])
            } else {
                coast = null
                wlsEcef
            }
        }

        // Sanitize R once and use the sanitized copy everywhere (gate, update, Joseph K·R·Kᵀ):
        // a single NaN entry would otherwise poison P permanently and self-disable the filter.
        val r =
            Array(3) { i ->
                DoubleArray(3) { j -> safePositionCovEntry(rMeasurement, i, j, isDiagonal = i == j) }
            }

        if (!initialized) {
            initFromMeasurement(zx, zy, zz, r, timeNanos)
            coast = null
            applyVelocityMeasurement(velocityEcef, velocityCovarianceM2PerS2, velocityVarianceM2PerS2)
            return Triple(x[0], x[1], x[2])
        }

        val dtSec =
            (timeNanos - lastTimeNanos).toDouble() / GnssMathUtils.NANOS_PER_SECOND.toDouble()
        lastTimeNanos = timeNanos

        if (dtSec <= 0.0 ||
            dtSec * GnssMathUtils.NANOS_PER_SECOND.toDouble() > GnssMathUtils.KALMAN_RESET_GAP_NANOS.toDouble()
        ) {
            initFromMeasurement(zx, zy, zz, r, timeNanos)
            coast = null
            applyVelocityMeasurement(velocityEcef, velocityCovarianceM2PerS2, velocityVarianceM2PerS2)
            return Triple(x[0], x[1], x[2])
        }

        predict(dtSec)

        val y0 = zx - x[0]
        val y1 = zy - x[1]
        val y2 = zz - x[2]
        val innovEuclid = sqrt(y0 * y0 + y1 * y1 + y2 * y2)
        // null when S is not invertible — fall back to the metric (Euclidean) gate alone.
        val innovMahal = mahalanobisInnovation(doubleArrayOf(y0, y1, y2), r)
        val gated =
            (innovMahal != null && innovMahal > GnssMathUtils.KALMAN_MAX_INNOVATION_SIGMA) ||
                innovEuclid > GnssMathUtils.KALMAN_MAX_INNOVATION_METERS
        if (gated) {
            consecutiveRejections++
            if (consecutiveRejections >= GnssMathUtils.KALMAN_MAX_CONSECUTIVE_REJECTIONS) {
                // Persistent disagreement: the filter state (not the measurements) is wrong.
                initFromMeasurement(zx, zy, zz, r, timeNanos)
                coast = null
                applyVelocityMeasurement(velocityEcef, velocityCovarianceM2PerS2, velocityVarianceM2PerS2)
            } else {
                // Single/short outlier burst: REJECT it and coast on the prediction. Re-initializing
                // onto the gated measurement would pass every multipath spike through at full gain
                // and destroy the converged velocity/covariance state.
                applyVelocityMeasurement(velocityEcef, velocityCovarianceM2PerS2, velocityVarianceM2PerS2)
                rememberCoast(zx - x[0], zy - x[1], zz - x[2])
            }
            return Triple(x[0], x[1], x[2])
        }
        consecutiveRejections = 0
        coast = null

        updateWithPositionMeasurement(doubleArrayOf(y0, y1, y2), r)
        clipVelocity()
        applyVelocityMeasurement(velocityEcef, velocityCovarianceM2PerS2, velocityVarianceM2PerS2)
        applyRelativeDisplacement(relativeDisplacementEcef, relativeDisplacementCovarianceM2)
        return Triple(x[0], x[1], x[2])
    }

    /** ECEF velocity after the last [fuse]. Zero until a velocity measurement is applied. */
    fun velocityEcef(): Triple<Double, Double, Double> = Triple(x[3], x[4], x[5])

    private fun applyRelativeDisplacement(
        deltaEcef: Triple<Double, Double, Double>?,
        covariance: Array<DoubleArray>?,
    ) {
        if (deltaEcef == null || covariance == null || !initialized || !hasPreviousEpoch) return
        val (dx, dy, dz) = deltaEcef
        if (!dx.isFinite() || !dy.isFinite() || !dz.isFinite()) return
        val r = sanitizeCovariance3x3(covariance, minDiag = 1e-4) ?: return

        val innovation =
            doubleArrayOf(
                dx - (x[0] - x[6]),
                dy - (x[1] - x[7]),
                dz - (x[2] - x[8]),
            )

        // H = [I_3, 0_3, -I_3] => H P H^T = P_00 + P_66 - P_06 - P_60
        val s =
            Array(3) { i ->
                DoubleArray(3) { j ->
                    (p[i][j] + p[6 + i][6 + j] - p[i][6 + j] - p[6 + i][j] + r[i][j]).coerceFinite()
                }
            }
        val sInv = invert3x3(s) ?: return

        val pHt =
            Array(STATE_DIM) { i ->
                DoubleArray(3) { m ->
                    p[i][m] - p[i][6 + m]
                }
            }
        val k = Array(STATE_DIM) { DoubleArray(3) }
        for (i in 0 until STATE_DIM) {
            for (j in 0 until 3) {
                var sum = 0.0
                for (m in 0 until 3) {
                    sum += pHt[i][m] * sInv[m][j]
                }
                k[i][j] = sum
            }
        }

        for (i in 0 until STATE_DIM) {
            for (j in 0 until 3) {
                x[i] += k[i][j] * innovation[j]
            }
        }

        // Joseph form: P = (I - K H) P (I - K H)^T + K R K^T
        val iMinusKh = Array(STATE_DIM) { DoubleArray(STATE_DIM) }
        for (i in 0 until STATE_DIM) {
            for (j in 0 until STATE_DIM) {
                val khEntry =
                    when (j) {
                        in 0..2 -> k[i][j]
                        in 6..8 -> -k[i][j - 6]
                        else -> 0.0
                    }
                iMinusKh[i][j] = (if (i == j) 1.0 else 0.0) - khEntry
            }
        }

        val p1 = matMul9(iMinusKh, p)
        val p2 = matMul9(p1, transpose9(iMinusKh))
        val krKt = Array(STATE_DIM) { DoubleArray(STATE_DIM) }
        for (i in 0 until STATE_DIM) {
            for (j in 0 until STATE_DIM) {
                var sum = 0.0
                for (a in 0 until 3) {
                    for (b in 0 until 3) {
                        sum += k[i][a] * r[a][b] * k[j][b]
                    }
                }
                krKt[i][j] = sum
            }
        }
        for (i in 0 until STATE_DIM) {
            for (j in 0 until STATE_DIM) {
                p[i][j] = p2[i][j] + krKt[i][j]
            }
        }
        symmetrize9(p)
        clipVelocity()
    }

    private fun applyVelocityMeasurement(
        velocityEcef: Triple<Double, Double, Double>?,
        covariance: Array<DoubleArray>?,
        variance: Triple<Double, Double, Double>?,
    ) {
        if (velocityEcef == null || !initialized) return
        val (vx, vy, vz) = velocityEcef
        if (!vx.isFinite() || !vy.isFinite() || !vz.isFinite()) return
        val r =
            if (covariance != null) {
                sanitizeCovariance3x3(covariance, minDiag = 1e-6) ?: return
            } else if (variance != null) {
                val (varX, varY, varZ) = variance
                if (!varX.isFinite() || !varY.isFinite() || !varZ.isFinite()) return
                if (varX <= 0.0 || varY <= 0.0 || varZ <= 0.0) return
                arrayOf(
                    doubleArrayOf(varX, 0.0, 0.0),
                    doubleArrayOf(0.0, varY, 0.0),
                    doubleArrayOf(0.0, 0.0, varZ),
                )
            } else {
                return
            }
        val innovation = doubleArrayOf(vx - x[3], vy - x[4], vz - x[5])
        updateLinear(innovation, r, stateOffset = 3)
        clipVelocity()
    }

    private fun sanitizeCovariance3x3(
        covariance: Array<DoubleArray>,
        minDiag: Double,
    ): Array<DoubleArray>? {
        if (covariance.size < 3 || covariance[0].size < 3 || covariance[1].size < 3 || covariance[2].size < 3) {
            return null
        }
        val result = Array(3) { DoubleArray(3) }
        for (i in 0 until 3) {
            val diag = covariance[i][i]
            if (!diag.isFinite() || diag <= 0.0) return null
            result[i][i] = diag.coerceIn(minDiag, 1e6)
        }
        for (i in 0 until 3) {
            for (j in i + 1 until 3) {
                val v1 = covariance[i][j]
                val v2 = covariance[j][i]
                if (!v1.isFinite() || !v2.isFinite()) return null
                val maxAbs = sqrt(result[i][i] * result[j][j])
                val avg = (0.5 * (v1 + v2)).coerceIn(-maxAbs, maxAbs)
                result[i][j] = avg
                result[j][i] = avg
            }
        }
        return result
    }

    private fun rememberCoast(
        innovationX: Double,
        innovationY: Double,
        innovationZ: Double,
    ) {
        coast =
            KalmanCoast(
                positionCovariance = Array(3) { i -> DoubleArray(3) { j -> p[i][j] } },
                innovationX = innovationX,
                innovationY = innovationY,
                innovationZ = innovationZ,
            )
    }

    private fun initFromMeasurement(
        zx: Double,
        zy: Double,
        zz: Double,
        rMeasurement: Array<DoubleArray>,
        timeNanos: Long,
    ) {
        x[0] = zx
        x[1] = zy
        x[2] = zz
        x[3] = 0.0
        x[4] = 0.0
        x[5] = 0.0
        x[6] = zx
        x[7] = zy
        x[8] = zz
        for (i in 0 until STATE_DIM) {
            for (j in 0 until STATE_DIM) {
                p[i][j] = 0.0
            }
        }
        for (i in 0 until 3) {
            for (j in 0 until 3) {
                val entry = safePositionCovEntry(rMeasurement, i, j, isDiagonal = i == j)
                p[i][j] = entry
                p[6 + i][6 + j] = entry
            }
        }
        val v0 = GnssMathUtils.KALMAN_INITIAL_VEL_VAR_M2_S2
        p[3][3] = v0
        p[4][4] = v0
        p[5][5] = v0
        lastTimeNanos = timeNanos
        initialized = true
        hasPreviousEpoch = false
        consecutiveRejections = 0
    }

    private fun safePositionCovEntry(
        r: Array<DoubleArray>,
        i: Int,
        j: Int,
        isDiagonal: Boolean,
    ): Double {
        val v = r.getOrNull(i)?.getOrNull(j) ?: return if (isDiagonal) 25.0 else 0.0
        if (!v.isFinite()) return if (isDiagonal) 25.0 else 0.0
        if (isDiagonal) return v.coerceIn(1e-4, 1e6)
        return v.coerceIn(-1e6, 1e6)
    }

    private fun predict(dt: Double) {
        // Preserve pre-predict position in x[6..8] so TDCP relates r_k to r_{k-1} with exact Cov(r_k, r_{k-1}).
        val prevRx = x[0]
        val prevRy = x[1]
        val prevRz = x[2]
        x[0] = prevRx + x[3] * dt
        x[1] = prevRy + x[4] * dt
        x[2] = prevRz + x[5] * dt
        x[6] = prevRx
        x[7] = prevRy
        x[8] = prevRz

        val f = buildF9(dt)
        val fp = matMul9(f, p)
        val fpFt = matMul9(fp, transpose9(f))
        val q = buildProcessNoiseQ9(dt, accelNoiseDensity)
        for (i in 0 until STATE_DIM) {
            for (j in 0 until STATE_DIM) {
                p[i][j] = fpFt[i][j] + q[i][j]
            }
        }
        symmetrize9(p)
        hasPreviousEpoch = true
    }

    private fun updateWithPositionMeasurement(
        innovation: DoubleArray,
        rMeasurement: Array<DoubleArray>,
    ) {
        updateLinear(innovation, rMeasurement, stateOffset = 0)
    }

    /**
     * Kalman update for a 3-vector measurement of states [stateOffset, stateOffset + 3).
     * Position uses offset 0. Velocity uses offset 3. Joseph form, same as before.
     */
    private fun updateLinear(
        innovation: DoubleArray,
        rMeasurement: Array<DoubleArray>,
        stateOffset: Int,
    ) {
        val s =
            Array(3) { i ->
                DoubleArray(3) { j ->
                    (p[stateOffset + i][stateOffset + j] + rMeasurement[i][j]).coerceFinite()
                }
            }
        val sInv = invert3x3(s) ?: return

        val k = Array(STATE_DIM) { DoubleArray(3) }
        for (i in 0 until STATE_DIM) {
            for (j in 0 until 3) {
                var sum = 0.0
                for (m in 0 until 3) {
                    sum += p[i][stateOffset + m] * sInv[m][j]
                }
                k[i][j] = sum
            }
        }

        for (i in 0 until STATE_DIM) {
            for (j in 0 until 3) {
                x[i] += k[i][j] * innovation[j]
            }
        }

        // Joseph form: P = (I - K H) P (I - K H)^T + K R K^T
        val iMinusKh = Array(STATE_DIM) { DoubleArray(STATE_DIM) }
        for (i in 0 until STATE_DIM) {
            for (j in 0 until STATE_DIM) {
                val khEntry = if (j in stateOffset until stateOffset + 3) k[i][j - stateOffset] else 0.0
                iMinusKh[i][j] = (if (i == j) 1.0 else 0.0) - khEntry
            }
        }

        val p1 = matMul9(iMinusKh, p)
        val p2 = matMul9(p1, transpose9(iMinusKh))

        val krKt = Array(STATE_DIM) { DoubleArray(STATE_DIM) }
        for (i in 0 until STATE_DIM) {
            for (j in 0 until STATE_DIM) {
                var sum = 0.0
                for (a in 0 until 3) {
                    for (b in 0 until 3) {
                        sum += k[i][a] * rMeasurement[a][b] * k[j][b]
                    }
                }
                krKt[i][j] = sum
            }
        }
        for (i in 0 until STATE_DIM) {
            for (j in 0 until STATE_DIM) {
                p[i][j] = p2[i][j] + krKt[i][j]
            }
        }
        symmetrize9(p)
    }

    private fun clipVelocity() {
        val vx = x[3]
        val vy = x[4]
        val vz = x[5]
        val spd = sqrt(vx * vx + vy * vy + vz * vz)
        val maxSpd = GnssMathUtils.KALMAN_MAX_SPEED_MPS
        if (spd > maxSpd && spd > 1e-6) {
            val s = maxSpd / spd
            x[3] *= s
            x[4] *= s
            x[5] *= s
            // Scale only the velocity block (indices 3..5) across all 9 rows/cols;
            // scaling >= 3 would corrupt delayed position covariances at 6..8.
            for (i in 0 until STATE_DIM) {
                for (j in 0 until STATE_DIM) {
                    val si = if (i in 3..5) s else 1.0
                    val sj = if (j in 3..5) s else 1.0
                    p[i][j] *= si * sj
                }
            }
            symmetrize9(p)
        }
    }

    /**
     * Mahalanobis norm of the position innovation, or null when S = P₀₀ + R is not invertible
     * (degenerate covariance). Callers must not compare a meters-valued fallback against the
     * sigma threshold — that silently turned the statistical gate into a 4 m absolute gate.
     * The file-level [invert3x3] is used: it rejects non-finite determinants, so a poisoned
     * covariance cannot produce a NaN-filled "inverse".
     */
    private fun mahalanobisInnovation(
        y: DoubleArray,
        rMeasurement: Array<DoubleArray>,
    ): Double? {
        val s = Array(3) { i -> DoubleArray(3) { j -> p[i][j] + rMeasurement[i][j] } }
        val sInv = invert3x3(s) ?: return null
        var d2 = 0.0
        for (i in 0 until 3) for (j in 0 until 3) d2 += y[i] * sInv[i][j] * y[j]
        return sqrt(d2.coerceAtLeast(0.0))
    }

    private companion object {
        const val STATE_DIM = 9
    }
}

private fun Double.coerceFinite(): Double = if (isFinite()) this else 1.0

private fun buildF9(dt: Double): Array<DoubleArray> {
    val f = Array(9) { DoubleArray(9) { 0.0 } }
    for (i in 0 until 3) {
        f[i][i] = 1.0
        f[i][i + 3] = dt
        f[i + 3][i + 3] = 1.0
        f[i + 6][i] = 1.0
    }
    return f
}

private fun transpose9(a: Array<DoubleArray>): Array<DoubleArray> {
    val t = Array(9) { DoubleArray(9) }
    for (i in 0 until 9) {
        for (j in 0 until 9) {
            t[i][j] = a[j][i]
        }
    }
    return t
}

private fun matMul9(
    a: Array<DoubleArray>,
    b: Array<DoubleArray>,
): Array<DoubleArray> {
    val c = Array(9) { DoubleArray(9) }
    for (i in 0 until 9) {
        for (j in 0 until 9) {
            var sum = 0.0
            for (k in 0 until 9) {
                sum += a[i][k] * b[k][j]
            }
            c[i][j] = sum
        }
    }
    return c
}

/** Continuous white acceleration per axis → discrete Q (9×9) for CV model on [r_k, v_k]; r_{k-1} is noiseless copy. */
private fun buildProcessNoiseQ9(
    dt: Double,
    qa: Double,
): Array<DoubleArray> {
    val q = Array(9) { DoubleArray(9) }
    val dt2 = dt * dt
    val dt3 = dt2 * dt
    val q11 = qa * dt3 / 3.0
    val q12 = qa * dt2 / 2.0
    val q22 = qa * dt
    for (axis in 0 until 3) {
        val pi = axis
        val vi = axis + 3
        q[pi][pi] += q11
        q[pi][vi] += q12
        q[vi][pi] += q12
        q[vi][vi] += q22
    }
    return q
}

private fun symmetrize9(p: Array<DoubleArray>) {
    for (i in 0 until 9) {
        for (j in i + 1 until 9) {
            val v = 0.5 * (p[i][j] + p[j][i])
            p[i][j] = v
            p[j][i] = v
        }
    }
}

/** Cramer's rule; returns null if singular. */
private fun invert3x3(m: Array<DoubleArray>): Array<DoubleArray>? {
    val a = m[0][0]
    val b = m[0][1]
    val c = m[0][2]
    val d = m[1][0]
    val e = m[1][1]
    val f = m[1][2]
    val g = m[2][0]
    val h = m[2][1]
    val ii = m[2][2]
    val det =
        a * (e * ii - f * h) -
            b * (d * ii - f * g) +
            c * (d * h - e * g)
    if (!det.isFinite() || abs(det) < 1e-18) return null
    val invDet = 1.0 / det
    return arrayOf(
        doubleArrayOf(
            (e * ii - f * h) * invDet,
            (c * h - b * ii) * invDet,
            (b * f - c * e) * invDet,
        ),
        doubleArrayOf(
            (f * g - d * ii) * invDet,
            (a * ii - c * g) * invDet,
            (c * d - a * f) * invDet,
        ),
        doubleArrayOf(
            (d * h - e * g) * invDet,
            (b * g - a * h) * invDet,
            (a * e - b * d) * invDet,
        ),
    )
}
