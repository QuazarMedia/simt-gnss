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
        lastTimeNanos = 0L
        consecutiveRejections = 0
        coast = null
        x.fill(0.0)
        for (row in p) row.fill(0.0)
    }

    /**
     * Fuses a new WLS ECEF fix. Returns **filtered** ECEF position (meters).
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
        displacementEcef: Triple<Double, Double, Double>? = null,
        displacementVarianceM2: Triple<Double, Double, Double>? = null,
    ): Triple<Double, Double, Double> {
        val (zx, zy, zz) = wlsEcef

        // A non-finite measurement cannot be fused or adopted; coast on the current state
        // (or report it back unfiltered before initialization).
        if (!zx.isFinite() || !zy.isFinite() || !zz.isFinite()) {
            return if (initialized) {
                rememberCoast(0.0, 0.0, 0.0)
                positioned(velocityEcef, velocityVarianceM2PerS2, displacementEcef, displacementVarianceM2)
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
            return positioned(velocityEcef, velocityVarianceM2PerS2, displacementEcef, displacementVarianceM2)
        }

        val dtSec =
            (timeNanos - lastTimeNanos).toDouble() / GnssMathUtils.NANOS_PER_SECOND.toDouble()
        lastTimeNanos = timeNanos

        if (dtSec <= 0.0 ||
            dtSec * GnssMathUtils.NANOS_PER_SECOND.toDouble() > GnssMathUtils.KALMAN_RESET_GAP_NANOS.toDouble()
        ) {
            initFromMeasurement(zx, zy, zz, r, timeNanos)
            coast = null
            return positioned(velocityEcef, velocityVarianceM2PerS2, displacementEcef, displacementVarianceM2)
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
            } else {
                // Single/short outlier burst: REJECT it and coast on the prediction. Re-initializing
                // onto the gated measurement would pass every multipath spike through at full gain
                // and destroy the converged velocity/covariance state.
                rememberCoast(y0, y1, y2)
            }
            return positioned(velocityEcef, velocityVarianceM2PerS2, displacementEcef, displacementVarianceM2)
        }
        consecutiveRejections = 0
        coast = null

        updateWithPositionMeasurement(doubleArrayOf(y0, y1, y2), r)
        clipVelocity()
        return positioned(velocityEcef, velocityVarianceM2PerS2, displacementEcef, displacementVarianceM2)
    }

    /** ECEF velocity after the last [fuse]. Zero until a velocity measurement is applied. */
    fun velocityEcef(): Triple<Double, Double, Double> = Triple(x[3], x[4], x[5])

    private fun positioned(
        velocityEcef: Triple<Double, Double, Double>?,
        velocityVarianceM2PerS2: Triple<Double, Double, Double>?,
        displacementEcef: Triple<Double, Double, Double>?,
        displacementVarianceM2: Triple<Double, Double, Double>?,
    ): Triple<Double, Double, Double> {
        applyVelocityMeasurement(velocityEcef, velocityVarianceM2PerS2)
        applyDisplacementMeasurement(displacementEcef, displacementVarianceM2)
        return Triple(x[0], x[1], x[2])
    }

    /** Position measurement of the previous published point plus the carrier displacement. */
    private fun applyDisplacementMeasurement(
        positionEcef: Triple<Double, Double, Double>?,
        variance: Triple<Double, Double, Double>?,
    ) {
        if (positionEcef == null || variance == null || !initialized) return
        val (px, py, pz) = positionEcef
        val (varX, varY, varZ) = variance
        if (!px.isFinite() || !py.isFinite() || !pz.isFinite()) return
        if (!varX.isFinite() || !varY.isFinite() || !varZ.isFinite()) return
        if (varX <= 0.0 || varY <= 0.0 || varZ <= 0.0) return
        val innovation = doubleArrayOf(px - x[0], py - x[1], pz - x[2])
        val r =
            arrayOf(
                doubleArrayOf(varX, 0.0, 0.0),
                doubleArrayOf(0.0, varY, 0.0),
                doubleArrayOf(0.0, 0.0, varZ),
            )
        updateLinear(innovation, r, stateOffset = 0)
        clipVelocity()
    }

    private fun applyVelocityMeasurement(
        velocityEcef: Triple<Double, Double, Double>?,
        variance: Triple<Double, Double, Double>?,
    ) {
        if (velocityEcef == null || variance == null || !initialized) return
        val (vx, vy, vz) = velocityEcef
        val (varX, varY, varZ) = variance
        if (!vx.isFinite() || !vy.isFinite() || !vz.isFinite()) return
        if (!varX.isFinite() || !varY.isFinite() || !varZ.isFinite()) return
        if (varX <= 0.0 || varY <= 0.0 || varZ <= 0.0) return
        val innovation = doubleArrayOf(vx - x[3], vy - x[4], vz - x[5])
        val r =
            arrayOf(
                doubleArrayOf(varX, 0.0, 0.0),
                doubleArrayOf(0.0, varY, 0.0),
                doubleArrayOf(0.0, 0.0, varZ),
            )
        updateLinear(innovation, r, stateOffset = 3)
        clipVelocity()
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
        for (i in 0 until STATE_DIM) {
            for (j in 0 until STATE_DIM) {
                p[i][j] = 0.0
            }
        }
        for (i in 0 until 3) {
            for (j in 0 until 3) {
                p[i][j] = safePositionCovEntry(rMeasurement, i, j, isDiagonal = i == j)
            }
        }
        val v0 = GnssMathUtils.KALMAN_INITIAL_VEL_VAR_M2_S2
        p[3][3] = v0
        p[4][4] = v0
        p[5][5] = v0
        lastTimeNanos = timeNanos
        initialized = true
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
        // x := F x
        val rx = x[0] + x[3] * dt
        val ry = x[1] + x[4] * dt
        val rz = x[2] + x[5] * dt
        x[0] = rx
        x[1] = ry
        x[2] = rz

        val f = buildF(dt)
        val fp = matMul6(f, p)
        val fpFt = matMul6(fp, transpose6(f))
        val q = buildProcessNoiseQ(dt, accelNoiseDensity)
        for (i in 0 until STATE_DIM) {
            for (j in 0 until STATE_DIM) {
                p[i][j] = fpFt[i][j] + q[i][j]
            }
        }
        symmetrize6(p)
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

        val p1 = matMul6(iMinusKh, p)
        val p2 = matMul6(p1, transpose6(iMinusKh))

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
        symmetrize6(p)
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
            // Scaling velocity by s means Var(v') = s² Var(v), Cov(r,v') = s Cov(r,v).
            // Previous implementation used a single `s` factor for both blocks, under-inflating
            // the velocity variance and stranding the filter at a pessimistic covariance.
            for (i in 0 until STATE_DIM) {
                for (j in 0 until STATE_DIM) {
                    val si = if (i >= 3) s else 1.0
                    val sj = if (j >= 3) s else 1.0
                    p[i][j] *= si * sj
                }
            }
            symmetrize6(p)
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
        const val STATE_DIM = 6
    }
}

private fun Double.coerceFinite(): Double = if (isFinite()) this else 1.0

private fun buildF(dt: Double): Array<DoubleArray> {
    val f = Array(6) { DoubleArray(6) { 0.0 } }
    for (i in 0 until 3) {
        f[i][i] = 1.0
        f[i][i + 3] = dt
        f[i + 3][i + 3] = 1.0
    }
    return f
}

private fun transpose6(a: Array<DoubleArray>): Array<DoubleArray> {
    val t = Array(6) { DoubleArray(6) }
    for (i in 0 until 6) {
        for (j in 0 until 6) {
            t[i][j] = a[j][i]
        }
    }
    return t
}

private fun matMul6(
    a: Array<DoubleArray>,
    b: Array<DoubleArray>,
): Array<DoubleArray> {
    val c = Array(6) { DoubleArray(6) }
    for (i in 0 until 6) {
        for (j in 0 until 6) {
            var sum = 0.0
            for (k in 0 until 6) {
                sum += a[i][k] * b[k][j]
            }
            c[i][j] = sum
        }
    }
    return c
}

/** Continuous white acceleration per axis → discrete Q (6×6) for CV model, axes (r,v) interleaved by component. */
private fun buildProcessNoiseQ(
    dt: Double,
    qa: Double,
): Array<DoubleArray> {
    val q = Array(6) { DoubleArray(6) }
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

private fun symmetrize6(p: Array<DoubleArray>) {
    for (i in 0 until 6) {
        for (j in i + 1 until 6) {
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
