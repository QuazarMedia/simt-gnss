package app.simt.gnss.core.solver

import app.simt.gnss.core.utils.GnssMathUtils
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.sqrt

/**
 * Receiver velocity from uncorrected pseudorange rates.
 *
 * One epoch, linear: the unknowns are ECEF velocity and one receiver clock drift.
 * Satellite velocity and satellite clock drift are already removed by the caller.
 * The range-rate model is
 * `PRR = los · (v_sat − v_rx) + clockDrift_rx − clockDrift_sat`,
 * with `los` the unit vector from the receiver to the satellite. Android's
 * pseudorange rate is positive when the satellite is receding, which is this sign.
 *
 * Fewer than [MIN_OBSERVATIONS] valid rates, or a residual that cannot be excluded
 * while keeping that count, returns null. The caller then leaves the inter-epoch
 * filter on its position-only path.
 */
object DopplerVelocitySolver {
    const val MIN_OBSERVATIONS = 5
    private const val MAX_EXCLUSIONS = 2
    private const val RESIDUAL_SIGMA = 4.0
    private const val RESIDUAL_FLOOR_METERS_PER_SECOND = 1.0
    private const val MIN_VARIANCE_M2_PER_S2 = 0.0025

    data class Observation(
        val losX: Double,
        val losY: Double,
        val losZ: Double,
        val satelliteVelocityX: Double,
        val satelliteVelocityY: Double,
        val satelliteVelocityZ: Double,
        /** c · d(Δt_sv)/dt, meters per second. */
        val satelliteClockDriftMetersPerSecond: Double,
        /** Uncorrected pseudorange rate, meters per second. */
        val pseudorangeRateMetersPerSecond: Double,
        val sigmaMetersPerSecond: Double,
    )

    data class Solution(
        val velocityX: Double,
        val velocityY: Double,
        val velocityZ: Double,
        val clockDriftMetersPerSecond: Double,
        val varianceX: Double,
        val varianceY: Double,
        val varianceZ: Double,
        val covarianceEcef: Array<DoubleArray> =
            arrayOf(
                doubleArrayOf(varianceX, 0.0, 0.0),
                doubleArrayOf(0.0, varianceY, 0.0),
                doubleArrayOf(0.0, 0.0, varianceZ),
            ),
    )

    private class InternalSolve(
        val solution: Solution,
        val leverages: DoubleArray,
    )

    fun solve(observations: List<Observation>): Solution? = solve(observations, RESIDUAL_FLOOR_METERS_PER_SECOND, RESIDUAL_SIGMA, MIN_VARIANCE_M2_PER_S2)

    /**
     * Same four-unknown line-of-sight solve as [solve], with the caller's residual limit.
     * Time-differenced carrier phase uses it for a displacement: the observation is already
     * the receiver term, so satellite velocity and satellite clock drift are zero.
     */
    fun solve(
        observations: List<Observation>,
        residualFloor: Double,
        residualSigma: Double,
        minVariance: Double = MIN_VARIANCE_M2_PER_S2,
    ): Solution? {
        val rows =
            observations.filter { obs ->
                obs.sigmaMetersPerSecond.isFinite() &&
                    obs.sigmaMetersPerSecond > 0.0 &&
                    obs.pseudorangeRateMetersPerSecond.isFinite() &&
                    obs.losX.isFinite() &&
                    obs.losY.isFinite() &&
                    obs.losZ.isFinite() &&
                    obs.satelliteVelocityX.isFinite() &&
                    obs.satelliteVelocityY.isFinite() &&
                    obs.satelliteVelocityZ.isFinite() &&
                    obs.satelliteClockDriftMetersPerSecond.isFinite()
            }
        if (rows.size < MIN_OBSERVATIONS) return null
        return solveRows(rows, residualFloor, residualSigma, minVariance)
    }

    private fun solveRows(
        initialRows: List<Observation>,
        residualFloor: Double,
        residualSigma: Double,
        minVariance: Double,
    ): Solution? {
        var rows = initialRows
        var excluded = 0
        while (true) {
            val solved = solveOnce(rows, minVariance) ?: return null
            if (excluded >= MAX_EXCLUSIONS) return solved.solution
            val worst = worstIndex(rows, solved.solution, solved.leverages, residualFloor, residualSigma) ?: return solved.solution
            val redundancy = sqrt((1.0 - solved.leverages[worst]).coerceAtLeast(1e-6))
            val standardizedResidual = abs(residual(rows[worst], solved.solution)) / redundancy
            val limit =
                max(
                    residualFloor,
                    residualSigma * rows[worst].sigmaMetersPerSecond,
                )
            if (standardizedResidual <= limit) return solved.solution
            rows = rows.filterIndexed { index, _ -> index != worst }
            excluded++
            if (rows.size < MIN_OBSERVATIONS) return null
        }
    }

    private fun solveOnce(
        rows: List<Observation>,
        minVariance: Double,
    ): InternalSolve? {
        val design = Array(rows.size) { DoubleArray(4) }
        val observed = DoubleArray(rows.size)
        val weights = DoubleArray(rows.size)
        for (i in rows.indices) {
            val obs = rows[i]
            design[i][0] = -obs.losX
            design[i][1] = -obs.losY
            design[i][2] = -obs.losZ
            design[i][3] = 1.0
            val geometric =
                obs.losX * obs.satelliteVelocityX +
                    obs.losY * obs.satelliteVelocityY +
                    obs.losZ * obs.satelliteVelocityZ
            observed[i] = obs.pseudorangeRateMetersPerSecond - geometric + obs.satelliteClockDriftMetersPerSecond
            weights[i] = 1.0 / (obs.sigmaMetersPerSecond * obs.sigmaMetersPerSecond)
        }
        val solved =
            GnssPositionSolver.solve(
                design,
                observed,
                weights,
                accuracy = GnssPositionSolver.Accuracy.HIGH,
                needCovariance = true,
            ) ?: return null
        val x = solved.deltaX
        if (x.size < 4 || x.any { !it.isFinite() }) return null
        val q = solved.covarianceMatrix
        val cov3 = sanitizeCovariance3x3(q, minVariance)
        val leverages = computeLeverages(design, weights, q)
        return InternalSolve(
            solution =
                Solution(
                    velocityX = x[0],
                    velocityY = x[1],
                    velocityZ = x[2],
                    clockDriftMetersPerSecond = x[3],
                    varianceX = cov3[0][0],
                    varianceY = cov3[1][1],
                    varianceZ = cov3[2][2],
                    covarianceEcef = cov3,
                ),
            leverages = leverages,
        )
    }

    private fun sanitizeCovariance3x3(
        covariance: Array<DoubleArray>,
        minVariance: Double,
    ): Array<DoubleArray> {
        val floor = if (minVariance.isFinite() && minVariance > 0.0) minVariance else MIN_VARIANCE_M2_PER_S2
        val result = Array(3) { DoubleArray(3) }
        for (i in 0 until 3) {
            val raw = covariance.getOrNull(i)?.getOrNull(i)
            result[i][i] =
                if (raw != null && raw.isFinite() && raw > 0.0) {
                    raw.coerceAtLeast(floor)
                } else {
                    1.0.coerceAtLeast(floor)
                }
        }
        for (i in 0 until 3) {
            for (j in i + 1 until 3) {
                val v1 = covariance.getOrNull(i)?.getOrNull(j)
                val v2 = covariance.getOrNull(j)?.getOrNull(i)
                val sanitized =
                    if (v1 != null && v2 != null && v1.isFinite() && v2.isFinite()) {
                        val maxAbs = sqrt(result[i][i] * result[j][j])
                        (0.5 * (v1 + v2)).coerceIn(-maxAbs, maxAbs)
                    } else {
                        0.0
                    }
                result[i][j] = sanitized
                result[j][i] = sanitized
            }
        }
        return result
    }

    private fun computeLeverages(
        design: Array<DoubleArray>,
        weights: DoubleArray,
        q: Array<DoubleArray>,
    ): DoubleArray {
        val leverages = DoubleArray(design.size)
        if (q.size < 4 || q.any { it.size < 4 }) return leverages
        for (i in design.indices) {
            var aQa = 0.0
            for (j in 0 until 4) {
                var qAi = 0.0
                for (k in 0 until 4) {
                    qAi += q[j][k] * design[i][k]
                }
                aQa += design[i][j] * qAi
            }
            val hii = weights[i] * aQa
            leverages[i] = if (hii.isFinite()) hii.coerceIn(0.0, GnssMathUtils.RAIM_LEVERAGE_MAX) else 0.0
        }
        return leverages
    }

    private fun residual(
        obs: Observation,
        solution: Solution,
    ): Double {
        val geometric =
            obs.losX * (obs.satelliteVelocityX - solution.velocityX) +
                obs.losY * (obs.satelliteVelocityY - solution.velocityY) +
                obs.losZ * (obs.satelliteVelocityZ - solution.velocityZ)
        return obs.pseudorangeRateMetersPerSecond -
            (geometric + solution.clockDriftMetersPerSecond - obs.satelliteClockDriftMetersPerSecond)
    }

    private fun worstIndex(
        rows: List<Observation>,
        solution: Solution,
        leverages: DoubleArray,
        residualFloor: Double,
        residualSigma: Double,
    ): Int? {
        var worst = -1
        var worstRatio = 0.0
        for (i in rows.indices) {
            val redundancy = sqrt((1.0 - leverages[i]).coerceAtLeast(1e-6))
            val standardized = abs(residual(rows[i], solution)) / redundancy
            val limit = max(residualFloor, residualSigma * rows[i].sigmaMetersPerSecond).coerceAtLeast(1e-12)
            val ratio = standardized / limit
            if (ratio > worstRatio) {
                worstRatio = ratio
                worst = i
            }
        }
        return if (worst >= 0) worst else null
    }
}
