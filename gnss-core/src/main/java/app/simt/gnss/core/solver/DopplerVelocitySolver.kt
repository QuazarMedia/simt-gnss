package app.simt.gnss.core.solver

import kotlin.math.abs
import kotlin.math.max

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
    )

    fun solve(observations: List<Observation>): Solution? = solve(observations, RESIDUAL_FLOOR_METERS_PER_SECOND, RESIDUAL_SIGMA)

    /**
     * Same four-unknown line-of-sight solve as [solve], with the caller's residual limit.
     * Time-differenced carrier phase uses it for a displacement: the observation is already
     * the receiver term, so satellite velocity and satellite clock drift are zero.
     */
    fun solve(
        observations: List<Observation>,
        residualFloor: Double,
        residualSigma: Double,
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
        return solveRows(rows, residualFloor, residualSigma)
    }

    private fun solveRows(
        initialRows: List<Observation>,
        residualFloor: Double,
        residualSigma: Double,
    ): Solution? {
        var rows = initialRows
        var excluded = 0
        while (true) {
            val solution = solveOnce(rows) ?: return null
            if (excluded >= MAX_EXCLUSIONS) return solution
            val worst = worstIndex(rows, solution) ?: return solution
            val limit =
                max(
                    residualFloor,
                    residualSigma * rows[worst].sigmaMetersPerSecond,
                )
            if (abs(residual(rows[worst], solution)) <= limit) return solution
            rows = rows.filterIndexed { index, _ -> index != worst }
            excluded++
            if (rows.size < MIN_OBSERVATIONS) return null
        }
    }

    private fun solveOnce(rows: List<Observation>): Solution? {
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
        val cov = solved.covarianceMatrix
        return Solution(
            velocityX = x[0],
            velocityY = x[1],
            velocityZ = x[2],
            clockDriftMetersPerSecond = x[3],
            varianceX = varianceAt(cov, 0),
            varianceY = varianceAt(cov, 1),
            varianceZ = varianceAt(cov, 2),
        )
    }

    private fun varianceAt(
        covariance: Array<DoubleArray>,
        index: Int,
    ): Double {
        val value = covariance.getOrNull(index)?.getOrNull(index) ?: return 1.0
        if (!value.isFinite() || value <= 0.0) return 1.0
        return value.coerceAtLeast(MIN_VARIANCE_M2_PER_S2)
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
    ): Int? {
        var worst = -1
        var worstAbs = 0.0
        for (i in rows.indices) {
            val absResidual = abs(residual(rows[i], solution))
            if (absResidual > worstAbs) {
                worstAbs = absResidual
                worst = i
            }
        }
        return if (worst >= 0) worst else null
    }
}
