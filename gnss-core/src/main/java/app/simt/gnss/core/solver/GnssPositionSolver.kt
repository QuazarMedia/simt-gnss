package app.simt.gnss.core.solver

/**
 * Weighted Least Squares (WLS) solver selector for GNSS positioning.
 * Uses QR for high-accuracy mode and Cholesky for medium-accuracy mode.
 */
object GnssPositionSolver {
    enum class Accuracy {
        HIGH,
        MEDIUM,
    }

    data class WlsSolution(
        val deltaX: DoubleArray,
        val covarianceMatrix: Array<DoubleArray>,
    )

    /**
     * @param needCovariance When `false`, the solver returns only [WlsSolution.deltaX] and leaves
     *   [WlsSolution.covarianceMatrix] empty. Callers that iterate (WLS) should pass `false` while
     *   converging and request the covariance once on the final, converged solve — the covariance
     *   (DOP / accuracy / RAIM leverage) is only consumed for the accepted fix.
     */
    fun solve(
        A: Array<DoubleArray>,
        b: DoubleArray,
        w: DoubleArray,
        accuracy: Accuracy = Accuracy.HIGH,
        needCovariance: Boolean = true,
    ): WlsSolution? {
        return when (accuracy) {
            Accuracy.HIGH -> GnssPositionSolverQr.solve(A, b, w, needCovariance)
            Accuracy.MEDIUM -> GnssPositionSolverCholesky.solve(A, b, w, needCovariance)
        }
    }
}
