package app.simt.gnss.core.solver

import kotlin.math.abs
import kotlin.math.sqrt

internal object GnssPositionSolverCholesky {
    /**
     * @param needCovariance When `false`, only [GnssPositionSolver.WlsSolution.deltaX] is computed
     *   and [GnssPositionSolver.WlsSolution.covarianceMatrix] is returned empty. The WLS loop solves
     *   for the state update on every iteration but only consumes the covariance once (at
     *   convergence), so skipping the inverse on intermediate iterations avoids an O(m³) step per
     *   iteration.
     */
    fun solve(
        A: Array<DoubleArray>,
        b: DoubleArray,
        w: DoubleArray,
        needCovariance: Boolean = true,
    ): GnssPositionSolver.WlsSolution? {
        val n = A.size
        if (n == 0) return null
        val m = A[0].size
        if (m == 0 || b.size != n || w.size != n) return null

        val atwa = Array(m) { DoubleArray(m) }
        for (i in 0 until m) {
            for (j in 0 until m) {
                var sum = 0.0
                for (k in 0 until n) {
                    val wk = w[k]
                    if (wk <= 0.0 || !wk.isFinite()) continue
                    sum += A[k][i] * wk * A[k][j]
                }
                atwa[i][j] = sum
            }
        }
        val atwb = DoubleArray(m)
        for (i in 0 until m) {
            var sum = 0.0
            for (k in 0 until n) {
                val wk = w[k]
                if (wk <= 0.0 || !wk.isFinite()) continue
                sum += A[k][i] * wk * b[k]
            }
            atwb[i] = sum
        }

        // Factor AᵀWA once and reuse the Cholesky factor for both the state update and the
        // (optional) inverse. The previous implementation re-factorized the matrix once per
        // right-hand side when inverting, making the inverse O(m⁴); reusing the factor makes it
        // O(m³) (one factorization + m back-substitutions of O(m²) each).
        val l = decompose(atwa) ?: return null
        val deltaX = solveWithFactor(l, atwb) ?: return null
        val inverseH =
            if (needCovariance) {
                invertFromFactor(l) ?: return null
            } else {
                emptyArray()
            }

        return GnssPositionSolver.WlsSolution(deltaX, inverseH)
    }

    /** Cholesky decomposition H = L Lᵀ; returns the lower-triangular factor L, or null if not SPD. */
    private fun decompose(H: Array<DoubleArray>): Array<DoubleArray>? {
        val n = H.size
        // Relative positive-definite threshold: normal equations square the condition
        // number, and an absolute d <= 0 guard let pivots down to ~1e-300 pass — finite but
        // garbage solutions with up to ~1e12 amplification on degenerate geometry.
        var maxDiagonal = 0.0
        for (i in 0 until n) {
            val h = H[i][i]
            if (!h.isFinite()) return null
            if (h > maxDiagonal) maxDiagonal = h
        }
        val pivotFloor = (maxDiagonal * 1e-12).coerceAtLeast(Double.MIN_VALUE)
        val l = Array(n) { DoubleArray(n) }
        for (i in 0 until n) {
            for (j in 0..i) {
                var sum = 0.0
                for (k in 0 until j) {
                    sum += l[i][k] * l[j][k]
                }
                if (i == j) {
                    val d = H[i][i] - sum
                    if (!d.isFinite() || d < pivotFloor) return null
                    l[i][j] = sqrt(d)
                } else {
                    l[i][j] = (H[i][j] - sum) / l[j][j]
                }
            }
        }
        return l
    }

    /** Solves H x = g given the Cholesky factor L of H (forward then back substitution). */
    private fun solveWithFactor(
        l: Array<DoubleArray>,
        g: DoubleArray,
    ): DoubleArray? {
        val n = g.size

        val y = DoubleArray(n)
        for (i in 0 until n) {
            var sum = 0.0
            for (k in 0 until i) {
                sum += l[i][k] * y[k]
            }
            if (abs(l[i][i]) < 1e-12) return null
            y[i] = (g[i] - sum) / l[i][i]
        }

        val x = DoubleArray(n)
        for (i in n - 1 downTo 0) {
            var sum = 0.0
            for (k in i + 1 until n) {
                sum += l[k][i] * x[k]
            }
            if (abs(l[i][i]) < 1e-12) return null
            x[i] = (y[i] - sum) / l[i][i]
        }

        return x
    }

    /** Computes H⁻¹ from the Cholesky factor L by solving L Lᵀ xᵢ = eᵢ for each unit column. */
    private fun invertFromFactor(l: Array<DoubleArray>): Array<DoubleArray>? {
        val n = l.size
        val inverse = Array(n) { DoubleArray(n) }
        val e = DoubleArray(n)
        for (col in 0 until n) {
            e.fill(0.0)
            e[col] = 1.0
            val column = solveWithFactor(l, e) ?: return null
            for (row in 0 until n) {
                inverse[row][col] = column[row]
            }
        }
        return inverse
    }
}
