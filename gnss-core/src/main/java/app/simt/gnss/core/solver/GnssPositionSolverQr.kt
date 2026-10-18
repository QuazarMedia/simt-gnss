package app.simt.gnss.core.solver

import kotlin.math.abs
import kotlin.math.sqrt

internal object GnssPositionSolverQr {
    private const val EPS = 1e-14

    private data class QrDecompositionResult(
        val r: Array<DoubleArray>,
        val qtb: DoubleArray,
    )

    /**
     * @param needCovariance When `false`, only [GnssPositionSolver.WlsSolution.deltaX] is computed
     *   and [GnssPositionSolver.WlsSolution.covarianceMatrix] is returned empty. The WLS loop solves
     *   for the state update on every iteration but only consumes the covariance once (at
     *   convergence), so skipping the R⁻¹ R⁻ᵀ assembly on intermediate iterations avoids an O(m³)
     *   step per iteration.
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
        if (m == 0 || n < m || b.size != n || w.size != n) return null

        val weightedA = Array(n) { DoubleArray(m) }
        val weightedB = DoubleArray(n)
        for (i in 0 until n) {
            val wi = w[i]
            if (!wi.isFinite() || wi < 0.0) return null
            val sqrtW = sqrt(wi)
            weightedB[i] = b[i] * sqrtW
            for (j in 0 until m) {
                weightedA[i][j] = A[i][j] * sqrtW
            }
        }

        val qr = decomposeHouseholderQr(weightedA, weightedB) ?: return null
        val deltaX = backSubstituteUpperTriangular(qr.r, qr.qtb) ?: return null
        val inverseH =
            if (needCovariance) {
                covarianceFromUpperTriangularR(qr.r) ?: return null
            } else {
                emptyArray()
            }

        return GnssPositionSolver.WlsSolution(deltaX, inverseH)
    }

    private fun decomposeHouseholderQr(
        weightedA: Array<DoubleArray>,
        weightedB: DoubleArray,
    ): QrDecompositionResult? {
        val n = weightedA.size
        val m = weightedA[0].size
        val r = Array(n) { i -> weightedA[i].clone() }
        val qtb = weightedB.clone()

        for (k in 0 until m) {
            var normX = 0.0
            for (i in k until n) {
                normX += r[i][k] * r[i][k]
            }
            normX = sqrt(normX)
            if (!normX.isFinite()) return null
            if (normX < EPS) continue

            val x0 = r[k][k]
            val alpha = if (x0 >= 0.0) -normX else normX

            val v = DoubleArray(n - k)
            v[0] = x0 - alpha
            for (i in k + 1 until n) {
                v[i - k] = r[i][k]
            }

            var vNormSq = 0.0
            for (i in v.indices) {
                vNormSq += v[i] * v[i]
            }
            if (!vNormSq.isFinite()) return null
            if (vNormSq < EPS) continue

            for (j in k until m) {
                var dot = 0.0
                for (i in k until n) {
                    dot += v[i - k] * r[i][j]
                }
                val factor = 2.0 * dot / vNormSq
                for (i in k until n) {
                    r[i][j] -= factor * v[i - k]
                }
            }

            var dotB = 0.0
            for (i in k until n) {
                dotB += v[i - k] * qtb[i]
            }
            val factorB = 2.0 * dotB / vNormSq
            for (i in k until n) {
                qtb[i] -= factorB * v[i - k]
            }
        }

        val upperR = Array(m) { i -> DoubleArray(m) { j -> if (j < i) 0.0 else r[i][j] } }
        val reducedQtb = DoubleArray(m) { qtb[it] }
        return QrDecompositionResult(upperR, reducedQtb)
    }

    private fun backSubstituteUpperTriangular(
        r: Array<DoubleArray>,
        y: DoubleArray,
    ): DoubleArray? {
        val n = y.size
        val x = DoubleArray(n)
        for (i in n - 1 downTo 0) {
            var sum = 0.0
            for (k in i + 1 until n) {
                sum += r[i][k] * x[k]
            }
            val rii = r[i][i]
            if (!rii.isFinite() || abs(rii) < EPS) return null
            x[i] = (y[i] - sum) / rii
            if (!x[i].isFinite()) return null
        }
        return x
    }

    private fun covarianceFromUpperTriangularR(r: Array<DoubleArray>): Array<DoubleArray>? {
        val n = r.size
        val invR = Array(n) { DoubleArray(n) }

        for (col in 0 until n) {
            for (row in n - 1 downTo 0) {
                var sum = 0.0
                for (k in row + 1 until n) {
                    sum += r[row][k] * invR[k][col]
                }
                val rhs = if (row == col) 1.0 else 0.0
                val diag = r[row][row]
                if (!diag.isFinite() || abs(diag) < EPS) return null
                invR[row][col] = (rhs - sum) / diag
                if (!invR[row][col].isFinite()) return null
            }
        }

        val covariance = Array(n) { DoubleArray(n) }
        for (i in 0 until n) {
            for (j in 0 until n) {
                var sum = 0.0
                for (k in 0 until n) {
                    sum += invR[i][k] * invR[j][k]
                }
                covariance[i][j] = sum
            }
        }
        return covariance
    }
}
