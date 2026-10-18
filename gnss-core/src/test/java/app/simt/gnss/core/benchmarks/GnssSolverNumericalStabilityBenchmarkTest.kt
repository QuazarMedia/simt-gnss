package app.simt.gnss.core.benchmarks

import app.simt.gnss.core.solver.GnssPositionSolver
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.abs
import kotlin.math.sqrt

class GnssSolverNumericalStabilityBenchmarkTest {
    private fun l2Norm(v: DoubleArray): Double = sqrt(v.sumOf { it * it })

    private fun buildIllConditionedDesignMatrix(rows: Int): Array<DoubleArray> {
        val matrix = Array(rows) { DoubleArray(4) }
        for (i in 0 until rows) {
            val t = 0.999 + 0.002 * i.toDouble() / (rows - 1).toDouble()
            matrix[i][0] = 1.0
            matrix[i][1] = t
            matrix[i][2] = t * t
            matrix[i][3] = t * t * t
        }
        return matrix
    }

    private fun buildGeometryMatrix(
        rows: Int,
        span: Double,
    ): Array<DoubleArray> {
        val matrix = Array(rows) { DoubleArray(4) }
        for (i in 0 until rows) {
            val centered = i.toDouble() - (rows - 1).toDouble() / 2.0
            val t = centered * span
            matrix[i][0] = 1.0
            matrix[i][1] = t
            matrix[i][2] = t * t
            matrix[i][3] = t * t * t
        }
        return matrix
    }

    private fun residualNorm(
        a: Array<DoubleArray>,
        x: DoubleArray,
        b: DoubleArray,
    ): Double {
        val ax = multiply(a, x)
        return l2Norm(DoubleArray(b.size) { i -> ax[i] - b[i] })
    }

    private fun jacobiEigenvaluesSymmetric(matrix: Array<DoubleArray>): DoubleArray {
        val n = matrix.size
        val a = Array(n) { i -> matrix[i].clone() }
        val maxSweeps = 100

        repeat(maxSweeps) {
            var p = 0
            var q = 1
            var maxOffDiag = 0.0
            for (i in 0 until n) {
                for (j in i + 1 until n) {
                    val v = abs(a[i][j])
                    if (v > maxOffDiag) {
                        maxOffDiag = v
                        p = i
                        q = j
                    }
                }
            }

            if (maxOffDiag < 1e-14) return DoubleArray(n) { a[it][it] }

            val app = a[p][p]
            val aqq = a[q][q]
            val apq = a[p][q]

            val phi = 0.5 * kotlin.math.atan2(2.0 * apq, aqq - app)
            val c = kotlin.math.cos(phi)
            val s = kotlin.math.sin(phi)

            for (k in 0 until n) {
                if (k != p && k != q) {
                    val aik = a[k][p]
                    val akq = a[k][q]
                    a[k][p] = c * aik - s * akq
                    a[p][k] = a[k][p]
                    a[k][q] = s * aik + c * akq
                    a[q][k] = a[k][q]
                }
            }

            val newApp = c * c * app - 2.0 * s * c * apq + s * s * aqq
            val newAqq = s * s * app + 2.0 * s * c * apq + c * c * aqq
            a[p][p] = newApp
            a[q][q] = newAqq
            a[p][q] = 0.0
            a[q][p] = 0.0
        }

        return DoubleArray(n) { a[it][it] }
    }

    private fun covarianceConditionNumber(covariance: Array<DoubleArray>): Double {
        val eigen = jacobiEigenvaluesSymmetric(covariance).map { abs(it) }.sorted()
        val minEig = eigen.firstOrNull { it > 1e-18 } ?: 1e-18
        val maxEig = eigen.lastOrNull() ?: 1.0
        return maxEig / minEig
    }

    private fun multiply(
        a: Array<DoubleArray>,
        x: DoubleArray,
    ): DoubleArray {
        val out = DoubleArray(a.size)
        for (i in a.indices) {
            var sum = 0.0
            for (j in x.indices) {
                sum += a[i][j] * x[j]
            }
            out[i] = sum
        }
        return out
    }

    private fun solveByNormalEquationCholesky(
        a: Array<DoubleArray>,
        b: DoubleArray,
        w: DoubleArray,
    ): DoubleArray? {
        val n = a.size
        val m = a[0].size
        val atwa = Array(m) { DoubleArray(m) }
        val atwb = DoubleArray(m)

        for (i in 0 until m) {
            for (j in 0 until m) {
                var sum = 0.0
                for (k in 0 until n) {
                    sum += a[k][i] * w[k] * a[k][j]
                }
                atwa[i][j] = sum
            }
            var rhs = 0.0
            for (k in 0 until n) {
                rhs += a[k][i] * w[k] * b[k]
            }
            atwb[i] = rhs
        }

        return choleskySolve(atwa, atwb)
    }

    private fun choleskySolve(
        h: Array<DoubleArray>,
        g: DoubleArray,
    ): DoubleArray? {
        val n = g.size
        val l = Array(n) { DoubleArray(n) }

        for (i in 0 until n) {
            for (j in 0..i) {
                var sum = 0.0
                for (k in 0 until j) {
                    sum += l[i][k] * l[j][k]
                }
                if (i == j) {
                    val d = h[i][i] - sum
                    if (d <= 0.0) return null
                    l[i][j] = sqrt(d)
                } else {
                    if (abs(l[j][j]) < 1e-12) return null
                    l[i][j] = (h[i][j] - sum) / l[j][j]
                }
            }
        }

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

    @Test
    fun benchmarkQrSolverBeatsNormalEquationCholeskyOnIllConditionedSystem() {
        val rows = 24
        val a = buildIllConditionedDesignMatrix(rows)
        val trueX = doubleArrayOf(12.0, 1.8, -1.8, 0.35)
        val cleanB = multiply(a, trueX)
        val b = DoubleArray(rows) { idx -> cleanB[idx] + (if (idx % 2 == 0) 1e-7 else -1e-7) }
        val w = DoubleArray(rows) { 1.0 }

        val qrSolution = GnssPositionSolver.solve(a, b, w)
        assertNotNull("QR-based solver should produce a solution", qrSolution)

        val qrDelta = qrSolution!!.deltaX
        val qrError = l2Norm(DoubleArray(trueX.size) { i -> qrDelta[i] - trueX[i] })
        val qrResidual = residualNorm(a, qrDelta, b)

        val choleskyDelta = solveByNormalEquationCholesky(a, b, w)
        if (choleskyDelta == null) {
            println("GNSS_BENCHMARK_QR_STABILITY cholesky_failed=true qr_error=$qrError qr_residual=$qrResidual")
            assertTrue("QR residual should remain small even when Cholesky fails", qrResidual < 1e-3)
            return
        }

        val choleskyError = l2Norm(DoubleArray(trueX.size) { i -> choleskyDelta[i] - trueX[i] })
        val choleskyResidual = residualNorm(a, choleskyDelta, b)
        val errorImprovement = choleskyError / qrError
        val residualImprovement = choleskyResidual / qrResidual

        println(
            "GNSS_BENCHMARK_QR_STABILITY cholesky_error=$choleskyError qr_error=$qrError error_improvement_x=$errorImprovement " +
                "cholesky_residual=$choleskyResidual qr_residual=$qrResidual residual_improvement_x=$residualImprovement",
        )

        assertTrue("QR should be at least as accurate as normal-equation Cholesky", qrError <= choleskyError * 1.001)
        assertTrue("QR should have equal or lower residual than Cholesky", qrResidual <= choleskyResidual * 1.001)
    }

    @Test
    fun benchmarkQrCovarianceConditionWorsensWithPoorGeometry() {
        val rows = 24
        val trueX = doubleArrayOf(4.0, -1.2, 0.8, 0.15)
        val weights = DoubleArray(rows) { 1.0 }

        val wellGeometry = buildGeometryMatrix(rows = rows, span = 0.25)
        val mediumGeometry = buildGeometryMatrix(rows = rows, span = 0.05)
        val poorGeometry = buildGeometryMatrix(rows = rows, span = 0.01)

        val wellSolution = GnssPositionSolver.solve(wellGeometry, multiply(wellGeometry, trueX), weights)
        val mediumSolution = GnssPositionSolver.solve(mediumGeometry, multiply(mediumGeometry, trueX), weights)
        val poorSolution = GnssPositionSolver.solve(poorGeometry, multiply(poorGeometry, trueX), weights)

        assertNotNull("QR should solve well-conditioned geometry", wellSolution)
        assertNotNull("QR should solve medium-conditioned geometry", mediumSolution)
        assertNotNull("QR should solve poor-conditioned geometry", poorSolution)

        val conditionWell = covarianceConditionNumber(wellSolution!!.covarianceMatrix)
        val conditionMedium = covarianceConditionNumber(mediumSolution!!.covarianceMatrix)
        val conditionPoor = covarianceConditionNumber(poorSolution!!.covarianceMatrix)

        println(
            "GNSS_BENCHMARK_QR_COV_CONDITION cond_well=$conditionWell cond_medium=$conditionMedium cond_poor=$conditionPoor growth_well_to_poor_x=${conditionPoor / conditionWell}",
        )

        assertTrue("Condition number should worsen as geometry degrades", conditionWell < conditionMedium)
        assertTrue("Condition number should worsen further for poor geometry", conditionMedium < conditionPoor)
        assertTrue("Expected at least 100x condition growth from well to poor geometry", conditionPoor / conditionWell > 100.0)
    }
}
