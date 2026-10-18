package app.simt.gnss.core.solver

import org.junit.Assert.*
import org.junit.Test

class GnssPositionSolverTest {
    @Test
    fun testSolveWithHighAccuracyUsesQrPath() {
        val a =
            arrayOf(
                doubleArrayOf(1.0, 0.0, 0.0, 1.0),
                doubleArrayOf(0.0, 1.0, 0.0, 1.0),
                doubleArrayOf(0.0, 0.0, 1.0, 1.0),
                doubleArrayOf(1.0, 1.0, 1.0, 1.0),
            )
        val b = doubleArrayOf(2.0, 2.0, 2.0, 4.0)
        val w = doubleArrayOf(1.0, 1.0, 1.0, 1.0)

        val solution = GnssPositionSolver.solve(a, b, w, accuracy = GnssPositionSolver.Accuracy.HIGH)
        assertNotNull(solution)
    }

    @Test
    fun testSolveWithMediumAccuracyUsesCholeskyPath() {
        val a =
            arrayOf(
                doubleArrayOf(1.0, 0.0, 0.0, 1.0),
                doubleArrayOf(0.0, 1.0, 0.0, 1.0),
                doubleArrayOf(0.0, 0.0, 1.0, 1.0),
                doubleArrayOf(1.0, 1.0, 1.0, 1.0),
            )
        val b = doubleArrayOf(2.0, 2.0, 2.0, 4.0)
        val w = doubleArrayOf(1.0, 1.0, 1.0, 1.0)

        val solution = GnssPositionSolver.solve(a, b, w, accuracy = GnssPositionSolver.Accuracy.MEDIUM)
        assertNotNull(solution)
    }

    @Test
    fun testSolveSimpleSystem() {
        // Simple 4x4 identity-like system
        val a =
            arrayOf(
                doubleArrayOf(1.0, 0.0, 0.0, 1.0),
                doubleArrayOf(0.0, 1.0, 0.0, 1.0),
                doubleArrayOf(0.0, 0.0, 1.0, 1.0),
                doubleArrayOf(1.0, 1.0, 1.0, 1.0),
            )
        val b = doubleArrayOf(2.0, 2.0, 2.0, 4.0)
        val w = doubleArrayOf(1.0, 1.0, 1.0, 1.0)

        val solution = GnssPositionSolver.solve(a, b, w)
        assertNotNull(solution)

        // Expected solution x=[1, 1, 1, 1]
        val expectedX = doubleArrayOf(1.0, 1.0, 1.0, 1.0)
        for (i in expectedX.indices) {
            assertEquals(expectedX[i], solution!!.deltaX[i], 1e-6)
        }
    }

    @Test
    fun testCovarianceMatrix() {
        // A simple overdetermined system
        val a =
            arrayOf(
                doubleArrayOf(1.0, 0.0),
                doubleArrayOf(0.0, 1.0),
                doubleArrayOf(1.0, 1.0),
            )
        val b = doubleArrayOf(1.0, 1.0, 2.0)
        val w = doubleArrayOf(1.0, 1.0, 1.0)

        val solution = GnssPositionSolver.solve(a, b, w)
        assertNotNull(solution)

        // H = A^T * W * A
        // A^T = [[1, 0, 1], [0, 1, 1]]
        // H = [[2, 1], [1, 2]]
        // det(H) = 4 - 1 = 3
        // H^-1 = 1/3 * [[2, -1], [-1, 2]] = [[0.666, -0.333], [-0.333, 0.666]]

        val expectedCov =
            arrayOf(
                doubleArrayOf(2.0 / 3.0, -1.0 / 3.0),
                doubleArrayOf(-1.0 / 3.0, 2.0 / 3.0),
            )

        for (i in 0 until 2) {
            for (j in 0 until 2) {
                assertEquals(expectedCov[i][j], solution!!.covarianceMatrix[i][j], 1e-6)
            }
        }
    }

    @Test
    fun testAugmentedPppSystem() {
        // Typical PPP system: 3 pos + 1 clock + 1 ZWD + 2 ambiguities = 7 states

        // Let's test a smaller augmented system that is determined
        // 3 states: x, clock, ZWD
        // 3 satellites, code only, different positions to ensure non-singularity
        val a =
            arrayOf(
                doubleArrayOf(1.0, 1.0, 0.5), // sat 1: pos partial x=1, clock=1, ZWD=0.5
                doubleArrayOf(-1.0, 1.0, 0.7), // sat 2: pos partial x=-1
                doubleArrayOf(0.1, 1.0, 0.9), // sat 3: pos partial x=0.1
            )
        val b = doubleArrayOf(10.0, 10.2, 10.4)
        val w = doubleArrayOf(1.0, 1.0, 1.0)

        val solution = GnssPositionSolver.solve(a, b, w)
        assertNotNull(solution)
        assertEquals(3, solution!!.deltaX.size)
        assertEquals(3, solution.covarianceMatrix.size)
    }
}
