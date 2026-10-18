package app.simt.gnss.core.solver

import org.junit.Assert.assertArrayEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Test

class LambdaAmbiguityResolverTest {
    @Test
    fun resolve_roundsIndependentTightAmbiguities() {
        val float = doubleArrayOf(1.2, 2.8, -0.9)
        val q = diagonal(0.01, 0.01, 0.01)

        val result = LambdaAmbiguityResolver.resolve(float, q)
        assertNotNull(result)
        requireNotNull(result)
        assertArrayEquals(doubleArrayOf(1.0, 3.0, -1.0), result.fixedAmbiguities, 1e-9)
        assertTrue("tight ambiguities should pass the ratio test (was ${result.ratio})", result.accepted)
    }

    @Test
    fun resolve_recoversTrueIntegersFromCorrelatedCovariance() {
        // Float values are the true integers perturbed within a small, correlated covariance.
        val truth = doubleArrayOf(5.0, -3.0, 3.0)
        val float = doubleArrayOf(5.08, -2.94, 3.05)
        val q =
            arrayOf(
                doubleArrayOf(0.040, 0.020, 0.000),
                doubleArrayOf(0.020, 0.040, 0.010),
                doubleArrayOf(0.000, 0.010, 0.030),
            )

        val result = LambdaAmbiguityResolver.resolve(float, q)
        assertNotNull(result)
        requireNotNull(result)
        assertArrayEquals(truth, result.fixedAmbiguities, 1e-9)
        assertTrue("ratio ${result.ratio} should exceed threshold", result.accepted)
    }

    @Test
    fun resolve_rejectsAmbiguousLowConfidenceCase() {
        // Large variances and float values exactly between integers -> no decisive winner.
        val float = doubleArrayOf(0.5, 0.5)
        val q = diagonal(0.5, 0.5)

        val result = LambdaAmbiguityResolver.resolve(float, q)
        assertNotNull(result)
        requireNotNull(result)
        assertFalse("ambiguous case must be rejected (ratio ${result.ratio})", result.accepted)
        assertTrue("ratio should be near 1 for an ambiguous case", result.ratio < 3.0)
    }

    @Test
    fun resolve_handlesSingleAmbiguity() {
        val result = LambdaAmbiguityResolver.resolve(doubleArrayOf(2.3), arrayOf(doubleArrayOf(0.01)))
        assertNotNull(result)
        requireNotNull(result)
        assertArrayEquals(doubleArrayOf(2.0), result.fixedAmbiguities, 1e-9)
        assertTrue(result.accepted)
    }

    @Test
    fun resolve_returnsNullForNonPositiveDefiniteCovariance() {
        val result = LambdaAmbiguityResolver.resolve(doubleArrayOf(1.0, 2.0), diagonal(0.0, 0.01))
        org.junit.Assert.assertNull(result)
    }

    private fun diagonal(vararg values: Double): Array<DoubleArray> = Array(values.size) { i -> DoubleArray(values.size) { j -> if (i == j) values[i] else 0.0 } }
}
