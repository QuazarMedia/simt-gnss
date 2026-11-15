package app.simt.gnss.core.solver

import kotlin.math.sqrt

/**
 * Linear algebra for the double-differenced RTK solver: the whitened WLS
 * system holder, the shared-reference whitening transform, a Cholesky
 * factorization, and small geometry helpers. Extracted verbatim from
 * [RtkDoubleDifferenceSolver] (members were its private helpers); pure
 * functions, no solver state.
 *
 * Linear system handed to the WLS solver. [a]/[b] are WHITENED per shared-reference
 * group (unit weights [w]); [metricA]/[metricB] keep the pre-whitening rows so residual
 * gating and the reported RMS stay in meters.
 */
internal class LinearSystem(
    val a: Array<DoubleArray>,
    val b: DoubleArray,
    val w: DoubleArray,
    val metricA: Array<DoubleArray>,
    val metricB: DoubleArray,
)

/**
 * Whitens the rows of one shared-reference group. Double differences against a common
 * reference are NOT independent: Cov(DD_i, DD_j) = σ_ref² for i ≠ j (a dominant rank-1
 * term, since the reference is deliberately the best satellite). Diagonal weights made
 * the estimate non-BLUE, reported an optimistic covariance of the wrong SHAPE to
 * LAMBDA (mis-calibrating the ratio test), and let reference-satellite multipath leak
 * coherently into every DD while the residual screen treated rows as independent.
 * Σ = LLᵀ is factored and the group's rows replaced with L⁻¹·rows at unit weight, so
 * the solver's (AᵀA)⁻¹ becomes the true (AᵀΣ⁻¹A)⁻¹.
 */
internal fun whitenSharedReferenceGroup(
    a: Array<DoubleArray>,
    b: DoubleArray,
    w: DoubleArray,
    rowIndices: IntArray,
    memberSigmas: DoubleArray,
    referenceSigma: Double,
) {
    val k = rowIndices.size
    val refVariance = referenceSigma * referenceSigma
    if (k == 1) {
        w[rowIndices[0]] = 1.0 / (memberSigmas[0] * memberSigmas[0] + refVariance)
        return
    }
    val sigma =
        Array(k) { i ->
            DoubleArray(k) { j ->
                if (i == j) memberSigmas[i] * memberSigmas[i] + refVariance else refVariance
            }
        }
    val l = choleskyLower(sigma)
    if (l == null) {
        // Σ is SPD for any positive sigmas; this only trips on non-finite inputs —
        // degrade to the diagonal approximation rather than aborting the epoch.
        for (i in 0 until k) w[rowIndices[i]] = 1.0 / sigma[i][i]
        return
    }
    val m = a[rowIndices[0]].size
    val whitenedA = Array(k) { DoubleArray(m) }
    val whitenedB = DoubleArray(k)
    for (i in 0 until k) {
        val source = a[rowIndices[i]]
        for (c in 0 until m) whitenedA[i][c] = source[c]
        whitenedB[i] = b[rowIndices[i]]
        for (j in 0 until i) {
            val lij = l[i][j]
            if (lij == 0.0) continue
            for (c in 0 until m) whitenedA[i][c] -= lij * whitenedA[j][c]
            whitenedB[i] -= lij * whitenedB[j]
        }
        val inverseDiagonal = 1.0 / l[i][i]
        for (c in 0 until m) whitenedA[i][c] *= inverseDiagonal
        whitenedB[i] *= inverseDiagonal
    }
    for (i in 0 until k) {
        a[rowIndices[i]] = whitenedA[i]
        b[rowIndices[i]] = whitenedB[i]
        w[rowIndices[i]] = 1.0
    }
}

internal fun choleskyLower(matrix: Array<DoubleArray>): Array<DoubleArray>? {
    val n = matrix.size
    val l = Array(n) { DoubleArray(n) }
    for (i in 0 until n) {
        for (j in 0..i) {
            var sum = 0.0
            for (p in 0 until j) sum += l[i][p] * l[j][p]
            if (i == j) {
                val d = matrix[i][i] - sum
                if (!d.isFinite() || d <= 0.0) return null
                l[i][j] = sqrt(d)
            } else {
                l[i][j] = (matrix[i][j] - sum) / l[j][j]
            }
        }
    }
    return l
}

internal fun distance(
    a: DoubleArray,
    b: DoubleArray,
): Double {
    val dx = a[0] - b[0]
    val dy = a[1] - b[1]
    val dz = a[2] - b[2]
    return sqrt(dx * dx + dy * dy + dz * dz)
}

internal fun lineOfSight(
    from: DoubleArray,
    to: DoubleArray,
): DoubleArray {
    val dx = to[0] - from[0]
    val dy = to[1] - from[1]
    val dz = to[2] - from[2]
    val r = sqrt(dx * dx + dy * dy + dz * dz)
    return doubleArrayOf(dx / r, dy / r, dz / r)
}

internal fun norm(v: DoubleArray): Double = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
