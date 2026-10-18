package app.simt.gnss.core.solver

import kotlin.math.floor
import kotlin.math.sqrt

/**
 * LAMBDA (Least-squares AMBiguity Decorrelation Adjustment) integer ambiguity resolver.
 *
 * Given float double-differenced carrier-phase ambiguities and their covariance, this finds the
 * integer vector that best fits, plus the second-best, and reports the **ratio test** statistic
 * (second-best squared residual / best squared residual). The caller must only accept the fix when
 * [Result.accepted] is true — i.e. the best integer candidate is decisively better than the runner-up.
 *
 * This is meaningful **only** on double-differenced ambiguities, where the satellite and receiver
 * phase biases cancel and the ambiguity is a true integer. It must not be applied to undifferenced /
 * float-PPP ambiguities.
 *
 * Implementation follows de Jonge & Tiberius (1996) and the RTKLIB `lambda()` reference: LᵀDL
 * factorization, integer (Z) decorrelation, and the modified-LAMBDA shrinking integer search.
 * Matrices are stored column-major (index `i + j*n`) to mirror the reference and limit translation
 * error.
 */
object LambdaAmbiguityResolver {
    private const val LOOP_MAX = 10_000

    /** Default ratio-test threshold. Conservative (3.0) to strongly favour float over a risky fix. */
    const val DEFAULT_RATIO_THRESHOLD = 3.0

    data class Result(
        /** Best integer candidate, in the original ambiguity ordering (cycles). */
        val fixedAmbiguities: DoubleArray,
        /** Ratio test value = (second-best residual norm) / (best residual norm); larger is safer. */
        val ratio: Double,
        /** True when [ratio] ≥ the threshold, i.e. the fix is safe to use. */
        val accepted: Boolean,
    )

    /**
     * @param floatAmbiguities the float DD ambiguity estimates (length n).
     * @param covariance the n×n ambiguity covariance (symmetric positive-definite).
     */
    fun resolve(
        floatAmbiguities: DoubleArray,
        covariance: Array<DoubleArray>,
        ratioThreshold: Double = DEFAULT_RATIO_THRESHOLD,
    ): Result? {
        val n = floatAmbiguities.size
        if (n < 1 || covariance.size != n || covariance.any { it.size != n }) return null

        val q = DoubleArray(n * n) { covariance[it % n][it / n] } // q[i + j*n] = covariance[i][j]
        val l = DoubleArray(n * n)
        val d = DoubleArray(n)
        if (!ltDlFactorize(n, q, l, d)) return null

        val z = identityColumnMajor(n)
        reduction(n, l, d, z)

        // Decorrelated float ambiguities: zFloat = Zᵀ a
        val zFloat = DoubleArray(n)
        for (i in 0 until n) {
            var sum = 0.0
            for (k in 0 until n) sum += z[k + i * n] * floatAmbiguities[k]
            zFloat[i] = sum
        }

        val candidates = DoubleArray(2 * n) // two best integer vectors (decorrelated space)
        val residualNorms = DoubleArray(2)
        if (!search(n, l, d, zFloat, candidates, residualNorms)) return null

        // Back-transform the best candidate to the original space: solve Zᵀ a = candidate.
        val bestDecorrelated = DoubleArray(n) { candidates[it + 0 * n] }
        val fixed = solveTransposed(n, z, bestDecorrelated) ?: return null
        for (i in 0 until n) fixed[i] = round(fixed[i])

        // residualNorms[0] ≈ 0 means the float ambiguities sit essentially exactly on the best
        // integer vector — the strongest possible fix, not a degenerate one to reject.
        val ratio =
            when {
                residualNorms[0] > 1e-12 -> residualNorms[1] / residualNorms[0]
                residualNorms[1] > 1e-12 -> Double.MAX_VALUE
                else -> 0.0 // both candidates fit perfectly: ambiguous, reject
            }
        return Result(fixed, ratio, ratio >= ratioThreshold)
    }

    /** LᵀDL factorization: Q = Lᵀ diag(D) L, with L unit lower-triangular. */
    private fun ltDlFactorize(
        n: Int,
        q: DoubleArray,
        l: DoubleArray,
        d: DoubleArray,
    ): Boolean {
        val a = q.copyOf()
        for (i in n - 1 downTo 0) {
            d[i] = a[i + i * n]
            if (d[i] <= 0.0) return false
            val s = sqrt(d[i])
            for (j in 0..i) l[i + j * n] = a[i + j * n] / s
            for (j in 0..i - 1) for (k in 0..j) a[j + k * n] -= l[i + k * n] * l[i + j * n]
            for (j in 0..i) l[i + j * n] /= l[i + i * n]
        }
        return true
    }

    private fun gauss(
        n: Int,
        l: DoubleArray,
        z: DoubleArray,
        i: Int,
        j: Int,
    ) {
        val mu = round(l[i + j * n]).toInt()
        if (mu != 0) {
            for (k in i until n) l[k + j * n] -= mu * l[k + i * n]
            for (k in 0 until n) z[k + j * n] -= mu * z[k + i * n]
        }
    }

    private fun permute(
        n: Int,
        l: DoubleArray,
        d: DoubleArray,
        j: Int,
        del: Double,
        z: DoubleArray,
    ) {
        val eta = d[j] / del
        val lam = d[j + 1] * l[j + 1 + j * n] / del
        d[j] = eta * d[j + 1]
        d[j + 1] = del
        for (k in 0..j - 1) {
            val a0 = l[j + k * n]
            val a1 = l[j + 1 + k * n]
            l[j + k * n] = -l[j + 1 + j * n] * a0 + a1
            l[j + 1 + k * n] = eta * a0 + lam * a1
        }
        l[j + 1 + j * n] = lam
        for (k in j + 2 until n) swap(l, k + j * n, k + (j + 1) * n)
        for (k in 0 until n) swap(z, k + j * n, k + (j + 1) * n)
    }

    /** Integer (Z) decorrelation: reduces correlation so the integer search is fast and reliable. */
    private fun reduction(
        n: Int,
        l: DoubleArray,
        d: DoubleArray,
        z: DoubleArray,
    ) {
        var j = n - 2
        var k = n - 2
        while (j >= 0) {
            if (j <= k) for (i in j + 1 until n) gauss(n, l, z, i, j)
            val del = d[j] + l[j + 1 + j * n] * l[j + 1 + j * n] * d[j + 1]
            if (del + 1e-6 < d[j + 1]) {
                permute(n, l, d, j, del, z)
                k = j
                j = n - 2
            } else {
                j--
            }
        }
    }

    /** Modified-LAMBDA shrinking integer search for the two best integer vectors. */
    private fun search(
        n: Int,
        l: DoubleArray,
        d: DoubleArray,
        zFloat: DoubleArray,
        candidates: DoubleArray,
        residualNorms: DoubleArray,
    ): Boolean {
        val m = 2
        val s = DoubleArray(n * n)
        val dist = DoubleArray(n)
        val zb = DoubleArray(n)
        val z = DoubleArray(n)
        val step = DoubleArray(n)

        var k = n - 1
        var nn = 0
        var imax = 0
        var maxDist = 1e99

        dist[k] = 0.0
        zb[k] = zFloat[k]
        z[k] = round(zb[k])
        var y = zb[k] - z[k]
        step[k] = sign(y)

        var searchComplete = false
        for (c in 0 until LOOP_MAX) {
            val newDist = dist[k] + y * y / d[k]
            if (newDist < maxDist) {
                if (k != 0) {
                    k--
                    dist[k] = newDist
                    for (i in 0..k) s[k + i * n] = s[k + 1 + i * n] + (z[k + 1] - zb[k + 1]) * l[k + 1 + i * n]
                    zb[k] = zFloat[k] + s[k + k * n]
                    z[k] = round(zb[k])
                    y = zb[k] - z[k]
                    step[k] = sign(y)
                } else {
                    if (nn < m) {
                        if (nn == 0 || newDist > residualNorms[imax]) imax = nn
                        for (i in 0 until n) candidates[i + nn * n] = z[i]
                        residualNorms[nn++] = newDist
                    } else {
                        if (newDist < residualNorms[imax]) {
                            for (i in 0 until n) candidates[i + imax * n] = z[i]
                            residualNorms[imax] = newDist
                            imax = 0
                            for (i in 0 until m) if (residualNorms[imax] < residualNorms[i]) imax = i
                        }
                        maxDist = residualNorms[imax]
                    }
                    z[0] += step[0]
                    y = zb[0] - z[0]
                    step[0] = -step[0] - sign(step[0])
                }
            } else {
                if (k == n - 1) {
                    searchComplete = true
                    break
                }
                k++
                z[k] += step[k]
                y = zb[k] - z[k]
                step[k] = -step[k] - sign(step[k])
            }
        }
        // An exhausted (LOOP_MAX) search has NOT proven the recorded candidates are the global
        // best and second-best — a ratio computed from them is meaningless and can validate a
        // wrong fix on ill-conditioned covariances. Mirror RTKLIB: treat exhaustion as failure.
        if (!searchComplete || nn < m) return false

        // Order the two candidates so the best (smallest residual) is column 0.
        if (residualNorms[0] > residualNorms[1]) {
            val tmp = residualNorms[0]
            residualNorms[0] = residualNorms[1]
            residualNorms[1] = tmp
            for (i in 0 until n) {
                val t = candidates[i + 0 * n]
                candidates[i + 0 * n] = candidates[i + 1 * n]
                candidates[i + 1 * n] = t
            }
        }
        return true
    }

    /** Solves Zᵀ x = b (Z column-major n×n) via Gaussian elimination with partial pivoting. */
    private fun solveTransposed(
        n: Int,
        z: DoubleArray,
        b: DoubleArray,
    ): DoubleArray? {
        val a = Array(n) { i -> DoubleArray(n) { j -> z[j + i * n] } } // a = Zᵀ
        val x = b.copyOf()
        for (col in 0 until n) {
            var pivot = col
            for (r in col + 1 until n) if (kotlin.math.abs(a[r][col]) > kotlin.math.abs(a[pivot][col])) pivot = r
            if (kotlin.math.abs(a[pivot][col]) < 1e-12) return null
            if (pivot != col) {
                val tr = a[pivot]
                a[pivot] = a[col]
                a[col] = tr
                val tx = x[pivot]
                x[pivot] = x[col]
                x[col] = tx
            }
            for (r in 0 until n) {
                if (r == col) continue
                val f = a[r][col] / a[col][col]
                for (cc in col until n) a[r][cc] -= f * a[col][cc]
                x[r] -= f * x[col]
            }
        }
        for (i in 0 until n) x[i] /= a[i][i]
        return x
    }

    private fun identityColumnMajor(n: Int): DoubleArray = DoubleArray(n * n) { if (it % n == it / n) 1.0 else 0.0 }

    private fun round(x: Double): Double = floor(x + 0.5)

    private fun sign(x: Double): Double = if (x <= 0.0) -1.0 else 1.0

    private fun swap(
        v: DoubleArray,
        i: Int,
        j: Int,
    ) {
        val t = v[i]
        v[i] = v[j]
        v[j] = t
    }
}
