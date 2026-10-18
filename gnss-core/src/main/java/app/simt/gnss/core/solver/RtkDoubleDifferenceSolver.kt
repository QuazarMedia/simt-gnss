package app.simt.gnss.core.solver
import app.simt.gnss.core.utils.GnssMathUtils.SPEED_OF_LIGHT
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.roundToInt
import kotlin.math.sqrt

/**
 * Double-differenced RTK baseline solver.
 *
 * Given the known base-station ECEF position and, per satellite, the rover and base range-like
 * observations (code pseudorange, or de-ambiguated carrier phase) plus the satellite ECEF position,
 * this solves for the rover position relative to the base. Differencing across receivers and then
 * across satellites cancels satellite clock/orbit, receiver clocks, and (to first order over a short
 * baseline) the ionosphere and troposphere — so the only unknowns are the three rover coordinates.
 * A base epoch that lags the rover does not cancel the satellite clock: remove it from each
 * observation with [withoutSatelliteClock] before differencing.
 *
 * **Partitioning:** double differences are formed only **within a (constellation, frequency band)
 * partition**, each with its own highest-elevation reference satellite. The between-receiver single
 * difference cancels each receiver's clock, but the rover−base differential *inter-system* and
 * *inter-frequency* biases (hardware delays, metres of code ISB between a phone and a geodetic
 * base) do not cancel across systems or bands — and a carrier DD across different wavelengths has
 * no integer ambiguity at all. All partitions share the three position states in one system.
 *
 * This is the production counterpart of `GnssRtkDoubleDifferenceBenchmarkTest`: same observation
 * model, plus iterated weighted least squares (via [GnssPositionSolver]), a post-fit residual outlier
 * gate, and a position covariance for accuracy reporting.
 */
object RtkDoubleDifferenceSolver {
    /**
     * Removes the broadcast satellite clock from a range observation.
     *
     * The pseudorange is geometric range plus receiver clock minus satellite clock. Double
     * differences cancel the satellite clock only when both receivers observed it at the same
     * time. Across a base lag the remainder is `c·af1·Δt` on that satellite.
     */
    fun withoutSatelliteClock(
        observationMeters: Double,
        satelliteClockSeconds: Double,
    ): Double = observationMeters + satelliteClockSeconds * SPEED_OF_LIGHT

    private const val MAX_ITERATIONS = 10
    private const val CONVERGENCE_METERS = 1e-4

    /** Minimum satellites to solve 3 unknowns from double differences (reference + 3 differences). */
    const val MIN_SATELLITES = 4

    /** Minimum double-difference rows to solve the 3 position unknowns. */
    private const val MIN_DD_ROWS = 3

    /** Post-fit |DD residual| above which a satellite is treated as an outlier (code-level gate). */
    private const val MAX_DD_RESIDUAL_METERS = 4.0
    private const val MAX_EXCLUSIONS = 3

    data class SatelliteInput(
        /** Satellite ECEF at the rover's transmit time (used for the rover line-of-sight and range). */
        val satelliteEcef: DoubleArray,
        /** Satellite ECEF at the base's transmit time (used only for the base range, so a base/rover
         *  epoch offset does not corrupt the double difference). */
        val baseSatelliteEcef: DoubleArray,
        val roverObservationMeters: Double,
        val baseObservationMeters: Double,
        val elevationDeg: Double,
        /** A-priori one-sigma of the (undifferenced) observation, used for relative weighting. */
        val sigmaMeters: Double,
        /** Android `Constellation.*`; DDs are only formed within one constellation. */
        val constellationType: Int = 0,
        /** Carrier frequency rounded to MHz; DDs are only formed within one band. */
        val bandMhz: Int = 0,
    )

    data class Solution(
        val roverEcef: DoubleArray,
        val residualRms: Double,
        /** 3×3 ECEF position covariance (unit-weight), scale by σ₀² for a metric covariance. */
        val covariance: Array<DoubleArray>,
        val usedSatelliteCount: Int,
    )

    /** Per-row pairing of a satellite index with its partition's reference-satellite index. */
    private data class DdRow(val satIndex: Int, val refIndex: Int)

    /** Highest-elevation reference per partition, then one row per non-reference satellite. */
    private fun <T> buildRows(
        items: List<T>,
        partitionKeyOf: (T) -> Any,
        elevationOf: (T) -> Double,
    ): List<DdRow> {
        val byPartition = items.indices.groupBy { partitionKeyOf(items[it]) }
        val rows = ArrayList<DdRow>(items.size)
        for ((_, indices) in byPartition) {
            if (indices.size < 2) continue // a lone satellite cannot form a double difference
            val ref = indices.maxByOrNull { elevationOf(items[it]) }!!
            for (i in indices) {
                if (i != ref) rows.add(DdRow(i, ref))
            }
        }
        return rows
    }

    private fun codePartitionKey(input: SatelliteInput): Any = input.constellationType to input.bandMhz

    private fun carrierPartitionKey(input: CarrierSatelliteInput): Any = input.constellationType to (input.wavelengthMeters * 1e4).roundToInt()

    /**
     * @param initialRover starting estimate (the base position is a fine seed for a short baseline).
     */
    fun solve(
        baseEcef: DoubleArray,
        inputs: List<SatelliteInput>,
        initialRover: DoubleArray,
    ): Solution? {
        var active = inputs.toMutableList()
        var exclusions = 0

        while (active.size >= MIN_SATELLITES) {
            val rows = buildRows(active, ::codePartitionKey) { it.elevationDeg }
            if (rows.size < MIN_DD_ROWS) return null
            val baseRange = DoubleArray(active.size) { distance(active[it].baseSatelliteEcef, baseEcef) }
            val singleDiff = DoubleArray(active.size) { active[it].roverObservationMeters - active[it].baseObservationMeters }

            var x = initialRover.copyOf()
            var converged = false
            for (iter in 0 until MAX_ITERATIONS) {
                val system = buildSystem(x, active, rows, baseRange, singleDiff)
                val solution = GnssPositionSolver.solve(system.a, system.b, system.w, needCovariance = false) ?: return null
                x = doubleArrayOf(x[0] + solution.deltaX[0], x[1] + solution.deltaX[1], x[2] + solution.deltaX[2])
                if (norm(solution.deltaX) < CONVERGENCE_METERS) {
                    converged = true
                    break
                }
            }
            if (!converged) return null

            // Final solve at the converged geometry, with covariance, then post-fit residual gating.
            val system = buildSystem(x, active, rows, baseRange, singleDiff)
            val solution = GnssPositionSolver.solve(system.a, system.b, system.w, needCovariance = true) ?: return null

            var sumSq = 0.0
            var maxResidual = 0.0
            var maxResidualRow = -1
            // Residual gating and the reported RMS use the METRIC (pre-whitening) rows:
            // the 4 m outlier threshold is in meters, the whitened rows are unitless.
            for (row in rows.indices) {
                var predicted = 0.0
                for (j in 0 until 3) predicted += system.metricA[row][j] * solution.deltaX[j]
                val residual = kotlin.math.abs(system.metricB[row] - predicted)
                sumSq += residual * residual
                if (residual > maxResidual) {
                    maxResidual = residual
                    maxResidualRow = row
                }
            }
            // Divide by the degrees of freedom (rows − 3 position states), not n: E[Σr²]
            // = σ²(n − m), and /n under-reports the RMS by 2x at the minimum row count —
            // loosening the float-sanity gate exactly where it matters most.
            val residualRms = if (rows.isNotEmpty()) sqrt(sumSq / (rows.size - 3).coerceAtLeast(1)) else 0.0

            val canExclude = active.size > MIN_SATELLITES && exclusions < MAX_EXCLUSIONS
            if (canExclude && maxResidual > MAX_DD_RESIDUAL_METERS && maxResidualRow >= 0) {
                active.removeAt(rows[maxResidualRow].satIndex)
                exclusions++
                continue
            }

            return Solution(
                roverEcef = x,
                residualRms = residualRms,
                covariance = solution.covarianceMatrix,
                usedSatelliteCount = active.size,
            )
        }
        return null
    }

    /** Minimum carrier-tracked satellites required to even attempt integer fixing. */
    const val MIN_FIX_SATELLITES = 5

    /**
     * A fixed solution is rejected if it lands further than this from the (already validated) float
     * solution — a last guard against a ratio-passing-but-wrong integer fix reaching the caller.
     */
    private const val MAX_FIX_FLOAT_SEPARATION_METERS = 1.0

    /** Fewest ambiguities a partial fix may keep. One reference plus these differences is [MIN_FIX_SATELLITES]. */
    private const val MIN_PARTIAL_AMBIGUITIES = MIN_FIX_SATELLITES - 1

    /** A carried ambiguity further than this from the new float is a slip the Doppler test missed. */
    private const val AMBIGUITY_DISAGREEMENT_CYCLES = 0.5
    private const val AMBIGUITY_DISAGREEMENT_SIGMA = 4.0

    data class CarrierSatelliteInput(
        val satelliteEcef: DoubleArray,
        val baseSatelliteEcef: DoubleArray,
        val roverCodeMeters: Double,
        val baseCodeMeters: Double,
        val roverCarrierMeters: Double,
        val baseCarrierMeters: Double,
        val wavelengthMeters: Double,
        val elevationDeg: Double,
        val codeSigmaMeters: Double,
        val carrierSigmaMeters: Double,
        /** Android `Constellation.*`; carrier DDs across systems carry non-integer biases. */
        val constellationType: Int = 0,
        /** Satellite id. Negative when the caller has none; priors cannot match those rows. */
        val svid: Int = -1,
    )

    /**
     * One double-difference ambiguity carried from the previous epoch.
     * [varianceCycles2] is the previous posterior, not a tightened substitute for it.
     */
    data class AmbiguityState(
        val constellationType: Int,
        val satSvid: Int,
        val refSvid: Int,
        val wavelengthMeters: Double,
        val cycles: Double,
        val varianceCycles2: Double,
    )

    data class FixedSolution(
        val roverEcef: DoubleArray,
        val residualRms: Double,
        val covariance: Array<DoubleArray>,
        val usedSatelliteCount: Int,
        val ratio: Double,
    )

    /**
     * Attempts a carrier-phase integer-fixed solution. Forms a float solution (position + DD integer
     * ambiguities) from code and carrier double differences, resolves the ambiguities with
     * [LambdaAmbiguityResolver], and — only if the ratio test passes and the fixed position agrees
     * with [floatRoverEcef] — re-solves the position with the ambiguities held at their integers,
     * yielding a centimetre-class fix.
     *
     * Double differences are formed per (constellation, wavelength) partition; the DD ambiguity is
     * an integer only when both satellites share the same wavelength and system.
     *
     * Returns null whenever fixing is not safe (too few carrier satellites, the ratio test
     * fails, or the fixed position disagrees with the float). The caller must then keep its float
     * solution: this method never degrades the float result, it can only upgrade it.
     *
     * [priors] are previous-epoch double-difference ambiguities. One that disagrees with this
     * epoch's unconstrained float is dropped. The rest enter the float system as their own
     * variance, and [onFloat] receives the posterior whether or not a fix is published.
     * Integer search retries without the loosest ambiguity, re-solving that subset, until the
     * ratio test passes or fewer than [MIN_PARTIAL_AMBIGUITIES] remain.
     */
    fun solveFixed(
        baseEcef: DoubleArray,
        inputs: List<CarrierSatelliteInput>,
        initialRover: DoubleArray,
        floatRoverEcef: DoubleArray,
        ratioThreshold: Double = LambdaAmbiguityResolver.DEFAULT_RATIO_THRESHOLD,
        priors: List<AmbiguityState> = emptyList(),
        onFloat: (List<AmbiguityState>) -> Unit = {},
    ): FixedSolution? {
        if (inputs.size < MIN_FIX_SATELLITES) return null
        val rows = buildRows(inputs, ::carrierPartitionKey) { it.elevationDeg }
        val ambiguityCount = rows.size
        if (ambiguityCount < MIN_PARTIAL_AMBIGUITIES) return null

        val baseRange = DoubleArray(inputs.size) { distance(inputs[it].baseSatelliteEcef, baseEcef) }
        val sdCode = DoubleArray(inputs.size) { inputs[it].roverCodeMeters - inputs[it].baseCodeMeters }
        val sdCarrier = DoubleArray(inputs.size) { inputs[it].roverCarrierMeters - inputs[it].baseCarrierMeters }

        val unconstrained =
            iterateFloat(initialRover, inputs, rows, baseRange, sdCode, sdCarrier, emptyList()) ?: return null
        val kept =
            priors.filter { prior ->
                val index = ambiguityIndex(inputs, rows, prior)
                if (index < 0) return@filter false
                val limit =
                    max(
                        AMBIGUITY_DISAGREEMENT_CYCLES,
                        AMBIGUITY_DISAGREEMENT_SIGMA * sqrt(prior.varianceCycles2.coerceAtLeast(0.0)),
                    )
                abs(unconstrained.ambiguities[index] - prior.cycles) <= limit
            }
        val estimate =
            if (kept.isEmpty()) {
                unconstrained
            } else {
                iterateFloat(unconstrained.position, inputs, rows, baseRange, sdCode, sdCarrier, kept) ?: unconstrained
            }
        onFloat(floatStates(inputs, rows, estimate.ambiguities, estimate.covariance))

        // A weak ambiguity is left float and the rest are re-solved without it. Subsetting the
        // joint covariance leaves the correlation with the weak satellite in the rest, so the
        // ratio test keeps failing on satellites the carrier has already measured well.
        var activeRows = rows
        var activeEstimate = estimate
        var acceptedRows: List<DdRow>? = null
        var acceptedCycles: DoubleArray? = null
        var acceptedRatio = 0.0
        while (activeRows.size >= MIN_PARTIAL_AMBIGUITIES) {
            val lambda =
                LambdaAmbiguityResolver.resolve(activeEstimate.ambiguities, activeEstimate.covariance, ratioThreshold)
            if (lambda != null && lambda.accepted) {
                acceptedRows = activeRows
                acceptedCycles = lambda.fixedAmbiguities
                acceptedRatio = lambda.ratio
                break
            }
            if (activeRows.size == MIN_PARTIAL_AMBIGUITIES) break
            val worst = activeEstimate.covariance.indices.maxByOrNull { activeEstimate.covariance[it][it] } ?: break
            activeRows = activeRows.filterIndexed { index, _ -> index != worst }
            val reducedPriors = kept.filter { ambiguityIndex(inputs, activeRows, it) >= 0 }
            activeEstimate =
                iterateFloat(
                    activeEstimate.position,
                    inputs,
                    activeRows,
                    baseRange,
                    sdCode,
                    sdCarrier,
                    reducedPriors,
                ) ?: return null
        }
        val fixedRows = acceptedRows ?: return null
        val fixedAmbiguities = acceptedCycles ?: return null
        val ratio = acceptedRatio

        var xFixed = activeEstimate.position.copyOf()
        var fixedConverged = false
        var finalSystem: LinearSystem? = null
        var finalSolution: GnssPositionSolver.WlsSolution? = null
        for (iter in 0 until MAX_ITERATIONS) {
            val system = buildFixedSystem(xFixed, inputs, fixedRows, baseRange, sdCarrier, fixedAmbiguities)
            val solution = GnssPositionSolver.solve(system.a, system.b, system.w, needCovariance = fixedConverged) ?: return null
            xFixed = doubleArrayOf(xFixed[0] + solution.deltaX[0], xFixed[1] + solution.deltaX[1], xFixed[2] + solution.deltaX[2])
            if (fixedConverged) {
                finalSystem = system
                finalSolution = solution
                break
            }
            if (norm(doubleArrayOf(solution.deltaX[0], solution.deltaX[1], solution.deltaX[2])) < CONVERGENCE_METERS) {
                fixedConverged = true
            }
        }
        val system = finalSystem ?: return null
        val solution = finalSolution ?: return null

        // Last guard: a correct fix must agree with the validated code-float position.
        if (distance(xFixed, floatRoverEcef) > MAX_FIX_FLOAT_SEPARATION_METERS) return null

        var sumSq = 0.0
        for (row in fixedRows.indices) {
            var predicted = 0.0
            for (j in 0 until 3) predicted += system.metricA[row][j] * solution.deltaX[j]
            val residual = system.metricB[row] - predicted
            sumSq += residual * residual
        }
        val residualRms = if (fixedRows.isNotEmpty()) sqrt(sumSq / (fixedRows.size - 3).coerceAtLeast(1)) else 0.0
        val used = (fixedRows.map { it.satIndex } + fixedRows.map { it.refIndex }).toSet().size

        return FixedSolution(
            roverEcef = xFixed,
            residualRms = residualRms,
            covariance = solution.covarianceMatrix,
            usedSatelliteCount = used,
            ratio = ratio,
        )
    }

    private data class FloatEstimate(
        val position: DoubleArray,
        val ambiguities: DoubleArray,
        val covariance: Array<DoubleArray>,
    )

    private fun iterateFloat(
        initialRover: DoubleArray,
        inputs: List<CarrierSatelliteInput>,
        rows: List<DdRow>,
        baseRange: DoubleArray,
        sdCode: DoubleArray,
        sdCarrier: DoubleArray,
        priors: List<AmbiguityState>,
    ): FloatEstimate? {
        var x = initialRover.copyOf()
        var ambiguityFloat: DoubleArray? = null
        var ambiguityCovariance: Array<DoubleArray>? = null
        var converged = false
        for (iter in 0 until MAX_ITERATIONS) {
            val system = buildFloatSystem(x, inputs, rows, baseRange, sdCode, sdCarrier, priors)
            val solution = GnssPositionSolver.solve(system.a, system.b, system.w, needCovariance = converged) ?: return null
            x = doubleArrayOf(x[0] + solution.deltaX[0], x[1] + solution.deltaX[1], x[2] + solution.deltaX[2])
            if (converged) {
                ambiguityFloat = DoubleArray(rows.size) { solution.deltaX[3 + it] }
                ambiguityCovariance =
                    Array(rows.size) { i ->
                        DoubleArray(rows.size) { j -> solution.covarianceMatrix[3 + i][3 + j] }
                    }
                break
            }
            if (norm(doubleArrayOf(solution.deltaX[0], solution.deltaX[1], solution.deltaX[2])) < CONVERGENCE_METERS) {
                converged = true
            }
        }
        val ambiguities = ambiguityFloat ?: return null
        val covariance = ambiguityCovariance ?: return null
        return FloatEstimate(x, ambiguities, covariance)
    }

    private fun ambiguityIndex(
        inputs: List<CarrierSatelliteInput>,
        rows: List<DdRow>,
        prior: AmbiguityState,
    ): Int {
        for ((index, row) in rows.withIndex()) {
            val sat = inputs[row.satIndex]
            val ref = inputs[row.refIndex]
            if (sat.svid == prior.satSvid &&
                ref.svid == prior.refSvid &&
                sat.constellationType == prior.constellationType &&
                abs(sat.wavelengthMeters - prior.wavelengthMeters) < 1e-4
            ) {
                return index
            }
        }
        return -1
    }

    private fun floatStates(
        inputs: List<CarrierSatelliteInput>,
        rows: List<DdRow>,
        ambiguities: DoubleArray,
        covariance: Array<DoubleArray>,
    ): List<AmbiguityState> {
        val states = ArrayList<AmbiguityState>(rows.size)
        for ((index, row) in rows.withIndex()) {
            val sat = inputs[row.satIndex]
            val ref = inputs[row.refIndex]
            val variance = covariance[index][index]
            if (sat.svid < 0 || ref.svid < 0 || !ambiguities[index].isFinite() || !variance.isFinite() || variance <= 0.0) {
                continue
            }
            states +=
                AmbiguityState(
                    constellationType = sat.constellationType,
                    satSvid = sat.svid,
                    refSvid = ref.svid,
                    wavelengthMeters = sat.wavelengthMeters,
                    cycles = ambiguities[index],
                    varianceCycles2 = variance,
                )
        }
        return states
    }

    private fun buildFloatSystem(
        rover: DoubleArray,
        inputs: List<CarrierSatelliteInput>,
        rows: List<DdRow>,
        baseRange: DoubleArray,
        sdCode: DoubleArray,
        sdCarrier: DoubleArray,
        priors: List<AmbiguityState> = emptyList(),
    ): LinearSystem {
        val k = rows.size
        val m = 3 + k
        val a = Array(2 * k) { DoubleArray(m) }
        val b = DoubleArray(2 * k)
        val w = DoubleArray(2 * k)

        for ((idx, row) in rows.withIndex()) {
            val sat = row.satIndex
            val ref = row.refIndex
            val eRef = lineOfSight(rover, inputs[ref].satelliteEcef)
            val rangeRef = distance(rover, inputs[ref].satelliteEcef)
            val eK = lineOfSight(rover, inputs[sat].satelliteEcef)
            val ddGeom = (distance(rover, inputs[sat].satelliteEcef) - baseRange[sat]) - (rangeRef - baseRange[ref])
            val p0 = eRef[0] - eK[0]
            val p1 = eRef[1] - eK[1]
            val p2 = eRef[2] - eK[2]

            // Code DD row — position only.
            a[idx][0] = p0
            a[idx][1] = p1
            a[idx][2] = p2
            b[idx] = (sdCode[sat] - sdCode[ref]) - ddGeom

            // Carrier DD row — position plus this satellite's integer ambiguity (coefficient λ).
            val carrierRow = k + idx
            a[carrierRow][0] = p0
            a[carrierRow][1] = p1
            a[carrierRow][2] = p2
            a[carrierRow][3 + idx] = inputs[sat].wavelengthMeters
            b[carrierRow] = (sdCarrier[sat] - sdCarrier[ref]) - ddGeom
        }

        val metricA = Array(2 * k) { a[it].copyOf() }
        val metricB = b.copyOf()
        // Whiten the code block (rows [0..k)) and carrier block (rows [k..2k)) separately,
        // per shared-reference group: code and carrier noises are independent of each
        // other, but within each block every DD against the same reference is correlated.
        // The ambiguity covariance handed to LAMBDA then carries the true reference-induced
        // structure instead of a falsely diagonal one.
        for ((_, groupRows) in rows.indices.groupBy { rows[it].refIndex }) {
            val indices = groupRows.toIntArray()
            val referenceIndex = rows[indices[0]].refIndex
            whitenSharedReferenceGroup(
                a,
                b,
                w,
                indices,
                memberSigmas = DoubleArray(indices.size) { inputs[rows[indices[it]].satIndex].codeSigmaMeters },
                referenceSigma = inputs[referenceIndex].codeSigmaMeters,
            )
            val carrierIndices = IntArray(indices.size) { k + indices[it] }
            whitenSharedReferenceGroup(
                a,
                b,
                w,
                carrierIndices,
                memberSigmas = DoubleArray(indices.size) { inputs[rows[indices[it]].satIndex].carrierSigmaMeters },
                referenceSigma = inputs[referenceIndex].carrierSigmaMeters,
            )
        }
        val matched = ArrayList<Pair<Int, AmbiguityState>>()
        for (prior in priors) {
            val index = ambiguityIndex(inputs, rows, prior)
            if (index >= 0 && prior.varianceCycles2.isFinite() && prior.varianceCycles2 > 0.0 && prior.cycles.isFinite()) {
                matched += index to prior
            }
        }
        if (matched.isEmpty()) return LinearSystem(a, b, w, metricA, metricB)

        // Priors are independent of the shared-reference correlation, so they are appended
        // after whitening rather than mixed into a double-difference group.
        val n = 2 * k + matched.size
        val extendedA = Array(n) { i -> if (i < 2 * k) a[i] else DoubleArray(m) }
        val extendedB = DoubleArray(n) { i -> if (i < 2 * k) b[i] else 0.0 }
        val extendedW = DoubleArray(n) { i -> if (i < 2 * k) w[i] else 0.0 }
        val extendedMetricA = Array(n) { i -> if (i < 2 * k) metricA[i] else DoubleArray(m) }
        val extendedMetricB = DoubleArray(n) { i -> if (i < 2 * k) metricB[i] else 0.0 }
        for ((extra, pair) in matched.withIndex()) {
            val (index, prior) = pair
            val row = 2 * k + extra
            val wavelength = inputs[rows[index].satIndex].wavelengthMeters
            val sigmaMeters = (wavelength * sqrt(prior.varianceCycles2)).coerceAtLeast(1e-6)
            extendedA[row][3 + index] = wavelength
            extendedB[row] = wavelength * prior.cycles
            extendedW[row] = 1.0 / (sigmaMeters * sigmaMeters)
            extendedMetricA[row][3 + index] = wavelength
            extendedMetricB[row] = wavelength * prior.cycles
        }
        return LinearSystem(extendedA, extendedB, extendedW, extendedMetricA, extendedMetricB)
    }

    private fun buildFixedSystem(
        rover: DoubleArray,
        inputs: List<CarrierSatelliteInput>,
        rows: List<DdRow>,
        baseRange: DoubleArray,
        sdCarrier: DoubleArray,
        fixedAmbiguities: DoubleArray,
    ): LinearSystem {
        val a = Array(rows.size) { DoubleArray(3) }
        val b = DoubleArray(rows.size)
        val w = DoubleArray(rows.size)

        for ((idx, row) in rows.withIndex()) {
            val sat = row.satIndex
            val ref = row.refIndex
            val eRef = lineOfSight(rover, inputs[ref].satelliteEcef)
            val rangeRef = distance(rover, inputs[ref].satelliteEcef)
            val eK = lineOfSight(rover, inputs[sat].satelliteEcef)
            val ddGeom = (distance(rover, inputs[sat].satelliteEcef) - baseRange[sat]) - (rangeRef - baseRange[ref])
            a[idx][0] = eRef[0] - eK[0]
            a[idx][1] = eRef[1] - eK[1]
            a[idx][2] = eRef[2] - eK[2]
            // Subtract the now-known integer ambiguity so only position remains unknown.
            b[idx] = (sdCarrier[sat] - sdCarrier[ref]) - ddGeom - inputs[sat].wavelengthMeters * fixedAmbiguities[idx]
        }

        val metricA = Array(rows.size) { a[it].copyOf() }
        val metricB = b.copyOf()
        for ((_, groupRows) in rows.indices.groupBy { rows[it].refIndex }) {
            val indices = groupRows.toIntArray()
            whitenSharedReferenceGroup(
                a,
                b,
                w,
                indices,
                memberSigmas = DoubleArray(indices.size) { inputs[rows[indices[it]].satIndex].carrierSigmaMeters },
                referenceSigma = inputs[rows[indices[0]].refIndex].carrierSigmaMeters,
            )
        }
        return LinearSystem(a, b, w, metricA, metricB)
    }

    /**
     * Linear system handed to the WLS solver. [a]/[b] are WHITENED per shared-reference
     * group (unit weights [w]); [metricA]/[metricB] keep the pre-whitening rows so residual
     * gating and the reported RMS stay in meters.
     */
    private class LinearSystem(
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
    private fun whitenSharedReferenceGroup(
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

    private fun choleskyLower(matrix: Array<DoubleArray>): Array<DoubleArray>? {
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

    private fun buildSystem(
        rover: DoubleArray,
        active: List<SatelliteInput>,
        rows: List<DdRow>,
        baseRange: DoubleArray,
        singleDiff: DoubleArray,
    ): LinearSystem {
        val a = Array(rows.size) { DoubleArray(3) }
        val b = DoubleArray(rows.size)
        val w = DoubleArray(rows.size)

        for ((rowIdx, row) in rows.withIndex()) {
            val k = row.satIndex
            val ref = row.refIndex
            val eRef = lineOfSight(rover, active[ref].satelliteEcef)
            val rangeRef = distance(rover, active[ref].satelliteEcef)
            val eK = lineOfSight(rover, active[k].satelliteEcef)
            val rangeK = distance(rover, active[k].satelliteEcef)
            val ddObserved = singleDiff[k] - singleDiff[ref]
            val ddModeled = (rangeK - baseRange[k]) - (rangeRef - baseRange[ref])

            // d(ddModeled)/d(rover) = (-e_k) - (-e_ref) = e_ref - e_k
            a[rowIdx][0] = eRef[0] - eK[0]
            a[rowIdx][1] = eRef[1] - eK[1]
            a[rowIdx][2] = eRef[2] - eK[2]
            b[rowIdx] = ddObserved - ddModeled
        }

        val metricA = Array(rows.size) { a[it].copyOf() }
        val metricB = b.copyOf()
        for ((_, groupRows) in rows.indices.groupBy { rows[it].refIndex }) {
            val indices = groupRows.toIntArray()
            whitenSharedReferenceGroup(
                a,
                b,
                w,
                indices,
                memberSigmas = DoubleArray(indices.size) { active[rows[indices[it]].satIndex].sigmaMeters },
                referenceSigma = active[rows[indices[0]].refIndex].sigmaMeters,
            )
        }
        return LinearSystem(a, b, w, metricA, metricB)
    }

    private fun distance(
        a: DoubleArray,
        b: DoubleArray,
    ): Double {
        val dx = a[0] - b[0]
        val dy = a[1] - b[1]
        val dz = a[2] - b[2]
        return sqrt(dx * dx + dy * dy + dz * dz)
    }

    private fun lineOfSight(
        from: DoubleArray,
        to: DoubleArray,
    ): DoubleArray {
        val dx = to[0] - from[0]
        val dy = to[1] - from[1]
        val dz = to[2] - from[2]
        val r = sqrt(dx * dx + dy * dy + dz * dz)
        return doubleArrayOf(dx / r, dy / r, dz / r)
    }

    private fun norm(v: DoubleArray): Double = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
}
