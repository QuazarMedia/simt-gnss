package app.simt.gnss.core.solver
import app.simt.gnss.core.model.Constellation
import app.simt.gnss.core.utils.GnssMathUtils
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
        val degreesOfFreedom: Int = 0,
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

    private fun carrierPartitionKey(input: CarrierSatelliteInput): Any = input.constellationType to (input.wavelengthMeters * 1e5).roundToInt()

    private fun buildCarrierRows(
        items: List<CarrierSatelliteInput>,
        priors: List<AmbiguityState>,
    ): List<DdRow> {
        val byPartition = items.indices.groupBy { carrierPartitionKey(items[it]) }
        val rows = ArrayList<DdRow>(items.size)
        for ((_, indices) in byPartition) {
            if (indices.size < 2) continue
            val maxElevIndex = indices.maxByOrNull { items[it].elevationDeg } ?: continue
            val maxElev = items[maxElevIndex].elevationDeg
            val sample = items[indices.first()]
            val activePriorRefs =
                priors
                    .filter {
                        it.constellationType == sample.constellationType &&
                            abs(it.wavelengthMeters - sample.wavelengthMeters) < WAVELENGTH_MATCH_TOLERANCE_METERS
                    }.mapTo(HashSet()) { it.refSvid }
            val hysteresisRef =
                if (activePriorRefs.isEmpty()) {
                    null
                } else {
                    indices
                        .filter {
                            items[it].svid in activePriorRefs &&
                                items[it].elevationDeg >= maxElev - REF_HYSTERESIS_DEG &&
                                items[it].elevationDeg >= MIN_HYSTERESIS_REF_ELEVATION_DEG
                        }.maxByOrNull { items[it].elevationDeg }
                }
            val ref = hysteresisRef ?: maxElevIndex
            for (i in indices) {
                if (i != ref) rows.add(DdRow(i, ref))
            }
        }
        return rows
    }

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
                degreesOfFreedom = (rows.size - 3).coerceAtLeast(0),
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
    private const val REF_HYSTERESIS_DEG = 3.0
    private const val MIN_HYSTERESIS_REF_ELEVATION_DEG = 15.0
    private const val WAVELENGTH_MATCH_TOLERANCE_METERS = 1e-5
    private const val PRIOR_CARRIER_FLOOR_METERS = 0.10
    private const val PRIOR_CARRIER_RESIDUAL_FLOOR_METERS = 0.25
    private const val PRIOR_CARRIER_RESIDUAL_SIGMA = 4.0
    private const val PRIOR_POSITION_CHI2_LIMIT = 11.3
    private const val PRIOR_PULL_SIGMA_LIMIT = 4.0

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
        val degreesOfFreedom: Int = 0,
    )

    /**
     * Attempts a carrier-phase integer-fixed solution. Forms a float solution (position + DD integer
     * ambiguities) from code and carrier double differences, resolves the ambiguities with
     * [LambdaAmbiguityResolver], and — only if the ratio test passes and the fixed position agrees
     * with [floatRoverEcef] — re-solves the position with the ambiguities held at their integers,
     * yielding a centimetre-class fix.
     *
     * Double differences are formed per (constellation, wavelength) partition; the DD ambiguity is
     * an integer only when both satellites share the same wavelength and system. GLONASS inputs are
     * excluded because FDMA wavelengths differ per satellite and cannot form integer-valued DDs
     * across satellites.
     *
     * Returns null whenever fixing is not safe (too few carrier satellites, the ratio test
     * fails, or the fixed position disagrees with the float). The caller must then keep its float
     * solution: this method never degrades the float result, it can only upgrade it.
     *
     * [priors] are previous-epoch double-difference ambiguities. Priors are pivoted onto the
     * current reference satellite when the reference switches, gated against the unconstrained
     * float's combined innovation variance plus post-fit consistency, and the surviving subset's
     * posterior is passed to [onFloat] after partial-fix subsetting.
     */
    fun solveFixed(
        baseEcef: DoubleArray,
        inputs: List<CarrierSatelliteInput>,
        initialRover: DoubleArray,
        floatRoverEcef: DoubleArray,
        ratioThreshold: Double = LambdaAmbiguityResolver.DEFAULT_RATIO_THRESHOLD,
        priors: List<AmbiguityState> = emptyList(),
        onFloat: (List<AmbiguityState>) -> Unit = {},
        maxFixFloatSeparationMeters: Double = MAX_FIX_FLOAT_SEPARATION_METERS,
    ): FixedSolution? {
        val activeInputs = inputs.filter { it.constellationType != Constellation.GLONASS }
        if (activeInputs.size < MIN_FIX_SATELLITES) return null
        val rows = buildCarrierRows(activeInputs, priors)
        val ambiguityCount = rows.size
        if (ambiguityCount < MIN_PARTIAL_AMBIGUITIES) return null

        val pivotedPriors = pivotPriors(activeInputs, rows, priors)
        val baseRange = DoubleArray(activeInputs.size) { distance(activeInputs[it].baseSatelliteEcef, baseEcef) }
        val sdCode = DoubleArray(activeInputs.size) { activeInputs[it].roverCodeMeters - activeInputs[it].baseCodeMeters }
        val sdCarrier = DoubleArray(activeInputs.size) { activeInputs[it].roverCarrierMeters - activeInputs[it].baseCarrierMeters }

        val unconstrained =
            iterateFloat(initialRover, activeInputs, rows, baseRange, sdCode, sdCarrier, emptyList()) ?: return null
        val kept =
            pivotedPriors
                .filter { prior ->
                    val index = ambiguityIndex(activeInputs, rows, prior)
                    if (index < 0) return@filter false
                    if (!prior.cycles.isFinite() || !prior.varianceCycles2.isFinite() || prior.varianceCycles2 <= 0.0) {
                        return@filter false
                    }
                    val unconstrainedVar =
                        unconstrained.covariance
                            .getOrNull(index)
                            ?.getOrNull(index)
                            ?.takeIf { it.isFinite() && it >= 0.0 } ?: 0.0
                    val innovSigma = sqrt(unconstrainedVar + prior.varianceCycles2)
                    val limit = max(AMBIGUITY_DISAGREEMENT_CYCLES, AMBIGUITY_DISAGREEMENT_SIGMA * innovSigma)
                    abs(unconstrained.ambiguities[index] - prior.cycles) <= limit
                }.toMutableList()

        var estimate = unconstrained
        var evictedPriorCount = 0
        while (kept.isNotEmpty()) {
            if (evictedPriorCount > 0 && kept.size < MIN_PARTIAL_AMBIGUITIES) {
                kept.clear()
                estimate = unconstrained
                break
            }
            val candidate =
                iterateFloat(unconstrained.position, activeInputs, rows, baseRange, sdCode, sdCarrier, kept)
            if (candidate == null) {
                kept.clear()
                estimate = unconstrained
                break
            }
            val inconsistentIndex =
                findInconsistentPriorIndex(
                    inputs = activeInputs,
                    rows = rows,
                    baseRange = baseRange,
                    sdCarrier = sdCarrier,
                    unconstrained = unconstrained,
                    candidate = candidate,
                    kept = kept,
                )
            if (inconsistentIndex < 0) {
                estimate = candidate
                break
            }
            kept.removeAt(inconsistentIndex)
            evictedPriorCount++
        }

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
            val reducedPriors = kept.filter { ambiguityIndex(activeInputs, activeRows, it) >= 0 }
            val nextEstimate =
                iterateFloat(
                    activeEstimate.position,
                    activeInputs,
                    activeRows,
                    baseRange,
                    sdCode,
                    sdCarrier,
                    reducedPriors,
                ) ?: break
            activeEstimate = nextEstimate
        }

        val fixedRows = acceptedRows
        val fixedAmbiguities = acceptedCycles
        if (fixedRows == null || fixedAmbiguities == null) {
            onFloat(cleanFloatStates(activeInputs, rows, baseRange, sdCarrier, estimate))
            return null
        }
        val ratio = acceptedRatio

        var xFixed = activeEstimate.position.copyOf()
        var fixedConverged = false
        var finalSystem: LinearSystem? = null
        var finalSolution: GnssPositionSolver.WlsSolution? = null
        for (iter in 0 until MAX_ITERATIONS) {
            val system = buildFixedSystem(xFixed, activeInputs, fixedRows, baseRange, sdCarrier, fixedAmbiguities)
            val solution = GnssPositionSolver.solve(system.a, system.b, system.w, needCovariance = fixedConverged)
            if (solution == null) {
                onFloat(cleanFloatStates(activeInputs, rows, baseRange, sdCarrier, estimate))
                return null
            }
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
        val system = finalSystem
        val solution = finalSolution
        if (system == null || solution == null) {
            onFloat(cleanFloatStates(activeInputs, rows, baseRange, sdCarrier, estimate))
            return null
        }

        // Last guard: a correct fix must agree with the validated code-float position.
        if (distance(xFixed, floatRoverEcef) > maxFixFloatSeparationMeters) {
            onFloat(cleanFloatStates(activeInputs, rows, baseRange, sdCarrier, estimate))
            return null
        }

        onFloat(floatStates(activeInputs, activeRows, activeEstimate.ambiguities, activeEstimate.covariance))

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
            degreesOfFreedom = (fixedRows.size - 3).coerceAtLeast(0),
        )
    }

    private data class FloatEstimate(
        val position: DoubleArray,
        val ambiguities: DoubleArray,
        val covariance: Array<DoubleArray>,
        val positionCovariance: Array<DoubleArray>,
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
        var positionCovariance: Array<DoubleArray>? = null
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
                positionCovariance =
                    Array(3) { i ->
                        DoubleArray(3) { j -> solution.covarianceMatrix[i][j] }
                    }
                break
            }
            if (norm(doubleArrayOf(solution.deltaX[0], solution.deltaX[1], solution.deltaX[2])) < CONVERGENCE_METERS) {
                converged = true
            }
        }
        val ambiguities = ambiguityFloat ?: return null
        val covariance = ambiguityCovariance ?: return null
        val posCovariance = positionCovariance ?: return null
        return FloatEstimate(x, ambiguities, covariance, posCovariance)
    }

    private fun pivotPriors(
        inputs: List<CarrierSatelliteInput>,
        rows: List<DdRow>,
        priors: List<AmbiguityState>,
    ): List<AmbiguityState> {
        if (priors.isEmpty()) return emptyList()
        val result = ArrayList<AmbiguityState>(priors.size)
        val rowsByPartition = rows.groupBy { carrierPartitionKey(inputs[it.satIndex]) }
        for ((_, partitionRows) in rowsByPartition) {
            val refSat = inputs[partitionRows.first().refIndex]
            val newRefSvid = refSat.svid
            if (newRefSvid < 0) continue
            val partitionPriors =
                priors.filter {
                    it.constellationType == refSat.constellationType &&
                        abs(it.wavelengthMeters - refSat.wavelengthMeters) < WAVELENGTH_MATCH_TOLERANCE_METERS &&
                        it.cycles.isFinite() &&
                        it.varianceCycles2.isFinite() &&
                        it.varianceCycles2 > 0.0
                }
            for (prior in partitionPriors) {
                if (prior.refSvid == newRefSvid) result += prior
            }
            val oldRefSvids = partitionPriors.map { it.refSvid }.distinct().filter { it != newRefSvid }
            for (oldRefSvid in oldRefSvids) {
                val pivot =
                    partitionPriors.firstOrNull {
                        it.refSvid == oldRefSvid && it.satSvid == newRefSvid
                    } ?: continue
                for (prior in partitionPriors) {
                    if (prior.refSvid != oldRefSvid) continue
                    if (prior.satSvid == newRefSvid) {
                        result +=
                            prior.copy(
                                satSvid = oldRefSvid,
                                refSvid = newRefSvid,
                                cycles = -pivot.cycles,
                                varianceCycles2 = pivot.varianceCycles2,
                            )
                    } else {
                        result +=
                            prior.copy(
                                refSvid = newRefSvid,
                                cycles = prior.cycles - pivot.cycles,
                                varianceCycles2 = prior.varianceCycles2 + pivot.varianceCycles2,
                            )
                    }
                }
            }
        }
        return result
    }

    private fun findInconsistentPriorIndex(
        inputs: List<CarrierSatelliteInput>,
        rows: List<DdRow>,
        baseRange: DoubleArray,
        sdCarrier: DoubleArray,
        unconstrained: FloatEstimate,
        candidate: FloatEstimate,
        kept: List<AmbiguityState>,
    ): Int {
        val m = kept.size
        val rowIndices = IntArray(m) { ambiguityIndex(inputs, rows, kept[it]) }
        val d = DoubleArray(m)
        val s = Array(m) { DoubleArray(m) }
        for (i in 0 until m) {
            val ri = rowIndices[i]
            if (ri < 0) return i
            d[i] = unconstrained.ambiguities[ri] - kept[i].cycles
            val floorCycles = PRIOR_CARRIER_FLOOR_METERS / inputs[rows[ri].satIndex].wavelengthMeters
            for (j in 0 until m) {
                val rj = rowIndices[j]
                s[i][j] = unconstrained.covariance[ri][rj]
                if (i == j) {
                    s[i][j] += kept[i].varianceCycles2 + floorCycles * floorCycles
                }
            }
        }
        val sInv = GnssMathUtils.invertSquareMatrix(s)
        var mahalanobisChi2 = 0.0
        var baardaWorstIndex = -1
        var baardaWorstStat = -1.0
        if (sInv != null) {
            val z = DoubleArray(m)
            for (i in 0 until m) {
                var sum = 0.0
                for (j in 0 until m) sum += sInv[i][j] * d[j]
                z[i] = sum
                mahalanobisChi2 += d[i] * sum
            }
            for (i in 0 until m) {
                val diag = sInv[i][i]
                val stat = if (diag > 1e-12) (z[i] * z[i]) / diag else 0.0
                if (stat > baardaWorstStat) {
                    baardaWorstStat = stat
                    baardaWorstIndex = i
                }
            }
        }

        var maxPullSigma = 0.0
        var maxCarrierRes = 0.0
        var maxCarrierExceeded = false
        for ((index, row) in rows.withIndex()) {
            val sat = row.satIndex
            val ref = row.refIndex
            val ddGeom =
                (distance(candidate.position, inputs[sat].satelliteEcef) - baseRange[sat]) -
                    (distance(candidate.position, inputs[ref].satelliteEcef) - baseRange[ref])
            val residualMeters =
                abs((sdCarrier[sat] - sdCarrier[ref]) - ddGeom - inputs[sat].wavelengthMeters * candidate.ambiguities[index])
            if (residualMeters > maxCarrierRes) maxCarrierRes = residualMeters
            val satSigma = inputs[sat].carrierSigmaMeters
            val refSigma = inputs[ref].carrierSigmaMeters
            val ddCarrierSigma = sqrt(2.0 * (satSigma * satSigma + refSigma * refSigma))
            if (residualMeters > max(PRIOR_CARRIER_RESIDUAL_FLOOR_METERS, PRIOR_CARRIER_RESIDUAL_SIGMA * ddCarrierSigma)) {
                maxCarrierExceeded = true
            }
        }

        val dx =
            doubleArrayOf(
                candidate.position[0] - unconstrained.position[0],
                candidate.position[1] - unconstrained.position[1],
                candidate.position[2] - unconstrained.position[2],
            )
        val qPosInv = GnssMathUtils.invertSquareMatrix(unconstrained.positionCovariance)
        var posChi2 = 0.0
        if (qPosInv != null) {
            for (i in 0 until 3) {
                for (j in 0 until 3) {
                    posChi2 += dx[i] * qPosInv[i][j] * dx[j]
                }
            }
        }

        var worstPullIndex = -1
        var worstScore = -1.0
        for ((kIdx, prior) in kept.withIndex()) {
            val rowIdx = rowIndices[kIdx]
            val priorPullSigma = abs(candidate.ambiguities[rowIdx] - prior.cycles) / sqrt(prior.varianceCycles2)
            if (priorPullSigma > maxPullSigma) maxPullSigma = priorPullSigma
            val innovVar =
                (unconstrained.covariance.getOrNull(rowIdx)?.getOrNull(rowIdx) ?: 0.0).coerceAtLeast(0.0) +
                    prior.varianceCycles2
            val innovScore = abs(unconstrained.ambiguities[rowIdx] - prior.cycles) / sqrt(innovVar)
            val score = max(priorPullSigma, innovScore)
            if (score > worstScore) {
                worstScore = score
                worstPullIndex = kIdx
            }
        }

        val chi2Exceeded = sInv != null && mahalanobisChi2 > (11.0 + 3.0 * m)
        val inconsistent =
            chi2Exceeded || maxCarrierExceeded || maxPullSigma > PRIOR_PULL_SIGMA_LIMIT || posChi2 > PRIOR_POSITION_CHI2_LIMIT
        if (!inconsistent) return -1
        return if (chi2Exceeded && baardaWorstIndex >= 0) baardaWorstIndex else worstPullIndex
    }

    private fun cleanFloatStates(
        inputs: List<CarrierSatelliteInput>,
        rows: List<DdRow>,
        baseRange: DoubleArray,
        sdCarrier: DoubleArray,
        estimate: FloatEstimate,
    ): List<AmbiguityState> {
        val survivingIndices =
            rows.indices.filter { index ->
                val row = rows[index]
                val sat = row.satIndex
                val ref = row.refIndex
                val ddGeom =
                    (distance(estimate.position, inputs[sat].satelliteEcef) - baseRange[sat]) -
                        (distance(estimate.position, inputs[ref].satelliteEcef) - baseRange[ref])
                val residualMeters =
                    abs((sdCarrier[sat] - sdCarrier[ref]) - ddGeom - inputs[sat].wavelengthMeters * estimate.ambiguities[index])
                val satSigma = inputs[sat].carrierSigmaMeters
                val refSigma = inputs[ref].carrierSigmaMeters
                val ddCarrierSigma = sqrt(2.0 * (satSigma * satSigma + refSigma * refSigma))
                residualMeters <= PRIOR_CARRIER_RESIDUAL_SIGMA * ddCarrierSigma
            }
        if (survivingIndices.isEmpty()) return emptyList()
        val cleanRows = survivingIndices.map { rows[it] }
        val cleanAmbiguities = DoubleArray(survivingIndices.size) { estimate.ambiguities[survivingIndices[it]] }
        val cleanCovariance =
            Array(survivingIndices.size) { i ->
                DoubleArray(survivingIndices.size) { j ->
                    estimate.covariance[survivingIndices[i]][survivingIndices[j]]
                }
            }
        return floatStates(inputs, cleanRows, cleanAmbiguities, cleanCovariance)
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
                abs(sat.wavelengthMeters - prior.wavelengthMeters) < WAVELENGTH_MATCH_TOLERANCE_METERS
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
}
