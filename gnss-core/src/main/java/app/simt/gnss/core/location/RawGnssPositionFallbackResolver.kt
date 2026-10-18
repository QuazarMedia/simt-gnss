package app.simt.gnss.core.location
import app.simt.gnss.core.model.Constellation
import app.simt.gnss.core.model.PositionResult
import app.simt.gnss.core.model.SatelliteMeasurement
import app.simt.gnss.core.solver.GnssPositionSolver

fun interface RawGnssPositionComputer {
    fun compute(
        measurements: List<SatelliteMeasurement>,
        initialLat: Double,
        initialLon: Double,
        initialAlt: Double,
        usePpp: Boolean,
        gpsTimeNanos: Long,
        accuracy: GnssPositionSolver.Accuracy,
    ): PositionResult?
}

object RawGnssPositionFallbackResolver {
    /**
     * Wall-clock budget (ms) for the *fallback* stages that run after the full-set attempt fails.
     * When the full set cannot be solved, the resolver otherwise tries every constellation-exclusion,
     * top-N subset and single-constellation combination — each a full WLS+RAIM solve — which on a bad
     * epoch can run dozens of solves back-to-back and starve the next epoch (solves are serialized on
     * the epoch mutex). The primary full-set attempt always runs; the combinatorial stages stop once
     * this budget is exhausted so a pathological epoch cannot block the following one.
     */
    private const val DEFAULT_FALLBACK_BUDGET_MS = 1_500L

    fun resolve(
        measurements: List<SatelliteMeasurement>,
        initialLat: Double,
        initialLon: Double,
        initialAlt: Double,
        gpsTimeNanos: Long,
        primaryAccuracy: GnssPositionSolver.Accuracy,
        fallbackConstellationExclusions: IntArray,
        positionComputer: RawGnssPositionComputer,
        attemptLogger: ((String) -> Unit)? = null,
        fallbackBudgetMs: Long = DEFAULT_FALLBACK_BUDGET_MS,
        nowNanos: () -> Long = System::nanoTime,
    ): PositionResult? {
        attemptLogger?.invoke(
            "Starting raw GNSS fallback resolution measurements=${measurements.size} constellations=${measurements.constellationSummary()}",
        )

        // The full-set attempt is the most likely to succeed and the cheapest path, so it always runs
        // (and is not charged against the fallback budget). The deadline gates only the combinatorial
        // fallback stages below.
        attemptPositionSolutions(
            measurements = measurements,
            initialLat = initialLat,
            initialLon = initialLon,
            initialAlt = initialAlt,
            gpsTimeNanos = gpsTimeNanos,
            primaryAccuracy = primaryAccuracy,
            positionComputer = positionComputer,
            stageLabel = "full set",
            attemptLogger = attemptLogger,
        )?.let { return it }

        val deadlineNanos = nowNanos() + fallbackBudgetMs * 1_000_000L

        fun budgetExhausted(stage: String): Boolean {
            if (nowNanos() < deadlineNanos) return false
            attemptLogger?.invoke(
                "Raw GNSS fallback budget (${fallbackBudgetMs}ms) exhausted before $stage; stopping fallback search",
            )
            return true
        }

        for (constellationToExclude in fallbackConstellationExclusions) {
            val fallbackMeasurements = measurements.filterNot { it.constellationType == constellationToExclude }
            if (fallbackMeasurements.size < 4 || fallbackMeasurements.size == measurements.size) continue
            if (budgetExhausted("exclude ${constellationToExclude.constellationName()}")) return null

            attemptPositionSolutions(
                measurements = fallbackMeasurements,
                initialLat = initialLat,
                initialLon = initialLon,
                initialAlt = initialAlt,
                gpsTimeNanos = gpsTimeNanos,
                primaryAccuracy = primaryAccuracy,
                positionComputer = positionComputer,
                stageLabel = "exclude ${constellationToExclude.constellationName()}",
                attemptLogger = attemptLogger,
            )?.let { return it }
        }

        val constellationGroups =
            measurements
                .groupBy { it.constellationType }
                .values
                .sortedWith(
                    compareByDescending<List<SatelliteMeasurement>> { constellationMeasurements ->
                        constellationMeasurements.map { it.svid }.distinct().size
                    }.thenByDescending { constellationMeasurements -> constellationMeasurements.size },
                )

        for (takeCount in 2 until constellationGroups.size) {
            val subsetMeasurements = constellationGroups.take(takeCount).flatten()
            if (subsetMeasurements.size < 4 || subsetMeasurements.size == measurements.size) continue
            if (budgetExhausted("top $takeCount constellations")) return null

            attemptPositionSolutions(
                measurements = subsetMeasurements,
                initialLat = initialLat,
                initialLon = initialLon,
                initialAlt = initialAlt,
                gpsTimeNanos = gpsTimeNanos,
                primaryAccuracy = primaryAccuracy,
                includePpp = false,
                positionComputer = positionComputer,
                stageLabel = "top $takeCount constellations",
                attemptLogger = attemptLogger,
            )?.let { return it }
        }

        constellationGroups
            .filter { constellationMeasurements ->
                constellationMeasurements.map { it.svid }.distinct().size >= 4
            }.forEach { constellationMeasurements ->
                val stageLabel = "single constellation ${constellationMeasurements.first().constellationType.constellationName()}"
                if (budgetExhausted(stageLabel)) return null

                attemptPositionSolutions(
                    measurements = constellationMeasurements,
                    initialLat = initialLat,
                    initialLon = initialLon,
                    initialAlt = initialAlt,
                    gpsTimeNanos = gpsTimeNanos,
                    primaryAccuracy = primaryAccuracy,
                    includePpp = false,
                    positionComputer = positionComputer,
                    stageLabel = stageLabel,
                    attemptLogger = attemptLogger,
                )?.let { return it }
            }

        attemptLogger?.invoke(
            "Raw GNSS fallback resolution exhausted all strategies without a solution constellations=${measurements.constellationSummary()}",
        )
        return null
    }

    private fun attemptPositionSolutions(
        measurements: List<SatelliteMeasurement>,
        initialLat: Double,
        initialLon: Double,
        initialAlt: Double,
        gpsTimeNanos: Long,
        primaryAccuracy: GnssPositionSolver.Accuracy,
        positionComputer: RawGnssPositionComputer,
        stageLabel: String,
        attemptLogger: ((String) -> Unit)?,
        includePpp: Boolean = true,
    ): PositionResult? {
        val accuracyOrder =
            when (primaryAccuracy) {
                GnssPositionSolver.Accuracy.HIGH -> listOf(GnssPositionSolver.Accuracy.HIGH, GnssPositionSolver.Accuracy.MEDIUM)
                GnssPositionSolver.Accuracy.MEDIUM -> listOf(GnssPositionSolver.Accuracy.MEDIUM)
            }
        val constellationSummary = measurements.constellationSummary()

        if (includePpp) {
            accuracyOrder.forEach { candidateAccuracy ->
                attemptLogger?.invoke(
                    "Attempting $stageLabel PPP accuracy=$candidateAccuracy measurements=${measurements.size} constellations=$constellationSummary",
                )
                positionComputer.compute(
                    measurements = measurements,
                    initialLat = initialLat,
                    initialLon = initialLon,
                    initialAlt = initialAlt,
                    usePpp = true,
                    gpsTimeNanos = gpsTimeNanos,
                    accuracy = candidateAccuracy,
                )?.let { result ->
                    attemptLogger?.invoke(
                        "Solved $stageLabel PPP accuracy=$candidateAccuracy horizontalAccuracy=${result.horizontalAccuracyMeters} usedInFix=${result.usedInFixCount}",
                    )
                    return result
                }
                attemptLogger?.invoke(
                    "No solution for $stageLabel PPP accuracy=$candidateAccuracy constellations=$constellationSummary",
                )
            }
        }

        accuracyOrder.forEach { candidateAccuracy ->
            attemptLogger?.invoke(
                "Attempting $stageLabel code-only accuracy=$candidateAccuracy measurements=${measurements.size} constellations=$constellationSummary",
            )
            positionComputer.compute(
                measurements = measurements,
                initialLat = initialLat,
                initialLon = initialLon,
                initialAlt = initialAlt,
                usePpp = false,
                gpsTimeNanos = gpsTimeNanos,
                accuracy = candidateAccuracy,
            )?.let { result ->
                attemptLogger?.invoke(
                    "Solved $stageLabel code-only accuracy=$candidateAccuracy horizontalAccuracy=${result.horizontalAccuracyMeters} usedInFix=${result.usedInFixCount}",
                )
                return result
            }
            attemptLogger?.invoke(
                "No solution for $stageLabel code-only accuracy=$candidateAccuracy constellations=$constellationSummary",
            )
        }

        return null
    }
}

private fun List<SatelliteMeasurement>.constellationSummary(): String =
    groupBy { it.constellationType }
        .entries
        .sortedByDescending { (_, constellationMeasurements) -> constellationMeasurements.map { it.svid }.distinct().size }
        .joinToString(",") { (constellationType, constellationMeasurements) ->
            "${constellationType.constellationName()}=${constellationMeasurements.map { it.svid }.distinct().size}"
        }

private fun Int.constellationName(): String =
    when (this) {
        Constellation.GPS -> "GPS"
        Constellation.GLONASS -> "GLO"
        Constellation.GALILEO -> "GAL"
        Constellation.BEIDOU -> "BDS"
        Constellation.QZSS -> "QZS"
        Constellation.IRNSS -> "IRN"
        else -> toString()
    }
