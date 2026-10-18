package app.simt.gnss.core.benchmarks

import app.simt.gnss.core.utils.GnssMathUtils
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sqrt

class GnssWeightingAndRaimBenchmarkTest {
    private data class ResidualScenario(
        val elevationDeg: Double,
        val cn0DbHz: Double,
        val pseudorangeSigmaMeters: Double,
        val residualMeters: Double,
    )

    private fun baseWeight(
        elevationDeg: Double,
        sigmaMeters: Double,
    ): Double {
        val sinElev = sin(Math.toRadians(elevationDeg)).coerceAtLeast(0.1)
        return (1.0 / sigmaMeters.pow(2)) * (sinElev * sinElev)
    }

    private fun weightedEstimate(
        scenarios: List<ResidualScenario>,
        useCn0Weighting: Boolean,
    ): Double {
        var numerator = 0.0
        var denominator = 0.0

        scenarios.forEach { scenario ->
            val boundedSigma =
                scenario.pseudorangeSigmaMeters.coerceIn(
                    GnssMathUtils.MIN_PSEUDORANGE_SIGMA_METERS,
                    GnssMathUtils.MAX_PSEUDORANGE_SIGMA_METERS,
                )
            val base = baseWeight(scenario.elevationDeg, boundedSigma)
            val cn0Factor = if (useCn0Weighting) GnssMathUtils.cn0WeightFactor(scenario.cn0DbHz) else 1.0
            val w = base * cn0Factor
            numerator += w * scenario.residualMeters
            denominator += w
        }

        return if (denominator > 0.0) numerator / denominator else 0.0
    }

    private fun rms(values: List<Double>): Double {
        val meanSq = values.map { it * it }.average()
        return sqrt(meanSq)
    }

    @Test
    fun benchmarkCn0WeightingDeweightsLowCn0MultipathBias() {
        // High-C/N0 satellites are close to truth; low-C/N0 satellites have urban-like multipath bias.
        val scenarios =
            listOf(
                ResidualScenario(70.0, 46.0, 0.9, 0.30),
                ResidualScenario(62.0, 44.0, 1.0, -0.20),
                ResidualScenario(55.0, 42.0, 1.1, 0.25),
                ResidualScenario(48.0, 41.0, 1.2, -0.35),
                ResidualScenario(22.0, 24.0, 1.6, 14.5),
                ResidualScenario(16.0, 21.0, 1.8, 18.0),
            )

        val estimateWithoutCn0 = weightedEstimate(scenarios, useCn0Weighting = false)
        val estimateWithCn0 = weightedEstimate(scenarios, useCn0Weighting = true)

        val absErrorWithout = abs(estimateWithoutCn0)
        val absErrorWith = abs(estimateWithCn0)
        val improvement = absErrorWithout / absErrorWith

        println(
            "GNSS_BENCHMARK_CN0_WEIGHTING abs_error_without=$absErrorWithout abs_error_with=$absErrorWith improvement_x=$improvement",
        )

        assertTrue("C/N0 weighting should reduce multipath-driven solution bias", absErrorWith < absErrorWithout)
        assertTrue("Expected >=2x bias reduction from C/N0 weighting", improvement > 2.0)
    }

    @Test
    fun benchmarkRaim15mGateRejectsOutlierThat30mWouldKeep() {
        val codeResidualsMeters = listOf(0.7, 1.0, 1.2, 0.9, 18.0, 1.1)
        val maxResidual = codeResidualsMeters.maxOrNull() ?: 0.0

        val oldThreshold = 30.0
        val newThreshold = GnssMathUtils.RAIM_RESIDUAL_THRESHOLD_METERS

        val rejectedByOld = maxResidual > oldThreshold
        val rejectedByNew = maxResidual > newThreshold

        val rmsAll = rms(codeResidualsMeters)
        val rmsAfterNewGate = rms(codeResidualsMeters.filter { it <= newThreshold })

        println(
            "GNSS_BENCHMARK_RAIM_GATE max_residual=$maxResidual old_threshold=$oldThreshold new_threshold=$newThreshold rms_all=$rmsAll rms_after_new_gate=$rmsAfterNewGate",
        )

        assertTrue("Old 30m gate should keep this urban multipath outlier", !rejectedByOld)
        assertTrue("New RAIM gate should reject this outlier", rejectedByNew)
        assertTrue("Rejecting outlier should reduce residual RMS", rmsAfterNewGate < rmsAll)
    }
}
