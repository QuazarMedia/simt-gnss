package app.simt.gnss.core.benchmarks

import app.simt.gnss.core.solver.GnssPositionSolver
import app.simt.gnss.core.utils.GnssMathUtils
import app.simt.gnss.core.utils.GnssMathUtils.SPEED_OF_LIGHT
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Quantifies the accuracy gain of RTK double-differencing over undifferenced (absolute) positioning,
 * using a synthetic but physically-grounded scenario.
 *
 * The model injects the error budget that double-differencing is designed to remove:
 *   - a per-satellite **common-mode** bias (residual satellite clock + orbit + atmosphere left over
 *     after the absolute engine's own modeling) that is shared by the base and rover, and
 *   - per-receiver clock offsets shared across all satellites,
 *   - a small **baseline-dependent decorrelation** term (the part of the atmosphere that does *not*
 *     cancel, growing with base–rover distance),
 *   - independent measurement noise.
 *
 * The undifferenced solve (4 unknowns: position + receiver clock) absorbs the receiver clock but is
 * dragged by the per-satellite common-mode bias. The double-differenced solve (3 unknowns: the
 * baseline vector relative to the known base) cancels both the common-mode bias and the receiver
 * clocks, leaving only the decorrelation term and noise.
 *
 * This is the reference behaviour the production OSR/MSM double-difference solver must reproduce.
 */
class GnssRtkDoubleDifferenceBenchmarkTest {
    private data class Sat(val azDeg: Double, val elDeg: Double)

    private val baseLatDeg = 47.0
    private val baseLonDeg = 8.0
    private val baseAltM = 400.0
    private val slantRangeM = 22_000_000.0

    /** GPS L1 wavelength (m) — used for the carrier-phase benchmark. */
    private val l1WavelengthM = SPEED_OF_LIGHT / GnssMathUtils.L1_FREQUENCY_HZ

    /** Eight satellites spread in azimuth and elevation for a well-conditioned geometry. */
    private val sats =
        listOf(
            Sat(30.0, 72.0),
            Sat(95.0, 48.0),
            Sat(160.0, 30.0),
            Sat(225.0, 58.0),
            Sat(290.0, 22.0),
            Sat(50.0, 15.0),
            Sat(200.0, 66.0),
            Sat(330.0, 40.0),
        )

    @Test
    fun benchmarkCodeDoubleDifferenceCancelsCommonModeErrors() {
        val scenario = buildScenario(eastM = 1500.0, northM = 800.0, upM = 0.0, codeNoiseM = 0.3)

        val undiffError = solveUndifferencedError(scenario)
        val ddError = solveDoubleDifferencedError(scenario, scenario.roverCode, scenario.baseCode)

        println(
            "GNSS_BENCHMARK_RTK_CODE_DD baseline_m=${"%.0f".format(scenario.baselineLengthM)} " +
                "undiff_error_m=$undiffError dd_error_m=$ddError improvement_x=${undiffError / ddError}",
        )

        assertTrue("Double-differenced code error should be sub-metre, was $ddError m", ddError < 1.0)
        assertTrue(
            "Double-differencing should beat undifferenced positioning by >=4x (undiff=$undiffError dd=$ddError)",
            undiffError / ddError > 4.0,
        )
    }

    @Test
    fun benchmarkCarrierPhaseDoubleDifferenceReachesCentimeter() {
        val scenario = buildScenario(eastM = 1500.0, northM = 800.0, upM = 0.0, codeNoiseM = 0.3)

        // With ambiguities resolved (the LAMBDA step), the integer cycle counts are known and removed,
        // so the double difference rides on the millimetre-noise carrier instead of the decimetre code.
        val roverPhaseDeambiguated =
            DoubleArray(sats.size) { scenario.roverPhase[it] - scenario.roverAmbiguityCycles[it] * l1WavelengthM }
        val basePhaseDeambiguated =
            DoubleArray(sats.size) { scenario.basePhase[it] - scenario.baseAmbiguityCycles[it] * l1WavelengthM }

        val ddCarrierError = solveDoubleDifferencedError(scenario, roverPhaseDeambiguated, basePhaseDeambiguated)
        val ddCodeError = solveDoubleDifferencedError(scenario, scenario.roverCode, scenario.baseCode)

        println(
            "GNSS_BENCHMARK_RTK_CARRIER_DD dd_code_error_m=$ddCodeError dd_carrier_error_m=$ddCarrierError " +
                "improvement_x=${ddCodeError / ddCarrierError}",
        )

        assertTrue("Resolved-ambiguity carrier DD should reach centimetre level, was $ddCarrierError m", ddCarrierError < 0.05)
        assertTrue(
            "Fixed carrier DD should clearly beat float code DD (code=$ddCodeError carrier=$ddCarrierError)",
            ddCodeError / ddCarrierError > 3.0,
        )
    }

    @Test
    fun benchmarkRtkAccuracyDegradesWithBaselineLength() {
        // The baseline-decorrelation term grows with distance: short baselines cancel the atmosphere
        // almost perfectly; long baselines leave a residual that RTK cannot remove.
        val shortBaseline =
            solveDoubleDifferencedError(
                buildScenario(eastM = 1_000.0, northM = 0.0, upM = 0.0, codeNoiseM = 0.3).also { it.assertGeometry() },
            ) { it.roverCode to it.baseCode }
        val longBaseline =
            solveDoubleDifferencedError(
                buildScenario(eastM = 80_000.0, northM = 0.0, upM = 0.0, codeNoiseM = 0.3),
            ) { it.roverCode to it.baseCode }

        println(
            "GNSS_BENCHMARK_RTK_BASELINE short_1km_error_m=$shortBaseline long_80km_error_m=$longBaseline " +
                "degradation_x=${longBaseline / shortBaseline}",
        )

        assertTrue("Short-baseline RTK should be sub-metre, was $shortBaseline m", shortBaseline < 1.0)
        assertTrue(
            "RTK error should grow with baseline length (short=$shortBaseline long=$longBaseline)",
            longBaseline > shortBaseline,
        )
    }

    // ---- Scenario construction -------------------------------------------------------------------

    private class Scenario(
        val baseEcef: DoubleArray,
        val roverTrueEcef: DoubleArray,
        val satEcef: List<DoubleArray>,
        val roverCode: DoubleArray,
        val baseCode: DoubleArray,
        val roverPhase: DoubleArray,
        val basePhase: DoubleArray,
        val roverAmbiguityCycles: DoubleArray,
        val baseAmbiguityCycles: DoubleArray,
        val referenceSatIndex: Int,
        val baselineLengthM: Double,
    ) {
        fun assertGeometry() {
            require(satEcef.size >= 4) { "need >=4 satellites" }
        }
    }

    private fun buildScenario(
        eastM: Double,
        northM: Double,
        upM: Double,
        codeNoiseM: Double,
    ): Scenario {
        val (bx, by, bz) = GnssMathUtils.latLonAltToEcef(baseLatDeg, baseLonDeg, baseAltM)
        val baseEcef = doubleArrayOf(bx, by, bz)

        val baselineEcef = enuToEcefDelta(baseLatDeg, baseLonDeg, eastM, northM, upM)
        val roverTrueEcef = add(baseEcef, baselineEcef)
        val baselineLengthM = norm(baselineEcef)
        val baselineKm = baselineLengthM / 1000.0

        val satEcef =
            sats.map { sat ->
                val elRad = Math.toRadians(sat.elDeg)
                val azRad = Math.toRadians(sat.azDeg)
                val east = Math.cos(elRad) * sin(azRad)
                val north = Math.cos(elRad) * Math.cos(azRad)
                val up = sin(elRad)
                add(baseEcef, enuToEcefDelta(baseLatDeg, baseLonDeg, east * slantRangeM, north * slantRangeM, up * slantRangeM))
            }

        val n = sats.size
        val roverCode = DoubleArray(n)
        val baseCode = DoubleArray(n)
        val roverPhase = DoubleArray(n)
        val basePhase = DoubleArray(n)
        val roverAmbiguityCycles = DoubleArray(n)
        val baseAmbiguityCycles = DoubleArray(n)

        // Per-receiver clock offsets (metres) shared across all satellites — cancel in the double
        // difference, estimated as a free parameter in the undifferenced solve.
        val roverClock = 12.0
        val baseClock = -5.0

        for (i in 0 until n) {
            val elRad = Math.toRadians(sats[i].elDeg)
            // Common-mode residual bias (satellite clock + orbit + atmosphere left after modeling),
            // identical at base and rover; larger at low elevation (longer atmospheric path).
            val clockOrbitResidual = 1.5 * pseudo(i * 7 + 1)
            val atmosphereResidual = (0.6 + 0.4 * Math.abs(pseudo(i * 7 + 3))) / sin(elRad)
            val commonBias = clockOrbitResidual + atmosphereResidual

            // Baseline-dependent decorrelation: the part of the atmosphere that does NOT cancel,
            // proportional to baseline length (a few mm per km at zenith, reaching decimetres over
            // tens of km), worse at low elevation. This is the residual RTK cannot remove.
            val decorrelation = (baselineKm * RTK_DECORRELATION_M_PER_KM) / sin(elRad)

            val roverRange = range(roverTrueEcef, satEcef[i])
            val baseRange = range(baseEcef, satEcef[i])

            val roverCodeNoise = pseudo(i * 13 + 5) * codeNoiseM
            val baseCodeNoise = pseudo(i * 13 + 9) * codeNoiseM
            roverCode[i] = roverRange + roverClock + commonBias + decorrelation + roverCodeNoise
            baseCode[i] = baseRange + baseClock + commonBias + baseCodeNoise

            // Carrier phase: millimetre noise plus an integer-cycle ambiguity per receiver.
            val roverPhaseNoise = pseudo(i * 17 + 2) * 0.003
            val basePhaseNoise = pseudo(i * 17 + 6) * 0.003
            roverAmbiguityCycles[i] = (1000.0 + (i * 31 % 97)).toLong().toDouble()
            baseAmbiguityCycles[i] = (1000.0 + (i * 53 % 89)).toLong().toDouble()
            roverPhase[i] =
                roverRange + roverClock + commonBias + decorrelation + roverAmbiguityCycles[i] * l1WavelengthM + roverPhaseNoise
            basePhase[i] =
                baseRange + baseClock + commonBias + baseAmbiguityCycles[i] * l1WavelengthM + basePhaseNoise
        }

        val referenceSatIndex = sats.indices.maxByOrNull { sats[it].elDeg } ?: 0

        return Scenario(
            baseEcef = baseEcef,
            roverTrueEcef = roverTrueEcef,
            satEcef = satEcef,
            roverCode = roverCode,
            baseCode = baseCode,
            roverPhase = roverPhase,
            basePhase = basePhase,
            roverAmbiguityCycles = roverAmbiguityCycles,
            baseAmbiguityCycles = baseAmbiguityCycles,
            referenceSatIndex = referenceSatIndex,
            baselineLengthM = baselineLengthM,
        )
    }

    // ---- Solvers ---------------------------------------------------------------------------------

    /** Absolute (undifferenced) WLS: 3 position unknowns + 1 receiver-clock unknown. */
    private fun solveUndifferencedError(scenario: Scenario): Double {
        val obs = scenario.roverCode
        val n = obs.size
        var x = scenario.baseEcef.copyOf()
        var clock = 0.0

        repeat(MAX_ITERATIONS) {
            val a = Array(n) { DoubleArray(4) }
            val b = DoubleArray(n)
            val w = DoubleArray(n) { 1.0 }
            for (i in 0 until n) {
                val e = lineOfSight(x, scenario.satEcef[i])
                a[i][0] = -e[0]
                a[i][1] = -e[1]
                a[i][2] = -e[2]
                a[i][3] = 1.0
                b[i] = obs[i] - (range(x, scenario.satEcef[i]) + clock)
            }
            val solution =
                GnssPositionSolver.solve(a, b, w, GnssPositionSolver.Accuracy.HIGH, needCovariance = false)
                    ?: return norm(sub(x, scenario.roverTrueEcef))
            x = doubleArrayOf(x[0] + solution.deltaX[0], x[1] + solution.deltaX[1], x[2] + solution.deltaX[2])
            clock += solution.deltaX[3]
        }
        return norm(sub(x, scenario.roverTrueEcef))
    }

    private fun solveDoubleDifferencedError(
        scenario: Scenario,
        select: (Scenario) -> Pair<DoubleArray, DoubleArray>,
    ): Double {
        val (rover, base) = select(scenario)
        return solveDoubleDifferencedError(scenario, rover, base)
    }

    /**
     * Double-differenced WLS for the baseline vector: 3 position unknowns, no clock term (it cancels).
     * [roverObs]/[baseObs] are range-like observations (code pseudoranges or de-ambiguated carrier).
     */
    private fun solveDoubleDifferencedError(
        scenario: Scenario,
        roverObs: DoubleArray,
        baseObs: DoubleArray,
    ): Double {
        val n = roverObs.size
        val ref = scenario.referenceSatIndex
        val singleDiff = DoubleArray(n) { roverObs[it] - baseObs[it] }
        val baseRange = DoubleArray(n) { range(scenario.baseEcef, scenario.satEcef[it]) }
        val rows = (0 until n).filter { it != ref }

        var x = scenario.baseEcef.copyOf()
        repeat(MAX_ITERATIONS) {
            val a = Array(rows.size) { DoubleArray(3) }
            val b = DoubleArray(rows.size)
            val w = DoubleArray(rows.size) { 1.0 }
            val eRef = lineOfSight(x, scenario.satEcef[ref])
            val rangeRef = range(x, scenario.satEcef[ref])
            for ((row, k) in rows.withIndex()) {
                val eK = lineOfSight(x, scenario.satEcef[k])
                val ddObserved = singleDiff[k] - singleDiff[ref]
                val ddModeled = (range(x, scenario.satEcef[k]) - baseRange[k]) - (rangeRef - baseRange[ref])
                // d(ddModeled)/dx = (-e_k) - (-e_ref) = e_ref - e_k
                a[row][0] = eRef[0] - eK[0]
                a[row][1] = eRef[1] - eK[1]
                a[row][2] = eRef[2] - eK[2]
                b[row] = ddObserved - ddModeled
            }
            val solution =
                GnssPositionSolver.solve(a, b, w, GnssPositionSolver.Accuracy.HIGH, needCovariance = false)
                    ?: return norm(sub(x, scenario.roverTrueEcef))
            x = doubleArrayOf(x[0] + solution.deltaX[0], x[1] + solution.deltaX[1], x[2] + solution.deltaX[2])
        }
        return norm(sub(x, scenario.roverTrueEcef))
    }

    // ---- Vector / geometry helpers ---------------------------------------------------------------

    private fun enuToEcefDelta(
        latDeg: Double,
        lonDeg: Double,
        east: Double,
        north: Double,
        up: Double,
    ): DoubleArray {
        val lat = Math.toRadians(latDeg)
        val lon = Math.toRadians(lonDeg)
        val sinLat = sin(lat)
        val cosLat = Math.cos(lat)
        val sinLon = sin(lon)
        val cosLon = Math.cos(lon)
        val dx = -sinLon * east - sinLat * cosLon * north + cosLat * cosLon * up
        val dy = cosLon * east - sinLat * sinLon * north + cosLat * sinLon * up
        val dz = cosLat * north + sinLat * up
        return doubleArrayOf(dx, dy, dz)
    }

    private fun add(
        a: DoubleArray,
        b: DoubleArray,
    ) = doubleArrayOf(a[0] + b[0], a[1] + b[1], a[2] + b[2])

    private fun sub(
        a: DoubleArray,
        b: DoubleArray,
    ) = doubleArrayOf(a[0] - b[0], a[1] - b[1], a[2] - b[2])

    private fun norm(v: DoubleArray) = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])

    private fun range(
        a: DoubleArray,
        b: DoubleArray,
    ) = norm(sub(b, a))

    /** Unit line-of-sight vector pointing from [from] toward [to]. */
    private fun lineOfSight(
        from: DoubleArray,
        to: DoubleArray,
    ): DoubleArray {
        val d = sub(to, from)
        val r = norm(d)
        return doubleArrayOf(d[0] / r, d[1] / r, d[2] / r)
    }

    /** Deterministic pseudo-random value in [-1, 1] for reproducible noise. */
    private fun pseudo(seed: Int): Double {
        val x = sin(seed * 12.9898 + 78.233) * 43758.5453
        return 2.0 * (x - Math.floor(x)) - 1.0
    }

    private companion object {
        const val MAX_ITERATIONS = 10

        /**
         * Residual differential atmospheric delay per km of baseline at zenith (metres). Represents
         * the part of the ionosphere/troposphere that does not cancel between base and rover and
         * grows with separation — a few mm/km, reaching decimetres over tens of km.
         */
        const val RTK_DECORRELATION_M_PER_KM = 3.0e-3
    }
}
