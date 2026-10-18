package app.simt.gnss.core.solver

import app.simt.gnss.core.utils.GnssMathUtils
import app.simt.gnss.core.utils.GnssMathUtils.SPEED_OF_LIGHT
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.sin
import kotlin.math.sqrt

class RtkDoubleDifferenceSolverTest {
    private data class Sat(val azDeg: Double, val elDeg: Double)

    private val baseLatDeg = 47.0
    private val baseLonDeg = 8.0
    private val slantRangeM = 22_000_000.0

    private val sats =
        listOf(
            Sat(30.0, 72.0),
            Sat(95.0, 48.0),
            Sat(160.0, 30.0),
            Sat(225.0, 58.0),
            Sat(290.0, 22.0),
            Sat(50.0, 18.0),
            Sat(200.0, 64.0),
        )

    @Test
    fun solve_recoversRoverToDecimeterFromDoubleDifferences() {
        val scenario = buildScenario(blunderSatIndex = -1)
        val solution = RtkDoubleDifferenceSolver.solve(scenario.baseEcef, scenario.inputs, scenario.baseEcef)

        assertNotNull(solution)
        requireNotNull(solution)
        val error = distance(solution.roverEcef, scenario.roverTrueEcef)
        assertTrue("DD code solution should be sub-metre, was $error m", error < 0.6)
        assertEquals("no satellites should be excluded in the clean case", sats.size, solution.usedSatelliteCount)
        assertEquals(3, solution.covariance.size)
    }

    @Test
    fun solve_rejectsOutlierSatelliteAndStillRecovers() {
        val scenario = buildScenario(blunderSatIndex = 3, blunderMeters = 25.0)
        val solution = RtkDoubleDifferenceSolver.solve(scenario.baseEcef, scenario.inputs, scenario.baseEcef)

        assertNotNull(solution)
        requireNotNull(solution)
        val error = distance(solution.roverEcef, scenario.roverTrueEcef)
        assertTrue("solution should recover after excluding the outlier, error was $error m", error < 0.6)
        assertTrue("the blundered satellite should be excluded", solution.usedSatelliteCount < sats.size)
    }

    @Test
    fun solve_returnsNullBelowMinimumSatellites() {
        val scenario = buildScenario(blunderSatIndex = -1)
        val tooFew = scenario.inputs.take(RtkDoubleDifferenceSolver.MIN_SATELLITES - 1)
        assertNull(RtkDoubleDifferenceSolver.solve(scenario.baseEcef, tooFew, scenario.baseEcef))
    }

    @Test
    fun solveFixed_resolvesAmbiguitiesAndReachesCentimeter() {
        val scenario = buildCarrierScenario(carrierNoiseMeters = 0.002) // ~2 mm carrier noise
        val floatSolution = RtkDoubleDifferenceSolver.solve(scenario.baseEcef, scenario.codeInputs, scenario.baseEcef)
        requireNotNull(floatSolution)

        val fixed =
            RtkDoubleDifferenceSolver.solveFixed(
                baseEcef = scenario.baseEcef,
                inputs = scenario.carrierInputs,
                initialRover = scenario.baseEcef,
                floatRoverEcef = floatSolution.roverEcef,
            )

        assertNotNull("a clean carrier scenario should fix", fixed)
        requireNotNull(fixed)
        val error = distance(fixed.roverEcef, scenario.roverTrueEcef)
        assertTrue("fixed solution should be centimetre-class, was $error m", error < 0.05)
        assertTrue("ratio ${fixed.ratio} should pass the test", fixed.ratio >= 3.0)
        // And it must improve on the code float it upgraded from.
        assertTrue(error < distance(floatSolution.roverEcef, scenario.roverTrueEcef))
    }

    @Test
    fun solveFixed_refusesToFixWhenCarrierIsTooNoisy() {
        val scenario = buildCarrierScenario(carrierNoiseMeters = 0.20) // 20 cm: far too noisy to fix
        val floatSolution = RtkDoubleDifferenceSolver.solve(scenario.baseEcef, scenario.codeInputs, scenario.baseEcef)
        requireNotNull(floatSolution)

        val fixed =
            RtkDoubleDifferenceSolver.solveFixed(
                baseEcef = scenario.baseEcef,
                inputs = scenario.carrierInputs,
                initialRover = scenario.baseEcef,
                floatRoverEcef = floatSolution.roverEcef,
            )

        assertNull("must refuse to fix (fall back to float) when the ratio test cannot pass", fixed)
    }

    @Test
    fun solveFixed_carriedIntegerFixesANoisyEpochTheSingleEpochRefuses() {
        val scenario = buildCarrierScenario(carrierNoiseMeters = 0.06)
        val floatSolution = RtkDoubleDifferenceSolver.solve(scenario.baseEcef, scenario.codeInputs, scenario.baseEcef)
        requireNotNull(floatSolution)
        assertNull(
            "the noisy epoch alone does not fix",
            RtkDoubleDifferenceSolver.solveFixed(
                baseEcef = scenario.baseEcef,
                inputs = scenario.carrierInputs,
                initialRover = scenario.baseEcef,
                floatRoverEcef = floatSolution.roverEcef,
            ),
        )

        val fixed =
            RtkDoubleDifferenceSolver.solveFixed(
                baseEcef = scenario.baseEcef,
                inputs = scenario.carrierInputs,
                initialRover = scenario.baseEcef,
                floatRoverEcef = floatSolution.roverEcef,
                priors = carriedPriors(scenario, cycleOffset = 0.0),
            )

        assertNotNull(fixed)
        requireNotNull(fixed)
        val error = distance(fixed.roverEcef, scenario.roverTrueEcef)
        val floatError = distance(floatSolution.roverEcef, scenario.roverTrueEcef)
        assertTrue(
            "a carried integer should beat the code float ($floatError m), was $error m",
            error < floatError && error < 0.25,
        )
        assertTrue(fixed.ratio >= 3.0)
    }

    @Test
    fun solveFixed_dropsAPriorThatDisagreesWithTheFloat() {
        val scenario = buildCarrierScenario(carrierNoiseMeters = 0.06)
        val floatSolution = RtkDoubleDifferenceSolver.solve(scenario.baseEcef, scenario.codeInputs, scenario.baseEcef)
        requireNotNull(floatSolution)
        var unconstrained = emptyList<RtkDoubleDifferenceSolver.AmbiguityState>()
        var withBadPrior = emptyList<RtkDoubleDifferenceSolver.AmbiguityState>()

        val rejected =
            RtkDoubleDifferenceSolver.solveFixed(
                baseEcef = scenario.baseEcef,
                inputs = scenario.carrierInputs,
                initialRover = scenario.baseEcef,
                floatRoverEcef = floatSolution.roverEcef,
                priors = carriedPriors(scenario, cycleOffset = 5.0),
                onFloat = { withBadPrior = it },
            )
        RtkDoubleDifferenceSolver.solveFixed(
            baseEcef = scenario.baseEcef,
            inputs = scenario.carrierInputs,
            initialRover = scenario.baseEcef,
            floatRoverEcef = floatSolution.roverEcef,
            onFloat = { unconstrained = it },
        )

        assertNull(rejected)
        assertEquals(unconstrained.size, withBadPrior.size)
        for (index in unconstrained.indices) {
            assertEquals(unconstrained[index].cycles, withBadPrior[index].cycles, 1e-6)
        }
    }

    @Test
    fun solveFixed_partialFixDropsTheLooseAmbiguity() {
        val scenario = buildCarrierScenario(carrierNoiseMeters = 0.002, looseExtra = true)
        val floatSolution = RtkDoubleDifferenceSolver.solve(scenario.baseEcef, scenario.codeInputs, scenario.baseEcef)
        requireNotNull(floatSolution)

        val fixed =
            RtkDoubleDifferenceSolver.solveFixed(
                baseEcef = scenario.baseEcef,
                inputs = scenario.carrierInputs,
                initialRover = scenario.baseEcef,
                floatRoverEcef = floatSolution.roverEcef,
            )

        assertNotNull(fixed)
        requireNotNull(fixed)
        val error = distance(fixed.roverEcef, scenario.roverTrueEcef)
        assertTrue("partial fix should stay centimetre-class, was $error m", error < 0.05)
        assertTrue("the half-cycle satellite should be left out, used ${fixed.usedSatelliteCount}", fixed.usedSatelliteCount < scenario.carrierInputs.size)
        assertTrue(fixed.ratio >= 3.0)
    }

    private fun carriedPriors(
        scenario: CarrierScenario,
        cycleOffset: Double,
    ): List<RtkDoubleDifferenceSolver.AmbiguityState> {
        val wavelength = scenario.carrierInputs.first().wavelengthMeters
        return (1 until scenario.carrierInputs.size).map { index ->
            RtkDoubleDifferenceSolver.AmbiguityState(
                constellationType = 0,
                satSvid = index + 1,
                refSvid = 1,
                wavelengthMeters = wavelength,
                cycles = 2.0 * index + cycleOffset,
                varianceCycles2 = 1e-4,
            )
        }
    }

    private class Scenario(
        val baseEcef: DoubleArray,
        val roverTrueEcef: DoubleArray,
        val inputs: List<RtkDoubleDifferenceSolver.SatelliteInput>,
    )

    private class CarrierScenario(
        val baseEcef: DoubleArray,
        val roverTrueEcef: DoubleArray,
        val carrierInputs: List<RtkDoubleDifferenceSolver.CarrierSatelliteInput>,
        val codeInputs: List<RtkDoubleDifferenceSolver.SatelliteInput>,
    )

    private fun buildCarrierScenario(
        carrierNoiseMeters: Double,
        looseExtra: Boolean = false,
    ): CarrierScenario {
        val (bx, by, bz) = GnssMathUtils.latLonAltToEcef(baseLatDeg, baseLonDeg, 400.0)
        val baseEcef = doubleArrayOf(bx, by, bz)
        val roverTrueEcef = add(baseEcef, enuToEcefDelta(1500.0, 800.0, 0.0))
        val wavelength = SPEED_OF_LIGHT / 1_575_420_000.0
        val roverClock = 12.0
        val baseClock = -5.0

        val carrierInputs = ArrayList<RtkDoubleDifferenceSolver.CarrierSatelliteInput>()
        val codeInputs = ArrayList<RtkDoubleDifferenceSolver.SatelliteInput>()
        for ((i, sat) in sats.withIndex()) {
            val elRad = Math.toRadians(sat.elDeg)
            val azRad = Math.toRadians(sat.azDeg)
            val satEcef =
                add(
                    baseEcef,
                    enuToEcefDelta(
                        Math.cos(elRad) * sin(azRad) * slantRangeM,
                        Math.cos(elRad) * Math.cos(azRad) * slantRangeM,
                        sin(elRad) * slantRangeM,
                    ),
                )
            val commonBias = 1.5 * pseudo(i * 7 + 1) + 2.0 / sin(elRad)
            val roverRange = distance(roverTrueEcef, satEcef)
            val baseRange = distance(baseEcef, satEcef)
            val roverCodeNoise = pseudo(i * 13 + 5) * 0.3
            val baseCodeNoise = pseudo(i * 13 + 9) * 0.3
            val roverCarrierNoise = pseudo(i * 17 + 2) * carrierNoiseMeters
            val baseCarrierNoise = pseudo(i * 17 + 6) * carrierNoiseMeters
            val roverN = (1000 + i * 7).toDouble()
            val baseN = (1000 + i * 5).toDouble()

            val roverCode = roverRange + roverClock + commonBias + roverCodeNoise
            val baseCode = baseRange + baseClock + commonBias + baseCodeNoise

            carrierInputs.add(
                RtkDoubleDifferenceSolver.CarrierSatelliteInput(
                    satelliteEcef = satEcef,
                    baseSatelliteEcef = satEcef, // simultaneous epochs in this synthetic scenario
                    roverCodeMeters = roverCode,
                    baseCodeMeters = baseCode,
                    roverCarrierMeters = roverRange + roverClock + commonBias + roverN * wavelength + roverCarrierNoise,
                    baseCarrierMeters = baseRange + baseClock + commonBias + baseN * wavelength + baseCarrierNoise,
                    wavelengthMeters = wavelength,
                    elevationDeg = sat.elDeg,
                    codeSigmaMeters = 0.5,
                    carrierSigmaMeters = 0.01,
                    svid = i + 1,
                ),
            )
            codeInputs.add(
                RtkDoubleDifferenceSolver.SatelliteInput(
                    satelliteEcef = satEcef,
                    baseSatelliteEcef = satEcef, // simultaneous epochs in this synthetic scenario
                    roverObservationMeters = roverCode,
                    baseObservationMeters = baseCode,
                    elevationDeg = sat.elDeg,
                    sigmaMeters = 0.5,
                ),
            )
        }
        if (looseExtra) {
            val elRad = Math.toRadians(35.0)
            val azRad = Math.toRadians(120.0)
            val satEcef =
                add(
                    baseEcef,
                    enuToEcefDelta(
                        Math.cos(elRad) * sin(azRad) * slantRangeM,
                        Math.cos(elRad) * Math.cos(azRad) * slantRangeM,
                        sin(elRad) * slantRangeM,
                    ),
                )
            val roverRange = distance(roverTrueEcef, satEcef)
            val baseRange = distance(baseEcef, satEcef)
            val roverCode = roverRange + roverClock
            val baseCode = baseRange + baseClock
            carrierInputs.add(
                RtkDoubleDifferenceSolver.CarrierSatelliteInput(
                    satelliteEcef = satEcef,
                    baseSatelliteEcef = satEcef,
                    roverCodeMeters = roverCode,
                    baseCodeMeters = baseCode,
                    roverCarrierMeters = roverRange + roverClock + 0.5 * wavelength,
                    baseCarrierMeters = baseRange + baseClock,
                    wavelengthMeters = wavelength,
                    elevationDeg = 35.0,
                    codeSigmaMeters = 0.5,
                    carrierSigmaMeters = 5.0,
                    svid = 99,
                ),
            )
            codeInputs.add(
                RtkDoubleDifferenceSolver.SatelliteInput(
                    satelliteEcef = satEcef,
                    baseSatelliteEcef = satEcef,
                    roverObservationMeters = roverCode,
                    baseObservationMeters = baseCode,
                    elevationDeg = 35.0,
                    sigmaMeters = 0.5,
                ),
            )
        }
        return CarrierScenario(baseEcef, roverTrueEcef, carrierInputs, codeInputs)
    }

    private fun buildScenario(
        blunderSatIndex: Int,
        blunderMeters: Double = 0.0,
    ): Scenario {
        val (bx, by, bz) = GnssMathUtils.latLonAltToEcef(baseLatDeg, baseLonDeg, 400.0)
        val baseEcef = doubleArrayOf(bx, by, bz)
        val roverTrueEcef = add(baseEcef, enuToEcefDelta(1500.0, 800.0, 0.0))

        val roverClock = 12.0
        val baseClock = -5.0
        val sigma = 0.5

        val inputs =
            sats.mapIndexed { i, sat ->
                val elRad = Math.toRadians(sat.elDeg)
                val azRad = Math.toRadians(sat.azDeg)
                val satEcef =
                    add(
                        baseEcef,
                        enuToEcefDelta(
                            Math.cos(elRad) * sin(azRad) * slantRangeM,
                            Math.cos(elRad) * Math.cos(azRad) * slantRangeM,
                            sin(elRad) * slantRangeM,
                        ),
                    )
                val commonBias = 1.5 * pseudo(i * 7 + 1) + 2.0 / sin(elRad)
                val roverNoise = pseudo(i * 13 + 5) * 0.3
                val baseNoise = pseudo(i * 13 + 9) * 0.3
                val blunder = if (i == blunderSatIndex) blunderMeters else 0.0

                RtkDoubleDifferenceSolver.SatelliteInput(
                    satelliteEcef = satEcef,
                    baseSatelliteEcef = satEcef, // simultaneous epochs in this synthetic scenario
                    roverObservationMeters = distance(roverTrueEcef, satEcef) + roverClock + commonBias + roverNoise + blunder,
                    baseObservationMeters = distance(baseEcef, satEcef) + baseClock + commonBias + baseNoise,
                    elevationDeg = sat.elDeg,
                    sigmaMeters = sigma,
                )
            }
        return Scenario(baseEcef, roverTrueEcef, inputs)
    }

    private fun enuToEcefDelta(
        east: Double,
        north: Double,
        up: Double,
    ): DoubleArray {
        val lat = Math.toRadians(baseLatDeg)
        val lon = Math.toRadians(baseLonDeg)
        val sinLat = sin(lat)
        val cosLat = Math.cos(lat)
        val sinLon = sin(lon)
        val cosLon = Math.cos(lon)
        return doubleArrayOf(
            -sinLon * east - sinLat * cosLon * north + cosLat * cosLon * up,
            cosLon * east - sinLat * sinLon * north + cosLat * sinLon * up,
            cosLat * north + sinLat * up,
        )
    }

    private fun add(
        a: DoubleArray,
        b: DoubleArray,
    ) = doubleArrayOf(a[0] + b[0], a[1] + b[1], a[2] + b[2])

    private fun distance(
        a: DoubleArray,
        b: DoubleArray,
    ): Double {
        val dx = a[0] - b[0]
        val dy = a[1] - b[1]
        val dz = a[2] - b[2]
        return sqrt(dx * dx + dy * dy + dz * dz)
    }

    @Test
    fun baseLag_satelliteClockChange_cancelsOnceEachObservationRemovesIt() {
        val lagSeconds = 30.0
        val tRover = 1_000.0
        val tBase = tRover - lagSeconds
        val af1 = 1.0e-11
        val af1Ref = -1.0e-11
        val geometric = 22_000_000.0

        fun singleDifference(
            rate: Double,
            removeClock: Boolean,
        ): Double {
            val clockRover = rate * tRover
            val clockBase = rate * tBase
            val rover = geometric - SPEED_OF_LIGHT * clockRover
            val base = geometric - SPEED_OF_LIGHT * clockBase
            return if (removeClock) {
                RtkDoubleDifferenceSolver.withoutSatelliteClock(rover, clockRover) -
                    RtkDoubleDifferenceSolver.withoutSatelliteClock(base, clockBase)
            } else {
                rover - base
            }
        }

        val rawDoubleDifference = singleDifference(af1, removeClock = false) - singleDifference(af1Ref, removeClock = false)
        val corrected = singleDifference(af1, removeClock = true) - singleDifference(af1Ref, removeClock = true)
        val oneSatelliteAtThirtySeconds = SPEED_OF_LIGHT * af1 * lagSeconds
        assertEquals(-SPEED_OF_LIGHT * (af1 - af1Ref) * lagSeconds, rawDoubleDifference, 1e-6)
        assertEquals(0.0, corrected, 1e-6)
        assertTrue(oneSatelliteAtThirtySeconds < 0.1)
        assertTrue(SPEED_OF_LIGHT * af1 * 1.0 < 0.01)
    }

    private fun pseudo(seed: Int): Double {
        val x = sin(seed * 12.9898 + 78.233) * 43758.5453
        return 2.0 * (x - Math.floor(x)) - 1.0
    }
}
