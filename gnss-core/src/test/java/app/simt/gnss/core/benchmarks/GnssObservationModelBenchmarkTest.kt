package app.simt.gnss.core.benchmarks
import app.simt.gnss.core.correction.AtmosphericCorrection
import app.simt.gnss.core.model.Constellation
import app.simt.gnss.core.orbit.SatellitePositionCalculator
import app.simt.gnss.core.utils.GnssMathUtils
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.pow
import kotlin.math.sqrt

class GnssObservationModelBenchmarkTest {
    private data class SatScenario(
        val satX: Double,
        val satY: Double,
        val satZ: Double,
        val elevationDeg: Double,
        val azimuthDeg: Double,
    )

    private fun geometricRange(
        rx: Double,
        ry: Double,
        rz: Double,
        sx: Double,
        sy: Double,
        sz: Double,
    ): Double = sqrt((sx - rx).pow(2) + (sy - ry).pow(2) + (sz - rz).pow(2))

    private fun rms(values: List<Double>): Double {
        val meanSq = values.map { it * it }.average()
        return sqrt(meanSq)
    }

    private fun deterministicCodeNoise(index: Int): Double {
        val sequence = doubleArrayOf(0.65, -0.42, 0.31, -0.58, 0.74, -0.27, 0.49, -0.36)
        return sequence[index % sequence.size]
    }

    @Test
    fun benchmarkSagnacCorrectionReducesModelError() {
        val trueLat = 48.8566
        val trueLon = 2.3522
        val trueAlt = 80.0
        val (rx, ry, rz) = GnssMathUtils.latLonAltToEcef(trueLat, trueLon, trueAlt)

        val satellites =
            listOf(
                70.0 to 10.0,
                45.0 to 75.0,
                35.0 to 140.0,
                55.0 to 220.0,
                30.0 to 300.0,
                65.0 to 350.0,
            ).map { (el, az) ->
                val (sx, sy, sz) =
                    SatellitePositionCalculator.getSatellitePositionApproximation(
                        constellationType = Constellation.GPS,
                        elevation = el,
                        azimuth = az,
                        observerLat = trueLat,
                        observerLon = trueLon,
                        observerAlt = trueAlt,
                    )
                SatScenario(sx, sy, sz, el, az)
            }

        val residualsWithoutSagnac = mutableListOf<Double>()
        val residualsWithSagnac = mutableListOf<Double>()

        satellites.forEach { sat ->
            val rho = geometricRange(rx, ry, rz, sat.satX, sat.satY, sat.satZ)
            val sagnac = sagnacCorrectionMeters(rx, ry, sat.satX, sat.satY)

            // Synthetic truth measurement: geometric + Sagnac only
            val measured = rho + sagnac

            val modelWithoutSagnac = rho
            val modelWithSagnac = rho + sagnac

            residualsWithoutSagnac += measured - modelWithoutSagnac
            residualsWithSagnac += measured - modelWithSagnac
        }

        val rmsWithout = rms(residualsWithoutSagnac)
        val rmsWith = rms(residualsWithSagnac)
        println("GNSS_BENCHMARK_SAGNAC rms_without=$rmsWithout rms_with=$rmsWith improvement_x=${rmsWithout / rmsWith}")

        // Sagnac-inclusive model should nearly eliminate this deterministic bias
        assertTrue("Expected Sagnac-aware RMS to be < 0.02m, was $rmsWith", rmsWith < 0.02)
        assertTrue("Expected at least 100x RMS improvement, without=$rmsWithout with=$rmsWith", rmsWithout / rmsWith > 100.0)
    }

    @Test
    fun benchmarkSagnacCorrectionWithDeterministicNoise() {
        val trueLat = 48.8566
        val trueLon = 2.3522
        val trueAlt = 80.0
        val (rx, ry, rz) = GnssMathUtils.latLonAltToEcef(trueLat, trueLon, trueAlt)

        val satellites =
            listOf(
                70.0 to 10.0,
                45.0 to 75.0,
                35.0 to 140.0,
                55.0 to 220.0,
                30.0 to 300.0,
                65.0 to 350.0,
            ).map { (el, az) ->
                val (sx, sy, sz) =
                    SatellitePositionCalculator.getSatellitePositionApproximation(
                        constellationType = Constellation.GPS,
                        elevation = el,
                        azimuth = az,
                        observerLat = trueLat,
                        observerLon = trueLon,
                        observerAlt = trueAlt,
                    )
                SatScenario(sx, sy, sz, el, az)
            }

        val residualsWithoutSagnac = mutableListOf<Double>()
        val residualsWithSagnac = mutableListOf<Double>()

        satellites.forEachIndexed { idx, sat ->
            val rho = geometricRange(rx, ry, rz, sat.satX, sat.satY, sat.satZ)
            val sagnac = sagnacCorrectionMeters(rx, ry, sat.satX, sat.satY)
            val noise = deterministicCodeNoise(idx)

            val measured = rho + sagnac + noise

            val modelWithoutSagnac = rho
            val modelWithSagnac = rho + sagnac

            residualsWithoutSagnac += measured - modelWithoutSagnac
            residualsWithSagnac += measured - modelWithSagnac
        }

        val rmsWithout = rms(residualsWithoutSagnac)
        val rmsWith = rms(residualsWithSagnac)
        println("GNSS_BENCHMARK_NOISY_SAGNAC rms_without=$rmsWithout rms_with=$rmsWith improvement_x=${rmsWithout / rmsWith}")

        assertTrue("Expected noisy Sagnac-aware RMS to be lower", rmsWith < rmsWithout)
        assertTrue("Expected noisy Sagnac improvement >= 2x, without=$rmsWithout with=$rmsWith", rmsWithout / rmsWith > 2.0)
    }

    @Test
    fun benchmarkIteratedStateCorrectionsReduceModelErrorVsSeedState() {
        // True receiver state
        val trueLat = 37.7749
        val trueLon = -122.4194
        val trueAlt = 30.0

        // Seed state deliberately offset (~1-2 km horizontal, +120m vertical)
        val seedLat = trueLat + 0.015
        val seedLon = trueLon - 0.015
        val seedAlt = trueAlt + 120.0

        val gpsTimeMs = 1_737_800_000_000L

        val satellites =
            listOf(
                75.0 to 20.0,
                50.0 to 95.0,
                25.0 to 160.0,
                60.0 to 240.0,
                35.0 to 300.0,
                45.0 to 350.0,
            )

        val errorsUsingSeedState = mutableListOf<Double>()
        val errorsUsingIteratedState = mutableListOf<Double>()

        satellites.forEach { (elev, azim) ->
            // Truth satellite approximation from true receiver state
            val (satXTrue, satYTrue, satZTrue) =
                SatellitePositionCalculator.getSatellitePositionApproximation(
                    constellationType = Constellation.GPS,
                    elevation = elev,
                    azimuth = azim,
                    observerLat = trueLat,
                    observerLon = trueLon,
                    observerAlt = trueAlt,
                )

            // Legacy-like satellite approximation from stale seed state
            val (satXSeed, satYSeed, satZSeed) =
                SatellitePositionCalculator.getSatellitePositionApproximation(
                    constellationType = Constellation.GPS,
                    elevation = elev,
                    azimuth = azim,
                    observerLat = seedLat,
                    observerLon = seedLon,
                    observerAlt = seedAlt,
                )

            val (rx, ry, rz) = GnssMathUtils.latLonAltToEcef(trueLat, trueLon, trueAlt)

            val geometricTrue = geometricRange(rx, ry, rz, satXTrue, satYTrue, satZTrue)
            val geometricSeed = geometricRange(rx, ry, rz, satXSeed, satYSeed, satZSeed)

            val tropoTrue = AtmosphericCorrection.computeTroposphericDelay(elev, trueAlt, trueLat)
            val tropoSeed = AtmosphericCorrection.computeTroposphericDelay(elev, seedAlt, seedLat)

            val ionoTrue = AtmosphericCorrection.computeIonosphericDelay(elev, azim, trueLat, trueLon, gpsTimeMs)
            val ionoSeed = AtmosphericCorrection.computeIonosphericDelay(elev, azim, seedLat, seedLon, gpsTimeMs)

            val measured = geometricTrue + tropoTrue + ionoTrue

            val modelUsingSeed = geometricSeed + tropoSeed + ionoSeed
            val modelUsingIterated = geometricTrue + tropoTrue + ionoTrue

            errorsUsingSeedState += measured - modelUsingSeed
            errorsUsingIteratedState += measured - modelUsingIterated
        }

        val rmsSeed = rms(errorsUsingSeedState)
        val rmsIterated = rms(errorsUsingIteratedState)
        println("GNSS_BENCHMARK_ITERATED_STATE rms_seed=$rmsSeed rms_iterated=$rmsIterated improvement_x=${rmsSeed / rmsIterated}")

        assertTrue("Expected iterated-state model RMS < 0.02m, was $rmsIterated", rmsIterated < 0.02)
        assertTrue("Expected >=10x improvement over seed-state model, seed=$rmsSeed iterated=$rmsIterated", rmsSeed / rmsIterated > 10.0)
    }

    @Test
    fun benchmarkIteratedStateCorrectionsWithDeterministicNoise() {
        val trueLat = 37.7749
        val trueLon = -122.4194
        val trueAlt = 30.0

        val seedLat = trueLat + 0.015
        val seedLon = trueLon - 0.015
        val seedAlt = trueAlt + 120.0

        val gpsTimeMs = 1_737_800_000_000L

        val satellites =
            listOf(
                75.0 to 20.0,
                50.0 to 95.0,
                25.0 to 160.0,
                60.0 to 240.0,
                35.0 to 300.0,
                45.0 to 350.0,
            )

        val errorsUsingSeedState = mutableListOf<Double>()
        val errorsUsingIteratedState = mutableListOf<Double>()

        satellites.forEachIndexed { idx, (elev, azim) ->
            val (satXTrue, satYTrue, satZTrue) =
                SatellitePositionCalculator.getSatellitePositionApproximation(
                    constellationType = Constellation.GPS,
                    elevation = elev,
                    azimuth = azim,
                    observerLat = trueLat,
                    observerLon = trueLon,
                    observerAlt = trueAlt,
                )

            val (satXSeed, satYSeed, satZSeed) =
                SatellitePositionCalculator.getSatellitePositionApproximation(
                    constellationType = Constellation.GPS,
                    elevation = elev,
                    azimuth = azim,
                    observerLat = seedLat,
                    observerLon = seedLon,
                    observerAlt = seedAlt,
                )

            val (rx, ry, rz) = GnssMathUtils.latLonAltToEcef(trueLat, trueLon, trueAlt)

            val geometricTrue = geometricRange(rx, ry, rz, satXTrue, satYTrue, satZTrue)
            val geometricSeed = geometricRange(rx, ry, rz, satXSeed, satYSeed, satZSeed)

            val tropoTrue = AtmosphericCorrection.computeTroposphericDelay(elev, trueAlt, trueLat)
            val tropoSeed = AtmosphericCorrection.computeTroposphericDelay(elev, seedAlt, seedLat)

            val ionoTrue = AtmosphericCorrection.computeIonosphericDelay(elev, azim, trueLat, trueLon, gpsTimeMs)
            val ionoSeed = AtmosphericCorrection.computeIonosphericDelay(elev, azim, seedLat, seedLon, gpsTimeMs)

            val noise = deterministicCodeNoise(idx)
            val measured = geometricTrue + tropoTrue + ionoTrue + noise

            val modelUsingSeed = geometricSeed + tropoSeed + ionoSeed
            val modelUsingIterated = geometricTrue + tropoTrue + ionoTrue

            errorsUsingSeedState += measured - modelUsingSeed
            errorsUsingIteratedState += measured - modelUsingIterated
        }

        val rmsSeed = rms(errorsUsingSeedState)
        val rmsIterated = rms(errorsUsingIteratedState)
        println("GNSS_BENCHMARK_NOISY_ITERATED_STATE rms_seed=$rmsSeed rms_iterated=$rmsIterated improvement_x=${rmsSeed / rmsIterated}")

        assertTrue("Expected noisy iterated-state RMS to be lower", rmsIterated < rmsSeed)
        assertTrue("Expected noisy iterated-state improvement >= 20x, seed=$rmsSeed iterated=$rmsIterated", rmsSeed / rmsIterated > 20.0)
    }

    /** IS-GPS-200 first-order Sagnac term — kept test-side; production rotates satellite ECEF instead. */
    private fun sagnacCorrectionMeters(
        receiverX: Double,
        receiverY: Double,
        satelliteX: Double,
        satelliteY: Double,
    ): Double = (GnssMathUtils.OMEGA_EARTH / GnssMathUtils.SPEED_OF_LIGHT) * (satelliteX * receiverY - satelliteY * receiverX)
}
