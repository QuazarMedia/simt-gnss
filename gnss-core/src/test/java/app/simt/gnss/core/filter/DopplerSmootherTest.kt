package app.simt.gnss.core.filter

import app.simt.gnss.core.utils.GnssMathUtils
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.abs

class DopplerSmootherTest {
    // ---------------------------------------------------------------------------------------------
    // The receiver-clock-step tests below (#84).
    //
    // The scenario is the one every real receiver runs: the hardware clock error is steady, the
    // geometry moves, and the *correction* the platform applies to the pseudorange — full bias plus
    // bias — takes a step every epoch. The reported rate is pure geometry and knows nothing about
    // that correction, so a prediction built from the rate alone starts each epoch a whole clock step
    // away from where the pseudorange actually is.
    //
    // CLOCK_STEP_NANOS is a plausible step for a warm receiver's clock model to take (about 100 m of
    // range, well under one metre of clock), and the metres are *derived* from it rather than written
    // beside it, so the pseudorange the test builds and the step the filter is handed cannot drift
    // apart by a rounding error.
    // ---------------------------------------------------------------------------------------------
    @Test
    fun smooth_reliableDoppler_reducesRawNoiseAgainstPrediction() {
        val smoother = DopplerSmoother()
        val satKey = Triple(1, 7, 1575)
        val t0 = 1_000_000_000L

        val first =
            smoother.smooth(
                signalKey = satKey,
                rawPseudorange = 20_000_000.0,
                pseudorangeRateMetersPerSecond = 500.0,
                pseudorangeRateUncertaintyMetersPerSecond = 0.3,
                epochTimeNanos = t0,
                clockStepMeters = 0.0,
            )
        assertEquals(20_000_000.0, first, 1e-9)

        val expectedPredicted = 20_000_000.0 + 500.0
        val noisyRaw = expectedPredicted + 30.0

        val second =
            smoother.smooth(
                signalKey = satKey,
                rawPseudorange = noisyRaw,
                pseudorangeRateMetersPerSecond = 500.0,
                pseudorangeRateUncertaintyMetersPerSecond = 0.3,
                epochTimeNanos = t0 + GnssMathUtils.NANOS_PER_SECOND,
                clockStepMeters = 0.0,
            )

        val rawError = abs(noisyRaw - expectedPredicted)
        val smoothedError = abs(second - expectedPredicted)

        assertTrue("Smoothed result should be closer to Doppler prediction than raw input", smoothedError < rawError)
        assertTrue("Smoothed result should stay near raw value", abs(second - noisyRaw) < rawError)
    }

    @Test
    fun smooth_unreliableDopplerRate_returnsRawPseudorange() {
        val smoother = DopplerSmoother()
        val satKey = Triple(3, 10, 1575)
        val t0 = 2_000_000_000L

        smoother.smooth(
            signalKey = satKey,
            rawPseudorange = 21_000_000.0,
            pseudorangeRateMetersPerSecond = 300.0,
            pseudorangeRateUncertaintyMetersPerSecond = 0.5,
            epochTimeNanos = t0,
            clockStepMeters = 0.0,
        )

        val raw = 20_999_700.0
        val result =
            smoother.smooth(
                signalKey = satKey,
                rawPseudorange = raw,
                pseudorangeRateMetersPerSecond = 300.0,
                pseudorangeRateUncertaintyMetersPerSecond = GnssMathUtils.DOPPLER_MAX_RATE_UNCERTAINTY_MPS + 0.5,
                epochTimeNanos = t0 + GnssMathUtils.NANOS_PER_SECOND,
                clockStepMeters = 0.0,
            )

        assertEquals("Unreliable Doppler rate should bypass smoothing", raw, result, 1e-9)
    }

    @Test
    fun reset_clearsState_andNextObservationStartsFresh() {
        val smoother = DopplerSmoother()
        val satKey = Triple(5, 12, 1575)
        val t0 = 3_000_000_000L

        smoother.smooth(
            signalKey = satKey,
            rawPseudorange = 22_000_000.0,
            pseudorangeRateMetersPerSecond = 200.0,
            pseudorangeRateUncertaintyMetersPerSecond = 0.3,
            epochTimeNanos = t0,
            clockStepMeters = 0.0,
        )

        smoother.reset()

        val rawAfterReset = 21_999_800.0
        val resultAfterReset =
            smoother.smooth(
                signalKey = satKey,
                rawPseudorange = rawAfterReset,
                pseudorangeRateMetersPerSecond = 200.0,
                pseudorangeRateUncertaintyMetersPerSecond = 0.3,
                epochTimeNanos = t0 + GnssMathUtils.NANOS_PER_SECOND,
                clockStepMeters = 0.0,
            )

        assertEquals("After reset, first sample should return raw pseudorange", rawAfterReset, resultAfterReset, 1e-9)
    }

    @Test
    fun aSteadyClockStep_mustNotMakeTheEstimateWorseThanTheCodeNoise() {
        val smoother = DopplerSmoother()
        val satKey = Triple(6, 3, 1575)
        val startNanos = 5_000_000_000L

        // A satellite crossing overhead at a constant, believable rate, and a receiver whose hardware
        // clock error does not drift: the reported rate is therefore the pure geometric rate, which is
        // all the platform can report and all the filter is given.
        //
        // The first sample is seeded and not asserted on. With no previous value there is nothing to
        // smooth against, so passing the raw pseudorange through is the correct behaviour at any
        // accuracy, and an assertion that fails on epoch 0 would be testing that rather than the clock.
        smoother.smooth(
            signalKey = satKey,
            rawPseudorange = START_RANGE,
            pseudorangeRateMetersPerSecond = -RANGE_RATE_MPS,
            pseudorangeRateUncertaintyMetersPerSecond = RATE_SIGMA_MPS,
            epochTimeNanos = startNanos,
            clockStepMeters = 0.0,
        )

        // Alternating code noise, so the smoother has something real to remove and the assertion below
        // is about the smoother rather than about a lucky epoch.
        for (epoch in 1 until EPOCH_COUNT) {
            val clean = START_RANGE - RANGE_RATE_MPS * epoch - epoch * CLOCK_STEP_METERS
            val raw = clean + if (epoch % 2 == 0) CODE_NOISE_METERS else -CODE_NOISE_METERS

            val smoothed =
                smoother.smooth(
                    signalKey = satKey,
                    rawPseudorange = raw,
                    pseudorangeRateMetersPerSecond = -RANGE_RATE_MPS,
                    pseudorangeRateUncertaintyMetersPerSecond = RATE_SIGMA_MPS,
                    epochTimeNanos = startNanos + epoch * GnssMathUtils.NANOS_PER_SECOND,
                    clockStepMeters = CLOCK_STEP_METERS,
                )

            val error = abs(smoothed - clean)
            assertTrue(
                "epoch $epoch: the smoothed range is ${"%.2f".format(error)} m from the truth, which is more than " +
                    "half of the ${"%.1f".format(CODE_NOISE_METERS)} m of code noise it exists to remove. A prediction " +
                    "that ignores the receiver's clock correction starts every epoch a whole ${"%.1f".format(CLOCK_STEP_METERS)} " +
                    "m step behind the pseudorange, and blends that step into the answer as though it were noise.",
                error < CODE_NOISE_METERS / 2.0,
            )
        }
    }

    @Test
    fun aClockStepJustUnderTheDeviationGate_mustNotBeAcceptedAsNoise() {
        val smoother = DopplerSmoother()
        val satKey = Triple(6, 4, 1575)
        val startNanos = 6_000_000_000L

        // The single most dangerous shape this defect can take. A static signal, so the rate is
        // exactly zero and the prediction is simply the previous value; then the clock takes one step
        // that is deliberately *just inside* the deviation gate. The gate compares the candidate
        // against the raw pseudorange, so a step it cannot distinguish from noise is blended in at
        // full weight and handed to the estimator as a smoothed measurement.
        smoother.smooth(
            signalKey = satKey,
            rawPseudorange = START_RANGE,
            pseudorangeRateMetersPerSecond = 0.0,
            pseudorangeRateUncertaintyMetersPerSecond = RATE_SIGMA_MPS,
            epochTimeNanos = startNanos,
            clockStepMeters = 0.0,
        )
        val raw = START_RANGE - GATE_EDGE_CLOCK_STEP_METERS
        val smoothed =
            smoother.smooth(
                signalKey = satKey,
                rawPseudorange = raw,
                pseudorangeRateMetersPerSecond = 0.0,
                pseudorangeRateUncertaintyMetersPerSecond = RATE_SIGMA_MPS,
                epochTimeNanos = startNanos + GnssMathUtils.NANOS_PER_SECOND,
                clockStepMeters = GATE_EDGE_CLOCK_STEP_METERS,
            )

        val moved = abs(smoothed - raw)
        assertTrue(
            "the smoother moved the pseudorange ${"%.1f".format(moved)} m on one epoch, from a step of only " +
                "${"%.1f".format(GATE_EDGE_CLOCK_STEP_METERS)} m that fits inside the " +
                "${"%.0f".format(GnssMathUtils.DOPPLER_MAX_RAW_DEVIATION_METERS)} m deviation gate. The gate is the only " +
                "thing standing between a clock step and a range error, so a step of this size is not " +
                "detectable by comparing against the raw pseudorange at all - it has to be predicted out.",
            moved < 1.0,
        )
    }

    @Test
    fun steadyClockStepAndWhiteCodeNoise_smoothingBeatsTheRawCode() {
        val smoother = DopplerSmoother()
        val satKey = Triple(6, 5, 1575)
        val startNanos = 7_000_000_000L
        val noise = 10.0
        var rawSq = 0.0
        var smoothSq = 0.0
        var n = 0
        for (epoch in 0..40) {
            val clean = START_RANGE - RANGE_RATE_MPS * epoch - epoch * CLOCK_STEP_METERS
            val sampleNoise = whiteNoiseMeters(epoch, noise)
            val smoothed =
                smoother.smooth(
                    signalKey = satKey,
                    rawPseudorange = clean + sampleNoise,
                    pseudorangeRateMetersPerSecond = -RANGE_RATE_MPS,
                    pseudorangeRateUncertaintyMetersPerSecond = RATE_SIGMA_MPS,
                    epochTimeNanos = startNanos + epoch * GnssMathUtils.NANOS_PER_SECOND,
                    clockStepMeters = if (epoch == 0) 0.0 else CLOCK_STEP_METERS,
                )
            if (epoch >= 20) {
                rawSq += sampleNoise * sampleNoise
                val err = smoothed - clean
                smoothSq += err * err
                n++
            }
        }
        val rawRms = kotlin.math.sqrt(rawSq / n)
        val smoothRms = kotlin.math.sqrt(smoothSq / n)
        assertTrue(
            "smoothed rms ${"%.2f".format(smoothRms)} m is not below 60% of the raw rms ${"%.2f".format(rawRms)} m",
            smoothRms < rawRms * 0.6,
        )
    }

    private fun whiteNoiseMeters(
        epoch: Int,
        amplitude: Double,
    ): Double {
        var state = 1
        repeat(epoch + 1) {
            state = state * 1103515245 + 12345
        }
        val unit = ((state ushr 16) and 0x7fff) / 32767.0
        return (unit * 2.0 - 1.0) * amplitude
    }

    private companion object {
        /** Epochs in the clock-step walk. Long enough for the smoother's recursion to reach its steady state. */
        const val EPOCH_COUNT = 6

        /** A believable standalone range, far from the origin so nothing reads as a precision artefact. */
        const val START_RANGE = 24_000_000.0

        /** A satellite crossing the face of the watch: fast, constant, and entirely ordinary. */
        const val RANGE_RATE_MPS = 700.0

        /** Typical phone code noise. The bar for "did smoothing help" is half of this. */
        const val CODE_NOISE_METERS = 20.0

        /** A rate uncertainty any receiver can report for Doppler. */
        const val RATE_SIGMA_MPS = 0.3

        /**
         * The clock correction this receiver applies, in nanoseconds, growing by this much every epoch.
         *
         * Chosen because it is what a real clock model does when it refits, and because at the speed of
         * light it is about 100 m of range - a tenth of the gate, and a hundredth of the range itself.
         */
        const val CLOCK_STEP_NANOS = 333.5640952

        /** The same step as range, derived rather than written down so the two cannot drift apart. */
        const val CLOCK_STEP_METERS = CLOCK_STEP_NANOS * 1e-9 * GnssMathUtils.SPEED_OF_LIGHT

        /**
         * How far inside the deviation gate this step is placed.
         *
         * Not a rounding margin but a deliberate choice: the gate is the only defence against a bad
         * prediction, and the whole point of the test is that a clock step *inside* the gate defeats it
         * anyway. A step exactly on the threshold would be ambiguous, so the step is set well clear of
         * both the gate and the code noise it is being mistaken for.
         */
        const val GATE_EDGE_MARGIN_METERS = 7.0

        /** A clock step sized to sit just inside the deviation gate, in metres of range. */
        const val GATE_EDGE_CLOCK_STEP_METERS = GnssMathUtils.DOPPLER_MAX_RAW_DEVIATION_METERS - GATE_EDGE_MARGIN_METERS
    }
}
