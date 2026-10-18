package app.simt.gnss.core.filter
import app.simt.gnss.core.model.AdrState
import app.simt.gnss.core.model.Constellation
import app.simt.gnss.core.utils.GnssMathUtils
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.abs

/**
 * The Hatch filter, keyed per signal, with the platform's pseudorange rate driving its slip
 * detector.
 *
 * Every test supplies an [EPOCH_NANOS] cadence and a rate that matches the scenario's own motion,
 * because a mismatched rate is now itself a slip signal — which is correct behaviour and makes a
 * rate-less test meaningless rather than merely lax.
 */
class HatchFilterTest {
    private val l1Key = Triple(Constellation.GPS, 7, 1575)
    private val l5Key = Triple(Constellation.GPS, 7, 1176)
    private val valid = AdrState.VALID

    @Test
    fun dualFrequency_signalsKeepIndependentFilterStates() {
        // Same satellite, two bands, wildly different ADR origins (arbitrary per channel).
        // Clean ramps for 5 epochs, then a +10 m multipath blip on the L1 pseudorange only.
        val filter = HatchFilter()
        val l1True = 2.0e7
        val l5True = 2.0e7 + 5.0
        val l1AdrOrigin = 100.0
        val l5AdrOrigin = 987_654.0 // ~1e6 m away — cross-band delta would trip every check

        for (t in 0 until 5) {
            filter.beginNewMeasurementsEpoch()
            filter.smooth(l1Key, l1True + t, l1AdrOrigin + t, valid, epoch(t), RATE_MPS, RATE_SIGMA_MPS, NO_CLOCK_STEP)
            filter.smooth(l5Key, l5True + t, l5AdrOrigin + t, valid, epoch(t), RATE_MPS, RATE_SIGMA_MPS, NO_CLOCK_STEP)
        }

        filter.beginNewMeasurementsEpoch()
        val l1Smoothed =
            filter.smooth(
                l1Key,
                l1True + 5 + BLIP_METERS,
                l1AdrOrigin + 5,
                valid,
                epoch(5),
                RATE_MPS,
                RATE_SIGMA_MPS,
                NO_CLOCK_STEP,
            )
        val l5Smoothed = filter.smooth(l5Key, l5True + 5, l5AdrOrigin + 5, valid, epoch(5), RATE_MPS, RATE_SIGMA_MPS, NO_CLOCK_STEP)

        // With an engaged filter (count = 6) the blip is attenuated to ~10/6 ≈ 1.7 m. A filter
        // that resets every epoch (the per-satellite-key failure mode) would return the raw
        // blip in full.
        assertTrue(
            "L1 blip must be attenuated by an engaged filter (got error ${abs(l1Smoothed - (l1True + 5))} m)",
            abs(l1Smoothed - (l1True + 5)) < 3.0,
        )
        // The clean L5 signal must be untouched by L1's blip or by the cross-band ADR offset.
        assertEquals(l5True + 5, l5Smoothed, 1e-6)
    }

    @Test
    fun adrReset_flushesSmoothingEvenWhenDeltaPassesNumericChecks() {
        // An ADR rebase of +20 m with a 1 m pseudorange change: |ΔADR − ΔPR| = 19 m passes the
        // 30 m consistency check and the rate agrees with the pseudorange — only the RESET flag
        // marks the delta as meaningless.
        val filter = HatchFilter()
        val pr = 2.0e7
        for (t in 0 until 4) {
            filter.beginNewMeasurementsEpoch()
            filter.smooth(l1Key, pr + t, 50.0 + t, valid, epoch(t), RATE_MPS, RATE_SIGMA_MPS, NO_CLOCK_STEP)
        }

        filter.beginNewMeasurementsEpoch()
        val out =
            filter.smooth(
                l1Key,
                pr + 4,
                50.0 + 4 + 20.0, // rebased accumulation
                valid or AdrState.RESET,
                epoch(4),
                RATE_MPS,
                RATE_SIGMA_MPS,
                NO_CLOCK_STEP,
            )

        // Reset must flush the filter: output is the raw pseudorange, not pulled ~19 m by
        // the rebased ADR delta.
        assertEquals(pr + 4, out, 1e-9)
    }

    @Test
    fun cycleSlipFlag_resetsSmoothing() {
        val filter = HatchFilter()
        val pr = 2.0e7
        for (t in 0 until 4) {
            filter.beginNewMeasurementsEpoch()
            filter.smooth(l1Key, pr + t, 50.0 + t, valid, epoch(t), RATE_MPS, RATE_SIGMA_MPS, NO_CLOCK_STEP)
        }
        filter.beginNewMeasurementsEpoch()
        val out =
            filter.smooth(
                l1Key,
                pr + 4,
                50.0 + 4 + 3.0, // small slip, below numeric thresholds
                valid or AdrState.CYCLE_SLIP,
                epoch(4),
                RATE_MPS,
                RATE_SIGMA_MPS,
                NO_CLOCK_STEP,
            )
        assertEquals(pr + 4, out, 1e-9)
    }

    @Test
    fun aFastMovingSatellite_stillSmooths() {
        // The defect this pins down. A satellite that is falling towards the antenna changes its
        // range by its own range rate, and that rate is tens to hundreds of metres per second for a
        // watch that is not moving at all. The filter used to treat any |ΔADR| over 100 m as a cycle
        // slip, on the reasoning that "100 m in 1 s" would be absurd — but 100 m in 1 s is absurd
        // for the *receiver* moving, not for the *satellite*, so on a static watch the test tripped
        // on almost every epoch of every satellite and the Hatch smoother never engaged. In
        // production terms this is not a subtle accuracy loss: the filter was inert. A reference
        // walk smoothed 0 of 2 055 updates before the fix.
        //
        // The scenario is therefore a range rate no phone can reject: -820 m/s, the order of a GPS
        // satellite near the horizon as seen by a stationary receiver. Five clean epochs, then a
        // +10 m code blip. An engaged filter attenuates the blip to ~10/6 ≈ 1.7 m; a filter that
        // resets every epoch hands back the full 10 m.
        val filter = HatchFilter()
        val startPseudorange = 2.0e7
        val rangeRate = -820.0

        for (t in 0 until 5) {
            filter.beginNewMeasurementsEpoch()
            filter.smooth(
                l1Key,
                startPseudorange + rangeRate * t,
                100.0 + rangeRate * t,
                valid,
                epoch(t),
                rangeRate,
                RATE_SIGMA_MPS,
                NO_CLOCK_STEP,
            )
        }

        filter.beginNewMeasurementsEpoch()
        val epoch = 5
        val smoothed =
            filter.smooth(
                l1Key,
                startPseudorange + rangeRate * epoch + BLIP_METERS,
                100.0 + rangeRate * epoch,
                valid,
                epoch(epoch),
                rangeRate,
                RATE_SIGMA_MPS,
                NO_CLOCK_STEP,
            )
        val truth = startPseudorange + rangeRate * epoch

        assertTrue(
            "a satellite moving at $rangeRate m/s must still be smoothed: the blip survived at " +
                "${abs(smoothed - truth)} m, so the filter reset on the satellite's own motion",
            abs(smoothed - truth) < BLIP_METERS / 2.0,
        )
    }

    @Test
    fun aSignalWithNoUsableRate_stillSmooths() {
        // The other half of the same defect. A receiver that cannot report a rate must not be read
        // as one that has slipped: with no physical prediction available, the old fixed-jump check
        // was the only thing standing between a fast satellite and a reset, and it fell every time.
        // The flag-based detectors and the code-consistency check remain; what is gone is the
        // blanket reset that made the filter inert for everyone.
        val filter = HatchFilter()
        val startPseudorange = 2.0e7
        val rangeRate = -820.0

        for (t in 0 until 5) {
            filter.beginNewMeasurementsEpoch()
            filter.smooth(
                l1Key,
                startPseudorange + rangeRate * t,
                100.0 + rangeRate * t,
                valid,
                epoch(t),
                pseudorangeRateMetersPerSecond = NO_RATE,
                pseudorangeRateUncertaintyMetersPerSecond = NO_RATE,
                clockStepMeters = NO_CLOCK_STEP,
            )
        }

        filter.beginNewMeasurementsEpoch()
        val epoch = 5
        val smoothed =
            filter.smooth(
                l1Key,
                startPseudorange + rangeRate * epoch + BLIP_METERS,
                100.0 + rangeRate * epoch,
                valid,
                epoch(epoch),
                pseudorangeRateMetersPerSecond = NO_RATE,
                pseudorangeRateUncertaintyMetersPerSecond = NO_RATE,
                clockStepMeters = NO_CLOCK_STEP,
            )
        val truth = startPseudorange + rangeRate * epoch

        assertTrue(
            "a signal with no usable rate must still be smoothed: the blip survived at " +
                "${abs(smoothed - truth)} m",
            abs(smoothed - truth) < BLIP_METERS / 2.0,
        )
    }

    @Test
    fun aCarrierDeltaThatContradictsTheRangeRate_resetsSmoothing() {
        // The replacement detector, asserted in the positive direction: the pseudorange and the
        // carrier agree with each other and with nothing, but the carrier delta is 40 m where the
        // reported rate predicts 0.15 m, so the carrier slipped and the filter must start over
        // rather than average the discontinuity into the output. This is the same comparison the
        // old 100 m jump test was reaching for, made against a predictor that is in the right
        // clock domain.
        val filter = HatchFilter()
        val pr = 2.0e7
        val rangeRate = -820.0
        val slipMeters = 40.0

        for (t in 0 until 5) {
            filter.beginNewMeasurementsEpoch()
            filter.smooth(
                l1Key,
                pr + rangeRate * t,
                100.0 + rangeRate * t,
                valid,
                epoch(t),
                rangeRate,
                RATE_SIGMA_MPS,
                NO_CLOCK_STEP,
            )
        }

        filter.beginNewMeasurementsEpoch()
        val epoch = 5
        // The code pseudorange also carries the slip, so the code-consistency check passes too —
        // which is the point: the Doppler residual is the only test here that can see it.
        val out =
            filter.smooth(
                l1Key,
                pr + rangeRate * epoch + slipMeters,
                100.0 + rangeRate * epoch + slipMeters,
                valid,
                epoch(epoch),
                rangeRate,
                RATE_SIGMA_MPS,
                NO_CLOCK_STEP,
            )

        assertEquals(
            "a carrier delta the rate cannot explain must reset the filter",
            pr + rangeRate * epoch + slipMeters,
            out,
            1e-9,
        )
    }

    @Test
    fun codeNoiseBelowTheDetectorFloor_keepsSmoothingEngaged() {
        // The false-slip side of the trade, and the only observable that can tell the two behaviours
        // apart. A filter that resets returns the raw pseudorange; a filter that is engaged returns
        // its own carrier prediction. Those two differ only when the *pseudorange* is the noisy one,
        // which is the physical reality this filter exists for: code is noisy, carrier is clean. So
        // the noise goes on the pseudorange and the carrier carries the truth, and a reset shows up
        // as the full noise surviving in the output.
        //
        // Five centimetres is a tenth of the detector's floor, which is the point: it is well inside
        // the region the detector must ignore. Resetting here would restore the original failure in
        // miniature — a filter that restarts on every epoch smooths nothing — and it would do so
        // silently, because a filter that resets produces plausible numbers.
        val filter = HatchFilter()
        val pr = 2.0e7
        val rangeRate = -820.0
        val noiseMeters = GnssMathUtils.ADR_DOPPLER_SLIP_FLOOR_METERS / 10.0

        for (t in 0 until 5) {
            filter.beginNewMeasurementsEpoch()
            filter.smooth(
                l1Key,
                pr + rangeRate * t,
                100.0 + rangeRate * t,
                valid,
                epoch(t),
                rangeRate,
                RATE_SIGMA_MPS,
                NO_CLOCK_STEP,
            )
        }

        filter.beginNewMeasurementsEpoch()
        val last = 5
        val truth = pr + rangeRate * last
        val smoothed =
            filter.smooth(
                l1Key,
                truth + noiseMeters,
                100.0 + rangeRate * last,
                valid,
                epoch(last),
                rangeRate,
                RATE_SIGMA_MPS,
                NO_CLOCK_STEP,
            )

        assertTrue(
            "code noise of $noiseMeters m must not stop the filter: the output carries " +
                "${abs(smoothed - truth)} m of it, so the filter reset and handed back the raw " +
                "pseudorange",
            abs(smoothed - truth) < noiseMeters / 2.0,
        )
    }

    @Test
    fun invalidAdr_leavesNoOrphanedState() {
        // The invalid-ADR branch must clear all per-signal maps (a lingering smoothingCount
        // entry previously evaded the stale eviction forever).
        val filter = HatchFilter()
        filter.beginNewMeasurementsEpoch()
        filter.smooth(l1Key, 100.0, 1.0, valid, epoch(0), RATE_MPS, RATE_SIGMA_MPS, NO_CLOCK_STEP)
        filter.smooth(l1Key, 101.0, null, 0, epoch(1), RATE_MPS, RATE_SIGMA_MPS, NO_CLOCK_STEP) // invalid ADR clears state

        // Re-engage: first call after clearing behaves like a fresh signal (returns raw).
        filter.beginNewMeasurementsEpoch()
        assertEquals(102.0, filter.smooth(l1Key, 102.0, 3.0, valid, epoch(2), RATE_MPS, RATE_SIGMA_MPS, NO_CLOCK_STEP), 1e-9)
    }

    @Test
    fun aSteadyClockStep_mustNotReadAsACodeCarrierDisagreement() {
        // The other half of #83, and the reason the clock correction has to reach the Hatch filter at
        // all. The carrier delta is pure geometry; the code pseudorange it is compared against has
        // been steered by the receiver clock. When the clock takes a step, that difference is not a
        // disagreement between the two observables — it is the step, and it appears in the residual in
        // full.
        //
        // At 100 m per epoch the residual is ten times the 30 m threshold, so the filter resets on
        // every epoch for a reason that has nothing to do with the signal. The scenario is therefore
        // built so that the *only* thing moving the pseudorange apart from the carrier is the clock:
        // the reported rate matches the carrier exactly, so the Doppler residual is zero, and a
        // blip injected at the end has to be smoothed to prove the filter stayed engaged.
        val filter = HatchFilter()
        val rate = -820.0

        for (t in 0 until 5) {
            filter.beginNewMeasurementsEpoch()
            val geometry = START_RANGE + rate * t
            filter.smooth(l1Key, geometry - t * CLOCK_STEP_METERS, ADR_ORIGIN + rate * t, valid, epoch(t), rate, RATE_SIGMA_MPS, CLOCK_STEP_METERS)
        }

        val last = 5
        val geometry = START_RANGE + rate * last
        val smoothed =
            filter.smooth(
                l1Key,
                geometry - last * CLOCK_STEP_METERS + BLIP_METERS,
                ADR_ORIGIN + rate * last,
                valid,
                epoch(last),
                rate,
                RATE_SIGMA_MPS,
                CLOCK_STEP_METERS,
            )

        val error = abs(smoothed - (geometry - last * CLOCK_STEP_METERS))
        assertTrue(
            "the carrier and the code are consistent to within the Doppler residual here, so the only " +
                "reason the filter could reset is the ${"%.1f".format(CLOCK_STEP_METERS)} m clock step landing in " +
                "the consistency residual: the blip came back at ${"%.2f".format(error)} m. The code pseudorange " +
                "is clock-steered and the carrier delta is not, so the step must be taken out of both " +
                "the residual and the propagation before either can be believed.",
            error < BLIP_METERS / 2.0,
        )
    }

    @Test
    fun codeNoiseAboveThirtyMetres_staysSmoothedWhenTheReportedSigmaCoversIt() {
        // A fixed 30 m consistency limit rejects a healthy update once the code itself is noisier
        // than that. With σ = 20 m the limit is 5·√2·σ ≈ 141 m, so a 40 m code excursion must
        // still be smoothed rather than adopted as the new truth.
        val filter = HatchFilter()
        val rate = -820.0
        val sigma = 20.0
        val excursion = 40.0
        for (t in 0 until 5) {
            filter.beginNewMeasurementsEpoch()
            val geometry = START_RANGE + rate * t
            filter.smooth(
                l1Key,
                geometry,
                ADR_ORIGIN + rate * t,
                valid,
                epoch(t),
                rate,
                RATE_SIGMA_MPS,
                NO_CLOCK_STEP,
                codeSigmaMeters = sigma,
            )
        }
        val t = 5
        val geometry = START_RANGE + rate * t
        val smoothed =
            filter.smooth(
                l1Key,
                geometry + excursion,
                ADR_ORIGIN + rate * t,
                valid,
                epoch(t),
                rate,
                RATE_SIGMA_MPS,
                NO_CLOCK_STEP,
                codeSigmaMeters = sigma,
            )
        assertTrue(
            "a ${excursion.toInt()} m code excursion at σ = ${sigma.toInt()} m came back " +
                "${"%.1f".format(kotlin.math.abs(smoothed - geometry))} m from the carrier, so the widened " +
                "consistency limit still reset the filter",
            kotlin.math.abs(smoothed - geometry) < excursion / 2.0,
        )
    }

    @Test
    fun steadyClockStepAndCodeNoise_smoothingBeatsTheRawCode() {
        // The accuracy claim for this slice, in the units the position solver consumes: range error
        // of one signal. ±10 m of code noise on top of a 100 m/epoch clock step. The raw error is
        // the noise itself (10 m every epoch). Smoothing has to beat that, or the clock step is
        // being blended in as if it were noise.
        val filter = HatchFilter()
        val rate = -820.0
        val noise = 10.0
        var rawSq = 0.0
        var smoothSq = 0.0
        var n = 0
        for (t in 0..40) {
            filter.beginNewMeasurementsEpoch()
            val clean = START_RANGE + rate * t - t * CLOCK_STEP_METERS
            val sampleNoise = whiteNoiseMeters(t, noise)
            val raw = clean + sampleNoise
            val smoothed =
                filter.smooth(
                    l1Key,
                    raw,
                    ADR_ORIGIN + rate * t,
                    valid,
                    epoch(t),
                    rate,
                    RATE_SIGMA_MPS,
                    CLOCK_STEP_METERS,
                    codeSigmaMeters = noise,
                )
            if (t >= 20) {
                rawSq += sampleNoise * sampleNoise
                val err = smoothed - clean
                smoothSq += err * err
                n++
            }
        }
        val rawRms = kotlin.math.sqrt(rawSq / n)
        val smoothRms = kotlin.math.sqrt(smoothSq / n)
        assertTrue(
            "after convergence the smoothed range rms is ${"%.2f".format(smoothRms)} m against a raw " +
                "rms of ${"%.2f".format(rawRms)} m. Smoothing has to reduce the code error, not trade it " +
                "for the clock step",
            smoothRms < rawRms / 2.0,
        )
    }

    private fun epoch(index: Int): Long = EPOCH_NANOS * index

    /** Deterministic uniform noise in ±[amplitude], so the rms is white-noise rather than an alternation the filter can cancel. */
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
        /** The multipath excursion injected into the code, in metres. */
        const val BLIP_METERS = 10.0

        /** 1 Hz epoch spacing, as nanoseconds. */
        const val EPOCH_NANOS = 1_000_000_000L

        /** The 1 m per epoch ramp the shared scenarios move at. */
        const val RATE_MPS = 1.0

        /** A plausible phone rate uncertainty, in metres per second. */
        const val RATE_SIGMA_MPS = 0.05

        /** What a receiver reports when it has no rate: a finite zero, per the platform contract. */
        const val NO_RATE = 0.0

        /** A receiver whose clock correction did not move between epochs. Most scenarios are these. */
        const val NO_CLOCK_STEP = 0.0

        /** A standalone range, far from the origin so nothing reads as a precision artefact. */
        const val START_RANGE = 24_000_000.0

        /** An arbitrary but plausible carrier accumulation base, as Android reports it. */
        const val ADR_ORIGIN = 4_321_987.0

        /**
         * The receiver clock correction this scenario applies, growing by this much every epoch.
         *
         * About 100 m of range at the speed of light, which is what a receiver's clock model takes when
         * it refits — and ten times the code-consistency threshold, so a filter that leaves the step in
         * the residual resets every epoch on a perfectly healthy signal.
         */
        const val CLOCK_STEP_NANOS = 333.5640952

        /** The same step as range, derived rather than written down so the two cannot drift apart. */
        const val CLOCK_STEP_METERS = CLOCK_STEP_NANOS * 1e-9 * GnssMathUtils.SPEED_OF_LIGHT
    }
}
