package app.simt.gnss.core.solver
import app.simt.gnss.core.model.AdrState
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class RtkAmbiguityMemoryTest {
    @Test
    fun applicablePriors_returnsTheStoredPosteriorWhenTheCarrierAgrees() {
        val memory = RtkAmbiguityMemory()
        memory.remember(listOf(state(cycles = 4.0, variance = 1.0)), agreeing(100.0, 200.0), timeNanos = 1_000_000_000L)

        val priors = memory.applicablePriors(agreeing(110.0, 210.0), timeNanos = 2_000_000_000L)

        assertEquals(1, priors.size)
        assertEquals(4.0, priors[0].cycles, 0.0)
        assertTrue(priors[0].varianceCycles2 > 1.0)
    }

    @Test
    fun applicablePriors_dropsACarrierThatSlipped() {
        val memory = RtkAmbiguityMemory()
        memory.remember(listOf(state(cycles = 4.0, variance = 1.0)), agreeing(100.0, 200.0), timeNanos = 1_000_000_000L)

        val priors = memory.applicablePriors(agreeing(110.0, 212.0), timeNanos = 2_000_000_000L)

        assertTrue(priors.isEmpty())
    }

    @Test
    fun applicablePriors_dropsThePriorWhenTheGapExceedsFiveSeconds() {
        val memory = RtkAmbiguityMemory()
        memory.remember(listOf(state(cycles = 4.0, variance = 1.0)), agreeing(100.0, 200.0), timeNanos = 1_000_000_000L)

        val priors = memory.applicablePriors(agreeing(160.0, 260.0), timeNanos = 7_000_000_000L)

        assertTrue(priors.isEmpty())
    }

    @Test
    fun applicablePriors_dropsAHalfCycleAndAMissingRate() {
        val memory = RtkAmbiguityMemory()
        memory.remember(
            listOf(state(cycles = 4.0, variance = 1.0), state(cycles = 6.0, variance = 1.0, satSvid = 8, refSvid = 2)),
            listOf(
                sample(svid = 1, adrMeters = 100.0),
                sample(svid = 2, adrMeters = 50.0),
                sample(svid = 7, adrMeters = 200.0),
                sample(svid = 8, adrMeters = 80.0),
            ),
            timeNanos = 1_000_000_000L,
        )

        val priors =
            memory.applicablePriors(
                samples =
                    listOf(
                        sample(svid = 1, adrMeters = 110.0, halfCycleAmbiguous = true),
                        sample(svid = 2, adrMeters = 60.0, rateUncertainty = 0.0),
                        sample(svid = 7, adrMeters = 210.0),
                        sample(svid = 8, adrMeters = 90.0),
                    ),
                timeNanos = 2_000_000_000L,
            )

        assertTrue(priors.isEmpty())
    }

    @Test
    fun applicablePriors_dropsPriorWhenBaseLockTimeResetsOrDecreases() {
        val memory = RtkAmbiguityMemory()
        memory.remember(
            listOf(state(cycles = 4.0, variance = 1.0)),
            listOf(
                sample(svid = 1, adrMeters = 100.0, bandMhz = 1575, baseLockTimeMillis = 60_000L),
                sample(svid = 7, adrMeters = 200.0, bandMhz = 1575, baseLockTimeMillis = 60_000L),
            ),
            timeNanos = 1_000_000_000L,
        )

        // Continuous rover ADR, but base lock time drops to 0 ms on SV 7 -> base cycle slip.
        val afterReset =
            memory.applicablePriors(
                listOf(
                    sample(svid = 1, adrMeters = 110.0, bandMhz = 1575, baseLockTimeMillis = 61_000L),
                    sample(svid = 7, adrMeters = 210.0, bandMhz = 1575, baseLockTimeMillis = 0L),
                ),
                timeNanos = 2_000_000_000L,
            )
        assertTrue("base lock reset must invalidate prior even when rover ADR is continuous", afterReset.isEmpty())

        // Continuous non-decreasing base lock time keeps the prior.
        val continuous =
            memory.applicablePriors(
                listOf(
                    sample(svid = 1, adrMeters = 110.0, bandMhz = 1575, baseLockTimeMillis = 61_000L),
                    sample(svid = 7, adrMeters = 210.0, bandMhz = 1575, baseLockTimeMillis = 61_000L),
                ),
                timeNanos = 2_000_000_000L,
            )
        assertEquals(1, continuous.size)
    }

    @Test
    fun applicablePriors_pivotsWhenOldReferenceDropsOut() {
        val memory = RtkAmbiguityMemory()
        memory.remember(
            listOf(
                state(cycles = 4.0, variance = 0.5, satSvid = 7, refSvid = 1),
                state(cycles = 9.0, variance = 0.5, satSvid = 8, refSvid = 1),
            ),
            listOf(
                sample(svid = 1, adrMeters = 100.0, bandMhz = 1575),
                sample(svid = 7, adrMeters = 200.0, bandMhz = 1575),
                sample(svid = 8, adrMeters = 300.0, bandMhz = 1575),
            ),
            timeNanos = 1_000_000_000L,
        )

        // SV 1 drops out on epoch 2; surviving SV 7 and SV 8 pivot to refSvid = 7 with N_{8,7} = 9 - 4 = 5.
        val pivoted =
            memory.applicablePriors(
                listOf(
                    sample(svid = 7, adrMeters = 210.0, bandMhz = 1575),
                    sample(svid = 8, adrMeters = 310.0, bandMhz = 1575),
                ),
                timeNanos = 2_000_000_000L,
            )
        assertEquals(1, pivoted.size)
        assertEquals(8, pivoted[0].satSvid)
        assertEquals(7, pivoted[0].refSvid)
        assertEquals(5.0, pivoted[0].cycles, 1e-9)
    }

    private fun agreeing(
        referenceAdrMeters: Double,
        satelliteAdrMeters: Double,
    ) = listOf(
        sample(svid = 1, adrMeters = referenceAdrMeters),
        sample(svid = 7, adrMeters = satelliteAdrMeters),
    )

    private fun state(
        cycles: Double,
        variance: Double,
        satSvid: Int = 7,
        refSvid: Int = 1,
    ) = RtkDoubleDifferenceSolver.AmbiguityState(
        constellationType = 1,
        satSvid = satSvid,
        refSvid = refSvid,
        wavelengthMeters = 0.1903,
        cycles = cycles,
        varianceCycles2 = variance,
    )

    private fun sample(
        adrMeters: Double,
        svid: Int = 1,
        halfCycleAmbiguous: Boolean = false,
        rateUncertainty: Double = 0.05,
        bandMhz: Int = 0,
        baseLockTimeMillis: Long = -1L,
    ) = RtkAmbiguityMemory.Sample(
        constellationType = 1,
        svid = svid,
        adrMeters = adrMeters,
        adrState = AdrState.VALID,
        rateMetersPerSecond = 10.0,
        rateUncertaintyMetersPerSecond = rateUncertainty,
        halfCycleAmbiguous = halfCycleAmbiguous,
        bandMhz = bandMhz,
        baseLockTimeMillis = baseLockTimeMillis,
    )
}
