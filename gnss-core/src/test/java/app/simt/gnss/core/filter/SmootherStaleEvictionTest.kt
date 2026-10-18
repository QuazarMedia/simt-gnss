package app.simt.gnss.core.filter
import app.simt.gnss.core.model.AdrState
import app.simt.gnss.core.model.Constellation
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class SmootherStaleEvictionTest {
    private val satKey = Triple(Constellation.GPS, 7, 1575)

    @Test
    fun hatchFilter_evictsStateAfterStaleEpochs() {
        val f = HatchFilter()
        f.beginNewMeasurementsEpoch()
        val first =
            f.smooth(
                signalKey = satKey,
                rawPseudorange = 100.0,
                currentAdr = 0.0,
                adrState = AdrState.VALID,
                epochTimeNanos = 1_000_000_000L,
                pseudorangeRateMetersPerSecond = 0.0,
                pseudorangeRateUncertaintyMetersPerSecond = 0.0,
                clockStepMeters = 0.0,
            )
        assertTrue(first.isFinite())

        repeat(121) { f.beginNewMeasurementsEpoch() }

        val afterStale =
            f.smooth(
                signalKey = satKey,
                rawPseudorange = 222.0,
                currentAdr = 1.0,
                adrState = AdrState.VALID,
                epochTimeNanos = 122_000_000_000L,
                pseudorangeRateMetersPerSecond = 0.0,
                pseudorangeRateUncertaintyMetersPerSecond = 0.0,
                clockStepMeters = 0.0,
            )
        // No prior smoothed state after eviction → raw pseudorange path
        assertEquals(222.0, afterStale, 1e-9)
    }

    @Test
    fun dopplerSmoother_evictsStateAfterStaleEpochs() {
        val f = DopplerSmoother()
        f.beginNewMeasurementsEpoch()
        val t0 = 1_000_000_000L
        f.smooth(
            signalKey = satKey,
            rawPseudorange = 50.0,
            pseudorangeRateMetersPerSecond = 0.0,
            pseudorangeRateUncertaintyMetersPerSecond = 0.01,
            epochTimeNanos = t0,
            clockStepMeters = 0.0,
        )

        repeat(121) { f.beginNewMeasurementsEpoch() }

        val afterStale =
            f.smooth(
                signalKey = satKey,
                rawPseudorange = 99.0,
                pseudorangeRateMetersPerSecond = 0.0,
                pseudorangeRateUncertaintyMetersPerSecond = 0.01,
                epochTimeNanos = t0 + 1_000_000_000L,
                clockStepMeters = 0.0,
            )
        assertEquals(99.0, afterStale, 1e-9)
    }
}
