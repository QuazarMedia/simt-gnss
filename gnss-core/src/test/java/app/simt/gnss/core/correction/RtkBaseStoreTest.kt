package app.simt.gnss.core.correction
import app.simt.gnss.core.model.Constellation
import app.simt.gnss.core.parser.RtcmMsmParser
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class RtkBaseStoreTest {
    private var clock = 1_000L
    private val store = RtkBaseStore(nowMs = { clock })

    private fun obs(
        svid: Int,
        freqHz: Double = 1_575_420_000.0,
        constellation: Int = Constellation.GPS,
    ) = RtcmMsmParser.BaseObservation(
        constellationType = constellation,
        svid = svid,
        carrierFrequencyHz = freqHz,
        pseudorangeMeters = 21_000_000.0 + svid,
        carrierPhaseMeters = 21_000_000.0 + svid,
        cnrDbHz = 45.0,
        epochTimeMs = 0L,
    )

    @Test
    fun storesAndRetrievesBasePositionAndObservationsByBand() {
        store.setBasePosition(4_278_387.0, 601_233.0, 4_638_360.0)
        store.putObservations(listOf(obs(5), obs(12), obs(5, freqHz = 1_176_450_000.0)))

        val pos = store.getBasePosition()
        assertNotNull(pos)
        assertEquals(4_278_387.0, pos!!.xMeters, 1e-6)

        // L1 and L5 of the same satellite are distinct entries (different band).
        assertNotNull(store.getObservation(Constellation.GPS, 5, 1_575_420_000.0))
        assertNotNull(store.getObservation(Constellation.GPS, 5, 1_176_450_000.0))
        assertEquals(
            21_000_005.0,
            store.getObservation(Constellation.GPS, 5, 1_575_420_000.0)!!.pseudorangeMeters,
            1e-6,
        )
        assertNull(store.getObservation(Constellation.GPS, 99, 1_575_420_000.0))
    }

    @Test
    fun matchesFrequencyWithinSameMhzBandDespiteFloatJitter() {
        store.putObservations(listOf(obs(7, freqHz = 1_575_420_000.0)))
        // Rover reports a slightly different float for the same L1 band.
        assertNotNull(store.getObservation(Constellation.GPS, 7, 1_575_419_000.0))
    }

    @Test
    fun expiresStaleObservationsButKeepsStaticPositionLonger() {
        store.setBasePosition(1.0, 2.0, 3.0)
        store.putObservations(listOf(obs(5), obs(12), obs(20), obs(25)))
        assertTrue(store.hasFreshData())

        clock += 11_000L // past the 10 s observation lifetime
        assertNull("observations should be stale", store.getObservation(Constellation.GPS, 5, 1_575_420_000.0))
        assertFalse(store.hasFreshData())
        // The surveyed position is static and tolerated far longer than observations.
        assertNotNull("base position should still be valid", store.getBasePosition())

        clock += 120_000L
        assertNull("base position should eventually expire", store.getBasePosition())
    }

    @Test
    fun ignoresObservationsWithUnmappedFrequency() {
        store.setBasePosition(1.0, 2.0, 3.0)
        store.putObservations(listOf(obs(5, freqHz = 0.0)))
        assertFalse(store.hasFreshData(minObservations = 1))
    }
}

/**
 * Epoch-coherence and signal-preference behaviour added in the review follow-up: the base
 * clock cancels in a double difference only within one epoch, and the phone-trackable
 * (C/A-class) signal must win over P/W-class signals sharing the band.
 */
class RtkBaseStoreEpochCoherenceTest {
    private var clockMs = 0L
    private val store = app.simt.gnss.core.correction.RtkBaseStore { clockMs }

    private fun observation(
        svid: Int,
        epochMs: Long,
        signalId: Int = 2,
        pseudorange: Double = 2.0e7,
    ) = app.simt.gnss.core.parser.RtcmMsmParser.BaseObservation(
        constellationType = Constellation.GPS,
        svid = svid,
        carrierFrequencyHz = 1.57542e9,
        pseudorangeMeters = pseudorange,
        carrierPhaseMeters = null,
        cnrDbHz = 45.0,
        epochTimeMs = epochMs,
        signalId = signalId,
    )

    @org.junit.Test
    fun observationFromAnOlderEpoch_isNotServedNextToANewerOne() {
        // Epoch 1: satellites 1 and 2. Epoch 2: satellite 1 only (2 dropped out).
        store.putObservations(listOf(observation(1, 1000L), observation(2, 1000L)))
        clockMs += 1000
        store.putObservations(listOf(observation(1, 2000L)))

        org.junit.Assert.assertNotNull(store.getObservation(Constellation.GPS, 1, 1.57542e9))
        org.junit.Assert.assertNull(
            "satellite 2's old-epoch observation must not mix with the new epoch",
            store.getObservation(Constellation.GPS, 2, 1.57542e9),
        )
    }

    @org.junit.Test
    fun preferredSignal_winsRegardlessOfArrivalOrder() {
        // 1C (id 2) then 1W (id 4) in one epoch → 1C kept.
        store.putObservations(listOf(observation(1, 1000L, signalId = 2, pseudorange = 100.0)))
        store.putObservations(listOf(observation(1, 1000L, signalId = 4, pseudorange = 200.0)))
        org.junit.Assert.assertEquals(
            100.0,
            store.getObservation(Constellation.GPS, 1, 1.57542e9)!!.pseudorangeMeters,
            0.0,
        )

        // New epoch: 1W first, then 1C → 1C replaces.
        store.putObservations(listOf(observation(1, 2000L, signalId = 4, pseudorange = 300.0)))
        store.putObservations(listOf(observation(1, 2000L, signalId = 2, pseudorange = 400.0)))
        org.junit.Assert.assertEquals(
            400.0,
            store.getObservation(Constellation.GPS, 1, 1.57542e9)!!.pseudorangeMeters,
            0.0,
        )
    }

    @org.junit.Test
    fun hasFreshData_countsOnlyCurrentEpochObservations() {
        store.putObservations((1..4).map { observation(it, 1000L) })
        store.setBasePosition(1.0, 2.0, 3.0)
        org.junit.Assert.assertTrue(store.hasFreshData(minObservations = 4))

        // New epoch with only two satellites: the two stale-epoch leftovers must not count.
        store.putObservations(listOf(observation(1, 2000L), observation(2, 2000L)))
        org.junit.Assert.assertFalse(store.hasFreshData(minObservations = 4))
        org.junit.Assert.assertTrue(store.hasFreshData(minObservations = 2))
    }
}
