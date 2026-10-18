package app.simt.gnss

import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner

/**
 * Exercises the public [SimtGnssEngine] façade: construction, the neutral diagnostics snapshot and the
 * null/empty-input contracts that do not require a real receiver.
 */
@RunWith(RobolectricTestRunner::class)
class SimtGnssEngineTest {
    @Test
    fun emptyMeasurements_yieldNoSolution() {
        val engine = SimtGnssEngine()
        assertNull(engine.computePosition(emptyList()))
        assertNull(engine.computeRtkPosition(emptyList(), gpsTimeNanos = 1_000_000_000L))
    }

    @Test
    fun diagnostics_snapshotIsPopulatedBeforeAnyFix() {
        val engine = SimtGnssEngine()
        val diagnostics = engine.diagnostics()
        // No ephemerides imported yet, but the summary must still be a coherent string.
        assertNotNull(diagnostics.storedEphemerides)
        assertFalse(diagnostics.hasFix)
        assertNull(diagnostics.lastSolution)
    }

    @Test
    fun storesAreExposedAndEmpty() {
        val engine = SimtGnssEngine()
        assertNotNull(engine.ssrStore)
        assertNotNull(engine.rtkBaseStore)
        assertFalse(engine.rtkBaseStore.hasFreshData())
    }

    @Test
    fun ingestRtcmFrame_acceptsArbitraryBytesWithoutSignal() {
        val engine = SimtGnssEngine()
        // Garbage bytes must not throw; the stream parser simply finds no frame.
        engine.ingestRtcmFrame(byteArrayOf(0x00, 0x01, 0x02, 0x03))
        assertTrue(engine.diagnostics().ssrCorrections.isNotEmpty())
    }

    @Test
    fun smoothingMode_canBeSelected() {
        val engine = SimtGnssEngine()
        engine.setSmoothingMode(SmoothingMode.DOPPLER)
        engine.setSmoothingMode(SmoothingMode.CARRIER_HATCH)
    }
}
