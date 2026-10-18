package app.simt.gnss

import android.location.GnssMeasurement
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner

/**
 * Pins the carrier half-cycle ambiguity gate: an ADR state reported but not resolved must block
 * integer fixing, a fully resolved one is usable, and legacy HALs that never report the status
 * are accepted (so the gate never disables an otherwise healthy fleet).
 */
@RunWith(RobolectricTestRunner::class)
class HalfCycleAmbiguityGateTest {
    @Test
    fun halfCycleGate_blocksReportedUnresolved_acceptsResolvedAndLegacy() {
        val processor = GnssMeasurementProcessor()
        val valid = GnssMeasurement.ADR_STATE_VALID
        // Reported + unresolved -> ambiguous, must be blocked from integer fixing.
        assertTrue(
            processor.isCarrierHalfCycleAmbiguous(valid or GnssMeasurement.ADR_STATE_HALF_CYCLE_REPORTED),
        )
        // Reported + resolved -> usable.
        assertFalse(
            processor.isCarrierHalfCycleAmbiguous(
                valid or GnssMeasurement.ADR_STATE_HALF_CYCLE_REPORTED or GnssMeasurement.ADR_STATE_HALF_CYCLE_RESOLVED,
            ),
        )
        // Legacy HALs that never report the status -> accepted (don't disable the fleet).
        assertFalse(processor.isCarrierHalfCycleAmbiguous(valid))
    }
}
