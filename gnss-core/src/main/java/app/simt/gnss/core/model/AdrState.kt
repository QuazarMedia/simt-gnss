package app.simt.gnss.core.model

/**
 * Android-neutral accumulated-delta-range (ADR) state bit flags.
 *
 * The values are identical to `android.location.GnssMeasurement.ADR_STATE_*`, so a bitmask read
 * from the platform can be tested against these directly.
 */
object AdrState {
    const val VALID = 1
    const val RESET = 2
    const val CYCLE_SLIP = 4
    const val HALF_CYCLE_RESOLVED = 8
    const val HALF_CYCLE_REPORTED = 16
}
