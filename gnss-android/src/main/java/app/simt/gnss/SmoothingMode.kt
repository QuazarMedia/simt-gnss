package app.simt.gnss

/**
 * Carrier-smoothing strategy applied to the code pseudoranges before the position solve.
 *
 * - [CARRIER_HATCH] uses the Hatch carrier-smoothing filter; it needs a valid accumulated
 *   delta range (carrier phase) and falls back per-signal when the ADR state is unusable.
 * - [DOPPLER] smooths the pseudoranges with the Doppler-derived pseudorange rate instead of
 *   carrier phase, for receivers or signals where carrier phase is not available.
 */
enum class SmoothingMode {
    CARRIER_HATCH,
    DOPPLER,
}
