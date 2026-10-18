package app.simt.gnss.core.platform

/**
 * Monotonic elapsed-time source.
 *
 * Implementations must be monotonic and unaffected by wall-clock changes. On Android the library
 * installs a `android.os.SystemClock.elapsedRealtime()`-backed implementation; a plain-JVM
 * consumer keeps [DEFAULT].
 */
fun interface TimeSource {
    fun elapsedRealtimeMs(): Long

    companion object {
        /** JVM default: milliseconds from an arbitrary but monotonic origin. */
        val DEFAULT: TimeSource = TimeSource { System.nanoTime() / 1_000_000L }
    }
}
