package app.simt.gnss.core.model

/**
 * Android-neutral GNSS constellation identifiers.
 *
 * The integer values are identical to `android.location.GnssStatus.CONSTELLATION_*`, so a value
 * read from the platform can be passed straight through and compared here. Keeping the constants
 * in the pure-JVM core removes any dependency on the Android framework.
 */
object Constellation {
    const val UNKNOWN = 0
    const val GPS = 1
    const val SBAS = 2
    const val GLONASS = 3
    const val QZSS = 4
    const val BEIDOU = 5
    const val GALILEO = 6
    const val IRNSS = 7
}
