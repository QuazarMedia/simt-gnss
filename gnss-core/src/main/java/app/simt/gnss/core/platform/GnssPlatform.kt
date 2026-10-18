package app.simt.gnss.core.platform

/**
 * Process-wide platform seams for the pure-JVM core.
 *
 * `:gnss-android` installs Android-backed implementations (`SystemClock` and `Log`) at startup;
 * a plain-JVM consumer keeps the defaults ([TimeSource.DEFAULT] and [GnssLogger.NONE]).
 */
object GnssPlatform {
    @Volatile
    var logger: GnssLogger = GnssLogger.NONE

    @Volatile
    var timeSource: TimeSource = TimeSource.DEFAULT
}
