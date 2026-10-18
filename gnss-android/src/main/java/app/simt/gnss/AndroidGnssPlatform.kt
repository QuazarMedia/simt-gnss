package app.simt.gnss

import android.os.SystemClock
import android.util.Log
import app.simt.gnss.core.platform.GnssLogger
import app.simt.gnss.core.platform.GnssPlatform
import app.simt.gnss.core.platform.TimeSource

/**
 * Installs the Android-backed implementations of the platform seams declared by `:gnss-core`.
 *
 * The pure-JVM core reads its clock and logger through [GnssPlatform]; on Android those must be
 * backed by `android.os.SystemClock` (monotonic, includes deep sleep) and `android.util.Log`.
 * [GnssEngine] and [GnssMeasurementProcessor] install them on first construction, so every entry
 * point into the engine — public façade or internal processor — runs against the Android seams.
 */
internal object AndroidGnssPlatform {
    fun install() {
        GnssPlatform.timeSource = TimeSource { SystemClock.elapsedRealtime() }
        GnssPlatform.logger = LogGnssLogger
    }
}

private object LogGnssLogger : GnssLogger {
    override fun d(
        tag: String,
        message: String,
    ) {
        Log.d(tag, message)
    }

    override fun w(
        tag: String,
        message: String,
    ) {
        Log.w(tag, message)
    }

    override fun e(
        tag: String,
        message: String,
        throwable: Throwable?,
    ) {
        Log.e(tag, message, throwable)
    }
}
