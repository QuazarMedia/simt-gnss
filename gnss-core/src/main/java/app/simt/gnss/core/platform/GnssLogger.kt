package app.simt.gnss.core.platform

/**
 * Logging seam so the pure-JVM core never links against `android.util.Log`.
 *
 * `:gnss-android` installs a Log-backed implementation through [GnssPlatform]; a plain-JVM
 * consumer keeps the no-op default.
 */
interface GnssLogger {
    fun d(
        tag: String,
        message: String,
    )

    fun w(
        tag: String,
        message: String,
    )

    fun e(
        tag: String,
        message: String,
        throwable: Throwable? = null,
    )

    companion object {
        /** Default: silently discards every message. */
        val NONE: GnssLogger =
            object : GnssLogger {
                override fun d(
                    tag: String,
                    message: String,
                ) = Unit

                override fun w(
                    tag: String,
                    message: String,
                ) = Unit

                override fun e(
                    tag: String,
                    message: String,
                    throwable: Throwable?,
                ) = Unit
            }
    }
}
