package app.simt.gnss

import android.location.GnssStatus
import app.simt.gnss.core.correction.SsrCorrectionApplicator
import app.simt.gnss.core.correction.SsrCorrections
import app.simt.gnss.core.model.SatelliteEphemeris
import app.simt.gnss.core.utils.GnssMathUtils
import kotlin.math.*

/**
 * SSR correction pairing: IOD matching against broadcast ephemerides and atomic
 * orbit+clock pairing. Wraps the [SsrCorrections] store supplied by the processor.
 */
internal class SsrCorrectionPipeline(internal val store: SsrCorrections.Store) {
    /**
     * Checks whether an SSR correction's IOD matches this broadcast ephemeris.
     * - GPS/QZSS/Galileo: matches `ephemeris.iode` when `>= 0`.
     * - BeiDou: matches `ephemeris.iode` (AODE) OR `(toe / 8) mod 240` (RTCM DF470).
     * - GLONASS: matches `ephemeris.iode`, `tb` 15-min index from `gloTbGpsTow`, or `0` fallback.
     */
    internal fun ssrIodMatches(
        ephemeris: SatelliteEphemeris,
        ssrIod: Int,
        leapSeconds: Int,
    ): Boolean {
        if (ssrIod < 0) return false
        return when {
            ephemeris.constellationType == GnssStatus.CONSTELLATION_BEIDOU -> {
                val toeMod240 = (ephemeris.toe / 8.0).roundToInt().mod(240)
                when {
                    ephemeris.iode >= 0 -> ssrIod == ephemeris.iode || ssrIod == toeMod240
                    ephemeris.toe > 0.0 -> ssrIod == toeMod240
                    else -> true
                }
            }
            ephemeris.isGlonass || ephemeris.constellationType == GnssStatus.CONSTELLATION_GLONASS -> {
                val gloTodSeconds =
                    (ephemeris.gloTbGpsTow - leapSeconds + GnssMathUtils.GLONASS_UTC_OFFSET_SECONDS)
                        .mod(GnssMathUtils.SECONDS_IN_DAY)
                val tbIndex = (gloTodSeconds / 900.0).roundToInt().mod(96)
                ssrIod == ephemeris.iode || ssrIod == tbIndex || ssrIod == 0
            }
            ephemeris.iode >= 0 -> ssrIod == ephemeris.iode
            else -> true
        }
    }

    /**
     * Atomically pairs IOD-matched SSR orbit and clock corrections when [usePpp] is enabled.
     * Applying orbit without clock (or vice versa) breaks radial-clock cancellation.
     */
    internal fun getApplicableSsrPair(
        constellationType: Int,
        svid: Int,
        ephemeris: SatelliteEphemeris,
        usePpp: Boolean,
        leapSeconds: Int,
    ): Pair<SsrCorrections.OrbitCorrection, SsrCorrections.ClockCorrection>? {
        if (!usePpp) return null
        val orbit = store.getOrbit(constellationType, svid) ?: return null
        if (!ssrIodMatches(ephemeris, orbit.iod, leapSeconds)) return null
        val clock = store.getClock(constellationType, svid) ?: return null
        if (orbit.iodSsr >= 0 && clock.iodSsr >= 0 && orbit.iodSsr != clock.iodSsr) return null
        if (clock.iod >= 0) {
            if (clock.iod != orbit.iod) return null
        } else if (clock.iodSsr < 0 || clock.iodSsr != orbit.iodSsr) {
            return null
        }
        return orbit to clock
    }

    internal fun polynomialAgeSeconds(
        epochTime: Double,
        updateIntervalSeconds: Double,
        timescale: SsrCorrections.EpochTimescale,
        gpstSeconds: Double,
        currentTimeSeconds: Double,
        leapSeconds: Int,
    ): Double =
        when (timescale) {
            SsrCorrections.EpochTimescale.GLONASS_TOD -> {
                val gloCurrentTodSeconds =
                    (gpstSeconds - leapSeconds + GnssMathUtils.GLONASS_UTC_OFFSET_SECONDS)
                        .mod(GnssMathUtils.SECONDS_IN_DAY)
                SsrCorrectionApplicator.ssrPolynomialAgeSeconds(
                    gloCurrentTodSeconds,
                    epochTime,
                    updateIntervalSeconds,
                    periodSeconds = GnssMathUtils.SECONDS_IN_DAY,
                )
            }
            SsrCorrections.EpochTimescale.BDT ->
                SsrCorrectionApplicator.ssrPolynomialAgeSeconds(
                    currentTimeSeconds,
                    epochTime,
                    updateIntervalSeconds,
                    periodSeconds = GnssMathUtils.SECONDS_IN_WEEK,
                )
            SsrCorrections.EpochTimescale.GPST ->
                SsrCorrectionApplicator.ssrPolynomialAgeSeconds(
                    gpstSeconds,
                    epochTime,
                    updateIntervalSeconds,
                    periodSeconds = GnssMathUtils.SECONDS_IN_WEEK,
                )
        }
}
