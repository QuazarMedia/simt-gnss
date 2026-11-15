package app.simt.gnss

import android.location.GnssStatus
import android.os.SystemClock
import android.util.Log
import app.simt.gnss.core.model.SatelliteEphemeris
import app.simt.gnss.core.model.SatelliteMeasurement
import app.simt.gnss.core.utils.GnssMathUtils
import kotlin.math.*

/**
 * Broadcast-ephemeris store: the single map of (constellation, svid) to the latest
 * [SatelliteEphemeris], plus freshness/validity checks and the import path for
 * externally provided ephemerides.
 */
internal class GnssEphemerisStore {
    private val ephemerisMap = mutableMapOf<Pair<Int, Int>, SatelliteEphemeris>()

    internal fun describeFreshEphemerisCoverage(measurements: List<SatelliteMeasurement>): String {
        if (measurements.isEmpty()) return "none"

        return measurements
            .groupBy { it.constellationType }
            .entries
            .sortedBy { it.key }
            .joinToString(",") { (constellationType, constellationMeasurements) ->
                val uniqueSatKeys = constellationMeasurements.map { it.constellationType to it.svid }.distinct()
                val freshCount = uniqueSatKeys.count(::hasFreshEphemerisLocked)
                "${getConstellationName(constellationType)}=$freshCount/${uniqueSatKeys.size}"
            }
    }

    internal fun describeStoredEphemerides(): String {
        if (ephemerisMap.isEmpty()) return "none"

        return ephemerisMap
            .entries
            .groupBy { it.key.first }
            .entries
            .sortedBy { it.key }
            .joinToString(",") { (constellationType, entries) ->
                val freshCount = entries.count { (_, ephemeris) -> isFreshEphemerisLocked(ephemeris) }
                "${getConstellationName(constellationType)}=$freshCount/${entries.size}"
            }
    }

    internal fun storeEphemeris(
        satKey: Pair<Int, Int>,
        ephemeris: SatelliteEphemeris,
        sourceLabel: String,
    ) {
        ephemerisMap[satKey] = ephemeris
        Log.d(
            TAG,
            "Stored ${getConstellationName(ephemeris.constellationType)} ephemeris svid=${ephemeris.svid} source=$sourceLabel stored=${describeStoredEphemeridesLocked()}",
        )
    }

    /** TOW the ephemeris data is referenced to (orbit epoch, in GPST), for recency comparisons. */
    private fun ephemerisReferenceTow(ephemeris: SatelliteEphemeris): Double =
        when {
            ephemeris.isGlonass -> ephemeris.gloTbGpsTow
            ephemeris.isSbas -> ephemeris.sbasT0 // seconds of day; only compared SBAS-vs-SBAS
            ephemeris.constellationType == GnssStatus.CONSTELLATION_BEIDOU ->
                (ephemeris.toe + GnssMathUtils.BDT_GPST_OFFSET_SECONDS).mod(GnssMathUtils.SECONDS_IN_WEEK)
            else -> ephemeris.toe
        }

    /**
     * Import externally-obtained ephemeris records (e.g. from broadcast ephemeris download).
     *
     * A record only replaces an existing one when its *data* is more recent — closer
     * orbit-reference time (toe / GLONASS tb) to the current epoch. Comparing decode
     * wall-time (the previous behaviour) let every periodic RINEX import overwrite fresher
     * over-the-air ephemerides with up-to-day-old records, because imports always carry a
     * newer receipt timestamp.
     *
     * @param gpsTimeOfWeekSeconds current GPS TOW used as the recency reference
     * @return Number of records imported
     */
    fun importExternalEphemeris(
        records: List<SatelliteEphemeris>,
        gpsTimeOfWeekSeconds: Double,
    ): Int {
        var imported = 0
        for (eph in records) {
            val key = eph.constellationType to eph.svid
            val existing = ephemerisMap[key]
            val isFresherData =
                existing == null ||
                    abs(GnssMathUtils.wrapToHalfWeekSeconds(gpsTimeOfWeekSeconds - ephemerisReferenceTow(eph))) <
                    abs(GnssMathUtils.wrapToHalfWeekSeconds(gpsTimeOfWeekSeconds - ephemerisReferenceTow(existing)))
            if (isFresherData) {
                ephemerisMap[key] = eph
                imported++
            }
        }
        if (imported > 0) {
            Log.d(TAG, "Imported $imported external ephemeris records, stored=${describeStoredEphemeridesLocked()}")
        }
        return imported
    }

    private fun hasFreshEphemerisLocked(satKey: Pair<Int, Int>): Boolean = ephemerisMap[satKey]?.let(::isFreshEphemerisLocked) == true

    internal fun isFreshEphemerisLocked(ephemeris: SatelliteEphemeris): Boolean = SystemClock.elapsedRealtime() - ephemeris.lastUpdateTime < GnssMathUtils.EPHEMERIS_STALENESS_MS

    /**
     * Extended validity check for offline use. Keplerian records stay usable for 24 h after
     * receipt. Orbit error over that window is not ~2 m/hour: it is applied per satellite as
     * range variance from |t − toe| ([GnssMathUtils.broadcastEphemerisRangeSigmaMeters]).
     * GLONASS state-vector propagation is capped at 6 h of receipt and additionally by tb age.
     *
     * @param gpsTimeOfWeekSeconds current GPS TOW for data-validity (tb-age) checks;
     *   NaN restricts the check to receipt age (used by the logging summaries).
     */
    internal fun isUsableEphemerisLocked(
        ephemeris: SatelliteEphemeris,
        gpsTimeOfWeekSeconds: Double = Double.NaN,
    ): Boolean {
        val ageMs = SystemClock.elapsedRealtime() - ephemeris.lastUpdateTime
        val maxAge =
            when {
                ephemeris.isSbas -> GnssMathUtils.SBAS_EPHEMERIS_EXTENDED_VALIDITY_MS
                ephemeris.isGlonass -> GnssMathUtils.GLONASS_EPHEMERIS_EXTENDED_VALIDITY_MS
                else -> GnssMathUtils.EPHEMERIS_EXTENDED_VALIDITY_MS
            }
        if (ageMs >= maxAge) return false
        // GLONASS validity is bounded by *tb* age, not receipt age: the broadcast force
        // model holds for ±15 min around tb (refreshed every 30 min), and RK4 propagation
        // beyond GLONASS_MAX_PROPAGATION_SECONDS accumulates km-class orbit error.
        if (ephemeris.isGlonass && !gpsTimeOfWeekSeconds.isNaN()) {
            val tbAgeSeconds = abs(GnssMathUtils.wrapToHalfWeekSeconds(gpsTimeOfWeekSeconds - ephemeris.gloTbGpsTow))
            if (tbAgeSeconds > GnssMathUtils.GLONASS_MAX_PROPAGATION_SECONDS) return false
        }
        return true
    }

    /** |t − toe| in seconds, wrapped to half a week. SBAS has no Keplerian toe. */
    internal fun ephemerisAgeSeconds(
        ephemeris: SatelliteEphemeris,
        timeOfWeekSeconds: Double,
    ): Double {
        if (ephemeris.isSbas) return 0.0
        val referenceTow = if (ephemeris.isGlonass) ephemeris.gloTbGpsTow else ephemeris.toe
        return abs(GnssMathUtils.wrapToHalfWeekSeconds(timeOfWeekSeconds - referenceTow))
    }

    private fun describeStoredEphemeridesLocked(): String {
        if (ephemerisMap.isEmpty()) return "none"

        return ephemerisMap
            .entries
            .groupBy { it.key.first }
            .entries
            .sortedBy { it.key }
            .joinToString(",") { (constellationType, entries) ->
                val freshCount = entries.count { (_, ephemeris) -> isFreshEphemerisLocked(ephemeris) }
                val usableCount = entries.count { (_, ephemeris) -> isUsableEphemerisLocked(ephemeris) }
                if (freshCount == usableCount) {
                    "${getConstellationName(constellationType)}=$freshCount/${entries.size}"
                } else {
                    "${getConstellationName(constellationType)}=$freshCount+${usableCount - freshCount}/${entries.size}"
                }
            }
    }

    /** Ephemeris for [satKey], or null. */
    internal fun get(satKey: Pair<Int, Int>): SatelliteEphemeris? = ephemerisMap[satKey]

    /** Drops all stored ephemerides (session reset). */
    internal fun clearEphemerides() {
        ephemerisMap.clear()
    }

    companion object {
        private const val TAG = "GnssMeasurementProcessor"
    }
}
