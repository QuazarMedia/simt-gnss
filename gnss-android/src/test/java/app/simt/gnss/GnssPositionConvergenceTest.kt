package app.simt.gnss

import android.location.GnssStatus
import android.os.SystemClock
import app.simt.gnss.core.model.GnssSignalBand
import app.simt.gnss.core.model.SatelliteEphemeris
import app.simt.gnss.core.model.SatelliteMeasurement
import app.simt.gnss.core.solver.GnssPositionSolver
import app.simt.gnss.core.utils.GnssMathUtils
import org.junit.Assert.*
import org.junit.Before
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner

@RunWith(RobolectricTestRunner::class)
class GnssPositionConvergenceTest {
    private lateinit var processor: GnssMeasurementProcessor

    @Before
    fun setUp() {
        processor = GnssMeasurementProcessor()
    }

    @Test
    fun coldStart_antipodalSydneyFromZeroSeed_convergesWithinOneMeter() {
        val trueLat = -33.8688
        val trueLon = 151.2093
        val trueAlt = 50.0
        val (ephemerides, measurements) =
            buildSyntheticSkyEpoch(
                constellationType = GnssStatus.CONSTELLATION_GPS,
                lat = trueLat,
                lon = trueLon,
                alt = trueAlt,
                gpsTimeNanos = SYNTHETIC_GPS_TIME_NANOS,
                clockBiasMeters = 120_000.0,
            )
        assertTrue("Need >= 5 visible SVs in Sydney sky, got ${measurements.size}", measurements.size >= 5)

        val coldProcessor = GnssMeasurementProcessor()
        assertEquals(ephemerides.size, coldProcessor.importExternalEphemeris(ephemerides, (SYNTHETIC_GPS_TIME_NANOS / 1e9) % GnssMathUtils.SECONDS_IN_WEEK))

        val fix =
            coldProcessor.computePosition(
                measurements = measurements,
                initialLat = 0.0,
                initialLon = 0.0,
                initialAlt = 0.0,
                usePpp = false,
                gpsTimeNanos = SYNTHETIC_GPS_TIME_NANOS,
            )
        val nonNullFix = requireNotNull(fix) { "Cold-start from (0,0,0) must converge in Sydney" }
        val errorMeters = ecefDistanceMeters(trueLat, trueLon, trueAlt, nonNullFix.latitude, nonNullFix.longitude, nonNullFix.altitude)
        assertTrue("Cold-start antipodal error must be < 1.0 m (was $errorMeters m)", errorMeters < 1.0)
    }

    @Test
    fun stalePppState_expiresAfterThirtySeconds_andConvergesAtNewLocation() {
        val parisLat = 48.8566
        val parisLon = 2.3522
        val parisAlt = 35.0
        val t1Nanos = 100L * GnssMathUtils.NANOS_PER_SECOND
        val (parisEph, parisMeas) =
            buildSyntheticSkyEpoch(
                constellationType = GnssStatus.CONSTELLATION_GPS,
                lat = parisLat,
                lon = parisLon,
                alt = parisAlt,
                gpsTimeNanos = t1Nanos,
            )

        val tokyoLat = 35.6762
        val tokyoLon = 139.6503
        val tokyoAlt = 40.0
        val t2Nanos = 200L * GnssMathUtils.NANOS_PER_SECOND // dt = 100 s > 30 s
        val (tokyoEph, tokyoMeas) =
            buildSyntheticSkyEpoch(
                constellationType = GnssStatus.CONSTELLATION_GPS,
                lat = tokyoLat,
                lon = tokyoLon,
                alt = tokyoAlt,
                gpsTimeNanos = t2Nanos,
            )

        val pppProcessor = GnssMeasurementProcessor()
        pppProcessor.importExternalEphemeris(parisEph, (t1Nanos / 1e9) % GnssMathUtils.SECONDS_IN_WEEK)
        val parisFix =
            pppProcessor.computePosition(
                measurements = parisMeas,
                initialLat = parisLat,
                initialLon = parisLon,
                initialAlt = parisAlt,
                usePpp = true,
                gpsTimeNanos = t1Nanos,
            )
        assertNotNull(parisFix)

        val tokyoProcessorEph = tokyoEph.map { it.copy(svid = it.svid + 40) }
        val tokyoMeasShifted = tokyoMeas.map { it.copy(svid = it.svid + 40) }
        pppProcessor.importExternalEphemeris(tokyoProcessorEph, (t2Nanos / 1e9) % GnssMathUtils.SECONDS_IN_WEEK)

        val tokyoFix =
            pppProcessor.computePosition(
                measurements = tokyoMeasShifted,
                initialLat = 0.0,
                initialLon = 0.0,
                initialAlt = 0.0,
                usePpp = true,
                gpsTimeNanos = t2Nanos,
            )
        val nonNullTokyoFix = requireNotNull(tokyoFix) { "Stale PPP seed (>30 s) must expire so Tokyo fix converges from (0,0,0)" }
        val tokyoErrorMeters = ecefDistanceMeters(tokyoLat, tokyoLon, tokyoAlt, nonNullTokyoFix.latitude, nonNullTokyoFix.longitude, nonNullTokyoFix.altitude)
        assertTrue("Tokyo fix error must be < 1.0 m (was $tokyoErrorMeters m)", tokyoErrorMeters < 1.0)
    }

    @Test
    fun galileoE5bAndE5aOnly_neverPairsInto27xNoiseAmplifiedIf() {
        val trueLat = 48.8566
        val trueLon = 2.3522
        val trueAlt = 50.0
        val (ephemerides, e5aMeas) =
            buildSyntheticSkyEpoch(
                constellationType = GnssStatus.CONSTELLATION_GALILEO,
                lat = trueLat,
                lon = trueLon,
                alt = trueAlt,
                gpsTimeNanos = SYNTHETIC_GPS_TIME_NANOS,
                carrierFrequencyHz = GnssMathUtils.L5_FREQUENCY_HZ,
            )
        assertTrue(e5aMeas.size >= 5)

        // Add E5b with a +15 m code differential on the first SV: if erroneously paired with E5a into IF,
        // the 27.5x amplification injects ~412 m of range error! With E5b+E5a rejected from IF pairing,
        // single-frequency E5a stays exact (< 1 m).
        val e5bMeas =
            e5aMeas.mapIndexed { idx, m ->
                m.copy(
                    carrierFrequencyHz = GnssMathUtils.E5B_FREQUENCY_HZ,
                    pseudorangeMeters = m.pseudorangeMeters + (if (idx == 0) 15.0 else 0.0),
                    isL5 = false,
                    signalBand = GnssSignalBand.E5B_B2I,
                )
            }

        val galProcessor = GnssMeasurementProcessor()
        galProcessor.importExternalEphemeris(ephemerides, (SYNTHETIC_GPS_TIME_NANOS / 1e9) % GnssMathUtils.SECONDS_IN_WEEK)

        val fix =
            galProcessor.computePosition(
                measurements = e5bMeas + e5aMeas,
                initialLat = trueLat,
                initialLon = trueLon,
                initialAlt = trueAlt,
                usePpp = false,
                gpsTimeNanos = SYNTHETIC_GPS_TIME_NANOS,
            )
        val nonNullFix = requireNotNull(fix)
        val errorMeters = ecefDistanceMeters(trueLat, trueLon, trueAlt, nonNullFix.latitude, nonNullFix.longitude, nonNullFix.altitude)
        assertTrue("E5b+E5a must NOT form IF (error was $errorMeters m)", errorMeters < 1.0)
    }

    @Test
    fun singleSvConstellationPruning_andUnderdeterminedMultiGnss() {
        val trueLat = 48.8566
        val trueLon = 2.3522
        val trueAlt = 100.0
        val tow = (SYNTHETIC_GPS_TIME_NANOS / 1e9) % GnssMathUtils.SECONDS_IN_WEEK
        val (gpsEph, gpsAll) =
            buildSyntheticSkyEpoch(
                constellationType = GnssStatus.CONSTELLATION_GPS,
                lat = trueLat,
                lon = trueLon,
                alt = trueAlt,
                gpsTimeNanos = SYNTHETIC_GPS_TIME_NANOS,
                maxSatellites = 5,
            )
        assertEquals(5, gpsAll.size)

        val receipt = SystemClock.elapsedRealtime()
        val gloEph =
            SatelliteEphemeris(
                constellationType = GnssStatus.CONSTELLATION_GLONASS,
                svid = 1,
                isGlonass = true,
                gloTbGpsTow = tow,
                gloPosX = 15_000_000.0,
                gloPosY = 12_000_000.0,
                gloPosZ = 17_000_000.0,
                lastUpdateTime = receipt,
            )
        val gloBlunder =
            SatelliteMeasurement(
                constellationType = GnssStatus.CONSTELLATION_GLONASS,
                svid = 1,
                pseudorangeMeters = 22_000_000.0 + 50_000.0,
                carrierFrequencyHz = GnssMathUtils.GLONASS_G1_NOMINAL_FREQUENCY_HZ,
                cn0DbHz = 42.0,
                elevationDegrees = 50.0,
                azimuthDegrees = 120.0,
                accumulatedDeltaRangeMeters = null,
                pseudorangeUncertaintyMeters = 0.5,
            )

        val mixedProcessor = GnssMeasurementProcessor()
        mixedProcessor.importExternalEphemeris(gpsEph + gloEph, tow)

        val prunedFix =
            mixedProcessor.computePosition(
                measurements = gpsAll + gloBlunder,
                initialLat = trueLat,
                initialLon = trueLon,
                initialAlt = trueAlt,
                usePpp = false,
                gpsTimeNanos = SYNTHETIC_GPS_TIME_NANOS,
            )
        val nonNullPrunedFix = requireNotNull(prunedFix) { "5 GPS + 1 GLONASS blunder must prune GLONASS and solve cleanly on 5 GPS" }
        assertEquals(5, nonNullPrunedFix.usedInFixCount)
        val errorMeters = ecefDistanceMeters(trueLat, trueLon, trueAlt, nonNullPrunedFix.latitude, nonNullPrunedFix.longitude, nonNullPrunedFix.altitude)
        assertTrue("Pruned 5-GPS fix must be accurate (< 1.0 m, got $errorMeters m)", errorMeters < 1.0)

        // 2 GPS + 1 GAL + 1 BDS + 1 GLO (5 SVs across 4 constellations, mathematically underdetermined)
        val (galEph, galOne) =
            buildSyntheticSkyEpoch(
                constellationType = GnssStatus.CONSTELLATION_GALILEO,
                lat = trueLat,
                lon = trueLon,
                alt = trueAlt,
                gpsTimeNanos = SYNTHETIC_GPS_TIME_NANOS,
                maxSatellites = 1,
            )
        val (bdsEph, bdsOne) =
            buildSyntheticSkyEpoch(
                constellationType = GnssStatus.CONSTELLATION_BEIDOU,
                lat = trueLat,
                lon = trueLon,
                alt = trueAlt,
                gpsTimeNanos = SYNTHETIC_GPS_TIME_NANOS,
                carrierFrequencyHz = GnssMathUtils.B1I_FREQUENCY_HZ,
                maxSatellites = 1,
            )
        val underdeterminedProcessor = GnssMeasurementProcessor()
        underdeterminedProcessor.importExternalEphemeris(gpsEph.take(2) + galEph + bdsEph + gloEph, tow)
        val underdeterminedMeas = gpsAll.take(2) + galOne + bdsOne + gloBlunder

        assertNull(
            "Underdetermined 2 GPS + 1 GAL + 1 BDS + 1 GLO must return null in HIGH (QR) mode",
            underdeterminedProcessor.computePosition(
                measurements = underdeterminedMeas,
                initialLat = trueLat,
                initialLon = trueLon,
                initialAlt = trueAlt,
                usePpp = false,
                gpsTimeNanos = SYNTHETIC_GPS_TIME_NANOS,
                accuracy = GnssPositionSolver.Accuracy.HIGH,
            ),
        )
        assertNull(
            "Underdetermined 2 GPS + 1 GAL + 1 BDS + 1 GLO must return null in MEDIUM (Cholesky) mode",
            underdeterminedProcessor.computePosition(
                measurements = underdeterminedMeas,
                initialLat = trueLat,
                initialLon = trueLon,
                initialAlt = trueAlt,
                usePpp = false,
                gpsTimeNanos = SYNTHETIC_GPS_TIME_NANOS,
                accuracy = GnssPositionSolver.Accuracy.MEDIUM,
            ),
        )
    }
}
