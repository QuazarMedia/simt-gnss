package app.simt.gnss

import android.location.GnssStatus
import app.simt.gnss.core.correction.AtmosphericCorrection
import app.simt.gnss.core.model.GnssSignalBand
import app.simt.gnss.core.model.SatelliteMeasurement
import app.simt.gnss.core.orbit.SatellitePositionCalculator
import app.simt.gnss.core.utils.GnssMathUtils
import org.junit.Assert.*
import org.junit.Before
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import kotlin.math.pow
import kotlin.math.sqrt

@RunWith(RobolectricTestRunner::class)
class GnssMultiBandTest {
    private lateinit var processor: GnssMeasurementProcessor

    @Before
    fun setUp() {
        processor = GnssMeasurementProcessor()
    }

    @Test
    fun signalBandClassification_mapsAllBandsAccurately() {
        assertEquals(GnssSignalBand.L1_E1_B1C, GnssSignalBand.fromFrequencyHz(GnssMathUtils.L1_FREQUENCY_HZ, GnssStatus.CONSTELLATION_GPS))
        assertEquals(GnssSignalBand.L1_E1_B1C, GnssSignalBand.fromFrequencyHz(GnssMathUtils.GLONASS_G1_NOMINAL_FREQUENCY_HZ, GnssStatus.CONSTELLATION_GLONASS))
        assertEquals(GnssSignalBand.B1I, GnssSignalBand.fromFrequencyHz(GnssMathUtils.B1I_FREQUENCY_HZ, GnssStatus.CONSTELLATION_BEIDOU))
        assertEquals(GnssSignalBand.B3I, GnssSignalBand.fromFrequencyHz(GnssMathUtils.B3I_FREQUENCY_HZ, GnssStatus.CONSTELLATION_BEIDOU))
        assertEquals(GnssSignalBand.L2, GnssSignalBand.fromFrequencyHz(1_227_600_000.0, GnssStatus.CONSTELLATION_GPS))
        assertEquals(GnssSignalBand.E5B_B2I, GnssSignalBand.fromFrequencyHz(GnssMathUtils.E5B_FREQUENCY_HZ, GnssStatus.CONSTELLATION_GALILEO))
        assertEquals(GnssSignalBand.L5_E5A_B2A, GnssSignalBand.fromFrequencyHz(GnssMathUtils.L5_FREQUENCY_HZ, GnssStatus.CONSTELLATION_GALILEO))
        assertEquals(GnssSignalBand.UNKNOWN, GnssSignalBand.fromFrequencyHz(0.0, GnssStatus.CONSTELLATION_GPS))
        assertEquals(GnssSignalBand.UNKNOWN, GnssSignalBand.fromFrequencyHz(Double.NaN, GnssStatus.CONSTELLATION_GPS))
    }

    @Test
    fun dualFrequencyIf_galileoE1E5bAndIfSigmaScaling_workCleanly() {
        val trueLat = 48.8566
        val trueLon = 2.3522
        val trueAlt = 50.0
        val (ephemerides, e1Meas) =
            buildSyntheticSkyEpoch(
                constellationType = GnssStatus.CONSTELLATION_GALILEO,
                lat = trueLat,
                lon = trueLon,
                alt = trueAlt,
                gpsTimeNanos = SYNTHETIC_GPS_TIME_NANOS,
                carrierFrequencyHz = GnssMathUtils.L1_FREQUENCY_HZ,
                applyAtmosphericAndRelativistic = false,
            )
        assertTrue(e1Meas.size >= 6)

        val toe = (SYNTHETIC_GPS_TIME_NANOS / 1e9) % GnssMathUtils.SECONDS_IN_WEEK
        val (recX, recY, recZ) = GnssMathUtils.latLonAltToEcef(trueLat, trueLon, trueAlt)
        val unixSeconds = GnssMathUtils.GPS_EPOCH_UNIX_SECONDS + (SYNTHETIC_GPS_TIME_NANOS / 1_000_000_000L)
        val utcCalendar = java.util.Calendar.getInstance(java.util.TimeZone.getTimeZone("UTC"))
        utcCalendar.timeInMillis = unixSeconds * 1000L
        val dayOfYear = utcCalendar.get(java.util.Calendar.DAY_OF_YEAR)

        val bgdA = 1.2e-9
        val bgdB = 2.5e-9
        val c = GnssMathUtils.SPEED_OF_LIGHT
        val inavEph =
            ephemerides.map {
                it.copy(
                    tgd = bgdB,
                    galileoBgdE1E5aSeconds = bgdA,
                    galileoBgdE1E5bSeconds = bgdB,
                    galileoInavClock = true,
                )
            }

        // Build E1 and E5b with realistic I/NAV BGD + troposphere + 1/f^2 ionosphere
        val dualMeas = mutableListOf<SatelliteMeasurement>()
        e1Meas.forEachIndexed { idx, base ->
            val eph = inavEph[idx]
            val dryDelay = AtmosphericCorrection.computeTroposphericDelay(base.elevationDegrees, trueAlt, trueLat, dayOfYear)
            val ionoL1 = 8.0
            val ionoE5b = ionoL1 * (GnssMathUtils.L1_FREQUENCY_HZ / GnssMathUtils.E5B_FREQUENCY_HZ).pow(2)
            val tx = SatellitePositionCalculator.calculateSatellitePosition(eph, toe)
            val relCorr = AtmosphericCorrection.computeRelativisticCorrection(recX, recY, recZ, tx.first, tx.second, tx.third)
            // Add +3.5 m noise on E5b of SV 0 (with IF sigma scaling, z_i stays well below 4 sigma;
            // without IF sigma scaling, ~2.31 * 3.5 m = 8.1 m residual against ~0.77 m sigma would exceed 10 sigma and trigger false RAIM!)
            val e5bNoise = if (idx == 0) 3.5 else 0.0
            val pE1 = base.pseudorangeMeters + dryDelay + ionoL1 + relCorr + c * bgdB
            val pE5b = base.pseudorangeMeters + dryDelay + ionoE5b + relCorr + c * (GnssMathUtils.L1_FREQUENCY_HZ / GnssMathUtils.E5B_FREQUENCY_HZ).pow(2) * bgdB + e5bNoise
            dualMeas += base.copy(pseudorangeMeters = pE1, carrierFrequencyHz = GnssMathUtils.L1_FREQUENCY_HZ, signalBand = GnssSignalBand.L1_E1_B1C)
            dualMeas +=
                base.copy(
                    pseudorangeMeters = pE5b,
                    carrierFrequencyHz = GnssMathUtils.E5B_FREQUENCY_HZ,
                    pseudorangeUncertaintyMeters = 2.0,
                    signalBand = GnssSignalBand.E5B_B2I,
                )
        }

        val galProcessor = GnssMeasurementProcessor()
        galProcessor.importExternalEphemeris(inavEph, toe)
        val fix =
            galProcessor.computePosition(
                measurements = dualMeas,
                initialLat = trueLat,
                initialLon = trueLon,
                initialAlt = trueAlt,
                usePpp = false,
                gpsTimeNanos = SYNTHETIC_GPS_TIME_NANOS,
            )
        val nonNullFix = requireNotNull(fix)
        // With proper IF sigma propagation (sigmaLow >= 2.0 -> sigmaIf >= 3.2 m), SV 0 is NOT falsely excluded by RAIM!
        assertEquals("SV 0 with moderate E5b noise must not be falsely excluded when IF sigma is scaled", e1Meas.size, nonNullFix.usedInFixCount)
    }

    @Test
    fun mixedBeidouB1iAndB1cB2a_splitsClockStatesAndEliminatesInterSignalBias() {
        val trueLat = 48.8566
        val trueLon = 2.3522
        val trueAlt = 50.0
        val (bdsEph, bdsBaseMeas) =
            buildSyntheticSkyEpoch(
                constellationType = GnssStatus.CONSTELLATION_BEIDOU,
                lat = trueLat,
                lon = trueLon,
                alt = trueAlt,
                gpsTimeNanos = SYNTHETIC_GPS_TIME_NANOS,
                carrierFrequencyHz = GnssMathUtils.B1I_FREQUENCY_HZ,
                clockBiasMeters = 10_000.0,
                applyAtmosphericAndRelativistic = false,
            )
        assertTrue("Need >= 8 BeiDou SVs for split-clock test, got ${bdsBaseMeas.size}", bdsBaseMeas.size >= 8)

        val toe = (SYNTHETIC_GPS_TIME_NANOS / 1e9) % GnssMathUtils.SECONDS_IN_WEEK - GnssMathUtils.BDT_GPST_OFFSET_SECONDS
        val (recX, recY, recZ) = GnssMathUtils.latLonAltToEcef(trueLat, trueLon, trueAlt)
        val unixSeconds = GnssMathUtils.GPS_EPOCH_UNIX_SECONDS + (SYNTHETIC_GPS_TIME_NANOS / 1_000_000_000L)
        val utcCalendar = java.util.Calendar.getInstance(java.util.TimeZone.getTimeZone("UTC"))
        utcCalendar.timeInMillis = unixSeconds * 1000L
        val dayOfYear = utcCalendar.get(java.util.Calendar.DAY_OF_YEAR)
        val c = GnssMathUtils.SPEED_OF_LIGHT
        val tgd1 = 1.5e-8
        val unmodeledBds3OffsetMeters = 18.0

        val splitIdx = bdsBaseMeas.size / 2
        val ephWithTgd = bdsEph.map { it.copy(tgd = tgd1) }
        val mixedMeas = mutableListOf<SatelliteMeasurement>()
        bdsBaseMeas.forEachIndexed { idx, base ->
            val eph = ephWithTgd[idx]
            val dryDelay = AtmosphericCorrection.computeTroposphericDelay(base.elevationDegrees, trueAlt, trueLat, dayOfYear)
            val rough = SatellitePositionCalculator.calculateSatellitePosition(eph, toe)
            val roughRange = sqrt((rough.first - recX).pow(2) + (rough.second - recY).pow(2) + (rough.third - recZ).pow(2))
            val flight = roughRange / c
            val transmit = SatellitePositionCalculator.calculateSatellitePosition(eph, toe - flight)
            val rotated = SatellitePositionCalculator.rotateEcefForEarthRotationDuringTransit(transmit.first, transmit.second, transmit.third, flight)
            val relCorr = AtmosphericCorrection.computeRelativisticCorrection(recX, recY, recZ, rotated.first, rotated.second, rotated.third)
            if (idx < splitIdx) {
                // B3I-calibrated single-frequency B1I (includes c * TGD1 + Klobuchar iono)
                val ionoB1i = AtmosphericCorrection.computeIonosphericDelay(base.elevationDegrees, base.azimuthDegrees, trueLat, trueLon, SYNTHETIC_GPS_TIME_NANOS / 1_000_000L, null, null) * (GnssMathUtils.L1_FREQUENCY_HZ / GnssMathUtils.B1I_FREQUENCY_HZ).pow(2)
                mixedMeas +=
                    base.copy(
                        pseudorangeMeters = base.pseudorangeMeters + dryDelay + ionoB1i + relCorr + c * tgd1,
                        carrierFrequencyHz = GnssMathUtils.B1I_FREQUENCY_HZ,
                        signalBand = GnssSignalBand.B1I,
                    )
            } else {
                // Uncalibrated BDS-3 B1C + B2a IF pair with an 18 m systematic inter-signal bias
                val ionoL1 = AtmosphericCorrection.computeIonosphericDelay(base.elevationDegrees, base.azimuthDegrees, trueLat, trueLon, SYNTHETIC_GPS_TIME_NANOS / 1_000_000L, null, null)
                val ionoB2a = ionoL1 * (GnssMathUtils.L1_FREQUENCY_HZ / GnssMathUtils.L5_FREQUENCY_HZ).pow(2)
                mixedMeas +=
                    base.copy(
                        pseudorangeMeters = base.pseudorangeMeters + dryDelay + ionoL1 + relCorr + unmodeledBds3OffsetMeters,
                        carrierFrequencyHz = GnssMathUtils.L1_FREQUENCY_HZ,
                        signalBand = GnssSignalBand.L1_E1_B1C,
                    )
                mixedMeas +=
                    base.copy(
                        pseudorangeMeters = base.pseudorangeMeters + dryDelay + ionoB2a + relCorr + unmodeledBds3OffsetMeters,
                        carrierFrequencyHz = GnssMathUtils.L5_FREQUENCY_HZ,
                        isL5 = true,
                        signalBand = GnssSignalBand.L5_E5A_B2A,
                    )
            }
        }

        val bdsProcessor = GnssMeasurementProcessor()
        bdsProcessor.importExternalEphemeris(ephWithTgd, (SYNTHETIC_GPS_TIME_NANOS / 1e9) % GnssMathUtils.SECONDS_IN_WEEK)
        val fix =
            bdsProcessor.computePosition(
                measurements = mixedMeas,
                initialLat = trueLat,
                initialLon = trueLon,
                initialAlt = trueAlt,
                usePpp = false,
                gpsTimeNanos = SYNTHETIC_GPS_TIME_NANOS,
            )
        val nonNullFix = requireNotNull(fix) { "Mixed BeiDou B1I + B1C/B2a epoch must solve cleanly" }
        val errorMeters = ecefDistanceMeters(trueLat, trueLon, trueAlt, nonNullFix.latitude, nonNullFix.longitude, nonNullFix.altitude)
        assertTrue("Separate BDS-3 clock state must absorb 18 m offset (error=$errorMeters m)", errorMeters < 1.0)
        assertEquals(bdsBaseMeas.size, nonNullFix.usedInFixCount)
    }
}
