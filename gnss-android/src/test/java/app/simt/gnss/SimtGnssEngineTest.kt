package app.simt.gnss

import android.location.GnssStatus
import android.os.SystemClock
import app.simt.gnss.core.correction.AtmosphericCorrection
import app.simt.gnss.core.correction.SsrCorrections
import app.simt.gnss.core.model.SatelliteEphemeris
import app.simt.gnss.core.model.SatelliteMeasurement
import app.simt.gnss.core.orbit.SatellitePositionCalculator
import app.simt.gnss.core.parser.Rtcm3FrameParser
import app.simt.gnss.core.parser.RtcmMsmParser
import app.simt.gnss.core.utils.GnssMathUtils
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import kotlin.math.pow
import kotlin.math.sqrt

/**
 * Exercises the public [SimtGnssEngine] façade: construction, the neutral diagnostics snapshot,
 * session resets, non-committing probes, and the null/empty-input contracts.
 */
@RunWith(RobolectricTestRunner::class)
class SimtGnssEngineTest {
    @Test
    fun emptyMeasurements_yieldNoSolution() {
        val engine = SimtGnssEngine()
        assertNull(engine.computePosition(emptyList()))
        assertNull(engine.computeRtkPosition(emptyList(), gpsTimeNanos = 1_000_000_000L))
    }

    @Test
    fun diagnostics_snapshotIsPopulatedBeforeAnyFix() {
        val engine = SimtGnssEngine()
        val diagnostics = engine.diagnostics()
        // No ephemerides imported yet, but the summary must still be a coherent string.
        assertNotNull(diagnostics.storedEphemerides)
        assertFalse(diagnostics.hasFix)
        assertNull(diagnostics.lastSolution)
    }

    @Test
    fun storesAreExposedAndEmpty() {
        val engine = SimtGnssEngine()
        assertNotNull(engine.ssrStore)
        assertNotNull(engine.rtkBaseStore)
        assertFalse(engine.rtkBaseStore.hasFreshData())
    }

    @Test
    fun ingestRtcmFrame_acceptsArbitraryBytesWithoutSignal() {
        val engine = SimtGnssEngine()
        // Garbage bytes must not throw; the stream parser simply finds no frame.
        engine.ingestRtcmFrame(byteArrayOf(0x00, 0x01, 0x02, 0x03))
        assertTrue(engine.diagnostics().ssrCorrections.isNotEmpty())
    }

    @Test
    fun smoothingMode_canBeSelected() {
        val engine = SimtGnssEngine()
        engine.setSmoothingMode(SmoothingMode.DOPPLER)
        engine.setSmoothingMode(SmoothingMode.CARRIER_HATCH)
    }

    @Test
    fun resetSession_preserveEphemerisTrue_clearsFiltersAndCorrectionsButRetainsEphemeris() {
        val engine = SimtGnssEngine()
        val lat = 48.8566
        val lon = 2.3522
        val alt = 50.0
        val gpsTimeNanos = 345_600_000_000_000L
        val tow = (gpsTimeNanos / 1e9) % GnssMathUtils.SECONDS_IN_WEEK
        val (ephs, measurements) = buildSyntheticGpsEpoch(lat, lon, alt, gpsTimeNanos)

        assertEquals(ephs.size, engine.importExternalEphemeris(ephs, tow))
        val nowMs = SystemClock.elapsedRealtime()
        engine.ssrStore.putOrbit(
            SsrCorrections.OrbitCorrection(
                constellationType = GnssStatus.CONSTELLATION_GPS,
                svid = ephs.first().svid,
                deltaRadial = 0.0,
                deltaAlongTrack = 0.0,
                deltaCrossTrack = 0.0,
                iod = ephs.first().iode,
                epochTime = tow,
                receivedAtMs = nowMs,
            ),
        )
        engine.ssrStore.putClock(
            SsrCorrections.ClockCorrection(
                constellationType = GnssStatus.CONSTELLATION_GPS,
                svid = ephs.first().svid,
                c0 = 0.0,
                iod = ephs.first().iode,
                epochTime = tow,
                receivedAtMs = nowMs,
            ),
        )
        val (baseX, baseY, baseZ) = GnssMathUtils.latLonAltToEcef(lat, lon, alt)
        engine.rtkBaseStore.setBasePosition(baseX, baseY, baseZ, referenceStationId = 100)
        engine.rtkBaseStore.putObservations(
            measurements.map { m ->
                RtcmMsmParser.BaseObservation(
                    constellationType = m.constellationType,
                    svid = m.svid,
                    carrierFrequencyHz = m.carrierFrequencyHz,
                    pseudorangeMeters = m.pseudorangeMeters,
                    carrierPhaseMeters = null,
                    cnrDbHz = 42.0,
                    epochTimeMs = (tow * 1000.0).toLong(),
                    signalId = 2,
                    referenceStationId = 100,
                )
            },
        )
        // Feed an incomplete RTCM3 frame header before reset to verify the stream buffer resets.
        engine.ingestRtcmFrame(byteArrayOf(0xD3.toByte(), 0x00, 0x20, 0x42))

        val fix = engine.computePosition(measurements, lat, lon, alt, usePpp = true, gpsTimeNanos = gpsTimeNanos)
        assertNotNull(fix)
        assertTrue(engine.diagnostics().hasFix)
        assertTrue(engine.ssrStore.hasValidCorrections)
        assertTrue(engine.rtkBaseStore.hasFreshData())
        assertTrue(engine.hasActivePppOrSsrState(gpsTimeNanos))

        engine.resetSession(preserveEphemeris = true, clearCorrections = true)

        assertFalse(engine.diagnostics().hasFix)
        assertNull(engine.diagnostics().lastSolution)
        assertFalse(engine.ssrStore.hasValidCorrections)
        assertFalse(engine.rtkBaseStore.hasFreshData())
        assertFalse(engine.hasActivePppOrSsrState(gpsTimeNanos))
        assertNotEquals("none", engine.describeStoredEphemerides())

        // Verify RTCM3 stream parser accepts a clean frame immediately after reset.
        val valid1005 = buildRtcm1005Frame(stationId = 7, xMeters = baseX, yMeters = baseY, zMeters = baseZ)
        engine.ingestRtcmFrame(valid1005)
        val decodedBase = requireNotNull(engine.rtkBaseStore.getBasePosition())
        assertEquals(7, decodedBase.referenceStationId)
    }

    @Test
    fun resetSession_preserveEphemerisFalse_clearsEphemerides() {
        val engine = SimtGnssEngine()
        val gpsTimeNanos = 345_600_000_000_000L
        val tow = (gpsTimeNanos / 1e9) % GnssMathUtils.SECONDS_IN_WEEK
        val (ephs, measurements) = buildSyntheticGpsEpoch(48.8566, 2.3522, 50.0, gpsTimeNanos)

        assertEquals(ephs.size, engine.importExternalEphemeris(ephs, tow))
        assertNotNull(engine.computePosition(measurements, 48.8566, 2.3522, 50.0, usePpp = false, gpsTimeNanos = gpsTimeNanos))
        assertNotEquals("none", engine.describeStoredEphemerides())

        engine.resetSession(preserveEphemeris = false)

        assertEquals("none", engine.describeStoredEphemerides())
        assertFalse(engine.diagnostics().hasFix)
        assertFalse(engine.hasActivePppOrSsrState(gpsTimeNanos))
    }

    @Test
    fun computePosition_commitStateFalse_solvesWithoutMutatingPppOrKalmanState() {
        val engine = SimtGnssEngine()
        val latA = 48.8566
        val lonA = 2.3522
        val altA = 50.0
        val t1Nanos = 345_600_000_000_000L
        val tow = (t1Nanos / 1e9) % GnssMathUtils.SECONDS_IN_WEEK
        val (ephs, measA) = buildSyntheticGpsEpoch(latA, lonA, altA, t1Nanos)
        engine.importExternalEphemeris(ephs, tow)

        // Probe before any committed fix: must return a fix without populating diagnostics or PPP seed.
        val probe0 =
            engine.computePosition(
                measurements = measA,
                initialLat = latA,
                initialLon = lonA,
                initialAlt = altA,
                usePpp = false,
                gpsTimeNanos = t1Nanos,
                commitState = false,
            )
        assertNotNull(probe0)
        assertFalse(engine.diagnostics().hasFix)
        assertFalse(engine.hasActivePppOrSsrState(t1Nanos))

        // Commit a real fix at Location A.
        val committedA =
            engine.computePosition(
                measurements = measA,
                initialLat = latA,
                initialLon = lonA,
                initialAlt = altA,
                usePpp = false,
                gpsTimeNanos = t1Nanos,
                commitState = true,
            )
        assertNotNull(committedA)
        assertTrue(engine.diagnostics().hasFix)

        // Probe a candidate 200 m away at t2 = t1 + 1s with commitState = false: must return Location B
        // cleanly (unfiltered by A's Kalman state) and leave Location A's diagnostics untouched.
        val latB = 48.8584
        val lonB = 2.3540
        val t2Nanos = t1Nanos + GnssMathUtils.NANOS_PER_SECOND
        val measB = buildSyntheticGpsMeasurements(ephs, latB, lonB, altA, t2Nanos)
        val probeB =
            requireNotNull(
                engine.computePosition(
                    measurements = measB,
                    initialLat = latB,
                    initialLon = lonB,
                    initialAlt = altA,
                    usePpp = false,
                    gpsTimeNanos = t2Nanos,
                    commitState = false,
                ),
            )
        val committedLast = requireNotNull(engine.diagnostics().lastSolution)
        assertEquals(latB, probeB.latitude, 1e-4)
        assertEquals(lonB, probeB.longitude, 1e-4)
        assertEquals(latA, committedLast.latitude, 1e-4)
    }

    private fun buildSyntheticGpsMeasurements(
        ephemerides: List<SatelliteEphemeris>,
        lat: Double,
        lon: Double,
        alt: Double,
        gpsTimeNanos: Long,
    ): List<SatelliteMeasurement> {
        val tow = (gpsTimeNanos / 1e9) % GnssMathUtils.SECONDS_IN_WEEK
        val (recX, recY, recZ) = GnssMathUtils.latLonAltToEcef(lat, lon, alt)
        val unixSeconds = GnssMathUtils.GPS_EPOCH_UNIX_SECONDS + (gpsTimeNanos / 1_000_000_000L)
        val utcCalendar = java.util.Calendar.getInstance(java.util.TimeZone.getTimeZone("UTC"))
        utcCalendar.timeInMillis = unixSeconds * 1000L
        val dayOfYear = utcCalendar.get(java.util.Calendar.DAY_OF_YEAR)
        val measurements = ArrayList<SatelliteMeasurement>(ephemerides.size)
        for (eph in ephemerides) {
            val rough = SatellitePositionCalculator.calculateSatellitePosition(eph, tow)
            val roughRange =
                sqrt(
                    (rough.first - recX).pow(2) +
                        (rough.second - recY).pow(2) +
                        (rough.third - recZ).pow(2),
                )
            val flight = roughRange / GnssMathUtils.SPEED_OF_LIGHT
            val tx = SatellitePositionCalculator.calculateSatellitePosition(eph, tow - flight)
            val rotated =
                SatellitePositionCalculator.rotateEcefForEarthRotationDuringTransit(
                    tx.first,
                    tx.second,
                    tx.third,
                    flight,
                )
            val (elev, az) =
                GnssMathUtils.ecefLosToElevationAzimuth(
                    receiverX = recX,
                    receiverY = recY,
                    receiverZ = recZ,
                    receiverLatDeg = lat,
                    receiverLonDeg = lon,
                    satelliteX = rotated.first,
                    satelliteY = rotated.second,
                    satelliteZ = rotated.third,
                )
            if (elev < 20.0) continue
            val range =
                sqrt(
                    (rotated.first - recX).pow(2) +
                        (rotated.second - recY).pow(2) +
                        (rotated.third - recZ).pow(2),
                )
            val dry = AtmosphericCorrection.computeTroposphericDelay(elev, alt, lat, dayOfYear)
            val iono = AtmosphericCorrection.computeIonosphericDelay(elev, az, lat, lon, gpsTimeNanos / 1_000_000L, null, null)
            val rel = AtmosphericCorrection.computeRelativisticCorrection(recX, recY, recZ, rotated.first, rotated.second, rotated.third)
            measurements +=
                SatelliteMeasurement(
                    constellationType = GnssStatus.CONSTELLATION_GPS,
                    svid = eph.svid,
                    pseudorangeMeters = range + dry + iono + rel,
                    carrierFrequencyHz = GnssMathUtils.L1_FREQUENCY_HZ,
                    cn0DbHz = 42.0,
                    elevationDegrees = elev,
                    azimuthDegrees = az,
                    accumulatedDeltaRangeMeters = null,
                    pseudorangeUncertaintyMeters = 0.5,
                )
        }
        return measurements
    }

    private fun buildSyntheticGpsEpoch(
        lat: Double,
        lon: Double,
        alt: Double,
        gpsTimeNanos: Long,
    ): Pair<List<SatelliteEphemeris>, List<SatelliteMeasurement>> {
        val tow = (gpsTimeNanos / 1e9) % GnssMathUtils.SECONDS_IN_WEEK
        val (recX, recY, recZ) = GnssMathUtils.latLonAltToEcef(lat, lon, alt)
        val receipt = SystemClock.elapsedRealtime()
        val unixSeconds = GnssMathUtils.GPS_EPOCH_UNIX_SECONDS + (gpsTimeNanos / 1_000_000_000L)
        val utcCalendar = java.util.Calendar.getInstance(java.util.TimeZone.getTimeZone("UTC"))
        utcCalendar.timeInMillis = unixSeconds * 1000L
        val dayOfYear = utcCalendar.get(java.util.Calendar.DAY_OF_YEAR)

        val ephemerides = ArrayList<SatelliteEphemeris>()
        val measurements = ArrayList<SatelliteMeasurement>()
        var svid = 1
        for (raanDeg in 0 until 360 step 30) {
            for (meanDeg in listOf(0.0, 90.0, 180.0, 270.0)) {
                val eph =
                    SatelliteEphemeris(
                        constellationType = GnssStatus.CONSTELLATION_GPS,
                        svid = svid,
                        iode = 12,
                        toe = tow,
                        toc = tow,
                        sqrtA = sqrt(26_560_000.0),
                        eccentricity = 0.0,
                        i0 = Math.toRadians(55.0),
                        omega0 = Math.toRadians(raanDeg.toDouble()),
                        omega = 0.0,
                        m0 = Math.toRadians(meanDeg),
                        lastUpdateTime = receipt,
                    )
                val rough = SatellitePositionCalculator.calculateSatellitePosition(eph, tow)
                val roughRange =
                    sqrt(
                        (rough.first - recX).pow(2) +
                            (rough.second - recY).pow(2) +
                            (rough.third - recZ).pow(2),
                    )
                val flight = roughRange / GnssMathUtils.SPEED_OF_LIGHT
                val tx = SatellitePositionCalculator.calculateSatellitePosition(eph, tow - flight)
                val rotated =
                    SatellitePositionCalculator.rotateEcefForEarthRotationDuringTransit(
                        tx.first,
                        tx.second,
                        tx.third,
                        flight,
                    )
                val (elev, az) =
                    GnssMathUtils.ecefLosToElevationAzimuth(
                        receiverX = recX,
                        receiverY = recY,
                        receiverZ = recZ,
                        receiverLatDeg = lat,
                        receiverLonDeg = lon,
                        satelliteX = rotated.first,
                        satelliteY = rotated.second,
                        satelliteZ = rotated.third,
                    )
                if (elev >= 20.0) {
                    val range =
                        sqrt(
                            (rotated.first - recX).pow(2) +
                                (rotated.second - recY).pow(2) +
                                (rotated.third - recZ).pow(2),
                        )
                    val dry = AtmosphericCorrection.computeTroposphericDelay(elev, alt, lat, dayOfYear)
                    val iono = AtmosphericCorrection.computeIonosphericDelay(elev, az, lat, lon, gpsTimeNanos / 1_000_000L, null, null)
                    val rel = AtmosphericCorrection.computeRelativisticCorrection(recX, recY, recZ, rotated.first, rotated.second, rotated.third)
                    ephemerides += eph
                    measurements +=
                        SatelliteMeasurement(
                            constellationType = GnssStatus.CONSTELLATION_GPS,
                            svid = svid,
                            pseudorangeMeters = range + dry + iono + rel,
                            carrierFrequencyHz = GnssMathUtils.L1_FREQUENCY_HZ,
                            cn0DbHz = 42.0,
                            elevationDegrees = elev,
                            azimuthDegrees = az,
                            accumulatedDeltaRangeMeters = null,
                            pseudorangeUncertaintyMeters = 0.5,
                        )
                }
                svid++
            }
        }
        return ephemerides to measurements
    }

    private fun buildRtcm1005Frame(
        stationId: Int,
        xMeters: Double,
        yMeters: Double,
        zMeters: Double,
    ): ByteArray {
        val bits = ArrayList<Int>()

        fun writeBits(
            value: Long,
            count: Int,
        ) {
            for (i in count - 1 downTo 0) {
                bits.add(((value shr i) and 1L).toInt())
            }
        }
        writeBits(1005L, 12)
        writeBits(stationId.toLong(), 12)
        writeBits(0L, 6) // ITRF year
        writeBits(1L, 1) // GPS indicator
        writeBits(0L, 1) // GLONASS indicator
        writeBits(0L, 1) // Galileo indicator
        writeBits(0L, 1) // ref station indicator
        writeBits((xMeters / 0.0001).toLong(), 38)
        writeBits(0L, 1)
        writeBits(0L, 1)
        writeBits((yMeters / 0.0001).toLong(), 38)
        writeBits(0L, 2)
        writeBits((zMeters / 0.0001).toLong(), 38)
        val payloadLen = (bits.size + 7) / 8
        val frame = ByteArray(3 + payloadLen + 3)
        frame[0] = 0xD3.toByte()
        frame[1] = ((payloadLen shr 8) and 0x03).toByte()
        frame[2] = (payloadLen and 0xFF).toByte()
        bits.forEachIndexed { idx, bit ->
            if (bit != 0) {
                frame[3 + idx / 8] = (frame[3 + idx / 8].toInt() or (1 shl (7 - idx % 8))).toByte()
            }
        }
        val crc = Rtcm3FrameParser.crc24q(frame, 0, 3 + payloadLen)
        frame[3 + payloadLen] = ((crc shr 16) and 0xFF).toByte()
        frame[3 + payloadLen + 1] = ((crc shr 8) and 0xFF).toByte()
        frame[3 + payloadLen + 2] = (crc and 0xFF).toByte()
        return frame
    }
}
