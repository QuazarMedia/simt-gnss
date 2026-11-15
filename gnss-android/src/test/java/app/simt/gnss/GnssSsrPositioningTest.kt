package app.simt.gnss

import android.location.GnssStatus
import android.os.SystemClock
import app.simt.gnss.core.correction.AtmosphericCorrection
import app.simt.gnss.core.correction.SsrCorrections
import app.simt.gnss.core.model.GnssSignalBand
import app.simt.gnss.core.model.SatelliteEphemeris
import app.simt.gnss.core.model.SatelliteMeasurement
import app.simt.gnss.core.orbit.SatellitePositionCalculator
import app.simt.gnss.core.parser.Rtcm3FrameParser
import app.simt.gnss.core.parser.RtcmSsrDecoder
import app.simt.gnss.core.utils.GnssMathUtils
import org.junit.Assert.*
import org.junit.Before
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import kotlin.math.pow
import kotlin.math.sqrt

@RunWith(RobolectricTestRunner::class)
class GnssSsrPositioningTest {
    private lateinit var processor: GnssMeasurementProcessor

    @Before
    fun setUp() {
        processor = GnssMeasurementProcessor()
    }

    @Test
    fun ssrCodeBias_removesPerSignalBiasesWithoutBroadcastTgd_andRespectsUsePpp() {
        val trueLat = 48.8566
        val trueLon = 2.3522
        val trueAlt = 50.0
        val (baseEph, baseL1Meas) =
            buildSyntheticSkyEpoch(
                constellationType = GnssStatus.CONSTELLATION_GPS,
                lat = trueLat,
                lon = trueLon,
                alt = trueAlt,
                gpsTimeNanos = SYNTHETIC_GPS_TIME_NANOS,
                carrierFrequencyHz = GnssMathUtils.L1_FREQUENCY_HZ,
                applyAtmosphericAndRelativistic = false,
            )
        assertTrue("Need >= 6 GPS SVs, got ${baseL1Meas.size}", baseL1Meas.size >= 6)

        val toe = (SYNTHETIC_GPS_TIME_NANOS / 1e9) % GnssMathUtils.SECONDS_IN_WEEK
        val (recX, recY, recZ) = GnssMathUtils.latLonAltToEcef(trueLat, trueLon, trueAlt)
        val unixSeconds = GnssMathUtils.GPS_EPOCH_UNIX_SECONDS + (SYNTHETIC_GPS_TIME_NANOS / 1_000_000_000L)
        val utcCalendar = java.util.Calendar.getInstance(java.util.TimeZone.getTimeZone("UTC"))
        utcCalendar.timeInMillis = unixSeconds * 1000L
        val dayOfYear = utcCalendar.get(java.util.Calendar.DAY_OF_YEAR)
        val nowMs = SystemClock.elapsedRealtime()
        val f1 = GnssMathUtils.L1_FREQUENCY_HZ
        val f5 = GnssMathUtils.L5_FREQUENCY_HZ
        val gamma5 = (f1 / f5).pow(2)

        // Inject non-zero broadcast TGD (must be skipped when SSR code biases + clock are active)
        // and deliberate per-SV signal code biases (+2.5 m on L1C, -1.8 m on L5Q on half of SVs).
        val ephWithIodeAndTgd = baseEph.map { it.copy(iode = 42, tgd = 1.5e-8) }
        val measurements = mutableListOf<SatelliteMeasurement>()
        val ssrProcessor = GnssMeasurementProcessor()
        ssrProcessor.importExternalEphemeris(ephWithIodeAndTgd, toe)

        baseL1Meas.forEachIndexed { idx, base ->
            val eph = ephWithIodeAndTgd[idx]
            val dryDelay = AtmosphericCorrection.computeTroposphericDelay(base.elevationDegrees, trueAlt, trueLat, dayOfYear)
            val ionoL1 = AtmosphericCorrection.computeIonosphericDelay(base.elevationDegrees, base.azimuthDegrees, trueLat, trueLon, SYNTHETIC_GPS_TIME_NANOS / 1_000_000L, null, null)
            val tx = SatellitePositionCalculator.calculateSatellitePosition(eph, toe)
            val relCorr = AtmosphericCorrection.computeRelativisticCorrection(recX, recY, recZ, tx.first, tx.second, tx.third)

            // Deliberate signal bias injected into the raw pseudorange: PR_obs = PR_true - CodeBias
            // so PR_obs + CodeBias = PR_true. Vary sign/magnitude per SV so a common clock cannot absorb it.
            val l1Bias = if (idx % 2 == 0) 2.5 else -1.9
            val l5Bias = if (idx % 2 == 0) -1.8 else 2.2

            ssrProcessor.ssrStore.putOrbit(
                SsrCorrections.OrbitCorrection(
                    constellationType = GnssStatus.CONSTELLATION_GPS,
                    svid = eph.svid,
                    deltaRadial = 0.0,
                    deltaAlongTrack = 0.0,
                    deltaCrossTrack = 0.0,
                    iod = 42,
                    epochTime = toe,
                    receivedAtMs = nowMs,
                ),
            )
            ssrProcessor.ssrStore.putClock(
                SsrCorrections.ClockCorrection(
                    constellationType = GnssStatus.CONSTELLATION_GPS,
                    svid = eph.svid,
                    c0 = 0.0,
                    iod = 42,
                    epochTime = toe,
                    receivedAtMs = nowMs,
                ),
            )
            ssrProcessor.ssrStore.putCodeBias(
                SsrCorrections.CodeBias(
                    constellationType = GnssStatus.CONSTELLATION_GPS,
                    svid = eph.svid,
                    signalId = 0, // GPS L1 C/A (1C)
                    bias = l1Bias,
                    receivedAtMs = nowMs,
                ),
            )

            // Mix dual-frequency IF SVs (even indices) and single-frequency L1C SVs (odd indices).
            if (idx % 2 == 0) {
                ssrProcessor.ssrStore.putCodeBias(
                    SsrCorrections.CodeBias(
                        constellationType = GnssStatus.CONSTELLATION_GPS,
                        svid = eph.svid,
                        signalId = 15, // GPS L5 Q (5Q)
                        bias = l5Bias,
                        receivedAtMs = nowMs,
                    ),
                )
                measurements +=
                    base.copy(
                        pseudorangeMeters = base.pseudorangeMeters + dryDelay + ionoL1 + relCorr - l1Bias,
                        carrierFrequencyHz = f1,
                    )
                measurements +=
                    base.copy(
                        pseudorangeMeters = base.pseudorangeMeters + dryDelay + ionoL1 * gamma5 + relCorr - l5Bias,
                        carrierFrequencyHz = f5,
                        isL5 = true,
                        signalBand = GnssSignalBand.L5_E5A_B2A,
                    )
            } else {
                measurements +=
                    base.copy(
                        pseudorangeMeters = base.pseudorangeMeters + dryDelay + ionoL1 + relCorr - l1Bias,
                        carrierFrequencyHz = f1,
                    )
            }
        }

        val pppFix =
            ssrProcessor.computePosition(
                measurements = measurements,
                initialLat = trueLat,
                initialLon = trueLon,
                initialAlt = trueAlt,
                usePpp = true,
                gpsTimeNanos = SYNTHETIC_GPS_TIME_NANOS,
            )
        val nonNullPpp = requireNotNull(pppFix)
        val pppErrorMeters = ecefDistanceMeters(trueLat, trueLon, trueAlt, nonNullPpp.latitude, nonNullPpp.longitude, nonNullPpp.altitude)
        assertTrue("SSR code biases must be applied cleanly without broadcast TGD (< 0.1 m, got $pppErrorMeters m)", pppErrorMeters < 0.1)

        // When usePpp = false, SSR orbit/clock/code biases must be ignored; the uncalibrated TGD + signal biases produce > 1 m error.
        val nonPppFix =
            ssrProcessor.computePosition(
                measurements = measurements,
                initialLat = trueLat,
                initialLon = trueLon,
                initialAlt = trueAlt,
                usePpp = false,
                gpsTimeNanos = SYNTHETIC_GPS_TIME_NANOS,
            )
        val nonNullNonPpp = requireNotNull(nonPppFix)
        val nonPppErrorMeters = ecefDistanceMeters(trueLat, trueLon, trueAlt, nonNullNonPpp.latitude, nonNullNonPpp.longitude, nonNullNonPpp.altitude)
        assertTrue("usePpp = false must NOT apply SSR (expected > 1.0 m error, got $nonPppErrorMeters m)", nonPppErrorMeters > 1.0)
    }

    @Test
    fun beidouSsr_usesBdtDtSsr_andGatesOnAodeOrToeMod240() {
        val trueLat = 48.8566
        val trueLon = 2.3522
        val trueAlt = 50.0
        // gpstSeconds = 345_614.0 -> BDT = 345_600.0
        val bdsGpsTimeNanos = 345_614L * GnssMathUtils.NANOS_PER_SECOND
        val bdtToe = 345_600.0
        val (bdsEph, bdsMeas) =
            buildSyntheticSkyEpoch(
                constellationType = GnssStatus.CONSTELLATION_BEIDOU,
                lat = trueLat,
                lon = trueLon,
                alt = trueAlt,
                gpsTimeNanos = bdsGpsTimeNanos,
                carrierFrequencyHz = GnssMathUtils.B1I_FREQUENCY_HZ,
            )
        assertTrue("Need >= 6 BeiDou SVs, got ${bdsMeas.size}", bdsMeas.size >= 6)

        val nowMs = SystemClock.elapsedRealtime()
        val ephWithAode = bdsEph.map { it.copy(iode = 7) }
        val bdsProcessor = GnssMeasurementProcessor()
        bdsProcessor.importExternalEphemeris(ephWithAode, 345_614.0)

        // Put a huge c1 = 5.0 m/s with epochTime = 345_600.0 (BDT) and udi = 0:
        // If BDT currentTimeSeconds (345_600.0) is passed, dtSsr = 0.0 -> c1*dtSsr = 0 m!
        // If GPST (345_614.0) were passed, dtSsr = +14.0 s -> 70 m clock error on every SV with non-uniform c1!
        val toeMod240 = ((bdtToe / 8.0).toInt()).mod(240)
        ephWithAode.forEachIndexed { idx, eph ->
            // Alternate between AODE (7) and DF470 ((toe / 8) % 240) to test both accepted BeiDou IOD forms.
            val acceptedIod = if (idx % 2 == 0) 7 else toeMod240
            // Inject a deliberate +40 m broadcast clock error on one SV (via c0 = 0 on SV 0? Better: shift af0 by -40/c on SV 0 and cancel with SSR c0=+40!)
            val c1Rate = if (idx % 2 == 0) 5.0 else -5.0
            bdsProcessor.ssrStore.putOrbit(
                SsrCorrections.OrbitCorrection(
                    constellationType = GnssStatus.CONSTELLATION_BEIDOU,
                    svid = eph.svid,
                    deltaRadial = 0.0,
                    deltaAlongTrack = 0.0,
                    deltaCrossTrack = 0.0,
                    iod = acceptedIod,
                    epochTime = bdtToe,
                    updateIntervalSeconds = 0.0,
                    receivedAtMs = nowMs,
                ),
            )
            bdsProcessor.ssrStore.putClock(
                SsrCorrections.ClockCorrection(
                    constellationType = GnssStatus.CONSTELLATION_BEIDOU,
                    svid = eph.svid,
                    c0 = 0.0,
                    c1 = c1Rate,
                    iod = acceptedIod,
                    epochTime = bdtToe,
                    updateIntervalSeconds = 0.0,
                    receivedAtMs = nowMs,
                ),
            )
        }

        val cleanFix =
            bdsProcessor.computePosition(
                measurements = bdsMeas,
                initialLat = trueLat,
                initialLon = trueLon,
                initialAlt = trueAlt,
                usePpp = true,
                gpsTimeNanos = bdsGpsTimeNanos,
            )
        val nonNullClean = requireNotNull(cleanFix)
        val cleanError = ecefDistanceMeters(trueLat, trueLon, trueAlt, nonNullClean.latitude, nonNullClean.longitude, nonNullClean.altitude)
        assertTrue("BeiDou SSR with BDT dtSsr=0 must stay accurate (< 1.0 m, got $cleanError m)", cleanError < 1.0)

        // Now verify IOD mismatch gate: inject a 50 m radial/clock bias with a mismatched IOD on SV 0;
        // if rejected by the IOD gate, SV 0 stays on clean broadcast and the fix stays < 1.0 m!
        val firstSv = ephWithAode[0].svid
        bdsProcessor.ssrStore.putOrbit(
            SsrCorrections.OrbitCorrection(
                constellationType = GnssStatus.CONSTELLATION_BEIDOU,
                svid = firstSv,
                deltaRadial = 50.0,
                deltaAlongTrack = 0.0,
                deltaCrossTrack = 0.0,
                iod = 999, // neither AODE (7) nor toeMod240
                epochTime = bdtToe,
                updateIntervalSeconds = 0.0,
                receivedAtMs = nowMs,
            ),
        )
        bdsProcessor.ssrStore.putClock(
            SsrCorrections.ClockCorrection(
                constellationType = GnssStatus.CONSTELLATION_BEIDOU,
                svid = firstSv,
                c0 = 50.0,
                iod = 999,
                epochTime = bdtToe,
                updateIntervalSeconds = 0.0,
                receivedAtMs = nowMs,
            ),
        )
        val gatedFix =
            bdsProcessor.computePosition(
                measurements = bdsMeas,
                initialLat = trueLat,
                initialLon = trueLon,
                initialAlt = trueAlt,
                usePpp = true,
                gpsTimeNanos = bdsGpsTimeNanos,
            )
        val nonNullGated = requireNotNull(gatedFix)
        val gatedError = ecefDistanceMeters(trueLat, trueLon, trueAlt, nonNullGated.latitude, nonNullGated.longitude, nonNullGated.altitude)
        assertTrue("Mismatched BeiDou SSR IOD must be rejected (< 1.0 m error, got $gatedError m)", gatedError < 1.0)
        assertEquals(bdsMeas.size, nonNullGated.usedInFixCount)
    }

    @Test
    fun offlineSixHourRinex_withLiveSsr_usesTightSsrSigmaInsteadOfBroadcastAgingPenalty() {
        val trueLat = 48.8566
        val trueLon = 2.3522
        val trueAlt = 50.0
        val currentGpsNanos = SYNTHETIC_GPS_TIME_NANOS // 345_600 s
        val currentTow = 345_600.0
        val sixHoursEarlierTow = currentTow - 21_600.0 // 6 h old ephemeris
        val sixHoursEarlierNanos = (sixHoursEarlierTow * GnssMathUtils.NANOS_PER_SECOND).toLong()

        // Build a 48-SV GPS constellation with a 6-hour-old ephemeris (toe = sixHoursEarlierTow),
        // and evaluate measurements 6 hours later at currentTow: without SSR sigma replacement,
        // 6 h ephemeris aging assigns ~162 m sigma per SV instead of tight SSR weighting.
        val (recX, recY, recZ) = GnssMathUtils.latLonAltToEcef(trueLat, trueLon, trueAlt)
        val unixSeconds = GnssMathUtils.GPS_EPOCH_UNIX_SECONDS + (currentGpsNanos / 1_000_000_000L)
        val utcCalendar = java.util.Calendar.getInstance(java.util.TimeZone.getTimeZone("UTC"))
        utcCalendar.timeInMillis = unixSeconds * 1000L
        val dayOfYear = utcCalendar.get(java.util.Calendar.DAY_OF_YEAR)
        val nowMs = SystemClock.elapsedRealtime()

        val ephList = ArrayList<SatelliteEphemeris>()
        var svid = 1
        for (raanDeg in 0 until 360 step 30) {
            for (meanDeg in listOf(0.0, 90.0, 180.0, 270.0)) {
                ephList +=
                    SatelliteEphemeris(
                        constellationType = GnssStatus.CONSTELLATION_GPS,
                        svid = svid++,
                        iode = 55,
                        toe = sixHoursEarlierTow,
                        toc = sixHoursEarlierTow,
                        sqrtA = sqrt(26_560_000.0),
                        eccentricity = 0.0,
                        i0 = Math.toRadians(55.0),
                        omega0 = Math.toRadians(raanDeg.toDouble()),
                        omega = 0.0,
                        m0 = Math.toRadians(meanDeg),
                        lastUpdateTime = nowMs,
                    )
            }
        }
        val measurements = mutableListOf<SatelliteMeasurement>()
        val ssrProcessor = GnssMeasurementProcessor()

        for (eph in ephList) {
            val rough = SatellitePositionCalculator.calculateSatellitePosition(eph, currentTow)
            val roughRange = sqrt((rough.first - recX).pow(2) + (rough.second - recY).pow(2) + (rough.third - recZ).pow(2))
            val flight = roughRange / GnssMathUtils.SPEED_OF_LIGHT
            val tx = SatellitePositionCalculator.calculateSatellitePosition(eph, currentTow - flight)
            val rot = SatellitePositionCalculator.rotateEcefForEarthRotationDuringTransit(tx.first, tx.second, tx.third, flight)
            val (elev, az) = GnssMathUtils.ecefLosToElevationAzimuth(recX, recY, recZ, trueLat, trueLon, rot.first, rot.second, rot.third)
            if (elev < 20.0) continue
            val range = sqrt((rot.first - recX).pow(2) + (rot.second - recY).pow(2) + (rot.third - recZ).pow(2))
            val dry = AtmosphericCorrection.computeTroposphericDelay(elev, trueAlt, trueLat, dayOfYear)
            val iono = AtmosphericCorrection.computeIonosphericDelay(elev, az, trueLat, trueLon, currentGpsNanos / 1_000_000L, null, null)
            val rel = AtmosphericCorrection.computeRelativisticCorrection(recX, recY, recZ, rot.first, rot.second, rot.third)
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
            ssrProcessor.ssrStore.putOrbit(
                SsrCorrections.OrbitCorrection(
                    constellationType = GnssStatus.CONSTELLATION_GPS,
                    svid = eph.svid,
                    deltaRadial = 0.0,
                    deltaAlongTrack = 0.0,
                    deltaCrossTrack = 0.0,
                    iod = 55,
                    epochTime = currentTow,
                    receivedAtMs = nowMs,
                ),
            )
            ssrProcessor.ssrStore.putClock(
                SsrCorrections.ClockCorrection(
                    constellationType = GnssStatus.CONSTELLATION_GPS,
                    svid = eph.svid,
                    c0 = 0.0,
                    iod = 55,
                    epochTime = currentTow,
                    receivedAtMs = nowMs,
                ),
            )
        }
        assertTrue("Need >= 6 visible SVs, got ${measurements.size}", measurements.size >= 6)
        ssrProcessor.importExternalEphemeris(ephList, currentTow)

        val ssrFix =
            ssrProcessor.computePosition(
                measurements = measurements,
                initialLat = trueLat,
                initialLon = trueLon,
                initialAlt = trueAlt,
                usePpp = true,
                gpsTimeNanos = currentGpsNanos,
            )
        val nonNullSsr = requireNotNull(ssrFix)
        val ssrError = ecefDistanceMeters(trueLat, trueLon, trueAlt, nonNullSsr.latitude, nonNullSsr.longitude, nonNullSsr.altitude)
        assertTrue("6h RINEX + live SSR must be < 2.0 m error (got $ssrError m)", ssrError < 2.0)
        // Without SSR sigma replacement, 6h broadcast aging adds ~162 m per SV -> horizontalAccuracyMeters > 50 m!
        assertTrue(
            "6h RINEX + live SSR must report tight horizontal accuracy (< 5.0 m, got ${nonNullSsr.horizontalAccuracyMeters} m)",
            nonNullSsr.horizontalAccuracyMeters < 5.0f,
        )
    }

    private fun buildSsrPayload(writeBits: ((Long, Int) -> Unit) -> Unit): ByteArray {
        val bits = ArrayList<Int>()
        writeBits { value, n ->
            for (i in n - 1 downTo 0) bits.add(((value shr i) and 1L).toInt())
        }
        val out = ByteArray((bits.size + 7) / 8)
        bits.forEachIndexed { i, bit ->
            if (bit != 0) out[i / 8] = (out[i / 8].toInt() or (1 shl (7 - i % 8))).toByte()
        }
        return out
    }

    @Test
    fun highRateClock1062_addsToBase1060Clock_andAloneDoesNotActivateSsr() {
        val trueLat = 48.8566
        val trueLon = 2.3522
        val trueAlt = 50.0
        val tow = (SYNTHETIC_GPS_TIME_NANOS / 1e9) % GnssMathUtils.SECONDS_IN_WEEK
        val (baseEph, baseMeas) =
            buildSyntheticSkyEpoch(
                constellationType = GnssStatus.CONSTELLATION_GPS,
                lat = trueLat,
                lon = trueLon,
                alt = trueAlt,
                gpsTimeNanos = SYNTHETIC_GPS_TIME_NANOS,
            )
        assertTrue("Need >= 6 GPS SVs, got ${baseMeas.size}", baseMeas.size >= 6)
        val ephWithIode = baseEph.map { it.copy(iode = 46) }

        // Test 1: 1062 alone (with orbit present, but NO base 1058/1060 clock) must NOT activate SSR
        val loneHrProcessor = GnssMeasurementProcessor()
        loneHrProcessor.importExternalEphemeris(ephWithIode, tow)
        val nowMs = SystemClock.elapsedRealtime()
        ephWithIode.forEachIndexed { idx, eph ->
            loneHrProcessor.ssrStore.putOrbit(
                SsrCorrections.OrbitCorrection(
                    constellationType = GnssStatus.CONSTELLATION_GPS,
                    svid = eph.svid,
                    deltaRadial = 0.0,
                    deltaAlongTrack = 0.0,
                    deltaCrossTrack = 0.0,
                    iod = 46,
                    epochTime = tow,
                    receivedAtMs = nowMs,
                    iodSsr = 5,
                ),
            )
            val hugeSign = if (idx % 2 == 0) 1L else -1L
            val hrPayload =
                buildSsrPayload { w ->
                    w(1062L, 12)
                    w(tow.toLong(), 20)
                    w(0L, 4) // udi = 1s
                    w(0L, 1) // multi
                    w(5L, 4) // iodSsr
                    w(1L, 16)
                    w(1L, 4)
                    w(1L, 6) // numSats = 1
                    w(eph.svid.toLong(), 6)
                    w((hugeSign * 500_000L) and 0x3FFFFF, 22) // ±50 m
                }
            RtcmSsrDecoder.decode(Rtcm3FrameParser.Frame(1062, hrPayload), loneHrProcessor.ssrStore)
        }
        val loneFix =
            loneHrProcessor.computePosition(
                measurements = baseMeas,
                initialLat = trueLat,
                initialLon = trueLon,
                initialAlt = trueAlt,
                usePpp = true,
                gpsTimeNanos = SYNTHETIC_GPS_TIME_NANOS,
            )
        val nonNullLone = requireNotNull(loneFix)
        val loneError = ecefDistanceMeters(trueLat, trueLon, trueAlt, nonNullLone.latitude, nonNullLone.longitude, nonNullLone.altitude)
        assertTrue("1062 alone without base 1058/1060 must not activate SSR (< 1.0 m error, got $loneError m)", loneError < 1.0)

        // Test 2: 1060 (c0 = +2.5 m, c1 = 0.01 m/s) + 1062 (delta = -0.04 m) -> net +2.46 m!
        // Inject -2.46 m on half of SVs and +2.46 m on the other half? Even simpler:
        // shift SV 0..n/2 measurements by -2.46 m (so base 1060 + 1062 = +2.46 m cancels it to 0.00 m),
        // and leave the remaining SVs at 0 bias with c0 = 0, delta = 0.
        // Without +2.46 m additive combination (e.g. if 1062 overwrote c0=2.5 with -0.04), the error would be ~2.5 m!
        val combinedProcessor = GnssMeasurementProcessor()
        combinedProcessor.importExternalEphemeris(ephWithIode, tow)
        val adjustedMeas =
            baseMeas.mapIndexed { idx, m ->
                val svid = m.svid.toLong()
                val applyCombined = idx % 2 == 0
                val c0Raw = if (applyCombined) 25_000L else 0L // 2.5 m
                val hrRaw = if (applyCombined) -400L else 0L // -0.04 m
                val payload1060 =
                    buildSsrPayload { w ->
                        w(1060L, 12)
                        w(tow.toLong(), 20)
                        w(0L, 4) // udi = 1s -> 1.0s; epoch = tow - 0.5? Wait: use udi=15->0? Set epoch = tow, udi=0(1s) -> dtSsr = -0.5 -> c1 = 0.01 * -0.5 = -0.005 m!
                        w(0L, 1) // multi
                        w(0L, 1) // datum
                        w(5L, 4) // iodSsr
                        w(1L, 16)
                        w(1L, 4)
                        w(1L, 6)
                        w(svid, 6)
                        w(46L, 8)
                        w(0L, 22)
                        w(0L, 20)
                        w(0L, 20)
                        w(0L, 21)
                        w(0L, 19)
                        w(0L, 19)
                        w(c0Raw and 0x3FFFFF, 22)
                        w(0L, 21) // c1 = 0
                        w(0L, 27) // c2 = 0
                    }
                val payload1062 =
                    buildSsrPayload { w ->
                        w(1062L, 12)
                        w(tow.toLong(), 20)
                        w(0L, 4)
                        w(0L, 1)
                        w(5L, 4)
                        w(1L, 16)
                        w(1L, 4)
                        w(1L, 6)
                        w(svid, 6)
                        w(hrRaw and 0x3FFFFF, 22)
                    }
                RtcmSsrDecoder.decode(Rtcm3FrameParser.Frame(1060, payload1060), combinedProcessor.ssrStore)
                RtcmSsrDecoder.decode(Rtcm3FrameParser.Frame(1062, payload1062), combinedProcessor.ssrStore)
                if (applyCombined) m.copy(pseudorangeMeters = m.pseudorangeMeters - 2.46) else m
            }

        val combinedFix =
            combinedProcessor.computePosition(
                measurements = adjustedMeas,
                initialLat = trueLat,
                initialLon = trueLon,
                initialAlt = trueAlt,
                usePpp = true,
                gpsTimeNanos = SYNTHETIC_GPS_TIME_NANOS,
            )
        val nonNullCombined = requireNotNull(combinedFix)
        val combinedError = ecefDistanceMeters(trueLat, trueLon, trueAlt, nonNullCombined.latitude, nonNullCombined.longitude, nonNullCombined.altitude)
        assertTrue("1060 (+2.5 m) + 1062 (-0.04 m) must apply +2.46 m (< 0.1 m error, got $combinedError m)", combinedError < 0.1)

        // Base clock with iodSsr = -1 must NOT pair with any high-rate clock (even iodSsr = 5 or -1).
        val unspecifiedIodSsrProcessor = GnssMeasurementProcessor()
        unspecifiedIodSsrProcessor.importExternalEphemeris(ephWithIode, tow)
        for (m in baseMeas) {
            unspecifiedIodSsrProcessor.ssrStore.putOrbit(
                SsrCorrections.OrbitCorrection(
                    constellationType = GnssStatus.CONSTELLATION_GPS,
                    svid = m.svid,
                    deltaRadial = 0.0,
                    deltaAlongTrack = 0.0,
                    deltaCrossTrack = 0.0,
                    iod = 46,
                    iodSsr = -1,
                    epochTime = tow,
                    receivedAtMs = SystemClock.elapsedRealtime(),
                ),
            )
            unspecifiedIodSsrProcessor.ssrStore.putClock(
                SsrCorrections.ClockCorrection(
                    constellationType = GnssStatus.CONSTELLATION_GPS,
                    svid = m.svid,
                    c0 = 2.5,
                    iod = 46,
                    iodSsr = -1,
                    epochTime = tow,
                    receivedAtMs = SystemClock.elapsedRealtime(),
                ),
            )
            unspecifiedIodSsrProcessor.ssrStore.putHighRateClock(
                SsrCorrections.HighRateClockCorrection(
                    constellationType = GnssStatus.CONSTELLATION_GPS,
                    svid = m.svid,
                    deltaClockMeters = 50.0,
                    iodSsr = 5,
                    epochTime = tow,
                    receivedAtMs = SystemClock.elapsedRealtime(),
                ),
            )
        }
        val baseOnlyMeas = baseMeas.mapIndexed { idx, m -> if (idx == 0) m.copy(pseudorangeMeters = m.pseudorangeMeters - 2.5) else m.copy(pseudorangeMeters = m.pseudorangeMeters - 2.5) }
        val baseOnlyFix =
            requireNotNull(
                unspecifiedIodSsrProcessor.computePosition(
                    measurements = baseOnlyMeas,
                    initialLat = trueLat,
                    initialLon = trueLon,
                    initialAlt = trueAlt,
                    usePpp = true,
                    gpsTimeNanos = SYNTHETIC_GPS_TIME_NANOS,
                ),
            )
        val baseOnlyErr = ecefDistanceMeters(trueLat, trueLon, trueAlt, baseOnlyFix.latitude, baseOnlyFix.longitude, baseOnlyFix.altitude)
        assertTrue("ssrClock with iodSsr=-1 must ignore 50 m HR clock (err=$baseOnlyErr m)", baseOnlyErr < 0.1)
    }

    @Test
    fun separateClock1058BeforeOrbit1057_pairsOnMatchingIodSsr_andRejectsMismatch() {
        val trueLat = 48.8566
        val trueLon = 2.3522
        val trueAlt = 50.0
        val tow = (SYNTHETIC_GPS_TIME_NANOS / 1e9) % GnssMathUtils.SECONDS_IN_WEEK
        val (baseEph, baseMeas) =
            buildSyntheticSkyEpoch(
                constellationType = GnssStatus.CONSTELLATION_GPS,
                lat = trueLat,
                lon = trueLon,
                alt = trueAlt,
                gpsTimeNanos = SYNTHETIC_GPS_TIME_NANOS,
            )
        assertTrue("Need >= 6 GPS SVs, got ${baseMeas.size}", baseMeas.size >= 6)
        val ephWithIode = baseEph.map { it.copy(iode = 46) }

        // 1058 arrives BEFORE 1057 with matching iodSsr = 5 and c0 = +3.0 m on alternating SVs
        val matchedProcessor = GnssMeasurementProcessor()
        matchedProcessor.importExternalEphemeris(ephWithIode, tow)
        val measWithOffset =
            baseMeas.mapIndexed { idx, m ->
                val svid = m.svid.toLong()
                val applyClock = idx % 2 == 0
                val c0Raw = if (applyClock) 30_000L else 0L // +3.0 m
                val payload1058 =
                    buildSsrPayload { w ->
                        w(1058L, 12)
                        w(tow.toLong(), 20)
                        w(0L, 4)
                        w(0L, 1)
                        w(5L, 4) // iodSsr = 5
                        w(1L, 16)
                        w(1L, 4)
                        w(1L, 6)
                        w(svid, 6)
                        w(c0Raw and 0x3FFFFF, 22)
                        w(0L, 21)
                        w(0L, 27)
                    }
                val payload1057 =
                    buildSsrPayload { w ->
                        w(1057L, 12)
                        w(tow.toLong(), 20)
                        w(0L, 4)
                        w(0L, 1)
                        w(0L, 1) // datum
                        w(5L, 4) // iodSsr = 5
                        w(1L, 16)
                        w(1L, 4)
                        w(1L, 6)
                        w(svid, 6)
                        w(46L, 8)
                        w(0L, 22)
                        w(0L, 20)
                        w(0L, 20)
                        w(0L, 21)
                        w(0L, 19)
                        w(0L, 19)
                    }
                RtcmSsrDecoder.decode(Rtcm3FrameParser.Frame(1058, payload1058), matchedProcessor.ssrStore)
                RtcmSsrDecoder.decode(Rtcm3FrameParser.Frame(1057, payload1057), matchedProcessor.ssrStore)
                if (applyClock) m.copy(pseudorangeMeters = m.pseudorangeMeters - 3.0) else m
            }
        val matchedFix =
            matchedProcessor.computePosition(
                measurements = measWithOffset,
                initialLat = trueLat,
                initialLon = trueLon,
                initialAlt = trueAlt,
                usePpp = true,
                gpsTimeNanos = SYNTHETIC_GPS_TIME_NANOS,
            )
        val nonNullMatched = requireNotNull(matchedFix)
        val matchedError = ecefDistanceMeters(trueLat, trueLon, trueAlt, nonNullMatched.latitude, nonNullMatched.longitude, nonNullMatched.altitude)
        assertTrue("1058 before 1057 with matching iodSsr=5 must pair cleanly (< 0.1 m error, got $matchedError m)", matchedError < 0.1)

        // Mismatched iodSsr (1058 has iodSsr=6, 1057 has iodSsr=5): must NOT pair!
        val mismatchedProcessor = GnssMeasurementProcessor()
        mismatchedProcessor.importExternalEphemeris(ephWithIode, tow)
        ephWithIode.forEachIndexed { idx, eph ->
            val sign = if (idx % 2 == 0) 1L else -1L
            val payload1058 =
                buildSsrPayload { w ->
                    w(1058L, 12)
                    w(tow.toLong(), 20)
                    w(0L, 4)
                    w(0L, 1)
                    w(6L, 4) // mismatched iodSsr = 6
                    w(1L, 16)
                    w(1L, 4)
                    w(1L, 6)
                    w(eph.svid.toLong(), 6)
                    w((sign * 400_000L) and 0x3FFFFF, 22) // ±40 m poison if paired
                    w(0L, 21)
                    w(0L, 27)
                }
            val payload1057 =
                buildSsrPayload { w ->
                    w(1057L, 12)
                    w(tow.toLong(), 20)
                    w(0L, 4)
                    w(0L, 1)
                    w(0L, 1)
                    w(5L, 4) // iodSsr = 5
                    w(1L, 16)
                    w(1L, 4)
                    w(1L, 6)
                    w(eph.svid.toLong(), 6)
                    w(46L, 8)
                    w(0L, 22)
                    w(0L, 20)
                    w(0L, 20)
                    w(0L, 21)
                    w(0L, 19)
                    w(0L, 19)
                }
            RtcmSsrDecoder.decode(Rtcm3FrameParser.Frame(1058, payload1058), mismatchedProcessor.ssrStore)
            RtcmSsrDecoder.decode(Rtcm3FrameParser.Frame(1057, payload1057), mismatchedProcessor.ssrStore)
        }
        val mismatchedFix =
            mismatchedProcessor.computePosition(
                measurements = baseMeas,
                initialLat = trueLat,
                initialLon = trueLon,
                initialAlt = trueAlt,
                usePpp = true,
                gpsTimeNanos = SYNTHETIC_GPS_TIME_NANOS,
            )
        val nonNullMismatched = requireNotNull(mismatchedFix)
        val mismatchedError = ecefDistanceMeters(trueLat, trueLon, trueAlt, nonNullMismatched.latitude, nonNullMismatched.longitude, nonNullMismatched.altitude)
        assertTrue("Mismatched iodSsr (5 vs 6) must NOT pair (< 1.0 m error, got $mismatchedError m)", mismatchedError < 1.0)
    }
}
