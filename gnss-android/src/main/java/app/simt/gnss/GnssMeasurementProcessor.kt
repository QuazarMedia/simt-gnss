package app.simt.gnss

import android.location.GnssMeasurement
import android.location.GnssMeasurementsEvent
import android.location.GnssNavigationMessage
import android.location.GnssStatus
import android.os.Build
import android.os.SystemClock
import android.util.Log
import app.simt.gnss.core.correction.AtmosphericCorrection
import app.simt.gnss.core.correction.RtkBaseStore
import app.simt.gnss.core.correction.SsrCorrectionApplicator
import app.simt.gnss.core.correction.SsrCorrections
import app.simt.gnss.core.filter.DopplerSmoother
import app.simt.gnss.core.filter.GnssInterEpochKalmanFilter
import app.simt.gnss.core.filter.HatchFilter
import app.simt.gnss.core.filter.carrierDeltaMatchesPseudorangeRate
import app.simt.gnss.core.filter.receiverClockStepMeters
import app.simt.gnss.core.model.GnssSolutionMode
import app.simt.gnss.core.model.PositionResult
import app.simt.gnss.core.model.SatelliteEphemeris
import app.simt.gnss.core.model.SatelliteMeasurement
import app.simt.gnss.core.orbit.SatellitePositionCalculator
import app.simt.gnss.core.orbit.TransmitTiming
import app.simt.gnss.core.parser.BeidouNavMessageParser
import app.simt.gnss.core.parser.GalileoNavMessageParser
import app.simt.gnss.core.parser.GlonassNavMessageParser
import app.simt.gnss.core.parser.GpsCnavMt30Parser
import app.simt.gnss.core.parser.GpsNavMessageParser
import app.simt.gnss.core.parser.SbasNavMessageParser
import app.simt.gnss.core.solver.DopplerVelocitySolver
import app.simt.gnss.core.solver.GnssPositionSolver
import app.simt.gnss.core.solver.GnssPositionSolver.Accuracy
import app.simt.gnss.core.solver.RtkAmbiguityMemory
import app.simt.gnss.core.solver.RtkDoubleDifferenceSolver
import app.simt.gnss.core.solver.TdcpDisplacementSolver
import app.simt.gnss.core.utils.GnssMathUtils
import app.simt.gnss.core.utils.GnssMathUtils.SPEED_OF_LIGHT
import app.simt.gnss.core.validator.GpsNavMessageValidator
import kotlin.math.*

/**
 * Orchestrates GNSS measurement processing and position computation.
 *
 * @param suppressSbas When `true`, SBAS (WAAS, EGNOS, MSAS, GAGAN, SDCM, …) is completely bypassed:
 *   SBAS L1 navigation messages are not decoded, SBAS satellites are not used as ranging sources,
 *   and SBAS fast/long-term/iono-grid corrections are never applied. Intended for dual-frequency
 *   (L1 + L5 / E5) devices, where the L1/L5 iono-free combination is substantially more accurate
 *   than SBAS grid corrections; skipping SBAS also avoids the CPU/battery cost of decoding the
 *   250 bps SBAS message stream when it cannot improve the solution.
 */
internal class GnssMeasurementProcessor(
    private val suppressSbas: Boolean = false,
) {
    init {
        AndroidGnssPlatform.install()
    }

    private val lock = Any()
    private val hatchFilter = HatchFilter()
    private val dopplerSmoother = DopplerSmoother()

    /**
     * Previous epoch's `fullBiasNanos`, or `null` before one exists. Kept as a `Long`: the value is
     * ~1.4×10¹⁸ ns and a `Double` difference at that magnitude is quantized to 256 ns (≈ 77 m).
     * [lastBiasNanos] holds the sub-nanosecond part, which is already a `Double` in the platform API.
     */
    private var lastFullBiasNanos: Long? = null
    private var lastBiasNanos: Double = 0.0

    /**
     * Hardware clock discontinuity count at the previous epoch, or `null` before one exists. A
     * change means the clock was rebased, so [lastFullBiasNanos] belongs to a different clock and
     * must not be differenced.
     */
    private var lastHardwareClockDiscontinuityCount: Int? = null

    /** Hardware clock step for the epoch currently being solved. Zero when there is no previous clock. */
    private var epochClockStepMeters: Double = 0.0

    /** Published ECEF at the previous fix, the linearization point for carrier displacement. */
    private var tdcpAnchor: Triple<Double, Double, Double>? = null
    private var tdcpEpochNanos: Long = 0L
    private val tdcpSignals = HashMap<Pair<Int, Int>, TdcpSignal>()
    private val rtkAmbiguityMemory = RtkAmbiguityMemory()
    private val ephemerisMap = mutableMapOf<Pair<Int, Int>, SatelliteEphemeris>()

    /** Latest GPS CNAV MT30 ISC(L1CA)/ISC(L5I5) per SV; keyed like [ephemerisMap]. */
    private val gpsCnavIscMap = mutableMapOf<Pair<Int, Int>, GpsCnavIscRecord>()
    private val subframeStorage = mutableMapOf<Pair<Int, Int>, Array<ByteArray?>>()
    private val galileoWordStorage = mutableMapOf<Pair<Int, Int>, MutableMap<Int, ByteArray>>()
    private val glonassStringStorage = mutableMapOf<Pair<Int, Int>, MutableMap<Int, ByteArray>>()

    /** SBAS PRN mask (MT 1) with its IODP, keyed by SBAS PRN. */
    private val sbasPrnMasks = mutableMapOf<Int, SbasNavMessageParser.SbasPrnMask>()

    /** SBAS fast corrections (MT 2–5/24): PRC per mask index, keyed by SBAS PRN. */
    private val sbasFastCorrections = mutableMapOf<Int, MutableMap<Int, SbasNavMessageParser.SbasFastCorrection>>()

    /**
     * SBAS long-term corrections (MT 24/25), keyed by **GPS PRN** — resolved at ingest by
     * translating the broadcast PRN-mask number through the IODP-matching MT1 mask.
     */
    private val sbasLongTermCorrections = mutableMapOf<Int, SbasNavMessageParser.SbasLongTermCorrection>()

    /** GPS TOW (seconds) when the last GLONASS nav message was received, for tb conversion. */
    private var lastGpsTimeOfWeekSeconds: Double = 0.0

    /** GPS−UTC leap seconds from the receiver clock (fallback: compile-time constant). */
    private var lastKnownLeapSeconds: Int = GnssMathUtils.GPS_UTC_LEAP_SECONDS.toInt()
    private val gpsLastBits = mutableMapOf<Pair<Int, Int>, Int>()
    private var ionoAlpha: DoubleArray? = null
    private var ionoBeta: DoubleArray? = null

    /** SSR (State Space Representation) corrections from IGS RTS NTRIP stream. */
    val ssrStore = SsrCorrections.Store()

    /** RTK reference-station observations + position from an OSR (MSM) NTRIP stream. */
    val rtkBaseStore = RtkBaseStore()

    private val pppState = PppState()
    private val interEpochKalman = GnssInterEpochKalmanFilter()
    private var smoothingMode: SmoothingMode = SmoothingMode.CARRIER_HATCH

    fun setSmoothingMode(mode: SmoothingMode) =
        synchronized(lock) {
            if (smoothingMode == mode) return@synchronized
            smoothingMode = mode
            hatchFilter.reset()
            dopplerSmoother.reset()
            interEpochKalman.reset()
        }

    /**
     * Seed position for the next epoch, plus the ambiguity and zenith-wet-delay holders a
     * sequential PPP filter would carry. The single-epoch solve does not estimate either
     * (review #85): a phase row with its own free ambiguity adds nothing, and a free wet
     * delay then trades off against height and clock.
     */
    internal class PppState {
        /** Float ambiguity state (meters), keyed by constellation + SVID; not integer-resolved. */
        val carrierAmbiguities = mutableMapOf<Pair<Int, Int>, Double>()
        var zenithWetDelay: Double = 0.1
        var lastX: Double? = null
        var lastY: Double? = null
        var lastZ: Double? = null
        var lastTimeNanos: Long = 0

        fun hasLastPosition(): Boolean = lastX != null

        fun getLastPositionEcef(): Triple<Double, Double, Double>? {
            val x = lastX ?: return null
            val y = lastY ?: return null
            val z = lastZ ?: return null
            return Triple(x, y, z)
        }

        fun updatePosition(
            x: Double,
            y: Double,
            z: Double,
            timeNanos: Long,
        ) {
            lastX = x
            lastY = y
            lastZ = z
            lastTimeNanos = timeNanos
        }

        fun resetAmbiguity(satKey: Pair<Int, Int>) {
            carrierAmbiguities.remove(satKey)
        }

        fun reset() {
            carrierAmbiguities.clear()
            zenithWetDelay = 0.1
            lastX = null
            lastY = null
            lastZ = null
            lastTimeNanos = 0
        }
    }

    private data class GpsCnavIscRecord(
        val iscL1CaSeconds: Double,
        val iscL5I5Seconds: Double,
        val lastUpdateElapsedMs: Long,
    )

    /**
     * Cached per-SV model terms reused across WLS iterations (see [GnssMathUtils.WLS_MODEL_REUSE_DISTANCE_METERS]).
     * [refX]/[refY]/[refZ]/[refClockBias] record the receiver state at which the terms were last
     * evaluated; the cache is consulted only while the receiver estimate stays within the reuse radius.
     */
    private class SvModelTerms(
        val refX: Double,
        val refY: Double,
        val refZ: Double,
        val refClockBias: Double,
        val satX: Double,
        val satY: Double,
        val satZ: Double,
        val satClockMeters: Double,
        val ionoDelayL1: Double,
        val dryDelay: Double,
        val elevationDeg: Double,
        val azimuthDeg: Double,
    )

    internal fun getConstellationForNavigationMessageType(type: Int): Int =
        when (type) {
            GnssNavigationMessage.TYPE_GPS_L1CA,
            GnssNavigationMessage.TYPE_GPS_L2CNAV,
            GnssNavigationMessage.TYPE_GPS_L5CNAV,
            -> GnssStatus.CONSTELLATION_GPS
            GnssNavigationMessage.TYPE_QZS_L1CA -> GnssStatus.CONSTELLATION_QZSS
            GnssNavigationMessage.TYPE_IRN_L5CA,
            GnssNavigationMessage.TYPE_IRN_L5,
            GnssNavigationMessage.TYPE_IRN_L1,
            -> GnssStatus.CONSTELLATION_IRNSS
            GnssNavigationMessage.TYPE_GAL_F,
            GnssNavigationMessage.TYPE_GAL_I,
            -> GnssStatus.CONSTELLATION_GALILEO
            GnssNavigationMessage.TYPE_GLO_L1CA -> GnssStatus.CONSTELLATION_GLONASS
            GnssNavigationMessage.TYPE_BDS_CNAV1,
            GnssNavigationMessage.TYPE_BDS_CNAV2,
            GnssNavigationMessage.TYPE_BDS_D1,
            GnssNavigationMessage.TYPE_BDS_D2,
            -> GnssStatus.CONSTELLATION_BEIDOU
            0x0101 -> GnssStatus.CONSTELLATION_SBAS // TYPE_SBAS_L1CA (not yet in all SDK versions)
            else -> {
                // Some chipsets report SBAS nav messages with constellation type = SBAS (2)
                // but use an unrecognized message type. Check if the SVID falls in the SBAS
                // PRN range (120–158) and treat as SBAS.
                GnssStatus.CONSTELLATION_UNKNOWN
            }
        }

    /**
     * Process GNSS navigation messages to extract Ephemeris data.
     */
    fun processNavigationMessage(message: GnssNavigationMessage) =
        synchronized(lock) {
            // Accept STATUS_PARITY_PASSED and STATUS_UNKNOWN (many chipsets report UNKNOWN
            // even for valid messages; rejecting them blocks all ephemeris acquisition).
            if (message.status == GnssNavigationMessage.STATUS_PARITY_REBUILT) {
                return@synchronized
            }

            var constellation = getConstellationForNavigationMessageType(message.type)
            val svid = message.svid
            // Fallback: some chipsets report SBAS nav messages with an unrecognized message
            // type. If constellation is unknown but SVID falls in the SBAS PRN range, treat
            // as SBAS. Also handle when Android reports SBAS constellation type directly.
            if (constellation == GnssStatus.CONSTELLATION_UNKNOWN && svid in 120..158) {
                constellation = GnssStatus.CONSTELLATION_SBAS
            }
            val satKey = Pair(constellation, svid)
            val data = message.data

            Log.d(TAG, "Nav msg type=${message.type} constellation=${getConstellationName(constellation)} svid=$svid dataSize=${data.size} status=${message.status}")

            when (constellation) {
                GnssStatus.CONSTELLATION_GPS -> {
                    when (message.type) {
                        GnssNavigationMessage.TYPE_GPS_L1CA -> {
                            processGpsLikeL1CaNavigationMessage(
                                satKey = satKey,
                                svid = svid,
                                data = data,
                                constellationType = GnssStatus.CONSTELLATION_GPS,
                                updateKlobucharCoefficients = true,
                                parityAttested = message.status == GnssNavigationMessage.STATUS_PARITY_PASSED,
                            )
                        }
                        GnssNavigationMessage.TYPE_GPS_L2CNAV,
                        GnssNavigationMessage.TYPE_GPS_L5CNAV,
                        -> {
                            GpsCnavMt30Parser.parseL1L5IscIfComplete(data)?.let { isc ->
                                gpsCnavIscMap[satKey] =
                                    GpsCnavIscRecord(
                                        iscL1CaSeconds = isc.iscL1CaSeconds,
                                        iscL5I5Seconds = isc.iscL5I5Seconds,
                                        lastUpdateElapsedMs = SystemClock.elapsedRealtime(),
                                    )
                            }
                        }
                    }
                }
                GnssStatus.CONSTELLATION_QZSS -> {
                    if (message.type == GnssNavigationMessage.TYPE_QZS_L1CA) {
                        processGpsLikeL1CaNavigationMessage(
                            satKey = satKey,
                            svid = svid,
                            data = data,
                            constellationType = GnssStatus.CONSTELLATION_QZSS,
                            updateKlobucharCoefficients = false,
                            parityAttested = message.status == GnssNavigationMessage.STATUS_PARITY_PASSED,
                        )
                    }
                }
                GnssStatus.CONSTELLATION_GALILEO -> {
                    if (message.type == GnssNavigationMessage.TYPE_GAL_I) {
                        // Galileo I/NAV page (240 bits): bits 0-1 are sync flags,
                        // word type is bits 2-7 (Galileo OS SIS ICD Section 4.3.2)
                        val wordType = GnssMathUtils.getBits(data, 2, 6)
                        Log.d(TAG, "GAL I/NAV svid=$svid wordType=$wordType dataSize=${data.size} wordsCollected=${galileoWordStorage[satKey]?.size ?: 0}")
                        if (wordType in 1..6) {
                            val words = galileoWordStorage.getOrPut(satKey) { mutableMapOf() }
                            words[wordType] = data

                            // Ephemeris is fully defined by Words 1-5 (Word 6 only carries UTC
                            // and GPS-Galileo time offsets, which we do not use for ranging).
                            if ((1..5).all { words.containsKey(it) }) {
                                val ephemeris = GalileoNavMessageParser.parseGalileoEphemeris(svid, words)
                                if (ephemeris != null) {
                                    storeEphemeris(satKey, ephemeris, sourceLabel = "GAL_I")
                                } else {
                                    Log.w(TAG, "GAL I/NAV ephemeris parse returned null svid=$svid — word data may be corrupted")
                                }
                            }
                        }
                    }
                }
                GnssStatus.CONSTELLATION_GLONASS -> {
                    if (message.type == GnssNavigationMessage.TYPE_GLO_L1CA) {
                        // GLONASS L1OF: 85 data bits in 11 bytes (MSB-first).
                        // String number from Android HAL submessageId (1-15).
                        val stringNumber = message.submessageId
                        // Verify against data bits 1-4 (string number embedded in data)
                        val dataStringNumber = GnssMathUtils.getBits(data, 1, 4)
                        val effectiveStringNumber = if (stringNumber in 1..15) stringNumber else dataStringNumber
                        Log.d(TAG, "GLO L1OF svid=$svid string=$effectiveStringNumber dataSize=${data.size} stringsCollected=${glonassStringStorage[satKey]?.size ?: 0}")
                        if (effectiveStringNumber in 1..4) {
                            val strings = glonassStringStorage.getOrPut(satKey) { mutableMapOf() }
                            strings[effectiveStringNumber] = data

                            if (strings.size >= 4) {
                                val gpsTow = lastGpsTimeOfWeekSeconds
                                val ephemeris =
                                    GlonassNavMessageParser.parseGlonassEphemeris(
                                        svid = svid,
                                        strings = strings,
                                        gpsTimeOfWeekSeconds = gpsTow,
                                        leapSeconds = lastKnownLeapSeconds,
                                    )
                                if (ephemeris != null) {
                                    storeEphemeris(satKey, ephemeris, sourceLabel = "GLO_L1OF")
                                } else {
                                    Log.w(TAG, "GLO L1OF ephemeris parse returned null svid=$svid — string data may be corrupted")
                                }
                            }
                        }
                    }
                }
                GnssStatus.CONSTELLATION_SBAS -> {
                    // On dual-frequency (L1 + L5/E5) devices, SBAS augmentation is redundant:
                    // the L1/L5 iono-free combination is more accurate than SBAS grid corrections,
                    // so skip the 250 bps message decode entirely to save CPU/battery.
                    if (suppressSbas) return@synchronized
                    processSbasNavigationMessage(svid, data)
                }
                GnssStatus.CONSTELLATION_BEIDOU -> {
                    if (message.type == GnssNavigationMessage.TYPE_BDS_D1) {
                        // AOSP HAL: BDS D1 = 40 bytes, 10×4-byte words,
                        // 30-bit word right-justified in 32 bits (2 MSB = 0).
                        val bdsStride = if (data.size >= 40) 32 else 30
                        val bdsOffset = if (bdsStride == 32) 2 else 0
                        val compact = ByteArray(38)
                        for (i in 0 until 10) {
                            val word = GnssMathUtils.getBits(data, i * bdsStride + bdsOffset, 30)
                            GnssMathUtils.packBits(compact, i * 30, 30, word)
                        }

                        val subframeId = GnssMathUtils.getBits(compact, 15, 3)
                        Log.d(TAG, "BDS D1 svid=$svid subframeId=$subframeId dataSize=${data.size}")
                        if (subframeId in 1..3) {
                            val frames = subframeStorage.getOrPut(satKey) { arrayOfNulls<ByteArray>(5) }
                            frames[subframeId - 1] = compact

                            if (frames[0] != null && frames[1] != null && frames[2] != null) {
                                val ephemeris = BeidouNavMessageParser.parseBeidouEphemeris(svid, frames[0]!!, frames[1]!!, frames[2]!!)
                                if (ephemeris != null) {
                                    storeEphemeris(satKey, ephemeris, sourceLabel = "BDS_D1")
                                } else {
                                    Log.w(TAG, "BDS D1 ephemeris parse returned null svid=$svid — subframe data may be corrupted")
                                }
                            }
                        }
                    }
                }
            }
        }

    internal fun describeFreshEphemerisCoverage(measurements: List<SatelliteMeasurement>): String =
        synchronized(lock) {
            if (measurements.isEmpty()) return@synchronized "none"

            measurements
                .groupBy { it.constellationType }
                .entries
                .sortedBy { it.key }
                .joinToString(",") { (constellationType, constellationMeasurements) ->
                    val uniqueSatKeys = constellationMeasurements.map { it.constellationType to it.svid }.distinct()
                    val freshCount = uniqueSatKeys.count(::hasFreshEphemerisLocked)
                    "${getConstellationName(constellationType)}=$freshCount/${uniqueSatKeys.size}"
                }
        }

    internal fun describeStoredEphemerides(): String =
        synchronized(lock) {
            if (ephemerisMap.isEmpty()) return@synchronized "none"

            ephemerisMap
                .entries
                .groupBy { it.key.first }
                .entries
                .sortedBy { it.key }
                .joinToString(",") { (constellationType, entries) ->
                    val freshCount = entries.count { (_, ephemeris) -> isFreshEphemerisLocked(ephemeris) }
                    "${getConstellationName(constellationType)}=$freshCount/${entries.size}"
                }
        }

    private fun storeEphemeris(
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

    /** TOW the ephemeris data is referenced to (orbit epoch), for recency comparisons. */
    private fun ephemerisReferenceTow(ephemeris: SatelliteEphemeris): Double =
        when {
            ephemeris.isGlonass -> ephemeris.gloTbGpsTow
            ephemeris.isSbas -> ephemeris.sbasT0 // seconds of day; only compared SBAS-vs-SBAS
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
    ): Int =
        synchronized(lock) {
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
            imported
        }

    /**
     * Process SBAS L1 C/A navigation messages. SBAS broadcasts 250-bit messages at 250 bps
     * containing GEO ephemeris (MT 9), fast corrections (MT 2–5/24), and long-term
     * corrections (MT 24/25).
     *
     * The GEO ephemeris (MT 9) allows using the SBAS satellite itself as a ranging source,
     * adding an additional measurement to the position solution. Fast/long-term corrections
     * improve GPS satellite clock and orbit estimates. (The MT 18/26 ionospheric grid is not
     * consumed: single-frequency ionosphere uses Klobuchar, and dual-frequency devices bypass
     * SBAS entirely.)
     */
    private fun processSbasNavigationMessage(
        svid: Int,
        data: ByteArray,
    ) {
        if (data.size < 32) return
        val mt = SbasNavMessageParser.getMessageType(data)
        val satKey = GnssStatus.CONSTELLATION_SBAS to svid
        Log.d(TAG, "SBAS nav msg svid=$svid MT=$mt dataSize=${data.size}")

        when (mt) {
            1 -> {
                // PRN mask — maps mask slots to actual PRNs for fast/long-term corrections
                SbasNavMessageParser.parsePrnMask(data)?.let { mask ->
                    sbasPrnMasks[svid] = mask
                    Log.d(TAG, "SBAS MT1 PRN mask svid=$svid iodp=${mask.iodp}: ${mask.maskToPrn.size} satellites monitored")
                }
            }
            2, 3, 4, 5 -> {
                // Fast corrections for GPS satellites (PRC = pseudorange correction)
                SbasNavMessageParser.parseFastCorrections(data, mt)?.let { corrections ->
                    val store = sbasFastCorrections.getOrPut(svid) { mutableMapOf() }
                    for (fc in corrections) {
                        store[fc.prnMaskIndex] = fc
                    }
                    Log.d(TAG, "SBAS MT$mt fast corrections svid=$svid: ${corrections.size} PRCs")
                }
            }
            9 -> {
                // GEO Navigation Message — ephemeris for SBAS satellite itself (ranging)
                SbasNavMessageParser.parseMt9GeoNavigation(svid, data)?.let { ephemeris ->
                    storeEphemeris(satKey, ephemeris, sourceLabel = "SBAS_MT9")
                }
            }
            24 -> {
                // Mixed fast + long-term corrections
                SbasNavMessageParser.parseMt24MixedCorrections(data)?.let { result ->
                    val store = sbasFastCorrections.getOrPut(svid) { mutableMapOf() }
                    for (fc in result.fastCorrections) {
                        store[fc.prnMaskIndex] = fc
                    }
                    storeSbasLongTermCorrections(svid, result.longTermCorrections)
                    Log.d(TAG, "SBAS MT24 mixed svid=$svid: ${result.fastCorrections.size} fast, ${result.longTermCorrections.size} long-term")
                }
            }
            25 -> {
                // Long-term corrections (orbit + clock deltas for GPS satellites)
                SbasNavMessageParser.parseLongTermCorrections(data)?.let { corrections ->
                    storeSbasLongTermCorrections(svid, corrections)
                    Log.d(TAG, "SBAS MT25 long-term corrections: ${corrections.size} satellites")
                }
            }
        }
    }

    /**
     * Resolves long-term corrections to GPS PRNs through this SBAS satellite's MT1 mask.
     * The broadcast field is a 1-based PRN *mask number*, not a PRN (DO-229 Table A-5) —
     * applying it directly as a PRN put corrections of up to ±71 m on the wrong satellites.
     * Corrections whose IODP does not match the current mask (mask changeover in progress)
     * are dropped; non-GPS slots (GLONASS, GEO) are not consumed.
     */
    private fun storeSbasLongTermCorrections(
        sbasSvid: Int,
        corrections: List<SbasNavMessageParser.SbasLongTermCorrection>,
    ) {
        val mask = sbasPrnMasks[sbasSvid] ?: return
        for (ltc in corrections) {
            if (ltc.iodp != mask.iodp) continue
            val prn = mask.maskToPrn[ltc.prnMaskNumber - 1] ?: continue
            if (prn !in 1..37) continue // GPS slots only
            sbasLongTermCorrections[prn] = ltc
        }
    }

    /**
     * Decodes one GPS/QZSS L1 C/A subframe payload into the canonical 38-byte buffer
     * (10 × 30-bit words, data bits upright, parity bits as received), or null when the
     * subframe cannot be trusted.
     *
     * The D30* decision comes from the TLM **preamble**: 0x8B means the data bits are upright
     * (pre-decoded delivery, or raw with D30* = 0), its complement 0x74 means the previous
     * word's D30* complemented them; words 2–10 then chain off each previous raw word. The
     * previous subframe-ID heuristic accepted complemented subframes wholesale — IDs 2–5
     * invert to other *valid* IDs, so raw streams were stored mostly corrupted.
     *
     * When the HAL has not attested parity ([parityAttested] = false, i.e. STATUS_UNKNOWN),
     * every word must pass the IS-GPS-200 §20.3.5.2 Hamming check or the subframe is
     * rejected — unverifiable data must not become an ephemeris (the A-GNSS download covers
     * devices whose HALs deliver undecodable streams). Word 1's D29* is unknown across
     * message gaps; if no previous-word state is stored, both values are tried.
     */
    internal fun decodeGpsL1CaSubframe(
        satKey: Pair<Int, Int>,
        data: ByteArray,
        parityAttested: Boolean,
    ): ByteArray? {
        // AOSP HAL: GPS/QZSS L1CA = 40 bytes, 10×4-byte words,
        // 30-bit word right-justified in 32 bits (2 MSB padding = 0).
        // Older/non-standard HALs may deliver 38 bytes (contiguous 30-bit words).
        if (data.size < 38) return null
        val stride = if (data.size >= 40) 32 else 30
        val offset = if (stride == 32) 2 else 0
        val rawWords = IntArray(10) { GnssMathUtils.getBits(data, it * stride + offset, 30) }

        val initialD30 =
            when ((rawWords[0] ushr 22) and 0xFF) {
                TLM_PREAMBLE -> 0
                TLM_PREAMBLE_INVERTED -> 1
                else -> return null // not synchronized on a TLM word — corrupt payload
            }
        val storedLastBits = gpsLastBits[satKey]

        val subframe = ByteArray(38)
        var allParityValid = true
        var previousRaw = 0
        for (k in 0 until 10) {
            val raw = rawWords[k]
            val d30Star = if (k == 0) initialD30 else previousRaw and 1
            val d29StarCandidates =
                if (k == 0) {
                    if (storedLastBits != null) intArrayOf((storedLastBits shr 1) and 1) else intArrayOf(0, 1)
                } else {
                    intArrayOf((previousRaw shr 1) and 1)
                }
            if (d29StarCandidates.none { GpsNavMessageValidator.hasValidParity(raw, it, d30Star) }) {
                allParityValid = false
            }
            val decoded = if (d30Star != 0) GpsNavMessageValidator.getDecodedWord(raw, 1) else raw
            GnssMathUtils.packBits(subframe, k * 30, 30, decoded)
            previousRaw = raw
        }
        if (!parityAttested && !allParityValid) return null
        gpsLastBits[satKey] = rawWords[9] and 0x03

        val subframeId = GnssMathUtils.getBits(subframe, 49, 3)
        return if (subframeId in 1..5) subframe else null
    }

    private fun processGpsLikeL1CaNavigationMessage(
        satKey: Pair<Int, Int>,
        svid: Int,
        data: ByteArray,
        constellationType: Int,
        updateKlobucharCoefficients: Boolean,
        parityAttested: Boolean,
    ) {
        val decodedSubframe = decodeGpsL1CaSubframe(satKey, data, parityAttested)
        if (decodedSubframe == null) {
            Log.w(TAG, "GPS/QZSS nav msg svid=$svid rejected (preamble/parity/subframe-id) dataSize=${data.size} attested=$parityAttested")
            return
        }

        val subframeId = GnssMathUtils.getBits(decodedSubframe, 49, 3)
        Log.d(TAG, "GPS/QZSS nav msg svid=$svid subframeId=$subframeId dataSize=${data.size}")

        if (updateKlobucharCoefficients && subframeId == 4) {
            GpsNavMessageParser.parseKlobucharCoefficients(decodedSubframe)?.let { (alpha, beta) ->
                ionoAlpha = alpha
                ionoBeta = beta
            }
        }

        val frames = subframeStorage.getOrPut(satKey) { arrayOfNulls<ByteArray>(5) }
        frames[subframeId - 1] = decodedSubframe

        if (frames[0] != null && frames[1] != null && frames[2] != null) {
            val ephemeris = GpsNavMessageParser.parseGpsEphemeris(svid, frames[0]!!, frames[1]!!, frames[2]!!)
            if (ephemeris != null) {
                storeEphemeris(
                    satKey = satKey,
                    ephemeris = ephemeris.copy(constellationType = constellationType),
                    sourceLabel = if (constellationType == GnssStatus.CONSTELLATION_QZSS) "QZS_L1CA" else "GPS_L1CA",
                )
            } else {
                Log.w(TAG, "GPS/QZSS ephemeris parse returned null svid=$svid — subframe data may be corrupted")
            }
        }
    }

    private fun hasFreshEphemerisLocked(satKey: Pair<Int, Int>): Boolean = ephemerisMap[satKey]?.let(::isFreshEphemerisLocked) == true

    private fun isFreshEphemerisLocked(ephemeris: SatelliteEphemeris): Boolean = SystemClock.elapsedRealtime() - ephemeris.lastUpdateTime < GnssMathUtils.EPHEMERIS_STALENESS_MS

    /**
     * Transmit timing for an RTK observation. There is no receiver-clock estimate here, so the
     * rotation interval is PR/c + Δt_sv. The same Δt_sv is removed from the transmit epoch.
     */
    private data class RtkTiming(
        val timing: TransmitTiming,
        val satelliteClockSeconds: Double,
    )

    private fun rtkTransmitTiming(
        ephemeris: SatelliteEphemeris,
        receptionTimeSeconds: Double,
        pseudorangeMeters: Double,
    ): RtkTiming {
        val approxFlight =
            (pseudorangeMeters / SPEED_OF_LIGHT).coerceIn(MIN_TRAVEL_TIME_SECONDS, MAX_TRAVEL_TIME_SECONDS)
        val satelliteClockSeconds =
            SatellitePositionCalculator.getSatelliteClockCorrection(
                ephemeris,
                receptionTimeSeconds - approxFlight,
                includeGroupDelay = false,
            )
        return RtkTiming(
            timing =
                SatellitePositionCalculator.transmitTiming(
                    receptionTimeSeconds = receptionTimeSeconds,
                    pseudorangeMeters = pseudorangeMeters,
                    receiverClockBiasMeters = 0.0,
                    satelliteClockSeconds = satelliteClockSeconds,
                ),
            satelliteClockSeconds = satelliteClockSeconds,
        )
    }

    /**
     * Extended validity check for offline use. Keplerian records stay usable for 24 h after
     * receipt. Orbit error over that window is not ~2 m/hour: it is applied per satellite as
     * range variance from |t − toe| ([GnssMathUtils.broadcastEphemerisRangeSigmaMeters]).
     * GLONASS state-vector propagation is capped at 6 h of receipt and additionally by tb age.
     *
     * @param gpsTimeOfWeekSeconds current GPS TOW for data-validity (tb-age) checks;
     *   NaN restricts the check to receipt age (used by the logging summaries).
     */
    private fun isUsableEphemerisLocked(
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

    private data class ReceptionSatellite(
        val x: Double,
        val y: Double,
        val z: Double,
        val velocityX: Double,
        val velocityY: Double,
        val velocityZ: Double,
        val clockSeconds: Double,
        val clockDriftMetersPerSecond: Double,
    )

    private data class TdcpSignal(
        val frequencyHz: Double,
        val adrMeters: Double,
        val satX: Double,
        val satY: Double,
        val satZ: Double,
        val satClockSeconds: Double,
    )

    /** Satellite position and clock at transmit, rotated into the reception frame. */
    private fun receptionSatellite(
        key: Pair<Int, Int>,
        meas: SatelliteMeasurement,
        clockBiases: DoubleArray,
        constellationToIndex: Map<Int, Int>,
        gpstSeconds: Double,
    ): ReceptionSatellite? {
        val ephemeris = ephemerisMap[key]?.takeIf { isUsableEphemerisLocked(it, gpstSeconds) } ?: return null
        val timeSeconds =
            if (meas.constellationType == GnssStatus.CONSTELLATION_BEIDOU) {
                gpstSeconds - GnssMathUtils.BDT_GPST_OFFSET_SECONDS
            } else {
                gpstSeconds
            }
        val bias = clockBiases[constellationToIndex[meas.constellationType] ?: 0]
        val satelliteClock =
            SatellitePositionCalculator.getSatelliteClockCorrection(
                ephemeris,
                timeSeconds - meas.pseudorangeMeters / SPEED_OF_LIGHT,
                includeGroupDelay = false,
            )
        val timing =
            SatellitePositionCalculator.transmitTiming(
                receptionTimeSeconds = timeSeconds,
                pseudorangeMeters = meas.pseudorangeMeters,
                receiverClockBiasMeters = bias,
                satelliteClockSeconds = satelliteClock,
            )
        val transmit = SatellitePositionCalculator.calculateSatellitePosition(ephemeris, timing.transmitTimeSeconds)
        val transmitVelocity = SatellitePositionCalculator.satelliteEcefVelocity(ephemeris, timing.transmitTimeSeconds)
        val sat =
            SatellitePositionCalculator.rotateEcefForEarthRotationDuringTransit(
                transmit.first,
                transmit.second,
                transmit.third,
                timing.rotationSeconds,
            )
        val vel =
            SatellitePositionCalculator.rotateEcefForEarthRotationDuringTransit(
                transmitVelocity.first,
                transmitVelocity.second,
                transmitVelocity.third,
                timing.rotationSeconds,
            )
        return ReceptionSatellite(
            x = sat.first,
            y = sat.second,
            z = sat.third,
            velocityX = vel.first,
            velocityY = vel.second,
            velocityZ = vel.third,
            clockSeconds =
                SatellitePositionCalculator.getSatelliteClockCorrection(
                    ephemeris,
                    timing.transmitTimeSeconds,
                    includeGroupDelay = false,
                ),
            clockDriftMetersPerSecond =
                SatellitePositionCalculator.satelliteClockDriftMetersPerSecond(ephemeris, timing.transmitTimeSeconds),
        )
    }

    /**
     * One pseudorange rate per satellite. Null when fewer than five rates agree, which leaves
     * the inter-epoch filter on its position-only update.
     */
    private fun dopplerVelocity(
        measurements: List<SatelliteMeasurement>,
        receiverX: Double,
        receiverY: Double,
        receiverZ: Double,
        clockBiases: DoubleArray,
        constellationToIndex: Map<Int, Int>,
        gpstSeconds: Double,
    ): DopplerVelocitySolver.Solution? {
        val bestBySatellite = LinkedHashMap<Pair<Int, Int>, SatelliteMeasurement>()
        for (meas in measurements) {
            val rate = meas.pseudorangeRateMetersPerSecond
            val sigma = meas.pseudorangeRateUncertaintyMetersPerSecond
            if (!rate.isFinite() || !sigma.isFinite() || sigma <= 0.0 || sigma > MAX_PSEUDORANGE_RATE_SIGMA_MPS) continue
            val key = meas.constellationType to meas.svid
            val current = bestBySatellite[key]
            if (current == null || sigma < current.pseudorangeRateUncertaintyMetersPerSecond) {
                bestBySatellite[key] = meas
            }
        }
        val observations = ArrayList<DopplerVelocitySolver.Observation>(bestBySatellite.size)
        for ((key, meas) in bestBySatellite) {
            val sat = receptionSatellite(key, meas, clockBiases, constellationToIndex, gpstSeconds) ?: continue
            val dx = sat.x - receiverX
            val dy = sat.y - receiverY
            val dz = sat.z - receiverZ
            val range = sqrt(dx * dx + dy * dy + dz * dz)
            if (range < 1.0e6) continue
            observations +=
                DopplerVelocitySolver.Observation(
                    losX = dx / range,
                    losY = dy / range,
                    losZ = dz / range,
                    satelliteVelocityX = sat.velocityX,
                    satelliteVelocityY = sat.velocityY,
                    satelliteVelocityZ = sat.velocityZ,
                    satelliteClockDriftMetersPerSecond = sat.clockDriftMetersPerSecond,
                    pseudorangeRateMetersPerSecond = meas.pseudorangeRateMetersPerSecond,
                    sigmaMetersPerSecond = meas.pseudorangeRateUncertaintyMetersPerSecond.coerceAtLeast(0.05),
                )
        }
        return DopplerVelocitySolver.solve(observations)
    }

    /** Horizontal speed and bearing from the filter velocity after a Doppler update. */
    private fun speedAndBearingDegrees(
        latDeg: Double,
        lonDeg: Double,
    ): Pair<Double, Double> {
        val (vx, vy, vz) = interEpochKalman.velocityEcef()
        val (east, north, _) = GnssMathUtils.ecefDeltaToEnu(vx, vy, vz, latDeg, lonDeg)
        var bearing = Math.toDegrees(atan2(east, north))
        if (bearing < 0.0) bearing += 360.0
        return hypot(east, north) to bearing
    }

    /**
     * Carrier displacement from the previous published position. Null unless five slip-free
     * signals agree, which leaves the filter on the update it already has.
     */
    private fun tdcpDisplacement(
        measurements: List<SatelliteMeasurement>,
        clockBiases: DoubleArray,
        constellationToIndex: Map<Int, Int>,
        gpstSeconds: Double,
        gpsTimeNanos: Long,
    ): TdcpDisplacementSolver.Displacement? {
        val anchor = tdcpAnchor ?: return null
        val dtSeconds = (gpsTimeNanos - tdcpEpochNanos).toDouble() / GnssMathUtils.NANOS_PER_SECOND.toDouble()
        if (dtSeconds <= 0.0 || dtSeconds > GnssMathUtils.DOPPLER_MAX_DT_SECONDS) return null
        val satellites = ArrayList<TdcpDisplacementSolver.Satellite>()
        val seen = HashSet<Pair<Int, Int>>()
        for (meas in measurements) {
            val adr = meas.accumulatedDeltaRangeMeters ?: continue
            if (!hasValidAdr(meas.adrState) || isCarrierHalfCycleAmbiguous(meas.adrState)) continue
            val key = meas.constellationType to meas.svid
            if (!seen.add(key)) continue
            val previous = tdcpSignals[key] ?: continue
            if (abs(previous.frequencyHz - meas.carrierFrequencyHz) > 1_000.0) continue
            val agrees =
                carrierDeltaMatchesPseudorangeRate(
                    currentAdrMeters = adr,
                    previousAdrMeters = previous.adrMeters,
                    dtSeconds = dtSeconds,
                    rateMetersPerSecond = meas.pseudorangeRateMetersPerSecond,
                    rateUncertaintyMetersPerSecond = meas.pseudorangeRateUncertaintyMetersPerSecond,
                )
            if (agrees != true) continue
            val sat = receptionSatellite(key, meas, clockBiases, constellationToIndex, gpstSeconds) ?: continue
            val dx = sat.x - anchor.first
            val dy = sat.y - anchor.second
            val dz = sat.z - anchor.third
            val currentRange = sqrt(dx * dx + dy * dy + dz * dz)
            val previousDx = previous.satX - anchor.first
            val previousDy = previous.satY - anchor.second
            val previousDz = previous.satZ - anchor.third
            val previousRange = sqrt(previousDx * previousDx + previousDy * previousDy + previousDz * previousDz)
            if (currentRange < 1.0e6 || previousRange < 1.0e6) continue
            satellites +=
                TdcpDisplacementSolver.Satellite(
                    losX = dx / currentRange,
                    losY = dy / currentRange,
                    losZ = dz / currentRange,
                    previousRangeMeters = previousRange,
                    currentRangeMeters = currentRange,
                    previousAdrMeters = previous.adrMeters,
                    currentAdrMeters = adr,
                    previousSatelliteClockSeconds = previous.satClockSeconds,
                    currentSatelliteClockSeconds = sat.clockSeconds,
                )
        }
        return TdcpDisplacementSolver.solve(satellites, epochClockStepMeters)
    }

    private fun rememberTdcp(
        measurements: List<SatelliteMeasurement>,
        positionX: Double,
        positionY: Double,
        positionZ: Double,
        clockBiases: DoubleArray,
        constellationToIndex: Map<Int, Int>,
        gpstSeconds: Double,
        gpsTimeNanos: Long,
    ) {
        tdcpAnchor = Triple(positionX, positionY, positionZ)
        tdcpEpochNanos = gpsTimeNanos
        tdcpSignals.clear()
        for (meas in measurements) {
            val adr = meas.accumulatedDeltaRangeMeters ?: continue
            if (!hasValidAdr(meas.adrState) || isCarrierHalfCycleAmbiguous(meas.adrState)) continue
            val key = meas.constellationType to meas.svid
            if (tdcpSignals.containsKey(key) && meas.isL5) continue
            val sat = receptionSatellite(key, meas, clockBiases, constellationToIndex, gpstSeconds) ?: continue
            tdcpSignals[key] =
                TdcpSignal(
                    frequencyHz = meas.carrierFrequencyHz,
                    adrMeters = adr,
                    satX = sat.x,
                    satY = sat.y,
                    satZ = sat.z,
                    satClockSeconds = sat.clockSeconds,
                )
        }
    }

    private fun clearTdcp() {
        tdcpAnchor = null
        tdcpEpochNanos = 0L
        tdcpSignals.clear()
    }

    /** |t − toe| in seconds, wrapped to half a week. SBAS has no Keplerian toe. */
    private fun ephemerisAgeSeconds(
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

    private fun sanitizeObserverLatitudeDegrees(
        candidateLatitudeDegrees: Double,
        fallbackLatitudeDegrees: Double,
    ): Double {
        val fallback = if (fallbackLatitudeDegrees.isFinite()) fallbackLatitudeDegrees.coerceIn(-90.0, 90.0) else 0.0
        return if (candidateLatitudeDegrees.isFinite()) {
            candidateLatitudeDegrees.coerceIn(-90.0, 90.0)
        } else {
            fallback
        }
    }

    private fun sanitizeObserverLongitudeDegrees(
        candidateLongitudeDegrees: Double,
        fallbackLongitudeDegrees: Double,
    ): Double {
        val rawLongitude = if (candidateLongitudeDegrees.isFinite()) candidateLongitudeDegrees else fallbackLongitudeDegrees
        val normalizedLongitude = ((rawLongitude + 180.0) % 360.0 + 360.0) % 360.0 - 180.0
        return if (normalizedLongitude == -180.0) 180.0 else normalizedLongitude
    }

    private fun sanitizeObserverAltitudeMeters(
        candidateAltitudeMeters: Double,
        fallbackAltitudeMeters: Double,
    ): Double {
        val fallback = if (fallbackAltitudeMeters.isFinite()) fallbackAltitudeMeters.coerceIn(-500.0, 20_000.0) else 0.0
        return if (candidateAltitudeMeters.isFinite()) {
            candidateAltitudeMeters.coerceIn(-500.0, 20_000.0)
        } else {
            fallback
        }
    }

    /**
     * Process a GNSS measurements event and extract high-quality satellite data.
     *
     * @param selectedConstellationTypes Android `GnssStatus.CONSTELLATION_*` integers that
     *   the caller wants to include. Measurements whose `constellationType` is not in this
     *   set are dropped before any filtering / solving — saving work and preventing the
     *   disabled constellations from influencing the position, the satellite counts or the
     *   diagnostics. When `null`, no constellation-level filtering is applied (useful for
     *   tests and the legacy "process everything" behaviour).
     */
    fun processMeasurements(
        event: GnssMeasurementsEvent,
        gnssStatus: GnssStatus?,
        selectedConstellationTypes: Set<Int>? = null,
    ): ProcessedMeasurements =
        synchronized(lock) {
            val processedMeasurements = mutableListOf<SatelliteMeasurement>()
            val measurements = event.measurements
            val clock = event.clock

            // A hardware clock discontinuity means the receiver's timing solution jumped — a rebase,
            // a suspend, a leap-second step. The previous epoch's correction no longer describes
            // anything, so any carried smoother state is built on a difference that is not the one
            // being measured, and every signal would have to detect its own slip to escape it. Reset
            // once, here, on the platform's own signal, rather than letting each filter discover it.
            // The count is compared rather than its value used: the platform gives the number of
            // discontinuities since boot and has no sentinel for "none", so a change is the only
            // evidence available.
            if (lastHardwareClockDiscontinuityCount != null &&
                clock.hardwareClockDiscontinuityCount != lastHardwareClockDiscontinuityCount
            ) {
                hatchFilter.reset()
                dopplerSmoother.reset()
                lastFullBiasNanos = null
                clearTdcp()
                rtkAmbiguityMemory.clear()
            }
            lastHardwareClockDiscontinuityCount = clock.hardwareClockDiscontinuityCount

            // One step per epoch. fullBias is differenced in Long; adding it to biasNanos in Double
            // first would quantize the step onto a 256 ns grid before the smoothers ever see it.
            val hasFullBias = Build.VERSION.SDK_INT >= Build.VERSION_CODES.N && clock.hasFullBiasNanos()
            val biasNanos = if (hasFullBias && clock.hasBiasNanos()) clock.biasNanos else 0.0
            val clockStepMeters =
                if (hasFullBias) {
                    receiverClockStepMeters(lastFullBiasNanos, clock.fullBiasNanos, lastBiasNanos, biasNanos)
                } else {
                    0.0
                }
            epochClockStepMeters = clockStepMeters
            if (hasFullBias) {
                lastFullBiasNanos = clock.fullBiasNanos
                lastBiasNanos = biasNanos
            } else {
                lastFullBiasNanos = null
                lastBiasNanos = 0.0
            }

            hatchFilter.beginNewMeasurementsEpoch()
            dopplerSmoother.beginNewMeasurementsEpoch()

            // Compute GPS time from GnssClock fields (nanoseconds since GPS epoch)
            val gpsTimeNanos =
                if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N && clock.hasFullBiasNanos()) {
                    clock.timeNanos - clock.fullBiasNanos - (if (clock.hasBiasNanos()) clock.biasNanos.toLong() else 0L)
                } else {
                    // Fallback: approximate from system clock (UTC + ~18 leap seconds)
                    (System.currentTimeMillis() / 1000L + GnssMathUtils.GPS_UTC_LEAP_SECONDS) * GnssMathUtils.NANOS_PER_SECOND
                }

            // Cache GPS TOW for GLONASS tb conversion in processNavigationMessage
            lastGpsTimeOfWeekSeconds = (gpsTimeNanos / 1e9) % GnssMathUtils.SECONDS_IN_WEEK

            // GPS−UTC leap seconds: prefer the receiver-reported value (survives future leap
            // events); fall back to the compile-time constant.
            val epochLeapSeconds =
                if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N && clock.hasLeapSecond()) {
                    clock.leapSecond.toDouble()
                } else {
                    GnssMathUtils.GPS_UTC_LEAP_SECONDS.toDouble()
                }
            lastKnownLeapSeconds = epochLeapSeconds.toInt()

            val satelliteInfo = mutableMapOf<Pair<Int, Int>, Pair<Double, Double>>()
            gnssStatus?.let { status ->
                for (i in 0 until status.satelliteCount) {
                    val constellationType = status.getConstellationType(i)
                    val svid = status.getSvid(i)
                    val elevation = status.getElevationDegrees(i).toDouble()
                    val azimuth = status.getAzimuthDegrees(i).toDouble()
                    satelliteInfo[constellationType to svid] = elevation to azimuth
                }
            }

            for (measurement in measurements) {
                val constellationType = measurement.constellationType
                // Drop measurements from constellations the user has disabled in settings.
                if (selectedConstellationTypes != null &&
                    constellationType !in selectedConstellationTypes
                ) {
                    continue
                }
                // On dual-frequency (L1 + L5/E5) devices, SBAS is bypassed — its GEO ranging
                // signals offer no accuracy benefit over the L1/L5 iono-free combination and
                // processing them wastes battery.
                if (suppressSbas && constellationType == GnssStatus.CONSTELLATION_SBAS) {
                    continue
                }
                val svid = measurement.svid
                val satKey = constellationType to svid

                val statusAngles = satelliteInfo[satKey]
                val hasStatusElevation = statusAngles != null
                val (elevationRaw, azimuthRaw) = statusAngles ?: (0.0 to 0.0)
                // GnssStatus often lags the first measurement epochs; 0° elevation would drop all SVs.
                val elevationDegrees =
                    if (hasStatusElevation) {
                        elevationRaw
                    } else {
                        PLACEHOLDER_ELEVATION_WHEN_STATUS_UNKNOWN_DEG
                    }
                val azimuthDegrees = if (hasStatusElevation) azimuthRaw else 0.0

                // Lowered thresholds for better urban performance (weighting handles quality)
                if (measurement.cn0DbHz < 25.0) continue
                if (hasStatusElevation && elevationRaw < 15.0) continue

                val isL5 = measurement.carrierFrequencyHz < GnssMathUtils.L5_FREQUENCY_THRESHOLD_HZ

                if (Build.VERSION.SDK_INT < Build.VERSION_CODES.N || !clock.hasFullBiasNanos()) continue

                // Keep the large part of the receive time in Long: timeNanos − fullBiasNanos
                // ≈ 1.4e18 ns exceeds Double's 2^53 integer range (ULP ≈ 256 ns ≈ 77 m), so a
                // Double sum quantizes per-measurement timeOffsetNanos differences onto that
                // grid. The week/day modulo is taken in Long inside the seam and only the
                // small fractional terms are applied in Double afterwards.
                val receiveTimeWholeNanos = clock.timeNanos - clock.fullBiasNanos
                val receiveTimeFractionNanos =
                    measurement.timeOffsetNanos - (if (clock.hasBiasNanos()) clock.biasNanos else 0.0)

                val codeSigmaMeters =
                    (measurement.receivedSvTimeUncertaintyNanos * (SPEED_OF_LIGHT / 1e9))
                        .coerceIn(0.0, GnssMathUtils.MAX_PSEUDORANGE_SIGMA_METERS)
                val pseudorangeMeters =
                    computeRawPseudorangeMeters(
                        constellationType = constellationType,
                        measurementState = measurement.state,
                        receiveTimeWholeGpstNanos = receiveTimeWholeNanos,
                        receiveTimeFractionNanos = receiveTimeFractionNanos,
                        receivedSvTimeNanos = measurement.receivedSvTimeNanos,
                        leapSeconds = epochLeapSeconds,
                    ) ?: continue

                val adrMeters = measurement.accumulatedDeltaRangeMeters
                val adrState = measurement.accumulatedDeltaRangeState
                // Smoother state is per *signal*, not per satellite: L1 and L5 of one SV have
                // unrelated ADR origins and pseudorange baselines, so sharing one filter entry
                // mixes the bands (perpetual resets or meter-level cross-band bias).
                val signalKey =
                    Triple(
                        constellationType,
                        svid,
                        (measurement.carrierFrequencyHz / 1e6).toInt(),
                    )
                val smoothedPr =
                    when (smoothingMode) {
                        SmoothingMode.CARRIER_HATCH ->
                            hatchFilter.smooth(
                                signalKey = signalKey,
                                rawPseudorange = pseudorangeMeters,
                                currentAdr = adrMeters,
                                adrState = adrState,
                                // The platform's own range rate for this signal, in the same hardware
                                // clock domain as the carrier delta. The filter needs it to tell a
                                // cycle slip apart from a satellite simply flying past: a carrier
                                // delta of several hundred metres per second is ordinary, and the
                                // filter used to treat it as a slip.
                                epochTimeNanos = gpsTimeNanos,
                                pseudorangeRateMetersPerSecond = measurement.pseudorangeRateMetersPerSecond,
                                pseudorangeRateUncertaintyMetersPerSecond = measurement.pseudorangeRateUncertaintyMetersPerSecond,
                                // The pseudorange above is clock-corrected and the carrier is not, so
                                // the filter is told the correction's step or it will read the receiver's
                                // own refit as a carrier-versus-code disagreement and as a step it must
                                // propagate.
                                clockStepMeters = clockStepMeters,
                                codeSigmaMeters = codeSigmaMeters,
                            )
                        SmoothingMode.DOPPLER ->
                            dopplerSmoother.smooth(
                                signalKey = signalKey,
                                rawPseudorange = pseudorangeMeters,
                                pseudorangeRateMetersPerSecond = measurement.pseudorangeRateMetersPerSecond,
                                pseudorangeRateUncertaintyMetersPerSecond = measurement.pseudorangeRateUncertaintyMetersPerSecond,
                                epochTimeNanos = gpsTimeNanos,
                                // Same reason, same number: the prediction is built from an uncorrected
                                // rate and an already-corrected pseudorange, and without the step the two
                                // differ by a whole correction — which on a refitting receiver is 100 m,
                                // so this smoother was adding error rather than removing it.
                                clockStepMeters = clockStepMeters,
                            )
                    }

                val wavelength = SPEED_OF_LIGHT / measurement.carrierFrequencyHz
                val carrierPhaseMeters = if (hasValidAdr(adrState)) adrMeters else null

                processedMeasurements.add(
                    SatelliteMeasurement(
                        constellationType = constellationType,
                        svid = svid,
                        pseudorangeMeters = smoothedPr,
                        rawPseudorangeMeters = pseudorangeMeters,
                        carrierFrequencyHz = measurement.carrierFrequencyHz.toDouble(),
                        cn0DbHz = measurement.cn0DbHz,
                        elevationDegrees = elevationDegrees,
                        azimuthDegrees = azimuthDegrees,
                        accumulatedDeltaRangeMeters = measurement.accumulatedDeltaRangeMeters,
                        pseudorangeUncertaintyMeters = codeSigmaMeters,
                        isL5 = isL5,
                        constellationName = getConstellationName(constellationType),
                        wavelengthMeters = wavelength,
                        carrierPhaseMeters = carrierPhaseMeters,
                        adrState = adrState,
                        pseudorangeRateMetersPerSecond = measurement.pseudorangeRateMetersPerSecond,
                        pseudorangeRateUncertaintyMetersPerSecond = measurement.pseudorangeRateUncertaintyMetersPerSecond,
                    ),
                )

                // Critical PPP Fix: Handle Cycle Slips
                if ((adrState and GnssMeasurement.ADR_STATE_CYCLE_SLIP) != 0 ||
                    (adrState and GnssMeasurement.ADR_STATE_RESET) != 0
                ) {
                    Log.d(TAG, "Cycle Slip/Reset detected for SVID: ${measurement.svid} (Constellation: $constellationType). Resetting ambiguity.")
                    pppState.resetAmbiguity(satKey)
                }
            }
            return ProcessedMeasurements(processedMeasurements, gpsTimeNanos)
        }

    private fun hasValidAdr(state: Int): Boolean {
        return (state and GnssMeasurement.ADR_STATE_VALID) != 0 &&
            (state and GnssMeasurement.ADR_STATE_CYCLE_SLIP) == 0 &&
            (state and GnssMeasurement.ADR_STATE_RESET) == 0
    }

    /**
     * True when the chipset reports the carrier half-cycle ambiguity status AND it is not
     * resolved: the accumulated phase may be offset by λ/2, so the carrier must not feed
     * integer ambiguity fixing. Chipsets that don't report the status (HALF_CYCLE_REPORTED
     * unset, incl. pre-API-28 HALs that never set either bit) are accepted — rejecting them
     * would disable RTK fixing on the whole legacy fleet.
     */
    internal fun isCarrierHalfCycleAmbiguous(adrState: Int): Boolean =
        (adrState and GnssMeasurement.ADR_STATE_HALF_CYCLE_REPORTED) != 0 &&
            (adrState and GnssMeasurement.ADR_STATE_HALF_CYCLE_RESOLVED) == 0

    /**
     * Forms the raw pseudorange from receiver and satellite times, or returns null when the
     * measurement is unusable.
     *
     * Decoded satellite time is required: without it `receivedSvTimeNanos` may contain only a
     * sub-millisecond code-phase offset, making the pseudorange wrong by hundreds of km.
     * GLONASS HALs flag decoded time via the GLO-specific STATE_GLO_TOD_DECODED/KNOWN bits —
     * the STATE_TOW_* bits are never set for GLONASS per the GnssMeasurement contract, so
     * requiring them unconditionally silently dropped the whole constellation.
     *
     * Timescales: `receivedSvTimeNanos` is GPST time-of-week for GPS/QZSS/Galileo/SBAS, BDT
     * time-of-week for BeiDou (GPST − 14 s), and **GLONASS time-of-day** (UTC(SU) + 3 h) for
     * GLONASS. The GPST receiver time is shifted into the satellite timescale before the
     * modulo difference — without the GLONASS shift the difference is off by 3 h − leap ≈
     * 10,782 s and every GLONASS pseudorange came out negative and was culled.
     *
     * Precision: [receiveTimeWholeGpstNanos] must be the exact Long `timeNanos − fullBiasNanos`
     * (≈1.4e18 ns exceeds Double's 2^53 integer range; a Double sum quantizes onto a 256 ns ≈
     * 77 m grid, turning per-measurement timeOffsetNanos differences into differential range
     * error). The week/day modulo is taken in Long; only the small [receiveTimeFractionNanos]
     * (timeOffsetNanos − biasNanos) is applied in Double on the sub-week remainder.
     */
    internal fun computeRawPseudorangeMeters(
        constellationType: Int,
        measurementState: Int,
        receiveTimeWholeGpstNanos: Long,
        receiveTimeFractionNanos: Double,
        receivedSvTimeNanos: Long,
        leapSeconds: Double,
    ): Double? {
        val isGlonass = constellationType == GnssStatus.CONSTELLATION_GLONASS

        val towDecoded = (measurementState and GnssMeasurement.STATE_TOW_DECODED) != 0
        val towKnown =
            Build.VERSION.SDK_INT >= Build.VERSION_CODES.R &&
                (measurementState and GnssMeasurement.STATE_TOW_KNOWN) != 0
        val gloTodDecoded =
            isGlonass &&
                (
                    (measurementState and GnssMeasurement.STATE_GLO_TOD_DECODED) != 0 ||
                        (measurementState and GnssMeasurement.STATE_GLO_TOD_KNOWN) != 0
                )
        if (!towDecoded && !towKnown && !gloTodDecoded) return null

        val unitNanos = if (isGlonass) GnssMathUtils.NANOS_PER_DAY else GnssMathUtils.NANOS_PER_WEEK

        // Exact sub-unit remainder in Long (< 6.05e14 — exactly representable in Double),
        // then the small Double terms.
        var wholeModuloNanos = receiveTimeWholeGpstNanos % unitNanos
        if (wholeModuloNanos < 0) wholeModuloNanos += unitNanos
        val glonassShiftNanos =
            if (isGlonass) (GnssMathUtils.GLONASS_UTC_OFFSET_SECONDS - leapSeconds) * 1e9 else 0.0
        val receiverTimeInSvTimescaleNanos = wholeModuloNanos.toDouble() + receiveTimeFractionNanos + glonassShiftNanos

        // For BeiDou, Android delivers `receivedSvTimeNanos` in the BDT time system, which
        // lags GPS by 14 seconds; convert BDT → GPST by adding 14 s to the transmit time,
        // otherwise every BeiDou pseudorange is biased by 14 s × c ≈ 4.2e9 m.
        val bdtToGpstOffsetNanos =
            if (constellationType == GnssStatus.CONSTELLATION_BEIDOU) {
                GnssMathUtils.BDT_GPST_OFFSET_SECONDS * 1e9
            } else {
                0.0
            }
        val transmitTimeNanos = receivedSvTimeNanos.toDouble() + bdtToGpstOffsetNanos

        var prNanos = receiverTimeInSvTimescaleNanos - transmitTimeNanos
        if (prNanos < 0) prNanos += unitNanos
        if (prNanos > unitNanos / 2.0) prNanos -= unitNanos

        val pseudorangeMeters = (prNanos / 1e9) * SPEED_OF_LIGHT

        // Sanity check: valid pseudoranges span ~15,000 km (low MEO) to ~42,000 km
        // (SBAS/QZSS GEO at low elevation); anything outside indicates a decoding error.
        if (pseudorangeMeters <= 0.0 || pseudorangeMeters.isNaN() ||
            pseudorangeMeters < MIN_REASONABLE_PSEUDORANGE_METERS ||
            pseudorangeMeters > MAX_REASONABLE_PSEUDORANGE_METERS
        ) {
            return null
        }
        return pseudorangeMeters
    }

    fun computePosition(
        measurements: List<SatelliteMeasurement>,
        initialLat: Double = 0.0,
        initialLon: Double = 0.0,
        initialAlt: Double = 0.0,
        usePpp: Boolean = true,
        gpsTimeNanos: Long = 0L,
        accuracy: Accuracy = Accuracy.HIGH,
    ): PositionResult? =
        synchronized(lock) {
            if (measurements.size < 4) return null

            val groupedBySv = measurements.groupBy { it.constellationType to it.svid }
            // Use a mutable list that we can modify during RAIM iteration
            var activeMeasurements = mutableListOf<SatelliteMeasurement>()
            val ifCorrectedSatKeys = mutableSetOf<Pair<Int, Int>>()
            // IF-combined SVs whose satellite clock must EXCLUDE the broadcast group delay
            // (LNAV/BGD path: the per-signal biases cancel in the IF combination).
            val ifClockWithoutGroupDelaySatKeys = mutableSetOf<Pair<Int, Int>>()

            // Initial filtering and IF combination
            for ((satKey, svMeasurements) in groupedBySv) {
                val l1 = svMeasurements.find { !it.isL5 }
                val l5 = svMeasurements.find { it.isL5 }

                if (l1 != null && l5 != null) {
                    val f1 = l1.carrierFrequencyHz
                    val f5 = l5.carrierFrequencyHz

                    val ephemerisForIf =
                        ephemerisMap[satKey]?.takeIf { isUsableEphemerisLocked(it, (gpsTimeNanos / 1e9) % GnssMathUtils.SECONDS_IN_WEEK) }

                    val ifAdjustment =
                        codePseudorangesForIonosphereFreeMeters(
                            satKey = satKey,
                            l1Meters = l1.pseudorangeMeters,
                            l5Meters = l5.pseudorangeMeters,
                            ephemeris = ephemerisForIf,
                        )
                    val l1CodeForIf = ifAdjustment.l1Meters
                    val l5CodeForIf = ifAdjustment.l5Meters
                    if (!ifAdjustment.clockIncludesGroupDelay) {
                        ifClockWithoutGroupDelaySatKeys.add(satKey)
                    }

                    // Pseudorange IF Combination
                    val prIf = ((f1 * f1) * l1CodeForIf - (f5 * f5) * l5CodeForIf) / (f1 * f1 - f5 * f5)

                    // Carrier Phase IF Combination (if available)
                    val cpIf =
                        l1.carrierPhaseMeters?.let { cp1 ->
                            l5.carrierPhaseMeters?.let { cp5 ->
                                ((f1 * f1) * cp1 - (f5 * f5) * cp5) / (f1 * f1 - f5 * f5)
                            }
                        }

                    val f1Sq = f1 * f1
                    val f5Sq = f5 * f5
                    val denom = f1Sq - f5Sq
                    val sigma1 = l1.pseudorangeUncertaintyMeters
                    val sigma5 = l5.pseudorangeUncertaintyMeters
                    val sigmaIf = sqrt((f1Sq * f1Sq * sigma1 * sigma1 + f5Sq * f5Sq * sigma5 * sigma5) / (denom * denom))

                    activeMeasurements.add(
                        l1.copy(
                            pseudorangeMeters = prIf,
                            carrierPhaseMeters = cpIf,
                            pseudorangeUncertaintyMeters = sigmaIf,
                        ),
                    )
                    ifCorrectedSatKeys.add(satKey)
                } else {
                    activeMeasurements.add(l1 ?: svMeasurements.first())
                }
            }

            // RAIM Iteration Loop
            // We will try up to 3 times to remove outliers (or until we run out of satellites)
            // Max excluded satellites = 3 to avoid excessive computation
            var excludedCount = 0
            val maxExclusions = 3

            // Remove measurements that lack usable ephemeris — without satellite clock corrections
            // (~100–500 km per SV), these measurements destroy the WLS solution.
            // Uses extended validity (24h Keplerian / 6h GLONASS) so positioning works offline.
            val preEphemerisCount = activeMeasurements.size
            val nowTow = (gpsTimeNanos / 1e9) % GnssMathUtils.SECONDS_IN_WEEK
            activeMeasurements.removeAll { measurement ->
                val satKey = measurement.constellationType to measurement.svid
                ephemerisMap[satKey]?.let { isUsableEphemerisLocked(it, nowTow) } != true
            }
            if (activeMeasurements.size < preEphemerisCount) {
                Log.d(TAG, "Filtered ${preEphemerisCount - activeMeasurements.size} measurements without ephemeris, kept ${activeMeasurements.size}")
            }
            if (activeMeasurements.size < 4) {
                Log.d(TAG, "Not enough measurements with ephemeris (${activeMeasurements.size}/4 minimum) — cannot solve")
                return null
            }

            // RAIM may strip SVs across iterations; constellation set is recomputed inside the
            // loop. The exclusion budget (maxExclusions) is enforced in the RAIM branch below,
            // so the loop is bounded solely by having enough satellites to solve.
            while (activeMeasurements.size >= 4) {
                // --- Standard WLS / PPP Logic Start ---

                // All SVs now have ephemeris (approximate-geometry-only SVs were filtered out above)
                val maxWlsIterations = GnssMathUtils.MAX_WLS_ITERATIONS
                val positionConvergenceThresholdMeters = GnssMathUtils.CONVERGENCE_THRESHOLD_METERS
                val clockConvergenceThresholdMeters = GnssMathUtils.CONVERGENCE_THRESHOLD_METERS * GnssMathUtils.SPEED_OF_LIGHT / 1e6
                val raimResidualThresholdMeters = GnssMathUtils.RAIM_RESIDUAL_THRESHOLD_METERS
                val minHorizontalAccuracyMeters = GnssMathUtils.MIN_HORIZONTAL_ACCURACY
                val maxHorizontalAccuracyMeters = GnssMathUtils.MAX_HORIZONTAL_ACCURACY

                // Day-of-year (1..366 GPST-based) used for Niell seasonal amplitude in the
                // hydrostatic mapping function. Derive from GPS epoch so troposphere modelling
                // stays aligned across hemispheres and seasons.
                val dayOfYear: Int =
                    run {
                        val unixSeconds = GnssMathUtils.GPS_EPOCH_UNIX_SECONDS + (gpsTimeNanos / 1_000_000_000L)
                        val utcCalendar = java.util.Calendar.getInstance(java.util.TimeZone.getTimeZone("UTC"))
                        utcCalendar.timeInMillis = unixSeconds * 1000L
                        utcCalendar.get(java.util.Calendar.DAY_OF_YEAR)
                    }

                var (currentX, currentY, currentZ) =
                    if (usePpp) {
                        pppState.getLastPositionEcef() ?: GnssMathUtils.latLonAltToEcef(initialLat, initialLon, initialAlt)
                    } else {
                        GnssMathUtils.latLonAltToEcef(initialLat, initialLon, initialAlt)
                    }

                // Handle multi-constellation clock biases (ISB)
                val constellationTypes = activeMeasurements.map { it.constellationType }.distinct().sorted()
                val numConstellations = constellationTypes.size
                val constellationToIndex = constellationTypes.withIndex().associate { it.value to it.index }

                val clockBiases = DoubleArray(numConstellations) { 0.0 }
                var lastResidualsRms = 0.0
                // A-posteriori variance-of-unit-weight (reduced chi-square), dimensionless. Scales the
                // metric formal covariance for honest accuracy/Kalman-R reporting; 1.0 = fit matches
                // the a-priori measurement model. Floored at 1.0 so we never report better than formal.
                var lastVarianceFactor = 1.0

                var maxResidual = 0.0
                var maxResidualIdx = -1
                var maxStandardizedResidual = 0.0
                var maxStandardizedResidualIdx = -1

                // WLS Iterations
                var wlsConverged = false
                var designMatrix: Array<DoubleArray>? = null
                var observationVector: DoubleArray? = null
                var solutionWrapper: GnssPositionSolver.WlsSolution? = null
                // Unweighted geometry normal-matrix inverse for true (dimensionless) DOPs.
                var dopMatrix: Array<DoubleArray>? = null

                // Per-SV cache of satellite position / clock and atmospheric model terms, reused across
                // iterations while the receiver estimate barely moves. Scoped to this RAIM pass so a
                // re-solve after an outlier exclusion starts clean.
                val svModelCache = HashMap<Pair<Int, Int>, SvModelTerms>()

                for (iter in 0 until maxWlsIterations) {
                    val (currentLat, currentLon, currentAlt) = GnssMathUtils.ecefToLatLonAlt(currentX, currentY, currentZ)
                    val observerLat = sanitizeObserverLatitudeDegrees(currentLat, initialLat)
                    val observerLon = sanitizeObserverLongitudeDegrees(currentLon, initialLon)
                    val observerAlt = sanitizeObserverAltitudeMeters(currentAlt, initialAlt)

                    // Single-epoch phase rows each own a free ambiguity, so they do not inform
                    // position, and a free zenith wet delay then inflates height and clock
                    // (review #85). The troposphere stays fully modelled. Sequential PPP is
                    // what would make the phase informative.
                    val n = activeMeasurements.size
                    val m = 3 + numConstellations

                    designMatrix = Array(n) { DoubleArray(m) }
                    observationVector = DoubleArray(n)
                    val weights = DoubleArray(n)
                    // Per-row a-priori measurement sigma (σ_i). Parallel to [weights]; used by
                    // the standardized-residual RAIM check after the WLS solve converges.
                    val measurementSigmas = DoubleArray(n)

                    for (i in activeMeasurements.indices) {
                        val meas = activeMeasurements[i]
                        val satKey = meas.constellationType to meas.svid
                        val ephemeris =
                            ephemerisMap[satKey]?.takeIf { isUsableEphemerisLocked(it, (gpsTimeNanos / 1e9) % GnssMathUtils.SECONDS_IN_WEEK) }

                        val gpstSeconds = (gpsTimeNanos / 1e9) % GnssMathUtils.SECONDS_IN_WEEK
                        val currentTimeSeconds = if (meas.constellationType == GnssStatus.CONSTELLATION_BEIDOU) gpstSeconds - GnssMathUtils.BDT_GPST_OFFSET_SECONDS else gpstSeconds

                        val currentConstellationBias =
                            clockBiases[constellationToIndex[meas.constellationType] ?: 0]

                        // Per-SV model terms (satellite ECEF + clock, ionosphere, troposphere) depend only
                        // on the ephemeris/clock-bias and the observer geometry, which converge within the
                        // first WLS iterations. They are insensitive to sub-metre changes in the receiver
                        // estimate (the satellite position does not depend on it at all; iono/tropo gradients
                        // are < 1 mm per metre), so once the receiver settles within
                        // WLS_MODEL_REUSE_DISTANCE_METERS of where they were last evaluated we reuse them —
                        // skipping the per-iteration Kepler solve and Klobuchar evaluation. The
                        // line-of-sight, range, relativistic (Shapiro) and residual below are still rebuilt
                        // every iteration from the current receiver position.
                        val cachedModel = if (ephemeris != null) svModelCache[satKey] else null
                        val reuseModel =
                            cachedModel != null &&
                                abs(currentX - cachedModel.refX) <= GnssMathUtils.WLS_MODEL_REUSE_DISTANCE_METERS &&
                                abs(currentY - cachedModel.refY) <= GnssMathUtils.WLS_MODEL_REUSE_DISTANCE_METERS &&
                                abs(currentZ - cachedModel.refZ) <= GnssMathUtils.WLS_MODEL_REUSE_DISTANCE_METERS &&
                                abs(currentConstellationBias - cachedModel.refClockBias) <= GnssMathUtils.WLS_MODEL_REUSE_DISTANCE_METERS

                        val satX: Double
                        val satY: Double
                        val satZ: Double
                        val iterElevationDeg: Double
                        val iterAzimuthDeg: Double
                        val dryDelay: Double
                        val ionoDelayL1: Double
                        val satClockCorr: Double

                        if (reuseModel && cachedModel != null) {
                            satX = cachedModel.satX
                            satY = cachedModel.satY
                            satZ = cachedModel.satZ
                            iterElevationDeg = cachedModel.elevationDeg
                            iterAzimuthDeg = cachedModel.azimuthDeg
                            dryDelay = cachedModel.dryDelay
                            ionoDelayL1 = cachedModel.ionoDelayL1
                            satClockCorr = cachedModel.satClockMeters
                        } else {
                            // The pseudorange and currentTimeSeconds share the receiver clock, so it
                            // cancels in the transmit epoch (t = t_rx − PR/c − Δt_sv). The flight
                            // time that rotates ECEF keeps the estimated clock: (PR − b̂)/c + Δt_sv.
                            // One clock evaluation at t_rx − PR/c is enough; af1 across Δt_sv is noise.
                            val transmitTiming =
                                if (ephemeris != null) {
                                    val approxFlight =
                                        (meas.pseudorangeMeters / SPEED_OF_LIGHT)
                                            .coerceIn(MIN_TRAVEL_TIME_SECONDS, MAX_TRAVEL_TIME_SECONDS)
                                    val satelliteClockSeconds =
                                        SatellitePositionCalculator.getSatelliteClockCorrection(
                                            ephemeris,
                                            currentTimeSeconds - approxFlight,
                                            includeGroupDelay = false,
                                        )
                                    SatellitePositionCalculator.transmitTiming(
                                        receptionTimeSeconds = currentTimeSeconds,
                                        pseudorangeMeters = meas.pseudorangeMeters,
                                        receiverClockBiasMeters = currentConstellationBias,
                                        satelliteClockSeconds = satelliteClockSeconds,
                                    )
                                } else {
                                    null
                                }
                            val (computedSatX, computedSatY, computedSatZ) =
                                if (ephemeris != null) {
                                    val travelTime =
                                        (transmitTiming?.rotationSeconds ?: 0.0)
                                            .coerceIn(MIN_TRAVEL_TIME_SECONDS, MAX_TRAVEL_TIME_SECONDS)
                                    val txTime = transmitTiming?.transmitTimeSeconds ?: currentTimeSeconds
                                    val (tx, ty, tz) =
                                        SatellitePositionCalculator.calculateSatellitePosition(
                                            ephemeris,
                                            txTime,
                                        )
                                    val (rx, ry, rz) =
                                        SatellitePositionCalculator.rotateEcefForEarthRotationDuringTransit(
                                            transmitX = tx,
                                            transmitY = ty,
                                            transmitZ = tz,
                                            transitTimeSeconds = travelTime,
                                        )
                                    // Apply SSR orbit correction if available. Gated on the broadcast
                                    // ephemeris IOD: SSR deltas reference one specific broadcast issue,
                                    // and applying them across an ephemeris changeover injects the
                                    // meters-level difference between the two issues.
                                    val ssrOrbit = getApplicableSsrOrbit(meas.constellationType, meas.svid, ephemeris)
                                    if (ssrOrbit != null) {
                                        val vel =
                                            SsrCorrectionApplicator.computeSatelliteVelocity(
                                                ephemeris,
                                                txTime,
                                                Triple(tx, ty, tz),
                                                SatellitePositionCalculator::calculateSatellitePosition,
                                            )
                                        val dtSsr =
                                            SsrCorrectionApplicator.ssrPolynomialAgeSeconds(
                                                gpstSeconds,
                                                ssrOrbit.epochTime,
                                                ssrOrbit.updateIntervalSeconds,
                                            )
                                        val beidouGeo =
                                            meas.constellationType == GnssStatus.CONSTELLATION_BEIDOU &&
                                                (meas.svid in 1..5 || meas.svid in 59..63)
                                        SsrCorrectionApplicator.applyOrbitCorrection(
                                            rx,
                                            ry,
                                            rz,
                                            vel.first,
                                            vel.second,
                                            vel.third,
                                            ssrOrbit.deltaRadial + ssrOrbit.dotRadial * dtSsr,
                                            ssrOrbit.deltaAlongTrack + ssrOrbit.dotAlongTrack * dtSsr,
                                            ssrOrbit.deltaCrossTrack + ssrOrbit.dotCrossTrack * dtSsr,
                                            nonRotatingVelocity = beidouGeo,
                                        )
                                    } else {
                                        // Apply SBAS long-term orbit correction if available
                                        // (GPS only, IODE-matched to this broadcast ephemeris)
                                        val sbasLtc =
                                            if (meas.constellationType == GnssStatus.CONSTELLATION_GPS) {
                                                getSbasLongTermCorrection(meas.svid, ephemeris.iode)
                                            } else {
                                                null
                                            }
                                        if (sbasLtc != null) {
                                            Triple(rx + sbasLtc.deltaX, ry + sbasLtc.deltaY, rz + sbasLtc.deltaZ)
                                        } else {
                                            Triple(rx, ry, rz)
                                        }
                                    }
                                } else {
                                    // No ephemeris — use approximate satellite position from the
                                    // fixed seed (not the WLS iterate) to avoid circular dependency.
                                    SatellitePositionCalculator.getSatellitePositionApproximation(
                                        constellationType = meas.constellationType,
                                        elevation = meas.elevationDegrees,
                                        azimuth = meas.azimuthDegrees,
                                        observerLat = initialLat,
                                        observerLon = initialLon,
                                        observerAlt = initialAlt,
                                    )
                                }
                            satX = computedSatX
                            satY = computedSatY
                            satZ = computedSatZ

                            val (computedElevationDeg, computedAzimuthDeg) =
                                GnssMathUtils.ecefLosToElevationAzimuth(
                                    receiverX = currentX,
                                    receiverY = currentY,
                                    receiverZ = currentZ,
                                    receiverLatDeg = currentLat,
                                    receiverLonDeg = currentLon,
                                    satelliteX = satX,
                                    satelliteY = satY,
                                    satelliteZ = satZ,
                                )
                            iterElevationDeg = computedElevationDeg
                            iterAzimuthDeg = computedAzimuthDeg

                            dryDelay =
                                AtmosphericCorrection.computeTroposphericDelay(
                                    iterElevationDeg,
                                    observerAlt,
                                    observerLat,
                                    dayOfYear,
                                )

                            // Single-frequency ionosphere: Klobuchar with the broadcast α/β (and the
                            // geomagnetic-latitude step), scaled per carrier below. Galileo SVs use
                            // Klobuchar too: the previous "NeQuick-G" implementation substituted an
                            // invented climatology for the CCIR maps/MODIP grid the broadcast aᵢ
                            // coefficients are fitted against, making its output unvalidated and
                            // regionally worse than Klobuchar — a real NeQuick-G needs those tables.
                            ionoDelayL1 =
                                if (ifCorrectedSatKeys.contains(satKey)) {
                                    0.0
                                } else {
                                    AtmosphericCorrection.computeIonosphericDelay(iterElevationDeg, iterAzimuthDeg, observerLat, observerLon, gpsTimeNanos / 1_000_000L, ionoAlpha, ionoBeta)
                                }

                            satClockCorr =
                                if (ephemeris != null) {
                                    val clockTimeSec = transmitTiming?.transmitTimeSeconds ?: currentTimeSeconds
                                    // Satellite clock WITHOUT the broadcast group delay; the
                                    // signal-specific term is applied below. GPS/BeiDou: none on the
                                    // LNAV ionosphere-free path, plain TGD for CNAV/BeiDou, γ·TGD
                                    // on one frequency. Galileo (OS SIS ICD §5.1.5): the clock's own
                                    // BGD on E1, and for an I/NAV clock the E5a and E1/E5a formulas
                                    // that convert it onto the F/NAV reference.
                                    var clockMeters =
                                        SatellitePositionCalculator.getSatelliteClockCorrection(
                                            ephemeris,
                                            clockTimeSec,
                                            includeGroupDelay = false,
                                        ) * SPEED_OF_LIGHT
                                    val groupDelaySeconds =
                                        groupDelaySecondsForMeasurement(
                                            ephemeris = ephemeris,
                                            constellationType = meas.constellationType,
                                            carrierFrequencyHz = meas.carrierFrequencyHz,
                                            ionosphereFree = satKey in ifCorrectedSatKeys,
                                            clockExcludesGroupDelay = satKey in ifClockWithoutGroupDelaySatKeys,
                                        )
                                    if (groupDelaySeconds.isFinite() && groupDelaySeconds != 0.0) {
                                        clockMeters -= groupDelaySeconds * SPEED_OF_LIGHT
                                    }
                                    // Apply SSR clock correction if available (IOD-gated like the orbit)
                                    val ssrClock = getApplicableSsrClock(meas.constellationType, meas.svid, ephemeris)
                                    if (ssrClock != null) {
                                        val dtSsr =
                                            SsrCorrectionApplicator.ssrPolynomialAgeSeconds(
                                                gpstSeconds,
                                                ssrClock.epochTime,
                                                ssrClock.updateIntervalSeconds,
                                            )
                                        clockMeters = SsrCorrectionApplicator.applyClockCorrection(clockMeters, ssrClock.c0, ssrClock.c1, ssrClock.c2, dtSsr)
                                    } else {
                                        // SBAS fast (PRC) and long-term clock corrections are an
                                        // *alternative* to SSR, not additive: both correct the same
                                        // broadcast clock error, so stacking them on an SSR clock
                                        // would double-correct (mirrors the orbit-side precedence).
                                        clockMeters += getSbasFastCorrectionMeters(meas.constellationType, meas.svid)
                                        clockMeters += getSbasLongTermClockCorrectionMeters(meas.constellationType, meas.svid, ephemeris.iode)
                                    }
                                    clockMeters
                                } else {
                                    0.0
                                }

                            if (ephemeris != null) {
                                svModelCache[satKey] =
                                    SvModelTerms(
                                        refX = currentX,
                                        refY = currentY,
                                        refZ = currentZ,
                                        refClockBias = currentConstellationBias,
                                        satX = satX,
                                        satY = satY,
                                        satZ = satZ,
                                        satClockMeters = satClockCorr,
                                        ionoDelayL1 = ionoDelayL1,
                                        dryDelay = dryDelay,
                                        elevationDeg = iterElevationDeg,
                                        azimuthDeg = iterAzimuthDeg,
                                    )
                            }
                        }

                        val range = sqrt((satX - currentX).pow(2) + (satY - currentY).pow(2) + (satZ - currentZ).pow(2))
                        if (range < 1e-6) continue

                        val dx = (satX - currentX) / range
                        val dy = (satY - currentY) / range
                        val dz = (satZ - currentZ) / range

                        // Code observation: partial derivatives of PR w.r.t. receiver position
                        // dPR/d(rec) = -(sat - rec)/range, so negate the unit LOS vector
                        designMatrix[i][0] = -dx
                        designMatrix[i][1] = -dy
                        designMatrix[i][2] = -dz
                        val constellationIdx = constellationToIndex[meas.constellationType] ?: 0
                        designMatrix[i][3 + constellationIdx] = 1.0

                        // Klobuchar returns the slant delay at the GPS L1 frequency. Scale to the
                        // measurement's actual carrier by (f_L1/f)² so that single-frequency non-L1
                        // SVs (L5/E5a-only, or GLONASS L1) are corrected at the right magnitude.
                        // L1/E1 scales by about 1. IF-combined SVs have ionoDelayL1 = 0.
                        val ionoDelay =
                            if (meas.carrierFrequencyHz > 0.0) {
                                ionoDelayL1 * (GnssMathUtils.L1_FREQUENCY_HZ / meas.carrierFrequencyHz).pow(2)
                            } else {
                                ionoDelayL1
                            }
                        val relCorr =
                            AtmosphericCorrection.computeRelativisticCorrection(
                                receiverX = currentX,
                                receiverY = currentY,
                                receiverZ = currentZ,
                                satelliteX = satX,
                                satelliteY = satY,
                                satelliteZ = satZ,
                            )

                        val currentClockBias = currentConstellationBias
                        // Observation residual: PR + satClockCorr - range - sagnac - clockBias - tropo - iono - shapiro
                        // Shapiro delay increases measured PR, so subtract it
                        val pMeasured = meas.pseudorangeMeters + satClockCorr - range - currentClockBias - dryDelay - ionoDelay - relCorr
                        observationVector[i] = pMeasured

                        // Observation variance model: combine Android's `receivedSvTimeUncertaintyNanos × c`
                        // (a *time-tagging* floor, typically <1 m) with the physical elevation-/C/N₀-dependent
                        // code noise model σ² = (a² + b²/sin²el) · 10^(−(CN₀−CN₀_ref)/10). This replaces the
                        // previous product of (1/σ_Android²) · sin²(el) · 10^((CN₀−CN₀_ref)/10), which double-
                        // applied an elevation-weighting term while leaving multipath noise un-modelled.
                        val sigmaAndroid =
                            meas.pseudorangeUncertaintyMeters.coerceIn(
                                GnssMathUtils.MIN_PSEUDORANGE_SIGMA_METERS,
                                GnssMathUtils.MAX_PSEUDORANGE_SIGMA_METERS,
                            )
                        val sigmaModel = GnssMathUtils.pseudorangeSigmaFromElevationAndCn0(iterElevationDeg, meas.cn0DbHz)
                        val sigmaCode =
                            max(sigmaAndroid, sigmaModel).coerceIn(
                                GnssMathUtils.MIN_PSEUDORANGE_SIGMA_METERS,
                                GnssMathUtils.MAX_PSEUDORANGE_SIGMA_METERS,
                            )
                        val sigma =
                            GnssMathUtils.observationSigmaMeters(
                                sigmaCode,
                                if (ephemeris == null) 0.0 else ephemerisAgeSeconds(ephemeris, currentTimeSeconds),
                            )
                        measurementSigmas[i] = sigma
                        weights[i] = 1.0 / sigma.pow(2)
                    }

                    // Intermediate iterations only need the state update (deltaX); the covariance is
                    // computed once at convergence (below). Skipping it here avoids an O(m³) inverse
                    // on every iteration.
                    val solution = GnssPositionSolver.solve(designMatrix, observationVector, weights, accuracy = accuracy, needCovariance = false) ?: break // Matrix singular?
                    val deltaX = solution.deltaX

                    currentX += deltaX[0]
                    currentY += deltaX[1]
                    currentZ += deltaX[2]

                    for (j in 0 until numConstellations) {
                        clockBiases[j] += deltaX[3 + j]
                    }

                    val posDelta = sqrt(deltaX[0].pow(2) + deltaX[1].pow(2) + deltaX[2].pow(2))
                    val maxClockDelta = (3 until deltaX.size.coerceAtMost(3 + numConstellations)).maxOfOrNull { abs(deltaX[it]) } ?: 0.0
                    val converged = posDelta < positionConvergenceThresholdMeters && maxClockDelta < clockConvergenceThresholdMeters

                    if (!converged) {
                        if (iter == 0 || iter == maxWlsIterations - 1) {
                            val ephCount = activeMeasurements.count { m -> ephemerisMap[m.constellationType to m.svid]?.let(::isFreshEphemerisLocked) == true }
                            Log.d(
                                TAG,
                                "WLS iter=$iter posDelta=${"%.1f".format(posDelta)}m clockDelta=${"%.1f".format(maxClockDelta)}m sats=${activeMeasurements.size} ephSats=$ephCount",
                            )
                        }
                        continue
                    }

                    wlsConverged = true

                    // Converged: now compute the covariance once for DOP / accuracy / the Kalman
                    // measurement noise, and the standardized-residual RAIM gate. Re-solving the same
                    // (geometry-converged) system with the covariance is one extra factorization per
                    // accepted fix, far cheaper than computing it on every iteration.
                    val convergedSolution = GnssPositionSolver.solve(designMatrix, observationVector, weights, accuracy = accuracy, needCovariance = true) ?: break
                    solutionWrapper = convergedSolution

                    // DOP is defined on the UNWEIGHTED geometry: Q = (G^T G)^-1 over the code
                    // rows' position+clock columns. The weighted covariance (1/sigma^2 weights)
                    // is metric (m^2) — reporting its diagonals as "DOP" published sigma*DOP
                    // values (and arbitrary ones once phase rows entered the normal matrix).
                    dopMatrix =
                        run {
                            val mGeo = 3 + numConstellations
                            val normal = Array(mGeo) { DoubleArray(mGeo) }
                            for (i in 0 until activeMeasurements.size) {
                                if (weights[i] <= 0.0) continue
                                for (r in 0 until mGeo) {
                                    val gr = designMatrix[i][r]
                                    if (gr == 0.0) continue
                                    for (c in r until mGeo) {
                                        normal[r][c] += gr * designMatrix[i][c]
                                    }
                                }
                            }
                            for (r in 0 until mGeo) {
                                for (c in 0 until r) normal[r][c] = normal[c][r]
                            }
                            GnssMathUtils.invertSquareMatrix(normal)
                        }

                    // Post-fit residuals: residual = b − A · deltaX (observation mismatch after correction).
                    var sumSqResiduals = 0.0
                    var validCodeRows = 0
                    maxResidual = 0.0
                    maxResidualIdx = -1
                    maxStandardizedResidual = 0.0
                    maxStandardizedResidualIdx = -1

                    // Standardized residual uses the leverage h_ii of each code row.
                    //   h_ii = w_i · aᵢᵀ Q aᵢ,   Q = (AᵀWA)⁻¹  (convergedSolution.covarianceMatrix)
                    // z_i = |r_i| / (σ_i · √(1 − h_ii))
                    // This auto-scales with each SV's expected noise and its geometric redundancy —
                    // replacing the old fixed RAIM_RESIDUAL_THRESHOLD_METERS = 8 m absolute gate.
                    val qCov = convergedSolution.covarianceMatrix
                    for (i in 0 until activeMeasurements.size) {
                        if (weights[i] <= 0.0) continue
                        validCodeRows++
                        var predicted = 0.0
                        for (j in 0 until m) {
                            predicted += designMatrix[i][j] * deltaX[j]
                        }
                        val res = abs(observationVector[i] - predicted)
                        sumSqResiduals += res.pow(2)

                        if (res > maxResidual) {
                            maxResidual = res
                            maxResidualIdx = i
                        }

                        var aQa = 0.0
                        for (j in 0 until m) {
                            var qAi = 0.0
                            for (k in 0 until m) {
                                qAi += qCov[j][k] * designMatrix[i][k]
                            }
                            aQa += designMatrix[i][j] * qAi
                        }
                        val hii = (weights[i] * aQa).coerceIn(0.0, GnssMathUtils.RAIM_LEVERAGE_MAX)
                        val redundancy = sqrt((1.0 - hii).coerceAtLeast(1e-6))
                        val sigmaI = measurementSigmas[i].coerceAtLeast(GnssMathUtils.MIN_PSEUDORANGE_SIGMA_METERS)
                        val zi = res / (sigmaI * redundancy)
                        if (zi > maxStandardizedResidual) {
                            maxStandardizedResidual = zi
                            maxStandardizedResidualIdx = i
                        }
                    }
                    lastResidualsRms = if (validCodeRows > 0) sqrt(sumSqResiduals / validCodeRows) else 0.0

                    // A-posteriori variance factor (reduced chi-square). The covariance is metric
                    // (weights are 1/σ²), so the reported accuracy is sqrt(factor) × formal-sigma:
                    // factor ≈ 1 when the fit matches the noise model, > 1 when residuals exceed it.
                    //
                    // The residual sum and the degrees of freedom must refer to the SAME system:
                    // all n rows (code + phase) against dof = n − m. Dividing the code-only sum by
                    // validCodeRows − m pinned the factor at 1.0 whenever phase rows were present
                    // (m includes the ZWD and one ambiguity per phase row, so validCodeRows − m ≤ 0
                    // in the default PPP mode) — reported accuracy never inflated on bad epochs and
                    // the Kalman measurement covariance under-weighted them.
                    var sumWeightedSqResidualsAllRows = 0.0
                    var validRowsAll = 0
                    for (i in 0 until n) {
                        if (weights[i] <= 0.0) continue
                        validRowsAll++
                        var predicted = 0.0
                        for (j in 0 until m) {
                            predicted += designMatrix[i][j] * deltaX[j]
                        }
                        val res = observationVector[i] - predicted
                        sumWeightedSqResidualsAllRows += weights[i] * res * res
                    }
                    val redundancyAllRows = validRowsAll - m
                    lastVarianceFactor =
                        if (redundancyAllRows > 0) {
                            (sumWeightedSqResidualsAllRows / redundancyAllRows).coerceAtLeast(1.0)
                        } else {
                            1.0
                        }

                    val ephCount = activeMeasurements.count { m -> ephemerisMap[m.constellationType to m.svid]?.let(::isFreshEphemerisLocked) == true }
                    Log.d(
                        TAG,
                        "WLS iter=$iter posDelta=${"%.1f".format(
                            posDelta,
                        )}m clockDelta=${"%.1f".format(
                            maxClockDelta,
                        )}m rms=${"%.1f".format(lastResidualsRms)}m maxRes=${"%.1f".format(maxResidual)}m sats=${activeMeasurements.size} ephSats=$ephCount",
                    )
                    break
                } // End WLS Loop

                if (wlsConverged && solutionWrapper != null) {
                    // RAIM Check — **normalized** (standardized) post-fit residuals.
                    //
                    // Prefer the standardized residual z_i when leverages are computable (covariance
                    // available): a SV is rejected when z_i = |r_i| / (σ_i · √(1 − h_ii)) exceeds
                    // RAIM_STANDARDIZED_RESIDUAL_SIGMA. This correctly down-weights low-elevation SVs
                    // (which have larger σ_i) and penalises high-leverage observations (small
                    // √(1−h_ii)). Fall back to the raw metric gate only when we could not compute
                    // standardized residuals (e.g. degenerate covariance).
                    val standardizedRaimAvailable = maxStandardizedResidualIdx >= 0
                    val raimTrippedStandardized =
                        standardizedRaimAvailable &&
                            activeMeasurements.size > 4 &&
                            maxStandardizedResidual > GnssMathUtils.RAIM_STANDARDIZED_RESIDUAL_SIGMA
                    val raimTrippedAbsolute =
                        !standardizedRaimAvailable &&
                            activeMeasurements.size > 4 &&
                            maxResidual > raimResidualThresholdMeters

                    if ((raimTrippedStandardized || raimTrippedAbsolute) && excludedCount < maxExclusions) {
                        val excludeIdx =
                            if (raimTrippedStandardized) maxStandardizedResidualIdx else maxResidualIdx
                        val svid = activeMeasurements[excludeIdx].svid
                        val why =
                            if (raimTrippedStandardized) {
                                "standardized z=${"%.2f".format(maxStandardizedResidual)}σ (r=${"%.1f".format(maxResidual)}m)"
                            } else {
                                "absolute residual ${"%.1f".format(maxResidual)}m"
                            }
                        Log.w(TAG, "RAIM: Outlier detected! Excluding SVID $svid ($why). Retrying...")
                        activeMeasurements.removeAt(excludeIdx)
                        excludedCount++
                        continue
                    }

                    // If we get here, either no outlier or the exclusion cap was reached.
                    // Carrier ambiguities stay float (see [PppState] kdoc): undifferenced /
                    // ionosphere-free ambiguities are not single-wavelength integers without
                    // external satellite phase-bias products, so no integer fixing is attempted.

                    // Create Result — inter-epoch CV Kalman on ECEF position (design remark #2)
                    val wlsEcef = Triple(currentX, currentY, currentZ)
                    val gpstSeconds = (gpsTimeNanos / 1e9) % GnssMathUtils.SECONDS_IN_WEEK
                    val doppler =
                        dopplerVelocity(
                            activeMeasurements,
                            currentX,
                            currentY,
                            currentZ,
                            clockBiases,
                            constellationToIndex,
                            gpstSeconds,
                        )
                    val displacement =
                        if (gpsTimeNanos > 0L) {
                            tdcpDisplacement(
                                activeMeasurements,
                                clockBiases,
                                constellationToIndex,
                                gpstSeconds,
                                gpsTimeNanos,
                            )
                        } else {
                            clearTdcp()
                            null
                        }
                    val anchor = tdcpAnchor
                    val (outX, outY, outZ) =
                        if (gpsTimeNanos > 0L) {
                            // Measurement covariance for the Kalman = a-posteriori variance factor ×
                            // the metric formal covariance (dimensionally correct m²), instead of the
                            // previous residualsRms² × covariance which double-counted the noise scale.
                            val rMeas =
                                Array(3) { ri ->
                                    DoubleArray(3) { cj ->
                                        lastVarianceFactor * solutionWrapper.covarianceMatrix[ri][cj]
                                    }
                                }
                            interEpochKalman.fuse(
                                wlsEcef,
                                rMeas,
                                gpsTimeNanos,
                                velocityEcef = doppler?.let { Triple(it.velocityX, it.velocityY, it.velocityZ) },
                                velocityVarianceM2PerS2 = doppler?.let { Triple(it.varianceX, it.varianceY, it.varianceZ) },
                                displacementEcef =
                                    if (displacement == null || anchor == null) {
                                        null
                                    } else {
                                        Triple(
                                            anchor.first + displacement.deltaX,
                                            anchor.second + displacement.deltaY,
                                            anchor.third + displacement.deltaZ,
                                        )
                                    },
                                displacementVarianceM2 =
                                    displacement?.let { Triple(it.varianceX, it.varianceY, it.varianceZ) },
                            )
                        } else {
                            interEpochKalman.reset()
                            wlsEcef
                        }

                    val (lat, lon, alt) = GnssMathUtils.ecefToLatLonAlt(outX, outY, outZ)
                    if (gpsTimeNanos > 0L) {
                        rememberTdcp(
                            activeMeasurements,
                            outX,
                            outY,
                            outZ,
                            clockBiases,
                            constellationToIndex,
                            gpstSeconds,
                            gpsTimeNanos,
                        )
                    }
                    pppState.updatePosition(outX, outY, outZ, gpsTimeNanos)

                    val dopsAndAccuracy = computeDopsAndAccuracy(solutionWrapper.covarianceMatrix, dopMatrix, lat, lon, lastVarianceFactor, numConstellations)

                    // Accepted fixes keep the WLS accuracy. A coasted epoch publishes the
                    // prediction, so its accuracy is the predicted covariance, and at least
                    // the horizontal gap to the measurement that was rejected.
                    val coast = interEpochKalman.coastReport()
                    val covHorizontalAccuracy: Float
                    val covVerticalAccuracy: Float
                    if (coast == null) {
                        covHorizontalAccuracy = dopsAndAccuracy[5].toFloat()
                        covVerticalAccuracy = dopsAndAccuracy[6].toFloat()
                    } else {
                        val (varE, varN, varU) = rotateCovarianceToEnu(coast.positionCovariance, lat, lon)
                        val innovH =
                            GnssMathUtils.ecefDeltaHorizontalMeters(
                                coast.innovationX,
                                coast.innovationY,
                                coast.innovationZ,
                                lat,
                                lon,
                            )
                        covHorizontalAccuracy = max(sqrt(varE + varN), innovH).toFloat()
                        covVerticalAccuracy = sqrt(varU).toFloat()
                    }

                    val gpsTimeOfWeekSeconds =
                        if (gpsTimeNanos > 0L) {
                            (gpsTimeNanos / 1_000_000_000.0) % GnssMathUtils.SECONDS_IN_WEEK
                        } else {
                            (System.currentTimeMillis() / 1000.0 - GnssMathUtils.GPS_EPOCH_UNIX_SECONDS + GnssMathUtils.GPS_UTC_LEAP_SECONDS) % GnssMathUtils.SECONDS_IN_WEEK
                        }
                    val ephemerisSigmaCeiling =
                        activeMeasurements.maxOfOrNull { m ->
                            val eph = ephemerisMap[m.constellationType to m.svid] ?: return@maxOfOrNull 0.0
                            val timeSeconds =
                                if (m.constellationType == GnssStatus.CONSTELLATION_BEIDOU) {
                                    gpsTimeOfWeekSeconds - GnssMathUtils.BDT_GPST_OFFSET_SECONDS
                                } else {
                                    gpsTimeOfWeekSeconds
                                }
                            GnssMathUtils.broadcastEphemerisRangeSigmaMeters(ephemerisAgeSeconds(eph, timeSeconds))
                        } ?: 0.0

                    val horizontalAccuracyFloor = minHorizontalAccuracyMeters
                    val verticalAccuracyFloor = minHorizontalAccuracyMeters
                    // The ceiling only stops the clamp from hiding uncertainty already computed above.
                    val coastHorizontal = if (coast == null) 0f else covHorizontalAccuracy
                    val coastVertical = if (coast == null) 0f else covVerticalAccuracy
                    val effectiveMaxHorizontal = maxHorizontalAccuracyMeters + ephemerisSigmaCeiling.toFloat() + coastHorizontal
                    val effectiveMaxVertical =
                        maxHorizontalAccuracyMeters * GnssMathUtils.VERTICAL_ACCURACY_FACTOR +
                            ephemerisSigmaCeiling.toFloat() +
                            coastVertical

                    val horizontalAcc = covHorizontalAccuracy.coerceIn(horizontalAccuracyFloor, effectiveMaxHorizontal)
                    val verticalAcc = covVerticalAccuracy.coerceIn(verticalAccuracyFloor, effectiveMaxVertical)
                    val speedAndBearing = if (doppler == null) null else speedAndBearingDegrees(lat, lon)

                    return@synchronized PositionResult(
                        latitude = lat,
                        longitude = lon,
                        altitude = alt,
                        horizontalAccuracyMeters = horizontalAcc,
                        verticalAccuracyMeters = verticalAcc,
                        timestamp = System.currentTimeMillis(),
                        gdop = dopsAndAccuracy[0],
                        pdop = dopsAndAccuracy[1],
                        hdop = dopsAndAccuracy[2],
                        vdop = dopsAndAccuracy[3],
                        tdop = dopsAndAccuracy[4],
                        residualsRms = lastResidualsRms,
                        // Distinct physical satellites observed this epoch. `measurements` holds one row
                        // per signal/frequency, so a dual-frequency SV (L1+L5) would otherwise be
                        // double-counted; collapse to unique (constellation, svid) pairs.
                        satelliteCount = measurements.distinctBy { it.constellationType to it.svid }.size,
                        usedInFixCount = activeMeasurements.size,
                        speedMetersPerSecond = speedAndBearing?.first,
                        bearingDegrees = speedAndBearing?.second,
                    )
                } else {
                    // Convergence failed
                    Log.d(
                        TAG,
                        "WLS did not converge after $maxWlsIterations iterations — lastRms=${"%.1f".format(lastResidualsRms)}m maxRes=${"%.1f".format(maxResidual)}m sats=${activeMeasurements.size}",
                    )
                    break // Stop iterating, return null
                }
            }

            return null
        }

    /**
     * RTK double-differenced position from base-station OSR observations held in [rtkBaseStore].
     *
     * For each rover measurement that has a fresh matching base observation and a usable broadcast
     * ephemeris, the satellite ECEF position is computed and a double-difference observation is formed
     * (see [RtkDoubleDifferenceSolver]). Differencing cancels satellite clock/orbit, receiver clocks,
     * and short-baseline atmosphere, yielding a decimeter-class float fix relative to the known base.
     * Returns null when there is no fresh base data, too few matched satellites, or the solve fails —
     * callers should then fall back to [computePosition].
     */
    private fun rtkCarrierSamples(measurements: List<SatelliteMeasurement>): List<RtkAmbiguityMemory.Sample> {
        val byKey = LinkedHashMap<Pair<Int, Int>, RtkAmbiguityMemory.Sample>()
        for (meas in measurements) {
            val key = meas.constellationType to meas.svid
            if (byKey.containsKey(key) && meas.isL5) continue
            byKey[key] =
                RtkAmbiguityMemory.Sample(
                    constellationType = meas.constellationType,
                    svid = meas.svid,
                    adrMeters = meas.accumulatedDeltaRangeMeters,
                    adrState = meas.adrState,
                    rateMetersPerSecond = meas.pseudorangeRateMetersPerSecond,
                    rateUncertaintyMetersPerSecond = meas.pseudorangeRateUncertaintyMetersPerSecond,
                    halfCycleAmbiguous = isCarrierHalfCycleAmbiguous(meas.adrState),
                )
        }
        return byKey.values.toList()
    }

    fun computeRtkPosition(
        measurements: List<SatelliteMeasurement>,
        gpsTimeNanos: Long,
        initialLat: Double = 0.0,
        initialLon: Double = 0.0,
        initialAlt: Double = 0.0,
    ): PositionResult? =
        synchronized(lock) {
            if (gpsTimeNanos <= 0L) return null
            val basePosition = rtkBaseStore.getBasePosition() ?: return null
            val baseEcef = doubleArrayOf(basePosition.xMeters, basePosition.yMeters, basePosition.zMeters)

            val gpstSeconds = (gpsTimeNanos / 1e9) % GnssMathUtils.SECONDS_IN_WEEK
            val codeInputs = ArrayList<RtkDoubleDifferenceSolver.SatelliteInput>()
            val carrierInputs = ArrayList<RtkDoubleDifferenceSolver.CarrierSatelliteInput>()
            val usedSatKeys = HashSet<Pair<Int, Int>>()

            for (meas in measurements) {
                val satKey = meas.constellationType to meas.svid
                if (satKey in usedSatKeys) continue // one observation per satellite
                val ephemeris = ephemerisMap[satKey]?.takeIf { isUsableEphemerisLocked(it, gpstSeconds) } ?: continue
                // GLONASS MSM carries a day/time-of-day epoch (not a GPS TOW), so its base epoch can't
                // be aligned to the rover here — skip it for RTK (it's the hardest constellation for
                // RTK regardless, and is already excluded first in the absolute fallback).
                if (meas.constellationType == GnssStatus.CONSTELLATION_GLONASS) continue
                val baseObs =
                    rtkBaseStore.getObservation(meas.constellationType, meas.svid, meas.carrierFrequencyHz) ?: continue

                val currentTimeSeconds =
                    if (meas.constellationType == GnssStatus.CONSTELLATION_BEIDOU) {
                        gpstSeconds - GnssMathUtils.BDT_GPST_OFFSET_SECONDS
                    } else {
                        gpstSeconds
                    }

                // Rover satellite position at the rover's transmit time. The receiver clock cancels
                // in t_rx − PR/c; Δt_sv still has to come off (review #88). No rover-clock
                // estimate exists on this path, so the rotation interval is PR/c + Δt_sv.
                val roverTiming = rtkTransmitTiming(ephemeris, currentTimeSeconds, meas.pseudorangeMeters)
                val (tx, ty, tz) =
                    SatellitePositionCalculator.calculateSatellitePosition(ephemeris, roverTiming.timing.transmitTimeSeconds)
                val satEcef =
                    SatellitePositionCalculator.rotateEcefForEarthRotationDuringTransit(
                        tx,
                        ty,
                        tz,
                        roverTiming.timing.rotationSeconds.coerceIn(MIN_TRAVEL_TIME_SECONDS, MAX_TRAVEL_TIME_SECONDS),
                    ).let { doubleArrayOf(it.first, it.second, it.third) }

                // Base satellite position at the BASE observation's epoch. The base stream lags the
                // rover by network latency; over that interval the satellite moves ~km along the line
                // of sight, so reusing the rover-epoch position for the base range would corrupt the
                // double difference (kilometre-level residuals). The MSM epoch is already in the
                // constellation's timescale (BDT for BeiDou, GPST otherwise), matching currentTimeSeconds.
                val baseEpochSeconds = baseObs.epochTimeMs / 1000.0
                if (kotlin.math.abs(baseEpochSeconds - currentTimeSeconds) > MAX_BASE_ROVER_EPOCH_DIFF_SECONDS) continue
                val baseTravel = rtkTransmitTiming(ephemeris, baseEpochSeconds, baseObs.pseudorangeMeters)
                val (btx, bty, btz) =
                    SatellitePositionCalculator.calculateSatellitePosition(ephemeris, baseTravel.timing.transmitTimeSeconds)
                val baseSatEcef =
                    SatellitePositionCalculator.rotateEcefForEarthRotationDuringTransit(
                        btx,
                        bty,
                        btz,
                        baseTravel.timing.rotationSeconds.coerceIn(MIN_TRAVEL_TIME_SECONDS, MAX_TRAVEL_TIME_SECONDS),
                    )
                        .let { doubleArrayOf(it.first, it.second, it.third) }

                val codeSigma =
                    GnssMathUtils.pseudorangeSigmaFromElevationAndCn0(meas.elevationDegrees, meas.cn0DbHz)
                        .coerceIn(GnssMathUtils.MIN_PSEUDORANGE_SIGMA_METERS, GnssMathUtils.MAX_PSEUDORANGE_SIGMA_METERS)

                codeInputs.add(
                    RtkDoubleDifferenceSolver.SatelliteInput(
                        satelliteEcef = satEcef,
                        baseSatelliteEcef = baseSatEcef,
                        roverObservationMeters =
                            RtkDoubleDifferenceSolver.withoutSatelliteClock(
                                meas.rawPseudorangeMeters,
                                roverTiming.satelliteClockSeconds,
                            ),
                        baseObservationMeters =
                            RtkDoubleDifferenceSolver.withoutSatelliteClock(
                                baseObs.pseudorangeMeters,
                                baseTravel.satelliteClockSeconds,
                            ),
                        elevationDeg = meas.elevationDegrees,
                        sigmaMeters = codeSigma,
                        constellationType = meas.constellationType,
                        bandMhz = (meas.carrierFrequencyHz / 1e6).toInt(),
                    ),
                )

                // Carrier double-difference input — only when both rover and base have valid
                // phase AND the half-cycle ambiguity is not flagged unresolved: a Costas-loop
                // phase that is still half-cycle ambiguous is offset by λ/2, which makes the DD
                // ambiguity half-integer — integer fixing on it is wrong by ~10 cm per SV.
                val roverCarrier = meas.carrierPhaseMeters
                val baseCarrier = baseObs.carrierPhaseMeters
                if (roverCarrier != null && baseCarrier != null && meas.wavelengthMeters > 0.0 &&
                    !isCarrierHalfCycleAmbiguous(meas.adrState)
                ) {
                    val sinEl = sin(Math.toRadians(meas.elevationDegrees)).coerceAtLeast(0.05)
                    val carrierSigma = (CARRIER_PHASE_BASE_SIGMA_METERS / sinEl).coerceIn(CARRIER_PHASE_BASE_SIGMA_METERS, 0.05)
                    carrierInputs.add(
                        RtkDoubleDifferenceSolver.CarrierSatelliteInput(
                            satelliteEcef = satEcef,
                            baseSatelliteEcef = baseSatEcef,
                            roverCodeMeters =
                                RtkDoubleDifferenceSolver.withoutSatelliteClock(
                                    meas.rawPseudorangeMeters,
                                    roverTiming.satelliteClockSeconds,
                                ),
                            baseCodeMeters =
                                RtkDoubleDifferenceSolver.withoutSatelliteClock(
                                    baseObs.pseudorangeMeters,
                                    baseTravel.satelliteClockSeconds,
                                ),
                            roverCarrierMeters =
                                RtkDoubleDifferenceSolver.withoutSatelliteClock(
                                    roverCarrier,
                                    roverTiming.satelliteClockSeconds,
                                ),
                            baseCarrierMeters =
                                RtkDoubleDifferenceSolver.withoutSatelliteClock(
                                    baseCarrier,
                                    baseTravel.satelliteClockSeconds,
                                ),
                            wavelengthMeters = meas.wavelengthMeters,
                            elevationDeg = meas.elevationDegrees,
                            codeSigmaMeters = codeSigma,
                            carrierSigmaMeters = carrierSigma,
                            constellationType = meas.constellationType,
                            svid = meas.svid,
                        ),
                    )
                }
                usedSatKeys.add(satKey)
            }

            if (codeInputs.size < RtkDoubleDifferenceSolver.MIN_SATELLITES) {
                Log.d(TAG, "RTK: only ${codeInputs.size} matched base+ephemeris satellites — cannot form double differences")
                return null
            }

            val seedRover =
                pppState.getLastPositionEcef()?.let { doubleArrayOf(it.first, it.second, it.third) }
                    ?: if (initialLat != 0.0 || initialLon != 0.0) {
                        val (sx, sy, sz) = GnssMathUtils.latLonAltToEcef(initialLat, initialLon, initialAlt)
                        doubleArrayOf(sx, sy, sz)
                    } else {
                        baseEcef
                    }

            // Always compute the robust code double-difference float solution (decimeter-class).
            val floatSolution = RtkDoubleDifferenceSolver.solve(baseEcef, codeInputs, seedRover) ?: return null

            // Sanity guard: a healthy code DD has sub-metre-to-few-metre residuals. If the post-fit
            // rms is implausibly large after outlier rejection — e.g. a bad reference satellite that
            // corrupts every double difference and cannot be excluded — the DD is untrustworthy this
            // epoch, so abandon RTK and let the caller fall back to the absolute fix rather than
            // returning a garbage position.
            if (floatSolution.residualRms > RTK_MAX_ACCEPTABLE_FLOAT_RMS_METERS) {
                Log.d(TAG, "RTK float rms=${"%.1f".format(floatSolution.residualRms)}m exceeds sanity limit — falling back to absolute")
                return null
            }

            // Then attempt to upgrade to an integer-fixed (centimeter) solution. This is purely
            // additive: solveFixed returns null unless the LAMBDA ratio test passes AND the fixed
            // position agrees with the float, so a marginal or wrong fix can never replace the float.
            val carrierSamples = rtkCarrierSamples(measurements)
            val fixedSolution =
                if (carrierInputs.size >= RtkDoubleDifferenceSolver.MIN_FIX_SATELLITES) {
                    var carried: List<RtkDoubleDifferenceSolver.AmbiguityState>? = null
                    val fixed =
                        RtkDoubleDifferenceSolver.solveFixed(
                            baseEcef = baseEcef,
                            inputs = carrierInputs,
                            initialRover = floatSolution.roverEcef,
                            floatRoverEcef = floatSolution.roverEcef,
                            priors = rtkAmbiguityMemory.applicablePriors(carrierSamples, gpsTimeNanos),
                            onFloat = { carried = it },
                        )
                    if (carried != null) {
                        rtkAmbiguityMemory.remember(carried, carrierSamples, gpsTimeNanos)
                    }
                    fixed
                } else {
                    null
                }

            val roverEcef: DoubleArray
            val residualRms: Double
            val covariance: Array<DoubleArray>
            val usedCount: Int
            val refSigma: Double
            val mode: String
            if (fixedSolution != null) {
                roverEcef = fixedSolution.roverEcef
                residualRms = fixedSolution.residualRms
                covariance = fixedSolution.covariance
                usedCount = fixedSolution.usedSatelliteCount
                refSigma = carrierInputs.map { it.carrierSigmaMeters }.average().coerceAtLeast(0.001)
                mode = "FIXED ratio=${"%.1f".format(fixedSolution.ratio)}"
            } else {
                roverEcef = floatSolution.roverEcef
                residualRms = floatSolution.residualRms
                covariance = floatSolution.covariance
                usedCount = floatSolution.usedSatelliteCount
                refSigma = codeInputs.map { it.sigmaMeters }.average().coerceAtLeast(0.1)
                mode = "FLOAT"
            }

            val (lat, lon, alt) = GnssMathUtils.ecefToLatLonAlt(roverEcef[0], roverEcef[1], roverEcef[2])
            pppState.updatePosition(roverEcef[0], roverEcef[1], roverEcef[2], gpsTimeNanos)

            val accuracy = computeRtkAccuracy(covariance, lat, lon, refSigma, residualRms)
            Log.d(
                TAG,
                "RTK $mode: sats=$usedCount hAcc=${"%.2f".format(accuracy.horizontalMeters)}m rms=${"%.2f".format(residualRms)}m ${rtkBaseStore.describe()}",
            )

            return@synchronized PositionResult(
                latitude = lat,
                longitude = lon,
                altitude = alt,
                horizontalAccuracyMeters = accuracy.horizontalMeters.toFloat(),
                verticalAccuracyMeters = accuracy.verticalMeters.toFloat(),
                timestamp = System.currentTimeMillis(),
                gdop = accuracy.pdop,
                pdop = accuracy.pdop,
                hdop = accuracy.hdop,
                vdop = accuracy.vdop,
                tdop = 0.0,
                residualsRms = residualRms,
                satelliteCount = measurements.distinctBy { it.constellationType to it.svid }.size,
                usedInFixCount = usedCount,
                solutionMode = if (fixedSolution != null) GnssSolutionMode.RTK_FIXED else GnssSolutionMode.RTK_FLOAT,
            )
        }

    private data class RtkAccuracy(
        val horizontalMeters: Double,
        val verticalMeters: Double,
        val hdop: Double,
        val vdop: Double,
        val pdop: Double,
    )

    /**
     * Derives horizontal/vertical accuracy and DOP from the double-difference position covariance.
     *
     * The reported accuracy is **a-posteriori**, matching the absolute solver: the geometric DOP
     * (formal ENU sigma normalized by the a-priori observation sigma) is scaled by the *realized*
     * post-fit residual RMS, not the assumed measurement noise. This keeps the accuracy honest — a
     * wrong integer fix, a bad base decode, or a long baseline all blow up the residuals and inflate
     * the reported accuracy, instead of advertising a tight circle around a position that did not
     * actually improve. (A purely formal covariance would always look tight regardless of realized
     * error, which biases the best-candidate selection toward overconfident fixes.)
     */
    private fun computeRtkAccuracy(
        covariance: Array<DoubleArray>,
        lat: Double,
        lon: Double,
        refSigmaMeters: Double,
        residualRms: Double,
    ): RtkAccuracy {
        val latRad = Math.toRadians(lat)
        val lonRad = Math.toRadians(lon)
        val sinLat = sin(latRad)
        val cosLat = cos(latRad)
        val sinLon = sin(lonRad)
        val cosLon = cos(lonRad)
        val r =
            arrayOf(
                doubleArrayOf(-sinLon, cosLon, 0.0),
                doubleArrayOf(-sinLat * cosLon, -sinLat * sinLon, cosLat),
                doubleArrayOf(cosLat * cosLon, cosLat * sinLon, sinLat),
            )
        val qEnu = Array(3) { DoubleArray(3) }
        for (i in 0 until 3) {
            for (j in 0 until 3) {
                var sum = 0.0
                for (k in 0 until 3) {
                    for (l in 0 until 3) {
                        sum += r[i][k] * covariance[k][l] * r[j][l]
                    }
                }
                qEnu[i][j] = sum
            }
        }
        val varE = qEnu[0][0].coerceAtLeast(0.0)
        val varN = qEnu[1][1].coerceAtLeast(0.0)
        val varU = qEnu[2][2].coerceAtLeast(0.0)
        val refSigma = refSigmaMeters.coerceAtLeast(1e-3)
        // Geometric dilution: formal ENU sigma divided by the a-priori observation sigma.
        val hdop = sqrt(varE + varN) / refSigma
        val vdop = sqrt(varU) / refSigma
        val pdop = sqrt(varE + varN + varU) / refSigma

        // A-posteriori unit-weight sigma from the realized residuals (honest scale), floored so a
        // genuine carrier fix can still report a few cm but never an unrealistic sub-cm circle.
        val sigma0 = residualRms.coerceAtLeast(RTK_ACCURACY_SIGMA0_FLOOR_METERS)

        return RtkAccuracy(
            horizontalMeters =
                (sigma0 * hdop).coerceIn(
                    GnssMathUtils.MIN_HORIZONTAL_ACCURACY.toDouble(),
                    GnssMathUtils.MAX_HORIZONTAL_ACCURACY.toDouble(),
                ),
            verticalMeters =
                (sigma0 * vdop).coerceIn(
                    GnssMathUtils.MIN_HORIZONTAL_ACCURACY.toDouble(),
                    (GnssMathUtils.MAX_HORIZONTAL_ACCURACY * GnssMathUtils.VERTICAL_ACCURACY_FACTOR).toDouble(),
                ),
            hdop = hdop,
            vdop = vdop,
            pdop = pdop,
        )
    }

    /**
     * Code pseudoranges to feed the ionosphere-free combination, plus the satellite-clock
     * group-delay convention the combined observable must pair with.
     */
    internal data class IfCodeAdjustment(
        val l1Meters: Double,
        val l5Meters: Double,
        /**
         * Whether the satellite clock applied to the IF observable should still include the
         * broadcast group delay (−TGD). True only when the pseudoranges were aligned to the
         * clock's reference signal pair (CNAV ISCs → L1 P(Y); BDS TGD1 → B1I/B3 convention).
         */
        val clockIncludesGroupDelay: Boolean,
    )

    /**
     * Group delay subtracted from the broadcast clock, in seconds.
     *
     * GPS and BeiDou keep the previous convention: no delay on an LNAV/BGD ionosphere-free
     * clock, plain TGD for BeiDou and for an IF row, and γ·TGD on a single frequency.
     * Galileo follows OS SIS ICD §5.1.5. An I/NAV clock on the E1/E5a ionosphere-free
     * path still adjusts, by BGD(E1,E5b) − BGD(E1,E5a), because that offset does not
     * cancel in the combination.
     */
    internal fun groupDelaySecondsForMeasurement(
        ephemeris: SatelliteEphemeris,
        constellationType: Int,
        carrierFrequencyHz: Double,
        ionosphereFree: Boolean,
        clockExcludesGroupDelay: Boolean,
    ): Double {
        if (constellationType == GnssStatus.CONSTELLATION_GALILEO) {
            return galileoGroupDelaySeconds(ephemeris, carrierFrequencyHz, ionosphereFree)
        }
        if (clockExcludesGroupDelay) return 0.0
        val tgdSeconds = ephemeris.tgd
        if (!tgdSeconds.isFinite() || tgdSeconds == 0.0) return 0.0
        if (ionosphereFree ||
            constellationType == GnssStatus.CONSTELLATION_BEIDOU ||
            carrierFrequencyHz <= 0.0
        ) {
            return tgdSeconds
        }
        return tgdSeconds * (GnssMathUtils.L1_FREQUENCY_HZ / carrierFrequencyHz).pow(2)
    }

    /**
     * Galileo OS SIS ICD §5.1.5. γ₅ₐ = (f_E1 / f)².
     * E1 uses the clock's own BGD. E5a with an F/NAV clock is γ·BGD(E1,E5a); with an
     * I/NAV clock it is BGD(E1,E5b) + (γ − 1)·BGD(E1,E5a). The E1/E5a ionosphere-free
     * combination is 0 for F/NAV and BGD(E1,E5b) − BGD(E1,E5a) for I/NAV.
     * A record that only carries [SatelliteEphemeris.tgd] is treated as F/NAV BGD(E1,E5a).
     */
    internal fun galileoGroupDelaySeconds(
        ephemeris: SatelliteEphemeris,
        carrierFrequencyHz: Double,
        ionosphereFree: Boolean,
    ): Double {
        val legacy =
            !ephemeris.galileoInavClock &&
                ephemeris.galileoBgdE1E5aSeconds == 0.0 &&
                ephemeris.galileoBgdE1E5bSeconds == 0.0
        val bgdA = if (legacy) ephemeris.tgd else ephemeris.galileoBgdE1E5aSeconds
        val bgdB = if (legacy) ephemeris.tgd else ephemeris.galileoBgdE1E5bSeconds
        if (!bgdA.isFinite() || !bgdB.isFinite()) return 0.0
        if (ionosphereFree) return if (ephemeris.galileoInavClock) bgdB - bgdA else 0.0
        val e5a = carrierFrequencyHz > 0.0 && carrierFrequencyHz < GnssMathUtils.L5_FREQUENCY_THRESHOLD_HZ
        val gamma =
            if (carrierFrequencyHz > 0.0) {
                (GnssMathUtils.L1_FREQUENCY_HZ / carrierFrequencyHz).pow(2)
            } else {
                1.0
            }
        return if (e5a) {
            if (ephemeris.galileoInavClock) bgdB + (gamma - 1.0) * bgdA else gamma * bgdA
        } else {
            (if (ephemeris.galileoInavClock) bgdB else bgdA) * gamma
        }
    }

    /**
     * Adjusts dual-frequency **code** pseudoranges before the ionosphere-free linear combination
     * (see [computePosition]).
     *
     * - **GPS with recent CNAV MT30:** applies **+ISC·c** to L1 and L5 per IS-GPS-705; the IF
     *   observable is then referenced to L1 P(Y) and pairs with the −TGD clock.
     * - **GPS LNAV / Galileo BGD fallback:** **no adjustment.** Per IS-GPS-200/705 the L1 bias
     *   relative to the broadcast clock is c·TGD and the L5 bias c·γ₁₅·TGD, whose IF combination
     *   is identically zero — so the raw IF observable pairs with the clock **without** −TGD.
     *   (The previous +TGD·c on L5 together with the −TGD clock left ≈ −2.26·c·TGD per SV, up
     *   to ±8 m, on every dual-frequency fix.)
     * - **BeiDou D1:** raw L1/L5 with the −TGD1 clock kept (TGD2 is not decoded; B1I-referenced
     *   TGD1 is the closest available convention).
     */
    internal fun codePseudorangesForIonosphereFreeMeters(
        satKey: Pair<Int, Int>,
        l1Meters: Double,
        l5Meters: Double,
        ephemeris: SatelliteEphemeris?,
    ): IfCodeAdjustment {
        if (satKey.first == GnssStatus.CONSTELLATION_BEIDOU) {
            return IfCodeAdjustment(l1Meters, l5Meters, clockIncludesGroupDelay = true)
        }
        if (satKey.first == GnssStatus.CONSTELLATION_GPS) {
            val iscRec = gpsCnavIscMap[satKey]
            if (iscRec != null &&
                SystemClock.elapsedRealtime() - iscRec.lastUpdateElapsedMs < GnssMathUtils.EPHEMERIS_STALENESS_MS
            ) {
                return IfCodeAdjustment(
                    l1Meters = l1Meters + iscRec.iscL1CaSeconds * SPEED_OF_LIGHT,
                    l5Meters = l5Meters + iscRec.iscL5I5Seconds * SPEED_OF_LIGHT,
                    clockIncludesGroupDelay = true,
                )
            }
        }
        return IfCodeAdjustment(l1Meters, l5Meters, clockIncludesGroupDelay = false)
    }

    /**
     * Computes DOP values and covariance-derived accuracy.
     * Returns [gdop, pdop, hdop, vdop, tdop, horizontalAccuracyMeters, verticalAccuracyMeters].
     *
     * DOPs come from the UNWEIGHTED geometry inverse [dopMatrix] (dimensionless, classical
     * definition); accuracy comes from the metric weighted covariance scaled by the
     * a-posteriori variance factor. When the geometry inverse is unavailable (degenerate
     * normal matrix) the DOPs are reported as 0 rather than substituting metric sigmas.
     */
    private fun computeDopsAndAccuracy(
        weightedCovariance: Array<DoubleArray>,
        dopMatrix: Array<DoubleArray>?,
        lat: Double,
        lon: Double,
        aPosterioriVarianceFactor: Double,
        numClockStates: Int = 1,
    ): DoubleArray {
        var gdop = 0.0
        var pdop = 0.0
        var hdop = 0.0
        var vdop = 0.0
        var tdop = 0.0
        if (dopMatrix != null) {
            pdop = sqrt(dopMatrix[0][0] + dopMatrix[1][1] + dopMatrix[2][2])
            var clockTraceSum = 0.0
            for (k in 3 until (3 + numClockStates).coerceAtMost(dopMatrix.size)) {
                clockTraceSum += dopMatrix[k][k]
            }
            tdop = sqrt(clockTraceSum)
            gdop = sqrt(dopMatrix[0][0] + dopMatrix[1][1] + dopMatrix[2][2] + clockTraceSum)
            val (dopVarE, dopVarN, dopVarU) = rotateCovarianceToEnu(dopMatrix, lat, lon)
            hdop = sqrt(dopVarE + dopVarN)
            vdop = sqrt(dopVarU)
        }

        // Accuracy = sqrt(varianceFactor) x formal sigma. The WLS weights are 1/sigma^2, so
        // the weighted covariance is the metric formal covariance in m^2; the dimensionless
        // a-posteriori factor (>= 1) scales it for the realized fit.
        val (varE, varN, varU) = rotateCovarianceToEnu(weightedCovariance, lat, lon)
        val sigma0 = sqrt(aPosterioriVarianceFactor)
        val horizontalAccuracy = sigma0 * sqrt(varE + varN)
        val verticalAccuracy = sigma0 * sqrt(varU)

        return doubleArrayOf(gdop, pdop, hdop, vdop, tdop, horizontalAccuracy, verticalAccuracy)
    }

    /** Rotates the position 3x3 block of [covariance] into ENU; returns (varE, varN, varU). */
    private fun rotateCovarianceToEnu(
        covariance: Array<DoubleArray>,
        lat: Double,
        lon: Double,
    ): Triple<Double, Double, Double> {
        val latRad = Math.toRadians(lat)
        val lonRad = Math.toRadians(lon)
        val sinLat = sin(latRad)
        val cosLat = cos(latRad)
        val sinLon = sin(lonRad)
        val cosLon = cos(lonRad)

        val r =
            arrayOf(
                doubleArrayOf(-sinLon, cosLon, 0.0),
                doubleArrayOf(-sinLat * cosLon, -sinLat * sinLon, cosLat),
                doubleArrayOf(cosLat * cosLon, cosLat * sinLon, sinLat),
            )
        val enu = DoubleArray(3)
        for (i in 0 until 3) {
            var sum = 0.0
            for (k in 0 until 3) {
                for (l in 0 until 3) {
                    sum += r[i][k] * covariance[k][l] * r[i][l]
                }
            }
            enu[i] = sum.coerceAtLeast(0.0)
        }
        return Triple(enu[0], enu[1], enu[2])
    }

    private fun getConstellationName(type: Int): String =
        when (type) {
            GnssStatus.CONSTELLATION_GPS -> "GPS"
            GnssStatus.CONSTELLATION_GLONASS -> "GLONASS"
            GnssStatus.CONSTELLATION_GALILEO -> "GALILEO"
            GnssStatus.CONSTELLATION_BEIDOU -> "BEIDOU"
            GnssStatus.CONSTELLATION_QZSS -> "QZSS"
            GnssStatus.CONSTELLATION_IRNSS -> "NAVIC"
            GnssStatus.CONSTELLATION_SBAS -> "SBAS"
            else -> "UNKNOWN"
        }

    /**
     * Broadcast-ephemeris IOD used to gate SSR application for this constellation, or -1 to
     * skip the gate. GPS (IODE) and Galileo (IODnav) have well-defined matches against the
     * SSR per-satellite IOD. BeiDou SSR IOD conventions differ between providers (AODE vs
     * toe-derived), so the gate is skipped there rather than silently rejecting every BDS
     * correction. An ephemeris with unknown issue (iode = -1) also skips the gate.
     */
    private fun ssrMatchIod(ephemeris: SatelliteEphemeris): Int = if (ephemeris.constellationType == GnssStatus.CONSTELLATION_BEIDOU) -1 else ephemeris.iode

    /**
     * SSR orbit correction applicable to this satellite/ephemeris, or null. GLONASS SSR is
     * never applied: its DF386 epoch is UTC(SU) day-seconds, which is not yet converted to
     * the GPS timescale used for the age/extrapolation terms.
     */
    private fun getApplicableSsrOrbit(
        constellationType: Int,
        svid: Int,
        ephemeris: SatelliteEphemeris,
    ): SsrCorrections.OrbitCorrection? {
        if (constellationType == GnssStatus.CONSTELLATION_GLONASS) return null
        return ssrStore.getOrbit(constellationType, svid, ssrMatchIod(ephemeris))
    }

    /** SSR clock correction applicable to this satellite/ephemeris, or null. See [getApplicableSsrOrbit]. */
    private fun getApplicableSsrClock(
        constellationType: Int,
        svid: Int,
        ephemeris: SatelliteEphemeris,
    ): SsrCorrections.ClockCorrection? {
        if (constellationType == GnssStatus.CONSTELLATION_GLONASS) return null
        return ssrStore.getClock(constellationType, svid, ssrMatchIod(ephemeris))
    }

    /**
     * Returns the SBAS fast pseudorange correction (PRC) for a GPS satellite, in meters.
     * Scans all tracked SBAS PRN masks and returns the first matching fresh PRC, or 0.0 if none.
     * The PRC should be *added* to the satellite clock correction.
     *
     * Stale corrections older than [GnssMathUtils.SBAS_FAST_CORRECTION_STALENESS_MS] are discarded.
     * Only applies to GPS satellites (SBAS monitors GPS L1 primarily).
     */
    private fun getSbasFastCorrectionMeters(
        constellationType: Int,
        svid: Int,
    ): Double {
        if (constellationType != GnssStatus.CONSTELLATION_GPS) return 0.0
        val now = SystemClock.elapsedRealtime()
        for ((sbasSvid, mask) in sbasPrnMasks) {
            val fastStore = sbasFastCorrections[sbasSvid] ?: continue
            // Find this GPS PRN in the SBAS PRN mask
            val maskEntry = mask.maskToPrn.entries.firstOrNull { it.value == svid } ?: continue
            val fc = fastStore[maskEntry.key] ?: continue
            // The correction indexes into the mask it was broadcast against: across an IODP
            // changeover the slot→PRN assignment shifts, so a mismatched IODP must not apply.
            if (fc.iodp != mask.iodp) continue
            // UDREI = 14 or 15 means "do not use" / "not monitored"
            if (fc.udrei >= 14) continue
            // Discard stale corrections
            if (now - fc.receivedElapsedMs > GnssMathUtils.SBAS_FAST_CORRECTION_STALENESS_MS) continue
            return fc.prc
        }
        return 0.0
    }

    /**
     * Returns the SBAS long-term orbit correction for a GPS satellite, or null if unavailable,
     * stale, or computed against a different broadcast-ephemeris issue than [ephemerisIode]
     * (DO-229 §2.1.1.4.9 — applying a correction across an ephemeris changeover injects the
     * meters-level difference between the two issues).
     */
    internal fun getSbasLongTermCorrection(
        svid: Int,
        ephemerisIode: Int,
    ): SbasNavMessageParser.SbasLongTermCorrection? {
        val ltc = sbasLongTermCorrections[svid] ?: return null
        if (SystemClock.elapsedRealtime() - ltc.receivedElapsedMs > GnssMathUtils.SBAS_LONG_TERM_CORRECTION_STALENESS_MS) return null
        if (ephemerisIode >= 0 && ltc.iode != ephemerisIode) return null
        return ltc
    }

    /**
     * Returns the SBAS long-term clock correction (delta-af0) for a GPS satellite, in meters,
     * or 0.0 if not available, stale, or IODE-mismatched.
     */
    private fun getSbasLongTermClockCorrectionMeters(
        constellationType: Int,
        svid: Int,
        ephemerisIode: Int,
    ): Double {
        if (constellationType != GnssStatus.CONSTELLATION_GPS) return 0.0
        val ltc = getSbasLongTermCorrection(svid, ephemerisIode) ?: return 0.0
        return ltc.deltaAf0 * GnssMathUtils.SPEED_OF_LIGHT
    }

    companion object {
        private const val TAG = "GnssMeasurementProcessor"
        private const val MAX_PSEUDORANGE_RATE_SIGMA_MPS = 5.0

        /**
         * When [android.location.GnssStatus] has not yet reported this SV, elevation defaults to 0° in the
         * Android API. Using a modest placeholder keeps the SV in the solution while C/N0 and WLS geometry
         * (recomputed each iteration) dominate; iono uses this azimuth/elev until status arrives.
         */
        private const val PLACEHOLDER_ELEVATION_WHEN_STATUS_UNKNOWN_DEG = 20.0

        /** GPS TLM word preamble (IS-GPS-200 §20.3.3.1) and its D30*-complemented form. */
        private const val TLM_PREAMBLE = 0x8B
        private const val TLM_PREAMBLE_INVERTED = 0x74

        /** Zenith one-sigma carrier-phase noise (m) for RTK; scaled by 1/sin(elevation) per satellite. */
        private const val CARRIER_PHASE_BASE_SIGMA_METERS = 0.003

        /** Max base/rover epoch offset (s) tolerated for a double difference; guards against stale base data. */
        private const val MAX_BASE_ROVER_EPOCH_DIFF_SECONDS = 30.0

        /**
         * Code double-difference post-fit residual RMS above which the RTK fix is rejected (and the
         * caller falls back to absolute positioning). Healthy code DD is sub-metre to a few metres;
         * a much larger value means a bad/uncorrectable satellite (often the reference) corrupted the
         * solution this epoch.
         */
        private const val RTK_MAX_ACCEPTABLE_FLOAT_RMS_METERS = 15.0

        /**
         * Floor for the a-posteriori unit-weight sigma in RTK accuracy. Lets a genuine carrier fix
         * report a few cm while preventing an unrealistically optimistic sub-cm circle on consumer
         * hardware.
         */
        private const val RTK_ACCURACY_SIGMA0_FLOOR_METERS = 0.02

        /** Minimum reasonable pseudorange in meters (~15,000 km — low-elevation GEO/MEO). */
        private const val MIN_REASONABLE_PSEUDORANGE_METERS = 15_000_000.0

        /** Maximum reasonable pseudorange in meters (~42,000 km — SBAS/QZSS GEO at low elevation). */
        private const val MAX_REASONABLE_PSEUDORANGE_METERS = 42_000_000.0

        /**
         * Lower clamp for travel time used during satellite-position / clock refinement. Travel
         * times below ~50 ms would imply a physically impossible SV distance (< 15,000 km) and
         * typically indicate a runaway receiver-clock estimate during early WLS iterations.
         */
        private const val MIN_TRAVEL_TIME_SECONDS =
            MIN_REASONABLE_PSEUDORANGE_METERS / SPEED_OF_LIGHT

        /** Upper clamp for travel time (~117 ms, corresponding to ~35,000 km). */
        private const val MAX_TRAVEL_TIME_SECONDS =
            MAX_REASONABLE_PSEUDORANGE_METERS / SPEED_OF_LIGHT
    }
}
