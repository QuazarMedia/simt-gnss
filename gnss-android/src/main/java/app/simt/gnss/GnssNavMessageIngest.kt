package app.simt.gnss

import android.location.GnssNavigationMessage
import android.location.GnssStatus
import android.os.SystemClock
import android.util.Log
import app.simt.gnss.core.model.Constellation
import app.simt.gnss.core.model.GnssSignalBand
import app.simt.gnss.core.model.SatelliteEphemeris
import app.simt.gnss.core.parser.BeidouNavMessageParser
import app.simt.gnss.core.parser.GalileoNavMessageParser
import app.simt.gnss.core.parser.GlonassNavMessageParser
import app.simt.gnss.core.parser.GpsCnavMt30Parser
import app.simt.gnss.core.parser.GpsNavMessageParser
import app.simt.gnss.core.parser.SbasNavMessageParser
import app.simt.gnss.core.platform.GnssPlatform
import app.simt.gnss.core.utils.GnssMathUtils
import app.simt.gnss.core.utils.GnssMathUtils.SPEED_OF_LIGHT
import app.simt.gnss.core.validator.GpsNavMessageValidator
import kotlin.math.*

/**
 * Navigation-message ingest: decodes GPS/Galileo/GLONASS/BeiDou/SBAS L1 navigation
 * streams into the ephemeris store, SBAS correction tables and the ionosphere grid
 * coefficients. Owns the per-satellite bit/subframe accumulation state.
 */
internal class GnssNavMessageIngest(
    private val ephemerisStore: GnssEphemerisStore,
    private val suppressSbas: Boolean,
) {
    /** Latest GPS CNAV MT30 ISC(L1CA)/ISC(L5I5) per SV; keyed like the ephemeris-store map. */
    private val gpsCnavIscMap = mutableMapOf<Pair<Int, Int>, GpsCnavIscRecord>()

    private val subframeStorage = mutableMapOf<Pair<Int, Int>, Array<ByteArray?>>()

    private val galileoWordStorage = mutableMapOf<Pair<Int, Int>, MutableMap<Int, ByteArray>>()

    private val glonassStringStorage = mutableMapOf<Pair<Int, Int>, MutableMap<Int, Pair<ByteArray, Long>>>()

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
    internal var lastGpsTimeOfWeekSeconds: Double = 0.0
    internal var hasGpsTimeOfWeek: Boolean = false

    /** GPS−UTC leap seconds from the receiver clock (fallback: compile-time constant). */
    internal var lastKnownLeapSeconds: Int = GnssMathUtils.GPS_UTC_LEAP_SECONDS.toInt()

    private val gpsLastBits = mutableMapOf<Pair<Int, Int>, Int>()
    internal var ionoAlpha: DoubleArray? = null
    internal var ionoBeta: DoubleArray? = null

    internal fun currentSystemGpsTimeNanos(leapSeconds: Int = lastKnownLeapSeconds): Long = ((System.currentTimeMillis() / 1000L - GnssMathUtils.GPS_EPOCH_UNIX_SECONDS) + leapSeconds) * GnssMathUtils.NANOS_PER_SECOND

    internal data class GpsCnavIscRecord(
        val iscL1CaSeconds: Double,
        val iscL5I5Seconds: Double,
        val lastUpdateElapsedMs: Long,
    )

    /**
     * Process GNSS navigation messages to extract Ephemeris data.
     */
    fun processNavigationMessage(message: GnssNavigationMessage) {
        // Accept STATUS_PARITY_PASSED and STATUS_UNKNOWN (many chipsets report UNKNOWN
        // even for valid messages; rejecting them blocks all ephemeris acquisition).
        if (message.status == GnssNavigationMessage.STATUS_PARITY_REBUILT) {
            return
        }

        var constellation = navigationMessageConstellationType(message.type)
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
                        recordGpsCnavMt30Isc(svid, data)
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
                                ephemerisStore.storeEphemeris(satKey, ephemeris, sourceLabel = "GAL_I")
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
                    if (effectiveStringNumber in 1..4 && data.size >= 11) {
                        val nowMs = GnssPlatform.timeSource.elapsedRealtimeMs()
                        val strings = glonassStringStorage.getOrPut(satKey) { mutableMapOf() }
                        // Evict strings older than 30 s: strings 1–4 broadcast within 8 s of one
                        // 30 s frame, so older fragments may belong to a different tb boundary.
                        strings.entries.removeAll { (_, entry) -> nowMs - entry.second > GLONASS_STRING_MAX_AGE_MS }
                        if (effectiveStringNumber == 2) {
                            val newTb = GnssMathUtils.getBits(data, 9, 7)
                            val previousString2 = strings[2]?.first
                            if (previousString2 != null && GnssMathUtils.getBits(previousString2, 9, 7) != newTb) {
                                strings.keys.removeAll { it != 2 }
                            }
                        }
                        strings[effectiveStringNumber] = data to nowMs

                        if (strings.size >= 4) {
                            val gpsTow =
                                if (hasGpsTimeOfWeek) {
                                    lastGpsTimeOfWeekSeconds
                                } else {
                                    (currentSystemGpsTimeNanos(lastKnownLeapSeconds) / 1e9).mod(GnssMathUtils.SECONDS_IN_WEEK)
                                }
                            val ephemeris =
                                GlonassNavMessageParser.parseGlonassEphemeris(
                                    svid = svid,
                                    strings = strings.mapValues { (_, entry) -> entry.first },
                                    gpsTimeOfWeekSeconds = gpsTow,
                                    leapSeconds = lastKnownLeapSeconds,
                                )
                            if (ephemeris != null) {
                                ephemerisStore.storeEphemeris(satKey, ephemeris, sourceLabel = "GLO_L1OF")
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
                if (suppressSbas) return
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
                                ephemerisStore.storeEphemeris(satKey, ephemeris, sourceLabel = "BDS_D1")
                            } else {
                                Log.w(TAG, "BDS D1 ephemeris parse returned null svid=$svid — subframe data may be corrupted")
                            }
                        }
                    }
                }
            }
        }
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
                    ephemerisStore.storeEphemeris(satKey, ephemeris, sourceLabel = "SBAS_MT9")
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
                ephemerisStore.storeEphemeris(
                    satKey = satKey,
                    ephemeris = ephemeris.copy(constellationType = constellationType),
                    sourceLabel = if (constellationType == GnssStatus.CONSTELLATION_QZSS) "QZS_L1CA" else "GPS_L1CA",
                )
            } else {
                Log.w(TAG, "GPS/QZSS ephemeris parse returned null svid=$svid — subframe data may be corrupted")
            }
        }
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
     * Adjusts dual-frequency **code** pseudoranges before the ionosphere-free linear combination
     * (see [computePosition]).
     *
     * - **GPS with recent CNAV MT30:** applies **+ISC·c** to L1 and L5 per IS-GPS-705; the IF
     *   observable is then referenced to L1 P(Y) and pairs with the −TGD clock.
     * - **GPS LNAV / Galileo BGD fallback:** **no adjustment** on aligned pairs; for Galileo
     *   I/NAV on E1/E5b the IF bias is 0 (`clockIncludesGroupDelay = false`), while on E1/E5a
     *   it pairs with `BGD(E1,E5b) − BGD(E1,E5a)` (`clockIncludesGroupDelay = true`).
     * - **BeiDou D1/RINEX (B3I clock reference):** subtracts `c·TGD1` from `B1I` and `c·TGD2`
     *   from `B2I` before forming the IF combination (`0` on `B3I`, `B1C`, `B2a`), aligning the
     *   IF observable to the B3I broadcast clock (`clockIncludesGroupDelay = false`).
     */
    internal fun recordGpsCnavMt30Isc(
        svid: Int,
        data: ByteArray,
    ) {
        GpsCnavMt30Parser.parseL1L5IscIfComplete(data)?.let { isc ->
            gpsCnavIscMap[GnssStatus.CONSTELLATION_GPS to svid] =
                GpsCnavIscRecord(
                    iscL1CaSeconds = isc.iscL1CaSeconds,
                    iscL5I5Seconds = isc.iscL5I5Seconds,
                    lastUpdateElapsedMs = SystemClock.elapsedRealtime(),
                )
        }
    }

    internal fun codePseudorangesForIonosphereFreeMeters(
        satKey: Pair<Int, Int>,
        l1Meters: Double,
        l5Meters: Double,
        ephemeris: SatelliteEphemeris?,
        upperBandHz: Double =
            if (satKey.first == GnssStatus.CONSTELLATION_BEIDOU) {
                GnssMathUtils.B1I_FREQUENCY_HZ
            } else {
                GnssMathUtils.L1_FREQUENCY_HZ
            },
        lowerBandHz: Double = GnssMathUtils.L5_FREQUENCY_HZ,
    ): IfCodeAdjustment {
        if (satKey.first == GnssStatus.CONSTELLATION_BEIDOU) {
            val upperBand = GnssSignalBand.fromFrequencyHz(upperBandHz, Constellation.BEIDOU)
            val lowerBand = GnssSignalBand.fromFrequencyHz(lowerBandHz, Constellation.BEIDOU)
            val upperAdjusted =
                if (ephemeris != null &&
                    upperBand == GnssSignalBand.B1I &&
                    ephemeris.tgd.isFinite() &&
                    ephemeris.tgd != 0.0
                ) {
                    l1Meters - ephemeris.tgd * SPEED_OF_LIGHT
                } else {
                    l1Meters
                }
            val lowerAdjusted =
                if (ephemeris != null &&
                    lowerBand == GnssSignalBand.E5B_B2I &&
                    ephemeris.beidouTgd2Seconds.isFinite() &&
                    ephemeris.beidouTgd2Seconds != 0.0
                ) {
                    l5Meters - ephemeris.beidouTgd2Seconds * SPEED_OF_LIGHT
                } else {
                    l5Meters
                }
            return IfCodeAdjustment(upperAdjusted, lowerAdjusted, clockIncludesGroupDelay = false)
        }
        if (satKey.first == GnssStatus.CONSTELLATION_GALILEO && ephemeris != null) {
            val lowerBand = GnssSignalBand.fromFrequencyHz(lowerBandHz, Constellation.GALILEO)
            val clockIncludesGroupDelay =
                if (lowerBand == GnssSignalBand.E5B_B2I) {
                    !ephemeris.galileoInavClock
                } else {
                    ephemeris.galileoInavClock
                }
            return IfCodeAdjustment(l1Meters, l5Meters, clockIncludesGroupDelay = clockIncludesGroupDelay)
        }
        if (satKey.first == GnssStatus.CONSTELLATION_GPS) {
            val upperBand = GnssSignalBand.fromFrequencyHz(upperBandHz, Constellation.GPS)
            val lowerBand = GnssSignalBand.fromFrequencyHz(lowerBandHz, Constellation.GPS)
            if (upperBand == GnssSignalBand.L1_E1_B1C && lowerBand == GnssSignalBand.L5_E5A_B2A) {
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
        }
        return IfCodeAdjustment(l1Meters, l5Meters, clockIncludesGroupDelay = false)
    }

    /**
     * Returns the SBAS fast pseudorange correction (PRC) for a GPS satellite, in meters.
     * Scans all tracked SBAS PRN masks and returns the first matching fresh PRC, or 0.0 if none.
     * The PRC should be *added* to the satellite clock correction.
     *
     * Stale corrections older than [GnssMathUtils.SBAS_FAST_CORRECTION_STALENESS_MS] are discarded.
     * Only applies to GPS satellites (SBAS monitors GPS L1 primarily).
     */
    internal fun getSbasFastCorrectionMeters(
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
    internal fun getSbasLongTermClockCorrectionMeters(
        constellationType: Int,
        svid: Int,
        ephemerisIode: Int,
    ): Double {
        if (constellationType != GnssStatus.CONSTELLATION_GPS) return 0.0
        val ltc = getSbasLongTermCorrection(svid, ephemerisIode) ?: return 0.0
        return ltc.deltaAf0 * GnssMathUtils.SPEED_OF_LIGHT
    }

    /** Clears SBAS fast/long-term corrections (session reset with clearCorrections). */
    internal fun clearSbasCorrections() {
        sbasFastCorrections.clear()
        sbasLongTermCorrections.clear()
    }

    /** Clears all navigation-message state (session reset without preserving ephemeris). */
    internal fun clearNavigationState() {
        subframeStorage.clear()
        glonassStringStorage.clear()
        galileoWordStorage.clear()
        gpsCnavIscMap.clear()
        gpsLastBits.clear()
        sbasPrnMasks.clear()
        ionoAlpha = null
        ionoBeta = null
        hasGpsTimeOfWeek = false
        lastGpsTimeOfWeekSeconds = 0.0
    }

    companion object {
        private const val TAG = "GnssMeasurementProcessor"
        private const val GLONASS_STRING_MAX_AGE_MS = 30_000L

        /** GPS TLM word preamble (IS-GPS-200 §20.3.3.1) and its D30*-complemented form. */
        private const val TLM_PREAMBLE = 0x8B
        private const val TLM_PREAMBLE_INVERTED = 0x74
    }
}
