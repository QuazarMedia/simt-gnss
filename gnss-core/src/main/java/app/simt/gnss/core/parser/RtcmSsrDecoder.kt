package app.simt.gnss.core.parser
import app.simt.gnss.core.correction.SsrCorrections
import app.simt.gnss.core.model.Constellation
import app.simt.gnss.core.platform.GnssPlatform

/**
 * Decodes RTCM 3 SSR (State Space Representation) messages into [SsrCorrections] objects.
 *
 * Supported RTCM3 message types:
 * - **Separate orbit**: 1057 (GPS), 1063 (GLONASS), 1240 (Galileo), 1246 (QZSS), 1258 (BeiDou)
 * - **Separate clock**: 1058 (GPS), 1064 (GLONASS), 1241 (Galileo), 1247 (QZSS), 1259 (BeiDou)
 * - **Code bias**: 1059 (GPS), 1065 (GLONASS), 1242 (Galileo), 1248 (QZSS), 1260 (BeiDou)
 * - **Combined orbit+clock**: 1060 (GPS), 1066 (GLONASS), 1243 (Galileo), 1249 (QZSS), 1261 (BeiDou)
 * - **High-rate clock**: 1062 (GPS), 1068 (GLONASS), 1245 (Galileo), 1251 (QZSS), 1263 (BeiDou)
 * - **Phase bias**: 1265 (GPS), 1266 (GLONASS), 1267 (Galileo), 1268 (QZSS), 1270 (BeiDou)
 * - **IGS SSR**: 4076 (v1 subtypes 21–26 GPS, 41–46 GLONASS, 61–66 Galileo, 81–86 QZSS, 101–106 BeiDou)
 *
 * Reference: RTCM Standard 10403.3, IGS SSR Format v1.00 (igs.org/rts/formats)
 */
object RtcmSsrDecoder {
    private const val TAG = "RtcmSsrDecoder"
    private const val IGS_SSR_MESSAGE_TYPE = 4076
    private const val IGS_SSR_VERSION_1 = 1

    private enum class SsrMessageKind {
        ORBIT,
        CLOCK,
        COMBINED,
        HIGH_RATE_CLOCK,
        CODE_BIAS,
        PHASE_BIAS,
    }

    /** RTCM SSR update-interval indicator (0..15) to seconds. Same table as RTKLIB `ssr_udi`. */
    internal fun ssrUpdateIntervalSeconds(index: Int): Double =
        when (index) {
            0 -> 1.0
            1 -> 2.0
            2 -> 5.0
            3 -> 10.0
            4 -> 15.0
            5 -> 30.0
            6 -> 60.0
            7 -> 120.0
            8 -> 240.0
            9 -> 300.0
            10 -> 600.0
            11 -> 900.0
            12 -> 1800.0
            13 -> 3600.0
            14 -> 7200.0
            15 -> 10800.0
            else -> 0.0
        }

    /**
     * Per-constellation SSR field widths (RTCM 10403.3 / IGS SSR; cf. RTKLIB `decode_ssr1..7`):
     * satellite-id bits, broadcast-IOD bits, and the extra BDS IODCRC width.
     * The epoch time is 20 bits of GPS/GST/BDT seconds-of-week — except GLONASS, whose
     * DF386 epoch is 17 bits of GLONASS time (UTC(SU) + 3 h) seconds-of-day.
     */
    private data class SsrLayout(
        val epochBits: Int,
        val satIdBits: Int,
        val iodBits: Int,
        val iodCrcBits: Int,
    )

    private fun layoutFor(constellationType: Int): SsrLayout =
        when (constellationType) {
            Constellation.GLONASS -> SsrLayout(epochBits = 17, satIdBits = 5, iodBits = 8, iodCrcBits = 0)
            Constellation.GALILEO -> SsrLayout(epochBits = 20, satIdBits = 6, iodBits = 10, iodCrcBits = 0)
            Constellation.QZSS -> SsrLayout(epochBits = 20, satIdBits = 4, iodBits = 8, iodCrcBits = 0)
            Constellation.BEIDOU -> SsrLayout(epochBits = 20, satIdBits = 6, iodBits = 10, iodCrcBits = 24)
            else -> SsrLayout(epochBits = 20, satIdBits = 6, iodBits = 8, iodCrcBits = 0) // GPS
        }

    /**
     * IGS SSR 4076 v1.00 uses uniform 20-bit GNSS epoch, 6-bit satId, and 0-bit IODCRC,
     * with 10-bit IODnav for Galileo and 8-bit IOD for all other constellations.
     */
    private fun igsLayoutFor(constellationType: Int): SsrLayout =
        SsrLayout(
            epochBits = 20,
            satIdBits = 6,
            iodBits = if (constellationType == Constellation.GALILEO) 10 else 8,
            iodCrcBits = 0,
        )

    /** Orbit (22+20+20+21+19+19) field bits, excluding satId/IOD/IODCRC. */
    private const val ORBIT_FIELD_BITS = 121

    /** Clock (22+21+27) field bits, excluding satId. */
    private const val CLOCK_FIELD_BITS = 70

    /** Orbit + clock field bits, excluding satId/IOD/IODCRC. */
    private const val ORBIT_CLOCK_FIELD_BITS = ORBIT_FIELD_BITS + CLOCK_FIELD_BITS

    /** High-rate clock (22) field bits, excluding satId. */
    private const val HIGH_RATE_CLOCK_FIELD_BITS = 22

    /** Separate orbit message types per constellation. */
    private val ORBIT_TYPES =
        mapOf(
            1057 to Constellation.GPS,
            1063 to Constellation.GLONASS,
            1240 to Constellation.GALILEO,
            1246 to Constellation.QZSS,
            1258 to Constellation.BEIDOU,
        )

    /** Separate clock message types per constellation. */
    private val CLOCK_TYPES =
        mapOf(
            1058 to Constellation.GPS,
            1064 to Constellation.GLONASS,
            1241 to Constellation.GALILEO,
            1247 to Constellation.QZSS,
            1259 to Constellation.BEIDOU,
        )

    /** Code bias message types. */
    private val CODE_BIAS_TYPES =
        mapOf(
            1059 to Constellation.GPS,
            1065 to Constellation.GLONASS,
            1242 to Constellation.GALILEO,
            1248 to Constellation.QZSS,
            1260 to Constellation.BEIDOU,
        )

    /** Orbit+clock combined message types per constellation. */
    private val ORBIT_CLOCK_TYPES =
        mapOf(
            1060 to Constellation.GPS,
            1066 to Constellation.GLONASS,
            1243 to Constellation.GALILEO,
            1249 to Constellation.QZSS,
            1261 to Constellation.BEIDOU,
        )

    /** High-rate clock message types per constellation. */
    private val HIGH_RATE_CLOCK_TYPES =
        mapOf(
            1062 to Constellation.GPS,
            1068 to Constellation.GLONASS,
            1245 to Constellation.GALILEO,
            1251 to Constellation.QZSS,
            1263 to Constellation.BEIDOU,
        )

    /** Phase bias message types. */
    private val PHASE_BIAS_TYPES =
        mapOf(
            1265 to Constellation.GPS,
            1266 to Constellation.GLONASS,
            1267 to Constellation.GALILEO,
            1268 to Constellation.QZSS,
            1270 to Constellation.BEIDOU,
        )

    /** Maps an RTCM/IGS SSR satellite id to the platform SVID convention. */
    private fun svidFor(
        constellationType: Int,
        satId: Int,
    ): Int = if (constellationType == Constellation.QZSS && satId in 1..10) 192 + satId else satId

    private fun igsSubtypeMapping(subtype: Int): Pair<Int, SsrMessageKind>? {
        val constellation =
            when (subtype / 20) {
                1 -> Constellation.GPS
                2 -> Constellation.GLONASS
                3 -> Constellation.GALILEO
                4 -> Constellation.QZSS
                5 -> Constellation.BEIDOU
                else -> return null
            }
        val kind =
            when (subtype % 20) {
                1 -> SsrMessageKind.ORBIT
                2 -> SsrMessageKind.CLOCK
                3 -> SsrMessageKind.COMBINED
                4 -> SsrMessageKind.HIGH_RATE_CLOCK
                5 -> SsrMessageKind.CODE_BIAS
                6 -> SsrMessageKind.PHASE_BIAS
                else -> return null
            }
        return constellation to kind
    }

    /**
     * Returns whether the given message type is handled by this decoder.
     */
    fun isSupported(messageType: Int): Boolean =
        messageType == IGS_SSR_MESSAGE_TYPE ||
            messageType in ORBIT_TYPES ||
            messageType in CLOCK_TYPES ||
            messageType in CODE_BIAS_TYPES ||
            messageType in ORBIT_CLOCK_TYPES ||
            messageType in HIGH_RATE_CLOCK_TYPES ||
            messageType in PHASE_BIAS_TYPES

    /**
     * Decode an RTCM3 SSR message and store results in the given [SsrCorrections.Store].
     *
     * @param frame Parsed RTCM3 frame
     * @param store SSR correction store to populate
     */
    fun decode(
        frame: Rtcm3FrameParser.Frame,
        store: SsrCorrections.Store,
    ) {
        val msgType = frame.messageType
        try {
            when {
                msgType == IGS_SSR_MESSAGE_TYPE -> decodeIgsSsr(frame.payload, store)
                msgType in ORBIT_TYPES ->
                    decodeStandard(frame.payload, msgType, ORBIT_TYPES[msgType]!!, SsrMessageKind.ORBIT, store)
                msgType in CLOCK_TYPES ->
                    decodeStandard(frame.payload, msgType, CLOCK_TYPES[msgType]!!, SsrMessageKind.CLOCK, store)
                msgType in CODE_BIAS_TYPES ->
                    decodeStandard(frame.payload, msgType, CODE_BIAS_TYPES[msgType]!!, SsrMessageKind.CODE_BIAS, store)
                msgType in ORBIT_CLOCK_TYPES ->
                    decodeStandard(frame.payload, msgType, ORBIT_CLOCK_TYPES[msgType]!!, SsrMessageKind.COMBINED, store)
                msgType in HIGH_RATE_CLOCK_TYPES ->
                    decodeStandard(frame.payload, msgType, HIGH_RATE_CLOCK_TYPES[msgType]!!, SsrMessageKind.HIGH_RATE_CLOCK, store)
                msgType in PHASE_BIAS_TYPES ->
                    decodeStandard(frame.payload, msgType, PHASE_BIAS_TYPES[msgType]!!, SsrMessageKind.PHASE_BIAS, store)
            }
        } catch (e: Exception) {
            GnssPlatform.logger.w(TAG, "Failed to decode RTCM SSR msg $msgType: ${e.message}")
        }
    }

    // ---- Standard RTCM SSR header & dispatch ----

    private fun decodeStandard(
        payload: ByteArray,
        msgType: Int,
        constellationType: Int,
        kind: SsrMessageKind,
        store: SsrCorrections.Store,
    ) {
        val reader = BitReader(payload)
        val layout = layoutFor(constellationType)
        val hasDatumBit = kind == SsrMessageKind.ORBIT || kind == SsrMessageKind.COMBINED
        val hasPhaseBiasFlags = kind == SsrMessageKind.PHASE_BIAS
        // 12 (msgType) + epochBits + 4 (udi) + 1 (multi) + datum + 4 (iodSsr) + 16 (provider) + 4 (solution) + flags + 6 (numSats)
        val headerBits = 47 + layout.epochBits + (if (hasDatumBit) 1 else 0) + (if (hasPhaseBiasFlags) 2 else 0)
        if (!reader.hasRemaining(headerBits)) return

        reader.skip(12) // message type (already known)
        // SSR header. Note: orbit-bearing messages carry a 1-bit Satellite Reference Datum (DF375)
        // between the multiple-message indicator and IOD SSR — clock/bias messages do not.
        val epochTime = reader.readBits(layout.epochBits).toDouble()
        val updateIntervalSeconds = ssrUpdateIntervalSeconds(reader.readBits(4))
        reader.skip(1) // multiple message indicator
        if (hasDatumBit) reader.skip(1) // DF375: 0 = ITRF, 1 = regional
        val iodSsr = reader.readBits(4)
        reader.skip(16) // provider ID
        reader.skip(4) // solution ID
        if (hasPhaseBiasFlags) reader.skip(2) // dispersive bias consistency + MW consistency
        val numSats = reader.readBits(6)

        val epochTimescale = SsrCorrections.defaultEpochTimescale(constellationType)
        decodePayloadBlocks(
            reader = reader,
            msgLabel = "msg $msgType",
            constellationType = constellationType,
            kind = kind,
            layout = layout,
            epochTime = epochTime,
            updateIntervalSeconds = updateIntervalSeconds,
            iodSsr = iodSsr,
            epochTimescale = epochTimescale,
            numSats = numSats,
            store = store,
        )
    }

    // ---- IGS SSR 4076 header & dispatch ----

    private fun decodeIgsSsr(
        payload: ByteArray,
        store: SsrCorrections.Store,
    ) {
        val reader = BitReader(payload)
        // 12 (msgType) + 3 (version) + 8 (subtype)
        if (!reader.hasRemaining(23)) return
        reader.skip(12) // 4076
        val version = reader.readBits(3)
        if (version != IGS_SSR_VERSION_1) return
        val subtype = reader.readBits(8)
        val (constellationType, kind) = igsSubtypeMapping(subtype) ?: return

        val hasDatumBit = kind == SsrMessageKind.ORBIT || kind == SsrMessageKind.COMBINED
        val hasPhaseBiasFlags = kind == SsrMessageKind.PHASE_BIAS
        // IGS SSR common header: 20 (epoch) + 4 (udi) + 1 (multi) + 4 (iodSsr) + 16 (provider) + 4 (solution) = 49,
        // followed by subtype-specific header flags (datum or phase-bias consistency) and 6-bit numSats.
        val remainingHeaderBits = 55 + (if (hasDatumBit) 1 else 0) + (if (hasPhaseBiasFlags) 2 else 0)
        if (!reader.hasRemaining(remainingHeaderBits)) return

        val layout = igsLayoutFor(constellationType)
        val epochTime = reader.readBits(layout.epochBits).toDouble()
        val updateIntervalSeconds = ssrUpdateIntervalSeconds(reader.readBits(4))
        reader.skip(1) // multiple message indicator
        val iodSsr = reader.readBits(4)
        reader.skip(16) // provider ID
        reader.skip(4) // solution ID
        if (hasDatumBit) reader.skip(1) // DF375 satellite reference datum (after solutionId in IGS SSR v1.00)
        if (hasPhaseBiasFlags) reader.skip(2) // dispersive bias consistency + MW consistency
        val numSats = reader.readBits(6)

        // IGS SSR v1.00 §3.2: epoch time is GPS seconds of week for all constellations.
        decodePayloadBlocks(
            reader = reader,
            msgLabel = "4076/$subtype",
            constellationType = constellationType,
            kind = kind,
            layout = layout,
            epochTime = epochTime,
            updateIntervalSeconds = updateIntervalSeconds,
            iodSsr = iodSsr,
            epochTimescale = SsrCorrections.EpochTimescale.GPST,
            numSats = numSats,
            store = store,
        )
    }

    // ---- Shared per-satellite payload decoding ----

    private fun decodePayloadBlocks(
        reader: BitReader,
        msgLabel: String,
        constellationType: Int,
        kind: SsrMessageKind,
        layout: SsrLayout,
        epochTime: Double,
        updateIntervalSeconds: Double,
        iodSsr: Int,
        epochTimescale: SsrCorrections.EpochTimescale,
        numSats: Int,
        store: SsrCorrections.Store,
    ) {
        val now = GnssPlatform.timeSource.elapsedRealtimeMs()
        when (kind) {
            SsrMessageKind.ORBIT -> {
                val perSatBits = layout.satIdBits + layout.iodBits + layout.iodCrcBits + ORBIT_FIELD_BITS
                for (s in 0 until numSats) {
                    if (!reader.hasRemaining(perSatBits)) break
                    val svid = svidFor(constellationType, reader.readBits(layout.satIdBits))
                    val iod = reader.readBits(layout.iodBits)
                    if (layout.iodCrcBits > 0) reader.skip(layout.iodCrcBits)
                    store.putOrbit(readOrbit(reader, constellationType, svid, iod, epochTime, updateIntervalSeconds, now, iodSsr, epochTimescale))
                }
            }
            SsrMessageKind.CLOCK -> {
                val perSatBits = layout.satIdBits + CLOCK_FIELD_BITS
                for (s in 0 until numSats) {
                    if (!reader.hasRemaining(perSatBits)) break
                    val svid = svidFor(constellationType, reader.readBits(layout.satIdBits))
                    val matchingIod = store.getOrbit(constellationType, svid)?.takeIf { it.iodSsr == iodSsr }?.iod ?: -1
                    store.putClock(readClock(reader, constellationType, svid, matchingIod, epochTime, updateIntervalSeconds, now, iodSsr, epochTimescale))
                }
            }
            SsrMessageKind.COMBINED -> {
                val perSatBits = layout.satIdBits + layout.iodBits + layout.iodCrcBits + ORBIT_CLOCK_FIELD_BITS
                for (s in 0 until numSats) {
                    // Guard with the true block size: a truncated message must not store
                    // partially zero-filled corrections under a garbage satellite id.
                    if (!reader.hasRemaining(perSatBits)) break
                    val svid = svidFor(constellationType, reader.readBits(layout.satIdBits))
                    val iod = reader.readBits(layout.iodBits)
                    if (layout.iodCrcBits > 0) reader.skip(layout.iodCrcBits)
                    store.putOrbit(readOrbit(reader, constellationType, svid, iod, epochTime, updateIntervalSeconds, now, iodSsr, epochTimescale))
                    store.putClock(readClock(reader, constellationType, svid, iod, epochTime, updateIntervalSeconds, now, iodSsr, epochTimescale))
                }
            }
            SsrMessageKind.HIGH_RATE_CLOCK -> {
                val perSatBits = layout.satIdBits + HIGH_RATE_CLOCK_FIELD_BITS
                for (s in 0 until numSats) {
                    if (!reader.hasRemaining(perSatBits)) break
                    val svid = svidFor(constellationType, reader.readBits(layout.satIdBits))
                    val deltaClockMeters = reader.readSignedBits(22) * 0.1e-3 // 0.1 mm → meters
                    store.putHighRateClock(
                        SsrCorrections.HighRateClockCorrection(
                            constellationType = constellationType,
                            svid = svid,
                            deltaClockMeters = deltaClockMeters,
                            iodSsr = iodSsr,
                            epochTime = epochTime,
                            updateIntervalSeconds = updateIntervalSeconds,
                            epochTimescale = epochTimescale,
                            receivedAtMs = now,
                        ),
                    )
                }
            }
            SsrMessageKind.CODE_BIAS -> {
                for (s in 0 until numSats) {
                    if (!reader.hasRemaining(layout.satIdBits + 5)) break
                    val svid = svidFor(constellationType, reader.readBits(layout.satIdBits))
                    val numBiases = reader.readBits(5)
                    for (b in 0 until numBiases) {
                        if (!reader.hasRemaining(19)) break
                        val signalId = reader.readBits(5)
                        val bias = reader.readSignedBits(14) * 0.01 // 0.01 m resolution
                        store.putCodeBias(
                            SsrCorrections.CodeBias(
                                constellationType = constellationType,
                                svid = svid,
                                signalId = signalId,
                                bias = bias,
                                receivedAtMs = now,
                            ),
                        )
                    }
                }
            }
            SsrMessageKind.PHASE_BIAS -> {
                for (s in 0 until numSats) {
                    // satId + numBiases(5) + yaw angle (9, ×1/256 semicircle) + yaw rate (8, ×1/8192 sc/s)
                    if (!reader.hasRemaining(layout.satIdBits + 5 + 9 + 8)) break
                    val svid = svidFor(constellationType, reader.readBits(layout.satIdBits))
                    val numBiases = reader.readBits(5)
                    reader.skip(9) // yaw angle
                    reader.skip(8) // yaw rate
                    for (b in 0 until numBiases) {
                        if (!reader.hasRemaining(32)) break
                        val signalId = reader.readBits(5)
                        val integerIndicator = reader.readBits(1)
                        reader.skip(2) // wide lane integer indicator
                        reader.skip(4) // discontinuity counter
                        val bias = reader.readSignedBits(20) * 0.0001 // 0.1 mm resolution → meters
                        store.putPhaseBias(
                            SsrCorrections.PhaseBias(
                                constellationType = constellationType,
                                svid = svid,
                                signalId = signalId,
                                bias = bias,
                                integerIndicator = integerIndicator,
                                receivedAtMs = now,
                            ),
                        )
                    }
                }
            }
        }
        GnssPlatform.logger.d(TAG, "Decoded SSR $kind $msgLabel: $numSats sats, epoch=$epochTime")
    }

    private fun readOrbit(
        reader: BitReader,
        constellationType: Int,
        svid: Int,
        iod: Int,
        epochTime: Double,
        updateIntervalSeconds: Double,
        receivedAtMs: Long,
        iodSsr: Int,
        epochTimescale: SsrCorrections.EpochTimescale,
    ): SsrCorrections.OrbitCorrection {
        val deltaRadial = reader.readSignedBits(22) * 0.1e-3 // 0.1 mm → meters
        val deltaAlongTrack = reader.readSignedBits(20) * 0.4e-3 // 0.4 mm → meters
        val deltaCrossTrack = reader.readSignedBits(20) * 0.4e-3
        val dotRadial = reader.readSignedBits(21) * 0.001e-3 // 0.001 mm/s → m/s
        val dotAlongTrack = reader.readSignedBits(19) * 0.004e-3
        val dotCrossTrack = reader.readSignedBits(19) * 0.004e-3
        return SsrCorrections.OrbitCorrection(
            constellationType = constellationType,
            svid = svid,
            deltaRadial = deltaRadial,
            deltaAlongTrack = deltaAlongTrack,
            deltaCrossTrack = deltaCrossTrack,
            dotRadial = dotRadial,
            dotAlongTrack = dotAlongTrack,
            dotCrossTrack = dotCrossTrack,
            iod = iod,
            epochTime = epochTime,
            updateIntervalSeconds = updateIntervalSeconds,
            receivedAtMs = receivedAtMs,
            iodSsr = iodSsr,
            epochTimescale = epochTimescale,
        )
    }

    private fun readClock(
        reader: BitReader,
        constellationType: Int,
        svid: Int,
        iod: Int,
        epochTime: Double,
        updateIntervalSeconds: Double,
        receivedAtMs: Long,
        iodSsr: Int,
        epochTimescale: SsrCorrections.EpochTimescale,
    ): SsrCorrections.ClockCorrection {
        val c0 = reader.readSignedBits(22) * 0.1e-3 // 0.1 mm → meters
        val c1 = reader.readSignedBits(21) * 0.001e-3 // 0.001 mm/s → m/s
        val c2 = reader.readSignedBits(27) * 0.00002e-3 // 0.00002 mm/s² → m/s²
        return SsrCorrections.ClockCorrection(
            constellationType = constellationType,
            svid = svid,
            c0 = c0,
            c1 = c1,
            c2 = c2,
            iod = iod,
            epochTime = epochTime,
            updateIntervalSeconds = updateIntervalSeconds,
            receivedAtMs = receivedAtMs,
            iodSsr = iodSsr,
            epochTimescale = epochTimescale,
        )
    }

    // ---- Bit reader utility ----

    /**
     * MSB-first bit reader over a byte array.
     */
    private class BitReader(private val data: ByteArray) {
        private var bitPos = 0

        fun hasRemaining(bits: Int): Boolean = bitPos + bits <= data.size * 8

        fun readBits(n: Int): Int {
            var value = 0
            for (i in 0 until n) {
                val byteIdx = (bitPos + i) / 8
                val bitIdx = 7 - ((bitPos + i) % 8)
                if (byteIdx < data.size) {
                    value = (value shl 1) or ((data[byteIdx].toInt() shr bitIdx) and 1)
                } else {
                    value = value shl 1
                }
            }
            bitPos += n
            return value
        }

        fun readSignedBits(n: Int): Int {
            val unsigned = readBits(n)
            return if (unsigned and (1 shl (n - 1)) != 0) {
                unsigned or ((-1) shl n)
            } else {
                unsigned
            }
        }

        fun skip(n: Int) {
            bitPos += n
        }
    }
}
