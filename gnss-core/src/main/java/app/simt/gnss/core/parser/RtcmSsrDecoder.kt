package app.simt.gnss.core.parser
import app.simt.gnss.core.correction.SsrCorrections
import app.simt.gnss.core.model.Constellation
import app.simt.gnss.core.platform.GnssPlatform

/**
 * Decodes RTCM 3 SSR (State Space Representation) messages into [SsrCorrections] objects.
 *
 * Supported RTCM3 message types:
 * - **Combined orbit+clock**: 1060 (GPS), 1066 (GLONASS), 1243 (Galileo), 1261 (BeiDou)
 * - **Code bias**: 1059 (GPS), 1065 (GLONASS), 1242 (Galileo), 1260 (BeiDou)
 * - **Phase bias**: 1265 (GPS), 1266 (GLONASS), 1267 (Galileo), 1270 (BeiDou)
 *
 * Reference: RTCM Standard 10403.3, IGS SSR Format (igs.org/rts/formats)
 */
object RtcmSsrDecoder {
    private const val TAG = "RtcmSsrDecoder"

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
     * DF386 epoch is 17 bits of UTC(SU) seconds-of-day.
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
            Constellation.BEIDOU -> SsrLayout(epochBits = 20, satIdBits = 6, iodBits = 10, iodCrcBits = 24)
            else -> SsrLayout(epochBits = 20, satIdBits = 6, iodBits = 8, iodCrcBits = 0) // GPS
        }

    /** Orbit (22+20+20+21+19+19) + clock (22+21+27) field bits, excluding satId/IOD/IODCRC. */
    private const val ORBIT_CLOCK_FIELD_BITS = 191

    /** Orbit+clock combined message types per constellation. */
    private val ORBIT_CLOCK_TYPES =
        mapOf(
            1060 to Constellation.GPS,
            1066 to Constellation.GLONASS,
            1243 to Constellation.GALILEO,
            1261 to Constellation.BEIDOU,
        )

    /** Code bias message types. */
    private val CODE_BIAS_TYPES =
        mapOf(
            1059 to Constellation.GPS,
            1065 to Constellation.GLONASS,
            1242 to Constellation.GALILEO,
            1260 to Constellation.BEIDOU,
        )

    /** Phase bias message types. */
    private val PHASE_BIAS_TYPES =
        mapOf(
            1265 to Constellation.GPS,
            1266 to Constellation.GLONASS,
            1267 to Constellation.GALILEO,
            1270 to Constellation.BEIDOU,
        )

    /**
     * Returns whether the given message type is handled by this decoder.
     */
    fun isSupported(messageType: Int): Boolean = messageType in ORBIT_CLOCK_TYPES || messageType in CODE_BIAS_TYPES || messageType in PHASE_BIAS_TYPES

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
                msgType in ORBIT_CLOCK_TYPES -> decodeOrbitClock(frame.payload, msgType, ORBIT_CLOCK_TYPES[msgType]!!, store)
                msgType in CODE_BIAS_TYPES -> decodeCodeBias(frame.payload, msgType, CODE_BIAS_TYPES[msgType]!!, store)
                msgType in PHASE_BIAS_TYPES -> decodePhaseBias(frame.payload, msgType, PHASE_BIAS_TYPES[msgType]!!, store)
            }
        } catch (e: Exception) {
            GnssPlatform.logger.w(TAG, "Failed to decode RTCM SSR msg $msgType: ${e.message}")
        }
    }

    // ---- Combined orbit + clock (message types 1060, 1066, 1243, 1261) ----

    private fun decodeOrbitClock(
        payload: ByteArray,
        msgType: Int,
        constellationType: Int,
        store: SsrCorrections.Store,
    ) {
        val reader = BitReader(payload)
        val layout = layoutFor(constellationType)
        reader.skip(12) // message type (already known)

        // SSR header. Note: orbit-bearing messages (1060/1066/1243/1261) carry a 1-bit
        // Satellite Reference Datum (DF375) between the multiple-message indicator and
        // IOD SSR — code/phase-bias messages do not. Omitting it shifts every following
        // field by one bit and turns all decoded corrections into bounded garbage.
        val epochTime = reader.readBits(layout.epochBits).toDouble() // s of week (GLONASS: s of UTC(SU) day)
        val updateIntervalSeconds = ssrUpdateIntervalSeconds(reader.readBits(4))
        val multiMessageFlag = reader.readBits(1)
        val satelliteReferenceDatum = reader.readBits(1) // DF375: 0 = ITRF, 1 = regional
        val iodSsr = reader.readBits(4) // IOD SSR
        val ssrProviderId = reader.readBits(16)
        val ssrSolutionId = reader.readBits(4)
        val numSats = reader.readBits(6)

        val now = GnssPlatform.timeSource.elapsedRealtimeMs()
        val perSatelliteBits = layout.satIdBits + layout.iodBits + layout.iodCrcBits + ORBIT_CLOCK_FIELD_BITS

        for (s in 0 until numSats) {
            // Guard with the true block size: a truncated message must not store
            // partially zero-filled corrections under a garbage satellite id.
            if (!reader.hasRemaining(perSatelliteBits)) break

            val satId = reader.readBits(layout.satIdBits)
            val iod = reader.readBits(layout.iodBits) // IOD of the matching broadcast ephemeris
            if (layout.iodCrcBits > 0) reader.skip(layout.iodCrcBits) // BDS IODCRC

            // Orbit corrections (radial, along-track, cross-track) in mm
            val deltaRadial = reader.readSignedBits(22) * 0.1e-3 // 0.1 mm → meters
            val deltaAlongTrack = reader.readSignedBits(20) * 0.4e-3 // 0.4 mm → meters
            val deltaCrossTrack = reader.readSignedBits(20) * 0.4e-3

            // Orbit rate corrections in mm/s
            val dotRadial = reader.readSignedBits(21) * 0.001e-3 // 0.001 mm/s → m/s
            val dotAlongTrack = reader.readSignedBits(19) * 0.004e-3
            val dotCrossTrack = reader.readSignedBits(19) * 0.004e-3

            // Clock correction C0, C1, C2
            val c0 = reader.readSignedBits(22) * 0.1e-3 // 0.1 mm → meters
            val c1 = reader.readSignedBits(21) * 0.001e-3 // 0.001 mm/s → m/s
            val c2 = reader.readSignedBits(27) * 0.00002e-3 // 0.00002 mm/s² → m/s²

            store.putOrbit(
                SsrCorrections.OrbitCorrection(
                    constellationType = constellationType,
                    svid = satId,
                    deltaRadial = deltaRadial,
                    deltaAlongTrack = deltaAlongTrack,
                    deltaCrossTrack = deltaCrossTrack,
                    dotRadial = dotRadial,
                    dotAlongTrack = dotAlongTrack,
                    dotCrossTrack = dotCrossTrack,
                    iod = iod,
                    epochTime = epochTime,
                    updateIntervalSeconds = updateIntervalSeconds,
                    receivedAtMs = now,
                ),
            )

            store.putClock(
                SsrCorrections.ClockCorrection(
                    constellationType = constellationType,
                    svid = satId,
                    c0 = c0,
                    c1 = c1,
                    c2 = c2,
                    iod = iod,
                    epochTime = epochTime,
                    updateIntervalSeconds = updateIntervalSeconds,
                    receivedAtMs = now,
                ),
            )
        }
        GnssPlatform.logger.d(TAG, "Decoded SSR orbit+clock msg $msgType: $numSats sats, epoch=$epochTime")
    }

    // ---- Code bias (message types 1059, 1065, 1242, 1260) ----

    private fun decodeCodeBias(
        payload: ByteArray,
        msgType: Int,
        constellationType: Int,
        store: SsrCorrections.Store,
    ) {
        val reader = BitReader(payload)
        val layout = layoutFor(constellationType)
        reader.skip(12) // message type

        // SSR header — bias messages carry NO satellite-reference-datum bit (DF375).
        reader.skip(layout.epochBits) // epoch (GLONASS: 17-bit day seconds)
        reader.skip(4) // update interval
        reader.skip(1) // multi-message
        reader.skip(4) // IOD SSR
        reader.skip(16) // provider ID
        reader.skip(4) // solution ID
        val numSats = reader.readBits(6)

        val now = GnssPlatform.timeSource.elapsedRealtimeMs()

        for (s in 0 until numSats) {
            if (!reader.hasRemaining(layout.satIdBits + 5)) break

            val satId = reader.readBits(layout.satIdBits)
            val numBiases = reader.readBits(5)

            for (b in 0 until numBiases) {
                if (!reader.hasRemaining(19)) break
                val signalId = reader.readBits(5) // RTCM signal ID
                val bias = reader.readSignedBits(14) * 0.01 // 0.01 m resolution

                store.putCodeBias(
                    SsrCorrections.CodeBias(
                        constellationType = constellationType,
                        svid = satId,
                        signalId = signalId,
                        bias = bias,
                        receivedAtMs = now,
                    ),
                )
            }
        }
        GnssPlatform.logger.d(TAG, "Decoded SSR code bias msg $msgType: $numSats sats")
    }

    // ---- Phase bias (message types 1265, 1266, 1267, 1270) ----

    private fun decodePhaseBias(
        payload: ByteArray,
        msgType: Int,
        constellationType: Int,
        store: SsrCorrections.Store,
    ) {
        val reader = BitReader(payload)
        val layout = layoutFor(constellationType)
        reader.skip(12) // message type

        // SSR header — bias messages carry NO satellite-reference-datum bit (DF375).
        reader.skip(layout.epochBits) // epoch (GLONASS: 17-bit day seconds)
        reader.skip(4) // update interval
        reader.skip(1) // multi-message
        reader.skip(4) // IOD SSR
        reader.skip(16) // provider ID
        reader.skip(4) // solution ID
        reader.skip(1) // dispersive bias consistency flag
        reader.skip(1) // MW consistency flag
        val numSats = reader.readBits(6)

        val now = GnssPlatform.timeSource.elapsedRealtimeMs()

        for (s in 0 until numSats) {
            // satId + numBiases(5) + yaw angle (9, ×1/256 semicircle) + yaw rate (8, ×1/8192 sc/s)
            if (!reader.hasRemaining(layout.satIdBits + 5 + 9 + 8)) break

            val satId = reader.readBits(layout.satIdBits)
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
                        svid = satId,
                        signalId = signalId,
                        bias = bias,
                        integerIndicator = integerIndicator,
                        receivedAtMs = now,
                    ),
                )
            }
        }
        GnssPlatform.logger.d(TAG, "Decoded SSR phase bias msg $msgType: $numSats sats")
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
