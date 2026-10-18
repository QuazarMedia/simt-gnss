package app.simt.gnss.core.parser

/**
 * RTCM 3 binary frame parser.
 *
 * RTCM 3 frames have the structure:
 * - Preamble: 0xD3 (8 bits)
 * - Reserved: 6 bits (must be 0)
 * - Message length: 10 bits (0–1023 bytes)
 * - Message data: variable length
 * - CRC-24Q: 24 bits (Qualcomm CRC)
 *
 * This parser incrementally consumes bytes and emits complete, CRC-verified frames.
 */
object Rtcm3FrameParser {
    /** Maximum RTCM3 message payload per spec. */
    private const val MAX_PAYLOAD = 1023

    /** CRC-24Q polynomial: x^24 + x^23 + x^6 + x^5 + x + 1 */
    private val CRC24Q_TABLE =
        IntArray(256).also { table ->
            for (i in 0 until 256) {
                var crc = i shl 16
                for (j in 0 until 8) {
                    crc = crc shl 1
                    if (crc and 0x1000000 != 0) {
                        crc = crc xor 0x1864CFB
                    }
                }
                table[i] = crc and 0xFFFFFF
            }
        }

    /**
     * A parsed RTCM3 frame.
     * @property messageType The 12-bit message type number
     * @property payload The complete message payload (including the 12-bit type header)
     */
    data class Frame(
        val messageType: Int,
        val payload: ByteArray,
    )

    /**
     * Stateful parser that accumulates bytes and calls [onFrame] for each valid frame.
     */
    class StreamParser(private val onFrame: (Frame) -> Unit) {
        private val buffer = ByteArray(MAX_PAYLOAD + 6) // preamble + header + max payload + CRC
        private var pos = 0
        private var state = State.SYNC
        private var payloadLen = 0

        private enum class State {
            SYNC,
            HEADER,
            PAYLOAD,
            CRC,
        }

        /**
         * Feed data into the parser. Thread-safe: caller must synchronize if multi-threaded.
         */
        fun feed(
            data: ByteArray,
            offset: Int = 0,
            length: Int = data.size,
        ) {
            for (i in offset until offset + length) {
                feed(data[i])
            }
        }

        private fun feed(b: Byte) {
            when (state) {
                State.SYNC -> {
                    if (b == 0xD3.toByte()) {
                        buffer[0] = b
                        pos = 1
                        state = State.HEADER
                    }
                }
                State.HEADER -> {
                    buffer[pos++] = b
                    if (pos == 3) {
                        // Bytes 1-2: 6 reserved bits (must be 0) + 10-bit length
                        val reserved = (buffer[1].toInt() and 0xFF) shr 2
                        if (reserved != 0) {
                            rescanRetainedBytes()
                            return
                        }
                        payloadLen = ((buffer[1].toInt() and 0x03) shl 8) or (buffer[2].toInt() and 0xFF)
                        if (payloadLen > MAX_PAYLOAD) {
                            rescanRetainedBytes()
                            return
                        }
                        // Zero-length frames are legal transport keep-alives: skip straight
                        // to the CRC (nothing to emit, but the framing must stay in sync).
                        state = if (payloadLen == 0) State.CRC else State.PAYLOAD
                    }
                }
                State.PAYLOAD -> {
                    buffer[pos++] = b
                    if (pos == 3 + payloadLen) {
                        state = State.CRC
                    }
                }
                State.CRC -> {
                    buffer[pos++] = b
                    if (pos == 3 + payloadLen + 3) {
                        // Verify CRC-24Q over bytes 0..2+payloadLen-1
                        val crcComputed = crc24q(buffer, 0, 3 + payloadLen)
                        val crcReceived =
                            ((buffer[3 + payloadLen].toInt() and 0xFF) shl 16) or
                                ((buffer[3 + payloadLen + 1].toInt() and 0xFF) shl 8) or
                                (buffer[3 + payloadLen + 2].toInt() and 0xFF)

                        if (crcComputed == crcReceived) {
                            // A 12-bit message type needs ≥ 2 payload bytes; zero-length
                            // keep-alives carry nothing to emit.
                            if (payloadLen >= 2) {
                                val msgType =
                                    ((buffer[3].toInt() and 0xFF) shl 4) or
                                        ((buffer[4].toInt() and 0xFF) shr 4)
                                val payload = buffer.copyOfRange(3, 3 + payloadLen)
                                onFrame(Frame(msgType, payload))
                            }
                            resetToSync()
                        } else {
                            rescanRetainedBytes()
                        }
                    }
                }
            }
        }

        /**
         * Re-scans the bytes retained since a false 0xD3 lock instead of discarding them.
         * When joining mid-stream, ~4 false preambles appear per KB of payload, and an MSM
         * epoch often approaches the 1 KB frame maximum — dropping the swallowed bytes
         * wholesale could throw away one or two whole correction epochs per resync. The
         * recursion consumes at least the false preamble byte per level, so depth is
         * bounded by the buffer size.
         */
        private fun rescanRetainedBytes() {
            val retainedLength = pos
            if (retainedLength <= 1) {
                resetToSync()
                return
            }
            val retained = buffer.copyOfRange(1, retainedLength)
            resetToSync()
            feed(retained, 0, retained.size)
        }

        private fun resetToSync() {
            state = State.SYNC
            pos = 0
            payloadLen = 0
        }

        fun reset() = resetToSync()
    }

    /** Compute CRC-24Q over a byte range. */
    fun crc24q(
        data: ByteArray,
        offset: Int,
        length: Int,
    ): Int {
        var crc = 0
        for (i in offset until offset + length) {
            crc = ((crc shl 8) and 0xFFFFFF) xor CRC24Q_TABLE[((crc shr 16) xor (data[i].toInt() and 0xFF)) and 0xFF]
        }
        return crc
    }
}
