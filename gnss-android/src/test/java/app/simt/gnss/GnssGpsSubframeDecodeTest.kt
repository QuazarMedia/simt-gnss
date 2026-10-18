package app.simt.gnss

import android.location.GnssStatus
import app.simt.gnss.core.utils.GnssMathUtils
import app.simt.gnss.core.validator.GpsNavMessageValidator
import org.junit.Assert.assertArrayEquals
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner

/**
 * Tests of [GnssMeasurementProcessor.decodeGpsL1CaSubframe]: TLM-preamble-driven D30*
 * decoding and IS-GPS-200 parity verification of as-broadcast GPS subframes.
 */
@RunWith(RobolectricTestRunner::class)
class GnssGpsSubframeDecodeTest {
    private val satKey = GnssStatus.CONSTELLATION_GPS to 7

    /** Source (decoded) 24-bit data of 10 words: TLM with preamble 0x8B, HOW with subframe ID. */
    private fun sourceWords(subframeId: Int): IntArray {
        val words = IntArray(10) { 0x000A00 + it } // arbitrary distinct data
        words[0] = 0x8B0000 or 0x1234 // TLM: preamble + message
        // HOW: TOW in bits 1-17, subframe ID in bits 20-22 (1-indexed) → shift 24-22 = 2.
        words[1] = (0x123 shl 7) or (subframeId shl 2)
        return words
    }

    /** Encodes source words into the as-broadcast 40-byte HAL payload with valid parity chaining. */
    private fun broadcastPayload(
        source: IntArray,
        initialD29Star: Int = 0,
        initialD30Star: Int = 0,
        corruptWordIndex: Int = -1,
    ): ByteArray {
        val payload = ByteArray(40)
        var d29Star = initialD29Star
        var d30Star = initialD30Star
        for (k in 0 until 10) {
            var word = GpsNavMessageValidator.encodeWord(source[k], d29Star, d30Star)
            if (k == corruptWordIndex) word = word xor (1 shl 13) // flip one data bit
            // 30-bit word right-justified in a 4-byte slot (skip the top 2 bits).
            GnssMathUtils.packBits(payload, k * 32 + 2, 30, word)
            d29Star = (word shr 1) and 1
            d30Star = word and 1
        }
        return payload
    }

    private fun decodedDataBits(subframe: ByteArray): IntArray = IntArray(10) { GnssMathUtils.getBits(subframe, it * 30, 24) }

    @Test
    fun uprightStream_decodesSourceData() {
        val source = sourceWords(subframeId = 2)
        val processor = GnssMeasurementProcessor()
        val subframe = processor.decodeGpsL1CaSubframe(satKey, broadcastPayload(source), parityAttested = false)
        assertNotNull("Upright broadcast stream must decode", subframe)
        assertArrayEquals(source, decodedDataBits(subframe!!))
        assertEquals(2, GnssMathUtils.getBits(subframe, 49, 3))
    }

    @Test
    fun complementedStream_decodesToSameSourceData() {
        // D30* = 1 entering the subframe complements the TLM data bits (preamble reads 0x74);
        // the decoder must recognise it and recover identical source data.
        val source = sourceWords(subframeId = 3)
        val processor = GnssMeasurementProcessor()
        val payload = broadcastPayload(source, initialD29Star = 1, initialD30Star = 1)
        val subframe = processor.decodeGpsL1CaSubframe(satKey, payload, parityAttested = false)
        assertNotNull("Complemented broadcast stream must decode", subframe)
        assertArrayEquals(source, decodedDataBits(subframe!!))
        assertEquals(3, GnssMathUtils.getBits(subframe, 49, 3))
    }

    @Test
    fun corruptedWord_withoutHalAttestation_isRejected() {
        val source = sourceWords(subframeId = 1)
        val processor = GnssMeasurementProcessor()
        val payload = broadcastPayload(source, corruptWordIndex = 4)
        assertNull(
            "Unattested subframe with a parity failure must be rejected",
            processor.decodeGpsL1CaSubframe(satKey, payload, parityAttested = false),
        )
    }

    @Test
    fun corruptedParityBits_withHalAttestation_stillDecodes() {
        // STATUS_PARITY_PASSED: the HAL has verified (and possibly rewritten) the stream, so
        // the local parity check must not reject it.
        val source = sourceWords(subframeId = 1)
        val processor = GnssMeasurementProcessor()
        val payload = broadcastPayload(source, corruptWordIndex = 4)
        val subframe = processor.decodeGpsL1CaSubframe(satKey, payload, parityAttested = true)
        assertNotNull(subframe)
        assertEquals(1, GnssMathUtils.getBits(subframe!!, 49, 3))
    }

    @Test
    fun garbageWithoutTlmPreamble_isRejectedEvenIfSubframeIdLooksValid() {
        // The old heuristic accepted any buffer whose bits 50-52 read 1..5 — including
        // complemented subframes (IDs 2..5 invert to other valid IDs) and noise.
        val payload = ByteArray(40)
        GnssMathUtils.packBits(payload, 32 + 2 + 19, 3, 2) // HOW subframe ID = 2, no TLM preamble
        val processor = GnssMeasurementProcessor()
        assertNull(processor.decodeGpsL1CaSubframe(satKey, payload, parityAttested = false))
    }
}
