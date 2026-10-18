package app.simt.gnss.core.validator

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.random.Random

class GpsNavMessageValidatorTest {
    @Test
    fun encodeThenVerify_roundTripsForAllParityStates() {
        val random = Random(7)
        for (case in 0 until 200) {
            val source = random.nextInt(1 shl 24)
            val d29Star = random.nextInt(2)
            val d30Star = random.nextInt(2)
            val word = GpsNavMessageValidator.encodeWord(source, d29Star, d30Star)
            assertTrue(
                "case $case: encoded word must verify",
                GpsNavMessageValidator.hasValidParity(word, d29Star, d30Star),
            )
            // Decoding restores the source data bits.
            val decoded = GpsNavMessageValidator.getDecodedWord(word, (d29Star shl 1) or d30Star)
            assertEquals("case $case: decode must restore source", source, (decoded ushr 6) and 0xFFFFFF)
        }
    }

    @Test
    fun anySingleBitFlip_isDetected() {
        // The (32,26) Hamming structure detects every single-bit error: flipping any of the
        // 30 word bits (data or parity) must invalidate the parity.
        val random = Random(11)
        for (case in 0 until 20) {
            val source = random.nextInt(1 shl 24)
            val d29Star = random.nextInt(2)
            val d30Star = random.nextInt(2)
            val word = GpsNavMessageValidator.encodeWord(source, d29Star, d30Star)
            for (bit in 0 until 30) {
                val corrupted = word xor (1 shl bit)
                assertFalse(
                    "case $case: flip of bit $bit must be detected",
                    GpsNavMessageValidator.hasValidParity(corrupted, d29Star, d30Star),
                )
            }
        }
    }

    @Test
    fun wrongParityState_isDetected() {
        // A word verified against the wrong D29*/D30* must fail (the previous word's parity
        // bits participate in the equations).
        val word = GpsNavMessageValidator.encodeWord(0x8B0000 or 0x1234, d29Star = 0, d30Star = 0)
        assertTrue(GpsNavMessageValidator.hasValidParity(word, 0, 0))
        assertFalse(GpsNavMessageValidator.hasValidParity(word, 1, 0))
        assertFalse(GpsNavMessageValidator.hasValidParity(word, 0, 1))
    }

    @Test
    fun zeroWord_withZeroState_isAValidCodeword() {
        assertTrue(GpsNavMessageValidator.hasValidParity(0, 0, 0))
    }
}
