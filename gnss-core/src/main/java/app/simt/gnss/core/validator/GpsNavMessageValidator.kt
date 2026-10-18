package app.simt.gnss.core.validator

/**
 * Handles GPS L1 C/A navigation message decoding according to IS-GPS-200.
 */
object GpsNavMessageValidator {
    /** XOR mask covering data bits d1–d24 of a right-justified 30-bit word. */
    private const val DATA_BITS_MASK = 0x3FFFFFC0

    /**
     * Decodes the first 24 bits of a GPS word based on the last bit of the previous word ($D_{30}^*$).
     *
     * In GPS, if the last bit of the previous word is 1, the data bits of the current word
     * are inverted (XORed) before transmission to ensure DC balance.
     *
     * @param word The 30-bit GPS word.
     * @param lastBitsOfPrevWord Bits 29 and 30 of the previous word (stored in the lower 2 bits).
     * @return The 30-bit word with data bits (1-24) restored to their original state.
     */
    fun getDecodedWord(
        word: Int,
        lastBitsOfPrevWord: Int,
    ): Int {
        val d30Star = lastBitsOfPrevWord and 0x01
        return if (d30Star != 0) {
            word xor DATA_BITS_MASK
        } else {
            word
        }
    }

    /**
     * Verifies the six Hamming parity bits of a raw (as-broadcast) 30-bit word per
     * IS-GPS-200 §20.3.5.2 Table 20-XIV.
     *
     * The parity equations operate on the *source* data bits (i.e. the received d1–d24 with
     * the D30* complement removed) XORed with the previous word's transmitted parity bits
     * D29* and D30*. The received parity bits d25–d30 are compared as transmitted.
     *
     * @param rawWord the 30-bit word exactly as received (data bits possibly complemented)
     * @param d29Star bit 29 of the previous transmitted word
     * @param d30Star bit 30 of the previous transmitted word
     */
    fun hasValidParity(
        rawWord: Int,
        d29Star: Int,
        d30Star: Int,
    ): Boolean {
        val source = if (d30Star != 0) rawWord xor DATA_BITS_MASK else rawWord

        // Source data bit i (1-indexed, 1..24) and received parity bit i (25..30).
        fun d(i: Int): Int = (source shr (30 - i)) and 1

        fun received(i: Int): Int = (rawWord shr (30 - i)) and 1

        fun xorOf(vararg bits: Int): Int {
            var acc = 0
            for (b in bits) acc = acc xor d(b)
            return acc
        }

        val d25 = d29Star xor xorOf(1, 2, 3, 5, 6, 10, 11, 12, 13, 14, 17, 18, 20, 23)
        val d26 = d30Star xor xorOf(2, 3, 4, 6, 7, 11, 12, 13, 14, 15, 18, 19, 21, 24)
        val d27 = d29Star xor xorOf(1, 3, 4, 5, 7, 8, 12, 13, 14, 15, 16, 19, 20, 22)
        val d28 = d30Star xor xorOf(2, 4, 5, 6, 8, 9, 13, 14, 15, 16, 17, 20, 21, 23)
        val d29 = d30Star xor xorOf(1, 3, 5, 6, 7, 9, 10, 14, 15, 16, 17, 18, 21, 22, 24)
        val d30 = d29Star xor xorOf(3, 5, 6, 8, 9, 10, 11, 13, 15, 19, 22, 23, 24)

        return d25 == received(25) && d26 == received(26) && d27 == received(27) &&
            d28 == received(28) && d29 == received(29) && d30 == received(30)
    }

    /**
     * Computes the six parity bits and returns the complete 30-bit transmitted word for the
     * given source data bits and previous-word parity state. Used to build the as-broadcast
     * stream (the data bits are complemented when D30* = 1, per IS-GPS-200 §20.3.5.2).
     *
     * @param sourceDataBits24 the 24 source data bits, right-justified
     */
    fun encodeWord(
        sourceDataBits24: Int,
        d29Star: Int,
        d30Star: Int,
    ): Int {
        fun d(i: Int): Int = (sourceDataBits24 shr (24 - i)) and 1

        fun xorOf(vararg bits: Int): Int {
            var acc = 0
            for (b in bits) acc = acc xor d(b)
            return acc
        }

        val d25 = d29Star xor xorOf(1, 2, 3, 5, 6, 10, 11, 12, 13, 14, 17, 18, 20, 23)
        val d26 = d30Star xor xorOf(2, 3, 4, 6, 7, 11, 12, 13, 14, 15, 18, 19, 21, 24)
        val d27 = d29Star xor xorOf(1, 3, 4, 5, 7, 8, 12, 13, 14, 15, 16, 19, 20, 22)
        val d28 = d30Star xor xorOf(2, 4, 5, 6, 8, 9, 13, 14, 15, 16, 17, 20, 21, 23)
        val d29 = d30Star xor xorOf(1, 3, 5, 6, 7, 9, 10, 14, 15, 16, 17, 18, 21, 22, 24)
        val d30 = d29Star xor xorOf(3, 5, 6, 8, 9, 10, 11, 13, 15, 19, 22, 23, 24)

        val transmittedData =
            if (d30Star != 0) (sourceDataBits24.inv() and 0xFFFFFF) else (sourceDataBits24 and 0xFFFFFF)
        return (transmittedData shl 6) or (d25 shl 5) or (d26 shl 4) or (d27 shl 3) or
            (d28 shl 2) or (d29 shl 1) or d30
    }
}
