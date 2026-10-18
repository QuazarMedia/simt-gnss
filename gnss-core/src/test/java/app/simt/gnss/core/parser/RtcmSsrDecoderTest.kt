package app.simt.gnss.core.parser
import app.simt.gnss.core.correction.SsrCorrections
import app.simt.gnss.core.model.Constellation
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Test

/**
 * Reference-vector tests for [RtcmSsrDecoder]. The encoder below writes messages field by
 * field per the RTCM 10403.3 / IGS SSR DF layout (DF385/DF386 epoch, DF375 satellite
 * reference datum on orbit-bearing messages, per-constellation satellite-id and IOD widths,
 * 9+8-bit yaw fields and 0.1 mm phase-bias resolution) — independently of the decoder's
 * internal offsets, so any layout drift fails these tests.
 */
class RtcmSsrDecoderTest {
    private class BitWriter {
        private val bits = ArrayList<Int>()

        fun write(
            value: Long,
            n: Int,
        ): BitWriter {
            for (i in n - 1 downTo 0) bits.add(((value shr i) and 1L).toInt())
            return this
        }

        fun toByteArray(): ByteArray {
            val out = ByteArray((bits.size + 7) / 8)
            bits.forEachIndexed { i, bit ->
                if (bit != 0) out[i / 8] = (out[i / 8].toInt() or (1 shl (7 - i % 8))).toByte()
            }
            return out
        }
    }

    /** Writes the common SSR header; [datumBit] only for orbit-bearing messages. */
    private fun BitWriter.ssrHeader(
        msgType: Int,
        epoch: Long,
        epochBits: Int,
        numSats: Int,
        datumBit: Boolean,
        phaseBiasFlags: Boolean = false,
    ): BitWriter {
        write(msgType.toLong(), 12)
        write(epoch, epochBits)
        write(2L, 4) // update interval
        write(0L, 1) // multiple message indicator
        if (datumBit) write(0L, 1) // DF375 satellite reference datum
        write(5L, 4) // IOD SSR
        write(258L, 16) // provider id
        write(1L, 4) // solution id
        if (phaseBiasFlags) {
            write(0L, 1) // dispersive bias consistency
            write(0L, 1) // MW consistency
        }
        write(numSats.toLong(), 6)
        return this
    }

    private fun BitWriter.orbitClockBlock(
        satId: Long,
        satIdBits: Int,
        iod: Long,
        iodBits: Int,
        iodCrcBits: Int = 0,
        dRadialRaw: Long,
        dAlongRaw: Long,
        dCrossRaw: Long,
        c0Raw: Long,
    ): BitWriter {
        write(satId, satIdBits)
        write(iod, iodBits)
        if (iodCrcBits > 0) write(0xABCDEFL, iodCrcBits)
        write(dRadialRaw and 0x3FFFFF, 22)
        write(dAlongRaw and 0xFFFFF, 20)
        write(dCrossRaw and 0xFFFFF, 20)
        write(7L, 21) // dot radial raw
        write(-3L and 0x7FFFF, 19) // dot along raw
        write(2L, 19) // dot cross raw
        write(c0Raw and 0x3FFFFF, 22)
        write(11L, 21) // c1 raw
        write(-5L and 0x7FFFFFF, 27) // c2 raw
        return this
    }

    private fun frame(
        msgType: Int,
        payload: ByteArray,
    ) = Rtcm3FrameParser.Frame(msgType, payload)

    // ------------------------------------------------------------------ orbit + clock

    @Test
    fun decode_gps1060_recoversOrbitAndClockAfterDatumBit() {
        val store = SsrCorrections.Store()
        val payload =
            BitWriter()
                .ssrHeader(1060, epoch = 345_600L, epochBits = 20, numSats = 2, datumBit = true)
                .orbitClockBlock(
                    satId = 12,
                    satIdBits = 6,
                    iod = 46,
                    iodBits = 8,
                    dRadialRaw = 1234,
                    dAlongRaw = -250,
                    dCrossRaw = 75,
                    c0Raw = -2222,
                )
                .orbitClockBlock(
                    satId = 25,
                    satIdBits = 6,
                    iod = 101,
                    iodBits = 8,
                    dRadialRaw = -987,
                    dAlongRaw = 333,
                    dCrossRaw = -44,
                    c0Raw = 555,
                )
                .toByteArray()

        RtcmSsrDecoder.decode(frame(1060, payload), store)

        val orbit12 = store.getOrbit(Constellation.GPS, 12)
        assertNotNull("PRN 12 orbit must decode (DF375 datum bit handled)", orbit12)
        assertEquals(46, orbit12!!.iod)
        assertEquals(1234 * 0.1e-3, orbit12.deltaRadial, 1e-9)
        assertEquals(-250 * 0.4e-3, orbit12.deltaAlongTrack, 1e-9)
        assertEquals(75 * 0.4e-3, orbit12.deltaCrossTrack, 1e-9)
        assertEquals(7 * 0.001e-3, orbit12.dotRadial, 1e-12)
        assertEquals(-3 * 0.004e-3, orbit12.dotAlongTrack, 1e-12)
        assertEquals(345_600.0, orbit12.epochTime, 1e-9)
        assertEquals(5.0, orbit12.updateIntervalSeconds, 1e-9)

        val clock12 = store.getClock(Constellation.GPS, 12)!!
        assertEquals(-2222 * 0.1e-3, clock12.c0, 1e-9)
        assertEquals(11 * 0.001e-3, clock12.c1, 1e-12)
        assertEquals(-5 * 0.00002e-3, clock12.c2, 1e-15)

        // Second satellite block alignment proves the per-satellite stride is right.
        val orbit25 = store.getOrbit(Constellation.GPS, 25)
        assertNotNull(orbit25)
        assertEquals(101, orbit25!!.iod)
        assertEquals(-987 * 0.1e-3, orbit25.deltaRadial, 1e-9)
        assertEquals(555 * 0.1e-3, store.getClock(Constellation.GPS, 25)!!.c0, 1e-9)
    }

    @Test
    fun decode_galileo1243_usesTenBitIod() {
        val store = SsrCorrections.Store()
        val payload =
            BitWriter()
                .ssrHeader(1243, epoch = 100_000L, epochBits = 20, numSats = 2, datumBit = true)
                .orbitClockBlock(
                    satId = 3,
                    satIdBits = 6,
                    iod = 700,
                    iodBits = 10, // > 255: needs 10 bits
                    dRadialRaw = 500,
                    dAlongRaw = 100,
                    dCrossRaw = -100,
                    c0Raw = 999,
                )
                .orbitClockBlock(
                    satId = 30,
                    satIdBits = 6,
                    iod = 81,
                    iodBits = 10,
                    dRadialRaw = -41,
                    dAlongRaw = 7,
                    dCrossRaw = 9,
                    c0Raw = -13,
                )
                .toByteArray()

        RtcmSsrDecoder.decode(frame(1243, payload), store)

        val orbit3 = store.getOrbit(Constellation.GALILEO, 3)!!
        assertEquals("Galileo IODnav is 10 bits", 700, orbit3.iod)
        assertEquals(500 * 0.1e-3, orbit3.deltaRadial, 1e-9)
        val orbit30 = store.getOrbit(Constellation.GALILEO, 30)!!
        assertEquals(81, orbit30.iod)
        assertEquals(-41 * 0.1e-3, orbit30.deltaRadial, 1e-9)
    }

    @Test
    fun decode_glonass1066_usesSeventeenBitEpochAndFiveBitSatId() {
        val store = SsrCorrections.Store()
        val payload =
            BitWriter()
                .ssrHeader(1066, epoch = 86_399L, epochBits = 17, numSats = 1, datumBit = true)
                .orbitClockBlock(
                    satId = 17,
                    satIdBits = 5,
                    iod = 33,
                    iodBits = 8,
                    dRadialRaw = 222,
                    dAlongRaw = -333,
                    dCrossRaw = 444,
                    c0Raw = -555,
                )
                .toByteArray()

        RtcmSsrDecoder.decode(frame(1066, payload), store)

        val orbit = store.getOrbit(Constellation.GLONASS, 17)
        assertNotNull("GLONASS slot 17 must decode with the 17-bit DF386 epoch", orbit)
        assertEquals(86_399.0, orbit!!.epochTime, 1e-9)
        assertEquals(33, orbit.iod)
        assertEquals(222 * 0.1e-3, orbit.deltaRadial, 1e-9)
        assertEquals(-555 * 0.1e-3, store.getClock(Constellation.GLONASS, 17)!!.c0, 1e-9)
    }

    @Test
    fun decode_beidou1261_skipsIodCrc() {
        val store = SsrCorrections.Store()
        val payload =
            BitWriter()
                .ssrHeader(1261, epoch = 200_000L, epochBits = 20, numSats = 2, datumBit = true)
                .orbitClockBlock(
                    satId = 6,
                    satIdBits = 6,
                    iod = 555,
                    iodBits = 10,
                    iodCrcBits = 24,
                    dRadialRaw = 1500,
                    dAlongRaw = 60,
                    dCrossRaw = -80,
                    c0Raw = 314,
                )
                .orbitClockBlock(
                    satId = 33,
                    satIdBits = 6,
                    iod = 9,
                    iodBits = 10,
                    iodCrcBits = 24,
                    dRadialRaw = -77,
                    dAlongRaw = 11,
                    dCrossRaw = 13,
                    c0Raw = -17,
                )
                .toByteArray()

        RtcmSsrDecoder.decode(frame(1261, payload), store)

        val orbit6 = store.getOrbit(Constellation.BEIDOU, 6)!!
        assertEquals(555, orbit6.iod)
        assertEquals(1500 * 0.1e-3, orbit6.deltaRadial, 1e-9)
        // Second block aligned only if the 24-bit IODCRC was consumed.
        val orbit33 = store.getOrbit(Constellation.BEIDOU, 33)!!
        assertEquals(9, orbit33.iod)
        assertEquals(-77 * 0.1e-3, orbit33.deltaRadial, 1e-9)
    }

    @Test
    fun decode_truncatedMessage_storesNoPartialBlock() {
        val store = SsrCorrections.Store()
        val full =
            BitWriter()
                .ssrHeader(1060, epoch = 1000L, epochBits = 20, numSats = 2, datumBit = true)
                .orbitClockBlock(
                    satId = 12,
                    satIdBits = 6,
                    iod = 46,
                    iodBits = 8,
                    dRadialRaw = 1234,
                    dAlongRaw = -250,
                    dCrossRaw = 75,
                    c0Raw = -2222,
                )
                .toByteArray() // second declared satellite never written

        RtcmSsrDecoder.decode(frame(1060, full), store)

        assertNotNull(store.getOrbit(Constellation.GPS, 12))
        // Nothing else may be stored: a zero-filled partial block would land on svid 0.
        assertNull(store.getOrbit(Constellation.GPS, 0))
    }

    // ------------------------------------------------------------------ biases

    @Test
    fun decode_codeBias1059_noDatumBitAndCentimeterScale() {
        val store = SsrCorrections.Store()
        val payload =
            BitWriter()
                .ssrHeader(1059, epoch = 5000L, epochBits = 20, numSats = 1, datumBit = false)
                .write(7L, 6) // satId
                .write(2L, 5) // numBiases
                .write(0L, 5).write(123L, 14) // signal 0 (C1C), +1.23 m
                .write(11L, 5).write(-321L and 0x3FFF, 14) // signal 11, −3.21 m
                .toByteArray()

        RtcmSsrDecoder.decode(frame(1059, payload), store)

        assertEquals(1.23, store.getCodeBias(Constellation.GPS, 7, 0)!!.bias, 1e-9)
        assertEquals(-3.21, store.getCodeBias(Constellation.GPS, 7, 11)!!.bias, 1e-9)
    }

    @Test
    fun decode_phaseBias1265_nineEightYawAndTenthMillimeterScale() {
        val store = SsrCorrections.Store()
        val payload =
            BitWriter()
                .ssrHeader(1265, epoch = 5000L, epochBits = 20, numSats = 1, datumBit = false, phaseBiasFlags = true)
                .write(9L, 6) // satId
                .write(2L, 5) // numBiases
                .write(311L, 9) // yaw angle (9 bits)
                .write(-100L and 0xFF, 8) // yaw rate (8 bits)
                .write(1L, 5).write(1L, 1).write(0L, 2).write(3L, 4).write(12345L, 20) // bias 1
                .write(24L, 5).write(0L, 1).write(0L, 2).write(0L, 4).write(-54321L and 0xFFFFF, 20) // bias 2
                .toByteArray()

        RtcmSsrDecoder.decode(frame(1265, payload), store)

        val bias1 = store.getPhaseBias(Constellation.GPS, 9, 1)
        assertNotNull("First phase bias must decode (9+8-bit yaw fields consumed)", bias1)
        assertEquals(12345 * 0.0001, bias1!!.bias, 1e-9)
        assertEquals(1, bias1.integerIndicator)
        // Second bias block alignment proves the per-bias stride.
        val bias2 = store.getPhaseBias(Constellation.GPS, 9, 24)!!
        assertEquals(-54321 * 0.0001, bias2.bias, 1e-9)
        assertEquals(0, bias2.integerIndicator)
    }
}
