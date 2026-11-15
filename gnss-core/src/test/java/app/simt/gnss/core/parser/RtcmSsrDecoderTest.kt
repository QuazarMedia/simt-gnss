package app.simt.gnss.core.parser
import app.simt.gnss.core.correction.SsrCorrections
import app.simt.gnss.core.model.Constellation
import app.simt.gnss.core.model.GnssSignalBand
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
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

    @Test
    fun decode_separateOrbit1057AndClock1058_pairsViaIodSsr() {
        val store = SsrCorrections.Store()
        val orbitPayload =
            BitWriter()
                .ssrHeader(1057, epoch = 345_600L, epochBits = 20, numSats = 1, datumBit = true)
                .write(12L, 6) // satId
                .write(46L, 8) // IOD
                .write(1234L and 0x3FFFFF, 22) // deltaRadial
                .write(-250L and 0xFFFFF, 20) // deltaAlong
                .write(75L and 0xFFFFF, 20) // deltaCross
                .write(7L, 21) // dotRadial
                .write(-3L and 0x7FFFF, 19) // dotAlong
                .write(2L, 19) // dotCross
                .toByteArray()

        val clockPayload =
            BitWriter()
                .ssrHeader(1058, epoch = 345_605L, epochBits = 20, numSats = 1, datumBit = false)
                .write(12L, 6) // satId
                .write(-2222L and 0x3FFFFF, 22) // c0
                .write(11L, 21) // c1
                .write(-5L and 0x7FFFFFF, 27) // c2
                .toByteArray()

        RtcmSsrDecoder.decode(frame(1057, orbitPayload), store)
        RtcmSsrDecoder.decode(frame(1058, clockPayload), store)

        val orbit = store.getOrbit(Constellation.GPS, 12)
        val clock = store.getClock(Constellation.GPS, 12)
        assertNotNull(orbit)
        assertNotNull(clock)
        assertEquals(46, orbit!!.iod)
        assertEquals(5, orbit.iodSsr)
        assertEquals(46, clock!!.iod)
        assertEquals(5, clock.iodSsr)
        assertEquals(1234 * 0.1e-3, orbit.deltaRadial, 1e-9)
        assertEquals(-250 * 0.4e-3, orbit.deltaAlongTrack, 1e-9)
        assertEquals(-2222 * 0.1e-3, clock.c0, 1e-9)
        assertEquals(11 * 0.001e-3, clock.c1, 1e-12)
        assertEquals(-5 * 0.00002e-3, clock.c2, 1e-15)
    }

    @Test
    fun decode_qzss1249And1248_mapsSvidTo192PlusSatId() {
        val store = SsrCorrections.Store()
        val combinedPayload =
            BitWriter()
                .ssrHeader(1249, epoch = 345_600L, epochBits = 20, numSats = 1, datumBit = true)
                .orbitClockBlock(
                    satId = 3,
                    satIdBits = 4,
                    iod = 77,
                    iodBits = 8,
                    dRadialRaw = 500,
                    dAlongRaw = -100,
                    dCrossRaw = 50,
                    c0Raw = 800,
                )
                .toByteArray()
        val codeBiasPayload =
            BitWriter()
                .ssrHeader(1248, epoch = 345_600L, epochBits = 20, numSats = 1, datumBit = false)
                .write(3L, 4) // QZSS satId 3 -> SVID 195
                .write(1L, 5) // numBiases
                .write(0L, 5).write(215L, 14) // signal 0 (1C), +2.15 m
                .toByteArray()

        RtcmSsrDecoder.decode(frame(1249, combinedPayload), store)
        RtcmSsrDecoder.decode(frame(1248, codeBiasPayload), store)

        val orbit = store.getOrbit(Constellation.QZSS, 195)
        val clock = store.getClock(Constellation.QZSS, 195)
        val codeBias = store.getCodeBias(Constellation.QZSS, 195, 0)
        assertNotNull(orbit)
        assertNotNull(clock)
        assertNotNull(codeBias)
        assertEquals(77, orbit!!.iod)
        assertEquals(500 * 0.1e-3, orbit.deltaRadial, 1e-9)
        assertEquals(800 * 0.1e-3, clock!!.c0, 1e-9)
        assertEquals(2.15, codeBias!!.bias, 1e-9)
    }

    private fun BitWriter.igs4076Header(
        subtype: Int,
        epoch: Long,
        numSats: Int,
        datumBit: Boolean,
    ): BitWriter {
        write(4076L, 12)
        write(1L, 3) // version 1
        write(subtype.toLong(), 8)
        write(epoch, 20)
        write(2L, 4) // UDI = 5 s
        write(0L, 1) // multi-message
        write(6L, 4) // IOD SSR
        write(100L, 16) // provider
        write(1L, 4) // solution
        if (datumBit) write(0L, 1) // DF375 datum after solutionId in IGS 4076
        write(numSats.toLong(), 6)
        return this
    }

    @Test
    fun decode_igs4076_gps23Galileo65Beidou103_roundTrips() {
        val store = SsrCorrections.Store()
        val gps23 =
            BitWriter()
                .igs4076Header(subtype = 23, epoch = 345_600L, numSats = 1, datumBit = true)
                .orbitClockBlock(
                    satId = 8,
                    satIdBits = 6,
                    iod = 91,
                    iodBits = 8,
                    dRadialRaw = -400,
                    dAlongRaw = 120,
                    dCrossRaw = -60,
                    c0Raw = 1111,
                )
                .toByteArray()
        val gal65 =
            BitWriter()
                .igs4076Header(subtype = 65, epoch = 345_600L, numSats = 1, datumBit = false)
                .write(14L, 6) // E14
                .write(2L, 5) // numBiases
                .write(1L, 5).write(-185L and 0x3FFF, 14) // E1B (-1.85 m)
                .write(6L, 5).write(240L, 14) // E5a Q (+2.40 m)
                .toByteArray()
        val bds103 =
            BitWriter()
                .igs4076Header(subtype = 103, epoch = 345_614L, numSats = 1, datumBit = true)
                .orbitClockBlock(
                    satId = 19,
                    satIdBits = 6,
                    iod = 12,
                    iodBits = 8, // IGS 4076 uses 8-bit IOD and 0-bit IODCRC
                    dRadialRaw = 900,
                    dAlongRaw = -30,
                    dCrossRaw = 40,
                    c0Raw = -750,
                )
                .toByteArray()

        RtcmSsrDecoder.decode(frame(4076, gps23), store)
        RtcmSsrDecoder.decode(frame(4076, gal65), store)
        RtcmSsrDecoder.decode(frame(4076, bds103), store)

        val gpsOrbit = store.getOrbit(Constellation.GPS, 8)!!
        assertEquals(91, gpsOrbit.iod)
        assertEquals(-400 * 0.1e-3, gpsOrbit.deltaRadial, 1e-9)
        assertEquals(1111 * 0.1e-3, store.getClock(Constellation.GPS, 8)!!.c0, 1e-9)

        assertEquals(-1.85, store.getCodeBias(Constellation.GALILEO, 14, 1)!!.bias, 1e-9)
        assertEquals(2.40, store.getCodeBias(Constellation.GALILEO, 14, 6)!!.bias, 1e-9)

        val bdsOrbit = store.getOrbit(Constellation.BEIDOU, 19)!!
        assertEquals(12, bdsOrbit.iod)
        assertEquals(SsrCorrections.EpochTimescale.GPST, bdsOrbit.epochTimescale)
        assertEquals(900 * 0.1e-3, bdsOrbit.deltaRadial, 1e-9)
        assertEquals(-750 * 0.1e-3, store.getClock(Constellation.BEIDOU, 19)!!.c0, 1e-9)
    }

    @Test
    fun decode_igs4076_galileo63_usesTenBitIodAcrossTwoSatellites() {
        val store = SsrCorrections.Store()
        val gal63 =
            BitWriter()
                .igs4076Header(subtype = 63, epoch = 345_600L, numSats = 2, datumBit = true)
                .orbitClockBlock(
                    satId = 3,
                    satIdBits = 6,
                    iod = 700,
                    iodBits = 10, // > 255: Galileo IODnav is 10 bits in IGS 4076
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

        RtcmSsrDecoder.decode(frame(4076, gal63), store)

        val orbit3 = store.getOrbit(Constellation.GALILEO, 3)!!
        assertEquals(700, orbit3.iod)
        assertEquals(500 * 0.1e-3, orbit3.deltaRadial, 1e-9)
        assertEquals(999 * 0.1e-3, store.getClock(Constellation.GALILEO, 3)!!.c0, 1e-9)

        val orbit30 = store.getOrbit(Constellation.GALILEO, 30)!!
        assertEquals(81, orbit30.iod)
        assertEquals(-41 * 0.1e-3, orbit30.deltaRadial, 1e-9)
        assertEquals(-13 * 0.1e-3, store.getClock(Constellation.GALILEO, 30)!!.c0, 1e-9)
    }

    @Test
    fun decode_highRateClock1062_doesNotOverwriteBaseClockAndIsRejectedAlone() {
        val store = SsrCorrections.Store()
        val hrPayload =
            BitWriter()
                .ssrHeader(1062, epoch = 345_605L, epochBits = 20, numSats = 1, datumBit = false)
                .write(12L, 6) // satId
                .write(-400L and 0x3FFFFF, 22) // -0.04 m
                .toByteArray()

        // 1062 arriving alone must NOT populate base ClockCorrection
        RtcmSsrDecoder.decode(frame(1062, hrPayload), store)
        assertNull(store.getClock(Constellation.GPS, 12))
        assertEquals(-0.04, store.getHighRateClock(Constellation.GPS, 12, iodSsr = 5)!!.deltaClockMeters, 1e-9)

        // Now decode base 1060 with c0 = 2.5 m, c1 = 0.01 m/s, same iodSsr = 5
        val combined1060 =
            BitWriter()
                .ssrHeader(1060, epoch = 345_600L, epochBits = 20, numSats = 1, datumBit = true)
                .write(12L, 6)
                .write(46L, 8)
                .write(0L, 22).write(0L, 20).write(0L, 20)
                .write(0L, 21).write(0L, 19).write(0L, 19)
                .write(25_000L, 22) // c0 = 2.5 m
                .write(10_000L, 21) // c1 = 0.01 m/s
                .write(0L, 27)
                .toByteArray()
        RtcmSsrDecoder.decode(frame(1060, combined1060), store)

        // Re-decode 1062 after 1060: base clock must remain untouched
        RtcmSsrDecoder.decode(frame(1062, hrPayload), store)

        val baseClock = store.getClock(Constellation.GPS, 12)!!
        assertEquals(2.5, baseClock.c0, 1e-9)
        assertEquals(0.01, baseClock.c1, 1e-12)
        assertEquals(-0.04, store.getHighRateClock(Constellation.GPS, 12, iodSsr = 5)!!.deltaClockMeters, 1e-9)

        // Unspecified iodSsr (-1) must never be served by getHighRateClock
        store.putHighRateClock(
            SsrCorrections.HighRateClockCorrection(
                constellationType = Constellation.GPS,
                svid = 19,
                deltaClockMeters = -0.09,
                iodSsr = -1,
                epochTime = 345_600.0,
                receivedAtMs = 0L,
            ),
        )
        assertNull(store.getHighRateClock(Constellation.GPS, 19, iodSsr = -1))
    }

    @Test
    fun gpsL1SignalIds_excludes2cSignalId5_whileGpsL2Includes5() {
        val l1Ids = SsrCorrections.ssrCodeBiasSignalIds(Constellation.GPS, GnssSignalBand.L1_E1_B1C)
        val l2Ids = SsrCorrections.ssrCodeBiasSignalIds(Constellation.GPS, GnssSignalBand.L2)
        assertFalse("GPS L1 must NOT match RTCM SSR signal ID 5 (2C)", 5 in l1Ids)
        assertTrue("GPS L2 must include RTCM SSR signal ID 5 (2C)", 5 in l2Ids)
    }
}
