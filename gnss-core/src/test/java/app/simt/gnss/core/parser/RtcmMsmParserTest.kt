package app.simt.gnss.core.parser
import app.simt.gnss.core.model.Constellation
import app.simt.gnss.core.utils.GnssMathUtils.SPEED_OF_LIGHT
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class RtcmMsmParserTest {
    private val rangeMs = SPEED_OF_LIGHT * 0.001

    @Test
    fun isSupported_recognizesMsm4Through7ForKnownConstellations() {
        assertTrue(RtcmMsmParser.isSupported(1074)) // GPS MSM4
        assertTrue(RtcmMsmParser.isSupported(1077)) // GPS MSM7
        assertTrue(RtcmMsmParser.isSupported(1094)) // Galileo MSM4
        assertTrue(RtcmMsmParser.isSupported(1127)) // BeiDou MSM7
        assertFalse(RtcmMsmParser.isSupported(1073)) // MSM3 (not supported)
        assertFalse(RtcmMsmParser.isSupported(1005)) // station coordinates
        assertFalse(RtcmMsmParser.isSupported(1060)) // SSR orbit+clock
    }

    @Test
    fun parse_decodesGpsMsm7RoundTrip() {
        val writer = BitWriter()
        // --- MSM header ---
        writer.u(1077, 12) // GPS MSM7
        writer.u(1234, 12) // reference station id
        writer.u(100_000, 30) // epoch time (ms)
        writer.u(0, 1 + 3 + 7 + 2 + 2 + 1 + 3) // multi-msg, IODS, reserved, clock/ext, smoothing

        // satellite mask (64 bits): satellites 5 and 12 -> bit indices 4 and 11
        repeat(64) { i -> writer.u(if (i == 4 || i == 11) 1 else 0, 1) }
        // signal mask (32 bits): signal id 2 (GPS 1C / L1 C/A) -> bit index 1
        repeat(32) { i -> writer.u(if (i == 1) 1 else 0, 1) }
        // cell mask (nSat * nSig = 2 * 1 = 2 bits): both present
        writer.u(1, 1)
        writer.u(1, 1)

        // --- Satellite data (MSM7 = with extended info + rough rates) ---
        writer.u(70, 8) // sat0 rough range integer ms
        writer.u(71, 8) // sat1 rough range integer ms
        writer.u(0, 4) // sat0 extended info
        writer.u(0, 4) // sat1 extended info
        writer.u(512, 10) // sat0 rough range modulo (512 * 2^-10 = 0.5 ms)
        writer.u(256, 10) // sat1 rough range modulo (256 * 2^-10 = 0.25 ms)
        writer.s(0, 14) // sat0 rough phase-range-rate
        writer.s(0, 14) // sat1 rough phase-range-rate

        // --- Signal data (2 cells, MSM7 high-resolution widths) ---
        writer.s(0, 20) // cell0 fine pseudorange
        writer.s(0, 20) // cell1 fine pseudorange
        writer.s(0, 24) // cell0 fine phase
        writer.s(0, 24) // cell1 fine phase
        writer.u(100, 10) // cell0 lock
        writer.u(100, 10) // cell1 lock
        writer.u(0, 1) // cell0 half-cycle ambiguity
        writer.u(0, 1) // cell1 half-cycle ambiguity
        writer.u(720, 10) // cell0 CNR (720 * 2^-4 = 45.0 dB-Hz)
        writer.u(640, 10) // cell1 CNR (640 * 2^-4 = 40.0 dB-Hz)
        writer.s(0, 15) // cell0 fine phase-range-rate
        writer.s(0, 15) // cell1 fine phase-range-rate

        val result = RtcmMsmParser.parse(writer.toByteArray())

        assertNotNull("MSM7 should decode", result)
        requireNotNull(result)
        assertEquals(1234, result.referenceStationId)
        assertEquals(100_000L, result.epochTimeMs)
        assertEquals(2, result.observations.size)

        val sat5 = result.observations.first { it.svid == 5 }
        assertEquals(Constellation.GPS, sat5.constellationType)
        assertEquals(1_575_420_000.0, sat5.carrierFrequencyHz, 1.0)
        assertEquals(70.5 * rangeMs, sat5.pseudorangeMeters, 1e-3)
        assertNotNull(sat5.carrierPhaseMeters)
        assertEquals(70.5 * rangeMs, sat5.carrierPhaseMeters!!, 1e-3)
        assertEquals(45.0, sat5.cnrDbHz, 1e-9)

        val sat12 = result.observations.first { it.svid == 12 }
        assertEquals(71.25 * rangeMs, sat12.pseudorangeMeters, 1e-3)
        assertEquals(40.0, sat12.cnrDbHz, 1e-9)
    }

    @Test
    fun parse_dropsCarrierPhaseWhenHalfCycleAmbiguityPresent() {
        val writer = BitWriter()
        writer.u(1074, 12) // GPS MSM4
        writer.u(1, 12)
        writer.u(0, 30)
        writer.u(0, 1 + 3 + 7 + 2 + 2 + 1 + 3)
        repeat(64) { i -> writer.u(if (i == 0) 1 else 0, 1) } // satellite 1
        repeat(32) { i -> writer.u(if (i == 1) 1 else 0, 1) } // signal id 2
        writer.u(1, 1) // single cell present

        // MSM4 satellite data: rough range int + rough range modulo (no extended info / rates)
        writer.u(80, 8)
        writer.u(0, 10)
        // MSM4 signal data: 15-bit PR, 22-bit phase, 4-bit lock, 1-bit half-cycle, 6-bit CNR
        writer.s(0, 15)
        writer.s(0, 22)
        writer.u(5, 4)
        writer.u(1, 1) // half-cycle ambiguity present -> phase must be dropped
        writer.u(40, 6)

        val result = RtcmMsmParser.parse(writer.toByteArray())
        requireNotNull(result)
        assertEquals(1, result.observations.size)
        val obs = result.observations.first()
        assertEquals(80.0 * rangeMs, obs.pseudorangeMeters, 1e-3)
        assertNull("half-cycle ambiguity should drop carrier phase", obs.carrierPhaseMeters)
        assertEquals(40.0, obs.cnrDbHz, 1e-9)
    }

    @Test
    fun parse_returnsNullForNonMsmMessage() {
        val writer = BitWriter()
        writer.u(1060, 12) // SSR orbit+clock, not MSM
        assertNull(RtcmMsmParser.parse(writer.toByteArray()))
    }

    /** MSB-first bit writer mirroring GnssMathUtils.getBitsLong's read order. */
    private class BitWriter {
        private val bits = ArrayList<Int>()

        fun u(
            value: Long,
            n: Int,
        ) {
            for (i in n - 1 downTo 0) bits.add(((value shr i) and 1L).toInt())
        }

        fun u(
            value: Int,
            n: Int,
        ) = u(value.toLong(), n)

        fun s(
            value: Long,
            n: Int,
        ) = u(value and ((1L shl n) - 1), n)

        fun toByteArray(): ByteArray {
            val out = ByteArray((bits.size + 7) / 8)
            for (idx in bits.indices) {
                if (bits[idx] == 1) {
                    out[idx / 8] = (out[idx / 8].toInt() or (1 shl (7 - idx % 8))).toByte()
                }
            }
            return out
        }
    }
}
