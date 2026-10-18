package app.simt.gnss.core.parser

import org.junit.Assert.assertArrayEquals
import org.junit.Assert.assertEquals
import org.junit.Test

class Rtcm3FrameParserTest {
    private fun buildFrame(payload: ByteArray): ByteArray {
        val frame = ByteArray(3 + payload.size + 3)
        frame[0] = 0xD3.toByte()
        frame[1] = ((payload.size shr 8) and 0x03).toByte()
        frame[2] = (payload.size and 0xFF).toByte()
        payload.copyInto(frame, 3)
        val crc = Rtcm3FrameParser.crc24q(frame, 0, 3 + payload.size)
        frame[3 + payload.size] = ((crc shr 16) and 0xFF).toByte()
        frame[3 + payload.size + 1] = ((crc shr 8) and 0xFF).toByte()
        frame[3 + payload.size + 2] = (crc and 0xFF).toByte()
        return frame
    }

    private fun msmLikePayload(
        msgType: Int,
        size: Int,
    ): ByteArray =
        ByteArray(size).also {
            it[0] = ((msgType shr 4) and 0xFF).toByte()
            it[1] = (((msgType and 0x0F) shl 4) or 0x05).toByte()
            for (i in 2 until size) it[i] = (i * 31).toByte()
        }

    @Test
    fun crc24q_matchesPublishedCheckValue() {
        // CRC-24Q (poly 0x864CFB, init 0, no reflection): check("123456789") = 0xCDE703.
        val data = "123456789".toByteArray(Charsets.US_ASCII)
        assertEquals(0xCDE703, Rtcm3FrameParser.crc24q(data, 0, data.size))
    }

    @Test
    fun feed_singleFrame_emitsOnce() {
        val frames = ArrayList<Rtcm3FrameParser.Frame>()
        val parser = Rtcm3FrameParser.StreamParser { frames.add(it) }
        val payload = msmLikePayload(1074, 60)
        parser.feed(buildFrame(payload))
        assertEquals(1, frames.size)
        assertEquals(1074, frames[0].messageType)
        assertArrayEquals(payload, frames[0].payload)
    }

    @Test
    fun feed_framesSplitAcrossArbitraryChunks_allEmitted() {
        val frames = ArrayList<Rtcm3FrameParser.Frame>()
        val parser = Rtcm3FrameParser.StreamParser { frames.add(it) }
        val stream = buildFrame(msmLikePayload(1074, 120)) + buildFrame(msmLikePayload(1005, 19)) + buildFrame(msmLikePayload(1084, 200))
        var pos = 0
        val chunkSizes = intArrayOf(1, 2, 3, 7, 11, 19)
        var c = 0
        while (pos < stream.size) {
            val take = minOf(chunkSizes[c % chunkSizes.size], stream.size - pos)
            parser.feed(stream, pos, take)
            pos += take
            c++
        }
        assertEquals(3, frames.size)
        assertEquals(listOf(1074, 1005, 1084), frames.map { it.messageType })
    }

    @Test
    fun falsePreamble_doesNotSwallowTheEmbeddedRealFrame() {
        // A stray 0xD3 with a plausible header swallows the following bytes as its
        // "payload"; when its CRC fails, the retained bytes must be rescanned so the real
        // frame inside them is still recovered (joining mid-stream hits this constantly).
        val frames = ArrayList<Rtcm3FrameParser.Frame>()
        val parser = Rtcm3FrameParser.StreamParser { frames.add(it) }
        val real = buildFrame(msmLikePayload(1077, 40))
        // False lock: preamble + reserved-ok header declaring a 60-byte payload.
        val falseStart = byteArrayOf(0xD3.toByte(), 0x00, 60)
        parser.feed(falseStart + real + ByteArray(30) { 0x55 })
        assertEquals("the embedded real frame must be recovered", 1, frames.size)
        assertEquals(1077, frames[0].messageType)
    }

    @Test
    fun corruptedFrame_doesNotAffectTheNextFrame() {
        val frames = ArrayList<Rtcm3FrameParser.Frame>()
        val parser = Rtcm3FrameParser.StreamParser { frames.add(it) }
        val corrupted = buildFrame(msmLikePayload(1074, 50)).also { it[10] = (it[10] + 1).toByte() }
        parser.feed(corrupted + buildFrame(msmLikePayload(1005, 19)))
        assertEquals(1, frames.size)
        assertEquals(1005, frames[0].messageType)
    }

    @Test
    fun zeroLengthKeepAlive_isConsumedSilently() {
        val frames = ArrayList<Rtcm3FrameParser.Frame>()
        val parser = Rtcm3FrameParser.StreamParser { frames.add(it) }
        val keepAlive = buildFrame(ByteArray(0))
        parser.feed(keepAlive + buildFrame(msmLikePayload(1094, 25)))
        assertEquals("keep-alive emits nothing; following frame must parse", 1, frames.size)
        assertEquals(1094, frames[0].messageType)
    }
}
