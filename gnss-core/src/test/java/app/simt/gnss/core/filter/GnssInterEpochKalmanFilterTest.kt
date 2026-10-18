package app.simt.gnss.core.filter

import app.simt.gnss.core.utils.GnssMathUtils
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test
import kotlin.math.abs
import kotlin.math.sqrt

class GnssInterEpochKalmanFilterTest {
    private lateinit var filter: GnssInterEpochKalmanFilter

    @Before
    fun setUp() {
        filter = GnssInterEpochKalmanFilter()
    }

    private fun diagR(varianceM2: Double): Array<DoubleArray> =
        Array(3) { i ->
            DoubleArray(3) { j ->
                if (i == j) varianceM2 else 0.0
            }
        }

    @Test
    fun firstEpoch_returnsWlsPosition() {
        val wls = Triple(1_000_000.0, 2_000_000.0, 3_000_000.0)
        val r = diagR(25.0)
        val out = filter.fuse(wls, r, 1_000_000_000L)
        assertEquals(wls.first, out.first, 1e-6)
        assertEquals(wls.second, out.second, 1e-6)
        assertEquals(wls.third, out.third, 1e-6)
    }

    @Test
    fun stationaryMeasurements_pullEstimateTowardMean() {
        val r = diagR(100.0)
        val t0 = 2_000_000_000_000L
        val dt = GnssMathUtils.NANOS_PER_SECOND
        val p0 = Triple(100.0, 200.0, 300.0)
        filter.fuse(p0, r, t0)
        val pNoisy = Triple(110.0, 190.0, 305.0)
        val out = filter.fuse(pNoisy, r, t0 + dt)
        // Filtered position should lie between prior prediction and measurement (looser than raw noisy)
        val distToNoisy =
            sqrt(
                (out.first - pNoisy.first).let { it * it } +
                    (out.second - pNoisy.second).let { it * it } +
                    (out.third - pNoisy.third).let { it * it },
            )
        val distToPrev =
            sqrt(
                (out.first - p0.first).let { it * it } +
                    (out.second - p0.second).let { it * it } +
                    (out.third - p0.third).let { it * it },
            )
        assertTrue(distToNoisy < 12.0)
        assertTrue(distToPrev < 12.0)
    }

    @Test
    fun largeInnovation_isRejectedNotAdopted() {
        // A single gated epoch is an outlier: the filter must coast on its prediction,
        // NOT re-initialize onto the spike (which would pass it through at full gain).
        val r = diagR(4.0)
        val t0 = 0L
        filter.fuse(Triple(0.0, 0.0, 0.0), r, t0)
        val jump = Triple(10_000.0, 0.0, 0.0)
        val out = filter.fuse(jump, r, t0 + GnssMathUtils.NANOS_PER_SECOND)
        assertEquals("Outlier epoch must be rejected (coast near prior)", 0.0, out.first, 1.0)
    }

    @Test
    fun persistentLargeInnovation_reinitializesAfterConsecutiveRejections() {
        // If the measurements disagree for KALMAN_MAX_CONSECUTIVE_REJECTIONS epochs in a row
        // the *filter state* is wrong (e.g. undetected displacement) — follow the measurements.
        val r = diagR(4.0)
        var t = 0L
        val dt = GnssMathUtils.NANOS_PER_SECOND
        filter.fuse(Triple(0.0, 0.0, 0.0), r, t)
        val jump = Triple(10_000.0, 0.0, 0.0)
        var out = Triple(0.0, 0.0, 0.0)
        repeat(GnssMathUtils.KALMAN_MAX_CONSECUTIVE_REJECTIONS) {
            t += dt
            out = filter.fuse(jump, r, t)
        }
        assertEquals("Persistent disagreement must re-initialize to the measurement", jump.first, out.first, 1.0)
        // And the filter must operate normally afterwards.
        t += dt
        out = filter.fuse(Triple(10_001.0, 0.0, 0.0), r, t)
        assertTrue(out.first in 10_000.0..10_001.0)
    }

    @Test
    fun reset_clearsState() {
        filter.fuse(Triple(1.0, 2.0, 3.0), diagR(9.0), 100L)
        filter.reset()
        val wls = Triple(10.0, 20.0, 30.0)
        val out = filter.fuse(wls, diagR(9.0), 200L)
        assertEquals(wls.first, out.first, 1e-9)
        assertEquals(wls.second, out.second, 1e-9)
        assertEquals(wls.third, out.third, 1e-9)
    }

    @Test
    fun nonIncreasingTime_reinitializes() {
        val r = diagR(16.0)
        val t = 500L
        filter.fuse(Triple(1.0, 0.0, 0.0), r, t)
        val out = filter.fuse(Triple(5.0, 0.0, 0.0), r, t)
        assertTrue(abs(out.first - 5.0) < 1e-9)
    }

    // -----------------------------------------------------------------------------------------
    //  Innovation gate: the fix in §40/§41 + this batch now compares the *Mahalanobis* distance
    //  (unitless) against [KALMAN_MAX_INNOVATION_SIGMA], with a secondary Euclidean safety net
    //  at [KALMAN_MAX_INNOVATION_METERS]. Both gates must work independently.
    // -----------------------------------------------------------------------------------------

    @Test
    fun mahalanobisGate_tinyInnovationWithSmallR_stillAccepted() {
        // With R very small (1 m²), σ per axis ≈ 1 m. An innovation of 3 m has Mahalanobis ≈
        // 3 / sqrt(1 + Q_pos) ≈ 3 / sqrt(~1.1) ≈ 2.86 → under the 4σ gate, so the filter must
        // fuse rather than reinit.
        val r = diagR(1.0)
        val t0 = 0L
        filter.fuse(Triple(0.0, 0.0, 0.0), r, t0)
        val out = filter.fuse(Triple(3.0, 0.0, 0.0), r, t0 + GnssMathUtils.NANOS_PER_SECOND)
        // Fused estimate must lie strictly between 0 and 3 (reinit would yield exactly 3).
        assertTrue("Fused X should be < measurement (filter pulled back toward prior)", out.first < 3.0)
        assertTrue("Fused X should be > prior (filter moved toward measurement)", out.first > 0.0)
    }

    @Test
    fun mahalanobisGate_largeInnovationWithLargeR_isRejected() {
        // R = 100 m² (σ ≈ 10 m per axis). A 10_000 m innovation gives Mahalanobis ≈ 1000 σ,
        // well above the 4σ chi-square gate, so the epoch must be rejected (coast on prior).
        val r = diagR(100.0)
        val t0 = 0L
        filter.fuse(Triple(0.0, 0.0, 0.0), r, t0)
        val out = filter.fuse(Triple(10_000.0, 0.0, 0.0), r, t0 + GnssMathUtils.NANOS_PER_SECOND)
        assertEquals("Gated epoch must not move the estimate to the outlier", 0.0, out.first, 1.0)
    }

    @Test
    fun euclideanSafetyNet_triggersEvenWhenCovarianceIsInflated() {
        // With R = 1e8 m² (σ ≈ 10 km per axis), a 600 m innovation is only 0.06 σ — the
        // Mahalanobis gate alone would *not* reject it. The Euclidean safety net
        // (KALMAN_MAX_INNOVATION_METERS = 500 m) must still gate the epoch (coast), and
        // persistence must re-initialize.
        val hugeR = diagR(1e8)
        var t = 0L
        val dt = GnssMathUtils.NANOS_PER_SECOND
        filter.fuse(Triple(0.0, 0.0, 0.0), hugeR, t)
        t += dt
        var out = filter.fuse(Triple(600.0, 0.0, 0.0), hugeR, t)
        assertEquals("First 600 m jump with σ ≈ 10 km is gated by the Euclidean net", 0.0, out.first, 1.0)
        repeat(GnssMathUtils.KALMAN_MAX_CONSECUTIVE_REJECTIONS - 1) {
            t += dt
            out = filter.fuse(Triple(600.0, 0.0, 0.0), hugeR, t)
        }
        assertEquals("Persistent 600 m offset must re-initialize", 600.0, out.first, 1.0)
    }

    @Test
    fun nanMeasurementCovariance_doesNotPoisonTheFilter() {
        val r = diagR(4.0)
        var t = 0L
        val dt = GnssMathUtils.NANOS_PER_SECOND
        filter.fuse(Triple(100.0, 0.0, 0.0), r, t)

        val poisoned =
            diagR(4.0).also {
                it[0][0] = Double.NaN
                it[1][2] = Double.NaN
            }
        t += dt
        val outPoisoned = filter.fuse(Triple(101.0, 0.0, 0.0), poisoned, t)
        assertTrue("Output must stay finite with NaN entries in R", outPoisoned.first.isFinite())

        // The filter must keep functioning on subsequent clean epochs.
        t += dt
        val outClean = filter.fuse(Triple(102.0, 0.0, 0.0), r, t)
        assertTrue(outClean.first.isFinite())
        assertEquals(102.0, outClean.first, 5.0)
    }

    @Test
    fun nanMeasurement_isIgnored() {
        val r = diagR(4.0)
        filter.fuse(Triple(100.0, 0.0, 0.0), r, 0L)
        val out = filter.fuse(Triple(Double.NaN, 0.0, 0.0), r, GnssMathUtils.NANOS_PER_SECOND)
        assertEquals("Non-finite measurement must be ignored (coast)", 100.0, out.first, 1e-6)
    }

    @Test
    fun innovationThresholdConstants_haveSaneValues() {
        // Compile-time guard: if someone ever resets these to the pre-fix 75 m value the gate
        // returns to being effectively disabled (at typical σ ≈ 5 m the required innovation
        // would be 75·5 = 375 m).
        assertTrue(
            "Mahalanobis sigma gate must be small (~3-5), was ${GnssMathUtils.KALMAN_MAX_INNOVATION_SIGMA}",
            GnssMathUtils.KALMAN_MAX_INNOVATION_SIGMA in 2.0..6.0,
        )
        assertTrue(
            "Euclidean safety net must be large enough to not trigger on normal noise (≥200 m)",
            GnssMathUtils.KALMAN_MAX_INNOVATION_METERS >= 200.0,
        )
    }

    @Test
    fun velocityClip_atSpeedLimit_doesNotDivergeCovariance() {
        // Drive a rapid X displacement that will push the filter's velocity estimate above
        // KALMAN_MAX_SPEED_MPS (55 m/s). After several epochs the clip should engage and the
        // filter must remain numerically stable (no NaN/Inf, bounded output).
        val r = diagR(1.0)
        var t = 0L
        val dtNs = GnssMathUtils.NANOS_PER_SECOND
        filter.fuse(Triple(0.0, 0.0, 0.0), r, t)
        // Jump X by 200 m every second for 5 s → implied speed 200 m/s (well above clip).
        var out = Triple(0.0, 0.0, 0.0)
        for (i in 1..5) {
            t += dtNs
            out = filter.fuse(Triple(i * 200.0, 0.0, 0.0), r, t)
        }
        assertTrue("Position X finite after velocity clip", out.first.isFinite())
        assertTrue("Position Y finite after velocity clip", out.second.isFinite())
        assertTrue("Position Z finite after velocity clip", out.third.isFinite())
    }

    @Test
    fun gatedEpoch_reportsPredictedCovarianceAndInnovation_acceptedEpochClearsIt() {
        val r = diagR(100.0)
        val t0 = 0L
        filter.fuse(Triple(0.0, 0.0, 0.0), r, t0)
        assertNull(filter.coastReport())

        val out = filter.fuse(Triple(10_000.0, 0.0, 0.0), r, t0 + GnssMathUtils.NANOS_PER_SECOND)
        assertEquals(0.0, out.first, 1.0)
        val coast = filter.coastReport()
        assertNotNull(coast)
        assertEquals(10_000.0, coast!!.innovationX, 1.0)
        assertTrue(coast.positionCovariance[0][0] > 0.0)

        filter.fuse(Triple(0.0, 0.0, 0.0), r, t0 + 2 * GnssMathUtils.NANOS_PER_SECOND)
        assertNull("an accepted fix must not keep the coast report", filter.coastReport())
    }

    @Test
    fun velocityMeasurement_updatesVelocity_andAbsenceLeavesItAtZero() {
        val r = diagR(25.0)
        filter.fuse(Triple(0.0, 0.0, 0.0), r, 0L)
        assertEquals(0.0, filter.velocityEcef().first, 1e-9)

        filter.fuse(
            Triple(0.0, 0.0, 0.0),
            r,
            GnssMathUtils.NANOS_PER_SECOND,
            velocityEcef = Triple(5.0, -1.0, 0.5),
            velocityVarianceM2PerS2 = Triple(0.01, 0.01, 0.01),
        )
        val velocity = filter.velocityEcef()
        assertEquals(5.0, velocity.first, 0.15)
        assertEquals(-1.0, velocity.second, 0.15)
        assertEquals(0.5, velocity.third, 0.15)
    }

    @Test
    fun displacementMeasurement_pullsPositionToTheCarrierDelta() {
        val r = diagR(25.0)
        filter.fuse(Triple(0.0, 0.0, 0.0), r, 0L)
        val out =
            filter.fuse(
                Triple(0.0, 0.0, 0.0),
                r,
                GnssMathUtils.NANOS_PER_SECOND,
                displacementEcef = Triple(1.0, -0.4, 0.2),
                displacementVarianceM2 = Triple(1e-4, 1e-4, 1e-4),
            )
        assertEquals(1.0, out.first, 0.05)
        assertEquals(-0.4, out.second, 0.05)
        assertEquals(0.2, out.third, 0.05)
    }
}
