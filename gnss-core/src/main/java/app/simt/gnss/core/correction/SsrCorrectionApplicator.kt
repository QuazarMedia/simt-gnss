package app.simt.gnss.core.correction

import app.simt.gnss.core.model.SatelliteEphemeris
import app.simt.gnss.core.utils.GnssMathUtils
import kotlin.math.sqrt

/**
 * Applies SSR (State Space Representation) corrections to broadcast ephemeris-derived
 * satellite positions and clocks.
 *
 * SSR orbit corrections are given in the satellite-body radial/along-track/cross-track (RAC) frame.
 * This class converts them to ECEF deltas and applies them to the broadcast position.
 *
 * **Fallback strategy:**
 * - If SSR data is available and IOD matches: apply orbit+clock corrections → decimeter accuracy
 * - If SSR is stale or IOD mismatch: use broadcast-only → meter accuracy (existing behavior)
 * - If no ephemeris at all: use geometric approximation (existing behavior)
 *
 * All methods are stateless and thread-safe.
 */
object SsrCorrectionApplicator {
    /**
     * Apply SSR orbit correction to a broadcast-derived ECEF satellite position.
     *
     * The correction transforms from radial/along-track/cross-track (RAC) frame
     * to ECEF using the satellite position and velocity vectors.
     *
     * @param satX Broadcast ECEF X (meters)
     * @param satY Broadcast ECEF Y (meters)
     * @param satZ Broadcast ECEF Z (meters)
     * @param satVx Satellite ECEF velocity X (m/s) — from SatellitePositionCalculator
     * @param satVy Satellite ECEF velocity Y (m/s)
     * @param satVz Satellite ECEF velocity Z (m/s)
     * @param deltaRadial Radial correction (meters, positive = away from earth)
     * @param deltaAlongTrack Along-track correction (meters, positive = in velocity direction)
     * @param deltaCrossTrack Cross-track correction (meters, positive = right of velocity)
     * @return Corrected ECEF position (x, y, z)
     */
    fun applyOrbitCorrection(
        satX: Double,
        satY: Double,
        satZ: Double,
        satVx: Double,
        satVy: Double,
        satVz: Double,
        deltaRadial: Double,
        deltaAlongTrack: Double,
        deltaCrossTrack: Double,
        nonRotatingVelocity: Boolean = false,
    ): Triple<Double, Double, Double> {
        val r = sqrt(satX * satX + satY * satY + satZ * satZ)
        if (r < 1.0) return Triple(satX, satY, satZ)

        // IGS SSR v1.00 §4.1 and RTKLIB `satpos_ssr` build this triad from the ECEF
        // velocity. Only a GEO, whose ECEF velocity can be ~0, uses the non-rotating
        // velocity ṙ_ECEF + ω⊕ × r.
        val vx: Double
        val vy: Double
        val vz: Double
        if (nonRotatingVelocity) {
            vx = satVx - GnssMathUtils.OMEGA_EARTH * satY
            vy = satVy + GnssMathUtils.OMEGA_EARTH * satX
            vz = satVz
        } else {
            vx = satVx
            vy = satVy
            vz = satVz
        }
        val v = sqrt(vx * vx + vy * vy + vz * vz)
        if (v < 1e-6) return Triple(satX, satY, satZ)

        val eA = doubleArrayOf(vx / v, vy / v, vz / v)

        val cx = satY * vz - satZ * vy
        val cy = satZ * vx - satX * vz
        val cz = satX * vy - satY * vx
        val cmag = sqrt(cx * cx + cy * cy + cz * cz)
        if (cmag < 1e-10) return Triple(satX, satY, satZ)
        val eC = doubleArrayOf(cx / cmag, cy / cmag, cz / cmag)

        // Radial completes the right-handed triad: eR = eA × eC (equals r̂ only for a
        // circular orbit; the flight-path angle is part of the SSR frame definition).
        val eR =
            doubleArrayOf(
                eA[1] * eC[2] - eA[2] * eC[1],
                eA[2] * eC[0] - eA[0] * eC[2],
                eA[0] * eC[1] - eA[1] * eC[0],
            )

        // SSR convention: correction is subtracted (δr reduces range when positive)
        // Per RTCM SSR: correctedPos = broadcastPos - (δR·eR + δA·eA + δC·eC)
        val corrX = satX - (deltaRadial * eR[0] + deltaAlongTrack * eA[0] + deltaCrossTrack * eC[0])
        val corrY = satY - (deltaRadial * eR[1] + deltaAlongTrack * eA[1] + deltaCrossTrack * eC[1])
        val corrZ = satZ - (deltaRadial * eR[2] + deltaAlongTrack * eA[2] + deltaCrossTrack * eC[2])

        return Triple(corrX, corrY, corrZ)
    }

    /**
     * Apply SSR clock correction to a broadcast-derived satellite clock error.
     *
     * @param broadcastClockCorr Broadcast clock correction in meters (af0 + af1·dt + af2·dt² scaled by c)
     * @param c0 SSR clock correction C0 (meters)
     * @param c1 SSR clock correction C1 (m/s)
     * @param c2 SSR clock correction C2 (m/s²)
     * @param dtSsr Time since SSR epoch (seconds)
     * @return Corrected clock error in meters
     */
    fun applyClockCorrection(
        broadcastClockCorr: Double,
        c0: Double,
        c1: Double = 0.0,
        c2: Double = 0.0,
        dtSsr: Double = 0.0,
    ): Double {
        // SSR clock correction is added to broadcast clock
        // δClock = C0 + C1·dt + C2·dt²
        val ssrClockDelta = c0 + c1 * dtSsr + c2 * dtSsr * dtSsr
        return broadcastClockCorr + ssrClockDelta
    }

    /**
     * Compute approximate satellite ECEF velocity from broadcast Keplerian ephemeris.
     *
     * This is needed to construct the RAC frame for orbit corrections.
     * Uses finite differencing over a small time step.
     *
     * @param ephemeris Broadcast ephemeris
     * @param timeSec Transmission time in seconds of week
     * @param posEcef Satellite ECEF position at timeSec
     * @param calculator Function that computes ECEF position from (ephemeris, time)
     * @return (vx, vy, vz) velocity in m/s
     */
    fun computeSatelliteVelocity(
        ephemeris: SatelliteEphemeris,
        timeSec: Double,
        posEcef: Triple<Double, Double, Double>,
        calculator: (SatelliteEphemeris, Double) -> Triple<Double, Double, Double>,
    ): Triple<Double, Double, Double> {
        val dt = 0.5 // seconds (finite difference step)
        val (xp, yp, zp) = calculator(ephemeris, timeSec + dt)
        val (xm, ym, zm) = calculator(ephemeris, timeSec - dt)
        return Triple(
            (xp - xm) / (2.0 * dt),
            (yp - ym) / (2.0 * dt),
            (zp - zm) / (2.0 * dt),
        )
    }

    /**
     * Age of an SSR rate polynomial. The rates are referenced to epoch + ½ update interval
     * (RTKLIB `t1 -= udi/2`). An unknown interval (0) leaves the age at t − epoch.
     */
    fun ssrPolynomialAgeSeconds(
        gpsSeconds: Double,
        epochTime: Double,
        updateIntervalSeconds: Double,
    ): Double = GnssMathUtils.wrapToHalfWeekSeconds(gpsSeconds - epochTime) - updateIntervalSeconds / 2.0
}
