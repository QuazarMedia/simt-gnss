package app.simt.gnss.core.orbit
import app.simt.gnss.core.model.Constellation
import app.simt.gnss.core.model.SatelliteEphemeris
import app.simt.gnss.core.utils.GnssMathUtils
import app.simt.gnss.core.utils.GnssMathUtils.GLONASS_AE
import app.simt.gnss.core.utils.GnssMathUtils.GLONASS_J2
import app.simt.gnss.core.utils.GnssMathUtils.GLONASS_MU
import app.simt.gnss.core.utils.GnssMathUtils.GLONASS_OMEGA_E
import app.simt.gnss.core.utils.GnssMathUtils.GLONASS_RK4_STEP
import app.simt.gnss.core.utils.GnssMathUtils.MU
import app.simt.gnss.core.utils.GnssMathUtils.OMEGA_EARTH
import app.simt.gnss.core.utils.GnssMathUtils.SPEED_OF_LIGHT
import kotlin.math.*

/**
 * Transmit epoch and the flight time used to rotate ECEF during signal travel.
 * [transmitTimeSeconds] is a constellation time-of-week; [rotationSeconds] is a duration.
 */
data class TransmitTiming(
    val transmitTimeSeconds: Double,
    val rotationSeconds: Double,
)

/**
 * Calculates satellite position and clock corrections from Ephemeris data.
 */
object SatellitePositionCalculator {
    /**
     * Per-constellation Keplerian gravitational constant. The broadcast sqrtA/Δn are fitted
     * against the value mandated by each system's ICD: IS-GPS-200 §20.3.3.4.3.3 requires
     * μ = 3.986005×10¹⁴ for GPS (QZSS/IRNSS follow), while Galileo OS SIS ICD and
     * BDS-SIS-ICD use 3.986004418×10¹⁴. Mixing them costs ~1 m of along-track error per
     * hour of ephemeris age for GPS.
     */
    private fun keplerMu(constellationType: Int): Double =
        when (constellationType) {
            Constellation.GALILEO, Constellation.BEIDOU -> MU
            else -> GnssMathUtils.GPS_MU
        }

    private const val VELOCITY_STEP_SECONDS = 0.01
    private const val MIN_APPROXIMATION_ALTITUDE_METERS = -500.0
    private const val MAX_APPROXIMATION_ALTITUDE_METERS = 20_000.0

    private fun sanitizeLatitudeDegrees(latitudeDegrees: Double): Double =
        if (latitudeDegrees.isFinite()) {
            latitudeDegrees.coerceIn(-90.0, 90.0)
        } else {
            0.0
        }

    private fun sanitizeLongitudeDegrees(longitudeDegrees: Double): Double {
        val rawLongitude = if (longitudeDegrees.isFinite()) longitudeDegrees else 0.0
        val wrappedLongitude = ((rawLongitude + 180.0) % 360.0 + 360.0) % 360.0 - 180.0
        return if (wrappedLongitude == -180.0) 180.0 else wrappedLongitude
    }

    private fun sanitizeAltitudeMeters(altitudeMeters: Double): Double =
        if (altitudeMeters.isFinite()) {
            altitudeMeters.coerceIn(MIN_APPROXIMATION_ALTITUDE_METERS, MAX_APPROXIMATION_ALTITUDE_METERS)
        } else {
            0.0
        }

    private fun sanitizeElevationDegrees(elevationDegrees: Double): Double =
        if (elevationDegrees.isFinite()) {
            elevationDegrees.coerceIn(0.0, 90.0)
        } else {
            90.0
        }

    /**
     * BeiDou GEO satellites use a distinct reference frame (tilted by −5° about the ECEF X-axis
     * and spinning with the broadcast frame rather than with Earth) as specified in
     * BDS-SIS-ICD-B1I §5.2.4.1. PRNs 1–5 are BDS-2 GEOs; PRNs 59–63 are BDS-3 GEOs. Applying the
     * generic IS-GPS-200 formulation to these satellites yields position errors on the order of
     * kilometres — the additional Rx(−5°) · Rz(ω_e · t_k) rotation below is mandatory.
     */
    private fun isBeidouGeo(ephemeris: SatelliteEphemeris): Boolean =
        ephemeris.constellationType == Constellation.BEIDOU &&
            (ephemeris.svid in 1..5 || ephemeris.svid in 59..63)

    /**
     * Calculates the precise ECEF position of a satellite using orbital parameters.
     * Implementation follows IS-GPS-200 for GPS/Galileo/QZSS/IRNSS/BeiDou-MEO/IGSO, the special
     * BDS-SIS-ICD-B1I §5.2.4.1 formulation for BeiDou GEO satellites, and RK4 integration for
     * GLONASS.
     */
    fun calculateSatellitePosition(
        ephemeris: SatelliteEphemeris,
        time: Double,
    ): Triple<Double, Double, Double> {
        if (ephemeris.isGlonass) return calculateGlonassPosition(ephemeris, time)
        if (ephemeris.isSbas) return calculateSbasPosition(ephemeris, time)

        val mu = keplerMu(ephemeris.constellationType)
        val omegaEarth =
            when (ephemeris.constellationType) {
                Constellation.BEIDOU -> 7.2921150e-5
                else -> OMEGA_EARTH
            }

        var tk = time - ephemeris.toe
        if (tk > 302400.0) tk -= 604800.0
        if (tk < -302400.0) tk += 604800.0

        val a = ephemeris.sqrtA * ephemeris.sqrtA
        val n = sqrt(mu / a.pow(3)) + ephemeris.deltaN
        val mk = ephemeris.m0 + n * tk

        var ek = mk
        for (iter in 0 until 10) {
            val ekNext = mk + ephemeris.eccentricity * sin(ek)
            if (abs(ekNext - ek) < 1e-12) {
                ek = ekNext
                break
            }
            ek = ekNext
        }

        val vk = atan2(sqrt(1.0 - ephemeris.eccentricity.pow(2)) * sin(ek), cos(ek) - ephemeris.eccentricity)
        val phik = vk + ephemeris.omega

        val duk = ephemeris.cus * sin(2.0 * phik) + ephemeris.cuc * cos(2.0 * phik)
        val drk = ephemeris.crs * sin(2.0 * phik) + ephemeris.crc * cos(2.0 * phik)
        val dik = ephemeris.cis * sin(2.0 * phik) + ephemeris.cic * cos(2.0 * phik)

        val uk = phik + duk
        val rk = a * (1.0 - ephemeris.eccentricity * cos(ek)) + drk
        val ik = ephemeris.i0 + ephemeris.iDot * tk + dik

        val xkPrime = rk * cos(uk)
        val ykPrime = rk * sin(uk)

        return if (isBeidouGeo(ephemeris)) {
            // BDS-SIS-ICD-B1I §5.2.4.1 GEO: Ω_k uses Ω_dot·t_k (no −ω_e·t_k term) and the
            // intermediate ECEF position is rotated by Rz(ω_e·t_k) · Rx(−5°).
            val omegakGeo = ephemeris.omega0 + ephemeris.omegaDot * tk - omegaEarth * ephemeris.toe
            val xGk = xkPrime * cos(omegakGeo) - ykPrime * cos(ik) * sin(omegakGeo)
            val yGk = xkPrime * sin(omegakGeo) + ykPrime * cos(ik) * cos(omegakGeo)
            val zGk = ykPrime * sin(ik)

            // Rx(−5°): rotate about X by −5°
            val phiX = Math.toRadians(-5.0)
            val cosPhiX = cos(phiX)
            val sinPhiX = sin(phiX)
            val x1 = xGk
            val y1 = yGk * cosPhiX + zGk * sinPhiX
            val z1 = -yGk * sinPhiX + zGk * cosPhiX

            // Rz(ω_e · t_k)
            val phiZ = omegaEarth * tk
            val cosPhiZ = cos(phiZ)
            val sinPhiZ = sin(phiZ)
            val x = x1 * cosPhiZ + y1 * sinPhiZ
            val y = -x1 * sinPhiZ + y1 * cosPhiZ
            val z = z1
            Triple(x, y, z)
        } else {
            // IS-GPS-200 §20.3.3.4.3.4: Ω_k = Ω_0 + (Ω_dot − ω_e) · t_k − ω_e · t_oe
            val omegak = ephemeris.omega0 + (ephemeris.omegaDot - omegaEarth) * tk - omegaEarth * ephemeris.toe
            val x = xkPrime * cos(omegak) - ykPrime * cos(ik) * sin(omegak)
            val y = xkPrime * sin(omegak) + ykPrime * cos(ik) * cos(omegak)
            val z = ykPrime * sin(ik)
            Triple(x, y, z)
        }
    }

    /**
     * ECEF velocity at [time] by central difference of [calculateSatellitePosition].
     * The same propagator supplies GPS, Galileo, BeiDou and GLONASS, so the velocity
     * stays in the frame the position already uses.
     */
    fun satelliteEcefVelocity(
        ephemeris: SatelliteEphemeris,
        time: Double,
    ): Triple<Double, Double, Double> {
        val ahead = calculateSatellitePosition(ephemeris, time + VELOCITY_STEP_SECONDS)
        val behind = calculateSatellitePosition(ephemeris, time - VELOCITY_STEP_SECONDS)
        val span = 2.0 * VELOCITY_STEP_SECONDS
        return Triple(
            (ahead.first - behind.first) / span,
            (ahead.second - behind.second) / span,
            (ahead.third - behind.third) / span,
        )
    }

    /** c · d(Δt_sv)/dt at [time], meters per second. Group delay is constant and drops out. */
    fun satelliteClockDriftMetersPerSecond(
        ephemeris: SatelliteEphemeris,
        time: Double,
    ): Double {
        val ahead = getSatelliteClockCorrection(ephemeris, time + VELOCITY_STEP_SECONDS, includeGroupDelay = false)
        val behind = getSatelliteClockCorrection(ephemeris, time - VELOCITY_STEP_SECONDS, includeGroupDelay = false)
        return (ahead - behind) / (2.0 * VELOCITY_STEP_SECONDS) * SPEED_OF_LIGHT
    }

    /**
     * Rotates satellite ECEF coordinates from the transmit-time Earth-fixed frame to the
     * reception-time frame. During signal propagation (~0.07 s for MEO GNSS), the ECEF axes
     * rotate with Earth by ω·τ about +Z; without this, line-of-sight geometry is biased on the
     * order of tens of meters.
     *
     * Convention: positive rotation about +Z matches WGS84 Earth rotation (same as [OMEGA_EARTH]).
     */
    fun rotateEcefForEarthRotationDuringTransit(
        transmitX: Double,
        transmitY: Double,
        transmitZ: Double,
        transitTimeSeconds: Double,
        earthRotationRateRadPerSec: Double = OMEGA_EARTH,
    ): Triple<Double, Double, Double> {
        if (!transitTimeSeconds.isFinite() || transitTimeSeconds <= 0.0) {
            return Triple(transmitX, transmitY, transmitZ)
        }
        val theta = earthRotationRateRadPerSec * transitTimeSeconds
        val cosT = cos(theta)
        val sinT = sin(theta)
        val x = transmitX * cosT + transmitY * sinT
        val y = -transmitX * sinT + transmitY * cosT
        val z = transmitZ
        return Triple(x, y, z)
    }

    /**
     * Transmit epoch and Earth-rotation interval for one pseudorange.
     *
     * [receptionTimeSeconds] and the pseudorange share the receiver clock, so that clock
     * cancels in the transmit epoch: t = t_rx − PR/c − Δt_sv (IS-GPS-200 §20.3.3.3.3.1).
     * The signal's flight time, which rotates the ECEF frame, is (PR − b̂)/c + Δt_sv.
     * [satelliteClockSeconds] is Δt_sv from one evaluation at t_rx − PR/c; af1 over that
     * step is negligible.
     */
    fun transmitTiming(
        receptionTimeSeconds: Double,
        pseudorangeMeters: Double,
        receiverClockBiasMeters: Double,
        satelliteClockSeconds: Double,
    ): TransmitTiming {
        val transmitTimeSeconds = receptionTimeSeconds - pseudorangeMeters / SPEED_OF_LIGHT - satelliteClockSeconds
        val rotationSeconds =
            (pseudorangeMeters - receiverClockBiasMeters) / SPEED_OF_LIGHT + satelliteClockSeconds
        return TransmitTiming(transmitTimeSeconds, rotationSeconds)
    }

    /**
     * Calculates satellite clock correction including relativistic effects.
     *
     * @param includeGroupDelay when true (default), subtracts the broadcast group delay
     *   (GPS TGD / Galileo BGD / BDS TGD1) — correct for an L1/E1/B1I single-frequency user.
     *   Pass false when the caller applies a signal-specific group delay itself (e.g. the
     *   ionosphere-free combination, whose per-signal biases cancel identically, or an
     *   L5-only user who must apply γ·TGD instead).
     */
    fun getSatelliteClockCorrection(
        ephemeris: SatelliteEphemeris,
        time: Double,
        includeGroupDelay: Boolean = true,
    ): Double {
        if (ephemeris.isGlonass) return getGlonassClockCorrection(ephemeris, time)
        if (ephemeris.isSbas) return getSbasClockCorrection(ephemeris, time)

        val mu = keplerMu(ephemeris.constellationType)
        val c = SPEED_OF_LIGHT

        var tkOrbit = time - ephemeris.toe
        if (tkOrbit > 302400.0) tkOrbit -= 604800.0
        if (tkOrbit < -302400.0) tkOrbit += 604800.0

        var tkClock = time - ephemeris.toc
        if (tkClock > 302400.0) tkClock -= 604800.0
        if (tkClock < -302400.0) tkClock += 604800.0

        val dt = ephemeris.af0 + ephemeris.af1 * tkClock + ephemeris.af2 * tkClock.pow(2)

        val mk = ephemeris.m0 + (sqrt(mu / ephemeris.sqrtA.pow(6)) + ephemeris.deltaN) * tkOrbit
        var ek = mk
        for (iter in 0 until 10) {
            val ekNext = mk + ephemeris.eccentricity * sin(ek)
            if (abs(ekNext - ek) < 1e-12) {
                ek = ekNext
                break
            }
            ek = ekNext
        }
        val dtr = (-2.0 * sqrt(mu) / (c * c)) * ephemeris.eccentricity * ephemeris.sqrtA * sin(ek)

        return dt + dtr - (if (includeGroupDelay) ephemeris.tgd else 0.0)
    }

    /**
     * Fallback approximation for satellite position in ECEF using Azimuth, Elevation, and Observer position.
     */
    fun getSatellitePositionApproximation(
        constellationType: Int,
        elevation: Double,
        azimuth: Double,
        observerLat: Double,
        observerLon: Double,
        observerAlt: Double,
    ): Triple<Double, Double, Double> {
        val orbitRadius =
            when (constellationType) {
                Constellation.GPS -> 26560000.0
                Constellation.GLONASS -> 25510000.0
                Constellation.GALILEO -> 29600000.0
                Constellation.BEIDOU -> 27906000.0
                Constellation.QZSS -> 42164000.0
                Constellation.IRNSS -> 42164000.0
                Constellation.SBAS -> 42164000.0
                else -> 26560000.0
            }

        val safeObserverLat = sanitizeLatitudeDegrees(observerLat)
        val safeObserverLon = sanitizeLongitudeDegrees(observerLon)
        val safeObserverAlt = sanitizeAltitudeMeters(observerAlt)
        val safeElevation = sanitizeElevationDegrees(elevation)
        val safeAzimuth = if (azimuth.isFinite()) azimuth else 0.0

        // Convert observer LLA to ECEF
        val (obsX, obsY, obsZ) = GnssMathUtils.latLonAltToEcef(safeObserverLat, safeObserverLon, safeObserverAlt)

        // Correct slant range from observer to satellite using spherical geometry:
        // d = -Re*sin(el) + sqrt(Rorbit² - Re²*cos²(el))
        // Use actual geocentric distance (not equatorial radius) to account for WGS84 flattening;
        // at 48°N the difference is ~17 km, causing elevation-dependent range errors of ~10 km.
        val observerR = sqrt(obsX * obsX + obsY * obsY + obsZ * obsZ)
        val elevRad = Math.toRadians(safeElevation)
        val sinEl = sin(elevRad)
        val cosEl = cos(elevRad)
        val reCosEl = observerR * cosEl
        val distance = -observerR * sinEl + sqrt(orbitRadius * orbitRadius - reCosEl * reCosEl)
        val azimRad = Math.toRadians(safeAzimuth)

        // Convert El/Az/Dist to Topocentric ENU (East, North, Up)
        val distHorizontal = distance * cos(elevRad)
        val e = distHorizontal * sin(azimRad)
        val n = distHorizontal * cos(azimRad)
        val u = distance * sin(elevRad)

        // Rotate ENU to ECEF
        val latRad = Math.toRadians(safeObserverLat)
        val lonRad = Math.toRadians(safeObserverLon)

        val sinLat = sin(latRad)
        val cosLat = cos(latRad)
        val sinLon = sin(lonRad)
        val cosLon = cos(lonRad)

        val dx = -sinLon * e - sinLat * cosLon * n + cosLat * cosLon * u
        val dy = cosLon * e - sinLat * sinLon * n + cosLat * sinLon * u
        val dz = cosLat * n + sinLat * u

        return Triple(obsX + dx, obsY + dy, obsZ + dz)
    }

    // ---- SBAS GEO satellite position (second-order Taylor expansion) ----

    /**
     * Computes SBAS GEO satellite ECEF position from MT 9 navigation message using
     * second-order Taylor expansion from the broadcast reference time t0:
     *   X(t) = X0 + Vx·dt + 0.5·Ax·dt²
     *
     * SBAS t0 is given as seconds within the current day. The solver passes GPS TOW,
     * so we convert to seconds-of-day and compute dt.
     *
     * @param ephemeris SBAS ephemeris with ECEF position/velocity/acceleration at t0
     * @param time GPS Time of Week (seconds) at signal transmission
     */
    private fun calculateSbasPosition(
        ephemeris: SatelliteEphemeris,
        time: Double,
    ): Triple<Double, Double, Double> {
        val timeOfDay = time % GnssMathUtils.SECONDS_IN_DAY
        var dt = timeOfDay - ephemeris.sbasT0
        // Handle day rollover
        if (dt > GnssMathUtils.SECONDS_IN_DAY / 2.0) dt -= GnssMathUtils.SECONDS_IN_DAY
        if (dt < -GnssMathUtils.SECONDS_IN_DAY / 2.0) dt += GnssMathUtils.SECONDS_IN_DAY

        val x = ephemeris.sbasPosX + ephemeris.sbasVelX * dt + 0.5 * ephemeris.sbasAccX * dt * dt
        val y = ephemeris.sbasPosY + ephemeris.sbasVelY * dt + 0.5 * ephemeris.sbasAccY * dt * dt
        val z = ephemeris.sbasPosZ + ephemeris.sbasVelZ * dt + 0.5 * ephemeris.sbasAccZ * dt * dt
        return Triple(x, y, z)
    }

    /**
     * SBAS satellite clock correction: dt_sv = aGf0 + aGf1·(t − t0).
     *
     * @param ephemeris SBAS ephemeris with clock bias and drift
     * @param time GPS Time of Week (seconds) at signal transmission
     */
    private fun getSbasClockCorrection(
        ephemeris: SatelliteEphemeris,
        time: Double,
    ): Double {
        val timeOfDay = time % GnssMathUtils.SECONDS_IN_DAY
        var dt = timeOfDay - ephemeris.sbasT0
        if (dt > GnssMathUtils.SECONDS_IN_DAY / 2.0) dt -= GnssMathUtils.SECONDS_IN_DAY
        if (dt < -GnssMathUtils.SECONDS_IN_DAY / 2.0) dt += GnssMathUtils.SECONDS_IN_DAY

        return ephemeris.sbasAgf0 + ephemeris.sbasAgf1 * dt
    }

    // ---- GLONASS orbit propagation (RK4 numerical integration) ----

    /**
     * Computes GLONASS satellite ECEF position by numerically integrating the
     * equations of motion from the broadcast epoch tb.
     * Follows GLONASS ICD Section 4.3.2 and RTKLIB ephemeris.c.
     *
     * @param ephemeris GLONASS ephemeris with Cartesian state vector at tb
     * @param time GPS Time of Week (seconds) at signal transmission
     */
    private fun calculateGlonassPosition(
        ephemeris: SatelliteEphemeris,
        time: Double,
    ): Triple<Double, Double, Double> {
        var dt = time - ephemeris.gloTbGpsTow
        if (dt > 302400.0) dt -= 604800.0
        if (dt < -302400.0) dt += 604800.0

        // The broadcast force model (constant luni-solar acceleration at tb, J2-only) is
        // specified for |t − tb| ≤ 15 min and broadcasts refresh every 30 min; integrating
        // further accumulates km-class error and unbounded CPU (one RK4 step per 60 s).
        // Callers gate usability on tb age; this clamp is the defensive bound.
        dt = dt.coerceIn(-GnssMathUtils.GLONASS_MAX_PROPAGATION_SECONDS, GnssMathUtils.GLONASS_MAX_PROPAGATION_SECONDS)

        // State vector: [x, y, z, vx, vy, vz] in meters and m/s
        val state =
            doubleArrayOf(
                ephemeris.gloPosX,
                ephemeris.gloPosY,
                ephemeris.gloPosZ,
                ephemeris.gloVelX,
                ephemeris.gloVelY,
                ephemeris.gloVelZ,
            )
        val acc = doubleArrayOf(ephemeris.gloAccX, ephemeris.gloAccY, ephemeris.gloAccZ)

        // Integrate from tb to transmission time using adaptive RK4 steps. Scratch buffers
        // are allocated once per call, not per step (the WLS calls this in a hot loop).
        val scratch = Rk4Scratch()
        var remaining = dt
        val step = if (dt < 0) -GLONASS_RK4_STEP else GLONASS_RK4_STEP
        while (abs(remaining) > 1e-9) {
            val h = if (abs(remaining) < GLONASS_RK4_STEP) remaining else step
            glonassRk4Step(state, acc, h, scratch)
            remaining -= h
        }

        return Triple(state[0], state[1], state[2])
    }

    private class Rk4Scratch {
        val k1 = DoubleArray(6)
        val k2 = DoubleArray(6)
        val k3 = DoubleArray(6)
        val k4 = DoubleArray(6)
        val w = DoubleArray(6)
    }

    /**
     * Single 4th-order Runge-Kutta step for GLONASS orbit in Earth-fixed frame.
     */
    private fun glonassRk4Step(
        state: DoubleArray,
        acc: DoubleArray,
        h: Double,
        scratch: Rk4Scratch,
    ) {
        glonassDeq(state, acc, scratch.k1)

        for (i in 0 until 6) scratch.w[i] = state[i] + scratch.k1[i] * h / 2.0
        glonassDeq(scratch.w, acc, scratch.k2)

        for (i in 0 until 6) scratch.w[i] = state[i] + scratch.k2[i] * h / 2.0
        glonassDeq(scratch.w, acc, scratch.k3)

        for (i in 0 until 6) scratch.w[i] = state[i] + scratch.k3[i] * h
        glonassDeq(scratch.w, acc, scratch.k4)

        for (i in 0 until 6) {
            state[i] += (scratch.k1[i] + 2.0 * scratch.k2[i] + 2.0 * scratch.k3[i] + scratch.k4[i]) * h / 6.0
        }
    }

    /**
     * Equations of motion for GLONASS satellite in PZ-90 Earth-fixed frame.
     * Includes central gravity, J2 perturbation, Coriolis/centrifugal, and broadcast accelerations.
     * Reference: GLONASS ICD A.3.1.2 / RTKLIB deq().
     */
    private fun glonassDeq(
        state: DoubleArray,
        acc: DoubleArray,
        out: DoubleArray,
    ) {
        val x = state[0]
        val y = state[1]
        val z = state[2]
        val vx = state[3]
        val vy = state[4]

        val r2 = x * x + y * y + z * z
        if (r2 <= 0.0) {
            out.fill(0.0)
            return
        }

        val r = sqrt(r2)
        val r3 = r2 * r
        val omg2 = GLONASS_OMEGA_E * GLONASS_OMEGA_E

        // J2 perturbation factor: 3/2 · J2 · μ · ae² / r⁵
        val a = 1.5 * GLONASS_J2 * GLONASS_MU * GLONASS_AE * GLONASS_AE / (r2 * r3)
        val b = 5.0 * z * z / r2 // 5·z²/r²
        val c = -GLONASS_MU / r3 - a * (1.0 - b) // -μ/r³ - a·(1 - 5z²/r²)

        out[0] = vx
        out[1] = vy
        out[2] = state[5]
        out[3] = (c + omg2) * x + 2.0 * GLONASS_OMEGA_E * vy + acc[0]
        out[4] = (c + omg2) * y - 2.0 * GLONASS_OMEGA_E * vx + acc[1]
        out[5] = (c - 2.0 * a) * z + acc[2]
    }

    /**
     * GLONASS satellite clock: dt = −τn + γn·(t − tb) = af0 + af1·dt.
     *
     * No separate relativistic term. τn and γn already contain it (IGS SSR v1.00 §7.2.2);
     * adding −2·r·v/c² on top biases each satellite by a constant −2·(r·v)/c metres for the
     * whole ±45 min window, because r and v are the tb state vector.
     */
    private fun getGlonassClockCorrection(
        ephemeris: SatelliteEphemeris,
        time: Double,
    ): Double {
        var dt = time - ephemeris.gloTbGpsTow
        if (dt > 302400.0) dt -= 604800.0
        if (dt < -302400.0) dt += 604800.0

        return ephemeris.af0 + ephemeris.af1 * dt
    }
}
