package app.simt.gnss.core.orbit
import app.simt.gnss.core.model.Constellation
import app.simt.gnss.core.model.SatelliteEphemeris
import app.simt.gnss.core.utils.GnssMathUtils
import org.junit.Assert.*
import org.junit.Test
import kotlin.math.*

class SatellitePositionCalculatorTest {
    @Test
    fun testSatellitePositionCalculation() {
        // A dummy ephemeris for a circular orbit
        val ephemeris =
            SatelliteEphemeris(
                constellationType = Constellation.GPS,
                svid = 1,
                week = 2300,
                toe = 0.0,
                af0 = 0.0,
                af1 = 0.0,
                af2 = 0.0,
                m0 = 0.0,
                deltaN = 0.0,
                eccentricity = 0.0,
                sqrtA = sqrt(26560000.0), // GPS semimajor axis ~26560km
                omega0 = 0.0,
                i0 = PI / 4.0, // 45 degrees
                omega = 0.0,
                omegaDot = 0.0,
                iDot = 0.0,
                cuc = 0.0,
                cus = 0.0,
                crc = 0.0,
                crs = 0.0,
                cic = 0.0,
                cis = 0.0,
                tgd = 0.0,
                lastUpdateTime = 0L,
            )

        val t = 0.0
        val pos = SatellitePositionCalculator.calculateSatellitePosition(ephemeris, t)

        // At t=0, M=0, E=0, v=0, u=0.
        // x' = r*cos(u) = 26560000.0, y' = r*sin(u) = 0.
        // For Omega_k = 0, i_k = 45 deg:
        // X = x'*cos(Omega) - y'*cos(i)*sin(Omega) = 26560000.0 * 1 = 26560000.0
        // Y = x'*sin(Omega) + y'*cos(i)*cos(Omega) = 0
        // Z = y'*sin(i) = 0

        assertEquals(26560000.0, pos.first, 1.0)
        assertEquals(0.0, pos.second, 1.0)
        assertEquals(0.0, pos.third, 1.0)
    }

    @Test
    fun testSatelliteClockCorrection() {
        val ephemeris =
            SatelliteEphemeris(
                constellationType = Constellation.GPS,
                svid = 1,
                week = 2300,
                toe = 0.0,
                af0 = 1e-4, // 100 microseconds
                af1 = 1e-9,
                af2 = 0.0,
                m0 = 0.0,
                deltaN = 0.0,
                eccentricity = 0.0,
                sqrtA = 5153.0,
                omega0 = 0.0,
                i0 = 0.0,
                omega = 0.0,
                omegaDot = 0.0,
                iDot = 0.0,
                cuc = 0.0,
                cus = 0.0,
                crc = 0.0,
                crs = 0.0,
                cic = 0.0,
                cis = 0.0,
                tgd = 1e-9,
                lastUpdateTime = 0L,
            )

        val t = 0.0
        val corr = SatellitePositionCalculator.getSatelliteClockCorrection(ephemeris, t)

        // At t=0, dt = t - toe = 0.
        // corr = af0 + af1*dt + af2*dt^2 + dtr - tgd
        // eccentricity = 0 => dtr = 0
        // corr = 1e-4 - 1e-9 = 9.9999e-5

        assertEquals(9.9999e-5, corr, 1e-12)
    }

    @Test
    fun rotateEcefForEarthRotationDuringTransit_matchesAnalyticFormulaOnEquator() {
        val x = 1.0e7
        val y = 0.0
        val z = 3.0e6
        val tau = 0.075 // ~75 ms light time for MEO range
        val omega = 7.2921151467e-5
        val theta = omega * tau

        val (rx, ry, rz) =
            SatellitePositionCalculator.rotateEcefForEarthRotationDuringTransit(
                transmitX = x,
                transmitY = y,
                transmitZ = z,
                transitTimeSeconds = tau,
                earthRotationRateRadPerSec = omega,
            )

        assertEquals(x * cos(theta) + y * sin(theta), rx, 1e-6)
        assertEquals(-x * sin(theta) + y * cos(theta), ry, 1e-6)
        assertEquals(z, rz, 1e-9)
    }

    @Test
    fun rotateEcefForEarthRotationDuringTransit_nonFiniteOrNonPositiveTime_isNoOp() {
        val (a, b, c) =
            SatellitePositionCalculator.rotateEcefForEarthRotationDuringTransit(1.0, 2.0, 3.0, Double.NaN)
        assertEquals(1.0, a, 0.0)
        assertEquals(2.0, b, 0.0)
        assertEquals(3.0, c, 0.0)
    }

    @Test
    fun getSatellitePositionApproximation_irnssUsesGeosynchronousOrbitRadius() {
        val navicPosition =
            SatellitePositionCalculator.getSatellitePositionApproximation(
                Constellation.IRNSS,
                elevation = 90.0,
                azimuth = 0.0,
                observerLat = 0.0,
                observerLon = 0.0,
                observerAlt = 0.0,
            )
        val gpsPosition =
            SatellitePositionCalculator.getSatellitePositionApproximation(
                Constellation.GPS,
                elevation = 90.0,
                azimuth = 0.0,
                observerLat = 0.0,
                observerLon = 0.0,
                observerAlt = 0.0,
            )

        val navicRadius = sqrt(navicPosition.first * navicPosition.first + navicPosition.second * navicPosition.second + navicPosition.third * navicPosition.third)
        val gpsRadius = sqrt(gpsPosition.first * gpsPosition.first + gpsPosition.second * gpsPosition.second + gpsPosition.third * gpsPosition.third)

        assertEquals(42164000.0, navicRadius, 1.0)
        assertTrue("NavIC approximation should use a higher orbit than GPS", navicRadius > gpsRadius)
    }

    @Test
    fun getSatellitePositionApproximation_clampsImpossibleObserverState() {
        val position =
            SatellitePositionCalculator.getSatellitePositionApproximation(
                Constellation.GPS,
                elevation = 35.0,
                azimuth = 120.0,
                observerLat = Double.NaN,
                observerLon = 540.0,
                observerAlt = 4.972484658008615E7,
            )

        assertTrue("Approximate satellite X should stay finite", position.first.isFinite())
        assertTrue("Approximate satellite Y should stay finite", position.second.isFinite())
        assertTrue("Approximate satellite Z should stay finite", position.third.isFinite())
    }

    // ---------------------------------------------------------------------------------------------
    //  BeiDou GEO: BDS-SIS-ICD-B1I v3.0 §5.2.4.1 — the GEO formulation uses Ω_k without the
    //  −ω_e·t_k term and then applies Rz(ω_e·t_k) · Rx(−5°). At t_k = 0 the Rz rotation is the
    //  identity and only the −5° tilt remains, giving a clean analytical reference.
    // ---------------------------------------------------------------------------------------------

    private fun beidouGeoLikeEphemeris(svid: Int) =
        SatelliteEphemeris(
            constellationType = Constellation.BEIDOU,
            svid = svid,
            week = 900,
            toe = 0.0,
            toc = 0.0,
            af0 = 0.0,
            af1 = 0.0,
            af2 = 0.0,
            m0 = 0.0,
            deltaN = 0.0,
            eccentricity = 0.0,
            // BDS-2 GEO: a ≈ 42,164,200 m (geostationary)
            sqrtA = sqrt(42_164_200.0),
            omega0 = 0.0,
            i0 = 0.0,
            omega = 0.0,
            omegaDot = 0.0,
            iDot = 0.0,
            cuc = 0.0,
            cus = 0.0,
            crc = 0.0,
            crs = 0.0,
            cic = 0.0,
            cis = 0.0,
            tgd = 0.0,
            lastUpdateTime = 0L,
        )

    @Test
    fun beidouGeo_atT0_appliesMinusFiveDegreeXRotationOnly() {
        val eph = beidouGeoLikeEphemeris(svid = 1)
        val pos = SatellitePositionCalculator.calculateSatellitePosition(eph, time = 0.0)

        // At t=0 with all angles zero and e=0: x_gk = a, y_gk = 0, z_gk = 0.
        // After Rx(-5°) the point on the +X axis is unchanged, and then Rz(0)=I.
        val a = 42_164_200.0
        assertEquals(a, pos.first, 1.0)
        assertEquals(0.0, pos.second, 1.0)
        assertEquals(0.0, pos.third, 1.0)
    }

    @Test
    fun beidouGeo_differsFromGenericFormulaForNonZeroYGk() {
        // Place the SV in-plane at u_k = 90° (ω = 90°) so the intermediate (x_gk, y_gk, z_gk) is
        // (0, a, 0). The −5° X-axis rotation then maps that to a point with |z| ≈ a·sin(5°) ≈
        // 3.67 Mm; the generic (non-GEO) formula would yield z = 0 for i = 0.
        val eph = beidouGeoLikeEphemeris(svid = 1).copy(omega = PI / 2.0)
        val pos = SatellitePositionCalculator.calculateSatellitePosition(eph, time = 0.0)

        val a = 42_164_200.0
        val expectedAbsZ = a * sin(Math.toRadians(5.0))
        assertEquals(
            "GEO −5° tilt must produce |z| = a·sin(5°) when y_gk = a",
            expectedAbsZ,
            abs(pos.third),
            1.0,
        )
    }

    @Test
    fun beidouMeo_usesGenericFormula_noTilt() {
        // PRN 11 is a BDS-2 MEO. The generic IS-GPS-200 formula should yield z=0 for i=0.
        val eph =
            beidouGeoLikeEphemeris(svid = 11).copy(
                sqrtA = sqrt(27_906_100.0), // BDS MEO semi-major axis
                omega = PI / 2.0,
            )
        val pos = SatellitePositionCalculator.calculateSatellitePosition(eph, time = 0.0)

        // For a MEO with i=0 at ω=90°, z should be ~0 (no −5° tilt applied).
        assertEquals("MEO at i=0 must have z≈0", 0.0, pos.third, 1.0)
    }

    // ---------------------------------------------------------------------------------------------
    //  Clock correction — verify that toc is used (not toe) for the clock polynomial when they
    //  differ, per fix §11 of GNSS_CODE_REVIEW.
    // ---------------------------------------------------------------------------------------------

    @Test
    fun clockCorrection_polynomialIsReferencedToToc_notToe() {
        val toe = 345_600.0
        val toc = toe + 600.0 // toc 10 min after toe
        val af0 = 1e-4
        val af1 = 1e-9
        val eph =
            SatelliteEphemeris(
                constellationType = Constellation.GPS,
                svid = 5,
                week = 2300,
                toe = toe,
                toc = toc,
                af0 = af0,
                af1 = af1,
                af2 = 0.0,
                m0 = 0.0,
                deltaN = 0.0,
                eccentricity = 0.0,
                sqrtA = 5153.0,
                omega0 = 0.0,
                i0 = 0.0,
                omega = 0.0,
                omegaDot = 0.0,
                iDot = 0.0,
                cuc = 0.0,
                cus = 0.0,
                crc = 0.0,
                crs = 0.0,
                cic = 0.0,
                cis = 0.0,
                tgd = 0.0,
                lastUpdateTime = 0L,
            )

        // Evaluating at t = toc should give exactly af0 (dt_clock = 0).
        val corr = SatellitePositionCalculator.getSatelliteClockCorrection(eph, toc)
        assertEquals("At t=toc, clock correction must equal af0 (plus dtr≈0 for e=0)", af0, corr, 1e-12)

        // Evaluating at t = toe should NOT equal af0 — would require dt = 0 which only happens at toc.
        val corrAtToe = SatellitePositionCalculator.getSatelliteClockCorrection(eph, toe)
        val expectedAtToe = af0 + af1 * (toe - toc)
        assertEquals("At t=toe with toc≠toe, clock correction must use (t-toc)", expectedAtToe, corrAtToe, 1e-12)
    }

    // -------------------------------------------------------------------------
    //  SBAS GEO position (second-order Taylor expansion)
    // -------------------------------------------------------------------------

    private fun sbasEphemeris(
        t0: Double = 43200.0,
        posX: Double = 42164000.0,
        posY: Double = 0.0,
        posZ: Double = 0.0,
        velX: Double = 0.0,
        velY: Double = 0.0,
        velZ: Double = 0.0,
        accX: Double = 0.0,
        accY: Double = 0.0,
        accZ: Double = 0.0,
        agf0: Double = 0.0,
        agf1: Double = 0.0,
    ) = SatelliteEphemeris(
        constellationType = Constellation.SBAS,
        svid = 135,
        isSbas = true,
        sbasT0 = t0,
        sbasPosX = posX,
        sbasPosY = posY,
        sbasPosZ = posZ,
        sbasVelX = velX,
        sbasVelY = velY,
        sbasVelZ = velZ,
        sbasAccX = accX,
        sbasAccY = accY,
        sbasAccZ = accZ,
        sbasAgf0 = agf0,
        sbasAgf1 = agf1,
        af0 = agf0,
        af1 = agf1,
        lastUpdateTime = 0L,
    )

    @Test
    fun sbasPosition_atT0_returnsExactReferencePosition() {
        val eph =
            sbasEphemeris(
                t0 = 43200.0,
                posX = 42164000.0,
                posY = -1000000.0,
                posZ = 500000.0,
                velX = 1.5,
                velY = -0.3,
                velZ = 0.1,
            )
        // At t = t0 (= 43200 s of day), dt = 0, position should be exactly the reference
        val tow = 43200.0 // matches t0 within the same day
        val pos = SatellitePositionCalculator.calculateSatellitePosition(eph, tow)
        assertEquals(42164000.0, pos.first, 1e-6)
        assertEquals(-1000000.0, pos.second, 1e-6)
        assertEquals(500000.0, pos.third, 1e-6)
    }

    @Test
    fun sbasPosition_withVelocity_linearMotionCorrect() {
        val eph =
            sbasEphemeris(
                t0 = 40000.0,
                posX = 42164000.0,
                posY = 0.0,
                posZ = 0.0,
                velX = 10.0,
                velY = -5.0,
                velZ = 2.0,
            )
        val dt = 60.0 // 60 seconds after t0
        val tow = 40000.0 + dt
        val pos = SatellitePositionCalculator.calculateSatellitePosition(eph, tow)
        assertEquals(42164000.0 + 10.0 * dt, pos.first, 1e-6)
        assertEquals(-5.0 * dt, pos.second, 1e-6)
        assertEquals(2.0 * dt, pos.third, 1e-6)
    }

    @Test
    fun sbasPosition_withAcceleration_quadraticTermCorrect() {
        val eph =
            sbasEphemeris(
                t0 = 50000.0,
                posX = 42164000.0,
                velX = 0.0,
                accX = 0.001, // 1 mm/s²
            )
        val dt = 100.0
        val tow = 50000.0 + dt
        val pos = SatellitePositionCalculator.calculateSatellitePosition(eph, tow)
        // x = X0 + 0 + 0.5 * 0.001 * 100² = 42164000.0 + 5.0
        assertEquals(42164000.0 + 0.5 * 0.001 * dt * dt, pos.first, 1e-6)
    }

    @Test
    fun sbasPosition_dayRollover_handlesNegativeDt() {
        // t0 near end of day (86300 s), GPS TOW wraps to next day (0 + small offset)
        val eph = sbasEphemeris(t0 = 86300.0, posX = 42164000.0, velX = 10.0)
        // TOW = 86500, which is 200 s past midnight → timeOfDay = 86500 % 86400 = 100
        // dt = 100 - 86300 = -86200, then + 86400 = 200 (correct)
        val tow = 86500.0
        val pos = SatellitePositionCalculator.calculateSatellitePosition(eph, tow)
        assertEquals(42164000.0 + 10.0 * 200.0, pos.first, 1e-6)
    }

    @Test
    fun sbasClock_atT0_returnsAgf0() {
        val eph = sbasEphemeris(t0 = 43200.0, agf0 = 1e-7, agf1 = 2e-11)
        val corr = SatellitePositionCalculator.getSatelliteClockCorrection(eph, 43200.0)
        assertEquals(1e-7, corr, 1e-15)
    }

    @Test
    fun sbasClock_withDrift_linearCorrection() {
        val eph = sbasEphemeris(t0 = 43200.0, agf0 = 1e-7, agf1 = 2e-11)
        val dt = 120.0
        val corr = SatellitePositionCalculator.getSatelliteClockCorrection(eph, 43200.0 + dt)
        assertEquals(1e-7 + 2e-11 * dt, corr, 1e-15)
    }

    @Test
    fun getSatellitePositionApproximation_sbas_usesGeoOrbitRadius() {
        val pos =
            SatellitePositionCalculator.getSatellitePositionApproximation(
                Constellation.SBAS,
                elevation = 90.0,
                azimuth = 0.0,
                observerLat = 0.0,
                observerLon = 0.0,
                observerAlt = 0.0,
            )
        val radius = sqrt(pos.first.pow(2) + pos.second.pow(2) + pos.third.pow(2))
        assertEquals("SBAS should use GEO orbit radius", 42164000.0, radius, 1.0)
    }

    @Test
    fun glonassClock_doesNotAddASecondRelativisticTerm() {
        // τn/γn already include relativity. A parallel r·v at tb used to add −2·(r·v)/c² on
        // top, a constant range bias of −2·(r·v)/c for the whole validity window.
        val x = 2.55e7
        val vx = 10.0
        val tb = 43_200.0
        val af0 = -1.0e-6
        val af1 = 2.0e-11
        val eph =
            SatelliteEphemeris(
                constellationType = Constellation.GLONASS,
                svid = 5,
                af0 = af0,
                af1 = af1,
                lastUpdateTime = 0L,
                gloPosX = x,
                gloVelX = vx,
                gloTbGpsTow = tb,
                isGlonass = true,
            )
        val dt = 120.0
        val corr = SatellitePositionCalculator.getSatelliteClockCorrection(eph, tb + dt)
        val broadcast = af0 + af1 * dt
        assertEquals(broadcast, corr, 1e-15)
        val c = GnssMathUtils.SPEED_OF_LIGHT
        val removedSeconds = -2.0 * (x * vx) / (c * c)
        assertTrue(
            "the old −2·r·v/c² term is ${"%.2f".format(removedSeconds * c)} m on this state and must stay out",
            abs(corr - (broadcast + removedSeconds)) > 1e-9,
        )
    }

    @Test
    fun transmitTiming_removesSatelliteClockAndKeepsReceiverClockInTheFlightTime() {
        val reception = 100_000.0
        val pseudorange = 20_200_000.0
        val receiverClock = 300_000.0
        val satelliteClock = 5.0e-4
        val timing =
            SatellitePositionCalculator.transmitTiming(
                receptionTimeSeconds = reception,
                pseudorangeMeters = pseudorange,
                receiverClockBiasMeters = receiverClock,
                satelliteClockSeconds = satelliteClock,
            )
        val c = GnssMathUtils.SPEED_OF_LIGHT
        assertEquals(reception - pseudorange / c - satelliteClock, timing.transmitTimeSeconds, 1e-12)
        assertEquals((pseudorange - receiverClock) / c + satelliteClock, timing.rotationSeconds, 1e-12)
        // Old epoch was reception − (PR − b̂)/c. The difference is b̂/c + Δt_sv, about 6 m along-track here.
        val oldEpoch = reception - (pseudorange - receiverClock) / c
        val alongTrackMeters = (oldEpoch - timing.transmitTimeSeconds) * 3_900.0
        assertTrue(alongTrackMeters > 5.0)
    }

    @Test
    fun satelliteClockDrift_matchesAf1TimesCWhenEccentricityIsZero() {
        val af1 = 1.0e-10
        val ephemeris =
            SatelliteEphemeris(
                constellationType = Constellation.GPS,
                svid = 1,
                toe = 0.0,
                af1 = af1,
                eccentricity = 0.0,
                sqrtA = sqrt(26_560_000.0),
                lastUpdateTime = 0L,
            )
        val drift = SatellitePositionCalculator.satelliteClockDriftMetersPerSecond(ephemeris, 1_000.0)
        assertEquals(af1 * GnssMathUtils.SPEED_OF_LIGHT, drift, 1e-4)
    }

    @Test
    fun glonassVelocity_atTbMatchesTheBroadcastVelocity() {
        val ephemeris =
            SatelliteEphemeris(
                constellationType = Constellation.GLONASS,
                svid = 1,
                isGlonass = true,
                gloPosX = 1.5e7,
                gloPosY = 1.5e7,
                gloPosZ = 1.5e7,
                gloVelX = 1_000.0,
                gloVelY = -1_500.0,
                gloVelZ = 800.0,
                gloTbGpsTow = 1_000.0,
                lastUpdateTime = 0L,
            )
        val velocity = SatellitePositionCalculator.satelliteEcefVelocity(ephemeris, 1_000.0)
        assertEquals(1_000.0, velocity.first, 0.05)
        assertEquals(-1_500.0, velocity.second, 0.05)
        assertEquals(800.0, velocity.third, 0.05)
    }
}
