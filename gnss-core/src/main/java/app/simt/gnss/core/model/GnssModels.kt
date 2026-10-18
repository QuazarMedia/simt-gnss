package app.simt.gnss.core.model

/**
 * Data class representing a processed single-satellite measurement.
 */
data class SatelliteMeasurement(
    val constellationType: Int,
    val svid: Int,
    val pseudorangeMeters: Double,
    /**
     * Pseudorange before carrier or Doppler smoothing. RTK code double-differences use this:
     * the base observation is raw, and a smoothed rover against a raw base puts the rover-only
     * smoothing bias into every difference. Defaults to [pseudorangeMeters] so a caller that
     * never smoothed is unchanged.
     */
    val rawPseudorangeMeters: Double = pseudorangeMeters,
    val carrierFrequencyHz: Double,
    val cn0DbHz: Double,
    val elevationDegrees: Double,
    val azimuthDegrees: Double,
    val accumulatedDeltaRangeMeters: Double?,
    val pseudorangeUncertaintyMeters: Double,
    val isL5: Boolean = false,
    val constellationName: String = "",
    val wavelengthMeters: Double = 0.0,
    val carrierPhaseMeters: Double? = null,
    val adrState: Int = 0,
    /** Uncorrected pseudorange rate. NaN when the platform did not provide one. */
    val pseudorangeRateMetersPerSecond: Double = Double.NaN,
    val pseudorangeRateUncertaintyMetersPerSecond: Double = Double.NaN,
)

/** How a [PositionResult] was solved. */
enum class GnssSolutionMode {
    /** Undifferenced absolute positioning (broadcast / SBAS / PPP-SSR). */
    ABSOLUTE,

    /** RTK double-difference with float ambiguities (decimeter-class). */
    RTK_FLOAT,

    /** RTK double-difference with integer-fixed ambiguities (centimeter-class). */
    RTK_FIXED,
}

/**
 * Result of a position computation.
 */
data class PositionResult(
    val latitude: Double,
    val longitude: Double,
    val altitude: Double,
    val horizontalAccuracyMeters: Float,
    val verticalAccuracyMeters: Float,
    val timestamp: Long,
    val gdop: Double = 0.0,
    val pdop: Double = 0.0,
    val hdop: Double = 0.0,
    val vdop: Double = 0.0,
    val tdop: Double = 0.0,
    val residualsRms: Double = 0.0,
    val satelliteCount: Int = 0,
    val usedInFixCount: Int = 0,
    val solutionMode: GnssSolutionMode = GnssSolutionMode.ABSOLUTE,
    /** Horizontal speed from Doppler, null when fewer than five rates agreed. */
    val speedMetersPerSecond: Double? = null,
    /** Clockwise from true north, degrees, null when [speedMetersPerSecond] is null. */
    val bearingDegrees: Double? = null,
)

/**
 * Generic Ephemeris data from navigation message.
 * Contains Keplerian elements for precise orbit calculation.
 */
data class SatelliteEphemeris(
    val constellationType: Int,
    val svid: Int,
    val week: Int = 0,
    /**
     * Issue of Data of this broadcast ephemeris (GPS IODE, Galileo IODnav), or -1 when
     * unknown. SSR orbit/clock corrections are referenced to one specific broadcast issue
     * and must only be applied when this matches the correction's IOD.
     */
    val iode: Int = -1,
    val toe: Double = 0.0,
    val toc: Double = toe,
    val af0: Double = 0.0,
    val af1: Double = 0.0,
    val af2: Double = 0.0,
    val m0: Double = 0.0,
    val deltaN: Double = 0.0,
    val eccentricity: Double = 0.0,
    val sqrtA: Double = 0.0,
    val omega0: Double = 0.0,
    val i0: Double = 0.0,
    val omega: Double = 0.0,
    val omegaDot: Double = 0.0,
    val iDot: Double = 0.0,
    val cuc: Double = 0.0,
    val cus: Double = 0.0,
    val crc: Double = 0.0,
    val crs: Double = 0.0,
    val cic: Double = 0.0,
    val cis: Double = 0.0,
    val tgd: Double = 0.0,
    /**
     * Galileo BGD(E1, E5a), seconds. 0 unless this record is Galileo and the field was decoded.
     * The F/NAV clock is referenced to this delay (OS SIS ICD §5.1.5).
     */
    val galileoBgdE1E5aSeconds: Double = 0.0,
    /**
     * Galileo BGD(E1, E5b), seconds. The I/NAV clock is referenced to this delay.
     */
    val galileoBgdE1E5bSeconds: Double = 0.0,
    /**
     * True when [af0]/[af1]/[af2] are the I/NAV (E1/E5b) clock. OTA Galileo is always I/NAV.
     * A RINEX record with data-source bit 8 set is F/NAV and stays false.
     */
    val galileoInavClock: Boolean = false,
    val lastUpdateTime: Long,
    // GLONASS-specific fields (PZ-90 Earth-fixed Cartesian state vector at epoch tb)
    val gloPosX: Double = 0.0,
    val gloPosY: Double = 0.0,
    val gloPosZ: Double = 0.0,
    val gloVelX: Double = 0.0,
    val gloVelY: Double = 0.0,
    val gloVelZ: Double = 0.0,
    val gloAccX: Double = 0.0,
    val gloAccY: Double = 0.0,
    val gloAccZ: Double = 0.0,
    val gloTauN: Double = 0.0,
    val gloGammaN: Double = 0.0,
    val gloTbGpsTow: Double = 0.0,
    val gloBn: Int = 0,
    val isGlonass: Boolean = false,
    // SBAS GEO satellite fields (MT 9 ephemeris — ECEF position, velocity, acceleration at t0)
    val isSbas: Boolean = false,
    val sbasT0: Double = 0.0,
    val sbasPosX: Double = 0.0,
    val sbasPosY: Double = 0.0,
    val sbasPosZ: Double = 0.0,
    val sbasVelX: Double = 0.0,
    val sbasVelY: Double = 0.0,
    val sbasVelZ: Double = 0.0,
    val sbasAccX: Double = 0.0,
    val sbasAccY: Double = 0.0,
    val sbasAccZ: Double = 0.0,
    val sbasAgf0: Double = 0.0,
    val sbasAgf1: Double = 0.0,
)
