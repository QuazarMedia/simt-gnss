package app.simt.gnss.core.solver

import app.simt.gnss.core.utils.GnssMathUtils.SPEED_OF_LIGHT

/**
 * Receiver displacement from time-differenced carrier phase.
 *
 * Each satellite contributes one observation already cleared by the Doppler slip test.
 * With the receiver held at the previous position, the carrier change that is not satellite
 * motion, satellite clock, or the hardware clock step is
 * `y = −los · Δr + Δb`.
 * Fewer than five such satellites, or a residual above [RESIDUAL_FLOOR_METERS] that cannot
 * be dropped without falling under five, returns null. One cycle is below the Doppler slip
 * floor, so this residual is what rejects it.
 */
object TdcpDisplacementSolver {
    const val RESIDUAL_FLOOR_METERS = 0.10
    private const val RESIDUAL_SIGMA = 4.0
    const val SIGMA_METERS = 0.02

    data class Satellite(
        val losX: Double,
        val losY: Double,
        val losZ: Double,
        val previousRangeMeters: Double,
        val currentRangeMeters: Double,
        val previousAdrMeters: Double,
        val currentAdrMeters: Double,
        val previousSatelliteClockSeconds: Double,
        val currentSatelliteClockSeconds: Double,
    )

    data class Displacement(
        val deltaX: Double,
        val deltaY: Double,
        val deltaZ: Double,
        val varianceX: Double,
        val varianceY: Double,
        val varianceZ: Double,
    )

    fun solve(
        satellites: List<Satellite>,
        clockStepMeters: Double,
    ): Displacement? {
        val observations =
            satellites.map { sat ->
                val compensatedDelta = sat.currentAdrMeters - sat.previousAdrMeters - clockStepMeters
                val satelliteMotion = sat.currentRangeMeters - sat.previousRangeMeters
                val satelliteClockMeters =
                    (sat.currentSatelliteClockSeconds - sat.previousSatelliteClockSeconds) * SPEED_OF_LIGHT
                DopplerVelocitySolver.Observation(
                    losX = sat.losX,
                    losY = sat.losY,
                    losZ = sat.losZ,
                    satelliteVelocityX = 0.0,
                    satelliteVelocityY = 0.0,
                    satelliteVelocityZ = 0.0,
                    satelliteClockDriftMetersPerSecond = 0.0,
                    pseudorangeRateMetersPerSecond = compensatedDelta - satelliteMotion + satelliteClockMeters,
                    sigmaMetersPerSecond = SIGMA_METERS,
                )
            }
        val solved =
            DopplerVelocitySolver.solve(
                observations,
                residualFloor = RESIDUAL_FLOOR_METERS,
                residualSigma = RESIDUAL_SIGMA,
            ) ?: return null
        return Displacement(
            deltaX = solved.velocityX,
            deltaY = solved.velocityY,
            deltaZ = solved.velocityZ,
            varianceX = solved.varianceX,
            varianceY = solved.varianceY,
            varianceZ = solved.varianceZ,
        )
    }
}
