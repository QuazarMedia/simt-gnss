package app.simt.gnss

import android.location.GnssStatus
import app.simt.gnss.core.filter.GnssInterEpochKalmanFilter
import app.simt.gnss.core.filter.carrierDeltaMatchesPseudorangeRate
import app.simt.gnss.core.model.SatelliteMeasurement
import app.simt.gnss.core.orbit.SatellitePositionCalculator
import app.simt.gnss.core.solver.DopplerVelocitySolver
import app.simt.gnss.core.solver.TdcpDisplacementSolver
import app.simt.gnss.core.utils.GnssMathUtils
import app.simt.gnss.core.utils.GnssMathUtils.SPEED_OF_LIGHT
import kotlin.math.*

/**
 * TDCP (time-differenced carrier-phase) memory: the anchor epoch and per-signal
 * samples, plus the Doppler-velocity and speed/bearing derivations built on top of
 * it, and the per-epoch hardware clock step consumed by the displacement solve.
 * Cleared on session reset and hardware-clock discontinuities.
 */
internal class TdcpMemory(
    private val ephemerisStore: GnssEphemerisStore,
    private val interEpochKalman: GnssInterEpochKalmanFilter,
) {
    /** Published ECEF at the previous fix, the linearization point for carrier displacement. */
    private var tdcpAnchor: Triple<Double, Double, Double>? = null
    private var tdcpEpochNanos: Long = 0L
    private val tdcpSignals = HashMap<Pair<Int, Int>, TdcpSignal>()

    /** Hardware clock step for the epoch currently being solved. Zero when there is no previous clock. */
    internal var epochClockStepMeters: Double = 0.0

    internal data class ReceptionSatellite(
        val x: Double,
        val y: Double,
        val z: Double,
        val velocityX: Double,
        val velocityY: Double,
        val velocityZ: Double,
        val clockSeconds: Double,
        val clockDriftMetersPerSecond: Double,
    )

    internal data class TdcpSignal(
        val frequencyHz: Double,
        val adrMeters: Double,
        val satX: Double,
        val satY: Double,
        val satZ: Double,
        val satClockSeconds: Double,
    )

    /** Satellite position and clock at transmit, rotated into the reception frame. */
    private fun receptionSatellite(
        key: Pair<Int, Int>,
        meas: SatelliteMeasurement,
        clockBiases: DoubleArray,
        clockGroupToIndex: Map<Int, Int>,
        clockGroupBySatKey: Map<Pair<Int, Int>, Int>,
        gpstSeconds: Double,
    ): ReceptionSatellite? {
        val ephemeris = ephemerisStore.get(key)?.takeIf { ephemerisStore.isUsableEphemerisLocked(it, gpstSeconds) } ?: return null
        val timeSeconds =
            if (meas.constellationType == GnssStatus.CONSTELLATION_BEIDOU) {
                gpstSeconds - GnssMathUtils.BDT_GPST_OFFSET_SECONDS
            } else {
                gpstSeconds
            }
        val clockGroup = clockGroupBySatKey[key] ?: meas.constellationType
        val bias = clockBiases[clockGroupToIndex[clockGroup] ?: 0]
        val satelliteClock =
            SatellitePositionCalculator.getSatelliteClockCorrection(
                ephemeris,
                timeSeconds - meas.pseudorangeMeters / SPEED_OF_LIGHT,
                includeGroupDelay = false,
            )
        val timing =
            SatellitePositionCalculator.transmitTiming(
                receptionTimeSeconds = timeSeconds,
                pseudorangeMeters = meas.pseudorangeMeters,
                receiverClockBiasMeters = bias,
                satelliteClockSeconds = satelliteClock,
            )
        val transmit = SatellitePositionCalculator.calculateSatellitePosition(ephemeris, timing.transmitTimeSeconds)
        val transmitVelocity = SatellitePositionCalculator.satelliteEcefVelocity(ephemeris, timing.transmitTimeSeconds)
        val sat =
            SatellitePositionCalculator.rotateEcefForEarthRotationDuringTransit(
                transmit.first,
                transmit.second,
                transmit.third,
                timing.rotationSeconds,
            )
        val vel =
            SatellitePositionCalculator.rotateEcefForEarthRotationDuringTransit(
                transmitVelocity.first,
                transmitVelocity.second,
                transmitVelocity.third,
                timing.rotationSeconds,
            )
        return ReceptionSatellite(
            x = sat.first,
            y = sat.second,
            z = sat.third,
            velocityX = vel.first,
            velocityY = vel.second,
            velocityZ = vel.third,
            clockSeconds =
                SatellitePositionCalculator.getSatelliteClockCorrection(
                    ephemeris,
                    timing.transmitTimeSeconds,
                    includeGroupDelay = false,
                ),
            clockDriftMetersPerSecond =
                SatellitePositionCalculator.satelliteClockDriftMetersPerSecond(ephemeris, timing.transmitTimeSeconds),
        )
    }

    /**
     * One pseudorange rate per satellite. Null when fewer than five rates agree, which leaves
     * the inter-epoch filter on its position-only update.
     */
    internal fun dopplerVelocity(
        measurements: List<SatelliteMeasurement>,
        receiverX: Double,
        receiverY: Double,
        receiverZ: Double,
        clockBiases: DoubleArray,
        clockGroupToIndex: Map<Int, Int>,
        clockGroupBySatKey: Map<Pair<Int, Int>, Int>,
        gpstSeconds: Double,
    ): DopplerVelocitySolver.Solution? {
        val bestBySatellite = LinkedHashMap<Pair<Int, Int>, SatelliteMeasurement>()
        for (meas in measurements) {
            val rate = meas.pseudorangeRateMetersPerSecond
            val sigma = meas.pseudorangeRateUncertaintyMetersPerSecond
            if (!rate.isFinite() || !sigma.isFinite() || sigma <= 0.0 || sigma > MAX_PSEUDORANGE_RATE_SIGMA_MPS) continue
            val key = meas.constellationType to meas.svid
            val current = bestBySatellite[key]
            if (current == null || sigma < current.pseudorangeRateUncertaintyMetersPerSecond) {
                bestBySatellite[key] = meas
            }
        }
        val observations = ArrayList<DopplerVelocitySolver.Observation>(bestBySatellite.size)
        for ((key, meas) in bestBySatellite) {
            val sat = receptionSatellite(key, meas, clockBiases, clockGroupToIndex, clockGroupBySatKey, gpstSeconds) ?: continue
            val dx = sat.x - receiverX
            val dy = sat.y - receiverY
            val dz = sat.z - receiverZ
            val range = sqrt(dx * dx + dy * dy + dz * dz)
            if (range < 1.0e6) continue
            observations +=
                DopplerVelocitySolver.Observation(
                    losX = dx / range,
                    losY = dy / range,
                    losZ = dz / range,
                    satelliteVelocityX = sat.velocityX,
                    satelliteVelocityY = sat.velocityY,
                    satelliteVelocityZ = sat.velocityZ,
                    satelliteClockDriftMetersPerSecond = sat.clockDriftMetersPerSecond,
                    pseudorangeRateMetersPerSecond = meas.pseudorangeRateMetersPerSecond,
                    sigmaMetersPerSecond = meas.pseudorangeRateUncertaintyMetersPerSecond.coerceAtLeast(0.05),
                )
        }
        return DopplerVelocitySolver.solve(observations)
    }

    /** Horizontal speed and bearing from the filter velocity after a Doppler update. */
    internal fun speedAndBearingDegrees(
        latDeg: Double,
        lonDeg: Double,
    ): Pair<Double, Double> {
        val (vx, vy, vz) = interEpochKalman.velocityEcef()
        return speedAndBearingFromVelocityEcef(vx, vy, vz, latDeg, lonDeg)
    }

    internal fun speedAndBearingFromVelocityEcef(
        vx: Double,
        vy: Double,
        vz: Double,
        latDeg: Double,
        lonDeg: Double,
    ): Pair<Double, Double> {
        val (east, north, _) = GnssMathUtils.ecefDeltaToEnu(vx, vy, vz, latDeg, lonDeg)
        var bearing = Math.toDegrees(atan2(east, north))
        if (bearing < 0.0) bearing += 360.0
        return hypot(east, north) to bearing
    }

    /**
     * Carrier displacement from the previous published position. Null unless five slip-free
     * signals agree, which leaves the filter on the update it already has.
     */
    internal fun tdcpDisplacement(
        measurements: List<SatelliteMeasurement>,
        clockBiases: DoubleArray,
        clockGroupToIndex: Map<Int, Int>,
        clockGroupBySatKey: Map<Pair<Int, Int>, Int>,
        gpstSeconds: Double,
        gpsTimeNanos: Long,
    ): TdcpDisplacementSolver.Displacement? {
        val anchor = tdcpAnchor ?: return null
        val dtSeconds = (gpsTimeNanos - tdcpEpochNanos).toDouble() / GnssMathUtils.NANOS_PER_SECOND.toDouble()
        if (dtSeconds <= 0.0 || dtSeconds > GnssMathUtils.DOPPLER_MAX_DT_SECONDS) return null
        val satellites = ArrayList<TdcpDisplacementSolver.Satellite>()
        val seen = HashSet<Pair<Int, Int>>()
        for (meas in measurements) {
            val adr = meas.accumulatedDeltaRangeMeters ?: continue
            if (!hasValidAdr(meas.adrState) || isCarrierHalfCycleAmbiguous(meas.adrState)) continue
            val key = meas.constellationType to meas.svid
            if (key in seen) continue
            val previous = tdcpSignals[key] ?: continue
            if (abs(previous.frequencyHz - meas.carrierFrequencyHz) > 1_000.0) continue
            seen.add(key)
            val agrees =
                carrierDeltaMatchesPseudorangeRate(
                    currentAdrMeters = adr,
                    previousAdrMeters = previous.adrMeters,
                    dtSeconds = dtSeconds,
                    rateMetersPerSecond = meas.pseudorangeRateMetersPerSecond,
                    rateUncertaintyMetersPerSecond = meas.pseudorangeRateUncertaintyMetersPerSecond,
                )
            if (agrees != true) continue
            val sat = receptionSatellite(key, meas, clockBiases, clockGroupToIndex, clockGroupBySatKey, gpstSeconds) ?: continue
            val dx = sat.x - anchor.first
            val dy = sat.y - anchor.second
            val dz = sat.z - anchor.third
            val currentRange = sqrt(dx * dx + dy * dy + dz * dz)
            val previousDx = previous.satX - anchor.first
            val previousDy = previous.satY - anchor.second
            val previousDz = previous.satZ - anchor.third
            val previousRange = sqrt(previousDx * previousDx + previousDy * previousDy + previousDz * previousDz)
            if (currentRange < 1.0e6 || previousRange < 1.0e6) continue
            satellites +=
                TdcpDisplacementSolver.Satellite(
                    losX = dx / currentRange,
                    losY = dy / currentRange,
                    losZ = dz / currentRange,
                    previousRangeMeters = previousRange,
                    currentRangeMeters = currentRange,
                    previousAdrMeters = previous.adrMeters,
                    currentAdrMeters = adr,
                    previousSatelliteClockSeconds = previous.satClockSeconds,
                    currentSatelliteClockSeconds = sat.clockSeconds,
                )
        }
        return TdcpDisplacementSolver.solve(satellites, epochClockStepMeters)
    }

    internal fun rememberTdcp(
        measurements: List<SatelliteMeasurement>,
        positionX: Double,
        positionY: Double,
        positionZ: Double,
        clockBiases: DoubleArray,
        clockGroupToIndex: Map<Int, Int>,
        clockGroupBySatKey: Map<Pair<Int, Int>, Int>,
        gpstSeconds: Double,
        gpsTimeNanos: Long,
    ) {
        tdcpAnchor = Triple(positionX, positionY, positionZ)
        tdcpEpochNanos = gpsTimeNanos
        tdcpSignals.clear()
        for (meas in measurements) {
            val adr = meas.accumulatedDeltaRangeMeters ?: continue
            if (!hasValidAdr(meas.adrState) || isCarrierHalfCycleAmbiguous(meas.adrState)) continue
            val key = meas.constellationType to meas.svid
            if (tdcpSignals.containsKey(key) && !effectiveSignalBand(meas).isUpperBand) continue
            val sat = receptionSatellite(key, meas, clockBiases, clockGroupToIndex, clockGroupBySatKey, gpstSeconds) ?: continue
            tdcpSignals[key] =
                TdcpSignal(
                    frequencyHz = meas.carrierFrequencyHz,
                    adrMeters = adr,
                    satX = sat.x,
                    satY = sat.y,
                    satZ = sat.z,
                    satClockSeconds = sat.clockSeconds,
                )
        }
    }

    internal fun clear() {
        tdcpAnchor = null
        tdcpEpochNanos = 0L
        tdcpSignals.clear()
    }

    companion object {
        private const val MAX_PSEUDORANGE_RATE_SIGMA_MPS = 5.0
    }
}
