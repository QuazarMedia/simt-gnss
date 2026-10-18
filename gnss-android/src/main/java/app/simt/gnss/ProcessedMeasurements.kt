package app.simt.gnss

import app.simt.gnss.core.model.SatelliteMeasurement

/**
 * Result of processing one GNSS measurement epoch: the measurements that survived quality
 * gating, together with the receiver GPS time (nanoseconds) they belong to.
 *
 * @property measurements the usable satellite measurements for the epoch.
 * @property gpsTimeNanos the receiver's GPS time for the epoch, in nanoseconds.
 */
data class ProcessedMeasurements(
    val measurements: List<SatelliteMeasurement>,
    val gpsTimeNanos: Long,
)
