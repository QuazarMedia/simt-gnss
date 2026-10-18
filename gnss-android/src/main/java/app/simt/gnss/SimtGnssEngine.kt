package app.simt.gnss

import android.location.GnssMeasurementsEvent
import android.location.GnssNavigationMessage
import android.location.GnssStatus
import app.simt.gnss.core.correction.RtkBaseStore
import app.simt.gnss.core.correction.SsrCorrections
import app.simt.gnss.core.model.PositionResult
import app.simt.gnss.core.model.SatelliteEphemeris
import app.simt.gnss.core.model.SatelliteMeasurement
import app.simt.gnss.core.parser.Rtcm1005Parser
import app.simt.gnss.core.parser.Rtcm3FrameParser
import app.simt.gnss.core.parser.RtcmMsmParser
import app.simt.gnss.core.parser.RtcmSsrDecoder
import app.simt.gnss.core.solver.GnssPositionSolver

/**
 * Public entry point to the GNSS engine.
 *
 * The engine turns raw Android GNSS callbacks — measurement epochs, navigation messages and
 * status — into ECEF positions, plus optional RTK/PPP corrections decoded from an RTCM 3 stream.
 * It owns the per-receiver state (ephemeris store, carrier/Doppler smoothers, inter-epoch Kalman
 * filter, SSR and RTK-base correction stores), so **one instance per receiver/antenna** is
 * expected; all public methods are safe to call from any thread.
 *
 * The heavy algorithmic work lives in `:gnss-core`; this class is a thin, stable façade over the
 * internal [GnssMeasurementProcessor] and exposes only neutral model types.
 *
 * @param suppressSbas When `true`, SBAS (WAAS, EGNOS, MSAS, GAGAN, SDCM, …) is completely
 *   bypassed: SBAS navigation messages are not decoded, SBAS satellites are not used as ranging
 *   sources, and SBAS fast/long-term/iono-grid corrections are never applied. Intended for
 *   dual-frequency (L1 + L5 / E5) devices, where the iono-free combination is more accurate than
 *   the SBAS grid and decoding the 250 bps stream cannot improve the solution.
 */
class SimtGnssEngine(
    suppressSbas: Boolean = false,
) {
    private val processor = GnssMeasurementProcessor(suppressSbas)

    @Volatile
    private var lastSolution: PositionResult? = null

    init {
        AndroidGnssPlatform.install()
    }

    /** SSR (State Space Representation) corrections decoded from an IGS RTS / PPP stream. */
    val ssrStore: SsrCorrections.Store get() = processor.ssrStore

    /** RTK reference-station observations and position decoded from an OSR (MSM) stream. */
    val rtkBaseStore: RtkBaseStore get() = processor.rtkBaseStore

    /** Select the carrier-smoothing strategy used before the position solve. */
    fun setSmoothingMode(mode: SmoothingMode) = processor.setSmoothingMode(mode)

    /** Feed a single navigation message (ephemeris/almanac source) into the engine. */
    fun processNavigationMessage(message: GnssNavigationMessage) = processor.processNavigationMessage(message)

    /**
     * Process a GNSS measurement epoch and extract the usable satellite measurements.
     *
     * @param selectedConstellationTypes `android.location.GnssStatus.CONSTELLATION_*` integers to
     *   include. Measurements whose constellation is not in the set are dropped before filtering or
     *   solving. When `null`, no constellation-level filtering is applied.
     */
    fun processMeasurements(
        event: GnssMeasurementsEvent,
        gnssStatus: GnssStatus?,
        selectedConstellationTypes: Set<Int>? = null,
    ): ProcessedMeasurements = processor.processMeasurements(event, gnssStatus, selectedConstellationTypes)

    /**
     * Import externally obtained ephemerides (e.g. broadcast ephemeris downloaded from an A-GNSS
     * service).
     *
     * @return the number of records that replaced an older or absent entry.
     */
    fun importExternalEphemeris(
        records: List<SatelliteEphemeris>,
        gpsTimeOfWeekSeconds: Double,
    ): Int = processor.importExternalEphemeris(records, gpsTimeOfWeekSeconds)

    /**
     * Solve an absolute (standalone or PPP-corrected) position from one epoch's measurements.
     *
     * @param initialLat initial latitude in degrees (linearization/fallback seed).
     * @param initialLon initial longitude in degrees.
     * @param initialAlt initial altitude in metres.
     * @param usePpp apply SSR corrections from [ssrStore] when available.
     * @param gpsTimeNanos receiver GPS time for the epoch, in nanoseconds.
     * @param accuracy solver accuracy/effort trade-off.
     * @return the solution, or `null` when fewer than four usable measurements are available.
     */
    fun computePosition(
        measurements: List<SatelliteMeasurement>,
        initialLat: Double = 0.0,
        initialLon: Double = 0.0,
        initialAlt: Double = 0.0,
        usePpp: Boolean = true,
        gpsTimeNanos: Long = 0L,
        accuracy: GnssPositionSolver.Accuracy = GnssPositionSolver.Accuracy.HIGH,
    ): PositionResult? =
        processor
            .computePosition(measurements, initialLat, initialLon, initialAlt, usePpp, gpsTimeNanos, accuracy)
            .also { if (it != null) lastSolution = it }

    /**
     * Solve an RTK (relative) position against the base station in [rtkBaseStore].
     *
     * @param gpsTimeNanos receiver GPS time for the epoch, in nanoseconds.
     * @return the solution, or `null` when no base position is known or the epoch is unusable.
     */
    fun computeRtkPosition(
        measurements: List<SatelliteMeasurement>,
        gpsTimeNanos: Long,
        initialLat: Double = 0.0,
        initialLon: Double = 0.0,
        initialAlt: Double = 0.0,
    ): PositionResult? =
        processor
            .computeRtkPosition(measurements, gpsTimeNanos, initialLat, initialLon, initialAlt)
            .also { if (it != null) lastSolution = it }

    /** Human-readable summary of the ephemerides currently held by the engine (for diagnostics). */
    fun describeStoredEphemerides(): String = processor.describeStoredEphemerides()

    /** Human-readable summary of which constellations the candidate measurements cover. */
    fun describeFreshEphemerisCoverage(measurements: List<SatelliteMeasurement>): String = processor.describeFreshEphemerisCoverage(measurements)

    /**
     * Feed raw RTCM 3 bytes (for example an NTRIP correction stream) into the engine.
     *
     * Frames carrying SSR corrections, MSM base observations or base-station coordinates are routed
     * to [ssrStore] and [rtkBaseStore]; unhandled message types are ignored. Data may be supplied in
     * arbitrary chunks; framing is handled internally.
     */
    fun ingestRtcmFrame(frame: ByteArray) = rtcmStreamParser.feed(frame)

    /** Snapshot of the engine's current state for logging or a diagnostics panel. */
    fun diagnostics(): SimtGnssEngineDiagnostics =
        SimtGnssEngineDiagnostics(
            storedEphemerides = processor.describeStoredEphemerides(),
            ssrCorrections = processor.ssrStore.describe(),
            rtkBase = processor.rtkBaseStore.describe(),
            rtkBaseFresh = processor.rtkBaseStore.hasFreshData(),
            lastSolution = lastSolution,
        )

    private val rtcmStreamParser =
        Rtcm3FrameParser.StreamParser { frame ->
            when {
                RtcmSsrDecoder.isSupported(frame.messageType) ->
                    RtcmSsrDecoder.decode(frame, processor.ssrStore)
                RtcmMsmParser.isSupported(frame.messageType) ->
                    RtcmMsmParser
                        .parse(frame.payload)
                        ?.let { processor.rtkBaseStore.putObservations(it.observations) }
                Rtcm1005Parser.isSupported(frame.messageType) ->
                    Rtcm1005Parser.parse(frame.payload)?.let {
                        processor.rtkBaseStore.setBasePosition(it.xMeters, it.yMeters, it.zMeters)
                    }
            }
        }
}

/**
 * Neutral snapshot of a [SimtGnssEngine]'s current state.
 *
 * @property storedEphemerides human-readable summary of the held ephemerides.
 * @property ssrCorrections human-readable summary of the SSR correction store.
 * @property rtkBase human-readable summary of the RTK base-observation store.
 * @property rtkBaseFresh `true` when the base store holds enough fresh observations for RTK.
 * @property lastSolution the most recent solution produced by the engine, if any.
 */
data class SimtGnssEngineDiagnostics(
    val storedEphemerides: String,
    val ssrCorrections: String,
    val rtkBase: String,
    val rtkBaseFresh: Boolean,
    val lastSolution: PositionResult?,
) {
    val hasFix: Boolean get() = lastSolution != null
}
