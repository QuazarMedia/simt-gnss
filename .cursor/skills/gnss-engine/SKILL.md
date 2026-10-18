---
name: gnss-engine
description: Navigate and extend the standalone SIMT GNSS engine library (simt-gnss) - :gnss-core pure-JVM parsers/solvers/filters/corrections/orbit, and :gnss-android (the processor plus the public SimtGnssEngine facade). Use for any task mentioning GNSS, RTK, PPP, RTCM, SBAS, NTRIP, ephemeris, pseudorange, carrier phase, ionosphere/troposphere, Klobuchar, satellite constellations, raw GnssMeasurement processing, or the simt-gnss repository itself.
---

# SIMT GNSS engine library (`simt-gnss`)

The GNSS processing pipeline that used to live in `simt-app/feature/gnss` now ships as a
standalone library so any Android app can use it as an alternative to RTKLIB. It is
physics-heavy and review-hardened (56+ tracked fixes) — **do not "simplify" formulas,
constants, or gating logic**; verify changes against the relevant ICD and the design docs.

## Modules

| Module | Kind | Contents |
|---|---|---|
| `:gnss-core` | pure Kotlin/JVM, **zero `android.*` imports** | parsers, solvers, filters, corrections, orbit, math, models — usable on desktop/server for post-processing |
| `:gnss-android` | Android library (namespace `app.simt.gnss`) | `GnssMeasurementProcessor` (internal orchestrator) + public `SimtGnssEngine` façade |

## Package map — `:gnss-core` (`gnss-core/src/main/java/app/simt/gnss/core/`)

| Package | Contents |
|---|---|
| `parser/` | Stateless `object` parsers: `GpsNavMessageParser` (LNAV + Klobuchar), `GpsCnavMt30Parser` (ISC), `GalileoNavMessageParser` (I/NAV), `BeidouNavMessageParser` (D1, BDT), `GlonassNavMessageParser` (L1OF, RK4 orbits), `SbasNavMessageParser` (MT1/2-5/9/24-25), `Rtcm3FrameParser` (CRC-24Q framing), `RtcmMsmParser` (MSM4–7 base obs), `Rtcm1005Parser` (base ECEF), `RtcmSsrDecoder` (SSR orbit/clock/bias) |
| `correction/` | Atmospheric models (Saastamoinen + Niell, Klobuchar), `SsrCorrections` store (IODE-matched, staleness-dropped), `RtkBaseStore`, `SsrCorrectionApplicator` |
| `solver/` | `GnssPositionSolver` (selector) → `GnssPositionSolverQr` (Householder, HIGH accuracy) / `GnssPositionSolverCholesky` (MEDIUM); `RtkDoubleDifferenceSolver`, `LambdaAmbiguityResolver`, `RtkAmbiguityMemory`, `DopplerVelocitySolver`, `TdcpDisplacementSolver` |
| `filter/` | `HatchFilter` (ADR smoothing + slip detection), `DopplerSmoother` (fallback), `GnssInterEpochKalmanFilter` (6-state ECEF CV; resets on time gap / innovation / mode change), `PseudorangeRate`, `ReceiverClockStep` |
| `orbit/` | `SatellitePositionCalculator` (Kepler + GLONASS RK4 + elevation/azimuth approximation fallback) |
| `model/` | `GnssModels` (`SatelliteMeasurement`, `SatelliteEphemeris`, `PositionResult`, `GnssSolutionMode`), plus the neutral `Constellation` and `AdrState` constants |
| `location/` | `RawGnssPositionFallbackResolver` (+ `RawGnssPositionComputer`) — constellation-exclusion fallback ladder |
| `agnss/` | `BroadcastEphemerisProvider` — RINEX 3 mixed broadcast-ephemeris parser (download is the caller's job) |
| `utils/` | `GnssMathUtils` — bit access utilities `getBits` / `getBitsLong` / `getSignedBitsLong` / `packBits` (MSB-first), constants (week/day nanos, leap seconds) |
| `validator/` | `GpsNavMessageValidator` (e.g. LNAV D30 inversion) |
| `platform/` | Neutral seams: `GnssLogger`, `TimeSource`, `GnssPlatform` |

## `:gnss-android` — the engine and its façade

| File | Contents |
|---|---|
| `GnssMeasurementProcessor.kt` | **internal orchestrator**: epoch processing, ephemeris maps keyed by (constellation, svid), SBAS state (MT1 mask + IODP/IODE), Klobuchar coefficients, PPP float state, solve entries `computePosition(...)` / `computeRtkPosition(...)` |
| `SimtGnssEngine.kt` | **public façade** delegating to the processor: `processNavigationMessage`, `processMeasurements`, `importExternalEphemeris`, `computePosition`, `computeRtkPosition`, `setSmoothingMode`, `ingestRtcmFrame`, `ssrStore`, `rtkBaseStore`, `describeStoredEphemerides`, `describeFreshEphemerisCoverage`, `diagnostics()` — plus `SimtGnssEngineDiagnostics` |
| `ProcessedMeasurements.kt`, `SmoothingMode.kt` | Public neutral types (`measurements` + `gpsTimeNanos`; `CARRIER_HATCH` / `DOPPLER`) |
| `AndroidGnssPlatform.kt` | Installs the Android backends into `GnssPlatform` (`SystemClock.elapsedRealtime()`, a `android.util.Log`-backed logger) |

Data flow: platform callbacks (`GnssMeasurementsEvent`, `GnssNavigationMessage`, `GnssStatus`) → quality gates (C/N0, elevation) → Hatch/Doppler smoothing → dual-freq IF combination when L1+L5 → per-iteration corrections (sat pos+clock, earth rotation, tropo, iono, Sagnac, relativistic) → iterated WLS with per-constellation clock states (ISB) → RAIM outlier rejection → covariance/DOP → inter-epoch Kalman → fix acceptance gate (consumer concern — the `gnss-integration` skill in `simt-app`).

## Neutral seams — the rule that keeps `:gnss-core` pure

`:gnss-core` must never gain an `android.*` import. Android concepts go through:

- `model/Constellation` (`GPS=1`, `SBAS=2`, `GLONASS=3`, `QZSS=4`, `BEIDOU=5`, `GALILEO=6`, `IRNSS=7`) instead of `android.location.GnssStatus.CONSTELLATION_*`.
- `model/AdrState` (`VALID=1`, `RESET=2`, `CYCLE_SLIP=4`) instead of `GnssMeasurement.ADR_STATE_*`.
- `GnssPlatform.timeSource.elapsedRealtimeMs()` instead of `android.os.SystemClock`.
- `GnssPlatform.logger` instead of `android.util.Log`.

Cross-module API that must stay `public` (used by consumers): `RawGnssPositionComputer`, `RawGnssPositionFallbackResolver`, `RtkAmbiguityMemory`, `DopplerVelocitySolver.solve`, `GnssInterEpochKalmanFilter.{KalmanCoast,coastReport(),velocityEcef()}`, `carrierDeltaMatchesPseudorangeRate`, `receiverClockStepMeters`, `GnssPositionSolver` (+ nested `Accuracy`), `SsrCorrectionApplicator`, `GpsNavMessageValidator` and the `model/` types.

## Convention: adding a parser for a new message type

1. **File** `gnss-core/src/main/java/app/simt/gnss/core/parser/<Name>Parser.kt` — a stateless `object` with `fun parse(payload: ByteArray): Result?`. Return `null` on malformed input; check minimum bit length first; read fields with `GnssMathUtils.getBits*` at explicit bit offsets with ICD scaling factors as named constants.
2. **Result** is a small `data class` next to the parser (or in `model/`).
3. **Register** the dispatch: RTCM types hook into the `Rtcm3FrameParser`/`RtcmSsrDecoder` dispatch (and the façade's `ingestRtcmFrame`); nav messages hook into `GnssMeasurementProcessor.processNavigationMessage`'s per-constellation branch.
4. **Test** in the mirrored dir `gnss-core/src/test/java/app/simt/gnss/core/parser/<Name>ParserTest.kt` using the **round-trip pattern**: build the payload with `GnssMathUtils.packBits` (or a local `writeSigned` helper — see `Rtcm1005ParserTest`), parse, assert fields to ICD precision. Also test: truncated payload → null, wrong message type → null, sign-boundary values.
5. Keep parsers allocation-light — they run per message at 1 Hz+ streams.

## Tests & commands

- `./gradlew :gnss-core:test` — pure JVM, no Robolectric. `./gradlew :gnss-android:test` — Robolectric; `gnss-android/src/test/resources/robolectric.properties` pins `sdk=36` (Robolectric's ceiling, below compileSdk 37).
- `./gradlew build` (compile + tests + lint) and `./gradlew spotlessApply` (ktlint, 4-space, LF, 300 cols) before committing.
- Solver/processor tests build **synthetic geometry**: `createConsistentMeasurement(...)`-style helpers in `GnssMeasurementProcessorTest` derive pseudoranges from `SatellitePositionCalculator` so the truth position is recoverable.
- `gnss-core/src/test/.../benchmarks/` holds model-validation tests (residual RMS improvement when a correction is applied) — extend these when adding a physical model.
- `gnss-android/src/test/.../SimtGnssEngineTest.kt` exercises the public façade.

## Invariants & gotchas (violations here caused real bugs)

- **Time systems**: receiver time is GPST-based. BDT is 14 s behind GPST (BeiDou pseudoranges need the offset before transit-time division). GLONASS tb is UTC(+3h)-based and **day-modulo** (`NANOS_PER_DAY`), all others **week-modulo** (`NANOS_PER_WEEK`); normalize wraparound/negative transit times before dividing by c. Leap seconds from `GnssClock` when available, else the constant fallback in `GnssMathUtils`.
- **Units/frames**: ECEF WGS84 meters internally; elevation/azimuth recomputed each WLS iteration; iono delay scales by f²; phase ambiguities are estimated **in meters, float only** (no integer fixing by design).
- **Pairing rules**: TGD/ISC corrections must match the signal actually used; IF combination only when both frequencies are valid for the same satellite; SSR orbit/clock applied only when correction IOD matches the broadcast ephemeris IODE; SBAS fast/long-term corrections resolved through the MT1 PRN-mask slot under the current IODP — never by raw PRN.
- **Ephemeris validity**: Kepler ~24 h from toe, GLONASS ~6 h from tb, SBAS GEO ~6 h; stale → fall back to `getSatellitePositionApproximation` (km-class, iteration-0 only).
- **Half-cycle / slip**: phase observations gated on half-cycle ambiguity flags; Hatch filter resets on detected slips — preserve these gates when touching smoothing.
- **Threading**: callbacks arrive on arbitrary threads; processor state is instance-locked and consumers serialize epochs. Keep new state inside the existing locking discipline.

## Governance docs (they live in `simt-app`)

- `docs/feature-plans/gnss/GNSS_FEATURE_PLAN.md` — model formulation, roadmap, **traceability matrix (§10) — update it when adding symbols**
- `docs/feature-plans/gnss/GNSS_CODE_REVIEW.md` — past findings with rationale; check before re-introducing a rejected idea (e.g. post-solve integer ambiguity rounding was deliberately removed)

When you fix a defect here, record it in `GNSS_CODE_REVIEW.md` following the existing numbered format.
