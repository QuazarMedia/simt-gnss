# Plan — Extract the GNSS engine into the `simt-gnss` library

Status: In progress
Owner: simt-app extraction

## 1. Goal

Move the GNSS processing engine out of `simt-app`'s `feature:gnss` module into a
**standalone, Simt-agnostic Kotlin library** hosted in the sibling repository
`~/code/simt/simt-gnss`. The library must contain no Simt code (no `app.simt.core.*`,
no Hilt, no Compose, no Play Services) and must be reusable by any Android app as an
alternative to RTKLIB. `simt-app` consumes it and keeps only its UI, DI and platform
adapters.

## 2. Target state

Two repositories:

- **`~/code/simt/simt-gnss`** — new standalone library.
  - `:gnss-core` — pure Kotlin/JVM algorithms (parsers, solvers, filters, corrections,
    orbit, math, models). No Android dependency.
  - `:gnss-android` — Android library: the orchestrator/processor, the public `GnssEngine`
    façade, platform mappers and neutral diagnostics.
  - Namespace `app.simt.gnss.*`.
- **`simt-app`** — `feature:gnss` keeps UI + DI + location/cloud adapters, deletes the
  moved engine, and depends on `app.simt:gnss-android` via a **composite build**.

## 3. What moves (engine-only — 34 main files)

### `:gnss-core` (pure JVM)

| Source (from `feature/gnss/src/main/java/app/simt/feature/gnss/`) |
|---|
| `data/parser/*` (10 files) |
| `data/solver/*` (8 files) |
| `data/filter/*` (5 files) |
| `data/correction/*` (4 files) |
| `data/orbit/SatellitePositionCalculator.kt` |
| `data/utils/GnssMathUtils.kt` |
| `data/model/GnssModels.kt` |
| `data/validator/GpsNavMessageValidator.kt` |
| `data/agnss/BroadcastEphemerisProvider.kt` (RINEX parser) |
| `data/location/RawGnssPositionFallbackResolver.kt` |

### `:gnss-android`

| Source |
|---|
| `data/GnssMeasurementProcessor.kt` (the orchestrator, ~2,958 LOC) |
| new public `GnssEngine` façade + platform mappers + `GnssEngineDiagnostics` |

### Stays in `simt-app` (13 files)

`data/cloud/{RtkCorrectionServiceImpl, PppCorrectionServiceImpl, BroadcastEphemerisServiceImpl, CorrectionCoordinator, ChunkedStreamDecoder, NmeaGga}.kt`;
`data/location/{GnssLocationServiceImpl, GpsProviderLocationServiceImpl, RawGnssLocationServiceImpl}.kt`;
`di/GnssModule.kt`; `ui/{GnssScreen, GnssSettingsSheet, GnssViewModel}.kt`.

Rationale: the cloud files are Simt network/domain adapters (NTRIP, RINEX download,
decoding helpers), not GNSS processing; the location files are `core:domain`
`LocationService` implementations; `di`/`ui` are Hilt/Compose.

## 4. Making `:gnss-core` Android-free

The engine touches Android in exactly four places, all mechanical to replace. Introduce
four neutral seams in `app.simt.gnss.core`:

1. **`Constellation`** — `object` with `GPS=1, SBAS=2, GLONASS=3, QZSS=4, BEIDOU=5,
   GALILEO=6, IRNSS=7` (values identical to `GnssStatus.CONSTELLATION_*`).
   Replaces every `GnssStatus.CONSTELLATION_*` use (parsers, `SatellitePositionCalculator`,
   `RtcmMsmParser`, `RtcmSsrDecoder`, `RtkBaseStore`, `BroadcastEphemerisProvider`).
2. **`AdrState`** — `object` with `VALID=1, RESET=2, CYCLE_SLIP=4` (identical to
   `GnssMeasurement.ADR_STATE_*`). Replaces uses in `HatchFilter`, `RtkAmbiguityMemory`.
3. **`GnssLogger`** — interface (`d/w/e`) with a no-op default; `:gnss-android` wires an
   `android.util.Log` delegate. Replaces all `Log.*` uses.
4. **`TimeSource`** — `elapsedRealtimeMs()`; JVM default `System.nanoTime()/1_000_000`,
   Android override `SystemClock.elapsedRealtime()`. Replaces all
   `SystemClock.elapsedRealtime()` uses.

`constellationType` stays `Int` in `SatelliteMeasurement`/`SatelliteEphemeris`, so the
data format and every downstream comparison are unchanged. These substitutions are
renames only — no formula, constant or gating logic changes.

## 5. Public API (`:gnss-android`)

`GnssEngine` (public) wraps the existing `internal GnssMeasurementProcessor`:

- `onNavigationMessage(GnssNavigationMessage)`
- `onMeasurements(GnssMeasurementsEvent, GnssStatus?): ProcessedMeasurements`
- `solve(initialLat, initialLon, initialAlt, gpsTimeNanos, accuracy, usePpp): PositionResult?`
- `solveRtk(gpsTimeNanos, initial…): PositionResult?`
- `importEphemeris(List<SatelliteEphemeris>, towSeconds): Int`
- `setSmoothingMode(...)`, `ingestRtcmFrame(ByteArray)` (routes to `Rtcm3FrameParser` /
  MSM / SSR), `ssrStore` / `rtkBaseStore` access
- `diagnostics: StateFlow<GnssEngineDiagnostics>` (new neutral type)

Neutral model types (`PositionResult`, `SatelliteMeasurement`, `SatelliteEphemeris`,
`GnssSolutionMode`, `ProcessedMeasurements`) become public API.

`simt-app` maps `GnssEngineDiagnostics` → its `core:domain` `RawGnssDiagnostics` inside
`GnssLocationServiceImpl`; `core:domain` is untouched.

## 6. Repository scaffolding

`simt-gnss/`:

- `settings.gradle.kts` including `:gnss-core` and `:gnss-android`.
- Version catalog pinned to Simt's toolchain: Kotlin 2.4.21, AGP 9.4.1, compileSdk 37,
  minSdk 28.
- Gradle wrapper 9.4.x (copied from `simt-app`).
- `spotless`/ktlint config (4-space indent, LF, 300-char line limit).
- `.gitignore`, `README.md`, `LICENSE` (Apache-2.0).
- Adopted by `simt-dev` (`simt init --yes`) after the build files exist, so the
  `gradle`/`kotlin` stack rules are projected.
- `:gnss-core` uses the JVM library plugin; `:gnss-android` uses `com.android.library` +
  Kotlin. `:gnss-core` tests run as plain JVM; `:gnss-android` tests use Robolectric
  4.16.1 (matching Simt).

`simt-app` wiring:

- `settings.gradle.kts`: `includeBuild("../simt-gnss")` with `dependencySubstitution` for
  `app.simt:gnss-core` / `app.simt:gnss-android`.
- `feature/gnss/build.gradle.kts`: `implementation("app.simt:gnss-android")`.

## 7. Migration sequence

1. Scaffold `simt-gnss`; verify the empty build compiles.
2. Copy the 34 engine files + engine tests into the two modules; repackage
   `app.simt.feature.gnss.data.*` → `app.simt.gnss.core.*` / `app.simt.gnss.*`.
3. Apply the four core seams (§4).
4. Add the `GnssEngine` façade; make engine types public.
5. In `simt-app`: rewrite the 13 adapters to import from the library; delete moved code
   with `git rm` (tracked files only — no recursive `rm`); fix imports.
6. Validate.

History is not preserved across repos (fresh commits); tests port unchanged.

## 8. Acceptance checks / gates

- Library: `cd simt-gnss && ./gradlew :gnss-core:test :gnss-android:test` all pass;
  `./gradlew build` succeeds.
- Anti-coupling: `grep -rn "app\.simt\.core\|app\.simt\.feature" simt-gnss/**/src` returns
  nothing.
- Consumer: `cd simt-app && ./gradlew :feature:gnss:testDebugUnitTest` green (same test
  count as today); `./gradlew :app:assembleDebug` green.
- Test parity is the definition of a clean extraction: identical tests, identical
  assertions, passing unchanged.

## 9. Risks / guardrails

- The processor and formulas are review-hardened (56+ fixes,
  `docs/feature-plans/gnss/GNSS_CODE_REVIEW.md`). The move is **rename-only**; any
  formula/gating change is a defect and is out of scope.
- `GnssScreen`/`GnssViewModel` (Compose) stay behind — no UI moves.
- Composite build means a local path dependency; publishing/CI is deliberately deferred.

## 10. Implementation slice tracker

| id | state | depends on | slice |
|----|-------|-----------|-------|
| S1 | Queued | none | Scaffold `simt-gnss` Gradle build (`:gnss-core` + `:gnss-android`), wrapper, version catalog, license/readme |
| S2 | Queued | S1 | Adopt repo with `simt init --yes`; commit scaffold |
| S3 | Queued | S1 | Port `:gnss-core` sources + tests; apply the four Android-free seams |
| S4 | Queued | S3 | Port `:gnss-android` processor + tests; add `GnssEngine` façade and `GnssEngineDiagnostics` |
| S5 | Queued | S4 | Wire the composite build into `simt-app`; rewrite adapters; `git rm` moved code |
| S6 | Queued | S5 | Full validation: library tests, anti-coupling grep, `:feature:gnss:testDebugUnitTest`, `:app:assembleDebug` |
