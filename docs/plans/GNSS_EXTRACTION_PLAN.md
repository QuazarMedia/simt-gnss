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
  - `:gnss-android` — Android library: the orchestrator/processor, the public `SimtGnssEngine`
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
| new public `SimtGnssEngine` façade + platform mappers + `SimtGnssEngineDiagnostics` |

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

`SimtGnssEngine` (public) is a thin façade over the public-in-module `internal
GnssMeasurementProcessor`; its names mirror the processor so `simt-app` only swaps the type:

- `processNavigationMessage(GnssNavigationMessage)`
- `processMeasurements(GnssMeasurementsEvent, GnssStatus?, selectedConstellationTypes): ProcessedMeasurements`
- `computePosition(measurements, initialLat, initialLon, initialAlt, usePpp, gpsTimeNanos, accuracy): PositionResult?`
- `computeRtkPosition(measurements, gpsTimeNanos, initial…): PositionResult?`
- `importExternalEphemeris(List<SatelliteEphemeris>, towSeconds): Int`
- `setSmoothingMode(SmoothingMode)`, `ingestRtcmFrame(ByteArray)` (routes to
  `Rtcm3FrameParser` / `RtcmMsmParser` / `Rtcm1005Parser` / `RtcmSsrDecoder`),
  `ssrStore` / `rtkBaseStore`, `describeStoredEphemerides()`,
  `describeFreshEphemerisCoverage(measurements)`
- `diagnostics(): SimtGnssEngineDiagnostics` (new neutral snapshot type)

`ProcessedMeasurements` and `SmoothingMode` were promoted from the processor's nested
declarations to public top-level types in `app.simt.gnss`; neutral model types
(`PositionResult`, `SatelliteMeasurement`, `SatelliteEphemeris`, `GnssSolutionMode`) are
public in `:gnss-core`.

`simt-app` swaps `GnssMeasurementProcessor` → `SimtGnssEngine` and the moved imports;
`core:domain` is untouched.

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
4. Add the `SimtGnssEngine` façade; make engine types public.
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
| S1 | Complete | none | Scaffold `simt-gnss` Gradle build (`:gnss-core` + `:gnss-android`), wrapper, version catalog, license/readme |
| S2 | Complete | S1 | Adopt repo with `simt init --yes`; commit scaffold |
| S3 | Complete | S1 | Port `:gnss-core` sources + tests; apply the four Android-free seams |
| S4 | Complete | S3 | Port `:gnss-android` processor + tests; add `SimtGnssEngine` façade and `SimtGnssEngineDiagnostics` |
| S5 | Complete | S4 | Wire the composite build into `simt-app`; rewrite adapters; `git rm` moved code |
| S6 | Complete | S5 | Full validation: library tests, anti-coupling grep, `:feature:gnss:testDebugUnitTest`, `:app:assembleDebug` |

### Evidence

- **S1** scaffold — commit `fb25c8a`; `./gradlew projects` OK.
- **S2** adoption — commit `fb25c8a`; simt-dev wrote `simt.json` (id `simt-gnss`, stack gradle),
  `AGENTS.md`, `.editorconfig`, `.gitattributes`.
- **S3** `:gnss-core` — commit `f680647`; `./gradlew :gnss-core:test` → 38 classes / 252 tests /
  0 failures; `spotlessCheck` OK.
- **S4** `:gnss-android` — commit `b26063d`; `./gradlew :gnss-core:test :gnss-android:test` →
  44 classes / 287 tests / 0 failures; `./gradlew build` green.
  Parity: simt-app `:feature:gnss` 295 tests = 287 moved + 8 staying
  (`ChunkedStreamDecoderTest` 4, `NmeaGgaTest` 3, `GnssViewModelTest` 1).
- **S5** composite build + adapter rewrite — `simt-app` commit `5c84f549e`; 34 sources + 42 tests
  deleted, `GnssLocationServiceImpl` repointed to `app.simt.gnss(.core).*`.
- **S6** validation — library `./gradlew build` green (core 38 classes / 252 tests,
  android 7 classes / 40 tests = 292, the 5 extra being the new `SimtGnssEngineTest`); simt-app
  `:feature:gnss:testDebugUnitTest` 3 classes / 8 tests green; `:app:assembleDebug` green;
  anti-coupling grep over `gnss-core/src` + `gnss-android/src` CLEAN (no `app.simt.core` /
  `app.simt.feature` / androidx / dagger references), and `:gnss-core` has no `android.*` imports.

### Post-extraction changes

- **Façade rename** — the public façade `GnssEngine` was renamed to `SimtGnssEngine`, together
  with its snapshot type (`GnssEngineDiagnostics` → `SimtGnssEngineDiagnostics`) and its test
  (`GnssEngineTest` → `SimtGnssEngineTest`), so the public API stays branded to Simt alongside
  the `app.simt.gnss` package. `simt-app`'s `GnssLocationServiceImpl` was repointed in the same
  change. Library tests re-verified: `:gnss-core` 38 classes / 252 tests, `:gnss-android`
  7 classes / 40 tests (292 total, 0 failures); `simt-app` `:feature:gnss:testDebugUnitTest`
  3 classes / 8 tests green.
- **Publishing & CI** — the library publishes to GitHub Packages
  (`https://maven.pkg.github.com/QuazarMedia/simt-gnss`) as `app.simt:gnss-core` and
  `app.simt:gnss-android`, versioned by `VERSION_NAME` (default `0.1.0`), each with a
  `-sources.jar`. Both modules opt into `maven-publish` + `signing` (signing only when
  `-PsigningKeyId/-PsigningKey/-PsigningPassword` are supplied); coordinates, POM metadata and
  the target repository are declared once in the root build file so the two artifacts cannot
  drift. `.github/workflows/build.yml` runs `./gradlew build` on `main` and on pull requests;
  `.github/workflows/publish.yml` publishes on `v*` tags (or by manual dispatch) and derives the
  version from the tag. The estate's generated `simt-dev-check.yml` was deliberately not added:
  it installs Node dependencies and expects a `simt-dev` binary, and this repo is a Node-less
  Gradle project whose `simt.json` declares no gates. Evidence: `./gradlew publishToMavenLocal`
  green, writing `app/simt/gnss-core/0.1.0/{.jar,-sources.jar,.pom,.module}` and
  `app/simt/gnss-android/0.1.0/{.aar,-sources.jar,.pom,.module}`; the AAR POM depends on
  `app.simt:gnss-core:0.1.0`; `./gradlew build` green including `:gnss-android:lintDebug`.
- **Skill split** — `gnss-engine` now lives here (`.cursor/skills/gnss-engine/SKILL.md`, plus the
  `.claude/skills` symlink the estate uses) and documents the library's modules, neutral seams,
  public API, parser convention and invariants; `simt-app` carries a consumer-oriented
  `gnss-integration` skill instead, covering the composite-build boundary, the NTRIP/location
  adapters and the diagnostics mapping. Both are registered in
  `simt-dev/graph/skills.graph.json`. Evidence: `node bin/simt-dev.mjs graph lint` → `OK no
  findings` (38 skills, byte counts verified); `node --test test/*.test.mjs` → 1097 passing.
- **Consumer proguard** — `simt-app/feature/gnss` gained the `proguard-rules.pro` its release
  build already referenced; the engine's own rules travel with `app.simt:gnss-android`.
