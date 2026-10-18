# simt-gnss

A standalone, app-agnostic GNSS processing engine for Android, packaged as a reusable Kotlin
library. It turns raw Android GNSS observations into positions — broadcast-ephemeris decoding,
satellite orbit and clock computation, atmospheric and RTK/PPP corrections, carrier smoothing,
and weighted least-squares positioning.

The library is deliberately independent of any host application. It contains **no**
application-specific code: no dependency-injection framework, no UI toolkit, no Play Services,
no networking. Any Android app can consume it, and the algorithm layer runs on plain JVM.

## Features

**Signal decoding**

- Broadcast navigation messages: GPS LNAV (L1 C/A), GPS CNAV MT30 (L2C/L5 ISC), Galileo I/NAV,
  BeiDou D1 (BDT time base), GLONASS L1OF, SBAS (types 1, 2–5, 9, 24–25).
- Nav-message validation, including the LNAV D30 polarity inversion.
- RTCM 3: CRC-24Q frame/stream parsing, MSM4–MSM7 observation messages, message 1005 base-station
  ECEF, and SSR orbit/clock/code-bias/phase-bias decoding.

**Modelling**

- Orbit and clock propagation: Keplerian elements (GPS, Galileo, BeiDou, QZSS), GLONASS RK4
  numerical integration, SBAS GEO, plus elevation/azimuth and Earth-rotation (Sagnac) terms.
- Ionosphere: Klobuchar model, plus the ionosphere-free L1/L5 combination for dual-frequency
  receivers.
- Troposphere: Saastamoinen with the Niell mapping function.
- Broadcast-ephemeris import from RINEX 3 (mixed) files, and external ephemeris injection.

**Positioning**

- Iterated weighted least squares with two matrix strategies — Householder QR (high accuracy) and
  Cholesky (medium) — with per-constellation clock states (inter-system bias) and RAIM outlier
  rejection.
- Covariance and DOP output (`gdop`, `pdop`, `hdop`, `vdop`, `tdop`), residual RMS and fix metadata.
- Doppler velocity solution and TDCP (time-differenced carrier phase) displacement solution.
- Relative positioning: RTK double-difference (float) with LAMBDA integer ambiguity resolution and
  ambiguity memory across epochs.
- Correction application: SSR orbit/clock/bias (applied only when the correction IOD matches the
  broadcast ephemeris IODE, dropped when stale) and an RTK base-observation store.
- Activity smoothing: Hatch carrier smoothing with cycle-slip detection, a Doppler smoother as
  fallback, and a 6-state inter-epoch Kalman filter with resets on time gaps, innovation outliers
  and mode changes.
- Raw-GNSS fallback ladder that progressively excludes constellations when a fix cannot be found,
  under a configurable time budget.

**Engineering**

- `:gnss-core` is pure JVM: zero `android.*` imports. Android concepts arrive through neutral
  seams — pluggable clock and logger (`GnssPlatform`) and neutral `Constellation` / `AdrState`
  constants.
- Results are neutral model types, so the engine can be driven from any source of observations.

Not included by design: NTRIP/network clients, UI, persistence and dependency injection. The
library processes what you feed it and returns a solution; the host application owns transports
and acceptance policy.

## Modules

| Module | Kind | Contents |
|---|---|---|
| `:gnss-core` | pure Kotlin/JVM | The algorithms: nav-message and RTCM parsers, orbit/clock computation, solvers (QR, Cholesky, Doppler, RTK double-difference, TDCP), filters (Hatch, Doppler, inter-epoch Kalman), correction models and stores, ephemeris (RINEX) provider, math utilities and the shared data models. No Android dependency — usable for desktop/server post-processing. |
| `:gnss-android` | Android library | The engine orchestrator (`GnssMeasurementProcessor`) and the public `SimtGnssEngine` façade that binds it to `android.location` types (`GnssMeasurementsEvent`, `GnssNavigationMessage`, `GnssStatus`). |

`:gnss-android` depends on `:gnss-core`; `:gnss-core` depends on nothing but the Kotlin
standard library.

## Using it

```kotlin
// from :gnss-android
val engine = SimtGnssEngine(suppressSbas = false)

engine.processNavigationMessage(navMessage)                    // feed raw nav messages
val epoch = engine.processMeasurements(event, gnssStatus)       // feed raw measurements
engine.ingestRtcmFrame(rtcmBytes)                               // optional: feed an NTRIP/RTCM stream

val fix: PositionResult? = engine.computePosition(
    measurements = epoch.measurements,
    initialLat = lat, initialLon = lon, initialAlt = alt,
    gpsTimeNanos = epoch.gpsTimeNanos,
)
// or, against the base station in engine.rtkBaseStore:
val rtk: PositionResult? = engine.computeRtkPosition(epoch.measurements, epoch.gpsTimeNanos)
```

All results use neutral model types (`PositionResult`, `SatelliteMeasurement`,
`SatelliteEphemeris`, `GnssSolutionMode`, `ProcessedMeasurements`, `SimtGnssEngineDiagnostics`)
declared in `:gnss-core` / `:gnss-android`. `engine.diagnostics()` returns a neutral snapshot
for logging or a diagnostics panel.

## Consuming the library

Releases are published to Maven Central as `app.simt:gnss-core` and `app.simt:gnss-android`:

```kotlin
dependencies {
    implementation("app.simt:gnss-android:0.1.0") // brings :gnss-core transitively
}
```

For plain-JVM post-processing, depend on `app.simt:gnss-core` alone: its parsers, solvers,
filters and models are Android-free. `:gnss-android` installs the `SystemClock`/`Log` platform
seams automatically on construction.

To develop against a local checkout instead, substitute the published modules in the consumer's
`settings.gradle.kts`:

```kotlin
includeBuild("../simt-gnss") {
    dependencySubstitution {
        substitute(module("app.simt:gnss-core")).using(project(":gnss-core"))
        substitute(module("app.simt:gnss-android")).using(project(":gnss-android"))
    }
}
```

## Building

```sh
./gradlew build          # compile + unit tests
./gradlew :gnss-core:test :gnss-android:test
./gradlew spotlessApply  # format
```

`minSdk` 28, `compileSdk` 37, JVM toolchain 21.

## Releasing

Releases are published to Maven Central from CI: pushing a version tag (for example `v0.2.0`)
runs the release workflow, which uploads both modules and derives the version from the tag.
Every artifact is signed and ships with its sources and javadoc.

Release notes live in [CHANGELOG.md](CHANGELOG.md). Once the upload succeeds, the same workflow
creates the GitHub release for the tag, using this changelog's section for that version as its
body. To exercise the publications locally without publishing, run `./gradlew publishToMavenLocal`.

## License

Apache-2.0. See [LICENSE](LICENSE).
