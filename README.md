# simt-gnss

A standalone, app-agnostic GNSS processing engine for Android, packaged as a reusable
Kotlin library. It turns raw Android GNSS observations into positions: broadcast-ephemeris
decoding, satellite orbit/clock computation, atmospheric and RTK/PPP corrections, carrier
smoothing, and weighted least-squares positioning.

The library is deliberately independent of any host application. It contains **no**
application-specific code — no Hilt, no Compose, no Play Services, no host packages. Any
Android app can consume it.

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

Releases are published to Maven Central as `app.simt:gnss-core` and `app.simt:gnss-android`.
Until the first release is verified there, consume it as a **composite build**. In the
consumer's `settings.gradle.kts`:

```kotlin
includeBuild("../simt-gnss") {
    dependencySubstitution {
        substitute(module("app.simt:gnss-core")).using(project(":gnss-core"))
        substitute(module("app.simt:gnss-android")).using(project(":gnss-android"))
    }
}
```

then depend on it:

```kotlin
dependencies {
    implementation("app.simt:gnss-android") // brings :gnss-core transitively
}
```

For plain-JVM post-processing, depend on `app.simt:gnss-core` alone: its parsers, solvers,
filters and models are Android-free. `:gnss-android` installs the `SystemClock`/`Log` platform
seams automatically on construction.

## Building

```sh
./gradlew build          # compile + unit tests
./gradlew :gnss-core:test :gnss-android:test
./gradlew spotlessApply  # format
```

`minSdk` 28, `compileSdk` 37, JVM toolchain 21.

## Releasing

Releases go to Maven Central from CI via the Central Portal Publisher API
(`com.gradleup.nmcp`). `.github/workflows/publish.yml` runs on a `v*` tag — or by hand through
`workflow_dispatch` with an explicit version — and uploads both modules.

Required repository secrets:

| Secret | Value |
|---|---|
| `CENTRAL_PORTAL_USERNAME`, `CENTRAL_PORTAL_PASSWORD` | A Central Portal user token (central.sonatype.com → Account → Generate User Token) |
| `SIGNING_KEY`, `SIGNING_PASSWORD` | An ASCII-armored GPG private key and its passphrase — Central rejects unsigned artifacts |

The `app.simt` group needs a verified `simt.app` namespace on the Portal: add the TXT record
carrying the namespace's verification key to the **exact** domain `simt.app` (not a
subdomain), and only then press "Verify Namespace".

`publishingType` is `USER_MANAGED`, so a release finishes with a manual confirmation in the
Portal UI; switch it to `AUTOMATIC` in `settings.gradle.kts` to publish as soon as the upload
passes validation. `./gradlew publishToMavenLocal` exercises the same publications locally.

## License

Apache-2.0. See [LICENSE](LICENSE).
