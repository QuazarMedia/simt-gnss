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
| `:gnss-android` | Android library | The engine orchestrator (`GnssMeasurementProcessor`) and the public `GnssEngine` façade that binds it to `android.location` types (`GnssMeasurementsEvent`, `GnssNavigationMessage`, `GnssStatus`). |

`:gnss-android` depends on `:gnss-core`; `:gnss-core` depends on nothing but the Kotlin
standard library.

## Using it

```kotlin
// from :gnss-android
val engine = GnssEngine(suppressSbas = false)
engine.onNavigationMessage(navMessage)                 // feed raw nav messages
val epoch = engine.onMeasurements(event, gnssStatus)   // feed raw measurements
val fix: PositionResult? = engine.solve(
    initialLat = lat, initialLon = lon, initialAlt = alt, gpsTimeNanos = epoch.gpsTimeNanos,
)
```

All results use neutral model types (`PositionResult`, `SatelliteMeasurement`,
`SatelliteEphemeris`, `GnssSolutionMode`) declared in `:gnss-core`.

## Building

```sh
./gradlew build          # compile + unit tests
./gradlew :gnss-core:test :gnss-android:test
./gradlew spotlessApply  # format
```

`minSdk` 28, `compileSdk` 37, JVM toolchain 21.

## License

Apache-2.0. See [LICENSE](LICENSE).
