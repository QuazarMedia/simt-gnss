# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project
adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.2.0] - 2026-10-19

### Added

- **`:gnss-core`** — `BroadcastEphemerisProvider.parseRinex3MixedNav` overloads that take a
  `BufferedReader` or a `Sequence<String>` and parse line-by-line, holding only the current
  record in memory. The previous `String` entry point built the whole file (and, via
  `String.lines()`, a second list of every line) in one parse; a full day's broadcast navigation
  file is several megabytes and OOM'd A-GNSS on a heap-constrained device.

## [0.1.0] - 2026-10-18

First public release.

### Added

- **`:gnss-core`** — pure-JVM GNSS algorithms:
  - Broadcast navigation-message parsers: GPS LNAV (L1 C/A), GPS CNAV MT30 (L2C/L5 ISC),
    Galileo I/NAV, BeiDou D1 (BDT), GLONASS L1OF, SBAS (types 1, 2–5, 9, 24–25), plus LNAV D30
    polarity validation.
  - RTCM 3 support: CRC-24Q frame/stream parsing, MSM4–MSM7 observations, message 1005 base
    ECEF, SSR orbit/clock/code-bias/phase-bias decoding.
  - Orbit and clock computation: Keplerian propagation (GPS, Galileo, BeiDou, QZSS), GLONASS RK4
    integration, SBAS GEO, elevation/azimuth, Earth-rotation (Sagnac) correction.
  - Atmospheric models: Klobuchar ionosphere, Saastamoinen + Niell troposphere, and the
    ionosphere-free L1/L5 combination.
  - Positioning: iterated weighted least squares with Householder-QR (high accuracy) and
    Cholesky (medium) strategies, per-constellation clock states (ISB), RAIM outlier rejection,
    covariance and DOP output, Doppler velocity and TDCP displacement solutions.
  - Relative positioning: RTK double-difference (float) with LAMBDA integer ambiguity resolution
    and cross-epoch ambiguity memory.
  - Correction stores: IODE-matched, staleness-dropping SSR store, and an RTK base-observation
    store.
  - Filtering: Hatch carrier smoothing with cycle-slip detection, Doppler smoother fallback, and
    a 6-state inter-epoch Kalman filter with time-gap, innovation and mode-change resets.
  - Raw-GNSS constellation-exclusion fallback ladder with a configurable time budget.
  - RINEX 3 (mixed) broadcast-ephemeris import and external ephemeris injection.
  - MSB-first bit-access utilities, week/day time constants, and the neutral model types.
- **`:gnss-android`** — the `GnssMeasurementProcessor` orchestrator and the public
  `SimtGnssEngine` façade over Android's `android.location` types, RTCM stream ingestion,
  smoothing-mode selection, a neutral diagnostics snapshot, and the pluggable platform seams
  (clock and logger).
- Neutral `Constellation` / `AdrState` constants, so `:gnss-core` carries zero `android.*`
  imports.
- Published to Maven Central as `app.simt:gnss-core` and `app.simt:gnss-android`, with sources
  and javadoc artifacts, signed.
- CI: a build/test workflow and a tag-driven Maven Central release workflow that creates the
  matching GitHub release once the upload succeeds.

[Unreleased]: https://github.com/QuazarMedia/simt-gnss/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/QuazarMedia/simt-gnss/releases/tag/v0.1.0
