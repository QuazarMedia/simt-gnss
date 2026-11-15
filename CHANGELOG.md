# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project
adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.3.0] - 2026-11-15

Six-slice remediation program for engine accuracy and fix reliability (see
`docs/plans/2026-11-gnss-engine-accuracy-remediation.md`), plus AGP/Spotless version
alignment.

### Added

- **`:gnss-android` / `:gnss-core`** — `SimtGnssEngine.resetSession(preserveEphemeris, clearCorrections)`
  to reset transient session filter/smoother/TDCP/RTK-ambiguity/RTCM-parser state between
  capture sessions while optionally preserving broadcast ephemerides and Klobuchar parameters.
- **`:gnss-android` / `:gnss-core`** — `commitState` parameter on `SimtGnssEngine.computePosition`
  (default `true`), `SimtGnssEngine.hasActivePppOrSsrState(...)`, and `hasPppOrSsrState` on
  `RawGnssPositionFallbackResolver.resolve(...)` so fallback callers can probe candidate subsets
  without mutating session filter state and skip duplicate `usePpp = true` solves when no PPP
  seed or SSR corrections are active.
- **`:gnss-core` / `:gnss-android`** — `GnssInterEpochKalmanFilter.syncToExternalFix(...)` and
  Doppler velocity/bearing output (`speedMetersPerSecond`, `bearingDegrees`) on RTK fixes.

### Fixed

- **`:gnss-core` / `:gnss-android`** — Upgraded the inter-epoch Kalman filter to a 9D
  delayed-state (`[r_k, v_k, r_{k-1}]`) formulation fusing TDCP displacements and full 3×3
  Doppler velocity covariance, with per-signal Hatch/Doppler smoother clock-step tracking
  (`Long` `fullBiasNanos`), Hatch gap resets, and smoothing-aware code sigmas.
- **`:gnss-android` / `:gnss-core`** — Explicit multi-band ionosphere-free pairing and sigma
  scaling (preventing E5a+E5b noise amplification), BeiDou `TGD1`/`TGD2` group delays and BDS-3
  clock-group splitting, single-SV constellation pruning, and cold-start coarse-seed WLS
  convergence.
- **`:gnss-android` / `:gnss-core`** — Pilot `STATE_TOW_KNOWN` / `STATE_MSEC_AMBIGUOUS` gating,
  `GPS_EPOCH_UNIX_SECONDS` fallback conversion, GLONASS L1OF string `tb` frame coherence and
  `l_n` health checks, and RINEX 3 line-0 `toc` + full-epoch week-age bounds.
- **`:gnss-core` / `:gnss-android`** — SSR code biases (`1059`/`1065`/`1242`/`1260`), split SSR
  (`1058`/`1062`/etc.) and IGS `4076` decoding, and BeiDou/GLONASS SSR IOD and polynomial-age
  handling.
- **`:gnss-core` / `:gnss-android`** — RTK 50 km baseline cap, differential troposphere and
  (> 10 km) differential Klobuchar ionosphere, float-covariance ambiguity prior gating and DD
  reference-pivot transform, MSM lock-time cycle-slip detection, `referenceStationId` coherence,
  and RTK-to-absolute Kalman/TDCP handover.

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

[Unreleased]: https://github.com/QuazarMedia/simt-gnss/compare/v0.2.0...HEAD
[0.3.0]: https://github.com/QuazarMedia/simt-gnss/releases/tag/v0.3.0
[0.1.0]: https://github.com/QuazarMedia/simt-gnss/releases/tag/v0.1.0
