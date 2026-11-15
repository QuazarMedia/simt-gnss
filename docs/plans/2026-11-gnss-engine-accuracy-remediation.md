# GNSS Engine Accuracy & Fix-Reliability Remediation (`simt-gnss`)

**Review date:** 2026-11-15
**Review verdict:** **BLOCK** (5 Critical, 7 High findings across `app.simt:gnss-core` and `app.simt:gnss-android`)

## Target invariants

1. **Unbiased delayed-state TDCP & Kalman fusion:** TDCP relative carrier displacements (`r_k - r_{k-1}`) update a 9-state `[r_k, v_k, r_{k-1}]` filter through `H = [I_3, 0_3, -I_3]` with full `3×3` covariance — never injecting `anchor + Δr` as an absolute `5 cm` measurement that freezes Epoch-1 WLS error or collapses `P_pos`.
2. **Per-signal clock-step continuity & calibrated code noise:** `HatchFilter` and `DopplerSmoother` store `lastFullBiasNanos` / `lastBiasNanos` per `signalKey` (never dropping multi-epoch clock steps across brief satellite occlusions), reset cleanly on `dt > 5 s`, and propagate smoothing age `k` into realistic smartphone observation sigmas.
3. **Physically sound multi-band IF, group delays & WLS geometry:** Band classification distinguishes `L1/E1/B1C`, `B1I`, `L2`, `E5b/B2I`, `B3I`, and `L5/E5a/B2a` (never pairing `E5b+E5a` or `B2I+B2a` at `27.5×` noise amplification), scales IF `sigmaModel` by the IF factor, models B3I-referenced BeiDou `TGD1`/`TGD2` per tracked band, prunes zero-DOF single-SV constellations before WLS/RAIM, and converges cleanly from `(0, 0, 0)` cold seeds.
4. **ICD-faithful state gates, GLONASS strings & RINEX import:** `STATE_TOW_KNOWN` works on API 28–29 (with code-lock required and `STATE_MSEC_AMBIGUOUS` rejected), `gpsTimeNanos` fallback subtracts `GPS_EPOCH_UNIX_SECONDS`, GLONASS strings 1–4 enforce same-`tb` frame coherence and `l_n == 0` health, and RINEX 3 parses true line-0 `toc`, 14 s BDT age offsets, and full calendar/week bounds.
5. **Complete SSR & atmospheric/multi-epoch RTK:** SSR applies decoded signal code biases (`1059`/`1065`/`1242`/`1260`) without stacking broadcast `TGD`/`BGD`, pairs IOD-matched orbit+clock atomically (including split SSR, IGS `4076`, GLONASS `DF386` TOD, and BeiDou `AODE` + BDT `dtSsr`), and RTK caps baseline length (`50 km`), models differential troposphere/ionosphere, gates priors with unconstrained float covariance, pivot-transforms DD ambiguities on reference swaps, decodes MSM lock-time slips (`DF402`/`DF407`), and enforces `referenceStationId` coherence.
6. **Clean session lifecycle façade:** `SimtGnssEngine.resetSession(preserveEphemeris = true)` resets transient filters/stores between standalone captures while keeping fresh OTA/A-GNSS ephemerides, and RTK updates/resets Kalman + TDCP state seamlessly.

## Execution waves

| wave | slices |
|------|--------|
| 1 | 1, 3 |
| 2 | 2, 4, 5 |
| 3 | 6 |

## Slice tracker

| id | tier | lane | depends | title | state | evidence |
|----|------|------|---------|-------|-------|----------|
| 1 | L | kalman-smoothing | - | Delayed-state 9D TDCP-Kalman fusion, per-signal smoother clock steps, Hatch gap reset & smoothing-aware code sigma | Complete | 5220bfe69a6aa027d1e34bc40aa5a52207ca029a · test: ./gradlew :gnss-core:test :gnss-android:test |
| 2 | L | signal-wls | 1 | Explicit multi-band IF pairing & sigma scaling, BeiDou `TGD1`/`TGD2` group delays, single-SV constellation pruning & cold-start WLS convergence | Complete | 79833af50fe386a91f7eabdfec78d6d546092e98 · test: ./gradlew :gnss-core:test :gnss-android:test |
| 3 | M | nav-timescale | - | Pilot `STATE_TOW_KNOWN` / `STATE_MSEC_AMBIGUOUS` gating, `GPS_EPOCH_UNIX_SECONDS` fallback, GLONASS frame/`l_n` validation & RINEX `toc`/week-age fixes | Queued | |
| 4 | L | ssr-ppp | 2, 3 | Apply SSR code biases (`1059`/`1065`/`1242`/`1260`), fix BeiDou/GLONASS SSR IOD & BDT timing, and decode split SSR + IGS `4076` | Queued | |
| 5 | L | rtk-core | 1 | RTK 50 km baseline cap, differential tropo/iono, float-covariance prior gating, DD pivot transform, MSM lock-time slips & `referenceStationId` | Queued | |
| 6 | M | engine-facade | 1, 2, 3, 4, 5 | Add `SimtGnssEngine.resetSession()`, sync RTK/Kalman transitions & RTK Doppler velocity/bearing output | Queued | |

---

## Slice 1 — Delayed-state TDCP-Kalman fusion & per-signal smoother continuity

- **Modules:** `:gnss-core`, `:gnss-android`
- **Files:** `GnssInterEpochKalmanFilter.kt`, `TdcpDisplacementSolver.kt`, `DopplerVelocitySolver.kt`, `HatchFilter.kt`, `DopplerSmoother.kt`, `ReceiverClockStep.kt`, `GnssModels.kt`, `GnssMathUtils.kt`, `GnssMeasurementProcessor.kt`
- **Changes:**
  - Expand `GnssInterEpochKalmanFilter` to 9 states `[r_k, v_k, r_{k-1}]` (`9×9` `P`, `F_aug` copying prior `r_{k-1}` before `predict(dt)` so cross-covariance `Cov(r_k, r_{k-1})` is exact) and fuse TDCP as relative displacement `z = Δr_k` with `H = [I_3, 0_3, -I_3]` and full `3×3` `R_tdcp` (removing `anchor + Δr` absolute updates that froze Epoch-1 WLS bias and collapsed `P_pos` to `0.0025 m²`).
  - Skip `applyDisplacementMeasurement` and `predict`-less updates when an epoch is gated/coasted or `wlsEcef` is non-finite so `coastReport()` matches the published state.
  - Add leverage-normalized residual screening `|r_i| / (σ_i √(1 - h_ii))` and dedicated displacement variance floor (`1e-4 m²`) in `TdcpDisplacementSolver`; move `seen.add(key)` after frequency matching in `GnssMeasurementProcessor.tdcpDisplacement`.
  - Store `lastFullBiasNanos` and `lastBiasNanos` **per `signalKey`** inside `HatchFilter` and `DopplerSmoother` so 1–4 epoch signal occlusions subtract the cumulative clock step `C(t_now) - C(t_last_seen)`; reset `HatchFilter` when `prevAdr != null && dtSeconds == null` (`dt > 5 s`).
  - Record Hatch smoothing count `smoothingEpochs` on `SatelliteMeasurement` and calibrate raw smartphone code noise (`a_raw`, `b_raw` tapering with `1/√k` to smoothed floor) so unsmoothed `k = 1` epochs do not claim `0.3 m` geodetic formal accuracy.
- **Acceptance checks:**
  - `./gradlew :gnss-core:test --tests app.simt.gnss.core.filter.GnssInterEpochKalmanFilterTest` (asserting a `+10 m` Epoch-1 WLS error with 1 mm TDCP displacements converges to `0 m` over subsequent unbiased WLS epochs without tripping the 4σ Mahalanobis gate).
  - `./gradlew :gnss-core:test --tests app.simt.gnss.core.filter.HatchFilterTest` (2-epoch SV dropout across a 100 m/epoch receiver clock refit injects `0 m` differential bias; `dt = 6 s` resets `smoothingCount`).
  - `./gradlew :gnss-core:test --tests app.simt.gnss.core.solver.TdcpDisplacementSolverTest`

## Slice 2 — Multi-band IF pairing, BeiDou `TGD1`/`TGD2`, single-SV pruning & cold-start WLS

- **Modules:** `:gnss-core`, `:gnss-android`
- **Files:** `GnssMeasurementProcessor.kt`, `GnssMathUtils.kt`, `GnssModels.kt`, `BeidouNavMessageParser.kt`, `BroadcastEphemerisProvider.kt`, `GnssPositionSolverCholesky.kt`
- **Changes:**
  - Replace `isL5 = carrierFrequencyHz < 1.2e9` with explicit `GnssSignalBand` (`L1_E1_B1C`, `B1I`, `L2`, `E5B_B2I`, `B3I`, `L5_E5A_B2A`) and guard `!hasCarrierFrequencyHz() || carrierFrequencyHz <= 0f` with constellation primary-band defaults.
  - Form dual-frequency IF only between an upper-L band (`>= 1.55 GHz`) and a wide-separated lower-L band (`<= 1.28 GHz`), preferring `L1/L5`, `E1/E5a`, `B1C/B2a`, `B1I/B2a`, and falling back to `E1/E5b` or `B1I/B2I` — never pairing `E5b + E5a` or `B2I + B2a` (`27.5×` noise blowup).
  - Propagate per-band `sigmaModel(el, cn0, smoothingEpochs)` through the IF variance formula \(\sigma_{\text{IF}} = \sqrt{f_1^4 \sigma_1^2 + f_2^4 \sigma_2^2} / (f_1^2 - f_2^2)\).
  - Parse `TGD2` in `BeidouNavMessageParser` (D1 SF1) and `BroadcastEphemerisProvider` (RINEX `l6` cols 61–80); apply exact B3I-referenced group delays (`TGD1` on `B1I`, `TGD2` on `B2I`, IF linear combination on `B1I/B2I`, and `0.0` rather than `1.0 · TGD1` on `B1C/B2a`, with separate clock state when uncalibrated `B1I` and `B1C/B2a` are mixed).
  - Iteratively prune constellations with `< 2` usable SVs before WLS (and re-prune after each RAIM removal); gate RAIM on `validRows > m` (`dof >= 1`); add `if (n < m) return null` to `GnssPositionSolverCholesky`.
  - Use isotropic `45°` elevation weighting on early WLS iterations while `posDelta > 100_000.0` (or Earth-center/antipodal cold seeds), raise `MAX_WLS_ITERATIONS` to `12`, snapshot usable `SatelliteEphemeris` once per solve, and expire `PppState` when `gpsTimeNanos - lastTimeNanos > 30_000_000_000L` or far from non-zero `initialLat/Lon`.
- **Acceptance checks:**
  - `./gradlew :gnss-android:test --tests app.simt.gnss.GnssGroupDelayConventionTest`
  - `./gradlew :gnss-android:test --tests app.simt.gnss.GnssMeasurementProcessorTest` (antipodal `(0,0,0)` cold-start convergence, `E5b+E5a` rejection, `2 GPS + 1 GAL + 1 BDS + 1 GLO` graceful pruning vs `4 GPS + 1 GLO` unbiased `residualsRms`/RAIM).
  - `./gradlew :gnss-core:test --tests app.simt.gnss.core.parser.BeidouNavMessageParserTest`

## Slice 3 — Measurement state gates, GLONASS epoch/frame integrity & RINEX `toc`/week bounds

- **Modules:** `:gnss-core`, `:gnss-android`
- **Files:** `GnssMeasurementProcessor.kt`, `GlonassNavMessageParser.kt`, `BroadcastEphemerisProvider.kt`
- **Changes:**
  - Remove `Build.VERSION.SDK_INT >= Build.VERSION_CODES.R` on `STATE_TOW_KNOWN`; reject `STATE_MSEC_AMBIGUOUS`; require `STATE_CODE_LOCK` (or `STATE_GAL_E1BC_CODE_LOCK` for Galileo) when `STATE_TOW_KNOWN` is set without `STATE_TOW_DECODED`.
  - Subtract `GnssMathUtils.GPS_EPOCH_UNIX_SECONDS` in `GnssMeasurementProcessor.kt:1107` fallback, compute system-clock GPS TOW on demand when `lastGpsTimeOfWeekSeconds` is uninitialized in `processNavigationMessage`, and preserve `lastKnownLeapSeconds` across epochs where `clock.hasLeapSecond()` is false.
  - Store receive timestamp and String-2 `tb` per SV in `glonassStringStorage` so strings 1–4 from different 30-minute `tb` frames or >30 s apart are never stitched; check String 3 `l_n` health (`getBits(str3, 20, 1) == 0`).
  - Parse RINEX 3 line-0 `YYYY MM DD HH MM SS` into constellation `toc` for `G`/`J`/`E`/`C` records in `BroadcastEphemerisProvider`; subtract `BDT_GPST_OFFSET_SECONDS` (14 s) for BeiDou in `ageSeconds` and `importExternalEphemeris`; validate full epoch Unix/GPS timestamp against optional `currentUnixSeconds` so 7-day-old RINEX files cannot wrap modulo `604,800 s` to `ageSeconds ≈ 0`.
- **Acceptance checks:**
  - `./gradlew :gnss-android:test --tests app.simt.gnss.GnssMeasurementProcessorRawEpochTest`
  - `./gradlew :gnss-core:test --tests app.simt.gnss.core.parser.GlonassNavMessageParserTest`
  - `./gradlew :gnss-core:test --tests app.simt.gnss.core.agnss.BroadcastEphemerisProviderTest`

## Slice 4 — SSR code biases, BeiDou/GLONASS SSR gating & split/IGS-4076 decoding

- **Modules:** `:gnss-core`, `:gnss-android`
- **Files:** `RtcmSsrDecoder.kt`, `SsrCorrections.kt`, `GnssMeasurementProcessor.kt`, `BeidouNavMessageParser.kt`, `BroadcastEphemerisProvider.kt`
- **Changes:**
  - Apply `ssrStore.getCodeBias(constellation, svid, signalId)` per tracked band / IF combination when `ssrClock` is active, and skip broadcast `TGD`/`BGD`/`ISC` when SSR clock + code biases are applied.
  - Require **both** IOD-matched `ssrOrbit` and `ssrClock` before applying SSR to an SV, and use SSR orbit+clock sigma (`~0.2 m`) instead of `broadcastEphemerisRangeSigmaMeters(ephemerisAgeSeconds)` when valid SSR corrections are applied.
  - Pass BDT `currentTimeSeconds` (not `gpstSeconds`) to `ssrPolynomialAgeSeconds` for BeiDou; parse BeiDou `AODE` in `BeidouNavMessageParser` and `BroadcastEphemerisProvider` and match BeiDou SSR `iod` / `(toe / 8) % 240` instead of `-1`.
  - Convert GLONASS `DF386` UTC(SU) TOD to GPST TOW (with `lastKnownLeapSeconds` and half-day wrap) and enable GLONASS SSR.
  - Decode separate orbit (`1057`, `1063`, `1240`, `1258`), separate clock (`1058`, `1064`, `1241`, `1259`), high-rate clock (`1062`, `1068`, `1245`, `1263`), QZSS (`1246–1251`), and IGS SSR `4076` frames in `RtcmSsrDecoder`.
- **Acceptance checks:**
  - `./gradlew :gnss-core:test --tests app.simt.gnss.core.parser.RtcmSsrDecoderTest`
  - `./gradlew :gnss-android:test --tests app.simt.gnss.GnssMeasurementProcessorTest` (SSR code-bias application, BeiDou BDT `dtSsr` + IOD mismatch rejection, offline 6 h RINEX + live SSR tight weighting).

## Slice 5 — RTK differential atmosphere, baseline cap, prior gating, DD pivot & MSM coherence

- **Modules:** `:gnss-core`, `:gnss-android`
- **Files:** `RtkDoubleDifferenceSolver.kt`, `RtkAmbiguityMemory.kt`, `RtcmMsmParser.kt`, `RtkBaseStore.kt`, `GnssMeasurementProcessor.kt`
- **Changes:**
  - Reject RTK when baseline `||baseEcef - seedRover||` (or solved baseline) exceeds `MAX_RTK_BASELINE_METERS = 50_000.0`, falling back cleanly to `computePosition`.
  - Compute geometric elevation at `baseEcef` and `roverEcef` and subtract Saastamoinen + Niell tropospheric delay (plus differential Klobuchar `-iono` on code, `+iono` on carrier for baselines `> 10 km`) at both base and rover.
  - Gate carried ambiguity priors in `solveFixed` using `4.0 * sqrt(unconstrained.covariance[i][i] + prior.varianceCycles2)` instead of fixed `0.5` cycles against noisy single-epoch code float; call `onFloat` **after** partial-fix outlier subsetting; scale `MAX_FIX_FLOAT_SEPARATION_METERS` by float covariance (`max(1.5, 4.0 * σ_float_3D)`); avoid 0-DOF `0.02 m` `sigma0` floor on 4-SV code floats.
  - Add DD reference-satellite pivot transformation (`N_{k,newRef} = N_{k,oldRef} - N_{newRef,oldRef}` + covariance transform + `3°` reference hysteresis) in `RtkAmbiguityMemory` / `RtkDoubleDifferenceSolver`.
  - Decode MSM lock-time indicator (`DF402`/`DF407`) in `RtcmMsmParser` to drop priors on base cycle slips; reject GLONASS `DF419 > 13` and guard `Constellation.GLONASS` out of `solveFixed`; key `rtkCarrierSamples` by `(constellation, svid, bandMhz)` with consistent primary-band selection; wrap `baseEpochSeconds - currentTimeSeconds` with `wrapToHalfWeekSeconds`; enforce `referenceStationId` matching between `Rtcm1005` and `RtcmMsm` in `RtkBaseStore`.
- **Acceptance checks:**
  - `./gradlew :gnss-core:test --tests app.simt.gnss.core.solver.RtkDoubleDifferenceSolverTest`
  - `./gradlew :gnss-core:test --tests app.simt.gnss.core.solver.RtkAmbiguityMemoryTest`
  - `./gradlew :gnss-core:test --tests app.simt.gnss.core.correction.RtkBaseStoreTest`
  - `./gradlew :gnss-core:test --tests app.simt.gnss.core.parser.RtcmMsmParserTest`

## Slice 6 — `SimtGnssEngine` session lifecycle API & RTK/Kalman unification

- **Modules:** `:gnss-android`, `:gnss-core`
- **Files:** `SimtGnssEngine.kt`, `GnssMeasurementProcessor.kt`, `CHANGELOG.md`
- **Changes:**
  - Add `SimtGnssEngine.resetSession(preserveEphemeris: Boolean = true, clearCorrections: Boolean = true)` clearing `pppState`, `interEpochKalman`, `hatchFilter`, `dopplerSmoother`, `tdcpAnchor`/`tdcpSignals`, `rtkAmbiguityMemory`, and `lastFullBiasNanos` (plus `ssrStore`/`rtkBaseStore` when `clearCorrections = true`) while preserving valid ephemerides and Klobuchar parameters.
  - Gate `getApplicableSsrOrbit`/`getApplicableSsrClock` and stateful `interEpochKalman`/`pppState` updates cleanly across `RawGnssPositionFallbackResolver` probe attempts so rejected fallback subsets cannot pollute session filters.
  - Update/synchronize `interEpochKalman` and `tdcpAnchor` when `computeRtkPosition` succeeds and attach Doppler `speedMetersPerSecond` / `bearingDegrees` to RTK `PositionResult`.
- **Acceptance checks:**
  - `./gradlew :gnss-android:test --tests app.simt.gnss.SimtGnssEngineTest`
  - `./gradlew :gnss-core:test :gnss-android:test spotlessCheck`
