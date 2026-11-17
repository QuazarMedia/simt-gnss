# Galileo gains, calculations and acceptance strategy

Parent: [scope, canonical tracker and source register](2026-11-galileo-capabilities.md). This is a **prospective study**, not measured SIMT performance. Numeric examples below are labelled assumptions; service targets are not local benchmarks.

## 1. What counts as a gain

Measure five different outcomes:

- **Honest accuracy:** horizontal/vertical error against independent truth, bias, RMSE, P50/P68/P95 and covariance calibration.
- **Ordinary TTFF:** time from a defined receiver/session start to the first valid code-position fix. This is not carrier acquisition time.
- **Time to accuracy:** time to a predeclared error threshold sustained for a declared interval, including reconvergence after slips/outages.
- **Authentication latency:** first verified navigation issue (TTFA), first policy-eligible navigation-authenticated fix (TTFAF), and first SAS-covered capture/result. Keep their scopes distinct.
- **Resilience/availability/cost:** false accepts/dismisses, honest fixes withheld, outages, energy-to-fix, CPU, callback delay and retained memory.

A feature can improve security while worsening availability or latency. An acquisition improvement may not affect the critical path. A smaller formal covariance is not an accuracy improvement unless ground-truth error and empirical coverage agree.

## 2. Honest accuracy: model the dominant error

For illustrative independent range-error terms:

```text
σ_range² ≈ σ_orbit_clock² + σ_code_noise² + σ_multipath²
          + σ_residual_iono² + σ_residual_tropo² + σ_bias²
position covariance ≈ (Hᵀ R^-1 H)^-1, using a full-rank geometry model
horizontal scale ≈ HDOP * σ_range only for the simplified equal-noise model
```

For corrected satellite position and clock errors, range residual is approximately `LOS_rx_to_sat · δr_sat − c*δt_sat`, plus bias/modelling error. Orbit and clock errors may be correlated; apply atomic compatible corrections and **do not add their headline P95 limits as independent Gaussian sigmas**.

### HAS correction-only examples assumptions, not measurements

| Assumed effective terms | Before | After correction | Interpretation |
|---|---|---|---|
| Phone: code noise 3 m, multipath 3 m, orbit/clock 1.5→0.2 m | `sqrt(9+9+2.25)=4.50 m` | `sqrt(9+9+0.04)=4.247 m` | About **5.6%** lower range RMS; poor local measurements still dominate |
| Clean receiver: code noise 0.3 m, multipath 0.2 m, orbit/clock 1.5→0.2 m | `sqrt(.09+.04+2.25)=1.543 m` | `sqrt(.09+.04+.04)=0.412 m` | About **73.3%** lower range RMS, before other errors |

At an assumed HDOP=1.5, the phone equal-noise horizontal scale changes roughly **6.75→6.37 m**, not to 0.2 m. The assumed 0.2 m term is merely a scenario value, **not a translation of H2 correction MPLs**. Ionosphere, troposphere, signal-bias residuals, IF noise and their correlations must be added in a real budget.

For E1/E5a IF, `α≈2.260604`, `β≈−1.260604`; independent equal code noise is multiplied by `sqrt(α²+β²)≈2.58833`. Two 3 m raw code observations therefore yield about **7.765 m IF code noise**, before multipath. IF removes first-order ionosphere, but does not remove receiver noise. Do not use this IF example and the uncombined phone budget as if they were the same complete model.

### HAS service performance categories keep the percentiles straight

- **H2 §3 correction MPLs:** constellation-average/RMS metrics over 30-day characterization: orbit ≤20 cm Galileo / ≤33 cm GPS (95%); clock ≤12 cm Galileo / ≤15 cm GPS (95%); code biases ≤50 cm (95%). These are correction metrics under specified service-area/validity assumptions, **not receiver position P95**.
- **H2 Appendix E indicative Initial Service positioning:** Galileo-only ≤25 cm horizontal / ≤30 cm vertical **P68**; Galileo+GPS ≤15 cm horizontal / ≤20 cm vertical **P68**. Static, open sky, qualified float HAS-UA, supported signal combinations, at least 5 or 8 valid corrected satellites respectively and the SDD usage assumptions. These are **non-committed indicative values**, not phone guarantees.
- **H3 full-service targets:** horizontal <20 cm and vertical <40 cm **P95**, convergence <300 s SL1 / <100 s SL2, availability 99%. SL2 includes regional atmospheric corrections; do not claim its convergence benefit from an SL1-only decoder.
- **Live capability:** phase bias support/availability, coverage, delivery latency and current status must be observed. The 2023 SDD and 2025 reference note differ on phase-bias inclusion; missing products are never represented as zero-valued valid corrections.

For an ideal isotropic 2D Gaussian, horizontal radial `P95 = sqrt(-2 ln .05)*σ_axis ≈2.448σ_axis`. A 20 cm P95 would imply about **8.17 cm per horizontal axis** in that ideal model. This illustrates how demanding the target is; it is not a valid conversion of arbitrary H2 P68/static results, anisotropic real errors or correction MPLs.

Sequential carrier-phase estimation can reduce white measurement noise over time; ideal averaging is `σ/sqrt(N)`. Multipath, correlated code, drifting atmosphere, cycle slips, reference changes and hardware clock jumps break that simplification. H-8 must model continuous ambiguity states and full covariance, not claim centimetres after a fixed number of epochs.

### G2G accuracy contribution

Better satellite clocks/orbits may lower broadcast SISE; changed chip bandwidth/correlation properties may change receiver tracking noise. Neither amount is specified as a present SIMT gain by the early G2 note.

Illustration only: combined non-space range noise 1 m and space error 1.5→0.5 m gives `sqrt(1+2.25)=1.803→sqrt(1+.25)=1.118 m`, about **38.0%** lower RMS. If HAS already reduces the same space term to 0.2 m, improving the uncorrected broadcast term is not another independent 38% improvement on top of HAS. Evaluate one combined error budget.

QP is principally an acquisition aid. A lower effective tracking bandwidth can worsen ranging precision if used as a standalone range; general timing-noise scaling `σ_tau ∝ 1/(B_effective*sqrt((C/N0)*T_integration))` depends on discriminator/filter constants and signal shape. Do not assume faster acquisition means finer tracking or better steady-state fix accuracy.

## 3. Startup and convergence critical paths

Use **absolute readiness timestamps from the same monotonic start**:

```text
TTFF ≈ max(t_enough_acquired, t_unambiguous_time,
           t_required_navigation_or_aid_ready, t_enough_quality_observations) + T_solve
HAS usable result also requires t_complete_valid_correction_bundle
TTFAF also requires t_required_nav_evidence_verified
SAS result also requires applicable_RECS, verified_disclosed_key and covered_capture
```

If a navigation stage starts only after acquisition, its readiness timestamp already includes acquisition. Do not both sum stages and then count them again in a maximum. PPP convergence is an additional time-to-accuracy process; HAS cannot guarantee a faster ordinary first code fix.

### G1 E5a-QP timing example assumptions

Sequential stages: legacy acquisition 30 s + navigation/time 20 s + solve 0.05 s gives **50.05 s**. A 3× acquisition improvement changes acquisition to 10 s, giving **30.05 s**: overall only **1.67× faster**, about 40% less time.

With navigation/assistance ready independently at 20 s, the same acquisition change gives `max(30,20)+.05=30.05 s` versus `max(10,20)+.05=20.05 s`: only **1.50×**. If a longer independent readiness stage dominates both, ordinary TTFF gain can be near zero.

Energy-to-fix is `integral P(t) dt`, not simply proportional to code length. Report acquisition joules and total startup joules. Higher compute parallelism or receiver handoff can change power despite lower algorithmic complexity.

### HAS message-delivery example assumptions

For k pages, n simultaneous broadcasting satellites and independent usable/unique-page probability p, a best-case average collection estimate is `T_pages≈k/(n*p)` seconds if each sends a page per second. Example k=20, n=6, p=.8 ⇒ **4.17 s**. This ignores repeated PIDs, scheduling, non-broadcasting SVs, startup sync, missing masks/clock blocks and Internet latency; it is not an operational bound. The normative incomplete-message timeout is **150 s**, not expected acquisition time.

A HAS code result is usable only after issue-compatible orbit/clock/required code biases are all valid. A phase estimator needs continuous usable observations; TTFF, correction readiness and convergence must all be reported.

### OSNMA timing and security contribution

A normal tag's key is sent in the next 30 s MACK; slow ADKD12 adds **300 s**, so key selection includes one plus ten subframes. Full reception, root/public-key bootstrap and navigation history create extra delay. Cold/warm/hot **trust** starts differ from receiver cold/warm/hot navigation starts.

O3 tag-availability metrics include ADKD0 opportunities for four satellites within 30 s and all in-view within 600 s; ADKD12 four within 240 s; ADKD4 one within 60 s. These refer to **tag provision**, not guaranteed receiver TTFA or TTFAF. ADKD12 provision does not remove delayed-key waiting. Safe time requires half-width B<15 s for normal or B<165 s for slow tags (O2 Annex C).

With identical honest observations/geometry, OSNMA's intrinsic numeric-accuracy gain is **zero**. A strict subset may worsen DOP: illustrative HDOP 1.5→2.3 after exclusions is a **53%** increase in geometry-amplified error scale. Its value is avoiding accepted falsified navigation under the defined threat model, not improving C/N0 or proving measured ranges genuine.

For ideal independent admissible 40-bit tags, per-attempt blind-guess probability is approximately `9.09e-13`. A union bound for q attempts is `q/2^40`; protocol timing/trust flaws, correlated/repeated tags, receiver compromise or relay attacks are not covered by that number. Missing authentication is not evidence of spoofing.

### SAS and G2 TDS timing

SAS adds deferred RECS/key/capture verification. No official normative latency/performance profile was located for this study; report **unknown**, not a fabricated number. Signal authenticity may help reject manipulated ranges within its eventual threat model; honest-fix accuracy does not improve automatically.

The early G2 note's <248 ms resolves a **±2.4 s coarse-time ambiguity** from one healthy QP signal with suitable prior and tracking. It does not include receiver acquisition, full navigation/week, enough satellite geometry or authentication. If those are already ready and time ambiguity is the bottleneck, it can improve startup; otherwise the total fix benefit may be zero.

## 4. Per-slice contribution map

| Slices | Independent value | Numeric improvement claim allowed at completion |
|---|---|---|
| A-0/A-1/A-2 | Protected baseline, exact inputs, isolated bounded state | None; must establish non-regression |
| H-1/H-2/H-3 | Correct delivery and decoding | Decode/reception latency and conformance only |
| H-4/H-5 | Valid compatible corrections with correct signs/references | Correction-level residuals against oracle/truth |
| H-6 | Opt-in corrected code solve | Dataset/device-specific code-position gain, not full HAS target |
| H-7 | Original uncombined code/phase input | Continuity/coverage/noise evidence, not sustained PPP accuracy |
| H-8/H-9 | Multi-epoch float PPP and qualification | Qualified time-to-accuracy and empirical errors with stated service/device conditions |
| O-1/O-2/O-3 | Safe page/trust/time/root handling | Coverage/bootstrap/compute cost; no honest accuracy gain |
| O-4/O-5/O-6/O-7 | Verified nav evidence, lifecycle and explicit policy | TTFA/TTFAF, bounded tampering outcomes and availability cost |
| S-1 | Honest readiness matrix | No SAS support or fix gain |
| S-2/S-3/S-4 | Official deferred signal evidence, once unblocked | Measured signal/threat/latency envelope only; optional range benefit separately qualified |
| Q-1/Q-2 | Telemetry and reference acquisition | Reference conformance; no phone speedup claim |
| Q-3/Q-4 | Actual receiver handoff and startup measurement | Paired device-specific acquisition/TTFF/energy gains |
| G-1/G-2/G-3 | Safe provisional identities/codes/TDS experiment | Early-interface laboratory time-resolution result only |
| G-4 | Final deployed signal/format integration | Device/service-specific gain after official conformance and field evidence |
| V-1/V-2 | Mixed-feature non-regression and attributable field evidence | Only claims backed by completed capability-specific acceptance |

Do not add gains across rows/features; HAS/G2 overlap in space-error correction, QP/TDS overlap in startup timing, and OSNMA/SAS cover different authenticity layers. H-6 plus OSNMA is not “authenticated HAS”: correction provenance is separate.

## 5. Cross-cutting implementation slices

The classes below are proposed test deliverables, not existing tests. Use the parent's core/Android focused Gradle command template after creating each one.

### A-0 Freeze the compatibility baseline

- **Deliverable:** deterministic legacy replay fixtures, public API/JVM consumer signature snapshot, solver/state/lifecycle golden outputs and baseline performance recordings at the actual implementation starting SHA.
- **Implementation:** unchanged legacy test expectations; GPS/QZSS/Galileo/BeiDou/GLONASS/SBAS, absolute SSR on/off, MSM/base RTK float/fixed, smoothing/IF/clock jumps, cold seed, imports, RAIM and reset/probe coverage. Record numeric/state hashes before running any companion code.
- **Acceptance:** proposed `GalileoLegacyCompatibilityTest` plus existing focused tests below pass. Legacy nullability/enum/constructor/method/default behaviour is identical; deterministic numeric outputs are bit-identical for unchanged inputs/order. No existing test is removed/weakened, including completed accuracy-remediation assertions. No companion construction means no new stores/queues/work. Benefit: compatibility evidence only.

### A-1 Independent contracts

- **Deliverable:** immutable full-time/per-signal/source/navigation/clock-uncertainty/capability/evidence types in an additive Galileo package, leaving existing data classes/enum values/signatures unchanged.
- **Implementation:** explicit units and raw-versus-smoothed/IF provenance, trusted-versus-observed time, navigation issue/source hashes, unsupported/unknown capabilities; core remains Android/network-free.
- **Acceptance:** proposed `GalileoContractTest`: input copies cannot be mutated externally; no frequency-only signal inference, trust inferred from CRC or false product completeness; old JVM/Kotlin consumer source and API snapshot still compile/match. No extra dependency without an O-2-specific provider necessity and separate review. Benefit: safe implementable seams only.

### A-2 Session isolation

- **Deliverable:** companion lifecycle, bounded caches/queues and immutable read snapshots; no implicit writes to old solver stores/filter state.
- **Implementation:** asynchronous crypto off legacy locks; source-specific reset, expired-data eviction, no-op/failed probe; explicitly inject navigation snapshots rather than change legacy freshness.
- **Acceptance:** proposed `GalileoSessionIsolationTest`: interleaved ingestion/reset/expiry, two engines/two companions, concurrent reads, malformed inputs, failed alternate solve and `commit=false` preserve old state/lastSolution. Start with engineering budgets of ≤1 MiB retained HAS assembly payload/cache and ≤8 MiB OSNMA byte/history cache, with metadata/actual heap measured separately; cap processing queues and reject excess work deterministically. Budget changes require measured justification, not removal of bounds. SAS snapshot caps are separately set after its normative requirements. Benefit: protection, not positioning gain.

### V-1 Mixed-feature replay

- **Deliverable:** causal comparison matrix using identical epochs across legacy-only, HAS code only, OSNMA observe-only, strict authenticated-navigation and explicitly combined eligible paths.
- **Implementation:** test SSR/SBAS/HAS source isolation, unauthenticated HAS plus authenticated I/NAV, wrong IOD, service Don't-Use, key revocation, missing transport, malicious lengths, time rollback, clock/slip/reset and no-fix. Include sequential PPP/QP/SAS/G2 cases only as those slices actually become qualified; report exclusions.
- **Acceptance:** proposed `GalileoMixedFeatureReplayTest`: all legacy outputs/state remain bit-identical whether companion succeeds/fails; HAS cannot leak into public SSR/base stores; strict policies do not overwrite legacy lastSolution; no half-bundle mixing or unsupported-signal acceptance. Paired opt-in workload benchmarks must keep legacy epoch-processing P95 increase ≤5% on the named target device at fixed rates, with measured scheduling/heap effects; if measurement noise exceeds the threshold, report inconclusive rather than pass. No opt-in workload means no measured extra callback work. This is a proposed engineering budget, not an achieved result.

### V-2 Field evidence and claims

- **Deliverable:** reproducible performance report linked to input hashes, implementation SHA, service/specification editions, receiver/firmware and independent truth; no field log uploads without privacy approval.
- **Implementation:** separate geodetic and Android devices; static open sky, walking/driving, urban multipath and outage/restart scenarios; identical RF playback where lawful/available or matched paired real sessions with clear observational limitations. Retain full failure/unavailability denominators.
- **Acceptance:** at least 30 paired startup trials per claimed mode/device/environment and at least three independent sessions per positioning condition; include a 24-hour static reference run if comparing H2's 24-hour indicative metrics. Record P50/P68/P95, vertical/horizontal bias/RMSE, confidence intervals, correction/authentication coverage, ambiguity resets, predeclared convergence threshold/dwell (example 60 s), uptime, energy and CPU/memory. Use a ≥10× more accurate independent truth reference where feasible; otherwise account for reference uncertainty and withhold finer claims. Truth must not be produced by the same HAS solution being evaluated.
- **Claim gate:** numeric improvement requires a predeclared meaningful difference and confidence interval supporting it, with no worsened false-fix/covariance-calibration outcome. Unit/reference tests alone cannot qualify phone decimetre accuracy, QP receiver speedup, SAS production support or G2 deployment. A report can honestly conclude “no material gain” or “unsupported”; that is not failed research.

## 6. Existing focused regression gates

Verified existing classes and exact commands, to run during future applicable implementation slices, not claimed as run for this documentation:

```sh
./gradlew :gnss-core:test --tests app.simt.gnss.core.parser.GalileoNavMessageParserTest
./gradlew :gnss-core:test --tests app.simt.gnss.core.correction.SsrCorrectionApplicatorTest
./gradlew :gnss-core:test --tests app.simt.gnss.core.parser.RtcmSsrDecoderTest
./gradlew :gnss-core:test --tests app.simt.gnss.core.solver.GnssPositionSolverTest
./gradlew :gnss-core:test --tests app.simt.gnss.core.filter.GnssInterEpochKalmanFilterTest
./gradlew :gnss-core:test --tests app.simt.gnss.core.filter.HatchFilterTest
./gradlew :gnss-core:test --tests app.simt.gnss.core.solver.RtkAmbiguityMemoryTest
./gradlew :gnss-core:test --tests app.simt.gnss.core.solver.RtkDoubleDifferenceSolverTest
./gradlew :gnss-android:testDebugUnitTest --tests app.simt.gnss.SimtGnssEngineTest
./gradlew :gnss-android:testDebugUnitTest --tests app.simt.gnss.GnssMeasurementProcessorRawEpochTest
./gradlew :gnss-android:testDebugUnitTest --tests app.simt.gnss.GnssGroupDelayConventionTest
./gradlew :gnss-android:testDebugUnitTest --tests app.simt.gnss.GnssSsrPositioningTest
./gradlew :gnss-android:testDebugUnitTest --tests app.simt.gnss.GnssMultiBandTest
./gradlew :gnss-android:testDebugUnitTest --tests app.simt.gnss.HalfCycleAmbiguityGateTest
```

Core tests are in `gnss-core/src/test/java/app/simt/gnss/core/` with parser/correction/solver/filter package subdirectories; Android tests are in `gnss-android/src/test/java/app/simt/gnss/`. Verify the paths/classes at implementation time, since this baseline may evolve. Full two-module tests/Spotless/build run at the parent's named phase exits, never via publishing tasks.

## 7. Documentation-only completion check

For this plan's creation, check all six Markdown files for: local links/anchors, unique tracker IDs, existing dependency targets and an acyclic graph, a detailed acceptance section for every slice, explicit blocked production prerequisites, ≤400 lines per file, balanced code fences, no trailing whitespace and a documentation-only working-tree diff. Independently recalculate the illustrative budgets and timing/IF factors. Check source editions against the actual PDF headings, not just catalogue filenames.

The successful Gradle task-discovery command confirms runner task names only. No implementation, numerical conformance tests, device feature exercise or field benchmark is claimed complete by this document. All future slice evidence belongs in the canonical tracker with actual test commands/results and approved implementation SHA; no commit/push is authorised here.
