# Galileo capabilities: additive implementation plan

**Study date:** 2026-11-17. **Repository:** `simt-gnss`. **Baseline:** `be368fececb39ce22d5a97dd49837bf63d45d38b`.

**Status:** planning only; no feature implementation is authorised by this document. All implementation slices below are Planned or Blocked. The user's constraint is: **“without touching or impacting the already implemented features.”**

## Reading order

1. This document: scope, architecture, non-regression contract and canonical slice tracker.
2. [HAS decoder, corrections and sequential PPP](2026-11-galileo-has.md).
3. [OSNMA navigation authentication](2026-11-galileo-osnma.md).
4. [SAS readiness and signal authentication](2026-11-galileo-sas.md).
5. [E5 Quasi-Pilot and Galileo Second Generation](2026-11-galileo-startup-g2g.md).
6. [Accuracy, latency, security and validation study](2026-11-galileo-gains-validation.md).

Source identifiers H1–H6, O1–O4, Q1–Q2, G1 and S1 refer to the source register below, not slice IDs. Slice IDs use hyphens, for example H-1.

## Conclusions and current readiness

| Capability | What the library can add | What it cannot provide alone | Defensible gain |
|---|---|---|---|
| HAS | Page/message decoder, independent correction store, explicitly requested corrected code solve, new sequential float PPP | E6-B RF reception; IDD access credentials; geodetic phase continuity and calibrated antennas | Lower orbit/clock/bias error; decimetre positioning is conditional on the complete estimator, observations and service |
| OSNMA | E1-B page extraction, trust-chain and TESLA verification, issue-bound evidence, separate authenticated-fix policy | A trustworthy clock from untrusted GNSS; authentication of pseudoranges | Detects specified navigation-data tampering; no intrinsic honest-fix accuracy or acquisition gain |
| SAS | Readiness contracts and laboratory evidence model; later official RECS processing and hardware evidence adapter | Encrypted E6-C ranging from ordinary Android scalar observations | Signal-origin evidence, subject to the eventual threat model; not a blanket anti-replay guarantee |
| G1 E5a-QP | Capability reporting, independent code reference implementation and startup benchmarks | Changing chipset acquisition/tracking through standard Android callbacks | GSC reports typical 8× acquisition-complexity reduction and 3× acquisition-speed improvement, not 3× total TTFF |
| G2G | Versioned signal identity, early-interface reference codes/TDS prototype and future adapters | Satellite upgrades, unpublished navigation formats, unsupported RF bands | Potential faster coarse-time resolution; actual fix and SISE gains require deployment and measurements |

HAS Initial Service began 2023-01-24. OSNMA Initial Service became operational 2025-07-24. GSC lists 12 G1 E5a-QP-reconfigured satellites at study time; this is not the number visible to a user. S1 describes SAS qualification/testing and expected documentation, not an operational service declaration. G1 is early interface information, explicitly subject to change.

**Important discrepancies:** the in-force HAS catalogue dates its SIS ICD entry July 2023, but the downloaded `in_force` PDF identifies itself as Issue 1.0, May 2022. H4 is catalogued December 2025, but its internal heading is November 2025. Pin the document bytes, not a search snippet. H2 says Initial Service phase biases were not yet provided in January 2023; H4 §3.2.3 includes phase biases. Neither establishes the live stream's November 2026 contents. Decoder support, observed block availability and operational performance must be reported separately.

## Verified architecture and protected behaviour

| Existing evidence, repository-relative | Consequence for this plan |
|---|---|
| [SimtGnssEngine.kt](../../gnss-android/src/main/java/app/simt/gnss/SimtGnssEngine.kt), lines 35–172 | Stable thread-safe façade, public SSR/base stores, navigation/measurement ingestion, absolute/RTK solves, probe and reset semantics remain unchanged |
| [GnssNavMessageIngest.kt](../../gnss-android/src/main/java/app/simt/gnss/GnssNavMessageIngest.kt), lines 74–79, 126–147 | Existing rebuilt-parity rejection; only Galileo TYPE_GAL_I and words 1–6 enter the cache; use a separate full-page input for OSNMA |
| [GalileoNavMessageParser.kt](../../gnss-core/src/main/java/app/simt/gnss/core/parser/GalileoNavMessageParser.kt), lines 39–48, 68–163 | Android 228-bit even/odd layout, health, IOD and both BGDs already implemented; do not rewrite the ephemeris parser |
| [GnssMeasurementProcessor.kt](../../gnss-android/src/main/java/app/simt/gnss/GnssMeasurementProcessor.kt), lines 103–185 | Existing PppState holds a seed/ambiguity placeholders, not a sequential PPP covariance estimator; existing reset remains its own lifecycle |
| [GnssAbsoluteSolver.kt](../../gnss-android/src/main/java/app/simt/gnss/GnssAbsoluteSolver.kt), lines 178–181, 199, 496 | Absolute estimation intentionally avoids unobservable single-epoch phase rows; HAS must not reintroduce them |
| [SsrCorrectionApplicator.kt](../../gnss-core/src/main/java/app/simt/gnss/core/correction/SsrCorrectionApplicator.kt), lines 54–96, 155–164 | Existing SSR subtracts orbit deltas and evaluates polynomials at epoch + half interval; HAS adds orbit deltas and has different time semantics |
| [SsrCorrectionPipeline.kt](../../gnss-android/src/main/java/app/simt/gnss/SsrCorrectionPipeline.kt), lines 52–69 | Existing atomic IOD-matched SSR orbit/clock pairing is protected; do not feed HAS into the exposed SSR store |
| [GnssModels.kt](../../gnss-core/src/main/java/app/simt/gnss/core/model/GnssModels.kt) | Existing bands are frequency buckets, not Galileo component/code identities; new signals need independent exact identities |
| [ProcessedMeasurements.kt](../../gnss-android/src/main/java/app/simt/gnss/ProcessedMeasurements.kt), lines 12–15 | Legacy processed epochs are not complete uncombined, per-code PPP observations or authentication evidence |
| [Completed accuracy-remediation tracker](2026-11-gnss-engine-accuracy-remediation.md), lines 27–32 | All six completed remediation slices remain untouched, including noise, clock continuity, IF, SSR, RTK and lifecycle corrections |

`:gnss-core` remains pure JVM with no Android or network dependency. `:gnss-android` remains an Android adapter, not an NTRIP client, persistence layer or RF receiver. The host owns transport, credentials, secure time/trust provisioning, antenna files, device commands and user acceptance policy. No changes to `simt-app` are included.

## Isolation contract

1. Prefer **new companion objects/classes with explicit calls**, rather than adding a global switch to the existing solver. Proposed names in the detail documents do not exist yet.
2. A host delivers a copied navigation page separately to the existing engine and the new Galileo session. No automatic hook into legacy ingest is necessary. Full original pages must reach OSNMA before the host discards non-ephemeris word types.
3. Keep HAS corrections, OSNMA page histories, SAS evidence, PPP state and startup telemetry in independent bounded stores. No writes into `ssrStore`, `rtkBaseStore`, `PppState`, Hatch, TDCP or the existing Kalman filter.
4. New estimation consumes immutable snapshots of original observations and navigation records. Do not mutate `SatelliteMeasurement`, reinterpret `isL5`, overwrite imported ephemeris, or relabel `GnssSolutionMode`.
5. A HAS-specific raw observation adapter operates separately from `processMeasurements`; the latter may already combine/smooth measurements. Sharing already-smoothed code with phase without modelling correlation is forbidden.
6. Strict authentication is a **separate explicitly invoked solve/policy**, not silent rejection in `computePosition`. Observe-only evidence never alters legacy weighting, satellite selection or diagnostics.
7. A companion session has its own reset/probe methods. Hosts explicitly reset both sessions when desired. Legacy `resetSession` keeps its existing contract; new metadata must not claim it reset unowned state.
8. A failed or non-committing alternate solve changes neither session's committed solution. No HAS/PPP result silently seeds legacy WLS/RTK, and vice versa.
9. New signal identity never widens existing frequency buckets. Hardware capabilities are `unknown`, `declared`, `observed` or `unsupported`, with firmware/version evidence; a frequency match alone is insufficient.
10. Cryptographic verification stays off the legacy processing lock; bounded asynchronous work, monotonic timestamps and immutable input copies prevent added callback latency and races.
11. Only after separate approval may a minimal legacy façade adapter be considered. It is not required by this plan and cannot be used to justify changing legacy algorithms or defaults.

“No impact” is a testable compatibility requirement, not a promise that extra opt-in workloads consume zero CPU. With no companion constructed, behaviour must be identical. With a companion running, legacy numeric output and state must remain identical on deterministic replay; measure scheduling/resource effects separately.

## Canonical slice tracker

Tier estimates S/M/L are scope, not calendar commitments. Lane ownership is within `simt-gnss` unless explicitly labelled host/hardware. Every slice inherits A-0's compatibility gate. A Blocked row cannot become Complete merely through synthetic tests.

| id | tier | lane | depends | title | state | evidence |
|---|---|---|---|---|---|---|
| A-0 | M | compatibility | - | Freeze legacy replay, API and state baselines | Planned | [Acceptance](2026-11-galileo-gains-validation.md#a-0-freeze-the-compatibility-baseline) |
| A-1 | M | contracts | A-0 | Independent Galileo inputs, capabilities and result evidence | Planned | [Acceptance](2026-11-galileo-gains-validation.md#a-1-independent-contracts) |
| A-2 | M | lifecycle | A-1 | Bounded session, snapshots, probe and reset isolation | Planned | [Acceptance](2026-11-galileo-gains-validation.md#a-2-session-isolation) |
| H-1 | S | has-pages | A-1 | Validated E6-B HAS page boundary and header | Planned | [Details](2026-11-galileo-has.md#h-1-page-boundary-and-header) |
| H-2 | M | has-rs | H-1 | HPVRS erasure reconstruction | Planned | [Details](2026-11-galileo-has.md#h-2-hpvrs-reconstruction) |
| H-3 | M | has-masks | H-2 | MT1 masks, component layouts and exact identities | Planned | [Details](2026-11-galileo-has.md#h-3-mt1-layout-and-masks) |
| H-4 | M | has-store | H-3, A-2 | Issue-bound, time-valid atomic correction snapshots | Planned | [Details](2026-11-galileo-has.md#h-4-correction-store-and-validity) |
| H-5 | M | has-model | H-4 | HAS-specific orbit, clock, code and phase application | Planned | [Details](2026-11-galileo-has.md#h-5-correction-application) |
| H-6 | M | has-code | H-5 | Separate corrected code-positioning path | Planned | [Details](2026-11-galileo-has.md#h-6-corrected-code-positioning) |
| H-7 | M | has-input | H-5, A-2 | Uncombined phase observation and host-input adapters | Planned | [Details](2026-11-galileo-has.md#h-7-observation-and-host-adapters) |
| H-8 | L | has-ppp | H-6, H-7 | New sequential uncombined float PPP estimator | Planned | [Details](2026-11-galileo-has.md#h-8-sequential-float-ppp) |
| H-9 | L | has-validation | H-8 | Reference/replay and device-specific performance qualification | Planned | [Details](2026-11-galileo-has.md#h-9-reference-and-device-qualification) |
| O-1 | M | osnma-pages | A-1 | E1-B full-page extraction and timed subframe history | Planned | [Details](2026-11-galileo-osnma.md#o-1-full-page-extraction) |
| O-2 | M | osnma-crypto | A-1 | Trust material, crypto provider and trusted-time contracts | Planned | [Details](2026-11-galileo-osnma.md#o-2-trust-crypto-and-time) |
| O-3 | M | osnma-root | O-1, O-2 | DSM-PKR and DSM-KROOT validation | Planned | [Details](2026-11-galileo-osnma.md#o-3-public-key-and-root-verification) |
| O-4 | L | osnma-mack | O-3 | TESLA, MACK, MACSEQ and exact navigation tags | Planned | [Details](2026-11-galileo-osnma.md#o-4-tesla-and-navigation-tags) |
| O-5 | M | osnma-status | O-4, A-2 | Renewals, revocation, expiry and issue-bound evidence | Planned | [Details](2026-11-galileo-osnma.md#o-5-lifecycle-and-evidence) |
| O-6 | M | osnma-policy | O-5 | Explicit authenticated-navigation fix policy | Planned | [Details](2026-11-galileo-osnma.md#o-6-separate-fix-policy) |
| O-7 | L | osnma-validation | O-6 | Official vectors, hostile-input replay and TTFA qualification | Planned | [Details](2026-11-galileo-osnma.md#o-7-conformance-and-performance) |
| S-1 | S | sas-readiness | A-1 | Normative-service and E6-C hardware readiness dossier | Planned | [Details](2026-11-galileo-sas.md#s-1-readiness-dossier) |
| S-2 | M | sas-recs | S-1, O-5 | Official RECS validation and disclosed-key processing | Blocked | Public ICD/vectors and authorised access required |
| S-3 | L | sas-correlation | S-2 | E6-C snapshot correlation or verified hardware evidence | Blocked | ICD and capable calibrated receiver required |
| S-4 | M | sas-policy | S-3, O-6 | Signal-evidence policy and delayed-result qualification | Blocked | Official threat model and S-2/S-3 evidence required |
| Q-1 | S | qp-capability | A-1 | Exact G1 E5a-QP identity and availability telemetry | Planned | [Details](2026-11-galileo-startup-g2g.md#q-1-g1-capability-and-telemetry) |
| Q-2 | M | qp-reference | Q-1 | Independent 330-chip reference acquisition prototype | Planned | [Details](2026-11-galileo-startup-g2g.md#q-2-g1-reference-codes-and-acquisition) |
| Q-3 | L | qp-hardware | Q-2 | Receiver handoff and unambiguous-time qualification | Blocked | Receiver/firmware or SDR acquisition access required |
| Q-4 | M | qp-validation | Q-1 | Cold/warm/hot TTFF, energy and acquisition study | Planned | [Details](2026-11-galileo-startup-g2g.md#q-4-g1-startup-benchmark) |
| G-1 | S | g2-contracts | A-1 | Versioned G2 signal and satellite-generation metadata | Planned | [Details](2026-11-galileo-startup-g2g.md#g-1-g2-identity-and-readiness) |
| G-2 | M | g2-reference | G-1 | Provisional Gold-code reference prototype | Planned | [Details](2026-11-galileo-startup-g2g.md#g-2-g2-code-reference-prototype) |
| G-3 | M | g2-tds | G-2 | Provisional TDS/QHS decoder and coarse-time experiment | Planned | [Details](2026-11-galileo-startup-g2g.md#g-3-tds-and-quasi-health-prototype) |
| G-4 | L | g2-production | G-3 | Final-ICD receiver integration and new-message conformance | Blocked | Final ICD, code assignments, receiver and operational evidence required |
| V-1 | M | mixed-replay | H-6, O-6, A-2 | Mixed-feature compatibility and source-policy qualification | Planned | [Acceptance](2026-11-galileo-gains-validation.md#v-1-mixed-feature-replay) |
| V-2 | L | field-evidence | V-1 | Reproducible per-capability field performance report | Planned | [Acceptance](2026-11-galileo-gains-validation.md#v-2-field-evidence-and-claims) |

Q-4 can establish a baseline without Q-3; no QP gain may be claimed until a real QP-capable receiver has been compared. V-2 qualifies only implemented/unblocked capabilities and must explicitly exclude blocked ones. H-8 and O-4 are substantial estimators/protocols: split internally further if the first implementation slice cannot be reviewed independently, retaining the same exit gates.

## Execution waves and stop conditions

- **Wave 0:** A-0, A-1, A-2. Produce reusable compatibility evidence before any live-path integration.
- **Wave 1:** H-1…H-5 and O-1…O-3 in independent worktrees/lanes. Q-1, Q-4, G-1 and S-1 can proceed as telemetry/readiness work.
- **Wave 2:** H-6/H-7, O-4…O-7 and V-1. This provides corrected code fixes and usable authentication without claiming full HAS accuracy.
- **Wave 3:** H-8/H-9; receiver-backed Q-2/Q-3/Q-4. G-2/G-3 remain versioned laboratory prototypes.
- **Wave 4:** S-2…S-4 and G-4 only after prerequisites are evidenced. V-2 issues separate capability-specific reports.

Stop a slice on missing normative bit definitions, unavailable trustworthy time, unsupported crypto, unverified signal identity, absent hardware or failed legacy replay. Never substitute invented protocol details, permissive gates, mock field evidence or a changed existing test expectation.

## Acceptance command convention

Tasks were discovered with `./gradlew :gnss-core:tasks --all :gnss-android:tasks --all --console=plain`; this succeeded on 2026-11-17 with existing AGP/deprecation warnings. This was task discovery, **not a test-suite run**.

Detail-document test class names are proposed. Once each class exists, its focused gate is:

```sh
./gradlew :gnss-core:test --tests app.simt.gnss.core.galileo.<ProposedTestClass>
./gradlew :gnss-android:testDebugUnitTest --tests app.simt.gnss.<ProposedAdapterTestClass>
```

For an implementation phase exit (named cross-system trigger):

```sh
./gradlew :gnss-core:test :gnss-android:testDebugUnitTest spotlessCheck
./gradlew build
```

Run focused old-path tests named in the validation document for every applicable slice; run both complete modules at phase exits. Future executable slices require behavioural test deltas. No test execution is necessary to claim this documentation was written; documentation checks must not be described as feature validation. Commits, pushes and deployments require separate authorisation.

## Source register

Sources were reviewed on 2026-11-17. Normative specifications govern decoding; reference algorithms guide estimator design; service targets and news are not receiver guarantees. Detailed derivations below are engineering proposals, not modifications to an official ICD.

| ID | Document and status | Relevant sections |
|---|---|---|
| H1 | [HAS SIS ICD, in-force PDF, Issue 1.0, internally May 2022](https://www.gsc-europa.eu/sites/default/files/sites/all/files/Galileo-HAS-SIS-ICD_in_force.pdf) | §§2–7; Annex B generator matrix and Annex C worked reconstruction |
| H2 | [HAS SDD 1.0, January 2023](https://www.gsc-europa.eu/sites/default/files/sites/all/files/Galileo-HAS-SDD_in_force.pdf) | §2 assumptions/coverage; §3 correction MPLs; Appendices D/E estimator and indicative performance |
| H3 | [HAS information note 1.1, December 2025](https://www.gsc-europa.eu/sites/default/files/sites/all/files/Galileo-HAS-Info-Note_in_force.pdf) | §3 Table 2 full-service targets; §5 roadmap |
| H4 | [HAS Reference User Algorithm Technical Note 1.0, internally November 2025](https://www.gsc-europa.eu/sites/default/files/sites/all/files/Galileo_HAS_Reference_User_Algorithm_TN_in_force.pdf) | §§3.1–3.3 observation models, preprocessing, float EKF; antenna/wind-up appendices |
| H5 | [In-force HAS catalogue](https://www.gsc-europa.eu/electronic-library/programme-reference-documents/galileo-in-force/has) | Current editions; IDD ICD 1.1 registration restriction; June 2026 HAS-UA manual |
| H6 | [HAS reference algorithm portal](https://www.gsc-europa.eu/support-to-developers/galileo-has-reference-algorithm) | Reference software/data access and terms to verify before fixture import |
| O1 | [OSNMA SIS ICD 1.1, October 2023](https://www.gsc-europa.eu/sites/default/files/sites/all/files/Galileo-OSNMA-SIS-ICD_in_force.pdf) | §§2–6 and Annexes B/C, exact data/tag/chain formats |
| O2 | [OSNMA Receiver Guidelines 1.3, January 2024](https://www.gsc-europa.eu/sites/default/files/sites/all/files/Galileo-OSNMA-RX-Guidelines_in_force.pdf) | §§2–5, Annex B test vectors, Annex C trusted-time bounds |
| O3 | [OSNMA SDD 1.0, July 2025](https://www.gsc-europa.eu/sites/default/files/sites/all/files/Galileo-OSNMA-SDD_in_force.pdf) | Service scope and authentication-tag availability, not guaranteed TTFAF |
| O4 | [In-force OSNMA catalogue](https://www.gsc-europa.eu/electronic-library/programme-reference-documents/galileo-in-force/osnma) | PKI policy, IDD and official vector archive |
| Q1 | [OS SIS ICD 2.2, November 2025](https://www.gsc-europa.eu/sites/default/files/sites/all/files/Galileo_OS_SIS_ICD_v2.2.pdf) | §2.3.1.4; §§3.1, 3.4.2, 3.6.1; Table 38 page fields; §5.1.9.4 CRC |
| Q2 | [GSC OS Quasi-pilot service page](https://www.gsc-europa.eu/galileo/services/galileo-open-service/quasipilot) | G1 availability/NAGUs and typical acquisition benefits |
| G1 | [G2 evolution of Quasi-Pilot signals, Issue 1.0, December 2025](https://www.gsc-europa.eu/sites/default/files/sites/all/files/G2_Evolution_of_Quasi-Pilot_Signals_and_Interface_Control_Information.pdf) | Early, changeable interface; §§2–5 signals/codes/TDS/QHS, not a final G2 navigation ICD |
| S1 | [EUSPA: Galileo SAS underway, 2026-08-06](https://www.euspa.europa.eu/newsroom-events/news/galileo-signal-authentication-service-underway) | E6-C, future OSNMA-key RECS, qualification and documentation readiness |

Recheck catalogues/NAGUs before implementation. Store URL, internal issue/date, retrieval time and SHA-256 with each imported fixture. Reviewed HAS H1 SHA-256: `7b1058520175cf4d135df0fc17c4fb339a95d645804599242204405dd2a88873`; G1: `6943c9eb98a9c8c0cc068c9716c37e10bd7f6dfb7f821e1d77e7b71ef4019ce8`. Official document copyright belongs to the European Union; attribution and redistribution terms require review before vendoring annexes, code or test data. Temporary research PDFs are not project assets.
