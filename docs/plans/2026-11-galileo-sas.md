# Galileo SAS: readiness-gated signal authentication

Parent: [canonical tracker and source register](2026-11-galileo-capabilities.md). Dependency: [OSNMA verifier and trusted time](2026-11-galileo-osnma.md). All production SAS slices are blocked pending evidence, not deferred behind fabricated interfaces.

## What is verified as of 2026-11-17

S1, published **2026-08-06**, describes Galileo completing qualification of SAS's Ground Mission Segment and E-GSC elements. SAS uses the **encrypted E6-C signal component**. The E-GSC SAS server supplies Internet-delivered **RECS: Re-Encrypted Code Sequences**, re-encrypted with **future OSNMA keys**. After legitimate disclosure/verification, users can obtain ranging information without storing confidential signal-generation material.

Internal L3-satellite encryption testing began 2025-12-03. Additional encryption tests and a final configuration were targeted within 2026, except IOV GSAT0101/0102/0103. These are programme testing/rollout statements, **not proof of an operational civilian SAS service or guaranteed receiver performance**.

S1 says official ICD/receiver guidelines would be published in coming months. The public SAS normative ICD, receiver guidelines and conformance vectors were **not located in the official sources/searches reviewed for this study**. This is a dated research limit, not proof that no newer document exists. Recheck GSC and [Service Notice 21](https://www.gsc-europa.eu/sites/default/files/sites/all/files/Galileo-service-notice-21-v1.0.pdf) before any production implementation.

**Do not infer** RECS framing, encryption algorithm/mode, nonce, signal-time mapping, key-index mapping, tag placement, file validity, access conditions or detection thresholds from news or the service acronym.

## Architectural and hardware boundary

Proposed isolated package: `app.simt.gnss.core.galileo.sas`. Initially only readiness/evidence contracts are useful. Later components may include `SasRecsVerifier`, `SasCorrelationEvidence` and `SasEvidencePolicy`; those names are provisional until the official ICD is reviewed.

Two possible hardware paths must be qualified separately:

1. **Sample path:** receiver/SDR exposes timestamped E6-C complex samples or suitable correlation snapshots, sample clock calibration, antenna/channel group delay and original capture-time provenance.
2. **Trusted receiver path:** device firmware implements the official SAS algorithm and exports documented, issue/time-bound evidence with a trustworthy local device interface and failure status. A bare vendor Boolean is not cryptographic proof.

Ordinary Android `SatelliteMeasurement` pseudorange, ADR, carrier frequency and C/N0 do not contain encrypted spreading-code samples and are **not sufficient** to implement SAS correlation. E6 availability alone does not prove encrypted E6-C capture or RECS support. Do not reconstruct missing signal evidence from scalar ranges or OSNMA-verified ephemerides.

The host owns RECS credentials/transport, optional file persistence, snapshot acquisition and receiver firmware. No confidential satellite signal-generation keys are stored in this library. Disclosed OSNMA keys are consumed only after chain/time verification and under the eventual RECS applicability rules.

## Non-normative calculation study

These equations specify experiment accounting, not an official SAS algorithm.

A complex-sample correlation experiment could evaluate a known sequence s against captured samples y over allowed delay/Doppler hypotheses:

```text
C(tau,nu) = Σ_n y[n] * conjugate(s[n - tau]) * exp(-j*2*pi*nu*n/Fs)
normalized statistic = |C|² / (Σ|y|² * Σ|s|²)
```

The actual sample representation, filtering, reference sequences, integration duration and decision rule must come from the ICD/receiver guidelines. Record the full hypothesis set, calibration and capture interval; do not hide an arbitrary threshold behind “authenticated”.

For N hypothesis tests, a conservative union bound is `P_FA_total ≤ N * P_FA_cell`; selecting a per-cell threshold while ignoring the search grid inflates false-positive risk. Estimate false accept/dismiss under independently generated data and official vectors, with specified confidence intervals. Code correlation alone cannot prove absence of delay relay/meaconing; the official time/geometry constraints determine what is detectable.

Resource accounting:

```text
snapshot_bytes = Fs * T_capture * bytes_per_complex_sample
retained_bytes = snapshot_bytes * queued_snapshots
T_result >= max(snapshot_complete, required_key_verified, applicable_RECS_ready)
             + correlation_compute_time
range_error_from_timestamp = c * timestamp_error
```

Illustration only: 10 MS/s, 0.25 s, signed 16-bit I and Q ⇒ **10 MB per snapshot**. Four such pending captures need 40 MB plus processing overhead. This is not a prescribed SAS sample rate or duration. A 10 ns timestamp error corresponds to about **3 m**; an E6-C range fusion therefore needs calibrated clock/channel delay, not just a correlation peak.

Result times are inherently delayed by key disclosure/transport/computation; evidence for a previous capture is historical evidence, not automatically proof of a later live fix. Store capture interval, key applicability/verification time, processing time, satellite/component, clock uncertainty and the precise measurements covered.

## Slices and explicit unblocking criteria

### S-1 Readiness dossier

- **Deliverable:** a small versioned capability/evidence contract plus a device/service readiness matrix. No speculative decoder, fake RECS messages or production authentication status.
- **Implementation:** enumerate unknown versus unsupported versus documented/observed E6-C capture; firmware/version; official service declaration/notices; protocol editions; trust chain; host access and calibration. Keep RECS, OSNMA and receiver identities distinct.
- **Acceptance:** proposed `SasCapabilityTest`: an ordinary Android scalar epoch reports SAS unavailable; E6 frequency alone remains insufficient; receiver firmware declaration is distinguishable from verified evidence; absent ICD prevents a production-supported status. Completing this dossier alone delivers no fix accuracy/TTFF gain and must say so.

### S-2 Official RECS and disclosed-key processing BLOCKED

- **Unblock with:** official normative ICD, receiver guidelines, versioned conformance vectors, authorised service access and a documented OSNMA-key binding.
- **Deliverable after unblocking:** exact RECS metadata/parser/cryptographic validation and verified-key applicability adapter, with host-owned network I/O.
- **Implementation after unblocking:** bind sequence data to satellite/component/full signal time, service version, capture applicability and verified OSNMA chain/key; enforce all official authenticity/integrity/expiry rules. No guess at a cipher or key offset is allowed in advance.
- **Acceptance:** proposed `SasRecsConformanceTest`: all official nominal and negative vectors; corrupt/truncated/expired data, wrong satellite/time/chain/key, revoked material and disclosure-order violations reject or stay unavailable as required. Neither unverified keys nor just a TLS connection can produce verified signal evidence. No credential/content leakage into diagnostics; legacy SSR and navigation remain unchanged.

### S-3 Signal correlation or hardware evidence BLOCKED

- **Unblock with:** S-2 plus timestamped E6-C-capable hardware/SDR or a documented verified-evidence firmware interface; calibrated capture/channel delay and official correlation/decision requirements.
- **Deliverable after unblocking:** one qualified hardware path, not an abstraction claiming support for both without evidence; bounded snapshot/processing queue and historical evidence output.
- **Implementation after unblocking:** official matched-signal processing and uncertainty accounting, capture-before-relevant-disclosure requirements, validated receiver metadata; avoid unbounded RF buffers.
- **Acceptance:** proposed `SasSignalEvidenceTest`: official correlation/evidence vectors, wrong sequence, noise-only/off-band input, time misalignment, clock error, resampling/group delay, dropped captures and resource exhaustion. An offline authorised laboratory replay checks specified delay/threat limits without transmitting RF. Predeclare P_FA/P_MD and thresholds from the official profile; unavailable evidence must never be called spoofing or verified. Measure memory/CPU and prove legacy scheduling stays inside V-1's budget.

### S-4 Signal-evidence policy BLOCKED

- **Unblock with:** S-2/S-3 conformance, documented operational service scope and official threat/delay model.
- **Deliverable after unblocking:** explicitly requested `SasEvidencePolicy`, separate from observe-only OSNMA and all legacy position APIs; per-capture result provenance and expiry.
- **Implementation after unblocking:** associate evidence only with the measurement/capture it covers; combine OSNMA navigation and SAS signal evidence without broadening either scope. Any additional E6-C range enters only a new qualified estimator with independent noise/delay calibration.
- **Acceptance:** proposed `SasEvidencePolicyTest`: stale/history-only evidence cannot authenticate a newer fix; mixed satellites/signals are labelled accurately; missing RECS, unavailable Internet, revoked key or delayed processing yields a reasoned unavailable result. Offline threat tests respect the official detection envelope. Report time-to-evidence and false-accept/dismiss intervals separately from honest-position error; no total-TTFF improvement or universal meaconing protection is claimed.

## Expected contribution

The primary benefit is **signal authenticity evidence**, complementing OSNMA's navigation-data authenticity. With the same honest measurements/estimator, neither produces an intrinsic reduction of range noise, DOP or covariance. Additional E6-C ranging could help only if it adds valid independent information; no quantitative accuracy gain is established by S1.

SAS can make a trustworthy-result policy **slower**, because key disclosure, RECS availability and correlation lie on its critical path. Offline/online requirements and service coverage remain unsettled until the official documents are obtained. Successful S-1 research is not production SAS support, and S-2…S-4 cannot be marked complete using mocks or secondary-paper assumptions.

S1 is an EUSPA programme-status source, not a normative wire specification. The proposed equations/resource examples above are engineering analysis only.
