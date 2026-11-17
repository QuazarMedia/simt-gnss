# Galileo OSNMA: navigation authentication without legacy policy changes

Parent: [tracker, protected architecture and source register](2026-11-galileo-capabilities.md). Security/latency analysis: [gain study](2026-11-galileo-gains-validation.md).

## Service scope and proposed seam

OSNMA Initial Service is operational since **2025-07-24**. It authenticates specified Galileo I/NAV navigation data. It does **not** authenticate measured travel times, receiver position, Android system time, GPS navigation, or HAS E6-B corrections. CRC/parity validity is channel integrity, not cryptographic authentication.

Proposed pure-JVM package: `app.simt.gnss.core.galileo.osnma`, with `OsnmaPageExtractor`, `OsnmaDsmAssembler`, `OsnmaVerifier` and issue-bound `NavigationAuthenticationEvidence`. Proposed Android adapter: `GalileoOsnmaPageAdapter`. All are new components. The host feeds copied full pages to the companion session; existing `processNavigationMessage` and `GalileoNavMessageParser` remain unchanged.

Android's `TYPE_GAL_I` alone may not identify E1-B versus E5b-I. Receiver/protocol provenance must establish the component before interpreting the 40-bit E1-B field as OSNMA. Do not treat random reserved bits from an E5b page as a capability declaration. Never rely on a Galileo ephemeris-only cache for authentication history.

## Calculation and protocol contract

### 1. Full-page bit allocation

Q1 Table 38 and the existing parser establish the Android representation: 114 decoded bits for each even/odd half, packed MSB-first in 29 bytes, with sync/tails absent. For a **nominal E1-B** even+odd pair, zero-based flat bit offsets are:

| Flat bits | Contents |
|---|---|
| 0, 1 | even marker 0, nominal page type 0 |
| 2…113 | first 112 navigation bits |
| 114, 115 | odd marker 1, nominal page type 0 |
| 116…131 | remaining 16 navigation bits |
| 132…171 | OSNMA: HKROOT 8 bits then MACK 32 bits |
| 172…193 | SAR 22 bits |
| 194…195 | spare 2 bits |
| 196…219 | CRC 24 bits |
| 220…227 | SSP 8 bits, not CRC-protected |
| 228…231 | byte-packing padding, not page data |

For nominal E1-B pairs the CRC covers flat bits **0…195**, excluding CRC, SSP, padding and stripped tail/sync bits. Implement the generator and bit orientation from Q1 §5.1.9.4, using an independent reference fixture; do not infer the CRC polynomial from the existing bit getter. E5b has a different semantic field layout. Alert/dummy pages must not be interpreted as nominal authentication frames.

A 30 s subframe has 15 pages: **120 HKROOT bits and 480 MACK bits**. Assemble by transmitting PRN, component, full GST subframe, page position and source, not arrival order. Track reception independently using a trusted clock mapping plus monotonic time. Duplicates, missing pages, odd/even mismatch or navigation-issue changes cannot be repaired by stitching unrelated histories.

### 2. Trust and cryptographic primitives

O2 Table 1 requires ECDSA P-256/P-521, SHA-256/SHA-512, SHA3-256 where selected, HMAC-SHA256 and CMAC-AES. Prefer established audited platform/provider primitives; verify support on JVM 21 and Android minSdk 28 before selecting a dependency. Unsupported advertised algorithms produce `unsupported`, never a skipped verification marked successful.

Keep OSNMA serialization separate from crypto implementation: MSB truncation, exact bit concatenation, zero-padding to bytes, compressed public-key point parsing, and wire ECDSA `r||s` versus provider DER encoding. P-521 signatures use two 66-byte values containing 521 significant bits each. Reject invalid lengths/points/encodings and compare tags/roots in constant time. Do not implement home-grown elliptic curves, AES or hash primitives.

Host-provisioned trusted Merkle roots/public keys have source, validity and rollback/version metadata. A downloaded key without an authenticated trust path is **not** trusted. Network retrieval and persistence remain host responsibilities; the core receives validated trust material and records its identity in every verification result.

O1 §6 public-key verification:

```text
leaf = NPKT || NPKID || NPK
x_0 = H(leaf)
x_next = H(left_child || right_child)
repeat through four levels of the 16-leaf Merkle tree
computed_root must equal the explicitly trusted root
```

Preserve path ordering from MID, declared Merkle algorithm and DSM-PKR lengths/padding. DSM-KROOT verifies an ECDSA signature over the exact O1 Eq. 14 field concatenation plus byte padding:

```text
NMA_Header || CIDKR || Reserved1 || HF || MF || KS || TS || MACLT ||
Reserved2 || WNK || TOWHK || alpha || KROOT
```

A root/key with a valid signature still needs correct applicability time, chain identity, accepted algorithm/lengths and revocation status. NMAS/CPKS transitions must not be trusted merely because unauthenticated header bits changed.

### 3. Trusted time: the strict bound matters

O2 Annex C defines **half-width uncertainty B** such that true GST at event i is in `[t_Rx−B, t_Rx+B]`. This differs from a full-window “30 s” interpretation:

```text
normal tags usable only if B < TL/2 = 15 s
slow ADKD12 usable only if B < TL/2 + 150 s = 165 s
TL = 30 s
```

Equality is not sufficient. Validate the data/tag reception-before-key-disclosure condition with the conservative reception **upper bound**, including capture timestamp uncertainty, callback delay, GST conversion and oscillator drift. A clock contract supplies a monotonic reference and a trustworthy GST interval, not only an unchecked wall-clock value.

For bounded holdover, a conservative engineering model is `B(t)=B0 + drift_bound*elapsed + conversion_bound + timestamp_bound`, with all terms expressed in seconds. No supplied drift bound means no inferred safe holdover. Wall-clock rollback, device reboot, stale time, unknown GST week or reception after disclosure produces unavailable/not-safe evidence, even when a MAC happens to match.

OSNMA-authenticated navigation does not authenticate the receiver's timing solution (O2 Annex C); using that solution to narrow B without independent evidence is circular. G2 TDS is likewise not an independent cryptographically trusted clock.

### 4. TESLA and tag calculation

O1 §§5.8.2/6.4: normal tags in a MACK use a key in the **next** MACK. ADKD12 has **10 additional subframes** of disclosure delay, not a separate instant verification mode. Do not merge tag broadcast opportunity, disclosed-key receipt and authenticated-fix readiness.

```text
K_i = trunc_MSB(l_k, H_chain(K_(i+1) || GST_SF,i || alpha))
i = (GST_SF,i - GST0)/30 + 1
```

The GST value used for the key-chain hash has O1's precise E1 subframe-start-minus-one-second convention; root evaluation uses `GST0−30`. Tag GST uses the tag subframe convention. Do not reuse one unlabeled timestamp for both. Verify a disclosed key backward against the signed root or an already authenticated earlier key from the **same chain**; cap hash work and cache authenticated anchors.

O1 §6.7:

```text
tag = trunc_MSB(l_t, MAC(K,
  PRND || PRNA || GST_SF || CTR || NMAS || navdata || byte_padding))
Tag0 = trunc_MSB(l_t, MAC(K,
  PRNA || GST_SF || CTR || NMAS || navdata || byte_padding))
```

PRNA is the transmitting authentication satellite; PRND is the satellite whose navigation is being authenticated. They can differ. CTR is 8-bit and position-specific; Tag0 has CTR=1. `navdata` is the exact concatenation selected by **ADKD/COP and O1 Annex B**, not a serialization of floating-point ephemeris fields or whatever word values are currently cached.

- ADKD0 and ADKD12 cover defined ephemeris/clock/status data from words 1–5.
- ADKD4 covers specified timing parameters, not a substitute ephemeris signature.
- Word 16 reduced CED and arbitrary imported RINEX data are not automatically ADKD0-authenticated.
- COP=0 dummy tags can be checked as defined, but do not authenticate user navigation.
- Flexible tags require verified MACSEQ over PRNA/GST and the ordered flexible Tag-Info fields. MACLT fixed entries must also be enforced. Reserved tags never authorize navigation.

Tag accumulation follows O1 §5.8.1: minimum equivalent length `L_min` requires `l_t*N_t ≥ L_min` only for admissible distinct tags tied to the **same PRND, ADKD and exact navigation data/COP applicability**. Repeated copies are not independent evidence. With L=40 independent equivalent bits, the ideal single-attempt guessing probability is `2^-40 ≈ 9.09e-13`; it is not a system-wide spoofing probability or an excuse to ignore trust/time/replay conditions.

## Proposed state and policy

Maintain per-navigation-version evidence keyed by constellation, satellite, source, full issue/epoch and byte hash. Suggested outcomes are `unavailable`, `collecting`, `waiting_for_key`, `verified`, `failed`, `expired`, `revoked`, `unsupported` and `time_unsafe`, with exact reason and evidence times. These are proposed sidecar states, not additions to legacy model enums.

“Verified” requires trusted root/key, valid signature/chain, safe reception time, exact authenticated data/tag binding, accepted service status, sufficient equivalent tag bits and non-expired issue. Authentication evidence is immutable; later revocation updates eligibility without rewriting old historical facts. A new IOD with no proof starts unauthenticated even if the previous issue was verified.

Default companion reporting is **observe-only**. An explicitly requested authenticated-navigation fix selects only eligible Galileo ephemeris snapshots and uses a separate estimator/session. It cannot silently prune the legacy solve, label unverified other constellations authenticated, or relax geometry/quality requirements to force a fix. A host choosing mixed authenticated Galileo and unauthenticated GPS must receive explicit per-satellite provenance and an overall **mixed**, not fully authenticated, label.

## Slices and acceptance

All proposed test classes use the parent's focused Gradle commands. O-1…O-5 add evidence, O-6 adds an explicit policy, O-7 establishes actual security/latency behaviour; none intrinsically improves honest code accuracy.

### O-1 Full-page extraction

- **Deliverable:** `OsnmaPageExtractor`, original-byte bounded navigation history and Android E1-B provenance mapper.
- **Implementation:** offsets/table above, independent CRC, full-page copy, timed 15-page subframes and DSM/HKROOT/MACK extraction before any word-type reduction. Keep legacy rebuilt-parity/health handling untouched.
- **Acceptance:** proposed `OsnmaPageExtractorTest` and `GalileoOsnmaPageAdapterTest`: golden Android page bytes; OSNMA flip fails CRC; SSP-only flip does not fail CRC; malformed halves/alert/dummy/E5b/unknown-component cannot authenticate; 1–15 page completeness/reorder/dropout and duplicate conflicts. The exact same pages still give unchanged legacy ephemerides/solve output, including existing Galileo parser tests.

### O-2 Trust, crypto and time

- **Deliverable:** trusted-material and clock-interval contracts, supported-primitive provider adapter and algorithm self-tests.
- **Implementation:** exact bit/DER conversion and independent known-answer crypto vectors; no silent unsupported-algorithm fallback; host-supplied uncertainty/holdover.
- **Acceptance:** proposed `OsnmaCryptoProviderTest` and `OsnmaTrustedTimeTest`: P-256/P-521 signatures, invalid points/signatures, hash/MAC KATs, non-byte-aligned truncation; B just below/equal/above 15 s and 165 s; drift, stale anchor, wall rollback/reboot, callback latency and late replay. JVM and supported Android provider matrix must all pass or that provider is explicitly unsupported. Unsafe time never returns verified.

### O-3 Public key and root verification

- **Deliverable:** bounded DSM-PKR/KROOT assembly, trusted Merkle path and signed root validation.
- **Implementation:** block IDs/lengths, root/public-key IDs, exact signature message, padding and applicability; isolate histories across sources/chain versions.
- **Acceptance:** proposed `OsnmaDsmVerificationTest`: official nominal/renewal fixtures, every Merkle-path direction, tampered branch/root/alpha/signature/reserved/padding, wrong PKID/CID, missing/duplicate/conflicting blocks and excessive work. A self-signed attacker-provided root/public key never establishes trust. No partial DSM changes active verified chain state.

### O-4 TESLA and navigation tags

- **Deliverable:** authenticated disclosed-key cache, MACK/MACLT/MACSEQ handling and exact ADKD0/4/12 tag verifier.
- **Implementation:** formulas/conventions above, COP applicability, cross-authentication, Tag0 special form and admissible equivalent-bit accumulation. Preserve navigation histories across normal update boundaries; never reinterpret current cache as historical proof.
- **Acceptance:** proposed `OsnmaTagVerificationTest`: official tag results; key hash timestamp/index one-step error; ADKD12 extra delay; wrong PRNA/PRND/CTR/COP/ADKD/NMAS/nav bit; dummy and flexible tags; stale data and late-received matching forged tags. Byte-exact Annex B concatenation checked independently. Repeated identical tags cannot reach the security threshold. Correct MAC with unsafe time remains unverified.

### O-5 Lifecycle and evidence

- **Deliverable:** status machine for chain/public-key renewal and revocation, Merkle renewal, alert, expiry and per-issue evidence.
- **Implementation:** verified transition authority; quarantine unverified status changes; conservative temporary unavailability where required, not permanent attacker-triggered deletion of trusted material. Version/epoch binding and bounded histories; independent session reset.
- **Acceptance:** proposed `OsnmaLifecycleTest`: every official Annex B lifecycle trace, including both alert steps; verified revocation removes eligibility; unauthenticated flags cannot install a root/key or permanently revoke one; rollover cannot resurrect old proof; new imported/OTA issue not inherited-authenticated. Companion reset/probe leaves all old engine stores and lastSolution unchanged.

### O-6 Separate fix policy

- **Deliverable:** explicitly invoked `AuthenticatedNavigationFixPolicy` and sidecar result recording eligible/excluded ephemerides and mixed-constellation status.
- **Implementation:** require current proof for the exact navigation record used at transmit time; run a separate pure solve or reject unavailable geometry. No automatic “suspected spoof” flag merely for missing OSNMA pages.
- **Acceptance:** proposed `AuthenticatedNavigationFixPolicyTest`: verified/unverified/revoked mixed SVs, no geometry, single-SV constellation, stale IOD, forged imported record, observe-only and strict calls. Strict cannot report a fix with insufficient rank or label GPS/HAS/ranges authenticated. Legacy `computePosition`, weighting, diagnostics and RTK results are identical regardless of companion evidence.

### O-7 Conformance and performance

- **Deliverable:** reproducible full official-vector replay, private offline negative tests and TTFA/TTFAF report for cold/warm/hot trust starts.
- **Implementation:** obtain [official vector archive](https://www.gsc-europa.eu/sites/default/files/sites/all/files/Test_vectors.zip) via O4/O2 Annex B, verify license/hash/metadata and correct preloaded trust/time for each scenario. No harmful RF transmissions are needed.
- **Acceptance:** proposed `OsnmaOfficialVectorReplayTest`: all documented nominal/renewal/revocation/alert outcomes and timestamps agree with the reference; no skipped lifecycle scenario described as pass. Measure verified-nav age, first verified issue, first policy-eligible fix, false-positive/unavailable rates, queue delay and memory under malformed streams. O3 availability intervals are not used as guaranteed receiver TTFA. Real-device component/provenance/time support remains a release prerequisite.

## Security and performance limits

- A valid OSNMA tag does not prove a measured range is honest. Delayed retransmission/meaconing or local receiver compromise can still affect the fix within the threat model.
- OSNMA-authenticated I/NAV does not authenticate HAS messages. Keep correction-source provenance separate.
- Strict policy can **increase** time to an eligible fix and reduce satellite availability; report that cost beside the resilience benefit.
- Root/signature verification and long chains must be bounded/off the legacy callback lock. Record unsupported inputs instead of falling back to an insecure acceptance path.
- Material renewal/alert recovery needs host trust refresh, not a hardcoded URL fetch in core. No trusted-time capability means unavailable authentication, not simulated success.

O1/O2/O3 numerical/protocol definitions are © European Union, 2023–2025. This is non-normative implementation analysis; follow the source documents and their redistribution terms.
