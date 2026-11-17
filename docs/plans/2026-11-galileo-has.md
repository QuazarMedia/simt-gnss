# Galileo HAS: decoding, application and sequential positioning

Parent: [canonical tracker and sources](2026-11-galileo-capabilities.md). Gains and cross-feature gates: [validation study](2026-11-galileo-gains-validation.md). **All work is proposed; no implementation exists from this plan.**

## Scope and architecture

HAS supplies corrections for **Galileo and GPS**. E6-B SIS and Internet Data Distribution are alternative delivery paths, not interchangeable wire protocols. H5 lists the IDD ICD as registration-controlled; do not guess its framing or treat its stream as ordinary RTCM SSR.

Proposed new pure-JVM package: `app.simt.gnss.core.galileo.has`. Proposed components: `HasPageDecoder`, `HasMessageAssembler`, `HasMt1Decoder`, `HasCorrectionStore`, `HasCorrectionApplicator`, `HasCodePositionSolver`, `HasPppFilter`. Proposed Android companion: `GalileoHasObservationAdapter`. These are new names, not existing source files.

Inputs are immutable decoded pages or explicit host-supplied correction records, raw per-signal epochs, reference-navigation records and antenna/environment metadata. Host record injection must preserve the same issue, time, signal, service-status and completeness gates as SIS decoding; it cannot bypass H-4/H-5.

Keep this store separate from `SsrCorrections.Store`. Existing RTCM message decoding, SSR freshness, provider pairing, correction weights, SBAS, absolute WLS and RTK are not modified. No full-service atmospheric format, HAS data authentication or PPP ambiguity resolution is assumed by this plan.

## Calculation contract

### 1. Page framing and HPVRS

H1 §§2–3: a decoded HAS page contains **448 bits = 56 bytes**, comprising a 24-bit header and 424-bit/53-byte encoded body. Header order is `HASS:2, reserved:2, MT:2, MID:5, MS:5, PID:8`; information-page count `k = MS + 1`, 1…32. PID is 1…255; zero is reserved. `AF3BC3` is a dummy header and must be discarded before semantic interpretation.

A normalized page is accepted only with valid upstream E6-B C/NAV framing/FEC and 24-bit CRC evidence. A 56-byte payload alone cannot establish its CRC: the surrounding C/NAV page is needed. H-1 accepts already-decoded validated pages; a raw C/NAV adapter requires independently verified framing and is not a Kotlin RF demodulator.

H1 §6: field arithmetic is GF(256), with primitive polynomial

```text
p(x) = x^8 + x^4 + x^3 + x^2 + 1  (0x11D)
addition = XOR; multiplication/inversion are field operations
RS(255,32,224), systematic generator G of size 255 × 32
D[i,j] = G[PID_i - 1,j], j = 0…k-1
M[:,q] = inverse_GF(D) × C[:,q], q = 0…52
```

Use **k distinct valid transmitted PIDs**, including parity-only subsets; do not concatenate received pages. For shortened messages, the last 32−k information rows are zero; zero-only systematic PIDs k+1…32 are not valid useful transmitted rows. Invert once for all 53 columns. Preserve H1 Annex B's row/column and symbol order. Annex C is the independent worked oracle.

Assembly is keyed by delivery-source identity, MT, MID, k and an assembly generation/first-reception lifecycle. PIDs from multiple transmitting satellites may belong to the same SIS message; do not partition solely by transmitting SVID. Conversely, mixing unverified independent delivery sources is forbidden. Duplicate PIDs add no rank; conflicting duplicate contents invalidate that assembly. After **150 seconds** incomplete MID accumulation is discarded. Never let duplicate traffic extend this deadline.

Decode work is roughly `O(k³ + 53k²)` field operations; k=32 gives about 32,768 + 54,272 leading-order operations, not a JVM timing guarantee. Bound inversion caching and pending-message memory. CRC/RS detect channel corruption, **not malicious correction provenance**.

### 2. MT1 masks, units and association

H1 §5: MT1 header is 32 bits: `TOH:12`, six ordered block-presence bits, reserved:4, Mask ID:5, IOD Set ID:5. TOH is 0…3599 seconds in GST. Decode blocks in the order **mask, orbit, full-set clock, subset clock, code bias, phase bias**. Satellite, signal and cell masks determine subsequent row counts and ordering.

Retain exact constellation, satellite, Galileo/GPS signal ID and **reference navigation message** from the mask. A carrier frequency does not identify E1-B versus E1-C, E5a-I versus E5a-Q, or a reference clock. Reject use when the receiver cannot identify a matching HAS signal. Unknown future trailing information is not fabricated into known blocks; H1 §5.1 allows extension data after the known content.

| Field | Signed bits / scale | Special values and meaning |
|---|---|---|
| Reference IOD | GPS 8 bits; Galileo 10 bits | Match GPS IODE/IODC relationship or Galileo IODnav and navigation-message reference |
| Orbit radial | 13 / 0.0025 m | Most-negative bit pattern = unavailable |
| Orbit in/cross-track | 12 / 0.008 m | Most-negative bit pattern = unavailable |
| Clock DCC | 13 / 0.0025 m | Most-negative = unavailable; `0111111111111` = satellite shall not be used |
| Clock DCM | 2-bit unsigned | 00→1, 01→2, 10→3, 11→4; NOT powers of two |
| Code bias | 11 / 0.02 m | Most-negative = unavailable |
| Phase bias | 11 / 0.01 cycles | Most-negative = unavailable; convert cycles using the exact signal wavelength |
| Phase discontinuity | 2-bit counter | Any observed change resets that signal's ambiguity; missed wraps remain a continuity risk |

Recognise sentinels **before** signed scaling. H1 VI index 0…14 maps to `[5,10,15,20,30,60,90,120,180,240,300,600,900,1800,3600]` seconds; 15 is reserved. This is a **validity interval**, not an SSR polynomial update interval.

H1 §7.7 reconstructs reference time using **complete-message GST reception time** `t_r`:

```text
h = floor(t_r / 3600)
candidate = 3600*h + TOH
t_MT1 = candidate if candidate <= t_r else candidate - 3600
```

Preserve full week/epoch information and explicitly convert GST/GPST; aligned timescales are not permission to equate their week numbering or ignore offset uncertainty. A bounded receiver-time interval that gives multiple possible hour assignments must not produce an arbitrarily tight-validity correction.

H1 §7.6: clock/bias-only MT1s require previously defined matching `(Mask ID, IOD Set ID)` and reference IODs. One pair has at most one definition in a 30-minute window; rollovers are possible afterwards. Retain definition-generation time and expire associations conservatively; equality of short IDs alone never revives historical data. Each component has its own epoch and VI. Use an atomic snapshot with all **required** components valid at the observation/transmission evaluation epoch, rather than demanding equal component epochs or averaging sources.

### 3. HAS-specific correction application

H1 §7 uses **ECEF broadcast velocity**, not an inertial velocity and not automatically `r/|r|` as radial direction:

```text
e_t = v / |v|
e_w = (r × v) / |r × v|
e_n = e_t × e_w
r_HAS = r_BC + e_n*DR + e_t*DIT + e_w*DCT
δC_m = DCC_m * DCM
δt_HAS = δt_BC_polynomial + Δt_rel + δC_m/c
Δt_rel = -2(r_BC · v_BC)/c²
P_corrected = P_raw + CB_m
L_corrected_m = L_raw_m + λ*PB_cycles
```

`c=299792458 m/s`. HAS **adds** orbit deltas. The existing RTCM applicator **subtracts** them and has different age/rate semantics: do not reuse it by passing nominally identical deltas. No HAS orbit/clock-rate polynomial is invented from the current SSR model. Do not count relativity twice when the broadcast clock helper already includes it.

H1 corrections refer to the **ionosphere-free satellite antenna phase centre of the mask's reference navigation-message signals** and GTRF. They are not unlabelled COM corrections. Either remain explicitly in that reference APC and derive per-signal PCO consistently, or transform to COM with official PCO/attitude data and apply signal-specific PCO/PCV once. Mixing the two formulations double-counts centimetre-to-metre antenna offsets.

HAS code biases **replace** broadcast BGD/TGD for corrected observations. Do not stack them with existing IF/group-delay preprocessing. Keep inter-system receiver/common clock offsets as estimable nuisance terms, particularly for HAS GPS clocks. If the selected Galileo clock reference is F/NAV but only I/NAV records are available, reject that corrected path; never silently reinterpret `galileoInavClock` or retrofit F/NAV into the old parser.

### 4. Code versus sequential phase positioning

For new corrected code rows, with corrected satellite clock in metres:

```text
P_c = ρ + b_constellation - c*δt_HAS + T + μ_f*I + receiver_bias + ε_P
μ_f = (f_E1/f)²
P_IF = α*P_1 + β*P_2
α = f_1²/(f_1²-f_2²); β = -f_2²/(f_1²-f_2²)
Var(P_IF) = α²σ_1² + β²σ_2² + 2αβ Cov(P_1,P_2)
```

Evaluate transmit time and Earth rotation/Sagnac consistently with the new orbit, include Shapiro, and preserve realistic code noise. Reject incomplete bias sets rather than using HAS clocks with incompatible uncorrected code. Orbit/clock correction improves systematic error; it does not make smartphone code noise 0.2 m.

For **uncombined sequential float PPP**, use corrected code and carrier in metres:

```text
P_c = ρ + b - c*δt_HAS + T + μ_f*I + antenna_P + receiver_code_bias + ε_P
L_c = ρ + b - c*δt_HAS + T - μ_f*I + λ*N_float + antenna_L + windup_m + ε_L
range_rate = LOS · (v_sat - v_rx) + receiver_clock_drift - sat_clock_drift
```

Use the actual signed LOS projection for Doppler; a vector-speed magnitude is not range rate. The equations show opposite first-order ionosphere signs and ambiguities carried across epochs. Code already smoothed with these phases is correlated and cannot be treated as independent raw code.

Recommended reference-compatible formulation follows H4/H2: satellite-satellite single differences **within each constellation and signal**, uncombined EKF and float ambiguities. A candidate state is `[r(3),v(3),a(3),ZWD, ΔI_s, B_s,f]`; B is a floated metre ambiguity including constant differential phase nuisance biases. State dimension changes with satellites/signals. Receiver clock and stable same-signal receiver biases cancel in SD; inter-system differencing is not performed. Document the datum and rank before implementation; do not include redundant undifferenced clock/bias states.

Let D difference each satellite against the constellation reference, then `z_SD=D*z` and **`R_SD=D*R*Dᵀ`**, including off-diagonal shared-reference noise. Wet-delay row is `m_w(s)−m_w(ref)`, and first-order ionosphere is `±μ_f ΔI_s`. When the reference changes, transform ionosphere/ambiguity states and covariance with `x'=T*x, P'=T*P*Tᵀ`; do not recycle old values under new keys.

```text
x_minus = F*x; P_minus = F*P*Fᵀ + Q
K = P_minus*Hᵀ * solve(H*P_minus*Hᵀ + R)
x_plus = x_minus + K*(z-h(x_minus))
P_plus = (I-KH)*P_minus*(I-KH)ᵀ + K*R*Kᵀ
```

Use numerical solves, not explicit floating-point inversion; the GF inversion above is a separate algebraic operation. Joseph or square-root covariance update preserves symmetry/PSD. Tune process noise from recorded dynamics, not from desired advertised accuracy. Missing phase biases may be absorbed as floated stable nuisance ambiguities **only with demonstrated continuity and compatible reference-clock/signal conventions**; otherwise withhold phase positioning. Phase-bias availability never implies PPP-AR is authorised or valid.

## Implementation slices and acceptance

Every proposed test below runs with the parent's focused command convention; fixture provenance and legacy gates are mandatory. Benefits refer to the gain study: H-1…H-5 enable correction delivery, H-6 can lower systematic code error, H-7/H-8 enable sustained phase accuracy, H-9 establishes actual gains.

### H-1 Page boundary and header

- **Deliverable:** `HasPageDecoder` accepting an immutable 56-byte validated HAS payload and explicit source/SVID/reception/CRC metadata; typed status and unsupported-message outcomes.
- **Implementation:** dummy detection, exact field widths/MS+1, CRC provenance requirement; operational/test/reserved/Don't-Use distinction. Test data remains diagnostic-only unless explicitly authorised as test input.
- **Acceptance:** proposed `HasPageDecoderTest`: all header extrema, MS=0/31, PID=0/1/255, 55/57-byte inputs, dummy page and reserved MT/status; independent ICD-derived bytes. Don't-Use invalidates only HAS state and never SSR. Invalid input produces no store mutation. A-0 remains green.

### H-2 HPVRS reconstruction

- **Deliverable:** `HasMessageAssembler`, GF operations and generator-matrix-backed reconstruction.
- **Implementation:** immutable bounded accumulators, source-coherent cross-satellite collection, one inversion per PID subset; 150 s deadline measured by monotonic time.
- **Acceptance:** proposed `HasMessageAssemblerTest`: Annex C byte-exact decoded message; k=1/32 and parity-only/mixed subsets; fewer than k pages cannot decode; duplicate/conflicting PID, forbidden zero rows, reordering, MID reuse, k change and timeout edges. Allocation remains under a declared cap during sustained malformed input. Fixture must not be generated solely by the decoder's own offsets/matrix code.

### H-3 MT1 layout and masks

- **Deliverable:** exact MT1 block models with unscaled sentinel information, NAV reference, signal identities, cell masks and unsupported-extension outcome.
- **Implementation:** length-bounded bit cursor, mask-driven loops, full/subset clock mapping, DCM=1…4, correct bias units/PDI. Reserved extension information may remain opaque after known valid blocks.
- **Acceptance:** proposed `HasMt1DecoderTest`: independent GPS/Galileo fixtures for every block, sparse cell masks, subset clocks, zero/max populations, signed extrema, unavailable/Don't-Use sentinels, truncated blocks and future tails. One signal cannot receive its neighbour's bias. No partial malformed message is committed.

### H-4 Correction store and validity

- **Deliverable:** `HasCorrectionStore` with immutable per-epoch bundles and separate bounded navigation-IOD history supplied by the host.
- **Implementation:** reference time from complete reception; full epochs; conservative use interval `[t_MT1,t_MT1+VI)`; exact issue/message/APC binding; distinct source namespaces; never resurrect rollover definitions. Different component epochs are allowed if all applicable.
- **Acceptance:** proposed `HasCorrectionStoreTest`: hour/week rollovers, unknown GST, future/expired components, every VI/reserved VI, absent mask/orbit definitions, mismatched IOD, stale/navigation-source mismatch, duplicate clocks and 30-minute ID reuse. Reader sees either complete old or complete new bundle; not mixed components. Test/reset cannot clear another provider or any legacy store.

### H-5 Correction application

- **Deliverable:** `HasCorrectionApplicator` returning corrected orbit/clock and per-signal observation adjustments with explicit reference metadata.
- **Implementation:** equations above; no SSR midpoint/rates; BGD/TGD replacement; APC/COM/relativity applied exactly once. Missing required code bias/reference navigation blocks use, not silent substitution.
- **Acceptance:** proposed `HasCorrectionApplicatorTest`: synthetic non-circular ECEF orbit distinguishes e_n from r-hat; +1 m radial gives +e_n; clock DCM=3 does not become 4; +0.2 m code bias adds +0.2 m; +0.1 cycle phase adds +0.1λ. Compare independent reference values within 1 mm orbit/range and 0.001/c seconds clock after matching conventions; this is a numerical target, not an accuracy promise. Existing `SsrCorrectionApplicatorTest` is unchanged and passes.

### H-6 Corrected code positioning

- **Deliverable:** explicitly requested `HasCodePositionSolver`; separate result metadata reports `HAS_CODE`, not decimetre PPP or RTK fixed.
- **Implementation:** pure numerical solver reuse only where immutable/stateless; its own rows, clock nuisance groups, IF covariance, residual tests and seed. Source selection is explicit and atomic; no automatic HAS/SSR blend. Legacy output is never overwritten.
- **Acceptance:** proposed `HasCodePositionSolverTest`: noiseless known-position corrections remove injected orbit/clock/bias errors to numerical tolerance; mismatched/missing bias sets are excluded with reasons; insufficient geometry returns unavailable; realistic phone noise does not collapse covariance. Legacy solve/store/lastSolution remain identical with companion success, failure and probe. Existing group-delay/SSR/raw-epoch tests stay green.

### H-7 Observation and host adapters

- **Deliverable:** uncombined original-code/ADR/Doppler epoch model and Android mapper; normalized host SIS/IDD input contract, not a new networking stack.
- **Implementation:** exact code identity or unavailable; receiver/hardware clock discontinuity, ADR validity/half-cycle/slip, per-signal wavelength, observation uncertainties and reception timing. Original code, smoothed code and IF code have distinct provenance; PPP uses original uncombined code.
- **Acceptance:** proposed core `HasObservationEpochTest` and Android `GalileoHasObservationAdapterTest`: both Galileo E1/E5a and E1/E5b, GPS pairing, dropped signal, ADR reset, half-cycle ambiguity, hardware-clock step, missing code identity and duplicate epochs. Mapped phase sign agrees with independent Doppler/ADR motion. IDD wire adapter remains **blocked until H5's registered ICD and access are obtained**; normalized offline input can ship without it.

### H-8 Sequential float PPP

- **Deliverable:** `HasPppFilter`, separated raw-observation preprocessing/atmosphere/antenna modelling, dynamic SD state and convergence evidence.
- **Implementation:** seed from corrected IF code; carried ambiguities/ZWD/ionosphere; correlated SD covariance; pivot transforms; cycle slips/clock jumps/outlier exclusion; official antenna metadata and attitude/phase wind-up. Invalid/missing antenna calibration is explicitly lower-grade, not silently calibrated. No fake NeQuick approximation or reuse of RTK LAMBDA for PPP fixing.
- **Acceptance:** proposed `HasPppFilterTest`: finite-difference Jacobians, PSD covariance, observable-state rank, single-epoch phase-with-free-ambiguity gives no false position information, unbiased long replay, multipath/outlier, missing reference, dynamic pivot, signal slip/PDI change/epoch gap. Independent noiseless replay stays within 1 cm after burn-in and does not falsely improve without valid observations. Innovation/NIS and empirical covariance calibration must pass predeclared statistical tests. No modification of legacy PppState, Kalman or RTK ambiguity memory.

### H-9 Reference and device qualification

- **Deliverable:** reproducible HAS-UA/oracle comparison plus separate geodetic and Android reports, component-availability timeline and time-to-accuracy curves.
- **Implementation:** record service status, actual phase-bias coverage, ephemeris source/issue, antenna/reference frame, initialization and resets; compare like modes and epochs, not published best-case numbers against an unrelated phone trace.
- **Acceptance:** proposed `HasReferenceReplayTest`: decoded bytes match oracle; correction-level differences satisfy H-5 tolerance; estimator outputs meet the predeclared reference-mode error envelope and do not diverge or understate uncertainty. Field qualification follows V-2. The H3 <20 cm/<40 cm P95 and <300 s/<100 s convergence values are **full-service targets**, not mandatory phone acceptance bars or guaranteed Initial Service results. Without suitable recordings/hardware, release claims remain blocked even if unit tests pass.

## Explicit non-goals and unresolved dependencies

- No production RF demodulator, receiver firmware commands, IDD credential storage or transport in this repository.
- No inference that OSNMA authenticates HAS corrections: O1 authenticates specified I/NAV data, not E6-B correction contents. Trusted Internet transport establishes transport provenance, not an equivalent SIS signature.
- No unconditional 0.2 m weighting, hidden broadcast fallback, PPP-AR or use of phase after an unexplained discontinuity.
- H4 permits older reference-navigation issues than the legacy freshness policy; any such extension belongs only in the new HAS path, with full-calendar/IOD validation and independently qualified propagation. Do not loosen the existing ephemeris store.
- Obtain registered IDD documentation and reference-software/data permissions before implementing those external formats or vendoring fixtures. Confirm current phase-bias service provision and future authenticated/atmospheric extensions with live data and the current in-force ICD.

Source: Galileo High Accuracy Service Reference User Algorithm - Technical Note © European Union, 2025. H1/H2/H3 and their numerical definitions are © European Union, 2022–2025; see the parent source register and each document's terms. Proposed software design and illustrative error models here are non-normative engineering analysis.
