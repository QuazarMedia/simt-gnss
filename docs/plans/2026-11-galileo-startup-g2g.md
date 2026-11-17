# Galileo startup: G1 E5a Quasi-Pilot and G2G

Parent: [canonical slice tracker and sources](2026-11-galileo-capabilities.md). Numerical startup/error-budget study: [gains and validation](2026-11-galileo-gains-validation.md).

## Distinguish three different accelerators

1. **G1 E5a-QP:** shorter-code acquisition at 1176.450 MHz, described in OS SIS ICD 2.2. It has no G2 Time Determination Symbol.
2. **Existing-generation I/NAV improvements:** SSP, reduced CED word 16 and FEC2 words 17–20 can separately improve navigation-data acquisition/recovery. They are not E5a-QP and are **not implementation scope here**; any later plan needs its own decoder, quality/OSNMA coverage study and approval.
3. **G2 QP/TDS:** provisional new signals and coarse-time aid in early G2 interface information. Not the same code/rate/frequency as G1 QP, nor a complete future navigation-message ICD.

`simt-gnss` processes already-acquired observations/navigation pages. There is no sample-level RF acquisition/tracking subsystem or generic Android API for enabling QP correlators. Library-only work can report observed capabilities, validate explicit externally supplied signal/time evidence and benchmark startup; it cannot promise acquisition gains on an unsupported phone.

## G1 E5a-QP: verified parameters and calculations

Q1 §2.3.1.4, §3.1 Table 15, §3.4.2 Table 21 and §3.6.1 specify:

| Property | G1 E5a-QP |
|---|---|
| Carrier | 1176.450 MHz |
| Modulation / chip rate | BPSK / **5.115 Mchip/s** |
| Primary code | 330 chips; 40 memory-code family members |
| Repetition | 31 repetitions in 2 ms; no overlay/secondary code |
| Primary period | `330 / 5.115e6 = 64.516129 µs = 2/31 ms` |
| Minimum/maximum received power in Q1 | −160.75 / −155.50 dBW, under ICD conditions |

Construct each primary code using the three published short codes:

```text
c[n] = a15[n mod 15] XOR b11[n mod 11] XOR d10[n mod 10], n=0…329
330 = lcm(15,11,10)
```

Take time-ordered significant bits according to the ICD's hexadecimal convention; trailing padded hexadecimal bits are not chips. Compare the complete primary sequence with **independent Annex C.9 memory-code values**, not only the same generator run twice. Use the published **G1** code-to-satellite assignment; never infer a GSAT name's SVID or borrow G2's 50-code table.

The short code creates range periodicity `c*T_code ≈ 19,341.45 m`. QP acquisition alone does **not** provide an unambiguous satellite transmit time/pseudorange. Aid, longer-code handoff and normal navigation/time lock remain necessary. Existing code-lock/TOW-known/decoded and millisecond-ambiguity/ADR/half-cycle quality gates remain untouched.

Q2 reports typical **8× lower acquisition complexity** and **3× faster acquisition**, relative to other E5 components. These are programme-level typical comparisons, not a derivable 31× total speedup from 10230/330, nor universal device guarantees. Lower chip rate and shorter hypotheses help acquisition; handoff, oscillator search, hardware parallelism and time/navigation readiness determine total TTFF.

A laboratory acquisition study can use matched filtering over delay/Doppler hypotheses:

```text
R(tau,nu) = Σ y[n] * c[n-tau] * exp(-j*2*pi*nu*n/Fs)
FFT acquisition cost ≈ N_Doppler * N_PRN * N_FFT*log2(N_FFT) * N_integrations
```

Do not prescribe a universal C/N0 or peak threshold without a receiver/detection model. Short codes have cross-correlation/false-lock tradeoffs; record detection/false-alarm probability, search range, sample filtering, integration and power. Fewer operations alone is not evidence of a correctly timed fix.

## G1 slices

### Q-1 G1 capability and telemetry

- **Deliverable:** independent exact signal identity `G1_E5A_QP`, capability/version evidence and acquisition/navigation/first-fix timestamp model.
- **Implementation:** distinguish declared QP support from observed acquisition; use host-provided current metadata/NAGUs and documented G1 code assignments. Q2 lists 12 reconfigured satellites at study time, but visibility/health/support varies. Do not hardcode that count into selection.
- **Acceptance:** proposed `GalileoStartupCapabilityTest`: legacy E5a-Q/I not labelled QP merely by frequency; unknown/unsupported receiver and stale metadata remain explicit; GSAT/SVID identity is independently verified; telemetry uses monotonic intervals and reports missing events as unknown. Legacy `GnssSignalBand`, `isL5`, ephemeris and solve outputs remain unchanged.
- **Gain:** reliable observability, not faster acquisition by itself.

### Q-2 G1 reference codes and acquisition

- **Deliverable:** offline reference code generator and bounded acquisition experiment in new pure-JVM code/test support; no automatic SDR pipeline or receiver command.
- **Implementation:** 40 complete 330-chip sequences and 31-fold repetition; match independent Annex C.9; model specified sample/chip rate and declared hypotheses. Any high-throughput receiver DSP implementation belongs in hardware/firmware integration, not the legacy processor.
- **Acceptance:** proposed `GalileoG1QuasiPilotCodeTest`: all independent code sequences match bit-for-bit; period, padding and assignments match Q1; real or independent synthetic offline I/Q fixtures locate delay/Doppler within the declared bin resolution; noise-only, wrong-code and cross-correlation cases cannot pass unqualified. Repeat timing experiments at equal detection conditions; do not claim 8×/3× from code length alone.
- **Gain:** conformance/reference readiness; actual gains require Q-3/Q-4.

### Q-3 G1 receiver handoff BLOCKED

- **Unblock with:** a documented QP-capable firmware/receiver interface or an authorised SDR receive-only acquisition/tracking pipeline, independently validated sample rate/clock and code assignments.
- **Deliverable after unblocking:** receiver-specific external handoff and time-evidence adapter, not changes to generic Android quality gates.
- **Implementation after unblocking:** resolve QP short-code ambiguity using qualified navigation/assistance/long-code lock; record handoff success and clock uncertainty. Keep QP-derived acquisition evidence separate from OSNMA authentication and usable ranges.
- **Acceptance:** proposed `GalileoQuasiPilotHandoffTest`: ambiguous acquisition cannot become a usable range; false-lock, old aid, satellite change, no-navigation/TOW, oscillator error and failed handoff return unavailable; output agrees with independent unambiguous receiver timing. Legacy millisecond-ambiguity/code-lock/half-cycle tests retain every assertion. No harmful RF transmission is required.
- **Gain:** possible reduced receiver acquisition latency/energy, not guaranteed improved steady-state position accuracy.

### Q-4 G1 startup benchmark

- **Deliverable:** per-device cold/warm/hot and aided/unaided timing/energy report with legacy baseline; can be built before Q-3, but QP claims require a real qualified device comparison.
- **Implementation:** define reset semantics, firmware, time uncertainty, retained ephemerides, RF playback/sky conditions, enabled constellations and acquisition thresholds. Separate `first acquisition`, `usable time`, `full ephemeris`, `first fix`, `first authenticated fix` and `time to accuracy`.
- **Acceptance:** proposed `GalileoStartupTimelineTest` for timeline arithmetic plus Q-3 device recordings for benefits. At least 30 paired starts per mode/device/environment, median/P90 and confidence intervals, failed-start rate and joules-to-fix. QP must not increase false fixes or range ambiguity. Hardware without enable/disable control or equal conditions cannot support a causal QP speedup claim; mark observational evidence accordingly.
- **Gain:** measured acquisition/TTFF/energy benefit, conditional on the critical path.

## G2G: verified early interface versus future promises

G1 source here means the **G2 early-interface document identifier**, not the G1 generation. It is Issue 1.0, December 2025 and explicitly subject to change before a final OS ICD. G2G is a satellite/payload generation, **not a new constellation enum**. Legacy Galileo remains Galileo; new components require explicit identities and receiver support.

The early note gives:

| Provisional signal | Carrier MHz | Chip rate Mchip/s | Code |
|---|---|---|---|
| E1-QP1 | 1576.443 | 1.023, BPSK | 1023-chip Gold |
| E1-QP2 | 1597.926 | 1.023, BPSK; possible BOC(1,1) alternative | 1023-chip Gold |
| E5c-QP | 1182.588 | 1.023, BPSK | 1023-chip Gold |

E1-QP1 provisioning and E1-QP2 modulation remain provisional. Code-to-satellite assignment is **not supplied** in this note. E6 QP and new high-rate data components cannot be implemented from this note's QP parameters. The G1 1176.450 MHz E5a-QP is not intended on G2 satellites and will be superseded.

Existing frequency buckets could misclassify 1576.443 MHz as legacy E1 or 1182.588 MHz as legacy E5a/L5. Do not expand/rename old bands or use `isL5` for these signals. Proposed exact identities include frequency, component, modulation, primary-code definition, specification edition and measured receiver evidence; satellite generation is separate metadata.

### Provisional Gold-code reference

The note specifies 50 codes, length 1023, two 10-stage registers initialized to all ones. Feedback polynomials are octal **2415**, taps `[2,3,8,10]`, and **2443**, taps `[1,5,8,10]`; XOR register outputs after the published per-code register-2 shift. Define bit direction/output stage/shift convention from the referenced OS ICD and verify independently. Do not assign code numbers to satellites until official assignment is available.

`1023/1.023e6=1 ms`, a `c*1 ms≈299,792.458 m` code-range ambiguity. This is not G1's 64.516 µs primary period and does not provide a complete week/time solution.

### Provisional TDS and quasi-health

The early note's healthy overlay has a **240 ms Variable Transition Position symbol** with 20 states, j=0…19:

```text
first sub-symbol duration  = 40 + 4*j ms
second sub-symbol duration = 200 - 4*j ms
sum = 240 ms
table-associated time in 4.8 s cycle = 0.24*(j+1) s
```

Use the note's overlay polarity/start-of-week alignment and transition pattern when matching these states. Healthy TDS can resolve a **±2.4 s ambiguity in less than 248 ms** under the note's conditions. This is a coarse-time aid from one healthy acquired signal, **not TTFF <248 ms**, an absolute week determination, OSNMA time trust or a navigation-message decode.

Select an absolute candidate only when prior time uncertainty makes the 4.8 s-cycle solution unique; an interval crossing candidate boundaries can be ambiguous even if narrower than 4.8 s. With absent/broader prior, low SNR or uncertain transition/hypothesis, return unresolved. Propagate timing uncertainty rather than rounding it into a precise pseudorange.

The note's non-healthy QHS overlays contain no VTP:

| QHS | Meaning | Symbols / hexadecimal pattern |
|---|---|---|
| QHS1 | out of service | 64 / `FFFFFFFF00000000` |
| QHS2 | spare | 48 / `FFFFFF000000` |
| QHS3 | in test | 40 / `FFFFF00000` |

These patterns do not authorize a production fix. Unknown or non-healthy overlay is not coerced into a healthy TDS state.

## G2 slices

### G-1 G2 identity and readiness

- **Deliverable:** versioned early-interface capability models and satellite-generation metadata, with explicit provisional versus final/observed states.
- **Implementation:** component/frequency/modulation/code identity independent of existing band heuristics; do not assert G2 launches, active services or SISE improvement without current operational evidence.
- **Acceptance:** proposed `GalileoG2CapabilityTest`: three provisional frequencies cannot become legacy E1/E5a by bucket matching; Galileo constellation remains unchanged; unknown code assignment/new message/receiver support returns unavailable; early-edition metadata cannot mark production support. Legacy frequency/IF tests pass unchanged.
- **Gain:** safe readiness and classification; no fix improvement by itself.

### G-2 G2 code reference prototype

- **Deliverable:** laboratory-only 50-code Gold reference implementation, explicitly tagged with early-interface edition.
- **Implementation:** registers/taps/initial state and published shifts; independent expected chips/correlation checks; no invented SVID mapping.
- **Acceptance:** proposed `GalileoG2CodeReferenceTest`: all published shifts, 1023-chip length/1 ms period and independent code samples; distinguish G1 330-chip codes; unknown assignments cannot enter a production solve. Reference fixtures require licensing/provenance review, not a circular generator oracle.
- **Gain:** receiver/reference preparation, not live acquisition support.

### G-3 TDS and quasi-health prototype

- **Deliverable:** pure-JVM decoder of explicitly supplied overlay transition evidence and uncertainty; no raw Android correlation claim or new legacy pseudorange gate.
- **Implementation:** 20 VTP states, two durations, 4.8 s-cycle/prior disambiguation and QHS rejection; healthy component/code provenance is required. TDS is not trusted time for OSNMA.
- **Acceptance:** proposed `GalileoG2TdsTest`: all 20 states, partial symbols, start/week/4.8 s boundaries, valid and non-unique priors, transition jitter, competing peaks, every QHS pattern and unknown/test signal. Independent synthetic healthy examples meet the note's <248 ms time-resolution claim under its assumptions; no navigation/ephemeris means no fabricated fix. No untrusted TDS input can shrink O-2's trusted-time bound.
- **Gain:** demonstrated coarse-time ambiguity resolution in controlled conditions, not verified device TTFF.

### G-4 Final-ICD production integration BLOCKED

- **Unblock with:** in-force final signal/navigation ICDs, official code assignments, conformance vectors, service/deployment status and a capable receiver/firmware interface.
- **Deliverable after unblocking:** version-specific receiver adapter and any newly published navigation/time decoding, alongside complete legacy-mode compatibility.
- **Implementation after unblocking:** final signal/QHS/health, clock/orbit/message references and real measurement identity; revised formats use new decoders, not lenient fallback in the old parser. High-rate data, E6 QP and other G2 changes stay unavailable until specified.
- **Acceptance:** proposed `GalileoG2ProductionConformanceTest`: all final official vectors, early-versus-final parameter differences, unknown message/code/firmware, legacy-only and mixed G1/G2 replay; Q-4/V-2 field criteria. A laboratory TDS prototype is insufficient to mark this slice complete.
- **Gain:** actual deployment/receiver-specific startup/measurement benefit; magnitude remains to be measured.

## Changes software cannot implement

Digital/reconfigurable satellite payloads, inter-satellite links, improved clocks/navigation antennas, propulsion and ground control are space/ground infrastructure. Better satellite SISE may improve broadcast fixes without a new user algorithm, but only measured error budgets support a quantitative claim. HAS can already reduce some of the same orbit/clock error, so G2 and HAS gains are not independent or additive.

Q1 and the early G2 note's signal definitions are © European Union, 2025. Source editions and limits are listed in the parent register; reference experiments here are non-normative and production RF/hardware work remains explicitly gated.
