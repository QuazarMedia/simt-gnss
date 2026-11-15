# Large-File Refactor — simt-gnss (and simt-app companion)

**Date:** 2026-11-15
**Scope:** bring every source file over the 1000-line limit under it by extracting
cohesive files. Pure structural refactoring — no behavior, formula, constant, or
gating-logic changes (review-hardened code).

## Target invariants

1. **Behavior identity:** outputs, side effects, ordering, and error behavior stay
   byte-for-byte identical; no formula, constant, or gate is touched.
2. **Public API stability:** `SimtGnssEngine` façade, `RtkDoubleDifferenceSolver`
   signatures, and every cross-module public seam listed in the gnss-engine skill
   are unchanged.
3. **File-size limits:** no resulting file exceeds 1000 lines (`limits.fileLines.error`);
   the 400-line warning is a guideline for cohesion, not a hard split trigger.
4. **Green gate per slice:** `./gradlew build` (compile + `:gnss-core:test` +
   `:gnss-android:test` + lint) on the simt-gnss tree; `spotlessApply` applied before commit.

## Slice tracker

| id | lane | depends | title | state | evidence |
|----|------|---------|-------|-------|----------|
| 1 | processor-split | - | Split `GnssMeasurementProcessor.kt` (3,851 lines) into cohesive internal files | Todo | |
| 2 | rtk-split | 1 | Split `RtkDoubleDifferenceSolver.kt` (1,062 lines) into cohesive files | Todo | |
| 3 | test-split | 1 | Split `GnssMeasurementProcessorTest.kt` (2,506 lines) into per-area test files | Todo | |

Companion slice in `simt-app` (own tracker, `simt-app/docs/plans/2026-11-large-file-refactor.md`):
split `feature/gnss/.../data/location/GnssLocationServiceImpl.kt` (1,869 lines).

---

## Slice 1 — `GnssMeasurementProcessor.kt` (3,851 lines)

- **Module:** `:gnss-android`
- **Changes:** extract cohesive internal collaborators out of the processor
  (e.g. measurement-quality gates, pseudorange/clock-domain assembly, per-constellation
  ionosphere/group-delay application, ephemeris storage, nav-message dispatch,
  SBAS state) into `internal` files in the same package; the processor keeps
  orchestration, locking discipline, and public entry points.
- **Acceptance checks:**
  - `./gradlew build` green.
  - `GnssMeasurementProcessor.kt` under 1000 lines; no new file over 1000.
  - `git grep -c "fun computePosition\|fun processMeasurements\|fun processNavigationMessage" gnss-android/src/main` — public entry points unchanged.

## Slice 2 — `RtkDoubleDifferenceSolver.kt` (1,062 lines)

- **Module:** `:gnss-core`
- **Changes:** extract cohesive helpers (DD observable formation, float solution,
  fixed/LAMBDA path, covariance/prior handling) into files in the same package;
  the solver class keeps its public surface.
- **Acceptance checks:**
  - `./gradlew build` green (includes `RtkDoubleDifferenceSolverTest`,
    `RtkAmbiguityMemoryTest`).
  - No file over 1000 lines; public signatures unchanged.

## Slice 3 — `GnssMeasurementProcessorTest.kt` (2,506 lines)

- **Module:** `:gnss-android` (test source set)
- **Changes:** split into per-area test classes in the same package (raw-epoch
  assembly, group-delay conventions, SSR application, fallback/RAIM, smoothing
  integration), sharing a small `internal` fixture helper where the original
  file had duplicated setup. No test is added, removed, or weakened.
- **Acceptance checks:**
  - `./gradlew :gnss-android:test` green with the same test count as before the split.
  - No file over 1000 lines.
