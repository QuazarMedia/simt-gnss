<!-- simt:begin -->
<!-- simt:generated -->
<!-- simt:sha=e64c69c548517daaa897c8b18aa9f9903d22e20b -->
<!-- simt:sources=rules/core.md,rules/stacks/android.md,rules/stacks/jvm.md,rules/mdc/intake-questions.mdc,rules/mdc/slice-commit.mdc,rules/mdc/test-gate.mdc -->
<!-- Generated content. Do not edit by hand - your changes are overwritten.
     Run `simt sync --yes` to regenerate, or `simt check` to see what is stale. -->

## Core

Where a rule cites a decision, run `simt rules why ADR-NNN` to read it,
including whether it is still in force.

These rules are stack-agnostic. Whatever is specific to your stack, your product or your
commands belongs in this repository, not here.

### Truth

- Never claim a test you did not run, a check you did not execute, or a deploy you did not
  perform.
- Read the real runner manifest before running anything. A command absent from it is not a
  command. (Which file that is depends on the stack: `package.json` scripts, Gradle tasks,
  `pyproject.toml` entry points.)
- Report what you left alone: unrelated dirty files, skipped checks, unreviewed scope.

### Git

- Commit locally, report the SHA. **Push only on explicit approval.** (ADR-001)
- Before asking for push approval, state whether the push triggers a deployment, a migration
  or any remote mutation.
- Never use `--no-verify`, amend, rebase, reset or rewrite history unless explicitly asked.
- Never commit failing, partial, secret-bearing or unrelated work.

### Refactoring

- Outside the current change's blast radius, do not refactor. (ADR-002)
- Inside it, a second real use extracts, and the extraction is finished: no orphaned imports,
  exports, styles, messages or files remain.
- A third copy of the same logic is a defect. So is an abstraction invented for one caller.

### Validation

- Focused tests are the default gate. A full suite runs only against a named trigger: phase
  exit, release gate, cross-system behaviour, a repo-declared numeric gate, explicit
  instruction, or pre-push on a declared protected branch. (ADR-003)
- Name the exact commands and test files. Partial pass is a blocker; `6/8 pass` is the
  canonical example of a blocked state. Never justify a skip without a pre-existing written
  reason. (ADR-006)
- Evidence discipline is mandatory everywhere. A **numeric** bar is declared per repo in
  `simt-dev.json`, never assumed centrally. (ADR-006)
- The pre-commit hook owns format and lint for staged files; do not re-run what it just ran.
  Manual runs are correct when the hook is absent, when diagnosing a failure, or for a check
  the hook does not cover. (ADR-004)
- Declare a gate only if this repository can run it — a script that exists, a tool that is
  installed, a command you name. A gate that cannot run blocks every commit until someone
  reaches for `--no-verify`. (ADR-017)
- Invoke tools through the repository's own runner — `npm run`, `uv run -m`, `./gradlew` —
  never a package runner like `npx` or a bare console entrypoint. (ADR-010)

### Code and review

- No type-system escapes, no unawaited async work, no debug printing in application code.
- Enforced: max-file-lines
- Never weaken CSP, CORS, auth, audit, cache or validation controls to silence a test, warning
  or alert.
- Review verdicts are exactly three: `BLOCK`, `APPROVE WITH RESIDUAL RISK` (must list what was
  not reviewed), `APPROVE`. (ADR-008)
- Accessibility, correctness, privacy and security are hard gates, not subjective tradeoffs.
- Never direct two agents to edit the same files concurrently. Select the minimum viable team.
- Every agent works to a stated budget and stops at a concrete blocker rather than drifting.
  Approval by budget never converts a known failure into an approval.

### Scope of these rules

Product policy, stack-specific rule bodies, repo commands and paths stay in the repo. A rule
here states only what holds everywhere; the moment one repository needs an exception, it is that
repository's rule, not a widening of this one.

## Stack: android

Applies to: `has-android-manifest` — some module has a
`src/main/AndroidManifest.xml`. A JVM backend with no Android module does not get
this profile; see `jvm`.

### Build

- `./gradlew` only. Never a system `gradle`. Never `--configuration-cache` where a
  repo documents that it breaks.
- `./gradlew spotlessApply` is the formatting entrypoint in repos that use Spotless.
- The manifest is not hand-edited for a build-level change: `applicationId`, `versionCode`
  and `versionName` are declared in the module's build file and land in the merged
  manifest. A value in two places is a value that will disagree.

### Modules and layers

- `:core:domain` depends on no other internal module. Enforce it; do not add an
  import that breaks the direction.
- Layer suffixes: `*Repository`, `*ViewModel`, `*UiState`, `*Screen`, `*Service`.
- Feature modules never import `androidx.compose.ui.window.Dialog`, M3 `AlertDialog`
  or M3 `ModalBottomSheet`. The raw `Dialog` window belongs to allowlisted `core:ui`
  surfaces, and `checkOverlayModalPolicy` fails `./gradlew check` when a feature
  bypasses it.

### Accessibility

- Every user-facing string is a resource, with a `cd_` content description where an
  icon or image carries meaning. Decorative images declare `contentDescription = null`
  explicitly.
- Touch targets meet the platform minimum, and the minimum is asserted rather than
  eyeballed.
- Respect the system font scale and the display size the user has chosen; a fixed
  `sp` value or a hard-coded cell height breaks both.

### Wear OS

- `applicationId` in the wear module must equal the phone app's. Divergence makes the
  Data Layer fail silently — no error, no delivery.
- Wear and phone target SDKs differ by design for Play Console reasons. Do not "align"
  them.

### Data

- Room: bump the version, add the migration, extend the migration test. Never edit an
  applied migration.

### Testing

- Unit tests for view models, repositories and mappers; instrumentation tests for UI.
- `Dispatchers.setMain` in a `@BeforeTest`, restored in `@AfterTest`.
- Assert behaviour, not implementation. Never weaken an assertion, an accessibility
  check or a fixture to match current behaviour.

## Stack: jvm

Applies to: `gradle`, `has-maven-pom`, `kotlin`, `java` — a JVM build. The Android
rules live in the `android` profile, which selects on `has-android-manifest`.

### Build

- `./gradlew` only, never a system `gradle`; `./mvnw` where a wrapper is committed,
  never a system `mvn`. Never `--configuration-cache` where a repo documents that it
  breaks.
- `./gradlew spotlessApply` is the formatting entrypoint in repos that use Spotless.
- Pin the JVM toolchain in the build file (`java.toolchain`, `sourceCompatibility`).
  A JDK on the developer's PATH is not a floor anything enforces.

### Modules and layers

- `:core:domain` depends on no other internal module. Enforce it; do not add an
  import that breaks the direction.
- Layer suffixes: `*Repository`, `*Service`, `*Mapper`, `*Controller`. A class named
  `*Repository` that talks to HTTP is not a repository.

### Kotlin

- Four-space indent, LF, 300-char line limit.
- No `!!`. Prefer a sealed `RepositoryResult<T>` over exceptions for expected failures.
- Expose `MutableStateFlow` as `StateFlow` via `.asStateFlow()`; update state with
  `_state.update { }`.
- Java and Kotlin interop: `Optional` in a Java signature, a nullability annotation in
  a Kotlin one, and never a platform type left untyped at the boundary.

### Testing

- Unit tests for services, repositories and mappers; an integration test only when the
  behaviour is genuinely cross-system.
- Assert behaviour, not implementation. Never weaken an assertion to match current
  behaviour.
- Test the narrowest layer that proves the behaviour.

### Data

- One new sequential immutable migration per change. Never edit an applied migration.
  Define coexistence order and forward repair; do not promise SQL rollback.
- Prepared, parameterised statements only. Never interpolate a value into SQL, and
  never assemble SQL by string concatenation from a caller-supplied identifier.

Commands for this repository:

- build: `./gradlew build`
- focused test (Android): `./gradlew :{module}:testDebugUnitTest --tests {class}`
- focused test (JVM): `./gradlew test --tests {class}`

## Process: intake-questions

When starting, resuming, or scoping a feature slice or plan task, do not jump into code
without resolving open material decisions.

### Agent Defaults (Do Not Ask)

Apply documented defaults without questions for:

- Operational/procedural choices (tracker updates, commit vs push, validation depth,
  artifact shape).
- Slice sequencing and repository ownership when already defined in the active plan
  tracker table (`docs/plans/*`).
- Standard engineering conventions.

### When To Ask (1-5 Questions Max)

Ask only when a functionality, business, privacy, security, or technical decision is not
settled in authority docs, and the answer materially changes behavior, scope, contracts,
migrations, or risk.

Five is a ceiling, not a quota. Ask fewer when fewer decisions are genuinely open; never pad
the list to reach it, and never split one decision into several questions to fit under it.

- Ask the user, offering choices with the recommended one first.
- If that is not available, present numbered choices with `(recommended)` on the first option.
- Do not ask operational or procedural choices; apply the best documented option.

### Required First Turn

1. State the smallest independently valuable slice, the owning repo, the active plan
   tracker task ID (from `docs/plans/*`), and the acceptance checks.
2. Either ask the unresolved material decision questions, or state that the authority docs
   leave no material open decision and proceed to implementation.
3. Proceed without waiting when no material decision remains.

## Process: slice-commit

A slice is the smallest change that is independently valuable on its own. Committing
one is a mechanical procedure with a fixed order, because every step after it depends
on the one before: you cannot review a diff you have not staged, and you cannot claim
evidence for a test you have not run.

### Order

1. **Run the focused test the change needs**, through the repository's own runner,
   naming the exact command and the exact file — see `test-gate`. A full suite is
   for a named trigger, not for progress.
2. **Stage only what the slice touched.** `git status` first, and leave unrelated work
   alone. Report what you left alone; silence about it reads as "I reverted it".
3. **Read the staged diff.** Not the working tree — the diff. It is what will be
   committed, and it is the last chance to notice a file that does not belong.
4. **Write the message.** Conventional Commits, imperative subject, no trailing period:
   `fix(gates): declare only gates a repo can run, and report the ones it cannot`.
   The subject says what changed; the body says why, and what was deliberately not
   changed.
5. **Commit.** `git commit`, non-interactive. Report the SHA.
6. **Stop.**

### Pushing is a separate decision

**Never push because the work is finished.** Push is a separate act with its own
approval, and pushing the branch a remote treats as production can deploy it.

Before asking for approval, state what the push will do: which branch, how many
commits, and whether it triggers a deployment, a migration, or any other remote
mutation. Then wait. Approval to commit is not approval to push.

Never use `--no-verify`. A hook that fails is telling you something about the change;
skipping it turns a gate into a suggestion and the next person has no way to know it
was skipped.

### What never belongs in a commit

- Failing, partial, or unrun work. A commit is a claim; a partial one is a false one.
- Secrets, tokens, or generated artefacts that drift.
- Work nobody asked for. Unrelated changes in a slice commit are invisible in review
  and impossible to revert on their own.

## Process: test-gate

Applies to every feature slice, bugfix, and plan tracker task. Docs/config-only chores are
exempt from test deltas but require named doc/format checks to pass.

### Implementer Requirements

1. **Name exact commands and test files**: your test runner, through the repository's own script or task, selecting
   `path/to/the.changed.test.ts` on its own.
2. **100% Pass Required**: the named commands must exit 0 with every test passing. A
   partial pass (e.g. 6/8 passing) is a hard blocker, and so is quoting the passing subset.
3. **Behavioral Test Deltas**:
   - New feature/contract/route path -> new or extended test asserting the behavior.
   - Bugfix -> regression test reproducing the failure mode.
   - If no test file changed, justify why the existing tests fully prove the invariant.
4. **Cross-Repo Order**: commit the API producer (contract/migration) before any
   consumer generates against it. Never apply remote migrations or deploy from uncommitted
   code.

### Reviewer Mandatory Block

Return `BLOCK` when:

- Any test fails, is skipped (without pre-existing justification), or is unrun.
- An executable diff has no test delta and no explicit justification.
- The coordinator cites partial suite results or defers a failing test.
- **Consumer side:** a consumer requires a new API field with no backward-compatibility or
  deploy-order evidence.
- **Producer side:** the API producer ships without a test asserting the new invariant
  or field.

The last two are complementary, not alternatives. A reviewer that checks only one of them
misses half of the failure they exist to catch.

## Disclosure

- gnss-engine — Navigate and extend the simt-gnss GNSS engine library (app.simt:gnss-core / app.simt:gnss-android) - pure-JVM parsers, solvers, filters, corrections and orbit computation, plus the Android GnssMeasurementProcessor and its public SimtGnssEngine facade. Use for any task mentioning GNSS, RTK, PPP, RTCM, SBAS, NTRIP, ephemeris, pseudorange, carrier phase, ionosphere/troposphere, Klobuchar, satellite constellations, or raw GnssMeasurement processing.

<!-- simt:end -->
