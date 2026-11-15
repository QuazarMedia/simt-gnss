---
name: simt-commands
description: Command reference for simt-dev: which commands are mutating vs read-only, and a scenario-to-command mapping for the agent.
---

## Dry run

Most simt-dev commands that write to the repository default to **dry-run**:
they print what they would change and exit without writing. Add `--yes` to
apply. Read the dry-run output first — it is the plan — then re-run with
`--yes`. A dry-run that shows an unexpected file is a reason to stop, not to
add `--yes` and hope.

### Mutating commands (need `--yes` to write)

`init`, `sync`, `plan done`, `slice start`, `slice finish`, `slice abandon`,
`agents install`, `agent-hooks install`, `hooks install`, `doctor --fix`,
`ignore`, `ci`, `shared install`, `lessons review`, `decide`, `undo`,
`skills install`, `bench mine|run|ablate|probes|variants`, `pack new|publish`,
`lease`, `plugin <harness>`, `workspace upgrade`

### Read-only commands (no flag needed)

`brief`, `why`, `tests`, `gate`, `impact`, `map`, `check`, `status`,
`plan status|next`, `rules *`, `agents status`, `agent-hooks status`,
`hooks status`, `evidence`, `evidence check`, `evidence validate`,
`pr-body`, `explain`, `graph lint`,
`skills index|brief|show|why|find|compile|eval`, `index`, `index overlay`,
`review`, `tokens`, `harness *`, `compare`, `workspace compare|share`,
`distill`, `docs readme`, `local doctor`, `shared plan`,
`bench report|critic`, `pack lint`, `mcp`, `completions`,
`lessons list|capture|discover`, `knowledge *`, `work *`, `verify *`,
`action status|review`, `adapter check`

## Command reference

Pick the command by what you need, not by what you remember:

| You need to… | Command | Notes |
|---|---|---|
| Understand a task before exploring | `simt brief "<task>"` | Kind, commands, files, tests, decisions |
| Know what governs a file | `simt why <file>` | Decisions, rules, skills, gates, lessons |
| Find tests for a file | `simt tests [path]` | Import, conventional, or coverage |
| Find tests a change reaches | `simt gate [path...]` | `--changed` reads the diff |
| See who a change affects | `simt impact <file\|symbol>` | Importers, tests, contracts |
| Find files a task touches | `simt map --task "<text>"` | `--level outline\|names\|signatures` |
| Read tracker state | `simt plan status` | Counts per tracker file, and the share done |
| Get the next slice | `simt plan next [<id>]` | Smallest ready slice, ~1 KB |
| Stay inside a tier, wave or lane | `simt plan next --tier L --wave 1 --lane core` | Any filter, or all three at once |
| See what the tool read from a tracker | `simt plan labels` | Matched headers, unmatched columns |
| Mark a slice done | `simt plan done <id> <sha> --test "<cmd>" --yes` | Dry-run without `--yes` |
| Start a slice in a worktree | `simt slice start <id> --yes` | Branch `slice/<id>`, brief beside it |
| Finish a slice | `simt slice finish <id> --yes` | Gates, commit, record SHA |
| Abandon a slice | `simt slice abandon <id> --yes` | Removes worktree, returns row |
| Check the gate | `simt check` | Projection drift, manifest, policy |
| Regenerate projection | `simt sync --yes` | Idempotent; `--check` for drift only |
| Explain a finding | `simt explain <CODE>` | Decision, fix, trigger |
| Read a rule | `simt rules show <id>` | One rule body |
| Read a decision | `simt rules why <ADR-NNN>` | `--all` for every ruling |
| Scaffold an ADR | `simt decide [<title>] --yes` | Next number, skeleton, frontmatter |
| Adopt a repo | `simt init [path] --yes` | `--backup` tags the pre-adoption commit |
| Install the agent | `simt agents install --yes` | `--harness <id>` for one |
| Check agent status | `simt agents status` | Present, absent, drift, no-file |
| Install hooks | `simt hooks install --yes` | Shared dispatcher; `--agent` writes each harness hooks file |
| Run a hook event | `simt hooks run <event>` | As git would |
| Install agent hooks | `simt agent-hooks install --yes` | Per-harness policies |
| Diagnose config | `simt doctor [--fix]` | `--fix` applies safe remedies |
| Pin a skill | `simt skills install <name> --yes` | `--variant` for multi-variant |
| Search skills | `simt skills find <query>` | Capability and keyword |
| Rebuild the index | `simt index` | From git blob hashes |
| Get PR body | `simt pr-body [--base <ref>]` | Evidence, decisions, gate state |
| Check evidence | `simt evidence` | Runs recorded against the tree |
| Review the staged diff | `simt review` | ADR-008 verdict skeleton |
| Undo last writes | `simt undo --yes` | Only if every file is untouched |
| Claim files for a session | `simt lease [<paths>...] --yes` | TTL; pre-edit hook refuses |
| Measure token cost | `simt tokens` | Bytes a reader loads at session start |
| Snapshot the repository | `simt status` | `--line` prints the last check's line |
| Check what a commit claims | `simt evidence check <msg-file>` | A run this tree does not have |
| Compare sibling repositories | `simt compare` | Adoption and drift matrix |
| Check local model endpoints | `simt local doctor` | `--allow-remote` to probe |
| See what a shared install would write | `simt shared plan` | `shared install --yes` writes it |
| Render the README command list | `simt docs readme` | From the spec; `--yes` writes it |
| Compile a skill | `simt skills compile [<name>]` | Agent Skills standard |
| Score the registry | `simt skills eval [corpus]` | Against recorded transcripts |
| Refresh the workflows | `simt ci` | This package's own `.github` |

## Doors (JSON in, facts out)

The Plan 0004 modules answer through read-only doors: each takes a JSON document
(`--input <file>`, or `--claims`/`--refs`/`--journal` where the table says) and
returns the module's own facts. None of them write to the repository. A door with
an MCP name is also a read-only MCP tool calling the same fact function, so the
CLI and a client cannot disagree about a fact.

| You need to… | Command | MCP tool |
|---|---|---|
| What a claim derives from, what depends on it, what applied at a date, conflicts | `simt knowledge why <id> --claims <file>` | `knowledge_why` |
| The retrieval units a long document yields, with anchors and byte ranges | `simt knowledge sections <file.md>` | — |
| Ranked evidence units for a question, and what was excluded | `simt knowledge rank --input <f> --query "…"` | — |
| Stale, unsupported knowledge ranked by consequence per effort | `simt knowledge debt --input <f>` | — |
| Which outputs a source, template or skill change puts in question | `simt knowledge derivations --input <f>` | — |
| What a change reaches over typed edges, and where the graph is unknown | `simt knowledge impact --input <f> --refs <ids>` | — |
| A bounded packet another repo can read; validate such a packet | `simt knowledge export` / `import --input <f>` | `knowledge_export` / `knowledge_import` |
| The next observation worth making; the narrowest checks that prove a change; closure over obligations | `simt verify plan\|select\|status --input <f>` | `verify_plan` / `verify_select` / `verify_status` |
| Obligations against observations; a generated doc vs its template; procedures' imports and prerequisites | `simt verify contracts\|documents\|procedures --input <f>` | `verify_contracts` / `verify_documents` / `verify_procedures` |
| What this repo's work is: artifacts, activities | `simt work inspect` | `work_inspect` |
| Ready tasks that may run together; a resume capsule; revalidate a capsule; traces proposed as a procedure; the proof-bearing task packet | `simt work schedule\|handoff\|resume\|graduate\|packet --input <f>` | `work_packet` |
| Where an operation stands from its journal | `simt action status --journal <f>` | `action_status` |
| A reviewable bundle of contract, delta, target, authorization, postconditions | `simt action review --input <f>` | `action_review` |
| Validate adapter descriptors; observe-only vs execute | `simt adapter check --input <f>` | `adapter_check` |
| Deduplicate discovery candidates and report their authority | `simt lessons discover --input <f>` | `lessons_discover` |
| A working-tree or staged node labelled by its provenance | `simt index overlay --input <f>` | `index_overlay` |
| Validate ledger evidence and the chain of custody behind it | `simt evidence validate --input <f>` | `evidence_validate` |

## Tiers, waves, lanes and reviewers

A tracker may carry a tier column (headers `Tier`, `Model tier`, `Class`, `Effort`). The tag
is the cell's first word without its notes, so `L (S)` is an L slice with an S reviewer.
`--tier <name>` answers the next ready slice in that class, `--wave <name>` the next ready one
in that wave (the plan's wave tables), and both together apply as one filter. A filter that
matches nothing says so — `no queued slices in tier S` — and never falls back to the whole
tracker.

A lane is the subsystem a slice's files belong to (headers `Lane`, `Stream`, `Workstream`,
`Track`), read by the same first-word rule. Lanes are meant to own disjoint file sets, which
is what makes concurrency safe: `--lane core` and `--lane capture` handed to two agents
touch different files, while two slices in one lane do not. Combine `--lane` with `--tier` or `--wave` when a lane must also stay inside a
class or a wave.

A `Reviewer` column (headers `Reviewer`, `Reviewed by`, `Review by`, `Reviewer profile`) names
who reviews the slice — a model id or a person, as free text, because the tool has no reason
to know which. The `To review` state means the work is finished and not accepted: it is not
ready work, and nothing depending on it is ready either. `plan next` lists what is waiting
under `awaiting review:`, and `simt plan done <id> --state "to review"` hands a slice over.
