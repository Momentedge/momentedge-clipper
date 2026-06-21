# Momentedge Clipper — contributor & agent notes

This is the entry point for contributing to or operating clipper. The other docs
own their angle, and this file does not repeat them:

- **[README.md](README.md)** — user-facing: what clipper is, how to install, run,
  configure, and the trigger interface. Start there to *use* clipper.
- **[ARCHITECTURE.md](ARCHITECTURE.md)** — the technical overview: thread model,
  tailing, atomic clip publication, recovery, the `ros`/`mcap` seam, deployment.
- **[crates/clipper/CLAUDE.md](crates/clipper/CLAUDE.md)** — the recorder's deep
  internals and concurrency invariants, for changing the crate.
- **[examples/trigger-pub/CLAUDE.md](examples/trigger-pub/CLAUDE.md)** and
  **[sim/CLAUDE.md](sim/CLAUDE.md)** — the example trigger source and sim camera.

Build, CI, and packaging mechanics live in **skills** (loaded on demand):

- **`build`** ([`.claude/skills/build/SKILL.md`](.claude/skills/build/SKILL.md))
  — the Nix dev shell, per-distro builds, the r2r/IDL model, tests and coverage,
  the live e2e suite, the binary cache.
- **`ci`** ([`.claude/skills/ci/SKILL.md`](.claude/skills/ci/SKILL.md)) — the
  GitHub Actions workflows, skip rules, and local `act` testing.
- **`packaging`** ([`.claude/skills/packaging/SKILL.md`](.claude/skills/packaging/SKILL.md))
  — the two-deb (bloom + cargo-deb) release pipeline and its gotchas.

## The recorder in one line

`clipper` is a triggered clip recorder: it owns no sensor subscriptions and no
message index, and cuts clips out of a continuous on-disk `ros2 bag record` on
trigger events, copying MCAP messages straight through (decoding only each
message's log time). The example [`trigger-pub`](examples/trigger-pub/CLAUDE.md)
is a periodic `Trigger` publisher that drives it during development. The full
design is in [ARCHITECTURE.md](ARCHITECTURE.md) and
[crates/clipper/CLAUDE.md](crates/clipper/CLAUDE.md).

## The sim camera (`sim/`)

`sim/` is the in-repo data source: a synthetic gscam camera (`videotestsrc` →
raw + H.265 topics) driven by `sim/cam_sim.sh` (`run` / `record` / `stop`). It
is a launch/config tree, not a Cargo crate. Its ROS packages are the `simPaths`
half of `nix/ros-env.nix`, gated by `withSim` and pulled in only for the distros
in `simDistros` (`jazzy`, `humble`, `lyrical`, `rolling`); Humble needs the
`simOverlays` pkg-config fix. The recorder, the e2e suite, and deployment use
none of the sim stack — the `corePaths` half builds on every distro regardless.
Overview and usage: [`sim/README.md`](sim/README.md); gotchas and the distro
matrix detail: [`sim/CLAUDE.md`](sim/CLAUDE.md).

## Workspace layout

A virtual workspace (no root package), so `resolver = "3"` (the edition-2024
resolver) is set explicitly — a virtual workspace does not infer the resolver
from member editions and otherwise falls back to `"1"` with a warning. Members
are the recorder crate (`crates/clipper`) and the example trigger source
(`examples/trigger-pub`) — the example stays a member so it inherits
`[workspace.package]` and the shared `[workspace.dependencies]` versions, rather
than for shipping. `momentedge_msgs/` (ROS2 interface package) and `sim/` (the
sim camera's launch/config tree) are not Cargo members.

```
crates/clipper/         # triggered clip recorder tailing the continuous mcap
momentedge_msgs/        # local ROS2 interface package (Trigger, Recorded)
examples/               # setup guides (continuous, split-bags, launch) + trigger-pub
sim/                    # synthetic gscam camera (sim/cam_sim.sh) — see sim/README.md
nix/                    # flake package defs: momentedge-msgs, ros-env, binaries
scripts/                # record.sh, run.sh, build-on-target.sh, packaging scripts
flake.nix               # per-distro ROS2 dev shells + nix-built binaries
```

## Sibling repositories

- `../ros2_sources` — the bag replay that feeds the recorder (its README has the
  workflow).

## Keeping docs in sync

The docs describe the same system from different angles. **After a change to
behaviour, build/run steps, dependencies, or layout, update the relevant docs in
the same change** — do not duplicate; cross-reference:

- User-facing overview, quickstart, configuration → **README.md**.
- Technical/system design → **ARCHITECTURE.md** (crate internals →
  `crates/clipper/CLAUDE.md`).
- Build / CI / packaging mechanics → the **`build` / `ci` / `packaging`** skills.
- Contributor orientation and conventions → this **CLAUDE.md**.

If a fact would otherwise appear in two places, it belongs in the most specific
one and the others link to it.

## Beads Issue Tracker

This project uses **bd (beads)** for issue tracking. Run `bd prime` to see full workflow context and commands.

### Quick Reference

```bash
bd ready              # Find available work
bd show <id>          # View issue details
bd update <id> --claim  # Claim work
bd close <id>         # Complete work
```

### Rules

- Use `bd` for ALL task tracking — do NOT use TodoWrite, TaskCreate, or markdown TODO lists
- Run `bd prime` for detailed command reference and session close protocol
- Use `bd remember` for persistent knowledge — do NOT use MEMORY.md files

**Architecture in one line:** issues live in a local Dolt DB; sync uses `refs/dolt/data` on your git remote; `.beads/issues.jsonl` is a passive export. See https://github.com/gastownhall/beads/blob/main/docs/SYNC_CONCEPTS.md for details and anti-patterns.

## Agent Context Profiles

The managed Beads block is task-tracking guidance, not permission to override repository, user, or orchestrator instructions.

- **Conservative (default)**: Use `bd` for task tracking. Do not run git commits, git pushes, or Dolt remote sync unless explicitly asked. At handoff, report changed files, validation, and suggested next commands.
- **Minimal**: Keep tool instruction files as pointers to `bd prime`; use the same conservative git policy unless active instructions say otherwise.
- **Team-maintainer**: Only when the repository explicitly opts in, agents may close beads, run quality gates, commit, and push as part of session close. A current "do not commit" or "do not push" instruction still wins.

## Session Completion

This protocol applies when ending a Beads implementation workflow. It is subordinate to explicit user, repository, and orchestrator instructions.

1. **File issues for remaining work** - Create beads for anything that needs follow-up
2. **Run quality gates** (if code changed) - Tests, linters, builds
3. **Update issue status** - Close finished work, update in-progress items
4. **Handle git/sync by active profile**:
   ```bash
   # Conservative/minimal/default: report status and proposed commands; wait for approval.
   git status

   # Team-maintainer opt-in only, unless current instructions forbid it:
   git pull --rebase
   git push
   git status
   ```
5. **Hand off** - Summarize changes, validation, issue status, and any blocked sync/commit/push step

**Critical rules:**
- Explicit user or orchestrator instructions override this Beads block.
- Do not commit or push without clear authority from the active profile or the current user request.
- If a required sync or push is blocked, stop and report the exact command and error.
