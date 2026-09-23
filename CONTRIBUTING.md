# Contributing to Momentedge Clipper

Bug reports, questions and patches are all welcome. File an issue at
[Momentedge/momentedge-clipper/issues](https://github.com/Momentedge/momentedge-clipper/issues),
or open a pull request.

This page is everything you need to get from a clone to a merged change. If you
have not run clipper yet, the [README](README.md) explains what it does and how
to cut a first clip; [Installing and building](docs/install.md) covers the ways
to get a binary.

## What a good change looks like

clipper cuts clips out of a continuous `ros2 bag record` on trigger events. It
owns no sensor subscriptions and no message index, and it copies MCAP messages
straight through, decoding nothing but each message's timestamp. Three
properties follow from that, and a change is expected to keep all three true:

- **No message body is ever decoded.** That is what makes clipper agnostic to
  your message types. Deserializing a payload changes what the project is.
- **ROS is optional.** It is a cargo feature, off by default. `crates/clip` and
  `crates/tail` link no ROS at all, and neither does the recorder unless you ask
  for it. A dependency that escapes the feature gate breaks consumers who have
  no ROS installation.
- **A clip is a complete, standalone MCAP file in a directory that says what it
  is.** `clip_metadata.yaml` is written last, and its presence is what
  "complete" means to everything downstream.

Small fixes and documentation corrections need no preamble — send them. For
anything that changes a flag, the trigger contract, the clip directory, or the
behaviour a deployed recorder depends on, open an issue first so the design
discussion happens before the code.

## Layout

A Cargo virtual workspace. The three crates that matter:

| Path | What it is |
|---|---|
| `crates/clip` | the shared library: MCAP format layer, the recording index, the cut, the trigger contract, the clip directory and its document. No ROS. |
| `crates/tail` | what following a recording still being written adds: discovery, the recording collection, coverage, retention, the waits before a cut. No ROS. |
| `crates/clipper` | the binary. `clipper tail` is the recorder; `clipper clip` cuts windows out of a finished recording and exits. |

Around them: `momentedge_msgs/` is the local ROS 2 interface package carrying
`Trigger` and `Recorded`, [`examples/`](examples/README.md) holds the trigger
publisher and the MCAP-writer examples, `sim/` is a synthetic camera for local
testing, and `nix/` with `flake.nix` builds the per-distro ROS 2 dev shells.

## Building

Most work needs neither ROS nor Nix. `clip`, `tail` and the recorder's default
build are ROS-free, so the inner loop is plain cargo from the repo root:

```bash
cargo build -p clipper                     # the ROS-free binary
cargo test -p clip -p tail -p clipper
```

The device build adds the live `Trigger` subscription and the `Recorded`
publish. It links r2r, so it needs a ROS 2 environment, which the Nix dev shell
provides:

```bash
nix develop                                            # jazzy, the default
nix develop --command cargo build -p clipper --features ros
```

`nix develop .#humble` and `.#lyrical` select the other supported distros.
Rolling gets a working shell but cannot build the Rust crates: r2r references an
rmw QoS variant Rolling has removed.

Flakes only see git-tracked files. A new or renamed file under
`momentedge_msgs/` has to be `git add`ed before `nix develop` or the build, or
evaluation fails with *"Path … is not tracked by Git"*.

**Prerequisites.** A **nightly** Rust toolchain, plus `just`, `cargo-nextest`,
and `cargo-llvm-cov` if you want coverage. Nightly is a hard requirement for the
gate, not a preference: `rustfmt.toml` sets unstable options, and several test
modules name a clippy lint that only nightly clippy knows — an unknown lint name
is an *error* under `-D warnings`, so the lint pass fails outright on stable.
Building and testing work fine on stable.

## Tests

A change is expected to arrive with tests. There are around 300 unit and
integration tests across the three crates, and they set the local convention:

- **A test name is a sentence** saying what the test asserts —
  `a_growing_file_is_not_re_yielded`, `an_empty_clip_says_which_kind_of_empty_it_is`.
- **A `///` doc comment above the test says why the property matters**, not what
  the code does. The reason a test exists is the part a reader cannot recover
  from the assertions.
- Lints apply inside `#[cfg(test)]` too. Each test module carries one inner
  `#![allow(…, reason = "…")]` rather than an attribute per case.

### The end-to-end suite

`crates/clipper/tests/e2e.rs` drives the real stack: an actual `ros2 bag
record`, triggers published from the CLI, and `ros2 topic echo` for `Recorded`.
It is awkward, and deliberately so — it needs a real ROS 2 environment, it is
gated behind `CLIPPER_E2E`, and it runs serialized for roughly ten minutes
because the tests sleep out real trigger windows:

```bash
nix develop --command just e2e
```

With `CLIPPER_E2E` unset every case prints a skip notice and passes, so the
ordinary `cargo test` run is unaffected. If your change cannot touch the live
path, say so in the pull request and a maintainer will run the suite.

## Before you open a pull request

```bash
just check                            # fmt-check, then clippy and rustdoc with warnings denied
just test                             # nextest over the three crates, then the doctests
nix develop --command just check-ros  # the same clippy gate with the ros feature on
```

`just check` and `just test` are the gate every change has to pass. Run
`check-ros` too if you touched anything behind the `ros` feature, and `just e2e`
if you can. `just cov` renders a coverage report; there is no threshold to meet,
it is there to find the branch you forgot. `just --list` names every recipe.

## Commit messages

[Conventional commits](https://www.conventionalcommits.org/), with the crate as
the scope where one applies: `fix(tail): prune an ended recording that holds no
message`.

**Subjects are published verbatim.** git-cliff turns the commit log between two
release tags into the body of the GitHub release, so your subject line is read
by people who will never see the diff. Write it as a sentence stating what the
change makes true, in the present tense, lower case, with no trailing period.
`feat`, `fix` and `perf` lead the release notes; `refactor`, `docs`, `test`,
`build`, `ci`, `style` and `chore` are collapsed beneath them.

## Documentation

The docs describe the same system from different angles, and they move with the
code in the *same* change:

- A flag, an environment variable, a TOML key, the trigger contract, the clip's
  document, or what a deployed recorder does → the matching page under
  [`docs/`](docs/).
- The pitch, the quickstart, the install path → [README.md](README.md). It is a
  primer, so a new fact earns a place there only by changing what clipper *is*
  or what it costs to run; everything else goes to `docs/` and is linked.
- Thread model, tailing, clip publication, recovery, deployment →
  [ARCHITECTURE.md](ARCHITECTURE.md).

Write for a reader who has never seen the previous version: describe the system
as it is, not as a change from something that to them never existed. The diff
belongs in the commit message.

The repository also carries `CLAUDE.md` files aimed at AI coding agents working
on the crates. You do not need them to contribute, and nothing here depends on
them.

## What happens next

CI runs on every push to a branch in this repository: rustfmt, a ROS-free lane
covering all three crates on a stock toolchain with no ROS installed, the copper
example, and a full build plus unit and e2e suites on Humble, Jazzy and Lyrical.

It does **not** run on `pull_request` events, so a pull request opened from a
fork produces no CI run of its own. That is not a judgement on the change — a
maintainer pushes the branch to run it. Expect that to be the slowest part of
the review.

## License

clipper is [Apache-2.0](LICENSE). Contributions are accepted under the same
licence; there is no CLA to sign.
