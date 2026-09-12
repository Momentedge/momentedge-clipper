# Workspace tasks. `just --list` shows them all.
#
# Every recipe runs the *caller's* toolchain and never enters a nix shell of its
# own: the recipes whose names end in `-ros` link r2r and must be invoked from
# inside the dev shell (`nix develop --command just check-ros`), and everything
# else runs straight from the repo root on the system toolchain, which is the
# ROS-free lane CI holds down.
#
# `rustfmt.toml` carries unstable options, so `fmt` and `fmt-check` need a
# nightly rustfmt; the dev box's system rustfmt is nightly, as is CI's.

default:
    just --list

# The three crates the ROS-free lane covers. The examples are workspace members
# for their metadata and their lockfile, not for this loop — `cu-mcap-record`
# alone drags in the whole cu29 tree — and CI gives them their own job.
crates := "-p clip -p tail -p clipper"

# Format every crate, examples included.
fmt:
    cargo fmt --all

# Check formatting without writing (the gate CI runs).
fmt-check:
    cargo fmt --all --check

# `-D warnings` is what makes the `warn` levels in `[workspace.lints]` a gate
# rather than a suggestion; clippy exits zero on all of them without it, and
# `RUSTDOCFLAGS` is the same promotion for the `[workspace.lints.rustdoc]`
# block. Building the docs is the only thing that runs rustdoc's lints: a broken
# intra-doc link warns during a doctest run and fails here.

# One `-p` list is one feature resolution: linting the three together turns on
# `clip/clap` and both `test-support` features, because the recorder asks for
# them. `clip` alone is the feature set a downstream consumer actually gets, so
# it is linted on its own first.

# Verify formatting, compile, then lint code and docs with warnings denied.
check: fmt-check
    cargo clippy -p clip --all-targets -- -D warnings
    cargo clippy {{crates}} --all-targets -- -D warnings
    RUSTDOCFLAGS="-D warnings" cargo doc {{crates}} --no-deps

# The same gate with ROS on. Run it from inside the dev shell:
#   nix develop --command just check-ros
check-ros:
    cargo clippy {{crates}} --features clipper/ros --all-targets -- -D warnings

# Two runners because the first cannot do the second's job: nextest gives every
# test its own process and has no doctest support at all, so an example in a doc
# comment is compiled and executed only by the `--doc` run beside it.

# Run the test suites and the doctests.
test:
    cargo nextest run {{crates}}
    cargo test {{crates}} --doc

# Lint and test the example crates too — the lane CI splits into its own jobs.
test-examples:
    cargo clippy --locked --all-targets -p cu-mcap-record -p custom-mcap-writer -p chunked-mcap-writer -- -D warnings
    cargo test --locked -p cu-mcap-record -p custom-mcap-writer -p chunked-mcap-writer

# The live ROS 2 e2e suite: a real `ros2 bag record`, CLI-published triggers,
# and `ros2 topic echo` for `Recorded`. Gated on CLIPPER_E2E, and `--features
# ros` is not optional — the suite starts the recorder on `--trigger-source ros`.
# Run it from inside the dev shell:
#   nix develop --command just e2e
e2e:
    CLIPPER_E2E=1 cargo nextest run -p clipper --features ros --profile e2e -E 'binary(e2e)'

# One instrumented run, three renderings of it: `--no-report` leaves the raw
# profile behind so the cobertura file, the browsable HTML and the table all
# describe the same execution. Everything lands under `target/`, which is
# already ignored. Doctests are outside the measurement — the test runner does
# not carry them.
#
# `--branch` adds the branch columns. It rests on rustc's
# `-Z coverage-options=branch`, so it is nightly-only and prints an unstable
# warning; the toolchain is already nightly for `rustfmt`. LLVM counts a branch
# where control flow forks on a condition — `if`, `&&`, `||`, `while let` — and a
# `match` arm is a *region*, not a branch, so code that decides by matching
# reports few branches and a high region count rather than a gap in the tests.
#
# The `-p` list is load-bearing: `-p clipper` alone builds no test binary for
# `clip` or `tail`, so the libraries' own tests never run and their lines —
# instrumented all the same, as path dependencies — report only what the
# recorder's tests happen to reach.

# Measure coverage, branches included, and print the per-file table.
cov:
    cargo llvm-cov clean --workspace
    mkdir -p target/llvm-cov
    cargo llvm-cov nextest {{crates}} --branch --no-report
    cargo llvm-cov report --branch --cobertura --output-path target/llvm-cov/coverage.cobertura.xml
    cargo llvm-cov report --branch --html
    cargo llvm-cov report --branch

# Open the browsable report `cov` wrote.
cov-open:
    xdg-open target/llvm-cov/html/index.html
