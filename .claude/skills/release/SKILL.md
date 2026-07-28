---
name: release
description: >
  Writing the human half of a release — the annotated tag's message, which becomes
  the release headline and the overview above the generated changelog. Use when
  cutting a release, drafting or trimming release notes, or rewording a release
  that is already published.
---

# Release notes — the half a generator cannot write

A release page has two authors. The generated changelog answers **what landed**:
it is derived from the conventional-commit subjects between two tags, it links
every commit, and it is already complete. The annotation answers **what this
release is for** — the one question a commit list cannot.

Write only the second. Every sentence that restates the first is waste.

## The mechanism

Tag annotated, never lightweight:

```bash
git tag -a v0.2.0        # opens an editor
git push origin v0.2.0
```

git-cliff reads the annotation as `message`: **the first line becomes the
headline, the rest becomes the prose beneath it.** The generated groups follow.
A lightweight tag has no annotation to read and renders the changelog alone —
degraded, not broken.

The annotation is the only human-written part of a release, and it is immutable
once pushed. Draft it before tagging.

## The rules

1. **Headline** — one line, ≤ 60 characters, plain sentence or imperative. No
   version number, no `feat(scope):` prefix. It answers *what this release does
   for me*: `Fix the release pipeline`, `Clip windows can follow capture time`.

2. **Length tracks reader impact, not commit count.** A 40-commit release that
   changes nothing a user touches gets one paragraph; a 3-commit release that
   changes a default gets one paragraph too.

   | release | prose |
   |---|---|
   | maintenance, no behaviour change | 1 paragraph, 2–3 sentences |
   | one new capability | 1 paragraph on it; a 2nd only if something else changes what a reader *does* |
   | first release, or breaking | 2 paragraphs — no prior context to lean on |

3. **Consequences, not mechanisms.** The commit body already holds the
   postmortem and the changelog links to it. If a sentence explains *how* a bug
   worked, cut it. "The release job failed to publish" earns its place; "the
   missing newline absorbed the heredoc delimiter" does not.

4. **Name the artifact.** `--time-source publish`, `examples/cu-mcap-record`,
   the package name. Concrete nouns let a reader decide whether this release is
   for them; abstractions ("improved timing flexibility") do not.

5. **Say what to do.** Safe to upgrade, default changed, anything breaking — one
   clause is enough. End on it.

6. **Change-relative wording belongs here.** "now", "no longer", "previously"
   are banned from durable documentation because they describe a past the reader
   never saw. Release notes are the documented exception, alongside commit
   messages: describing change *is* the job.

7. **Never restate the changelog.** If a sentence is a commit subject rewritten
   as prose, delete it. The reader is three lines from the real one.

8. **Skip repo-internal process news** unless it changes what a contributor
   does. "Releases now generate their own notes" is a commit, not an overview.

9. **If a previous tag never published**, say so and say what ships in its
   place. A reader who skipped a version needs to know where it went.

## Drafting

Write the draft to a file and render it against the real commit list *before*
tagging, so the prose and the generated groups are read together:

```bash
git-cliff --unreleased --with-tag-message "$(cat draft.txt)"
```

`--unreleased` covers commits since the newest tag. To re-render a tag that
already exists — for a reword — use `--latest` for the newest tag, or an
explicit range for an older one:

```bash
git-cliff --latest                  --with-tag-message "$(cat draft.txt)"
git-cliff v0.1.0..v0.1.1            --with-tag-message "$(cat draft.txt)"
```

Then read the whole page and ask: *does the prose say anything the list below
already says?* Cut what it does.

End the file with a trailing newline. Shell heredocs and CI steps that read
notes back through a delimiter fail on a file that does not.

### Two traps when re-rendering an old tag

- **`--tag-pattern` does not scope output to one tag.** It filters which tags
  are recognised as releases, so restricting it to a single tag collapses the
  entire history into that one release. Use a range.
- **A first release has no lower bound.** `<root>..v0.1.0` re-renders every
  pre-release tag in the range as its own section. When that list is long
  enough to bury the prose, publish the overview plus a commits link instead —
  the changelog for a first release is churn no one reads.

## Rewording a published release

The tag annotation is immutable without a force-retag, which rewrites published
history — do not. The **release body** is independent and freely editable:

```bash
git-cliff --latest --with-tag-message "$(cat draft.txt)" -o body.md
gh release edit v0.1.3 --notes-file body.md
```

Future releases pick the new wording up from the annotation; past ones keep the
body you set here. The two diverge, and that is fine — the body is what readers
see.

## Calibration

The same release, before and after. Four commits, all in CI and packaging, zero
shipped code:

> **Releases publish again**
>
> v0.1.2 was tagged and built, then published nothing: the release job died
> rendering its notes, so the copper producer path that release introduced […]
> reached no release page. Its packages ship here instead, alongside the two
> fixes that let them out.
>
> The notes ended without a trailing newline, which absorbed the delimiter the
> Actions runner reads them back with and failed the step before it could
> publish. Separately, every distro leg of CI had been failing on an unrelated
> crate's build script, where a cargo-cache clean deleted build-script binaries
> while the fingerprints marking them current survived.
>
> Both were faults in the plumbing around the recorder rather than in it:
> nothing about clipper's behaviour changes between v0.1.2 and this release.

160 words, two of three paragraphs on mechanism (rule 3), the one fact a reader
acts on buried mid-sentence (rule 4), and a defensive close that is one useful
clause stretched to a paragraph (rule 5).

> **Fix the release pipeline**
>
> v0.1.2 built its packages and then failed to publish them, so what that
> release introduced never reached anyone: `examples/cu-mcap-record`, a copper
> (cu29) sink task writing an MCAP recording that clipper tails live. Those
> packages ship here. The recorder is unchanged since v0.1.1 — upgrade for the
> example, not for new behaviour.

55 words: what happened, what you get, what to do.

## This repo

[`cliff.toml`](../../../cliff.toml) holds the template and its own header
documents the grouping rules. Pushing a `v*` tag runs
[`release.yml`](../../../.github/workflows/release.yml), which builds the
packages and writes the release body from the annotation. Job layout and the
`act` recipe live in the **`ci`** skill; the two-deb pipeline in **`packaging`**.

git-cliff is not in the dev shell — invoke it however your environment provides
it. The commands above name it plainly.
