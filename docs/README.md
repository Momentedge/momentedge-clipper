# clipper documentation

One page per question a user arrives with. For what clipper is and how to cut a
first clip, start at the [README](../README.md).

| Page | What it answers |
|---|---|
| [Try it without a robot](try-it.md) | cutting a real clip on a laptop: a Rust toolchain, no ROS, two minutes |
| [Installing and building](install.md) | the `.deb`s, the two cargo builds, the nix package, a target build, the upgrade note |
| [Configuration](configuration.md) | every flag, environment variable and configuration-file key, and which layer wins |
| [Triggers and time](triggers-and-time.md) | what a trigger is, the two ways one reaches clipper, which clock a window lives on |
| [`clipper clip`](clip-command.md) | cutting from a recording nobody is writing any more: bag directories, refusals, re-runs |
| [What a clip carries](clip-manifest.md) | the clip directory, every field of its `clip_metadata.yaml`, how a clip's id is derived |
| [Operating clipper](operating.md) | shutdown, logs, retention, a recording that stops producing clips, overload, tuning |
| [What it costs to run](performance.md) | measured CPU, memory and disk on a Jetson, and what clipper does to the recorder |

Inside clipper — threads, tailing, recovery — is
[ARCHITECTURE.md](../ARCHITECTURE.md); the words these pages lean on are defined
once in the [glossary](../CONTEXT.md).
