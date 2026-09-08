//! The layered configuration file: two TOML files under the environment and the
//! command line.
//!
//! A setting resolves through four layers, weakest first — the **system** file
//! (`/etc/momentedge/clipper.toml`), the **per-run** file, the **environment**,
//! and the **command-line flag** — with the built-in default underneath all
//! four. This module owns the bottom two: it merges the files into one value per
//! key and says which file decided it. The top two belong to the CLI parser,
//! which takes what this returns as each argument's default, so a flag beats an
//! environment variable beats a file beats a default with no further wiring.
//! Layers merge per setting, not per file: a per-run file naming one key leaves
//! every other key to the layers below it.
//!
//! Both files share one schema and one loader, so every subcommand reads the
//! same file:
//!
//! ```toml
//! [settings]              # one key per flag: `--grace-secs` is `grace_secs`
//! record_dir = "/data/record"
//! grace_secs = 45
//!
//! [topics]                # which topics a clip is cut from (`crate::select`)
//! include = ["/imu/data"]
//! exclude_regex = "^/diagnostics"
//! ```
//!
//! **The system file may set every key; a per-run file may not.** A key naming
//! the machine the recorder runs on or the resources it may spend there is
//! [`Scope::SystemOnly`], and a per-run file setting one is refused by name
//! ([`Layered::refusals`]) with the system value left standing. The environment
//! and the flags are unscoped — whoever launches the process commands both
//! anyway.
//!
//! **A file that is not there is not an error.** Both files are optional in
//! every combination; a path an operator named explicitly and that is not there
//! is warned about, since it is likelier a typo than a choice. A file that *is*
//! there and does not parse, names a key that does not exist, or carries a value
//! of the wrong type fails the run, so a misspelled key never silently does
//! nothing.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result, bail};
use log::warn;
use serde::Deserialize;

use crate::select::{ChannelSelection, Spec};

/// Where the system configuration file is read from when nothing moves it.
pub const SYSTEM_CONFIG_PATH: &str = "/etc/momentedge/clipper.toml";

/// The layer that decided a setting's value.
///
/// The variants are in resolution order — the strongest layer that names a
/// setting wins it — and `Ord` is that order, so the documented precedence is a
/// property of the type rather than of the prose. This module produces only
/// `Builtin`, `System` and `Run`; the CLI parser reports the top two.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Layer {
    /// Nothing named the setting; the value compiled into the parser stands.
    Builtin,
    /// The system configuration file.
    System,
    /// The per-run configuration file.
    Run,
    /// A `MOMENTEDGE_*` environment variable.
    Env,
    /// A command-line flag.
    Flag,
}

/// Whether a per-run file may set a key.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Scope {
    /// Either file may set it: the key describes one job.
    Any,
    /// The system file alone. The key describes the machine the recorder runs
    /// on or the resources it may spend there, which one job does not get to
    /// redecide.
    SystemOnly,
}

/// Every key a `[settings]` table may carry, and who may set it.
///
/// The key set is the union of the modes' flags — the key is the flag without
/// its `--` and with `-` written `_` — so a file names settings by the same
/// words the command line does. The list is explicit rather than derived from
/// the parser because the *scope* is a decision no argument definition carries;
/// the recorder holds the two sides together with a test that every mode's
/// arguments appear here and nothing here is a stranger to every mode.
///
/// `--config`, `--system-config` and `--print-config` are deliberately absent:
/// they are about the configuration rather than in it, so no file can name
/// another file.
const SETTINGS: &[(&str, Scope)] = &[
    // `clipper tail`
    ("record_dir", Scope::SystemOnly),
    ("out_dir", Scope::Any),
    ("interface", Scope::SystemOnly),
    ("time_source", Scope::SystemOnly),
    ("grace_secs", Scope::Any),
    ("clip_compression", Scope::Any),
    ("extract_parallelism", Scope::SystemOnly),
    ("watch_old_files_duration", Scope::SystemOnly),
    ("delete_old_files", Scope::SystemOnly),
    // `clipper clip`
    ("recording", Scope::Any),
    ("trigger_time", Scope::Any),
    ("preroll", Scope::Any),
    ("postroll", Scope::Any),
    ("trigger_name", Scope::Any),
    ("trigger_description", Scope::Any),
];

/// Who may set `key`, or `None` for a key no mode has an argument for.
pub fn scope_of(key: &str) -> Option<Scope> {
    SETTINGS
        .iter()
        .find_map(|(name, scope)| (*name == key).then_some(*scope))
}

/// Every `[settings]` key, in the order this module lists them.
pub fn setting_keys() -> impl Iterator<Item = &'static str> {
    SETTINGS.iter().map(|(name, _)| *name)
}

/// The `[topics]` keys, in the order a report lists them.
const TOPIC_KEYS: [&str; 6] = [
    "all",
    "include",
    "include_regex",
    "exclude",
    "exclude_regex",
    "exclude_trigger_topic",
];

/// One resolved value and the layer that decided it.
///
/// The value is text because that is what the CLI parser takes as a default and
/// what a report prints; the parser owns turning it into a path, a count or an
/// enum, so a file's value is checked by exactly the code a flag's value is.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Setting {
    value: String,
    layer: Layer,
}

impl Setting {
    /// The value as the CLI parser takes it.
    pub fn value(&self) -> &str {
        &self.value
    }

    /// The layer that decided it.
    pub fn layer(&self) -> Layer {
        self.layer
    }
}

/// The two configuration files merged: the default each setting takes into the
/// CLI parser, the topic selection, and what a per-run file was refused.
#[derive(Debug)]
pub struct Layered {
    system_path: PathBuf,
    run_path: Option<PathBuf>,
    settings: BTreeMap<String, Setting>,
    topics: Vec<(&'static str, Setting)>,
    selection: ChannelSelection,
    refusals: Vec<String>,
}

impl Layered {
    /// Read both files and merge them.
    ///
    /// `system` moves the system file off [`SYSTEM_CONFIG_PATH`]; `run` names
    /// the per-run file, which has no built-in location. A file missing from the
    /// built-in location is ordinary and silent; one missing from a path an
    /// operator named is warned about and then treated the same way. Anything
    /// else about a file that exists — unreadable, unparseable, an unknown key,
    /// a value of the wrong shape, an uncompilable pattern — fails, naming the
    /// file and the key.
    pub fn load(system: Option<&Path>, run: Option<&Path>) -> Result<Self> {
        let system_path =
            system.map_or_else(|| PathBuf::from(SYSTEM_CONFIG_PATH), Path::to_path_buf);
        let system_file = read_file(&system_path, system.is_some())?;
        let run_file = run.map(|p| read_file(p, true)).transpose()?.flatten();

        let mut settings = BTreeMap::new();
        let mut refusals = Vec::new();
        if let Some(file) = &system_file {
            for (key, value) in &file.settings {
                settings.insert(key.clone(), Setting::new(value, Layer::System));
            }
        }
        if let (Some(file), Some(path)) = (&run_file, run) {
            let path = path.display();
            for (key, value) in &file.settings {
                // A refusal is reported and dropped, not fatal: the run is
                // legitimate, it simply does not get to decide this key.
                if scope_of(key) == Some(Scope::SystemOnly) {
                    let refusal = format!(
                        "the per-run configuration file {path} may not set `{key}`; \
                         the system configuration decides it"
                    );
                    warn!("{refusal}");
                    refusals.push(refusal);
                    continue;
                }
                settings.insert(key.clone(), Setting::new(value, Layer::Run));
            }
        }

        // Which files were actually read, for the one error the merge can
        // raise: a pattern the `regex` crate will not take names its key, and
        // an operator still has to be told which file to open.
        let read: Vec<String> = [
            system_file
                .as_ref()
                .map(|_| system_path.display().to_string()),
            run_file
                .as_ref()
                .zip(run)
                .map(|(_, path)| path.display().to_string()),
        ]
        .into_iter()
        .flatten()
        .collect();
        let system_topics = system_file.map(|f| f.topics).unwrap_or_default();
        let run_topics = run_file.map(|f| f.topics).unwrap_or_default();
        let (spec, topics) = merge_topics(&system_topics, &run_topics);
        let selection = ChannelSelection::try_from(spec)
            .with_context(|| format!("in the [topics] table of {}", read.join(" and ")))?;

        Ok(Layered {
            system_path,
            run_path: run.map(Path::to_path_buf),
            settings,
            topics,
            selection,
            refusals,
        })
    }

    /// The default the CLI parser should give `key`, or `None` where no file
    /// named it and the parser's own default stands.
    pub fn setting(&self, key: &str) -> Option<&Setting> {
        self.settings.get(key)
    }

    /// Every setting a file named, in key order.
    pub fn settings(&self) -> impl Iterator<Item = (&str, &Setting)> {
        self.settings.iter().map(|(k, v)| (k.as_str(), v))
    }

    /// Every `[topics]` key with the value the selection actually uses and the
    /// layer that decided it — including the keys no file named, which is what
    /// makes the report the whole selection rather than the part somebody
    /// happened to write down.
    pub fn topics(&self) -> impl Iterator<Item = (&str, &Setting)> {
        self.topics.iter().map(|(k, v)| (*k, v))
    }

    /// Which topics a clip is cut from.
    pub fn selection(&self) -> &ChannelSelection {
        &self.selection
    }

    /// Take the selection out, for handing to the cut path.
    pub fn into_selection(self) -> ChannelSelection {
        self.selection
    }

    /// The per-run keys refused for being the system's to decide, one message
    /// each, already logged. A report repeats them so the run's configuration
    /// and the reason it is not what the per-run file asked for are read
    /// together.
    pub fn refusals(&self) -> &[String] {
        &self.refusals
    }

    /// How a report names `layer` — with the file's path for the two file
    /// layers, since "system file" alone does not say which file was read.
    pub fn origin(&self, layer: Layer) -> String {
        match layer {
            Layer::Builtin => "built-in default".to_string(),
            Layer::System => format!("system file {}", self.system_path.display()),
            Layer::Run => match &self.run_path {
                Some(path) => format!("per-run file {}", path.display()),
                None => "per-run file".to_string(),
            },
            Layer::Env => "environment".to_string(),
            Layer::Flag => "flag".to_string(),
        }
    }
}

impl Setting {
    fn new(value: &str, layer: Layer) -> Self {
        Setting {
            value: value.to_string(),
            layer,
        }
    }
}

/// One configuration file as it was written, with its `[settings]` values
/// already rendered as the text the CLI parser takes.
#[derive(Debug)]
struct ParsedFile {
    settings: BTreeMap<String, String>,
    topics: TopicsFile,
}

/// The file's shape for serde. `deny_unknown_fields` turns a misspelled table
/// name into a startup error; the `[settings]` keys are checked against
/// [`SETTINGS`] by hand, since the table is one map rather than a struct.
#[derive(Debug, Default, Deserialize)]
#[serde(deny_unknown_fields)]
struct ConfigFile {
    #[serde(default)]
    settings: BTreeMap<String, toml::Value>,
    #[serde(default)]
    topics: TopicsFile,
}

/// The `[topics]` table as written: every key an `Option`, so "absent" and
/// "set to the default" stay distinguishable across the merge.
#[derive(Clone, Debug, Default, Deserialize)]
#[serde(deny_unknown_fields)]
struct TopicsFile {
    all: Option<bool>,
    include: Option<Vec<String>>,
    include_regex: Option<String>,
    exclude: Option<Vec<String>>,
    exclude_regex: Option<String>,
    exclude_trigger_topic: Option<bool>,
}

/// Read and validate one file. `Ok(None)` is "there is no such file", legal in
/// every combination; `named` only decides whether that is worth a warning.
fn read_file(path: &Path, named: bool) -> Result<Option<ParsedFile>> {
    let text = match std::fs::read_to_string(path) {
        Ok(text) => text,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
            if named {
                warn!(
                    "configuration file {} does not exist; continuing without it",
                    path.display()
                );
            }
            return Ok(None);
        }
        Err(e) => {
            return Err(e).with_context(|| format!("reading {}", path.display()));
        }
    };
    let file: ConfigFile =
        toml::from_str(&text).with_context(|| format!("parsing {}", path.display()))?;
    let mut settings = BTreeMap::new();
    for (key, value) in &file.settings {
        let rendered = render(key, value).with_context(|| format!("in {}", path.display()))?;
        settings.insert(key.clone(), rendered);
    }
    Ok(Some(ParsedFile {
        settings,
        topics: file.topics,
    }))
}

/// One `[settings]` value as the text the CLI parser takes, refusing a key no
/// mode has an argument for and a value no argument could take.
///
/// Rendering rather than typing is what keeps the two sides honest: the file's
/// value is parsed by the same argument definition a flag's value is, so a file
/// cannot smuggle in a value the command line would have rejected.
fn render(key: &str, value: &toml::Value) -> Result<String> {
    if scope_of(key).is_none() {
        bail!(
            "unknown key `settings.{key}`; the keys are: {}",
            setting_keys().collect::<Vec<_>>().join(", ")
        );
    }
    #[expect(
        clippy::wildcard_enum_match_arm,
        reason = "toml::Value is foreign; every shape a setting cannot take is one refusal"
    )]
    match value {
        toml::Value::String(s) => Ok(s.clone()),
        toml::Value::Integer(i) => Ok(i.to_string()),
        toml::Value::Boolean(b) => Ok(b.to_string()),
        other => bail!(
            "`settings.{key}` is {}; it must be a string, an integer or a boolean",
            other.type_str()
        ),
    }
}

/// Merge the two files' `[topics]` tables per key — the per-run file's value
/// where it set one, the system file's otherwise — into the spec the selection
/// is compiled from and the report of what each key resolved to.
fn merge_topics(system: &TopicsFile, run: &TopicsFile) -> (Spec, Vec<(&'static str, Setting)>) {
    /// The value and the layer for one key: the per-run file's if it set one,
    /// else the system file's, else nothing at the built-in layer.
    fn pick<T: Clone>(system: &Option<T>, run: &Option<T>) -> (Option<T>, Layer) {
        match (system, run) {
            (_, Some(v)) => (Some(v.clone()), Layer::Run),
            (Some(v), None) => (Some(v.clone()), Layer::System),
            (None, None) => (None, Layer::Builtin),
        }
    }

    let (all, all_layer) = pick(&system.all, &run.all);
    let (include, include_layer) = pick(&system.include, &run.include);
    let (include_regex, include_regex_layer) = pick(&system.include_regex, &run.include_regex);
    let (exclude, exclude_layer) = pick(&system.exclude, &run.exclude);
    let (exclude_regex, exclude_regex_layer) = pick(&system.exclude_regex, &run.exclude_regex);
    let (exclude_trigger, exclude_trigger_layer) =
        pick(&system.exclude_trigger_topic, &run.exclude_trigger_topic);

    let spec = Spec {
        all,
        include: include.clone().unwrap_or_default(),
        include_regex: include_regex.clone(),
        exclude: exclude.clone().unwrap_or_default(),
        exclude_regex: exclude_regex.clone(),
        exclude_trigger_topic: exclude_trigger.unwrap_or(false),
    };
    // `all` is reported as the value the selection uses, which with no key of
    // its own is the one the include keys imply.
    let effective_all = spec
        .all
        .unwrap_or(spec.include.is_empty() && spec.include_regex.is_none());
    let report = vec![
        ("all", Setting::new(&effective_all.to_string(), all_layer)),
        (
            "include",
            Setting::new(&render_list(&spec.include), include_layer),
        ),
        (
            "include_regex",
            Setting::new(&render_pattern(&spec.include_regex), include_regex_layer),
        ),
        (
            "exclude",
            Setting::new(&render_list(&spec.exclude), exclude_layer),
        ),
        (
            "exclude_regex",
            Setting::new(&render_pattern(&spec.exclude_regex), exclude_regex_layer),
        ),
        (
            "exclude_trigger_topic",
            Setting::new(
                &spec.exclude_trigger_topic.to_string(),
                exclude_trigger_layer,
            ),
        ),
    ];
    debug_assert_eq!(
        report.iter().map(|(k, _)| *k).collect::<Vec<_>>(),
        TOPIC_KEYS,
        "the report covers every topic key"
    );
    (spec, report)
}

/// A topic list as a report prints it: `["/a", "/b"]`, `[]` when empty.
fn render_list(topics: &[String]) -> String {
    let quoted: Vec<String> = topics.iter().map(|t| format!("{t:?}")).collect();
    format!("[{}]", quoted.join(", "))
}

/// A pattern as a report prints it, or `(unset)` where there is none — an empty
/// pattern matches everything, so it cannot stand in for absence.
fn render_pattern(pattern: &Option<String>) -> String {
    pattern.clone().unwrap_or_else(|| "(unset)".to_string())
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::indexing_slicing,
        reason = "a failed unwrap or a panicking index is a failing test"
    )]

    use super::*;
    use crate::testing::test_dir;

    /// Write `text` to `name` under a fresh scratch directory and hand back the
    /// path, so each test's files are its own.
    fn file(dir: &Path, name: &str, text: &str) -> PathBuf {
        let path = dir.join(name);
        std::fs::write(&path, text).unwrap();
        path
    }

    fn value_of<'a>(layered: &'a Layered, key: &str) -> &'a str {
        layered
            .setting(key)
            .unwrap_or_else(|| panic!("{key} is set"))
            .value()
    }

    fn topic_of<'a>(layered: &'a Layered, key: &str) -> &'a Setting {
        layered
            .topics()
            .find_map(|(k, v)| (k == key).then_some(v))
            .unwrap_or_else(|| panic!("{key} is reported"))
    }

    #[test]
    fn both_files_missing_is_legal_and_sets_nothing() -> Result<()> {
        let dir = test_dir("config-missing")?;
        let layered = Layered::load(
            Some(&dir.join("no-system.toml")),
            Some(&dir.join("no-run.toml")),
        )?;
        assert_eq!(layered.settings().count(), 0);
        assert!(layered.refusals().is_empty());
        assert!(layered.selection().selects("/imu/data"));
        std::fs::remove_dir_all(dir)?;
        Ok(())
    }

    #[test]
    fn either_file_alone_is_legal() -> Result<()> {
        let dir = test_dir("config-one-file")?;
        let system = file(&dir, "system.toml", "[settings]\ngrace_secs = 45\n");
        let run = file(&dir, "run.toml", "[settings]\nout_dir = \"/tmp/clips\"\n");

        let system_only = Layered::load(Some(&system), Some(&dir.join("absent.toml")))?;
        assert_eq!(value_of(&system_only, "grace_secs"), "45");

        let run_only = Layered::load(Some(&dir.join("absent.toml")), Some(&run))?;
        assert_eq!(value_of(&run_only, "out_dir"), "/tmp/clips");

        std::fs::remove_dir_all(dir)?;
        Ok(())
    }

    /// The bottom two of the four layers, in order: the per-run file wins the
    /// key both files set, and leaves the keys it says nothing about to the
    /// system file.
    #[test]
    fn the_run_file_wins_the_keys_it_may_set_and_leaves_the_rest() -> Result<()> {
        let dir = test_dir("config-layers")?;
        let system = file(
            &dir,
            "system.toml",
            "[settings]\ngrace_secs = 45\nout_dir = \"/data/clipped\"\n",
        );
        let run = file(&dir, "run.toml", "[settings]\nout_dir = \"/tmp/clips\"\n");
        let layered = Layered::load(Some(&system), Some(&run))?;

        assert_eq!(value_of(&layered, "out_dir"), "/tmp/clips");
        assert_eq!(layered.setting("out_dir").unwrap().layer(), Layer::Run);
        assert_eq!(value_of(&layered, "grace_secs"), "45");
        assert_eq!(
            layered.setting("grace_secs").unwrap().layer(),
            Layer::System
        );

        std::fs::remove_dir_all(dir)?;
        Ok(())
    }

    #[test]
    fn a_run_file_setting_a_system_key_is_refused_by_name() -> Result<()> {
        let dir = test_dir("config-refusal")?;
        let system = file(
            &dir,
            "system.toml",
            "[settings]\nrecord_dir = \"/data/record\"\n",
        );
        let run = file(
            &dir,
            "run.toml",
            "[settings]\nrecord_dir = \"/tmp/mine\"\nout_dir = \"/tmp/clips\"\n",
        );
        let layered = Layered::load(Some(&system), Some(&run))?;

        assert_eq!(
            value_of(&layered, "record_dir"),
            "/data/record",
            "the system value stands"
        );
        assert_eq!(
            layered.setting("record_dir").unwrap().layer(),
            Layer::System
        );
        assert_eq!(layered.refusals().len(), 1);
        assert!(
            layered.refusals()[0].contains("record_dir"),
            "the refusal names the key: {}",
            layered.refusals()[0]
        );
        assert!(
            layered.refusals()[0].contains("run.toml"),
            "the refusal names the file: {}",
            layered.refusals()[0]
        );
        assert_eq!(
            value_of(&layered, "out_dir"),
            "/tmp/clips",
            "the refusal costs the run file only the key it may not set"
        );

        std::fs::remove_dir_all(dir)?;
        Ok(())
    }

    /// With no system file at all there is nothing to stand, so the built-in
    /// default does — the refusal is never a promotion.
    #[test]
    fn a_refused_key_falls_to_the_built_in_default_when_no_system_file_set_it() -> Result<()> {
        let dir = test_dir("config-refusal-default")?;
        let run = file(&dir, "run.toml", "[settings]\ndelete_old_files = true\n");
        let layered = Layered::load(Some(&dir.join("absent.toml")), Some(&run))?;
        assert!(layered.setting("delete_old_files").is_none());
        assert_eq!(layered.refusals().len(), 1);
        std::fs::remove_dir_all(dir)?;
        Ok(())
    }

    #[test]
    fn every_scalar_shape_renders_as_the_parser_takes_it() -> Result<()> {
        let dir = test_dir("config-scalars")?;
        let system = file(
            &dir,
            "system.toml",
            "[settings]\ngrace_secs = 45\ndelete_old_files = true\ntime_source = \"publish\"\n",
        );
        let layered = Layered::load(Some(&system), None)?;
        assert_eq!(value_of(&layered, "grace_secs"), "45");
        assert_eq!(value_of(&layered, "delete_old_files"), "true");
        assert_eq!(value_of(&layered, "time_source"), "publish");
        std::fs::remove_dir_all(dir)?;
        Ok(())
    }

    #[test]
    fn a_present_but_broken_file_fails_naming_the_file_and_the_key() -> Result<()> {
        let dir = test_dir("config-broken")?;
        let cases = [
            ("bad-syntax.toml", "[settings\n", "parsing"),
            (
                "unknown-key.toml",
                "[settings]\ngrace_sec = 3\n",
                "grace_sec",
            ),
            ("unknown-table.toml", "[topic]\nall = true\n", "topic"),
            (
                "wrong-type.toml",
                "[settings]\ngrace_secs = [1, 2]\n",
                "grace_secs",
            ),
            (
                "bad-pattern.toml",
                "[topics]\nexclude_regex = \"[\"\n",
                "exclude_regex",
            ),
        ];
        for (name, text, needle) in cases {
            let path = file(&dir, name, text);
            let err = format!(
                "{:#}",
                Layered::load(Some(&path), None).expect_err("this file must not load")
            );
            assert!(err.contains(needle), "{name}: {err}");
            assert!(err.contains(name), "{name} is named: {err}");
        }
        std::fs::remove_dir_all(dir)?;
        Ok(())
    }

    #[test]
    fn topic_keys_merge_per_key_and_report_their_layer() -> Result<()> {
        let dir = test_dir("config-topics")?;
        let system = file(
            &dir,
            "system.toml",
            "[topics]\nexclude_regex = \"^/diagnostics\"\nexclude = [\"/tf_static\"]\n",
        );
        let run = file(
            &dir,
            "run.toml",
            "[topics]\ninclude = [\"/imu/data\"]\nexclude = []\n",
        );
        let layered = Layered::load(Some(&system), Some(&run))?;

        assert_eq!(topic_of(&layered, "include").value(), "[\"/imu/data\"]");
        assert_eq!(topic_of(&layered, "include").layer(), Layer::Run);
        assert_eq!(topic_of(&layered, "exclude").value(), "[]");
        assert_eq!(
            topic_of(&layered, "exclude").layer(),
            Layer::Run,
            "a key the run file set to empty is still the run file's"
        );
        assert_eq!(topic_of(&layered, "exclude_regex").value(), "^/diagnostics");
        assert_eq!(topic_of(&layered, "exclude_regex").layer(), Layer::System);
        assert_eq!(
            topic_of(&layered, "all").value(),
            "false",
            "an include key implies all = false"
        );
        assert_eq!(topic_of(&layered, "all").layer(), Layer::Builtin);
        assert_eq!(topic_of(&layered, "include_regex").value(), "(unset)");

        let selection = layered.selection();
        assert!(selection.selects("/imu/data"));
        assert!(!selection.selects("/camera/image_raw"));
        assert!(!selection.selects("/diagnostics"));

        std::fs::remove_dir_all(dir)?;
        Ok(())
    }

    #[test]
    fn the_report_covers_every_topic_key_with_no_file_at_all() -> Result<()> {
        let dir = test_dir("config-topics-default")?;
        let layered = Layered::load(Some(&dir.join("absent.toml")), None)?;
        let reported: Vec<&str> = layered.topics().map(|(k, _)| k).collect();
        assert_eq!(reported, TOPIC_KEYS);
        assert!(layered.topics().all(|(_, v)| v.layer() == Layer::Builtin));
        assert_eq!(topic_of(&layered, "all").value(), "true");
        std::fs::remove_dir_all(dir)?;
        Ok(())
    }

    #[test]
    fn origin_names_the_file_each_layer_read() -> Result<()> {
        let dir = test_dir("config-origin")?;
        let system = file(&dir, "system.toml", "");
        let run = file(&dir, "run.toml", "");
        let layered = Layered::load(Some(&system), Some(&run))?;
        assert_eq!(layered.origin(Layer::Builtin), "built-in default");
        assert!(layered.origin(Layer::System).contains("system.toml"));
        assert!(layered.origin(Layer::Run).contains("run.toml"));
        assert_eq!(layered.origin(Layer::Env), "environment");
        assert_eq!(layered.origin(Layer::Flag), "flag");
        std::fs::remove_dir_all(dir)?;
        Ok(())
    }

    #[test]
    fn the_default_system_path_is_the_documented_one() {
        assert_eq!(SYSTEM_CONFIG_PATH, "/etc/momentedge/clipper.toml");
    }

    /// The layer order is the documented resolution order, and `Ord` is what
    /// says so — a report and a merge both lean on it.
    #[test]
    fn layers_order_weakest_to_strongest() {
        let mut layers = [
            Layer::Flag,
            Layer::Builtin,
            Layer::Env,
            Layer::Run,
            Layer::System,
        ];
        layers.sort_unstable();
        assert_eq!(
            layers,
            [
                Layer::Builtin,
                Layer::System,
                Layer::Run,
                Layer::Env,
                Layer::Flag
            ]
        );
    }

    /// A file may not name a file: the three flags that locate configuration
    /// are not keys, so a file cannot redirect the loader that read it.
    #[test]
    fn the_configuration_flags_are_not_settings_keys() {
        for key in ["config", "system_config", "print_config"] {
            assert_eq!(scope_of(key), None, "{key} must not be a settings key");
        }
    }
}
