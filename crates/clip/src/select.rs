//! Which of a recording's topics a clip is cut from.
//!
//! A recording carries everything the robot published; a clip rarely needs all
//! of it. [`ChannelSelection`] is the decision, taken once per topic name, that
//! the cut path applies at the only two places it can matter — where a channel
//! is registered in the output and where a message is copied — so an excluded
//! topic contributes neither a channel, nor a schema, nor a message, nor a
//! manifest key to the clip.
//!
//! The type is a *decision*, not a parser: it holds compiled patterns and
//! answers [`ChannelSelection::selects`]. Turning configuration text into one is
//! [`Spec`]'s `TryFrom`, which is where a malformed pattern is rejected, and
//! reading that text out of a file is [`crate::config`]'s.
//!
//! **Two rules are not the configuration's to make.** clipper's own
//! announcement topic ([`ANNOUNCE_TOPIC`]) is never copied into a clip — a clip
//! is about the robot, not about clipper announcing clips about the robot — and
//! the trigger topic ([`crate::trigger::TRIGGER_TOPIC`]) is kept unless
//! [`Spec::exclude_trigger_topic`] says otherwise, so a clip carries the trigger
//! that asked for it by default.

use anyhow::{Context, Result};
use regex::Regex;

use crate::trigger::{ANNOUNCE_TOPIC, TRIGGER_TOPIC};

/// The topic-selection keys as a configuration file spells them, before any of
/// them is checked: the flat mirror [`ChannelSelection`] is built from.
///
/// `all` is an `Option` because its default is not a constant: it is `true`
/// while no include key is set (take everything) and `false` once one is (take
/// what the include keys name), so `None` means "whichever the rest of this
/// spec implies" and `Some` is an operator overruling that.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Spec {
    /// Take every topic. Defaults per the note above; `Some(true)` beside an
    /// include list widens the clip back to everything.
    pub all: Option<bool>,
    /// Exact topic names to keep.
    pub include: Vec<String>,
    /// Keep every topic this regular expression matches.
    pub include_regex: Option<String>,
    /// Exact topic names to drop, whatever selected them.
    pub exclude: Vec<String>,
    /// Drop every topic this regular expression matches, whatever selected it.
    pub exclude_regex: Option<String>,
    /// Drop the trigger topic too. Off by default, so a clip keeps the triggers
    /// recorded alongside the data unless it is told not to.
    pub exclude_trigger_topic: bool,
}

/// Which topics a clip is cut from: the include side, the exclude side, and the
/// two rules no configuration reaches.
///
/// [`Default`] is every topic — what a run with no configuration file cuts, and
/// still not the announcement topic.
#[derive(Clone, Debug)]
pub struct ChannelSelection {
    all: bool,
    include: Vec<String>,
    include_regex: Option<Regex>,
    exclude: Vec<String>,
    exclude_regex: Option<Regex>,
    exclude_trigger_topic: bool,
}

impl Default for ChannelSelection {
    fn default() -> Self {
        Self {
            all: true,
            include: Vec::new(),
            include_regex: None,
            exclude: Vec::new(),
            exclude_regex: None,
            exclude_trigger_topic: false,
        }
    }
}

impl TryFrom<Spec> for ChannelSelection {
    type Error = anyhow::Error;

    /// Compile the patterns and resolve `all`, rejecting a regular expression
    /// the `regex` crate will not take with the key that carried it named — the
    /// one boundary where selection text becomes a decision.
    fn try_from(spec: Spec) -> Result<Self> {
        let named = |key: &'static str, pattern: Option<String>| -> Result<Option<Regex>> {
            pattern
                .map(|p| Regex::new(&p).with_context(|| format!("topics.{key} = {p:?}")))
                .transpose()
        };
        let names_an_include = !spec.include.is_empty() || spec.include_regex.is_some();
        Ok(Self {
            all: spec.all.unwrap_or(!names_an_include),
            include: spec.include,
            include_regex: named("include_regex", spec.include_regex)?,
            exclude: spec.exclude,
            exclude_regex: named("exclude_regex", spec.exclude_regex)?,
            exclude_trigger_topic: spec.exclude_trigger_topic,
        })
    }
}

impl ChannelSelection {
    /// Whether a clip is cut from `topic`.
    ///
    /// Dropping wins over selecting, the rule `ros2 bag record` applies to its
    /// own exclusions, so an operator can name a wide include and carve one
    /// topic back out of it. The two fixed rules are tested first: they are not
    /// exclusions an include can outrank.
    pub fn selects(&self, topic: &str) -> bool {
        if topic == ANNOUNCE_TOPIC {
            return false;
        }
        if self.exclude_trigger_topic && topic == TRIGGER_TOPIC {
            return false;
        }
        if self.exclude.iter().any(|t| t == topic) {
            return false;
        }
        if self
            .exclude_regex
            .as_ref()
            .is_some_and(|re| re.is_match(topic))
        {
            return false;
        }
        self.all
            || self.include.iter().any(|t| t == topic)
            || self
                .include_regex
                .as_ref()
                .is_some_and(|re| re.is_match(topic))
    }

    /// Whether this selection can refuse anything at all — false only for the
    /// [`Default`] one, which takes every topic but the announcement.
    ///
    /// The cut path logs what it dropped; a run that configured nothing has
    /// nothing to say about it.
    pub fn is_narrowing(&self) -> bool {
        !self.all
            || self.exclude_trigger_topic
            || !self.exclude.is_empty()
            || self.exclude_regex.is_some()
    }
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::unwrap_used,
        clippy::expect_used,
        reason = "a failed unwrap is a failing test"
    )]

    use super::*;

    fn selection(spec: Spec) -> ChannelSelection {
        ChannelSelection::try_from(spec).unwrap()
    }

    #[test]
    fn the_default_takes_every_topic_but_the_announcement() {
        let sel = ChannelSelection::default();
        assert!(sel.selects("/camera/image_raw"));
        assert!(sel.selects(TRIGGER_TOPIC), "triggers are kept by default");
        assert!(!sel.selects(ANNOUNCE_TOPIC));
        assert!(!sel.is_narrowing());
    }

    /// The announcement topic is not an exclusion an include can outrank: it is
    /// refused however hard the configuration asks for it.
    #[test]
    fn the_announcement_topic_is_never_selected() {
        for spec in [
            Spec {
                all: Some(true),
                ..Spec::default()
            },
            Spec {
                include: vec![ANNOUNCE_TOPIC.to_string()],
                ..Spec::default()
            },
            Spec {
                include_regex: Some("^/events/".to_string()),
                ..Spec::default()
            },
        ] {
            assert!(!selection(spec.clone()).selects(ANNOUNCE_TOPIC), "{spec:?}");
        }
    }

    #[test]
    fn the_trigger_topic_goes_only_when_its_own_key_says_so() {
        assert!(selection(Spec::default()).selects(TRIGGER_TOPIC));
        assert!(
            !selection(Spec {
                exclude_trigger_topic: true,
                ..Spec::default()
            })
            .selects(TRIGGER_TOPIC)
        );
        // …and the key governs that topic alone.
        assert!(
            selection(Spec {
                exclude_trigger_topic: true,
                ..Spec::default()
            })
            .selects("/imu/data")
        );
    }

    #[test]
    fn an_include_list_narrows_to_itself() {
        let sel = selection(Spec {
            include: vec!["/imu/data".to_string()],
            ..Spec::default()
        });
        assert!(sel.selects("/imu/data"));
        assert!(!sel.selects("/camera/image_raw"));
        assert!(
            !sel.selects(TRIGGER_TOPIC),
            "an include list is the whole clip, triggers included"
        );
        assert!(sel.is_narrowing());
    }

    #[test]
    fn an_include_regex_narrows_to_what_it_matches() {
        let sel = selection(Spec {
            include_regex: Some("^/camera/".to_string()),
            ..Spec::default()
        });
        assert!(sel.selects("/camera/front/image_raw"));
        assert!(!sel.selects("/imu/data"));
    }

    #[test]
    fn all_beside_an_include_list_widens_back_to_everything() {
        let sel = selection(Spec {
            all: Some(true),
            include: vec!["/imu/data".to_string()],
            ..Spec::default()
        });
        assert!(sel.selects("/imu/data"));
        assert!(sel.selects("/camera/image_raw"));
        assert!(!sel.is_narrowing());
    }

    #[test]
    fn all_false_alone_selects_nothing() {
        let sel = selection(Spec {
            all: Some(false),
            ..Spec::default()
        });
        assert!(!sel.selects("/imu/data"));
        assert!(!sel.selects(TRIGGER_TOPIC));
    }

    #[test]
    fn dropping_wins_over_selecting() {
        let sel = selection(Spec {
            include: vec!["/imu/data".to_string(), "/tf".to_string()],
            exclude: vec!["/tf".to_string()],
            ..Spec::default()
        });
        assert!(sel.selects("/imu/data"));
        assert!(!sel.selects("/tf"));

        let sel = selection(Spec {
            include_regex: Some("^/camera/".to_string()),
            exclude_regex: Some("depth".to_string()),
            ..Spec::default()
        });
        assert!(sel.selects("/camera/front/image_raw"));
        assert!(!sel.selects("/camera/front/depth"));
    }

    /// The patterns are unanchored: `^/camera/` matches at the start of a name,
    /// a bare word matches anywhere in one. The README says so, so a test holds
    /// it down.
    #[test]
    fn patterns_are_unanchored() {
        let sel = selection(Spec {
            exclude_regex: Some("diag".to_string()),
            ..Spec::default()
        });
        assert!(!sel.selects("/robot/diagnostics"));
        assert!(sel.selects("/robot/status"));
    }

    #[test]
    fn a_malformed_pattern_is_refused_by_its_key() {
        let err = ChannelSelection::try_from(Spec {
            exclude_regex: Some("[unterminated".to_string()),
            ..Spec::default()
        })
        .expect_err("an unterminated class is not a regex");
        let rendered = format!("{err:#}");
        assert!(
            rendered.contains("topics.exclude_regex"),
            "the key is named: {rendered}"
        );
        assert!(
            ChannelSelection::try_from(Spec {
                include_regex: Some("(".to_string()),
                ..Spec::default()
            })
            .is_err()
        );
    }
}
