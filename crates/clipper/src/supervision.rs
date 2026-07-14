//! Spawning threads whose death is observable.
//!
//! A thread that dies silently is the failure mode this module exists to
//! prevent: a dead tailer degrades every clip to a grace-timeout cut, a dead
//! interface stops acting on triggers, and neither announces itself. Every
//! long-lived thread is therefore spawned through [`spawn_supervised`], which
//! pairs it with a channel carrying its verdict, and a watcher selects over
//! those channels — see `supervise` in `main`, which encodes what the recorder
//! does about each verdict.
//!
//! Two ways a thread can end, and both must surface:
//!
//! - it **returns**, and its value (typically an `anyhow::Result<()>`) arrives
//!   on the channel;
//! - it **panics**, and the closure unwinds without sending, so the channel
//!   disconnects instead. [`harvest_panic`] then joins the handle — immediate,
//!   since the disconnect already proves the thread is dead — and lifts the
//!   panic payload into the error chain via [`panic_text`].

use std::thread::{self, JoinHandle};

use crossbeam_channel::{Receiver, bounded};

/// One supervised thread: the channel its closure's return value arrives on,
/// and the join handle a watcher harvests a panic payload from after a
/// disconnect.
pub(crate) type Supervised<T> = (Receiver<T>, JoinHandle<()>);

/// Spawn a named thread whose return value arrives on the paired channel.
///
/// A watcher selects on that channel: a received value is the thread's verdict
/// (clean return or typed error); a disconnect without a value means the closure
/// unwound (panicked) before it could send, and the join handle then carries the
/// payload for [`harvest_panic`].
pub(crate) fn spawn_supervised<T: Send + 'static>(
    name: &str,
    f: impl FnOnce() -> T + Send + 'static,
) -> Supervised<T> {
    let (tx, rx) = bounded(1);
    let handle = thread::Builder::new()
        .name(name.to_string())
        .spawn(move || {
            // The send fails only when supervision is already gone, and the
            // value is then moot.
            let _ = tx.send(f());
        })
        .expect("spawning thread");
    (rx, handle)
}

/// Render a panic payload (from [`JoinHandle::join`] or
/// [`std::panic::catch_unwind`]) as text: panics carry a `&str` or `String`
/// message in practice; anything else gets a placeholder.
pub(crate) fn panic_text(payload: &(dyn std::any::Any + Send)) -> String {
    payload
        .downcast_ref::<&str>()
        .map(|s| s.to_string())
        .or_else(|| payload.downcast_ref::<String>().cloned())
        .unwrap_or_else(|| "non-string panic payload".to_string())
}

/// The error for a supervised thread whose result channel disconnected without
/// a value: the closure unwound before it could send, so the join — immediate,
/// the disconnect proves the thread is already dead — carries the panic payload.
pub(crate) fn harvest_panic(handle: JoinHandle<()>) -> anyhow::Error {
    match handle.join() {
        Err(payload) => anyhow::anyhow!("thread panicked: {}", panic_text(payload.as_ref())),
        // Unreachable for spawn_supervised threads (a returning closure always
        // sends first), but a sane shape regardless.
        Ok(()) => anyhow::anyhow!("thread exited without reporting a result"),
    }
}
