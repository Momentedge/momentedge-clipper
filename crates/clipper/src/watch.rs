//! A minimal blocking analogue of a watch channel: one shared value whose
//! observers either read the current state or block until a predicate over it
//! holds.
//!
//! Built on `Mutex` + `Condvar`. Writers mutate the value under the lock and
//! `notify_all` only when the value actually changed; waiters use
//! [`Condvar::wait_timeout_while`], which re-checks the predicate across
//! spurious wakeups and recomputes the remaining timeout, so a wake without a
//! satisfying change never releases a waiter early. There is no
//! sender/receiver split and no closed state: the watch lives as long as any
//! `Arc` to it, and every wait is bounded by its caller's timeout.

use std::sync::{Condvar, Mutex};
use std::time::Duration;

pub struct Watch<T> {
    value: Mutex<T>,
    changed: Condvar,
}

impl<T: Clone> Watch<T> {
    pub fn new(initial: T) -> Self {
        Watch {
            value: Mutex::new(initial),
            changed: Condvar::new(),
        }
    }

    /// The current value, by clone. Production readers wait on a predicate
    /// via [`Self::wait_timeout_for`]; the tests observe state through this.
    #[cfg(test)]
    pub fn get(&self) -> T {
        self.value.lock().unwrap().clone()
    }

    /// Replace the value and wake every waiter.
    pub fn send_replace(&self, value: T) {
        *self.value.lock().unwrap() = value;
        self.changed.notify_all();
    }

    /// Mutate the value in place. `modify` returns whether it changed the
    /// value; waiters are woken only then.
    pub fn send_if_modified(&self, modify: impl FnOnce(&mut T) -> bool) {
        let mut value = self.value.lock().unwrap();
        if modify(&mut value) {
            self.changed.notify_all();
        }
    }

    /// Block until `pred` holds for the value, or `timeout` elapses. Returns
    /// whether the predicate was satisfied (`false` = timed out). A predicate
    /// that already holds returns immediately.
    pub fn wait_timeout_for(&self, timeout: Duration, mut pred: impl FnMut(&T) -> bool) -> bool {
        let guard = self.value.lock().unwrap();
        let (_guard, res) = self
            .changed
            .wait_timeout_while(guard, timeout, |v| !pred(v))
            .unwrap();
        !res.timed_out()
    }
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;
    use std::time::Instant;

    use super::*;

    #[test]
    fn wait_returns_immediately_when_the_predicate_already_holds() {
        let w = Watch::new(5u64);
        let started = Instant::now();
        assert!(w.wait_timeout_for(Duration::from_secs(30), |v| *v == 5));
        assert!(
            started.elapsed() < Duration::from_secs(5),
            "a satisfied predicate must not wait"
        );
    }

    #[test]
    fn wait_blocks_until_a_send_satisfies_the_predicate() {
        let w = Arc::new(Watch::new(0u64));
        let setter = w.clone();
        let h = std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(50));
            setter.send_replace(7);
        });

        let started = Instant::now();
        assert!(w.wait_timeout_for(Duration::from_secs(10), |v| *v == 7));
        assert!(
            started.elapsed() >= Duration::from_millis(40),
            "the wait must block until the satisfying send"
        );
        h.join().unwrap();
    }

    #[test]
    fn wait_times_out_when_the_predicate_never_holds() {
        let w = Watch::new(0u64);
        let started = Instant::now();
        assert!(!w.wait_timeout_for(Duration::from_millis(50), |v| *v == 7));
        assert!(
            started.elapsed() >= Duration::from_millis(50),
            "the timeout must run its full course"
        );
    }

    #[test]
    fn send_if_modified_wakes_only_on_change() {
        // A send that modifies nothing (modify returns false) must not
        // release a waiter; the next send that does modify must. The waiter
        // re-checks its predicate on every wake, so even a spurious wakeup in
        // between cannot release it early.
        let w = Arc::new(Watch::new(0u64));
        let setter = w.clone();
        let h = std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(30));
            setter.send_if_modified(|_| false); // no change: no wake
            std::thread::sleep(Duration::from_millis(30));
            setter.send_if_modified(|v| {
                *v = 9;
                true
            });
        });

        let started = Instant::now();
        assert!(w.wait_timeout_for(Duration::from_secs(10), |v| *v == 9));
        assert!(
            started.elapsed() >= Duration::from_millis(50),
            "the no-change send must not have released the wait"
        );
        h.join().unwrap();
    }

    #[test]
    fn get_returns_the_current_value() {
        let w = Watch::new(1u64);
        assert_eq!(w.get(), 1);
        w.send_replace(2);
        assert_eq!(w.get(), 2);
    }
}
