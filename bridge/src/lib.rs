//! The map bridge: what it serves the page, and what it keeps.
//!
//! [`http`] is the transport: HTTP and WebSocket, implemented here; gzip via
//! flate2 and TLS via rustls. [`map`] holds the aircraft and their tracks for
//! the quarter of an hour the page draws. [`hexdb`] looks up registration,
//! type, operator, route and photograph from hexdb.io and caches them on disk.

pub mod hexdb;
pub mod http;
pub mod map;

use std::sync::{Mutex, MutexGuard};

/// Lock a mutex, recovering it if a thread panicked while holding it. Nothing
/// in this crate leaves shared state half-built, so the contents are still
/// usable, and refusing to serve because an unrelated thread died would be
/// worse.
pub(crate) fn lock<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(|e| e.into_inner())
}
