//! Cycle counters for the `profile` and `profile_fine` features.
//!
//! With neither feature on, both counters are constant zero and every use of
//! them compiles away. rdtsc costs a few cycles where `Instant::now` costs tens
//! of nanoseconds, which matters when the phases being timed are around a
//! microsecond; other architectures fall back to a monotonic clock.

/// A tick for the coarse phase timers (`profile`).
#[cfg(feature = "profile")]
#[inline(always)]
pub fn tick() -> u64 { counter() }
#[cfg(not(feature = "profile"))]
#[inline(always)]
pub fn tick() -> u64 { 0 }

/// A tick for the sub-phase timers inside the demodulator and the soft
/// search (`profile_fine`).
#[cfg(feature = "profile_fine")]
#[inline(always)]
pub fn fine_tick() -> u64 { counter() }
#[cfg(not(feature = "profile_fine"))]
#[inline(always)]
pub fn fine_tick() -> u64 { 0 }

#[cfg(all(feature = "profile", target_arch = "x86_64"))]
#[inline(always)]
fn counter() -> u64 {
    // SAFETY: rdtsc has no preconditions; it reads the timestamp counter.
    unsafe { core::arch::x86_64::_rdtsc() }
}
#[cfg(all(feature = "profile", not(target_arch = "x86_64")))]
#[inline(always)]
fn counter() -> u64 {
    use std::sync::OnceLock;
    use std::time::Instant;
    static BASE: OnceLock<Instant> = OnceLock::new();
    BASE.get_or_init(Instant::now).elapsed().as_nanos() as u64
}
