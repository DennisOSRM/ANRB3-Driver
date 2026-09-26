//! The decode path does no per-burst allocation: working memory is buffers
//! owned by `Decoder` and `Stages`, candidate frames are stack arrays, and
//! `Frame` is `Copy`. What does allocate is those buffers growing to size -
//! the demodulator caches on the first bursts and whenever a longer one
//! arrives, the address whitelist as aircraft appear. Buffer growth accounts
//! for a few allocations early in a capture and none after it; the test fails
//! if the count scales with the number of bursts.
//!
//! A counting allocator wraps the system one and counts only while this
//! thread is inside the decode loop, so allocations made by the test harness
//! on its own threads are not counted.
//!
//! The test returns early, and passes, when its capture file is absent.

use anrb::{corpus, protocol, Decoder};
use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::sync::atomic::{AtomicUsize, Ordering::Relaxed};

static ALLOCS: AtomicUsize = AtomicUsize::new(0);
thread_local! { static COUNTING: Cell<bool> = const { Cell::new(false) }; }

struct Counting;
// SAFETY: every call is forwarded unchanged to the system allocator; the
// only addition is a counter, and reading a const-initialised thread_local
// Cell does not allocate.
unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, l: Layout) -> *mut u8 {
        if COUNTING.with(Cell::get) { ALLOCS.fetch_add(1, Relaxed); }
        unsafe { System.alloc(l) }
    }
    unsafe fn dealloc(&self, p: *mut u8, l: Layout) { unsafe { System.dealloc(p, l) } }
    unsafe fn realloc(&self, p: *mut u8, l: Layout, n: usize) -> *mut u8 {
        if COUNTING.with(Cell::get) { ALLOCS.fetch_add(1, Relaxed); }
        unsafe { System.realloc(p, l, n) }
    }
}
#[global_allocator]
static A: Counting = Counting;

/// The first bursts of a real 15-minute capture: enough to fill the address
/// whitelist and run every correction pass, few enough to stay quick in a
/// debug build.
const BURSTS: usize = 20_000;

/// Growth by doubling costs O(log n) allocations; allocating per burst would
/// cost thousands here.
const GROWTH_BUDGET: usize = 24;

#[test]
fn decoding_allocates_only_to_grow() {
    let Ok(d) = std::fs::read("../captures/tuning_15min_v2.raw") else {
        eprintln!("corpus absent, skipping");
        return;
    };
    // Collect the bursts first, so the loop below does nothing but decode.
    let bursts: Vec<(u32, &[u8])> = corpus::raw_log(&d).expect("an ANRBRAW1 log").take(BURSTS).collect();

    let mut dec = Decoder::new();
    let mut frames = 0usize;
    COUNTING.with(|c| c.set(true));
    for &(ms, data) in &bursts {
        if protocol::is_pong(data) { continue; }
        for (off, len) in protocol::segments(data) {
            if dec.decode_burst(&data[off..off + len], ms).is_some() { frames += 1; }
        }
    }
    COUNTING.with(|c| c.set(false));

    assert!(frames > 1000, "expected a real workload, got {frames} frames");
    assert!(dec.stats.soft_hit > 0, "the soft-decision pass should have run");
    let n = ALLOCS.load(Relaxed);
    assert!(n <= GROWTH_BUDGET,
            "{n} allocations over {} bursts: something is allocating per burst", bursts.len());
}
