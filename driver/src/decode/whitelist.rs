//! Trusted addresses, for the formats that cannot self-validate.

use super::frame::is_valid_icao;

/// Addresses learned from CRC-validated traffic, used to recognise the
/// address-overlaid downlink formats whose parity cannot self-validate.
///
/// AP carries the address bit for bit, so a demodulation error in AP bit j
/// yields exactly `address ^ (1<<j)` - and AP is the last 24 bits of the
/// frame, where timing has drifted furthest from the preamble lock. Two
/// trusted addresses that sit close together in address space therefore cannot
/// be told apart, and neither may be used. Measured against live traffic, a
/// separation of two bits admits a pair whose misattribution rate is 0.057%.
/// When the table is full, the least recently heard address is evicted.
/// Without eviction the 8,192 slots fill in about three days at the measured
/// rate of two new addresses a minute; after that no new address is learned,
/// and every lookup miss scans the whole table.
pub(super) struct Whitelist {
    /// The address in each slot. A slot keeps its address after an eviction.
    key: Vec<u32>,
    /// Times each slot's address was heard; 0 for a free slot.
    heard: Vec<u16>,
    /// Receiver milliseconds when the slot was last heard from, for choosing
    /// what to evict.
    seen: Vec<u32>,
    /// Too close to another trusted address to be distinguishable.
    ambiguous: Vec<bool>,
    /// Slots of the trusted addresses, so admitting one can be checked against
    /// the others without walking the table.
    trusted: Vec<usize>,
    /// Addresses held.
    n: usize,
    /// Addresses refused for sitting too close to another trusted one.
    #[cfg(test)]
    refused: usize,
    /// Addresses dropped to make room.
    pub(super) evicted: u64,
}

impl Whitelist {
    const BITS: u32 = 13;
    const SIZE: usize = 1 << Self::BITS;
    /// Times an address must be heard before it is trusted.
    const MIN_SEEN: u16 = 2;
    /// Minimum Hamming distance between trusted addresses.
    const MIN_SEP: u32 = 2;

    pub(super) fn new() -> Self {
        Whitelist {
            key: vec![0; Self::SIZE],
            heard: vec![0; Self::SIZE],
            seen: vec![0; Self::SIZE],
            ambiguous: vec![false; Self::SIZE],
            trusted: Vec::new(),
            n: 0,
            #[cfg(test)]
            refused: 0,
            evicted: 0,
        }
    }
    fn hash(a: u32) -> usize {
        ((a.wrapping_mul(2654435761) >> (32 - Self::BITS)) as usize) & (Self::SIZE - 1)
    }
    /// Note an address, heard at `ms`. Slots are reused when the table is
    /// full, so a slot with no count but a key left in it is a hole a lookup
    /// must probe past rather than stop at - linear probing puts an entry
    /// anywhere after its hash, and stopping early would lose it.
    pub(super) fn add(&mut self, icao: u32, ms: u32) {
        if !is_valid_icao(icao) {
            return;
        }
        let h = Self::hash(icao);
        let mut hole = None;
        for i in 0..Self::SIZE {
            let k = (h + i) & (Self::SIZE - 1);
            if self.heard[k] == 0 {
                if self.key[k] == 0 {
                    let slot = hole.unwrap_or(k);
                    self.fill(slot, icao, ms);
                    return;
                }
                hole.get_or_insert(k); // a slot something was evicted from
                continue;
            }
            if self.key[k] == icao {
                self.heard[k] = self.heard[k].saturating_add(1);
                self.seen[k] = ms;
                if self.heard[k] == Self::MIN_SEEN {
                    self.admit(k);
                }
                return;
            }
        }
        // Every slot holds something. The one heard longest ago goes, and the
        // new address takes the hole it leaves - found by probing again, since
        // it has to sit where a lookup for it will look.
        if let Some(k) = hole {
            self.fill(k, icao, ms);
            return;
        }
        self.evict_oldest();
        self.add(icao, ms);
    }

    /// Put an address in a slot that is free or has been freed.
    fn fill(&mut self, k: usize, icao: u32, ms: u32) {
        self.key[k] = icao;
        self.heard[k] = 1;
        self.seen[k] = ms;
        self.ambiguous[k] = false; // whatever was here is gone
        self.n += 1;
    }

    /// Drop the address heard longest ago. Its slot keeps its key so lookups
    /// still probe past it, but counts as empty for the next insert. An
    /// address that was refused for sitting too close to this one stays
    /// refused: that errs towards not decoding a frame rather than decoding
    /// it as the wrong aircraft.
    fn evict_oldest(&mut self) {
        let mut oldest = 0usize;
        for k in 0..Self::SIZE {
            if self.heard[k] > 0 && self.seen[k] < self.seen[oldest] {
                oldest = k;
            }
        }
        self.heard[oldest] = 0;
        self.ambiguous[oldest] = false;
        self.trusted.retain(|&s| s != oldest);
        self.n -= 1;
        self.evicted += 1;
    }

    /// An address becomes trusted the moment it reaches `MIN_SEEN`. At that
    /// point, and only then, check it against the addresses already trusted:
    /// any pair within `MIN_SEP` bits is mutually ambiguous and both are
    /// refused: keeping either would misattribute the other's frames.
    fn admit(&mut self, slot: usize) {
        for i in 0..self.trusted.len() {
            let o = self.trusted[i];
            if (self.key[slot] ^ self.key[o]).count_ones() >= Self::MIN_SEP {
                continue;
            }
            for k in [slot, o] {
                if !self.ambiguous[k] {
                    self.ambiguous[k] = true;
                    #[cfg(test)]
                    {
                        self.refused += 1;
                    }
                }
            }
        }
        self.trusted.push(slot);
    }
    pub(super) fn has(&self, icao: u32) -> bool {
        if !is_valid_icao(icao) {
            return false;
        }
        let h = Self::hash(icao);
        for i in 0..Self::SIZE {
            let k = (h + i) & (Self::SIZE - 1);
            if self.heard[k] == 0 && self.key[k] == 0 {
                return false;
            }
            if self.heard[k] > 0 && self.key[k] == icao {
                return !self.ambiguous[k] && self.heard[k] >= Self::MIN_SEEN;
            }
        }
        false
    }
    pub(super) fn count(&self) -> usize {
        self.n
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 3C6551 and 3C6555 were both overhead during the 15-minute capture and
    /// differ in a single bit, so neither can be trusted for overlaid decoding
    /// once both are known. The live captures do not exercise this, because
    /// 3C6555 was heard once and `MIN_SEEN` keeps it out.
    #[test]
    fn close_addresses_refuse_each_other() {
        let mut wl = Whitelist::new();
        for _ in 0..4 {
            wl.add(0x3C_6551, 0);
        }
        assert!(wl.has(0x3C_6551), "a lone trusted address is usable");
        for _ in 0..4 {
            wl.add(0x3C_6555, 0);
        }
        assert!(
            !wl.has(0x3C_6551) && !wl.has(0x3C_6555),
            "two trusted addresses one bit apart are both refused"
        );
        assert_eq!(wl.refused, 2, "both members of the pair are counted");
        for _ in 0..4 {
            wl.add(0x44_00B7, 0);
        }
        assert!(wl.has(0x44_00B7), "a distant address is unaffected");
    }

    /// A third address close to one already refused is refused too, and the
    /// one already refused is counted once.
    #[test]
    fn a_refused_address_is_counted_once() {
        let mut wl = Whitelist::new();
        for a in [0x3C_6551, 0x3C_6555, 0x3C_6550] {
            wl.add(a, 0);
            wl.add(a, 0);
        }
        assert_eq!(wl.refused, 3);
        assert!(!wl.has(0x3C_6551) && !wl.has(0x3C_6555) && !wl.has(0x3C_6550));
    }

    /// The separation of 2 still admits a two-bit pair, whose measured
    /// misattribution rate is 0.057%, an accepted rate.
    #[test]
    fn separation_threshold_is_honoured() {
        let mut wl = Whitelist::new();
        assert_eq!((0x4D_2014u32 ^ 0x4D_2410u32).count_ones(), 2);
        for _ in 0..4 {
            wl.add(0x4D_2014, 0);
        }
        for _ in 0..4 {
            wl.add(0x4D_2410, 0);
        }
        assert!(
            wl.has(0x4D_2014) && wl.has(0x4D_2410),
            "a two-bit pair survives the separation of 2"
        );
    }

    /// Filling the table and then some: the count holds at capacity, the new
    /// addresses are usable, and the ones still in the table are still found -
    /// which is the part linear probing makes easy to get wrong, since a slot
    /// freed by an eviction sits in the middle of somebody's probe chain.
    #[test]
    fn full_table_makes_room() {
        let mut wl = Whitelist::new();
        let addr = |i: u32| 0x3C_0000u32.wrapping_add(i.wrapping_mul(7919)) & 0xFF_FFFF;
        // 500 more than the table holds, each heard twice so it is trusted.
        for i in 0..Whitelist::SIZE as u32 + 500 {
            wl.add(addr(i), i);
            wl.add(addr(i), i);
        }
        assert_eq!(
            wl.count(),
            Whitelist::SIZE,
            "the table stays full, not overfull"
        );
        assert!(
            wl.evicted >= 500,
            "the oldest made way: {} evicted",
            wl.evicted
        );

        let last = Whitelist::SIZE as u32 + 499;
        assert!(
            wl.has(addr(last)),
            "the most recently heard address is trusted"
        );
        assert!(!wl.has(addr(0)), "the first one heard is gone");
        // Everything from the last SIZE arrivals that was not evicted must
        // still be findable; a broken probe chain shows up here.
        let mut found = 0;
        for i in (Whitelist::SIZE as u32 + 500 - 2000)..(Whitelist::SIZE as u32 + 500) {
            if wl.has(addr(i)) {
                found += 1;
            }
        }
        assert_eq!(found, 2000, "every recent address is still reachable");
    }

    /// Eviction must not leave a hole that hides an address behind it. The
    /// addresses here are spread out rather than consecutive, because
    /// neighbours one bit apart refuse each other by design and that would
    /// mask what this is testing.
    #[test]
    fn a_freed_slot_is_probed_past() {
        let mut wl = Whitelist::new();
        let old = |i: u32| 0x40_0000u32.wrapping_add(i.wrapping_mul(7919)) & 0xFF_FFFF;
        let new = |i: u32| 0x90_0000u32.wrapping_add(i.wrapping_mul(4093)) & 0xFF_FFFF;
        for i in 0..Whitelist::SIZE as u32 {
            wl.add(old(i), i);
            wl.add(old(i), i);
        }
        let kept = old(Whitelist::SIZE as u32 - 1); // heard most recently
        assert!(wl.has(kept), "trusted before any eviction");
        for i in 0..64u32 {
            // force 64 evictions
            wl.add(new(i), 1_000_000 + i);
            wl.add(new(i), 1_000_000 + i);
        }
        assert_eq!(wl.evicted, 64, "one slot freed per newcomer");
        assert!(
            wl.has(kept),
            "an address behind a freed slot is still found"
        );
        assert!(wl.has(new(63)), "and so is the newcomer");
        assert_eq!(wl.count(), Whitelist::SIZE);
    }

    /// An address seen once is what keeps the quieter member of a close pair
    /// out in the first place.
    #[test]
    fn seen_once_is_not_trusted() {
        let mut wl = Whitelist::new();
        wl.add(0x3C_6551, 0);
        assert!(!wl.has(0x3C_6551));
    }

    /// The rule must not fire between an untrusted address and a trusted one -
    /// an address below `MIN_SEEN` has not been admitted and cannot poison a
    /// neighbour.
    #[test]
    fn unadmitted_neighbour_does_not_poison() {
        let mut wl = Whitelist::new();
        for _ in 0..4 {
            wl.add(0x3C_6551, 0);
        }
        wl.add(0x3C_6555, 0); // seen once only
        assert!(
            wl.has(0x3C_6551),
            "a neighbour below MIN_SEEN leaves the trusted address usable"
        );
        assert_eq!(wl.refused, 0);
    }
}
