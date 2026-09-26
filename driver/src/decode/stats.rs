//! What the decoder counts, and the options that change which passes run.

/// Cycles spent per phase, when built with the `profile` feature.
#[derive(Default, Clone, Copy, Debug)]
pub struct Phases {
    /// All of `decode_burst`.
    pub total: u64,
    /// The demodulator.
    pub demod: u64,
    /// Framing, CRC and correction.
    pub frame: u64,
    /// The soft search, a part of `frame`.
    pub soft: u64,
    /// Copying demodulator output into the per-burst cache.
    pub cache_copy: u64,
}

/// Counters accumulated over a decoding run.
#[derive(Default, Clone, Copy, Debug)]
pub struct Stats {
    /// Bursts decoded.
    pub bursts: u64,
    /// Frames recovered.
    pub frames: u64,
    /// Frames recovered with no correction.
    pub clean: u64,
    /// Frames recovered by correcting one bit.
    pub fixed1: u64,
    /// Frames recovered by correcting two bits.
    pub fixed2: u64,
    /// Candidates the soft search worked on.
    pub soft_tried: u64,
    /// Soft-search attempts that corrected one or two bits.
    pub soft_hit: u64,
    /// Address-overlaid candidates checked against the whitelist.
    pub overlaid_tried: u64,
    /// Address-overlaid candidates whose address was trusted.
    pub overlaid_hit: u64,
    // Diagnostic counters, enabled in every build: anrb-replay reports them,
    // and they are plain increments with no measurable cost.

    /// Demodulator runs: grid configurations evaluated.
    pub stage_runs: u64,
    /// Candidate framings whose CRC was computed.
    pub crc_calls: u64,
    /// Framings examined: offsets that passed the downlink-format filter in
    /// `stage_cached`.
    pub offsets: u64,
    /// Offsets examined, split by which correction pass was running.
    pub offsets_by_pass: [u64; 5],
    /// Framings of the category the current pass handles (pure or
    /// address-overlaid).
    pub offsets_kept: u64,
    /// Configurations entered, by pass.
    pub cfgs_by_pass: [u64; 5],
}

impl Stats {
    /// Frames recovered per burst, as a percentage.
    pub fn yield_pct(&self) -> f64 {
        if self.bursts > 0 { 100.0 * self.frames as f64 / self.bursts as f64 } else { 0.0 }
    }
}

/// Options that change which passes run.
#[derive(Clone, Copy, Debug)]
pub struct Options {
    /// Confidence-guided 2-bit correction. On by default.
    pub soft: bool,
    /// Blind 2-bit correction. Recovers more frames but accepts noise as false
    /// addresses.
    pub blind2: bool,
    /// Address-overlaid downlink formats. Off by default: on this hardware
    /// every measured hit was a coincidental match.
    pub overlaid: bool,
}

impl Default for Options {
    fn default() -> Self { Options { soft: true, blind2: false, overlaid: false } }
}
