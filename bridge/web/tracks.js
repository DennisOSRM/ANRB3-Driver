/* Track bookkeeping, kept free of the map so it can be tested without a
 * browser. It covers: what counts as a gap, which altitude colours a leg, when
 * a point falls out of the cache, what a label says beside a callsign, and
 * which symbol an aircraft is drawn with.
 *
 * Written with var and function so it loads both as a plain script in the page
 * (sets globalThis.Tracks) and as a module under Node (module.exports). */
(function (root) {
  'use strict';

  var GAP_MS = 60 * 1000; // silence longer than this is drawn dashed
  var HISTORY_MS = 15 * 60 * 1000; // default age at which a point is forgotten
  var FADE_MS = 5 * 1000; // a quiet aircraft fades over its last five seconds
  var CLIMB_FPM = 300; // a vertical rate below this either way is level flight

  /* A point is [tSeconds, lon, lat, altFt|null]. */

  /** Add points from an update to a track. Points not newer than the last one
   *  held are dropped, so a duplicate or replayed update cannot add a backward
   *  jump. Returns how many were added. */
  function append(pts, fresh) {
    var last = pts.length ? pts[pts.length - 1][0] : -Infinity,
      n = 0;
    for (var i = 0; i < fresh.length; i++) {
      if (fresh[i][0] > last) {
        pts.push(fresh[i]);
        last = fresh[i][0];
        n++;
      }
    }
    return n;
  }

  /** Drop points older than `historyMs` (default HISTORY_MS) before `nowSec`. */
  function trim(pts, nowSec, historyMs) {
    var cut = nowSec - (historyMs === undefined ? HISTORY_MS : historyMs) / 1000;
    if (!pts.length || pts[0][0] >= cut) return pts;
    return pts.filter(function (p) {
      return p[0] >= cut;
    });
  }

  /** True when the aircraft was unheard across this leg for long enough that
   *  the straight line between the two points is not a flown path. */
  function isGap(a, b, gapMs) {
    return (b[0] - a[0]) * 1000 > (gapMs === undefined ? GAP_MS : gapMs);
  }

  /**
   * How visible an aircraft is `ageSec` after it was last heard, when it leaves
   * the map at `inactiveSec`: fully until its last FADE_MS, then down to 0 at
   * the moment it goes, so it fades out rather than vanishing.
   */
  function fade(ageSec, inactiveSec, fadeMs) {
    var span = (fadeMs === undefined ? FADE_MS : fadeMs) / 1000;
    var left = inactiveSec - ageSec;
    if (left >= span) return 1;
    if (left <= 0) return 0;
    return left / span;
  }

  /**
   * One LineString per consecutive pair, so the colour can follow the altitude
   * recorded on that leg. A single feature per aircraft could not do this:
   * line-gradient is a paint property and cannot vary per feature.
   *
   * The altitude carried is the one at the *end* of the leg, which is the more
   * recent of the two readings.
   */
  function segments(icao, pts, opts) {
    opts = opts || {};
    var out = [];
    for (var i = 0; i + 1 < pts.length; i++) {
      out.push({
        type: 'Feature',
        properties: {
          icao: icao,
          alt: pts[i + 1][3],
          gap: isGap(pts[i], pts[i + 1], opts.gapMs),
          sel: opts.selected === icao,
          dim: opts.selected != null && opts.selected !== icao,
          fade: opts.fade === undefined ? 1 : opts.fade,
        },
        geometry: {
          type: 'LineString',
          coordinates: [
            [pts[i][1], pts[i][2]],
            [pts[i + 1][1], pts[i + 1][2]],
          ],
        },
      });
    }
    return out;
  }

  /**
   * What the label says about an aircraft beside its callsign, each part only
   * when it is known: climbing or descending, autopilot engaged (from the
   * target-state modes, which only Beast carries), military (an address the
   * page found in a military block; see countries.js), flight level and
   * ground speed. A vertical rate inside CLIMB_FPM either way is level flight
   * with some noise in it, and on the ground it means nothing, so neither gets
   * an arrow; nor does the ground get a flight level. The flight level is the
   * barometric altitude in hundreds of feet, three digits as a controller says
   * it, and nothing below FL000.
   */
  function ornaments(meta) {
    var t = (meta.x && meta.x.target) || {};
    var vr = meta.gnd ? null : meta.vr;
    var alt = meta.gnd ? null : meta.alt;
    return {
      mil: !!meta.mil,
      ap: !!(t.modes && t.modes.indexOf('AP') >= 0),
      trend: vr == null ? '' : vr >= CLIMB_FPM ? 'climb' : vr <= -CLIMB_FPM ? 'descend' : '',
      fl: alt == null ? '' : 'FL' + ('00' + Math.max(0, Math.round(alt / 100))).slice(-3),
      spd: meta.gs == null ? '' : Math.round(meta.gs) + 'kt',
    };
  }

  /**
   * The label's parts after the callsign, in order, each styled on its own:
   *
   *     GAF681 ▲ AP M
   *     FL350 452kt
   *
   * The markers follow the callsign on its line, and the figures make a
   * second line under it. Every part carries the separator before it - a
   * space, or the line break for the first figure - so a part that is missing
   * leaves no gap and both lines stay centred. With no figures there is no
   * second line, and with nothing known every part is empty.
   */
  function labelParts(o) {
    var marks = [
      o.trend === 'climb' ? '▲' : o.trend === 'descend' ? '▼' : '',
      o.ap ? 'AP' : '',
      o.mil ? 'M' : '',
    ].map(function (p) {
      return p && ' ' + p;
    });
    var started = false;
    var figures = [o.fl, o.spd].map(function (p) {
      if (!p) return '';
      p = (started ? ' ' : '\n') + p;
      started = true;
      return p;
    });
    return marks.concat(figures);
  }

  /*
   * Which symbol an aircraft is drawn with. Its ICAO type designator (A320,
   * B77W, from the registration lookup) says the most and decides when it is
   * known; otherwise the emitter category it transmits itself (DO-260B
   * 2.2.3.2.5.2) gives the rough class; with neither it is a jet.
   *
   * Designators listed one by one, because their prefixes say nothing
   * reliable: B461 is a four-engined BAe 146, B190 a Beech 1900, B105 a
   * helicopter.
   */
  var TYPE_SHAPE = {};
  [
    [
      'quad',
      'A342 A343 A345 A346 A388 B741 B742 B743 B744 B748 B74D B74R B74S BLCF B461 B462 B463 ' +
        'RJ70 RJ85 RJ1H A400 C130 C30J C135 C141 C17 C5 C5M E3TF E3CF K35R K35E B703 IL76 ' +
        'IL86 IL96 A124 A225 P3 DHC7 VC10 DC8 CONI L188',
    ],
    [
      'heavy',
      'A306 A30B A310 A332 A333 A337 A338 A339 A359 A35K B762 B763 B764 B772 B773 B77L B77W ' +
        'B778 B779 B788 B789 B78X MD11 DC10 L101 KC10',
    ],
    [
      'regional',
      'CRJ1 CRJ2 CRJ7 CRJ9 CRJX E135 E145 E35L E170 E175 E190 E195 E290 E295 E75L E75S F70 ' +
        'F100 BCS1 BCS3 SU95 AJ27 J328',
    ],
    [
      'bizjet',
      'C500 C501 C510 C525 C25A C25B C25C C25M C550 C551 C55B C560 C56X C650 C680 C68A C700 ' +
        'C750 E50P E55P E545 E550 LJ31 LJ35 LJ40 LJ45 LJ60 LJ70 LJ75 GLF2 GLF3 GLF4 GLF5 GLF6 ' +
        'GA5C GA6C GA7C GA8C GLEX GL5T GL7T GL8T CL30 CL35 CL60 F2TH F900 FA10 FA20 FA50 FA6X ' +
        'FA7X FA8X H25B H25C HA4T HDJT PC24 PRM1 SF50 BE40',
    ],
    [
      'turboprop',
      'AT43 AT44 AT45 AT46 AT72 AT73 AT75 AT76 DH8A DH8B DH8C DH8D DHC6 B190 BE20 BE30 B350 ' +
        'BE9L BE9T BE99 SF34 JS31 JS32 JS41 SW4 D228 D328 F27 F50 PC12 C208 C08T TBM7 TBM8 ' +
        'TBM9 P180 L410 AN24 AN26 AN32 C160 C212 C295 CN35 SB20 E120 SH36 PAY2 PAY3 PAY4 KODI ' +
        'T6 PC21 PC7 PC9 TUCA',
    ],
    [
      'heli',
      'EC20 EC25 EC30 EC35 EC45 EC55 EC75 H160 AS32 AS50 AS55 AS65 B06 B06T B105 B407 ' +
        'B412 B427 B429 B430 B505 BK17 R22 R44 R66 A109 A119 A139 A169 A189 S76 S92 NH90 ' +
        'H60 UH1 CH47 H47 H64 EH10 LYNX TIGR MI8 MI24 G2CA',
    ],
    [
      'fighter',
      'F16 F15 F18H F18S F35 F22 F5 EUFI TOR RFAL MIR2 HAWK T38 A10 AJET M346 L39 L159 MG29 ' +
        'SU27 SU30 SU35 F4',
    ],
    ['light', 'ULAC AN2'],
    ['glider', 'GLID'],
    ['balloon', 'BALL'],
  ].forEach(function (g) {
    g[1].split(' ').forEach(function (code) {
      TYPE_SHAPE[code] = g[0];
    });
  });
  // Light singles and twins are too many to list, and their designators have
  // readable families: Cessna C1xx and C2xx pistons, Pipers, Cirrus, Diamonds
  // and the like. The families share their prefixes with a few military
  // types - C135, C141, C160, C212 - so the lists above are looked at first.
  var LIGHT =
    /^(C1\d\d|C2[0-4]\d|P28|PA\d|SR2|DA[2-6]|DR[34]|BE3[3-6]|BE5[58]|BE76|M20|C77R|AA5|TB[12]\d|RV\d|P68|G115|Z42|C42)/;
  var CATEGORY_SHAPE = {
    A1: 'light',
    A2: 'regional',
    A3: 'jet',
    A4: 'jet',
    A5: 'heavy',
    A6: 'fighter',
    A7: 'heli',
    B1: 'glider',
    B2: 'balloon',
    B4: 'light',
    B6: 'drone',
    C1: 'ground',
    C2: 'ground',
  };
  /** The symbol for an aircraft, from its type designator and category. */
  function shape(code, category) {
    code = (code || '').toUpperCase();
    return TYPE_SHAPE[code] || (LIGHT.test(code) && 'light') || CATEGORY_SHAPE[category] || 'jet';
  }

  /** The point drawn with a plane icon: the most recent position. */
  function head(icao, pts, meta, f) {
    if (!pts.length) return null;
    var last = pts[pts.length - 1];
    var parts = labelParts(ornaments(meta));
    return {
      type: 'Feature',
      properties: {
        icao: icao,
        cs: meta.cs,
        trk: meta.trk,
        alt: meta.alt,
        shape: shape(meta.type, meta.x && meta.x.category),
        trend: parts[0],
        ap: parts[1],
        mil: parts[2],
        fl: parts[3],
        spd: parts[4],
        fade: f === undefined ? 1 : f,
      },
      geometry: { type: 'Point', coordinates: [last[1], last[2]] },
    };
  }

  /**
   * Everything the two sources need, from the whole fleet. With `opts.now`
   * and `opts.inactive` (seconds, on the bridge's clock) each aircraft's
   * features carry the fade for how long it has been quiet, taken from its
   * `lastSeen`; `fading` says whether any is part-way out, so the caller knows
   * to keep redrawing.
   */
  function build(fleet, selected, opts) {
    opts = opts || {};
    var tf = [],
      pf = [],
      nTracks = 0,
      nPts = 0,
      fading = false;
    var timed = opts.now !== undefined && opts.inactive !== undefined;
    fleet.forEach(function (a, icao) {
      nPts += a.pts.length;
      if (a.pts.length > 1) nTracks++;
      var f =
        timed && a.lastSeen !== undefined
          ? fade(opts.now - a.lastSeen, opts.inactive, opts.fadeMs)
          : 1;
      if (f < 1) fading = true;
      tf.push.apply(tf, segments(icao, a.pts, { selected: selected, gapMs: opts.gapMs, fade: f }));
      var h = head(icao, a.pts, a.meta, f);
      if (h) pf.push(h);
    });
    return { tracks: tf, planes: pf, nTracks: nTracks, nPts: nPts, fading: fading };
  }

  var api = {
    GAP_MS: GAP_MS,
    HISTORY_MS: HISTORY_MS,
    FADE_MS: FADE_MS,
    CLIMB_FPM: CLIMB_FPM,
    append: append,
    trim: trim,
    isGap: isGap,
    fade: fade,
    ornaments: ornaments,
    labelParts: labelParts,
    shape: shape,
    segments: segments,
    head: head,
    build: build,
  };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  root.Tracks = api;
})(typeof globalThis !== 'undefined' ? globalThis : this);
