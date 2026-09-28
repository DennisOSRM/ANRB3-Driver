/* Track bookkeeping, kept free of the map so it can be tested without a
 * browser. It covers: what counts as a gap, which altitude colours a leg, and
 * when a point falls out of the cache.
 *
 * Written with var and function so it loads both as a plain script in the page
 * (sets globalThis.Tracks) and as a module under Node (module.exports). */
(function (root) {
  'use strict';

  var GAP_MS = 60 * 1000; // silence longer than this is drawn dashed
  var HISTORY_MS = 15 * 60 * 1000; // default age at which a point is forgotten
  var FADE_MS = 5 * 1000; // a quiet aircraft fades over its last five seconds

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

  /** The point drawn with a plane icon: the most recent position. */
  function head(icao, pts, meta, f) {
    if (!pts.length) return null;
    var last = pts[pts.length - 1];
    return {
      type: 'Feature',
      properties: {
        icao: icao,
        cs: meta.cs,
        trk: meta.trk,
        alt: meta.alt,
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
    append: append,
    trim: trim,
    isGap: isGap,
    fade: fade,
    segments: segments,
    head: head,
    build: build,
  };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  root.Tracks = api;
})(typeof globalThis !== 'undefined' ? globalThis : this);
