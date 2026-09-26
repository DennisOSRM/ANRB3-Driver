/* Unit tests for tracks.js (the logic that decides what is drawn).
 * Run: node bridge/web/test_tracks.cjs */
const T = require('./tracks.js');

let checks = 0, fails = 0;
const ok = (c, m) => { checks++; if (!c) { fails++; console.log('  FAIL ' + m); } };
const eq = (a, b, m) => ok(JSON.stringify(a) === JSON.stringify(b),
  m + '  (got ' + JSON.stringify(a) + ', want ' + JSON.stringify(b) + ')');

console.log('track logic tests');

const t0 = 1700000000;
// climbing, one point every 4 s
const climb = [];
for (let i = 0; i < 5; i++) climb.push([t0 + i * 4, 8.0 + i * 0.01, 50.0, i * 1000]);

// --- the gap rule -------------------------------------------------------
ok(!T.isGap(climb[0], climb[1]), 'four seconds apart is not a gap');
ok(T.isGap([t0, 8, 50, 0], [t0 + 61, 8, 50, 0]), 'sixty-one seconds apart is a gap');
ok(!T.isGap([t0, 8, 50, 0], [t0 + 60, 8, 50, 0]),
   'exactly sixty seconds is not yet a gap - the threshold is strictly greater');

// A track that was lost and came back: only the bridging leg is dashed, and
// the legs either side of it stay solid.
const rejoin = climb.concat([[t0 + 16 + 240, 8.30, 50.10, 38000],
                             [t0 + 16 + 244, 8.31, 50.11, 37000]]);
const segs = T.segments('3C6551', rejoin, {});
eq(segs.length, rejoin.length - 1, 'one segment per consecutive pair');
eq(segs.map(s => s.properties.gap),
   [false, false, false, false, true, false],
   'only the leg spanning the silence is a gap');

// --- altitude drives the colour ----------------------------------------
eq(segs.slice(0, 4).map(s => s.properties.alt), [1000, 2000, 3000, 4000],
   'each leg carries the altitude at its later end');
const withNull = T.segments('X', [[t0, 8, 50, null], [t0 + 4, 8.01, 50, null]], {});
eq(withNull[0].properties.alt, null,
   'a leg with no altitude reported carries null, for the style to coalesce');

// --- selection ----------------------------------------------------------
const sel = T.segments('3C6551', climb, { selected: '3C6551' });
ok(sel.every(s => s.properties.sel === true && s.properties.dim === false),
   'the selected aircraft is flagged and not dimmed');
const other = T.segments('4009DA', climb, { selected: '3C6551' });
ok(other.every(s => s.properties.sel === false && s.properties.dim === true),
   'everything else dims while a selection is held');
const none = T.segments('4009DA', climb, { selected: null });
ok(none.every(s => s.properties.dim === false), 'nothing dims with no selection');

// --- the plane at the tip ----------------------------------------------
const h = T.head('3C6551', rejoin, { cs: 'DLH8AB', trk: 68, alt: 37000 });
eq(h.geometry.coordinates, [8.31, 50.11], 'the icon sits on the most recent point');
eq(h.properties.trk, 68, 'and carries the track for icon-rotate');
ok(T.head('X', [], {}) === null, 'an aircraft with no positions gets no icon');

// --- the history window (fifteen minutes by default) --------------------
const old = [[t0 - 16 * 60, 8, 50, 100], [t0 - 14 * 60, 8, 50, 200], [t0 - 60, 8, 50, 300]];
eq(T.trim(old, t0).length, 2, 'points past fifteen minutes are dropped');
eq(T.trim(old, t0)[0][3], 200, 'and the ones inside the window are kept in order');
eq(T.trim(climb, t0 + 10).length, climb.length, 'a fresh track is left alone');
eq(T.trim([], t0).length, 0, 'an empty track trims to empty');
eq(T.trim(old, t0, 30 * 60 * 1000).length, 3, 'a longer history from the bridge keeps older points');
eq(T.trim(old, t0, 5 * 60 * 1000).length, 1, 'a shorter one drops more');

// --- whole-fleet assembly ----------------------------------------------
const fleet = new Map([
  ['3C6551', { meta: { cs: 'DLH8AB', trk: 68, alt: 37000 }, pts: rejoin }],
  ['4009DA', { meta: { cs: 'BAW11', trk: 110, alt: 35000 }, pts: climb }],
  ['4CA739', { meta: { cs: null, trk: null, alt: null }, pts: [] }],
]);
const built = T.build(fleet, null, {});
eq(built.tracks.length, (rejoin.length - 1) + (climb.length - 1),
   'segments come from every aircraft that has a track');
eq(built.planes.length, 2, 'an aircraft with no positions contributes no icon');
eq(built.nTracks, 2, 'the track count ignores aircraft with a single point');
eq(built.nPts, rejoin.length + climb.length, 'the point count covers the whole fleet');

// --- fading out --------------------------------------------------------
// Removed at 60 s of silence; the last five seconds fade linearly.
eq(T.fade(0, 60), 1, 'heard a moment ago is fully drawn');
eq(T.fade(55, 60), 1, 'fully drawn until the last five seconds');
eq(T.fade(57.5, 60), 0.5, 'half way through the fade is half drawn');
eq(T.fade(60, 60), 0, 'gone at the moment it leaves the map');
eq(T.fade(75, 60), 0, 'and stays gone if the drop is late');

const quiet = new Map([
  ['3C6551', { meta: { cs: 'DLH8AB', trk: 68, alt: 37000 }, pts: climb, lastSeen: t0 + 16 }],
  ['4009DA', { meta: { cs: 'BAW11', trk: 110, alt: 35000 }, pts: climb, lastSeen: t0 + 70 }],
]);
const faded = T.build(quiet, null, { now: t0 + 16 + 58, inactive: 60 });
const fadeOf = icao => faded.planes.find(p => p.properties.icao === icao).properties.fade;
ok(Math.abs(fadeOf('3C6551') - 0.4) < 1e-9, '58 s quiet: 2 s left of the 5 s fade');
eq(fadeOf('4009DA'), 1, 'one heard 4 s ago is untouched');
ok(faded.tracks.filter(s => s.properties.icao === '3C6551').every(s => Math.abs(s.properties.fade - 0.4) < 1e-9),
   'its whole track fades with it, not only the icon');
eq(faded.fading, true, 'build says a fade is in progress, so the page keeps redrawing');
eq(T.build(quiet, null, { now: t0 + 20, inactive: 60 }).fading, false, 'and says so when none is');
eq(T.build(quiet, null, {}).planes[0].properties.fade, 1, 'without a clock nothing fades');

// --- appending updates ---------------------------------------------------
// Appending ignores points not newer than the track's last point.
const held = climb.slice();
eq(T.append(held, climb), 0, 'a replayed history adds nothing');
eq(held.length, climb.length, 'and the track is unchanged');
const next = [t0 + 20, 8.05, 50.0, 5000];
eq(T.append(held, climb.concat([next])), 1, 'only the point past the end is new');
eq(held[held.length - 1], next, 'and it goes on the end');
eq(T.append(held, [[t0 + 24, 8.06, 50, 5000], [t0 + 22, 8.9, 51, 0]]), 1,
   'a point older than one added in the same update is refused too');
ok(held.every((p, i) => i === 0 || p[0] > held[i - 1][0]), 'time only moves forward along a track');

console.log(checks + ' checks, ' + fails + ' failures');
process.exit(fails ? 1 : 0);
