/* Unit tests for tracks.js (the logic that decides what is drawn).
 * Run: node bridge/web/test_tracks.cjs */
const T = require('./tracks.js');

let checks = 0,
  fails = 0;
const ok = (c, m) => {
  checks++;
  if (!c) {
    fails++;
    console.log('  FAIL ' + m);
  }
};
const eq = (a, b, m) =>
  ok(
    JSON.stringify(a) === JSON.stringify(b),
    m + '  (got ' + JSON.stringify(a) + ', want ' + JSON.stringify(b) + ')',
  );

console.log('track logic tests');

const t0 = 1700000000;
// climbing, one point every 4 s
const climb = [];
for (let i = 0; i < 5; i++) climb.push([t0 + i * 4, 8.0 + i * 0.01, 50.0, i * 1000]);

// --- the gap rule -------------------------------------------------------
ok(!T.isGap(climb[0], climb[1]), 'four seconds apart is not a gap');
ok(T.isGap([t0, 8, 50, 0], [t0 + 61, 8, 50, 0]), 'sixty-one seconds apart is a gap');
ok(
  !T.isGap([t0, 8, 50, 0], [t0 + 60, 8, 50, 0]),
  'exactly sixty seconds is not yet a gap - the threshold is strictly greater',
);

// A track that was lost and came back: only the bridging leg is dashed, and
// the legs either side of it stay solid.
const rejoin = climb.concat([
  [t0 + 16 + 240, 8.3, 50.1, 38000],
  [t0 + 16 + 244, 8.31, 50.11, 37000],
]);
const segs = T.segments('3C6551', rejoin, {});
eq(segs.length, rejoin.length - 1, 'one segment per consecutive pair');
eq(
  segs.map((s) => s.properties.gap),
  [false, false, false, false, true, false],
  'only the leg spanning the silence is a gap',
);

// --- altitude drives the colour ----------------------------------------
eq(
  segs.slice(0, 4).map((s) => s.properties.alt),
  [1000, 2000, 3000, 4000],
  'each leg carries the altitude at its later end',
);
const withNull = T.segments(
  'X',
  [
    [t0, 8, 50, null],
    [t0 + 4, 8.01, 50, null],
  ],
  {},
);
eq(
  withNull[0].properties.alt,
  null,
  'a leg with no altitude reported carries null, for the style to coalesce',
);

// --- selection ----------------------------------------------------------
const sel = T.segments('3C6551', climb, { selected: '3C6551' });
ok(
  sel.every((s) => s.properties.sel === true && s.properties.dim === false),
  'the selected aircraft is flagged and not dimmed',
);
const other = T.segments('4009DA', climb, { selected: '3C6551' });
ok(
  other.every((s) => s.properties.sel === false && s.properties.dim === true),
  'everything else dims while a selection is held',
);
const none = T.segments('4009DA', climb, { selected: null });
ok(
  none.every((s) => s.properties.dim === false),
  'nothing dims with no selection',
);

// --- the plane at the tip ----------------------------------------------
const h = T.head('3C6551', rejoin, { cs: 'DLH8AB', trk: 68, alt: 37000 });
eq(h.geometry.coordinates, [8.31, 50.11], 'the icon sits on the most recent point');
eq(h.properties.trk, 68, 'and carries the track for icon-rotate');
ok(T.head('X', [], {}) === null, 'an aircraft with no positions gets no icon');

// --- the markers after the callsign ---------------------------------------
eq(
  T.ornaments({}),
  { mil: false, ap: false, trend: '', fl: '', spd: '' },
  'nothing known, nothing drawn',
);
eq(T.ornaments({ mil: true }).mil, true, 'a military address is marked');
eq(
  T.ornaments({ alt: 34975, gs: 451.6, vr: 1800, x: { target: { modes: ['AP', 'LNAV'] } } }),
  { mil: false, ap: true, trend: 'climb', fl: 'FL350', spd: '452kt' },
  'autopilot on, climbing, FL350, and the speed rounded to the knot',
);
eq(
  T.ornaments({ vr: -T.CLIMB_FPM, x: { target: { modes: ['LNAV'] } } }).ap,
  false,
  'a mode other than AP is not the autopilot',
);
eq(T.ornaments({ vr: -T.CLIMB_FPM }).trend, 'descend', 'the threshold counts as descending');
eq(T.ornaments({ vr: T.CLIMB_FPM - 1 }).trend, '', 'just under it is level');
eq(T.ornaments({ vr: 0 }).trend, '', 'level is no arrow');
eq(T.ornaments({ vr: 2000, gnd: 1 }).trend, '', 'on the ground a vertical rate means nothing');
eq(T.ornaments({ gs: 0 }).spd, '0kt', 'standing still is a known speed');
eq(T.ornaments({ alt: 4500 }).fl, 'FL045', 'a flight level is always three digits');
eq(T.ornaments({ alt: 0 }).fl, 'FL000', 'sea level is FL000');
eq(T.ornaments({ alt: -350 }).fl, 'FL000', 'and nothing reads lower');
eq(T.ornaments({ alt: 1200, gnd: 1 }).fl, '', 'on the ground there is no flight level');
// Parts in label order: arrow, AP, M on the callsign's line; FL, speed under it.
eq(
  T.labelParts({ mil: true, ap: true, trend: 'climb', fl: 'FL350', spd: '452kt' }),
  [' ▲', ' AP', ' M', '\nFL350', ' 452kt'],
  'everything known: markers after the callsign, figures on the line under it',
);
eq(
  T.labelParts({ mil: true, ap: false, trend: '', fl: '', spd: '452kt' }),
  ['', '', ' M', '', '\n452kt'],
  'a missing marker leaves no gap, and the figures start their line without a space',
);
eq(
  T.labelParts({ mil: false, ap: false, trend: 'descend', fl: 'FL045', spd: '' }),
  [' ▼', '', '', '\nFL045', ''],
  'flight level alone',
);
eq(
  T.labelParts({ mil: false, ap: true, trend: '', fl: '', spd: '' }),
  ['', ' AP', '', '', ''],
  'markers without figures: no second line',
);
eq(
  T.labelParts({ mil: false, ap: false, trend: '', fl: '', spd: '' }),
  ['', '', '', '', ''],
  'nothing known: the label is the callsign alone',
);
const marked = T.head('3C6551', climb, { cs: 'DLH8AB', alt: 12000, gs: 300, vr: -900, x: {} });
eq(
  ['trend', 'ap', 'mil', 'fl', 'spd'].map((k) => marked.properties[k]),
  [' ▼', '', '', '\nFL120', ' 300kt'],
  'the icon carries the parts for its label',
);

// --- the symbol --------------------------------------------------------
const shapes = (pairs) => pairs.map(([code, cat]) => T.shape(code, cat));
eq(
  shapes([
    ['A388'],
    ['B744'],
    ['B461'],
    ['C130'],
    ['B77W'],
    ['A359'],
    ['A320'],
    ['B38M'],
    ['E190'],
    ['CRJ9'],
    ['C68A'],
    ['GLF6'],
    ['AT76'],
    ['DH8D'],
    ['C208'],
    ['C172'],
    ['P28A'],
    ['SR22'],
    ['EC35'],
    ['B105'],
    ['EUFI'],
    ['GLID'],
    ['BALL'],
  ]),
  [
    'quad',
    'quad',
    'quad',
    'quad',
    'heavy',
    'heavy',
    'jet',
    'jet',
    'regional',
    'regional',
    'bizjet',
    'bizjet',
    'turboprop',
    'turboprop',
    'turboprop',
    'light',
    'light',
    'light',
    'heli',
    'heli',
    'fighter',
    'glider',
    'balloon',
  ],
  'the type designator picks the symbol',
);
eq(
  shapes([
    [null, 'A1'],
    [null, 'A5'],
    [null, 'A7'],
    [null, 'B6'],
    [null, 'C1'],
    [null, null],
  ]),
  ['light', 'heavy', 'heli', 'drone', 'ground', 'jet'],
  'without one the emitter category does, and with neither it is a jet',
);
eq(T.shape('B744', 'A3'), 'quad', 'a known designator wins over the category');
eq(T.shape('ZZZZ', 'A7'), 'heli', 'an unlisted designator falls back to the category');
eq(T.shape('ec35', null), 'heli', 'case does not matter');
eq(
  shapes([['P8', 'A3'], ['IL86'], ['DHC7'], ['TEX2'], ['PC21'], ['AN2']]),
  ['jet', 'quad', 'quad', 'turboprop', 'turboprop', 'light'],
  'a twin-engined P-8, four-engined IL-86 and Dash 7, turboprop trainers, a piston An-2',
);
eq(
  shapes([['C135'], ['C141'], ['C160'], ['C212'], ['C152'], ['C206']]),
  ['quad', 'quad', 'turboprop', 'turboprop', 'light', 'light'],
  'military types with a Cessna-like designator are not light aircraft',
);
eq(
  T.head('3C6551', climb, { cs: 'DLH8AB', type: 'A321', x: { category: 'A3' } }).properties.shape,
  'jet',
  'the icon carries its symbol',
);

// --- the history window (fifteen minutes by default) --------------------
const old = [
  [t0 - 16 * 60, 8, 50, 100],
  [t0 - 14 * 60, 8, 50, 200],
  [t0 - 60, 8, 50, 300],
];
eq(T.trim(old, t0).length, 2, 'points past fifteen minutes are dropped');
eq(T.trim(old, t0)[0][3], 200, 'and the ones inside the window are kept in order');
eq(T.trim(climb, t0 + 10).length, climb.length, 'a fresh track is left alone');
eq(T.trim([], t0).length, 0, 'an empty track trims to empty');
eq(
  T.trim(old, t0, 30 * 60 * 1000).length,
  3,
  'a longer history from the bridge keeps older points',
);
eq(T.trim(old, t0, 5 * 60 * 1000).length, 1, 'a shorter one drops more');

// --- whole-fleet assembly ----------------------------------------------
const fleet = new Map([
  ['3C6551', { meta: { cs: 'DLH8AB', trk: 68, alt: 37000 }, pts: rejoin }],
  ['4009DA', { meta: { cs: 'BAW11', trk: 110, alt: 35000 }, pts: climb }],
  ['4CA739', { meta: { cs: null, trk: null, alt: null }, pts: [] }],
]);
const built = T.build(fleet, null, {});
eq(
  built.tracks.length,
  rejoin.length - 1 + (climb.length - 1),
  'segments come from every aircraft that has a track',
);
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
const fadeOf = (icao) => faded.planes.find((p) => p.properties.icao === icao).properties.fade;
ok(Math.abs(fadeOf('3C6551') - 0.4) < 1e-9, '58 s quiet: 2 s left of the 5 s fade');
eq(fadeOf('4009DA'), 1, 'one heard 4 s ago is untouched');
ok(
  faded.tracks
    .filter((s) => s.properties.icao === '3C6551')
    .every((s) => Math.abs(s.properties.fade - 0.4) < 1e-9),
  'its whole track fades with it, not only the icon',
);
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
eq(
  T.append(held, [
    [t0 + 24, 8.06, 50, 5000],
    [t0 + 22, 8.9, 51, 0],
  ]),
  1,
  'a point older than one added in the same update is refused too',
);
ok(
  held.every((p, i) => i === 0 || p[0] > held[i - 1][0]),
  'time only moves forward along a track',
);

// --- every case of the label ---------------------------------------------
// Each field at and either side of its boundaries, with the others unknown.
const part = (meta, k) => T.ornaments(meta)[k];
eq(
  [-2000, -300, -299, 0, 299, 300, 2000].map((vr) => part({ vr }, 'trend')),
  ['descend', 'descend', '', '', '', 'climb', 'climb'],
  'the arrow switches on at exactly CLIMB_FPM either way',
);
eq(
  [undefined, null].map((vr) => part({ vr }, 'trend')),
  ['', ''],
  'no vertical rate, no arrow',
);
eq(
  [-2000, 2000].map((vr) => part({ vr, gnd: 1 }, 'trend')),
  ['', ''],
  'on the ground no arrow either way',
);
eq(
  [
    undefined,
    {},
    { target: {} },
    { target: { modes: [] } },
    { target: { modes: ['ALT', 'VNAV', 'LNAV', 'APP'] } },
    { target: { modes: ['AP'] } },
    { target: { modes: ['LNAV', 'AP'] } },
  ].map((x) => part({ x }, 'ap')),
  [false, false, false, false, false, true, true],
  'AP only when the modes are known and include it',
);
eq(
  [undefined, false, true].map((mil) => part({ mil }, 'mil')),
  [false, false, true],
  'M only for a military address',
);
eq(
  [undefined, null, -350, 0, 49, 50, 4500, 34949, 34950, 99949, 99950, 126700].map((alt) =>
    part({ alt }, 'fl'),
  ),
  [
    '',
    '',
    'FL000',
    'FL000',
    'FL000',
    'FL001',
    'FL045',
    'FL349',
    'FL350',
    'FL999',
    'FL1000',
    'FL1267',
  ],
  'flight levels round to the nearest hundred feet, at least three digits, none below FL000',
);
eq(part({ alt: 35000, gnd: 1 }, 'fl'), '', 'no flight level on the ground');
eq(
  [undefined, null, 0, 0.49, 0.5, 451.49, 451.5, 1200].map((gs) => part({ gs }, 'spd')),
  ['', '', '0kt', '0kt', '1kt', '451kt', '452kt', '1200kt'],
  'ground speed rounds to the knot',
);
eq(part({ gs: 12, gnd: 1 }, 'spd'), '12kt', 'speed is shown on the ground too');

// Every combination of the five parts, against the label written out by hand:
// the markers after the callsign each with a space before it, the figures on
// a line of their own with a space between them.
let combos = 0;
for (const trend of ['', 'climb', 'descend'])
  for (const ap of [false, true])
    for (const mil of [false, true])
      for (const fl of ['', 'FL350'])
        for (const spd of ['', '452kt']) {
          const o = { trend, ap, mil, fl, spd };
          const marks = [trend && { climb: '▲', descend: '▼' }[trend], ap && 'AP', mil && 'M'];
          const figures = [fl, spd].filter(Boolean);
          const want =
            marks
              .filter(Boolean)
              .map((m) => ' ' + m)
              .join('') + (figures.length ? '\n' + figures.join(' ') : '');
          const got = T.labelParts(o);
          ok(got.join('') === want, 'label for ' + JSON.stringify(o) + ': ' + JSON.stringify(got));
          ok(
            got.every((p, i) => !p === ![...marks, fl, spd][i]),
            'a part is empty exactly when it is unknown: ' + JSON.stringify(o),
          );
          combos++;
        }
eq(combos, 48, 'all 48 combinations were tried');

// --- every case of the symbol ----------------------------------------------
eq(
  ['A0', 'A1', 'A2', 'A3', 'A4', 'A5', 'A6', 'A7'].map((c) => T.shape(null, c)),
  ['jet', 'light', 'regional', 'jet', 'jet', 'heavy', 'fighter', 'heli'],
  'every category in set A',
);
eq(
  ['B0', 'B1', 'B2', 'B3', 'B4', 'B5', 'B6', 'B7'].map((c) => T.shape(null, c)),
  ['jet', 'glider', 'balloon', 'jet', 'light', 'jet', 'drone', 'jet'],
  'every category in set B; parachutists, reserved and space vehicles fall back to a jet',
);
eq(
  ['C0', 'C1', 'C2', 'C3', 'C4', 'D0', undefined, null, ''].map((c) => T.shape(null, c)),
  ['jet', 'ground', 'ground', 'jet', 'jet', 'jet', 'jet', 'jet', 'jet'],
  'surface vehicles are ground; obstacles, set D and no category are a jet',
);
// One designator from each end of every family in the light-aircraft rule.
// prettier-ignore
const LIGHT_CODES = [
  'C120', 'C140', 'C150', 'C152', 'C162', 'C170', 'C172', 'C175', 'C177', 'C180', 'C182',
  'C185', 'C188', 'C195', 'C205', 'C206', 'C207', 'C210', 'C310', 'C340', 'C402', 'C414',
  'C421', 'C77R', 'C42', 'P28A', 'P28B', 'P28R', 'PA18', 'PA22', 'PA24', 'PA31', 'PA34',
  'PA44', 'PA46', 'SR20', 'SR22', 'DA20', 'DA40', 'DA42', 'DA50', 'DA62', 'DR40', 'BE33',
  'BE35', 'BE36', 'BE55', 'BE58', 'BE76', 'M20P', 'M20T', 'AA5', 'TB9', 'TB10', 'TB20',
  'TB21', 'RV4', 'RV7', 'RV10', 'RV14', 'P68', 'G115', 'Z42', 'c172',
];
eq(
  LIGHT_CODES.filter((code) => T.shape(code, 'A3') !== 'light'),
  [],
  'every light-aircraft family, even under a category that says otherwise',
);
// Designators that share a family's start but are not light aircraft: the
// whole designator has to match, so these fall back to their category.
// prettier-ignore
const NOT_LIGHT = [
  ['C101', 'A6', 'fighter', 'CASA C-101 Aviojet'], ['C123', 'A3', 'jet', 'Fairchild C-123 Provider'],
  ['C119', 'A3', 'jet', 'Fairchild C-119'], ['C121', 'A3', 'jet', 'Lockheed C-121'],
  ['C425', 'A2', 'regional', 'Cessna Conquest I'], ['C441', 'A2', 'regional', 'Cessna Conquest II'],
  ['C250', 'A3', 'jet', 'none'], ['C300', 'A3', 'jet', 'none'], ['C1', 'A3', 'jet', 'none'],
  ['C1720', 'A3', 'jet', 'too long'], ['P28', 'A3', 'jet', 'no variant letter'],
  ['PA42', 'A2', 'regional', 'Piper Cheyenne III, PAY3 elsewhere'], ['PA47', 'A3', 'jet', 'PiperJet'],
  ['SR21', 'A3', 'jet', 'none'], ['SR71', 'A3', 'jet', 'Lockheed SR-71'], ['DA10', 'A3', 'jet', 'none'],
  ['DA70', 'A3', 'jet', 'none'], ['DR30', 'A3', 'jet', 'none'], ['DR50', 'A3', 'jet', 'none'],
  ['BE34', 'A3', 'jet', 'none'], ['BE37', 'A3', 'jet', 'none'], ['BE56', 'A3', 'jet', 'none'],
  ['BE77', 'A3', 'jet', 'none'], ['M20', 'A3', 'jet', 'no variant letter'],
  ['TB30', 'A6', 'fighter', 'Socata TB 30 Epsilon'], ['TB11', 'A3', 'jet', 'none'],
  ['RV', 'A3', 'jet', 'no number'], ['RV100', 'A3', 'jet', 'too long'], ['P680', 'A3', 'jet', 'too long'],
  ['C42A', 'A3', 'jet', 'too long'], ['PA', 'A3', 'jet', 'no number'],
];
for (const [code, cat, want, name] of NOT_LIGHT)
  eq(T.shape(code, cat), want, code + ' (' + name + ') follows its category ' + cat);

eq(
  [
    ['C130', 'A1'],
    ['C208', 'A1'],
    ['C25A', 'A1'],
    ['C68A', 'A1'],
    ['C17', 'A1'],
  ].map(([code, cat]) => T.shape(code, cat)),
  ['quad', 'turboprop', 'bizjet', 'bizjet', 'quad'],
  'a listed designator wins over the light-aircraft families and the category',
);
eq(
  [
    ['A320', 'A5'],
    ['A320', undefined],
    ['XXXX', 'B2'],
  ].map(([code, cat]) => T.shape(code, cat)),
  ['heavy', 'jet', 'balloon'],
  'an unlisted designator leaves the category to decide',
);
eq(
  [{ type: 'B744' }, { type: 'B744', x: {} }, { x: { category: 'A7' } }, {}].map(
    (meta) => T.head('X', climb, meta).properties.shape,
  ),
  ['quad', 'quad', 'heli', 'jet'],
  'the icon takes its symbol from the type, or the category, or neither',
);

// --- every listed designator ----------------------------------------------
// Written out aircraft by aircraft, apart from the lists in tracks.js, so a
// designator put in the wrong group, or a group's list mistyped, shows here.
// prettier-ignore
const DESIGNATORS = [
  ['A342', 'quad', 'Airbus A340-200'],
  ['A343', 'quad', 'Airbus A340-300'],
  ['A345', 'quad', 'Airbus A340-500'],
  ['A346', 'quad', 'Airbus A340-600'],
  ['A388', 'quad', 'Airbus A380-800'],
  ['B741', 'quad', 'Boeing 747-100'],
  ['B742', 'quad', 'Boeing 747-200'],
  ['B743', 'quad', 'Boeing 747-300'],
  ['B744', 'quad', 'Boeing 747-400'],
  ['B748', 'quad', 'Boeing 747-8'],
  ['B74D', 'quad', 'Boeing 747-400 Domestic'],
  ['B74R', 'quad', 'Boeing 747SR'],
  ['B74S', 'quad', 'Boeing 747SP'],
  ['BLCF', 'quad', 'Boeing 747 Dreamlifter'],
  ['B461', 'quad', 'BAe 146-100'],
  ['B462', 'quad', 'BAe 146-200'],
  ['B463', 'quad', 'BAe 146-300'],
  ['RJ70', 'quad', 'Avro RJ70'],
  ['RJ85', 'quad', 'Avro RJ85'],
  ['RJ1H', 'quad', 'Avro RJ100'],
  ['A400', 'quad', 'Airbus A400M'],
  ['C130', 'quad', 'Lockheed C-130 Hercules'],
  ['C30J', 'quad', 'Lockheed C-130J'],
  ['C135', 'quad', 'Boeing C-135'],
  ['C141', 'quad', 'Lockheed C-141 StarLifter'],
  ['C17', 'quad', 'Boeing C-17 Globemaster III'],
  ['C5', 'quad', 'Lockheed C-5 Galaxy'],
  ['C5M', 'quad', 'Lockheed C-5M Super Galaxy'],
  ['E3TF', 'quad', 'Boeing E-3 Sentry (TF33)'],
  ['E3CF', 'quad', 'Boeing E-3 Sentry (CFM56)'],
  ['K35R', 'quad', 'Boeing KC-135R Stratotanker'],
  ['K35E', 'quad', 'Boeing KC-135E'],
  ['B703', 'quad', 'Boeing 707-300'],
  ['IL76', 'quad', 'Ilyushin Il-76'],
  ['IL86', 'quad', 'Ilyushin Il-86'],
  ['IL96', 'quad', 'Ilyushin Il-96'],
  ['A124', 'quad', 'Antonov An-124'],
  ['A225', 'quad', 'Antonov An-225'],
  ['P3', 'quad', 'Lockheed P-3 Orion'],
  ['DHC7', 'quad', 'de Havilland Canada Dash 7'],
  ['VC10', 'quad', 'Vickers VC10'],
  ['DC8', 'quad', 'Douglas DC-8'],
  ['CONI', 'quad', 'Lockheed Constellation'],
  ['L188', 'quad', 'Lockheed L-188 Electra'],
  ['A306', 'heavy', 'Airbus A300-600'],
  ['A30B', 'heavy', 'Airbus A300B'],
  ['A310', 'heavy', 'Airbus A310'],
  ['A332', 'heavy', 'Airbus A330-200'],
  ['A333', 'heavy', 'Airbus A330-300'],
  ['A337', 'heavy', 'Airbus BelugaXL'],
  ['A338', 'heavy', 'Airbus A330-800neo'],
  ['A339', 'heavy', 'Airbus A330-900neo'],
  ['A359', 'heavy', 'Airbus A350-900'],
  ['A35K', 'heavy', 'Airbus A350-1000'],
  ['B762', 'heavy', 'Boeing 767-200'],
  ['B763', 'heavy', 'Boeing 767-300'],
  ['B764', 'heavy', 'Boeing 767-400'],
  ['B772', 'heavy', 'Boeing 777-200'],
  ['B773', 'heavy', 'Boeing 777-300'],
  ['B77L', 'heavy', 'Boeing 777-200LR'],
  ['B77W', 'heavy', 'Boeing 777-300ER'],
  ['B778', 'heavy', 'Boeing 777-8'],
  ['B779', 'heavy', 'Boeing 777-9'],
  ['B788', 'heavy', 'Boeing 787-8'],
  ['B789', 'heavy', 'Boeing 787-9'],
  ['B78X', 'heavy', 'Boeing 787-10'],
  ['MD11', 'heavy', 'McDonnell Douglas MD-11'],
  ['DC10', 'heavy', 'McDonnell Douglas DC-10'],
  ['L101', 'heavy', 'Lockheed L-1011 TriStar'],
  ['KC10', 'heavy', 'McDonnell Douglas KC-10 Extender'],
  ['CRJ1', 'regional', 'Bombardier CRJ100'],
  ['CRJ2', 'regional', 'Bombardier CRJ200'],
  ['CRJ7', 'regional', 'Bombardier CRJ700'],
  ['CRJ9', 'regional', 'Bombardier CRJ900'],
  ['CRJX', 'regional', 'Bombardier CRJ1000'],
  ['E135', 'regional', 'Embraer ERJ 135'],
  ['E145', 'regional', 'Embraer ERJ 145'],
  ['E170', 'regional', 'Embraer 170'],
  ['E175', 'regional', 'Embraer 175'],
  ['E190', 'regional', 'Embraer 190'],
  ['E195', 'regional', 'Embraer 195'],
  ['E290', 'regional', 'Embraer 190-E2'],
  ['E295', 'regional', 'Embraer 195-E2'],
  ['E75L', 'regional', 'Embraer 175 (long wing)'],
  ['E75S', 'regional', 'Embraer 175 (short wing)'],
  ['F70', 'regional', 'Fokker 70'],
  ['F100', 'regional', 'Fokker 100'],
  ['BCS1', 'regional', 'Airbus A220-100'],
  ['BCS3', 'regional', 'Airbus A220-300'],
  ['SU95', 'regional', 'Sukhoi Superjet 100'],
  ['AJ27', 'regional', 'COMAC ARJ21'],
  ['J328', 'regional', 'Fairchild Dornier 328JET'],
  ['E35L', 'bizjet', 'Embraer Legacy 600'],
  ['C500', 'bizjet', 'Cessna Citation I'],
  ['C501', 'bizjet', 'Cessna Citation I/SP'],
  ['C510', 'bizjet', 'Cessna Citation Mustang'],
  ['C525', 'bizjet', 'Cessna CitationJet'],
  ['C25A', 'bizjet', 'Cessna CJ2'],
  ['C25B', 'bizjet', 'Cessna CJ3'],
  ['C25C', 'bizjet', 'Cessna CJ4'],
  ['C25M', 'bizjet', 'Cessna Citation M2'],
  ['C550', 'bizjet', 'Cessna Citation II'],
  ['C551', 'bizjet', 'Cessna Citation II/SP'],
  ['C55B', 'bizjet', 'Cessna Citation Bravo'],
  ['C560', 'bizjet', 'Cessna Citation V'],
  ['C56X', 'bizjet', 'Cessna Citation Excel'],
  ['C650', 'bizjet', 'Cessna Citation III'],
  ['C680', 'bizjet', 'Cessna Citation Sovereign'],
  ['C68A', 'bizjet', 'Cessna Citation Latitude'],
  ['C700', 'bizjet', 'Cessna Citation Longitude'],
  ['C750', 'bizjet', 'Cessna Citation X'],
  ['E50P', 'bizjet', 'Embraer Phenom 100'],
  ['E55P', 'bizjet', 'Embraer Phenom 300'],
  ['E545', 'bizjet', 'Embraer Praetor 500'],
  ['E550', 'bizjet', 'Embraer Praetor 600'],
  ['LJ31', 'bizjet', 'Learjet 31'],
  ['LJ35', 'bizjet', 'Learjet 35'],
  ['LJ40', 'bizjet', 'Learjet 40'],
  ['LJ45', 'bizjet', 'Learjet 45'],
  ['LJ60', 'bizjet', 'Learjet 60'],
  ['LJ70', 'bizjet', 'Learjet 70'],
  ['LJ75', 'bizjet', 'Learjet 75'],
  ['GLF2', 'bizjet', 'Gulfstream II'],
  ['GLF3', 'bizjet', 'Gulfstream III'],
  ['GLF4', 'bizjet', 'Gulfstream IV'],
  ['GLF5', 'bizjet', 'Gulfstream V'],
  ['GLF6', 'bizjet', 'Gulfstream G650'],
  ['GA5C', 'bizjet', 'Gulfstream G500'],
  ['GA6C', 'bizjet', 'Gulfstream G600'],
  ['GA7C', 'bizjet', 'Gulfstream G700'],
  ['GA8C', 'bizjet', 'Gulfstream G800'],
  ['GLEX', 'bizjet', 'Bombardier Global Express'],
  ['GL5T', 'bizjet', 'Bombardier Global 5000'],
  ['GL7T', 'bizjet', 'Bombardier Global 7500'],
  ['GL8T', 'bizjet', 'Bombardier Global 8000'],
  ['CL30', 'bizjet', 'Bombardier Challenger 300'],
  ['CL35', 'bizjet', 'Bombardier Challenger 350'],
  ['CL60', 'bizjet', 'Bombardier Challenger 600'],
  ['F2TH', 'bizjet', 'Dassault Falcon 2000'],
  ['F900', 'bizjet', 'Dassault Falcon 900'],
  ['FA10', 'bizjet', 'Dassault Falcon 10'],
  ['FA20', 'bizjet', 'Dassault Falcon 20'],
  ['FA50', 'bizjet', 'Dassault Falcon 50'],
  ['FA6X', 'bizjet', 'Dassault Falcon 6X'],
  ['FA7X', 'bizjet', 'Dassault Falcon 7X'],
  ['FA8X', 'bizjet', 'Dassault Falcon 8X'],
  ['H25B', 'bizjet', 'Hawker 800'],
  ['H25C', 'bizjet', 'Hawker 1000'],
  ['HA4T', 'bizjet', 'Hawker 4000'],
  ['HDJT', 'bizjet', 'Honda HA-420 HondaJet'],
  ['PC24', 'bizjet', 'Pilatus PC-24'],
  ['PRM1', 'bizjet', 'Beechcraft Premier I'],
  ['SF50', 'bizjet', 'Cirrus Vision Jet'],
  ['BE40', 'bizjet', 'Beechjet 400'],
  ['AT43', 'turboprop', 'ATR 42-300'],
  ['AT44', 'turboprop', 'ATR 42-400'],
  ['AT45', 'turboprop', 'ATR 42-500'],
  ['AT46', 'turboprop', 'ATR 42-600'],
  ['AT72', 'turboprop', 'ATR 72'],
  ['AT73', 'turboprop', 'ATR 72-210'],
  ['AT75', 'turboprop', 'ATR 72-500'],
  ['AT76', 'turboprop', 'ATR 72-600'],
  ['DH8A', 'turboprop', 'Dash 8-100'],
  ['DH8B', 'turboprop', 'Dash 8-200'],
  ['DH8C', 'turboprop', 'Dash 8-300'],
  ['DH8D', 'turboprop', 'Dash 8-400'],
  ['DHC6', 'turboprop', 'DHC-6 Twin Otter'],
  ['B190', 'turboprop', 'Beechcraft 1900'],
  ['BE20', 'turboprop', 'King Air 200'],
  ['BE30', 'turboprop', 'King Air 300'],
  ['B350', 'turboprop', 'King Air 350'],
  ['BE9L', 'turboprop', 'King Air 90'],
  ['BE9T', 'turboprop', 'King Air F90'],
  ['BE99', 'turboprop', 'Beechcraft 99'],
  ['SF34', 'turboprop', 'Saab 340'],
  ['JS31', 'turboprop', 'BAe Jetstream 31'],
  ['JS32', 'turboprop', 'BAe Jetstream 32'],
  ['JS41', 'turboprop', 'BAe Jetstream 41'],
  ['SW4', 'turboprop', 'Fairchild Metro'],
  ['D228', 'turboprop', 'Dornier 228'],
  ['D328', 'turboprop', 'Dornier 328'],
  ['F27', 'turboprop', 'Fokker F27'],
  ['F50', 'turboprop', 'Fokker 50'],
  ['PC12', 'turboprop', 'Pilatus PC-12'],
  ['C208', 'turboprop', 'Cessna Caravan'],
  ['C08T', 'turboprop', 'Cessna turbine conversion'],
  ['TBM7', 'turboprop', 'Socata TBM 700'],
  ['TBM8', 'turboprop', 'Socata TBM 850'],
  ['TBM9', 'turboprop', 'Daher TBM 900'],
  ['P180', 'turboprop', 'Piaggio Avanti'],
  ['L410', 'turboprop', 'Let L-410'],
  ['AN24', 'turboprop', 'Antonov An-24'],
  ['AN26', 'turboprop', 'Antonov An-26'],
  ['AN32', 'turboprop', 'Antonov An-32'],
  ['C160', 'turboprop', 'Transall C-160'],
  ['C212', 'turboprop', 'CASA C-212'],
  ['C295', 'turboprop', 'Airbus C295'],
  ['CN35', 'turboprop', 'CASA CN-235'],
  ['SB20', 'turboprop', 'Saab 2000'],
  ['E120', 'turboprop', 'Embraer Brasilia'],
  ['SH36', 'turboprop', 'Shorts 360'],
  ['PAY2', 'turboprop', 'Piper Cheyenne II'],
  ['PAY3', 'turboprop', 'Piper Cheyenne III'],
  ['PAY4', 'turboprop', 'Piper Cheyenne 400'],
  ['KODI', 'turboprop', 'Quest Kodiak'],
  ['PC21', 'turboprop', 'Pilatus PC-21'],
  ['PC7', 'turboprop', 'Pilatus PC-7'],
  ['PC9', 'turboprop', 'Pilatus PC-9'],
  ['TUCA', 'turboprop', 'Embraer Tucano'],
  ['TEX2', 'turboprop', 'Beechcraft T-6 Texan II'],
  ['EC20', 'heli', 'Eurocopter EC120'],
  ['EC25', 'heli', 'Airbus H225'],
  ['EC30', 'heli', 'Airbus H130'],
  ['EC35', 'heli', 'Airbus H135'],
  ['EC45', 'heli', 'Airbus H145'],
  ['EC55', 'heli', 'Airbus H155'],
  ['EC75', 'heli', 'Airbus H175'],
  ['H160', 'heli', 'Airbus H160'],
  ['AS32', 'heli', 'Airbus H215 Super Puma'],
  ['AS50', 'heli', 'Airbus H125 Ecureuil'],
  ['AS55', 'heli', 'Eurocopter AS355 Ecureuil 2'],
  ['AS65', 'heli', 'Eurocopter AS365 Dauphin'],
  ['B06', 'heli', 'Bell 206'],
  ['B06T', 'heli', 'Bell 206L TwinRanger'],
  ['B105', 'heli', 'MBB Bo 105'],
  ['B407', 'heli', 'Bell 407'],
  ['B412', 'heli', 'Bell 412'],
  ['B427', 'heli', 'Bell 427'],
  ['B429', 'heli', 'Bell 429'],
  ['B430', 'heli', 'Bell 430'],
  ['B505', 'heli', 'Bell 505'],
  ['BK17', 'heli', 'MBB/Kawasaki BK 117'],
  ['R22', 'heli', 'Robinson R22'],
  ['R44', 'heli', 'Robinson R44'],
  ['R66', 'heli', 'Robinson R66'],
  ['A109', 'heli', 'Leonardo AW109'],
  ['A119', 'heli', 'Leonardo AW119'],
  ['A139', 'heli', 'Leonardo AW139'],
  ['A169', 'heli', 'Leonardo AW169'],
  ['A189', 'heli', 'Leonardo AW189'],
  ['S76', 'heli', 'Sikorsky S-76'],
  ['S92', 'heli', 'Sikorsky S-92'],
  ['NH90', 'heli', 'NHIndustries NH90'],
  ['H60', 'heli', 'Sikorsky H-60 Black Hawk'],
  ['UH1', 'heli', 'Bell UH-1 Iroquois'],
  ['CH47', 'heli', 'Boeing CH-47 Chinook'],
  ['H47', 'heli', 'Boeing H-47 Chinook'],
  ['H64', 'heli', 'Boeing AH-64 Apache'],
  ['EH10', 'heli', 'Leonardo AW101'],
  ['LYNX', 'heli', 'Westland Lynx'],
  ['TIGR', 'heli', 'Airbus Tiger'],
  ['MI8', 'heli', 'Mil Mi-8'],
  ['MI24', 'heli', 'Mil Mi-24'],
  ['G2CA', 'heli', 'Guimbal Cabri G2'],
  ['F16', 'fighter', 'General Dynamics F-16'],
  ['F15', 'fighter', 'McDonnell Douglas F-15'],
  ['F18H', 'fighter', 'McDonnell Douglas F/A-18 Hornet'],
  ['F18S', 'fighter', 'Boeing F/A-18E/F Super Hornet'],
  ['F35', 'fighter', 'Lockheed Martin F-35'],
  ['F22', 'fighter', 'Lockheed Martin F-22'],
  ['F5', 'fighter', 'Northrop F-5'],
  ['EUFI', 'fighter', 'Eurofighter Typhoon'],
  ['TOR', 'fighter', 'Panavia Tornado'],
  ['RFAL', 'fighter', 'Dassault Rafale'],
  ['MIR2', 'fighter', 'Dassault Mirage 2000'],
  ['HAWK', 'fighter', 'BAE Hawk'],
  ['T38', 'fighter', 'Northrop T-38 Talon'],
  ['A10', 'fighter', 'Fairchild A-10'],
  ['AJET', 'fighter', 'Dassault/Dornier Alpha Jet'],
  ['M346', 'fighter', 'Leonardo M-346'],
  ['L39', 'fighter', 'Aero L-39 Albatros'],
  ['L159', 'fighter', 'Aero L-159 Alca'],
  ['MG29', 'fighter', 'Mikoyan MiG-29'],
  ['SU27', 'fighter', 'Sukhoi Su-27'],
  ['SU30', 'fighter', 'Sukhoi Su-30'],
  ['SU35', 'fighter', 'Sukhoi Su-35'],
  ['F4', 'fighter', 'McDonnell Douglas F-4 Phantom II'],
  ['ULAC', 'light', 'ultralight'],
  ['AN2', 'light', 'Antonov An-2'],
  ['T6', 'light', 'North American T-6 Texan'],
  ['GLID', 'glider', 'glider'],
  ['BALL', 'balloon', 'balloon'],
];
for (const [code, want, name] of DESIGNATORS)
  ok(
    T.shape(code, null) === want,
    code + ' (' + name + ') is ' + want + ', not ' + T.shape(code, null),
  );
// And nothing listed in tracks.js is left out of the table, or the other way.
eq(
  Object.keys(T.designators()).filter((code) => !DESIGNATORS.some((d) => d[0] === code)),
  [],
  'every designator tracks.js lists is in the table',
);
eq(
  DESIGNATORS.filter(([code, want]) => T.designators()[code] !== want).map((d) => d[0]),
  [],
  'every designator in the table is listed in tracks.js under the same symbol',
);
eq(
  new Set(DESIGNATORS.map((d) => d[0])).size,
  DESIGNATORS.length,
  'no designator twice in the table',
);

// --- the remaining defaults ------------------------------------------------
const fresh = [];
eq(T.append(fresh, climb.slice(0, 2)), 2, 'an empty track takes every point');
ok(!T.isGap(climb[0], climb[1], 5000), 'four seconds is no gap at a five-second threshold');
ok(T.isGap(climb[0], climb[1], 3000), 'but is one at three seconds');
eq(T.fade(55, 60, 10000), 0.5, 'a longer fade starts earlier');
eq(T.segments('X', climb).length, climb.length - 1, 'segments without options');
eq(T.build(new Map([['X', { meta: {}, pts: climb }]])).planes.length, 1, 'build without options');

// Where there is no globalThis, the module hangs itself on `this`.
{
  const vm = require('vm');
  const holder = {};
  vm.runInNewContext(
    '(function (globalThis, module) {\n' +
      require('fs').readFileSync(__dirname + '/tracks.js', 'utf8') +
      '\n}).call(holder)',
    { holder },
  );
  eq(typeof holder.Tracks.shape, 'function', 'loaded without globalThis, it sets this.Tracks');
}

// A designator put in two groups stops the page from loading, rather than
// silently taking the later group.
{
  const vm = require('vm');
  const src = require('fs').readFileSync(__dirname + '/tracks.js', 'utf8');
  const twice = src.replace("['balloon', 'BALL']", "['balloon', 'BALL GLID']");
  ok(twice !== src, 'the test found the balloon list to change');
  let thrown = null;
  try {
    vm.runInNewContext(twice, {});
  } catch (e) {
    thrown = e.message;
  }
  eq(thrown, 'GLID is listed as glider and balloon', 'a designator in two groups is refused');
}

// --- the receiver statistics -----------------------------------------------
{
  const site = [50, 8];
  const ranges = new Array(72).fill(0);
  ranges[0] = 60; // north, centred at 2.5 degrees
  ranges[18] = 60; // east
  ranges[36] = 60; // south
  const ring = T.rangeRing(site, 5, ranges);
  eq(ring.length, 4, 'three sectors make a ring of three points, closed');
  eq(ring[0], ring[3], 'the ring ends where it starts');
  ok(Math.abs(ring[0][1] - 51) < 0.01, '60 NM north is a degree of latitude: ' + ring[0]);
  ok(ring[0][0] > 8 && ring[0][0] < 8.1, 'a little east of north, the middle of the sector');
  // East is 92.5 degrees, the middle of its sector: a little south of 50.
  ok(ring[1][1] < 50 && ring[1][1] > 49.9 && ring[1][0] > 9.5, 'east: ' + ring[1]);
  ok(Math.abs(ring[2][1] - 49) < 0.01, 'south: ' + ring[2]);
  eq(T.rangeRing(site, 5, [10, 0, 10]), null, 'two sectors make no outline');
  eq(T.rangeRing(site, 5, new Array(72).fill(0)), null, 'no range, no outline');

  const t0 = 1000;
  eq(
    T.sparkPath(
      [
        [1000, 0],
        [1060, 5],
        [1120, 10],
      ],
      t0,
      1120,
      10,
      120,
      40,
      90,
    ),
    'M0 40 L60 20 L120 0',
    'points across the box, the highest at the top',
  );
  eq(
    T.sparkPath(
      [
        [1000, 5],
        [1060, 5],
        [1300, 5],
        [1360, 20],
      ],
      t0,
      1360,
      10,
      360,
      40,
      90,
    ),
    'M0 20 L60 20 M300 20 L360 0',
    'a gap starts a new line; values above the top are kept in the box',
  );
  eq(
    T.sparkPath([[900, 1]], t0, 1100, 10, 100, 40, 90),
    '',
    'points outside the time span are left out',
  );
  eq(
    T.sparkPath([[1000, 3]], t0, 1100, 0, 100, 40, 90),
    'M0 40',
    'nothing to scale by sits at the bottom',
  );
}

console.log(checks + ' checks, ' + fails + ' failures');
process.exit(fails ? 1 : 0);
