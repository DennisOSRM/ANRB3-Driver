# ANRB3 driver

A userspace driver and Mode-S/ADS-B decoder for the AirNav RadarBox (USB
`0403:a2e0`), and a bridge that serves what it receives as a live map. No
vendor software, kernel module or `ftdi_sio` binding is involved: the device is
driven over libusb.

The box is an FTDI FT232R in front of a PIC that sends raw samples, eight per
Mode-S bit. It answers a challenge-response handshake, then streams bursts of
samples until told to stop. Demodulation, framing, error correction, CRC
validation and ADS-B decoding all happen here.

    driver/   the device, the decoder, the tracker, the feeds
    bridge/   the map: an HTTP and WebSocket server, and its page
    docker/   images for both

## Building

Rust 1.82 or later, and libusb 1.0 development headers for the driver.

    cargo build --release

That produces `target/release/anrb-rx` (the receiver), `anrb-replay` (offline
decoding of a recorded stream) and `anrb-map` (the bridge).

## Running the receiver

    anrb-rx --server

It claims the USB device, unlocks it, and serves what it decodes on two ports:

| port | format |
|---|---|
| 30005 | Mode-S Beast binary, every frame |
| 30003 | BaseStation/SBS-1 text, decoded fields |

Feeder clients read Beast on 30005 the same as from any other receiver. Without
`--server` it draws a dashboard instead; `--help` lists the rest.

The timestamp field in the Beast output is zero, the value that means "none".
This receiver has no 12 MHz clock and knows time only to the millisecond, so
filling the field from it would give timestamps unusable for multilateration.
ADS-B feeding works; switch MLAT off in any feeder that reads from this
receiver.

The driver needs read and write access to the USB device node.
`deploy/99-anrb.rules` gives the device node to the group `anrb`; add your
user to that group, or run as root.

## Running the bridge

    anrb-map --root bridge/web

It reads Beast from the receiver (`--feed sbs` for BaseStation, which is what
other receivers serve) and serves a map on port 8080 (`--http-port`), with
tracks coloured by altitude and a panel per aircraft. `--cert` and `--key` serve HTTPS, which the browser
requires before it will share the viewer's location.

Registration, type, operator, route and a photograph come from
[hexdb.io](https://hexdb.io/). The bridge asks, not the browser, and keeps the
answers in `~/.cache/anrb-map` (`--cache DIR`) for ten days, misses included.

`--no-hexdb` disables it: no outbound request is made, the cache directory is
not created, the `api/` and `photo/` endpoints answer 404,
and every update tells the page so, which stops it asking. The map still draws
tracks, altitudes, callsigns and everything the aircraft themselves transmit.

`--logos DIR` serves airline logos from a directory of `<ICAO code>.bmp` files,
and is likewise off unless asked for.

`bridge/web/countries.js` maps an address to its state of registry; it is
generated from the range table in
[tar1090](https://github.com/wiedehopf/tar1090)'s `flags.js`, which follows the
assignment table in ICAO Annex 10 Volume III.

## Deploying to another machine

    deploy/ship.sh pi@raspberrypi

builds both images on this machine for the target's architecture (arm64,
32-bit ARM or amd64, asked of the target), sends them over ssh, and starts
them there with `deploy/compose.yml`. The target needs Docker with the compose
plugin and nothing else: the code is cross-compiled here, and the images
contain no step that runs on the target's platform during the build, so no
emulator is needed either.

Both containers run as uid 10001, not root. The driver reaches the box through
the `anrb` group, which has to exist on the target along with the udev rule.
`ship.sh` copies the rule to `~/anrb/99-anrb.rules` on the target. Once, before
the first deployment, run on the target:

    sudo groupadd --system anrb
    sudo cp ~/anrb/99-anrb.rules /etc/udev/rules.d/
    sudo udevadm control --reload && sudo udevadm trigger

`ship.sh` looks up the group's number and passes it to compose. If the group
is missing, it stops with these instructions before building anything.

Compose publishes the bridge on the target's loopback only, at 127.0.0.1:8088. `deploy/nginx.conf` is a
snippet that serves it at `/radar/` from an existing nginx `server` block,
including the WebSocket upgrade; the page addresses everything relative to its
own path, so the prefix can be changed in the snippet alone.

## Docker

    docker build -f docker/Dockerfile -t anrb .
    docker run -d --name anrb --restart unless-stopped \
        -v /dev/bus/usb:/dev/bus/usb --device-cgroup-rule='c 189:* rmw' \
        --group-add "$(getent group anrb | cut -d: -f3)" \
        -v /etc/localtime:/etc/localtime:ro \
        -p 30003:30003 -p 30005:30005 anrb

The whole USB bus is passed through rather than one device node: the driver
power-cycles the box when it finds it wedged, and it comes back as a different
device number that a `--device` binding cannot follow. The image runs as uid
10001, so `--group-add` with the group from the udev rule is what lets it open
the device. The `/etc/localtime` mount gives the container the host's time
zone; without it, BaseStation timestamps and log lines are in UTC.

    docker build -f docker/Dockerfile --target bridge -t anrb-map .

The bridge image expects the driver at `anrb:30005`; `--host` changes that.

## Decoding

The device sends samples, not frames, so the decoder does the work a Beast
receiver does in hardware:

- A three-stage demodulator that computes the group sums as nibble popcounts
  of the packed samples, and evaluates the threshold recurrence as a parallel
  prefix scan over Boolean functions.
- A search over 36 framing configurations per burst, with a single-entry memo
  that skips the repeated work when the sample offset has not changed.
- CRC-24 validation, one-bit correction from a syndrome table, and a two-bit
  search guided by demodulator confidence rather than tried blind.
- SSSE3 and NEON kernels with a scalar fallback, each checked against the
  scalar path by the test suite.

Address-overlaid formats (DF0/4/5/16/20/21) carry no self-validating parity, so
they are accepted only for addresses already learned from CRC-validated
traffic, and only when no two trusted addresses sit close enough in address
space to be confused by a single bit error.

Positions are published only once two fixes agree: a CPR pair decoded from a
frame the decoder repaired can otherwise place an aircraft a whole zone away,
and a single corrupt message should not move it there.

## Tests

    cargo test --release

The driver's tests cover the decoder, the CRC and its correction paths, CPR,
the tracker's confirmation rules, the feeds and the terminal dashboard. The
bridge's cover its HTTP and WebSocket handling, the cache, and the track
bookkeeping the page depends on. Three integration tests read capture files
from `captures/`, which is not in this repository: two read `bursts.bin` and
one reads `tuning_15min_v2.raw`. They pass without checking anything when the
files are absent.

    node bridge/web/test_tracks.cjs

checks the drawing rules the page applies - what counts as a gap in a track,
which altitude colours a leg, when a point leaves the cache.
