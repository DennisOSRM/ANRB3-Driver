# syntax=docker/dockerfile:1
#
# Two images from one build. The default target is the driver, which holds the
# USB device and serves the feeds; `--target bridge` is the map bridge, which
# reads those feeds over TCP and serves a page.
#
# The images are built on one machine and run on another. The compiler always
# runs natively on the build machine and cross-compiles for the platform asked
# for, and the images it produces contain no step that has to run on that
# platform, so no emulator is involved and the target machine never compiles
# anything:
#
#     docker buildx build --platform linux/arm64 -f docker/Dockerfile -t anrb --load .
#     docker buildx build --platform linux/arm64 -f docker/Dockerfile --target bridge -t anrb-map --load .
#
# linux/amd64, linux/arm64 (a 64-bit Raspberry Pi) and linux/arm/v7 (a 32-bit
# one) are supported. deploy/ship.sh does all of it and sends the result over.

FROM --platform=$BUILDPLATFORM rust:1-bookworm AS build
ARG TARGETARCH
ARG TARGETVARIANT

# The Rust target, the GNU triple its C toolchain and libraries use, and the
# Debian architecture - the same machine under three names.
RUN set -eu; \
    case "$TARGETARCH${TARGETVARIANT:+/$TARGETVARIANT}" in \
      amd64)  echo "x86_64-unknown-linux-gnu x86_64-linux-gnu amd64" ;; \
      arm64)  echo "aarch64-unknown-linux-gnu aarch64-linux-gnu arm64" ;; \
      arm/v7) echo "armv7-unknown-linux-gnueabihf arm-linux-gnueabihf armhf" ;; \
      *) echo "unsupported platform: $TARGETARCH $TARGETVARIANT" >&2; exit 1 ;; \
    esac > /target

# libusb for the target, and a cross compiler when the target is not this
# machine: the linker needs one, and so does ring, which the bridge's TLS
# builds from C.
RUN set -eu; read triple gnu deb < /target; \
    native=$(dpkg --print-architecture); \
    if [ "$deb" != "$native" ]; then dpkg --add-architecture "$deb"; fi; \
    apt-get update; \
    apt-get install -y --no-install-recommends pkg-config "libusb-1.0-0-dev:$deb" "libudev1:$deb" \
        $( [ "$deb" != "$native" ] && echo "gcc-$gnu libc6-dev-$deb-cross" ); \
    rm -rf /var/lib/apt/lists/*; \
    rustup target add "$triple"

WORKDIR /src
COPY Cargo.toml Cargo.lock ./
COPY driver/ driver/
COPY bridge/ bridge/

# Debug info is for profiling on a development machine; an image has no use
# for it, and it is most of the binary's size.
ENV CARGO_PROFILE_RELEASE_DEBUG=false

# The binaries go to /out/bin, the two shared libraries the driver needs at
# run time to /out/lib, and the page to /out/web without test_tracks.cjs, its
# unit test. The runtime images take them with a COPY rather than install
# anything.
RUN --mount=type=cache,target=/usr/local/cargo/registry \
    --mount=type=cache,target=/src/target \
    set -eu; read triple gnu deb < /target; \
    export PKG_CONFIG_ALLOW_CROSS=1 PKG_CONFIG_LIBDIR="/usr/lib/$gnu/pkgconfig"; \
    if [ "$deb" != "$(dpkg --print-architecture)" ]; then \
        upper=$(echo "$triple" | tr 'a-z-' 'A-Z_'); lower=$(echo "$triple" | tr '-' '_'); \
        eval "export CARGO_TARGET_${upper}_LINKER=$gnu-gcc CC_${lower}=$gnu-gcc"; \
    fi; \
    # Two builds, not one: built together, Cargo would unify the driver
    # library's features and hand the bridge the USB code it opts out of,
    # libusb and all, which its image does not carry.
    cargo build --release --locked --target "$triple" -p anrb --bin anrb-rx; \
    cargo build --release --locked --target "$triple" -p anrb-map; \
    mkdir -p /out/bin /out/lib; \
    cp "target/$triple/release/anrb-rx" "target/$triple/release/anrb-map" /out/bin/; \
    cp -L "/usr/lib/$gnu/libusb-1.0.so.0" "/usr/lib/$gnu/libudev.so.1" /out/lib/; \
    mkdir -p /out/cache; \
    cp -R bridge/web /out/web; rm /out/web/test_tracks.cjs

# ---- the driver -------------------------------------------------------------

FROM debian:bookworm-slim AS driver

LABEL org.opencontainers.image.title="anrb" \
      org.opencontainers.image.description="AirNav RadarBox driver serving Beast (30005) and BaseStation (30003)" \
      org.opencontainers.image.licenses="BSD-2-Clause"

# Copied rather than installed, so that building this image for another
# platform runs nothing on that platform.
COPY --from=build /out/lib/ /usr/local/lib/anrb/
ENV LD_LIBRARY_PATH=/usr/local/lib/anrb
COPY --from=build /out/bin/anrb-rx /usr/local/bin/anrb-rx

# Not root. The driver needs nothing but read and write on the RadarBox's USB
# device node, and that is granted to a group on the host by a udev rule
# (deploy/99-anrb.rules), which the container joins with --group-add. Run
# without that group, it cannot open the device and logs an error.
USER 10001:10001

EXPOSE 30003/tcp 30005/tcp

# The health check connects to the Beast port rather than checking the process.
HEALTHCHECK --interval=30s --timeout=5s --start-period=30s \
    CMD bash -c 'exec 3<>/dev/tcp/127.0.0.1/30005'

ENTRYPOINT ["anrb-rx"]
CMD ["--server"]

# ---- the bridge -------------------------------------------------------------

FROM debian:bookworm-slim AS bridge

LABEL org.opencontainers.image.title="anrb-map" \
      org.opencontainers.image.description="Live map bridge for the AirNav RadarBox driver" \
      org.opencontainers.image.licenses="BSD-2-Clause"

# Nothing to install: the bridge links no C library, and the root certificates
# it checks hexdb.io against are compiled in.
COPY --from=build /out/bin/anrb-map /usr/local/bin/anrb-map
COPY --from=build /out/web/ /srv/web/
# Owned by the user the bridge runs as, so that a volume mounted here starts
# out writable by it: Docker copies the ownership into a new named volume.
COPY --from=build --chown=10001:10001 /out/cache/ /var/cache/anrb-map/

# Not root: it reads a TCP feed, serves HTTP and writes its cache.
USER 10001:10001

EXPOSE 8080/tcp
VOLUME ["/var/cache/anrb-map"]

HEALTHCHECK --interval=30s --timeout=5s --start-period=10s \
    CMD bash -c 'exec 3<>/dev/tcp/127.0.0.1/8080'

ENTRYPOINT ["anrb-map"]
CMD ["--root", "/srv/web", "--cache", "/var/cache/anrb-map", "--host", "anrb"]
