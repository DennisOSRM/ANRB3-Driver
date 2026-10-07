#!/bin/sh
# Build the receiver's images and start them on another machine.
#
#     deploy/ship.sh pi@raspberrypi
#     deploy/ship.sh --build-on-target pi@raspberrypi
#
# By default the images are cross-compiled on this machine for the target's
# platform and travel over ssh as a Docker save stream; the target never
# builds or pulls. With --build-on-target, the sources travel instead and the
# target builds the images natively, so this machine needs no Docker at all.
#
# The target needs Docker and the compose plugin, and with --build-on-target
# also buildx. compose.yml, nginx.conf and 99-anrb.rules are copied to ~/anrb
# there.
set -eu

on_target=false
if [ "${1:-}" = --build-on-target ]; then
    on_target=true
    shift
fi
host=${1:?usage: deploy/ship.sh [--build-on-target] user@host}
here=$(cd "$(dirname "$0")/.." && pwd)

arch=$(ssh "$host" uname -m)
case "$arch" in
    aarch64|arm64) platform=linux/arm64 ;;
    armv7l|armv6l) platform=linux/arm/v7 ;;
    x86_64)        platform=linux/amd64 ;;
    *) echo "ship.sh: no image for $arch" >&2; exit 1 ;;
esac

if $on_target && ! ssh "$host" docker buildx version >/dev/null 2>&1; then
    echo "ship.sh: $host has no docker buildx, which the Dockerfile needs." >&2
    echo "On Debian or Ubuntu: sudo apt install docker-buildx" >&2
    exit 1
fi

ssh "$host" mkdir -p anrb
scp -q "$here/deploy/compose.yml" "$here/deploy/nginx.conf" "$here/deploy/99-anrb.rules" "$host:anrb/"

# The driver runs unprivileged, in the group the udev rule gives the device
# to. That group's number differs from machine to machine, so it is looked up
# there and handed to compose. This runs before the build, so a missing group
# stops the script early.
gid=$(ssh "$host" 'getent group anrb | cut -d: -f3')
if [ -z "$gid" ]; then
    echo "ship.sh: $host has no group anrb, so the driver could not open the device." >&2
    echo "On $host, once:" >&2
    echo "  sudo groupadd --system anrb" >&2
    echo "  sudo cp ~/anrb/99-anrb.rules /etc/udev/rules.d/" >&2
    echo "  sudo udevadm control --reload && sudo udevadm trigger" >&2
    exit 1
fi

if $on_target; then
    # The files git tracks, as they are in the working tree: the same sources
    # a build here would use, without build output or untracked files. They
    # are removed again when the build ends, whether it worked or not. Build
    # cache older than 30 days is pruned; newer cache keeps the next build to
    # the changed crates.
    echo "sending the sources to $host"
    (cd "$here" && git ls-files -z | tar --null -T - -cf -) |
        ssh "$host" 'rm -rf anrb/src && mkdir -p anrb/src && tar -x -C anrb/src'
    echo "building for $platform on $host"
    ssh "$host" "set -e; cd anrb/src
        trap 'rm -rf ~/anrb/src' EXIT
        for target in driver bridge; do
            name=anrb; [ \$target = bridge ] && name=anrb-map
            docker buildx build --platform $platform -f docker/Dockerfile \
                --target \$target -t \$name:latest --load .
        done
        docker buildx prune -f --filter until=720h >/dev/null"
else
    echo "building for $platform"
    for target in driver bridge; do
        name=anrb; [ "$target" = bridge ] && name=anrb-map
        docker buildx build --platform "$platform" -f "$here/docker/Dockerfile" \
            --target "$target" -t "$name:latest" --load "$here"
    done

    echo "sending the images to $host"
    docker save anrb:latest anrb-map:latest | gzip -1 | ssh "$host" 'gunzip | docker load'
fi

# .env may hold other settings; only the group's line is replaced. The new
# file starts as a copy of the old one, so it keeps the old one's permissions.
ssh "$host" "cd anrb && touch .env && cp -p .env .env.new &&
    { grep -v '^ANRB_USB_GID=' .env; echo ANRB_USB_GID=$gid; } > .env.new && mv .env.new .env"
ssh "$host" 'cd anrb && docker compose up -d && docker compose ps'
