#!/bin/sh
# Build the receiver's images here and start them on another machine.
#
#     deploy/ship.sh pi@raspberrypi
#
# The target's platform is asked of the target, the images are cross-compiled
# for it on this machine, and they travel over ssh as a Docker save stream.
# The target needs Docker and the compose plugin, and nothing else: it never
# builds or pulls. compose.yml, nginx.conf and 99-anrb.rules are copied to
# ~/anrb there.
set -eu

host=${1:?usage: deploy/ship.sh user@host}
here=$(cd "$(dirname "$0")/.." && pwd)

arch=$(ssh "$host" uname -m)
case "$arch" in
    aarch64|arm64) platform=linux/arm64 ;;
    armv7l|armv6l) platform=linux/arm/v7 ;;
    x86_64)        platform=linux/amd64 ;;
    *) echo "ship.sh: no image for $arch" >&2; exit 1 ;;
esac

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

echo "building for $platform"
for target in driver bridge; do
    name=anrb; [ "$target" = bridge ] && name=anrb-map
    docker buildx build --platform "$platform" -f "$here/docker/Dockerfile" \
        --target "$target" -t "$name:latest" --load "$here"
done

echo "sending the images to $host"
docker save anrb:latest anrb-map:latest | gzip -1 | ssh "$host" 'gunzip | docker load'

ssh "$host" "echo ANRB_USB_GID=$gid > anrb/.env"
ssh "$host" 'cd anrb && docker compose up -d && docker compose ps'
