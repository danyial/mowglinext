#!/usr/bin/env bash
# dev-podman.sh — start a local arm64 podman container that mirrors the Pi5
# ROS2 build environment, so we can iterate on map_server_node etc. without a
# 12-min GHA cycle plus a 2-min pull on the Pi.
#
# Modes:
#   build       Run `colcon build` for the packages passed via PACKAGES (or
#               the full workspace if unset). Uses the Pi5's image as the base
#               so all rosdep deps are already installed.
#   shell       Open an interactive bash. Workspace is mounted read-write
#               under /ros2_ws/src so edits made on the Mac are immediately
#               visible inside the container.
#   service     Run a single-shot `ros2 service call` against the running
#               Pi5 (passes through the local DDS network — needs CYCLONEDDS_URI
#               configured to the Pi). Mostly useful for reproducing live-tune
#               bugs without waiting on a Pi5 deploy.
#
# Requires: podman (or docker, set ENGINE=docker), an arm64 host or
# qemu-user-static for x86 hosts.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENGINE="${ENGINE:-podman}"
IMAGE="${IMAGE:-ghcr.io/danyial/mowglinext/mowgli-ros2:migrate-upstream-localization}"
PACKAGES="${PACKAGES:-}"

mode="${1:-shell}"

case "$mode" in
  build)
    echo "==> colcon build${PACKAGES:+ for: $PACKAGES} (image: $IMAGE)"
    "$ENGINE" run --rm \
      --platform=linux/arm64 \
      -v "$REPO_ROOT/ros2/src:/ros2_ws/src:Z" \
      "$IMAGE" \
      bash -c "
        set -e
        source /opt/ros/kilted/setup.bash
        cd /ros2_ws
        colcon build --symlink-install ${PACKAGES:+--packages-select $PACKAGES}
      "
    ;;
  shell)
    echo "==> interactive shell (image: $IMAGE)"
    "$ENGINE" run --rm -it \
      --platform=linux/arm64 \
      -v "$REPO_ROOT/ros2/src:/ros2_ws/src:Z" \
      "$IMAGE" \
      bash
    ;;
  service)
    if [[ -z "${CYCLONEDDS_URI:-}" ]]; then
      echo "warning: CYCLONEDDS_URI is unset — discovery will not reach the Pi5." >&2
      echo "         Point it at install/config/cyclonedds.xml or unset to test against a local node only." >&2
    fi
    shift
    echo "==> ros2 service call $*"
    "$ENGINE" run --rm \
      --platform=linux/arm64 \
      ${CYCLONEDDS_URI:+-e CYCLONEDDS_URI="$CYCLONEDDS_URI" -v "$CYCLONEDDS_URI:$CYCLONEDDS_URI:ro"} \
      --network=host \
      "$IMAGE" \
      bash -c "source /opt/ros/kilted/setup.bash && ros2 service call $*"
    ;;
  *)
    echo "usage: $0 {build|shell|service} [args]" >&2
    exit 2
    ;;
esac
