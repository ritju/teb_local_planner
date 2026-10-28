#!/bin/sh

# CMake invokes this launcher as:
#   serialize_compile.sh --lock <lock-file> <compiler> <compiler arguments...>
# Keep one lock for this package even when Make changes directory for a target.
# flock is provided by util-linux in the ROS base image.
set -eu

case "${1-}" in
  --lock)
  lock_file=$2
  shift 2
  ;;
  *)
  lock_file=${TEB_COMPILE_LOCK_FILE:-/tmp/teb_local_planner-compile.lock}
  ;;
esac
mkdir -p "$(dirname "$lock_file")"
exec flock -x "$lock_file" "$@"
