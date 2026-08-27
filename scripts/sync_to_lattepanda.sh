#!/usr/bin/env bash
# Rsyncs this repo to the LattePanda (or any real-hardware build target) --
# same pattern used by the earlier Python project, since this dev sandbox
# has no raw-socket-capable NIC and can't run EtherCAT traffic itself.
#
# Usage: scripts/sync_to_lattepanda.sh <user>@<host> [remote-path]
set -euo pipefail

if [ "$#" -lt 1 ]; then
    echo "Usage: $0 <user>@<host> [remote-path]" >&2
    exit 2
fi

DEST="$1"
REMOTE_PATH="${2:-otto_suite}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

rsync -avz --delete \
    --exclude 'build/' \
    --exclude '.git/' \
    --exclude '.claude/' \
    "${REPO_ROOT}/" "${DEST}:${REMOTE_PATH}/"

echo "Synced to ${DEST}:${REMOTE_PATH}"
echo "On the remote machine:"
echo "  cd ${REMOTE_PATH} && cmake -B build && cmake --build build"
