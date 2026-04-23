#!/usr/bin/env bash
# record_all_crisp_baselines.sh — Record baseline rosbags + manifests for every
# crisp role × UR arm combination exercised in M2.
#
# Thin wrapper around scripts/record_crisp_baseline.sh. Runs the 3×2 grid
# sequentially (only one sim can run at a time). Continues on failure so a
# single flaky cold-run does not invalidate the rest; logs failures at the
# end.

set -e
set -o pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

DURATION_S="${1:-10}"

ROLES=(joint cartesian gravity)
ROBOTS=(ur5e ur15)

failed=()
for role in "${ROLES[@]}"; do
    for robot in "${ROBOTS[@]}"; do
        echo
        echo "=== record: role=$role robot=$robot ==="
        if ! bash "$REPO_ROOT/scripts/record_crisp_baseline.sh" "$role" "$robot" "$DURATION_S"; then
            echo "!!! FAILED: role=$role robot=$robot" >&2
            failed+=("${role}_${robot}")
        fi
        sleep 3
    done
done

if (( ${#failed[@]} )); then
    echo
    echo "Failures: ${failed[*]}" >&2
    exit 1
fi
echo
echo "All 6 role×arm baselines recorded."
