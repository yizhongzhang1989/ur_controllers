#!/usr/bin/env bash
# run_tests.sh — Unified test entrypoint.
#
# Runs, in order:
#   1. Standalone unit tests under tests/unit/ (gtest binaries or pytest).
#   2. colcon test (if the workspace has been built).
#   3. Integration / launch tests under tests/integration/ (pytest).
#
# Exits 0 only if every stage passes. Missing stages are skipped with a
# warning, not an error — this lets the script be used from M0 onwards.
#
# Flags:
#   --dry-run    Print what would run, do nothing, exit 0.
#   --unit-only  Run only unit tests.

set -u

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

DRY_RUN=0
UNIT_ONLY=0
for arg in "$@"; do
    case "$arg" in
        --dry-run) DRY_RUN=1 ;;
        --unit-only) UNIT_ONLY=1 ;;
        -h|--help)
            sed -n '2,16p' "$0"
            exit 0 ;;
        *) echo "unknown arg: $arg" >&2; exit 2 ;;
    esac
done

fail=0
ran_any=0

say() { printf '[run_tests] %s\n' "$*"; }

run_or_dry() {
    if (( DRY_RUN )); then
        say "DRY: $*"
    else
        say "RUN: $*"
        "$@" || return 1
    fi
}

# --- 1. Unit tests -----------------------------------------------------------
say "stage 1: unit tests"
if compgen -G "tests/unit/test_*.py" > /dev/null; then
    ran_any=1
    if ! run_or_dry python3 -m pytest -q tests/unit; then
        say "unit tests FAILED"
        fail=1
    fi
elif compgen -G "tests/unit/*" > /dev/null; then
    say "tests/unit/ has files but no test_*.py pattern; skipping"
else
    say "no tests/unit/test_*.py — skipping"
fi

if (( UNIT_ONLY )); then
    (( fail == 0 )) && say "done (unit-only, green)" || say "done (unit-only, red)"
    exit "$fail"
fi

# --- 2. colcon test ----------------------------------------------------------
say "stage 2: colcon test"
if command -v colcon >/dev/null 2>&1 && compgen -G "src/*/package.xml" >/dev/null; then
    if [[ -d build && -d install ]]; then
        ran_any=1
        if ! run_or_dry colcon test --event-handlers console_direct+ --return-code-on-test-failure; then
            say "colcon test FAILED"
            fail=1
        fi
    else
        say "no build/ or install/ — skipping colcon test (build first)"
    fi
else
    say "colcon not available or no packages — skipping"
fi

# --- 3. Integration / launch tests ------------------------------------------
say "stage 3: integration tests"
if compgen -G "tests/integration/test_*.py" > /dev/null; then
    ran_any=1
    if ! run_or_dry python3 -m pytest -q tests/integration; then
        say "integration tests FAILED"
        fail=1
    fi
else
    say "no tests/integration/test_*.py — skipping"
fi

# --- summary -----------------------------------------------------------------
if (( ran_any == 0 )); then
    say "warning: no tests ran (scaffolding stage). Exiting 0."
    exit 0
fi

if (( fail == 0 )); then
    say "all test stages passed"
else
    say "one or more test stages FAILED"
fi
exit "$fail"
