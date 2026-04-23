#!/usr/bin/env bash
# setup_env.sh — install ur_controllers workspace prerequisites.
#
# Thin idempotent wrapper around `apt`, `rosdep`, and `pip`. Installs the
# system packages listed in README.md "Prerequisites" plus the build /
# test tooling our scripts assume, runs `rosdep install` with the
# ADR-0005 skip list, and (optionally) installs the pre-commit hooks
# locally so a fresh checkout can run `pre-commit run --all-files`
# without further hunting.
#
# Safe to re-run; `apt-get install` and `pip install --upgrade` are
# idempotent, and `rosdep init` is guarded.
#
# Flags:
#   --dry-run         Print what would run, do nothing, exit 0.
#   --no-pre-commit   Skip the pre-commit / ruff pip install step.
#   -h | --help       Print this header.

set -u

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

DRY_RUN=0
SKIP_PRE_COMMIT=0
for arg in "$@"; do
    case "$arg" in
        --dry-run) DRY_RUN=1 ;;
        --no-pre-commit) SKIP_PRE_COMMIT=1 ;;
        -h|--help)
            sed -n '2,19p' "$0"
            exit 0 ;;
        *) echo "unknown arg: $arg" >&2; exit 2 ;;
    esac
done

say() { printf '[setup_env] %s\n' "$*"; }

run_or_dry() {
    if (( DRY_RUN )); then
        say "DRY: $*"
    else
        say "RUN: $*"
        "$@" || return 1
    fi
}

# Use sudo only when not already root and sudo is available.
SUDO=""
if [[ ${EUID:-$(id -u)} -ne 0 ]]; then
    if command -v sudo >/dev/null 2>&1; then
        SUDO="sudo"
    fi
fi

fail=0

# --- 1. apt packages --------------------------------------------------------
# Sourced from README.md "Prerequisites" plus the build / test tooling the
# CI workflow installs explicitly. Keep this list in sync with both.
APT_PACKAGES=(
    python3
    python3-pip
    python3-colcon-common-extensions
    python3-colcon-defaults
    python3-rosdep
    python3-vcstool
    python3-pytest
    python3-yaml
    git
    build-essential
    ros-humble-mujoco-ros2-control
    ros-humble-ur-description
    ros-humble-ros2-control
    ros-humble-ros2-controllers
    ros-humble-pinocchio
    ros-humble-rosbridge-suite
    ros-humble-ros-gz
    ros-humble-gz-ros2-control
    ros-humble-xacro
)

say "stage 1: apt install (${#APT_PACKAGES[@]} packages)"
if ! run_or_dry $SUDO apt-get update; then
    say "apt-get update FAILED"
    fail=1
fi
if ! run_or_dry env DEBIAN_FRONTEND=noninteractive $SUDO apt-get install -y \
        --no-install-recommends "${APT_PACKAGES[@]}"; then
    say "apt-get install FAILED"
    fail=1
fi

# --- 2. rosdep --------------------------------------------------------------
# `rosdep init` errors out if the sources file already exists (the ROS apt
# package usually pre-creates it); guard so re-runs stay idempotent. Skip
# the same packages that ADR-0005 strips from the colcon build so rosdep
# does not try to resolve their MuJoCo / catkin deps.
say "stage 2: rosdep"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    if ! run_or_dry $SUDO rosdep init; then
        say "rosdep init FAILED (continuing — may already be initialised)"
    fi
fi
if ! run_or_dry rosdep update; then
    say "rosdep update FAILED"
    fail=1
fi
if ! run_or_dry rosdep install --from-paths src third_party --ignore-src -r -y \
        --skip-keys "cartesian_controller_simulation cartesian_controller_tests"; then
    say "rosdep install FAILED"
    fail=1
fi

# --- 3. pre-commit + ruff (developer tooling) ------------------------------
if (( SKIP_PRE_COMMIT == 0 )); then
    say "stage 3: pip install pre-commit + ruff (user-site)"
    if ! run_or_dry python3 -m pip install --user --upgrade pre-commit ruff; then
        say "pip install pre-commit/ruff FAILED"
        fail=1
    fi
    # Wire the local git hook so `git commit` runs the same checks CI does.
    # `pre-commit install` is idempotent.
    if ! run_or_dry pre-commit install; then
        say "pre-commit install FAILED (is ~/.local/bin on PATH?)"
        fail=1
    fi
else
    say "stage 3: skipped (--no-pre-commit)"
fi

if (( fail == 0 )); then
    say "all stages passed"
else
    say "one or more stages FAILED"
fi
exit "$fail"
