#!/usr/bin/env bash
# auto_dev_loop.sh — Outer driver for automated iterative development.
#
# Each iteration:
#   1. Refresh docs/STATUS.md via status_check.sh.
#   2. Invoke the coding agent (Copilot CLI) with a prompt that forces it to
#      pick exactly one task, implement it with tests, and commit.
#   3. Run scripts/run_tests.sh as a hard gate.
#   4. Detect no-progress and stop conditions.
#
# Stop conditions:
#   - `.STOP` file at repo root.
#   - MAX_ITERS reached (default 20, override via env).
#   - Two consecutive iterations with no new commit.
#   - All ROADMAP milestones checked off.
#
# Usage:
#   scripts/auto_dev_loop.sh                 # default config
#   MAX_ITERS=5 scripts/auto_dev_loop.sh     # cap iterations
#   COPILOT_CMD="copilot -p" scripts/auto_dev_loop.sh   # override agent cmd

set -u

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

MAX_ITERS="${MAX_ITERS:-20}"
COPILOT_CMD="${COPILOT_CMD:-copilot}"   # override if your CLI differs
LOG_DIR="$REPO_ROOT/.auto_dev_logs"
mkdir -p "$LOG_DIR"

log() { printf '[auto_dev_loop] %s\n' "$*" | tee -a "$LOG_DIR/loop.log"; }

prev_head="$(git rev-parse HEAD 2>/dev/null || echo none)"
no_progress_streak=0

for ((i = 1; i <= MAX_ITERS; i++)); do
    log "=== iteration $i / $MAX_ITERS ==="

    if [[ -f "$REPO_ROOT/.STOP" ]]; then
        log "stop: .STOP file present"
        break
    fi

    # All-done check: no unchecked items in ROADMAP.
    if ! grep -qE '^\s*-\s*\[[ ~!]\]' docs/ROADMAP.md; then
        log "stop: all ROADMAP items complete"
        break
    fi

    # 1. Refresh state.
    scripts/status_check.sh > "$LOG_DIR/status_$i.log" 2>&1 || \
        log "warning: status_check.sh failed (continuing)"

    # 2. Compose agent prompt.
    prompt=$(cat <<'EOF'
You are working in the ur_controllers repository. Follow AGENTS.md strictly.

Steps you MUST take this iteration:
1. Read AGENTS.md, docs/ROADMAP.md, docs/STATUS.md, docs/DECISIONS.md,
   and `git log --oneline -n 20`.
2. Identify the SINGLE next highest-value unchecked task. Do not attempt
   more than one task.
3. Implement it. Add or extend tests in the appropriate tests/ subdir.
4. Run scripts/run_tests.sh. Iterate until it exits 0.
5. Commit with a Conventional Commits message (one logical change).
6. Update docs/STATUS.md to reflect the new state, and append to
   docs/DECISIONS.md if you made an architectural decision.
7. If the task is blocked (needs human input, requires real hardware, or
   needs a submodule bump), do NOT guess. Update docs/STATUS.md with a
   clear blocker entry, commit only the STATUS update, and stop.

Do not modify files under third_party/. Do not push. Do not weaken tests.
EOF
)

    # 3. Invoke agent (user can override COPILOT_CMD for a different CLI).
    log "invoking agent: $COPILOT_CMD"
    if ! $COPILOT_CMD "$prompt" 2>&1 | tee "$LOG_DIR/agent_$i.log"; then
        log "warning: agent command exited non-zero"
    fi

    # 4. Gate: tests must be green after the iteration.
    if ! scripts/run_tests.sh > "$LOG_DIR/tests_$i.log" 2>&1; then
        log "tests failed; asking agent to fix"
        fix_prompt="scripts/run_tests.sh failed. Read the latest test log in $LOG_DIR, diagnose, fix, re-run until green, then amend or add a fix commit. Do not weaken tests."
        $COPILOT_CMD "$fix_prompt" 2>&1 | tee "$LOG_DIR/agent_fix_$i.log" || true
        if ! scripts/run_tests.sh > "$LOG_DIR/tests_${i}_after_fix.log" 2>&1; then
            log "stop: tests still failing after fix attempt; manual intervention required"
            break
        fi
    fi

    # 5. Progress check.
    new_head="$(git rev-parse HEAD)"
    if [[ "$new_head" == "$prev_head" ]]; then
        no_progress_streak=$((no_progress_streak + 1))
        log "no new commit this iteration (streak=$no_progress_streak)"
        if (( no_progress_streak >= 2 )); then
            log "stop: two consecutive iterations with no progress"
            break
        fi
    else
        no_progress_streak=0
        log "new commit: $(git log -1 --oneline)"
    fi
    prev_head="$new_head"
done

log "=== loop finished ==="
