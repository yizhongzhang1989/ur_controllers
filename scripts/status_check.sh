#!/usr/bin/env bash
# status_check.sh — Print a concise snapshot of repo state for the agent.
#
# Writes a human+machine readable summary to stdout. Intended to be redirected
# into docs/STATUS.md or a logfile before each agent iteration.

set -u

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

echo "# Repo status snapshot"
echo
echo "_Generated: $(date -u +%Y-%m-%dT%H:%M:%SZ) by scripts/status_check.sh._"
echo

echo "## Git"
echo
echo '```'
echo "branch: $(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo 'n/a')"
echo "head:   $(git rev-parse --short HEAD 2>/dev/null || echo 'n/a')"
echo
echo "recent commits:"
git log --oneline -n 10 2>/dev/null || echo "  (no commits yet)"
echo
echo "working tree:"
git status --short 2>/dev/null || echo "  (not a git repo)"
echo '```'
echo

echo "## Submodules"
echo
echo '```'
if [[ -f .gitmodules ]]; then
    git submodule status 2>/dev/null || echo "  (submodules not initialised)"
else
    echo "  (no submodules configured yet)"
fi
echo '```'
echo

echo "## Roadmap progress"
echo
if [[ -f docs/ROADMAP.md ]]; then
    total=$(grep -cE '^\s*-\s*\[[ x~!]\]' docs/ROADMAP.md || true)
    done_=$(grep -cE '^\s*-\s*\[x\]' docs/ROADMAP.md || true)
    echo "- completed: ${done_:-0} / ${total:-0}"
    echo
    echo "### Next unchecked items"
    echo
    grep -nE '^\s*-\s*\[[ ~!]\]' docs/ROADMAP.md | head -n 10 | sed 's/^/    /'
else
    echo "- docs/ROADMAP.md missing"
fi
echo

echo "## Tests"
echo
if [[ -x scripts/run_tests.sh ]]; then
    if scripts/run_tests.sh --dry-run >/dev/null 2>&1; then
        echo "- scripts/run_tests.sh: executable (dry-run ok)"
    else
        echo "- scripts/run_tests.sh: executable"
    fi
else
    echo "- scripts/run_tests.sh: missing or not executable"
fi
echo

echo "## Package inventory"
echo
echo '```'
echo "src/ packages:"
if compgen -G "src/*/package.xml" > /dev/null; then
    for p in src/*/package.xml; do
        echo "  - $(dirname "$p")"
    done
else
    echo "  (none)"
fi
echo
echo "third_party/ entries:"
if [[ -d third_party ]] && [[ -n "$(ls -A third_party 2>/dev/null)" ]]; then
    for d in third_party/*/; do
        echo "  - ${d%/}"
    done
else
    echo "  (empty)"
fi
echo '```'
