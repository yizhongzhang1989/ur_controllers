# Decisions

Append-only log of architectural decisions. Newer entries at the bottom.
Format: ADR-lite. Do not delete past entries; supersede with a new one.

---

## ADR-0001 — Repository layout and automation contract

- **Date:** 2026-04-22
- **Status:** Accepted
- **Context:** Project is driven by an automated coding agent iterating via a
  bash loop. Clear, stable structure is required so the agent can locate
  sources, tests, and state files deterministically.
- **Decision:**
  - `third_party/` holds git submodules, read-only to the agent.
  - `src/` holds our own controller packages.
  - `bringup/`, `evaluation/`, `tests/` are top-level and ours.
  - `docs/ROADMAP.md` is the goal source of truth (human-edited).
  - `docs/STATUS.md` is the live state (agent-overwritten).
  - `docs/DECISIONS.md` is append-only.
  - `AGENTS.md` defines the iteration contract.
- **Consequences:** Any agent or human reading these five files in order
  (AGENTS → ROADMAP → STATUS → DECISIONS → git log) has enough context to
  produce the next commit.

---

## ADR-0002 — Placeholder: ROS 2 distribution

- **Date:** TBD
- **Status:** Proposed
- **Context:** ros2_control APIs and package availability differ between
  distros. Must be pinned before M1.
- **Decision:** TBD (candidates: Humble LTS, Jazzy LTS).
- **Consequences:** CI image, rosdep keys, and driver package versions all
  follow from this choice.
---

## ADR-0003 — Submodule ownership split

- **Date:** 2026-04-22
- **Status:** Accepted
- **Context:** Three projects live under `third_party/`. Two are external
  (`crisp_controllers`, `cartesian_controllers`) and must stay pristine so
  they can be re-pulled from upstream without local drift. One
  (`ur_simulator`) is our own simulator, still under active development, and
  needs to be editable from within this workbench.
- **Decision:**
  - `crisp_controllers` and `cartesian_controllers` are **read-only**. The
    agent only ever updates their submodule pointer, and only on explicit
    human request.
  - `ur_simulator` development happens on a dedicated `auto_dev` branch of
    the submodule (created from `main`, pushed to origin). The branch is
    tracked via `branch = auto_dev` in `.gitmodules`. All edits inside the
    submodule must target this branch; commits happen inside
    `third_party/ur_simulator` first, then the parent repo bumps the
    pointer in a follow-up commit.
- **Consequences:** `git submodule update --remote third_party/ur_simulator`
  fast-forwards to the latest `auto_dev`. Upstream `main` of ur_simulator is
  never directly touched by the agent; merges from `auto_dev` → `main` are
  human-gated via pull request.
