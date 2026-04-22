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
