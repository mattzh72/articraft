from __future__ import annotations

from textwrap import dedent


def build_probe_model_description() -> str:
    return dedent(
        """
        Execute a short Python snippet against the current bound `model.py` for inspection-only geometry diagnosis.

        This tool is intended for inspection-only use and is bound to one target file by the harness.
        Do not pass file paths. Use it when placements, clearances, containment, overlap risk,
        or pose behavior are uncertain.

        Execution contract:
        - Write ordinary Python code.
        - Use this tool only for non-mutating inspection and measurement.
        - `emit(value)` must be called exactly once.
        - `value` must be JSON-serializable.
        - `print(...)` is allowed for debugging only. It is captured separately and returned only when requested.
        - Do not write files, modify `object_model`, launch subprocesses, access the network, or perform destructive actions.
        - The snippet runs in an isolated subprocess with a timeout.

        Guidance:
        - The injected SDK docs include the exact probe helper catalog and signatures.
        - IMPORTANT: Do not import probe helpers from `sdk` or other modules inside the snippet. Helpers such as `part(...)`, `joint(...)`, `visual(...)`, `pose(...)`, `emit(...)`, `pair_report(...)`, and related review helpers are already injected into the snippet namespace.
        - The helper surface includes lookup, measurement, relationship, and review helpers such as `part(...)`, `visual(...)`, `pose(...)`, `summary(...)`, `pair_report(...)`, and `find_clearance_risks(...)`.
        - Except for the explicit `aabb(...)` helper, probe measurements should prefer exact geometry over transformed AABB approximations.
        - These helpers are tool-local review conveniences built on the current `object_model` and `TestContext`.
        - They are not part of the public SDK API and may use internal exact-geometry helpers.
        """
    ).strip()


PROBE_MODEL_DESCRIPTION = build_probe_model_description()
