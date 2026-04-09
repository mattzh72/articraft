<modeling>
GEOMETRY
- Import from `sdk_hybrid`, not `sdk`. Keep `build_object_model()` and `run_tests()` as top-level entry points.
- Choose the SDK or CadQuery representation that best achieves a realistic result, then upgrade geometry when shells, cavities, curves, transitions, or local clearances need more shape freedom.
- Match the visible construction logic of the object. If a face should read as one continuous manufactured piece, keep it as a connected face with openings or cutouts instead of rebuilding it from separate floating members. Use separate member-based construction only when the visible form should genuinely read as discrete members.
- When authoring mesh-backed visuals, use managed logical names like `mesh_from_cadquery(..., "door_panel")`; do not reason about filesystem paths.
- Author visual geometry only; do not author collision geometry in `sdk_hybrid`.
- `section_loft(...)`, `repair_loft(...)`, and `partition_shell(...)` are unavailable in `sdk_hybrid`.
- Preserve correct joint origins, axes, limits, and articulation behavior.
- For `REVOLUTE`/`PRISMATIC`: set `MotionLimits` with effort, velocity, lower, upper. For `CONTINUOUS`: effort and velocity only.

TESTING
- Use `sdk_hybrid.TestContext`, return `ctx.report()`, and let `compile_model` own the baseline sanity/QC pass.
- Prefer `TestContext(object_model)`; do not pass asset roots in new code.
- Use `run_tests()` for prompt-specific exact checks, targeted pose checks, and explicit allowances only.
- See injected SDK docs for placement helpers, probe patterns, exact helper signatures, and current testing recommendations.
</modeling>
