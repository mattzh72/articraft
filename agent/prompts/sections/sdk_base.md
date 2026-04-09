<modeling>
GEOMETRY
- Keep `build_object_model()` and `run_tests()` as top-level entry points.
- Import from `sdk`, not `sdk_hybrid`.
- Choose the SDK representation that best achieves a realistic result, then upgrade geometry when visible shape, hollowness, curvature, or clearances require it.
- Match the visible construction logic of the object. If a face should read as one continuous manufactured piece, keep it as a connected face with openings or cutouts instead of rebuilding it from separate floating members. Use separate member-based construction only when the visible form should genuinely read as discrete members.
- When authoring mesh-backed visuals, use managed logical names like `mesh_from_geometry(..., "door_panel")`; do not reason about filesystem paths.
- Author visual geometry only; do not author collision geometry in `sdk`.
- Preserve correct joint origins, axes, limits, and articulation behavior.

TESTING
- Use `sdk.TestContext`, return `ctx.report()`, and let `compile_model` own the baseline sanity/QC pass.
- Prefer `TestContext(object_model)`; do not pass asset roots in new code.
- Use `run_tests()` for prompt-specific exact checks, targeted pose checks, and explicit allowances only.
- See injected SDK docs for placement helpers, probe patterns, exact helper signatures, and current testing recommendations.
</modeling>
