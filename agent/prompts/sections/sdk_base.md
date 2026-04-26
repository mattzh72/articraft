<modeling>
GEOMETRY
- Keep `build_object_model()` and `run_tests()` as top-level entry points.
- Import public authoring APIs directly from `sdk`.
- Do not guess Python submodules from docs topic names. For example, use `from sdk import place_on_face`, not `from sdk.placement import place_on_face`.
- Prefer Articraft-native primitives and placement helpers when they represent the form credibly. This is more simple to use than pure CadQuery.
- Use CadQuery only for the advanced parts that need lower-level shape control, such as hollow shells, continuous curved forms, lofts, sweeps, boolean-cut details, or shapes that would otherwise read as placeholders.
- Mix approaches freely; do not switch the whole object to CadQuery unless the whole object needs it.
- Match the visible construction logic of the object. If a face should read as one continuous manufactured piece, keep it as a connected face with openings or cutouts instead of rebuilding it from separate floating members. Use separate member-based construction only when the visible form should genuinely read as discrete members.
- When authoring mesh-backed visuals, use managed logical names like `mesh_from_geometry(..., "door_panel")` or `mesh_from_cadquery(..., "door_panel")`; do not reason about filesystem paths.
- Author visual geometry only; do not author collision geometry in `sdk`.
- Preserve correct joint origins, axes, limits, and articulation behavior.

TESTING
- Use `sdk.TestContext`, return `ctx.report()`, and let `compile_model` own the baseline sanity/QC pass.
- Prefer `TestContext(object_model)`; do not pass asset roots in new code.
- Use `run_tests()` for prompt-specific exact checks, targeted pose checks, and explicit allowances only.
- Treat overlap findings as classification tasks first: decide whether the reported intersection is intentional design embedding that should be covered by a scoped `ctx.allow_overlap(...)`, or an unintended collision that needs geometry, mount, or pose changes. Accepted intentional cases include proxy nesting, captured pins or shafts, seated trim, and compliant compression.
- Pair every `ctx.allow_overlap(...)` with at least one exact proof check such as `expect_within(...)`, `expect_overlap(...)`, `expect_gap(..., max_penetration=...)`, `expect_contact(...)`, or a decisive pose check.
</modeling>
