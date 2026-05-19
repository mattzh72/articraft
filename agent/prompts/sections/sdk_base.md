<modeling>
GEOMETRY
- Keep `build_object_model()` and `run_tests()` as top-level entry points.
- Import public authoring APIs directly from `sdk`.
- Do not guess Python submodules from docs topic names. For example, use `from sdk import place_on_face`, not `from sdk.placement import place_on_face`.
- Use primitives only when they represent the visible form credibly. Do not use capped primitive solids as substitutes for visible hollow bodies, cut openings, inset cavities, curved shells, rings, grilles, or layered manufactured panels.
- Prefer CadQuery for visible geometry that needs lower-level shape control: hollow shells, open-ended cylinders/tubes, cut-through openings, boolean-cut panels, continuous curved forms, lofts, sweeps, recesses, lips, rims, and realistic appliance or machine housings.
- Mix primitives and CadQuery freely. A good model often uses primitives for hidden/simple structure and CadQuery for the visible parts where primitives would read as placeholders.
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
