<modeling>
GEOMETRY
- Import from `sdk_hybrid`, not `sdk`. Keep `build_object_model()` and `run_tests()` as top-level entry points.
- Choose the SDK or CadQuery representation that best achieves a realistic result. Use simple primitives or minimal sketches/extrusions to lock envelope, proportions, supports, and attachment layout; switch to hybrid/CadQuery geometry as soon as visible shells, cavities, curves, transitions, or local clearances need more shape freedom.
- When authoring mesh-backed visuals, use managed logical names like `mesh_from_cadquery(..., "door_panel")`; do not reason about filesystem paths.
- Model openings, cavities, and hollow structure explicitly. Never cap a visible opening with a solid placeholder.
- If the object is layered or nested, model each layer with clear visual separation.
- Hidden supports and internal structure can use simple primitives.
- Within one part, avoid disconnected visual islands. Small separate-looking features should be fused into the main body or connected by real ribs, brackets, pins, collars, stems, or wall thickness rather than suspended inside the part.
- For mounted child parts whose position is defined by a parent surface, prefer placement helpers over hand-tuned `Origin(...)` coordinates.
- Use `place_on_surface(...)` by default for rigid mounts onto housings, shells, panels, pedals, feet, knobs, buttons, pads, brackets, and similar surface-mounted parts.
- Use `place_on_face(...)` or `place_on_face_uv(...)` only when the parent is truly box-like and the semantic reference is a specific face.
- Use `proud_for_flush_mount(...)` when a centered child should sit flush instead of half-embedded.
- Use realistic materials and restrained real-world colors.
- Author visual geometry only; do not author collision geometry in `sdk_hybrid`.
- `section_loft(...)`, `repair_loft(...)`, and `partition_shell(...)` are unavailable in `sdk_hybrid`.
- Preserve correct joint axes, origins, limits, and articulation behavior.
- Joint axis sign matters: `origin` defines the joint frame relative to the
  parent, `axis` is written in that frame, positive revolute motion follows the
  right-hand rule, and positive prismatic motion translates along `+axis`.
  Negate `axis` when increasing joint position should open, raise, or extend
  outward but the current choice moves into the parent instead.
- For `REVOLUTE`/`PRISMATIC`: set `MotionLimits` with effort, velocity, lower, upper. For `CONTINUOUS`: effort and velocity only.

TESTING
- Use `sdk_hybrid.TestContext`, return `ctx.report()`, and let `compile_model` own the baseline sanity/QC pass.
- `compile_model` automatically checks model validity, exactly one root part, mesh assets, floating disconnected part groups, disconnected geometry islands inside a part, and current-pose real 3D overlaps.
- Prefer `TestContext(object_model)`; do not pass asset roots in new code.
- Use the object-first pattern: resolve Part, Articulation, and named Visual objects once, then pass them to `ctx.expect_*` and `ctx.allow_*` helpers. See SDK docs for exact signatures.
- Use `run_tests()` for prompt-specific exact checks, targeted pose checks, and explicit allowances only.
- Once `run_tests()` references a visual by exact `elem_*` name, treat that name as a contract. Preserve it or update every dependent check in the same edit.
- `fail_if_parts_overlap_*` and `ctx.allow_overlap(...)` mean real 3D interpenetration; `expect_overlap(...)` is a separate projected footprint check. If parts are nested but should remain clear, use `expect_within(...)`, `expect_gap(...)`, `expect_contact(...)`, and/or `expect_overlap(...)` instead of `allow_overlap(...)`.
- Every part must be tested for: presence, connection to neighbors, correct placement, and no unsupported disconnected sub-geometry when the visuals make that ambiguous.
- If a part has multiple visual regions, prefer exact contact/support checks for the critical mounts instead of broad heuristics.
- Delay brittle element-scoped or numeric constraints until the representation is stable. Prefer prompt-critical part-level exact checks first.
- Do not add blanket lower/upper pose sweeps by default. Keep pose-specific checks narrow and only add them when a prompt-critical articulation remains ambiguous after exact checks.
- If an automated compile warning suggests floating-looking or disconnected geometry, investigate with `probe_model` before changing geometry or relaxing assumptions.
- When support or floating status is ambiguous, probe first, then encode the real invariant in tests.
- See SDK docs for deprecated helpers and current recommendations.
</modeling>
