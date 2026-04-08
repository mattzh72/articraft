<modeling>
GEOMETRY
- Keep `build_object_model()` and `run_tests()` as top-level entry points.
- Import from `sdk`, not `sdk_hybrid`.
- Choose the SDK representation that best achieves a realistic result, then upgrade geometry when visible shape, hollowness, curvature, or clearances require it.
- When authoring mesh-backed visuals, use managed logical names like `mesh_from_geometry(..., "door_panel")`; do not reason about filesystem paths.
- Author visual geometry only; do not author collision geometry in `sdk`.
- Preserve correct joint origins, axes, limits, and articulation behavior.

LINK NAMING
- Give each link an extremely concise semantic name: ideally just the part name, plus a short intrinsic location or shape cue only when needed to distinguish similar parts.
- Keep every link name to a single underscore-joined string with at most 5 words.
- Do not encode articulation state in link names. Avoid state words such as `open`, `closed`, `extended`, `pulled_out`, `ajar`, `tilted`, or `rotated`.
- Prefer names that say what the part is and, when helpful, what shape it has.
- Use location words only when the object has a meaningful canonical orientation or another clear object-intrinsic reference frame.
- When similar parts are reliably distinguishable, prefer object-intrinsic spatial cues such as `front_handle` or `side_support`.
- Do not invent `left`, `right`, `front`, or `back` distinctions for symmetric or orientation-ambiguous objects. Some objects have only a partial intrinsic frame: a humanoid part can be `left_arm`, but the two doors of a symmetric cabinet usually should not be `left_door` and `right_door`.
- If only part of the intrinsic frame is meaningful, use only that part. For example, if `front` and `back` are meaningful but `left` and `right` are ambiguous, use `front_*` or `rear_*` when needed and do not force side labels.
- If repeated parts are semantically identical and not intrinsically distinguishable, reuse the same base name and add numeric suffixes such as `door_0`, `door_1`. For 2D repeated layouts, names like `key_0_0`, `key_0_1` are acceptable.

TESTING
- Use `sdk.TestContext`, return `ctx.report()`, and let `compile_model` own the baseline sanity/QC pass.
- Prefer `TestContext(object_model)`; do not pass asset roots in new code.
- Use `run_tests()` for prompt-specific exact checks, targeted pose checks, and explicit allowances only.
- See injected SDK docs for placement helpers, probe patterns, exact helper signatures, and current testing recommendations.
</modeling>
