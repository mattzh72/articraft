<modeling_charter>
- Work within the existing scaffolded editable code and keep top-level `build_object_model()` and `run_tests()`.
- Import from `sdk_hybrid`, not `sdk`, and keep articulations, tests, and URDF structure explicit.
- Prioritize believable silhouette, prompt-named visible features, and correct articulation over conservative QC cleanup.
- Identify the dominant silhouette and primary hero features early, then spend geometry budget where those features read clearly.
- Establish the proportion hierarchy before adding detail: identify the dominant envelope, the largest opening or face, the thickest and thinnest visible sections, and any recessed or nested visible layers.
- When the object implies a dominant opening, intake, cavity, bowl, screen, window, or wheel arch, let that opening occupy the necessary share of the body envelope instead of undersizing it to make the rest easier.
- Treat shell thickness, lip thickness, taper, and visible gap spacing as first-class modeling decisions, not cleanup values.
- If the real object reads as layered or nested, model that hierarchy explicitly so outer shells, recessed shells, cavities, and inner cores have clear visual separation.
- Bias toward more geometric richness in visible hero regions when that is what realism requires. For prompt-critical visible structure, complexity is a feature, not a bug.
- Make prompt-named visible features present and legible; do not let placeholder geometry flatten or hide them.
- Use realistic materials and restrained real-world colors when the prompt does not specify them.
- Author visuals only; do not author collision geometry in `sdk_hybrid`.
- Prefer simple primitives when they match the form cleanly, but use CadQuery or hybrid geometry when visible shells, cavities, curved tubes, or other silhouette-critical geometry need more shape freedom.
- Do not default silhouette-critical shells, ducts, nacelles, blades, petals, housings, or fairings to plain boxes/cylinders when the real object would read as a shaped shell or aero surface.
- If visible internal structure, hollowness, or cavity-bearing geometry is inherent to the object, model it explicitly even when the prompt does not specify it.
- For inlets, exhausts, vents, ducts, and similar openings, model the aperture and cavity explicitly. Do not cap a visible opening with a solid placeholder primitive.
- When a region seems too tall, too skinny, too shallow, too covered, or too small, interpret that as a silhouette/proportion problem first, not a detail problem.
- `section_loft(...)`, `repair_loft(...)`, and `partition_shell(...)` are unavailable in `sdk_hybrid`.
- For unfamiliar hybrid or CadQuery geometry, use `find_examples` before inventing a pattern from scratch.
- Preserve correct joint axes, origins, limits, and overall articulation behavior.
- For `REVOLUTE` and `PRISMATIC`, set `MotionLimits` with `effort`, `velocity`, `lower`, and `upper`. For `CONTINUOUS`, set `effort` and `velocity` only.
</modeling_charter>

<verification_contract>
- Use `sdk_hybrid.TestContext`, keep the scaffolded hard gates, and return `ctx.report()`.
- Use the injected SDK docs for exact helper signatures, tolerance caveats, CadQuery examples, and advanced testing patterns instead of re-deriving the APIs here.
- Broad `warn_if_*` checks are sensors, not proof. Use prompt-specific exact visual checks as the main regression tests.
- Prefer object-first tests: resolve `Part`, `Articulation`, and named `Visual` objects once, then pass those exact objects into `ctx.expect_*`, `ctx.allow_*`, and `ctx.pose(...)`.
- Prefer many small exact visual checks such as `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, and `expect_within(...)` over a few vague assertions.
- If the object has a mounted subassembly, write explicit `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, and `expect_within(...)` checks against the relevant local features.
- When proportion or nesting is visually central, add exact checks that prove dominance or hierarchy using gap, overlap, within, or contact relationships between the relevant visuals.
- Prefer tests that prove aperture presence, shell setback, and nested-layer ordering over vague global coverage assumptions.
- The model is not done until every applicable visual coverage category is proved: hero features are present, mounted parts are connected/seated, important parts are in the right place, key poses are believable, and each new visible form or mechanism has a matching assertion.
- Test both rest pose and meaningful operating poses. For continuous joints, use explicit sampled angles rather than fake lower/upper limits.
- Use `ctx.warn_if_articulation_overlaps(...)` only when joint clearance is genuinely uncertain or mechanically important.
- If a warning-tier heuristic fires, investigate it with `probe_model` before editing geometry or relaxing thresholds.
- If support/floating is ambiguous, use `probe_model` helpers first, then encode the real invariant in `run_tests()`.
- Deprecated as default scaffold heuristics in new generated code: `warn_if_articulation_origin_near_geometry(...)`, `warn_if_overlaps(...)`, and blanket `warn_if_articulation_overlaps(...)`. Add them only with specific justification.
- Make attachment checks primary evidence of realism. Use narrow overlap allowances only for justified nested or conservative cases, and do not introduce visible air gaps just to silence broad QC.
</verification_contract>
