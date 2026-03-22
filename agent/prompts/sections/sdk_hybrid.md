<harness_requirements>
- Treat the editable code as initially empty.
- The editable section must define top-level `build_object_model()` and `run_tests()`.
- Import from `sdk_hybrid`, not `sdk`.
- `sdk_hybrid` is its own authoring surface for hybrid runs.
- Author visuals only; do not author collision geometry in `sdk_hybrid`.
- `section_loft(...)`, `repair_loft(...)`, and `partition_shell(...)` are unavailable in `sdk_hybrid`.
- Not every model needs CadQuery. Prefer primitives when they are sufficient, and reach for CadQuery when you need shape freedom the base SDK does not provide cleanly.
- When you are about to build unfamiliar CadQuery geometry, call `find_examples` first so you can adapt a concrete repository example before editing.
- `find_examples` is lexical, not semantic. Search with short concrete terms like object names, feature names, geometry operations, and CadQuery API names.
- Keep articulations, tests, and URDF structure explicit in `sdk_hybrid`.
</harness_requirements>

<modeling_priority>
- Prioritize realistic visible geometry and believable motion over perfectly conservative collision cleanliness.
- Prefer visually rich, realistic, and well-composed designs over simplistic, generic, or lazy assemblies.
- Make the object read clearly as the requested item from silhouette, proportion, and visible substructure, not just from passing tests.
- Identify all prompt-named visible features and the dominant silhouette drivers.
- Treat the dominant silhouette and the most defining 3-6 of those features as the primary hero features.
- Make every prompt-named visible feature present and legible, then spend the most geometric complexity on the dominant exterior silhouette and the primary hero features.
- Do not let placeholder geometry hide or flatten the features that most define the object.
- Do not default silhouette-critical visible forms such as body shells, fenders, nacelles, ducts, housings, exterior panels, wheels, rims, or other dominant outer surfaces to plain boxes/cylinders unless the real object is genuinely that simple.
- If the real object should be hollow, thin-walled, or cavity-bearing, model that cavity explicitly instead of leaving the part as a solid block.
- Do not omit important internal structures when they are visible, mechanically necessary, cavity-defining, or essential to the object reading correctly.
- Plain primitives are acceptable for hidden structure, brackets, shafts, spacers, and other secondary support geometry when they do not control the object's visual identity.
- Give the model a plausible real-world material and color palette, not just geometry.
- Use color to clarify part function and material changes, but keep the palette coherent and believable.
- Preserve correct joint axes, limits, origins, and overall articulation behavior.
- For `ArticulationType.REVOLUTE` and `ArticulationType.PRISMATIC`, always set `motion_limits=MotionLimits(effort=..., velocity=..., lower=..., upper=...)`.
- For `ArticulationType.CONTINUOUS`, always set `motion_limits=MotionLimits(effort=..., velocity=...)` and never set `lower` or `upper`.
- Do not omit `motion_limits` for continuous joints; export and strict validation require explicit `effort` and `velocity`.
- If forced to trade off, keep the realistic-looking visual model and fix geometry, joints, and tests thoughtfully before reducing visible fidelity.
- For curved tubular members such as hooks, handles, loops, rails, guards, whisk wires, cages, and bent support frames, do not default to chained primitive cylinders.
- If the real part should read as one continuously bent member, prefer spline- or path-based tube geometry such as `tube_from_spline_points(...)`, `sweep_profile_along_spline(...)`, or `WirePath`.
- Use `wire_from_points(...)` when the intended shape is explicitly piecewise-linear with visible elbows or hard corners.
- Do not expect a sparse polyline with small fillets to read as a truly smooth bent hook or handle.
- Treat this as a silhouette decision: choose the representation that best matches the perceived physical form, even if a primitive assembly would compile more easily.
</modeling_priority>

<qc_and_overlap_handling>
- The harness runs collision-based overlap QC on generated collision geometry and will surface overlaps as a non-blocking warning by default.
  - SDK compile generates collisions from visuals automatically.
  - Overlap QC is deliberately broad and can be noisy because it relies on generated collisions, AABB reasoning, and convex decomposition.
  - Thin wires, thin blades, concave shells, nested assemblies, and other awkward geometry can trip conservative false positives.
  - Treat overlap QC as a conservative backstop, not as the primary proof that parts look attached.
  - Slight intended interpenetration can be acceptable when it makes a mounted or nested assembly look seated instead of floating.
  - Prefer flush or nearly flush attachment when parts are meant to look mounted, and avoid visible air gaps unless the real object clearly has clearance.
  - Fix clearly bad overlaps by adjusting joint origins, limits, or visual geometry placement.
  - If a small overlap is intentional or likely conservative, explicitly allow only the specific pair(s) in `run_tests()` via `ctx.allow_overlap(...)`.
  - On models with non-fixed joints, use `ctx.check_articulation_overlaps(...)` as the primary failure-tier clearance gate for articulated parent/child pairs.
  - Still run `ctx.warn_if_overlaps(...)` so broader non-joint overlap findings and allowances are tracked.
</qc_and_overlap_handling>

<run_tests_requirements>
- Use `sdk_hybrid.TestContext` and return `ctx.report()`.
- Use `ctx.warn_if_articulation_origin_near_geometry(tol=0.015)` as the default articulation-origin sensor.
- The harness truncates articulation-origin tolerances to 3 decimals and caps them at `0.15`.
- Use `ctx.warn_if_part_geometry_disconnected(use="visual")` as the default disconnected-geometry sensor.
- On models with `REVOLUTE`, `PRISMATIC`, or `CONTINUOUS` articulations, use `ctx.check_articulation_overlaps(max_pose_samples=...)` as the default joint-clearance check.
- Use `ctx.warn_if_coplanar_surfaces(use="visual", ignore_adjacent=True, ignore_fixed=True)` only when it is likely to add value for the current object.
- Use `ctx.warn_if_overlaps(max_pose_samples=..., ignore_adjacent=True, ignore_fixed=True)` as the broad warning-tier overlap sensor.
- `warn_if_articulation_origin_near_geometry(...)` is not scale-aware; its tolerance is a fixed metric distance in meters.
- A tolerance that is sensible for a compact object may be meaningless for a vehicle-sized or aircraft-sized assembly.
- Only loosen articulation-origin tolerance when the geometry genuinely needs it, and keep it tight.
- `warn_if_coplanar_surfaces(...)` is a deliberately noisy visual heuristic based on element AABB faces; flush mounts, bezels, grilles, and panel seams can trigger it even when the model is acceptable.
- Prefer relation-aware defaults for coplanar checks. Adjacent/fixed panel mounts are usually low-confidence hints, not proof that geometry is wrong.
- Use `ctx.allow_coplanar_surfaces("a", "b", reason="...")` narrowly for intentional flush mounts or panel seams that the heuristic still reports.
- Treat these `warn_if_*` sensors as deliberately dumb static heuristics. They may or may not be useful for the current object, and they must not carry the burden of proving realism.
- Include `ctx.warn_if_overlaps(...)` on every model as a broad warning-tier sensor unless you have a very specific reason not to.
- Thoroughly test intended geometry, layout, and kinematic behavior with meaningful checks.
- Add prompt-specific semantic regression checks with `expect_*` assertions for the important visible and mechanical claims.
</run_tests_requirements>

<testing_principles>
- Your tests are not a formality. They are the primary mechanism to prevent visually wrong URDFs from being accepted.

- Prefer many small checks over a few broad ones. Each check encodes a single invariant.
- Test both rest pose and key mechanism poses.
- For `ArticulationType.CONTINUOUS`, use explicit sampled angles such as `0`, `pi / 2`, and `pi` instead of lower/upper limit poses.
- Prefer AABB-based intent checks (`expect_aabb_*`) for placement assertions; link origins are often misleading.
- Primitive geometry constructors (`Box`, `Cylinder`, `Sphere`) only take shape parameters. Put transforms on `visual(..., origin=...)` or `Inertial.from_geometry(..., origin=...)`.
- Treat tests as support for realism and motion, not as a reason to degrade the visible model into primitive geometry.
- Treat `warn_if_articulation_origin_near_geometry(...)` and `warn_if_part_geometry_disconnected(...)` as deliberately dumb static sensors.
- Treat `check_articulation_overlaps(...)` differently: it is a targeted failure-tier clearance check for non-fixed articulated parent/child pairs.
- Treat `warn_if_coplanar_surfaces(...)` the same way: it is a deliberately noisy flush-surface sensor, not semantic proof.
- A coplanar warning alone is not a reason to add visible air gaps or distort a legitimate mounted panel.
- Treat `warn_if_overlaps(...)` the same way: it is a deliberately broad collision/QC sensor, not semantic proof.
- They can surface suspicious composition, but they are not semantic regression tests and may be misleading for perfectly acceptable geometry.
- Add prompt-specific checks for the primary hero features, the prompt-named visible features, and the dominant visible forms so the most important visual commitments are enforced, not just generic structural sanity.
- When hollowness, cavities, vents, liners, baffles, ribs, or other internal structures are important to the object, add checks that prove those claims instead of leaving them implicit.
- When you add a new visible form or mechanism, add or refine tests that prove that new claim before moving on. Do not let geometry complexity outpace test coverage.
- Prevent floating parts:
  - Use `warn_if_part_geometry_disconnected(use="visual")` when it is a helpful sensor for disconnected subassemblies inside one part.
  - Make attachment checks primary. Use explicit `expect_aabb_*` checks to show that mounted parts sit where they should.
  - Add explicit attachment checks such as `expect_aabb_gap(parent, child, axis="z", ...)` for joints that should touch or insert.
  - Preferred signatures:
    - `expect_aabb_gap(..., axis="z", max_gap=..., max_penetration=...)`
    - `expect_aabb_contact(...)`
    - `expect_joint_motion_axis(..., world_axis="x"|"y"|"z", direction="positive"|"negative")`
  - Pair gap checks with footprint or containment checks when helpful, and add pose-specific checks at important limits.
  - Use `expect_joint_motion_axis(...)` only when the moving link's AABB center should actually translate along that world axis.
    - Good fits: lids, doors, sliders, pedals, levers.
    - Do not use it for centered continuous rotors, fans, propellers, wheels, or knobs spinning in place around their own centerline.
    - For those, use pose-specific checks at multiple angles instead.
  - For rigid supports and mounted subassemblies, default to near-zero gap checks.
    - Use `expect_aabb_gap(..., axis="z", max_gap=0.001, max_penetration=0.0)` unless the real object visibly has more clearance.
- Allow overlaps only when justified:
  - Use `ctx.allow_overlap("a", "b", reason="...")` narrowly for legitimate nested mechanisms or conservative false-positives.
  - On articulated models, use `ctx.check_articulation_overlaps(...)` unless the overlap is genuinely intentional and justified.
  - Do not introduce visible air gaps just to satisfy conservative overlap QC.
  - Still call `ctx.warn_if_overlaps(...)` so the allowance is validated and tracked.
- Add selective separation checks only for pairs that truly must remain clear across motion.
</testing_principles>

<recommended_test_structure>
1) Sanity: `check_model_valid`, `check_mesh_files_exist`
2) Joint clearance: on models with non-fixed articulations, include `check_articulation_overlaps(max_pose_samples=...)`
3) Broad warning sensors: include `warn_if_overlaps`, and include `warn_if_coplanar_surfaces` / `warn_if_articulation_origin_near_geometry` / `warn_if_part_geometry_disconnected` when they are useful
4) Geometry warning backstop: `warn_if_overlaps(max_pose_samples=..., ignore_adjacent=True, ignore_fixed=True)`
5) Intent: multiple prompt-specific `expect_*` semantic regression checks, with attachment checks as primary evidence of realism
6) Pose coverage: for each important joint, include checks at lower/upper limits using `with ctx.pose({"joint_name": value}): ...` or `with ctx.pose(joint_name=value): ...`; for continuous joints, use explicit sampled angles instead of lower/upper limits
</recommended_test_structure>

<intent_checks>
- Use `expect_joint_motion_axis(...)` only when the moving link's AABB center should actually translate along that world axis.
- Do not use it for centered continuous rotors, wheels, knobs, propellers, or fans spinning in place around their own centerline.
- For in-place rotation, use pose-specific containment, overlap, and clearance checks at multiple angles instead.
</intent_checks>

<api_gotchas>
- `ctx.pose(...)` takes either a mapping or keyword args:
  - `with ctx.pose({"joint_name": value}): ...`
  - `with ctx.pose(joint_name=value): ...`
- Do not pass positional `(joint_name, value)` arguments to `ctx.pose(...)`.
</api_gotchas>
