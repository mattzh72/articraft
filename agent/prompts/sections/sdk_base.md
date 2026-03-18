# Harness requirements
- Treat the editable code as initially empty.
- The editable section must define top-level `build_object_model()` and `run_tests()`.
- Modeling priority:
  - Prioritize realistic visible geometry and believable motion over perfectly conservative collision cleanliness.
  - Prefer visually rich, realistic, and well-composed designs over simplistic, generic, or lazy assemblies.
  - Make the object read clearly as the requested item from silhouette, proportion, and visible substructure, not just from passing tests.
  - Give the model a plausible real-world material and color palette, not just geometry.
  - Define and apply named materials when the object would realistically have visible finish differences (for example painted metal, brushed steel, black plastic, rubber, glass).
  - When the prompt does not specify colors, infer realistic default colors from the object category and use restrained, believable tones rather than arbitrary bright colors.
  - Use color to clarify part function and material changes, but keep the palette coherent and close to what a manufactured real object would use.
  - For transparent or translucent parts, use appropriate alpha values instead of making everything fully opaque.
  - Author visuals only; do not author collision geometry in `sdk`.
  - Use `part.visual(...)` with explicit origins/materials, and set inertia separately when needed.
  - Prefer detailed visual meshes/profiles for silhouette-critical parts.
  - Do not simplify visuals down to boxes/cylinders just to silence conservative overlap warnings.
  - Preserve correct joint axes, limits, origins, and overall articulation behavior.
  - For `ArticulationType.REVOLUTE` and `ArticulationType.PRISMATIC`, always set `motion_limits=MotionLimits(effort=..., velocity=..., lower=..., upper=...)`.
  - For `ArticulationType.CONTINUOUS`, always set `motion_limits=MotionLimits(effort=..., velocity=...)` and never set `lower` or `upper`.
  - Do not omit `motion_limits` for continuous joints; export and strict validation require explicit `effort` and `velocity`.
  - If forced to trade off, keep the realistic-looking visual model and fix geometry/joints/tests before reducing visible fidelity.
  - For curved tubular members such as hooks, handles, loops, rails, guards, whisk wires, cages, and bent support frames, do not default to chained primitive cylinders.
  - If the real part should read as one continuously bent member, prefer spline- or path-based tube geometry such as `tube_from_spline_points(...)`, `sweep_profile_along_spline(...)`, or `WirePath`.
  - Use `wire_from_points(...)` when the intended shape is explicitly piecewise-linear with visible elbows or hard corners.
  - Do not expect a sparse polyline with small fillets to read as a truly smooth bent hook or handle.
  - Treat this as a silhouette decision: choose the representation that best matches the perceived physical form, even if a primitive assembly would compile more easily.
- The harness runs collision-based overlap QC on generated collision geometry and will surface overlaps as a non-blocking warning by default.
  - SDK compile generates collisions from visuals automatically.
  - Treat overlap QC as a conservative backstop, not as the primary proof that parts look attached.
  - Slight intended interpenetration can be acceptable when it makes a mounted or nested assembly look seated instead of floating.
  - Prefer flush or nearly flush attachment when parts are meant to look mounted, and avoid visible air gaps unless the real object clearly has clearance.
  - Fix clearly bad overlaps by adjusting visual geometry, joint origins, and limits.
  - If a small overlap is intentional or likely conservative, explicitly allow only the specific pair(s) in `run_tests()` via `ctx.allow_overlap(...)` and still run `ctx.check_no_overlaps(...)` so the allowance is tracked.
- `run_tests()` requirements (strict):
  - Use `sdk.TestContext` and return `ctx.report()`.
  - Include `ctx.check_articulation_origin_near_geometry(tol=0.01)`.
  - Only loosen articulation-origin tolerance when the geometry genuinely needs it, and keep it tight.
  - Treat articulation-origin proximity as a coarse sanity check only, not proof that mounting faces are flush.
  - Include `ctx.check_part_geometry_connected(use="visual")`.
  - Thoroughly test intended geometry/layout/kinematic behavior with meaningful checks.

# Testing (CRITICAL)

Your tests are not a formality. They are the primary mechanism to prevent visually wrong URDFs from being accepted.

## Principles
- Prefer **many small checks** over a few broad ones. Each check encodes a single invariant.
- Test both **rest pose** and **key mechanism poses** (limits and typical operating position).
- For `ArticulationType.CONTINUOUS`, use explicit sampled angles such as `0`, `pi / 2`, and `pi` instead of lower/upper limit poses.
- Prefer **AABB-based intent checks** (`expect_aabb_*`) for placement assertions; link origins are often misleading.
- Primitive geometry constructors (`Box`, `Cylinder`, `Sphere`) only take shape parameters. Put transforms on `visual(..., origin=...)` or `Inertial.from_geometry(..., origin=...)`.
- Treat tests as support for realism and motion, not as a reason to degrade the visible model into primitive geometry.
- Prevent floating parts:
  - Run `check_part_geometry_connected(use="visual")` so disconnected subassemblies inside one part do not slip through.
  - Make attachment checks primary. Use explicit `expect_aabb_*` checks to show that mounted parts sit where they should.
  - Add explicit “attachment” checks such as `expect_aabb_gap(parent, child, axis="z", ...)` for joints that should touch/insert.
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
    - This especially applies to axle supports, brackets, hinge plates, legs, and body-mounted frames.
- Allow overlaps only when justified:
  - Use `ctx.allow_overlap("a", "b", reason="...")` narrowly for legitimate nested mechanisms or conservative false-positives.
  - Do not introduce visible air gaps just to satisfy conservative overlap QC.
  - Still call `ctx.check_no_overlaps(...)` so the allowance is validated and tracked.
- Add selective separation checks only for pairs that truly must remain clear across motion.
- Repair order when QC pushes against realism:
  - first fix visual geometry or joint placement
  - then add a narrow allowance for acceptable edge cases
  - only as a last resort simplify the visible mesh

## Recommended test structure (pattern)
1) Sanity: `check_model_valid`, `check_mesh_files_exist`, `check_articulation_origin_near_geometry`, `check_part_geometry_connected`
2) Geometry backstop: `check_no_overlaps(max_pose_samples=..., ignore_adjacent=True, ignore_fixed=True)` (+ explicit allowances when needed)
3) Intent: multiple `expect_*` checks, with attachment checks as primary evidence of realism
4) Pose coverage: for each important joint, include checks at lower/upper limits using `with ctx.pose({"joint_name": value}): ...` or `with ctx.pose(joint_name=value): ...`; for continuous joints, use explicit sampled angles instead of lower/upper limits
