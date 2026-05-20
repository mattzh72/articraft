from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    Material,
    mesh_from_cadquery,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="playground_seesaw")

    # --- Fulcrum (fixed root part) ---
    fulcrum = model.part("fulcrum")

    # Triangular fulcrum geometry (CadQuery)
    def _fulcrum_geom():
        import cadquery as cq
        # Triangle in XZ plane: base 0.6m (x from -0.3 to 0.3), height 1.0m (z from 0 to 1.0)
        # Extrude 0.3m along Y (depth, matches plank width)
        geom = (
            cq.Workplane("XZ")
            .polyline([(-0.3, 0.0), (0.3, 0.0), (0.0, 1.0), (-0.3, 0.0)])
            .close()
            .extrude(0.3)
        )
        return geom

    fulcrum_mesh = mesh_from_cadquery(_fulcrum_geom(), "fulcrum_body")
    fulcrum.visual(
        fulcrum_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="fulcrum_body",
        material=Material(name="fulcrum_material", color=(0.4, 0.4, 0.4))  # Dark gray
    )

    # --- Plank (moving part) ---
    plank = model.part("plank")

    # Main plank body: 3m long, 0.3m wide, 0.05m thick, bright yellow with chamfered edges
    def _plank_geom():
        import cadquery as cq
        # Create box and chamfer long edges (along X-axis) by 5mm
        geom = cq.Workplane("XY").box(3.0, 0.3, 0.05).edges("|X").chamfer(0.005)
        return geom

    plank_mesh = mesh_from_cadquery(_plank_geom(), "plank_body")
    plank.visual(
        plank_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="plank_body",
        material=Material(name="plank_material", color=(1.0, 1.0, 0.0))  # Bright yellow
    )

    # Left seat (red, on left end)
    plank.visual(
        Box((0.4, 0.3, 0.05)),
        origin=Origin(xyz=(-1.3, 0.0, 0.05)),  # Centered 0.2m from left end, on top of plank
        name="left_seat",
        material=Material(name="seat_material", color=(1.0, 0.0, 0.0))  # Red
    )

    # Right seat (red, on right end)
    plank.visual(
        Box((0.4, 0.3, 0.05)),
        origin=Origin(xyz=(1.3, 0.0, 0.05)),
        name="right_seat",
        material=Material(name="seat_material", color=(1.0, 0.0, 0.0))
    )

    # Left handle (blue, vertical cylinder next to left seat)
    plank.visual(
        Cylinder(radius=0.02, length=0.4),
        origin=Origin(xyz=(-1.3, 0.0, 0.275)),  # Top of seat + half handle height
        name="left_handle",
        material=Material(name="handle_material", color=(0.0, 0.0, 1.0))  # Blue
    )

    # Right handle (blue, vertical cylinder next to right seat)
    plank.visual(
        Cylinder(radius=0.02, length=0.4),
        origin=Origin(xyz=(1.3, 0.0, 0.275)),
        name="right_handle",
        material=Material(name="handle_material", color=(0.0, 0.0, 1.0))
    )

    # Left rubber bumper (black, under left plank end)
    plank.visual(
        Cylinder(radius=0.05, length=0.05),
        origin=Origin(xyz=(-1.5, 0.0, -0.05)),  # Under left end, below plank
        name="left_bumper",
        material=Material(name="bumper_material", color=(0.1, 0.1, 0.1))  # Dark rubber
    )

    # Right rubber bumper (black, under right plank end)
    plank.visual(
        Cylinder(radius=0.05, length=0.05),
        origin=Origin(xyz=(1.5, 0.0, -0.05)),
        name="right_bumper",
        material=Material(name="bumper_material", color=(0.1, 0.1, 0.1))
    )

    # --- Revolute joint (pitch at fulcrum top) ---
    model.articulation(
        "fulcrum_to_plank",
        ArticulationType.REVOLUTE,
        parent=fulcrum,
        child=plank,
        origin=Origin(xyz=(0.0, 0.0, 1.0)),  # Top of fulcrum (1.0m height)
        axis=(0.0, 1.0, 0.0),  # Rotate around Y-axis (pitch)
        motion_limits=MotionLimits(
            effort=100.0,
            velocity=2.0,
            lower=-0.3,  # ~-17 degrees (left end down)
            upper=0.3    # ~17 degrees (right end down)
        )
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fulcrum = object_model.get_part("fulcrum")
    plank = object_model.get_part("plank")
    joint = object_model.get_articulation("fulcrum_to_plank")

    # Allow intentional pivot contact overlap between fulcrum and plank
    ctx.allow_overlap(
        "fulcrum", "plank",
        elem_a="fulcrum_body",
        elem_b="plank_body",
        reason="Intentional pivot contact between fulcrum top and plank center"
    )

    # Validate joint configuration
    ctx.check("joint_exists", joint is not None, details="Missing fulcrum-to-plank articulation")
    ctx.check("joint_type_revolute", joint.articulation_type == ArticulationType.REVOLUTE, details="Articulation is not revolute")
    ctx.check("joint_axis_y", joint.axis == (0.0, 1.0, 0.0), details=f"Joint axis incorrect: {joint.axis}")
    ctx.check(
        "joint_limits",
        joint.motion_limits.lower == -0.3 and joint.motion_limits.upper == 0.3,
        details=f"Joint limits incorrect: lower={joint.motion_limits.lower}, upper={joint.motion_limits.upper}"
    )

    # Rest pose (q=0): plank level, at fulcrum height
    with ctx.pose({joint: 0.0}):
        plank_pos = ctx.part_world_position(plank)
        ctx.check(
            "rest_pose_height",
            abs(plank_pos[2] - 1.0) < 0.001,
            details=f"Plank rest height incorrect: {plank_pos[2]} (expected ~1.0)"
        )

    # Extended pose (q=0.3): right end up, left end down
    with ctx.pose({joint: 0.3}):
        aabb_min, aabb_max = ctx.part_world_aabb(plank)
        # Right end (positive x) should be higher than left end (negative x)
        ctx.check(
            "extended_right_higher",
            aabb_max[2] > aabb_min[2],
            details=f"Right end not higher at q=0.3: min_z={aabb_min[2]}, max_z={aabb_max[2]}"
        )
        # Right end should be ~1.44m above ground (1.0 + 1.5*sin(0.3) ≈ 1.44)
        ctx.check(
            "extended_right_height",
            aabb_max[2] > 1.4,
            details=f"Right end too low at q=0.3: {aabb_max[2]}"
        )

    # Retracted pose (q=-0.3): left end up, right end down
    with ctx.pose({joint: -0.3}):
        aabb_min, aabb_max = ctx.part_world_aabb(plank)
        ctx.check(
            "retracted_left_higher",
            aabb_max[2] > aabb_min[2],
            details=f"Left end not higher at q=-0.3: min_z={aabb_min[2]}, max_z={aabb_max[2]}"
        )
        ctx.check(
            "retracted_left_height",
            aabb_max[2] > 1.4,
            details=f"Left end too low at q=-0.3: {aabb_max[2]}"
        )

    # Validate visible details (seats, handles, bumpers)
    plank_visual_names = {v.name for v in plank.visuals}
    required_visuals = {"plank_body", "left_seat", "right_seat", "left_handle", "right_handle", "left_bumper", "right_bumper"}
    ctx.check(
        "visible_details_present",
        required_visuals.issubset(plank_visual_names),
        details=f"Missing visuals: {required_visuals - plank_visual_names}"
    )

    # Validate fulcrum-plank contact at joint
    ctx.expect_contact(fulcrum, plank, name="fulcrum_plank_contact")

    return ctx.report()

object_model = build_object_model()
