from __future__ import annotations

import math
from cadquery import Workplane, Vector
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _rounded_plank(length: float, width: float, thickness: float, radius: float = 0.01) -> object:
    """Create a plank with rounded edges using CadQuery."""
    # Create a box and fillet all edges
    plank = (
        Workplane("XY")
        .box(length, width, thickness)
        .edges("|Z")
        .fillet(radius)
    )
    return mesh_from_cadquery(plank, "plank_body")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wood_log_seesaw")

    # Natural wood material
    wood_material = Material(name="natural_wood", rgba=(0.65, 0.45, 0.25, 1.0))
    dark_wood_material = Material(name="dark_wood", rgba=(0.5, 0.35, 0.2, 1.0))
    metal_material = Material(name="bolt_metal", rgba=(0.7, 0.7, 0.7, 1.0))

    # ========== FULCRUM (root part, fixed base) ==========
    fulcrum = model.part("fulcrum")

    # Central log-like fulcrum: horizontal cylinder (along Y-axis) with wood texture
    # Cylinder is aligned with local Z, so rotate 90° around X to make it horizontal along Y
    fulcrum_log = Cylinder(radius=0.15, length=0.8)  # radius 15cm, length 80cm
    fulcrum.visual(
        fulcrum_log,
        origin=Origin(xyz=(0.0, 0.0, 0.15), rpy=(math.pi/2, 0.0, 0.0)),  # Rotate to Y-axis, lift so bottom is at z=0
        material=wood_material,
        name="fulcrum_log",
    )

    # ========== PLANK (moving part, child of fulcrum) ==========
    plank = model.part("plank")

    # Main plank body: 3m long, 0.3m wide, 0.05m thick with rounded edges
    # Plank part frame is at joint origin (0,0,0.15) in world. 
    # Plank bottom should be at top of fulcrum (0.3m), so plank center is at 0.325m world Z
    # Relative to plank part frame: 0.325 - 0.15 = 0.175m
    plank_length = 3.0
    plank_width = 0.3
    plank_thickness = 0.05
    plank_mesh = _rounded_plank(plank_length, plank_width, plank_thickness, radius=0.015)
    plank.visual(
        plank_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.175)),  # Relative to plank part frame
        material=dark_wood_material,
        name="plank_body",
    )

    # Plank seats at each end (35cm x 25cm x 3cm)
    # Seat bottom on top of plank: plank top is at 0.325 + 0.025 = 0.35m world Z
    # Seat center Z: 0.35 + 0.015 = 0.365m world Z → relative: 0.365 - 0.15 = 0.215m
    seat_size = (0.35, 0.25, 0.03)
    for i, x_offset in enumerate([-1.4, 1.4]):  # 10cm from each end of the 3m plank
        seat = Box(seat_size)
        plank.visual(
            seat,
            origin=Origin(xyz=(x_offset, 0.0, 0.215)),  # Relative to plank part frame
            material=wood_material,
            name=f"seat_{i}",
        )

    # Handle posts at each end (vertical cylinders, 40cm tall, 3cm diameter)
    # Handle center Z: 0.35 (plank top) + 0.2 (half handle height) = 0.55m world Z → relative: 0.55 - 0.15 = 0.4m
    handle_height = 0.4
    handle_radius = 0.015
    for i, x_offset in enumerate([-1.45, 1.45]):
        handle = Cylinder(radius=handle_radius, length=handle_height)
        plank.visual(
            handle,
            origin=Origin(xyz=(x_offset, 0.0, 0.4)),  # Relative to plank part frame
            material=dark_wood_material,
            name=f"handle_post_{i}",
        )

    # Bolt heads at pivot point (center of plank)
    # Bolt center Z: 0.35 (plank top) + 0.005 (half bolt thickness) = 0.355m world Z → relative: 0.355 - 0.15 = 0.205m
    bolt_radius = 0.02
    bolt_thickness = 0.01
    for i, y_offset in enumerate([-0.1, 0.1]):  # Two bolts on each side of center
        bolt = Cylinder(radius=bolt_radius, length=bolt_thickness)
        plank.visual(
            bolt,
            origin=Origin(xyz=(0.0, y_offset, 0.205)),  # Relative to plank part frame
            material=metal_material,
            name=f"bolt_head_{i}",
        )

    # ========== ARTICULATION (pitch joint around Y-axis) ==========
    model.articulation(
        "plank_pitch",
        ArticulationType.REVOLUTE,
        parent=fulcrum,
        child=plank,
        # Joint origin at top of fulcrum (0,0,0.3) in world, which is (0,0,0.15) in fulcrum's frame
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        axis=(0.0, 1.0, 0.0),  # Rotate around Y-axis (pitch)
        motion_limits=MotionLimits(effort=100.0, velocity=1.0, lower=-0.35, upper=0.35),  # ~±20° range
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fulcrum = object_model.get_part("fulcrum")
    plank = object_model.get_part("plank")
    joint = object_model.get_articulation("plank_pitch")

    # ===== Mechanism Tests =====
    ctx.check("has_single_articulation", len(object_model.joints) == 1, "Should have exactly one articulation")
    ctx.check("articulation_type_revolute", joint.articulation_type == ArticulationType.REVOLUTE, "Joint should be revolute")
    ctx.check("joint_axis_y", joint.axis == (0.0, 1.0, 0.0), "Joint axis should be along Y for pitch")

    # ===== Support/Contact Test =====
    with ctx.pose({joint: 0.0}):
        # At rest, plank should contact fulcrum at pivot point
        ctx.expect_contact(fulcrum, plank, elem_a="fulcrum_log", name="plank_contacts_fulcrum_at_rest")

    # ===== Closed/Rest Pose Test =====
    with ctx.pose({joint: 0.0}):
        # Check that plank body visual is at correct height (0.325m center)
        plank_body_aabb = ctx.part_element_world_aabb(plank, elem="plank_body")
        if plank_body_aabb:
            body_center_z = (plank_body_aabb[0][2] + plank_body_aabb[1][2]) / 2
            ctx.check(
                "rest_pose_horizontal",
                abs(body_center_z - 0.325) < 0.01,
                f"Plank body center should be ~0.325m high at rest, got {body_center_z:.3f}m"
            )

    # ===== Extended Pose Tests =====
    with ctx.pose({joint: 0.35}):  # Positive rotation: -X end up, +X end down
        plank_aabb = ctx.part_world_aabb(plank)
        if plank_aabb:
            min_z = plank_aabb[0][2]
            max_z = plank_aabb[1][2]
            # For 3m plank pitched 0.35rad, Z range should be ~1.0m (3*sin(0.35)), which is less than plank length (3m)
            ctx.check(
                "positive_pitch_lowers_positive_x_end",
                max_z - min_z < 2.0,  # Should be pitched, not flat
                f"Plank should be pitched at +0.35rad, Z range: {max_z - min_z:.2f}m"
            )
            # Check that +X end is lower than -X end
            pos_x_end = ctx.part_world_position(plank)
            if pos_x_end:
                ctx.check(
                    "positive_pitch_positive_x_lower",
                    pos_x_end[2] < 0.325,  # Center should be lower on positive X side
                    f"Positive X end should be lower at +0.35rad pitch, center Z: {pos_x_end[2]:.2f}"
                )

    with ctx.pose({joint: -0.35}):  # Negative rotation: +X end up, -X end down
        plank_aabb = ctx.part_world_aabb(plank)
        if plank_aabb:
            min_z = plank_aabb[0][2]
            max_z = plank_aabb[1][2]
            ctx.check(
                "negative_pitch_lowers_negative_x_end",
                max_z - min_z < 2.0,
                f"Plank should be pitched at -0.35rad, Z range: {max_z - min_z:.2f}m"
            )
            # Check that -X end is lower than +X end
            pos_x_end = ctx.part_world_position(plank)
            if pos_x_end:
                ctx.check(
                    "negative_pitch_negative_x_lower",
                    pos_x_end[2] < 0.325,  # Center should be lower on negative X side? Wait, no: negative pitch rotates around Y, so -X end goes down
                    f"Negative X end should be lower at -0.35rad pitch, center Z: {pos_x_end[2]:.2f}"
                )

    # ===== Visible Details Tests =====
    plank_visuals = plank.visuals
    ctx.check(
        "has_seats",
        any(v.name.startswith("seat_") for v in plank_visuals),
        "Plank should have seat visuals"
    )
    ctx.check(
        "has_handle_posts",
        any(v.name.startswith("handle_post_") for v in plank_visuals),
        "Plank should have handle post visuals"
    )
    ctx.check(
        "has_bolt_heads",
        any(v.name.startswith("bolt_head_") for v in plank_visuals),
        "Plank should have bolt head visuals"
    )
    ctx.check(
        "has_rounded_plank",
        any(v.name == "plank_body" for v in plank_visuals),
        "Plank should have rounded body visual"
    )

    return ctx.report()


object_model = build_object_model()
