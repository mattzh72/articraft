from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mini_itx_cube_pc_case")

    chassis_color = model.material("powder_coated_steel", rgba=(0.11, 0.11, 0.12, 1.0))
    vent_color = model.material("vent_mesh", rgba=(0.20, 0.22, 0.24, 1.0))
    glass_color = model.material("tempered_glass", rgba=(0.18, 0.22, 0.26, 0.32))
    foot_color = model.material("rubber_feet", rgba=(0.05, 0.05, 0.05, 1.0))

    body_width = 0.290
    body_depth = 0.290
    body_height = 0.260
    frame_thickness = 0.008
    floor_thickness = 0.008
    left_rail_height = 0.022
    foot_radius = 0.015
    foot_height = 0.010

    chassis = model.part("chassis")
    chassis.visual(
        Box((0.282, 0.282, floor_thickness)),
        origin=Origin(xyz=(0.0, 0.0, foot_height + floor_thickness / 2.0)),
        material=chassis_color,
        name="floor",
    )
    chassis.visual(
        Box((body_width, frame_thickness, body_height - foot_height - floor_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                -(body_depth / 2.0 - frame_thickness / 2.0),
                foot_height + floor_thickness + (body_height - foot_height - floor_thickness) / 2.0,
            )
        ),
        material=chassis_color,
        name="front_wall",
    )
    chassis.visual(
        Box((body_width, frame_thickness, body_height - foot_height - floor_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                body_depth / 2.0 - frame_thickness / 2.0,
                foot_height + floor_thickness + (body_height - foot_height - floor_thickness) / 2.0,
            )
        ),
        material=chassis_color,
        name="rear_wall",
    )
    chassis.visual(
        Box((frame_thickness, body_depth - 0.016, body_height - foot_height - floor_thickness)),
        origin=Origin(
            xyz=(
                body_width / 2.0 - frame_thickness / 2.0,
                0.0,
                foot_height + floor_thickness + (body_height - foot_height - floor_thickness) / 2.0,
            )
        ),
        material=chassis_color,
        name="right_wall",
    )
    chassis.visual(
        Box((frame_thickness, body_depth - 0.016, left_rail_height)),
        origin=Origin(
            xyz=(
                -(body_width / 2.0 - frame_thickness / 2.0),
                0.0,
                foot_height + floor_thickness + left_rail_height / 2.0,
            )
        ),
        material=chassis_color,
        name="left_bottom_rail",
    )
    chassis.visual(
        Box((frame_thickness, body_depth - 0.016, left_rail_height)),
        origin=Origin(
            xyz=(
                -(body_width / 2.0 - frame_thickness / 2.0),
                0.0,
                body_height - left_rail_height / 2.0,
            )
        ),
        material=chassis_color,
        name="left_top_rail",
    )
    chassis.visual(
        Cylinder(radius=0.005, length=0.060),
        origin=Origin(
            xyz=(-0.085, body_depth / 2.0, body_height - 0.005),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=chassis_color,
        name="top_hinge_knuckle_left",
    )
    chassis.visual(
        Cylinder(radius=0.005, length=0.060),
        origin=Origin(
            xyz=(0.085, body_depth / 2.0, body_height - 0.005),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=chassis_color,
        name="top_hinge_knuckle_right",
    )
    chassis.visual(
        Cylinder(radius=0.005, length=0.070),
        origin=Origin(
            xyz=(-(body_width / 2.0 - 0.005), -0.102, foot_height + floor_thickness + left_rail_height),
            rpy=(-pi / 2.0, 0.0, 0.0),
        ),
        material=chassis_color,
        name="glass_hinge_knuckle_front",
    )
    chassis.visual(
        Cylinder(radius=0.005, length=0.070),
        origin=Origin(
            xyz=(-(body_width / 2.0 - 0.005), 0.102, foot_height + floor_thickness + left_rail_height),
            rpy=(-pi / 2.0, 0.0, 0.0),
        ),
        material=chassis_color,
        name="glass_hinge_knuckle_rear",
    )

    foot_offsets = ((-0.095, -0.095), (-0.095, 0.095), (0.095, -0.095), (0.095, 0.095))
    for index, (x_pos, y_pos) in enumerate(foot_offsets, start=1):
        chassis.visual(
            Cylinder(radius=foot_radius, length=foot_height),
            origin=Origin(xyz=(x_pos, y_pos, foot_height / 2.0)),
            material=foot_color,
            name=f"foot_{index}",
        )

    top_panel = model.part("top_panel")
    top_panel.visual(
        Box((0.292, 0.292, 0.006)),
        origin=Origin(xyz=(0.0, -0.146, 0.003)),
        material=chassis_color,
        name="lid_plate",
    )
    top_panel.visual(
        Box((0.182, 0.182, 0.0015)),
        origin=Origin(xyz=(0.0, -0.146, 0.00675)),
        material=vent_color,
        name="lid_vent",
    )
    top_panel.visual(
        Box((0.080, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, -0.286, 0.008)),
        material=vent_color,
        name="front_pull",
    )
    top_panel.visual(
        Cylinder(radius=0.005, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.005), rpy=(0.0, pi / 2.0, 0.0)),
        material=chassis_color,
        name="top_center_hinge_knuckle",
    )

    side_glass = model.part("side_glass")
    side_glass.visual(
        Box((0.004, 0.274, 0.202)),
        origin=Origin(xyz=(0.002, 0.0, 0.101)),
        material=glass_color,
        name="glass_pane",
    )
    side_glass.visual(
        Box((0.004, 0.274, 0.010)),
        origin=Origin(xyz=(0.002, 0.0, 0.005)),
        material=chassis_color,
        name="glass_hinge_strip",
    )
    side_glass.visual(
        Cylinder(radius=0.005, length=0.120),
        origin=Origin(xyz=(-0.005, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=chassis_color,
        name="glass_center_hinge_knuckle",
    )

    model.articulation(
        "top_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=top_panel,
        origin=Origin(xyz=(0.0, body_depth / 2.0, body_height)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.5,
            lower=-1.9,
            upper=0.0,
        ),
    )
    model.articulation(
        "side_glass_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=side_glass,
        origin=Origin(
            xyz=(
                -(body_width / 2.0 + 0.004),
                0.0,
                foot_height + floor_thickness + left_rail_height,
            )
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=-1.5,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chassis = object_model.get_part("chassis")
    top_panel = object_model.get_part("top_panel")
    side_glass = object_model.get_part("side_glass")
    top_hinge = object_model.get_articulation("top_hinge")
    side_glass_hinge = object_model.get_articulation("side_glass_hinge")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "required parts present",
        {part.name for part in object_model.parts} == {"chassis", "top_panel", "side_glass"},
        details="Expected chassis, top_panel, and side_glass parts.",
    )
    ctx.check(
        "top panel hinge axis",
        tuple(top_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"Expected top hinge axis (1, 0, 0), got {top_hinge.axis}.",
    )
    ctx.check(
        "side glass hinge axis",
        tuple(side_glass_hinge.axis) == (0.0, 1.0, 0.0),
        details=f"Expected side hinge axis (0, 1, 0), got {side_glass_hinge.axis}.",
    )
    ctx.check(
        "hinge motion ranges",
        (
            top_hinge.motion_limits is not None
            and top_hinge.motion_limits.lower is not None
            and top_hinge.motion_limits.upper == 0.0
            and top_hinge.motion_limits.lower < -1.0
            and side_glass_hinge.motion_limits is not None
            and side_glass_hinge.motion_limits.lower is not None
            and side_glass_hinge.motion_limits.upper == 0.0
            and side_glass_hinge.motion_limits.lower < -1.0
        ),
        details="Both hinged panels should open from the closed 0 rad pose into negative angles.",
    )

    closed_glass_center_x = None
    with ctx.pose({side_glass_hinge: 0.0}):
        closed_glass_aabb = ctx.part_element_world_aabb(side_glass, elem="glass_pane")
        if closed_glass_aabb is not None:
            closed_glass_center_x = (closed_glass_aabb[0][0] + closed_glass_aabb[1][0]) / 2.0

    with ctx.pose({top_hinge: 0.0, side_glass_hinge: 0.0}):
        ctx.expect_gap(
            top_panel,
            chassis,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            name="top panel seats on chassis rim",
        )
        ctx.expect_overlap(
            top_panel,
            chassis,
            axes="xy",
            min_overlap=0.285,
            name="top panel covers chassis footprint",
        )
        ctx.expect_gap(
            chassis,
            side_glass,
            axis="x",
            max_gap=0.001,
            max_penetration=0.0,
            name="side glass closes flush to opening",
        )
        ctx.expect_overlap(
            chassis,
            side_glass,
            axes="yz",
            min_overlap=0.200,
            name="side glass covers left side aperture",
        )

    with ctx.pose({top_hinge: -1.2}):
        ctx.expect_gap(
            top_panel,
            chassis,
            axis="z",
            positive_elem="front_pull",
            min_gap=0.090,
            name="top panel lifts at the front when opened",
        )

    with ctx.pose({side_glass_hinge: -1.2}):
        open_glass_aabb = ctx.part_element_world_aabb(side_glass, elem="glass_pane")
        open_glass_center_x = None
        if open_glass_aabb is not None:
            open_glass_center_x = (open_glass_aabb[0][0] + open_glass_aabb[1][0]) / 2.0
        ctx.check(
            "side glass swings outward from the chassis",
            (
                closed_glass_center_x is not None
                and open_glass_center_x is not None
                and open_glass_center_x < closed_glass_center_x - 0.080
            ),
            details=(
                "Expected glass panel center to swing at least 80 mm outward from the closed pose; "
                f"closed_center_x={closed_glass_center_x} open_center_x={open_glass_center_x}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
