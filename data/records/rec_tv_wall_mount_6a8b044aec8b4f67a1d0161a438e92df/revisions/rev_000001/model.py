from __future__ import annotations

import math

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bare_heavy_wall_display_mount")

    black_steel = Material("black_powder_coated_steel", rgba=(0.02, 0.023, 0.025, 1.0))
    edge_worn = Material("worn_dark_steel_edges", rgba=(0.13, 0.14, 0.14, 1.0))
    bolt_black = Material("blackened_bolt_heads", rgba=(0.005, 0.005, 0.006, 1.0))

    # The root frame is the shoulder pivot center.  +X points out from the wall,
    # Y is horizontal along the wall, and Z is vertical.
    wall_plate = model.part("wall_plate")
    wall_plate.visual(
        Box((0.040, 0.360, 0.580)),
        origin=Origin(xyz=(-0.075, 0.0, 0.0)),
        material=black_steel,
        name="back_plate",
    )
    wall_plate.visual(
        Box((0.018, 0.035, 0.520)),
        origin=Origin(xyz=(-0.046, -0.145, 0.0)),
        material=edge_worn,
        name="side_flange_0",
    )
    wall_plate.visual(
        Box((0.018, 0.035, 0.520)),
        origin=Origin(xyz=(-0.046, 0.145, 0.0)),
        material=edge_worn,
        name="side_flange_1",
    )
    wall_plate.visual(
        Box((0.018, 0.075, 0.480)),
        origin=Origin(xyz=(-0.046, 0.0, 0.0)),
        material=edge_worn,
        name="center_spine",
    )
    wall_plate.visual(
        Box((0.120, 0.025, 0.150)),
        origin=Origin(xyz=(0.005, 0.065, 0.0)),
        material=black_steel,
        name="shoulder_yoke_pos",
    )
    wall_plate.visual(
        Box((0.120, 0.025, 0.150)),
        origin=Origin(xyz=(0.005, -0.065, 0.0)),
        material=black_steel,
        name="shoulder_yoke_neg",
    )
    for y, z, idx in (
        (-0.105, 0.205, 0),
        (0.105, 0.205, 1),
        (-0.105, -0.205, 2),
        (0.105, -0.205, 3),
    ):
        wall_plate.visual(
            Cylinder(radius=0.020, length=0.008),
            origin=Origin(xyz=(-0.051, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bolt_black,
            name=f"wall_bolt_{idx}",
        )
    for y, idx in ((0.0805, 0), (-0.0805, 1)):
        wall_plate.visual(
            Cylinder(radius=0.020, length=0.006),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=bolt_black,
            name=f"shoulder_pin_head_{idx}",
        )

    inner_arm = model.part("inner_arm")
    inner_arm.visual(
        Box((0.100, 0.105, 0.110)),
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
        material=black_steel,
        name="shoulder_tongue",
    )
    inner_arm.visual(
        Box((0.300, 0.060, 0.035)),
        origin=Origin(xyz=(0.190, 0.0, 0.070)),
        material=black_steel,
        name="upper_tube",
    )
    inner_arm.visual(
        Box((0.300, 0.060, 0.035)),
        origin=Origin(xyz=(0.190, 0.0, -0.070)),
        material=black_steel,
        name="lower_tube",
    )
    inner_arm.visual(
        Box((0.300, 0.018, 0.105)),
        origin=Origin(xyz=(0.190, 0.0, 0.0)),
        material=edge_worn,
        name="web_plate",
    )
    inner_arm.visual(
        Box((0.045, 0.105, 0.105)),
        origin=Origin(xyz=(0.335, 0.0, 0.0)),
        material=black_steel,
        name="elbow_end_block",
    )
    inner_arm.visual(
        Box((0.100, 0.025, 0.130)),
        origin=Origin(xyz=(0.380, 0.065, 0.0)),
        material=black_steel,
        name="elbow_yoke_pos",
    )
    inner_arm.visual(
        Box((0.100, 0.025, 0.130)),
        origin=Origin(xyz=(0.380, -0.065, 0.0)),
        material=black_steel,
        name="elbow_yoke_neg",
    )
    for y, idx in ((0.0805, 0), (-0.0805, 1)):
        inner_arm.visual(
            Cylinder(radius=0.020, length=0.006),
            origin=Origin(xyz=(0.380, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=bolt_black,
            name=f"elbow_pin_head_{idx}",
        )

    outer_arm = model.part("outer_arm")
    outer_arm.visual(
        Box((0.100, 0.105, 0.110)),
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
        material=black_steel,
        name="elbow_tongue",
    )
    outer_arm.visual(
        Box((0.270, 0.056, 0.035)),
        origin=Origin(xyz=(0.175, 0.0, 0.070)),
        material=black_steel,
        name="upper_tube",
    )
    outer_arm.visual(
        Box((0.270, 0.056, 0.035)),
        origin=Origin(xyz=(0.175, 0.0, -0.070)),
        material=black_steel,
        name="lower_tube",
    )
    outer_arm.visual(
        Box((0.270, 0.016, 0.105)),
        origin=Origin(xyz=(0.175, 0.0, 0.0)),
        material=edge_worn,
        name="web_plate",
    )
    outer_arm.visual(
        Box((0.045, 0.105, 0.105)),
        origin=Origin(xyz=(0.295, 0.0, 0.0)),
        material=black_steel,
        name="swivel_end_block",
    )
    outer_arm.visual(
        Box((0.100, 0.025, 0.130)),
        origin=Origin(xyz=(0.340, 0.065, 0.0)),
        material=black_steel,
        name="swivel_yoke_pos",
    )
    outer_arm.visual(
        Box((0.100, 0.025, 0.130)),
        origin=Origin(xyz=(0.340, -0.065, 0.0)),
        material=black_steel,
        name="swivel_yoke_neg",
    )
    for y, idx in ((0.0805, 0), (-0.0805, 1)):
        outer_arm.visual(
            Cylinder(radius=0.020, length=0.006),
            origin=Origin(xyz=(0.340, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=bolt_black,
            name=f"swivel_pin_head_{idx}",
        )

    head_frame = model.part("head_frame")
    head_frame.visual(
        Box((0.095, 0.105, 0.110)),
        origin=Origin(xyz=(0.025, 0.0, 0.0)),
        material=black_steel,
        name="swivel_tongue",
    )
    head_frame.visual(
        Box((0.105, 0.055, 0.055)),
        origin=Origin(xyz=(0.105, 0.0, 0.0)),
        material=black_steel,
        name="neck_block",
    )
    head_frame.visual(
        Box((0.025, 0.340, 0.025)),
        origin=Origin(xyz=(0.160, 0.0, 0.170)),
        material=black_steel,
        name="top_rail",
    )
    head_frame.visual(
        Box((0.025, 0.340, 0.025)),
        origin=Origin(xyz=(0.160, 0.0, -0.170)),
        material=black_steel,
        name="bottom_rail",
    )
    head_frame.visual(
        Box((0.025, 0.025, 0.340)),
        origin=Origin(xyz=(0.160, 0.170, 0.0)),
        material=black_steel,
        name="side_rail_pos",
    )
    head_frame.visual(
        Box((0.025, 0.025, 0.340)),
        origin=Origin(xyz=(0.160, -0.170, 0.0)),
        material=black_steel,
        name="side_rail_neg",
    )
    head_frame.visual(
        Box((0.025, 0.300, 0.035)),
        origin=Origin(xyz=(0.160, 0.0, 0.0)),
        material=edge_worn,
        name="rear_cross_spine",
    )
    head_frame.visual(
        Box((0.025, 0.055, 0.300)),
        origin=Origin(xyz=(0.160, 0.0, 0.0)),
        material=edge_worn,
        name="rear_vertical_spine",
    )
    head_frame.visual(
        Box((0.055, 0.035, 0.120)),
        origin=Origin(xyz=(0.200, 0.165, 0.0)),
        material=black_steel,
        name="tilt_bearing_pos",
    )
    head_frame.visual(
        Box((0.055, 0.035, 0.120)),
        origin=Origin(xyz=(0.200, -0.165, 0.0)),
        material=black_steel,
        name="tilt_bearing_neg",
    )

    tilt_crossbar = model.part("tilt_crossbar")
    tilt_crossbar.visual(
        Box((0.035, 0.295, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=black_steel,
        name="crossbar_beam",
    )
    tilt_crossbar.visual(
        Box((0.030, 0.200, 0.120)),
        origin=Origin(xyz=(0.025, 0.0, 0.0)),
        material=black_steel,
        name="front_plate",
    )
    for y, z, idx in (
        (-0.070, 0.040, 0),
        (0.070, 0.040, 1),
        (-0.070, -0.040, 2),
        (0.070, -0.040, 3),
    ):
        tilt_crossbar.visual(
            Cylinder(radius=0.012, length=0.006),
            origin=Origin(xyz=(0.043, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bolt_black,
            name=f"vesa_bolt_{idx}",
        )

    model.articulation(
        "shoulder_swing",
        ArticulationType.REVOLUTE,
        parent=wall_plate,
        child=inner_arm,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "elbow_swing",
        ArticulationType.REVOLUTE,
        parent=inner_arm,
        child=outer_arm,
        origin=Origin(xyz=(0.380, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=100.0, velocity=1.2, lower=-2.20, upper=2.20),
    )
    model.articulation(
        "head_swivel",
        ArticulationType.REVOLUTE,
        parent=outer_arm,
        child=head_frame,
        origin=Origin(xyz=(0.340, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.5, lower=-1.60, upper=1.60),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=head_frame,
        child=tilt_crossbar,
        origin=Origin(xyz=(0.200, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.0, lower=-0.35, upper=0.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    wall_plate = object_model.get_part("wall_plate")
    inner_arm = object_model.get_part("inner_arm")
    outer_arm = object_model.get_part("outer_arm")
    head_frame = object_model.get_part("head_frame")
    tilt_crossbar = object_model.get_part("tilt_crossbar")

    shoulder = object_model.get_articulation("shoulder_swing")
    elbow = object_model.get_articulation("elbow_swing")
    swivel = object_model.get_articulation("head_swivel")
    tilt = object_model.get_articulation("head_tilt")

    ctx.check(
        "four explicit revolute stages",
        len(object_model.articulations) == 4
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in object_model.articulations),
        details="The mount should expose shoulder, elbow, head swivel, and head tilt joints.",
    )
    ctx.expect_contact(
        wall_plate,
        inner_arm,
        elem_a="shoulder_yoke_pos",
        elem_b="shoulder_tongue",
        name="wall clevis supports shoulder tongue",
    )
    ctx.expect_contact(
        inner_arm,
        outer_arm,
        elem_a="elbow_yoke_pos",
        elem_b="elbow_tongue",
        name="inner arm clevis supports outer arm",
    )
    ctx.expect_contact(
        outer_arm,
        head_frame,
        elem_a="swivel_yoke_pos",
        elem_b="swivel_tongue",
        name="outer arm clevis supports head swivel",
    )
    ctx.expect_contact(
        head_frame,
        tilt_crossbar,
        elem_a="tilt_bearing_pos",
        elem_b="crossbar_beam",
        name="square frame bearings carry the tilt crossbar",
    )

    rest_outer = ctx.part_world_position(outer_arm)
    rest_head = ctx.part_world_position(head_frame)
    rest_crossbar = ctx.part_world_position(tilt_crossbar)
    rest_plate_aabb = ctx.part_element_world_aabb(tilt_crossbar, elem="front_plate")

    with ctx.pose({shoulder: 0.70}):
        swung_outer = ctx.part_world_position(outer_arm)
    ctx.check(
        "shoulder swing moves the elbow stage sideways",
        rest_outer is not None and swung_outer is not None and swung_outer[1] > rest_outer[1] + 0.20,
        details=f"rest={rest_outer}, swung={swung_outer}",
    )

    with ctx.pose({elbow: 0.90}):
        folded_head = ctx.part_world_position(head_frame)
    ctx.check(
        "elbow swing folds the second arm link",
        rest_head is not None and folded_head is not None and folded_head[1] > rest_head[1] + 0.20,
        details=f"rest={rest_head}, folded={folded_head}",
    )

    with ctx.pose({swivel: 0.75}):
        swiveled_crossbar = ctx.part_world_position(tilt_crossbar)
    ctx.check(
        "head swivel yaws the head frame",
        rest_crossbar is not None
        and swiveled_crossbar is not None
        and swiveled_crossbar[1] > rest_crossbar[1] + 0.10,
        details=f"rest={rest_crossbar}, swiveled={swiveled_crossbar}",
    )

    with ctx.pose({tilt: 0.30}):
        tilted_plate_aabb = ctx.part_element_world_aabb(tilt_crossbar, elem="front_plate")
    ctx.check(
        "head tilt rotates the display crossbar",
        rest_plate_aabb is not None
        and tilted_plate_aabb is not None
        and tilted_plate_aabb[0][2] < rest_plate_aabb[0][2] - 0.005,
        details=f"rest={rest_plate_aabb}, tilted={tilted_plate_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
