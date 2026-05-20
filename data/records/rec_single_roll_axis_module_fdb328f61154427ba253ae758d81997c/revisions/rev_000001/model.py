from __future__ import annotations

import math

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
    model = ArticulatedObject(name="compact_roll_axis_spindle_module")

    painted_steel = model.material("painted_steel", rgba=(0.10, 0.12, 0.14, 1.0))
    blue_grey = model.material("blue_grey_support", rgba=(0.20, 0.28, 0.34, 1.0))
    dark_cap = model.material("blackened_fasteners", rgba=(0.025, 0.025, 0.025, 1.0))
    satin_shaft = model.material("satin_steel", rgba=(0.70, 0.72, 0.70, 1.0))
    bearing_metal = model.material("bearing_metal", rgba=(0.54, 0.56, 0.57, 1.0))
    tool_face = model.material("ground_tooling_face", rgba=(0.82, 0.82, 0.78, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.52, 0.30, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=painted_steel,
        name="base_plate",
    )
    base.visual(
        Box((0.40, 0.19, 0.112)),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=blue_grey,
        name="boxed_support",
    )
    base.visual(
        Box((0.43, 0.215, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material=painted_steel,
        name="top_cap",
    )

    # Two pillow-block style supports carry the roll shaft on both sides while
    # leaving a visible clearance window around the rotating shaft.
    axis_z = 0.235
    block_thickness = 0.052
    base.visual(
        Box((block_thickness, 0.140, 0.052)),
        origin=Origin(xyz=(-0.165, 0.0, 0.180)),
        material=blue_grey,
        name="bearing_block_0",
    )
    base.visual(
        Box((block_thickness, 0.026, 0.142)),
        origin=Origin(xyz=(-0.165, -0.058, 0.225)),
        material=blue_grey,
        name="bearing_block_0_cheek_0",
    )
    base.visual(
        Box((block_thickness, 0.026, 0.142)),
        origin=Origin(xyz=(-0.165, 0.058, 0.225)),
        material=blue_grey,
        name="bearing_block_0_cheek_1",
    )
    base.visual(
        Box((block_thickness, 0.140, 0.030)),
        origin=Origin(xyz=(-0.165, 0.0, 0.279)),
        material=blue_grey,
        name="bearing_block_0_cap",
    )
    base.visual(
        Box((0.012, 0.070, 0.008)),
        origin=Origin(xyz=(-0.136, 0.0, axis_z - 0.0255)),
        material=bearing_metal,
        name="bearing_block_0_lower_race",
    )
    base.visual(
        Box((0.012, 0.070, 0.008)),
        origin=Origin(xyz=(-0.136, 0.0, axis_z + 0.0255)),
        material=bearing_metal,
        name="bearing_block_0_upper_race",
    )
    base.visual(
        Box((0.012, 0.0235, 0.044)),
        origin=Origin(xyz=(-0.136, -0.03325, axis_z)),
        material=bearing_metal,
        name="bearing_block_0_side_race_0",
    )
    base.visual(
        Box((0.012, 0.0235, 0.044)),
        origin=Origin(xyz=(-0.136, 0.03325, axis_z)),
        material=bearing_metal,
        name="bearing_block_0_side_race_1",
    )
    base.visual(
        Box((block_thickness, 0.140, 0.052)),
        origin=Origin(xyz=(0.165, 0.0, 0.180)),
        material=blue_grey,
        name="bearing_block_1",
    )
    base.visual(
        Box((block_thickness, 0.026, 0.142)),
        origin=Origin(xyz=(0.165, -0.058, 0.225)),
        material=blue_grey,
        name="bearing_block_1_cheek_0",
    )
    base.visual(
        Box((block_thickness, 0.026, 0.142)),
        origin=Origin(xyz=(0.165, 0.058, 0.225)),
        material=blue_grey,
        name="bearing_block_1_cheek_1",
    )
    base.visual(
        Box((block_thickness, 0.140, 0.030)),
        origin=Origin(xyz=(0.165, 0.0, 0.279)),
        material=blue_grey,
        name="bearing_block_1_cap",
    )
    base.visual(
        Box((0.012, 0.070, 0.008)),
        origin=Origin(xyz=(0.136, 0.0, axis_z - 0.0255)),
        material=bearing_metal,
        name="bearing_block_1_lower_race",
    )
    base.visual(
        Box((0.012, 0.070, 0.008)),
        origin=Origin(xyz=(0.136, 0.0, axis_z + 0.0255)),
        material=bearing_metal,
        name="bearing_block_1_upper_race",
    )
    base.visual(
        Box((0.012, 0.0235, 0.044)),
        origin=Origin(xyz=(0.136, -0.03325, axis_z)),
        material=bearing_metal,
        name="bearing_block_1_side_race_0",
    )
    base.visual(
        Box((0.012, 0.0235, 0.044)),
        origin=Origin(xyz=(0.136, 0.03325, axis_z)),
        material=bearing_metal,
        name="bearing_block_1_side_race_1",
    )

    # Mounting screw heads and small gussets make the bench read as bolted-down.
    for x in (-0.215, 0.215):
        for y in (-0.115, 0.115):
            base.visual(
                Cylinder(radius=0.012, length=0.006),
                origin=Origin(xyz=(x, y, 0.032), rpy=(0.0, 0.0, 0.0)),
                material=dark_cap,
                name=f"base_bolt_{x:+.3f}_{y:+.3f}",
            )
    for x in (-0.080, 0.080):
        for y in (-0.100, 0.100):
            base.visual(
                Box((0.020, 0.018, 0.075)),
                origin=Origin(xyz=(x, y, 0.095)),
                material=painted_steel,
                name=f"support_gusset_{x:+.3f}_{y:+.3f}",
            )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.022, length=0.425),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_shaft,
        name="shaft",
    )
    spindle.visual(
        Cylinder(radius=0.032, length=0.020),
        origin=Origin(xyz=(-0.117, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing_metal,
        name="collar_0",
    )
    spindle.visual(
        Cylinder(radius=0.032, length=0.020),
        origin=Origin(xyz=(0.117, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing_metal,
        name="collar_1",
    )
    spindle.visual(
        Cylinder(radius=0.064, length=0.030),
        origin=Origin(xyz=(0.030, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_shaft,
        name="flange_body",
    )
    spindle.visual(
        Cylinder(radius=0.052, length=0.010),
        origin=Origin(xyz=(0.050, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=tool_face,
        name="tooling_face",
    )
    spindle.visual(
        Cylinder(radius=0.025, length=0.014),
        origin=Origin(xyz=(0.058, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing_metal,
        name="pilot_boss",
    )
    for i in range(6):
        angle = 2.0 * math.pi * i / 6.0
        y = 0.037 * math.cos(angle)
        z = 0.037 * math.sin(angle)
        spindle.visual(
            Cylinder(radius=0.0042, length=0.006),
            origin=Origin(xyz=(0.057, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_cap,
            name=f"face_bolt_{i}",
        )

    model.articulation(
        "roll_axis",
        ArticulationType.REVOLUTE,
        parent=base,
        child=spindle,
        origin=Origin(xyz=(0.0, 0.0, axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=12.0, lower=-math.pi, upper=math.pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    spindle = object_model.get_part("spindle")
    roll_axis = object_model.get_articulation("roll_axis")

    ctx.check(
        "single roll revolute joint",
        len(object_model.articulations) == 1
        and roll_axis.articulation_type == ArticulationType.REVOLUTE
        and tuple(roll_axis.axis) == (1.0, 0.0, 0.0),
        details=f"articulations={object_model.articulations}",
    )
    ctx.expect_overlap(
        spindle,
        base,
        axes="x",
        elem_a="shaft",
        elem_b="bearing_block_0",
        min_overlap=0.030,
        name="shaft passes through first bearing support",
    )
    ctx.expect_overlap(
        spindle,
        base,
        axes="x",
        elem_a="shaft",
        elem_b="bearing_block_1",
        min_overlap=0.030,
        name="shaft passes through second bearing support",
    )
    ctx.expect_gap(
        spindle,
        base,
        axis="x",
        positive_elem="collar_1",
        negative_elem="bearing_block_0",
        min_gap=0.010,
        name="center tooling clears the first support",
    )
    ctx.expect_gap(
        base,
        spindle,
        axis="x",
        positive_elem="bearing_block_1",
        negative_elem="collar_1",
        min_gap=0.010,
        name="right collar clears the second support",
    )

    rest_aabb = ctx.part_world_aabb(spindle)
    with ctx.pose({roll_axis: math.pi / 2.0}):
        turned_aabb = ctx.part_world_aabb(spindle)
        ctx.expect_overlap(
            spindle,
            base,
            axes="x",
            elem_a="shaft",
            elem_b="bearing_block_0",
            min_overlap=0.030,
            name="rotated shaft remains carried in first bearing",
        )
    ctx.check(
        "flange spins without translating",
        rest_aabb is not None
        and turned_aabb is not None
        and abs(rest_aabb[0][0] - turned_aabb[0][0]) < 1e-6
        and abs(rest_aabb[1][0] - turned_aabb[1][0]) < 1e-6,
        details=f"rest={rest_aabb}, turned={turned_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
