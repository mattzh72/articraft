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
    model = ArticulatedObject(name="xyz_cartesian_stage")

    anodized = model.material("dark_anodized_aluminum", color=(0.10, 0.11, 0.12, 1.0))
    carriage_blue = model.material("blue_anodized_carriage", color=(0.05, 0.20, 0.42, 1.0))
    rail_steel = model.material("brushed_steel", color=(0.70, 0.72, 0.70, 1.0))
    black_shadow = model.material("cable_gap_shadow", color=(0.01, 0.012, 0.015, 1.0))
    plate_gray = model.material("machined_plate", color=(0.55, 0.57, 0.58, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.90, 0.30, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=anodized,
        name="base_plate",
    )
    base.visual(
        Box((0.78, 0.026, 0.036)),
        origin=Origin(xyz=(0.0, -0.090, 0.058)),
        material=rail_steel,
        name="x_rail_0",
    )
    base.visual(
        Box((0.78, 0.026, 0.036)),
        origin=Origin(xyz=(0.0, 0.090, 0.058)),
        material=rail_steel,
        name="x_rail_1",
    )
    for i, x in enumerate((-0.42, 0.42)):
        base.visual(
            Box((0.035, 0.26, 0.030)),
            origin=Origin(xyz=(x, 0.0, 0.055)),
            material=anodized,
            name=f"x_end_stop_{i}",
        )
    base.visual(
        Box((0.66, 0.052, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        material=black_shadow,
        name="x_cable_trough",
    )

    x_carriage = model.part("x_carriage")
    x_carriage.visual(
        Box((0.180, 0.055, 0.045)),
        origin=Origin(xyz=(0.0, -0.075, 0.0225)),
        material=carriage_blue,
        name="x_bearing_shoe_0",
    )
    x_carriage.visual(
        Box((0.180, 0.055, 0.045)),
        origin=Origin(xyz=(0.0, 0.075, 0.0225)),
        material=carriage_blue,
        name="x_bearing_shoe_1",
    )
    for i, x in enumerate((-0.075, 0.075)):
        x_carriage.visual(
            Box((0.030, 0.205, 0.045)),
            origin=Origin(xyz=(x, 0.0, 0.0225)),
            material=carriage_blue,
            name=f"x_bridge_{i}",
        )
    x_carriage.visual(
        Box((0.120, 0.095, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=black_shadow,
        name="x_cable_gap",
    )
    for i, x in enumerate((-0.120, 0.120)):
        x_carriage.visual(
            Box((0.065, 0.350, 0.035)),
            origin=Origin(xyz=(x, 0.0, 0.0605)),
            material=anodized,
            name=f"y_bed_side_{i}",
        )
    for i, y in enumerate((-0.145, 0.145)):
        x_carriage.visual(
            Box((0.295, 0.040, 0.035)),
            origin=Origin(xyz=(0.0, y, 0.0605)),
            material=anodized,
            name=f"y_bed_bridge_{i}",
        )
    x_carriage.visual(
        Box((0.170, 0.235, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        material=black_shadow,
        name="y_cable_gap",
    )
    x_carriage.visual(
        Box((0.026, 0.315, 0.032)),
        origin=Origin(xyz=(-0.100, 0.0, 0.094)),
        material=rail_steel,
        name="y_rail_0",
    )
    x_carriage.visual(
        Box((0.026, 0.315, 0.032)),
        origin=Origin(xyz=(0.100, 0.0, 0.094)),
        material=rail_steel,
        name="y_rail_1",
    )

    y_carriage = model.part("y_carriage")
    y_carriage.visual(
        Box((0.055, 0.140, 0.040)),
        origin=Origin(xyz=(-0.100, 0.0, 0.020)),
        material=carriage_blue,
        name="y_bearing_shoe_0",
    )
    y_carriage.visual(
        Box((0.055, 0.140, 0.040)),
        origin=Origin(xyz=(0.100, 0.0, 0.020)),
        material=carriage_blue,
        name="y_bearing_shoe_1",
    )
    for i, y in enumerate((-0.055, 0.055)):
        y_carriage.visual(
            Box((0.255, 0.030, 0.040)),
            origin=Origin(xyz=(0.0, y, 0.020)),
            material=carriage_blue,
            name=f"y_bridge_{i}",
        )
    y_carriage.visual(
        Box((0.125, 0.080, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=black_shadow,
        name="vertical_cable_gap",
    )
    for i, x in enumerate((-0.115, 0.115)):
        y_carriage.visual(
            Box((0.055, 0.045, 0.400)),
            origin=Origin(xyz=(x, 0.055, 0.240)),
            material=anodized,
            name=f"z_frame_side_{i}",
        )
    for i, z in enumerate((0.060, 0.420)):
        y_carriage.visual(
            Box((0.285, 0.045, 0.040)),
            origin=Origin(xyz=(0.0, 0.055, z)),
            material=anodized,
            name=f"z_frame_bridge_{i}",
        )
    y_carriage.visual(
        Box((0.120, 0.006, 0.260)),
        origin=Origin(xyz=(0.0, 0.030, 0.245)),
        material=black_shadow,
        name="z_cable_window",
    )
    y_carriage.visual(
        Box((0.025, 0.025, 0.320)),
        origin=Origin(xyz=(-0.065, 0.020, 0.240)),
        material=rail_steel,
        name="z_rail_0",
    )
    y_carriage.visual(
        Box((0.025, 0.025, 0.320)),
        origin=Origin(xyz=(0.065, 0.020, 0.240)),
        material=rail_steel,
        name="z_rail_1",
    )

    z_carriage = model.part("z_carriage")
    z_carriage.visual(
        Box((0.205, 0.045, 0.120)),
        origin=Origin(xyz=(0.0, -0.015, 0.060)),
        material=carriage_blue,
        name="z_slide_block",
    )
    z_carriage.visual(
        Box((0.160, 0.020, 0.160)),
        origin=Origin(xyz=(0.0, -0.047, 0.060)),
        material=plate_gray,
        name="end_effector_plate",
    )
    z_carriage.visual(
        Box((0.070, 0.004, 0.070)),
        origin=Origin(xyz=(0.0, -0.058, 0.060)),
        material=black_shadow,
        name="tool_clearance_square",
    )
    for i, (x, z) in enumerate(((-0.055, 0.005), (0.055, 0.005), (-0.055, 0.115), (0.055, 0.115))):
        z_carriage.visual(
            Cylinder(radius=0.008, length=0.008),
            origin=Origin(xyz=(x, -0.061, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rail_steel,
            name=f"plate_bolt_{i}",
        )

    model.articulation(
        "base_to_x",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_carriage,
        origin=Origin(xyz=(-0.250, 0.0, 0.076)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.50),
    )
    model.articulation(
        "x_to_y",
        ArticulationType.PRISMATIC,
        parent=x_carriage,
        child=y_carriage,
        origin=Origin(xyz=(0.0, -0.100, 0.110)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.30, lower=0.0, upper=0.20),
    )
    model.articulation(
        "y_to_z",
        ArticulationType.PRISMATIC,
        parent=y_carriage,
        child=z_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.20, lower=0.0, upper=0.22),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    x_carriage = object_model.get_part("x_carriage")
    y_carriage = object_model.get_part("y_carriage")
    z_carriage = object_model.get_part("z_carriage")
    base_to_x = object_model.get_articulation("base_to_x")
    x_to_y = object_model.get_articulation("x_to_y")
    y_to_z = object_model.get_articulation("y_to_z")

    ctx.expect_gap(
        x_carriage,
        base,
        axis="z",
        positive_elem="x_bearing_shoe_0",
        negative_elem="x_rail_0",
        max_gap=0.002,
        max_penetration=0.0,
        name="x carriage bears on lower rail",
    )
    ctx.expect_gap(
        y_carriage,
        x_carriage,
        axis="z",
        positive_elem="y_bearing_shoe_0",
        negative_elem="y_rail_0",
        max_gap=0.002,
        max_penetration=0.0,
        name="y carriage bears on middle rail",
    )
    ctx.expect_gap(
        y_carriage,
        z_carriage,
        axis="y",
        positive_elem="z_rail_0",
        negative_elem="z_slide_block",
        max_gap=0.002,
        max_penetration=0.0,
        name="z carriage bears against vertical rail",
    )

    plate_aabb = ctx.part_element_world_aabb(z_carriage, elem="end_effector_plate")
    if plate_aabb is None:
        ctx.fail("end-effector plate exists", "missing end_effector_plate visual")
    else:
        plate_min, plate_max = plate_aabb
        dx = plate_max[0] - plate_min[0]
        dz = plate_max[2] - plate_min[2]
        ctx.check(
            "end-effector plate is square",
            abs(dx - dz) < 0.002 and dx > 0.12,
            details=f"dx={dx:.4f}, dz={dz:.4f}",
        )

    rest_pos = {
        "x": ctx.part_world_position(x_carriage),
        "y": ctx.part_world_position(y_carriage),
        "z": ctx.part_world_position(z_carriage),
    }
    with ctx.pose({base_to_x: 0.50, x_to_y: 0.20, y_to_z: 0.22}):
        extended_pos = {
            "x": ctx.part_world_position(x_carriage),
            "y": ctx.part_world_position(y_carriage),
            "z": ctx.part_world_position(z_carriage),
        }
    ctx.check(
        "three linear axes move independently positive",
        rest_pos["x"] is not None
        and rest_pos["y"] is not None
        and rest_pos["z"] is not None
        and extended_pos["x"] is not None
        and extended_pos["y"] is not None
        and extended_pos["z"] is not None
        and extended_pos["x"][0] > rest_pos["x"][0] + 0.45
        and extended_pos["y"][1] > rest_pos["y"][1] + 0.17
        and extended_pos["z"][2] > rest_pos["z"][2] + 0.19,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
