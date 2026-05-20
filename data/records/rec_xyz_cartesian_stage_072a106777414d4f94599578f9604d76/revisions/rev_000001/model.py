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


X_TRAVEL = 0.34
Y_TRAVEL = 0.16
Z_TRAVEL = 0.16


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="xyz_cartesian_stage")

    model.material("painted_cast_iron", rgba=(0.18, 0.21, 0.24, 1.0))
    model.material("brushed_aluminum", rgba=(0.70, 0.72, 0.74, 1.0))
    model.material("hardened_steel", rgba=(0.55, 0.58, 0.62, 1.0))
    model.material("black_bellows", rgba=(0.03, 0.03, 0.035, 1.0))
    model.material("dark_fasteners", rgba=(0.01, 0.01, 0.012, 1.0))

    base = model.part("base")
    base.visual(
        Box((1.24, 0.42, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material="painted_cast_iron",
        name="base_plate",
    )
    for x in (-0.50, 0.50):
        for y in (-0.155, 0.155):
            base.visual(
                Box((0.11, 0.07, 0.025)),
                origin=Origin(xyz=(x, y, -0.0115)),
                material="dark_fasteners",
                name=f"leveling_foot_{x}_{y}",
            )

    for y, rail_name in ((-0.13, "x_rail_rear"), (0.13, "x_rail_front")):
        base.visual(
            Box((1.06, 0.035, 0.037)),
            origin=Origin(xyz=(0.0, y, 0.0975)),
            material="hardened_steel",
            name=rail_name,
        )
    for x in (-0.54, 0.54):
        base.visual(
            Box((0.035, 0.31, 0.045)),
            origin=Origin(xyz=(x, 0.0, 0.102)),
            material="painted_cast_iron",
            name=f"x_end_stop_{'low' if x < 0 else 'high'}",
        )
    base.visual(
        Cylinder(radius=0.011, length=0.98),
        origin=Origin(xyz=(0.0, 0.0, 0.123), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="hardened_steel",
        name="x_ball_screw",
    )
    for x in (-0.50, 0.50):
        base.visual(
            Box((0.048, 0.065, 0.062)),
            origin=Origin(xyz=(x, 0.0, 0.111)),
            material="brushed_aluminum",
            name=f"x_screw_bearing_{'low' if x < 0 else 'high'}",
        )

    x_carriage = model.part("x_carriage")
    for x, y, pad_name in (
        (-0.095, -0.13, "x_bearing_pad_-0.095_-0.13"),
        (-0.095, 0.13, "x_bearing_pad_-0.095_0.13"),
        (0.095, -0.13, "x_bearing_pad_0.095_-0.13"),
        (0.095, 0.13, "x_bearing_pad_0.095_0.13"),
    ):
        x_carriage.visual(
            Box((0.105, 0.058, 0.025)),
            origin=Origin(xyz=(x, y, 0.0125)),
            material="hardened_steel",
            name=pad_name,
        )
    x_carriage.visual(
        Box((0.30, 0.34, 0.042)),
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        material="brushed_aluminum",
        name="x_saddle_plate",
    )
    x_carriage.visual(
        Box((0.23, 0.58, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0835)),
        material="brushed_aluminum",
        name="y_slide_bed",
    )
    for x, rail_name in ((-0.07, "y_rail_side_0"), (0.07, "y_rail_side_1")):
        x_carriage.visual(
            Box((0.035, 0.49, 0.025)),
            origin=Origin(xyz=(x, 0.0, 0.1135)),
            material="hardened_steel",
            name=rail_name,
        )
    for y in (-0.265, 0.265):
        x_carriage.visual(
            Box((0.20, 0.030, 0.045)),
            origin=Origin(xyz=(0.0, y, 0.104)),
            material="painted_cast_iron",
            name=f"y_end_stop_{'low' if y < 0 else 'high'}",
        )
    x_carriage.visual(
        Cylinder(radius=0.008, length=0.43),
        origin=Origin(xyz=(0.0, 0.0, 0.134), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="hardened_steel",
        name="y_ball_screw",
    )
    x_carriage.visual(
        Box((0.060, 0.080, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.114)),
        material="brushed_aluminum",
        name="y_screw_nut",
    )

    y_carriage = model.part("y_carriage")
    for x, y, pad_name in (
        (-0.07, -0.085, "y_bearing_pad_-0.07_-0.085"),
        (-0.07, 0.085, "y_bearing_pad_-0.07_0.085"),
        (0.07, -0.085, "y_bearing_pad_0.07_-0.085"),
        (0.07, 0.085, "y_bearing_pad_0.07_0.085"),
    ):
        y_carriage.visual(
            Box((0.058, 0.105, 0.022)),
            origin=Origin(xyz=(x, y, 0.011)),
            material="hardened_steel",
            name=pad_name,
        )
    y_carriage.visual(
        Box((0.215, 0.195, 0.043)),
        origin=Origin(xyz=(0.0, 0.0, 0.0435)),
        material="brushed_aluminum",
        name="y_saddle_plate",
    )
    y_carriage.visual(
        Box((0.185, 0.040, 0.300)),
        origin=Origin(xyz=(0.0, -0.0575, 0.215)),
        material="painted_cast_iron",
        name="z_back_plate",
    )
    y_carriage.visual(
        Box((0.205, 0.035, 0.050)),
        origin=Origin(xyz=(0.0, -0.060, 0.090)),
        material="brushed_aluminum",
        name="z_lower_mount",
    )
    y_carriage.visual(
        Box((0.205, 0.075, 0.045)),
        origin=Origin(xyz=(0.0, -0.040, 0.3425)),
        material="brushed_aluminum",
        name="z_top_cap",
    )
    for x, rail_name in ((-0.055, "z_rail_side_0"), (0.055, "z_rail_side_1")):
        y_carriage.visual(
            Box((0.025, 0.020, 0.240)),
            origin=Origin(xyz=(x, -0.0275, 0.205)),
            material="hardened_steel",
            name=rail_name,
        )
    y_carriage.visual(
        Cylinder(radius=0.007, length=0.235),
        origin=Origin(xyz=(0.0, -0.060, 0.205)),
        material="hardened_steel",
        name="z_ball_screw",
    )

    z_carriage = model.part("z_carriage")
    z_carriage.visual(
        Box((0.150, 0.035, 0.110)),
        origin=Origin(xyz=(0.0, 0.0175, 0.055)),
        material="brushed_aluminum",
        name="z_slider_block",
    )
    z_carriage.visual(
        Box((0.220, 0.160, 0.025)),
        origin=Origin(xyz=(0.0, 0.100, 0.1225)),
        material="brushed_aluminum",
        name="tool_plate",
    )
    z_carriage.visual(
        Box((0.165, 0.024, 0.028)),
        origin=Origin(xyz=(0.0, 0.0175, -0.014)),
        material="black_bellows",
        name="lower_wiper",
    )
    for x in (-0.070, 0.070):
        for y in (0.050, 0.150):
            z_carriage.visual(
                Cylinder(radius=0.011, length=0.004),
                origin=Origin(xyz=(x, y, 0.137)),
                material="dark_fasteners",
                name=f"tool_hole_{x}_{y}",
            )

    model.articulation(
        "base_to_x_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.116)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-X_TRAVEL, upper=X_TRAVEL, effort=800.0, velocity=0.35),
    )
    model.articulation(
        "x_carriage_to_y_carriage",
        ArticulationType.PRISMATIC,
        parent=x_carriage,
        child=y_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.126)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-Y_TRAVEL, upper=Y_TRAVEL, effort=450.0, velocity=0.30),
    )
    model.articulation(
        "y_carriage_to_z_carriage",
        ArticulationType.PRISMATIC,
        parent=y_carriage,
        child=z_carriage,
        origin=Origin(xyz=(0.0, -0.0175, 0.105)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=Z_TRAVEL, effort=350.0, velocity=0.22),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    x_carriage = object_model.get_part("x_carriage")
    y_carriage = object_model.get_part("y_carriage")
    z_carriage = object_model.get_part("z_carriage")
    x_joint = object_model.get_articulation("base_to_x_carriage")
    y_joint = object_model.get_articulation("x_carriage_to_y_carriage")
    z_joint = object_model.get_articulation("y_carriage_to_z_carriage")

    ctx.check(
        "three orthogonal prismatic stage joints",
        x_joint.articulation_type == ArticulationType.PRISMATIC
        and y_joint.articulation_type == ArticulationType.PRISMATIC
        and z_joint.articulation_type == ArticulationType.PRISMATIC
        and x_joint.axis == (1.0, 0.0, 0.0)
        and y_joint.axis == (0.0, 1.0, 0.0)
        and z_joint.axis == (0.0, 0.0, 1.0),
        details=f"axes: X={x_joint.axis}, Y={y_joint.axis}, Z={z_joint.axis}",
    )

    ctx.expect_contact(
        x_carriage,
        base,
        elem_a="x_bearing_pad_-0.095_-0.13",
        elem_b="x_rail_rear",
        name="X carriage rides on rear base rail",
    )
    ctx.expect_contact(
        y_carriage,
        x_carriage,
        elem_a="y_bearing_pad_-0.07_-0.085",
        elem_b="y_rail_side_0",
        name="Y carriage rides on X-carriage rail",
    )
    ctx.expect_contact(
        z_carriage,
        y_carriage,
        elem_a="z_slider_block",
        elem_b="z_rail_side_0",
        name="Z slider bears on vertical guide rail",
    )

    rest_x = ctx.part_world_position(x_carriage)
    rest_y = ctx.part_world_position(y_carriage)
    rest_z = ctx.part_world_position(z_carriage)
    with ctx.pose({x_joint: X_TRAVEL, y_joint: Y_TRAVEL, z_joint: Z_TRAVEL}):
        moved_x = ctx.part_world_position(x_carriage)
        moved_y = ctx.part_world_position(y_carriage)
        moved_z = ctx.part_world_position(z_carriage)

    ctx.check(
        "upper limits move stages along X Y and Z",
        rest_x is not None
        and rest_y is not None
        and rest_z is not None
        and moved_x is not None
        and moved_y is not None
        and moved_z is not None
        and moved_x[0] > rest_x[0] + 0.30
        and moved_y[1] > rest_y[1] + 0.14
        and moved_z[2] > rest_z[2] + 0.14,
        details=f"rest={(rest_x, rest_y, rest_z)}, moved={(moved_x, moved_y, moved_z)}",
    )

    return ctx.report()


object_model = build_object_model()
