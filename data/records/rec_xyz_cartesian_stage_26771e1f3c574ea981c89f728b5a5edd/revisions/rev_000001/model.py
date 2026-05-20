from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="wall_backed_xyz_positioning_assembly")

    dark = Material("matte_dark_frame", rgba=(0.08, 0.09, 0.10, 1.0))
    wall_mat = Material("powder_coated_wall_plate", rgba=(0.18, 0.20, 0.22, 1.0))
    rail_mat = Material("brushed_steel_rails", rgba=(0.72, 0.74, 0.72, 1.0))
    blue = Material("blue_machined_carriages", rgba=(0.05, 0.24, 0.55, 1.0))
    orange = Material("orange_tool_mount", rgba=(0.92, 0.36, 0.06, 1.0))
    black = Material("black_bearing_blocks", rgba=(0.015, 0.017, 0.018, 1.0))

    support = model.part("support")
    support.visual(
        Box((1.35, 0.050, 1.15)),
        origin=Origin(xyz=(0.0, -0.030, 0.575)),
        material=wall_mat,
        name="wall_plate",
    )
    support.visual(
        Box((1.22, 0.18, 0.040)),
        origin=Origin(xyz=(0.0, 0.040, 0.020)),
        material=dark,
        name="bottom_shelf",
    )
    for z, rail_name in ((0.56, "upper_x_rail"), (0.38, "lower_x_rail")):
        support.visual(
            Cylinder(radius=0.012, length=1.16),
            origin=Origin(xyz=(0.0, 0.026, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=rail_mat,
            name=rail_name,
        )
        for i, x in enumerate((-0.48, 0.0, 0.48)):
            support.visual(
                Box((0.050, 0.066, 0.050)),
                origin=Origin(xyz=(x, 0.006, z)),
                material=dark,
                name=f"{rail_name}_stand_{i}",
            )

    for x in (-0.57, 0.57):
        support.visual(
            Box((0.038, 0.070, 0.92)),
            origin=Origin(xyz=(x, -0.006, 0.54)),
            material=dark,
            name=f"side_upright_{0 if x < 0 else 1}",
        )

    for x in (-0.60, 0.60):
        for z in (0.16, 0.98):
            support.visual(
                Cylinder(radius=0.018, length=0.009),
                origin=Origin(xyz=(x, -0.001, z), rpy=(-pi / 2.0, 0.0, 0.0)),
                material=rail_mat,
                name=f"wall_bolt_{x}_{z}",
            )

    x_stage = model.part("x_stage")
    x_stage.visual(
        Box((0.260, 0.045, 0.340)),
        origin=Origin(xyz=(0.0, 0.100, 0.470)),
        material=blue,
        name="x_backplate",
    )
    x_stage.visual(
        Box((0.180, 0.028, 0.250)),
        origin=Origin(xyz=(0.0, 0.068, 0.470)),
        material=black,
        name="x_bearing_web",
    )
    for z, shoe_name in ((0.56, "upper_x_shoe"), (0.38, "lower_x_shoe")):
        x_stage.visual(
            Box((0.220, 0.028, 0.056)),
            origin=Origin(xyz=(0.0, 0.052, z)),
            material=black,
            name=shoe_name,
        )
    x_stage.visual(
        Box((0.240, 0.080, 0.150)),
        origin=Origin(xyz=(0.0, 0.110, 0.620)),
        material=blue,
        name="upper_riser",
    )
    x_stage.visual(
        Box((0.320, 0.430, 0.035)),
        origin=Origin(xyz=(0.0, 0.235, 0.660)),
        material=blue,
        name="y_axis_deck",
    )
    for x, rail_name in ((-0.080, "y_rail_0"), (0.080, "y_rail_1")):
        x_stage.visual(
            Cylinder(radius=0.012, length=0.420),
            origin=Origin(xyz=(x, 0.235, 0.689), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=rail_mat,
            name=rail_name,
        )

    y_stage = model.part("y_stage")
    y_stage.visual(
        Box((0.220, 0.160, 0.035)),
        origin=Origin(xyz=(0.0, 0.140, 0.735)),
        material=blue,
        name="y_top_plate",
    )
    for x, shoe_name in ((-0.080, "y_shoe_0"), (0.080, "y_shoe_1")):
        y_stage.visual(
            Box((0.052, 0.105, 0.020)),
            origin=Origin(xyz=(x, 0.140, 0.711)),
            material=black,
            name=shoe_name,
        )
    y_stage.visual(
        Box((0.220, 0.035, 0.420)),
        origin=Origin(xyz=(0.0, 0.205, 0.920)),
        material=blue,
        name="z_upright_plate",
    )
    y_stage.visual(
        Box((0.250, 0.050, 0.040)),
        origin=Origin(xyz=(0.0, 0.205, 1.120)),
        material=blue,
        name="z_top_cap",
    )
    y_stage.visual(
        Box((0.250, 0.050, 0.040)),
        origin=Origin(xyz=(0.0, 0.205, 0.720)),
        material=blue,
        name="z_bottom_cap",
    )
    for x, rail_name in ((-0.070, "z_rail_0"), (0.070, "z_rail_1")):
        y_stage.visual(
            Box((0.040, 0.035, 0.390)),
            origin=Origin(xyz=(x, 0.225, 0.910)),
            material=dark,
            name=f"{rail_name}_seat",
        )
        y_stage.visual(
            Cylinder(radius=0.010, length=0.360),
            origin=Origin(xyz=(x, 0.238, 0.910)),
            material=rail_mat,
            name=rail_name,
        )

    z_carriage = model.part("z_carriage")
    for x, shoe_name in ((-0.070, "z_shoe_0"), (0.070, "z_shoe_1")):
        z_carriage.visual(
            Box((0.045, 0.026, 0.105)),
            origin=Origin(xyz=(x, 0.261, 0.790)),
            material=black,
            name=shoe_name,
        )
    z_carriage.visual(
        Box((0.110, 0.020, 0.130)),
        origin=Origin(xyz=(0.0, 0.278, 0.790)),
        material=black,
        name="z_shoe_bridge",
    )
    z_carriage.visual(
        Box((0.180, 0.038, 0.150)),
        origin=Origin(xyz=(0.0, 0.300, 0.790)),
        material=orange,
        name="z_face_plate",
    )
    z_carriage.visual(
        Box((0.100, 0.040, 0.080)),
        origin=Origin(xyz=(0.0, 0.335, 0.790)),
        material=orange,
        name="tool_mount_pad",
    )

    model.articulation(
        "support_to_x_stage",
        ArticulationType.PRISMATIC,
        parent=support,
        child=x_stage,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.35, lower=-0.40, upper=0.40),
    )
    model.articulation(
        "x_stage_to_y_stage",
        ArticulationType.PRISMATIC,
        parent=x_stage,
        child=y_stage,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.25, lower=0.0, upper=0.19),
    )
    model.articulation(
        "y_stage_to_z_carriage",
        ArticulationType.PRISMATIC,
        parent=y_stage,
        child=z_carriage,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.22, lower=0.0, upper=0.22),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    support = object_model.get_part("support")
    x_stage = object_model.get_part("x_stage")
    y_stage = object_model.get_part("y_stage")
    z_carriage = object_model.get_part("z_carriage")
    x_joint = object_model.get_articulation("support_to_x_stage")
    y_joint = object_model.get_articulation("x_stage_to_y_stage")
    z_joint = object_model.get_articulation("y_stage_to_z_carriage")

    prismatic_joints = [
        joint
        for joint in object_model.articulations
        if joint.articulation_type == ArticulationType.PRISMATIC
    ]
    ctx.check(
        "three stacked prismatic axes",
        len(prismatic_joints) == 3,
        details=f"found {len(prismatic_joints)} prismatic joints",
    )
    ctx.expect_overlap(
        x_stage,
        support,
        axes="x",
        min_overlap=0.18,
        elem_a="upper_x_shoe",
        elem_b="upper_x_rail",
        name="x carriage shoe remains over fixed rail",
    )
    ctx.expect_overlap(
        y_stage,
        x_stage,
        axes="y",
        min_overlap=0.10,
        elem_a="y_shoe_0",
        elem_b="y_rail_0",
        name="y carriage shoe remains over deck rail",
    )
    ctx.expect_overlap(
        z_carriage,
        y_stage,
        axes="z",
        min_overlap=0.09,
        elem_a="z_shoe_0",
        elem_b="z_rail_0",
        name="vertical carriage shoe remains on guide rail",
    )

    rest_x = ctx.part_world_position(x_stage)
    rest_y = ctx.part_world_position(y_stage)
    rest_z = ctx.part_world_position(z_carriage)
    with ctx.pose({x_joint: 0.40, y_joint: 0.19, z_joint: 0.22}):
        max_x = ctx.part_world_position(x_stage)
        max_y = ctx.part_world_position(y_stage)
        max_z = ctx.part_world_position(z_carriage)
        ctx.expect_overlap(
            x_stage,
            support,
            axes="x",
            min_overlap=0.06,
            elem_a="upper_x_shoe",
            elem_b="upper_x_rail",
            name="x axis retains rail engagement at travel end",
        )
        ctx.expect_overlap(
            y_stage,
            x_stage,
            axes="y",
            min_overlap=0.06,
            elem_a="y_shoe_0",
            elem_b="y_rail_0",
            name="y axis retains rail engagement at travel end",
        )
        ctx.expect_overlap(
            z_carriage,
            y_stage,
            axes="z",
            min_overlap=0.08,
            elem_a="z_shoe_0",
            elem_b="z_rail_0",
            name="z axis retains rail engagement at travel end",
        )

    ctx.check(
        "positive commands move along x y z",
        rest_x is not None
        and rest_y is not None
        and rest_z is not None
        and max_x is not None
        and max_y is not None
        and max_z is not None
        and max_x[0] > rest_x[0] + 0.35
        and max_y[1] > rest_y[1] + 0.16
        and max_z[2] > rest_z[2] + 0.19,
        details=f"rest=({rest_x}, {rest_y}, {rest_z}), max=({max_x}, {max_y}, {max_z})",
    )

    return ctx.report()


object_model = build_object_model()
