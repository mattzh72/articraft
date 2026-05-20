from __future__ import annotations

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
    model = ArticulatedObject(name="three_axis_cartesian_transfer_stage")

    model.material("cast_black", rgba=(0.03, 0.035, 0.04, 1.0))
    model.material("machined_steel", rgba=(0.70, 0.72, 0.70, 1.0))
    model.material("brushed_aluminum", rgba=(0.46, 0.50, 0.54, 1.0))
    model.material("blue_anodized", rgba=(0.04, 0.20, 0.55, 1.0))
    model.material("dark_rubber", rgba=(0.01, 0.012, 0.012, 1.0))
    model.material("tool_steel", rgba=(0.18, 0.18, 0.17, 1.0))

    base = model.part("base_rail")
    base.visual(
        Box((1.25, 0.32, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material="cast_black",
        name="base_plinth",
    )
    for name, y in (("base_rail_0", -0.085), ("base_rail_1", 0.085)):
        base.visual(
            Box((1.12, 0.035, 0.035)),
            origin=Origin(xyz=(0.0, y, 0.095)),
            material="machined_steel",
            name=name,
        )
    for index, x in enumerate((-0.575, 0.575)):
        base.visual(
            Box((0.045, 0.25, 0.095)),
            origin=Origin(xyz=(x, 0.0, 0.127)),
            material="cast_black",
            name=f"x_stop_{index}",
        )

    x_carriage = model.part("x_carriage")
    for name, y in (("x_bearing_0", -0.085), ("x_bearing_1", 0.085)):
        x_carriage.visual(
            Box((0.22, 0.055, 0.018)),
            origin=Origin(xyz=(0.0, y, 0.009)),
            material="dark_rubber",
            name=name,
        )
    x_carriage.visual(
        Box((0.32, 0.27, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material="blue_anodized",
        name="x_saddle",
    )
    for name, x in (("y_rail_0", -0.075), ("y_rail_1", 0.075)):
        x_carriage.visual(
            Box((0.028, 0.44, 0.033)),
            origin=Origin(xyz=(x, 0.0, 0.0665)),
            material="machined_steel",
            name=name,
        )

    y_slide = model.part("y_slide")
    for name, x in (("y_bearing_0", -0.075), ("y_bearing_1", 0.075)):
        y_slide.visual(
            Box((0.055, 0.10, 0.018)),
            origin=Origin(xyz=(x, 0.0, 0.009)),
            material="dark_rubber",
            name=name,
        )
    y_slide.visual(
        Box((0.26, 0.22, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material="brushed_aluminum",
        name="y_saddle",
    )
    y_slide.visual(
        Box((0.18, 0.16, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.063)),
        material="blue_anodized",
        name="column_foot",
    )
    y_slide.visual(
        Box((0.070, 0.10, 0.340)),
        origin=Origin(xyz=(-0.035, 0.0, 0.247)),
        material="brushed_aluminum",
        name="z_column",
    )
    for name, y in (("z_guide_0", -0.035), ("z_guide_1", 0.035)):
        y_slide.visual(
            Box((0.014, 0.018, 0.285)),
            origin=Origin(xyz=(0.005, y, 0.235)),
            material="machined_steel",
            name=name,
        )

    vertical_head = model.part("vertical_head")
    vertical_head.visual(
        Box((0.100, 0.140, 0.120)),
        origin=Origin(xyz=(0.050, 0.0, 0.0)),
        material="blue_anodized",
        name="head_body",
    )
    vertical_head.visual(
        Box((0.018, 0.120, 0.100)),
        origin=Origin(xyz=(0.109, 0.0, 0.0)),
        material="brushed_aluminum",
        name="front_plate",
    )
    vertical_head.visual(
        Box((0.055, 0.070, 0.035)),
        origin=Origin(xyz=(0.120, 0.0, -0.052)),
        material="brushed_aluminum",
        name="spindle_clamp",
    )
    vertical_head.visual(
        Cylinder(radius=0.025, length=0.080),
        origin=Origin(xyz=(0.130, 0.0, -0.078)),
        material="tool_steel",
        name="tool_nose",
    )

    model.articulation(
        "base_to_x_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_carriage,
        origin=Origin(xyz=(-0.30, 0.0, 0.1125)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=450.0, velocity=0.45, lower=0.0, upper=0.52),
    )
    model.articulation(
        "x_carriage_to_y_slide",
        ArticulationType.PRISMATIC,
        parent=x_carriage,
        child=y_slide,
        origin=Origin(xyz=(0.0, -0.12, 0.083)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=260.0, velocity=0.35, lower=0.0, upper=0.24),
    )
    model.articulation(
        "y_slide_to_vertical_head",
        ArticulationType.PRISMATIC,
        parent=y_slide,
        child=vertical_head,
        origin=Origin(xyz=(0.012, 0.0, 0.17)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.25, lower=0.0, upper=0.16),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_rail")
    x_carriage = object_model.get_part("x_carriage")
    y_slide = object_model.get_part("y_slide")
    vertical_head = object_model.get_part("vertical_head")
    x_joint = object_model.get_articulation("base_to_x_carriage")
    y_joint = object_model.get_articulation("x_carriage_to_y_slide")
    z_joint = object_model.get_articulation("y_slide_to_vertical_head")

    prismatic_joints = [x_joint, y_joint, z_joint]
    ctx.check(
        "three prismatic axes",
        len(prismatic_joints) == 3
        and all(j.articulation_type == ArticulationType.PRISMATIC for j in prismatic_joints)
        and x_joint.axis == (1.0, 0.0, 0.0)
        and y_joint.axis == (0.0, 1.0, 0.0)
        and z_joint.axis == (0.0, 0.0, 1.0),
        details=f"axes={[j.axis for j in prismatic_joints]}",
    )

    ctx.expect_gap(
        x_carriage,
        base,
        axis="z",
        positive_elem="x_bearing_0",
        negative_elem="base_rail_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="x carriage rides on fixed rail",
    )
    ctx.expect_gap(
        y_slide,
        x_carriage,
        axis="z",
        positive_elem="y_bearing_0",
        negative_elem="y_rail_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="y slide rides on x carriage rail",
    )
    ctx.expect_gap(
        vertical_head,
        y_slide,
        axis="x",
        positive_elem="head_body",
        negative_elem="z_guide_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="vertical head runs on z guide face",
    )

    rest_x = ctx.part_world_position(x_carriage)
    rest_y = ctx.part_world_position(y_slide)
    rest_z = ctx.part_world_position(vertical_head)
    with ctx.pose({x_joint: 0.52, y_joint: 0.24, z_joint: 0.16}):
        ctx.expect_overlap(
            x_carriage,
            base,
            axes="xy",
            elem_a="x_bearing_0",
            elem_b="base_rail_0",
            min_overlap=0.03,
            name="x bearing remains on base rail at travel",
        )
        ctx.expect_overlap(
            y_slide,
            x_carriage,
            axes="xy",
            elem_a="y_bearing_0",
            elem_b="y_rail_0",
            min_overlap=0.025,
            name="y bearing remains on cross rail at travel",
        )
        ctx.expect_overlap(
            vertical_head,
            y_slide,
            axes="yz",
            elem_a="head_body",
            elem_b="z_guide_0",
            min_overlap=0.015,
            name="z head remains engaged with vertical guide",
        )
        travel_x = ctx.part_world_position(x_carriage)
        travel_y = ctx.part_world_position(y_slide)
        travel_z = ctx.part_world_position(vertical_head)

    ctx.check(
        "positive travel directions",
        rest_x is not None
        and rest_y is not None
        and rest_z is not None
        and travel_x is not None
        and travel_y is not None
        and travel_z is not None
        and travel_x[0] > rest_x[0] + 0.45
        and travel_y[1] > rest_y[1] + 0.20
        and travel_z[2] > rest_z[2] + 0.12,
        details=f"rest={(rest_x, rest_y, rest_z)}, travel={(travel_x, travel_y, travel_z)}",
    )

    return ctx.report()


object_model = build_object_model()
