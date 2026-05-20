from __future__ import annotations

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
    model = ArticulatedObject(name="linear_elbow_positioner")

    aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.73, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.12, 0.13, 0.14, 1.0))
    black_cover = model.material("matte_black_cover", rgba=(0.015, 0.017, 0.018, 1.0))
    blue_anodized = model.material("blue_anodized", rgba=(0.08, 0.22, 0.42, 1.0))
    amber_pad = model.material("polymer_bearing_pad", rgba=(0.88, 0.66, 0.22, 1.0))

    base = model.part("base")
    base.visual(
        Box((1.34, 0.36, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=aluminum,
        name="base_plate",
    )
    for idx, y in enumerate((-0.105, 0.105)):
        base.visual(
            Box((1.16, 0.055, 0.010)),
            origin=Origin(xyz=(0.0, y, 0.040)),
            material=aluminum,
            name=f"rail_foot_{idx}",
        )
        base.visual(
            Box((1.12, 0.032, 0.030)),
            origin=Origin(xyz=(0.0, y, 0.060)),
            material=dark_steel,
            name="rail_0" if idx == 0 else "rail_1",
        )
    base.visual(
        Box((0.72, 0.060, 0.010)),
        origin=Origin(xyz=(-0.08, 0.0, 0.040)),
        material=aluminum,
        name="actuator_saddle",
    )
    base.visual(
        Box((0.72, 0.070, 0.035)),
        origin=Origin(xyz=(-0.08, 0.0, 0.0625)),
        material=black_cover,
        name="linear_actuator_cover",
    )
    for idx, x in enumerate((-0.60, 0.60)):
        base.visual(
            Box((0.035, 0.145, 0.060)),
            origin=Origin(xyz=(x, 0.0, 0.065)),
            material=dark_steel,
            name=f"end_stop_{idx}",
        )
    for idx, (x, y) in enumerate(
        (
            (-0.53, -0.155),
            (-0.53, 0.155),
            (0.53, -0.155),
            (0.53, 0.155),
            (-0.25, -0.155),
            (0.25, 0.155),
        )
    ):
        base.visual(
            Cylinder(radius=0.012, length=0.006),
            origin=Origin(xyz=(x, y, 0.038)),
            material=dark_steel,
            name=f"base_bolt_{idx}",
        )

    carriage = model.part("carriage")
    for idx, y in enumerate((-0.105, 0.105)):
        carriage.visual(
            Box((0.220, 0.060, 0.035)),
            origin=Origin(xyz=(0.0, y, 0.0175)),
            material=dark_steel,
            name="bearing_block_0" if idx == 0 else "bearing_block_1",
        )
    carriage.visual(
        Box((0.340, 0.285, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.049)),
        material=aluminum,
        name="carriage_plate",
    )
    carriage.visual(
        Cylinder(radius=0.075, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.076)),
        material=aluminum,
        name="rotary_pedestal",
    )
    carriage.visual(
        Box((0.180, 0.045, 0.032)),
        origin=Origin(xyz=(-0.115, -0.121, 0.079)),
        material=black_cover,
        name="carriage_actuator_cover",
    )
    for idx, (x, y) in enumerate(((-0.115, -0.080), (-0.115, 0.080), (0.115, -0.080), (0.115, 0.080))):
        carriage.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(x, y, 0.066)),
            material=dark_steel,
            name=f"carriage_bolt_{idx}",
        )

    elbow = model.part("elbow")
    elbow.visual(
        Cylinder(radius=0.085, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark_steel,
        name="rotary_disc",
    )
    elbow.visual(
        Cylinder(radius=0.050, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0475)),
        material=aluminum,
        name="central_hub",
    )
    for idx, y in enumerate((-0.052, 0.052)):
        elbow.visual(
            Box((0.620, 0.012, 0.054)),
            origin=Origin(xyz=(0.340, y, 0.047)),
            material=aluminum,
            name=f"side_plate_{idx}",
        )
    elbow.visual(
        Box((0.620, 0.104, 0.020)),
        origin=Origin(xyz=(0.340, 0.0, 0.010)),
        material=aluminum,
        name="bottom_strap",
    )
    elbow.visual(
        Box((0.620, 0.104, 0.014)),
        origin=Origin(xyz=(0.340, 0.0, 0.081)),
        material=aluminum,
        name="top_strap",
    )
    elbow.visual(
        Box((0.235, 0.066, 0.034)),
        origin=Origin(xyz=(0.225, 0.0, 0.105)),
        material=black_cover,
        name="elbow_actuator_cover",
    )
    elbow.visual(
        Box((0.090, 0.118, 0.016)),
        origin=Origin(xyz=(0.640, 0.0, 0.084)),
        material=dark_steel,
        name="nose_top_clamp",
    )
    elbow.visual(
        Box((0.090, 0.118, 0.016)),
        origin=Origin(xyz=(0.640, 0.0, 0.012)),
        material=dark_steel,
        name="nose_bottom_clamp",
    )
    for idx, y in enumerate((-0.050, 0.050)):
        elbow.visual(
            Box((0.090, 0.014, 0.046)),
            origin=Origin(xyz=(0.640, y, 0.049)),
            material=dark_steel,
            name=f"nose_side_clamp_{idx}",
        )

    extension = model.part("extension")
    extension.visual(
        Box((0.550, 0.064, 0.040)),
        origin=Origin(xyz=(0.225, 0.0, 0.050)),
        material=blue_anodized,
        name="inner_tube",
    )
    extension.visual(
        Box((0.140, 0.040, 0.004)),
        origin=Origin(xyz=(0.100, 0.0, 0.072)),
        material=amber_pad,
        name="top_wear_pad",
    )
    extension.visual(
        Box((0.140, 0.040, 0.010)),
        origin=Origin(xyz=(0.100, 0.0, 0.025)),
        material=amber_pad,
        name="bottom_wear_pad",
    )
    for idx, y in enumerate((-0.039, 0.039)):
        extension.visual(
            Box((0.140, 0.014, 0.030)),
            origin=Origin(xyz=(0.100, y, 0.050)),
            material=amber_pad,
            name=f"side_wear_pad_{idx}",
        )
    extension.visual(
        Box((0.055, 0.110, 0.080)),
        origin=Origin(xyz=(0.5275, 0.0, 0.050)),
        material=aluminum,
        name="tool_plate",
    )
    extension.visual(
        Cylinder(radius=0.032, length=0.035),
        origin=Origin(xyz=(0.570, 0.0, 0.050), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="tool_boss",
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(-0.30, 0.0, 0.075)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.45, lower=0.0, upper=0.65),
    )
    model.articulation(
        "carriage_to_elbow",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=elbow,
        origin=Origin(xyz=(0.0, 0.0, 0.089)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.4, lower=-1.57, upper=1.57),
    )
    model.articulation(
        "elbow_to_extension",
        ArticulationType.PRISMATIC,
        parent=elbow,
        child=extension,
        origin=Origin(xyz=(0.230, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.22),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    elbow = object_model.get_part("elbow")
    extension = object_model.get_part("extension")
    linear = object_model.get_articulation("base_to_carriage")
    rotary = object_model.get_articulation("carriage_to_elbow")
    telescope = object_model.get_articulation("elbow_to_extension")

    ctx.expect_contact(
        carriage,
        base,
        elem_a="bearing_block_0",
        elem_b="rail_0",
        name="lower carriage bearing sits on its rail",
    )
    ctx.expect_contact(
        carriage,
        base,
        elem_a="bearing_block_1",
        elem_b="rail_1",
        name="upper carriage bearing sits on its rail",
    )
    ctx.expect_gap(
        elbow,
        carriage,
        axis="z",
        positive_elem="rotary_disc",
        negative_elem="rotary_pedestal",
        max_gap=0.001,
        max_penetration=0.000001,
        name="rotary disc seats on carriage pedestal",
    )
    ctx.expect_within(
        extension,
        elbow,
        axes="yz",
        inner_elem="inner_tube",
        margin=0.001,
        name="telescoping tube stays inside sleeve cross-section",
    )
    ctx.expect_overlap(
        extension,
        elbow,
        axes="x",
        elem_a="inner_tube",
        elem_b="top_strap",
        min_overlap=0.20,
        name="telescoping tube is retained in the outer sleeve at rest",
    )
    ctx.expect_gap(
        elbow,
        extension,
        axis="z",
        positive_elem="top_strap",
        negative_elem="top_wear_pad",
        max_gap=0.001,
        max_penetration=0.0,
        name="top telescoping wear pad rides under sleeve",
    )
    ctx.expect_gap(
        extension,
        elbow,
        axis="z",
        positive_elem="bottom_wear_pad",
        negative_elem="bottom_strap",
        max_gap=0.001,
        max_penetration=0.000001,
        name="bottom telescoping wear pad rides on sleeve",
    )

    rest_carriage = ctx.part_world_position(carriage)
    with ctx.pose({linear: 0.65}):
        extended_carriage = ctx.part_world_position(carriage)
        ctx.expect_contact(
            carriage,
            base,
            elem_a="bearing_block_0",
            elem_b="rail_0",
            name="extended carriage bearing remains on its rail",
        )
    ctx.check(
        "linear carriage travels along base rails",
        rest_carriage is not None
        and extended_carriage is not None
        and extended_carriage[0] > rest_carriage[0] + 0.60,
        details=f"rest={rest_carriage}, extended={extended_carriage}",
    )

    rest_tool = ctx.part_element_world_aabb(extension, elem="tool_plate")
    with ctx.pose({rotary: 1.0}):
        swung_tool = ctx.part_element_world_aabb(extension, elem="tool_plate")
    ctx.check(
        "rotary elbow swings the telescoping stage",
        rest_tool is not None
        and swung_tool is not None
        and swung_tool[1][1] > rest_tool[1][1] + 0.25,
        details=f"rest_tool={rest_tool}, swung_tool={swung_tool}",
    )

    rest_extension = ctx.part_world_position(extension)
    with ctx.pose({telescope: 0.22}):
        extended_extension = ctx.part_world_position(extension)
        ctx.expect_overlap(
            extension,
            elbow,
            axes="x",
            elem_a="inner_tube",
            elem_b="top_strap",
            min_overlap=0.20,
            name="telescoping tube remains captured at full extension",
        )
        ctx.expect_gap(
            elbow,
            extension,
            axis="z",
            positive_elem="top_strap",
            negative_elem="top_wear_pad",
            max_gap=0.001,
            max_penetration=0.0,
            name="extended top wear pad remains supported",
        )
    ctx.check(
        "final telescope extends away from elbow",
        rest_extension is not None
        and extended_extension is not None
        and extended_extension[0] > rest_extension[0] + 0.20,
        details=f"rest={rest_extension}, extended={extended_extension}",
    )

    return ctx.report()


object_model = build_object_model()
