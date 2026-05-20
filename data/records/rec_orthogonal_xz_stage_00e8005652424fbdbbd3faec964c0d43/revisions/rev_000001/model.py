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
    model = ArticulatedObject(name="bench_xz_transfer_stage")

    cast_gray = model.material("cast_gray", rgba=(0.18, 0.19, 0.20, 1.0))
    black = model.material("black_oxide", rgba=(0.015, 0.015, 0.014, 1.0))
    rail_steel = model.material("ground_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    carriage_aluminum = model.material("carriage_aluminum", rgba=(0.78, 0.80, 0.78, 1.0))
    mast_blue = model.material("blue_anodized", rgba=(0.05, 0.14, 0.42, 1.0))

    base_rail = model.part("base_rail")
    base_rail.visual(
        Box((0.90, 0.20, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=cast_gray,
        name="base_plate",
    )
    base_rail.visual(
        Box((0.78, 0.055, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0525)),
        material=rail_steel,
        name="center_rail",
    )
    base_rail.visual(
        Box((0.78, 0.040, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.073)),
        material=rail_steel,
        name="rail_cap",
    )
    for x in (-0.425, 0.425):
        base_rail.visual(
            Box((0.035, 0.17, 0.055)),
            origin=Origin(xyz=(x, 0.0, 0.0625)),
            material=black,
            name=f"end_stop_{0 if x < 0 else 1}",
        )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.18, 0.14, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=carriage_aluminum,
        name="saddle_plate",
    )
    for y in (-0.049, 0.049):
        carriage.visual(
            Box((0.15, 0.032, 0.034)),
            origin=Origin(xyz=(0.0, y, -0.014)),
            material=carriage_aluminum,
            name=f"side_shoe_{0 if y < 0 else 1}",
        )
    for x in (-0.055, 0.055):
        carriage.visual(
            Cylinder(radius=0.009, length=0.006),
            origin=Origin(xyz=(x, -0.066, 0.050), rpy=(1.5708, 0.0, 0.0)),
            material=black,
            name=f"saddle_bolt_{0 if x < 0 else 1}",
        )

    mast = model.part("mast")
    mast.visual(
        Box((0.13, 0.12, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=black,
        name="foot_plate",
    )
    mast.visual(
        Box((0.060, 0.065, 0.360)),
        origin=Origin(xyz=(0.0, 0.025, 0.200)),
        material=mast_blue,
        name="upright_column",
    )
    mast.visual(
        Box((0.012, 0.012, 0.320)),
        origin=Origin(xyz=(-0.021, -0.013, 0.200)),
        material=rail_steel,
        name="front_rail_0",
    )
    mast.visual(
        Box((0.012, 0.012, 0.320)),
        origin=Origin(xyz=(0.021, -0.013, 0.200)),
        material=rail_steel,
        name="front_rail_1",
    )
    mast.visual(
        Box((0.080, 0.080, 0.025)),
        origin=Origin(xyz=(0.0, 0.025, 0.3925)),
        material=black,
        name="top_stop",
    )

    vertical_head = model.part("vertical_head")
    vertical_head.visual(
        Box((0.105, 0.048, 0.090)),
        origin=Origin(xyz=(0.0, -0.024, 0.045)),
        material=carriage_aluminum,
        name="head_body",
    )
    vertical_head.visual(
        Box((0.070, 0.035, 0.045)),
        origin=Origin(xyz=(0.0, -0.065, 0.045)),
        material=black,
        name="front_tool_plate",
    )
    for x in (-0.032, 0.032):
        for z in (0.028, 0.062):
            vertical_head.visual(
                Cylinder(radius=0.006, length=0.006),
                origin=Origin(xyz=(x, -0.051, z), rpy=(1.5708, 0.0, 0.0)),
                material=black,
                name=f"head_bolt_{0 if x < 0 else 1}_{0 if z < 0.04 else 1}",
            )

    model.articulation(
        "rail_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base_rail,
        child=carriage,
        origin=Origin(xyz=(-0.25, 0.0, 0.076)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.20, lower=0.0, upper=0.45),
    )
    model.articulation(
        "carriage_to_mast",
        ArticulationType.FIXED,
        parent=carriage,
        child=mast,
        origin=Origin(xyz=(0.020, 0.0, 0.045)),
    )
    model.articulation(
        "mast_to_head",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=vertical_head,
        origin=Origin(xyz=(0.0, -0.019, 0.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.15, lower=0.0, upper=0.22),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base_rail = object_model.get_part("base_rail")
    carriage = object_model.get_part("carriage")
    mast = object_model.get_part("mast")
    vertical_head = object_model.get_part("vertical_head")
    x_slide = object_model.get_articulation("rail_to_carriage")
    z_slide = object_model.get_articulation("mast_to_head")

    prismatic = [
        art
        for art in object_model.articulations
        if art.articulation_type == ArticulationType.PRISMATIC
    ]
    ctx.check(
        "two orthogonal prismatic slides",
        len(prismatic) == 2
        and tuple(x_slide.axis) == (1.0, 0.0, 0.0)
        and tuple(z_slide.axis) == (0.0, 0.0, 1.0),
        details=f"prismatic={[(a.name, a.axis) for a in prismatic]}",
    )

    ctx.expect_gap(
        carriage,
        base_rail,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="saddle_plate",
        negative_elem="rail_cap",
        name="carriage sits on rail cap",
    )
    ctx.expect_overlap(
        carriage,
        base_rail,
        axes="x",
        min_overlap=0.15,
        elem_a="saddle_plate",
        elem_b="rail_cap",
        name="carriage retained on horizontal rail",
    )
    ctx.expect_gap(
        mast,
        carriage,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="foot_plate",
        negative_elem="saddle_plate",
        name="mast foot is bolted to carriage top",
    )
    ctx.expect_gap(
        mast,
        vertical_head,
        axis="y",
        max_gap=0.001,
        max_penetration=0.000001,
        positive_elem="front_rail_0",
        negative_elem="head_body",
        name="head rides against vertical guide rail",
    )
    ctx.expect_overlap(
        vertical_head,
        mast,
        axes="z",
        min_overlap=0.08,
        elem_a="head_body",
        elem_b="front_rail_0",
        name="head retained on vertical guide at low position",
    )

    rest_carriage_pos = ctx.part_world_position(carriage)
    with ctx.pose({x_slide: 0.45}):
        ctx.expect_overlap(
            carriage,
            base_rail,
            axes="x",
            min_overlap=0.15,
            elem_a="saddle_plate",
            elem_b="rail_cap",
            name="carriage retained on horizontal rail at full travel",
        )
        extended_carriage_pos = ctx.part_world_position(carriage)

    ctx.check(
        "horizontal slide moves along X",
        rest_carriage_pos is not None
        and extended_carriage_pos is not None
        and extended_carriage_pos[0] > rest_carriage_pos[0] + 0.40,
        details=f"rest={rest_carriage_pos}, extended={extended_carriage_pos}",
    )

    rest_head_pos = ctx.part_world_position(vertical_head)
    with ctx.pose({z_slide: 0.22}):
        ctx.expect_overlap(
            vertical_head,
            mast,
            axes="z",
            min_overlap=0.05,
            elem_a="head_body",
            elem_b="front_rail_0",
            name="head remains engaged at top of stroke",
        )
        raised_head_pos = ctx.part_world_position(vertical_head)

    ctx.check(
        "vertical head moves along Z",
        rest_head_pos is not None
        and raised_head_pos is not None
        and raised_head_pos[2] > rest_head_pos[2] + 0.20,
        details=f"rest={rest_head_pos}, raised={raised_head_pos}",
    )

    return ctx.report()


object_model = build_object_model()
