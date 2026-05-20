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
    model = ArticulatedObject(name="xz_positioning_stage")

    aluminum = model.material("brushed_clear_anodized_aluminum", rgba=(0.72, 0.74, 0.73, 1.0))
    dark_rail = model.material("black_hardened_steel", rgba=(0.025, 0.027, 0.030, 1.0))
    blue = model.material("blue_anodized_carriage", rgba=(0.05, 0.22, 0.55, 1.0))
    black = model.material("matte_black_end_stop", rgba=(0.01, 0.01, 0.012, 1.0))
    tool = model.material("satin_tool_plate", rgba=(0.50, 0.52, 0.50, 1.0))
    screw = model.material("dark_socket_screws", rgba=(0.015, 0.015, 0.016, 1.0))

    base = model.part("base_rail")
    base.visual(
        Box((0.70, 0.18, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=aluminum,
        name="base_plate",
    )
    for index, y in enumerate((-0.055, 0.055)):
        base.visual(
            Box((0.62, 0.025, 0.032)),
            origin=Origin(xyz=(0.0, y, 0.045)),
            material=aluminum,
            name=f"rail_rib_{index}",
        )
        base.visual(
            Box((0.62, 0.018, 0.008)),
            origin=Origin(xyz=(0.0, y, 0.063)),
            material=dark_rail,
            name=f"rail_strip_{index}",
        )
    for index, x in enumerate((-0.335, 0.335)):
        base.visual(
            Box((0.024, 0.145, 0.042)),
            origin=Origin(xyz=(x, 0.0, 0.051)),
            material=black,
            name=f"end_stop_{index}",
        )
    for index, (x, y) in enumerate(
        (
            (-0.250, -0.070),
            (-0.250, 0.070),
            (0.000, -0.070),
            (0.000, 0.070),
            (0.250, -0.070),
            (0.250, 0.070),
        )
    ):
        base.visual(
            Cylinder(radius=0.009, length=0.006),
            origin=Origin(xyz=(x, y, 0.030)),
            material=screw,
            name=f"base_screw_{index}",
        )

    carriage = model.part("carriage")
    for index, y in enumerate((-0.055, 0.055)):
        carriage.visual(
            Box((0.160, 0.030, 0.026)),
            origin=Origin(xyz=(0.0, y, 0.013)),
            material=dark_rail,
            name=f"bearing_shoe_{index}",
        )
    carriage.visual(
        Box((0.185, 0.160, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=blue,
        name="cross_slide_plate",
    )
    carriage.visual(
        Box((0.145, 0.120, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        material=blue,
        name="saddle_block",
    )
    carriage.visual(
        Box((0.125, 0.022, 0.340)),
        origin=Origin(xyz=(0.0, 0.050, 0.237)),
        material=aluminum,
        name="upright_plate",
    )
    for index, x in enumerate((-0.070, 0.070)):
        carriage.visual(
            Box((0.024, 0.052, 0.115)),
            origin=Origin(xyz=(x, 0.026, 0.123)),
            material=blue,
            name=f"side_gusset_{index}",
        )
    for name, x in (("vertical_rail_0", -0.040), ("vertical_rail_1", 0.040)):
        carriage.visual(
            Box((0.018, 0.020, 0.300)),
            origin=Origin(xyz=(x, 0.030, 0.235)),
            material=dark_rail,
            name=name,
        )
    for index, z in enumerate((0.075, 0.395)):
        carriage.visual(
            Box((0.115, 0.025, 0.022)),
            origin=Origin(xyz=(0.0, 0.030, z)),
            material=black,
            name=f"z_end_stop_{index}",
        )
    carriage.visual(
        Cylinder(radius=0.0065, length=0.325),
        origin=Origin(xyz=(0.0, 0.030, 0.235)),
        material=screw,
        name="lead_screw",
    )

    vertical_slide = model.part("vertical_slide")
    vertical_slide.visual(
        Box((0.100, 0.016, 0.105)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=blue,
        name="slide_block",
    )
    vertical_slide.visual(
        Box((0.078, 0.012, 0.092)),
        origin=Origin(xyz=(0.0, -0.0002, 0.0)),
        material=tool,
        name="tool_plate",
    )
    vertical_slide.visual(
        Cylinder(radius=0.014, length=0.020),
        origin=Origin(xyz=(0.0, -0.016, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=tool,
        name="tool_boss",
    )
    for index, (x, z) in enumerate(((-0.026, -0.030), (0.026, -0.030), (-0.026, 0.030), (0.026, 0.030))):
        vertical_slide.visual(
            Cylinder(radius=0.0045, length=0.0045),
            origin=Origin(xyz=(x, -0.008, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=screw,
            name=f"tool_screw_{index}",
        )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(-0.160, 0.0, 0.067)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.20, lower=0.0, upper=0.320),
    )
    model.articulation(
        "carriage_to_vertical_slide",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=vertical_slide,
        origin=Origin(xyz=(0.0, 0.012, 0.130)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.14, lower=0.0, upper=0.180),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_rail")
    carriage = object_model.get_part("carriage")
    vertical_slide = object_model.get_part("vertical_slide")
    lower_axis = object_model.get_articulation("base_to_carriage")
    upper_axis = object_model.get_articulation("carriage_to_vertical_slide")

    ctx.check(
        "stage has two prismatic axes",
        len(object_model.articulations) == 2
        and lower_axis.articulation_type == ArticulationType.PRISMATIC
        and upper_axis.articulation_type == ArticulationType.PRISMATIC,
        details=f"articulations={object_model.articulations}",
    )

    with ctx.pose({lower_axis: 0.0, upper_axis: 0.0}):
        for index in (0, 1):
            ctx.expect_gap(
                carriage,
                base,
                axis="z",
                positive_elem=f"bearing_shoe_{index}",
                negative_elem=f"rail_strip_{index}",
                max_gap=0.001,
                max_penetration=0.0,
                name=f"x bearing shoe {index} sits on its rail",
            )
            ctx.expect_overlap(
                carriage,
                base,
                axes="xy",
                elem_a=f"bearing_shoe_{index}",
                elem_b=f"rail_strip_{index}",
                min_overlap=0.015,
                name=f"x bearing shoe {index} remains on rail footprint",
            )
            ctx.expect_gap(
                carriage,
                vertical_slide,
                axis="y",
                positive_elem=f"vertical_rail_{index}",
                negative_elem="slide_block",
                max_gap=0.001,
                max_penetration=0.000001,
                name=f"z slide block bears against rail {index}",
            )
            ctx.expect_overlap(
                vertical_slide,
                carriage,
                axes="xz",
                elem_a="slide_block",
                elem_b=f"vertical_rail_{index}",
                min_overlap=0.015,
                name=f"z slide block overlaps rail {index} in projection",
            )

    carriage_rest = ctx.part_world_position(carriage)
    with ctx.pose({lower_axis: 0.320}):
        carriage_extended = ctx.part_world_position(carriage)
        for index in (0, 1):
            ctx.expect_overlap(
                carriage,
                base,
                axes="xy",
                elem_a=f"bearing_shoe_{index}",
                elem_b=f"rail_strip_{index}",
                min_overlap=0.015,
                name=f"x bearing shoe {index} retained at full travel",
            )
    ctx.check(
        "lower axis moves carriage along positive X",
        carriage_rest is not None
        and carriage_extended is not None
        and carriage_extended[0] > carriage_rest[0] + 0.25,
        details=f"rest={carriage_rest}, extended={carriage_extended}",
    )

    slide_rest = ctx.part_world_position(vertical_slide)
    with ctx.pose({upper_axis: 0.180}):
        slide_extended = ctx.part_world_position(vertical_slide)
        ctx.expect_within(
            vertical_slide,
            carriage,
            axes="z",
            inner_elem="slide_block",
            outer_elem="vertical_rail_0",
            margin=0.010,
            name="vertical slide block stays within guide length",
        )
        ctx.expect_overlap(
            vertical_slide,
            carriage,
            axes="xz",
            elem_a="slide_block",
            elem_b="vertical_rail_0",
            min_overlap=0.015,
            name="vertical slide remains engaged at raised travel",
        )
    ctx.check(
        "upper axis moves tool plate along positive Z",
        slide_rest is not None and slide_extended is not None and slide_extended[2] > slide_rest[2] + 0.14,
        details=f"rest={slide_rest}, extended={slide_extended}",
    )

    return ctx.report()


object_model = build_object_model()
