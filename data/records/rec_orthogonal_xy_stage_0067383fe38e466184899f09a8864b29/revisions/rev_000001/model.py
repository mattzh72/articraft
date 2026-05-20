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
    model = ArticulatedObject(name="compact_pick_and_place_stage")

    hard_anodized = model.material("hard_anodized_aluminum", rgba=(0.46, 0.48, 0.50, 1.0))
    dark_anodized = model.material("black_anodized_covers", rgba=(0.02, 0.022, 0.026, 1.0))
    rail_steel = model.material("ground_steel_rails", rgba=(0.72, 0.74, 0.72, 1.0))
    screw_steel = model.material("brushed_shoulder_screws", rgba=(0.80, 0.78, 0.70, 1.0))
    bumper = model.material("urethane_end_bumpers", rgba=(0.03, 0.03, 0.025, 1.0))
    deck_blue = model.material("blue_tooling_plate", rgba=(0.12, 0.22, 0.34, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.56, 0.34, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=hard_anodized,
        name="base_plate",
    )
    base.visual(
        Box((0.43, 0.026, 0.018)),
        origin=Origin(xyz=(0.0, -0.105, 0.049)),
        material=rail_steel,
        name="x_rail_0",
    )
    base.visual(
        Box((0.43, 0.026, 0.018)),
        origin=Origin(xyz=(0.0, 0.105, 0.049)),
        material=rail_steel,
        name="x_rail_1",
    )
    for idx, y in enumerate((-0.105, 0.105)):
        for sx in (-1.0, 1.0):
            base.visual(
                Box((0.030, 0.034, 0.020)),
                origin=Origin(xyz=(sx * 0.230, y, 0.050)),
                material=rail_steel,
                name=f"x_rail_cap_{idx}_{'pos' if sx > 0 else 'neg'}",
            )
    base.visual(
        Box((0.47, 0.052, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=dark_anodized,
        name="x_cover_strip",
    )
    base.visual(
        Box((0.035, 0.150, 0.040)),
        origin=Origin(xyz=(-0.225, 0.0, 0.060)),
        material=hard_anodized,
        name="x_stop_neg",
    )
    base.visual(
        Box((0.035, 0.150, 0.040)),
        origin=Origin(xyz=(0.225, 0.0, 0.060)),
        material=hard_anodized,
        name="x_stop_pos",
    )
    for sx, name in ((-1.0, "x_stop_neg"), (1.0, "x_stop_pos")):
        base.visual(
            Box((0.008, 0.075, 0.020)),
            origin=Origin(xyz=(sx * 0.204, 0.0, 0.061)),
            material=bumper,
            name=f"{name}_bumper",
        )
    for ix, x in enumerate((-0.235, 0.235)):
        for iy, y in enumerate((-0.135, 0.135)):
            base.visual(
                Cylinder(radius=0.010, length=0.006),
                origin=Origin(xyz=(x, y, 0.043)),
                material=screw_steel,
                name=f"base_screw_{ix}_{iy}",
            )

    first = model.part("first_carriage")
    first.visual(
        Box((0.225, 0.190, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0535)),
        material=hard_anodized,
        name="first_block",
    )
    first.visual(
        Box((0.118, 0.052, 0.026)),
        origin=Origin(xyz=(0.0, -0.105, 0.013)),
        material=rail_steel,
        name="x_shoe_0",
    )
    first.visual(
        Box((0.118, 0.052, 0.026)),
        origin=Origin(xyz=(0.0, 0.105, 0.013)),
        material=rail_steel,
        name="x_shoe_1",
    )
    first.visual(
        Box((0.006, 0.075, 0.020)),
        origin=Origin(xyz=(-0.1155, -0.105, 0.036)),
        material=dark_anodized,
        name="x_wiper_neg_0",
    )
    first.visual(
        Box((0.006, 0.075, 0.020)),
        origin=Origin(xyz=(0.1155, -0.105, 0.036)),
        material=dark_anodized,
        name="x_wiper_pos_0",
    )
    first.visual(
        Box((0.006, 0.075, 0.020)),
        origin=Origin(xyz=(-0.1155, 0.105, 0.036)),
        material=dark_anodized,
        name="x_wiper_neg_1",
    )
    first.visual(
        Box((0.006, 0.075, 0.020)),
        origin=Origin(xyz=(0.1155, 0.105, 0.036)),
        material=dark_anodized,
        name="x_wiper_pos_1",
    )
    first.visual(
        Box((0.026, 0.440, 0.016)),
        origin=Origin(xyz=(-0.074, 0.0, 0.089)),
        material=rail_steel,
        name="y_rail_0",
    )
    first.visual(
        Box((0.026, 0.440, 0.016)),
        origin=Origin(xyz=(0.074, 0.0, 0.089)),
        material=rail_steel,
        name="y_rail_1",
    )
    first.visual(
        Box((0.054, 0.360, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=dark_anodized,
        name="y_cover_strip",
    )
    first.visual(
        Box((0.165, 0.034, 0.032)),
        origin=Origin(xyz=(0.0, -0.195, 0.113)),
        material=hard_anodized,
        name="y_stop_neg",
    )
    first.visual(
        Box((0.165, 0.034, 0.032)),
        origin=Origin(xyz=(0.0, 0.195, 0.113)),
        material=hard_anodized,
        name="y_stop_pos",
    )
    for sy, name in ((-1.0, "y_stop_neg"), (1.0, "y_stop_pos")):
        first.visual(
            Box((0.075, 0.008, 0.018)),
            origin=Origin(xyz=(0.0, sy * 0.174, 0.114)),
            material=bumper,
            name=f"{name}_bumper",
        )
    for x in (-0.074, 0.074):
        for y in (-0.158, 0.158):
            first.visual(
                Cylinder(radius=0.007, length=0.005),
                origin=Origin(xyz=(x, y, 0.0995)),
                material=screw_steel,
                name=f"rail_screw_{'pos' if x > 0 else 'neg'}_{'front' if y > 0 else 'rear'}",
            )

    second = model.part("second_carriage")
    second.visual(
        Box((0.052, 0.112, 0.024)),
        origin=Origin(xyz=(-0.074, 0.0, 0.012)),
        material=rail_steel,
        name="y_shoe_0",
    )
    second.visual(
        Box((0.052, 0.112, 0.024)),
        origin=Origin(xyz=(0.074, 0.0, 0.012)),
        material=rail_steel,
        name="y_shoe_1",
    )
    second.visual(
        Box((0.172, 0.215, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.049)),
        material=hard_anodized,
        name="second_block",
    )
    second.visual(
        Box((0.180, 0.180, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.083)),
        material=deck_blue,
        name="tooling_deck",
    )
    second.visual(
        Box((0.145, 0.006, 0.020)),
        origin=Origin(xyz=(0.0, -0.1105, 0.036)),
        material=dark_anodized,
        name="y_wiper_neg",
    )
    second.visual(
        Box((0.145, 0.006, 0.020)),
        origin=Origin(xyz=(0.0, 0.1105, 0.036)),
        material=dark_anodized,
        name="y_wiper_pos",
    )
    for ix, x in enumerate((-0.062, 0.062)):
        for iy, y in enumerate((-0.062, 0.062)):
            second.visual(
                Cylinder(radius=0.008, length=0.006),
                origin=Origin(xyz=(x, y, 0.095)),
                material=screw_steel,
                name=f"deck_screw_{ix}_{iy}",
            )
    for ix, x in enumerate((-0.045, 0.0, 0.045)):
        for iy, y in enumerate((-0.045, 0.0, 0.045)):
            second.visual(
                Cylinder(radius=0.005, length=0.002),
                origin=Origin(xyz=(x, y, 0.093)),
                material=dark_anodized,
                name=f"tooling_hole_{ix}_{iy}",
            )

    model.articulation(
        "x_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=first,
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=-0.080, upper=0.080),
    )
    model.articulation(
        "y_slide",
        ArticulationType.PRISMATIC,
        parent=first,
        child=second,
        origin=Origin(xyz=(0.0, 0.0, 0.097)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.30, lower=-0.055, upper=0.055),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    first = object_model.get_part("first_carriage")
    second = object_model.get_part("second_carriage")
    x_slide = object_model.get_articulation("x_slide")
    y_slide = object_model.get_articulation("y_slide")

    ctx.check(
        "two orthogonal prismatic stages",
        len(object_model.articulations) == 2
        and x_slide.articulation_type == ArticulationType.PRISMATIC
        and y_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(x_slide.axis) == (1.0, 0.0, 0.0)
        and tuple(y_slide.axis) == (0.0, 1.0, 0.0),
        details=f"articulations={object_model.articulations}",
    )

    ctx.expect_gap(
        first,
        base,
        axis="z",
        positive_elem="x_shoe_0",
        negative_elem="x_rail_0",
        min_gap=0.0,
        max_gap=0.001,
        name="x carriage shoe seats on ground rail",
    )
    ctx.expect_overlap(
        first,
        base,
        axes="x",
        elem_a="x_shoe_0",
        elem_b="x_rail_0",
        min_overlap=0.100,
        name="x guide has retained rail overlap at center",
    )
    ctx.expect_gap(
        second,
        first,
        axis="z",
        positive_elem="y_shoe_0",
        negative_elem="y_rail_0",
        min_gap=0.0,
        max_gap=0.001,
        name="y carriage shoe seats on cross rail",
    )
    ctx.expect_overlap(
        second,
        first,
        axes="y",
        elem_a="y_shoe_0",
        elem_b="y_rail_0",
        min_overlap=0.100,
        name="y guide has retained rail overlap at center",
    )

    rest_first = ctx.part_world_position(first)
    rest_second = ctx.part_world_position(second)
    with ctx.pose({x_slide: 0.080}):
        ctx.expect_gap(
            base,
            first,
            axis="x",
            positive_elem="x_stop_pos",
            negative_elem="x_wiper_pos_0",
            min_gap=0.003,
            name="positive x stop remains clear at travel limit",
        )
        ctx.expect_overlap(
            first,
            base,
            axes="x",
            elem_a="x_shoe_0",
            elem_b="x_rail_0",
            min_overlap=0.100,
            name="x guide overlap is retained at positive travel",
        )
        extended_first = ctx.part_world_position(first)
    with ctx.pose({x_slide: -0.080}):
        ctx.expect_gap(
            first,
            base,
            axis="x",
            positive_elem="x_wiper_neg_0",
            negative_elem="x_stop_neg",
            min_gap=0.003,
            name="negative x stop remains clear at travel limit",
        )
    with ctx.pose({y_slide: 0.055}):
        ctx.expect_gap(
            first,
            second,
            axis="y",
            positive_elem="y_stop_pos",
            negative_elem="y_wiper_pos",
            min_gap=0.003,
            name="positive y stop remains clear at travel limit",
        )
        ctx.expect_overlap(
            second,
            first,
            axes="y",
            elem_a="y_shoe_0",
            elem_b="y_rail_0",
            min_overlap=0.100,
            name="y guide overlap is retained at positive travel",
        )
        extended_second = ctx.part_world_position(second)
    with ctx.pose({y_slide: -0.055}):
        ctx.expect_gap(
            second,
            first,
            axis="y",
            positive_elem="y_wiper_neg",
            negative_elem="y_stop_neg",
            min_gap=0.003,
            name="negative y stop remains clear at travel limit",
        )

    ctx.check(
        "x slide moves the first carriage along x",
        rest_first is not None and extended_first is not None and extended_first[0] > rest_first[0] + 0.075,
        details=f"rest={rest_first}, extended={extended_first}",
    )
    ctx.check(
        "y slide moves the tooling deck along y",
        rest_second is not None and extended_second is not None and extended_second[1] > rest_second[1] + 0.050,
        details=f"rest={rest_second}, extended={extended_second}",
    )

    return ctx.report()


object_model = build_object_model()
