from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_linear_extension")

    dark_aluminum = model.material("dark_hard_anodized_aluminum", rgba=(0.10, 0.11, 0.12, 1.0))
    satin_steel = model.material("satin_ground_steel", rgba=(0.68, 0.70, 0.70, 1.0))
    bright_steel = model.material("bright_machined_guide_faces", rgba=(0.86, 0.88, 0.86, 1.0))
    bronze = model.material("bronze_bearing_wear_pads", rgba=(0.74, 0.52, 0.25, 1.0))
    black_rubber = model.material("black_rubber_buffers", rgba=(0.01, 0.01, 0.01, 1.0))
    red_anodized = model.material("red_anodized_stop_collars", rgba=(0.62, 0.05, 0.03, 1.0))

    outer = model.part("outer_beam")
    outer.visual(
        Box((1.12, 0.220, 0.035)),
        origin=Origin(xyz=(0.560, 0.0, 0.0175)),
        material=dark_aluminum,
        name="outer_base",
    )
    outer.visual(
        Box((1.12, 0.006, 0.006)),
        origin=Origin(xyz=(0.560, 0.0, 0.0380)),
        material=bright_steel,
        name="outer_wear_strip",
    )
    for suffix, y in (("0", 0.095), ("1", -0.095)):
        outer.visual(
            Box((1.12, 0.030, 0.112)),
            origin=Origin(xyz=(0.560, y, 0.090)),
            material=dark_aluminum,
            name=f"outer_side_wall_{suffix}",
        )
        outer.visual(
            Box((1.10, 0.006, 0.072)),
            origin=Origin(xyz=(0.570, 0.074 if y > 0 else -0.074, 0.083)),
            material=bright_steel,
            name=f"outer_side_guide_{suffix}",
        )
    outer.visual(
        Box((1.12, 0.070, 0.020)),
        origin=Origin(xyz=(0.560, 0.060, 0.155)),
        material=dark_aluminum,
        name="outer_top_keeper_0",
    )
    outer.visual(
        Box((1.12, 0.070, 0.020)),
        origin=Origin(xyz=(0.560, -0.060, 0.155)),
        material=dark_aluminum,
        name="outer_top_keeper_1",
    )

    outer.visual(
        Box((0.035, 0.220, 0.170)),
        origin=Origin(xyz=(-0.0175, 0.0, 0.085)),
        material=dark_aluminum,
        name="outer_rear_end_block",
    )
    outer.visual(
        Box((0.015, 0.020, 0.018)),
        origin=Origin(xyz=(0.0525, 0.055, 0.132)),
        material=black_rubber,
        name="outer_rear_buffer_0",
    )
    outer.visual(
        Box((0.015, 0.020, 0.018)),
        origin=Origin(xyz=(0.0525, -0.055, 0.132)),
        material=black_rubber,
        name="outer_rear_buffer_1",
    )
    outer.visual(
        Box((0.045, 0.026, 0.020)),
        origin=Origin(xyz=(0.0225, 0.055, 0.132)),
        material=dark_aluminum,
        name="outer_rear_buffer_mount_0",
    )
    outer.visual(
        Box((0.045, 0.026, 0.020)),
        origin=Origin(xyz=(0.0225, -0.055, 0.132)),
        material=dark_aluminum,
        name="outer_rear_buffer_mount_1",
    )
    outer.visual(
        Box((0.020, 0.020, 0.018)),
        origin=Origin(xyz=(1.085, 0.073, 0.132)),
        material=black_rubber,
        name="outer_front_buffer_0",
    )
    outer.visual(
        Box((0.020, 0.020, 0.018)),
        origin=Origin(xyz=(1.085, -0.073, 0.132)),
        material=black_rubber,
        name="outer_front_buffer_1",
    )
    outer.visual(
        Box((0.045, 0.040, 0.170)),
        origin=Origin(xyz=(1.0975, 0.090, 0.085)),
        material=dark_aluminum,
        name="outer_front_cheek_0",
    )
    outer.visual(
        Box((0.045, 0.040, 0.170)),
        origin=Origin(xyz=(1.0975, -0.090, 0.085)),
        material=dark_aluminum,
        name="outer_front_cheek_1",
    )
    outer.visual(
        Box((0.045, 0.155, 0.018)),
        origin=Origin(xyz=(1.0975, 0.0, 0.161)),
        material=dark_aluminum,
        name="outer_front_lintel",
    )

    inner = model.part("inner_rail")
    inner.visual(
        Box((0.880, 0.054, 0.018)),
        origin=Origin(xyz=(0.440, 0.0, 0.049)),
        material=bright_steel,
        name="inner_bottom_shoe",
    )
    inner.visual(
        Box((0.880, 0.074, 0.040)),
        origin=Origin(xyz=(0.440, 0.0, 0.078)),
        material=satin_steel,
        name="inner_main_web",
    )
    inner.visual(
        Box((0.880, 0.044, 0.025)),
        origin=Origin(xyz=(0.440, 0.0, 0.1105)),
        material=bright_steel,
        name="inner_top_track",
    )
    for suffix, y in (("0", 0.039), ("1", -0.039)):
        inner.visual(
            Box((0.860, 0.006, 0.054)),
            origin=Origin(xyz=(0.450, y, 0.083)),
            material=bright_steel,
            name=f"inner_side_face_{suffix}",
        )
    inner.visual(
        Box((0.025, 0.130, 0.018)),
        origin=Origin(xyz=(0.0125, 0.0, 0.132)),
        material=red_anodized,
        name="inner_rear_collar",
    )
    inner.visual(
        Box((0.050, 0.012, 0.018)),
        origin=Origin(xyz=(0.650, 0.072, 0.132)),
        material=red_anodized,
        name="inner_front_collar",
    )
    inner.visual(
        Box((0.050, 0.031, 0.006)),
        origin=Origin(xyz=(0.650, 0.0525, 0.078)),
        material=red_anodized,
        name="inner_front_collar_web_0",
    )
    inner.visual(
        Box((0.050, 0.004, 0.046)),
        origin=Origin(xyz=(0.650, 0.067, 0.101)),
        material=red_anodized,
        name="inner_front_collar_post_0",
    )
    inner.visual(
        Box((0.050, 0.012, 0.018)),
        origin=Origin(xyz=(0.650, -0.072, 0.132)),
        material=red_anodized,
        name="inner_front_collar_1",
    )
    inner.visual(
        Box((0.050, 0.031, 0.006)),
        origin=Origin(xyz=(0.650, -0.0525, 0.078)),
        material=red_anodized,
        name="inner_front_collar_web_1",
    )
    inner.visual(
        Box((0.050, 0.004, 0.046)),
        origin=Origin(xyz=(0.650, -0.067, 0.101)),
        material=red_anodized,
        name="inner_front_collar_post_1",
    )
    inner.visual(
        Box((0.018, 0.060, 0.060)),
        origin=Origin(xyz=(0.875, 0.0, 0.087)),
        material=satin_steel,
        name="inner_pull_end",
    )

    carriage = model.part("nose_carriage")
    carriage.visual(
        Box((0.245, 0.040, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.1275)),
        material=bronze,
        name="carriage_bearing_pad",
    )
    carriage.visual(
        Box((0.255, 0.044, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.162)),
        material=dark_aluminum,
        name="carriage_top_bridge",
    )
    carriage.visual(
        Box((0.245, 0.128, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.137)),
        material=dark_aluminum,
        name="carriage_cross_tie",
    )
    for suffix, y in (("0", 0.055), ("1", -0.055)):
        carriage.visual(
            Box((0.245, 0.018, 0.058)),
            origin=Origin(xyz=(0.0, y, 0.116)),
            material=dark_aluminum,
            name=f"carriage_side_cheek_{suffix}",
        )
        carriage.visual(
            Box((0.210, 0.010, 0.032)),
            origin=Origin(xyz=(0.0, 0.045 if y > 0 else -0.045, 0.105)),
            material=bronze,
            name=f"carriage_side_gib_{suffix}",
        )
    carriage.visual(
        Box((0.052, 0.044, 0.050)),
        origin=Origin(xyz=(0.150, 0.0, 0.157)),
        material=dark_aluminum,
        name="carriage_nose_block",
    )
    carriage.visual(
        Box((0.014, 0.040, 0.044)),
        origin=Origin(xyz=(0.183, 0.0, 0.154)),
        material=black_rubber,
        name="carriage_nose_buffer",
    )

    model.articulation(
        "outer_to_inner",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=inner,
        origin=Origin(xyz=(0.060, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=450.0, velocity=0.35, lower=0.0, upper=0.340),
    )
    model.articulation(
        "inner_to_carriage",
        ArticulationType.PRISMATIC,
        parent=inner,
        child=carriage,
        origin=Origin(xyz=(0.660, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.25, lower=0.0, upper=0.200),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_beam")
    inner = object_model.get_part("inner_rail")
    carriage = object_model.get_part("nose_carriage")
    outer_slide = object_model.get_articulation("outer_to_inner")
    carriage_slide = object_model.get_articulation("inner_to_carriage")

    ctx.expect_contact(
        inner,
        outer,
        elem_a="inner_bottom_shoe",
        elem_b="outer_wear_strip",
        name="inner rail is grounded on the outer guide strip",
    )
    ctx.expect_overlap(
        inner,
        outer,
        axes="x",
        elem_a="inner_bottom_shoe",
        elem_b="outer_wear_strip",
        min_overlap=0.80,
        name="collapsed inner rail has long engagement in outer beam",
    )
    ctx.expect_contact(
        carriage,
        inner,
        elem_a="carriage_bearing_pad",
        elem_b="inner_top_track",
        name="nose carriage bearing pad rides on the inner top track",
    )
    ctx.expect_overlap(
        carriage,
        inner,
        axes="x",
        elem_a="carriage_bearing_pad",
        elem_b="inner_top_track",
        min_overlap=0.22,
        name="collapsed carriage has full bearing engagement",
    )
    ctx.expect_contact(
        inner,
        outer,
        elem_a="inner_rear_collar",
        elem_b="outer_rear_buffer_0",
        name="collapsed rear collar bears on rear rubber buffer",
    )
    ctx.expect_gap(
        outer,
        inner,
        axis="z",
        positive_elem="outer_top_keeper_0",
        negative_elem="inner_front_collar",
        min_gap=0.002,
        name="inner collar clears the top keeper strip",
    )
    ctx.expect_gap(
        outer,
        carriage,
        axis="y",
        positive_elem="outer_top_keeper_0",
        negative_elem="carriage_top_bridge",
        min_gap=0.002,
        name="carriage bridge clears the positive keeper slot edge",
    )
    ctx.expect_gap(
        carriage,
        outer,
        axis="y",
        positive_elem="carriage_top_bridge",
        negative_elem="outer_top_keeper_1",
        min_gap=0.002,
        name="carriage bridge clears the negative keeper slot edge",
    )

    rest_carriage_pos = ctx.part_world_position(carriage)
    with ctx.pose({outer_slide: 0.340, carriage_slide: 0.200}):
        ctx.expect_overlap(
            inner,
            outer,
            axes="x",
            elem_a="inner_bottom_shoe",
            elem_b="outer_wear_strip",
            min_overlap=0.65,
            name="extended inner rail remains deeply captured in outer beam",
        )
        ctx.expect_overlap(
            carriage,
            inner,
            axes="x",
            elem_a="carriage_bearing_pad",
            elem_b="inner_top_track",
            min_overlap=0.13,
            name="extended carriage keeps believable bearing engagement",
        )
        ctx.expect_contact(
            inner,
            outer,
            elem_a="inner_front_collar",
            elem_b="outer_front_buffer_0",
            name="extended front collar reaches the outer rubber stop",
        )
        ctx.expect_gap(
            outer,
            inner,
            axis="z",
            positive_elem="outer_top_keeper_0",
            negative_elem="inner_front_collar",
            min_gap=0.002,
            name="extended inner collar still clears the top keeper strip",
        )
        ctx.expect_gap(
            outer,
            carriage,
            axis="y",
            positive_elem="outer_top_keeper_0",
            negative_elem="carriage_top_bridge",
            min_gap=0.002,
            name="extended carriage bridge clears the positive keeper slot edge",
        )
        ctx.expect_gap(
            carriage,
            outer,
            axis="y",
            positive_elem="carriage_top_bridge",
            negative_elem="outer_top_keeper_1",
            min_gap=0.002,
            name="extended carriage bridge clears the negative keeper slot edge",
        )
        extended_carriage_pos = ctx.part_world_position(carriage)

    ctx.check(
        "serial prismatic chain extends the nose forward",
        rest_carriage_pos is not None
        and extended_carriage_pos is not None
        and extended_carriage_pos[0] > rest_carriage_pos[0] + 0.50,
        details=f"rest={rest_carriage_pos}, extended={extended_carriage_pos}",
    )

    return ctx.report()


object_model = build_object_model()
