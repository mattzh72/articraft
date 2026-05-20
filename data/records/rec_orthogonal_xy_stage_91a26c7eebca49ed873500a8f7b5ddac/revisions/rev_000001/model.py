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
    model = ArticulatedObject(name="compact_service_xy_axis")

    anodized_black = model.material("anodized_black", rgba=(0.02, 0.022, 0.026, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.68, 0.70, 0.70, 1.0))
    ground_steel = model.material("ground_steel", rgba=(0.88, 0.89, 0.86, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    safety_orange = model.material("safety_orange", rgba=(0.95, 0.32, 0.05, 1.0))

    lower_guide = model.part("lower_guide")
    lower_guide.visual(
        Box((0.78, 0.26, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=anodized_black,
        name="base_plate",
    )
    lower_guide.visual(
        Box((0.70, 0.032, 0.040)),
        origin=Origin(xyz=(0.0, -0.085, 0.055)),
        material=ground_steel,
        name="lower_rail_0",
    )
    lower_guide.visual(
        Box((0.70, 0.032, 0.040)),
        origin=Origin(xyz=(0.0, 0.085, 0.055)),
        material=ground_steel,
        name="lower_rail_1",
    )
    lower_guide.visual(
        Box((0.72, 0.030, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        material=brushed_aluminum,
        name="center_stiffener",
    )
    for idx, x in enumerate((-0.355, 0.355)):
        lower_guide.visual(
            Box((0.055, 0.245, 0.055)),
            origin=Origin(xyz=(x, 0.0, 0.0625)),
            material=anodized_black,
            name=f"end_stop_{idx}",
        )
        lower_guide.visual(
            Box((0.018, 0.220, 0.030)),
            origin=Origin(xyz=(x * 0.96, 0.0, 0.105)),
            material=dark_rubber,
            name=f"bumper_{idx}",
        )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.50, 0.230, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=brushed_aluminum,
        name="carriage_plate",
    )
    for idx, y in enumerate((-0.122, 0.122)):
        carriage.visual(
            Box((0.48, 0.020, 0.046)),
            origin=Origin(xyz=(0.0, y, 0.005)),
            material=anodized_black,
            name=f"side_skirt_{idx}",
        )
    carriage.visual(
        Box((0.030, 0.340, 0.018)),
        origin=Origin(xyz=(-0.078, 0.0, 0.054)),
        material=ground_steel,
        name="upper_rail_0",
    )
    carriage.visual(
        Box((0.030, 0.340, 0.018)),
        origin=Origin(xyz=(0.078, 0.0, 0.054)),
        material=ground_steel,
        name="upper_rail_1",
    )
    carriage.visual(
        Box((0.18, 0.030, 0.016)),
        origin=Origin(xyz=(0.0, -0.125, 0.026)),
        material=safety_orange,
        name="front_travel_flag",
    )
    carriage.visual(
        Box((0.18, 0.030, 0.016)),
        origin=Origin(xyz=(0.0, 0.125, 0.026)),
        material=safety_orange,
        name="rear_travel_flag",
    )

    cross_slide = model.part("cross_slide")
    cross_slide.visual(
        Box((0.250, 0.285, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=anodized_black,
        name="cross_saddle",
    )
    cross_slide.visual(
        Box((0.205, 0.235, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        material=brushed_aluminum,
        name="top_cap",
    )
    cross_slide.visual(
        Box((0.120, 0.075, 0.045)),
        origin=Origin(xyz=(0.0, 0.112, 0.0615)),
        material=brushed_aluminum,
        name="output_neck",
    )
    cross_slide.visual(
        Box((0.160, 0.020, 0.160)),
        origin=Origin(xyz=(0.0, 0.158, 0.113)),
        material=brushed_aluminum,
        name="output_face",
    )
    for ix, x in enumerate((-0.050, 0.050)):
        for iz, z in enumerate((0.073, 0.153)):
            cross_slide.visual(
                Cylinder(radius=0.010, length=0.006),
                origin=Origin(xyz=(x, 0.171, z), rpy=(-pi / 2.0, 0.0, 0.0)),
                material=ground_steel,
                name=f"face_bolt_{ix}_{iz}",
            )

    model.articulation(
        "guide_to_carriage",
        ArticulationType.PRISMATIC,
        parent=lower_guide,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.45, lower=-0.10, upper=0.10),
    )
    model.articulation(
        "carriage_to_cross_slide",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=cross_slide,
        origin=Origin(xyz=(0.0, 0.0, 0.063)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=140.0, velocity=0.35, lower=-0.055, upper=0.055),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower_guide = object_model.get_part("lower_guide")
    carriage = object_model.get_part("carriage")
    cross_slide = object_model.get_part("cross_slide")
    x_joint = object_model.get_articulation("guide_to_carriage")
    y_joint = object_model.get_articulation("carriage_to_cross_slide")

    ctx.expect_gap(
        carriage,
        lower_guide,
        axis="z",
        positive_elem="carriage_plate",
        negative_elem="lower_rail_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="carriage rides on lower rail",
    )
    ctx.expect_overlap(
        carriage,
        lower_guide,
        axes="xy",
        elem_a="carriage_plate",
        elem_b="lower_rail_0",
        min_overlap=0.030,
        name="carriage footprint is retained on lower guide",
    )
    ctx.expect_gap(
        cross_slide,
        carriage,
        axis="z",
        positive_elem="cross_saddle",
        negative_elem="upper_rail_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="cross-slide rides on upper rail",
    )
    ctx.expect_overlap(
        cross_slide,
        carriage,
        axes="xy",
        elem_a="cross_saddle",
        elem_b="upper_rail_0",
        min_overlap=0.025,
        name="cross-slide footprint is retained on upper guide",
    )

    rest_carriage_pos = ctx.part_world_position(carriage)
    rest_cross_pos = ctx.part_world_position(cross_slide)
    with ctx.pose({x_joint: 0.10, y_joint: 0.055}):
        ctx.expect_overlap(
            carriage,
            lower_guide,
            axes="x",
            elem_a="carriage_plate",
            elem_b="lower_rail_0",
            min_overlap=0.25,
            name="extended carriage keeps insertion on lower rail",
        )
        ctx.expect_overlap(
            cross_slide,
            carriage,
            axes="y",
            elem_a="cross_saddle",
            elem_b="upper_rail_0",
            min_overlap=0.20,
            name="extended cross-slide keeps insertion on upper rail",
        )
        extended_carriage_pos = ctx.part_world_position(carriage)
        extended_cross_pos = ctx.part_world_position(cross_slide)

    ctx.check(
        "lower stage moves along X",
        rest_carriage_pos is not None
        and extended_carriage_pos is not None
        and extended_carriage_pos[0] > rest_carriage_pos[0] + 0.09,
        details=f"rest={rest_carriage_pos}, extended={extended_carriage_pos}",
    )
    ctx.check(
        "upper stage moves along Y",
        rest_cross_pos is not None
        and extended_cross_pos is not None
        and extended_cross_pos[1] > rest_cross_pos[1] + 0.05,
        details=f"rest={rest_cross_pos}, extended={extended_cross_pos}",
    )

    return ctx.report()


object_model = build_object_model()
