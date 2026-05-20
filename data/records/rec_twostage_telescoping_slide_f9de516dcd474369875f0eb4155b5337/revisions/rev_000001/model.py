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
    model = ArticulatedObject(name="linear_extension_module")

    dark_anodized = model.material("dark_anodized_aluminum", rgba=(0.05, 0.055, 0.06, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.68, 0.70, 0.72, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.46, 0.48, 0.50, 1.0))
    bronze_wear = model.material("bronze_wear_strips", rgba=(0.70, 0.47, 0.20, 1.0))
    rubber = model.material("black_rubber_stops", rgba=(0.01, 0.01, 0.012, 1.0))
    screw = model.material("socket_head_screws", rgba=(0.015, 0.016, 0.018, 1.0))

    base = model.part("guide_base")
    base.visual(
        Box((0.86, 0.26, 0.028)),
        origin=Origin(xyz=(0.43, 0.0, 0.014)),
        material=dark_anodized,
        name="base_plate",
    )
    for idx, y in enumerate((-0.094, 0.094)):
        base.visual(
            Box((0.78, 0.034, 0.058)),
            origin=Origin(xyz=(0.43, y, 0.056)),
            material=dark_anodized,
            name=f"base_rail_{idx}",
        )
        base.visual(
            Box((0.72, 0.006, 0.026)),
            origin=Origin(xyz=(0.43, y * 0.775, 0.060)),
            material=bronze_wear,
            name=f"base_wear_strip_{idx}",
        )
        for x, tag in ((0.070, "rear"), (0.790, "front")):
            base.visual(
                Box((0.044, 0.052, 0.076)),
                origin=Origin(xyz=(x, y, 0.066)),
                material=satin_steel,
                name=f"{tag}_end_stop_{idx}",
            )
            base.visual(
                Box((0.018, 0.010, 0.030)),
                origin=Origin(xyz=(x, y * 0.755, 0.064)),
                material=rubber,
                name=f"{tag}_bumper_{idx}",
            )
    for idx, (x, y) in enumerate(
        ((0.12, -0.110), (0.12, 0.110), (0.74, -0.110), (0.74, 0.110))
    ):
        base.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(x, y, 0.030)),
            material=screw,
            name=f"base_screw_{idx}",
        )

    outer_slide = model.part("outer_slide")
    outer_slide.visual(
        Box((0.62, 0.122, 0.036)),
        origin=Origin(xyz=(0.310, 0.0, 0.0)),
        material=brushed_aluminum,
        name="outer_slide_body",
    )
    outer_slide.visual(
        Box((0.56, 0.090, 0.014)),
        origin=Origin(xyz=(0.320, 0.0, 0.025)),
        material=brushed_aluminum,
        name="outer_top_land",
    )
    for idx, y in enumerate((-0.06485, 0.06485)):
        outer_slide.visual(
            Box((0.520, 0.010, 0.028)),
            origin=Origin(xyz=(0.310, y, 0.000)),
            material=bronze_wear,
            name=f"outer_bearing_shoe_{idx}",
        )
    for idx, y in enumerate((-0.050, 0.050)):
        outer_slide.visual(
            Box((0.56, 0.018, 0.026)),
            origin=Origin(xyz=(0.320, y, 0.042)),
            material=satin_steel,
            name=f"upper_guide_rail_{idx}",
        )
        outer_slide.visual(
            Box((0.50, 0.004, 0.014)),
            origin=Origin(xyz=(0.320, y * 0.72, 0.056)),
            material=bronze_wear,
            name=f"upper_wear_strip_{idx}",
        )
        for x, tag in ((0.080, "inner_rear"), (0.560, "inner_front")):
            outer_slide.visual(
                Box((0.034, 0.025, 0.030)),
                origin=Origin(xyz=(x, y, 0.068)),
                material=satin_steel,
                name=f"{tag}_stop_{idx}",
            )
            outer_slide.visual(
                Box((0.016, 0.007, 0.020)),
                origin=Origin(xyz=(x, y * 0.74, 0.068)),
                material=rubber,
                name=f"{tag}_bumper_{idx}",
            )
        for x in (0.155, 0.485):
            outer_slide.visual(
                Cylinder(radius=0.006, length=0.006),
                origin=Origin(xyz=(x, y, 0.056)),
                material=screw,
                name=f"upper_rail_screw_{idx}_{int(x * 1000)}",
            )
    outer_slide.visual(
        Box((0.040, 0.118, 0.034)),
        origin=Origin(xyz=(0.020, 0.0, 0.001)),
        material=brushed_aluminum,
        name="outer_rear_cap",
    )
    outer_slide.visual(
        Box((0.040, 0.118, 0.034)),
        origin=Origin(xyz=(0.600, 0.0, 0.001)),
        material=brushed_aluminum,
        name="outer_front_cap",
    )

    inner_slide = model.part("inner_slide")
    inner_slide.visual(
        Box((0.460, 0.052, 0.026)),
        origin=Origin(xyz=(0.230, 0.0, 0.0)),
        material=satin_steel,
        name="inner_slide_body",
    )
    for idx, y in enumerate((-0.030, 0.030)):
        inner_slide.visual(
            Box((0.420, 0.008, 0.018)),
            origin=Origin(xyz=(0.235, y, 0.000)),
            material=bronze_wear,
            name=f"inner_bearing_shoe_{idx}",
        )
    inner_slide.visual(
        Box((0.380, 0.035, 0.018)),
        origin=Origin(xyz=(0.250, 0.0, 0.022)),
        material=brushed_aluminum,
        name="inner_top_rib",
    )
    inner_slide.visual(
        Box((0.048, 0.052, 0.026)),
        origin=Origin(xyz=(0.470, 0.0, 0.0)),
        material=satin_steel,
        name="front_neck",
    )
    inner_slide.visual(
        Box((0.036, 0.115, 0.070)),
        origin=Origin(xyz=(0.500, 0.0, 0.007)),
        material=brushed_aluminum,
        name="front_carriage",
    )
    inner_slide.visual(
        Box((0.016, 0.095, 0.030)),
        origin=Origin(xyz=(0.522, 0.0, 0.010)),
        material=rubber,
        name="front_stop_pad",
    )
    for idx, y in enumerate((-0.038, 0.038)):
        inner_slide.visual(
            Cylinder(radius=0.005, length=0.006),
            origin=Origin(xyz=(0.500, y, 0.041)),
            material=screw,
            name=f"carriage_screw_{idx}",
        )

    model.articulation(
        "base_to_outer_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=outer_slide,
        origin=Origin(xyz=(0.085, 0.0, 0.060)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.320),
    )
    model.articulation(
        "outer_to_inner_slide",
        ArticulationType.PRISMATIC,
        parent=outer_slide,
        child=inner_slide,
        origin=Origin(xyz=(0.120, 0.0, 0.067)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.32, lower=0.0, upper=0.260),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("guide_base")
    outer = object_model.get_part("outer_slide")
    inner = object_model.get_part("inner_slide")
    outer_joint = object_model.get_articulation("base_to_outer_slide")
    inner_joint = object_model.get_articulation("outer_to_inner_slide")

    ctx.expect_within(
        outer,
        base,
        axes="y",
        inner_elem="outer_slide_body",
        outer_elem="base_plate",
        margin=0.0,
        name="outer slide is centered between base rails",
    )
    ctx.expect_gap(
        outer,
        base,
        axis="z",
        positive_elem="outer_slide_body",
        negative_elem="base_plate",
        min_gap=0.006,
        max_gap=0.020,
        name="outer slide clears base plate",
    )
    ctx.expect_overlap(
        outer,
        base,
        axes="x",
        elem_a="outer_slide_body",
        elem_b="base_plate",
        min_overlap=0.45,
        name="outer slide remains deeply engaged at rest",
    )
    ctx.expect_within(
        inner,
        outer,
        axes="y",
        inner_elem="inner_slide_body",
        outer_elem="outer_top_land",
        margin=0.0,
        name="inner slide is centered on upper guide",
    )
    ctx.expect_overlap(
        inner,
        outer,
        axes="x",
        elem_a="inner_slide_body",
        elem_b="outer_slide_body",
        min_overlap=0.30,
        name="inner slide remains engaged at rest",
    )

    outer_rest = ctx.part_world_position(outer)
    inner_rest = ctx.part_world_position(inner)
    with ctx.pose({outer_joint: 0.320, inner_joint: 0.260}):
        ctx.expect_overlap(
            outer,
            base,
            axes="x",
            elem_a="outer_slide_body",
            elem_b="base_plate",
            min_overlap=0.25,
            name="outer slide retains insertion at full travel",
        )
        ctx.expect_overlap(
            inner,
            outer,
            axes="x",
            elem_a="inner_slide_body",
            elem_b="outer_slide_body",
            min_overlap=0.18,
            name="inner slide retains insertion at full travel",
        )
        ctx.expect_within(
            inner,
            outer,
            axes="y",
            inner_elem="inner_slide_body",
            outer_elem="outer_top_land",
            margin=0.0,
            name="inner slide stays laterally guided at full travel",
        )
        outer_extended = ctx.part_world_position(outer)
        inner_extended = ctx.part_world_position(inner)

    ctx.check(
        "outer section extends in positive x",
        outer_rest is not None
        and outer_extended is not None
        and outer_extended[0] > outer_rest[0] + 0.30,
        details=f"rest={outer_rest}, extended={outer_extended}",
    )
    ctx.check(
        "inner section adds second positive x extension",
        inner_rest is not None
        and inner_extended is not None
        and inner_extended[0] > inner_rest[0] + 0.55,
        details=f"rest={inner_rest}, extended={inner_extended}",
    )

    return ctx.report()


object_model = build_object_model()
