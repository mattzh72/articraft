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
    model = ArticulatedObject(name="vertical_sideways_stage")

    cast_iron = model.material("dark_cast_iron", color=(0.10, 0.11, 0.12, 1.0))
    blue_enamel = model.material("blue_enamel", color=(0.08, 0.23, 0.42, 1.0))
    brushed_steel = model.material("brushed_steel", color=(0.72, 0.74, 0.72, 1.0))
    black_rubber = model.material("black_rubber", color=(0.02, 0.02, 0.018, 1.0))
    marking_white = model.material("engraved_white", color=(0.92, 0.92, 0.86, 1.0))

    mast = model.part("mast")
    mast.visual(
        Box((0.36, 0.24, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=cast_iron,
        name="base_plate",
    )
    mast.visual(
        Box((0.085, 0.062, 0.720)),
        origin=Origin(xyz=(0.0, 0.0, 0.400)),
        material=blue_enamel,
        name="upright_column",
    )
    mast.visual(
        Cylinder(radius=0.009, length=0.680),
        origin=Origin(xyz=(-0.050, -0.038, 0.400)),
        material=brushed_steel,
        name="vertical_rail_0",
    )
    mast.visual(
        Cylinder(radius=0.009, length=0.680),
        origin=Origin(xyz=(0.050, -0.038, 0.400)),
        material=brushed_steel,
        name="vertical_rail_1",
    )
    mast.visual(
        Box((0.150, 0.030, 0.035)),
        origin=Origin(xyz=(0.0, -0.040, 0.075)),
        material=cast_iron,
        name="lower_rail_clamp",
    )
    mast.visual(
        Box((0.150, 0.030, 0.035)),
        origin=Origin(xyz=(0.0, -0.040, 0.725)),
        material=cast_iron,
        name="upper_rail_clamp",
    )
    mast.visual(
        Box((0.010, 0.004, 0.560)),
        origin=Origin(xyz=(0.045, -0.032, 0.400)),
        material=marking_white,
        name="travel_scale",
    )
    for x in (-0.145, 0.145):
        for y in (-0.085, 0.085):
            mast.visual(
                Box((0.040, 0.030, 0.010)),
                origin=Origin(xyz=(x, y, -0.005)),
                material=black_rubber,
                name=f"foot_{'n' if y < 0 else 'p'}_{'n' if x < 0 else 'p'}",
            )

    head = model.part("head")
    head.visual(
        Box((0.165, 0.035, 0.130)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=blue_enamel,
        name="carriage_plate",
    )
    for x in (-0.050, 0.050):
        for z in (-0.038, 0.038):
            head.visual(
                Box((0.038, 0.020, 0.043)),
                origin=Origin(xyz=(x, -0.0275, z)),
                material=cast_iron,
                name=f"bearing_cap_{0 if x < 0 else 1}_{0 if z < 0 else 1}",
            )
    head.visual(
        Box((0.220, 0.018, 0.038)),
        origin=Origin(xyz=(0.0, -0.0260, 0.020)),
        material=brushed_steel,
        name="side_guide_rail",
    )
    for x in (-0.116, 0.116):
        head.visual(
            Box((0.012, 0.030, 0.060)),
            origin=Origin(xyz=(x, -0.0260, 0.020)),
            material=cast_iron,
            name=f"side_stop_{0 if x < 0 else 1}",
        )

    slide = model.part("slide")
    slide.visual(
        Box((0.140, 0.045, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=cast_iron,
        name="slide_block",
    )
    slide.visual(
        Box((0.120, 0.035, 0.012)),
        origin=Origin(xyz=(0.0, -0.004, 0.0335)),
        material=brushed_steel,
        name="top_table",
    )
    for x in (-0.035, 0.035):
        slide.visual(
            Box((0.020, 0.006, 0.003)),
            origin=Origin(xyz=(x, -0.004, 0.0410)),
            material=black_rubber,
            name=f"table_slot_{0 if x < 0 else 1}",
        )

    model.articulation(
        "mast_to_head",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=head,
        origin=Origin(xyz=(0.0, -0.0645, 0.200)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=0.0, upper=0.340),
    )
    model.articulation(
        "head_to_slide",
        ArticulationType.PRISMATIC,
        parent=head,
        child=slide,
        origin=Origin(xyz=(0.0, -0.0575, 0.025)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.12, lower=-0.080, upper=0.080),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    head = object_model.get_part("head")
    slide = object_model.get_part("slide")
    vertical = object_model.get_articulation("mast_to_head")
    sideways = object_model.get_articulation("head_to_slide")

    ctx.check(
        "stage uses two prismatic joints",
        vertical.articulation_type == ArticulationType.PRISMATIC
        and sideways.articulation_type == ArticulationType.PRISMATIC,
        details=f"vertical={vertical.articulation_type}, sideways={sideways.articulation_type}",
    )
    ctx.expect_overlap(
        head,
        mast,
        axes="z",
        elem_a="carriage_plate",
        elem_b="vertical_rail_0",
        min_overlap=0.10,
        name="head remains engaged on vertical rail at low travel",
    )
    ctx.expect_gap(
        mast,
        head,
        axis="y",
        positive_elem="vertical_rail_0",
        negative_elem="carriage_plate",
        max_gap=0.001,
        max_penetration=0.001,
        name="head carriage rides against vertical rail",
    )
    ctx.expect_overlap(
        slide,
        head,
        axes="x",
        elem_a="slide_block",
        elem_b="side_guide_rail",
        min_overlap=0.12,
        name="side slide is centered on transverse guide",
    )
    ctx.expect_gap(
        head,
        slide,
        axis="y",
        positive_elem="side_guide_rail",
        negative_elem="slide_block",
        max_gap=0.001,
        max_penetration=0.001,
        name="side slide is seated on guide rail",
    )

    low_head_position = ctx.part_world_position(head)
    centered_slide_position = ctx.part_world_position(slide)
    with ctx.pose({vertical: 0.340, sideways: 0.080}):
        ctx.expect_overlap(
            head,
            mast,
            axes="z",
            elem_a="carriage_plate",
            elem_b="vertical_rail_0",
            min_overlap=0.10,
            name="raised head remains engaged on vertical rail",
        )
        ctx.expect_overlap(
            slide,
            head,
            axes="x",
            elem_a="slide_block",
            elem_b="side_guide_rail",
            min_overlap=0.04,
            name="offset slide retains transverse guide engagement",
        )
        raised_head_position = ctx.part_world_position(head)
        offset_slide_position = ctx.part_world_position(slide)

    ctx.check(
        "vertical stage raises the head",
        low_head_position is not None
        and raised_head_position is not None
        and raised_head_position[2] > low_head_position[2] + 0.30,
        details=f"low={low_head_position}, raised={raised_head_position}",
    )
    ctx.check(
        "sideways stage shifts the slide",
        centered_slide_position is not None
        and offset_slide_position is not None
        and offset_slide_position[0] > centered_slide_position[0] + 0.07,
        details=f"center={centered_slide_position}, offset={offset_slide_position}",
    )

    return ctx.report()


object_model = build_object_model()
