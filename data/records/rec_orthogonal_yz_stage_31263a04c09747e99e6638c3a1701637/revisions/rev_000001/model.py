from __future__ import annotations

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
    model = ArticulatedObject(name="wall_backed_transfer_stage")

    plate_mat = Material("powder_coated_plate", rgba=(0.72, 0.75, 0.76, 1.0))
    rail_mat = Material("dark_hardened_rail", rgba=(0.10, 0.11, 0.12, 1.0))
    saddle_mat = Material("blue_anodized_saddle", rgba=(0.05, 0.20, 0.55, 1.0))
    carriage_mat = Material("orange_carriage", rgba=(0.95, 0.42, 0.10, 1.0))
    bolt_mat = Material("brushed_bolt_heads", rgba=(0.42, 0.43, 0.42, 1.0))

    plate = model.part("wall_plate")
    plate.visual(
        Box((1.35, 0.060, 1.60)),
        origin=Origin(xyz=(0.0, 0.0, 0.80)),
        material=plate_mat,
        name="back_plate",
    )
    plate.visual(
        Box((1.12, 0.040, 0.055)),
        origin=Origin(xyz=(0.0, -0.050, 1.18)),
        material=rail_mat,
        name="top_rail",
    )
    plate.visual(
        Box((1.12, 0.040, 0.055)),
        origin=Origin(xyz=(0.0, -0.050, 0.72)),
        material=rail_mat,
        name="bottom_rail",
    )
    for x in (-0.62, 0.62):
        plate.visual(
            Box((0.045, 0.050, 0.60)),
            origin=Origin(xyz=(x, -0.055, 0.95)),
            material=rail_mat,
            name=f"travel_stop_{'neg' if x < 0 else 'pos'}",
        )
    for x in (-0.58, 0.58):
        for z in (0.22, 0.52, 1.08, 1.38):
            plate.visual(
                Cylinder(radius=0.018, length=0.012),
                origin=Origin(xyz=(x, -0.036, z), rpy=(1.5708, 0.0, 0.0)),
                material=bolt_mat,
                name=f"wall_bolt_{'neg' if x < 0 else 'pos'}_{int(z * 100):03d}",
            )

    saddle = model.part("saddle")
    saddle.visual(
        Box((0.28, 0.035, 0.42)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=saddle_mat,
        name="saddle_plate",
    )
    saddle.visual(
        Box((0.22, 0.035, 0.075)),
        origin=Origin(xyz=(0.0, 0.0, 0.23)),
        material=saddle_mat,
        name="top_shoe",
    )
    saddle.visual(
        Box((0.22, 0.035, 0.075)),
        origin=Origin(xyz=(0.0, 0.0, -0.23)),
        material=saddle_mat,
        name="bottom_shoe",
    )
    saddle.visual(
        Box((0.10, 0.025, 0.46)),
        origin=Origin(xyz=(0.0, -0.030, -0.34)),
        material=rail_mat,
        name="vertical_guide",
    )
    saddle.visual(
        Box((0.16, 0.030, 0.13)),
        origin=Origin(xyz=(0.0, -0.020, -0.17)),
        material=saddle_mat,
        name="drop_bracket",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.16, 0.040, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=carriage_mat,
        name="slide_block",
    )
    carriage.visual(
        Box((0.20, 0.025, 0.12)),
        origin=Origin(xyz=(0.0, -0.032, -0.03)),
        material=carriage_mat,
        name="front_plate",
    )
    carriage.visual(
        Box((0.10, 0.050, 0.08)),
        origin=Origin(xyz=(0.0, -0.008, -0.13)),
        material=rail_mat,
        name="tool_mount",
    )

    model.articulation(
        "plate_to_saddle",
        ArticulationType.PRISMATIC,
        parent=plate,
        child=saddle,
        origin=Origin(xyz=(0.0, -0.0875, 0.95)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.35, lower=-0.34, upper=0.34),
    )
    model.articulation(
        "saddle_to_carriage",
        ArticulationType.PRISMATIC,
        parent=saddle,
        child=carriage,
        origin=Origin(xyz=(0.0, -0.0625, -0.32)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.25, lower=0.0, upper=0.28),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    plate = object_model.get_part("wall_plate")
    saddle = object_model.get_part("saddle")
    carriage = object_model.get_part("carriage")
    h_slide = object_model.get_articulation("plate_to_saddle")
    v_slide = object_model.get_articulation("saddle_to_carriage")

    ctx.check(
        "stage has two moving prismatic axes",
        len(object_model.articulations) == 2
        and all(j.articulation_type == ArticulationType.PRISMATIC for j in object_model.articulations),
    )
    ctx.expect_contact(
        saddle,
        plate,
        elem_a="top_shoe",
        elem_b="top_rail",
        contact_tol=0.001,
        name="upper saddle shoe rides on the upper rail",
    )
    ctx.expect_contact(
        saddle,
        plate,
        elem_a="bottom_shoe",
        elem_b="bottom_rail",
        contact_tol=0.001,
        name="lower saddle shoe rides on the lower rail",
    )
    ctx.expect_contact(
        carriage,
        saddle,
        elem_a="slide_block",
        elem_b="vertical_guide",
        contact_tol=0.001,
        name="vertical carriage bears on the drop guide",
    )

    saddle_rest = ctx.part_world_position(saddle)
    carriage_rest = ctx.part_world_position(carriage)

    with ctx.pose({h_slide: 0.34}):
        saddle_right = ctx.part_world_position(saddle)
        ctx.expect_overlap(
            saddle,
            plate,
            axes="z",
            elem_a="top_shoe",
            elem_b="top_rail",
            min_overlap=0.05,
            name="right-travel saddle remains engaged with top rail",
        )
    with ctx.pose({h_slide: -0.34}):
        saddle_left = ctx.part_world_position(saddle)
        ctx.expect_overlap(
            saddle,
            plate,
            axes="z",
            elem_a="bottom_shoe",
            elem_b="bottom_rail",
            min_overlap=0.05,
            name="left-travel saddle remains engaged with bottom rail",
        )
    with ctx.pose({v_slide: 0.28}):
        carriage_lowered = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            carriage,
            saddle,
            axes="z",
            elem_a="slide_block",
            elem_b="vertical_guide",
            min_overlap=0.05,
            name="lowered carriage stays captured on vertical guide",
        )

    ctx.check(
        "saddle moves left and right",
        saddle_rest is not None
        and saddle_left is not None
        and saddle_right is not None
        and saddle_left[0] < saddle_rest[0] - 0.25
        and saddle_right[0] > saddle_rest[0] + 0.25,
        details=f"left={saddle_left}, rest={saddle_rest}, right={saddle_right}",
    )
    ctx.check(
        "carriage moves downward on vertical slide",
        carriage_rest is not None
        and carriage_lowered is not None
        and carriage_lowered[2] < carriage_rest[2] - 0.20,
        details=f"rest={carriage_rest}, lowered={carriage_lowered}",
    )

    return ctx.report()


object_model = build_object_model()
