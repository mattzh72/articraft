from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="hinged_slider_service_mechanism")

    dark_steel = Material("dark_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    blue_paint = Material("blue_paint", rgba=(0.05, 0.18, 0.42, 1.0))
    hard_chrome = Material("hard_chrome", rgba=(0.68, 0.70, 0.72, 1.0))
    bronze = Material("bronze_bushings", rgba=(0.72, 0.48, 0.20, 1.0))
    safety_orange = Material("safety_orange", rgba=(1.0, 0.42, 0.05, 1.0))

    support = model.part("support")
    support.visual(
        Box((0.56, 0.38, 0.04)),
        origin=Origin(xyz=(-0.18, 0.0, 0.02)),
        material=dark_steel,
        name="floor_plate",
    )
    support.visual(
        Box((0.12, 0.28, 0.58)),
        origin=Origin(xyz=(-0.30, 0.0, 0.33)),
        material=blue_paint,
        name="pedestal",
    )
    for index, y in enumerate((-0.13, 0.13)):
        support.visual(
            Box((0.24, 0.045, 0.07)),
            origin=Origin(xyz=(-0.15, y, 0.62)),
            material=blue_paint,
            name=f"yoke_arm_{index}",
        )
        support.visual(
            Box((0.10, 0.045, 0.20)),
            origin=Origin(xyz=(0.02, y, 0.62)),
            material=blue_paint,
            name=f"yoke_cheek_{index}",
        )
    support.visual(
        Cylinder(radius=0.022, length=0.32),
        origin=Origin(xyz=(0.0, 0.0, 0.62), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hard_chrome,
        name="hinge_pin",
    )

    pivot_frame = model.part("pivot_frame")
    pivot_frame.visual(
        Cylinder(radius=0.060, length=0.18),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=bronze,
        name="hinge_barrel",
    )
    pivot_frame.visual(
        Box((0.08, 0.17, 0.06)),
        origin=Origin(xyz=(0.07, 0.0, 0.0)),
        material=blue_paint,
        name="root_crosshead",
    )
    for index, y in enumerate((-0.065, 0.065)):
        pivot_frame.visual(
            Box((0.58, 0.035, 0.040)),
            origin=Origin(xyz=(0.34, y, 0.0)),
            material=blue_paint,
            name=f"guide_rail_{index}",
        )
    for index, y in enumerate((-0.090, 0.090)):
        pivot_frame.visual(
            Box((0.035, 0.035, 0.080)),
            origin=Origin(xyz=(0.6475, y, 0.0)),
            material=blue_paint,
            name=f"front_stop_{index}",
        )

    extension_block = model.part("extension_block")
    extension_block.visual(
        Box((0.28, 0.095, 0.060)),
        origin=Origin(xyz=(0.04, 0.0, 0.0)),
        material=hard_chrome,
        name="slide_body",
    )
    extension_block.visual(
        Box((0.12, 0.090, 0.012)),
        origin=Origin(xyz=(0.04, 0.0, 0.036)),
        material=dark_steel,
        name="top_wear_pad",
    )
    extension_block.visual(
        Cylinder(radius=0.045, length=0.060),
        origin=Origin(xyz=(0.21, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=bronze,
        name="wrist_boss",
    )

    output = model.part("output")
    output.visual(
        Cylinder(radius=0.055, length=0.035),
        origin=Origin(xyz=(0.0175, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="output_flange",
    )
    output.visual(
        Box((0.025, 0.018, 0.050)),
        origin=Origin(xyz=(0.040, 0.0, 0.055)),
        material=safety_orange,
        name="drive_key",
    )

    model.articulation(
        "root_hinge",
        ArticulationType.REVOLUTE,
        parent=support,
        child=pivot_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.62)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2, lower=-0.55, upper=0.85),
    )
    model.articulation(
        "frame_slide",
        ArticulationType.PRISMATIC,
        parent=pivot_frame,
        child=extension_block,
        origin=Origin(xyz=(0.42, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.30, lower=0.0, upper=0.20),
    )
    model.articulation(
        "wrist_roll",
        ArticulationType.REVOLUTE,
        parent=extension_block,
        child=output,
        origin=Origin(xyz=(0.24, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.5, lower=-1.57, upper=1.57),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    support = object_model.get_part("support")
    pivot_frame = object_model.get_part("pivot_frame")
    extension_block = object_model.get_part("extension_block")
    output = object_model.get_part("output")

    root_hinge = object_model.get_articulation("root_hinge")
    frame_slide = object_model.get_articulation("frame_slide")
    wrist_roll = object_model.get_articulation("wrist_roll")

    ctx.allow_overlap(
        support,
        pivot_frame,
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        reason="The fixed hinge pin is intentionally captured inside the pivot-frame barrel proxy.",
    )
    ctx.expect_within(
        support,
        pivot_frame,
        axes="xz",
        inner_elem="hinge_pin",
        outer_elem="hinge_barrel",
        margin=0.001,
        name="hinge pin is centered inside the barrel",
    )
    ctx.expect_overlap(
        support,
        pivot_frame,
        axes="y",
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        min_overlap=0.16,
        name="hinge pin spans the pivot barrel",
    )

    ctx.check(
        "service mechanism has revolute prismatic revolute chain",
        root_hinge.articulation_type == ArticulationType.REVOLUTE
        and frame_slide.articulation_type == ArticulationType.PRISMATIC
        and wrist_roll.articulation_type == ArticulationType.REVOLUTE,
        details="Expected root hinge, frame slide, and distal wrist roll joints.",
    )
    ctx.expect_within(
        extension_block,
        pivot_frame,
        axes="yz",
        inner_elem="slide_body",
        margin=0.002,
        name="extension block sits within the guide channel",
    )
    ctx.expect_overlap(
        extension_block,
        pivot_frame,
        axes="x",
        elem_a="slide_body",
        min_overlap=0.10,
        name="retracted slider remains engaged in the frame",
    )
    ctx.expect_contact(
        output,
        extension_block,
        elem_a="output_flange",
        elem_b="wrist_boss",
        contact_tol=0.001,
        name="output flange seats against wrist boss",
    )

    rest_slider_pos = ctx.part_world_position(extension_block)
    with ctx.pose({frame_slide: 0.20}):
        ctx.expect_overlap(
            extension_block,
            pivot_frame,
            axes="x",
            elem_a="slide_body",
            min_overlap=0.08,
            name="extended slider still retained by guide rails",
        )
        extended_slider_pos = ctx.part_world_position(extension_block)
    ctx.check(
        "extension block translates along the frame",
        rest_slider_pos is not None
        and extended_slider_pos is not None
        and extended_slider_pos[0] > rest_slider_pos[0] + 0.18,
        details=f"rest={rest_slider_pos}, extended={extended_slider_pos}",
    )

    rest_output_aabb = ctx.part_element_world_aabb(output, elem="drive_key")
    with ctx.pose({wrist_roll: 1.0}):
        rolled_output_aabb = ctx.part_element_world_aabb(output, elem="drive_key")
    ctx.check(
        "distal output visibly rolls about its shaft",
        rest_output_aabb is not None
        and rolled_output_aabb is not None
        and rolled_output_aabb[0][1] < rest_output_aabb[0][1] - 0.025,
        details=f"rest={rest_output_aabb}, rolled={rolled_output_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
