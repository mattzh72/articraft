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
    model = ArticulatedObject(name="bridge_backed_transfer_arm")

    dark_steel = Material("dark_blued_steel", color=(0.08, 0.10, 0.12, 1.0))
    painted_steel = Material("painted_steel", color=(0.16, 0.19, 0.21, 1.0))
    rail_chrome = Material("ground_guide_rail", color=(0.62, 0.66, 0.68, 1.0))
    carriage_blue = Material("blue_carriage_casting", color=(0.05, 0.20, 0.43, 1.0))
    hinge_yellow = Material("yellow_hinge_frame", color=(0.92, 0.62, 0.12, 1.0))
    ram_steel = Material("brushed_output_ram", color=(0.72, 0.74, 0.73, 1.0))
    nose_orange = Material("orange_transfer_nose", color=(0.95, 0.28, 0.06, 1.0))
    rubber = Material("black_rubber", color=(0.015, 0.015, 0.014, 1.0))

    support = model.part("rear_support")
    support.visual(
        Box((0.88, 0.56, 0.05)),
        origin=Origin(xyz=(-0.05, 0.0, 0.025)),
        material=dark_steel,
        name="floor_foot",
    )
    support.visual(
        Box((0.08, 0.07, 0.75)),
        origin=Origin(xyz=(-0.45, -0.22, 0.40)),
        material=painted_steel,
        name="post_0",
    )
    support.visual(
        Box((0.08, 0.07, 0.75)),
        origin=Origin(xyz=(-0.45, 0.22, 0.40)),
        material=painted_steel,
        name="post_1",
    )
    support.visual(
        Box((0.11, 0.52, 0.065)),
        origin=Origin(xyz=(-0.45, 0.0, 0.775)),
        material=painted_steel,
        name="rear_bridge",
    )
    support.visual(
        Box((0.84, 0.035, 0.035)),
        origin=Origin(xyz=(-0.04, -0.13, 0.755)),
        material=rail_chrome,
        name="guide_rail_0",
    )
    support.visual(
        Box((0.84, 0.035, 0.035)),
        origin=Origin(xyz=(-0.04, 0.13, 0.755)),
        material=rail_chrome,
        name="guide_rail_1",
    )
    support.visual(
        Box((0.06, 0.31, 0.08)),
        origin=Origin(xyz=(0.38, 0.0, 0.755)),
        material=painted_steel,
        name="front_tie",
    )
    support.visual(
        Box((0.54, 0.045, 0.04)),
        origin=Origin(xyz=(-0.18, -0.22, 1.02)),
        material=painted_steel,
        name="upper_backbone_0",
    )
    support.visual(
        Box((0.54, 0.045, 0.04)),
        origin=Origin(xyz=(-0.18, 0.22, 1.02)),
        material=painted_steel,
        name="upper_backbone_1",
    )
    support.visual(
        Box((0.06, 0.48, 0.045)),
        origin=Origin(xyz=(0.08, 0.0, 1.02)),
        material=painted_steel,
        name="bridge_cap",
    )
    support.visual(
        Box((0.05, 0.045, 0.215)),
        origin=Origin(xyz=(-0.45, -0.22, 0.905)),
        material=painted_steel,
        name="bridge_riser_0",
    )
    support.visual(
        Box((0.05, 0.045, 0.215)),
        origin=Origin(xyz=(-0.45, 0.22, 0.905)),
        material=painted_steel,
        name="bridge_riser_1",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.20, 0.34, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=carriage_blue,
        name="saddle_plate",
    )
    carriage.visual(
        Box((0.17, 0.24, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=carriage_blue,
        name="lower_saddle",
    )
    carriage.visual(
        Box((0.16, 0.035, 0.070)),
        origin=Origin(xyz=(0.0, -0.13, 0.035)),
        material=carriage_blue,
        name="roller_cheek_0",
    )
    carriage.visual(
        Cylinder(radius=0.026, length=0.052),
        origin=Origin(xyz=(-0.055, -0.13, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="roller_0_0",
    )
    carriage.visual(
        Cylinder(radius=0.026, length=0.052),
        origin=Origin(xyz=(0.055, -0.13, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="roller_0_1",
    )
    carriage.visual(
        Box((0.16, 0.035, 0.070)),
        origin=Origin(xyz=(0.0, 0.13, 0.035)),
        material=carriage_blue,
        name="roller_cheek_1",
    )
    carriage.visual(
        Cylinder(radius=0.026, length=0.052),
        origin=Origin(xyz=(-0.055, 0.13, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="roller_1_0",
    )
    carriage.visual(
        Cylinder(radius=0.026, length=0.052),
        origin=Origin(xyz=(0.055, 0.13, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="roller_1_1",
    )
    carriage.visual(
        Box((0.060, 0.16, 0.070)),
        origin=Origin(xyz=(0.075, 0.0, 0.095)),
        material=carriage_blue,
        name="hinge_pedestal",
    )
    carriage.visual(
        Box((0.055, 0.025, 0.105)),
        origin=Origin(xyz=(0.130, -0.075, 0.130)),
        material=carriage_blue,
        name="hinge_lug_0",
    )
    carriage.visual(
        Box((0.055, 0.025, 0.105)),
        origin=Origin(xyz=(0.130, 0.075, 0.130)),
        material=carriage_blue,
        name="hinge_lug_1",
    )
    carriage.visual(
        Cylinder(radius=0.012, length=0.21),
        origin=Origin(xyz=(0.130, 0.0, 0.130), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rail_chrome,
        name="hinge_pin",
    )

    frame = model.part("hinged_frame")
    frame.visual(
        Cylinder(radius=0.024, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hinge_yellow,
        name="hinge_barrel",
    )
    frame.visual(
        Box((0.065, 0.088, 0.012)),
        origin=Origin(xyz=(0.030, 0.0, 0.026)),
        material=hinge_yellow,
        name="rear_web",
    )
    frame.visual(
        Box((0.065, 0.088, 0.012)),
        origin=Origin(xyz=(0.030, 0.0, -0.026)),
        material=hinge_yellow,
        name="lower_rear_web",
    )
    frame.visual(
        Box((0.070, 0.170, 0.050)),
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
        material=hinge_yellow,
        name="rear_crossbar",
    )
    frame.visual(
        Box((0.43, 0.025, 0.034)),
        origin=Origin(xyz=(0.255, -0.075, 0.0)),
        material=hinge_yellow,
        name="side_rail_0",
    )
    frame.visual(
        Box((0.43, 0.025, 0.034)),
        origin=Origin(xyz=(0.255, 0.075, 0.0)),
        material=hinge_yellow,
        name="side_rail_1",
    )
    frame.visual(
        Box((0.035, 0.025, 0.090)),
        origin=Origin(xyz=(0.440, -0.075, 0.0)),
        material=hinge_yellow,
        name="front_upright_0",
    )
    frame.visual(
        Box((0.035, 0.025, 0.090)),
        origin=Origin(xyz=(0.440, 0.075, 0.0)),
        material=hinge_yellow,
        name="front_upright_1",
    )
    frame.visual(
        Box((0.18, 0.13, 0.015)),
        origin=Origin(xyz=(0.510, 0.0, 0.0475)),
        material=hinge_yellow,
        name="ram_sleeve_top",
    )
    frame.visual(
        Box((0.18, 0.13, 0.015)),
        origin=Origin(xyz=(0.510, 0.0, -0.0475)),
        material=hinge_yellow,
        name="ram_sleeve_bottom",
    )
    frame.visual(
        Box((0.18, 0.015, 0.080)),
        origin=Origin(xyz=(0.510, -0.065, 0.0)),
        material=hinge_yellow,
        name="ram_sleeve_side_0",
    )
    frame.visual(
        Box((0.18, 0.015, 0.080)),
        origin=Origin(xyz=(0.510, 0.065, 0.0)),
        material=hinge_yellow,
        name="ram_sleeve_side_1",
    )

    ram = model.part("output_ram")
    ram.visual(
        Box((0.260, 0.050, 0.032)),
        origin=Origin(xyz=(0.100, 0.0, 0.0)),
        material=ram_steel,
        name="ram_bar",
    )
    ram.visual(
        Box((0.120, 0.040, 0.024)),
        origin=Origin(xyz=(0.060, 0.0, -0.028)),
        material=ram_steel,
        name="lower_glide_shoe",
    )
    ram.visual(
        Cylinder(radius=0.034, length=0.035),
        origin=Origin(xyz=(0.2475, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=nose_orange,
        name="nose_pad",
    )

    model.articulation(
        "support_to_carriage",
        ArticulationType.PRISMATIC,
        parent=support,
        child=carriage,
        origin=Origin(xyz=(-0.24, 0.0, 0.7985)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.35, lower=0.0, upper=0.30),
    )
    model.articulation(
        "carriage_to_frame",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=frame,
        origin=Origin(xyz=(0.130, 0.0, 0.130)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.4, lower=0.0, upper=1.15),
    )
    model.articulation(
        "frame_to_ram",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=ram,
        origin=Origin(xyz=(0.440, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.45, lower=0.0, upper=0.16),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("rear_support")
    carriage = object_model.get_part("carriage")
    frame = object_model.get_part("hinged_frame")
    ram = object_model.get_part("output_ram")
    carriage_slide = object_model.get_articulation("support_to_carriage")
    frame_hinge = object_model.get_articulation("carriage_to_frame")
    ram_slide = object_model.get_articulation("frame_to_ram")

    ctx.check(
        "joint chain is prismatic revolute prismatic",
        carriage_slide.articulation_type == ArticulationType.PRISMATIC
        and frame_hinge.articulation_type == ArticulationType.REVOLUTE
        and ram_slide.articulation_type == ArticulationType.PRISMATIC,
        details=(
            f"{carriage_slide.articulation_type}, "
            f"{frame_hinge.articulation_type}, {ram_slide.articulation_type}"
        ),
    )
    ctx.allow_overlap(
        carriage,
        frame,
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        reason="The hinge pin is intentionally captured through the hinged frame barrel.",
    )
    ctx.expect_within(
        carriage,
        frame,
        axes="xz",
        inner_elem="hinge_pin",
        outer_elem="hinge_barrel",
        margin=0.001,
        name="hinge pin is centered inside the barrel",
    )
    ctx.expect_overlap(
        carriage,
        frame,
        axes="y",
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        min_overlap=0.085,
        name="hinge pin spans the frame barrel",
    )
    ctx.expect_gap(
        carriage,
        support,
        axis="z",
        positive_elem="roller_0_0",
        negative_elem="guide_rail_0",
        min_gap=-0.001,
        max_gap=0.002,
        name="carriage rollers ride on rear guide rail",
    )
    ctx.expect_within(
        ram,
        frame,
        axes="y",
        inner_elem="ram_bar",
        outer_elem="ram_sleeve_top",
        margin=0.004,
        name="output ram stays centered between sleeve sides",
    )
    ctx.expect_within(
        ram,
        frame,
        axes="z",
        inner_elem="ram_bar",
        outer_elem="ram_sleeve_side_0",
        margin=0.004,
        name="output ram stays centered between sleeve caps",
    )
    ctx.expect_contact(
        ram,
        frame,
        elem_a="lower_glide_shoe",
        elem_b="ram_sleeve_bottom",
        contact_tol=0.001,
        name="lower glide shoe bears on the ram sleeve",
    )
    ctx.expect_overlap(
        ram,
        frame,
        axes="x",
        elem_a="ram_bar",
        elem_b="ram_sleeve_top",
        min_overlap=0.04,
        name="retracted ram remains captured in sleeve",
    )

    rest_carriage = ctx.part_world_position(carriage)
    rest_ram = ctx.part_world_position(ram)
    with ctx.pose({carriage_slide: 0.30, ram_slide: 0.16}):
        extended_carriage = ctx.part_world_position(carriage)
        extended_ram = ctx.part_world_position(ram)
        ctx.expect_overlap(
            ram,
            frame,
            axes="x",
            elem_a="ram_bar",
            elem_b="ram_sleeve_top",
            min_overlap=0.025,
            name="extended ram remains retained by sleeve",
        )
    ctx.check(
        "carriage slide moves forward",
        rest_carriage is not None
        and extended_carriage is not None
        and extended_carriage[0] > rest_carriage[0] + 0.25,
        details=f"rest={rest_carriage}, extended={extended_carriage}",
    )
    ctx.check(
        "nose ram slides out from hinged frame",
        rest_ram is not None and extended_ram is not None and extended_ram[0] > rest_ram[0] + 0.14,
        details=f"rest={rest_ram}, extended={extended_ram}",
    )

    with ctx.pose({frame_hinge: 0.9}):
        ctx.expect_origin_gap(
            ram,
            frame,
            axis="z",
            min_gap=0.0,
            name="positive hinge motion lifts the output end",
        )

    return ctx.report()


object_model = build_object_model()
