from __future__ import annotations

import math

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
    model = ArticulatedObject(name="turntable_z_elevator")

    cast_grey = Material("cast_dark_grey", rgba=(0.16, 0.17, 0.18, 1.0))
    black = Material("matte_black", rgba=(0.025, 0.027, 0.030, 1.0))
    steel = Material("brushed_steel", rgba=(0.72, 0.74, 0.73, 1.0))
    blue = Material("anodized_blue", rgba=(0.08, 0.18, 0.36, 1.0))
    brass = Material("oiled_bronze", rgba=(0.70, 0.50, 0.22, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.320, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=black,
        name="floor_flange",
    )
    pedestal.visual(
        Cylinder(radius=0.275, length=0.155),
        origin=Origin(xyz=(0.0, 0.0, 0.1125)),
        material=cast_grey,
        name="base_housing",
    )
    pedestal.visual(
        Cylinder(radius=0.235, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.2075)),
        material=cast_grey,
        name="top_collar",
    )
    pedestal.visual(
        Box((0.155, 0.014, 0.072)),
        origin=Origin(xyz=(0.0, -0.277, 0.105)),
        material=black,
        name="service_panel",
    )
    pedestal.visual(
        Box((0.040, 0.022, 0.028)),
        origin=Origin(xyz=(0.0, -0.292, 0.060)),
        material=black,
        name="cable_gland",
    )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.205, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=steel,
        name="disk",
    )
    turntable.visual(
        Cylinder(radius=0.075, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0525)),
        material=black,
        name="center_boss",
    )
    for index, (x, y) in enumerate(
        (
            (0.135, 0.000),
            (0.000, 0.135),
            (-0.135, 0.000),
            (0.000, -0.135),
        )
    ):
        turntable.visual(
            Cylinder(radius=0.014, length=0.010),
            origin=Origin(xyz=(x, y, 0.034)),
            material=black,
            name=f"socket_head_{index}",
        )

    elevator_frame = model.part("elevator_frame")
    elevator_frame.visual(
        Box((0.340, 0.190, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=blue,
        name="mount_plate",
    )
    elevator_frame.visual(
        Box((0.075, 0.060, 0.700)),
        origin=Origin(xyz=(0.0, 0.075, 0.400)),
        material=blue,
        name="rear_spine",
    )
    elevator_frame.visual(
        Cylinder(radius=0.013, length=0.725),
        origin=Origin(xyz=(-0.115, 0.0, 0.415)),
        material=steel,
        name="guide_rod_0",
    )
    elevator_frame.visual(
        Box((0.070, 0.030, 0.038)),
        origin=Origin(xyz=(-0.115, 0.0, 0.069)),
        material=black,
        name="lower_clamp_0",
    )
    elevator_frame.visual(
        Box((0.070, 0.030, 0.038)),
        origin=Origin(xyz=(-0.115, 0.0, 0.761)),
        material=black,
        name="upper_clamp_0",
    )
    elevator_frame.visual(
        Cylinder(radius=0.013, length=0.725),
        origin=Origin(xyz=(0.115, 0.0, 0.415)),
        material=steel,
        name="guide_rod_1",
    )
    elevator_frame.visual(
        Box((0.070, 0.030, 0.038)),
        origin=Origin(xyz=(0.115, 0.0, 0.069)),
        material=black,
        name="lower_clamp_1",
    )
    elevator_frame.visual(
        Box((0.070, 0.030, 0.038)),
        origin=Origin(xyz=(0.115, 0.0, 0.761)),
        material=black,
        name="upper_clamp_1",
    )
    elevator_frame.visual(
        Cylinder(radius=0.009, length=0.695),
        origin=Origin(xyz=(0.0, -0.036, 0.425)),
        material=brass,
        name="lead_screw",
    )
    elevator_frame.visual(
        Box((0.360, 0.190, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, 0.785)),
        material=blue,
        name="top_bridge",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.310, 0.044, 0.115)),
        origin=Origin(xyz=(0.0, -0.070, 0.0)),
        material=black,
        name="slide_plate",
    )
    carriage.visual(
        Box((0.072, 0.034, 0.140)),
        origin=Origin(xyz=(-0.115, -0.032, 0.0)),
        material=black,
        name="bearing_block_0",
    )
    carriage.visual(
        Box((0.012, 0.044, 0.140)),
        origin=Origin(xyz=(-0.145, -0.010, 0.0)),
        material=steel,
        name="bearing_ear_0_0",
    )
    carriage.visual(
        Box((0.012, 0.044, 0.140)),
        origin=Origin(xyz=(-0.085, -0.010, 0.0)),
        material=steel,
        name="bearing_ear_0_1",
    )
    carriage.visual(
        Box((0.072, 0.034, 0.140)),
        origin=Origin(xyz=(0.115, -0.032, 0.0)),
        material=black,
        name="bearing_block_1",
    )
    carriage.visual(
        Box((0.012, 0.044, 0.140)),
        origin=Origin(xyz=(0.085, -0.010, 0.0)),
        material=steel,
        name="bearing_ear_1_0",
    )
    carriage.visual(
        Box((0.012, 0.044, 0.140)),
        origin=Origin(xyz=(0.145, -0.010, 0.0)),
        material=steel,
        name="bearing_ear_1_1",
    )
    carriage.visual(
        Box((0.100, 0.035, 0.140)),
        origin=Origin(xyz=(0.0, -0.026, 0.0)),
        material=brass,
        name="screw_nut",
    )
    carriage.visual(
        Box((0.130, 0.060, 0.090)),
        origin=Origin(xyz=(0.0, -0.070, 0.095)),
        material=black,
        name="riser_web",
    )
    carriage.visual(
        Box((0.380, 0.130, 0.038)),
        origin=Origin(xyz=(0.0, -0.125, 0.159)),
        material=blue,
        name="top_plate",
    )

    model.articulation(
        "pedestal_to_turntable",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2),
    )
    model.articulation(
        "turntable_to_frame",
        ArticulationType.FIXED,
        parent=turntable,
        child=elevator_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
    )
    model.articulation(
        "elevator_slide",
        ArticulationType.PRISMATIC,
        parent=elevator_frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.255)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.35, lower=0.0, upper=0.310),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    turntable = object_model.get_part("turntable")
    frame = object_model.get_part("elevator_frame")
    carriage = object_model.get_part("carriage")
    slide = object_model.get_articulation("elevator_slide")
    swivel = object_model.get_articulation("pedestal_to_turntable")

    ctx.allow_overlap(
        carriage,
        frame,
        elem_a="screw_nut",
        elem_b="lead_screw",
        reason="The lead screw is intentionally captured through the carriage nut block.",
    )
    ctx.expect_within(
        frame,
        carriage,
        axes="xy",
        inner_elem="lead_screw",
        outer_elem="screw_nut",
        margin=0.002,
        name="lead screw passes through carriage nut",
    )
    ctx.expect_overlap(
        carriage,
        frame,
        axes="z",
        elem_a="screw_nut",
        elem_b="lead_screw",
        min_overlap=0.120,
        name="carriage nut remains engaged with lead screw",
    )

    ctx.expect_gap(
        turntable,
        pedestal,
        axis="z",
        positive_elem="disk",
        negative_elem="top_collar",
        max_gap=0.001,
        max_penetration=0.0,
        name="turntable disk seats on pedestal collar",
    )
    ctx.expect_overlap(
        turntable,
        pedestal,
        axes="xy",
        elem_a="disk",
        elem_b="top_collar",
        min_overlap=0.20,
        name="turntable disk is centered over cylindrical housing",
    )
    ctx.expect_gap(
        frame,
        turntable,
        axis="z",
        positive_elem="mount_plate",
        negative_elem="center_boss",
        max_gap=0.001,
        max_penetration=0.0005,
        name="elevator frame is bolted to turntable boss",
    )
    ctx.expect_within(
        carriage,
        frame,
        axes="x",
        inner_elem="slide_plate",
        outer_elem="top_bridge",
        margin=0.020,
        name="carriage plate remains between guide uprights",
    )
    ctx.expect_overlap(
        carriage,
        frame,
        axes="z",
        elem_a="bearing_block_0",
        elem_b="guide_rod_0",
        min_overlap=0.120,
        name="lowered carriage has guide engagement",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.310}):
        raised_pos = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            carriage,
            frame,
            axes="z",
            elem_a="bearing_block_0",
            elem_b="guide_rod_0",
            min_overlap=0.120,
            name="raised carriage stays captured on guide rails",
        )
    ctx.check(
        "elevator slide raises the carriage",
        rest_pos is not None
        and raised_pos is not None
        and raised_pos[2] > rest_pos[2] + 0.300,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    with ctx.pose({swivel: math.pi / 2.0}):
        ctx.expect_overlap(
            frame,
            turntable,
            axes="xy",
            elem_a="mount_plate",
            elem_b="disk",
            min_overlap=0.15,
            name="rotated elevator frame remains on turntable",
        )

    return ctx.report()


object_model = build_object_model()
