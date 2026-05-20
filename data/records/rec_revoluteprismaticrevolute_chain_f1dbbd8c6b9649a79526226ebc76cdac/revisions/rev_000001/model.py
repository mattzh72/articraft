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
    model = ArticulatedObject(name="service_manipulator")

    dark_cast = model.material("dark_cast", color=(0.08, 0.09, 0.10, 1.0))
    satin_metal = model.material("satin_metal", color=(0.62, 0.66, 0.68, 1.0))
    blue_housing = model.material("blue_housing", color=(0.10, 0.25, 0.45, 1.0))
    amber_guard = model.material("amber_guard", color=(0.95, 0.58, 0.12, 1.0))
    black_rubber = model.material("black_rubber", color=(0.015, 0.015, 0.014, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.34, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=dark_cast,
        name="floor_plate",
    )
    base.visual(
        Cylinder(radius=0.24, length=0.065),
        origin=Origin(xyz=(0.0, 0.0, 0.0775)),
        material=satin_metal,
        name="bearing_boss",
    )
    base.visual(
        Cylinder(radius=0.30, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=black_rubber,
        name="base_foot",
    )

    turret = model.part("turret")
    turret.visual(
        Cylinder(radius=0.205, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=blue_housing,
        name="turntable_cap",
    )
    turret.visual(
        Cylinder(radius=0.085, length=0.455),
        origin=Origin(xyz=(0.0, 0.0, 0.2875)),
        material=blue_housing,
        name="riser_column",
    )
    turret.visual(
        Box((0.19, 0.24, 0.21)),
        origin=Origin(xyz=(0.040, 0.0, 0.545)),
        material=blue_housing,
        name="shoulder_housing",
    )
    turret.visual(
        Box((0.625, 0.180, 0.025)),
        origin=Origin(xyz=(0.4425, 0.0, 0.6275)),
        material=blue_housing,
        name="top_rail",
    )
    turret.visual(
        Box((0.625, 0.180, 0.025)),
        origin=Origin(xyz=(0.4425, 0.0, 0.4725)),
        material=blue_housing,
        name="bottom_rail",
    )
    turret.visual(
        Box((0.625, 0.026, 0.135)),
        origin=Origin(xyz=(0.4425, 0.077, 0.550)),
        material=blue_housing,
        name="side_rail_0",
    )
    turret.visual(
        Box((0.625, 0.026, 0.135)),
        origin=Origin(xyz=(0.4425, -0.077, 0.550)),
        material=blue_housing,
        name="side_rail_1",
    )
    turret.visual(
        Box((0.030, 0.180, 0.025)),
        origin=Origin(xyz=(0.760, 0.0, 0.6275)),
        material=amber_guard,
        name="front_top_lip",
    )
    turret.visual(
        Box((0.030, 0.180, 0.025)),
        origin=Origin(xyz=(0.760, 0.0, 0.4725)),
        material=amber_guard,
        name="front_bottom_lip",
    )
    turret.visual(
        Box((0.030, 0.026, 0.135)),
        origin=Origin(xyz=(0.760, 0.077, 0.550)),
        material=amber_guard,
        name="front_side_lip_0",
    )
    turret.visual(
        Box((0.030, 0.026, 0.135)),
        origin=Origin(xyz=(0.760, -0.077, 0.550)),
        material=amber_guard,
        name="front_side_lip_1",
    )

    slide = model.part("slide")
    slide.visual(
        Box((0.720, 0.090, 0.070)),
        origin=Origin(xyz=(0.360, 0.0, 0.0)),
        material=satin_metal,
        name="inner_member",
    )
    slide.visual(
        Box((0.240, 0.078, 0.030)),
        origin=Origin(xyz=(0.350, 0.0, 0.050)),
        material=black_rubber,
        name="top_slide_shoe",
    )
    slide.visual(
        Box((0.240, 0.078, 0.030)),
        origin=Origin(xyz=(0.350, 0.0, -0.050)),
        material=black_rubber,
        name="bottom_slide_shoe",
    )
    slide.visual(
        Box((0.240, 0.019, 0.058)),
        origin=Origin(xyz=(0.350, 0.0545, 0.0)),
        material=black_rubber,
        name="side_slide_shoe_0",
    )
    slide.visual(
        Box((0.240, 0.019, 0.058)),
        origin=Origin(xyz=(0.350, -0.0545, 0.0)),
        material=black_rubber,
        name="side_slide_shoe_1",
    )
    slide.visual(
        Box((0.160, 0.150, 0.120)),
        origin=Origin(xyz=(0.780, 0.0, 0.0)),
        material=satin_metal,
        name="wrist_carriage",
    )
    slide.visual(
        Box((0.045, 0.150, 0.085)),
        origin=Origin(xyz=(0.880, 0.0, 0.0)),
        material=satin_metal,
        name="yoke_bridge",
    )
    slide.visual(
        Box((0.110, 0.024, 0.170)),
        origin=Origin(xyz=(0.940, 0.076, 0.0)),
        material=satin_metal,
        name="yoke_cheek_0",
    )
    slide.visual(
        Box((0.110, 0.024, 0.170)),
        origin=Origin(xyz=(0.940, -0.076, 0.0)),
        material=satin_metal,
        name="yoke_cheek_1",
    )

    wrist = model.part("wrist")
    wrist.visual(
        Cylinder(radius=0.034, length=0.116),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_cast,
        name="hinge_barrel",
    )
    wrist.visual(
        Cylinder(radius=0.010, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="hinge_pin",
    )
    wrist.visual(
        Box((0.205, 0.100, 0.080)),
        origin=Origin(xyz=(0.1325, 0.0, 0.0)),
        material=blue_housing,
        name="wrist_housing",
    )
    wrist.visual(
        Box((0.045, 0.160, 0.125)),
        origin=Origin(xyz=(0.2575, 0.0, 0.0)),
        material=dark_cast,
        name="tool_plate",
    )
    wrist.visual(
        Cylinder(radius=0.035, length=0.010),
        origin=Origin(xyz=(0.2825, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=amber_guard,
        name="tool_bolt_pattern",
    )

    model.articulation(
        "base_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=turret,
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=1.2, lower=-2.8, upper=2.8),
    )
    model.articulation(
        "slide_extension",
        ArticulationType.PRISMATIC,
        parent=turret,
        child=slide,
        origin=Origin(xyz=(0.170, 0.0, 0.550)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.35, lower=0.0, upper=0.250),
    )
    model.articulation(
        "wrist_pitch",
        ArticulationType.REVOLUTE,
        parent=slide,
        child=wrist,
        origin=Origin(xyz=(0.940, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.8, lower=-1.2, upper=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    turret = object_model.get_part("turret")
    slide = object_model.get_part("slide")
    wrist = object_model.get_part("wrist")
    base_yaw = object_model.get_articulation("base_yaw")
    slide_extension = object_model.get_articulation("slide_extension")
    wrist_pitch = object_model.get_articulation("wrist_pitch")

    ctx.allow_overlap(
        wrist,
        slide,
        elem_a="hinge_pin",
        elem_b="yoke_cheek_0",
        reason="The wrist pin is intentionally captured through the simplified solid yoke cheek.",
    )
    ctx.allow_overlap(
        wrist,
        slide,
        elem_a="hinge_pin",
        elem_b="yoke_cheek_1",
        reason="The wrist pin is intentionally captured through the simplified solid yoke cheek.",
    )
    ctx.expect_overlap(
        wrist,
        slide,
        axes="y",
        min_overlap=0.010,
        elem_a="hinge_pin",
        elem_b="yoke_cheek_0",
        name="positive yoke cheek captures hinge pin",
    )
    ctx.expect_overlap(
        wrist,
        slide,
        axes="y",
        min_overlap=0.010,
        elem_a="hinge_pin",
        elem_b="yoke_cheek_1",
        name="negative yoke cheek captures hinge pin",
    )

    ctx.expect_gap(
        turret,
        slide,
        axis="z",
        min_gap=0.020,
        positive_elem="top_rail",
        negative_elem="inner_member",
        name="slide clears top guide rail",
    )
    ctx.expect_gap(
        slide,
        turret,
        axis="z",
        min_gap=0.020,
        positive_elem="inner_member",
        negative_elem="bottom_rail",
        name="slide clears lower guide rail",
    )
    ctx.expect_gap(
        turret,
        slide,
        axis="y",
        min_gap=0.015,
        positive_elem="side_rail_0",
        negative_elem="inner_member",
        name="slide clears positive side guide",
    )
    ctx.expect_gap(
        slide,
        turret,
        axis="y",
        min_gap=0.015,
        positive_elem="inner_member",
        negative_elem="side_rail_1",
        name="slide clears negative side guide",
    )
    ctx.expect_overlap(
        slide,
        turret,
        axes="x",
        min_overlap=0.55,
        elem_a="inner_member",
        elem_b="top_rail",
        name="retracted slide is deeply retained",
    )

    slide_rest = ctx.part_world_position(slide)
    with ctx.pose({slide_extension: 0.250}):
        slide_extended = ctx.part_world_position(slide)
        ctx.expect_overlap(
            slide,
            turret,
            axes="x",
            min_overlap=0.30,
            elem_a="inner_member",
            elem_b="top_rail",
            name="extended slide remains retained",
        )

    ctx.check(
        "telescoping slide extends along boom",
        slide_rest is not None
        and slide_extended is not None
        and slide_extended[0] > slide_rest[0] + 0.20,
        details=f"rest={slide_rest}, extended={slide_extended}",
    )

    wrist_rest = ctx.part_element_world_aabb(wrist, elem="tool_plate")
    with ctx.pose({wrist_pitch: -0.8}):
        wrist_raised = ctx.part_element_world_aabb(wrist, elem="tool_plate")
    ctx.check(
        "wrist hinge pitches the tool plate",
        wrist_rest is not None
        and wrist_raised is not None
        and wrist_raised[1][2] > wrist_rest[1][2] + 0.08,
        details=f"rest={wrist_rest}, raised={wrist_raised}",
    )

    wrist_center = ctx.part_world_position(wrist)
    with ctx.pose({base_yaw: 0.75}):
        yawed_wrist_center = ctx.part_world_position(wrist)
    ctx.check(
        "rotary base sweeps the arm about z",
        wrist_center is not None
        and yawed_wrist_center is not None
        and yawed_wrist_center[1] > wrist_center[1] + 0.55,
        details=f"rest={wrist_center}, yawed={yawed_wrist_center}",
    )

    return ctx.report()


object_model = build_object_model()
