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
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="prp_inspection_rig")

    dark = Material("matte_graphite", rgba=(0.08, 0.09, 0.10, 1.0))
    rail = Material("hardened_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    blue = Material("blue_anodized", rgba=(0.04, 0.22, 0.62, 1.0))
    brass = Material("brass_bearing", rgba=(0.80, 0.58, 0.24, 1.0))
    black = Material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    orange = Material("orange_markings", rgba=(1.0, 0.48, 0.08, 1.0))
    glass = Material("smoked_lens", rgba=(0.02, 0.10, 0.16, 0.85))

    base = model.part("base")
    base.visual(
        Box((0.96, 0.34, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=dark,
        name="base_plate",
    )
    base.visual(
        Box((0.84, 0.032, 0.045)),
        origin=Origin(xyz=(0.0, -0.105, 0.0765)),
        material=rail,
        name="rail_0",
    )
    base.visual(
        Box((0.84, 0.032, 0.045)),
        origin=Origin(xyz=(0.0, 0.105, 0.0765)),
        material=rail,
        name="rail_1",
    )
    for idx, x in enumerate((-0.44, 0.44)):
        base.visual(
            Box((0.035, 0.29, 0.065)),
            origin=Origin(xyz=(x, 0.0, 0.0865)),
            material=dark,
            name=f"end_stop_{idx}",
        )
    base.visual(
        Box((0.76, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, -0.172, 0.058)),
        material=orange,
        name="scale_strip",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.23, 0.255, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=blue,
        name="table",
    )
    for idx, y in enumerate((-0.133, 0.133)):
        carriage.visual(
            Box((0.22, 0.018, 0.055)),
            origin=Origin(xyz=(0.0, y, 0.0070)),
            material=blue,
            name=f"side_guard_{idx}",
        )
    carriage.visual(
        Box((0.110, 0.105, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.0690)),
        material=blue,
        name="pedestal",
    )
    for idx, y in enumerate((-0.075, 0.075)):
        carriage.visual(
            Box((0.065, 0.024, 0.116)),
            origin=Origin(xyz=(0.0, y, 0.0920)),
            material=blue,
            name=f"hinge_lug_{idx}",
        )
        carriage.visual(
            Cylinder(radius=0.019, length=0.043),
            origin=Origin(xyz=(0.0, y, 0.145), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=brass,
            name=f"hinge_knuckle_{idx}",
        )
    carriage.visual(
        Cylinder(radius=0.0085, length=0.225),
        origin=Origin(xyz=(0.0, 0.0, 0.145), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=rail,
        name="hinge_pin",
    )

    pivot_frame = model.part("pivot_frame")
    pivot_frame.visual(
        Cylinder(radius=0.017, length=0.078),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="central_barrel",
    )
    pivot_frame.visual(
        Box((0.055, 0.088, 0.016)),
        origin=Origin(xyz=(0.026, 0.0, 0.023)),
        material=blue,
        name="hub_web",
    )
    for idx, y in enumerate((-0.040, 0.040)):
        pivot_frame.visual(
            Box((0.310, 0.018, 0.030)),
            origin=Origin(xyz=(0.170, y, 0.0)),
            material=blue,
            name=f"side_rail_{idx}",
        )
    pivot_frame.visual(
        Box((0.025, 0.090, 0.034)),
        origin=Origin(xyz=(0.205, 0.0, 0.0)),
        material=blue,
        name="cross_brace",
    )
    pivot_frame.visual(
        Box((0.180, 0.090, 0.018)),
        origin=Origin(xyz=(0.380, 0.0, 0.039)),
        material=dark,
        name="sleeve_top",
    )
    pivot_frame.visual(
        Box((0.180, 0.090, 0.018)),
        origin=Origin(xyz=(0.380, 0.0, -0.039)),
        material=dark,
        name="sleeve_bottom",
    )
    for idx, y in enumerate((-0.045, 0.045)):
        pivot_frame.visual(
            Box((0.180, 0.018, 0.078)),
            origin=Origin(xyz=(0.380, y, 0.0)),
            material=dark,
            name=f"sleeve_side_{idx}",
        )

    nose = model.part("nose")
    nose.visual(
        Box((0.340, 0.028, 0.060)),
        origin=Origin(xyz=(0.145, 0.0, 0.0)),
        material=rail,
        name="slide_bar",
    )
    nose.visual(
        Cylinder(radius=0.025, length=0.035),
        origin=Origin(xyz=(0.330, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=black,
        name="front_collar",
    )
    nose.visual(
        Cylinder(radius=0.012, length=0.055),
        origin=Origin(xyz=(0.372, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=rail,
        name="probe_tip",
    )
    nose.visual(
        Sphere(radius=0.014),
        origin=Origin(xyz=(0.408, 0.0, 0.0)),
        material=glass,
        name="probe_lens",
    )

    model.articulation(
        "base_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(-0.30, 0.0, 0.099)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=0.0, upper=0.50),
    )
    model.articulation(
        "hinge",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=pivot_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=28.0, velocity=1.2, lower=-0.45, upper=1.05),
    )
    model.articulation(
        "nose_slide",
        ArticulationType.PRISMATIC,
        parent=pivot_frame,
        child=nose,
        origin=Origin(xyz=(0.310, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=22.0, velocity=0.22, lower=0.0, upper=0.14),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    pivot_frame = object_model.get_part("pivot_frame")
    nose = object_model.get_part("nose")
    base_slide = object_model.get_articulation("base_slide")
    hinge = object_model.get_articulation("hinge")
    nose_slide = object_model.get_articulation("nose_slide")

    ctx.allow_overlap(
        carriage,
        pivot_frame,
        elem_a="hinge_pin",
        elem_b="central_barrel",
        reason="The steel hinge pin is intentionally captured inside the rotating hinge barrel.",
    )

    ctx.check(
        "motion order is prismatic revolute prismatic",
        base_slide.articulation_type == ArticulationType.PRISMATIC
        and hinge.articulation_type == ArticulationType.REVOLUTE
        and nose_slide.articulation_type == ArticulationType.PRISMATIC
        and base_slide.parent == "base"
        and base_slide.child == "carriage"
        and hinge.parent == "carriage"
        and hinge.child == "pivot_frame"
        and nose_slide.parent == "pivot_frame"
        and nose_slide.child == "nose",
        details=(
            f"order={base_slide.parent}->{base_slide.child}, "
            f"{hinge.parent}->{hinge.child}, {nose_slide.parent}->{nose_slide.child}"
        ),
    )

    ctx.expect_gap(
        carriage,
        base,
        axis="z",
        positive_elem="table",
        negative_elem="rail_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="carriage rests on first base rail",
    )
    ctx.expect_gap(
        carriage,
        base,
        axis="z",
        positive_elem="table",
        negative_elem="rail_1",
        max_gap=0.001,
        max_penetration=0.0,
        name="carriage rests on second base rail",
    )
    ctx.expect_overlap(
        carriage,
        pivot_frame,
        axes="xyz",
        elem_a="hinge_pin",
        elem_b="central_barrel",
        min_overlap=0.015,
        name="hinge pin passes through pivot barrel",
    )
    ctx.expect_within(
        nose,
        pivot_frame,
        axes="yz",
        inner_elem="slide_bar",
        margin=0.002,
        name="nose slider stays inside sleeve clearance envelope",
    )
    ctx.expect_overlap(
        nose,
        pivot_frame,
        axes="x",
        elem_a="slide_bar",
        min_overlap=0.16,
        name="collapsed nose retains long sleeve insertion",
    )

    carriage_rest = ctx.part_world_position(carriage)
    with ctx.pose({base_slide: 0.50}):
        carriage_extended = ctx.part_world_position(carriage)
        ctx.expect_within(
            carriage,
            base,
            axes="x",
            inner_elem="table",
            outer_elem="rail_0",
            margin=0.005,
            name="extended carriage stays on rail length",
        )
    ctx.check(
        "base slide moves carriage along positive x",
        carriage_rest is not None
        and carriage_extended is not None
        and carriage_extended[0] > carriage_rest[0] + 0.45,
        details=f"rest={carriage_rest}, extended={carriage_extended}",
    )

    nose_rest = ctx.part_world_position(nose)
    with ctx.pose({nose_slide: 0.14}):
        nose_extended = ctx.part_world_position(nose)
        ctx.expect_overlap(
            nose,
            pivot_frame,
            axes="x",
            elem_a="slide_bar",
            min_overlap=0.040,
            name="extended nose remains inserted in sleeve",
        )
    ctx.check(
        "terminal slider moves nose forward",
        nose_rest is not None
        and nose_extended is not None
        and nose_extended[0] > nose_rest[0] + 0.12,
        details=f"rest={nose_rest}, extended={nose_extended}",
    )

    nose_level = ctx.part_world_position(nose)
    with ctx.pose({hinge: 0.85}):
        nose_raised = ctx.part_world_position(nose)
    ctx.check(
        "positive hinge rotation lifts pivot frame",
        nose_level is not None
        and nose_raised is not None
        and nose_raised[2] > nose_level[2] + 0.15,
        details=f"level={nose_level}, raised={nose_raised}",
    )

    return ctx.report()


object_model = build_object_model()
