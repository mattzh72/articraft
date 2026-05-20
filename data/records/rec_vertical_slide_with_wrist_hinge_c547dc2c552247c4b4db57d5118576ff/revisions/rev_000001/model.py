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
    model = ArticulatedObject(name="compact_service_lift")

    steel = model.material("dark_powder_coat", rgba=(0.10, 0.12, 0.13, 1.0))
    rail_paint = model.material("blue_lift_frame", rgba=(0.05, 0.17, 0.36, 1.0))
    deck_paint = model.material("orange_platen", rgba=(0.92, 0.38, 0.06, 1.0))
    rubber = model.material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    bright_steel = model.material("brushed_steel", rgba=(0.70, 0.72, 0.70, 1.0))
    warning = model.material("warning_yellow", rgba=(1.0, 0.78, 0.06, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.78, 0.54, 0.09)),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=steel,
        name="floor_base",
    )
    body.visual(
        Box((0.62, 0.045, 0.055)),
        origin=Origin(xyz=(0.0, 0.245, 0.117)),
        material=rail_paint,
        name="side_rail_0",
    )
    body.visual(
        Box((0.62, 0.045, 0.055)),
        origin=Origin(xyz=(0.0, -0.245, 0.117)),
        material=rail_paint,
        name="side_rail_1",
    )
    body.visual(
        Box((0.56, 0.05, 0.525)),
        origin=Origin(xyz=(0.0, 0.225, 0.3475)),
        material=rail_paint,
        name="guide_wall_0",
    )
    body.visual(
        Box((0.56, 0.05, 0.525)),
        origin=Origin(xyz=(0.0, -0.225, 0.3475)),
        material=rail_paint,
        name="guide_wall_1",
    )
    body.visual(
        Box((0.10, 0.50, 0.08)),
        origin=Origin(xyz=(-0.27, 0.0, 0.57)),
        material=rail_paint,
        name="rear_top_tie",
    )
    body.visual(
        Cylinder(radius=0.042, length=0.09),
        origin=Origin(xyz=(0.0, 0.0, 0.135)),
        material=bright_steel,
        name="lift_ram",
    )
    body.visual(
        Cylinder(radius=0.060, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.101)),
        material=steel,
        name="ram_collar",
    )

    platen = model.part("platen")
    platen.visual(
        Box((0.50, 0.34, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=deck_paint,
        name="deck_plate",
    )
    platen.visual(
        Box((0.42, 0.25, 0.012)),
        origin=Origin(xyz=(-0.02, 0.0, 0.061)),
        material=rubber,
        name="rubber_top",
    )
    platen.visual(
        Box((0.44, 0.035, 0.18)),
        origin=Origin(xyz=(0.0, 0.1825, 0.09)),
        material=steel,
        name="slide_shoe_0",
    )
    platen.visual(
        Box((0.44, 0.035, 0.18)),
        origin=Origin(xyz=(0.0, -0.1825, 0.09)),
        material=steel,
        name="slide_shoe_1",
    )
    platen.visual(
        Box((0.050, 0.052, 0.072)),
        origin=Origin(xyz=(0.265, 0.132, 0.091)),
        material=deck_paint,
        name="hinge_cheek_0",
    )
    platen.visual(
        Box((0.050, 0.052, 0.072)),
        origin=Origin(xyz=(0.265, -0.132, 0.091)),
        material=deck_paint,
        name="hinge_cheek_1",
    )
    platen.visual(
        Cylinder(radius=0.012, length=0.34),
        origin=Origin(xyz=(0.265, 0.0, 0.115), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bright_steel,
        name="hinge_pin",
    )

    faceplate = model.part("faceplate")
    faceplate.visual(
        Cylinder(radius=0.025, length=0.18),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bright_steel,
        name="hinge_barrel",
    )
    faceplate.visual(
        Box((0.035, 0.24, 0.22)),
        origin=Origin(xyz=(0.019, 0.0, 0.128)),
        material=deck_paint,
        name="tilt_panel",
    )
    faceplate.visual(
        Box((0.006, 0.205, 0.024)),
        origin=Origin(xyz=(0.039, 0.0, 0.085)),
        material=warning,
        name="warning_stripe_0",
    )
    faceplate.visual(
        Box((0.006, 0.205, 0.024)),
        origin=Origin(xyz=(0.039, 0.0, 0.155)),
        material=warning,
        name="warning_stripe_1",
    )

    model.articulation(
        "body_to_platen",
        ArticulationType.PRISMATIC,
        parent=body,
        child=platen,
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.16, lower=0.0, upper=0.24),
    )

    model.articulation(
        "platen_to_faceplate",
        ArticulationType.REVOLUTE,
        parent=platen,
        child=faceplate,
        origin=Origin(xyz=(0.265, 0.0, 0.115)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=1.2, lower=-0.15, upper=0.95),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    platen = object_model.get_part("platen")
    faceplate = object_model.get_part("faceplate")
    lift = object_model.get_articulation("body_to_platen")
    tilt = object_model.get_articulation("platen_to_faceplate")

    ctx.allow_overlap(
        platen,
        faceplate,
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        reason="The faceplate hinge barrel is intentionally captured around the platen hinge pin.",
    )

    ctx.expect_contact(
        body,
        platen,
        elem_a="lift_ram",
        elem_b="deck_plate",
        contact_tol=0.002,
        name="ram supports platen at the lowered stop",
    )
    ctx.expect_gap(
        platen,
        body,
        axis="z",
        positive_elem="deck_plate",
        negative_elem="lift_ram",
        max_gap=0.002,
        max_penetration=0.0,
        name="platen underside sits on ram without sinking",
    )
    ctx.expect_contact(
        body,
        platen,
        elem_a="guide_wall_0",
        elem_b="slide_shoe_0",
        contact_tol=0.001,
        name="first slide shoe bears on guide wall",
    )
    ctx.expect_contact(
        body,
        platen,
        elem_a="guide_wall_1",
        elem_b="slide_shoe_1",
        contact_tol=0.001,
        name="second slide shoe bears on guide wall",
    )
    ctx.expect_within(
        platen,
        faceplate,
        axes="xz",
        inner_elem="hinge_pin",
        outer_elem="hinge_barrel",
        margin=0.014,
        name="hinge pin is coaxial inside faceplate barrel",
    )
    ctx.expect_overlap(
        platen,
        faceplate,
        axes="y",
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        min_overlap=0.17,
        name="hinge pin spans the faceplate barrel",
    )

    lowered = ctx.part_world_position(platen)
    with ctx.pose({lift: 0.20}):
        raised = ctx.part_world_position(platen)
        ctx.expect_overlap(
            body,
            platen,
            axes="z",
            elem_a="guide_wall_0",
            elem_b="slide_shoe_0",
            min_overlap=0.11,
            name="raised platen remains in the guide track",
        )

    untilted = ctx.part_world_aabb(faceplate)
    with ctx.pose({tilt: 0.70}):
        tilted = ctx.part_world_aabb(faceplate)

    ctx.check(
        "platen travels upward on a vertical prismatic joint",
        lowered is not None and raised is not None and raised[2] > lowered[2] + 0.18,
        details=f"lowered={lowered}, raised={raised}",
    )
    ctx.check(
        "faceplate tilts forward about the platen hinge",
        untilted is not None
        and tilted is not None
        and tilted[1][0] > untilted[1][0] + 0.07,
        details=f"untilted={untilted}, tilted={tilted}",
    )

    return ctx.report()


object_model = build_object_model()
