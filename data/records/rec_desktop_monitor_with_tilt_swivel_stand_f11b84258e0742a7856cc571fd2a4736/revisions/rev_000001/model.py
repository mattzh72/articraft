from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


OUTER_W = 0.540
OUTER_H = 0.320
BODY_D = 0.040
BODY_CENTER_Z = 0.050
SCREEN_W = 0.529
SCREEN_H = 0.299
SCREEN_CENTER_Z = 0.0555
BOTTOM_Z = BODY_CENTER_Z - OUTER_H / 2.0
BUTTON_Y = 0.026
BUTTON_XS = (0.138, 0.155, 0.172, 0.189)


def make_display_housing() -> cq.Workplane:
    housing = (
        cq.Workplane("XY")
        .box(OUTER_W, BODY_D, OUTER_H, centered=(True, False, True))
        .translate((0.0, 0.0, BODY_CENTER_Z))
        .edges("|Z")
        .fillet(0.010)
    )

    rear_bulge = (
        cq.Workplane("XY")
        .box(0.145, 0.020, 0.112, centered=(True, False, True))
        .translate((0.0, -0.020, 0.014))
        .edges("|Z")
        .fillet(0.008)
    )
    housing = housing.union(rear_bulge)

    screen_recess = (
        cq.Workplane("XY")
        .box(SCREEN_W + 0.010, 0.004, SCREEN_H + 0.010, centered=(True, False, True))
        .translate((0.0, BODY_D - 0.004, SCREEN_CENTER_Z))
    )
    housing = housing.cut(screen_recess)

    return housing


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_monitor")

    graphite = model.material("graphite", rgba=(0.17, 0.18, 0.19, 1.0))
    charcoal = model.material("charcoal", rgba=(0.10, 0.11, 0.12, 1.0))
    panel_black = model.material("panel_black", rgba=(0.04, 0.05, 0.06, 1.0))
    button_black = model.material("button_black", rgba=(0.07, 0.07, 0.08, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.240, 0.185, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=graphite,
        name="plate",
    )
    base.visual(
        Cylinder(radius=0.028, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=charcoal,
        name="collar",
    )

    neck = model.part("neck")
    neck.visual(
        Cylinder(radius=0.022, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=charcoal,
        name="swivel_collar",
    )
    neck.visual(
        Box((0.036, 0.020, 0.170)),
        origin=Origin(xyz=(0.0, -0.031, 0.091)),
        material=graphite,
        name="column",
    )
    neck.visual(
        Box((0.062, 0.012, 0.026)),
        origin=Origin(xyz=(0.0, -0.034, 0.189)),
        material=graphite,
        name="hinge_block",
    )
    neck.visual(
        Cylinder(radius=0.008, length=0.056),
        origin=Origin(xyz=(0.0, -0.028, 0.192), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="hinge_barrel",
    )

    display = model.part("display")
    display.visual(
        mesh_from_cadquery(make_display_housing(), "monitor_housing"),
        material=charcoal,
        name="housing",
    )
    display.visual(
        Box((SCREEN_W, 0.003, SCREEN_H)),
        origin=Origin(xyz=(0.0, BODY_D - 0.0035, SCREEN_CENTER_Z)),
        material=panel_black,
        name="panel_glass",
    )
    display.visual(
        Cylinder(radius=0.034, length=0.018),
        origin=Origin(xyz=(0.0, -0.014, 0.006), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="hub",
    )

    model.articulation(
        "swivel",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=neck,
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.5),
    )
    model.articulation(
        "tilt",
        ArticulationType.REVOLUTE,
        parent=neck,
        child=display,
        origin=Origin(xyz=(0.0, 0.0, 0.192)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.8,
            lower=-0.10,
            upper=0.35,
        ),
    )

    for index, x in enumerate(BUTTON_XS):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.009, 0.010, 0.005)),
            origin=Origin(xyz=(0.0, 0.0, -0.0025)),
            material=button_black,
            name="cap",
        )
        model.articulation(
            f"button_{index}_travel",
            ArticulationType.PRISMATIC,
            parent=display,
            child=button,
            origin=Origin(xyz=(x, BUTTON_Y, BOTTOM_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=0.050,
                lower=0.0,
                upper=0.003,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    display = object_model.get_part("display")
    button_0 = object_model.get_part("button_0")
    button_3 = object_model.get_part("button_3")

    swivel = object_model.get_articulation("swivel")
    tilt = object_model.get_articulation("tilt")
    button_0_travel = object_model.get_articulation("button_0_travel")

    ctx.allow_overlap(
        display,
        object_model.get_part("neck"),
        elem_a="hub",
        elem_b="hinge_barrel",
        reason="The tilt hinge is represented as a coaxial rear hub around the neck barrel.",
    )

    ctx.expect_gap(
        display,
        base,
        axis="z",
        positive_elem="housing",
        negative_elem="plate",
        min_gap=0.070,
        max_gap=0.120,
        name="display clears the pedestal base",
    )

    rest_button = ctx.part_world_position(button_3)
    with ctx.pose({swivel: math.pi / 2.0}):
        turned_button = ctx.part_world_position(button_3)
    ctx.check(
        "swivel rotates the control bank around the base axis",
        rest_button is not None
        and turned_button is not None
        and abs(turned_button[0] - rest_button[0]) > 0.12
        and abs(turned_button[1] - rest_button[1]) > 0.12,
        details=f"rest={rest_button}, turned={turned_button}",
    )

    rest_lower_bezel = ctx.part_world_position(button_0)
    with ctx.pose({tilt: 0.35}):
        tilted_lower_bezel = ctx.part_world_position(button_0)
    ctx.check(
        "tilt raises and pitches the lower bezel",
        rest_lower_bezel is not None
        and tilted_lower_bezel is not None
        and tilted_lower_bezel[2] > rest_lower_bezel[2] + 0.010
        and tilted_lower_bezel[1] > rest_lower_bezel[1] + 0.020,
        details=f"rest={rest_lower_bezel}, tilted={tilted_lower_bezel}",
    )

    rest_button_press = ctx.part_world_position(button_0)
    with ctx.pose({button_0_travel: 0.003}):
        pressed_button = ctx.part_world_position(button_0)
    ctx.check(
        "menu button depresses into the lower bezel",
        rest_button_press is not None
        and pressed_button is not None
        and pressed_button[2] > rest_button_press[2] + 0.0025,
        details=f"rest={rest_button_press}, pressed={pressed_button}",
    )

    return ctx.report()


object_model = build_object_model()
