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


UNIT_LENGTH = 0.86
UNIT_HEIGHT = 0.30
UNIT_DEPTH = 0.21
END_WALL = 0.012
HINGE_Y = 0.148
HINGE_Z = 0.088
BUTTON_Z = 0.198
BUTTON_Y = 0.205
BUTTON_X_POSITIONS = (0.309, 0.337)


def _build_body_shell() -> cq.Workplane:
    outer = (
        cq.Workplane("YZ")
        .moveTo(0.000, 0.050)
        .lineTo(0.000, 0.288)
        .threePointArc((0.055, 0.300), (0.132, 0.290))
        .threePointArc((0.205, 0.260), (0.209, 0.198))
        .threePointArc((0.205, 0.146), (0.160, 0.092))
        .threePointArc((0.110, 0.067), (0.048, 0.060))
        .close()
        .extrude(UNIT_LENGTH / 2.0, both=True)
    )

    inner = (
        cq.Workplane("YZ")
        .moveTo(-0.004, 0.064)
        .lineTo(-0.004, 0.276)
        .threePointArc((0.040, 0.286), (0.120, 0.278))
        .threePointArc((0.188, 0.248), (0.192, 0.198))
        .threePointArc((0.188, 0.152), (0.158, 0.114))
        .threePointArc((0.112, 0.084), (0.052, 0.072))
        .close()
        .extrude((UNIT_LENGTH / 2.0) - END_WALL, both=True)
    )

    shell = outer.cut(inner)

    outlet_opening = (
        cq.Workplane("XY")
        .box(UNIT_LENGTH - 0.060, 0.114, 0.054)
        .translate((0.000, 0.148, 0.078))
    )
    shell = shell.cut(outlet_opening)

    indicator_slot = (
        cq.Workplane("XY")
        .box(0.090, 0.020, 0.010)
        .translate((0.300, 0.191, 0.197))
    )
    shell = shell.cut(indicator_slot)

    for button_x in BUTTON_X_POSITIONS:
        button_aperture = (
            cq.Workplane("XY")
            .box(0.015, 0.026, 0.010)
            .translate((button_x, 0.196, BUTTON_Z))
        )
        shell = shell.cut(button_aperture)

    return shell


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mini_split_indoor_unit")

    shell_white = model.material("shell_white", rgba=(0.95, 0.96, 0.95, 1.0))
    trim_grey = model.material("trim_grey", rgba=(0.69, 0.71, 0.73, 1.0))
    outlet_charcoal = model.material("outlet_charcoal", rgba=(0.18, 0.19, 0.20, 1.0))
    button_grey = model.material("button_grey", rgba=(0.84, 0.85, 0.86, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shell(), "mini_split_shell"),
        material=shell_white,
        name="shell",
    )
    body.visual(
        Box((UNIT_LENGTH - 0.048, 0.010, 0.010)),
        origin=Origin(xyz=(0.000, 0.148, 0.089)),
        material=outlet_charcoal,
        name="outlet_lip",
    )
    body.visual(
        Box((0.080, 0.008, 0.028)),
        origin=Origin(xyz=(0.323, 0.198, 0.197)),
        material=trim_grey,
        name="control_panel",
    )

    flap = model.part("flap")
    flap.visual(
        Cylinder(radius=0.0045, length=UNIT_LENGTH - 0.054),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_grey,
        name="hinge_barrel",
    )
    flap.visual(
        Box((UNIT_LENGTH - 0.054, 0.056, 0.005)),
        origin=Origin(
            xyz=(0.000, 0.028, -0.012),
            rpy=(math.radians(-18.0), 0.0, 0.0),
        ),
        material=trim_grey,
        name="flap_panel",
    )

    model.articulation(
        "body_to_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flap,
        origin=Origin(xyz=(0.000, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(72.0),
        ),
    )

    for index, button_x in enumerate(BUTTON_X_POSITIONS):
        button = model.part(f"mode_button_{index}")
        button.visual(
            Box((0.013, 0.008, 0.007)),
            origin=Origin(xyz=(0.000, -0.002, 0.000)),
            material=button_grey,
            name="button_cap",
        )
        button.visual(
            Box((0.008, 0.010, 0.004)),
            origin=Origin(xyz=(0.000, -0.010, 0.000)),
            material=button_grey,
            name="button_stem",
        )

        model.articulation(
            f"body_to_mode_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(button_x, BUTTON_Y, BUTTON_Z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.05,
                lower=0.0,
                upper=0.0035,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    flap = object_model.get_part("flap")
    button_0 = object_model.get_part("mode_button_0")
    button_1 = object_model.get_part("mode_button_1")

    flap_joint = object_model.get_articulation("body_to_flap")
    button_0_joint = object_model.get_articulation("body_to_mode_button_0")
    button_1_joint = object_model.get_articulation("body_to_mode_button_1")

    ctx.expect_overlap(
        flap,
        body,
        axes="x",
        elem_a="flap_panel",
        elem_b="outlet_lip",
        min_overlap=0.760,
        name="flap spans nearly the full discharge width",
    )
    ctx.expect_overlap(
        button_0,
        body,
        axes="z",
        elem_a="button_cap",
        elem_b="control_panel",
        min_overlap=0.006,
        name="upper first mode button sits within the control panel band",
    )
    ctx.expect_overlap(
        button_1,
        body,
        axes="z",
        elem_a="button_cap",
        elem_b="control_panel",
        min_overlap=0.006,
        name="second mode button sits within the control panel band",
    )

    ctx.allow_overlap(
        body,
        flap,
        elem_a="outlet_lip",
        elem_b="hinge_barrel",
        reason="The flap's hidden hinge barrel nests inside the molded outlet lip.",
    )
    ctx.allow_overlap(
        body,
        flap,
        elem_a="shell",
        elem_b="hinge_barrel",
        reason="The shell mesh keeps the lower outlet lip continuous around the hidden hinge barrel.",
    )
    ctx.allow_overlap(
        body,
        flap,
        elem_a="shell",
        elem_b="flap_panel",
        reason="The closed discharge vane seats into the simplified lower outlet recess.",
    )

    flap_rest = ctx.part_element_world_aabb(flap, elem="flap_panel")
    button_0_rest = ctx.part_world_position(button_0)
    button_1_rest = ctx.part_world_position(button_1)

    with ctx.pose({flap_joint: math.radians(60.0)}):
        flap_open = ctx.part_element_world_aabb(flap, elem="flap_panel")

    ctx.check(
        "flap opens downward from the outlet lip",
        flap_rest is not None
        and flap_open is not None
        and flap_open[0][2] < flap_rest[0][2] - 0.015,
        details=f"rest={flap_rest}, open={flap_open}",
    )

    with ctx.pose({button_0_joint: 0.0035}):
        button_0_pressed = ctx.part_world_position(button_0)
        button_1_still = ctx.part_world_position(button_1)

    ctx.check(
        "first mode button depresses independently",
        button_0_rest is not None
        and button_1_rest is not None
        and button_0_pressed is not None
        and button_1_still is not None
        and button_0_pressed[1] < button_0_rest[1] - 0.002
        and abs(button_1_still[1] - button_1_rest[1]) <= 1e-6,
        details=(
            f"button_0_rest={button_0_rest}, button_0_pressed={button_0_pressed}, "
            f"button_1_rest={button_1_rest}, button_1_during_press={button_1_still}"
        ),
    )

    with ctx.pose({button_1_joint: 0.0035}):
        button_1_pressed = ctx.part_world_position(button_1)
        button_0_still = ctx.part_world_position(button_0)

    ctx.check(
        "second mode button depresses independently",
        button_0_rest is not None
        and button_1_rest is not None
        and button_1_pressed is not None
        and button_0_still is not None
        and button_1_pressed[1] < button_1_rest[1] - 0.002
        and abs(button_0_still[1] - button_0_rest[1]) <= 1e-6,
        details=(
            f"button_1_rest={button_1_rest}, button_1_pressed={button_1_pressed}, "
            f"button_0_rest={button_0_rest}, button_0_during_press={button_0_still}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
