from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_WIDTH = 0.122
BODY_HEIGHT = 0.122
BODY_DEPTH = 0.026
SCREEN_WIDTH = 0.076
SCREEN_HEIGHT = 0.038
SCREEN_RECESS = 0.0026
BUTTON_CAP_WIDTH = 0.019
BUTTON_CAP_HEIGHT = 0.010
BUTTON_CAP_DEPTH = 0.0024
BUTTON_STEM_WIDTH = 0.021
BUTTON_STEM_HEIGHT = 0.012
BUTTON_STEM_DEPTH = 0.0070
BUTTON_TRAVEL = 0.0015
BUTTON_POCKET_WIDTH = 0.021
BUTTON_POCKET_HEIGHT = 0.012
BUTTON_POCKET_DEPTH = 0.0095
BUTTON_ROW_Z = -0.029
BUTTON_XS = (-0.033, -0.011, 0.011, 0.033)
DIAL_DIAMETER = 0.034
DIAL_HEIGHT = 0.010
DIAL_SHAFT_LENGTH = 0.004


def _build_body_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)
        .translate((0.0, BODY_DEPTH * 0.5, 0.0))
        .edges("|Z")
        .fillet(0.010)
    )
    body = body.faces(">Y").edges().fillet(0.004)

    screen_cutter = (
        cq.Workplane("XY")
        .box(SCREEN_WIDTH, SCREEN_RECESS + 0.0005, SCREEN_HEIGHT)
        .translate((0.0, BODY_DEPTH - SCREEN_RECESS * 0.5 + 0.00025, 0.016))
    )
    body = body.cut(screen_cutter)

    for x in BUTTON_XS:
        button_cutter = (
            cq.Workplane("XY")
            .box(BUTTON_POCKET_WIDTH, BUTTON_POCKET_DEPTH + 0.0005, BUTTON_POCKET_HEIGHT)
            .translate(
                (
                    x,
                    BODY_DEPTH - BUTTON_POCKET_DEPTH * 0.5 + 0.00025,
                    BUTTON_ROW_Z,
                )
            )
        )
        body = body.cut(button_cutter)

    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="digital_thermostat")

    shell = model.material("shell", rgba=(0.92, 0.93, 0.94, 1.0))
    glass = model.material("glass", rgba=(0.08, 0.11, 0.13, 0.70))
    button = model.material("button", rgba=(0.84, 0.85, 0.87, 1.0))
    button_stem = model.material("button_stem", rgba=(0.20, 0.21, 0.23, 1.0))
    dial = model.material("dial", rgba=(0.23, 0.24, 0.26, 1.0))
    accent = model.material("accent", rgba=(0.78, 0.80, 0.82, 1.0))
    shaft = model.material("shaft", rgba=(0.54, 0.55, 0.58, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shape(), "thermostat_body"),
        material=shell,
        name="shell",
    )

    display = model.part("display")
    display.visual(
        Box((SCREEN_WIDTH - 0.006, 0.0016, SCREEN_HEIGHT - 0.006)),
        material=glass,
        name="glass",
    )
    model.articulation(
        "body_to_display",
        ArticulationType.FIXED,
        parent=body,
        child=display,
        origin=Origin(
            xyz=(
                0.0,
                BODY_DEPTH - SCREEN_RECESS + 0.0008,
                0.016,
            )
        ),
    )

    dial_part = model.part("dial")
    dial_part.visual(
        Cylinder(radius=0.0038, length=DIAL_SHAFT_LENGTH),
        origin=Origin(xyz=(DIAL_SHAFT_LENGTH * 0.5, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=shaft,
        name="shaft",
    )
    dial_part.visual(
        mesh_from_geometry(
            KnobGeometry(
                DIAL_DIAMETER,
                DIAL_HEIGHT,
                body_style="skirted",
                top_diameter=0.030,
                base_diameter=0.036,
                edge_radius=0.0012,
                center=False,
            ),
            "thermostat_dial",
        ),
        origin=Origin(xyz=(DIAL_SHAFT_LENGTH, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=dial,
        name="knob",
    )
    dial_part.visual(
        Box((0.0008, 0.0014, 0.009)),
        origin=Origin(
            xyz=(DIAL_SHAFT_LENGTH + DIAL_HEIGHT - 0.0004, 0.0, DIAL_DIAMETER * 0.28)
        ),
        material=accent,
        name="pointer",
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial_part,
        origin=Origin(xyz=(BODY_WIDTH * 0.5, BODY_DEPTH * 0.5, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )

    for index, x in enumerate(BUTTON_XS):
        button_part = model.part(f"button_{index}")
        button_part.visual(
            Box((BUTTON_CAP_WIDTH, BUTTON_CAP_DEPTH, BUTTON_CAP_HEIGHT)),
            origin=Origin(xyz=(0.0, BUTTON_CAP_DEPTH * 0.5, 0.0)),
            material=button,
            name="cap",
        )
        button_part.visual(
            Box((BUTTON_STEM_WIDTH, BUTTON_STEM_DEPTH, BUTTON_STEM_HEIGHT)),
            origin=Origin(xyz=(0.0, -BUTTON_STEM_DEPTH * 0.5, 0.0)),
            material=button_stem,
            name="stem",
        )
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button_part,
            origin=Origin(xyz=(x, BODY_DEPTH, BUTTON_ROW_Z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.05,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    display = object_model.get_part("display")
    dial = object_model.get_part("dial")
    dial_joint = object_model.get_articulation("body_to_dial")
    button_parts = [object_model.get_part(f"button_{index}") for index in range(4)]
    button_joints = [object_model.get_articulation(f"body_to_button_{index}") for index in range(4)]

    for index, button_part in enumerate(button_parts):
        ctx.allow_overlap(
            body,
            button_part,
            elem_a="shell",
            elem_b="stem",
            reason=(
                "The thermostat body is simplified as a solid front shell while each button stem "
                "stands in for a guided plunger running inside a blind internal pocket."
            ),
        )

    ctx.expect_within(
        display,
        body,
        axes="xz",
        margin=0.002,
        name="display stays within the thermostat face",
    )
    ctx.expect_contact(
        dial,
        body,
        elem_a="shaft",
        name="side dial shaft stays mounted to the body",
    )

    dial_limits = dial_joint.motion_limits
    ctx.check(
        "dial is continuous with unbounded travel",
        dial_joint.joint_type == ArticulationType.CONTINUOUS
        and dial_limits is not None
        and dial_limits.lower is None
        and dial_limits.upper is None,
        details=f"type={dial_joint.joint_type}, limits={dial_limits}",
    )

    for index, button_part in enumerate(button_parts):
        ctx.expect_within(
            button_part,
            body,
            axes="xz",
            margin=0.0015,
            name=f"button {index} stays aligned within the front face",
        )

    for index, (button_part, button_joint) in enumerate(zip(button_parts, button_joints)):
        rest_position = ctx.part_world_position(button_part)
        other_button = button_parts[(index + 1) % len(button_parts)]
        other_rest = ctx.part_world_position(other_button)
        with ctx.pose({button_joint: BUTTON_TRAVEL}):
            pressed_position = ctx.part_world_position(button_part)
            other_pressed = ctx.part_world_position(other_button)

        ctx.check(
            f"button {index} depresses inward",
            rest_position is not None
            and pressed_position is not None
            and pressed_position[1] < rest_position[1] - 0.001,
            details=f"rest={rest_position}, pressed={pressed_position}",
        )
        ctx.check(
            f"button {index} moves independently",
            other_rest is not None
            and other_pressed is not None
            and abs(other_pressed[1] - other_rest[1]) < 1e-6,
            details=f"other_rest={other_rest}, other_pressed={other_pressed}",
        )

    return ctx.report()


object_model = build_object_model()
