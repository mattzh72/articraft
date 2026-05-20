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


BODY_X = 0.56
BODY_Y = 0.325
BODY_Z = 0.047
GLASS_X = 0.548
GLASS_Y = 0.313
GLASS_Z = 0.005
TRIM_WIDTH = 0.172
TRIM_HEIGHT = 0.018
TRIM_THICKNESS = 0.0035
TRIM_OVERLAP = 0.0002
BUTTON_TRAVEL = 0.0025
BUTTON_SLOT_X = 0.024
BUTTON_SLOT_Y = 0.020
BUTTON_SLOT_Z = 0.0092
BUTTON_CAP_X = BUTTON_SLOT_X
BUTTON_CAP_Y = 0.004
BUTTON_CAP_Z = BUTTON_SLOT_Z
BUTTON_STEM_X = 0.020
BUTTON_STEM_Y = 0.014
BUTTON_STEM_Z = 0.0074
BUTTON_CENTERS_X = (-0.042, 0.0, 0.042)
BUTTON_CENTER_Z = 0.016
DIAL_CENTER_Y = -0.083
DIAL_CENTER_Z = 0.021


def _build_housing_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(BODY_X, BODY_Y, BODY_Z, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.018)
        .edges(">Z")
        .chamfer(0.0025)
    )

    body = body.cut(
        cq.Workplane("XY")
        .pushPoints([(x, -BODY_Y / 2.0 + BUTTON_SLOT_Y / 2.0) for x in BUTTON_CENTERS_X])
        .box(BUTTON_SLOT_X, BUTTON_SLOT_Y, BUTTON_SLOT_Z, centered=(True, True, False))
        .translate((0.0, 0.0, BUTTON_CENTER_Z - BUTTON_SLOT_Z / 2.0))
    )

    body = body.cut(
        cq.Workplane("YZ")
        .center(DIAL_CENTER_Y, DIAL_CENTER_Z)
        .circle(0.0052)
        .extrude(-0.012)
        .translate((BODY_X / 2.0, 0.0, 0.0))
    )

    body = body.cut(
        cq.Workplane("YZ")
        .center(DIAL_CENTER_Y, DIAL_CENTER_Z)
        .circle(0.0105)
        .extrude(-0.0025)
        .translate((BODY_X / 2.0, 0.0, 0.0))
    )

    return body


def _build_glass_shape() -> cq.Workplane:
    zone_centers = [(-0.138, 0.026), (0.138, 0.026)]
    glass = (
        cq.Workplane("XY")
        .box(GLASS_X, GLASS_Y, GLASS_Z, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.012)
    )

    glass = (
        glass.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(zone_centers)
        .circle(0.086)
        .circle(0.078)
        .cutBlind(-0.00045)
    )
    glass = (
        glass.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(zone_centers)
        .circle(0.010)
        .cutBlind(-0.00025)
    )
    return glass


def _build_trim_shape() -> cq.Workplane:
    trim = cq.Workplane("XY").box(
        TRIM_WIDTH,
        TRIM_THICKNESS,
        TRIM_HEIGHT,
        centered=(True, True, False),
    )
    trim = trim.cut(
        cq.Workplane("XY")
        .pushPoints([(x, 0.0) for x in BUTTON_CENTERS_X])
        .box(BUTTON_SLOT_X, TRIM_THICKNESS * 1.4, BUTTON_SLOT_Z, centered=(True, True, False))
        .translate((0.0, 0.0, BUTTON_CENTER_Z - BUTTON_SLOT_Z / 2.0))
    )
    return trim


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="induction_cooktop")

    housing = model.material("housing", rgba=(0.17, 0.18, 0.19, 1.0))
    trim = model.material("trim", rgba=(0.12, 0.13, 0.14, 1.0))
    glass = model.material("glass", rgba=(0.06, 0.06, 0.07, 0.94))
    button = model.material("button", rgba=(0.67, 0.70, 0.72, 1.0))
    knob = model.material("knob", rgba=(0.13, 0.13, 0.14, 1.0))
    knob_mark = model.material("knob_mark", rgba=(0.85, 0.86, 0.87, 1.0))
    foot = model.material("foot", rgba=(0.08, 0.08, 0.09, 1.0))

    cooktop = model.part("cooktop")
    cooktop.visual(
        mesh_from_cadquery(_build_housing_shape(), "cooktop_housing"),
        material=housing,
        name="housing_shell",
    )
    cooktop.visual(
        mesh_from_cadquery(_build_glass_shape(), "cooktop_glass"),
        origin=Origin(xyz=(0.0, 0.0, BODY_Z - 0.0002)),
        material=glass,
        name="glass_plate",
    )
    cooktop.visual(
        mesh_from_cadquery(_build_trim_shape(), "cooktop_trim"),
        origin=Origin(
            xyz=(
                0.0,
                -BODY_Y / 2.0 - TRIM_THICKNESS / 2.0 + TRIM_OVERLAP,
                0.0,
            )
        ),
        material=trim,
        name="front_trim",
    )
    cooktop.visual(
        Box((0.160, 0.020, 0.006)),
        origin=Origin(xyz=(-0.130, 0.105, 0.003)),
        material=foot,
        name="rear_foot_0",
    )
    cooktop.visual(
        Box((0.160, 0.020, 0.006)),
        origin=Origin(xyz=(0.130, 0.105, 0.003)),
        material=foot,
        name="rear_foot_1",
    )
    cooktop.visual(
        Box((0.120, 0.018, 0.005)),
        origin=Origin(xyz=(-0.120, -0.100, 0.0025)),
        material=foot,
        name="front_foot_0",
    )
    cooktop.visual(
        Box((0.120, 0.018, 0.005)),
        origin=Origin(xyz=(0.120, -0.100, 0.0025)),
        material=foot,
        name="front_foot_1",
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.0044, length=0.010),
        origin=Origin(xyz=(-0.004, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob,
        name="shaft",
    )
    dial.visual(
        Cylinder(radius=0.018, length=0.022),
        origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob,
        name="knob_body",
    )
    dial.visual(
        Cylinder(radius=0.0115, length=0.004),
        origin=Origin(xyz=(0.001, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob,
        name="collar",
    )
    dial.visual(
        Box((0.0022, 0.0034, 0.011)),
        origin=Origin(xyz=(0.023, 0.0, 0.009)),
        material=knob_mark,
        name="indicator",
    )
    model.articulation(
        "cooktop_to_dial",
        ArticulationType.CONTINUOUS,
        parent=cooktop,
        child=dial,
        origin=Origin(xyz=(BODY_X / 2.0, DIAL_CENTER_Y, DIAL_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=6.0),
    )

    button_origin_y = -BODY_Y / 2.0 - TRIM_THICKNESS + TRIM_OVERLAP
    for index, button_x in enumerate(BUTTON_CENTERS_X):
        button_part = model.part(f"button_{index}")
        button_part.visual(
            Box((BUTTON_CAP_X, BUTTON_CAP_Y, BUTTON_CAP_Z)),
            origin=Origin(xyz=(0.0, 0.001, BUTTON_CENTER_Z)),
            material=button,
            name="cap",
        )
        button_part.visual(
            Box((BUTTON_STEM_X, BUTTON_STEM_Y, BUTTON_STEM_Z)),
            origin=Origin(
                xyz=(
                    0.0,
                    BUTTON_CAP_Y / 2.0 + BUTTON_STEM_Y / 2.0,
                    BUTTON_CENTER_Z,
                )
            ),
            material=button,
            name="stem",
        )
        model.articulation(
            f"cooktop_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=cooktop,
            child=button_part,
            origin=Origin(xyz=(button_x, button_origin_y, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.06,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cooktop = object_model.get_part("cooktop")
    dial = object_model.get_part("dial")
    dial_joint = object_model.get_articulation("cooktop_to_dial")
    button_parts = [object_model.get_part(f"button_{index}") for index in range(3)]
    button_joints = [
        object_model.get_articulation(f"cooktop_to_button_{index}") for index in range(3)
    ]

    ctx.check(
        "dial uses continuous rotation",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={dial_joint.articulation_type}",
    )
    ctx.expect_origin_gap(
        dial,
        cooktop,
        axis="x",
        min_gap=BODY_X / 2.0 - 0.005,
        name="dial is mounted on the right side",
    )

    ctx.expect_origin_distance(
        button_parts[0],
        button_parts[1],
        axes="x",
        min_dist=0.038,
        max_dist=0.046,
        name="left and center buttons are distinctly spaced",
    )
    ctx.expect_origin_distance(
        button_parts[1],
        button_parts[2],
        axes="x",
        min_dist=0.038,
        max_dist=0.046,
        name="center and right buttons are distinctly spaced",
    )
    ctx.expect_origin_gap(
        cooktop,
        button_parts[1],
        axis="y",
        min_gap=BODY_Y / 2.0 + 0.002,
        name="buttons sit on the front trim",
    )

    for index, (button_part, button_joint) in enumerate(zip(button_parts, button_joints)):
        rest_position = ctx.part_world_position(button_part)
        upper = button_joint.motion_limits.upper if button_joint.motion_limits else None
        with ctx.pose({button_joint: upper if upper is not None else BUTTON_TRAVEL}):
            pressed_position = ctx.part_world_position(button_part)

        moved_inward = (
            rest_position is not None
            and pressed_position is not None
            and pressed_position[1] > rest_position[1] + 0.0015
            and abs(pressed_position[0] - rest_position[0]) < 1e-6
            and abs(pressed_position[2] - rest_position[2]) < 1e-6
        )
        ctx.check(
            f"button_{index} presses inward independently",
            moved_inward,
            details=f"rest={rest_position}, pressed={pressed_position}",
        )

    return ctx.report()


object_model = build_object_model()
