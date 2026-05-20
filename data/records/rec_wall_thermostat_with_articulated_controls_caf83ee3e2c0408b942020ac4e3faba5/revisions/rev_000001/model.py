from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_WIDTH = 0.118
BODY_HEIGHT = 0.086
BODY_DEPTH = 0.027
BODY_CORNER_RADIUS = 0.010

DIAL_Y = 0.008
DIAL_OUTER_RADIUS = 0.030
DIAL_INNER_RADIUS = 0.022
DIAL_GUIDE_OUTER_RADIUS = 0.0282
DIAL_GUIDE_INNER_RADIUS = 0.0240
DIAL_FRONT_THICKNESS = 0.0048
DIAL_GUIDE_DEPTH = 0.0016

BUTTON_ROW_Y = -0.028
BUTTON_XS = (-0.020, 0.000, 0.020)
BUTTON_CAP_SIZE = (0.016, 0.008, 0.0024)
BUTTON_STEM_SIZE = (0.0118, 0.0048, 0.0047)
BUTTON_TRAVEL = 0.0018
BUTTON_SLOT_SIZE = (0.0128, 0.0060)
BUTTON_SLOT_DEPTH = 0.0068


def _build_body_shape() -> cq.Workplane:
    body = cq.Workplane("XY").rect(BODY_WIDTH, BODY_HEIGHT).extrude(BODY_DEPTH)
    body = body.edges("|Z").fillet(BODY_CORNER_RADIUS)
    body = body.faces(">Z").edges().fillet(0.0025)
    body = body.faces("<Z").edges().fillet(0.0012)

    body = (
        body.faces(">Z")
        .workplane()
        .center(0.0, DIAL_Y)
        .circle(DIAL_OUTER_RADIUS + 0.0010)
        .circle(DIAL_GUIDE_INNER_RADIUS - 0.0010)
        .cutBlind(-0.0019)
    )

    for button_x in BUTTON_XS:
        body = (
            body.faces(">Z")
            .workplane()
            .center(button_x, BUTTON_ROW_Y)
            .rect(BUTTON_SLOT_SIZE[0], BUTTON_SLOT_SIZE[1])
            .cutBlind(-BUTTON_SLOT_DEPTH)
        )

    return body


def _build_dial_ring_shape() -> cq.Workplane:
    front_ring = (
        cq.Workplane("XY")
        .circle(DIAL_OUTER_RADIUS)
        .circle(DIAL_INNER_RADIUS)
        .extrude(DIAL_FRONT_THICKNESS)
    )
    guide_ring = (
        cq.Workplane("XY")
        .circle(DIAL_GUIDE_OUTER_RADIUS)
        .circle(DIAL_GUIDE_INNER_RADIUS)
        .extrude(DIAL_GUIDE_DEPTH)
        .translate((0.0, 0.0, -DIAL_GUIDE_DEPTH))
    )
    return front_ring.union(guide_ring)


def _build_button_shape() -> cq.Workplane:
    cap = cq.Workplane("XY").rect(BUTTON_CAP_SIZE[0], BUTTON_CAP_SIZE[1]).extrude(BUTTON_CAP_SIZE[2])
    cap = cap.faces(">Z").edges().fillet(0.0010)
    stem = (
        cq.Workplane("XY")
        .rect(BUTTON_STEM_SIZE[0], BUTTON_STEM_SIZE[1])
        .extrude(BUTTON_STEM_SIZE[2])
        .translate((0.0, 0.0, -BUTTON_STEM_SIZE[2]))
    )
    return cap.union(stem)


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((float(mins[i]) + float(maxs[i])) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_thermostat")

    body_finish = model.material("body_finish", rgba=(0.94, 0.95, 0.96, 1.0))
    dial_finish = model.material("dial_finish", rgba=(0.79, 0.81, 0.84, 1.0))
    accent_dark = model.material("accent_dark", rgba=(0.22, 0.24, 0.27, 1.0))
    button_finish = model.material("button_finish", rgba=(0.83, 0.85, 0.88, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shape(), "thermostat_body"),
        material=body_finish,
        name="housing",
    )
    body.visual(
        Box((0.033, 0.033, 0.0012)),
        origin=Origin(xyz=(0.0, DIAL_Y, BODY_DEPTH - 0.0006)),
        material=accent_dark,
        name="hub",
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_cadquery(_build_dial_ring_shape(), "thermostat_dial"),
        material=dial_finish,
        name="ring",
    )
    dial.visual(
        Box((0.004, 0.006, 0.0012)),
        origin=Origin(xyz=(0.0, DIAL_OUTER_RADIUS - 0.0035, DIAL_FRONT_THICKNESS - 0.0003)),
        material=accent_dark,
        name="marker",
    )

    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, DIAL_Y, BODY_DEPTH)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.15, velocity=6.0),
    )

    button_mesh = mesh_from_cadquery(_build_button_shape(), "thermostat_button")
    for index, button_x in enumerate(BUTTON_XS):
        button = model.part(f"button_{index}")
        button.visual(
            button_mesh,
            material=button_finish,
            name="button_shell",
        )
        button.visual(
            Box((BUTTON_CAP_SIZE[0] - 0.003, 0.0014, 0.0007)),
            origin=Origin(
                xyz=(
                    0.0,
                    BUTTON_CAP_SIZE[1] * 0.5 - 0.0015,
                    BUTTON_CAP_SIZE[2] - 0.00025,
                )
            ),
            material=accent_dark,
            name="button_mark",
        )

        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(button_x, BUTTON_ROW_Y, BODY_DEPTH)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.04,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    dial = object_model.get_part("dial")
    dial_joint = object_model.get_articulation("body_to_dial")
    buttons = [object_model.get_part(f"button_{index}") for index in range(3)]
    button_joints = [object_model.get_articulation(f"body_to_button_{index}") for index in range(3)]

    body_aabb = ctx.part_world_aabb(body)
    ctx.check("body_aabb_present", body_aabb is not None, "Thermostat body should produce a world AABB.")
    if body_aabb is not None:
        mins, maxs = body_aabb
        size = tuple(float(maxs[i] - mins[i]) for i in range(3))
        ctx.check("body_width_scale", 0.112 <= size[0] <= 0.124, f"size={size!r}")
        ctx.check("body_height_scale", 0.080 <= size[1] <= 0.092, f"size={size!r}")
        ctx.check("body_depth_scale", 0.024 <= size[2] <= 0.032, f"size={size!r}")

    ctx.expect_overlap(
        dial,
        body,
        axes="xy",
        min_overlap=0.040,
        elem_a="ring",
        name="dial stays centered on the thermostat face",
    )

    rest_button_positions = [ctx.part_world_position(button) for button in buttons]
    with ctx.pose({button_joints[1]: BUTTON_TRAVEL}):
        pressed_positions = [ctx.part_world_position(button) for button in buttons]

    middle_rest = rest_button_positions[1]
    middle_pressed = pressed_positions[1]
    ctx.check(
        "middle button depresses inward",
        middle_rest is not None
        and middle_pressed is not None
        and middle_pressed[2] < middle_rest[2] - 0.0015,
        details=f"rest={middle_rest!r}, pressed={middle_pressed!r}",
    )
    for index in (0, 2):
        side_rest = rest_button_positions[index]
        side_pressed = pressed_positions[index]
        ctx.check(
            f"button_{index} stays independent",
            side_rest is not None
            and side_pressed is not None
            and abs(side_pressed[2] - side_rest[2]) <= 1e-6,
            details=f"rest={side_rest!r}, posed={side_pressed!r}",
        )

    marker_rest = _aabb_center(ctx.part_element_world_aabb(dial, elem="marker"))
    with ctx.pose({dial_joint: math.pi * 0.5}):
        marker_quarter = _aabb_center(ctx.part_element_world_aabb(dial, elem="marker"))
    ctx.check(
        "dial ring rotates about center",
        marker_rest is not None
        and marker_quarter is not None
        and abs(marker_quarter[0] - marker_rest[0]) > 0.015
        and abs(marker_quarter[1] - marker_rest[1]) > 0.015,
        details=f"rest={marker_rest!r}, quarter_turn={marker_quarter!r}",
    )

    return ctx.report()


object_model = build_object_model()
