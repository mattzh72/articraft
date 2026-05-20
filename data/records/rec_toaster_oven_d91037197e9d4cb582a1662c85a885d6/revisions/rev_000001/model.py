from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

BODY_WIDTH = 0.48
BODY_DEPTH = 0.38
BODY_HEIGHT = 0.30
BODY_FRONT_Y = BODY_DEPTH * 0.5

CAVITY_WIDTH = 0.338
CAVITY_DEPTH = 0.37
CAVITY_HEIGHT = 0.214
CAVITY_CENTER_X = -0.046
CAVITY_CENTER_Y = 0.020
CAVITY_CENTER_Z = 0.010

CONTROL_PANEL_WIDTH = 0.104
CONTROL_PANEL_THICKNESS = 0.0035
CONTROL_PANEL_HEIGHT = 0.228
CONTROL_PANEL_CENTER_X = 0.185
CONTROL_PANEL_CENTER_Z = 0.010

DOOR_WIDTH = 0.348
DOOR_THICKNESS = 0.022
DOOR_HEIGHT = 0.206
DOOR_HINGE_X = CAVITY_CENTER_X
DOOR_HINGE_Z = CAVITY_CENTER_Z - CAVITY_HEIGHT * 0.5

KNOB_DIAMETER = 0.039
KNOB_HEIGHT = 0.022
KNOB_FACE_Y = BODY_FRONT_Y + CONTROL_PANEL_THICKNESS
KNOB_CENTER_X = 0.183
KNOB_Z_POSITIONS = (0.074, 0.018, -0.038)


def _build_body_shell() -> object:
    outer = (
        cq.Workplane("XY")
        .box(BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)
        .edges("|Z")
        .fillet(0.010)
    )
    cavity = cq.Workplane("XY").box(CAVITY_WIDTH, CAVITY_DEPTH, CAVITY_HEIGHT).translate(
        (CAVITY_CENTER_X, CAVITY_CENTER_Y, CAVITY_CENTER_Z)
    )
    return outer.cut(cavity)


def _build_door_frame() -> object:
    panel = cq.Workplane("XY").box(DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT).translate(
        (0.0, DOOR_THICKNESS * 0.5, DOOR_HEIGHT * 0.5)
    )
    window_cut = cq.Workplane("XY").box(
        DOOR_WIDTH - 0.086,
        DOOR_THICKNESS + 0.012,
        DOOR_HEIGHT - 0.094,
    ).translate((0.0, DOOR_THICKNESS * 0.55, DOOR_HEIGHT * 0.50))
    frame = panel.cut(window_cut)

    handle_bar = cq.Workplane("XY").box(
        DOOR_WIDTH * 0.56,
        0.018,
        0.014,
    ).translate((0.0, DOOR_THICKNESS + 0.015, DOOR_HEIGHT * 0.74))

    standoff = cq.Workplane("XY").box(0.016, 0.020, 0.030)
    handle_posts = standoff.translate(
        (DOOR_WIDTH * 0.22, DOOR_THICKNESS + 0.006, DOOR_HEIGHT * 0.74)
    ).union(
        standoff.translate(
            (-DOOR_WIDTH * 0.22, DOOR_THICKNESS + 0.006, DOOR_HEIGHT * 0.74)
        )
    )

    return frame.union(handle_bar).union(handle_posts)


def _build_knob_mesh():
    return mesh_from_geometry(
        KnobGeometry(
            KNOB_DIAMETER,
            KNOB_HEIGHT,
            body_style="skirted",
            top_diameter=0.032,
            skirt=KnobSkirt(0.047, 0.004, flare=0.06),
            grip=KnobGrip(style="fluted", count=16, depth=0.0012),
            indicator=KnobIndicator(
                style="line",
                mode="engraved",
                depth=0.0007,
                angle_deg=0.0,
            ),
            center=False,
        ),
        "toaster_oven_control_knob",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_toaster_oven")

    body_finish = model.material("body_finish", rgba=(0.18, 0.19, 0.20, 1.0))
    trim_finish = model.material("trim_finish", rgba=(0.67, 0.69, 0.72, 1.0))
    glass_finish = model.material("glass_finish", rgba=(0.14, 0.20, 0.24, 0.35))
    foot_finish = model.material("foot_finish", rgba=(0.10, 0.10, 0.11, 1.0))
    knob_finish = model.material("knob_finish", rgba=(0.10, 0.11, 0.12, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shell(), "toaster_oven_body_shell"),
        material=body_finish,
        name="shell",
    )
    body.visual(
        Box((CONTROL_PANEL_WIDTH, CONTROL_PANEL_THICKNESS, CONTROL_PANEL_HEIGHT)),
        origin=Origin(
            xyz=(
                CONTROL_PANEL_CENTER_X,
                BODY_FRONT_Y + CONTROL_PANEL_THICKNESS * 0.5,
                CONTROL_PANEL_CENTER_Z,
            )
        ),
        material=trim_finish,
        name="control_panel",
    )

    for index, (x_pos, y_pos) in enumerate(
        (
            (-0.175, 0.140),
            (0.175, 0.140),
            (-0.175, -0.140),
            (0.175, -0.140),
        )
    ):
        body.visual(
            Box((0.040, 0.028, 0.010)),
            origin=Origin(xyz=(x_pos, y_pos, -BODY_HEIGHT * 0.5 - 0.005)),
            material=foot_finish,
            name=f"foot_{index}",
        )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(_build_door_frame(), "toaster_oven_door_frame"),
        material=trim_finish,
        name="door_frame",
    )
    door.visual(
        Box((DOOR_WIDTH - 0.068, 0.004, DOOR_HEIGHT - 0.090)),
        origin=Origin(
            xyz=(
                0.0,
                DOOR_THICKNESS * 0.28,
                DOOR_HEIGHT * 0.50,
            )
        ),
        material=glass_finish,
        name="window",
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(DOOR_HINGE_X, BODY_FRONT_Y, DOOR_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.8,
            lower=0.0,
            upper=1.55,
        ),
    )

    knob_mesh = _build_knob_mesh()
    for index, z_pos in enumerate(KNOB_Z_POSITIONS):
        knob = model.part(f"knob_{index}")
        knob.visual(
            Cylinder(radius=0.0055, length=0.010),
            origin=Origin(
                xyz=(0.0, 0.005, 0.0),
                rpy=(-math.pi * 0.5, 0.0, 0.0),
            ),
            material=trim_finish,
            name="shaft",
        )
        knob.visual(
            knob_mesh,
            origin=Origin(
                xyz=(0.0, 0.008, 0.0),
                rpy=(-math.pi * 0.5, 0.0, 0.0),
            ),
            material=knob_finish,
            name="knob_cap",
        )
        model.articulation(
            f"body_to_knob_{index}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=knob,
            origin=Origin(xyz=(KNOB_CENTER_X, KNOB_FACE_Y, z_pos)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.08, velocity=6.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    door_hinge = object_model.get_articulation("body_to_door")
    knob_parts = [object_model.get_part(f"knob_{index}") for index in range(3)]
    knob_joints = [object_model.get_articulation(f"body_to_knob_{index}") for index in range(3)]

    ctx.expect_gap(
        door,
        body,
        axis="y",
        positive_elem="door_frame",
        negative_elem="shell",
        min_gap=0.0,
        max_gap=0.002,
        name="door sits flush to the front shell when closed",
    )

    closed_door_aabb = ctx.part_element_world_aabb(door, elem="door_frame")
    with ctx.pose({door_hinge: door_hinge.motion_limits.upper}):
        open_door_aabb = ctx.part_element_world_aabb(door, elem="door_frame")

    ctx.check(
        "door opens downward and outward",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][2] < closed_door_aabb[1][2] - 0.15
        and open_door_aabb[1][1] > closed_door_aabb[1][1] + 0.12,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    for index, (knob, joint) in enumerate(zip(knob_parts, knob_joints)):
        ctx.expect_gap(
            knob,
            body,
            axis="y",
            positive_elem="shaft",
            negative_elem="control_panel",
            min_gap=0.0,
            max_gap=0.001,
            name=f"knob_{index} shaft seats on the control panel",
        )
        ctx.check(
            f"knob_{index} uses a continuous joint",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and joint.motion_limits is not None
            and joint.motion_limits.lower is None
            and joint.motion_limits.upper is None,
            details=f"type={joint.articulation_type}, limits={joint.motion_limits}",
        )

    knob_positions = [ctx.part_world_position(knob) for knob in knob_parts]
    ctx.check(
        "knobs form a vertical control stack",
        all(position is not None for position in knob_positions)
        and max(abs(knob_positions[index][0] - knob_positions[0][0]) for index in range(1, 3)) < 0.002
        and max(abs(knob_positions[index][1] - knob_positions[0][1]) for index in range(1, 3)) < 0.002
        and knob_positions[0][2] > knob_positions[1][2] > knob_positions[2][2]
        and knob_positions[0][2] - knob_positions[2][2] > 0.09,
        details=f"positions={knob_positions}",
    )

    return ctx.report()


object_model = build_object_model()
