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


BODY_DEPTH = 0.36
BODY_WIDTH = 0.46
BODY_HEIGHT = 0.30

OPENING_WIDTH = 0.308
OPENING_HEIGHT = 0.203
OPENING_CENTER_Y = -0.059
OPENING_BOTTOM_Z = 0.054

CONTROL_FACE_THICKNESS = 0.006
CONTROL_FACE_WIDTH = 0.122
CONTROL_FACE_HEIGHT = 0.246
CONTROL_CENTER_Y = 0.167
CONTROL_FRONT_X = BODY_DEPTH * 0.5 + CONTROL_FACE_THICKNESS * 0.5

DOOR_THICKNESS = 0.018
BUTTON_TRAVEL = 0.006


def _build_body_shell() -> cq.Workplane:
    shell = cq.Workplane("XY").box(BODY_DEPTH, BODY_WIDTH, BODY_HEIGHT).translate((0.0, 0.0, BODY_HEIGHT * 0.5))

    cavity_depth = 0.312
    cavity_width = 0.300
    cavity_height = 0.205
    cavity_center_x = BODY_DEPTH * 0.5 - cavity_depth * 0.5 + 0.010
    cavity_center_z = OPENING_BOTTOM_Z + cavity_height * 0.5
    cavity = (
        cq.Workplane("XY")
        .box(cavity_depth, cavity_width, cavity_height)
        .translate((cavity_center_x, OPENING_CENTER_Y, cavity_center_z))
    )
    return shell.cut(cavity)


def _build_door_frame() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(DOOR_THICKNESS, OPENING_WIDTH, OPENING_HEIGHT)
        .translate((DOOR_THICKNESS * 0.5, 0.0, OPENING_HEIGHT * 0.5))
    )
    window_cut = (
        cq.Workplane("XY")
        .box(DOOR_THICKNESS + 0.006, OPENING_WIDTH - 0.082, OPENING_HEIGHT - 0.092)
        .translate((DOOR_THICKNESS * 0.5, 0.0, OPENING_HEIGHT * 0.54))
    )
    return outer.cut(window_cut)


def _build_rack() -> cq.Workplane:
    rack_depth = 0.230
    rack_width = 0.270
    rail_thickness = 0.004

    rack = (
        cq.Workplane("XY")
        .box(rack_depth, rail_thickness, rail_thickness)
        .translate((0.0, -rack_width * 0.5, 0.0))
    )
    rack = rack.union(
        cq.Workplane("XY").box(rack_depth, rail_thickness, rail_thickness).translate((0.0, rack_width * 0.5, 0.0))
    )
    rack = rack.union(
        cq.Workplane("XY").box(rail_thickness, rack_width, rail_thickness).translate((-rack_depth * 0.5, 0.0, 0.0))
    )
    rack = rack.union(
        cq.Workplane("XY").box(rail_thickness, rack_width, rail_thickness).translate((rack_depth * 0.5, 0.0, 0.0))
    )

    for bar_index in range(5):
        y = -0.080 + bar_index * 0.040
        rack = rack.union(cq.Workplane("XY").box(rack_depth, 0.003, 0.003).translate((0.0, y, 0.0)))

    return rack


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="digital_toaster_oven")

    housing = model.material("housing", rgba=(0.72, 0.73, 0.75, 1.0))
    trim = model.material("trim", rgba=(0.10, 0.10, 0.11, 1.0))
    glass = model.material("glass", rgba=(0.24, 0.29, 0.32, 0.34))
    display_bezel = model.material("display_bezel", rgba=(0.14, 0.15, 0.17, 1.0))
    display_glow = model.material("display_glow", rgba=(0.16, 0.47, 0.55, 0.62))
    button_color = model.material("button", rgba=(0.84, 0.84, 0.82, 1.0))
    rack_finish = model.material("rack", rgba=(0.82, 0.82, 0.83, 1.0))

    body = model.part("body")
    body.visual(mesh_from_cadquery(_build_body_shell(), "toaster_oven_shell"), material=housing, name="shell")
    body.visual(
        Box((CONTROL_FACE_THICKNESS, CONTROL_FACE_WIDTH, CONTROL_FACE_HEIGHT)),
        origin=Origin(xyz=(CONTROL_FRONT_X, CONTROL_CENTER_Y, 0.165)),
        material=trim,
        name="control_face",
    )
    body.visual(
        Box((0.006, 0.078, 0.038)),
        origin=Origin(xyz=(BODY_DEPTH * 0.5 + 0.009, 0.146, 0.212)),
        material=display_bezel,
        name="display_bezel",
    )
    body.visual(
        Box((0.004, 0.064, 0.026)),
        origin=Origin(xyz=(BODY_DEPTH * 0.5 + 0.014, 0.146, 0.212)),
        material=display_glow,
        name="display",
    )
    body.visual(
        Box((0.012, BODY_WIDTH - 0.040, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=trim,
        name="base_skirt",
    )

    rack = model.part("rack")
    rack.visual(mesh_from_cadquery(_build_rack(), "toaster_oven_rack"), material=rack_finish, name="rack")
    model.articulation(
        "body_to_rack",
        ArticulationType.FIXED,
        parent=body,
        child=rack,
        origin=Origin(xyz=(-0.005, OPENING_CENTER_Y, 0.145)),
    )

    door = model.part("door")
    door.visual(mesh_from_cadquery(_build_door_frame(), "toaster_oven_door_frame"), material=trim, name="frame")
    door.visual(
        Box((0.006, OPENING_WIDTH - 0.036, OPENING_HEIGHT - 0.040)),
        origin=Origin(xyz=(0.008, 0.0, OPENING_HEIGHT * 0.54)),
        material=glass,
        name="glass",
    )
    door.visual(
        Box((0.018, 0.016, 0.020)),
        origin=Origin(xyz=(0.018, -0.124, 0.152)),
        material=trim,
        name="handle_post_0",
    )
    door.visual(
        Box((0.018, 0.016, 0.020)),
        origin=Origin(xyz=(0.018, 0.124, 0.152)),
        material=trim,
        name="handle_post_1",
    )
    door.visual(
        Cylinder(radius=0.010, length=OPENING_WIDTH - 0.056),
        origin=Origin(xyz=(0.031, 0.0, 0.152), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=trim,
        name="handle",
    )
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(BODY_DEPTH * 0.5, OPENING_CENTER_Y, OPENING_BOTTOM_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.5, lower=0.0, upper=1.55),
    )

    dial_mesh = mesh_from_geometry(
        KnobGeometry(
            0.044,
            0.026,
            body_style="skirted",
            top_diameter=0.034,
            base_diameter=0.050,
            edge_radius=0.002,
            side_draft_deg=7.0,
            center=False,
        ),
        "toaster_oven_dial",
    )
    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.005, length=0.010),
        origin=Origin(xyz=(0.003, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=trim,
        name="shaft",
    )
    dial.visual(
        dial_mesh,
        origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
        material=trim,
        name="knob",
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(BODY_DEPTH * 0.5 + 0.006, 0.176, 0.086)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=8.0),
    )

    button_positions = [
        (0.204, 0.221),
        (0.204, 0.188),
        (0.204, 0.155),
        (0.204, 0.122),
    ]
    for index, (button_y, button_z) in enumerate(button_positions):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.006, 0.022, 0.016)),
            origin=Origin(xyz=(-0.003, 0.0, 0.0)),
            material=trim,
            name="stem",
        )
        button.visual(
            Box((0.012, 0.018, 0.014)),
            origin=Origin(xyz=(0.006, 0.0, 0.0)),
            material=button_color,
            name="cap",
        )
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(BODY_DEPTH * 0.5 + 0.012, button_y, button_z)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=0.08, lower=0.0, upper=BUTTON_TRAVEL),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    door_joint = object_model.get_articulation("body_to_door")
    dial_joint = object_model.get_articulation("body_to_dial")

    with ctx.pose({door_joint: 0.0}):
        ctx.expect_gap(
            door,
            body,
            axis="x",
            positive_elem="frame",
            negative_elem="shell",
            max_gap=0.001,
            max_penetration=0.0,
            name="door sits just in front of the oven shell",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="yz",
            elem_a="frame",
            elem_b="shell",
            min_overlap=0.19,
            name="door covers the front opening footprint",
        )

    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_joint: 1.35}):
        open_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door swings down and outward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] < closed_aabb[1][2] - 0.12
        and open_aabb[1][0] > closed_aabb[1][0] + 0.09,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    ctx.check(
        "dial uses a continuous rotary joint on the front-facing shaft axis",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(dial_joint.axis) == (1.0, 0.0, 0.0)
        and dial_joint.motion_limits is not None
        and dial_joint.motion_limits.lower is None
        and dial_joint.motion_limits.upper is None,
        details=(
            f"type={dial_joint.articulation_type}, axis={dial_joint.axis}, "
            f"limits={dial_joint.motion_limits}"
        ),
    )

    for index in range(4):
        button = object_model.get_part(f"button_{index}")
        button_joint = object_model.get_articulation(f"body_to_button_{index}")
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({button_joint: BUTTON_TRAVEL}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"button_{index} depresses inward",
            rest_pos is not None and pressed_pos is not None and pressed_pos[0] < rest_pos[0] - 0.004,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )
        ctx.expect_overlap(
            button,
            body,
            axes="yz",
            elem_a="cap",
            elem_b="control_face",
            min_overlap=0.010,
            name=f"button_{index} stays within the control panel footprint",
        )

    return ctx.report()


object_model = build_object_model()
