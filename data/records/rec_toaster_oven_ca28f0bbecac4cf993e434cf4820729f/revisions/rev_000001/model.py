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


BODY_W = 0.58
BODY_D = 0.40
BODY_H = 0.34
SHELL_T = 0.016
FRONT_LIP = 0.028
CORNER_R = 0.024
TOP_R = 0.018

OPENING_W = 0.406
OPENING_H = 0.228
OPENING_X = -0.050
OPENING_BOTTOM = 0.060

DOOR_T = 0.024
DOOR_CENTER_GAP = 0.004
DOOR_W = OPENING_W / 2.0 - DOOR_CENTER_GAP / 2.0
DOOR_H = OPENING_H + 0.006
DOOR_RECESS_D = 0.004
DOOR_FRAME_MARGIN = 0.028
DOOR_GLASS_W = DOOR_W - 2.0 * DOOR_FRAME_MARGIN
DOOR_GLASS_H = DOOR_H - 0.102
DOOR_GLASS_Z = 0.018
DOOR_MOUNT_GAP = 0.0

HANDLE_W = 0.014
HANDLE_D = 0.016
HANDLE_H = 0.112
HANDLE_EDGE = 0.030
HANDLE_POST_W = 0.010
HANDLE_POST_D = 0.014
HANDLE_POST_H = 0.018
HANDLE_STANDOFF = 0.012

CONTROL_PANEL_W = 0.116
CONTROL_PANEL_T = 0.014
CONTROL_PANEL_H = 0.270
CONTROL_X = 0.220
KNOB_DIAMETER = 0.056
KNOB_DEPTH = 0.029
KNOB_ZS = (0.252, 0.186, 0.120)

FOOT_R = 0.018
FOOT_H = 0.016


def _body_shell_shape():
    outer = cq.Workplane("XY").box(BODY_W, BODY_D, BODY_H, centered=(True, True, False))
    outer = outer.edges("|Z").fillet(CORNER_R)
    outer = outer.edges(">Z").fillet(TOP_R)

    cavity = (
        cq.Workplane("XY")
        .box(
            BODY_W - 2.0 * SHELL_T,
            BODY_D - SHELL_T - FRONT_LIP,
            BODY_H - 2.0 * SHELL_T,
            centered=(True, True, False),
        )
        .translate((0.0, (SHELL_T - FRONT_LIP) / 2.0, SHELL_T))
    )
    opening = (
        cq.Workplane("XY")
        .box(OPENING_W, FRONT_LIP + 0.020, OPENING_H, centered=(True, False, False))
        .translate((OPENING_X, BODY_D / 2.0 - FRONT_LIP - 0.010, OPENING_BOTTOM))
    )
    return outer.cut(cavity).cut(opening)


def _door_frame_shape(sign: float):
    frame = cq.Workplane("XY").box(DOOR_W, DOOR_T, DOOR_H)
    frame = frame.translate((sign * (DOOR_W / 2.0), 0.0, 0.0))
    pocket = (
        cq.Workplane("XY")
        .box(DOOR_GLASS_W, DOOR_RECESS_D, DOOR_GLASS_H)
        .translate(
            (
                sign * (DOOR_FRAME_MARGIN + DOOR_GLASS_W / 2.0),
                DOOR_T / 2.0 - DOOR_RECESS_D / 2.0,
                DOOR_GLASS_Z,
            )
        )
    )
    return frame.cut(pocket)


def _door_handle_shape(sign: float):
    handle_x = sign * (DOOR_W - HANDLE_EDGE - HANDLE_W / 2.0)
    grip = (
        cq.Workplane("XY")
        .box(HANDLE_W, HANDLE_D, HANDLE_H)
        .edges("|Z")
        .fillet(0.004)
        .translate((handle_x, DOOR_T / 2.0 + HANDLE_STANDOFF + HANDLE_D / 2.0, 0.0))
    )
    upper_post = cq.Workplane("XY").box(HANDLE_POST_W, HANDLE_POST_D, HANDLE_POST_H).translate(
        (
            handle_x,
            DOOR_T / 2.0 + HANDLE_POST_D / 2.0,
            HANDLE_H * 0.30,
        )
    )
    lower_post = cq.Workplane("XY").box(HANDLE_POST_W, HANDLE_POST_D, HANDLE_POST_H).translate(
        (
            handle_x,
            DOOR_T / 2.0 + HANDLE_POST_D / 2.0,
            -HANDLE_H * 0.30,
        )
    )
    return grip.union(upper_post).union(lower_post)


def _door_glass_origin(sign: float) -> Origin:
    return Origin(
        xyz=(
            sign * (DOOR_FRAME_MARGIN + DOOR_GLASS_W / 2.0),
            DOOR_T / 2.0 - DOOR_RECESS_D / 2.0,
            DOOR_GLASS_Z,
        )
    )


def _aabb_axis_value(aabb, which: str, axis: int):
    if aabb is None:
        return None
    return aabb[0][axis] if which == "min" else aabb[1][axis]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retro_toaster_oven")

    enamel = model.material("enamel", rgba=(0.87, 0.23, 0.16, 1.0))
    cream = model.material("cream", rgba=(0.95, 0.89, 0.75, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.18, 0.16, 0.15, 1.0))
    chrome = model.material("chrome", rgba=(0.81, 0.83, 0.86, 1.0))
    glass = model.material("glass", rgba=(0.12, 0.14, 0.16, 0.68))
    rubber = model.material("rubber", rgba=(0.11, 0.11, 0.12, 1.0))

    body_shell = mesh_from_cadquery(_body_shell_shape(), "toaster_body_shell")
    left_door_frame = mesh_from_cadquery(_door_frame_shape(1.0), "door_frame_left")
    right_door_frame = mesh_from_cadquery(_door_frame_shape(-1.0), "door_frame_right")
    left_handle = mesh_from_cadquery(_door_handle_shape(1.0), "door_handle_left")
    right_handle = mesh_from_cadquery(_door_handle_shape(-1.0), "door_handle_right")
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            KNOB_DIAMETER,
            KNOB_DEPTH,
            body_style="skirted",
            top_diameter=0.042,
            skirt=KnobSkirt(0.062, 0.006, flare=0.06),
            grip=KnobGrip(style="fluted", count=18, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
            center=False,
        ),
        "control_knob",
    )

    body = model.part("body")
    body.visual(body_shell, material=enamel, name="housing")
    body.visual(
        Box((CONTROL_PANEL_W, CONTROL_PANEL_T, CONTROL_PANEL_H)),
        origin=Origin(
            xyz=(
                CONTROL_X,
                BODY_D / 2.0 + CONTROL_PANEL_T / 2.0,
                OPENING_BOTTOM + OPENING_H / 2.0 + 0.010,
            )
        ),
        material=cream,
        name="control_panel",
    )
    body.visual(
        Box((0.066, CONTROL_PANEL_T * 1.4, 0.040)),
        origin=Origin(xyz=(CONTROL_X, BODY_D / 2.0 + CONTROL_PANEL_T * 0.7, 0.308)),
        material=dark_trim,
        name="badge_panel",
    )
    for index, knob_z in enumerate(KNOB_ZS):
        body.visual(
            Cylinder(radius=0.032, length=0.004),
            origin=Origin(
                xyz=(CONTROL_X, BODY_D / 2.0 + 0.002, knob_z),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=chrome,
            name=f"bezel_{index}",
        )
    for foot_x in (-0.205, 0.205):
        for foot_y in (-0.132, 0.132):
            body.visual(
                Cylinder(radius=FOOT_R, length=FOOT_H),
                origin=Origin(xyz=(foot_x, foot_y, FOOT_H / 2.0)),
                material=rubber,
                name=f"foot_{'rear' if foot_y < 0.0 else 'front'}_{'in' if foot_x < 0.0 else 'out'}",
            )

    left_door = model.part("door_0")
    left_door.visual(left_door_frame, material=chrome, name="frame")
    left_door.visual(
        Box((DOOR_GLASS_W, DOOR_RECESS_D, DOOR_GLASS_H)),
        origin=_door_glass_origin(1.0),
        material=glass,
        name="glass",
    )
    left_door.visual(left_handle, material=chrome, name="handle")

    right_door = model.part("door_1")
    right_door.visual(right_door_frame, material=chrome, name="frame")
    right_door.visual(
        Box((DOOR_GLASS_W, DOOR_RECESS_D, DOOR_GLASS_H)),
        origin=_door_glass_origin(-1.0),
        material=glass,
        name="glass",
    )
    right_door.visual(right_handle, material=chrome, name="handle")

    knob_parts = []
    for index, knob_z in enumerate(KNOB_ZS):
        knob = model.part(f"knob_{index}")
        knob.visual(
            knob_mesh,
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=chrome,
            name="knob",
        )
        knob_parts.append((knob, knob_z))

    left_hinge_x = OPENING_X - OPENING_W / 2.0
    right_hinge_x = OPENING_X + OPENING_W / 2.0
    door_hinge_z = OPENING_BOTTOM + OPENING_H / 2.0

    model.articulation(
        "body_to_door_0",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_door,
        origin=Origin(xyz=(left_hinge_x, BODY_D / 2.0 + DOOR_T / 2.0 + DOOR_MOUNT_GAP, door_hinge_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.45),
    )
    model.articulation(
        "body_to_door_1",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_door,
        origin=Origin(xyz=(right_hinge_x, BODY_D / 2.0 + DOOR_T / 2.0 + DOOR_MOUNT_GAP, door_hinge_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.45),
    )

    for index, (knob, knob_z) in enumerate(knob_parts):
        model.articulation(
            f"body_to_knob_{index}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=knob,
            origin=Origin(xyz=(CONTROL_X, BODY_D / 2.0 + CONTROL_PANEL_T, knob_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.5, velocity=6.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    left_door = object_model.get_part("door_0")
    right_door = object_model.get_part("door_1")
    left_hinge = object_model.get_articulation("body_to_door_0")
    right_hinge = object_model.get_articulation("body_to_door_1")
    knob_joints = [object_model.get_articulation(f"body_to_knob_{index}") for index in range(3)]
    knob_parts = [object_model.get_part(f"knob_{index}") for index in range(3)]

    ctx.expect_gap(
        left_door,
        body,
        axis="y",
        positive_elem="frame",
        negative_elem="housing",
        min_gap=0.0,
        max_gap=0.001,
        name="left door seats against the body face",
    )
    ctx.expect_gap(
        right_door,
        body,
        axis="y",
        positive_elem="frame",
        negative_elem="housing",
        min_gap=0.0,
        max_gap=0.001,
        name="right door seats against the body face",
    )
    ctx.expect_gap(
        right_door,
        left_door,
        axis="x",
        positive_elem="frame",
        negative_elem="frame",
        min_gap=0.001,
        max_gap=0.006,
        name="double doors meet with a narrow center reveal",
    )
    ctx.expect_gap(
        knob_parts[0],
        body,
        axis="y",
        positive_elem="knob",
        negative_elem="control_panel",
        min_gap=0.0,
        max_gap=0.001,
        name="top knob mounts directly on the control panel",
    )

    knob_positions = [ctx.part_world_position(knob) for knob in knob_parts]
    knob_stack_ok = (
        all(position is not None for position in knob_positions)
        and max(abs(knob_positions[index][0] - knob_positions[0][0]) for index in range(3)) < 0.002
        and knob_positions[0][2] > knob_positions[1][2] > knob_positions[2][2]
    )
    ctx.check(
        "control knobs form a vertical stack",
        knob_stack_ok,
        details=f"positions={knob_positions}",
    )

    continuous_knobs_ok = True
    for joint in knob_joints:
        limits = joint.motion_limits
        continuous_knobs_ok = continuous_knobs_ok and (
            joint.articulation_type == ArticulationType.CONTINUOUS
            and limits is not None
            and limits.lower is None
            and limits.upper is None
            and joint.axis == (0.0, 1.0, 0.0)
        )
    ctx.check(
        "control knobs use continuous front-shaft rotation",
        continuous_knobs_ok,
        details=", ".join(
            f"{joint.name}: type={joint.articulation_type}, axis={joint.axis}, limits={joint.motion_limits}"
            for joint in knob_joints
        ),
    )

    closed_left = ctx.part_element_world_aabb(left_door, elem="frame")
    closed_right = ctx.part_element_world_aabb(right_door, elem="frame")
    with ctx.pose({left_hinge: 1.30, right_hinge: 1.30}):
        open_left = ctx.part_element_world_aabb(left_door, elem="frame")
        open_right = ctx.part_element_world_aabb(right_door, elem="frame")

    left_swings_out = (
        _aabb_axis_value(closed_left, "max", 1) is not None
        and _aabb_axis_value(open_left, "max", 1) is not None
        and _aabb_axis_value(open_left, "max", 1) > _aabb_axis_value(closed_left, "max", 1) + 0.12
    )
    right_swings_out = (
        _aabb_axis_value(closed_right, "max", 1) is not None
        and _aabb_axis_value(open_right, "max", 1) is not None
        and _aabb_axis_value(open_right, "max", 1) > _aabb_axis_value(closed_right, "max", 1) + 0.12
    )
    ctx.check(
        "doors swing outward from the front opening",
        left_swings_out and right_swings_out,
        details=f"closed_left={closed_left}, open_left={open_left}, closed_right={closed_right}, open_right={open_right}",
    )

    return ctx.report()


object_model = build_object_model()
