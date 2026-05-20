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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


CABINET_WIDTH = 0.82
CABINET_DEPTH_REAR = -0.37
CABINET_HEIGHT = 1.93

BODY_PROFILE = (
    (CABINET_DEPTH_REAR, 0.00),
    (0.36, 0.00),
    (0.36, 0.72),
    (0.44, 0.84),
    (0.30, 0.96),
    (0.12, 1.58),
    (0.10, 1.84),
    (0.10, CABINET_HEIGHT),
    (CABINET_DEPTH_REAR, CABINET_HEIGHT),
)

BELLY_CAVITY_DEPTH = 0.18
BELLY_CAVITY_WIDTH = 0.56
BELLY_CAVITY_HEIGHT = 0.50
BELLY_CAVITY_CENTER_X = 0.27
BELLY_CAVITY_CENTER_Z = 0.34

DOOR_THICKNESS = 0.035
DOOR_WIDTH = 0.53
DOOR_HEIGHT = 0.46
DOOR_HINGE_X = 0.325
DOOR_BOTTOM_Z = 0.10
DOOR_FRONT_X = DOOR_HINGE_X + DOOR_THICKNESS

DISPLAY_PITCH = math.atan2(0.12 - 0.30, 1.58 - 0.96)
DISPLAY_CENTER_X = 0.21
DISPLAY_CENTER_Z = 1.28
DISPLAY_BEZEL_THICKNESS = 0.024
DISPLAY_BEZEL_WIDTH = 0.60
DISPLAY_BEZEL_HEIGHT = 0.48
DISPLAY_GLASS_THICKNESS = 0.006
DISPLAY_GLASS_WIDTH = 0.54
DISPLAY_GLASS_HEIGHT = 0.42

DECK_NORMAL_PITCH = math.atan2(0.12, 0.14)
BUTTON_RADIUS = 0.045
BUTTON_LENGTH = 0.030
BUTTON_SURFACE_X = 0.37
BUTTON_SURFACE_Z = 0.90
BUTTON_Y_POSITIONS = (-0.25, -0.08, 0.08, 0.25)

HANDLE_PIVOT_X = 0.22
HANDLE_PIVOT_Y = CABINET_WIDTH / 2.0 + 0.072
HANDLE_PIVOT_Z = 1.08
HANDLE_VECTOR_X = 0.34
HANDLE_VECTOR_Z = -0.20
HANDLE_ARM_LENGTH = math.hypot(HANDLE_VECTOR_X, HANDLE_VECTOR_Z)
HANDLE_ARM_PITCH = math.atan2(HANDLE_VECTOR_X, HANDLE_VECTOR_Z)


def _panel_normal_from_pitch(pitch: float) -> tuple[float, float, float]:
    return (math.cos(pitch), 0.0, -math.sin(pitch))


def _axis_from_pitch(pitch: float) -> tuple[float, float, float]:
    return (math.sin(pitch), 0.0, math.cos(pitch))


def _aabb_center(bounds) -> tuple[float, float, float] | None:
    if bounds is None:
        return None
    lo, hi = bounds
    return tuple((lo_i + hi_i) * 0.5 for lo_i, hi_i in zip(lo, hi))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slant_top_video_slot_machine")

    cabinet_finish = model.material("cabinet_finish", rgba=(0.14, 0.15, 0.17, 1.0))
    trim_finish = model.material("trim_finish", rgba=(0.05, 0.05, 0.06, 1.0))
    screen_finish = model.material("screen_finish", rgba=(0.05, 0.09, 0.14, 1.0))
    button_red = model.material("button_red", rgba=(0.78, 0.13, 0.13, 1.0))
    button_green = model.material("button_green", rgba=(0.10, 0.55, 0.24, 1.0))
    button_blue = model.material("button_blue", rgba=(0.12, 0.34, 0.72, 1.0))
    button_gold = model.material("button_gold", rgba=(0.79, 0.62, 0.18, 1.0))
    door_finish = model.material("door_finish", rgba=(0.22, 0.22, 0.24, 1.0))
    metal_finish = model.material("metal_finish", rgba=(0.68, 0.69, 0.72, 1.0))
    handle_finish = model.material("handle_finish", rgba=(0.86, 0.87, 0.88, 1.0))
    knob_finish = model.material("knob_finish", rgba=(0.12, 0.12, 0.13, 1.0))

    cabinet = model.part("cabinet")

    shell = (
        cq.Workplane("XZ")
        .polyline(BODY_PROFILE)
        .close()
        .extrude(CABINET_WIDTH / 2.0, both=True)
    )
    belly_cavity = (
        cq.Workplane("XY")
        .box(BELLY_CAVITY_DEPTH, BELLY_CAVITY_WIDTH, BELLY_CAVITY_HEIGHT)
        .translate((BELLY_CAVITY_CENTER_X, 0.0, BELLY_CAVITY_CENTER_Z))
    )
    shell = shell.cut(belly_cavity)

    cabinet.visual(
        mesh_from_cadquery(shell, "slot_machine_shell"),
        material=cabinet_finish,
        name="shell",
    )

    display_normal = _panel_normal_from_pitch(DISPLAY_PITCH)
    bezel_center = (
        DISPLAY_CENTER_X + display_normal[0] * 0.004,
        0.0,
        DISPLAY_CENTER_Z + display_normal[2] * 0.004,
    )
    screen_center = (
        DISPLAY_CENTER_X + display_normal[0] * 0.012,
        0.0,
        DISPLAY_CENTER_Z + display_normal[2] * 0.012,
    )

    cabinet.visual(
        Box((DISPLAY_BEZEL_THICKNESS, DISPLAY_BEZEL_WIDTH, DISPLAY_BEZEL_HEIGHT)),
        origin=Origin(xyz=bezel_center, rpy=(0.0, DISPLAY_PITCH, 0.0)),
        material=trim_finish,
        name="display_bezel",
    )
    cabinet.visual(
        Box((DISPLAY_GLASS_THICKNESS, DISPLAY_GLASS_WIDTH, DISPLAY_GLASS_HEIGHT)),
        origin=Origin(xyz=screen_center, rpy=(0.0, DISPLAY_PITCH, 0.0)),
        material=screen_finish,
        name="screen",
    )

    button_axis = _axis_from_pitch(DECK_NORMAL_PITCH)
    button_materials = (button_red, button_green, button_blue, button_gold)
    for index, (y_pos, material) in enumerate(zip(BUTTON_Y_POSITIONS, button_materials)):
        button_center = (
            BUTTON_SURFACE_X + button_axis[0] * (BUTTON_LENGTH * 0.35),
            y_pos,
            BUTTON_SURFACE_Z + button_axis[2] * (BUTTON_LENGTH * 0.35),
        )
        cabinet.visual(
            Cylinder(radius=BUTTON_RADIUS, length=BUTTON_LENGTH),
            origin=Origin(xyz=button_center, rpy=(0.0, DECK_NORMAL_PITCH, 0.0)),
            material=material,
            name=f"button_{index}",
        )

    cabinet.visual(
        Box((0.020, 0.020, 0.50)),
        origin=Origin(xyz=(0.350, -0.275, 0.35)),
        material=metal_finish,
        name="door_jamb",
    )

    door = model.part("door")
    door.visual(
        Box((DOOR_THICKNESS, DOOR_WIDTH, DOOR_HEIGHT)),
        origin=Origin(xyz=(DOOR_THICKNESS / 2.0, DOOR_WIDTH / 2.0, DOOR_HEIGHT / 2.0)),
        material=door_finish,
        name="door_panel",
    )
    door.visual(
        Box((0.036, 0.020, 0.48)),
        origin=Origin(xyz=(0.053, -0.010, DOOR_HEIGHT / 2.0)),
        material=metal_finish,
        name="hinge_leaf",
    )
    door.visual(
        Box((0.020, 0.15, 0.04)),
        origin=Origin(xyz=(DOOR_THICKNESS + 0.005, DOOR_WIDTH * 0.54, 0.11)),
        material=metal_finish,
        name="door_pull",
    )
    door.visual(
        Cylinder(radius=0.018, length=0.020),
        origin=Origin(
            xyz=(DOOR_THICKNESS + 0.010, DOOR_WIDTH * 0.80, DOOR_HEIGHT * 0.58),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=metal_finish,
        name="door_lock",
    )

    bracket = model.part("bracket")
    bracket.visual(
        Box((0.22, 0.010, 0.40)),
        origin=Origin(xyz=(0.0, -0.067, 0.0)),
        material=metal_finish,
        name="mount_pad",
    )
    bracket.visual(
        Box((0.05, 0.050, 0.05)),
        origin=Origin(xyz=(0.0, -0.0425, 0.10)),
        material=metal_finish,
        name="upper_spacer",
    )
    bracket.visual(
        Box((0.05, 0.050, 0.05)),
        origin=Origin(xyz=(0.0, -0.0425, -0.10)),
        material=metal_finish,
        name="lower_spacer",
    )
    bracket.visual(
        Box((0.16, 0.010, 0.28)),
        origin=Origin(xyz=(0.0, -0.020, 0.0)),
        material=metal_finish,
        name="inner_ear",
    )
    bracket.visual(
        Box((0.16, 0.010, 0.28)),
        origin=Origin(xyz=(0.0, 0.020, 0.0)),
        material=metal_finish,
        name="outer_ear",
    )
    bracket.visual(
        Box((0.12, 0.040, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.135)),
        material=metal_finish,
        name="upper_bridge",
    )
    bracket.visual(
        Box((0.12, 0.040, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, -0.135)),
        material=metal_finish,
        name="lower_bridge",
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.024, length=0.030),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=handle_finish,
        name="hub",
    )
    handle.visual(
        Cylinder(radius=0.017, length=HANDLE_ARM_LENGTH),
        origin=Origin(
            xyz=(HANDLE_VECTOR_X / 2.0, 0.0, HANDLE_VECTOR_Z / 2.0),
            rpy=(0.0, HANDLE_ARM_PITCH, 0.0),
        ),
        material=handle_finish,
        name="arm",
    )
    handle.visual(
        Sphere(radius=0.043),
        origin=Origin(xyz=(HANDLE_VECTOR_X, 0.0, HANDLE_VECTOR_Z)),
        material=knob_finish,
        name="knob",
    )

    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(DOOR_HINGE_X, -DOOR_WIDTH / 2.0, DOOR_BOTTOM_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.40, effort=20.0, velocity=1.2),
    )
    model.articulation(
        "cabinet_to_bracket",
        ArticulationType.FIXED,
        parent=cabinet,
        child=bracket,
        origin=Origin(xyz=(HANDLE_PIVOT_X, HANDLE_PIVOT_Y, HANDLE_PIVOT_Z)),
    )
    model.articulation(
        "bracket_to_handle",
        ArticulationType.REVOLUTE,
        parent=bracket,
        child=handle,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.05, effort=15.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    bracket = object_model.get_part("bracket")
    handle = object_model.get_part("handle")
    door_hinge = object_model.get_articulation("cabinet_to_door")
    handle_joint = object_model.get_articulation("bracket_to_handle")

    ctx.allow_overlap(
        cabinet,
        door,
        elem_a="shell",
        elem_b="hinge_leaf",
        reason="The cabinet-side hinge strap is intentionally simplified as a leaf nested into the shell thickness at the belly door edge.",
    )

    ctx.expect_gap(
        bracket,
        cabinet,
        axis="y",
        positive_elem="inner_ear",
        negative_elem="shell",
        min_gap=0.025,
        max_gap=0.055,
        name="handle bracket stands off from the cabinet wall",
    )
    ctx.expect_gap(
        handle,
        cabinet,
        axis="y",
        negative_elem="shell",
        min_gap=0.010,
        name="resting handle stays outboard of the cabinet side",
    )

    closed_door_bounds = ctx.part_element_world_aabb(door, elem="door_panel")
    ctx.check(
        "belly door sits flush with the lower front opening",
        closed_door_bounds is not None and abs(closed_door_bounds[1][0] - DOOR_FRONT_X) < 0.006,
        details=f"door_panel_bounds={closed_door_bounds}",
    )
    ctx.expect_contact(
        door,
        cabinet,
        elem_a="hinge_leaf",
        elem_b="door_jamb",
        name="belly door shows a mounted side hinge connection",
    )

    door_upper = door_hinge.motion_limits.upper if door_hinge.motion_limits is not None else None
    if door_upper is not None:
        with ctx.pose({door_hinge: door_upper}):
            open_door_bounds = ctx.part_element_world_aabb(door, elem="door_panel")
        ctx.check(
            "belly door swings outward on its side hinge",
            closed_door_bounds is not None
            and open_door_bounds is not None
            and open_door_bounds[1][0] > closed_door_bounds[1][0] + 0.18,
            details=f"closed={closed_door_bounds}, open={open_door_bounds}",
        )

    rest_knob_bounds = ctx.part_element_world_aabb(handle, elem="knob")
    handle_upper = handle_joint.motion_limits.upper if handle_joint.motion_limits is not None else None
    if handle_upper is not None:
        with ctx.pose({handle_joint: handle_upper}):
            pulled_knob_bounds = ctx.part_element_world_aabb(handle, elem="knob")
            ctx.expect_gap(
                handle,
                cabinet,
                axis="y",
                negative_elem="shell",
                min_gap=0.010,
                name="pulled handle stays clear of the cabinet side",
            )
        rest_knob_center = _aabb_center(rest_knob_bounds)
        pulled_knob_center = _aabb_center(pulled_knob_bounds)
        ctx.check(
            "pull handle drops through a traditional downward pull arc",
            rest_knob_center is not None
            and pulled_knob_center is not None
            and pulled_knob_center[2] < rest_knob_center[2] - 0.14
            and pulled_knob_center[0] < rest_knob_center[0] - 0.10,
            details=f"rest={rest_knob_center}, pulled={pulled_knob_center}",
        )

    return ctx.report()


object_model = build_object_model()
