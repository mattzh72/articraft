from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    DomeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_RADIUS = 0.115
LOWER_BAND_RADIUS = 0.109
BODY_HEIGHT = 0.170
LOWER_BAND_HEIGHT = 0.040
BODY_SHELL_THICKNESS = 0.0035

LID_OUTER_RADIUS = 0.121
LID_RIM_HEIGHT = 0.014
LID_DOME_HEIGHT = 0.034
LID_SHELL_THICKNESS = 0.003
LID_SEAT_GAP = 0.0

PANEL_WIDTH = 0.080
PANEL_DEPTH = 0.016
PANEL_HEIGHT = 0.050
PANEL_CENTER_Z = 0.084
DISPLAY_WIDTH = 0.046
DISPLAY_HEIGHT = 0.018
DISPLAY_DEPTH = 0.003
DISPLAY_CENTER_Z = 0.099

BUTTON_SIZE = 0.024
BUTTON_DEPTH = 0.010
BUTTON_CENTER_Z = 0.057
BUTTON_TRAVEL = 0.003

HANDLE_AXIS_Z = BODY_HEIGHT + 0.014
HANDLE_AXIS_Y = -0.090
HANDLE_BOSS_RADIUS = 0.008
HANDLE_BOSS_LENGTH = 0.016
HANDLE_BOSS_X = BODY_RADIUS + HANDLE_BOSS_LENGTH / 2.0 - 0.0025

HANDLE_HUB_RADIUS = 0.0075
HANDLE_HUB_LENGTH = 0.014
HANDLE_HUB_X = HANDLE_BOSS_X + HANDLE_BOSS_LENGTH / 2.0 + HANDLE_HUB_LENGTH / 2.0 + 0.002
HANDLE_GRIP_RADIUS = 0.007
HANDLE_GRIP_LENGTH = 0.164
HANDLE_GRIP_Y = 0.015
HANDLE_GRIP_Z = 0.100


def _segment_origin(start: tuple[float, float, float], end: tuple[float, float, float]) -> tuple[Origin, float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    center = ((start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0, (start[2] + end[2]) / 2.0)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    return Origin(xyz=center, rpy=(0.0, pitch, yaw)), length


def _build_body_shell() -> cq.Workplane:
    lower_core = cq.Workplane("XY").circle(LOWER_BAND_RADIUS).extrude(BODY_HEIGHT)
    upper_sleeve = (
        cq.Workplane("XY")
        .circle(BODY_RADIUS)
        .extrude(BODY_HEIGHT - LOWER_BAND_HEIGHT + 0.0015)
        .translate((0.0, 0.0, LOWER_BAND_HEIGHT - 0.0015))
    )
    outer = lower_core.union(upper_sleeve)
    inner = (
        cq.Workplane("XY")
        .circle(LOWER_BAND_RADIUS - BODY_SHELL_THICKNESS)
        .extrude(BODY_HEIGHT - BODY_SHELL_THICKNESS)
        .translate((0.0, 0.0, BODY_SHELL_THICKNESS))
    )
    shell = outer.cut(inner)

    control_panel = (
        cq.Workplane("XY")
        .box(PANEL_WIDTH, PANEL_DEPTH, PANEL_HEIGHT)
        .translate((0.0, BODY_RADIUS + PANEL_DEPTH / 2.0 - 0.003, PANEL_CENTER_Z))
    )
    latch_zone = (
        cq.Workplane("XY")
        .box(0.054, 0.010, 0.028)
        .translate((0.0, BODY_RADIUS + 0.004, BODY_HEIGHT - 0.020))
    )
    rear_hinge_block = (
        cq.Workplane("XY")
        .box(0.056, 0.018, 0.018)
        .translate((0.0, -BODY_RADIUS + 0.004, BODY_HEIGHT - 0.015))
    )
    return shell.union(control_panel).union(latch_zone).union(rear_hinge_block)


def _build_side_bracket(sign: float) -> cq.Workplane:
    lower_support = (
        cq.Workplane("XY")
        .box(0.040, 0.040, 0.024)
        .translate((sign * 0.101, -0.080, 0.156))
    )
    upper_neck = (
        cq.Workplane("XY")
        .box(0.012, 0.020, 0.016)
        .translate((sign * 0.118, HANDLE_AXIS_Y, 0.176))
    )
    boss = (
        cq.Workplane("YZ")
        .circle(HANDLE_BOSS_RADIUS)
        .extrude(HANDLE_BOSS_LENGTH, both=True)
        .translate((sign * HANDLE_BOSS_X, HANDLE_AXIS_Y, HANDLE_AXIS_Z))
    )
    return lower_support.union(upper_neck).union(boss)


def _build_lid_shell():
    rim = CylinderGeometry(LID_OUTER_RADIUS, LID_RIM_HEIGHT)
    rim.translate(0.0, 0.0, LID_RIM_HEIGHT / 2.0)
    dome = DomeGeometry(LID_OUTER_RADIUS)
    dome.scale(1.0, 1.0, LID_DOME_HEIGHT / LID_OUTER_RADIUS)
    dome.translate(0.0, 0.0, LID_RIM_HEIGHT)
    return rim.merge(dome)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rice_cooker")

    shell_white = model.material("shell_white", rgba=(0.94, 0.95, 0.96, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.72, 0.75, 0.78, 1.0))
    charcoal = model.material("charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    display_black = model.material("display_black", rgba=(0.07, 0.09, 0.11, 1.0))
    button_red = model.material("button_red", rgba=(0.69, 0.15, 0.15, 1.0))

    body = model.part("body")
    body.visual(mesh_from_cadquery(_build_body_shell(), "body_shell"), material=shell_white, name="body_shell")
    body.visual(
        Box((DISPLAY_WIDTH, DISPLAY_DEPTH, DISPLAY_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                BODY_RADIUS + PANEL_DEPTH - DISPLAY_DEPTH / 2.0 - 0.0006,
                DISPLAY_CENTER_Z,
            )
        ),
        material=display_black,
        name="display",
    )

    bracket_0 = model.part("bracket_0")
    bracket_0.visual(mesh_from_cadquery(_build_side_bracket(-1.0), "bracket_0"), material=shell_white, name="bracket_shell")
    bracket_1 = model.part("bracket_1")
    bracket_1.visual(mesh_from_cadquery(_build_side_bracket(1.0), "bracket_1"), material=shell_white, name="bracket_shell")

    lid = model.part("lid")
    lid.visual(
        mesh_from_geometry(_build_lid_shell(), "lid_shell"),
        origin=Origin(xyz=(0.0, LID_OUTER_RADIUS, 0.0)),
        material=shell_white,
        name="lid_shell",
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=HANDLE_GRIP_RADIUS, length=HANDLE_GRIP_LENGTH),
        origin=Origin(xyz=(0.0, HANDLE_GRIP_Y, HANDLE_GRIP_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="grip",
    )
    left_hub_center = (-HANDLE_HUB_X, 0.0, 0.0)
    right_hub_center = (HANDLE_HUB_X, 0.0, 0.0)
    left_grip_end = (-HANDLE_GRIP_LENGTH / 2.0, HANDLE_GRIP_Y, HANDLE_GRIP_Z)
    right_grip_end = (HANDLE_GRIP_LENGTH / 2.0, HANDLE_GRIP_Y, HANDLE_GRIP_Z)
    left_hub_inner = (-HANDLE_HUB_X + HANDLE_HUB_LENGTH / 2.0, 0.0, 0.0)
    right_hub_inner = (HANDLE_HUB_X - HANDLE_HUB_LENGTH / 2.0, 0.0, 0.0)
    left_arm_origin, left_arm_length = _segment_origin(left_hub_inner, left_grip_end)
    right_arm_origin, right_arm_length = _segment_origin(right_hub_inner, right_grip_end)
    handle.visual(
        Cylinder(radius=HANDLE_HUB_RADIUS, length=HANDLE_HUB_LENGTH),
        origin=Origin(xyz=left_hub_center, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="hub_0",
    )
    handle.visual(
        Cylinder(radius=HANDLE_HUB_RADIUS, length=HANDLE_HUB_LENGTH),
        origin=Origin(xyz=right_hub_center, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="hub_1",
    )
    handle.visual(
        Cylinder(radius=HANDLE_GRIP_RADIUS, length=left_arm_length),
        origin=left_arm_origin,
        material=charcoal,
        name="arm_0",
    )
    handle.visual(
        Cylinder(radius=HANDLE_GRIP_RADIUS, length=right_arm_length),
        origin=right_arm_origin,
        material=charcoal,
        name="arm_1",
    )

    button = model.part("button")
    button.visual(
        Box((BUTTON_SIZE, BUTTON_DEPTH, BUTTON_SIZE * 0.66)),
        origin=Origin(xyz=(0.0, BUTTON_DEPTH / 2.0 - 0.0005, 0.0)),
        material=button_red,
        name="button_cap",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -LID_OUTER_RADIUS, BODY_HEIGHT + LID_SEAT_GAP)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=0.0, upper=1.55),
    )
    model.articulation("body_to_bracket_0", ArticulationType.FIXED, parent=body, child=bracket_0, origin=Origin())
    model.articulation("body_to_bracket_1", ArticulationType.FIXED, parent=body, child=bracket_1, origin=Origin())
    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, HANDLE_AXIS_Y, HANDLE_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.8, lower=-1.05, upper=0.0),
    )
    model.articulation(
        "body_to_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=button,
        origin=Origin(xyz=(0.0, BODY_RADIUS + PANEL_DEPTH - 0.003, BUTTON_CENTER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.04, lower=0.0, upper=BUTTON_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    bracket_0 = object_model.get_part("bracket_0")
    bracket_1 = object_model.get_part("bracket_1")
    lid = object_model.get_part("lid")
    handle = object_model.get_part("handle")
    button = object_model.get_part("button")

    lid_hinge = object_model.get_articulation("body_to_lid")
    handle_hinge = object_model.get_articulation("body_to_handle")
    button_slide = object_model.get_articulation("body_to_button")

    lid_limits = lid_hinge.motion_limits
    handle_limits = handle_hinge.motion_limits
    button_limits = button_slide.motion_limits

    ctx.allow_overlap(
        body,
        bracket_0,
        elem_a="body_shell",
        elem_b="bracket_shell",
        reason="Each carry-handle bracket is modeled as a compact shell-mounted subassembly that is partially embedded into the cooker side wall.",
    )
    ctx.allow_overlap(
        body,
        bracket_1,
        elem_a="body_shell",
        elem_b="bracket_shell",
        reason="Each carry-handle bracket is modeled as a compact shell-mounted subassembly that is partially embedded into the cooker side wall.",
    )
    ctx.allow_overlap(
        bracket_0,
        handle,
        elem_a="bracket_shell",
        elem_b="arm_0",
        reason="The side pivot bracket is simplified as a compact solid support, so the handle arm roots intentionally sink slightly into the bracket volume.",
    )
    ctx.allow_overlap(
        bracket_1,
        handle,
        elem_a="bracket_shell",
        elem_b="arm_1",
        reason="The side pivot bracket is simplified as a compact solid support, so the handle arm roots intentionally sink slightly into the bracket volume.",
    )
    ctx.allow_overlap(
        bracket_0,
        handle,
        elem_a="bracket_shell",
        elem_b="hub_0",
        reason="The carry handle pivots are represented with simplified concentric hub solids nested into compact body brackets.",
    )
    ctx.allow_overlap(
        bracket_1,
        handle,
        elem_a="bracket_shell",
        elem_b="hub_1",
        reason="The carry handle pivots are represented with simplified concentric hub solids nested into compact body brackets.",
    )

    with ctx.pose({lid_hinge: 0.0, handle_hinge: 0.0, button_slide: 0.0}):
        ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.19, name="lid covers the cooker body")
        ctx.expect_gap(
            handle,
            lid,
            axis="z",
            positive_elem="grip",
            negative_elem="lid_shell",
            min_gap=0.004,
            name="carry handle clears the lid shell",
        )
        ctx.expect_overlap(
            button,
            body,
            axes="xz",
            elem_a="button_cap",
            min_overlap=0.015,
            name="power button stays aligned on the front panel",
        )

    if lid_limits is not None and lid_limits.upper is not None:
        closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
        with ctx.pose({lid_hinge: lid_limits.upper}):
            open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
        ctx.check(
            "lid opens upward",
            closed_lid_aabb is not None
            and open_lid_aabb is not None
            and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.025,
            details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
        )

    if handle_limits is not None and handle_limits.upper is not None:
        closed_grip_aabb = ctx.part_element_world_aabb(handle, elem="grip")
        with ctx.pose({handle_hinge: handle_limits.lower}):
            lowered_grip_aabb = ctx.part_element_world_aabb(handle, elem="grip")
        ctx.check(
            "handle folds downward toward the lid",
            closed_grip_aabb is not None
            and lowered_grip_aabb is not None
            and lowered_grip_aabb[1][2] < closed_grip_aabb[1][2] - 0.030,
            details=f"rest={closed_grip_aabb}, lowered={lowered_grip_aabb}",
        )

    if button_limits is not None and button_limits.upper is not None:
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({button_slide: button_limits.upper}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            "power button presses inward",
            rest_pos is not None and pressed_pos is not None and pressed_pos[1] < rest_pos[1] - 0.0015,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
