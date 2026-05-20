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


BODY_DEPTH = 0.052
BODY_WIDTH = 0.070
LOWER_BODY_HEIGHT = 0.094
UPPER_BODY_HEIGHT = 0.022
BODY_BASE_Z = 0.012
BODY_TOP_Z = BODY_BASE_Z + LOWER_BODY_HEIGHT + UPPER_BODY_HEIGHT
SWIVEL_PLINTH_HEIGHT = 0.006
SWIVEL_Z = BODY_TOP_Z + SWIVEL_PLINTH_HEIGHT

HEAD_WIDTH = 0.084
HEAD_DEPTH = 0.050
HEAD_HEIGHT = 0.046


def _y_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XZ", origin=center).circle(radius).extrude(length / 2.0, both=True)


def _x_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("YZ", origin=center).circle(radius).extrude(length / 2.0, both=True)


def _z_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY", origin=center).circle(radius).extrude(length / 2.0, both=True)


def _body_shell_shape() -> cq.Workplane:
    lower_outer = (
        cq.Workplane("XY")
        .box(BODY_DEPTH, BODY_WIDTH, LOWER_BODY_HEIGHT)
        .edges("|Z")
        .fillet(0.006)
        .translate((0.0, 0.0, BODY_BASE_Z + LOWER_BODY_HEIGHT / 2.0))
    )
    upper_outer = (
        cq.Workplane("XY")
        .box(0.046, 0.062, UPPER_BODY_HEIGHT)
        .edges("|Z")
        .fillet(0.004)
        .translate((0.0, 0.0, BODY_BASE_Z + LOWER_BODY_HEIGHT + UPPER_BODY_HEIGHT / 2.0))
    )
    swivel_plinth = (
        cq.Workplane("XY")
        .box(0.032, 0.024, SWIVEL_PLINTH_HEIGHT)
        .edges("|Z")
        .fillet(0.002)
        .translate((0.0, 0.0, BODY_TOP_Z + SWIVEL_PLINTH_HEIGHT / 2.0))
    )

    shell = lower_outer.union(upper_outer).union(swivel_plinth)

    inner_cavity = cq.Workplane("XY").box(0.042, 0.060, 0.092).translate((0.0, 0.0, 0.068))
    door_opening = cq.Workplane("XY").box(0.042, 0.020, 0.086).translate((0.0, 0.029, 0.068))

    return shell.cut(inner_cavity).cut(door_opening)


def _body_hinge_shape() -> cq.Workplane:
    hinge_x = -0.020
    hinge_y = BODY_WIDTH / 2.0 - 0.0015
    upper_knuckle = _z_cylinder(0.0024, 0.020, (hinge_x, hinge_y, 0.097))
    lower_knuckle = _z_cylinder(0.0024, 0.020, (hinge_x, hinge_y, 0.039))
    return upper_knuckle.union(lower_knuckle)


def _neck_base_shape() -> cq.Workplane:
    hinge_z = 0.046

    collar = cq.Workplane("XY").circle(0.016).extrude(0.008)
    column = cq.Workplane("XY").box(0.022, 0.018, 0.026).translate((0.0, 0.0, 0.019))
    bridge = cq.Workplane("XY").box(0.010, 0.100, 0.010).translate((-0.008, 0.0, 0.021))
    return collar.union(column).union(bridge)


def _neck_arm_shape(sign: float) -> cq.Workplane:
    arm_y = sign * 0.048
    hinge_z = 0.046
    arm = cq.Workplane("XY").box(0.014, 0.006, 0.038).translate((-0.003, arm_y, 0.032))
    boss = _y_cylinder(0.007, 0.008, (0.0, arm_y, hinge_z))
    bore = _y_cylinder(0.0052, 0.012, (0.0, arm_y, hinge_z))
    return arm.union(boss).cut(bore)


def _head_shell_shape() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .box(HEAD_DEPTH, HEAD_WIDTH, HEAD_HEIGHT)
        .edges("|Z")
        .fillet(0.006)
        .translate((0.027, 0.0, 0.030))
    )
    front_recess = cq.Workplane("XY").box(0.014, 0.070, 0.030).translate((0.047, 0.0, 0.030))
    shell = shell.cut(front_recess)
    left_cheek = _x_cylinder(0.009, 0.034, (0.024, 0.032, 0.030))
    right_cheek = _x_cylinder(0.009, 0.034, (0.024, -0.032, 0.030))
    shell = shell.union(left_cheek).union(right_cheek)
    left_lug = cq.Workplane("XY").box(0.010, 0.004, 0.014).translate((0.005, 0.040, 0.004))
    right_lug = cq.Workplane("XY").box(0.010, 0.004, 0.014).translate((0.005, -0.040, 0.004))
    shell = shell.union(left_lug).union(right_lug)

    trunnion_offset_y = HEAD_WIDTH / 2.0 + 0.005
    left_trunnion = _y_cylinder(0.0052, 0.010, (0.0, trunnion_offset_y, 0.0))
    right_trunnion = _y_cylinder(0.0052, 0.010, (0.0, -trunnion_offset_y, 0.0))

    return shell.union(left_trunnion).union(right_trunnion)


def _battery_door_hardware_shape() -> cq.Workplane:
    hinge_barrel = _z_cylinder(0.0022, 0.028, (0.0, 0.0, 0.0))
    hinge_web = cq.Workplane("XY").box(0.004, 0.003, 0.030).translate((0.002, 0.0, 0.0))
    return hinge_barrel.union(hinge_web)


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) / 2.0 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="speedlight")

    body_black = model.material("body_black", color=(0.12, 0.12, 0.13))
    trim_dark = model.material("trim_dark", color=(0.18, 0.18, 0.20))
    diffuser_white = model.material("diffuser_white", color=(0.92, 0.92, 0.88))

    body = model.part("body")
    body.visual(mesh_from_cadquery(_body_shell_shape(), "body_shell"), material=body_black, name="body_shell")
    body.visual(mesh_from_cadquery(_body_hinge_shape(), "body_hinge"), material=trim_dark, name="door_hinge")
    body.visual(
        Box((0.030, 0.026, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=trim_dark,
        name="shoe_mount",
    )
    body.visual(
        Box((0.022, 0.018, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=trim_dark,
        name="shoe_foot",
    )

    neck = model.part("neck")
    neck.visual(mesh_from_cadquery(_neck_base_shape(), "neck_base"), material=body_black, name="neck_base")
    neck.visual(mesh_from_cadquery(_neck_arm_shape(1.0), "neck_arm_0"), material=body_black, name="arm_0")
    neck.visual(mesh_from_cadquery(_neck_arm_shape(-1.0), "neck_arm_1"), material=body_black, name="arm_1")

    head = model.part("head")
    head.visual(mesh_from_cadquery(_head_shell_shape(), "head_shell"), material=body_black, name="head_shell")
    head.visual(
        Box((0.006, 0.068, 0.028)),
        origin=Origin(xyz=(0.043, 0.0, 0.030)),
        material=diffuser_white,
        name="diffuser",
    )

    battery_door = model.part("battery_door")
    battery_door.visual(
        Box((0.037, 0.003, 0.084)),
        origin=Origin(xyz=(0.0225, 0.0, 0.0)),
        material=body_black,
        name="panel",
    )
    battery_door.visual(
        mesh_from_cadquery(_battery_door_hardware_shape(), "battery_door_hardware"),
        material=trim_dark,
        name="hinge_barrel",
    )

    model.articulation(
        "body_to_neck",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=neck,
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=8.0),
    )
    model.articulation(
        "neck_to_head",
        ArticulationType.REVOLUTE,
        parent=neck,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=4.0, lower=-0.12, upper=1.57),
    )
    model.articulation(
        "body_to_battery_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=battery_door,
        origin=Origin(xyz=(-0.020, BODY_WIDTH / 2.0 - 0.0015, 0.068)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=2.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    head = object_model.get_part("head")
    battery_door = object_model.get_part("battery_door")

    swivel = object_model.get_articulation("body_to_neck")
    tilt = object_model.get_articulation("neck_to_head")
    door_hinge = object_model.get_articulation("body_to_battery_door")

    ctx.allow_overlap(
        head,
        "neck",
        elem_a="head_shell",
        elem_b="arm_0",
        reason="The head trunnion is intentionally captured inside the exposed left yoke boss.",
    )
    ctx.allow_overlap(
        head,
        "neck",
        elem_a="head_shell",
        elem_b="arm_1",
        reason="The head trunnion is intentionally captured inside the exposed right yoke boss.",
    )

    ctx.expect_overlap(
        battery_door,
        body,
        axes="xz",
        elem_a="panel",
        elem_b="body_shell",
        min_overlap=0.036,
        name="battery door covers the side opening",
    )

    closed_shell = ctx.part_element_world_aabb(body, elem="body_shell")
    closed_panel = ctx.part_element_world_aabb(battery_door, elem="panel")
    closed_shell_max_y = closed_shell[1][1] if closed_shell is not None else None
    closed_panel_max_y = closed_panel[1][1] if closed_panel is not None else None
    ctx.check(
        "battery door sits flush with the body side",
        closed_shell_max_y is not None
        and closed_panel_max_y is not None
        and abs(closed_shell_max_y - closed_panel_max_y) <= 0.0007,
        details=f"body_shell_max_y={closed_shell_max_y}, door_panel_max_y={closed_panel_max_y}",
    )

    closed_door_center = _aabb_center(closed_panel)
    with ctx.pose({door_hinge: 1.2}):
        opened_panel = ctx.part_element_world_aabb(battery_door, elem="panel")
        opened_door_center = _aabb_center(opened_panel)
    ctx.check(
        "battery door opens outward from the side wall",
        closed_door_center is not None
        and opened_door_center is not None
        and opened_door_center[1] > closed_door_center[1] + 0.012,
        details=f"closed_center={closed_door_center}, opened_center={opened_door_center}",
    )

    closed_diffuser = _aabb_center(ctx.part_element_world_aabb(head, elem="diffuser"))
    with ctx.pose({tilt: 1.35}):
        tilted_diffuser = _aabb_center(ctx.part_element_world_aabb(head, elem="diffuser"))
    ctx.check(
        "flash head tilts upward",
        closed_diffuser is not None
        and tilted_diffuser is not None
        and tilted_diffuser[2] > closed_diffuser[2] + 0.015,
        details=f"closed_diffuser={closed_diffuser}, tilted_diffuser={tilted_diffuser}",
    )

    with ctx.pose({swivel: math.pi / 2.0}):
        swiveled_diffuser = _aabb_center(ctx.part_element_world_aabb(head, elem="diffuser"))
    ctx.check(
        "neck swivel turns the head around the vertical axis",
        closed_diffuser is not None
        and swiveled_diffuser is not None
        and swiveled_diffuser[1] > closed_diffuser[1] + 0.025
        and abs(swiveled_diffuser[0]) < closed_diffuser[0] - 0.010,
        details=f"closed_diffuser={closed_diffuser}, swiveled_diffuser={swiveled_diffuser}",
    )

    return ctx.report()


object_model = build_object_model()
