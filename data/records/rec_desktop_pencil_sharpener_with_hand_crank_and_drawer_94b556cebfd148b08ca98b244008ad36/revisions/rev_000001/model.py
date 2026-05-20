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

BODY_L = 0.136
BODY_W = 0.092
BODY_H = 0.118
WALL = 0.004
BOTTOM = 0.006
FRONT_SKIN = 0.009

PORT_RADIUS = 0.006
PORT_Z = 0.078
PORT_TUNNEL = 0.043

DRAWER_RUNNER_W = 0.003
DRAWER_CORE_W = 0.067
DRAWER_W = DRAWER_CORE_W + (2.0 * DRAWER_RUNNER_W)
DRAWER_H = 0.039
DRAWER_L = 0.075
DRAWER_FLOOR = 0.003
DRAWER_WALL = 0.003
DRAWER_BOTTOM_Z = 0.010
DRAWER_CLOSED_X = (BODY_L / 2.0) - (DRAWER_L / 2.0)
DRAWER_TRAVEL = 0.040

HUB_X = 0.014
HUB_Z = 0.073
HUB_RADIUS = 0.015
HUB_THICK = 0.010
HUB_COLLAR_RADIUS = 0.0105
HUB_COLLAR_THICK = 0.007
HUB_HINGE_X = 0.021
HUB_HINGE_Y = 0.010
HUB_HINGE_RADIUS = 0.0045
HUB_HINGE_LENGTH = 0.006

ARM_LEN = 0.058
ARM_BAR_W = 0.010
ARM_BAR_T = 0.006
ARM_FOLD_LIMIT = 1.35


def _box_between(
    min_x: float,
    max_x: float,
    min_y: float,
    max_y: float,
    min_z: float,
    max_z: float,
) -> cq.Workplane:
    return cq.Workplane("XY").box(
        max_x - min_x,
        max_y - min_y,
        max_z - min_z,
    ).translate(
        (
            (min_x + max_x) / 2.0,
            (min_y + max_y) / 2.0,
            (min_z + max_z) / 2.0,
        )
    )


def _y_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate(center)
    )


def _z_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((center[0], center[1], center[2] - (length / 2.0)))
    )


def _body_shape() -> cq.Workplane:
    overlap = 0.0008
    drawer_open_half_w = (DRAWER_W / 2.0) + 0.002
    drawer_top = DRAWER_BOTTOM_Z + DRAWER_H + 0.001

    left_wall = _box_between(
        -BODY_L / 2.0,
        (BODY_L / 2.0) - FRONT_SKIN + overlap,
        -BODY_W / 2.0,
        -BODY_W / 2.0 + WALL + overlap,
        0.0,
        BODY_H,
    )
    right_wall = _box_between(
        -BODY_L / 2.0,
        (BODY_L / 2.0) - FRONT_SKIN + overlap,
        BODY_W / 2.0 - WALL - overlap,
        BODY_W / 2.0,
        0.0,
        BODY_H,
    )
    back_wall = _box_between(
        -BODY_L / 2.0,
        -BODY_L / 2.0 + WALL + overlap,
        -BODY_W / 2.0,
        BODY_W / 2.0,
        0.0,
        BODY_H,
    )
    top_shell = _box_between(
        -BODY_L / 2.0 + WALL - overlap,
        BODY_L / 2.0,
        -BODY_W / 2.0 + WALL - overlap,
        BODY_W / 2.0 - WALL + overlap,
        BODY_H - WALL - overlap,
        BODY_H,
    )
    rear_floor = _box_between(
        -BODY_L / 2.0 + WALL - overlap,
        DRAWER_CLOSED_X - (DRAWER_L / 2.0) - 0.004,
        -BODY_W / 2.0 + WALL - overlap,
        BODY_W / 2.0 - WALL + overlap,
        0.0,
        BOTTOM,
    )
    front_sill = _box_between(
        BODY_L / 2.0 - FRONT_SKIN,
        BODY_L / 2.0,
        -BODY_W / 2.0,
        BODY_W / 2.0,
        0.0,
        DRAWER_BOTTOM_Z + overlap,
    )
    left_jamb = _box_between(
        BODY_L / 2.0 - FRONT_SKIN,
        BODY_L / 2.0,
        -BODY_W / 2.0,
        -drawer_open_half_w,
        DRAWER_BOTTOM_Z - overlap,
        drawer_top,
    )
    right_jamb = _box_between(
        BODY_L / 2.0 - FRONT_SKIN,
        BODY_L / 2.0,
        drawer_open_half_w,
        BODY_W / 2.0,
        DRAWER_BOTTOM_Z - overlap,
        drawer_top,
    )
    upper_front = _box_between(
        BODY_L / 2.0 - FRONT_SKIN,
        BODY_L / 2.0,
        -BODY_W / 2.0,
        BODY_W / 2.0,
        drawer_top - overlap,
        BODY_H - (WALL / 2.0),
    )
    cutter_bulkhead = _box_between(
        -0.020,
        -0.012,
        -0.025,
        0.025,
        0.051,
        0.094,
    )

    body = (
        left_wall.union(right_wall)
        .union(back_wall)
        .union(top_shell)
        .union(rear_floor)
        .union(front_sill)
        .union(left_jamb)
        .union(right_jamb)
        .union(upper_front)
        .union(cutter_bulkhead)
    )

    port_cut = (
        cq.Workplane("YZ")
        .workplane(offset=(BODY_L / 2.0) - PORT_TUNNEL)
        .center(0.0, PORT_Z)
        .circle(PORT_RADIUS)
        .extrude(PORT_TUNNEL + 0.004)
    )
    port_bevel = (
        cq.Workplane("YZ")
        .workplane(offset=(BODY_L / 2.0) - 0.003)
        .center(0.0, PORT_Z)
        .circle(PORT_RADIUS + 0.0018)
        .extrude(0.004)
    )
    crank_recess = (
        cq.Workplane("XZ")
        .workplane(offset=(BODY_W / 2.0) - 0.0035)
        .center(HUB_X, HUB_Z)
        .circle(0.0065)
        .extrude(0.005)
    )

    return body.cut(port_cut).cut(port_bevel).cut(crank_recess)


def _drawer_shape() -> cq.Workplane:
    tray = _box_between(
        -DRAWER_L / 2.0,
        DRAWER_L / 2.0,
        -DRAWER_CORE_W / 2.0,
        DRAWER_CORE_W / 2.0,
        0.0,
        DRAWER_H,
    )
    tray = tray.edges("|Z").fillet(0.002)

    inner_cavity = _box_between(
        -DRAWER_L / 2.0 + DRAWER_WALL,
        DRAWER_L / 2.0 - DRAWER_WALL,
        -DRAWER_CORE_W / 2.0 + DRAWER_WALL,
        DRAWER_CORE_W / 2.0 - DRAWER_WALL,
        DRAWER_FLOOR,
        DRAWER_H + 0.002,
    )
    tray = tray.cut(inner_cavity)

    left_runner = _box_between(
        -DRAWER_L / 2.0 + 0.006,
        DRAWER_L / 2.0 - 0.008,
        -DRAWER_W / 2.0,
        -DRAWER_CORE_W / 2.0,
        0.010,
        0.018,
    )
    right_runner = _box_between(
        -DRAWER_L / 2.0 + 0.006,
        DRAWER_L / 2.0 - 0.008,
        DRAWER_CORE_W / 2.0,
        DRAWER_W / 2.0,
        0.010,
        0.018,
    )
    tray = tray.union(left_runner).union(right_runner)

    finger_pull = (
        cq.Workplane("XZ")
        .workplane(offset=-(DRAWER_W / 2.0) - 0.001)
        .center((DRAWER_L / 2.0) - 0.006, 0.022)
        .circle(0.011)
        .extrude(DRAWER_W + 0.002)
    )
    tray = tray.cut(finger_pull)

    return tray


def _hub_shape() -> cq.Workplane:
    disk = _y_cylinder(HUB_RADIUS, HUB_THICK, (0.0, HUB_THICK / 2.0, 0.0))
    collar = _y_cylinder(
        HUB_COLLAR_RADIUS,
        HUB_COLLAR_THICK,
        (0.0, HUB_THICK + (HUB_COLLAR_THICK / 2.0), 0.0),
    )
    spoke = _box_between(
        0.0,
        HUB_HINGE_X,
        HUB_HINGE_Y - 0.0045,
        HUB_HINGE_Y + 0.0045,
        -0.004,
        0.004,
    )
    hinge_pad = _y_cylinder(
        0.0035,
        0.004,
        (HUB_HINGE_X, HUB_HINGE_Y, 0.0),
    )
    return disk.union(collar).union(spoke).union(hinge_pad)


def _arm_shape() -> cq.Workplane:
    arm = _box_between(
        0.012,
        ARM_LEN,
        -ARM_BAR_W / 2.0,
        ARM_BAR_W / 2.0,
        -ARM_BAR_T / 2.0,
        ARM_BAR_T / 2.0,
    )

    hinge_root = _box_between(
        0.0,
        0.022,
        -0.006,
        0.006,
        -0.0045,
        0.0045,
    )
    arm = arm.union(hinge_root)

    relief = _box_between(
        -0.001,
        0.004,
        -0.0025,
        0.0025,
        -0.006,
        0.006,
    )
    arm = arm.cut(relief)

    grip_stem = _z_cylinder(0.003, 0.013, (ARM_LEN - 0.007, 0.0, 0.0065))
    grip = _z_cylinder(0.0052, 0.022, (ARM_LEN - 0.007, 0.0, 0.016))
    arm = arm.union(grip_stem).union(grip)

    return arm


def _front_panel_shape() -> cq.Workplane:
    drawer_top = DRAWER_BOTTOM_Z + DRAWER_H + 0.001
    panel = _box_between(
        BODY_L / 2.0 - FRONT_SKIN,
        BODY_L / 2.0,
        -BODY_W / 2.0,
        BODY_W / 2.0,
        drawer_top - 0.0005,
        BODY_H,
    )
    port_cut = (
        cq.Workplane("YZ")
        .workplane(offset=(BODY_L / 2.0) - PORT_TUNNEL)
        .center(0.0, PORT_Z)
        .circle(PORT_RADIUS)
        .extrude(PORT_TUNNEL + 0.004)
    )
    port_bevel = (
        cq.Workplane("YZ")
        .workplane(offset=(BODY_L / 2.0) - 0.003)
        .center(0.0, PORT_Z)
        .circle(PORT_RADIUS + 0.0018)
        .extrude(0.004)
    )
    return panel.cut(port_cut).cut(port_bevel)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_desktop_pencil_sharpener")

    body_color = model.material("body_color", rgba=(0.22, 0.24, 0.27, 1.0))
    drawer_color = model.material("drawer_color", rgba=(0.17, 0.18, 0.20, 1.0))
    metal = model.material("metal", rgba=(0.78, 0.79, 0.80, 1.0))
    arm_color = model.material("arm_color", rgba=(0.10, 0.10, 0.11, 1.0))

    body = model.part("body")
    eps = 0.0008
    drawer_open_half_w = (DRAWER_W / 2.0) + 0.002
    drawer_top = DRAWER_BOTTOM_Z + DRAWER_H + 0.001

    def add_body_box(name: str, size: tuple[float, float, float], xyz: tuple[float, float, float]) -> None:
        body.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material=body_color,
            name=name,
        )

    add_body_box(
        "left_wall",
        (BODY_L - FRONT_SKIN + eps, WALL + eps, BODY_H),
        (-FRONT_SKIN / 2.0 - (eps / 2.0), -BODY_W / 2.0 + (WALL + eps) / 2.0, BODY_H / 2.0),
    )
    add_body_box(
        "right_wall",
        (BODY_L - FRONT_SKIN + eps, WALL + eps, BODY_H),
        (-FRONT_SKIN / 2.0 - (eps / 2.0), BODY_W / 2.0 - (WALL + eps) / 2.0, BODY_H / 2.0),
    )
    add_body_box(
        "back_wall",
        (WALL + eps, BODY_W, BODY_H),
        (-BODY_L / 2.0 + (WALL + eps) / 2.0, 0.0, BODY_H / 2.0),
    )
    add_body_box(
        "top_shell",
        (BODY_L - WALL + eps, BODY_W, WALL + eps),
        (WALL / 2.0 - (eps / 2.0), 0.0, BODY_H - (WALL + eps) / 2.0),
    )
    rear_floor_len = DRAWER_CLOSED_X - (DRAWER_L / 2.0) - 0.004 - (-BODY_L / 2.0 + WALL - eps / 2.0)
    add_body_box(
        "rear_floor",
        (rear_floor_len, BODY_W - 2.0 * WALL + eps, BOTTOM),
        (
            (-BODY_L / 2.0 + WALL - eps / 2.0) + (rear_floor_len / 2.0),
            0.0,
            BOTTOM / 2.0,
        ),
    )
    add_body_box(
        "front_sill",
        (FRONT_SKIN, BODY_W, DRAWER_BOTTOM_Z + eps),
        (BODY_L / 2.0 - FRONT_SKIN / 2.0, 0.0, (DRAWER_BOTTOM_Z + eps) / 2.0),
    )
    jamb_h = drawer_top - DRAWER_BOTTOM_Z + eps
    jamb_y = (BODY_W / 2.0) - drawer_open_half_w
    add_body_box(
        "jamb_0",
        (FRONT_SKIN, jamb_y, jamb_h),
        (
            BODY_L / 2.0 - FRONT_SKIN / 2.0,
            -drawer_open_half_w - jamb_y / 2.0,
            DRAWER_BOTTOM_Z + jamb_h / 2.0 - eps / 2.0,
        ),
    )
    add_body_box(
        "jamb_1",
        (FRONT_SKIN, jamb_y, jamb_h),
        (
            BODY_L / 2.0 - FRONT_SKIN / 2.0,
            drawer_open_half_w + jamb_y / 2.0,
            DRAWER_BOTTOM_Z + jamb_h / 2.0 - eps / 2.0,
        ),
    )
    body.visual(
        mesh_from_cadquery(_front_panel_shape(), "sharpener_front_panel"),
        material=body_color,
        name="front_panel",
    )

    drawer = model.part("drawer")
    drawer.visual(
        mesh_from_cadquery(_drawer_shape(), "sharpener_drawer"),
        material=drawer_color,
        name="bin",
    )

    crank_hub = model.part("crank_hub")
    crank_hub.visual(
        mesh_from_cadquery(_hub_shape(), "sharpener_crank_hub"),
        material=metal,
        name="hub",
    )

    crank_arm = model.part("crank_arm")
    crank_arm.visual(
        mesh_from_cadquery(_arm_shape(), "sharpener_crank_arm"),
        material=arm_color,
        name="arm",
    )

    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(DRAWER_CLOSED_X, 0.0, DRAWER_BOTTOM_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=DRAWER_TRAVEL,
            effort=12.0,
            velocity=0.18,
        ),
    )
    model.articulation(
        "hub_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=crank_hub,
        origin=Origin(xyz=(HUB_X, BODY_W / 2.0, HUB_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=8.0,
        ),
    )
    model.articulation(
        "arm_fold",
        ArticulationType.REVOLUTE,
        parent=crank_hub,
        child=crank_arm,
        origin=Origin(xyz=(HUB_HINGE_X, HUB_HINGE_Y, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=ARM_FOLD_LIMIT,
            effort=4.0,
            velocity=2.2,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    crank_hub = object_model.get_part("crank_hub")
    crank_arm = object_model.get_part("crank_arm")
    drawer_slide = object_model.get_articulation("drawer_slide")
    hub_spin = object_model.get_articulation("hub_spin")
    arm_fold = object_model.get_articulation("arm_fold")

    ctx.allow_overlap(
        crank_hub,
        crank_arm,
        elem_a="hub",
        elem_b="arm",
        reason="The folding crank is represented with simplified hinge knuckles that intentionally interpenetrate at the fold pin.",
    )

    ctx.expect_within(
        drawer,
        body,
        axes="yz",
        margin=0.003,
        name="drawer stays centered in the sharpener opening",
    )
    ctx.expect_overlap(
        drawer,
        body,
        axes="x",
        min_overlap=0.040,
        name="closed drawer remains substantially inserted",
    )

    rest_drawer_pos = ctx.part_world_position(drawer)
    with ctx.pose({drawer_slide: DRAWER_TRAVEL}):
        ctx.expect_within(
            drawer,
            body,
            axes="yz",
            margin=0.003,
            name="extended drawer stays aligned on the runners",
        )
        ctx.expect_overlap(
            drawer,
            body,
            axes="x",
            min_overlap=0.030,
            name="extended drawer still retains insertion",
        )
        extended_drawer_pos = ctx.part_world_position(drawer)

    ctx.check(
        "drawer extends out the front",
        rest_drawer_pos is not None
        and extended_drawer_pos is not None
        and extended_drawer_pos[0] > rest_drawer_pos[0] + 0.03,
        details=f"rest={rest_drawer_pos}, extended={extended_drawer_pos}",
    )

    rest_arm_aabb = ctx.part_world_aabb(crank_arm)
    with ctx.pose({arm_fold: ARM_FOLD_LIMIT}):
        folded_arm_aabb = ctx.part_world_aabb(crank_arm)
    ctx.check(
        "crank arm folds downward",
        rest_arm_aabb is not None
        and folded_arm_aabb is not None
        and folded_arm_aabb[0][2] < rest_arm_aabb[0][2] - 0.02
        and folded_arm_aabb[1][0] < rest_arm_aabb[1][0] - 0.015,
        details=f"rest={rest_arm_aabb}, folded={folded_arm_aabb}",
    )

    with ctx.pose({hub_spin: math.pi / 2.0}):
        spun_arm_aabb = ctx.part_world_aabb(crank_arm)
    ctx.check(
        "hub rotates the crank assembly around the side shaft",
        rest_arm_aabb is not None
        and spun_arm_aabb is not None
        and spun_arm_aabb[1][2] < rest_arm_aabb[1][2] - 0.03,
        details=f"rest={rest_arm_aabb}, spun={spun_arm_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
