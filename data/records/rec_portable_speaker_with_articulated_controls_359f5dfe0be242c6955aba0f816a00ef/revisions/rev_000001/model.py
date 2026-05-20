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
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

BODY_W = 0.400
BODY_D = 0.190
BODY_H = 0.285

GRILLE_W = 0.304
GRILLE_H = 0.184
GRILLE_RECESS = 0.030

HANDLE_PIVOT_Z = 0.150
HANDLE_PIVOT_Y = 0.000
HANDLE_PIVOT_X = BODY_W * 0.5 + 0.030

CONTROL_FACE_Y = BODY_D * 0.5 + 0.010
CONTROL_Z = BODY_H - 0.041


def _build_body_shell() -> object:
    shell = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H, centered=(True, True, True))
        .translate((0.0, 0.0, BODY_H * 0.5))
        .edges("|Z")
        .fillet(0.022)
    )

    control_bridge = (
        cq.Workplane("XY")
        .box(0.262, 0.020, 0.040, centered=(True, True, True))
        .translate((0.0, BODY_D * 0.5, BODY_H - 0.025))
        .edges("|Z")
        .fillet(0.006)
    )
    shell = shell.union(control_bridge)

    grille_opening = (
        cq.Workplane("XY")
        .box(GRILLE_W, GRILLE_RECESS + 0.004, GRILLE_H, centered=(True, True, True))
        .translate((0.0, BODY_D * 0.5 - GRILLE_RECESS * 0.5 + 0.001, 0.134))
        .edges("|Y")
        .fillet(0.010)
    )
    shell = shell.cut(grille_opening)

    wheel_boss = (
        cq.Workplane("XY")
        .box(0.088, 0.014, 0.050, centered=(True, True, True))
        .translate((-0.055, BODY_D * 0.5 + 0.003, CONTROL_Z))
        .edges("|Z")
        .fillet(0.005)
    )
    shell = shell.union(wheel_boss)

    control_pad = (
        cq.Workplane("XY")
        .box(0.130, 0.012, 0.032, centered=(True, True, True))
        .translate((0.060, BODY_D * 0.5 + 0.004, CONTROL_Z - 0.001))
        .edges("|Z")
        .fillet(0.004)
    )
    shell = shell.union(control_pad)

    hinge_boss = (
        cq.Workplane("XY")
        .box(0.016, 0.028, 0.016, centered=(True, True, True))
        .edges("|Y")
        .fillet(0.005)
    )
    for side in (-1.0, 1.0):
        for y_pos in (-0.032, 0.032):
            shell = shell.union(
                hinge_boss.translate(
                    (side * (BODY_W * 0.5 + 0.006), y_pos, HANDLE_PIVOT_Z)
                )
            )

    return shell


def _build_bumpers() -> object:
    bumper = (
        cq.Workplane("XY")
        .box(0.050, 0.028, 0.052, centered=(True, True, True))
        .edges("|Z")
        .fillet(0.008)
    )

    bumpers = None
    for x_pos in (-0.176, 0.176):
        for y_pos in (-0.080, 0.080):
            for z_pos in (0.034, 0.252):
                placed = bumper.translate((x_pos, y_pos, z_pos))
                bumpers = placed if bumpers is None else bumpers.union(placed)
    return bumpers


def _build_handle() -> object:
    frame = (
        cq.Workplane("XY")
        .box(0.012, 0.114, 0.154, centered=(True, True, True))
        .translate((-0.010, 0.0, 0.055))
    )
    cutout = (
        cq.Workplane("XY")
        .box(0.020, 0.072, 0.104, centered=(True, True, True))
        .translate((-0.010, 0.0, 0.055))
    )
    frame = frame.cut(cutout)

    bridge = cq.Workplane("XY").box(0.012, 0.104, 0.020, centered=(True, True, True)).translate(
        (-0.002, 0.0, -0.010)
    )

    hinge_barrel = (
        cq.Workplane("XY")
        .box(0.016, 0.028, 0.016, centered=(True, True, True))
        .edges("|Y")
        .fillet(0.005)
    )

    handle = frame.union(bridge)
    handle = handle.union(hinge_barrel.translate((0.0, -0.032, 0.0)))
    handle = handle.union(hinge_barrel.translate((0.0, 0.032, 0.0)))
    return handle


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="jobsite_speaker")

    shell_mat = model.material("shell_mat", rgba=(0.18, 0.19, 0.20, 1.0))
    bumper_mat = model.material("bumper_mat", rgba=(0.10, 0.10, 0.10, 1.0))
    grille_mat = model.material("grille_mat", rgba=(0.27, 0.29, 0.31, 1.0))
    handle_mat = model.material("handle_mat", rgba=(0.15, 0.15, 0.16, 1.0))
    wheel_mat = model.material("wheel_mat", rgba=(0.11, 0.11, 0.12, 1.0))
    control_mat = model.material("control_mat", rgba=(0.12, 0.13, 0.14, 1.0))
    button_mat = model.material("button_mat", rgba=(0.22, 0.23, 0.24, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shell(), "speaker_body_shell"),
        material=shell_mat,
        name="shell",
    )
    body.visual(
        mesh_from_cadquery(_build_bumpers(), "speaker_corner_bumpers"),
        material=bumper_mat,
        name="bumpers",
    )

    grille = model.part("grille")
    grille.visual(
        mesh_from_geometry(
            PerforatedPanelGeometry(
                (0.288, 0.168),
                0.004,
                hole_diameter=0.0046,
                pitch=(0.0084, 0.0084),
                frame=0.010,
                corner_radius=0.010,
                stagger=True,
                center=False,
            ),
            "speaker_grille",
        ),
        origin=Origin(xyz=(0.0, BODY_D * 0.5 - GRILLE_RECESS + 0.004, 0.050), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grille_mat,
        name="front_grille",
    )
    model.articulation(
        "body_to_grille",
        ArticulationType.FIXED,
        parent=body,
        child=grille,
    )

    handle_mesh = mesh_from_cadquery(_build_handle(), "speaker_side_handle")

    right_handle = model.part("right_handle")
    right_handle.visual(
        handle_mesh,
        material=handle_mat,
        name="handle",
    )
    model.articulation(
        "body_to_right_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_handle,
        origin=Origin(xyz=(HANDLE_PIVOT_X, HANDLE_PIVOT_Y, HANDLE_PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.5,
            lower=0.0,
            upper=1.32,
        ),
    )

    left_handle = model.part("left_handle")
    left_handle.visual(
        handle_mesh,
        origin=Origin(rpy=(0.0, 0.0, math.pi)),
        material=handle_mat,
        name="handle",
    )
    model.articulation(
        "body_to_left_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_handle,
        origin=Origin(xyz=(-HANDLE_PIVOT_X, HANDLE_PIVOT_Y, HANDLE_PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.5,
            lower=0.0,
            upper=1.32,
        ),
    )

    selector_wheel = model.part("selector_wheel")
    selector_wheel.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.064,
                0.018,
                body_style="cylindrical",
                edge_radius=0.002,
                grip=KnobGrip(style="knurled", count=28, depth=0.0012),
                center=False,
            ),
            "selector_wheel",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=wheel_mat,
        name="wheel",
    )
    model.articulation(
        "body_to_selector_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector_wheel,
        origin=Origin(xyz=(-0.055, CONTROL_FACE_Y, CONTROL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=8.0,
        ),
    )

    power_rocker = model.part("power_rocker")
    power_rocker.visual(
        Box((0.026, 0.004, 0.004)),
        origin=Origin(),
        material=control_mat,
        name="pivot",
    )
    power_rocker.visual(
        Box((0.024, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, 0.005, 0.004)),
        material=button_mat,
        name="cap",
    )
    model.articulation(
        "body_to_power_rocker",
        ArticulationType.REVOLUTE,
        parent=body,
        child=power_rocker,
        origin=Origin(xyz=(0.020, CONTROL_FACE_Y, CONTROL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=-0.28,
            upper=0.28,
        ),
    )

    for index, x_pos in enumerate((0.052, 0.082)):
        button = model.part(f"menu_button_{index}")
        button.visual(
            Cylinder(radius=0.008, length=0.005),
            origin=Origin(xyz=(0.0, 0.0025, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=button_mat,
            name="button",
        )
        model.articulation(
            f"body_to_menu_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x_pos, CONTROL_FACE_Y, CONTROL_Z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.05,
                lower=0.0,
                upper=0.003,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    right_handle = object_model.get_part("right_handle")
    left_handle = object_model.get_part("left_handle")
    selector_wheel = object_model.get_part("selector_wheel")
    power_rocker = object_model.get_part("power_rocker")
    menu_button_0 = object_model.get_part("menu_button_0")
    menu_button_1 = object_model.get_part("menu_button_1")

    right_joint = object_model.get_articulation("body_to_right_handle")
    left_joint = object_model.get_articulation("body_to_left_handle")
    rocker_joint = object_model.get_articulation("body_to_power_rocker")
    button_joint_0 = object_model.get_articulation("body_to_menu_button_0")
    button_joint_1 = object_model.get_articulation("body_to_menu_button_1")

    ctx.expect_origin_distance(
        selector_wheel,
        power_rocker,
        axes="x",
        min_dist=0.060,
        max_dist=0.090,
        name="selector wheel stays clearly separate from the power rocker",
    )
    ctx.expect_origin_distance(
        menu_button_0,
        power_rocker,
        axes="x",
        min_dist=0.020,
        max_dist=0.045,
        name="first menu button stays distinct beside the rocker",
    )
    ctx.expect_origin_distance(
        menu_button_0,
        menu_button_1,
        axes="x",
        min_dist=0.020,
        max_dist=0.040,
        name="menu buttons remain two separate controls",
    )
    ctx.expect_gap(
        selector_wheel,
        "body",
        axis="y",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem="wheel",
        negative_elem="shell",
        name="selector wheel mounts flush to the front control boss",
    )

    right_upper = right_joint.motion_limits.upper if right_joint.motion_limits is not None else None
    left_upper = left_joint.motion_limits.upper if left_joint.motion_limits is not None else None
    rocker_lower = rocker_joint.motion_limits.lower if rocker_joint.motion_limits is not None else None
    rocker_upper = rocker_joint.motion_limits.upper if rocker_joint.motion_limits is not None else None
    button_upper_0 = button_joint_0.motion_limits.upper if button_joint_0.motion_limits is not None else None
    button_upper_1 = button_joint_1.motion_limits.upper if button_joint_1.motion_limits is not None else None

    if right_upper is not None:
        right_rest = ctx.part_world_aabb(right_handle)
        with ctx.pose({right_joint: right_upper}):
            right_open = ctx.part_world_aabb(right_handle)
        ctx.check(
            "right handle folds outward from the side",
            right_rest is not None
            and right_open is not None
            and right_open[1][0] > right_rest[1][0] + 0.080,
            details=f"rest={right_rest}, open={right_open}",
        )

    if left_upper is not None:
        left_rest = ctx.part_world_aabb(left_handle)
        with ctx.pose({left_joint: left_upper}):
            left_open = ctx.part_world_aabb(left_handle)
        ctx.check(
            "left handle folds outward from the side",
            left_rest is not None
            and left_open is not None
            and left_open[0][0] < left_rest[0][0] - 0.080,
            details=f"rest={left_rest}, open={left_open}",
        )

    if rocker_lower is not None and rocker_upper is not None:
        with ctx.pose({rocker_joint: rocker_lower}):
            rocker_low = ctx.part_world_aabb(power_rocker)
        with ctx.pose({rocker_joint: rocker_upper}):
            rocker_high = ctx.part_world_aabb(power_rocker)
        ctx.check(
            "power rocker actually rocks on its pivot",
            rocker_low is not None
            and rocker_high is not None
            and abs(rocker_high[1][1] - rocker_low[1][1]) > 0.0015,
            details=f"low={rocker_low}, high={rocker_high}",
        )

    if button_upper_0 is not None and button_upper_1 is not None:
        rest_0 = ctx.part_world_position(menu_button_0)
        rest_1 = ctx.part_world_position(menu_button_1)
        with ctx.pose({button_joint_0: button_upper_0}):
            pressed_0 = ctx.part_world_position(menu_button_0)
            idle_1 = ctx.part_world_position(menu_button_1)
        with ctx.pose({button_joint_1: button_upper_1}):
            pressed_1 = ctx.part_world_position(menu_button_1)
            idle_0 = ctx.part_world_position(menu_button_0)

        ctx.check(
            "menu button 0 depresses independently",
            rest_0 is not None
            and rest_1 is not None
            and pressed_0 is not None
            and idle_1 is not None
            and pressed_0[1] < rest_0[1] - 0.0025
            and abs(idle_1[1] - rest_1[1]) < 1e-6,
            details=f"rest_0={rest_0}, pressed_0={pressed_0}, rest_1={rest_1}, idle_1={idle_1}",
        )
        ctx.check(
            "menu button 1 depresses independently",
            rest_0 is not None
            and rest_1 is not None
            and pressed_1 is not None
            and idle_0 is not None
            and pressed_1[1] < rest_1[1] - 0.0025
            and abs(idle_0[1] - rest_0[1]) < 1e-6,
            details=f"rest_1={rest_1}, pressed_1={pressed_1}, rest_0={rest_0}, idle_0={idle_0}",
        )

    return ctx.report()


object_model = build_object_model()
