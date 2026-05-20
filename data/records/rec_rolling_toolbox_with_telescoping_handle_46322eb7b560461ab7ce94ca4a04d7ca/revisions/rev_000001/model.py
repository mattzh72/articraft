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


BODY_D = 0.36
BODY_W = 0.58
BODY_H = 0.58
BODY_WALL = 0.006
BODY_CORNER_R = 0.022

LID_D = 0.378
LID_W = 0.596
LID_H = 0.064
LID_TOP_SKIN = 0.010

DRAWER_FACE_W = 0.468
DRAWER_FACE_H = 0.122
DRAWER_FACE_T = 0.018
DRAWER_BODY_D = 0.175
DRAWER_TRAY_W = 0.450
DRAWER_SIDE_T = 0.008
DRAWER_BOTTOM_T = 0.008
DRAWER_SIDE_H = 0.100
DRAWER_BOTTOM_Z = 0.056
DRAWER_TRAVEL = 0.120

DRAWER_OPEN_W = 0.476
DRAWER_OPEN_H = 0.128
DRAWER_OPEN_D = 0.205
DRAWER_GUIDE_L = 0.175
DRAWER_GUIDE_T = 0.006
DRAWER_GUIDE_H = 0.014

MAIN_CAVITY_START_Z = 0.212

CHANNEL_OUTER_D = 0.024
CHANNEL_OUTER_W = 0.030
CHANNEL_WALL = 0.003
CHANNEL_H = 0.300
CHANNEL_X = -BODY_D / 2.0 - CHANNEL_OUTER_D / 2.0
CHANNEL_Y = 0.198
HANDLE_TRAVEL = 0.180

HANDLE_RAIL_D = 0.016
HANDLE_RAIL_W = 0.020
HANDLE_RAIL_L = 0.560
HANDLE_SPAN = CHANNEL_Y * 2.0

WHEEL_R = 0.080
WHEEL_T = 0.034
WHEEL_X = -BODY_D / 2.0 + 0.035
WHEEL_Y = BODY_W / 2.0 + WHEEL_T / 2.0
WHEEL_Z = 0.085


def _body_shell_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").box(BODY_D, BODY_W, BODY_H).translate((0.0, 0.0, BODY_H / 2.0))
    outer = outer.edges("|Z").fillet(BODY_CORNER_R)

    main_cavity_h = BODY_H - MAIN_CAVITY_START_Z + 0.020
    main_cavity = (
        cq.Workplane("XY")
        .box(BODY_D - 2.0 * BODY_WALL, BODY_W - 2.0 * BODY_WALL, main_cavity_h)
        .translate((0.0, 0.0, MAIN_CAVITY_START_Z + main_cavity_h / 2.0))
    )

    drawer_bay = (
        cq.Workplane("XY")
        .box(DRAWER_OPEN_D, DRAWER_OPEN_W, DRAWER_OPEN_H)
        .translate(
            (
                BODY_D / 2.0 - DRAWER_OPEN_D / 2.0 + 0.002,
                0.0,
                DRAWER_BOTTOM_Z + DRAWER_OPEN_H / 2.0,
            )
        )
    )

    shape = outer.cut(main_cavity).cut(drawer_bay)
    return shape


def _channel_sleeve_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").box(CHANNEL_OUTER_D, CHANNEL_OUTER_W, CHANNEL_H)
    inner = cq.Workplane("XY").box(
        CHANNEL_OUTER_D - 2.0 * CHANNEL_WALL,
        CHANNEL_OUTER_W - 2.0 * CHANNEL_WALL,
        CHANNEL_H + 0.004,
    )
    return outer.cut(inner)


def _lid_shell_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(LID_D, LID_W, LID_H)
        .translate((LID_D / 2.0, 0.0, LID_H / 2.0))
        .edges("|Z")
        .fillet(0.016)
    )
    inner = (
        cq.Workplane("XY")
        .box(BODY_D + 0.010, BODY_W + 0.010, LID_H - LID_TOP_SKIN + 0.002)
        .translate(((BODY_D + 0.010) / 2.0 + 0.004, 0.0, (LID_H - LID_TOP_SKIN + 0.002) / 2.0))
    )
    return outer.cut(inner)


def _drawer_face_shape() -> cq.Workplane:
    face = (
        cq.Workplane("XY")
        .box(DRAWER_FACE_T, DRAWER_FACE_W, DRAWER_FACE_H)
        .translate((-DRAWER_FACE_T / 2.0, 0.0, DRAWER_FACE_H / 2.0))
        .edges("|Z")
        .fillet(0.005)
    )
    finger_pull = (
        cq.Workplane("XZ")
        .center(-DRAWER_FACE_T * 0.45, DRAWER_FACE_H - 0.032)
        .circle(0.016)
        .extrude(0.220, both=True)
    )
    return face.cut(finger_pull)


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(lower, upper))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_rolling_tool_chest")

    shell = model.material("shell_charcoal", rgba=(0.15, 0.16, 0.17, 1.0))
    lid_mat = model.material("lid_graphite", rgba=(0.18, 0.19, 0.20, 1.0))
    drawer_mat = model.material("drawer_dark", rgba=(0.19, 0.20, 0.21, 1.0))
    handle_mat = model.material("handle_black", rgba=(0.08, 0.08, 0.09, 1.0))
    runner_mat = model.material("runner_gray", rgba=(0.42, 0.44, 0.47, 1.0))
    wheel_mat = model.material("wheel_rubber", rgba=(0.09, 0.09, 0.10, 1.0))
    hub_mat = model.material("hub_gray", rgba=(0.45, 0.47, 0.50, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_shape(), "tool_chest_body_shell"),
        material=shell,
        name="shell",
    )

    for z_pos in (0.032, 0.245, 0.365):
        body.visual(
            Box((0.008, BODY_W * 0.80, 0.018)),
            origin=Origin(xyz=(BODY_D / 2.0 + 0.004, 0.0, z_pos)),
            material=shell,
            name=f"front_rib_{int(round(z_pos * 1000.0))}",
        )
    for z_pos in (0.165, 0.305, 0.445):
        for y_sign, side_name in ((1.0, "left"), (-1.0, "right")):
            body.visual(
                Box((BODY_D * 0.58, 0.008, 0.018)),
                origin=Origin(
                    xyz=(
                        0.010,
                        y_sign * (BODY_W / 2.0 + 0.004),
                        z_pos,
                    )
                ),
                material=shell,
                name=f"{side_name}_rib_{int(round(z_pos * 1000.0))}",
            )

    body.visual(
        mesh_from_cadquery(_channel_sleeve_shape(), "tool_chest_left_channel"),
        origin=Origin(xyz=(CHANNEL_X, CHANNEL_Y, BODY_H - CHANNEL_H / 2.0)),
        material=runner_mat,
        name="left_channel",
    )
    body.visual(
        mesh_from_cadquery(_channel_sleeve_shape(), "tool_chest_right_channel"),
        origin=Origin(xyz=(CHANNEL_X, -CHANNEL_Y, BODY_H - CHANNEL_H / 2.0)),
        material=runner_mat,
        name="right_channel",
    )

    guide_center_x = BODY_D / 2.0 - 0.090
    guide_center_y = DRAWER_OPEN_W / 2.0 - DRAWER_GUIDE_T / 2.0
    guide_center_z = DRAWER_BOTTOM_Z + 0.020
    body.visual(
        Box((DRAWER_GUIDE_L, DRAWER_GUIDE_T, DRAWER_GUIDE_H)),
        origin=Origin(xyz=(guide_center_x, guide_center_y, guide_center_z)),
        material=runner_mat,
        name="left_guide",
    )
    body.visual(
        Box((DRAWER_GUIDE_L, DRAWER_GUIDE_T, DRAWER_GUIDE_H)),
        origin=Origin(xyz=(guide_center_x, -guide_center_y, guide_center_z)),
        material=runner_mat,
        name="right_guide",
    )

    for y_sign, side_name in ((1.0, "left"), (-1.0, "right")):
        body.visual(
            Box((0.050, 0.020, 0.094)),
            origin=Origin(
                xyz=(
                    -BODY_D / 2.0 + 0.025,
                    y_sign * (BODY_W / 2.0 - 0.010),
                    0.095,
                )
            ),
            material=shell,
            name=f"{side_name}_wheel_mount",
        )
    for y_sign, side_name in ((1.0, "left"), (-1.0, "right")):
        body.visual(
            Box((0.042, 0.056, 0.020)),
            origin=Origin(
                xyz=(
                    BODY_D / 2.0 - 0.036,
                    y_sign * 0.180,
                    0.010,
                )
            ),
            material=shell,
            name=f"{side_name}_foot",
        )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shell_shape(), "tool_chest_lid"),
        material=lid_mat,
        name="lid_shell",
    )
    for y_pos in (-0.165, 0.0, 0.165):
        lid.visual(
            Box((LID_D * 0.62, 0.020, 0.006)),
            origin=Origin(xyz=(LID_D * 0.46, y_pos, LID_H + 0.003)),
            material=lid_mat,
            name=f"lid_rib_{int(round((y_pos + 0.200) * 1000.0))}",
        )

    drawer = model.part("drawer")
    drawer.visual(
        mesh_from_cadquery(_drawer_face_shape(), "tool_chest_drawer_face"),
        material=drawer_mat,
        name="drawer_face",
    )
    tray_center_x = -(DRAWER_FACE_T + DRAWER_BODY_D / 2.0)
    drawer.visual(
        Box((DRAWER_BODY_D, DRAWER_TRAY_W, DRAWER_BOTTOM_T)),
        origin=Origin(xyz=(tray_center_x, 0.0, DRAWER_BOTTOM_T / 2.0)),
        material=drawer_mat,
        name="drawer_floor",
    )
    drawer.visual(
        Box((DRAWER_BODY_D, DRAWER_SIDE_T, DRAWER_SIDE_H)),
        origin=Origin(
            xyz=(
                tray_center_x,
                DRAWER_TRAY_W / 2.0 - DRAWER_SIDE_T / 2.0,
                DRAWER_SIDE_H / 2.0,
            )
        ),
        material=drawer_mat,
        name="left_wall",
    )
    drawer.visual(
        Box((DRAWER_BODY_D, DRAWER_SIDE_T, DRAWER_SIDE_H)),
        origin=Origin(
            xyz=(
                tray_center_x,
                -DRAWER_TRAY_W / 2.0 + DRAWER_SIDE_T / 2.0,
                DRAWER_SIDE_H / 2.0,
            )
        ),
        material=drawer_mat,
        name="right_wall",
    )
    drawer.visual(
        Box((DRAWER_SIDE_T, DRAWER_TRAY_W, DRAWER_SIDE_H)),
        origin=Origin(
            xyz=(
                -(DRAWER_FACE_T + DRAWER_BODY_D) + DRAWER_SIDE_T / 2.0,
                0.0,
                DRAWER_SIDE_H / 2.0,
            )
        ),
        material=drawer_mat,
        name="rear_wall",
    )
    drawer.visual(
        Box((DRAWER_GUIDE_L, 0.007, 0.012)),
        origin=Origin(
            xyz=(
                -0.090,
                DRAWER_TRAY_W / 2.0 + 0.0035,
                0.020,
            )
        ),
        material=runner_mat,
        name="left_slide",
    )
    drawer.visual(
        Box((DRAWER_GUIDE_L, 0.007, 0.012)),
        origin=Origin(
            xyz=(
                -0.090,
                -(DRAWER_TRAY_W / 2.0 + 0.0035),
                0.020,
            )
        ),
        material=runner_mat,
        name="right_slide",
    )

    handle = model.part("handle")
    handle.visual(
        Box((HANDLE_RAIL_D, HANDLE_RAIL_W, HANDLE_RAIL_L)),
        origin=Origin(xyz=(0.0, HANDLE_SPAN / 2.0, 0.0)),
        material=handle_mat,
        name="left_rail",
    )
    handle.visual(
        Box((HANDLE_RAIL_D, HANDLE_RAIL_W, HANDLE_RAIL_L)),
        origin=Origin(xyz=(0.0, -HANDLE_SPAN / 2.0, 0.0)),
        material=handle_mat,
        name="right_rail",
    )
    handle.visual(
        Box((0.028, HANDLE_SPAN + 0.060, 0.028)),
        origin=Origin(xyz=(-0.004, 0.0, HANDLE_RAIL_L / 2.0 - 0.014)),
        material=handle_mat,
        name="grip",
    )
    handle.visual(
        Box((CHANNEL_OUTER_D + 0.010, CHANNEL_OUTER_W + 0.010, 0.010)),
        origin=Origin(xyz=(-0.010, HANDLE_SPAN / 2.0, 0.005)),
        material=handle_mat,
        name="left_stop",
    )
    handle.visual(
        Box((CHANNEL_OUTER_D + 0.010, CHANNEL_OUTER_W + 0.010, 0.010)),
        origin=Origin(xyz=(-0.010, -HANDLE_SPAN / 2.0, 0.005)),
        material=handle_mat,
        name="right_stop",
    )

    for side_name, y_pos in (("left", WHEEL_Y), ("right", -WHEEL_Y)):
        wheel = model.part(f"{side_name}_wheel")
        wheel.visual(
            Cylinder(radius=WHEEL_R, length=WHEEL_T),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=wheel_mat,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.050, length=WHEEL_T * 0.72),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hub_mat,
            name="hub",
        )
        wheel.visual(
            Cylinder(radius=0.026, length=0.010),
            origin=Origin(
                xyz=(0.0, -0.012 if side_name == "left" else 0.012, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=hub_mat,
            name="cap",
        )
        model.articulation(
            f"{side_name}_wheel_spin",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel,
            origin=Origin(xyz=(WHEEL_X, y_pos, WHEEL_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=20.0, velocity=20.0),
        )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-BODY_D / 2.0, 0.0, BODY_H)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=0.0, upper=1.35),
    )
    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(BODY_D / 2.0 + DRAWER_FACE_T, 0.0, 0.060)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.22, lower=0.0, upper=DRAWER_TRAVEL),
    )
    model.articulation(
        "handle_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=handle,
        origin=Origin(xyz=(CHANNEL_X, 0.0, BODY_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.20, lower=0.0, upper=HANDLE_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    drawer = object_model.get_part("drawer")
    handle = object_model.get_part("handle")

    lid_hinge = object_model.get_articulation("lid_hinge")
    drawer_slide = object_model.get_articulation("drawer_slide")
    handle_slide = object_model.get_articulation("handle_slide")

    lid_limits = lid_hinge.motion_limits
    drawer_limits = drawer_slide.motion_limits
    handle_limits = handle_slide.motion_limits

    with ctx.pose({lid_hinge: 0.0, drawer_slide: 0.0, handle_slide: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="shell",
            max_gap=0.008,
            max_penetration=0.0,
            name="lid rests on the chest rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_shell",
            elem_b="shell",
            min_overlap=0.300,
            name="lid covers the top compartment footprint",
        )
        ctx.expect_overlap(
            drawer,
            body,
            axes="x",
            elem_a="left_slide",
            elem_b="left_guide",
            min_overlap=0.100,
            name="drawer keeps substantial runner engagement when closed",
        )
        ctx.expect_within(
            handle,
            body,
            axes="xy",
            inner_elem="left_rail",
            outer_elem="left_channel",
            margin=0.001,
            name="stowed handle rail stays centered in the left rear channel",
        )
        ctx.expect_overlap(
            handle,
            body,
            axes="z",
            elem_a="left_rail",
            elem_b="left_channel",
            min_overlap=0.240,
            name="stowed handle remains deeply inserted in the rear channel",
        )

        closed_drawer_face = ctx.part_element_world_aabb(drawer, elem="drawer_face")
        closed_handle_grip = ctx.part_element_world_aabb(handle, elem="grip")
        closed_lid_shell = ctx.part_element_world_aabb(lid, elem="lid_shell")

    if drawer_limits is not None and drawer_limits.upper is not None:
        with ctx.pose({drawer_slide: drawer_limits.upper}):
            ctx.expect_overlap(
                drawer,
                body,
                axes="x",
                elem_a="left_slide",
                elem_b="left_guide",
                min_overlap=0.035,
                name="drawer runner stays engaged when extended",
            )
            extended_drawer_face = ctx.part_element_world_aabb(drawer, elem="drawer_face")

        closed_drawer_center = _aabb_center(closed_drawer_face)
        extended_drawer_center = _aabb_center(extended_drawer_face)
        ctx.check(
            "drawer extends forward",
            closed_drawer_center is not None
            and extended_drawer_center is not None
            and extended_drawer_center[0] > closed_drawer_center[0] + 0.100,
            details=f"closed={closed_drawer_center}, extended={extended_drawer_center}",
        )
        ctx.check(
            "drawer face stays aligned within the lower shell",
            closed_drawer_center is not None
            and extended_drawer_center is not None
            and abs(extended_drawer_center[1] - closed_drawer_center[1]) <= 0.002
            and abs(extended_drawer_center[2] - closed_drawer_center[2]) <= 0.002,
            details=f"closed={closed_drawer_center}, extended={extended_drawer_center}",
        )

    if handle_limits is not None and handle_limits.upper is not None:
        with ctx.pose({handle_slide: handle_limits.upper}):
            ctx.expect_within(
                handle,
                body,
                axes="xy",
                inner_elem="left_rail",
                outer_elem="left_channel",
                margin=0.001,
                name="extended handle rail stays centered in the left rear channel",
            )
            ctx.expect_overlap(
                handle,
                body,
                axes="z",
                elem_a="left_rail",
                elem_b="left_channel",
                min_overlap=0.090,
                name="extended handle retains insertion in the rear channel",
            )
            extended_handle_grip = ctx.part_element_world_aabb(handle, elem="grip")

        closed_grip_center = _aabb_center(closed_handle_grip)
        extended_grip_center = _aabb_center(extended_handle_grip)
        ctx.check(
            "pull handle rises upward when extended",
            closed_grip_center is not None
            and extended_grip_center is not None
            and extended_grip_center[2] > closed_grip_center[2] + 0.150,
            details=f"closed={closed_grip_center}, extended={extended_grip_center}",
        )

    if lid_limits is not None and lid_limits.upper is not None:
        with ctx.pose({lid_hinge: lid_limits.upper}):
            opened_lid_shell = ctx.part_element_world_aabb(lid, elem="lid_shell")

        ctx.check(
            "lid opens upward from the rear hinge line",
            closed_lid_shell is not None
            and opened_lid_shell is not None
            and opened_lid_shell[1][2] > closed_lid_shell[1][2] + 0.180,
            details=f"closed={closed_lid_shell}, opened={opened_lid_shell}",
        )

    return ctx.report()


object_model = build_object_model()
