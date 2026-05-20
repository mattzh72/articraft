from __future__ import annotations

from math import pi

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


BODY_W = 0.38
BODY_D = 0.27
BODY_H = 0.28
BODY_CORNER_R = 0.028
BODY_TOP_R = 0.014

OPENING_W = 0.302
OPENING_D = 0.182
OPENING_Y = -0.010
POCKET_DEPTH = 0.055

GUIDE_W = 0.012
GUIDE_H = 0.010
GUIDE_L = 0.146
GUIDE_X = 0.118
GUIDE_Z = BODY_H - POCKET_DEPTH + GUIDE_H / 2.0 - 0.0005

TRAY_W = 0.290
TRAY_D = 0.172
TRAY_H = 0.034
TRAY_WALL = 0.003
TRAY_FLOOR = 0.003
TRAY_CLOSED_Y = OPENING_Y - 0.004
RUNNER_W = 0.010
RUNNER_H = 0.006
RUNNER_L = 0.154
RUNNER_X = GUIDE_X
TRAY_TRAVEL = 0.085

COVER_W = 0.328
COVER_D = 0.204
COVER_H = 0.031
COVER_WALL = 0.003
COVER_TOP = 0.003
COVER_CLOSED_Z = BODY_H

HANDLE_POST_W = 0.014
HANDLE_POST_D = 0.010
HANDLE_SPAN = 0.132
HANDLE_JOINT_Y = BODY_D / 2.0 - 0.005
HANDLE_JOINT_Z = 0.220
HANDLE_TRAVEL = 0.160

WHEEL_R = 0.050
WHEEL_W = 0.028
WHEEL_Y = 0.090
WHEEL_Z = 0.050

LATCH_X = BODY_W / 2.0 + 0.005
LATCH_Y = 0.020
LATCH_Z = BODY_H - 0.004


def _mesh(shape, name: str):
    return mesh_from_cadquery(shape, name)


def _make_body_shell() -> cq.Workplane:
    shell = cq.Workplane("XY").box(BODY_W, BODY_D, BODY_H, centered=(True, True, False))
    shell = shell.edges("|Z").fillet(BODY_CORNER_R)

    pocket = (
        cq.Workplane("XY")
        .box(OPENING_W, OPENING_D, POCKET_DEPTH + 0.001, centered=(True, True, False))
        .translate((0.0, OPENING_Y, BODY_H - POCKET_DEPTH))
    )
    shell = shell.cut(pocket)

    finger_slot = (
        cq.Workplane("XY")
        .box(0.082, 0.024, 0.016, centered=(True, True, False))
        .translate((0.0, OPENING_Y - OPENING_D / 2.0 - 0.008, BODY_H - 0.016))
    )
    shell = shell.cut(finger_slot)

    rear_recess = (
        cq.Workplane("XY")
        .box(0.214, 0.018, 0.182, centered=(True, False, False))
        .translate((0.0, BODY_D / 2.0 - 0.018, 0.098))
    )
    shell = shell.cut(rear_recess)

    left_channel = (
        cq.Workplane("XY")
        .box(0.014, 0.014, 0.185, centered=(True, False, False))
        .translate((-HANDLE_SPAN / 2.0, BODY_D / 2.0 - 0.014, 0.095))
    )
    right_channel = left_channel.translate((HANDLE_SPAN, 0.0, 0.0))
    shell = shell.cut(left_channel).cut(right_channel)

    return shell


def _make_cover_shell() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(COVER_W, COVER_D, COVER_H, centered=(True, True, False))
        .translate((0.0, -COVER_D / 2.0, 0.0))
    )

    inner = (
        cq.Workplane("XY")
        .box(
            COVER_W - 2.0 * COVER_WALL,
            COVER_D - 2.0 * COVER_WALL,
            COVER_H + 0.004,
            centered=(True, True, False),
        )
        .translate((0.0, -COVER_D / 2.0, COVER_TOP))
    )
    lip = (
        cq.Workplane("XY")
        .box(0.090, 0.010, 0.010, centered=(True, True, False))
        .translate((0.0, -COVER_D + 0.008, 0.010))
    )
    return outer.cut(inner).union(lip)


def _make_tray_shell() -> cq.Workplane:
    outer = cq.Workplane("XY").box(TRAY_W, TRAY_D, TRAY_H, centered=(True, True, False)).translate(
        (0.0, 0.0, RUNNER_H)
    )
    inner = (
        cq.Workplane("XY")
        .box(
            TRAY_W - 2.0 * TRAY_WALL,
            TRAY_D - 2.0 * TRAY_WALL,
            TRAY_H + 0.002,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, RUNNER_H + TRAY_FLOOR))
    )
    return outer.cut(inner)


def _make_handle_frame() -> cq.Workplane:
    left_post = cq.Workplane("XY").box(HANDLE_POST_W, HANDLE_POST_D, 0.280).translate(
        (-HANDLE_SPAN / 2.0, 0.0, 0.020)
    )
    right_post = left_post.translate((HANDLE_SPAN, 0.0, 0.0))
    grip = cq.Workplane("XY").box(0.164, 0.014, 0.016).translate((0.0, 0.0, 0.152))
    return left_post.union(right_post).union(grip)


def _make_latch() -> cq.Workplane:
    ring = cq.Workplane("YZ").circle(0.010).extrude(0.005, both=True)
    hole = cq.Workplane("YZ").circle(0.0056).extrude(0.006, both=True)
    arm = cq.Workplane("XY").box(0.010, 0.016, 0.036).translate((0.0, -0.002, 0.020))
    bridge = cq.Workplane("XY").box(0.010, 0.024, 0.012).translate((0.0, -0.006, 0.043))
    return ring.cut(hole).union(arm).union(bridge)


def _make_wheel() -> cq.Workplane:
    return cq.Workplane("YZ").circle(WHEEL_R).extrude(WHEEL_W / 2.0, both=True)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rolling_tool_case")

    body_plastic = model.material("body_plastic", rgba=(0.16, 0.17, 0.18, 1.0))
    trim_black = model.material("trim_black", rgba=(0.08, 0.08, 0.09, 1.0))
    latch_dark = model.material("latch_dark", rgba=(0.14, 0.14, 0.15, 1.0))
    tray_gray = model.material("tray_gray", rgba=(0.36, 0.38, 0.40, 1.0))
    handle_metal = model.material("handle_metal", rgba=(0.73, 0.75, 0.78, 1.0))
    wheel_black = model.material("wheel_black", rgba=(0.05, 0.05, 0.05, 1.0))
    cover_smoke = model.material("cover_smoke", rgba=(0.18, 0.24, 0.28, 0.58))
    axle_dark = model.material("axle_dark", rgba=(0.26, 0.27, 0.29, 1.0))

    body = model.part("case_body")
    body.visual(_mesh(_make_body_shell(), "rolling_tool_case_body"), material=body_plastic, name="shell")
    body.visual(
        Box((GUIDE_W, GUIDE_L, GUIDE_H)),
        origin=Origin(xyz=(-GUIDE_X, TRAY_CLOSED_Y, GUIDE_Z)),
        material=trim_black,
        name="left_guide",
    )
    body.visual(
        Box((GUIDE_W, GUIDE_L, GUIDE_H)),
        origin=Origin(xyz=(GUIDE_X, TRAY_CLOSED_Y, GUIDE_Z)),
        material=trim_black,
        name="right_guide",
    )
    body.visual(
        Box((0.004, 0.020, 0.180)),
        origin=Origin(xyz=(-(HANDLE_SPAN / 2.0 + HANDLE_POST_W / 2.0 + 0.002), HANDLE_JOINT_Y - 0.004, 0.190)),
        material=axle_dark,
        name="left_handle_guide",
    )
    body.visual(
        Box((0.004, 0.020, 0.180)),
        origin=Origin(xyz=((HANDLE_SPAN / 2.0 + HANDLE_POST_W / 2.0 + 0.002), HANDLE_JOINT_Y - 0.004, 0.190)),
        material=axle_dark,
        name="right_handle_guide",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.002),
        origin=Origin(xyz=(-0.190, WHEEL_Y, WHEEL_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material=axle_dark,
        name="left_axle_stub",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.002),
        origin=Origin(xyz=(0.190, WHEEL_Y, WHEEL_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material=axle_dark,
        name="right_axle_stub",
    )
    body.visual(
        Box((0.034, 0.024, 0.010)),
        origin=Origin(xyz=(-0.118, -(BODY_D / 2.0) + 0.028, 0.005)),
        material=trim_black,
        name="left_foot",
    )
    body.visual(
        Box((0.034, 0.024, 0.010)),
        origin=Origin(xyz=(0.118, -(BODY_D / 2.0) + 0.028, 0.005)),
        material=trim_black,
        name="right_foot",
    )

    cover = model.part("organizer_cover")
    cover.visual(
        _mesh(_make_cover_shell(), "rolling_tool_case_cover"),
        material=cover_smoke,
        name="cover_shell",
    )

    tray = model.part("organizer_tray")
    tray.visual(
        _mesh(_make_tray_shell(), "rolling_tool_case_tray"),
        material=tray_gray,
        name="tray_shell",
    )
    tray.visual(
        Box((0.004, TRAY_D - 0.016, TRAY_H - 0.010)),
        origin=Origin(xyz=(0.0, 0.0, RUNNER_H + (TRAY_H - 0.010) / 2.0)),
        material=trim_black,
        name="center_divider",
    )
    tray.visual(
        Box((TRAY_W - 0.022, 0.004, TRAY_H - 0.016)),
        origin=Origin(xyz=(0.0, 0.026, RUNNER_H + (TRAY_H - 0.016) / 2.0)),
        material=trim_black,
        name="cross_divider",
    )
    tray.visual(
        Box((RUNNER_W, RUNNER_L, RUNNER_H)),
        origin=Origin(xyz=(-RUNNER_X, 0.0, RUNNER_H / 2.0)),
        material=trim_black,
        name="left_runner",
    )
    tray.visual(
        Box((RUNNER_W, RUNNER_L, RUNNER_H)),
        origin=Origin(xyz=(RUNNER_X, 0.0, RUNNER_H / 2.0)),
        material=trim_black,
        name="right_runner",
    )

    handle = model.part("rear_handle")
    handle.visual(
        _mesh(_make_handle_frame(), "rolling_tool_case_handle"),
        material=handle_metal,
        name="handle_frame",
    )

    left_latch = model.part("left_latch")
    left_latch.visual(
        _mesh(_make_latch(), "rolling_tool_case_left_latch"),
        material=latch_dark,
        name="latch_lever",
    )

    right_latch = model.part("right_latch")
    right_latch.visual(
        _mesh(_make_latch(), "rolling_tool_case_right_latch"),
        material=latch_dark,
        name="latch_lever",
    )

    left_wheel = model.part("left_rear_wheel")
    left_wheel.visual(
        _mesh(_make_wheel(), "rolling_tool_case_left_wheel"),
        material=wheel_black,
        name="wheel_shell",
    )

    right_wheel = model.part("right_rear_wheel")
    right_wheel.visual(
        _mesh(_make_wheel(), "rolling_tool_case_right_wheel"),
        material=wheel_black,
        name="wheel_shell",
    )

    model.articulation(
        "body_to_organizer_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cover,
        origin=Origin(xyz=(0.0, OPENING_Y + COVER_D / 2.0, COVER_CLOSED_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.28),
    )
    model.articulation(
        "body_to_organizer_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(0.0, TRAY_CLOSED_Y, GUIDE_Z + GUIDE_H / 2.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.18, lower=0.0, upper=TRAY_TRAVEL),
    )
    model.articulation(
        "body_to_rear_handle",
        ArticulationType.PRISMATIC,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, HANDLE_JOINT_Y, HANDLE_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.20, lower=0.0, upper=HANDLE_TRAVEL),
    )
    model.articulation(
        "body_to_left_latch",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_latch,
        origin=Origin(xyz=(-LATCH_X, LATCH_Y, LATCH_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=3.0, lower=0.0, upper=1.05),
    )
    model.articulation(
        "body_to_right_latch",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_latch,
        origin=Origin(xyz=(LATCH_X, LATCH_Y, LATCH_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=3.0, lower=0.0, upper=1.05),
    )
    model.articulation(
        "body_to_left_rear_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=left_wheel,
        origin=Origin(xyz=(-((BODY_W / 2.0) + (WHEEL_W / 2.0) + 0.001), WHEEL_Y, WHEEL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=18.0),
    )
    model.articulation(
        "body_to_right_rear_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=right_wheel,
        origin=Origin(xyz=((BODY_W / 2.0) + (WHEEL_W / 2.0) + 0.001, WHEEL_Y, WHEEL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("case_body")
    cover = object_model.get_part("organizer_cover")
    tray = object_model.get_part("organizer_tray")
    handle = object_model.get_part("rear_handle")
    left_latch = object_model.get_part("left_latch")
    right_latch = object_model.get_part("right_latch")

    cover_joint = object_model.get_articulation("body_to_organizer_cover")
    tray_joint = object_model.get_articulation("body_to_organizer_tray")
    handle_joint = object_model.get_articulation("body_to_rear_handle")
    left_latch_joint = object_model.get_articulation("body_to_left_latch")

    ctx.expect_gap(
        cover,
        body,
        axis="z",
        positive_elem="cover_shell",
        negative_elem="shell",
        max_gap=0.003,
        max_penetration=0.0,
        name="organizer cover seats on the body rim",
    )
    ctx.expect_overlap(
        cover,
        body,
        axes="xy",
        elem_a="cover_shell",
        elem_b="shell",
        min_overlap=0.20,
        name="organizer cover spans the case opening",
    )
    ctx.expect_contact(
        tray,
        body,
        elem_a="left_runner",
        elem_b="left_guide",
        name="left tray runner rides on the guide",
    )
    ctx.expect_contact(
        tray,
        body,
        elem_a="right_runner",
        elem_b="right_guide",
        name="right tray runner rides on the guide",
    )
    ctx.expect_overlap(
        tray,
        body,
        axes="y",
        elem_a="left_runner",
        elem_b="left_guide",
        min_overlap=0.11,
        name="closed tray stays engaged on the left guide",
    )
    ctx.expect_overlap(
        left_latch,
        body,
        axes="yz",
        min_overlap=0.012,
        name="left latch bridges the body seam zone",
    )
    ctx.expect_overlap(
        left_latch,
        cover,
        axes="yz",
        min_overlap=0.012,
        name="left latch reaches the cover seam zone",
    )
    ctx.expect_overlap(
        right_latch,
        body,
        axes="yz",
        min_overlap=0.012,
        name="right latch bridges the body seam zone",
    )
    ctx.expect_overlap(
        right_latch,
        cover,
        axes="yz",
        min_overlap=0.012,
        name="right latch reaches the cover seam zone",
    )

    handle_rest = ctx.part_world_position(handle)
    tray_rest = ctx.part_world_position(tray)
    cover_rest_aabb = ctx.part_world_aabb(cover)
    latch_rest_aabb = ctx.part_world_aabb(left_latch)

    handle_upper = handle_joint.motion_limits.upper if handle_joint.motion_limits is not None else None
    if handle_upper is not None:
        with ctx.pose({handle_joint: handle_upper}):
            handle_extended = ctx.part_world_position(handle)
        ctx.check(
            "rear handle extends upward",
            handle_rest is not None
            and handle_extended is not None
            and handle_extended[2] > handle_rest[2] + 0.10,
            details=f"rest={handle_rest}, extended={handle_extended}",
        )

    cover_upper = cover_joint.motion_limits.upper if cover_joint.motion_limits is not None else None
    if cover_upper is not None:
        with ctx.pose({cover_joint: cover_upper}):
            cover_open_aabb = ctx.part_world_aabb(cover)
        ctx.check(
            "organizer cover opens upward",
            cover_rest_aabb is not None
            and cover_open_aabb is not None
            and cover_open_aabb[1][2] > cover_rest_aabb[1][2] + 0.08,
            details=f"closed={cover_rest_aabb}, open={cover_open_aabb}",
        )

    tray_upper = tray_joint.motion_limits.upper if tray_joint.motion_limits is not None else None
    if cover_upper is not None and tray_upper is not None:
        with ctx.pose({cover_joint: cover_upper, tray_joint: tray_upper}):
            tray_extended = ctx.part_world_position(tray)
            ctx.expect_contact(
                tray,
                body,
                elem_a="left_runner",
                elem_b="left_guide",
                name="extended tray still rides on the left guide",
            )
            ctx.expect_contact(
                tray,
                body,
                elem_a="right_runner",
                elem_b="right_guide",
                name="extended tray still rides on the right guide",
            )
            ctx.expect_overlap(
                tray,
                body,
                axes="y",
                elem_a="left_runner",
                elem_b="left_guide",
                min_overlap=0.05,
                name="extended tray retains insertion on the left guide",
            )
            ctx.expect_gap(
                cover,
                tray,
                axis="z",
                min_gap=0.001,
                name="open cover clears the extended tray",
            )
        ctx.check(
            "organizer tray slides toward the front",
            tray_rest is not None
            and tray_extended is not None
            and tray_extended[1] < tray_rest[1] - 0.05,
            details=f"rest={tray_rest}, extended={tray_extended}",
        )

    latch_upper = left_latch_joint.motion_limits.upper if left_latch_joint.motion_limits is not None else None
    if latch_upper is not None:
        with ctx.pose({left_latch_joint: latch_upper}):
            latch_open_aabb = ctx.part_world_aabb(left_latch)
        ctx.check(
            "left latch rotates rearward off the seam",
            latch_rest_aabb is not None
            and latch_open_aabb is not None
            and latch_open_aabb[1][1] > latch_rest_aabb[1][1] + 0.02,
            details=f"closed={latch_rest_aabb}, open={latch_open_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
