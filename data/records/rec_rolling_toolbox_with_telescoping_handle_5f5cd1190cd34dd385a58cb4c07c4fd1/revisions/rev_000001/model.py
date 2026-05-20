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


BODY_W = 0.74
BODY_D = 0.44
BODY_H = 0.38
BODY_CLEARANCE = 0.08
BODY_WALL = 0.010
BODY_CORNER = 0.030

LID_W = 0.758
LID_D = 0.456
LID_H = 0.094
LID_WALL = 0.006
LID_SEAM_GAP = 0.002

HANDLE_CHANNEL_X = 0.14
HANDLE_CHANNEL_Y = -0.236
HANDLE_CHANNEL_BOTTOM = BODY_CLEARANCE
HANDLE_CHANNEL_H = 0.35
HANDLE_CHANNEL_W = 0.038
HANDLE_CHANNEL_D = 0.026
HANDLE_LEG_W = 0.018
HANDLE_LEG_D = 0.014
HANDLE_BAR_LENGTH = 0.66
HANDLE_GRIP_W = 0.46
HANDLE_TRAVEL = 0.28

WHEEL_RADIUS = 0.095


def _mesh(shape, name: str):
    return mesh_from_cadquery(shape, name)


def _make_body_shell() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(BODY_CORNER)
        .edges(">Z")
        .fillet(0.014)
        .translate((0.0, 0.0, BODY_CLEARANCE))
    )
    inner = (
        cq.Workplane("XY")
        .box(BODY_W - 2.0 * BODY_WALL, BODY_D - 2.0 * BODY_WALL, BODY_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(max(BODY_CORNER - BODY_WALL, 0.012))
        .translate((0.0, 0.0, BODY_CLEARANCE + BODY_WALL))
    )
    shell = outer.cut(inner)

    notch_w = 0.14
    notch_d = 0.13
    notch_h = 0.16
    notch_x = BODY_W * 0.5 - notch_w * 0.5
    for sign in (-1.0, 1.0):
        notch = (
            cq.Workplane("XY")
            .box(notch_w, notch_d, notch_h, centered=(True, False, False))
            .translate((sign * notch_x, -BODY_D * 0.5, BODY_CLEARANCE))
        )
        shell = shell.cut(notch)

    for z_base in (0.115, 0.180, 0.245, 0.310):
        front_band = (
            cq.Workplane("XY")
            .box(BODY_W * 0.76, 0.008, 0.012, centered=(True, True, False))
            .translate((0.0, BODY_D * 0.5 + 0.004, z_base))
        )
        shell = shell.union(front_band)
        for sign in (-1.0, 1.0):
            side_band = (
                cq.Workplane("XY")
                .box(0.008, BODY_D * 0.62, 0.012, centered=(True, True, False))
                .translate((sign * (BODY_W * 0.5 + 0.004), 0.015, z_base))
            )
            shell = shell.union(side_band)

    top_lip = (
        cq.Workplane("XY")
        .box(BODY_W * 0.78, 0.012, 0.028, centered=(True, True, False))
        .translate((0.0, BODY_D * 0.5 + 0.002, BODY_CLEARANCE + BODY_H - 0.050))
    )
    shell = shell.union(top_lip)
    return shell


def _make_channel_shape(x_center: float) -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(HANDLE_CHANNEL_W, HANDLE_CHANNEL_D, HANDLE_CHANNEL_H, centered=(True, True, False))
        .translate((x_center, HANDLE_CHANNEL_Y, HANDLE_CHANNEL_BOTTOM))
    )
    inner = (
        cq.Workplane("XY")
        .box(HANDLE_CHANNEL_W - 0.008, HANDLE_CHANNEL_D - 0.008, HANDLE_CHANNEL_H + 0.004, centered=(True, True, False))
        .translate((x_center, HANDLE_CHANNEL_Y, HANDLE_CHANNEL_BOTTOM - 0.002))
    )
    collar = (
        cq.Workplane("XY")
        .box(HANDLE_CHANNEL_W + 0.008, HANDLE_CHANNEL_D + 0.004, 0.020, centered=(True, True, False))
        .translate((x_center, HANDLE_CHANNEL_Y, HANDLE_CHANNEL_BOTTOM))
    )
    return outer.cut(inner).union(collar)


def _make_lid_shell() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(LID_W, LID_D, LID_H, centered=(True, False, False))
        .edges("|Z")
        .fillet(0.026)
        .edges(">Z")
        .fillet(0.010)
    )
    inner = (
        cq.Workplane("XY")
        .box(LID_W - 2.0 * LID_WALL, LID_D - 2.0 * LID_WALL, LID_H, centered=(True, False, False))
        .edges("|Z")
        .fillet(0.018)
        .translate((0.0, LID_WALL, LID_WALL))
    )
    return outer.cut(inner)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_rolling_tool_chest")

    shell_plastic = model.material("shell_plastic", rgba=(0.17, 0.19, 0.21, 1.0))
    lid_plastic = model.material("lid_plastic", rgba=(0.21, 0.23, 0.25, 1.0))
    trim_black = model.material("trim_black", rgba=(0.07, 0.07, 0.08, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.34, 0.36, 0.39, 1.0))
    handle_steel = model.material("handle_steel", rgba=(0.78, 0.80, 0.83, 1.0))
    grip_black = model.material("grip_black", rgba=(0.11, 0.11, 0.12, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.06, 0.06, 0.06, 1.0))

    body = model.part("body")
    body.visual(_mesh(_make_body_shell(), "tool_chest_body_shell"), material=shell_plastic, name="body_shell")
    body.visual(
        Box((0.12, 0.11, BODY_CLEARANCE + 0.004)),
        origin=Origin(xyz=(-0.22, 0.15, BODY_CLEARANCE * 0.5 + 0.002)),
        material=trim_black,
        name="front_foot_0",
    )
    body.visual(
        Box((0.12, 0.11, BODY_CLEARANCE + 0.004)),
        origin=Origin(xyz=(0.22, 0.15, BODY_CLEARANCE * 0.5 + 0.002)),
        material=trim_black,
        name="front_foot_1",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.120),
        origin=Origin(
            xyz=(-0.310, -(BODY_D * 0.5 + 0.009), BODY_CLEARANCE + BODY_H - 0.012),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=dark_steel,
        name="hinge_barrel_0",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.120),
        origin=Origin(
            xyz=(0.310, -(BODY_D * 0.5 + 0.009), BODY_CLEARANCE + BODY_H - 0.012),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=dark_steel,
        name="hinge_barrel_1",
    )
    body.visual(
        Box((0.080, 0.030, 0.130)),
        origin=Origin(xyz=(-0.205, -0.206, 0.145)),
        material=trim_black,
        name="wheel_bracket_0",
    )
    body.visual(
        Box((0.080, 0.030, 0.130)),
        origin=Origin(xyz=(0.205, -0.206, 0.145)),
        material=trim_black,
        name="wheel_bracket_1",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.028),
        origin=Origin(xyz=(-0.376, -0.255, WHEEL_RADIUS), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="axle_stub_0",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.028),
        origin=Origin(xyz=(0.376, -0.255, WHEEL_RADIUS), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="axle_stub_1",
    )

    lid = model.part("lid")
    lid.visual(_mesh(_make_lid_shell(), "tool_chest_lid_shell"), material=lid_plastic, name="lid_shell")

    rear_guide_0 = model.part("rear_guide_0")
    rear_guide_0.visual(
        Box((HANDLE_CHANNEL_W, 0.006, HANDLE_CHANNEL_H)),
        origin=Origin(xyz=(0.0, HANDLE_CHANNEL_D * 0.5 - 0.003, HANDLE_CHANNEL_H * 0.5)),
        material=trim_black,
        name="front_strip",
    )
    rear_guide_0.visual(
        Box((0.004, HANDLE_CHANNEL_D, HANDLE_CHANNEL_H)),
        origin=Origin(xyz=(-HANDLE_CHANNEL_W * 0.5 + 0.002, 0.0, HANDLE_CHANNEL_H * 0.5)),
        material=trim_black,
        name="outer_rail",
    )
    rear_guide_0.visual(
        Box((0.004, HANDLE_CHANNEL_D, HANDLE_CHANNEL_H)),
        origin=Origin(xyz=(HANDLE_CHANNEL_W * 0.5 - 0.002, 0.0, HANDLE_CHANNEL_H * 0.5)),
        material=trim_black,
        name="inner_rail",
    )

    rear_guide_1 = model.part("rear_guide_1")
    rear_guide_1.visual(
        Box((HANDLE_CHANNEL_W, 0.006, HANDLE_CHANNEL_H)),
        origin=Origin(xyz=(0.0, HANDLE_CHANNEL_D * 0.5 - 0.003, HANDLE_CHANNEL_H * 0.5)),
        material=trim_black,
        name="front_strip",
    )
    rear_guide_1.visual(
        Box((0.004, HANDLE_CHANNEL_D, HANDLE_CHANNEL_H)),
        origin=Origin(xyz=(-HANDLE_CHANNEL_W * 0.5 + 0.002, 0.0, HANDLE_CHANNEL_H * 0.5)),
        material=trim_black,
        name="inner_rail",
    )
    rear_guide_1.visual(
        Box((0.004, HANDLE_CHANNEL_D, HANDLE_CHANNEL_H)),
        origin=Origin(xyz=(HANDLE_CHANNEL_W * 0.5 - 0.002, 0.0, HANDLE_CHANNEL_H * 0.5)),
        material=trim_black,
        name="outer_rail",
    )

    lid_hinge_0 = model.part("lid_hinge_0")
    lid_hinge_0.visual(
        Cylinder(radius=0.010, length=0.080),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="barrel",
    )
    lid_hinge_0.visual(
        Box((0.080, 0.012, 0.020)),
        origin=Origin(xyz=(0.0, 0.016, -0.010)),
        material=dark_steel,
        name="leaf",
    )

    lid_hinge_1 = model.part("lid_hinge_1")
    lid_hinge_1.visual(
        Cylinder(radius=0.010, length=0.080),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="barrel",
    )
    lid_hinge_1.visual(
        Box((0.080, 0.012, 0.020)),
        origin=Origin(xyz=(0.0, 0.016, -0.010)),
        material=dark_steel,
        name="leaf",
    )

    pull_handle = model.part("pull_handle")
    pull_handle.visual(
        Box((HANDLE_LEG_W, HANDLE_LEG_D, HANDLE_BAR_LENGTH)),
        origin=Origin(xyz=(-HANDLE_CHANNEL_X, 0.0, HANDLE_BAR_LENGTH * 0.5)),
        material=handle_steel,
        name="left_leg",
    )
    pull_handle.visual(
        Box((HANDLE_LEG_W, HANDLE_LEG_D, HANDLE_BAR_LENGTH)),
        origin=Origin(xyz=(HANDLE_CHANNEL_X, 0.0, HANDLE_BAR_LENGTH * 0.5)),
        material=handle_steel,
        name="right_leg",
    )
    pull_handle.visual(
        Box((HANDLE_GRIP_W, 0.020, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, HANDLE_BAR_LENGTH - 0.015)),
        material=grip_black,
        name="top_grip",
    )
    pull_handle.visual(
        Box((HANDLE_GRIP_W, 0.016, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, HANDLE_BAR_LENGTH - 0.120)),
        material=handle_steel,
        name="brace",
    )

    wheel_tire_mesh = _mesh(
        cq.Workplane("YZ").circle(WHEEL_RADIUS).circle(0.073).extrude(0.028, both=True),
        "tool_chest_wheel_tire",
    )
    wheel_rim_mesh = _mesh(
        cq.Workplane("YZ").circle(0.073).circle(0.026).extrude(0.020, both=True),
        "tool_chest_wheel_rim",
    )

    rear_wheel_0 = model.part("rear_wheel_0")
    rear_wheel_0.visual(
        wheel_tire_mesh,
        material=rubber_black,
        name="tire",
    )
    rear_wheel_0.visual(
        wheel_rim_mesh,
        material=dark_steel,
        name="rim",
    )
    rear_wheel_0.visual(
        Cylinder(radius=0.024, length=0.066),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="hub",
    )

    rear_wheel_1 = model.part("rear_wheel_1")
    rear_wheel_1.visual(
        wheel_tire_mesh,
        material=rubber_black,
        name="tire",
    )
    rear_wheel_1.visual(
        wheel_rim_mesh,
        material=dark_steel,
        name="rim",
    )
    rear_wheel_1.visual(
        Cylinder(radius=0.024, length=0.066),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="hub",
    )

    side_latch_0 = model.part("side_latch_0")
    side_latch_0.visual(
        Cylinder(radius=0.008, length=0.030),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pivot",
    )
    side_latch_0.visual(
        Box((0.014, 0.028, 0.110)),
        origin=Origin(xyz=(-0.008, 0.0, 0.055)),
        material=trim_black,
        name="lever",
    )
    side_latch_0.visual(
        Box((0.024, 0.028, 0.018)),
        origin=Origin(xyz=(-0.013, 0.0, 0.108)),
        material=trim_black,
        name="hook",
    )

    side_latch_1 = model.part("side_latch_1")
    side_latch_1.visual(
        Cylinder(radius=0.008, length=0.030),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pivot",
    )
    side_latch_1.visual(
        Box((0.014, 0.028, 0.110)),
        origin=Origin(xyz=(0.008, 0.0, 0.055)),
        material=trim_black,
        name="lever",
    )
    side_latch_1.visual(
        Box((0.024, 0.028, 0.018)),
        origin=Origin(xyz=(0.013, 0.0, 0.108)),
        material=trim_black,
        name="hook",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -(BODY_D * 0.5 + 0.008), BODY_CLEARANCE + BODY_H + LID_SEAM_GAP)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.8, lower=0.0, upper=1.45),
    )
    model.articulation(
        "body_to_pull_handle",
        ArticulationType.PRISMATIC,
        parent=body,
        child=pull_handle,
        origin=Origin(xyz=(0.0, HANDLE_CHANNEL_Y + 0.003, HANDLE_CHANNEL_BOTTOM)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.45, lower=0.0, upper=HANDLE_TRAVEL),
    )
    model.articulation(
        "body_to_rear_guide_0",
        ArticulationType.FIXED,
        parent=body,
        child=rear_guide_0,
        origin=Origin(xyz=(-HANDLE_CHANNEL_X, HANDLE_CHANNEL_Y + 0.003, HANDLE_CHANNEL_BOTTOM)),
    )
    model.articulation(
        "body_to_rear_guide_1",
        ArticulationType.FIXED,
        parent=body,
        child=rear_guide_1,
        origin=Origin(xyz=(HANDLE_CHANNEL_X, HANDLE_CHANNEL_Y + 0.003, HANDLE_CHANNEL_BOTTOM)),
    )
    model.articulation(
        "lid_to_hinge_0",
        ArticulationType.FIXED,
        parent=lid,
        child=lid_hinge_0,
        origin=Origin(xyz=(-0.210, -0.001, -0.014)),
    )
    model.articulation(
        "lid_to_hinge_1",
        ArticulationType.FIXED,
        parent=lid,
        child=lid_hinge_1,
        origin=Origin(xyz=(0.210, -0.001, -0.014)),
    )
    model.articulation(
        "rear_wheel_0_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=rear_wheel_0,
        origin=Origin(xyz=(-0.390, -0.255, WHEEL_RADIUS)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=20.0),
    )
    model.articulation(
        "rear_wheel_1_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=rear_wheel_1,
        origin=Origin(xyz=(0.390, -0.255, WHEEL_RADIUS)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=20.0),
    )
    model.articulation(
        "body_to_side_latch_0",
        ArticulationType.REVOLUTE,
        parent=body,
        child=side_latch_0,
        origin=Origin(xyz=(-(BODY_W * 0.5 + 0.008), 0.095, BODY_CLEARANCE + BODY_H - 0.065)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=0.0, upper=1.10),
    )
    model.articulation(
        "body_to_side_latch_1",
        ArticulationType.REVOLUTE,
        parent=body,
        child=side_latch_1,
        origin=Origin(xyz=((BODY_W * 0.5 + 0.008), 0.095, BODY_CLEARANCE + BODY_H - 0.065)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=0.0, upper=1.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    lid_hinge_0 = object_model.get_part("lid_hinge_0")
    lid_hinge_1 = object_model.get_part("lid_hinge_1")
    pull_handle = object_model.get_part("pull_handle")
    rear_guide_0 = object_model.get_part("rear_guide_0")
    rear_guide_1 = object_model.get_part("rear_guide_1")
    rear_wheel_0 = object_model.get_part("rear_wheel_0")
    rear_wheel_1 = object_model.get_part("rear_wheel_1")
    side_latch_0 = object_model.get_part("side_latch_0")
    side_latch_1 = object_model.get_part("side_latch_1")
    lid_hinge = object_model.get_articulation("body_to_lid")
    handle_slide = object_model.get_articulation("body_to_pull_handle")
    latch_0_joint = object_model.get_articulation("body_to_side_latch_0")
    latch_1_joint = object_model.get_articulation("body_to_side_latch_1")

    ctx.allow_overlap(
        body,
        lid_hinge_0,
        elem_a="body_shell",
        elem_b="leaf",
        reason="The rear hinge plate is intentionally simplified as a flush-mounted leaf nested into the shell lip at the hinge line.",
    )
    ctx.allow_overlap(
        body,
        lid_hinge_1,
        elem_a="body_shell",
        elem_b="leaf",
        reason="The rear hinge plate is intentionally simplified as a flush-mounted leaf nested into the shell lip at the hinge line.",
    )
    ctx.allow_overlap(
        body,
        rear_wheel_0,
        elem_a="axle_stub_0",
        elem_b="hub",
        reason="The wheel hub is modeled as a solid proxy surrounding the axle stub for the rolling joint.",
    )
    ctx.allow_overlap(
        body,
        rear_wheel_1,
        elem_a="axle_stub_1",
        elem_b="hub",
        reason="The wheel hub is modeled as a solid proxy surrounding the axle stub for the rolling joint.",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="body_shell",
            max_gap=0.006,
            max_penetration=0.0,
            name="lid sits tightly on the chest body",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_shell",
            elem_b="body_shell",
            min_overlap=0.30,
            name="lid covers the main compartment opening",
        )
        closed_lid_aabb = ctx.part_world_aabb(lid)

    with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
        open_lid_aabb = ctx.part_world_aabb(lid)

    ctx.check(
        "lid swings upward above the body",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.12,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    with ctx.pose({handle_slide: 0.0}):
        ctx.expect_gap(
            pull_handle,
            rear_guide_0,
            axis="x",
            positive_elem="left_leg",
            negative_elem="outer_rail",
            min_gap=0.005,
            max_gap=0.011,
            name="left handle leg clears the outer rear guide rail at rest",
        )
        ctx.expect_gap(
            rear_guide_0,
            pull_handle,
            axis="x",
            positive_elem="inner_rail",
            negative_elem="left_leg",
            min_gap=0.005,
            max_gap=0.011,
            name="left handle leg clears the inner rear guide rail at rest",
        )
        ctx.expect_gap(
            rear_guide_0,
            pull_handle,
            axis="y",
            positive_elem="front_strip",
            negative_elem="left_leg",
            max_gap=0.001,
            max_penetration=0.0,
            name="left handle leg stays behind the rear guide face at rest",
        )
        ctx.expect_gap(
            rear_guide_1,
            pull_handle,
            axis="x",
            positive_elem="outer_rail",
            negative_elem="right_leg",
            min_gap=0.005,
            max_gap=0.011,
            name="right handle leg clears the outer rear guide rail at rest",
        )
        ctx.expect_gap(
            pull_handle,
            rear_guide_1,
            axis="x",
            positive_elem="right_leg",
            negative_elem="inner_rail",
            min_gap=0.005,
            max_gap=0.011,
            name="right handle leg clears the inner rear guide rail at rest",
        )
        ctx.expect_gap(
            rear_guide_1,
            pull_handle,
            axis="y",
            positive_elem="front_strip",
            negative_elem="right_leg",
            max_gap=0.001,
            max_penetration=0.0,
            name="right handle leg stays behind the rear guide face at rest",
        )
        ctx.expect_overlap(
            pull_handle,
            rear_guide_0,
            axes="z",
            elem_a="left_leg",
            elem_b="front_strip",
            min_overlap=0.26,
            name="left handle leg remains deeply inserted when collapsed",
        )
        ctx.expect_overlap(
            pull_handle,
            rear_guide_1,
            axes="z",
            elem_a="right_leg",
            elem_b="front_strip",
            min_overlap=0.26,
            name="right handle leg remains deeply inserted when collapsed",
        )
        rest_handle_pos = ctx.part_world_position(pull_handle)

    with ctx.pose({handle_slide: handle_slide.motion_limits.upper}):
        ctx.expect_gap(
            pull_handle,
            rear_guide_0,
            axis="x",
            positive_elem="left_leg",
            negative_elem="outer_rail",
            min_gap=0.005,
            max_gap=0.011,
            name="left handle leg keeps side clearance in the rear guide when extended",
        )
        ctx.expect_gap(
            rear_guide_0,
            pull_handle,
            axis="x",
            positive_elem="inner_rail",
            negative_elem="left_leg",
            min_gap=0.005,
            max_gap=0.011,
            name="left handle leg keeps inner-rail clearance when extended",
        )
        ctx.expect_gap(
            rear_guide_0,
            pull_handle,
            axis="y",
            positive_elem="front_strip",
            negative_elem="left_leg",
            max_gap=0.001,
            max_penetration=0.0,
            name="left handle leg stays behind the rear guide face when extended",
        )
        ctx.expect_gap(
            rear_guide_1,
            pull_handle,
            axis="x",
            positive_elem="outer_rail",
            negative_elem="right_leg",
            min_gap=0.005,
            max_gap=0.011,
            name="right handle leg keeps side clearance in the rear guide when extended",
        )
        ctx.expect_gap(
            pull_handle,
            rear_guide_1,
            axis="x",
            positive_elem="right_leg",
            negative_elem="inner_rail",
            min_gap=0.005,
            max_gap=0.011,
            name="right handle leg keeps inner-rail clearance when extended",
        )
        ctx.expect_gap(
            rear_guide_1,
            pull_handle,
            axis="y",
            positive_elem="front_strip",
            negative_elem="right_leg",
            max_gap=0.001,
            max_penetration=0.0,
            name="right handle leg stays behind the rear guide face when extended",
        )
        ctx.expect_overlap(
            pull_handle,
            rear_guide_0,
            axes="z",
            elem_a="left_leg",
            elem_b="front_strip",
            min_overlap=0.065,
            name="left handle leg keeps retained insertion at full extension",
        )
        ctx.expect_overlap(
            pull_handle,
            rear_guide_1,
            axes="z",
            elem_a="right_leg",
            elem_b="front_strip",
            min_overlap=0.065,
            name="right handle leg keeps retained insertion at full extension",
        )
        extended_handle_pos = ctx.part_world_position(pull_handle)

    ctx.check(
        "pull handle extends upward",
        rest_handle_pos is not None
        and extended_handle_pos is not None
        and extended_handle_pos[2] > rest_handle_pos[2] + 0.20,
        details=f"rest={rest_handle_pos}, extended={extended_handle_pos}",
    )

    ctx.expect_overlap(
        rear_wheel_0,
        body,
        axes="xyz",
        min_overlap=0.02,
        elem_a="rim",
        elem_b="axle_stub_0",
        name="left rear wheel stays captured on the left axle stub",
    )
    ctx.expect_overlap(
        rear_wheel_1,
        body,
        axes="xyz",
        min_overlap=0.02,
        elem_a="rim",
        elem_b="axle_stub_1",
        name="right rear wheel stays captured on the right axle stub",
    )

    with ctx.pose({latch_0_joint: 0.0, latch_1_joint: 0.0}):
        ctx.expect_overlap(
            side_latch_0,
            body,
            axes="z",
            elem_a="lever",
            elem_b="body_shell",
            min_overlap=0.05,
            name="left latch covers the body side below the seam",
        )
        ctx.expect_overlap(
            side_latch_0,
            lid,
            axes="z",
            elem_a="lever",
            elem_b="lid_shell",
            min_overlap=0.035,
            name="left latch bridges up onto the lid side",
        )
        ctx.expect_overlap(
            side_latch_1,
            body,
            axes="z",
            elem_a="lever",
            elem_b="body_shell",
            min_overlap=0.05,
            name="right latch covers the body side below the seam",
        )
        ctx.expect_overlap(
            side_latch_1,
            lid,
            axes="z",
            elem_a="lever",
            elem_b="lid_shell",
            min_overlap=0.035,
            name="right latch bridges up onto the lid side",
        )
        latch_0_closed_aabb = ctx.part_world_aabb(side_latch_0)
        latch_1_closed_aabb = ctx.part_world_aabb(side_latch_1)

    with ctx.pose({latch_0_joint: latch_0_joint.motion_limits.upper, latch_1_joint: latch_1_joint.motion_limits.upper}):
        latch_0_open_aabb = ctx.part_world_aabb(side_latch_0)
        latch_1_open_aabb = ctx.part_world_aabb(side_latch_1)

    ctx.check(
        "left latch swings outward",
        latch_0_closed_aabb is not None
        and latch_0_open_aabb is not None
        and latch_0_open_aabb[0][0] < latch_0_closed_aabb[0][0] - 0.03,
        details=f"closed={latch_0_closed_aabb}, open={latch_0_open_aabb}",
    )
    ctx.check(
        "right latch swings outward",
        latch_1_closed_aabb is not None
        and latch_1_open_aabb is not None
        and latch_1_open_aabb[1][0] > latch_1_closed_aabb[1][0] + 0.03,
        details=f"closed={latch_1_closed_aabb}, open={latch_1_open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
