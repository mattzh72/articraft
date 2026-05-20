from __future__ import annotations

from math import atan2, pi, sqrt

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


BODY_W = 0.18
BODY_D = 0.26
BODY_H = 0.325
BODY_WALL = 0.0028
BODY_CORNER = 0.022

LID_W = 0.19
LID_D = 0.266
LID_BASE_T = 0.008
LID_DOME_H = 0.016
LID_CORNER = 0.018
LID_HINGE_R = 0.005

BUCKET_W = 0.164
BUCKET_D = 0.238
BUCKET_H = 0.286
BUCKET_WALL = 0.0022
BUCKET_CORNER = 0.017
BUCKET_RIM_H = 0.008
BUCKET_Z = 0.013

HANDLE_PIVOT_Y = -BUCKET_D * 0.5 + 0.026
HANDLE_PIVOT_Z = BUCKET_H - 0.021
HANDLE_SPAN = 0.144
HANDLE_R = 0.0034
HANDLE_DROP_Y = -0.010
HANDLE_DROP_Z = -0.043

PEDAL_PIVOT_Y = -BODY_D * 0.5 - 0.010
PEDAL_PIVOT_Z = 0.0258
PEDAL_LINK_X = BODY_W * 0.5 + 0.006


def _cq_box(size: tuple[float, float, float], xyz: tuple[float, float, float]) -> cq.Workplane:
    sx, sy, sz = size
    return (
        cq.Workplane("XY")
        .box(sx, sy, sz, centered=(True, True, False))
        .translate(xyz)
    )


def _body_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(BODY_CORNER)
    )
    inner = (
        cq.Workplane("XY")
        .box(BODY_W - 2.0 * BODY_WALL, BODY_D - 2.0 * BODY_WALL, BODY_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(max(BODY_CORNER - BODY_WALL, 0.002))
        .translate((0.0, 0.0, BODY_WALL))
    )
    shell = outer.cut(inner)
    rear_band = _cq_box(
        (BODY_W * 0.82, 0.012, 0.018),
        (0.0, BODY_D * 0.5 - 0.006, BODY_H - 0.018),
    )
    front_pedal_mount = _cq_box(
        (0.090, 0.014, 0.023),
        (0.0, -BODY_D * 0.5 + 0.001, 0.0),
    )
    return shell.union(rear_band).union(front_pedal_mount)


def _bucket_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(BUCKET_W, BUCKET_D, BUCKET_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(BUCKET_CORNER)
    )
    inner = (
        cq.Workplane("XY")
        .box(BUCKET_W - 2.0 * BUCKET_WALL, BUCKET_D - 2.0 * BUCKET_WALL, BUCKET_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(max(BUCKET_CORNER - BUCKET_WALL, 0.002))
        .translate((0.0, 0.0, BUCKET_WALL))
    )
    shell = outer.cut(inner)
    rim = _cq_box(
        (BUCKET_W + 0.006, BUCKET_D + 0.006, BUCKET_RIM_H),
        (0.0, 0.0, BUCKET_H - BUCKET_RIM_H),
    ).cut(
        _cq_box(
            (BUCKET_W - 0.006, BUCKET_D - 0.006, BUCKET_RIM_H + 0.004),
            (0.0, 0.0, BUCKET_H - BUCKET_RIM_H - 0.001),
        )
    )
    boss_offset_x = BUCKET_W * 0.5 - BUCKET_WALL - 0.003
    boss_z = HANDLE_PIVOT_Z - 0.006
    boss_left = _cq_box((0.007, 0.015, 0.012), (boss_offset_x, HANDLE_PIVOT_Y, boss_z))
    boss_right = _cq_box((0.007, 0.015, 0.012), (-boss_offset_x, HANDLE_PIVOT_Y, boss_z))
    return shell.union(rim).union(boss_left).union(boss_right)


def _lid_shape() -> cq.Workplane:
    shell = (
        _cq_box((LID_W, LID_D, LID_BASE_T), (0.0, -LID_D * 0.5, 0.0))
        .edges("|Z")
        .fillet(LID_CORNER)
    )
    dome = (
        _cq_box(
            (LID_W - 0.034, LID_D - 0.054, LID_DOME_H),
            (0.0, -LID_D * 0.5 - 0.004, LID_BASE_T),
        )
        .edges("|Z")
        .fillet(0.017)
        .edges(">Z")
        .fillet(0.009)
    )
    hinge_band = (
        cq.Workplane("YZ")
        .circle(LID_HINGE_R)
        .extrude(LID_W * 0.88)
        .translate((-LID_W * 0.44, 0.0, LID_HINGE_R * 0.72))
    )
    return shell.union(dome).union(hinge_band)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slim_step_bin")

    body_finish = model.material("body_finish", rgba=(0.90, 0.91, 0.89, 1.0))
    lid_finish = model.material("lid_finish", rgba=(0.88, 0.89, 0.87, 1.0))
    pedal_finish = model.material("pedal_finish", rgba=(0.14, 0.15, 0.16, 1.0))
    bucket_finish = model.material("bucket_finish", rgba=(0.78, 0.80, 0.81, 1.0))
    handle_finish = model.material("handle_finish", rgba=(0.32, 0.34, 0.36, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_W, BODY_D, BODY_WALL)),
        origin=Origin(xyz=(0.0, 0.0, BODY_WALL * 0.5)),
        material=body_finish,
        name="bottom",
    )
    body.visual(
        Box((BODY_W, BODY_WALL, BODY_H)),
        origin=Origin(xyz=(0.0, -BODY_D * 0.5 + BODY_WALL * 0.5, BODY_H * 0.5)),
        material=body_finish,
        name="front_wall",
    )
    body.visual(
        Box((BODY_W, BODY_WALL, BODY_H)),
        origin=Origin(xyz=(0.0, BODY_D * 0.5 - BODY_WALL * 0.5, BODY_H * 0.5)),
        material=body_finish,
        name="back_wall",
    )
    body.visual(
        Box((BODY_WALL, BODY_D - 2.0 * BODY_WALL, BODY_H)),
        origin=Origin(xyz=(BODY_W * 0.5 - BODY_WALL * 0.5, 0.0, BODY_H * 0.5)),
        material=body_finish,
        name="side_wall_0",
    )
    body.visual(
        Box((BODY_WALL, BODY_D - 2.0 * BODY_WALL, BODY_H)),
        origin=Origin(xyz=(-BODY_W * 0.5 + BODY_WALL * 0.5, 0.0, BODY_H * 0.5)),
        material=body_finish,
        name="side_wall_1",
    )
    body.visual(
        Box((BODY_W * 0.82, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, BODY_D * 0.5 - 0.006, BODY_H - 0.009)),
        material=body_finish,
        name="rear_band",
    )
    body.visual(
        Box((0.090, 0.016, 0.023)),
        origin=Origin(xyz=(0.0, -BODY_D * 0.5 + 0.001, 0.0115)),
        material=body_finish,
        name="pedal_mount",
    )

    bucket = model.part("bucket")
    bucket.visual(
        Box((BUCKET_W, BUCKET_D, BUCKET_WALL)),
        origin=Origin(xyz=(0.0, 0.0, BUCKET_WALL * 0.5)),
        material=bucket_finish,
        name="bottom",
    )
    bucket.visual(
        Box((BUCKET_W, BUCKET_WALL, BUCKET_H)),
        origin=Origin(xyz=(0.0, -BUCKET_D * 0.5 + BUCKET_WALL * 0.5, BUCKET_H * 0.5)),
        material=bucket_finish,
        name="front_wall",
    )
    bucket.visual(
        Box((BUCKET_W, BUCKET_WALL, BUCKET_H)),
        origin=Origin(xyz=(0.0, BUCKET_D * 0.5 - BUCKET_WALL * 0.5, BUCKET_H * 0.5)),
        material=bucket_finish,
        name="back_wall",
    )
    bucket.visual(
        Box((BUCKET_WALL, BUCKET_D - 2.0 * BUCKET_WALL, BUCKET_H)),
        origin=Origin(xyz=(BUCKET_W * 0.5 - BUCKET_WALL * 0.5, 0.0, BUCKET_H * 0.5)),
        material=bucket_finish,
        name="side_wall_0",
    )
    bucket.visual(
        Box((BUCKET_WALL, BUCKET_D - 2.0 * BUCKET_WALL, BUCKET_H)),
        origin=Origin(xyz=(-BUCKET_W * 0.5 + BUCKET_WALL * 0.5, 0.0, BUCKET_H * 0.5)),
        material=bucket_finish,
        name="side_wall_1",
    )
    bucket.visual(
        Box((BODY_W - 2.0 * BODY_WALL, 0.010, BUCKET_RIM_H)),
        origin=Origin(
            xyz=(
                0.0,
                -0.5 * (BODY_D - 2.0 * BODY_WALL) + 0.005,
                BUCKET_H - BUCKET_RIM_H * 0.5,
            )
        ),
        material=bucket_finish,
        name="rim_front",
    )
    bucket.visual(
        Box((BODY_W - 2.0 * BODY_WALL, 0.010, BUCKET_RIM_H)),
        origin=Origin(
            xyz=(
                0.0,
                0.5 * (BODY_D - 2.0 * BODY_WALL) - 0.005,
                BUCKET_H - BUCKET_RIM_H * 0.5,
            )
        ),
        material=bucket_finish,
        name="rim_back",
    )
    bucket.visual(
        Box((0.010, BODY_D - 2.0 * BODY_WALL - 0.010, BUCKET_RIM_H)),
        origin=Origin(
            xyz=(
                0.5 * (BODY_W - 2.0 * BODY_WALL) - 0.005,
                0.0,
                BUCKET_H - BUCKET_RIM_H * 0.5,
            )
        ),
        material=bucket_finish,
        name="rim_side_0",
    )
    bucket.visual(
        Box((0.010, BODY_D - 2.0 * BODY_WALL - 0.010, BUCKET_RIM_H)),
        origin=Origin(
            xyz=(
                -0.5 * (BODY_W - 2.0 * BODY_WALL) + 0.005,
                0.0,
                BUCKET_H - BUCKET_RIM_H * 0.5,
            )
        ),
        material=bucket_finish,
        name="rim_side_1",
    )
    boss_offset_x = BUCKET_W * 0.5 - BUCKET_WALL - 0.003
    bucket.visual(
        Box((0.007, 0.015, 0.012)),
        origin=Origin(xyz=(boss_offset_x, HANDLE_PIVOT_Y, HANDLE_PIVOT_Z - 0.006)),
        material=bucket_finish,
        name="handle_boss_0",
    )
    bucket.visual(
        Box((0.007, 0.015, 0.012)),
        origin=Origin(xyz=(-boss_offset_x, HANDLE_PIVOT_Y, HANDLE_PIVOT_Z - 0.006)),
        material=bucket_finish,
        name="handle_boss_1",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shape(), "step_bin_lid"),
        material=lid_finish,
        name="lid_shell",
    )

    pedal = model.part("pedal")
    pedal.visual(
        Cylinder(radius=0.005, length=0.110),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=pedal_finish,
        name="pivot_rod",
    )
    pedal.visual(
        Box((0.096, 0.034, 0.008)),
        origin=Origin(xyz=(0.0, -0.031, 0.003)),
        material=pedal_finish,
        name="tread_pad",
    )
    pedal.visual(
        Box((0.018, 0.022, 0.020)),
        origin=Origin(xyz=(0.032, -0.015, 0.004)),
        material=pedal_finish,
        name="cheek_0",
    )
    pedal.visual(
        Box((0.018, 0.022, 0.020)),
        origin=Origin(xyz=(-0.032, -0.015, 0.004)),
        material=pedal_finish,
        name="cheek_1",
    )
    pedal.visual(
        Box((0.048, 0.016, 0.014)),
        origin=Origin(xyz=(0.070, -0.005, 0.004)),
        material=pedal_finish,
        name="linkage_bracket",
    )
    pedal_arm_length = 0.112
    pedal_arm_roll = atan2(0.004, 0.112)
    pedal.visual(
        Cylinder(radius=0.004, length=pedal_arm_length),
        origin=Origin(
            xyz=(PEDAL_LINK_X, -0.002, pedal_arm_length * 0.5),
            rpy=(pedal_arm_roll, 0.0, 0.0),
        ),
        material=pedal_finish,
        name="linkage_arm",
    )
    pedal.visual(
        Box((0.016, 0.014, 0.014)),
        origin=Origin(xyz=(PEDAL_LINK_X, -0.001, pedal_arm_length + 0.001)),
        material=pedal_finish,
        name="linkage_head",
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=HANDLE_R, length=HANDLE_SPAN),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=handle_finish,
        name="top_bar",
    )
    handle.visual(
        Cylinder(radius=HANDLE_R, length=HANDLE_SPAN),
        origin=Origin(
            xyz=(0.0, HANDLE_DROP_Y, HANDLE_DROP_Z),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=handle_finish,
        name="grip_bar",
    )
    leg_length = sqrt(HANDLE_DROP_Y**2 + HANDLE_DROP_Z**2)
    leg_roll = atan2(-HANDLE_DROP_Y, HANDLE_DROP_Z)
    half_span = HANDLE_SPAN * 0.5
    handle.visual(
        Cylinder(radius=HANDLE_R, length=leg_length),
        origin=Origin(
            xyz=(half_span, HANDLE_DROP_Y * 0.5, HANDLE_DROP_Z * 0.5),
            rpy=(leg_roll, 0.0, 0.0),
        ),
        material=handle_finish,
        name="leg_0",
    )
    handle.visual(
        Cylinder(radius=HANDLE_R, length=leg_length),
        origin=Origin(
            xyz=(-half_span, HANDLE_DROP_Y * 0.5, HANDLE_DROP_Z * 0.5),
            rpy=(leg_roll, 0.0, 0.0),
        ),
        material=handle_finish,
        name="leg_1",
    )

    model.articulation(
        "body_to_bucket",
        ArticulationType.FIXED,
        parent=body,
        child=bucket,
        origin=Origin(xyz=(0.0, 0.0, BUCKET_Z)),
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, BODY_D * 0.5 - 0.003, BODY_H + 0.001)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.20),
    )
    model.articulation(
        "body_to_pedal",
        ArticulationType.REVOLUTE,
        parent=body,
        child=pedal,
        origin=Origin(xyz=(0.0, PEDAL_PIVOT_Y, PEDAL_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=3.0, lower=0.0, upper=0.45),
    )
    model.articulation(
        "bucket_to_handle",
        ArticulationType.REVOLUTE,
        parent=bucket,
        child=handle,
        origin=Origin(xyz=(0.0, HANDLE_PIVOT_Y, HANDLE_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    bucket = object_model.get_part("bucket")
    lid = object_model.get_part("lid")
    pedal = object_model.get_part("pedal")
    handle = object_model.get_part("handle")

    lid_hinge = object_model.get_articulation("body_to_lid")
    pedal_hinge = object_model.get_articulation("body_to_pedal")
    handle_hinge = object_model.get_articulation("bucket_to_handle")

    ctx.allow_overlap(
        body,
        pedal,
        elem_a="pedal_mount",
        elem_b="pivot_rod",
        reason="The pedal axle is intentionally represented as captured by the simplified front hinge bracket.",
    )

    ctx.expect_within(
        bucket,
        body,
        axes="xy",
        margin=0.010,
        name="bucket stays within body footprint",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.150,
            name="closed lid covers the bin opening",
        )
        ctx.expect_gap(
            lid,
            handle,
            axis="z",
            min_gap=0.020,
            name="resting handle stays below the lid line",
        )

    closed_lid_box = None
    open_lid_box = None
    with ctx.pose({lid_hinge: 0.0}):
        closed_lid_box = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 1.20}):
        open_lid_box = ctx.part_world_aabb(lid)
    ctx.check(
        "lid opens upward from the rear hinge",
        closed_lid_box is not None
        and open_lid_box is not None
        and open_lid_box[1][2] > closed_lid_box[1][2] + 0.08,
        details=f"closed={closed_lid_box}, open={open_lid_box}",
    )

    pedal_rest = None
    pedal_pressed = None
    with ctx.pose({pedal_hinge: 0.0}):
        pedal_rest = ctx.part_element_world_aabb(pedal, elem="tread_pad")
    with ctx.pose({pedal_hinge: 0.45}):
        pedal_pressed = ctx.part_element_world_aabb(pedal, elem="tread_pad")
    ctx.check(
        "pedal tread rotates downward when pressed",
        pedal_rest is not None
        and pedal_pressed is not None
        and pedal_pressed[0][2] < pedal_rest[0][2] - 0.010,
        details=f"rest={pedal_rest}, pressed={pedal_pressed}",
    )

    handle_rest = None
    handle_raised = None
    with ctx.pose({handle_hinge: 0.0}):
        handle_rest = ctx.part_element_world_aabb(handle, elem="grip_bar")
    with ctx.pose({handle_hinge: 1.35}):
        handle_raised = ctx.part_element_world_aabb(handle, elem="grip_bar")
    ctx.check(
        "inner bucket handle lifts upward",
        handle_rest is not None
        and handle_raised is not None
        and handle_raised[1][2] > handle_rest[1][2] + 0.018,
        details=f"rest={handle_rest}, raised={handle_raised}",
    )

    return ctx.report()


object_model = build_object_model()
