from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    wire_from_points,
)

BODY_W = 0.38
BODY_D = 0.34
BODY_H = 0.72
BODY_WALL = 0.005
BODY_CORNER_R = 0.020

PEDAL_RECESS_W = 0.30
PEDAL_RECESS_D = 0.055
PEDAL_RECESS_H = 0.095
PEDAL_RECESS_Z = 0.056

HANDLE_NOTCH_W = 0.16
HANDLE_NOTCH_D = 0.050
HANDLE_NOTCH_H = 0.20
HANDLE_NOTCH_Z = 0.61

BUCKET_W = 0.346
BUCKET_D = 0.298
BUCKET_H = 0.62
BUCKET_WALL = 0.004
BUCKET_Z = 0.028

LID_W = 0.392
LID_D = 0.348
LID_PANEL_T = 0.008
LID_SKIRT_T = 0.010
LID_SIDE_SKIRT_H = 0.020
LID_FRONT_LIP_H = 0.030
LID_HINGE_R = 0.007

PEDAL_W = 0.275
PEDAL_LEN = 0.105
PEDAL_T = 0.018
PEDAL_AXLE_R = 0.0065

HANDLE_SPAN = 0.150
HANDLE_DROP = 0.058
HANDLE_RADIUS = 0.0032
HANDLE_PIVOT_Z = BUCKET_H - 0.055
HANDLE_PIVOT_Y = BUCKET_D / 2.0 + 0.013


def _centered_box(size_x: float, size_y: float, size_z: float, center_xyz: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(size_x, size_y, size_z).translate(center_xyz)


def _body_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H)
        .edges("|Z")
        .fillet(BODY_CORNER_R)
        .translate((0.0, 0.0, BODY_H / 2.0))
    )
    inner = _centered_box(
        BODY_W - 2.0 * BODY_WALL,
        BODY_D - 2.0 * BODY_WALL,
        BODY_H - BODY_WALL,
        (0.0, 0.0, BODY_WALL + (BODY_H - BODY_WALL) / 2.0),
    )
    shell = outer.cut(inner)

    pedal_recess = _centered_box(
        PEDAL_RECESS_W,
        PEDAL_RECESS_D,
        PEDAL_RECESS_H,
        (0.0, BODY_D / 2.0 - PEDAL_RECESS_D / 2.0 + 0.001, PEDAL_RECESS_Z),
    )
    handle_notch = _centered_box(
        HANDLE_NOTCH_W,
        HANDLE_NOTCH_D,
        HANDLE_NOTCH_H,
        (0.0, BODY_D / 2.0 - HANDLE_NOTCH_D / 2.0 + 0.001, HANDLE_NOTCH_Z),
    )
    pedal_mount = _centered_box(
        PEDAL_RECESS_W + 0.004,
        0.016,
        0.022,
        (0.0, BODY_D / 2.0 - 0.005, 0.074),
    )
    return shell.cut(pedal_recess).cut(handle_notch).union(pedal_mount)


def _bucket_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(BUCKET_W, BUCKET_D, BUCKET_H)
        .edges("|Z")
        .fillet(0.012)
        .translate((0.0, 0.0, BUCKET_H / 2.0))
    )
    inner = _centered_box(
        BUCKET_W - 2.0 * BUCKET_WALL,
        BUCKET_D - 2.0 * BUCKET_WALL,
        BUCKET_H - BUCKET_WALL,
        (0.0, 0.0, BUCKET_WALL + (BUCKET_H - BUCKET_WALL) / 2.0),
    )
    shell = outer.cut(inner)

    rim_outer = _centered_box(BUCKET_W + 0.010, BUCKET_D + 0.010, 0.012, (0.0, 0.0, BUCKET_H - 0.006))
    rim_inner = _centered_box(BUCKET_W - 0.004, BUCKET_D - 0.004, 0.014, (0.0, 0.0, BUCKET_H - 0.006))
    rim = rim_outer.cut(rim_inner)

    boss_left = _centered_box(0.016, 0.010, 0.018, (-HANDLE_SPAN / 2.0, BUCKET_D / 2.0 + 0.005, HANDLE_PIVOT_Z))
    boss_right = _centered_box(0.016, 0.010, 0.018, (HANDLE_SPAN / 2.0, BUCKET_D / 2.0 + 0.005, HANDLE_PIVOT_Z))
    return shell.union(rim).union(boss_left).union(boss_right)


def _bucket_tab_shape(side_sign: float) -> cq.Workplane:
    return _centered_box(
        0.012,
        0.028,
        0.010,
        (side_sign * (BUCKET_W / 2.0 + 0.006), -0.078, BUCKET_H - 0.010),
    )


def _lid_shape() -> cq.Workplane:
    panel = _centered_box(LID_W, LID_D, LID_PANEL_T, (0.0, LID_D / 2.0, LID_PANEL_T / 2.0))
    rear_web = _centered_box(LID_W * 0.95, 0.018, 0.016, (0.0, 0.009, -0.003))
    front_lip = _centered_box(LID_W, 0.012, LID_FRONT_LIP_H, (0.0, LID_D - 0.006, -(LID_FRONT_LIP_H - LID_PANEL_T) / 2.0))
    side_skirt_left = _centered_box(
        LID_SKIRT_T,
        LID_D - 0.028,
        LID_SIDE_SKIRT_H,
        (-LID_W / 2.0 + LID_SKIRT_T / 2.0, LID_D / 2.0 + 0.008, -(LID_SIDE_SKIRT_H - LID_PANEL_T) / 2.0),
    )
    side_skirt_right = _centered_box(
        LID_SKIRT_T,
        LID_D - 0.028,
        LID_SIDE_SKIRT_H,
        (LID_W / 2.0 - LID_SKIRT_T / 2.0, LID_D / 2.0 + 0.008, -(LID_SIDE_SKIRT_H - LID_PANEL_T) / 2.0),
    )
    hinge_barrel = (
        cq.Workplane("YZ")
        .circle(LID_HINGE_R)
        .extrude(LID_W * 0.78, both=True)
        .translate((0.0, 0.0, 0.004))
    )
    return panel.union(rear_web).union(front_lip).union(side_skirt_left).union(side_skirt_right).union(hinge_barrel)


def _pedal_shape() -> cq.Workplane:
    tread = _centered_box(PEDAL_W, PEDAL_LEN, PEDAL_T, (0.0, PEDAL_LEN / 2.0 + 0.020, -0.029))
    bridge = _centered_box(PEDAL_W * 0.76, 0.042, 0.022, (0.0, 0.014, -0.010))
    axle = cq.Workplane("YZ").circle(PEDAL_AXLE_R).extrude(PEDAL_W * 0.82, both=True)
    pedal = tread.union(bridge).union(axle)

    for groove_y in (0.026, 0.050, 0.074, 0.098):
        groove = _centered_box(PEDAL_W * 0.84, 0.006, 0.003, (0.0, groove_y, -0.0195))
        pedal = pedal.cut(groove)

    return pedal


def _handle_geometry():
    return wire_from_points(
        [
            (-HANDLE_SPAN / 2.0, 0.0, 0.0),
            (-HANDLE_SPAN / 2.0, 0.0, -HANDLE_DROP),
            (HANDLE_SPAN / 2.0, 0.0, -HANDLE_DROP),
            (HANDLE_SPAN / 2.0, 0.0, 0.0),
        ],
        radius=HANDLE_RADIUS,
        closed_path=False,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.018,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hospital_waste_bin")

    body_finish = model.material("body_finish", rgba=(0.94, 0.95, 0.96, 1.0))
    lid_finish = model.material("lid_finish", rgba=(0.74, 0.12, 0.14, 1.0))
    pedal_finish = model.material("pedal_finish", rgba=(0.16, 0.17, 0.18, 1.0))
    bucket_finish = model.material("bucket_finish", rgba=(0.80, 0.82, 0.84, 1.0))
    handle_finish = model.material("handle_finish", rgba=(0.42, 0.44, 0.46, 1.0))

    body = model.part("body")
    body.visual(mesh_from_cadquery(_body_shape(), "body_shell"), material=body_finish, name="body_shell")

    bucket = model.part("bucket")
    bucket.visual(
        mesh_from_cadquery(_bucket_shape(), "bucket_shell"),
        material=bucket_finish,
        name="bucket_shell",
    )
    bucket.visual(mesh_from_cadquery(_bucket_tab_shape(-1.0), "bucket_tab_0"), material=bucket_finish, name="bucket_tab_0")
    bucket.visual(mesh_from_cadquery(_bucket_tab_shape(1.0), "bucket_tab_1"), material=bucket_finish, name="bucket_tab_1")

    lid = model.part("lid")
    lid.visual(mesh_from_cadquery(_lid_shape(), "lid_panel"), material=lid_finish, name="lid_panel")

    pedal = model.part("pedal")
    pedal.visual(mesh_from_cadquery(_pedal_shape(), "pedal_lever"), material=pedal_finish, name="pedal_lever")

    bucket_handle = model.part("bucket_handle")
    bucket_handle.visual(
        mesh_from_geometry(_handle_geometry(), "bucket_handle"),
        material=handle_finish,
        name="bucket_handle",
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
        origin=Origin(xyz=(0.0, -BODY_D / 2.0 + 0.002, BODY_H + 0.026)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.32, effort=12.0, velocity=1.2),
    )
    model.articulation(
        "body_to_pedal",
        ArticulationType.REVOLUTE,
        parent=body,
        child=pedal,
        origin=Origin(xyz=(0.0, BODY_D / 2.0 + 0.010, 0.074)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.45, effort=20.0, velocity=1.5),
    )
    model.articulation(
        "bucket_to_bucket_handle",
        ArticulationType.REVOLUTE,
        parent=bucket,
        child=bucket_handle,
        origin=Origin(xyz=(0.0, HANDLE_PIVOT_Y, HANDLE_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.20, effort=4.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    bucket = object_model.get_part("bucket")
    lid = object_model.get_part("lid")
    pedal = object_model.get_part("pedal")
    bucket_handle = object_model.get_part("bucket_handle")

    lid_hinge = object_model.get_articulation("body_to_lid")
    pedal_hinge = object_model.get_articulation("body_to_pedal")
    handle_hinge = object_model.get_articulation("bucket_to_bucket_handle")

    ctx.allow_overlap(
        body,
        bucket,
        elem_a="body_shell",
        elem_b="bucket_tab_0",
        reason="The removable inner bucket is hung from simplified side tabs that intentionally seat into the shell ledge.",
    )
    ctx.allow_overlap(
        body,
        bucket,
        elem_a="body_shell",
        elem_b="bucket_tab_1",
        reason="The removable inner bucket is hung from simplified side tabs that intentionally seat into the shell ledge.",
    )

    ctx.expect_within(
        bucket,
        body,
        axes="xy",
        margin=0.025,
        name="bucket stays nested within the outer body footprint",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            min_gap=0.0,
            max_gap=0.018,
            name="closed lid sits just above the body rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.30,
            name="closed lid covers the bin opening",
        )
        ctx.expect_gap(
            lid,
            bucket_handle,
            axis="z",
            min_gap=0.09,
            name="inner bucket handle remains below the lid line",
        )

    closed_lid_aabb = None
    open_lid_aabb = None
    if lid_hinge.motion_limits is not None and lid_hinge.motion_limits.upper is not None:
        with ctx.pose({lid_hinge: 0.0}):
            closed_lid_aabb = ctx.part_world_aabb(lid)
        with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
            open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid opens upward on the rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.10,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    rest_pedal_aabb = None
    pressed_pedal_aabb = None
    if pedal_hinge.motion_limits is not None and pedal_hinge.motion_limits.upper is not None:
        with ctx.pose({pedal_hinge: 0.0}):
            rest_pedal_aabb = ctx.part_world_aabb(pedal)
        with ctx.pose({pedal_hinge: pedal_hinge.motion_limits.upper}):
            pressed_pedal_aabb = ctx.part_world_aabb(pedal)
    ctx.check(
        "wide front pedal depresses downward",
        rest_pedal_aabb is not None
        and pressed_pedal_aabb is not None
        and pressed_pedal_aabb[0][2] < rest_pedal_aabb[0][2] - 0.015,
        details=f"rest={rest_pedal_aabb}, pressed={pressed_pedal_aabb}",
    )

    rest_handle_aabb = None
    lifted_handle_aabb = None
    if handle_hinge.motion_limits is not None and handle_hinge.motion_limits.upper is not None:
        with ctx.pose({handle_hinge: 0.0}):
            rest_handle_aabb = ctx.part_world_aabb(bucket_handle)
        with ctx.pose({handle_hinge: handle_hinge.motion_limits.upper}):
            lifted_handle_aabb = ctx.part_world_aabb(bucket_handle)
    ctx.check(
        "bucket handle lifts on side pivots",
        rest_handle_aabb is not None
        and lifted_handle_aabb is not None
        and lifted_handle_aabb[0][2] > rest_handle_aabb[0][2] + 0.020
        and lifted_handle_aabb[1][1] > rest_handle_aabb[1][1] + 0.020,
        details=f"rest={rest_handle_aabb}, lifted={lifted_handle_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
