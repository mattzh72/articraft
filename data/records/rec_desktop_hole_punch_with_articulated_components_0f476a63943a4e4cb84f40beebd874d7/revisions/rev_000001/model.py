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


def _bridge_shell_mesh() -> object:
    center_shell = (
        cq.Workplane("XY")
        .box(0.074, 0.026, 0.018)
        .translate((-0.010, 0.0, 0.015))
    )
    rear_hump = (
        cq.Workplane("XY")
        .box(0.030, 0.026, 0.018)
        .translate((-0.047, 0.0, 0.024))
    )
    front_nose = (
        cq.Workplane("XY")
        .box(0.034, 0.020, 0.010)
        .translate((0.020, 0.0, 0.010))
    )
    return center_shell.union(rear_hump).union(front_nose)


def _handle_shell_mesh() -> object:
    rear_neck = (
        cq.Workplane("XY")
        .box(0.018, 0.012, 0.011)
        .translate((0.011, 0.0, 0.0055))
    )
    top_grip = (
        cq.Workplane("XY")
        .box(0.062, 0.022, 0.008)
        .translate((0.048, 0.0, 0.009))
    )
    finger_lip = (
        cq.Workplane("XY")
        .box(0.014, 0.020, 0.004)
        .translate((0.078, 0.0, 0.004))
    )
    underside_relief = (
        cq.Workplane("XY")
        .box(0.042, 0.016, 0.004)
        .translate((0.050, 0.0, 0.004))
    )
    return rear_neck.union(top_grip).union(finger_lip).cut(underside_relief)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_one_hole_punch")

    painted_metal = model.material("painted_metal", rgba=(0.22, 0.24, 0.28, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.65, 0.68, 0.72, 1.0))
    clear_smoke = model.material("clear_smoke", rgba=(0.72, 0.78, 0.84, 0.38))

    body = model.part("body")
    body.visual(
        Box((0.118, 0.038, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=painted_metal,
        name="base_plate",
    )
    body.visual(
        mesh_from_cadquery(_bridge_shell_mesh(), "bridge_shell"),
        material=painted_metal,
        name="bridge_shell",
    )
    body.visual(
        Box((0.038, 0.018, 0.010)),
        origin=Origin(xyz=(0.020, 0.0, 0.010)),
        material=painted_metal,
        name="throat_block",
    )
    body.visual(
        Cylinder(radius=0.0065, length=0.010),
        origin=Origin(xyz=(0.022, 0.0, 0.029)),
        material=satin_steel,
        name="die_collar",
    )
    body.visual(
        Box((0.024, 0.020, 0.014)),
        origin=Origin(xyz=(-0.049, 0.0, 0.024)),
        material=painted_metal,
        name="rear_shell",
    )
    body.visual(
        Box((0.014, 0.006, 0.022)),
        origin=Origin(xyz=(-0.033, 0.0085, 0.029)),
        material=painted_metal,
        name="hinge_mount_0",
    )
    body.visual(
        Box((0.014, 0.006, 0.022)),
        origin=Origin(xyz=(-0.033, -0.0085, 0.029)),
        material=painted_metal,
        name="hinge_mount_1",
    )
    body.visual(
        Box((0.010, 0.004, 0.004)),
        origin=Origin(xyz=(0.002, 0.0085, 0.004)),
        material=painted_metal,
        name="cover_mount_0",
    )
    body.visual(
        Box((0.010, 0.004, 0.004)),
        origin=Origin(xyz=(0.002, -0.0085, 0.004)),
        material=painted_metal,
        name="cover_mount_1",
    )
    body.visual(
        Box((0.006, 0.004, 0.010)),
        origin=Origin(xyz=(-0.051, 0.006, 0.024)),
        material=painted_metal,
        name="tab_mount_0",
    )
    body.visual(
        Box((0.006, 0.004, 0.010)),
        origin=Origin(xyz=(-0.051, -0.006, 0.024)),
        material=painted_metal,
        name="tab_mount_1",
    )

    handle = model.part("handle")
    handle.visual(
        Box((0.020, 0.011, 0.010)),
        origin=Origin(xyz=(0.010, 0.0, 0.005)),
        material=painted_metal,
        name="rear_neck",
    )
    handle.visual(
        Box((0.058, 0.022, 0.008)),
        origin=Origin(xyz=(0.049, 0.0, 0.0085)),
        material=painted_metal,
        name="handle_grip",
    )
    handle.visual(
        Box((0.012, 0.020, 0.004)),
        origin=Origin(xyz=(0.076, 0.0, 0.004)),
        material=painted_metal,
        name="finger_lip",
    )
    handle.visual(
        Cylinder(radius=0.004, length=0.009),
        origin=Origin(xyz=(0.0, 0.0, 0.0045), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="hinge_barrel",
    )

    chip_cover = model.part("chip_cover")
    chip_cover.visual(
        Box((0.003, 0.013, 0.018)),
        origin=Origin(xyz=(0.0015, 0.0, -0.009)),
        material=clear_smoke,
        name="rear_wall",
    )
    chip_cover.visual(
        Cylinder(radius=0.002, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=clear_smoke,
        name="cover_hinge_barrel",
    )
    chip_cover.visual(
        Box((0.034, 0.020, 0.0016)),
        origin=Origin(xyz=(0.018, 0.0, -0.0142)),
        material=clear_smoke,
        name="cover_floor",
    )
    chip_cover.visual(
        Box((0.036, 0.0016, 0.015)),
        origin=Origin(xyz=(0.018, 0.0102, -0.0075)),
        material=clear_smoke,
        name="side_wall_0",
    )
    chip_cover.visual(
        Box((0.036, 0.0016, 0.015)),
        origin=Origin(xyz=(0.018, -0.0102, -0.0075)),
        material=clear_smoke,
        name="side_wall_1",
    )
    chip_cover.visual(
        Box((0.0016, 0.020, 0.015)),
        origin=Origin(xyz=(0.0368, 0.0, -0.0075)),
        material=clear_smoke,
        name="front_wall",
    )

    lock_tab = model.part("lock_tab")
    lock_tab.visual(
        Cylinder(radius=0.002, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="tab_barrel",
    )
    lock_tab.visual(
        Box((0.004, 0.008, 0.007)),
        origin=Origin(xyz=(0.003, 0.0, 0.0035)),
        material=painted_metal,
        name="tab_stem",
    )
    lock_tab.visual(
        Box((0.016, 0.008, 0.0025)),
        origin=Origin(xyz=(0.011, 0.0, 0.0066)),
        material=painted_metal,
        name="tab_plate",
    )
    lock_tab.visual(
        Box((0.004, 0.008, 0.006)),
        origin=Origin(xyz=(0.018, 0.0, 0.0026)),
        material=painted_metal,
        name="tab_hook",
    )

    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(-0.043, 0.0, 0.0305)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(62.0),
        ),
    )

    model.articulation(
        "body_to_chip_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=chip_cover,
        origin=Origin(xyz=(-0.001, 0.0, 0.006)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )

    model.articulation(
        "body_to_lock_tab",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lock_tab,
        origin=Origin(xyz=(-0.055, 0.0, 0.031)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(80.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    handle = object_model.get_part("handle")
    chip_cover = object_model.get_part("chip_cover")
    lock_tab = object_model.get_part("lock_tab")
    handle_hinge = object_model.get_articulation("body_to_handle")
    cover_hinge = object_model.get_articulation("body_to_chip_cover")
    tab_hinge = object_model.get_articulation("body_to_lock_tab")

    with ctx.pose({handle_hinge: 0.0}):
        ctx.expect_gap(
            handle,
            body,
            axis="z",
            positive_elem="handle_grip",
            negative_elem="bridge_shell",
            min_gap=0.001,
            max_gap=0.020,
            name="handle grip rests just above the body bridge",
        )
        closed_handle_aabb = ctx.part_world_aabb(handle)

    with ctx.pose({handle_hinge: handle_hinge.motion_limits.upper}):
        open_handle_aabb = ctx.part_world_aabb(handle)

    ctx.check(
        "handle opens upward from the rear hinge",
        closed_handle_aabb is not None
        and open_handle_aabb is not None
        and open_handle_aabb[1][2] > closed_handle_aabb[1][2] + 0.025
        and open_handle_aabb[1][0] < closed_handle_aabb[1][0] - 0.015,
        details=f"closed={closed_handle_aabb}, open={open_handle_aabb}",
    )

    with ctx.pose({cover_hinge: 0.0}):
        ctx.expect_gap(
            body,
            chip_cover,
            axis="z",
            positive_elem="throat_block",
            negative_elem="cover_floor",
            min_gap=0.010,
            max_gap=0.018,
            name="chip cover sits just below the die opening",
        )
        closed_cover_aabb = ctx.part_world_aabb(chip_cover)

    with ctx.pose({cover_hinge: cover_hinge.motion_limits.upper}):
        open_cover_aabb = ctx.part_world_aabb(chip_cover)

    ctx.check(
        "chip cover swings downward to empty chads",
        closed_cover_aabb is not None
        and open_cover_aabb is not None
        and open_cover_aabb[0][2] < closed_cover_aabb[0][2] - 0.010,
        details=f"closed={closed_cover_aabb}, open={open_cover_aabb}",
    )

    with ctx.pose({tab_hinge: 0.0}):
        stowed_tab_aabb = ctx.part_world_aabb(lock_tab)

    with ctx.pose({tab_hinge: tab_hinge.motion_limits.upper}):
        latched_tab_aabb = ctx.part_world_aabb(lock_tab)

    ctx.check(
        "locking tab folds down from the rear shell",
        stowed_tab_aabb is not None
        and latched_tab_aabb is not None
        and latched_tab_aabb[0][2] < stowed_tab_aabb[0][2] - 0.010,
        details=f"stowed={stowed_tab_aabb}, latched={latched_tab_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
