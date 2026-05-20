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


BASE_DIAMETER = 0.108
BASE_TOP_DIAMETER = 0.084
BASE_HEIGHT = 0.018

BODY_LENGTH = 0.112
BODY_WIDTH = 0.068
BODY_Z0 = 0.014
BODY_LOWER_HEIGHT = 0.054
BODY_TOP_HEIGHT = 0.020
BODY_HEIGHT = BODY_LOWER_HEIGHT + BODY_TOP_HEIGHT
BODY_FRONT_X = BODY_LENGTH / 2.0

DRAWER_OPENING_WIDTH = 0.050
DRAWER_OPENING_HEIGHT = 0.024
DRAWER_CENTER_Z = BODY_Z0 + 0.027
DRAWER_TRAVEL = 0.034

PENCIL_HOLE_RADIUS = 0.0054
PENCIL_HOLE_Z = BODY_Z0 + 0.051

HANDLE_SHAFT_RADIUS = 0.004
HANDLE_X = -0.004
HANDLE_Z = BODY_Z0 + 0.044
HANDLE_Y = BODY_WIDTH / 2.0 - 0.0035

LEVER_PIVOT_X = 0.024
LEVER_PIVOT_Z = -0.004


def _body_shape() -> cq.Workplane:
    suction_base = (
        cq.Workplane("XY")
        .circle(BASE_DIAMETER / 2.0)
        .workplane(offset=BASE_HEIGHT)
        .circle(BASE_TOP_DIAMETER / 2.0)
        .loft(combine=True)
    )

    floor = cq.Workplane("XY").box(0.086, BODY_WIDTH - 0.010, 0.004, centered=(True, True, False)).translate((0.0, 0.0, BODY_Z0 + 0.008))
    rear_wall = cq.Workplane("XY").box(0.010, BODY_WIDTH - 0.004, 0.058, centered=(True, True, False)).translate((-0.051, 0.0, BODY_Z0 + 0.008))
    side_wall = cq.Workplane("XY").box(0.094, 0.007, 0.058, centered=(True, True, False)).translate((0.0, 0.0, BODY_Z0 + 0.008))
    left_wall = side_wall.translate((0.0, 0.0305, 0.0))
    right_wall = side_wall.translate((0.0, -0.0305, 0.0))

    front_frame = cq.Workplane("XY").box(0.010, BODY_WIDTH, 0.060, centered=(True, True, False)).translate((0.051, 0.0, BODY_Z0 + 0.008))
    frame_drawer_opening = (
        cq.Workplane("YZ")
        .workplane(offset=0.051 + 0.006)
        .center(0.0, DRAWER_CENTER_Z)
        .rect(DRAWER_OPENING_WIDTH, DRAWER_OPENING_HEIGHT)
        .extrude(-0.014)
    )
    frame_pencil_hole = (
        cq.Workplane("YZ")
        .workplane(offset=0.051 + 0.006)
        .center(0.0, PENCIL_HOLE_Z)
        .circle(PENCIL_HOLE_RADIUS)
        .extrude(-0.016)
    )
    frame_bezel = (
        cq.Workplane("YZ")
        .workplane(offset=0.051 + 0.006)
        .center(0.0, PENCIL_HOLE_Z)
        .circle(0.008)
        .extrude(-0.003)
    )
    front_frame = front_frame.cut(frame_drawer_opening).cut(frame_bezel).cut(frame_pencil_hole)

    roof = cq.Workplane("XY").box(0.092, BODY_WIDTH - 0.014, 0.007, centered=(True, True, False)).translate((0.0, 0.0, BODY_Z0 + 0.062))
    top_cap = (
        cq.Workplane("XY")
        .workplane(offset=BODY_Z0 + 0.062)
        .rect(0.092, BODY_WIDTH - 0.014)
        .workplane(offset=0.016)
        .rect(0.074, BODY_WIDTH - 0.026)
        .loft(combine=True)
    )

    shaft_boss = (
        cq.Workplane("XZ")
        .workplane(offset=BODY_WIDTH / 2.0 + 0.004)
        .center(HANDLE_X, HANDLE_Z)
        .circle(0.010)
        .extrude(-0.008)
    )

    lug_y = 0.010
    lug_shape = (
        cq.Workplane("XY")
        .center(LEVER_PIVOT_X, 0.0)
        .workplane(offset=-0.006)
        .box(0.010, 0.006, 0.010, centered=(True, True, False))
    )
    lever_lugs = lug_shape.translate((0.0, lug_y, 0.0)).union(lug_shape.translate((0.0, -lug_y, 0.0)))

    body = (
        suction_base.union(floor)
        .union(rear_wall)
        .union(left_wall)
        .union(right_wall)
        .union(front_frame)
        .union(roof)
        .union(top_cap)
        .union(shaft_boss)
        .union(lever_lugs)
    )

    side_bore = (
        cq.Workplane("XZ")
        .workplane(offset=BODY_WIDTH / 2.0 + 0.005)
        .center(HANDLE_X, HANDLE_Z)
        .circle(HANDLE_SHAFT_RADIUS + 0.0012)
        .extrude(-0.014)
    )
    return body.cut(side_bore)


def _drawer_panel_shape() -> cq.Workplane:
    panel = cq.Workplane("XY").box(0.004, 0.058, 0.032, centered=(False, True, True))
    pull = (
        cq.Workplane("XZ")
        .workplane(offset=0.0)
        .center(0.002, 0.007)
        .circle(0.008)
        .extrude(0.070)
    )
    return panel.cut(pull)


def _drawer_tray_shape() -> cq.Workplane:
    tray_outer = cq.Workplane("XY").box(0.052, 0.048, 0.021, centered=(True, True, True)).translate((-0.025, 0.0, -0.002))
    tray_inner = cq.Workplane("XY").box(0.046, 0.042, 0.018, centered=(True, True, True)).translate((-0.026, 0.0, 0.002))
    return tray_outer.cut(tray_inner)


def _base_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(BASE_DIAMETER / 2.0)
        .workplane(offset=BASE_HEIGHT)
        .circle(BASE_TOP_DIAMETER / 2.0)
        .loft(combine=True)
    )


def _front_frame_shape() -> cq.Workplane:
    frame = cq.Workplane("XY").box(0.010, BODY_WIDTH, 0.060, centered=(True, True, True))
    drawer_opening = (
        cq.Workplane("YZ")
        .workplane(offset=0.006)
        .center(0.0, DRAWER_CENTER_Z - 0.046)
        .rect(DRAWER_OPENING_WIDTH, DRAWER_OPENING_HEIGHT)
        .extrude(-0.014)
    )
    pencil_hole = (
        cq.Workplane("YZ")
        .workplane(offset=0.006)
        .center(0.0, PENCIL_HOLE_Z - 0.046)
        .circle(PENCIL_HOLE_RADIUS)
        .extrude(-0.016)
    )
    bezel = (
        cq.Workplane("YZ")
        .workplane(offset=0.006)
        .center(0.0, PENCIL_HOLE_Z - 0.046)
        .circle(0.008)
        .extrude(-0.003)
    )
    return frame.cut(drawer_opening).cut(bezel).cut(pencil_hole)


def _pencil_panel_shape() -> cq.Workplane:
    panel = cq.Workplane("XY").box(0.010, 0.040, 0.018, centered=(True, True, True))
    hole = (
        cq.Workplane("YZ")
        .workplane(offset=0.006)
        .circle(PENCIL_HOLE_RADIUS)
        .extrude(-0.016)
    )
    bezel = (
        cq.Workplane("YZ")
        .workplane(offset=0.006)
        .circle(0.008)
        .extrude(-0.003)
    )
    return panel.cut(bezel).cut(hole)


def _right_wall_shape() -> cq.Workplane:
    wall = cq.Workplane("XY").box(0.094, 0.007, 0.058, centered=(True, True, True))
    boss = (
        cq.Workplane("XZ")
        .workplane(offset=0.004)
        .center(HANDLE_X, HANDLE_Z - 0.045)
        .circle(0.010)
        .extrude(-0.008)
    )
    bore = (
        cq.Workplane("XZ")
        .workplane(offset=0.005)
        .center(HANDLE_X, HANDLE_Z - 0.045)
        .circle(HANDLE_SHAFT_RADIUS + 0.0001)
        .extrude(-0.014)
    )
    return wall.union(boss).cut(bore)


def _handle_shape() -> cq.Workplane:
    shaft = cq.Workplane("XY").circle(HANDLE_SHAFT_RADIUS).extrude(0.030).rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0).translate((0.0, -0.015, 0.0))
    arm = cq.Workplane("XY").box(0.048, 0.008, 0.006, centered=(False, False, True)).translate((0.0, 0.011, 0.0))
    drop = cq.Workplane("XY").box(0.008, 0.008, 0.024, centered=(False, False, False)).translate((0.040, 0.011, -0.024))
    grip = cq.Workplane("XY").circle(0.0055).extrude(0.022).rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0).translate((0.044, 0.008, -0.024))
    return shaft.union(arm).union(drop).union(grip)


def _lever_shape() -> cq.Workplane:
    barrel = cq.Workplane("XY").circle(0.0035).extrude(0.012).rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0).translate((0.0, -0.006, 0.0))
    arm = cq.Workplane("XY").box(0.046, 0.010, 0.006, centered=(False, True, True)).translate((-0.046, 0.0, -0.004))
    toe = cq.Workplane("XY").box(0.020, 0.018, 0.006, centered=(True, True, True)).translate((-0.044, 0.0, -0.004))
    return barrel.union(arm).union(toe)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="suction_base_pencil_sharpener")

    body_plastic = model.material("body_plastic", rgba=(0.83, 0.16, 0.17, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.17, 0.17, 0.18, 1.0))
    drawer_plastic = model.material("drawer_plastic", rgba=(0.84, 0.84, 0.86, 1.0))
    handle_grip = model.material("handle_grip", rgba=(0.10, 0.10, 0.11, 1.0))
    lever_plastic = model.material("lever_plastic", rgba=(0.09, 0.09, 0.10, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_base_shape(), "sharpener_base"),
        material=dark_plastic,
        name="base_shell",
    )
    body.visual(
        Box((0.086, BODY_WIDTH - 0.010, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=body_plastic,
        name="floor",
    )
    body.visual(
        Box((0.010, BODY_WIDTH - 0.004, 0.058)),
        origin=Origin(xyz=(-0.051, 0.0, 0.045)),
        material=body_plastic,
        name="rear_wall",
    )
    body.visual(
        Box((0.094, 0.007, 0.058)),
        origin=Origin(xyz=(0.0, -0.0305, 0.045)),
        material=body_plastic,
        name="left_wall",
    )
    body.visual(
        mesh_from_cadquery(_right_wall_shape(), "sharpener_right_wall"),
        origin=Origin(xyz=(0.0, 0.0305, 0.045)),
        material=body_plastic,
        name="right_wall",
    )
    body.visual(
        Box((0.010, BODY_WIDTH, 0.012)),
        origin=Origin(xyz=(0.051, 0.0, 0.022)),
        material=body_plastic,
        name="front_lower",
    )
    body.visual(
        Box((0.010, 0.009, 0.024)),
        origin=Origin(xyz=(0.051, 0.0295, 0.041)),
        material=body_plastic,
        name="body_shell",
    )
    body.visual(
        Box((0.010, 0.009, 0.024)),
        origin=Origin(xyz=(0.051, -0.0295, 0.041)),
        material=body_plastic,
        name="front_side_1",
    )
    body.visual(
        Box((0.010, BODY_WIDTH, 0.012)),
        origin=Origin(xyz=(0.051, 0.0, 0.059)),
        material=body_plastic,
        name="front_bridge",
    )
    body.visual(
        mesh_from_cadquery(_pencil_panel_shape(), "sharpener_pencil_panel"),
        origin=Origin(xyz=(0.051, 0.0, PENCIL_HOLE_Z)),
        material=body_plastic,
        name="pencil_panel",
    )
    body.visual(
        Box((0.010, 0.014, 0.018)),
        origin=Origin(xyz=(0.051, 0.027, PENCIL_HOLE_Z)),
        material=body_plastic,
        name="front_upper_side_0",
    )
    body.visual(
        Box((0.010, 0.014, 0.018)),
        origin=Origin(xyz=(0.051, -0.027, PENCIL_HOLE_Z)),
        material=body_plastic,
        name="front_upper_side_1",
    )
    body.visual(
        Box((0.092, BODY_WIDTH - 0.014, 0.007)),
        origin=Origin(xyz=(0.0, 0.0, 0.0775)),
        material=body_plastic,
        name="roof",
    )
    body.visual(
        Box((0.084, BODY_WIDTH - 0.018, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=body_plastic,
        name="top_cap",
    )
    body.visual(
        Box((0.010, 0.006, 0.010)),
        origin=Origin(xyz=(LEVER_PIVOT_X, 0.010, -0.001)),
        material=dark_plastic,
        name="lever_lug_0",
    )
    body.visual(
        Box((0.010, 0.006, 0.010)),
        origin=Origin(xyz=(LEVER_PIVOT_X, -0.010, -0.001)),
        material=dark_plastic,
        name="lever_lug_1",
    )

    drawer = model.part("drawer")
    drawer.visual(
        mesh_from_cadquery(_drawer_panel_shape(), "drawer_panel"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_plastic,
        name="front_panel",
    )
    drawer.visual(
        mesh_from_cadquery(_drawer_tray_shape(), "drawer_tray"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=drawer_plastic,
        name="tray",
    )

    crank = model.part("crank")
    crank.visual(
        Cylinder(radius=HANDLE_SHAFT_RADIUS, length=0.030),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=handle_grip,
        name="shaft",
    )
    crank.visual(
        Cylinder(radius=0.007, length=0.003),
        origin=Origin(xyz=(0.0, 0.0055, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="hub",
    )
    crank.visual(
        Box((0.048, 0.008, 0.006)),
        origin=Origin(xyz=(0.024, 0.015, 0.0)),
        material=dark_plastic,
        name="arm",
    )
    crank.visual(
        Box((0.008, 0.008, 0.024)),
        origin=Origin(xyz=(0.044, 0.015, -0.012)),
        material=dark_plastic,
        name="drop",
    )
    crank.visual(
        Cylinder(radius=0.0055, length=0.022),
        origin=Origin(xyz=(0.044, 0.019, -0.024), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=handle_grip,
        name="handle_mesh",
    )

    lever = model.part("lever")
    lever.visual(
        Cylinder(radius=0.0035, length=0.014),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=lever_plastic,
        name="pivot_barrel",
    )
    lever.visual(
        Box((0.046, 0.010, 0.006)),
        origin=Origin(xyz=(-0.023, 0.0, -0.004)),
        material=lever_plastic,
        name="arm",
    )
    lever.visual(
        Box((0.020, 0.018, 0.006)),
        origin=Origin(xyz=(-0.044, 0.0, -0.004)),
        material=lever_plastic,
        name="lever_mesh",
    )

    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(BODY_FRONT_X, 0.0, DRAWER_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.20, lower=0.0, upper=DRAWER_TRAVEL),
    )
    model.articulation(
        "body_to_crank",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=crank,
        origin=Origin(xyz=(HANDLE_X, HANDLE_Y, HANDLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=10.0),
    )
    model.articulation(
        "body_to_lever",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever,
        origin=Origin(xyz=(LEVER_PIVOT_X, 0.0, LEVER_PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=0.0, upper=1.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    crank = object_model.get_part("crank")
    lever = object_model.get_part("lever")

    drawer_joint = object_model.get_articulation("body_to_drawer")
    crank_joint = object_model.get_articulation("body_to_crank")
    lever_joint = object_model.get_articulation("body_to_lever")

    drawer_limits = drawer_joint.motion_limits
    lever_limits = lever_joint.motion_limits

    with ctx.pose({drawer_joint: 0.0}):
        ctx.expect_gap(
            drawer,
            body,
            axis="x",
            positive_elem="front_panel",
            negative_elem="body_shell",
            min_gap=0.0,
            max_gap=0.003,
            name="drawer front sits nearly flush with the housing",
        )
        ctx.expect_within(
            drawer,
            body,
            axes="yz",
            margin=0.006,
            name="closed drawer stays centered within the body envelope",
        )
        ctx.expect_overlap(
            drawer,
            body,
            axes="x",
            min_overlap=0.030,
            name="closed drawer remains deeply inserted",
        )

    if drawer_limits is not None and drawer_limits.upper is not None:
        rest_panel_aabb = ctx.part_element_world_aabb(drawer, elem="front_panel")
        rest_pos = ctx.part_world_position(drawer)
        with ctx.pose({drawer_joint: drawer_limits.upper}):
            extended_panel_aabb = ctx.part_element_world_aabb(drawer, elem="front_panel")
            extended_pos = ctx.part_world_position(drawer)
            ctx.expect_within(
                drawer,
                body,
                axes="yz",
                margin=0.006,
                name="extended drawer stays aligned with the housing",
            )
            ctx.expect_overlap(
                drawer,
                body,
                axes="x",
                min_overlap=0.015,
                name="extended drawer keeps retained insertion",
            )
        ctx.check(
            "drawer slides outward from the body",
            rest_pos is not None
            and extended_pos is not None
            and extended_panel_aabb is not None
            and rest_panel_aabb is not None
            and extended_pos[0] > rest_pos[0] + 0.020
            and extended_panel_aabb[0][0] > rest_panel_aabb[0][0] + 0.020,
            details=f"rest_pos={rest_pos}, extended_pos={extended_pos}, rest_panel={rest_panel_aabb}, extended_panel={extended_panel_aabb}",
        )

    crank_aabb_0 = ctx.part_element_world_aabb(crank, elem="handle_mesh")
    with ctx.pose({crank_joint: math.pi / 2.0}):
        crank_aabb_90 = ctx.part_element_world_aabb(crank, elem="handle_mesh")
    ctx.check(
        "side crank rotates about its shaft",
        crank_aabb_0 is not None
        and crank_aabb_90 is not None
        and abs(crank_aabb_90[1][0] - crank_aabb_0[1][0]) > 0.012
        and abs(crank_aabb_90[0][2] - crank_aabb_0[0][2]) > 0.012,
        details=f"rest={crank_aabb_0}, quarter_turn={crank_aabb_90}",
    )

    if lever_limits is not None and lever_limits.upper is not None:
        lever_aabb_0 = ctx.part_element_world_aabb(lever, elem="lever_mesh")
        with ctx.pose({lever_joint: lever_limits.upper}):
            lever_aabb_open = ctx.part_element_world_aabb(lever, elem="lever_mesh")
        ctx.check(
            "base lock lever swings down from its tucked position",
            lever_aabb_0 is not None
            and lever_aabb_open is not None
            and lever_aabb_open[0][2] < lever_aabb_0[0][2] - 0.010,
            details=f"tucked={lever_aabb_0}, open={lever_aabb_open}",
        )

    return ctx.report()


object_model = build_object_model()
