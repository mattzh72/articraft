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


BODY_DEPTH = 0.110
BODY_WIDTH = 0.072
BODY_HEIGHT = 0.128
BODY_WALL = 0.004
BODY_FRONT_WALL = 0.010
BODY_REAR_WALL = 0.006
BODY_TOP_WALL = 0.005
BODY_BOTTOM_WALL = 0.004

DRAWER_TRAVEL = 0.052
DRAWER_WIDTH = 0.056
DRAWER_HEIGHT = 0.029
DRAWER_DEPTH = 0.083

PORT_RADIUS = 0.0065
FRONT_BOSS_RADIUS = 0.014
FRONT_BOSS_LENGTH = 0.006
AXLE_BOSS_RADIUS = 0.012
AXLE_BOSS_LENGTH = 0.006


def _midpoint(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _build_housing_shell():
    shell = (
        cq.Workplane("XY")
        .box(BODY_DEPTH, BODY_WIDTH, BODY_HEIGHT, centered=(True, True, False))
        .edges("|Z").fillet(0.007)
        .edges(">Z").fillet(0.005)
    )

    inner_depth = BODY_DEPTH - BODY_FRONT_WALL - BODY_REAR_WALL
    inner_width = BODY_WIDTH - 2.0 * BODY_WALL
    inner_height = BODY_HEIGHT - BODY_TOP_WALL - BODY_BOTTOM_WALL
    inner_x = (BODY_REAR_WALL - BODY_FRONT_WALL) * 0.5
    cavity = (
        cq.Workplane("XY")
        .box(inner_depth, inner_width, inner_height, centered=(True, True, False))
        .translate((inner_x, 0.0, BODY_BOTTOM_WALL))
    )
    shell = shell.cut(cavity)

    drawer_opening = (
        cq.Workplane("YZ")
        .workplane(offset=BODY_DEPTH * 0.5)
        .rect(DRAWER_WIDTH + 0.002, DRAWER_HEIGHT + 0.003)
        .extrude(-0.022)
        .translate((0.0, 0.0, 0.018))
    )
    shell = shell.cut(drawer_opening)

    pencil_port = (
        cq.Workplane("YZ")
        .workplane(offset=BODY_DEPTH * 0.5 + FRONT_BOSS_LENGTH + 0.001)
        .circle(PORT_RADIUS)
        .extrude(-(FRONT_BOSS_LENGTH + BODY_FRONT_WALL + 0.020))
        .translate((0.0, 0.0, 0.086))
    )
    shell = shell.cut(pencil_port)

    dial_clearance = (
        cq.Workplane("YZ")
        .workplane(offset=BODY_DEPTH * 0.5 + FRONT_BOSS_LENGTH + 0.001)
        .circle(FRONT_BOSS_RADIUS + 0.0025)
        .extrude(-0.003)
        .translate((0.0, 0.0, 0.086))
    )
    shell = shell.cut(dial_clearance)

    return shell


def _build_selector_ring():
    ring_thickness = 0.0055
    outer_radius = 0.0205
    inner_radius = 0.0155
    ring = (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(ring_thickness)
        .translate((-ring_thickness * 0.5, 0.0, 0.0))
    )
    return ring


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="classroom_pencil_sharpener")

    cast_metal = model.material("cast_metal", rgba=(0.27, 0.30, 0.33, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.21, 1.0))
    drawer_black = model.material("drawer_black", rgba=(0.16, 0.16, 0.17, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.67, 0.69, 0.72, 1.0))
    knob_black = model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))
    accent = model.material("accent", rgba=(0.80, 0.76, 0.62, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_housing_shell(), "sharpener_housing"),
        material=cast_metal,
        name="housing_shell",
    )
    body.visual(
        Cylinder(radius=AXLE_BOSS_RADIUS, length=AXLE_BOSS_LENGTH),
        origin=Origin(
            xyz=(0.010, BODY_WIDTH * 0.5 + AXLE_BOSS_LENGTH * 0.5, 0.072),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=dark_steel,
        name="side_boss",
    )
    body.visual(
        Box((0.050, 0.056, 0.004)),
        origin=Origin(xyz=(-0.010, 0.0, 0.002)),
        material=dark_steel,
        name="base_pad",
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((DRAWER_DEPTH, 0.051, 0.0025)),
        origin=Origin(xyz=(-DRAWER_DEPTH * 0.5, 0.0, 0.00125)),
        material=drawer_black,
        name="tray_floor",
    )
    drawer.visual(
        Box((DRAWER_DEPTH, 0.0022, 0.026)),
        origin=Origin(xyz=(-DRAWER_DEPTH * 0.5, DRAWER_WIDTH * 0.5 - 0.0011, 0.013)),
        material=drawer_black,
        name="tray_side_0",
    )
    drawer.visual(
        Box((DRAWER_DEPTH, 0.0022, 0.026)),
        origin=Origin(xyz=(-DRAWER_DEPTH * 0.5, -DRAWER_WIDTH * 0.5 + 0.0011, 0.013)),
        material=drawer_black,
        name="tray_side_1",
    )
    drawer.visual(
        Box((0.0025, DRAWER_WIDTH, 0.026)),
        origin=Origin(xyz=(-DRAWER_DEPTH + 0.00125, 0.0, 0.013)),
        material=drawer_black,
        name="tray_rear",
    )
    drawer.visual(
        Box((0.0025, DRAWER_WIDTH, 0.026)),
        origin=Origin(xyz=(-0.00125, 0.0, 0.013)),
        material=drawer_black,
        name="tray_front",
    )
    drawer.visual(
        Box((0.004, DRAWER_WIDTH + 0.002, DRAWER_HEIGHT + 0.001)),
        origin=Origin(xyz=(0.002, 0.0, DRAWER_HEIGHT * 0.5)),
        material=cast_metal,
        name="front_panel",
    )
    drawer.visual(
        Cylinder(radius=0.0045, length=0.030),
        origin=Origin(
            xyz=(0.0065, 0.0, DRAWER_HEIGHT * 0.56),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=satin_steel,
        name="pull_handle",
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_cadquery(_build_selector_ring(), "sharpener_selector_ring"),
        material=dark_steel,
        name="selector_ring",
    )
    dial.visual(
        Box((0.0055, 0.009, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=accent,
        name="thumb_tab",
    )

    crank = model.part("crank")
    crank.visual(
        Cylinder(radius=0.011, length=0.010),
        origin=Origin(xyz=(0.0, 0.005, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=dark_steel,
        name="hub",
    )

    arm_a = (0.0, 0.014, 0.0)
    arm_b = (0.022, 0.022, -0.036)
    crank.visual(
        Cylinder(radius=0.0055, length=_distance(arm_a, arm_b)),
        origin=Origin(xyz=_midpoint(arm_a, arm_b), rpy=_rpy_for_cylinder(arm_a, arm_b)),
        material=dark_steel,
        name="arm",
    )
    stem_a = (0.022, 0.020, -0.036)
    stem_b = (0.026, 0.021, -0.040)
    crank.visual(
        Cylinder(radius=0.0048, length=_distance(stem_a, stem_b)),
        origin=Origin(
            xyz=_midpoint(stem_a, stem_b),
            rpy=_rpy_for_cylinder(stem_a, stem_b),
        ),
        material=dark_steel,
        name="knob_bridge",
    )
    crank.visual(
        Cylinder(radius=0.0032, length=0.012),
        origin=Origin(
            xyz=(0.026, 0.025, -0.040),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=satin_steel,
        name="knob_stem",
    )

    knob = model.part("knob")
    knob.visual(
        Cylinder(radius=0.008, length=0.018),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=knob_black,
        name="grip",
    )
    knob.visual(
        Box((0.004, 0.014, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=accent,
        name="grip_ridge",
    )

    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(BODY_DEPTH * 0.5, 0.0, BODY_BOTTOM_WALL + 0.001)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.10,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(BODY_DEPTH * 0.5 + 0.00275, 0.0, 0.086)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=4.0),
    )
    model.articulation(
        "body_to_crank",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=crank,
        origin=Origin(xyz=(0.010, BODY_WIDTH * 0.5 + AXLE_BOSS_LENGTH, 0.072)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=8.0),
    )
    model.articulation(
        "crank_to_knob",
        ArticulationType.CONTINUOUS,
        parent=crank,
        child=knob,
        origin=Origin(xyz=(0.026, 0.040, -0.040)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=10.0),
    )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[index] + upper[index]) * 0.5 for index in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    dial = object_model.get_part("dial")
    crank = object_model.get_part("crank")
    knob = object_model.get_part("knob")

    drawer_joint = object_model.get_articulation("body_to_drawer")
    dial_joint = object_model.get_articulation("body_to_dial")
    crank_joint = object_model.get_articulation("body_to_crank")
    knob_joint = object_model.get_articulation("crank_to_knob")

    ctx.expect_gap(
        drawer,
        body,
        axis="x",
        positive_elem="front_panel",
        negative_elem="housing_shell",
        min_gap=-0.001,
        max_gap=0.006,
        name="drawer front sits near the housing front when closed",
    )
    ctx.expect_within(
        drawer,
        body,
        axes="yz",
        margin=0.010,
        name="drawer stays centered within the housing opening",
    )
    ctx.expect_contact(
        crank,
        body,
        elem_a="hub",
        elem_b="side_boss",
        contact_tol=0.0008,
        name="crank hub stays supported by the side axle boss",
    )
    ctx.expect_contact(
        knob,
        crank,
        elem_a="grip",
        elem_b="knob_stem",
        contact_tol=0.0008,
        name="crank knob is mounted on the handle stem",
    )

    drawer_rest = ctx.part_world_position(drawer)
    with ctx.pose({drawer_joint: DRAWER_TRAVEL}):
        ctx.expect_gap(
            drawer,
            body,
            axis="x",
            positive_elem="front_panel",
            negative_elem="housing_shell",
            min_gap=0.045,
            name="drawer pulls forward out of the housing",
        )
        ctx.expect_within(
            drawer,
            body,
            axes="yz",
            margin=0.010,
            name="drawer remains aligned while extended",
        )
        drawer_open = ctx.part_world_position(drawer)

    ctx.check(
        "drawer origin moves forward along the slide axis",
        drawer_rest is not None
        and drawer_open is not None
        and drawer_open[0] > drawer_rest[0] + 0.045,
        details=f"rest={drawer_rest}, open={drawer_open}",
    )

    thumb_rest = _aabb_center(ctx.part_element_world_aabb(dial, elem="thumb_tab"))
    with ctx.pose({dial_joint: math.pi * 0.5}):
        thumb_turn = _aabb_center(ctx.part_element_world_aabb(dial, elem="thumb_tab"))
    ctx.check(
        "selector dial visibly rotates around the pencil port",
        thumb_rest is not None
        and thumb_turn is not None
        and math.hypot(thumb_turn[1] - thumb_rest[1], thumb_turn[2] - thumb_rest[2]) > 0.010,
        details=f"rest={thumb_rest}, turned={thumb_turn}",
    )

    knob_rest = ctx.part_world_position(knob)
    with ctx.pose({crank_joint: math.pi * 0.5}):
        knob_crank_turn = ctx.part_world_position(knob)
    ctx.check(
        "hand crank swings the knob tip through a real circular path",
        knob_rest is not None
        and knob_crank_turn is not None
        and math.hypot(knob_crank_turn[0] - knob_rest[0], knob_crank_turn[2] - knob_rest[2]) > 0.035,
        details=f"rest={knob_rest}, turned={knob_crank_turn}",
    )

    ridge_rest = _aabb_center(ctx.part_element_world_aabb(knob, elem="grip_ridge"))
    with ctx.pose({knob_joint: math.pi * 0.5}):
        ridge_spin = _aabb_center(ctx.part_element_world_aabb(knob, elem="grip_ridge"))
    ctx.check(
        "crank knob itself spins on its own axle",
        ridge_rest is not None
        and ridge_spin is not None
        and math.hypot(ridge_spin[0] - ridge_rest[0], ridge_spin[2] - ridge_rest[2]) > 0.006,
        details=f"rest={ridge_rest}, spun={ridge_spin}",
    )

    return ctx.report()


object_model = build_object_model()
