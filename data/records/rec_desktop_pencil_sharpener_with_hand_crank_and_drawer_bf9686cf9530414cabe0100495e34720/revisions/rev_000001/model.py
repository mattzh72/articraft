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


BODY_DEPTH = 0.125
BODY_WIDTH = 0.084
BODY_WALL = 0.0035
CRANK_X = 0.072
CRANK_Z = 0.056
CRANK_SHAFT_RADIUS = 0.0040
DRAWER_CENTER_Z = 0.021
DRAWER_OPENING_WIDTH = 0.062
DRAWER_OPENING_HEIGHT = 0.034
DRAWER_TRAVEL = 0.045
PENCIL_Z = 0.056
PENCIL_ENTRY_RADIUS = 0.0068
DIAL_OUTER_RADIUS = 0.018
DIAL_INNER_RADIUS = 0.0085
DIAL_THICKNESS = 0.005


def _body_shell_shape() -> cq.Workplane:
    outer_profile = [
        (0.0, 0.0),
        (0.0, 0.074),
        (0.018, 0.079),
        (0.078, 0.091),
        (BODY_DEPTH, 0.083),
        (BODY_DEPTH, 0.0),
    ]
    inner_profile = [
        (BODY_WALL, 0.004),
        (BODY_WALL, 0.070),
        (0.020, 0.074),
        (0.075, 0.084),
        (BODY_DEPTH - BODY_WALL, 0.076),
        (BODY_DEPTH - BODY_WALL, 0.004),
    ]

    shell = (
        cq.Workplane("XZ")
        .polyline(outer_profile)
        .close()
        .extrude(BODY_WIDTH * 0.5, both=True)
        .edges("|Y")
        .fillet(0.003)
    )

    bearing_boss = (
        cq.Workplane("XZ", origin=(0.0, BODY_WIDTH * 0.5, 0.0))
        .center(CRANK_X, CRANK_Z)
        .circle(0.010)
        .extrude(0.010)
    )
    shell = shell.union(bearing_boss)

    cavity = (
        cq.Workplane("XZ")
        .polyline(inner_profile)
        .close()
        .extrude((BODY_WIDTH - 2.0 * BODY_WALL) * 0.5, both=True)
    )
    shell = shell.cut(cavity)

    cutter_floor = cq.Workplane("XY").box(0.044, BODY_WIDTH - 2.0 * BODY_WALL, 0.004).translate(
        (0.034, 0.0, 0.039)
    )
    cutter_block = cq.Workplane("XY").box(0.020, 0.028, 0.022).translate((0.045, 0.0, 0.051))
    shell = shell.union(cutter_floor).union(cutter_block)

    drawer_opening = (
        cq.Workplane("YZ", origin=(-0.002, 0.0, 0.0))
        .center(0.0, DRAWER_CENTER_Z)
        .rect(DRAWER_OPENING_WIDTH, DRAWER_OPENING_HEIGHT)
        .extrude(0.024)
    )
    pencil_hole = (
        cq.Workplane("YZ", origin=(-0.003, 0.0, 0.0))
        .center(0.0, PENCIL_Z)
        .circle(PENCIL_ENTRY_RADIUS)
        .extrude(BODY_DEPTH + 0.012)
    )
    crank_hole = (
        cq.Workplane("XZ", origin=(0.0, BODY_WIDTH * 0.5 + 0.012, 0.0))
        .center(CRANK_X, CRANK_Z)
        .circle(CRANK_SHAFT_RADIUS + 0.0008)
        .extrude(-0.030)
    )

    return shell.cut(drawer_opening).cut(pencil_hole).cut(crank_hole)


def _suction_base_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .workplane(offset=-0.014)
        .circle(0.039)
        .workplane(offset=0.010)
        .circle(0.034)
        .workplane(offset=0.004)
        .circle(0.029)
        .loft(combine=True)
        .translate((0.067, 0.0, 0.0))
    )


def _dial_ring_shape() -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(DIAL_OUTER_RADIUS)
        .circle(DIAL_INNER_RADIUS)
        .extrude(DIAL_THICKNESS * 0.5, both=True)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="suction_base_pencil_sharpener")

    body_red = model.material("body_red", rgba=(0.67, 0.15, 0.12, 1.0))
    base_rubber = model.material("base_rubber", rgba=(0.14, 0.14, 0.15, 1.0))
    drawer_smoke = model.material("drawer_smoke", rgba=(0.30, 0.32, 0.36, 1.0))
    dial_cream = model.material("dial_cream", rgba=(0.90, 0.86, 0.76, 1.0))
    grip_black = model.material("grip_black", rgba=(0.13, 0.13, 0.14, 1.0))
    metal = model.material("metal", rgba=(0.72, 0.73, 0.75, 1.0))
    marker_dark = model.material("marker_dark", rgba=(0.08, 0.08, 0.09, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_shape(), "sharpener_body_shell"),
        material=body_red,
        name="body_shell",
    )
    body.visual(
        mesh_from_cadquery(_suction_base_shape(), "sharpener_suction_base"),
        material=base_rubber,
        name="suction_base",
    )
    body.visual(
        Box((0.002, 0.003, 0.008)),
        origin=Origin(xyz=(0.001, 0.0, PENCIL_Z + 0.019)),
        material=marker_dark,
        name="dial_marker",
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.004, 0.068, 0.038)),
        origin=Origin(xyz=(-0.002, 0.0, 0.0)),
        material=drawer_smoke,
        name="panel",
    )
    drawer.visual(
        Box((0.004, 0.020, 0.005)),
        origin=Origin(xyz=(-0.0045, 0.0, -0.004)),
        material=drawer_smoke,
        name="pull",
    )
    drawer.visual(
        Box((0.086, 0.054, 0.0025)),
        origin=Origin(xyz=(0.043, 0.0, -0.01075)),
        material=drawer_smoke,
        name="tray_floor",
    )
    drawer.visual(
        Box((0.086, 0.0025, 0.0215)),
        origin=Origin(xyz=(0.043, 0.02575, 0.00125)),
        material=drawer_smoke,
        name="tray_side_0",
    )
    drawer.visual(
        Box((0.086, 0.0025, 0.0215)),
        origin=Origin(xyz=(0.043, -0.02575, 0.00125)),
        material=drawer_smoke,
        name="tray_side_1",
    )
    drawer.visual(
        Box((0.0025, 0.054, 0.0215)),
        origin=Origin(xyz=(0.08475, 0.0, 0.00125)),
        material=drawer_smoke,
        name="tray_back",
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_cadquery(_dial_ring_shape(), "sharpener_point_dial"),
        origin=Origin(xyz=(-0.0045, 0.0, 0.0)),
        material=dial_cream,
        name="dial_ring",
    )
    dial.visual(
        Box((0.003, 0.004, 0.006)),
        origin=Origin(xyz=(-0.0045, 0.0, DIAL_OUTER_RADIUS - 0.0015)),
        material=dial_cream,
        name="dial_tab",
    )
    for index, (y_pos, z_pos) in enumerate(((0.0, 0.013), (0.010, -0.006), (-0.010, -0.006))):
        dial.visual(
            Box((0.0025, 0.004, 0.004)),
            origin=Origin(xyz=(-0.00125, y_pos, z_pos)),
            material=dial_cream,
            name=f"dial_pad_{index}",
        )

    crank = model.part("crank")
    crank.visual(
        Cylinder(radius=CRANK_SHAFT_RADIUS, length=0.016),
        origin=Origin(rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=metal,
        name="shaft",
    )
    crank.visual(
        Cylinder(radius=0.0095, length=0.004),
        origin=Origin(xyz=(0.0, 0.009, 0.0), rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=metal,
        name="hub",
    )
    crank.visual(
        Box((0.009, 0.006, 0.040)),
        origin=Origin(xyz=(-0.011, 0.011, -0.019), rpy=(0.0, 0.50, 0.0)),
        material=metal,
        name="arm",
    )
    crank.visual(
        Box((0.005, 0.006, 0.010)),
        origin=Origin(xyz=(-0.018, 0.011, -0.031)),
        material=metal,
        name="grip_post",
    )
    crank.visual(
        Cylinder(radius=0.005, length=0.020),
        origin=Origin(xyz=(-0.022, 0.011, -0.038), rpy=(0.0, -math.pi * 0.5, 0.0)),
        material=grip_black,
        name="grip",
    )

    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(0.0, 0.0, DRAWER_CENTER_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.18,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, 0.0, PENCIL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=8.0),
    )
    model.articulation(
        "body_to_crank",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=crank,
        origin=Origin(xyz=(CRANK_X, BODY_WIDTH * 0.5 + 0.005, CRANK_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    dial = object_model.get_part("dial")
    crank = object_model.get_part("crank")
    drawer_slide = object_model.get_articulation("body_to_drawer")
    dial_joint = object_model.get_articulation("body_to_dial")
    crank_joint = object_model.get_articulation("body_to_crank")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    with ctx.pose({drawer_slide: 0.0}):
        ctx.expect_within(
            drawer,
            body,
            axes="yz",
            inner_elem="tray_floor",
            outer_elem="body_shell",
            margin=0.0005,
            name="drawer tray stays centered in the shell",
        )
        ctx.expect_overlap(
            drawer,
            body,
            axes="x",
            elem_a="tray_floor",
            elem_b="body_shell",
            min_overlap=0.080,
            name="drawer remains deeply inserted when closed",
        )
        ctx.expect_gap(
            body,
            dial,
            axis="x",
            positive_elem="body_shell",
            negative_elem="dial_ring",
            min_gap=0.0010,
            max_gap=0.0030,
            name="dial stands visibly off the front shell",
        )

    drawer_rest = ctx.part_world_position(drawer)
    with ctx.pose({drawer_slide: DRAWER_TRAVEL}):
        ctx.expect_within(
            drawer,
            body,
            axes="yz",
            inner_elem="tray_floor",
            outer_elem="body_shell",
            margin=0.0005,
            name="drawer tray stays guided when opened",
        )
        ctx.expect_overlap(
            drawer,
            body,
            axes="x",
            elem_a="tray_floor",
            elem_b="body_shell",
            min_overlap=0.035,
            name="drawer retains insertion at full extension",
        )
        drawer_open = ctx.part_world_position(drawer)

    ctx.check(
        "drawer slides out from the front",
        drawer_rest is not None
        and drawer_open is not None
        and drawer_open[0] < drawer_rest[0] - 0.035,
        details=f"rest={drawer_rest}, open={drawer_open}",
    )

    dial_rest = _aabb_center(ctx.part_element_world_aabb(dial, elem="dial_tab"))
    with ctx.pose({dial_joint: math.pi * 0.5}):
        dial_turned = _aabb_center(ctx.part_element_world_aabb(dial, elem="dial_tab"))
    ctx.check(
        "dial rotates around the pencil-entry axis",
        dial_rest is not None
        and dial_turned is not None
        and abs(dial_turned[1] - dial_rest[1]) > 0.010
        and abs(dial_turned[2] - dial_rest[2]) > 0.010,
        details=f"rest={dial_rest}, turned={dial_turned}",
    )

    grip_rest = _aabb_center(ctx.part_element_world_aabb(crank, elem="grip"))
    with ctx.pose({crank_joint: math.pi * 0.5}):
        grip_turned = _aabb_center(ctx.part_element_world_aabb(crank, elem="grip"))
    ctx.check(
        "crank sweeps around the side shaft",
        grip_rest is not None
        and grip_turned is not None
        and grip_turned[2] > grip_rest[2] + 0.025
        and abs(grip_turned[0] - grip_rest[0]) > 0.010,
        details=f"rest={grip_rest}, turned={grip_turned}",
    )

    return ctx.report()


object_model = build_object_model()
