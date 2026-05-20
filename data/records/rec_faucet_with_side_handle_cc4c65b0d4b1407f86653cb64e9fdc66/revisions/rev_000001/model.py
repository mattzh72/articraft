from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_RADIUS = 0.029
BASE_HEIGHT = 0.008
BODY_WIDTH = 0.046
BODY_DEPTH = 0.050
BODY_HEIGHT = 0.046
COLLAR_OUTER_RADIUS = 0.0195
COLLAR_INNER_RADIUS = 0.0140
COLLAR_HEIGHT = 0.010
HANDLE_PAD_SIZE = (0.010, 0.020, 0.022)
HANDLE_PIVOT_X = BODY_WIDTH / 2.0 + HANDLE_PAD_SIZE[0] - 0.001
HANDLE_PIVOT_Z = BASE_HEIGHT + 0.032

SPOUT_OUTER = 0.026
SPOUT_INNER = 0.018
SPOUT_RISE = 0.166
SPOUT_BEND = 0.038
SPOUT_REACH = 0.198
SPOUT_INNER_REACH = 0.178
TIP_CENTER_Z = SPOUT_RISE + SPOUT_BEND
TIP_UNDERSIDE_Z = TIP_CENTER_Z - SPOUT_OUTER / 2.0

COVER_WIDTH = 0.022
COVER_LENGTH = 0.021
COVER_THICKNESS = 0.0022
COVER_HINGE_Y = SPOUT_INNER_REACH - 0.010
COVER_AXIS_Z = TIP_UNDERSIDE_Z - 0.0004


def _spout_path(reach: float) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .moveTo(0.0, 0.0)
        .lineTo(0.0, SPOUT_RISE)
        .threePointArc((SPOUT_BEND * 0.35, SPOUT_RISE + SPOUT_BEND * 0.72), (SPOUT_BEND, TIP_CENTER_Z))
        .lineTo(reach, TIP_CENTER_Z)
        .wire()
    )


def _body_shape() -> cq.Workplane:
    tower = (
        cq.Workplane("XY")
        .box(BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.0045)
        .translate((0.0, 0.0, BASE_HEIGHT))
    )
    base = cq.Workplane("XY").circle(BASE_RADIUS).extrude(BASE_HEIGHT)
    collar = (
        cq.Workplane("XY")
        .workplane(offset=BASE_HEIGHT + BODY_HEIGHT - 0.002)
        .circle(COLLAR_OUTER_RADIUS)
        .circle(COLLAR_INNER_RADIUS)
        .extrude(COLLAR_HEIGHT)
    )
    pad_x, pad_y, pad_z = HANDLE_PAD_SIZE
    handle_pad = (
        cq.Workplane("XY")
        .box(pad_x, pad_y, pad_z, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.0016)
        .translate((BODY_WIDTH / 2.0 + pad_x / 2.0 - 0.001, 0.0, HANDLE_PIVOT_Z - pad_z / 2.0))
    )
    return base.union(tower).union(collar).union(handle_pad)


def _spout_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").rect(SPOUT_OUTER, SPOUT_OUTER).sweep(_spout_path(SPOUT_REACH), transition="round")
    inner = cq.Workplane("XY").rect(SPOUT_INNER, SPOUT_INNER).sweep(
        _spout_path(SPOUT_INNER_REACH),
        transition="round",
    )
    spout = outer.cut(inner)

    recess = (
        cq.Workplane("XY")
        .box(COVER_WIDTH + 0.0015, COVER_LENGTH + 0.002, 0.006, centered=(True, False, False))
        .translate((0.0, COVER_HINGE_Y - 0.001, TIP_UNDERSIDE_Z - 0.001))
    )
    spout = spout.cut(recess)

    hinge_radius = 0.0023
    parent_knuckle_len = 0.006
    left_start = -SPOUT_OUTER / 2.0
    right_start = SPOUT_OUTER / 2.0 - parent_knuckle_len

    left_knuckle = (
        cq.Workplane("YZ")
        .center(COVER_HINGE_Y, COVER_AXIS_Z)
        .circle(hinge_radius)
        .extrude(parent_knuckle_len)
        .translate((left_start, 0.0, 0.0))
    )
    right_knuckle = (
        cq.Workplane("YZ")
        .center(COVER_HINGE_Y, COVER_AXIS_Z)
        .circle(hinge_radius)
        .extrude(parent_knuckle_len)
        .translate((right_start, 0.0, 0.0))
    )
    left_bridge = (
        cq.Workplane("XY")
        .box(parent_knuckle_len, 0.0045, 0.0035, centered=(False, True, False))
        .translate((left_start, COVER_HINGE_Y, COVER_AXIS_Z - 0.0012))
    )
    right_bridge = (
        cq.Workplane("XY")
        .box(parent_knuckle_len, 0.0045, 0.0035, centered=(False, True, False))
        .translate((right_start, COVER_HINGE_Y, COVER_AXIS_Z - 0.0012))
    )
    return spout.union(left_knuckle).union(right_knuckle).union(left_bridge).union(right_bridge)


def _handle_shape() -> cq.Workplane:
    pivot_barrel = cq.Workplane("YZ").circle(0.0042).extrude(0.010)
    root_block = (
        cq.Workplane("XY")
        .box(0.014, 0.016, 0.010, centered=(False, True, False))
        .translate((0.0, 0.0, -0.0045))
    )
    paddle = (
        cq.Workplane("XY")
        .box(0.011, 0.060, 0.006, centered=(False, False, False))
        .translate((0.010, 0.0, -0.0025))
    )
    tip = (
        cq.Workplane("XY")
        .box(0.0125, 0.020, 0.007, centered=(False, False, False))
        .translate((0.0095, 0.042, -0.0030))
    )
    return pivot_barrel.union(root_block).union(paddle).union(tip)


def _aerator_cover_shape() -> cq.Workplane:
    knuckle = cq.Workplane("YZ").circle(0.00195).extrude(0.012).translate((-0.006, 0.0, 0.0))
    hinge_tab = (
        cq.Workplane("XY")
        .box(0.012, 0.004, 0.003, centered=(True, False, False))
        .translate((0.0, 0.0, -0.0026))
    )
    plate = (
        cq.Workplane("XY")
        .box(COVER_WIDTH, COVER_LENGTH, COVER_THICKNESS, centered=(True, False, False))
        .translate((0.0, 0.001, -COVER_THICKNESS - 0.0004))
    )
    slot = (
        cq.Workplane("XY")
        .box(0.009, 0.008, 0.006, centered=(True, True, False))
        .translate((0.0, 0.013, -0.005))
    )
    return knuckle.union(hinge_tab).union(plate).cut(slot)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="modern_kitchen_faucet")

    chrome = model.material("chrome", rgba=(0.82, 0.84, 0.86, 1.0))
    graphite = model.material("graphite", rgba=(0.18, 0.18, 0.20, 1.0))

    body = model.part("body")
    body.visual(mesh_from_cadquery(_body_shape(), "body_shell"), material=chrome, name="body_shell")

    spout = model.part("spout")
    spout.visual(mesh_from_cadquery(_spout_shape(), "spout_shell"), material=chrome, name="spout_shell")

    handle = model.part("handle")
    handle.visual(mesh_from_cadquery(_handle_shape(), "handle_paddle"), material=graphite, name="handle_paddle")

    aerator_cover = model.part("aerator_cover")
    aerator_cover.visual(
        mesh_from_cadquery(_aerator_cover_shape(), "aerator_leaf"),
        material=chrome,
        name="aerator_leaf",
    )

    model.articulation(
        "body_to_spout",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT + BODY_HEIGHT + COLLAR_HEIGHT - 0.002)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5),
    )
    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(HANDLE_PIVOT_X, 0.0, HANDLE_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=0.0, upper=1.05),
    )
    model.articulation(
        "spout_to_aerator_cover",
        ArticulationType.REVOLUTE,
        parent=spout,
        child=aerator_cover,
        origin=Origin(xyz=(0.0, COVER_HINGE_Y, COVER_AXIS_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    spout = object_model.get_part("spout")
    handle = object_model.get_part("handle")
    aerator_cover = object_model.get_part("aerator_cover")

    spout_joint = object_model.get_articulation("body_to_spout")
    handle_joint = object_model.get_articulation("body_to_handle")
    cover_joint = object_model.get_articulation("spout_to_aerator_cover")

    ctx.expect_overlap(
        aerator_cover,
        spout,
        axes="xy",
        min_overlap=0.010,
        name="aerator cover sits within the spout tip footprint",
    )

    closed_spout = ctx.part_element_world_aabb(spout, elem="spout_shell")
    with ctx.pose({spout_joint: -math.pi / 2.0}):
        turned_spout = ctx.part_element_world_aabb(spout, elem="spout_shell")
    ctx.check(
        "spout rotates around the vertical body axis",
        closed_spout is not None
        and turned_spout is not None
        and closed_spout[1][1] > 0.18
        and closed_spout[1][0] < 0.05
        and turned_spout[1][0] > 0.18
        and turned_spout[1][1] < 0.08,
        details=f"closed={closed_spout}, turned={turned_spout}",
    )

    closed_handle = ctx.part_world_aabb(handle)
    with ctx.pose({handle_joint: 1.0}):
        opened_handle = ctx.part_world_aabb(handle)
    ctx.check(
        "side handle lifts on its short pivot",
        closed_handle is not None
        and opened_handle is not None
        and opened_handle[1][2] > closed_handle[1][2] + 0.02,
        details=f"closed={closed_handle}, opened={opened_handle}",
    )

    closed_cover = ctx.part_world_aabb(aerator_cover)
    with ctx.pose({cover_joint: 1.2}):
        opened_cover = ctx.part_world_aabb(aerator_cover)
    ctx.check(
        "aerator cover flips down from the spout tip",
        closed_cover is not None
        and opened_cover is not None
        and opened_cover[0][2] < closed_cover[0][2] - 0.010,
        details=f"closed={closed_cover}, opened={opened_cover}",
    )

    return ctx.report()


object_model = build_object_model()
