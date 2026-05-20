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


BASE_SIZE = 0.120
BASE_THICKNESS = 0.014
POST_RADIUS = 0.012
POST_HEIGHT = 0.070
POST_HEAD_RADIUS = 0.020
POST_HEAD_HEIGHT = 0.014
PAN_Z = BASE_THICKNESS + POST_HEIGHT + POST_HEAD_HEIGHT

YOKE_ARM_Y = 0.051
YOKE_ARM_THICKNESS = 0.010
TILT_Z = 0.055
YOKE_ARM_HEIGHT = 0.068

CAN_OUTER_RADIUS = 0.045
CAN_INNER_RADIUS = 0.040
CAN_CENTER_Z = 0.040
CAN_START_X = -0.010
CAN_LENGTH = 0.110
CAN_FRONT_X = CAN_START_X + CAN_LENGTH
LENS_CENTER_X = 0.093
LENS_RADIUS = 0.040
TRUNNION_RADIUS = 0.0085
TRUNNION_BOSS_RADIUS = 0.009
TRUNNION_BOSS_LENGTH = 0.028

COVER_HINGE_X = CAN_FRONT_X + 0.0045
COVER_HINGE_Y = 0.044
COVER_OUTER_RADIUS = 0.046
COVER_GLASS_RADIUS = 0.034


def _make_yoke_shape():
    hub = (
        cq.Workplane("XY")
        .circle(0.018)
        .extrude(0.014)
    )
    cross = (
        cq.Workplane("XY")
        .center(-0.010, 0.0)
        .box(0.024, 0.094, 0.012, centered=(True, True, False))
    )
    left_arm = (
        cq.Workplane("XY")
        .center(0.0, 0.052)
        .box(0.014, 0.012, YOKE_ARM_HEIGHT, centered=(True, True, False))
    )
    right_arm = (
        cq.Workplane("XY")
        .center(0.0, -0.052)
        .box(0.014, 0.012, YOKE_ARM_HEIGHT, centered=(True, True, False))
    )
    return hub.union(cross).union(left_arm).union(right_arm).val()


def _make_can_shape():
    shell_tube = (
        cq.Workplane("YZ")
        .workplane(offset=CAN_START_X + 0.007)
        .center(0.0, CAN_CENTER_Z)
        .circle(CAN_OUTER_RADIUS)
        .circle(CAN_INNER_RADIUS)
        .extrude(CAN_LENGTH - 0.011)
    )
    rear_wall = (
        cq.Workplane("YZ")
        .workplane(offset=CAN_START_X)
        .center(0.0, CAN_CENTER_Z)
        .circle(CAN_OUTER_RADIUS)
        .extrude(0.007)
    )
    front_bezel = (
        cq.Workplane("YZ")
        .workplane(offset=CAN_FRONT_X - 0.014)
        .center(0.0, CAN_CENTER_Z)
        .circle(0.049)
        .circle(CAN_INNER_RADIUS + 0.001)
        .extrude(0.014)
    )
    rear_cap = (
        cq.Workplane("YZ")
        .workplane(offset=CAN_START_X - 0.012)
        .center(0.0, CAN_CENTER_Z)
        .circle(0.031)
        .extrude(0.012)
    )
    left_trunnion_boss = (
        cq.Workplane("XZ")
        .workplane(offset=0.018)
        .center(0.0, 0.0)
        .circle(TRUNNION_BOSS_RADIUS)
        .extrude(TRUNNION_BOSS_LENGTH)
    )
    right_trunnion_boss = (
        cq.Workplane("XZ")
        .workplane(offset=-(0.018 + TRUNNION_BOSS_LENGTH))
        .center(0.0, 0.0)
        .circle(TRUNNION_BOSS_RADIUS)
        .extrude(TRUNNION_BOSS_LENGTH)
    )
    hinge_leaf = (
        cq.Workplane("XY")
        .center(CAN_FRONT_X - 0.006, COVER_HINGE_Y + 0.002)
        .box(0.014, 0.006, 0.030)
        .translate((0.0, 0.0, CAN_CENTER_Z))
    )
    lower_hinge_pad = (
        cq.Workplane("XY")
        .center(CAN_FRONT_X + 0.001, COVER_HINGE_Y + 0.002)
        .workplane(offset=-0.015)
        .circle(0.004)
        .extrude(0.010)
        .translate((0.0, 0.0, CAN_CENTER_Z))
    )
    upper_hinge_pad = (
        cq.Workplane("XY")
        .center(CAN_FRONT_X + 0.001, COVER_HINGE_Y + 0.002)
        .workplane(offset=0.005)
        .circle(0.004)
        .extrude(0.010)
        .translate((0.0, 0.0, CAN_CENTER_Z))
    )
    body = rear_wall.union(shell_tube.val())
    body = body.union(front_bezel.val())
    body = body.union(rear_cap.val())
    body = body.union(left_trunnion_boss.val())
    body = body.union(right_trunnion_boss.val())
    body = body.union(hinge_leaf.val())
    body = body.union(lower_hinge_pad.val())
    body = body.union(upper_hinge_pad.val())
    return body.val()


def _make_cover_shape():
    ring = (
        cq.Workplane("YZ")
        .workplane(offset=0.001)
        .center(-COVER_HINGE_Y, 0.0)
        .circle(COVER_OUTER_RADIUS)
        .circle(COVER_GLASS_RADIUS)
        .extrude(0.004)
    )
    hinge_barrel = (
        cq.Workplane("XY")
        .workplane(offset=-0.013)
        .circle(0.004)
        .extrude(0.026)
    )
    strap = (
        cq.Workplane("XY")
        .center(0.002, -0.004)
        .box(0.004, 0.012, 0.020)
    )
    latch_tab = (
        cq.Workplane("XY")
        .center(0.002, -0.086)
        .box(0.004, 0.010, 0.014)
    )
    return ring.union(hinge_barrel).union(strap).union(latch_tab)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_spotlight")

    matte_black = model.material("matte_black", rgba=(0.10, 0.10, 0.11, 1.0))
    satin_black = model.material("satin_black", rgba=(0.16, 0.16, 0.18, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.26, 0.27, 0.29, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.58, 0.72, 0.84, 0.38))

    base = model.part("base")
    base.visual(
        Box((BASE_SIZE, BASE_SIZE, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material=dark_steel,
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=POST_RADIUS, length=POST_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + POST_HEIGHT / 2.0)),
        material=satin_black,
        name="support_post",
    )
    base.visual(
        Cylinder(radius=POST_HEAD_RADIUS, length=POST_HEAD_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + POST_HEIGHT + POST_HEAD_HEIGHT / 2.0)),
        material=matte_black,
        name="pan_head",
    )

    yoke = model.part("yoke")
    yoke.visual(mesh_from_cadquery(_make_yoke_shape(), "yoke_frame"), material=dark_steel, name="yoke_frame")

    can = model.part("can")
    can.visual(mesh_from_cadquery(_make_can_shape(), "lamp_can"), material=matte_black, name="can_shell")
    can.visual(
        Cylinder(radius=LENS_RADIUS, length=0.004),
        origin=Origin(xyz=(LENS_CENTER_X, 0.0, CAN_CENTER_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material=lens_glass,
        name="front_lens",
    )

    cover = model.part("cover")
    cover.visual(mesh_from_cadquery(_make_cover_shape(), "lens_cover"), material=dark_steel, name="cover_frame")
    cover.visual(
        Cylinder(radius=COVER_GLASS_RADIUS, length=0.0015),
        origin=Origin(xyz=(0.003, -COVER_HINGE_Y, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=lens_glass,
        name="cover_glass",
    )

    model.articulation(
        "pan",
        ArticulationType.REVOLUTE,
        parent=base,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, PAN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-2.2, upper=2.2, effort=18.0, velocity=2.2),
    )
    model.articulation(
        "tilt",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=can,
        origin=Origin(xyz=(0.0, 0.0, TILT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.75, upper=1.10, effort=14.0, velocity=2.0),
    )
    model.articulation(
        "cover_hinge",
        ArticulationType.REVOLUTE,
        parent=can,
        child=cover,
        origin=Origin(xyz=(COVER_HINGE_X, COVER_HINGE_Y, CAN_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.45, effort=4.0, velocity=2.8),
    )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    lo, hi = aabb
    return tuple((a + b) / 2.0 for a, b in zip(lo, hi))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    can = object_model.get_part("can")
    cover = object_model.get_part("cover")
    pan = object_model.get_articulation("pan")
    tilt = object_model.get_articulation("tilt")
    cover_hinge = object_model.get_articulation("cover_hinge")

    ctx.allow_overlap(
        can,
        "yoke",
        elem_a="can_shell",
        elem_b="yoke_frame",
        reason="The lamp can uses simplified trunnion bosses seated into the yoke pivot pads instead of fully modeled drilled hinge hardware.",
    )
    ctx.expect_origin_gap(
        "yoke",
        base,
        axis="z",
        min_gap=0.090,
        max_gap=0.105,
        name="yoke mounts at the top of the support post",
    )
    ctx.expect_gap(
        cover,
        can,
        axis="x",
        positive_elem="cover_glass",
        negative_elem="front_lens",
        min_gap=0.006,
        max_gap=0.014,
        name="closed lens cover sits just ahead of the front lens",
    )
    ctx.expect_overlap(
        cover,
        can,
        axes="yz",
        elem_a="cover_glass",
        elem_b="front_lens",
        min_overlap=0.060,
        name="closed cover remains centered over the lens",
    )

    rest_lens = _aabb_center(ctx.part_element_world_aabb(can, elem="front_lens"))
    with ctx.pose({tilt: 0.90}):
        tilted_lens = _aabb_center(ctx.part_element_world_aabb(can, elem="front_lens"))
    ctx.check(
        "lamp can tilts upward",
        rest_lens is not None and tilted_lens is not None and tilted_lens[2] > rest_lens[2] + 0.050,
        details=f"rest={rest_lens}, tilted={tilted_lens}",
    )

    with ctx.pose({pan: 1.10}):
        panned_lens = _aabb_center(ctx.part_element_world_aabb(can, elem="front_lens"))
    ctx.check(
        "yoke pans around the vertical post",
        rest_lens is not None and panned_lens is not None and abs(panned_lens[1] - rest_lens[1]) > 0.070,
        details=f"rest={rest_lens}, panned={panned_lens}",
    )

    closed_cover = _aabb_center(ctx.part_element_world_aabb(cover, elem="cover_glass"))
    with ctx.pose({cover_hinge: 1.20}):
        open_cover = _aabb_center(ctx.part_element_world_aabb(cover, elem="cover_glass"))
        ctx.expect_gap(
            cover,
            can,
            axis="x",
            positive_elem="cover_glass",
            negative_elem="front_lens",
            min_gap=0.018,
            name="opened cover swings clear of the lens",
        )
    ctx.check(
        "lens cover opens on the can side hinge",
        closed_cover is not None
        and open_cover is not None
        and open_cover[0] > closed_cover[0] + 0.025
        and open_cover[1] > closed_cover[1] + 0.020,
        details=f"closed={closed_cover}, open={open_cover}",
    )

    return ctx.report()


object_model = build_object_model()
