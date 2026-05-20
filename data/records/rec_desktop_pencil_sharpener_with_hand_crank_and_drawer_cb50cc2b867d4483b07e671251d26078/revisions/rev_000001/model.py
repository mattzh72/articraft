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


BODY_X = 0.176
BODY_Y = 0.112
BODY_Z = 0.126
BODY_FRONT_X = BODY_X * 0.5

DRAWER_CENTER_Z = -0.020
DRAWER_TRAVEL = 0.056


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


def _drawer_mesh():
    drawer = (
        cq.Workplane("XY")
        .box(0.104, 0.084, 0.034)
        .translate((-0.052, 0.0, 0.0))
        .union(
            cq.Workplane("XY")
            .box(0.012, 0.094, 0.046)
            .translate((-0.006, 0.0, 0.0))
        )
    )

    drawer = drawer.cut(
        cq.Workplane("XY")
        .box(0.092, 0.072, 0.026)
        .translate((-0.054, 0.0, 0.006))
    )

    finger_pull = (
        cq.Workplane("XZ")
        .center(0.005, 0.010)
        .circle(0.014)
        .extrude(0.060, both=True)
    )
    drawer = drawer.cut(finger_pull)

    return mesh_from_cadquery(drawer, "sharpener_drawer")


def _dial_ring_mesh():
    ring = (
        cq.Workplane("YZ")
        .circle(0.022)
        .circle(0.0125)
        .extrude(0.007)
    )
    return mesh_from_cadquery(ring, "sharpener_dial_ring")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_sharpener")

    body_color = model.material("body_color", rgba=(0.16, 0.18, 0.20, 1.0))
    drawer_color = model.material("drawer_color", rgba=(0.22, 0.23, 0.25, 1.0))
    trim_color = model.material("trim_color", rgba=(0.62, 0.64, 0.67, 1.0))
    crank_color = model.material("crank_color", rgba=(0.13, 0.14, 0.15, 1.0))
    dial_color = model.material("dial_color", rgba=(0.13, 0.14, 0.15, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.072, BODY_Y, BODY_Z)),
        origin=Origin(xyz=(-0.052, 0.0, 0.0)),
        material=body_color,
        name="rear_shell",
    )
    body.visual(
        Box((0.104, BODY_Y, 0.062)),
        origin=Origin(xyz=(0.036, 0.0, 0.032)),
        material=body_color,
        name="top_shell",
    )
    body.visual(
        Box((0.104, BODY_Y, 0.018)),
        origin=Origin(xyz=(0.036, 0.0, -0.054)),
        material=body_color,
        name="bottom_shell",
    )
    for side_name, side_y in (("side_wall_0", 0.0505), ("side_wall_1", -0.0505)):
        body.visual(
            Box((0.104, 0.011, 0.046)),
            origin=Origin(xyz=(0.036, side_y, -0.021)),
            material=body_color,
            name=side_name,
        )
    body.visual(
        Cylinder(radius=0.022, length=0.008),
        origin=Origin(
            xyz=(0.006, BODY_Y * 0.5 + 0.004, 0.010),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim_color,
        name="side_hub",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(
            xyz=(BODY_FRONT_X + 0.002, 0.0, 0.026),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=trim_color,
        name="entry_socket",
    )

    drawer = model.part("drawer")
    drawer.visual(_drawer_mesh(), material=drawer_color, name="drawer_shell")

    dial = model.part("dial")
    dial.visual(_dial_ring_mesh(), material=dial_color, name="ring")
    dial.visual(
        Box((0.006, 0.004, 0.006)),
        origin=Origin(xyz=(0.005, 0.0, 0.019)),
        material=dial_color,
        name="pointer",
    )

    crank = model.part("crank")
    crank.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(
            xyz=(0.0, 0.006, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=crank_color,
        name="hub",
    )
    arm_start = (0.0, 0.006, 0.0)
    arm_end = (0.028, 0.006, -0.036)
    crank.visual(
        Cylinder(radius=0.004, length=_distance(arm_start, arm_end)),
        origin=Origin(
            xyz=_midpoint(arm_start, arm_end),
            rpy=_rpy_for_cylinder(arm_start, arm_end),
        ),
        material=crank_color,
        name="arm",
    )
    crank.visual(
        Cylinder(radius=0.005, length=0.018),
        origin=Origin(
            xyz=(arm_end[0], 0.015, arm_end[2]),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=crank_color,
        name="grip_stem",
    )
    crank.visual(
        Cylinder(radius=0.007, length=0.030),
        origin=Origin(
            xyz=(arm_end[0], 0.024, arm_end[2]),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=crank_color,
        name="grip",
    )

    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(BODY_FRONT_X, 0.0, DRAWER_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.16,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(BODY_FRONT_X, 0.0, 0.026)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=8.0),
    )
    model.articulation(
        "body_to_crank",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=crank,
        origin=Origin(xyz=(0.006, BODY_Y * 0.5 + 0.008, 0.010)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    dial = object_model.get_part("dial")
    crank = object_model.get_part("crank")
    drawer_joint = object_model.get_articulation("body_to_drawer")
    dial_joint = object_model.get_articulation("body_to_dial")
    crank_joint = object_model.get_articulation("body_to_crank")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    ctx.expect_within(
        drawer,
        body,
        axes="yz",
        margin=0.004,
        name="drawer stays laterally nested in the sharpener body",
    )
    ctx.expect_overlap(
        drawer,
        body,
        axes="x",
        min_overlap=0.060,
        name="closed drawer remains deeply inserted",
    )

    rest_pos = ctx.part_world_position(drawer)
    with ctx.pose({drawer_joint: DRAWER_TRAVEL}):
        ctx.expect_within(
            drawer,
            body,
            axes="yz",
            margin=0.004,
            name="extended drawer stays aligned with the body opening",
        )
        ctx.expect_overlap(
            drawer,
            body,
            axes="x",
            min_overlap=0.040,
            name="drawer retains insertion at full extension",
        )
        extended_pos = ctx.part_world_position(drawer)

    ctx.check(
        "drawer extends forward",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.045,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )
    ctx.expect_origin_gap(
        dial,
        body,
        axis="x",
        min_gap=0.085,
        max_gap=0.091,
        name="dial sits on the front face around the entry axis",
    )
    ctx.expect_origin_gap(
        crank,
        body,
        axis="y",
        min_gap=0.060,
        max_gap=0.068,
        name="crank sits on the right side hub",
    )

    dial_rest = _aabb_center(ctx.part_element_world_aabb(dial, elem="pointer"))
    with ctx.pose({dial_joint: 1.0}):
        dial_turned = _aabb_center(ctx.part_element_world_aabb(dial, elem="pointer"))
    ctx.check(
        "dial pointer rotates around the pencil entry axis",
        dial_rest is not None
        and dial_turned is not None
        and abs(dial_turned[1] - dial_rest[1]) > 0.008
        and abs(dial_turned[0] - dial_rest[0]) < 0.002,
        details=f"rest={dial_rest}, turned={dial_turned}",
    )

    crank_rest = _aabb_center(ctx.part_element_world_aabb(crank, elem="grip"))
    with ctx.pose({crank_joint: math.pi / 2.0}):
        crank_turned = _aabb_center(ctx.part_element_world_aabb(crank, elem="grip"))
    ctx.check(
        "crank grip swings around the side hub",
        crank_rest is not None
        and crank_turned is not None
        and crank_turned[2] > crank_rest[2] + 0.040
        and abs(crank_turned[1] - crank_rest[1]) < 0.004,
        details=f"rest={crank_rest}, turned={crank_turned}",
    )

    return ctx.report()


object_model = build_object_model()
