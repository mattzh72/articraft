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


BODY_WIDTH = 0.118
BODY_HEIGHT = 0.118
BODY_DEPTH = 0.026
WALL_PLATE_DEPTH = 0.004
BODY_FRONT_Y = BODY_DEPTH * 0.5

RING_CENTER_Z = 0.014
RING_OUTER_RADIUS = 0.030
RING_INNER_RADIUS = 0.0215
RING_DEPTH = 0.005
RING_ORIGIN_Y = BODY_FRONT_Y + 0.0002 + (RING_DEPTH * 0.5)

COVER_WIDTH = 0.107
COVER_HEIGHT = 0.036
COVER_DEPTH = 0.0048
COVER_POCKET_DEPTH = 0.006
COVER_CENTER_Z = -0.0365
COVER_HINGE_Z = COVER_CENTER_Z - (COVER_HEIGHT * 0.5)
COVER_HINGE_Y = BODY_FRONT_Y - COVER_DEPTH


def _build_body_shell() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .box(BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)
        .edges("|Z")
        .fillet(0.007)
    )

    shell = (
        shell.faces(">Y")
        .workplane()
        .center(0.0, RING_CENTER_Z)
        .circle(RING_OUTER_RADIUS + 0.003)
        .cutBlind(-0.0012)
    )

    shell = (
        shell.faces(">Y")
        .workplane()
        .center(0.0, RING_CENTER_Z)
        .circle(RING_INNER_RADIUS - 0.001)
        .cutBlind(-0.0022)
    )

    shell = (
        shell.faces(">Y")
        .workplane()
        .center(0.0, COVER_CENTER_Z)
        .rect(COVER_WIDTH + 0.0015, COVER_HEIGHT + 0.0015)
        .cutBlind(-COVER_POCKET_DEPTH)
    )

    return shell


def _build_ring() -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .circle(RING_OUTER_RADIUS)
        .circle(RING_INNER_RADIUS)
        .extrude(RING_DEPTH * 0.5, both=True)
    )


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((float(mins[i]) + float(maxs[i])) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_thermostat")

    wall_plate = model.material("wall_plate", rgba=(0.12, 0.13, 0.14, 1.0))
    housing = model.material("housing", rgba=(0.93, 0.94, 0.92, 1.0))
    cover_finish = model.material("cover_finish", rgba=(0.88, 0.89, 0.87, 1.0))
    ring_finish = model.material("ring_finish", rgba=(0.28, 0.30, 0.33, 1.0))
    glass = model.material("glass", rgba=(0.14, 0.18, 0.21, 0.68))
    accent = model.material("accent", rgba=(0.77, 0.69, 0.44, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shell(), "thermostat_shell"),
        material=housing,
        name="shell",
    )
    body.visual(
        Box((0.128, WALL_PLATE_DEPTH, 0.128)),
        origin=Origin(xyz=(0.0, -0.015, 0.0)),
        material=wall_plate,
        name="wall_plate",
    )
    body.visual(
        Cylinder(radius=0.0185, length=0.0026),
        origin=Origin(
            xyz=(0.0, BODY_FRONT_Y - 0.0016, RING_CENTER_Z),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=glass,
        name="display",
    )
    body.visual(
        Box((0.0045, 0.0015, 0.010)),
        origin=Origin(
            xyz=(0.0, BODY_FRONT_Y + 0.00075, RING_CENTER_Z + RING_OUTER_RADIUS + 0.004)
        ),
        material=accent,
        name="setpoint_mark",
    )
    for index, x_pos in enumerate((-0.041, 0.041)):
        body.visual(
            Box((0.018, 0.009, 0.010)),
            origin=Origin(xyz=(x_pos, 0.0035, COVER_HINGE_Z - 0.0025)),
            material=housing,
            name=f"hinge_mount_{index}",
        )
        body.visual(
            Cylinder(radius=0.0028, length=0.022),
            origin=Origin(
                xyz=(x_pos, COVER_HINGE_Y, COVER_HINGE_Z),
                rpy=(0.0, math.pi * 0.5, 0.0),
            ),
            material=wall_plate,
            name=f"hinge_knuckle_{index}",
        )

    ring = model.part("ring")
    ring.visual(
        mesh_from_cadquery(_build_ring(), "thermostat_ring"),
        material=ring_finish,
        name="ring_band",
    )
    ring.visual(
        Box((0.008, 0.0015, 0.004)),
        origin=Origin(xyz=(0.0215, 0.0021, 0.015)),
        material=accent,
        name="index",
    )

    cover = model.part("cover")
    cover.visual(
        Box((COVER_WIDTH, COVER_DEPTH, COVER_HEIGHT)),
        origin=Origin(xyz=(0.0, COVER_DEPTH * 0.5, COVER_HEIGHT * 0.5)),
        material=cover_finish,
        name="cover_panel",
    )
    cover.visual(
        Box((0.060, 0.006, 0.005)),
        origin=Origin(xyz=(0.0, 0.0031, 0.0025)),
        material=accent,
        name="pull_lip",
    )
    cover.visual(
        Cylinder(radius=0.0028, length=0.050),
        origin=Origin(
            xyz=(0.0, 0.0006, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=wall_plate,
        name="hinge_barrel",
    )

    model.articulation(
        "body_to_ring",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=ring,
        origin=Origin(xyz=(0.0, RING_ORIGIN_Y, RING_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=5.0),
    )
    model.articulation(
        "body_to_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cover,
        origin=Origin(xyz=(0.0, COVER_HINGE_Y, COVER_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=2.5, lower=0.0, upper=1.95),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    ring = object_model.get_part("ring")
    cover = object_model.get_part("cover")
    ring_joint = object_model.get_articulation("body_to_ring")
    cover_joint = object_model.get_articulation("body_to_cover")

    ctx.expect_gap(
        ring,
        body,
        axis="y",
        min_gap=0.0001,
        max_gap=0.004,
        negative_elem="shell",
        name="ring floats just proud of the face",
    )
    ctx.expect_overlap(
        ring,
        body,
        axes="x",
        min_overlap=0.058,
        name="ring stays centered across thermostat width",
    )
    ctx.expect_overlap(
        ring,
        body,
        axes="z",
        min_overlap=0.058,
        name="ring stays centered across thermostat height",
    )

    with ctx.pose({cover_joint: 0.0}):
        ctx.expect_overlap(
            cover,
            body,
            axes="x",
            min_overlap=0.102,
            name="closed cover spans almost the full lower width",
        )
        ctx.expect_overlap(
            cover,
            body,
            axes="z",
            min_overlap=0.030,
            name="closed cover sits within the lower face region",
        )

    closed_cover = ctx.part_element_world_aabb(cover, elem="cover_panel")
    closed_cover_center = _aabb_center(closed_cover)

    with ctx.pose({cover_joint: 1.8}):
        open_cover = ctx.part_element_world_aabb(cover, elem="cover_panel")
        open_cover_center = _aabb_center(open_cover)
        ctx.check(
            "cover pulls down and outward",
            closed_cover_center is not None
            and open_cover_center is not None
            and open_cover_center[1] > closed_cover_center[1] + 0.012
            and open_cover_center[2] < closed_cover_center[2] - 0.02,
            details=f"closed_center={closed_cover_center}, open_center={open_cover_center}",
        )

    closed_index = ctx.part_element_world_aabb(ring, elem="index")
    closed_index_center = _aabb_center(closed_index)
    with ctx.pose({ring_joint: math.pi * 0.5}):
        turned_index = ctx.part_element_world_aabb(ring, elem="index")
        turned_index_center = _aabb_center(turned_index)
        ctx.check(
            "ring index moves when the dial turns",
            closed_index_center is not None
            and turned_index_center is not None
            and (
                abs(turned_index_center[0] - closed_index_center[0]) > 0.01
                or abs(turned_index_center[2] - closed_index_center[2]) > 0.01
            ),
            details=f"closed_index={closed_index_center}, turned_index={turned_index_center}",
        )

    return ctx.report()


object_model = build_object_model()
