from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

BASE_W = 0.31
BASE_D = 0.23
BASE_T = 0.012

POST_W = 0.016
POST_D = 0.024
POST_H = 0.286
POST_X = 0.112
POST_Y = -0.076

CROSSHEAD_W = 0.286
CROSSHEAD_D = 0.036
CROSSHEAD_H = 0.034
SLIDE_HOME_Z = 0.190
SLIDE_LOWER = -0.045
SLIDE_UPPER = 0.075

HINGE_Y = 0.050
HINGE_Z = 0.010

TRAY_W = 0.292
TRAY_D = 0.238
TRAY_T = 0.0045
VENT_W = 0.118
VENT_D = 0.082
TRAY_REST_TILT = math.radians(18.0)
TRAY_TILT_LOWER = -math.radians(12.0)
TRAY_TILT_UPPER = math.radians(42.0)


def _base_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(BASE_W, BASE_D, BASE_T, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.008)
    )
    rear_beam = (
        cq.Workplane("XY")
        .box((2.0 * POST_X) + 0.052, 0.020, 0.018, centered=(True, True, False))
        .translate((0.0, POST_Y, BASE_T))
    )

    base = plate.union(rear_beam)
    for sign in (-1.0, 1.0):
        boss = (
            cq.Workplane("XY")
            .box(0.048, 0.040, 0.022, centered=(True, True, False))
            .translate((sign * POST_X, POST_Y, BASE_T))
        )
        base = base.union(boss)

    return base


def _crosshead_shape() -> cq.Workplane:
    carriage = (
        cq.Workplane("XY")
        .box(0.224, 0.020, 0.026)
        .translate((0.0, 0.032, 0.0))
    )
    hinge_block = cq.Workplane("XY").box(0.188, 0.018, 0.020).translate((0.0, 0.050, 0.0))
    carriage = carriage.union(hinge_block)
    center_knuckle = (
        cq.Workplane("YZ")
        .circle(0.0045)
        .extrude(0.053, both=True)
        .translate((0.0, HINGE_Y - 0.008, HINGE_Z - 0.005))
    )
    carriage = carriage.union(center_knuckle)

    for sign in (-1.0, 1.0):
        front_pad = (
            cq.Workplane("XY")
            .box(POST_W + 0.014, 0.012, CROSSHEAD_H)
            .translate((sign * POST_X, 0.018, 0.0))
        )
        outer_cheek = (
            cq.Workplane("XY")
            .box(0.006, 0.028, CROSSHEAD_H)
            .translate((sign * (POST_X + (POST_W / 2.0) + 0.003), 0.026, 0.0))
        )
        inner_cheek = (
            cq.Workplane("XY")
            .box(0.006, 0.028, CROSSHEAD_H)
            .translate((sign * (POST_X - (POST_W / 2.0) - 0.003), 0.026, 0.0))
        )
        carriage = carriage.union(front_pad).union(outer_cheek).union(inner_cheek)

    return carriage


def _tray_shape() -> cq.Workplane:
    tray = cq.Workplane("XY").box(TRAY_W, TRAY_D, TRAY_T, centered=(True, False, False))

    vent = (
        cq.Workplane("XY")
        .box(VENT_W, VENT_D, TRAY_T + 0.012, centered=(True, True, False))
        .translate((0.0, 0.128, -0.004))
    )
    tray = tray.cut(vent)

    side_rail = cq.Workplane("XY").box(0.009, 0.186, 0.014, centered=(True, False, False))
    tray = tray.union(
        side_rail.translate(((TRAY_W / 2.0) - 0.0045, 0.024, TRAY_T))
    ).union(
        side_rail.translate((-(TRAY_W / 2.0) + 0.0045, 0.024, TRAY_T))
    )

    front_lip = (
        cq.Workplane("XY")
        .box(0.198, 0.014, 0.017, centered=(True, False, False))
        .translate((0.0, TRAY_D - 0.014, TRAY_T))
    )
    rear_rib = (
        cq.Workplane("XY")
        .box(0.178, 0.020, 0.010, centered=(True, False, False))
        .translate((0.0, 0.0, TRAY_T))
    )
    tray = tray.union(front_lip).union(rear_rib)

    return tray.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), math.degrees(TRAY_REST_TILT))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="laptop_stand")

    graphite = model.material("graphite", rgba=(0.22, 0.24, 0.27, 1.0))
    satin_silver = model.material("satin_silver", rgba=(0.70, 0.72, 0.75, 1.0))
    tray_black = model.material("tray_black", rgba=(0.13, 0.14, 0.15, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_shape(), "laptop_stand_base"),
        material=graphite,
        name="deck",
    )
    for index, sign in enumerate((-1.0, 1.0)):
        base.visual(
            Box((POST_W, POST_D, POST_H)),
            origin=Origin(xyz=(sign * POST_X, POST_Y, BASE_T + 0.004 + (POST_H / 2.0))),
            material=graphite,
            name=f"post_{index}",
        )

    crosshead = model.part("crosshead")
    crosshead.visual(
        mesh_from_cadquery(_crosshead_shape(), "laptop_stand_crosshead"),
        material=satin_silver,
        name="carriage",
    )

    tray = model.part("tray")
    tray.visual(
        mesh_from_cadquery(_tray_shape(), "laptop_stand_tray"),
        material=tray_black,
        name="tray_shell",
    )

    slide = model.articulation(
        "base_to_crosshead",
        ArticulationType.PRISMATIC,
        parent=base,
        child=crosshead,
        origin=Origin(xyz=(0.0, POST_Y, SLIDE_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.18,
            lower=SLIDE_LOWER,
            upper=SLIDE_UPPER,
        ),
    )
    slide.meta["qc_samples"] = [SLIDE_LOWER, 0.0, SLIDE_UPPER]

    tilt = model.articulation(
        "crosshead_to_tray",
        ArticulationType.REVOLUTE,
        parent=crosshead,
        child=tray,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.6,
            lower=TRAY_TILT_LOWER,
            upper=TRAY_TILT_UPPER,
        ),
    )
    tilt.meta["qc_samples"] = [TRAY_TILT_LOWER, 0.0, TRAY_TILT_UPPER]

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    crosshead = object_model.get_part("crosshead")
    tray = object_model.get_part("tray")
    slide = object_model.get_articulation("base_to_crosshead")
    tilt = object_model.get_articulation("crosshead_to_tray")

    with ctx.pose({slide: SLIDE_LOWER, tilt: TRAY_TILT_LOWER}):
        tray_aabb = ctx.part_world_aabb(tray)
        tray_min_z = None if tray_aabb is None else tray_aabb[0][2]
        ctx.check(
            "tray clears the base plate at the lowest working pose",
            tray_min_z is not None and tray_min_z > 0.10,
            details=f"tray_min_z={tray_min_z}",
        )

    with ctx.pose({slide: 0.0}):
        tray_aabb = ctx.part_world_aabb(tray)
        tray_span = None if tray_aabb is None else tray_aabb[1][0] - tray_aabb[0][0]
        ctx.check(
            "shared tray spans both guide posts",
            tray_span is not None and tray_span > (2.0 * POST_X) + 0.04,
            details=f"tray_span={tray_span}",
        )

    with ctx.pose({slide: SLIDE_LOWER}):
        low_pos = ctx.part_world_position(crosshead)
    with ctx.pose({slide: SLIDE_UPPER}):
        high_pos = ctx.part_world_position(crosshead)
    ctx.check(
        "crosshead slides upward on the guide posts",
        low_pos is not None and high_pos is not None and high_pos[2] > low_pos[2] + 0.10,
        details=f"low={low_pos}, high={high_pos}",
    )

    with ctx.pose({tilt: TRAY_TILT_LOWER}):
        low_tray_aabb = ctx.part_world_aabb(tray)
    with ctx.pose({tilt: TRAY_TILT_UPPER}):
        high_tray_aabb = ctx.part_world_aabb(tray)
    low_front_z = None if low_tray_aabb is None else low_tray_aabb[1][2]
    high_front_z = None if high_tray_aabb is None else high_tray_aabb[1][2]
    ctx.check(
        "tray front rises with positive hinge motion",
        low_front_z is not None and high_front_z is not None and high_front_z > low_front_z + 0.08,
        details=f"low_front_z={low_front_z}, high_front_z={high_front_z}",
    )

    return ctx.report()


object_model = build_object_model()
