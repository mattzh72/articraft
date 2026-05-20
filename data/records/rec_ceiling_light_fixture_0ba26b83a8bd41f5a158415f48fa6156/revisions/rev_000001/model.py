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

TRIM_SIZE = 0.220
OPENING_SIZE = 0.148
TRIM_THICKNESS = 0.007
WELL_OUTER = 0.162
WELL_INNER = 0.148
WELL_DEPTH = 0.090

GIMBAL_OD = 0.132
GIMBAL_ID = 0.114
GIMBAL_RING_THICKNESS = 0.012
GIMBAL_ARM_OFFSET_Y = 0.050
GIMBAL_ARM_SIZE = (0.024, 0.014, 0.040)
GIMBAL_BOSS_RADIUS = 0.008
GIMBAL_BOSS_LENGTH = 0.014
GIMBAL_GUIDE_PAD_DEPTH = 0.010
GIMBAL_GUIDE_PAD_SPAN = 0.028
GIMBAL_GUIDE_PAD_THICKNESS = 0.016
GIMBAL_GUIDE_PAD_OFFSET = WELL_INNER / 2.0 - GIMBAL_GUIDE_PAD_DEPTH / 2.0
GIMBAL_PIVOT_Z = -0.018
GIMBAL_ORIGIN_Z = 0.081

LAMP_BODY_RADIUS = 0.046
LAMP_BODY_LENGTH = 0.048
LAMP_BEZEL_RADIUS = 0.051
LAMP_BEZEL_LENGTH = 0.010
LAMP_TRUNNION_RADIUS = 0.006
LAMP_TRUNNION_LENGTH = 0.086


def _build_frame_shape():
    # Adapted from the vent-shell pattern into a square recessed trim with a
    # deeper architectural light well instead of a flat grille housing.
    trim = (
        cq.Workplane("XY")
        .box(TRIM_SIZE, TRIM_SIZE, TRIM_THICKNESS)
        .translate((0.0, 0.0, TRIM_THICKNESS / 2.0))
    )
    trim = trim.cut(
        cq.Workplane("XY")
        .box(OPENING_SIZE, OPENING_SIZE, TRIM_THICKNESS + 0.004)
        .translate((0.0, 0.0, TRIM_THICKNESS / 2.0))
    )

    well_shell = (
        cq.Workplane("XY")
        .box(WELL_OUTER, WELL_OUTER, WELL_DEPTH)
        .translate((0.0, 0.0, TRIM_THICKNESS + WELL_DEPTH / 2.0))
    )
    well_shell = well_shell.cut(
        cq.Workplane("XY")
        .box(WELL_INNER, WELL_INNER, WELL_DEPTH + 0.006)
        .translate((0.0, 0.0, TRIM_THICKNESS + WELL_DEPTH / 2.0 + 0.002))
    )

    shadow_step = (
        cq.Workplane("XY")
        .box(OPENING_SIZE + 0.016, OPENING_SIZE + 0.016, 0.006)
        .translate((0.0, 0.0, TRIM_THICKNESS + 0.003))
    )
    shadow_step = shadow_step.cut(
        cq.Workplane("XY")
        .box(OPENING_SIZE, OPENING_SIZE, 0.010)
        .translate((0.0, 0.0, TRIM_THICKNESS + 0.003))
    )

    return trim.union(well_shell).union(shadow_step).findSolid()


def _build_gimbal_shape():
    ring = (
        cq.Workplane("XY")
        .circle(GIMBAL_OD / 2.0)
        .circle(GIMBAL_ID / 2.0)
        .extrude(GIMBAL_RING_THICKNESS)
        .translate((0.0, 0.0, -GIMBAL_RING_THICKNESS / 2.0))
    )

    for side in (-1.0, 1.0):
        arm = cq.Workplane("XY").box(*GIMBAL_ARM_SIZE).translate(
            (0.0, side * GIMBAL_ARM_OFFSET_Y, GIMBAL_PIVOT_Z)
        )
        boss = (
            cq.Workplane("XY")
            .circle(GIMBAL_BOSS_RADIUS)
            .extrude(GIMBAL_BOSS_LENGTH)
            .translate((0.0, 0.0, -GIMBAL_BOSS_LENGTH / 2.0))
            .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
            .translate((0.0, side * GIMBAL_ARM_OFFSET_Y, GIMBAL_PIVOT_Z))
        )
        ring = ring.union(arm).union(boss)

    for side in (-1.0, 1.0):
        ring = ring.union(
            cq.Workplane("XY").box(
                GIMBAL_GUIDE_PAD_DEPTH,
                GIMBAL_GUIDE_PAD_SPAN,
                GIMBAL_GUIDE_PAD_THICKNESS,
            ).translate((side * GIMBAL_GUIDE_PAD_OFFSET, 0.0, 0.0))
        )
        ring = ring.union(
            cq.Workplane("XY").box(
                GIMBAL_GUIDE_PAD_SPAN,
                GIMBAL_GUIDE_PAD_DEPTH,
                GIMBAL_GUIDE_PAD_THICKNESS,
            ).translate((0.0, side * GIMBAL_GUIDE_PAD_OFFSET, 0.0))
        )

    return ring.findSolid()


def _build_lamp_shape():
    body = (
        cq.Workplane("XY")
        .circle(LAMP_BODY_RADIUS)
        .extrude(LAMP_BODY_LENGTH)
        .translate((0.0, 0.0, -0.060))
    )
    body = body.cut(
        cq.Workplane("XY")
        .circle(0.037)
        .extrude(0.046)
        .translate((0.0, 0.0, -0.056))
    )

    bezel = (
        cq.Workplane("XY")
        .circle(LAMP_BEZEL_RADIUS)
        .circle(0.039)
        .extrude(LAMP_BEZEL_LENGTH)
        .translate((0.0, 0.0, -0.062))
    )
    knuckle = (
        cq.Workplane("XY")
        .circle(0.019)
        .extrude(0.014)
        .translate((0.0, 0.0, -0.010))
    )
    trunnion = (
        cq.Workplane("XY")
        .circle(LAMP_TRUNNION_RADIUS)
        .extrude(LAMP_TRUNNION_LENGTH)
        .translate((0.0, 0.0, -LAMP_TRUNNION_LENGTH / 2.0))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
    )
    return body.union(bezel).union(knuckle).union(trunnion).findSolid()


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="recessed_gimbal_ceiling_light")

    trim_white = model.material("trim_white", rgba=(0.95, 0.95, 0.93, 1.0))
    satin_graphite = model.material("satin_graphite", rgba=(0.23, 0.24, 0.26, 1.0))
    lamp_black = model.material("lamp_black", rgba=(0.11, 0.12, 0.13, 1.0))

    frame = model.part("frame")
    trim_rail = (TRIM_SIZE - OPENING_SIZE) / 2.0
    trim_center = OPENING_SIZE / 2.0 + trim_rail / 2.0
    for index, y in enumerate((-trim_center, trim_center)):
        frame.visual(
            Box((TRIM_SIZE, trim_rail, TRIM_THICKNESS)),
            origin=Origin(xyz=(0.0, y, TRIM_THICKNESS / 2.0)),
            material=trim_white,
            name=f"trim_{index}",
        )
    for index, x in enumerate((-trim_center, trim_center), start=2):
        frame.visual(
            Box((trim_rail, OPENING_SIZE, TRIM_THICKNESS)),
            origin=Origin(xyz=(x, 0.0, TRIM_THICKNESS / 2.0)),
            material=trim_white,
            name=f"trim_{index}",
        )

    step_rail = 0.008
    step_outer = OPENING_SIZE + 0.016
    step_center = OPENING_SIZE / 2.0 + step_rail / 2.0
    step_z = TRIM_THICKNESS + 0.003
    for index, y in enumerate((-step_center, step_center)):
        frame.visual(
            Box((step_outer, step_rail, 0.006)),
            origin=Origin(xyz=(0.0, y, step_z)),
            material=satin_graphite,
            name=f"step_{index}",
        )
    for index, x in enumerate((-step_center, step_center), start=2):
        frame.visual(
            Box((step_rail, OPENING_SIZE, 0.006)),
            origin=Origin(xyz=(x, 0.0, step_z)),
            material=satin_graphite,
            name=f"step_{index}",
        )

    wall_thickness = (WELL_OUTER - WELL_INNER) / 2.0
    wall_center = WELL_INNER / 2.0 + wall_thickness / 2.0
    wall_depth = WELL_DEPTH + 0.001
    wall_z = 0.006 + wall_depth / 2.0
    for index, y in enumerate((-wall_center, wall_center)):
        frame.visual(
            Box((WELL_OUTER, wall_thickness, wall_depth)),
            origin=Origin(xyz=(0.0, y, wall_z)),
            material=satin_graphite,
            name=f"wall_{index}",
        )
    for index, x in enumerate((-wall_center, wall_center), start=2):
        frame.visual(
            Box((wall_thickness, WELL_INNER, wall_depth)),
            origin=Origin(xyz=(x, 0.0, wall_z)),
            material=satin_graphite,
            name=f"wall_{index}",
        )

    gimbal = model.part("gimbal")
    gimbal.visual(
        mesh_from_cadquery(_build_gimbal_shape(), "gimbal_ring"),
        material=satin_graphite,
        name="gimbal_ring",
    )

    lamp = model.part("lamp")
    lamp.visual(
        Cylinder(radius=LAMP_BODY_RADIUS, length=LAMP_BODY_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, -0.044)),
        material=lamp_black,
        name="lamp_body",
    )
    lamp.visual(
        Cylinder(radius=0.024, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
        material=lamp_black,
        name="lamp_neck",
    )
    lamp.visual(
        Cylinder(radius=LAMP_BEZEL_RADIUS, length=LAMP_BEZEL_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, -0.057)),
        material=lamp_black,
        name="lamp_bezel",
    )
    lamp.visual(
        Cylinder(radius=0.019, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=lamp_black,
        name="lamp_knuckle",
    )
    lamp.visual(
        Cylinder(radius=LAMP_TRUNNION_RADIUS, length=LAMP_TRUNNION_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=lamp_black,
        name="lamp_trunnion",
    )
    lamp.visual(
        Cylinder(radius=0.037, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.054)),
        material=satin_graphite,
        name="lamp_aperture",
    )

    model.articulation(
        "frame_to_gimbal",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=gimbal,
        origin=Origin(xyz=(0.0, 0.0, GIMBAL_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=4.0),
    )
    model.articulation(
        "gimbal_to_lamp",
        ArticulationType.REVOLUTE,
        parent=gimbal,
        child=lamp,
        origin=Origin(xyz=(0.0, 0.0, GIMBAL_PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=-math.radians(38.0),
            upper=math.radians(38.0),
        ),
    )

    return model


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    low, high = aabb
    return tuple((low[i] + high[i]) / 2.0 for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    gimbal = object_model.get_part("gimbal")
    lamp = object_model.get_part("lamp")
    yaw = object_model.get_articulation("frame_to_gimbal")
    tilt = object_model.get_articulation("gimbal_to_lamp")

    ctx.expect_origin_distance(
        gimbal,
        frame,
        axes="xy",
        max_dist=0.001,
        name="gimbal rotation axis stays centered in the trim",
    )
    ctx.expect_origin_distance(
        lamp,
        gimbal,
        axes="xy",
        max_dist=0.001,
        name="lamp hinge stays on the gimbal centerline",
    )
    ctx.allow_overlap(
        gimbal,
        lamp,
        elem_a="gimbal_ring",
        elem_b="lamp_trunnion",
        reason="The lamp is intentionally carried on a trunnion axle captured inside the gimbal bearing bosses.",
    )

    frame_aabb = ctx.part_world_aabb(frame)
    lamp_aabb = ctx.part_element_world_aabb(lamp, elem="lamp_bezel")
    frame_front_z = frame_aabb[0][2] if frame_aabb is not None else None
    lamp_front_z = lamp_aabb[0][2] if lamp_aabb is not None else None
    ctx.check(
        "lamp sits recessed behind the trim plane",
        frame_front_z is not None
        and lamp_front_z is not None
        and lamp_front_z >= frame_front_z + 0.0005
        and lamp_front_z <= frame_front_z + 0.018,
        details=f"frame_front_z={frame_front_z}, lamp_front_z={lamp_front_z}",
    )

    rest_center = _aabb_center(lamp_aabb)
    with ctx.pose({tilt: math.radians(30.0)}):
        tilted_center = _aabb_center(ctx.part_element_world_aabb(lamp, elem="lamp_bezel"))
    with ctx.pose({yaw: math.pi / 2.0, tilt: math.radians(30.0)}):
        spun_center = _aabb_center(ctx.part_element_world_aabb(lamp, elem="lamp_bezel"))

    tilt_dx = None if rest_center is None or tilted_center is None else tilted_center[0] - rest_center[0]
    tilt_dy = None if rest_center is None or tilted_center is None else tilted_center[1] - rest_center[1]
    spin_dx = None if rest_center is None or spun_center is None else spun_center[0] - rest_center[0]
    spin_dy = None if rest_center is None or spun_center is None else spun_center[1] - rest_center[1]

    ctx.check(
        "tilt articulation aims the lamp off axis",
        tilt_dx is not None and tilt_dy is not None and abs(tilt_dx) > 0.008 and abs(tilt_dy) < 0.006,
        details=f"rest_center={rest_center}, tilted_center={tilted_center}",
    )
    ctx.check(
        "continuous rotation redirects the tilt azimuth",
        spin_dx is not None
        and spin_dy is not None
        and abs(spin_dy) > 0.008
        and abs(spin_dx) < max(0.006, abs(tilt_dx or 0.0) * 0.55),
        details=f"rest_center={rest_center}, tilted_center={tilted_center}, spun_center={spun_center}",
    )

    return ctx.report()


object_model = build_object_model()
