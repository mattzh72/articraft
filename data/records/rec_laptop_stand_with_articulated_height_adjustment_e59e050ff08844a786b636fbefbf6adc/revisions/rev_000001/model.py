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


BASE_WIDTH = 0.340
BASE_DEPTH = 0.260
BASE_THICKNESS = 0.015
REAR_BEAM_WIDTH = 0.220
REAR_BEAM_DEPTH = 0.050
REAR_BEAM_HEIGHT = 0.040
ROD_RADIUS = 0.009
ROD_SPACING = 0.170
ROD_Y = 0.090
ROD_LENGTH = 0.330
SLIDE_REST_Z = 0.053
SLIDE_TRAVEL = 0.160
TRAY_NEUTRAL_TILT = -0.32


def _build_carriage_mesh() -> object:
    sleeve_outer = 0.018
    sleeve_length = 0.105
    rod_clearance = 0.0020

    left_sleeve = (
        cq.Workplane("XY")
        .center(-ROD_SPACING * 0.5, 0.0)
        .circle(sleeve_outer)
        .extrude(sleeve_length)
    )
    right_sleeve = (
        cq.Workplane("XY")
        .center(ROD_SPACING * 0.5, 0.0)
        .circle(sleeve_outer)
        .extrude(sleeve_length)
    )

    bridge = cq.Workplane("XY").box(0.228, 0.038, 0.050).translate((0.0, -0.010, 0.046))
    foot = cq.Workplane("XY").box(0.060, 0.020, 0.022).translate((0.0, -0.010, 0.011))
    pedestal = cq.Workplane("XY").box(0.086, 0.042, 0.046).translate((0.0, -0.032, 0.072))
    yoke_bridge = cq.Workplane("XY").box(0.042, 0.020, 0.014).translate((0.0, -0.040, 0.084))
    left_ear = cq.Workplane("XY").box(0.016, 0.018, 0.020).translate((-0.034, -0.040, 0.096))
    right_ear = cq.Workplane("XY").box(0.016, 0.018, 0.020).translate((0.034, -0.040, 0.096))

    carriage = left_sleeve.union(right_sleeve)
    for feature in (bridge, foot, pedestal, yoke_bridge, left_ear, right_ear):
        carriage = carriage.union(feature)

    left_hole = (
        cq.Workplane("XY")
        .center(-ROD_SPACING * 0.5, 0.0)
        .circle(ROD_RADIUS + rod_clearance)
        .extrude(sleeve_length + 0.006)
        .translate((0.0, 0.0, -0.003))
    )
    right_hole = (
        cq.Workplane("XY")
        .center(ROD_SPACING * 0.5, 0.0)
        .circle(ROD_RADIUS + rod_clearance)
        .extrude(sleeve_length + 0.006)
        .translate((0.0, 0.0, -0.003))
    )

    carriage = carriage.cut(left_hole).cut(right_hole)
    return carriage


def _build_tray_mesh() -> object:
    panel = cq.Workplane("XY").box(0.300, 0.220, 0.008).translate((0.0, -0.124, 0.017))
    front_stop = cq.Workplane("XY").box(0.288, 0.022, 0.024).translate((0.0, -0.220, 0.019))
    left_rail = cq.Workplane("XY").box(0.012, 0.202, 0.018).translate((-0.144, -0.124, 0.018))
    right_rail = cq.Workplane("XY").box(0.012, 0.202, 0.018).translate((0.144, -0.124, 0.018))
    hinge_mount = cq.Workplane("XY").box(0.074, 0.032, 0.026).translate((0.0, -0.022, 0.010))

    tray_deck = panel
    for feature in (front_stop, left_rail, right_rail, hinge_mount):
        tray_deck = tray_deck.union(feature)
    tray_deck = tray_deck.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), math.degrees(TRAY_NEUTRAL_TILT))

    hinge_barrel = (
        cq.Workplane("XY")
        .circle(0.012)
        .extrude(0.052)
        .translate((0.0, 0.0, -0.026))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
    )

    return tray_deck.union(hinge_barrel)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="laptop_stand")

    charcoal = model.material("charcoal", rgba=(0.19, 0.20, 0.22, 1.0))
    graphite = model.material("graphite", rgba=(0.28, 0.29, 0.31, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.72, 0.74, 1.0))
    satin_black = model.material("satin_black", rgba=(0.12, 0.13, 0.14, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.09, 0.10, 1.0))

    base = model.part("base")
    base.visual(
        Box((BASE_WIDTH, BASE_DEPTH, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS * 0.5)),
        material=charcoal,
        name="base_slab",
    )
    base.visual(
        Box((REAR_BEAM_WIDTH, REAR_BEAM_DEPTH, REAR_BEAM_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                ROD_Y,
                BASE_THICKNESS - 0.002 + REAR_BEAM_HEIGHT * 0.5,
            )
        ),
        material=graphite,
        name="rear_beam",
    )
    for index, x_pos in enumerate((-ROD_SPACING * 0.5, ROD_SPACING * 0.5)):
        base.visual(
            Cylinder(radius=ROD_RADIUS, length=ROD_LENGTH),
            origin=Origin(
                xyz=(
                    x_pos,
                    ROD_Y,
                    BASE_THICKNESS - 0.002 + REAR_BEAM_HEIGHT + ROD_LENGTH * 0.5 - 0.002,
                )
            ),
            material=steel,
            name=f"rod_{index}",
        )
    base.visual(
        Box((0.036, 0.020, 0.004)),
        origin=Origin(xyz=(-0.120, -0.090, 0.002)),
        material=rubber,
        name="foot_0",
    )
    base.visual(
        Box((0.036, 0.020, 0.004)),
        origin=Origin(xyz=(0.120, -0.090, 0.002)),
        material=rubber,
        name="foot_1",
    )
    base.visual(
        Box((0.036, 0.020, 0.004)),
        origin=Origin(xyz=(-0.120, 0.100, 0.002)),
        material=rubber,
        name="foot_2",
    )
    base.visual(
        Box((0.036, 0.020, 0.004)),
        origin=Origin(xyz=(0.120, 0.100, 0.002)),
        material=rubber,
        name="foot_3",
    )
    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_build_carriage_mesh(), "carriage"),
        material=satin_black,
        name="carriage_body",
    )

    tray = model.part("tray")
    tray.visual(
        mesh_from_cadquery(_build_tray_mesh(), "tray"),
        material=graphite,
        name="tray_body",
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, ROD_Y, SLIDE_REST_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=70.0,
            velocity=0.18,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )
    model.articulation(
        "carriage_to_tray",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=tray,
        origin=Origin(xyz=(0.0, -0.040, 0.118)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=-0.20,
            upper=0.50,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    tray = object_model.get_part("tray")
    slide = object_model.get_articulation("base_to_carriage")
    tilt = object_model.get_articulation("carriage_to_tray")

    ctx.expect_within(
        tray,
        base,
        axes="x",
        margin=0.025,
        name="tray stays within the wide base width",
    )

    with ctx.pose({slide: 0.0, tilt: tilt.motion_limits.lower}):
        shallow_clearance = ctx.part_world_aabb(tray)
        base_aabb = ctx.part_element_world_aabb(base, elem="base_slab")
    ctx.check(
        "tray stays above the base at the shallow tilt limit",
        shallow_clearance is not None
        and base_aabb is not None
        and shallow_clearance[0][2] > base_aabb[1][2] + 0.10,
        details=f"tray={shallow_clearance}, base={base_aabb}",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: slide.motion_limits.upper}):
        raised_pos = ctx.part_world_position(carriage)
    ctx.check(
        "carriage lifts upward on the twin guide rods",
        rest_pos is not None
        and raised_pos is not None
        and raised_pos[2] > rest_pos[2] + 0.12,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    with ctx.pose({slide: 0.0, tilt: tilt.motion_limits.lower}):
        shallow_front = ctx.part_world_aabb(tray)
    with ctx.pose({slide: 0.0, tilt: tilt.motion_limits.upper}):
        steep_front = ctx.part_world_aabb(tray)
    ctx.check(
        "tray hinge raises the front stop as tilt increases",
        shallow_front is not None
        and steep_front is not None
        and steep_front[1][2] > shallow_front[1][2] + 0.08,
        details=f"shallow={shallow_front}, steep={steep_front}",
    )

    return ctx.report()


object_model = build_object_model()
