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


BASE_RADIUS = 0.21
BASE_THICKNESS = 0.018
BASE_HUB_RADIUS = 0.060
BASE_HUB_HEIGHT = 0.020

SHROUD_OUTER_RADIUS = 0.027
SHROUD_INNER_RADIUS = 0.022
SHROUD_HEIGHT = 0.340

PISTON_RADIUS = 0.019
PISTON_LENGTH = 0.540
PISTON_CENTER_Z = 0.080
PISTON_TOP_Z = PISTON_CENTER_Z + PISTON_LENGTH * 0.5
LIFT_TRAVEL = 0.110

SEAT_WIDTH = 0.360
SEAT_THICKNESS = 0.050
SEAT_CUSHION_CENTER_Z = 0.037


def _seat_cushion_mesh():
    return (
        cq.Workplane("XY")
        .box(SEAT_WIDTH, SEAT_WIDTH, SEAT_THICKNESS)
        .edges("|Z")
        .fillet(0.028)
    )


def _shroud_mesh():
    return (
        cq.Workplane("XY")
        .circle(SHROUD_OUTER_RADIUS)
        .circle(SHROUD_INNER_RADIUS)
        .extrude(SHROUD_HEIGHT)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hydraulic_bar_stool")

    polished_steel = model.material("polished_steel", rgba=(0.76, 0.78, 0.81, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.23, 0.24, 0.26, 1.0))
    seat_vinyl = model.material("seat_vinyl", rgba=(0.11, 0.11, 0.12, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=BASE_RADIUS, length=BASE_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS * 0.5)),
        material=polished_steel,
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=0.155, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=rubber,
        name="floor_ring",
    )
    base.visual(
        Cylinder(radius=BASE_HUB_RADIUS, length=BASE_HUB_HEIGHT),
        origin=Origin(
            xyz=(0.0, 0.0, BASE_THICKNESS + BASE_HUB_HEIGHT * 0.5),
        ),
        material=polished_steel,
        name="base_hub",
    )

    shroud = model.part("shroud")
    shroud.visual(
        mesh_from_cadquery(_shroud_mesh(), "bar_stool_shroud"),
        material=polished_steel,
        name="shroud",
    )

    column = model.part("column")
    column.visual(
        Cylinder(radius=PISTON_RADIUS, length=PISTON_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, PISTON_CENTER_Z)),
        material=polished_steel,
        name="piston",
    )
    column.visual(
        Cylinder(radius=0.028, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=dark_metal,
        name="guide_collar",
    )
    column.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, PISTON_TOP_Z - 0.009)),
        material=dark_metal,
        name="mount_collar",
    )

    seat = model.part("seat")
    seat.visual(
        Cylinder(radius=0.032, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=dark_metal,
        name="swivel_hub",
    )
    seat.visual(
        Box((0.200, 0.200, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=dark_metal,
        name="underframe",
    )
    seat.visual(
        Box((0.060, 0.070, 0.012)),
        origin=Origin(xyz=(0.0, 0.125, 0.006)),
        material=dark_metal,
        name="lever_mount",
    )
    seat.visual(
        Box((0.008, 0.020, 0.024)),
        origin=Origin(xyz=(-0.018, 0.150, -0.011)),
        material=dark_metal,
        name="pivot_cheek_0",
    )
    seat.visual(
        Box((0.008, 0.020, 0.024)),
        origin=Origin(xyz=(0.018, 0.150, -0.011)),
        material=dark_metal,
        name="pivot_cheek_1",
    )
    seat.visual(
        mesh_from_cadquery(_seat_cushion_mesh(), "bar_stool_seat_cushion"),
        origin=Origin(xyz=(0.0, 0.0, SEAT_CUSHION_CENTER_Z)),
        material=seat_vinyl,
        name="cushion",
    )

    release_lever = model.part("release_lever")
    release_lever.visual(
        Cylinder(radius=0.006, length=0.028),
        origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
        material=polished_steel,
        name="pivot_barrel",
    )
    release_lever.visual(
        Box((0.012, 0.090, 0.010)),
        origin=Origin(xyz=(0.0, 0.045, -0.020), rpy=(-0.48, 0.0, 0.0)),
        material=polished_steel,
        name="handle_tip",
    )

    model.articulation(
        "base_to_shroud",
        ArticulationType.FIXED,
        parent=base,
        child=shroud,
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + BASE_HUB_HEIGHT)),
    )
    model.articulation(
        "shroud_lift",
        ArticulationType.PRISMATIC,
        parent=shroud,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, SHROUD_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.22,
            lower=0.0,
            upper=LIFT_TRAVEL,
        ),
    )
    model.articulation(
        "column_swivel",
        ArticulationType.CONTINUOUS,
        parent=column,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, PISTON_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=24.0, velocity=4.5),
    )
    model.articulation(
        "lever_pivot",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=release_lever,
        origin=Origin(xyz=(0.0, 0.150, -0.012)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=3.0,
            lower=-0.30,
            upper=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    shroud = object_model.get_part("shroud")
    column = object_model.get_part("column")
    seat = object_model.get_part("seat")
    release_lever = object_model.get_part("release_lever")

    shroud_lift = object_model.get_articulation("shroud_lift")
    column_swivel = object_model.get_articulation("column_swivel")
    lever_pivot = object_model.get_articulation("lever_pivot")

    lift_upper = 0.0
    if shroud_lift.motion_limits is not None and shroud_lift.motion_limits.upper is not None:
        lift_upper = shroud_lift.motion_limits.upper

    ctx.expect_within(
        column,
        shroud,
        axes="xy",
        inner_elem="piston",
        outer_elem="shroud",
        name="piston stays centered in the shroud",
    )
    ctx.expect_overlap(
        column,
        shroud,
        axes="z",
        elem_a="piston",
        elem_b="shroud",
        min_overlap=0.18,
        name="lowered piston remains substantially inserted",
    )
    ctx.expect_origin_distance(
        seat,
        column,
        axes="xy",
        max_dist=0.001,
        name="seat is centered on the column axis",
    )

    seat_rest = ctx.part_world_position(seat)
    lever_tip_rest = ctx.part_element_world_aabb(release_lever, elem="handle_tip")

    with ctx.pose({column_swivel: math.pi / 4.0}):
        ctx.expect_origin_distance(
            seat,
            column,
            axes="xy",
            max_dist=0.001,
            name="seat remains centered while swiveling",
        )

    with ctx.pose({shroud_lift: lift_upper}):
        ctx.expect_within(
            column,
            shroud,
            axes="xy",
            inner_elem="piston",
            outer_elem="shroud",
            name="raised piston stays centered in the shroud",
        )
        ctx.expect_overlap(
            column,
            shroud,
            axes="z",
            elem_a="piston",
            elem_b="shroud",
            min_overlap=0.08,
            name="raised piston keeps retained insertion",
        )
        seat_raised = ctx.part_world_position(seat)

    ctx.check(
        "seat rises through the hydraulic travel",
        seat_rest is not None
        and seat_raised is not None
        and seat_raised[2] > seat_rest[2] + 0.09,
        details=f"rest={seat_rest}, raised={seat_raised}",
    )

    ctx.expect_gap(
        seat,
        release_lever,
        axis="z",
        positive_elem="underframe",
        negative_elem="handle_tip",
        min_gap=0.001,
        name="release lever hangs below the seat frame",
    )

    with ctx.pose({lever_pivot: 0.45}):
        ctx.expect_gap(
            seat,
            release_lever,
            axis="z",
            positive_elem="underframe",
            negative_elem="handle_tip",
            max_penetration=0.0,
            name="release lever still clears the seat when lifted",
        )
        lever_tip_raised = ctx.part_element_world_aabb(release_lever, elem="handle_tip")

    ctx.check(
        "release lever lifts upward",
        lever_tip_rest is not None
        and lever_tip_raised is not None
        and lever_tip_raised[1][2] > lever_tip_rest[1][2] + 0.015,
        details=f"rest_tip={lever_tip_rest}, raised_tip={lever_tip_raised}",
    )

    return ctx.report()


object_model = build_object_model()
