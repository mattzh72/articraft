from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


COLUMN_BASE_SIZE = 0.72
COLUMN_BASE_THICKNESS = 0.04
COLUMN_OUTER_RADIUS = 0.11
PIVOT_Z = 2.55
BEAM_TURNTABLE_RADIUS = 0.15
BEAM_TURNTABLE_HEIGHT = 0.10
BEAM_LENGTH = 2.15
BEAM_START_X = 0.18
BEAM_DEPTH = 0.20
BEAM_FLANGE_WIDTH = 0.14
BEAM_FLANGE_THICKNESS = 0.018
BEAM_WEB_THICKNESS = 0.014
BEAM_ARM_CENTER_Z = 0.13

CARRIAGE_LENGTH = 0.24
CARRIAGE_SIDE_PLATE_THICKNESS = 0.014
CARRIAGE_SIDE_PLATE_HEIGHT = 0.10
CARRIAGE_SIDE_PLATE_CENTER_Z = -0.056
CARRIAGE_PLATE_CENTER_Y = 0.053
CARRIAGE_TOP_RUNNER_THICKNESS = 0.012
CARRIAGE_JOINT_X = 0.62
CARRIAGE_JOINT_Z = BEAM_ARM_CENTER_Z - (BEAM_DEPTH / 2.0)
CARRIAGE_TRAVEL = 1.28


def _box_centered(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cylinder_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length).translate(
        (center[0], center[1] - (length / 2.0), center[2])
    )


def _column_shape() -> cq.Workplane:
    base_plate = cq.Workplane("XY").box(
        COLUMN_BASE_SIZE,
        COLUMN_BASE_SIZE,
        COLUMN_BASE_THICKNESS,
    ).translate((0.0, 0.0, COLUMN_BASE_THICKNESS / 2.0))

    pedestal = cq.Workplane("XY").box(0.32, 0.32, 0.05).translate((0.0, 0.0, 0.045))

    mast = (
        cq.Workplane("XY")
        .circle(COLUMN_OUTER_RADIUS)
        .extrude(PIVOT_Z - 0.31)
        .translate((0.0, 0.0, 0.035))
    )

    upper_head = cq.Workplane("XY").circle(0.13).extrude(0.18).translate((0.0, 0.0, PIVOT_Z - 0.31))
    bearing_seat = cq.Workplane("XY").circle(0.155).extrude(0.14).translate((0.0, 0.0, PIVOT_Z - 0.14))

    rib = _box_centered((0.12, 0.065, 0.92), (0.12, 0.0, 0.50))

    column = base_plate.union(pedestal).union(mast).union(upper_head).union(bearing_seat)
    for angle in (0.0, 90.0, 180.0, 270.0):
        column = column.union(rib.rotate((0, 0, 0), (0, 0, 1), angle))

    return column


def _beam_shape() -> cq.Workplane:
    arm_center_x = BEAM_START_X + (BEAM_LENGTH / 2.0)

    turntable = (
        cq.Workplane("XY")
        .circle(BEAM_TURNTABLE_RADIUS)
        .extrude(BEAM_TURNTABLE_HEIGHT)
        .translate((0.0, 0.0, 0.0))
    )

    web = _box_centered(
        (BEAM_LENGTH, BEAM_WEB_THICKNESS, BEAM_DEPTH - (2.0 * BEAM_FLANGE_THICKNESS)),
        (arm_center_x, 0.0, BEAM_ARM_CENTER_Z),
    )
    top_flange = _box_centered(
        (BEAM_LENGTH, BEAM_FLANGE_WIDTH, BEAM_FLANGE_THICKNESS),
        (arm_center_x, 0.0, BEAM_ARM_CENTER_Z + (BEAM_DEPTH / 2.0) - (BEAM_FLANGE_THICKNESS / 2.0)),
    )
    bottom_flange = _box_centered(
        (BEAM_LENGTH, BEAM_FLANGE_WIDTH, BEAM_FLANGE_THICKNESS),
        (arm_center_x, 0.0, BEAM_ARM_CENTER_Z - (BEAM_DEPTH / 2.0) + (BEAM_FLANGE_THICKNESS / 2.0)),
    )
    pedestal = _box_centered((0.20, 0.14, 0.10), (0.12, 0.0, 0.09))
    root_block = _box_centered((0.24, 0.10, 0.15), (0.25, 0.0, 0.13))
    root_shoe = _box_centered((0.28, 0.12, 0.035), (0.19, 0.0, 0.048))
    inboard_stop = _box_centered((0.02, 0.06, 0.08), (BEAM_START_X + 0.05, 0.0, 0.07))
    outboard_stop = _box_centered((0.02, 0.06, 0.08), (BEAM_START_X + BEAM_LENGTH - 0.05, 0.0, 0.07))

    beam = turntable.union(web).union(top_flange).union(bottom_flange)
    beam = beam.union(pedestal).union(root_block).union(root_shoe)
    beam = beam.union(inboard_stop).union(outboard_stop)
    return beam


def _carriage_shape() -> cq.Workplane:
    top_runner = _box_centered(
        (0.22, 0.12, CARRIAGE_TOP_RUNNER_THICKNESS),
        (0.0, 0.0, -(CARRIAGE_TOP_RUNNER_THICKNESS / 2.0)),
    )
    left_plate = _box_centered(
        (CARRIAGE_LENGTH, CARRIAGE_SIDE_PLATE_THICKNESS, CARRIAGE_SIDE_PLATE_HEIGHT),
        (0.0, CARRIAGE_PLATE_CENTER_Y, CARRIAGE_SIDE_PLATE_CENTER_Z),
    )
    right_plate = _box_centered(
        (CARRIAGE_LENGTH, CARRIAGE_SIDE_PLATE_THICKNESS, CARRIAGE_SIDE_PLATE_HEIGHT),
        (0.0, -CARRIAGE_PLATE_CENTER_Y, CARRIAGE_SIDE_PLATE_CENTER_Z),
    )

    front_tie = _box_centered((0.034, 0.106, 0.028), (0.075, 0.0, -0.020))
    rear_tie = _box_centered((0.034, 0.106, 0.028), (-0.075, 0.0, -0.020))
    center_housing = _box_centered((0.13, 0.09, 0.10), (0.0, 0.0, -0.070))
    hanger_block = _box_centered((0.085, 0.075, 0.12), (0.0, 0.0, -0.180))
    load_pad = _box_centered((0.11, 0.065, 0.035), (0.0, 0.0, -0.2575))
    left_guide = _box_centered((0.06, 0.018, 0.028), (0.0, 0.061, -0.020))
    right_guide = _box_centered((0.06, 0.018, 0.028), (0.0, -0.061, -0.020))

    carriage = top_runner.union(left_plate).union(right_plate)
    carriage = carriage.union(front_tie).union(rear_tie)
    carriage = carriage.union(center_housing).union(hanger_block).union(load_pad)
    carriage = carriage.union(left_guide).union(right_guide)
    return carriage


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swing_beam_carriage_module")

    model.material("column_paint", rgba=(0.27, 0.30, 0.32, 1.0))
    model.material("beam_yellow", rgba=(0.92, 0.75, 0.16, 1.0))
    model.material("carriage_gray", rgba=(0.50, 0.52, 0.55, 1.0))

    column = model.part("column")
    column.visual(
        mesh_from_cadquery(_column_shape(), "column_structure"),
        material="column_paint",
        name="column_structure",
    )
    column.inertial = Inertial.from_geometry(
        Box((COLUMN_BASE_SIZE, COLUMN_BASE_SIZE, PIVOT_Z + 0.18)),
        mass=420.0,
        origin=Origin(xyz=(0.0, 0.0, (PIVOT_Z + 0.18) / 2.0)),
    )

    beam = model.part("beam")
    beam.visual(
        mesh_from_cadquery(_beam_shape(), "swing_beam"),
        material="beam_yellow",
        name="swing_beam",
    )
    beam.inertial = Inertial.from_geometry(
        Box((BEAM_START_X + BEAM_LENGTH, BEAM_FLANGE_WIDTH, BEAM_DEPTH)),
        mass=185.0,
        origin=Origin(xyz=((BEAM_START_X + BEAM_LENGTH) / 2.0, 0.0, 0.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_shape(), "beam_carriage"),
        material="carriage_gray",
        name="beam_carriage",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_LENGTH, 0.20, 0.28)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, -0.02)),
    )

    model.articulation(
        "column_to_beam",
        ArticulationType.REVOLUTE,
        parent=column,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-1.45, upper=1.45, effort=900.0, velocity=0.8),
    )
    model.articulation(
        "beam_to_carriage",
        ArticulationType.PRISMATIC,
        parent=beam,
        child=carriage,
        origin=Origin(xyz=(CARRIAGE_JOINT_X, 0.0, CARRIAGE_JOINT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=CARRIAGE_TRAVEL, effort=1200.0, velocity=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    column = object_model.get_part("column")
    beam = object_model.get_part("beam")
    carriage = object_model.get_part("carriage")
    beam_swing = object_model.get_articulation("column_to_beam")
    carriage_slide = object_model.get_articulation("beam_to_carriage")

    ctx.check("column exists", column is not None)
    ctx.check("beam exists", beam is not None)
    ctx.check("carriage exists", carriage is not None)

    with ctx.pose({beam_swing: 0.0, carriage_slide: 0.0}):
        ctx.expect_contact(
            beam,
            carriage,
            contact_tol=0.001,
            name="carriage runs in contact with the beam at home",
        )
        ctx.expect_overlap(
            beam,
            carriage,
            axes="x",
            min_overlap=0.12,
            name="carriage remains engaged under beam at home",
        )

    with ctx.pose({beam_swing: 0.0, carriage_slide: CARRIAGE_TRAVEL}):
        ctx.expect_contact(
            beam,
            carriage,
            contact_tol=0.001,
            name="carriage remains supported by the beam at full reach",
        )
        ctx.expect_overlap(
            beam,
            carriage,
            axes="x",
            min_overlap=0.12,
            name="carriage remains under beam at full reach",
        )

    with ctx.pose({beam_swing: 0.0, carriage_slide: 0.0}):
        home_carriage_pos = ctx.part_world_position(carriage)
    with ctx.pose({beam_swing: 0.0, carriage_slide: CARRIAGE_TRAVEL}):
        extended_carriage_pos = ctx.part_world_position(carriage)
    ctx.check(
        "carriage slides outward along beam",
        home_carriage_pos is not None
        and extended_carriage_pos is not None
        and extended_carriage_pos[0] > home_carriage_pos[0] + 1.0,
        details=f"home={home_carriage_pos}, extended={extended_carriage_pos}",
    )

    with ctx.pose({beam_swing: 0.0, carriage_slide: 0.5}):
        rest_swept_pos = ctx.part_world_position(carriage)
    with ctx.pose({beam_swing: 0.95, carriage_slide: 0.5}):
        rotated_swept_pos = ctx.part_world_position(carriage)
    ctx.check(
        "beam sweep carries carriage around column",
        rest_swept_pos is not None
        and rotated_swept_pos is not None
        and rotated_swept_pos[1] > rest_swept_pos[1] + 0.45,
        details=f"rest={rest_swept_pos}, rotated={rotated_swept_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
