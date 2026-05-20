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


PLATE_LENGTH = 0.240
PLATE_WIDTH = 0.110
PLATE_THICKNESS = 0.012

SUPPORT_CENTER_X = 0.074
SUPPORT_THICKNESS = 0.018
SUPPORT_WIDTH = 0.074
SUPPORT_HEIGHT = 0.086
SUPPORT_EMBED = 0.0003
SUPPORT_PEDESTAL_HEIGHT = 0.032

SPINDLE_AXIS_Z = 0.055
BOSS_RADIUS = 0.024
BOSS_LENGTH = SUPPORT_THICKNESS + 0.010
SUPPORT_INNER_FACE_X = SUPPORT_CENTER_X - SUPPORT_THICKNESS / 2.0

SHAFT_RADIUS = 0.010
SHAFT_END_CLEARANCE = 0.0015
SHAFT_HALF_LENGTH = SUPPORT_INNER_FACE_X - SHAFT_END_CLEARANCE
SHAFT_LENGTH = 2.0 * SHAFT_HALF_LENGTH

BODY_RADIUS = 0.024
BODY_LENGTH = 0.082
REAR_COLLAR_RADIUS = 0.018
REAR_COLLAR_LENGTH = 0.014
OUTPUT_FLANGE_RADIUS = 0.030
OUTPUT_FLANGE_LENGTH = 0.016
REAR_COLLAR_CENTER_X = -(BODY_LENGTH / 2.0 + REAR_COLLAR_LENGTH / 2.0)
OUTPUT_FLANGE_CENTER_X = BODY_LENGTH / 2.0 + OUTPUT_FLANGE_LENGTH / 2.0 - 0.004

SUPPORT_ORIGIN_Z = PLATE_THICKNESS + SUPPORT_HEIGHT / 2.0 - SUPPORT_EMBED
SUPPORT_AXIS_LOCAL_Z = SPINDLE_AXIS_Z - SUPPORT_ORIGIN_Z


def _base_plate_shape() -> cq.Workplane:
    hole_x = 0.084
    hole_y = 0.032
    return (
        cq.Workplane("XY")
        .box(PLATE_LENGTH, PLATE_WIDTH, PLATE_THICKNESS)
        .translate((0.0, 0.0, PLATE_THICKNESS / 2.0))
        .edges("|Z")
        .fillet(0.006)
        .faces(">Z")
        .workplane()
        .pushPoints(
            [
                (-hole_x, -hole_y),
                (-hole_x, hole_y),
                (hole_x, -hole_y),
                (hole_x, hole_y),
            ]
        )
        .hole(0.008)
    )


def _support_shape(side: str) -> cq.Workplane:
    side_sign = -1.0 if side == "left" else 1.0
    cheek = (
        cq.Workplane("XY")
        .box(SUPPORT_THICKNESS, SUPPORT_WIDTH, SUPPORT_HEIGHT)
        .edges(">Z")
        .fillet(0.004)
    )
    pedestal = cq.Workplane("XY").box(
        SUPPORT_THICKNESS + 0.020,
        SUPPORT_WIDTH * 0.56,
        SUPPORT_PEDESTAL_HEIGHT,
    ).translate((0.0, 0.0, -SUPPORT_HEIGHT / 2.0 + SUPPORT_PEDESTAL_HEIGHT / 2.0))
    boss_center_x = side_sign * (SUPPORT_THICKNESS / 2.0 + BOSS_LENGTH / 2.0 - 0.002)
    boss = (
        cq.Workplane(
            "YZ",
            origin=(boss_center_x - BOSS_LENGTH / 2.0, 0.0, SUPPORT_AXIS_LOCAL_Z),
        )
        .circle(BOSS_RADIUS)
        .extrude(BOSS_LENGTH)
    )
    split_slot = (
        cq.Workplane("XY")
        .box(BOSS_LENGTH + 0.006, SUPPORT_WIDTH * 0.22, SUPPORT_HEIGHT * 0.52)
        .translate(
            (boss_center_x, 0.0, SUPPORT_HEIGHT * 0.24 + SUPPORT_AXIS_LOCAL_Z)
        )
    )
    return cheek.union(pedestal).union(boss).cut(split_slot)


def _shaft_journal_shape() -> cq.Workplane:
    return (
        cq.Workplane("YZ", origin=(-SHAFT_LENGTH / 2.0, 0.0, 0.0))
        .circle(SHAFT_RADIUS)
        .extrude(SHAFT_LENGTH)
    )


def _center_body_shape() -> cq.Workplane:
    return (
        cq.Workplane("YZ", origin=(-BODY_LENGTH / 2.0, 0.0, 0.0))
        .circle(BODY_RADIUS)
        .extrude(BODY_LENGTH)
    )


def _rear_collar_shape() -> cq.Workplane:
    return (
        cq.Workplane(
            "YZ",
            origin=(REAR_COLLAR_CENTER_X - REAR_COLLAR_LENGTH / 2.0, 0.0, 0.0),
        )
        .circle(REAR_COLLAR_RADIUS)
        .extrude(REAR_COLLAR_LENGTH)
    )


def _output_flange_shape() -> cq.Workplane:
    return (
        cq.Workplane(
            "YZ",
            origin=(
                OUTPUT_FLANGE_CENTER_X - OUTPUT_FLANGE_LENGTH / 2.0,
                0.0,
                0.0,
            ),
        )
        .circle(OUTPUT_FLANGE_RADIUS)
        .extrude(OUTPUT_FLANGE_LENGTH)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="split_cheek_roll_axis_spindle")

    model.material("base_steel", rgba=(0.33, 0.36, 0.39, 1.0))
    model.material("support_gray", rgba=(0.45, 0.48, 0.52, 1.0))
    model.material("spindle_dark", rgba=(0.16, 0.18, 0.20, 1.0))

    base = model.part("base_frame")
    base.visual(
        mesh_from_cadquery(_base_plate_shape(), "base_plate"),
        material="base_steel",
        name="base_plate",
    )
    base.visual(
        mesh_from_cadquery(_support_shape("left"), "left_support"),
        origin=Origin(xyz=(-SUPPORT_CENTER_X, 0.0, SUPPORT_ORIGIN_Z)),
        material="support_gray",
        name="left_support",
    )
    base.visual(
        mesh_from_cadquery(_support_shape("right"), "right_support"),
        origin=Origin(xyz=(SUPPORT_CENTER_X, 0.0, SUPPORT_ORIGIN_Z)),
        material="support_gray",
        name="right_support",
    )
    base.inertial = Inertial.from_geometry(
        Box((PLATE_LENGTH, PLATE_WIDTH, SUPPORT_HEIGHT + PLATE_THICKNESS)),
        mass=6.2,
        origin=Origin(xyz=(0.0, 0.0, (SUPPORT_HEIGHT + PLATE_THICKNESS) / 2.0)),
    )

    spindle = model.part("spindle_head")
    spindle.visual(
        mesh_from_cadquery(_shaft_journal_shape(), "shaft_journal"),
        material="spindle_dark",
        name="shaft_journal",
    )
    spindle.visual(
        mesh_from_cadquery(_center_body_shape(), "center_body"),
        material="spindle_dark",
        name="center_body",
    )
    spindle.visual(
        mesh_from_cadquery(_rear_collar_shape(), "rear_collar"),
        material="spindle_dark",
        name="rear_collar",
    )
    spindle.visual(
        mesh_from_cadquery(_output_flange_shape(), "output_flange"),
        material="spindle_dark",
        name="output_flange",
    )
    spindle.inertial = Inertial.from_geometry(
        Box((SHAFT_LENGTH, OUTPUT_FLANGE_RADIUS * 2.0, OUTPUT_FLANGE_RADIUS * 2.0)),
        mass=1.1,
        origin=Origin(),
    )

    model.articulation(
        "roll_spin",
        ArticulationType.REVOLUTE,
        parent=base,
        child=spindle,
        origin=Origin(xyz=(0.0, 0.0, SPINDLE_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=8.0,
            lower=-pi,
            upper=pi,
        ),
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

    base = object_model.get_part("base_frame")
    spindle = object_model.get_part("spindle_head")
    roll_spin = object_model.get_articulation("roll_spin")

    base_plate = base.get_visual("base_plate")
    left_support = base.get_visual("left_support")
    right_support = base.get_visual("right_support")
    shaft_journal = spindle.get_visual("shaft_journal")
    center_body = spindle.get_visual("center_body")
    output_flange = spindle.get_visual("output_flange")

    limits = roll_spin.motion_limits
    ctx.check(
        "roll module parts and joint exist",
        base is not None and spindle is not None and roll_spin is not None,
        details="Expected base_frame, spindle_head, and roll_spin articulation.",
    )
    ctx.check(
        "roll joint is aligned to spindle shaft axis",
        roll_spin.axis == (1.0, 0.0, 0.0)
        and limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower < 0.0 < limits.upper,
        details=f"axis={roll_spin.axis}, limits={limits}",
    )

    ctx.expect_gap(
        spindle,
        base,
        axis="x",
        min_gap=0.012,
        max_gap=0.018,
        positive_elem=center_body,
        negative_elem=left_support,
        name="spindle clears left cheek support",
    )
    ctx.expect_gap(
        base,
        spindle,
        axis="x",
        min_gap=0.012,
        max_gap=0.018,
        positive_elem=right_support,
        negative_elem=center_body,
        name="spindle clears right cheek support",
    )
    ctx.expect_gap(
        base,
        spindle,
        axis="x",
        min_gap=0.001,
        max_gap=0.006,
        positive_elem=right_support,
        negative_elem=output_flange,
        name="output flange stays just inside the right cheek plane",
    )
    ctx.expect_gap(
        spindle,
        base,
        axis="z",
        min_gap=0.010,
        max_gap=0.018,
        positive_elem=output_flange,
        negative_elem=base_plate,
        name="spindle rides above the grounded base plate",
    )

    rest_pos = ctx.part_world_position(spindle)
    with ctx.pose({roll_spin: 1.4}):
        posed_pos = ctx.part_world_position(spindle)
    centered_in_spin = (
        rest_pos is not None
        and posed_pos is not None
        and max(abs(a - b) for a, b in zip(rest_pos, posed_pos)) <= 1e-6
    )
    ctx.check(
        "roll articulation spins in place between fixed supports",
        centered_in_spin,
        details=f"rest={rest_pos}, posed={posed_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
