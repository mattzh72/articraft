from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

TOP_Z = 0.39
PAD_WIDTH = 0.30
PAD_THICKNESS = 0.07
SEAT_LENGTH = 0.33
BACKREST_LENGTH = 0.82

LADDER_PITCH = -0.95
LADDER_LENGTH = 0.40

BRACE_PITCH = -0.55
BRACE_LENGTH = 0.22


def _pitched_z_point(distance: float, pitch: float) -> tuple[float, float]:
    return (distance * math.sin(pitch), distance * math.cos(pitch))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_bench")

    frame_color = model.material("frame_dark", rgba=(0.12, 0.12, 0.13, 1.0))
    pad_color = model.material("pad_black", rgba=(0.08, 0.08, 0.09, 1.0))
    metal_color = model.material("metal_mid", rgba=(0.50, 0.50, 0.52, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.09, 0.56, 0.05)),
        origin=Origin(xyz=(-0.33, 0.0, 0.025)),
        material=frame_color,
        name="front_foot",
    )
    frame.visual(
        Box((0.09, 0.56, 0.05)),
        origin=Origin(xyz=(0.53, 0.0, 0.025)),
        material=frame_color,
        name="rear_foot",
    )
    for idx, y in enumerate((-0.18, 0.18)):
        frame.visual(
            Box((0.05, 0.05, 0.34)),
            origin=Origin(xyz=(-0.33, y, 0.22)),
            material=frame_color,
            name=f"front_leg_{idx}",
        )
        frame.visual(
            Box((0.05, 0.05, 0.34)),
            origin=Origin(xyz=(0.53, y, 0.22)),
            material=frame_color,
            name=f"rear_leg_{idx}",
        )
        frame.visual(
            Box((0.86, 0.05, 0.05)),
            origin=Origin(xyz=(0.10, y, 0.365)),
            material=frame_color,
            name=f"side_rail_{idx}",
        )

    frame.visual(
        Box((0.05, 0.08, 0.21)),
        origin=Origin(xyz=(-0.33, 0.0, 0.155)),
        material=frame_color,
        name="front_post",
    )
    frame.visual(
        Box((0.05, 0.08, 0.21)),
        origin=Origin(xyz=(0.53, 0.0, 0.155)),
        material=frame_color,
        name="rear_post",
    )
    frame.visual(
        Box((0.86, 0.08, 0.05)),
        origin=Origin(xyz=(0.10, 0.0, 0.285)),
        material=frame_color,
        name="spine",
    )
    frame.visual(
        Box((0.06, 0.31, 0.05)),
        origin=Origin(xyz=(-0.33, 0.0, 0.365)),
        material=frame_color,
        name="front_cross",
    )
    frame.visual(
        Box((0.06, 0.31, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.365)),
        material=frame_color,
        name="hinge_cross",
    )
    frame.visual(
        Box((0.06, 0.08, 0.08)),
        origin=Origin(xyz=(-0.33, 0.0, 0.35)),
        material=frame_color,
        name="seat_pedestal",
    )
    frame.visual(
        Box((0.06, 0.08, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.35)),
        material=frame_color,
        name="backrest_pedestal",
    )
    seat = model.part("seat")
    seat.visual(
        Box((SEAT_LENGTH, PAD_WIDTH, 0.065)),
        origin=Origin(xyz=(SEAT_LENGTH / 2.0, 0.0, 0.0325)),
        material=pad_color,
        name="seat_pad",
    )
    seat.visual(
        Box((0.22, 0.10, 0.04)),
        origin=Origin(xyz=(0.16, 0.0, -0.02)),
        material=frame_color,
        name="seat_spine",
    )
    seat.visual(
        Box((0.06, 0.12, 0.07)),
        origin=Origin(xyz=(0.24, -0.02, -0.035)),
        material=metal_color,
        name="rear_block",
    )

    backrest = model.part("backrest")
    backrest.visual(
        Box((BACKREST_LENGTH, PAD_WIDTH, PAD_THICKNESS)),
        origin=Origin(xyz=(BACKREST_LENGTH / 2.0, 0.0, PAD_THICKNESS / 2.0)),
        material=pad_color,
        name="backrest_pad",
    )
    backrest.visual(
        Box((0.48, 0.05, 0.05)),
        origin=Origin(xyz=(0.30, 0.085, -0.025)),
        material=frame_color,
        name="bracket",
    )
    backrest.visual(
        Box((0.06, 0.10, 0.10)),
        origin=Origin(xyz=(0.09, 0.085, -0.05)),
        material=metal_color,
        name="hinge_block",
    )
    backrest.visual(
        Box((0.12, 0.06, 0.08)),
        origin=Origin(xyz=(0.56, 0.055, -0.04)),
        material=metal_color,
        name="hook_block",
    )

    ladder = model.part("ladder")
    ladder_mid_x, ladder_mid_z = _pitched_z_point(LADDER_LENGTH / 2.0, LADDER_PITCH)
    ladder.visual(
        Box((0.04, 0.02, LADDER_LENGTH)),
        origin=Origin(xyz=(ladder_mid_x, -0.03, ladder_mid_z), rpy=(0.0, LADDER_PITCH, 0.0)),
        material=metal_color,
        name="side_0",
    )
    ladder.visual(
        Box((0.04, 0.02, LADDER_LENGTH)),
        origin=Origin(xyz=(ladder_mid_x, 0.03, ladder_mid_z), rpy=(0.0, LADDER_PITCH, 0.0)),
        material=metal_color,
        name="side_1",
    )
    for idx, distance in enumerate((0.10, 0.18, 0.26, 0.32)):
        rung_x, rung_z = _pitched_z_point(distance, LADDER_PITCH)
        ladder.visual(
            Box((0.04, 0.08, 0.02)),
            origin=Origin(xyz=(rung_x, 0.0, rung_z)),
            material=metal_color,
            name=f"rung_{idx}",
        )
    top_x, top_z = _pitched_z_point(0.35, LADDER_PITCH)
    ladder.visual(
        Box((0.06, 0.08, 0.03)),
        origin=Origin(xyz=(top_x, 0.0, top_z)),
        material=frame_color,
        name="top_yoke",
    )

    brace = model.part("brace")
    brace_mid_x, brace_mid_z = _pitched_z_point(BRACE_LENGTH / 2.0, BRACE_PITCH)
    brace.visual(
        Box((0.03, 0.02, BRACE_LENGTH)),
        origin=Origin(xyz=(brace_mid_x, -0.03, brace_mid_z), rpy=(0.0, BRACE_PITCH, 0.0)),
        material=metal_color,
        name="strap_0",
    )
    brace.visual(
        Box((0.03, 0.02, BRACE_LENGTH)),
        origin=Origin(xyz=(brace_mid_x, 0.03, brace_mid_z), rpy=(0.0, BRACE_PITCH, 0.0)),
        material=metal_color,
        name="strap_1",
    )
    bridge_x, bridge_z = _pitched_z_point(0.14, BRACE_PITCH)
    brace.visual(
        Box((0.04, 0.08, 0.03)),
        origin=Origin(xyz=(bridge_x, 0.0, bridge_z)),
        material=frame_color,
        name="brace_bridge",
    )

    model.articulation(
        "frame_to_backrest",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=backrest,
        origin=Origin(xyz=(0.0, 0.0, TOP_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=150.0, velocity=1.6, lower=0.0, upper=1.15),
    )
    model.articulation(
        "frame_to_seat",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=seat,
        origin=Origin(xyz=(-0.33, 0.0, TOP_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.6, lower=0.0, upper=0.24),
    )
    model.articulation(
        "frame_to_ladder",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=ladder,
        origin=Origin(xyz=(0.49, -0.245, 0.15)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.4, lower=-0.15, upper=0.40),
    )
    model.articulation(
        "frame_to_brace",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=brace,
        origin=Origin(xyz=(-0.05, 0.245, 0.20)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.6, lower=-0.50, upper=0.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    seat = object_model.get_part("seat")
    backrest = object_model.get_part("backrest")
    ladder = object_model.get_part("ladder")
    brace = object_model.get_part("brace")

    backrest_hinge = object_model.get_articulation("frame_to_backrest")
    seat_hinge = object_model.get_articulation("frame_to_seat")
    ladder_hinge = object_model.get_articulation("frame_to_ladder")
    brace_hinge = object_model.get_articulation("frame_to_brace")

    ctx.expect_gap(
        backrest,
        seat,
        axis="x",
        positive_elem="backrest_pad",
        negative_elem="seat_pad",
        min_gap=0.0,
        max_gap=0.01,
        name="seat and backrest meet at the hinge line",
    )
    ctx.expect_overlap(
        backrest,
        seat,
        axes="y",
        elem_a="backrest_pad",
        elem_b="seat_pad",
        min_overlap=0.25,
        name="seat and backrest stay aligned across bench width",
    )
    ctx.expect_gap(
        backrest,
        ladder,
        axis="y",
        positive_elem="bracket",
        negative_elem="top_yoke",
        min_gap=0.02,
        name="ladder support sits behind the backrest bracket",
    )
    ctx.expect_gap(
        seat,
        brace,
        axis="z",
        positive_elem="seat_pad",
        negative_elem="brace_bridge",
        min_gap=0.015,
        max_gap=0.08,
        name="brace remains a distinct member below the seat section",
    )

    def top_z(part_name: str, elem: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part_name, elem=elem)
        return None if aabb is None else aabb[1][2]

    flat_backrest_z = top_z("backrest", "backrest_pad")
    flat_seat_z = top_z("seat", "seat_pad")
    rest_ladder_z = top_z("ladder", "top_yoke")
    rest_brace_z = top_z("brace", "brace_bridge")

    with ctx.pose(
        {
            backrest_hinge: backrest_hinge.motion_limits.upper,
            seat_hinge: seat_hinge.motion_limits.upper,
            ladder_hinge: ladder_hinge.motion_limits.upper,
        }
    ):
        incline_backrest_z = top_z("backrest", "backrest_pad")
        incline_seat_z = top_z("seat", "seat_pad")
        incline_ladder_z = top_z("ladder", "top_yoke")

    with ctx.pose({brace_hinge: brace_hinge.motion_limits.lower}):
        tucked_brace_z = top_z("brace", "brace_bridge")

    ctx.check(
        "backrest pitches upward",
        flat_backrest_z is not None
        and incline_backrest_z is not None
        and incline_backrest_z > flat_backrest_z + 0.22,
        details=f"flat={flat_backrest_z}, incline={incline_backrest_z}",
    )
    ctx.check(
        "seat front hinge lifts rear of seat",
        flat_seat_z is not None
        and incline_seat_z is not None
        and incline_seat_z > flat_seat_z + 0.04,
        details=f"flat={flat_seat_z}, incline={incline_seat_z}",
    )
    ctx.check(
        "ladder support rotates upward",
        rest_ladder_z is not None
        and incline_ladder_z is not None
        and incline_ladder_z > rest_ladder_z + 0.07,
        details=f"rest={rest_ladder_z}, incline={incline_ladder_z}",
    )
    ctx.check(
        "brace tucks farther under the bench",
        rest_brace_z is not None
        and tucked_brace_z is not None
        and tucked_brace_z < rest_brace_z - 0.035,
        details=f"rest={rest_brace_z}, tucked={tucked_brace_z}",
    )

    return ctx.report()


object_model = build_object_model()
