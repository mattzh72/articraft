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


CABINET_LENGTH = 1.82
CABINET_WIDTH = 0.94
CABINET_HEIGHT = 0.89
OPENING_LENGTH = 1.60
OPENING_WIDTH = 0.74
LINER_FLOOR_Z = 0.20

LOWER_TRACK_Y = 0.352
UPPER_TRACK_Y = 0.390
TRACK_LENGTH = 1.54
LOWER_TRACK_TOP_Z = CABINET_HEIGHT + 0.010
UPPER_TRACK_TOP_Z = CABINET_HEIGHT + 0.034

LID_LENGTH = 0.83
LOWER_LID_WIDTH = 0.75
UPPER_LID_WIDTH = 0.81
LID_TRAVEL = 0.68
LID_CLOSED_X = 0.385


def _build_cabinet_shell() -> object:
    outer = cq.Workplane("XY").box(
        CABINET_LENGTH,
        CABINET_WIDTH,
        CABINET_HEIGHT,
        centered=(True, True, False),
    )
    outer = outer.edges("|Z").fillet(0.028)

    cavity = (
        cq.Workplane("XY")
        .box(
            OPENING_LENGTH,
            OPENING_WIDTH,
            CABINET_HEIGHT - LINER_FLOOR_Z + 0.03,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, LINER_FLOOR_Z))
    )
    return outer.cut(cavity)


def _add_sliding_lid(
    part,
    *,
    width: float,
    track_y: float,
    handle_sign: float,
    frame_material: str,
    glass_material: str,
) -> None:
    frame_thickness = 0.028
    frame_height = 0.018
    runner_length = 0.76
    runner_width = 0.016
    runner_height = 0.006

    frame_y = width * 0.5 - frame_thickness * 0.5
    frame_x = LID_LENGTH * 0.5 - frame_thickness * 0.5
    glass_length = LID_LENGTH - frame_thickness * 1.05
    glass_width = width - frame_thickness * 1.05

    part.visual(
        Box((runner_length, runner_width, runner_height)),
        origin=Origin(xyz=(0.0, track_y, runner_height * 0.5)),
        material=frame_material,
        name="front_runner",
    )
    part.visual(
        Box((runner_length, runner_width, runner_height)),
        origin=Origin(xyz=(0.0, -track_y, runner_height * 0.5)),
        material=frame_material,
        name="rear_runner",
    )

    part.visual(
        Box((LID_LENGTH, frame_thickness, frame_height)),
        origin=Origin(xyz=(0.0, frame_y, 0.011)),
        material=frame_material,
        name="front_frame",
    )
    part.visual(
        Box((LID_LENGTH, frame_thickness, frame_height)),
        origin=Origin(xyz=(0.0, -frame_y, 0.011)),
        material=frame_material,
        name="rear_frame",
    )
    part.visual(
        Box((frame_thickness, width - frame_thickness * 2.0, frame_height)),
        origin=Origin(xyz=(frame_x, 0.0, 0.011)),
        material=frame_material,
        name="right_frame",
    )
    part.visual(
        Box((frame_thickness, width - frame_thickness * 2.0, frame_height)),
        origin=Origin(xyz=(-frame_x, 0.0, 0.011)),
        material=frame_material,
        name="left_frame",
    )

    part.visual(
        Box((glass_length, glass_width, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=glass_material,
        name="glass",
    )

    part.visual(
        Box((0.020, 0.170, 0.010)),
        origin=Origin(
            xyz=(handle_sign * (LID_LENGTH * 0.5 - 0.010), 0.0, 0.021),
        ),
        material=frame_material,
        name="handle",
    )


def _add_control_knob(part, *, body_material: str, accent_material: str) -> None:
    part.visual(
        Cylinder(radius=0.0055, length=0.018),
        origin=Origin(xyz=(0.009, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=accent_material,
        name="shaft",
    )
    part.visual(
        Cylinder(radius=0.026, length=0.006),
        origin=Origin(xyz=(0.015, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body_material,
        name="skirt",
    )
    part.visual(
        Cylinder(radius=0.021, length=0.018),
        origin=Origin(xyz=(0.027, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body_material,
        name="body",
    )
    part.visual(
        Box((0.010, 0.003, 0.012)),
        origin=Origin(xyz=(0.035, 0.0, 0.015)),
        material=accent_material,
        name="pointer",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="island_display_freezer")

    cabinet_white = model.material("cabinet_white", rgba=(0.94, 0.95, 0.96, 1.0))
    trim_grey = model.material("trim_grey", rgba=(0.49, 0.52, 0.56, 1.0))
    rail_aluminum = model.material("rail_aluminum", rgba=(0.76, 0.79, 0.82, 1.0))
    frame_aluminum = model.material("frame_aluminum", rgba=(0.70, 0.73, 0.76, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.67, 0.79, 0.86, 0.32))
    control_black = model.material("control_black", rgba=(0.12, 0.13, 0.15, 1.0))
    control_silver = model.material("control_silver", rgba=(0.78, 0.80, 0.82, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(_build_cabinet_shell(), "island_display_freezer_shell"),
        material=cabinet_white,
        name="shell",
    )

    cabinet.visual(
        Box((CABINET_LENGTH - 0.08, 0.024, 0.130)),
        origin=Origin(xyz=(0.0, CABINET_WIDTH * 0.5 - 0.012, 0.065)),
        material=trim_grey,
        name="front_skirt",
    )
    cabinet.visual(
        Box((CABINET_LENGTH - 0.08, 0.024, 0.130)),
        origin=Origin(xyz=(0.0, -CABINET_WIDTH * 0.5 + 0.012, 0.065)),
        material=trim_grey,
        name="rear_skirt",
    )
    cabinet.visual(
        Box((0.024, CABINET_WIDTH - 0.10, 0.130)),
        origin=Origin(xyz=(CABINET_LENGTH * 0.5 - 0.012, 0.0, 0.065)),
        material=trim_grey,
        name="right_skirt",
    )
    cabinet.visual(
        Box((0.024, CABINET_WIDTH - 0.10, 0.130)),
        origin=Origin(xyz=(-CABINET_LENGTH * 0.5 + 0.012, 0.0, 0.065)),
        material=trim_grey,
        name="left_skirt",
    )

    cabinet.visual(
        Box((TRACK_LENGTH, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, LOWER_TRACK_Y, CABINET_HEIGHT + 0.005)),
        material=rail_aluminum,
        name="lower_front_track",
    )
    cabinet.visual(
        Box((TRACK_LENGTH, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, -LOWER_TRACK_Y, CABINET_HEIGHT + 0.005)),
        material=rail_aluminum,
        name="lower_rear_track",
    )
    for index, x_pos in enumerate((-0.790, 0.790)):
        cabinet.visual(
            Box((0.040, 0.018, 0.010)),
            origin=Origin(xyz=(x_pos, LOWER_TRACK_Y, CABINET_HEIGHT + 0.005)),
            material=rail_aluminum,
            name=f"lower_front_stop_{index}",
        )
        cabinet.visual(
            Box((0.040, 0.018, 0.010)),
            origin=Origin(xyz=(x_pos, -LOWER_TRACK_Y, CABINET_HEIGHT + 0.005)),
            material=rail_aluminum,
            name=f"lower_rear_stop_{index}",
        )
    cabinet.visual(
        Box((TRACK_LENGTH, 0.022, 0.034)),
        origin=Origin(xyz=(0.0, UPPER_TRACK_Y, CABINET_HEIGHT + 0.017)),
        material=rail_aluminum,
        name="upper_front_track",
    )
    cabinet.visual(
        Box((TRACK_LENGTH, 0.022, 0.034)),
        origin=Origin(xyz=(0.0, -UPPER_TRACK_Y, CABINET_HEIGHT + 0.017)),
        material=rail_aluminum,
        name="upper_rear_track",
    )

    cabinet.visual(
        Box((0.010, 0.170, 0.220)),
        origin=Origin(xyz=(CABINET_LENGTH * 0.5 + 0.005, 0.0, 0.510)),
        material=trim_grey,
        name="control_plate",
    )

    upper_lid = model.part("upper_lid")
    _add_sliding_lid(
        upper_lid,
        width=UPPER_LID_WIDTH,
        track_y=UPPER_TRACK_Y,
        handle_sign=1.0,
        frame_material="frame_aluminum",
        glass_material="smoked_glass",
    )

    lower_lid = model.part("lower_lid")
    _add_sliding_lid(
        lower_lid,
        width=LOWER_LID_WIDTH,
        track_y=LOWER_TRACK_Y,
        handle_sign=-1.0,
        frame_material="frame_aluminum",
        glass_material="smoked_glass",
    )

    control_knob = model.part("control_knob")
    _add_control_knob(
        control_knob,
        body_material="control_black",
        accent_material="control_silver",
    )

    model.articulation(
        "cabinet_to_upper_lid",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=upper_lid,
        origin=Origin(xyz=(-LID_CLOSED_X, 0.0, UPPER_TRACK_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.45,
            lower=0.0,
            upper=LID_TRAVEL,
        ),
    )
    model.articulation(
        "cabinet_to_lower_lid",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=lower_lid,
        origin=Origin(xyz=(LID_CLOSED_X, 0.0, LOWER_TRACK_TOP_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.45,
            lower=0.0,
            upper=LID_TRAVEL,
        ),
    )
    model.articulation(
        "cabinet_to_control_knob",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=control_knob,
        origin=Origin(xyz=(CABINET_LENGTH * 0.5 + 0.010, 0.0, 0.510)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    upper_lid = object_model.get_part("upper_lid")
    lower_lid = object_model.get_part("lower_lid")
    control_knob = object_model.get_part("control_knob")

    upper_slide = object_model.get_articulation("cabinet_to_upper_lid")
    lower_slide = object_model.get_articulation("cabinet_to_lower_lid")
    knob_joint = object_model.get_articulation("cabinet_to_control_knob")

    ctx.check(
        "upper lid uses a rightward prismatic track",
        upper_slide.articulation_type == ArticulationType.PRISMATIC and upper_slide.axis == (1.0, 0.0, 0.0),
        details=f"type={upper_slide.articulation_type}, axis={upper_slide.axis}",
    )
    ctx.check(
        "lower lid uses a leftward prismatic track",
        lower_slide.articulation_type == ArticulationType.PRISMATIC and lower_slide.axis == (-1.0, 0.0, 0.0),
        details=f"type={lower_slide.articulation_type}, axis={lower_slide.axis}",
    )
    ctx.check(
        "temperature knob rotates continuously on the end wall",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS and knob_joint.axis == (1.0, 0.0, 0.0),
        details=f"type={knob_joint.articulation_type}, axis={knob_joint.axis}",
    )

    ctx.expect_gap(
        upper_lid,
        cabinet,
        axis="z",
        positive_elem="front_runner",
        negative_elem="upper_front_track",
        max_gap=0.001,
        max_penetration=0.0,
        name="upper lid front runner seats on the upper front rail",
    )
    ctx.expect_gap(
        upper_lid,
        cabinet,
        axis="z",
        positive_elem="rear_runner",
        negative_elem="upper_rear_track",
        max_gap=0.001,
        max_penetration=0.0,
        name="upper lid rear runner seats on the upper rear rail",
    )
    ctx.expect_gap(
        lower_lid,
        cabinet,
        axis="z",
        positive_elem="front_runner",
        negative_elem="lower_front_track",
        max_gap=0.001,
        max_penetration=0.0,
        name="lower lid front runner seats on the lower front rail",
    )
    ctx.expect_gap(
        lower_lid,
        cabinet,
        axis="z",
        positive_elem="rear_runner",
        negative_elem="lower_rear_track",
        max_gap=0.001,
        max_penetration=0.0,
        name="lower lid rear runner seats on the lower rear rail",
    )

    ctx.expect_overlap(
        upper_lid,
        lower_lid,
        axes="x",
        min_overlap=0.05,
        name="closed lids overlap at the center split",
    )
    ctx.expect_gap(
        control_knob,
        cabinet,
        axis="x",
        positive_elem="shaft",
        negative_elem="control_plate",
        max_gap=0.001,
        max_penetration=0.0,
        name="control knob shaft seats against the control plate",
    )

    upper_rest = ctx.part_world_position(upper_lid)
    upper_limit = upper_slide.motion_limits.upper if upper_slide.motion_limits is not None else None
    upper_open = None
    if upper_limit is not None:
        with ctx.pose({upper_slide: upper_limit}):
            ctx.expect_gap(
                upper_lid,
                cabinet,
                axis="z",
                positive_elem="front_runner",
                negative_elem="upper_front_track",
                max_gap=0.001,
                max_penetration=0.0,
                name="upper lid stays seated on the upper front rail when opened",
            )
            ctx.expect_overlap(
                upper_lid,
                cabinet,
                axes="x",
                elem_a="front_runner",
                elem_b="upper_front_track",
                min_overlap=0.18,
                name="upper lid retains engagement with the upper rail at full travel",
            )
            upper_open = ctx.part_world_position(upper_lid)
    ctx.check(
        "upper lid opens toward the opposite end",
        upper_rest is not None and upper_open is not None and upper_open[0] > upper_rest[0] + 0.25,
        details=f"rest={upper_rest}, open={upper_open}",
    )

    lower_rest = ctx.part_world_position(lower_lid)
    lower_limit = lower_slide.motion_limits.upper if lower_slide.motion_limits is not None else None
    lower_open = None
    if lower_limit is not None:
        with ctx.pose({lower_slide: lower_limit}):
            ctx.expect_gap(
                lower_lid,
                cabinet,
                axis="z",
                positive_elem="rear_runner",
                negative_elem="lower_rear_track",
                max_gap=0.001,
                max_penetration=0.0,
                name="lower lid stays seated on the lower rear rail when opened",
            )
            ctx.expect_overlap(
                lower_lid,
                cabinet,
                axes="x",
                elem_a="rear_runner",
                elem_b="lower_rear_track",
                min_overlap=0.18,
                name="lower lid retains engagement with the lower rail at full travel",
            )
            lower_open = ctx.part_world_position(lower_lid)
    ctx.check(
        "lower lid opens toward the opposite end",
        lower_rest is not None and lower_open is not None and lower_open[0] < lower_rest[0] - 0.25,
        details=f"rest={lower_rest}, open={lower_open}",
    )

    return ctx.report()


object_model = build_object_model()
