from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


DESK_LENGTH = 2.40
DESK_DEPTH = 0.90
TOP_THICKNESS = 0.038
FRAME_RAIL_THICKNESS = 0.040
FRAME_RAIL_WIDTH = 0.070
FRAME_RAIL_LENGTH = 2.20
FRAME_RAIL_SPAN_Y = 0.270
COLUMN_SPAN_X = 0.82

INNER_SIZE = (0.086, 0.056, 0.660)
OUTER_SIZE = (0.110, 0.080, 0.560)
OUTER_BASE_HEIGHT = 0.040
OUTER_FOOT_SIZE = (0.120, 0.700, 0.030)
LIFT_OFFSET = 0.100
LIFT_TRAVEL = 0.420

TRAY_SIZE = (0.820, 0.300, 0.018)
TRAY_RUNNER_SIZE = (0.012, 0.280, 0.016)
TRAY_GUIDE_SIZE = (0.018, 0.380, 0.024)
TRAY_CLOSED_Y = -0.295
TRAY_Z = -0.029
TRAY_TRAVEL = 0.220


def _add_box(part, size, xyz, *, material: str, name: str) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _build_inner_stage(part, *, material: str) -> None:
    _add_box(
        part,
        (0.160, 0.100, 0.010),
        (0.0, 0.0, -0.005),
        material=material,
        name="top_plate",
    )
    _add_box(
        part,
        INNER_SIZE,
        (0.0, 0.0, -INNER_SIZE[2] / 2.0),
        material=material,
        name="inner_member",
    )


def _build_outer_stage(part, *, material: str) -> None:
    wall = 0.006
    _add_box(
        part,
        (OUTER_SIZE[0], wall, OUTER_SIZE[2]),
        (0.0, -(OUTER_SIZE[1] / 2.0) + (wall / 2.0), -OUTER_SIZE[2] / 2.0),
        material=material,
        name="front_wall",
    )
    _add_box(
        part,
        (OUTER_SIZE[0], wall, OUTER_SIZE[2]),
        (0.0, (OUTER_SIZE[1] / 2.0) - (wall / 2.0), -OUTER_SIZE[2] / 2.0),
        material=material,
        name="rear_wall",
    )
    _add_box(
        part,
        (wall, OUTER_SIZE[1], OUTER_SIZE[2]),
        (-(OUTER_SIZE[0] / 2.0) + (wall / 2.0), 0.0, -OUTER_SIZE[2] / 2.0),
        material=material,
        name="left_wall",
    )
    _add_box(
        part,
        (wall, OUTER_SIZE[1], OUTER_SIZE[2]),
        ((OUTER_SIZE[0] / 2.0) - (wall / 2.0), 0.0, -OUTER_SIZE[2] / 2.0),
        material=material,
        name="right_wall",
    )
    _add_box(
        part,
        (0.150, 0.120, OUTER_BASE_HEIGHT),
        (0.0, 0.0, -(OUTER_SIZE[2] + OUTER_BASE_HEIGHT / 2.0)),
        material=material,
        name="foot_mount",
    )
    _add_box(
        part,
        OUTER_FOOT_SIZE,
        (
            0.0,
            0.0,
            -(OUTER_SIZE[2] + OUTER_BASE_HEIGHT + OUTER_FOOT_SIZE[2] / 2.0),
        ),
        material=material,
        name="foot",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_standing_desk")

    model.material("top_wood", rgba=(0.67, 0.54, 0.37, 1.0))
    model.material("frame_steel", rgba=(0.19, 0.21, 0.23, 1.0))
    model.material("column_steel", rgba=(0.83, 0.85, 0.88, 1.0))
    model.material("plastic_black", rgba=(0.08, 0.08, 0.09, 1.0))

    frame = model.part("frame")
    _add_box(
        frame,
        (DESK_LENGTH, DESK_DEPTH, TOP_THICKNESS),
        (0.0, 0.0, 0.059),
        material="top_wood",
        name="desktop",
    )
    _add_box(
        frame,
        (FRAME_RAIL_LENGTH, FRAME_RAIL_WIDTH, FRAME_RAIL_THICKNESS),
        (0.0, -FRAME_RAIL_SPAN_Y, 0.028),
        material="frame_steel",
        name="front_rail",
    )
    _add_box(
        frame,
        (FRAME_RAIL_LENGTH, FRAME_RAIL_WIDTH, FRAME_RAIL_THICKNESS),
        (0.0, FRAME_RAIL_SPAN_Y, 0.028),
        material="frame_steel",
        name="rear_rail",
    )
    for name, x in (("left_crossmember", -COLUMN_SPAN_X), ("center_crossmember", 0.0), ("right_crossmember", COLUMN_SPAN_X)):
        _add_box(
            frame,
            (0.100, 0.610, FRAME_RAIL_THICKNESS),
            (x, 0.0, 0.028),
            material="frame_steel",
            name=name,
        )
    for name, x in (("left_pad", -COLUMN_SPAN_X), ("center_pad", 0.0), ("right_pad", COLUMN_SPAN_X)):
        _add_box(
            frame,
            (0.180, 0.140, 0.010),
            (x, 0.0, 0.005),
            material="frame_steel",
            name=name,
        )
    _add_box(
        frame,
        TRAY_GUIDE_SIZE,
        (-0.406, -0.295, -0.004),
        material="frame_steel",
        name="left_guide",
    )
    _add_box(
        frame,
        TRAY_GUIDE_SIZE,
        (0.406, -0.295, -0.004),
        material="frame_steel",
        name="right_guide",
    )

    left_inner = model.part("left_inner")
    center_inner = model.part("center_inner")
    right_inner = model.part("right_inner")
    for part in (left_inner, center_inner, right_inner):
        _build_inner_stage(part, material="column_steel")

    left_outer = model.part("left_outer")
    center_outer = model.part("center_outer")
    right_outer = model.part("right_outer")
    for part in (left_outer, center_outer, right_outer):
        _build_outer_stage(part, material="column_steel")

    tray = model.part("tray")
    _add_box(
        tray,
        TRAY_SIZE,
        (0.0, 0.0, -TRAY_SIZE[2] / 2.0),
        material="frame_steel",
        name="tray_panel",
    )
    _add_box(
        tray,
        (TRAY_SIZE[0], 0.016, 0.028),
        (0.0, -(TRAY_SIZE[1] / 2.0) + 0.008, -0.014),
        material="frame_steel",
        name="front_lip",
    )
    _add_box(
        tray,
        TRAY_RUNNER_SIZE,
        (-0.391, 0.0, 0.008),
        material="frame_steel",
        name="left_runner",
    )
    _add_box(
        tray,
        TRAY_RUNNER_SIZE,
        (0.391, 0.0, 0.008),
        material="frame_steel",
        name="right_runner",
    )

    controller = model.part("controller")
    _add_box(
        controller,
        (0.160, 0.075, 0.018),
        (0.0, 0.0, -0.009),
        material="plastic_black",
        name="housing",
    )
    _add_box(
        controller,
        (0.116, 0.056, 0.012),
        (0.0, -0.010, -0.024),
        material="plastic_black",
        name="housing_nose",
    )
    _add_box(
        controller,
        (0.070, 0.075, 0.010),
        (0.0, 0.0, -0.023),
        material="plastic_black",
        name="housing_back",
    )

    paddle = model.part("paddle")
    _add_box(
        paddle,
        (0.072, 0.050, 0.008),
        (0.0, -0.050, -0.014),
        material="plastic_black",
        name="paddle_blade",
    )
    _add_box(
        paddle,
        (0.018, 0.026, 0.020),
        (0.0, -0.016, -0.010),
        material="plastic_black",
        name="paddle_stem",
    )

    model.articulation(
        "frame_to_left_inner",
        ArticulationType.FIXED,
        parent=frame,
        child=left_inner,
        origin=Origin(xyz=(-COLUMN_SPAN_X, 0.0, 0.0)),
    )
    model.articulation(
        "frame_to_center_inner",
        ArticulationType.FIXED,
        parent=frame,
        child=center_inner,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    model.articulation(
        "frame_to_right_inner",
        ArticulationType.FIXED,
        parent=frame,
        child=right_inner,
        origin=Origin(xyz=(COLUMN_SPAN_X, 0.0, 0.0)),
    )
    model.articulation(
        "center_lift",
        ArticulationType.PRISMATIC,
        parent=center_inner,
        child=center_outer,
        origin=Origin(xyz=(0.0, 0.0, -LIFT_OFFSET)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=LIFT_TRAVEL, effort=1500.0, velocity=0.05),
    )
    model.articulation(
        "left_lift",
        ArticulationType.PRISMATIC,
        parent=left_inner,
        child=left_outer,
        origin=Origin(xyz=(0.0, 0.0, -LIFT_OFFSET)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=LIFT_TRAVEL, effort=1500.0, velocity=0.05),
        mimic=Mimic(joint="center_lift"),
    )
    model.articulation(
        "right_lift",
        ArticulationType.PRISMATIC,
        parent=right_inner,
        child=right_outer,
        origin=Origin(xyz=(0.0, 0.0, -LIFT_OFFSET)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=LIFT_TRAVEL, effort=1500.0, velocity=0.05),
        mimic=Mimic(joint="center_lift"),
    )
    model.articulation(
        "tray_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=tray,
        origin=Origin(xyz=(0.0, TRAY_CLOSED_Y, TRAY_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=TRAY_TRAVEL, effort=80.0, velocity=0.25),
    )
    model.articulation(
        "frame_to_controller",
        ArticulationType.FIXED,
        parent=frame,
        child=controller,
        origin=Origin(xyz=(0.0, -0.392, 0.040)),
    )
    model.articulation(
        "paddle_tilt",
        ArticulationType.REVOLUTE,
        parent=controller,
        child=paddle,
        origin=Origin(xyz=(0.0, -0.010, -0.030)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.30, upper=0.30, effort=4.0, velocity=2.0),
    )

    return model


def _check_column_fit(ctx: TestContext, inner, outer, prefix: str, overlap_min: float) -> None:
    ctx.expect_gap(
        outer,
        inner,
        axis="x",
        positive_elem="right_wall",
        negative_elem="inner_member",
        min_gap=0.004,
        max_gap=0.008,
        name=f"{prefix} right wall clearance",
    )
    ctx.expect_gap(
        inner,
        outer,
        axis="x",
        positive_elem="inner_member",
        negative_elem="left_wall",
        min_gap=0.004,
        max_gap=0.008,
        name=f"{prefix} left wall clearance",
    )
    ctx.expect_gap(
        outer,
        inner,
        axis="y",
        positive_elem="rear_wall",
        negative_elem="inner_member",
        min_gap=0.004,
        max_gap=0.008,
        name=f"{prefix} rear wall clearance",
    )
    ctx.expect_gap(
        inner,
        outer,
        axis="y",
        positive_elem="inner_member",
        negative_elem="front_wall",
        min_gap=0.004,
        max_gap=0.008,
        name=f"{prefix} front wall clearance",
    )
    ctx.expect_overlap(
        inner,
        outer,
        axes="z",
        elem_a="inner_member",
        elem_b="front_wall",
        min_overlap=overlap_min,
        name=f"{prefix} retained insertion",
    )


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    tray = object_model.get_part("tray")
    controller = object_model.get_part("controller")
    paddle = object_model.get_part("paddle")
    left_inner = object_model.get_part("left_inner")
    center_inner = object_model.get_part("center_inner")
    right_inner = object_model.get_part("right_inner")
    left_outer = object_model.get_part("left_outer")
    center_outer = object_model.get_part("center_outer")
    right_outer = object_model.get_part("right_outer")

    center_lift = object_model.get_articulation("center_lift")
    tray_slide = object_model.get_articulation("tray_slide")
    paddle_tilt = object_model.get_articulation("paddle_tilt")

    lift_upper = center_lift.motion_limits.upper if center_lift.motion_limits is not None else 0.0
    tray_upper = tray_slide.motion_limits.upper if tray_slide.motion_limits is not None else 0.0
    paddle_upper = paddle_tilt.motion_limits.upper if paddle_tilt.motion_limits is not None else 0.0

    ctx.expect_contact(
        left_inner,
        frame,
        elem_a="top_plate",
        elem_b="left_pad",
        name="left column head mounts to frame",
    )
    ctx.expect_contact(
        center_inner,
        frame,
        elem_a="top_plate",
        elem_b="center_pad",
        name="center column head mounts to frame",
    )
    ctx.expect_contact(
        right_inner,
        frame,
        elem_a="top_plate",
        elem_b="right_pad",
        name="right column head mounts to frame",
    )

    _check_column_fit(ctx, left_inner, left_outer, "left column", overlap_min=0.54)
    _check_column_fit(ctx, center_inner, center_outer, "center column", overlap_min=0.54)
    _check_column_fit(ctx, right_inner, right_outer, "right column", overlap_min=0.54)

    rest_left = ctx.part_world_position(left_outer)
    rest_center = ctx.part_world_position(center_outer)
    rest_right = ctx.part_world_position(right_outer)
    rest_tray = ctx.part_world_position(tray)
    rest_paddle_aabb = ctx.part_element_world_aabb(paddle, elem="paddle_blade")

    ctx.expect_contact(
        tray,
        frame,
        elem_a="left_runner",
        elem_b="left_guide",
        name="left tray runner contacts left guide",
    )
    ctx.expect_contact(
        tray,
        frame,
        elem_a="right_runner",
        elem_b="right_guide",
        name="right tray runner contacts right guide",
    )
    ctx.expect_overlap(
        tray,
        frame,
        axes="y",
        elem_a="left_runner",
        elem_b="left_guide",
        min_overlap=0.27,
        name="closed tray remains captured in left guide",
    )
    ctx.expect_overlap(
        tray,
        frame,
        axes="y",
        elem_a="right_runner",
        elem_b="right_guide",
        min_overlap=0.27,
        name="closed tray remains captured in right guide",
    )
    ctx.expect_gap(
        frame,
        tray,
        axis="z",
        positive_elem="desktop",
        negative_elem="tray_panel",
        min_gap=0.045,
        max_gap=0.085,
        name="tray hangs below the desktop",
    )
    ctx.expect_contact(
        controller,
        frame,
        elem_a="housing",
        elem_b="desktop",
        name="controller housing mounts under desktop",
    )

    with ctx.pose({center_lift: lift_upper}):
        _check_column_fit(ctx, left_inner, left_outer, "left column extended", overlap_min=0.13)
        _check_column_fit(ctx, center_inner, center_outer, "center column extended", overlap_min=0.13)
        _check_column_fit(ctx, right_inner, right_outer, "right column extended", overlap_min=0.13)

        lifted_left = ctx.part_world_position(left_outer)
        lifted_center = ctx.part_world_position(center_outer)
        lifted_right = ctx.part_world_position(right_outer)
        if all(pos is not None for pos in (rest_left, rest_center, rest_right, lifted_left, lifted_center, lifted_right)):
            left_drop = rest_left[2] - lifted_left[2]
            center_drop = rest_center[2] - lifted_center[2]
            right_drop = rest_right[2] - lifted_right[2]
            ctx.check(
                "all three lifts stay synchronized",
                abs(left_drop - center_drop) < 1e-6
                and abs(right_drop - center_drop) < 1e-6
                and center_drop > 0.40,
                details=f"left={left_drop}, center={center_drop}, right={right_drop}",
            )

    with ctx.pose({tray_slide: tray_upper}):
        ctx.expect_contact(
            tray,
            frame,
            elem_a="left_runner",
            elem_b="left_guide",
            name="left tray runner stays aligned when extended",
        )
        ctx.expect_contact(
            tray,
            frame,
            elem_a="right_runner",
            elem_b="right_guide",
            name="right tray runner stays aligned when extended",
        )
        ctx.expect_overlap(
            tray,
            frame,
            axes="y",
            elem_a="left_runner",
            elem_b="left_guide",
            min_overlap=0.10,
            name="left guide retains tray at full extension",
        )
        ctx.expect_overlap(
            tray,
            frame,
            axes="y",
            elem_a="right_runner",
            elem_b="right_guide",
            min_overlap=0.10,
            name="right guide retains tray at full extension",
        )

        extended_tray = ctx.part_world_position(tray)
        if rest_tray is not None and extended_tray is not None:
            ctx.check(
                "keyboard tray slides forward",
                extended_tray[1] < rest_tray[1] - 0.18,
                details=f"rest={rest_tray}, extended={extended_tray}",
            )

    with ctx.pose({paddle_tilt: paddle_upper}):
        tipped_paddle_aabb = ctx.part_element_world_aabb(paddle, elem="paddle_blade")
        if rest_paddle_aabb is not None and tipped_paddle_aabb is not None:
            ctx.check(
                "paddle tips downward",
                tipped_paddle_aabb[0][2] < rest_paddle_aabb[0][2] - 0.008,
                details=f"rest={rest_paddle_aabb}, tipped={tipped_paddle_aabb}",
            )

    return ctx.report()


object_model = build_object_model()
