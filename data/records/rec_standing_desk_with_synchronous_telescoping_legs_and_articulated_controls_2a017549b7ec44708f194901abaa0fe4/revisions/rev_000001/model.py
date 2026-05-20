from __future__ import annotations

import math

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


DESK_WIDTH = 0.92
DESK_DEPTH = 0.50
TOP_THICKNESS = 0.024
LEG_SPACING = 0.52
STAGE_SIZE = (0.064, 0.034, 0.58)
OUTER_SIZE = (0.078, 0.050, 0.54)
LIFT_TRAVEL = 0.30


def _build_top(desktop, *, top_oak, frame_graphite, frame_black) -> None:
    desktop.visual(
        Box((DESK_WIDTH, DESK_DEPTH, TOP_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, TOP_THICKNESS / 2.0)),
        material=top_oak,
        name="top_panel",
    )
    desktop.visual(
        Box((0.70, 0.035, 0.040)),
        origin=Origin(xyz=(0.0, -0.110, -0.020)),
        material=frame_graphite,
        name="front_rail",
    )
    desktop.visual(
        Box((0.70, 0.035, 0.040)),
        origin=Origin(xyz=(0.0, 0.110, -0.020)),
        material=frame_graphite,
        name="rear_rail",
    )
    desktop.visual(
        Box((0.12, 0.22, 0.040)),
        origin=Origin(xyz=(-LEG_SPACING / 2.0, 0.0, -0.020)),
        material=frame_graphite,
        name="end_rail_0",
    )
    desktop.visual(
        Box((0.15, 0.12, 0.012)),
        origin=Origin(xyz=(-LEG_SPACING / 2.0, 0.0, -0.046)),
        material=frame_black,
        name="left_mount",
    )
    desktop.visual(
        Box((0.12, 0.22, 0.040)),
        origin=Origin(xyz=(LEG_SPACING / 2.0, 0.0, -0.020)),
        material=frame_graphite,
        name="end_rail_1",
    )
    desktop.visual(
        Box((0.15, 0.12, 0.012)),
        origin=Origin(xyz=(LEG_SPACING / 2.0, 0.0, -0.046)),
        material=frame_black,
        name="right_mount",
    )


def _build_stage(stage, *, stage_metal, cap_black) -> None:
    stage.visual(
        Box((0.090, 0.060, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=cap_black,
        name="carriage_cap",
    )
    stage.visual(
        Box(STAGE_SIZE),
        origin=Origin(xyz=(0.0, 0.0, -STAGE_SIZE[2] / 2.0)),
        material=stage_metal,
        name="stage",
    )
    stage.visual(
        Box((0.004, 0.020, 0.160)),
        origin=Origin(xyz=(0.034, 0.0, -0.120)),
        material=cap_black,
        name="guide_pad_0",
    )
    stage.visual(
        Box((0.004, 0.020, 0.160)),
        origin=Origin(xyz=(-0.034, 0.0, -0.120)),
        material=cap_black,
        name="guide_pad_1",
    )


def _build_outer_leg(outer, *, shell_black, foot_black) -> None:
    outer_x, outer_y, outer_z = OUTER_SIZE
    wall = 0.003
    outer.visual(
        Box((outer_x, wall, outer_z)),
        origin=Origin(xyz=(0.0, outer_y / 2.0 - wall / 2.0, -outer_z / 2.0)),
        material=shell_black,
        name="front_wall",
    )
    outer.visual(
        Box((outer_x, wall, outer_z)),
        origin=Origin(xyz=(0.0, -outer_y / 2.0 + wall / 2.0, -outer_z / 2.0)),
        material=shell_black,
        name="rear_wall",
    )
    outer.visual(
        Box((wall, outer_y, outer_z)),
        origin=Origin(xyz=(outer_x / 2.0 - wall / 2.0, 0.0, -outer_z / 2.0)),
        material=shell_black,
        name="side_wall_0",
    )
    outer.visual(
        Box((wall, outer_y, outer_z)),
        origin=Origin(xyz=(-outer_x / 2.0 + wall / 2.0, 0.0, -outer_z / 2.0)),
        material=shell_black,
        name="side_wall_1",
    )
    outer.visual(
        Box((0.084, 0.012, 0.054)),
        origin=Origin(xyz=(0.0, 0.031, -0.566)),
        material=shell_black,
        name="gusset_0",
    )
    outer.visual(
        Box((0.084, 0.012, 0.054)),
        origin=Origin(xyz=(0.0, -0.031, -0.566)),
        material=shell_black,
        name="gusset_1",
    )
    outer.visual(
        Box((0.082, 0.58, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, -0.605)),
        material=foot_black,
        name="foot_bar",
    )
    outer.visual(
        Box((0.090, 0.070, 0.006)),
        origin=Origin(xyz=(0.0, 0.245, -0.621)),
        material=foot_black,
        name="foot_pad_0",
    )
    outer.visual(
        Box((0.090, 0.070, 0.006)),
        origin=Origin(xyz=(0.0, -0.245, -0.621)),
        material=foot_black,
        name="foot_pad_1",
    )


def _build_control_pod(control_pod, *, shell_black, frame_black) -> None:
    control_pod.visual(
        Box((0.076, 0.028, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=frame_black,
        name="mount_plate",
    )
    control_pod.visual(
        Box((0.114, 0.024, 0.018)),
        origin=Origin(xyz=(0.0, 0.010, -0.013)),
        material=shell_black,
        name="pod_shell",
    )
    control_pod.visual(
        Box((0.008, 0.016, 0.018)),
        origin=Origin(xyz=(-0.036, -0.010, -0.017)),
        material=frame_black,
        name="side_cheek_0",
    )
    control_pod.visual(
        Box((0.008, 0.016, 0.018)),
        origin=Origin(xyz=(0.036, -0.010, -0.017)),
        material=frame_black,
        name="side_cheek_1",
    )
    control_pod.visual(
        Box((0.006, 0.016, 0.018)),
        origin=Origin(xyz=(0.0, -0.010, -0.017)),
        material=frame_black,
        name="center_divider",
    )


def _build_paddle(paddle, *, stage_metal) -> None:
    paddle.visual(
        Box((0.028, 0.010, 0.032)),
        origin=Origin(xyz=(0.0, -0.005, -0.018)),
        material=stage_metal,
        name="paddle",
    )
    paddle.visual(
        Box((0.018, 0.006, 0.006)),
        origin=Origin(xyz=(0.0, 0.000, -0.003)),
        material=stage_metal,
        name="pivot_tab",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_standing_desk")

    top_oak = model.material("top_oak", rgba=(0.72, 0.58, 0.40, 1.0))
    frame_graphite = model.material("frame_graphite", rgba=(0.23, 0.24, 0.26, 1.0))
    frame_black = model.material("frame_black", rgba=(0.12, 0.13, 0.14, 1.0))
    stage_metal = model.material("stage_metal", rgba=(0.60, 0.62, 0.65, 1.0))
    shell_black = model.material("shell_black", rgba=(0.16, 0.17, 0.18, 1.0))
    foot_black = model.material("foot_black", rgba=(0.18, 0.19, 0.20, 1.0))

    desktop = model.part("desktop")
    _build_top(
        desktop,
        top_oak=top_oak,
        frame_graphite=frame_graphite,
        frame_black=frame_black,
    )

    left_stage = model.part("left_stage")
    _build_stage(left_stage, stage_metal=stage_metal, cap_black=frame_black)
    right_stage = model.part("right_stage")
    _build_stage(right_stage, stage_metal=stage_metal, cap_black=frame_black)

    left_outer = model.part("left_outer")
    _build_outer_leg(left_outer, shell_black=shell_black, foot_black=foot_black)
    right_outer = model.part("right_outer")
    _build_outer_leg(right_outer, shell_black=shell_black, foot_black=foot_black)

    control_pod = model.part("control_pod")
    _build_control_pod(control_pod, shell_black=shell_black, frame_black=frame_black)
    up_paddle = model.part("up_paddle")
    _build_paddle(up_paddle, stage_metal=stage_metal)
    down_paddle = model.part("down_paddle")
    _build_paddle(down_paddle, stage_metal=stage_metal)

    model.articulation(
        "desktop_to_left_stage",
        ArticulationType.FIXED,
        parent=desktop,
        child=left_stage,
        origin=Origin(xyz=(-LEG_SPACING / 2.0, 0.0, -0.052)),
    )
    model.articulation(
        "desktop_to_right_stage",
        ArticulationType.FIXED,
        parent=desktop,
        child=right_stage,
        origin=Origin(xyz=(LEG_SPACING / 2.0, 0.0, -0.052)),
    )
    model.articulation(
        "left_stage_to_left_outer",
        ArticulationType.PRISMATIC,
        parent=left_stage,
        child=left_outer,
        origin=Origin(xyz=(0.0, 0.0, -0.038)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=240.0,
            velocity=0.040,
            lower=0.0,
            upper=LIFT_TRAVEL,
        ),
    )
    model.articulation(
        "right_stage_to_right_outer",
        ArticulationType.PRISMATIC,
        parent=right_stage,
        child=right_outer,
        origin=Origin(xyz=(0.0, 0.0, -0.038)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=240.0,
            velocity=0.040,
            lower=0.0,
            upper=LIFT_TRAVEL,
        ),
        mimic=Mimic("left_stage_to_left_outer"),
    )
    model.articulation(
        "desktop_to_control_pod",
        ArticulationType.FIXED,
        parent=desktop,
        child=control_pod,
        origin=Origin(xyz=(0.344, -0.176, 0.0)),
    )
    model.articulation(
        "pod_to_up_paddle",
        ArticulationType.REVOLUTE,
        parent=control_pod,
        child=up_paddle,
        origin=Origin(xyz=(-0.018, -0.011, -0.006)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=4.0,
            lower=0.0,
            upper=math.radians(30.0),
        ),
    )
    model.articulation(
        "pod_to_down_paddle",
        ArticulationType.REVOLUTE,
        parent=control_pod,
        child=down_paddle,
        origin=Origin(xyz=(0.018, -0.011, -0.006)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=4.0,
            lower=0.0,
            upper=math.radians(30.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    desktop = object_model.get_part("desktop")
    left_stage = object_model.get_part("left_stage")
    right_stage = object_model.get_part("right_stage")
    left_outer = object_model.get_part("left_outer")
    right_outer = object_model.get_part("right_outer")
    control_pod = object_model.get_part("control_pod")
    up_paddle = object_model.get_part("up_paddle")
    down_paddle = object_model.get_part("down_paddle")
    left_lift = object_model.get_articulation("left_stage_to_left_outer")
    up_joint = object_model.get_articulation("pod_to_up_paddle")
    down_joint = object_model.get_articulation("pod_to_down_paddle")
    limits = left_lift.motion_limits
    upper = 0.0 if limits is None or limits.upper is None else limits.upper
    up_limits = up_joint.motion_limits
    paddle_upper = 0.0 if up_limits is None or up_limits.upper is None else up_limits.upper

    ctx.expect_gap(
        desktop,
        left_outer,
        axis="z",
        positive_elem="left_mount",
        max_gap=0.050,
        min_gap=0.020,
        name="left frame plate stays visibly above the lifting column",
    )
    ctx.expect_gap(
        desktop,
        right_outer,
        axis="z",
        positive_elem="right_mount",
        max_gap=0.050,
        min_gap=0.020,
        name="right frame plate stays visibly above the lifting column",
    )
    ctx.expect_within(
        left_stage,
        left_outer,
        axes="xy",
        elem_a="stage",
        margin=0.012,
        name="left stage stays centered in its outer column",
    )
    ctx.expect_within(
        right_stage,
        right_outer,
        axes="xy",
        elem_a="stage",
        margin=0.012,
        name="right stage stays centered in its outer column",
    )
    ctx.expect_overlap(
        left_stage,
        left_outer,
        axes="z",
        elem_a="stage",
        elem_b="front_wall",
        min_overlap=0.22,
        name="left stage remains inserted at the low position",
    )
    ctx.expect_overlap(
        right_stage,
        right_outer,
        axes="z",
        elem_a="stage",
        elem_b="front_wall",
        min_overlap=0.22,
        name="right stage remains inserted at the low position",
    )

    rest_left = ctx.part_world_position(left_outer)
    rest_right = ctx.part_world_position(right_outer)
    with ctx.pose({left_lift: upper}):
        ctx.expect_within(
            left_stage,
            left_outer,
            axes="xy",
            elem_a="stage",
            margin=0.012,
            name="left stage stays centered when extended",
        )
        ctx.expect_within(
            right_stage,
            right_outer,
            axes="xy",
            elem_a="stage",
            margin=0.012,
            name="right stage stays centered when extended",
        )
        ctx.expect_overlap(
            left_stage,
            left_outer,
            axes="z",
            elem_a="stage",
            elem_b="front_wall",
            min_overlap=0.18,
            name="left stage keeps retained insertion at full height",
        )
        ctx.expect_overlap(
            right_stage,
            right_outer,
            axes="z",
            elem_a="stage",
            elem_b="front_wall",
            min_overlap=0.18,
            name="right stage keeps retained insertion at full height",
        )
        extended_left = ctx.part_world_position(left_outer)
        extended_right = ctx.part_world_position(right_outer)

    left_drop = None if rest_left is None or extended_left is None else rest_left[2] - extended_left[2]
    right_drop = None if rest_right is None or extended_right is None else rest_right[2] - extended_right[2]
    ctx.check(
        "lift columns travel downward together as a synchronized pair",
        left_drop is not None
        and right_drop is not None
        and left_drop > 0.28
        and abs(left_drop - right_drop) < 1e-5,
        details=f"left_drop={left_drop}, right_drop={right_drop}",
    )
    ctx.expect_gap(
        down_paddle,
        up_paddle,
        axis="x",
        min_gap=0.006,
        name="controller paddles remain visibly split",
    )

    desktop_pos = ctx.part_world_position(desktop)
    pod_pos = ctx.part_world_position(control_pod)
    ctx.check(
        "control pod hangs under the front right corner",
        desktop_pos is not None
        and pod_pos is not None
        and pod_pos[0] > desktop_pos[0] + 0.25
        and pod_pos[1] < desktop_pos[1] - 0.12
        and pod_pos[2] <= desktop_pos[2],
        details=f"desktop_pos={desktop_pos}, pod_pos={pod_pos}",
    )

    up_rest = ctx.part_element_world_aabb(up_paddle, elem="paddle")
    down_rest = ctx.part_element_world_aabb(down_paddle, elem="paddle")
    with ctx.pose({up_joint: paddle_upper, down_joint: paddle_upper}):
        up_pressed = ctx.part_element_world_aabb(up_paddle, elem="paddle")
        down_pressed = ctx.part_element_world_aabb(down_paddle, elem="paddle")
        ctx.expect_gap(
            down_paddle,
            up_paddle,
            axis="x",
            min_gap=0.006,
            name="controller paddles stay separated while pressed",
        )

    up_swing = None
    down_swing = None
    if up_rest is not None and up_pressed is not None:
        up_swing = up_pressed[1][1] - up_rest[1][1]
    if down_rest is not None and down_pressed is not None:
        down_swing = down_pressed[1][1] - down_rest[1][1]
    ctx.check(
        "both paddles sweep on their local pivots",
        up_swing is not None
        and down_swing is not None
        and up_swing > 0.010
        and down_swing > 0.010,
        details=f"up_swing={up_swing}, down_swing={down_swing}",
    )

    return ctx.report()


object_model = build_object_model()
