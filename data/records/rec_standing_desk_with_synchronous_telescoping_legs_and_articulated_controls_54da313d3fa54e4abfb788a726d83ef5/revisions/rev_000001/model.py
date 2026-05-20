from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


OUTER_SIZE = 0.120
COLUMN_WALL = 0.012
FOOT_THICKNESS = 0.028
SOCKET_BASE_THICKNESS = 0.024
OUTER_HEIGHT = 0.656
STAGE_SIZE = 0.086
STAGE_LENGTH = 0.560
STAGE_CAP_SIZE = 0.132
STAGE_CAP_THICKNESS = 0.028
LIFT_TRAVEL = 0.420

TOP_THICKNESS = 0.030
UNDERFRAME_HEIGHT = 0.060
PAD_THICKNESS = 0.020
CORNER_FRAME_THICKNESS = 0.040
CORNER_FRAME_Z = OUTER_HEIGHT - 0.110

LONG_TOP_SIZE = (1.80, 0.82)
RETURN_TOP_SIZE = (0.82, 1.40)
LONG_TOP_CENTER = (0.62, -0.02)
RETURN_TOP_CENTER = (-0.02, 0.38)

FRONT_POS = (1.10, -0.04, 0.0)
RETURN_POS = (-0.04, 0.82, 0.0)


def _offset_from_front(target_xy: tuple[float, float]) -> tuple[float, float]:
    return (target_xy[0] - FRONT_POS[0], target_xy[1] - FRONT_POS[1])


def _add_outer_column(
    part,
    *,
    steel,
    foot_size: tuple[float, float, float] | None = None,
    foot_yaw: float = 0.0,
    foot_cross: bool = False,
) -> None:
    if foot_cross:
        part.visual(
            Box((0.56, 0.08, FOOT_THICKNESS)),
            origin=Origin(xyz=(0.0, 0.0, FOOT_THICKNESS / 2.0)),
            material=steel,
            name="foot_x",
        )
        part.visual(
            Box((0.08, 0.56, FOOT_THICKNESS)),
            origin=Origin(xyz=(0.0, 0.0, FOOT_THICKNESS / 2.0)),
            material=steel,
            name="foot_y",
        )
    elif foot_size is not None:
        part.visual(
            Box(foot_size),
            origin=Origin(
                xyz=(0.0, 0.0, FOOT_THICKNESS / 2.0),
                rpy=(0.0, 0.0, foot_yaw),
            ),
            material=steel,
            name="foot",
        )

    part.visual(
        Box((OUTER_SIZE, OUTER_SIZE, SOCKET_BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, FOOT_THICKNESS + SOCKET_BASE_THICKNESS / 2.0)),
        material=steel,
        name="socket_base",
    )

    wall_height = OUTER_HEIGHT - FOOT_THICKNESS - SOCKET_BASE_THICKNESS
    wall_center_z = FOOT_THICKNESS + SOCKET_BASE_THICKNESS + wall_height / 2.0
    wall_offset = (OUTER_SIZE - COLUMN_WALL) / 2.0
    inner_span = OUTER_SIZE - 2.0 * COLUMN_WALL

    part.visual(
        Box((COLUMN_WALL, OUTER_SIZE, wall_height)),
        origin=Origin(xyz=(wall_offset, 0.0, wall_center_z)),
        material=steel,
        name="shell_right",
    )
    part.visual(
        Box((COLUMN_WALL, OUTER_SIZE, wall_height)),
        origin=Origin(xyz=(-wall_offset, 0.0, wall_center_z)),
        material=steel,
        name="shell_left",
    )
    part.visual(
        Box((inner_span, COLUMN_WALL, wall_height)),
        origin=Origin(xyz=(0.0, wall_offset, wall_center_z)),
        material=steel,
        name="shell_back",
    )
    part.visual(
        Box((inner_span, COLUMN_WALL, wall_height)),
        origin=Origin(xyz=(0.0, -wall_offset, wall_center_z)),
        material=steel,
        name="shell_front",
    )


def _add_lift_stage(part, *, steel, cap) -> None:
    part.visual(
        Box((STAGE_SIZE, STAGE_SIZE, STAGE_LENGTH)),
        origin=Origin(xyz=(0.0, 0.0, -STAGE_LENGTH / 2.0)),
        material=steel,
        name="stage",
    )
    part.visual(
        Box((STAGE_CAP_SIZE, STAGE_CAP_SIZE, STAGE_CAP_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, STAGE_CAP_THICKNESS / 2.0)),
        material=cap,
        name="cap",
    )


def _add_paddle(part, *, plastic) -> None:
    part.visual(
        Cylinder(radius=0.0045, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=plastic,
        name="pivot",
    )
    part.visual(
        Box((0.036, 0.024, 0.010)),
        origin=Origin(xyz=(0.0, -0.014, -0.008)),
        material=plastic,
        name="blade",
    )
    part.visual(
        Box((0.030, 0.004, 0.006)),
        origin=Origin(xyz=(0.0, -0.028, -0.005)),
        material=plastic,
        name="lip",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="l_shaped_standing_desk")

    matte_black = model.material("matte_black", rgba=(0.18, 0.18, 0.19, 1.0))
    graphite = model.material("graphite", rgba=(0.28, 0.30, 0.33, 1.0))
    desktop_oak = model.material("desktop_oak", rgba=(0.67, 0.55, 0.42, 1.0))
    controller_black = model.material("controller_black", rgba=(0.10, 0.10, 0.11, 1.0))
    satin_black = model.material("satin_black", rgba=(0.08, 0.08, 0.09, 1.0))

    corner_column = model.part("corner_column")
    _add_outer_column(corner_column, steel=matte_black, foot_cross=True)

    front_beam = model.part("front_beam")
    front_beam.visual(
        Box((0.76, 0.080, CORNER_FRAME_THICKNESS)),
        origin=Origin(xyz=(0.38, 0.0, CORNER_FRAME_Z)),
        material=graphite,
        name="beam",
    )

    return_beam = model.part("return_beam")
    return_beam.visual(
        Box((0.080, 0.48, CORNER_FRAME_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.24, CORNER_FRAME_Z)),
        material=graphite,
        name="beam",
    )

    front_column = model.part("front_column")
    _add_outer_column(
        front_column,
        steel=matte_black,
        foot_size=(0.080, 0.700, FOOT_THICKNESS),
    )

    return_column = model.part("return_column")
    _add_outer_column(
        return_column,
        steel=matte_black,
        foot_size=(0.700, 0.080, FOOT_THICKNESS),
    )

    front_stage = model.part("front_stage")
    _add_lift_stage(front_stage, steel=graphite, cap=matte_black)

    corner_stage = model.part("corner_stage")
    _add_lift_stage(corner_stage, steel=graphite, cap=matte_black)

    return_stage = model.part("return_stage")
    _add_lift_stage(return_stage, steel=graphite, cap=matte_black)

    desktop = model.part("desktop")

    front_pad_xy = (0.0, 0.0)
    corner_pad_xy = _offset_from_front((0.0, 0.0))
    return_pad_xy = _offset_from_front((RETURN_POS[0], RETURN_POS[1]))
    long_center_xy = _offset_from_front(LONG_TOP_CENTER)
    return_center_xy = _offset_from_front(RETURN_TOP_CENTER)

    desktop.visual(
        Box((0.160, 0.160, PAD_THICKNESS)),
        origin=Origin(xyz=(front_pad_xy[0], front_pad_xy[1], PAD_THICKNESS / 2.0)),
        material=graphite,
        name="front_pad",
    )
    desktop.visual(
        Box((0.160, 0.160, PAD_THICKNESS)),
        origin=Origin(xyz=(corner_pad_xy[0], corner_pad_xy[1], PAD_THICKNESS / 2.0)),
        material=graphite,
        name="corner_pad",
    )
    desktop.visual(
        Box((0.160, 0.160, PAD_THICKNESS)),
        origin=Origin(xyz=(return_pad_xy[0], return_pad_xy[1], PAD_THICKNESS / 2.0)),
        material=graphite,
        name="return_pad",
    )
    desktop.visual(
        Box((1.18, 0.100, UNDERFRAME_HEIGHT)),
        origin=Origin(xyz=(-0.55, -0.02, UNDERFRAME_HEIGHT / 2.0)),
        material=graphite,
        name="frame_long",
    )
    desktop.visual(
        Box((0.100, 0.90, UNDERFRAME_HEIGHT)),
        origin=Origin(xyz=(-1.12, 0.41, UNDERFRAME_HEIGHT / 2.0)),
        material=graphite,
        name="frame_return",
    )
    desktop.visual(
        Box((LONG_TOP_SIZE[0], LONG_TOP_SIZE[1], TOP_THICKNESS)),
        origin=Origin(
            xyz=(
                long_center_xy[0],
                long_center_xy[1],
                UNDERFRAME_HEIGHT + TOP_THICKNESS / 2.0,
            )
        ),
        material=desktop_oak,
        name="long_top",
    )
    desktop.visual(
        Box((RETURN_TOP_SIZE[0], RETURN_TOP_SIZE[1], TOP_THICKNESS)),
        origin=Origin(
            xyz=(
                return_center_xy[0],
                return_center_xy[1],
                UNDERFRAME_HEIGHT + TOP_THICKNESS / 2.0,
            )
        ),
        material=desktop_oak,
        name="return_top",
    )

    controller_center = (-0.56, -0.362, 0.046)
    desktop.visual(
        Box((0.172, 0.056, 0.028)),
        origin=Origin(xyz=controller_center),
        material=controller_black,
        name="controller_body",
    )
    desktop.visual(
        Box((0.110, 0.028, 0.012)),
        origin=Origin(xyz=(controller_center[0], controller_center[1] + 0.010, 0.054)),
        material=controller_black,
        name="controller_mount",
    )
    desktop.visual(
        Box((0.008, 0.026, 0.012)),
        origin=Origin(xyz=(controller_center[0], controller_center[1] - 0.006, 0.036)),
        material=satin_black,
        name="controller_divider",
    )

    up_paddle = model.part("up_paddle")
    _add_paddle(up_paddle, plastic=satin_black)

    down_paddle = model.part("down_paddle")
    _add_paddle(down_paddle, plastic=satin_black)

    model.articulation(
        "corner_to_front_beam",
        ArticulationType.FIXED,
        parent=corner_column,
        child=front_beam,
        origin=Origin(xyz=(0.28, 0.0, 0.0)),
    )
    model.articulation(
        "corner_to_return_beam",
        ArticulationType.FIXED,
        parent=corner_column,
        child=return_beam,
        origin=Origin(xyz=(0.0, 0.28, 0.0)),
    )
    model.articulation(
        "front_beam_to_front_column",
        ArticulationType.FIXED,
        parent=front_beam,
        child=front_column,
        origin=Origin(xyz=(0.82, -0.04, 0.0)),
    )
    model.articulation(
        "return_beam_to_return_column",
        ArticulationType.FIXED,
        parent=return_beam,
        child=return_column,
        origin=Origin(xyz=(-0.04, 0.54, 0.0)),
    )

    lift_limits = MotionLimits(
        effort=800.0,
        velocity=0.060,
        lower=0.0,
        upper=LIFT_TRAVEL,
    )
    model.articulation(
        "front_lift",
        ArticulationType.PRISMATIC,
        parent=front_column,
        child=front_stage,
        origin=Origin(xyz=(0.0, 0.0, OUTER_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=lift_limits,
    )
    model.articulation(
        "corner_lift",
        ArticulationType.PRISMATIC,
        parent=corner_column,
        child=corner_stage,
        origin=Origin(xyz=(0.0, 0.0, OUTER_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=800.0,
            velocity=0.060,
            lower=0.0,
            upper=LIFT_TRAVEL,
        ),
        mimic=Mimic(joint="front_lift"),
    )
    model.articulation(
        "return_lift",
        ArticulationType.PRISMATIC,
        parent=return_column,
        child=return_stage,
        origin=Origin(xyz=(0.0, 0.0, OUTER_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=800.0,
            velocity=0.060,
            lower=0.0,
            upper=LIFT_TRAVEL,
        ),
        mimic=Mimic(joint="front_lift"),
    )

    model.articulation(
        "front_stage_to_desktop",
        ArticulationType.FIXED,
        parent=front_stage,
        child=desktop,
        origin=Origin(xyz=(0.0, 0.0, STAGE_CAP_THICKNESS)),
    )

    model.articulation(
        "desktop_to_up_paddle",
        ArticulationType.REVOLUTE,
        parent=desktop,
        child=up_paddle,
        origin=Origin(xyz=(controller_center[0] - 0.024, controller_center[1] - 0.028, 0.036)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=-0.18,
            upper=0.35,
        ),
    )
    model.articulation(
        "desktop_to_down_paddle",
        ArticulationType.REVOLUTE,
        parent=desktop,
        child=down_paddle,
        origin=Origin(xyz=(controller_center[0] + 0.024, controller_center[1] - 0.028, 0.036)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=-0.35,
            upper=0.18,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front_column = object_model.get_part("front_column")
    corner_column = object_model.get_part("corner_column")
    return_column = object_model.get_part("return_column")
    front_stage = object_model.get_part("front_stage")
    corner_stage = object_model.get_part("corner_stage")
    return_stage = object_model.get_part("return_stage")
    desktop = object_model.get_part("desktop")
    up_paddle = object_model.get_part("up_paddle")
    down_paddle = object_model.get_part("down_paddle")

    front_lift = object_model.get_articulation("front_lift")
    up_joint = object_model.get_articulation("desktop_to_up_paddle")
    down_joint = object_model.get_articulation("desktop_to_down_paddle")

    ctx.allow_overlap(
        desktop,
        up_paddle,
        elem_a="controller_body",
        elem_b="pivot",
        reason="The up paddle rotates on a short pivot barrel recessed into the controller body.",
    )
    ctx.allow_overlap(
        desktop,
        down_paddle,
        elem_a="controller_body",
        elem_b="pivot",
        reason="The down paddle rotates on a short pivot barrel recessed into the controller body.",
    )

    for stage, column, stage_name in (
        (front_stage, front_column, "front"),
        (corner_stage, corner_column, "corner"),
        (return_stage, return_column, "return"),
    ):
        ctx.expect_origin_distance(
            stage,
            column,
            axes="xy",
            max_dist=0.001,
            name=f"{stage_name} stage stays centered over its column",
        )

    ctx.expect_gap(
        down_paddle,
        up_paddle,
        axis="x",
        min_gap=0.010,
        name="controller paddles remain visibly split",
    )

    desktop_rest = ctx.part_world_position(desktop)
    with ctx.pose({front_lift: LIFT_TRAVEL}):
        ctx.expect_overlap(
            front_stage,
            front_column,
            axes="z",
            min_overlap=0.13,
            name="front stage retains insertion at full lift",
        )
        ctx.expect_overlap(
            corner_stage,
            corner_column,
            axes="z",
            min_overlap=0.13,
            name="corner stage retains insertion at full lift",
        )
        ctx.expect_overlap(
            return_stage,
            return_column,
            axes="z",
            min_overlap=0.13,
            name="return stage retains insertion at full lift",
        )

        front_pos = ctx.part_world_position(front_stage)
        corner_pos = ctx.part_world_position(corner_stage)
        return_pos = ctx.part_world_position(return_stage)
        desktop_high = ctx.part_world_position(desktop)

        ctx.check(
            "three lift stages stay synchronized",
            front_pos is not None
            and corner_pos is not None
            and return_pos is not None
            and abs(front_pos[2] - corner_pos[2]) <= 1e-6
            and abs(front_pos[2] - return_pos[2]) <= 1e-6,
            details=f"front={front_pos}, corner={corner_pos}, return={return_pos}",
        )
        ctx.check(
            "desktop rises with the lift stages",
            desktop_rest is not None
            and desktop_high is not None
            and desktop_high[2] > desktop_rest[2] + 0.40,
            details=f"rest={desktop_rest}, high={desktop_high}",
        )

    up_rest = ctx.part_element_world_aabb(up_paddle, elem="blade")
    down_rest = ctx.part_element_world_aabb(down_paddle, elem="blade")
    with ctx.pose({up_joint: 0.30, down_joint: -0.30}):
        up_pressed = ctx.part_element_world_aabb(up_paddle, elem="blade")
        down_pressed = ctx.part_element_world_aabb(down_paddle, elem="blade")

    ctx.check(
        "up paddle tips upward on its local pivot",
        up_rest is not None
        and up_pressed is not None
        and up_pressed[1][2] > up_rest[1][2] + 0.006,
        details=f"rest={up_rest}, pressed={up_pressed}",
    )
    ctx.check(
        "down paddle tips downward on its local pivot",
        down_rest is not None
        and down_pressed is not None
        and down_pressed[0][2] < down_rest[0][2] - 0.006,
        details=f"rest={down_rest}, pressed={down_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
