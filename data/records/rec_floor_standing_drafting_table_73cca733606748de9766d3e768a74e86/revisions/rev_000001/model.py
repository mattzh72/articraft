from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


POST_CENTER_X = 0.31
OUTER_POST_TOP_Z = 0.66
LIFT_TRAVEL = 0.22


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    yaw = math.atan2(dy, dx)
    length_xy = math.hypot(dx, dy)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, *, radius: float, material, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _add_outer_post(base, *, x_center: float, prefix: str, frame, foot) -> None:
    base.visual(
        Box((0.10, 0.72, 0.04)),
        origin=Origin(xyz=(x_center, 0.0, 0.02)),
        material=frame,
        name=f"{prefix}_runner",
    )

    for index, y in enumerate((-0.30, 0.30)):
        base.visual(
            Cylinder(radius=0.016, length=0.014),
            origin=Origin(xyz=(x_center, y, -0.007)),
            material=foot,
            name=f"{prefix}_foot_{index}",
        )

    sleeve_height = 0.62
    sleeve_z = 0.04 + sleeve_height / 2.0
    outer_x = 0.086
    outer_y = 0.060
    wall = 0.006

    base.visual(
        Box((outer_x, wall, sleeve_height)),
        origin=Origin(xyz=(x_center, outer_y / 2.0 - wall / 2.0, sleeve_z)),
        material=frame,
        name=f"{prefix}_outer_front",
    )
    base.visual(
        Box((outer_x, wall, sleeve_height)),
        origin=Origin(xyz=(x_center, -outer_y / 2.0 + wall / 2.0, sleeve_z)),
        material=frame,
        name=f"{prefix}_outer_rear",
    )
    base.visual(
        Box((wall, outer_y - 2.0 * wall, sleeve_height)),
        origin=Origin(xyz=(x_center - outer_x / 2.0 + wall / 2.0, 0.0, sleeve_z)),
        material=frame,
        name=f"{prefix}_outer_inner_side",
    )
    base.visual(
        Box((wall, outer_y - 2.0 * wall, sleeve_height)),
        origin=Origin(xyz=(x_center + outer_x / 2.0 - wall / 2.0, 0.0, sleeve_z)),
        material=frame,
        name=f"{prefix}_outer_outer_side",
    )

    _add_member(
        base,
        (x_center, -0.27, 0.04),
        (x_center, -0.09, 0.28),
        radius=0.010,
        material=frame,
        name=f"{prefix}_rear_brace",
    )
    _add_member(
        base,
        (x_center, 0.27, 0.04),
        (x_center, 0.09, 0.28),
        radius=0.010,
        material=frame,
        name=f"{prefix}_front_brace",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drafting_table")

    frame = model.material("frame", rgba=(0.22, 0.23, 0.25, 1.0))
    steel = model.material("steel", rgba=(0.63, 0.66, 0.70, 1.0))
    board_surface = model.material("board_surface", rgba=(0.88, 0.83, 0.70, 1.0))
    board_edge = model.material("board_edge", rgba=(0.42, 0.33, 0.24, 1.0))
    knob_finish = model.material("knob_finish", rgba=(0.10, 0.10, 0.11, 1.0))
    foot = model.material("foot", rgba=(0.08, 0.08, 0.09, 1.0))

    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.058,
            0.028,
            body_style="mushroom",
            grip=KnobGrip(style="fluted", count=8, depth=0.0024),
            center=False,
        ),
        "drafting_table_clamp_knob",
    )

    base = model.part("base")
    _add_outer_post(base, x_center=-POST_CENTER_X, prefix="left", frame=frame, foot=foot)
    _add_outer_post(base, x_center=POST_CENTER_X, prefix="right", frame=frame, foot=foot)
    base.visual(
        Box((0.56, 0.06, 0.035)),
        origin=Origin(xyz=(0.0, 0.23, 0.0575)),
        material=frame,
        name="front_stretcher",
    )
    base.visual(
        Box((0.56, 0.06, 0.035)),
        origin=Origin(xyz=(0.0, -0.23, 0.0575)),
        material=frame,
        name="rear_stretcher",
    )
    base.visual(
        Box((0.54, 0.040, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        material=frame,
        name="mid_tie",
    )

    left_stage = model.part("left_stage")
    left_stage.visual(
        Box((0.070, 0.048, 0.70)),
        origin=Origin(xyz=(0.0, 0.0, -0.08)),
        material=steel,
        name="inner_member",
    )
    left_stage.visual(
        Box((0.094, 0.068, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.283)),
        material=frame,
        name="top_cap",
    )

    right_stage = model.part("right_stage")
    right_stage.visual(
        Box((0.070, 0.048, 0.70)),
        origin=Origin(xyz=(0.0, 0.0, -0.08)),
        material=steel,
        name="inner_member",
    )
    right_stage.visual(
        Box((0.094, 0.068, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.283)),
        material=frame,
        name="top_cap",
    )

    model.articulation(
        "left_lift",
        ArticulationType.PRISMATIC,
        parent=base,
        child=left_stage,
        origin=Origin(xyz=(-POST_CENTER_X, 0.0, OUTER_POST_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.10,
            lower=0.0,
            upper=LIFT_TRAVEL,
        ),
    )
    model.articulation(
        "right_lift",
        ArticulationType.PRISMATIC,
        parent=base,
        child=right_stage,
        origin=Origin(xyz=(POST_CENTER_X, 0.0, OUTER_POST_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.10,
            lower=0.0,
            upper=LIFT_TRAVEL,
        ),
        mimic=Mimic("left_lift"),
    )

    head = model.part("head")
    head.visual(
        Box((0.11, 0.08, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=frame,
        name="left_saddle",
    )
    head.visual(
        Box((0.11, 0.08, 0.04)),
        origin=Origin(xyz=(0.62, 0.0, 0.02)),
        material=frame,
        name="right_saddle",
    )
    head.visual(
        Box((0.66, 0.09, 0.05)),
        origin=Origin(xyz=(0.31, 0.0, 0.025)),
        material=frame,
        name="crossbeam",
    )
    head.visual(
        Box((0.11, 0.07, 0.09)),
        origin=Origin(xyz=(0.31, 0.0, 0.065)),
        material=frame,
        name="mast",
    )
    head.visual(
        Box((0.18, 0.08, 0.064)),
        origin=Origin(xyz=(0.31, -0.02, 0.08)),
        material=frame,
        name="head_block",
    )
    head.visual(
        Box((0.036, 0.06, 0.064)),
        origin=Origin(xyz=(0.22, -0.02, 0.08)),
        material=frame,
        name="left_pivot_block",
    )
    head.visual(
        Box((0.036, 0.06, 0.064)),
        origin=Origin(xyz=(0.40, -0.055, 0.08)),
        material=frame,
        name="knob_mount",
    )
    model.articulation(
        "left_stage_to_head",
        ArticulationType.FIXED,
        parent=left_stage,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.296)),
    )

    board = model.part("board")
    board.visual(
        Cylinder(radius=0.018, length=0.18),
        origin=Origin(xyz=(0.0, -0.01, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hinge_barrel",
    )
    board.visual(
        Box((0.24, 0.040, 0.026)),
        origin=Origin(xyz=(0.0, 0.015, 0.002)),
        material=board_edge,
        name="hinge_plate",
    )
    board.visual(
        Box((1.08, 0.76, 0.018)),
        origin=Origin(xyz=(0.0, 0.39, 0.038)),
        material=board_surface,
        name="surface",
    )
    board.visual(
        Box((0.022, 0.76, 0.035)),
        origin=Origin(xyz=(-0.529, 0.39, 0.026)),
        material=board_edge,
        name="left_frame",
    )
    board.visual(
        Box((0.022, 0.76, 0.035)),
        origin=Origin(xyz=(0.529, 0.39, 0.026)),
        material=board_edge,
        name="right_frame",
    )
    board.visual(
        Box((1.08, 0.026, 0.032)),
        origin=Origin(xyz=(0.0, 0.018, 0.024)),
        material=board_edge,
        name="rear_rail",
    )
    board.visual(
        Box((0.94, 0.018, 0.030)),
        origin=Origin(xyz=(0.0, 0.752, 0.026)),
        material=board_edge,
        name="paper_lip",
    )
    board.visual(
        Box((0.20, 0.40, 0.040)),
        origin=Origin(xyz=(0.0, 0.34, 0.010)),
        material=board_edge,
        name="center_stiffener",
    )

    model.articulation(
        "board_tilt",
        ArticulationType.REVOLUTE,
        parent=head,
        child=board,
        origin=Origin(xyz=(0.31, -0.01, 0.13)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.8,
            lower=0.0,
            upper=1.1,
        ),
    )

    knob = model.part("knob")
    knob.visual(
        Cylinder(radius=0.005, length=0.024),
        origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="shaft",
    )
    knob.visual(
        Cylinder(radius=0.014, length=0.014),
        origin=Origin(xyz=(0.031, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hub",
    )
    knob.visual(
        knob_mesh,
        origin=Origin(xyz=(0.038, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_finish,
        name="grip",
    )
    model.articulation(
        "knob_spin",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=knob,
        origin=Origin(xyz=(0.418, -0.055, 0.09)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    left_stage = object_model.get_part("left_stage")
    right_stage = object_model.get_part("right_stage")
    head = object_model.get_part("head")
    board = object_model.get_part("board")
    knob = object_model.get_part("knob")

    left_lift = object_model.get_articulation("left_lift")
    right_lift = object_model.get_articulation("right_lift")
    board_tilt = object_model.get_articulation("board_tilt")
    knob_spin = object_model.get_articulation("knob_spin")

    lift_limits = left_lift.motion_limits
    tilt_limits = board_tilt.motion_limits

    ctx.check(
        "left lift is prismatic",
        left_lift.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={left_lift.articulation_type!r}",
    )
    ctx.check(
        "right lift is prismatic",
        right_lift.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={right_lift.articulation_type!r}",
    )
    ctx.check(
        "board tilt is revolute",
        board_tilt.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={board_tilt.articulation_type!r}",
    )
    ctx.check(
        "knob is continuous",
        knob_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={knob_spin.articulation_type!r}",
    )

    with ctx.pose({left_lift: 0.0, board_tilt: 0.0}):
        ctx.expect_contact(
            board,
            head,
            elem_a="hinge_barrel",
            elem_b="head_block",
            name="board hinge bears on head block at rest",
        )
        ctx.expect_contact(
            head,
            left_stage,
            elem_a="left_saddle",
            elem_b="top_cap",
            name="left stage supports head at rest",
        )
        ctx.expect_contact(
            head,
            right_stage,
            elem_a="right_saddle",
            elem_b="top_cap",
            name="right stage supports head at rest",
        )
        ctx.expect_gap(
            knob,
            head,
            axis="x",
            positive_elem="shaft",
            negative_elem="knob_mount",
            max_gap=0.001,
            max_penetration=1e-6,
            name="knob seats against head without overlap",
        )

    if lift_limits is not None and lift_limits.upper is not None:
        upper = lift_limits.upper

        rest_left = ctx.part_world_position(left_stage)
        rest_right = ctx.part_world_position(right_stage)
        with ctx.pose({left_lift: upper}):
            ctx.expect_contact(
                head,
                left_stage,
                elem_a="left_saddle",
                elem_b="top_cap",
                name="left stage supports head when raised",
            )
            ctx.expect_contact(
                head,
                right_stage,
                elem_a="right_saddle",
                elem_b="top_cap",
                name="right stage supports head when raised",
            )

            raised_left = ctx.part_world_position(left_stage)
            raised_right = ctx.part_world_position(right_stage)
            ctx.check(
                "twin stages rise together",
                rest_left is not None
                and rest_right is not None
                and raised_left is not None
                and raised_right is not None
                and raised_left[2] > rest_left[2] + 0.18
                and raised_right[2] > rest_right[2] + 0.18
                and abs(raised_left[2] - raised_right[2]) < 1e-6,
                details=(
                    f"rest_left={rest_left}, rest_right={rest_right}, "
                    f"raised_left={raised_left}, raised_right={raised_right}"
                ),
            )

            left_member = ctx.part_element_world_aabb(left_stage, elem="inner_member")
            right_member = ctx.part_element_world_aabb(right_stage, elem="inner_member")
            left_outer = ctx.part_element_world_aabb(base, elem="left_outer_front")
            right_outer = ctx.part_element_world_aabb(base, elem="right_outer_front")
            left_insertion = None
            right_insertion = None
            if left_member is not None and left_outer is not None:
                left_insertion = float(left_outer[1][2] - left_member[0][2])
            if right_member is not None and right_outer is not None:
                right_insertion = float(right_outer[1][2] - right_member[0][2])

            ctx.check(
                "left stage retains insertion at full lift",
                left_insertion is not None and left_insertion >= 0.18,
                details=f"insertion={left_insertion}",
            )
            ctx.check(
                "right stage retains insertion at full lift",
                right_insertion is not None and right_insertion >= 0.18,
                details=f"insertion={right_insertion}",
            )

    if tilt_limits is not None and tilt_limits.upper is not None:
        rest_lip = None
        raised_lip = None
        with ctx.pose({board_tilt: 0.0}):
            rest_lip = ctx.part_element_world_aabb(board, elem="paper_lip")
        with ctx.pose({board_tilt: tilt_limits.upper}):
            raised_lip = ctx.part_element_world_aabb(board, elem="paper_lip")

        ctx.check(
            "board front edge rises when tilted",
            rest_lip is not None
            and raised_lip is not None
            and raised_lip[1][2] > rest_lip[1][2] + 0.45,
            details=f"rest_lip={rest_lip}, raised_lip={raised_lip}",
        )

    return ctx.report()


object_model = build_object_model()
