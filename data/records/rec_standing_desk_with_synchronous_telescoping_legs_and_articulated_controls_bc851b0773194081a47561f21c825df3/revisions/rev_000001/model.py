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


DESK_TRAVEL = 0.38
BUTTON_TRAVEL = 0.003


def _add_outer_column(
    model: ArticulatedObject,
    *,
    name: str,
    steel: str,
    foot_length: float,
    foot_yaw: float,
) -> None:
    column = model.part(name)

    column.visual(
        Box((0.110, 0.012, 0.500)),
        origin=Origin(xyz=(0.0, 0.035, -0.250)),
        material=steel,
        name="front_wall",
    )
    column.visual(
        Box((0.110, 0.012, 0.500)),
        origin=Origin(xyz=(0.0, -0.035, -0.250)),
        material=steel,
        name="rear_wall",
    )
    column.visual(
        Box((0.012, 0.058, 0.500)),
        origin=Origin(xyz=(0.049, 0.0, -0.250)),
        material=steel,
        name="side_wall_0",
    )
    column.visual(
        Box((0.012, 0.058, 0.500)),
        origin=Origin(xyz=(-0.049, 0.0, -0.250)),
        material=steel,
        name="side_wall_1",
    )
    column.visual(
        Box((0.110, 0.018, 0.120)),
        origin=Origin(xyz=(0.0, 0.032, -0.560)),
        material=steel,
        name="brace_front",
    )
    column.visual(
        Box((0.110, 0.018, 0.120)),
        origin=Origin(xyz=(0.0, -0.032, -0.560)),
        material=steel,
        name="brace_rear",
    )
    column.visual(
        Box((foot_length, 0.095, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, -0.650), rpy=(0.0, 0.0, foot_yaw)),
        material=steel,
        name="foot",
    )


def _add_button(
    model: ArticulatedObject,
    handset,
    *,
    index: int,
    x: float,
    z: float,
    material: str,
) -> None:
    button = model.part(f"button_{index}")
    button.visual(
        Box((0.026, 0.008, 0.016)),
        origin=Origin(xyz=(0.0, -0.004, 0.0)),
        material=material,
        name="cap",
    )

    model.articulation(
        f"handset_to_button_{index}",
        ArticulationType.PRISMATIC,
        parent=handset,
        child=button,
        origin=Origin(xyz=(x, -0.016, z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.05,
            lower=0.0,
            upper=BUTTON_TRAVEL,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="standing_corner_desk")

    laminate = model.material("laminate", rgba=(0.69, 0.62, 0.52, 1.0))
    frame = model.material("frame", rgba=(0.18, 0.19, 0.21, 1.0))
    stage = model.material("stage", rgba=(0.72, 0.74, 0.77, 1.0))
    handset_shell = model.material("handset_shell", rgba=(0.10, 0.11, 0.12, 1.0))
    button_face = model.material("button_face", rgba=(0.20, 0.22, 0.24, 1.0))

    desk = model.part("desk")
    desk.visual(
        Box((1.10, 0.80, 0.030)),
        origin=Origin(xyz=(0.87, -0.10, 0.075)),
        material=laminate,
        name="long_top",
    )
    desk.visual(
        Box((0.50, 0.49, 0.030)),
        origin=Origin(xyz=(0.07, 0.055, 0.075)),
        material=laminate,
        name="corner_top",
    )
    desk.visual(
        Box((0.80, 0.85, 0.030)),
        origin=Origin(xyz=(-0.08, 0.725, 0.075)),
        material=laminate,
        name="return_top",
    )

    desk.visual(
        Box((0.22, 0.18, 0.060)),
        origin=Origin(xyz=(1.12, -0.08, 0.030)),
        material=frame,
        name="long_mount",
    )
    desk.visual(
        Box((0.22, 0.22, 0.060)),
        origin=Origin(xyz=(0.15, 0.08, 0.030)),
        material=frame,
        name="corner_mount",
    )
    desk.visual(
        Box((0.18, 0.22, 0.060)),
        origin=Origin(xyz=(-0.08, 0.88, 0.030)),
        material=frame,
        name="return_mount",
    )
    desk.visual(
        Box((1.18, 0.10, 0.060)),
        origin=Origin(xyz=(0.63, 0.00, 0.030)),
        material=frame,
        name="long_beam",
    )
    desk.visual(
        Box((0.10, 0.86, 0.060)),
        origin=Origin(xyz=(0.04, 0.48, 0.030)),
        material=frame,
        name="return_beam",
    )
    desk.visual(
        Box((0.88, 0.060, 0.060)),
        origin=Origin(xyz=(0.84, -0.34, 0.030)),
        material=frame,
        name="front_rail",
    )
    desk.visual(
        Box((0.080, 0.340, 0.060)),
        origin=Origin(xyz=(0.84, -0.17, 0.030)),
        material=frame,
        name="front_brace",
    )
    desk.visual(
        Box((0.060, 0.560, 0.060)),
        origin=Origin(xyz=(-0.31, 0.67, 0.030)),
        material=frame,
        name="return_rail",
    )
    desk.visual(
        Box((0.28, 0.080, 0.060)),
        origin=Origin(xyz=(-0.19, 0.88, 0.030)),
        material=frame,
        name="return_brace",
    )

    desk.visual(
        Box((0.078, 0.050, 0.640)),
        origin=Origin(xyz=(1.12, -0.08, -0.270)),
        material=stage,
        name="long_stage",
    )
    desk.visual(
        Box((0.078, 0.050, 0.640)),
        origin=Origin(xyz=(0.15, 0.08, -0.270)),
        material=stage,
        name="corner_stage",
    )
    desk.visual(
        Box((0.078, 0.050, 0.640)),
        origin=Origin(xyz=(-0.08, 0.88, -0.270)),
        material=stage,
        name="return_stage",
    )

    _add_outer_column(
        model,
        name="long_column",
        steel="frame",
        foot_length=0.74,
        foot_yaw=math.pi / 2.0,
    )
    _add_outer_column(
        model,
        name="corner_column",
        steel="frame",
        foot_length=0.64,
        foot_yaw=math.pi / 4.0,
    )
    _add_outer_column(
        model,
        name="return_column",
        steel="frame",
        foot_length=0.74,
        foot_yaw=0.0,
    )

    lead_lift = model.articulation(
        "desk_to_long_column",
        ArticulationType.PRISMATIC,
        parent=desk,
        child="long_column",
        origin=Origin(xyz=(1.12, -0.08, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=400.0,
            velocity=0.04,
            lower=0.0,
            upper=DESK_TRAVEL,
        ),
    )
    model.articulation(
        "desk_to_corner_column",
        ArticulationType.PRISMATIC,
        parent=desk,
        child="corner_column",
        origin=Origin(xyz=(0.15, 0.08, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=400.0,
            velocity=0.04,
            lower=0.0,
            upper=DESK_TRAVEL,
        ),
        mimic=Mimic(joint=lead_lift.name),
    )
    model.articulation(
        "desk_to_return_column",
        ArticulationType.PRISMATIC,
        parent=desk,
        child="return_column",
        origin=Origin(xyz=(-0.08, 0.88, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=400.0,
            velocity=0.04,
            lower=0.0,
            upper=DESK_TRAVEL,
        ),
        mimic=Mimic(joint=lead_lift.name),
    )

    handset = model.part("handset")
    handset.visual(
        Box((0.180, 0.034, 0.006)),
        origin=Origin(xyz=(0.0, -0.003, -0.003)),
        material=handset_shell,
        name="top_wall",
    )
    handset.visual(
        Box((0.180, 0.034, 0.006)),
        origin=Origin(xyz=(0.0, -0.003, -0.053)),
        material=handset_shell,
        name="bottom_wall",
    )
    handset.visual(
        Box((0.180, 0.006, 0.056)),
        origin=Origin(xyz=(0.0, 0.014, -0.028)),
        material=handset_shell,
        name="back_wall",
    )
    handset.visual(
        Box((0.006, 0.034, 0.050)),
        origin=Origin(xyz=(-0.087, -0.003, -0.028)),
        material=handset_shell,
        name="side_wall_0",
    )
    handset.visual(
        Box((0.006, 0.034, 0.050)),
        origin=Origin(xyz=(0.087, -0.003, -0.028)),
        material=handset_shell,
        name="side_wall_1",
    )
    handset.visual(
        Box((0.180, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, -0.018, -0.006)),
        material=handset_shell,
        name="face_top",
    )
    handset.visual(
        Box((0.180, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, -0.018, -0.050)),
        material=handset_shell,
        name="face_bottom",
    )
    handset.visual(
        Box((0.010, 0.004, 0.034)),
        origin=Origin(xyz=(-0.085, -0.018, -0.028)),
        material=handset_shell,
        name="face_side_0",
    )
    handset.visual(
        Box((0.010, 0.004, 0.034)),
        origin=Origin(xyz=(0.085, -0.018, -0.028)),
        material=handset_shell,
        name="face_side_1",
    )

    model.articulation(
        "desk_to_handset",
        ArticulationType.FIXED,
        parent=desk,
        child=handset,
        origin=Origin(xyz=(0.68, -0.448, 0.060)),
    )

    for index, x in enumerate((-0.056, -0.018, 0.018, 0.056)):
        handset.visual(
            Box((0.003, 0.004, 0.048)),
            origin=Origin(xyz=(x - 0.0145, -0.018, -0.028)),
            material=handset_shell,
            name=f"guide_left_{index}",
        )
        handset.visual(
            Box((0.003, 0.004, 0.048)),
            origin=Origin(xyz=(x + 0.0145, -0.018, -0.028)),
            material=handset_shell,
            name=f"guide_right_{index}",
        )
        _add_button(
            model,
            handset,
            index=index,
            x=x,
            z=-0.028,
            material="button_face",
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    desk = object_model.get_part("desk")
    handset = object_model.get_part("handset")
    long_column = object_model.get_part("long_column")
    corner_column = object_model.get_part("corner_column")
    return_column = object_model.get_part("return_column")
    lead_lift = object_model.get_articulation("desk_to_long_column")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")
    button_0_joint = object_model.get_articulation("handset_to_button_0")

    for column_name, stage_elem in (
        ("long_column", "long_stage"),
        ("corner_column", "corner_stage"),
        ("return_column", "return_stage"),
    ):
        column = object_model.get_part(column_name)
        ctx.expect_within(
            desk,
            column,
            axes="xy",
            inner_elem=stage_elem,
            margin=0.0,
            name=f"{stage_elem} stays centered in {column_name}",
        )
        ctx.expect_overlap(
            desk,
            column,
            axes="z",
            elem_a=stage_elem,
            min_overlap=0.49,
            name=f"{stage_elem} remains inserted at seated height",
        )

    with ctx.pose({lead_lift: DESK_TRAVEL}):
        for column_name, stage_elem in (
            ("long_column", "long_stage"),
            ("corner_column", "corner_stage"),
            ("return_column", "return_stage"),
        ):
            column = object_model.get_part(column_name)
            ctx.expect_within(
                desk,
                column,
                axes="xy",
                inner_elem=stage_elem,
                margin=0.0,
                name=f"{stage_elem} stays centered in {column_name} at standing height",
            )
            ctx.expect_overlap(
                desk,
                column,
                axes="z",
                elem_a=stage_elem,
                min_overlap=0.20,
                name=f"{stage_elem} retains insertion at standing height",
            )

    rest_long = ctx.part_world_position(long_column)
    rest_corner = ctx.part_world_position(corner_column)
    rest_return = ctx.part_world_position(return_column)
    with ctx.pose({lead_lift: DESK_TRAVEL}):
        raised_long = ctx.part_world_position(long_column)
        raised_corner = ctx.part_world_position(corner_column)
        raised_return = ctx.part_world_position(return_column)

    long_drop = None if rest_long is None or raised_long is None else rest_long[2] - raised_long[2]
    corner_drop = None if rest_corner is None or raised_corner is None else rest_corner[2] - raised_corner[2]
    return_drop = None if rest_return is None or raised_return is None else rest_return[2] - raised_return[2]

    ctx.check(
        "columns travel downward together",
        long_drop is not None
        and corner_drop is not None
        and return_drop is not None
        and long_drop > 0.35
        and abs(long_drop - corner_drop) < 1e-6
        and abs(long_drop - return_drop) < 1e-6,
        details=(
            f"long_drop={long_drop}, corner_drop={corner_drop}, "
            f"return_drop={return_drop}"
        ),
    )

    ctx.expect_overlap(
        handset,
        desk,
        axes="x",
        min_overlap=0.16,
        name="handset spans the long run front zone",
    )
    ctx.expect_gap(
        desk,
        handset,
        axis="z",
        positive_elem="long_top",
        negative_elem="top_wall",
        max_penetration=0.0,
        max_gap=0.001,
        name="handset top seats against the long run underside",
    )

    handset_pos = ctx.part_world_position(handset)
    ctx.check(
        "handset sits at the front edge of the long run",
        handset_pos is not None and handset_pos[1] < -0.43,
        details=f"handset_pos={handset_pos}",
    )

    rest_button_0 = ctx.part_world_position(button_0)
    rest_button_1 = ctx.part_world_position(button_1)
    with ctx.pose({button_0_joint: BUTTON_TRAVEL}):
        pressed_button_0 = ctx.part_world_position(button_0)
        pressed_button_1 = ctx.part_world_position(button_1)

    ctx.check(
        "button_0 depresses inward without moving button_1",
        rest_button_0 is not None
        and pressed_button_0 is not None
        and rest_button_1 is not None
        and pressed_button_1 is not None
        and pressed_button_0[1] > rest_button_0[1] + 0.0025
        and abs(pressed_button_1[1] - rest_button_1[1]) < 1e-6,
        details=(
            f"button_0_rest={rest_button_0}, button_0_pressed={pressed_button_0}, "
            f"button_1_rest={rest_button_1}, button_1_pressed={pressed_button_1}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
