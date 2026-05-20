from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


COLUMN_XS = (-0.85, 0.0, 0.85)
LIFT_TRAVEL = 0.38


WOOD = Material("warm_maple_laminate", color=(0.72, 0.55, 0.34, 1.0))
BLACK = Material("black_powder_coated_steel", color=(0.015, 0.017, 0.018, 1.0))
DARK = Material("matte_black_plastic", color=(0.025, 0.026, 0.028, 1.0))
RUBBER = Material("soft_dark_rubber", color=(0.04, 0.04, 0.042, 1.0))
GREY = Material("cool_grey_button", color=(0.32, 0.34, 0.35, 1.0))
SILVER = Material("brushed_inner_steel", color=(0.55, 0.57, 0.56, 1.0))


def _add_outer_square_sleeve(part, *, x: float, index: int) -> None:
    """Four wall visuals leave a real open core for the sliding inner stage."""

    od = 0.14
    wall = 0.015
    height = 0.57
    zc = 0.335
    part.visual(
        Box((od, wall, height)),
        origin=Origin(xyz=(x, -od / 2 + wall / 2, zc)),
        material=BLACK,
        name=f"outer_sleeve_{index}_front",
    )
    part.visual(
        Box((od, wall, height)),
        origin=Origin(xyz=(x, od / 2 - wall / 2, zc)),
        material=BLACK,
        name=f"outer_sleeve_{index}_rear",
    )
    part.visual(
        Box((wall, od - 2 * wall, height)),
        origin=Origin(xyz=(x - od / 2 + wall / 2, 0.0, zc)),
        material=BLACK,
        name=f"outer_sleeve_{index}_side_0",
    )
    part.visual(
        Box((wall, od - 2 * wall, height)),
        origin=Origin(xyz=(x + od / 2 - wall / 2, 0.0, zc)),
        material=BLACK,
        name=f"outer_sleeve_{index}_side_1",
    )

    # A slightly thicker lip makes the top opening read as a telescoping sleeve.
    lip_z = 0.608
    lip_h = 0.024
    lip_od = 0.16
    part.visual(
        Box((lip_od, wall, lip_h)),
        origin=Origin(xyz=(x, -lip_od / 2 + wall / 2, lip_z)),
        material=BLACK,
        name=f"sleeve_lip_{index}_front",
    )
    part.visual(
        Box((lip_od, wall, lip_h)),
        origin=Origin(xyz=(x, lip_od / 2 - wall / 2, lip_z)),
        material=BLACK,
        name=f"sleeve_lip_{index}_rear",
    )
    part.visual(
        Box((wall, lip_od - 2 * wall, lip_h)),
        origin=Origin(xyz=(x - lip_od / 2 + wall / 2, 0.0, lip_z)),
        material=BLACK,
        name=f"sleeve_lip_{index}_side_0",
    )
    part.visual(
        Box((wall, lip_od - 2 * wall, lip_h)),
        origin=Origin(xyz=(x + lip_od / 2 - wall / 2, 0.0, lip_z)),
        material=BLACK,
        name=f"sleeve_lip_{index}_side_1",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_style_three_column_standing_desk")

    base = model.part("base_frame")
    # Long shared base rail and individual bench feet tie the three columns into
    # one stationary welded frame.
    base.visual(
        Box((2.05, 0.07, 0.075)),
        origin=Origin(xyz=(0.0, -0.105, 0.0875)),
        material=BLACK,
        name="front_floor_rail",
    )
    base.visual(
        Box((2.05, 0.07, 0.075)),
        origin=Origin(xyz=(0.0, 0.105, 0.0875)),
        material=BLACK,
        name="rear_floor_rail",
    )
    for i, x in enumerate(COLUMN_XS):
        base.visual(
            Box((0.28, 0.86, 0.055)),
            origin=Origin(xyz=(x, 0.0, 0.0275)),
            material=BLACK,
            name=f"foot_{i}",
        )
        base.visual(
            Box((0.21, 0.07, 0.07)),
            origin=Origin(xyz=(x, -0.105, 0.075)),
            material=BLACK,
            name=f"front_plinth_{i}",
        )
        base.visual(
            Box((0.21, 0.07, 0.07)),
            origin=Origin(xyz=(x, 0.105, 0.075)),
            material=BLACK,
            name=f"rear_plinth_{i}",
        )
        _add_outer_square_sleeve(base, x=x, index=i)

    lift_limits = MotionLimits(effort=900.0, velocity=0.06, lower=0.0, upper=LIFT_TRAVEL)
    stage_parts = []
    for i, x in enumerate(COLUMN_XS):
        stage = model.part(f"inner_stage_{i}")
        stage.visual(
            Box((0.084, 0.084, 0.58)),
            origin=Origin(xyz=(0.0, 0.0, -0.22)),
            material=SILVER,
            name="inner_tube",
        )
        stage.visual(
            Box((0.118, 0.118, 0.04)),
            origin=Origin(xyz=(0.0, 0.0, 0.05)),
            material=BLACK,
            name="upper_cap",
        )
        stage_parts.append(stage)

        mimic = None if i == 1 else Mimic("base_to_inner_stage_1")
        model.articulation(
            f"base_to_inner_stage_{i}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=stage,
            origin=Origin(xyz=(x, 0.0, 0.59)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=lift_limits,
            mimic=mimic,
        )

    desk = model.part("desktop_frame")
    # Rigid long desktop: shared workstation scale, with visible steel subframe
    # spanning all three lifting columns.
    desk.visual(
        Box((2.40, 0.90, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0975)),
        material=WOOD,
        name="long_desktop",
    )
    desk.visual(
        Box((2.18, 0.070, 0.060)),
        origin=Origin(xyz=(0.0, -0.32, 0.045)),
        material=BLACK,
        name="front_frame_rail",
    )
    desk.visual(
        Box((2.18, 0.070, 0.060)),
        origin=Origin(xyz=(0.0, 0.32, 0.045)),
        material=BLACK,
        name="rear_frame_rail",
    )
    desk.visual(
        Box((2.18, 0.075, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=BLACK,
        name="center_spine",
    )
    for i, x in enumerate(COLUMN_XS):
        desk.visual(
            Box((0.11, 0.72, 0.060)),
            origin=Origin(xyz=(x, 0.0, 0.045)),
            material=BLACK,
            name=f"cross_bearer_{i}",
        )
        desk.visual(
            Box((0.24, 0.20, 0.025)),
            origin=Origin(xyz=(x, 0.0, 0.0125)),
            material=BLACK,
            name=f"support_plate_{i}",
        )
        desk.visual(
            Box((0.15, 0.15, 0.045)),
            origin=Origin(xyz=(x, 0.0, 0.030)),
            material=BLACK,
            name=f"column_socket_{i}",
        )

    model.articulation(
        "inner_stage_1_to_desktop_frame",
        ArticulationType.FIXED,
        parent=stage_parts[1],
        child=desk,
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
    )

    paddle_housing = model.part("paddle_housing")
    paddle_housing.visual(
        Box((0.18, 0.07, 0.04)),
        origin=Origin(),
        material=DARK,
        name="housing",
    )
    paddle_housing.visual(
        Box((0.018, 0.025, 0.030)),
        origin=Origin(xyz=(-0.072, -0.035, -0.027)),
        material=DARK,
        name="hinge_tab_0",
    )
    paddle_housing.visual(
        Box((0.018, 0.025, 0.030)),
        origin=Origin(xyz=(0.072, -0.035, -0.027)),
        material=DARK,
        name="hinge_tab_1",
    )
    model.articulation(
        "desktop_frame_to_paddle_housing",
        ArticulationType.FIXED,
        parent=desk,
        child=paddle_housing,
        origin=Origin(xyz=(0.0, -0.472, 0.055)),
    )

    paddle = model.part("paddle")
    paddle.visual(
        Cylinder(radius=0.006, length=0.125),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=BLACK,
        name="hinge_barrel",
    )
    paddle.visual(
        Box((0.14, 0.060, 0.012)),
        origin=Origin(xyz=(0.0, -0.030, -0.012)),
        material=RUBBER,
        name="paddle_face",
    )
    model.articulation(
        "paddle_housing_to_paddle",
        ArticulationType.REVOLUTE,
        parent=paddle_housing,
        child=paddle,
        origin=Origin(xyz=(0.0, -0.040, -0.035)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=2.0, lower=-0.30, upper=0.30),
    )

    handset = model.part("handset")
    handset.visual(
        Box((0.24, 0.12, 0.035)),
        origin=Origin(),
        material=DARK,
        name="body",
    )
    handset.visual(
        Box((0.20, 0.085, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.0195)),
        material=BLACK,
        name="button_bezel",
    )
    model.articulation(
        "desktop_frame_to_handset",
        ArticulationType.FIXED,
        parent=desk,
        child=handset,
        origin=Origin(xyz=(0.48, -0.472, 0.0575)),
    )

    button_positions = (
        (-0.045, -0.024),
        (0.045, -0.024),
        (-0.045, 0.024),
        (0.045, 0.024),
    )
    for i, (x, y) in enumerate(button_positions):
        button = model.part(f"button_{i}")
        button.visual(
            Cylinder(radius=0.018, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, -0.005)),
            material=GREY,
            name="cap",
        )
        button.visual(
            Cylinder(radius=0.012, length=0.008),
            origin=Origin(xyz=(0.0, 0.0, -0.013)),
            material=RUBBER,
            name="rubber_skirt",
        )
        model.articulation(
            f"handset_to_button_{i}",
            ArticulationType.PRISMATIC,
            parent=handset,
            child=button,
            origin=Origin(xyz=(x, y, -0.0175)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.04, lower=0.0, upper=0.006),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    desk = object_model.get_part("desktop_frame")
    center_lift = object_model.get_articulation("base_to_inner_stage_1")

    for i in range(3):
        stage = object_model.get_part(f"inner_stage_{i}")
        ctx.expect_contact(
            stage,
            desk,
            elem_a="upper_cap",
            elem_b=f"support_plate_{i}",
            contact_tol=0.0005,
            name=f"inner stage {i} bears against shared desktop frame",
        )

    rest_desk_pos = ctx.part_world_position(desk)
    with ctx.pose({center_lift: LIFT_TRAVEL}):
        raised_desk_pos = ctx.part_world_position(desk)
        for i in range(3):
            stage = object_model.get_part(f"inner_stage_{i}")
            ctx.expect_contact(
                stage,
                desk,
                elem_a="upper_cap",
                elem_b=f"support_plate_{i}",
                contact_tol=0.0005,
                name=f"raised inner stage {i} still supports the desktop",
            )

    ctx.check(
        "synchronized lift raises the rigid desktop",
        rest_desk_pos is not None
        and raised_desk_pos is not None
        and raised_desk_pos[2] > rest_desk_pos[2] + LIFT_TRAVEL - 0.005,
        details=f"rest={rest_desk_pos}, raised={raised_desk_pos}",
    )

    handset = object_model.get_part("handset")
    for i in range(4):
        button = object_model.get_part(f"button_{i}")
        ctx.expect_gap(
            handset,
            button,
            axis="z",
            max_gap=0.0005,
            max_penetration=0.0005,
            positive_elem="body",
            negative_elem="cap",
            name=f"button {i} sits proud in handset face",
        )

    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")
    rest_b0 = ctx.part_world_position(button_0)
    rest_b1 = ctx.part_world_position(button_1)
    with ctx.pose({"handset_to_button_0": 0.006}):
        pressed_b0 = ctx.part_world_position(button_0)
        pressed_b1 = ctx.part_world_position(button_1)

    ctx.check(
        "keypad buttons actuate independently",
        rest_b0 is not None
        and rest_b1 is not None
        and pressed_b0 is not None
        and pressed_b1 is not None
        and pressed_b0[2] > rest_b0[2] + 0.005
        and abs(pressed_b1[2] - rest_b1[2]) < 0.0005,
        details=f"b0 rest={rest_b0}, b0 pressed={pressed_b0}, b1 rest={rest_b1}, b1 pressed={pressed_b1}",
    )

    paddle = object_model.get_part("paddle")
    paddle_joint = object_model.get_articulation("paddle_housing_to_paddle")
    rest_aabb = ctx.part_world_aabb(paddle)
    with ctx.pose({paddle_joint: 0.30}):
        tilted_aabb = ctx.part_world_aabb(paddle)
    ctx.check(
        "front paddle tilts under the center edge",
        rest_aabb is not None
        and tilted_aabb is not None
        and tilted_aabb[0][2] < rest_aabb[0][2] - 0.004,
        details=f"rest_aabb={rest_aabb}, tilted_aabb={tilted_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
