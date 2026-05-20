from __future__ import annotations

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


DESK_WIDTH = 1.45
DESK_DEPTH = 0.72
DESK_THICKNESS = 0.04

LEG_X = 0.50
OUTER_TOP_Z = 0.625
LIFT_TRAVEL = 0.46

DESKTOP_CENTER_Z_FROM_STAGE = 0.1375

PADDLE_PUSH_ANGLE = 0.35


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_leg_office_standing_desk")

    wood = Material("warm_oak_laminate", color=(0.72, 0.48, 0.26, 1.0))
    dark_edge = Material("dark_edge_band", color=(0.15, 0.11, 0.08, 1.0))
    black_metal = Material("black_powder_coated_steel", color=(0.02, 0.023, 0.025, 1.0))
    satin_metal = Material("sliding_satin_steel", color=(0.58, 0.60, 0.60, 1.0))
    plastic = Material("matte_black_plastic", color=(0.005, 0.005, 0.006, 1.0))
    rubber = Material("dark_rubber", color=(0.01, 0.01, 0.012, 1.0))
    label_white = Material("white_button_icons", color=(0.92, 0.92, 0.86, 1.0))

    base = model.part("base_frame")

    # Floor feet: long fore-aft foot bars with small leveling pads.
    for index, x in enumerate((-LEG_X, LEG_X)):
        suffix = f"_{index}"
        base.visual(
            Box((0.16, 0.62, 0.045)),
            origin=Origin(xyz=(x, 0.0, 0.0225)),
            material=black_metal,
            name=f"foot{suffix}",
        )
        for y in (-0.25, 0.25):
            base.visual(
                Cylinder(radius=0.027, length=0.012),
                origin=Origin(xyz=(x, y, -0.002)),
                material=rubber,
                name=f"leveler{suffix}_{'front' if y < 0.0 else 'rear'}",
            )

    # Two hollow fixed outer columns made from four walls each, so the inner
    # lifting stages visibly slide inside clear sleeves rather than through
    # solid blocks.
    outer_x = 0.10
    outer_y = 0.075
    wall = 0.010
    outer_h = OUTER_TOP_Z - 0.045
    outer_center_z = 0.045 + outer_h / 2.0
    for x, names in (
        (
            -LEG_X,
            (
                "column_0_outer_wall",
                "column_0_inner_wall",
                "column_0_front_wall",
                "column_0_rear_wall",
            ),
        ),
        (
            LEG_X,
            (
                "column_1_outer_wall",
                "column_1_inner_wall",
                "column_1_front_wall",
                "column_1_rear_wall",
            ),
        ),
    ):
        for wall_name, sx in ((names[0], -1.0), (names[1], 1.0)):
            base.visual(
                Box((wall, outer_y, outer_h)),
                origin=Origin(xyz=(x + sx * (outer_x / 2.0 - wall / 2.0), 0.0, outer_center_z)),
                material=black_metal,
                name=wall_name,
            )
        for wall_name, y in ((names[2], -1.0), (names[3], 1.0)):
            base.visual(
                Box((outer_x - 2.0 * wall, wall, outer_h)),
                origin=Origin(xyz=(x, y * (outer_y / 2.0 - wall / 2.0), outer_center_z)),
                material=black_metal,
                name=wall_name,
            )

    # A lower stretcher makes the two fixed leg bases one rigid frame and reads
    # as the stationary crossmember between the lifting columns.
    base.visual(
        Box((0.90, 0.045, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.54)),
        material=black_metal,
        name="lower_crossbeam",
    )

    def add_inner_stage(name: str) -> None:
        stage = model.part(name)
        stage.visual(
            Box((0.080, 0.055, 0.680)),
            origin=Origin(xyz=(0.0, 0.0, -0.230)),
            material=satin_metal,
            name="inner_tube",
        )
        stage.visual(
            Box((0.280, 0.120, 0.025)),
            origin=Origin(xyz=(0.0, 0.0, 0.105)),
            material=black_metal,
            name="top_plate",
        )
        stage.visual(
            Box((0.145, 0.032, 0.040)),
            origin=Origin(xyz=(0.0, 0.0, 0.075)),
            material=black_metal,
            name="plate_rib",
        )

    add_inner_stage("inner_stage_0")
    add_inner_stage("inner_stage_1")

    lift_limits = MotionLimits(effort=1200.0, velocity=0.08, lower=0.0, upper=LIFT_TRAVEL)
    model.articulation(
        "lift_stage_0",
        ArticulationType.PRISMATIC,
        parent=base,
        child="inner_stage_0",
        origin=Origin(xyz=(-LEG_X, 0.0, OUTER_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=lift_limits,
    )
    model.articulation(
        "lift_stage_1",
        ArticulationType.PRISMATIC,
        parent=base,
        child="inner_stage_1",
        origin=Origin(xyz=(LEG_X, 0.0, OUTER_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=lift_limits,
        mimic=Mimic(joint="lift_stage_0", multiplier=1.0, offset=0.0),
    )

    desktop = model.part("desktop")
    desktop.visual(
        Box((DESK_WIDTH, DESK_DEPTH, DESK_THICKNESS)),
        origin=Origin(),
        material=wood,
        name="desktop_top",
    )
    desktop.visual(
        Box((DESK_WIDTH, 0.026, 0.044)),
        origin=Origin(xyz=(0.0, -DESK_DEPTH / 2.0 + 0.013, 0.0)),
        material=dark_edge,
        name="front_edge_band",
    )
    desktop.visual(
        Box((DESK_WIDTH, 0.026, 0.044)),
        origin=Origin(xyz=(0.0, DESK_DEPTH / 2.0 - 0.013, 0.0)),
        material=dark_edge,
        name="rear_edge_band",
    )
    desktop.visual(
        Box((0.026, DESK_DEPTH, 0.044)),
        origin=Origin(xyz=(-DESK_WIDTH / 2.0 + 0.013, 0.0, 0.0)),
        material=dark_edge,
        name="side_edge_band_0",
    )
    desktop.visual(
        Box((0.026, DESK_DEPTH, 0.044)),
        origin=Origin(xyz=(DESK_WIDTH / 2.0 - 0.013, 0.0, 0.0)),
        material=dark_edge,
        name="side_edge_band_1",
    )
    # Moving under-desk steel frame, rear enough to stay clear of the handset.
    desktop.visual(
        Box((1.10, 0.055, 0.035)),
        origin=Origin(xyz=(0.0, 0.205, -0.0375)),
        material=black_metal,
        name="moving_crossbeam",
    )
    for x in (-LEG_X, LEG_X):
        desktop.visual(
            Box((0.075, 0.220, 0.030)),
            origin=Origin(xyz=(x, 0.230, -0.035)),
            material=black_metal,
            name=f"side_rail_{0 if x < 0.0 else 1}",
        )

    model.articulation(
        "stage_to_desktop",
        ArticulationType.FIXED,
        parent="inner_stage_0",
        child=desktop,
        origin=Origin(xyz=(LEG_X, 0.0, DESKTOP_CENTER_Z_FROM_STAGE)),
    )

    handset = model.part("handset")
    handset.visual(
        Box((0.190, 0.075, 0.032)),
        origin=Origin(),
        material=plastic,
        name="controller_body",
    )
    handset.visual(
        Box((0.160, 0.040, 0.014)),
        origin=Origin(xyz=(0.0, 0.0575, 0.023)),
        material=plastic,
        name="mount_tab",
    )
    handset.visual(
        Box((0.014, 0.016, 0.052)),
        origin=Origin(xyz=(0.0, -0.0455, -0.008)),
        material=plastic,
        name="center_divider",
    )
    handset.visual(
        Box((0.154, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, -0.052, 0.018)),
        material=plastic,
        name="pivot_lip",
    )
    model.articulation(
        "desktop_to_handset",
        ArticulationType.FIXED,
        parent=desktop,
        child=handset,
        origin=Origin(xyz=(0.43, -0.392, -0.050)),
    )

    def add_paddle(name: str, x: float, icon: str) -> None:
        paddle = model.part(name)
        paddle.visual(
            Cylinder(radius=0.006, length=0.052),
            origin=Origin(rpy=(0.0, 1.57079632679, 0.0)),
            material=plastic,
            name="hinge_barrel",
        )
        paddle.visual(
            Box((0.052, 0.010, 0.045)),
            origin=Origin(xyz=(0.0, -0.006, -0.026)),
            material=rubber,
            name="paddle_plate",
        )
        # Raised white icons are intentionally small separate inlaid features on
        # each paddle face; the two paddles remain split by a visible center gap.
        if icon == "up":
            paddle.visual(
                Box((0.006, 0.003, 0.020)),
                origin=Origin(xyz=(0.0, -0.012, -0.028)),
                material=label_white,
                name="arrow_stem",
            )
            paddle.visual(
                Box((0.024, 0.003, 0.006)),
                origin=Origin(xyz=(0.0, -0.012, -0.015)),
                material=label_white,
                name="arrow_head",
            )
        else:
            paddle.visual(
                Box((0.006, 0.003, 0.020)),
                origin=Origin(xyz=(0.0, -0.012, -0.020)),
                material=label_white,
                name="arrow_stem",
            )
            paddle.visual(
                Box((0.024, 0.003, 0.006)),
                origin=Origin(xyz=(0.0, -0.012, -0.035)),
                material=label_white,
                name="arrow_head",
            )

        model.articulation(
            f"handset_to_{name}",
            ArticulationType.REVOLUTE,
            parent=handset,
            child=paddle,
            origin=Origin(xyz=(x, -0.0435, 0.004)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.4, velocity=3.0, lower=0.0, upper=PADDLE_PUSH_ANGLE),
        )

    add_paddle("up_paddle", -0.037, "up")
    add_paddle("down_paddle", 0.037, "down")

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_frame")
    stage_0 = object_model.get_part("inner_stage_0")
    stage_1 = object_model.get_part("inner_stage_1")
    desktop = object_model.get_part("desktop")
    handset = object_model.get_part("handset")
    up_paddle = object_model.get_part("up_paddle")
    down_paddle = object_model.get_part("down_paddle")

    lift = object_model.get_articulation("lift_stage_0")
    lift_1 = object_model.get_articulation("lift_stage_1")
    up_hinge = object_model.get_articulation("handset_to_up_paddle")
    down_hinge = object_model.get_articulation("handset_to_down_paddle")

    ctx.check(
        "second lift stage mimics primary lift",
        lift_1.mimic is not None and lift_1.mimic.joint == "lift_stage_0",
        details=f"mimic={lift_1.mimic}",
    )

    ctx.expect_gap(
        desktop,
        stage_0,
        axis="z",
        positive_elem="desktop_top",
        negative_elem="top_plate",
        max_gap=0.001,
        max_penetration=0.0,
        name="desktop seats on first top plate",
    )
    ctx.expect_gap(
        desktop,
        stage_1,
        axis="z",
        positive_elem="desktop_top",
        negative_elem="top_plate",
        max_gap=0.001,
        max_penetration=0.0,
        name="desktop seats on second top plate",
    )
    ctx.expect_gap(
        desktop,
        handset,
        axis="z",
        positive_elem="desktop_top",
        negative_elem="mount_tab",
        max_gap=0.001,
        max_penetration=0.0,
        name="handset mount tab touches desktop underside",
    )

    ctx.expect_gap(
        down_paddle,
        up_paddle,
        axis="x",
        min_gap=0.010,
        positive_elem="paddle_plate",
        negative_elem="paddle_plate",
        name="up and down paddles stay visibly split",
    )

    ctx.expect_overlap(
        stage_0,
        base,
        axes="z",
        elem_a="inner_tube",
        elem_b="column_0_front_wall",
        min_overlap=0.35,
        name="first lift tube remains inserted at sitting height",
    )
    ctx.expect_overlap(
        stage_1,
        base,
        axes="z",
        elem_a="inner_tube",
        elem_b="column_1_front_wall",
        min_overlap=0.35,
        name="second lift tube remains inserted at sitting height",
    )

    stage_0_rest = ctx.part_world_position(stage_0)
    stage_1_rest = ctx.part_world_position(stage_1)
    with ctx.pose({lift: LIFT_TRAVEL}):
        stage_0_extended = ctx.part_world_position(stage_0)
        stage_1_extended = ctx.part_world_position(stage_1)
        desktop_extended = ctx.part_world_position(desktop)
        ctx.expect_gap(
            desktop,
            stage_1,
            axis="z",
            positive_elem="desktop_top",
            negative_elem="top_plate",
            max_gap=0.001,
            max_penetration=0.0,
            name="second top plate stays aligned at standing height",
        )
        ctx.expect_overlap(
            stage_0,
            base,
            axes="z",
            elem_a="inner_tube",
            elem_b="column_0_front_wall",
            min_overlap=0.08,
            name="first lift tube retains insertion at standing height",
        )
        ctx.expect_overlap(
            stage_1,
            base,
            axes="z",
            elem_a="inner_tube",
            elem_b="column_1_front_wall",
            min_overlap=0.08,
            name="second lift tube retains insertion at standing height",
        )

    ctx.check(
        "lift stages remain vertically synchronized",
        stage_0_rest is not None
        and stage_1_rest is not None
        and stage_0_extended is not None
        and stage_1_extended is not None
        and abs(stage_0_rest[2] - stage_1_rest[2]) < 1e-6
        and abs(stage_0_extended[2] - stage_1_extended[2]) < 1e-6,
        details=f"rest=({stage_0_rest}, {stage_1_rest}) extended=({stage_0_extended}, {stage_1_extended})",
    )
    ctx.check(
        "desktop rises by standing travel",
        desktop_extended is not None
        and stage_0_rest is not None
        and desktop_extended[2] > stage_0_rest[2] + DESKTOP_CENTER_Z_FROM_STAGE + 0.40,
        details=f"desktop_extended={desktop_extended}, stage_0_rest={stage_0_rest}",
    )

    rest_up = ctx.part_element_world_aabb(up_paddle, elem="paddle_plate")
    rest_down = ctx.part_element_world_aabb(down_paddle, elem="paddle_plate")
    with ctx.pose({up_hinge: PADDLE_PUSH_ANGLE, down_hinge: PADDLE_PUSH_ANGLE}):
        pushed_up = ctx.part_element_world_aabb(up_paddle, elem="paddle_plate")
        pushed_down = ctx.part_element_world_aabb(down_paddle, elem="paddle_plate")

    ctx.check(
        "up paddle pivots forward",
        rest_up is not None and pushed_up is not None and pushed_up[0][1] < rest_up[0][1] - 0.005,
        details=f"rest={rest_up}, pushed={pushed_up}",
    )
    ctx.check(
        "down paddle pivots forward",
        rest_down is not None and pushed_down is not None and pushed_down[0][1] < rest_down[0][1] - 0.005,
        details=f"rest={rest_down}, pushed={pushed_down}",
    )

    return ctx.report()


object_model = build_object_model()
