from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="floor_standing_drafting_table")

    painted_steel = model.material("satin_black_steel", color=(0.05, 0.055, 0.06, 1.0))
    dark_steel = model.material("dark_inner_steel", color=(0.12, 0.13, 0.14, 1.0))
    aluminum = model.material("brushed_aluminum", color=(0.62, 0.64, 0.62, 1.0))
    board_core = model.material("warm_maple_edge", color=(0.76, 0.55, 0.32, 1.0))
    drawing_mat = model.material("pale_drawing_surface", color=(0.86, 0.88, 0.78, 1.0))
    drawer_mat = model.material("painted_drawer", color=(0.22, 0.24, 0.25, 1.0))

    base = model.part("base")

    # Twin floor feet and a low stretcher frame, sized for a drafting-studio table.
    for x in (-0.42, 0.42):
        base.visual(
            Box((0.12, 0.92, 0.07)),
            origin=Origin(xyz=(x, 0.0, 0.035)),
            material=painted_steel,
            name=f"floor_foot_{0 if x < 0 else 1}",
        )

    for y in (-0.34, 0.34):
        base.visual(
            Box((0.96, 0.055, 0.06)),
            origin=Origin(xyz=(0.0, y, 0.075)),
            material=painted_steel,
            name=f"floor_stretcher_{0 if y < 0 else 1}",
        )

    base.visual(
        Box((0.82, 0.045, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.31)),
        material=painted_steel,
        name="post_tie_bar",
    )

    # Two open square sleeves.  Each sleeve is built from four walls and a top
    # collar so the inner stages can really pass through a visible clearance.
    for post_i, x in enumerate((-0.42, 0.42)):
        for y, wall_name in ((-0.0365, "rear"), (0.0365, "front")):
            post_wall_name = f"outer_post_{post_i}_{wall_name}_wall"
            if post_i == 0 and wall_name == "front":
                post_wall_name = "outer_post_0_front_wall"
            elif post_i == 1 and wall_name == "front":
                post_wall_name = "outer_post_1_front_wall"
            base.visual(
                Box((0.112, 0.018, 0.65)),
                origin=Origin(xyz=(x, y, 0.385)),
                material=painted_steel,
                name=post_wall_name,
            )
        for xoff, wall_name in ((-0.0365, "side_0"), (0.0365, "side_1")):
            base.visual(
                Box((0.018, 0.112, 0.65)),
                origin=Origin(xyz=(x + xoff, 0.0, 0.385)),
                material=painted_steel,
                name=f"outer_post_{post_i}_{wall_name}_wall",
            )
        base.visual(
            Box((0.145, 0.032, 0.05)),
            origin=Origin(xyz=(x, -0.052, 0.705)),
            material=aluminum,
            name=f"top_collar_{post_i}_rear",
        )
        base.visual(
            Box((0.145, 0.032, 0.05)),
            origin=Origin(xyz=(x, 0.052, 0.705)),
            material=aluminum,
            name=f"top_collar_{post_i}_front",
        )
        base.visual(
            Box((0.032, 0.145, 0.05)),
            origin=Origin(xyz=(x - 0.052, 0.0, 0.705)),
            material=aluminum,
            name=f"top_collar_{post_i}_side_0",
        )
        base.visual(
            Box((0.032, 0.145, 0.05)),
            origin=Origin(xyz=(x + 0.052, 0.0, 0.705)),
            material=aluminum,
            name=f"top_collar_{post_i}_side_1",
        )

    inner_stages = model.part("inner_stages")
    for stage_i, x in enumerate((-0.42, 0.42)):
        stage_name = "inner_stage_0" if stage_i == 0 else "inner_stage_1"
        inner_stages.visual(
            Box((0.055, 0.055, 0.60)),
            # Lower ends remain hidden in the outer sleeves even at full travel.
            origin=Origin(xyz=(x, 0.0, 0.025)),
            material=dark_steel,
            name=stage_name,
        )

    inner_stages.visual(
        Box((0.92, 0.075, 0.07)),
        origin=Origin(xyz=(0.0, 0.0, 0.35)),
        material=dark_steel,
        name="head_crossbar",
    )
    inner_stages.visual(
        Box((0.17, 0.11, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.425)),
        material=dark_steel,
        name="central_support",
    )
    inner_stages.visual(
        Box((0.15, 0.25, 0.07)),
        origin=Origin(xyz=(0.0, -0.105, 0.465)),
        material=dark_steel,
        name="support_neck",
    )
    inner_stages.visual(
        Cylinder(radius=0.024, length=0.58),
        origin=Origin(xyz=(0.0, -0.19, 0.49), rpy=(0.0, 1.57079632679, 0.0)),
        material=aluminum,
        name="transverse_hinge_pin",
    )
    for ear_i, x in enumerate((-0.12, 0.12)):
        inner_stages.visual(
            Box((0.040, 0.080, 0.12)),
            origin=Origin(xyz=(x, -0.19, 0.455)),
            material=dark_steel,
            name=f"support_clevis_{ear_i}",
        )

    height_joint = model.articulation(
        "base_to_inner_stages",
        ArticulationType.PRISMATIC,
        parent=base,
        child=inner_stages,
        origin=Origin(xyz=(0.0, 0.0, 0.70)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=260.0, velocity=0.16, lower=0.0, upper=0.18),
    )

    board = model.part("board")
    board.visual(
        Box((1.36, 0.86, 0.035)),
        origin=Origin(xyz=(0.0, 0.14, 0.055)),
        material=drawing_mat,
        name="drawing_surface",
    )
    # Warm wood edging and a stiff under-frame keep the panel from reading as a
    # single featureless slab.
    board.visual(
        Box((1.40, 0.035, 0.055)),
        origin=Origin(xyz=(0.0, -0.305, 0.025)),
        material=board_core,
        name="rear_edge",
    )
    board.visual(
        Box((1.40, 0.035, 0.055)),
        origin=Origin(xyz=(0.0, 0.585, 0.025)),
        material=board_core,
        name="front_edge",
    )
    for rail_i, x in enumerate((-0.685, 0.685)):
        board.visual(
            Box((0.035, 0.89, 0.055)),
            origin=Origin(xyz=(x, 0.14, 0.025)),
            material=board_core,
            name=f"side_edge_{rail_i}",
        )

    board.visual(
        Box((1.24, 0.032, 0.030)),
        origin=Origin(xyz=(0.0, 0.585, 0.0875)),
        material=aluminum,
        name="paper_stop_lip",
    )

    # Board-side hinge leaves sit just above the pin carried by the support head.
    for leaf_i, x in enumerate((-0.24, 0.24)):
        board.visual(
            Box((0.060, 0.032, 0.050)),
            origin=Origin(xyz=(x, 0.0, 0.049)),
            material=aluminum,
            name=f"board_hinge_leaf_{leaf_i}",
        )

    # Short runners and brackets below the frame carry the shallow accessory
    # drawer without hiding it inside a bulky cabinet.
    for y, name_suffix in ((0.30, "rear"), (0.60, "front")):
        board.visual(
            Box((1.28, 0.035, 0.035)),
            origin=Origin(xyz=(0.0, y, -0.015)),
            material=dark_steel,
            name=f"runner_crossbar_{name_suffix}",
        )
    for runner_i, x in enumerate((-0.4225, 0.4225)):
        runner_name = "drawer_runner_0" if runner_i == 0 else "drawer_runner_1"
        board.visual(
            Box((0.030, 0.46, 0.035)),
            origin=Origin(xyz=(x, 0.46, -0.075)),
            material=dark_steel,
            name=runner_name,
        )
        for y, hanger_name in ((0.30, "rear"), (0.60, "front")):
            board.visual(
                Box((0.046, 0.030, 0.070)),
                origin=Origin(xyz=(x, y, -0.045)),
                material=dark_steel,
                name=f"runner_hanger_{runner_i}_{hanger_name}",
            )

    board_joint = model.articulation(
        "inner_stages_to_board",
        ArticulationType.REVOLUTE,
        parent=inner_stages,
        child=board,
        origin=Origin(xyz=(0.0, -0.19, 0.49)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.55, lower=0.0, upper=1.05),
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.78, 0.36, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        material=drawer_mat,
        name="tray_bottom",
    )
    for side_i, x in enumerate((-0.395, 0.395)):
        drawer.visual(
            Box((0.025, 0.36, 0.062)),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=drawer_mat,
            name=f"tray_side_{side_i}",
        )
    drawer.visual(
        Box((0.82, 0.025, 0.070)),
        origin=Origin(xyz=(0.0, 0.18, 0.002)),
        material=drawer_mat,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.78, 0.025, 0.055)),
        origin=Origin(xyz=(0.0, -0.18, -0.002)),
        material=drawer_mat,
        name="drawer_back",
    )
    drawer.visual(
        Box((0.24, 0.030, 0.026)),
        origin=Origin(xyz=(0.0, 0.198, 0.026)),
        material=aluminum,
        name="front_pull",
    )

    drawer_joint = model.articulation(
        "board_to_drawer",
        ArticulationType.PRISMATIC,
        parent=board,
        child=drawer,
        origin=Origin(xyz=(0.0, 0.46, -0.115)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.28, lower=0.0, upper=0.30),
    )

    # Keep references explicit for static analyzers while preserving concise names.
    _ = (height_joint, board_joint, drawer_joint)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    inner_stages = object_model.get_part("inner_stages")
    board = object_model.get_part("board")
    drawer = object_model.get_part("drawer")
    height_joint = object_model.get_articulation("base_to_inner_stages")
    board_joint = object_model.get_articulation("inner_stages_to_board")
    drawer_joint = object_model.get_articulation("board_to_drawer")

    ctx.expect_overlap(
        inner_stages,
        base,
        axes="z",
        elem_a="inner_stage_0",
        elem_b="outer_post_0_front_wall",
        min_overlap=0.18,
        name="first inner stage is retained inside its outer post",
    )
    ctx.expect_overlap(
        inner_stages,
        base,
        axes="z",
        elem_a="inner_stage_1",
        elem_b="outer_post_1_front_wall",
        min_overlap=0.18,
        name="second inner stage is retained inside its outer post",
    )
    ctx.expect_overlap(
        drawer,
        board,
        axes="y",
        elem_a="tray_bottom",
        elem_b="drawer_runner_0",
        min_overlap=0.25,
        name="closed drawer is supported along the runner length",
    )

    rest_height = ctx.part_world_position(inner_stages)
    with ctx.pose({height_joint: 0.18}):
        raised_height = ctx.part_world_position(inner_stages)
        ctx.expect_overlap(
            inner_stages,
            base,
            axes="z",
            elem_a="inner_stage_0",
            elem_b="outer_post_0_front_wall",
            min_overlap=0.08,
            name="raised first stage remains inserted",
        )
        ctx.expect_overlap(
            inner_stages,
            base,
            axes="z",
            elem_a="inner_stage_1",
            elem_b="outer_post_1_front_wall",
            min_overlap=0.08,
            name="raised second stage remains inserted",
        )
    ctx.check(
        "height stage translates upward",
        rest_height is not None and raised_height is not None and raised_height[2] > rest_height[2] + 0.16,
        details=f"rest={rest_height}, raised={raised_height}",
    )

    flat_surface = ctx.part_element_world_aabb(board, elem="drawing_surface")
    with ctx.pose({board_joint: 0.85}):
        tilted_surface = ctx.part_element_world_aabb(board, elem="drawing_surface")
    ctx.check(
        "board hinge lifts the drawing surface",
        flat_surface is not None
        and tilted_surface is not None
        and tilted_surface[1][2] > flat_surface[1][2] + 0.25,
        details=f"flat={flat_surface}, tilted={tilted_surface}",
    )

    drawer_rest = ctx.part_world_position(drawer)
    with ctx.pose({drawer_joint: 0.30}):
        drawer_extended = ctx.part_world_position(drawer)
        ctx.expect_overlap(
            drawer,
            board,
            axes="y",
            elem_a="tray_bottom",
            elem_b="drawer_runner_0",
            min_overlap=0.08,
            name="extended drawer remains on the short runners",
        )
    ctx.check(
        "drawer slides forward on its guide runners",
        drawer_rest is not None
        and drawer_extended is not None
        and drawer_extended[1] > drawer_rest[1] + 0.25,
        details=f"rest={drawer_rest}, extended={drawer_extended}",
    )

    return ctx.report()


object_model = build_object_model()
