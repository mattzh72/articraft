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
    model = ArticulatedObject(name="drafting_stand")

    powder = model.material("powder", rgba=(0.18, 0.19, 0.20, 1.0))
    graphite = model.material("graphite", rgba=(0.28, 0.29, 0.31, 1.0))
    aluminum = model.material("aluminum", rgba=(0.70, 0.72, 0.74, 1.0))
    board_face = model.material("board_face", rgba=(0.78, 0.76, 0.71, 1.0))
    drawer_face = model.material("drawer_face", rgba=(0.33, 0.34, 0.37, 1.0))

    base_thickness = 0.035
    outer_width = 0.090
    outer_depth = 0.070
    wall = 0.008
    outer_bottom = 0.045
    outer_height = 0.350
    outer_top = outer_bottom + outer_height

    board_depth = 0.430
    board_width = 0.580

    pedestal_base = model.part("pedestal_base")
    pedestal_base.visual(
        Cylinder(radius=0.210, length=base_thickness),
        origin=Origin(xyz=(0.0, 0.0, base_thickness / 2.0)),
        material=powder,
        name="base_disk",
    )
    pedestal_base.visual(
        Cylinder(radius=0.072, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, base_thickness + 0.009)),
        material=powder,
        name="base_riser",
    )
    pedestal_base.visual(
        Box((0.112, 0.092, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0525)),
        material=powder,
        name="sleeve_collar",
    )
    pedestal_base.visual(
        Box((wall, outer_depth, outer_height)),
        origin=Origin(
            xyz=(-(outer_width / 2.0) + (wall / 2.0), 0.0, outer_bottom + outer_height / 2.0)
        ),
        material=graphite,
        name="sleeve_front",
    )
    pedestal_base.visual(
        Box((wall, outer_depth, outer_height)),
        origin=Origin(
            xyz=((outer_width / 2.0) - (wall / 2.0), 0.0, outer_bottom + outer_height / 2.0)
        ),
        material=graphite,
        name="sleeve_rear",
    )
    pedestal_base.visual(
        Box((outer_width - 2.0 * wall, wall, outer_height)),
        origin=Origin(
            xyz=(0.0, -(outer_depth / 2.0) + (wall / 2.0), outer_bottom + outer_height / 2.0)
        ),
        material=graphite,
        name="sleeve_left",
    )
    pedestal_base.visual(
        Box((outer_width - 2.0 * wall, wall, outer_height)),
        origin=Origin(
            xyz=(0.0, (outer_depth / 2.0) - (wall / 2.0), outer_bottom + outer_height / 2.0)
        ),
        material=graphite,
        name="sleeve_right",
    )

    inner_post = model.part("inner_post")
    inner_post.visual(
        Box((0.062, 0.042, 0.600)),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=aluminum,
        name="mast",
    )
    inner_post.visual(
        Box((0.108, 0.088, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=powder,
        name="adjuster_collar",
    )
    inner_post.visual(
        Box((0.080, 0.060, 0.040)),
        origin=Origin(xyz=(0.010, 0.0, 0.284333)),
        material=powder,
        name="head_block",
    )
    inner_post.visual(
        Box((0.035, 0.170, 0.020)),
        origin=Origin(xyz=(0.004, 0.0, 0.246)),
        material=powder,
        name="yoke_bar",
    )
    inner_post.visual(
        Box((0.050, 0.160, 0.010)),
        origin=Origin(xyz=(-0.060, 0.0, 0.297), rpy=(0.0, -0.22, 0.0)),
        material=powder,
        name="support_pad",
    )
    inner_post.visual(
        Box((0.120, 0.120, 0.030)),
        origin=Origin(xyz=(-0.010, 0.0, 0.280)),
        material=powder,
        name="support_stem",
    )
    inner_post.visual(
        Box((0.024, 0.012, 0.070)),
        origin=Origin(xyz=(0.0, -0.078, 0.270)),
        material=powder,
        name="yoke_left",
    )
    inner_post.visual(
        Box((0.024, 0.012, 0.070)),
        origin=Origin(xyz=(0.0, 0.078, 0.270)),
        material=powder,
        name="yoke_right",
    )
    inner_post.visual(
        Cylinder(radius=0.011, length=0.020),
        origin=Origin(xyz=(0.044, 0.0, 0.025), rpy=(0.0, 1.57079632679, 0.0)),
        material=powder,
        name="height_knob",
    )

    model.articulation(
        "pedestal_to_post",
        ArticulationType.PRISMATIC,
        parent=pedestal_base,
        child=inner_post,
        origin=Origin(xyz=(0.0, 0.0, outer_top)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.18,
            lower=0.0,
            upper=0.180,
        ),
    )

    board = model.part("board")
    board.visual(
        Box((board_depth, board_width, 0.014)),
        origin=Origin(xyz=(-board_depth / 2.0, 0.0, 0.018)),
        material=board_face,
        name="panel",
    )
    board.visual(
        Box((board_depth, 0.018, 0.024)),
        origin=Origin(xyz=(-board_depth / 2.0, -(board_width / 2.0) + 0.009, 0.017)),
        material=graphite,
        name="frame_left",
    )
    board.visual(
        Box((board_depth, 0.018, 0.024)),
        origin=Origin(xyz=(-board_depth / 2.0, (board_width / 2.0) - 0.009, 0.017)),
        material=graphite,
        name="frame_right",
    )
    board.visual(
        Box((0.030, board_width, 0.026)),
        origin=Origin(xyz=(-0.015, 0.0, 0.018)),
        material=graphite,
        name="rear_rail",
    )
    board.visual(
        Box((0.018, board_width, 0.024)),
        origin=Origin(xyz=(-board_depth + 0.009, 0.0, 0.017)),
        material=graphite,
        name="front_rail",
    )
    board.visual(
        Box((0.030, 0.460, 0.008)),
        origin=Origin(xyz=(-board_depth - 0.008, 0.0, 0.018)),
        material=graphite,
        name="lip_shelf",
    )
    board.visual(
        Box((0.012, 0.460, 0.025)),
        origin=Origin(xyz=(-board_depth + 0.006, 0.0, 0.0365)),
        material=graphite,
        name="lip_upstand",
    )
    board.visual(
        Box((0.090, 0.300, 0.018)),
        origin=Origin(xyz=(-0.090, 0.0, 0.002)),
        material=powder,
        name="runner_bridge",
    )
    board.visual(
        Box((0.180, 0.022, 0.012)),
        origin=Origin(xyz=(-0.180, -0.110, -0.009)),
        material=aluminum,
        name="runner_left",
    )
    board.visual(
        Box((0.180, 0.022, 0.012)),
        origin=Origin(xyz=(-0.180, 0.110, -0.009)),
        material=aluminum,
        name="runner_right",
    )
    board.visual(
        Box((0.030, 0.180, 0.016)),
        origin=Origin(xyz=(-0.060, 0.0, 0.004)),
        material=powder,
        name="hinge_spine",
    )

    model.articulation(
        "post_to_board",
        ArticulationType.REVOLUTE,
        parent=inner_post,
        child=board,
        origin=Origin(xyz=(0.0, 0.0, 0.306), rpy=(0.0, -0.22, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.6,
            lower=-0.180,
            upper=0.950,
        ),
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.160, 0.018, 0.010)),
        origin=Origin(xyz=(-0.080, -0.110, -0.005)),
        material=aluminum,
        name="shoe_left",
    )
    drawer.visual(
        Box((0.160, 0.018, 0.010)),
        origin=Origin(xyz=(-0.080, 0.110, -0.005)),
        material=aluminum,
        name="shoe_right",
    )
    drawer.visual(
        Box((0.180, 0.220, 0.006)),
        origin=Origin(xyz=(-0.090, 0.0, -0.026)),
        material=drawer_face,
        name="tray_bottom",
    )
    drawer.visual(
        Box((0.180, 0.006, 0.032)),
        origin=Origin(xyz=(-0.090, -0.110, -0.021)),
        material=drawer_face,
        name="tray_left",
    )
    drawer.visual(
        Box((0.180, 0.006, 0.032)),
        origin=Origin(xyz=(-0.090, 0.110, -0.021)),
        material=drawer_face,
        name="tray_right",
    )
    drawer.visual(
        Box((0.008, 0.220, 0.028)),
        origin=Origin(xyz=(-0.014, 0.0, -0.018)),
        material=drawer_face,
        name="tray_back",
    )
    drawer.visual(
        Box((0.010, 0.220, 0.032)),
        origin=Origin(xyz=(-0.175, 0.0, -0.020)),
        material=drawer_face,
        name="tray_front",
    )
    drawer.visual(
        Box((0.018, 0.080, 0.014)),
        origin=Origin(xyz=(-0.188, 0.0, -0.018)),
        material=powder,
        name="pull",
    )

    model.articulation(
        "board_to_drawer",
        ArticulationType.PRISMATIC,
        parent=board,
        child=drawer,
        origin=Origin(xyz=(-0.095, 0.0, -0.015)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.20,
            lower=0.0,
            upper=0.100,
        ),
    )

    return model


def _z_extent(ctx: TestContext, part_name: str, elem: str) -> tuple[float, float] | None:
    aabb = ctx.part_element_world_aabb(part_name, elem=elem)
    if aabb is None:
        return None
    return aabb[0][2], aabb[1][2]


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal_base = object_model.get_part("pedestal_base")
    inner_post = object_model.get_part("inner_post")
    board = object_model.get_part("board")
    drawer = object_model.get_part("drawer")

    post_slide = object_model.get_articulation("pedestal_to_post")
    board_tilt = object_model.get_articulation("post_to_board")
    drawer_slide = object_model.get_articulation("board_to_drawer")

    post_limits = post_slide.motion_limits
    tilt_limits = board_tilt.motion_limits
    drawer_limits = drawer_slide.motion_limits

    ctx.allow_overlap(
        board,
        inner_post,
        elem_a="runner_bridge",
        elem_b="support_pad",
        reason="The simplified tilt support uses a solid saddle pad nested into the underside bridge instead of a detailed notched cradle.",
    )
    ctx.allow_overlap(
        board,
        inner_post,
        elem_a="hinge_spine",
        elem_b="support_pad",
        reason="The simplified tilt support uses a solid saddle pad under the board spine instead of a detailed notched cradle.",
    )
    ctx.allow_overlap(
        board,
        inner_post,
        elem_a="runner_bridge",
        elem_b="support_stem",
        reason="The compact top-support web is simplified as a solid brace nested into the underside bridge.",
    )
    ctx.allow_overlap(
        board,
        inner_post,
        elem_a="hinge_spine",
        elem_b="support_stem",
        reason="The compact top-support web is simplified as a solid brace tucked into the underside spine.",
    )
    ctx.allow_overlap(
        board,
        drawer,
        elem_a="runner_left",
        elem_b="shoe_left",
        reason="The drawer guides are simplified as overlapping runner and shoe solids rather than detailed channeled slides.",
    )
    ctx.allow_overlap(
        board,
        drawer,
        elem_a="runner_right",
        elem_b="shoe_right",
        reason="The drawer guides are simplified as overlapping runner and shoe solids rather than detailed channeled slides.",
    )

    ctx.expect_gap(
        inner_post,
        pedestal_base,
        axis="x",
        positive_elem="mast",
        negative_elem="sleeve_front",
        min_gap=0.005,
        max_gap=0.007,
        name="mast clears front sleeve wall",
    )
    ctx.expect_gap(
        pedestal_base,
        inner_post,
        axis="x",
        positive_elem="sleeve_rear",
        negative_elem="mast",
        min_gap=0.005,
        max_gap=0.007,
        name="mast clears rear sleeve wall",
    )
    ctx.expect_gap(
        inner_post,
        pedestal_base,
        axis="y",
        positive_elem="mast",
        negative_elem="sleeve_left",
        min_gap=0.005,
        max_gap=0.007,
        name="mast clears left sleeve wall",
    )
    ctx.expect_gap(
        pedestal_base,
        inner_post,
        axis="y",
        positive_elem="sleeve_right",
        negative_elem="mast",
        min_gap=0.005,
        max_gap=0.007,
        name="mast clears right sleeve wall",
    )
    ctx.expect_overlap(
        inner_post,
        pedestal_base,
        axes="z",
        elem_a="mast",
        elem_b="sleeve_front",
        min_overlap=0.280,
        name="collapsed mast remains deeply nested in sleeve",
    )

    if post_limits is not None and post_limits.upper is not None:
        rest_post = ctx.part_world_position(inner_post)
        with ctx.pose({post_slide: post_limits.upper}):
            ctx.expect_overlap(
                inner_post,
                pedestal_base,
                axes="z",
                elem_a="mast",
                elem_b="sleeve_front",
                min_overlap=0.135,
                name="extended mast still retains insertion in sleeve",
            )
            extended_post = ctx.part_world_position(inner_post)
        ctx.check(
            "post extends upward",
            rest_post is not None
            and extended_post is not None
            and extended_post[2] > rest_post[2] + 0.12,
            details=f"rest={rest_post}, extended={extended_post}",
        )

    ctx.expect_within(
        drawer,
        board,
        axes="y",
        inner_elem="shoe_left",
        outer_elem="runner_left",
        margin=0.002,
        name="left drawer shoe stays aligned on runner",
    )
    ctx.expect_within(
        drawer,
        board,
        axes="y",
        inner_elem="shoe_right",
        outer_elem="runner_right",
        margin=0.002,
        name="right drawer shoe stays aligned on runner",
    )
    ctx.expect_overlap(
        drawer,
        board,
        axes="x",
        elem_a="shoe_left",
        elem_b="runner_left",
        min_overlap=0.140,
        name="closed drawer remains engaged on left runner",
    )
    ctx.expect_overlap(
        drawer,
        board,
        axes="x",
        elem_a="shoe_right",
        elem_b="runner_right",
        min_overlap=0.140,
        name="closed drawer remains engaged on right runner",
    )

    if drawer_limits is not None and drawer_limits.upper is not None:
        rest_drawer = ctx.part_world_position(drawer)
        with ctx.pose({drawer_slide: drawer_limits.upper}):
            ctx.expect_overlap(
                drawer,
                board,
                axes="x",
                elem_a="shoe_left",
                elem_b="runner_left",
                min_overlap=0.055,
                name="open drawer retains left runner engagement",
            )
            ctx.expect_overlap(
                drawer,
                board,
                axes="x",
                elem_a="shoe_right",
                elem_b="runner_right",
                min_overlap=0.055,
                name="open drawer retains right runner engagement",
            )
            extended_drawer = ctx.part_world_position(drawer)
        ctx.check(
            "drawer slides outward",
            rest_drawer is not None
            and extended_drawer is not None
            and extended_drawer[0] < rest_drawer[0] - 0.08,
            details=f"rest={rest_drawer}, extended={extended_drawer}",
        )

    if tilt_limits is not None and tilt_limits.lower is not None and tilt_limits.upper is not None:
        with ctx.pose({board_tilt: tilt_limits.lower}):
            low_lip = _z_extent(ctx, "board", "lip_upstand")
        with ctx.pose({board_tilt: tilt_limits.upper}):
            high_lip = _z_extent(ctx, "board", "lip_upstand")
        ctx.check(
            "board front edge lifts as tilt increases",
            low_lip is not None and high_lip is not None and high_lip[1] > low_lip[1] + 0.20,
            details=f"low={low_lip}, high={high_lip}",
        )

    return ctx.report()


object_model = build_object_model()
