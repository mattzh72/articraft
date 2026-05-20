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

# Advanced; only use CadQuery if the native sdk is not enough to represent the shapes you want:
# import cadquery as cq
# from sdk import mesh_from_cadquery


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="standing_desk")

    model.material("frame_black", rgba=(0.16, 0.16, 0.17, 1.0))
    model.material("frame_dark", rgba=(0.12, 0.12, 0.13, 1.0))
    model.material("desktop_oak", rgba=(0.66, 0.54, 0.40, 1.0))
    model.material("accent_grey", rgba=(0.52, 0.54, 0.57, 1.0))
    model.material("controller_black", rgba=(0.08, 0.08, 0.09, 1.0))
    model.material("controller_grey", rgba=(0.32, 0.34, 0.36, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.846, 0.080, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.570)),
        material="frame_black",
        name="crossbeam",
    )

    column_specs = (
        ("column_0", -0.470),
        ("column_1", 0.470),
    )
    stage_specs = (
        ("lift_0", "column_0", None),
        ("lift_1", "column_1", Mimic(joint="column_0_to_lift_0")),
    )

    for column_name, x_pos in column_specs:
        column = model.part(column_name)
        column.visual(
            Box((0.080, 0.680, 0.030)),
            origin=Origin(xyz=(0.0, 0.0, -0.585)),
            material="frame_dark",
            name="foot",
        )
        column.visual(
            Box((0.094, 0.007, 0.570)),
            origin=Origin(xyz=(0.0, 0.0265, -0.285)),
            material="frame_black",
            name="front_wall",
        )
        column.visual(
            Box((0.094, 0.007, 0.570)),
            origin=Origin(xyz=(0.0, -0.0265, -0.285)),
            material="frame_black",
            name="rear_wall",
        )
        column.visual(
            Box((0.007, 0.046, 0.570)),
            origin=Origin(xyz=(-0.0435, 0.0, -0.285)),
            material="frame_black",
            name="inner_wall",
        )
        column.visual(
            Box((0.007, 0.046, 0.570)),
            origin=Origin(xyz=(0.0435, 0.0, -0.285)),
            material="frame_black",
            name="outer_wall",
        )
        model.articulation(
            f"frame_to_{column_name}",
            ArticulationType.FIXED,
            parent=frame,
            child=column,
            origin=Origin(xyz=(x_pos, 0.0, 0.600)),
        )

    for stage_name, column_name, mimic in stage_specs:
        stage = model.part(stage_name)
        stage.visual(
            Box((0.072, 0.038, 0.670)),
            origin=Origin(xyz=(0.0, 0.0, -0.235)),
            material="accent_grey",
            name="stage_member",
        )
        stage.visual(
            Box((0.150, 0.120, 0.015)),
            origin=Origin(xyz=(0.0, 0.0, 0.1075)),
            material="frame_black",
            name="top_plate",
        )
        model.articulation(
            f"{column_name}_to_{stage_name}",
            ArticulationType.PRISMATIC,
            parent=column_name,
            child=stage,
            origin=Origin(),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                lower=0.0,
                upper=0.370,
                effort=800.0,
                velocity=0.040,
            ),
            mimic=mimic,
        )

    desktop = model.part("desktop")
    desktop.visual(
        Box((1.600, 0.750, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material="desktop_oak",
        name="top_panel",
    )
    desktop.visual(
        Box((0.760, 0.100, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material="frame_black",
        name="underframe",
    )
    model.articulation(
        "lift_0_to_desktop",
        ArticulationType.FIXED,
        parent="lift_0",
        child=desktop,
        origin=Origin(xyz=(0.470, 0.0, 0.115)),
    )

    handset = model.part("handset")
    handset.visual(
        Box((0.150, 0.055, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material="controller_black",
        name="body",
    )
    handset.visual(
        Box((0.090, 0.034, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.026)),
        material="controller_black",
        name="chin",
    )
    model.articulation(
        "desktop_to_handset",
        ArticulationType.FIXED,
        parent=desktop,
        child=handset,
        origin=Origin(xyz=(0.500, 0.330, 0.0)),
    )

    up_paddle = model.part("up_paddle")
    up_paddle.visual(
        Box((0.030, 0.010, 0.022)),
        origin=Origin(xyz=(0.0, 0.005, -0.011)),
        material="controller_grey",
        name="face",
    )
    model.articulation(
        "handset_to_up_paddle",
        ArticulationType.REVOLUTE,
        parent=handset,
        child=up_paddle,
        origin=Origin(xyz=(-0.020, 0.0275, -0.008)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.220,
            upper=0.080,
            effort=1.5,
            velocity=2.0,
        ),
    )

    down_paddle = model.part("down_paddle")
    down_paddle.visual(
        Box((0.030, 0.010, 0.022)),
        origin=Origin(xyz=(0.0, 0.005, -0.011)),
        material="controller_grey",
        name="face",
    )
    model.articulation(
        "handset_to_down_paddle",
        ArticulationType.REVOLUTE,
        parent=handset,
        child=down_paddle,
        origin=Origin(xyz=(0.020, 0.0275, -0.008)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.220,
            upper=0.080,
            effort=1.5,
            velocity=2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    column_0 = object_model.get_part("column_0")
    column_1 = object_model.get_part("column_1")
    lift_0 = object_model.get_part("lift_0")
    lift_1 = object_model.get_part("lift_1")
    desktop = object_model.get_part("desktop")
    handset = object_model.get_part("handset")
    up_paddle = object_model.get_part("up_paddle")
    down_paddle = object_model.get_part("down_paddle")

    lift_joint = object_model.get_articulation("column_0_to_lift_0")
    up_joint = object_model.get_articulation("handset_to_up_paddle")
    down_joint = object_model.get_articulation("handset_to_down_paddle")

    lift_limits = lift_joint.motion_limits
    up_limits = up_joint.motion_limits
    down_limits = down_joint.motion_limits

    ctx.expect_origin_distance(
        lift_0,
        lift_1,
        axes="z",
        max_dist=0.001,
        name="lift stages start aligned in height",
    )
    ctx.expect_overlap(
        desktop,
        lift_0,
        axes="xy",
        elem_a="top_panel",
        elem_b="top_plate",
        min_overlap=0.120,
        name="desktop overlaps left lift plate",
    )
    ctx.expect_overlap(
        desktop,
        lift_1,
        axes="xy",
        elem_a="top_panel",
        elem_b="top_plate",
        min_overlap=0.120,
        name="desktop overlaps right lift plate",
    )
    ctx.expect_gap(
        desktop,
        lift_0,
        axis="z",
        positive_elem="top_panel",
        negative_elem="top_plate",
        max_gap=0.001,
        max_penetration=0.0,
        name="desktop seats on left lift plate",
    )
    ctx.expect_gap(
        desktop,
        lift_1,
        axis="z",
        positive_elem="top_panel",
        negative_elem="top_plate",
        max_gap=0.001,
        max_penetration=0.0,
        name="desktop seats on right lift plate",
    )
    ctx.expect_gap(
        down_paddle,
        up_paddle,
        axis="x",
        min_gap=0.008,
        name="controller paddles remain visibly split",
    )

    rest_lift_0 = ctx.part_world_position(lift_0)
    rest_lift_1 = ctx.part_world_position(lift_1)
    rest_up_face = ctx.part_element_world_aabb(up_paddle, elem="face")
    rest_down_face = ctx.part_element_world_aabb(down_paddle, elem="face")

    if lift_limits is not None and lift_limits.upper is not None:
        with ctx.pose({lift_joint: lift_limits.upper}):
            ctx.expect_origin_distance(
                lift_0,
                lift_1,
                axes="z",
                max_dist=0.001,
                name="lift stages stay aligned at full rise",
            )
            ctx.expect_origin_distance(
                lift_0,
                column_0,
                axes="xy",
                max_dist=0.001,
                name="left lift stays centered in its outer column",
            )
            ctx.expect_origin_distance(
                lift_1,
                column_1,
                axes="xy",
                max_dist=0.001,
                name="right lift stays centered in its outer column",
            )
            ctx.expect_overlap(
                lift_0,
                column_0,
                axes="z",
                elem_a="stage_member",
                min_overlap=0.180,
                name="left lift keeps retained insertion at full rise",
            )
            ctx.expect_overlap(
                lift_1,
                column_1,
                axes="z",
                elem_a="stage_member",
                min_overlap=0.180,
                name="right lift keeps retained insertion at full rise",
            )
            ctx.expect_gap(
                desktop,
                lift_0,
                axis="z",
                positive_elem="top_panel",
                negative_elem="top_plate",
                max_gap=0.001,
                max_penetration=0.0,
                name="desktop stays seated on left plate at full rise",
            )
            ctx.expect_gap(
                desktop,
                lift_1,
                axis="z",
                positive_elem="top_panel",
                negative_elem="top_plate",
                max_gap=0.001,
                max_penetration=0.0,
                name="desktop stays seated on right plate at full rise",
            )
            raised_lift_0 = ctx.part_world_position(lift_0)
            raised_lift_1 = ctx.part_world_position(lift_1)
        ctx.check(
            "both lift stages rise together",
            rest_lift_0 is not None
            and rest_lift_1 is not None
            and raised_lift_0 is not None
            and raised_lift_1 is not None
            and raised_lift_0[2] > rest_lift_0[2] + 0.300
            and raised_lift_1[2] > rest_lift_1[2] + 0.300
            and abs((raised_lift_0[2] - rest_lift_0[2]) - (raised_lift_1[2] - rest_lift_1[2])) < 1e-6,
            details=(
                f"rest_0={rest_lift_0}, raised_0={raised_lift_0}, "
                f"rest_1={rest_lift_1}, raised_1={raised_lift_1}"
            ),
        )

    if up_limits is not None and up_limits.lower is not None:
        with ctx.pose({up_joint: up_limits.lower}):
            pressed_up_face = ctx.part_element_world_aabb(up_paddle, elem="face")
        ctx.check(
            "up paddle rotates on its local pivot",
            rest_up_face is not None
            and pressed_up_face is not None
            and pressed_up_face[0][1] < rest_up_face[0][1] - 0.003,
            details=f"rest={rest_up_face}, pressed={pressed_up_face}",
        )

    if down_limits is not None and down_limits.lower is not None:
        with ctx.pose({down_joint: down_limits.lower}):
            pressed_down_face = ctx.part_element_world_aabb(down_paddle, elem="face")
        ctx.check(
            "down paddle rotates on its local pivot",
            rest_down_face is not None
            and pressed_down_face is not None
            and pressed_down_face[0][1] < rest_down_face[0][1] - 0.003,
            details=f"rest={rest_down_face}, pressed={pressed_down_face}",
        )

    ctx.expect_gap(
        desktop,
        handset,
        axis="z",
        positive_elem="top_panel",
        negative_elem="body",
        min_gap=0.0,
        max_gap=0.001,
        name="handset hangs directly beneath the desktop",
    )

    return ctx.report()


object_model = build_object_model()
