from __future__ import annotations

import math

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


def _add_caster(
    model: ArticulatedObject,
    *,
    index: int,
    x: float,
    y: float,
    fork_material,
    wheel_material,
    parent,
) -> None:
    fork = model.part(f"caster_fork_{index}")
    fork.visual(
        Box((0.032, 0.020, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=fork_material,
        name="top_plate",
    )
    fork.visual(
        Cylinder(radius=0.008, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        material=fork_material,
        name="stem",
    )
    fork.visual(
        Box((0.030, 0.018, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=fork_material,
        name="bridge",
    )
    for sx in (-0.013, 0.013):
        fork.visual(
            Box((0.005, 0.016, 0.030)),
            origin=Origin(xyz=(-0.0125 if sx < 0.0 else 0.0125, 0.0, 0.015)),
            material=fork_material,
            name=f"leg_{0 if sx < 0.0 else 1}",
        )

    wheel = model.part(f"caster_wheel_{index}")
    wheel.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=wheel_material,
        name="wheel_tire",
    )
    wheel.visual(
        Cylinder(radius=0.014, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=fork_material,
        name="wheel_hub",
    )

    model.articulation(
        f"base_to_caster_fork_{index}",
        ArticulationType.FIXED,
        parent=parent,
        child=fork,
        origin=Origin(xyz=(x, y, 0.0)),
    )
    model.articulation(
        f"caster_spin_{index}",
        ArticulationType.CONTINUOUS,
        parent=fork,
        child=wheel,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=12.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hospital_overbed_table")

    frame_gray = model.material("frame_gray", rgba=(0.68, 0.70, 0.73, 1.0))
    darker_frame = model.material("darker_frame", rgba=(0.42, 0.45, 0.48, 1.0))
    tray_beige = model.material("tray_beige", rgba=(0.88, 0.84, 0.74, 1.0))
    stainless = model.material("stainless", rgba=(0.78, 0.80, 0.82, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.12, 0.12, 0.13, 1.0))
    control_dark = model.material("control_dark", rgba=(0.22, 0.23, 0.24, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.10, 0.62, 0.022)),
        origin=Origin(xyz=(-0.28, 0.0, 0.067)),
        material=frame_gray,
        name="spine",
    )
    base.visual(
        Box((0.63, 0.08, 0.022)),
        origin=Origin(xyz=(-0.015, 0.27, 0.067)),
        material=frame_gray,
        name="front_leg",
    )
    base.visual(
        Box((0.63, 0.08, 0.022)),
        origin=Origin(xyz=(-0.015, -0.27, 0.067)),
        material=frame_gray,
        name="rear_leg",
    )
    base.visual(
        Box((0.18, 0.14, 0.010)),
        origin=Origin(xyz=(-0.18, -0.02, 0.073)),
        material=darker_frame,
        name="post_pad",
    )

    for index, (x, y) in enumerate(
        (
            (-0.28, 0.27),
            (-0.28, -0.27),
            (0.30, 0.27),
            (0.30, -0.27),
        )
    ):
        _add_caster(
            model,
            index=index,
            x=x,
            y=y,
            fork_material=darker_frame,
            wheel_material=rubber_black,
            parent=base,
        )

    outer_post = model.part("outer_post")
    outer_post.visual(
        Box((0.11, 0.09, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=darker_frame,
        name="post_shoe",
    )
    outer_post.visual(
        Box((0.005, 0.052, 0.410)),
        origin=Origin(xyz=(-0.0365, 0.0, 0.223)),
        material=frame_gray,
        name="sleeve_left",
    )
    outer_post.visual(
        Box((0.005, 0.052, 0.410)),
        origin=Origin(xyz=(0.0365, 0.0, 0.223)),
        material=frame_gray,
        name="sleeve_right",
    )
    outer_post.visual(
        Box((0.068, 0.005, 0.410)),
        origin=Origin(xyz=(0.0, 0.0235, 0.223)),
        material=frame_gray,
        name="sleeve_front",
    )
    outer_post.visual(
        Box((0.068, 0.005, 0.410)),
        origin=Origin(xyz=(0.0, -0.0235, 0.223)),
        material=frame_gray,
        name="sleeve_back",
    )
    outer_post.visual(
        Box((0.010, 0.062, 0.040)),
        origin=Origin(xyz=(-0.033, 0.0, 0.408)),
        material=darker_frame,
        name="collar_left",
    )
    outer_post.visual(
        Box((0.010, 0.062, 0.040)),
        origin=Origin(xyz=(0.033, 0.0, 0.408)),
        material=darker_frame,
        name="collar_right",
    )
    outer_post.visual(
        Box((0.068, 0.010, 0.040)),
        origin=Origin(xyz=(0.0, 0.020, 0.408)),
        material=darker_frame,
        name="collar_front",
    )
    outer_post.visual(
        Box((0.068, 0.010, 0.040)),
        origin=Origin(xyz=(0.0, -0.020, 0.408)),
        material=darker_frame,
        name="collar_back",
    )

    inner_column = model.part("inner_column")
    inner_column.visual(
        Box((0.056, 0.030, 0.720)),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=stainless,
        name="inner_tube",
    )

    support_head = model.part("support_head")
    support_head.visual(
        Box((0.060, 0.050, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=darker_frame,
        name="neck_block",
    )
    support_head.visual(
        Box((0.340, 0.090, 0.024)),
        origin=Origin(xyz=(0.160, -0.090, 0.058)),
        material=darker_frame,
        name="head_arm",
    )
    support_head.visual(
        Box((0.080, 0.060, 0.024)),
        origin=Origin(xyz=(0.030, -0.040, 0.049)),
        material=darker_frame,
        name="arm_web",
    )
    support_head.visual(
        Box((0.160, 0.050, 0.020)),
        origin=Origin(xyz=(0.160, -0.155, 0.072)),
        material=frame_gray,
        name="head_saddle",
    )
    support_head.visual(
        Box((0.020, 0.030, 0.044)),
        origin=Origin(xyz=(0.095, -0.155, 0.060)),
        material=darker_frame,
        name="hinge_cheek_0",
    )
    support_head.visual(
        Box((0.020, 0.030, 0.044)),
        origin=Origin(xyz=(0.225, -0.155, 0.060)),
        material=darker_frame,
        name="hinge_cheek_1",
    )

    tray = model.part("tray")
    tray.visual(
        Box((0.780, 0.400, 0.012)),
        origin=Origin(xyz=(0.190, 0.200, 0.006)),
        material=tray_beige,
        name="tray_panel",
    )
    tray.visual(
        Box((0.780, 0.012, 0.020)),
        origin=Origin(xyz=(0.190, 0.006, 0.022)),
        material=tray_beige,
        name="rear_lip",
    )
    tray.visual(
        Box((0.780, 0.012, 0.020)),
        origin=Origin(xyz=(0.190, 0.394, 0.022)),
        material=tray_beige,
        name="front_lip",
    )
    tray.visual(
        Box((0.012, 0.376, 0.020)),
        origin=Origin(xyz=(-0.194, 0.200, 0.022)),
        material=tray_beige,
        name="side_lip_0",
    )
    tray.visual(
        Box((0.012, 0.376, 0.020)),
        origin=Origin(xyz=(0.574, 0.200, 0.022)),
        material=tray_beige,
        name="side_lip_1",
    )

    clip_bar = model.part("clip_bar")
    for index, local_x in enumerate((-0.160, 0.160)):
        clip_bar.visual(
            Cylinder(radius=0.004, length=0.018),
            origin=Origin(xyz=(local_x, 0.006, 0.003), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=stainless,
            name=f"hinge_barrel_{index}",
        )
        clip_bar.visual(
            Box((0.014, 0.028, 0.018)),
            origin=Origin(xyz=(local_x, 0.014, 0.009)),
            material=stainless,
            name=f"clip_support_{index}",
        )
    clip_bar.visual(
        Cylinder(radius=0.006, length=0.600),
        origin=Origin(xyz=(0.0, 0.028, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="clip_rod",
    )

    lever = model.part("lever")
    for index, local_x in enumerate((-0.050, 0.050)):
        lever.visual(
            Cylinder(radius=0.004, length=0.022),
            origin=Origin(xyz=(local_x, 0.0, -0.004), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=darker_frame,
            name=f"pivot_barrel_{index}",
        )
        lever.visual(
            Box((0.018, 0.040, 0.010)),
            origin=Origin(xyz=(local_x, 0.020, -0.013)),
            material=darker_frame,
            name=f"lever_arm_{index}",
        )
    lever.visual(
        Box((0.150, 0.020, 0.010)),
        origin=Origin(xyz=(0.0, 0.040, -0.020)),
        material=control_dark,
        name="lever_handle",
    )

    model.articulation(
        "base_to_outer_post",
        ArticulationType.FIXED,
        parent=base,
        child=outer_post,
        origin=Origin(xyz=(-0.20, -0.02, 0.078)),
    )
    model.articulation(
        "post_slide",
        ArticulationType.PRISMATIC,
        parent=outer_post,
        child=inner_column,
        origin=Origin(xyz=(0.0, 0.0, 0.428)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.20, lower=0.0, upper=0.220),
    )
    model.articulation(
        "column_to_head",
        ArticulationType.FIXED,
        parent=inner_column,
        child=support_head,
        origin=Origin(xyz=(0.0, 0.0, 0.400)),
    )
    model.articulation(
        "tray_tilt",
        ArticulationType.REVOLUTE,
        parent=support_head,
        child=tray,
        origin=Origin(xyz=(0.160, -0.155, 0.082)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=-0.10, upper=0.55),
    )
    model.articulation(
        "clip_bar_tilt",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=clip_bar,
        origin=Origin(xyz=(0.190, 0.400, 0.028)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.5, lower=0.0, upper=1.10),
    )
    model.articulation(
        "lever_squeeze",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=lever,
        origin=Origin(xyz=(0.180, 0.342, 0.000)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=0.0, upper=0.28),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    outer_post = object_model.get_part("outer_post")
    inner_column = object_model.get_part("inner_column")
    support_head = object_model.get_part("support_head")
    tray = object_model.get_part("tray")
    clip_bar = object_model.get_part("clip_bar")
    lever = object_model.get_part("lever")

    post_slide = object_model.get_articulation("post_slide")
    tray_tilt = object_model.get_articulation("tray_tilt")
    clip_bar_tilt = object_model.get_articulation("clip_bar_tilt")
    lever_squeeze = object_model.get_articulation("lever_squeeze")

    ctx.expect_within(
        inner_column,
        outer_post,
        axes="xy",
        inner_elem="inner_tube",
        margin=0.0,
        name="inner column stays centered in the outer sleeve",
    )
    ctx.expect_overlap(
        inner_column,
        outer_post,
        axes="z",
        elem_a="inner_tube",
        min_overlap=0.290,
        name="collapsed column remains deeply inserted",
    )
    ctx.expect_gap(
        tray,
        support_head,
        axis="z",
        positive_elem="tray_panel",
        negative_elem="head_saddle",
        max_gap=0.001,
        max_penetration=0.0,
        name="tray rests on the support head saddle",
    )
    ctx.expect_gap(
        clip_bar,
        tray,
        axis="y",
        positive_elem="clip_rod",
        negative_elem="front_lip",
        min_gap=0.010,
        max_gap=0.030,
        name="clip bar sits just forward of the front tray rim",
    )
    ctx.expect_gap(
        tray,
        lever,
        axis="z",
        positive_elem="tray_panel",
        negative_elem="lever_handle",
        min_gap=0.008,
        max_gap=0.025,
        name="release lever hangs below the tray underside at rest",
    )

    slide_limits = post_slide.motion_limits
    if slide_limits is not None and slide_limits.upper is not None:
        inner_rest = ctx.part_element_world_aabb(inner_column, elem="inner_tube")
        with ctx.pose({post_slide: slide_limits.upper}):
            ctx.expect_within(
                inner_column,
                outer_post,
                axes="xy",
                inner_elem="inner_tube",
                margin=0.0,
                name="extended column stays centered in the sleeve",
            )
            ctx.expect_overlap(
                inner_column,
                outer_post,
                axes="z",
                elem_a="inner_tube",
                min_overlap=0.095,
                name="extended column keeps retained insertion",
            )
            inner_extended = ctx.part_element_world_aabb(inner_column, elem="inner_tube")
        ctx.check(
            "column raises the tray support",
            inner_rest is not None
            and inner_extended is not None
            and inner_extended[1][2] > inner_rest[1][2] + 0.18,
            details=f"rest={inner_rest}, extended={inner_extended}",
        )

    tray_limits = tray_tilt.motion_limits
    if tray_limits is not None and tray_limits.upper is not None:
        tray_rest = ctx.part_element_world_aabb(tray, elem="front_lip")
        with ctx.pose({tray_tilt: tray_limits.upper}):
            tray_raised = ctx.part_element_world_aabb(tray, elem="front_lip")
        ctx.check(
            "tray tilts upward from the rear hinge",
            tray_rest is not None
            and tray_raised is not None
            and tray_raised[1][2] > tray_rest[1][2] + 0.18,
            details=f"rest={tray_rest}, raised={tray_raised}",
        )

    clip_limits = clip_bar_tilt.motion_limits
    if clip_limits is not None and clip_limits.upper is not None:
        clip_rest = ctx.part_element_world_aabb(clip_bar, elem="clip_rod")
        with ctx.pose({clip_bar_tilt: clip_limits.upper}):
            clip_raised = ctx.part_element_world_aabb(clip_bar, elem="clip_rod")
        ctx.check(
            "clip bar rotates up from the tray edge",
            clip_rest is not None
            and clip_raised is not None
            and clip_raised[1][2] > clip_rest[1][2] + 0.012,
            details=f"rest={clip_rest}, raised={clip_raised}",
        )

    lever_limits = lever_squeeze.motion_limits
    if lever_limits is not None and lever_limits.upper is not None:
        lever_rest = ctx.part_element_world_aabb(lever, elem="lever_handle")
        with ctx.pose({lever_squeeze: lever_limits.upper}):
            ctx.expect_gap(
                tray,
                lever,
                axis="z",
                positive_elem="tray_panel",
                negative_elem="lever_handle",
                min_gap=0.0,
                max_gap=0.010,
                name="squeezed lever approaches the tray underside without crossing it",
            )
            lever_raised = ctx.part_element_world_aabb(lever, elem="lever_handle")
        ctx.check(
            "lever squeezes upward",
            lever_rest is not None
            and lever_raised is not None
            and lever_raised[1][2] > lever_rest[1][2] + 0.008,
            details=f"rest={lever_rest}, raised={lever_raised}",
        )

    return ctx.report()


object_model = build_object_model()
