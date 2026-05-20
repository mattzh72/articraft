from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="bariatric_overbed_table")

    powder = model.material("powder_coated_steel", color=(0.72, 0.75, 0.76, 1.0))
    dark_steel = model.material("dark_steel", color=(0.22, 0.23, 0.24, 1.0))
    brushed = model.material("brushed_metal", color=(0.55, 0.57, 0.58, 1.0))
    rubber = model.material("black_rubber", color=(0.015, 0.014, 0.013, 1.0))
    laminate = model.material("warm_laminate", color=(0.86, 0.78, 0.58, 1.0))
    rim = model.material("grey_plastic_rim", color=(0.64, 0.66, 0.64, 1.0))
    paddle_mat = model.material("blue_release_paddle", color=(0.05, 0.25, 0.70, 1.0))
    label = model.material("caster_white_mark", color=(0.94, 0.94, 0.88, 1.0))
    slide_pad_mat = model.material("white_slide_pads", color=(0.88, 0.88, 0.82, 1.0))

    base = model.part("base")

    # Wide patient-room H-base: two long runners connected by a broad cross tube.
    for x, name in ((-0.47, "runner_0"), (0.47, "runner_1")):
        base.visual(
            Box((0.115, 1.08, 0.070)),
            origin=Origin(xyz=(x, 0.0, 0.125)),
            material=powder,
            name=name,
        )
    base.visual(
        Box((1.06, 0.120, 0.070)),
        origin=Origin(xyz=(0.0, -0.32, 0.125)),
        material=powder,
        name="cross_tube",
    )
    base.visual(
        Box((0.825, 0.080, 0.055)),
        origin=Origin(xyz=(0.0, 0.18, 0.130)),
        material=powder,
        name="front_tie",
    )

    # Twin rectangular outer lift columns, modeled as open sleeves rather than
    # solid blocks so the inner posts visibly slide inside them.
    column_centers = (-0.28, 0.28)
    base.visual(
        Box((0.010, 0.080, 0.540)),
        origin=Origin(xyz=(-0.325, -0.32, 0.425)),
        material=dark_steel,
        name="outer_column_0_side_0",
    )
    base.visual(
        Box((0.010, 0.080, 0.540)),
        origin=Origin(xyz=(-0.235, -0.32, 0.425)),
        material=dark_steel,
        name="outer_column_0_side_1",
    )
    base.visual(
        Box((0.100, 0.010, 0.540)),
        origin=Origin(xyz=(-0.28, -0.355, 0.425)),
        material=dark_steel,
        name="outer_column_0_back_wall",
    )
    base.visual(
        Box((0.100, 0.010, 0.540)),
        origin=Origin(xyz=(-0.28, -0.285, 0.425)),
        material=dark_steel,
        name="outer_column_0_front_wall",
    )
    base.visual(
        Box((0.125, 0.105, 0.026)),
        origin=Origin(xyz=(-0.28, -0.32, 0.170)),
        material=powder,
        name="column_foot_0",
    )
    base.visual(
        Box((0.010, 0.080, 0.540)),
        origin=Origin(xyz=(0.235, -0.32, 0.425)),
        material=dark_steel,
        name="outer_column_1_side_0",
    )
    base.visual(
        Box((0.010, 0.080, 0.540)),
        origin=Origin(xyz=(0.325, -0.32, 0.425)),
        material=dark_steel,
        name="outer_column_1_side_1",
    )
    base.visual(
        Box((0.100, 0.010, 0.540)),
        origin=Origin(xyz=(0.28, -0.355, 0.425)),
        material=dark_steel,
        name="outer_column_1_back_wall",
    )
    base.visual(
        Box((0.100, 0.010, 0.540)),
        origin=Origin(xyz=(0.28, -0.285, 0.425)),
        material=dark_steel,
        name="outer_column_1_front_wall",
    )
    base.visual(
        Box((0.125, 0.105, 0.026)),
        origin=Origin(xyz=(0.28, -0.32, 0.170)),
        material=powder,
        name="column_foot_1",
    )

    # Four caster forks are fixed to the base; only the wheels spin.
    caster_positions = (
        (-0.47, -0.46),
        (0.47, -0.46),
        (-0.47, 0.46),
        (0.47, 0.46),
    )
    for idx, (x, y) in enumerate(caster_positions):
        base.visual(
            Box((0.008, 0.058, 0.080)),
            origin=Origin(xyz=(x - 0.028, y, 0.060)),
            material=brushed,
            name=f"caster_fork_{idx}_side_0",
        )
        base.visual(
            Box((0.008, 0.058, 0.080)),
            origin=Origin(xyz=(x + 0.028, y, 0.060)),
            material=brushed,
            name=f"caster_fork_{idx}_side_1",
        )
        base.visual(
            Box((0.076, 0.062, 0.014)),
            origin=Origin(xyz=(x, y, 0.096)),
            material=brushed,
            name=f"caster_yoke_{idx}",
        )

    inner_posts = model.part("inner_posts")
    inner_posts.visual(
        Box((0.050, 0.036, 0.640)),
        origin=Origin(xyz=(-0.28, -0.32, 0.020)),
        material=brushed,
        name="post_0",
    )
    inner_posts.visual(
        Box((0.050, 0.036, 0.640)),
        origin=Origin(xyz=(0.28, -0.32, 0.020)),
        material=brushed,
        name="post_1",
    )
    for idx, x in enumerate(column_centers):
        inner_posts.visual(
            Box((0.080, 0.060, 0.045)),
            origin=Origin(xyz=(x, -0.32, -0.250)),
            material=slide_pad_mat,
            name=f"slide_pad_{idx}",
        )
    inner_posts.visual(
        Box((0.620, 0.042, 0.040)),
        origin=Origin(xyz=(0.0, -0.32, 0.315)),
        material=brushed,
        name="post_bridge",
    )
    inner_posts.visual(
        Box((0.560, 0.030, 0.030)),
        origin=Origin(xyz=(0.0, -0.32, 0.155)),
        material=brushed,
        name="lower_bridge",
    )

    model.articulation(
        "base_to_inner_posts",
        ArticulationType.PRISMATIC,
        parent=base,
        child=inner_posts,
        origin=Origin(xyz=(0.0, 0.0, 0.685)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=280.0, velocity=0.16, lower=0.0, upper=0.220),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.660, 0.070, 0.034)),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=dark_steel,
        name="carriage_crosshead",
    )
    for x in column_centers:
        carriage.visual(
            Box((0.095, 0.082, 0.045)),
            origin=Origin(xyz=(x, 0.0, 0.022)),
            material=dark_steel,
            name=f"post_clamp_{0 if x < 0 else 1}",
        )
    carriage.visual(
        Cylinder(radius=0.012, length=1.02),
        origin=Origin(xyz=(0.0, 0.0, 0.045), rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed,
        name="tray_hinge_pin",
    )
    for x in (-0.52, 0.52):
        carriage.visual(
            Box((0.035, 0.060, 0.060)),
            origin=Origin(xyz=(x, 0.0, 0.030)),
            material=dark_steel,
            name=f"hinge_cheek_{0 if x < 0 else 1}",
        )

    model.articulation(
        "inner_posts_to_carriage",
        ArticulationType.FIXED,
        parent=inner_posts,
        child=carriage,
        origin=Origin(xyz=(0.0, -0.32, 0.340)),
    )

    tray = model.part("tray")
    tray.visual(
        Box((1.200, 0.610, 0.035)),
        origin=Origin(xyz=(0.0, 0.315, 0.035)),
        material=laminate,
        name="tray_panel",
    )
    tray.visual(
        Box((1.255, 0.040, 0.055)),
        origin=Origin(xyz=(0.0, 0.010, 0.066)),
        material=rim,
        name="rear_rim",
    )
    tray.visual(
        Box((1.255, 0.045, 0.060)),
        origin=Origin(xyz=(0.0, 0.625, 0.070)),
        material=rim,
        name="front_rim",
    )
    tray.visual(
        Box((0.045, 0.640, 0.055)),
        origin=Origin(xyz=(-0.625, 0.320, 0.066)),
        material=rim,
        name="side_rim_0",
    )
    tray.visual(
        Box((0.045, 0.640, 0.055)),
        origin=Origin(xyz=(0.625, 0.320, 0.066)),
        material=rim,
        name="side_rim_1",
    )
    tray.visual(
        Box((1.060, 0.490, 0.006)),
        origin=Origin(xyz=(0.0, 0.340, 0.0555)),
        material=model.material("slightly_glossy_top", color=(0.92, 0.86, 0.66, 1.0)),
        name="tray_inset",
    )
    tray.visual(
        Box((1.000, 0.030, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_steel,
        name="tray_hinge_leaf",
    )

    # Chart clip supports and hinge are distinct raised hardware at the front rim.
    tray.visual(
        Box((0.040, 0.030, 0.080)),
        origin=Origin(xyz=(-0.42, 0.625, 0.095)),
        material=brushed,
        name="clip_support_0",
    )
    tray.visual(
        Box((0.040, 0.030, 0.080)),
        origin=Origin(xyz=(0.42, 0.625, 0.095)),
        material=brushed,
        name="clip_support_1",
    )
    tray.visual(
        Cylinder(radius=0.006, length=0.900),
        origin=Origin(xyz=(0.0, 0.625, 0.135), rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed,
        name="clip_hinge_pin",
    )

    # Underside release-paddle hinge mounts, kept below the tabletop.
    for idx, x in enumerate((0.275, 0.565)):
        tray.visual(
            Box((0.025, 0.070, 0.032)),
            origin=Origin(xyz=(x, 0.220, 0.002)),
            material=dark_steel,
            name=f"paddle_bracket_{idx}",
        )

    model.articulation(
        "carriage_to_tray",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=65.0, velocity=0.55, lower=-0.15, upper=0.55),
    )

    release_paddle = model.part("release_paddle")
    release_paddle.visual(
        Cylinder(radius=0.006, length=0.265),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed,
        name="paddle_pivot",
    )
    release_paddle.visual(
        Box((0.220, 0.120, 0.014)),
        origin=Origin(xyz=(0.0, 0.070, -0.046)),
        material=paddle_mat,
        name="paddle_blade",
    )
    for x in (-0.070, 0.070):
        release_paddle.visual(
            Box((0.018, 0.070, 0.050)),
            origin=Origin(xyz=(x, 0.033, -0.024)),
            material=paddle_mat,
            name=f"paddle_arm_{0 if x < 0 else 1}",
        )
    release_paddle.visual(
        Box((0.160, 0.025, 0.018)),
        origin=Origin(xyz=(0.0, 0.120, -0.040)),
        material=paddle_mat,
        name="finger_lip",
    )
    model.articulation(
        "tray_to_release_paddle",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=release_paddle,
        origin=Origin(xyz=(0.420, 0.220, -0.010)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-0.55, upper=0.25),
    )

    clip_bar = model.part("clip_bar")
    clip_bar.visual(
        Cylinder(radius=0.008, length=0.780),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed,
        name="clip_hinge_sleeve",
    )
    clip_bar.visual(
        Box((0.780, 0.020, 0.016)),
        origin=Origin(xyz=(0.0, -0.015, 0.009)),
        material=brushed,
        name="clip_web",
    )
    clip_bar.visual(
        Box((0.900, 0.032, 0.014)),
        origin=Origin(xyz=(0.0, -0.035, 0.022)),
        material=brushed,
        name="clip_bar_blade",
    )
    clip_bar.visual(
        Box((0.860, 0.012, 0.020)),
        origin=Origin(xyz=(0.0, -0.054, 0.010)),
        material=brushed,
        name="paper_grip_lip",
    )
    model.articulation(
        "tray_to_clip_bar",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=clip_bar,
        origin=Origin(xyz=(0.0, 0.625, 0.135)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.5, lower=0.0, upper=1.10),
    )

    # Caster wheel parts. Cylinders are aligned with local X so each continuous
    # joint spins about the visible axle line through the fork.
    for idx, (x, y) in enumerate(caster_positions):
        caster = model.part(f"caster_{idx}")
        caster.visual(
            Cylinder(radius=0.040, length=0.032),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=rubber,
            name="tire",
        )
        caster.visual(
            Cylinder(radius=0.021, length=0.038),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=brushed,
            name="hub",
        )
        caster.visual(
            Cylinder(radius=0.006, length=0.056),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=brushed,
            name="axle",
        )
        caster.visual(
            Box((0.002, 0.010, 0.018)),
            origin=Origin(xyz=(0.020, 0.0, 0.030)),
            material=label,
            name="spin_mark",
        )
        model.articulation(
            f"base_to_caster_{idx}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=caster,
            origin=Origin(xyz=(x, y, 0.045)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=18.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    inner_posts = object_model.get_part("inner_posts")
    tray = object_model.get_part("tray")
    release_paddle = object_model.get_part("release_paddle")
    clip_bar = object_model.get_part("clip_bar")

    lift = object_model.get_articulation("base_to_inner_posts")
    tray_tilt = object_model.get_articulation("carriage_to_tray")
    clip_hinge = object_model.get_articulation("tray_to_clip_bar")

    ctx.allow_overlap(
        tray,
        clip_bar,
        elem_a="clip_hinge_pin",
        elem_b="clip_hinge_sleeve",
        reason="The chart clip sleeve intentionally wraps the narrow hinge pin along the tray rim.",
    )
    ctx.expect_overlap(
        tray,
        clip_bar,
        axes="xyz",
        elem_a="clip_hinge_pin",
        elem_b="clip_hinge_sleeve",
        min_overlap=0.010,
        name="clip sleeve is retained around rim hinge pin",
    )

    caster_joints = [object_model.get_articulation(f"base_to_caster_{i}") for i in range(4)]
    ctx.check(
        "all four casters spin continuously",
        all(j.articulation_type == ArticulationType.CONTINUOUS for j in caster_joints),
        details=str([j.articulation_type for j in caster_joints]),
    )

    ctx.check(
        "lift is prismatic",
        lift.articulation_type == ArticulationType.PRISMATIC,
        details=str(lift.articulation_type),
    )

    # The matched lift posts are retained inside the hollow outer sleeves.
    ctx.expect_gap(
        base,
        inner_posts,
        axis="y",
        positive_elem="outer_column_0_front_wall",
        negative_elem="post_0",
        min_gap=0.006,
        max_gap=0.030,
        name="post 0 clears front sleeve wall",
    )
    ctx.expect_gap(
        base,
        inner_posts,
        axis="y",
        positive_elem="outer_column_1_front_wall",
        negative_elem="post_1",
        min_gap=0.006,
        max_gap=0.030,
        name="post 1 clears front sleeve wall",
    )
    ctx.expect_overlap(
        inner_posts,
        base,
        axes="z",
        elem_a="post_0",
        elem_b="outer_column_0_front_wall",
        min_overlap=0.25,
        name="post 0 remains inserted in sleeve",
    )
    ctx.expect_overlap(
        inner_posts,
        base,
        axes="z",
        elem_a="post_1",
        elem_b="outer_column_1_front_wall",
        min_overlap=0.25,
        name="post 1 remains inserted in sleeve",
    )

    rest_inner = ctx.part_world_position(inner_posts)
    with ctx.pose({lift: 0.20}):
        raised_inner = ctx.part_world_position(inner_posts)
        ctx.expect_overlap(
            inner_posts,
            base,
            axes="z",
            elem_a="post_0",
            elem_b="outer_column_0_front_wall",
            min_overlap=0.08,
            name="raised post 0 keeps retained insertion",
        )
        ctx.expect_overlap(
            inner_posts,
            base,
            axes="z",
            elem_a="post_1",
            elem_b="outer_column_1_front_wall",
            min_overlap=0.08,
            name="raised post 1 keeps retained insertion",
        )
    ctx.check(
        "lift travel raises both posts",
        rest_inner is not None and raised_inner is not None and raised_inner[2] > rest_inner[2] + 0.18,
        details=f"rest={rest_inner}, raised={raised_inner}",
    )

    tray_rest = ctx.part_element_world_aabb(tray, elem="tray_panel")
    with ctx.pose({tray_tilt: 0.45}):
        tray_raised = ctx.part_element_world_aabb(tray, elem="tray_panel")
    ctx.check(
        "tray tilts upward on transverse hinge",
        tray_rest is not None
        and tray_raised is not None
        and tray_raised[1][2] > tray_rest[1][2] + 0.12,
        details=f"rest={tray_rest}, tilted={tray_raised}",
    )

    ctx.expect_gap(
        tray,
        release_paddle,
        axis="z",
        positive_elem="tray_panel",
        negative_elem="paddle_blade",
        min_gap=0.020,
        name="release paddle is below the work surface",
    )

    ctx.expect_overlap(
        tray,
        clip_bar,
        axes="x",
        elem_a="clip_support_0",
        elem_b="clip_bar_blade",
        min_overlap=0.025,
        name="clip bar spans support 0",
    )
    ctx.expect_overlap(
        tray,
        clip_bar,
        axes="x",
        elem_a="clip_support_1",
        elem_b="clip_bar_blade",
        min_overlap=0.025,
        name="clip bar spans support 1",
    )

    clip_rest = ctx.part_element_world_aabb(clip_bar, elem="clip_bar_blade")
    with ctx.pose({clip_hinge: 0.80}):
        clip_lifted = ctx.part_element_world_aabb(clip_bar, elem="clip_bar_blade")
    ctx.check(
        "clip bar rotates upward on rim hinge",
        clip_rest is not None
        and clip_lifted is not None
        and clip_lifted[1][2] > clip_rest[1][2] + 0.015,
        details=f"rest={clip_rest}, lifted={clip_lifted}",
    )

    return ctx.report()


object_model = build_object_model()
