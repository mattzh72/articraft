from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pneumatic_lift_overbed_table")

    powder = model.material("warm_white_powder_coat", rgba=(0.88, 0.89, 0.86, 1.0))
    trim = model.material("soft_gray_trim", rgba=(0.48, 0.51, 0.52, 1.0))
    black = model.material("black_rubber", rgba=(0.02, 0.022, 0.02, 1.0))
    chrome = model.material("brushed_chrome", rgba=(0.72, 0.74, 0.72, 1.0))
    tray_mat = model.material("clinical_tray_laminate", rgba=(0.94, 0.95, 0.91, 1.0))
    blue = model.material("blue_release_paddle", rgba=(0.10, 0.34, 0.58, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.12, 0.68, 0.055)),
        origin=Origin(xyz=(-0.33, 0.0, 0.115)),
        material=powder,
        name="rear_crossbar",
    )
    for y, name in ((0.265, "runner_0"), (-0.265, "runner_1")):
        base.visual(
            Box((0.94, 0.085, 0.055)),
            origin=Origin(xyz=(0.12, y, 0.115)),
            material=powder,
            name=name,
        )
    base.visual(
        Box((0.17, 0.22, 0.030)),
        origin=Origin(xyz=(-0.33, 0.0, 0.1575)),
        material=powder,
        name="column_foot_plate",
    )

    caster_positions = [
        (-0.29, 0.265),
        (0.55, 0.265),
        (-0.29, -0.265),
        (0.55, -0.265),
    ]
    for idx, (x, y) in enumerate(caster_positions):
        base.visual(
            Box((0.026, 0.008, 0.055)),
            origin=Origin(xyz=(x, y - 0.028, 0.0625)),
            material=trim,
            name=f"caster_fork_{idx}_0",
        )
        base.visual(
            Box((0.026, 0.008, 0.055)),
            origin=Origin(xyz=(x, y + 0.028, 0.0625)),
            material=trim,
            name=f"caster_fork_{idx}_1",
        )
        base.visual(
            Cylinder(radius=0.006, length=0.070),
            origin=Origin(xyz=(x, y, 0.038), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=chrome,
            name=f"caster_axle_{idx}",
        )

    outer_sleeve = model.part("outer_sleeve")
    # Four separate walls leave a visible hollow sleeve so the sliding lift stack
    # reads as nested rather than a single solid post.
    outer_sleeve.visual(
        Box((0.012, 0.080, 0.540)),
        origin=Origin(xyz=(-0.0525, 0.0, 0.270)),
        material=powder,
        name="sleeve_wall_0",
    )
    outer_sleeve.visual(
        Box((0.012, 0.080, 0.540)),
        origin=Origin(xyz=(0.0525, 0.0, 0.270)),
        material=powder,
        name="sleeve_wall_1",
    )
    outer_sleeve.visual(
        Box((0.105, 0.012, 0.540)),
        origin=Origin(xyz=(0.0, -0.040, 0.270)),
        material=powder,
        name="sleeve_wall_2",
    )
    outer_sleeve.visual(
        Box((0.105, 0.012, 0.540)),
        origin=Origin(xyz=(0.0, 0.040, 0.270)),
        material=powder,
        name="sleeve_wall_3",
    )
    outer_sleeve.visual(
        Box((0.018, 0.100, 0.022)),
        origin=Origin(xyz=(-0.061, 0.0, 0.541)),
        material=trim,
        name="top_collar_0",
    )
    outer_sleeve.visual(
        Box((0.018, 0.100, 0.022)),
        origin=Origin(xyz=(0.061, 0.0, 0.541)),
        material=trim,
        name="top_collar_1",
    )
    outer_sleeve.visual(
        Box((0.125, 0.018, 0.022)),
        origin=Origin(xyz=(0.0, -0.049, 0.541)),
        material=trim,
        name="top_collar_2",
    )
    outer_sleeve.visual(
        Box((0.125, 0.018, 0.022)),
        origin=Origin(xyz=(0.0, 0.049, 0.541)),
        material=trim,
        name="top_collar_3",
    )

    lift_column = model.part("lift_column")
    lift_column.visual(
        Box((0.060, 0.038, 0.720)),
        origin=Origin(xyz=(0.0, 0.0, -0.140)),
        material=chrome,
        name="inner_mast",
    )
    lift_column.visual(
        Box((0.0165, 0.022, 0.055)),
        origin=Origin(xyz=(0.03825, 0.0, -0.100)),
        material=trim,
        name="guide_pad_0",
    )
    lift_column.visual(
        Box((0.0165, 0.022, 0.055)),
        origin=Origin(xyz=(-0.03825, 0.0, -0.100)),
        material=trim,
        name="guide_pad_1",
    )
    lift_column.visual(
        Box((0.028, 0.015, 0.055)),
        origin=Origin(xyz=(0.0, 0.0265, -0.100)),
        material=trim,
        name="guide_pad_2",
    )
    lift_column.visual(
        Box((0.028, 0.015, 0.055)),
        origin=Origin(xyz=(0.0, -0.0265, -0.100)),
        material=trim,
        name="guide_pad_3",
    )
    lift_column.visual(
        Cylinder(radius=0.0075, length=0.610),
        origin=Origin(xyz=(-0.037, 0.0, -0.185)),
        material=black,
        name="gas_spring",
    )
    lift_column.visual(
        Cylinder(radius=0.006, length=0.250),
        origin=Origin(xyz=(-0.037, 0.0, 0.125)),
        material=chrome,
        name="gas_rod",
    )
    lift_column.visual(
        Box((0.080, 0.060, 0.026)),
        origin=Origin(xyz=(-0.012, 0.0, 0.130)),
        material=trim,
        name="lower_gas_bracket",
    )
    lift_column.visual(
        Box((0.082, 0.140, 0.050)),
        origin=Origin(xyz=(-0.004, 0.0, 0.205)),
        material=trim,
        name="column_head",
    )
    lift_column.visual(
        Box((0.048, 0.012, 0.044)),
        origin=Origin(xyz=(0.060, -0.066, 0.215)),
        material=trim,
        name="tilt_yoke_0",
    )
    lift_column.visual(
        Box((0.048, 0.012, 0.044)),
        origin=Origin(xyz=(0.060, 0.066, 0.215)),
        material=trim,
        name="tilt_yoke_1",
    )
    lift_column.visual(
        Cylinder(radius=0.008, length=0.160),
        origin=Origin(xyz=(0.060, 0.0, 0.230), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="tilt_pin",
    )

    tray = model.part("tray")
    tray.visual(
        Cylinder(radius=0.022, length=0.118),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim,
        name="hinge_boss",
    )
    tray.visual(
        Box((0.090, 0.120, 0.014)),
        origin=Origin(xyz=(0.045, 0.0, 0.012)),
        material=trim,
        name="hinge_plate",
    )
    tray.visual(
        Box((0.760, 0.420, 0.028)),
        origin=Origin(xyz=(0.380, 0.0, 0.022)),
        material=tray_mat,
        name="tray_panel",
    )
    tray.visual(
        Box((0.790, 0.026, 0.034)),
        origin=Origin(xyz=(0.395, 0.223, 0.053)),
        material=tray_mat,
        name="rim_0",
    )
    tray.visual(
        Box((0.790, 0.026, 0.034)),
        origin=Origin(xyz=(0.395, -0.223, 0.053)),
        material=tray_mat,
        name="rim_1",
    )
    tray.visual(
        Box((0.026, 0.420, 0.034)),
        origin=Origin(xyz=(0.773, 0.0, 0.053)),
        material=tray_mat,
        name="front_rim",
    )
    tray.visual(
        Box((0.026, 0.420, 0.034)),
        origin=Origin(xyz=(0.000, 0.0, 0.053)),
        material=tray_mat,
        name="hinge_rim",
    )
    for y, name in ((-0.150, "clip_support_0"), (0.150, "clip_support_1")):
        tray.visual(
            Box((0.024, 0.018, 0.070)),
            origin=Origin(xyz=(0.760, y, 0.079)),
            material=trim,
            name=name,
        )
    tray.visual(
        Box((0.025, 0.052, 0.036)),
        origin=Origin(xyz=(0.430, -0.235, -0.010)),
        material=trim,
        name="paddle_bracket_0",
    )
    tray.visual(
        Box((0.025, 0.052, 0.036)),
        origin=Origin(xyz=(0.500, -0.235, -0.010)),
        material=trim,
        name="paddle_bracket_1",
    )

    release_paddle = model.part("release_paddle")
    release_paddle.visual(
        Cylinder(radius=0.006, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="pivot_pin",
    )
    release_paddle.visual(
        Box((0.150, 0.058, 0.012)),
        origin=Origin(xyz=(0.0, 0.040, -0.012)),
        material=blue,
        name="paddle_blade",
    )
    release_paddle.visual(
        Box((0.026, 0.032, 0.018)),
        origin=Origin(xyz=(0.0, 0.012, -0.006)),
        material=blue,
        name="paddle_hub",
    )

    clip_bar = model.part("clip_bar")
    clip_bar.visual(
        Cylinder(radius=0.008, length=0.360),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="bar_hinge",
    )
    clip_bar.visual(
        Box((0.055, 0.250, 0.012)),
        origin=Origin(xyz=(-0.026, 0.0, 0.014)),
        material=trim,
        name="clip_strip",
    )

    for idx, (x, y) in enumerate(caster_positions):
        caster = model.part(f"caster_{idx}")
        caster.visual(
            Cylinder(radius=0.038, length=0.032),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=black,
            name="tire",
        )
        caster.visual(
            Cylinder(radius=0.018, length=0.044),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=trim,
            name="hub",
        )
        model.articulation(
            f"base_to_caster_{idx}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=caster,
            origin=Origin(xyz=(x, y, 0.038)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=20.0),
        )

    model.articulation(
        "base_to_outer_sleeve",
        ArticulationType.FIXED,
        parent=base,
        child=outer_sleeve,
        origin=Origin(xyz=(-0.33, 0.0, 0.1725)),
    )
    model.articulation(
        "sleeve_to_lift_column",
        ArticulationType.PRISMATIC,
        parent=outer_sleeve,
        child=lift_column,
        origin=Origin(xyz=(0.0, 0.0, 0.520)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.18, lower=0.0, upper=0.240),
    )
    model.articulation(
        "lift_column_to_tray",
        ArticulationType.REVOLUTE,
        parent=lift_column,
        child=tray,
        origin=Origin(xyz=(0.060, 0.0, 0.230)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=-0.25, upper=0.65),
    )
    model.articulation(
        "tray_to_release_paddle",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=release_paddle,
        origin=Origin(xyz=(0.465, -0.235, -0.020)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.5, lower=0.0, upper=0.55),
    )
    model.articulation(
        "tray_to_clip_bar",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=clip_bar,
        origin=Origin(xyz=(0.760, 0.0, 0.098)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    outer_sleeve = object_model.get_part("outer_sleeve")
    lift_column = object_model.get_part("lift_column")
    tray = object_model.get_part("tray")
    release_paddle = object_model.get_part("release_paddle")
    clip_bar = object_model.get_part("clip_bar")

    lift = object_model.get_articulation("sleeve_to_lift_column")
    tilt = object_model.get_articulation("lift_column_to_tray")
    paddle = object_model.get_articulation("tray_to_release_paddle")
    clip = object_model.get_articulation("tray_to_clip_bar")

    for idx in range(4):
        caster = object_model.get_part(f"caster_{idx}")
        ctx.allow_overlap(
            base,
            caster,
            elem_a=f"caster_axle_{idx}",
            elem_b="hub",
            reason="The caster axle is intentionally captured through the wheel hub.",
        )
        ctx.expect_overlap(
            base,
            caster,
            axes="y",
            elem_a=f"caster_axle_{idx}",
            elem_b="hub",
            min_overlap=0.030,
            name=f"caster {idx} axle retained in hub",
        )
        ctx.allow_overlap(
            base,
            caster,
            elem_a=f"caster_axle_{idx}",
            elem_b="tire",
            reason="The small caster tire is a solid visual proxy; the real axle passes through its central bore.",
        )
        ctx.expect_overlap(
            base,
            caster,
            axes="y",
            elem_a=f"caster_axle_{idx}",
            elem_b="tire",
            min_overlap=0.025,
            name=f"caster {idx} axle passes through wheel body",
        )

    ctx.allow_overlap(
        lift_column,
        tray,
        elem_a="tilt_pin",
        elem_b="hinge_boss",
        reason="The tray hinge boss is intentionally sleeved around the column-head tilt pin.",
    )
    ctx.expect_overlap(
        lift_column,
        tray,
        axes="y",
        elem_a="tilt_pin",
        elem_b="hinge_boss",
        min_overlap=0.10,
        name="tray hinge boss surrounds tilt pin",
    )

    for support in ("clip_support_0", "clip_support_1"):
        ctx.allow_overlap(
            clip_bar,
            tray,
            elem_a="bar_hinge",
            elem_b=support,
            reason="The chart clip hinge rod is intentionally carried in the short support tab.",
        )
        ctx.expect_overlap(
            clip_bar,
            tray,
            axes="yz",
            elem_a="bar_hinge",
            elem_b=support,
            min_overlap=0.010,
            name=f"clip bar hinge captured by {support}",
        )

    for bracket in ("paddle_bracket_0", "paddle_bracket_1"):
        ctx.allow_overlap(
            release_paddle,
            tray,
            elem_a="pivot_pin",
            elem_b=bracket,
            reason="The release paddle pivot pin is intentionally captured in the underside bracket.",
        )
        ctx.expect_overlap(
            release_paddle,
            tray,
            axes="x",
            elem_a="pivot_pin",
            elem_b=bracket,
            min_overlap=0.020,
            name=f"release paddle pin captured by {bracket}",
        )

    ctx.expect_contact(
        outer_sleeve,
        base,
        elem_a="sleeve_wall_0",
        elem_b="column_foot_plate",
        contact_tol=0.002,
        name="outer sleeve sits on base plate",
    )
    ctx.expect_within(
        lift_column,
        outer_sleeve,
        axes="xy",
        inner_elem="inner_mast",
        margin=0.0,
        name="lift mast remains centered within sleeve footprint",
    )
    ctx.expect_overlap(
        lift_column,
        outer_sleeve,
        axes="z",
        elem_a="inner_mast",
        elem_b="sleeve_wall_0",
        min_overlap=0.30,
        name="collapsed lift column remains deeply nested",
    )

    rest_lift_pos = ctx.part_world_position(lift_column)
    rest_tray_aabb = ctx.part_world_aabb(tray)
    rest_paddle_aabb = ctx.part_world_aabb(release_paddle)
    rest_clip_aabb = ctx.part_world_aabb(clip_bar)
    with ctx.pose({lift: 0.240}):
        ctx.expect_overlap(
            lift_column,
            outer_sleeve,
            axes="z",
            elem_a="inner_mast",
            elem_b="sleeve_wall_0",
            min_overlap=0.20,
            name="raised lift column still retained in sleeve",
        )
        raised_lift_pos = ctx.part_world_position(lift_column)

    with ctx.pose({tilt: 0.55}):
        tilted_tray_aabb = ctx.part_world_aabb(tray)

    with ctx.pose({paddle: 0.45}):
        lowered_paddle_aabb = ctx.part_world_aabb(release_paddle)

    with ctx.pose({clip: 0.9}):
        raised_clip_aabb = ctx.part_world_aabb(clip_bar)

    ctx.check(
        "lift column moves upward on prismatic slide",
        rest_lift_pos is not None
        and raised_lift_pos is not None
        and raised_lift_pos[2] > rest_lift_pos[2] + 0.20,
        details=f"rest={rest_lift_pos}, raised={raised_lift_pos}",
    )
    ctx.check(
        "tray tilt hinge raises distal tray edge",
        rest_tray_aabb is not None
        and tilted_tray_aabb is not None
        and tilted_tray_aabb[1][2] > rest_tray_aabb[1][2] + 0.08,
        details=f"rest={rest_tray_aabb}, tilted={tilted_tray_aabb}",
    )
    ctx.check(
        "release paddle rotates downward",
        rest_paddle_aabb is not None
        and lowered_paddle_aabb is not None
        and lowered_paddle_aabb[0][2] < rest_paddle_aabb[0][2] - 0.015,
        details=f"rest={rest_paddle_aabb}, lowered={lowered_paddle_aabb}",
    )
    ctx.check(
        "chart clip bar flips upward from tray rim",
        rest_clip_aabb is not None
        and raised_clip_aabb is not None
        and raised_clip_aabb[1][2] > rest_clip_aabb[1][2] + 0.030,
        details=f"rest={rest_clip_aabb}, raised={raised_clip_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
