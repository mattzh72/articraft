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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="overbed_table")

    steel = model.material("steel", rgba=(0.76, 0.78, 0.80, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.26, 0.28, 0.31, 1.0))
    tray_finish = model.material("tray_finish", rgba=(0.72, 0.74, 0.77, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.10, 0.10, 0.11, 1.0))
    clip_finish = model.material("clip_finish", rgba=(0.60, 0.62, 0.66, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.60, 0.11, 0.03)),
        origin=Origin(xyz=(-0.02, 0.0, 0.075)),
        material=dark_steel,
        name="base_beam",
    )
    base.visual(
        Box((0.10, 0.08, 0.05)),
        origin=Origin(xyz=(0.18, 0.0, 0.105)),
        material=dark_steel,
        name="column_mount",
    )
    base.visual(
        Box((0.006, 0.046, 0.43)),
        origin=Origin(xyz=(0.155, 0.0, 0.335)),
        material=steel,
        name="sleeve_side_neg_x",
    )
    base.visual(
        Box((0.006, 0.046, 0.43)),
        origin=Origin(xyz=(0.205, 0.0, 0.335)),
        material=steel,
        name="sleeve_side_pos_x",
    )
    base.visual(
        Box((0.044, 0.006, 0.43)),
        origin=Origin(xyz=(0.18, -0.020, 0.335)),
        material=steel,
        name="sleeve_side_neg_y",
    )
    base.visual(
        Box((0.044, 0.006, 0.43)),
        origin=Origin(xyz=(0.18, 0.020, 0.335)),
        material=steel,
        name="sleeve_side_pos_y",
    )
    base.visual(
        Box((0.008, 0.060, 0.020)),
        origin=Origin(xyz=(0.154, 0.0, 0.545)),
        material=dark_steel,
        name="sleeve_collar_neg_x",
    )
    base.visual(
        Box((0.008, 0.060, 0.020)),
        origin=Origin(xyz=(0.206, 0.0, 0.545)),
        material=dark_steel,
        name="sleeve_collar_pos_x",
    )
    base.visual(
        Box((0.044, 0.008, 0.020)),
        origin=Origin(xyz=(0.18, -0.026, 0.545)),
        material=dark_steel,
        name="sleeve_collar_neg_y",
    )
    base.visual(
        Box((0.044, 0.008, 0.020)),
        origin=Origin(xyz=(0.18, 0.026, 0.545)),
        material=dark_steel,
        name="sleeve_collar_pos_y",
    )

    caster_locations = (
        ("caster_0", -0.27, -0.040),
        ("caster_1", -0.27, 0.040),
        ("caster_2", 0.23, -0.040),
        ("caster_3", 0.23, 0.040),
    )
    for index, (_, cx, cy) in enumerate(caster_locations):
        base.visual(
            Cylinder(radius=0.006, length=0.016),
            origin=Origin(xyz=(cx, cy, 0.052)),
            material=steel,
            name=f"caster_stem_{index}",
        )
        base.visual(
            Box((0.024, 0.024, 0.008)),
            origin=Origin(xyz=(cx, cy, 0.044)),
            material=steel,
            name=f"caster_crown_{index}",
        )
        for side, leg_y in enumerate((cy - 0.011, cy + 0.011)):
            base.visual(
                Box((0.006, 0.004, 0.024)),
                origin=Origin(xyz=(cx, leg_y, 0.028)),
                material=steel,
                name=f"caster_leg_{index}_{side}",
            )

    inner_column = model.part("inner_column")
    inner_column.visual(
        Box((0.036, 0.028, 0.70)),
        origin=Origin(xyz=(0.0, 0.0, -0.03)),
        material=steel,
        name="column_member",
    )
    for pad_x in (-0.020, 0.020):
        inner_column.visual(
            Box((0.004, 0.010, 0.140)),
            origin=Origin(xyz=(pad_x, 0.0, -0.170)),
            material=dark_steel,
            name=f"guide_pad_{'neg_x' if pad_x < 0.0 else 'pos_x'}",
        )
    inner_column.visual(
        Box((0.050, 0.036, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.330)),
        material=dark_steel,
        name="head_post",
    )
    inner_column.visual(
        Box((0.120, 0.062, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.348)),
        material=dark_steel,
        name="head_cap",
    )
    for cheek_x in (-0.028, 0.028):
        inner_column.visual(
            Box((0.014, 0.054, 0.020)),
            origin=Origin(xyz=(cheek_x, 0.0, 0.345)),
            material=dark_steel,
            name=f"head_cheek_{'rear' if cheek_x < 0.0 else 'front'}",
        )

    tray = model.part("tray")
    tray.visual(
        Box((0.62, 0.38, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=tray_finish,
        name="tray_panel",
    )
    tray.visual(
        Box((0.010, 0.340, 0.014)),
        origin=Origin(xyz=(0.305, 0.0, 0.025)),
        material=dark_steel,
        name="front_lip",
    )

    clip_bar = model.part("clip_bar")
    for side, support_y in enumerate((-0.095, 0.095)):
        clip_bar.visual(
            Cylinder(radius=0.004, length=0.016),
            origin=Origin(xyz=(0.0, support_y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=clip_finish,
            name=f"clip_hinge_{side}",
        )
        clip_bar.visual(
            Box((0.040, 0.012, 0.006)),
            origin=Origin(xyz=(-0.020, support_y, 0.003)),
            material=clip_finish,
            name=f"clip_arm_{side}",
        )
        clip_bar.visual(
            Box((0.008, 0.012, 0.022)),
            origin=Origin(xyz=(-0.040, support_y, 0.017)),
            material=clip_finish,
            name=f"clip_support_{side}",
        )
    clip_bar.visual(
        Cylinder(radius=0.006, length=0.230),
        origin=Origin(xyz=(-0.040, 0.0, 0.028), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=clip_finish,
        name="clip_bar",
    )

    for caster_name, _, _ in caster_locations:
        caster = model.part(caster_name)
        caster.visual(
            Cylinder(radius=0.020, length=0.014),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=wheel_rubber,
            name="wheel_tire",
        )
        caster.visual(
            Cylinder(radius=0.014, length=0.016),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name="wheel_core",
        )

    model.articulation(
        "base_to_inner_column",
        ArticulationType.PRISMATIC,
        parent=base,
        child=inner_column,
        origin=Origin(xyz=(0.18, 0.0, 0.550)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.18,
            lower=0.0,
            upper=0.20,
        ),
    )
    model.articulation(
        "inner_column_to_tray",
        ArticulationType.REVOLUTE,
        parent=inner_column,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, 0.355)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(52.0),
        ),
    )
    model.articulation(
        "tray_to_clip_bar",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=clip_bar,
        origin=Origin(xyz=(0.305, 0.0, 0.032)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(58.0),
        ),
    )

    for caster_name, caster_x, caster_y in caster_locations:
        caster = model.get_part(caster_name)
        model.articulation(
            f"base_to_{caster_name}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=caster,
            origin=Origin(xyz=(caster_x, caster_y, 0.020)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=18.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    inner_column = object_model.get_part("inner_column")
    tray = object_model.get_part("tray")
    clip_bar = object_model.get_part("clip_bar")

    column_slide = object_model.get_articulation("base_to_inner_column")
    tray_tilt = object_model.get_articulation("inner_column_to_tray")
    clip_hinge = object_model.get_articulation("tray_to_clip_bar")

    ctx.expect_overlap(
        inner_column,
        base,
        axes="z",
        elem_a="column_member",
        elem_b="sleeve_side_pos_x",
        min_overlap=0.37,
        name="column remains deeply inserted at rest",
    )
    ctx.expect_gap(
        base,
        inner_column,
        axis="x",
        positive_elem="sleeve_side_pos_x",
        negative_elem="column_member",
        min_gap=0.003,
        max_gap=0.006,
        name="column clears the positive x sleeve wall",
    )
    ctx.expect_gap(
        inner_column,
        base,
        axis="x",
        positive_elem="column_member",
        negative_elem="sleeve_side_neg_x",
        min_gap=0.003,
        max_gap=0.006,
        name="column clears the negative x sleeve wall",
    )
    ctx.expect_gap(
        base,
        inner_column,
        axis="y",
        positive_elem="sleeve_side_pos_y",
        negative_elem="column_member",
        min_gap=0.002,
        max_gap=0.004,
        name="column clears the positive y sleeve wall",
    )
    ctx.expect_gap(
        inner_column,
        base,
        axis="y",
        positive_elem="column_member",
        negative_elem="sleeve_side_neg_y",
        min_gap=0.002,
        max_gap=0.004,
        name="column clears the negative y sleeve wall",
    )
    ctx.expect_origin_distance(
        tray,
        inner_column,
        axes="xy",
        max_dist=0.001,
        name="tray stays centered over the column",
    )
    ctx.expect_gap(
        tray,
        inner_column,
        axis="z",
        positive_elem="tray_panel",
        negative_elem="head_cap",
        max_gap=0.002,
        max_penetration=0.0,
        name="tray panel seats on the centered support head",
    )
    ctx.expect_overlap(
        clip_bar,
        tray,
        axes="y",
        elem_a="clip_bar",
        elem_b="front_lip",
        min_overlap=0.20,
        name="clip bar spans the tray edge",
    )

    slide_limits = column_slide.motion_limits
    if slide_limits is not None and slide_limits.upper is not None:
        rest_tray_pos = ctx.part_world_position(tray)
        with ctx.pose({column_slide: slide_limits.upper}):
            ctx.expect_overlap(
                inner_column,
                base,
                axes="z",
                elem_a="column_member",
                elem_b="sleeve_side_pos_x",
                min_overlap=0.17,
                name="extended column retains insertion",
            )
            raised_tray_pos = ctx.part_world_position(tray)
        ctx.check(
            "column raises the tray",
            rest_tray_pos is not None
            and raised_tray_pos is not None
            and raised_tray_pos[2] > rest_tray_pos[2] + 0.15,
            details=f"rest={rest_tray_pos}, raised={raised_tray_pos}",
        )

    tilt_limits = tray_tilt.motion_limits
    if tilt_limits is not None and tilt_limits.upper is not None:
        rest_front = ctx.part_element_world_aabb(tray, elem="front_lip")
        with ctx.pose({tray_tilt: tilt_limits.upper}):
            tilted_front = ctx.part_element_world_aabb(tray, elem="front_lip")
        ctx.check(
            "tray front edge tilts upward",
            rest_front is not None
            and tilted_front is not None
            and tilted_front[0][2] > rest_front[0][2] + 0.18,
            details=f"rest={rest_front}, tilted={tilted_front}",
        )

    clip_limits = clip_hinge.motion_limits
    if clip_limits is not None and clip_limits.upper is not None:
        rest_clip = ctx.part_element_world_aabb(clip_bar, elem="clip_bar")
        with ctx.pose({clip_hinge: clip_limits.upper}):
            opened_clip = ctx.part_element_world_aabb(clip_bar, elem="clip_bar")
        ctx.check(
            "clip bar rotates upward from the tray rim",
            rest_clip is not None
            and opened_clip is not None
            and opened_clip[0][2] > rest_clip[0][2] + 0.015,
            details=f"rest={rest_clip}, opened={opened_clip}",
        )

    return ctx.report()


object_model = build_object_model()
