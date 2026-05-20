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
    model = ArticulatedObject(name="compact_overbed_table")

    powder = Material("warm_white_powder_coat", rgba=(0.86, 0.84, 0.78, 1.0))
    dark_metal = Material("dark_grey_metal", rgba=(0.18, 0.19, 0.20, 1.0))
    rubber = Material("soft_black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    maple = Material("pale_maple_laminate", rgba=(0.92, 0.78, 0.55, 1.0))
    black = Material("matte_black_plastic", rgba=(0.04, 0.04, 0.04, 1.0))
    chrome = Material("brushed_chrome", rgba=(0.68, 0.70, 0.70, 1.0))

    # The root is the low, bed-clearance base plus the fixed outer height sleeve.
    # A narrow foot rail is deliberately offset rearward, while the column and
    # tray hinge remain on the table centerline.
    base = model.part("base")
    base.visual(
        Box((0.72, 0.12, 0.035)),
        origin=Origin(xyz=(0.0, -0.22, 0.090)),
        material=powder,
        name="offset_foot_rail",
    )
    base.visual(
        Box((0.13, 0.27, 0.035)),
        origin=Origin(xyz=(0.0, -0.095, 0.090)),
        material=powder,
        name="base_neck",
    )
    base.visual(
        Box((0.13, 0.13, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        material=powder,
        name="column_plinth",
    )

    # Four walls make the sleeve visibly hollow; the inner stage slides through
    # the clear center rather than intersecting a solid proxy.
    sleeve_height = 0.455
    sleeve_center_z = 0.115 + sleeve_height / 2.0
    base.visual(
        Box((0.074, 0.008, sleeve_height)),
        origin=Origin(xyz=(0.0, -0.034, sleeve_center_z)),
        material=powder,
        name="sleeve_front_wall",
    )
    base.visual(
        Box((0.074, 0.008, sleeve_height)),
        origin=Origin(xyz=(0.0, 0.034, sleeve_center_z)),
        material=powder,
        name="sleeve_rear_wall",
    )
    base.visual(
        Box((0.008, 0.060, sleeve_height)),
        origin=Origin(xyz=(-0.034, 0.0, sleeve_center_z)),
        material=powder,
        name="sleeve_side_wall_0",
    )
    base.visual(
        Box((0.008, 0.060, sleeve_height)),
        origin=Origin(xyz=(0.034, 0.0, sleeve_center_z)),
        material=powder,
        name="sleeve_side_wall_1",
    )
    base.visual(
        Box((0.044, 0.010, 0.160)),
        origin=Origin(xyz=(0.0, -0.025, 0.480)),
        material=dark_metal,
        name="front_guide_pad",
    )
    base.visual(
        Box((0.044, 0.010, 0.160)),
        origin=Origin(xyz=(0.0, 0.025, 0.480)),
        material=dark_metal,
        name="rear_guide_pad",
    )

    # Collars stiffen the rectangular tube while keeping the middle open.
    for z, prefix in ((0.120, "lower"), (0.565, "upper")):
        base.visual(
            Box((0.092, 0.010, 0.020)),
            origin=Origin(xyz=(0.0, -0.043, z)),
            material=dark_metal,
            name=f"{prefix}_sleeve_front_collar",
        )
        base.visual(
            Box((0.092, 0.010, 0.020)),
            origin=Origin(xyz=(0.0, 0.043, z)),
            material=dark_metal,
            name=f"{prefix}_sleeve_rear_collar",
        )
        base.visual(
            Box((0.010, 0.078, 0.020)),
            origin=Origin(xyz=(-0.043, 0.0, z)),
            material=dark_metal,
            name=f"{prefix}_sleeve_side_collar_0",
        )
        base.visual(
            Box((0.010, 0.078, 0.020)),
            origin=Origin(xyz=(0.043, 0.0, z)),
            material=dark_metal,
            name=f"{prefix}_sleeve_side_collar_1",
        )

    # Small yokes around each caster wheel attach directly to the foot rail.
    wheel_centers = (
        (-0.30, -0.265, 0.035),
        (0.30, -0.265, 0.035),
        (-0.30, -0.175, 0.035),
        (0.30, -0.175, 0.035),
    )
    for index, (x, y, z) in enumerate(wheel_centers):
        for side in (-1.0, 1.0):
            base.visual(
                Box((0.004, 0.040, 0.055)),
                origin=Origin(xyz=(x + side * 0.013, y, z + 0.028)),
                material=dark_metal,
                name=f"caster_fork_{index}_{0 if side < 0 else 1}",
            )
        base.visual(
            Box((0.042, 0.046, 0.008)),
            origin=Origin(xyz=(x, y, z + 0.057)),
            material=dark_metal,
            name=f"caster_crown_{index}",
        )

    # Rolling wheels are separate continuous joints on horizontal axles.
    for index, (x, y, z) in enumerate(wheel_centers):
        wheel = model.part(f"wheel_{index}")
        wheel.visual(
            Cylinder(radius=0.032, length=0.022),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rubber,
            name="rubber_tire",
        )
        wheel.visual(
            Cylinder(radius=0.014, length=0.026),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=chrome,
            name="wheel_hub",
        )
        model.articulation(
            f"wheel_axle_{index}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=wheel,
            origin=Origin(xyz=(x, y, z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=20.0),
        )

    # The sliding inner stage carries only a short centered head, not a hidden
    # offset support arm, so the tray is visibly balanced over the column.
    mast = model.part("mast")
    mast.visual(
        Box((0.040, 0.040, 0.620)),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=chrome,
        name="inner_column",
    )
    mast.visual(
        Box((0.105, 0.070, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.288)),
        material=dark_metal,
        name="centered_head_block",
    )
    mast.visual(
        Cylinder(radius=0.006, length=0.150),
        origin=Origin(xyz=(0.0, 0.0, 0.308), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="tray_hinge_pin",
    )

    model.articulation(
        "column_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 0.565)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.18, lower=0.0, upper=0.180),
    )

    top = model.part("top")
    top.visual(
        Box((0.620, 0.380, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=maple,
        name="tray_panel",
    )
    top.visual(
        Box((0.640, 0.018, 0.050)),
        origin=Origin(xyz=(0.0, -0.199, 0.050)),
        material=black,
        name="front_lip",
    )
    top.visual(
        Box((0.640, 0.010, 0.026)),
        origin=Origin(xyz=(0.0, 0.195, 0.036)),
        material=black,
        name="rear_edge_band",
    )
    top.visual(
        Box((0.012, 0.380, 0.022)),
        origin=Origin(xyz=(-0.316, 0.0, 0.036)),
        material=black,
        name="side_band_0",
    )
    top.visual(
        Box((0.012, 0.380, 0.022)),
        origin=Origin(xyz=(0.316, 0.0, 0.036)),
        material=black,
        name="side_band_1",
    )
    top.visual(
        Cylinder(radius=0.010, length=0.135),
        origin=Origin(xyz=(0.0, 0.0, 0.001), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="tilt_bushing",
    )
    top.visual(
        Box((0.034, 0.018, 0.012)),
        origin=Origin(xyz=(-0.220, 0.205, 0.043)),
        material=black,
        name="clip_saddle_0",
    )
    top.visual(
        Box((0.034, 0.018, 0.012)),
        origin=Origin(xyz=(0.220, 0.205, 0.043)),
        material=black,
        name="clip_saddle_1",
    )

    model.articulation(
        "top_tilt",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=top,
        origin=Origin(xyz=(0.0, 0.0, 0.308)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.75, lower=-0.35, upper=0.70),
    )

    clip_bar = model.part("clip_bar")
    clip_bar.visual(
        Cylinder(radius=0.006, length=0.520),
        origin=Origin(xyz=(0.0, 0.0, 0.003), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="hinge_rod",
    )
    clip_bar.visual(
        Box((0.020, 0.012, 0.070)),
        origin=Origin(xyz=(-0.220, 0.010, 0.038)),
        material=dark_metal,
        name="clip_support_0",
    )
    clip_bar.visual(
        Box((0.020, 0.012, 0.070)),
        origin=Origin(xyz=(0.220, 0.010, 0.038)),
        material=dark_metal,
        name="clip_support_1",
    )
    clip_bar.visual(
        Box((0.540, 0.020, 0.018)),
        origin=Origin(xyz=(0.0, 0.018, 0.080)),
        material=dark_metal,
        name="chart_bar",
    )

    model.articulation(
        "clip_hinge",
        ArticulationType.REVOLUTE,
        parent=top,
        child=clip_bar,
        origin=Origin(xyz=(0.0, 0.205, 0.052)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.2, lower=-0.15, upper=1.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    mast = object_model.get_part("mast")
    top = object_model.get_part("top")
    clip_bar = object_model.get_part("clip_bar")
    slide = object_model.get_articulation("column_slide")
    tilt = object_model.get_articulation("top_tilt")
    clip = object_model.get_articulation("clip_hinge")

    ctx.allow_overlap(
        mast,
        top,
        elem_a="tray_hinge_pin",
        elem_b="tilt_bushing",
        reason="The top hinge bushing is modeled as a solid proxy around the captured steel pin.",
    )

    ctx.check(
        "column slide is prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={slide.articulation_type}",
    )
    ctx.check(
        "top tilt is a horizontal hinge",
        tilt.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 3) for v in tilt.axis) == (1.0, 0.0, 0.0),
        details=f"type={tilt.articulation_type}, axis={tilt.axis}",
    )
    ctx.check(
        "clip bar has its own hinge",
        clip.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 3) for v in clip.axis) == (1.0, 0.0, 0.0),
        details=f"type={clip.articulation_type}, axis={clip.axis}",
    )
    for index in range(4):
        joint = object_model.get_articulation(f"wheel_axle_{index}")
        ctx.check(
            f"wheel {index} axle spins continuously",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"type={joint.articulation_type}",
        )

    with ctx.pose({slide: 0.0, tilt: 0.0, clip: 0.0}):
        ctx.expect_origin_distance(
            mast,
            top,
            axes="xy",
            max_dist=0.001,
            name="tray hinge is centered over column",
        )
        ctx.expect_overlap(
            top,
            mast,
            axes="xy",
            elem_a="tray_panel",
            elem_b="inner_column",
            min_overlap=0.030,
            name="tray footprint sits over column centerline",
        )
        ctx.expect_within(
            mast,
            top,
            axes="yz",
            inner_elem="tray_hinge_pin",
            outer_elem="tilt_bushing",
            margin=0.002,
            name="hinge pin is centered inside tilt bushing",
        )
        ctx.expect_overlap(
            mast,
            top,
            axes="x",
            elem_a="tray_hinge_pin",
            elem_b="tilt_bushing",
            min_overlap=0.120,
            name="tilt bushing captures the hinge pin length",
        )
        ctx.expect_gap(
            top,
            mast,
            axis="z",
            positive_elem="tray_panel",
            negative_elem="centered_head_block",
            min_gap=0.002,
            max_gap=0.030,
            name="support head sits just below tray underside",
        )
        ctx.expect_overlap(
            clip_bar,
            top,
            axes="x",
            elem_a="hinge_rod",
            elem_b="rear_edge_band",
            min_overlap=0.45,
            name="clip hinge runs along tray rear edge",
        )

    rest_pos = ctx.part_world_position(mast)
    with ctx.pose({slide: 0.180}):
        raised_pos = ctx.part_world_position(mast)
        ctx.expect_overlap(
            mast,
            base,
            axes="z",
            elem_a="inner_column",
            elem_b="sleeve_front_wall",
            min_overlap=0.120,
            name="raised column remains inserted in sleeve",
        )
    ctx.check(
        "column slide raises tray support",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.15,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    rest_clip = ctx.part_element_world_aabb(clip_bar, elem="chart_bar")
    with ctx.pose({clip: 0.90}):
        folded_clip = ctx.part_element_world_aabb(clip_bar, elem="chart_bar")
    ctx.check(
        "clip bar rotates away from upright edge",
        rest_clip is not None
        and folded_clip is not None
        and folded_clip[0][1] < rest_clip[0][1] - 0.020,
        details=f"rest={rest_clip}, folded={folded_clip}",
    )

    return ctx.report()


object_model = build_object_model()
