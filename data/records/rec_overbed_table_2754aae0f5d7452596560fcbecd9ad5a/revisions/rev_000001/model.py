from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _square_tube(outer: float, inner: float, height: float, *, fillet: float = 0.0):
    tube = (
        cq.Workplane("XY")
        .box(outer, outer, height)
        .faces(">Z")
        .workplane()
        .rect(inner, inner)
        .cutBlind(-(height + 0.01))
    )
    if fillet > 0.0:
        tube = tube.edges("|Z").fillet(fillet)
    return tube


def _rounded_panel(length: float, width: float, thickness: float, *, corner_radius: float):
    return (
        cq.Workplane("XY")
        .box(length, width, thickness)
        .edges("|Z")
        .fillet(corner_radius)
    )


def _add_caster_wheel(part, *, tire_radius: float, tire_width: float, rubber: str, metal: str) -> None:
    spin_origin = Origin(rpy=(-pi / 2.0, 0.0, 0.0))
    part.visual(
        Cylinder(radius=tire_radius, length=tire_width),
        origin=spin_origin,
        material=rubber,
        name="tire",
    )
    part.visual(
        Cylinder(radius=tire_radius * 0.58, length=tire_width * 0.70),
        origin=spin_origin,
        material=metal,
        name="hub",
    )
    part.visual(
        Cylinder(radius=tire_radius * 0.18, length=tire_width * 1.35),
        origin=spin_origin,
        material=metal,
        name="axle_sleeve",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="overbed_table")

    model.material("powder_steel", rgba=(0.78, 0.80, 0.82, 1.0))
    model.material("chrome", rgba=(0.86, 0.88, 0.90, 1.0))
    model.material("dark_rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    model.material("laminate", rgba=(0.77, 0.67, 0.53, 1.0))
    model.material("knob_plastic", rgba=(0.16, 0.17, 0.18, 1.0))
    model.material("glide_nylon", rgba=(0.18, 0.18, 0.17, 1.0))
    model.material("clip_steel", rgba=(0.72, 0.74, 0.76, 1.0))

    rail_len = 0.66
    rail_w = 0.05
    rail_h = 0.03
    rail_x = 0.04
    rail_y = 0.15
    rail_bottom_z = 0.064
    rail_center_z = rail_bottom_z + (rail_h / 2.0)
    rail_top_z = rail_bottom_z + rail_h

    sleeve_x = -0.20
    sleeve_y = -0.15
    sleeve_h = 0.42
    sleeve_top_z = rail_top_z + sleeve_h
    collar_z = rail_top_z + sleeve_h - 0.055

    caster_x = 0.32
    caster_z = 0.031

    base = model.part("base")
    base.visual(
        Box((rail_len, rail_w, rail_h)),
        origin=Origin(xyz=(rail_x, -rail_y, rail_center_z)),
        material="powder_steel",
        name="rail_0",
    )
    base.visual(
        Box((rail_len, rail_w, rail_h)),
        origin=Origin(xyz=(rail_x, rail_y, rail_center_z)),
        material="powder_steel",
        name="rail_1",
    )
    base.visual(
        Box((0.09, (2.0 * rail_y) + rail_w, rail_h)),
        origin=Origin(xyz=(-0.25, 0.0, rail_center_z)),
        material="powder_steel",
        name="rear_tie",
    )
    base.visual(
        Box((0.11, 0.09, 0.05)),
        origin=Origin(xyz=(sleeve_x, sleeve_y, rail_top_z + 0.025)),
        material="powder_steel",
        name="column_shoe",
    )
    base.visual(
        mesh_from_cadquery(
            _square_tube(0.060, 0.044, sleeve_h, fillet=0.003),
            "outer_sleeve",
        ),
        origin=Origin(xyz=(sleeve_x, sleeve_y, rail_top_z + (sleeve_h / 2.0))),
        material="powder_steel",
        name="outer_sleeve",
    )
    base.visual(
        mesh_from_cadquery(
            _square_tube(0.082, 0.050, 0.072, fillet=0.004),
            "height_collar",
        ),
        origin=Origin(xyz=(sleeve_x, sleeve_y, collar_z)),
        material="powder_steel",
        name="height_collar",
    )
    base.visual(
        Cylinder(radius=0.008, length=0.024),
        origin=Origin(
            xyz=(sleeve_x, sleeve_y - 0.053, collar_z),
            rpy=(-pi / 2.0, 0.0, 0.0),
        ),
        material="chrome",
        name="knob_hub",
    )
    for index, y_pos in enumerate((-rail_y, rail_y)):
        base.visual(
            Box((0.018, 0.018, 0.050)),
            origin=Origin(xyz=(caster_x, y_pos, 0.085)),
            material="powder_steel",
            name=f"caster_stem_{index}",
        )
        base.visual(
            Box((0.020, 0.038, 0.010)),
            origin=Origin(xyz=(caster_x, y_pos, 0.074)),
            material="powder_steel",
            name=f"caster_bridge_{index}",
        )
        base.visual(
            Box((0.012, 0.004, 0.052)),
            origin=Origin(xyz=(caster_x, y_pos - 0.015, 0.045)),
            material="powder_steel",
            name=f"fork_0_{index}",
        )
        base.visual(
            Box((0.012, 0.004, 0.052)),
            origin=Origin(xyz=(caster_x, y_pos + 0.015, 0.045)),
            material="powder_steel",
            name=f"fork_1_{index}",
        )
        base.visual(
            Box((0.050, 0.070, rail_bottom_z)),
            origin=Origin(xyz=(-0.28, y_pos, rail_bottom_z / 2.0)),
            material="glide_nylon",
            name=f"glide_{index}",
        )

    post = model.part("post")
    post.visual(
        Box((0.036, 0.036, 0.84)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material="chrome",
        name="post",
    )
    post.visual(
        Box((0.34, 0.05, 0.028)),
        origin=Origin(xyz=(0.17, 0.0, 0.39)),
        material="powder_steel",
        name="arm",
    )
    post.visual(
        Box((0.080, 0.38, 0.018)),
        origin=Origin(xyz=(0.30, 0.17, 0.40)),
        material="powder_steel",
        name="tray_support",
    )
    post.visual(
        Box((0.10, 0.016, 0.18)),
        origin=Origin(xyz=(0.05, 0.0, 0.31)),
        material="powder_steel",
        name="gusset",
    )

    tray = model.part("tray")
    tray.visual(
        mesh_from_cadquery(
            _rounded_panel(0.42, 0.78, 0.024, corner_radius=0.020),
            "tray_top",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material="laminate",
        name="tray_top",
    )
    tray.visual(
        Box((0.018, 0.72, 0.030)),
        origin=Origin(xyz=(0.201, 0.0, -0.015)),
        material="powder_steel",
        name="front_rim",
    )
    tray.visual(
        Box((0.30, 0.018, 0.030)),
        origin=Origin(xyz=(0.0, -0.381, -0.015)),
        material="powder_steel",
        name="side_rim_0",
    )
    tray.visual(
        Box((0.30, 0.018, 0.030)),
        origin=Origin(xyz=(0.0, 0.381, -0.015)),
        material="powder_steel",
        name="side_rim_1",
    )
    for index, y_pos in enumerate((-0.22, 0.22)):
        tray.visual(
            Box((0.006, 0.014, 0.016)),
            origin=Origin(xyz=(0.208, y_pos, 0.014)),
            material="powder_steel",
            name=f"clip_pad_{index}",
        )

    clip_bar = model.part("clip_bar")
    clip_bar.visual(
        Cylinder(radius=0.006, length=0.70),
        origin=Origin(xyz=(0.070, 0.0, 0.020), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="clip_steel",
        name="bar",
    )
    for index, y_pos in enumerate((-0.22, 0.22)):
        clip_bar.visual(
            Box((0.066, 0.012, 0.020)),
            origin=Origin(xyz=(0.039, y_pos, 0.010)),
            material="clip_steel",
            name=f"support_{index}",
        )

    collar_knob = model.part("collar_knob")
    collar_knob.visual(
        Cylinder(radius=0.008, length=0.010),
        origin=Origin(xyz=(0.0, -0.005, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="chrome",
        name="stem",
    )
    collar_knob.visual(
        Cylinder(radius=0.020, length=0.016),
        origin=Origin(xyz=(0.0, -0.017, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="knob_plastic",
        name="grip",
    )
    collar_knob.visual(
        Box((0.012, 0.014, 0.006)),
        origin=Origin(xyz=(0.018, -0.017, 0.0)),
        material="knob_plastic",
        name="lobe_0",
    )
    collar_knob.visual(
        Box((0.012, 0.014, 0.006)),
        origin=Origin(xyz=(-0.018, -0.017, 0.0)),
        material="knob_plastic",
        name="lobe_1",
    )
    collar_knob.visual(
        Box((0.006, 0.014, 0.012)),
        origin=Origin(xyz=(0.0, -0.017, 0.018)),
        material="knob_plastic",
        name="lobe_2",
    )
    collar_knob.visual(
        Box((0.006, 0.014, 0.012)),
        origin=Origin(xyz=(0.0, -0.017, -0.018)),
        material="knob_plastic",
        name="lobe_3",
    )

    caster_0 = model.part("caster_0")
    _add_caster_wheel(
        caster_0,
        tire_radius=0.031,
        tire_width=0.020,
        rubber="dark_rubber",
        metal="chrome",
    )

    caster_1 = model.part("caster_1")
    _add_caster_wheel(
        caster_1,
        tire_radius=0.031,
        tire_width=0.020,
        rubber="dark_rubber",
        metal="chrome",
    )

    model.articulation(
        "height_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=post,
        origin=Origin(xyz=(sleeve_x, sleeve_y, sleeve_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.25, lower=0.0, upper=0.22),
    )
    model.articulation(
        "post_to_tray",
        ArticulationType.FIXED,
        parent=post,
        child=tray,
        origin=Origin(xyz=(0.30, 0.17, 0.409)),
    )
    model.articulation(
        "clip_hinge",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=clip_bar,
        origin=Origin(xyz=(0.205, 0.0, 0.010)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.5, lower=0.0, upper=1.15),
    )
    model.articulation(
        "knob_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=collar_knob,
        origin=Origin(xyz=(sleeve_x, sleeve_y - 0.065, collar_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=10.0),
    )
    model.articulation(
        "caster_0_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=caster_0,
        origin=Origin(xyz=(caster_x, -rail_y, caster_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=18.0),
    )
    model.articulation(
        "caster_1_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=caster_1,
        origin=Origin(xyz=(caster_x, rail_y, caster_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    post = object_model.get_part("post")
    tray = object_model.get_part("tray")
    clip_bar = object_model.get_part("clip_bar")
    collar_knob = object_model.get_part("collar_knob")

    height_slide = object_model.get_articulation("height_slide")
    clip_hinge = object_model.get_articulation("clip_hinge")

    ctx.expect_within(
        post,
        base,
        axes="xy",
        inner_elem="post",
        outer_elem="outer_sleeve",
        margin=0.0,
        name="post stays centered in sleeve at rest",
    )
    ctx.expect_overlap(
        post,
        base,
        axes="z",
        elem_a="post",
        elem_b="outer_sleeve",
        min_overlap=0.34,
        name="post remains well inserted at rest",
    )
    ctx.expect_contact(
        collar_knob,
        base,
        elem_a="stem",
        elem_b="knob_hub",
        name="knob seats on the collar hub",
    )
    ctx.expect_overlap(
        clip_bar,
        tray,
        axes="y",
        elem_a="bar",
        elem_b="tray_top",
        min_overlap=0.65,
        name="clip bar spans the tray width",
    )

    slide_limits = height_slide.motion_limits
    if slide_limits is not None and slide_limits.upper is not None:
        tray_rest = ctx.part_world_position(tray)
        with ctx.pose({height_slide: slide_limits.upper}):
            ctx.expect_within(
                post,
                base,
                axes="xy",
                inner_elem="post",
                outer_elem="outer_sleeve",
                margin=0.0,
                name="post stays centered in sleeve when raised",
            )
            ctx.expect_overlap(
                post,
                base,
                axes="z",
                elem_a="post",
                elem_b="outer_sleeve",
                min_overlap=0.12,
                name="post keeps retained insertion when raised",
            )
            tray_high = ctx.part_world_position(tray)
        ctx.check(
            "tray raises vertically",
            tray_rest is not None
            and tray_high is not None
            and tray_high[2] > tray_rest[2] + 0.18,
            details=f"rest={tray_rest}, high={tray_high}",
        )

    clip_limits = clip_hinge.motion_limits
    if clip_limits is not None and clip_limits.upper is not None:
        bar_rest = ctx.part_element_world_aabb(clip_bar, elem="bar")
        with ctx.pose({clip_hinge: clip_limits.upper}):
            bar_open = ctx.part_element_world_aabb(clip_bar, elem="bar")
        rest_center_z = None
        open_center_z = None
        if bar_rest is not None:
            rest_center_z = (bar_rest[0][2] + bar_rest[1][2]) * 0.5
        if bar_open is not None:
            open_center_z = (bar_open[0][2] + bar_open[1][2]) * 0.5
        ctx.check(
            "clip bar flips upward",
            rest_center_z is not None
            and open_center_z is not None
            and open_center_z > rest_center_z + 0.05,
            details=f"rest_z={rest_center_z}, open_z={open_center_z}",
        )

    return ctx.report()


object_model = build_object_model()
