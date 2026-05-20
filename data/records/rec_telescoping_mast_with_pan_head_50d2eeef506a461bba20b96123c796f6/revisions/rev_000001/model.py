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
    mesh_from_cadquery,
)
import cadquery as cq


def _annular_tube(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    """A thin-walled vertical tube with real open bore, authored in meters."""
    outer = cq.Workplane("XY").circle(outer_radius).extrude(height)
    cutter = cq.Workplane("XY").circle(inner_radius).extrude(height + 0.004).translate((0.0, 0.0, -0.002))
    return outer.cut(cutter)


def _base_sleeve_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").circle(0.090).extrude(0.018)
    sleeve = _annular_tube(0.032, 0.025, 0.280).translate((0.0, 0.0, 0.018))
    top_collar = _annular_tube(0.042, 0.025, 0.032).translate((0.0, 0.0, 0.266))
    clamp_lug = cq.Workplane("XY").box(0.030, 0.026, 0.036).translate((0.047, 0.0, 0.284))
    clamp_screw = (
        cq.Workplane("XY")
        .circle(0.006)
        .extrude(0.045)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90)
        .translate((0.062, 0.0, 0.284))
    )
    return plate.union(sleeve).union(top_collar).union(clamp_lug).union(clamp_screw)


def _first_stage_shape() -> cq.Workplane:
    mast = _annular_tube(0.020, 0.0145, 0.400)
    upper_collar = _annular_tube(0.026, 0.0145, 0.030).translate((0.0, 0.0, 0.370))
    etched_band = _annular_tube(0.0215, 0.0145, 0.010).translate((0.0, 0.0, 0.305))
    sleeve_stop = _annular_tube(0.038, 0.020, 0.012).translate((0.0, 0.0, 0.220))
    return mast.union(upper_collar).union(etched_band).union(sleeve_stop)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_mast_pan_head")

    anodized_black = model.material("anodized_black", color=(0.02, 0.022, 0.026, 1.0))
    satin_aluminum = model.material("satin_aluminum", color=(0.68, 0.70, 0.72, 1.0))
    dark_aluminum = model.material("dark_aluminum", color=(0.24, 0.26, 0.28, 1.0))
    rubber_black = model.material("rubber_black", color=(0.005, 0.005, 0.006, 1.0))
    sensor_glass = model.material("sensor_glass", color=(0.05, 0.10, 0.16, 1.0))

    base_sleeve = model.part("base_sleeve")
    base_sleeve.visual(
        mesh_from_cadquery(_base_sleeve_shape(), "base_sleeve"),
        material=anodized_black,
        name="sleeve_body",
    )
    for index, (x, y) in enumerate(((0.065, 0.065), (-0.065, 0.065), (0.065, -0.065), (-0.065, -0.065))):
        base_sleeve.visual(
            Cylinder(radius=0.006, length=0.003),
            origin=Origin(xyz=(x, y, 0.019)),
            material=dark_aluminum,
            name=f"mount_bolt_{index}",
        )

    mast_stage_0 = model.part("mast_stage_0")
    mast_stage_0.visual(
        mesh_from_cadquery(_first_stage_shape(), "mast_stage_0"),
        origin=Origin(xyz=(0.0, 0.0, -0.220)),
        material=satin_aluminum,
        name="stage_tube",
    )

    mast_stage_1 = model.part("mast_stage_1")
    mast_stage_1.visual(
        Cylinder(radius=0.0105, length=0.380),
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
        material=dark_aluminum,
        name="inner_tube",
    )
    mast_stage_1.visual(
        Cylinder(radius=0.0185, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=satin_aluminum,
        name="stage_stop",
    )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.028, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=anodized_black,
        name="turntable_bearing",
    )
    pan_head.visual(
        Box((0.120, 0.078, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=dark_aluminum,
        name="sensor_plate",
    )
    pan_head.visual(
        Box((0.028, 0.044, 0.026)),
        origin=Origin(xyz=(0.046, 0.0, 0.043)),
        material=anodized_black,
        name="front_sensor_block",
    )
    pan_head.visual(
        Cylinder(radius=0.012, length=0.014),
        origin=Origin(xyz=(0.067, 0.0, 0.043), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=sensor_glass,
        name="front_lens",
    )
    pan_head.visual(
        Box((0.070, 0.010, 0.006)),
        origin=Origin(xyz=(-0.015, 0.0, 0.033)),
        material=rubber_black,
        name="rubber_pad",
    )

    model.articulation(
        "base_to_stage_0",
        ArticulationType.PRISMATIC,
        parent=base_sleeve,
        child=mast_stage_0,
        origin=Origin(xyz=(0.0, 0.0, 0.298)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.25, lower=0.0, upper=0.150),
    )
    model.articulation(
        "stage_0_to_stage_1",
        ArticulationType.PRISMATIC,
        parent=mast_stage_0,
        child=mast_stage_1,
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.25, lower=0.0, upper=0.150),
    )
    model.articulation(
        "stage_1_to_head",
        ArticulationType.REVOLUTE,
        parent=mast_stage_1,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_sleeve = object_model.get_part("base_sleeve")
    mast_stage_0 = object_model.get_part("mast_stage_0")
    mast_stage_1 = object_model.get_part("mast_stage_1")
    pan_head = object_model.get_part("pan_head")
    base_slide = object_model.get_articulation("base_to_stage_0")
    upper_slide = object_model.get_articulation("stage_0_to_stage_1")
    pan = object_model.get_articulation("stage_1_to_head")

    ctx.allow_overlap(
        base_sleeve,
        mast_stage_0,
        elem_a="sleeve_body",
        elem_b="stage_tube",
        reason=(
            "The first mast stage is intentionally captured inside the base sleeve; "
            "the visible model includes the retained sliding insertion and stop collar."
        ),
    )

    ctx.expect_within(
        mast_stage_0,
        base_sleeve,
        axes="xy",
        inner_elem="stage_tube",
        outer_elem="sleeve_body",
        margin=0.002,
        name="first stage is centered in base sleeve",
    )
    ctx.expect_overlap(
        mast_stage_0,
        base_sleeve,
        axes="z",
        elem_a="stage_tube",
        elem_b="sleeve_body",
        min_overlap=0.070,
        name="first stage retained in base at rest",
    )
    ctx.expect_within(
        mast_stage_1,
        mast_stage_0,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="stage_tube",
        margin=0.002,
        name="second stage is centered in first stage",
    )
    ctx.expect_overlap(
        mast_stage_1,
        mast_stage_0,
        axes="z",
        elem_a="inner_tube",
        elem_b="stage_tube",
        min_overlap=0.120,
        name="second stage retained in first stage at rest",
    )

    rest_stage_0 = ctx.part_world_position(mast_stage_0)
    rest_stage_1 = ctx.part_world_position(mast_stage_1)
    with ctx.pose({base_slide: 0.150, upper_slide: 0.150}):
        ctx.expect_overlap(
            mast_stage_0,
            base_sleeve,
            axes="z",
            elem_a="stage_tube",
            elem_b="sleeve_body",
            min_overlap=0.060,
            name="first stage remains inserted when extended",
        )
        ctx.expect_overlap(
            mast_stage_1,
            mast_stage_0,
            axes="z",
            elem_a="inner_tube",
            elem_b="stage_tube",
            min_overlap=0.060,
            name="second stage remains inserted when extended",
        )
        extended_stage_0 = ctx.part_world_position(mast_stage_0)
        extended_stage_1 = ctx.part_world_position(mast_stage_1)

    ctx.check(
        "first stage slides upward 150 mm",
        rest_stage_0 is not None
        and extended_stage_0 is not None
        and extended_stage_0[2] > rest_stage_0[2] + 0.145,
        details=f"rest={rest_stage_0}, extended={extended_stage_0}",
    )
    ctx.check(
        "second stage adds another 150 mm",
        rest_stage_1 is not None
        and extended_stage_1 is not None
        and extended_stage_1[2] > rest_stage_1[2] + 0.295,
        details=f"rest={rest_stage_1}, extended={extended_stage_1}",
    )

    def lens_center_x():
        bounds = ctx.part_element_world_aabb(pan_head, elem="front_lens")
        if bounds is None:
            return None
        low, high = bounds
        return (low[0] + high[0]) * 0.5

    lens_x_rest = lens_center_x()
    with ctx.pose({pan: math.pi}):
        lens_x_rotated = lens_center_x()
    ctx.check(
        "pan head rotates about vertical axis",
        lens_x_rest is not None and lens_x_rotated is not None and lens_x_rotated < -0.04 < lens_x_rest,
        details=f"lens_x_rest={lens_x_rest}, lens_x_rotated={lens_x_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
