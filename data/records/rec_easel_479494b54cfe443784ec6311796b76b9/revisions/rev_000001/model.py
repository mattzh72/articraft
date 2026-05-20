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


BASE_H = 0.045
SLEEVE_R_OUT = 0.052
SLEEVE_R_IN = 0.035
SLEEVE_H = 0.520
COLLAR_H = 0.055
COLLAR_TOP_Z = BASE_H + SLEEVE_H + 0.025
POST_TRAVEL = 0.350
POST_HINGE_Z = 0.540


def _hollow_cylinder_mesh(name: str, outer_r: float, inner_r: float, height: float):
    """A real annular tube, not a solid proxy, for clear telescoping motion."""
    shape = cq.Workplane("XY").circle(outer_r).circle(inner_r).extrude(height)
    return mesh_from_cadquery(shape, name, tolerance=0.001, angular_tolerance=0.05)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_post_adjustable_easel")

    black_metal = model.material("satin_black_metal", rgba=(0.015, 0.014, 0.013, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.63, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.82, 0.85, 0.86, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.02, 0.02, 0.018, 1.0))
    beech_wood = model.material("warm_beech_wood", rgba=(0.74, 0.50, 0.28, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.285, length=BASE_H),
        origin=Origin(xyz=(0.0, 0.0, BASE_H / 2.0)),
        material=black_metal,
        name="weighted_round_base",
    )
    base.visual(
        _hollow_cylinder_mesh("base_tube_mesh", SLEEVE_R_OUT, SLEEVE_R_IN, SLEEVE_H),
        origin=Origin(xyz=(0.0, 0.0, BASE_H)),
        material=brushed_steel,
        name="base_tube",
    )
    base.visual(
        _hollow_cylinder_mesh("top_collar_mesh", 0.066, SLEEVE_R_IN, COLLAR_H),
        origin=Origin(xyz=(0.0, 0.0, COLLAR_TOP_Z - COLLAR_H)),
        material=black_metal,
        name="top_collar",
    )

    post = model.part("post")
    post.visual(
        Cylinder(radius=0.0275, length=0.960),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=chrome,
        name="inner_post",
    )
    post.visual(
        Cylinder(radius=SLEEVE_R_IN + 0.0005, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, -0.400)),
        material=black_plastic,
        name="lower_guide_bushing",
    )
    post.visual(
        Box((0.070, 0.180, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, POST_HINGE_Z - 0.055)),
        material=brushed_steel,
        name="fork_bridge",
    )
    for side, y in enumerate((-0.075, 0.075)):
        post.visual(
            Box((0.080, 0.026, 0.130)),
            origin=Origin(xyz=(0.0, y, POST_HINGE_Z)),
            material=brushed_steel,
            name=f"hinge_cheek_{side}",
        )

    model.articulation(
        "base_to_post",
        ArticulationType.PRISMATIC,
        parent=base,
        child=post,
        origin=Origin(xyz=(0.0, 0.0, COLLAR_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.20, lower=0.0, upper=POST_TRAVEL),
    )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.024, length=0.124),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="hinge_barrel",
    )
    for side, y in enumerate((-0.045, 0.045)):
        cradle.visual(
            Box((0.080, 0.018, 0.080)),
            origin=Origin(xyz=(0.050, y, 0.055)),
            material=brushed_steel,
            name=f"hinge_link_{side}",
        )
    cradle.visual(
        Box((0.026, 0.560, 0.032)),
        origin=Origin(xyz=(0.070, 0.0, 0.075)),
        material=beech_wood,
        name="bottom_rail",
    )
    cradle.visual(
        Box((0.120, 0.620, 0.035)),
        origin=Origin(xyz=(0.115, 0.0, 0.045)),
        material=beech_wood,
        name="canvas_shelf",
    )
    cradle.visual(
        Box((0.022, 0.620, 0.030)),
        origin=Origin(xyz=(0.170, 0.0, 0.072)),
        material=beech_wood,
        name="front_lip",
    )
    for side, y in enumerate((-0.250, 0.250)):
        cradle.visual(
            Box((0.032, 0.032, 0.690)),
            origin=Origin(xyz=(0.070, y, 0.390)),
            material=beech_wood,
            name=f"side_rail_{side}",
        )
    cradle.visual(
        Box((0.030, 0.560, 0.032)),
        origin=Origin(xyz=(0.070, 0.0, 0.735)),
        material=beech_wood,
        name="top_rail",
    )
    model.articulation(
        "post_to_cradle",
        ArticulationType.REVOLUTE,
        parent=post,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, POST_HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=1.0, lower=0.0, upper=0.75),
    )

    lock_knob = model.part("height_knob")
    lock_knob.visual(
        Cylinder(radius=0.007, length=0.036),
        origin=Origin(xyz=(0.018, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="threaded_stem",
    )
    lock_knob.visual(
        Cylinder(radius=0.040, length=0.024),
        origin=Origin(xyz=(0.048, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_plastic,
        name="knob_cap",
    )
    lock_knob.visual(
        Box((0.020, 0.050, 0.012)),
        origin=Origin(xyz=(0.048, 0.0, 0.037)),
        material=black_plastic,
        name="grip_tab",
    )
    model.articulation(
        "base_to_height_knob",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lock_knob,
        origin=Origin(xyz=(SLEEVE_R_OUT, 0.0, BASE_H + 0.360)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=-math.pi, upper=math.pi),
    )

    tilt_knob = model.part("tilt_knob")
    tilt_knob.visual(
        Cylinder(radius=0.044, length=0.028),
        origin=Origin(xyz=(0.0, 0.014, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_plastic,
        name="knob_cap",
    )
    tilt_knob.visual(
        Box((0.022, 0.026, 0.014)),
        origin=Origin(xyz=(0.0, 0.014, 0.039)),
        material=black_plastic,
        name="grip_tab",
    )
    model.articulation(
        "post_to_tilt_knob",
        ArticulationType.REVOLUTE,
        parent=post,
        child=tilt_knob,
        origin=Origin(xyz=(0.0, 0.088, POST_HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    post = object_model.get_part("post")
    cradle = object_model.get_part("cradle")
    slide = object_model.get_articulation("base_to_post")
    tilt = object_model.get_articulation("post_to_cradle")

    ctx.allow_overlap(
        post,
        base,
        elem_a="lower_guide_bushing",
        elem_b="base_tube",
        reason="The hidden nylon guide bushing is represented with a tiny interference fit against the telescoping sleeve wall.",
    )

    ctx.expect_within(
        post,
        base,
        axes="xy",
        inner_elem="inner_post",
        outer_elem="top_collar",
        margin=0.010,
        name="inner post is centered inside the top collar",
    )
    ctx.expect_overlap(
        post,
        base,
        axes="z",
        elem_a="inner_post",
        elem_b="base_tube",
        min_overlap=0.200,
        name="collapsed telescoping post remains inserted",
    )
    ctx.expect_within(
        post,
        base,
        axes="xy",
        inner_elem="lower_guide_bushing",
        outer_elem="base_tube",
        margin=0.002,
        name="guide bushing stays inside the sleeve bore",
    )

    rest_pos = ctx.part_world_position(post)
    with ctx.pose({slide: POST_TRAVEL}):
        ctx.expect_overlap(
            post,
            base,
            axes="z",
            elem_a="inner_post",
            elem_b="base_tube",
            min_overlap=0.080,
            name="extended telescoping post keeps retained insertion",
        )
        ctx.expect_overlap(
            post,
            base,
            axes="z",
            elem_a="lower_guide_bushing",
            elem_b="base_tube",
            min_overlap=0.035,
            name="extended guide bushing remains inside base tube",
        )
        raised_pos = ctx.part_world_position(post)
    ctx.check(
        "prismatic post raises the cradle support",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.300,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    rest_aabb = ctx.part_world_aabb(cradle)
    with ctx.pose({tilt: 0.65}):
        tilted_aabb = ctx.part_world_aabb(cradle)
    if rest_aabb is not None and tilted_aabb is not None:
        rest_center_x = (rest_aabb[0][0] + rest_aabb[1][0]) / 2.0
        tilted_center_x = (tilted_aabb[0][0] + tilted_aabb[1][0]) / 2.0
        ctx.check(
            "cradle hinge tilts the canvas support forward",
            tilted_center_x > rest_center_x + 0.070,
            details=f"rest_center_x={rest_center_x}, tilted_center_x={tilted_center_x}",
        )
    else:
        ctx.fail("cradle hinge tilts the canvas support forward", "missing cradle AABB")

    return ctx.report()


object_model = build_object_model()
