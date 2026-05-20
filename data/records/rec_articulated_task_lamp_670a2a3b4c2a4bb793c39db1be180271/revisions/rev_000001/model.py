from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _shade_shell_mesh():
    """Thin, open conical reflector shade, revolved around local +Z."""

    outer_profile = [
        (0.036, -0.040),
        (0.048, -0.050),
        (0.108, -0.205),
        (0.116, -0.220),
    ]
    inner_profile = [
        (0.024, -0.046),
        (0.039, -0.060),
        (0.098, -0.202),
        (0.106, -0.213),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=72,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
        "conical_reflector_shade",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swing_arm_desk_lamp")

    black = model.material("black_enamel", rgba=(0.02, 0.022, 0.025, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.11, 0.12, 0.13, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.62, 0.63, 0.60, 1.0))
    rubber = model.material("black_rubber", rgba=(0.006, 0.006, 0.006, 1.0))
    warm_glass = model.material("warm_bulb_glass", rgba=(1.0, 0.86, 0.52, 0.72))

    clamp = model.part("clamp")
    clamp.visual(
        Box((0.045, 0.090, 0.300)),
        origin=Origin(xyz=(0.000, 0.000, 0.150)),
        material=dark_metal,
        name="back_spine",
    )
    clamp.visual(
        Box((0.180, 0.090, 0.045)),
        origin=Origin(xyz=(0.070, 0.000, 0.275)),
        material=dark_metal,
        name="top_arm",
    )
    clamp.visual(
        Box((0.180, 0.090, 0.045)),
        origin=Origin(xyz=(0.070, 0.000, 0.035)),
        material=dark_metal,
        name="lower_arm",
    )
    clamp.visual(
        Cylinder(radius=0.032, length=0.012),
        origin=Origin(xyz=(0.125, 0.000, 0.2465)),
        material=rubber,
        name="fixed_jaw_pad",
    )
    clamp.visual(
        Cylinder(radius=0.022, length=0.120),
        origin=Origin(xyz=(0.000, 0.000, 0.357)),
        material=dark_metal,
        name="upright_post",
    )
    clamp.visual(
        Cylinder(radius=0.044, length=0.024),
        origin=Origin(xyz=(0.000, 0.000, 0.429)),
        material=brushed_steel,
        name="swivel_bearing",
    )
    clamp.visual(
        Cylinder(radius=0.017, length=0.018),
        origin=Origin(xyz=(0.125, 0.000, 0.058)),
        material=brushed_steel,
        name="threaded_boss",
    )

    clamp_screw = model.part("clamp_screw")
    clamp_screw.visual(
        Cylinder(radius=0.0085, length=0.130),
        origin=Origin(xyz=(0.000, 0.000, 0.065)),
        material=brushed_steel,
        name="screw_stem",
    )
    clamp_screw.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(xyz=(0.000, 0.000, 0.018)),
        material=black,
        name="thumb_knob",
    )
    clamp_screw.visual(
        Cylinder(radius=0.030, length=0.010),
        origin=Origin(xyz=(0.000, 0.000, 0.135)),
        material=rubber,
        name="swivel_pad",
    )

    boom = model.part("boom")
    boom.visual(
        Cylinder(radius=0.027, length=0.044),
        origin=Origin(xyz=(0.000, 0.000, 0.022)),
        material=brushed_steel,
        name="pivot_collar",
    )
    boom.visual(
        Cylinder(radius=0.018, length=0.620),
        origin=Origin(xyz=(0.310, 0.000, 0.035), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="horizontal_tube",
    )
    boom.visual(
        Box((0.060, 0.012, 0.070)),
        origin=Origin(xyz=(0.650, 0.034, 0.035)),
        material=dark_metal,
        name="end_yoke_0",
    )
    boom.visual(
        Box((0.060, 0.012, 0.070)),
        origin=Origin(xyz=(0.650, -0.034, 0.035)),
        material=dark_metal,
        name="end_yoke_1",
    )
    boom.visual(
        Box((0.070, 0.080, 0.022)),
        origin=Origin(xyz=(0.615, 0.000, 0.064)),
        material=dark_metal,
        name="yoke_bridge",
    )

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=0.018, length=0.056),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="tilt_barrel",
    )
    shade.visual(
        Cylinder(radius=0.010, length=0.052),
        origin=Origin(xyz=(0.000, 0.000, -0.026)),
        material=brushed_steel,
        name="neck_stem",
    )
    shade.visual(
        _shade_shell_mesh(),
        origin=Origin(),
        material=black,
        name="reflector_shell",
    )
    shade.visual(
        Cylinder(radius=0.026, length=0.052),
        origin=Origin(xyz=(0.000, 0.000, -0.061)),
        material=brushed_steel,
        name="lamp_socket",
    )
    shade.visual(
        Sphere(radius=0.032),
        origin=Origin(xyz=(0.000, 0.000, -0.115)),
        material=warm_glass,
        name="bulb",
    )

    model.articulation(
        "clamp_to_screw",
        ArticulationType.PRISMATIC,
        parent=clamp,
        child=clamp_screw,
        origin=Origin(xyz=(0.125, 0.000, 0.058)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.04, lower=0.0, upper=0.055),
    )
    model.articulation(
        "clamp_to_boom",
        ArticulationType.REVOLUTE,
        parent=clamp,
        child=boom,
        origin=Origin(xyz=(0.000, 0.000, 0.441)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=math.radians(-135),
            upper=math.radians(135),
        ),
    )
    model.articulation(
        "boom_to_shade",
        ArticulationType.REVOLUTE,
        parent=boom,
        child=shade,
        origin=Origin(xyz=(0.650, 0.000, 0.035)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.2,
            lower=math.radians(-55),
            upper=math.radians(70),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    clamp = object_model.get_part("clamp")
    screw = object_model.get_part("clamp_screw")
    boom = object_model.get_part("boom")
    shade = object_model.get_part("shade")
    screw_slide = object_model.get_articulation("clamp_to_screw")
    boom_swivel = object_model.get_articulation("clamp_to_boom")
    shade_tilt = object_model.get_articulation("boom_to_shade")

    ctx.allow_overlap(
        clamp,
        screw,
        elem_a="threaded_boss",
        elem_b="screw_stem",
        reason="The clamp screw is intentionally captured in the threaded boss.",
    )
    ctx.expect_within(
        screw,
        clamp,
        axes="xy",
        inner_elem="screw_stem",
        outer_elem="threaded_boss",
        margin=0.0,
        name="screw stem centered inside threaded boss",
    )
    ctx.expect_overlap(
        screw,
        clamp,
        axes="z",
        elem_a="screw_stem",
        elem_b="threaded_boss",
        min_overlap=0.008,
        name="screw stem remains engaged in threaded boss",
    )

    with ctx.pose({boom_swivel: 0.0, shade_tilt: 0.0, screw_slide: 0.0}):
        ctx.expect_gap(
            boom,
            clamp,
            axis="z",
            positive_elem="pivot_collar",
            negative_elem="swivel_bearing",
            min_gap=0.0,
            max_gap=0.003,
            name="boom collar sits on clamp bearing",
        )
        ctx.expect_gap(
            boom,
            shade,
            axis="y",
            positive_elem="end_yoke_0",
            negative_elem="tilt_barrel",
            min_gap=0.0,
            max_gap=0.002,
            name="upper yoke cheek captures barrel",
        )
        ctx.expect_gap(
            shade,
            boom,
            axis="y",
            positive_elem="tilt_barrel",
            negative_elem="end_yoke_1",
            min_gap=0.0,
            max_gap=0.002,
            name="lower yoke cheek captures barrel",
        )
        ctx.expect_gap(
            screw,
            clamp,
            axis="z",
            positive_elem="screw_stem",
            negative_elem="lower_arm",
            min_gap=0.0,
            max_gap=0.001,
            name="clamp screw seats on lower arm",
        )

    rest_screw_pos = ctx.part_world_position(screw)
    with ctx.pose({screw_slide: 0.055}):
        raised_screw_pos = ctx.part_world_position(screw)

    ctx.check(
        "clamp screw advances upward",
        rest_screw_pos is not None
        and raised_screw_pos is not None
        and raised_screw_pos[2] > rest_screw_pos[2] + 0.050,
        details=f"rest={rest_screw_pos}, raised={raised_screw_pos}",
    )

    rest_shade_pos = ctx.part_world_position(shade)
    with ctx.pose({boom_swivel: math.radians(45)}):
        swiveled_shade_pos = ctx.part_world_position(shade)

    ctx.check(
        "boom swivel swings shade sideways",
        rest_shade_pos is not None
        and swiveled_shade_pos is not None
        and abs(swiveled_shade_pos[1] - rest_shade_pos[1]) > 0.35,
        details=f"rest={rest_shade_pos}, swiveled={swiveled_shade_pos}",
    )

    rest_shade_aabb = ctx.part_world_aabb(shade)
    with ctx.pose({shade_tilt: math.radians(45)}):
        tilted_shade_aabb = ctx.part_world_aabb(shade)

    ctx.check(
        "shade tilt raises reflector rim",
        rest_shade_aabb is not None
        and tilted_shade_aabb is not None
        and tilted_shade_aabb[1][2] > rest_shade_aabb[1][2] + 0.030,
        details=f"rest={rest_shade_aabb}, tilted={tilted_shade_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
