from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _build_sleeve_shell() -> cq.Workplane:
    lower_socket = cq.Workplane("XY").circle(0.053).extrude(0.055)
    main_tube = cq.Workplane("XY").circle(0.037).extrude(0.350)
    clamp_band = cq.Workplane("XY").circle(0.047).extrude(0.070).translate((0.0, 0.0, 0.280))
    bore = cq.Workplane("XY").circle(0.0295).extrude(0.352)
    return lower_socket.union(main_tube).union(clamp_band).cut(bore)


def _build_can_shell() -> cq.Workplane:
    body = cq.Workplane("YZ").circle(0.138).extrude(0.340).translate((-0.150, 0.0, 0.0))
    front_ring = cq.Workplane("YZ").circle(0.156).extrude(0.030).translate((0.190, 0.0, 0.0))
    rear_box = cq.Workplane("XY").box(0.105, 0.105, 0.078).translate((-0.098, 0.0, 0.108))
    inner_bore = cq.Workplane("YZ").circle(0.122).extrude(0.365).translate((-0.115, 0.0, 0.0))
    rear_port = cq.Workplane("XY").box(0.040, 0.055, 0.032).translate((-0.150, 0.0, 0.108))
    return body.union(front_ring).union(rear_box).cut(inner_bore).cut(rear_port)


def _add_clip_geometry(clip_part, *, inward_sign: float, material) -> None:
    clip_part.visual(
        Cylinder(radius=0.0042, length=0.020),
        origin=Origin(),
        material=material,
        name="barrel",
    )
    clip_part.visual(
        Box((0.010, 0.010, 0.020)),
        origin=Origin(xyz=(0.005, 0.006 * inward_sign, 0.0)),
        material=material,
        name="leaf",
    )
    clip_part.visual(
        Box((0.024, 0.008, 0.018)),
        origin=Origin(xyz=(0.018, 0.010 * inward_sign, 0.0)),
        material=material,
        name="jaw",
    )
    clip_part.visual(
        Box((0.008, 0.010, 0.010)),
        origin=Origin(xyz=(0.028, 0.014 * inward_sign, -0.004)),
        material=material,
        name="hook",
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    min_corner, max_corner = aabb
    return tuple((min_corner[index] + max_corner[index]) / 2.0 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stage_spotlight_on_telescoping_yoke_stand")

    textured_black = model.material("textured_black", rgba=(0.12, 0.12, 0.13, 1.0))
    charcoal = model.material("charcoal", rgba=(0.19, 0.20, 0.22, 1.0))
    steel = model.material("steel", rgba=(0.63, 0.65, 0.69, 1.0))
    aluminum = model.material("aluminum", rgba=(0.54, 0.56, 0.60, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.060, length=0.082),
        origin=Origin(xyz=(0.0, 0.0, 0.114)),
        material=textured_black,
        name="hub",
    )
    stand.visual(
        mesh_from_cadquery(_build_sleeve_shell(), "stand_sleeve_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material=textured_black,
        name="sleeve_shell",
    )
    stand.visual(
        Cylinder(radius=0.006, length=0.026),
        origin=Origin(xyz=(0.047, 0.0, 0.395), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="clamp_stem",
    )
    stand.visual(
        Sphere(radius=0.015),
        origin=Origin(xyz=(0.064, 0.0, 0.395)),
        material=rubber,
        name="clamp_knob",
    )

    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        c = math.cos(angle)
        s = math.sin(angle)
        leg_mesh = tube_from_spline_points(
            [
                (0.040 * c, 0.040 * s, 0.134),
                (0.180 * c, 0.180 * s, 0.092),
                (0.430 * c, 0.430 * s, 0.022),
            ],
            radius=0.012,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        )
        stand.visual(
            mesh_from_geometry(leg_mesh, f"stand_leg_{index}"),
            material=textured_black,
            name=f"leg_{index}",
        )
        stand.visual(
            Sphere(radius=0.017),
            origin=Origin(xyz=(0.430 * c, 0.430 * s, 0.022)),
            material=rubber,
            name=f"foot_{index}",
        )

    post = model.part("post")
    post.visual(
        Cylinder(radius=0.0255, length=1.080),
        origin=Origin(xyz=(0.0, 0.0, 0.260)),
        material=aluminum,
        name="post_tube",
    )
    post.visual(
        Cylinder(radius=0.032, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.820)),
        material=charcoal,
        name="pan_seat",
    )
    post.visual(
        Cylinder(radius=0.037, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=charcoal,
        name="stop_collar",
    )

    model.articulation(
        "stand_to_post",
        ArticulationType.PRISMATIC,
        parent=stand,
        child=post,
        origin=Origin(xyz=(0.0, 0.0, 0.500)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=0.18,
            lower=0.0,
            upper=0.180,
        ),
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.031, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=textured_black,
        name="pan_spigot",
    )
    yoke.visual(
        Box((0.080, 0.314, 0.032)),
        origin=Origin(xyz=(-0.010, 0.0, 0.046)),
        material=textured_black,
        name="bridge",
    )
    yoke.visual(
        Box((0.070, 0.020, 0.340)),
        origin=Origin(xyz=(-0.010, 0.157, 0.190)),
        material=textured_black,
        name="arm_upper",
    )
    yoke.visual(
        Box((0.070, 0.020, 0.340)),
        origin=Origin(xyz=(-0.010, -0.157, 0.190)),
        material=textured_black,
        name="arm_lower",
    )
    yoke.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(xyz=(0.004, 0.176, 0.235), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="tilt_knob_upper",
    )
    yoke.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(xyz=(0.004, -0.176, 0.235), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="tilt_knob_lower",
    )
    yoke.visual(
        Cylinder(radius=0.023, length=0.008),
        origin=Origin(xyz=(0.0, 0.147, 0.235), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="bushing_upper",
    )
    yoke.visual(
        Cylinder(radius=0.023, length=0.008),
        origin=Origin(xyz=(0.0, -0.147, 0.235), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="bushing_lower",
    )

    model.articulation(
        "post_to_yoke",
        ArticulationType.CONTINUOUS,
        parent=post,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.845)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=2.2,
        ),
    )

    can = model.part("can")
    can.visual(
        mesh_from_cadquery(_build_can_shell(), "spotlight_can_shell"),
        material=charcoal,
        name="can_shell",
    )
    can.visual(
        Cylinder(radius=0.020, length=0.022),
        origin=Origin(xyz=(0.0, 0.132, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="trunnion_upper",
    )
    can.visual(
        Cylinder(radius=0.020, length=0.022),
        origin=Origin(xyz=(0.0, -0.132, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="trunnion_lower",
    )
    can.visual(
        Box((0.012, 0.018, 0.024)),
        origin=Origin(xyz=(0.216, 0.132, 0.0)),
        material=steel,
        name="clip_pad_upper",
    )
    can.visual(
        Box((0.012, 0.018, 0.024)),
        origin=Origin(xyz=(0.216, -0.132, 0.0)),
        material=steel,
        name="clip_pad_lower",
    )

    model.articulation(
        "yoke_to_can",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=can,
        origin=Origin(xyz=(0.0, 0.0, 0.235)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.8,
            lower=-0.70,
            upper=0.75,
        ),
    )

    left_clip = model.part("left_clip")
    _add_clip_geometry(left_clip, inward_sign=-1.0, material=steel)

    right_clip = model.part("right_clip")
    _add_clip_geometry(right_clip, inward_sign=1.0, material=steel)

    model.articulation(
        "can_to_left_clip",
        ArticulationType.REVOLUTE,
        parent=can,
        child=left_clip,
        origin=Origin(xyz=(0.226, 0.132, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=4.0,
            lower=0.0,
            upper=1.00,
        ),
    )
    model.articulation(
        "can_to_right_clip",
        ArticulationType.REVOLUTE,
        parent=can,
        child=right_clip,
        origin=Origin(xyz=(0.226, -0.132, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=4.0,
            lower=0.0,
            upper=1.00,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("stand")
    post = object_model.get_part("post")
    left_clip = object_model.get_part("left_clip")
    right_clip = object_model.get_part("right_clip")

    post_slide = object_model.get_articulation("stand_to_post")
    yoke_pan = object_model.get_articulation("post_to_yoke")
    can_tilt = object_model.get_articulation("yoke_to_can")
    left_hinge = object_model.get_articulation("can_to_left_clip")
    right_hinge = object_model.get_articulation("can_to_right_clip")

    slide_upper = post_slide.motion_limits.upper or 0.0

    with ctx.pose({post_slide: 0.0}):
        ctx.expect_within(
            post,
            stand,
            axes="xy",
            inner_elem="post_tube",
            outer_elem="sleeve_shell",
            margin=0.0035,
            name="post stays centered in sleeve at rest",
        )
        ctx.expect_overlap(
            post,
            stand,
            axes="z",
            elem_a="post_tube",
            elem_b="sleeve_shell",
            min_overlap=0.270,
            name="post remains deeply inserted at rest",
        )
        rest_post = ctx.part_world_position(post)

    with ctx.pose({post_slide: slide_upper}):
        ctx.expect_within(
            post,
            stand,
            axes="xy",
            inner_elem="post_tube",
            outer_elem="sleeve_shell",
            margin=0.0035,
            name="post stays centered when extended",
        )
        ctx.expect_overlap(
            post,
            stand,
            axes="z",
            elem_a="post_tube",
            elem_b="sleeve_shell",
            min_overlap=0.090,
            name="post retains insertion when extended",
        )
        extended_post = ctx.part_world_position(post)

    ctx.check(
        "post extends upward",
        rest_post is not None
        and extended_post is not None
        and extended_post[2] > rest_post[2] + 0.12,
        details=f"rest={rest_post}, extended={extended_post}",
    )

    clip_rest = ctx.part_world_position(left_clip)
    with ctx.pose({yoke_pan: math.pi / 2.0}):
        clip_panned = ctx.part_world_position(left_clip)
    ctx.check(
        "yoke pans around the post",
        clip_rest is not None
        and clip_panned is not None
        and math.hypot(clip_panned[0] - clip_rest[0], clip_panned[1] - clip_rest[1]) > 0.18,
        details=f"rest={clip_rest}, panned={clip_panned}",
    )

    with ctx.pose({can_tilt: 0.0}):
        tilt_rest = ctx.part_world_position(left_clip)
    with ctx.pose({can_tilt: 0.60}):
        tilt_up = ctx.part_world_position(left_clip)
    ctx.check(
        "can tilts upward",
        tilt_rest is not None and tilt_up is not None and tilt_up[2] > tilt_rest[2] + 0.07,
        details=f"rest={tilt_rest}, tilted={tilt_up}",
    )

    left_jaw_rest = _aabb_center(ctx.part_element_world_aabb(left_clip, elem="jaw"))
    with ctx.pose({left_hinge: 0.90}):
        left_jaw_open = _aabb_center(ctx.part_element_world_aabb(left_clip, elem="jaw"))
    ctx.check(
        "left clip swings outward",
        left_jaw_rest is not None
        and left_jaw_open is not None
        and left_jaw_open[1] > left_jaw_rest[1] + 0.010,
        details=f"rest={left_jaw_rest}, open={left_jaw_open}",
    )

    right_jaw_rest = _aabb_center(ctx.part_element_world_aabb(right_clip, elem="jaw"))
    with ctx.pose({right_hinge: 0.90}):
        right_jaw_open = _aabb_center(ctx.part_element_world_aabb(right_clip, elem="jaw"))
    ctx.check(
        "right clip swings outward",
        right_jaw_rest is not None
        and right_jaw_open is not None
        and right_jaw_open[1] < right_jaw_rest[1] - 0.010,
        details=f"rest={right_jaw_rest}, open={right_jaw_open}",
    )

    return ctx.report()


object_model = build_object_model()
