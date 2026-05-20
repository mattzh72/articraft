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
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


SIDEWALK_SIZE = 1.20
SIDEWALK_THICKNESS = 0.18
SLEEVE_OUTER_RADIUS = 0.095
SLEEVE_INNER_RADIUS = 0.079
SLEEVE_FLANGE_RADIUS = 0.110
SLEEVE_DEPTH = 0.34
SLEEVE_FLANGE_THICKNESS = 0.014
POST_OUTER_RADIUS = 0.074
POST_INNER_RADIUS = 0.066
POST_TOTAL_LENGTH = 1.26
POST_LOWER_EXTENSION = 0.34
POST_TRAVEL = 0.22
SLEEVE_SEGMENTS = 36
HINGE_AXIS_Y = 0.148
HINGE_AXIS_Z = 0.0175


def _sidewalk_shape() -> cq.Workplane:
    slab = cq.Workplane("XY").box(
        SIDEWALK_SIZE,
        SIDEWALK_SIZE,
        SIDEWALK_THICKNESS,
    ).translate((0.0, 0.0, -SIDEWALK_THICKNESS * 0.5))
    opening = cq.Workplane("XY").circle(SLEEVE_OUTER_RADIUS + 0.004).extrude(-SIDEWALK_THICKNESS)
    chamfer_relief = (
        cq.Workplane("XY")
        .circle(SLEEVE_FLANGE_RADIUS + 0.015)
        .circle(SLEEVE_OUTER_RADIUS + 0.004)
        .extrude(-0.010)
    )
    return slab.cut(opening).cut(chamfer_relief)


def _sleeve_shape() -> cq.Workplane:
    sleeve_tube = (
        cq.Workplane("XY")
        .circle(SLEEVE_OUTER_RADIUS)
        .circle(SLEEVE_INNER_RADIUS)
        .extrude(-SLEEVE_DEPTH)
    )
    top_flange = (
        cq.Workplane("XY")
        .circle(SLEEVE_FLANGE_RADIUS)
        .circle(SLEEVE_INNER_RADIUS)
        .extrude(SLEEVE_FLANGE_THICKNESS)
    )
    housing = (
        cq.Workplane("XY")
        .center(0.0, 0.118)
        .rect(0.074, 0.056)
        .extrude(SLEEVE_FLANGE_THICKNESS)
    )
    left_knuckle = (
        cq.Workplane("YZ")
        .workplane(offset=-0.031)
        .center(HINGE_AXIS_Y, HINGE_AXIS_Z)
        .circle(0.0042)
        .extrude(0.018)
    )
    right_knuckle = (
        cq.Workplane("YZ")
        .workplane(offset=0.013)
        .center(HINGE_AXIS_Y, HINGE_AXIS_Z)
        .circle(0.0042)
        .extrude(0.018)
    )
    return sleeve_tube.union(top_flange).union(housing).union(left_knuckle).union(right_knuckle)


def _post_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(POST_OUTER_RADIUS)
        .circle(POST_INNER_RADIUS)
        .extrude(POST_TOTAL_LENGTH)
        .translate((0.0, 0.0, -POST_LOWER_EXTENSION))
    )


def _cover_shape() -> cq.Workplane:
    cover_plate = (
        cq.Workplane("XY")
        .center(0.0, -0.0295)
        .rect(0.066, 0.051)
        .extrude(-0.0030)
    )
    hinge_bridge = (
        cq.Workplane("XY")
        .center(0.0, -0.006)
        .rect(0.030, 0.012)
        .extrude(-0.0030)
    )
    center_knuckle = (
        cq.Workplane("YZ")
        .workplane(offset=-0.0115)
        .center(0.0, 0.0)
        .circle(0.0038)
        .extrude(0.023)
    )
    finger_ridge = (
        cq.Workplane("XY")
        .center(0.0, -0.049)
        .rect(0.024, 0.007)
        .extrude(0.0012)
    )
    return cover_plate.union(hinge_bridge).union(center_knuckle).union(finger_ridge)


def _cap_shape() -> cq.Workplane:
    cap = cq.Workplane("XY").circle(0.078).extrude(0.010)
    cap = cap.union(cq.Workplane("XY").circle(0.028).extrude(0.018))
    cap = cap.union(cq.Workplane("XY").circle(0.068).extrude(0.004))
    return cap.faces(">Z").workplane().rect(0.022, 0.004).cutBlind(-0.006)


def _add_ring_segments(
    part,
    *,
    prefix: str,
    inner_radius: float,
    outer_radius: float,
    height: float,
    center_z: float,
    material,
) -> None:
    mean_radius = 0.5 * (inner_radius + outer_radius)
    radial_thickness = outer_radius - inner_radius
    tangential_length = 1.08 * 2.0 * mean_radius * math.tan(math.pi / SLEEVE_SEGMENTS)
    for index in range(SLEEVE_SEGMENTS):
        angle = 2.0 * math.pi * index / SLEEVE_SEGMENTS
        part.visual(
            Box((tangential_length, radial_thickness, height)),
            origin=Origin(
                xyz=(mean_radius * math.cos(angle), mean_radius * math.sin(angle), center_z),
                rpy=(0.0, 0.0, angle + math.pi * 0.5),
            ),
            material=material,
            name=f"{prefix}_{index}",
        )


def _add_guide_pads(part, *, material) -> None:
    radial_thickness = SLEEVE_INNER_RADIUS - POST_OUTER_RADIUS
    mean_radius = POST_OUTER_RADIUS + radial_thickness * 0.5
    tangential_length = 0.018
    for index in range(3):
        angle = 2.0 * math.pi * index / 3.0 + math.pi / 6.0
        part.visual(
            Box((tangential_length, radial_thickness, 0.090)),
            origin=Origin(
                xyz=(mean_radius * math.cos(angle), mean_radius * math.sin(angle), -0.080),
                rpy=(0.0, 0.0, angle + math.pi * 0.5),
            ),
            material=material,
            name=f"guide_pad_{index}",
        )


def _add_support_lugs(part, *, material) -> None:
    for index in range(3):
        angle = 2.0 * math.pi * index / 3.0 + math.pi / 6.0
        part.visual(
            Box((0.024, 0.030, 0.008)),
            origin=Origin(
                xyz=(0.085 * math.cos(angle), 0.085 * math.sin(angle), 0.004),
                rpy=(0.0, 0.0, angle + math.pi * 0.5),
            ),
            material=material,
            name=f"support_lug_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="removable_access_bollard")

    concrete = model.material("concrete", rgba=(0.70, 0.70, 0.68, 1.0))
    galvanized = model.material("galvanized", rgba=(0.62, 0.65, 0.69, 1.0))
    post_paint = model.material("post_paint", rgba=(0.18, 0.20, 0.22, 1.0))
    satin_cap = model.material("satin_cap", rgba=(0.75, 0.77, 0.80, 1.0))

    sleeve = model.part("sleeve")
    sleeve.visual(
        mesh_from_cadquery(_sidewalk_shape(), "bollard_sidewalk"),
        material=concrete,
        name="sidewalk",
    )
    _add_ring_segments(
        sleeve,
        prefix="wall",
        inner_radius=SLEEVE_INNER_RADIUS,
        outer_radius=SLEEVE_OUTER_RADIUS,
        height=SLEEVE_DEPTH,
        center_z=-SLEEVE_DEPTH * 0.5,
        material=galvanized,
    )
    _add_ring_segments(
        sleeve,
        prefix="rim",
        inner_radius=SLEEVE_INNER_RADIUS,
        outer_radius=SLEEVE_FLANGE_RADIUS,
        height=SLEEVE_FLANGE_THICKNESS,
        center_z=SLEEVE_FLANGE_THICKNESS * 0.5,
        material=galvanized,
    )
    sleeve.visual(
        Box((0.010, 0.056, SLEEVE_FLANGE_THICKNESS)),
        origin=Origin(xyz=(-0.028, 0.118, SLEEVE_FLANGE_THICKNESS * 0.5)),
        material=galvanized,
        name="lock_cheek_0",
    )
    sleeve.visual(
        Box((0.010, 0.056, SLEEVE_FLANGE_THICKNESS)),
        origin=Origin(xyz=(0.028, 0.118, SLEEVE_FLANGE_THICKNESS * 0.5)),
        material=galvanized,
        name="lock_cheek_1",
    )
    sleeve.visual(
        Box((0.056, 0.010, SLEEVE_FLANGE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.141, SLEEVE_FLANGE_THICKNESS * 0.5)),
        material=galvanized,
        name="lock_backstop",
    )
    sleeve.visual(
        Cylinder(radius=0.0042, length=0.018),
        origin=Origin(
            xyz=(-0.022, HINGE_AXIS_Y, HINGE_AXIS_Z),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=galvanized,
        name="hinge_knuckle_0",
    )
    sleeve.visual(
        Cylinder(radius=0.0042, length=0.018),
        origin=Origin(
            xyz=(0.022, HINGE_AXIS_Y, HINGE_AXIS_Z),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=galvanized,
        name="hinge_knuckle_1",
    )
    _add_guide_pads(sleeve, material=galvanized)

    post = model.part("post")
    post.visual(
        mesh_from_cadquery(_post_shape(), "bollard_post"),
        material=post_paint,
        name="post_shell",
    )
    _add_support_lugs(post, material=post_paint)

    lock_cover = model.part("lock_cover")
    lock_cover.visual(
        Box((0.066, 0.051, 0.003)),
        origin=Origin(xyz=(0.0, -0.0295, -0.0015)),
        material=galvanized,
        name="cover_plate",
    )
    lock_cover.visual(
        Box((0.030, 0.012, 0.003)),
        origin=Origin(xyz=(0.0, -0.006, -0.0015)),
        material=galvanized,
        name="cover_bridge",
    )
    lock_cover.visual(
        Cylinder(radius=0.0038, length=0.023),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=galvanized,
        name="cover_knuckle",
    )
    lock_cover.visual(
        Box((0.024, 0.007, 0.0012)),
        origin=Origin(xyz=(0.0, -0.049, 0.0006)),
        material=galvanized,
        name="cover_pull",
    )

    top_cap = model.part("top_cap")
    top_cap.visual(
        mesh_from_cadquery(_cap_shape(), "bollard_top_cap"),
        material=satin_cap,
        name="cap_shell",
    )

    model.articulation(
        "sleeve_to_post",
        ArticulationType.PRISMATIC,
        parent=sleeve,
        child=post,
        origin=Origin(xyz=(0.0, 0.0, SLEEVE_FLANGE_THICKNESS)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=0.12,
            lower=0.0,
            upper=POST_TRAVEL,
        ),
    )
    model.articulation(
        "sleeve_to_lock_cover",
        ArticulationType.REVOLUTE,
        parent=sleeve,
        child=lock_cover,
        origin=Origin(xyz=(0.0, HINGE_AXIS_Y, HINGE_AXIS_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=0.0,
            upper=1.55,
        ),
    )
    model.articulation(
        "post_to_top_cap",
        ArticulationType.CONTINUOUS,
        parent=post,
        child=top_cap,
        origin=Origin(xyz=(0.0, 0.0, POST_TOTAL_LENGTH - POST_LOWER_EXTENSION)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=8.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    sleeve = object_model.get_part("sleeve")
    post = object_model.get_part("post")
    lock_cover = object_model.get_part("lock_cover")

    post_slide = object_model.get_articulation("sleeve_to_post")
    cover_hinge = object_model.get_articulation("sleeve_to_lock_cover")
    cap_spin = object_model.get_articulation("post_to_top_cap")

    ctx.check(
        "post articulation is prismatic",
        post_slide.articulation_type == ArticulationType.PRISMATIC and tuple(post_slide.axis) == (0.0, 0.0, 1.0),
        details=f"type={post_slide.articulation_type}, axis={post_slide.axis}",
    )
    ctx.check(
        "lock cover articulation is revolute",
        cover_hinge.articulation_type == ArticulationType.REVOLUTE and tuple(cover_hinge.axis) == (-1.0, 0.0, 0.0),
        details=f"type={cover_hinge.articulation_type}, axis={cover_hinge.axis}",
    )
    ctx.check(
        "top cap articulation is continuous",
        cap_spin.articulation_type == ArticulationType.CONTINUOUS and tuple(cap_spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={cap_spin.articulation_type}, axis={cap_spin.axis}",
    )

    ctx.expect_origin_distance(
        post,
        sleeve,
        axes="xy",
        max_dist=0.002,
        name="seated post stays coaxial with sleeve",
    )
    ctx.expect_overlap(
        post,
        sleeve,
        axes="z",
        min_overlap=0.32,
        name="seated post remains deeply engaged in sleeve",
    )

    post_limits = post_slide.motion_limits
    rest_post_position = ctx.part_world_position(post)
    raised_post_position = None
    if post_limits is not None and post_limits.upper is not None:
        with ctx.pose({post_slide: post_limits.upper}):
            ctx.expect_origin_distance(
                post,
                sleeve,
                axes="xy",
                max_dist=0.002,
                name="raised post stays coaxial with sleeve",
            )
            ctx.expect_overlap(
                post,
                sleeve,
                axes="z",
                min_overlap=0.11,
                name="raised post retains insertion in sleeve",
            )
            raised_post_position = ctx.part_world_position(post)

    ctx.check(
        "post lifts upward at full travel",
        rest_post_position is not None
        and raised_post_position is not None
        and raised_post_position[2] > rest_post_position[2] + 0.18,
        details=f"rest={rest_post_position}, raised={raised_post_position}",
    )

    cover_limits = cover_hinge.motion_limits
    rest_cover_aabb = ctx.part_world_aabb(lock_cover)
    open_cover_aabb = None
    if cover_limits is not None and cover_limits.upper is not None:
        with ctx.pose({cover_hinge: cover_limits.upper}):
            open_cover_aabb = ctx.part_world_aabb(lock_cover)

    ctx.check(
        "lock cover swings upward from the sleeve rim",
        rest_cover_aabb is not None
        and open_cover_aabb is not None
        and open_cover_aabb[1][2] > rest_cover_aabb[1][2] + 0.035,
        details=f"rest={rest_cover_aabb}, open={open_cover_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
