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


SLEEVE_OUTER_RADIUS = 0.145
SLEEVE_INNER_RADIUS = 0.118
SLEEVE_DEPTH = 1.05
SLEEVE_FLANGE_RADIUS = 0.175
SLEEVE_FLANGE_THICKNESS = 0.022

POST_RADIUS = 0.105
POST_LENGTH = 1.12
POST_CENTER_Z = -0.41
POST_TRAVEL = 0.72

CROWN_RADIUS = 0.110
CROWN_THICKNESS = 0.028
CROWN_CENTER_Z = POST_CENTER_Z + (POST_LENGTH * 0.5) + (CROWN_THICKNESS * 0.5)

BEZEL_RADIUS = 0.050
BEZEL_THICKNESS = 0.012
BEZEL_CENTER_Z = CROWN_CENTER_Z + (CROWN_THICKNESS * 0.5) + (BEZEL_THICKNESS * 0.5) - 0.001

LIGHT_RADIUS = 0.034
LIGHT_THICKNESS = 0.010
LIGHT_CENTER_Z = BEZEL_CENTER_Z + 0.006

HINGE_X = -0.042
HINGE_Z = 0.190
HINGE_BARREL_RADIUS = 0.004
HINGE_BARREL_LENGTH = 0.086
HINGE_PLATE_X = -0.050
HINGE_PLATE_THICKNESS = 0.008
HINGE_PLATE_HEIGHT = 0.020

COVER_OPEN_ANGLE = 1.35


def _sleeve_shape() -> cq.Workplane:
    sleeve_tube = (
        cq.Workplane("XY")
        .circle(SLEEVE_OUTER_RADIUS)
        .extrude(SLEEVE_DEPTH)
        .cut(
            cq.Workplane("XY")
            .circle(SLEEVE_INNER_RADIUS)
            .extrude(SLEEVE_DEPTH + 0.002)
            .translate((0.0, 0.0, -0.001))
        )
    )

    top_guide = (
        cq.Workplane("XY")
        .circle(SLEEVE_INNER_RADIUS)
        .extrude(0.085)
        .cut(
            cq.Workplane("XY")
            .circle(POST_RADIUS + 0.007)
            .extrude(0.087)
            .translate((0.0, 0.0, -0.001))
        )
        .translate((0.0, 0.0, SLEEVE_DEPTH - 0.085))
    )

    flange = (
        cq.Workplane("XY")
        .circle(SLEEVE_FLANGE_RADIUS)
        .extrude(SLEEVE_FLANGE_THICKNESS)
        .cut(
            cq.Workplane("XY")
            .circle(SLEEVE_INNER_RADIUS + 0.002)
            .extrude(SLEEVE_FLANGE_THICKNESS + 0.002)
            .translate((0.0, 0.0, -0.001))
        )
        .translate((0.0, 0.0, SLEEVE_DEPTH))
    )

    return sleeve_tube.union(top_guide).union(flange)


def _cover_shell_shape() -> cq.Workplane:
    shell_outer = (
        cq.Workplane("XY")
        .box(0.092, 0.084, 0.036, centered=(False, True, False))
        .translate((0.006, 0.0, 0.008))
    )
    shell_inner = (
        cq.Workplane("XY")
        .box(0.084, 0.076, 0.040, centered=(False, True, False))
        .translate((0.010, 0.0, -0.002))
    )
    rear_tongue = (
        cq.Workplane("XY")
        .box(0.014, 0.084, 0.012, centered=(False, True, False))
        .translate((-0.002, 0.0, 0.002))
    )
    return shell_outer.cut(shell_inner).union(rear_tongue)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retractable_security_bollard")

    sleeve_steel = model.material("sleeve_steel", rgba=(0.30, 0.32, 0.34, 1.0))
    trim_steel = model.material("trim_steel", rgba=(0.63, 0.65, 0.67, 1.0))
    post_steel = model.material("post_steel", rgba=(0.76, 0.78, 0.80, 1.0))
    crown_black = model.material("crown_black", rgba=(0.10, 0.11, 0.12, 1.0))
    lens_red = model.material("lens_red", rgba=(0.86, 0.12, 0.12, 0.90))
    cover_amber = model.material("cover_amber", rgba=(0.84, 0.55, 0.10, 0.55))

    sleeve = model.part("sleeve")
    sleeve.visual(
        mesh_from_cadquery(_sleeve_shape(), "sleeve_body"),
        origin=Origin(xyz=(0.0, 0.0, -SLEEVE_DEPTH)),
        material=sleeve_steel,
        name="sleeve_body",
    )
    sleeve.visual(
        Cylinder(radius=SLEEVE_FLANGE_RADIUS, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, SLEEVE_FLANGE_THICKNESS + 0.002)),
        material=trim_steel,
        name="collar_trim",
    )

    post = model.part("post")
    post.visual(
        Cylinder(radius=POST_RADIUS, length=POST_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, POST_CENTER_Z)),
        material=post_steel,
        name="post_body",
    )
    post.visual(
        Cylinder(radius=CROWN_RADIUS, length=CROWN_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, CROWN_CENTER_Z)),
        material=crown_black,
        name="crown",
    )
    post.visual(
        Cylinder(radius=BEZEL_RADIUS, length=BEZEL_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, BEZEL_CENTER_Z)),
        material=crown_black,
        name="light_bezel",
    )
    post.visual(
        Cylinder(radius=LIGHT_RADIUS, length=LIGHT_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, LIGHT_CENTER_Z)),
        material=lens_red,
        name="warning_light",
    )
    post.visual(
        Box((HINGE_PLATE_THICKNESS, HINGE_BARREL_LENGTH, HINGE_PLATE_HEIGHT)),
        origin=Origin(xyz=(HINGE_PLATE_X, 0.0, HINGE_Z)),
        material=crown_black,
        name="hinge_plate",
    )

    light_cover = model.part("light_cover")
    light_cover.visual(
        mesh_from_cadquery(_cover_shell_shape(), "light_cover"),
        material=cover_amber,
        name="hood_shell",
    )
    light_cover.visual(
        Cylinder(radius=HINGE_BARREL_RADIUS, length=HINGE_BARREL_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=crown_black,
        name="hinge_barrel",
    )

    model.articulation(
        "post_slide",
        ArticulationType.PRISMATIC,
        parent=sleeve,
        child=post,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=POST_TRAVEL,
            effort=2500.0,
            velocity=0.18,
        ),
    )
    model.articulation(
        "cover_hinge",
        ArticulationType.REVOLUTE,
        parent=post,
        child=light_cover,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=COVER_OPEN_ANGLE,
            effort=2.0,
            velocity=2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    sleeve = object_model.get_part("sleeve")
    post = object_model.get_part("post")
    light_cover = object_model.get_part("light_cover")
    post_slide = object_model.get_articulation("post_slide")
    cover_hinge = object_model.get_articulation("cover_hinge")

    with ctx.pose({post_slide: 0.0, cover_hinge: 0.0}):
        ctx.expect_overlap(
            post,
            sleeve,
            axes="xy",
            elem_a="post_body",
            elem_b="sleeve_body",
            min_overlap=0.20,
            name="post stays centered over the sleeve opening",
        )
        ctx.expect_gap(
            light_cover,
            post,
            axis="x",
            positive_elem="hinge_barrel",
            negative_elem="hinge_plate",
            max_gap=0.001,
            max_penetration=0.0,
            name="cover hinge barrel seats against the crown hinge plate",
        )
        ctx.expect_overlap(
            light_cover,
            post,
            axes="xy",
            elem_a="hood_shell",
            elem_b="warning_light",
            min_overlap=0.05,
            name="closed cover stays over the warning light",
        )

    rest_post_pos = ctx.part_world_position(post)
    with ctx.pose({post_slide: POST_TRAVEL, cover_hinge: 0.0}):
        ctx.expect_overlap(
            post,
            sleeve,
            axes="xy",
            elem_a="post_body",
            elem_b="sleeve_body",
            min_overlap=0.20,
            name="raised post remains aligned with the sleeve",
        )
        ctx.expect_overlap(
            post,
            sleeve,
            axes="z",
            elem_a="post_body",
            elem_b="sleeve_body",
            min_overlap=0.24,
            name="raised post keeps visible retained insertion in the sleeve",
        )
        extended_post_pos = ctx.part_world_position(post)

    ctx.check(
        "post extends upward from the sleeve",
        rest_post_pos is not None
        and extended_post_pos is not None
        and extended_post_pos[2] > rest_post_pos[2] + 0.60,
        details=f"rest={rest_post_pos}, extended={extended_post_pos}",
    )

    closed_hood_aabb = None
    open_hood_aabb = None
    light_aabb = None
    with ctx.pose({post_slide: POST_TRAVEL, cover_hinge: 0.0}):
        closed_hood_aabb = ctx.part_element_world_aabb(light_cover, elem="hood_shell")
        light_aabb = ctx.part_element_world_aabb(post, elem="warning_light")
    with ctx.pose({post_slide: POST_TRAVEL, cover_hinge: COVER_OPEN_ANGLE}):
        open_hood_aabb = ctx.part_element_world_aabb(light_cover, elem="hood_shell")

    closed_light_center = None
    if light_aabb is not None:
        closed_light_center = tuple(
            0.5 * (light_aabb[0][axis] + light_aabb[1][axis]) for axis in range(3)
        )

    ctx.check(
        "closed hood spans the warning light footprint",
        closed_hood_aabb is not None
        and closed_light_center is not None
        and closed_hood_aabb[0][0] <= closed_light_center[0] <= closed_hood_aabb[1][0]
        and closed_hood_aabb[0][1] <= closed_light_center[1] <= closed_hood_aabb[1][1]
        and closed_hood_aabb[1][2] > light_aabb[1][2],
        details=f"hood={closed_hood_aabb}, light={light_aabb}",
    )

    ctx.check(
        "cover opens upward",
        closed_hood_aabb is not None
        and open_hood_aabb is not None
        and open_hood_aabb[1][2] > closed_hood_aabb[1][2] + 0.045,
        details=f"closed={closed_hood_aabb}, open={open_hood_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
