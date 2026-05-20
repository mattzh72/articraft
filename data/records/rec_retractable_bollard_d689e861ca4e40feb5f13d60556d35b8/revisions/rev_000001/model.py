from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


POST_RADIUS = 0.102
POST_VISIBLE_HEIGHT = 0.92
POST_HIDDEN_DEPTH = 0.40
POST_TOTAL_LENGTH = POST_VISIBLE_HEIGHT + POST_HIDDEN_DEPTH
POST_TRAVEL = 0.95

SLEEVE_INNER_RADIUS = 0.112
SLEEVE_OUTER_RADIUS = 0.145
SLEEVE_DEPTH = 1.42
COLLAR_OUTER_RADIUS = 0.168
COLLAR_HEIGHT = 0.028

FLAP_HINGE_X = COLLAR_OUTER_RADIUS + 0.012
FLAP_HINGE_Z = 0.010
FLAP_THICKNESS = 0.010
FLAP_WIDTH = 0.190
FLAP_LENGTH = 0.304
FLAP_KNUCKLE_RADIUS = 0.011
FLAP_CHILD_KNUCKLE_LEN = 0.034
FLAP_PARENT_KNUCKLE_LEN = 0.026
FLAP_KNUCKLE_GAP = 0.004

LIGHT_HINGE_Y = -0.028
LIGHT_HINGE_Z = POST_VISIBLE_HEIGHT + 0.018
LIGHT_KNUCKLE_RADIUS = 0.006
LIGHT_CHILD_KNUCKLE_LEN = 0.024
LIGHT_PARENT_KNUCKLE_LEN = 0.016
LIGHT_COVER_RADIUS = 0.042
LIGHT_COVER_THICKNESS = 0.010
LIGHT_COVER_OFFSET_Y = 0.050
SUPPORT_SHOULDER_RADIUS = 0.109
SUPPORT_SHOULDER_HEIGHT = 0.015
SUPPORT_SHOULDER_TOP_Z = -0.032


def _make_sleeve_shape() -> cq.Workplane:
    flange = (
        cq.Workplane("XY")
        .circle(COLLAR_OUTER_RADIUS)
        .circle(SLEEVE_INNER_RADIUS)
        .extrude(COLLAR_HEIGHT)
        .translate((0.0, 0.0, -COLLAR_HEIGHT))
    )
    tube = (
        cq.Workplane("XY")
        .circle(SLEEVE_OUTER_RADIUS)
        .circle(SLEEVE_INNER_RADIUS)
        .extrude(SLEEVE_DEPTH)
        .translate((0.0, 0.0, -SLEEVE_DEPTH))
    )
    support_ledge = (
        cq.Workplane("XY")
        .circle(SLEEVE_OUTER_RADIUS)
        .circle(SUPPORT_SHOULDER_RADIUS)
        .extrude(0.016)
        .translate((0.0, 0.0, SUPPORT_SHOULDER_TOP_Z))
    )
    barrel_spacing = (FLAP_CHILD_KNUCKLE_LEN / 2.0) + (FLAP_PARENT_KNUCKLE_LEN / 2.0) + (FLAP_KNUCKLE_GAP / 2.0)
    ear_a = (
        cq.Workplane("XY")
        .box(0.060, 0.030, 0.018, centered=(True, True, False))
        .translate((FLAP_HINGE_X + 0.012, -barrel_spacing, -0.004))
    )
    ear_b = (
        cq.Workplane("XY")
        .box(0.060, 0.030, 0.018, centered=(True, True, False))
        .translate((FLAP_HINGE_X + 0.012, barrel_spacing, -0.004))
    )
    barrel_a = (
        cq.Workplane("XY")
        .circle(FLAP_KNUCKLE_RADIUS)
        .extrude(FLAP_PARENT_KNUCKLE_LEN)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -90.0)
        .translate(
            (
                FLAP_HINGE_X,
                -barrel_spacing - (FLAP_PARENT_KNUCKLE_LEN / 2.0),
                FLAP_HINGE_Z,
            )
        )
    )
    barrel_b = (
        cq.Workplane("XY")
        .circle(FLAP_KNUCKLE_RADIUS)
        .extrude(FLAP_PARENT_KNUCKLE_LEN)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -90.0)
        .translate(
            (
                FLAP_HINGE_X,
                barrel_spacing - (FLAP_PARENT_KNUCKLE_LEN / 2.0),
                FLAP_HINGE_Z,
            )
        )
    )
    return flange.union(tube).union(support_ledge).union(ear_a).union(ear_b).union(barrel_a).union(barrel_b)


def _make_post_shape() -> cq.Workplane:
    shaft = (
        cq.Workplane("XY")
        .circle(POST_RADIUS)
        .extrude(POST_TOTAL_LENGTH)
        .translate((0.0, 0.0, -POST_HIDDEN_DEPTH))
    )
    support_shoulder = (
        cq.Workplane("XY")
        .circle(SUPPORT_SHOULDER_RADIUS)
        .circle(POST_RADIUS - 0.003)
        .extrude(SUPPORT_SHOULDER_HEIGHT)
        .translate((0.0, 0.0, SUPPORT_SHOULDER_TOP_Z - SUPPORT_SHOULDER_HEIGHT))
    )
    crown_pad = (
        cq.Workplane("XY")
        .circle(0.056)
        .extrude(0.010)
        .translate((0.0, 0.0, POST_VISIBLE_HEIGHT))
    )
    lug_spacing = (LIGHT_CHILD_KNUCKLE_LEN / 2.0) + (LIGHT_PARENT_KNUCKLE_LEN / 2.0)
    hinge_ear_a = (
        cq.Workplane("XY")
        .box(0.014, 0.024, 0.018, centered=(True, True, False))
        .translate((-lug_spacing, LIGHT_HINGE_Y, POST_VISIBLE_HEIGHT))
    )
    hinge_ear_b = (
        cq.Workplane("XY")
        .box(0.014, 0.024, 0.018, centered=(True, True, False))
        .translate((lug_spacing, LIGHT_HINGE_Y, POST_VISIBLE_HEIGHT))
    )
    lug_a = (
        cq.Workplane("YZ")
        .circle(LIGHT_KNUCKLE_RADIUS)
        .extrude(LIGHT_PARENT_KNUCKLE_LEN)
        .translate(
            (
                -lug_spacing - (LIGHT_PARENT_KNUCKLE_LEN / 2.0),
                LIGHT_HINGE_Y,
                LIGHT_HINGE_Z,
            )
        )
    )
    lug_b = (
        cq.Workplane("YZ")
        .circle(LIGHT_KNUCKLE_RADIUS)
        .extrude(LIGHT_PARENT_KNUCKLE_LEN)
        .translate(
            (
                lug_spacing - (LIGHT_PARENT_KNUCKLE_LEN / 2.0),
                LIGHT_HINGE_Y,
                LIGHT_HINGE_Z,
            )
        )
    )

    return support_shoulder.union(shaft).union(crown_pad).union(hinge_ear_a).union(hinge_ear_b).union(lug_a).union(lug_b)


def _make_flap_shape() -> cq.Workplane:
    knuckle = (
        cq.Workplane("XY")
        .circle(FLAP_KNUCKLE_RADIUS)
        .extrude(FLAP_CHILD_KNUCKLE_LEN)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -90.0)
        .translate((0.0, -(FLAP_CHILD_KNUCKLE_LEN / 2.0), 0.0))
    )
    strap = (
        cq.Workplane("XY")
        .box(FLAP_THICKNESS, 0.030, 0.028, centered=(True, True, False))
        .translate(((FLAP_THICKNESS / 2.0), 0.0, 0.0))
    )
    leaf = (
        cq.Workplane("XY")
        .box(FLAP_THICKNESS, FLAP_WIDTH, FLAP_LENGTH - 0.028, centered=(True, True, False))
        .translate(((FLAP_THICKNESS / 2.0), 0.0, 0.028))
    )
    return knuckle.union(strap).union(leaf)


def _make_light_cover_shape() -> cq.Workplane:
    knuckle = cq.Workplane("YZ").circle(LIGHT_KNUCKLE_RADIUS).extrude(LIGHT_CHILD_KNUCKLE_LEN).translate(
        (-(LIGHT_CHILD_KNUCKLE_LEN / 2.0), 0.0, 0.0)
    )
    bridge = (
        cq.Workplane("XY")
        .box(0.010, 0.012, 0.004, centered=(True, False, False))
        .translate((0.0, 0.0, 0.004))
    )
    cap = (
        cq.Workplane("XY")
        .circle(LIGHT_COVER_RADIUS)
        .extrude(LIGHT_COVER_THICKNESS)
        .translate((0.0, LIGHT_COVER_OFFSET_Y, 0.006))
    )
    return knuckle.union(bridge).union(cap)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retractable_bollard")

    model.material("stainless", rgba=(0.76, 0.78, 0.80, 1.0))
    model.material("sleeve_steel", rgba=(0.58, 0.60, 0.62, 1.0))
    model.material("amber_lens", rgba=(0.96, 0.69, 0.20, 0.72))

    sleeve = model.part("sleeve")
    sleeve.visual(
        mesh_from_cadquery(_make_sleeve_shape(), "sleeve_body"),
        material="sleeve_steel",
        name="sleeve_body",
    )

    post = model.part("post")
    post.visual(
        mesh_from_cadquery(_make_post_shape(), "post_body"),
        material="stainless",
        name="post_body",
    )

    sleeve_flap = model.part("sleeve_flap")
    sleeve_flap.visual(
        mesh_from_cadquery(_make_flap_shape(), "sleeve_flap_leaf"),
        material="stainless",
        name="flap_leaf",
    )

    light_cover = model.part("light_cover")
    light_cover.visual(
        mesh_from_cadquery(_make_light_cover_shape(), "light_cover"),
        material="amber_lens",
        name="light_cover",
    )

    model.articulation(
        "post_slide",
        ArticulationType.PRISMATIC,
        parent=sleeve,
        child=post,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=POST_TRAVEL, effort=1200.0, velocity=0.25),
    )
    model.articulation(
        "sleeve_flap_hinge",
        ArticulationType.REVOLUTE,
        parent=sleeve,
        child=sleeve_flap,
        origin=Origin(xyz=(FLAP_HINGE_X, 0.0, FLAP_HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.53, upper=0.0, effort=60.0, velocity=1.5),
    )
    model.articulation(
        "light_cover_hinge",
        ArticulationType.REVOLUTE,
        parent=post,
        child=light_cover,
        origin=Origin(xyz=(0.0, LIGHT_HINGE_Y, LIGHT_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.10, effort=4.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    sleeve = object_model.get_part("sleeve")
    post = object_model.get_part("post")
    sleeve_flap = object_model.get_part("sleeve_flap")
    light_cover = object_model.get_part("light_cover")
    post_slide = object_model.get_articulation("post_slide")
    sleeve_flap_hinge = object_model.get_articulation("sleeve_flap_hinge")
    light_cover_hinge = object_model.get_articulation("light_cover_hinge")

    ctx.allow_overlap(
        post,
        sleeve,
        elem_a="post_body",
        elem_b="sleeve_body",
        reason="The post uses a simplified internal guide shoulder riding on the sleeve ledge.",
    )
    ctx.allow_overlap(
        sleeve_flap,
        sleeve,
        elem_a="flap_leaf",
        elem_b="sleeve_body",
        reason="The sleeve flap uses a simplified interleaved hinge barrel at the collar edge.",
    )
    ctx.allow_overlap(
        light_cover,
        post,
        elem_a="light_cover",
        elem_b="post_body",
        reason="The warning light cover uses a simplified crown hinge barrel nested into the post hinge ears.",
    )
    ctx.expect_origin_distance(
        post,
        sleeve,
        axes="xy",
        max_dist=0.001,
        name="post stays centered on the sleeve axis",
    )
    ctx.expect_contact(
        light_cover,
        post,
        contact_tol=0.001,
        name="warning light cover stays mounted to the crown hinge",
    )

    extended_post_aabb = ctx.part_element_world_aabb(post, elem="post_body")
    if post_slide.motion_limits is not None and post_slide.motion_limits.upper is not None:
        with ctx.pose({post_slide: post_slide.motion_limits.upper}):
            retracted_post_aabb = ctx.part_element_world_aabb(post, elem="post_body")
            ctx.expect_origin_distance(
                post,
                sleeve,
                axes="xy",
                max_dist=0.001,
                name="retracted post remains centered in the sleeve",
            )
        ctx.check(
            "post retracts below the collar plane",
            extended_post_aabb is not None
            and retracted_post_aabb is not None
            and retracted_post_aabb[1][2] < 0.02
            and retracted_post_aabb[1][2] < extended_post_aabb[1][2] - 0.85,
            details=f"extended={extended_post_aabb}, retracted={retracted_post_aabb}",
        )

        if sleeve_flap_hinge.motion_limits is not None and sleeve_flap_hinge.motion_limits.lower is not None:
            with ctx.pose(
                {
                    post_slide: post_slide.motion_limits.upper,
                    sleeve_flap_hinge: sleeve_flap_hinge.motion_limits.lower,
                }
            ):
                closed_flap_aabb = ctx.part_element_world_aabb(sleeve_flap, elem="flap_leaf")
                ctx.expect_overlap(
                    sleeve_flap,
                    sleeve,
                    axes="xy",
                    min_overlap=0.18,
                    name="closed sleeve flap bridges the pavement opening",
                )
                ctx.check(
                    "closed sleeve flap spans across the sleeve mouth",
                    closed_flap_aabb is not None
                    and closed_flap_aabb[0][0] < -0.08
                    and closed_flap_aabb[1][0] > 0.08
                    and -0.01 < closed_flap_aabb[0][2] < 0.02
                    and 0.0 < closed_flap_aabb[1][2] < 0.05,
                    details=f"closed_flap={closed_flap_aabb}",
                )

    if light_cover_hinge.motion_limits is not None and light_cover_hinge.motion_limits.upper is not None:
        closed_cover_aabb = ctx.part_element_world_aabb(light_cover, elem="light_cover")
        with ctx.pose({light_cover_hinge: light_cover_hinge.motion_limits.upper}):
            open_cover_aabb = ctx.part_element_world_aabb(light_cover, elem="light_cover")
            ctx.expect_contact(
                light_cover,
                post,
                contact_tol=0.001,
                name="opened warning light cover remains hinged to the post",
            )
        ctx.check(
            "warning light cover lifts above the crown when opened",
            closed_cover_aabb is not None
            and open_cover_aabb is not None
            and open_cover_aabb[1][2] > closed_cover_aabb[1][2] + 0.03,
            details=f"closed={closed_cover_aabb}, open={open_cover_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
