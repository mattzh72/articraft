from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


SLEEVE_DEPTH = 1.00
SLEEVE_OUTER_RADIUS = 0.145
SLEEVE_INNER_RADIUS = 0.120
FLANGE_OUTER_RADIUS = 0.190
FLANGE_INNER_RADIUS = 0.118
FLANGE_THICKNESS = 0.030
GUIDE_HEIGHT = 0.065
GUIDE_WIDTH = 0.034

POST_RADIUS = 0.110
POST_INSERTION_AT_FULL_HEIGHT = 0.240
POST_EXPOSED_AT_FULL_HEIGHT = 0.760
POST_TOTAL_LENGTH = POST_INSERTION_AT_FULL_HEIGHT + POST_EXPOSED_AT_FULL_HEIGHT
POST_TRAVEL = 0.700

KEY_CAP_RADIUS = 0.034
KEY_CAP_HEIGHT = 0.014
KEY_SLOT_LENGTH = 0.026
KEY_SLOT_WIDTH = 0.006
KEY_SLOT_DEPTH = 0.004


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retractable_security_bollard")

    model.material("sleeve_steel", rgba=(0.30, 0.32, 0.34, 1.0))
    model.material("post_steel", rgba=(0.76, 0.79, 0.82, 1.0))
    model.material("lock_cap", rgba=(0.17, 0.18, 0.19, 1.0))

    sleeve = model.part("sleeve")
    sleeve.visual(
        mesh_from_cadquery(_sleeve_shape(), "sleeve_housing_v2"),
        material="sleeve_steel",
        name="sleeve_housing",
    )

    post = model.part("post")
    post.visual(
        mesh_from_cadquery(_post_shape(), "post_body_v2"),
        material="post_steel",
        name="post_body",
    )

    key_cap = model.part("key_cap")
    key_cap.visual(
        mesh_from_cadquery(_key_cap_shape(), "key_cap_v1"),
        material="lock_cap",
        name="key_cap",
    )

    model.articulation(
        "sleeve_to_post",
        ArticulationType.PRISMATIC,
        parent=sleeve,
        child=post,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2500.0,
            velocity=0.18,
            lower=-POST_TRAVEL,
            upper=0.0,
        ),
    )

    model.articulation(
        "post_to_key_cap",
        ArticulationType.REVOLUTE,
        parent=post,
        child=key_cap,
        origin=Origin(xyz=(0.0, 0.0, POST_EXPOSED_AT_FULL_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=4.0,
            lower=-0.75,
            upper=0.75,
        ),
    )

    return model


def _sleeve_shape() -> cq.Workplane:
    tube = cq.Workplane("XY").circle(SLEEVE_OUTER_RADIUS).extrude(-SLEEVE_DEPTH)
    inner_bore = (
        cq.Workplane("XY")
        .circle(SLEEVE_INNER_RADIUS)
        .extrude(-(SLEEVE_DEPTH + 0.060))
        .translate((0.0, 0.0, 0.030))
    )
    tube = tube.cut(inner_bore)
    drain_slot = (
        cq.Workplane("XY")
        .box(0.320, 0.090, SLEEVE_DEPTH + 0.040)
        .translate((0.0, 0.0, -SLEEVE_DEPTH / 2.0))
    )
    for angle_deg in (0.0, 120.0, 240.0):
        tube = tube.cut(drain_slot.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg))
    flange = (
        cq.Workplane("XY")
        .circle(FLANGE_OUTER_RADIUS)
        .extrude(FLANGE_THICKNESS)
        .translate((0.0, 0.0, -FLANGE_THICKNESS / 2.0))
        .cut(
            cq.Workplane("XY")
            .circle(FLANGE_INNER_RADIUS)
            .extrude(FLANGE_THICKNESS + 0.020)
            .translate((0.0, 0.0, -FLANGE_THICKNESS / 2.0 - 0.010))
        )
    )
    guide_pad = (
        cq.Workplane("XY")
        .box(SLEEVE_INNER_RADIUS - POST_RADIUS, GUIDE_WIDTH, GUIDE_HEIGHT)
        .translate(
            (
                (POST_RADIUS + SLEEVE_INNER_RADIUS) / 2.0,
                0.0,
                -GUIDE_HEIGHT / 2.0,
            )
        )
    )
    guide_cluster = guide_pad
    for angle_deg in (120.0, 240.0):
        guide_cluster = guide_cluster.union(
            guide_pad.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
        )
    return tube.union(flange).union(guide_cluster).edges("%CIRCLE and >Z").fillet(0.004)


def _post_shape() -> cq.Workplane:
    post = (
        cq.Workplane("XY")
        .circle(POST_RADIUS)
        .extrude(POST_TOTAL_LENGTH)
        .translate((0.0, 0.0, -POST_INSERTION_AT_FULL_HEIGHT))
    )
    return post.faces(">Z").edges().fillet(0.012)


def _key_cap_shape() -> cq.Workplane:
    cap = cq.Workplane("XY").circle(KEY_CAP_RADIUS).extrude(KEY_CAP_HEIGHT)
    slot = (
        cq.Workplane("XY")
        .box(KEY_SLOT_LENGTH, KEY_SLOT_WIDTH, KEY_SLOT_DEPTH + 0.002)
        .translate((0.0, 0.0, KEY_CAP_HEIGHT - KEY_SLOT_DEPTH / 2.0))
    )
    return cap.cut(slot).faces(">Z").edges().fillet(0.002)


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    sleeve = object_model.get_part("sleeve")
    post = object_model.get_part("post")
    key_cap = object_model.get_part("key_cap")
    slide = object_model.get_articulation("sleeve_to_post")
    cap_rotate = object_model.get_articulation("post_to_key_cap")
    limits = slide.motion_limits
    cap_limits = cap_rotate.motion_limits

    ctx.allow_overlap(
        sleeve,
        post,
        elem_a="sleeve_housing",
        elem_b="post_body",
        reason="The sliding bollard rides within a simplified sleeve proxy; centering and retained insertion are verified explicitly.",
    )

    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({slide: limits.upper}):
            ctx.expect_within(
                post,
                sleeve,
                axes="xy",
                inner_elem="post_body",
                outer_elem="sleeve_housing",
                margin=0.0,
                name="raised post stays centered in sleeve",
            )
            ctx.expect_overlap(
                post,
                sleeve,
                axes="z",
                elem_a="post_body",
                elem_b="sleeve_housing",
                min_overlap=0.24,
                name="raised post keeps retained insertion",
            )
            sleeve_aabb = ctx.part_element_world_aabb(sleeve, elem="sleeve_housing")
            post_aabb = ctx.part_element_world_aabb(post, elem="post_body")
            ctx.check(
                "raised bollard stands clearly above collar",
                sleeve_aabb is not None
                and post_aabb is not None
                and post_aabb[1][2] > sleeve_aabb[1][2] + 0.70,
                details=f"sleeve={sleeve_aabb}, post={post_aabb}",
            )

        with ctx.pose({slide: limits.lower}):
            ctx.expect_within(
                post,
                sleeve,
                axes="xy",
                inner_elem="post_body",
                outer_elem="sleeve_housing",
                margin=0.0,
                name="retracted post stays centered in sleeve",
            )
            ctx.expect_overlap(
                post,
                sleeve,
                axes="z",
                elem_a="post_body",
                elem_b="sleeve_housing",
                min_overlap=0.90,
                name="retracted post remains deeply inserted",
            )
            sleeve_aabb = ctx.part_element_world_aabb(sleeve, elem="sleeve_housing")
            post_aabb = ctx.part_element_world_aabb(post, elem="post_body")
            ctx.check(
                "retracted bollard nests near flange height",
                sleeve_aabb is not None
                and post_aabb is not None
                and 0.02 <= post_aabb[1][2] - sleeve_aabb[1][2] <= 0.10,
                details=f"sleeve={sleeve_aabb}, post={post_aabb}",
            )

        retracted_position = None
        raised_position = None
        with ctx.pose({slide: limits.lower}):
            retracted_position = ctx.part_world_position(post)
        with ctx.pose({slide: limits.upper}):
            raised_position = ctx.part_world_position(post)
        ctx.check(
            "post rises upward through prismatic travel",
            retracted_position is not None
            and raised_position is not None
            and raised_position[2] > retracted_position[2] + 0.65,
            details=f"retracted={retracted_position}, raised={raised_position}",
        )

    ctx.expect_gap(
        key_cap,
        post,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="key_cap",
        negative_elem="post_body",
        name="key cap seats on top of the bollard post",
    )
    ctx.expect_within(
        key_cap,
        post,
        axes="xy",
        inner_elem="key_cap",
        outer_elem="post_body",
        margin=0.0,
        name="key cap stays centered over the post top",
    )

    if cap_limits is not None and cap_limits.lower is not None and cap_limits.upper is not None:
        rest_cap_position = None
        turned_cap_position = None
        with ctx.pose({cap_rotate: cap_limits.lower}):
            rest_cap_position = ctx.part_world_position(key_cap)
        with ctx.pose({cap_rotate: cap_limits.upper}):
            turned_cap_position = ctx.part_world_position(key_cap)
            ctx.expect_gap(
                key_cap,
                post,
                axis="z",
                max_gap=0.001,
                max_penetration=0.0,
                positive_elem="key_cap",
                negative_elem="post_body",
                name="rotated key cap still seats on the bollard post",
            )
        ctx.check(
            "key cap rotates in place about a vertical lock axis",
            rest_cap_position is not None
            and turned_cap_position is not None
            and abs(rest_cap_position[0] - turned_cap_position[0]) <= 0.001
            and abs(rest_cap_position[1] - turned_cap_position[1]) <= 0.001
            and abs(rest_cap_position[2] - turned_cap_position[2]) <= 0.001,
            details=f"rest={rest_cap_position}, turned={turned_cap_position}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
