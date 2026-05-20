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

COLLAR_HEIGHT = 0.055
SLEEVE_DEPTH = 0.420
SLEEVE_BODY_RADIUS = 0.102
SLEEVE_COLLAR_RADIUS = 0.118
SLEEVE_BORE_RADIUS = 0.0835

POST_RADIUS = 0.078
POST_HIDDEN_LENGTH = 0.380
POST_VISIBLE_LENGTH = 0.880
POST_TOTAL_LENGTH = POST_HIDDEN_LENGTH + POST_VISIBLE_LENGTH
POST_CENTER_Z = (POST_VISIBLE_LENGTH - POST_HIDDEN_LENGTH) / 2.0
POST_TRAVEL = 0.250

SERVICE_BOSS_RADIUS = 0.028
SERVICE_BOSS_LENGTH = 0.022
SERVICE_CAP_RADIUS = 0.022
SERVICE_CAP_LENGTH = 0.010
SERVICE_CAP_Z = 0.620
SERVICE_BOSS_CENTER_X = POST_RADIUS + 0.011
SERVICE_CAP_CENTER_X = SERVICE_BOSS_CENTER_X + SERVICE_BOSS_LENGTH / 2.0 + SERVICE_CAP_LENGTH / 2.0

TOP_CAP_RADIUS = 0.034
TOP_CAP_HEIGHT = 0.018
TOP_CAP_CENTER_Z = POST_VISIBLE_LENGTH + TOP_CAP_HEIGHT / 2.0


def _add_ring_staves(
    part,
    *,
    inner_radius: float,
    outer_radius: float,
    height: float,
    z_min: float,
    count: int,
    material,
    name_prefix: str,
    width_scale: float,
) -> None:
    mid_radius = (inner_radius + outer_radius) / 2.0
    radial_depth = outer_radius - inner_radius
    tangential_width = 2.0 * math.pi * mid_radius / count * width_scale

    for index in range(count):
        angle = 2.0 * math.pi * index / count
        part.visual(
            Box((radial_depth, tangential_width, height)),
            origin=Origin(
                xyz=(
                    mid_radius * math.cos(angle),
                    mid_radius * math.sin(angle),
                    z_min + height / 2.0,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=material,
            name=f"{name_prefix}_{index}",
        )


def _top_cap_shape() -> cq.Workplane:
    slot_depth = 0.0025
    cap = cq.Workplane("XY").circle(TOP_CAP_RADIUS).extrude(TOP_CAP_HEIGHT)
    slot = (
        cq.Workplane("XY")
        .rect(0.022, 0.0045)
        .extrude(slot_depth)
        .translate((0.0, 0.0, TOP_CAP_HEIGHT - slot_depth))
    )
    return cap.cut(slot).translate((0.0, 0.0, -TOP_CAP_HEIGHT / 2.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_security_post")

    sleeve_dark = model.material("sleeve_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    post_steel = model.material("post_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    cap_black = model.material("cap_black", rgba=(0.10, 0.11, 0.12, 1.0))
    brass = model.material("brass", rgba=(0.63, 0.53, 0.28, 1.0))

    sleeve = model.part("sleeve")
    _add_ring_staves(
        sleeve,
        inner_radius=SLEEVE_BORE_RADIUS,
        outer_radius=SLEEVE_BODY_RADIUS,
        height=SLEEVE_DEPTH,
        z_min=-SLEEVE_DEPTH,
        count=40,
        material=sleeve_dark,
        name_prefix="body_stave",
        width_scale=0.98,
    )
    _add_ring_staves(
        sleeve,
        inner_radius=POST_RADIUS,
        outer_radius=SLEEVE_COLLAR_RADIUS,
        height=COLLAR_HEIGHT + 0.005,
        z_min=-0.005,
        count=36,
        material=sleeve_dark,
        name_prefix="collar_stave",
        width_scale=1.02,
    )
    _add_ring_staves(
        sleeve,
        inner_radius=POST_RADIUS,
        outer_radius=SLEEVE_COLLAR_RADIUS + 0.010,
        height=0.012,
        z_min=0.0,
        count=40,
        material=sleeve_dark,
        name_prefix="flange_stave",
        width_scale=1.02,
    )

    post = model.part("post")
    post.visual(
        Cylinder(radius=POST_RADIUS, length=POST_TOTAL_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, POST_CENTER_Z)),
        material=post_steel,
        name="post_tube",
    )
    post.visual(
        Cylinder(radius=POST_RADIUS + 0.004, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.790)),
        material=post_steel,
        name="top_band",
    )
    post.visual(
        Cylinder(radius=SERVICE_BOSS_RADIUS, length=SERVICE_BOSS_LENGTH),
        origin=Origin(
            xyz=(SERVICE_BOSS_CENTER_X, 0.0, SERVICE_CAP_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=sleeve_dark,
        name="boss_housing",
    )
    post.visual(
        Box((0.014, 0.056, 0.090)),
        origin=Origin(xyz=(POST_RADIUS + 0.004, 0.0, SERVICE_CAP_Z)),
        material=sleeve_dark,
        name="boss_backplate",
    )

    service_cap = model.part("service_cap")
    service_cap.visual(
        Cylinder(radius=SERVICE_CAP_RADIUS, length=SERVICE_CAP_LENGTH),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cap_black,
        name="cap_body",
    )
    service_cap.visual(
        Box((0.0015, 0.018, 0.004)),
        origin=Origin(xyz=(SERVICE_CAP_LENGTH / 2.0, 0.0, 0.0)),
        material=brass,
        name="cap_key",
    )

    top_cap = model.part("top_cap")
    top_cap.visual(
        mesh_from_cadquery(_top_cap_shape(), "security_post_top_cap"),
        material=cap_black,
        name="cap_body",
    )

    model.articulation(
        "sleeve_to_post",
        ArticulationType.PRISMATIC,
        parent=sleeve,
        child=post,
        origin=Origin(xyz=(0.0, 0.0, COLLAR_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=800.0,
            velocity=0.18,
            lower=0.0,
            upper=POST_TRAVEL,
        ),
    )
    model.articulation(
        "post_to_service_cap",
        ArticulationType.CONTINUOUS,
        parent=post,
        child=service_cap,
        origin=Origin(xyz=(SERVICE_CAP_CENTER_X, 0.0, SERVICE_CAP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=6.0),
    )
    model.articulation(
        "post_to_top_cap",
        ArticulationType.CONTINUOUS,
        parent=post,
        child=top_cap,
        origin=Origin(xyz=(0.0, 0.0, TOP_CAP_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    sleeve = object_model.get_part("sleeve")
    post = object_model.get_part("post")
    service_cap = object_model.get_part("service_cap")
    top_cap = object_model.get_part("top_cap")
    sleeve_to_post = object_model.get_articulation("sleeve_to_post")
    post_to_service_cap = object_model.get_articulation("post_to_service_cap")
    post_to_top_cap = object_model.get_articulation("post_to_top_cap")

    ctx.expect_overlap(
        post,
        sleeve,
        axes="xy",
        elem_a="post_tube",
        min_overlap=0.150,
        name="post stays centered in the sleeve collar at rest",
    )
    ctx.expect_overlap(
        post,
        sleeve,
        axes="z",
        elem_a="post_tube",
        min_overlap=0.360,
        name="collapsed post remains deeply retained in the sleeve",
    )
    ctx.expect_gap(
        service_cap,
        post,
        axis="x",
        positive_elem="cap_body",
        negative_elem="boss_housing",
        max_gap=0.0005,
        max_penetration=0.0,
        name="service cap seats against its side housing",
    )
    ctx.expect_gap(
        top_cap,
        post,
        axis="z",
        positive_elem="cap_body",
        negative_elem="post_tube",
        max_gap=0.0005,
        max_penetration=0.0,
        name="keyed top cap sits on the post crown",
    )

    rest_post_pos = ctx.part_world_position(post)
    rest_service_pos = ctx.part_world_position(service_cap)
    rest_top_pos = ctx.part_world_position(top_cap)
    upper_limit = sleeve_to_post.motion_limits.upper if sleeve_to_post.motion_limits is not None else POST_TRAVEL

    with ctx.pose({sleeve_to_post: upper_limit}):
        ctx.expect_overlap(
            post,
            sleeve,
            axes="xy",
            elem_a="post_tube",
            min_overlap=0.150,
            name="extended post stays centered in the sleeve collar",
        )
        ctx.expect_overlap(
            post,
            sleeve,
            axes="z",
            elem_a="post_tube",
            min_overlap=0.120,
            name="extended post still retains insertion in the sleeve",
        )
        ctx.expect_gap(
            service_cap,
            post,
            axis="x",
            positive_elem="cap_body",
            negative_elem="boss_housing",
            max_gap=0.0005,
            max_penetration=0.0,
            name="service cap stays seated at full extension",
        )
        ctx.expect_gap(
            top_cap,
            post,
            axis="z",
            positive_elem="cap_body",
            negative_elem="post_tube",
            max_gap=0.0005,
            max_penetration=0.0,
            name="top cap stays seated at full extension",
        )
        extended_post_pos = ctx.part_world_position(post)

    with ctx.pose({post_to_service_cap: 1.35, post_to_top_cap: 0.90}):
        ctx.expect_gap(
            service_cap,
            post,
            axis="x",
            positive_elem="cap_body",
            negative_elem="boss_housing",
            max_gap=0.0005,
            max_penetration=0.0,
            name="service cap stays seated while spun",
        )
        ctx.expect_gap(
            top_cap,
            post,
            axis="z",
            positive_elem="cap_body",
            negative_elem="post_tube",
            max_gap=0.0005,
            max_penetration=0.0,
            name="top cap stays seated while spun",
        )
        rotated_service_pos = ctx.part_world_position(service_cap)
        rotated_top_pos = ctx.part_world_position(top_cap)

    ctx.check(
        "post extends upward",
        rest_post_pos is not None
        and extended_post_pos is not None
        and extended_post_pos[2] > rest_post_pos[2] + 0.20,
        details=f"rest={rest_post_pos}, extended={extended_post_pos}",
    )
    ctx.check(
        "service cap spins in place",
        rest_service_pos is not None
        and rotated_service_pos is not None
        and max(abs(a - b) for a, b in zip(rest_service_pos, rotated_service_pos)) < 1e-6,
        details=f"rest={rest_service_pos}, rotated={rotated_service_pos}",
    )
    ctx.check(
        "top cap spins in place",
        rest_top_pos is not None
        and rotated_top_pos is not None
        and max(abs(a - b) for a, b in zip(rest_top_pos, rotated_top_pos)) < 1e-6,
        details=f"rest={rest_top_pos}, rotated={rotated_top_pos}",
    )

    return ctx.report()


object_model = build_object_model()
