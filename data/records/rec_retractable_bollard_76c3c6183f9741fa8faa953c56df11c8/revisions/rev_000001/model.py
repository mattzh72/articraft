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


POST_RADIUS = 0.100
POST_LENGTH = 1.580
POST_CENTER_Z = 0.110

CROWN_RADIUS = 0.103
CROWN_THICKNESS = 0.028
CROWN_CENTER_Z = 0.897
CROWN_TOP_Z = CROWN_CENTER_Z + (CROWN_THICKNESS / 2.0)

CAP_SEAT_RADIUS = 0.045
CAP_SEAT_THICKNESS = 0.006
CAP_SEAT_CENTER_Z = CROWN_TOP_Z + (CAP_SEAT_THICKNESS / 2.0)
CAP_SEAT_TOP_Z = CAP_SEAT_CENTER_Z + (CAP_SEAT_THICKNESS / 2.0)

SLEEVE_INNER_RADIUS = 0.111
SLEEVE_OUTER_RADIUS = 0.126
SLEEVE_COLLAR_RADIUS = 0.145
SLEEVE_DEPTH = 1.700
SLEEVE_COLLAR_HEIGHT = 0.014
SLEEVE_COLLAR_SKIRT = 0.050
SLEEVE_DRAIN_THICKNESS = 0.012
SLEEVE_DRAIN_RADIUS = 0.030

POST_TRAVEL = 0.900

CAP_FLANGE_RADIUS = 0.038
CAP_FLANGE_HEIGHT = 0.004
CAP_BODY_RADIUS = 0.034
CAP_BODY_HEIGHT = 0.011
CAP_TOTAL_HEIGHT = CAP_FLANGE_HEIGHT + CAP_BODY_HEIGHT
CAP_SLOT_LENGTH = 0.018
CAP_SLOT_WIDTH = 0.004
CAP_SLOT_DEPTH = 0.0045


def _sleeve_shape() -> cq.Workplane:
    tube = (
        cq.Workplane("XY")
        .circle(SLEEVE_OUTER_RADIUS)
        .circle(SLEEVE_INNER_RADIUS)
        .extrude(-SLEEVE_DEPTH)
    )
    collar = (
        cq.Workplane("XY")
        .circle(SLEEVE_COLLAR_RADIUS)
        .circle(SLEEVE_INNER_RADIUS)
        .extrude(SLEEVE_COLLAR_HEIGHT)
    )
    collar_skirt = (
        cq.Workplane("XY")
        .circle(SLEEVE_COLLAR_RADIUS)
        .circle(SLEEVE_OUTER_RADIUS - 0.001)
        .extrude(-SLEEVE_COLLAR_SKIRT)
    )
    drain_plate = (
        cq.Workplane("XY")
        .circle(SLEEVE_OUTER_RADIUS)
        .circle(SLEEVE_DRAIN_RADIUS)
        .extrude(SLEEVE_DRAIN_THICKNESS)
        .translate((0.0, 0.0, -SLEEVE_DEPTH))
    )
    return tube.union(collar).union(collar_skirt).union(drain_plate)


def _keyed_cap_shape() -> cq.Workplane:
    cap = cq.Workplane("XY").circle(CAP_FLANGE_RADIUS).extrude(CAP_FLANGE_HEIGHT)
    cap = cap.faces(">Z").workplane().circle(CAP_BODY_RADIUS).extrude(CAP_BODY_HEIGHT)
    slot = (
        cq.Workplane("XY")
        .rect(CAP_SLOT_LENGTH, CAP_SLOT_WIDTH)
        .extrude(CAP_SLOT_DEPTH)
        .translate((0.0, 0.0, CAP_TOTAL_HEIGHT - CAP_SLOT_DEPTH))
    )
    key_bore = (
        cq.Workplane("XY")
        .circle(0.003)
        .extrude(CAP_SLOT_DEPTH)
        .translate((0.0, 0.0, CAP_TOTAL_HEIGHT - CAP_SLOT_DEPTH))
    )
    return cap.cut(slot).cut(key_bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flush_mount_bollard")

    galvanized_steel = model.material("galvanized_steel", rgba=(0.60, 0.62, 0.65, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.25, 0.27, 0.30, 1.0))
    brushed_stainless = model.material("brushed_stainless", rgba=(0.76, 0.78, 0.80, 1.0))
    beacon_amber = model.material("beacon_amber", rgba=(0.96, 0.62, 0.16, 0.95))
    smoked_cover = model.material("smoked_cover", rgba=(0.70, 0.76, 0.80, 0.55))

    sleeve = model.part("sleeve")
    sleeve.visual(
        mesh_from_cadquery(_sleeve_shape(), "bollard_sleeve"),
        material=dark_steel,
        name="sleeve_body",
    )

    post = model.part("post")
    post.visual(
        Cylinder(radius=POST_RADIUS, length=POST_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, POST_CENTER_Z)),
        material=galvanized_steel,
        name="post_body",
    )
    post.visual(
        Cylinder(radius=SLEEVE_INNER_RADIUS, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, -0.420)),
        material=dark_steel,
        name="guide_shoe",
    )
    post.visual(
        Cylinder(radius=CROWN_RADIUS, length=CROWN_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, CROWN_CENTER_Z)),
        material=dark_steel,
        name="crown_plate",
    )
    post.visual(
        Cylinder(radius=CAP_SEAT_RADIUS, length=CAP_SEAT_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, CAP_SEAT_CENTER_Z)),
        material=dark_steel,
        name="cap_seat",
    )
    post.visual(
        Box((0.016, 0.050, 0.018)),
        origin=Origin(xyz=(0.050, 0.0, 0.920)),
        material=dark_steel,
        name="cover_mount",
    )
    post.visual(
        Cylinder(radius=0.022, length=0.006),
        origin=Origin(xyz=(0.070, 0.0, 0.914)),
        material=dark_steel,
        name="warning_base",
    )
    post.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.070, 0.0, 0.922)),
        material=beacon_amber,
        name="warning_lens",
    )

    cap = model.part("cap")
    cap.visual(
        mesh_from_cadquery(_keyed_cap_shape(), "bollard_cap"),
        material=brushed_stainless,
        name="keyed_cap",
    )

    cover = model.part("cover")
    cover.visual(
        Cylinder(radius=0.004, length=0.048),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_stainless,
        name="hinge_barrel",
    )
    cover.visual(
        Box((0.018, 0.042, 0.004)),
        origin=Origin(xyz=(0.009, 0.0, 0.002)),
        material=brushed_stainless,
        name="hinge_leaf",
    )
    cover.visual(
        Cylinder(radius=0.024, length=0.004),
        origin=Origin(xyz=(0.026, 0.0, 0.004)),
        material=smoked_cover,
        name="cover_lid",
    )
    cover.visual(
        Cylinder(radius=0.020, length=0.003),
        origin=Origin(xyz=(0.029, 0.0, 0.007)),
        material=smoked_cover,
        name="cover_rim",
    )

    model.articulation(
        "sleeve_to_post",
        ArticulationType.PRISMATIC,
        parent=sleeve,
        child=post,
        origin=Origin(),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=3000.0,
            velocity=0.20,
            lower=0.0,
            upper=POST_TRAVEL,
        ),
    )
    model.articulation(
        "post_to_cap",
        ArticulationType.CONTINUOUS,
        parent=post,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, CAP_SEAT_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=6.0,
        ),
    )
    model.articulation(
        "post_to_cover",
        ArticulationType.REVOLUTE,
        parent=post,
        child=cover,
        origin=Origin(xyz=(0.050, 0.0, 0.933)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=3.0,
            lower=0.0,
            upper=1.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    sleeve = object_model.get_part("sleeve")
    post = object_model.get_part("post")
    cap = object_model.get_part("cap")
    cover = object_model.get_part("cover")

    slide_joint = object_model.get_articulation("sleeve_to_post")
    cover_joint = object_model.get_articulation("post_to_cover")

    ctx.expect_within(
        post,
        sleeve,
        axes="xy",
        inner_elem="post_body",
        outer_elem="sleeve_body",
        margin=0.0,
        name="post footprint stays within sleeve footprint",
    )
    ctx.expect_overlap(
        post,
        sleeve,
        axes="z",
        elem_a="post_body",
        elem_b="sleeve_body",
        min_overlap=0.60,
        name="extended post remains retained in sleeve",
    )
    ctx.expect_gap(
        cap,
        post,
        axis="z",
        positive_elem="keyed_cap",
        negative_elem="cap_seat",
        max_gap=0.0010,
        max_penetration=0.0,
        name="keyed cap stays visibly separate at crown",
    )
    ctx.expect_overlap(
        cap,
        post,
        axes="xy",
        elem_a="keyed_cap",
        elem_b="crown_plate",
        min_overlap=0.070,
        name="keyed cap stays centered on the crown",
    )
    ctx.expect_contact(
        cover,
        post,
        elem_a="hinge_barrel",
        elem_b="cover_mount",
        name="warning cover stays mounted to the crown hinge",
    )
    ctx.expect_gap(
        cover,
        post,
        axis="z",
        positive_elem="cover_lid",
        negative_elem="warning_lens",
        min_gap=0.004,
        max_gap=0.010,
        name="closed warning cover clears the light lens",
    )
    ctx.allow_overlap(
        post,
        sleeve,
        elem_a="guide_shoe",
        elem_b="sleeve_body",
        reason="The sliding post uses a close-running internal guide shoe that is intentionally represented inside the sleeve cavity mesh.",
    )

    rest_post_position = ctx.part_world_position(post)
    rest_cover_aabb = ctx.part_element_world_aabb(cover, elem="cover_lid")

    slide_limits = slide_joint.motion_limits
    if slide_limits is not None and slide_limits.upper is not None:
        with ctx.pose({slide_joint: slide_limits.upper}):
            ctx.expect_within(
                post,
                sleeve,
                axes="xy",
                inner_elem="post_body",
                outer_elem="sleeve_body",
                margin=0.0,
                name="retracted post footprint stays within sleeve footprint",
            )
            retracted_post_position = ctx.part_world_position(post)
        ctx.check(
            "post retracts downward into sleeve",
            rest_post_position is not None
            and retracted_post_position is not None
            and retracted_post_position[2] < rest_post_position[2] - 0.85,
            details=f"rest={rest_post_position}, retracted={retracted_post_position}",
        )

    cover_limits = cover_joint.motion_limits
    if cover_limits is not None and cover_limits.upper is not None:
        with ctx.pose({cover_joint: cover_limits.upper}):
            ctx.expect_gap(
                cover,
                cap,
                axis="z",
                positive_elem="cover_lid",
                negative_elem="keyed_cap",
                min_gap=0.002,
                name="opened cover rises above the keyed cap",
            )
            opened_cover_aabb = ctx.part_element_world_aabb(cover, elem="cover_lid")
        ctx.check(
            "cover opens upward from the crown",
            rest_cover_aabb is not None
            and opened_cover_aabb is not None
            and opened_cover_aabb[1][2] > rest_cover_aabb[1][2] + 0.015,
            details=f"rest={rest_cover_aabb}, opened={opened_cover_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
