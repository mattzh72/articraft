from __future__ import annotations

import math

import cadquery as cq

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
    mesh_from_cadquery,
)

PLATE_OUTER_RADIUS = 0.165
PLATE_THICKNESS = 0.016
POST_RADIUS = 0.098
POST_HOLE_RADIUS = 0.106
SLEEVE_OUTER_RADIUS = 0.118
SLEEVE_DEPTH = 0.180

POST_LENGTH = 1.000
POST_CENTER_Z = 0.240
POST_TRAVEL = 0.200
REFLECTOR_BAND_Z = 0.600
REFLECTOR_BAND_HEIGHT = 0.072
GUIDE_ROLLER_RADIUS = 0.004
GUIDE_ROLLER_RADIAL_CENTER = 0.102
GUIDE_ROLLER_Z = -0.016

FLAP_OPENING_WIDTH = 0.094
FLAP_OPENING_DEPTH = 0.040
FLAP_OPENING_CENTER_Y = 0.134
FLAP_LEAF_WIDTH = 0.092
FLAP_LEAF_DEPTH = 0.034
FLAP_THICKNESS = 0.006
HINGE_RADIUS = 0.0042
HINGE_AXIS_Y = 0.148
HINGE_AXIS_Z = 0.0125
PARENT_HINGE_LENGTH = 0.028
CHILD_HINGE_LENGTH = 0.026
CHILD_HINGE_OFFSET_X = 0.031

CAP_RADIUS = 0.094
CAP_HEIGHT = 0.014
KEY_BAR_LENGTH = 0.048
KEY_BAR_WIDTH = 0.014
KEY_BAR_HEIGHT = 0.008
POST_CROWN_Z = POST_CENTER_Z + (POST_LENGTH * 0.5)


def _build_collar_plate_shape():
    plate = (
        cq.Workplane("XY")
        .circle(PLATE_OUTER_RADIUS)
        .circle(POST_HOLE_RADIUS)
        .extrude(PLATE_THICKNESS, both=True)
    )

    flap_cut = cq.Workplane("XY", origin=(0.0, FLAP_OPENING_CENTER_Y, 0.0)).box(
        FLAP_OPENING_WIDTH,
        FLAP_OPENING_DEPTH,
        PLATE_THICKNESS * 3.0,
    )
    plate = plate.cut(flap_cut)

    hinge_pad = cq.Workplane("XY", origin=(0.0, 0.155, 0.006)).box(
        0.030,
        0.016,
        0.012,
    )
    hinge_barrel = cq.Workplane("YZ", origin=(0.0, HINGE_AXIS_Y, HINGE_AXIS_Z)).circle(
        HINGE_RADIUS
    ).extrude(PARENT_HINGE_LENGTH, both=True)

    return plate.union(hinge_pad).union(hinge_barrel)


def _build_guide_sleeve_shape():
    return LatheGeometry.from_shell_profiles(
        [
            (SLEEVE_OUTER_RADIUS, -SLEEVE_DEPTH),
            (SLEEVE_OUTER_RADIUS, 0.0),
        ],
        [
            (POST_HOLE_RADIUS, -SLEEVE_DEPTH),
            (POST_HOLE_RADIUS, 0.0),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )


def _span(aabb, axis_index: int) -> float | None:
    if aabb is None:
        return None
    return aabb[1][axis_index] - aabb[0][axis_index]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_lane_bollard")

    collar_metal = model.material("collar_metal", rgba=(0.27, 0.28, 0.30, 1.0))
    post_steel = model.material("post_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    reflector = model.material("reflector", rgba=(0.90, 0.73, 0.16, 1.0))
    cap_dark = model.material("cap_dark", rgba=(0.14, 0.15, 0.16, 1.0))
    key_metal = model.material("key_metal", rgba=(0.60, 0.63, 0.67, 1.0))

    collar = model.part("collar")
    collar.visual(
        mesh_from_cadquery(_build_collar_plate_shape(), "bollard_collar_plate"),
        material=collar_metal,
        name="collar_plate",
    )
    collar.visual(
        mesh_from_geometry(_build_guide_sleeve_shape(), "bollard_guide_sleeve"),
        material=collar_metal,
        name="guide_sleeve",
    )

    post = model.part("post")
    post.visual(
        Cylinder(radius=POST_RADIUS, length=POST_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, POST_CENTER_Z)),
        material=post_steel,
        name="post_shell",
    )
    post.visual(
        Cylinder(radius=POST_RADIUS + 0.0015, length=REFLECTOR_BAND_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, REFLECTOR_BAND_Z)),
        material=reflector,
        name="reflector_band",
    )
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        post.visual(
            Sphere(radius=GUIDE_ROLLER_RADIUS),
            origin=Origin(
                xyz=(
                    GUIDE_ROLLER_RADIAL_CENTER * math.cos(angle),
                    GUIDE_ROLLER_RADIAL_CENTER * math.sin(angle),
                    GUIDE_ROLLER_Z,
                ),
            ),
            material=cap_dark,
            name=f"guide_roller_{index}",
        )

    flap = model.part("flap")
    flap.visual(
        Box((FLAP_LEAF_WIDTH, FLAP_LEAF_DEPTH, FLAP_THICKNESS)),
        origin=Origin(
            xyz=(0.0, -(FLAP_LEAF_DEPTH * 0.5) - 0.003, -0.0072),
        ),
        material=collar_metal,
        name="flap_leaf",
    )
    for index, x_pos in enumerate((-CHILD_HINGE_OFFSET_X, CHILD_HINGE_OFFSET_X)):
        flap.visual(
            Cylinder(radius=HINGE_RADIUS, length=CHILD_HINGE_LENGTH),
            origin=Origin(xyz=(x_pos, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=collar_metal,
            name=f"flap_barrel_{index}",
        )
        flap.visual(
            Box((0.010, 0.006, 0.004)),
            origin=Origin(xyz=(x_pos, -0.003, -0.0042)),
            material=collar_metal,
            name=f"flap_bridge_{index}",
        )

    top_cap = model.part("top_cap")
    top_cap.visual(
        Cylinder(radius=CAP_RADIUS, length=CAP_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, CAP_HEIGHT * 0.5)),
        material=cap_dark,
        name="cap_body",
    )
    top_cap.visual(
        Box((KEY_BAR_LENGTH, KEY_BAR_WIDTH, KEY_BAR_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, CAP_HEIGHT + (KEY_BAR_HEIGHT * 0.5))),
        material=key_metal,
        name="key_bar",
    )

    model.articulation(
        "collar_to_post",
        ArticulationType.PRISMATIC,
        parent=collar,
        child=post,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.10,
            lower=0.0,
            upper=POST_TRAVEL,
        ),
    )
    model.articulation(
        "collar_to_flap",
        ArticulationType.REVOLUTE,
        parent=collar,
        child=flap,
        origin=Origin(xyz=(0.0, HINGE_AXIS_Y, HINGE_AXIS_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=1.20,
        ),
    )
    model.articulation(
        "post_to_top_cap",
        ArticulationType.REVOLUTE,
        parent=post,
        child=top_cap,
        origin=Origin(xyz=(0.0, 0.0, POST_CROWN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=4.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    collar = object_model.get_part("collar")
    post = object_model.get_part("post")
    flap = object_model.get_part("flap")
    top_cap = object_model.get_part("top_cap")

    post_lift = object_model.get_articulation("collar_to_post")
    flap_hinge = object_model.get_articulation("collar_to_flap")
    cap_joint = object_model.get_articulation("post_to_top_cap")

    for barrel_name in ("flap_barrel_0", "flap_barrel_1"):
        ctx.allow_overlap(
            collar,
            flap,
            elem_a="collar_plate",
            elem_b=barrel_name,
            reason=(
                "The collar plate mesh includes the compact hinge bracket, and the flap barrels "
                "are simplified solid knuckles wrapped around the same hinge axis."
            ),
        )
    ctx.allow_overlap(
        collar,
        flap,
        elem_a="collar_plate",
        elem_b="flap_leaf",
        reason=(
            "The collar mesh keeps the annular casting as one continuous plate, while the access "
            "flap leaf stands in for an inset service cover recessed into that collar."
        ),
    )

    for roller_name in ("guide_roller_0", "guide_roller_1", "guide_roller_2"):
        ctx.allow_overlap(
            collar,
            post,
            elem_a="guide_sleeve",
            elem_b=roller_name,
            reason=(
                "The buried guide sleeve is represented as a solid proxy mesh, while the post "
                "rollers stand in for the captured sliding guides inside that sleeve."
            ),
        )

    ctx.expect_within(
        post,
        collar,
        axes="xy",
        inner_elem="post_shell",
        outer_elem="guide_sleeve",
        margin=0.0,
        name="post stays centered in the guide sleeve at rest",
    )
    ctx.expect_overlap(
        post,
        collar,
        axes="z",
        elem_a="post_shell",
        elem_b="guide_sleeve",
        min_overlap=0.050,
        name="post remains inserted in the sleeve at rest",
    )

    post_rest = ctx.part_world_position(post)
    with ctx.pose({post_lift: POST_TRAVEL}):
        ctx.expect_within(
            post,
            collar,
            axes="xy",
            inner_elem="post_shell",
            outer_elem="guide_sleeve",
            margin=0.0,
            name="raised post stays centered in the sleeve",
        )
        ctx.expect_overlap(
            post,
            collar,
            axes="z",
            elem_a="post_shell",
            elem_b="guide_sleeve",
            min_overlap=0.050,
            name="raised post keeps retained insertion",
        )
        post_raised = ctx.part_world_position(post)

    ctx.check(
        "post raises upward",
        post_rest is not None and post_raised is not None and post_raised[2] > post_rest[2] + 0.150,
        details=f"rest={post_rest}, raised={post_raised}",
    )

    ctx.expect_contact(
        post,
        top_cap,
        elem_a="post_shell",
        elem_b="cap_body",
        contact_tol=1e-5,
        name="top cap seats on the post crown",
    )

    flap_rest = ctx.part_element_world_aabb(flap, elem="flap_leaf")
    with ctx.pose({flap_hinge: 1.0}):
        flap_open = ctx.part_element_world_aabb(flap, elem="flap_leaf")

    ctx.check(
        "flap opens upward from the collar",
        flap_rest is not None
        and flap_open is not None
        and flap_open[1][2] > flap_rest[1][2] + 0.020,
        details=f"closed={flap_rest}, open={flap_open}",
    )

    key_bar_rest = ctx.part_element_world_aabb(top_cap, elem="key_bar")
    with ctx.pose({cap_joint: math.pi / 2.0}):
        key_bar_turned = ctx.part_element_world_aabb(top_cap, elem="key_bar")

    rest_x_span = _span(key_bar_rest, 0)
    rest_y_span = _span(key_bar_rest, 1)
    turned_x_span = _span(key_bar_turned, 0)
    turned_y_span = _span(key_bar_turned, 1)
    ctx.check(
        "key bar changes orientation when the cap rotates",
        rest_x_span is not None
        and rest_y_span is not None
        and turned_x_span is not None
        and turned_y_span is not None
        and rest_x_span > rest_y_span
        and turned_y_span > turned_x_span,
        details=(
            f"rest_spans=({rest_x_span}, {rest_y_span}), "
            f"turned_spans=({turned_x_span}, {turned_y_span})"
        ),
    )

    return ctx.report()


object_model = build_object_model()
