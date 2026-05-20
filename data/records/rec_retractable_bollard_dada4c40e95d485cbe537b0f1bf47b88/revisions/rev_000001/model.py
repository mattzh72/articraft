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
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

FLANGE_RADIUS = 0.18
FLANGE_THICKNESS = 0.018
SLEEVE_OUTER_RADIUS = 0.145
SLEEVE_INNER_RADIUS = 0.109
SLEEVE_DEPTH = 0.96

POST_RADIUS = 0.095
POST_INSERTION = 0.23
POST_TOP_HEIGHT = 0.74
CAP_RADIUS = 0.106
CAP_HEIGHT = 0.028
POST_TRAVEL = 0.725

KEY_CAP_RADIUS = 0.028
KEY_CAP_HEIGHT = 0.012
KEY_SLOT_LENGTH = 0.020
KEY_SLOT_WIDTH = 0.004

FLAP_LENGTH = 0.056
FLAP_WIDTH = 0.052
FLAP_THICKNESS = 0.005
FLAP_HINGE_RADIUS = 0.005
FLAP_HINGE_Y_OFFSET = 0.018
FLAP_HINGE_X = 0.162
FLAP_HINGE_Z = FLAP_HINGE_RADIUS
FLAP_SETBACK = 0.006


def _sleeve_geometry() -> LatheGeometry:
    outer_profile = [
        (SLEEVE_OUTER_RADIUS, -SLEEVE_DEPTH),
        (SLEEVE_OUTER_RADIUS, -FLANGE_THICKNESS),
        (FLANGE_RADIUS, -FLANGE_THICKNESS),
        (FLANGE_RADIUS, 0.0),
    ]
    inner_profile = [
        (SLEEVE_INNER_RADIUS, -SLEEVE_DEPTH),
        (SLEEVE_INNER_RADIUS, -0.12),
        (POST_RADIUS, -0.10),
        (POST_RADIUS, -0.04),
        (CAP_RADIUS + 0.001, -0.01),
        (CAP_RADIUS + 0.001, 0.0),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=72,
    )


def _post_shape() -> cq.Workplane:
    shaft_height = POST_TOP_HEIGHT - CAP_HEIGHT + POST_INSERTION
    shaft = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, -POST_INSERTION))
        .circle(POST_RADIUS)
        .extrude(shaft_height)
    )
    cap = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, POST_TOP_HEIGHT - CAP_HEIGHT))
        .circle(CAP_RADIUS)
        .extrude(CAP_HEIGHT)
    )
    return shaft.union(cap)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retractable_security_bollard")

    sleeve_steel = model.material("sleeve_steel", rgba=(0.38, 0.40, 0.43, 1.0))
    post_steel = model.material("post_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    reflector = model.material("reflector", rgba=(0.93, 0.78, 0.20, 1.0))
    hardware_black = model.material("hardware_black", rgba=(0.11, 0.12, 0.13, 1.0))

    sleeve = model.part("sleeve")
    sleeve.visual(
        mesh_from_geometry(_sleeve_geometry(), "sleeve_shell"),
        material=sleeve_steel,
        name="sleeve_shell",
    )
    for idx, y_pos in enumerate((-FLAP_HINGE_Y_OFFSET, FLAP_HINGE_Y_OFFSET)):
        sleeve.visual(
            Box((0.016, 0.014, 0.008)),
            origin=Origin(xyz=(FLAP_HINGE_X, y_pos, 0.002)),
            material=sleeve_steel,
            name=f"hinge_pedestal_{idx}",
        )
        sleeve.visual(
            Cylinder(radius=FLAP_HINGE_RADIUS, length=0.014),
            origin=Origin(xyz=(FLAP_HINGE_X, y_pos, FLAP_HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=sleeve_steel,
            name=f"hinge_knuckle_{idx}",
        )

    post = model.part("post")
    post.visual(
        mesh_from_cadquery(_post_shape(), "post_shell"),
        material=post_steel,
        name="post_shell",
    )
    post.visual(
        Cylinder(radius=POST_RADIUS + 0.0015, length=0.11),
        origin=Origin(xyz=(0.0, 0.0, 0.46)),
        material=reflector,
        name="reflective_band",
    )

    key_cap = model.part("key_cap")
    key_cap.visual(
        Cylinder(radius=KEY_CAP_RADIUS, length=KEY_CAP_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, KEY_CAP_HEIGHT / 2.0)),
        material=hardware_black,
        name="key_cap_shell",
    )
    key_cap.visual(
        Box((KEY_SLOT_LENGTH, KEY_SLOT_WIDTH, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, KEY_CAP_HEIGHT - 0.001)),
        material=post_steel,
        name="key_slot",
    )

    maintenance_flap = model.part("maintenance_flap")
    maintenance_flap.visual(
        Box((FLAP_LENGTH, FLAP_WIDTH, FLAP_THICKNESS)),
        origin=Origin(xyz=(-(FLAP_LENGTH / 2.0) - FLAP_SETBACK, 0.0, -FLAP_THICKNESS / 2.0)),
        material=sleeve_steel,
        name="flap_plate",
    )
    maintenance_flap.visual(
        Cylinder(radius=FLAP_HINGE_RADIUS, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=sleeve_steel,
        name="flap_barrel",
    )
    maintenance_flap.visual(
        Box((0.008, 0.020, 0.004)),
        origin=Origin(xyz=(-0.004, 0.0, -0.003)),
        material=sleeve_steel,
        name="flap_bridge",
    )
    maintenance_flap.visual(
        Box((0.010, 0.026, 0.004)),
        origin=Origin(xyz=(-FLAP_LENGTH - FLAP_SETBACK + 0.005, 0.0, -0.003)),
        material=hardware_black,
        name="flap_lip",
    )

    model.articulation(
        "sleeve_to_post",
        ArticulationType.PRISMATIC,
        parent=sleeve,
        child=post,
        origin=Origin(),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=POST_TRAVEL,
            effort=1800.0,
            velocity=0.20,
        ),
    )
    model.articulation(
        "post_to_key_cap",
        ArticulationType.REVOLUTE,
        parent=post,
        child=key_cap,
        origin=Origin(xyz=(0.0, 0.0, POST_TOP_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-1.1,
            upper=1.1,
            effort=8.0,
            velocity=1.0,
        ),
    )
    model.articulation(
        "sleeve_to_maintenance_flap",
        ArticulationType.REVOLUTE,
        parent=sleeve,
        child=maintenance_flap,
        origin=Origin(xyz=(FLAP_HINGE_X, 0.0, FLAP_HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=1.2,
            effort=6.0,
            velocity=1.5,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    sleeve = object_model.get_part("sleeve")
    post = object_model.get_part("post")
    key_cap = object_model.get_part("key_cap")
    maintenance_flap = object_model.get_part("maintenance_flap")
    post_slide = object_model.get_articulation("sleeve_to_post")
    key_turn = object_model.get_articulation("post_to_key_cap")
    flap_hinge = object_model.get_articulation("sleeve_to_maintenance_flap")

    ctx.allow_overlap(
        sleeve,
        post,
        elem_a="sleeve_shell",
        elem_b="post_shell",
        reason="The bollard post is intentionally represented as a telescoping member guided inside the subterranean sleeve housing.",
    )

    ctx.expect_origin_distance(
        post,
        sleeve,
        axes="xy",
        min_dist=0.0,
        max_dist=0.001,
        name="post stays concentric with sleeve",
    )
    ctx.expect_overlap(
        post,
        sleeve,
        axes="z",
        min_overlap=0.20,
        name="raised post remains inserted in sleeve",
    )
    ctx.expect_gap(
        key_cap,
        post,
        axis="z",
        positive_elem="key_cap_shell",
        negative_elem="post_shell",
        max_gap=0.001,
        max_penetration=0.001,
        name="key cap sits on the bollard top",
    )
    ctx.expect_gap(
        maintenance_flap,
        sleeve,
        axis="z",
        positive_elem="flap_plate",
        negative_elem="sleeve_shell",
        max_gap=0.001,
        max_penetration=0.0,
        name="maintenance flap sits flush on sleeve collar",
    )

    raised_pos = ctx.part_world_position(post)
    closed_flap_aabb = ctx.part_element_world_aabb(maintenance_flap, elem="flap_plate")
    with ctx.pose({post_slide: POST_TRAVEL}):
        ctx.expect_overlap(
            post,
            sleeve,
            axes="z",
            min_overlap=0.85,
            name="retracted post remains deeply inside sleeve",
        )
        lowered_pos = ctx.part_world_position(post)
        ctx.expect_gap(
            key_cap,
            post,
            axis="z",
            positive_elem="key_cap_shell",
            negative_elem="post_shell",
            max_gap=0.001,
            max_penetration=0.001,
            name="key cap stays seated while the bollard retracts",
        )

    with ctx.pose({key_turn: 0.8}):
        ctx.expect_gap(
            key_cap,
            post,
            axis="z",
            positive_elem="key_cap_shell",
            negative_elem="post_shell",
            max_gap=0.001,
            max_penetration=0.001,
            name="key cap rotates in place on the top surface",
        )

    with ctx.pose({flap_hinge: 1.0}):
        open_flap_aabb = ctx.part_element_world_aabb(maintenance_flap, elem="flap_plate")

    ctx.check(
        "post retracts downward into sleeve",
        raised_pos is not None
        and lowered_pos is not None
        and lowered_pos[2] < raised_pos[2] - 0.60,
        details=f"raised={raised_pos}, lowered={lowered_pos}",
    )
    ctx.check(
        "maintenance flap lifts upward when opened",
        closed_flap_aabb is not None
        and open_flap_aabb is not None
        and open_flap_aabb[1][2] > closed_flap_aabb[1][2] + 0.03,
        details=f"closed={closed_flap_aabb}, open={open_flap_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
