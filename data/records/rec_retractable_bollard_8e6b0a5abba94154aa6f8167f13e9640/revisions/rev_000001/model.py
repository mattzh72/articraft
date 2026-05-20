from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_lane_rising_bollard")

    cast_iron = model.material("dark_cast_iron", rgba=(0.055, 0.058, 0.060, 1.0))
    blackened = model.material("blackened_hinge", rgba=(0.012, 0.012, 0.011, 1.0))
    brushed_steel = model.material("brushed_stainless_steel", rgba=(0.62, 0.66, 0.66, 1.0))
    cap_black = model.material("black_keyed_cap", rgba=(0.015, 0.016, 0.017, 1.0))
    reflective_yellow = model.material("amber_reflective_band", rgba=(1.0, 0.74, 0.08, 1.0))

    collar_outer_radius = 0.33
    collar_inner_radius = 0.12
    collar_thickness = 0.035
    sleeve_outer_radius = 0.13
    sleeve_inner_radius = 0.105
    sleeve_depth = 0.45

    collar_ring = (
        cq.Workplane("XY")
        .circle(collar_outer_radius)
        .circle(collar_inner_radius)
        .extrude(collar_thickness)
        .translate((0.0, 0.0, -collar_thickness))
    )
    flap_cutter = (
        cq.Workplane("XY")
        .box(0.175, 0.068, 0.080)
        .translate((0.2225, 0.0, -0.020))
    )
    collar_ring = collar_ring.cut(flap_cutter)
    guide_sleeve = (
        cq.Workplane("XY")
        .circle(sleeve_outer_radius)
        .circle(sleeve_inner_radius)
        .extrude(sleeve_depth)
        .translate((0.0, 0.0, -sleeve_depth))
    )

    collar = model.part("collar")
    collar.visual(
        mesh_from_cadquery(collar_ring, "collar_plate"),
        material=cast_iron,
        name="annular_collar",
    )
    collar.visual(
        mesh_from_cadquery(guide_sleeve, "guide_sleeve"),
        material=cast_iron,
        name="guide_sleeve",
    )

    # Flush socket-head bolts around the traffic-grade collar.
    for i, angle in enumerate((math.radians(a) for a in (45, 90, 135, 225, 270, 315))):
        collar.visual(
            Cylinder(radius=0.015, length=0.006),
            origin=Origin(xyz=(0.26 * math.cos(angle), 0.26 * math.sin(angle), 0.003)),
            material=blackened,
            name=f"bolt_head_{i}",
        )

    # Two fixed hinge knuckles and pads straddle the narrow access flap.
    for suffix, y in (("0", -0.043), ("1", 0.043)):
        collar.visual(
            Box((0.038, 0.016, 0.008)),
            origin=Origin(xyz=(0.145, y, 0.004)),
            material=blackened,
            name=f"hinge_pad_{suffix}",
        )
        collar.visual(
            Cylinder(radius=0.007, length=0.016),
            origin=Origin(xyz=(0.145, y, 0.012), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=blackened,
            name=f"hinge_knuckle_{suffix}",
        )

    post = model.part("post")
    post.visual(
        Cylinder(radius=0.090, length=0.950),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=brushed_steel,
        name="rising_cylinder",
    )
    post.visual(
        Cylinder(radius=0.106, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, -0.390)),
        material=brushed_steel,
        name="guide_bushing",
    )
    for i, z in enumerate((0.285, 0.395)):
        post.visual(
            Cylinder(radius=0.092, length=0.032),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=reflective_yellow,
            name=f"reflective_band_{i}",
        )

    access_flap = model.part("access_flap")
    access_flap.visual(
        Box((0.150, 0.052, 0.012)),
        origin=Origin(xyz=(0.080, 0.0, -0.018)),
        material=cast_iron,
        name="flap_plate",
    )
    access_flap.visual(
        Box((0.065, 0.044, 0.006)),
        origin=Origin(xyz=(0.035, 0.0, -0.0095)),
        material=blackened,
        name="hinge_leaf",
    )
    access_flap.visual(
        Cylinder(radius=0.007, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=blackened,
        name="hinge_barrel",
    )
    access_flap.visual(
        Cylinder(radius=0.004, length=0.102),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="hinge_pin",
    )

    top_cap = model.part("top_cap")
    top_cap.visual(
        Cylinder(radius=0.096, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=cap_black,
        name="round_cap",
    )
    top_cap.visual(
        Box((0.074, 0.014, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0290)),
        material=blackened,
        name="key_slot",
    )

    model.articulation(
        "collar_to_post",
        ArticulationType.PRISMATIC,
        parent=collar,
        child=post,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.18, lower=0.0, upper=0.32),
    )
    model.articulation(
        "collar_to_access_flap",
        ArticulationType.REVOLUTE,
        parent=collar,
        child=access_flap,
        origin=Origin(xyz=(0.145, 0.0, 0.012)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=0.0, upper=1.35),
    )
    model.articulation(
        "post_to_top_cap",
        ArticulationType.REVOLUTE,
        parent=post,
        child=top_cap,
        origin=Origin(xyz=(0.0, 0.0, 0.500)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=math.pi / 2.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    collar = object_model.get_part("collar")
    post = object_model.get_part("post")
    flap = object_model.get_part("access_flap")
    cap = object_model.get_part("top_cap")
    lift = object_model.get_articulation("collar_to_post")
    flap_hinge = object_model.get_articulation("collar_to_access_flap")
    cap_turn = object_model.get_articulation("post_to_top_cap")

    ctx.allow_overlap(
        collar,
        post,
        elem_a="guide_sleeve",
        elem_b="guide_bushing",
        reason="The hidden guide bushing is intentionally captured in the sleeve so the rising bollard is physically retained.",
    )
    for knuckle in ("hinge_knuckle_0", "hinge_knuckle_1"):
        ctx.allow_overlap(
            collar,
            flap,
            elem_a=knuckle,
            elem_b="hinge_pin",
            reason="The access-flap hinge pin is intentionally captured inside the fixed hinge knuckle.",
        )

    ctx.check(
        "post uses vertical prismatic lift",
        lift.articulation_type == ArticulationType.PRISMATIC and tuple(lift.axis) == (0.0, 0.0, 1.0),
        details=f"type={lift.articulation_type}, axis={lift.axis}",
    )
    ctx.check(
        "flap and cap are revolute controls",
        flap_hinge.articulation_type == ArticulationType.REVOLUTE
        and cap_turn.articulation_type == ArticulationType.REVOLUTE,
        details=f"flap={flap_hinge.articulation_type}, cap={cap_turn.articulation_type}",
    )
    ctx.expect_within(
        post,
        collar,
        axes="xy",
        inner_elem="guide_bushing",
        outer_elem="guide_sleeve",
        margin=0.0,
        name="post guide bushing stays centered in sleeve",
    )
    ctx.expect_overlap(
        post,
        collar,
        axes="z",
        elem_a="guide_bushing",
        elem_b="guide_sleeve",
        min_overlap=0.045,
        name="lowered post is captured in guide sleeve",
    )
    for knuckle in ("hinge_knuckle_0", "hinge_knuckle_1"):
        ctx.expect_overlap(
            flap,
            collar,
            axes="y",
            elem_a="hinge_pin",
            elem_b=knuckle,
            min_overlap=0.012,
            name=f"hinge pin spans {knuckle}",
        )
    ctx.expect_gap(
        cap,
        post,
        axis="z",
        positive_elem="round_cap",
        negative_elem="rising_cylinder",
        max_gap=0.001,
        max_penetration=0.0,
        name="top cap sits on post crown",
    )
    collar_aabb = ctx.part_element_world_aabb(collar, elem="annular_collar")
    flap_aabb = ctx.part_element_world_aabb(flap, elem="flap_plate")
    ctx.check(
        "closed flap top is flush with collar surface",
        collar_aabb is not None and flap_aabb is not None and abs(collar_aabb[1][2] - flap_aabb[1][2]) <= 0.001,
        details=f"collar={collar_aabb}, flap={flap_aabb}",
    )

    rest_post = ctx.part_world_position(post)
    rest_flap = ctx.part_element_world_aabb(flap, elem="flap_plate")
    rest_cap_slot = ctx.part_element_world_aabb(cap, elem="key_slot")

    with ctx.pose({lift: 0.32}):
        raised_post = ctx.part_world_position(post)
        ctx.expect_overlap(
            post,
            collar,
            axes="z",
            elem_a="guide_bushing",
            elem_b="guide_sleeve",
            min_overlap=0.040,
            name="raised post remains retained in sleeve",
        )

    with ctx.pose({flap_hinge: 1.0}):
        raised_flap = ctx.part_element_world_aabb(flap, elem="flap_plate")

    with ctx.pose({cap_turn: math.pi / 2.0}):
        turned_cap_slot = ctx.part_element_world_aabb(cap, elem="key_slot")

    ctx.check(
        "post rises upward at upper travel",
        rest_post is not None and raised_post is not None and raised_post[2] > rest_post[2] + 0.28,
        details=f"rest={rest_post}, raised={raised_post}",
    )
    ctx.check(
        "access flap opens upward",
        rest_flap is not None and raised_flap is not None and raised_flap[1][2] > rest_flap[1][2] + 0.06,
        details=f"rest={rest_flap}, raised={raised_flap}",
    )
    ctx.check(
        "keyed cap turns about vertical crown axis",
        rest_cap_slot is not None
        and turned_cap_slot is not None
        and (turned_cap_slot[1][1] - turned_cap_slot[0][1]) > (rest_cap_slot[1][1] - rest_cap_slot[0][1]) + 0.04,
        details=f"rest={rest_cap_slot}, turned={turned_cap_slot}",
    )

    return ctx.report()


object_model = build_object_model()
