from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

PAVEMENT_SIZE = 1.6
PAVEMENT_THICKNESS = 0.18
SLEEVE_DEPTH = 0.95
SLEEVE_OUTER_RADIUS = 0.135
SLEEVE_INNER_RADIUS = 0.118
FLANGE_OUTER_RADIUS = 0.175
FLANGE_INNER_RADIUS = 0.122
FLANGE_HEIGHT = 0.028
BASE_STOP_HEIGHT = 0.04

POST_OUTER_RADIUS = 0.105
POST_INNER_RADIUS = 0.092
POST_HIDDEN_LENGTH = SLEEVE_DEPTH + FLANGE_HEIGHT
POST_VISIBLE_REST = 0.62
POST_TRAVEL = 0.20
RETAINER_HOLE_RADIUS = 0.022
RETAINER_FLANGE_RADIUS = 0.040

CAP_GAP = 0.004
CAP_RADIUS = 0.111
CAP_HEIGHT = 0.038
CAP_STEM_RADIUS = 0.018
CAP_STEM_LENGTH = 0.028

REFLECTOR_HEIGHT = 0.055
REFLECTOR_RADIUS = POST_OUTER_RADIUS + 0.0015


def _solid_cylinder(radius: float, z_min: float, z_max: float) -> cq.Workplane:
    return cq.Workplane("XY").workplane(offset=z_min).circle(radius).extrude(z_max - z_min)


def _annulus(outer_radius: float, inner_radius: float, z_min: float, z_max: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .workplane(offset=z_min)
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(z_max - z_min)
    )


def _pavement_shape() -> cq.Workplane:
    slab = (
        cq.Workplane("XY")
        .workplane(offset=-PAVEMENT_THICKNESS)
        .rect(PAVEMENT_SIZE, PAVEMENT_SIZE)
        .extrude(PAVEMENT_THICKNESS)
    )
    opening = _solid_cylinder(FLANGE_INNER_RADIUS, -PAVEMENT_THICKNESS - 0.01, 0.01)
    return slab.cut(opening)


def _tube_shell(outer_radius: float, inner_radius: float, z_min: float, z_max: float) -> cq.Workplane:
    return _annulus(outer_radius, inner_radius, z_min, z_max)


def _cap_body_shape() -> cq.Workplane:
    cap_body = _solid_cylinder(CAP_RADIUS, CAP_GAP, CAP_GAP + CAP_HEIGHT)
    key_slot = (
        cq.Workplane("XY")
        .workplane(offset=CAP_GAP + CAP_HEIGHT - 0.014)
        .rect(0.038, 0.012)
        .extrude(0.018)
    )
    key_core = _solid_cylinder(0.008, CAP_GAP + CAP_HEIGHT - 0.012, CAP_GAP + CAP_HEIGHT + 0.002)
    return cap_body.cut(key_slot).cut(key_core)


def _axis_is_vertical(axis: tuple[float, float, float] | None) -> bool:
    if axis is None:
        return False
    return all(abs(a - b) < 1e-9 for a, b in zip(axis, (0.0, 0.0, 1.0)))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retractable_security_bollard")

    concrete = model.material("concrete", rgba=(0.59, 0.59, 0.57, 1.0))
    galvanized = model.material("galvanized", rgba=(0.55, 0.58, 0.61, 1.0))
    stainless = model.material("stainless", rgba=(0.77, 0.79, 0.81, 1.0))
    bollard_paint = model.material("bollard_paint", rgba=(0.17, 0.18, 0.19, 1.0))
    reflector = model.material("reflector", rgba=(0.90, 0.75, 0.18, 1.0))
    cap_finish = model.material("cap_finish", rgba=(0.10, 0.11, 0.12, 1.0))

    ground_sleeve = model.part("ground_sleeve")
    ground_sleeve.visual(
        mesh_from_cadquery(_pavement_shape(), "pavement_pad"),
        material=concrete,
        name="pavement_pad",
    )
    ground_sleeve.visual(
        mesh_from_cadquery(
            _annulus(SLEEVE_OUTER_RADIUS, SLEEVE_INNER_RADIUS, -SLEEVE_DEPTH, 0.0),
            "sleeve_body",
        ),
        material=galvanized,
        name="sleeve_body",
    )
    ground_sleeve.visual(
        mesh_from_cadquery(
            _solid_cylinder(SLEEVE_INNER_RADIUS, -SLEEVE_DEPTH - BASE_STOP_HEIGHT, -SLEEVE_DEPTH),
            "base_stop",
        ),
        material=galvanized,
        name="base_stop",
    )
    ground_sleeve.visual(
        mesh_from_cadquery(
            _annulus(FLANGE_OUTER_RADIUS, FLANGE_INNER_RADIUS, 0.0, FLANGE_HEIGHT),
            "flange_collar",
        ),
        material=stainless,
        name="flange_collar",
    )

    post = model.part("post")
    post.visual(
        mesh_from_cadquery(
            _tube_shell(
                POST_OUTER_RADIUS,
                POST_INNER_RADIUS,
                -POST_HIDDEN_LENGTH,
                POST_VISIBLE_REST,
            ),
            "tube_shell",
        ),
        material=bollard_paint,
        name="tube_shell",
    )
    post.visual(
        mesh_from_cadquery(
            _annulus(
                POST_INNER_RADIUS,
                RETAINER_HOLE_RADIUS,
                POST_VISIBLE_REST - 0.012,
                POST_VISIBLE_REST - 0.006,
            ),
            "retainer_plate",
        ),
        material=galvanized,
        name="retainer_plate",
    )
    post.visual(
        Cylinder(radius=REFLECTOR_RADIUS, length=REFLECTOR_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, 0.37)),
        material=reflector,
        name="reflector_0",
    )
    post.visual(
        Cylinder(radius=REFLECTOR_RADIUS, length=REFLECTOR_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, 0.47)),
        material=reflector,
        name="reflector_1",
    )

    top_cap = model.part("top_cap")
    top_cap.visual(
        mesh_from_cadquery(_cap_body_shape(), "cap_body"),
        material=cap_finish,
        name="cap_body",
    )
    top_cap.visual(
        Cylinder(radius=CAP_STEM_RADIUS, length=CAP_STEM_LENGTH + CAP_GAP),
        origin=Origin(xyz=(0.0, 0.0, (CAP_GAP - CAP_STEM_LENGTH) / 2.0)),
        material=galvanized,
        name="cap_stem",
    )
    top_cap.visual(
        Cylinder(radius=RETAINER_FLANGE_RADIUS, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=galvanized,
        name="retainer_flange",
    )

    model.articulation(
        "sleeve_to_post",
        ArticulationType.PRISMATIC,
        parent=ground_sleeve,
        child=post,
        origin=Origin(xyz=(0.0, 0.0, FLANGE_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6000.0,
            velocity=0.30,
            lower=0.0,
            upper=POST_TRAVEL,
        ),
    )
    model.articulation(
        "post_to_cap",
        ArticulationType.CONTINUOUS,
        parent=post,
        child=top_cap,
        origin=Origin(xyz=(0.0, 0.0, POST_VISIBLE_REST)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=6.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ground_sleeve = object_model.get_part("ground_sleeve")
    post = object_model.get_part("post")
    top_cap = object_model.get_part("top_cap")
    post_lift = object_model.get_articulation("sleeve_to_post")
    cap_spin = object_model.get_articulation("post_to_cap")

    lift_limits = post_lift.motion_limits
    spin_limits = cap_spin.motion_limits

    ctx.check(
        "post uses vertical prismatic lift",
        post_lift.articulation_type == ArticulationType.PRISMATIC and _axis_is_vertical(post_lift.axis),
        details=f"type={post_lift.articulation_type}, axis={post_lift.axis}",
    )
    ctx.check(
        "cap uses vertical continuous spin",
        cap_spin.articulation_type == ArticulationType.CONTINUOUS
        and _axis_is_vertical(cap_spin.axis)
        and spin_limits is not None
        and spin_limits.lower is None
        and spin_limits.upper is None,
        details=f"type={cap_spin.articulation_type}, axis={cap_spin.axis}, limits={spin_limits}",
    )

    with ctx.pose({post_lift: 0.0}):
        ctx.expect_origin_distance(
            post,
            ground_sleeve,
            axes="xy",
            max_dist=0.001,
            name="resting post stays concentric with sleeve",
        )
        ctx.expect_overlap(
            post,
            ground_sleeve,
            axes="xy",
            elem_a="tube_shell",
            elem_b="sleeve_body",
            min_overlap=0.20,
            name="resting post aligns over sleeve opening",
        )
        ctx.expect_overlap(
            post,
            ground_sleeve,
            axes="z",
            elem_a="tube_shell",
            elem_b="sleeve_body",
            min_overlap=0.90,
            name="resting post remains deeply guided by sleeve",
        )
        ctx.expect_contact(
            post,
            ground_sleeve,
            elem_a="tube_shell",
            elem_b="base_stop",
            name="post is supported by the sleeve base stop",
        )
        ctx.expect_gap(
            top_cap,
            post,
            axis="z",
            positive_elem="cap_body",
            negative_elem="tube_shell",
            min_gap=0.002,
            max_gap=0.008,
            name="cap body stays visibly separate from tube",
        )
        ctx.expect_overlap(
            top_cap,
            post,
            axes="xy",
            elem_a="cap_body",
            elem_b="tube_shell",
            min_overlap=0.20,
            name="cap stays centered on the post",
        )
        ctx.expect_contact(
            top_cap,
            post,
            elem_a="retainer_flange",
            elem_b="retainer_plate",
            name="cap remains mounted to the post",
        )

    if lift_limits is not None and lift_limits.upper is not None:
        rest_pos = ctx.part_world_position(post)
        with ctx.pose({post_lift: lift_limits.upper}):
            ctx.expect_origin_distance(
                post,
                ground_sleeve,
                axes="xy",
                max_dist=0.001,
                name="raised post stays concentric with sleeve",
            )
            ctx.expect_overlap(
                post,
                ground_sleeve,
                axes="z",
                elem_a="tube_shell",
                elem_b="sleeve_body",
                min_overlap=0.70,
                name="raised post retains insertion in sleeve",
            )
            raised_pos = ctx.part_world_position(post)

        ctx.check(
            "post rises upward",
            rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.15,
            details=f"rest={rest_pos}, raised={raised_pos}",
        )

    with ctx.pose({cap_spin: math.pi / 2.0}):
        ctx.expect_gap(
            top_cap,
            post,
            axis="z",
            positive_elem="cap_body",
            negative_elem="tube_shell",
            min_gap=0.001,
            max_gap=0.010,
            name="cap stays seated while rotating",
        )
        ctx.expect_overlap(
            top_cap,
            post,
            axes="xy",
            elem_a="cap_body",
            elem_b="tube_shell",
            min_overlap=0.20,
            name="cap rotates about its own axis",
        )
        ctx.expect_contact(
            top_cap,
            post,
            elem_a="retainer_flange",
            elem_b="retainer_plate",
            name="cap mount stays engaged while rotating",
        )

    return ctx.report()


object_model = build_object_model()
