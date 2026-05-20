from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    KnobGeometry,
    KnobGrip,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    ExtrudeGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="square_dial_thermostat")

    body_mat = Material("warm_white_plastic", rgba=(0.86, 0.84, 0.78, 1.0))
    face_mat = Material("matte_ivory_face", rgba=(0.93, 0.91, 0.86, 1.0))
    mark_mat = Material("charcoal_print", rgba=(0.05, 0.055, 0.055, 1.0))
    ring_mat = Material("soft_shadow_gray", rgba=(0.62, 0.61, 0.57, 1.0))
    red_mat = Material("warm_scale_red", rgba=(0.78, 0.18, 0.11, 1.0))
    blue_mat = Material("cool_scale_blue", rgba=(0.12, 0.28, 0.82, 1.0))

    body_side = 0.118
    body_depth = 0.018
    dial_diameter = 0.074
    dial_height = 0.014

    body = model.part("body")
    body_shell = ExtrudeGeometry.from_z0(
        rounded_rect_profile(body_side, body_side, 0.014, corner_segments=10),
        body_depth,
        cap=True,
        closed=True,
    )
    body.visual(
        mesh_from_geometry(body_shell, "body_shell"),
        material=body_mat,
        name="body_shell",
    )

    # A very shallow raised face pad gives the thermostat the molded, wall-control
    # look without making the body thick.
    face_pad = ExtrudeGeometry.from_z0(
        rounded_rect_profile(0.104, 0.104, 0.010, corner_segments=10),
        0.0008,
        cap=True,
        closed=True,
    )
    body.visual(
        mesh_from_geometry(face_pad, "face_pad"),
        origin=Origin(xyz=(0.0, 0.0, body_depth)),
        material=face_mat,
        name="face_pad",
    )

    body.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.0415, tube=0.0012, radial_segments=18, tubular_segments=72),
            "dial_socket_ring",
        ),
        origin=Origin(xyz=(0.0, 0.0, body_depth + 0.0011)),
        material=ring_mat,
        name="dial_socket_ring",
    )

    tick_radius = 0.049
    tick_z = body_depth + 0.0010
    for i, angle_deg in enumerate(range(-120, 121, 20)):
        angle = math.radians(angle_deg)
        is_major = angle_deg % 40 == 0
        tick_len = 0.0068 if is_major else 0.0048
        tick_width = 0.0014 if is_major else 0.0010
        material = blue_mat if angle_deg == -120 else red_mat if angle_deg == 120 else mark_mat
        body.visual(
            Box((tick_width, tick_len, 0.00045)),
            origin=Origin(
                xyz=(tick_radius * math.sin(angle), tick_radius * math.cos(angle), tick_z),
                rpy=(0.0, 0.0, -angle),
            ),
            material=material,
            name=f"scale_tick_{i}",
        )

    dial = model.part("dial")
    dial_mesh = mesh_from_geometry(
        KnobGeometry(
            dial_diameter,
            dial_height,
            body_style="cylindrical",
            edge_radius=0.0018,
            side_draft_deg=3.0,
            grip=KnobGrip(style="ribbed", count=42, depth=0.0009, width=0.0015),
            center=False,
        ),
        "dial_cap",
    )
    dial.visual(
        dial_mesh,
        material=face_mat,
        name="dial_cap",
    )
    dial.visual(
        Box((0.0032, 0.028, 0.00055)),
        origin=Origin(xyz=(0.0, 0.014, dial_height + 0.000275)),
        material=mark_mat,
        name="pointer_mark",
    )

    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, 0.0, body_depth + 0.0008)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    dial = object_model.get_part("dial")
    joint = object_model.get_articulation("body_to_dial")

    axis_ok = all(abs(a - b) < 1e-6 for a, b in zip(joint.axis, (0.0, 0.0, 1.0)))
    limits = joint.motion_limits
    ctx.check(
        "dial has unbounded continuous center rotation",
        joint.articulation_type == ArticulationType.CONTINUOUS
        and axis_ok
        and limits is not None
        and limits.lower is None
        and limits.upper is None,
        details=f"type={joint.articulation_type}, axis={joint.axis}, limits={limits}",
    )

    ctx.expect_gap(
        dial,
        body,
        axis="z",
        positive_elem="dial_cap",
        negative_elem="face_pad",
        max_gap=0.0003,
        max_penetration=0.0,
        name="dial seats on the raised face pad",
    )
    ctx.expect_within(
        dial,
        body,
        axes="xy",
        inner_elem="dial_cap",
        outer_elem="body_shell",
        margin=0.0,
        name="large dial remains inside the square body outline",
    )
    ctx.expect_overlap(
        dial,
        body,
        axes="xy",
        elem_a="dial_cap",
        elem_b="face_pad",
        min_overlap=0.070,
        name="dial is centered over the front control face",
    )

    def aabb_center(bounds):
        if bounds is None:
            return None
        lower, upper = bounds
        return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))

    rest_mark = aabb_center(ctx.part_element_world_aabb(dial, elem="pointer_mark"))
    with ctx.pose({joint: math.pi / 2.0}):
        turned_mark = aabb_center(ctx.part_element_world_aabb(dial, elem="pointer_mark"))
        ctx.expect_gap(
            dial,
            body,
            axis="z",
            positive_elem="dial_cap",
            negative_elem="face_pad",
            max_gap=0.0003,
            max_penetration=0.0,
            name="rotated dial stays seated on the body",
        )

    ctx.check(
        "pointer mark visibly rotates with the dial",
        rest_mark is not None
        and turned_mark is not None
        and abs(rest_mark[0]) < 0.003
        and rest_mark[1] > 0.010
        and turned_mark[0] < -0.010
        and abs(turned_mark[1]) < 0.003,
        details=f"rest={rest_mark}, turned={turned_mark}",
    )

    return ctx.report()


object_model = build_object_model()
