from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_arm_turnstile_gate")

    dark_steel = model.material("dark_steel", rgba=(0.12, 0.13, 0.14, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.65, 1.0))
    warm_steel = model.material("warm_steel", rgba=(0.78, 0.70, 0.53, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.95, 0.76, 0.12, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.03, 0.03, 0.035, 1.0))
    concrete = model.material("concrete", rgba=(0.50, 0.50, 0.48, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((1.90, 1.90, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=concrete,
        name="base_plate",
    )
    frame.visual(
        Cylinder(radius=0.070, length=1.085),
        origin=Origin(xyz=(0.0, 0.0, 0.6175)),
        material=dark_steel,
        name="central_column",
    )
    frame.visual(
        Cylinder(radius=0.115, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 1.1775)),
        material=brushed_steel,
        name="bearing_pad",
    )
    frame.visual(
        Cylinder(radius=0.075, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 1.1325)),
        material=warm_steel,
        name="bearing_neck",
    )

    half_span = 0.82
    post_radius = 0.035
    rail_radius = 0.025
    post_bottom = 0.070
    post_top = 1.560
    post_center_z = (post_bottom + post_top) * 0.5
    post_length = post_top - post_bottom
    post_positions = [
        (-half_span, -half_span),
        (half_span, -half_span),
        (half_span, half_span),
        (-half_span, half_span),
    ]
    for idx, (x, y) in enumerate(post_positions):
        frame.visual(
            Cylinder(radius=post_radius, length=post_length),
            origin=Origin(xyz=(x, y, post_center_z)),
            material=dark_steel,
            name=f"corner_post_{idx}",
        )
        frame.visual(
            Cylinder(radius=0.055, length=0.030),
            origin=Origin(xyz=(x, y, 0.095)),
            material=brushed_steel,
            name=f"post_foot_{idx}",
        )

    rail_levels = (0.95, 1.47)
    for level_index, z in enumerate(rail_levels):
        corners = [
            (-half_span, -half_span, z),
            (half_span, -half_span, z),
            (half_span, half_span, z),
            (-half_span, half_span, z),
        ]
        for side_index in range(4):
            _add_member(
                frame,
                corners[side_index],
                corners[(side_index + 1) % 4],
                rail_radius,
                brushed_steel,
                name=f"guard_rail_{level_index}_{side_index}",
            )

    # Diagonal braces make the fixed cage read as a rigid welded turnstile frame.
    _add_member(
        frame,
        (-half_span, -half_span, 0.95),
        (half_span, -half_span, 1.47),
        0.018,
        safety_yellow,
        name="braced_post_0",
    )
    _add_member(
        frame,
        (half_span, half_span, 0.95),
        (-half_span, half_span, 1.47),
        0.018,
        safety_yellow,
        name="braced_post_1",
    )
    frame.inertial = Inertial.from_geometry(
        Box((1.90, 1.90, 1.56)),
        mass=85.0,
        origin=Origin(xyz=(0.0, 0.0, 0.78)),
    )

    arm_hub = model.part("arm_hub")
    arm_hub.visual(
        Cylinder(radius=0.095, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=brushed_steel,
        name="hub_shell",
    )
    arm_hub.visual(
        Cylinder(radius=0.060, length=0.235),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_steel,
        name="hub_spindle",
    )
    arm_hub.visual(
        Cylinder(radius=0.115, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.1025)),
        material=warm_steel,
        name="top_collar",
    )

    arm_inner = 0.060
    arm_outer = 0.660
    for idx in range(3):
        angle = idx * 2.0 * math.pi / 3.0
        c = math.cos(angle)
        s = math.sin(angle)
        inner = (arm_inner * c, arm_inner * s, 0.0)
        outer = (arm_outer * c, arm_outer * s, 0.0)
        _add_member(
            arm_hub,
            inner,
            outer,
            0.026,
            safety_yellow,
            name=f"arm_{idx}",
        )
        arm_hub.visual(
            Sphere(radius=0.042),
            origin=Origin(xyz=outer),
            material=rubber_black,
            name=f"arm_tip_{idx}",
        )
        trim = ((arm_outer - 0.10) * c, (arm_outer - 0.10) * s, 0.0)
        arm_hub.visual(
            Cylinder(radius=0.031, length=0.040),
            origin=Origin(xyz=trim, rpy=_rpy_for_cylinder((0.0, 0.0, 0.0), (c, s, 0.0))),
            material=brushed_steel,
            name=f"tip_band_{idx}",
        )
    arm_hub.inertial = Inertial.from_geometry(
        Cylinder(radius=0.66, length=0.18),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    model.articulation(
        "hub_rotation",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=arm_hub,
        origin=Origin(xyz=(0.0, 0.0, 1.290)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=2.4),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    arm_hub = object_model.get_part("arm_hub")
    spin = object_model.get_articulation("hub_rotation")

    ctx.check(
        "hub uses a continuous vertical joint",
        spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.allow_overlap(
        arm_hub,
        frame,
        elem_a="hub_spindle",
        elem_b="bearing_pad",
        reason="The rotating spindle is intentionally captured inside the fixed bearing pad.",
    )
    ctx.expect_within(
        arm_hub,
        frame,
        axes="xy",
        inner_elem="hub_spindle",
        outer_elem="bearing_pad",
        margin=0.0,
        name="spindle is centered inside the bearing pad",
    )
    ctx.expect_overlap(
        arm_hub,
        frame,
        axes="z",
        elem_a="hub_spindle",
        elem_b="bearing_pad",
        min_overlap=0.020,
        name="spindle is retained in the bearing pad",
    )
    ctx.expect_contact(
        arm_hub,
        frame,
        elem_a="hub_shell",
        elem_b="bearing_pad",
        contact_tol=1e-5,
        name="rotor hub sits on the raised bearing",
    )
    ctx.expect_gap(
        arm_hub,
        frame,
        axis="z",
        positive_elem="hub_shell",
        negative_elem="base_plate",
        min_gap=1.10,
        name="rotating stage is high above the base",
    )
    ctx.expect_within(
        arm_hub,
        frame,
        axes="xy",
        inner_elem="arm_tip_0",
        outer_elem="base_plate",
        margin=0.0,
        name="arm tips remain inside the fixed frame footprint",
    )

    rest_aabb = ctx.part_element_world_aabb(arm_hub, elem="arm_0")
    with ctx.pose({spin: math.pi / 2.0}):
        turned_aabb = ctx.part_element_world_aabb(arm_hub, elem="arm_0")
        ctx.expect_within(
            arm_hub,
            frame,
            axes="xy",
            inner_elem="arm_tip_0",
            outer_elem="base_plate",
            margin=0.0,
            name="rotated arm stays inside the fixed frame footprint",
        )

    def _aabb_center_y(aabb) -> float | None:
        if aabb is None:
            return None
        return (aabb[0][1] + aabb[1][1]) * 0.5

    rest_y = _aabb_center_y(rest_aabb)
    turned_y = _aabb_center_y(turned_aabb)
    ctx.check(
        "arm geometry follows the hub rotation",
        rest_y is not None and turned_y is not None and turned_y > rest_y + 0.25,
        details=f"rest_y={rest_y}, turned_y={turned_y}",
    )

    return ctx.report()


object_model = build_object_model()
