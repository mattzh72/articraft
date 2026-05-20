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


def _rotated_box(
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    *,
    pitch: float = 0.0,
) -> tuple[Box, Origin]:
    return Box(size), Origin(xyz=center, rpy=(0.0, pitch, 0.0))


def _beam_between(
    part,
    p1: tuple[float, float, float],
    p2: tuple[float, float, float],
    *,
    thickness: float,
    material,
    name: str,
) -> None:
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    dz = p2[2] - p1[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if abs(dy) > 1e-9:
        raise ValueError("side-frame diagonal helper expects a constant Y plane")
    center = ((p1[0] + p2[0]) * 0.5, p1[1], (p1[2] + p2[2]) * 0.5)
    pitch = -math.atan2(dz, dx)
    geom, origin = _rotated_box((length, thickness, thickness), center, pitch=pitch)
    part.visual(geom, origin=origin, material=material, name=name)


def _cq_box(
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    *,
    rotate_y_deg: float = 0.0,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(size[0], size[1], size[2])
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), rotate_y_deg)
        .translate(center)
    )


def _make_wheel_wood() -> cq.Workplane:
    outer_radius = 0.445
    inner_radius = 0.370
    rim_y = 0.155
    rim_thickness = 0.040
    hub_radius = 0.090
    wheel_width = 0.360

    wheel = (
        cq.Workplane("XZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(rim_thickness, both=True)
        .translate((0.0, -rim_y, 0.0))
    )
    wheel = wheel.union(
        cq.Workplane("XZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(rim_thickness, both=True)
        .translate((0.0, rim_y, 0.0))
    )

    # A broad wooden hub barrel ties both side spoke sets together around the
    # iron axle, just as a small mill wheel would be built.
    wheel = wheel.union(
        cq.Workplane("XZ").circle(hub_radius).extrude(0.280, both=True)
    )

    spoke_inner = 0.065
    spoke_outer = 0.395
    spoke_length = spoke_outer - spoke_inner
    spoke_center = (spoke_outer + spoke_inner) * 0.5
    for side_y in (-0.135, 0.135):
        for i in range(8):
            angle = i * 45.0
            theta = math.radians(angle)
            cx = spoke_center * math.cos(theta)
            cz = spoke_center * math.sin(theta)
            wheel = wheel.union(
                _cq_box(
                    (spoke_length, 0.040, 0.040),
                    (cx, side_y, cz),
                    rotate_y_deg=-angle,
                )
            )

    # Undershot paddle boards span the wheel width and sit at the outer rim.
    paddle_radius = 0.425
    for i in range(12):
        angle = i * 30.0
        theta = math.radians(angle)
        cx = paddle_radius * math.cos(theta)
        cz = paddle_radius * math.sin(theta)
        wheel = wheel.union(
            _cq_box(
                (0.205, wheel_width, 0.050),
                (cx, 0.0, cz),
                rotate_y_deg=90.0 - angle,
            )
        )

    return wheel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="undershot_waterwheel")

    timber = model.material("weathered_timber", rgba=(0.48, 0.30, 0.16, 1.0))
    dark_wood = model.material("wet_paddle_wood", rgba=(0.34, 0.20, 0.10, 1.0))
    iron = model.material("dark_iron", rgba=(0.08, 0.075, 0.065, 1.0))
    water = model.material("shallow_water", rgba=(0.18, 0.43, 0.78, 0.62))

    frame = model.part("frame")

    # Water race / simple trough below the undershot wheel.
    frame.visual(
        Box((1.55, 0.56, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.2475)),
        material=timber,
        name="race_floor",
    )
    frame.visual(
        Box((1.40, 0.42, 0.013)),
        origin=Origin(xyz=(0.04, 0.0, 0.2765)),
        material=water,
        name="water_strip",
    )
    for y, suffix in ((-0.280, "0"), (0.280, "1")):
        frame.visual(
            Box((1.55, 0.035, 0.180)),
            origin=Origin(xyz=(0.0, y, 0.335)),
            material=timber,
            name=f"trough_side_{suffix}",
        )
    frame.visual(
        Box((0.075, 0.56, 0.235)),
        origin=Origin(xyz=(-0.735, 0.0, 0.365)),
        material=timber,
        name="trough_edge",
    )
    for x, suffix in ((-0.555, "0"), (0.555, "1"), (0.0, "2")):
        for y, side_suffix in ((-0.335, "0"), (0.335, "1")):
            frame.visual(
                Box((0.090, 0.140, 0.185)),
                origin=Origin(xyz=(x, y, 0.1925)),
                material=timber,
                name=f"race_bracket_{suffix}_{side_suffix}",
            )

    # Paired side frames.  Their central bearing pads tangent-support the axle,
    # while the rotating wheel remains visually framed between the two sides.
    for y, suffix in ((-0.385, "0"), (0.385, "1")):
        frame.visual(
            Box((1.22, 0.075, 0.080)),
            origin=Origin(xyz=(0.0, y, 0.060)),
            material=timber,
            name=f"side_sill_{suffix}",
        )
        for x, post_suffix in ((-0.555, "a"), (0.555, "b")):
            frame.visual(
                Box((0.085, 0.075, 0.720)),
                origin=Origin(xyz=(x, y, 0.420)),
                material=timber,
                name=f"side_post_{suffix}_{post_suffix}",
            )
        frame.visual(
            Box((0.110, 0.075, 0.690)),
            origin=Origin(xyz=(0.0, y, 0.410)),
            material=timber,
            name=("center_post_0" if suffix == "0" else "center_post_1"),
        )
        frame.visual(
            Box((0.210, 0.095, 0.060)),
            origin=Origin(xyz=(0.0, y, 0.785)),
            material=iron,
            name=("bearing_pad_0" if suffix == "0" else "bearing_pad_1"),
        )
        _beam_between(
            frame,
            (-0.555, y, 0.115),
            (0.0, y, 0.760),
            thickness=0.055,
            material=timber,
            name=f"diagonal_brace_{suffix}_0",
        )
        _beam_between(
            frame,
            (0.555, y, 0.115),
            (0.0, y, 0.760),
            thickness=0.055,
            material=timber,
            name=f"diagonal_brace_{suffix}_1",
        )

    for x, suffix in ((-0.555, "0"), (0.555, "1"), (0.0, "2")):
        frame.visual(
            Box((0.085, 0.845, 0.075)),
            origin=Origin(xyz=(x, 0.0, 0.060)),
            material=timber,
            name=f"base_crossbeam_{suffix}",
        )

    wheel = model.part("wheel")
    wheel.visual(
        mesh_from_cadquery(_make_wheel_wood(), "paddle_wheel_wood", tolerance=0.0015),
        material=dark_wood,
        name="wheel_wood",
    )
    wheel.visual(
        Cylinder(radius=0.035, length=0.980),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="axle",
    )
    wheel.visual(
        Cylinder(radius=0.065, length=0.420),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="hub_band",
    )
    for y, suffix in ((-0.500, "0"), (0.500, "1")):
        wheel.visual(
            Cylinder(radius=0.046, length=0.025),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=iron,
            name=f"axle_end_{suffix}",
        )

    model.articulation(
        "frame_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, 0.850)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=6.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    wheel = object_model.get_part("wheel")
    joint = object_model.get_articulation("frame_to_wheel")

    ctx.check(
        "wheel uses continuous axle rotation",
        joint.articulation_type == ArticulationType.CONTINUOUS and tuple(joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={joint.articulation_type}, axis={joint.axis}",
    )

    ctx.expect_contact(
        wheel,
        frame,
        elem_a="axle",
        elem_b="bearing_pad_0",
        contact_tol=0.001,
        name="axle rests on first bearing pad",
    )
    ctx.expect_contact(
        wheel,
        frame,
        elem_a="axle",
        elem_b="bearing_pad_1",
        contact_tol=0.001,
        name="axle rests on second bearing pad",
    )
    ctx.expect_gap(
        wheel,
        frame,
        axis="z",
        positive_elem="wheel_wood",
        negative_elem="race_floor",
        min_gap=0.060,
        max_gap=0.160,
        name="lower paddles clear the trough floor",
    )

    wheel_bounds = ctx.part_element_world_aabb(wheel, elem="wheel_wood")
    side_0 = ctx.part_element_world_aabb(frame, elem="center_post_0")
    side_1 = ctx.part_element_world_aabb(frame, elem="center_post_1")
    framed = False
    if wheel_bounds is not None and side_0 is not None and side_1 is not None:
        framed = side_0[1][1] < wheel_bounds[0][1] and side_1[0][1] > wheel_bounds[1][1]
    ctx.check(
        "paired side supports frame wheel width",
        framed,
        details=f"wheel={wheel_bounds}, side_0={side_0}, side_1={side_1}",
    )

    return ctx.report()


object_model = build_object_model()
