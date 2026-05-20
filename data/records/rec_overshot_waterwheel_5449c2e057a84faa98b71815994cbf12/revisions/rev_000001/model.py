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


AXLE_Z = 1.0
WHEEL_RADIUS = 0.82


def _cq_box(size: tuple[float, float, float], center: tuple[float, float, float]):
    return cq.Workplane("XY").box(*size).translate(center)


def _cq_box_between(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    section: tuple[float, float],
):
    """Rectangular beam between two YZ-plane points at constant X."""
    x0, y0, z0 = start
    x1, y1, z1 = end
    length = math.sqrt((y1 - y0) ** 2 + (z1 - z0) ** 2)
    # Local Z is the long axis; rotate about X to align with the YZ segment.
    angle = math.atan2(-(y1 - y0), z1 - z0)
    center = ((x0 + x1) * 0.5, (y0 + y1) * 0.5, (z0 + z1) * 0.5)
    return (
        cq.Workplane("XY")
        .box(section[0], section[1], length)
        .rotate((0, 0, 0), (1, 0, 0), math.degrees(angle))
        .translate(center)
    )


def _cq_oriented_wheel_box(
    size: tuple[float, float, float],
    x: float,
    local_y: float,
    local_z: float,
    theta: float,
):
    """Box whose local Z is radial and local Y is tangential about the X axle."""
    return (
        cq.Workplane("XY")
        .box(*size)
        .translate((x, local_y, local_z))
        .rotate((0, 0, 0), (1, 0, 0), -math.degrees(theta))
    )


def _cq_x_cylinder(
    radius: float,
    length: float,
    center: tuple[float, float, float],
):
    # A YZ workplane extrudes along +X; translate so the cylinder is centered.
    return (
        cq.Workplane("YZ")
        .circle(radius)
        .extrude(length)
        .translate((center[0] - 0.5 * length, center[1], center[2]))
    )


def _union_all(shapes: list[cq.Workplane]) -> cq.Workplane:
    combined = shapes[0]
    for shape in shapes[1:]:
        combined = combined.union(shape)
    return combined


def _build_wheel_body() -> cq.Workplane:
    pieces: list[cq.Workplane] = []
    rim_segments = 16
    bucket_count = 16
    spoke_count = 8

    rim_radius = 0.735
    rim_thickness = 0.095
    rim_chord = 2.0 * math.pi * rim_radius / rim_segments * 1.06
    for side_x in (-0.115, 0.115):
        for index in range(rim_segments):
            theta = 2.0 * math.pi * index / rim_segments
            pieces.append(
                _cq_oriented_wheel_box(
                    (0.050, rim_chord, rim_thickness),
                    side_x,
                    0.0,
                    rim_radius,
                    theta,
                )
            )

    for index in range(spoke_count):
        theta = 2.0 * math.pi * index / spoke_count
        pieces.append(
            _cq_oriented_wheel_box(
                (0.285, 0.055, 0.68),
                0.0,
                0.0,
                0.42,
                theta,
            )
        )

    # Shallow timber buckets: a back board, an outer lip, and one radial divider
    # for each pocket.  The small overlaps intentionally make them one wooden
    # rotating assembly with the side rims.
    bucket_tangent = 2.0 * math.pi * 0.80 / bucket_count * 0.78
    for index in range(bucket_count):
        theta = 2.0 * math.pi * (index + 0.5) / bucket_count
        pieces.append(
            _cq_oriented_wheel_box(
                (0.285, bucket_tangent, 0.060),
                0.0,
                0.0,
                0.770,
                theta,
            )
        )
        pieces.append(
            _cq_oriented_wheel_box(
                (0.285, bucket_tangent, 0.050),
                0.0,
                0.0,
                0.835,
                theta,
            )
        )
        pieces.append(
            _cq_oriented_wheel_box(
                (0.285, 0.055, 0.150),
                0.0,
                0.5 * bucket_tangent,
                0.805,
                theta,
            )
        )
        pieces.append(
            _cq_oriented_wheel_box(
                (0.285, 0.055, 0.150),
                0.0,
                -0.5 * bucket_tangent,
                0.805,
                theta,
            )
        )

    return _union_all(pieces)


def _build_frame_wood() -> cq.Workplane:
    pieces: list[cq.Workplane] = []
    side_xs = (-0.34, 0.34)

    for side_x in side_xs:
        pieces.append(_cq_box((0.080, 1.25, 0.080), (side_x, 0.0, 0.040)))
        pieces.append(_cq_box((0.085, 0.240, 0.260), (side_x, 0.0, AXLE_Z)))
        pieces.append(
            _cq_box_between(
                (side_x, -0.56, 0.080),
                (side_x, -0.075, 0.900),
                (0.075, 0.065),
            )
        )
        pieces.append(
            _cq_box_between(
                (side_x, 0.56, 0.080),
                (side_x, 0.075, 0.900),
                (0.075, 0.065),
            )
        )

    for y in (-0.56, 0.56):
        pieces.append(_cq_box((0.78, 0.080, 0.080), (0.0, y, 0.080)))

    # Chute support posts and the upper cross beam directly under the trough.
    for side_x in side_xs:
        pieces.append(_cq_box((0.070, 0.070, 1.86), (side_x, -0.56, 0.965)))
    pieces.append(_cq_box((0.78, 0.070, 0.080), (0.0, -0.56, 1.925)))

    # A short sloped overshot trough above the top-left quadrant of the wheel.
    pitch = -0.16
    chute_center = (0.0, -0.58, 1.985)

    def chute_box(
        size: tuple[float, float, float], local_center: tuple[float, float, float]
    ):
        return (
            cq.Workplane("XY")
            .box(*size)
            .translate(local_center)
            .rotate((0, 0, 0), (1, 0, 0), math.degrees(pitch))
            .translate(chute_center)
        )

    pieces.append(chute_box((0.350, 0.680, 0.035), (0.0, 0.0, 0.0)))
    pieces.append(chute_box((0.035, 0.680, 0.145), (-0.185, 0.0, 0.073)))
    pieces.append(chute_box((0.035, 0.680, 0.145), (0.185, 0.0, 0.073)))

    frame = _union_all(pieces)
    for side_x in side_xs:
        frame = frame.cut(_cq_x_cylinder(0.062, 0.140, (side_x, 0.0, AXLE_Z)))
    return frame


def _build_bearing_rings() -> cq.Workplane:
    rings = []
    for side_x in (-0.286, 0.286):
        ring = _cq_x_cylinder(0.092, 0.026, (side_x, 0.0, AXLE_Z)).cut(
            _cq_x_cylinder(0.030, 0.034, (side_x, 0.0, AXLE_Z))
        )
        rings.append(ring)
    return _union_all(rings)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="overshot_waterwheel")

    wood = model.material("warm_oak", rgba=(0.55, 0.32, 0.14, 1.0))
    dark_wood = model.material("dark_wet_wood", rgba=(0.32, 0.19, 0.10, 1.0))
    metal = model.material("dark_iron", rgba=(0.08, 0.075, 0.065, 1.0))
    water = model.material("chute_water", rgba=(0.25, 0.56, 0.88, 0.72))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_wood(), "frame_wood", tolerance=0.002),
        material=wood,
        name="frame_wood",
    )
    frame.visual(
        mesh_from_cadquery(_build_bearing_rings(), "bearing_rings", tolerance=0.0015),
        material=metal,
        name="bearing_rings",
    )
    frame.visual(
        Box((0.240, 0.560, 0.010)),
        origin=Origin(xyz=(0.0, -0.58, 2.000), rpy=(-0.16, 0.0, 0.0)),
        material=water,
        name="chute_water",
    )

    wheel = model.part("wheel")
    wheel.visual(
        mesh_from_cadquery(_build_wheel_body(), "wheel_body", tolerance=0.002),
        material=dark_wood,
        name="wheel_body",
    )
    wheel.visual(
        Cylinder(radius=0.125, length=0.230),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=wood,
        name="hub",
    )
    wheel.visual(
        Cylinder(radius=0.033, length=0.780),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="axle",
    )

    model.articulation(
        "axle_rotation",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, AXLE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=2.5),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    wheel = object_model.get_part("wheel")
    joint = object_model.get_articulation("axle_rotation")

    ctx.check(
        "wheel uses continuous horizontal axle",
        joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 3) for v in joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={joint.articulation_type}, axis={joint.axis}",
    )
    ctx.allow_overlap(
        frame,
        wheel,
        elem_a="bearing_rings",
        elem_b="axle",
        reason="The axle is intentionally captured as a lightly seated shaft inside the iron bearing bushings.",
    )
    ctx.expect_within(
        wheel,
        frame,
        axes="yz",
        inner_elem="axle",
        outer_elem="bearing_rings",
        margin=0.0,
        name="axle is centered in the bearing rings",
    )
    ctx.expect_overlap(
        frame,
        wheel,
        axes="x",
        elem_a="bearing_rings",
        elem_b="axle",
        min_overlap=0.020,
        name="axle passes through both bearing rings",
    )
    ctx.expect_within(
        wheel,
        frame,
        axes="x",
        inner_elem="wheel_body",
        outer_elem="frame_wood",
        margin=0.0,
        name="wheel body sits between side supports",
    )
    ctx.expect_gap(
        frame,
        wheel,
        axis="z",
        positive_elem="chute_water",
        negative_elem="wheel_body",
        min_gap=0.030,
        name="chute clears the top rim",
    )
    ctx.expect_overlap(
        frame,
        wheel,
        axes="xy",
        elem_a="chute_water",
        elem_b="wheel_body",
        min_overlap=0.12,
        name="chute is positioned over the wheel rim",
    )

    return ctx.report()


object_model = build_object_model()
