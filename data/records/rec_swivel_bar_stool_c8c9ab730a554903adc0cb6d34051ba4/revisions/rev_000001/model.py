from __future__ import annotations

import math

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
    TorusGeometry,
    mesh_from_geometry,
)


JOINT_Z = 0.665


def _midpoint(a, b):
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a, b) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(a, b):
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_tube(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _seat_shell_mesh():
    """Lathed molded seat with a downturned drip lip and slightly dished top."""
    profile = [
        (0.000, 0.112),
        (0.090, 0.106),
        (0.205, 0.108),
        (0.270, 0.120),
        (0.300, 0.106),
        (0.302, 0.086),
        (0.288, 0.068),
        (0.235, 0.062),
        (0.145, 0.074),
        (0.058, 0.096),
        (0.000, 0.098),
    ]
    return mesh_from_geometry(LatheGeometry(profile, segments=72, closed=True), "dished_seat_shell")


def _bearing_shroud_mesh():
    """Hollow rain shroud; the inner radius clears the fixed bearing housing."""
    outer = [(0.165, -0.032), (0.165, 0.070)]
    inner = [(0.145, 0.070), (0.145, -0.032)]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer,
            inner,
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
        "bearing_rain_shroud",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="outdoor_weatherproof_swivel_bar_stool")

    powder_graphite = model.material("powder_graphite", rgba=(0.11, 0.13, 0.14, 1.0))
    stainless = model.material("316_stainless", rgba=(0.72, 0.74, 0.72, 1.0))
    dark_rubber = model.material("uv_black_rubber", rgba=(0.015, 0.016, 0.015, 1.0))
    composite_seat = model.material("sealed_composite_seat", rgba=(0.08, 0.24, 0.25, 1.0))
    shadow = model.material("shadow_gap", rgba=(0.03, 0.035, 0.036, 1.0))

    footrest_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.245, tube=0.012, radial_segments=72, tubular_segments=16),
        "continuous_footrest_ring",
    )
    base_rim_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.335, tube=0.010, radial_segments=72, tubular_segments=12),
        "rolled_base_rim",
    )
    parent_seal_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.130, tube=0.006, radial_segments=64, tubular_segments=12),
        "fixed_bearing_seal",
    )
    seat_drip_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.288, tube=0.005, radial_segments=72, tubular_segments=12),
        "seat_drip_bead",
    )
    shroud_mesh = _bearing_shroud_mesh()
    seat_mesh = _seat_shell_mesh()

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.340, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=powder_graphite,
        name="floor_base",
    )
    pedestal.visual(
        base_rim_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=powder_graphite,
        name="rolled_base_rim",
    )
    pedestal.visual(
        Cylinder(radius=0.045, length=0.535),
        origin=Origin(xyz=(0.0, 0.0, 0.3125)),
        material=powder_graphite,
        name="sealed_pedestal_tube",
    )
    pedestal.visual(
        Cylinder(radius=0.070, length=0.065),
        origin=Origin(xyz=(0.0, 0.0, 0.6025)),
        material=powder_graphite,
        name="upper_welded_collar",
    )
    pedestal.visual(
        Cylinder(radius=0.128, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, JOINT_Z - 0.025)),
        material=powder_graphite,
        name="fixed_bearing_housing",
    )
    pedestal.visual(
        Cylinder(radius=0.118, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, JOINT_Z - 0.014)),
        material=stainless,
        name="fixed_race",
    )
    pedestal.visual(
        parent_seal_mesh,
        origin=Origin(xyz=(0.0, 0.0, JOINT_Z - 0.012)),
        material=dark_rubber,
        name="fixed_weather_seal",
    )
    pedestal.visual(
        footrest_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        material=stainless,
        name="footrest_ring",
    )

    for i, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        c = math.cos(angle)
        s = math.sin(angle)
        _add_tube(
            pedestal,
            (0.0, 0.0, 0.300),
            (0.245 * c, 0.245 * s, 0.300),
            0.010,
            stainless,
            name=f"footrest_spoke_{i}",
        )
        _add_tube(
            pedestal,
            (0.052 * c, 0.052 * s, 0.100),
            (0.155 * c, 0.155 * s, 0.045),
            0.010,
            powder_graphite,
            name=f"base_gusset_{i}",
        )
        pedestal.visual(
            Cylinder(radius=0.022, length=0.014),
            origin=Origin(xyz=(0.258 * c, 0.258 * s, 0.049)),
            material=stainless,
            name=f"anchor_washer_{i}",
        )
        pedestal.visual(
            Cylinder(radius=0.011, length=0.020),
            origin=Origin(xyz=(0.258 * c, 0.258 * s, 0.054)),
            material=stainless,
            name=f"anchor_bolt_{i}",
        )

    seat_stage = model.part("seat_stage")
    seat_stage.visual(
        Cylinder(radius=0.108, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=stainless,
        name="rotating_race",
    )
    seat_stage.visual(
        Cylinder(radius=0.075, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.082)),
        material=powder_graphite,
        name="central_hub",
    )
    seat_stage.visual(
        shroud_mesh,
        origin=Origin(),
        material=powder_graphite,
        name="bearing_rain_shroud",
    )
    seat_stage.visual(
        Cylinder(radius=0.185, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.082)),
        material=stainless,
        name="underseat_spider_plate",
    )
    seat_stage.visual(
        seat_mesh,
        origin=Origin(),
        material=composite_seat,
        name="molded_seat",
    )
    seat_stage.visual(
        seat_drip_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.071)),
        material=dark_rubber,
        name="outer_drip_gasket",
    )
    seat_stage.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.094, tube=0.005, radial_segments=64, tubular_segments=12),
            "upper_bearing_gasket",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=dark_rubber,
        name="upper_bearing_gasket",
    )
    for i, angle in enumerate((math.pi / 4.0, 3.0 * math.pi / 4.0, 5.0 * math.pi / 4.0, 7.0 * math.pi / 4.0)):
        c = math.cos(angle)
        s = math.sin(angle)
        seat_stage.visual(
            Cylinder(radius=0.014, length=0.014),
            origin=Origin(xyz=(0.132 * c, 0.132 * s, 0.098)),
            material=stainless,
            name=f"seat_bolt_head_{i}",
        )
        _add_tube(
            seat_stage,
            (0.040 * c, 0.040 * s, 0.080),
            (0.158 * c, 0.158 * s, 0.089),
            0.007,
            stainless,
            name=f"seat_spider_rib_{i}",
        )

    model.articulation(
        "seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=seat_stage,
        origin=Origin(xyz=(0.0, 0.0, JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=2.5),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    seat_stage = object_model.get_part("seat_stage")
    swivel = object_model.get_articulation("seat_swivel")

    ctx.expect_origin_distance(
        pedestal,
        seat_stage,
        axes="xy",
        max_dist=0.001,
        name="swivel axis is centered on the pedestal",
    )
    ctx.expect_gap(
        seat_stage,
        pedestal,
        axis="z",
        positive_elem="rotating_race",
        negative_elem="fixed_race",
        max_gap=0.001,
        max_penetration=0.0,
        name="bearing races seat without penetration",
    )
    ctx.expect_within(
        seat_stage,
        pedestal,
        axes="xy",
        inner_elem="rotating_race",
        outer_elem="fixed_race",
        margin=0.0,
        name="rotating race is captured within fixed race footprint",
    )
    ctx.expect_overlap(
        seat_stage,
        pedestal,
        axes="xy",
        elem_a="rotating_race",
        elem_b="fixed_race",
        min_overlap=0.18,
        name="bearing stack has broad supported contact area",
    )

    rest_position = ctx.part_world_position(seat_stage)
    with ctx.pose({swivel: math.pi / 2.0}):
        turned_position = ctx.part_world_position(seat_stage)
        ctx.expect_gap(
            seat_stage,
            pedestal,
            axis="z",
            positive_elem="rotating_race",
            negative_elem="fixed_race",
            max_gap=0.001,
            max_penetration=0.0,
            name="swiveled seat remains supported on bearing",
        )

    ctx.check(
        "swivel rotates about a stationary vertical bearing axis",
        rest_position is not None
        and turned_position is not None
        and abs(rest_position[0] - turned_position[0]) < 1e-6
        and abs(rest_position[1] - turned_position[1]) < 1e-6
        and abs(rest_position[2] - turned_position[2]) < 1e-6,
        details=f"rest={rest_position}, turned={turned_position}",
    )

    return ctx.report()


object_model = build_object_model()
