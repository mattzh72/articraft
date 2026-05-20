from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
    wire_from_points,
)


BASE_TOP_Z = 0.036
GLOBE_RADIUS = 0.120
AXIS_TILT = math.radians(23.5)
AXIS_UNIT = (math.sin(AXIS_TILT), 0.0, math.cos(AXIS_TILT))
POLE_IN_PEDESTAL = (0.082, 0.0, 0.184)


def _origin_between_xz(start: tuple[float, float, float], end: tuple[float, float, float]) -> tuple[Origin, float]:
    """Origin and length for a cylinder whose local Z axis runs between two XZ-plane points."""
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    # All authored struts lie in the XZ plane, so a pitch about Y aligns local +Z.
    pitch = math.atan2(dx, dz)
    midpoint = (
        (start[0] + end[0]) * 0.5,
        (start[1] + end[1]) * 0.5,
        (start[2] + end[2]) * 0.5,
    )
    return Origin(xyz=midpoint, rpy=(0.0, pitch, 0.0)), length


def _ring_base_shape() -> cq.Workplane:
    height = BASE_TOP_Z
    outer_radius = 0.210
    inner_radius = 0.116
    hub_radius = 0.056
    spoke_width = 0.022
    spoke_length = inner_radius - hub_radius + 0.016
    spoke_mid = (inner_radius + hub_radius) * 0.5

    base = cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(height)
    hub = cq.Workplane("XY").circle(hub_radius).extrude(height)
    shape = base.union(hub)

    for angle in (0.0, 120.0, 240.0):
        spoke = (
            cq.Workplane("XY")
            .box(spoke_length, spoke_width, height * 0.62)
            .translate((spoke_mid, 0.0, height * 0.50))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle)
        )
        shape = shape.union(spoke)

    return shape


def _meridian_arc(azimuth: float):
    points: list[tuple[float, float, float]] = []
    # Leave the support-side pole open so the single spindle and clip have room.
    for index in range(24):
        theta = 0.24 + (2.62 - 0.24) * index / 23
        radial = GLOBE_RADIUS * math.sin(theta)
        points.append(
            (
                radial * math.cos(azimuth),
                radial * math.sin(azimuth),
                GLOBE_RADIUS + GLOBE_RADIUS * math.cos(theta),
            )
        )
    return wire_from_points(
        points,
        radius=0.0015,
        radial_segments=10,
        closed_path=False,
        cap_ends=True,
        corner_mode="miter",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="minimalist_canted_globe")

    dark_metal = model.material("satin_black_metal", rgba=(0.020, 0.022, 0.024, 1.0))
    polished_metal = model.material("brushed_steel", rgba=(0.72, 0.70, 0.66, 1.0))
    bearing_shadow = model.material("dark_bearing_gap", rgba=(0.006, 0.006, 0.007, 1.0))
    ocean = model.material("muted_ocean_blue", rgba=(0.05, 0.22, 0.44, 1.0))
    grid = model.material("warm_ivory_grid", rgba=(0.92, 0.86, 0.70, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_ring_base_shape(), "ring_base", tolerance=0.0008, angular_tolerance=0.06),
        material=dark_metal,
        name="base_frame",
    )
    base.visual(
        Cylinder(radius=0.078, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z + 0.002)),
        material=bearing_shadow,
        name="bearing_shadow",
    )

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.075, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=polished_metal,
        name="turntable_disk",
    )
    pedestal.visual(
        mesh_from_geometry(TorusGeometry(radius=0.058, tube=0.0035, radial_segments=16, tubular_segments=72), "pedestal_bearing_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=dark_metal,
        name="bearing_ring",
    )
    pedestal.visual(
        Cylinder(radius=0.014, length=0.104),
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        material=polished_metal,
        name="upright_post",
    )

    socket_center = (
        POLE_IN_PEDESTAL[0] - AXIS_UNIT[0] * 0.026,
        0.0,
        POLE_IN_PEDESTAL[2] - AXIS_UNIT[2] * 0.026,
    )
    arm_end = (
        POLE_IN_PEDESTAL[0] - AXIS_UNIT[0] * 0.033,
        0.0,
        POLE_IN_PEDESTAL[2] - AXIS_UNIT[2] * 0.033,
    )
    arm_origin, arm_length = _origin_between_xz((0.0, 0.0, 0.112), arm_end)
    pedestal.visual(
        Cylinder(radius=0.011, length=arm_length),
        origin=arm_origin,
        material=polished_metal,
        name="canted_arm",
    )
    pedestal.visual(
        Cylinder(radius=0.017, length=0.020),
        origin=Origin(xyz=socket_center, rpy=(0.0, AXIS_TILT, 0.0)),
        material=polished_metal,
        name="spindle_socket",
    )
    pin_center = (
        POLE_IN_PEDESTAL[0] - AXIS_UNIT[0] * 0.010,
        0.0,
        POLE_IN_PEDESTAL[2] - AXIS_UNIT[2] * 0.010,
    )
    pedestal.visual(
        Cylinder(radius=0.006, length=0.056),
        origin=Origin(xyz=pin_center, rpy=(0.0, AXIS_TILT, 0.0)),
        material=dark_metal,
        name="spindle_pin",
    )

    globe = model.part("globe")
    globe.visual(
        Sphere(radius=GLOBE_RADIUS),
        origin=Origin(xyz=(0.0, 0.0, GLOBE_RADIUS)),
        material=ocean,
        name="globe_shell",
    )
    globe.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=grid,
        name="pole_socket",
    )
    globe.visual(
        mesh_from_geometry(TorusGeometry(radius=GLOBE_RADIUS + 0.0008, tube=0.0016, radial_segments=14, tubular_segments=96), "equator_band"),
        origin=Origin(xyz=(0.0, 0.0, GLOBE_RADIUS)),
        material=grid,
        name="equator_band",
    )
    for label, latitude in (("latitude_north", math.radians(35.0)), ("latitude_south", math.radians(-35.0))):
        globe.visual(
            mesh_from_geometry(
                TorusGeometry(
                    radius=GLOBE_RADIUS * math.cos(latitude) + 0.0008,
                    tube=0.0013,
                    radial_segments=12,
                    tubular_segments=80,
                ),
                label,
            ),
            origin=Origin(xyz=(0.0, 0.0, GLOBE_RADIUS + GLOBE_RADIUS * math.sin(latitude))),
            material=grid,
            name=label,
        )
    for index, azimuth in enumerate((0.0, math.pi * 0.5, math.pi * 0.25)):
        globe.visual(
            mesh_from_geometry(_meridian_arc(azimuth), f"meridian_{index}"),
            material=grid,
            name=f"meridian_{index}",
        )

    model.articulation(
        "base_to_pedestal",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=pedestal,
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=1.0),
        motion_properties=MotionProperties(damping=0.015, friction=0.01),
    )
    model.articulation(
        "pedestal_to_globe",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=globe,
        origin=Origin(xyz=POLE_IN_PEDESTAL, rpy=(0.0, AXIS_TILT, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=3.5),
        motion_properties=MotionProperties(damping=0.004, friction=0.002),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    pedestal = object_model.get_part("pedestal")
    globe = object_model.get_part("globe")
    turntable = object_model.get_articulation("base_to_pedestal")
    globe_spin = object_model.get_articulation("pedestal_to_globe")

    ctx.allow_overlap(
        pedestal,
        globe,
        elem_a="spindle_pin",
        elem_b="pole_socket",
        reason="The single-sided spindle is intentionally captured inside the rotating polar socket.",
    )
    ctx.allow_overlap(
        pedestal,
        globe,
        elem_a="spindle_pin",
        elem_b="globe_shell",
        reason="The pin tip intentionally enters the globe at its polar bushing so the sphere stays retained.",
    )

    ctx.check(
        "turntable joint is continuous",
        turntable.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={turntable.articulation_type}",
    )
    ctx.check(
        "globe spin joint is continuous",
        globe_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={globe_spin.articulation_type}",
    )
    ctx.check(
        "globe spin axis is canted",
        abs(globe_spin.origin.rpy[1] - AXIS_TILT) < 1.0e-6,
        details=f"tilt={globe_spin.origin.rpy[1]} expected={AXIS_TILT}",
    )
    ctx.expect_gap(
        pedestal,
        base,
        axis="z",
        positive_elem="turntable_disk",
        negative_elem="base_frame",
        max_gap=0.001,
        max_penetration=0.0,
        name="turntable sits on base bearing",
    )
    ctx.expect_overlap(
        pedestal,
        globe,
        axes="xyz",
        elem_a="spindle_pin",
        elem_b="pole_socket",
        min_overlap=0.003,
        name="spindle is captured by polar socket",
    )
    ctx.expect_overlap(
        pedestal,
        globe,
        axes="xyz",
        elem_a="spindle_pin",
        elem_b="globe_shell",
        min_overlap=0.006,
        name="spindle tip remains inserted in globe",
    )

    rest_pole = ctx.part_world_position(globe)
    with ctx.pose({globe_spin: 1.7}):
        spun_pole = ctx.part_world_position(globe)
    ctx.check(
        "globe spin keeps captured pole fixed",
        rest_pole is not None
        and spun_pole is not None
        and math.dist(rest_pole, spun_pole) < 1.0e-7,
        details=f"rest={rest_pole}, spun={spun_pole}",
    )

    with ctx.pose({turntable: math.pi * 0.5}):
        turned_pole = ctx.part_world_position(globe)
    ctx.check(
        "pedestal turns about vertical base axis",
        rest_pole is not None
        and turned_pole is not None
        and abs(turned_pole[0]) < 0.004
        and turned_pole[1] > 0.078
        and abs(turned_pole[2] - rest_pole[2]) < 0.001,
        details=f"rest={rest_pole}, turned={turned_pole}",
    )

    return ctx.report()


object_model = build_object_model()
