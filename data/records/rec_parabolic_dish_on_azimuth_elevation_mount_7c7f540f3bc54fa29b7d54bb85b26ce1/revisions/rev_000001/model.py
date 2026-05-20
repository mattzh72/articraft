from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    LatheGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _tube_between(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    *,
    segments: int = 16,
) -> MeshGeometry:
    """Build a capped straight tube mesh between two local points."""
    sx, sy, sz = start
    ex, ey, ez = end
    vx, vy, vz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(vx * vx + vy * vy + vz * vz)
    if length <= 1e-9:
        return MeshGeometry()

    geom = CylinderGeometry(radius, length, radial_segments=segments, closed=True)
    dx, dy, dz = vx / length, vy / length, vz / length
    # CylinderGeometry is centered on local +Z; rotate that axis onto the span.
    dot = max(-1.0, min(1.0, dz))
    angle = math.acos(dot)
    ax, ay, az = -dy, dx, 0.0
    axis_len = math.sqrt(ax * ax + ay * ay + az * az)
    if axis_len > 1e-9 and angle > 1e-9:
        geom.rotate((ax / axis_len, ay / axis_len, az / axis_len), angle)
    elif dz < 0.0:
        geom.rotate((1.0, 0.0, 0.0), math.pi)

    geom.translate((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5)
    return geom


def _build_truss_geometry() -> MeshGeometry:
    geom = MeshGeometry()
    rear_nodes = [
        (-0.08, 0.0, 0.0),
        (0.07, 0.0, 0.0),
    ]
    rim_nodes = [
        (0.58, 0.54, 0.38),
        (0.58, -0.54, 0.38),
        (0.58, 0.54, -0.38),
        (0.58, -0.54, -0.38),
        (0.58, 0.0, 0.63),
        (0.58, 0.0, -0.63),
    ]
    for node in rear_nodes:
        for rim in rim_nodes:
            geom.merge(_tube_between(node, rim, 0.015, segments=12))
    # A short rear vertical mast makes the rear support read as a rigid welded cage.
    geom.merge(_tube_between((-0.08, 0.0, -0.16), (-0.08, 0.0, 0.16), 0.018, segments=12))
    return geom


def _build_feed_geometry() -> MeshGeometry:
    geom = MeshGeometry()
    rim_points = [
        (0.63, 0.00, 0.742),
        (0.63, 0.00, -0.742),
        (0.63, 0.742, 0.00),
        (0.63, -0.742, 0.00),
    ]
    collar_points = [
        (0.91, 0.00, 0.075),
        (0.91, 0.00, -0.075),
        (0.91, 0.075, 0.00),
        (0.91, -0.075, 0.00),
    ]
    for a, b in zip(rim_points, collar_points):
        geom.merge(_tube_between(a, b, 0.011, segments=12))

    horn = LatheGeometry.from_shell_profiles(
        [(0.105, 0.00), (0.060, 0.08), (0.032, 0.18)],
        [(0.078, 0.018), (0.043, 0.085), (0.018, 0.155)],
        segments=48,
        start_cap="round",
        end_cap="flat",
        lip_samples=6,
    )
    # Lathe axis local Z -> dish boresight +X; the wide mouth faces the reflector.
    horn.rotate_y(math.pi / 2.0)
    horn.translate(0.90, 0.0, 0.0)
    geom.merge(horn)

    # A small collar ties the feed horn throat to the four support rods.
    geom.merge(_tube_between((0.90, -0.085, 0.0), (0.90, 0.085, 0.0), 0.018, segments=12))
    geom.merge(_tube_between((0.90, 0.0, -0.085), (0.90, 0.0, 0.085), 0.018, segments=12))
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ground_tracking_dish")

    concrete = Material("mat_concrete", rgba=(0.42, 0.41, 0.38, 1.0))
    dark_steel = Material("mat_dark_steel", rgba=(0.09, 0.11, 0.13, 1.0))
    blue_steel = Material("mat_blue_steel", rgba=(0.08, 0.15, 0.22, 1.0))
    bearing = Material("mat_bearing", rgba=(0.02, 0.025, 0.03, 1.0))
    reflector = Material("mat_reflector", rgba=(0.82, 0.84, 0.82, 1.0))
    feed_mat = Material("mat_feed", rgba=(0.18, 0.20, 0.19, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.58, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=concrete,
        name="ground_pad",
    )
    pedestal.visual(
        Cylinder(radius=0.18, length=0.55),
        origin=Origin(xyz=(0.0, 0.0, 0.395)),
        material=dark_steel,
        name="column",
    )
    pedestal.visual(
        Cylinder(radius=0.34, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.70)),
        material=bearing,
        name="bearing_ring",
    )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.42, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=blue_steel,
        name="turntable_disk",
    )
    turntable.visual(
        Cylinder(radius=0.27, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.19)),
        material=dark_steel,
        name="central_hub",
    )
    turntable.visual(
        Box((0.24, 0.92, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material=blue_steel,
        name="yoke_bridge",
    )
    for y, name in [(-0.4565, "yoke_0"), (0.4565, "yoke_1")]:
        turntable.visual(
            Box((0.18, 0.12, 1.05)),
            origin=Origin(xyz=(0.0, y, 0.625)),
            material=blue_steel,
            name=name,
        )
        turntable.visual(
            Cylinder(radius=0.18, length=0.028),
            origin=Origin(
                xyz=(0.0, math.copysign(0.4105, y), 0.86),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=bearing,
            name=f"pivot_pad_{name[-1]}",
        )
    turntable.visual(
        Box((0.16, 0.82, 0.12)),
        origin=Origin(xyz=(-0.16, 0.0, 0.42), rpy=(0.0, -0.28, 0.0)),
        material=blue_steel,
        name="rear_gusset",
    )

    dish = model.part("dish")
    dish_shell = LatheGeometry.from_shell_profiles(
        [(0.12, 0.00), (0.34, 0.11), (0.56, 0.28), (0.75, 0.46)],
        [(0.055, 0.035), (0.295, 0.135), (0.520, 0.300), (0.700, 0.430)],
        segments=96,
        start_cap="round",
        end_cap="round",
        lip_samples=10,
    )
    dish_shell.rotate_y(math.pi / 2.0)
    dish_shell.translate(0.17, 0.0, 0.0)
    dish.visual(
        mesh_from_geometry(dish_shell, "deep_reflector"),
        material=reflector,
        name="deep_reflector",
    )
    rim = TorusGeometry(radius=0.735, tube=0.025, radial_segments=18, tubular_segments=96)
    rim.rotate_y(math.pi / 2.0)
    rim.translate(0.63, 0.0, 0.0)
    dish.visual(mesh_from_geometry(rim, "rim_ring"), material=reflector, name="rim_ring")
    dish.visual(
        Cylinder(radius=0.085, length=0.78),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bearing,
        name="trunnion_shaft",
    )
    dish.visual(
        Cylinder(radius=0.14, length=0.037),
        origin=Origin(xyz=(0.0, -0.378, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bearing,
        name="bearing_0",
    )
    dish.visual(
        Cylinder(radius=0.14, length=0.037),
        origin=Origin(xyz=(0.0, 0.378, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bearing,
        name="bearing_1",
    )
    dish.visual(
        Cylinder(radius=0.16, length=0.22),
        origin=Origin(xyz=(0.06, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="rear_hub",
    )
    dish.visual(
        mesh_from_geometry(_build_truss_geometry(), "rear_truss"),
        material=dark_steel,
        name="rear_truss",
    )
    dish.visual(
        mesh_from_geometry(_build_feed_geometry(), "feed_assembly"),
        material=feed_mat,
        name="feed_assembly",
    )

    model.articulation(
        "base_to_turntable",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.73)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=0.8,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "turntable_to_dish",
        ArticulationType.REVOLUTE,
        parent=turntable,
        child=dish,
        origin=Origin(xyz=(0.0, 0.0, 0.86)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.55,
            lower=0.0,
            upper=math.radians(70.0),
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    turntable = object_model.get_part("turntable")
    dish = object_model.get_part("dish")
    azimuth = object_model.get_articulation("base_to_turntable")
    elevation = object_model.get_articulation("turntable_to_dish")

    ctx.expect_gap(
        turntable,
        pedestal,
        axis="z",
        positive_elem="turntable_disk",
        negative_elem="bearing_ring",
        max_gap=0.002,
        max_penetration=0.0,
        name="turntable sits on bearing ring",
    )
    ctx.expect_gap(
        turntable,
        dish,
        axis="y",
        positive_elem="yoke_1",
        negative_elem="bearing_1",
        min_gap=0.0,
        max_gap=0.006,
        name="positive yoke captures bearing",
    )
    ctx.expect_gap(
        dish,
        turntable,
        axis="y",
        positive_elem="bearing_0",
        negative_elem="yoke_0",
        min_gap=0.0,
        max_gap=0.006,
        name="negative yoke captures bearing",
    )

    limits = azimuth.motion_limits
    ctx.check(
        "azimuth has full sweep",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower <= -math.pi
        and limits.upper >= math.pi,
        details=f"limits={limits}",
    )
    el_limits = elevation.motion_limits
    ctx.check(
        "elevation range is seventy degrees",
        el_limits is not None
        and abs((el_limits.lower or 0.0) - 0.0) < 1e-6
        and el_limits.upper is not None
        and abs(el_limits.upper - math.radians(70.0)) < 1e-6,
        details=f"limits={el_limits}",
    )

    rest_feed = ctx.part_element_world_aabb(dish, elem="feed_assembly")
    with ctx.pose({elevation: math.radians(70.0)}):
        raised_feed = ctx.part_element_world_aabb(dish, elem="feed_assembly")
    rest_center_z = None if rest_feed is None else (rest_feed[0][2] + rest_feed[1][2]) * 0.5
    raised_center_z = None if raised_feed is None else (raised_feed[0][2] + raised_feed[1][2]) * 0.5
    ctx.check(
        "elevation raises feed horn",
        rest_center_z is not None
        and raised_center_z is not None
        and raised_center_z > rest_center_z + 0.45,
        details=f"rest_z={rest_center_z}, raised_z={raised_center_z}",
    )

    return ctx.report()


object_model = build_object_model()
