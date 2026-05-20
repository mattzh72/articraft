from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _regular_polygon(radius: float, sides: int, *, angle_offset: float = 0.0) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(angle_offset + 2.0 * math.pi * i / sides),
            radius * math.sin(angle_offset + 2.0 * math.pi * i / sides),
        )
        for i in range(sides)
    ]


def _prism_mesh(
    profile: list[tuple[float, float]],
    z0: float,
    z1: float,
) -> MeshGeometry:
    """Closed vertical prism from a counter-clockwise XY profile."""
    geom = MeshGeometry()
    bottom = [geom.add_vertex(x, y, z0) for x, y in profile]
    top = [geom.add_vertex(x, y, z1) for x, y in profile]
    n = len(profile)
    for i in range(n):
        j = (i + 1) % n
        geom.add_face(bottom[i], bottom[j], top[j])
        geom.add_face(bottom[i], top[j], top[i])

    bottom_center = geom.add_vertex(0.0, 0.0, z0)
    top_center = geom.add_vertex(0.0, 0.0, z1)
    for i in range(n):
        j = (i + 1) % n
        geom.add_face(bottom_center, bottom[i], bottom[j])
        geom.add_face(top_center, top[j], top[i])
    return geom


def _frustum_mesh(
    bottom_radius: float,
    top_radius: float,
    z0: float,
    z1: float,
    *,
    sides: int = 48,
    angle_offset: float = 0.0,
) -> MeshGeometry:
    geom = MeshGeometry()
    bottom_profile = _regular_polygon(bottom_radius, sides, angle_offset=angle_offset)
    top_profile = _regular_polygon(top_radius, sides, angle_offset=angle_offset)
    bottom = [geom.add_vertex(x, y, z0) for x, y in bottom_profile]
    top = [geom.add_vertex(x, y, z1) for x, y in top_profile]
    for i in range(sides):
        j = (i + 1) % sides
        geom.add_face(bottom[i], bottom[j], top[j])
        geom.add_face(bottom[i], top[j], top[i])

    bottom_center = geom.add_vertex(0.0, 0.0, z0)
    top_center = geom.add_vertex(0.0, 0.0, z1)
    for i in range(sides):
        j = (i + 1) % sides
        geom.add_face(bottom_center, bottom[i], bottom[j])
        geom.add_face(top_center, top[j], top[i])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="squat_harbor_lighthouse")

    whitewash = model.material("whitewashed_plaster", rgba=(0.88, 0.86, 0.78, 1.0))
    red_trim = model.material("weathered_red_trim", rgba=(0.58, 0.08, 0.06, 1.0))
    dark_metal = model.material("dark_bronze_metal", rgba=(0.05, 0.045, 0.04, 1.0))
    glass = model.material("slightly_green_glass", rgba=(0.50, 0.80, 0.86, 0.36))
    warm_glow = model.material("warm_beacon_glass", rgba=(1.0, 0.78, 0.22, 0.90))
    concrete = model.material("salt_stained_concrete", rgba=(0.48, 0.49, 0.47, 1.0))
    black_paint = model.material("black_painted_door", rgba=(0.015, 0.018, 0.02, 1.0))

    tower = model.part("tower")

    # Squat masonry base and gently tapered tower body.
    tower.visual(
        Cylinder(radius=1.65, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=concrete,
        name="foundation_plinth",
    )
    tower.visual(
        mesh_from_geometry(_frustum_mesh(1.34, 1.04, 0.24, 2.75, sides=64), "tapered_tower"),
        material=whitewash,
        name="tapered_tower",
    )
    tower.visual(
        Cylinder(radius=1.29, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 1.08)),
        material=red_trim,
        name="lower_red_belt",
    )
    tower.visual(
        Cylinder(radius=1.09, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 2.48)),
        material=red_trim,
        name="upper_red_belt",
    )

    # Surface details are slightly proud so they read as mounted trim rather than floating decals.
    tower.visual(
        Box((0.56, 0.045, 1.05)),
        origin=Origin(xyz=(0.0, -1.315, 0.82)),
        material=black_paint,
        name="front_door",
    )
    tower.visual(
        Box((0.34, 0.040, 0.36)),
        origin=Origin(xyz=(0.0, -1.165, 1.88)),
        material=glass,
        name="front_window",
    )
    tower.visual(
        Box((0.040, 0.34, 0.32)),
        origin=Origin(xyz=(1.125, 0.0, 1.62)),
        material=glass,
        name="side_window",
    )

    octagon_angle = math.pi / 8.0
    lantern_radius = 0.90
    lantern_apothem = lantern_radius * math.cos(math.pi / 8.0)
    lantern_side = 2.0 * lantern_radius * math.sin(math.pi / 8.0)
    octagon = _regular_polygon(lantern_radius, 8, angle_offset=octagon_angle)

    # Gallery deck, polygonal lantern enclosure, rails, and roof cap.
    tower.visual(
        mesh_from_geometry(_prism_mesh(_regular_polygon(1.42, 8, angle_offset=octagon_angle), 2.75, 2.93), "gallery_deck"),
        material=concrete,
        name="gallery_deck",
    )
    tower.visual(
        mesh_from_geometry(_prism_mesh(octagon, 2.93, 3.10), "lantern_floor"),
        material=red_trim,
        name="lantern_floor",
    )
    tower.visual(
        mesh_from_geometry(_prism_mesh(octagon, 4.04, 4.20), "upper_lintel"),
        material=red_trim,
        name="upper_lintel",
    )

    for i in range(8):
        vertex_angle = octagon_angle + 2.0 * math.pi * i / 8.0
        x = lantern_radius * math.cos(vertex_angle)
        y = lantern_radius * math.sin(vertex_angle)
        tower.visual(
            Cylinder(radius=0.040, length=1.02),
            origin=Origin(xyz=(x, y, 3.57)),
            material=dark_metal,
            name=f"lantern_post_{i}",
        )

        normal_angle = octagon_angle + 2.0 * math.pi * (i + 0.5) / 8.0
        nx = math.cos(normal_angle)
        ny = math.sin(normal_angle)
        yaw = normal_angle - math.pi / 2.0
        tower.visual(
            Box((lantern_side * 0.76, 0.030, 0.98)),
            origin=Origin(xyz=(lantern_apothem * nx, lantern_apothem * ny, 3.57), rpy=(0.0, 0.0, yaw)),
            material=glass,
            name=f"glass_pane_{i}",
        )

    rail_radius = 1.30
    rail_apothem = rail_radius * math.cos(math.pi / 8.0)
    rail_side = 2.0 * rail_radius * math.sin(math.pi / 8.0)
    for i in range(8):
        vertex_angle = octagon_angle + 2.0 * math.pi * i / 8.0
        tower.visual(
            Cylinder(radius=0.028, length=0.45),
            origin=Origin(xyz=(rail_radius * math.cos(vertex_angle), rail_radius * math.sin(vertex_angle), 3.155)),
            material=dark_metal,
            name=f"gallery_stanchion_{i}",
        )
        normal_angle = octagon_angle + 2.0 * math.pi * (i + 0.5) / 8.0
        nx = math.cos(normal_angle)
        ny = math.sin(normal_angle)
        tower.visual(
            Box((rail_side + 0.05, 0.035, 0.040)),
            origin=Origin(xyz=(rail_apothem * nx, rail_apothem * ny, 3.31), rpy=(0.0, 0.0, normal_angle - math.pi / 2.0)),
            material=dark_metal,
            name=f"gallery_rail_{i}",
        )

    tower.visual(
        mesh_from_geometry(_prism_mesh(_regular_polygon(1.10, 8, angle_offset=octagon_angle), 4.20, 4.30), "roof_eave"),
        material=red_trim,
        name="roof_eave",
    )
    tower.visual(
        mesh_from_geometry(_frustum_mesh(1.06, 0.34, 4.30, 4.78, sides=8, angle_offset=octagon_angle), "roof_cap"),
        material=red_trim,
        name="roof_cap",
    )
    tower.visual(
        Cylinder(radius=0.18, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 4.86)),
        material=dark_metal,
        name="vent_cupola",
    )
    tower.visual(
        Sphere(radius=0.09),
        origin=Origin(xyz=(0.0, 0.0, 4.99)),
        material=dark_metal,
        name="roof_finial",
    )

    # Fixed central pedestal that carries the rotating beacon inside the lantern room.
    tower.visual(
        Cylinder(radius=0.18, length=0.48),
        origin=Origin(xyz=(0.0, 0.0, 3.34)),
        material=dark_metal,
        name="pedestal",
    )

    beacon = model.part("beacon")
    beacon.visual(
        Cylinder(radius=0.24, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=dark_metal,
        name="bearing_disk",
    )
    beacon.visual(
        Cylinder(radius=0.045, length=0.34),
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
        material=dark_metal,
        name="spindle",
    )
    beacon.visual(
        Box((0.72, 0.055, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        material=dark_metal,
        name="cross_arm",
    )
    for side, x in (("front", 0.29), ("rear", -0.29)):
        beacon.visual(
            Box((0.050, 0.070, 0.28)),
            origin=Origin(xyz=(x, 0.0, 0.255)),
            material=dark_metal,
            name=f"{side}_yoke",
        )
    beacon.visual(
        Cylinder(radius=0.110, length=0.62),
        origin=Origin(xyz=(0.0, 0.0, 0.300), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_glow,
        name="lamp_barrel",
    )
    beacon.visual(
        Sphere(radius=0.135),
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        material=warm_glow,
        name="lamp_core",
    )
    beacon.visual(
        Cylinder(radius=0.160, length=0.080),
        origin=Origin(xyz=(0.340, 0.0, 0.300), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="lens_front",
    )
    beacon.visual(
        Cylinder(radius=0.160, length=0.080),
        origin=Origin(xyz=(-0.340, 0.0, 0.300), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="lens_rear",
    )

    model.articulation(
        "pedestal_to_beacon",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=beacon,
        origin=Origin(xyz=(0.0, 0.0, 3.58)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    beacon = object_model.get_part("beacon")
    spin = object_model.get_articulation("pedestal_to_beacon")

    ctx.check(
        "beacon uses continuous vertical rotation",
        spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.expect_contact(
        beacon,
        tower,
        elem_a="bearing_disk",
        elem_b="pedestal",
        contact_tol=0.001,
        name="beacon bearing is seated on pedestal",
    )
    ctx.expect_overlap(
        beacon,
        tower,
        axes="xy",
        elem_a="bearing_disk",
        elem_b="pedestal",
        min_overlap=0.30,
        name="bearing disk centered over pedestal footprint",
    )
    ctx.expect_within(
        beacon,
        tower,
        axes="xy",
        inner_elem="lamp_barrel",
        outer_elem="lantern_floor",
        margin=0.0,
        name="rotating lamp fits within lantern enclosure",
    )

    rest_aabb = ctx.part_element_world_aabb(beacon, elem="lens_front")
    rest_center = None
    if rest_aabb is not None:
        lo, hi = rest_aabb
        rest_center = ((lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0, (lo[2] + hi[2]) / 2.0)
    with ctx.pose({spin: math.pi / 2.0}):
        turned_aabb = ctx.part_element_world_aabb(beacon, elem="lens_front")
        turned_center = None
        if turned_aabb is not None:
            lo, hi = turned_aabb
            turned_center = ((lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0, (lo[2] + hi[2]) / 2.0)
        ctx.check(
            "lens sweeps around central axis",
            rest_center is not None
            and turned_center is not None
            and rest_center[0] > 0.30
            and abs(rest_center[1]) < 0.03
            and turned_center[1] > 0.30
            and abs(turned_center[0]) < 0.03,
            details=f"rest={rest_center}, turned={turned_center}",
        )

    return ctx.report()


object_model = build_object_model()
