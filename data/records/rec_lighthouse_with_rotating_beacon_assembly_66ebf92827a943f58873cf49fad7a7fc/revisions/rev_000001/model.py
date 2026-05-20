from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    ConeGeometry,
    CylinderGeometry,
    DomeGeometry,
    LatheGeometry,
    TorusGeometry,
    mesh_from_geometry,
)


def _tower_radius(z: float) -> float:
    """Approximate outside radius of the main tapered masonry tower."""
    points = [
        (0.00, 3.15),
        (0.48, 3.15),
        (0.95, 2.72),
        (1.25, 2.55),
        (4.50, 2.42),
        (9.50, 2.14),
        (15.50, 1.78),
        (19.80, 1.38),
        (20.45, 1.20),
    ]
    if z <= points[0][0]:
        return points[0][1]
    for (z0, r0), (z1, r1) in zip(points, points[1:]):
        if z <= z1:
            t = (z - z0) / (z1 - z0)
            return r0 + (r1 - r0) * t
    return points[-1][1]


def _radial_box(
    part,
    *,
    name: str,
    theta: float,
    z: float,
    radius: float,
    size: tuple[float, float, float],
    material,
    embed: float = 0.0,
) -> None:
    """Mount a box so its local +X points radially outward from the tower."""
    x = (radius + size[0] / 2.0 - embed) * math.cos(theta)
    y = (radius + size[0] / 2.0 - embed) * math.sin(theta)
    part.visual(
        Box(size),
        origin=Origin(xyz=(x, y, z), rpy=(0.0, 0.0, theta)),
        material=material,
        name=name,
    )


def _add_window(part, theta: float, z: float, index: int, glass, stone) -> None:
    radius = _tower_radius(z)
    _radial_box(
        part,
        name=f"window_glass_{index}",
        theta=theta,
        z=z,
        radius=radius,
        size=(0.08, 0.72, 1.05),
        material=glass,
        embed=0.012,
    )
    _radial_box(
        part,
        name=f"window_sill_{index}",
        theta=theta,
        z=z - 0.63,
        radius=radius + 0.035,
        size=(0.18, 0.96, 0.14),
        material=stone,
        embed=0.020,
    )
    _radial_box(
        part,
        name=f"window_head_{index}",
        theta=theta,
        z=z + 0.63,
        radius=radius + 0.035,
        size=(0.18, 0.96, 0.14),
        material=stone,
        embed=0.020,
    )
    _radial_box(
        part,
        name=f"window_jamb_a_{index}",
        theta=theta,
        z=z,
        radius=radius + 0.035,
        size=(0.16, 0.12, 1.32),
        material=stone,
        embed=0.020,
    )
    # local offset to one side of the radial panel
    tangent_x = -math.sin(theta)
    tangent_y = math.cos(theta)
    visual = part.visual(
        Box((0.16, 0.12, 1.32)),
        origin=Origin(
            xyz=(
                (radius + 0.08 - 0.020) * math.cos(theta) + 0.42 * tangent_x,
                (radius + 0.08 - 0.020) * math.sin(theta) + 0.42 * tangent_y,
                z,
            ),
            rpy=(0.0, 0.0, theta),
        ),
        material=stone,
        name=f"window_jamb_b_{index}",
    )
    # Move the first jamb to the opposite side after creation by replacing its
    # origin with a symmetric lateral offset.
    part.get_visual(f"window_jamb_a_{index}").origin = Origin(
        xyz=(
            (radius + 0.08 - 0.020) * math.cos(theta) - 0.42 * tangent_x,
            (radius + 0.08 - 0.020) * math.sin(theta) - 0.42 * tangent_y,
            z,
        ),
        rpy=(0.0, 0.0, theta),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rotating_lighthouse")

    whitewash = model.material("warm_whitewash", rgba=(0.92, 0.88, 0.78, 1.0))
    red_paint = model.material("weathered_red_paint", rgba=(0.70, 0.06, 0.035, 1.0))
    dark_stone = model.material("dark_granite", rgba=(0.24, 0.23, 0.22, 1.0))
    pale_stone = model.material("pale_limestone", rgba=(0.78, 0.73, 0.63, 1.0))
    glass = model.material("green_tinted_glass", rgba=(0.45, 0.76, 0.82, 0.38))
    dark_glass = model.material("dark_window_glass", rgba=(0.035, 0.055, 0.07, 1.0))
    black_iron = model.material("blackened_iron", rgba=(0.02, 0.02, 0.018, 1.0))
    brass = model.material("brushed_brass", rgba=(0.86, 0.62, 0.22, 1.0))
    warm_light = model.material("warm_lamp_glow", rgba=(1.0, 0.80, 0.28, 0.92))

    tower = model.part("tower")

    # A real-scale tapered masonry shaft: broad at the entrance and narrower
    # under the gallery, with a substantial hidden core rather than a token pole.
    tower_profile = [
        (0.0, 0.00),
        (3.15, 0.00),
        (3.15, 0.48),
        (2.72, 0.95),
        (2.55, 1.25),
        (2.42, 4.5),
        (2.14, 9.5),
        (1.78, 15.5),
        (1.38, 19.8),
        (1.20, 20.45),
        (0.0, 20.45),
    ]
    tower.visual(
        mesh_from_geometry(LatheGeometry(tower_profile, segments=96, closed=True), "tapered_tower"),
        material=whitewash,
        name="tapered_tower",
    )

    # Heavy base and foundation plinths.
    tower.visual(
        Cylinder(radius=3.85, length=0.38),
        origin=Origin(xyz=(0.0, 0.0, 0.19)),
        material=dark_stone,
        name="foundation_disk",
    )
    tower.visual(
        Cylinder(radius=3.35, length=0.48),
        origin=Origin(xyz=(0.0, 0.0, 0.62)),
        material=pale_stone,
        name="base_plinth",
    )
    tower.visual(
        mesh_from_geometry(TorusGeometry(3.20, 0.09, radial_segments=18, tubular_segments=96), "base_torus"),
        origin=Origin(xyz=(0.0, 0.0, 0.92)),
        material=pale_stone,
        name="base_round_molding",
    )

    # Classic painted day-mark bands follow the taper of the tower.
    for i, (z0, z1) in enumerate(((5.8, 7.55), (12.1, 13.9), (18.0, 19.25))):
        r0 = _tower_radius(z0)
        r1 = _tower_radius(z1)
        band_profile = [
            (r0 - 0.012, z0),
            (r0 + 0.045, z0),
            (r1 + 0.045, z1),
            (r1 - 0.012, z1),
        ]
        tower.visual(
            mesh_from_geometry(LatheGeometry(band_profile, segments=96, closed=True), f"red_band_{i}"),
            material=red_paint,
            name=f"red_band_{i}",
        )

    # Entrance with raised stone surround and a recessed dark green door.
    theta_front = -math.pi / 2.0
    entrance_radius = _tower_radius(1.65)
    _radial_box(
        tower,
        name="entrance_door",
        theta=theta_front,
        z=1.55,
        radius=entrance_radius,
        size=(0.12, 1.18, 2.22),
        material=model.material("deep_green_door", rgba=(0.02, 0.16, 0.12, 1.0)),
        embed=0.018,
    )
    _radial_box(
        tower,
        name="door_lintel",
        theta=theta_front,
        z=2.73,
        radius=entrance_radius + 0.05,
        size=(0.22, 1.56, 0.24),
        material=pale_stone,
        embed=0.030,
    )
    for side, offset in (("a", -0.72), ("b", 0.72)):
        tangent_x = -math.sin(theta_front)
        tangent_y = math.cos(theta_front)
        tower.visual(
            Box((0.20, 0.18, 2.45)),
            origin=Origin(
                xyz=(
                    (entrance_radius + 0.10 - 0.030) * math.cos(theta_front) + offset * tangent_x,
                    (entrance_radius + 0.10 - 0.030) * math.sin(theta_front) + offset * tangent_y,
                    1.62,
                ),
                rpy=(0.0, 0.0, theta_front),
            ),
            material=pale_stone,
            name=f"door_jamb_{side}",
        )

    # Narrow service windows alternate around the spiral-stair tower.
    for i, (theta, z) in enumerate(
        (
            (math.pi / 2.0, 4.45),
            (0.0, 8.85),
            (math.pi, 11.0),
            (-math.pi / 2.0, 15.75),
            (math.pi / 3.0, 18.6),
        )
    ):
        _add_window(tower, theta, z, i, dark_glass, pale_stone)

    # Gallery deck, parapet ring, and guard rail.
    tower.visual(
        Cylinder(radius=2.38, length=0.36),
        origin=Origin(xyz=(0.0, 0.0, 20.42)),
        material=pale_stone,
        name="gallery_deck",
    )
    tower.visual(
        Cylinder(radius=1.46, length=0.78),
        origin=Origin(xyz=(0.0, 0.0, 20.88)),
        material=pale_stone,
        name="lantern_pedestal",
    )
    tower.visual(
        mesh_from_geometry(TorusGeometry(2.38, 0.08, radial_segments=16, tubular_segments=96), "gallery_lip"),
        origin=Origin(xyz=(0.0, 0.0, 20.63)),
        material=pale_stone,
        name="gallery_lip",
    )
    tower.visual(
        mesh_from_geometry(TorusGeometry(2.08, 0.035, radial_segments=12, tubular_segments=96), "lower_rail"),
        origin=Origin(xyz=(0.0, 0.0, 21.10)),
        material=black_iron,
        name="lower_rail",
    )
    tower.visual(
        mesh_from_geometry(TorusGeometry(2.08, 0.04, radial_segments=12, tubular_segments=96), "upper_rail"),
        origin=Origin(xyz=(0.0, 0.0, 21.63)),
        material=black_iron,
        name="upper_rail",
    )
    for i in range(16):
        theta = 2.0 * math.pi * i / 16.0
        tower.visual(
            Cylinder(radius=0.035, length=1.20),
            origin=Origin(xyz=(2.08 * math.cos(theta), 2.08 * math.sin(theta), 21.10)),
            material=black_iron,
            name=f"gallery_post_{i}",
        )

    # Lantern room: octagonal glass house with metal mullions and roof.
    tower.visual(
        Cylinder(radius=1.58, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 21.20)),
        material=black_iron,
        name="lantern_floor_ring",
    )
    tower.visual(
        Cylinder(radius=1.48, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 21.30)),
        material=pale_stone,
        name="bearing_pad",
    )
    tower.visual(
        mesh_from_geometry(TorusGeometry(1.58, 0.055, radial_segments=14, tubular_segments=96), "lantern_top_ring"),
        origin=Origin(xyz=(0.0, 0.0, 23.94)),
        material=black_iron,
        name="lantern_top_ring",
    )
    tower.visual(
        mesh_from_geometry(TorusGeometry(1.58, 0.050, radial_segments=14, tubular_segments=96), "lantern_mid_ring"),
        origin=Origin(xyz=(0.0, 0.0, 22.62)),
        material=black_iron,
        name="lantern_mid_ring",
    )
    for i in range(8):
        theta = 2.0 * math.pi * i / 8.0
        _radial_box(
            tower,
            name=f"lantern_glass_{i}",
            theta=theta,
            z=22.62,
            radius=1.50,
            size=(0.045, 1.04, 2.55),
            material=glass,
            embed=0.0,
        )
        vertex = theta + math.pi / 8.0
        tower.visual(
            Cylinder(radius=0.055, length=2.72),
            origin=Origin(xyz=(1.62 * math.cos(vertex), 1.62 * math.sin(vertex), 22.62)),
            material=black_iron,
            name=f"lantern_mullion_{i}",
        )

    tower.visual(
        mesh_from_geometry(ConeGeometry(1.88, 0.92, radial_segments=96, closed=True), "lantern_roof"),
        origin=Origin(xyz=(0.0, 0.0, 24.45)),
        material=red_paint,
        name="lantern_roof",
    )
    tower.visual(
        mesh_from_geometry(DomeGeometry(0.58, radial_segments=72, height_segments=10, closed=True), "vent_dome"),
        origin=Origin(xyz=(0.0, 0.0, 24.95)),
        material=black_iron,
        name="vent_dome",
    )
    tower.visual(
        Cylinder(radius=0.20, length=0.90),
        origin=Origin(xyz=(0.0, 0.0, 25.30)),
        material=black_iron,
        name="vent_stack",
    )
    tower.visual(
        Sphere(radius=0.18),
        origin=Origin(xyz=(0.0, 0.0, 25.82)),
        material=brass,
        name="roof_finial",
    )

    beacon = model.part("beacon")
    beacon.visual(
        Cylinder(radius=0.42, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=brass,
        name="thrust_plate",
    )
    beacon.visual(
        Cylinder(radius=0.11, length=1.72),
        origin=Origin(xyz=(0.0, 0.0, 0.86)),
        material=brass,
        name="spindle",
    )
    beacon.visual(
        Sphere(radius=0.28),
        origin=Origin(xyz=(0.0, 0.0, 1.34)),
        material=warm_light,
        name="lamp",
    )
    beacon.visual(
        Box((2.38, 0.18, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 1.34)),
        material=brass,
        name="lens_bar",
    )
    beacon.visual(
        Box((0.12, 0.52, 0.72)),
        origin=Origin(xyz=(1.23, 0.0, 1.34)),
        material=glass,
        name="lens_0",
    )
    beacon.visual(
        Box((0.12, 0.52, 0.72)),
        origin=Origin(xyz=(-1.23, 0.0, 1.34)),
        material=glass,
        name="lens_1",
    )
    beacon.visual(
        Box((0.78, 0.08, 0.46)),
        origin=Origin(xyz=(0.47, 0.0, 1.34)),
        material=black_iron,
        name="reflector_0",
    )
    beacon.visual(
        Box((0.78, 0.08, 0.46)),
        origin=Origin(xyz=(-0.47, 0.0, 1.34)),
        material=black_iron,
        name="reflector_1",
    )

    model.articulation(
        "beacon_spin",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=beacon,
        origin=Origin(xyz=(0.0, 0.0, 21.35)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.45),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    beacon = object_model.get_part("beacon")
    spin = object_model.get_articulation("beacon_spin")

    ctx.check(
        "beacon has continuous vertical rotation",
        spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.expect_contact(
        beacon,
        tower,
        elem_a="thrust_plate",
        elem_b="bearing_pad",
        contact_tol=0.001,
        name="rotating assembly sits on bearing pad",
    )
    ctx.expect_within(
        beacon,
        tower,
        axes="xy",
        inner_elem="lens_bar",
        outer_elem="lantern_floor_ring",
        margin=0.02,
        name="beacon beam fits inside lantern room",
    )

    rest_aabb = ctx.part_world_aabb(beacon)
    with ctx.pose({spin: math.pi / 2.0}):
        ctx.expect_contact(
            beacon,
            tower,
            elem_a="thrust_plate",
            elem_b="bearing_pad",
            contact_tol=0.001,
            name="beacon remains seated while rotating",
        )
        ctx.expect_within(
            beacon,
            tower,
            axes="xy",
            inner_elem="lens_bar",
            outer_elem="lantern_floor_ring",
            margin=0.02,
            name="rotated beacon clears lantern glazing",
        )
        turned_aabb = ctx.part_world_aabb(beacon)

    if rest_aabb is not None and turned_aabb is not None:
        rest_dx = rest_aabb[1][0] - rest_aabb[0][0]
        rest_dy = rest_aabb[1][1] - rest_aabb[0][1]
        turned_dx = turned_aabb[1][0] - turned_aabb[0][0]
        turned_dy = turned_aabb[1][1] - turned_aabb[0][1]
        ctx.check(
            "beacon sweep visibly changes bearing",
            rest_dx > rest_dy * 2.0 and turned_dy > turned_dx * 2.0,
            details=f"rest=({rest_dx:.3f},{rest_dy:.3f}) turned=({turned_dx:.3f},{turned_dy:.3f})",
        )
    else:
        ctx.fail("beacon sweep visibly changes bearing", "missing beacon AABB in test pose")

    return ctx.report()


object_model = build_object_model()
