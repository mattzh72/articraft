from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _rotate_xy(x: float, y: float, angle: float) -> tuple[float, float]:
    ca = math.cos(angle)
    sa = math.sin(angle)
    return (x * ca - y * sa, x * sa + y * ca)


def _palm_blade_geometry(angle: float) -> MeshGeometry:
    """Broad cambered palm-leaf blade, rooted near the motor and pointing radial."""

    geom = MeshGeometry()
    stations = [
        # radius, half-width, center z, center crown height
        (0.330, 0.045, -0.003, 0.006),
        (0.430, 0.105, -0.006, 0.010),
        (0.620, 0.160, -0.013, 0.014),
        (0.850, 0.145, -0.026, 0.012),
        (1.060, 0.035, -0.043, 0.004),
    ]
    cross = [-1.0, -0.55, 0.0, 0.55, 1.0]
    thickness = 0.010

    top: list[list[int]] = []
    bottom: list[list[int]] = []
    for x, half_width, zc, crown in stations:
        top_row: list[int] = []
        bottom_row: list[int] = []
        for frac in cross:
            y = half_width * frac
            # Slight pitch across the leaf and a crowned central rib.
            z_top = zc + crown * (1.0 - abs(frac)) + 0.006 * frac
            xr, yr = _rotate_xy(x, y, angle)
            top_row.append(geom.add_vertex(xr, yr, z_top))
            bottom_row.append(geom.add_vertex(xr, yr, z_top - thickness))
        top.append(top_row)
        bottom.append(bottom_row)

    for i in range(len(stations) - 1):
        for j in range(len(cross) - 1):
            geom.add_face(top[i][j], top[i + 1][j], top[i + 1][j + 1])
            geom.add_face(top[i][j], top[i + 1][j + 1], top[i][j + 1])
            geom.add_face(bottom[i][j + 1], bottom[i + 1][j + 1], bottom[i + 1][j])
            geom.add_face(bottom[i][j + 1], bottom[i + 1][j], bottom[i][j])

    # Long side walls.
    for j in (0, len(cross) - 1):
        for i in range(len(stations) - 1):
            geom.add_face(top[i][j], bottom[i][j], bottom[i + 1][j])
            geom.add_face(top[i][j], bottom[i + 1][j], top[i + 1][j])

    # Root and tip caps.
    for i in (0, len(stations) - 1):
        for j in range(len(cross) - 1):
            geom.add_face(top[i][j], top[i][j + 1], bottom[i][j + 1])
            geom.add_face(top[i][j], bottom[i][j + 1], bottom[i][j])

    return geom


def _blade_spine_geometry(angle: float) -> MeshGeometry:
    """Raised center vein following the droop of the palm blade."""

    geom = MeshGeometry()
    stations = [
        (0.345, -0.000),
        (0.500, -0.002),
        (0.700, -0.010),
        (0.920, -0.024),
        (1.035, -0.037),
    ]
    half_width = 0.010
    thickness = 0.008
    top_left: list[int] = []
    top_right: list[int] = []
    bot_left: list[int] = []
    bot_right: list[int] = []
    for x, z in stations:
        for_side = []
        for y in (-half_width, half_width):
            xr, yr = _rotate_xy(x, y, angle)
            for_side.append((xr, yr))
        top_left.append(geom.add_vertex(for_side[0][0], for_side[0][1], z + thickness))
        top_right.append(geom.add_vertex(for_side[1][0], for_side[1][1], z + thickness))
        bot_left.append(geom.add_vertex(for_side[0][0], for_side[0][1], z))
        bot_right.append(geom.add_vertex(for_side[1][0], for_side[1][1], z))

    for i in range(len(stations) - 1):
        geom.add_face(top_left[i], top_left[i + 1], top_right[i + 1])
        geom.add_face(top_left[i], top_right[i + 1], top_right[i])
        geom.add_face(bot_right[i], bot_right[i + 1], bot_left[i + 1])
        geom.add_face(bot_right[i], bot_left[i + 1], bot_left[i])
        geom.add_face(top_left[i], bot_left[i], bot_left[i + 1])
        geom.add_face(top_left[i], bot_left[i + 1], top_left[i + 1])
        geom.add_face(top_right[i], top_right[i + 1], bot_right[i + 1])
        geom.add_face(top_right[i], bot_right[i + 1], bot_right[i])
    return geom


def _globe_rib_geometry() -> MeshGeometry:
    """Horizontal raised ribs around the frosted bottom glass globe."""

    geom = MeshGeometry()
    # Approximate the globe radius at each z, then merge torus bands there.
    bands = [
        (-0.137, 0.067),
        (-0.158, 0.086),
        (-0.181, 0.101),
        (-0.205, 0.104),
        (-0.229, 0.093),
        (-0.252, 0.071),
        (-0.272, 0.043),
    ]
    for z, radius in bands:
        geom.merge(TorusGeometry(radius=radius, tube=0.0035, radial_segments=10, tubular_segments=48).translate(0.0, 0.0, z))
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_blade_tropical_ceiling_fan")

    wicker = model.material("woven_wicker", rgba=(0.74, 0.55, 0.30, 1.0))
    dark_wicker = model.material("dark_wicker_strands", rgba=(0.42, 0.26, 0.12, 1.0))
    bronze = model.material("aged_bronze", rgba=(0.23, 0.18, 0.12, 1.0))
    palm = model.material("palm_leaf_green", rgba=(0.24, 0.43, 0.18, 1.0))
    palm_rib = model.material("dry_palm_ribs", rgba=(0.82, 0.71, 0.42, 1.0))
    glass = model.material("frosted_ribbed_glass", rgba=(0.86, 0.95, 0.92, 0.48))

    mount = model.part("mount")
    # Ceiling hardware and short downrod.
    mount.visual(Cylinder(radius=0.135, length=0.055), origin=Origin(xyz=(0.0, 0.0, 0.420)), material=bronze, name="ceiling_canopy")
    mount.visual(Cylinder(radius=0.024, length=0.310), origin=Origin(xyz=(0.0, 0.0, 0.255)), material=bronze, name="short_downrod")
    mount.visual(Cylinder(radius=0.042, length=0.035), origin=Origin(xyz=(0.0, 0.0, 0.092)), material=bronze, name="downrod_collar")

    # Bulged wicker/rattan motor housing.
    motor_profile = [
        (0.000, 0.098),
        (0.055, 0.098),
        (0.135, 0.080),
        (0.165, 0.035),
        (0.165, -0.040),
        (0.128, -0.086),
        (0.045, -0.100),
        (0.000, -0.100),
    ]
    mount.visual(mesh_from_geometry(LatheGeometry(motor_profile, segments=72), "wicker_motor_housing"), material=wicker, name="wicker_motor_housing")

    # Wicker weave: horizontal hoops plus vertical strands embedded in the shell.
    for idx, z in enumerate((-0.068, -0.040, -0.012, 0.016, 0.044, 0.070)):
        mount.visual(
            mesh_from_geometry(TorusGeometry(radius=0.166, tube=0.0038, radial_segments=8, tubular_segments=72), f"wicker_hoop_{idx}"),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=dark_wicker,
            name=f"wicker_hoop_{idx}",
        )
    for idx in range(24):
        angle = idx * math.tau / 24.0
        x, y = _rotate_xy(0.168, 0.0, angle)
        mount.visual(
            Cylinder(radius=0.0038, length=0.145),
            origin=Origin(xyz=(x, y, -0.004)),
            material=dark_wicker,
            name=f"wicker_strand_{idx}",
        )
    mount.visual(
        mesh_from_geometry(TorusGeometry(radius=0.175, tube=0.009, radial_segments=12, tubular_segments=96), "bearing_race"),
        material=bronze,
        name="bearing_race",
    )

    # Bottom light kit: retained collar and ribbed frosted glass globe.
    mount.visual(Cylinder(radius=0.076, length=0.040), origin=Origin(xyz=(0.0, 0.0, -0.112)), material=bronze, name="globe_collar")
    globe_profile = [
        (0.000, -0.110),
        (0.035, -0.112),
        (0.074, -0.128),
        (0.083, -0.138),
        (0.079, -0.146),
        (0.094, -0.160),
        (0.088, -0.168),
        (0.104, -0.187),
        (0.096, -0.196),
        (0.108, -0.215),
        (0.097, -0.226),
        (0.092, -0.242),
        (0.079, -0.253),
        (0.066, -0.265),
        (0.036, -0.292),
        (0.000, -0.297),
    ]
    mount.visual(mesh_from_geometry(LatheGeometry(globe_profile, segments=80), "bottom_globe"), material=glass, name="bottom_globe")

    rotor = model.part("rotor")
    # A narrow rotating bronze ring clears the fixed wicker motor shell.
    rotor.visual(mesh_from_geometry(TorusGeometry(radius=0.195, tube=0.012, radial_segments=12, tubular_segments=96), "rotor_ring"), material=bronze, name="rotor_ring")

    for blade_idx in range(3):
        angle = blade_idx * math.tau / 3.0
        cx, cy = _rotate_xy(0.302, 0.0, angle)
        rotor.visual(
            Box((0.200, 0.036, 0.014)),
            origin=Origin(xyz=(cx, cy, -0.001), rpy=(0.0, 0.0, angle)),
            material=bronze,
            name=f"blade_arm_{blade_idx}",
        )
        rotor.visual(
            mesh_from_geometry(_palm_blade_geometry(angle), f"palm_blade_{blade_idx}"),
            material=palm,
            name=f"palm_blade_{blade_idx}",
        )
        rotor.visual(
            mesh_from_geometry(_blade_spine_geometry(angle), f"blade_spine_{blade_idx}"),
            material=palm_rib,
            name=f"blade_spine_{blade_idx}",
        )
        # Two visible screw bosses clamp each palm blade to the bronze iron.
        for screw_idx, lateral in enumerate((-0.028, 0.028)):
            sx, sy = _rotate_xy(0.382, lateral, angle)
            rotor.visual(
                Cylinder(radius=0.014, length=0.010),
                origin=Origin(xyz=(sx, sy, 0.005)),
                material=bronze,
                name=f"blade_screw_{blade_idx}_{screw_idx}",
            )

    model.articulation(
        "fan_spin",
        ArticulationType.CONTINUOUS,
        parent=mount,
        child=rotor,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=18.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rotor = object_model.get_part("rotor")
    mount = object_model.get_part("mount")
    spin = object_model.get_articulation("fan_spin")

    blade_visuals = [v for v in rotor.visuals if v.name and v.name.startswith("palm_blade_")]
    ctx.allow_overlap(
        rotor,
        mount,
        elem_a="rotor_ring",
        elem_b="bearing_race",
        reason="The rotating bronze ring is intentionally captured on the fixed bearing race at the central fan joint.",
    )
    ctx.check("three broad palm blades", len(blade_visuals) == 3, details=f"found={len(blade_visuals)}")
    ctx.check("central continuous spin joint", spin.articulation_type == ArticulationType.CONTINUOUS, details=str(spin.articulation_type))
    ctx.expect_overlap(rotor, mount, axes="xy", elem_a="rotor_ring", elem_b="wicker_motor_housing", min_overlap=0.25, name="rotor ring surrounds motor housing footprint")
    ctx.expect_overlap(rotor, mount, axes="xy", elem_a="rotor_ring", elem_b="bearing_race", min_overlap=0.34, name="bearing race is captured by rotor ring")
    ctx.expect_gap(rotor, mount, axis="z", positive_elem="rotor_ring", negative_elem="bottom_globe", min_gap=0.08, name="rotor clears bottom glass globe")

    before = ctx.part_element_world_aabb(rotor, elem="palm_blade_0")
    with ctx.pose({spin: 0.45}):
        after = ctx.part_element_world_aabb(rotor, elem="palm_blade_0")
    moved = False
    if before is not None and after is not None:
        before_center_y = (before[0][1] + before[1][1]) * 0.5
        after_center_y = (after[0][1] + after[1][1]) * 0.5
        moved = abs(after_center_y - before_center_y) > 0.10
    ctx.check("blade rotates about vertical joint", moved, details=f"before={before}, after={after}")

    return ctx.report()


object_model = build_object_model()
