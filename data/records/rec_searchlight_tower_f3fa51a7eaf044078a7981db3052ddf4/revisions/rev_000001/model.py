from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    DomeGeometry,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    TrunnionYokeGeometry,
    mesh_from_geometry,
)


def _axis_rpy_from_z(p0: tuple[float, float, float], p1: tuple[float, float, float]) -> tuple[float, float, float]:
    """Return URDF rpy that points a local +Z cylinder from p0 toward p1."""
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 1e-9:
        return (0.0, 0.0, 0.0)
    ux, uy, uz = dx / length, dy / length, dz / length
    pitch = math.acos(max(-1.0, min(1.0, uz)))
    yaw = math.atan2(uy, ux) if abs(math.sin(pitch)) > 1e-9 else 0.0
    return (0.0, pitch, yaw)


def _cylinder_between(part, p0, p1, radius, material, name):
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5, (p0[2] + p1[2]) * 0.5),
            rpy=_axis_rpy_from_z(p0, p1),
        ),
        material=material,
        name=name,
    )


def _lamp_housing_mesh():
    # A thick, flared, hollow searchlight can.  The lamp axis is local +Z here;
    # visuals rotate it so the searchlight points along local +Y.
    return LatheGeometry.from_shell_profiles(
        outer_profile=(
            (0.255, -0.350),
            (0.310, -0.210),
            (0.350, 0.080),
            (0.385, 0.380),
        ),
        inner_profile=(
            (0.205, -0.330),
            (0.260, -0.195),
            (0.300, 0.080),
            (0.330, 0.360),
        ),
        segments=64,
        lip_samples=8,
    )


def _reflector_mesh():
    # Thin parabolic reflector bowl nested inside the open housing.
    return LatheGeometry.from_shell_profiles(
        outer_profile=(
            (0.045, -0.275),
            (0.090, -0.240),
            (0.170, -0.135),
            (0.255, 0.055),
            (0.310, 0.235),
        ),
        inner_profile=(
            (0.030, -0.255),
            (0.075, -0.220),
            (0.150, -0.115),
            (0.235, 0.060),
            (0.292, 0.225),
        ),
        segments=64,
        lip_samples=6,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_searchlight_tower")

    concrete = Material("weathered_concrete", rgba=(0.50, 0.50, 0.46, 1.0))
    galvanized = Material("dark_galvanized_steel", rgba=(0.23, 0.25, 0.25, 1.0))
    black = Material("matte_black_enamel", rgba=(0.02, 0.025, 0.025, 1.0))
    yellow = Material("safety_yellow", rgba=(0.95, 0.68, 0.05, 1.0))
    reflector = Material("polished_reflector", rgba=(0.92, 0.88, 0.72, 1.0))
    glass = Material("blue_tinted_glass", rgba=(0.55, 0.78, 1.0, 0.42))
    cable = Material("black_rubber_cable", rgba=(0.005, 0.005, 0.004, 1.0))
    bolt = Material("dark_bolt_heads", rgba=(0.08, 0.075, 0.065, 1.0))
    lamp_glow = Material("warm_arc_tube", rgba=(1.0, 0.78, 0.28, 0.75))

    tower = model.part("tower")
    tower.visual(Box((1.55, 1.55, 0.18)), origin=Origin(xyz=(0.0, 0.0, 0.09)), material=concrete, name="foundation_pad")
    tower.visual(Box((0.70, 0.70, 0.045)), origin=Origin(xyz=(0.0, 0.0, 0.2025)), material=galvanized, name="base_plate")

    # Mast and four battered corner legs give the installation real outdoor scale.
    tower.visual(Cylinder(radius=0.070, length=3.00), origin=Origin(xyz=(0.0, 0.0, 1.68)), material=galvanized, name="central_mast")
    for i, (x, y) in enumerate(((-0.32, -0.32), (0.32, -0.32), (0.32, 0.32), (-0.32, 0.32))):
        _cylinder_between(tower, (x, y, 0.22), (x * 0.55, y * 0.55, 3.08), 0.026, galvanized, f"tower_leg_{i}")

    # Stiff X-bracing on all four faces.
    z_levels = (0.38, 1.18, 1.98, 2.78)
    for level in range(3):
        z0, z1 = z_levels[level], z_levels[level + 1]
        s0 = 1.0 - 0.45 * (z0 - 0.22) / (3.08 - 0.22)
        s1 = 1.0 - 0.45 * (z1 - 0.22) / (3.08 - 0.22)
        low = 0.32 * s0
        high = 0.32 * s1
        # Front and rear faces.
        _cylinder_between(tower, (-low, -low, z0), (high, -high, z1), 0.013, galvanized, f"front_brace_{level}_0")
        _cylinder_between(tower, (low, -low, z0), (-high, -high, z1), 0.013, galvanized, f"front_brace_{level}_1")
        _cylinder_between(tower, (-low, low, z0), (high, high, z1), 0.013, galvanized, f"rear_brace_{level}_0")
        _cylinder_between(tower, (low, low, z0), (-high, high, z1), 0.013, galvanized, f"rear_brace_{level}_1")
        # Side faces.
        _cylinder_between(tower, (-low, -low, z0), (-high, high, z1), 0.013, galvanized, f"side_brace_{level}_0")
        _cylinder_between(tower, (-low, low, z0), (-high, -high, z1), 0.013, galvanized, f"side_brace_{level}_1")
        _cylinder_between(tower, (low, -low, z0), (high, high, z1), 0.013, galvanized, f"side_brace_{level}_2")
        _cylinder_between(tower, (low, low, z0), (high, -high, z1), 0.013, galvanized, f"side_brace_{level}_3")

    deck_z = 3.16
    tower.visual(Box((1.46, 1.46, 0.055)), origin=Origin(xyz=(0.0, 0.0, deck_z)), material=galvanized, name="deck_plate")
    # Guard rail posts and rails, connected to the deck.
    for i, (x, y) in enumerate(((-0.68, -0.68), (0.68, -0.68), (0.68, 0.68), (-0.68, 0.68))):
        tower.visual(Cylinder(radius=0.018, length=0.56), origin=Origin(xyz=(x, y, deck_z + 0.28)), material=yellow, name=f"rail_post_{i}")
    _cylinder_between(tower, (-0.68, -0.68, deck_z + 0.56), (0.68, -0.68, deck_z + 0.56), 0.016, yellow, "front_guardrail")
    _cylinder_between(tower, (-0.68, 0.68, deck_z + 0.56), (0.68, 0.68, deck_z + 0.56), 0.016, yellow, "rear_guardrail")
    _cylinder_between(tower, (-0.68, -0.68, deck_z + 0.56), (-0.68, 0.68, deck_z + 0.56), 0.016, yellow, "side_guardrail_0")
    _cylinder_between(tower, (0.68, -0.68, deck_z + 0.56), (0.68, 0.68, deck_z + 0.56), 0.016, yellow, "side_guardrail_1")

    # Electrical utility details attached to the mast.
    tower.visual(Box((0.18, 0.11, 0.26)), origin=Origin(xyz=(-0.145, -0.035, 1.00)), material=galvanized, name="junction_box")
    tower.visual(Box((0.11, 0.035, 2.20)), origin=Origin(xyz=(-0.095, -0.035, 1.95)), material=cable, name="vertical_conduit")
    _cylinder_between(tower, (-0.095, -0.035, 2.98), (-0.36, -0.22, 3.13), 0.018, cable, "service_cable")
    _cylinder_between(tower, (-0.055, -0.020, 1.00), (-0.130, -0.030, 1.00), 0.012, galvanized, "junction_bracket")

    # Bolt pattern on base plate and deck plate.
    for i, (x, y) in enumerate(((-0.27, -0.27), (0.27, -0.27), (0.27, 0.27), (-0.27, 0.27))):
        tower.visual(Cylinder(radius=0.030, length=0.018), origin=Origin(xyz=(x, y, 0.234)), material=bolt, name=f"anchor_bolt_{i}")
    for i, (x, y) in enumerate(((-0.24, 0.24), (0.24, 0.24), (0.24, -0.24), (-0.24, -0.24))):
        tower.visual(Cylinder(radius=0.022, length=0.014), origin=Origin(xyz=(x, y, deck_z + 0.0345)), material=bolt, name=f"turntable_bolt_{i}")

    azimuth = model.part("azimuth_carriage")
    azimuth.visual(Cylinder(radius=0.30, length=0.080), origin=Origin(xyz=(0.0, 0.0, 0.040)), material=black, name="bearing_disk")
    azimuth.visual(Cylinder(radius=0.23, length=0.045), origin=Origin(xyz=(0.0, 0.0, 0.1025)), material=galvanized, name="slew_ring")
    azimuth.visual(Box((0.62, 0.42, 0.070)), origin=Origin(xyz=(0.0, 0.0, 0.155)), material=black, name="saddle_plate")
    yoke_mesh = mesh_from_geometry(
        TrunnionYokeGeometry(
            (1.08, 0.34, 0.72),
            span_width=0.84,
            trunnion_diameter=0.130,
            trunnion_center_z=0.520,
            base_thickness=0.105,
            corner_radius=0.025,
            center=False,
        ),
        "trunnion_yoke",
    )
    azimuth.visual(yoke_mesh, origin=Origin(xyz=(0.0, 0.0, 0.170)), material=black, name="yoke")
    # Side gearbox and wheel support boss, tied into the yoke cheek.
    azimuth.visual(Box((0.16, 0.14, 0.20)), origin=Origin(xyz=(0.60, -0.235, 0.690)), material=galvanized, name="elevation_gearbox")
    azimuth.visual(Box((0.080, 0.120, 0.080)), origin=Origin(xyz=(0.69, -0.110, 0.690)), material=galvanized, name="gearbox_neck")
    azimuth.visual(Cylinder(radius=0.055, length=0.14), origin=Origin(xyz=(0.69, 0.0, 0.690), rpy=(0.0, math.pi / 2.0, 0.0)), material=galvanized, name="wheel_boss")

    yaw_joint = model.articulation(
        "tower_to_azimuth",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=azimuth,
        origin=Origin(xyz=(0.0, 0.0, deck_z + 0.0275)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=320.0, velocity=0.45, lower=-2.8, upper=2.8),
    )

    lamp = model.part("lamp_head")
    lamp.visual(mesh_from_geometry(_lamp_housing_mesh(), "lamp_housing"), origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)), material=black, name="housing_shell")
    lamp.visual(mesh_from_geometry(_reflector_mesh(), "reflector_bowl"), origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)), material=reflector, name="reflector_bowl")
    lamp.visual(mesh_from_geometry(TorusGeometry(radius=0.365, tube=0.026, radial_segments=24, tubular_segments=72), "front_bezel"), origin=Origin(xyz=(0.0, 0.375, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=galvanized, name="front_bezel")
    lamp.visual(Cylinder(radius=0.338, length=0.022), origin=Origin(xyz=(0.0, 0.348, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=glass, name="front_lens")
    lamp.visual(Cylinder(radius=0.238, length=0.075), origin=Origin(xyz=(0.0, -0.374, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=black, name="rear_cap")
    lamp.visual(mesh_from_geometry(DomeGeometry(0.245, radial_segments=48, height_segments=12), "rear_dome"), origin=Origin(xyz=(0.0, -0.410, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)), material=black, name="rear_dome")
    lamp.visual(Cylinder(radius=0.055, length=1.08), origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)), material=galvanized, name="trunnion_axle")
    lamp.visual(Cylinder(radius=0.078, length=0.080), origin=Origin(xyz=(-0.580, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=galvanized, name="trunnion_collar_0")
    lamp.visual(Cylinder(radius=0.078, length=0.080), origin=Origin(xyz=(0.580, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=galvanized, name="trunnion_collar_1")
    # Bulb and support stem are visibly inside the reflector, not floating.
    lamp.visual(Cylinder(radius=0.018, length=0.28), origin=Origin(xyz=(0.0, -0.105, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=galvanized, name="bulb_stem")
    lamp.visual(Sphere(radius=0.070), origin=Origin(xyz=(0.0, 0.035, 0.0)), material=lamp_glow, name="arc_bulb")
    lamp.visual(Box((0.12, 0.065, 0.055)), origin=Origin(xyz=(0.0, -0.395, 0.235)), material=cable, name="cable_gland")
    _cylinder_between(lamp, (-0.18, -0.04, 0.350), (0.18, -0.04, 0.350), 0.020, galvanized, "top_lift_handle")
    _cylinder_between(lamp, (-0.18, -0.04, 0.350), (-0.22, -0.10, 0.230), 0.016, galvanized, "handle_stanchion_0")
    _cylinder_between(lamp, (0.18, -0.04, 0.350), (0.22, -0.10, 0.230), 0.016, galvanized, "handle_stanchion_1")

    elev_joint = model.articulation(
        "azimuth_to_lamp",
        ArticulationType.REVOLUTE,
        parent=azimuth,
        child=lamp,
        origin=Origin(xyz=(0.0, 0.0, 0.690)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.55, lower=-0.25, upper=0.65),
    )

    wheel = model.part("elevation_wheel")
    wheel.visual(mesh_from_geometry(TorusGeometry(radius=0.135, tube=0.012, radial_segments=18, tubular_segments=56), "handwheel_ring"), origin=Origin(xyz=(0.080, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=black, name="handwheel_ring")
    wheel.visual(Cylinder(radius=0.035, length=0.100), origin=Origin(xyz=(0.040, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=galvanized, name="handwheel_hub")
    wheel.visual(Box((0.012, 0.250, 0.018)), origin=Origin(xyz=(0.080, 0.0, 0.0)), material=galvanized, name="handwheel_spoke_0")
    wheel.visual(Box((0.012, 0.018, 0.250)), origin=Origin(xyz=(0.080, 0.0, 0.0)), material=galvanized, name="handwheel_spoke_1")
    wheel.visual(Cylinder(radius=0.020, length=0.060), origin=Origin(xyz=(0.080, 0.0, 0.135), rpy=(0.0, math.pi / 2.0, 0.0)), material=yellow, name="spinner_grip")

    model.articulation(
        "azimuth_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=azimuth,
        child=wheel,
        origin=Origin(xyz=(0.770, 0.0, 0.690)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=6.0),
    )

    # Stash names to keep intent clear for tests without changing the authored geometry.
    model.meta["primary_motion"] = (yaw_joint.name, elev_joint.name)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    azimuth = object_model.get_part("azimuth_carriage")
    lamp = object_model.get_part("lamp_head")
    wheel = object_model.get_part("elevation_wheel")
    yaw = object_model.get_articulation("tower_to_azimuth")
    elev = object_model.get_articulation("azimuth_to_lamp")

    ctx.expect_contact(
        azimuth,
        tower,
        elem_a="bearing_disk",
        elem_b="deck_plate",
        contact_tol=0.002,
        name="turntable disk sits on tower deck",
    )
    ctx.expect_within(
        lamp,
        azimuth,
        axes="x",
        inner_elem="trunnion_axle",
        outer_elem="yoke",
        margin=0.010,
        name="trunnion axle is captured by the yoke width",
    )
    ctx.expect_gap(
        lamp,
        azimuth,
        axis="z",
        positive_elem="housing_shell",
        negative_elem="saddle_plate",
        min_gap=0.015,
        name="lamp housing clears the saddle plate at level aim",
    )
    ctx.expect_contact(
        wheel,
        azimuth,
        elem_a="handwheel_hub",
        elem_b="wheel_boss",
        contact_tol=0.002,
        name="handwheel hub mounts on gearbox boss",
    )

    with ctx.pose({elev: 0.0}):
        rest_front = ctx.part_element_world_aabb(lamp, elem="front_bezel")
    with ctx.pose({elev: 0.60}):
        up_front = ctx.part_element_world_aabb(lamp, elem="front_bezel")
    with ctx.pose({elev: -0.20}):
        down_front = ctx.part_element_world_aabb(lamp, elem="front_bezel")

    def _aabb_center_z(aabb):
        return None if aabb is None else (aabb[0][2] + aabb[1][2]) * 0.5

    ctx.check(
        "elevation raises and lowers the lamp face",
        rest_front is not None
        and up_front is not None
        and down_front is not None
        and _aabb_center_z(up_front) > _aabb_center_z(rest_front) + 0.12
        and _aabb_center_z(down_front) < _aabb_center_z(rest_front) - 0.05,
        details=f"rest={rest_front}, up={up_front}, down={down_front}",
    )

    with ctx.pose({yaw: 0.0}):
        rest_yaw_front = ctx.part_element_world_aabb(lamp, elem="front_bezel")
    with ctx.pose({yaw: 1.10}):
        yawed_front = ctx.part_element_world_aabb(lamp, elem="front_bezel")

    def _aabb_center_x(aabb):
        return None if aabb is None else (aabb[0][0] + aabb[1][0]) * 0.5

    ctx.check(
        "azimuth rotates the lamp around the mast",
        rest_yaw_front is not None
        and yawed_front is not None
        and abs(_aabb_center_x(yawed_front) - _aabb_center_x(rest_yaw_front)) > 0.25,
        details=f"rest={rest_yaw_front}, yawed={yawed_front}",
    )

    return ctx.report()


object_model = build_object_model()
