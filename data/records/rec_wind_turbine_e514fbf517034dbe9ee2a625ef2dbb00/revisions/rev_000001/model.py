from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _circle_loop(
    radius: float,
    z: float,
    *,
    center_x: float = 0.0,
    center_y: float = 0.0,
    segments: int = 24,
) -> list[tuple[float, float, float]]:
    return [
        (
            center_x + radius * math.cos((2.0 * math.pi * i) / segments),
            center_y + radius * math.sin((2.0 * math.pi * i) / segments),
            z,
        )
        for i in range(segments)
    ]


def _yz_rounded_section(
    x: float,
    width: float,
    height: float,
    corner_radius: float,
) -> list[tuple[float, float, float]]:
    return [(x, y, z) for y, z in rounded_rect_profile(width, height, corner_radius)]


def _build_tower_mesh():
    return LatheGeometry(
        [
            (0.0, 0.0),
            (1.48, 0.0),
            (0.58, 0.16),
            (0.50, 0.70),
            (0.42, 3.0),
            (0.33, 7.0),
            (0.25, 11.2),
            (0.23, 12.2),
            (0.0, 12.2),
        ],
        segments=72,
    )


def _build_nacelle_shell_mesh():
    return section_loft(
        [
            _yz_rounded_section(-0.85, 0.95, 0.95, 0.18),
            _yz_rounded_section(-0.15, 1.10, 1.22, 0.22),
            _yz_rounded_section(0.95, 1.20, 1.36, 0.24),
            _yz_rounded_section(2.20, 0.96, 1.05, 0.20),
            _yz_rounded_section(2.95, 0.42, 0.42, 0.08),
        ]
    )


def _rotate_xy(x: float, y: float, angle: float) -> tuple[float, float]:
    c = math.cos(angle)
    s = math.sin(angle)
    return (c * x - s * y, s * x + c * y)


def _blade_airfoil_section(
    z: float,
    chord: float,
    thickness: float,
    pitch: float,
) -> list[tuple[float, float, float]]:
    base_loop = [
        (-0.52 * thickness, -0.48 * chord),
        (-0.14 * thickness, -0.30 * chord),
        (0.12 * thickness, -0.08 * chord),
        (0.48 * thickness, 0.20 * chord),
        (0.22 * thickness, 0.50 * chord),
        (-0.08 * thickness, 0.48 * chord),
        (-0.46 * thickness, 0.16 * chord),
        (-0.36 * thickness, -0.22 * chord),
    ]
    return [(*_rotate_xy(x, y, pitch), z) for x, y in base_loop]


def _build_spinner_mesh():
    return LatheGeometry(
        [
            (0.0, -0.48),
            (0.10, -0.42),
            (0.22, -0.28),
            (0.34, -0.02),
            (0.32, 0.18),
            (0.16, 0.30),
            (0.0, 0.34),
        ],
        segments=64,
    ).rotate_y(math.pi / 2.0)


def _build_blade_mesh():
    return section_loft(
        [
            _blade_airfoil_section(0.36, 0.64, 0.20, math.radians(18.0)),
            _blade_airfoil_section(1.00, 0.52, 0.16, math.radians(11.0)),
            _blade_airfoil_section(1.72, 0.36, 0.11, math.radians(6.0)),
            _blade_airfoil_section(2.38, 0.22, 0.07, math.radians(2.0)),
            _blade_airfoil_section(2.82, 0.12, 0.03, math.radians(0.0)),
        ]
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="horizontal_axis_wind_turbine")

    concrete = model.material("concrete", rgba=(0.70, 0.71, 0.72, 1.0))
    tower_white = model.material("tower_white", rgba=(0.92, 0.94, 0.96, 1.0))
    nacelle_white = model.material("nacelle_white", rgba=(0.95, 0.96, 0.98, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.24, 0.26, 0.30, 1.0))
    steel = model.material("steel", rgba=(0.62, 0.65, 0.69, 1.0))

    tower_mesh = _save_mesh("tower_shell", _build_tower_mesh())
    nacelle_mesh = _save_mesh("nacelle_shell", _build_nacelle_shell_mesh())
    spinner_mesh = _save_mesh("hub_spinner", _build_spinner_mesh())
    blade_mesh = _save_mesh("turbine_blade", _build_blade_mesh())

    tower = model.part("tower")
    tower.visual(
        Box((3.8, 3.8, 1.1)),
        origin=Origin(xyz=(0.0, 0.0, 0.55)),
        material=concrete,
        name="foundation_pad",
    )
    tower.visual(
        Box((2.2, 2.2, 0.28)),
        origin=Origin(xyz=(0.0, 0.0, 1.24)),
        material=concrete,
        name="pedestal_cap",
    )
    tower.visual(
        tower_mesh,
        origin=Origin(xyz=(0.0, 0.0, 1.38)),
        material=tower_white,
        name="tower_shell",
    )
    tower.visual(
        Cylinder(radius=0.42, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 13.50)),
        material=dark_metal,
        name="yaw_bearing_base",
    )
    tower.visual(
        Cylinder(radius=0.34, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 13.67)),
        material=steel,
        name="yaw_ring",
    )
    tower.inertial = Inertial.from_geometry(
        Box((3.8, 3.8, 13.8)),
        mass=1800.0,
        origin=Origin(xyz=(0.0, 0.0, 6.9)),
    )

    nacelle = model.part("nacelle")
    nacelle.visual(
        Cylinder(radius=0.42, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.21)),
        material=dark_metal,
        name="yaw_deck",
    )
    nacelle.visual(
        Cylinder(radius=0.22, length=0.88),
        origin=Origin(xyz=(0.0, 0.0, 0.59)),
        material=steel,
        name="support_column",
    )
    nacelle.visual(
        nacelle_mesh,
        origin=Origin(xyz=(0.55, 0.0, 1.10)),
        material=nacelle_white,
        name="nacelle_shell",
    )
    nacelle.visual(
        Cylinder(radius=0.34, length=1.10),
        origin=Origin(xyz=(2.95, 0.0, 1.10), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="main_bearing_housing",
    )
    nacelle.visual(
        Box((0.95, 0.74, 0.34)),
        origin=Origin(xyz=(0.10, 0.0, 1.14)),
        material=dark_metal,
        name="machinery_base",
    )
    nacelle.visual(
        Box((0.82, 0.56, 0.55)),
        origin=Origin(xyz=(0.28, 0.0, 1.55)),
        material=steel,
        name="generator_pack",
    )
    nacelle.visual(
        Cylinder(radius=0.15, length=0.50),
        origin=Origin(xyz=(-0.62, 0.0, 1.28), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="rear_cooling_unit",
    )
    nacelle.inertial = Inertial.from_geometry(
        Box((4.1, 1.8, 2.2)),
        mass=320.0,
        origin=Origin(xyz=(0.95, 0.0, 1.12)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.34, length=0.84),
        origin=Origin(xyz=(0.42, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="hub_barrel",
    )
    rotor.visual(
        Cylinder(radius=0.26, length=0.28),
        origin=Origin(xyz=(0.02, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="rear_hub_flange",
    )
    rotor.visual(
        spinner_mesh,
        origin=Origin(xyz=(1.06, 0.0, 0.0)),
        material=nacelle_white,
        name="spinner",
    )
    for blade_index in range(3):
        angle = blade_index * (2.0 * math.pi / 3.0)
        rotor.visual(
            Cylinder(radius=0.15, length=0.54),
            origin=Origin(xyz=(0.36, 0.0, 0.27), rpy=(angle, 0.0, 0.0)),
            material=steel,
            name=f"blade_root_{blade_index}",
        )
        rotor.visual(
            blade_mesh,
            origin=Origin(xyz=(0.36, 0.0, 0.18), rpy=(angle, 0.0, 0.0)),
            material=tower_white,
            name=f"blade_{blade_index}",
        )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.42, length=1.64),
        mass=95.0,
        origin=Origin(xyz=(0.62, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "tower_to_nacelle_yaw",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=nacelle,
        origin=Origin(xyz=(0.0, 0.0, 13.61)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8000.0, velocity=0.35),
    )
    model.articulation(
        "nacelle_to_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=rotor,
        origin=Origin(xyz=(3.62, 0.0, 1.10)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    tower = object_model.get_part("tower")
    nacelle = object_model.get_part("nacelle")
    rotor = object_model.get_part("rotor")
    yaw = object_model.get_articulation("tower_to_nacelle_yaw")
    spin = object_model.get_articulation("nacelle_to_rotor_spin")

    ctx.expect_gap(
        nacelle,
        tower,
        axis="z",
        positive_elem="yaw_deck",
        negative_elem="yaw_ring",
        max_gap=0.06,
        max_penetration=0.02,
        name="nacelle sits directly on top yaw ring",
    )
    ctx.expect_gap(
        rotor,
        nacelle,
        axis="x",
        positive_elem="rear_hub_flange",
        negative_elem="main_bearing_housing",
        min_gap=0.0,
        max_gap=0.35,
        name="hub clears nacelle front housing",
    )
    ctx.expect_overlap(
        rotor,
        nacelle,
        axes="yz",
        elem_a="rear_hub_flange",
        elem_b="main_bearing_housing",
        min_overlap=0.40,
        name="hub stays coaxial with main bearing",
    )
    with ctx.pose({yaw: math.pi / 3.0}):
        ctx.expect_origin_gap(
            nacelle,
            tower,
            axis="z",
            min_gap=13.55,
            max_gap=13.67,
            name="yaw keeps nacelle at tower top height",
        )
        ctx.expect_origin_distance(
            rotor,
            tower,
            axes="xy",
            min_dist=3.55,
            max_dist=3.70,
            name="yaw rotates rotor around tower centerline",
        )
    with ctx.pose({spin: math.pi / 2.0}):
        ctx.expect_gap(
            rotor,
            nacelle,
            axis="x",
            positive_elem="rear_hub_flange",
            negative_elem="main_bearing_housing",
            min_gap=0.0,
            max_gap=0.35,
            name="rotor spin keeps hub clear of nacelle housing",
        )

    rotor_aabb = ctx.part_world_aabb(rotor)
    rotor_diameter = None
    if rotor_aabb is not None:
        rotor_diameter = max(
            rotor_aabb[1][1] - rotor_aabb[0][1],
            rotor_aabb[1][2] - rotor_aabb[0][2],
        )
    ctx.check(
        "rotor reads as multi-meter turbine rotor",
        rotor_diameter is not None and rotor_diameter > 5.2,
        details=f"rotor_diameter={rotor_diameter}",
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
