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
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    section_loft,
)


def _save_mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, name)


def _merge_geometries(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _rotate_xy(x: float, y: float, angle: float) -> tuple[float, float]:
    c = math.cos(angle)
    s = math.sin(angle)
    return (x * c - y * s, x * s + y * c)


def _nacelle_section(
    x: float,
    width: float,
    height: float,
    *,
    roof_bias: float = 1.0,
    belly_bias: float = 1.0,
) -> list[tuple[float, float, float]]:
    half_w = width * 0.5
    half_h = height * 0.5
    return [
        (x, 0.00, 0.98 * half_h * roof_bias),
        (x, 0.36 * half_w, 0.88 * half_h * roof_bias),
        (x, 0.50 * half_w, 0.34 * half_h),
        (x, 0.46 * half_w, -0.30 * half_h * belly_bias),
        (x, 0.20 * half_w, -0.90 * half_h * belly_bias),
        (x, -0.20 * half_w, -0.90 * half_h * belly_bias),
        (x, -0.46 * half_w, -0.30 * half_h * belly_bias),
        (x, -0.50 * half_w, 0.34 * half_h),
        (x, -0.36 * half_w, 0.88 * half_h * roof_bias),
    ]


def _airfoil_loop(
    span_z: float,
    chord: float,
    thickness: float,
    sweep: float,
    twist_deg: float,
) -> list[tuple[float, float, float]]:
    twist = math.radians(twist_deg)
    points_2d = [
        (-0.52 * chord, 0.00 * thickness),
        (-0.26 * chord, 0.55 * thickness),
        (0.20 * chord, 0.62 * thickness),
        (0.48 * chord, 0.18 * thickness),
        (0.50 * chord, -0.08 * thickness),
        (0.26 * chord, -0.32 * thickness),
        (-0.10 * chord, -0.42 * thickness),
        (-0.40 * chord, -0.20 * thickness),
    ]
    section: list[tuple[float, float, float]] = []
    for x_val, y_val in points_2d:
        xr, yr = _rotate_xy(x_val, y_val, twist)
        section.append((xr + sweep, yr, span_z))
    return section


def _make_tower_mesh() -> MeshGeometry:
    return LatheGeometry(
        [
            (3.20, 1.80),
            (3.05, 8.00),
            (2.80, 20.00),
            (2.45, 38.00),
            (2.10, 58.00),
            (1.88, 76.40),
            (1.82, 79.60),
            (0.00, 79.60),
            (0.00, 1.80),
        ],
        segments=72,
    )


def _make_nacelle_shell_mesh() -> MeshGeometry:
    sections = [
        _nacelle_section(-2.4, 2.2, 2.0, roof_bias=0.95, belly_bias=0.88),
        _nacelle_section(-0.2, 3.4, 3.0, roof_bias=1.00, belly_bias=0.92),
        _nacelle_section(3.0, 4.6, 4.1, roof_bias=1.04, belly_bias=0.95),
        _nacelle_section(6.6, 4.3, 4.0, roof_bias=1.04, belly_bias=0.95),
        _nacelle_section(9.2, 3.1, 3.1, roof_bias=1.00, belly_bias=0.90),
        _nacelle_section(11.2, 1.9, 2.0, roof_bias=0.92, belly_bias=0.82),
    ]
    return repair_loft(section_loft(sections))


def _make_spinner_mesh() -> MeshGeometry:
    return LatheGeometry(
        [
            (0.00, -0.55),
            (0.28, -0.48),
            (0.76, -0.22),
            (1.06, 0.20),
            (1.18, 0.56),
            (1.04, 0.92),
            (0.74, 1.22),
            (0.24, 1.46),
            (0.00, 1.56),
        ],
        segments=72,
    ).rotate_y(math.pi / 2.0).translate(1.35, 0.0, 0.0)


def _make_hub_mesh() -> MeshGeometry:
    barrel = (
        CylinderGeometry(radius=1.46, height=2.70, radial_segments=48)
        .rotate_y(math.pi / 2.0)
        .translate(-0.10, 0.0, 0.0)
    )
    rear_collar = (
        CylinderGeometry(radius=1.12, height=1.10, radial_segments=40)
        .rotate_y(math.pi / 2.0)
        .translate(-1.25, 0.0, 0.0)
    )
    return _merge_geometries([barrel, rear_collar])


def _make_single_blade_mesh() -> MeshGeometry:
    blade = repair_loft(
        section_loft(
            [
                _airfoil_loop(0.80, 3.60, 0.74, -0.35, 21.0),
                _airfoil_loop(4.20, 3.15, 0.58, 0.05, 16.0),
                _airfoil_loop(10.50, 2.35, 0.40, 0.80, 11.0),
                _airfoil_loop(18.40, 1.55, 0.23, 1.55, 6.5),
                _airfoil_loop(24.80, 0.82, 0.11, 2.10, 2.8),
            ]
        )
    )
    root_fairing = CylinderGeometry(radius=0.58, height=1.80, radial_segments=28).translate(
        0.0, 0.0, 0.60
    )
    return _merge_geometries([blade, root_fairing])


def _element_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lower, upper = aabb
    return (
        0.5 * (lower[0] + upper[0]),
        0.5 * (lower[1] + upper[1]),
        0.5 * (lower[2] + upper[2]),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="horizontal_axis_wind_turbine")

    concrete = model.material("concrete", rgba=(0.70, 0.71, 0.72, 1.0))
    tower_white = model.material("tower_white", rgba=(0.93, 0.94, 0.95, 1.0))
    nacelle_white = model.material("nacelle_white", rgba=(0.95, 0.96, 0.97, 1.0))
    machinery_grey = model.material("machinery_grey", rgba=(0.40, 0.43, 0.47, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.20, 0.23, 1.0))
    blade_white = model.material("blade_white", rgba=(0.92, 0.93, 0.94, 1.0))
    safety_red = model.material("safety_red", rgba=(0.78, 0.12, 0.11, 1.0))

    tower_mesh = _save_mesh("tower_shell", _make_tower_mesh())
    nacelle_mesh = _save_mesh("nacelle_shell", _make_nacelle_shell_mesh())
    hub_mesh = _save_mesh("hub_shell", _make_hub_mesh())
    spinner_mesh = _save_mesh("spinner_shell", _make_spinner_mesh())
    blade_base = _make_single_blade_mesh()
    blade_meshes = [
        _save_mesh("blade_0", blade_base.copy()),
        _save_mesh("blade_1", blade_base.copy().rotate_x(2.0 * math.pi / 3.0)),
        _save_mesh("blade_2", blade_base.copy().rotate_x(4.0 * math.pi / 3.0)),
    ]

    base = model.part("base")
    base.visual(
        Cylinder(radius=6.80, length=1.20),
        origin=Origin(xyz=(0.0, 0.0, 0.60)),
        material=concrete,
        name="foundation_pad",
    )
    base.visual(
        Cylinder(radius=4.75, length=0.60),
        origin=Origin(xyz=(0.0, 0.0, 1.50)),
        material=concrete,
        name="tower_plinth",
    )
    base.visual(tower_mesh, material=tower_white, name="tower_shell")
    base.visual(
        Cylinder(radius=2.05, length=0.34),
        origin=Origin(xyz=(0.0, 0.0, 79.43)),
        material=machinery_grey,
        name="tower_top_flange",
    )
    base.visual(
        Cylinder(radius=1.45, length=0.80),
        origin=Origin(xyz=(0.0, 0.0, 79.20)),
        material=dark_steel,
        name="tower_inner_stub",
    )
    base.inertial = Inertial.from_geometry(
        Box((13.6, 13.6, 79.9)),
        mass=220000.0,
        origin=Origin(xyz=(0.0, 0.0, 39.95)),
    )

    bearing_module = model.part("bearing_module")
    bearing_module.visual(
        Cylinder(radius=2.15, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=dark_steel,
        name="bearing_lower_flange",
    )
    bearing_module.visual(
        Cylinder(radius=1.92, length=0.82),
        origin=Origin(xyz=(0.0, 0.0, 0.65)),
        material=machinery_grey,
        name="yaw_bearing_drum",
    )
    bearing_module.visual(
        Cylinder(radius=2.20, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 1.15)),
        material=dark_steel,
        name="bearing_upper_flange",
    )
    for y_sign in (-1.0, 1.0):
        bearing_module.visual(
            Box((0.55, 0.24, 0.46)),
            origin=Origin(xyz=(0.0, y_sign * 1.78, 0.70)),
            material=machinery_grey,
            name=f"yaw_drive_{int(y_sign):+d}",
        )
    bearing_module.inertial = Inertial.from_geometry(
        Cylinder(radius=2.20, length=1.24),
        mass=12000.0,
        origin=Origin(xyz=(0.0, 0.0, 0.62)),
    )

    head_module = model.part("head_module")
    head_module.visual(
        Cylinder(radius=2.18, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=dark_steel,
        name="yaw_deck",
    )
    head_module.visual(
        Box((6.20, 3.30, 0.70)),
        origin=Origin(xyz=(0.90, 0.0, 0.55)),
        material=machinery_grey,
        name="bedplate",
    )
    head_module.visual(
        nacelle_mesh,
        origin=Origin(xyz=(-3.00, 0.0, 2.45)),
        material=nacelle_white,
        name="nacelle_shell",
    )
    head_module.visual(
        Cylinder(radius=0.84, length=4.40),
        origin=Origin(xyz=(6.40, 0.0, 3.00), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machinery_grey,
        name="main_shaft_housing",
    )
    head_module.visual(
        Box((2.10, 2.10, 1.20)),
        origin=Origin(xyz=(-1.80, 0.0, 2.55)),
        material=machinery_grey,
        name="generator_tail_box",
    )
    head_module.visual(
        Box((1.80, 1.50, 0.72)),
        origin=Origin(xyz=(0.40, 0.0, 4.18)),
        material=machinery_grey,
        name="roof_cooler",
    )
    head_module.inertial = Inertial.from_geometry(
        Box((12.0, 5.0, 5.0)),
        mass=56000.0,
        origin=Origin(xyz=(3.70, 0.0, 2.40)),
    )

    rotor = model.part("rotor")
    rotor.visual(hub_mesh, material=machinery_grey, name="hub_shell")
    rotor.visual(spinner_mesh, material=safety_red, name="spinner")
    rotor.visual(blade_meshes[0], material=blade_white, name="blade_0")
    rotor.visual(blade_meshes[1], material=blade_white, name="blade_1")
    rotor.visual(blade_meshes[2], material=blade_white, name="blade_2")
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=26.2, length=4.4),
        mass=19000.0,
        origin=Origin(xyz=(0.5, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "base_to_bearing",
        ArticulationType.FIXED,
        parent=base,
        child=bearing_module,
        origin=Origin(xyz=(0.0, 0.0, 79.60)),
    )
    model.articulation(
        "yaw_rotation",
        ArticulationType.CONTINUOUS,
        parent=bearing_module,
        child=head_module,
        origin=Origin(xyz=(0.0, 0.0, 1.24)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180000.0, velocity=0.30),
    )
    model.articulation(
        "rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=head_module,
        child=rotor,
        origin=Origin(xyz=(10.40, 0.0, 3.00)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=240000.0, velocity=1.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    bearing = object_model.get_part("bearing_module")
    head = object_model.get_part("head_module")
    rotor = object_model.get_part("rotor")
    yaw = object_model.get_articulation("yaw_rotation")
    spin = object_model.get_articulation("rotor_spin")

    ctx.expect_gap(
        bearing,
        base,
        axis="z",
        positive_elem="bearing_lower_flange",
        negative_elem="tower_top_flange",
        max_gap=0.001,
        max_penetration=1e-5,
        name="bearing module seats on tower top flange",
    )
    ctx.expect_gap(
        head,
        bearing,
        axis="z",
        positive_elem="yaw_deck",
        negative_elem="bearing_upper_flange",
        max_gap=0.001,
        max_penetration=1e-5,
        name="head module sits on bearing upper flange",
    )
    ctx.expect_origin_gap(
        rotor,
        head,
        axis="x",
        min_gap=10.0,
        max_gap=10.8,
        name="rotor hub projects forward of nacelle yaw axis",
    )
    ctx.expect_gap(
        rotor,
        base,
        axis="z",
        positive_elem="hub_shell",
        negative_elem="tower_top_flange",
        min_gap=1.8,
        name="hub stays above tower top hardware",
    )

    rest_rotor = ctx.part_world_position(rotor)
    with ctx.pose({yaw: math.pi / 2.0}):
        yawed_rotor = ctx.part_world_position(rotor)
    ctx.check(
        "yaw swings nacelle and rotor around the tower axis",
        rest_rotor is not None
        and yawed_rotor is not None
        and abs(yawed_rotor[0]) < 1.0
        and yawed_rotor[1] > 7.0
        and abs(yawed_rotor[2] - rest_rotor[2]) < 0.01,
        details=f"rest={rest_rotor}, yawed={yawed_rotor}",
    )

    blade_rest = _element_center(ctx.part_element_world_aabb(rotor, elem="blade_0"))
    with ctx.pose({spin: 2.0 * math.pi / 3.0}):
        blade_spun = _element_center(ctx.part_element_world_aabb(rotor, elem="blade_0"))
    ctx.check(
        "rotor spin turns a blade around the main shaft axis",
        blade_rest is not None
        and blade_spun is not None
        and abs(blade_spun[0] - blade_rest[0]) < 0.8
        and abs(blade_spun[1] - blade_rest[1]) > 10.0
        and abs(blade_spun[2] - blade_rest[2]) > 10.0,
        details=f"rest={blade_rest}, spun={blade_spun}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
