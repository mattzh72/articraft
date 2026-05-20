from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    repair_loft,
    section_loft,
)


TOWER_HEIGHT = 2.50
NACELLE_AXIS_Z = 0.225
ROTOR_X = -0.450
ROTOR_RADIUS = 0.720


def _merge(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _lathe_x(profile: list[tuple[float, float]], *, segments: int = 72) -> MeshGeometry:
    """Lathe a radius/axis profile and rotate it so the axis runs along local X."""
    return LatheGeometry(profile, segments=segments).rotate_y(math.pi / 2.0)


def _tower_shell() -> MeshGeometry:
    # A gently tapered painted-metal pole with a small bottom and top fillet read.
    return LatheGeometry(
        [
            (0.0, 0.050),
            (0.072, 0.050),
            (0.064, 0.140),
            (0.056, 0.800),
            (0.049, 1.700),
            (0.043, 2.420),
            (0.038, 2.490),
            (0.0, 2.490),
        ],
        segments=72,
    )


def _annular_sleeve_x(
    *,
    x_min: float,
    x_max: float,
    outer_radius: float,
    inner_radius: float,
    segments: int = 72,
) -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, x_min), (outer_radius, x_max)],
        [(inner_radius, x_min), (inner_radius, x_max)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    ).rotate_y(math.pi / 2.0)


def _nacelle_body() -> MeshGeometry:
    body = _lathe_x(
        [
            (0.0, -0.340),
            (0.052, -0.322),
            (0.100, -0.260),
            (0.132, -0.120),
            (0.138, 0.070),
            (0.118, 0.230),
            (0.070, 0.360),
            (0.020, 0.430),
            (0.0, 0.448),
        ],
        segments=88,
    )
    body.translate(0.0, 0.0, NACELLE_AXIS_Z)
    return body


def _nacelle_pedestal() -> MeshGeometry:
    # A short faired neck from the yaw bearing into the underside of the nacelle.
    return LatheGeometry(
        [
            (0.0, 0.018),
            (0.100, 0.018),
            (0.094, 0.052),
            (0.074, 0.130),
            (0.062, 0.160),
            (0.0, 0.160),
        ],
        segments=64,
    )


def _hub_body() -> MeshGeometry:
    # Spinner, hub barrel, rear shoulder, and inserted shaft share one smooth load path.
    return _lathe_x(
        [
            (0.0, -0.185),
            (0.022, -0.176),
            (0.052, -0.150),
            (0.083, -0.103),
            (0.104, -0.046),
            (0.104, 0.020),
            (0.072, 0.037),
            (0.045, 0.037),
            (0.045, 0.090),
            (0.0, 0.090),
        ],
        segments=88,
    )


def _root_cuffs() -> MeshGeometry:
    root = CylinderGeometry(radius=0.038, height=0.145, radial_segments=40).translate(
        0.0, 0.0, 0.142
    )
    bevel = CylinderGeometry(radius=0.048, height=0.026, radial_segments=40).translate(
        0.0, 0.0, 0.082
    )
    base = _merge([root, bevel])
    patterned = MeshGeometry()
    for index in range(3):
        patterned.merge(base.copy().rotate_x(index * math.tau / 3.0))
    return patterned


def _airfoil_section(
    radius: float,
    chord: float,
    thickness: float,
    twist_deg: float,
    sweep_y: float,
) -> list[tuple[float, float, float]]:
    # Section loop lies normal to the blade radial axis. X is aerodynamic
    # thickness / pitch direction, Y is local chord, Z is blade radius.
    pts_2d = [
        (-0.48 * thickness, -0.50 * chord),
        (0.12 * thickness, -0.36 * chord),
        (0.50 * thickness, -0.08 * chord),
        (0.42 * thickness, 0.26 * chord),
        (0.08 * thickness, 0.50 * chord),
        (-0.34 * thickness, 0.40 * chord),
        (-0.50 * thickness, 0.08 * chord),
        (-0.42 * thickness, -0.28 * chord),
    ]
    angle = math.radians(twist_deg)
    ca = math.cos(angle)
    sa = math.sin(angle)
    return [
        (x * ca - y * sa, sweep_y + x * sa + y * ca, radius)
        for x, y in pts_2d
    ]


def _blade_mesh(angle: float) -> MeshGeometry:
    blade = repair_loft(
        section_loft(
            [
                _airfoil_section(0.150, 0.150, 0.036, 18.0, -0.010),
                _airfoil_section(0.245, 0.132, 0.030, 12.0, -0.003),
                _airfoil_section(0.390, 0.104, 0.022, 6.0, 0.010),
                _airfoil_section(0.555, 0.068, 0.014, 1.5, 0.020),
                _airfoil_section(0.710, 0.026, 0.007, -1.0, 0.026),
            ]
        )
    )
    return blade.rotate_x(angle)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_consumer_wind_turbine")

    tower_paint = model.material("warm_white_painted_metal", rgba=(0.86, 0.88, 0.87, 1.0))
    nacelle_paint = model.material("satin_white_polymer", rgba=(0.92, 0.93, 0.91, 1.0))
    dark_metal = model.material("graphite_anodized_metal", rgba=(0.15, 0.16, 0.17, 1.0))
    bright_metal = model.material("brushed_aluminum", rgba=(0.64, 0.67, 0.69, 1.0))
    blade_finish = model.material("matte_composite_blade", rgba=(0.96, 0.96, 0.93, 1.0))
    elastomer = model.material("black_elastomer_seal", rgba=(0.025, 0.027, 0.030, 1.0))

    tower = model.part("tower")
    tower.visual(
        Cylinder(radius=0.230, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_metal,
        name="base_plate",
    )
    tower.visual(
        mesh_from_geometry(_tower_shell(), "tower_shell"),
        material=tower_paint,
        name="tower_shell",
    )
    tower.visual(
        Cylinder(radius=0.108, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, TOWER_HEIGHT - 0.025)),
        material=bright_metal,
        name="lower_yaw_race",
    )
    tower.visual(
        Cylinder(radius=0.020, length=0.012),
        origin=Origin(xyz=(0.165, 0.000, 0.056)),
        material=bright_metal,
        name="anchor_bolt_0",
    )
    tower.visual(
        Cylinder(radius=0.020, length=0.012),
        origin=Origin(xyz=(-0.082, 0.143, 0.056)),
        material=bright_metal,
        name="anchor_bolt_1",
    )
    tower.visual(
        Cylinder(radius=0.020, length=0.012),
        origin=Origin(xyz=(-0.082, -0.143, 0.056)),
        material=bright_metal,
        name="anchor_bolt_2",
    )
    tower.inertial = Inertial.from_geometry(
        Cylinder(radius=0.08, length=TOWER_HEIGHT),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, TOWER_HEIGHT / 2.0)),
    )

    nacelle = model.part("nacelle")
    nacelle.visual(
        Cylinder(radius=0.104, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=bright_metal,
        name="upper_yaw_race",
    )
    nacelle.visual(
        mesh_from_geometry(_nacelle_pedestal(), "yaw_pedestal"),
        material=nacelle_paint,
        name="yaw_pedestal",
    )
    nacelle.visual(
        mesh_from_geometry(_nacelle_body(), "nacelle_body"),
        material=nacelle_paint,
        name="nacelle_body",
    )
    bearing = _annular_sleeve_x(
        x_min=-0.395,
        x_max=-0.285,
        outer_radius=0.083,
        inner_radius=0.045,
    )
    bearing.translate(0.0, 0.0, NACELLE_AXIS_Z)
    nacelle.visual(
        mesh_from_geometry(bearing, "nose_bearing"),
        material=dark_metal,
        name="nose_bearing",
    )
    seam = TorusGeometry(radius=0.118, tube=0.0045, radial_segments=14, tubular_segments=72)
    seam.rotate_y(math.pi / 2.0).translate(0.215, 0.0, NACELLE_AXIS_Z)
    nacelle.visual(
        mesh_from_geometry(seam, "rear_service_seam"),
        material=elastomer,
        name="rear_service_seam",
    )
    nacelle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.15, length=0.72),
        mass=7.5,
        origin=Origin(xyz=(0.020, 0.0, NACELLE_AXIS_Z)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        mesh_from_geometry(_hub_body(), "hub_body"),
        material=bright_metal,
        name="hub_body",
    )
    rotor.visual(
        mesh_from_geometry(_root_cuffs(), "root_cuffs"),
        material=elastomer,
        name="root_cuffs",
    )
    for index in range(3):
        rotor.visual(
            mesh_from_geometry(_blade_mesh(index * math.tau / 3.0), f"blade_{index}"),
            material=blade_finish,
            name=f"blade_{index}",
        )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=ROTOR_RADIUS, length=0.08),
        mass=2.6,
        origin=Origin(),
    )

    model.articulation(
        "tower_to_nacelle",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=nacelle,
        origin=Origin(xyz=(0.0, 0.0, TOWER_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=24.0, velocity=0.7),
    )
    model.articulation(
        "nacelle_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=rotor,
        origin=Origin(xyz=(ROTOR_X, 0.0, NACELLE_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    nacelle = object_model.get_part("nacelle")
    rotor = object_model.get_part("rotor")
    yaw = object_model.get_articulation("tower_to_nacelle")
    spin = object_model.get_articulation("nacelle_to_rotor")

    ctx.check("major assemblies present", all(p is not None for p in (tower, nacelle, rotor)))
    ctx.check("yaw and rotor spin joints present", yaw is not None and spin is not None)
    if tower is None or nacelle is None or rotor is None or yaw is None or spin is None:
        return ctx.report()

    ctx.allow_overlap(
        nacelle,
        rotor,
        elem_a="nose_bearing",
        elem_b="hub_body",
        reason=(
            "The rotor shaft is intentionally represented as captured inside "
            "the nacelle nose bearing sleeve, giving the hub a real load path."
        ),
    )
    ctx.expect_contact(
        tower,
        nacelle,
        elem_a="lower_yaw_race",
        elem_b="upper_yaw_race",
        contact_tol=0.001,
        name="yaw bearing races are seated",
    )
    ctx.expect_within(
        rotor,
        nacelle,
        axes="yz",
        inner_elem="hub_body",
        outer_elem="nose_bearing",
        margin=0.105,
        name="hub is centered on nose bearing axis",
    )
    ctx.expect_overlap(
        rotor,
        nacelle,
        axes="x",
        elem_a="hub_body",
        elem_b="nose_bearing",
        min_overlap=0.025,
        name="rotor shaft remains captured in bearing sleeve",
    )

    rotor_aabb = ctx.part_world_aabb(rotor)
    if rotor_aabb is not None:
        mins, maxs = rotor_aabb
        rotor_span_y = float(maxs[1] - mins[1])
        rotor_span_z = float(maxs[2] - mins[2])
        ctx.check(
            "three blade rotor has broad swept disk",
            rotor_span_y > 1.05 and rotor_span_z > 1.05,
            details=f"rotor span y={rotor_span_y:.3f}, z={rotor_span_z:.3f}",
        )

    rotor_rest = ctx.part_world_position(rotor)
    with ctx.pose({yaw: 0.55}):
        rotor_yawed = ctx.part_world_position(rotor)
    ctx.check(
        "yaw joint carries nacelle and rotor around tower",
        rotor_rest is not None
        and rotor_yawed is not None
        and abs(rotor_yawed[1] - rotor_rest[1]) > 0.18,
        details=f"rest={rotor_rest}, yawed={rotor_yawed}",
    )

    return ctx.report()


object_model = build_object_model()
