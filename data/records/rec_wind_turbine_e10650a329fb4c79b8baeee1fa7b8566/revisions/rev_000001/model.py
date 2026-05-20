from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    LoftGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _save_mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, name)


def _frustum_mesh(
    *,
    bottom_radius: float,
    top_radius: float,
    height: float,
    z0: float = 0.0,
    segments: int = 64,
) -> MeshGeometry:
    """Closed tapered cylinder mesh centered on the tower axis."""
    geom = MeshGeometry()
    bottom: list[int] = []
    top: list[int] = []
    for index in range(segments):
        angle = math.tau * index / segments
        ca = math.cos(angle)
        sa = math.sin(angle)
        bottom.append(geom.add_vertex(bottom_radius * ca, bottom_radius * sa, z0))
        top.append(geom.add_vertex(top_radius * ca, top_radius * sa, z0 + height))
    bottom_center = geom.add_vertex(0.0, 0.0, z0)
    top_center = geom.add_vertex(0.0, 0.0, z0 + height)
    for index in range(segments):
        nxt = (index + 1) % segments
        geom.add_face(bottom[index], bottom[nxt], top[nxt])
        geom.add_face(bottom[index], top[nxt], top[index])
        geom.add_face(bottom_center, bottom[index], bottom[nxt])
        geom.add_face(top_center, top[nxt], top[index])
    return geom


def _blade_section(
    radius: float,
    chord: float,
    thickness: float,
    pitch_deg: float,
    sweep: float,
) -> list[tuple[float, float, float]]:
    """Airfoil-like section for a blade whose span initially follows +Z."""
    pitch = math.radians(pitch_deg)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    # Points are ordered around a thick, rounded airfoil in the local axial-x /
    # tangential-y section plane; pitch twists the chord along the span.
    section_2d = [
        (-0.35 * thickness, -0.50 * chord),
        (0.22 * thickness, -0.46 * chord),
        (0.58 * thickness, -0.20 * chord),
        (0.52 * thickness, 0.18 * chord),
        (0.12 * thickness, 0.50 * chord),
        (-0.40 * thickness, 0.36 * chord),
        (-0.55 * thickness, 0.06 * chord),
        (-0.50 * thickness, -0.28 * chord),
    ]
    loop: list[tuple[float, float, float]] = []
    for axial_x, tangential_y in section_2d:
        x = axial_x * cp - tangential_y * sp
        y = axial_x * sp + tangential_y * cp + sweep
        loop.append((x, y, radius))
    return loop


def _single_blade_mesh() -> MeshGeometry:
    sections = [
        _blade_section(0.90, 3.60, 0.85, 15.0, -0.10),
        _blade_section(3.20, 4.40, 0.72, 12.0, -0.22),
        _blade_section(10.0, 3.80, 0.48, 8.5, -0.48),
        _blade_section(24.0, 2.85, 0.30, 5.0, -0.92),
        _blade_section(40.0, 1.80, 0.18, 2.6, -1.25),
        _blade_section(56.0, 0.72, 0.08, 1.0, -1.55),
    ]
    return LoftGeometry(sections, cap=True, closed=True)


def _nacelle_shell_mesh() -> MeshGeometry:
    sections: list[list[tuple[float, float, float]]] = []
    for x, width_y, height_z, z_center in [
        (-5.0, 3.05, 2.65, 2.25),
        (-2.6, 3.85, 3.65, 2.62),
        (1.5, 4.20, 4.05, 2.72),
        (4.8, 3.55, 3.45, 2.62),
        (6.35, 2.38, 2.35, 2.58),
    ]:
        loop: list[tuple[float, float, float]] = []
        half_w = width_y * 0.5
        half_h = height_z * 0.5
        exponent = 2.55
        for index in range(72):
            angle = math.tau * index / 72
            ca = math.cos(angle)
            sa = math.sin(angle)
            y = half_w * math.copysign(abs(ca) ** (2.0 / exponent), ca)
            z = z_center + half_h * math.copysign(abs(sa) ** (2.0 / exponent), sa)
            # LoftGeometry profiles must lie in the XY plane at constant Z.
            # Build the nacelle along raw +Z, then rotate raw +Z into final +X.
            loop.append((-z, y, x))
        sections.append(loop)
    return LoftGeometry(sections, cap=True, closed=True).rotate_y(math.pi / 2.0)


def _three_blade_rotor_mesh() -> MeshGeometry:
    blade = _single_blade_mesh()
    rotor = MeshGeometry()
    for index in range(3):
        rotor.merge(blade.copy().rotate_x(index * math.tau / 3.0))
    return rotor


def _blade_root_cuffs_mesh() -> MeshGeometry:
    cuffs = MeshGeometry()
    cuff = CylinderGeometry(radius=0.55, height=2.25, radial_segments=28).translate(
        0.0,
        0.0,
        1.05,
    )
    for index in range(3):
        cuffs.merge(cuff.copy().rotate_x(index * math.tau / 3.0))
    return cuffs


def _spinner_mesh() -> MeshGeometry:
    # Lathe is around local +Z; rotate so the pointed spinner projects along +X.
    return LatheGeometry(
        [
            (0.0, 0.08),
            (1.02, 0.08),
            (1.16, 0.20),
            (0.96, 0.64),
            (0.56, 1.18),
            (0.16, 1.62),
            (0.0, 1.78),
        ],
        segments=72,
    ).rotate_y(math.pi / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_horizontal_axis_wind_turbine")

    tower_white = model.material("tower_white", rgba=(0.86, 0.88, 0.86, 1.0))
    nacelle_white = model.material("nacelle_white", rgba=(0.90, 0.91, 0.88, 1.0))
    blade_white = model.material("blade_white", rgba=(0.93, 0.94, 0.91, 1.0))
    bearing_steel = model.material("bearing_steel", rgba=(0.28, 0.30, 0.31, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.14, 0.15, 0.16, 1.0))
    concrete = model.material("concrete", rgba=(0.50, 0.51, 0.50, 1.0))
    safety_red = model.material("safety_red", rgba=(0.70, 0.08, 0.06, 1.0))

    tower = model.part("tower")
    tower.visual(
        Cylinder(radius=5.5, length=0.80),
        origin=Origin(xyz=(0.0, 0.0, 0.40)),
        material=concrete,
        name="foundation_pad",
    )
    tower.visual(
        Cylinder(radius=3.25, length=0.35),
        origin=Origin(xyz=(0.0, 0.0, 0.965)),
        material=bearing_steel,
        name="base_flange",
    )
    tower.visual(
        _save_mesh(
            "tapered_tower_shell",
            _frustum_mesh(bottom_radius=2.65, top_radius=1.38, height=88.80, z0=1.10),
        ),
        material=tower_white,
        name="tapered_tower_shell",
    )
    tower.visual(
        Box((0.10, 1.10, 2.15)),
        origin=Origin(xyz=(2.62, 0.0, 2.35)),
        material=dark_metal,
        name="service_door",
    )
    tower.visual(
        Cylinder(radius=1.92, length=0.55),
        origin=Origin(xyz=(0.0, 0.0, 89.725)),
        material=bearing_steel,
        name="tower_top_bearing",
    )
    tower.inertial = Inertial.from_geometry(
        Cylinder(radius=2.0, length=90.0),
        mass=210000.0,
        origin=Origin(xyz=(0.0, 0.0, 45.0)),
    )

    nacelle = model.part("nacelle")
    nacelle.visual(
        Cylinder(radius=1.62, length=0.65),
        origin=Origin(xyz=(0.0, 0.0, 0.325)),
        material=bearing_steel,
        name="yaw_skirt",
    )
    nacelle.visual(
        Cylinder(radius=1.25, length=0.36),
        origin=Origin(xyz=(0.0, 0.0, 0.83)),
        material=nacelle_white,
        name="yaw_neck",
    )
    nacelle.visual(
        Box((11.95, 0.85, 1.65)),
        origin=Origin(xyz=(1.165, 0.0, 1.825)),
        material=dark_metal,
        name="internal_bedplate",
    )
    nacelle.visual(
        Box((3.60, 2.40, 1.00)),
        origin=Origin(xyz=(0.0, 0.0, 1.10)),
        material=nacelle_white,
        name="yaw_fairing",
    )
    nacelle.visual(
        _save_mesh("nacelle_shell", _nacelle_shell_mesh()),
        material=nacelle_white,
        name="nacelle_shell",
    )
    nacelle.visual(
        Box((3.30, 0.16, 0.16)),
        origin=Origin(xyz=(0.8, 0.0, 4.76)),
        material=dark_metal,
        name="roof_service_rail",
    )
    nacelle.visual(
        Cylinder(radius=1.05, length=0.70),
        origin=Origin(xyz=(6.45, 0.0, 2.65), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=nacelle_white,
        name="nose_fairing",
    )
    nacelle.visual(
        Cylinder(radius=0.88, length=0.50),
        origin=Origin(xyz=(6.95, 0.0, 2.65), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing_steel,
        name="nose_bearing",
    )
    nacelle.visual(
        Cylinder(radius=0.48, length=0.30),
        origin=Origin(xyz=(7.03, 0.0, 2.65), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="main_shaft_stub",
    )
    nacelle.inertial = Inertial.from_geometry(
        Box((11.5, 4.2, 4.2)),
        mass=240000.0,
        origin=Origin(xyz=(0.7, 0.0, 2.65)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=1.18, length=1.20),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing_steel,
        name="hub_shell",
    )
    rotor.visual(
        _save_mesh("hub_spinner", _spinner_mesh()),
        material=blade_white,
        name="hub_spinner",
    )
    rotor.visual(
        _save_mesh("blade_root_cuffs", _blade_root_cuffs_mesh()),
        material=bearing_steel,
        name="blade_root_cuffs",
    )
    rotor.visual(
        _save_mesh("three_blades", _three_blade_rotor_mesh()),
        material=blade_white,
        name="three_blades",
    )
    rotor.visual(
        Cylinder(radius=0.22, length=0.16),
        origin=Origin(xyz=(1.86, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=safety_red,
        name="nose_tip",
    )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=56.0, length=1.2),
        mass=110000.0,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "yaw",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=nacelle,
        origin=Origin(xyz=(0.0, 0.0, 90.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=7.5e6, velocity=0.18),
    )
    model.articulation(
        "rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=rotor,
        origin=Origin(xyz=(7.80, 0.0, 2.65)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0e6, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    nacelle = object_model.get_part("nacelle")
    rotor = object_model.get_part("rotor")
    yaw = object_model.get_articulation("yaw")
    spin = object_model.get_articulation("rotor_spin")

    ctx.check(
        "nacelle yaw is continuous",
        yaw.articulation_type == ArticulationType.CONTINUOUS and yaw.axis == (0.0, 0.0, 1.0),
        details=f"type={yaw.articulation_type}, axis={yaw.axis}",
    )
    ctx.check(
        "rotor spin is continuous about shaft",
        spin.articulation_type == ArticulationType.CONTINUOUS and spin.axis == (1.0, 0.0, 0.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )

    ctx.expect_gap(
        nacelle,
        tower,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="yaw_skirt",
        negative_elem="tower_top_bearing",
        name="yaw skirt sits on tower bearing",
    )
    ctx.expect_overlap(
        nacelle,
        tower,
        axes="xy",
        min_overlap=2.8,
        elem_a="yaw_skirt",
        elem_b="tower_top_bearing",
        name="yaw bearing has broad circular support",
    )
    ctx.expect_contact(
        rotor,
        nacelle,
        elem_a="hub_shell",
        elem_b="nose_bearing",
        contact_tol=0.003,
        name="rotor hub seats against nose bearing",
    )

    rest_rotor_pos = ctx.part_world_position(rotor)
    with ctx.pose({yaw: math.pi / 2.0}):
        yawed_rotor_pos = ctx.part_world_position(rotor)
    ctx.check(
        "yaw carries rotor around vertical tower axis",
        rest_rotor_pos is not None
        and yawed_rotor_pos is not None
        and yawed_rotor_pos[1] > rest_rotor_pos[1] + 7.0
        and abs(yawed_rotor_pos[2] - rest_rotor_pos[2]) < 0.01,
        details=f"rest={rest_rotor_pos}, yawed={yawed_rotor_pos}",
    )

    with ctx.pose({spin: math.pi / 2.0}):
        spun_rotor_pos = ctx.part_world_position(rotor)
    ctx.check(
        "rotor spin holds shaft origin fixed",
        rest_rotor_pos is not None
        and spun_rotor_pos is not None
        and max(abs(a - b) for a, b in zip(rest_rotor_pos, spun_rotor_pos)) < 0.001,
        details=f"rest={rest_rotor_pos}, spun={spun_rotor_pos}",
    )

    return ctx.report()


object_model = build_object_model()
