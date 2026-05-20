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
    TorusGeometry,
    mesh_from_geometry,
    repair_loft,
    rounded_rect_profile,
    section_loft,
)


def _merge_geometries(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _lathe_along_x(profile: list[tuple[float, float]], *, segments: int = 72) -> MeshGeometry:
    return LatheGeometry(profile, segments=segments).rotate_y(math.pi / 2.0)


def _yz_section(
    x_pos: float,
    width: float,
    height: float,
    radius: float,
    z_center: float,
) -> list[tuple[float, float, float]]:
    return [
        (x_pos, y_pos, z_pos + z_center)
        for z_pos, y_pos in rounded_rect_profile(
            height,
            width,
            radius,
            corner_segments=10,
        )
    ]


def _airfoil_loop(
    span_z: float,
    chord: float,
    thickness: float,
    twist_deg: float,
    *,
    center_x: float = 0.0,
    center_y: float = 0.0,
) -> list[tuple[float, float, float]]:
    normalized_loop = [
        (-0.50, 0.00),
        (-0.34, 0.14),
        (-0.10, 0.36),
        (0.20, 0.42),
        (0.48, 0.12),
        (0.50, -0.02),
        (0.26, -0.22),
        (-0.02, -0.36),
        (-0.28, -0.22),
        (-0.44, -0.08),
    ]
    angle = math.radians(twist_deg)
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    section = []
    for x_norm, y_norm in normalized_loop:
        x_local = x_norm * chord
        y_local = y_norm * thickness
        x_rot = (x_local * cos_a) - (y_local * sin_a)
        y_rot = (x_local * sin_a) + (y_local * cos_a)
        section.append((center_x + x_rot, center_y + y_rot, span_z))
    return section


def _build_tower_shell() -> MeshGeometry:
    return LatheGeometry(
        [
            (0.0, 0.0),
            (1.95, 0.0),
            (1.90, 8.0),
            (1.72, 24.0),
            (1.44, 40.0),
            (1.28, 48.0),
            (0.0, 48.0),
        ],
        segments=80,
    )


def _build_nacelle_shell() -> MeshGeometry:
    return repair_loft(
        section_loft(
            [
                _yz_section(-2.30, 2.10, 2.20, 0.26, 1.95),
                _yz_section(0.20, 2.90, 3.00, 0.36, 2.10),
                _yz_section(2.70, 3.15, 3.15, 0.40, 2.18),
                _yz_section(3.65, 2.30, 2.40, 0.24, 2.14),
                _yz_section(3.95, 1.95, 1.92, 0.16, 2.14),
            ]
        )
    )


def _build_hub_core() -> MeshGeometry:
    return _lathe_along_x(
        [
            (0.0, -0.55),
            (0.50, -0.52),
            (1.06, -0.16),
            (1.24, 0.20),
            (1.20, 0.68),
            (0.92, 1.24),
            (0.44, 1.58),
            (0.0, 1.68),
        ],
        segments=80,
    )


def _blade_socket_geometry(angle: float) -> MeshGeometry:
    radius_y = -math.sin(angle)
    radius_z = math.cos(angle)
    socket = (
        CylinderGeometry(radius=0.55, height=1.40, radial_segments=36)
        .rotate_x(angle)
        .translate(0.12, radius_y * 1.25, radius_z * 1.25)
    )
    flange = (
        CylinderGeometry(radius=0.72, height=0.24, radial_segments=36)
        .rotate_x(angle)
        .translate(0.12, radius_y * 1.92, radius_z * 1.92)
    )
    return _merge_geometries(socket, flange)


def _radial_mount_origin(angle: float, radial_distance: float, *, x_pos: float) -> Origin:
    return Origin(
        xyz=(
            x_pos,
            -math.sin(angle) * radial_distance,
            math.cos(angle) * radial_distance,
        ),
        rpy=(angle, 0.0, 0.0),
    )


def _build_blade_shell() -> MeshGeometry:
    return repair_loft(
        section_loft(
            [
                _airfoil_loop(0.60, 2.60, 0.78, 14.0, center_x=0.00),
                _airfoil_loop(2.00, 2.42, 0.64, 12.5, center_x=0.05),
                _airfoil_loop(5.50, 2.10, 0.48, 10.0, center_x=0.18),
                _airfoil_loop(10.50, 1.56, 0.32, 7.0, center_x=0.45),
                _airfoil_loop(15.80, 0.94, 0.17, 4.2, center_x=0.80),
                _airfoil_loop(20.50, 0.36, 0.07, 2.0, center_x=1.08),
            ]
        )
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cost_optimized_wind_turbine")

    tower_white = model.material("tower_white", rgba=(0.90, 0.91, 0.92, 1.0))
    nacelle_white = model.material("nacelle_white", rgba=(0.93, 0.94, 0.95, 1.0))
    blade_white = model.material("blade_white", rgba=(0.95, 0.95, 0.96, 1.0))
    hub_gray = model.material("hub_gray", rgba=(0.68, 0.70, 0.73, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.28, 0.30, 0.33, 1.0))

    tower = model.part("tower")
    tower.visual(
        mesh_from_geometry(_build_tower_shell(), "tower_shell"),
        material=tower_white,
        name="tower_shell",
    )
    tower.visual(
        Cylinder(radius=2.35, length=0.35),
        origin=Origin(xyz=(0.0, 0.0, 0.175)),
        material=steel_dark,
        name="base_flange",
    )
    tower.visual(
        Cylinder(radius=1.55, length=0.25),
        origin=Origin(xyz=(0.0, 0.0, 47.875)),
        material=steel_dark,
        name="top_flange",
    )
    tower.inertial = Inertial.from_geometry(
        Cylinder(radius=1.80, length=48.0),
        mass=112000.0,
        origin=Origin(xyz=(0.0, 0.0, 24.0)),
    )

    nacelle = model.part("nacelle")
    nacelle.visual(
        mesh_from_geometry(_build_nacelle_shell(), "nacelle_shell"),
        material=nacelle_white,
        name="nacelle_shell",
    )
    nacelle.visual(
        Box((6.60, 2.10, 0.38)),
        origin=Origin(xyz=(1.00, 0.0, 0.69)),
        material=steel_dark,
        name="bedplate",
    )
    nacelle.visual(
        Cylinder(radius=1.35, length=0.50),
        origin=Origin(xyz=(0.0, 0.0, 0.25)),
        material=steel_dark,
        name="yaw_ring",
    )
    nacelle.visual(
        Box((0.64, 1.54, 1.46)),
        origin=Origin(xyz=(4.19, 0.0, 2.15)),
        material=steel_dark,
        name="bearing_carrier",
    )
    nacelle.visual(
        Box((0.36, 1.70, 0.24)),
        origin=Origin(xyz=(4.08, 0.0, 2.89)),
        material=steel_dark,
        name="bearing_clamp_cap",
    )
    nacelle.inertial = Inertial.from_geometry(
        Box((7.20, 3.20, 3.50)),
        mass=42000.0,
        origin=Origin(xyz=(1.10, 0.0, 2.05)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.28, length=0.82),
        origin=Origin(xyz=(-0.41, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_dark,
        name="main_shaft",
    )
    rotor.visual(
        Cylinder(radius=0.60, length=0.22),
        origin=Origin(xyz=(-0.93, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_dark,
        name="thrust_collar",
    )
    rotor.visual(
        mesh_from_geometry(_build_hub_core(), "hub_core"),
        material=hub_gray,
        name="hub_core",
    )
    blade_angles = [0.0, (2.0 * math.pi / 3.0), (4.0 * math.pi / 3.0)]
    for index, angle in enumerate(blade_angles, start=1):
        rotor.visual(
            Box((0.88, 1.08, 1.42)),
            origin=_radial_mount_origin(angle, 1.33, x_pos=0.12),
            material=hub_gray,
            name=f"blade_socket_{index}",
        )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=2.25, length=2.20),
        mass=9800.0,
        origin=Origin(xyz=(0.55, 0.0, 0.0)),
    )

    blade_shell_mesh = mesh_from_geometry(_build_blade_shell(), "blade_shell")
    for index in range(1, 4):
        blade = model.part(f"blade_{index}")
        blade.visual(
            Box((0.74, 0.96, 0.18)),
            origin=Origin(xyz=(0.0, 0.0, 0.09)),
            material=hub_gray,
            name="root_flange",
        )
        blade.visual(
            Cylinder(radius=0.46, length=0.96),
            origin=Origin(xyz=(0.0, 0.0, 0.66)),
            material=steel_dark,
            name="root_sleeve",
        )
        blade.visual(
            blade_shell_mesh,
            material=blade_white,
            name="blade_shell",
        )
        blade.inertial = Inertial.from_geometry(
            Box((2.60, 0.80, 20.50)),
            mass=3500.0,
            origin=Origin(xyz=(0.55, 0.0, 10.25)),
        )

    nacelle_yaw = model.articulation(
        "tower_to_nacelle_yaw",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=nacelle,
        origin=Origin(xyz=(0.0, 0.0, 48.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=220000.0, velocity=0.18),
    )
    rotor_spin = model.articulation(
        "nacelle_to_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=rotor,
        origin=Origin(xyz=(5.55, 0.0, 2.15)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=900000.0, velocity=0.35),
    )

    for index, angle in enumerate(blade_angles, start=1):
        radius_y = -math.sin(angle)
        radius_z = math.cos(angle)
        model.articulation(
            f"rotor_to_blade_{index}",
            ArticulationType.FIXED,
            parent=rotor,
            child=f"blade_{index}",
            origin=_radial_mount_origin(angle, 2.04, x_pos=0.12),
        )

    model.meta["primary_articulations"] = (nacelle_yaw.name, rotor_spin.name)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tower = object_model.get_part("tower")
    nacelle = object_model.get_part("nacelle")
    rotor = object_model.get_part("rotor")
    blade_1 = object_model.get_part("blade_1")
    blade_2 = object_model.get_part("blade_2")
    blade_3 = object_model.get_part("blade_3")

    top_flange = tower.get_visual("top_flange")
    yaw_ring = nacelle.get_visual("yaw_ring")
    nacelle_shell = nacelle.get_visual("nacelle_shell")
    bearing_carrier = nacelle.get_visual("bearing_carrier")
    main_shaft = rotor.get_visual("main_shaft")
    thrust_collar = rotor.get_visual("thrust_collar")
    hub_core = rotor.get_visual("hub_core")

    rotor_spin = object_model.get_articulation("nacelle_to_rotor_spin")
    nacelle_yaw = object_model.get_articulation("tower_to_nacelle_yaw")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "expected_part_count",
        len(object_model.parts) == 6,
        f"expected 6 parts, found {len(object_model.parts)}",
    )
    ctx.check(
        "yaw_axis_vertical",
        tuple(nacelle_yaw.axis) == (0.0, 0.0, 1.0),
        f"unexpected yaw axis {nacelle_yaw.axis}",
    )
    ctx.check(
        "rotor_axis_fore_aft",
        tuple(rotor_spin.axis) == (1.0, 0.0, 0.0),
        f"unexpected rotor axis {rotor_spin.axis}",
    )

    ctx.expect_contact(nacelle, tower, elem_a=yaw_ring, elem_b=top_flange)
    ctx.expect_contact(
        rotor,
        nacelle,
        elem_a=thrust_collar,
        elem_b=bearing_carrier,
        name="rotor_thrust_collar_seats_on_bearing_carrier",
    )
    ctx.expect_within(
        rotor,
        nacelle,
        axes="yz",
        inner_elem=main_shaft,
        outer_elem=bearing_carrier,
        margin=0.25,
        name="main_shaft_stays_within_bearing_window",
    )
    ctx.expect_gap(
        rotor,
        nacelle,
        axis="x",
        min_gap=0.18,
        max_gap=0.24,
        positive_elem=main_shaft,
        negative_elem=bearing_carrier,
        name="main_shaft_projects_forward_of_bearing_carrier",
    )
    ctx.expect_gap(
        rotor,
        nacelle,
        axis="x",
        min_gap=0.90,
        positive_elem=hub_core,
        negative_elem=nacelle_shell,
        name="hub_clears_nacelle_nose",
    )
    ctx.expect_origin_gap(rotor, tower, axis="x", min_gap=5.0)

    for index, blade in enumerate((blade_1, blade_2, blade_3), start=1):
        ctx.expect_contact(
            blade,
            rotor,
            elem_a=blade.get_visual("root_flange"),
            elem_b=rotor.get_visual(f"blade_socket_{index}"),
            name=f"blade_{index}_root_contacts_socket",
        )

    with ctx.pose({rotor_spin: math.pi / 2.0}):
        blade_1_position = ctx.part_world_position(blade_1)
        ctx.check(
            "rotor_spin_moves_upper_blade_clockwise",
            blade_1_position is not None
            and blade_1_position[1] < -1.5
            and 49.0 < blade_1_position[2] < 51.5,
            f"unexpected blade_1 root position after spin: {blade_1_position}",
        )

    with ctx.pose({nacelle_yaw: math.pi / 2.0}):
        rotor_position = ctx.part_world_position(rotor)
        ctx.check(
            "nacelle_yaw_swings_rotor_sideways",
            rotor_position is not None
            and rotor_position[1] > 5.0
            and abs(rotor_position[0]) < 1.0,
            f"unexpected rotor origin after yaw: {rotor_position}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
