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
    repair_loft,
    rounded_rect_profile,
    section_loft,
    superellipse_profile,
)


TOWER_HEIGHT = 38.5
NACELLE_FRONT_X = 3.35
ROTOR_CENTER_Z = 1.45
HUB_SOCKET_X = 0.72
HUB_SOCKET_INTERFACE_RADIUS = 2.45


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _yz_rounded_section(
    *,
    x: float,
    width_y: float,
    height_z: float,
    z_center: float,
    corner_radius: float,
    corner_segments: int = 10,
) -> list[tuple[float, float, float]]:
    profile = rounded_rect_profile(
        height_z,
        width_y,
        min(corner_radius, 0.49 * min(width_y, height_z)),
        corner_segments=corner_segments,
    )
    return [(x, y, z_center + z) for z, y in profile]


def _yz_superellipse_section(
    *,
    x: float,
    width_y: float,
    height_z: float,
    z_center: float = 0.0,
    exponent: float = 2.5,
    segments: int = 48,
) -> list[tuple[float, float, float]]:
    profile = superellipse_profile(height_z, width_y, exponent=exponent, segments=segments)
    return [(x, y, z_center + z) for z, y in profile]


def _airfoil_section(
    y_span: float,
    *,
    chord: float,
    thickness: float,
    sweep_x: float = 0.0,
    twist_deg: float = 0.0,
    z_offset: float = 0.0,
) -> list[tuple[float, float, float]]:
    half_t = 0.5 * thickness
    section_2d = [
        (-0.50 * chord, 0.00 * half_t),
        (-0.38 * chord, 0.44 * half_t),
        (-0.08 * chord, 1.00 * half_t),
        (0.26 * chord, 0.84 * half_t),
        (0.48 * chord, 0.24 * half_t),
        (0.50 * chord, -0.18 * half_t),
        (0.28 * chord, -0.66 * half_t),
        (-0.10 * chord, -0.92 * half_t),
        (-0.44 * chord, -0.22 * half_t),
    ]
    twist = math.radians(twist_deg)
    cos_t = math.cos(twist)
    sin_t = math.sin(twist)
    return [
        (
            sweep_x + (x * cos_t) + (z * sin_t),
            y_span,
            z_offset - (x * sin_t) + (z * cos_t),
        )
        for x, z in section_2d
    ]


def _radial_point(x: float, radius: float, roll_about_x: float) -> tuple[float, float, float]:
    return (
        x,
        radius * math.cos(roll_about_x),
        radius * math.sin(roll_about_x),
    )


def _build_tower_shell():
    outer_profile = [
        (2.45, 0.00),
        (2.40, 2.50),
        (2.24, 11.00),
        (1.98, 23.00),
        (1.72, 33.50),
        (1.56, 38.20),
    ]
    inner_profile = [
        (2.36, 0.14),
        (2.31, 2.50),
        (2.15, 11.00),
        (1.90, 23.00),
        (1.65, 33.50),
        (1.49, 38.20),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=88,
        start_cap="flat",
        end_cap="flat",
    )


def _build_nacelle_shell():
    sections = [
        _yz_rounded_section(
            x=-1.80,
            width_y=2.20,
            height_z=1.90,
            z_center=1.30,
            corner_radius=0.36,
        ),
        _yz_rounded_section(
            x=0.20,
            width_y=2.95,
            height_z=2.45,
            z_center=1.45,
            corner_radius=0.46,
        ),
        _yz_rounded_section(
            x=2.10,
            width_y=2.70,
            height_z=2.20,
            z_center=1.40,
            corner_radius=0.42,
        ),
        _yz_rounded_section(
            x=3.25,
            width_y=1.70,
            height_z=1.60,
            z_center=1.22,
            corner_radius=0.30,
        ),
    ]
    return repair_loft(section_loft(sections))


def _build_hub_shell():
    sections = [
        _yz_superellipse_section(x=0.00, width_y=1.56, height_z=1.56, exponent=2.2),
        _yz_superellipse_section(x=0.48, width_y=2.18, height_z=2.10, exponent=2.35),
        _yz_superellipse_section(x=1.15, width_y=2.04, height_z=1.98, exponent=2.35),
        _yz_superellipse_section(x=1.76, width_y=0.90, height_z=0.88, exponent=2.2),
        _yz_superellipse_section(x=2.02, width_y=0.18, height_z=0.18, exponent=2.0),
    ]
    return repair_loft(section_loft(sections))


def _build_blade_shell(span: float = 16.0):
    sections = [
        _airfoil_section(0.42, chord=2.20, thickness=0.46, sweep_x=0.10, twist_deg=18.0),
        _airfoil_section(2.40, chord=1.95, thickness=0.34, sweep_x=0.04, twist_deg=14.0),
        _airfoil_section(5.80, chord=1.52, thickness=0.24, sweep_x=-0.10, twist_deg=10.0),
        _airfoil_section(9.80, chord=1.02, thickness=0.15, sweep_x=-0.28, twist_deg=7.0),
        _airfoil_section(13.30, chord=0.64, thickness=0.09, sweep_x=-0.44, twist_deg=4.8),
        _airfoil_section(span, chord=0.32, thickness=0.035, sweep_x=-0.58, twist_deg=2.8),
    ]
    return repair_loft(section_loft(sections))


def _build_blade_part(model: ArticulatedObject, name: str, *, blade_skin, root_metal, mark_color):
    blade = model.part(name)
    blade.visual(
        Cylinder(radius=0.50, length=0.18),
        origin=Origin(xyz=(0.0, 0.09, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=root_metal,
        name="root_flange",
    )
    blade.visual(
        Cylinder(radius=0.43, length=0.48),
        origin=Origin(xyz=(0.0, 0.33, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=root_metal,
        name="root_barrel",
    )
    blade.visual(
        Box((0.20, 0.10, 0.06)),
        origin=Origin(xyz=(0.06, 0.16, 0.37)),
        material=root_metal,
        name="root_datum_flat",
    )
    blade.visual(
        Box((0.06, 0.06, 0.18)),
        origin=Origin(xyz=(0.37, 0.06, 0.0)),
        material=mark_color,
        name="root_index_mark",
    )
    blade.visual(
        _mesh(f"{name}_shell", _build_blade_shell()),
        material=blade_skin,
        name="blade_shell",
    )
    blade.visual(
        Box((0.26, 0.78, 0.18)),
        origin=Origin(xyz=(0.64, 2.65, 0.02)),
        material=blade_skin,
        name="pitch_pad",
    )
    blade.visual(
        Box((0.10, 0.46, 0.06)),
        origin=Origin(xyz=(0.74, 2.65, 0.12)),
        material=mark_color,
        name="pitch_witness",
    )
    blade.inertial = Inertial.from_geometry(
        Box((1.20, 16.20, 0.80)),
        mass=6200.0,
        origin=Origin(xyz=(0.0, 8.10, 0.0)),
    )
    return blade


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_wind_turbine")

    tower_white = model.material("tower_white", rgba=(0.88, 0.90, 0.92, 1.0))
    nacelle_white = model.material("nacelle_white", rgba=(0.84, 0.86, 0.89, 1.0))
    blade_white = model.material("blade_white", rgba=(0.93, 0.94, 0.95, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.56, 0.60, 0.66, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.28, 0.31, 0.35, 1.0))
    index_orange = model.material("index_orange", rgba=(0.92, 0.46, 0.12, 1.0))

    tower = model.part("tower")
    tower.visual(_mesh("tower_shell", _build_tower_shell()), material=tower_white, name="tower_shell")
    tower.visual(
        Cylinder(radius=2.62, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=machined_steel,
        name="base_flange",
    )
    tower.visual(
        Cylinder(radius=1.95, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 38.42)),
        material=machined_steel,
        name="top_flange",
    )
    tower.visual(
        Cylinder(radius=1.62, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 38.27)),
        material=machined_steel,
        name="top_transition_spigot",
    )
    tower.visual(
        Box((0.42, 0.18, 0.06)),
        origin=Origin(xyz=(1.45, 0.0, 38.47)),
        material=index_orange,
        name="azimuth_datum_pad",
    )
    tower.inertial = Inertial.from_geometry(
        Cylinder(radius=2.45, length=TOWER_HEIGHT),
        mass=125000.0,
        origin=Origin(xyz=(0.0, 0.0, TOWER_HEIGHT * 0.5)),
    )

    nacelle = model.part("nacelle")
    nacelle.visual(
        Cylinder(radius=1.82, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=machined_steel,
        name="yaw_ring",
    )
    nacelle.visual(
        Box((4.40, 2.60, 0.32)),
        origin=Origin(xyz=(0.20, 0.0, 0.31)),
        material=dark_steel,
        name="bedplate",
    )
    nacelle.visual(
        _mesh("nacelle_shell", _build_nacelle_shell()),
        material=nacelle_white,
        name="nacelle_shell",
    )
    nacelle.visual(
        Cylinder(radius=0.64, length=1.10),
        origin=Origin(xyz=(2.80, 0.0, ROTOR_CENTER_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="spindle_nose",
    )
    nacelle.visual(
        Box((0.46, 0.18, 0.03)),
        origin=Origin(xyz=(0.15, -0.70, 2.67)),
        material=machined_steel,
        name="survey_pad_port",
    )
    nacelle.visual(
        Box((0.46, 0.18, 0.03)),
        origin=Origin(xyz=(0.15, 0.70, 2.67)),
        material=machined_steel,
        name="survey_pad_starboard",
    )
    nacelle.visual(
        Cylinder(radius=0.08, length=0.22),
        origin=Origin(xyz=(-0.85, -1.05, 0.31)),
        material=machined_steel,
        name="yaw_trim_jack_port",
    )
    nacelle.visual(
        Cylinder(radius=0.08, length=0.22),
        origin=Origin(xyz=(-0.85, 1.05, 0.31)),
        material=machined_steel,
        name="yaw_trim_jack_starboard",
    )
    nacelle.visual(
        Box((0.16, 0.04, 0.10)),
        origin=Origin(xyz=(1.45, 0.0, 0.07)),
        material=index_orange,
        name="azimuth_pointer",
    )
    nacelle.inertial = Inertial.from_geometry(
        Box((5.50, 3.20, 2.90)),
        mass=56000.0,
        origin=Origin(xyz=(0.55, 0.0, 1.40)),
    )

    hub = model.part("hub")
    hub.visual(
        Cylinder(radius=0.68, length=0.12),
        origin=Origin(xyz=(0.06, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="rear_mount",
    )
    hub.visual(_mesh("hub_shell", _build_hub_shell()), material=nacelle_white, name="hub_shell")
    hub.visual(
        Box((0.30, 0.18, 0.08)),
        origin=Origin(xyz=(0.86, 0.0, 1.05)),
        material=machined_steel,
        name="hub_datum_flat",
    )

    socket_specs = [
        ("top", math.pi / 2.0),
        ("lower_a", 7.0 * math.pi / 6.0),
        ("lower_b", 11.0 * math.pi / 6.0),
    ]
    for socket_name, roll in socket_specs:
        hub.visual(
            Cylinder(radius=0.40, length=1.40),
            origin=Origin(
                xyz=_radial_point(HUB_SOCKET_X, 1.56, roll),
                rpy=(roll - math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_steel,
            name=f"socket_{socket_name}_body",
        )
        hub.visual(
            Cylinder(radius=0.52, length=0.20),
            origin=Origin(
                xyz=_radial_point(HUB_SOCKET_X, 2.35, roll),
                rpy=(roll - math.pi / 2.0, 0.0, 0.0),
            ),
            material=machined_steel,
            name=f"socket_{socket_name}_flange",
        )
        tangential = (0.0, -math.sin(roll), math.cos(roll))
        socket_center = _radial_point(HUB_SOCKET_X, 2.28, roll)
        hub.visual(
            Box((0.12, 0.08, 0.18)),
            origin=Origin(
                xyz=(
                    socket_center[0],
                    socket_center[1] + (0.43 * tangential[1]),
                    socket_center[2] + (0.43 * tangential[2]),
                )
            ),
            material=index_orange,
            name=f"socket_{socket_name}_index",
        )
    hub.inertial = Inertial.from_geometry(
        Cylinder(radius=1.20, length=2.10),
        mass=18000.0,
        origin=Origin(xyz=(0.92, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    blade_top = _build_blade_part(
        model,
        "blade_top",
        blade_skin=blade_white,
        root_metal=machined_steel,
        mark_color=index_orange,
    )
    blade_lower_a = _build_blade_part(
        model,
        "blade_lower_a",
        blade_skin=blade_white,
        root_metal=machined_steel,
        mark_color=index_orange,
    )
    blade_lower_b = _build_blade_part(
        model,
        "blade_lower_b",
        blade_skin=blade_white,
        root_metal=machined_steel,
        mark_color=index_orange,
    )

    nacelle_yaw = model.articulation(
        "tower_to_nacelle_yaw",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=nacelle,
        origin=Origin(xyz=(0.0, 0.0, TOWER_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=220000.0,
            velocity=0.20,
            lower=-math.radians(170.0),
            upper=math.radians(170.0),
        ),
    )
    rotor_spin = model.articulation(
        "nacelle_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=hub,
        origin=Origin(xyz=(NACELLE_FRONT_X, 0.0, ROTOR_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=450000.0, velocity=2.20),
    )

    blade_joints = [
        ("hub_to_blade_top_pitch", blade_top, math.pi / 2.0),
        ("hub_to_blade_lower_a_pitch", blade_lower_a, 7.0 * math.pi / 6.0),
        ("hub_to_blade_lower_b_pitch", blade_lower_b, 11.0 * math.pi / 6.0),
    ]
    for joint_name, blade_part, roll in blade_joints:
        model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent=hub,
            child=blade_part,
            origin=Origin(
                xyz=_radial_point(HUB_SOCKET_X, HUB_SOCKET_INTERFACE_RADIUS, roll),
                rpy=(roll, 0.0, 0.0),
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=90000.0,
                velocity=0.50,
                lower=-0.18,
                upper=1.45,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    nacelle = object_model.get_part("nacelle")
    hub = object_model.get_part("hub")
    blade_top = object_model.get_part("blade_top")
    blade_lower_a = object_model.get_part("blade_lower_a")
    blade_lower_b = object_model.get_part("blade_lower_b")
    nacelle_yaw = object_model.get_articulation("tower_to_nacelle_yaw")
    rotor_spin = object_model.get_articulation("nacelle_to_rotor")
    blade_top_pitch = object_model.get_articulation("hub_to_blade_top_pitch")

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

    ctx.expect_contact(
        nacelle,
        tower,
        elem_a="yaw_ring",
        elem_b="top_flange",
        name="yaw_ring_seats_on_tower_flange",
    )
    ctx.expect_overlap(
        nacelle,
        tower,
        axes="xy",
        elem_a="yaw_ring",
        elem_b="top_flange",
        min_overlap=3.20,
        name="yaw_ring_has_broad_tower_footprint",
    )
    ctx.expect_gap(
        nacelle,
        tower,
        axis="z",
        positive_elem="azimuth_pointer",
        negative_elem="azimuth_datum_pad",
        min_gap=0.015,
        max_gap=0.025,
        name="azimuth_pointer_maintains_controlled_gap",
    )
    ctx.expect_contact(
        hub,
        nacelle,
        elem_a="rear_mount",
        elem_b="spindle_nose",
        name="hub_mount_contacts_spindle_face",
    )

    for blade_part, socket_name, label in (
        (blade_top, "socket_top_flange", "top"),
        (blade_lower_a, "socket_lower_a_flange", "lower_a"),
        (blade_lower_b, "socket_lower_b_flange", "lower_b"),
    ):
        ctx.expect_contact(
            blade_part,
            hub,
            elem_a="root_flange",
            elem_b=socket_name,
            name=f"blade_root_{label}_seats_on_hub_flange",
        )

    with ctx.pose({nacelle_yaw: math.radians(15.0)}):
        hub_pos = ctx.part_world_position(hub)
        ctx.check(
            "positive_yaw_swings_rotor_to_positive_y",
            hub_pos is not None and hub_pos[1] > 0.80,
            details=f"hub position in yaw pose: {hub_pos}",
        )

    with ctx.pose({rotor_spin: 0.45}):
        blade_origin = ctx.part_world_position(blade_top)
        ctx.check(
            "positive_rotor_spin_advances_top_blade_toward_negative_y",
            blade_origin is not None and blade_origin[1] < -0.90,
            details=f"blade_top origin after rotor spin: {blade_origin}",
        )

    witness_center_rest = None
    witness_center_pitched = None
    rest_aabb = ctx.part_element_world_aabb(blade_top, elem="pitch_witness")
    if rest_aabb is not None:
        witness_center_rest = (
            0.5 * (rest_aabb[0][0] + rest_aabb[1][0]),
            0.5 * (rest_aabb[0][1] + rest_aabb[1][1]),
            0.5 * (rest_aabb[0][2] + rest_aabb[1][2]),
        )
    with ctx.pose({blade_top_pitch: 0.35}):
        pitched_aabb = ctx.part_element_world_aabb(blade_top, elem="pitch_witness")
        if pitched_aabb is not None:
            witness_center_pitched = (
                0.5 * (pitched_aabb[0][0] + pitched_aabb[1][0]),
                0.5 * (pitched_aabb[0][1] + pitched_aabb[1][1]),
                0.5 * (pitched_aabb[0][2] + pitched_aabb[1][2]),
            )
    ctx.check(
        "blade_pitch_moves_witness_mark",
        witness_center_rest is not None
        and witness_center_pitched is not None
        and (witness_center_pitched[1] - witness_center_rest[1]) > 0.20,
        details=(
            f"rest witness center: {witness_center_rest}, "
            f"pitched witness center: {witness_center_pitched}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
