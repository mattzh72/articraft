from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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
    mesh_from_geometry,
    repair_loft,
    section_loft,
    superellipse_profile,
)


TOWER_HEIGHT = 8.0
ROTOR_AXIS_HEIGHT = 0.26
ROTOR_ORIGIN_X = 0.56


def _save_mesh(geometry: MeshGeometry, name: str):
    return mesh_from_geometry(geometry, name)


def _merge_geometries(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _radial_pattern_x(
    base_geometry: MeshGeometry,
    count: int,
    *,
    angle_offset: float = 0.0,
) -> MeshGeometry:
    patterned = MeshGeometry()
    for index in range(count):
        patterned.merge(
            base_geometry.copy().rotate_x(angle_offset + (index * math.tau / count))
        )
    return patterned


def _superellipse_section_x(
    x_pos: float,
    width: float,
    height: float,
    *,
    z_center: float,
    exponent: float = 2.9,
    segments: int = 40,
) -> list[tuple[float, float, float]]:
    return [
        (x_pos, y_pos, z_center + z_pos)
        for y_pos, z_pos in superellipse_profile(
            width,
            height,
            exponent=exponent,
            segments=segments,
        )
    ]


def _blade_section(
    z_pos: float,
    *,
    chord: float,
    thickness: float,
    x_offset: float,
    y_offset: float,
    twist: float,
) -> list[tuple[float, float, float]]:
    half_t = thickness * 0.5
    base_loop = [
        (-0.54 * chord, -0.18 * half_t),
        (-0.24 * chord, 0.82 * half_t),
        (0.18 * chord, 1.00 * half_t),
        (0.50 * chord, 0.34 * half_t),
        (0.56 * chord, 0.00 * half_t),
        (0.46 * chord, -0.28 * half_t),
        (0.06 * chord, -0.92 * half_t),
        (-0.38 * chord, -0.64 * half_t),
    ]
    cos_t = math.cos(twist)
    sin_t = math.sin(twist)
    section: list[tuple[float, float, float]] = []
    for x_local, y_local in base_loop:
        x_rot = (x_local * cos_t) - (y_local * sin_t)
        y_rot = (x_local * sin_t) + (y_local * cos_t)
        section.append((x_offset + x_rot, y_offset + y_rot, z_pos))
    return section


def _build_tower_shell_mesh() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.240, 0.00),
            (0.222, 1.20),
            (0.194, 3.80),
            (0.162, 6.50),
            (0.128, TOWER_HEIGHT - 0.03),
        ],
        [
            (0.214, 0.02),
            (0.198, 1.20),
            (0.172, 3.80),
            (0.144, 6.50),
            (0.108, TOWER_HEIGHT - 0.05),
        ],
        segments=88,
        start_cap="flat",
        end_cap="flat",
    )


def _build_nacelle_shell_mesh() -> MeshGeometry:
    sections = [
        _superellipse_section_x(-0.36, 0.20, 0.18, z_center=0.24, exponent=2.7),
        _superellipse_section_x(-0.10, 0.34, 0.30, z_center=0.26, exponent=2.9),
        _superellipse_section_x(0.18, 0.44, 0.38, z_center=0.29, exponent=3.0),
        _superellipse_section_x(0.40, 0.38, 0.31, z_center=0.27, exponent=2.8),
        _superellipse_section_x(0.50, 0.24, 0.20, z_center=0.26, exponent=2.5),
    ]
    return repair_loft(section_loft(sections))


def _build_spinner_mesh() -> MeshGeometry:
    return LatheGeometry(
        [
            (0.0, 0.32),
            (0.032, 0.30),
            (0.092, 0.24),
            (0.138, 0.15),
            (0.165, 0.06),
            (0.168, 0.0),
            (0.0, 0.0),
        ],
        segments=72,
    ).rotate_y(math.pi / 2.0)


def _build_hub_shell_mesh() -> MeshGeometry:
    central_drum = CylinderGeometry(
        radius=0.172,
        height=0.24,
        radial_segments=56,
    ).rotate_y(math.pi / 2.0).translate(0.18, 0.0, 0.0)
    front_shoulder = CylinderGeometry(
        radius=0.148,
        height=0.08,
        radial_segments=48,
    ).rotate_y(math.pi / 2.0).translate(0.30, 0.0, 0.0)
    pitch_housing = CylinderGeometry(
        radius=0.086,
        height=0.28,
        radial_segments=40,
    ).translate(0.20, 0.0, 0.22)
    root_pad = CylinderGeometry(
        radius=0.098,
        height=0.10,
        radial_segments=40,
    ).translate(0.18, 0.0, 0.10)
    return _merge_geometries(
        [
            central_drum,
            front_shoulder,
            _radial_pattern_x(pitch_housing, 3),
            _radial_pattern_x(root_pad, 3),
        ]
    )


def _build_blade_mesh() -> MeshGeometry:
    blade = repair_loft(
        section_loft(
            [
                _blade_section(
                    0.30,
                    chord=0.22,
                    thickness=0.050,
                    x_offset=0.28,
                    y_offset=0.0,
                    twist=0.36,
                ),
                _blade_section(
                    0.62,
                    chord=0.18,
                    thickness=0.038,
                    x_offset=0.24,
                    y_offset=-0.015,
                    twist=0.28,
                ),
                _blade_section(
                    1.02,
                    chord=0.128,
                    thickness=0.024,
                    x_offset=0.18,
                    y_offset=-0.036,
                    twist=0.18,
                ),
                _blade_section(
                    1.38,
                    chord=0.082,
                    thickness=0.013,
                    x_offset=0.13,
                    y_offset=-0.067,
                    twist=0.08,
                ),
                _blade_section(
                    1.58,
                    chord=0.024,
                    thickness=0.006,
                    x_offset=0.09,
                    y_offset=-0.092,
                    twist=0.02,
                ),
            ]
        )
    )
    root_shank = CylinderGeometry(
        radius=0.055,
        height=0.30,
        radial_segments=32,
    ).translate(0.27, 0.0, 0.20)
    return _merge_geometries([root_shank, blade])


def _build_blade_root_boots_mesh() -> MeshGeometry:
    cuff = CylinderGeometry(
        radius=0.070,
        height=0.08,
        radial_segments=32,
    ).translate(0.24, 0.0, 0.22)
    return _radial_pattern_x(cuff, 3)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_consumer_wind_turbine")

    painted_metal = model.material("painted_metal", rgba=(0.86, 0.88, 0.90, 1.0))
    hub_metal = model.material("hub_metal", rgba=(0.46, 0.49, 0.53, 1.0))
    polymer_shell = model.material("polymer_shell", rgba=(0.95, 0.96, 0.97, 1.0))
    elastomer = model.material("elastomer", rgba=(0.14, 0.15, 0.16, 1.0))
    base_metal = model.material("base_metal", rgba=(0.34, 0.36, 0.39, 1.0))

    tower = model.part("tower")
    tower.visual(
        _save_mesh(_build_tower_shell_mesh(), "tower_shell"),
        material=painted_metal,
        name="tower_shell",
    )
    tower.visual(
        Cylinder(radius=0.275, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=base_metal,
        name="tower_base_flange",
    )
    tower.visual(
        Cylinder(radius=0.116, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, TOWER_HEIGHT - 0.05)),
        material=base_metal,
        name="tower_top_flange",
    )
    tower.inertial = Inertial.from_geometry(
        Cylinder(radius=0.20, length=TOWER_HEIGHT),
        mass=240.0,
        origin=Origin(xyz=(0.0, 0.0, TOWER_HEIGHT * 0.5)),
    )

    nacelle = model.part("nacelle")
    nacelle.visual(
        _save_mesh(_build_nacelle_shell_mesh(), "nacelle_shell"),
        material=painted_metal,
        name="nacelle_shell",
    )
    nacelle.visual(
        Cylinder(radius=0.116, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=hub_metal,
        name="tower_mount",
    )
    nacelle.visual(
        Cylinder(radius=0.108, length=0.14),
        origin=Origin(
            xyz=(ROTOR_ORIGIN_X - 0.07, 0.0, ROTOR_AXIS_HEIGHT),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=hub_metal,
        name="front_bearing_face",
    )
    nacelle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.22, length=0.96),
        mass=95.0,
        origin=Origin(
            xyz=(0.10, 0.0, 0.27),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.120, length=0.06),
        origin=Origin(
            xyz=(0.03, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=hub_metal,
        name="rear_bearing_collar",
    )
    rotor.visual(
        _save_mesh(_build_hub_shell_mesh(), "hub_shell"),
        material=hub_metal,
        name="hub_shell",
    )
    rotor.visual(
        _save_mesh(_build_spinner_mesh().translate(0.28, 0.0, 0.0), "spinner_cap"),
        material=polymer_shell,
        name="spinner_cap",
    )
    rotor.visual(
        _save_mesh(_radial_pattern_x(_build_blade_mesh(), 3), "blade_set"),
        material=polymer_shell,
        name="blade_set",
    )
    rotor.visual(
        _save_mesh(_build_blade_root_boots_mesh(), "blade_root_boots"),
        material=elastomer,
        name="blade_root_boots",
    )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.95, length=0.46),
        mass=28.0,
        origin=Origin(
            xyz=(0.18, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
    )

    model.articulation(
        "tower_to_nacelle",
        ArticulationType.FIXED,
        parent=tower,
        child=nacelle,
        origin=Origin(xyz=(0.0, 0.0, TOWER_HEIGHT)),
    )
    model.articulation(
        "nacelle_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=rotor,
        origin=Origin(xyz=(ROTOR_ORIGIN_X, 0.0, ROTOR_AXIS_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=32.0, velocity=2.8),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    nacelle = object_model.get_part("nacelle")
    rotor = object_model.get_part("rotor")
    rotor_spin = object_model.get_articulation("nacelle_to_rotor")

    tower_top_flange = tower.get_visual("tower_top_flange")
    tower_mount = nacelle.get_visual("tower_mount")
    front_bearing_face = nacelle.get_visual("front_bearing_face")
    rear_bearing_collar = rotor.get_visual("rear_bearing_collar")

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
        tower,
        nacelle,
        elem_a=tower_top_flange,
        elem_b=tower_mount,
        name="nacelle_is_supported_by_tower",
    )
    ctx.expect_contact(
        rotor,
        nacelle,
        elem_a=rear_bearing_collar,
        elem_b=front_bearing_face,
        name="rotor_is_supported_by_bearing_face",
    )
    ctx.expect_origin_gap(
        rotor,
        nacelle,
        axis="x",
        min_gap=0.55,
        max_gap=0.65,
        name="rotor_projects_forward_of_nacelle",
    )
    ctx.expect_origin_distance(
        rotor,
        nacelle,
        axes="y",
        max_dist=0.001,
        name="rotor_remains_centered_on_nacelle",
    )

    with ctx.pose({rotor_spin: math.pi / 3.0}):
        ctx.expect_contact(
            rotor,
            nacelle,
            elem_a=rear_bearing_collar,
            elem_b=front_bearing_face,
            name="bearing_support_persists_through_rotation",
        )
        ctx.fail_if_parts_overlap_in_current_pose(
            name="no_part_overlaps_at_offset_rotor_pose"
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
