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


ENGINE_CENTER_Z = 1.05
ENGINE_LENGTH = 2.90
ENGINE_FRONT_X = -ENGINE_LENGTH * 0.5
FAN_JOINT_X = -0.78


def _merge_geometries(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _mesh(geometry: MeshGeometry, name: str):
    return mesh_from_geometry(geometry, name)


def _radial_pattern_x(
    base_geometry: MeshGeometry,
    count: int,
    *,
    angle_offset: float = 0.0,
) -> MeshGeometry:
    patterned = MeshGeometry()
    for index in range(count):
        patterned.merge(base_geometry.copy().rotate_x(angle_offset + index * math.tau / count))
    return patterned


def _fan_blade_section(
    radius: float,
    x_center: float,
    z_center: float,
    chord: float,
    thickness: float,
) -> list[tuple[float, float, float]]:
    half_thickness = thickness * 0.5
    return [
        (x_center - 0.50 * chord, radius, z_center - 0.90 * half_thickness),
        (x_center + 0.08 * chord, radius, z_center - 0.22 * half_thickness),
        (x_center + 0.50 * chord, radius, z_center + 0.92 * half_thickness),
        (x_center - 0.14 * chord, radius, z_center + 0.26 * half_thickness),
    ]


def _build_nacelle_shell_mesh() -> MeshGeometry:
    shell = LatheGeometry.from_shell_profiles(
        [
            (0.82, 0.00),
            (0.85, 0.18),
            (0.82, 0.60),
            (0.76, 1.80),
            (0.66, 2.55),
            (0.55, 2.90),
        ],
        [
            (0.70, 0.00),
            (0.74, 0.18),
            (0.71, 0.62),
            (0.63, 1.75),
            (0.48, 2.50),
            (0.39, 2.90),
        ],
        segments=84,
        start_cap="round",
        end_cap="flat",
        lip_samples=10,
    )
    return shell.rotate_y(math.pi * 0.5).translate(ENGINE_FRONT_X, 0.0, 0.0)


def _build_spinner_mesh() -> MeshGeometry:
    spinner = LatheGeometry(
        [
            (0.0, -0.56),
            (0.03, -0.54),
            (0.08, -0.46),
            (0.13, -0.34),
            (0.18, -0.20),
            (0.21, -0.06),
            (0.17, 0.00),
            (0.0, 0.00),
        ],
        segments=64,
    )
    return spinner.rotate_y(math.pi * 0.5)


def _build_core_cone_mesh() -> MeshGeometry:
    cone = LatheGeometry(
        [
            (0.22, 0.00),
            (0.24, 0.06),
            (0.20, 0.16),
            (0.12, 0.30),
            (0.05, 0.40),
            (0.0, 0.46),
        ],
        segments=60,
    )
    return cone.rotate_y(math.pi * 0.5).translate(1.24, 0.0, 0.0)


def _build_fan_blades_mesh() -> MeshGeometry:
    blade = repair_loft(
        section_loft(
            [
                _fan_blade_section(0.14, -0.20, -0.018, 0.17, 0.034),
                _fan_blade_section(0.28, -0.17, 0.000, 0.15, 0.026),
                _fan_blade_section(0.46, -0.11, 0.022, 0.11, 0.016),
                _fan_blade_section(0.62, -0.06, 0.038, 0.08, 0.009),
            ]
        )
    )
    return _radial_pattern_x(blade, 14, angle_offset=math.pi / 14.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="display_geared_turbofan")

    stand_paint = model.material("stand_paint", rgba=(0.21, 0.23, 0.26, 1.0))
    nacelle_paint = model.material("nacelle_paint", rgba=(0.86, 0.88, 0.90, 1.0))
    metallic = model.material("metallic", rgba=(0.63, 0.66, 0.71, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.28, 0.30, 0.33, 1.0))

    stand = model.part("display_stand")
    stand.visual(
        Box((1.90, 2.00, 0.08)),
        origin=Origin(xyz=(-0.15, 0.25, 0.04)),
        material=stand_paint,
        name="base",
    )
    stand.visual(
        Box((0.22, 0.26, 1.90)),
        origin=Origin(xyz=(-0.10, 1.05, 1.03)),
        material=stand_paint,
        name="mast",
    )
    stand.visual(
        Box((0.20, 0.98, 0.08)),
        origin=Origin(xyz=(-0.10, 0.56, 1.97)),
        material=stand_paint,
        name="pylon_arm",
    )
    stand.visual(
        Box((0.24, 0.18, 0.06)),
        origin=Origin(xyz=(-0.10, 0.0, 1.90)),
        material=dark_metal,
        name="saddle",
    )

    nacelle = model.part("nacelle")
    nacelle.visual(
        _mesh(_build_nacelle_shell_mesh(), "nacelle_shell"),
        material=nacelle_paint,
        name="nacelle_shell",
    )
    nacelle.visual(
        Box((0.20, 0.16, 0.10)),
        origin=Origin(xyz=(-0.10, 0.0, 0.75)),
        material=dark_metal,
        name="mount_fairing",
    )
    nacelle.visual(
        Box((0.22, 0.18, 0.04)),
        origin=Origin(xyz=(-0.10, 0.0, 0.80)),
        material=dark_metal,
        name="mount_pad",
    )
    nacelle.visual(
        Cylinder(radius=0.17, length=0.30),
        origin=Origin(xyz=(-0.63, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=dark_metal,
        name="bearing_collar",
    )
    nacelle.visual(
        Cylinder(radius=0.22, length=1.72),
        origin=Origin(xyz=(0.38, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=dark_metal,
        name="core_sleeve",
    )
    nacelle.visual(
        _mesh(_build_core_cone_mesh(), "core_cone"),
        material=metallic,
        name="core_cone",
    )
    nacelle.visual(
        Box((0.12, 0.41, 0.05)),
        origin=Origin(xyz=(0.25, 0.425, 0.0)),
        material=dark_metal,
        name="strut_pos_y",
    )
    nacelle.visual(
        Box((0.12, 0.41, 0.05)),
        origin=Origin(xyz=(0.25, -0.425, 0.0)),
        material=dark_metal,
        name="strut_neg_y",
    )
    nacelle.visual(
        Box((0.12, 0.05, 0.41)),
        origin=Origin(xyz=(0.25, 0.0, 0.425)),
        material=dark_metal,
        name="strut_pos_z",
    )
    nacelle.visual(
        Box((0.12, 0.05, 0.41)),
        origin=Origin(xyz=(0.25, 0.0, -0.425)),
        material=dark_metal,
        name="strut_neg_z",
    )

    fan_rotor = model.part("fan_rotor")
    fan_rotor.visual(
        Cylinder(radius=0.17, length=0.17),
        origin=Origin(xyz=(-0.085, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=dark_metal,
        name="fan_hub",
    )
    fan_rotor.visual(
        _mesh(_build_spinner_mesh(), "spinner"),
        material=metallic,
        name="spinner",
    )
    fan_rotor.visual(
        _mesh(_build_fan_blades_mesh(), "fan_blades"),
        material=metallic,
        name="fan_blades",
    )

    model.articulation(
        "stand_to_nacelle",
        ArticulationType.FIXED,
        parent=stand,
        child=nacelle,
        origin=Origin(xyz=(0.0, 0.0, ENGINE_CENTER_Z)),
    )
    model.articulation(
        "fan_spin",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=fan_rotor,
        origin=Origin(xyz=(FAN_JOINT_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=140.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("display_stand")
    nacelle = object_model.get_part("nacelle")
    fan_rotor = object_model.get_part("fan_rotor")
    fan_spin = object_model.get_articulation("fan_spin")

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
        stand,
        nacelle,
        elem_a="saddle",
        elem_b="mount_pad",
        contact_tol=1e-4,
        name="display_pylon_supports_nacelle",
    )
    ctx.expect_contact(
        fan_rotor,
        nacelle,
        elem_a="fan_hub",
        elem_b="bearing_collar",
        contact_tol=1e-4,
        name="fan_rotor_clips_to_central_hub",
    )
    ctx.expect_within(
        fan_rotor,
        nacelle,
        axes=("y", "z"),
        margin=0.0,
        name="fan_rotor_stays_inside_nacelle_profile",
    )
    ctx.expect_overlap(
        fan_rotor,
        nacelle,
        axes=("y", "z"),
        min_overlap=0.30,
        name="fan_rotor_is_centered_in_nacelle",
    )

    fan_axis_ok = tuple(round(value, 3) for value in fan_spin.axis) == (1.0, 0.0, 0.0)
    fan_type_ok = fan_spin.joint_type == ArticulationType.CONTINUOUS
    fan_limits = fan_spin.motion_limits
    fan_limit_ok = fan_limits is not None and fan_limits.lower is None and fan_limits.upper is None
    ctx.check(
        "front_fan_uses_continuous_centerline_spin",
        fan_axis_ok and fan_type_ok and fan_limit_ok,
        details=(
            f"joint_type={fan_spin.joint_type}, axis={fan_spin.axis}, "
            f"limits={None if fan_limits is None else (fan_limits.lower, fan_limits.upper)}"
        ),
    )

    with ctx.pose({fan_spin: 1.8}):
        ctx.expect_contact(
            fan_rotor,
            nacelle,
            elem_a="fan_hub",
            elem_b="bearing_collar",
            contact_tol=1e-4,
            name="fan_hub_remains_clipped_while_spinning",
        )
        ctx.expect_within(
            fan_rotor,
            nacelle,
            axes=("y", "z"),
            margin=0.0,
            name="fan_rotor_remains_centered_while_spinning",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
