from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin, tau

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    ConeGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
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
    superellipse_profile,
    sweep_profile_along_spline,
    wire_from_points,
)


def _save_mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, name)


def _merge_geometries(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _xy_loop(
    profile: list[tuple[float, float]],
    z: float,
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(x + dx, y + dy, z) for x, y in profile]


def _xz_loop(
    profile: list[tuple[float, float]],
    y: float,
    *,
    dx: float = 0.0,
    dz: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(x + dx, y, z + dz) for x, z in profile]


def _ring_about_y(radius: float, tube: float, y: float) -> MeshGeometry:
    return (
        TorusGeometry(
            radius=radius,
            tube=tube,
            radial_segments=18,
            tubular_segments=54,
        )
        .rotate_x(-pi / 2.0)
        .translate(0.0, y, 0.0)
    )


def _straight_wire(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    radius: float,
    radial_segments: int = 12,
) -> MeshGeometry:
    return wire_from_points(
        [start, end],
        radius=radius,
        radial_segments=radial_segments,
        cap_ends=True,
    )


def _radial_guard_wire(
    angle: float,
    *,
    inner_radius: float,
    outer_radius: float,
    y: float,
    wire_radius: float,
) -> MeshGeometry:
    return _straight_wire(
        (cos(angle) * inner_radius, y, sin(angle) * inner_radius),
        (cos(angle) * outer_radius, y, sin(angle) * outer_radius),
        radius=wire_radius,
    )


def _guard_bridge(
    angle: float,
    *,
    radius: float,
    y0: float,
    y1: float,
    wire_radius: float,
) -> MeshGeometry:
    x = cos(angle) * radius
    z = sin(angle) * radius
    return _straight_wire((x, y0, z), (x, y1, z), radius=wire_radius)


def _fan_blade_section(
    radius: float,
    y_center: float,
    z_center: float,
    chord: float,
    thickness: float,
) -> list[tuple[float, float, float]]:
    half_thickness = thickness * 0.5
    return [
        (radius, y_center - 0.95 * half_thickness, z_center - 0.52 * chord),
        (radius, y_center + 0.20 * half_thickness, z_center - 0.12 * chord),
        (radius, y_center + 1.00 * half_thickness, z_center + 0.46 * chord),
        (radius, y_center - 0.22 * half_thickness, z_center + 0.10 * chord),
    ]


def _build_base_shell_mesh() -> MeshGeometry:
    return repair_loft(
        section_loft(
            [
                _xy_loop(rounded_rect_profile(0.250, 0.196, 0.040, corner_segments=8), 0.004),
                _xy_loop(rounded_rect_profile(0.240, 0.186, 0.036, corner_segments=8), 0.014),
                _xy_loop(rounded_rect_profile(0.212, 0.156, 0.028, corner_segments=8), 0.029),
            ]
        )
    )


def _build_neck_yoke_mesh() -> MeshGeometry:
    column = repair_loft(
        section_loft(
            [
                _xy_loop(rounded_rect_profile(0.066, 0.042, 0.013, corner_segments=8), 0.035, dy=-0.024),
                _xy_loop(rounded_rect_profile(0.060, 0.038, 0.012, corner_segments=8), 0.126, dy=-0.024),
                _xy_loop(rounded_rect_profile(0.052, 0.034, 0.010, corner_segments=8), 0.215, dy=-0.024),
            ]
        )
    )
    shoulder = BoxGeometry((0.108, 0.040, 0.026)).translate(0.0, -0.024, 0.210)
    arm_profile = rounded_rect_profile(0.018, 0.012, 0.0035, corner_segments=6)
    left_arm = sweep_profile_along_spline(
        [
            (0.028, -0.024, 0.204),
            (0.058, -0.026, 0.234),
            (0.092, -0.030, 0.264),
            (0.122, -0.036, 0.288),
            (0.136, -0.040, 0.296),
        ],
        profile=arm_profile,
        samples_per_segment=12,
        cap_profile=True,
    )
    right_arm = sweep_profile_along_spline(
        [
            (-0.028, -0.024, 0.204),
            (-0.058, -0.026, 0.234),
            (-0.092, -0.030, 0.264),
            (-0.122, -0.036, 0.288),
            (-0.136, -0.040, 0.296),
        ],
        profile=arm_profile,
        samples_per_segment=12,
        cap_profile=True,
    )
    rear_strap_profile = rounded_rect_profile(0.010, 0.008, 0.0025, corner_segments=5)
    left_rear_strap = sweep_profile_along_spline(
        [
            (0.128, -0.039, 0.294),
            (0.142, -0.047, 0.292),
            (0.156, -0.050, 0.294),
        ],
        profile=rear_strap_profile,
        samples_per_segment=10,
        cap_profile=True,
    )
    right_rear_strap = sweep_profile_along_spline(
        [
            (-0.128, -0.039, 0.294),
            (-0.142, -0.047, 0.292),
            (-0.156, -0.050, 0.294),
        ],
        profile=rear_strap_profile,
        samples_per_segment=10,
        cap_profile=True,
    )
    left_pivot = (
        CylinderGeometry(radius=0.030, height=0.016, radial_segments=32)
        .rotate_y(pi / 2.0)
        .translate(0.161, -0.024, 0.312)
    )
    right_pivot = (
        CylinderGeometry(radius=0.030, height=0.016, radial_segments=32)
        .rotate_y(pi / 2.0)
        .translate(-0.161, -0.024, 0.312)
    )
    left_knob = (
        CylinderGeometry(radius=0.034, height=0.010, radial_segments=32)
        .rotate_y(pi / 2.0)
        .translate(0.174, -0.024, 0.312)
    )
    right_knob = (
        CylinderGeometry(radius=0.034, height=0.010, radial_segments=32)
        .rotate_y(pi / 2.0)
        .translate(-0.174, -0.024, 0.312)
    )
    return _merge_geometries(
        [
            column,
            shoulder,
            left_arm,
            right_arm,
            left_rear_strap,
            right_rear_strap,
            left_pivot,
            right_pivot,
            left_knob,
            right_knob,
        ]
    )


def _build_head_housing_mesh() -> MeshGeometry:
    shell = repair_loft(
        section_loft(
            [
                _xz_loop(superellipse_profile(0.060, 0.060, exponent=2.2, segments=36), -0.020),
                _xz_loop(superellipse_profile(0.118, 0.112, exponent=2.5, segments=40), -0.052),
                _xz_loop(superellipse_profile(0.156, 0.150, exponent=2.6, segments=44), -0.086),
                _xz_loop(superellipse_profile(0.128, 0.122, exponent=2.4, segments=40), -0.120),
                _xz_loop(superellipse_profile(0.046, 0.046, exponent=2.0, segments=28), -0.150),
            ]
        )
    )
    front_collar = (
        CylinderGeometry(radius=0.029, height=0.012, radial_segments=32)
        .rotate_x(-pi / 2.0)
        .translate(0.0, -0.018, 0.0)
    )
    rear_cap = (
        CylinderGeometry(radius=0.018, height=0.010, radial_segments=28)
        .rotate_x(-pi / 2.0)
        .translate(0.0, -0.152, 0.0)
    )
    return _merge_geometries([shell, front_collar, rear_cap])


def _build_head_trunnion_mesh() -> MeshGeometry:
    left_boss = BoxGeometry((0.048, 0.032, 0.052)).translate(0.101, -0.024, 0.0)
    right_boss = BoxGeometry((0.048, 0.032, 0.052)).translate(-0.101, -0.024, 0.0)
    left_trunnion = (
        CylinderGeometry(radius=0.025, height=0.028, radial_segments=30)
        .rotate_y(pi / 2.0)
        .translate(0.139, -0.024, 0.0)
    )
    right_trunnion = (
        CylinderGeometry(radius=0.025, height=0.028, radial_segments=30)
        .rotate_y(pi / 2.0)
        .translate(-0.139, -0.024, 0.0)
    )
    return _merge_geometries([left_boss, right_boss, left_trunnion, right_trunnion])


def _build_guard_mesh() -> MeshGeometry:
    geometries = [
        _ring_about_y(0.145, 0.0050, 0.058),
        _ring_about_y(0.145, 0.0050, -0.022),
    ]
    for radius in (0.116, 0.086, 0.056):
        geometries.append(_ring_about_y(radius, 0.0018, 0.058))
        geometries.append(_ring_about_y(radius, 0.0018, -0.022))
    for index in range(12):
        angle = tau * index / 12.0
        geometries.append(
            _radial_guard_wire(
                angle,
                inner_radius=0.022,
                outer_radius=0.141,
                y=0.058,
                wire_radius=0.0016,
            )
        )
        geometries.append(
            _radial_guard_wire(
                angle + (tau / 24.0),
                inner_radius=0.024,
                outer_radius=0.141,
                y=-0.022,
                wire_radius=0.0016,
            )
        )
    for angle in (0.0, pi / 3.0, 2.0 * pi / 3.0, pi, 4.0 * pi / 3.0, 5.0 * pi / 3.0):
        geometries.append(
            _guard_bridge(
                angle,
                radius=0.145,
                y0=-0.022,
                y1=0.058,
                wire_radius=0.0020,
            )
        )
    geometries.append(
        CylinderGeometry(radius=0.024, height=0.008, radial_segments=28)
        .rotate_x(-pi / 2.0)
        .translate(0.0, -0.014, 0.0)
    )
    return _merge_geometries(geometries)


def _build_rotor_blade_hub_mesh() -> MeshGeometry:
    hub = (
        CylinderGeometry(radius=0.040, height=0.023, radial_segments=36)
        .rotate_x(-pi / 2.0)
        .translate(0.0, 0.0015, 0.0)
    )
    root_ring = (
        CylinderGeometry(radius=0.046, height=0.006, radial_segments=36)
        .rotate_x(-pi / 2.0)
        .translate(0.0, -0.001, 0.0)
    )
    rear_mount = (
        CylinderGeometry(radius=0.018, height=0.010, radial_segments=24)
        .rotate_x(-pi / 2.0)
        .translate(0.0, -0.015, 0.0)
    )
    blade = repair_loft(
        section_loft(
            [
                _fan_blade_section(0.034, -0.004, -0.010, 0.050, 0.013),
                _fan_blade_section(0.060, 0.002, 0.001, 0.076, 0.012),
                _fan_blade_section(0.090, 0.008, 0.013, 0.070, 0.010),
                _fan_blade_section(0.116, 0.012, 0.023, 0.052, 0.007),
                _fan_blade_section(0.128, 0.014, 0.027, 0.030, 0.004),
            ]
        )
    )
    blades = MeshGeometry()
    for index in range(3):
        blades.merge(blade.copy().rotate_y(index * tau / 3.0))
    return _merge_geometries([hub, root_ring, rear_mount, blades])


def _build_spinner_mesh() -> MeshGeometry:
    return (
        ConeGeometry(radius=0.028, height=0.024, radial_segments=36)
        .rotate_x(-pi / 2.0)
        .translate(0.0, 0.000, 0.0)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_tilting_fan")

    painted_metal = model.material("painted_metal", rgba=(0.84, 0.85, 0.82, 1.0))
    graphite_polymer = model.material("graphite_polymer", rgba=(0.23, 0.24, 0.25, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.63, 0.66, 0.69, 1.0))
    rotor_polymer = model.material("rotor_polymer", rgba=(0.68, 0.70, 0.73, 1.0))
    elastomer = model.material("elastomer", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base")
    base.inertial = Inertial.from_geometry(
        Box((0.28, 0.22, 0.34)),
        mass=4.4,
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
    )
    base.visual(
        _save_mesh("base_shell", _build_base_shell_mesh()),
        material=painted_metal,
        name="base_shell",
    )
    base.visual(
        _save_mesh(
            "base_insert",
            ExtrudeGeometry.from_z0(
                rounded_rect_profile(0.142, 0.108, 0.022, corner_segments=8),
                0.006,
            ).translate(0.0, -0.010, 0.029),
        ),
        material=graphite_polymer,
        name="base_insert",
    )
    base.visual(
        _save_mesh(
            "foot_pad",
            ExtrudeGeometry.from_z0(
                rounded_rect_profile(0.216, 0.168, 0.034, corner_segments=8),
                0.004,
            ),
        ),
        material=elastomer,
        name="foot_pad",
    )
    base.visual(
        Box((0.058, 0.036, 0.200)),
        origin=Origin(xyz=(0.0, -0.024, 0.129)),
        material=graphite_polymer,
        name="neck_column",
    )
    base.visual(
        Box((0.112, 0.040, 0.020)),
        origin=Origin(xyz=(0.0, -0.024, 0.219)),
        material=graphite_polymer,
        name="neck_shoulder",
    )
    base.visual(
        Box((0.116, 0.018, 0.024)),
        origin=Origin(xyz=(0.114, -0.032, 0.241)),
        material=graphite_polymer,
        name="left_support_arm",
    )
    base.visual(
        Box((0.116, 0.018, 0.024)),
        origin=Origin(xyz=(-0.114, -0.032, 0.241)),
        material=graphite_polymer,
        name="right_support_arm",
    )
    base.visual(
        Box((0.012, 0.024, 0.118)),
        origin=Origin(xyz=(0.178, -0.032, 0.294)),
        material=graphite_polymer,
        name="left_pivot_upright",
    )
    base.visual(
        Box((0.012, 0.024, 0.118)),
        origin=Origin(xyz=(-0.178, -0.032, 0.294)),
        material=graphite_polymer,
        name="right_pivot_upright",
    )
    base.visual(
        Cylinder(radius=0.028, length=0.038),
        origin=Origin(xyz=(0.172, -0.024, 0.312), rpy=(0.0, pi / 2.0, 0.0)),
        material=graphite_polymer,
        name="left_pivot_socket",
    )
    base.visual(
        Cylinder(radius=0.028, length=0.038),
        origin=Origin(xyz=(-0.172, -0.024, 0.312), rpy=(0.0, pi / 2.0, 0.0)),
        material=graphite_polymer,
        name="right_pivot_socket",
    )
    base.visual(
        Cylinder(radius=0.032, length=0.010),
        origin=Origin(xyz=(0.196, -0.024, 0.312), rpy=(0.0, pi / 2.0, 0.0)),
        material=elastomer,
        name="left_pivot_knob",
    )
    base.visual(
        Cylinder(radius=0.032, length=0.010),
        origin=Origin(xyz=(-0.196, -0.024, 0.312), rpy=(0.0, pi / 2.0, 0.0)),
        material=elastomer,
        name="right_pivot_knob",
    )
    base.visual(
        Box((0.024, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, -0.088, 0.016)),
        material=graphite_polymer,
        name="cord_exit",
    )
    base.visual(
        Box((0.036, 0.016, 0.006)),
        origin=Origin(xyz=(0.0, -0.090, 0.008)),
        material=elastomer,
        name="cord_strain_relief",
    )

    head = model.part("head")
    head.inertial = Inertial.from_geometry(
        Cylinder(radius=0.160, length=0.200),
        mass=1.8,
        origin=Origin(xyz=(0.0, -0.028, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
    )
    head.visual(
        _save_mesh("head_housing", _build_head_housing_mesh()),
        material=painted_metal,
        name="housing_shell",
    )
    head.visual(
        _save_mesh("head_trunnions", _build_head_trunnion_mesh()),
        material=graphite_polymer,
        name="pivot_trunnions",
    )
    head.visual(
        _save_mesh("guard_assembly", _build_guard_mesh()),
        material=satin_metal,
        name="guard_assembly",
    )
    head.visual(
        Cylinder(radius=0.022, length=0.006),
        origin=Origin(xyz=(0.0, 0.060, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=graphite_polymer,
        name="front_badge",
    )
    head.visual(
        Cylinder(radius=0.011, length=0.034),
        origin=Origin(xyz=(0.0, -0.003, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="motor_shaft",
    )

    rotor = model.part("rotor")
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.130, length=0.032),
        mass=0.26,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
    )
    rotor.visual(
        _save_mesh("rotor_blade_hub", _build_rotor_blade_hub_mesh()),
        origin=Origin(xyz=(0.0, 0.034, 0.0)),
        material=rotor_polymer,
        name="blade_hub_assembly",
    )
    rotor.visual(
        _save_mesh("spinner_cap", _build_spinner_mesh()),
        origin=Origin(xyz=(0.0, 0.034, 0.0)),
        material=satin_metal,
        name="spinner_cap",
    )

    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(0.0, -0.024, 0.312)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.8, lower=-0.55, upper=0.70),
    )
    model.articulation(
        "rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=rotor,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=35.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    head = object_model.get_part("head")
    rotor = object_model.get_part("rotor")
    tilt = object_model.get_articulation("head_tilt")
    spin = object_model.get_articulation("rotor_spin")

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

    ctx.expect_origin_gap(
        head,
        base,
        axis="z",
        min_gap=0.29,
        max_gap=0.33,
        name="head sits above weighted base",
    )
    ctx.expect_contact(
        head,
        base,
        elem_a="pivot_trunnions",
        elem_b="left_pivot_socket",
        name="head is carried by side pivots",
    )
    ctx.expect_contact(
        rotor,
        head,
        elem_a="blade_hub_assembly",
        elem_b="motor_shaft",
        name="rotor hub is seated on motor shaft",
    )
    ctx.expect_within(
        rotor,
        head,
        axes="xz",
        margin=0.0,
        name="rotor stays within guarded head envelope",
    )
    ctx.expect_gap(
        head,
        rotor,
        axis="y",
        positive_elem="front_badge",
        negative_elem="spinner_cap",
        min_gap=0.006,
        max_gap=0.020,
        name="front guard clears spinner",
    )

    ctx.check(
        "tilt articulation is explicit side-axis pivot",
        tilt.axis == (1.0, 0.0, 0.0)
        and tilt.motion_limits is not None
        and tilt.motion_limits.lower == -0.55
        and tilt.motion_limits.upper == 0.70,
        details=f"axis={tilt.axis} limits={tilt.motion_limits}",
    )
    ctx.check(
        "rotor spin is explicit hub axis",
        spin.axis == (0.0, 1.0, 0.0) and spin.motion_limits is not None,
        details=f"axis={spin.axis} limits={spin.motion_limits}",
    )

    front_badge_rest = ctx.part_element_world_aabb(head, elem="front_badge")
    with ctx.pose({tilt: 0.55}):
        front_badge_up = ctx.part_element_world_aabb(head, elem="front_badge")
    if front_badge_rest is not None and front_badge_up is not None:
        rest_center_z = 0.5 * (front_badge_rest[0][2] + front_badge_rest[1][2])
        up_center_z = 0.5 * (front_badge_up[0][2] + front_badge_up[1][2])
        ctx.check(
            "positive tilt raises fan nose",
            up_center_z > rest_center_z + 0.015,
            details=f"rest_z={rest_center_z:.4f}, up_z={up_center_z:.4f}",
        )
    else:
        ctx.fail("positive tilt raises fan nose", "front_badge AABB was unavailable")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
