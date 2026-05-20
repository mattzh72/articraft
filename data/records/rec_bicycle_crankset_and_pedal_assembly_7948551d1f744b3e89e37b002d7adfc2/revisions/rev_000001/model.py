from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
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


def _rotate_profile(
    profile: list[tuple[float, float]],
    angle: float,
) -> list[tuple[float, float]]:
    c = cos(angle)
    s = sin(angle)
    return [(c * x - s * y, s * x + c * y) for x, y in profile]


def _translate_profile(
    profile: list[tuple[float, float]],
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _circle_profile(
    radius: float,
    *,
    segments: int = 56,
    angle_offset: float = 0.0,
) -> list[tuple[float, float]]:
    return [
        (
            radius * cos(angle_offset + (2.0 * pi * index / segments)),
            radius * sin(angle_offset + (2.0 * pi * index / segments)),
        )
        for index in range(segments)
    ]


def _oval_profile(
    radius_x: float,
    radius_y: float,
    *,
    segments: int = 168,
    angle_offset: float = 0.0,
    tooth_count: int = 0,
    tooth_depth: float = 0.0,
) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for index in range(segments):
        angle = angle_offset + (2.0 * pi * index / segments)
        scale = 1.0
        if tooth_count > 0 and tooth_depth > 0.0:
            tooth_wave = 0.5 + 0.5 * sin(tooth_count * angle)
            scale += tooth_depth * tooth_wave * tooth_wave
        points.append((radius_x * scale * cos(angle), radius_y * scale * sin(angle)))
    return points


def _section_loop(
    width: float,
    depth: float,
    z: float,
    *,
    cx: float = 0.0,
    cy: float = 0.0,
    twist: float = 0.0,
) -> list[tuple[float, float, float]]:
    radius = min(width, depth) * 0.28
    profile = rounded_rect_profile(width, depth, radius, corner_segments=6)
    if twist:
        profile = _rotate_profile(profile, twist)
    return [(cx + x, cy + y, z) for x, y in profile]


def _hollow_tube_mesh(
    outer_radius: float,
    inner_radius: float,
    length: float,
    *,
    segments: int = 64,
):
    return ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius, segments=segments),
        [_circle_profile(inner_radius, segments=segments)],
        height=length,
        center=True,
    ).rotate_y(pi / 2.0)


def _chainring_mesh():
    outer = _oval_profile(
        0.112,
        0.098,
        segments=180,
        angle_offset=pi / 10.0,
        tooth_count=44,
        tooth_depth=0.018,
    )
    hole_profiles: list[list[tuple[float, float]]] = [
        _circle_profile(0.019, segments=40),
    ]
    bolt_circle_radius = 0.041
    for index in range(5):
        angle = (2.0 * pi * index / 5.0) - pi / 2.0
        hole_profiles.append(
            _translate_profile(
                _circle_profile(0.0036, segments=22),
                bolt_circle_radius * cos(angle),
                bolt_circle_radius * sin(angle),
            )
        )
        window_profile = _rotate_profile(
            rounded_rect_profile(0.026, 0.050, 0.0045, corner_segments=6),
            angle,
        )
        hole_profiles.append(
            _translate_profile(
                window_profile,
                0.062 * cos(angle),
                0.062 * sin(angle),
            )
        )
    return ExtrudeWithHolesGeometry(
        outer,
        hole_profiles,
        height=0.0042,
        center=True,
    ).rotate_y(pi / 2.0)


def _pedal_platform_mesh():
    outer = rounded_rect_profile(0.102, 0.088, 0.010, corner_segments=7)
    hole_profiles = [
        rounded_rect_profile(0.060, 0.042, 0.006, corner_segments=6),
        _translate_profile(
            rounded_rect_profile(0.020, 0.028, 0.004, corner_segments=5),
            0.0,
            0.022,
        ),
        _translate_profile(
            rounded_rect_profile(0.020, 0.028, 0.004, corner_segments=5),
            0.0,
            -0.022,
        ),
    ]
    return ExtrudeWithHolesGeometry(
        outer,
        hole_profiles,
        height=0.014,
        center=True,
    ).rotate_y(pi / 2.0)


def _add_pedal_visuals(
    part,
    *,
    mesh_name: str,
    side_sign: float,
    pedal_composite,
    satin_black,
    bolt_steel,
) -> None:
    platform_offset = 0.017 * side_sign
    pod_center = 0.013 * side_sign
    hub_center = 0.012 * side_sign
    cap_center = 0.026 * side_sign

    part.visual(
        _save_mesh(mesh_name, _pedal_platform_mesh()),
        origin=Origin(xyz=(platform_offset, 0.0, 0.0)),
        material=pedal_composite,
        name="platform",
    )
    part.visual(
        Box((0.026, 0.052, 0.020)),
        origin=Origin(xyz=(pod_center, 0.0, 0.0)),
        material=pedal_composite,
        name="bearing_pod",
    )
    part.visual(
        Cylinder(radius=0.0115, length=0.024),
        origin=Origin(xyz=(hub_center, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_black,
        name="pedal_hub",
    )
    part.visual(
        Cylinder(radius=0.0065, length=0.004),
        origin=Origin(xyz=(cap_center, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=bolt_steel,
        name="end_cap",
    )


def _right_crank_arm_mesh():
    sections = [
        _section_loop(0.052, 0.030, 0.000, cx=0.064, cy=-0.004, twist=0.10),
        _section_loop(0.046, 0.026, -0.038, cx=0.069, cy=-0.003, twist=0.07),
        _section_loop(0.041, 0.023, -0.095, cx=0.075, cy=-0.001, twist=0.04),
        _section_loop(0.037, 0.020, -0.150, cx=0.082, cy=0.000, twist=0.02),
        _section_loop(0.034, 0.018, -0.172, cx=0.084, cy=0.000, twist=0.00),
    ]
    return section_loft(sections)


def _left_crank_arm_mesh():
    sections = [
        _section_loop(0.050, 0.030, 0.000, cx=-0.030, cy=0.004, twist=-0.10),
        _section_loop(0.045, 0.026, 0.038, cx=-0.034, cy=0.003, twist=-0.07),
        _section_loop(0.040, 0.023, 0.095, cx=-0.038, cy=0.001, twist=-0.04),
        _section_loop(0.036, 0.020, 0.150, cx=-0.041, cy=0.000, twist=-0.02),
        _section_loop(0.034, 0.018, 0.172, cx=-0.042, cy=0.000, twist=0.00),
    ]
    return section_loft(sections)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="power_meter_crankset")

    alloy = model.material("alloy", rgba=(0.72, 0.74, 0.77, 1.0))
    dark_alloy = model.material("dark_alloy", rgba=(0.36, 0.38, 0.41, 1.0))
    crank_carbon = model.material("crank_carbon", rgba=(0.12, 0.13, 0.15, 1.0))
    satin_black = model.material("satin_black", rgba=(0.08, 0.09, 0.10, 1.0))
    chainring_black = model.material("chainring_black", rgba=(0.10, 0.10, 0.11, 1.0))
    bolt_steel = model.material("bolt_steel", rgba=(0.78, 0.80, 0.82, 1.0))
    pedal_composite = model.material("pedal_composite", rgba=(0.14, 0.15, 0.16, 1.0))

    shell_width = 0.068
    shell_outer_radius = 0.0205
    shell_inner_radius = 0.0175

    bb_shell = model.part("bb_shell")
    bb_shell.visual(
        _save_mesh(
            "bb_shell_main",
            _hollow_tube_mesh(shell_outer_radius, shell_inner_radius, shell_width, segments=72),
        ),
        material=alloy,
        name="bb_shell",
    )
    for side in (-1.0, 1.0):
        for offset in (0.0285, 0.0245, 0.0205):
            bb_shell.visual(
                _save_mesh(
                    f"bb_thread_{'l' if side < 0.0 else 'r'}_{int(offset * 10000)}",
                    _hollow_tube_mesh(0.0182, 0.0172, 0.0018, segments=56),
                ),
                origin=Origin(xyz=(side * offset, 0.0, 0.0)),
                material=dark_alloy,
            )
    bb_shell.inertial = Inertial.from_geometry(
        Cylinder(radius=shell_outer_radius, length=shell_width),
        mass=0.35,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    drive_side = model.part("drive_side")
    drive_side.visual(
        Cylinder(radius=0.012, length=0.145),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_alloy,
        name="spindle",
    )
    drive_side.visual(
        Cylinder(radius=shell_inner_radius, length=0.006),
        origin=Origin(xyz=(-0.037, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_alloy,
        name="left_journal",
    )
    drive_side.visual(
        Cylinder(radius=shell_inner_radius, length=0.006),
        origin=Origin(xyz=(0.037, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_alloy,
        name="right_journal",
    )
    drive_side.visual(
        Cylinder(radius=0.024, length=0.022),
        origin=Origin(xyz=(0.055, 0.0, -0.002), rpy=(0.0, pi / 2.0, 0.0)),
        material=crank_carbon,
        name="right_spindle_shoulder",
    )
    drive_side.visual(
        _save_mesh("right_crank_arm", _right_crank_arm_mesh()),
        material=crank_carbon,
        name="right_crank_arm",
    )
    drive_side.visual(
        Cylinder(radius=0.0105, length=0.022),
        origin=Origin(xyz=(0.091, 0.0, -0.172), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_alloy,
        name="right_pedal_boss",
    )
    drive_side.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(xyz=(0.081, 0.0, -0.172), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_alloy,
    )
    drive_side.visual(
        Cylinder(radius=0.028, length=0.014),
        origin=Origin(xyz=(0.068, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_black,
        name="spider_core",
    )
    drive_side.visual(
        Cylinder(radius=0.030, length=0.008),
        origin=Origin(xyz=(0.060, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_black,
        name="power_meter_pod",
    )
    drive_side.visual(
        Box((0.008, 0.020, 0.034)),
        origin=Origin(xyz=(0.056, 0.020, -0.004)),
        material=satin_black,
        name="battery_cover",
    )
    for index in range(5):
        angle = (2.0 * pi * index / 5.0) - pi / 2.0
        drive_side.visual(
            Box((0.008, 0.012, 0.056)),
            origin=Origin(
                xyz=(0.068, 0.023 * sin(angle), 0.023 * cos(angle)),
                rpy=(angle, 0.0, 0.0),
            ),
            material=satin_black,
            name=f"spider_arm_{index}",
        )
        drive_side.visual(
            Cylinder(radius=0.0042, length=0.012),
            origin=Origin(
                xyz=(0.074, 0.041 * sin(angle), 0.041 * cos(angle)),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=dark_alloy,
            name=f"chainring_bolt_standoff_{index}",
        )
        drive_side.visual(
            Cylinder(radius=0.0055, length=0.004),
            origin=Origin(
                xyz=(0.082, 0.041 * sin(angle), 0.041 * cos(angle)),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=bolt_steel,
            name=f"chainring_bolt_head_{index}",
        )
    drive_side.visual(
        _save_mesh("oval_chainring", _chainring_mesh()),
        origin=Origin(xyz=(0.078, 0.0, 0.0)),
        material=chainring_black,
        name="chainring",
    )
    drive_side.inertial = Inertial.from_geometry(
        Box((0.28, 0.24, 0.22)),
        mass=1.7,
        origin=Origin(xyz=(0.050, 0.0, -0.030)),
    )

    left_crank = model.part("left_crank")
    left_crank.visual(
        Cylinder(radius=0.023, length=0.018),
        origin=Origin(xyz=(-0.009, 0.0, 0.002), rpy=(0.0, pi / 2.0, 0.0)),
        material=crank_carbon,
        name="left_spindle_socket",
    )
    left_crank.visual(
        _save_mesh("left_crank_arm", _left_crank_arm_mesh()),
        material=crank_carbon,
        name="left_crank_arm",
    )
    left_crank.visual(
        Cylinder(radius=0.0105, length=0.022),
        origin=Origin(xyz=(-0.048, 0.0, 0.172), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_alloy,
        name="left_pedal_boss",
    )
    left_crank.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(xyz=(-0.038, 0.0, 0.172), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_alloy,
    )
    left_crank.inertial = Inertial.from_geometry(
        Box((0.14, 0.08, 0.22)),
        mass=0.72,
        origin=Origin(xyz=(-0.018, 0.0, 0.090)),
    )

    right_pedal = model.part("right_pedal")
    _add_pedal_visuals(
        right_pedal,
        mesh_name="right_pedal_platform",
        side_sign=1.0,
        pedal_composite=pedal_composite,
        satin_black=satin_black,
        bolt_steel=bolt_steel,
    )
    right_pedal.inertial = Inertial.from_geometry(
        Box((0.024, 0.102, 0.088)),
        mass=0.21,
        origin=Origin(),
    )

    left_pedal = model.part("left_pedal")
    _add_pedal_visuals(
        left_pedal,
        mesh_name="left_pedal_platform",
        side_sign=-1.0,
        pedal_composite=pedal_composite,
        satin_black=satin_black,
        bolt_steel=bolt_steel,
    )
    left_pedal.inertial = Inertial.from_geometry(
        Box((0.024, 0.102, 0.088)),
        mass=0.21,
        origin=Origin(),
    )

    model.articulation(
        "bb_spin",
        ArticulationType.CONTINUOUS,
        parent=bb_shell,
        child=drive_side,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=20.0),
    )
    model.articulation(
        "left_arm_mount",
        ArticulationType.FIXED,
        parent=drive_side,
        child=left_crank,
        origin=Origin(xyz=(-0.0725, 0.0, 0.0)),
    )
    model.articulation(
        "right_pedal_spin",
        ArticulationType.CONTINUOUS,
        parent=drive_side,
        child=right_pedal,
        origin=Origin(xyz=(0.102, 0.0, -0.172)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=35.0),
    )
    model.articulation(
        "left_pedal_spin",
        ArticulationType.CONTINUOUS,
        parent=left_crank,
        child=left_pedal,
        origin=Origin(xyz=(-0.059, 0.0, 0.172)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=35.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bb_shell = object_model.get_part("bb_shell")
    drive_side = object_model.get_part("drive_side")
    left_crank = object_model.get_part("left_crank")
    right_pedal = object_model.get_part("right_pedal")
    left_pedal = object_model.get_part("left_pedal")

    bb_spin = object_model.get_articulation("bb_spin")
    right_pedal_spin = object_model.get_articulation("right_pedal_spin")
    left_pedal_spin = object_model.get_articulation("left_pedal_spin")

    drive_side.get_visual("chainring")
    drive_side.get_visual("power_meter_pod")
    drive_side.get_visual("right_pedal_boss")
    left_crank.get_visual("left_pedal_boss")
    bb_shell.get_visual("bb_shell")

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
        "spindle_axis_aligned_with_bottom_bracket",
        bb_spin.axis == (1.0, 0.0, 0.0),
        f"Unexpected spindle axis: {bb_spin.axis}",
    )
    ctx.check(
        "pedal_axes_parallel_to_spindle",
        right_pedal_spin.axis == (1.0, 0.0, 0.0) and left_pedal_spin.axis == (1.0, 0.0, 0.0),
        f"Pedal axes were right={right_pedal_spin.axis}, left={left_pedal_spin.axis}",
    )

    ctx.expect_contact(
        drive_side,
        bb_shell,
        contact_tol=1e-4,
        name="spindle_supported_by_shell_bearings",
    )
    ctx.expect_overlap(
        drive_side,
        bb_shell,
        axes="yz",
        min_overlap=0.023,
        elem_a="spindle",
        elem_b="bb_shell",
        name="spindle_runs_through_shell",
    )
    ctx.expect_contact(
        left_crank,
        drive_side,
        contact_tol=1e-4,
        elem_a="left_spindle_socket",
        elem_b="spindle",
        name="left_crank_clamps_to_spindle",
    )
    ctx.expect_contact(
        right_pedal,
        drive_side,
        contact_tol=1e-4,
        elem_a="pedal_hub",
        elem_b="right_pedal_boss",
        name="right_pedal_spins_on_stub_axle",
    )
    ctx.expect_contact(
        left_pedal,
        left_crank,
        contact_tol=1e-4,
        elem_a="pedal_hub",
        elem_b="left_pedal_boss",
        name="left_pedal_spins_on_stub_axle",
    )
    ctx.expect_gap(
        drive_side,
        bb_shell,
        axis="x",
        positive_elem="chainring",
        negative_elem="bb_shell",
        min_gap=0.035,
        name="chainring_sits_outboard_of_shell",
    )

    with ctx.pose({bb_spin: pi / 2.0}):
        right_pos = ctx.part_world_position(right_pedal)
        left_pos = ctx.part_world_position(left_pedal)
        ctx.check(
            "quarter_turn_swings_pedals_fore_and_aft",
            right_pos is not None
            and left_pos is not None
            and right_pos[1] > 0.14
            and left_pos[1] < -0.14,
            f"Quarter turn positions were right={right_pos}, left={left_pos}",
        )
        ctx.expect_contact(
            drive_side,
            bb_shell,
            contact_tol=1e-4,
            name="spindle_remains_supported_at_quarter_turn",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
