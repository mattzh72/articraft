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
    ExtrudeGeometry,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _merge_geometries(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = geometries[0].copy()
    for geometry in geometries[1:]:
        merged.merge(geometry)
    return merged


def _circle_points(
    radius: float,
    x_pos: float,
    *,
    segments: int = 36,
    phase: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (
            x_pos,
            radius * math.cos(phase + (index * math.tau / segments)),
            radius * math.sin(phase + (index * math.tau / segments)),
        )
        for index in range(segments)
    ]


def _wire_ring(radius: float, x_pos: float, wire_radius: float, *, segments: int = 36) -> MeshGeometry:
    return tube_from_spline_points(
        _circle_points(radius, x_pos, segments=segments),
        radius=wire_radius,
        samples_per_segment=2,
        closed_spline=True,
        radial_segments=14,
        cap_ends=False,
    )


def _radial_wire(
    x_inner: float,
    r_inner: float,
    x_outer: float,
    r_outer: float,
    angle: float,
    wire_radius: float,
) -> MeshGeometry:
    c = math.cos(angle)
    s = math.sin(angle)
    mid_radius = 0.5 * (r_inner + r_outer)
    mid_x = 0.5 * (x_inner + x_outer)
    return tube_from_spline_points(
        [
            (x_inner, r_inner * c, r_inner * s),
            (mid_x, mid_radius * c, mid_radius * s),
            (x_outer, r_outer * c, r_outer * s),
        ],
        radius=wire_radius,
        samples_per_segment=6,
        radial_segments=12,
        cap_ends=True,
    )


def _blade_loop(
    radius: float,
    *,
    x_center: float,
    y_center: float,
    chord: float,
    thickness: float,
    pitch_deg: float,
) -> list[tuple[float, float, float]]:
    base_profile = [
        (-0.52 * chord, -0.42 * thickness),
        (0.10 * chord, -0.60 * thickness),
        (0.50 * chord, -0.12 * thickness),
        (0.42 * chord, 0.40 * thickness),
        (-0.08 * chord, 0.58 * thickness),
        (-0.48 * chord, 0.18 * thickness),
    ]
    angle = math.radians(pitch_deg)
    c = math.cos(angle)
    s = math.sin(angle)
    loop = []
    for x_local, y_local in base_profile:
        x_rot = (c * x_local) - (s * y_local) + x_center
        y_rot = (s * x_local) + (c * y_local) + y_center
        loop.append((x_rot, y_rot, radius))
    return loop


def _build_base_shell_mesh() -> MeshGeometry:
    return ExtrudeGeometry(rounded_rect_profile(0.25, 0.19, 0.035), 0.028)


def _build_head_shell_mesh() -> MeshGeometry:
    profile = [
        (0.006, -0.048),
        (0.034, -0.044),
        (0.046, -0.024),
        (0.049, -0.006),
        (0.044, 0.004),
        (0.034, 0.012),
        (0.022, 0.010),
        (0.016, -0.002),
    ]
    return LatheGeometry(profile, segments=56).rotate_y(math.pi / 2.0)


def _build_spinner_mesh() -> MeshGeometry:
    profile = [
        (0.0, 0.034),
        (0.010, 0.031),
        (0.020, 0.025),
        (0.024, 0.020),
        (0.0, 0.020),
    ]
    return LatheGeometry(profile, segments=40).rotate_y(math.pi / 2.0)


def _build_blade_mesh() -> MeshGeometry:
    blade = repair_loft(
        section_loft(
            [
                _blade_loop(0.034, x_center=0.010, y_center=-0.002, chord=0.022, thickness=0.0038, pitch_deg=56.0),
                _blade_loop(0.060, x_center=0.014, y_center=0.002, chord=0.036, thickness=0.0035, pitch_deg=38.0),
                _blade_loop(0.086, x_center=0.017, y_center=0.005, chord=0.026, thickness=0.0026, pitch_deg=24.0),
                _blade_loop(0.108, x_center=0.019, y_center=0.008, chord=0.016, thickness=0.0018, pitch_deg=12.0),
            ]
        ),
        repair="mesh",
    )
    pattern = _merge_geometries(
        [
            blade.copy().rotate_x(0.0),
            blade.copy().rotate_x(2.0 * math.pi / 3.0),
            blade.copy().rotate_x(4.0 * math.pi / 3.0),
        ]
    )
    return pattern


def _build_guard_cage_mesh() -> MeshGeometry:
    wire_radius = 0.0032
    geometries = [
        _wire_ring(0.032, 0.058, wire_radius, segments=30),
        _wire_ring(0.120, 0.080, wire_radius, segments=38),
        _wire_ring(0.146, 0.106, wire_radius, segments=40),
        _wire_ring(0.156, 0.132, wire_radius, segments=42),
        _wire_ring(0.160, 0.150, wire_radius, segments=42),
    ]
    for index in range(6):
        angle = index * math.tau / 6.0
        geometries.append(_radial_wire(0.058, 0.032, 0.080, 0.120, angle, 0.0030))
        geometries.append(_radial_wire(0.080, 0.120, 0.106, 0.146, angle, 0.0030))
        geometries.append(_radial_wire(0.106, 0.146, 0.132, 0.156, angle, 0.0030))
        geometries.append(_radial_wire(0.132, 0.156, 0.150, 0.160, angle, 0.0028))
    return _merge_geometries(geometries)


def _build_front_guard_body_mesh() -> MeshGeometry:
    wire_radius = 0.0035
    geometries = [
        _wire_ring(0.154, 0.050, wire_radius),
        _wire_ring(0.118, 0.064, wire_radius),
        _wire_ring(0.080, 0.076, wire_radius),
        _wire_ring(0.040, 0.086, wire_radius),
    ]
    for index in range(4):
        geometries.append(
            _radial_wire(
                0.086,
                0.022,
                0.050,
                0.151,
                (math.pi / 4.0) + (index * math.tau / 4.0),
                0.0030,
            )
        )
    for index in range(3):
        angle = index * math.tau / 3.0
        c = math.cos(angle)
        s = math.sin(angle)
        geometries.append(
            tube_from_spline_points(
                [
                    (0.000, 0.151 * c, 0.151 * s),
                    (0.025, 0.151 * c, 0.151 * s),
                    (0.050, 0.151 * c, 0.151 * s),
                ],
                radius=0.0030,
                samples_per_segment=2,
                radial_segments=12,
                cap_ends=True,
            )
        )
    return _merge_geometries(geometries)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilting_fan")

    dark_plastic = model.material("dark_plastic", rgba=(0.16, 0.17, 0.18, 1.0))
    medium_plastic = model.material("medium_plastic", rgba=(0.24, 0.25, 0.27, 1.0))
    steel = model.material("steel", rgba=(0.75, 0.77, 0.80, 1.0))
    blade_gray = model.material("blade_gray", rgba=(0.70, 0.72, 0.74, 0.92))

    base = model.part("base")
    base.visual(
        mesh_from_geometry(_build_base_shell_mesh(), "fan_base_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=dark_plastic,
        name="base_shell",
    )
    base.visual(
        Cylinder(radius=0.020, length=0.190),
        origin=Origin(xyz=(-0.035, 0.0, 0.123)),
        material=dark_plastic,
        name="stem",
    )
    base.visual(
        Box((0.040, 0.340, 0.018)),
        origin=Origin(xyz=(-0.052, 0.0, 0.227)),
        material=dark_plastic,
        name="yoke_bridge",
    )
    base.visual(
        Box((0.050, 0.014, 0.084)),
        origin=Origin(xyz=(-0.027, 0.173, 0.260)),
        material=dark_plastic,
        name="left_arm",
    )
    base.visual(
        Box((0.050, 0.014, 0.084)),
        origin=Origin(xyz=(-0.027, -0.173, 0.260)),
        material=dark_plastic,
        name="right_arm",
    )
    base.visual(
        Cylinder(radius=0.025, length=0.012),
        origin=Origin(xyz=(-0.008, 0.166, 0.260), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=medium_plastic,
        name="left_yoke_pad",
    )
    base.visual(
        Cylinder(radius=0.025, length=0.012),
        origin=Origin(xyz=(-0.008, -0.166, 0.260), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=medium_plastic,
        name="right_yoke_pad",
    )

    head = model.part("head")
    head.visual(
        mesh_from_geometry(_build_head_shell_mesh(), "fan_head_shell"),
        origin=Origin(xyz=(0.048, 0.0, 0.0)),
        material=medium_plastic,
        name="housing_shell",
    )
    head.visual(
        Box((0.058, 0.106, 0.050)),
        origin=Origin(xyz=(0.021, 0.101, 0.0)),
        material=medium_plastic,
        name="left_pivot_bracket",
    )
    head.visual(
        Box((0.058, 0.106, 0.050)),
        origin=Origin(xyz=(0.021, -0.101, 0.0)),
        material=medium_plastic,
        name="right_pivot_bracket",
    )
    head.visual(
        Cylinder(radius=0.018, length=0.022),
        origin=Origin(xyz=(0.0, 0.149, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=medium_plastic,
        name="left_trunnion",
    )
    head.visual(
        Cylinder(radius=0.018, length=0.022),
        origin=Origin(xyz=(0.0, -0.149, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=medium_plastic,
        name="right_trunnion",
    )
    head.visual(
        Cylinder(radius=0.032, length=0.016),
        origin=Origin(xyz=(0.052, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=medium_plastic,
        name="nose_flange",
    )
    head.visual(
        Cylinder(radius=0.008, length=0.014),
        origin=Origin(xyz=(0.067, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="shaft_stub",
    )
    head.visual(
        mesh_from_geometry(_build_guard_cage_mesh(), "fan_guard_cage"),
        material=steel,
        name="guard_cage",
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.034, length=0.018),
        origin=Origin(xyz=(0.009, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blade_gray,
        name="hub_shell",
    )
    rotor.visual(
        mesh_from_geometry(_build_blade_mesh(), "fan_blade_set"),
        material=blade_gray,
        name="blade_set",
    )

    model.articulation(
        "base_to_head_tilt",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.008, 0.0, 0.260)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=math.radians(-25.0),
            upper=math.radians(40.0),
        ),
    )
    model.articulation(
        "head_to_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=rotor,
        origin=Origin(xyz=(0.074, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=25.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    head = object_model.get_part("head")
    rotor = object_model.get_part("rotor")
    tilt = object_model.get_articulation("base_to_head_tilt")
    spin = object_model.get_articulation("head_to_rotor_spin")

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
        min_gap=0.24,
        max_gap=0.28,
        name="pivot height stays at realistic desk-fan scale",
    )
    ctx.expect_contact(
        head,
        base,
        elem_a="left_trunnion",
        elem_b="left_yoke_pad",
        name="left tilt pivot is physically supported",
    )
    ctx.expect_contact(
        head,
        base,
        elem_a="right_trunnion",
        elem_b="right_yoke_pad",
        name="right tilt pivot is physically supported",
    )
    ctx.expect_contact(
        head,
        head,
        elem_a="guard_cage",
        elem_b="nose_flange",
        name="guard cage mounts to the motor flange",
    )
    ctx.expect_contact(
        rotor,
        head,
        elem_a="hub_shell",
        elem_b="shaft_stub",
        name="rotor hub seats on the motor shaft",
    )
    ctx.expect_within(
        rotor,
        head,
        axes="yz",
        margin=0.0,
        outer_elem="guard_cage",
        name="rotor stays inside the guard cage footprint",
    )

    tilt_ok = (
        tilt.articulation_type == ArticulationType.REVOLUTE
        and tilt.motion_limits is not None
        and abs(tilt.axis[0]) < 1e-9
        and abs(tilt.axis[1] + 1.0) < 1e-9
        and abs(tilt.axis[2]) < 1e-9
    )
    ctx.check(
        "tilt articulation uses the side-pivot axis",
        tilt_ok,
        details=f"tilt joint type={tilt.articulation_type} axis={tilt.axis}",
    )

    spin_limits = spin.motion_limits
    spin_ok = (
        spin.articulation_type == ArticulationType.CONTINUOUS
        and spin_limits is not None
        and spin_limits.lower is None
        and spin_limits.upper is None
        and abs(spin.axis[0] - 1.0) < 1e-9
        and abs(spin.axis[1]) < 1e-9
        and abs(spin.axis[2]) < 1e-9
    )
    ctx.check(
        "rotor spin articulation is an axial continuous hub",
        spin_ok,
        details=f"spin joint type={spin.articulation_type} axis={spin.axis}",
    )

    with ctx.pose({tilt: 0.0}):
        rest_aabb = ctx.part_world_aabb(rotor)
    with ctx.pose({tilt: math.radians(30.0)}):
        raised_aabb = ctx.part_world_aabb(rotor)
        ctx.fail_if_parts_overlap_in_current_pose(name="tilted pose stays clear of self-overlap")

    if rest_aabb is None or raised_aabb is None:
        ctx.fail("positive tilt raises the fan nose", "front guard AABB was unavailable")
    else:
        rest_center_z = 0.5 * (rest_aabb[0][2] + rest_aabb[1][2])
        raised_center_z = 0.5 * (raised_aabb[0][2] + raised_aabb[1][2])
        ctx.check(
            "positive tilt raises the fan nose",
            raised_center_z > rest_center_z + 0.02,
            details=f"rest_z={rest_center_z:.4f}, raised_z={raised_center_z:.4f}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
