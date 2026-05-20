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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    section_loft,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _merge_geometries(geometries):
    merged = geometries[0].copy()
    for geometry in geometries[1:]:
        merged.merge(geometry)
    return merged


def _ring_x(radius: float, tube: float, x_pos: float):
    return TorusGeometry(
        radius=radius,
        tube=tube,
        radial_segments=18,
        tubular_segments=56,
    ).rotate_y(math.pi / 2.0).translate(x_pos, 0.0, 0.0)


def _rod(points, *, radius: float, samples_per_segment: int = 8):
    return tube_from_spline_points(
        points,
        radius=radius,
        samples_per_segment=samples_per_segment,
        radial_segments=12,
        cap_ends=True,
        up_hint=(0.0, 0.0, 1.0),
    )


def _polar_on_x(angle: float, x_pos: float, radius: float) -> tuple[float, float, float]:
    return (x_pos, radius * math.cos(angle), radius * math.sin(angle))


def _build_guard_mesh():
    rear_x = 0.0
    throat_x = 0.065
    mid_x = 0.135
    front_x = 0.215

    rear_r = 0.186
    throat_r = 0.208
    mid_r = 0.224
    front_r = 0.236

    geometries = [
        _ring_x(rear_r, 0.008, rear_x),
        _ring_x(throat_r, 0.007, throat_x),
        _ring_x(mid_r, 0.007, mid_x),
        _ring_x(front_r, 0.008, front_x),
        _ring_x(0.160, 0.0055, front_x),
        _ring_x(0.095, 0.005, front_x),
        CylinderGeometry(radius=0.028, height=0.012, radial_segments=20)
        .rotate_y(math.pi / 2.0)
        .translate(front_x, 0.0, 0.0),
    ]

    for index in range(8):
        angle = index * math.tau / 8.0
        geometries.append(
            _rod(
                [
                    (front_x, 0.0, 0.0),
                    _polar_on_x(angle, front_x, front_r - 0.003),
                ],
                radius=0.004,
                samples_per_segment=2,
            )
        )
        geometries.append(
            _rod(
                [
                    _polar_on_x(angle, rear_x, rear_r - 0.006),
                    _polar_on_x(angle, throat_x, throat_r - 0.007),
                    _polar_on_x(angle, mid_x, mid_r - 0.007),
                    _polar_on_x(angle, front_x, front_r - 0.007),
                ],
                radius=0.0045,
                samples_per_segment=8,
            )
        )

    return _merge_geometries(geometries)


def _build_rotor_blades_mesh():
    blade = section_loft(
        [
            [
                (0.018, 0.046, -0.008),
                (0.052, 0.046, -0.001),
                (0.040, 0.046, 0.010),
                (0.020, 0.046, 0.003),
            ],
            [
                (0.018, 0.108, -0.012),
                (0.046, 0.108, 0.004),
                (0.030, 0.108, 0.015),
                (0.019, 0.108, 0.002),
            ],
            [
                (0.024, 0.156, -0.010),
                (0.036, 0.156, 0.010),
                (0.026, 0.156, 0.015),
                (0.020, 0.156, 0.004),
            ],
        ]
    )

    geometries = []
    for index in range(5):
        geometries.append(blade.copy().rotate_x(index * math.tau / 5.0))
    return _merge_geometries(geometries)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilting_service_fan")

    frame_paint = model.material("frame_paint", rgba=(0.24, 0.26, 0.24, 1.0))
    housing_paint = model.material("housing_paint", rgba=(0.43, 0.45, 0.47, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.19, 0.20, 0.22, 1.0))
    blade_metal = model.material("blade_metal", rgba=(0.76, 0.78, 0.80, 1.0))
    zinc = model.material("zinc", rgba=(0.62, 0.64, 0.67, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))

    base_frame = model.part("base_frame")
    base_frame.visual(
        Box((0.56, 0.06, 0.04)),
        origin=Origin(xyz=(0.0, 0.150, 0.020)),
        material=frame_paint,
        name="left_skid",
    )
    base_frame.visual(
        Box((0.56, 0.06, 0.04)),
        origin=Origin(xyz=(0.0, -0.150, 0.020)),
        material=frame_paint,
        name="right_skid",
    )
    base_frame.visual(
        Box((0.08, 0.36, 0.05)),
        origin=Origin(xyz=(0.180, 0.0, 0.050)),
        material=frame_paint,
        name="front_crossmember",
    )
    base_frame.visual(
        Box((0.08, 0.36, 0.05)),
        origin=Origin(xyz=(-0.200, 0.0, 0.050)),
        material=frame_paint,
        name="rear_crossmember",
    )
    base_frame.visual(
        Box((0.18, 0.24, 0.06)),
        origin=Origin(xyz=(-0.030, 0.0, 0.090)),
        material=frame_paint,
        name="deck_plate",
    )
    base_frame.visual(
        Box((0.10, 0.10, 0.24)),
        origin=Origin(xyz=(-0.085, 0.0, 0.240)),
        material=frame_paint,
        name="front_mast",
    )
    base_frame.visual(
        Box((0.10, 0.16, 0.22)),
        origin=Origin(xyz=(-0.145, 0.0, 0.250)),
        material=frame_paint,
        name="rear_mast",
    )
    base_frame.visual(
        Cylinder(radius=0.024, length=0.39),
        origin=Origin(xyz=(-0.145, 0.0, 0.298), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_paint,
        name="yoke_cross_tube",
    )
    base_frame.visual(
        Box((0.17, 0.03, 0.30)),
        origin=Origin(xyz=(-0.072, 0.195, 0.470)),
        material=frame_paint,
        name="left_yoke_arm",
    )
    base_frame.visual(
        Box((0.17, 0.03, 0.30)),
        origin=Origin(xyz=(-0.072, -0.195, 0.470)),
        material=frame_paint,
        name="right_yoke_arm",
    )
    base_frame.visual(
        Cylinder(radius=0.045, length=0.035),
        origin=Origin(xyz=(0.0, 0.1775, 0.460), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_dark,
        name="left_pivot_collar",
    )
    base_frame.visual(
        Cylinder(radius=0.045, length=0.035),
        origin=Origin(xyz=(0.0, -0.1775, 0.460), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_dark,
        name="right_pivot_collar",
    )
    base_frame.visual(
        Cylinder(radius=0.030, length=0.025),
        origin=Origin(xyz=(0.0, 0.2075, 0.460), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="left_tilt_handwheel",
    )
    base_frame.visual(
        Cylinder(radius=0.030, length=0.025),
        origin=Origin(xyz=(0.0, -0.2075, 0.460), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="right_tilt_handwheel",
    )
    for x_pos in (-0.180, 0.180):
        base_frame.visual(
            Box((0.10, 0.06, 0.012)),
            origin=Origin(xyz=(x_pos, 0.150, 0.006)),
            material=rubber,
            name=f"left_foot_pad_{'front' if x_pos > 0 else 'rear'}",
        )
        base_frame.visual(
            Box((0.10, 0.06, 0.012)),
            origin=Origin(xyz=(x_pos, -0.150, 0.006)),
            material=rubber,
            name=f"right_foot_pad_{'front' if x_pos > 0 else 'rear'}",
        )
    base_frame.visual(
        Box((0.10, 0.03, 0.06)),
        origin=Origin(xyz=(-0.060, 0.135, 0.070)),
        material=frame_paint,
        name="left_skid_riser",
    )
    base_frame.visual(
        Box((0.10, 0.03, 0.06)),
        origin=Origin(xyz=(-0.060, -0.135, 0.070)),
        material=frame_paint,
        name="right_skid_riser",
    )
    base_frame.inertial = Inertial.from_geometry(
        Box((0.60, 0.42, 0.62)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.310)),
    )

    head_shell = model.part("head_shell")
    head_shell.visual(
        Cylinder(radius=0.088, length=0.200),
        origin=Origin(xyz=(-0.110, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=housing_paint,
        name="motor_barrel",
    )
    head_shell.visual(
        Cylinder(radius=0.068, length=0.032),
        origin=Origin(xyz=(-0.226, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=housing_paint,
        name="rear_service_cap",
    )
    head_shell.visual(
        Cylinder(radius=0.050, length=0.040),
        origin=Origin(xyz=(0.020, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_dark,
        name="bearing_nose",
    )
    head_shell.visual(
        Cylinder(radius=0.024, length=0.030),
        origin=Origin(xyz=(0.055, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_dark,
        name="shaft_tube",
    )
    head_shell.visual(
        Box((0.120, 0.280, 0.180)),
        origin=Origin(xyz=(-0.010, 0.0, 0.0)),
        material=steel_dark,
        name="front_carrier",
    )
    for name, y_pos, z_pos in (
        ("guard_lug_ul", 0.128, 0.128),
        ("guard_lug_ur", -0.128, 0.128),
        ("guard_lug_ll", 0.128, -0.128),
        ("guard_lug_lr", -0.128, -0.128),
    ):
        head_shell.visual(
            Box((0.027122, 0.028, 0.028)),
            origin=Origin(xyz=(0.053561, y_pos, z_pos)),
            material=steel_dark,
            name=name,
        )
    head_shell.visual(
        Box((0.140, 0.020, 0.110)),
        origin=Origin(xyz=(-0.050, 0.118, 0.0)),
        material=steel_dark,
        name="left_support_plate",
    )
    head_shell.visual(
        Box((0.140, 0.020, 0.110)),
        origin=Origin(xyz=(-0.050, -0.118, 0.0)),
        material=steel_dark,
        name="right_support_plate",
    )
    head_shell.visual(
        Cylinder(radius=0.036, length=0.060),
        origin=Origin(xyz=(-0.008, 0.130, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_dark,
        name="left_pivot_boss",
    )
    head_shell.visual(
        Cylinder(radius=0.036, length=0.060),
        origin=Origin(xyz=(-0.008, -0.130, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_dark,
        name="right_pivot_boss",
    )
    head_shell.visual(
        Box((0.080, 0.080, 0.035)),
        origin=Origin(xyz=(-0.118, 0.0, 0.086)),
        material=housing_paint,
        name="top_access_hatch",
    )
    head_shell.visual(
        _save_mesh(
            "head_guard_strut_ul",
            _rod([(0.028, 0.078, 0.078), (0.066, 0.128, 0.128)], radius=0.009, samples_per_segment=2),
        ),
        material=steel_dark,
        name="guard_strut_ul",
    )
    head_shell.visual(
        _save_mesh(
            "head_guard_strut_ur",
            _rod([(0.028, -0.078, 0.078), (0.066, -0.128, 0.128)], radius=0.009, samples_per_segment=2),
        ),
        material=steel_dark,
        name="guard_strut_ur",
    )
    head_shell.visual(
        _save_mesh(
            "head_guard_strut_ll",
            _rod([(0.028, 0.078, -0.078), (0.066, 0.128, -0.128)], radius=0.009, samples_per_segment=2),
        ),
        material=steel_dark,
        name="guard_strut_ll",
    )
    head_shell.visual(
        _save_mesh(
            "head_guard_strut_lr",
            _rod([(0.028, -0.078, -0.078), (0.066, -0.128, -0.128)], radius=0.009, samples_per_segment=2),
        ),
        material=steel_dark,
        name="guard_strut_lr",
    )
    head_shell.visual(
        Box((0.060, 0.034, 0.050)),
        origin=Origin(xyz=(-0.108, 0.0, 0.082)),
        material=zinc,
        name="junction_box",
    )
    for index, (y_pos, z_pos) in enumerate(
        ((0.030, 0.030), (0.030, -0.030), (-0.030, 0.030), (-0.030, -0.030))
    ):
        head_shell.visual(
            Cylinder(radius=0.006, length=0.012),
            origin=Origin(
                xyz=(-0.136, y_pos, z_pos),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=zinc,
            name=f"service_bolt_{index}",
        )
    head_shell.inertial = Inertial.from_geometry(
        Box((0.38, 0.42, 0.32)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    guard_assembly = model.part("guard_assembly")
    guard_assembly.visual(
        _save_mesh("fan_guard_basket", _build_guard_mesh()),
        material=steel_dark,
        name="guard_basket",
    )
    guard_assembly.inertial = Inertial.from_geometry(
        Cylinder(radius=0.24, length=0.27),
        mass=1.8,
        origin=Origin(xyz=(0.050, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(0.003, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_dark,
        name="shaft_stub",
    )
    rotor.visual(
        Cylinder(radius=0.046, length=0.036),
        origin=Origin(xyz=(0.032, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_dark,
        name="hub_barrel",
    )
    rotor.visual(
        Cylinder(radius=0.032, length=0.014),
        origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_dark,
        name="rear_flange",
    )
    rotor.visual(
        Cylinder(radius=0.040, length=0.028),
        origin=Origin(xyz=(0.060, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blade_metal,
        name="nose_shell",
    )
    rotor.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(xyz=(0.083, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blade_metal,
        name="nose_cap",
    )
    rotor.visual(
        _save_mesh("fan_rotor_mesh", _build_rotor_blades_mesh()),
        material=blade_metal,
        name="blade_set",
    )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.17, length=0.11),
        mass=1.4,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "base_to_head_tilt",
        ArticulationType.REVOLUTE,
        parent=base_frame,
        child=head_shell,
        origin=Origin(xyz=(0.0, 0.0, 0.460)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.0,
            lower=-0.35,
            upper=0.52,
        ),
    )
    model.articulation(
        "head_to_guard_mount",
        ArticulationType.FIXED,
        parent=head_shell,
        child=guard_assembly,
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
    )
    model.articulation(
        "head_to_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=head_shell,
        child=rotor,
        origin=Origin(xyz=(0.072, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=22.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    required_parts = ("base_frame", "head_shell", "guard_assembly", "rotor")
    resolved_parts = {}
    missing_parts = []
    for part_name in required_parts:
        try:
            resolved_parts[part_name] = object_model.get_part(part_name)
        except Exception:
            missing_parts.append(part_name)
    ctx.check(
        "core_parts_present",
        not missing_parts,
        f"missing parts: {', '.join(missing_parts)}" if missing_parts else "",
    )
    if missing_parts:
        return ctx.report()

    base_frame = resolved_parts["base_frame"]
    head_shell = resolved_parts["head_shell"]
    guard_assembly = resolved_parts["guard_assembly"]
    rotor = resolved_parts["rotor"]

    tilt = object_model.get_articulation("base_to_head_tilt")
    spin = object_model.get_articulation("head_to_rotor_spin")

    ctx.check(
        "tilt_joint_is_side_pivot",
        tilt.articulation_type == ArticulationType.REVOLUTE and tuple(tilt.axis) == (0.0, -1.0, 0.0),
        f"tilt joint should be revolute about -Y, got type={tilt.articulation_type} axis={tilt.axis}",
    )
    ctx.check(
        "rotor_spin_joint_is_explicit_hub_axis",
        spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (1.0, 0.0, 0.0),
        f"spin joint should be continuous about +X, got type={spin.articulation_type} axis={spin.axis}",
    )

    ctx.expect_contact(base_frame, head_shell, name="tilt_pivot_bosses_contact_yoke")
    ctx.expect_contact(head_shell, guard_assembly, name="guard_mounts_contact_head")
    ctx.expect_contact(head_shell, rotor, name="hub_bearing_contact")
    ctx.expect_within(rotor, guard_assembly, axes="yz", margin=0.03, name="rotor_stays_within_guard")
    ctx.expect_overlap(rotor, guard_assembly, axes="yz", min_overlap=0.29, name="rotor_and_guard_are_concentric")

    upper_tilt = 0.52
    if tilt.motion_limits is not None and tilt.motion_limits.upper is not None:
        upper_tilt = tilt.motion_limits.upper

    rest_rotor_z = None
    raised_rotor_z = None
    with ctx.pose({tilt: 0.0}):
        rest_position = ctx.part_world_position(rotor)
        if rest_position is not None:
            rest_rotor_z = rest_position[2]
    with ctx.pose({tilt: upper_tilt}):
        raised_position = ctx.part_world_position(rotor)
        if raised_position is not None:
            raised_rotor_z = raised_position[2]
        ctx.expect_contact(base_frame, head_shell, name="tilt_pivots_remain_seated_when_raised")
    ctx.check(
        "positive_tilt_raises_rotor",
        rest_rotor_z is not None
        and raised_rotor_z is not None
        and raised_rotor_z > rest_rotor_z + 0.02,
        f"expected rotor z to increase by > 0.02 m, got rest={rest_rotor_z}, raised={raised_rotor_z}",
    )

    with ctx.pose({spin: math.pi / 2.0}):
        ctx.expect_contact(head_shell, rotor, name="rotor_hub_stays_on_bearing_when_spun")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
