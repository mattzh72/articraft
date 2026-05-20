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
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, name)


def _merge_geometries(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _circle_point(x_pos: float, radius: float, angle: float) -> tuple[float, float, float]:
    return (x_pos, radius * math.cos(angle), radius * math.sin(angle))


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _support_bolt(
    radius: float = 0.0045,
    length: float = 0.014,
    *,
    axis: str = "x",
) -> tuple[Cylinder, Origin]:
    if axis == "x":
        return (
            Cylinder(radius=radius, length=length),
            Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        )
    if axis == "y":
        return (
            Cylinder(radius=radius, length=length),
            Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        )
    return (Cylinder(radius=radius, length=length), Origin())


def _build_stand_base_mesh() -> MeshGeometry:
    foot = ExtrudeGeometry(
        rounded_rect_profile(0.44, 0.30, 0.065, corner_segments=8),
        0.035,
        cap=True,
        center=True,
    )
    foot.translate(-0.055, 0.0, 0.0175)

    raised_pad = ExtrudeGeometry(
        rounded_rect_profile(0.23, 0.17, 0.038, corner_segments=8),
        0.018,
        cap=True,
        center=True,
    )
    raised_pad.translate(-0.085, 0.0, 0.044)

    toe = ExtrudeGeometry(
        rounded_rect_profile(0.18, 0.13, 0.026, corner_segments=8),
        0.014,
        cap=True,
        center=True,
    )
    toe.translate(0.080, 0.0, 0.041)
    return _merge_geometries(foot, raised_pad, toe)


def _build_head_cage_mesh() -> MeshGeometry:
    cage = MeshGeometry()
    ring_specs = (
        (-0.045, 0.205, 0.007),
        (0.040, 0.232, 0.006),
        (0.125, 0.205, 0.007),
    )
    for x_pos, radius, tube in ring_specs:
        cage.merge(
            TorusGeometry(
                radius=radius,
                tube=tube,
                radial_segments=14,
                tubular_segments=72,
            )
            .rotate_y(math.pi / 2.0)
            .translate(x_pos, 0.0, 0.0)
        )

    support_ring = (
        TorusGeometry(
            radius=0.078,
            tube=0.006,
            radial_segments=12,
            tubular_segments=48,
        )
        .rotate_y(math.pi / 2.0)
        .translate(-0.004, 0.0, 0.0)
    )
    cage.merge(support_ring)

    for index in range(16):
        angle = math.tau * index / 16.0
        cage.merge(
            tube_from_spline_points(
                [
                    _circle_point(-0.045, 0.205, angle),
                    _circle_point(0.040, 0.232, angle),
                    _circle_point(0.125, 0.205, angle),
                ],
                radius=0.0028,
                samples_per_segment=12,
                radial_segments=12,
                cap_ends=True,
            )
        )

    for index in range(6):
        angle = math.tau * index / 6.0 + (math.pi / 12.0)
        cage.merge(
            tube_from_spline_points(
                [
                    _circle_point(-0.004, 0.078, angle),
                    _circle_point(-0.020, 0.130, angle),
                    _circle_point(-0.045, 0.205, angle),
                ],
                radius=0.0042,
                samples_per_segment=10,
                radial_segments=12,
                cap_ends=True,
            )
        )

    return cage


def _build_rotor_blade_mesh() -> MeshGeometry:
    blade_profile = [
        (0.016, -0.004),
        (0.030, -0.007),
        (0.060, -0.009),
        (0.082, -0.004),
        (0.086, 0.0),
        (0.082, 0.004),
        (0.056, 0.008),
        (0.024, 0.006),
        (0.012, 0.002),
    ]
    base_blade = ExtrudeGeometry.from_z0(
        blade_profile,
        0.148,
        cap=True,
        closed=True,
    )
    base_blade.rotate_x(math.radians(-8.0))
    base_blade.translate(0.0, 0.0, 0.040)
    return base_blade


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_tilting_fan")

    cast_olive = model.material("cast_olive", rgba=(0.31, 0.35, 0.27, 1.0))
    warm_gray = model.material("warm_gray", rgba=(0.70, 0.71, 0.69, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.23, 0.25, 1.0))
    hardware = model.material("hardware", rgba=(0.60, 0.52, 0.35, 1.0))
    rotor_black = model.material("rotor_black", rgba=(0.16, 0.16, 0.17, 1.0))
    hatch_black = model.material("hatch_black", rgba=(0.10, 0.10, 0.11, 1.0))

    stand = model.part("stand")
    stand.visual(
        Box((0.44, 0.30, 0.030)),
        origin=Origin(xyz=(-0.055, 0.0, 0.015)),
        material=cast_olive,
        name="base_casting",
    )
    stand.visual(
        Box((0.22, 0.16, 0.024)),
        origin=Origin(xyz=(-0.085, 0.0, 0.042)),
        material=cast_olive,
        name="base_pad",
    )
    stand.visual(
        Box((0.18, 0.13, 0.020)),
        origin=Origin(xyz=(0.080, 0.0, 0.035)),
        material=cast_olive,
        name="toe_pad",
    )
    stand.visual(
        Box((0.115, 0.100, 0.018)),
        origin=Origin(xyz=(-0.005, 0.0, 0.033)),
        material=cast_olive,
        name="toe_bridge",
    )
    stand.visual(
        Cylinder(radius=0.042, length=0.240),
        origin=Origin(xyz=(-0.090, 0.0, 0.162)),
        material=cast_olive,
        name="stand_column",
    )
    stand.visual(
        Box((0.120, 0.150, 0.050)),
        origin=Origin(xyz=(-0.075, 0.0, 0.307)),
        material=cast_olive,
        name="column_adapter",
    )
    stand.visual(
        Cylinder(radius=0.018, length=0.220),
        origin=Origin(xyz=(-0.032, 0.0, 0.332), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="cross_tube",
    )
    stand.visual(
        Box((0.060, 0.170, 0.016)),
        origin=Origin(xyz=(-0.002, 0.0, 0.346)),
        material=warm_gray,
        name="yoke_adapter_plate",
    )
    stand.visual(
        Box((0.055, 0.024, 0.120)),
        origin=Origin(xyz=(0.016, 0.103, 0.436)),
        material=warm_gray,
        name="left_yoke_arm",
    )
    stand.visual(
        Box((0.055, 0.024, 0.120)),
        origin=Origin(xyz=(0.016, -0.103, 0.436)),
        material=warm_gray,
        name="right_yoke_arm",
    )
    stand.visual(
        _save_mesh(
            "left_upper_brace",
            tube_from_spline_points(
                [(-0.050, 0.118, 0.338), (-0.015, 0.114, 0.392), (0.010, 0.108, 0.432)],
                radius=0.012,
                samples_per_segment=10,
                radial_segments=14,
            ),
        ),
        material=dark_steel,
        name="left_upper_brace",
    )
    stand.visual(
        _save_mesh(
            "right_upper_brace",
            tube_from_spline_points(
                [(-0.050, -0.118, 0.338), (-0.015, -0.114, 0.392), (0.010, -0.108, 0.432)],
                radius=0.012,
                samples_per_segment=10,
                radial_segments=14,
            ),
        ),
        material=dark_steel,
        name="right_upper_brace",
    )
    stand.visual(
        _save_mesh(
            "left_lower_gusset",
            tube_from_spline_points(
                [(-0.178, 0.060, 0.028), (-0.126, 0.040, 0.132), (-0.090, 0.020, 0.240)],
                radius=0.011,
                samples_per_segment=10,
                radial_segments=12,
            ),
        ),
        material=dark_steel,
        name="left_lower_gusset",
    )
    stand.visual(
        _save_mesh(
            "right_lower_gusset",
            tube_from_spline_points(
                [(-0.178, -0.060, 0.028), (-0.126, -0.040, 0.132), (-0.090, -0.020, 0.240)],
                radius=0.011,
                samples_per_segment=10,
                radial_segments=12,
            ),
        ),
        material=dark_steel,
        name="right_lower_gusset",
    )
    stand.visual(
        Cylinder(radius=0.030, length=0.016),
        origin=Origin(xyz=(0.016, 0.123, 0.436), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="left_tilt_knob",
    )
    stand.visual(
        Cylinder(radius=0.030, length=0.016),
        origin=Origin(xyz=(0.016, -0.123, 0.436), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="right_tilt_knob",
    )
    stand.visual(
        Box((0.012, 0.085, 0.100)),
        origin=Origin(xyz=(-0.128, 0.0, 0.205)),
        material=hatch_black,
        name="stand_service_hatch",
    )
    for idx, (y_pos, z_pos) in enumerate(((0.028, 0.244), (-0.028, 0.244), (0.028, 0.166), (-0.028, 0.166))):
        stand.visual(
            Cylinder(radius=0.0045, length=0.014),
            origin=Origin(xyz=(-0.129, y_pos, z_pos), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hardware,
            name=f"stand_hatch_bolt_{idx:02d}",
        )
    for idx, (y_pos, z_pos) in enumerate(((0.055, 0.320), (-0.055, 0.320), (0.055, 0.294), (-0.055, 0.294))):
        stand.visual(
            Cylinder(radius=0.0045, length=0.014),
            origin=Origin(xyz=(-0.018, y_pos, z_pos), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hardware,
            name=f"adapter_bolt_{idx:02d}",
        )
    stand.inertial = Inertial.from_geometry(
        Box((0.46, 0.30, 0.50)),
        mass=8.0,
        origin=Origin(xyz=(-0.055, 0.0, 0.250)),
    )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.022, length=0.182),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="trunnion_bar",
    )
    head.visual(
        Box((0.064, 0.094, 0.076)),
        origin=Origin(xyz=(0.024, 0.0, 0.0)),
        material=warm_gray,
        name="rear_bridge",
    )
    head.visual(
        _save_mesh(
            "motor_shell",
            LatheGeometry(
                [
                    (0.0, -0.145),
                    (0.030, -0.142),
                    (0.056, -0.125),
                    (0.072, -0.076),
                    (0.078, -0.010),
                    (0.074, 0.010),
                    (0.060, 0.026),
                    (0.0, 0.032),
                ],
                segments=64,
            ).rotate_y(math.pi / 2.0),
        ),
        origin=Origin(xyz=(0.110, 0.0, 0.0)),
        material=warm_gray,
        name="motor_shell",
    )
    head.visual(
        Cylinder(radius=0.028, length=0.056),
        origin=Origin(xyz=(0.117, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="nose_bearing",
    )
    head.visual(
        Cylinder(radius=0.074, length=0.014),
        origin=Origin(xyz=(0.066, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="guard_support_collar",
    )
    head.visual(
        _save_mesh(
            "guard_rear_ring",
            TorusGeometry(radius=0.206, tube=0.007, radial_segments=14, tubular_segments=72),
        ),
        origin=Origin(xyz=(0.110, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_gray,
        name="guard_rear_ring",
    )
    head.visual(
        _save_mesh(
            "guard_mid_ring",
            TorusGeometry(radius=0.232, tube=0.006, radial_segments=14, tubular_segments=72),
        ),
        origin=Origin(xyz=(0.195, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_gray,
        name="guard_cage",
    )
    head.visual(
        _save_mesh(
            "guard_front_ring",
            TorusGeometry(radius=0.206, tube=0.007, radial_segments=14, tubular_segments=72),
        ),
        origin=Origin(xyz=(0.280, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_gray,
        name="guard_front_ring",
    )
    for index in range(12):
        angle = math.tau * index / 12.0
        rear_point = _circle_point(0.110, 0.206, angle)
        mid_point = _circle_point(0.195, 0.232, angle)
        front_point = _circle_point(0.280, 0.206, angle)
        head.visual(
            Cylinder(radius=0.0036, length=_distance(rear_point, mid_point)),
            origin=Origin(xyz=_midpoint(rear_point, mid_point), rpy=_rpy_for_cylinder(rear_point, mid_point)),
            material=warm_gray,
            name=f"guard_wire_rear_{index:02d}",
        )
        head.visual(
            Cylinder(radius=0.0036, length=_distance(mid_point, front_point)),
            origin=Origin(xyz=_midpoint(mid_point, front_point), rpy=_rpy_for_cylinder(mid_point, front_point)),
            material=warm_gray,
            name=f"guard_wire_front_{index:02d}",
        )
    for index in range(6):
        angle = math.tau * index / 6.0 + math.pi / 6.0
        collar_point = _circle_point(0.066, 0.074, angle)
        rear_point = _circle_point(0.110, 0.206, angle)
        head.visual(
            Cylinder(radius=0.0045, length=_distance(collar_point, rear_point)),
            origin=Origin(xyz=_midpoint(collar_point, rear_point), rpy=_rpy_for_cylinder(collar_point, rear_point)),
            material=dark_steel,
            name=f"guard_support_strut_{index:02d}",
        )
    head.visual(
        Box((0.060, 0.032, 0.084)),
        origin=Origin(xyz=(0.016, 0.046, 0.0)),
        material=warm_gray,
        name="left_adapter_plate",
    )
    head.visual(
        Box((0.060, 0.032, 0.084)),
        origin=Origin(xyz=(0.016, -0.046, 0.0)),
        material=warm_gray,
        name="right_adapter_plate",
    )
    head.visual(
        Box((0.062, 0.086, 0.012)),
        origin=Origin(xyz=(0.080, 0.0, 0.074)),
        material=hatch_black,
        name="head_service_hatch",
    )
    for idx, (x_pos, y_pos, z_pos) in enumerate(((0.010, 0.036, 0.026), (0.010, 0.036, -0.026), (0.010, -0.036, 0.026), (0.010, -0.036, -0.026))):
        head.visual(
            Cylinder(radius=0.0045, length=0.016),
            origin=Origin(xyz=(x_pos, y_pos, z_pos), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hardware,
            name=f"pivot_plate_bolt_{idx:02d}",
        )
    for idx, (x_pos, y_pos) in enumerate(((0.066, 0.028), (0.066, -0.028), (0.094, 0.028), (0.094, -0.028))):
        head.visual(
            Cylinder(radius=0.0045, length=0.014),
            origin=Origin(xyz=(x_pos, y_pos, 0.081)),
            material=hardware,
            name=f"head_hatch_bolt_{idx:02d}",
        )
    head.inertial = Inertial.from_geometry(
        Box((0.34, 0.50, 0.50)),
        mass=4.2,
        origin=Origin(xyz=(0.140, 0.0, 0.0)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.038, length=0.008),
        origin=Origin(xyz=(0.004, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="hub_washer",
    )
    rotor.visual(
        Cylinder(radius=0.046, length=0.056),
        origin=Origin(xyz=(0.032, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="rotor_hub",
    )
    rotor.visual(
        Cylinder(radius=0.070, length=0.012),
        origin=Origin(xyz=(0.014, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rotor_black,
        name="hub_flange",
    )
    rotor.visual(
        Cylinder(radius=0.020, length=0.050),
        origin=Origin(xyz=(0.032, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="blade_root_spindle",
    )
    rotor.visual(
        _save_mesh(
            "rotor_spinner",
            LatheGeometry(
                [
                    (0.0, 0.020),
                    (0.016, 0.028),
                    (0.030, 0.046),
                    (0.035, 0.068),
                    (0.024, 0.086),
                    (0.0, 0.092),
                ],
                segments=56,
            ).rotate_y(math.pi / 2.0),
        ),
        origin=Origin(xyz=(0.025, 0.0, 0.0)),
        material=warm_gray,
        name="rotor_spinner",
    )
    blade_mesh = _save_mesh("rotor_blade_leaf", _build_rotor_blade_mesh())
    for index in range(4):
        rotor.visual(
            Box((0.038, 0.018, 0.060)),
            origin=Origin(xyz=(0.040, 0.0, 0.032), rpy=(index * (math.pi / 2.0), 0.0, 0.0)),
            material=dark_steel,
            name=f"blade_arm_{index:02d}",
        )
        rotor.visual(
            blade_mesh,
            origin=Origin(rpy=(index * (math.pi / 2.0), 0.0, 0.0)),
            material=rotor_black,
            name=f"rotor_blade_{index:02d}",
        )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.190, length=0.110),
        mass=0.9,
        origin=Origin(xyz=(0.060, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "stand_to_head_tilt",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.436)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.4,
            lower=math.radians(-18.0),
            upper=math.radians(32.0),
        ),
    )
    model.articulation(
        "head_to_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=rotor,
        origin=Origin(xyz=(0.141, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=28.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    head = object_model.get_part("head")
    rotor = object_model.get_part("rotor")
    tilt = object_model.get_articulation("stand_to_head_tilt")
    spin = object_model.get_articulation("head_to_rotor_spin")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "tilt_axis_direction",
        tuple(tilt.axis) == (0.0, -1.0, 0.0),
        details=f"expected tilt axis (0, -1, 0), got {tilt.axis}",
    )
    ctx.check(
        "rotor_spin_axis_direction",
        tuple(spin.axis) == (1.0, 0.0, 0.0),
        details=f"expected rotor spin axis (1, 0, 0), got {spin.axis}",
    )

    with ctx.pose({tilt: 0.0, spin: 0.0}):
        ctx.expect_contact(head, stand, elem_a="trunnion_bar", elem_b="left_yoke_arm", name="left_pivot_supported_closed")
        ctx.expect_contact(head, stand, elem_a="trunnion_bar", elem_b="right_yoke_arm", name="right_pivot_supported_closed")
        ctx.expect_contact(rotor, head, elem_a="rotor_hub", elem_b="nose_bearing", name="rotor_hub_supported_on_nose")
        ctx.expect_within(
            rotor,
            head,
            axes="yz",
            inner_elem="rotor_blade_00",
            outer_elem="guard_cage",
            margin=0.0,
            name="rotor_blade_within_guard_projection",
        )
        closed_rotor_pos = ctx.part_world_position(rotor)

    with ctx.pose({tilt: math.radians(24.0), spin: 1.1}):
        ctx.expect_contact(head, stand, elem_a="trunnion_bar", elem_b="left_yoke_arm", name="left_pivot_supported_tilted")
        ctx.expect_contact(head, stand, elem_a="trunnion_bar", elem_b="right_yoke_arm", name="right_pivot_supported_tilted")
        ctx.expect_contact(rotor, head, elem_a="rotor_hub", elem_b="nose_bearing", name="rotor_hub_supported_while_spinning")
        tilted_rotor_pos = ctx.part_world_position(rotor)

    if closed_rotor_pos is not None and tilted_rotor_pos is not None:
        ctx.check(
            "positive_tilt_lifts_head_front",
            tilted_rotor_pos[2] > closed_rotor_pos[2] + 0.03,
            details=f"expected tilted rotor origin z > {closed_rotor_pos[2] + 0.03:.4f}, got {tilted_rotor_pos[2]:.4f}",
        )
    else:
        ctx.fail("rotor_pose_measurement", "could not measure rotor origin across tilt poses")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
