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
    TorusGeometry,
    mesh_from_geometry,
    section_loft,
    wire_from_points,
)


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


def _add_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _normalize(v: tuple[float, float, float]) -> tuple[float, float, float]:
    mag = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
    return (v[0] / mag, v[1] / mag, v[2] / mag)


def _cross(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _blade_section(
    *,
    radius: float,
    sweep_angle: float,
    chord: float,
    thickness: float,
    pitch: float,
    axial_offset: float,
) -> list[tuple[float, float, float]]:
    radial = (math.cos(sweep_angle), 0.0, math.sin(sweep_angle))
    tangent = (-math.sin(sweep_angle), 0.0, math.cos(sweep_angle))
    chord_dir = _normalize(
        (
            tangent[0] * math.cos(pitch),
            math.sin(pitch),
            tangent[2] * math.cos(pitch),
        )
    )
    normal = _normalize(_cross(radial, chord_dir))
    center = (radial[0] * radius, axial_offset, radial[2] * radius)
    half_chord = chord * 0.5
    half_thickness = thickness * 0.5
    return [
        (
            center[0] + chord_dir[0] * half_chord + normal[0] * half_thickness,
            center[1] + chord_dir[1] * half_chord + normal[1] * half_thickness,
            center[2] + chord_dir[2] * half_chord + normal[2] * half_thickness,
        ),
        (
            center[0] - chord_dir[0] * half_chord + normal[0] * half_thickness,
            center[1] - chord_dir[1] * half_chord + normal[1] * half_thickness,
            center[2] - chord_dir[2] * half_chord + normal[2] * half_thickness,
        ),
        (
            center[0] - chord_dir[0] * half_chord - normal[0] * half_thickness,
            center[1] - chord_dir[1] * half_chord - normal[1] * half_thickness,
            center[2] - chord_dir[2] * half_chord - normal[2] * half_thickness,
        ),
        (
            center[0] + chord_dir[0] * half_chord - normal[0] * half_thickness,
            center[1] + chord_dir[1] * half_chord - normal[1] * half_thickness,
            center[2] + chord_dir[2] * half_chord - normal[2] * half_thickness,
        ),
    ]


def _build_backward_curved_blade():
    return section_loft(
        [
            _blade_section(
                radius=0.098,
                sweep_angle=0.54,
                chord=0.142,
                thickness=0.0045,
                pitch=0.96,
                axial_offset=-0.034,
            ),
            _blade_section(
                radius=0.182,
                sweep_angle=0.16,
                chord=0.120,
                thickness=0.0040,
                pitch=0.76,
                axial_offset=-0.016,
            ),
            _blade_section(
                radius=0.252,
                sweep_angle=-0.18,
                chord=0.104,
                thickness=0.0036,
                pitch=0.58,
                axial_offset=0.000,
            ),
        ]
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_kitchen_exhaust_box_fan")

    housing_steel = model.material("housing_steel", rgba=(0.74, 0.76, 0.78, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.28, 0.30, 0.33, 1.0))
    black_paint = model.material("black_paint", rgba=(0.12, 0.12, 0.13, 1.0))
    blade_finish = model.material("blade_finish", rgba=(0.34, 0.36, 0.39, 1.0))
    control_red = model.material("control_red", rgba=(0.76, 0.17, 0.12, 1.0))
    marker_white = model.material("marker_white", rgba=(0.92, 0.93, 0.94, 1.0))

    housing_width = 0.82
    housing_height = 0.82
    housing_depth = 0.38
    wall_thickness = 0.028
    inner_half_span = (housing_width * 0.5) - wall_thickness

    housing = model.part("housing")
    housing.visual(
        Box((housing_width, housing_depth, wall_thickness)),
        origin=Origin(xyz=(0.0, 0.0, housing_height * 0.5 - wall_thickness * 0.5)),
        material=housing_steel,
        name="top_panel",
    )
    housing.visual(
        Box((housing_width, housing_depth, wall_thickness)),
        origin=Origin(xyz=(0.0, 0.0, -housing_height * 0.5 + wall_thickness * 0.5)),
        material=housing_steel,
        name="bottom_panel",
    )
    housing.visual(
        Box((wall_thickness, housing_depth, housing_height - 2.0 * wall_thickness)),
        origin=Origin(xyz=(-housing_width * 0.5 + wall_thickness * 0.5, 0.0, 0.0)),
        material=housing_steel,
        name="left_panel",
    )
    housing.visual(
        Box((wall_thickness, housing_depth, housing_height - 2.0 * wall_thickness)),
        origin=Origin(xyz=(housing_width * 0.5 - wall_thickness * 0.5, 0.0, 0.0)),
        material=housing_steel,
        name="right_panel",
    )
    housing.visual(
        Box((0.006, 0.170, 0.170)),
        origin=Origin(xyz=(housing_width * 0.5 + 0.003, -0.055, 0.060)),
        material=dark_steel,
        name="control_plate",
    )
    for index, (marker_y, marker_z) in enumerate(
        ((-0.092, 0.022), (-0.055, 0.058), (-0.018, 0.022))
    ):
        housing.visual(
            Box((0.003, 0.012, 0.004)),
            origin=Origin(xyz=(housing_width * 0.5 + 0.0075, marker_y, marker_z)),
            material=marker_white,
            name=f"dial_marker_{index}",
        )
    housing.inertial = Inertial.from_geometry(
        Box((housing_width, housing_depth, housing_height)),
        mass=24.0,
        origin=Origin(),
    )

    support_frame = model.part("support_frame")
    shroud_geom = LatheGeometry.from_shell_profiles(
        [
            (0.318, -0.150),
            (0.312, -0.108),
            (0.308, 0.010),
            (0.306, 0.086),
        ],
        [
            (0.300, -0.146),
            (0.292, -0.104),
            (0.288, 0.006),
            (0.286, 0.086),
        ],
        segments=72,
    ).rotate_x(-math.pi / 2.0)
    support_frame.visual(
        mesh_from_geometry(shroud_geom, "fan_venturi_shroud"),
        material=dark_steel,
        name="venturi_shroud",
    )
    support_frame.visual(
        Box((0.080, 0.100, 0.076)),
        origin=Origin(xyz=(0.0, 0.040, 0.344)),
        material=dark_steel,
        name="top_mount_arm",
    )
    support_frame.visual(
        Box((0.080, 0.100, 0.076)),
        origin=Origin(xyz=(0.0, 0.040, -0.344)),
        material=dark_steel,
        name="bottom_mount_arm",
    )
    support_frame.visual(
        Box((0.076, 0.100, 0.080)),
        origin=Origin(xyz=(0.344, 0.040, 0.0)),
        material=dark_steel,
        name="right_mount_arm",
    )
    support_frame.visual(
        Box((0.076, 0.100, 0.080)),
        origin=Origin(xyz=(-0.344, 0.040, 0.0)),
        material=dark_steel,
        name="left_mount_arm",
    )
    support_frame.visual(
        Cylinder(radius=0.078, length=0.140),
        origin=Origin(xyz=(0.0, 0.090, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black_paint,
        name="motor_can",
    )
    support_frame.visual(
        Cylinder(radius=0.031, length=0.030),
        origin=Origin(xyz=(0.0, 0.005, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black_paint,
        name="bearing_boss",
    )
    _add_member(
        support_frame,
        (0.0, 0.040, 0.078),
        (0.0, 0.040, 0.306),
        radius=0.010,
        material=dark_steel,
        name="top_spider",
    )
    _add_member(
        support_frame,
        (0.0, 0.040, -0.078),
        (0.0, 0.040, -0.306),
        radius=0.010,
        material=dark_steel,
        name="bottom_spider",
    )
    _add_member(
        support_frame,
        (0.078, 0.040, 0.0),
        (0.306, 0.040, 0.0),
        radius=0.010,
        material=dark_steel,
        name="right_spider",
    )
    _add_member(
        support_frame,
        (-0.078, 0.040, 0.0),
        (-0.306, 0.040, 0.0),
        radius=0.010,
        material=dark_steel,
        name="left_spider",
    )
    support_frame.inertial = Inertial.from_geometry(
        Box((0.640, 0.260, 0.640)),
        mass=5.5,
        origin=Origin(xyz=(0.0, 0.020, 0.0)),
    )

    guard = model.part("guard")
    guard.visual(
        mesh_from_geometry(
            wire_from_points(
                [
                    (0.376, -0.162, 0.376),
                    (0.376, -0.162, -0.376),
                    (-0.376, -0.162, -0.376),
                    (-0.376, -0.162, 0.376),
                ],
                radius=0.006,
                radial_segments=18,
                closed_path=True,
                corner_mode="fillet",
                corner_radius=0.060,
                corner_segments=8,
            ),
            "guard_outer_frame",
        ),
        material=dark_steel,
        name="outer_guard_frame",
    )
    guard.visual(
        mesh_from_geometry(
            TorusGeometry(
                radius=0.108,
                tube=0.0045,
                radial_segments=16,
                tubular_segments=56,
            ).rotate_x(-math.pi / 2.0),
            "guard_center_ring",
        ),
        origin=Origin(xyz=(0.0, -0.162, 0.0)),
        material=dark_steel,
        name="center_guard_ring",
    )
    for index, x_pos in enumerate((-0.240, -0.120, 0.0, 0.120, 0.240)):
        _add_member(
            guard,
            (x_pos, -0.162, -0.370),
            (x_pos, -0.162, 0.370),
            radius=0.0035,
            material=dark_steel,
            name=f"vertical_guard_wire_{index}",
        )
    for index, z_pos in enumerate((-0.240, -0.120, 0.0, 0.120, 0.240)):
        _add_member(
            guard,
            (-0.370, -0.162, z_pos),
            (0.370, -0.162, z_pos),
            radius=0.0035,
            material=dark_steel,
            name=f"horizontal_guard_wire_{index}",
        )
    guard.inertial = Inertial.from_geometry(
        Box((0.770, 0.020, 0.770)),
        mass=1.2,
        origin=Origin(xyz=(0.0, -0.162, 0.0)),
    )

    impeller = model.part("impeller")
    single_blade = _build_backward_curved_blade()
    blade_set = single_blade.clone()
    for blade_index in range(1, 6):
        blade_set.merge(single_blade.clone().rotate_y(blade_index * math.tau / 6.0))
    impeller.visual(
        mesh_from_geometry(blade_set, "impeller_blade_set"),
        material=blade_finish,
        name="blade_set",
    )
    impeller.visual(
        Cylinder(radius=0.060, length=0.082),
        origin=Origin(xyz=(0.0, -0.069, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black_paint,
        name="hub_shell",
    )
    impeller.visual(
        Cylinder(radius=0.112, length=0.010),
        origin=Origin(xyz=(0.0, -0.034, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=blade_finish,
        name="blade_backplate",
    )
    impeller.visual(
        Cylinder(radius=0.030, length=0.018),
        origin=Origin(xyz=(0.0, -0.019, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black_paint,
        name="rear_collar",
    )
    impeller.visual(
        Cylinder(radius=0.040, length=0.026),
        origin=Origin(xyz=(0.0, -0.115, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black_paint,
        name="nose_cap",
    )
    impeller.inertial = Inertial.from_geometry(
        Cylinder(radius=0.270, length=0.120),
        mass=3.2,
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
    )

    dial_knob = model.part("dial_knob")
    dial_knob.visual(
        Cylinder(radius=0.035, length=0.012),
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_paint,
        name="base_collar",
    )
    dial_knob.visual(
        Cylinder(radius=0.030, length=0.026),
        origin=Origin(xyz=(0.025, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=control_red,
        name="knob_body",
    )
    dial_knob.visual(
        Cylinder(radius=0.024, length=0.010),
        origin=Origin(xyz=(0.043, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=control_red,
        name="knob_face",
    )
    dial_knob.visual(
        Box((0.012, 0.006, 0.028)),
        origin=Origin(xyz=(0.037, 0.0, 0.042)),
        material=marker_white,
        name="pointer_fin",
    )
    dial_knob.inertial = Inertial.from_geometry(
        Box((0.060, 0.080, 0.080)),
        mass=0.10,
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
    )

    model.articulation(
        "housing_to_support_frame",
        ArticulationType.FIXED,
        parent=housing,
        child=support_frame,
        origin=Origin(),
    )
    model.articulation(
        "housing_to_guard",
        ArticulationType.FIXED,
        parent=housing,
        child=guard,
        origin=Origin(),
    )
    model.articulation(
        "support_frame_to_impeller",
        ArticulationType.CONTINUOUS,
        parent=support_frame,
        child=impeller,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=40.0),
    )
    model.articulation(
        "housing_to_dial_knob",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=dial_knob,
        origin=Origin(xyz=(housing_width * 0.5 + 0.006, -0.055, 0.060)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=1.6,
            lower=-0.70,
            upper=0.70,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    support_frame = object_model.get_part("support_frame")
    guard = object_model.get_part("guard")
    impeller = object_model.get_part("impeller")
    dial_knob = object_model.get_part("dial_knob")
    impeller_spin = object_model.get_articulation("support_frame_to_impeller")
    dial_joint = object_model.get_articulation("housing_to_dial_knob")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(guard, housing, name="guard_mounted_to_housing")
    ctx.expect_contact(support_frame, housing, name="support_frame_mounted_to_housing")
    ctx.expect_contact(
        impeller,
        support_frame,
        elem_a="rear_collar",
        elem_b="bearing_boss",
        name="impeller_bearing_contact",
    )
    ctx.expect_contact(dial_knob, housing, name="dial_knob_mounted_to_housing")
    ctx.expect_gap(
        impeller,
        guard,
        axis="y",
        min_gap=0.020,
        name="impeller_sits_behind_guard",
    )
    ctx.expect_overlap(
        impeller,
        guard,
        axes="xz",
        min_overlap=0.500,
        name="guard_covers_impeller_span",
    )
    ctx.expect_within(
        impeller,
        housing,
        axes="xz",
        margin=0.020,
        name="impeller_within_housing_profile",
    )
    ctx.check(
        "impeller_joint_axis_is_axial",
        impeller_spin.axis == (0.0, 1.0, 0.0),
        f"Expected impeller axis (0, 1, 0), got {impeller_spin.axis}",
    )
    ctx.check(
        "dial_joint_axis_is_side_normal",
        dial_joint.axis == (1.0, 0.0, 0.0),
        f"Expected dial axis (1, 0, 0), got {dial_joint.axis}",
    )
    dial_limits = dial_joint.motion_limits
    ctx.check(
        "dial_has_three_positions",
        dial_limits is not None
        and dial_limits.lower is not None
        and dial_limits.upper is not None
        and dial_limits.lower < 0.0 < dial_limits.upper,
        "Dial should expose low / medium / high positions around a centered detent.",
    )

    with ctx.pose({impeller_spin: math.pi / 6.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="impeller_rotated_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="impeller_rotated_pose_no_floating")
        ctx.expect_gap(
            impeller,
            guard,
            axis="y",
            min_gap=0.020,
            name="impeller_rotated_pose_guard_gap",
        )
        ctx.expect_contact(
            impeller,
            support_frame,
            elem_a="rear_collar",
            elem_b="bearing_boss",
            name="impeller_rotated_pose_bearing_contact",
        )

    if dial_limits is not None and dial_limits.lower is not None and dial_limits.upper is not None:
        rest_aabb = ctx.part_world_aabb(dial_knob)
        assert rest_aabb is not None
        with ctx.pose({dial_joint: dial_limits.lower}):
            lower_aabb = ctx.part_world_aabb(dial_knob)
            assert lower_aabb is not None
            ctx.fail_if_parts_overlap_in_current_pose(name="dial_lower_pose_no_overlap")
            ctx.fail_if_isolated_parts(name="dial_lower_pose_no_floating")
            ctx.expect_contact(
                dial_knob,
                housing,
                name="dial_lower_pose_mount_contact",
            )
        with ctx.pose({dial_joint: dial_limits.upper}):
            upper_aabb = ctx.part_world_aabb(dial_knob)
            assert upper_aabb is not None
            ctx.fail_if_parts_overlap_in_current_pose(name="dial_upper_pose_no_overlap")
            ctx.fail_if_isolated_parts(name="dial_upper_pose_no_floating")
            ctx.expect_contact(
                dial_knob,
                housing,
                name="dial_upper_pose_mount_contact",
            )
        ctx.check(
            "dial_pointer_sweeps_across_positions",
            lower_aabb[1][1] > rest_aabb[1][1] + 0.004
            and upper_aabb[0][1] < rest_aabb[0][1] - 0.004,
            (
                "Expected the asymmetric pointer fin to move across the side plate; "
                f"rest={rest_aabb}, lower={lower_aabb}, upper={upper_aabb}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
