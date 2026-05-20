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
    section_loft,
)


def _blade_section(
    span_x: float,
    half_width: float,
    thickness: float,
    *,
    y_shift: float = 0.0,
):
    half_thickness = thickness * 0.5
    return [
        (span_x, y_shift + half_width, half_thickness),
        (span_x, y_shift - half_width, half_thickness),
        (span_x, y_shift - half_width, -half_thickness),
        (span_x, y_shift + half_width, -half_thickness),
    ]


def _build_blade_mesh():
    return section_loft(
        [
            _blade_section(0.115, 0.052, 0.0076, y_shift=0.000),
            _blade_section(0.255, 0.049, 0.0066, y_shift=-0.003),
            _blade_section(0.430, 0.044, 0.0050, y_shift=-0.012),
            _blade_section(0.585, 0.026, 0.0032, y_shift=-0.020),
        ]
    )


def _build_motor_shell_mesh():
    return LatheGeometry.from_shell_profiles(
        [
            (0.088, 0.000),
            (0.128, -0.010),
            (0.145, -0.028),
            (0.142, -0.052),
        ],
        [
            (0.080, -0.003),
            (0.116, -0.011),
            (0.116, -0.041),
            (0.108, -0.052),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )


def _build_light_tray_shell_mesh():
    return LatheGeometry.from_shell_profiles(
        [
            (0.086, 0.014),
            (0.084, 0.006),
            (0.078, -0.006),
            (0.072, -0.014),
        ],
        [
            (0.062, 0.010),
            (0.064, 0.002),
            (0.068, -0.006),
            (0.072, -0.014),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )


def _build_light_ring_mesh():
    return LatheGeometry.from_shell_profiles(
        [
            (0.078, 0.003),
            (0.078, -0.003),
        ],
        [
            (0.062, 0.003),
            (0.062, -0.003),
        ],
        segments=56,
        start_cap="flat",
        end_cap="flat",
    )


def _aabb_extents(aabb):
    if aabb is None:
        return None
    (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
    return (max_x - min_x, max_y - min_y, max_z - min_z)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="contemporary_ceiling_fan")

    housing_finish = model.material("housing_finish", rgba=(0.17, 0.19, 0.22, 1.0))
    trim_finish = model.material("trim_finish", rgba=(0.26, 0.28, 0.31, 1.0))
    blade_finish = model.material("blade_finish", rgba=(0.78, 0.80, 0.82, 1.0))
    diffuser_finish = model.material("diffuser_finish", rgba=(0.94, 0.95, 0.97, 0.92))

    blade_mesh = mesh_from_geometry(_build_blade_mesh(), "fan_blade_v2")
    motor_shell_mesh = mesh_from_geometry(_build_motor_shell_mesh(), "motor_shell")

    housing = model.part("housing")
    housing.visual(
        Cylinder(radius=0.082, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
        material=housing_finish,
        name="canopy",
    )
    housing.visual(
        Cylinder(radius=0.090, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.034)),
        material=trim_finish,
        name="canopy_trim",
    )
    housing.visual(
        motor_shell_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.036)),
        material=housing_finish,
        name="motor_shell",
    )
    housing.visual(
        Cylinder(radius=0.060, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, -0.062)),
        material=trim_finish,
        name="motor_core",
    )
    housing.visual(
        Box((0.014, 0.018, 0.050)),
        origin=Origin(xyz=(-0.104, 0.094, -0.112)),
        material=trim_finish,
        name="hinge_lug_left",
    )
    housing.visual(
        Box((0.014, 0.018, 0.050)),
        origin=Origin(xyz=(0.104, 0.094, -0.112)),
        material=trim_finish,
        name="hinge_lug_right",
    )
    housing.inertial = Inertial.from_geometry(
        Cylinder(radius=0.145, length=0.088),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, -0.044)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.060, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=trim_finish,
        name="hub_flange",
    )
    rotor.visual(
        Cylinder(radius=0.056, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.016)),
        material=trim_finish,
        name="hub_body",
    )
    rotor.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.021)),
        material=trim_finish,
        name="axle_spindle",
    )
    for index, yaw in enumerate((0.0, 0.5 * math.pi, math.pi, 1.5 * math.pi), start=1):
        arm_radius = 0.087
        rotor.visual(
            Box((0.120, 0.024, 0.006)),
            origin=Origin(
                xyz=(arm_radius * math.cos(yaw), arm_radius * math.sin(yaw), -0.010),
                rpy=(-0.16, 0.0, yaw),
            ),
            material=trim_finish,
            name=f"blade_arm_{index}",
        )
        rotor.visual(
            blade_mesh,
            origin=Origin(xyz=(0.0, 0.0, -0.010), rpy=(-0.16, 0.0, yaw)),
            material=blade_finish,
            name=f"blade_{index}",
        )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.60, length=0.040),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
    )

    light_tray = model.part("light_tray")
    light_tray.visual(
        Cylinder(radius=0.0085, length=0.194),
        origin=Origin(rpy=(0.0, 0.5 * math.pi, 0.0)),
        material=trim_finish,
        name="hinge_barrel",
    )
    light_tray.visual(
        Box((0.188, 0.022, 0.016)),
        origin=Origin(xyz=(0.0, -0.011, -0.008)),
        material=trim_finish,
        name="hinge_bracket",
    )
    light_tray.visual(
        Box((0.102, 0.092, 0.024)),
        origin=Origin(xyz=(0.0, -0.056, -0.024)),
        material=trim_finish,
        name="tray_support",
    )
    light_tray.visual(
        Cylinder(radius=0.086, length=0.018),
        origin=Origin(xyz=(0.0, -0.098, -0.040)),
        material=housing_finish,
        name="tray_shell",
    )
    light_tray.visual(
        Cylinder(radius=0.074, length=0.006),
        origin=Origin(xyz=(0.0, -0.098, -0.050)),
        material=diffuser_finish,
        name="tray_diffuser",
    )
    light_tray.inertial = Inertial.from_geometry(
        Cylinder(radius=0.095, length=0.070),
        mass=1.1,
        origin=Origin(xyz=(0.0, -0.072, -0.032)),
    )

    model.articulation(
        "housing_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, -0.088)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=15.0),
    )
    model.articulation(
        "housing_to_light_tray",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=light_tray,
        origin=Origin(xyz=(0.0, 0.094, -0.112)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=1.5,
            lower=0.0,
            upper=1.20,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    rotor = object_model.get_part("rotor")
    light_tray = object_model.get_part("light_tray")
    spin = object_model.get_articulation("housing_to_rotor")
    tray_hinge = object_model.get_articulation("housing_to_light_tray")

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    ctx.check(
        "rotor_joint_is_continuous",
        spin.articulation_type == ArticulationType.CONTINUOUS,
        f"joint_type={spin.articulation_type}",
    )
    ctx.check(
        "rotor_axis_is_vertical",
        tuple(round(value, 6) for value in spin.axis) == (0.0, 0.0, 1.0),
        f"axis={spin.axis}",
    )
    ctx.check(
        "tray_joint_is_revolute",
        tray_hinge.articulation_type == ArticulationType.REVOLUTE,
        f"joint_type={tray_hinge.articulation_type}",
    )
    ctx.check(
        "tray_hinge_axis_is_horizontal",
        tuple(round(value, 6) for value in tray_hinge.axis) == (1.0, 0.0, 0.0),
        f"axis={tray_hinge.axis}",
    )
    tray_limits = tray_hinge.motion_limits
    ctx.check(
        "tray_hinge_limits_realistic",
        tray_limits is not None
        and tray_limits.lower == 0.0
        and tray_limits.upper is not None
        and 1.0 <= tray_limits.upper <= 1.35,
        f"limits={tray_limits}",
    )

    housing_shell_aabb = ctx.part_element_world_aabb(housing, elem="motor_shell")
    rotor_aabb = ctx.part_world_aabb(rotor)
    housing_shell_extents = _aabb_extents(housing_shell_aabb)
    rotor_extents = _aabb_extents(rotor_aabb)
    ctx.check(
        "housing_shell_low_profile",
        housing_shell_extents is not None
        and 0.27 <= max(housing_shell_extents[0], housing_shell_extents[1]) <= 0.31
        and 0.045 <= housing_shell_extents[2] <= 0.060,
        f"housing_shell_extents={housing_shell_extents}",
    )
    ctx.check(
        "fan_span_realistic",
        rotor_extents is not None and 1.10 <= max(rotor_extents[0], rotor_extents[1]) <= 1.24,
        f"rotor_extents={rotor_extents}",
    )

    for blade_name in ("blade_1", "blade_2", "blade_3", "blade_4"):
        ctx.check(
            f"{blade_name}_present",
            ctx.part_element_world_aabb(rotor, elem=blade_name) is not None,
            f"missing element {blade_name}",
        )

    ctx.expect_origin_distance(rotor, housing, axes="xy", max_dist=0.001, name="rotor_centered")
    ctx.expect_overlap(
        rotor,
        housing,
        axes="xy",
        min_overlap=0.10,
        elem_a="hub_body",
        elem_b="motor_shell",
        name="hub_nested_under_housing",
    )

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")
    with ctx.pose({tray_hinge: 0.0}):
        ctx.expect_gap(
            housing,
            rotor,
            axis="z",
            min_gap=0.0,
            max_gap=0.001,
            positive_elem="motor_core",
            negative_elem="hub_flange",
            name="rotor_flange_seated_to_motor_core",
        )
        ctx.expect_contact(
            light_tray,
            housing,
            elem_a="hinge_barrel",
            elem_b="hinge_lug_left",
            name="tray_left_hinge_contact_closed",
        )
        ctx.expect_contact(
            light_tray,
            housing,
            elem_a="hinge_barrel",
            elem_b="hinge_lug_right",
            name="tray_right_hinge_contact_closed",
        )
        ctx.expect_gap(
            housing,
            light_tray,
            axis="z",
            min_gap=0.045,
            max_gap=0.060,
            positive_elem="motor_shell",
            negative_elem="tray_shell",
            name="tray_closed_standoff_below_housing",
        )
        ctx.expect_within(
            light_tray,
            housing,
            axes="xy",
            margin=0.0,
            inner_elem="tray_diffuser",
            outer_elem="motor_shell",
            name="tray_centered_under_housing",
        )

    if tray_limits is not None and tray_limits.lower is not None and tray_limits.upper is not None:
        with ctx.pose({tray_hinge: tray_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="tray_hinge_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="tray_hinge_lower_no_floating")
        with ctx.pose({tray_hinge: tray_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="tray_hinge_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="tray_hinge_upper_no_floating")
            ctx.expect_contact(
                light_tray,
                housing,
                elem_a="hinge_barrel",
                elem_b="hinge_lug_left",
                name="tray_left_hinge_contact_open",
            )
            ctx.expect_contact(
                light_tray,
                housing,
                elem_a="hinge_barrel",
                elem_b="hinge_lug_right",
                name="tray_right_hinge_contact_open",
            )
            ctx.expect_gap(
                rotor,
                light_tray,
                axis="z",
                min_gap=0.030,
                positive_elem="blade_1",
                negative_elem="tray_diffuser",
                name="tray_open_clears_blade_plane",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
