from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
    tube_from_spline_points,
)


def _blade_section(
    x_pos: float,
    half_width: float,
    thickness: float,
    *,
    y_offset: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x_pos, y_offset - 1.00 * half_width, -0.22 * thickness),
        (x_pos, y_offset - 0.24 * half_width, -0.58 * thickness),
        (x_pos, y_offset + 0.98 * half_width, -0.12 * thickness),
        (x_pos, y_offset + 0.78 * half_width, 0.42 * thickness),
        (x_pos, y_offset - 0.10 * half_width, 0.60 * thickness),
        (x_pos, y_offset - 0.88 * half_width, 0.26 * thickness),
    ]


def _build_canopy_shell_mesh():
    outer_profile = [
        (0.020, 0.000),
        (0.050, -0.012),
        (0.070, -0.040),
        (0.074, -0.076),
        (0.058, -0.106),
    ]
    inner_profile = [
        (0.000, -0.002),
        (0.040, -0.014),
        (0.060, -0.040),
        (0.063, -0.078),
        (0.046, -0.094),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=56,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )


def _build_blade_set_mesh():
    blade_sections = [
        _blade_section(0.230, 0.052, 0.0075, y_offset=-0.006),
        _blade_section(0.410, 0.049, 0.0070, y_offset=0.000),
        _blade_section(0.575, 0.045, 0.0064, y_offset=0.011),
        _blade_section(0.730, 0.038, 0.0055, y_offset=0.018),
        _blade_section(0.835, 0.020, 0.0038, y_offset=0.020),
    ]
    single_blade = section_loft(blade_sections)
    single_blade.merge(BoxGeometry((0.135, 0.044, 0.008)).translate(0.165, 0.000, 0.000))
    single_blade.rotate_x(math.radians(12.0))

    blade_set = BoxGeometry((0.020, 0.020, 0.006)).translate(0.000, 0.000, -0.058)
    for index in range(4):
        blade_set.merge(single_blade.copy().rotate_z(math.pi / 4.0 + index * math.pi / 2.0))
    return blade_set


def _build_spotlight_arm_mesh():
    arm_mesh = BoxGeometry((0.018, 0.016, 0.020)).translate(0.000, 0.008, -0.004)

    arm_tube = tube_from_spline_points(
        [
            (0.000, 0.016, -0.004),
            (0.000, 0.052, -0.026),
            (0.000, 0.090, -0.070),
            (0.000, 0.100, -0.082),
        ],
        radius=0.006,
        samples_per_segment=14,
        radial_segments=18,
    )
    arm_mesh.merge(arm_tube)

    end_collar = CylinderGeometry(radius=0.010, height=0.012, radial_segments=24)
    end_collar.rotate_x(-math.pi / 2.0)
    end_collar.translate(0.000, 0.100, -0.082)
    arm_mesh.merge(end_collar)
    return arm_mesh


def _build_spotlight_head_shell_mesh():
    outer_profile = [
        (0.010, -0.004),
        (0.028, 0.000),
        (0.033, 0.020),
        (0.034, 0.052),
        (0.031, 0.076),
    ]
    inner_profile = [
        (0.000, -0.002),
        (0.022, 0.006),
        (0.026, 0.046),
        (0.024, 0.062),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=48,
        start_cap="flat",
        end_cap="flat",
        lip_samples=6,
    ).rotate_x(-math.pi / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ceiling_fan_with_spotlight")

    housing_white = model.material("housing_white", rgba=(0.93, 0.93, 0.92, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.72, 0.74, 0.77, 1.0))
    walnut = model.material("walnut", rgba=(0.46, 0.31, 0.19, 1.0))
    charcoal = model.material("charcoal", rgba=(0.14, 0.15, 0.17, 1.0))

    ceiling_mount = model.part("ceiling_mount")
    ceiling_mount.visual(
        Cylinder(radius=0.088, length=0.010),
        origin=Origin(xyz=(0.000, 0.000, -0.005)),
        material=brushed_metal,
        name="ceiling_plate",
    )
    ceiling_mount.visual(
        mesh_from_geometry(_build_canopy_shell_mesh(), "canopy_shell"),
        material=housing_white,
        name="canopy_shell",
    )
    ceiling_mount.visual(
        Cylinder(radius=0.013, length=0.160),
        origin=Origin(xyz=(0.000, 0.000, -0.185)),
        material=brushed_metal,
        name="downrod",
    )
    ceiling_mount.visual(
        Cylinder(radius=0.020, length=0.068),
        origin=Origin(xyz=(0.000, 0.000, -0.139)),
        material=brushed_metal,
        name="hanger_sleeve",
    )
    ceiling_mount.visual(
        Cylinder(radius=0.022, length=0.095),
        origin=Origin(xyz=(0.000, 0.000, -0.0575)),
        material=brushed_metal,
        name="downrod_socket",
    )
    ceiling_mount.visual(
        Cylinder(radius=0.024, length=0.020),
        origin=Origin(xyz=(0.000, 0.000, -0.275)),
        material=charcoal,
        name="lower_collar",
    )
    ceiling_mount.inertial = Inertial.from_geometry(
        Cylinder(radius=0.090, length=0.290),
        mass=2.8,
        origin=Origin(xyz=(0.000, 0.000, -0.145)),
    )

    rotor = model.part("fan_rotor")
    rotor.visual(
        Cylinder(radius=0.024, length=0.022),
        origin=Origin(xyz=(0.000, 0.000, -0.011)),
        material=charcoal,
        name="axle_collar",
    )
    rotor.visual(
        Cylinder(radius=0.125, length=0.036),
        origin=Origin(xyz=(0.000, 0.000, -0.034)),
        material=housing_white,
        name="motor_housing",
    )
    rotor.visual(
        Cylinder(radius=0.135, length=0.012),
        origin=Origin(xyz=(0.000, 0.000, -0.058)),
        material=brushed_metal,
        name="housing_trim_ring",
    )
    rotor.visual(
        Cylinder(radius=0.060, length=0.018),
        origin=Origin(xyz=(0.000, 0.000, -0.072)),
        material=charcoal,
        name="bottom_cap",
    )
    rotor.visual(
        mesh_from_geometry(_build_blade_set_mesh(), "blade_set"),
        origin=Origin(xyz=(0.000, 0.000, -0.004)),
        material=walnut,
        name="blade_set",
    )
    rotor.visual(
        Box((0.024, 0.010, 0.022)),
        origin=Origin(xyz=(0.000, 0.127, -0.048)),
        material=charcoal,
        name="arm_hinge_block",
    )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.620, length=0.090),
        mass=6.6,
        origin=Origin(xyz=(0.000, 0.000, -0.050)),
    )

    spotlight_arm = model.part("spotlight_arm")
    spotlight_arm.visual(
        mesh_from_geometry(_build_spotlight_arm_mesh(), "spotlight_arm"),
        material=charcoal,
        name="arm_body",
    )
    spotlight_arm.inertial = Inertial.from_geometry(
        Box((0.050, 0.130, 0.105)),
        mass=0.45,
        origin=Origin(xyz=(0.000, 0.060, -0.040)),
    )

    spotlight_head = model.part("spotlight_head")
    spotlight_head.visual(
        Cylinder(radius=0.011, length=0.012),
        origin=Origin(xyz=(0.000, 0.006, 0.000), rpy=(-math.pi / 2.0, 0.000, 0.000)),
        material=charcoal,
        name="rear_collar",
    )
    spotlight_head.visual(
        mesh_from_geometry(_build_spotlight_head_shell_mesh(), "spotlight_head_shell"),
        origin=Origin(xyz=(0.000, 0.006, 0.000)),
        material=charcoal,
        name="head_shell",
    )
    spotlight_head.inertial = Inertial.from_geometry(
        Cylinder(radius=0.036, length=0.080),
        mass=0.60,
        origin=Origin(xyz=(0.000, 0.038, 0.000), rpy=(-math.pi / 2.0, 0.000, 0.000)),
    )

    model.articulation(
        "fan_spin",
        ArticulationType.CONTINUOUS,
        parent=ceiling_mount,
        child=rotor,
        origin=Origin(xyz=(0.000, 0.000, -0.285)),
        axis=(0.000, 0.000, 1.000),
        motion_limits=MotionLimits(effort=25.0, velocity=18.0),
    )
    model.articulation(
        "arm_pivot",
        ArticulationType.REVOLUTE,
        parent=rotor,
        child=spotlight_arm,
        origin=Origin(xyz=(0.000, 0.132, -0.048)),
        axis=(1.000, 0.000, 0.000),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=-0.95,
            upper=0.45,
        ),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=spotlight_arm,
        child=spotlight_head,
        origin=Origin(xyz=(0.000, 0.106, -0.082)),
        axis=(1.000, 0.000, 0.000),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=-0.90,
            upper=0.70,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    try:
        ceiling_mount = object_model.get_part("ceiling_mount")
        rotor = object_model.get_part("fan_rotor")
        spotlight_arm = object_model.get_part("spotlight_arm")
        spotlight_head = object_model.get_part("spotlight_head")
        fan_spin = object_model.get_articulation("fan_spin")
        arm_pivot = object_model.get_articulation("arm_pivot")
        head_tilt = object_model.get_articulation("head_tilt")
    except Exception as exc:  # pragma: no cover - defensive authored test guard
        ctx.fail("required_parts_and_joints_present", str(exc))
        return ctx.report()

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
        "joint_types_and_axes",
        fan_spin.articulation_type == ArticulationType.CONTINUOUS
        and arm_pivot.articulation_type == ArticulationType.REVOLUTE
        and head_tilt.articulation_type == ArticulationType.REVOLUTE
        and fan_spin.axis == (0.0, 0.0, 1.0)
        and arm_pivot.axis == (1.0, 0.0, 0.0)
        and head_tilt.axis == (1.0, 0.0, 0.0),
        (
            f"fan_spin={fan_spin.articulation_type}/{fan_spin.axis}, "
            f"arm_pivot={arm_pivot.articulation_type}/{arm_pivot.axis}, "
            f"head_tilt={head_tilt.articulation_type}/{head_tilt.axis}"
        ),
    )

    ctx.expect_contact(ceiling_mount, rotor, contact_tol=0.0015, name="rotor_seated_on_downrod")
    ctx.expect_contact(rotor, spotlight_arm, contact_tol=0.0015, name="arm_hinge_connected")
    ctx.expect_contact(spotlight_arm, spotlight_head, contact_tol=0.0015, name="head_yoke_connected")

    ctx.expect_gap(
        rotor,
        spotlight_head,
        axis="z",
        min_gap=0.012,
        name="spotlight_head_hangs_below_rotor",
    )

    rotor_aabb = ctx.part_world_aabb(rotor)
    if rotor_aabb is None:
        ctx.fail("fan_span_realistic", "fan rotor AABB unavailable")
    else:
        span_x = rotor_aabb[1][0] - rotor_aabb[0][0]
        span_y = rotor_aabb[1][1] - rotor_aabb[0][1]
        ctx.check(
            "fan_span_realistic",
            1.10 <= span_x <= 1.35 and 1.10 <= span_y <= 1.35,
            f"span_x={span_x:.3f}, span_y={span_y:.3f}",
        )

    head_rest = ctx.part_world_position(spotlight_head)
    if head_rest is None:
        ctx.fail("spotlight_positioned_off_center", "head world position unavailable")
    else:
        radial_offset = math.hypot(head_rest[0], head_rest[1])
        ctx.check(
            "spotlight_positioned_off_center",
            0.18 <= radial_offset <= 0.30 and head_rest[2] < -0.36,
            f"radial_offset={radial_offset:.3f}, z={head_rest[2]:.3f}",
        )

    with ctx.pose({fan_spin: math.pi / 2.0}):
        spun_head = ctx.part_world_position(spotlight_head)
        if head_rest is None or spun_head is None:
            ctx.fail("fan_spin_moves_spotlight", "missing head positions for spin check")
        else:
            ctx.check(
                "fan_spin_moves_spotlight",
                abs(spun_head[0]) > 0.15 and abs(spun_head[1]) < 0.08,
                f"rest={head_rest}, spun={spun_head}",
            )
        ctx.expect_contact(ceiling_mount, rotor, contact_tol=0.0015, name="rotor_contact_after_spin")

    with ctx.pose({arm_pivot: -0.55}):
        lowered_head = ctx.part_world_position(spotlight_head)
        if head_rest is None or lowered_head is None:
            ctx.fail("arm_pivot_lowers_head", "missing head positions for arm check")
        else:
            ctx.check(
                "arm_pivot_lowers_head",
                lowered_head[2] < head_rest[2] - 0.020,
                f"rest={head_rest}, lowered={lowered_head}",
            )
        ctx.expect_contact(rotor, spotlight_arm, contact_tol=0.0015, name="arm_contact_in_pivot_pose")
        ctx.expect_gap(
            rotor,
            spotlight_head,
            axis="z",
            min_gap=0.006,
            name="head_clears_rotor_when_lowered",
        )

    rest_head_shell = ctx.part_element_world_aabb(spotlight_head, elem="head_shell")
    with ctx.pose({head_tilt: -0.65}):
        tilted_head_shell = ctx.part_element_world_aabb(spotlight_head, elem="head_shell")
        if rest_head_shell is None or tilted_head_shell is None:
            ctx.fail("head_tilt_changes_aim", "head shell AABB unavailable")
        else:
            ctx.check(
                "head_tilt_changes_aim",
                tilted_head_shell[0][2] < rest_head_shell[0][2] - 0.010,
                f"rest={rest_head_shell}, tilted={tilted_head_shell}",
            )
        ctx.expect_contact(
            spotlight_arm,
            spotlight_head,
            contact_tol=0.0015,
            name="head_contact_in_tilt_pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
