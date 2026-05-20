from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, cos, hypot, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


TURN_STEP = 2.0 * pi / 3.0


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _add_rotor_arm_set(part, *, arm_metal, grip_rubber) -> None:
    for index, angle in enumerate((0.0, TURN_STEP, 2.0 * TURN_STEP)):
        dx = cos(angle)
        dy = sin(angle)
        part.visual(
            Cylinder(radius=0.011, length=0.040),
            origin=Origin(
                xyz=(0.020 * dx, 0.020 * dy, 0.0),
                rpy=(0.0, pi / 2.0, angle),
            ),
            material=arm_metal,
            name=f"arm_root_{index}",
        )
        part.visual(
            Cylinder(radius=0.009, length=0.156),
            origin=Origin(
                xyz=(0.078 * dx, 0.078 * dy, 0.0),
                rpy=(0.0, pi / 2.0, angle),
            ),
            material=arm_metal,
            name=f"arm_tube_{index}",
        )
        part.visual(
            Cylinder(radius=0.0115, length=0.046),
            origin=Origin(
                xyz=(0.136 * dx, 0.136 * dy, 0.0),
                rpy=(0.0, pi / 2.0, angle),
            ),
            material=grip_rubber,
            name=f"grip_sleeve_{index}",
        )
        part.visual(
            Sphere(radius=0.013),
            origin=Origin(xyz=(0.166 * dx, 0.166 * dy, 0.0)),
            material=grip_rubber,
            name=f"end_bumper_{index}",
        )


def _element_center(ctx: TestContext, part, elem: str) -> tuple[float, float, float]:
    aabb = ctx.part_element_world_aabb(part, elem=elem)
    if aabb is None:
        raise ValueError(f"missing AABB for {part.name}:{elem}")
    return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))


def _angle_delta(target: float, reference: float) -> float:
    return ((target - reference + pi) % (2.0 * pi)) - pi


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_turnstile_gate")

    housing_gray = model.material("housing_gray", rgba=(0.24, 0.26, 0.28, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.16, 0.17, 0.18, 1.0))
    stainless = model.material("stainless", rgba=(0.74, 0.77, 0.80, 1.0))
    grip_rubber = model.material("grip_rubber", rgba=(0.09, 0.09, 0.10, 1.0))
    foot_rubber = model.material("foot_rubber", rgba=(0.12, 0.12, 0.12, 1.0))

    base_frame = model.part("base_frame")
    base_frame.inertial = Inertial.from_geometry(
        Box((0.22, 0.18, 0.36)),
        mass=3.0,
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
    )

    base_shell = ExtrudeGeometry.centered(
        rounded_rect_profile(0.22, 0.18, 0.022, corner_segments=8),
        0.016,
        cap=True,
        closed=True,
    )
    base_frame.visual(
        _save_mesh("desktop_turnstile_base_shell", base_shell),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=housing_gray,
        name="base_shell",
    )
    base_frame.visual(
        Box((0.108, 0.088, 0.040)),
        origin=Origin(xyz=(-0.036, 0.0, 0.036)),
        material=dark_gray,
        name="service_pod",
    )
    base_frame.visual(
        Cylinder(radius=0.024, length=0.214),
        origin=Origin(xyz=(0.0, 0.0, 0.123)),
        material=housing_gray,
        name="central_post",
    )
    base_frame.visual(
        Box((0.094, 0.064, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.244)),
        material=housing_gray,
        name="mast_shoulder",
    )

    bearing_core = LatheGeometry.from_shell_profiles(
        [
            (0.035, -0.024),
            (0.039, -0.020),
            (0.041, 0.000),
            (0.039, 0.020),
            (0.035, 0.024),
        ],
        [
            (0.029, -0.024),
            (0.029, 0.024),
        ],
        segments=56,
        start_cap="flat",
        end_cap="flat",
    )
    base_frame.visual(
        _save_mesh("desktop_turnstile_bearing_core", bearing_core),
        origin=Origin(xyz=(0.0, 0.0, 0.282)),
        material=stainless,
        name="bearing_core",
    )

    for foot_x, foot_y, foot_name in (
        (-0.070, -0.052, "foot_rear_left"),
        (-0.070, 0.052, "foot_rear_right"),
        (0.070, -0.052, "foot_front_left"),
        (0.070, 0.052, "foot_front_right"),
    ):
        base_frame.visual(
            Cylinder(radius=0.012, length=0.004),
            origin=Origin(xyz=(foot_x, foot_y, 0.002)),
            material=foot_rubber,
            name=foot_name,
        )

    rotor_assembly = model.part("rotor_assembly")
    rotor_assembly.inertial = Inertial.from_geometry(
        Box((0.36, 0.36, 0.10)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    rotor_assembly.visual(
        Cylinder(radius=0.012, length=0.062),
        origin=Origin(xyz=(0.0, 0.0, -0.021)),
        material=stainless,
        name="spindle",
    )
    rotor_assembly.visual(
        Cylinder(radius=0.034, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.019)),
        material=stainless,
        name="lower_thrust_ring",
    )
    rotor_assembly.visual(
        Cylinder(radius=0.030, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=dark_gray,
        name="hub_drum",
    )
    rotor_assembly.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=stainless,
        name="top_collar",
    )
    rotor_assembly.visual(
        Cylinder(radius=0.016, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=dark_gray,
        name="hub_cap",
    )
    _add_rotor_arm_set(rotor_assembly, arm_metal=stainless, grip_rubber=grip_rubber)

    model.articulation(
        "turnstile_rotation",
        ArticulationType.CONTINUOUS,
        parent=base_frame,
        child=rotor_assembly,
        origin=Origin(xyz=(0.0, 0.0, 0.325)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=3.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_frame = object_model.get_part("base_frame")
    rotor_assembly = object_model.get_part("rotor_assembly")
    turnstile_rotation = object_model.get_articulation("turnstile_rotation")

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

    ctx.expect_within(
        rotor_assembly,
        base_frame,
        axes="xy",
        inner_elem="spindle",
        outer_elem="bearing_core",
        margin=0.0,
        name="spindle_centered_in_bearing_core",
    )
    ctx.expect_contact(
        rotor_assembly,
        base_frame,
        elem_a="lower_thrust_ring",
        elem_b="bearing_core",
        contact_tol=1e-6,
        name="rotor_supported_by_bearing_core",
    )
    ctx.expect_gap(
        rotor_assembly,
        base_frame,
        axis="z",
        positive_elem="hub_drum",
        negative_elem="bearing_core",
        min_gap=0.010,
        max_gap=0.022,
        name="hub_drum_clears_bearing_housing",
    )
    ctx.expect_gap(
        rotor_assembly,
        base_frame,
        axis="z",
        positive_elem="arm_tube_0",
        negative_elem="bearing_core",
        min_gap=0.010,
        max_gap=0.022,
        name="arms_clear_bearing_top",
    )
    ctx.expect_gap(
        rotor_assembly,
        base_frame,
        axis="z",
        positive_elem="top_collar",
        negative_elem="bearing_core",
        min_gap=0.036,
        max_gap=0.060,
        name="top_collar_sits_above_bearing_core",
    )

    with ctx.pose({turnstile_rotation: 0.95}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_rotated_operating_pose")

    with ctx.pose({turnstile_rotation: 0.0}):
        arm_a_tip = _element_center(ctx, rotor_assembly, "end_bumper_0")
        arm_b_tip = _element_center(ctx, rotor_assembly, "end_bumper_1")
        arm_c_tip = _element_center(ctx, rotor_assembly, "end_bumper_2")

    rest_angles = sorted(atan2(point[1], point[0]) for point in (arm_a_tip, arm_b_tip, arm_c_tip))
    angle_gaps = [
        (rest_angles[(index + 1) % 3] - rest_angles[index]) % (2.0 * pi)
        for index in range(3)
    ]
    ctx.check(
        "three_arms_stay_evenly_spaced",
        max(abs(gap - TURN_STEP) for gap in angle_gaps) < 0.08,
        details=f"arm spacing gaps={angle_gaps}",
    )

    with ctx.pose({turnstile_rotation: 0.0}):
        tip_rest = _element_center(ctx, rotor_assembly, "end_bumper_0")
    with ctx.pose({turnstile_rotation: TURN_STEP}):
        tip_rotated = _element_center(ctx, rotor_assembly, "end_bumper_0")

    radius_rest = hypot(tip_rest[0], tip_rest[1])
    radius_rotated = hypot(tip_rotated[0], tip_rotated[1])
    delta_theta = _angle_delta(atan2(tip_rotated[1], tip_rotated[0]), atan2(tip_rest[1], tip_rest[0]))
    ctx.check(
        "arm_tip_follows_supported_spindle_rotation",
        abs(radius_rest - radius_rotated) < 0.003
        and abs(delta_theta - TURN_STEP) < 0.05
        and abs(tip_rest[2] - tip_rotated[2]) < 0.003,
        details=(
            f"radius_rest={radius_rest:.4f}, radius_rotated={radius_rotated:.4f}, "
            f"delta_theta={delta_theta:.4f}, z_rest={tip_rest[2]:.4f}, z_rotated={tip_rotated[2]:.4f}"
        ),
    )
    ctx.check(
        "desktop_envelope_stays_compact",
        radius_rest < 0.19 and abs(tip_rest[2] - 0.325) < 0.03,
        details=f"arm_tip_radius={radius_rest:.4f}, arm_tip_z={tip_rest[2]:.4f}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
