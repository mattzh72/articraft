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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


BODY_LENGTH = 0.18
BODY_WIDTH = 0.07
BODY_CORE_HEIGHT = 0.028
BODY_TOP_HEIGHT = 0.008
BODY_TOTAL_HEIGHT = BODY_CORE_HEIGHT + BODY_TOP_HEIGHT
HINGE_ROOT_X = 0.065
HINGE_ROOT_Y = 0.039
HINGE_RADIUS = 0.012
HINGE_THICKNESS = 0.004
ARM_BEAM_LENGTH = 0.106
ARM_BEAM_WIDTH = 0.018
ARM_BEAM_HEIGHT = 0.010
MOTOR_RADIUS = 0.016
MOTOR_HEIGHT = 0.010
PROP_SPAN_LENGTH = 0.068
PROP_CENTER_STRIP_WIDTH = 0.004
PROP_BLADE_LENGTH = 0.022
PROP_BLADE_WIDTH = 0.010
PROP_BLADE_THICKNESS = 0.0016
PROP_HUB_RADIUS = 0.007
PROP_HUB_HEIGHT = 0.004
ARM_FOLD_ANGLE = math.pi / 4.0


ARM_SPECS = (
    ("front_left", 1.0, 1.0),
    ("front_right", 1.0, -1.0),
    ("rear_left", -1.0, 1.0),
    ("rear_right", -1.0, -1.0),
)


def _arm_deployed_yaw(front_sign: float, side_sign: float) -> float:
    return side_sign * (math.pi / 4.0 if front_sign > 0.0 else 3.0 * math.pi / 4.0)


def _arm_fold_delta(front_sign: float, side_sign: float) -> float:
    return front_sign * side_sign * ARM_FOLD_ANGLE


def _arm_joint_limits(front_sign: float, side_sign: float) -> MotionLimits:
    fold_delta = _arm_fold_delta(front_sign, side_sign)
    lower = min(0.0, fold_delta)
    upper = max(0.0, fold_delta)
    return MotionLimits(
        effort=3.5,
        velocity=2.0,
        lower=lower,
        upper=upper,
    )


def _xy_section(length: float, width: float, radius: float, z: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in rounded_rect_profile(length, width, radius)]


def _yz_section(
    x: float,
    width: float,
    height: float,
    z_center: float,
    radius: float,
) -> list[tuple[float, float, float]]:
    return [(x, y, z_center + z) for y, z in rounded_rect_profile(width, height, radius)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_travel_quadrotor")

    shell_gray = model.material("shell_gray", rgba=(0.42, 0.44, 0.47, 1.0))
    matte_black = model.material("matte_black", rgba=(0.12, 0.12, 0.13, 1.0))
    arm_gray = model.material("arm_gray", rgba=(0.24, 0.25, 0.27, 1.0))
    motor_dark = model.material("motor_dark", rgba=(0.10, 0.10, 0.11, 1.0))
    prop_black = model.material("prop_black", rgba=(0.06, 0.06, 0.07, 1.0))
    accent = model.material("accent", rgba=(0.72, 0.73, 0.75, 1.0))

    body_shell_mesh = mesh_from_geometry(
        section_loft(
            [
                _xy_section(0.150, 0.052, 0.012, 0.003),
                _xy_section(0.180, 0.070, 0.016, 0.011),
                _xy_section(0.170, 0.066, 0.015, 0.026),
                _xy_section(0.132, 0.050, 0.012, BODY_TOTAL_HEIGHT),
            ]
        ),
        "quadrotor_body_shell",
    )
    arm_beam_mesh = mesh_from_geometry(
        section_loft(
            [
                _yz_section(0.010, 0.020, 0.010, 0.007, 0.0035),
                _yz_section(0.056, 0.017, 0.009, 0.007, 0.0030),
                _yz_section(0.108, 0.014, 0.008, 0.007, 0.0025),
            ]
        ),
        "quadrotor_arm_beam",
    )
    prop_blade_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            [
                (-0.014, -0.0028),
                (-0.010, -0.0040),
                (0.000, -0.0047),
                (0.009, -0.0043),
                (0.014, -0.0022),
                (0.014, 0.0022),
                (0.009, 0.0043),
                (0.000, 0.0047),
                (-0.010, 0.0040),
                (-0.014, 0.0028),
            ],
            PROP_BLADE_THICKNESS,
            center=True,
        ),
        "quadrotor_prop_blade",
    )

    body = model.part("body")
    body.visual(
        body_shell_mesh,
        material=shell_gray,
        name="body_core",
    )
    body.visual(
        Box((0.140, 0.056, BODY_TOP_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BODY_CORE_HEIGHT + BODY_TOP_HEIGHT * 0.5)),
        material=matte_black,
        name="top_cover",
    )
    body.visual(
        Box((0.082, 0.034, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=matte_black,
        name="bottom_battery",
    )
    body.visual(
        Box((0.040, 0.026, 0.005)),
        origin=Origin(xyz=(0.054, 0.0, 0.0295)),
        material=matte_black,
        name="front_nose",
    )
    body.visual(
        Box((0.050, 0.012, 0.004)),
        origin=Origin(xyz=(-0.062, 0.0, 0.026)),
        material=accent,
        name="rear_heat_sink",
    )

    for prefix, front_sign, side_sign in ARM_SPECS:
        body.visual(
            Cylinder(radius=HINGE_RADIUS, length=HINGE_THICKNESS),
            origin=Origin(
                xyz=(
                    front_sign * HINGE_ROOT_X,
                    side_sign * HINGE_ROOT_Y,
                    BODY_TOTAL_HEIGHT + HINGE_THICKNESS * 0.5,
                )
            ),
            material=arm_gray,
            name=f"{prefix}_hinge_base",
        )

    body.inertial = Inertial.from_geometry(
        Box((BODY_LENGTH, BODY_WIDTH, BODY_TOTAL_HEIGHT)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, BODY_TOTAL_HEIGHT * 0.5)),
    )

    for prefix, front_sign, side_sign in ARM_SPECS:
        arm_name = f"{prefix}_arm"
        prop_name = f"{prefix}_prop"
        joint_name = f"body_to_{arm_name}"
        prop_joint_name = f"{arm_name}_to_{prop_name}"

        arm = model.part(arm_name)
        arm.visual(
            Cylinder(radius=HINGE_RADIUS, length=HINGE_THICKNESS),
            origin=Origin(xyz=(0.0, 0.0, HINGE_THICKNESS * 0.5)),
            material=arm_gray,
            name="hinge_collar",
        )
        arm.visual(
            Box((0.020, 0.024, 0.006)),
            origin=Origin(xyz=(0.018, 0.0, 0.007)),
            material=matte_black,
            name="root_block",
        )
        arm.visual(
            arm_beam_mesh,
            material=arm_gray,
            name="beam",
        )
        arm.visual(
            Cylinder(radius=MOTOR_RADIUS, length=MOTOR_HEIGHT),
            origin=Origin(xyz=(0.118, 0.0, 0.011)),
            material=motor_dark,
            name="motor_pod",
        )
        arm.visual(
            Cylinder(radius=0.009, length=0.006),
            origin=Origin(xyz=(0.118, 0.0, 0.018)),
            material=accent,
            name="motor_cap",
        )
        arm.inertial = Inertial.from_geometry(
            Box((0.140, 0.030, 0.024)),
            mass=0.10,
            origin=Origin(xyz=(0.065, 0.0, 0.012)),
        )

        prop = model.part(prop_name)
        prop.visual(
            Cylinder(radius=PROP_HUB_RADIUS, length=PROP_HUB_HEIGHT),
            origin=Origin(xyz=(0.0, 0.0, PROP_HUB_HEIGHT * 0.5)),
            material=prop_black,
            name="hub",
        )
        prop.visual(
            prop_blade_mesh,
            origin=Origin(
                xyz=(0.014, 0.0, PROP_HUB_HEIGHT * 0.5 + 0.0006),
                rpy=(0.0, 0.12, 0.0),
            ),
            material=prop_black,
            name="blade_a",
        )
        prop.visual(
            prop_blade_mesh,
            origin=Origin(
                xyz=(-0.014, 0.0, PROP_HUB_HEIGHT * 0.5 + 0.0006),
                rpy=(0.0, -0.12, math.pi),
            ),
            material=prop_black,
            name="blade_b",
        )
        prop.inertial = Inertial.from_geometry(
            Box((0.074, 0.014, 0.006)),
            mass=0.012,
            origin=Origin(xyz=(0.0, 0.0, 0.003)),
        )

        model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent=body,
            child=arm,
            origin=Origin(
                xyz=(
                    front_sign * HINGE_ROOT_X,
                    side_sign * HINGE_ROOT_Y,
                    BODY_TOTAL_HEIGHT + HINGE_THICKNESS,
                ),
                rpy=(0.0, 0.0, _arm_deployed_yaw(front_sign, side_sign)),
            ),
            axis=(0.0, 0.0, 1.0),
            motion_limits=_arm_joint_limits(front_sign, side_sign),
        )

        model.articulation(
            prop_joint_name,
            ArticulationType.CONTINUOUS,
            parent=arm,
            child=prop,
            origin=Origin(xyz=(0.118, 0.0, 0.016)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.20, velocity=45.0),
        )

    return model


def _span(aabb: tuple[tuple[float, float, float], tuple[float, float, float]], axis: str) -> float:
    axis_index = {"x": 0, "y": 1, "z": 2}[axis]
    return aabb[1][axis_index] - aabb[0][axis_index]


def _aabb_center(
    aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None,
) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    body = object_model.get_part("body")

    arm_parts = {prefix: object_model.get_part(f"{prefix}_arm") for prefix, _, _ in ARM_SPECS}
    prop_parts = {prefix: object_model.get_part(f"{prefix}_prop") for prefix, _, _ in ARM_SPECS}
    arm_joints = {
        prefix: object_model.get_articulation(f"body_to_{prefix}_arm")
        for prefix, _, _ in ARM_SPECS
    }
    prop_joints = {
        prefix: object_model.get_articulation(f"{prefix}_arm_to_{prefix}_prop")
        for prefix, _, _ in ARM_SPECS
    }

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

    for prefix, front_sign, side_sign in ARM_SPECS:
        arm = arm_parts[prefix]
        prop = prop_parts[prefix]
        arm_joint = arm_joints[prefix]
        prop_joint = prop_joints[prefix]

        ctx.expect_contact(body, arm, name=f"{prefix}_arm_hinge_contact")
        ctx.expect_contact(arm, prop, name=f"{prefix}_prop_motor_contact")

        if front_sign > 0.0:
            ctx.expect_gap(
                prop,
                body,
                axis="x",
                min_gap=0.040,
                positive_elem="hub",
                name=f"{prefix}_hub_clears_front_of_body",
            )
        else:
            ctx.expect_gap(
                body,
                prop,
                axis="x",
                min_gap=0.040,
                negative_elem="hub",
                name=f"{prefix}_hub_clears_rear_of_body",
            )

        if side_sign > 0.0:
            ctx.expect_gap(
                prop,
                body,
                axis="y",
                min_gap=0.060,
                positive_elem="hub",
                name=f"{prefix}_hub_clears_left_side_of_body",
            )
        else:
            ctx.expect_gap(
                body,
                prop,
                axis="y",
                min_gap=0.060,
                negative_elem="hub",
                name=f"{prefix}_hub_clears_right_side_of_body",
            )

        limits = arm_joint.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({arm_joint: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{prefix}_arm_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{prefix}_arm_lower_no_floating")
                ctx.expect_contact(body, arm, name=f"{prefix}_arm_lower_contact")
            with ctx.pose({arm_joint: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{prefix}_arm_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{prefix}_arm_upper_no_floating")
                ctx.expect_contact(body, arm, name=f"{prefix}_arm_upper_contact")

        rest_prop_pos = ctx.part_world_position(prop)
        with ctx.pose({arm_joint: _arm_fold_delta(front_sign, side_sign)}):
            folded_prop_pos = ctx.part_world_position(prop)
            ctx.expect_contact(body, arm, name=f"{prefix}_arm_folded_contact")
            ctx.expect_contact(arm, prop, name=f"{prefix}_prop_folded_contact")
        movement_ok = (
            rest_prop_pos is not None
            and folded_prop_pos is not None
            and abs(folded_prop_pos[2] - rest_prop_pos[2]) <= 0.002
            and abs(folded_prop_pos[0]) < abs(rest_prop_pos[0]) - 0.040
            and (
                (side_sign > 0.0 and folded_prop_pos[1] > rest_prop_pos[1] + 0.020)
                or (side_sign < 0.0 and folded_prop_pos[1] < rest_prop_pos[1] - 0.020)
            )
        )
        ctx.check(
            f"{prefix}_arm_folds_flat_along_body_side",
            movement_ok,
            details="Propeller tip did not move inward and alongside the body as expected.",
        )

        with ctx.pose({prop_joint: math.pi / 2.0}):
            ctx.expect_contact(arm, prop, name=f"{prefix}_prop_spun_contact")

    folded_pose = {
        arm_joints[prefix]: _arm_fold_delta(front_sign, side_sign)
        for prefix, front_sign, side_sign in ARM_SPECS
    }
    with ctx.pose(folded_pose):
        ctx.fail_if_parts_overlap_in_current_pose(name="all_arms_folded_no_overlap")
        ctx.fail_if_isolated_parts(name="all_arms_folded_no_floating")

        for prefix, _, side_sign in ARM_SPECS:
            arm = arm_parts[prefix]
            prop = prop_parts[prefix]
            prop_joint = prop_joints[prefix]

            ctx.expect_contact(body, arm, name=f"{prefix}_all_folded_arm_contact")
            ctx.expect_contact(arm, prop, name=f"{prefix}_all_folded_prop_contact")

            if side_sign > 0.0:
                ctx.expect_gap(
                    prop,
                    body,
                    axis="y",
                    min_gap=0.090,
                    positive_elem="hub",
                    name=f"{prefix}_folded_hub_left_of_body",
                )
            else:
                ctx.expect_gap(
                    body,
                    prop,
                    axis="y",
                    min_gap=0.090,
                    negative_elem="hub",
                    name=f"{prefix}_folded_hub_right_of_body",
                )

            rest_blade_center = _aabb_center(ctx.part_element_world_aabb(prop, elem="blade_a"))
            rest_hub_center = _aabb_center(ctx.part_element_world_aabb(prop, elem="hub"))

            spun_pose = dict(folded_pose)
            spun_pose[prop_joint] = math.pi / 2.0
            with ctx.pose(spun_pose):
                spun_blade_center = _aabb_center(ctx.part_element_world_aabb(prop, elem="blade_a"))
                spun_hub_center = _aabb_center(ctx.part_element_world_aabb(prop, elem="hub"))
                ctx.expect_contact(arm, prop, name=f"{prefix}_all_folded_prop_spun_contact")

            rotation_ok = False
            if (
                rest_blade_center is not None
                and rest_hub_center is not None
                and spun_blade_center is not None
                and spun_hub_center is not None
            ):
                rest_vec = (
                    rest_blade_center[0] - rest_hub_center[0],
                    rest_blade_center[1] - rest_hub_center[1],
                )
                spun_vec = (
                    spun_blade_center[0] - spun_hub_center[0],
                    spun_blade_center[1] - spun_hub_center[1],
                )
                rest_len = math.hypot(*rest_vec)
                spun_len = math.hypot(*spun_vec)
                dot = rest_vec[0] * spun_vec[0] + rest_vec[1] * spun_vec[1]
                cross = rest_vec[0] * spun_vec[1] - rest_vec[1] * spun_vec[0]
                rotation_ok = (
                    rest_len >= 0.010
                    and abs(rest_len - spun_len) <= 0.002
                    and abs(rest_blade_center[2] - spun_blade_center[2]) <= 0.001
                    and abs(dot) <= 0.25 * rest_len * spun_len
                    and abs(cross) >= 0.75 * rest_len * spun_len
                )
            ctx.check(
                f"{prefix}_propeller_rotates_about_vertical_axle",
                rotation_ok,
                details="Propeller blade center did not trace a quarter-turn around the motor hub.",
            )

    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=24,
        ignore_adjacent=False,
        ignore_fixed=True,
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
