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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _add_box(part, size, xyz, *, material, name):
    return part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _add_y_cylinder(part, radius, length, xyz, *, material, name):
    return part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_service_robotic_leg")

    shell_dark = model.material("shell_dark", rgba=(0.23, 0.25, 0.28, 1.0))
    structure = model.material("structure", rgba=(0.35, 0.38, 0.42, 1.0))
    service_panel = model.material("service_panel", rgba=(0.55, 0.58, 0.62, 1.0))
    wear_pad = model.material("wear_pad", rgba=(0.14, 0.15, 0.16, 1.0))

    hip_carriage = model.part("hip_carriage")
    _add_box(
        hip_carriage,
        (0.18, 0.18, 0.065),
        (0.0, 0.0, 0.0975),
        material=structure,
        name="mount_block",
    )
    _add_box(
        hip_carriage,
        (0.10, 0.16, 0.085),
        (-0.045, 0.0, 0.085),
        material=shell_dark,
        name="actuator_pack",
    )
    _add_box(
        hip_carriage,
        (0.065, 0.16, 0.07),
        (0.055, 0.0, 0.075),
        material=shell_dark,
        name="front_gusset",
    )
    _add_box(
        hip_carriage,
        (0.10, 0.025, 0.15),
        (0.0, 0.0675, -0.005),
        material=structure,
        name="left_hip_cheek",
    )
    _add_box(
        hip_carriage,
        (0.10, 0.025, 0.15),
        (0.0, -0.0675, -0.005),
        material=structure,
        name="right_hip_cheek",
    )
    _add_y_cylinder(
        hip_carriage,
        0.034,
        0.025,
        (0.0, 0.0675, 0.0),
        material=service_panel,
        name="left_hip_boss",
    )
    _add_y_cylinder(
        hip_carriage,
        0.034,
        0.025,
        (0.0, -0.0675, 0.0),
        material=service_panel,
        name="right_hip_boss",
    )
    _add_box(
        hip_carriage,
        (0.03, 0.11, 0.03),
        (0.065, 0.0, -0.012),
        material=structure,
        name="front_lower_tie",
    )
    _add_box(
        hip_carriage,
        (0.03, 0.11, 0.03),
        (-0.065, 0.0, -0.012),
        material=structure,
        name="rear_lower_tie",
    )
    hip_carriage.inertial = Inertial.from_geometry(
        Box((0.19, 0.18, 0.22)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
    )

    thigh_module = model.part("thigh_module")
    _add_y_cylinder(
        thigh_module,
        0.032,
        0.11,
        (0.0, 0.0, -0.005),
        material=service_panel,
        name="hip_trunnion",
    )
    _add_box(
        thigh_module,
        (0.09, 0.09, 0.055),
        (0.0, 0.0, -0.045),
        material=structure,
        name="hip_housing",
    )
    _add_box(
        thigh_module,
        (0.12, 0.012, 0.14),
        (0.0, 0.038, -0.13),
        material=shell_dark,
        name="left_upper_shell",
    )
    _add_box(
        thigh_module,
        (0.12, 0.012, 0.14),
        (0.0, -0.038, -0.13),
        material=shell_dark,
        name="right_upper_shell",
    )
    _add_box(
        thigh_module,
        (0.12, 0.012, 0.14),
        (0.0, 0.038, -0.30),
        material=shell_dark,
        name="left_lower_shell",
    )
    _add_box(
        thigh_module,
        (0.12, 0.012, 0.14),
        (0.0, -0.038, -0.30),
        material=shell_dark,
        name="right_lower_shell",
    )
    _add_box(
        thigh_module,
        (0.03, 0.086, 0.33),
        (0.045, 0.0, -0.21),
        material=structure,
        name="front_spine",
    )
    _add_box(
        thigh_module,
        (0.022, 0.086, 0.33),
        (-0.049, 0.0, -0.21),
        material=structure,
        name="rear_rail",
    )
    _add_box(
        thigh_module,
        (0.05, 0.006, 0.12),
        (-0.002, -0.047, -0.29),
        material=service_panel,
        name="service_hatch",
    )
    _add_box(
        thigh_module,
        (0.10, 0.09, 0.05),
        (0.0, 0.0, -0.343),
        material=structure,
        name="knee_bridge",
    )
    _add_box(
        thigh_module,
        (0.06, 0.022, 0.11),
        (0.0, 0.056, -0.40),
        material=structure,
        name="left_knee_cheek",
    )
    _add_box(
        thigh_module,
        (0.06, 0.022, 0.11),
        (0.0, -0.056, -0.40),
        material=structure,
        name="right_knee_cheek",
    )
    thigh_module.inertial = Inertial.from_geometry(
        Box((0.14, 0.11, 0.43)),
        mass=16.0,
        origin=Origin(xyz=(0.0, 0.0, -0.21)),
    )

    shank_module = model.part("shank_module")
    _add_y_cylinder(
        shank_module,
        0.028,
        0.09,
        (0.0, 0.0, 0.0),
        material=service_panel,
        name="knee_trunnion",
    )
    _add_box(
        shank_module,
        (0.08, 0.08, 0.05),
        (0.0, 0.0, -0.04),
        material=structure,
        name="knee_housing",
    )
    _add_box(
        shank_module,
        (0.11, 0.012, 0.13),
        (0.0, 0.034, -0.12),
        material=shell_dark,
        name="left_upper_shell",
    )
    _add_box(
        shank_module,
        (0.11, 0.012, 0.13),
        (0.0, -0.034, -0.12),
        material=shell_dark,
        name="right_upper_shell",
    )
    _add_box(
        shank_module,
        (0.11, 0.012, 0.13),
        (0.0, 0.034, -0.295),
        material=shell_dark,
        name="left_lower_shell",
    )
    _add_box(
        shank_module,
        (0.11, 0.012, 0.13),
        (0.0, -0.034, -0.295),
        material=shell_dark,
        name="right_lower_shell",
    )
    _add_box(
        shank_module,
        (0.028, 0.078, 0.32),
        (0.041, 0.0, -0.20),
        material=structure,
        name="front_spine",
    )
    _add_box(
        shank_module,
        (0.02, 0.078, 0.32),
        (-0.046, 0.0, -0.20),
        material=structure,
        name="rear_rail",
    )
    _add_box(
        shank_module,
        (0.048, 0.006, 0.11),
        (0.0, 0.043, -0.28),
        material=service_panel,
        name="service_hatch",
    )
    _add_box(
        shank_module,
        (0.09, 0.08, 0.05),
        (0.0, 0.0, -0.343),
        material=structure,
        name="ankle_bridge",
    )
    _add_box(
        shank_module,
        (0.055, 0.02, 0.10),
        (0.0, 0.05, -0.40),
        material=structure,
        name="left_ankle_cheek",
    )
    _add_box(
        shank_module,
        (0.055, 0.02, 0.10),
        (0.0, -0.05, -0.40),
        material=structure,
        name="right_ankle_cheek",
    )
    shank_module.inertial = Inertial.from_geometry(
        Box((0.13, 0.10, 0.42)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, -0.20)),
    )

    foot_module = model.part("foot_module")
    _add_y_cylinder(
        foot_module,
        0.025,
        0.08,
        (0.0, 0.0, 0.0),
        material=service_panel,
        name="ankle_trunnion",
    )
    _add_box(
        foot_module,
        (0.07, 0.07, 0.085),
        (0.0, 0.0, -0.05),
        material=structure,
        name="ankle_knuckle",
    )
    _add_box(
        foot_module,
        (0.22, 0.11, 0.045),
        (0.045, 0.0, -0.10),
        material=shell_dark,
        name="foot_deck",
    )
    _add_box(
        foot_module,
        (0.07, 0.13, 0.035),
        (0.165, 0.0, -0.083),
        material=structure,
        name="toe_guard",
    )
    _add_box(
        foot_module,
        (0.055, 0.095, 0.055),
        (-0.085, 0.0, -0.095),
        material=structure,
        name="heel_block",
    )
    _add_box(
        foot_module,
        (0.14, 0.10, 0.018),
        (0.03, 0.0, -0.060),
        material=service_panel,
        name="top_access_plate",
    )
    _add_box(
        foot_module,
        (0.18, 0.008, 0.04),
        (0.04, 0.051, -0.088),
        material=structure,
        name="left_side_rail",
    )
    _add_box(
        foot_module,
        (0.18, 0.008, 0.04),
        (0.04, -0.051, -0.088),
        material=structure,
        name="right_side_rail",
    )
    foot_module.inertial = Inertial.from_geometry(
        Box((0.31, 0.14, 0.13)),
        mass=7.0,
        origin=Origin(xyz=(0.03, 0.0, -0.08)),
    )

    sole_pad = model.part("sole_pad")
    _add_box(
        sole_pad,
        (0.30, 0.14, 0.018),
        (0.0, 0.0, -0.009),
        material=wear_pad,
        name="sole_block",
    )
    sole_pad.inertial = Inertial.from_geometry(
        Box((0.30, 0.14, 0.018)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
    )

    model.articulation(
        "hip_pitch",
        ArticulationType.REVOLUTE,
        parent=hip_carriage,
        child=thigh_module,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=280.0,
            velocity=1.6,
            lower=math.radians(-35.0),
            upper=math.radians(55.0),
        ),
    )
    model.articulation(
        "knee_pitch",
        ArticulationType.REVOLUTE,
        parent=thigh_module,
        child=shank_module,
        origin=Origin(xyz=(0.0, 0.0, -0.40)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=320.0,
            velocity=1.8,
            lower=0.0,
            upper=math.radians(95.0),
        ),
    )
    model.articulation(
        "ankle_pitch",
        ArticulationType.REVOLUTE,
        parent=shank_module,
        child=foot_module,
        origin=Origin(xyz=(0.0, 0.0, -0.40)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=2.0,
            lower=math.radians(-35.0),
            upper=math.radians(25.0),
        ),
    )
    model.articulation(
        "foot_to_sole",
        ArticulationType.FIXED,
        parent=foot_module,
        child=sole_pad,
        origin=Origin(xyz=(0.025, 0.0, -0.1225)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    hip_carriage = object_model.get_part("hip_carriage")
    thigh_module = object_model.get_part("thigh_module")
    shank_module = object_model.get_part("shank_module")
    foot_module = object_model.get_part("foot_module")
    sole_pad = object_model.get_part("sole_pad")
    hip_pitch = object_model.get_articulation("hip_pitch")
    knee_pitch = object_model.get_articulation("knee_pitch")
    ankle_pitch = object_model.get_articulation("ankle_pitch")

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

    ctx.expect_contact(hip_carriage, thigh_module, name="hip_joint_is_physically_supported")
    ctx.expect_contact(thigh_module, shank_module, name="knee_joint_is_physically_supported")
    ctx.expect_contact(shank_module, foot_module, name="ankle_joint_is_physically_supported")
    ctx.expect_gap(
        foot_module,
        sole_pad,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        name="sole_pad_seats_flush_under_foot",
    )
    ctx.expect_overlap(
        foot_module,
        sole_pad,
        axes="xy",
        min_overlap=0.12,
        name="sole_pad_covers_foot_contact_patch",
    )

    def _check_joint(
        *,
        name: str,
        joint,
        parent: str,
        child: str,
        axis: tuple[float, float, float],
        lower: float,
        upper: float,
    ) -> None:
        limits = joint.motion_limits
        ok = (
            joint.parent == parent
            and joint.child == child
            and joint.axis == axis
            and limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and math.isclose(limits.lower, lower, abs_tol=1e-9)
            and math.isclose(limits.upper, upper, abs_tol=1e-9)
        )
        ctx.check(
            name,
            ok,
            (
                f"expected {parent}->{child} axis={axis} "
                f"limits=({lower:.4f}, {upper:.4f})"
            ),
        )

    _check_joint(
        name="hip_joint_limits_are_plausible",
        joint=hip_pitch,
        parent="hip_carriage",
        child="thigh_module",
        axis=(0.0, -1.0, 0.0),
        lower=math.radians(-35.0),
        upper=math.radians(55.0),
    )
    _check_joint(
        name="knee_joint_limits_are_plausible",
        joint=knee_pitch,
        parent="thigh_module",
        child="shank_module",
        axis=(0.0, 1.0, 0.0),
        lower=0.0,
        upper=math.radians(95.0),
    )
    _check_joint(
        name="ankle_joint_limits_are_plausible",
        joint=ankle_pitch,
        parent="shank_module",
        child="foot_module",
        axis=(0.0, -1.0, 0.0),
        lower=math.radians(-35.0),
        upper=math.radians(25.0),
    )

    neutral_foot_position = ctx.part_world_position(foot_module)
    with ctx.pose({hip_pitch: math.radians(30.0)}):
        swung_forward_position = ctx.part_world_position(foot_module)
    ctx.check(
        "hip_motion_swings_the_leg_forward",
        neutral_foot_position is not None
        and swung_forward_position is not None
        and swung_forward_position[0] > neutral_foot_position[0] + 0.10
        and swung_forward_position[2] > neutral_foot_position[2] + 0.08,
        "positive hip pitch should carry the foot forward and upward",
    )

    with ctx.pose({knee_pitch: math.radians(70.0)}):
        knee_flexed_position = ctx.part_world_position(foot_module)
    ctx.check(
        "knee_motion_compacts_the_leg",
        neutral_foot_position is not None
        and knee_flexed_position is not None
        and knee_flexed_position[2] > neutral_foot_position[2] + 0.12,
        "positive knee flexion should lift the foot toward the thigh",
    )

    neutral_toe_aabb = ctx.part_element_world_aabb(foot_module, elem="toe_guard")
    with ctx.pose({ankle_pitch: math.radians(20.0)}):
        dorsiflexed_toe_aabb = ctx.part_element_world_aabb(foot_module, elem="toe_guard")
    ctx.check(
        "ankle_motion_lifts_the_toe",
        neutral_toe_aabb is not None
        and dorsiflexed_toe_aabb is not None
        and dorsiflexed_toe_aabb[1][2] > neutral_toe_aabb[1][2] + 0.025,
        "positive ankle pitch should raise the toe guard",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
