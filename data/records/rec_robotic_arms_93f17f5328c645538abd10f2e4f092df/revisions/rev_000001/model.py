from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi, radians

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
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_robotic_arm")

    base_graphite = model.material("base_graphite", rgba=(0.17, 0.18, 0.20, 1.0))
    arm_silver = model.material("arm_silver", rgba=(0.76, 0.78, 0.80, 1.0))
    cartridge_dark = model.material("cartridge_dark", rgba=(0.28, 0.30, 0.33, 1.0))
    joint_trim = model.material("joint_trim", rgba=(0.56, 0.58, 0.61, 1.0))
    grip_rubber = model.material("grip_rubber", rgba=(0.10, 0.11, 0.12, 1.0))

    def xy_section(z: float, width: float, depth: float, radius: float) -> list[tuple[float, float, float]]:
        return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius, corner_segments=6)]

    def yz_section(
        x: float,
        width: float,
        height: float,
        radius: float,
        *,
        cy: float = 0.0,
        cz: float = 0.0,
    ) -> list[tuple[float, float, float]]:
        return [(x, cy + y, cz + z) for y, z in rounded_rect_profile(width, height, radius, corner_segments=6)]

    pedestal = model.part("pedestal")
    pedestal.visual(
        Box((0.18, 0.17, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=base_graphite,
        name="footprint_plate",
    )
    pedestal_shell = section_loft(
        [
            xy_section(0.014, 0.140, 0.130, 0.018),
            xy_section(0.060, 0.118, 0.110, 0.020),
            xy_section(0.112, 0.090, 0.090, 0.016),
        ]
    )
    pedestal.visual(
        mesh_from_geometry(pedestal_shell, "pedestal_shell"),
        material=base_graphite,
        name="pedestal_shell",
    )
    pedestal.visual(
        Cylinder(radius=0.056, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.117)),
        material=cartridge_dark,
        name="turntable_plate",
    )
    pedestal.visual(
        Box((0.060, 0.010, 0.028)),
        origin=Origin(xyz=(0.0, -0.052, 0.053)),
        material=joint_trim,
        name="service_panel",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((0.18, 0.17, 0.124)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, 0.062)),
    )

    turret = model.part("turret")
    turret.visual(
        Cylinder(radius=0.054, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=cartridge_dark,
        name="base_flange",
    )
    turret.visual(
        Box((0.076, 0.068, 0.044)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=base_graphite,
        name="turret_body",
    )
    turret.visual(
        Box((0.068, 0.060, 0.040)),
        origin=Origin(xyz=(-0.004, 0.0, 0.028)),
        material=base_graphite,
        name="rear_spine",
    )
    turret.visual(
        Box((0.028, 0.090, 0.014)),
        origin=Origin(xyz=(-0.016, 0.0, 0.082)),
        material=base_graphite,
        name="shoulder_bridge",
    )
    turret.visual(
        Box((0.028, 0.012, 0.056)),
        origin=Origin(xyz=(0.0, 0.031, 0.054)),
        material=cartridge_dark,
        name="left_shoulder_cheek",
    )
    turret.visual(
        Box((0.028, 0.012, 0.056)),
        origin=Origin(xyz=(0.0, -0.031, 0.054)),
        material=cartridge_dark,
        name="right_shoulder_cheek",
    )
    turret.visual(
        Box((0.032, 0.042, 0.020)),
        origin=Origin(xyz=(-0.018, 0.0, 0.030)),
        material=joint_trim,
        name="yaw_motor_cover",
    )
    turret.inertial = Inertial.from_geometry(
        Box((0.078, 0.090, 0.098)),
        mass=1.3,
        origin=Origin(xyz=(-0.002, 0.0, 0.049)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.012, length=0.050),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=joint_trim,
        name="shoulder_hub",
    )
    upper_arm.visual(
        Box((0.030, 0.028, 0.024)),
        origin=Origin(xyz=(0.015, 0.0, 0.012)),
        material=arm_silver,
        name="shoulder_root_block",
    )
    upper_arm.visual(
        Box((0.104, 0.028, 0.034)),
        origin=Origin(xyz=(0.082, 0.0, 0.017)),
        material=arm_silver,
        name="upper_arm_shell",
    )
    upper_arm.visual(
        Box((0.092, 0.018, 0.016)),
        origin=Origin(xyz=(0.084, 0.0, 0.038)),
        material=arm_silver,
        name="upper_arm_cap",
    )
    upper_arm.visual(
        Box((0.022, 0.050, 0.012)),
        origin=Origin(xyz=(0.142, 0.0, 0.016)),
        material=arm_silver,
        name="elbow_bridge",
    )
    upper_arm.visual(
        Box((0.020, 0.032, 0.020)),
        origin=Origin(xyz=(0.136, 0.0, 0.006)),
        material=arm_silver,
        name="elbow_web",
    )
    upper_arm.visual(
        Box((0.016, 0.010, 0.038)),
        origin=Origin(xyz=(0.150, 0.020, 0.0)),
        material=cartridge_dark,
        name="left_elbow_cheek",
    )
    upper_arm.visual(
        Box((0.016, 0.010, 0.038)),
        origin=Origin(xyz=(0.150, -0.020, 0.0)),
        material=cartridge_dark,
        name="right_elbow_cheek",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.166, 0.050, 0.056)),
        mass=1.0,
        origin=Origin(xyz=(0.083, 0.0, 0.018)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.009, length=0.030),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=joint_trim,
        name="elbow_hub",
    )
    forearm.visual(
        Box((0.024, 0.024, 0.020)),
        origin=Origin(xyz=(0.012, 0.0, 0.010)),
        material=arm_silver,
        name="elbow_root_block",
    )
    forearm.visual(
        Box((0.090, 0.026, 0.030)),
        origin=Origin(xyz=(0.067, 0.0, 0.015)),
        material=arm_silver,
        name="forearm_shell",
    )
    forearm.visual(
        Box((0.078, 0.016, 0.014)),
        origin=Origin(xyz=(0.067, 0.0, 0.036)),
        material=arm_silver,
        name="forearm_cap",
    )
    forearm.visual(
        Box((0.020, 0.044, 0.012)),
        origin=Origin(xyz=(0.124, 0.0, 0.014)),
        material=arm_silver,
        name="wrist_bridge",
    )
    forearm.visual(
        Box((0.018, 0.028, 0.018)),
        origin=Origin(xyz=(0.118, 0.0, 0.004)),
        material=arm_silver,
        name="wrist_web",
    )
    forearm.visual(
        Box((0.014, 0.010, 0.034)),
        origin=Origin(xyz=(0.130, 0.018, 0.0)),
        material=cartridge_dark,
        name="left_wrist_cheek",
    )
    forearm.visual(
        Box((0.014, 0.010, 0.034)),
        origin=Origin(xyz=(0.130, -0.018, 0.0)),
        material=cartridge_dark,
        name="right_wrist_cheek",
    )
    forearm.inertial = Inertial.from_geometry(
        Box((0.148, 0.044, 0.050)),
        mass=0.8,
        origin=Origin(xyz=(0.074, 0.0, 0.014)),
    )

    wrist_housing = model.part("wrist_housing")
    wrist_housing.visual(
        Cylinder(radius=0.009, length=0.026),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=joint_trim,
        name="wrist_pitch_hub",
    )
    wrist_housing.visual(
        Box((0.046, 0.024, 0.026)),
        origin=Origin(xyz=(0.024, 0.0, 0.0)),
        material=arm_silver,
        name="wrist_shell",
    )
    wrist_housing.visual(
        Box((0.024, 0.018, 0.014)),
        origin=Origin(xyz=(0.028, 0.0, 0.018)),
        material=arm_silver,
        name="wrist_cap",
    )
    wrist_housing.visual(
        Box((0.018, 0.032, 0.018)),
        origin=Origin(xyz=(0.046, 0.0, 0.0)),
        material=arm_silver,
        name="roll_mount_block",
    )
    wrist_housing.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.052, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=cartridge_dark,
        name="roll_cartridge",
    )
    wrist_housing.inertial = Inertial.from_geometry(
        Box((0.070, 0.032, 0.044)),
        mass=0.45,
        origin=Origin(xyz=(0.035, 0.0, 0.0)),
    )

    tool_head = model.part("tool_head")
    tool_head.visual(
        Cylinder(radius=0.022, length=0.014),
        origin=Origin(xyz=(0.007, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=joint_trim,
        name="rear_roll_drum",
    )
    tool_head.visual(
        Box((0.032, 0.030, 0.034)),
        origin=Origin(xyz=(0.026, 0.0, 0.0)),
        material=base_graphite,
        name="tool_mount_block",
    )
    tool_head.visual(
        Box((0.018, 0.028, 0.016)),
        origin=Origin(xyz=(0.042, 0.0, 0.0)),
        material=base_graphite,
        name="jaw_bridge",
    )
    tool_head.visual(
        Box((0.024, 0.008, 0.012)),
        origin=Origin(xyz=(0.054, 0.013, 0.0)),
        material=base_graphite,
        name="upper_jaw",
    )
    tool_head.visual(
        Box((0.024, 0.008, 0.012)),
        origin=Origin(xyz=(0.054, -0.013, 0.0)),
        material=base_graphite,
        name="lower_jaw",
    )
    tool_head.visual(
        Box((0.010, 0.006, 0.014)),
        origin=Origin(xyz=(0.066, 0.013, 0.0)),
        material=grip_rubber,
        name="upper_pad",
    )
    tool_head.visual(
        Box((0.010, 0.006, 0.014)),
        origin=Origin(xyz=(0.066, -0.013, 0.0)),
        material=grip_rubber,
        name="lower_pad",
    )
    tool_head.inertial = Inertial.from_geometry(
        Box((0.078, 0.034, 0.040)),
        mass=0.28,
        origin=Origin(xyz=(0.039, 0.0, 0.0)),
    )

    model.articulation(
        "base_yaw",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=turret,
        origin=Origin(xyz=(0.0, 0.0, 0.124)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.4, lower=-2.9, upper=2.9),
    )
    model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent=turret,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=radians(-60.0),
            upper=radians(85.0),
        ),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.150, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=2.0,
            lower=radians(-135.0),
            upper=radians(135.0),
        ),
    )
    model.articulation(
        "wrist_pitch",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_housing,
        origin=Origin(xyz=(0.130, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.4,
            lower=radians(-115.0),
            upper=radians(115.0),
        ),
    )
    model.articulation(
        "wrist_roll",
        ArticulationType.REVOLUTE,
        parent=wrist_housing,
        child=tool_head,
        origin=Origin(xyz=(0.060, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=3.0,
            lower=radians(-180.0),
            upper=radians(180.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    turret = object_model.get_part("turret")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist_housing = object_model.get_part("wrist_housing")
    tool_head = object_model.get_part("tool_head")

    base_yaw = object_model.get_articulation("base_yaw")
    shoulder_pitch = object_model.get_articulation("shoulder_pitch")
    elbow_pitch = object_model.get_articulation("elbow_pitch")
    wrist_pitch = object_model.get_articulation("wrist_pitch")
    wrist_roll = object_model.get_articulation("wrist_roll")

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
    ctx.allow_overlap(
        turret,
        upper_arm,
        elem_a="turret_body",
        elem_b="shoulder_hub",
        reason="Shoulder trunnion is intentionally captured inside the base shoulder cartridge.",
    )
    ctx.allow_overlap(
        turret,
        upper_arm,
        elem_a="rear_spine",
        elem_b="shoulder_hub",
        reason="Rear shoulder support wraps around the same captured shoulder trunnion as the main turret cartridge.",
    )
    ctx.allow_overlap(
        upper_arm,
        forearm,
        elem_a="elbow_web",
        elem_b="elbow_hub",
        reason="Elbow pivot hub intentionally passes through the upper-arm clevis web.",
    )
    ctx.allow_overlap(
        forearm,
        wrist_housing,
        elem_a="wrist_web",
        elem_b="wrist_pitch_hub",
        reason="Wrist pitch hub intentionally nests inside the forearm wrist clevis.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(turret, pedestal, name="turret_seated_on_pedestal")
    ctx.expect_contact(upper_arm, turret, name="shoulder_cartridge_contact")
    ctx.expect_contact(forearm, upper_arm, name="elbow_cartridge_contact")
    ctx.expect_contact(wrist_housing, forearm, name="wrist_pitch_cartridge_contact")
    ctx.expect_contact(tool_head, wrist_housing, name="wrist_roll_cartridge_contact")

    ctx.expect_overlap(turret, pedestal, axes="xy", min_overlap=0.08, name="pedestal_support_footprint")
    ctx.expect_overlap(upper_arm, turret, axes="yz", min_overlap=0.03, name="shoulder_yoke_contains_hub")
    ctx.expect_overlap(forearm, upper_arm, axes="yz", min_overlap=0.03, name="elbow_yoke_contains_hub")
    ctx.expect_overlap(wrist_housing, forearm, axes="yz", min_overlap=0.02, name="wrist_yoke_contains_hub")

    ctx.check(
        "joint_axis_order_readable",
        base_yaw.axis == (0.0, 0.0, 1.0)
        and shoulder_pitch.axis == (0.0, -1.0, 0.0)
        and elbow_pitch.axis == (0.0, -1.0, 0.0)
        and wrist_pitch.axis == (0.0, -1.0, 0.0)
        and wrist_roll.axis == (1.0, 0.0, 0.0),
        details=(
            f"axes: base={base_yaw.axis}, shoulder={shoulder_pitch.axis}, "
            f"elbow={elbow_pitch.axis}, wrist_pitch={wrist_pitch.axis}, wrist_roll={wrist_roll.axis}"
        ),
    )

    with ctx.pose(
        {
            shoulder_pitch: radians(40.0),
            elbow_pitch: radians(-55.0),
            wrist_pitch: radians(18.0),
        }
    ):
        reach_pos = ctx.part_world_position(tool_head)
        base_pos = ctx.part_world_position(pedestal)
        ctx.check(
            "reach_pose_projects_forward",
            reach_pos is not None
            and base_pos is not None
            and reach_pos[0] > base_pos[0] + 0.22
            and reach_pos[2] > base_pos[2] + 0.12,
            details=f"reach_pos={reach_pos}, base_pos={base_pos}",
        )
        ctx.expect_gap(tool_head, pedestal, axis="z", min_gap=0.030, name="reach_pose_clears_base_height")

    with ctx.pose(
        {
            shoulder_pitch: radians(80.0),
            elbow_pitch: radians(-103.0),
            wrist_pitch: radians(-92.0),
        }
    ):
        stow_pos = ctx.part_world_position(tool_head)
        base_pos = ctx.part_world_position(pedestal)
        ctx.check(
            "stow_pose_stays_compact",
            stow_pos is not None
            and base_pos is not None
            and abs(stow_pos[0] - base_pos[0]) < 0.14
            and abs(stow_pos[1] - base_pos[1]) < 0.10
            and stow_pos[2] < base_pos[2] + 0.24
            and stow_pos[2] > base_pos[2] + 0.18,
            details=f"stow_pos={stow_pos}, base_pos={base_pos}",
        )
        ctx.expect_gap(tool_head, pedestal, axis="z", min_gap=0.020, name="stow_tool_clears_pedestal")
        ctx.expect_gap(wrist_housing, pedestal, axis="z", min_gap=0.050, name="stow_wrist_clears_pedestal")
        ctx.expect_gap(forearm, pedestal, axis="z", min_gap=0.080, name="stow_forearm_clears_pedestal")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
