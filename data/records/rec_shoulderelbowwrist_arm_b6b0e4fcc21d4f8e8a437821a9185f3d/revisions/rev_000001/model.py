from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


SHOULDER_AXIS_X = 0.03
SHOULDER_AXIS_Z = 0.38
UPPER_ARM_LENGTH = 0.320
FOREARM_LENGTH = 0.228
WRIST_BODY_LENGTH = 0.114


def add_box(
    part,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    *,
    material: str,
    name: str | None = None,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=center),
        material=material,
        name=name,
    )


def add_cylinder_y(
    part,
    radius: float,
    length: float,
    center: tuple[float, float, float],
    *,
    material: str,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(-pi * 0.5, 0.0, 0.0)),
        material=material,
        name=name,
    )


def add_cylinder_x(
    part,
    radius: float,
    length: float,
    center: tuple[float, float, float],
    *,
    material: str,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(0.0, pi * 0.5, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_backed_three_joint_arm")

    model.material("frame_graphite", rgba=(0.20, 0.22, 0.25, 1.0))
    model.material("arm_shell", rgba=(0.82, 0.84, 0.86, 1.0))
    model.material("joint_black", rgba=(0.10, 0.11, 0.13, 1.0))
    model.material("tool_steel", rgba=(0.63, 0.66, 0.70, 1.0))

    rear_frame = model.part("rear_frame")
    add_box(rear_frame, (0.18, 0.22, 0.04), (-0.06, 0.0, 0.02), material="frame_graphite", name="frame_foot")
    add_box(rear_frame, (0.07, 0.16, 0.42), (-0.095, 0.0, 0.25), material="frame_graphite", name="rear_column")
    add_box(rear_frame, (0.10, 0.16, 0.10), (-0.07, 0.0, 0.09), material="frame_graphite")
    add_box(rear_frame, (0.10, 0.12, 0.08), (-0.04, 0.0, 0.37), material="frame_graphite")
    add_box(rear_frame, (0.13, 0.10, 0.03), (-0.035, 0.0, 0.47), material="frame_graphite")
    add_box(rear_frame, (0.06, 0.10, 0.10), (0.0, 0.0, SHOULDER_AXIS_Z), material="joint_black", name="frame_structure")
    add_box(rear_frame, (0.04, 0.024, 0.12), (-0.01, 0.048, 0.41), material="frame_graphite")
    add_box(rear_frame, (0.04, 0.024, 0.12), (-0.01, -0.048, 0.41), material="frame_graphite")
    rear_frame.inertial = Inertial.from_geometry(
        Box((0.18, 0.20, 0.49)),
        mass=18.0,
        origin=Origin(xyz=(-0.02, 0.0, 0.245)),
    )

    upper_arm = model.part("upper_arm")
    add_cylinder_y(upper_arm, 0.030, 0.060, (0.03, 0.0, 0.0), material="joint_black", name="shoulder_hub")
    add_box(upper_arm, (0.080, 0.050, 0.060), (0.06, 0.0, 0.0), material="arm_shell")
    add_box(upper_arm, (0.24, 0.060, 0.080), (0.17, 0.0, 0.0), material="arm_shell", name="upper_arm_shell")
    add_box(upper_arm, (0.07, 0.050, 0.070), (0.285, 0.0, 0.0), material="arm_shell")
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.34, 0.07, 0.10)),
        mass=6.0,
        origin=Origin(xyz=(0.17, 0.0, 0.0)),
    )

    forearm = model.part("forearm")
    add_cylinder_y(forearm, 0.025, 0.050, (0.025, 0.0, 0.0), material="joint_black", name="elbow_hub")
    add_box(forearm, (0.060, 0.038, 0.050), (0.045, 0.0, 0.0), material="arm_shell")
    add_box(forearm, (0.15, 0.045, 0.060), (0.11, 0.0, -0.004), material="arm_shell", name="forearm_shell")
    add_box(forearm, (0.086, 0.040, 0.052), (0.185, 0.0, -0.002), material="arm_shell")
    add_box(forearm, (0.070, 0.030, 0.030), (0.09, 0.0, -0.020), material="arm_shell")
    forearm.inertial = Inertial.from_geometry(
        Box((0.24, 0.05, 0.08)),
        mass=3.5,
        origin=Origin(xyz=(0.11, 0.0, 0.0)),
    )

    wrist = model.part("wrist_cartridge")
    add_cylinder_y(wrist, 0.020, 0.040, (0.020, 0.0, 0.0), material="joint_black", name="wrist_body")
    add_box(wrist, (0.055, 0.050, 0.050), (0.0525, 0.0, 0.0), material="joint_black")
    add_cylinder_x(wrist, 0.026, 0.014, (0.087, 0.0, 0.0), material="tool_steel", name="tool_flange")
    add_cylinder_x(wrist, 0.010, 0.020, (0.104, 0.0, 0.0), material="tool_steel")
    wrist.inertial = Inertial.from_geometry(
        Box((0.12, 0.06, 0.07)),
        mass=1.6,
        origin=Origin(xyz=(0.06, 0.0, 0.0)),
    )

    model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent=rear_frame,
        child=upper_arm,
        origin=Origin(xyz=(SHOULDER_AXIS_X, 0.0, SHOULDER_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=160.0,
            velocity=1.4,
            lower=-1.10,
            upper=1.25,
        ),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.8,
            lower=-0.15,
            upper=2.20,
        ),
    )
    model.articulation(
        "wrist_pitch",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist,
        origin=Origin(xyz=(FOREARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=2.4,
            lower=-1.25,
            upper=1.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rear_frame = object_model.get_part("rear_frame")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist = object_model.get_part("wrist_cartridge")
    wrist.get_visual("tool_flange")

    shoulder = object_model.get_articulation("shoulder_pitch")
    elbow = object_model.get_articulation("elbow_pitch")
    wrist_joint = object_model.get_articulation("wrist_pitch")

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

    ctx.expect_contact(upper_arm, rear_frame, name="shoulder_pin_bears_upper_arm")
    ctx.expect_contact(forearm, upper_arm, name="elbow_pin_bears_forearm")
    ctx.expect_contact(wrist, forearm, name="wrist_pin_bears_cartridge")

    axes_ok = all(
        tuple(round(component, 6) for component in joint.axis) == (0.0, -1.0, 0.0)
        for joint in (shoulder, elbow, wrist_joint)
    )
    ctx.check(
        "all_revolute_axes_pitch_upward",
        axes_ok,
        details=f"axes={[shoulder.axis, elbow.axis, wrist_joint.axis]}",
    )

    with ctx.pose(shoulder_pitch=0.0, elbow_pitch=0.0, wrist_pitch=0.0):
        upper_arm_aabb = ctx.part_world_aabb(upper_arm)
        forearm_aabb = ctx.part_world_aabb(forearm)
        upper_arm_length = None if upper_arm_aabb is None else upper_arm_aabb[1][0] - upper_arm_aabb[0][0]
        forearm_length = None if forearm_aabb is None else forearm_aabb[1][0] - forearm_aabb[0][0]
        ctx.check(
            "upper_arm_longer_than_forearm",
            (
                upper_arm_length is not None
                and forearm_length is not None
                and upper_arm_length > forearm_length + 0.05
            ),
            details=f"upper_arm_length={upper_arm_length}, forearm_length={forearm_length}",
        )
        ctx.expect_origin_gap(
            wrist,
            rear_frame,
            axis="x",
            min_gap=0.52,
            name="straight_arm_reaches_forward_of_frame",
        )
        forearm_rest_pos = ctx.part_world_position(forearm)
        wrist_rest_flange = ctx.part_element_world_aabb(wrist, elem="tool_flange")

    with ctx.pose(shoulder_pitch=0.95, elbow_pitch=0.0, wrist_pitch=0.0):
        forearm_raised_pos = ctx.part_world_position(forearm)
        ctx.check(
            "positive_shoulder_motion_lifts_elbow_joint",
            (
                forearm_rest_pos is not None
                and forearm_raised_pos is not None
                and forearm_raised_pos[2] > forearm_rest_pos[2] + 0.12
            ),
            details=f"rest={forearm_rest_pos}, raised={forearm_raised_pos}",
        )

    with ctx.pose(shoulder_pitch=0.0, elbow_pitch=1.10, wrist_pitch=0.0):
        wrist_folded_pos = ctx.part_world_position(wrist)
        ctx.check(
            "positive_elbow_motion_lifts_wrist_joint",
            (
                forearm_rest_pos is not None
                and wrist_folded_pos is not None
                and wrist_folded_pos[2] > forearm_rest_pos[2] + 0.08
            ),
            details=f"elbow_pose_wrist={wrist_folded_pos}, forearm_rest={forearm_rest_pos}",
        )

    def aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lower[index] + upper[index]) * 0.5 for index in range(3))

    with ctx.pose(shoulder_pitch=0.0, elbow_pitch=0.0, wrist_pitch=0.85):
        wrist_lifted_flange = ctx.part_element_world_aabb(wrist, elem="tool_flange")
        rest_center = aabb_center(wrist_rest_flange)
        lifted_center = aabb_center(wrist_lifted_flange)
        ctx.check(
            "positive_wrist_motion_lifts_tool_flange",
            (
                rest_center is not None
                and lifted_center is not None
                and lifted_center[2] > rest_center[2] + 0.015
            ),
            details=f"rest_center={rest_center}, lifted_center={lifted_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
