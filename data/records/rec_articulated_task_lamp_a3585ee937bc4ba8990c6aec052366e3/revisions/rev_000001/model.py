from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


BASE_WIDTH = 0.24
BASE_DEPTH = 0.18
BASE_THICKNESS = 0.018
BASE_RADIUS = 0.022

COLUMN_WIDTH = 0.052
COLUMN_DEPTH = 0.064
COLUMN_HEIGHT = 0.102

SHOULDER_X = -0.092
SHOULDER_Z = 0.145

FORK_GAP = 0.018
FORK_PLATE_THICKNESS = 0.006
FORK_PLATE_SPAN = 0.024
FORK_PLATE_HEIGHT = 0.040
OUTER_PLATE_OFFSET = (FORK_GAP * 0.5) + (FORK_PLATE_THICKNESS * 0.5)

LOWER_ARM_LENGTH = 0.223
UPPER_ARM_LENGTH = 0.205

ARM_HUB_RADIUS = 0.015
ARM_HUB_LENGTH = FORK_GAP
HEAD_HUB_RADIUS = 0.012
HEAD_HUB_LENGTH = 0.024
HEAD_PLATE_OFFSET = (HEAD_HUB_LENGTH * 0.5) + (FORK_PLATE_THICKNESS * 0.5)

HEAD_OUTER_RADIUS = 0.090
HEAD_INNER_RADIUS = 0.055
HEAD_RING_THICKNESS = 0.022
HEAD_CENTER_Z = -0.064


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _circle_profile(radius: float, samples: int = 48) -> list[tuple[float, float]]:
    return [
        (radius * cos((2.0 * pi * index) / samples), radius * sin((2.0 * pi * index) / samples))
        for index in range(samples)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="magnifying_work_lamp")

    cast_iron = model.material("cast_iron", rgba=(0.24, 0.24, 0.25, 1.0))
    arm_silver = model.material("arm_silver", rgba=(0.73, 0.74, 0.76, 1.0))
    steel = model.material("steel", rgba=(0.57, 0.59, 0.62, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.10, 0.10, 0.11, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.78, 0.90, 0.97, 0.38))
    diffuser_white = model.material("diffuser_white", rgba=(0.95, 0.95, 0.92, 1.0))

    base_plate_mesh = _save_mesh(
        "lamp_base_plate",
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(BASE_WIDTH, BASE_DEPTH, BASE_RADIUS, corner_segments=8),
            BASE_THICKNESS,
            cap=True,
        ),
    )
    head_ring_mesh = _save_mesh(
        "lamp_head_ring",
        ExtrudeWithHolesGeometry(
            _circle_profile(HEAD_OUTER_RADIUS, samples=56),
            [_circle_profile(HEAD_INNER_RADIUS, samples=56)],
            height=HEAD_RING_THICKNESS,
            center=True,
            cap=True,
        ),
    )
    bezel_mesh = _save_mesh(
        "lamp_inner_bezel",
        ExtrudeWithHolesGeometry(
            _circle_profile(HEAD_INNER_RADIUS + 0.010, samples=48),
            [_circle_profile(HEAD_INNER_RADIUS - 0.004, samples=48)],
            height=0.006,
            center=True,
            cap=True,
        ),
    )

    base = model.part("base")
    base.visual(base_plate_mesh, material=cast_iron, name="base_plate")
    base.visual(
        Box((0.056, COLUMN_DEPTH, COLUMN_HEIGHT)),
        origin=Origin(xyz=(SHOULDER_X - 0.016, 0.0, BASE_THICKNESS + (COLUMN_HEIGHT * 0.5))),
        material=cast_iron,
        name="column",
    )
    base.visual(
        Box((0.050, 0.050, 0.026)),
        origin=Origin(xyz=(SHOULDER_X - 0.031, 0.0, 0.108)),
        material=cast_iron,
        name="shoulder_block",
    )
    base.visual(
        Box((0.020, FORK_PLATE_THICKNESS, 0.046)),
        origin=Origin(xyz=(SHOULDER_X - 0.006, OUTER_PLATE_OFFSET, 0.143)),
        material=cast_iron,
        name="left_shoulder_plate",
    )
    base.visual(
        Box((0.020, FORK_PLATE_THICKNESS, 0.046)),
        origin=Origin(xyz=(SHOULDER_X - 0.006, -OUTER_PLATE_OFFSET, 0.143)),
        material=cast_iron,
        name="right_shoulder_plate",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_WIDTH, BASE_DEPTH, SHOULDER_Z)),
        mass=5.8,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z * 0.5)),
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        Cylinder(radius=ARM_HUB_RADIUS, length=ARM_HUB_LENGTH),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="shoulder_hub",
    )
    lower_arm.visual(
        Box((0.188, 0.020, 0.012)),
        origin=Origin(xyz=(0.109, 0.0, 0.0)),
        material=arm_silver,
        name="lower_beam",
    )
    lower_arm.visual(
        Box((0.024, 0.024, 0.020)),
        origin=Origin(xyz=(0.197, 0.0, 0.0)),
        material=arm_silver,
        name="elbow_bridge",
    )
    lower_arm.visual(
        Box((FORK_PLATE_SPAN, FORK_PLATE_THICKNESS, FORK_PLATE_HEIGHT)),
        origin=Origin(xyz=(LOWER_ARM_LENGTH - 0.002, OUTER_PLATE_OFFSET, 0.0)),
        material=arm_silver,
        name="left_elbow_plate",
    )
    lower_arm.visual(
        Box((FORK_PLATE_SPAN, FORK_PLATE_THICKNESS, FORK_PLATE_HEIGHT)),
        origin=Origin(xyz=(LOWER_ARM_LENGTH - 0.002, -OUTER_PLATE_OFFSET, 0.0)),
        material=arm_silver,
        name="right_elbow_plate",
    )
    lower_arm.inertial = Inertial.from_geometry(
        Box((LOWER_ARM_LENGTH + 0.010, 0.040, 0.040)),
        mass=1.1,
        origin=Origin(xyz=(0.112, 0.0, 0.0)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.014, length=ARM_HUB_LENGTH),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="elbow_hub",
    )
    upper_arm.visual(
        Box((0.174, 0.018, 0.011)),
        origin=Origin(xyz=(0.099, 0.0, 0.0)),
        material=arm_silver,
        name="upper_beam",
    )
    upper_arm.visual(
        Box((0.020, 0.032, 0.020)),
        origin=Origin(xyz=(0.182, 0.0, 0.0)),
        material=arm_silver,
        name="head_bridge",
    )
    upper_arm.visual(
        Box((0.016, FORK_PLATE_THICKNESS, 0.036)),
        origin=Origin(xyz=(0.194, HEAD_PLATE_OFFSET, 0.0)),
        material=arm_silver,
        name="left_head_plate",
    )
    upper_arm.visual(
        Box((0.016, FORK_PLATE_THICKNESS, 0.036)),
        origin=Origin(xyz=(0.194, -HEAD_PLATE_OFFSET, 0.0)),
        material=arm_silver,
        name="right_head_plate",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((UPPER_ARM_LENGTH + 0.010, 0.040, 0.038)),
        mass=0.9,
        origin=Origin(xyz=(0.103, 0.0, -0.004)),
    )

    lens_frame = model.part("lens_frame")
    lens_frame.visual(
        Cylinder(radius=HEAD_HUB_RADIUS, length=HEAD_HUB_LENGTH),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="tilt_trunnion",
    )
    lens_frame.visual(
        Box((0.040, 0.008, 0.012)),
        origin=Origin(xyz=(0.032, 0.016, -0.006)),
        material=steel,
        name="left_yoke_strut",
    )
    lens_frame.visual(
        Box((0.040, 0.008, 0.012)),
        origin=Origin(xyz=(0.032, -0.016, -0.006)),
        material=steel,
        name="right_yoke_strut",
    )
    lens_frame.visual(
        Box((0.024, 0.010, 0.060)),
        origin=Origin(xyz=(0.060, 0.018, -0.030)),
        material=steel,
        name="left_ring_support",
    )
    lens_frame.visual(
        Box((0.024, 0.010, 0.060)),
        origin=Origin(xyz=(0.060, -0.018, -0.030)),
        material=steel,
        name="right_ring_support",
    )
    lens_frame.visual(
        head_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, HEAD_CENTER_Z)),
        material=diffuser_white,
        name="ring_shade",
    )
    lens_frame.visual(
        bezel_mesh,
        origin=Origin(xyz=(0.0, 0.0, HEAD_CENTER_Z + 0.006)),
        material=steel,
        name="lens_bezel",
    )
    lens_frame.visual(
        Cylinder(radius=HEAD_INNER_RADIUS - 0.004, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, HEAD_CENTER_Z + 0.002)),
        material=lens_glass,
        name="magnifier_lens",
    )
    lens_frame.visual(
        Box((0.026, 0.012, 0.012)),
        origin=Origin(xyz=(HEAD_OUTER_RADIUS - 0.008, 0.0, HEAD_CENTER_Z)),
        material=dark_rubber,
        name="focus_handle",
    )
    lens_frame.inertial = Inertial.from_geometry(
        Cylinder(radius=HEAD_OUTER_RADIUS, length=0.035),
        mass=0.85,
        origin=Origin(xyz=(0.0, 0.0, HEAD_CENTER_Z)),
    )

    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_arm,
        origin=Origin(xyz=(SHOULDER_X, 0.0, SHOULDER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=0.0, upper=1.20),
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(LOWER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=2.0, lower=0.0, upper=1.55),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=lens_frame,
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.4, lower=-0.40, upper=0.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    lens_frame = object_model.get_part("lens_frame")

    shoulder_joint = object_model.get_articulation("shoulder_joint")
    elbow_joint = object_model.get_articulation("elbow_joint")
    head_tilt = object_model.get_articulation("head_tilt")

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

    ctx.expect_contact(
        lower_arm,
        base,
        elem_a="shoulder_hub",
        elem_b="left_shoulder_plate",
        name="shoulder_left_plate_contact",
    )
    ctx.expect_contact(
        lower_arm,
        base,
        elem_a="shoulder_hub",
        elem_b="right_shoulder_plate",
        name="shoulder_right_plate_contact",
    )
    ctx.expect_contact(
        upper_arm,
        lower_arm,
        elem_a="elbow_hub",
        elem_b="left_elbow_plate",
        name="elbow_left_plate_contact",
    )
    ctx.expect_contact(
        upper_arm,
        lower_arm,
        elem_a="elbow_hub",
        elem_b="right_elbow_plate",
        name="elbow_right_plate_contact",
    )
    ctx.expect_contact(
        lens_frame,
        upper_arm,
        elem_a="tilt_trunnion",
        elem_b="left_head_plate",
        name="head_left_plate_contact",
    )
    ctx.expect_contact(
        lens_frame,
        upper_arm,
        elem_a="tilt_trunnion",
        elem_b="right_head_plate",
        name="head_right_plate_contact",
    )
    ctx.expect_gap(
        lens_frame,
        base,
        axis="z",
        min_gap=0.045,
        max_gap=0.090,
        positive_elem="magnifier_lens",
        negative_elem="base_plate",
        name="lens_clear_of_base_plate",
    )
    ctx.expect_overlap(
        lens_frame,
        base,
        axes="y",
        min_overlap=0.090,
        elem_a="ring_shade",
        elem_b="base_plate",
        name="head_centered_over_base_width",
    )

    limits = shoulder_joint.motion_limits
    shoulder_axis_ok = (
        abs(shoulder_joint.axis[0]) < 1e-9
        and abs(abs(shoulder_joint.axis[1]) - 1.0) < 1e-9
        and abs(shoulder_joint.axis[2]) < 1e-9
    )
    shoulder_ok = (
        limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and abs(limits.upper - 1.20) < 1e-9
        and shoulder_axis_ok
    )
    ctx.check("shoulder_joint_axis_and_range", shoulder_ok, details=str(shoulder_joint.axis))

    limits = elbow_joint.motion_limits
    elbow_axis_ok = (
        abs(elbow_joint.axis[0]) < 1e-9
        and abs(abs(elbow_joint.axis[1]) - 1.0) < 1e-9
        and abs(elbow_joint.axis[2]) < 1e-9
    )
    elbow_ok = (
        limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and abs(limits.upper - 1.55) < 1e-9
        and elbow_axis_ok
    )
    ctx.check("elbow_joint_axis_and_range", elbow_ok, details=str(elbow_joint.axis))

    limits = head_tilt.motion_limits
    head_axis_ok = (
        abs(head_tilt.axis[0]) < 1e-9
        and abs(abs(head_tilt.axis[1]) - 1.0) < 1e-9
        and abs(head_tilt.axis[2]) < 1e-9
    )
    head_ok = (
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and abs(limits.lower + 0.40) < 1e-9
        and abs(limits.upper - 0.25) < 1e-9
        and head_axis_ok
    )
    ctx.check("head_tilt_axis_and_range", head_ok, details=str(head_tilt.axis))

    def _check_joint_extremes(joint, label: str) -> None:
        motion_limits = joint.motion_limits
        if motion_limits is None or motion_limits.lower is None or motion_limits.upper is None:
            ctx.fail(f"{label}_limits_defined", "Expected finite lower and upper motion limits.")
            return

        for pose_name, value in (("lower", motion_limits.lower), ("upper", motion_limits.upper)):
            with ctx.pose({joint: value}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{label}_{pose_name}_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{label}_{pose_name}_no_floating")
                ctx.expect_contact(
                    lower_arm,
                    base,
                    elem_a="shoulder_hub",
                    elem_b="left_shoulder_plate",
                    name=f"{label}_{pose_name}_shoulder_left_contact",
                )
                ctx.expect_contact(
                    lower_arm,
                    base,
                    elem_a="shoulder_hub",
                    elem_b="right_shoulder_plate",
                    name=f"{label}_{pose_name}_shoulder_right_contact",
                )
                ctx.expect_contact(
                    upper_arm,
                    lower_arm,
                    elem_a="elbow_hub",
                    elem_b="left_elbow_plate",
                    name=f"{label}_{pose_name}_elbow_left_contact",
                )
                ctx.expect_contact(
                    upper_arm,
                    lower_arm,
                    elem_a="elbow_hub",
                    elem_b="right_elbow_plate",
                    name=f"{label}_{pose_name}_elbow_right_contact",
                )
                ctx.expect_contact(
                    lens_frame,
                    upper_arm,
                    elem_a="tilt_trunnion",
                    elem_b="left_head_plate",
                    name=f"{label}_{pose_name}_head_left_contact",
                )
                ctx.expect_contact(
                    lens_frame,
                    upper_arm,
                    elem_a="tilt_trunnion",
                    elem_b="right_head_plate",
                    name=f"{label}_{pose_name}_head_right_contact",
                )

    _check_joint_extremes(shoulder_joint, "shoulder_joint")
    _check_joint_extremes(elbow_joint, "elbow_joint")
    _check_joint_extremes(head_tilt, "head_tilt")

    with ctx.pose({shoulder_joint: 0.90, elbow_joint: 0.45, head_tilt: -0.35}):
        ctx.fail_if_parts_overlap_in_current_pose(name="working_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="working_pose_no_floating")
        head_pos = ctx.part_world_position(lens_frame)
        base_aabb = ctx.part_world_aabb(base)
        if head_pos is None or base_aabb is None:
            ctx.fail("working_pose_measurements_available", "Could not evaluate posed lamp geometry.")
        else:
            ctx.check(
                "working_pose_head_height",
                head_pos[2] > base_aabb[1][2] + 0.18,
                details=f"head_z={head_pos[2]:.4f}, base_top={base_aabb[1][2]:.4f}",
            )
            ctx.check(
                "working_pose_forward_reach",
                head_pos[0] > 0.08,
                details=f"head_x={head_pos[0]:.4f}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
