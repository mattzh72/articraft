from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BASE_PLATE_SIZE = (0.18, 0.12, 0.016)
COLUMN_SIZE = (0.060, 0.060, 0.300)

SHOULDER_AXIS_X = 0.040
SHOULDER_AXIS_Z = 0.346
SHOULDER_GAP = 0.036
SHOULDER_EAR_THICKNESS = 0.010
SHOULDER_EAR_HEIGHT = 0.070
SHOULDER_REACH = 0.235

ELBOW_GAP = 0.032
ELBOW_EAR_THICKNESS = 0.010
ELBOW_EAR_HEIGHT = 0.060
ELBOW_REACH = 0.185

WRIST_GAP = 0.022
WRIST_CHEEK_THICKNESS = 0.010
WRIST_CHEEK_HEIGHT = 0.058


def _add_box(part, name: str, size: tuple[float, float, float], xyz, material) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cantilever_support_arm")

    base_color = model.material("base_paint", color=(0.18, 0.18, 0.20, 1.0))
    arm_color = model.material("arm_paint", color=(0.24, 0.33, 0.45, 1.0))
    steel_color = model.material("steel", color=(0.63, 0.65, 0.68, 1.0))
    plate_color = model.material("wrist_plate_finish", color=(0.72, 0.74, 0.77, 1.0))

    base = model.part("base_column")
    _add_box(
        base,
        "base_plate",
        BASE_PLATE_SIZE,
        (0.0, 0.0, BASE_PLATE_SIZE[2] / 2.0),
        base_color,
    )
    _add_box(
        base,
        "column",
        COLUMN_SIZE,
        (0.0, 0.0, BASE_PLATE_SIZE[2] + COLUMN_SIZE[2] / 2.0),
        base_color,
    )
    _add_box(
        base,
        "shoulder_backbone",
        (0.040, 0.028, 0.060),
        (-0.020, 0.0, SHOULDER_AXIS_Z),
        base_color,
    )
    _add_box(
        base,
        "left_shoulder_bridge",
        (0.040, 0.010, 0.034),
        (0.000, SHOULDER_GAP / 2.0 - 0.002, SHOULDER_AXIS_Z),
        steel_color,
    )
    _add_box(
        base,
        "right_shoulder_bridge",
        (0.040, 0.010, 0.034),
        (0.000, -(SHOULDER_GAP / 2.0 - 0.002), SHOULDER_AXIS_Z),
        steel_color,
    )
    _add_box(
        base,
        "left_shoulder_ear",
        (0.020, SHOULDER_EAR_THICKNESS, SHOULDER_EAR_HEIGHT),
        (SHOULDER_AXIS_X, SHOULDER_GAP / 2.0 + SHOULDER_EAR_THICKNESS / 2.0, SHOULDER_AXIS_Z),
        steel_color,
    )
    _add_box(
        base,
        "right_shoulder_ear",
        (0.020, SHOULDER_EAR_THICKNESS, SHOULDER_EAR_HEIGHT),
        (SHOULDER_AXIS_X, -(SHOULDER_GAP / 2.0 + SHOULDER_EAR_THICKNESS / 2.0), SHOULDER_AXIS_Z),
        steel_color,
    )

    shoulder = model.part("shoulder_link")
    _add_box(
        shoulder,
        "root_tongue",
        (0.020, SHOULDER_GAP, 0.040),
        (0.0, 0.0, 0.0),
        steel_color,
    )
    _add_box(
        shoulder,
        "main_beam",
        (0.190, 0.024, 0.044),
        (0.105, 0.0, 0.0),
        arm_color,
    )
    _add_box(
        shoulder,
        "distal_block",
        (0.040, 0.028, 0.044),
        (0.200, 0.0, 0.0),
        arm_color,
    )
    _add_box(
        shoulder,
        "left_elbow_bridge",
        (0.028, 0.006, 0.032),
        (0.219, ELBOW_GAP / 2.0 - 0.001, 0.0),
        steel_color,
    )
    _add_box(
        shoulder,
        "right_elbow_bridge",
        (0.028, 0.006, 0.032),
        (0.219, -(ELBOW_GAP / 2.0 - 0.001), 0.0),
        steel_color,
    )
    _add_box(
        shoulder,
        "left_elbow_ear",
        (0.016, ELBOW_EAR_THICKNESS, ELBOW_EAR_HEIGHT),
        (SHOULDER_REACH, ELBOW_GAP / 2.0 + ELBOW_EAR_THICKNESS / 2.0, 0.0),
        steel_color,
    )
    _add_box(
        shoulder,
        "right_elbow_ear",
        (0.016, ELBOW_EAR_THICKNESS, ELBOW_EAR_HEIGHT),
        (SHOULDER_REACH, -(ELBOW_GAP / 2.0 + ELBOW_EAR_THICKNESS / 2.0), 0.0),
        steel_color,
    )

    elbow = model.part("elbow_link")
    _add_box(
        elbow,
        "root_tongue",
        (0.010, ELBOW_GAP, 0.034),
        (0.005, 0.0, 0.0),
        steel_color,
    )
    _add_box(
        elbow,
        "main_beam",
        (0.150, 0.020, 0.036),
        (0.084, 0.0, 0.0),
        arm_color,
    )
    _add_box(
        elbow,
        "wrist_block",
        (0.028, 0.024, 0.036),
        (0.157, 0.0, 0.0),
        arm_color,
    )
    _add_box(
        elbow,
        "left_wrist_cheek",
        (0.030, WRIST_CHEEK_THICKNESS, WRIST_CHEEK_HEIGHT),
        (ELBOW_REACH, WRIST_GAP / 2.0 + WRIST_CHEEK_THICKNESS / 2.0, 0.0),
        steel_color,
    )
    _add_box(
        elbow,
        "right_wrist_cheek",
        (0.030, WRIST_CHEEK_THICKNESS, WRIST_CHEEK_HEIGHT),
        (ELBOW_REACH, -(WRIST_GAP / 2.0 + WRIST_CHEEK_THICKNESS / 2.0), 0.0),
        steel_color,
    )

    wrist = model.part("wrist_plate")
    _add_box(
        wrist,
        "root_tongue",
        (0.022, WRIST_GAP, 0.030),
        (0.0, 0.0, 0.0),
        steel_color,
    )
    _add_box(
        wrist,
        "plate_neck",
        (0.036, 0.020, 0.018),
        (0.020, 0.0, 0.0),
        plate_color,
    )
    _add_box(
        wrist,
        "mounting_plate",
        (0.014, 0.090, 0.075),
        (0.043, 0.0, 0.0),
        plate_color,
    )

    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent=base,
        child=shoulder,
        origin=Origin(xyz=(SHOULDER_AXIS_X, 0.0, SHOULDER_AXIS_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.5,
            lower=-0.40,
            upper=0.85,
        ),
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=shoulder,
        child=elbow,
        origin=Origin(xyz=(SHOULDER_REACH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=1.8,
            lower=-0.95,
            upper=0.95,
        ),
    )
    model.articulation(
        "wrist_joint",
        ArticulationType.REVOLUTE,
        parent=elbow,
        child=wrist,
        origin=Origin(xyz=(ELBOW_REACH, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=2.5,
            lower=-math.pi / 2.0,
            upper=math.pi / 2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_column")
    shoulder = object_model.get_part("shoulder_link")
    elbow = object_model.get_part("elbow_link")
    wrist = object_model.get_part("wrist_plate")

    shoulder_joint = object_model.get_articulation("shoulder_joint")
    elbow_joint = object_model.get_articulation("elbow_joint")
    wrist_joint = object_model.get_articulation("wrist_joint")

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
        "shoulder_joint_axis",
        tuple(shoulder_joint.axis) == (0.0, 1.0, 0.0),
        details=f"expected shoulder axis (0, 1, 0), got {shoulder_joint.axis}",
    )
    ctx.check(
        "elbow_joint_axis",
        tuple(elbow_joint.axis) == (0.0, 1.0, 0.0),
        details=f"expected elbow axis (0, 1, 0), got {elbow_joint.axis}",
    )
    ctx.check(
        "wrist_joint_axis",
        tuple(wrist_joint.axis) == (1.0, 0.0, 0.0),
        details=f"expected wrist axis (1, 0, 0), got {wrist_joint.axis}",
    )

    with ctx.pose({shoulder_joint: 0.0, elbow_joint: 0.0, wrist_joint: 0.0}):
        ctx.expect_contact(
            shoulder,
            base,
            name="shoulder_joint_contact",
        )
        ctx.expect_contact(
            elbow,
            shoulder,
            name="elbow_joint_contact",
        )
        ctx.expect_contact(
            wrist,
            elbow,
            name="wrist_joint_contact",
        )
        ctx.expect_origin_gap(
            shoulder,
            base,
            axis="z",
            min_gap=0.34,
            max_gap=0.35,
            name="shoulder_axis_height",
        )
        ctx.expect_origin_gap(
            elbow,
            shoulder,
            axis="x",
            min_gap=0.23,
            max_gap=0.24,
            name="shoulder_link_reach",
        )
        ctx.expect_origin_gap(
            wrist,
            elbow,
            axis="x",
            min_gap=0.18,
            max_gap=0.19,
            name="elbow_link_reach",
        )
        ctx.expect_overlap(
            wrist,
            elbow,
            axes="yz",
            min_overlap=0.040,
            name="wrist_plate_reads_as_tool_mount",
        )

    for joint, prefix, child, parent in (
        (shoulder_joint, "shoulder_joint", shoulder, base),
        (elbow_joint, "elbow_joint", elbow, shoulder),
        (wrist_joint, "wrist_joint", wrist, elbow),
    ):
        limits = joint.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({joint: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{prefix}_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{prefix}_lower_no_floating")
                ctx.expect_contact(
                    child,
                    parent,
                    name=f"{prefix}_lower_joint_contact",
                )
            with ctx.pose({joint: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{prefix}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{prefix}_upper_no_floating")
                ctx.expect_contact(
                    child,
                    parent,
                    name=f"{prefix}_upper_joint_contact",
                )

    ctx.fail_if_articulation_overlaps(
        max_pose_samples=32,
        name="sampled_adjacent_articulations_clear",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=32,
        ignore_adjacent=True,
        ignore_fixed=True,
    )
    ctx.fail_if_isolated_parts(max_pose_samples=16, name="sampled_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
