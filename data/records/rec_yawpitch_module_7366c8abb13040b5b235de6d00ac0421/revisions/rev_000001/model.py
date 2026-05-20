from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import radians

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


BASE_FOOT_RADIUS = 0.095
BASE_FOOT_HEIGHT = 0.020
BASE_HOUSING_RADIUS = 0.066
BASE_HOUSING_HEIGHT = 0.040
BASE_TOTAL_HEIGHT = BASE_FOOT_HEIGHT + BASE_HOUSING_HEIGHT

TURNTABLE_RADIUS = 0.058
TURNTABLE_THICKNESS = 0.012

FORK_DEPTH = 0.038
FORK_ARM_THICKNESS = 0.026
FORK_INNER_SPAN = 0.080
FORK_OUTER_SPAN = FORK_INNER_SPAN + 2.0 * FORK_ARM_THICKNESS
FORK_BRIDGE_HEIGHT = 0.030
FORK_ARM_HEIGHT = 0.145
FORK_ARM_CENTER_Y = 0.5 * FORK_INNER_SPAN + 0.5 * FORK_ARM_THICKNESS
PITCH_AXIS_Z = 0.112

CRADLE_PLATE_THICKNESS = 0.012
CRADLE_PLATE_WIDTH = 0.068
CRADLE_PLATE_HEIGHT = 0.055
CRADLE_BODY_DEPTH = 0.030
CRADLE_BODY_WIDTH = 0.044
CRADLE_BODY_HEIGHT = 0.042
PAD_DEPTH = 0.016
PAD_THICKNESS = 0.5 * (FORK_INNER_SPAN - CRADLE_PLATE_WIDTH)
PAD_HEIGHT = 0.030
PAD_CENTER_Y = 0.5 * CRADLE_PLATE_WIDTH + 0.5 * PAD_THICKNESS


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_axis_gimbal_module")

    base_mat = model.material("base_gray", rgba=(0.22, 0.24, 0.27, 1.0))
    fork_mat = model.material("fork_silver", rgba=(0.72, 0.75, 0.78, 1.0))
    cradle_mat = model.material("cradle_black", rgba=(0.10, 0.11, 0.12, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=BASE_FOOT_RADIUS, length=BASE_FOOT_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, 0.5 * BASE_FOOT_HEIGHT)),
        material=base_mat,
        name="foot",
    )
    base.visual(
        Cylinder(radius=BASE_HOUSING_RADIUS, length=BASE_HOUSING_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_FOOT_HEIGHT + 0.5 * BASE_HOUSING_HEIGHT)),
        material=base_mat,
        name="housing",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=BASE_HOUSING_RADIUS, length=BASE_TOTAL_HEIGHT),
        mass=1.25,
        origin=Origin(xyz=(0.0, 0.0, 0.5 * BASE_TOTAL_HEIGHT)),
    )

    fork = model.part("fork")
    fork.visual(
        Cylinder(radius=TURNTABLE_RADIUS, length=TURNTABLE_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, 0.5 * TURNTABLE_THICKNESS)),
        material=fork_mat,
        name="turntable",
    )
    fork.visual(
        Box((FORK_DEPTH, FORK_OUTER_SPAN, FORK_BRIDGE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, TURNTABLE_THICKNESS + 0.5 * FORK_BRIDGE_HEIGHT)),
        material=fork_mat,
        name="bridge",
    )
    fork.visual(
        Box((FORK_DEPTH, FORK_ARM_THICKNESS, FORK_ARM_HEIGHT)),
        origin=Origin(xyz=(0.0, FORK_ARM_CENTER_Y, TURNTABLE_THICKNESS + 0.5 * FORK_ARM_HEIGHT)),
        material=fork_mat,
        name="left_arm",
    )
    fork.visual(
        Box((FORK_DEPTH, FORK_ARM_THICKNESS, FORK_ARM_HEIGHT)),
        origin=Origin(xyz=(0.0, -FORK_ARM_CENTER_Y, TURNTABLE_THICKNESS + 0.5 * FORK_ARM_HEIGHT)),
        material=fork_mat,
        name="right_arm",
    )
    fork.inertial = Inertial.from_geometry(
        Box((FORK_DEPTH, FORK_OUTER_SPAN, FORK_ARM_HEIGHT + TURNTABLE_THICKNESS)),
        mass=0.58,
        origin=Origin(xyz=(0.0, 0.0, 0.5 * (FORK_ARM_HEIGHT + TURNTABLE_THICKNESS))),
    )

    cradle = model.part("cradle")
    cradle.visual(
        Box((CRADLE_PLATE_THICKNESS, CRADLE_PLATE_WIDTH, CRADLE_PLATE_HEIGHT)),
        origin=Origin(),
        material=cradle_mat,
        name="faceplate",
    )
    cradle.visual(
        Box((CRADLE_BODY_DEPTH, CRADLE_BODY_WIDTH, CRADLE_BODY_HEIGHT)),
        origin=Origin(xyz=(-0.5 * (CRADLE_BODY_DEPTH - CRADLE_PLATE_THICKNESS), 0.0, 0.0)),
        material=cradle_mat,
        name="body",
    )
    cradle.visual(
        Box((PAD_DEPTH, PAD_THICKNESS, PAD_HEIGHT)),
        origin=Origin(xyz=(0.0, PAD_CENTER_Y, 0.0)),
        material=cradle_mat,
        name="left_pad",
    )
    cradle.visual(
        Box((PAD_DEPTH, PAD_THICKNESS, PAD_HEIGHT)),
        origin=Origin(xyz=(0.0, -PAD_CENTER_Y, 0.0)),
        material=cradle_mat,
        name="right_pad",
    )
    cradle.inertial = Inertial.from_geometry(
        Box((CRADLE_BODY_DEPTH, FORK_INNER_SPAN, CRADLE_PLATE_HEIGHT)),
        mass=0.22,
        origin=Origin(xyz=(-0.006, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_fork_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=fork,
        origin=Origin(xyz=(0.0, 0.0, BASE_TOTAL_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-radians(160.0),
            upper=radians(160.0),
        ),
    )

    model.articulation(
        "fork_to_cradle_pitch",
        ArticulationType.REVOLUTE,
        parent=fork,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, PITCH_AXIS_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.5,
            lower=-radians(70.0),
            upper=radians(95.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    fork = object_model.get_part("fork")
    cradle = object_model.get_part("cradle")
    yaw = object_model.get_articulation("base_to_fork_yaw")
    pitch = object_model.get_articulation("fork_to_cradle_pitch")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        cradle,
        fork,
        elem_a="left_pad",
        elem_b="left_arm",
        reason="Pitch bearing pad remains in intended flush sliding contact; rotated box-vs-box narrow phase produces tiny numerical penetrations.",
    )
    ctx.allow_overlap(
        cradle,
        fork,
        elem_a="right_pad",
        elem_b="right_arm",
        reason="Pitch bearing pad remains in intended flush sliding contact; rotated box-vs-box narrow phase produces tiny numerical penetrations.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_isolated_parts(max_pose_samples=12, name="sampled_no_floating")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=24,
        ignore_adjacent=False,
        ignore_fixed=True,
    )

    ctx.check(
        "yaw_axis_vertical",
        tuple(round(v, 6) for v in yaw.axis) == (0.0, 0.0, 1.0),
        details=f"expected vertical yaw axis, got {yaw.axis}",
    )
    ctx.check(
        "pitch_axis_horizontal",
        tuple(round(v, 6) for v in pitch.axis) == (0.0, 1.0, 0.0),
        details=f"expected lateral pitch axis, got {pitch.axis}",
    )

    def check_pitch_support(prefix: str) -> None:
        ctx.expect_contact(
            fork,
            cradle,
            elem_a="left_arm",
            elem_b="left_pad",
            name=f"{prefix}_left_pad_contact",
        )
        ctx.expect_contact(
            fork,
            cradle,
            elem_a="right_arm",
            elem_b="right_pad",
            name=f"{prefix}_right_pad_contact",
        )

    with ctx.pose({yaw: 0.0, pitch: 0.0}):
        ctx.expect_gap(
            fork,
            base,
            axis="z",
            positive_elem="turntable",
            negative_elem="housing",
            min_gap=0.0,
            max_gap=0.0005,
            max_penetration=0.0,
            name="turntable_seats_on_base",
        )
        check_pitch_support("neutral")
        ctx.expect_within(
            cradle,
            fork,
            axes="y",
            margin=0.0,
            name="cradle_stays_between_fork_arms",
            inner_elem="faceplate",
        )

    yaw_limits = yaw.motion_limits
    if yaw_limits is not None and yaw_limits.lower is not None and yaw_limits.upper is not None:
        with ctx.pose({yaw: yaw_limits.lower, pitch: 0.0}):
            ctx.fail_if_parts_overlap_in_current_pose(name="yaw_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="yaw_lower_no_floating")
            ctx.expect_gap(
                fork,
                base,
                axis="z",
                positive_elem="turntable",
                negative_elem="housing",
                min_gap=0.0,
                max_gap=0.0005,
                max_penetration=0.0,
                name="yaw_lower_turntable_contact",
            )
            check_pitch_support("yaw_lower")
        with ctx.pose({yaw: yaw_limits.upper, pitch: 0.0}):
            ctx.fail_if_parts_overlap_in_current_pose(name="yaw_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="yaw_upper_no_floating")
            ctx.expect_gap(
                fork,
                base,
                axis="z",
                positive_elem="turntable",
                negative_elem="housing",
                min_gap=0.0,
                max_gap=0.0005,
                max_penetration=0.0,
                name="yaw_upper_turntable_contact",
            )
            check_pitch_support("yaw_upper")

    pitch_limits = pitch.motion_limits
    if pitch_limits is not None and pitch_limits.lower is not None and pitch_limits.upper is not None:
        with ctx.pose({yaw: 0.0, pitch: pitch_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="pitch_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="pitch_lower_no_floating")
            check_pitch_support("pitch_lower")
        with ctx.pose({yaw: 0.0, pitch: pitch_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="pitch_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="pitch_upper_no_floating")
            check_pitch_support("pitch_upper")

    if (
        yaw_limits is not None
        and yaw_limits.upper is not None
        and pitch_limits is not None
        and pitch_limits.lower is not None
    ):
        with ctx.pose({yaw: 0.65 * yaw_limits.upper, pitch: 0.85 * pitch_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="combined_pose_no_overlap")
            ctx.fail_if_isolated_parts(name="combined_pose_no_floating")
            check_pitch_support("combined")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
