# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import os

try:
    os.chdir("/")
except OSError:
    pass

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

SAFE_ASSET_ROOT = "/"


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="steel_floodlight_mast")

    galvanized_steel = model.material("galvanized_steel", rgba=(0.58, 0.60, 0.62, 1.0))
    painted_steel = model.material("painted_steel", rgba=(0.22, 0.24, 0.26, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.78, 0.84, 0.88, 0.40))

    mast = model.part("mast")
    mast.visual(
        Box((0.42, 0.42, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=galvanized_steel,
        name="base_plate",
    )
    mast.visual(
        Cylinder(radius=0.095, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=galvanized_steel,
        name="base_collar",
    )
    mast.visual(
        Cylinder(radius=0.08, length=5.40),
        origin=Origin(xyz=(0.0, 0.0, 2.78)),
        material=galvanized_steel,
        name="pole",
    )
    mast.visual(
        Cylinder(radius=0.09, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 5.54)),
        material=galvanized_steel,
        name="top_collar",
    )
    mast.visual(
        Box((0.22, 0.30, 0.06)),
        origin=Origin(xyz=(0.11, 0.0, 5.46)),
        material=painted_steel,
        name="crossarm",
    )
    mast.visual(
        Box((0.24, 0.06, 0.03)),
        origin=Origin(xyz=(0.12, 0.0, 5.36), rpy=(0.0, -0.64, 0.0)),
        material=painted_steel,
        name="brace",
    )
    mast.visual(
        Box((0.16, 0.04, 0.04)),
        origin=Origin(xyz=(0.23, -0.17, 5.44)),
        material=painted_steel,
        name="left_support_tie",
    )
    mast.visual(
        Box((0.16, 0.04, 0.04)),
        origin=Origin(xyz=(0.23, 0.17, 5.44)),
        material=painted_steel,
        name="right_support_tie",
    )
    mast.visual(
        Box((0.028, 0.024, 0.18)),
        origin=Origin(xyz=(0.304, -0.167, 5.54)),
        material=painted_steel,
        name="left_cheek",
    )
    mast.visual(
        Box((0.028, 0.024, 0.18)),
        origin=Origin(xyz=(0.304, 0.167, 5.54)),
        material=painted_steel,
        name="right_cheek",
    )
    mast.inertial = Inertial.from_geometry(
        Box((0.42, 0.42, 5.63)),
        mass=260.0,
        origin=Origin(xyz=(0.0, 0.0, 2.815)),
    )

    head = model.part("floodlight_head")
    head.visual(
        Box((0.012, 0.252, 0.20)),
        origin=Origin(xyz=(0.096, 0.0, 0.0)),
        material=painted_steel,
        name="back_panel",
    )
    head.visual(
        Box((0.156, 0.252, 0.012)),
        origin=Origin(xyz=(0.174, 0.0, 0.106)),
        material=painted_steel,
        name="top_shell",
    )
    head.visual(
        Box((0.156, 0.252, 0.012)),
        origin=Origin(xyz=(0.174, 0.0, -0.106)),
        material=painted_steel,
        name="bottom_shell",
    )
    head.visual(
        Box((0.254, 0.022, 0.20)),
        origin=Origin(xyz=(0.125, -0.136, 0.0)),
        material=painted_steel,
        name="left_side_shell",
    )
    head.visual(
        Box((0.254, 0.022, 0.20)),
        origin=Origin(xyz=(0.125, 0.136, 0.0)),
        material=painted_steel,
        name="right_side_shell",
    )
    head.visual(
        Box((0.012, 0.252, 0.018)),
        origin=Origin(xyz=(0.258, 0.0, 0.103)),
        material=galvanized_steel,
        name="front_top_frame",
    )
    head.visual(
        Box((0.012, 0.252, 0.018)),
        origin=Origin(xyz=(0.258, 0.0, -0.103)),
        material=galvanized_steel,
        name="front_bottom_frame",
    )
    head.visual(
        Box((0.012, 0.018, 0.188)),
        origin=Origin(xyz=(0.258, -0.117, 0.0)),
        material=galvanized_steel,
        name="front_left_frame",
    )
    head.visual(
        Box((0.012, 0.018, 0.188)),
        origin=Origin(xyz=(0.258, 0.117, 0.0)),
        material=galvanized_steel,
        name="front_right_frame",
    )
    head.visual(
        Box((0.006, 0.228, 0.164)),
        origin=Origin(xyz=(0.249, 0.0, 0.0)),
        material=lens_glass,
        name="lens",
    )
    head.visual(
        Cylinder(radius=0.018, length=0.02),
        origin=Origin(xyz=(0.0, -0.155, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=painted_steel,
        name="left_trunnion",
    )
    head.visual(
        Cylinder(radius=0.018, length=0.02),
        origin=Origin(xyz=(0.0, 0.155, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=painted_steel,
        name="right_trunnion",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.20, 0.276, 0.224)),
        mass=22.0,
        origin=Origin(xyz=(0.14, 0.0, 0.0)),
    )

    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=head,
        origin=Origin(xyz=(0.31, 0.0, 5.54)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.8,
            lower=-0.75,
            upper=0.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=SAFE_ASSET_ROOT)
    mast = object_model.get_part("mast")
    head = object_model.get_part("floodlight_head")
    tilt = object_model.get_articulation("head_tilt")

    base_plate = mast.get_visual("base_plate")
    base_collar = mast.get_visual("base_collar")
    pole = mast.get_visual("pole")
    crossarm = mast.get_visual("crossarm")
    left_support_tie = mast.get_visual("left_support_tie")
    right_support_tie = mast.get_visual("right_support_tie")
    left_cheek = mast.get_visual("left_cheek")
    right_cheek = mast.get_visual("right_cheek")
    back_panel = head.get_visual("back_panel")
    front_top_frame = head.get_visual("front_top_frame")
    front_bottom_frame = head.get_visual("front_bottom_frame")
    front_left_frame = head.get_visual("front_left_frame")
    front_right_frame = head.get_visual("front_right_frame")
    lens = head.get_visual("lens")
    left_trunnion = head.get_visual("left_trunnion")
    right_trunnion = head.get_visual("right_trunnion")
    limits = tilt.motion_limits

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        head,
        mast,
        elem_a=left_trunnion,
        elem_b=left_cheek,
        reason="left trunnion visually passes through a cheek plate that stands in for a drilled hinge hole",
    )
    ctx.allow_overlap(
        head,
        mast,
        elem_a=right_trunnion,
        elem_b=right_cheek,
        reason="right trunnion visually passes through a cheek plate that stands in for a drilled hinge hole",
    )
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=64)

    ctx.check(
        "tilt_joint_is_revolute_with_realistic_range",
        tilt.joint_type == ArticulationType.REVOLUTE
        and tuple(float(v) for v in tilt.axis) == (0.0, 1.0, 0.0)
        and limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and -1.2 <= limits.lower <= -0.7
        and 0.2 <= limits.upper <= 0.6,
        details=f"type={tilt.joint_type}, axis={tilt.axis}, limits={limits}",
    )

    ctx.expect_within(
        mast,
        mast,
        axes="xy",
        inner_elem=pole,
        outer_elem=base_plate,
        margin=0.0,
        name="pole_footprint_stays_on_square_base_plate",
    )
    ctx.expect_gap(
        mast,
        mast,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=base_collar,
        negative_elem=base_plate,
        name="base_collar_seats_on_base_plate",
    )
    ctx.expect_gap(
        mast,
        mast,
        axis="x",
        min_gap=0.16,
        positive_elem=left_cheek,
        negative_elem=pole,
        name="top_bracket_projects_forward_of_pole",
    )
    ctx.expect_contact(mast, mast, elem_a=left_support_tie, elem_b=crossarm)
    ctx.expect_contact(mast, mast, elem_a=right_support_tie, elem_b=crossarm)
    ctx.expect_contact(mast, mast, elem_a=left_support_tie, elem_b=left_cheek)
    ctx.expect_contact(mast, mast, elem_a=right_support_tie, elem_b=right_cheek)
    ctx.expect_contact(head, mast, elem_a=left_trunnion, elem_b=left_cheek)
    ctx.expect_contact(head, mast, elem_a=right_trunnion, elem_b=right_cheek)
    ctx.expect_gap(
        head,
        mast,
        axis="x",
        min_gap=0.10,
        positive_elem=lens,
        negative_elem=pole,
        name="floodlight_lens_projects_forward_of_pole",
    )
    ctx.expect_within(
        head,
        head,
        axes="yz",
        inner_elem=lens,
        outer_elem=back_panel,
        margin=0.0,
        name="lens_stays_within_housing_envelope",
    )
    ctx.expect_gap(
        head,
        head,
        axis="x",
        max_gap=0.006,
        max_penetration=0.0,
        positive_elem=front_top_frame,
        negative_elem=lens,
        name="lens_sits_just_behind_top_frame",
    )
    ctx.expect_gap(
        head,
        head,
        axis="x",
        max_gap=0.006,
        max_penetration=0.0,
        positive_elem=front_bottom_frame,
        negative_elem=lens,
        name="lens_sits_just_behind_bottom_frame",
    )
    ctx.expect_gap(
        head,
        head,
        axis="x",
        max_gap=0.006,
        max_penetration=0.0,
        positive_elem=front_left_frame,
        negative_elem=lens,
        name="lens_sits_just_behind_left_frame",
    )
    ctx.expect_gap(
        head,
        head,
        axis="x",
        max_gap=0.006,
        max_penetration=0.0,
        positive_elem=front_right_frame,
        negative_elem=lens,
        name="lens_sits_just_behind_right_frame",
    )
    ctx.expect_gap(
        head,
        mast,
        axis="x",
        min_gap=0.0,
        positive_elem=back_panel,
        negative_elem=crossarm,
        name="head_mount_stays_at_front_of_crossarm",
    )

    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({tilt: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="head_tilt_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="head_tilt_lower_no_floating")
            ctx.expect_contact(head, mast, elem_a=left_trunnion, elem_b=left_cheek)
            ctx.expect_contact(head, mast, elem_a=right_trunnion, elem_b=right_cheek)
            ctx.expect_gap(
                head,
                mast,
                axis="x",
                min_gap=0.05,
                positive_elem=lens,
                negative_elem=pole,
                name="lower_tilt_keeps_lens_ahead_of_pole",
            )
        with ctx.pose({tilt: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="head_tilt_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="head_tilt_upper_no_floating")
            ctx.expect_contact(head, mast, elem_a=left_trunnion, elem_b=left_cheek)
            ctx.expect_contact(head, mast, elem_a=right_trunnion, elem_b=right_cheek)
            ctx.expect_gap(
                head,
                mast,
                axis="x",
                min_gap=0.08,
                positive_elem=lens,
                negative_elem=pole,
                name="upper_tilt_keeps_lens_ahead_of_pole",
            )
            ctx.expect_gap(
                head,
                mast,
                axis="x",
                min_gap=0.12,
                positive_elem=back_panel,
                negative_elem=crossarm,
                name="upper_tilt_back_of_head_stays_forward_of_crossarm",
            )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
