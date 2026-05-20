from __future__ import annotations

import os
import pathlib

_ORIG_GETCWD = os.getcwd
_ORIG_PATH_RESOLVE = pathlib.Path.resolve


def _safe_getcwd() -> str:
    try:
        return _ORIG_GETCWD()
    except FileNotFoundError:
        os.chdir("/")
        return "/"


def _safe_resolve(self: pathlib.Path, *args, **kwargs):
    try:
        return _ORIG_PATH_RESOLVE(self, *args, **kwargs)
    except FileNotFoundError:
        os.chdir("/")
        return _ORIG_PATH_RESOLVE(self, *args, **kwargs)


os.getcwd = _safe_getcwd
pathlib.Path.resolve = _safe_resolve
os.chdir("/")

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math


ASSET_ROOT = os.path.dirname(__file__) if os.path.isabs(__file__) else "/"


def _ensure_working_dir() -> None:
    try:
        _ORIG_GETCWD()
    except FileNotFoundError:
        os.chdir("/")


_ensure_working_dir()

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    _ensure_working_dir()
    model = ArticulatedObject(name="corner_mount_cctv_bracket")

    powder_coat = model.material("powder_coat", rgba=(0.18, 0.20, 0.23, 1.0))
    housing_white = model.material("housing_white", rgba=(0.86, 0.88, 0.89, 1.0))
    lens_black = model.material("lens_black", rgba=(0.08, 0.09, 0.10, 1.0))
    glass = model.material("glass", rgba=(0.30, 0.40, 0.46, 0.55))

    bracket = model.part("bracket")
    bracket.visual(
        Box((0.006, 0.092, 0.122)),
        origin=Origin(xyz=(0.003, 0.046, 0.061)),
        material=powder_coat,
        name="left_wall_flange",
    )
    bracket.visual(
        Box((0.092, 0.006, 0.122)),
        origin=Origin(xyz=(0.046, 0.003, 0.061)),
        material=powder_coat,
        name="right_wall_flange",
    )
    plate_widths = [0.032, 0.046, 0.060, 0.076, 0.090, 0.076, 0.060, 0.046, 0.032]
    for index, span in enumerate(plate_widths):
        bracket.visual(
            Box((0.006, span, 0.012)),
            origin=Origin(
                xyz=(0.0385, 0.0385, 0.012 + index * 0.011),
                rpy=(0.0, 0.0, math.pi / 4.0),
            ),
            material=powder_coat,
            name="wall_plate" if index == 4 else f"wall_plate_slice_{index}",
        )
    bracket.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(
            xyz=(0.056, 0.056, 0.074),
            rpy=(0.0, math.pi / 2.0, math.pi / 4.0),
        ),
        material=powder_coat,
        name="arm_socket",
    )
    bracket.visual(
        Cylinder(radius=0.010, length=0.054),
        origin=Origin(
            xyz=(0.080, 0.080, 0.074),
            rpy=(0.0, math.pi / 2.0, math.pi / 4.0),
        ),
        material=powder_coat,
        name="extension_arm",
    )
    bracket.visual(
        Box((0.034, 0.034, 0.020)),
        origin=Origin(
            xyz=(0.058, 0.058, 0.067),
            rpy=(0.0, 0.0, math.pi / 4.0),
        ),
        material=powder_coat,
        name="arm_block",
    )
    bracket.visual(
        Box((0.026, 0.026, 0.024)),
        origin=Origin(
            xyz=(0.048, 0.048, 0.063),
            rpy=(0.0, 0.0, math.pi / 4.0),
        ),
        material=powder_coat,
        name="mount_gusset",
    )
    bracket.visual(
        Cylinder(radius=0.016, length=0.010),
        origin=Origin(xyz=(0.104, 0.104, 0.078)),
        material=powder_coat,
        name="pan_seat",
    )
    bracket.visual(
        Cylinder(radius=0.005, length=0.003),
        origin=Origin(xyz=(0.0045, 0.026, 0.090), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_black,
        name="left_upper_fastener",
    )
    bracket.visual(
        Cylinder(radius=0.005, length=0.003),
        origin=Origin(xyz=(0.0045, 0.066, 0.036), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_black,
        name="left_lower_fastener",
    )
    bracket.visual(
        Cylinder(radius=0.005, length=0.003),
        origin=Origin(xyz=(0.026, 0.0045, 0.090), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=lens_black,
        name="right_upper_fastener",
    )
    bracket.visual(
        Cylinder(radius=0.005, length=0.003),
        origin=Origin(xyz=(0.066, 0.0045, 0.036), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=lens_black,
        name="right_lower_fastener",
    )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.016, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=powder_coat,
        name="pan_base",
    )
    pan_head.visual(
        Cylinder(radius=0.006, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=powder_coat,
        name="pan_stem",
    )
    pan_head.visual(
        Box((0.010, 0.024, 0.026)),
        origin=Origin(xyz=(-0.005, 0.0, 0.043)),
        material=powder_coat,
        name="tilt_pedestal",
    )
    pan_head.visual(
        Box((0.008, 0.006, 0.036)),
        origin=Origin(xyz=(0.0, -0.028, 0.055)),
        material=powder_coat,
        name="left_yoke",
    )
    pan_head.visual(
        Box((0.008, 0.006, 0.036)),
        origin=Origin(xyz=(0.0, 0.028, 0.055)),
        material=powder_coat,
        name="right_yoke",
    )
    pan_head.visual(
        Box((0.008, 0.062, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.073)),
        material=powder_coat,
        name="yoke_bridge",
    )
    pan_head.visual(
        Box((0.006, 0.050, 0.006)),
        origin=Origin(xyz=(-0.004, 0.0, 0.046)),
        material=powder_coat,
        name="yoke_lower_tie",
    )

    camera = model.part("camera")
    camera.visual(
        Cylinder(radius=0.004, length=0.008),
        origin=Origin(xyz=(0.0, -0.021, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=powder_coat,
        name="left_trunnion",
    )
    camera.visual(
        Cylinder(radius=0.004, length=0.008),
        origin=Origin(xyz=(0.0, 0.021, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=powder_coat,
        name="right_trunnion",
    )
    camera.visual(
        Box((0.016, 0.042, 0.014)),
        origin=Origin(xyz=(0.012, 0.0, 0.0)),
        material=housing_white,
        name="pivot_hub",
    )
    camera.visual(
        Cylinder(radius=0.015, length=0.094),
        origin=Origin(xyz=(0.077, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=housing_white,
        name="body",
    )
    camera.visual(
        Cylinder(radius=0.018, length=0.024),
        origin=Origin(xyz=(0.028, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=housing_white,
        name="rear_cap",
    )
    camera.visual(
        Box((0.064, 0.034, 0.004)),
        origin=Origin(xyz=(0.086, 0.0, 0.017)),
        material=housing_white,
        name="sunshield",
    )
    camera.visual(
        Cylinder(radius=0.016, length=0.014),
        origin=Origin(xyz=(0.131, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_black,
        name="lens_bezel",
    )
    camera.visual(
        Cylinder(radius=0.007, length=0.004),
        origin=Origin(xyz=(0.140, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="lens_glass",
    )

    model.articulation(
        "pan_joint",
        ArticulationType.REVOLUTE,
        parent=bracket,
        child=pan_head,
        origin=Origin(xyz=(0.104, 0.104, 0.083)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.2,
            lower=-1.25,
            upper=1.25,
        ),
    )
    model.articulation(
        "tilt_joint",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=camera,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=2.5,
            lower=-0.45,
            upper=0.35,
        ),
    )
    return model


def run_tests() -> TestReport:
    _ensure_working_dir()
    ctx = TestContext(object_model, asset_root=ASSET_ROOT)
    bracket = object_model.get_part("bracket")
    pan_head = object_model.get_part("pan_head")
    camera = object_model.get_part("camera")
    pan_joint = object_model.get_articulation("pan_joint")
    tilt_joint = object_model.get_articulation("tilt_joint")
    wall_plate = bracket.get_visual("wall_plate")
    pan_seat = bracket.get_visual("pan_seat")
    pan_base = pan_head.get_visual("pan_base")
    left_wall_flange = bracket.get_visual("left_wall_flange")
    right_wall_flange = bracket.get_visual("right_wall_flange")
    left_yoke = pan_head.get_visual("left_yoke")
    right_yoke = pan_head.get_visual("right_yoke")
    yoke_bridge = pan_head.get_visual("yoke_bridge")
    left_trunnion = camera.get_visual("left_trunnion")
    right_trunnion = camera.get_visual("right_trunnion")
    lens_bezel = camera.get_visual("lens_bezel")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts(max_pose_samples=16)
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=64,
        ignore_adjacent=False,
        ignore_fixed=True,
    )

    ctx.check(
        "pan_joint_axis_vertical",
        tuple(pan_joint.axis) == (0.0, 0.0, 1.0),
        details=f"expected vertical pan axis, got {pan_joint.axis}",
    )
    ctx.check(
        "tilt_joint_axis_horizontal",
        tuple(tilt_joint.axis) == (0.0, 1.0, 0.0),
        details=f"expected horizontal tilt axis, got {tilt_joint.axis}",
    )

    pan_limits = pan_joint.motion_limits
    tilt_limits = tilt_joint.motion_limits
    ctx.check(
        "pan_range_realistic",
        pan_limits is not None
        and pan_limits.lower is not None
        and pan_limits.upper is not None
        and -1.5 < pan_limits.lower < -0.9
        and 0.9 < pan_limits.upper < 1.5,
        details=f"unexpected pan limits: {pan_limits}",
    )
    ctx.check(
        "tilt_range_realistic",
        tilt_limits is not None
        and tilt_limits.lower is not None
        and tilt_limits.upper is not None
        and -1.0 < tilt_limits.lower < -0.4
        and 0.3 < tilt_limits.upper < 0.9,
        details=f"unexpected tilt limits: {tilt_limits}",
    )

    with ctx.pose({pan_joint: 0.0, tilt_joint: 0.0}):
        ctx.expect_overlap(
            pan_head,
            bracket,
            axes="xy",
            min_overlap=0.020,
            elem_a=pan_base,
            elem_b=pan_seat,
            name="pan_head_seated_on_bracket",
        )
        ctx.expect_gap(
            pan_head,
            bracket,
            axis="z",
            max_gap=0.0005,
            max_penetration=0.0,
            positive_elem=pan_base,
            negative_elem=pan_seat,
            name="pan_head_vertical_seating",
        )
        ctx.expect_contact(
            camera,
            pan_head,
            elem_a=left_trunnion,
            elem_b=left_yoke,
            contact_tol=0.0005,
            name="left_trunnion_supported",
        )
        ctx.expect_contact(
            camera,
            pan_head,
            elem_a=right_trunnion,
            elem_b=right_yoke,
            contact_tol=0.0005,
            name="right_trunnion_supported",
        )
        ctx.expect_origin_gap(
            camera,
            bracket,
            axis="x",
            min_gap=0.090,
            name="camera_projects_forward_in_x",
        )
        ctx.expect_origin_gap(
            camera,
            bracket,
            axis="y",
            min_gap=0.090,
            name="camera_projects_forward_in_y",
        )
        ctx.expect_gap(
            camera,
            bracket,
            axis="x",
            min_gap=0.040,
            positive_elem=lens_bezel,
            negative_elem=wall_plate,
            name="lens_clears_triangular_plate",
        )
        ctx.expect_overlap(
            bracket,
            bracket,
            axes="z",
            min_overlap=0.100,
            elem_a=left_wall_flange,
            elem_b=right_wall_flange,
            name="corner_flanges_share_vertical_extent",
        )

    if pan_limits is not None and pan_limits.lower is not None and pan_limits.upper is not None:
        with ctx.pose({pan_joint: pan_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="pan_joint_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="pan_joint_lower_no_floating")
            ctx.expect_overlap(
                pan_head,
                bracket,
                axes="xy",
                min_overlap=0.020,
                elem_a=pan_base,
                elem_b=pan_seat,
                name="pan_joint_lower_still_seated",
            )
        with ctx.pose({pan_joint: pan_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="pan_joint_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="pan_joint_upper_no_floating")
            ctx.expect_overlap(
                pan_head,
                bracket,
                axes="xy",
                min_overlap=0.020,
                elem_a=pan_base,
                elem_b=pan_seat,
                name="pan_joint_upper_still_seated",
            )

    if tilt_limits is not None and tilt_limits.lower is not None and tilt_limits.upper is not None:
        with ctx.pose({tilt_joint: tilt_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="tilt_joint_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="tilt_joint_lower_no_floating")
            ctx.expect_contact(
                camera,
                pan_head,
                elem_a=left_trunnion,
                elem_b=left_yoke,
                contact_tol=0.0005,
                name="tilt_joint_lower_left_trunnion_contact",
            )
            ctx.expect_contact(
                camera,
                pan_head,
                elem_a=right_trunnion,
                elem_b=right_yoke,
                contact_tol=0.0005,
                name="tilt_joint_lower_right_trunnion_contact",
            )
        with ctx.pose({tilt_joint: tilt_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="tilt_joint_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="tilt_joint_upper_no_floating")
            ctx.expect_contact(
                camera,
                pan_head,
                elem_a=left_trunnion,
                elem_b=left_yoke,
                contact_tol=0.0005,
                name="tilt_joint_upper_left_trunnion_contact",
            )
            ctx.expect_contact(
                camera,
                pan_head,
                elem_a=right_trunnion,
                elem_b=right_yoke,
                contact_tol=0.0005,
                name="tilt_joint_upper_right_trunnion_contact",
            )

    with ctx.pose({pan_joint: -1.0, tilt_joint: -0.45}):
        ctx.fail_if_parts_overlap_in_current_pose(name="compound_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="compound_pose_no_floating")
        ctx.expect_contact(
            camera,
            pan_head,
            elem_a=left_trunnion,
            elem_b=left_yoke,
            contact_tol=0.0005,
            name="compound_pose_left_support",
        )
        ctx.expect_contact(
            camera,
            pan_head,
            elem_a=right_trunnion,
            elem_b=right_yoke,
            contact_tol=0.0005,
            name="compound_pose_right_support",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
