from __future__ import annotations

import os

_ORIG_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _ORIG_GETCWD()
    except FileNotFoundError:
        try:
            os.chdir("/")
        except Exception:
            return "/"
        return "/"


os.getcwd = _safe_getcwd
_safe_getcwd()

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="scanner_printer_combo")

    body_color = model.material("body_charcoal", rgba=(0.23, 0.24, 0.26, 1.0))
    lid_color = model.material("lid_silver", rgba=(0.78, 0.80, 0.82, 1.0))
    tray_color = model.material("tray_light_gray", rgba=(0.72, 0.74, 0.77, 1.0))
    trim_color = model.material("trim_gray", rgba=(0.52, 0.54, 0.57, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.10, 0.11, 0.12, 1.0))
    glass = model.material("glass", rgba=(0.08, 0.12, 0.15, 0.55))

    body_width = 0.50
    body_depth = 0.36
    body_height = 0.13
    wall = 0.012
    top_thickness = 0.012

    body = model.part("body")
    body.visual(
        Box((body_width, body_depth, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=body_color,
        name="bottom_plate",
    )
    body.visual(
        Box((wall, body_depth, body_height)),
        origin=Origin(xyz=(-(body_width - wall) / 2.0, 0.0, body_height / 2.0)),
        material=body_color,
        name="left_wall",
    )
    body.visual(
        Box((wall, body_depth, body_height)),
        origin=Origin(xyz=((body_width - wall) / 2.0, 0.0, body_height / 2.0)),
        material=body_color,
        name="right_wall",
    )
    body.visual(
        Box((body_width, wall, 0.100)),
        origin=Origin(xyz=(0.0, -(body_depth - wall) / 2.0, 0.050)),
        material=body_color,
        name="rear_wall",
    )
    body.visual(
        Box((0.074, wall, 0.024)),
        origin=Origin(xyz=(-0.213, (body_depth - wall) / 2.0, 0.012)),
        material=body_color,
        name="front_lower_left",
    )
    body.visual(
        Box((0.074, wall, 0.024)),
        origin=Origin(xyz=(0.213, (body_depth - wall) / 2.0, 0.012)),
        material=body_color,
        name="front_lower_right",
    )
    body.visual(
        Box((body_width, wall, 0.052)),
        origin=Origin(xyz=(0.0, (body_depth - wall) / 2.0, 0.104)),
        material=body_color,
        name="front_upper_fascia",
    )
    body.visual(
        Box((0.092, body_depth - wall, top_thickness)),
        origin=Origin(xyz=(-0.204, 0.0, 0.112)),
        material=body_color,
        name="top_frame_left",
    )
    body.visual(
        Box((0.092, body_depth - wall, top_thickness)),
        origin=Origin(xyz=(0.204, 0.0, 0.112)),
        material=body_color,
        name="top_frame_right",
    )
    body.visual(
        Box((body_width, 0.064, top_thickness)),
        origin=Origin(xyz=(0.0, 0.142, 0.112)),
        material=body_color,
        name="top_frame_front",
    )
    body.visual(
        Box((0.320, 0.228, 0.012)),
        origin=Origin(xyz=(0.0, -0.006, 0.110)),
        material=dark_trim,
        name="scan_well",
    )
    body.visual(
        Box((0.316, 0.224, 0.002)),
        origin=Origin(xyz=(0.0, -0.006, 0.117)),
        material=glass,
        name="scanner_glass",
    )
    body.visual(
        Box((0.150, 0.060, 0.004)),
        origin=Origin(xyz=(0.122, 0.140, 0.132)),
        material=trim_color,
        name="control_panel",
    )
    body.visual(
        Box((0.052, 0.024, 0.0015)),
        origin=Origin(xyz=(0.070, 0.140, 0.13475)),
        material=dark_trim,
        name="display",
    )
    body.visual(
        Box((0.050, 0.022, 0.002)),
        origin=Origin(xyz=(0.140, 0.140, 0.135)),
        material=dark_trim,
        name="buttons",
    )
    body.visual(
        Box((0.270, 0.004, 0.018)),
        origin=Origin(xyz=(0.0, 0.166, 0.069)),
        material=dark_trim,
        name="paper_slot",
    )
    body.visual(
        Box((0.300, 0.180, 0.054)),
        origin=Origin(xyz=(0.0, 0.078, 0.061)),
        material=dark_trim,
        name="paper_path_shadow",
    )
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            body.visual(
                Box((0.030, 0.030, 0.010)),
                origin=Origin(xyz=(0.212 * sx, 0.142 * sy, 0.005)),
                material=dark_trim,
                name=f"foot_{'l' if sx < 0 else 'r'}_{'rear' if sy < 0 else 'front'}",
            )
    for side, x in (("left", -0.110), ("right", 0.110)):
        body.visual(
            Box((0.092, 0.180, 0.012)),
            origin=Origin(xyz=(x, 0.066, 0.016)),
            material=trim_color,
            name=f"tray_support_{side}",
        )
    for side, x in (("left", -0.176), ("right", 0.176)):
        body.visual(
            Box((0.020, 0.028, 0.026)),
            origin=Origin(xyz=(x, -0.176, 0.109)),
            material=trim_color,
            name=f"hinge_support_{side}",
        )
        body.visual(
            Cylinder(radius=0.009, length=0.028),
            origin=Origin(
                xyz=(x, -0.189, 0.121),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=trim_color,
            name=f"body_hinge_{side}",
        )
    body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height)),
        mass=8.8,
        origin=Origin(xyz=(0.0, 0.0, body_height / 2.0)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((0.472, 0.312, 0.016)),
        origin=Origin(xyz=(0.0, 0.183, 0.007)),
        material=lid_color,
        name="lid_shell",
    )
    lid.visual(
        Box((0.306, 0.220, 0.002)),
        origin=Origin(xyz=(0.0, 0.183, 0.000)),
        material=trim_color,
        name="lid_liner",
    )
    lid.visual(
        Box((0.170, 0.014, 0.008)),
        origin=Origin(xyz=(0.0, 0.316, 0.019)),
        material=trim_color,
        name="lid_front_grip",
    )
    for side, x in (("left", -0.148), ("right", 0.148)):
        lid.visual(
            Box((0.024, 0.030, 0.012)),
            origin=Origin(xyz=(x, 0.015, 0.003)),
            material=trim_color,
            name=f"lid_hinge_bridge_{side}",
        )
        lid.visual(
            Cylinder(radius=0.009, length=0.028),
            origin=Origin(
                xyz=(x, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=trim_color,
            name=f"lid_hinge_{side}",
        )
    lid.inertial = Inertial.from_geometry(
        Box((0.472, 0.312, 0.016)),
        mass=1.9,
        origin=Origin(xyz=(0.0, 0.183, 0.007)),
    )

    tray = model.part("tray")
    tray.visual(
        Box((0.340, 0.180, 0.004)),
        origin=Origin(xyz=(0.0, 0.090, 0.010)),
        material=tray_color,
        name="tray_deck",
    )
    tray.visual(
        Box((0.330, 0.008, 0.026)),
        origin=Origin(xyz=(0.0, 0.176, 0.013)),
        material=tray_color,
        name="tray_front_lip",
    )
    for side, x in (("left", -0.167), ("right", 0.167)):
        tray.visual(
            Box((0.006, 0.160, 0.014)),
            origin=Origin(xyz=(x, 0.090, 0.017)),
            material=tray_color,
            name=f"tray_side_rail_{side}",
        )
    for side, x in (("left", -0.110), ("right", 0.110)):
        tray.visual(
            Box((0.090, 0.160, 0.012)),
            origin=Origin(xyz=(x, 0.080, 0.006)),
            material=dark_trim,
            name=f"tray_runner_{side}",
        )
    tray.inertial = Inertial.from_geometry(
        Box((0.340, 0.180, 0.030)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.090, 0.010)),
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -0.189, 0.121)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.0,
            lower=0.0,
            upper=1.15,
        ),
    )
    model.articulation(
        "tray_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=0.20,
            lower=0.0,
            upper=0.125,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, seed=0)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    tray = object_model.get_part("tray")
    lid_hinge = object_model.get_articulation("lid_hinge")
    tray_slide = object_model.get_articulation("tray_slide")

    bottom_plate = body.get_visual("bottom_plate")
    scanner_glass = body.get_visual("scanner_glass")
    control_panel = body.get_visual("control_panel")
    paper_slot = body.get_visual("paper_slot")
    tray_support_left = body.get_visual("tray_support_left")
    tray_support_right = body.get_visual("tray_support_right")
    body_hinge_left = body.get_visual("body_hinge_left")
    body_hinge_right = body.get_visual("body_hinge_right")

    lid_shell = lid.get_visual("lid_shell")
    lid_front_grip = lid.get_visual("lid_front_grip")
    lid_hinge_left = lid.get_visual("lid_hinge_left")
    lid_hinge_right = lid.get_visual("lid_hinge_right")

    tray_front_lip = tray.get_visual("tray_front_lip")
    tray_runner_left = tray.get_visual("tray_runner_left")
    tray_runner_right = tray.get_visual("tray_runner_right")
    tray_deck = tray.get_visual("tray_deck")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=48)

    body_aabb = ctx.part_world_aabb(body)
    if body_aabb is None:
        ctx.fail("body_has_world_bounds", "Body AABB was not available.")
    else:
        width = body_aabb[1][0] - body_aabb[0][0]
        depth = body_aabb[1][1] - body_aabb[0][1]
        height = body_aabb[1][2] - body_aabb[0][2]
        ctx.check(
            "body_is_wide_desktop_printer_scale",
            width > 0.46 and depth > 0.32 and 0.12 <= height <= 0.16 and width > depth,
            f"Expected a wide desktop all-in-one printer body, got dims {(width, depth, height)}.",
        )

    lid_limits = lid_hinge.motion_limits
    tray_limits = tray_slide.motion_limits
    ctx.check(
        "lid_hinge_axis_is_x",
        tuple(lid_hinge.axis) == (1.0, 0.0, 0.0),
        f"Expected lid hinge axis (1, 0, 0), got {lid_hinge.axis}.",
    )
    ctx.check(
        "tray_slide_axis_is_y",
        tuple(tray_slide.axis) == (0.0, 1.0, 0.0),
        f"Expected tray slide axis (0, 1, 0), got {tray_slide.axis}.",
    )
    ctx.check(
        "lid_hinge_limits_realistic",
        lid_limits is not None
        and lid_limits.lower == 0.0
        and lid_limits.upper is not None
        and 1.05 <= lid_limits.upper <= 1.35,
        f"Unexpected lid hinge limits: {lid_limits}.",
    )
    ctx.check(
        "tray_slide_limits_realistic",
        tray_limits is not None
        and tray_limits.lower == 0.0
        and tray_limits.upper is not None
        and 0.10 <= tray_limits.upper <= 0.14,
        f"Unexpected tray slide limits: {tray_limits}.",
    )

    with ctx.pose({lid_hinge: 0.0, tray_slide: 0.0}):
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.21,
            elem_a=lid_shell,
            elem_b=scanner_glass,
        )
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            min_gap=0.0015,
            max_gap=0.0035,
            positive_elem=lid_shell,
            negative_elem=scanner_glass,
        )
        ctx.expect_contact(lid, body, elem_a=lid_hinge_left, elem_b=body_hinge_left)
        ctx.expect_contact(lid, body, elem_a=lid_hinge_right, elem_b=body_hinge_right)
        ctx.expect_contact(tray, body, elem_a=tray_runner_left, elem_b=tray_support_left)
        ctx.expect_contact(tray, body, elem_a=tray_runner_right, elem_b=tray_support_right)
        ctx.expect_within(
            tray,
            body,
            axes="x",
            margin=0.0,
            inner_elem=tray_deck,
            outer_elem=bottom_plate,
        )
        ctx.expect_gap(
            tray,
            body,
            axis="y",
            min_gap=0.002,
            max_gap=0.008,
            positive_elem=tray_front_lip,
            negative_elem=paper_slot,
        )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
        ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")

    with ctx.pose({lid_hinge: lid_limits.upper}):
        ctx.expect_contact(lid, body, elem_a=lid_hinge_left, elem_b=body_hinge_left)
        ctx.expect_contact(lid, body, elem_a=lid_hinge_right, elem_b=body_hinge_right)
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            min_gap=0.18,
            positive_elem=lid_front_grip,
            negative_elem=control_panel,
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    with ctx.pose({tray_slide: 0.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="tray_slide_lower_no_overlap")
        ctx.fail_if_isolated_parts(name="tray_slide_lower_no_floating")

    with ctx.pose({tray_slide: tray_limits.upper}):
        ctx.expect_contact(tray, body, elem_a=tray_runner_left, elem_b=tray_support_left)
        ctx.expect_contact(tray, body, elem_a=tray_runner_right, elem_b=tray_support_right)
        ctx.expect_gap(
            tray,
            body,
            axis="y",
            min_gap=0.12,
            max_gap=0.14,
            positive_elem=tray_front_lip,
            negative_elem=paper_slot,
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="tray_slide_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="tray_slide_upper_no_floating")

    with ctx.pose({lid_hinge: lid_limits.upper, tray_slide: tray_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="scanner_printer_fully_open_no_overlap")
        ctx.fail_if_isolated_parts(name="scanner_printer_fully_open_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
