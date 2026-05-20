from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os
from pathlib import Path

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

try:
    os.getcwd()
except FileNotFoundError:
    os.chdir("/tmp")

ASSETS = AssetContext.from_script(__file__)
HERE = Path(__file__).resolve().parent


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="secretary_desk", assets=ASSETS)

    wood = model.material("wood", rgba=(0.37, 0.22, 0.13, 1.0))
    wood_dark = model.material("wood_dark", rgba=(0.24, 0.14, 0.09, 1.0))
    interior = model.material("interior", rgba=(0.66, 0.54, 0.40, 1.0))
    brass = model.material("brass", rgba=(0.76, 0.63, 0.28, 1.0))

    body = model.part("body")
    flap = model.part("flap")
    drawer = model.part("drawer")

    case_width = 0.92
    case_depth = 0.44
    case_front_y = case_depth / 2.0
    plinth_height = 0.07
    lower_case_height = 0.82
    hutch_height = 0.48
    side_thickness = 0.03
    back_thickness = 0.018
    top_thickness = 0.028
    frame_depth = 0.024

    opening_width = 0.68
    flap_height = 0.38
    flap_thickness = 0.022
    hinge_radius = 0.006
    front_frame_y = 0.19
    hinge_y = 0.208
    hinge_z = 0.334
    opening_bottom_z = hinge_z + hinge_radius
    opening_top_z = opening_bottom_z + flap_height
    stile_width = 0.09

    compartment_floor_thickness = 0.018
    compartment_floor_depth = 0.334
    compartment_floor_center_y = -0.035

    drawer_outer_width = 0.54
    drawer_height = 0.08
    drawer_front_thickness = 0.018
    drawer_side_thickness = 0.012
    drawer_back_thickness = 0.012
    drawer_bottom_thickness = 0.008
    drawer_side_depth = 0.17
    drawer_depth_total = drawer_side_depth + drawer_front_thickness
    drawer_closed_front_y = 0.07
    drawer_bottom_z = 0.445
    drawer_travel = 0.14
    guide_thickness = case_width / 2.0 - side_thickness - drawer_outer_width / 2.0
    guide_height = 0.06
    guide_length = 0.32
    guide_center_y = -0.02
    guide_center_z = drawer_bottom_z + guide_height / 2.0

    body.visual(
        Box((case_width, case_depth, plinth_height)),
        origin=Origin(xyz=(0.0, 0.0, plinth_height / 2.0)),
        material=wood_dark,
        name="plinth",
    )
    body.visual(
        Box((side_thickness, case_depth, lower_case_height - plinth_height)),
        origin=Origin(
            xyz=(
                -(case_width / 2.0 - side_thickness / 2.0),
                0.0,
                plinth_height + (lower_case_height - plinth_height) / 2.0,
            )
        ),
        material=wood,
        name="left_lower_side",
    )
    body.visual(
        Box((side_thickness, case_depth, lower_case_height - plinth_height)),
        origin=Origin(
            xyz=(
                case_width / 2.0 - side_thickness / 2.0,
                0.0,
                plinth_height + (lower_case_height - plinth_height) / 2.0,
            )
        ),
        material=wood,
        name="right_lower_side",
    )
    body.visual(
        Box((case_width - 2.0 * side_thickness, back_thickness, lower_case_height - plinth_height)),
        origin=Origin(
            xyz=(
                0.0,
                -(case_depth / 2.0 - back_thickness / 2.0),
                plinth_height + (lower_case_height - plinth_height) / 2.0,
            )
        ),
        material=wood,
        name="lower_back",
    )
    body.visual(
        Box((case_width, case_depth, top_thickness)),
        origin=Origin(xyz=(0.0, 0.0, lower_case_height - top_thickness / 2.0)),
        material=wood_dark,
        name="case_top",
    )

    body.visual(
        Box((case_width - 0.06, frame_depth, 0.08)),
        origin=Origin(xyz=(0.0, front_frame_y, 0.16)),
        material=wood_dark,
        name="false_drawer_lower",
    )
    body.visual(
        Box((case_width - 0.06, frame_depth, 0.08)),
        origin=Origin(xyz=(0.0, front_frame_y, 0.255)),
        material=wood_dark,
        name="false_drawer_upper",
    )
    body.visual(
        Cylinder(radius=0.009, length=0.018),
        origin=Origin(
            xyz=(-0.15, front_frame_y + frame_depth / 2.0 + 0.009, 0.255),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=brass,
        name="false_drawer_pull_left",
    )
    body.visual(
        Cylinder(radius=0.009, length=0.018),
        origin=Origin(
            xyz=(0.15, front_frame_y + frame_depth / 2.0 + 0.009, 0.255),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=brass,
        name="false_drawer_pull_right",
    )

    body.visual(
        Box((stile_width, frame_depth, flap_height)),
        origin=Origin(
            xyz=(
                -(opening_width / 2.0 + stile_width / 2.0),
                front_frame_y,
                opening_bottom_z + flap_height / 2.0,
            )
        ),
        material=wood_dark,
        name="opening_left_stile",
    )
    body.visual(
        Box((stile_width, frame_depth, flap_height)),
        origin=Origin(
            xyz=(
                opening_width / 2.0 + stile_width / 2.0,
                front_frame_y,
                opening_bottom_z + flap_height / 2.0,
            )
        ),
        material=wood_dark,
        name="opening_right_stile",
    )
    body.visual(
        Box((opening_width, frame_depth, 0.048)),
        origin=Origin(
            xyz=(0.0, front_frame_y, opening_bottom_z - 0.024)
        ),
        material=wood_dark,
        name="opening_bottom_rail",
    )
    body.visual(
        Box((opening_width, frame_depth, 0.07)),
        origin=Origin(
            xyz=(0.0, front_frame_y, opening_top_z + 0.035)
        ),
        material=wood_dark,
        name="opening_top_rail",
    )
    body.visual(
        Cylinder(radius=hinge_radius, length=0.14),
        origin=Origin(
            xyz=(-0.27, hinge_y, hinge_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=brass,
        name="hinge_knuckle_left",
    )
    body.visual(
        Cylinder(radius=hinge_radius, length=0.14),
        origin=Origin(
            xyz=(0.27, hinge_y, hinge_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=brass,
        name="hinge_knuckle_right",
    )
    body.visual(
        Box((opening_width, compartment_floor_depth, compartment_floor_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                compartment_floor_center_y,
                hinge_z + flap_thickness - compartment_floor_thickness / 2.0,
            )
        ),
        material=interior,
        name="compartment_floor",
    )
    body.visual(
        Box((guide_thickness, guide_length, guide_height)),
        origin=Origin(
            xyz=(
                -(case_width / 2.0 - side_thickness - guide_thickness / 2.0),
                guide_center_y,
                guide_center_z,
            )
        ),
        material=interior,
        name="drawer_guide_left",
    )
    body.visual(
        Box((guide_thickness, guide_length, guide_height)),
        origin=Origin(
            xyz=(
                case_width / 2.0 - side_thickness - guide_thickness / 2.0,
                guide_center_y,
                guide_center_z,
            )
        ),
        material=interior,
        name="drawer_guide_right",
    )
    body.visual(
        Box((0.78, 0.24, 0.018)),
        origin=Origin(xyz=(0.0, -0.082, 0.59)),
        material=interior,
        name="cubby_shelf_mid",
    )
    body.visual(
        Box((0.78, 0.24, 0.018)),
        origin=Origin(xyz=(0.0, -0.082, 0.68)),
        material=interior,
        name="cubby_shelf_upper",
    )
    body.visual(
        Box((0.015, 0.24, 0.16)),
        origin=Origin(xyz=(-0.17, -0.082, 0.61)),
        material=interior,
        name="cubby_divider_left",
    )
    body.visual(
        Box((0.015, 0.24, 0.16)),
        origin=Origin(xyz=(0.0, -0.082, 0.61)),
        material=interior,
        name="cubby_divider_center",
    )
    body.visual(
        Box((0.015, 0.24, 0.16)),
        origin=Origin(xyz=(0.17, -0.082, 0.61)),
        material=interior,
        name="cubby_divider_right",
    )
    body.visual(
        Box((side_thickness, 0.24, hutch_height)),
        origin=Origin(
            xyz=(
                -(case_width / 2.0 - side_thickness / 2.0),
                -0.10,
                lower_case_height + hutch_height / 2.0,
            )
        ),
        material=wood,
        name="hutch_left_side",
    )
    body.visual(
        Box((side_thickness, 0.24, hutch_height)),
        origin=Origin(
            xyz=(
                case_width / 2.0 - side_thickness / 2.0,
                -0.10,
                lower_case_height + hutch_height / 2.0,
            )
        ),
        material=wood,
        name="hutch_right_side",
    )
    body.visual(
        Box((case_width - 2.0 * side_thickness, back_thickness, hutch_height)),
        origin=Origin(
            xyz=(
                0.0,
                -(case_depth / 2.0 - back_thickness / 2.0),
                lower_case_height + hutch_height / 2.0,
            )
        ),
        material=wood,
        name="hutch_back",
    )
    body.visual(
        Box((case_width, 0.24, top_thickness)),
        origin=Origin(
            xyz=(0.0, -0.10, lower_case_height + hutch_height - top_thickness / 2.0)
        ),
        material=wood_dark,
        name="hutch_top",
    )
    body.visual(
        Box((0.78, 0.22, 0.018)),
        origin=Origin(xyz=(0.0, -0.10, 0.96)),
        material=wood,
        name="hutch_shelf_lower",
    )
    body.visual(
        Box((0.78, 0.22, 0.018)),
        origin=Origin(xyz=(0.0, -0.10, 1.12)),
        material=wood,
        name="hutch_shelf_upper",
    )

    flap.visual(
        Box((opening_width, flap_thickness, flap_height)),
        origin=Origin(
            xyz=(0.0, -flap_thickness / 2.0, hinge_radius + flap_height / 2.0)
        ),
        material=wood_dark,
        name="flap_panel",
    )
    flap.visual(
        Cylinder(radius=hinge_radius, length=0.40),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=brass,
        name="hinge_knuckle_center",
    )
    flap.visual(
        Box((0.44, 0.014, 0.018)),
        origin=Origin(
            xyz=(0.0, -0.007, hinge_radius + flap_height - 0.009)
        ),
        material=wood,
        name="writing_lip",
    )
    flap.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(
            xyz=(-0.18, 0.007, hinge_radius + 0.23),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=brass,
        name="flap_pull_left",
    )
    flap.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(
            xyz=(0.18, 0.007, hinge_radius + 0.23),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=brass,
        name="flap_pull_right",
    )

    drawer.visual(
        Box((drawer_outer_width, drawer_front_thickness, drawer_height)),
        origin=Origin(
            xyz=(0.0, drawer_front_thickness / 2.0, drawer_height / 2.0)
        ),
        material=wood_dark,
        name="drawer_front",
    )
    drawer.visual(
        Box(
            (
                drawer_outer_width - 2.0 * drawer_side_thickness,
                drawer_side_depth,
                drawer_bottom_thickness,
            )
        ),
        origin=Origin(
            xyz=(0.0, -drawer_side_depth / 2.0, drawer_bottom_thickness / 2.0)
        ),
        material=interior,
        name="drawer_bottom",
    )
    drawer.visual(
        Box((drawer_side_thickness, drawer_side_depth, drawer_height - 0.025)),
        origin=Origin(
            xyz=(
                -(drawer_outer_width / 2.0 - drawer_side_thickness / 2.0),
                -drawer_side_depth / 2.0,
                (drawer_height - 0.025) / 2.0,
            )
        ),
        material=interior,
        name="drawer_side_left",
    )
    drawer.visual(
        Box((drawer_side_thickness, drawer_side_depth, drawer_height - 0.025)),
        origin=Origin(
            xyz=(
                drawer_outer_width / 2.0 - drawer_side_thickness / 2.0,
                -drawer_side_depth / 2.0,
                (drawer_height - 0.025) / 2.0,
            )
        ),
        material=interior,
        name="drawer_side_right",
    )
    drawer.visual(
        Box(
            (
                drawer_outer_width - 2.0 * drawer_side_thickness,
                drawer_back_thickness,
                drawer_height - 0.025,
            )
        ),
        origin=Origin(
            xyz=(
                0.0,
                -drawer_side_depth + drawer_back_thickness / 2.0,
                (drawer_height - 0.025) / 2.0,
            )
        ),
        material=interior,
        name="drawer_back",
    )
    drawer.visual(
        Cylinder(radius=0.008, length=0.014),
        origin=Origin(
            xyz=(0.0, drawer_front_thickness + 0.007, drawer_height / 2.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=brass,
        name="drawer_pull",
    )

    model.articulation(
        "body_to_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flap,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=-math.pi / 2.0,
            upper=0.0,
        ),
    )
    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(0.0, drawer_closed_front_y, drawer_bottom_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.3,
            lower=0.0,
            upper=drawer_travel,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    body = object_model.get_part("body")
    flap = object_model.get_part("flap")
    drawer = object_model.get_part("drawer")
    flap_hinge = object_model.get_articulation("body_to_flap")
    drawer_slide = object_model.get_articulation("body_to_drawer")

    flap_panel = flap.get_visual("flap_panel")
    flap_knuckle = flap.get_visual("hinge_knuckle_center")
    writing_lip = flap.get_visual("writing_lip")
    drawer_front = drawer.get_visual("drawer_front")
    drawer_bottom = drawer.get_visual("drawer_bottom")
    drawer_side_left = drawer.get_visual("drawer_side_left")
    drawer_side_right = drawer.get_visual("drawer_side_right")
    opening_bottom_rail = body.get_visual("opening_bottom_rail")
    opening_top_rail = body.get_visual("opening_top_rail")
    left_knuckle = body.get_visual("hinge_knuckle_left")
    right_knuckle = body.get_visual("hinge_knuckle_right")
    compartment_floor = body.get_visual("compartment_floor")
    drawer_guide_left = body.get_visual("drawer_guide_left")
    drawer_guide_right = body.get_visual("drawer_guide_right")
    cubby_shelf_mid = body.get_visual("cubby_shelf_mid")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=48)

    ctx.expect_contact(flap, body, elem_a=flap_knuckle, elem_b=left_knuckle)
    ctx.expect_contact(flap, body, elem_a=flap_knuckle, elem_b=right_knuckle)
    ctx.expect_gap(
        flap,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=flap_panel,
        negative_elem=opening_bottom_rail,
    )
    ctx.expect_gap(
        body,
        flap,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=opening_top_rail,
        negative_elem=flap_panel,
    )
    ctx.expect_contact(drawer, body, elem_a=drawer_side_left, elem_b=drawer_guide_left)
    ctx.expect_contact(drawer, body, elem_a=drawer_side_right, elem_b=drawer_guide_right)
    ctx.expect_gap(
        body,
        drawer,
        axis="y",
        min_gap=0.08,
        positive_elem=opening_bottom_rail,
        negative_elem=drawer_front,
    )
    ctx.expect_gap(
        body,
        drawer,
        axis="z",
        min_gap=0.05,
        positive_elem=cubby_shelf_mid,
        negative_elem=drawer_front,
    )
    ctx.expect_gap(
        drawer,
        body,
        axis="z",
        min_gap=0.07,
        positive_elem=drawer_bottom,
        negative_elem=compartment_floor,
    )

    flap_limits = flap_hinge.motion_limits
    drawer_limits = drawer_slide.motion_limits

    if flap_limits is not None and flap_limits.lower is not None:
        with ctx.pose({flap_hinge: flap_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="flap_open_no_overlap")
            ctx.fail_if_isolated_parts(name="flap_open_no_floating")
            ctx.expect_contact(
                flap,
                body,
                elem_a=flap_knuckle,
                elem_b=left_knuckle,
                name="flap_left_knuckle_open_contact",
            )
            ctx.expect_contact(
                flap,
                body,
                elem_a=flap_knuckle,
                elem_b=right_knuckle,
                name="flap_right_knuckle_open_contact",
            )
            ctx.expect_gap(
                flap,
                body,
                axis="y",
                min_gap=0.35,
                positive_elem=writing_lip,
                negative_elem=opening_bottom_rail,
                name="flap_opens_forward",
            )
            ctx.expect_gap(
                drawer,
                flap,
                axis="z",
                min_gap=0.07,
                positive_elem=drawer_bottom,
                negative_elem=flap_panel,
                name="drawer_clears_open_flap",
            )

    if (
        flap_limits is not None
        and flap_limits.lower is not None
        and drawer_limits is not None
        and drawer_limits.upper is not None
    ):
        with ctx.pose({flap_hinge: flap_limits.lower, drawer_slide: drawer_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="combined_open_pose_no_overlap")
            ctx.fail_if_isolated_parts(name="combined_open_pose_no_floating")
            ctx.expect_contact(
                drawer,
                body,
                elem_a=drawer_side_left,
                elem_b=drawer_guide_left,
                name="drawer_left_guide_contact_open",
            )
            ctx.expect_contact(
                drawer,
                body,
                elem_a=drawer_side_right,
                elem_b=drawer_guide_right,
                name="drawer_right_guide_contact_open",
            )
            ctx.expect_gap(
                drawer,
                body,
                axis="y",
                min_gap=0.005,
                positive_elem=drawer_front,
                negative_elem=opening_bottom_rail,
                name="drawer_protrudes_past_case",
            )
            ctx.expect_gap(
                drawer,
                flap,
                axis="z",
                min_gap=0.07,
                positive_elem=drawer_bottom,
                negative_elem=flap_panel,
                name="drawer_clears_flap_when_both_open",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
