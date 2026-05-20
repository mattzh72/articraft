from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_range_hood", assets=ASSETS)

    brushed_steel = model.material("brushed_steel", rgba=(0.76, 0.78, 0.80, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.68, 0.70, 0.72, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.19, 0.20, 1.0))
    button_black = model.material("button_black", rgba=(0.14, 0.14, 0.15, 1.0))

    hood_width = 0.90
    canopy_height = 0.24
    canopy_top_width = 0.46
    canopy_top_depth = 0.22
    canopy_top_thickness = 0.014
    chimney_width = 0.32
    chimney_depth = 0.22
    chimney_height = 0.74

    front_panel_width = 0.90
    front_panel_height = 0.122
    front_panel_thickness = 0.012
    front_panel_center_y = 0.390
    front_panel_center_z = 0.140

    button_opening_width = 0.104
    button_opening_height = 0.054
    button_travel = 0.005
    button_depth = 0.028
    button_proud = 0.007
    center_mullion_width = 0.020
    button_center_offset = center_mullion_width * 0.5 + button_opening_width * 0.5
    outer_stile_width = (front_panel_width - 2.0 * button_opening_width - center_mullion_width) * 0.5
    rail_height = (front_panel_height - button_opening_height) * 0.5
    guide_depth = 0.040

    top_plate_center_y = canopy_top_depth * 0.5
    top_plate_center_z = canopy_height - canopy_top_thickness * 0.5
    top_plate_front_y = canopy_top_depth
    top_plate_bottom_z = canopy_height - canopy_top_thickness
    front_panel_back_y = front_panel_center_y - front_panel_thickness * 0.5
    shoulder_run = front_panel_back_y - top_plate_front_y
    shoulder_drop = (front_panel_center_z + front_panel_height * 0.5) - top_plate_bottom_z
    shoulder_length = math.hypot(shoulder_run, shoulder_drop)
    shoulder_angle = math.atan2(shoulder_drop, shoulder_run)
    guide_center_y = front_panel_center_y - (front_panel_thickness + guide_depth) * 0.5
    button_rest_center_y = (
        front_panel_center_y + front_panel_thickness * 0.5 + button_proud - button_depth * 0.5
    )

    hood_body = model.part("hood_body")
    hood_body.visual(
        Box((hood_width, 0.014, canopy_height)),
        origin=Origin(xyz=(0.0, 0.007, canopy_height * 0.5)),
        material=brushed_steel,
        name="back_plate",
    )
    hood_body.visual(
        Box((hood_width, canopy_top_depth, 0.10)),
        origin=Origin(xyz=(0.0, canopy_top_depth * 0.5, 0.05)),
        material=satin_steel,
        name="lower_body",
    )
    hood_body.visual(
        Box((canopy_top_width, canopy_top_depth, canopy_top_thickness)),
        origin=Origin(xyz=(0.0, top_plate_center_y, top_plate_center_z)),
        material=brushed_steel,
        name="top_plate",
    )
    hood_body.visual(
        Box((hood_width, shoulder_length, 0.012)),
        origin=Origin(
            xyz=(
                0.0,
                (top_plate_front_y + front_panel_back_y) * 0.5,
                (top_plate_bottom_z + front_panel_center_z + front_panel_height * 0.5) * 0.5,
            ),
            rpy=(shoulder_angle, 0.0, 0.0),
        ),
        material=brushed_steel,
        name="sloped_shoulder",
    )
    hood_body.visual(
        Box((front_panel_width, front_panel_thickness, rail_height)),
        origin=Origin(
            xyz=(
                0.0,
                front_panel_center_y,
                front_panel_center_z + button_opening_height * 0.5 + rail_height * 0.5,
            )
        ),
        material=brushed_steel,
        name="front_upper_rail",
    )
    hood_body.visual(
        Box((front_panel_width, front_panel_thickness, rail_height)),
        origin=Origin(
            xyz=(
                0.0,
                front_panel_center_y,
                front_panel_center_z - button_opening_height * 0.5 - rail_height * 0.5,
            )
        ),
        material=brushed_steel,
        name="front_lower_rail",
    )
    hood_body.visual(
        Box((outer_stile_width, front_panel_thickness, button_opening_height)),
        origin=Origin(
            xyz=(
                -(front_panel_width * 0.5 - outer_stile_width * 0.5),
                front_panel_center_y,
                front_panel_center_z,
            )
        ),
        material=brushed_steel,
        name="front_left_stile",
    )
    hood_body.visual(
        Box((center_mullion_width, front_panel_thickness, button_opening_height)),
        origin=Origin(xyz=(0.0, front_panel_center_y, front_panel_center_z)),
        material=brushed_steel,
        name="front_center_mullion",
    )
    hood_body.visual(
        Box((outer_stile_width, front_panel_thickness, button_opening_height)),
        origin=Origin(
            xyz=(
                front_panel_width * 0.5 - outer_stile_width * 0.5,
                front_panel_center_y,
                front_panel_center_z,
            )
        ),
        material=brushed_steel,
        name="front_right_stile",
    )
    hood_body.visual(
        Box((front_panel_width, guide_depth, rail_height)),
        origin=Origin(
            xyz=(
                0.0,
                guide_center_y,
                front_panel_center_z + button_opening_height * 0.5 + rail_height * 0.5,
            )
        ),
        material=charcoal,
        name="guide_upper",
    )
    hood_body.visual(
        Box((front_panel_width, guide_depth, rail_height)),
        origin=Origin(
            xyz=(
                0.0,
                guide_center_y,
                front_panel_center_z - button_opening_height * 0.5 - rail_height * 0.5,
            )
        ),
        material=charcoal,
        name="guide_lower",
    )
    hood_body.visual(
        Box((outer_stile_width, guide_depth, button_opening_height)),
        origin=Origin(
            xyz=(
                -(front_panel_width * 0.5 - outer_stile_width * 0.5),
                guide_center_y,
                front_panel_center_z,
            )
        ),
        material=charcoal,
        name="guide_left_stile",
    )
    hood_body.visual(
        Box((center_mullion_width, guide_depth, button_opening_height)),
        origin=Origin(xyz=(0.0, guide_center_y, front_panel_center_z)),
        material=charcoal,
        name="guide_center_mullion",
    )
    hood_body.visual(
        Box((outer_stile_width, guide_depth, button_opening_height)),
        origin=Origin(
            xyz=(
                front_panel_width * 0.5 - outer_stile_width * 0.5,
                guide_center_y,
                front_panel_center_z,
            )
        ),
        material=charcoal,
        name="guide_right_stile",
    )
    hood_body.visual(
        Box((0.62, 0.24, 0.014)),
        origin=Origin(xyz=(0.0, 0.27, 0.086)),
        material=charcoal,
        name="filter_panel",
    )
    hood_body.visual(
        Box((chimney_width, chimney_depth, chimney_height)),
        origin=Origin(xyz=(0.0, chimney_depth * 0.5, canopy_height + chimney_height * 0.5)),
        material=brushed_steel,
        name="chimney_cover",
    )

    hood_body.inertial = Inertial.from_geometry(
        Box((hood_width, 0.48, canopy_height + chimney_height)),
        mass=21.0,
        origin=Origin(xyz=(0.0, 0.24, 0.49)),
    )

    for button_name, x_center in (
        ("left_button", -button_center_offset),
        ("right_button", button_center_offset),
    ):
        button_part = model.part(button_name)
        button_part.visual(
            Box((button_opening_width, button_depth, button_opening_height)),
            origin=Origin(),
            material=button_black,
            name="button_pad",
        )
        button_part.inertial = Inertial.from_geometry(
            Box((button_opening_width, button_depth, button_opening_height)),
            mass=0.14,
            origin=Origin(),
        )
        model.articulation(
            f"{button_name}_travel",
            ArticulationType.PRISMATIC,
            parent=hood_body,
            child=button_part,
            origin=Origin(xyz=(x_center, button_rest_center_y, front_panel_center_z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.04,
                lower=0.0,
                upper=button_travel,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    hood_body = object_model.get_part("hood_body")
    left_button = object_model.get_part("left_button")
    right_button = object_model.get_part("right_button")
    left_button_travel = object_model.get_articulation("left_button_travel")
    right_button_travel = object_model.get_articulation("right_button_travel")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    hood_visual_names = {visual.name for visual in hood_body.visuals}
    ctx.check(
        "hood_core_visuals_present",
        {
            "sloped_shoulder",
            "front_upper_rail",
            "front_lower_rail",
            "filter_panel",
            "chimney_cover",
        }.issubset(hood_visual_names),
        f"Missing hood visuals: {sorted({'sloped_shoulder', 'front_upper_rail', 'front_lower_rail', 'filter_panel', 'chimney_cover'} - hood_visual_names)}",
    )
    ctx.check(
        "only_two_button_articulations",
        len(object_model.articulations) == 2,
        f"Expected exactly 2 articulations, found {len(object_model.articulations)}.",
    )

    for joint in (left_button_travel, right_button_travel):
        ctx.check(
            f"{joint.name}_is_prismatic",
            joint.articulation_type == ArticulationType.PRISMATIC,
            f"{joint.name} should be prismatic, found {joint.articulation_type}.",
        )
        ctx.check(
            f"{joint.name}_axis_is_panel_normal",
            tuple(joint.axis) == (0.0, -1.0, 0.0),
            f"{joint.name} axis should be (0, -1, 0), found {joint.axis}.",
        )
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name}_short_travel_limit",
            limits is not None and limits.lower == 0.0 and limits.upper == 0.005,
            f"{joint.name} should have short 5 mm travel, found {limits}.",
        )

    ctx.expect_contact(left_button, hood_body, name="left_button_supported")
    ctx.expect_contact(right_button, hood_body, name="right_button_supported")
    ctx.expect_overlap(
        left_button,
        hood_body,
        axes="xz",
        min_overlap=0.05,
        name="left_button_on_front_band",
    )
    ctx.expect_overlap(
        right_button,
        hood_body,
        axes="xz",
        min_overlap=0.05,
        name="right_button_on_front_band",
    )
    ctx.expect_origin_gap(
        right_button,
        left_button,
        axis="x",
        min_gap=0.10,
        max_gap=0.14,
        name="buttons_side_by_side_spacing",
    )
    ctx.expect_within(left_button, hood_body, axes="x", margin=0.0, name="left_button_within_hood_width")
    ctx.expect_within(right_button, hood_body, axes="x", margin=0.0, name="right_button_within_hood_width")

    left_rest = ctx.part_world_position(left_button)
    right_rest = ctx.part_world_position(right_button)
    if left_rest is not None and right_rest is not None:
        ctx.check(
            "buttons_centered_across_front_panel",
            abs((left_rest[0] + right_rest[0]) * 0.5) <= 0.005,
            f"Button centers should straddle x=0, found left={left_rest[0]:.4f}, right={right_rest[0]:.4f}.",
        )
        ctx.check(
            "buttons_level_with_each_other",
            abs(left_rest[2] - right_rest[2]) <= 1e-6,
            f"Buttons should be level, found z positions {left_rest[2]:.6f} and {right_rest[2]:.6f}.",
        )
        ctx.check(
            "buttons_in_control_band",
            0.11 <= left_rest[2] <= 0.17 and 0.11 <= right_rest[2] <= 0.17,
            f"Buttons should sit on the front control band, found z positions {left_rest[2]:.4f} and {right_rest[2]:.4f}.",
        )
    else:
        ctx.fail("button_rest_positions_available", "Could not resolve rest positions for both buttons.")

    with ctx.pose({left_button_travel: 0.0, right_button_travel: 0.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="buttons_rest_no_overlap")
        ctx.fail_if_isolated_parts(name="buttons_rest_no_floating")

    with ctx.pose({left_button_travel: 0.005, right_button_travel: 0.005}):
        ctx.fail_if_parts_overlap_in_current_pose(name="buttons_pressed_no_overlap")
        ctx.fail_if_isolated_parts(name="buttons_pressed_no_floating")
        ctx.expect_contact(left_button, hood_body, name="left_button_pressed_still_supported")
        ctx.expect_contact(right_button, hood_body, name="right_button_pressed_still_supported")

        left_pressed = ctx.part_world_position(left_button)
        right_pressed = ctx.part_world_position(right_button)
        if left_rest is not None and left_pressed is not None:
            ctx.check(
                "left_button_moves_inward",
                abs(left_pressed[0] - left_rest[0]) <= 1e-6
                and left_pressed[1] < left_rest[1] - 0.002
                and abs(left_pressed[2] - left_rest[2]) <= 1e-6,
                f"Left button should move straight inward from {left_rest} to {left_pressed}.",
            )
        else:
            ctx.fail("left_button_pressed_position_available", "Could not resolve left button pressed position.")
        if right_rest is not None and right_pressed is not None:
            ctx.check(
                "right_button_moves_inward",
                abs(right_pressed[0] - right_rest[0]) <= 1e-6
                and right_pressed[1] < right_rest[1] - 0.002
                and abs(right_pressed[2] - right_rest[2]) <= 1e-6,
                f"Right button should move straight inward from {right_rest} to {right_pressed}.",
            )
        else:
            ctx.fail("right_button_pressed_position_available", "Could not resolve right button pressed position.")

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
