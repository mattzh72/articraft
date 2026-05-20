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
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_range_hood", assets=ASSETS)

    stainless = model.material("stainless", rgba=(0.77, 0.79, 0.82, 1.0))
    charcoal = model.material("charcoal", rgba=(0.13, 0.13, 0.14, 1.0))
    black_glass = model.material("black_glass", rgba=(0.08, 0.08, 0.09, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.24, 0.24, 0.26, 1.0))

    canopy_width = 0.90
    canopy_depth = 0.50
    canopy_height = 0.18
    shell_thickness = 0.018
    top_thickness = 0.015

    chimney_width = 0.32
    chimney_depth = 0.26
    chimney_height = 0.64
    chimney_wall = 0.012

    front_y = 0.5 * canopy_depth - 0.5 * shell_thickness
    back_y = -front_y
    side_x = 0.5 * canopy_width - 0.5 * shell_thickness

    canopy = model.part("canopy")
    canopy.visual(
        Box((canopy_width, canopy_depth, top_thickness)),
        origin=Origin(xyz=(0.0, 0.0, canopy_height - 0.5 * top_thickness)),
        material=stainless,
        name="top_shell",
    )
    canopy.visual(
        Box((canopy_width, shell_thickness, canopy_height - top_thickness)),
        origin=Origin(xyz=(0.0, back_y, 0.5 * (canopy_height - top_thickness))),
        material=stainless,
        name="back_shell",
    )
    canopy.visual(
        Box((shell_thickness, canopy_depth - 2.0 * shell_thickness, canopy_height - top_thickness)),
        origin=Origin(xyz=(-side_x, 0.0, 0.5 * (canopy_height - top_thickness))),
        material=stainless,
        name="left_shell",
    )
    canopy.visual(
        Box((shell_thickness, canopy_depth - 2.0 * shell_thickness, canopy_height - top_thickness)),
        origin=Origin(xyz=(side_x, 0.0, 0.5 * (canopy_height - top_thickness))),
        material=stainless,
        name="right_shell",
    )
    canopy.visual(
        Box((canopy_width, shell_thickness, 0.070)),
        origin=Origin(xyz=(0.0, front_y, 0.035)),
        material=stainless,
        name="lower_front_strip",
    )
    canopy.visual(
        Box((canopy_width, shell_thickness, 0.055)),
        origin=Origin(xyz=(0.0, front_y, 0.1375)),
        material=stainless,
        name="upper_front_strip",
    )
    canopy.visual(
        Box((0.130, shell_thickness, 0.040)),
        origin=Origin(xyz=(-0.385, front_y, 0.090)),
        material=stainless,
        name="control_band_far_left",
    )
    canopy.visual(
        Box((0.095, shell_thickness, 0.040)),
        origin=Origin(xyz=(-0.2525, front_y, 0.090)),
        material=stainless,
        name="control_band_left_center",
    )
    canopy.visual(
        Box((0.095, shell_thickness, 0.040)),
        origin=Origin(xyz=(0.2525, front_y, 0.090)),
        material=stainless,
        name="control_band_right_center",
    )
    canopy.visual(
        Box((0.130, shell_thickness, 0.040)),
        origin=Origin(xyz=(0.385, front_y, 0.090)),
        material=stainless,
        name="control_band_far_right",
    )
    canopy.visual(
        Box((canopy_width - 2.0 * shell_thickness, canopy_depth - 2.0 * shell_thickness, 0.010)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                0.005,
            )
        ),
        material=dark_metal,
        name="baffle_filter",
    )
    canopy.visual(
        Box((0.84, 0.010, 0.040)),
        origin=Origin(xyz=(0.0, 0.227, 0.090)),
        material=black_glass,
        name="control_recess",
    )
    canopy.visual(
        Box((0.402, 0.020, 0.008)),
        origin=Origin(xyz=(0.0, 0.240, 0.106)),
        material=charcoal,
        name="button_guide_top",
    )
    canopy.visual(
        Box((0.402, 0.020, 0.008)),
        origin=Origin(xyz=(0.0, 0.240, 0.074)),
        material=charcoal,
        name="button_guide_bottom",
    )
    canopy.visual(
        Box((0.008, 0.020, 0.024)),
        origin=Origin(xyz=(-0.201, 0.240, 0.090)),
        material=charcoal,
        name="button_guide_left",
    )
    canopy.visual(
        Box((0.008, 0.020, 0.024)),
        origin=Origin(xyz=(0.201, 0.240, 0.090)),
        material=charcoal,
        name="button_guide_right",
    )

    canopy.inertial = Inertial.from_geometry(
        Box((canopy_width, canopy_depth, canopy_height)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.5 * canopy_height)),
    )

    chimney = model.part("chimney_cover")
    chimney.visual(
        Box((chimney_width, chimney_wall, chimney_height)),
        origin=Origin(xyz=(0.0, 0.5 * chimney_depth - 0.5 * chimney_wall, 0.5 * chimney_height)),
        material=stainless,
        name="chimney_front",
    )
    chimney.visual(
        Box((chimney_width, chimney_wall, chimney_height)),
        origin=Origin(xyz=(0.0, -0.5 * chimney_depth + 0.5 * chimney_wall, 0.5 * chimney_height)),
        material=stainless,
        name="chimney_back",
    )
    chimney.visual(
        Box((chimney_wall, chimney_depth - 2.0 * chimney_wall, chimney_height)),
        origin=Origin(xyz=(-0.5 * chimney_width + 0.5 * chimney_wall, 0.0, 0.5 * chimney_height)),
        material=stainless,
        name="chimney_left",
    )
    chimney.visual(
        Box((chimney_wall, chimney_depth - 2.0 * chimney_wall, chimney_height)),
        origin=Origin(xyz=(0.5 * chimney_width - 0.5 * chimney_wall, 0.0, 0.5 * chimney_height)),
        material=stainless,
        name="chimney_right",
    )
    chimney.inertial = Inertial.from_geometry(
        Box((chimney_width, chimney_depth, chimney_height)),
        mass=5.0,
        origin=Origin(xyz=(0.0, 0.0, 0.5 * chimney_height)),
    )

    button = model.part("center_button")
    button.visual(
        Box((0.394, 0.018, 0.024)),
        origin=Origin(xyz=(0.0, 0.002, 0.0)),
        material=charcoal,
        name="button_stem",
    )
    button.visual(
        Box((0.460, 0.008, 0.036)),
        origin=Origin(xyz=(0.0, 0.015, 0.0)),
        material=black_glass,
        name="button_cap",
    )
    button.inertial = Inertial.from_geometry(
        Box((0.460, 0.027, 0.036)),
        mass=0.15,
        origin=Origin(xyz=(0.0, 0.008, 0.0)),
    )

    left_knob = model.part("left_knob")
    left_knob.visual(
        Cylinder(radius=0.022, length=0.014),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_glass,
        name="knob_cap",
    )
    left_knob.visual(
        Box((0.005, 0.0015, 0.016)),
        origin=Origin(xyz=(0.0, 0.0070, 0.009)),
        material=charcoal,
        name="knob_indicator",
    )
    left_knob.inertial = Inertial.from_geometry(
        Box((0.044, 0.016, 0.044)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    right_knob = model.part("right_knob")
    right_knob.visual(
        Cylinder(radius=0.022, length=0.014),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_glass,
        name="knob_cap",
    )
    right_knob.visual(
        Box((0.005, 0.0015, 0.016)),
        origin=Origin(xyz=(0.0, 0.0070, 0.009)),
        material=charcoal,
        name="knob_indicator",
    )
    right_knob.inertial = Inertial.from_geometry(
        Box((0.044, 0.016, 0.044)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    model.articulation(
        "canopy_to_chimney",
        ArticulationType.FIXED,
        parent=canopy,
        child=chimney,
        origin=Origin(xyz=(0.0, 0.0, canopy_height)),
    )
    model.articulation(
        "canopy_to_center_button",
        ArticulationType.PRISMATIC,
        parent=canopy,
        child=button,
        origin=Origin(xyz=(0.0, 0.241, 0.090)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.04, lower=0.0, upper=0.002),
    )
    model.articulation(
        "canopy_to_left_knob",
        ArticulationType.CONTINUOUS,
        parent=canopy,
        child=left_knob,
        origin=Origin(xyz=(-0.310, 0.257, 0.090)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )
    model.articulation(
        "canopy_to_right_knob",
        ArticulationType.CONTINUOUS,
        parent=canopy,
        child=right_knob,
        origin=Origin(xyz=(0.310, 0.257, 0.090)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    def _aabb_center(aabb):
        return tuple(0.5 * (aabb[0][i] + aabb[1][i]) for i in range(3))

    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
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

    canopy = object_model.get_part("canopy")
    chimney = object_model.get_part("chimney_cover")
    button = object_model.get_part("center_button")
    left_knob = object_model.get_part("left_knob")
    right_knob = object_model.get_part("right_knob")

    chimney_joint = object_model.get_articulation("canopy_to_chimney")
    button_joint = object_model.get_articulation("canopy_to_center_button")
    left_knob_joint = object_model.get_articulation("canopy_to_left_knob")
    right_knob_joint = object_model.get_articulation("canopy_to_right_knob")

    button_limits = button_joint.motion_limits
    left_knob_limits = left_knob_joint.motion_limits
    right_knob_limits = right_knob_joint.motion_limits

    ctx.check(
        "part_names_present",
        all(
            object_model.get_part(name).name == name
            for name in ("canopy", "chimney_cover", "center_button", "left_knob", "right_knob")
        ),
        details="Expected canopy shell, chimney cover, center push-button, and both end knobs.",
    )
    ctx.check(
        "only_requested_articulations_present",
        (
            chimney_joint.articulation_type == ArticulationType.FIXED
            and button_joint.articulation_type == ArticulationType.PRISMATIC
            and left_knob_joint.articulation_type == ArticulationType.CONTINUOUS
            and right_knob_joint.articulation_type == ArticulationType.CONTINUOUS
        ),
        details="Only the center button and the two end knobs should articulate.",
    )
    ctx.check(
        "button_joint_axis_and_travel",
        (
            button_joint.axis == (0.0, -1.0, 0.0)
            and button_limits is not None
            and button_limits.lower == 0.0
            and button_limits.upper is not None
            and 0.001 <= button_limits.upper <= 0.003
        ),
        details="The center button should move inward along the hood front by a short realistic travel.",
    )
    ctx.check(
        "knob_joint_axes_and_limits",
        (
            left_knob_joint.axis == (0.0, 1.0, 0.0)
            and right_knob_joint.axis == (0.0, 1.0, 0.0)
            and left_knob_limits is not None
            and right_knob_limits is not None
            and left_knob_limits.lower is None
            and left_knob_limits.upper is None
            and right_knob_limits.lower is None
            and right_knob_limits.upper is None
        ),
        details="Both end knobs should rotate continuously about their face-normal axes.",
    )

    ctx.expect_contact(chimney, canopy, elem_a="chimney_front", elem_b="top_shell")
    ctx.expect_within(chimney, canopy, axes="xy", margin=0.0)
    ctx.expect_origin_distance(chimney, canopy, axes="xy", max_dist=0.001)

    ctx.expect_contact(button, canopy, elem_a="button_stem", elem_b="button_guide_left")
    ctx.expect_contact(button, canopy, elem_a="button_stem", elem_b="button_guide_right")
    ctx.expect_gap(
        button,
        canopy,
        axis="y",
        positive_elem="button_cap",
        negative_elem="control_band_left_center",
        min_gap=0.0018,
        max_gap=0.0022,
        name="button_rest_proud_gap",
    )

    ctx.expect_contact(left_knob, canopy, elem_a="knob_cap", elem_b="control_band_far_left")
    ctx.expect_contact(right_knob, canopy, elem_a="knob_cap", elem_b="control_band_far_right")
    ctx.expect_gap(
        left_knob,
        canopy,
        axis="y",
        positive_elem="knob_cap",
        negative_elem="control_band_far_left",
        max_gap=0.0002,
        max_penetration=0.0,
        name="left_knob_rest_proud_gap",
    )
    ctx.expect_gap(
        right_knob,
        canopy,
        axis="y",
        positive_elem="knob_cap",
        negative_elem="control_band_far_right",
        max_gap=0.0002,
        max_penetration=0.0,
        name="right_knob_rest_proud_gap",
    )
    ctx.expect_origin_gap(
        right_knob,
        left_knob,
        axis="x",
        min_gap=0.60,
        max_gap=0.64,
        name="knob_spacing_across_canopy",
    )

    if button_limits is not None and button_limits.lower is not None and button_limits.upper is not None:
        with ctx.pose({button_joint: button_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="button_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="button_lower_no_floating")
        with ctx.pose({button_joint: button_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="button_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="button_upper_no_floating")
            ctx.expect_contact(button, canopy, elem_a="button_stem", elem_b="button_guide_left")
            ctx.expect_contact(button, canopy, elem_a="button_stem", elem_b="button_guide_right")
            ctx.expect_gap(
                button,
                canopy,
                axis="y",
                positive_elem="button_cap",
                negative_elem="control_band_left_center",
                max_gap=0.0002,
                max_penetration=0.0,
                name="button_pressed_flush_gap",
            )

    with ctx.pose({left_knob_joint: math.pi / 2.0, right_knob_joint: -math.pi / 2.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="knobs_rotated_no_overlap")
        ctx.fail_if_isolated_parts(name="knobs_rotated_no_floating")
        ctx.expect_contact(left_knob, canopy, elem_a="knob_cap", elem_b="control_band_far_left")
        ctx.expect_contact(right_knob, canopy, elem_a="knob_cap", elem_b="control_band_far_right")

    left_indicator_rest = ctx.part_element_world_aabb(left_knob, elem="knob_indicator")
    right_indicator_rest = ctx.part_element_world_aabb(right_knob, elem="knob_indicator")
    if left_indicator_rest is not None and right_indicator_rest is not None:
        left_rest_center = _aabb_center(left_indicator_rest)
        right_rest_center = _aabb_center(right_indicator_rest)
        with ctx.pose({left_knob_joint: math.pi / 2.0, right_knob_joint: -math.pi / 2.0}):
            left_indicator_rotated = ctx.part_element_world_aabb(left_knob, elem="knob_indicator")
            right_indicator_rotated = ctx.part_element_world_aabb(right_knob, elem="knob_indicator")
            if left_indicator_rotated is not None and right_indicator_rotated is not None:
                left_rot_center = _aabb_center(left_indicator_rotated)
                right_rot_center = _aabb_center(right_indicator_rotated)
                ctx.check(
                    "left_knob_indicator_rotates_about_y",
                    left_rot_center[0] > left_rest_center[0] + 0.006
                    and left_rot_center[2] < left_rest_center[2] - 0.006,
                    details="Left knob indicator should sweep sideways around the face-normal axis.",
                )
                ctx.check(
                    "right_knob_indicator_rotates_about_y",
                    right_rot_center[0] < right_rest_center[0] - 0.006
                    and right_rot_center[2] < right_rest_center[2] - 0.006,
                    details="Right knob indicator should sweep sideways around the face-normal axis.",
                )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
