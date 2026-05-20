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

    stainless = model.material("stainless", rgba=(0.78, 0.80, 0.82, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.68, 0.70, 0.72, 1.0))
    charcoal = model.material("charcoal", rgba=(0.14, 0.15, 0.16, 1.0))
    filter_dark = model.material("filter_dark", rgba=(0.24, 0.25, 0.27, 1.0))
    soft_light = model.material("soft_light", rgba=(0.92, 0.90, 0.78, 0.65))

    canopy_width = 0.90
    canopy_depth = 0.55
    canopy_height = 0.19
    panel_thickness = 0.015

    chimney_width = 0.34
    chimney_depth = 0.28
    chimney_height = 0.28
    chimney_center_y = -canopy_depth * 0.5 + chimney_depth * 0.5

    front_panel_center_y = canopy_depth * 0.5 - panel_thickness * 0.5
    front_panel_outer_y = canopy_depth * 0.5

    control_z = 0.105
    control_band_height = 0.041
    control_band_bottom = control_z - control_band_height * 0.5
    control_band_top = control_z + control_band_height * 0.5
    knob_x = -0.375
    button_left_x = -0.285
    button_right_x = -0.235
    knob_hole_radius = 0.0175
    button_hole_radius = 0.0105
    knob_open_half_width = 0.021
    button_open_half_width = 0.0115

    hood = model.part("hood_body")
    hood.visual(
        Box((canopy_width, canopy_depth, panel_thickness)),
        origin=Origin(xyz=(0.0, 0.0, canopy_height - panel_thickness * 0.5)),
        material=stainless,
        name="canopy_top",
    )
    hood.visual(
        Box((canopy_width, panel_thickness, control_band_bottom)),
        origin=Origin(xyz=(0.0, front_panel_center_y, control_band_bottom * 0.5)),
        material=stainless,
        name="front_panel_lower",
    )
    hood.visual(
        Box((canopy_width, panel_thickness, canopy_height - control_band_top)),
        origin=Origin(
            xyz=(
                0.0,
                front_panel_center_y,
                control_band_top + (canopy_height - control_band_top) * 0.5,
            )
        ),
        material=stainless,
        name="front_panel_upper",
    )
    front_band_segments = (
        ("front_panel_far_left", -canopy_width * 0.5, knob_x - knob_open_half_width),
        ("front_panel_between_knob_and_left_button", knob_x + knob_open_half_width, button_left_x - button_open_half_width),
        ("front_panel_between_buttons", button_left_x + button_open_half_width, button_right_x - button_open_half_width),
        ("front_panel_right", button_right_x + button_open_half_width, canopy_width * 0.5),
    )
    for visual_name, x_min, x_max in front_band_segments:
        hood.visual(
            Box((x_max - x_min, panel_thickness, control_band_height)),
            origin=Origin(xyz=((x_min + x_max) * 0.5, front_panel_center_y, control_z)),
            material=stainless,
            name=visual_name,
        )
    hood.visual(
        Box((canopy_width, panel_thickness, canopy_height)),
        origin=Origin(xyz=(0.0, -canopy_depth * 0.5 + panel_thickness * 0.5, canopy_height * 0.5)),
        material=satin_steel,
        name="back_panel",
    )
    hood.visual(
        Box((panel_thickness, canopy_depth, canopy_height)),
        origin=Origin(xyz=(-canopy_width * 0.5 + panel_thickness * 0.5, 0.0, canopy_height * 0.5)),
        material=stainless,
        name="left_side_panel",
    )
    hood.visual(
        Box((panel_thickness, canopy_depth, canopy_height)),
        origin=Origin(xyz=(canopy_width * 0.5 - panel_thickness * 0.5, 0.0, canopy_height * 0.5)),
        material=stainless,
        name="right_side_panel",
    )
    hood.visual(
        Box((0.885, 0.070, 0.018)),
        origin=Origin(xyz=(0.0, 0.225, 0.021)),
        material=satin_steel,
        name="front_underside_rail",
    )
    hood.visual(
        Box((0.885, 0.030, 0.018)),
        origin=Origin(xyz=(0.0, -0.230, 0.021)),
        material=satin_steel,
        name="rear_underside_rail",
    )
    hood.visual(
        Box((0.030, 0.455, 0.018)),
        origin=Origin(xyz=(-0.4275, -0.0025, 0.021)),
        material=satin_steel,
        name="left_underside_rail",
    )
    hood.visual(
        Box((0.030, 0.455, 0.018)),
        origin=Origin(xyz=(0.4275, -0.0025, 0.021)),
        material=satin_steel,
        name="right_underside_rail",
    )
    hood.visual(
        Box((0.825, 0.390, 0.008)),
        origin=Origin(xyz=(0.0, -0.005, 0.021)),
        material=filter_dark,
        name="filter_baffle",
    )
    hood.visual(
        Box((0.12, 0.050, 0.008)),
        origin=Origin(xyz=(-0.22, 0.205, 0.008)),
        material=soft_light,
        name="left_task_light",
    )
    hood.visual(
        Box((0.12, 0.050, 0.008)),
        origin=Origin(xyz=(0.22, 0.205, 0.008)),
        material=soft_light,
        name="right_task_light",
    )
    hood.visual(
        Box((chimney_width, panel_thickness, chimney_height)),
        origin=Origin(
            xyz=(
                0.0,
                chimney_center_y + chimney_depth * 0.5 - panel_thickness * 0.5,
                canopy_height + chimney_height * 0.5,
            )
        ),
        material=stainless,
        name="chimney_front",
    )
    hood.visual(
        Box((chimney_width, panel_thickness, chimney_height)),
        origin=Origin(
            xyz=(
                0.0,
                chimney_center_y - chimney_depth * 0.5 + panel_thickness * 0.5,
                canopy_height + chimney_height * 0.5,
            )
        ),
        material=satin_steel,
        name="chimney_back",
    )
    hood.visual(
        Box((panel_thickness, chimney_depth, chimney_height)),
        origin=Origin(
            xyz=(
                -chimney_width * 0.5 + panel_thickness * 0.5,
                chimney_center_y,
                canopy_height + chimney_height * 0.5,
            )
        ),
        material=stainless,
        name="chimney_left",
    )
    hood.visual(
        Box((panel_thickness, chimney_depth, chimney_height)),
        origin=Origin(
            xyz=(
                chimney_width * 0.5 - panel_thickness * 0.5,
                chimney_center_y,
                canopy_height + chimney_height * 0.5,
            )
        ),
        material=stainless,
        name="chimney_right",
    )
    hood.inertial = Inertial.from_geometry(
        Box((canopy_width, canopy_depth, canopy_height + chimney_height)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, (canopy_height + chimney_height) * 0.5)),
    )

    knob = model.part("selector_knob")
    knob.visual(
        Cylinder(radius=knob_hole_radius, length=0.020),
        origin=Origin(xyz=(0.0, -0.005, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="knob_shaft",
    )
    knob.visual(
        Cylinder(radius=0.036, length=0.006),
        origin=Origin(xyz=(0.0, 0.003, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="knob_base",
    )
    knob.visual(
        Cylinder(radius=0.031, length=0.024),
        origin=Origin(xyz=(0.0, 0.018, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="knob_body",
    )
    knob.visual(
        Box((0.004, 0.004, 0.015)),
        origin=Origin(xyz=(0.0, 0.031, 0.020)),
        material=satin_steel,
        name="knob_pointer",
    )
    knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.036, length=0.032),
        mass=0.20,
        origin=Origin(xyz=(0.0, 0.012, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
    )

    button_left = model.part("push_button_left")
    button_left.visual(
        Cylinder(radius=button_hole_radius, length=0.021),
        origin=Origin(xyz=(0.0, -0.0045, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="button_shaft",
    )
    button_left.visual(
        Cylinder(radius=0.012, length=0.002),
        origin=Origin(xyz=(0.0, 0.001, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="button_cap",
    )
    button_left.inertial = Inertial.from_geometry(
        Cylinder(radius=0.012, length=0.023),
        mass=0.03,
        origin=Origin(xyz=(0.0, -0.0035, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
    )

    button_right = model.part("push_button_right")
    button_right.visual(
        Cylinder(radius=button_hole_radius, length=0.021),
        origin=Origin(xyz=(0.0, -0.0045, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="button_shaft",
    )
    button_right.visual(
        Cylinder(radius=0.012, length=0.002),
        origin=Origin(xyz=(0.0, 0.001, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="button_cap",
    )
    button_right.inertial = Inertial.from_geometry(
        Cylinder(radius=0.012, length=0.023),
        mass=0.03,
        origin=Origin(xyz=(0.0, -0.0035, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "hood_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=hood,
        child=knob,
        origin=Origin(xyz=(knob_x, front_panel_outer_y, control_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.35, velocity=8.0),
    )
    model.articulation(
        "hood_to_push_button_left",
        ArticulationType.PRISMATIC,
        parent=hood,
        child=button_left,
        origin=Origin(xyz=(button_left_x, front_panel_outer_y, control_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.04,
            lower=0.0,
            upper=0.0025,
        ),
    )
    model.articulation(
        "hood_to_push_button_right",
        ArticulationType.PRISMATIC,
        parent=hood,
        child=button_right,
        origin=Origin(xyz=(button_right_x, front_panel_outer_y, control_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.04,
            lower=0.0,
            upper=0.0025,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    hood = object_model.get_part("hood_body")
    knob = object_model.get_part("selector_knob")
    button_left = object_model.get_part("push_button_left")
    button_right = object_model.get_part("push_button_right")

    knob_joint = object_model.get_articulation("hood_to_selector_knob")
    button_left_joint = object_model.get_articulation("hood_to_push_button_left")
    button_right_joint = object_model.get_articulation("hood_to_push_button_right")

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=16)

    ctx.check(
        "knob_joint_is_continuous",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"expected continuous knob joint, got {knob_joint.articulation_type!r}",
    )
    ctx.check(
        "button_joints_are_prismatic",
        button_left_joint.articulation_type == ArticulationType.PRISMATIC
        and button_right_joint.articulation_type == ArticulationType.PRISMATIC,
        details="both buttons should use prismatic plungers",
    )
    ctx.check(
        "control_joint_axes_face_forward",
        knob_joint.axis == (0.0, 1.0, 0.0)
        and button_left_joint.axis == (0.0, -1.0, 0.0)
        and button_right_joint.axis == (0.0, -1.0, 0.0),
        details=(
            f"unexpected joint axes: knob={knob_joint.axis}, "
            f"left={button_left_joint.axis}, right={button_right_joint.axis}"
        ),
    )

    hood_aabb = ctx.part_world_aabb(hood)
    ctx.check("hood_body_present", hood_aabb is not None, details="hood body has no world AABB")
    if hood_aabb is not None:
        hood_width = hood_aabb[1][0] - hood_aabb[0][0]
        hood_depth = hood_aabb[1][1] - hood_aabb[0][1]
        hood_height = hood_aabb[1][2] - hood_aabb[0][2]
        ctx.check(
            "hood_realistic_proportions",
            0.88 <= hood_width <= 0.92 and 0.54 <= hood_depth <= 0.56 and 0.46 <= hood_height <= 0.49,
            details=(
                f"unexpected hood dimensions: width={hood_width:.3f}, "
                f"depth={hood_depth:.3f}, height={hood_height:.3f}"
            ),
        )

    knob_pos = ctx.part_world_position(knob)
    button_left_pos = ctx.part_world_position(button_left)
    button_right_pos = ctx.part_world_position(button_right)
    ctx.check(
        "controls_clustered_far_left",
        knob_pos is not None
        and button_left_pos is not None
        and button_right_pos is not None
        and knob_pos[0] < -0.33
        and button_left_pos[0] > knob_pos[0]
        and button_right_pos[0] > button_left_pos[0]
        and button_right_pos[0] < -0.18,
        details=(
            f"unexpected control positions: knob={knob_pos}, "
            f"left={button_left_pos}, right={button_right_pos}"
        ),
    )

    ctx.expect_contact(
        knob,
        hood,
        elem_a="knob_base",
        name="knob_seated_on_hood_face",
    )
    ctx.expect_contact(
        button_left,
        hood,
        elem_a="button_cap",
        name="left_button_seated_on_hood_face",
    )
    ctx.expect_contact(
        button_right,
        hood,
        elem_a="button_cap",
        name="right_button_seated_on_hood_face",
    )
    ctx.expect_gap(
        knob,
        hood,
        axis="y",
        min_gap=0.0,
        max_gap=0.050,
        positive_elem="knob_body",
        name="knob_projects_out_from_hood_face",
    )
    ctx.expect_gap(
        button_left,
        hood,
        axis="y",
        min_gap=0.0,
        max_gap=0.010,
        positive_elem="button_cap",
        name="left_button_protrudes_slightly",
    )
    ctx.expect_gap(
        button_right,
        hood,
        axis="y",
        min_gap=0.0,
        max_gap=0.010,
        positive_elem="button_cap",
        name="right_button_protrudes_slightly",
    )

    with ctx.pose({knob_joint: math.pi * 0.75}):
        ctx.expect_contact(
            knob,
            hood,
            elem_a="knob_base",
            name="knob_remains_seated_when_rotated",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="knob_rotated_no_overlap")
        ctx.fail_if_isolated_parts(name="knob_rotated_no_floating")

    for joint, part_name in (
        (button_left_joint, "left_button"),
        (button_right_joint, "right_button"),
    ):
        limits = joint.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({joint: limits.lower}):
                ctx.expect_contact(
                    joint.child,
                    hood,
                    elem_a="button_cap",
                    name=f"{part_name}_rest_contact",
                )
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{part_name}_rest_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{part_name}_rest_no_floating")
            with ctx.pose({joint: limits.upper}):
                ctx.expect_contact(
                    joint.child,
                    hood,
                    elem_a="button_cap",
                    name=f"{part_name}_pressed_contact",
                )
                ctx.expect_gap(
                    joint.child,
                    hood,
                    axis="y",
                    max_gap=0.001,
                    max_penetration=0.003,
                    positive_elem="button_cap",
                    name=f"{part_name}_pressed_travel_is_short",
                )
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{part_name}_pressed_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{part_name}_pressed_no_floating")

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
