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
    satin_dark = model.material("satin_dark", rgba=(0.20, 0.21, 0.22, 1.0))
    filter_aluminum = model.material("filter_aluminum", rgba=(0.56, 0.58, 0.60, 1.0))
    knob_black = model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))
    button_black = model.material("button_black", rgba=(0.14, 0.14, 0.15, 1.0))
    button_face = model.material("button_face", rgba=(0.24, 0.24, 0.25, 1.0))

    canopy_width = 0.90
    canopy_depth = 0.52
    canopy_height = 0.24
    shell_thickness = 0.015

    chimney_width = 0.36
    chimney_depth = 0.28
    chimney_height = 0.76
    chimney_thickness = 0.012

    control_knob_xs = (-0.22, 0.0, 0.22)
    button_xs = (-0.22, 0.22)

    body = model.part("hood_body")

    def add_body_box(
        name: str,
        size: tuple[float, float, float],
        xyz: tuple[float, float, float],
        material=stainless,
    ) -> None:
        body.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material=material,
            name=name,
        )

    half_width = canopy_width * 0.5
    half_depth = canopy_depth * 0.5
    wall_height = canopy_height - shell_thickness
    wall_center_z = wall_height * 0.5
    front_y = half_depth - shell_thickness * 0.5
    back_y = -half_depth + shell_thickness * 0.5
    side_x = half_width - shell_thickness * 0.5

    add_body_box(
        "canopy_top",
        (canopy_width, canopy_depth, shell_thickness),
        (0.0, 0.0, canopy_height - shell_thickness * 0.5),
    )
    add_body_box(
        "canopy_back",
        (canopy_width, shell_thickness, wall_height),
        (0.0, back_y, wall_center_z),
    )
    add_body_box(
        "canopy_left",
        (shell_thickness, canopy_depth, wall_height),
        (-side_x, 0.0, wall_center_z),
    )
    add_body_box(
        "canopy_right",
        (shell_thickness, canopy_depth, wall_height),
        (side_x, 0.0, wall_center_z),
    )

    add_body_box(
        "front_lower_strip",
        (canopy_width, shell_thickness, 0.060),
        (0.0, front_y, 0.030),
    )
    add_body_box(
        "front_slot_bottom_strip",
        (canopy_width, shell_thickness, 0.012),
        (0.0, front_y, 0.066),
    )
    add_body_box(
        "front_slot_top_strip",
        (canopy_width, shell_thickness, 0.012),
        (0.0, front_y, 0.114),
    )
    add_body_box(
        "front_slot_left_outer",
        (0.212, shell_thickness, 0.036),
        (-0.344, front_y, 0.090),
    )
    add_body_box(
        "front_slot_center",
        (0.404, shell_thickness, 0.036),
        (0.0, front_y, 0.090),
    )
    add_body_box(
        "front_slot_right_outer",
        (0.212, shell_thickness, 0.036),
        (0.344, front_y, 0.090),
    )
    add_body_box(
        "front_upper_panel",
        (canopy_width, shell_thickness, 0.105),
        (0.0, front_y, 0.1725),
    )
    body.visual(
        Box((canopy_width, shell_thickness, 0.140)),
        origin=Origin(xyz=(0.0, 0.225, 0.181), rpy=(math.radians(19.0), 0.0, 0.0)),
        material=stainless,
        name="sloped_front_fascia",
    )
    add_body_box(
        "underside_filter_panel",
        (canopy_width - 2.0 * shell_thickness, 0.400, 0.008),
        (0.0, -0.025, 0.014),
        material=filter_aluminum,
    )
    for rib_index, rib_x in enumerate((-0.22, 0.0, 0.22)):
        body.visual(
            Box((0.010, 0.360, 0.012)),
            origin=Origin(xyz=(rib_x, -0.025, 0.024)),
            material=satin_dark,
            name=f"filter_rib_{rib_index}",
        )

    chimney_front_y = -half_depth + chimney_depth - chimney_thickness * 0.5
    chimney_back_y = -half_depth + chimney_thickness * 0.5
    chimney_side_x = chimney_width * 0.5 - chimney_thickness * 0.5
    chimney_center_z = canopy_height + chimney_height * 0.5
    chimney_top_z = canopy_height + chimney_height - chimney_thickness * 0.5
    chimney_y_center = -half_depth + chimney_depth * 0.5

    add_body_box(
        "chimney_front",
        (chimney_width, chimney_thickness, chimney_height),
        (0.0, chimney_front_y, chimney_center_z),
    )
    add_body_box(
        "chimney_back",
        (chimney_width, chimney_thickness, chimney_height),
        (0.0, chimney_back_y, chimney_center_z),
    )
    add_body_box(
        "chimney_left",
        (chimney_thickness, chimney_depth, chimney_height),
        (-chimney_side_x, chimney_y_center, chimney_center_z),
    )
    add_body_box(
        "chimney_right",
        (chimney_thickness, chimney_depth, chimney_height),
        (chimney_side_x, chimney_y_center, chimney_center_z),
    )
    add_body_box(
        "chimney_cap",
        (chimney_width, chimney_depth, chimney_thickness),
        (0.0, chimney_y_center, chimney_top_z),
    )
    add_body_box(
        "rear_transition_plate",
        (chimney_width + 0.06, 0.10, shell_thickness),
        (0.0, -0.18, canopy_height - shell_thickness * 0.5),
        material=satin_dark,
    )

    body.inertial = Inertial.from_geometry(
        Box((canopy_width, canopy_depth, canopy_height + chimney_height)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, (canopy_height + chimney_height) * 0.5)),
    )

    knob_radius = 0.024
    knob_depth = 0.018
    shaft_radius = 0.009
    shaft_depth = 0.012
    knob_z = 0.030
    knob_joint_y = half_depth

    knob_parts = []
    for index, knob_x in enumerate(control_knob_xs):
        name = ("left", "center", "right")[index]
        knob_part = model.part(f"{name}_knob")
        knob_part.visual(
            Cylinder(radius=shaft_radius, length=shaft_depth),
            origin=Origin(xyz=(0.0, shaft_depth * 0.5, 0.0), rpy=(-math.pi * 0.5, 0.0, 0.0)),
            material=satin_dark,
            name="shaft",
        )
        knob_part.visual(
            Cylinder(radius=knob_radius, length=knob_depth),
            origin=Origin(
                xyz=(0.0, shaft_depth + knob_depth * 0.5, 0.0),
                rpy=(-math.pi * 0.5, 0.0, 0.0),
            ),
            material=knob_black,
            name="knob_body",
        )
        knob_part.visual(
            Box((0.006, 0.003, 0.010)),
            origin=Origin(xyz=(0.0, shaft_depth + knob_depth + 0.0015, 0.010)),
            material=button_face,
            name="indicator",
        )
        knob_part.inertial = Inertial.from_geometry(
            Cylinder(radius=knob_radius, length=shaft_depth + knob_depth),
            mass=0.18,
            origin=Origin(xyz=(0.0, 0.015, 0.0)),
        )
        model.articulation(
            f"hood_body_to_{name}_knob",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=knob_part,
            origin=Origin(xyz=(knob_x, knob_joint_y, knob_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.6, velocity=4.5),
        )
        knob_parts.append(knob_part)

    button_size = 0.036
    button_depth = 0.024
    button_z = 0.090
    button_rest_y = half_depth

    for index, button_x in enumerate(button_xs):
        name = ("left", "right")[index]
        button_part = model.part(f"{name}_button")
        button_part.visual(
            Box((button_size, button_depth, button_size)),
            origin=Origin(),
            material=button_black,
            name="button_body",
        )
        button_part.visual(
            Box((0.026, 0.003, 0.026)),
            origin=Origin(xyz=(0.0, button_depth * 0.5 + 0.0015, 0.0)),
            material=button_face,
            name="button_cap",
        )
        button_part.inertial = Inertial.from_geometry(
            Box((button_size, button_depth, button_size)),
            mass=0.08,
            origin=Origin(),
        )
        model.articulation(
            f"hood_body_to_{name}_button",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button_part,
            origin=Origin(xyz=(button_x, button_rest_y, button_z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.08,
                lower=0.0,
                upper=0.008,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    body = object_model.get_part("hood_body")
    left_knob = object_model.get_part("left_knob")
    center_knob = object_model.get_part("center_knob")
    right_knob = object_model.get_part("right_knob")
    left_button = object_model.get_part("left_button")
    right_button = object_model.get_part("right_button")

    left_knob_joint = object_model.get_articulation("hood_body_to_left_knob")
    center_knob_joint = object_model.get_articulation("hood_body_to_center_knob")
    right_knob_joint = object_model.get_articulation("hood_body_to_right_knob")
    left_button_joint = object_model.get_articulation("hood_body_to_left_button")
    right_button_joint = object_model.get_articulation("hood_body_to_right_button")

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
    ctx.check(
        "expected_control_articulation_count",
        len(getattr(object_model, "articulations", ())) == 5,
        details="Only the three knobs and two push-buttons should articulate.",
    )

    body_aabb = ctx.part_world_aabb(body)
    if body_aabb is None:
        ctx.fail("hood_body_has_aabb", "Range hood body has no measurable world AABB.")
    else:
        minimum, maximum = body_aabb
        width = maximum[0] - minimum[0]
        depth = maximum[1] - minimum[1]
        height = maximum[2] - minimum[2]
        ctx.check(
            "hood_body_overall_size",
            0.88 <= width <= 0.92 and 0.50 <= depth <= 0.53 and 0.98 <= height <= 1.01,
            details=(
                f"Expected a full-size chimney hood around 0.90 m wide, 0.52 m deep, "
                f"and 1.00 m tall, got {width:.3f} x {depth:.3f} x {height:.3f} m."
            ),
        )

    chimney_front_aabb = ctx.part_element_world_aabb(body, elem="chimney_front")
    canopy_top_aabb = ctx.part_element_world_aabb(body, elem="canopy_top")
    if chimney_front_aabb is None or canopy_top_aabb is None:
        ctx.fail("chimney_and_canopy_visuals_exist", "Expected named chimney and canopy visuals.")
    else:
        chimney_min, chimney_max = chimney_front_aabb
        canopy_min, canopy_max = canopy_top_aabb
        chimney_width = chimney_max[0] - chimney_min[0]
        canopy_width = canopy_max[0] - canopy_min[0]
        ctx.check(
            "chimney_is_narrower_than_canopy",
            0.34 <= chimney_width <= 0.38 and chimney_width < canopy_width,
            details=(
                f"Expected a centered rectangular chimney narrower than the lower canopy, "
                f"got chimney width {chimney_width:.3f} m and canopy width {canopy_width:.3f} m."
            ),
        )
        ctx.check(
            "chimney_starts_above_canopy",
            abs(chimney_min[2] - canopy_max[2]) <= 0.0015,
            details=(
                f"Expected chimney base to sit directly on the canopy top, "
                f"got z gap {chimney_min[2] - canopy_max[2]:.4f} m."
            ),
        )

    for knob in (left_knob, center_knob, right_knob):
        ctx.expect_contact(knob, body, name=f"{knob.name}_mounted_contact")
        ctx.expect_overlap(knob, body, axes="xz", min_overlap=0.030, name=f"{knob.name}_front_alignment")

    for button in (left_button, right_button):
        ctx.expect_contact(button, body, name=f"{button.name}_guide_contact")
        ctx.expect_overlap(button, body, axes="xz", min_overlap=0.034, name=f"{button.name}_slot_alignment")

    ctx.expect_origin_gap(center_knob, left_knob, axis="x", min_gap=0.219, max_gap=0.221, name="left_to_center_knob_spacing")
    ctx.expect_origin_gap(right_knob, center_knob, axis="x", min_gap=0.219, max_gap=0.221, name="center_to_right_knob_spacing")
    ctx.expect_origin_distance(left_button, left_knob, axes="x", max_dist=0.001, name="left_button_over_left_knob")
    ctx.expect_origin_distance(right_button, right_knob, axes="x", max_dist=0.001, name="right_button_over_right_knob")
    ctx.expect_origin_gap(left_button, left_knob, axis="z", min_gap=0.058, max_gap=0.062, name="left_button_above_left_knob")
    ctx.expect_origin_gap(right_button, right_knob, axis="z", min_gap=0.058, max_gap=0.062, name="right_button_above_right_knob")

    for joint in (left_knob_joint, center_knob_joint, right_knob_joint):
        ctx.check(
            f"{joint.name}_is_continuous_y_axis",
            joint.joint_type == ArticulationType.CONTINUOUS and tuple(joint.axis) == (0.0, 1.0, 0.0),
            details=f"Expected {joint.name} to be a continuous rotary knob joint on the front-to-back axis.",
        )
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name}_has_unbounded_rotation",
            limits is not None and limits.lower is None and limits.upper is None,
            details=f"Expected continuous knob joint {joint.name} to omit lower/upper motion bounds.",
        )

    for joint in (left_button_joint, right_button_joint):
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name}_is_prismatic_inward",
            joint.joint_type == ArticulationType.PRISMATIC and tuple(joint.axis) == (0.0, -1.0, 0.0),
            details=f"Expected {joint.name} to be an inward-travel prismatic button joint.",
        )
        ctx.check(
            f"{joint.name}_travel_range",
            limits is not None and limits.lower == 0.0 and limits.upper is not None and 0.007 <= limits.upper <= 0.009,
            details=f"Expected {joint.name} travel to be a short inward press of about 8 mm.",
        )

    left_button_rest_y = ctx.part_world_position(left_button)[1]
    right_button_rest_y = ctx.part_world_position(right_button)[1]

    for joint in (left_button_joint, right_button_joint):
        limits = joint.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({joint: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint.name}_lower_no_floating")
                button_part = object_model.get_part(joint.child)
                ctx.expect_contact(button_part, body, name=f"{joint.name}_lower_contact")
            with ctx.pose({joint: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint.name}_upper_no_floating")
                button_part = object_model.get_part(joint.child)
                ctx.expect_contact(button_part, body, name=f"{joint.name}_upper_contact")

    with ctx.pose({left_button_joint: left_button_joint.motion_limits.upper, right_button_joint: right_button_joint.motion_limits.upper}):
        left_button_pressed_y = ctx.part_world_position(left_button)[1]
        right_button_pressed_y = ctx.part_world_position(right_button)[1]
        ctx.check(
            "left_button_moves_inward",
            0.007 <= left_button_rest_y - left_button_pressed_y <= 0.009,
            details=(
                f"Expected left button to move inward about 8 mm, "
                f"got {left_button_rest_y - left_button_pressed_y:.4f} m."
            ),
        )
        ctx.check(
            "right_button_moves_inward",
            0.007 <= right_button_rest_y - right_button_pressed_y <= 0.009,
            details=(
                f"Expected right button to move inward about 8 mm, "
                f"got {right_button_rest_y - right_button_pressed_y:.4f} m."
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
