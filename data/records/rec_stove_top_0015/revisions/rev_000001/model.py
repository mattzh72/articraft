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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="five_burner_gas_hob_cabinet", assets=ASSETS)

    cabinet_white = model.material("cabinet_white", rgba=(0.93, 0.94, 0.95, 1.0))
    countertop_stone = model.material("countertop_stone", rgba=(0.78, 0.79, 0.80, 1.0))
    enamel_black = model.material("enamel_black", rgba=(0.10, 0.11, 0.12, 1.0))
    grate_black = model.material("grate_black", rgba=(0.16, 0.16, 0.17, 1.0))
    burner_metal = model.material("burner_metal", rgba=(0.38, 0.39, 0.41, 1.0))
    knob_black = model.material("knob_black", rgba=(0.13, 0.13, 0.14, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.73, 0.74, 0.75, 1.0))

    cabinet_width = 0.80
    cabinet_depth = 0.56
    toe_kick_height = 0.10
    cabinet_height = 0.76
    side_thickness = 0.018
    back_thickness = 0.012
    bottom_thickness = 0.018

    countertop_width = 0.90
    countertop_depth = 0.62
    countertop_thickness = 0.04
    counter_top_z = toe_kick_height + cabinet_height + countertop_thickness
    counter_mid_z = counter_top_z - countertop_thickness * 0.5

    hob_width = 0.75
    hob_depth = 0.52
    hob_pan_depth = 0.06
    hob_pan_center_z = counter_top_z - hob_pan_depth * 0.5
    hob_pan_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(hob_width, hob_depth, radius=0.032, corner_segments=10),
            hob_pan_depth,
            cap=True,
            center=True,
        ),
        ASSETS.mesh_path("hob_pan.obj"),
    )
    hob_trim_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(hob_width + 0.018, hob_depth + 0.018, radius=0.036, corner_segments=10),
            0.004,
            cap=True,
            center=True,
        ),
        ASSETS.mesh_path("hob_trim.obj"),
    )

    body = model.part("cabinet_body")

    side_panel_height = toe_kick_height + cabinet_height
    side_panel_z = side_panel_height * 0.5
    side_panel_x = cabinet_width * 0.5 - side_thickness * 0.5
    body.visual(
        Box((side_thickness, cabinet_depth, side_panel_height)),
        origin=Origin(xyz=(-side_panel_x, 0.0, side_panel_z)),
        material=cabinet_white,
        name="left_side_panel",
    )
    body.visual(
        Box((side_thickness, cabinet_depth, side_panel_height)),
        origin=Origin(xyz=(side_panel_x, 0.0, side_panel_z)),
        material=cabinet_white,
        name="right_side_panel",
    )
    body.visual(
        Box((cabinet_width - 2.0 * side_thickness, back_thickness, side_panel_height)),
        origin=Origin(
            xyz=(
                0.0,
                cabinet_depth * 0.5 - back_thickness * 0.5,
                side_panel_z,
            )
        ),
        material=cabinet_white,
        name="back_panel",
    )
    body.visual(
        Box((cabinet_width - 2.0 * side_thickness, cabinet_depth - back_thickness, bottom_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                -back_thickness * 0.5,
                toe_kick_height + bottom_thickness * 0.5,
            )
        ),
        material=cabinet_white,
        name="cabinet_floor",
    )
    body.visual(
        Box((cabinet_width - 2.0 * side_thickness, 0.018, toe_kick_height)),
        origin=Origin(
            xyz=(
                0.0,
                -(cabinet_depth * 0.5) + 0.054,
                toe_kick_height * 0.5,
            )
        ),
        material=cabinet_white,
        name="toe_kick",
    )
    body.visual(
        Box((cabinet_width - 2.0 * side_thickness, cabinet_depth - back_thickness, 0.12)),
        origin=Origin(
            xyz=(
                0.0,
                -back_thickness * 0.5,
                toe_kick_height + 0.64 + 0.06,
            )
        ),
        material=cabinet_white,
        name="upper_rail",
    )

    side_strip_width = (countertop_width - hob_width) * 0.5
    front_back_strip_depth = (countertop_depth - hob_depth) * 0.5
    side_strip_x = hob_width * 0.5 + side_strip_width * 0.5
    front_strip_y = -(hob_depth * 0.5 + front_back_strip_depth * 0.5)
    back_strip_y = -front_strip_y

    body.visual(
        Box((side_strip_width, countertop_depth, countertop_thickness)),
        origin=Origin(xyz=(-side_strip_x, 0.0, counter_mid_z)),
        material=countertop_stone,
        name="counter_left",
    )
    body.visual(
        Box((side_strip_width, countertop_depth, countertop_thickness)),
        origin=Origin(xyz=(side_strip_x, 0.0, counter_mid_z)),
        material=countertop_stone,
        name="counter_right",
    )
    body.visual(
        Box((hob_width, front_back_strip_depth, countertop_thickness)),
        origin=Origin(xyz=(0.0, front_strip_y, counter_mid_z)),
        material=countertop_stone,
        name="counter_front",
    )
    body.visual(
        Box((hob_width, front_back_strip_depth, countertop_thickness)),
        origin=Origin(xyz=(0.0, back_strip_y, counter_mid_z)),
        material=countertop_stone,
        name="counter_back",
    )
    body.visual(
        hob_trim_mesh,
        origin=Origin(xyz=(0.0, 0.0, counter_top_z - 0.002)),
        material=brushed_steel,
        name="hob_trim",
    )
    body.visual(
        hob_pan_mesh,
        origin=Origin(xyz=(0.0, 0.0, hob_pan_center_z)),
        material=enamel_black,
        name="hob_pan",
    )
    body.visual(
        Box((0.68, 0.006, 0.10)),
        origin=Origin(xyz=(0.0, -0.282, counter_top_z - 0.11)),
        material=enamel_black,
        name="control_mount_rail",
    )

    burner_specs = (
        ("burner_front_left", -0.22, -0.13, 0.038, 0.030, 0.022, 0.120, 0.014, 0.046),
        ("burner_back_left", -0.22, 0.15, 0.038, 0.030, 0.022, 0.120, 0.014, 0.046),
        ("burner_center", 0.0, 0.02, 0.060, 0.050, 0.034, 0.165, 0.018, 0.060),
        ("burner_front_right", 0.22, -0.13, 0.038, 0.030, 0.022, 0.120, 0.014, 0.046),
        ("burner_back_right", 0.22, 0.15, 0.038, 0.030, 0.022, 0.120, 0.014, 0.046),
    )
    for name_prefix, x_pos, y_pos, base_radius, crown_radius, cap_radius, trivet_span, trivet_width, foot_offset in burner_specs:
        body.visual(
            Cylinder(radius=base_radius, length=0.006),
            origin=Origin(xyz=(x_pos, y_pos, counter_top_z + 0.003)),
            material=burner_metal,
            name=f"{name_prefix}_base",
        )
        body.visual(
            Cylinder(radius=crown_radius, length=0.010),
            origin=Origin(xyz=(x_pos, y_pos, counter_top_z + 0.011)),
            material=burner_metal,
            name=f"{name_prefix}_crown",
        )
        body.visual(
            Cylinder(radius=cap_radius, length=0.006),
            origin=Origin(xyz=(x_pos, y_pos, counter_top_z + 0.019)),
            material=grate_black,
            name=f"{name_prefix}_cap",
        )
        for foot_index, (dx, dy) in enumerate(
            (
                (foot_offset, 0.0),
                (-foot_offset, 0.0),
                (0.0, foot_offset),
                (0.0, -foot_offset),
            )
        ):
            body.visual(
                Box((0.016, 0.016, 0.019)),
                origin=Origin(xyz=(x_pos + dx, y_pos + dy, counter_top_z + 0.0095)),
                material=grate_black,
                name=f"{name_prefix}_foot_{foot_index}",
            )
        body.visual(
            Box((trivet_span, trivet_width, 0.008)),
            origin=Origin(xyz=(x_pos, y_pos, counter_top_z + 0.023)),
            material=grate_black,
            name=f"{name_prefix}_trivet_x",
        )
        body.visual(
            Box((trivet_width, trivet_span, 0.008)),
            origin=Origin(xyz=(x_pos, y_pos, counter_top_z + 0.023)),
            material=grate_black,
            name=f"{name_prefix}_trivet_y",
        )

    body.inertial = Inertial.from_geometry(
        Box((countertop_width, countertop_depth, counter_top_z + 0.03)),
        mass=58.0,
        origin=Origin(xyz=(0.0, 0.0, (counter_top_z + 0.03) * 0.5)),
    )

    door_width = (cabinet_width - 0.006) * 0.5
    door_height = 0.64
    door_thickness = 0.020
    door_center_z = toe_kick_height + door_height * 0.5
    door_axis_y = -(cabinet_depth * 0.5)

    left_door = model.part("left_door")
    left_door.visual(
        Box((door_width, door_thickness, door_height)),
        origin=Origin(xyz=(door_width * 0.5, -door_thickness * 0.5, 0.0)),
        material=cabinet_white,
        name="door_panel",
    )
    left_door.visual(
        Box((0.016, 0.016, 0.18)),
        origin=Origin(xyz=(door_width - 0.055, -door_thickness - 0.008, 0.0)),
        material=brushed_steel,
        name="handle",
    )
    left_door.inertial = Inertial.from_geometry(
        Box((door_width, 0.036, door_height)),
        mass=6.2,
        origin=Origin(xyz=(door_width * 0.5, -0.018, 0.0)),
    )

    right_door = model.part("right_door")
    right_door.visual(
        Box((door_width, door_thickness, door_height)),
        origin=Origin(xyz=(-door_width * 0.5, -door_thickness * 0.5, 0.0)),
        material=cabinet_white,
        name="door_panel",
    )
    right_door.visual(
        Box((0.016, 0.016, 0.18)),
        origin=Origin(xyz=(-(door_width - 0.055), -door_thickness - 0.008, 0.0)),
        material=brushed_steel,
        name="handle",
    )
    right_door.inertial = Inertial.from_geometry(
        Box((door_width, 0.036, door_height)),
        mass=6.2,
        origin=Origin(xyz=(-door_width * 0.5, -0.018, 0.0)),
    )

    model.articulation(
        "left_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_door,
        origin=Origin(xyz=(-cabinet_width * 0.5, door_axis_y, door_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.3,
            lower=-math.radians(110.0),
            upper=0.0,
        ),
    )
    model.articulation(
        "right_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_door,
        origin=Origin(xyz=(cabinet_width * 0.5, door_axis_y, door_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.3,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )

    knob_specs = (
        ("knob_outer_left", -0.27, 0.022, 0.024, 0.008),
        ("knob_inner_left", -0.135, 0.022, 0.024, 0.008),
        ("knob_center", 0.0, 0.034, 0.030, 0.010),
        ("knob_inner_right", 0.135, 0.022, 0.024, 0.008),
        ("knob_outer_right", 0.27, 0.022, 0.024, 0.008),
    )
    knob_joint_y = -0.285
    knob_axis_z = counter_top_z - 0.105

    for knob_name, x_pos, radius, depth, shaft_radius in knob_specs:
        knob = model.part(knob_name)
        knob.visual(
            Cylinder(radius=radius, length=depth),
            origin=Origin(
                xyz=(0.0, -depth * 0.5, 0.0),
                rpy=(-math.pi * 0.5, 0.0, 0.0),
            ),
            material=knob_black,
            name="knob_body",
        )
        knob.visual(
            Box((radius * 0.32, 0.004, radius * 0.90)),
            origin=Origin(xyz=(0.0, -depth - 0.002, radius * 0.38)),
            material=brushed_steel,
            name="pointer",
        )
        knob.inertial = Inertial.from_geometry(
            Box((radius * 2.1, depth + 0.014, radius * 2.1)),
            mass=0.18 if knob_name == "knob_center" else 0.11,
            origin=Origin(xyz=(0.0, -depth * 0.35, 0.0)),
        )
        model.articulation(
            f"{knob_name}_joint",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=knob,
            origin=Origin(xyz=(x_pos, knob_joint_y, knob_axis_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.8,
                velocity=8.0,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root, seed=0)
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

    body = object_model.get_part("cabinet_body")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    knob_outer_left = object_model.get_part("knob_outer_left")
    knob_inner_left = object_model.get_part("knob_inner_left")
    knob_center = object_model.get_part("knob_center")
    knob_inner_right = object_model.get_part("knob_inner_right")
    knob_outer_right = object_model.get_part("knob_outer_right")

    left_door_hinge = object_model.get_articulation("left_door_hinge")
    right_door_hinge = object_model.get_articulation("right_door_hinge")
    knob_outer_left_joint = object_model.get_articulation("knob_outer_left_joint")
    knob_inner_left_joint = object_model.get_articulation("knob_inner_left_joint")
    knob_center_joint = object_model.get_articulation("knob_center_joint")
    knob_inner_right_joint = object_model.get_articulation("knob_inner_right_joint")
    knob_outer_right_joint = object_model.get_articulation("knob_outer_right_joint")

    knob_parts = (
        knob_outer_left,
        knob_inner_left,
        knob_center,
        knob_inner_right,
        knob_outer_right,
    )
    knob_joints = (
        knob_outer_left_joint,
        knob_inner_left_joint,
        knob_center_joint,
        knob_inner_right_joint,
        knob_outer_right_joint,
    )

    body_visual_names = {visual.name for visual in body.visuals}
    burner_cap_names = {
        "burner_front_left_cap",
        "burner_back_left_cap",
        "burner_center_cap",
        "burner_front_right_cap",
        "burner_back_right_cap",
    }
    ctx.check(
        "five_burner_caps_present",
        burner_cap_names.issubset(body_visual_names),
        details=f"missing burner caps: {sorted(burner_cap_names - body_visual_names)}",
    )

    for knob in knob_parts:
        ctx.expect_contact(knob, body, name=f"{knob.name}_contacts_hob")
    ctx.expect_contact(left_door, body, name="left_door_contacts_cabinet")
    ctx.expect_contact(right_door, body, name="right_door_contacts_cabinet")

    ctx.check(
        "left_door_axis_vertical",
        tuple(left_door_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"axis was {left_door_hinge.axis}",
    )
    ctx.check(
        "right_door_axis_vertical",
        tuple(right_door_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"axis was {right_door_hinge.axis}",
    )
    for joint in knob_joints:
        ctx.check(
            f"{joint.name}_axis_front_to_back",
            tuple(joint.axis) == (0.0, 1.0, 0.0),
            details=f"axis was {joint.axis}",
        )

    body_aabb = ctx.part_world_aabb(body)
    ctx.check("body_aabb_available", body_aabb is not None, details="cabinet body AABB missing")
    if body_aabb is not None:
        body_width = body_aabb[1][0] - body_aabb[0][0]
        body_depth = body_aabb[1][1] - body_aabb[0][1]
        ctx.check(
            "countertop_width_realistic",
            0.89 <= body_width <= 0.91,
            details=f"body width was {body_width:.4f} m",
        )
        ctx.check(
            "countertop_depth_realistic",
            0.61 <= body_depth <= 0.63,
            details=f"body depth was {body_depth:.4f} m",
        )

    counter_left_aabb = ctx.part_element_world_aabb(body, elem="counter_left")
    hob_pan_aabb = ctx.part_element_world_aabb(body, elem="hob_pan")
    ctx.check(
        "counter_and_hob_visuals_available",
        counter_left_aabb is not None and hob_pan_aabb is not None,
        details="counter_left or hob_pan visual AABB missing",
    )
    if counter_left_aabb is not None and hob_pan_aabb is not None:
        counter_top = counter_left_aabb[1][2]
        hob_top = hob_pan_aabb[1][2]
        hob_depth = hob_pan_aabb[1][2] - hob_pan_aabb[0][2]
        ctx.check(
            "hob_set_flush_with_countertop",
            abs(hob_top - counter_top) <= 0.001,
            details=f"counter top z={counter_top:.4f}, hob top z={hob_top:.4f}",
        )
        ctx.check(
            "hob_is_shallow",
            0.045 <= hob_depth <= 0.07,
            details=f"hob pan depth was {hob_depth:.4f} m",
        )

    knob_positions = [ctx.part_world_position(knob) for knob in knob_parts]
    ctx.check(
        "all_knob_positions_available",
        all(position is not None for position in knob_positions),
        details="one or more knob positions missing",
    )
    if all(position is not None for position in knob_positions):
        left_outer, left_inner, center, right_inner, right_outer = knob_positions
        assert left_outer is not None
        assert left_inner is not None
        assert center is not None
        assert right_inner is not None
        assert right_outer is not None
        ctx.check(
            "knob_layout_symmetric",
            abs(left_outer[0] + right_outer[0]) <= 0.001
            and abs(left_inner[0] + right_inner[0]) <= 0.001
            and abs(center[0]) <= 0.001
            and max(abs(position[1] - center[1]) for position in (left_outer, left_inner, right_inner, right_outer)) <= 0.001
            and max(abs(position[2] - center[2]) for position in (left_outer, left_inner, right_inner, right_outer)) <= 0.001,
            details=f"knob centers were {knob_positions}",
        )
        left_outer_spacing = left_inner[0] - left_outer[0]
        left_inner_spacing = center[0] - left_inner[0]
        right_inner_spacing = right_inner[0] - center[0]
        right_outer_spacing = right_outer[0] - right_inner[0]
        ctx.check(
            "knob_spacing_even",
            max(
                abs(left_outer_spacing - left_inner_spacing),
                abs(left_outer_spacing - right_inner_spacing),
                abs(left_outer_spacing - right_outer_spacing),
            )
            <= 0.001,
            details=(
                "adjacent knob spacings were "
                f"{left_outer_spacing:.4f}, {left_inner_spacing:.4f}, "
                f"{right_inner_spacing:.4f}, {right_outer_spacing:.4f}"
            ),
        )

    center_aabb = ctx.part_world_aabb(knob_center)
    small_aabb = ctx.part_world_aabb(knob_inner_left)
    ctx.check(
        "center_and_small_knob_aabbs_available",
        center_aabb is not None and small_aabb is not None,
        details="knob AABB missing",
    )
    if center_aabb is not None and small_aabb is not None:
        center_span_x = center_aabb[1][0] - center_aabb[0][0]
        small_span_x = small_aabb[1][0] - small_aabb[0][0]
        ctx.check(
            "center_knob_larger_than_side_knobs",
            center_span_x > small_span_x + 0.015,
            details=f"center span {center_span_x:.4f} vs small span {small_span_x:.4f}",
        )

    left_closed = ctx.part_world_aabb(left_door)
    right_closed = ctx.part_world_aabb(right_door)
    ctx.check(
        "door_aabbs_available_closed",
        left_closed is not None and right_closed is not None,
        details="closed door AABB missing",
    )
    if left_closed is not None and right_closed is not None:
        center_gap = right_closed[0][0] - left_closed[1][0]
        ctx.check(
            "closed_doors_have_small_center_gap",
            0.003 <= center_gap <= 0.009,
            details=f"door center gap was {center_gap:.4f} m",
        )

    with ctx.pose({left_door_hinge: -1.2, right_door_hinge: 1.2}):
        ctx.expect_contact(left_door, body, name="left_door_open_still_mounted")
        ctx.expect_contact(right_door, body, name="right_door_open_still_mounted")
        left_open = ctx.part_world_aabb(left_door)
        right_open = ctx.part_world_aabb(right_door)
        ctx.check(
            "door_aabbs_available_open",
            left_open is not None and right_open is not None,
            details="open door AABB missing",
        )
        if left_closed is not None and right_closed is not None and left_open is not None and right_open is not None:
            ctx.check(
                "left_door_swings_outward",
                left_open[0][1] < left_closed[0][1] - 0.12,
                details=f"closed min y {left_closed[0][1]:.4f}, open min y {left_open[0][1]:.4f}",
            )
            ctx.check(
                "right_door_swings_outward",
                right_open[0][1] < right_closed[0][1] - 0.12,
                details=f"closed min y {right_closed[0][1]:.4f}, open min y {right_open[0][1]:.4f}",
            )

    with ctx.pose(
        {
            knob_outer_left_joint: 0.8,
            knob_inner_left_joint: -0.9,
            knob_center_joint: 1.2,
            knob_inner_right_joint: -0.9,
            knob_outer_right_joint: 0.8,
        }
    ):
        for knob in knob_parts:
            ctx.expect_contact(knob, body, name=f"{knob.name}_contact_in_rotated_pose")
        rotated_center_aabb = ctx.part_world_aabb(knob_center)
        ctx.check(
            "center_knob_rotated_aabb_available",
            rotated_center_aabb is not None and center_aabb is not None,
            details="center knob AABB missing in rotated pose",
        )
        if rotated_center_aabb is not None and center_aabb is not None:
            ctx.check(
                "center_knob_pointer_moves_when_rotated",
                abs(rotated_center_aabb[1][2] - center_aabb[1][2]) > 0.003
                or abs(rotated_center_aabb[1][0] - center_aabb[1][0]) > 0.003,
                details=(
                    f"rest aabb {center_aabb}, rotated aabb {rotated_center_aabb}"
                ),
            )

    for articulated_joint in (left_door_hinge, right_door_hinge):
        limits = articulated_joint.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({articulated_joint: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{articulated_joint.name}_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{articulated_joint.name}_lower_no_floating")
            with ctx.pose({articulated_joint: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{articulated_joint.name}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{articulated_joint.name}_upper_no_floating")

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
