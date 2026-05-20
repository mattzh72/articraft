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
    model = ArticulatedObject(name="chimney_range_hood", assets=ASSETS)

    stainless = model.material("stainless", rgba=(0.78, 0.80, 0.82, 1.0))
    satin_dark = model.material("satin_dark", rgba=(0.28, 0.29, 0.31, 1.0))
    filter_metal = model.material("filter_metal", rgba=(0.56, 0.59, 0.62, 1.0))
    button_black = model.material("button_black", rgba=(0.16, 0.17, 0.18, 1.0))

    canopy_width = 0.90
    canopy_depth = 0.50
    canopy_height = 0.27
    front_band_height = 0.055
    front_band_depth = 0.020
    front_face_y = -canopy_depth / 2.0
    front_band_back_y = front_face_y + front_band_depth
    top_front_y = -0.05
    top_depth = canopy_depth / 2.0 - top_front_y
    side_thickness = 0.012
    back_thickness = 0.012
    top_thickness = 0.015
    front_panel_thickness = 0.014

    button_width = 0.022
    button_height = 0.012
    button_face_depth = 0.004
    button_body_depth = 0.010
    button_travel = 0.006
    button_x_positions = (-0.172, -0.128, -0.022, 0.022, 0.128, 0.172)
    button_z = front_band_height * 0.5

    def _build_side_panel_mesh(mesh_name: str):
        side_profile = [
            (-canopy_depth / 2.0, 0.0),
            (canopy_depth / 2.0, 0.0),
            (canopy_depth / 2.0, canopy_height),
            (top_front_y, canopy_height),
            (front_band_back_y, front_band_height),
        ]
        geom = ExtrudeGeometry(side_profile, side_thickness, center=True, closed=True)
        geom.rotate_x(math.pi / 2.0)
        geom.rotate_z(math.pi / 2.0)
        return mesh_from_geometry(geom, ASSETS.mesh_path(mesh_name))

    hood_body = model.part("hood_body")

    side_panel_mesh = _build_side_panel_mesh("range_hood_canopy_side.obj")

    control_strip_margin = (front_band_height - button_height) / 2.0
    opening_half_width = button_width / 2.0
    strip_edge_x = canopy_width / 2.0
    segment_edges = [-strip_edge_x]
    for button_x in button_x_positions:
        segment_edges.extend((button_x - opening_half_width, button_x + opening_half_width))
    segment_edges.append(strip_edge_x)

    hood_body.visual(
        Box((canopy_width, front_band_depth, control_strip_margin)),
        origin=Origin(
            xyz=(
                0.0,
                front_face_y + front_band_depth / 2.0,
                front_band_height - control_strip_margin / 2.0,
            )
        ),
        material=stainless,
        name="control_strip",
    )
    hood_body.visual(
        Box((canopy_width, front_band_depth, control_strip_margin)),
        origin=Origin(
            xyz=(
                0.0,
                front_face_y + front_band_depth / 2.0,
                control_strip_margin / 2.0,
            )
        ),
        material=stainless,
        name="control_strip_lower",
    )
    for segment_index in range(0, len(segment_edges) - 1, 2):
        segment_width = segment_edges[segment_index + 1] - segment_edges[segment_index]
        hood_body.visual(
            Box((segment_width, front_band_depth, button_height)),
            origin=Origin(
                xyz=(
                    (segment_edges[segment_index] + segment_edges[segment_index + 1]) / 2.0,
                    front_face_y + front_band_depth / 2.0,
                    button_z,
                )
            ),
            material=stainless,
            name=f"control_strip_segment_{segment_index // 2}",
        )

    front_slope_angle = math.atan2(top_front_y - front_band_back_y, canopy_height - front_band_height)
    front_panel_length = math.hypot(top_front_y - front_band_back_y, canopy_height - front_band_height)
    hood_body.visual(
        Box((canopy_width, front_panel_thickness, front_panel_length)),
        origin=Origin(
            xyz=(
                0.0,
                (front_band_back_y + top_front_y) / 2.0,
                (front_band_height + canopy_height) / 2.0,
            ),
            rpy=(-front_slope_angle, 0.0, 0.0),
        ),
        material=stainless,
        name="front_upper_panel",
    )
    hood_body.visual(
        side_panel_mesh,
        origin=Origin(xyz=(-canopy_width / 2.0 + side_thickness / 2.0, 0.0, 0.0)),
        material=stainless,
        name="left_canopy_side",
    )
    hood_body.visual(
        side_panel_mesh,
        origin=Origin(xyz=(canopy_width / 2.0 - side_thickness / 2.0, 0.0, 0.0)),
        material=stainless,
        name="right_canopy_side",
    )
    hood_body.visual(
        Box((canopy_width, back_thickness, canopy_height)),
        origin=Origin(
            xyz=(
                0.0,
                canopy_depth / 2.0 - back_thickness / 2.0,
                canopy_height / 2.0,
            )
        ),
        material=stainless,
        name="back_panel",
    )
    hood_body.visual(
        Box((canopy_width, top_depth, top_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                (top_front_y + canopy_depth / 2.0) / 2.0,
                canopy_height - top_thickness / 2.0,
            )
        ),
        material=stainless,
        name="canopy_top",
    )

    rail_height = 0.012
    rail_depth = 0.012
    filter_depth = 0.445
    filter_center_y = 0.0035
    filter_width = 0.420
    filter_thickness = 0.008
    side_rail_x = 0.432
    front_rail_y = -0.225
    back_rail_y = 0.232

    hood_body.visual(
        Box((0.876, rail_depth, rail_height)),
        origin=Origin(xyz=(0.0, front_rail_y, rail_height / 2.0)),
        material=satin_dark,
        name="front_filter_rail",
    )
    hood_body.visual(
        Box((0.876, rail_depth, rail_height)),
        origin=Origin(xyz=(0.0, back_rail_y, rail_height / 2.0)),
        material=satin_dark,
        name="back_filter_rail",
    )
    hood_body.visual(
        Box((0.012, 0.457, rail_height)),
        origin=Origin(xyz=(-side_rail_x, filter_center_y, rail_height / 2.0)),
        material=satin_dark,
        name="left_filter_rail",
    )
    hood_body.visual(
        Box((0.012, 0.457, rail_height)),
        origin=Origin(xyz=(side_rail_x, filter_center_y, rail_height / 2.0)),
        material=satin_dark,
        name="right_filter_rail",
    )
    hood_body.visual(
        Box((0.012, filter_depth, rail_height)),
        origin=Origin(xyz=(0.0, filter_center_y, rail_height / 2.0)),
        material=satin_dark,
        name="center_filter_divider",
    )
    hood_body.visual(
        Box((filter_width, filter_depth, filter_thickness)),
        origin=Origin(xyz=(-0.216, filter_center_y, filter_thickness / 2.0)),
        material=filter_metal,
        name="left_filter",
    )
    hood_body.visual(
        Box((filter_width, filter_depth, filter_thickness)),
        origin=Origin(xyz=(0.216, filter_center_y, filter_thickness / 2.0)),
        material=filter_metal,
        name="right_filter",
    )

    stack_outer = 0.30
    stack_wall = 0.012
    stack_height = 0.68
    stack_center_y = (top_front_y + canopy_depth / 2.0) / 2.0
    stack_center_z = canopy_height + stack_height / 2.0

    hood_body.visual(
        Box((stack_outer, stack_wall, stack_height)),
        origin=Origin(
            xyz=(
                0.0,
                stack_center_y - stack_outer / 2.0 + stack_wall / 2.0,
                stack_center_z,
            )
        ),
        material=stainless,
        name="chimney_front",
    )
    hood_body.visual(
        Box((stack_outer, stack_wall, stack_height)),
        origin=Origin(
            xyz=(
                0.0,
                stack_center_y + stack_outer / 2.0 - stack_wall / 2.0,
                stack_center_z,
            )
        ),
        material=stainless,
        name="chimney_back",
    )
    hood_body.visual(
        Box((stack_wall, stack_outer - 2.0 * stack_wall, stack_height)),
        origin=Origin(
            xyz=(
                -stack_outer / 2.0 + stack_wall / 2.0,
                stack_center_y,
                stack_center_z,
            )
        ),
        material=stainless,
        name="chimney_left",
    )
    hood_body.visual(
        Box((stack_wall, stack_outer - 2.0 * stack_wall, stack_height)),
        origin=Origin(
            xyz=(
                stack_outer / 2.0 - stack_wall / 2.0,
                stack_center_y,
                stack_center_z,
            )
        ),
        material=stainless,
        name="chimney_right",
    )

    hood_body.inertial = Inertial.from_geometry(
        Box((canopy_width, canopy_depth, canopy_height + stack_height)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, (canopy_height + stack_height) / 2.0)),
    )

    for index, button_x in enumerate(button_x_positions, start=1):
        button = model.part(f"button_{index}")
        button.visual(
            Box((button_width, button_face_depth, button_height)),
            origin=Origin(xyz=(0.0, -button_face_depth / 2.0, 0.0)),
            material=button_black,
            name="button_face",
        )
        button.visual(
            Box((button_width, button_body_depth, button_height)),
            origin=Origin(xyz=(0.0, button_body_depth / 2.0, 0.0)),
            material=button_black,
            name="button_body",
        )
        button.inertial = Inertial.from_geometry(
            Box((button_width, button_face_depth + button_body_depth, button_height)),
            mass=0.015,
            origin=Origin(xyz=(0.0, (button_body_depth - button_face_depth) / 2.0, 0.0)),
        )
        model.articulation(
            f"hood_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=hood_body,
            child=button,
            origin=Origin(xyz=(button_x, front_face_y, button_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=0.05,
                lower=0.0,
                upper=button_travel,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    hood_body = object_model.get_part("hood_body")
    buttons = [object_model.get_part(f"button_{index}") for index in range(1, 7)]
    joints = [object_model.get_articulation(f"hood_to_button_{index}") for index in range(1, 7)]
    expected_button_z = 0.055 * 0.5

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_isolated_parts(max_pose_samples=24, name="buttons_supported_across_travel")
    ctx.fail_if_articulation_overlaps(max_pose_samples=24, name="button_travel_clearance")

    required_visuals = {
        "control_strip",
        "front_upper_panel",
        "left_canopy_side",
        "right_canopy_side",
        "back_panel",
        "canopy_top",
        "left_filter",
        "right_filter",
        "chimney_front",
        "chimney_back",
        "chimney_left",
        "chimney_right",
    }
    hood_visual_names = {visual.name for visual in hood_body.visuals if visual.name is not None}
    ctx.check(
        "hood_visual_features_present",
        required_visuals.issubset(hood_visual_names),
        details=f"missing visuals: {sorted(required_visuals - hood_visual_names)}",
    )

    control_aabb = ctx.part_element_world_aabb(hood_body, elem="control_strip")
    top_aabb = ctx.part_element_world_aabb(hood_body, elem="canopy_top")
    chimney_front_aabb = ctx.part_element_world_aabb(hood_body, elem="chimney_front")
    chimney_back_aabb = ctx.part_element_world_aabb(hood_body, elem="chimney_back")
    chimney_left_aabb = ctx.part_element_world_aabb(hood_body, elem="chimney_left")
    chimney_right_aabb = ctx.part_element_world_aabb(hood_body, elem="chimney_right")

    assert control_aabb is not None
    assert top_aabb is not None
    assert chimney_front_aabb is not None
    assert chimney_back_aabb is not None
    assert chimney_left_aabb is not None
    assert chimney_right_aabb is not None

    canopy_taper = top_aabb[0][1] - control_aabb[1][1]
    ctx.check(
        "canopy_reads_as_trapezoid",
        canopy_taper > 0.15,
        details=f"top front offset {canopy_taper:.4f} m should sit well behind the front strip",
    )

    chimney_outer_width = chimney_right_aabb[1][0] - chimney_left_aabb[0][0]
    chimney_outer_depth = chimney_back_aabb[1][1] - chimney_front_aabb[0][1]
    ctx.check(
        "chimney_stack_is_square",
        abs(chimney_outer_width - chimney_outer_depth) <= 0.005
        and 0.29 <= chimney_outer_width <= 0.31,
        details=(
            f"chimney outer size width={chimney_outer_width:.4f} "
            f"depth={chimney_outer_depth:.4f}"
        ),
    )

    button_positions: list[float] = []
    for button, joint in zip(buttons, joints):
        limits = joint.motion_limits
        assert limits is not None
        assert limits.lower is not None
        assert limits.upper is not None

        ctx.check(
            f"{joint.name}_is_prismatic",
            joint.joint_type == ArticulationType.PRISMATIC and tuple(joint.axis) == (0.0, 1.0, 0.0),
            details=f"type={joint.joint_type} axis={joint.axis}",
        )

        rest_pos = ctx.part_world_position(button)
        assert rest_pos is not None
        button_positions.append(rest_pos[0])

        with ctx.pose({joint: limits.lower}):
            ctx.expect_contact(button, hood_body, name=f"{joint.name}_rest_contact")
            ctx.expect_gap(
                hood_body,
                button,
                axis="y",
                negative_elem="button_face",
                max_gap=0.0005,
                max_penetration=0.0005,
                name=f"{joint.name}_rest_front_seating",
            )
            ctx.check(
                f"{joint.name}_rest_height_centered",
                abs(rest_pos[2] - expected_button_z) <= 1e-6,
                details=f"button center z={rest_pos[2]:.4f}, expected={expected_button_z:.4f}",
            )

        with ctx.pose({joint: limits.upper}):
            ctx.expect_contact(button, hood_body, name=f"{joint.name}_pressed_contact")
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_pressed_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{joint.name}_pressed_no_floating")
            pressed_pos = ctx.part_world_position(button)
            assert pressed_pos is not None
            ctx.check(
                f"{joint.name}_presses_straight_in",
                abs((pressed_pos[1] - rest_pos[1]) - limits.upper) <= 0.0005
                and abs(pressed_pos[0] - rest_pos[0]) <= 1e-6
                and abs(pressed_pos[2] - rest_pos[2]) <= 1e-6,
                details=(
                    f"rest={rest_pos}, pressed={pressed_pos}, expected_y_shift={limits.upper:.4f}"
                ),
            )

    button_positions.sort()
    button_gaps = [button_positions[index + 1] - button_positions[index] for index in range(5)]
    narrow_gaps = (button_gaps[0], button_gaps[2], button_gaps[4])
    wide_gaps = (button_gaps[1], button_gaps[3])
    ctx.check(
        "buttons_grouped_in_three_pairs",
        max(narrow_gaps) - min(narrow_gaps) <= 0.003
        and min(wide_gaps) > max(narrow_gaps) + 0.04,
        details=f"button x gaps={button_gaps}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
