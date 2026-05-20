from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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

ASSETS = AssetContext.from_script(__file__)


def _canopy_height_at_y(
    y_pos: float,
    *,
    hood_depth: float,
    front_height: float,
    rear_height: float,
) -> float:
    blend = (y_pos + hood_depth / 2.0) / hood_depth
    return rear_height + (front_height - rear_height) * blend


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_range_hood", assets=ASSETS)

    stainless = model.material("stainless", rgba=(0.80, 0.81, 0.82, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.18, 0.19, 1.0))
    filter_gray = model.material("filter_gray", rgba=(0.35, 0.36, 0.38, 1.0))

    hood_width = 0.90
    hood_depth = 0.50
    front_height = 0.07
    rear_height = 0.18
    shell_thickness = 0.008
    front_face_y = hood_depth / 2.0
    top_angle = -0.22
    top_length = 0.52

    chimney_width = 0.32
    chimney_depth = 0.24
    chimney_height = 0.72
    chimney_center_y = -hood_depth / 2.0 + chimney_depth / 2.0

    frame_depth = 0.022
    cluster_width = 0.044
    cluster_height = 0.044
    cluster_center_x = 0.345
    cluster_center_z = 0.035
    cluster_left = cluster_center_x - cluster_width / 2.0
    cluster_right = cluster_center_x + cluster_width / 2.0
    cluster_bottom = cluster_center_z - cluster_height / 2.0
    cluster_top = cluster_center_z + cluster_height / 2.0
    frame_border = 0.003
    frame_mid = 0.006

    button_size = 0.016
    button_cap_depth = 0.006
    button_stem_depth = 0.024
    button_travel = 0.004

    knob_radius = 0.013
    knob_depth = 0.018
    knob_z = 0.033
    knob_x_left = -0.365
    knob_x_right = -0.335
    knob_panel_width = 0.082
    knob_panel_height = 0.042
    knob_panel_depth = 0.004

    housing = model.part("housing")

    side_segment_count = 12
    side_segment_depth = hood_depth / side_segment_count
    for side_name, x_pos in (
        ("right", hood_width / 2.0 - shell_thickness / 2.0),
        ("left", -hood_width / 2.0 + shell_thickness / 2.0),
    ):
        for segment_index in range(side_segment_count):
            segment_y = -hood_depth / 2.0 + side_segment_depth * (segment_index + 0.5)
            segment_height = _canopy_height_at_y(
                segment_y + side_segment_depth / 2.0,
                hood_depth=hood_depth,
                front_height=front_height,
                rear_height=rear_height,
            )
            housing.visual(
                Box((shell_thickness, side_segment_depth, segment_height)),
                origin=Origin(xyz=(x_pos, segment_y, segment_height / 2.0)),
                material=stainless,
                name=f"side_shell_{side_name}_{segment_index}",
            )
    housing.visual(
        Box((hood_width, top_length, shell_thickness)),
        origin=Origin(
            xyz=(0.0, 0.0, (front_height + rear_height) / 2.0),
            rpy=(top_angle, 0.0, 0.0),
        ),
        material=stainless,
        name="top_panel",
    )
    housing.visual(
        Box((hood_width, shell_thickness, rear_height)),
        origin=Origin(
            xyz=(0.0, -hood_depth / 2.0 + shell_thickness / 2.0, rear_height / 2.0)
        ),
        material=stainless,
        name="rear_panel",
    )

    housing.visual(
        Box((cluster_left + hood_width / 2.0, shell_thickness, front_height)),
        origin=Origin(
            xyz=(
                (-hood_width / 2.0 + cluster_left) / 2.0,
                front_face_y - shell_thickness / 2.0,
                front_height / 2.0,
            )
        ),
        material=stainless,
        name="front_valance_left",
    )
    housing.visual(
        Box((hood_width / 2.0 - cluster_right, shell_thickness, front_height)),
        origin=Origin(
            xyz=(
                (cluster_right + hood_width / 2.0) / 2.0,
                front_face_y - shell_thickness / 2.0,
                front_height / 2.0,
            )
        ),
        material=stainless,
        name="front_valance_right",
    )
    housing.visual(
        Box((cluster_width, shell_thickness, front_height - cluster_top)),
        origin=Origin(
            xyz=(
                cluster_center_x,
                front_face_y - shell_thickness / 2.0,
                cluster_top + (front_height - cluster_top) / 2.0,
            )
        ),
        material=stainless,
        name="front_cluster_fill_top",
    )
    housing.visual(
        Box((cluster_width, shell_thickness, cluster_bottom)),
        origin=Origin(
            xyz=(
                cluster_center_x,
                front_face_y - shell_thickness / 2.0,
                cluster_bottom / 2.0,
            )
        ),
        material=stainless,
        name="front_cluster_fill_bottom",
    )
    housing.visual(
        Box((knob_panel_width, knob_panel_depth, knob_panel_height)),
        origin=Origin(
            xyz=(
                (knob_x_left + knob_x_right) / 2.0,
                front_face_y - knob_panel_depth / 2.0,
                knob_z,
            )
        ),
        material=charcoal,
        name="knob_fascia",
    )

    frame_center_y = front_face_y - frame_depth / 2.0
    housing.visual(
        Box((cluster_width, frame_depth, frame_border)),
        origin=Origin(
            xyz=(cluster_center_x, frame_center_y, cluster_top - frame_border / 2.0)
        ),
        material=charcoal,
        name="button_frame_top",
    )
    housing.visual(
        Box((cluster_width, frame_depth, frame_border)),
        origin=Origin(
            xyz=(
                cluster_center_x,
                frame_center_y,
                cluster_bottom + frame_border / 2.0,
            )
        ),
        material=charcoal,
        name="button_frame_bottom",
    )
    housing.visual(
        Box((frame_border, frame_depth, cluster_height)),
        origin=Origin(
            xyz=(cluster_left + frame_border / 2.0, frame_center_y, cluster_center_z)
        ),
        material=charcoal,
        name="button_frame_left",
    )
    housing.visual(
        Box((frame_border, frame_depth, cluster_height)),
        origin=Origin(
            xyz=(cluster_right - frame_border / 2.0, frame_center_y, cluster_center_z)
        ),
        material=charcoal,
        name="button_frame_right",
    )
    housing.visual(
        Box((frame_mid, frame_depth, cluster_height)),
        origin=Origin(xyz=(cluster_center_x, frame_center_y, cluster_center_z)),
        material=charcoal,
        name="button_frame_mid_vertical",
    )
    housing.visual(
        Box((cluster_width, frame_depth, frame_mid)),
        origin=Origin(xyz=(cluster_center_x, frame_center_y, cluster_center_z)),
        material=charcoal,
        name="button_frame_mid_horizontal",
    )

    bottom_rail_depth = 0.03
    filter_depth = hood_depth - 2.0 * bottom_rail_depth
    center_rib_width = 0.016
    filter_width = (hood_width - 2.0 * shell_thickness - center_rib_width) / 2.0
    filter_height = 0.006
    filter_z = filter_height / 2.0
    filter_center_y = 0.0
    filter_offset_x = center_rib_width / 2.0 + filter_width / 2.0

    housing.visual(
        Box((hood_width, bottom_rail_depth, shell_thickness)),
        origin=Origin(
            xyz=(0.0, hood_depth / 2.0 - bottom_rail_depth / 2.0, shell_thickness / 2.0)
        ),
        material=stainless,
        name="bottom_front_rail",
    )
    housing.visual(
        Box((hood_width, bottom_rail_depth, shell_thickness)),
        origin=Origin(
            xyz=(0.0, -hood_depth / 2.0 + bottom_rail_depth / 2.0, shell_thickness / 2.0)
        ),
        material=stainless,
        name="bottom_rear_rail",
    )
    housing.visual(
        Box((shell_thickness, hood_depth, shell_thickness)),
        origin=Origin(
            xyz=(hood_width / 2.0 - shell_thickness / 2.0, 0.0, shell_thickness / 2.0)
        ),
        material=stainless,
        name="bottom_right_rail",
    )
    housing.visual(
        Box((shell_thickness, hood_depth, shell_thickness)),
        origin=Origin(
            xyz=(-hood_width / 2.0 + shell_thickness / 2.0, 0.0, shell_thickness / 2.0)
        ),
        material=stainless,
        name="bottom_left_rail",
    )
    housing.visual(
        Box((center_rib_width, filter_depth, shell_thickness)),
        origin=Origin(xyz=(0.0, filter_center_y, shell_thickness / 2.0)),
        material=stainless,
        name="filter_center_rib",
    )
    housing.visual(
        Box((filter_width, filter_depth, filter_height)),
        origin=Origin(xyz=(-filter_offset_x, filter_center_y, filter_z)),
        material=filter_gray,
        name="filter_left",
    )
    housing.visual(
        Box((filter_width, filter_depth, filter_height)),
        origin=Origin(xyz=(filter_offset_x, filter_center_y, filter_z)),
        material=filter_gray,
        name="filter_right",
    )

    housing.visual(
        Box((chimney_width, chimney_depth, shell_thickness)),
        origin=Origin(
            xyz=(0.0, chimney_center_y, rear_height - shell_thickness / 2.0)
        ),
        material=stainless,
        name="chimney_base_plate",
    )
    housing.visual(
        Box((chimney_width, shell_thickness, chimney_height)),
        origin=Origin(
            xyz=(
                0.0,
                chimney_center_y + chimney_depth / 2.0 - shell_thickness / 2.0,
                rear_height + chimney_height / 2.0,
            )
        ),
        material=stainless,
        name="chimney_front",
    )
    housing.visual(
        Box((chimney_width, shell_thickness, chimney_height)),
        origin=Origin(
            xyz=(
                0.0,
                chimney_center_y - chimney_depth / 2.0 + shell_thickness / 2.0,
                rear_height + chimney_height / 2.0,
            )
        ),
        material=stainless,
        name="chimney_back",
    )
    housing.visual(
        Box((shell_thickness, chimney_depth, chimney_height)),
        origin=Origin(
            xyz=(
                chimney_width / 2.0 - shell_thickness / 2.0,
                chimney_center_y,
                rear_height + chimney_height / 2.0,
            )
        ),
        material=stainless,
        name="chimney_right",
    )
    housing.visual(
        Box((shell_thickness, chimney_depth, chimney_height)),
        origin=Origin(
            xyz=(
                -chimney_width / 2.0 + shell_thickness / 2.0,
                chimney_center_y,
                rear_height + chimney_height / 2.0,
            )
        ),
        material=stainless,
        name="chimney_left",
    )

    knob_left = model.part("knob_left")
    knob_left.visual(
        Cylinder(radius=knob_radius, length=knob_depth),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="knob_shell",
    )

    knob_right = model.part("knob_right")
    knob_right.visual(
        Cylinder(radius=knob_radius, length=knob_depth),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="knob_shell",
    )

    button_positions = {
        "button_upper_left": (cluster_center_x - 0.011, cluster_center_z + 0.011),
        "button_upper_right": (cluster_center_x + 0.011, cluster_center_z + 0.011),
        "button_lower_left": (cluster_center_x - 0.011, cluster_center_z - 0.011),
        "button_lower_right": (cluster_center_x + 0.011, cluster_center_z - 0.011),
    }
    buttons = {}
    for name, (x_pos, z_pos) in button_positions.items():
        button = model.part(name)
        button.visual(
            Box((button_size, button_cap_depth, button_size)),
            material=charcoal,
            name="face",
        )
        button.visual(
            Box((button_size, button_stem_depth, button_size)),
            origin=Origin(xyz=(0.0, -(button_cap_depth + button_stem_depth) / 2.0, 0.0)),
            material=charcoal,
            name="stem",
        )
        buttons[name] = (button, x_pos, z_pos)

    model.articulation(
        "knob_left_spin",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=knob_left,
        origin=Origin(xyz=(knob_x_left, front_face_y + knob_depth / 2.0, knob_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=8.0),
    )
    model.articulation(
        "knob_right_spin",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=knob_right,
        origin=Origin(xyz=(knob_x_right, front_face_y + knob_depth / 2.0, knob_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=8.0),
    )

    for name, (button, x_pos, z_pos) in buttons.items():
        model.articulation(
            f"{name}_press",
            ArticulationType.PRISMATIC,
            parent=housing,
            child=button,
            origin=Origin(xyz=(x_pos, front_face_y + button_cap_depth / 2.0, z_pos)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.10,
                lower=0.0,
                upper=button_travel,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    housing = object_model.get_part("housing")
    knob_left = object_model.get_part("knob_left")
    knob_right = object_model.get_part("knob_right")
    button_upper_left = object_model.get_part("button_upper_left")
    button_upper_right = object_model.get_part("button_upper_right")
    button_lower_left = object_model.get_part("button_lower_left")
    button_lower_right = object_model.get_part("button_lower_right")

    knob_left_spin = object_model.get_articulation("knob_left_spin")
    knob_right_spin = object_model.get_articulation("knob_right_spin")
    button_upper_left_press = object_model.get_articulation("button_upper_left_press")
    button_upper_right_press = object_model.get_articulation("button_upper_right_press")
    button_lower_left_press = object_model.get_articulation("button_lower_left_press")
    button_lower_right_press = object_model.get_articulation("button_lower_right_press")

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

    for joint in (knob_left_spin, knob_right_spin):
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name}_continuous_axis",
            tuple(joint.axis) == (0.0, 1.0, 0.0),
            details=f"expected front-to-back axis, got {joint.axis}",
        )
        ctx.check(
            f"{joint.name}_continuous_limits",
            limits is not None and limits.lower is None and limits.upper is None,
            details="continuous knob joints should have unbounded angular travel",
        )

    for joint in (
        button_upper_left_press,
        button_upper_right_press,
        button_lower_left_press,
        button_lower_right_press,
    ):
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name}_prismatic_axis",
            tuple(joint.axis) == (0.0, -1.0, 0.0),
            details=f"expected inward push axis, got {joint.axis}",
        )
        ctx.check(
            f"{joint.name}_travel",
            limits is not None
            and limits.lower == 0.0
            and limits.upper is not None
            and 0.003 <= limits.upper <= 0.005,
            details=f"expected short inward travel, got {limits}",
        )

    hood_aabb = ctx.part_world_aabb(housing)
    if hood_aabb is not None:
        width = hood_aabb[1][0] - hood_aabb[0][0]
        depth = hood_aabb[1][1] - hood_aabb[0][1]
        height = hood_aabb[1][2] - hood_aabb[0][2]
        ctx.check(
            "hood_realistic_width",
            0.88 <= width <= 0.92,
            details=f"expected ~0.90 m width, got {width:.4f}",
        )
        ctx.check(
            "hood_realistic_depth",
            0.48 <= depth <= 0.52,
            details=f"expected ~0.50 m depth, got {depth:.4f}",
        )
        ctx.check(
            "hood_realistic_height",
            0.88 <= height <= 0.92,
            details=f"expected ~0.90 m overall height, got {height:.4f}",
        )
    else:
        ctx.fail("hood_aabb_exists", "housing AABB was unavailable")

    ctx.expect_contact(knob_left, housing, name="knob_left_mounted")
    ctx.expect_contact(knob_right, housing, name="knob_right_mounted")
    ctx.expect_within(knob_left, housing, axes="xz", margin=0.0, name="knob_left_on_front_band")
    ctx.expect_within(knob_right, housing, axes="xz", margin=0.0, name="knob_right_on_front_band")
    ctx.expect_origin_distance(
        knob_left,
        knob_right,
        axes="x",
        min_dist=0.02,
        max_dist=0.04,
        name="knobs_close_together",
    )

    for button in (
        button_upper_left,
        button_upper_right,
        button_lower_left,
        button_lower_right,
    ):
        ctx.expect_contact(
            button,
            housing,
            elem_a="stem",
            name=f"{button.name}_stem_guided",
        )
        ctx.expect_within(
            button,
            housing,
            axes="xz",
            inner_elem="face",
            margin=0.0,
            name=f"{button.name}_face_within_front_valance",
        )

    ctx.expect_origin_distance(
        button_upper_left,
        button_upper_right,
        axes="x",
        min_dist=0.02,
        max_dist=0.024,
        name="button_columns_spacing",
    )
    ctx.expect_origin_gap(
        button_upper_left,
        button_lower_left,
        axis="z",
        min_gap=0.02,
        max_gap=0.024,
        name="button_rows_spacing",
    )
    ctx.expect_origin_distance(
        knob_right,
        button_upper_left,
        axes="x",
        min_dist=0.64,
        name="controls_left_and_right_separated",
    )

    knob_left_pos = ctx.part_world_position(knob_left)
    knob_right_pos = ctx.part_world_position(knob_right)
    button_positions = [
        ctx.part_world_position(button_upper_left),
        ctx.part_world_position(button_upper_right),
        ctx.part_world_position(button_lower_left),
        ctx.part_world_position(button_lower_right),
    ]
    ctx.check(
        "knobs_on_far_left",
        knob_left_pos is not None
        and knob_right_pos is not None
        and knob_left_pos[0] < -0.34
        and knob_right_pos[0] < -0.30,
        details=f"knob positions: left={knob_left_pos}, right={knob_right_pos}",
    )
    ctx.check(
        "buttons_on_far_right",
        all(position is not None and position[0] > 0.32 for position in button_positions),
        details=f"button positions: {button_positions}",
    )

    button_upper_limit = button_upper_left_press.motion_limits
    if button_upper_limit is not None and button_upper_limit.upper is not None:
        with ctx.pose(
            {
                button_upper_left_press: button_upper_limit.upper,
                button_upper_right_press: button_upper_limit.upper,
                button_lower_left_press: button_upper_limit.upper,
                button_lower_right_press: button_upper_limit.upper,
                knob_left_spin: pi / 2.0,
                knob_right_spin: -pi / 2.0,
            }
        ):
            ctx.fail_if_parts_overlap_in_current_pose(name="controls_operated_no_overlap")
            ctx.fail_if_isolated_parts(name="controls_operated_no_floating")
            for button in (
                button_upper_left,
                button_upper_right,
                button_lower_left,
                button_lower_right,
            ):
                ctx.expect_contact(
                    button,
                    housing,
                    elem_a="stem",
                    name=f"{button.name}_pressed_still_guided",
                )

    with ctx.pose(
        {
            knob_left_spin: 1.3,
            knob_right_spin: -1.3,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="knobs_rotated_no_overlap")
        ctx.fail_if_isolated_parts(name="knobs_rotated_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
