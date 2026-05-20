from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="split_sliding_moonroof_cassette")

    aluminum = model.material("aluminum", rgba=(0.74, 0.76, 0.78, 1.0))
    anodized = model.material("anodized_rail", rgba=(0.52, 0.54, 0.57, 1.0))
    seal_black = model.material("seal_black", rgba=(0.08, 0.08, 0.09, 1.0))
    tinted_glass = model.material("tinted_glass", rgba=(0.10, 0.15, 0.19, 0.45))

    # Overall cassette dimensions roughly match a wide dual-panel panoramic roof module.
    overall_length = 0.96
    overall_width = 0.92
    frame_height = 0.04
    front_header = 0.055
    rear_header = 0.065
    side_sill_width = 0.035
    center_mullion_width = 0.05
    track_run = overall_length - front_header - rear_header

    lane_width = (overall_width - (2.0 * side_sill_width) - center_mullion_width) / 2.0
    left_lane_center_y = -((center_mullion_width + lane_width) / 2.0)
    right_lane_center_y = (center_mullion_width + lane_width) / 2.0
    track_center_x = (-overall_length / 2.0) + front_header + (track_run / 2.0)

    tray_thickness = 0.004
    rail_width = 0.016
    rail_height = 0.014
    rail_top_z = tray_thickness + rail_height

    panel_length = 0.50
    panel_width = 0.382
    panel_runner_width = rail_width
    panel_runner_height = 0.022
    panel_end_bar_thickness = 0.020
    glass_length = 0.46
    glass_width = 0.35
    glass_thickness = 0.006

    closed_front_gap = 0.003
    open_rear_gap = 0.006
    panel_closed_x = (-overall_length / 2.0) + front_header + closed_front_gap
    panel_travel = (
        (overall_length / 2.0)
        - rear_header
        - open_rear_gap
        - panel_length
        - panel_closed_x
    )

    runner_offset_y = (panel_width / 2.0) - (panel_runner_width / 2.0)
    left_outer_rail_y = left_lane_center_y - runner_offset_y
    left_inner_rail_y = left_lane_center_y + runner_offset_y
    right_inner_rail_y = right_lane_center_y - runner_offset_y
    right_outer_rail_y = right_lane_center_y + runner_offset_y

    frame = model.part("frame")
    frame.visual(
        Box((front_header, overall_width, frame_height)),
        origin=Origin(xyz=((-overall_length / 2.0) + (front_header / 2.0), 0.0, frame_height / 2.0)),
        material=aluminum,
        name="front_header",
    )
    frame.visual(
        Box((rear_header, overall_width, frame_height)),
        origin=Origin(xyz=((overall_length / 2.0) - (rear_header / 2.0), 0.0, frame_height / 2.0)),
        material=aluminum,
        name="rear_header",
    )
    frame.visual(
        Box((track_run, side_sill_width, frame_height)),
        origin=Origin(xyz=(track_center_x, (overall_width / 2.0) - (side_sill_width / 2.0), frame_height / 2.0)),
        material=aluminum,
        name="right_sill",
    )
    frame.visual(
        Box((track_run, side_sill_width, frame_height)),
        origin=Origin(xyz=(track_center_x, -((overall_width / 2.0) - (side_sill_width / 2.0)), frame_height / 2.0)),
        material=aluminum,
        name="left_sill",
    )
    frame.visual(
        Box((track_run, center_mullion_width, frame_height)),
        origin=Origin(xyz=(track_center_x, 0.0, frame_height / 2.0)),
        material=aluminum,
        name="center_mullion",
    )

    frame.visual(
        Box((track_run, lane_width, tray_thickness)),
        origin=Origin(xyz=(track_center_x, left_lane_center_y, tray_thickness / 2.0)),
        material=seal_black,
        name="left_tray",
    )
    frame.visual(
        Box((track_run, lane_width, tray_thickness)),
        origin=Origin(xyz=(track_center_x, right_lane_center_y, tray_thickness / 2.0)),
        material=seal_black,
        name="right_tray",
    )

    frame.visual(
        Box((track_run, rail_width, rail_height)),
        origin=Origin(xyz=(track_center_x, left_outer_rail_y, tray_thickness + (rail_height / 2.0))),
        material=anodized,
        name="left_outer_rail",
    )
    frame.visual(
        Box((track_run, rail_width, rail_height)),
        origin=Origin(xyz=(track_center_x, left_inner_rail_y, tray_thickness + (rail_height / 2.0))),
        material=anodized,
        name="left_inner_rail",
    )
    frame.visual(
        Box((track_run, rail_width, rail_height)),
        origin=Origin(xyz=(track_center_x, right_inner_rail_y, tray_thickness + (rail_height / 2.0))),
        material=anodized,
        name="right_inner_rail",
    )
    frame.visual(
        Box((track_run, rail_width, rail_height)),
        origin=Origin(xyz=(track_center_x, right_outer_rail_y, tray_thickness + (rail_height / 2.0))),
        material=anodized,
        name="right_outer_rail",
    )

    def add_panel(
        part_name: str,
        *,
        is_left: bool,
        lane_center_y: float,
        inner_runner_name: str,
        outer_runner_name: str,
    ):
        panel = model.part(part_name)
        panel.visual(
            Box((panel_length, panel_runner_width, panel_runner_height)),
            origin=Origin(
                xyz=(
                    panel_length / 2.0,
                    -runner_offset_y if is_left else runner_offset_y,
                    panel_runner_height / 2.0,
                )
            ),
            material=aluminum,
            name=outer_runner_name,
        )
        panel.visual(
            Box((panel_length, panel_runner_width, panel_runner_height)),
            origin=Origin(
                xyz=(
                    panel_length / 2.0,
                    runner_offset_y if is_left else -runner_offset_y,
                    panel_runner_height / 2.0,
                )
            ),
            material=aluminum,
            name=inner_runner_name,
        )
        panel.visual(
            Box((panel_end_bar_thickness, panel_width, panel_runner_height)),
            origin=Origin(
                xyz=(panel_end_bar_thickness / 2.0, 0.0, panel_runner_height / 2.0),
            ),
            material=aluminum,
            name="front_crossbar",
        )
        panel.visual(
            Box((panel_end_bar_thickness, panel_width, panel_runner_height)),
            origin=Origin(
                xyz=(panel_length - (panel_end_bar_thickness / 2.0), 0.0, panel_runner_height / 2.0),
            ),
            material=aluminum,
            name="rear_crossbar",
        )
        panel.visual(
            Box((glass_length, glass_width, glass_thickness)),
            origin=Origin(
                xyz=(
                    panel_length / 2.0,
                    0.0,
                    panel_runner_height - 0.002 + (glass_thickness / 2.0),
                )
            ),
            material=tinted_glass,
            name="glass",
        )

        model.articulation(
            f"frame_to_{part_name}",
            ArticulationType.PRISMATIC,
            parent=frame,
            child=panel,
            origin=Origin(xyz=(panel_closed_x, lane_center_y, rail_top_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=120.0,
                velocity=0.30,
                lower=0.0,
                upper=panel_travel,
            ),
        )

    add_panel(
        "left_panel",
        is_left=True,
        lane_center_y=left_lane_center_y,
        inner_runner_name="inner_runner",
        outer_runner_name="outer_runner",
    )
    add_panel(
        "right_panel",
        is_left=False,
        lane_center_y=right_lane_center_y,
        inner_runner_name="inner_runner",
        outer_runner_name="outer_runner",
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    left_panel = object_model.get_part("left_panel")
    right_panel = object_model.get_part("right_panel")
    left_slide = object_model.get_articulation("frame_to_left_panel")
    right_slide = object_model.get_articulation("frame_to_right_panel")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

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

    left_glass = ctx.part_element_world_aabb(left_panel, elem="glass")
    right_glass = ctx.part_element_world_aabb(right_panel, elem="glass")
    same_width = (
        left_glass is not None
        and right_glass is not None
        and abs((left_glass[1][1] - left_glass[0][1]) - (right_glass[1][1] - right_glass[0][1])) <= 1e-6
    )
    ctx.check(
        "glass panels have equal width",
        same_width,
        details=f"left_glass={left_glass}, right_glass={right_glass}",
    )

    ctx.expect_contact(
        left_panel,
        frame,
        elem_a="outer_runner",
        elem_b="left_outer_rail",
        name="left panel outer runner sits on left outer rail",
    )
    ctx.expect_contact(
        left_panel,
        frame,
        elem_a="inner_runner",
        elem_b="left_inner_rail",
        name="left panel inner runner sits on left inner rail",
    )
    ctx.expect_contact(
        right_panel,
        frame,
        elem_a="outer_runner",
        elem_b="right_outer_rail",
        name="right panel outer runner sits on right outer rail",
    )
    ctx.expect_contact(
        right_panel,
        frame,
        elem_a="inner_runner",
        elem_b="right_inner_rail",
        name="right panel inner runner sits on right inner rail",
    )

    left_closed_pos = ctx.part_world_position(left_panel)
    right_closed_pos = ctx.part_world_position(right_panel)
    left_upper = left_slide.motion_limits.upper if left_slide.motion_limits is not None else None
    right_upper = right_slide.motion_limits.upper if right_slide.motion_limits is not None else None

    with ctx.pose({left_slide: left_upper}):
        ctx.expect_contact(
            left_panel,
            frame,
            elem_a="outer_runner",
            elem_b="left_outer_rail",
            name="left outer runner stays supported at full travel",
        )
        ctx.expect_contact(
            left_panel,
            frame,
            elem_a="inner_runner",
            elem_b="left_inner_rail",
            name="left inner runner stays supported at full travel",
        )
        left_open_pos = ctx.part_world_position(left_panel)
        right_while_left_open_pos = ctx.part_world_position(right_panel)
        ctx.check(
            "left panel slides rearward independently",
            left_closed_pos is not None
            and left_open_pos is not None
            and left_upper is not None
            and left_open_pos[0] > left_closed_pos[0] + max(0.25, left_upper * 0.75),
            details=f"closed={left_closed_pos}, open={left_open_pos}, upper={left_upper}",
        )
        ctx.check(
            "right panel stays put while left opens",
            right_closed_pos is not None
            and right_while_left_open_pos is not None
            and abs(right_while_left_open_pos[0] - right_closed_pos[0]) <= 1e-6,
            details=f"closed={right_closed_pos}, pose={right_while_left_open_pos}",
        )

    with ctx.pose({right_slide: right_upper}):
        ctx.expect_contact(
            right_panel,
            frame,
            elem_a="outer_runner",
            elem_b="right_outer_rail",
            name="right outer runner stays supported at full travel",
        )
        ctx.expect_contact(
            right_panel,
            frame,
            elem_a="inner_runner",
            elem_b="right_inner_rail",
            name="right inner runner stays supported at full travel",
        )
        right_open_pos = ctx.part_world_position(right_panel)
        left_while_right_open_pos = ctx.part_world_position(left_panel)
        ctx.check(
            "right panel slides rearward independently",
            right_closed_pos is not None
            and right_open_pos is not None
            and right_upper is not None
            and right_open_pos[0] > right_closed_pos[0] + max(0.25, right_upper * 0.75),
            details=f"closed={right_closed_pos}, open={right_open_pos}, upper={right_upper}",
        )
        ctx.check(
            "left panel stays put while right opens",
            left_closed_pos is not None
            and left_while_right_open_pos is not None
            and abs(left_while_right_open_pos[0] - left_closed_pos[0]) <= 1e-6,
            details=f"closed={left_closed_pos}, pose={left_while_right_open_pos}",
        )

    with ctx.pose({left_slide: left_upper, right_slide: right_upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="both panels clear the frame at full travel")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
