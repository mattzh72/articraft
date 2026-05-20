from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hospital_overbed_table")

    powder_steel = model.material("powder_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.32, 0.34, 0.37, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.19, 0.21, 1.0))
    tray_surface = model.material("tray_surface", rgba=(0.91, 0.90, 0.86, 1.0))
    tray_trim = model.material("tray_trim", rgba=(0.78, 0.76, 0.71, 1.0))

    rail_length = 0.62
    rail_width = 0.055
    rail_height = 0.035
    rail_center_z = 0.095
    rail_center_x = rail_length * 0.5
    rail_offset_y = 0.180
    rear_crossbar_x = 0.030
    rear_crossbar_length = 0.060
    wheel_radius = 0.038
    wheel_tire_width = 0.022
    wheel_hub_width = 0.030
    wheel_axle_z = wheel_radius
    wheel_x_positions = (0.075, 0.545)
    fork_top_height = 0.012
    fork_plate_thickness = 0.005
    fork_plate_gap = wheel_hub_width
    fork_plate_offset = fork_plate_gap * 0.5 + fork_plate_thickness * 0.5
    brake_bar_axis_x = 0.125
    brake_bar_axis_z = 0.064

    outer_post_height = 0.550
    outer_post_outer_x = 0.064
    outer_post_outer_y = 0.044
    outer_post_wall = 0.005
    outer_post_base_x = 0.185
    outer_post_base_y = rail_offset_y
    rail_top_z = rail_center_z + rail_height * 0.5
    outer_post_shoe_height = 0.024
    outer_post_top_z = outer_post_shoe_height + outer_post_height

    base = model.part("base")
    base.visual(
        Box((rail_length, rail_width, rail_height)),
        origin=Origin(xyz=(rail_center_x, -rail_offset_y, rail_center_z)),
        material=powder_steel,
        name="rail_0",
    )
    base.visual(
        Box((rail_length, rail_width, rail_height)),
        origin=Origin(xyz=(rail_center_x, rail_offset_y, rail_center_z)),
        material=powder_steel,
        name="rail_1",
    )
    base.visual(
        Box((rear_crossbar_length, rail_offset_y * 2.0 + rail_width, rail_height)),
        origin=Origin(xyz=(rear_crossbar_x, 0.0, rail_center_z)),
        material=powder_steel,
        name="rear_crossbar",
    )
    for wheel_row, wheel_x in enumerate(wheel_x_positions):
        for wheel_col, wheel_y in enumerate((-rail_offset_y, rail_offset_y)):
            fork_index = wheel_row * 2 + wheel_col
            base.visual(
                Box((0.050, 0.045, fork_top_height)),
                origin=Origin(
                    xyz=(
                        wheel_x,
                        wheel_y,
                        rail_center_z - rail_height * 0.5 + fork_top_height * 0.5,
                    )
                ),
                material=powder_steel,
                name=f"fork_top_{fork_index}",
            )
            base.visual(
                Box((0.050, fork_plate_thickness, 0.040)),
                origin=Origin(
                    xyz=(
                        wheel_x,
                        wheel_y + fork_plate_offset,
                        wheel_axle_z + 0.020,
                    )
                ),
                material=powder_steel,
                name=f"fork_plate_{fork_index}_0",
            )
            base.visual(
                Box((0.050, fork_plate_thickness, 0.040)),
                origin=Origin(
                    xyz=(
                        wheel_x,
                        wheel_y - fork_plate_offset,
                        wheel_axle_z + 0.020,
                    )
                ),
                material=powder_steel,
                name=f"fork_plate_{fork_index}_1",
            )
    base.visual(
        Box((0.018, 0.018, 0.032)),
        origin=Origin(xyz=(brake_bar_axis_x, rail_offset_y - rail_width * 0.5 - 0.009, brake_bar_axis_z)),
        material=powder_steel,
        name="brake_bracket_0",
    )
    base.visual(
        Box((0.018, 0.018, 0.032)),
        origin=Origin(xyz=(brake_bar_axis_x, -rail_offset_y + rail_width * 0.5 + 0.009, brake_bar_axis_z)),
        material=powder_steel,
        name="brake_bracket_1",
    )
    outer_post = model.part("outer_post")
    outer_post.visual(
        Box((0.115, 0.085, outer_post_shoe_height)),
        origin=Origin(xyz=(0.0, 0.0, outer_post_shoe_height * 0.5)),
        material=powder_steel,
        name="mount_shoe",
    )
    sleeve_center_z = outer_post_shoe_height + outer_post_height * 0.5
    outer_post.visual(
        Box((outer_post_wall, outer_post_outer_y, outer_post_height)),
        origin=Origin(xyz=(outer_post_outer_x * 0.5 - outer_post_wall * 0.5, 0.0, sleeve_center_z)),
        material=powder_steel,
        name="front_wall",
    )
    outer_post.visual(
        Box((outer_post_wall, outer_post_outer_y, outer_post_height)),
        origin=Origin(xyz=(-outer_post_outer_x * 0.5 + outer_post_wall * 0.5, 0.0, sleeve_center_z)),
        material=powder_steel,
        name="rear_wall",
    )
    outer_post.visual(
        Box((outer_post_outer_x - 2.0 * outer_post_wall, outer_post_wall, outer_post_height)),
        origin=Origin(xyz=(0.0, outer_post_outer_y * 0.5 - outer_post_wall * 0.5, sleeve_center_z)),
        material=powder_steel,
        name="side_wall_0",
    )
    outer_post.visual(
        Box((outer_post_outer_x - 2.0 * outer_post_wall, outer_post_wall, outer_post_height)),
        origin=Origin(xyz=(0.0, -outer_post_outer_y * 0.5 + outer_post_wall * 0.5, sleeve_center_z)),
        material=powder_steel,
        name="side_wall_1",
    )
    inner_column = model.part("inner_column")
    inner_column.visual(
        Box((0.052, 0.032, 0.600)),
        origin=Origin(xyz=(0.0, 0.0, -0.080)),
        material=dark_steel,
        name="inner_tube",
    )
    inner_column.visual(
        Box((0.092, 0.200, 0.034)),
        origin=Origin(xyz=(0.0, -0.100, 0.220)),
        material=dark_steel,
        name="support_arm",
    )
    inner_column.visual(
        Box((0.110, 0.080, 0.040)),
        origin=Origin(xyz=(0.0, -0.200, 0.237)),
        material=powder_steel,
        name="support_head",
    )
    inner_column.visual(
        Box((0.001, 0.012, 0.045)),
        origin=Origin(xyz=(0.0265, 0.0, -0.018)),
        material=charcoal,
        name="guide_pad_0",
    )
    inner_column.visual(
        Box((0.001, 0.012, 0.045)),
        origin=Origin(xyz=(-0.0265, 0.0, -0.018)),
        material=charcoal,
        name="guide_pad_1",
    )
    inner_column.visual(
        Box((0.010, 0.001, 0.045)),
        origin=Origin(xyz=(0.0, 0.0165, -0.018)),
        material=charcoal,
        name="guide_pad_2",
    )
    inner_column.visual(
        Box((0.010, 0.001, 0.045)),
        origin=Origin(xyz=(0.0, -0.0165, -0.018)),
        material=charcoal,
        name="guide_pad_3",
    )

    tray = model.part("tray")
    tray_width_y = 0.780
    tray_depth_x = 0.420
    tray_panel_thickness = 0.018
    tray_lip_height = 0.020
    tray_lip_thickness = 0.012
    tray.visual(
        Box((tray_depth_x, tray_width_y, tray_panel_thickness)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=tray_surface,
        name="panel",
    )
    tray.visual(
        Box((tray_lip_thickness, tray_width_y, tray_lip_height)),
        origin=Origin(xyz=(tray_depth_x * 0.5 - tray_lip_thickness * 0.5, 0.0, 0.031)),
        material=tray_trim,
        name="front_lip",
    )
    tray.visual(
        Box((tray_lip_thickness, tray_width_y, tray_lip_height)),
        origin=Origin(xyz=(-tray_depth_x * 0.5 + tray_lip_thickness * 0.5, 0.0, 0.031)),
        material=tray_trim,
        name="rear_lip",
    )
    tray.visual(
        Box((tray_depth_x - 2.0 * tray_lip_thickness, tray_lip_thickness, tray_lip_height)),
        origin=Origin(xyz=(0.0, tray_width_y * 0.5 - tray_lip_thickness * 0.5, 0.031)),
        material=tray_trim,
        name="side_lip_0",
    )
    tray.visual(
        Box((tray_depth_x - 2.0 * tray_lip_thickness, tray_lip_thickness, tray_lip_height)),
        origin=Origin(xyz=(0.0, -tray_width_y * 0.5 + tray_lip_thickness * 0.5, 0.031)),
        material=tray_trim,
        name="side_lip_1",
    )
    tray.visual(
        Box((0.115, 0.085, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=powder_steel,
        name="receiver_block",
    )
    tray.visual(
        Box((0.014, 0.009, 0.018)),
        origin=Origin(xyz=(0.168, 0.1135, -0.006)),
        material=dark_steel,
        name="lever_bracket_0",
    )
    tray.visual(
        Box((0.014, 0.009, 0.018)),
        origin=Origin(xyz=(0.168, -0.1135, -0.006)),
        material=dark_steel,
        name="lever_bracket_1",
    )

    lever = model.part("lever")
    lever.visual(
        Box((0.010, 0.016, 0.016)),
        origin=Origin(xyz=(0.0, 0.104, -0.008)),
        material=charcoal,
        name="ear_0",
    )
    lever.visual(
        Box((0.010, 0.016, 0.016)),
        origin=Origin(xyz=(0.0, -0.104, -0.008)),
        material=charcoal,
        name="ear_1",
    )
    lever.visual(
        Box((0.026, 0.220, 0.014)),
        origin=Origin(xyz=(-0.012, 0.0, -0.018)),
        material=charcoal,
        name="handle",
    )

    brake_bar = model.part("brake_bar")
    brake_bar.visual(
        Box((0.040, 0.269, 0.018)),
        origin=Origin(xyz=(-0.012, 0.0, -0.010)),
        material=charcoal,
        name="pedal",
    )
    brake_bar.visual(
        Box((0.012, 0.269, 0.008)),
        origin=Origin(xyz=(-0.028, 0.0, 0.003)),
        material=dark_steel,
        name="ridge",
    )

    model.articulation(
        "base_to_outer_post",
        ArticulationType.FIXED,
        parent=base,
        child=outer_post,
        origin=Origin(xyz=(outer_post_base_x, outer_post_base_y, rail_top_z)),
    )
    model.articulation(
        "outer_post_to_inner_column",
        ArticulationType.PRISMATIC,
        parent=outer_post,
        child=inner_column,
        origin=Origin(xyz=(0.0, 0.0, outer_post_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.160,
            lower=0.0,
            upper=0.200,
        ),
    )
    model.articulation(
        "inner_column_to_tray",
        ArticulationType.REVOLUTE,
        parent=inner_column,
        child=tray,
        origin=Origin(xyz=(0.0, -0.200, 0.272)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.200,
            lower=-0.12,
            upper=0.55,
        ),
    )
    model.articulation(
        "tray_to_lever",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=lever,
        origin=Origin(xyz=(0.168, 0.0, 0.003)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.000,
            lower=0.0,
            upper=0.60,
        ),
    )
    model.articulation(
        "base_to_brake_bar",
        ArticulationType.REVOLUTE,
        parent=base,
        child=brake_bar,
        origin=Origin(xyz=(brake_bar_axis_x, 0.0, brake_bar_axis_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.500,
            lower=0.0,
            upper=0.40,
        ),
    )
    for wheel_row, wheel_x in enumerate(wheel_x_positions):
        for wheel_col, wheel_y in enumerate((-rail_offset_y, rail_offset_y)):
            caster_index = wheel_row * 2 + wheel_col
            caster = model.part(f"caster_{caster_index}")
            caster.visual(
                Cylinder(radius=0.023, length=wheel_hub_width),
                material=dark_steel,
                name="hub_core",
            )
            caster.visual(
                Cylinder(radius=wheel_radius, length=wheel_tire_width),
                material=charcoal,
                name="tire",
            )
            model.articulation(
                f"base_to_caster_{caster_index}",
                ArticulationType.CONTINUOUS,
                parent=base,
                child=caster,
                origin=Origin(xyz=(wheel_x, wheel_y, wheel_axle_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                axis=(0.0, 0.0, 1.0),
                motion_limits=MotionLimits(
                    effort=8.0,
                    velocity=20.0,
                ),
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    outer_post = object_model.get_part("outer_post")
    inner_column = object_model.get_part("inner_column")
    tray = object_model.get_part("tray")
    lever = object_model.get_part("lever")
    brake_bar = object_model.get_part("brake_bar")
    lift = object_model.get_articulation("outer_post_to_inner_column")
    tray_tilt = object_model.get_articulation("inner_column_to_tray")
    lever_joint = object_model.get_articulation("tray_to_lever")
    brake_joint = object_model.get_articulation("base_to_brake_bar")

    lift_limits = lift.motion_limits
    tilt_limits = tray_tilt.motion_limits

    ctx.expect_within(
        inner_column,
        outer_post,
        axes="xy",
        elem_a="inner_tube",
        margin=0.009,
        name="inner column stays centered in outer sleeve",
    )
    ctx.expect_overlap(
        inner_column,
        outer_post,
        axes="z",
        elem_a="inner_tube",
        min_overlap=0.16,
        name="collapsed inner column remains inserted in outer sleeve",
    )

    rest_pos = ctx.part_world_position(inner_column)
    if lift_limits is not None and lift_limits.upper is not None:
        with ctx.pose({lift: lift_limits.upper}):
            ctx.expect_within(
                inner_column,
                outer_post,
                axes="xy",
                elem_a="inner_tube",
                margin=0.009,
                name="extended inner column stays centered in outer sleeve",
            )
            ctx.expect_overlap(
                inner_column,
                outer_post,
                axes="z",
                elem_a="inner_tube",
                min_overlap=0.16,
                name="extended inner column still retains insertion",
            )
            extended_pos = ctx.part_world_position(inner_column)
        ctx.check(
            "inner column extends upward",
            rest_pos is not None and extended_pos is not None and extended_pos[2] > rest_pos[2] + 0.15,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    if tilt_limits is not None and tilt_limits.upper is not None:
        front_rest = ctx.part_element_world_aabb(tray, elem="front_lip")
        with ctx.pose({tray_tilt: tilt_limits.upper}):
            front_tilt = ctx.part_element_world_aabb(tray, elem="front_lip")
        ctx.check(
            "tray front lip rises when tilted",
            front_rest is not None
            and front_tilt is not None
            and front_tilt[1][2] > front_rest[1][2] + 0.08,
            details=f"rest={front_rest}, tilted={front_tilt}",
        )

    lever_limits = lever_joint.motion_limits
    if lever_limits is not None and lever_limits.upper is not None:
        lever_rest = ctx.part_element_world_aabb(lever, elem="handle")
        with ctx.pose({lever_joint: lever_limits.upper}):
            lever_squeezed = ctx.part_element_world_aabb(lever, elem="handle")
        ctx.check(
            "squeeze lever lifts toward tray",
            lever_rest is not None
            and lever_squeezed is not None
            and lever_squeezed[1][2] > lever_rest[1][2] + 0.01,
            details=f"rest={lever_rest}, squeezed={lever_squeezed}",
        )

    brake_limits = brake_joint.motion_limits
    if brake_limits is not None and brake_limits.upper is not None:
        brake_rest = ctx.part_element_world_aabb(brake_bar, elem="pedal")
        with ctx.pose({brake_joint: brake_limits.upper}):
            brake_pressed = ctx.part_element_world_aabb(brake_bar, elem="pedal")
        ctx.check(
            "brake bar rotates upward from the base",
            brake_rest is not None
            and brake_pressed is not None
            and brake_pressed[1][2] > brake_rest[1][2] + 0.01,
            details=f"rest={brake_rest}, pressed={brake_pressed}",
        )

    return ctx.report()


object_model = build_object_model()
