from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cabinet_air_purifier")

    housing_white = model.material("housing_white", rgba=(0.92, 0.93, 0.94, 1.0))
    grille_dark = model.material("grille_dark", rgba=(0.25, 0.28, 0.30, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.13, 0.14, 0.15, 1.0))
    steel = model.material("steel", rgba=(0.74, 0.76, 0.78, 1.0))
    filter_media = model.material("filter_media", rgba=(0.87, 0.89, 0.84, 1.0))

    width = 0.62
    depth = 0.30
    body_height = 0.70
    leg_height = 0.05
    wall_t = 0.015
    top_t = 0.020
    bottom_t = 0.012

    door_width = 0.54
    door_depth = 0.19
    door_t = 0.010
    door_front_h = 0.094
    door_hinge_x = -0.045

    cabinet_body = model.part("cabinet_body")
    cabinet_body.visual(
        Box((depth, wall_t, body_height)),
        origin=Origin(xyz=(0.0, width / 2.0 - wall_t / 2.0, leg_height + body_height / 2.0)),
        material=housing_white,
        name="left_side_shell",
    )
    cabinet_body.visual(
        Box((depth, wall_t, body_height)),
        origin=Origin(xyz=(0.0, -(width / 2.0 - wall_t / 2.0), leg_height + body_height / 2.0)),
        material=housing_white,
        name="right_side_shell",
    )
    cabinet_body.visual(
        Box((wall_t, width - 2.0 * wall_t, body_height)),
        origin=Origin(xyz=(-(depth / 2.0) + wall_t / 2.0, 0.0, leg_height + body_height / 2.0)),
        material=housing_white,
        name="rear_shell",
    )
    cabinet_body.visual(
        Box((depth, width, top_t)),
        origin=Origin(xyz=(0.0, 0.0, leg_height + body_height - top_t / 2.0)),
        material=housing_white,
        name="top_cap",
    )
    cabinet_body.visual(
        Box((wall_t, width - 2.0 * wall_t, body_height - 0.095 - top_t)),
        origin=Origin(
            xyz=(
                depth / 2.0 - wall_t / 2.0,
                0.0,
                leg_height + 0.095 + (body_height - 0.095 - top_t) / 2.0,
            )
        ),
        material=grille_dark,
        name="front_grille_panel",
    )
    cabinet_body.visual(
        Box((0.010, 0.16, 0.032)),
        origin=Origin(xyz=(depth / 2.0 - 0.005, 0.0, leg_height + body_height - 0.055)),
        material=trim_dark,
        name="control_strip",
    )
    cabinet_body.visual(
        Box((0.22, 0.18, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, leg_height + body_height - 0.003)),
        material=grille_dark,
        name="top_exhaust_panel",
    )
    cabinet_body.visual(
        Box((depth / 2.0 + door_hinge_x, width, bottom_t)),
        origin=Origin(
            xyz=(
                -depth / 2.0 + (depth / 2.0 + door_hinge_x) / 2.0,
                0.0,
                leg_height + bottom_t / 2.0,
            )
        ),
        material=housing_white,
        name="rear_floor_strip",
    )
    sill_width = 0.032
    cabinet_body.visual(
        Box((door_depth, sill_width, bottom_t)),
        origin=Origin(
            xyz=(
                door_hinge_x + door_depth / 2.0,
                width / 2.0 - sill_width / 2.0,
                leg_height + bottom_t / 2.0,
            )
        ),
        material=housing_white,
        name="left_bottom_sill",
    )
    cabinet_body.visual(
        Box((door_depth, sill_width, bottom_t)),
        origin=Origin(
            xyz=(
                door_hinge_x + door_depth / 2.0,
                -(width / 2.0 - sill_width / 2.0),
                leg_height + bottom_t / 2.0,
            )
        ),
        material=housing_white,
        name="right_bottom_sill",
    )

    leg_size = (0.060, 0.045, leg_height)
    leg_x = depth / 2.0 - leg_size[0] / 2.0
    leg_y = width / 2.0 - leg_size[1] / 2.0
    for name, x_sign, y_sign in (
        ("front_left_leg", 1.0, 1.0),
        ("front_right_leg", 1.0, -1.0),
        ("rear_left_leg", -1.0, 1.0),
        ("rear_right_leg", -1.0, -1.0),
    ):
        cabinet_body.visual(
            Box(leg_size),
            origin=Origin(
                xyz=(
                    x_sign * leg_x,
                    y_sign * leg_y,
                    leg_height / 2.0,
                )
            ),
            material=trim_dark,
            name=name,
        )

    cabinet_body.inertial = Inertial.from_geometry(
        Box((depth, width, body_height + leg_height)),
        mass=22.0,
        origin=Origin(xyz=(0.0, 0.0, (body_height + leg_height) / 2.0)),
    )

    guide_rail_frame = model.part("guide_rail_frame")
    rail_len = 0.225
    rail_start_x = -0.121
    rail_center_x = rail_start_x + rail_len / 2.0
    rail_y = 0.275
    rail_z = 0.082
    guide_rail_frame.visual(
        Box((rail_len, 0.014, 0.010)),
        origin=Origin(xyz=(rail_center_x, rail_y, rail_z)),
        material=steel,
        name="left_rail",
    )
    guide_rail_frame.visual(
        Box((rail_len, 0.014, 0.010)),
        origin=Origin(xyz=(rail_center_x, -rail_y, rail_z)),
        material=steel,
        name="right_rail",
    )
    guide_rail_frame.visual(
        Box((0.014, 0.55, 0.020)),
        origin=Origin(xyz=(-0.128, 0.0, rail_z)),
        material=steel,
        name="rear_mount_crossbar",
    )
    guide_rail_frame.inertial = Inertial.from_geometry(
        Box((0.239, 0.55, 0.020)),
        mass=1.2,
        origin=Origin(xyz=(-0.008, 0.0, rail_z)),
    )

    filter_door = model.part("filter_door")
    filter_door.visual(
        Box((door_depth, door_width, door_t)),
        origin=Origin(xyz=(door_depth / 2.0, 0.0, door_t / 2.0)),
        material=housing_white,
        name="door_bottom_panel",
    )
    filter_door.visual(
        Box((0.012, door_width, door_front_h)),
        origin=Origin(xyz=(door_depth - 0.006, 0.0, door_front_h / 2.0)),
        material=housing_white,
        name="door_front_fascia",
    )
    filter_door.visual(
        Box((0.018, 0.14, 0.018)),
        origin=Origin(xyz=(door_depth - 0.001, 0.0, 0.034)),
        material=trim_dark,
        name="door_pull",
    )
    filter_door.inertial = Inertial.from_geometry(
        Box((door_depth, door_width, door_front_h)),
        mass=1.8,
        origin=Origin(xyz=(door_depth / 2.0, 0.0, door_front_h / 2.0)),
    )

    filter_tray = model.part("filter_tray")
    tray_len = 0.232
    tray_width = 0.52
    tray_height = 0.050
    tray_x = -0.106
    filter_tray.visual(
        Box((tray_len, tray_width, 0.004)),
        origin=Origin(xyz=(tray_x, 0.0, 0.002)),
        material=steel,
        name="tray_floor",
    )
    filter_tray.visual(
        Box((tray_len, 0.008, tray_height)),
        origin=Origin(xyz=(tray_x, tray_width / 2.0 - 0.004, tray_height / 2.0 + 0.002)),
        material=steel,
        name="left_tray_wall",
    )
    filter_tray.visual(
        Box((tray_len, 0.008, tray_height)),
        origin=Origin(xyz=(tray_x, -(tray_width / 2.0 - 0.004), tray_height / 2.0 + 0.002)),
        material=steel,
        name="right_tray_wall",
    )
    filter_tray.visual(
        Box((0.008, tray_width, tray_height)),
        origin=Origin(xyz=(-0.218, 0.0, tray_height / 2.0 + 0.002)),
        material=steel,
        name="tray_back_wall",
    )
    filter_tray.visual(
        Box((0.012, tray_width, 0.060)),
        origin=Origin(xyz=(0.004, 0.0, 0.030)),
        material=steel,
        name="tray_front_lip",
    )
    filter_tray.visual(
        Box((0.220, 0.010, 0.008)),
        origin=Origin(xyz=(tray_x, 0.263, 0.004)),
        material=steel,
        name="left_runner",
    )
    filter_tray.visual(
        Box((0.220, 0.010, 0.008)),
        origin=Origin(xyz=(tray_x, -0.263, 0.004)),
        material=steel,
        name="right_runner",
    )
    filter_tray.visual(
        Box((0.190, 0.492, 0.040)),
        origin=Origin(xyz=(-0.119, 0.0, 0.024)),
        material=filter_media,
        name="filter_media_block",
    )
    filter_tray.inertial = Inertial.from_geometry(
        Box((tray_len, tray_width, 0.060)),
        mass=2.6,
        origin=Origin(xyz=(tray_x, 0.0, 0.030)),
    )

    model.articulation(
        "body_to_guide_rail_frame",
        ArticulationType.FIXED,
        parent=cabinet_body,
        child=guide_rail_frame,
        origin=Origin(),
    )
    model.articulation(
        "body_to_filter_door",
        ArticulationType.REVOLUTE,
        parent=cabinet_body,
        child=filter_door,
        origin=Origin(xyz=(door_hinge_x, 0.0, leg_height)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )
    model.articulation(
        "rails_to_filter_tray",
        ArticulationType.PRISMATIC,
        parent=guide_rail_frame,
        child=filter_tray,
        origin=Origin(xyz=(0.104, 0.0, 0.087)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.20,
            lower=0.0,
            upper=0.120,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet_body = object_model.get_part("cabinet_body")
    filter_door = object_model.get_part("filter_door")
    guide_rail_frame = object_model.get_part("guide_rail_frame")
    filter_tray = object_model.get_part("filter_tray")
    door_hinge = object_model.get_articulation("body_to_filter_door")
    tray_slide = object_model.get_articulation("rails_to_filter_tray")

    door_open = 1.10
    tray_open = 0.120

    with ctx.pose({door_hinge: 0.0, tray_slide: 0.0}):
        ctx.expect_gap(
            cabinet_body,
            filter_door,
            axis="z",
            positive_elem="front_grille_panel",
            negative_elem="door_front_fascia",
            min_gap=0.0,
            max_gap=0.004,
            name="door fascia closes tightly beneath the front grille",
        )
        ctx.expect_contact(
            filter_tray,
            guide_rail_frame,
            elem_a="left_runner",
            elem_b="left_rail",
            name="tray runner contacts the left guide rail",
        )
        ctx.expect_overlap(
            filter_tray,
            guide_rail_frame,
            axes="x",
            elem_a="left_runner",
            elem_b="left_rail",
            min_overlap=0.20,
            name="closed tray remains deeply engaged with the guide rail",
        )
        closed_door_fascia = ctx.part_element_world_aabb(filter_door, elem="door_front_fascia")
        closed_tray_pos = ctx.part_world_position(filter_tray)

    with ctx.pose({door_hinge: door_open}):
        open_door_fascia = ctx.part_element_world_aabb(filter_door, elem="door_front_fascia")

    ctx.check(
        "door drops well below the cabinet when opened from the rear hinge",
        closed_door_fascia is not None
        and open_door_fascia is not None
        and open_door_fascia[0][2] < closed_door_fascia[0][2] - 0.08
        and open_door_fascia[1][2] < closed_door_fascia[0][2] - 0.01,
        details=f"closed={closed_door_fascia}, open={open_door_fascia}",
    )

    with ctx.pose({tray_slide: tray_open}):
        ctx.expect_contact(
            filter_tray,
            guide_rail_frame,
            elem_a="left_runner",
            elem_b="left_rail",
            name="extended tray still rides on the left guide rail",
        )
        ctx.expect_overlap(
            filter_tray,
            guide_rail_frame,
            axes="x",
            elem_a="left_runner",
            elem_b="left_rail",
            min_overlap=0.09,
            name="extended tray still retains insertion on the guide rail",
        )
        extended_tray_pos = ctx.part_world_position(filter_tray)

    ctx.check(
        "filter tray slides forward out of the cabinet",
        closed_tray_pos is not None
        and extended_tray_pos is not None
        and extended_tray_pos[0] > closed_tray_pos[0] + 0.10,
        details=f"closed={closed_tray_pos}, extended={extended_tray_pos}",
    )

    with ctx.pose({door_hinge: door_open, tray_slide: tray_open}):
        ctx.expect_gap(
            filter_tray,
            filter_door,
            axis="z",
            min_gap=0.02,
            name="extended tray clears the dropped access door",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
