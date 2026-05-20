from __future__ import annotations

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
    model = ArticulatedObject(name="video_poker_terminal")

    model.material("cabinet_body", rgba=(0.14, 0.15, 0.17, 1.0))
    model.material("cabinet_trim", rgba=(0.06, 0.06, 0.07, 1.0))
    model.material("screen_glass", rgba=(0.03, 0.05, 0.07, 1.0))
    model.material("button_cap", rgba=(0.73, 0.12, 0.11, 1.0))
    model.material("button_stem", rgba=(0.16, 0.16, 0.17, 1.0))
    model.material("door_pull", rgba=(0.78, 0.79, 0.82, 1.0))

    width = 0.43
    depth = 0.26
    height = 0.70
    wall = 0.014
    front_skin = 0.018
    front_y = depth / 2.0 - front_skin / 2.0

    cabinet = model.part("cabinet")

    # Main shallow shell.
    cabinet.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(-(width - wall) / 2.0, 0.0, height / 2.0)),
        material="cabinet_body",
        name="side_wall_0",
    )
    cabinet.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=((width - wall) / 2.0, 0.0, height / 2.0)),
        material="cabinet_body",
        name="side_wall_1",
    )
    cabinet.visual(
        Box((width - 2.0 * wall, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, wall / 2.0)),
        material="cabinet_body",
        name="base_panel",
    )
    cabinet.visual(
        Box((width - 2.0 * wall, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, height - wall / 2.0)),
        material="cabinet_body",
        name="top_panel",
    )
    cabinet.visual(
        Box((width - 2.0 * wall, wall, height - 2.0 * wall)),
        origin=Origin(xyz=(0.0, -(depth - wall) / 2.0, height / 2.0)),
        material="cabinet_body",
        name="back_panel",
    )

    # Lower front frame around the cash-box door.
    lower_front_width = 0.402
    door_opening_width = 0.230
    door_opening_height = 0.158
    door_bottom = 0.072
    door_top = door_bottom + door_opening_height
    lower_side_width = (lower_front_width - door_opening_width) / 2.0
    lower_bottom_height = door_bottom
    lower_top_height = 0.035

    cabinet.visual(
        Box((lower_front_width, front_skin, lower_bottom_height)),
        origin=Origin(xyz=(0.0, front_y, lower_bottom_height / 2.0)),
        material="cabinet_body",
        name="door_frame_bottom",
    )
    cabinet.visual(
        Box((lower_side_width, front_skin, door_opening_height)),
        origin=Origin(
            xyz=(
                -(door_opening_width / 2.0 + lower_side_width / 2.0),
                front_y,
                door_bottom + door_opening_height / 2.0,
            )
        ),
        material="cabinet_body",
        name="door_frame_side_0",
    )
    cabinet.visual(
        Box((lower_side_width, front_skin, door_opening_height)),
        origin=Origin(
            xyz=(
                door_opening_width / 2.0 + lower_side_width / 2.0,
                front_y,
                door_bottom + door_opening_height / 2.0,
            )
        ),
        material="cabinet_body",
        name="door_frame_side_1",
    )
    cabinet.visual(
        Box((lower_front_width, front_skin, lower_top_height)),
        origin=Origin(xyz=(0.0, front_y, door_top + lower_top_height / 2.0)),
        material="cabinet_body",
        name="door_frame_top",
    )

    # Distinct control shelf between screen surround and cash door.
    shelf_width = 0.390
    shelf_depth = 0.084
    shelf_slot_width = 0.326
    shelf_slot_depth = 0.050
    shelf_top = 0.293
    shelf_thickness = 0.018
    shelf_center_y = 0.120
    shelf_center_z = shelf_top - shelf_thickness / 2.0
    rail_side = (shelf_width - shelf_slot_width) / 2.0
    rail_front = (shelf_depth - shelf_slot_depth) / 2.0

    cabinet.visual(
        Box((rail_side, shelf_depth, shelf_thickness)),
        origin=Origin(
            xyz=(-(shelf_slot_width / 2.0 + rail_side / 2.0), shelf_center_y, shelf_center_z)
        ),
        material="cabinet_body",
        name="shelf_side_0",
    )
    cabinet.visual(
        Box((rail_side, shelf_depth, shelf_thickness)),
        origin=Origin(
            xyz=(shelf_slot_width / 2.0 + rail_side / 2.0, shelf_center_y, shelf_center_z)
        ),
        material="cabinet_body",
        name="shelf_side_1",
    )
    cabinet.visual(
        Box((shelf_slot_width, rail_front, shelf_thickness)),
        origin=Origin(
            xyz=(0.0, shelf_center_y - (shelf_slot_depth / 2.0 + rail_front / 2.0), shelf_center_z)
        ),
        material="cabinet_body",
        name="shelf_back",
    )
    cabinet.visual(
        Box((shelf_slot_width, rail_front, shelf_thickness)),
        origin=Origin(
            xyz=(0.0, shelf_center_y + (shelf_slot_depth / 2.0 + rail_front / 2.0), shelf_center_z)
        ),
        material="cabinet_body",
        name="shelf_front",
    )
    cabinet.visual(
        Box((shelf_slot_width + 0.006, shelf_slot_depth + 0.006, 0.014)),
        origin=Origin(xyz=(0.0, shelf_center_y, 0.268)),
        material="cabinet_trim",
        name="button_tray",
    )
    cabinet.visual(
        Box((lower_front_width, 0.050, 0.040)),
        origin=Origin(xyz=(0.0, 0.105, 0.255)),
        material="cabinet_body",
        name="shelf_riser",
    )
    cabinet.visual(
        Box((shelf_width, 0.018, 0.040)),
        origin=Origin(xyz=(0.0, 0.156, 0.255)),
        material="cabinet_body",
        name="shelf_fascia",
    )

    # Front screen surround and recessed display.
    screen_width = 0.302
    screen_height = 0.222
    screen_bezel_width = 0.402
    screen_bezel_height = 0.292
    screen_bottom = 0.385
    screen_bezel_side = (screen_bezel_width - screen_width) / 2.0
    screen_bezel_top = (screen_bezel_height - screen_height) / 2.0
    screen_center_z = screen_bottom + screen_height / 2.0
    screen_bezel_center_z = screen_bottom + screen_bezel_height / 2.0

    cabinet.visual(
        Box((screen_bezel_side, front_skin, screen_bezel_height)),
        origin=Origin(
            xyz=(
                -(screen_width / 2.0 + screen_bezel_side / 2.0),
                front_y,
                screen_bezel_center_z,
            )
        ),
        material="cabinet_body",
        name="screen_side_0",
    )
    cabinet.visual(
        Box((screen_bezel_side, front_skin, screen_bezel_height)),
        origin=Origin(
            xyz=(
                screen_width / 2.0 + screen_bezel_side / 2.0,
                front_y,
                screen_bezel_center_z,
            )
        ),
        material="cabinet_body",
        name="screen_side_1",
    )
    cabinet.visual(
        Box((screen_width, front_skin, screen_bezel_top)),
        origin=Origin(
            xyz=(0.0, front_y, screen_bottom + screen_height + screen_bezel_top / 2.0)
        ),
        material="cabinet_body",
        name="screen_top",
    )
    cabinet.visual(
        Box((screen_width, front_skin, screen_bezel_top)),
        origin=Origin(xyz=(0.0, front_y, screen_bottom - screen_bezel_top / 2.0)),
        material="cabinet_body",
        name="screen_bottom",
    )
    cabinet.visual(
        Box((screen_width + 0.036, 0.030, screen_height + 0.060)),
        origin=Origin(xyz=(0.0, 0.100, screen_center_z)),
        material="cabinet_trim",
        name="screen_recess",
    )
    cabinet.visual(
        Box((screen_width, 0.010, screen_height)),
        origin=Origin(xyz=(0.0, 0.114, screen_center_z)),
        material="screen_glass",
        name="screen_glass",
    )

    # Cash-box door on a vertical hinge.
    door = model.part("cash_door")
    door_width = door_opening_width - 0.004
    door_height = door_opening_height - 0.004
    door_thickness = 0.014

    door.visual(
        Box((door_width, door_thickness, door_height)),
        origin=Origin(xyz=(door_width / 2.0, 0.0, door_height / 2.0)),
        material="cabinet_trim",
        name="door_panel",
    )
    door.visual(
        Box((0.022, 0.018, 0.055)),
        origin=Origin(xyz=(door_width - 0.030, 0.016, door_height / 2.0)),
        material="door_pull",
        name="door_pull",
    )

    door_hinge = model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(
            xyz=(
                -door_opening_width / 2.0,
                0.123,
                door_bottom + 0.002,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=1.7,
        ),
    )

    # Five independently pressing hold buttons across the shelf.
    button_count = 5
    button_spacing = 0.062
    button_cap_size = (0.050, 0.030, 0.010)
    button_stem_size = (0.030, 0.050, 0.012)
    button_center_y = shelf_center_y
    button_center_z = shelf_top + button_cap_size[2] / 2.0 + 0.001
    button_lower = 0.0
    button_upper = 0.0055

    for index in range(button_count):
        x_pos = (index - (button_count - 1) / 2.0) * button_spacing
        button = model.part(f"hold_button_{index}")
        button.visual(
            Box(button_cap_size),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material="button_cap",
            name="button_cap",
        )
        button.visual(
            Box(button_stem_size),
            origin=Origin(
                xyz=(
                    0.0,
                    0.0,
                    -(button_cap_size[2] + button_stem_size[2]) / 2.0,
                )
            ),
            material="button_stem",
            name="button_stem",
        )
        model.articulation(
            f"button_slide_{index}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=button,
            origin=Origin(xyz=(x_pos, button_center_y, button_center_z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=0.08,
                lower=button_lower,
                upper=button_upper,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("cash_door")
    door_hinge = object_model.get_articulation("door_hinge")

    button_parts = [object_model.get_part(f"hold_button_{index}") for index in range(5)]
    button_joints = [object_model.get_articulation(f"button_slide_{index}") for index in range(5)]

    shelf_aabb = ctx.part_element_world_aabb(cabinet, elem="shelf_front")
    screen_aabb = ctx.part_element_world_aabb(cabinet, elem="screen_glass")
    door_aabb = ctx.part_element_world_aabb(door, elem="door_panel")

    ctx.check(
        "control shelf sits above cash door",
        shelf_aabb is not None
        and door_aabb is not None
        and shelf_aabb[0][2] > door_aabb[1][2] + 0.010,
        details=f"shelf={shelf_aabb}, door={door_aabb}",
    )
    ctx.check(
        "screen sits above control shelf",
        shelf_aabb is not None
        and screen_aabb is not None
        and screen_aabb[0][2] > shelf_aabb[1][2] + 0.070,
        details=f"shelf={shelf_aabb}, screen={screen_aabb}",
    )

    closed_door = ctx.part_element_world_aabb(door, elem="door_panel")
    cabinet_frame = ctx.part_element_world_aabb(cabinet, elem="door_frame_top")
    ctx.check(
        "cash door is flush with front frame when closed",
        closed_door is not None
        and cabinet_frame is not None
        and abs(closed_door[1][1] - cabinet_frame[1][1]) <= 0.0015,
        details=f"door={closed_door}, frame={cabinet_frame}",
    )

    door_limits = door_hinge.motion_limits
    if door_limits is not None and door_limits.upper is not None:
        with ctx.pose({door_hinge: door_limits.upper}):
            opened_door = ctx.part_element_world_aabb(door, elem="door_panel")
            ctx.check(
                "cash door swings outward",
                closed_door is not None
                and opened_door is not None
                and opened_door[1][1] > closed_door[1][1] + 0.080,
                details=f"closed={closed_door}, opened={opened_door}",
            )

    button_positions = [ctx.part_world_position(button) for button in button_parts]
    valid_positions = [pos for pos in button_positions if pos is not None]
    if len(valid_positions) == len(button_positions):
        spacings = [
            valid_positions[index + 1][0] - valid_positions[index][0]
            for index in range(len(valid_positions) - 1)
        ]
        mean_spacing = sum(spacings) / len(spacings)
        even_spacing = all(abs(spacing - mean_spacing) <= 0.003 for spacing in spacings)
        same_row = (
            max(pos[1] for pos in valid_positions) - min(pos[1] for pos in valid_positions) <= 0.001
        )
        ctx.check(
            "hold buttons form an even row",
            even_spacing and same_row,
            details=f"positions={button_positions}, spacings={spacings}",
        )

    for button, joint in zip(button_parts, button_joints):
        rest_pos = ctx.part_world_position(button)
        limits = joint.motion_limits
        if limits is None or limits.upper is None:
            ctx.fail(f"{button.name} has press travel", "missing prismatic upper limit")
            continue
        with ctx.pose({joint: limits.upper}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"{button.name} presses downward",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[2] < rest_pos[2] - 0.004,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
