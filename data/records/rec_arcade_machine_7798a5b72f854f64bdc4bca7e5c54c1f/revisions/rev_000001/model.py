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


DEG = math.pi / 180.0


def _add_start_button(
    model: ArticulatedObject,
    cabinet,
    *,
    name: str,
    y: float,
    material: str,
    deck_angle: float,
) -> None:
    button = model.part(name)
    button.visual(
        Cylinder(radius=0.036, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.0057)),
        material="button_rim",
        name="skirt",
    )
    button.visual(
        Cylinder(radius=0.052, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0186)),
        material=material,
        name="cap",
    )

    model.articulation(
        f"cabinet_to_{name}",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=button,
        origin=Origin(xyz=(0.045, y, 0.976), rpy=(0.0, -deck_angle, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.20,
            lower=0.0,
            upper=0.008,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="upright_arcade_cabinet")

    model.material("cabinet_black", rgba=(0.08, 0.08, 0.09, 1.0))
    model.material("cabinet_blue", rgba=(0.10, 0.14, 0.30, 1.0))
    model.material("trim_dark", rgba=(0.18, 0.18, 0.20, 1.0))
    model.material("trim_light", rgba=(0.32, 0.34, 0.38, 1.0))
    model.material("monitor_glass", rgba=(0.05, 0.08, 0.10, 1.0))
    model.material("marquee_glow", rgba=(0.78, 0.82, 0.97, 1.0))
    model.material("door_dark", rgba=(0.14, 0.14, 0.16, 1.0))
    model.material("button_rim", rgba=(0.16, 0.16, 0.18, 1.0))
    model.material("button_green", rgba=(0.20, 0.88, 0.38, 1.0))
    model.material("button_blue", rgba=(0.18, 0.58, 0.95, 1.0))
    model.material("button_red", rgba=(0.94, 0.25, 0.22, 1.0))

    cabinet = model.part("cabinet")

    width = 0.82
    depth = 0.92
    height = 1.78
    wall = 0.03
    lower_height = 1.02
    deck_angle = 17.0 * DEG
    monitor_angle = 20.0 * DEG

    side_y = width / 2.0 - wall / 2.0

    cabinet.visual(
        Box((depth, width, wall)),
        origin=Origin(xyz=(depth / 2.0, 0.0, wall / 2.0)),
        material="cabinet_black",
        name="floor_panel",
    )
    cabinet.visual(
        Box((wall, width, height - 0.03)),
        origin=Origin(xyz=(depth - wall / 2.0, 0.0, (height - 0.03) / 2.0)),
        material="cabinet_black",
        name="back_panel",
    )

    for index, y in enumerate((-side_y, side_y)):
        cabinet.visual(
            Box((depth, wall, lower_height)),
            origin=Origin(xyz=(depth / 2.0, y, lower_height / 2.0)),
            material="cabinet_blue",
            name=f"side_panel_{index}",
        )
        cabinet.visual(
            Box((0.50, wall, 0.64)),
            origin=Origin(xyz=(0.64, y, 1.34), rpy=(0.0, -monitor_angle, 0.0)),
            material="cabinet_blue",
            name=f"upper_side_{index}",
        )

    door_width = 0.46
    door_height = 0.42
    opening_side = (width - door_width) / 2.0

    cabinet.visual(
        Box((wall, width, 0.10)),
        origin=Origin(xyz=(wall / 2.0, 0.0, 0.05)),
        material="cabinet_black",
        name="kick_panel",
    )
    cabinet.visual(
        Box((wall, opening_side, door_height)),
        origin=Origin(xyz=(wall / 2.0, -(door_width + opening_side) / 2.0, 0.31)),
        material="cabinet_black",
        name="door_left_stile",
    )
    cabinet.visual(
        Box((wall, opening_side, door_height)),
        origin=Origin(xyz=(wall / 2.0, (door_width + opening_side) / 2.0, 0.31)),
        material="cabinet_black",
        name="door_right_stile",
    )
    cabinet.visual(
        Box((wall, width, 0.18)),
        origin=Origin(xyz=(wall / 2.0, 0.0, 0.61)),
        material="cabinet_black",
        name="door_top_rail",
    )
    cabinet.visual(
        Box((0.06, 0.42, 0.15)),
        origin=Origin(xyz=(0.008, 0.0, 0.80)),
        material="trim_dark",
        name="coin_panel",
    )
    cabinet.visual(
        Box((0.014, 0.09, 0.022)),
        origin=Origin(xyz=(-0.014, -0.09, 0.83)),
        material="trim_light",
        name="coin_slot_0",
    )
    cabinet.visual(
        Box((0.014, 0.09, 0.022)),
        origin=Origin(xyz=(-0.014, 0.09, 0.83)),
        material="trim_light",
        name="coin_slot_1",
    )

    cabinet.visual(
        Box((0.24, 0.70, 0.14)),
        origin=Origin(xyz=(0.03, 0.0, 0.885), rpy=(0.0, -deck_angle, 0.0)),
        material="trim_dark",
        name="control_box",
    )
    cabinet.visual(
        Box((0.30, 0.76, 0.045)),
        origin=Origin(xyz=(0.07, 0.0, 0.962), rpy=(0.0, -deck_angle, 0.0)),
        material="trim_light",
        name="control_deck",
    )
    cabinet.visual(
        Box((0.05, 0.74, 0.05)),
        origin=Origin(xyz=(-0.11, 0.0, 0.89)),
        material="trim_dark",
        name="control_apron",
    )

    cabinet.visual(
        Box((0.20, 0.78, 0.54)),
        origin=Origin(xyz=(0.40, 0.0, 1.28), rpy=(0.0, -monitor_angle, 0.0)),
        material="cabinet_black",
        name="monitor_bay",
    )
    cabinet.visual(
        Box((0.024, 0.52, 0.32)),
        origin=Origin(xyz=(0.308, 0.0, 1.245), rpy=(0.0, -monitor_angle, 0.0)),
        material="monitor_glass",
        name="monitor_window",
    )

    cabinet.visual(
        Box((0.12, 0.76, 0.22)),
        origin=Origin(xyz=(0.51, 0.0, 1.57), rpy=(0.0, -8.0 * DEG, 0.0)),
        material="cabinet_black",
        name="marquee_box",
    )
    cabinet.visual(
        Box((0.016, 0.56, 0.11)),
        origin=Origin(xyz=(0.447, 0.0, 1.575), rpy=(0.0, -8.0 * DEG, 0.0)),
        material="marquee_glow",
        name="marquee_glow",
    )
    cabinet.visual(
        Box((0.40, width, wall)),
        origin=Origin(xyz=(0.72, 0.0, height - wall / 2.0)),
        material="cabinet_black",
        name="roof_panel",
    )

    service_door = model.part("service_door")
    service_door.visual(
        Box((0.022, door_width, door_height)),
        origin=Origin(xyz=(0.011, door_width / 2.0, 0.0)),
        material="door_dark",
        name="door_panel",
    )
    service_door.visual(
        Box((0.010, door_width - 0.08, door_height - 0.08)),
        origin=Origin(xyz=(0.015, door_width / 2.0, 0.0)),
        material="cabinet_black",
        name="door_inset",
    )
    service_door.visual(
        Box((0.030, 0.020, 0.060)),
        origin=Origin(xyz=(-0.007, door_width - 0.055, 0.0)),
        material="trim_light",
        name="latch_handle",
    )

    model.articulation(
        "cabinet_to_service_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=service_door,
        origin=Origin(xyz=(0.0, -door_width / 2.0, 0.31)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.5,
            lower=0.0,
            upper=1.25,
        ),
    )

    _add_start_button(
        model,
        cabinet,
        name="button_0",
        y=-0.14,
        material="button_green",
        deck_angle=deck_angle,
    )
    _add_start_button(
        model,
        cabinet,
        name="button_1",
        y=0.0,
        material="button_blue",
        deck_angle=deck_angle,
    )
    _add_start_button(
        model,
        cabinet,
        name="button_2",
        y=0.14,
        material="button_red",
        deck_angle=deck_angle,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    service_door = object_model.get_part("service_door")
    door_hinge = object_model.get_articulation("cabinet_to_service_door")

    for index in range(3):
        button = object_model.get_part(f"button_{index}")
        ctx.allow_overlap(
            button,
            cabinet,
            elem_a="skirt",
            elem_b="control_deck",
            reason="The start button collar is intentionally simplified as seating into the solid control-deck proxy rather than through a modeled cutout.",
        )
        ctx.expect_overlap(
            button,
            cabinet,
            axes="xy",
            elem_a="cap",
            elem_b="control_deck",
            min_overlap=0.07,
            name=f"button_{index} sits on the control deck footprint",
        )

    door_limits = door_hinge.motion_limits
    if door_limits is not None and door_limits.upper is not None:
        closed_aabb = ctx.part_element_world_aabb(service_door, elem="door_panel")
        with ctx.pose({door_hinge: door_limits.upper}):
            open_aabb = ctx.part_element_world_aabb(service_door, elem="door_panel")
        ctx.check(
            "service door swings out from the cabinet face",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[0][0] < closed_aabb[0][0] - 0.14,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )

    middle_button = object_model.get_part("button_1")
    outer_button = object_model.get_part("button_0")
    middle_joint = object_model.get_articulation("cabinet_to_button_1")
    middle_limits = middle_joint.motion_limits
    if middle_limits is not None and middle_limits.upper is not None:
        rest_middle = ctx.part_world_position(middle_button)
        rest_outer = ctx.part_world_position(outer_button)
        with ctx.pose({middle_joint: middle_limits.upper}):
            pressed_middle = ctx.part_world_position(middle_button)
            pressed_outer = ctx.part_world_position(outer_button)

        ctx.check(
            "middle start button presses down into the deck",
            rest_middle is not None
            and pressed_middle is not None
            and pressed_middle[0] > rest_middle[0] + 0.0015
            and pressed_middle[2] < rest_middle[2] - 0.006,
            details=f"rest={rest_middle}, pressed={pressed_middle}",
        )
        ctx.check(
            "start buttons move independently",
            rest_outer is not None
            and pressed_outer is not None
            and abs(pressed_outer[0] - rest_outer[0]) < 1e-6
            and abs(pressed_outer[1] - rest_outer[1]) < 1e-6
            and abs(pressed_outer[2] - rest_outer[2]) < 1e-6,
            details=f"rest_outer={rest_outer}, pressed_outer={pressed_outer}",
        )

    return ctx.report()


object_model = build_object_model()
