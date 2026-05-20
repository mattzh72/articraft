from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="premium_desktop_monitor")

    model.material("shell_dark", color=(0.14, 0.14, 0.15))
    model.material("screen_black", color=(0.04, 0.05, 0.06))
    model.material("stand_graphite", color=(0.22, 0.23, 0.25))
    model.material("button_black", color=(0.08, 0.08, 0.09))

    base_plate = model.part("base_plate")
    base_plate.visual(
        Box((0.30, 0.22, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material="stand_graphite",
        name="plate",
    )

    stand_body = model.part("stand_body")
    stand_body.visual(
        Cylinder(radius=0.052, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material="stand_graphite",
        name="swivel_hub",
    )
    stand_body.visual(
        Box((0.080, 0.045, 0.285)),
        origin=Origin(xyz=(0.0, 0.0, 0.1605)),
        material="stand_graphite",
        name="column",
    )
    stand_body.visual(
        Box((0.12, 0.05, 0.09)),
        origin=Origin(xyz=(0.0, 0.040, 0.258)),
        material="stand_graphite",
        name="neck",
    )
    stand_body.visual(
        Box((0.15, 0.065, 0.14)),
        origin=Origin(xyz=(0.0, 0.0825, 0.258)),
        material="stand_graphite",
        name="head_block",
    )

    tilt_carrier = model.part("tilt_carrier")
    tilt_carrier.visual(
        Box((0.13, 0.025, 0.12)),
        origin=Origin(xyz=(0.0, 0.0125, 0.0)),
        material="stand_graphite",
        name="tilt_plate",
    )
    tilt_carrier.visual(
        Cylinder(radius=0.052, length=0.03),
        origin=Origin(xyz=(0.0, 0.040, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="stand_graphite",
        name="pivot_drum",
    )

    display = model.part("display")
    display.visual(
        Box((0.622, 0.028, 0.366)),
        material="shell_dark",
        name="shell",
    )
    display.visual(
        Box((0.622, 0.032, 0.048)),
        origin=Origin(xyz=(0.0, 0.0, -0.159)),
        material="shell_dark",
        name="lower_chin",
    )
    display.visual(
        Box((0.596, 0.002, 0.338)),
        origin=Origin(xyz=(0.0, 0.014, 0.008)),
        material="screen_black",
        name="panel",
    )
    display.visual(
        Box((0.18, 0.016, 0.14)),
        origin=Origin(xyz=(0.0, -0.022, 0.0)),
        material="shell_dark",
        name="rear_boss",
    )

    button_x_positions = (-0.048, -0.020, 0.008, 0.036)
    button_parts = []
    for index, x_pos in enumerate(button_x_positions):
        button = model.part(f"menu_button_{index}")
        button.visual(
            Box((0.014, 0.007, 0.004)),
            origin=Origin(xyz=(0.0, 0.0, -0.002)),
            material="button_black",
            name="button_cap",
        )
        button_parts.append((button, x_pos))

    model.articulation(
        "base_to_stand",
        ArticulationType.CONTINUOUS,
        parent=base_plate,
        child=stand_body,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=4.0),
    )
    model.articulation(
        "stand_to_tilt",
        ArticulationType.REVOLUTE,
        parent=stand_body,
        child=tilt_carrier,
        origin=Origin(xyz=(0.0, 0.115, 0.258)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.5,
            lower=-0.22,
            upper=0.42,
        ),
    )
    model.articulation(
        "tilt_to_display",
        ArticulationType.CONTINUOUS,
        parent=tilt_carrier,
        child=display,
        origin=Origin(xyz=(0.0, 0.085, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=4.0),
    )

    for index, (button, x_pos) in enumerate(button_parts):
        model.articulation(
            f"display_to_menu_button_{index}",
            ArticulationType.PRISMATIC,
            parent=display,
            child=button,
            origin=Origin(xyz=(x_pos, 0.006, -0.183)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=0.05,
                lower=0.0,
                upper=0.0015,
            ),
        )

    return model


def _aabb_extents(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    return tuple(aabb[1][axis] - aabb[0][axis] for axis in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base_plate = object_model.get_part("base_plate")
    stand_body = object_model.get_part("stand_body")
    tilt_carrier = object_model.get_part("tilt_carrier")
    display = object_model.get_part("display")
    menu_button_0 = object_model.get_part("menu_button_0")
    menu_button_1 = object_model.get_part("menu_button_1")

    swivel = object_model.get_articulation("base_to_stand")
    tilt = object_model.get_articulation("stand_to_tilt")
    pivot = object_model.get_articulation("tilt_to_display")
    button_press = object_model.get_articulation("display_to_menu_button_0")

    ctx.expect_contact(
        stand_body,
        base_plate,
        elem_a="swivel_hub",
        elem_b="plate",
        name="swivel hub sits on the base plate",
    )
    ctx.expect_contact(
        tilt_carrier,
        stand_body,
        elem_a="tilt_plate",
        elem_b="head_block",
        name="tilt carrier seats against the stand head",
    )
    ctx.expect_contact(
        display,
        tilt_carrier,
        elem_a="rear_boss",
        elem_b="pivot_drum",
        name="display mount meets the pivot drum",
    )
    ctx.expect_contact(
        menu_button_0,
        display,
        elem_a="button_cap",
        elem_b="lower_chin",
        name="menu buttons sit under the lower bezel",
    )
    ctx.expect_gap(
        display,
        base_plate,
        axis="z",
        min_gap=0.04,
        name="display clears the desk base",
    )

    rest_display_pos = ctx.part_world_position(display)
    rest_button_0_pos = ctx.part_world_position(menu_button_0)
    rest_button_1_pos = ctx.part_world_position(menu_button_1)
    rest_display_aabb = ctx.part_world_aabb(display)

    tilt_upper = tilt.motion_limits.upper if tilt.motion_limits is not None else None
    if tilt_upper is not None:
        with ctx.pose({tilt: tilt_upper}):
            tilted_display_pos = ctx.part_world_position(display)
        ctx.check(
            "positive tilt leans the display back",
            rest_display_pos is not None
            and tilted_display_pos is not None
            and tilted_display_pos[1] < rest_display_pos[1] - 0.005
            and tilted_display_pos[2] > rest_display_pos[2] + 0.02,
            details=f"rest={rest_display_pos}, tilted={tilted_display_pos}",
        )

    with ctx.pose({swivel: pi / 2.0}):
        swiveled_display_pos = ctx.part_world_position(display)
    ctx.check(
        "stand swivel turns the display around the base axis",
        rest_display_pos is not None
        and swiveled_display_pos is not None
        and abs(swiveled_display_pos[0]) > 0.15
        and abs(swiveled_display_pos[1]) < 0.06,
        details=f"rest={rest_display_pos}, swiveled={swiveled_display_pos}",
    )

    with ctx.pose({pivot: pi / 2.0}):
        portrait_display_aabb = ctx.part_world_aabb(display)
    rest_extents = _aabb_extents(rest_display_aabb)
    portrait_extents = _aabb_extents(portrait_display_aabb)
    ctx.check(
        "screen pivots into a portrait footprint",
        rest_extents is not None
        and portrait_extents is not None
        and rest_extents[0] > rest_extents[2]
        and portrait_extents[2] > portrait_extents[0]
        and portrait_extents[2] > 0.55,
        details=f"rest_extents={rest_extents}, portrait_extents={portrait_extents}",
    )

    button_upper = button_press.motion_limits.upper if button_press.motion_limits is not None else None
    if button_upper is not None:
        with ctx.pose({button_press: button_upper}):
            pressed_button_0_pos = ctx.part_world_position(menu_button_0)
            pressed_button_1_pos = ctx.part_world_position(menu_button_1)
        ctx.check(
            "menu buttons press independently",
            rest_button_0_pos is not None
            and rest_button_1_pos is not None
            and pressed_button_0_pos is not None
            and pressed_button_1_pos is not None
            and pressed_button_0_pos[2] > rest_button_0_pos[2] + 0.001
            and abs(pressed_button_1_pos[2] - rest_button_1_pos[2]) < 1e-5,
            details=(
                f"button_0_rest={rest_button_0_pos}, button_0_pressed={pressed_button_0_pos}, "
                f"button_1_rest={rest_button_1_pos}, button_1_pressed={pressed_button_1_pos}"
            ),
        )

    return ctx.report()


object_model = build_object_model()
