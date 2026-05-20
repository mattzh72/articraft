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


WIDTH = 0.68
LOWER_WIDTH = 0.64
LOWER_DEPTH = 0.52
LOWER_HEIGHT = 0.86
PLINTH_DEPTH = 0.60
PLINTH_HEIGHT = 0.08
DECK_WIDTH = 0.60
DECK_DEPTH = 0.19
DECK_HEIGHT = 0.05
DISPLAY_ANGLE = math.radians(18.0)
SCREEN_FRAME_WIDTH = 0.54
SCREEN_FRAME_HEIGHT = 0.36
SCREEN_FRAME_DEPTH = 0.022
SCREEN_GLASS_WIDTH = 0.46
SCREEN_GLASS_HEIGHT = 0.29
DOOR_WIDTH = 0.40
DOOR_HEIGHT = 0.52
DOOR_THICKNESS = 0.024
BUTTON_RADIUS = 0.028
BUTTON_HEIGHT = 0.018
BUTTON_TRAVEL = 0.003


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slant_top_slot_machine")

    cabinet_paint = model.material("cabinet_paint", rgba=(0.12, 0.14, 0.17, 1.0))
    cabinet_trim = model.material("cabinet_trim", rgba=(0.22, 0.25, 0.29, 1.0))
    bezel_black = model.material("bezel_black", rgba=(0.05, 0.06, 0.07, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.12, 0.28, 0.34, 0.45))
    satin_silver = model.material("satin_silver", rgba=(0.70, 0.72, 0.76, 1.0))
    dark_handle = model.material("dark_handle", rgba=(0.13, 0.13, 0.14, 1.0))
    button_red = model.material("button_red", rgba=(0.82, 0.17, 0.18, 0.96))
    button_blue = model.material("button_blue", rgba=(0.16, 0.43, 0.86, 0.96))
    button_amber = model.material("button_amber", rgba=(0.90, 0.66, 0.14, 0.96))
    button_green = model.material("button_green", rgba=(0.18, 0.67, 0.31, 0.96))
    button_white = model.material("button_white", rgba=(0.92, 0.94, 0.96, 0.98))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((WIDTH, PLINTH_DEPTH, PLINTH_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.02, PLINTH_HEIGHT / 2.0)),
        material=cabinet_trim,
        name="base_plinth",
    )
    cabinet.visual(
        Box((LOWER_WIDTH, LOWER_DEPTH, LOWER_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.02, 0.48)),
        material=cabinet_paint,
        name="lower_body",
    )
    cabinet.visual(
        Box((0.66, 0.36, 0.58)),
        origin=Origin(xyz=(0.0, 0.06, 1.49)),
        material=cabinet_paint,
        name="upper_body",
    )
    cabinet.visual(
        Box((0.56, 0.07, 0.40)),
        origin=Origin(xyz=(0.0, -0.15, 1.27), rpy=(-DISPLAY_ANGLE, 0.0, 0.0)),
        material=bezel_black,
        name="display_bezel",
    )
    cabinet.visual(
        Box((0.58, 0.14, 0.28)),
        origin=Origin(xyz=(0.0, -0.04, 1.08)),
        material=cabinet_paint,
        name="display_riser",
    )
    cabinet.visual(
        Box((DECK_WIDTH, DECK_DEPTH, DECK_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.21, 0.94)),
        material=cabinet_trim,
        name="deck_core",
    )
    cabinet.visual(
        Box((0.08, 0.22, 0.12)),
        origin=Origin(xyz=(-0.25, -0.18, 0.88)),
        material=cabinet_trim,
        name="deck_support_0",
    )
    cabinet.visual(
        Box((0.08, 0.22, 0.12)),
        origin=Origin(xyz=(0.25, -0.18, 0.88)),
        material=cabinet_trim,
        name="deck_support_1",
    )
    cabinet.visual(
        Box((0.60, 0.03, 0.06)),
        origin=Origin(xyz=(0.0, -0.295, 0.92)),
        material=bezel_black,
        name="deck_lip",
    )
    cabinet.visual(
        Box((0.58, 0.18, 0.012)),
        origin=Origin(xyz=(0.0, -0.205, 0.963)),
        material=cabinet_trim,
        name="button_deck",
    )
    cabinet.visual(
        Box((0.56, 0.05, 0.018)),
        origin=Origin(xyz=(0.0, -0.255, 0.972)),
        material=bezel_black,
        name="button_rail",
    )
    cabinet.visual(
        Cylinder(radius=0.008, length=0.56),
        origin=Origin(xyz=(0.216, -0.294, 0.38)),
        material=satin_silver,
        name="hinge_mount",
    )
    cabinet.visual(
        Box((0.016, 0.060, 0.56)),
        origin=Origin(xyz=(0.222, -0.264, 0.38)),
        material=satin_silver,
        name="hinge_bracket",
    )

    display = model.part("display")
    display.visual(
        Box((SCREEN_FRAME_WIDTH, SCREEN_FRAME_DEPTH, SCREEN_FRAME_HEIGHT)),
        material=bezel_black,
        name="screen_frame",
    )
    display.visual(
        Box((SCREEN_GLASS_WIDTH, 0.004, SCREEN_GLASS_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.009, 0.0)),
        material=smoked_glass,
        name="screen_glass",
    )
    display.visual(
        Box((0.08, 0.009638, 0.06)),
        origin=Origin(xyz=(0.0, 0.015819, 0.0)),
        material=bezel_black,
        name="screen_standoff",
    )
    model.articulation(
        "cabinet_to_display",
        ArticulationType.FIXED,
        parent=cabinet,
        child=display,
        origin=Origin(xyz=(0.0, -0.215, 1.25), rpy=(-DISPLAY_ANGLE, 0.0, 0.0)),
    )

    belly_door = model.part("belly_door")
    belly_door.visual(
        Cylinder(radius=0.008, length=DOOR_HEIGHT),
        material=satin_silver,
        name="hinge_barrel",
    )
    belly_door.visual(
        Box((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(-DOOR_WIDTH / 2.0, 0.014, 0.0)),
        material=cabinet_trim,
        name="door_panel",
    )
    belly_door.visual(
        Box((0.30, 0.006, 0.40)),
        origin=Origin(xyz=(-0.20, 0.000, 0.0)),
        material=bezel_black,
        name="door_inset",
    )
    belly_door.visual(
        Box((0.018, 0.018, 0.14)),
        origin=Origin(xyz=(-0.09, -0.007, 0.01)),
        material=dark_handle,
        name="door_pull",
    )
    belly_door.visual(
        Cylinder(radius=0.013, length=0.010),
        origin=Origin(xyz=(-0.06, -0.003, -0.10), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_silver,
        name="door_lock",
    )
    model.articulation(
        "cabinet_to_belly_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=belly_door,
        origin=Origin(xyz=(DOOR_WIDTH / 2.0, -0.294, 0.38)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.6,
            lower=0.0,
            upper=1.45,
        ),
    )

    button_colors = (
        button_red,
        button_blue,
        button_amber,
        button_green,
        button_white,
    )
    button_x_positions = (-0.24, -0.12, 0.0, 0.12, 0.24)
    button_z_origin = 0.972 + 0.009
    for index, (button_x, button_color) in enumerate(zip(button_x_positions, button_colors)):
        button = model.part(f"play_button_{index}")
        button.visual(
            Cylinder(radius=BUTTON_RADIUS, length=BUTTON_HEIGHT),
            origin=Origin(xyz=(0.0, 0.0, BUTTON_HEIGHT / 2.0)),
            material=button_color,
            name="button_cap",
        )
        button.visual(
            Cylinder(radius=BUTTON_RADIUS * 0.78, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, BUTTON_HEIGHT - 0.003)),
            material=smoked_glass,
            name="button_lens",
        )
        model.articulation(
            f"cabinet_to_play_button_{index}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=button,
            origin=Origin(xyz=(button_x, -0.255, button_z_origin)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.10,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    belly_door = object_model.get_part("belly_door")
    door_hinge = object_model.get_articulation("cabinet_to_belly_door")

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_overlap(
            belly_door,
            cabinet,
            axes="xz",
            elem_a="door_panel",
            elem_b="lower_body",
            min_overlap=0.35,
            name="belly door covers the lower service area when closed",
        )

    rest_pull_aabb = ctx.part_element_world_aabb(belly_door, elem="door_pull")
    with ctx.pose({door_hinge: 1.20}):
        open_pull_aabb = ctx.part_element_world_aabb(belly_door, elem="door_pull")
    ctx.check(
        "belly door swings outward from the cabinet front",
        rest_pull_aabb is not None
        and open_pull_aabb is not None
        and open_pull_aabb[0][1] < rest_pull_aabb[0][1] - 0.06,
        details=f"closed={rest_pull_aabb}, open={open_pull_aabb}",
    )

    button_positions = []
    center_button = object_model.get_part("play_button_2")
    center_button_joint = object_model.get_articulation("cabinet_to_play_button_2")
    for index in range(5):
        button = object_model.get_part(f"play_button_{index}")
        button_positions.append(ctx.part_world_position(button))
        with ctx.pose({f"cabinet_to_play_button_{index}": 0.0}):
            ctx.expect_gap(
                button,
                cabinet,
                axis="z",
                positive_elem="button_cap",
                negative_elem="button_rail",
                min_gap=0.0,
                max_gap=0.0005,
                max_penetration=0.0,
                name=f"play button {index} seats onto the button rail at rest",
            )

    xs = [pos[0] for pos in button_positions if pos is not None]
    ys = [pos[1] for pos in button_positions if pos is not None]
    zs = [pos[2] for pos in button_positions if pos is not None]
    ctx.check(
        "play buttons form a clean leading-edge row",
        len(xs) == 5
        and xs == sorted(xs)
        and max(ys) - min(ys) < 0.002
        and max(zs) - min(zs) < 0.002,
        details=f"positions={button_positions}",
    )

    center_rest = ctx.part_world_position(center_button)
    with ctx.pose({center_button_joint: BUTTON_TRAVEL}):
        center_pressed = ctx.part_world_position(center_button)
    ctx.check(
        "center play button moves downward when pressed",
        center_rest is not None
        and center_pressed is not None
        and center_pressed[2] < center_rest[2] - 0.0025,
        details=f"rest={center_rest}, pressed={center_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
