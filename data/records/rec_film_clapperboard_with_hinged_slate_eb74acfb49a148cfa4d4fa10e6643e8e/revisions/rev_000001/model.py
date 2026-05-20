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


BOARD_WIDTH = 0.310
BOARD_HEIGHT = 0.240
BOARD_THICKNESS = 0.008
WHITEBOARD_WIDTH = 0.286
WHITEBOARD_HEIGHT = 0.206
WHITEBOARD_THICKNESS = 0.003
WHITEBOARD_BOTTOM = 0.015

HOUSING_WIDTH = 0.246
HOUSING_HEIGHT = 0.056
HOUSING_DEPTH = 0.034

DISPLAY_WIDTH = 0.138
DISPLAY_HEIGHT = 0.014
DISPLAY_DEPTH = 0.003
BEZEL_WIDTH = 0.176
BEZEL_HEIGHT = 0.028
BEZEL_DEPTH = 0.006

CLAPSTICK_WIDTH = 0.340
CLAPSTICK_DEPTH = 0.044
CLAPSTICK_THICKNESS = 0.017

BATTERY_DOOR_WIDTH = 0.126
BATTERY_DOOR_HEIGHT = 0.032
BATTERY_DOOR_THICKNESS = 0.003
BATTERY_DOOR_HINGE_Z = 0.010


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="smart_clapperboard")

    frame_black = model.material("frame_black", rgba=(0.08, 0.08, 0.09, 1.0))
    matte_white = model.material("matte_white", rgba=(0.96, 0.96, 0.95, 1.0))
    housing_black = model.material("housing_black", rgba=(0.10, 0.10, 0.11, 1.0))
    bezel_gray = model.material("bezel_gray", rgba=(0.22, 0.23, 0.25, 1.0))
    display_red = model.material("display_red", rgba=(0.72, 0.07, 0.05, 1.0))
    latch_gray = model.material("latch_gray", rgba=(0.44, 0.45, 0.47, 1.0))

    board = model.part("board")
    board.visual(
        Box((BOARD_WIDTH, BOARD_THICKNESS, BOARD_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BOARD_HEIGHT / 2.0)),
        material=frame_black,
        name="board_back",
    )
    board.visual(
        Box((WHITEBOARD_WIDTH, WHITEBOARD_THICKNESS, WHITEBOARD_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                BOARD_THICKNESS / 2.0 - WHITEBOARD_THICKNESS / 2.0,
                WHITEBOARD_BOTTOM + WHITEBOARD_HEIGHT / 2.0,
            )
        ),
        material=matte_white,
        name="whiteboard_face",
    )

    housing = model.part("housing")
    housing.visual(
        Box((HOUSING_WIDTH, HOUSING_DEPTH, HOUSING_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, HOUSING_HEIGHT / 2.0)),
        material=housing_black,
        name="housing_body",
    )
    housing.visual(
        Box((BEZEL_WIDTH, BEZEL_DEPTH, BEZEL_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                HOUSING_DEPTH / 2.0 + BEZEL_DEPTH / 2.0 - 0.0003,
                HOUSING_HEIGHT * 0.58,
            )
        ),
        material=bezel_gray,
        name="display_bezel",
    )
    housing.visual(
        Box((DISPLAY_WIDTH, DISPLAY_DEPTH, DISPLAY_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                HOUSING_DEPTH / 2.0 + DISPLAY_DEPTH / 2.0,
                HOUSING_HEIGHT * 0.58,
            )
        ),
        material=display_red,
        name="timecode_display",
    )

    clapstick = model.part("clapstick")
    clapstick.visual(
        Box((CLAPSTICK_WIDTH, CLAPSTICK_DEPTH, CLAPSTICK_THICKNESS)),
        origin=Origin(
            xyz=(0.0, CLAPSTICK_DEPTH / 2.0, -CLAPSTICK_THICKNESS / 2.0)
        ),
        material=frame_black,
        name="clap_bar",
    )
    stripe_centers = (-0.125, -0.065, -0.005, 0.055, 0.115)
    for index, x_center in enumerate(stripe_centers):
        clapstick.visual(
            Box((0.052, 0.0032, 0.026)),
            origin=Origin(
                xyz=(x_center, CLAPSTICK_DEPTH - 0.0016, -CLAPSTICK_THICKNESS / 2.0),
                rpy=(0.0, 0.68, 0.0),
            ),
            material=matte_white,
            name=f"stripe_{index}",
        )

    battery_door = model.part("battery_door")
    battery_door.visual(
        Box((BATTERY_DOOR_WIDTH, BATTERY_DOOR_THICKNESS, BATTERY_DOOR_HEIGHT)),
        origin=Origin(
            xyz=(0.0, -BATTERY_DOOR_THICKNESS / 2.0, BATTERY_DOOR_HEIGHT / 2.0)
        ),
        material=bezel_gray,
        name="battery_panel",
    )
    battery_door.visual(
        Box((0.034, 0.003, 0.005)),
        origin=Origin(xyz=(0.0, -BATTERY_DOOR_THICKNESS - 0.001, 0.024)),
        material=latch_gray,
        name="battery_latch",
    )

    model.articulation(
        "board_to_housing",
        ArticulationType.FIXED,
        parent=board,
        child=housing,
        origin=Origin(xyz=(0.0, 0.0, BOARD_HEIGHT)),
    )
    model.articulation(
        "housing_to_clapstick",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=clapstick,
        origin=Origin(
            xyz=(0.0, HOUSING_DEPTH / 2.0 + BEZEL_DEPTH - 0.0003, HOUSING_HEIGHT)
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=1.15,
            effort=8.0,
            velocity=4.0,
        ),
    )
    model.articulation(
        "housing_to_battery_door",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=battery_door,
        origin=Origin(
            xyz=(0.0, -HOUSING_DEPTH / 2.0, BATTERY_DOOR_HINGE_Z)
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=1.30,
            effort=2.0,
            velocity=3.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    board = object_model.get_part("board")
    housing = object_model.get_part("housing")
    clapstick = object_model.get_part("clapstick")
    battery_door = object_model.get_part("battery_door")
    clap_hinge = object_model.get_articulation("housing_to_clapstick")
    door_hinge = object_model.get_articulation("housing_to_battery_door")

    ctx.expect_gap(
        housing,
        board,
        axis="z",
        positive_elem="housing_body",
        negative_elem="board_back",
        max_gap=0.001,
        max_penetration=0.0,
        name="housing seats directly on the top of the slate board",
    )
    ctx.expect_gap(
        clapstick,
        housing,
        axis="y",
        positive_elem="clap_bar",
        negative_elem="display_bezel",
        max_gap=0.001,
        max_penetration=0.0,
        name="closed clapstick starts at the front of the timecode module",
    )
    ctx.expect_within(
        battery_door,
        housing,
        axes="xz",
        inner_elem="battery_panel",
        outer_elem="housing_body",
        margin=0.0,
        name="battery door fits within the rear housing footprint",
    )

    clap_limits = clap_hinge.motion_limits
    if clap_limits is not None and clap_limits.lower is not None and clap_limits.upper is not None:
        with ctx.pose({clap_hinge: clap_limits.lower}):
            closed_aabb = ctx.part_world_aabb(clapstick)
        with ctx.pose({clap_hinge: clap_limits.upper}):
            open_aabb = ctx.part_world_aabb(clapstick)
        ctx.check(
            "clapstick opens upward",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[1][2] > closed_aabb[1][2] + 0.020,
            details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
        )

    door_limits = door_hinge.motion_limits
    if door_limits is not None and door_limits.lower is not None and door_limits.upper is not None:
        with ctx.pose({door_hinge: door_limits.lower}):
            closed_aabb = ctx.part_world_aabb(battery_door)
        with ctx.pose({door_hinge: door_limits.upper}):
            open_aabb = ctx.part_world_aabb(battery_door)
        ctx.check(
            "battery door swings outward from the rear housing face",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[0][1] < closed_aabb[0][1] - 0.015,
            details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
