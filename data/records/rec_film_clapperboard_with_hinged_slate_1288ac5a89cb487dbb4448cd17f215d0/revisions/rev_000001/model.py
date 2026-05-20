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


BOARD_WIDTH = 0.280
BOARD_HEIGHT = 0.190
BOARD_THICKNESS = 0.007

CLAP_WIDTH = 0.286
CLAP_HEIGHT = 0.040
CLAP_THICKNESS = 0.012

HINGE_Y = 0.0045
HINGE_Z = BOARD_HEIGHT / 2.0 + 0.0017
HINGE_PIN_RADIUS = 0.0018
HINGE_LEAF_HEIGHT = 0.010
HINGE_LEAF_THICKNESS = 0.002

HOUSING_WIDTH = 0.180
HOUSING_HEIGHT = 0.034
HOUSING_DEPTH = 0.018
HOUSING_WALL = 0.003
HOUSING_Z = 0.036

BUTTON_WIDTH = 0.015
BUTTON_HEIGHT = 0.007
BUTTON_DEPTH = 0.0045
BUTTON_TRAVEL = 0.0022
BUTTON_X_POSITIONS = (-0.048, -0.016, 0.016, 0.048)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="timecode_film_slate")

    board_white = model.material("board_white", rgba=(0.93, 0.94, 0.92, 1.0))
    border_black = model.material("border_black", rgba=(0.08, 0.08, 0.08, 1.0))
    guide_gray = model.material("guide_gray", rgba=(0.63, 0.64, 0.64, 1.0))
    clap_black = model.material("clap_black", rgba=(0.10, 0.10, 0.11, 1.0))
    clap_white = model.material("clap_white", rgba=(0.95, 0.95, 0.94, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.73, 0.75, 0.77, 1.0))
    housing_black = model.material("housing_black", rgba=(0.13, 0.13, 0.14, 1.0))
    housing_trim = model.material("housing_trim", rgba=(0.22, 0.22, 0.23, 1.0))
    display_lens = model.material("display_lens", rgba=(0.10, 0.05, 0.05, 0.88))
    display_glow = model.material("display_glow", rgba=(0.76, 0.18, 0.12, 0.60))
    button_dark = model.material("button_dark", rgba=(0.20, 0.20, 0.21, 1.0))
    button_light = model.material("button_light", rgba=(0.55, 0.56, 0.58, 1.0))

    board = model.part("board")
    board.visual(
        Box((BOARD_WIDTH, BOARD_THICKNESS, BOARD_HEIGHT)),
        material=board_white,
        name="board_panel",
    )

    face_plate_thickness = 0.0004
    face_plate_y = BOARD_THICKNESS / 2.0 + face_plate_thickness / 2.0

    board.visual(
        Box((BOARD_WIDTH, face_plate_thickness, 0.012)),
        origin=Origin(xyz=(0.0, face_plate_y, BOARD_HEIGHT / 2.0 - 0.006)),
        material=border_black,
        name="top_border",
    )
    board.visual(
        Box((BOARD_WIDTH, face_plate_thickness, 0.012)),
        origin=Origin(xyz=(0.0, face_plate_y, -BOARD_HEIGHT / 2.0 + 0.006)),
        material=border_black,
        name="bottom_border",
    )
    board.visual(
        Box((0.012, face_plate_thickness, BOARD_HEIGHT - 0.024)),
        origin=Origin(xyz=(-BOARD_WIDTH / 2.0 + 0.006, face_plate_y, 0.0)),
        material=border_black,
        name="left_border",
    )
    board.visual(
        Box((0.012, face_plate_thickness, BOARD_HEIGHT - 0.024)),
        origin=Origin(xyz=(BOARD_WIDTH / 2.0 - 0.006, face_plate_y, 0.0)),
        material=border_black,
        name="right_border",
    )

    guide_thickness = 0.0003
    guide_y = BOARD_THICKNESS / 2.0 + guide_thickness / 2.0
    for index, guide_z in enumerate((0.030, -0.002, -0.036)):
        board.visual(
            Box((0.238, guide_thickness, 0.0012)),
            origin=Origin(xyz=(0.010, guide_y, guide_z)),
            material=guide_gray,
            name=f"guide_{index}",
        )
    board.visual(
        Box((0.0012, guide_thickness, 0.108)),
        origin=Origin(xyz=(-0.084, guide_y, -0.008)),
        material=guide_gray,
        name="column_guide",
    )

    board.visual(
        Box((BOARD_WIDTH - 0.010, HINGE_LEAF_THICKNESS, HINGE_LEAF_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                BOARD_THICKNESS / 2.0 + HINGE_LEAF_THICKNESS / 2.0,
                BOARD_HEIGHT / 2.0 - HINGE_LEAF_HEIGHT / 2.0,
            )
        ),
        material=hinge_metal,
        name="hinge_leaf",
    )
    board.visual(
        Cylinder(radius=HINGE_PIN_RADIUS, length=BOARD_WIDTH - 0.010),
        origin=Origin(
            xyz=(0.0, HINGE_Y, HINGE_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=hinge_metal,
        name="hinge_pin",
    )

    display_housing = model.part("display_housing")
    display_housing.visual(
        Box((HOUSING_WIDTH, HOUSING_WALL, HOUSING_HEIGHT)),
        origin=Origin(xyz=(0.0, -HOUSING_DEPTH / 2.0 + HOUSING_WALL / 2.0, 0.0)),
        material=housing_black,
        name="back_plate",
    )
    display_housing.visual(
        Box((HOUSING_WIDTH, HOUSING_DEPTH, HOUSING_WALL)),
        origin=Origin(xyz=(0.0, 0.0, HOUSING_HEIGHT / 2.0 - HOUSING_WALL / 2.0)),
        material=housing_black,
        name="top_wall",
    )
    display_housing.visual(
        Box((HOUSING_WIDTH, HOUSING_DEPTH, HOUSING_WALL)),
        origin=Origin(xyz=(0.0, 0.0, -HOUSING_HEIGHT / 2.0 + HOUSING_WALL / 2.0)),
        material=housing_black,
        name="bottom_wall",
    )
    display_housing.visual(
        Box((HOUSING_WALL, HOUSING_DEPTH, HOUSING_HEIGHT - 2.0 * HOUSING_WALL)),
        origin=Origin(xyz=(-HOUSING_WIDTH / 2.0 + HOUSING_WALL / 2.0, 0.0, 0.0)),
        material=housing_black,
        name="left_wall",
    )
    display_housing.visual(
        Box((HOUSING_WALL, HOUSING_DEPTH, HOUSING_HEIGHT - 2.0 * HOUSING_WALL)),
        origin=Origin(xyz=(HOUSING_WIDTH / 2.0 - HOUSING_WALL / 2.0, 0.0, 0.0)),
        material=housing_black,
        name="right_wall",
    )
    display_housing.visual(
        Box((0.144, HOUSING_WALL, 0.005)),
        origin=Origin(xyz=(0.0, HOUSING_DEPTH / 2.0 - HOUSING_WALL / 2.0, 0.012)),
        material=housing_trim,
        name="upper_bezel",
    )
    display_housing.visual(
        Box((0.144, HOUSING_WALL, 0.004)),
        origin=Origin(xyz=(0.0, HOUSING_DEPTH / 2.0 - HOUSING_WALL / 2.0, -0.002)),
        material=housing_trim,
        name="mid_bezel",
    )
    display_housing.visual(
        Box((0.006, HOUSING_WALL, 0.017)),
        origin=Origin(xyz=(-0.058, HOUSING_DEPTH / 2.0 - HOUSING_WALL / 2.0, 0.006)),
        material=housing_trim,
        name="display_left_bezel",
    )
    display_housing.visual(
        Box((0.006, HOUSING_WALL, 0.017)),
        origin=Origin(xyz=(0.058, HOUSING_DEPTH / 2.0 - HOUSING_WALL / 2.0, 0.006)),
        material=housing_trim,
        name="display_right_bezel",
    )
    display_housing.visual(
        Box((0.110, 0.012, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=display_glow,
        name="timecode_core",
    )
    display_housing.visual(
        Box((0.114, 0.0018, 0.016)),
        origin=Origin(xyz=(0.0, HOUSING_DEPTH / 2.0 - 0.0010, 0.006)),
        material=display_lens,
        name="timecode_lens",
    )
    guide_thickness_x = 0.0015
    for index, button_x in enumerate(BUTTON_X_POSITIONS):
        for side, sign in (("left", -1.0), ("right", 1.0)):
            display_housing.visual(
                Box((guide_thickness_x, 0.006, 0.009)),
                origin=Origin(
                    xyz=(
                        button_x + sign * (BUTTON_WIDTH / 2.0 + guide_thickness_x / 2.0),
                        0.0090,
                        -0.010,
                    )
                ),
                material=housing_trim,
                name=f"button_{index}_{side}_guide",
            )

    model.articulation(
        "board_to_display_housing",
        ArticulationType.FIXED,
        parent=board,
        child=display_housing,
        origin=Origin(
            xyz=(
                0.0,
                BOARD_THICKNESS / 2.0 + HOUSING_DEPTH / 2.0,
                HOUSING_Z,
            )
        ),
    )

    clapstick = model.part("clapstick")
    clapstick.visual(
        Box((CLAP_WIDTH, CLAP_THICKNESS, CLAP_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0078, -CLAP_HEIGHT / 2.0)),
        material=clap_black,
        name="bar",
    )
    clapstick.visual(
        Box((CLAP_WIDTH - 0.012, 0.003, 0.006)),
        origin=Origin(xyz=(0.0, 0.0105, -CLAP_HEIGHT + 0.003)),
        material=clap_white,
        name="strike_edge",
    )

    stripe_angle = math.radians(33.0)
    stripe_x_positions = (-0.112, -0.072, -0.032, 0.008, 0.048, 0.088)
    for index, stripe_x in enumerate(stripe_x_positions):
        clapstick.visual(
            Box((0.032, 0.0012, 0.060)),
            origin=Origin(
                xyz=(stripe_x, 0.0144, -0.019),
                rpy=(0.0, stripe_angle, 0.0),
            ),
            material=clap_white,
            name=f"stripe_{index}",
        )

    model.articulation(
        "board_to_clapstick",
        ArticulationType.REVOLUTE,
        parent=board,
        child=clapstick,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=4.5,
            lower=0.0,
            upper=1.15,
        ),
    )

    for index, button_x in enumerate(BUTTON_X_POSITIONS):
        button = model.part(f"button_{index}")
        button.visual(
            Box((BUTTON_WIDTH, BUTTON_DEPTH, BUTTON_HEIGHT)),
            material=button_light if index in (1, 2) else button_dark,
            name="cap",
        )
        button.visual(
            Box((0.010, 0.008, 0.004)),
            origin=Origin(xyz=(0.0, -0.0055, 0.0)),
            material=button_dark,
            name="stem",
        )
        model.articulation(
            f"display_housing_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=display_housing,
            child=button,
            origin=Origin(xyz=(button_x, HOUSING_DEPTH / 2.0, -0.010)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=0.10,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    board = object_model.get_part("board")
    clapstick = object_model.get_part("clapstick")
    display_housing = object_model.get_part("display_housing")
    clap_joint = object_model.get_articulation("board_to_clapstick")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")
    button_2 = object_model.get_part("button_2")
    button_3 = object_model.get_part("button_3")
    button_joint_0 = object_model.get_articulation("display_housing_to_button_0")
    button_joint_1 = object_model.get_articulation("display_housing_to_button_1")
    button_joint_2 = object_model.get_articulation("display_housing_to_button_2")
    button_joint_3 = object_model.get_articulation("display_housing_to_button_3")

    ctx.expect_overlap(
        clapstick,
        board,
        axes="x",
        min_overlap=0.24,
        name="clapstick spans nearly full board width",
    )
    ctx.expect_within(
        display_housing,
        board,
        axes="x",
        margin=0.004,
        name="display housing stays centered on the board width",
    )

    board_aabb = ctx.part_world_aabb(board)
    housing_aabb = ctx.part_world_aabb(display_housing)
    closed_clap_aabb = ctx.part_world_aabb(clapstick)
    ctx.check(
        "aabbs_available_at_rest",
        board_aabb is not None and housing_aabb is not None and closed_clap_aabb is not None,
        details=f"board={board_aabb}, housing={housing_aabb}, clapstick={closed_clap_aabb}",
    )

    board_panel_aabb = ctx.part_element_world_aabb(board, elem="board_panel")
    ctx.check(
        "board panel aabb available",
        board_panel_aabb is not None,
        details=f"board_panel_aabb={board_panel_aabb}",
    )

    if board_aabb is not None and housing_aabb is not None and closed_clap_aabb is not None:
        board_min, board_max = board_aabb
        housing_min, housing_max = housing_aabb
        clap_min, clap_max = closed_clap_aabb
        ctx.check(
            "display housing mounted beneath hinge line",
            housing_max[2] <= HINGE_Z - 0.006,
            details=f"housing_max_z={housing_max[2]:.5f}, hinge_z={HINGE_Z:.5f}",
        )
        ctx.check(
            "closed clapstick sits just in front of the hinge hardware",
            -0.0002 <= clap_min[1] - board_max[1] <= 0.0012,
            details=f"board_max_y={board_max[1]:.5f}, clap_back={clap_min[1]:.5f}",
        )
        ctx.check(
            "closed clapstick covers the upper writing area",
            clap_min[2] < board_max[2] - 0.020 and clap_max[2] > board_max[2] - 0.002,
            details=f"clap_z=({clap_min[2]:.5f}, {clap_max[2]:.5f}), board_top={board_max[2]:.5f}",
        )
        ctx.check(
            "display housing is narrower than the board",
            (housing_max[0] - housing_min[0]) < (board_max[0] - board_min[0]) - 0.070,
            details=(
                f"housing_width={housing_max[0] - housing_min[0]:.5f}, "
                f"board_width={board_max[0] - board_min[0]:.5f}"
            ),
        )
        if board_panel_aabb is not None:
            panel_min, panel_max = board_panel_aabb
            ctx.check(
                "closed clapstick stands off from the writing surface",
                0.0020 <= clap_min[1] - panel_max[1] <= 0.0045,
                details=f"panel_front={panel_max[1]:.5f}, clap_back={clap_min[1]:.5f}",
            )

    upper_clap = clap_joint.motion_limits.upper if clap_joint.motion_limits is not None else None
    ctx.check("clapstick_has_upper_limit", upper_clap is not None, details=f"limits={clap_joint.motion_limits}")
    if upper_clap is not None and closed_clap_aabb is not None:
        closed_min, closed_max = closed_clap_aabb
        with ctx.pose({clap_joint: upper_clap}):
            open_clap_aabb = ctx.part_world_aabb(clapstick)
        ctx.check(
            "open clapstick aabb available",
            open_clap_aabb is not None,
            details=f"open_clap_aabb={open_clap_aabb}",
        )
        if open_clap_aabb is not None:
            open_min, open_max = open_clap_aabb
            ctx.check(
                "clapstick opens outward",
                open_max[1] > closed_max[1] + 0.030,
                details=f"closed_max_y={closed_max[1]:.5f}, open_max_y={open_max[1]:.5f}",
            )
            ctx.check(
                "clapstick free edge lifts when opened",
                open_min[2] > closed_min[2] + 0.012,
                details=f"closed_min_z={closed_min[2]:.5f}, open_min_z={open_min[2]:.5f}",
            )

    for button in (button_0, button_1, button_2, button_3):
        ctx.expect_within(
            button,
            display_housing,
            axes="xz",
            margin=0.006,
            name=f"{button.name} stays within display housing footprint",
        )

    upper_button = button_joint_0.motion_limits.upper if button_joint_0.motion_limits is not None else None
    rest_button_0 = ctx.part_world_position(button_0)
    rest_button_1 = ctx.part_world_position(button_1)
    ctx.check(
        "button_rest_positions_available",
        rest_button_0 is not None and rest_button_1 is not None,
        details=f"button_0={rest_button_0}, button_1={rest_button_1}",
    )
    if upper_button is not None and rest_button_0 is not None and rest_button_1 is not None:
        with ctx.pose({button_joint_0: upper_button}):
            pressed_button_0 = ctx.part_world_position(button_0)
            untouched_button_1 = ctx.part_world_position(button_1)
            ctx.expect_within(
                button_0,
                display_housing,
                axes="xz",
                margin=0.006,
                name="button_0 stays captured while pressed",
            )
        ctx.check(
            "button_0 presses inward",
            pressed_button_0 is not None and pressed_button_0[1] < rest_button_0[1] - 0.0015,
            details=f"rest={rest_button_0}, pressed={pressed_button_0}",
        )
        ctx.check(
            "buttons articulate independently",
            untouched_button_1 is not None and abs(untouched_button_1[1] - rest_button_1[1]) < 1e-6,
            details=f"button_1_rest={rest_button_1}, button_1_during_button_0_press={untouched_button_1}",
        )

    all_button_press = {
        button_joint_0: button_joint_0.motion_limits.upper,
        button_joint_1: button_joint_1.motion_limits.upper,
        button_joint_2: button_joint_2.motion_limits.upper,
        button_joint_3: button_joint_3.motion_limits.upper,
    }
    with ctx.pose(all_button_press):
        for button in (button_0, button_1, button_2, button_3):
            ctx.expect_within(
                button,
                display_housing,
                axes="xz",
                margin=0.006,
                name=f"{button.name} stays aligned when fully pressed",
            )

    return ctx.report()


object_model = build_object_model()
