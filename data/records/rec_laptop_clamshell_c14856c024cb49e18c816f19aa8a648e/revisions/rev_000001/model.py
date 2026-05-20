from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


SHELL_WIDTH = 0.305
SHELL_DEPTH = 0.215
BODY_THICKNESS = 0.017
DECK_RISE = 0.004
DECK_TOP_Z = BODY_THICKNESS + DECK_RISE
REAR_SPINE_HEIGHT = 0.007
REAR_SPINE_TOP_Z = BODY_THICKNESS + REAR_SPINE_HEIGHT

LID_WIDTH = 0.298
LID_DEPTH = 0.208
LID_THICKNESS = 0.010
HINGE_Y = -0.1035
HINGE_Z = REAR_SPINE_TOP_Z

KEY_ROWS = 4
KEY_COLS = 7
KEY_WIDTH = 0.025
KEY_DEPTH = 0.016
KEY_HEIGHT = 0.0027
KEY_GAP_X = 0.004
KEY_GAP_Y = 0.004
KEY_START_X = -0.087
KEY_START_Y = -0.048
KEY_REST_Z = 0.0198
KEY_TRAVEL = 0.0012
SAMPLE_KEY_NAME = "key_r3_c4"


def _key_name(row: int, col: int) -> str:
    return f"key_r{row}_c{col}"


def _rect_profile(width: float, depth: float, *, cx: float = 0.0, cy: float = 0.0) -> list[tuple[float, float]]:
    hw = width / 2.0
    hd = depth / 2.0
    return [
        (cx - hw, cy - hd),
        (cx + hw, cy - hd),
        (cx + hw, cy + hd),
        (cx - hw, cy + hd),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="student_laptop")

    shell_silver = model.material("shell_silver", rgba=(0.73, 0.75, 0.78, 1.0))
    shell_dark = model.material("shell_dark", rgba=(0.17, 0.18, 0.20, 1.0))
    bezel_black = model.material("bezel_black", rgba=(0.07, 0.07, 0.08, 1.0))
    screen_black = model.material("screen_black", rgba=(0.05, 0.07, 0.09, 1.0))
    key_gray = model.material("key_gray", rgba=(0.87, 0.88, 0.90, 1.0))
    touchpad_gray = model.material("touchpad_gray", rgba=(0.62, 0.65, 0.69, 1.0))
    latch_gray = model.material("latch_gray", rgba=(0.69, 0.71, 0.74, 1.0))

    lower_shell = model.part("lower_shell")
    lower_shell.visual(
        Box((SHELL_WIDTH, SHELL_DEPTH, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=shell_silver,
        name="base_body",
    )
    lower_shell.visual(
        Box((0.008, SHELL_DEPTH, 0.014)),
        origin=Origin(xyz=(-0.1485, 0.0, 0.010)),
        material=shell_silver,
        name="left_sidewall",
    )
    lower_shell.visual(
        Box((0.008, SHELL_DEPTH, 0.014)),
        origin=Origin(xyz=(0.1485, 0.0, 0.010)),
        material=shell_silver,
        name="right_sidewall",
    )
    lower_shell.visual(
        Box((0.289, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, 0.1025, 0.010)),
        material=shell_silver,
        name="front_wall",
    )
    lower_shell.visual(
        Box((0.289, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, -0.1025, 0.010)),
        material=shell_silver,
        name="rear_wall",
    )
    lower_shell.visual(
        Box((SHELL_WIDTH, 0.076, DECK_RISE)),
        origin=Origin(xyz=(0.0, 0.0695, BODY_THICKNESS + DECK_RISE / 2.0)),
        material=shell_silver,
        name="front_palmrest",
    )
    lower_shell.visual(
        Box((0.033, 0.104, DECK_RISE)),
        origin=Origin(xyz=(-0.136, -0.008, BODY_THICKNESS + DECK_RISE / 2.0)),
        material=shell_silver,
        name="left_keyboard_rail",
    )
    lower_shell.visual(
        Box((0.033, 0.104, DECK_RISE)),
        origin=Origin(xyz=(0.136, -0.008, BODY_THICKNESS + DECK_RISE / 2.0)),
        material=shell_silver,
        name="right_keyboard_rail",
    )
    lower_shell.visual(
        Box((SHELL_WIDTH, 0.031, DECK_RISE)),
        origin=Origin(xyz=(0.0, -0.075, BODY_THICKNESS + DECK_RISE / 2.0)),
        material=shell_silver,
        name="rear_keyboard_rail",
    )
    lower_shell.visual(
        Box((SHELL_WIDTH, 0.020, REAR_SPINE_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.0975, BODY_THICKNESS + REAR_SPINE_HEIGHT / 2.0)),
        material=shell_silver,
        name="rear_spine",
    )

    keyboard_holes = []
    for row in range(KEY_ROWS):
        key_y = KEY_START_Y + row * (KEY_DEPTH + KEY_GAP_Y)
        for col in range(KEY_COLS):
            key_x = KEY_START_X + col * (KEY_WIDTH + KEY_GAP_X)
            keyboard_holes.append(_rect_profile(0.009, 0.009, cx=key_x, cy=key_y))
    keyboard_frame = ExtrudeWithHolesGeometry(
        _rect_profile(0.239, 0.091, cx=0.0, cy=-0.014),
        keyboard_holes,
        0.0012,
        center=True,
    )
    lower_shell.visual(
        mesh_from_geometry(keyboard_frame, "keyboard_frame"),
        origin=Origin(xyz=(0.0, 0.0, 0.0180)),
        material=shell_dark,
        name="keyboard_frame",
    )
    lower_shell.visual(
        Box((0.100, 0.060, 0.001)),
        origin=Origin(xyz=(0.0, 0.066, DECK_TOP_Z + 0.0005)),
        material=touchpad_gray,
        name="touchpad",
    )
    lower_shell.visual(
        Cylinder(radius=0.0035, length=0.250),
        origin=Origin(xyz=(0.0, -0.111, 0.0205), rpy=(0.0, pi / 2.0, 0.0)),
        material=shell_dark,
        name="hinge_barrel",
    )

    lid = model.part("lid")
    lid.visual(
        Box((LID_WIDTH, LID_DEPTH, LID_THICKNESS)),
        origin=Origin(xyz=(0.0, LID_DEPTH / 2.0, LID_THICKNESS / 2.0)),
        material=shell_silver,
        name="lid_back_cover",
    )
    lid.visual(
        Box((0.022, LID_DEPTH, 0.002)),
        origin=Origin(xyz=(-0.138, LID_DEPTH / 2.0, 0.009)),
        material=bezel_black,
        name="left_bezel",
    )
    lid.visual(
        Box((0.022, LID_DEPTH, 0.002)),
        origin=Origin(xyz=(0.138, LID_DEPTH / 2.0, 0.009)),
        material=bezel_black,
        name="right_bezel",
    )
    lid.visual(
        Box((LID_WIDTH, 0.018, 0.002)),
        origin=Origin(xyz=(0.0, 0.199, 0.009)),
        material=bezel_black,
        name="top_bezel",
    )
    lid.visual(
        Box((LID_WIDTH, 0.030, 0.002)),
        origin=Origin(xyz=(0.0, 0.015, 0.009)),
        material=bezel_black,
        name="bottom_bezel",
    )
    lid.visual(
        Box((0.254, 0.160, 0.0012)),
        origin=Origin(xyz=(0.0, 0.110, 0.0086)),
        material=screen_black,
        name="display_panel",
    )

    model.articulation(
        "lower_shell_to_lid",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=2.3),
    )

    left_latch = model.part("left_latch")
    left_latch.visual(
        Cylinder(radius=0.003, length=0.0025),
        origin=Origin(xyz=(0.0, 0.0, 0.00125)),
        material=latch_gray,
        name="left_latch_pivot",
    )
    left_latch.visual(
        Box((0.024, 0.008, 0.0025)),
        origin=Origin(xyz=(0.012, 0.0, 0.00125)),
        material=latch_gray,
        name="left_latch_tab",
    )
    left_latch.visual(
        Box((0.004, 0.010, 0.0025)),
        origin=Origin(xyz=(0.022, 0.0, 0.00125)),
        material=latch_gray,
        name="left_latch_hook",
    )
    model.articulation(
        "lower_shell_to_left_latch",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child=left_latch,
        origin=Origin(xyz=(-0.139, 0.090, DECK_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=5.0, lower=0.0, upper=0.65),
    )

    right_latch = model.part("right_latch")
    right_latch.visual(
        Cylinder(radius=0.003, length=0.0025),
        origin=Origin(xyz=(0.0, 0.0, 0.00125)),
        material=latch_gray,
        name="right_latch_pivot",
    )
    right_latch.visual(
        Box((0.024, 0.008, 0.0025)),
        origin=Origin(xyz=(-0.012, 0.0, 0.00125)),
        material=latch_gray,
        name="right_latch_tab",
    )
    right_latch.visual(
        Box((0.004, 0.010, 0.0025)),
        origin=Origin(xyz=(-0.022, 0.0, 0.00125)),
        material=latch_gray,
        name="right_latch_hook",
    )
    model.articulation(
        "lower_shell_to_right_latch",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child=right_latch,
        origin=Origin(xyz=(0.139, 0.090, DECK_TOP_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=5.0, lower=0.0, upper=0.65),
    )

    for row in range(KEY_ROWS):
        key_y = KEY_START_Y + row * (KEY_DEPTH + KEY_GAP_Y)
        for col in range(KEY_COLS):
            key_x = KEY_START_X + col * (KEY_WIDTH + KEY_GAP_X)
            key_name = _key_name(row + 1, col + 1)
            key_part = model.part(key_name)
            key_part.visual(
                Box((KEY_WIDTH, KEY_DEPTH, KEY_HEIGHT)),
                origin=Origin(xyz=(0.0, 0.0, KEY_HEIGHT / 2.0)),
                material=key_gray,
                name="keycap",
            )
            key_part.visual(
                Box((0.009, 0.009, 0.0030)),
                origin=Origin(xyz=(0.0, 0.0, -0.0015)),
                material=shell_dark,
                name="plunger",
            )
            model.articulation(
                f"lower_shell_to_{key_name}",
                ArticulationType.PRISMATIC,
                parent=lower_shell,
                child=key_part,
                origin=Origin(xyz=(key_x, key_y, KEY_REST_Z)),
                axis=(0.0, 0.0, -1.0),
                motion_limits=MotionLimits(
                    effort=0.5,
                    velocity=0.05,
                    lower=0.0,
                    upper=KEY_TRAVEL,
                ),
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    lower_shell = object_model.get_part("lower_shell")
    lid = object_model.get_part("lid")
    left_latch = object_model.get_part("left_latch")
    right_latch = object_model.get_part("right_latch")
    sample_key = object_model.get_part(SAMPLE_KEY_NAME)

    lid_joint = object_model.get_articulation("lower_shell_to_lid")
    left_latch_joint = object_model.get_articulation("lower_shell_to_left_latch")
    right_latch_joint = object_model.get_articulation("lower_shell_to_right_latch")
    sample_key_joint = object_model.get_articulation(f"lower_shell_to_{SAMPLE_KEY_NAME}")

    expected_parts = {
        "lower_shell",
        "lid",
        "left_latch",
        "right_latch",
        *{
            _key_name(row + 1, col + 1)
            for row in range(KEY_ROWS)
            for col in range(KEY_COLS)
        },
    }
    actual_parts = {part.name for part in object_model.parts}
    ctx.check(
        "all laptop parts are present",
        actual_parts == expected_parts,
        details=f"missing={sorted(expected_parts - actual_parts)}, extra={sorted(actual_parts - expected_parts)}",
    )

    with ctx.pose({lid_joint: 0.0}):
        ctx.expect_overlap(
            lid,
            lower_shell,
            axes="xy",
            elem_a="lid_back_cover",
            elem_b="base_body",
            min_overlap=0.20,
            name="closed lid covers the lower shell footprint",
        )
        ctx.expect_gap(
            lid,
            lower_shell,
            axis="z",
            positive_elem="lid_back_cover",
            negative_elem="rear_spine",
            max_gap=0.001,
            max_penetration=0.0,
            name="closed lid seats on the rear spine without penetrating it",
        )
        ctx.expect_within(
            sample_key,
            lower_shell,
            axes="xy",
            inner_elem="keycap",
            outer_elem="keyboard_frame",
            margin=0.002,
            name="sample key stays inside the keyboard field",
        )
        ctx.expect_gap(
            sample_key,
            lower_shell,
            axis="z",
            positive_elem="keycap",
            negative_elem="keyboard_frame",
            min_gap=0.0010,
            name="sample key sits proud of the keyboard floor at rest",
        )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_back_cover")
    with ctx.pose({lid_joint: 1.25}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_back_cover")
    ctx.check(
        "lid opens upward from the rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.10
        and open_lid_aabb[1][1] < closed_lid_aabb[1][1] - 0.02,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    closed_left_latch_aabb = ctx.part_element_world_aabb(left_latch, elem="left_latch_tab")
    closed_right_latch_aabb = ctx.part_element_world_aabb(right_latch, elem="right_latch_tab")
    with ctx.pose({left_latch_joint: 0.55, right_latch_joint: 0.55}):
        open_left_latch_aabb = ctx.part_element_world_aabb(left_latch, elem="left_latch_tab")
        open_right_latch_aabb = ctx.part_element_world_aabb(right_latch, elem="right_latch_tab")
    ctx.check(
        "side latches rotate forward on their corner pivots",
        closed_left_latch_aabb is not None
        and closed_right_latch_aabb is not None
        and open_left_latch_aabb is not None
        and open_right_latch_aabb is not None
        and open_left_latch_aabb[1][1] > closed_left_latch_aabb[1][1] + 0.006
        and open_right_latch_aabb[1][1] > closed_right_latch_aabb[1][1] + 0.006,
        details=(
            f"left_closed={closed_left_latch_aabb}, left_open={open_left_latch_aabb}, "
            f"right_closed={closed_right_latch_aabb}, right_open={open_right_latch_aabb}"
        ),
    )

    rest_key_aabb = ctx.part_element_world_aabb(sample_key, elem="keycap")
    with ctx.pose({sample_key_joint: KEY_TRAVEL}):
        ctx.expect_gap(
            sample_key,
            lower_shell,
            axis="z",
            positive_elem="keycap",
            negative_elem="keyboard_frame",
            max_gap=0.0002,
            max_penetration=0.0,
            name="sample key retains clearance above the keybed when pressed",
        )
        pressed_key_aabb = ctx.part_element_world_aabb(sample_key, elem="keycap")
    ctx.check(
        "sample key plunges downward on a short vertical stroke",
        rest_key_aabb is not None
        and pressed_key_aabb is not None
        and pressed_key_aabb[1][2] < rest_key_aabb[1][2] - 0.0008
        and pressed_key_aabb[0][2] < rest_key_aabb[0][2] - 0.0008,
        details=f"rest={rest_key_aabb}, pressed={pressed_key_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
