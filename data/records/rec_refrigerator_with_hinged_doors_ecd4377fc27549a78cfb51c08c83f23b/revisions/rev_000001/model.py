from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose, pi

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


FRIDGE_WIDTH = 0.91
FRIDGE_DEPTH = 0.73
FRIDGE_HEIGHT = 1.78

CASE_THICKNESS = 0.03
FACE_FRAME_DEPTH = 0.012
REVEAL_GAP = 0.004
TOP_REVEAL = 0.004

TOE_KICK_HEIGHT = 0.09
DIVIDER_THICKNESS = 0.03
DIVIDER_BOTTOM_Z = 0.51
DIVIDER_CENTER_Z = DIVIDER_BOTTOM_Z + DIVIDER_THICKNESS / 2.0
DIVIDER_TOP_Z = DIVIDER_BOTTOM_Z + DIVIDER_THICKNESS

UPPER_DOOR_THICKNESS = 0.065
UPPER_DOOR_WIDTH = (FRIDGE_WIDTH - 2.0 * REVEAL_GAP - 0.002) / 2.0
UPPER_DOOR_HEIGHT = FRIDGE_HEIGHT - DIVIDER_TOP_Z - REVEAL_GAP - TOP_REVEAL
UPPER_DOOR_BOTTOM_Z = DIVIDER_TOP_Z + REVEAL_GAP
UPPER_DOOR_CENTER_Z = UPPER_DOOR_BOTTOM_Z + UPPER_DOOR_HEIGHT / 2.0
HINGE_AXIS_OFFSET_Y = 0.008
DOOR_PANEL_OFFSET_Y = UPPER_DOOR_THICKNESS / 2.0 - HINGE_AXIS_OFFSET_Y

DRAWER_FRONT_THICKNESS = 0.075
DRAWER_FRONT_WIDTH = FRIDGE_WIDTH - 2.0 * REVEAL_GAP
DRAWER_FRONT_HEIGHT = DIVIDER_BOTTOM_Z - TOE_KICK_HEIGHT - 2.0 * REVEAL_GAP
DRAWER_FRONT_CENTER_Z = TOE_KICK_HEIGHT + REVEAL_GAP + DRAWER_FRONT_HEIGHT / 2.0

DRAWER_TUB_WIDTH = 0.77
DRAWER_TUB_DEPTH = 0.49
DRAWER_TUB_HEIGHT = 0.28
DRAWER_WALL_THICKNESS = 0.01
DRAWER_TUB_BOTTOM_Z = 0.11
DRAWER_TUB_CENTER_Z = DRAWER_TUB_BOTTOM_Z + DRAWER_TUB_HEIGHT / 2.0
DRAWER_TUB_CENTER_Y = FRIDGE_DEPTH / 2.0 - 0.004 - DRAWER_TUB_DEPTH / 2.0
LEFT_TUB_WALL_CENTER_X = -(DRAWER_TUB_WIDTH / 2.0 - DRAWER_WALL_THICKNESS / 2.0)
RIGHT_TUB_WALL_CENTER_X = -LEFT_TUB_WALL_CENTER_X
LEFT_TUB_WALL_OUTER_FACE_X = LEFT_TUB_WALL_CENTER_X - DRAWER_WALL_THICKNESS / 2.0
RIGHT_TUB_WALL_OUTER_FACE_X = RIGHT_TUB_WALL_CENTER_X + DRAWER_WALL_THICKNESS / 2.0
DRAWER_FRONT_WALL_THICKNESS = 0.018
DRAWER_FRONT_WALL_HEIGHT = DRAWER_TUB_HEIGHT - DRAWER_WALL_THICKNESS / 2.0
DRAWER_FRONT_WALL_CENTER_Z = DRAWER_TUB_BOTTOM_Z + DRAWER_FRONT_WALL_HEIGHT / 2.0
DRAWER_BRACKET_OVERLAP = 0.002
DRAWER_BRACKET_HEIGHT = 0.09

RAIL_LENGTH = 0.48
RAIL_WIDTH = 0.024
RAIL_HEIGHT = 0.05
RAIL_BACK_THICKNESS = 0.004
RAIL_FLANGE_THICKNESS = 0.004
RAIL_CENTER_Y = DRAWER_TUB_CENTER_Y
RAIL_CENTER_Z = DRAWER_TUB_BOTTOM_Z + 0.11
LEFT_RAIL_CENTER_X = -(FRIDGE_WIDTH / 2.0 - CASE_THICKNESS) + RAIL_WIDTH / 2.0
RIGHT_RAIL_CENTER_X = -LEFT_RAIL_CENTER_X
DRAWER_BRACKET_CENTER_Z = RAIL_CENTER_Z

RUNNER_WIDTH = 0.016
RUNNER_HEIGHT = RAIL_HEIGHT - 2.0 * RAIL_FLANGE_THICKNESS
RUNNER_LENGTH = 0.42
LEFT_RUNNER_CENTER_X = LEFT_RAIL_CENTER_X + 0.002
RIGHT_RUNNER_CENTER_X = RIGHT_RAIL_CENTER_X - 0.002
LEFT_RUNNER_INNER_FACE_X = LEFT_RUNNER_CENTER_X + RUNNER_WIDTH / 2.0
RIGHT_RUNNER_INNER_FACE_X = RIGHT_RUNNER_CENTER_X - RUNNER_WIDTH / 2.0
DRAWER_BRACKET_WIDTH = (
    LEFT_TUB_WALL_OUTER_FACE_X
    - LEFT_RUNNER_INNER_FACE_X
    + 2.0 * DRAWER_BRACKET_OVERLAP
)
LEFT_BRACKET_CENTER_X = (LEFT_TUB_WALL_OUTER_FACE_X + LEFT_RUNNER_INNER_FACE_X) / 2.0
RIGHT_BRACKET_CENTER_X = (RIGHT_TUB_WALL_OUTER_FACE_X + RIGHT_RUNNER_INNER_FACE_X) / 2.0

DRAWER_TRAVEL = 0.36


def _box(part, size, center, material, *, name=None):
    return part.visual(
        Box(size),
        origin=Origin(xyz=center),
        material=material,
        name=name,
    )


def _cylinder(part, radius, length, center, material, *, rpy=(0.0, 0.0, 0.0), name=None):
    return part.visual(
        Cylinder(radius, length),
        origin=Origin(xyz=center, rpy=rpy),
        material=material,
        name=name,
    )


def _door_handle(part, *, door_width, x_sign, panel_offset_y, door_height, material):
    front_face_y = panel_offset_y + UPPER_DOOR_THICKNESS / 2.0
    bar_center_y = front_face_y + 0.034
    handle_x = x_sign * (door_width / 2.0 - 0.055)
    if x_sign > 0:
        handle_x = door_width - 0.055
    else:
        handle_x = -door_width + 0.055

    _cylinder(
        part,
        0.011,
        0.82,
        (handle_x, bar_center_y, door_height / 2.0),
        material,
        name="handle_bar",
    )
    for mount_z in (door_height / 2.0 - 0.29, door_height / 2.0 + 0.29):
        _cylinder(
            part,
            0.008,
            0.024,
            (handle_x, front_face_y + 0.012, mount_z),
            material,
            rpy=(-pi / 2.0, 0.0, 0.0),
        )


def _drawer_handle(part, *, material):
    front_face_y = FRIDGE_DEPTH / 2.0 + DRAWER_FRONT_THICKNESS
    bar_center_y = FRIDGE_DEPTH / 2.0 + 0.033
    handle_z = DRAWER_FRONT_CENTER_Z + DRAWER_FRONT_HEIGHT / 2.0 - 0.08
    _cylinder(
        part,
        0.012,
        0.72,
        (0.0, bar_center_y, handle_z),
        material,
        rpy=(0.0, pi / 2.0, 0.0),
        name="handle_bar",
    )
    for mount_x in (-0.24, 0.24):
        _cylinder(
            part,
            0.008,
            0.024,
            (mount_x, FRIDGE_DEPTH / 2.0 + 0.012, handle_z),
            material,
            rpy=(-pi / 2.0, 0.0, 0.0),
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="french_door_refrigerator")

    stainless = model.material("stainless", rgba=(0.76, 0.78, 0.80, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.20, 0.22, 0.24, 1.0))
    inner_white = model.material("inner_white", rgba=(0.94, 0.95, 0.96, 1.0))
    rail_gray = model.material("rail_gray", rgba=(0.58, 0.60, 0.64, 1.0))
    shelf_glass = model.material("shelf_glass", rgba=(0.78, 0.88, 0.98, 0.45))

    cabinet = model.part("cabinet")
    inner_width = FRIDGE_WIDTH - 2.0 * CASE_THICKNESS
    inner_depth = FRIDGE_DEPTH - CASE_THICKNESS

    _box(
        cabinet,
        (CASE_THICKNESS, FRIDGE_DEPTH, FRIDGE_HEIGHT),
        (-(FRIDGE_WIDTH / 2.0 - CASE_THICKNESS / 2.0), 0.0, FRIDGE_HEIGHT / 2.0),
        stainless,
        name="left_wall",
    )
    _box(
        cabinet,
        (CASE_THICKNESS, FRIDGE_DEPTH, FRIDGE_HEIGHT),
        ((FRIDGE_WIDTH / 2.0 - CASE_THICKNESS / 2.0), 0.0, FRIDGE_HEIGHT / 2.0),
        stainless,
        name="right_wall",
    )
    _box(
        cabinet,
        (FRIDGE_WIDTH, FRIDGE_DEPTH, CASE_THICKNESS),
        (0.0, 0.0, CASE_THICKNESS / 2.0),
        stainless,
        name="bottom_panel",
    )
    _box(
        cabinet,
        (FRIDGE_WIDTH, FRIDGE_DEPTH, CASE_THICKNESS),
        (0.0, 0.0, FRIDGE_HEIGHT - CASE_THICKNESS / 2.0),
        stainless,
        name="top_panel",
    )
    _box(
        cabinet,
        (inner_width, CASE_THICKNESS, FRIDGE_HEIGHT),
        (0.0, -(FRIDGE_DEPTH / 2.0 - CASE_THICKNESS / 2.0), FRIDGE_HEIGHT / 2.0),
        inner_white,
        name="back_panel",
    )
    _box(
        cabinet,
        (inner_width, inner_depth, DIVIDER_THICKNESS),
        (0.0, -CASE_THICKNESS / 2.0, DIVIDER_CENTER_Z),
        inner_white,
        name="compartment_divider",
    )

    _box(
        cabinet,
        (CASE_THICKNESS, FACE_FRAME_DEPTH, FRIDGE_HEIGHT - TOE_KICK_HEIGHT),
        (
            -(FRIDGE_WIDTH / 2.0 - CASE_THICKNESS / 2.0),
            FRIDGE_DEPTH / 2.0 - FACE_FRAME_DEPTH / 2.0,
            TOE_KICK_HEIGHT + (FRIDGE_HEIGHT - TOE_KICK_HEIGHT) / 2.0,
        ),
        dark_trim,
        name="left_face_frame",
    )
    _box(
        cabinet,
        (CASE_THICKNESS, FACE_FRAME_DEPTH, FRIDGE_HEIGHT - TOE_KICK_HEIGHT),
        (
            FRIDGE_WIDTH / 2.0 - CASE_THICKNESS / 2.0,
            FRIDGE_DEPTH / 2.0 - FACE_FRAME_DEPTH / 2.0,
            TOE_KICK_HEIGHT + (FRIDGE_HEIGHT - TOE_KICK_HEIGHT) / 2.0,
        ),
        dark_trim,
        name="right_face_frame",
    )
    _box(
        cabinet,
        (inner_width, FACE_FRAME_DEPTH, CASE_THICKNESS),
        (0.0, FRIDGE_DEPTH / 2.0 - FACE_FRAME_DEPTH / 2.0, FRIDGE_HEIGHT - CASE_THICKNESS / 2.0),
        dark_trim,
        name="top_face_frame",
    )
    _box(
        cabinet,
        (inner_width, FACE_FRAME_DEPTH, DIVIDER_THICKNESS),
        (0.0, FRIDGE_DEPTH / 2.0 - FACE_FRAME_DEPTH / 2.0, DIVIDER_CENTER_Z),
        dark_trim,
        name="divider_face_frame",
    )
    _box(
        cabinet,
        (inner_width, 0.05, TOE_KICK_HEIGHT),
        (0.0, FRIDGE_DEPTH / 2.0 - 0.025, TOE_KICK_HEIGHT / 2.0),
        dark_trim,
        name="toe_kick_panel",
    )

    for shelf_name, shelf_z in (("upper_shelf", 0.91), ("mid_shelf", 1.26)):
        _box(
            cabinet,
            (inner_width, 0.55, 0.007),
            (0.0, -0.02, shelf_z),
            shelf_glass,
            name=shelf_name,
        )

    left_door = model.part("left_upper_door")
    _box(
        left_door,
        (UPPER_DOOR_WIDTH, UPPER_DOOR_THICKNESS, UPPER_DOOR_HEIGHT),
        (UPPER_DOOR_WIDTH / 2.0, DOOR_PANEL_OFFSET_Y, UPPER_DOOR_HEIGHT / 2.0),
        stainless,
        name="door_panel",
    )
    _door_handle(
        left_door,
        door_width=UPPER_DOOR_WIDTH,
        x_sign=1.0,
        panel_offset_y=DOOR_PANEL_OFFSET_Y,
        door_height=UPPER_DOOR_HEIGHT,
        material=dark_trim,
    )

    right_door = model.part("right_upper_door")
    _box(
        right_door,
        (UPPER_DOOR_WIDTH, UPPER_DOOR_THICKNESS, UPPER_DOOR_HEIGHT),
        (-UPPER_DOOR_WIDTH / 2.0, DOOR_PANEL_OFFSET_Y, UPPER_DOOR_HEIGHT / 2.0),
        stainless,
        name="door_panel",
    )
    _door_handle(
        right_door,
        door_width=UPPER_DOOR_WIDTH,
        x_sign=-1.0,
        panel_offset_y=DOOR_PANEL_OFFSET_Y,
        door_height=UPPER_DOOR_HEIGHT,
        material=dark_trim,
    )

    left_rail = model.part("left_outer_rail")
    _box(
        left_rail,
        (RAIL_BACK_THICKNESS, RAIL_LENGTH, RAIL_HEIGHT),
        (-(RAIL_WIDTH / 2.0 - RAIL_BACK_THICKNESS / 2.0), 0.0, 0.0),
        rail_gray,
        name="back_plate",
    )
    _box(
        left_rail,
        (RAIL_WIDTH, RAIL_LENGTH, RAIL_FLANGE_THICKNESS),
        (0.0, 0.0, RAIL_HEIGHT / 2.0 - RAIL_FLANGE_THICKNESS / 2.0),
        rail_gray,
        name="top_flange",
    )
    _box(
        left_rail,
        (RAIL_WIDTH, RAIL_LENGTH, RAIL_FLANGE_THICKNESS),
        (0.0, 0.0, -(RAIL_HEIGHT / 2.0 - RAIL_FLANGE_THICKNESS / 2.0)),
        rail_gray,
        name="bottom_flange",
    )

    right_rail = model.part("right_outer_rail")
    _box(
        right_rail,
        (RAIL_BACK_THICKNESS, RAIL_LENGTH, RAIL_HEIGHT),
        (RAIL_WIDTH / 2.0 - RAIL_BACK_THICKNESS / 2.0, 0.0, 0.0),
        rail_gray,
        name="back_plate",
    )
    _box(
        right_rail,
        (RAIL_WIDTH, RAIL_LENGTH, RAIL_FLANGE_THICKNESS),
        (0.0, 0.0, RAIL_HEIGHT / 2.0 - RAIL_FLANGE_THICKNESS / 2.0),
        rail_gray,
        name="top_flange",
    )
    _box(
        right_rail,
        (RAIL_WIDTH, RAIL_LENGTH, RAIL_FLANGE_THICKNESS),
        (0.0, 0.0, -(RAIL_HEIGHT / 2.0 - RAIL_FLANGE_THICKNESS / 2.0)),
        rail_gray,
        name="bottom_flange",
    )

    drawer = model.part("freezer_drawer")
    _box(
        drawer,
        (DRAWER_FRONT_WIDTH, DRAWER_FRONT_THICKNESS, DRAWER_FRONT_HEIGHT),
        (0.0, FRIDGE_DEPTH / 2.0 + DRAWER_FRONT_THICKNESS / 2.0, DRAWER_FRONT_CENTER_Z),
        stainless,
        name="drawer_front",
    )
    _box(
        drawer,
        (DRAWER_TUB_WIDTH, DRAWER_WALL_THICKNESS, DRAWER_TUB_HEIGHT),
        (0.0, DRAWER_TUB_CENTER_Y - DRAWER_TUB_DEPTH / 2.0 + DRAWER_WALL_THICKNESS / 2.0, DRAWER_TUB_CENTER_Z),
        inner_white,
        name="tub_back",
    )
    _box(
        drawer,
        (DRAWER_WALL_THICKNESS, DRAWER_TUB_DEPTH, DRAWER_TUB_HEIGHT),
        (-(DRAWER_TUB_WIDTH / 2.0 - DRAWER_WALL_THICKNESS / 2.0), DRAWER_TUB_CENTER_Y, DRAWER_TUB_CENTER_Z),
        inner_white,
        name="tub_left_wall",
    )
    _box(
        drawer,
        (DRAWER_WALL_THICKNESS, DRAWER_TUB_DEPTH, DRAWER_TUB_HEIGHT),
        (DRAWER_TUB_WIDTH / 2.0 - DRAWER_WALL_THICKNESS / 2.0, DRAWER_TUB_CENTER_Y, DRAWER_TUB_CENTER_Z),
        inner_white,
        name="tub_right_wall",
    )
    _box(
        drawer,
        (DRAWER_TUB_WIDTH, DRAWER_TUB_DEPTH, DRAWER_WALL_THICKNESS),
        (0.0, DRAWER_TUB_CENTER_Y, DRAWER_TUB_BOTTOM_Z + DRAWER_WALL_THICKNESS / 2.0),
        inner_white,
        name="tub_floor",
    )
    _box(
        drawer,
        (DRAWER_TUB_WIDTH - 2.0 * DRAWER_WALL_THICKNESS, DRAWER_FRONT_WALL_THICKNESS, DRAWER_FRONT_WALL_HEIGHT),
        (
            0.0,
            DRAWER_TUB_CENTER_Y + DRAWER_TUB_DEPTH / 2.0 - DRAWER_FRONT_WALL_THICKNESS / 2.0 + REVEAL_GAP,
            DRAWER_FRONT_WALL_CENTER_Z,
        ),
        inner_white,
        name="tub_front_wall",
    )
    _box(
        drawer,
        (DRAWER_BRACKET_WIDTH, RUNNER_LENGTH, DRAWER_BRACKET_HEIGHT),
        (
            (LEFT_RUNNER_CENTER_X - RUNNER_WIDTH / 2.0 + (-(DRAWER_TUB_WIDTH / 2.0 - DRAWER_WALL_THICKNESS / 2.0))) / 2.0,
            RAIL_CENTER_Y,
            DRAWER_BRACKET_CENTER_Z,
        ),
        inner_white,
        name="left_runner_bracket",
    )
    _box(
        drawer,
        (DRAWER_BRACKET_WIDTH, RUNNER_LENGTH, DRAWER_BRACKET_HEIGHT),
        (
            (RIGHT_RUNNER_CENTER_X + RUNNER_WIDTH / 2.0 + (DRAWER_TUB_WIDTH / 2.0 - DRAWER_WALL_THICKNESS / 2.0)) / 2.0,
            RAIL_CENTER_Y,
            DRAWER_BRACKET_CENTER_Z,
        ),
        inner_white,
        name="right_runner_bracket",
    )
    _box(
        drawer,
        (RUNNER_WIDTH, RUNNER_LENGTH, RUNNER_HEIGHT),
        (LEFT_RUNNER_CENTER_X, RAIL_CENTER_Y, RAIL_CENTER_Z),
        rail_gray,
        name="left_runner",
    )
    _box(
        drawer,
        (RUNNER_WIDTH, RUNNER_LENGTH, RUNNER_HEIGHT),
        (RIGHT_RUNNER_CENTER_X, RAIL_CENTER_Y, RAIL_CENTER_Z),
        rail_gray,
        name="right_runner",
    )
    _drawer_handle(drawer, material=dark_trim)

    model.articulation(
        "left_door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=left_door,
        origin=Origin(
            xyz=(
                -(FRIDGE_WIDTH / 2.0 - REVEAL_GAP),
                FRIDGE_DEPTH / 2.0 + HINGE_AXIS_OFFSET_Y,
                UPPER_DOOR_BOTTOM_Z,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.5,
            lower=0.0,
            upper=1.6,
        ),
    )
    model.articulation(
        "right_door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=right_door,
        origin=Origin(
            xyz=(
                FRIDGE_WIDTH / 2.0 - REVEAL_GAP,
                FRIDGE_DEPTH / 2.0 + HINGE_AXIS_OFFSET_Y,
                UPPER_DOOR_BOTTOM_Z,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.5,
            lower=-1.6,
            upper=0.0,
        ),
    )
    model.articulation(
        "cabinet_to_left_rail",
        ArticulationType.FIXED,
        parent=cabinet,
        child=left_rail,
        origin=Origin(xyz=(LEFT_RAIL_CENTER_X, RAIL_CENTER_Y, RAIL_CENTER_Z)),
    )
    model.articulation(
        "cabinet_to_right_rail",
        ArticulationType.FIXED,
        parent=cabinet,
        child=right_rail,
        origin=Origin(xyz=(RIGHT_RAIL_CENTER_X, RAIL_CENTER_Y, RAIL_CENTER_Z)),
    )
    model.articulation(
        "freezer_drawer_slide",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=drawer,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.35,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    left_door = object_model.get_part("left_upper_door")
    right_door = object_model.get_part("right_upper_door")
    left_rail = object_model.get_part("left_outer_rail")
    right_rail = object_model.get_part("right_outer_rail")
    drawer = object_model.get_part("freezer_drawer")

    left_hinge = object_model.get_articulation("left_door_hinge")
    right_hinge = object_model.get_articulation("right_door_hinge")
    drawer_slide = object_model.get_articulation("freezer_drawer_slide")

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

    expected_parts = {
        "cabinet",
        "left_upper_door",
        "right_upper_door",
        "left_outer_rail",
        "right_outer_rail",
        "freezer_drawer",
    }
    ctx.check(
        "all refrigerator parts present",
        {part.name for part in object_model.parts} == expected_parts,
        f"expected parts {sorted(expected_parts)}, got {sorted(part.name for part in object_model.parts)}",
    )
    ctx.check(
        "door hinge axes are vertical",
        left_hinge.axis == (0.0, 0.0, 1.0) and right_hinge.axis == (0.0, 0.0, 1.0),
        f"left axis={left_hinge.axis}, right axis={right_hinge.axis}",
    )
    ctx.check(
        "drawer slide axis is forward",
        drawer_slide.axis == (0.0, 1.0, 0.0),
        f"drawer axis={drawer_slide.axis}",
    )
    ctx.check(
        "door limits open outward from the center seam",
        left_hinge.motion_limits is not None
        and right_hinge.motion_limits is not None
        and isclose(left_hinge.motion_limits.lower or 0.0, 0.0, abs_tol=1e-9)
        and isclose(right_hinge.motion_limits.upper or 0.0, 0.0, abs_tol=1e-9)
        and (left_hinge.motion_limits.upper or 0.0) > 1.4
        and (right_hinge.motion_limits.lower or 0.0) < -1.4,
        "unexpected hinge limits",
    )
    ctx.check(
        "drawer travel is realistic",
        drawer_slide.motion_limits is not None
        and isclose(drawer_slide.motion_limits.lower or 0.0, 0.0, abs_tol=1e-9)
        and isclose(drawer_slide.motion_limits.upper or 0.0, DRAWER_TRAVEL, abs_tol=1e-9),
        "drawer travel limit changed",
    )

    ctx.expect_contact(left_door, cabinet, name="left door seals against cabinet in closed pose")
    ctx.expect_contact(right_door, cabinet, name="right door seals against cabinet in closed pose")
    ctx.expect_contact(left_rail, cabinet, name="left rail is mounted to cabinet")
    ctx.expect_contact(right_rail, cabinet, name="right rail is mounted to cabinet")
    ctx.expect_contact(drawer, cabinet, name="drawer seats against cabinet in closed pose")
    ctx.expect_contact(drawer, left_rail, name="drawer engages left runner rail")
    ctx.expect_contact(drawer, right_rail, name="drawer engages right runner rail")
    ctx.expect_gap(
        right_door,
        left_door,
        axis="x",
        min_gap=0.001,
        max_gap=0.004,
        name="upper doors meet with a narrow center reveal",
    )

    with ctx.pose({drawer_slide: 0.30}):
        ctx.expect_origin_gap(
            drawer,
            cabinet,
            axis="y",
            min_gap=0.29,
            max_gap=0.31,
            name="freezer drawer pulls forward on its slide axis",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
