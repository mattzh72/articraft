from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


WIDTH = 0.92
HEIGHT = 1.82
CASE_BACK_Y = -0.38
CASE_FRONT_Y = 0.355
CASE_DEPTH = CASE_FRONT_Y - CASE_BACK_Y
DOOR_THICKNESS = 0.045
HINGE_AXIS_Y = CASE_FRONT_Y + 0.008
SIDE_PANEL_THICKNESS = 0.032
TOP_PANEL_THICKNESS = 0.032
BACK_PANEL_THICKNESS = 0.022
TOE_KICK_HEIGHT = 0.095
SIDE_REVEAL = 0.005
CENTER_GAP = 0.006
ROW_GAP = 0.012
LOWER_DOOR_BOTTOM = 0.105
LOWER_DOOR_HEIGHT = 0.515
UPPER_DOOR_BOTTOM = LOWER_DOOR_BOTTOM + LOWER_DOOR_HEIGHT + ROW_GAP
UPPER_DOOR_HEIGHT = 1.165
DOOR_WIDTH = (WIDTH - (2.0 * SIDE_REVEAL) - CENTER_GAP) / 2.0
HINGE_RADIUS = 0.008
HANDLE_RADIUS = 0.010
HANDLE_POST_RADIUS = 0.006

LEFT_CLOSED_OUTER = -(WIDTH / 2.0) + SIDE_REVEAL
LEFT_CLOSED_INNER = -(CENTER_GAP / 2.0)
RIGHT_CLOSED_INNER = CENTER_GAP / 2.0
RIGHT_CLOSED_OUTER = (WIDTH / 2.0) - SIDE_REVEAL
LEFT_HINGE_X = -(WIDTH / 2.0) - 0.006
RIGHT_HINGE_X = (WIDTH / 2.0) + 0.006
DOOR_PANEL_CENTER_Y_REL = (CASE_FRONT_Y + (DOOR_THICKNESS / 2.0)) - HINGE_AXIS_Y


def _case_hinge_segments(door_height: float) -> list[tuple[str, float, float]]:
    if door_height > 0.8:
        return [
            ("top_outer", door_height / 2.0 - 0.045, 0.040),
            ("top_inner", door_height / 2.0 - 0.135, 0.040),
            ("bottom_inner", -(door_height / 2.0) + 0.135, 0.040),
            ("bottom_outer", -(door_height / 2.0) + 0.045, 0.040),
        ]
    return [
        ("top_outer", 0.225, 0.040),
        ("top_inner", 0.125, 0.040),
        ("bottom_inner", -0.125, 0.040),
        ("bottom_outer", -0.225, 0.040),
    ]


def _door_hinge_segments(door_height: float) -> list[tuple[str, float, float]]:
    if door_height > 0.8:
        return [
            ("top", door_height / 2.0 - 0.090, 0.050),
            ("bottom", -(door_height / 2.0) + 0.090, 0.050),
        ]
    return [
        ("top", 0.175, 0.060),
        ("bottom", -0.175, 0.060),
    ]


def _add_case_hinge(
    cabinet,
    *,
    prefix: str,
    hinge_x: float,
    z_center: float,
    door_height: float,
    finish,
):
    mount_sign = 1.0 if hinge_x < 0.0 else -1.0
    for segment_name, z_offset, segment_length in _case_hinge_segments(door_height):
        cabinet.visual(
            Cylinder(radius=HINGE_RADIUS, length=segment_length),
            origin=Origin(xyz=(hinge_x, HINGE_AXIS_Y, z_center + z_offset)),
            material=finish,
            name=f"{prefix}_case_barrel_{segment_name}",
        )
        cabinet.visual(
            Box((0.022, 0.024, segment_length)),
            origin=Origin(
                xyz=(
                    hinge_x + (mount_sign * 0.011),
                    CASE_FRONT_Y - 0.012,
                    z_center + z_offset,
                )
            ),
            material=finish,
            name=f"{prefix}_case_mount_{segment_name}",
        )


def _build_door(
    model: ArticulatedObject,
    *,
    name: str,
    hinge_x: float,
    closed_outer_x: float,
    closed_inner_x: float,
    door_bottom: float,
    door_height: float,
    panel_finish,
    handle_finish,
    hinge_finish,
) -> None:
    door = model.part(name)
    z_center = door_bottom + (door_height / 2.0)
    panel_center_x_rel = ((closed_outer_x + closed_inner_x) / 2.0) - hinge_x
    free_edge_rel = closed_inner_x - hinge_x if hinge_x > 0.0 else closed_inner_x - hinge_x
    if hinge_x < 0.0:
        free_edge_rel = closed_inner_x - hinge_x
    else:
        free_edge_rel = closed_inner_x - hinge_x
    swing_sign = 1.0 if free_edge_rel > 0.0 else -1.0

    door.visual(
        Box((DOOR_WIDTH, DOOR_THICKNESS, door_height)),
        origin=Origin(xyz=(panel_center_x_rel, DOOR_PANEL_CENTER_Y_REL, 0.0)),
        material=panel_finish,
        name="outer_panel",
    )
    door.visual(
        Box((DOOR_WIDTH - 0.040, 0.006, door_height - 0.070)),
        origin=Origin(xyz=(panel_center_x_rel, DOOR_PANEL_CENTER_Y_REL - 0.016, 0.0)),
        material=panel_finish,
        name="inner_skin",
    )

    handle_length = min(door_height * 0.72, 0.88 if door_height > 0.8 else 0.34)
    handle_center_z = 0.08 if door_height > 0.8 else 0.02
    handle_x = free_edge_rel - (swing_sign * 0.060)
    handle_grip_y = DOOR_PANEL_CENTER_Y_REL + (DOOR_THICKNESS / 2.0) + 0.020
    handle_post_y = DOOR_PANEL_CENTER_Y_REL + (DOOR_THICKNESS / 2.0) + 0.010

    door.visual(
        Cylinder(radius=HANDLE_RADIUS, length=handle_length),
        origin=Origin(xyz=(handle_x, handle_grip_y, handle_center_z)),
        material=handle_finish,
        name="handle_grip",
    )
    for index, post_z in enumerate(
        (
            handle_center_z - (handle_length / 2.0) + 0.070,
            handle_center_z + (handle_length / 2.0) - 0.070,
        )
    ):
        door.visual(
            Cylinder(radius=HANDLE_POST_RADIUS, length=0.020),
            origin=Origin(
                xyz=(handle_x, handle_post_y, post_z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=handle_finish,
            name=f"handle_post_{index}",
        )

    for segment_name, z_offset, segment_length in _door_hinge_segments(door_height):
        door.visual(
            Cylinder(radius=HINGE_RADIUS, length=segment_length),
            origin=Origin(xyz=(0.0, 0.0, z_offset)),
            material=hinge_finish,
            name=f"hinge_barrel_{segment_name}",
        )
        door.visual(
            Box((0.020, 0.016, segment_length)),
            origin=Origin(
                xyz=(swing_sign * 0.010, 0.004, z_offset),
            ),
            material=hinge_finish,
            name=f"hinge_leaf_{segment_name}",
        )

    door.inertial = Inertial.from_geometry(
        Box((DOOR_WIDTH, DOOR_THICKNESS, door_height)),
        mass=24.0 if door_height > 0.8 else 15.0,
        origin=Origin(xyz=(panel_center_x_rel, DOOR_PANEL_CENTER_Y_REL, 0.0)),
    )

    lower_limit, upper_limit = (0.0, 2.05) if hinge_x < 0.0 else (-2.05, 0.0)
    model.articulation(
        f"{name}_hinge",
        ArticulationType.REVOLUTE,
        parent="cabinet",
        child=door,
        origin=Origin(xyz=(hinge_x, HINGE_AXIS_Y, z_center)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=1.2,
            lower=lower_limit,
            upper=upper_limit,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="french_door_refrigerator")

    stainless = model.material("stainless", rgba=(0.76, 0.78, 0.80, 1.0))
    brushed_handle = model.material("brushed_handle", rgba=(0.58, 0.61, 0.64, 1.0))
    hinge_gray = model.material("hinge_gray", rgba=(0.40, 0.43, 0.46, 1.0))
    liner_white = model.material("liner_white", rgba=(0.93, 0.94, 0.95, 1.0))
    toe_kick = model.material("toe_kick", rgba=(0.10, 0.11, 0.12, 1.0))
    rear_panel = model.material("rear_panel", rgba=(0.24, 0.26, 0.28, 1.0))

    cabinet = model.part("cabinet")
    case_center_y = (CASE_BACK_Y + CASE_FRONT_Y) / 2.0

    cabinet.visual(
        Box((SIDE_PANEL_THICKNESS, CASE_DEPTH, HEIGHT)),
        origin=Origin(
            xyz=(
                -(WIDTH / 2.0) + (SIDE_PANEL_THICKNESS / 2.0),
                case_center_y,
                HEIGHT / 2.0,
            )
        ),
        material=stainless,
        name="left_side_panel",
    )
    cabinet.visual(
        Box((SIDE_PANEL_THICKNESS, CASE_DEPTH, HEIGHT)),
        origin=Origin(
            xyz=(
                (WIDTH / 2.0) - (SIDE_PANEL_THICKNESS / 2.0),
                case_center_y,
                HEIGHT / 2.0,
            )
        ),
        material=stainless,
        name="right_side_panel",
    )
    cabinet.visual(
        Box((WIDTH - (2.0 * SIDE_PANEL_THICKNESS), CASE_DEPTH, TOP_PANEL_THICKNESS)),
        origin=Origin(
            xyz=(0.0, case_center_y, HEIGHT - (TOP_PANEL_THICKNESS / 2.0))
        ),
        material=stainless,
        name="top_panel",
    )
    cabinet.visual(
        Box((WIDTH - (2.0 * SIDE_PANEL_THICKNESS), CASE_DEPTH - 0.140, 0.040)),
        origin=Origin(xyz=(0.0, -0.020, 0.020)),
        material=liner_white,
        name="base_deck",
    )
    cabinet.visual(
        Box((WIDTH - (2.0 * SIDE_PANEL_THICKNESS), BACK_PANEL_THICKNESS, HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                CASE_BACK_Y + (BACK_PANEL_THICKNESS / 2.0),
                HEIGHT / 2.0,
            )
        ),
        material=rear_panel,
        name="rear_skin",
    )
    cabinet.visual(
        Box((WIDTH - (2.0 * SIDE_PANEL_THICKNESS), 0.600, 0.028)),
        origin=Origin(xyz=(0.0, -0.060, UPPER_DOOR_BOTTOM - 0.018)),
        material=liner_white,
        name="crisper_deck",
    )
    cabinet.visual(
        Box((WIDTH - (2.0 * SIDE_PANEL_THICKNESS), 0.600, 0.030)),
        origin=Origin(
            xyz=(0.0, -0.040, LOWER_DOOR_BOTTOM + LOWER_DOOR_HEIGHT + 0.006)
        ),
        material=liner_white,
        name="pantry_ceiling",
    )
    cabinet.visual(
        Box((WIDTH - (2.0 * SIDE_PANEL_THICKNESS), 0.580, 0.010)),
        origin=Origin(xyz=(0.0, -0.040, 0.940)),
        material=liner_white,
        name="upper_shelf_low",
    )
    cabinet.visual(
        Box((WIDTH - (2.0 * SIDE_PANEL_THICKNESS), 0.560, 0.010)),
        origin=Origin(xyz=(0.0, -0.050, 1.250)),
        material=liner_white,
        name="upper_shelf_high",
    )
    cabinet.visual(
        Box((0.024, 0.100, HEIGHT - TOE_KICK_HEIGHT - 0.030)),
        origin=Origin(
            xyz=(
                0.0,
                CASE_FRONT_Y - 0.050,
                TOE_KICK_HEIGHT + ((HEIGHT - TOE_KICK_HEIGHT - 0.030) / 2.0),
            )
        ),
        material=liner_white,
        name="center_mullion",
    )
    cabinet.visual(
        Box((WIDTH - 0.040, 0.050, TOE_KICK_HEIGHT)),
        origin=Origin(xyz=(0.0, CASE_FRONT_Y - 0.085, TOE_KICK_HEIGHT / 2.0)),
        material=toe_kick,
        name="toe_kick_panel",
    )

    _add_case_hinge(
        cabinet,
        prefix="upper_left",
        hinge_x=LEFT_HINGE_X,
        z_center=UPPER_DOOR_BOTTOM + (UPPER_DOOR_HEIGHT / 2.0),
        door_height=UPPER_DOOR_HEIGHT,
        finish=hinge_gray,
    )
    _add_case_hinge(
        cabinet,
        prefix="upper_right",
        hinge_x=RIGHT_HINGE_X,
        z_center=UPPER_DOOR_BOTTOM + (UPPER_DOOR_HEIGHT / 2.0),
        door_height=UPPER_DOOR_HEIGHT,
        finish=hinge_gray,
    )
    _add_case_hinge(
        cabinet,
        prefix="lower_left",
        hinge_x=LEFT_HINGE_X,
        z_center=LOWER_DOOR_BOTTOM + (LOWER_DOOR_HEIGHT / 2.0),
        door_height=LOWER_DOOR_HEIGHT,
        finish=hinge_gray,
    )
    _add_case_hinge(
        cabinet,
        prefix="lower_right",
        hinge_x=RIGHT_HINGE_X,
        z_center=LOWER_DOOR_BOTTOM + (LOWER_DOOR_HEIGHT / 2.0),
        door_height=LOWER_DOOR_HEIGHT,
        finish=hinge_gray,
    )

    cabinet.inertial = Inertial.from_geometry(
        Box((WIDTH, CASE_DEPTH, HEIGHT)),
        mass=138.0,
        origin=Origin(xyz=(0.0, case_center_y, HEIGHT / 2.0)),
    )

    _build_door(
        model,
        name="upper_left_door",
        hinge_x=LEFT_HINGE_X,
        closed_outer_x=LEFT_CLOSED_OUTER,
        closed_inner_x=LEFT_CLOSED_INNER,
        door_bottom=UPPER_DOOR_BOTTOM,
        door_height=UPPER_DOOR_HEIGHT,
        panel_finish=stainless,
        handle_finish=brushed_handle,
        hinge_finish=hinge_gray,
    )
    _build_door(
        model,
        name="upper_right_door",
        hinge_x=RIGHT_HINGE_X,
        closed_outer_x=RIGHT_CLOSED_OUTER,
        closed_inner_x=RIGHT_CLOSED_INNER,
        door_bottom=UPPER_DOOR_BOTTOM,
        door_height=UPPER_DOOR_HEIGHT,
        panel_finish=stainless,
        handle_finish=brushed_handle,
        hinge_finish=hinge_gray,
    )
    _build_door(
        model,
        name="lower_left_door",
        hinge_x=LEFT_HINGE_X,
        closed_outer_x=LEFT_CLOSED_OUTER,
        closed_inner_x=LEFT_CLOSED_INNER,
        door_bottom=LOWER_DOOR_BOTTOM,
        door_height=LOWER_DOOR_HEIGHT,
        panel_finish=stainless,
        handle_finish=brushed_handle,
        hinge_finish=hinge_gray,
    )
    _build_door(
        model,
        name="lower_right_door",
        hinge_x=RIGHT_HINGE_X,
        closed_outer_x=RIGHT_CLOSED_OUTER,
        closed_inner_x=RIGHT_CLOSED_INNER,
        door_bottom=LOWER_DOOR_BOTTOM,
        door_height=LOWER_DOOR_HEIGHT,
        panel_finish=stainless,
        handle_finish=brushed_handle,
        hinge_finish=hinge_gray,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    upper_left = object_model.get_part("upper_left_door")
    upper_right = object_model.get_part("upper_right_door")
    lower_left = object_model.get_part("lower_left_door")
    lower_right = object_model.get_part("lower_right_door")
    upper_left_hinge = object_model.get_articulation("upper_left_door_hinge")
    upper_right_hinge = object_model.get_articulation("upper_right_door_hinge")
    lower_left_hinge = object_model.get_articulation("lower_left_door_hinge")
    lower_right_hinge = object_model.get_articulation("lower_right_door_hinge")

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

    for hinge_name, hinge, expects_positive in (
        ("upper_left_hinge_axis", upper_left_hinge, True),
        ("upper_right_hinge_axis", upper_right_hinge, False),
        ("lower_left_hinge_axis", lower_left_hinge, True),
        ("lower_right_hinge_axis", lower_right_hinge, False),
    ):
        ctx.check(
            hinge_name,
            hinge.axis == (0.0, 0.0, 1.0),
            f"expected vertical hinge axis, got {hinge.axis}",
        )
        limits = hinge.motion_limits
        ctx.check(
            f"{hinge_name}_limits",
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and (
                (expects_positive and limits.lower >= 0.0 and limits.upper > 0.0)
                or (not expects_positive and limits.lower < 0.0 and limits.upper <= 0.0)
            ),
            f"unexpected hinge limits: lower={None if limits is None else limits.lower}, "
            f"upper={None if limits is None else limits.upper}",
        )

    ctx.expect_origin_distance(
        upper_left,
        upper_right,
        axes="x",
        min_dist=0.930,
        max_dist=0.934,
        name="upper_hinge_axes_are_separate",
    )
    ctx.expect_origin_distance(
        lower_left,
        lower_right,
        axes="x",
        min_dist=0.930,
        max_dist=0.934,
        name="lower_hinge_axes_are_separate",
    )

    ctx.expect_contact(
        cabinet,
        upper_left,
        elem_a="upper_left_case_barrel_top_outer",
        elem_b="hinge_barrel_top",
        name="upper_left_hinge_barrel_support",
    )
    ctx.expect_contact(
        cabinet,
        upper_right,
        elem_a="upper_right_case_barrel_top_outer",
        elem_b="hinge_barrel_top",
        name="upper_right_hinge_barrel_support",
    )
    ctx.expect_contact(
        cabinet,
        lower_left,
        elem_a="lower_left_case_barrel_top_outer",
        elem_b="hinge_barrel_top",
        name="lower_left_hinge_barrel_support",
    )
    ctx.expect_contact(
        cabinet,
        lower_right,
        elem_a="lower_right_case_barrel_top_outer",
        elem_b="hinge_barrel_top",
        name="lower_right_hinge_barrel_support",
    )

    ctx.expect_gap(
        upper_right,
        upper_left,
        axis="x",
        min_gap=0.005,
        max_gap=0.007,
        name="upper_center_meeting_gap",
    )
    ctx.expect_gap(
        lower_right,
        lower_left,
        axis="x",
        min_gap=0.005,
        max_gap=0.007,
        name="lower_center_meeting_gap",
    )
    ctx.expect_gap(
        upper_left,
        lower_left,
        axis="z",
        min_gap=0.011,
        max_gap=0.013,
        name="left_row_gap",
    )
    ctx.expect_gap(
        upper_right,
        lower_right,
        axis="z",
        min_gap=0.011,
        max_gap=0.013,
        name="right_row_gap",
    )

    def _panel_front_extent(part) -> float | None:
        aabb = ctx.part_element_world_aabb(part, elem="outer_panel")
        if aabb is None:
            return None
        return aabb[1][1]

    with ctx.pose({upper_left_hinge: 1.35, upper_right_hinge: -1.35}):
        upper_left_front = _panel_front_extent(upper_left)
        upper_right_front = _panel_front_extent(upper_right)
        ctx.check(
            "upper_doors_swing_forward",
            upper_left_front is not None
            and upper_right_front is not None
            and upper_left_front > 0.74
            and upper_right_front > 0.74,
            f"front extents were left={upper_left_front}, right={upper_right_front}",
        )
        ctx.expect_gap(
            upper_right,
            upper_left,
            axis="x",
            min_gap=0.60,
            name="upper_doors_open_away_from_center",
        )

    with ctx.pose({lower_left_hinge: 1.35, lower_right_hinge: -1.35}):
        lower_left_front = _panel_front_extent(lower_left)
        lower_right_front = _panel_front_extent(lower_right)
        ctx.check(
            "lower_doors_swing_forward",
            lower_left_front is not None
            and lower_right_front is not None
            and lower_left_front > 0.74
            and lower_right_front > 0.74,
            f"front extents were left={lower_left_front}, right={lower_right_front}",
        )
        ctx.expect_gap(
            lower_right,
            lower_left,
            axis="x",
            min_gap=0.60,
            name="lower_doors_open_away_from_center",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
