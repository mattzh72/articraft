from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import os

_SPEC_ORIGIN = getattr(globals().get("__spec__"), "origin", None)
if isinstance(_SPEC_ORIGIN, str) and _SPEC_ORIGIN.startswith("/"):
    __file__ = _SPEC_ORIGIN
    _SCRIPT_DIR = os.path.dirname(_SPEC_ORIGIN)
    if _SCRIPT_DIR:
        try:
            os.chdir(_SCRIPT_DIR)
        except FileNotFoundError:
            os.chdir("/")

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

TOP_WIDTH = 1.32
TOP_DEPTH = 0.68
TOP_THICKNESS = 0.04
DESK_HEIGHT = 0.75

LEG_RADIUS = 0.012
LEG_HEIGHT = DESK_HEIGHT - TOP_THICKNESS
LEG_X = TOP_WIDTH / 2.0 - 0.09
LEG_Y = TOP_DEPTH / 2.0 - 0.08

CASE_WIDTH = 0.34
CASE_DEPTH = 0.44
CASE_HEIGHT = 0.25
CASE_CENTER_X = 0.34
CASE_FRONT_Y = -0.32
CASE_CENTER_Y = CASE_FRONT_Y + CASE_DEPTH / 2.0
CASE_BOTTOM_Z = DESK_HEIGHT - TOP_THICKNESS - CASE_HEIGHT
CASE_CENTER_Z = CASE_BOTTOM_Z + CASE_HEIGHT / 2.0

CASE_TOP_T = 0.016
CASE_BOTTOM_T = 0.014
CASE_SIDE_T = 0.014
CASE_BACK_T = 0.012
CASE_DIVIDER_T = 0.012
CASE_SIDE_HEIGHT = CASE_HEIGHT - CASE_TOP_T - CASE_BOTTOM_T + 0.002

DRAWER_BODY_WIDTH = 0.29
DRAWER_BODY_DEPTH = 0.35
DRAWER_BODY_HEIGHT = 0.082
DRAWER_FRONT_WIDTH = 0.308
DRAWER_FRONT_HEIGHT = 0.098
DRAWER_FRONT_T = 0.018
DRAWER_PULL_WIDTH = 0.16
DRAWER_PULL_HEIGHT = 0.012
DRAWER_PULL_DEPTH = 0.012
DRAWER_OPEN_TRAVEL = 0.22

TRACK_T = 0.005
TRACK_DEPTH = 0.30
TRACK_HEIGHT = 0.010
TRACK_CENTER_Y = -0.005
RUNNER_T = 0.006
RUNNER_DEPTH = 0.22
RUNNER_HEIGHT = 0.010

INNER_WIDTH = CASE_WIDTH - 2.0 * CASE_SIDE_T
INNER_DEPTH = CASE_DEPTH - CASE_BACK_T
OPENING_HEIGHT = (CASE_HEIGHT - CASE_TOP_T - CASE_BOTTOM_T - CASE_DIVIDER_T) / 2.0
DRAWER_SIDE_T = 0.012
DRAWER_BOTTOM_T = 0.009
DRAWER_BACK_T = 0.010
DRAWER_CLOSED_Y = -CASE_DEPTH / 2.0 + DRAWER_FRONT_T + DRAWER_BODY_DEPTH / 2.0
DRAWER_RUNNER_Y = TRACK_CENTER_Y - DRAWER_CLOSED_Y
UPPER_DRAWER_CENTER_Z = CASE_BOTTOM_Z + CASE_BOTTOM_T + OPENING_HEIGHT + CASE_DIVIDER_T + OPENING_HEIGHT / 2.0
LOWER_DRAWER_CENTER_Z = CASE_BOTTOM_Z + CASE_BOTTOM_T + OPENING_HEIGHT / 2.0


def _add_drawer_part(model: ArticulatedObject, name: str) -> None:
    drawer = model.part(name)
    drawer.visual(
        Box((DRAWER_BODY_WIDTH, DRAWER_BODY_DEPTH, DRAWER_BOTTOM_T)),
        material="soft_white",
        origin=Origin(xyz=(0.0, 0.0, -DRAWER_BODY_HEIGHT / 2.0 + DRAWER_BOTTOM_T / 2.0)),
        name="drawer_bottom",
    )
    drawer.visual(
        Box((DRAWER_SIDE_T, DRAWER_BODY_DEPTH, DRAWER_BODY_HEIGHT - DRAWER_BOTTOM_T)),
        material="soft_white",
        origin=Origin(
            xyz=(
                -(DRAWER_BODY_WIDTH / 2.0 - DRAWER_SIDE_T / 2.0),
                0.0,
                DRAWER_BOTTOM_T / 2.0,
            )
        ),
        name="drawer_left_side",
    )
    drawer.visual(
        Box((DRAWER_SIDE_T, DRAWER_BODY_DEPTH, DRAWER_BODY_HEIGHT - DRAWER_BOTTOM_T)),
        material="soft_white",
        origin=Origin(
            xyz=(
                DRAWER_BODY_WIDTH / 2.0 - DRAWER_SIDE_T / 2.0,
                0.0,
                DRAWER_BOTTOM_T / 2.0,
            )
        ),
        name="drawer_right_side",
    )
    drawer.visual(
        Box((DRAWER_BODY_WIDTH - 2.0 * DRAWER_SIDE_T, DRAWER_BACK_T, DRAWER_BODY_HEIGHT - DRAWER_BOTTOM_T)),
        material="soft_white",
        origin=Origin(
            xyz=(
                0.0,
                DRAWER_BODY_DEPTH / 2.0 - DRAWER_BACK_T / 2.0,
                DRAWER_BOTTOM_T / 2.0,
            )
        ),
        name="drawer_back",
    )
    drawer.visual(
        Box((DRAWER_FRONT_WIDTH, DRAWER_FRONT_T, DRAWER_FRONT_HEIGHT)),
        origin=Origin(xyz=(0.0, -(DRAWER_BODY_DEPTH / 2.0 + DRAWER_FRONT_T / 2.0), 0.0)),
        material="soft_white",
        name="drawer_front",
    )
    drawer.visual(
        Box((DRAWER_PULL_WIDTH, DRAWER_PULL_DEPTH, DRAWER_PULL_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -(DRAWER_BODY_DEPTH / 2.0 + DRAWER_FRONT_T + DRAWER_PULL_DEPTH / 2.0),
                DRAWER_FRONT_HEIGHT / 2.0 - 0.020,
            )
        ),
        material="light_oak",
        name="drawer_pull",
    )
    for side_name, runner_x in (
        ("left_runner", -(DRAWER_BODY_WIDTH / 2.0 + RUNNER_T / 2.0)),
        ("right_runner", DRAWER_BODY_WIDTH / 2.0 + RUNNER_T / 2.0),
    ):
        drawer.visual(
            Box((RUNNER_T, RUNNER_DEPTH, RUNNER_HEIGHT)),
            origin=Origin(xyz=(runner_x, DRAWER_RUNNER_Y, 0.0)),
            material="graphite_steel",
            name=side_name,
        )
    drawer.inertial = Inertial.from_geometry(
        Box((DRAWER_BODY_WIDTH, DRAWER_BODY_DEPTH, DRAWER_BODY_HEIGHT)),
        mass=3.0,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="scandinavian_desk")

    oak = model.material("light_oak", rgba=(0.74, 0.60, 0.42, 1.0))
    white = model.material("soft_white", rgba=(0.93, 0.93, 0.91, 1.0))
    graphite = model.material("graphite_steel", rgba=(0.24, 0.25, 0.27, 1.0))

    desk_frame = model.part("desk_frame")
    top_center_z = DESK_HEIGHT - TOP_THICKNESS / 2.0
    desk_frame.visual(
        Box((TOP_WIDTH, TOP_DEPTH, TOP_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, top_center_z)),
        material=oak,
        name="top",
    )
    for leg_name, x_pos, y_pos in (
        ("front_left_leg", -LEG_X, -LEG_Y),
        ("front_right_leg", LEG_X, -LEG_Y),
        ("rear_left_leg", -LEG_X, LEG_Y),
        ("rear_right_leg", LEG_X, LEG_Y),
    ):
        desk_frame.visual(
            Cylinder(radius=LEG_RADIUS, length=LEG_HEIGHT),
            origin=Origin(xyz=(x_pos, y_pos, LEG_HEIGHT / 2.0)),
            material=graphite,
            name=leg_name,
        )
    desk_frame.visual(
        Box((TOP_WIDTH - 0.16, 0.018, 0.03)),
        origin=Origin(xyz=(0.0, TOP_DEPTH / 2.0 - 0.055, DESK_HEIGHT - TOP_THICKNESS - 0.015)),
        material=graphite,
        name="rear_stretcher",
    )
    desk_frame.inertial = Inertial.from_geometry(
        Box((TOP_WIDTH, TOP_DEPTH, TOP_THICKNESS)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, top_center_z)),
    )

    drawer_case = model.part("drawer_case")
    local_top_z = CASE_HEIGHT / 2.0 - CASE_TOP_T / 2.0
    local_bottom_z = -CASE_HEIGHT / 2.0 + CASE_BOTTOM_T / 2.0
    local_side_x = CASE_WIDTH / 2.0 - CASE_SIDE_T / 2.0
    local_shell_z = 0.0
    local_back_y = CASE_DEPTH / 2.0 - CASE_BACK_T / 2.0
    local_divider_y = -CASE_BACK_T / 2.0
    upper_track_z = UPPER_DRAWER_CENTER_Z - CASE_CENTER_Z
    lower_track_z = LOWER_DRAWER_CENTER_Z - CASE_CENTER_Z
    left_track_x = -INNER_WIDTH / 2.0 + TRACK_T / 2.0
    right_track_x = INNER_WIDTH / 2.0 - TRACK_T / 2.0

    drawer_case.visual(
        Box((CASE_WIDTH, CASE_DEPTH, CASE_TOP_T)),
        origin=Origin(xyz=(0.0, 0.0, local_top_z)),
        material=white,
        name="case_top",
    )
    drawer_case.visual(
        Box((CASE_WIDTH, CASE_DEPTH, CASE_BOTTOM_T)),
        origin=Origin(xyz=(0.0, 0.0, local_bottom_z)),
        material=white,
        name="case_bottom",
    )
    drawer_case.visual(
        Box((CASE_SIDE_T, CASE_DEPTH, CASE_SIDE_HEIGHT)),
        origin=Origin(xyz=(-local_side_x, 0.0, local_shell_z)),
        material=white,
        name="case_left",
    )
    drawer_case.visual(
        Box((CASE_SIDE_T, CASE_DEPTH, CASE_SIDE_HEIGHT)),
        origin=Origin(xyz=(local_side_x, 0.0, local_shell_z)),
        material=white,
        name="case_right",
    )
    drawer_case.visual(
        Box((INNER_WIDTH, CASE_BACK_T, CASE_SIDE_HEIGHT)),
        origin=Origin(xyz=(0.0, local_back_y, local_shell_z)),
        material=white,
        name="case_back",
    )
    drawer_case.visual(
        Box((INNER_WIDTH, INNER_DEPTH, CASE_DIVIDER_T)),
        origin=Origin(xyz=(0.0, local_divider_y, 0.0)),
        material=white,
        name="case_divider",
    )
    for rail_name, x_pos, z_pos in (
        ("upper_left_track", left_track_x, upper_track_z),
        ("upper_right_track", right_track_x, upper_track_z),
        ("lower_left_track", left_track_x, lower_track_z),
        ("lower_right_track", right_track_x, lower_track_z),
    ):
        drawer_case.visual(
            Box((TRACK_T, TRACK_DEPTH, TRACK_HEIGHT)),
            origin=Origin(xyz=(x_pos, TRACK_CENTER_Y, z_pos)),
            material=graphite,
            name=rail_name,
        )
    drawer_case.inertial = Inertial.from_geometry(
        Box((CASE_WIDTH, CASE_DEPTH, CASE_HEIGHT)),
        mass=9.0,
    )

    _add_drawer_part(model, "upper_drawer")
    _add_drawer_part(model, "lower_drawer")

    model.articulation(
        "frame_to_drawer_case",
        ArticulationType.FIXED,
        parent=desk_frame,
        child=drawer_case,
        origin=Origin(xyz=(CASE_CENTER_X, CASE_CENTER_Y, CASE_CENTER_Z)),
    )
    model.articulation(
        "case_to_upper_drawer",
        ArticulationType.PRISMATIC,
        parent=drawer_case,
        child="upper_drawer",
        origin=Origin(xyz=(0.0, DRAWER_CLOSED_Y, upper_track_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.6, lower=0.0, upper=DRAWER_OPEN_TRAVEL),
    )
    model.articulation(
        "case_to_lower_drawer",
        ArticulationType.PRISMATIC,
        parent=drawer_case,
        child="lower_drawer",
        origin=Origin(xyz=(0.0, DRAWER_CLOSED_Y, lower_track_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.6, lower=0.0, upper=DRAWER_OPEN_TRAVEL),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    desk_frame = object_model.get_part("desk_frame")
    drawer_case = object_model.get_part("drawer_case")
    upper_drawer = object_model.get_part("upper_drawer")
    lower_drawer = object_model.get_part("lower_drawer")
    upper_slide = object_model.get_articulation("case_to_upper_drawer")
    lower_slide = object_model.get_articulation("case_to_lower_drawer")

    top = desk_frame.get_visual("top")
    case_top = drawer_case.get_visual("case_top")
    case_bottom = drawer_case.get_visual("case_bottom")
    case_left = drawer_case.get_visual("case_left")
    case_back = drawer_case.get_visual("case_back")
    case_divider = drawer_case.get_visual("case_divider")
    upper_left_track = drawer_case.get_visual("upper_left_track")
    upper_right_track = drawer_case.get_visual("upper_right_track")
    lower_left_track = drawer_case.get_visual("lower_left_track")
    lower_right_track = drawer_case.get_visual("lower_right_track")
    upper_bottom = upper_drawer.get_visual("drawer_bottom")
    upper_back = upper_drawer.get_visual("drawer_back")
    upper_front = upper_drawer.get_visual("drawer_front")
    upper_left_runner = upper_drawer.get_visual("left_runner")
    upper_right_runner = upper_drawer.get_visual("right_runner")
    lower_bottom = lower_drawer.get_visual("drawer_bottom")
    lower_back = lower_drawer.get_visual("drawer_back")
    lower_front = lower_drawer.get_visual("drawer_front")
    lower_left_runner = lower_drawer.get_visual("left_runner")
    lower_right_runner = lower_drawer.get_visual("right_runner")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=64, overlap_tol=0.0005, overlap_volume_tol=0.0)

    upper_limits = upper_slide.motion_limits
    lower_limits = lower_slide.motion_limits
    ctx.check(
        "upper_drawer_axis",
        tuple(upper_slide.axis) == (0.0, -1.0, 0.0),
        f"expected upper drawer axis (0, -1, 0), got {upper_slide.axis}",
    )
    ctx.check(
        "lower_drawer_axis",
        tuple(lower_slide.axis) == (0.0, -1.0, 0.0),
        f"expected lower drawer axis (0, -1, 0), got {lower_slide.axis}",
    )
    ctx.check(
        "upper_drawer_limits",
        upper_limits is not None and upper_limits.lower == 0.0 and upper_limits.upper == DRAWER_OPEN_TRAVEL,
        f"expected upper drawer travel 0.0..{DRAWER_OPEN_TRAVEL}, got {upper_limits}",
    )
    ctx.check(
        "lower_drawer_limits",
        lower_limits is not None and lower_limits.lower == 0.0 and lower_limits.upper == DRAWER_OPEN_TRAVEL,
        f"expected lower drawer travel 0.0..{DRAWER_OPEN_TRAVEL}, got {lower_limits}",
    )

    ctx.expect_within(drawer_case, desk_frame, axes="xy", inner_elem=case_top, outer_elem=top)
    ctx.expect_origin_gap(drawer_case, desk_frame, axis="x", min_gap=0.25, max_gap=0.40)
    ctx.expect_gap(
        desk_frame,
        drawer_case,
        axis="z",
        positive_elem=top,
        negative_elem=case_top,
        max_gap=0.001,
        max_penetration=0.0,
    )

    ctx.expect_contact(drawer_case, upper_drawer, elem_a=upper_left_track, elem_b=upper_left_runner)
    ctx.expect_contact(drawer_case, upper_drawer, elem_a=upper_right_track, elem_b=upper_right_runner)
    ctx.expect_contact(drawer_case, lower_drawer, elem_a=lower_left_track, elem_b=lower_left_runner)
    ctx.expect_contact(drawer_case, lower_drawer, elem_a=lower_right_track, elem_b=lower_right_runner)
    ctx.expect_within(upper_drawer, drawer_case, axes="x", inner_elem=upper_front, outer_elem=case_divider, margin=0.002)
    ctx.expect_within(lower_drawer, drawer_case, axes="x", inner_elem=lower_front, outer_elem=case_divider, margin=0.002)

    ctx.expect_gap(
        drawer_case,
        upper_drawer,
        axis="z",
        positive_elem=case_top,
        negative_elem=upper_back,
        min_gap=0.009,
        max_gap=0.013,
    )
    ctx.expect_gap(
        upper_drawer,
        drawer_case,
        axis="z",
        positive_elem=upper_bottom,
        negative_elem=case_divider,
        min_gap=0.009,
        max_gap=0.013,
    )
    ctx.expect_gap(
        drawer_case,
        lower_drawer,
        axis="z",
        positive_elem=case_divider,
        negative_elem=lower_back,
        min_gap=0.009,
        max_gap=0.013,
    )
    ctx.expect_gap(
        lower_drawer,
        drawer_case,
        axis="z",
        positive_elem=lower_bottom,
        negative_elem=case_bottom,
        min_gap=0.009,
        max_gap=0.013,
    )
    ctx.expect_gap(
        drawer_case,
        upper_drawer,
        axis="y",
        positive_elem=case_back,
        negative_elem=upper_back,
        min_gap=0.05,
        max_gap=0.07,
    )
    ctx.expect_gap(
        drawer_case,
        lower_drawer,
        axis="y",
        positive_elem=case_back,
        negative_elem=lower_back,
        min_gap=0.05,
        max_gap=0.07,
    )
    ctx.expect_gap(
        upper_drawer,
        lower_drawer,
        axis="z",
        positive_elem=upper_front,
        negative_elem=lower_front,
        min_gap=0.014,
        max_gap=0.022,
    )

    def check_front_position(name: str, drawer_part, drawer_front, expected_min_y: float, tol: float = 0.0015) -> None:
        front_aabb = ctx.part_element_world_aabb(drawer_part, elem=drawer_front)
        if front_aabb is None:
            ctx.fail(name, "drawer front AABB could not be resolved")
            return
        actual_min_y = front_aabb[0][1]
        ctx.check(
            name,
            abs(actual_min_y - expected_min_y) <= tol,
            f"expected drawer front min y near {expected_min_y:.4f} m, got {actual_min_y:.4f} m",
        )

    case_left_aabb = ctx.part_element_world_aabb(drawer_case, elem=case_left)
    if case_left_aabb is None:
        ctx.fail("case_front_reference", "case left side AABB could not be resolved")
        case_front_y = 0.0
    else:
        case_front_y = case_left_aabb[0][1]
    check_front_position("upper_drawer_closed_flush", upper_drawer, upper_front, case_front_y)
    check_front_position("lower_drawer_closed_flush", lower_drawer, lower_front, case_front_y)

    with ctx.pose({upper_slide: DRAWER_OPEN_TRAVEL}):
        ctx.fail_if_parts_overlap_in_current_pose(name="upper_drawer_open_no_overlap")
        ctx.fail_if_isolated_parts(name="upper_drawer_open_no_floating")
        ctx.expect_contact(drawer_case, upper_drawer, elem_a=upper_left_track, elem_b=upper_left_runner)
        ctx.expect_contact(drawer_case, upper_drawer, elem_a=upper_right_track, elem_b=upper_right_runner)
        ctx.expect_gap(
            drawer_case,
            upper_drawer,
            axis="y",
            positive_elem=case_back,
            negative_elem=upper_back,
            min_gap=0.27,
            max_gap=0.29,
        )
        check_front_position(
            "upper_drawer_open_extension",
            upper_drawer,
            upper_front,
            case_front_y - DRAWER_OPEN_TRAVEL,
        )

    with ctx.pose({lower_slide: DRAWER_OPEN_TRAVEL}):
        ctx.fail_if_parts_overlap_in_current_pose(name="lower_drawer_open_no_overlap")
        ctx.fail_if_isolated_parts(name="lower_drawer_open_no_floating")
        ctx.expect_contact(drawer_case, lower_drawer, elem_a=lower_left_track, elem_b=lower_left_runner)
        ctx.expect_contact(drawer_case, lower_drawer, elem_a=lower_right_track, elem_b=lower_right_runner)
        ctx.expect_gap(
            drawer_case,
            lower_drawer,
            axis="y",
            positive_elem=case_back,
            negative_elem=lower_back,
            min_gap=0.27,
            max_gap=0.29,
        )
        check_front_position(
            "lower_drawer_open_extension",
            lower_drawer,
            lower_front,
            case_front_y - DRAWER_OPEN_TRAVEL,
        )

    with ctx.pose({upper_slide: DRAWER_OPEN_TRAVEL, lower_slide: DRAWER_OPEN_TRAVEL}):
        ctx.fail_if_parts_overlap_in_current_pose(name="both_drawers_open_no_overlap")
        ctx.fail_if_isolated_parts(name="both_drawers_open_no_floating")
        ctx.expect_contact(drawer_case, upper_drawer, elem_a=upper_left_track, elem_b=upper_left_runner)
        ctx.expect_contact(drawer_case, upper_drawer, elem_a=upper_right_track, elem_b=upper_right_runner)
        ctx.expect_contact(drawer_case, lower_drawer, elem_a=lower_left_track, elem_b=lower_left_runner)
        ctx.expect_contact(drawer_case, lower_drawer, elem_a=lower_right_track, elem_b=lower_right_runner)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
