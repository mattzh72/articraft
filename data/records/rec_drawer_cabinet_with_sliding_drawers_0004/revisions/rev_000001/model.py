from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

CASE_WIDTH = 0.50
CASE_DEPTH = 0.40
CASE_HEIGHT = 0.50
PANEL_THICKNESS = 0.02
BACK_THICKNESS = 0.008
DIVIDER_THICKNESS = 0.016
INNER_WIDTH = CASE_WIDTH - 2.0 * PANEL_THICKNESS
OPENING_HEIGHT = (CASE_HEIGHT - 2.0 * PANEL_THICKNESS - DIVIDER_THICKNESS) / 2.0
DRAWER_LEVEL_Z = (OPENING_HEIGHT + DIVIDER_THICKNESS) / 2.0

DRAWER_WIDTH = 0.412
DRAWER_DEPTH = 0.33
DRAWER_HEIGHT = 0.168
DRAWER_SIDE = 0.012
DRAWER_BOTTOM = 0.010
DRAWER_BACK = 0.012

FRONT_WIDTH = 0.452
FRONT_HEIGHT = 0.214
FRONT_THICKNESS = 0.018

RAIL_WIDTH = 0.020
RAIL_HEIGHT = 0.022
RAIL_LENGTH = 0.30
BODY_RAIL_X = 0.220
DRAWER_RUNNER_X = 0.205

FRONT_CENTER_Y = DRAWER_DEPTH / 2.0 + FRONT_THICKNESS / 2.0
FRONT_FACE_Y = FRONT_CENTER_Y + FRONT_THICKNESS / 2.0
DRAWER_CLOSED_Y = CASE_DEPTH / 2.0 - FRONT_FACE_Y

HANDLE_BAR_WIDTH = 0.18
HANDLE_BAR_DEPTH = 0.008
HANDLE_BAR_HEIGHT = 0.012
HANDLE_POST_WIDTH = 0.016
HANDLE_POST_DEPTH = 0.012
HANDLE_POST_HEIGHT = 0.024
HANDLE_POST_X = 0.065
HANDLE_POST_CENTER_Y = FRONT_FACE_Y + HANDLE_POST_DEPTH / 2.0
HANDLE_BAR_CENTER_Y = FRONT_FACE_Y + HANDLE_POST_DEPTH + HANDLE_BAR_DEPTH / 2.0

DRAWER_MIN_Y = -DRAWER_DEPTH / 2.0
DRAWER_MAX_Y = HANDLE_BAR_CENTER_Y + HANDLE_BAR_DEPTH / 2.0
DRAWER_AABB_DEPTH = DRAWER_MAX_Y - DRAWER_MIN_Y
DRAWER_AABB_CENTER_Y = (DRAWER_MAX_Y + DRAWER_MIN_Y) / 2.0


def _box(part, name, size, xyz, material):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _add_drawer(part, prefix, wood, face, metal):
    side_center_x = DRAWER_WIDTH / 2.0 - DRAWER_SIDE / 2.0
    _box(part, f"{prefix}_left_side", (DRAWER_SIDE, DRAWER_DEPTH, DRAWER_HEIGHT), (-side_center_x, 0.0, 0.0), wood)
    _box(part, f"{prefix}_right_side", (DRAWER_SIDE, DRAWER_DEPTH, DRAWER_HEIGHT), (side_center_x, 0.0, 0.0), wood)
    _box(
        part,
        f"{prefix}_bottom",
        (DRAWER_WIDTH - 2.0 * DRAWER_SIDE, DRAWER_DEPTH, DRAWER_BOTTOM),
        (0.0, 0.0, -DRAWER_HEIGHT / 2.0 + DRAWER_BOTTOM / 2.0),
        wood,
    )
    _box(
        part,
        f"{prefix}_back",
        (DRAWER_WIDTH - 2.0 * DRAWER_SIDE, DRAWER_BACK, DRAWER_HEIGHT - DRAWER_BOTTOM),
        (0.0, -DRAWER_DEPTH / 2.0 + DRAWER_BACK / 2.0, DRAWER_BOTTOM / 2.0),
        wood,
    )
    _box(part, f"{prefix}_front", (FRONT_WIDTH, FRONT_THICKNESS, FRONT_HEIGHT), (0.0, FRONT_CENTER_Y, 0.0), face)
    _box(part, f"{prefix}_runner_left", (0.01, RAIL_LENGTH, RAIL_HEIGHT), (-DRAWER_RUNNER_X, 0.0, 0.0), metal)
    _box(part, f"{prefix}_runner_right", (0.01, RAIL_LENGTH, RAIL_HEIGHT), (DRAWER_RUNNER_X, 0.0, 0.0), metal)
    _box(
        part,
        f"{prefix}_handle_post_left",
        (HANDLE_POST_WIDTH, HANDLE_POST_DEPTH, HANDLE_POST_HEIGHT),
        (-HANDLE_POST_X, HANDLE_POST_CENTER_Y, 0.0),
        metal,
    )
    _box(
        part,
        f"{prefix}_handle_post_right",
        (HANDLE_POST_WIDTH, HANDLE_POST_DEPTH, HANDLE_POST_HEIGHT),
        (HANDLE_POST_X, HANDLE_POST_CENTER_Y, 0.0),
        metal,
    )
    _box(
        part,
        f"{prefix}_handle_bar",
        (HANDLE_BAR_WIDTH, HANDLE_BAR_DEPTH, HANDLE_BAR_HEIGHT),
        (0.0, HANDLE_BAR_CENTER_Y, 0.0),
        metal,
    )
    part.inertial = Inertial.from_geometry(
        Box((FRONT_WIDTH, DRAWER_AABB_DEPTH, FRONT_HEIGHT)),
        mass=3.0,
        origin=Origin(xyz=(0.0, DRAWER_AABB_CENTER_Y, 0.0)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bedside_nightstand")

    wood = model.material("wood", rgba=(0.60, 0.46, 0.31, 1.0))
    face = model.material("drawer_face", rgba=(0.66, 0.52, 0.36, 1.0))
    metal = model.material("handle_metal", rgba=(0.17, 0.18, 0.20, 1.0))

    cabinet = model.part("cabinet")
    _box(cabinet, "left_side", (PANEL_THICKNESS, CASE_DEPTH, CASE_HEIGHT), (-CASE_WIDTH / 2.0 + PANEL_THICKNESS / 2.0, 0.0, 0.0), wood)
    _box(cabinet, "right_side", (PANEL_THICKNESS, CASE_DEPTH, CASE_HEIGHT), (CASE_WIDTH / 2.0 - PANEL_THICKNESS / 2.0, 0.0, 0.0), wood)
    _box(cabinet, "top_panel", (CASE_WIDTH, CASE_DEPTH, PANEL_THICKNESS), (0.0, 0.0, CASE_HEIGHT / 2.0 - PANEL_THICKNESS / 2.0), wood)
    _box(cabinet, "bottom_panel", (CASE_WIDTH, CASE_DEPTH, PANEL_THICKNESS), (0.0, 0.0, -CASE_HEIGHT / 2.0 + PANEL_THICKNESS / 2.0), wood)
    _box(
        cabinet,
        "back_panel",
        (INNER_WIDTH, BACK_THICKNESS, CASE_HEIGHT - 2.0 * PANEL_THICKNESS),
        (0.0, -CASE_DEPTH / 2.0 + BACK_THICKNESS / 2.0, 0.0),
        wood,
    )
    _box(
        cabinet,
        "center_divider",
        (INNER_WIDTH, CASE_DEPTH - BACK_THICKNESS, DIVIDER_THICKNESS),
        (0.0, -BACK_THICKNESS / 2.0, 0.0),
        wood,
    )
    for prefix, z_center in (("top", DRAWER_LEVEL_Z), ("bottom", -DRAWER_LEVEL_Z)):
        _box(cabinet, f"{prefix}_left_rail", (RAIL_WIDTH, RAIL_LENGTH, RAIL_HEIGHT), (-BODY_RAIL_X, DRAWER_CLOSED_Y, z_center), metal)
        _box(cabinet, f"{prefix}_right_rail", (RAIL_WIDTH, RAIL_LENGTH, RAIL_HEIGHT), (BODY_RAIL_X, DRAWER_CLOSED_Y, z_center), metal)
    cabinet.inertial = Inertial.from_geometry(Box((CASE_WIDTH, CASE_DEPTH, CASE_HEIGHT)), mass=18.0)

    top_drawer = model.part("top_drawer")
    _add_drawer(top_drawer, "top", wood, face, metal)

    bottom_drawer = model.part("bottom_drawer")
    _add_drawer(bottom_drawer, "bottom", wood, face, metal)

    model.articulation(
        "top_slide",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=top_drawer,
        origin=Origin(xyz=(0.0, DRAWER_CLOSED_Y, DRAWER_LEVEL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.45, lower=0.0, upper=0.22),
    )
    model.articulation(
        "bottom_slide",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=bottom_drawer,
        origin=Origin(xyz=(0.0, DRAWER_CLOSED_Y, -DRAWER_LEVEL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.45, lower=0.0, upper=0.22),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, seed=0)
    cabinet = object_model.get_part("cabinet")
    top_drawer = object_model.get_part("top_drawer")
    bottom_drawer = object_model.get_part("bottom_drawer")
    top_slide = object_model.get_articulation("top_slide")
    bottom_slide = object_model.get_articulation("bottom_slide")

    top_front = top_drawer.get_visual("top_front")
    bottom_front = bottom_drawer.get_visual("bottom_front")
    top_handle = top_drawer.get_visual("top_handle_bar")
    bottom_handle = bottom_drawer.get_visual("bottom_handle_bar")
    top_right_runner = top_drawer.get_visual("top_runner_right")
    top_left_runner = top_drawer.get_visual("top_runner_left")
    bottom_right_runner = bottom_drawer.get_visual("bottom_runner_right")
    bottom_left_runner = bottom_drawer.get_visual("bottom_runner_left")
    top_right_rail = cabinet.get_visual("top_right_rail")
    top_left_rail = cabinet.get_visual("top_left_rail")
    bottom_right_rail = cabinet.get_visual("bottom_right_rail")
    bottom_left_rail = cabinet.get_visual("bottom_left_rail")

    def _aabb_size(aabb):
        return tuple(aabb[1][i] - aabb[0][i] for i in range(3))

    def _check_slide_axis_and_limits(slide, name):
        limits = slide.motion_limits
        axis_ok = tuple(slide.axis) == (0.0, 1.0, 0.0)
        limits_ok = (
            limits is not None
            and limits.lower == 0.0
            and limits.upper == 0.22
            and limits.effort >= 50.0
            and limits.velocity > 0.0
        )
        ctx.check(f"{name}_axis_is_y", axis_ok, f"{name} axis was {slide.axis!r}")
        ctx.check(
            f"{name}_limits_match_drawer_travel",
            limits_ok,
            f"{name} limits were {limits!r}",
        )

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts(max_pose_samples=16)
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=32, overlap_tol=0.0005, overlap_volume_tol=0.0)

    _check_slide_axis_and_limits(top_slide, "top_slide")
    _check_slide_axis_and_limits(bottom_slide, "bottom_slide")

    top_aabb = ctx.part_world_aabb(top_drawer)
    bottom_aabb = ctx.part_world_aabb(bottom_drawer)
    top_front_aabb = ctx.part_element_world_aabb(top_drawer, elem=top_front)
    bottom_front_aabb = ctx.part_element_world_aabb(bottom_drawer, elem=bottom_front)
    if top_aabb is not None and bottom_aabb is not None:
        ctx.check(
            "drawer_boxes_equal_size",
            _aabb_size(top_aabb) == _aabb_size(bottom_aabb),
            f"top drawer size {_aabb_size(top_aabb)!r} vs bottom drawer size {_aabb_size(bottom_aabb)!r}",
        )
    if top_front_aabb is not None and bottom_front_aabb is not None:
        ctx.check(
            "drawer_fronts_equal_size",
            _aabb_size(top_front_aabb) == _aabb_size(bottom_front_aabb),
            f"top front size {_aabb_size(top_front_aabb)!r} vs bottom front size {_aabb_size(bottom_front_aabb)!r}",
        )

    top_rest_pos = ctx.part_world_position(top_drawer)
    bottom_rest_pos = ctx.part_world_position(bottom_drawer)

    with ctx.pose({top_slide: 0.0, bottom_slide: 0.0}):
        ctx.expect_within(top_drawer, cabinet, axes="xz", inner_elem=top_front, name="top_front_centered_in_case")
        ctx.expect_within(bottom_drawer, cabinet, axes="xz", inner_elem=bottom_front, name="bottom_front_centered_in_case")
        ctx.expect_gap(
            top_drawer,
            bottom_drawer,
            axis="z",
            min_gap=0.02,
            positive_elem=top_front,
            negative_elem=bottom_front,
            name="drawer_fronts_have_divider_gap",
        )
        ctx.expect_gap(
            top_drawer,
            cabinet,
            axis="z",
            min_gap=0.003,
            max_gap=0.008,
            positive_elem=top_front,
            negative_elem="center_divider",
            name="top_drawer_above_center_divider",
        )
        ctx.expect_gap(
            cabinet,
            top_drawer,
            axis="z",
            min_gap=0.003,
            max_gap=0.008,
            positive_elem="top_panel",
            negative_elem=top_front,
            name="top_drawer_below_top_panel",
        )
        ctx.expect_gap(
            cabinet,
            bottom_drawer,
            axis="z",
            min_gap=0.003,
            max_gap=0.008,
            positive_elem="center_divider",
            negative_elem=bottom_front,
            name="bottom_drawer_below_center_divider",
        )
        ctx.expect_gap(
            bottom_drawer,
            cabinet,
            axis="z",
            min_gap=0.003,
            max_gap=0.008,
            positive_elem=bottom_front,
            negative_elem="bottom_panel",
            name="bottom_drawer_above_bottom_panel",
        )

        ctx.expect_within(top_drawer, top_drawer, axes="xz", inner_elem=top_handle, outer_elem=top_front, name="top_handle_stays_on_front")
        ctx.expect_within(bottom_drawer, bottom_drawer, axes="xz", inner_elem=bottom_handle, outer_elem=bottom_front, name="bottom_handle_stays_on_front")
        ctx.expect_gap(
            top_drawer,
            top_drawer,
            axis="y",
            min_gap=0.01,
            max_gap=0.02,
            positive_elem=top_handle,
            negative_elem=top_front,
            name="top_handle_projects_from_front",
        )
        ctx.expect_gap(
            bottom_drawer,
            bottom_drawer,
            axis="y",
            min_gap=0.01,
            max_gap=0.02,
            positive_elem=bottom_handle,
            negative_elem=bottom_front,
            name="bottom_handle_projects_from_front",
        )

        ctx.expect_contact(cabinet, top_drawer, elem_a=top_right_rail, elem_b=top_right_runner, name="top_right_runner_contacts_rail_closed")
        ctx.expect_contact(cabinet, top_drawer, elem_a=top_left_rail, elem_b=top_left_runner, name="top_left_runner_contacts_rail_closed")
        ctx.expect_contact(cabinet, bottom_drawer, elem_a=bottom_right_rail, elem_b=bottom_right_runner, name="bottom_right_runner_contacts_rail_closed")
        ctx.expect_contact(cabinet, bottom_drawer, elem_a=bottom_left_rail, elem_b=bottom_left_runner, name="bottom_left_runner_contacts_rail_closed")
        ctx.expect_overlap(
            cabinet,
            top_drawer,
            axes="yz",
            min_overlap=0.02,
            elem_a=top_right_rail,
            elem_b=top_right_runner,
            name="top_right_runner_tracks_rail_closed",
        )
        ctx.expect_overlap(
            cabinet,
            top_drawer,
            axes="yz",
            min_overlap=0.02,
            elem_a=top_left_rail,
            elem_b=top_left_runner,
            name="top_left_runner_tracks_rail_closed",
        )
        ctx.expect_overlap(
            cabinet,
            bottom_drawer,
            axes="yz",
            min_overlap=0.02,
            elem_a=bottom_right_rail,
            elem_b=bottom_right_runner,
            name="bottom_right_runner_tracks_rail_closed",
        )
        ctx.expect_overlap(
            cabinet,
            bottom_drawer,
            axes="yz",
            min_overlap=0.02,
            elem_a=bottom_left_rail,
            elem_b=bottom_left_runner,
            name="bottom_left_runner_tracks_rail_closed",
        )

    with ctx.pose({top_slide: 0.22, bottom_slide: 0.0}):
        ctx.fail_if_isolated_parts(name="top_slide_open_no_floating")
        ctx.fail_if_parts_overlap_in_current_pose(name="top_slide_open_no_overlap")
        top_open_pos = ctx.part_world_position(top_drawer)
        if top_rest_pos is not None and top_open_pos is not None:
            ctx.check(
                "top_slide_translates_only_along_y",
                abs(top_open_pos[0] - top_rest_pos[0]) < 1e-6
                and abs(top_open_pos[2] - top_rest_pos[2]) < 1e-6
                and abs((top_open_pos[1] - top_rest_pos[1]) - 0.22) < 1e-6,
                f"rest position {top_rest_pos!r} vs open position {top_open_pos!r}",
            )
        ctx.expect_origin_gap(top_drawer, cabinet, axis="y", min_gap=0.20, max_gap=0.25, name="top_drawer_origin_moves_forward")
        ctx.expect_contact(cabinet, top_drawer, elem_a=top_right_rail, elem_b=top_right_runner, name="top_right_runner_contacts_rail_open")
        ctx.expect_contact(cabinet, top_drawer, elem_a=top_left_rail, elem_b=top_left_runner, name="top_left_runner_contacts_rail_open")
        ctx.expect_overlap(
            cabinet,
            top_drawer,
            axes="yz",
            min_overlap=0.02,
            elem_a=top_right_rail,
            elem_b=top_right_runner,
            name="top_right_runner_tracks_rail_open",
        )
        ctx.expect_overlap(
            cabinet,
            top_drawer,
            axes="yz",
            min_overlap=0.02,
            elem_a=top_left_rail,
            elem_b=top_left_runner,
            name="top_left_runner_tracks_rail_open",
        )

    with ctx.pose({top_slide: 0.0, bottom_slide: 0.22}):
        ctx.fail_if_isolated_parts(name="bottom_slide_open_no_floating")
        ctx.fail_if_parts_overlap_in_current_pose(name="bottom_slide_open_no_overlap")
        bottom_open_pos = ctx.part_world_position(bottom_drawer)
        if bottom_rest_pos is not None and bottom_open_pos is not None:
            ctx.check(
                "bottom_slide_translates_only_along_y",
                abs(bottom_open_pos[0] - bottom_rest_pos[0]) < 1e-6
                and abs(bottom_open_pos[2] - bottom_rest_pos[2]) < 1e-6
                and abs((bottom_open_pos[1] - bottom_rest_pos[1]) - 0.22) < 1e-6,
                f"rest position {bottom_rest_pos!r} vs open position {bottom_open_pos!r}",
            )
        ctx.expect_origin_gap(bottom_drawer, cabinet, axis="y", min_gap=0.20, max_gap=0.25, name="bottom_drawer_origin_moves_forward")
        ctx.expect_contact(cabinet, bottom_drawer, elem_a=bottom_right_rail, elem_b=bottom_right_runner, name="bottom_right_runner_contacts_rail_open")
        ctx.expect_contact(cabinet, bottom_drawer, elem_a=bottom_left_rail, elem_b=bottom_left_runner, name="bottom_left_runner_contacts_rail_open")
        ctx.expect_overlap(
            cabinet,
            bottom_drawer,
            axes="yz",
            min_overlap=0.02,
            elem_a=bottom_right_rail,
            elem_b=bottom_right_runner,
            name="bottom_right_runner_tracks_rail_open",
        )
        ctx.expect_overlap(
            cabinet,
            bottom_drawer,
            axes="yz",
            min_overlap=0.02,
            elem_a=bottom_left_rail,
            elem_b=bottom_left_runner,
            name="bottom_left_runner_tracks_rail_open",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
