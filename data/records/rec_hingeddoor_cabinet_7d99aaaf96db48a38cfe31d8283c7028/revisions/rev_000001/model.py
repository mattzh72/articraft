from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


CABINET_WIDTH = 0.60
CABINET_HEIGHT = 0.55
CABINET_DEPTH = 0.18

SIDE_THICKNESS = 0.016
BACK_THICKNESS = 0.008
SHELF_THICKNESS = 0.014

DOOR_THICKNESS = 0.018
DOOR_CENTER_GAP = 0.004
DOOR_FRONT_GAP = 0.0015
DOOR_TOP_BOTTOM_REVEAL = 0.008
DOOR_HEIGHT = CABINET_HEIGHT - DOOR_TOP_BOTTOM_REVEAL
HINGE_STILE_SIZE = (0.010, 0.016, DOOR_HEIGHT - 0.040)
HINGE_STILE_CENTER_Y = 0.004
DOOR_PANEL_INSET_X = 0.008
DOOR_WIDTH = CABINET_WIDTH / 2.0 - DOOR_CENTER_GAP / 2.0 - DOOR_PANEL_INSET_X

SHELF_WIDTH = CABINET_WIDTH - 2.0 * SIDE_THICKNESS
SHELF_DEPTH = CABINET_DEPTH - BACK_THICKNESS - 0.022
SHELF_CENTER_Y = -CABINET_DEPTH / 2.0 + BACK_THICKNESS + SHELF_DEPTH / 2.0
LOWER_SHELF_Z = -0.09
UPPER_SHELF_Z = 0.09

PULL_EDGE_OFFSET = 0.040
PULL_BAR_SIZE = (0.014, 0.012, 0.12)
PULL_POST_SIZE = (0.012, 0.018, 0.012)
PULL_POST_Z_OFFSET = 0.038
PULL_BAR_CENTER_Y = (
    DOOR_THICKNESS / 2.0 + PULL_POST_SIZE[1] + PULL_BAR_SIZE[1] / 2.0
)
PULL_POST_CENTER_Y = DOOR_THICKNESS / 2.0 + PULL_POST_SIZE[1] / 2.0

LEFT_HINGE_X = -CABINET_WIDTH / 2.0
RIGHT_HINGE_X = CABINET_WIDTH / 2.0
HINGE_Y = CABINET_DEPTH / 2.0 - 0.006
DOOR_PANEL_CENTER_Y = (
    CABINET_DEPTH / 2.0 + DOOR_FRONT_GAP + DOOR_THICKNESS / 2.0 - HINGE_Y
)
DOOR_SWING = 1.92


def _add_pull_visuals(door_part, pull_x: float) -> None:
    door_part.visual(
        Box(PULL_BAR_SIZE),
        origin=Origin(xyz=(pull_x, PULL_BAR_CENTER_Y, 0.0)),
        material="pull_metal",
        name="pull_bar",
    )
    door_part.visual(
        Box(PULL_POST_SIZE),
        origin=Origin(xyz=(pull_x, PULL_POST_CENTER_Y, -PULL_POST_Z_OFFSET)),
        material="pull_metal",
        name="lower_pull_post",
    )
    door_part.visual(
        Box(PULL_POST_SIZE),
        origin=Origin(xyz=(pull_x, PULL_POST_CENTER_Y, PULL_POST_Z_OFFSET)),
        material="pull_metal",
        name="upper_pull_post",
    )


def _add_hidden_hinge_stile(door_part, side: str) -> None:
    stile_x = HINGE_STILE_SIZE[0] / 2.0 if side == "left" else -HINGE_STILE_SIZE[0] / 2.0
    door_part.visual(
        Box(HINGE_STILE_SIZE),
        origin=Origin(xyz=(stile_x, HINGE_STILE_CENTER_Y, 0.0)),
        material="pull_metal",
        name="hinge_stile",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_cabinet")

    model.material("cabinet_paint", rgba=(0.90, 0.90, 0.87, 1.0))
    model.material("door_paint", rgba=(0.96, 0.96, 0.94, 1.0))
    model.material("shelf_wood", rgba=(0.75, 0.66, 0.52, 1.0))
    model.material("pull_metal", rgba=(0.24, 0.24, 0.27, 1.0))

    body = model.part("cabinet_body")
    body.visual(
        Box((SIDE_THICKNESS, CABINET_DEPTH, CABINET_HEIGHT)),
        origin=Origin(xyz=(-CABINET_WIDTH / 2.0 + SIDE_THICKNESS / 2.0, 0.0, 0.0)),
        material="cabinet_paint",
        name="left_side",
    )
    body.visual(
        Box((SIDE_THICKNESS, CABINET_DEPTH, CABINET_HEIGHT)),
        origin=Origin(xyz=(CABINET_WIDTH / 2.0 - SIDE_THICKNESS / 2.0, 0.0, 0.0)),
        material="cabinet_paint",
        name="right_side",
    )
    body.visual(
        Box((CABINET_WIDTH - 2.0 * SIDE_THICKNESS, CABINET_DEPTH, SIDE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, CABINET_HEIGHT / 2.0 - SIDE_THICKNESS / 2.0)),
        material="cabinet_paint",
        name="top_panel",
    )
    body.visual(
        Box((CABINET_WIDTH - 2.0 * SIDE_THICKNESS, CABINET_DEPTH, SIDE_THICKNESS)),
        origin=Origin(
            xyz=(0.0, 0.0, -CABINET_HEIGHT / 2.0 + SIDE_THICKNESS / 2.0)
        ),
        material="cabinet_paint",
        name="bottom_panel",
    )
    body.visual(
        Box(
            (
                CABINET_WIDTH - 2.0 * SIDE_THICKNESS,
                BACK_THICKNESS,
                CABINET_HEIGHT - 2.0 * SIDE_THICKNESS,
            )
        ),
        origin=Origin(
            xyz=(
                0.0,
                -CABINET_DEPTH / 2.0 + BACK_THICKNESS / 2.0,
                0.0,
            )
        ),
        material="cabinet_paint",
        name="back_panel",
    )
    lower_shelf = model.part("lower_shelf")
    lower_shelf.visual(
        Box((SHELF_WIDTH, SHELF_DEPTH, SHELF_THICKNESS)),
        material="shelf_wood",
        name="shelf_board",
    )

    upper_shelf = model.part("upper_shelf")
    upper_shelf.visual(
        Box((SHELF_WIDTH, SHELF_DEPTH, SHELF_THICKNESS)),
        material="shelf_wood",
        name="shelf_board",
    )

    model.articulation(
        "body_to_lower_shelf",
        ArticulationType.FIXED,
        parent=body,
        child=lower_shelf,
        origin=Origin(xyz=(0.0, SHELF_CENTER_Y, LOWER_SHELF_Z)),
    )
    model.articulation(
        "body_to_upper_shelf",
        ArticulationType.FIXED,
        parent=body,
        child=upper_shelf,
        origin=Origin(xyz=(0.0, SHELF_CENTER_Y, UPPER_SHELF_Z)),
    )

    left_door = model.part("left_door")
    left_door.visual(
        Box((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(
            xyz=(DOOR_PANEL_INSET_X + DOOR_WIDTH / 2.0, DOOR_PANEL_CENTER_Y, 0.0)
        ),
        material="door_paint",
        name="door_panel",
    )
    _add_hidden_hinge_stile(left_door, "left")
    _add_pull_visuals(left_door, DOOR_PANEL_INSET_X + DOOR_WIDTH - PULL_EDGE_OFFSET)

    right_door = model.part("right_door")
    right_door.visual(
        Box((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(
            xyz=(
                -(DOOR_PANEL_INSET_X + DOOR_WIDTH / 2.0),
                DOOR_PANEL_CENTER_Y,
                0.0,
            )
        ),
        material="door_paint",
        name="door_panel",
    )
    _add_hidden_hinge_stile(right_door, "right")
    _add_pull_visuals(
        right_door,
        -(DOOR_PANEL_INSET_X + DOOR_WIDTH - PULL_EDGE_OFFSET),
    )

    model.articulation(
        "body_to_left_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_door,
        origin=Origin(xyz=(LEFT_HINGE_X, HINGE_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=0.0,
            upper=DOOR_SWING,
        ),
    )
    model.articulation(
        "body_to_right_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_door,
        origin=Origin(xyz=(RIGHT_HINGE_X, HINGE_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=-DOOR_SWING,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("cabinet_body")
    lower_shelf = object_model.get_part("lower_shelf")
    upper_shelf = object_model.get_part("upper_shelf")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")

    left_hinge = object_model.get_articulation("body_to_left_door")
    right_hinge = object_model.get_articulation("body_to_right_door")

    body_visual_names = {visual.name for visual in body.visuals}
    left_door_visual_names = {visual.name for visual in left_door.visuals}
    right_door_visual_names = {visual.name for visual in right_door.visuals}

    left_panel = left_door.get_visual("door_panel")
    right_panel = right_door.get_visual("door_panel")
    left_pull = left_door.get_visual("pull_bar")
    right_pull = right_door.get_visual("pull_bar")
    left_side = body.get_visual("left_side")
    right_side = body.get_visual("right_side")
    left_hinge_stile = left_door.get_visual("hinge_stile")
    right_hinge_stile = right_door.get_visual("hinge_stile")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        body,
        left_door,
        elem_a=left_side,
        elem_b=left_hinge_stile,
        reason="hidden left hinge stile sits inside the side-panel hinge recess to keep the door mounted",
    )
    ctx.allow_overlap(
        body,
        right_door,
        elem_a=right_side,
        elem_b=right_hinge_stile,
        reason="hidden right hinge stile sits inside the side-panel hinge recess to keep the door mounted",
    )

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

    ctx.check(
        "body_panels_present",
        {
            "left_side",
            "right_side",
            "top_panel",
            "bottom_panel",
            "back_panel",
        }.issubset(body_visual_names),
        details=f"body visuals: {sorted(body_visual_names)}",
    )
    ctx.check(
        "left_door_features_present",
        {
            "door_panel",
            "hinge_stile",
            "pull_bar",
            "lower_pull_post",
            "upper_pull_post",
        }.issubset(left_door_visual_names),
        details=f"left door visuals: {sorted(left_door_visual_names)}",
    )
    ctx.check(
        "right_door_features_present",
        {
            "door_panel",
            "hinge_stile",
            "pull_bar",
            "lower_pull_post",
            "upper_pull_post",
        }.issubset(right_door_visual_names),
        details=f"right door visuals: {sorted(right_door_visual_names)}",
    )

    ctx.expect_contact(lower_shelf, body, name="lower_shelf_contacts_body")
    ctx.expect_contact(upper_shelf, body, name="upper_shelf_contacts_body")
    ctx.expect_within(
        lower_shelf,
        body,
        axes="xyz",
        margin=0.0,
        name="lower_shelf_within_body_bounds",
    )
    ctx.expect_within(
        upper_shelf,
        body,
        axes="xyz",
        margin=0.0,
        name="upper_shelf_within_body_bounds",
    )

    ctx.check(
        "left_hinge_vertical",
        left_hinge.axis == (0.0, 0.0, 1.0),
        details=f"left hinge axis = {left_hinge.axis}",
    )
    ctx.check(
        "right_hinge_vertical",
        right_hinge.axis == (0.0, 0.0, 1.0),
        details=f"right hinge axis = {right_hinge.axis}",
    )

    left_limits = left_hinge.motion_limits
    right_limits = right_hinge.motion_limits
    ctx.check(
        "left_hinge_limits_outward",
        left_limits is not None
        and left_limits.lower is not None
        and left_limits.upper is not None
        and isclose(left_limits.lower, 0.0, abs_tol=1e-9)
        and left_limits.upper > 1.7,
        details=f"left hinge limits = {left_limits}",
    )
    ctx.check(
        "right_hinge_limits_outward",
        right_limits is not None
        and right_limits.lower is not None
        and right_limits.upper is not None
        and right_limits.lower < -1.7
        and isclose(right_limits.upper, 0.0, abs_tol=1e-9),
        details=f"right hinge limits = {right_limits}",
    )
    ctx.check(
        "hinges_at_cabinet_sides",
        isclose(left_hinge.origin.xyz[0], LEFT_HINGE_X, abs_tol=1e-9)
        and isclose(right_hinge.origin.xyz[0], RIGHT_HINGE_X, abs_tol=1e-9),
        details=(
            f"left hinge x={left_hinge.origin.xyz[0]}, "
            f"right hinge x={right_hinge.origin.xyz[0]}"
        ),
    )

    ctx.check(
        "left_pull_near_free_edge",
        left_pull.origin.xyz[0] > DOOR_WIDTH * 0.75
        and left_pull.origin.xyz[1] > DOOR_THICKNESS / 2.0,
        details=f"left pull origin = {left_pull.origin.xyz}",
    )
    ctx.check(
        "right_pull_near_free_edge",
        right_pull.origin.xyz[0] < -DOOR_WIDTH * 0.75
        and right_pull.origin.xyz[1] > DOOR_THICKNESS / 2.0,
        details=f"right pull origin = {right_pull.origin.xyz}",
    )

    with ctx.pose({left_hinge: 0.0, right_hinge: 0.0}):
        ctx.expect_gap(
            left_door,
            body,
            axis="y",
            min_gap=0.001,
            max_gap=0.004,
            positive_elem=left_panel,
            name="left_door_front_clearance",
        )
        ctx.expect_gap(
            right_door,
            body,
            axis="y",
            min_gap=0.001,
            max_gap=0.004,
            positive_elem=right_panel,
            name="right_door_front_clearance",
        )
        ctx.expect_gap(
            right_door,
            left_door,
            axis="x",
            min_gap=0.002,
            max_gap=0.006,
            positive_elem=right_panel,
            negative_elem=left_panel,
            name="closed_door_center_gap",
        )
        ctx.expect_overlap(
            left_door,
            body,
            axes="xz",
            min_overlap=0.20,
            elem_a=left_panel,
            name="left_door_covers_left_opening",
        )
        ctx.expect_overlap(
            right_door,
            body,
            axes="xz",
            min_overlap=0.20,
            elem_a=right_panel,
            name="right_door_covers_right_opening",
        )

    if left_limits is not None and left_limits.upper is not None:
        with ctx.pose({left_hinge: left_limits.upper, right_hinge: 0.0}):
            ctx.fail_if_parts_overlap_in_current_pose(
                name="left_door_open_no_overlap"
            )
            ctx.fail_if_isolated_parts(name="left_door_open_no_floating")
    if right_limits is not None and right_limits.lower is not None:
        with ctx.pose({left_hinge: 0.0, right_hinge: right_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(
                name="right_door_open_no_overlap"
            )
            ctx.fail_if_isolated_parts(name="right_door_open_no_floating")
    if (
        left_limits is not None
        and left_limits.upper is not None
        and right_limits is not None
        and right_limits.lower is not None
    ):
        with ctx.pose(
            {left_hinge: left_limits.upper, right_hinge: right_limits.lower}
        ):
            ctx.fail_if_parts_overlap_in_current_pose(
                name="both_doors_open_no_overlap"
            )
            ctx.fail_if_isolated_parts(name="both_doors_open_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
