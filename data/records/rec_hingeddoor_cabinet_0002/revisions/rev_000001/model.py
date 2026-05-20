from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)

CABINET_WIDTH = 0.60
CABINET_HEIGHT = 0.56
CABINET_DEPTH = 0.22
PANEL_THICKNESS = 0.018
BACK_THICKNESS = 0.006
SHELF_THICKNESS = 0.018
DOOR_THICKNESS = 0.018
DOOR_CENTER_GAP = 0.006
DOOR_FRONT_GAP = 0.0
DOOR_TOP_BOTTOM_REVEAL = 0.006
DOOR_OPEN_ANGLE = math.radians(110.0)

INNER_WIDTH = CABINET_WIDTH - (2.0 * PANEL_THICKNESS)
INNER_HEIGHT = CABINET_HEIGHT - (2.0 * PANEL_THICKNESS)
SHELF_DEPTH = CABINET_DEPTH - BACK_THICKNESS
DOOR_WIDTH = (CABINET_WIDTH - DOOR_CENTER_GAP) / 2.0
DOOR_HEIGHT = CABINET_HEIGHT - (2.0 * DOOR_TOP_BOTTOM_REVEAL)
LEFT_HINGE_X = -(CABINET_WIDTH / 2.0)
RIGHT_HINGE_X = CABINET_WIDTH / 2.0
HINGE_Y = CABINET_DEPTH + DOOR_THICKNESS + DOOR_FRONT_GAP
HANDLE_SIZE = (0.018, 0.012, 0.110)
HANDLE_INSET_FROM_FREE_EDGE = 0.030
HANDLE_EMBED = 0.002
HANDLE_Z = CABINET_HEIGHT / 2.0


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_cabinet", assets=ASSETS)

    cabinet_white = model.material("cabinet_white", rgba=(0.93, 0.93, 0.90, 1.0))
    interior_white = model.material("interior_white", rgba=(0.96, 0.95, 0.92, 1.0))
    handle_metal = model.material("handle_metal", rgba=(0.63, 0.65, 0.67, 1.0))

    body = model.part("cabinet_body")
    body.visual(
        Box((PANEL_THICKNESS, CABINET_DEPTH, CABINET_HEIGHT)),
        origin=Origin(
            xyz=(
                -(CABINET_WIDTH / 2.0) + (PANEL_THICKNESS / 2.0),
                CABINET_DEPTH / 2.0,
                CABINET_HEIGHT / 2.0,
            )
        ),
        material=cabinet_white,
        name="left_side_panel",
    )
    body.visual(
        Box((PANEL_THICKNESS, CABINET_DEPTH, CABINET_HEIGHT)),
        origin=Origin(
            xyz=(
                (CABINET_WIDTH / 2.0) - (PANEL_THICKNESS / 2.0),
                CABINET_DEPTH / 2.0,
                CABINET_HEIGHT / 2.0,
            )
        ),
        material=cabinet_white,
        name="right_side_panel",
    )
    body.visual(
        Box((INNER_WIDTH, CABINET_DEPTH, PANEL_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                CABINET_DEPTH / 2.0,
                PANEL_THICKNESS / 2.0,
            )
        ),
        material=cabinet_white,
        name="bottom_panel",
    )
    body.visual(
        Box((INNER_WIDTH, CABINET_DEPTH, PANEL_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                CABINET_DEPTH / 2.0,
                CABINET_HEIGHT - (PANEL_THICKNESS / 2.0),
            )
        ),
        material=cabinet_white,
        name="top_panel",
    )
    body.visual(
        Box((INNER_WIDTH, SHELF_DEPTH, SHELF_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                BACK_THICKNESS + (SHELF_DEPTH / 2.0),
                CABINET_HEIGHT / 2.0,
            )
        ),
        material=interior_white,
        name="center_shelf",
    )
    body.visual(
        Box((INNER_WIDTH, BACK_THICKNESS, INNER_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                BACK_THICKNESS / 2.0,
                CABINET_HEIGHT / 2.0,
            )
        ),
        material=interior_white,
        name="back_panel",
    )
    body.inertial = Inertial.from_geometry(
        Box((CABINET_WIDTH, CABINET_DEPTH, CABINET_HEIGHT)),
        mass=14.0,
        origin=Origin(
            xyz=(
                0.0,
                CABINET_DEPTH / 2.0,
                CABINET_HEIGHT / 2.0,
            )
        ),
    )

    left_door = model.part("left_door")
    left_door.visual(
        Box((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(
            xyz=(
                DOOR_WIDTH / 2.0,
                -(DOOR_THICKNESS / 2.0),
                CABINET_HEIGHT / 2.0,
            )
        ),
        material=cabinet_white,
        name="door_panel",
    )
    left_door.visual(
        Box(HANDLE_SIZE),
        origin=Origin(
            xyz=(
                DOOR_WIDTH - HANDLE_INSET_FROM_FREE_EDGE,
                (HANDLE_SIZE[1] / 2.0) - HANDLE_EMBED,
                HANDLE_Z,
            )
        ),
        material=handle_metal,
        name="handle_pull",
    )
    left_door.inertial = Inertial.from_geometry(
        Box((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        mass=2.0,
        origin=Origin(
            xyz=(
                DOOR_WIDTH / 2.0,
                -(DOOR_THICKNESS / 2.0),
                CABINET_HEIGHT / 2.0,
            )
        ),
    )

    right_door = model.part("right_door")
    right_door.visual(
        Box((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(
            xyz=(
                -(DOOR_WIDTH / 2.0),
                -(DOOR_THICKNESS / 2.0),
                CABINET_HEIGHT / 2.0,
            )
        ),
        material=cabinet_white,
        name="door_panel",
    )
    right_door.visual(
        Box(HANDLE_SIZE),
        origin=Origin(
            xyz=(
                -DOOR_WIDTH + HANDLE_INSET_FROM_FREE_EDGE,
                (HANDLE_SIZE[1] / 2.0) - HANDLE_EMBED,
                HANDLE_Z,
            )
        ),
        material=handle_metal,
        name="handle_pull",
    )
    right_door.inertial = Inertial.from_geometry(
        Box((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        mass=2.0,
        origin=Origin(
            xyz=(
                -(DOOR_WIDTH / 2.0),
                -(DOOR_THICKNESS / 2.0),
                CABINET_HEIGHT / 2.0,
            )
        ),
    )

    model.articulation(
        "left_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_door,
        origin=Origin(xyz=(LEFT_HINGE_X, HINGE_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.0,
            lower=0.0,
            upper=DOOR_OPEN_ANGLE,
        ),
    )
    model.articulation(
        "right_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_door,
        origin=Origin(xyz=(RIGHT_HINGE_X, HINGE_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.0,
            lower=-DOOR_OPEN_ANGLE,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    body = object_model.get_part("cabinet_body")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    left_hinge = object_model.get_articulation("left_door_hinge")
    right_hinge = object_model.get_articulation("right_door_hinge")

    left_side_panel = body.get_visual("left_side_panel")
    right_side_panel = body.get_visual("right_side_panel")
    shelf = body.get_visual("center_shelf")
    back_panel = body.get_visual("back_panel")
    left_panel = left_door.get_visual("door_panel")
    right_panel = right_door.get_visual("door_panel")
    left_handle = left_door.get_visual("handle_pull")
    right_handle = right_door.get_visual("handle_pull")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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

    body_aabb = ctx.part_world_aabb(body)
    ctx.check("body_aabb_exists", body_aabb is not None, details="Cabinet body needs measurable geometry.")
    if body_aabb is not None:
        width = body_aabb[1][0] - body_aabb[0][0]
        depth = body_aabb[1][1] - body_aabb[0][1]
        height = body_aabb[1][2] - body_aabb[0][2]
        ctx.check(
            "body_proportions_match_small_wall_cabinet",
            abs(width - CABINET_WIDTH) < 1e-4
            and abs(depth - CABINET_DEPTH) < 1e-4
            and abs(height - CABINET_HEIGHT) < 1e-4,
            details=f"Expected {(CABINET_WIDTH, CABINET_DEPTH, CABINET_HEIGHT)}, got {(width, depth, height)}.",
        )

    ctx.check(
        "door_hinges_use_vertical_axes",
        getattr(left_hinge, "axis", None) == (0.0, 0.0, 1.0)
        and getattr(right_hinge, "axis", None) == (0.0, 0.0, 1.0),
        details="Both doors should rotate on vertical hinge axes.",
    )
    ctx.check(
        "door_hinge_limits_reach_about_110_degrees",
        left_hinge.motion_limits is not None
        and right_hinge.motion_limits is not None
        and abs(left_hinge.motion_limits.lower - 0.0) < 1e-6
        and abs(left_hinge.motion_limits.upper - DOOR_OPEN_ANGLE) < 1e-6
        and abs(right_hinge.motion_limits.lower + DOOR_OPEN_ANGLE) < 1e-6
        and abs(right_hinge.motion_limits.upper - 0.0) < 1e-6,
        details="Expected mirrored side-hinged limits from closed to about 110 degrees open.",
    )

    ctx.expect_gap(
        left_door,
        body,
        axis="y",
        max_gap=0.0005,
        max_penetration=1e-6,
        positive_elem=left_panel,
        negative_elem=left_side_panel,
        name="left_door_sits_just_proud_of_cabinet_front",
    )
    ctx.expect_gap(
        right_door,
        body,
        axis="y",
        max_gap=0.0005,
        max_penetration=1e-6,
        positive_elem=right_panel,
        negative_elem=right_side_panel,
        name="right_door_sits_just_proud_of_cabinet_front",
    )
    ctx.expect_overlap(
        left_door,
        body,
        axes="xz",
        min_overlap=0.20,
        elem_a=left_panel,
        name="left_door_covers_left_half_of_front_opening",
    )
    ctx.expect_overlap(
        right_door,
        body,
        axes="xz",
        min_overlap=0.20,
        elem_a=right_panel,
        name="right_door_covers_right_half_of_front_opening",
    )
    ctx.expect_gap(
        right_door,
        left_door,
        axis="x",
        min_gap=0.002,
        max_gap=0.010,
        positive_elem=right_panel,
        negative_elem=left_panel,
        name="closed_doors_leave_only_a_small_center_reveal",
    )
    ctx.expect_contact(
        left_door,
        left_door,
        elem_a=left_panel,
        elem_b=left_handle,
        name="left_handle_is_attached_to_left_door_panel",
    )
    ctx.expect_contact(
        right_door,
        right_door,
        elem_a=right_panel,
        elem_b=right_handle,
        name="right_handle_is_attached_to_right_door_panel",
    )

    shelf_aabb = ctx.part_element_world_aabb(body, elem=shelf)
    back_aabb = ctx.part_element_world_aabb(body, elem=back_panel)
    ctx.check(
        "shelf_and_back_panel_exist",
        shelf_aabb is not None and back_aabb is not None,
        details="The cabinet body should include both a central shelf and a back panel.",
    )
    if shelf_aabb is not None:
        ctx.check(
            "shelf_sits_at_mid_height",
            abs(((shelf_aabb[0][2] + shelf_aabb[1][2]) / 2.0) - (CABINET_HEIGHT / 2.0)) < 1e-4,
            details="Expected the shelf to be centered vertically in the cabinet.",
        )

    left_closed_aabb = ctx.part_element_world_aabb(left_door, elem=left_panel)
    right_closed_aabb = ctx.part_element_world_aabb(right_door, elem=right_panel)
    ctx.check(
        "door_panel_aabbs_exist_in_closed_pose",
        left_closed_aabb is not None and right_closed_aabb is not None,
        details="Door panels need measurable bounds in the closed pose.",
    )

    with ctx.pose({left_hinge: math.radians(100.0), right_hinge: -math.radians(100.0)}):
        ctx.fail_if_parts_overlap_in_current_pose(name="open_pose_has_no_part_overlaps")
        left_open_aabb = ctx.part_element_world_aabb(left_door, elem=left_panel)
        right_open_aabb = ctx.part_element_world_aabb(right_door, elem=right_panel)
        ctx.check(
            "door_panel_aabbs_exist_in_open_pose",
            left_open_aabb is not None and right_open_aabb is not None,
            details="Door panels need measurable bounds in the open pose.",
        )
        if left_closed_aabb is not None and left_open_aabb is not None:
            left_closed_x_span = left_closed_aabb[1][0] - left_closed_aabb[0][0]
            left_open_x_span = left_open_aabb[1][0] - left_open_aabb[0][0]
            left_open_y_span = left_open_aabb[1][1] - left_open_aabb[0][1]
            ctx.check(
                "left_door_rotates_out_of_the_front_plane",
                left_open_x_span < (left_closed_x_span * 0.4) and left_open_y_span > 0.24,
                details=(
                    "Left door should narrow in X and stretch along Y when opened; "
                    f"closed_x={left_closed_x_span}, open_x={left_open_x_span}, open_y={left_open_y_span}."
                ),
            )
        if right_closed_aabb is not None and right_open_aabb is not None:
            right_closed_x_span = right_closed_aabb[1][0] - right_closed_aabb[0][0]
            right_open_x_span = right_open_aabb[1][0] - right_open_aabb[0][0]
            right_open_y_span = right_open_aabb[1][1] - right_open_aabb[0][1]
            ctx.check(
                "right_door_rotates_out_of_the_front_plane",
                right_open_x_span < (right_closed_x_span * 0.4) and right_open_y_span > 0.24,
                details=(
                    "Right door should narrow in X and stretch along Y when opened; "
                    f"closed_x={right_closed_x_span}, open_x={right_open_x_span}, open_y={right_open_y_span}."
                ),
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
