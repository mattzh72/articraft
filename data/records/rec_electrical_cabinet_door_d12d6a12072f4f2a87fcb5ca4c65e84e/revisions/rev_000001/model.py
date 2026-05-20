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


CABINET_WIDTH = 1.20
CABINET_DEPTH = 0.45
CABINET_HEIGHT = 2.00
PLINTH_HEIGHT = 0.08
WALL_THICKNESS = 0.025

DOOR_THICKNESS = 0.022
DOOR_CLEARANCE = 0.002
DOOR_EDGE_TO_HINGE = 0.026
CENTER_SEAM_GAP = 0.020

HINGE_RADIUS = 0.018
HINGE_BODY_SEGMENT = 0.16
HINGE_MOUNT_WIDTH = 0.012
HINGE_MOUNT_DEPTH = 0.030
HINGE_LEAF_WIDTH = 0.048
HINGE_LEAF_DEPTH = 0.010

ASTRAGAL_WIDTH = 0.040
ASTRAGAL_THICKNESS = 0.010

LATCH_BRACKET_WIDTH = 0.008
LATCH_BRACKET_DEPTH = 0.010
LATCH_BRACKET_HEIGHT = 0.020
LATCH_SLEEVE_LENGTH = 0.004
LATCH_SLEEVE_RADIUS = 0.010
LATCH_BAR_LENGTH = 0.19
LATCH_BAR_THICKNESS = 0.004
LATCH_BAR_HEIGHT = 0.03
LATCH_HANDLE_LENGTH = 0.035
LATCH_HANDLE_THICKNESS = 0.018
LATCH_HANDLE_HEIGHT = 0.05

DOOR_OPEN_ANGLE = math.radians(115.0)
LATCH_OPEN_ANGLE = math.radians(100.0)

SHELL_HEIGHT = CABINET_HEIGHT - PLINTH_HEIGHT
INNER_WIDTH = CABINET_WIDTH - (2.0 * WALL_THICKNESS)
REAR_HEIGHT = SHELL_HEIGHT - (2.0 * WALL_THICKNESS)
DOOR_HEIGHT = SHELL_HEIGHT - (2.0 * WALL_THICKNESS) - (2.0 * DOOR_CLEARANCE)
DOOR_CENTER_Z = PLINTH_HEIGHT + WALL_THICKNESS + DOOR_CLEARANCE + (DOOR_HEIGHT / 2.0)
FRONT_FACE_Y = CABINET_DEPTH / 2.0
HINGE_Y = FRONT_FACE_Y + HINGE_RADIUS
HINGE_DOOR_SEGMENT = DOOR_HEIGHT - (2.0 * HINGE_BODY_SEGMENT)
HINGE_BOTTOM_Z = DOOR_CENTER_Z - (HINGE_DOOR_SEGMENT / 2.0) - (HINGE_BODY_SEGMENT / 2.0)
HINGE_TOP_Z = DOOR_CENTER_Z + (HINGE_DOOR_SEGMENT / 2.0) + (HINGE_BODY_SEGMENT / 2.0)

DOOR_PANEL_WIDTH = (CABINET_WIDTH / 2.0) - DOOR_EDGE_TO_HINGE - (CENTER_SEAM_GAP / 2.0)
LEFT_DOOR_PANEL_CENTER_X = DOOR_EDGE_TO_HINGE + (DOOR_PANEL_WIDTH / 2.0)
RIGHT_DOOR_PANEL_CENTER_X = -LEFT_DOOR_PANEL_CENTER_X
DOOR_LEAF_CENTER_Y = -(HINGE_RADIUS + (DOOR_THICKNESS / 2.0))
DOOR_OUTER_FACE_LOCAL_Y = DOOR_LEAF_CENTER_Y + (DOOR_THICKNESS / 2.0)
HINGE_LEAF_CENTER_Y = DOOR_OUTER_FACE_LOCAL_Y + (HINGE_LEAF_DEPTH / 2.0)
RIGHT_ASTRAGAL_CENTER_X = -(DOOR_EDGE_TO_HINGE + DOOR_PANEL_WIDTH) - (ASTRAGAL_WIDTH / 2.0)
ASTRAGAL_CENTER_Y = DOOR_OUTER_FACE_LOCAL_Y + (ASTRAGAL_THICKNESS / 2.0)
LATCH_PIVOT_X = -(DOOR_EDGE_TO_HINGE + DOOR_PANEL_WIDTH) + (ASTRAGAL_WIDTH / 2.0)
LATCH_BRACKET_CENTER_Y = DOOR_OUTER_FACE_LOCAL_Y + (LATCH_BRACKET_DEPTH / 2.0)
LATCH_PIVOT_Y = DOOR_OUTER_FACE_LOCAL_Y + LATCH_BRACKET_DEPTH


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="floor_switchboard")

    cabinet_steel = model.material("cabinet_steel", rgba=(0.77, 0.80, 0.83, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.36, 0.39, 0.43, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.20, 0.21, 0.23, 1.0))
    latch_black = model.material("latch_black", rgba=(0.10, 0.10, 0.11, 1.0))

    body = model.part("cabinet_body")
    body.visual(
        Box((1.10, 0.36, PLINTH_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_HEIGHT / 2.0)),
        material=dark_steel,
        name="plinth",
    )
    body.visual(
        Box((WALL_THICKNESS, CABINET_DEPTH, SHELL_HEIGHT)),
        origin=Origin(
            xyz=(-CABINET_WIDTH / 2.0 + WALL_THICKNESS / 2.0, 0.0, PLINTH_HEIGHT + (SHELL_HEIGHT / 2.0)),
        ),
        material=cabinet_steel,
        name="left_side",
    )
    body.visual(
        Box((WALL_THICKNESS, CABINET_DEPTH, SHELL_HEIGHT)),
        origin=Origin(
            xyz=(CABINET_WIDTH / 2.0 - WALL_THICKNESS / 2.0, 0.0, PLINTH_HEIGHT + (SHELL_HEIGHT / 2.0)),
        ),
        material=cabinet_steel,
        name="right_side",
    )
    body.visual(
        Box((INNER_WIDTH, CABINET_DEPTH, WALL_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, CABINET_HEIGHT - WALL_THICKNESS / 2.0)),
        material=cabinet_steel,
        name="top_panel",
    )
    body.visual(
        Box((INNER_WIDTH, CABINET_DEPTH, WALL_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_HEIGHT + WALL_THICKNESS / 2.0)),
        material=cabinet_steel,
        name="bottom_panel",
    )
    body.visual(
        Box((INNER_WIDTH, WALL_THICKNESS, REAR_HEIGHT)),
        origin=Origin(
            xyz=(0.0, -CABINET_DEPTH / 2.0 + WALL_THICKNESS / 2.0, PLINTH_HEIGHT + (SHELL_HEIGHT / 2.0)),
        ),
        material=cabinet_steel,
        name="rear_panel",
    )

    for side_name, hinge_x, mount_sign in (
        ("left", -CABINET_WIDTH / 2.0, 1.0),
        ("right", CABINET_WIDTH / 2.0, -1.0),
    ):
        mount_center_x = hinge_x + mount_sign * (HINGE_MOUNT_WIDTH / 2.0)
        for suffix, z_center in (("lower", HINGE_BOTTOM_Z), ("upper", HINGE_TOP_Z)):
            body.visual(
                Cylinder(radius=HINGE_RADIUS, length=HINGE_BODY_SEGMENT),
                origin=Origin(xyz=(hinge_x, HINGE_Y, z_center)),
                material=hinge_metal,
                name=f"{side_name}_hinge_{suffix}",
            )
            body.visual(
                Box((HINGE_MOUNT_WIDTH, HINGE_MOUNT_DEPTH, HINGE_BODY_SEGMENT)),
                origin=Origin(
                    xyz=(mount_center_x, FRONT_FACE_Y + HINGE_MOUNT_DEPTH / 2.0, z_center),
                ),
                material=hinge_metal,
                name=f"{side_name}_hinge_{suffix}_mount",
            )

    body.inertial = Inertial.from_geometry(
        Box((CABINET_WIDTH, CABINET_DEPTH, CABINET_HEIGHT)),
        mass=180.0,
        origin=Origin(xyz=(0.0, 0.0, CABINET_HEIGHT / 2.0)),
    )

    left_door = model.part("left_door")
    left_door.visual(
        Box((DOOR_PANEL_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(LEFT_DOOR_PANEL_CENTER_X, DOOR_LEAF_CENTER_Y, 0.0)),
        material=cabinet_steel,
        name="door_leaf",
    )
    left_door.visual(
        Cylinder(radius=HINGE_RADIUS, length=HINGE_DOOR_SEGMENT),
        origin=Origin(),
        material=hinge_metal,
        name="hinge_barrel",
    )
    left_door.visual(
        Box((HINGE_LEAF_WIDTH, HINGE_LEAF_DEPTH, HINGE_DOOR_SEGMENT)),
        origin=Origin(xyz=(HINGE_LEAF_WIDTH / 2.0, HINGE_LEAF_CENTER_Y, 0.0)),
        material=hinge_metal,
        name="hinge_leaf",
    )
    left_door.inertial = Inertial.from_geometry(
        Box((DOOR_PANEL_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        mass=24.0,
        origin=Origin(xyz=(LEFT_DOOR_PANEL_CENTER_X, DOOR_LEAF_CENTER_Y, 0.0)),
    )

    right_door = model.part("right_door")
    right_door.visual(
        Box((DOOR_PANEL_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(RIGHT_DOOR_PANEL_CENTER_X, DOOR_LEAF_CENTER_Y, 0.0)),
        material=cabinet_steel,
        name="door_leaf",
    )
    right_door.visual(
        Cylinder(radius=HINGE_RADIUS, length=HINGE_DOOR_SEGMENT),
        origin=Origin(),
        material=hinge_metal,
        name="hinge_barrel",
    )
    right_door.visual(
        Box((HINGE_LEAF_WIDTH, HINGE_LEAF_DEPTH, HINGE_DOOR_SEGMENT)),
        origin=Origin(xyz=(-HINGE_LEAF_WIDTH / 2.0, HINGE_LEAF_CENTER_Y, 0.0)),
        material=hinge_metal,
        name="hinge_leaf",
    )
    right_door.visual(
        Box((ASTRAGAL_WIDTH, ASTRAGAL_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(RIGHT_ASTRAGAL_CENTER_X, ASTRAGAL_CENTER_Y, 0.0)),
        material=dark_steel,
        name="overlap_stile",
    )
    right_door.visual(
        Box((LATCH_BRACKET_WIDTH, LATCH_BRACKET_DEPTH, LATCH_BRACKET_HEIGHT)),
        origin=Origin(xyz=(LATCH_PIVOT_X, LATCH_BRACKET_CENTER_Y, 0.0)),
        material=dark_steel,
        name="latch_bracket",
    )
    right_door.inertial = Inertial.from_geometry(
        Box((DOOR_PANEL_WIDTH + ASTRAGAL_WIDTH, DOOR_THICKNESS + ASTRAGAL_THICKNESS, DOOR_HEIGHT)),
        mass=25.0,
        origin=Origin(xyz=(RIGHT_DOOR_PANEL_CENTER_X, DOOR_LEAF_CENTER_Y, 0.0)),
    )

    latch_bar = model.part("latch_bar")
    latch_bar.visual(
        Cylinder(radius=LATCH_SLEEVE_RADIUS, length=LATCH_SLEEVE_LENGTH),
        origin=Origin(
            xyz=(0.0, LATCH_SLEEVE_LENGTH / 2.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=hinge_metal,
        name="pivot_sleeve",
    )
    latch_bar.visual(
        Box((LATCH_BAR_LENGTH, LATCH_BAR_THICKNESS, LATCH_BAR_HEIGHT)),
        origin=Origin(xyz=(-LATCH_BAR_LENGTH / 2.0, LATCH_SLEEVE_LENGTH + LATCH_BAR_THICKNESS / 2.0, 0.0)),
        material=latch_black,
        name="swing_bar",
    )
    latch_bar.visual(
        Box((LATCH_HANDLE_LENGTH, LATCH_HANDLE_THICKNESS, LATCH_HANDLE_HEIGHT)),
        origin=Origin(
            xyz=(
                -0.72 * LATCH_BAR_LENGTH,
                LATCH_SLEEVE_LENGTH + (LATCH_HANDLE_THICKNESS / 2.0),
                0.0,
            ),
        ),
        material=latch_black,
        name="draw_bolt_handle",
    )
    latch_bar.inertial = Inertial.from_geometry(
        Box((LATCH_BAR_LENGTH, 0.024, LATCH_HANDLE_HEIGHT)),
        mass=0.8,
        origin=Origin(xyz=(-LATCH_BAR_LENGTH / 2.0, 0.012, 0.0)),
    )

    model.articulation(
        "left_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_door,
        origin=Origin(xyz=(-CABINET_WIDTH / 2.0, HINGE_Y, DOOR_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.5, lower=0.0, upper=DOOR_OPEN_ANGLE),
    )
    model.articulation(
        "right_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_door,
        origin=Origin(xyz=(CABINET_WIDTH / 2.0, HINGE_Y, DOOR_CENTER_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.5, lower=0.0, upper=DOOR_OPEN_ANGLE),
    )
    model.articulation(
        "latch_swing",
        ArticulationType.REVOLUTE,
        parent=right_door,
        child=latch_bar,
        origin=Origin(xyz=(LATCH_PIVOT_X, LATCH_PIVOT_Y, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=LATCH_OPEN_ANGLE),
    )

    return model


def run_tests() -> TestReport:
    def _aabb_extent(aabb, axis: str) -> float:
        axis_index = {"x": 0, "y": 1, "z": 2}[axis]
        return aabb[1][axis_index] - aabb[0][axis_index]

    def _aabb_min(aabb, axis: str) -> float:
        axis_index = {"x": 0, "y": 1, "z": 2}[axis]
        return aabb[0][axis_index]

    def _aabb_max(aabb, axis: str) -> float:
        axis_index = {"x": 0, "y": 1, "z": 2}[axis]
        return aabb[1][axis_index]

    ctx = TestContext(object_model)
    body = object_model.get_part("cabinet_body")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    latch_bar = object_model.get_part("latch_bar")
    left_hinge = object_model.get_articulation("left_door_hinge")
    right_hinge = object_model.get_articulation("right_door_hinge")
    latch_joint = object_model.get_articulation("latch_swing")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    with ctx.pose({left_hinge: 0.0, right_hinge: 0.0, latch_joint: 0.0}):
        ctx.expect_contact(body, left_door, elem_a="left_hinge_lower", elem_b="hinge_barrel", name="left_lower_hinge_contact")
        ctx.expect_contact(body, left_door, elem_a="left_hinge_upper", elem_b="hinge_barrel", name="left_upper_hinge_contact")
        ctx.expect_contact(body, right_door, elem_a="right_hinge_lower", elem_b="hinge_barrel", name="right_lower_hinge_contact")
        ctx.expect_contact(body, right_door, elem_a="right_hinge_upper", elem_b="hinge_barrel", name="right_upper_hinge_contact")
        ctx.expect_contact(right_door, latch_bar, elem_a="latch_bracket", elem_b="pivot_sleeve", name="latch_pivot_is_mounted")
        ctx.expect_gap(
            latch_bar,
            right_door,
            axis="y",
            min_gap=0.0,
            max_gap=0.020,
            positive_elem="swing_bar",
            negative_elem="overlap_stile",
            name="latch_bar_sits_proud_of_door_face",
        )
        ctx.expect_overlap(
            left_door,
            right_door,
            axes="xz",
            min_overlap=0.018,
            elem_a="door_leaf",
            elem_b="overlap_stile",
            name="overlap_stile_covers_center_seam",
        )

        left_leaf = ctx.part_element_world_aabb(left_door, elem="door_leaf")
        right_leaf = ctx.part_element_world_aabb(right_door, elem="door_leaf")
        overlap_stile = ctx.part_element_world_aabb(right_door, elem="overlap_stile")

        equal_width = False
        equal_width_details = "missing door leaf AABB"
        if left_leaf is not None and right_leaf is not None:
            left_width = _aabb_extent(left_leaf, "x")
            right_width = _aabb_extent(right_leaf, "x")
            equal_width = abs(left_width - right_width) <= 0.001
            equal_width_details = f"left_width={left_width:.4f}, right_width={right_width:.4f}"
        ctx.check("equal_width_front_doors", equal_width, equal_width_details)

        front_alignment_ok = False
        front_alignment_details = "missing door leaf AABB"
        if left_leaf is not None and right_leaf is not None:
            left_face = _aabb_max(left_leaf, "y")
            right_face = _aabb_max(right_leaf, "y")
            front_alignment_ok = abs(left_face - FRONT_FACE_Y) <= 0.001 and abs(right_face - FRONT_FACE_Y) <= 0.001
            front_alignment_details = f"left_face_y={left_face:.4f}, right_face_y={right_face:.4f}"
        ctx.check("doors_are_flush_with_front_plane", front_alignment_ok, front_alignment_details)

        seam_gap_ok = False
        seam_gap_details = "missing door leaf AABB"
        if left_leaf is not None and right_leaf is not None:
            seam_gap = _aabb_min(right_leaf, "x") - _aabb_max(left_leaf, "x")
            seam_gap_ok = abs(seam_gap - CENTER_SEAM_GAP) <= 0.001
            seam_gap_details = f"seam_gap={seam_gap:.4f}"
        ctx.check("equal_center_gap_between_door_leaves", seam_gap_ok, seam_gap_details)

        astragal_proud_ok = False
        astragal_proud_details = "missing overlap stile or left leaf AABB"
        if left_leaf is not None and overlap_stile is not None:
            proud = _aabb_max(overlap_stile, "y") - _aabb_max(left_leaf, "y")
            astragal_proud_ok = abs(proud - ASTRAGAL_THICKNESS) <= 0.001
            astragal_proud_details = f"proud={proud:.4f}"
        ctx.check("center_overlap_step_is_flush", astragal_proud_ok, astragal_proud_details)

        body_aabb = ctx.part_world_aabb(body)
        proportion_ok = False
        proportion_details = "missing cabinet body AABB"
        if body_aabb is not None:
            body_width = _aabb_extent(body_aabb, "x")
            body_depth = _aabb_extent(body_aabb, "y")
            body_height = _aabb_extent(body_aabb, "z")
            proportion_ok = 1.15 <= body_width <= 1.25 and 0.44 <= body_depth <= 0.55 and 1.95 <= body_height <= 2.05
            proportion_details = f"width={body_width:.3f}, depth={body_depth:.3f}, height={body_height:.3f}"
        ctx.check("switchboard_floor_standing_proportions", proportion_ok, proportion_details)

    left_limits = left_hinge.motion_limits
    if left_limits is not None and left_limits.upper is not None:
        with ctx.pose({left_hinge: left_limits.upper, right_hinge: 0.0, latch_joint: 0.0}):
            ctx.fail_if_parts_overlap_in_current_pose(name="left_door_open_no_overlap")
            ctx.fail_if_isolated_parts(name="left_door_open_no_floating")
            left_leaf = ctx.part_element_world_aabb(left_door, elem="door_leaf")
            left_swings_out = left_leaf is not None and _aabb_max(left_leaf, "y") > FRONT_FACE_Y + 0.30
            left_details = "missing left door AABB"
            if left_leaf is not None:
                left_details = f"open_max_y={_aabb_max(left_leaf, 'y'):.3f}"
            ctx.check("left_door_swings_outward", left_swings_out, left_details)

    right_limits = right_hinge.motion_limits
    if right_limits is not None and right_limits.upper is not None:
        with ctx.pose({left_hinge: 0.0, right_hinge: right_limits.upper, latch_joint: LATCH_OPEN_ANGLE}):
            ctx.fail_if_parts_overlap_in_current_pose(name="right_door_open_no_overlap")
            ctx.fail_if_isolated_parts(name="right_door_open_no_floating")
            right_leaf = ctx.part_element_world_aabb(right_door, elem="door_leaf")
            right_swings_out = right_leaf is not None and _aabb_max(right_leaf, "y") > FRONT_FACE_Y + 0.30
            right_details = "missing right door AABB"
            if right_leaf is not None:
                right_details = f"open_max_y={_aabb_max(right_leaf, 'y'):.3f}"
            ctx.check("right_door_swings_outward", right_swings_out, right_details)

    engaged_bar = None
    with ctx.pose({left_hinge: 0.0, right_hinge: 0.0, latch_joint: 0.0}):
        engaged_bar = ctx.part_element_world_aabb(latch_bar, elem="swing_bar")

    latch_limits = latch_joint.motion_limits
    if latch_limits is not None and latch_limits.upper is not None:
        with ctx.pose({left_hinge: 0.0, right_hinge: 0.0, latch_joint: latch_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="latch_unlocked_no_overlap")
            ctx.fail_if_isolated_parts(name="latch_unlocked_no_floating")
            unlocked_bar = ctx.part_element_world_aabb(latch_bar, elem="swing_bar")
            latch_rotates = False
            latch_details = "missing latch AABB"
            if engaged_bar is not None and unlocked_bar is not None:
                engaged_x = _aabb_extent(engaged_bar, "x")
                engaged_z = _aabb_extent(engaged_bar, "z")
                unlocked_x = _aabb_extent(unlocked_bar, "x")
                unlocked_z = _aabb_extent(unlocked_bar, "z")
                latch_rotates = engaged_x > engaged_z and unlocked_z > unlocked_x
                latch_details = (
                    f"engaged_x={engaged_x:.3f}, engaged_z={engaged_z:.3f}, "
                    f"unlocked_x={unlocked_x:.3f}, unlocked_z={unlocked_z:.3f}"
                )
            ctx.check("latch_swings_clear_of_center_seam", latch_rotates, latch_details)

    if (
        left_limits is not None
        and left_limits.upper is not None
        and right_limits is not None
        and right_limits.upper is not None
        and latch_limits is not None
        and latch_limits.upper is not None
    ):
        with ctx.pose({left_hinge: left_limits.upper, right_hinge: right_limits.upper, latch_joint: latch_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="all_articulations_open_no_overlap")
            ctx.fail_if_isolated_parts(name="all_articulations_open_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
