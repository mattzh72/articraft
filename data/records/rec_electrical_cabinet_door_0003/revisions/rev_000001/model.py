from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os

try:
    os.getcwd()
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

BODY_WIDTH = 0.320
BODY_HEIGHT = 0.420
BODY_DEPTH = 0.120
STEEL_THICKNESS = 0.004

BODY_HALF_WIDTH = BODY_WIDTH / 2.0
BODY_HALF_HEIGHT = BODY_HEIGHT / 2.0
OPENING_RIGHT_Y = BODY_HALF_WIDTH - STEEL_THICKNESS
OPENING_TOP_Z = BODY_HALF_HEIGHT - STEEL_THICKNESS

PIN_RADIUS = 0.0045
KNUCKLE_RADIUS = 0.0075
KNUCKLE_LENGTH = 0.112
KNUCKLE_Z_OFFSET = 0.130

HINGE_X = BODY_DEPTH + 0.006
HINGE_Y = -BODY_HALF_WIDTH - PIN_RADIUS - 0.001

DOOR_THICKNESS = 0.003
DOOR_RETURN_DEPTH = 0.015
DOOR_RETURN_WIDTH = 0.012
DOOR_TOP_OVERLAY = 0.012
DOOR_RIGHT_OVERLAY = 0.014
DOOR_LEFT_PANEL_OFFSET = 0.012

DOOR_RIGHT_EDGE = BODY_HALF_WIDTH + DOOR_RIGHT_OVERLAY
DOOR_TOP_EDGE = BODY_HALF_HEIGHT + DOOR_TOP_OVERLAY
DOOR_HEIGHT = DOOR_TOP_EDGE * 2.0
DOOR_TOTAL_WIDTH = DOOR_RIGHT_EDGE - HINGE_Y
OUTER_PANEL_WIDTH = DOOR_RIGHT_EDGE - (HINGE_Y + DOOR_LEFT_PANEL_OFFSET)
OUTER_PANEL_CENTER_Y = DOOR_LEFT_PANEL_OFFSET + (OUTER_PANEL_WIDTH / 2.0)
DOOR_PANEL_CENTER_X = BODY_DEPTH + (DOOR_THICKNESS / 2.0)
PANEL_LOCAL_X = DOOR_PANEL_CENTER_X - HINGE_X

TOP_BOTTOM_RETURN_WIDTH = OPENING_RIGHT_Y - HINGE_Y - DOOR_LEFT_PANEL_OFFSET
TOP_BOTTOM_RETURN_CENTER_Y = DOOR_LEFT_PANEL_OFFSET + (TOP_BOTTOM_RETURN_WIDTH / 2.0)
TOP_RETURN_CENTER_Z = OPENING_TOP_Z - (DOOR_RETURN_WIDTH / 2.0)
BOTTOM_RETURN_CENTER_Z = -TOP_RETURN_CENTER_Z
RIGHT_RETURN_CENTER_Y = OPENING_RIGHT_Y - (DOOR_RETURN_WIDTH / 2.0) - HINGE_Y
RIGHT_RETURN_HEIGHT = (2.0 * OPENING_TOP_Z) - (2.0 * DOOR_RETURN_WIDTH)
RETURN_LOCAL_X = (BODY_DEPTH - (DOOR_RETURN_DEPTH / 2.0)) - HINGE_X

BODY_HINGE_LEAF_DEPTH = 0.023
BODY_HINGE_LEAF_THICKNESS = 0.002
BODY_HINGE_LEAF_HEIGHT = BODY_HEIGHT - 0.040
BODY_HINGE_LEAF_CENTER_X = 0.1105
BODY_HINGE_LEAF_CENTER_Y = -BODY_HALF_WIDTH - (BODY_HINGE_LEAF_THICKNESS / 2.0)

PIN_TAB_DEPTH = 0.0075
PIN_TAB_WIDTH = 0.011
PIN_TAB_HEIGHT = 0.020
PIN_TAB_CENTER_X = (HINGE_X - PIN_RADIUS) - (PIN_TAB_DEPTH / 2.0)
PIN_TAB_CENTER_Y = -BODY_HALF_WIDTH - (PIN_TAB_WIDTH / 2.0)
UPPER_PIN_TAB_CENTER_Z = 0.200
LOWER_PIN_TAB_CENTER_Z = -0.200

DOOR_HINGE_LEAF_DEPTH = 0.006
DOOR_HINGE_LEAF_THICKNESS = 0.006
DOOR_HINGE_LEAF_HEIGHT = BODY_HEIGHT - 0.036
DOOR_HINGE_LEAF_CENTER_X = BODY_DEPTH + (DOOR_HINGE_LEAF_DEPTH / 2.0)
DOOR_HINGE_LEAF_LOCAL_X = DOOR_HINGE_LEAF_CENTER_X - HINGE_X
DOOR_HINGE_LEAF_LOCAL_Y = 0.009


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="surface_mount_enclosure")

    enclosure_steel = model.material("enclosure_steel", rgba=(0.62, 0.63, 0.66, 1.0))
    door_steel = model.material("door_steel", rgba=(0.54, 0.56, 0.59, 1.0))
    pin_finish = model.material("hinge_pin_finish", rgba=(0.22, 0.23, 0.24, 1.0))

    body = model.part("body")
    body.visual(
        Box((STEEL_THICKNESS, BODY_WIDTH, BODY_HEIGHT)),
        origin=Origin(xyz=(STEEL_THICKNESS / 2.0, 0.0, 0.0)),
        material=enclosure_steel,
        name="back_panel",
    )
    body.visual(
        Box((BODY_DEPTH, BODY_WIDTH - (2.0 * STEEL_THICKNESS), STEEL_THICKNESS)),
        origin=Origin(xyz=(BODY_DEPTH / 2.0, 0.0, BODY_HALF_HEIGHT - (STEEL_THICKNESS / 2.0))),
        material=enclosure_steel,
        name="top_wall",
    )
    body.visual(
        Box((BODY_DEPTH, BODY_WIDTH - (2.0 * STEEL_THICKNESS), STEEL_THICKNESS)),
        origin=Origin(xyz=(BODY_DEPTH / 2.0, 0.0, -BODY_HALF_HEIGHT + (STEEL_THICKNESS / 2.0))),
        material=enclosure_steel,
        name="bottom_wall",
    )
    body.visual(
        Box((BODY_DEPTH, STEEL_THICKNESS, BODY_HEIGHT - (2.0 * STEEL_THICKNESS))),
        origin=Origin(xyz=(BODY_DEPTH / 2.0, -BODY_HALF_WIDTH + (STEEL_THICKNESS / 2.0), 0.0)),
        material=enclosure_steel,
        name="left_wall",
    )
    body.visual(
        Box((BODY_DEPTH, STEEL_THICKNESS, BODY_HEIGHT - (2.0 * STEEL_THICKNESS))),
        origin=Origin(xyz=(BODY_DEPTH / 2.0, BODY_HALF_WIDTH - (STEEL_THICKNESS / 2.0), 0.0)),
        material=enclosure_steel,
        name="right_wall",
    )
    body.visual(
        Box((BODY_HINGE_LEAF_DEPTH, BODY_HINGE_LEAF_THICKNESS, BODY_HINGE_LEAF_HEIGHT)),
        origin=Origin(xyz=(BODY_HINGE_LEAF_CENTER_X, BODY_HINGE_LEAF_CENTER_Y, 0.0)),
        material=enclosure_steel,
        name="body_hinge_leaf",
    )
    body.visual(
        Box((PIN_TAB_DEPTH, PIN_TAB_WIDTH, PIN_TAB_HEIGHT)),
        origin=Origin(xyz=(PIN_TAB_CENTER_X, PIN_TAB_CENTER_Y, UPPER_PIN_TAB_CENTER_Z)),
        material=enclosure_steel,
        name="upper_pin_tab",
    )
    body.visual(
        Box((PIN_TAB_DEPTH, PIN_TAB_WIDTH, PIN_TAB_HEIGHT)),
        origin=Origin(xyz=(PIN_TAB_CENTER_X, PIN_TAB_CENTER_Y, LOWER_PIN_TAB_CENTER_Z)),
        material=enclosure_steel,
        name="lower_pin_tab",
    )
    body.visual(
        Cylinder(radius=PIN_RADIUS, length=DOOR_HEIGHT),
        origin=Origin(xyz=(HINGE_X, HINGE_Y, 0.0)),
        material=pin_finish,
        name="hinge_pin",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_DEPTH, BODY_WIDTH, BODY_HEIGHT)),
        mass=5.8,
        origin=Origin(xyz=(BODY_DEPTH / 2.0, 0.0, 0.0)),
    )

    door = model.part("door")
    door.visual(
        Box((DOOR_THICKNESS, OUTER_PANEL_WIDTH, DOOR_HEIGHT)),
        origin=Origin(xyz=(PANEL_LOCAL_X, OUTER_PANEL_CENTER_Y, 0.0)),
        material=door_steel,
        name="outer_panel",
    )
    door.visual(
        Box((DOOR_RETURN_DEPTH, TOP_BOTTOM_RETURN_WIDTH, DOOR_RETURN_WIDTH)),
        origin=Origin(xyz=(RETURN_LOCAL_X, TOP_BOTTOM_RETURN_CENTER_Y, TOP_RETURN_CENTER_Z)),
        material=door_steel,
        name="top_return",
    )
    door.visual(
        Box((DOOR_RETURN_DEPTH, TOP_BOTTOM_RETURN_WIDTH, DOOR_RETURN_WIDTH)),
        origin=Origin(xyz=(RETURN_LOCAL_X, TOP_BOTTOM_RETURN_CENTER_Y, BOTTOM_RETURN_CENTER_Z)),
        material=door_steel,
        name="bottom_return",
    )
    door.visual(
        Box((DOOR_RETURN_DEPTH, DOOR_RETURN_WIDTH, RIGHT_RETURN_HEIGHT)),
        origin=Origin(xyz=(RETURN_LOCAL_X, RIGHT_RETURN_CENTER_Y, 0.0)),
        material=door_steel,
        name="right_return",
    )
    door.visual(
        Box((DOOR_HINGE_LEAF_DEPTH, DOOR_HINGE_LEAF_THICKNESS, DOOR_HINGE_LEAF_HEIGHT)),
        origin=Origin(xyz=(DOOR_HINGE_LEAF_LOCAL_X, DOOR_HINGE_LEAF_LOCAL_Y, 0.0)),
        material=door_steel,
        name="hinge_leaf",
    )
    door.visual(
        Cylinder(radius=KNUCKLE_RADIUS, length=KNUCKLE_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, KNUCKLE_Z_OFFSET)),
        material=door_steel,
        name="upper_knuckle",
    )
    door.visual(
        Cylinder(radius=KNUCKLE_RADIUS, length=KNUCKLE_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, -KNUCKLE_Z_OFFSET)),
        material=door_steel,
        name="lower_knuckle",
    )
    door.inertial = Inertial.from_geometry(
        Box((DOOR_RETURN_DEPTH + DOOR_THICKNESS, DOOR_TOTAL_WIDTH, DOOR_HEIGHT)),
        mass=2.2,
        origin=Origin(
            xyz=(
                ((DOOR_PANEL_CENTER_X - (DOOR_RETURN_DEPTH / 2.0)) - HINGE_X),
                DOOR_TOTAL_WIDTH / 2.0,
                0.0,
            )
        ),
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(HINGE_X, HINGE_Y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(108.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    door_hinge = object_model.get_articulation("door_hinge")

    left_wall = body.get_visual("left_wall")
    top_wall = body.get_visual("top_wall")
    bottom_wall = body.get_visual("bottom_wall")
    right_wall = body.get_visual("right_wall")
    body_hinge_leaf = body.get_visual("body_hinge_leaf")
    upper_pin_tab = body.get_visual("upper_pin_tab")
    lower_pin_tab = body.get_visual("lower_pin_tab")
    hinge_pin = body.get_visual("hinge_pin")

    outer_panel = door.get_visual("outer_panel")
    top_return = door.get_visual("top_return")
    bottom_return = door.get_visual("bottom_return")
    right_return = door.get_visual("right_return")
    hinge_leaf = door.get_visual("hinge_leaf")
    upper_knuckle = door.get_visual("upper_knuckle")
    lower_knuckle = door.get_visual("lower_knuckle")

    ctx.allow_overlap(
        door,
        body,
        elem_a=upper_knuckle,
        elem_b=hinge_pin,
        reason="The hinge barrel is represented by a solid knuckle cylinder around a solid hinge pin instead of a hollow rolled sleeve.",
    )
    ctx.allow_overlap(
        door,
        body,
        elem_a=lower_knuckle,
        elem_b=hinge_pin,
        reason="The hinge barrel is represented by a solid knuckle cylinder around a solid hinge pin instead of a hollow rolled sleeve.",
    )

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    limits = door_hinge.motion_limits
    axis = door_hinge.axis
    ctx.check(
        "door_hinge_is_vertical_revolute",
        door_hinge.articulation_type == ArticulationType.REVOLUTE
        and abs(axis[0]) < 1e-9
        and abs(axis[1]) < 1e-9
        and abs(abs(axis[2]) - 1.0) < 1e-9,
        details=f"expected a vertical revolute hinge axis, got type={door_hinge.articulation_type} axis={axis}",
    )
    ctx.check(
        "door_hinge_limit_is_realistic",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and abs(limits.lower) <= 1e-9
        and math.radians(105.0) <= limits.upper <= math.radians(120.0),
        details=f"expected a realistic enclosure swing near 110 degrees, got {limits}",
    )

    ctx.expect_overlap(
        door,
        body,
        axes="yz",
        min_overlap=0.30,
        name="full_front_door_reads_as_front_cover",
    )
    ctx.expect_gap(
        door,
        body,
        axis="x",
        positive_elem=outer_panel,
        negative_elem=top_wall,
        min_gap=0.0,
        max_gap=0.001,
        name="outer_panel_sits_close_to_front_plane",
    )
    ctx.expect_gap(
        body,
        door,
        axis="z",
        positive_elem=top_wall,
        negative_elem=top_return,
        min_gap=0.0,
        max_gap=0.001,
        name="top_return_tucks_under_top_wall",
    )
    ctx.expect_gap(
        door,
        body,
        axis="z",
        positive_elem=bottom_return,
        negative_elem=bottom_wall,
        min_gap=0.0,
        max_gap=0.001,
        name="bottom_return_tucks_above_bottom_wall",
    )
    ctx.expect_gap(
        body,
        door,
        axis="y",
        positive_elem=right_wall,
        negative_elem=right_return,
        min_gap=0.0,
        max_gap=0.001,
        name="right_return_seats_inside_latch_side_wall",
    )
    ctx.expect_contact(
        body,
        body,
        elem_a=body_hinge_leaf,
        elem_b=left_wall,
        name="body_hinge_leaf_is_mounted_to_left_side",
    )
    ctx.expect_contact(
        body,
        body,
        elem_a=upper_pin_tab,
        elem_b=hinge_pin,
        name="upper_pin_tab_carries_hinge_pin",
    )
    ctx.expect_contact(
        body,
        body,
        elem_a=lower_pin_tab,
        elem_b=hinge_pin,
        name="lower_pin_tab_carries_hinge_pin",
    )
    ctx.expect_contact(
        body,
        body,
        elem_a=upper_pin_tab,
        elem_b=body_hinge_leaf,
        name="upper_pin_tab_is_welded_to_hinge_leaf",
    )
    ctx.expect_contact(
        body,
        body,
        elem_a=lower_pin_tab,
        elem_b=body_hinge_leaf,
        name="lower_pin_tab_is_welded_to_hinge_leaf",
    )
    ctx.expect_contact(
        door,
        door,
        elem_a=hinge_leaf,
        elem_b=outer_panel,
        name="door_hinge_leaf_is_attached_to_door_skin",
    )
    ctx.expect_overlap(
        door,
        body,
        axes="z",
        elem_a=hinge_leaf,
        elem_b=body_hinge_leaf,
        min_overlap=0.34,
        name="piano_hinge_leafs_run_most_of_door_height",
    )
    ctx.expect_contact(
        door,
        body,
        elem_a=upper_knuckle,
        elem_b=hinge_pin,
        name="upper_knuckle_runs_on_hinge_pin",
    )
    ctx.expect_contact(
        door,
        body,
        elem_a=lower_knuckle,
        elem_b=hinge_pin,
        name="lower_knuckle_runs_on_hinge_pin",
    )

    with ctx.pose({door_hinge: math.radians(95.0)}):
        ctx.expect_gap(
            body,
            door,
            axis="y",
            positive_elem=right_wall,
            negative_elem=outer_panel,
            min_gap=0.11,
            name="open_door_swings_clear_of_front_opening",
        )
        ctx.expect_contact(
            door,
            body,
            elem_a=upper_knuckle,
            elem_b=hinge_pin,
            name="upper_knuckle_stays_on_pin_when_open",
        )
        ctx.expect_contact(
            door,
            body,
            elem_a=lower_knuckle,
            elem_b=hinge_pin,
            name="lower_knuckle_stays_on_pin_when_open",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="door_open_pose_has_no_unplanned_overlaps")
        ctx.fail_if_isolated_parts(name="door_open_pose_has_no_floating_parts")

    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({door_hinge: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="door_hinge_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="door_hinge_lower_no_floating")
        with ctx.pose({door_hinge: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="door_hinge_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="door_hinge_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
