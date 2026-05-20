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


BODY_WIDTH = 0.44
BODY_DEPTH = 0.36
BODY_HEIGHT = 0.58
WALL = 0.018
FRONT_FRAME_DEPTH = 0.024
FRONT_JAMB_WIDTH = 0.020

PLINTH_HEIGHT = 0.20

HINGE_AXIS_X = -0.222
HINGE_AXIS_Y = BODY_DEPTH * 0.5
DOOR_BASE_Z = 0.018
DOOR_WIDTH = 0.412
DOOR_HEIGHT = 0.544
DOOR_THICKNESS = 0.022

DOOR_SWING_MAX = math.radians(110.0)
HANDLE_SWING_MAX = math.radians(65.0)


def _y_axis_origin(*, xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(math.pi * 0.5, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_parcel_mailbox")

    body_paint = model.material("body_paint", rgba=(0.17, 0.18, 0.20, 1.0))
    plinth_paint = model.material("plinth_paint", rgba=(0.10, 0.10, 0.11, 1.0))
    handle_metal = model.material("handle_metal", rgba=(0.72, 0.74, 0.76, 1.0))
    seal_black = model.material("seal_black", rgba=(0.08, 0.08, 0.09, 1.0))

    plinth = model.part("plinth")
    plinth.visual(
        Box((0.32, 0.24, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        material=plinth_paint,
        name="base_plate",
    )
    plinth.visual(
        Box((0.14, 0.14, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=plinth_paint,
        name="support_post",
    )
    plinth.visual(
        Box((0.20, 0.20, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        material=plinth_paint,
        name="plinth_collar",
    )
    plinth.visual(
        Box((0.26, 0.20, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.19)),
        material=plinth_paint,
        name="top_cap",
    )
    plinth.inertial = Inertial.from_geometry(
        Box((0.32, 0.24, PLINTH_HEIGHT)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, PLINTH_HEIGHT * 0.5)),
    )

    body = model.part("mailbox_body")
    body.visual(
        Box((BODY_WIDTH, WALL, BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, -(BODY_DEPTH * 0.5) + (WALL * 0.5), BODY_HEIGHT * 0.5)),
        material=body_paint,
        name="back_panel",
    )
    body.visual(
        Box((WALL, BODY_DEPTH - FRONT_FRAME_DEPTH, BODY_HEIGHT)),
        origin=Origin(
            xyz=(
                -(BODY_WIDTH * 0.5) + (WALL * 0.5),
                -(FRONT_FRAME_DEPTH * 0.5),
                BODY_HEIGHT * 0.5,
            )
        ),
        material=body_paint,
        name="left_side_shell",
    )
    body.visual(
        Box((WALL, BODY_DEPTH - FRONT_FRAME_DEPTH, BODY_HEIGHT)),
        origin=Origin(
            xyz=(
                (BODY_WIDTH * 0.5) - (WALL * 0.5),
                -(FRONT_FRAME_DEPTH * 0.5),
                BODY_HEIGHT * 0.5,
            )
        ),
        material=body_paint,
        name="right_side_shell",
    )
    body.visual(
        Box((FRONT_JAMB_WIDTH, FRONT_FRAME_DEPTH, DOOR_HEIGHT)),
        origin=Origin(
            xyz=(
                -(BODY_WIDTH * 0.5) + WALL + (FRONT_JAMB_WIDTH * 0.5),
                (BODY_DEPTH * 0.5) - (FRONT_FRAME_DEPTH * 0.5),
                DOOR_BASE_Z + (DOOR_HEIGHT * 0.5),
            )
        ),
        material=body_paint,
        name="left_jamb",
    )
    body.visual(
        Box((FRONT_JAMB_WIDTH, FRONT_FRAME_DEPTH, DOOR_HEIGHT)),
        origin=Origin(
            xyz=(
                (BODY_WIDTH * 0.5) - WALL - (FRONT_JAMB_WIDTH * 0.5),
                (BODY_DEPTH * 0.5) - (FRONT_FRAME_DEPTH * 0.5),
                DOOR_BASE_Z + (DOOR_HEIGHT * 0.5),
            )
        ),
        material=body_paint,
        name="right_jamb",
    )
    body.visual(
        Box((BODY_WIDTH - (2.0 * WALL), BODY_DEPTH, WALL)),
        origin=Origin(xyz=(0.0, 0.0, WALL * 0.5)),
        material=body_paint,
        name="floor_panel",
    )
    body.visual(
        Box((BODY_WIDTH - (2.0 * WALL), BODY_DEPTH, WALL)),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT - (WALL * 0.5))),
        material=body_paint,
        name="roof_panel",
    )
    body.visual(
        Box((0.020, 0.016, 0.22)),
        origin=Origin(xyz=(0.194, 0.172, 0.290)),
        material=seal_black,
        name="latch_stop",
    )
    body.visual(
        Cylinder(radius=0.008, length=0.086),
        origin=Origin(xyz=(HINGE_AXIS_X, HINGE_AXIS_Y, 0.067),),
        material=body_paint,
        name="lower_knuckle",
    )
    body.visual(
        Cylinder(radius=0.008, length=0.098),
        origin=Origin(xyz=(HINGE_AXIS_X, HINGE_AXIS_Y, 0.495)),
        material=body_paint,
        name="upper_knuckle",
    )
    body.visual(
        Box((0.022, 0.010, 0.086)),
        origin=Origin(xyz=(-0.205, 0.168, 0.067)),
        material=body_paint,
        name="lower_hinge_leaf",
    )
    body.visual(
        Box((0.022, 0.010, 0.098)),
        origin=Origin(xyz=(-0.205, 0.168, 0.495)),
        material=body_paint,
        name="upper_hinge_leaf",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT * 0.5)),
    )

    door = model.part("parcel_door")
    door.visual(
        Box((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(0.217, DOOR_THICKNESS * 0.5, DOOR_HEIGHT * 0.5)),
        material=body_paint,
        name="door_skin",
    )
    door.visual(
        Box((0.018, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(0.017, DOOR_THICKNESS * 0.5, DOOR_HEIGHT * 0.5)),
        material=body_paint,
        name="hinge_stile",
    )
    door.visual(
        Box((0.014, 0.010, 0.452)),
        origin=Origin(xyz=(0.404, 0.005, DOOR_HEIGHT * 0.5)),
        material=seal_black,
        name="free_edge_stile",
    )
    door.visual(
        Box((0.348, 0.010, 0.460)),
        origin=Origin(xyz=(0.216, 0.005, DOOR_HEIGHT * 0.5)),
        material=seal_black,
        name="inner_reinforcement",
    )
    door.visual(
        Cylinder(radius=0.008, length=0.334),
        origin=Origin(xyz=(0.0, 0.0, 0.260)),
        material=body_paint,
        name="door_knuckle",
    )
    door.inertial = Inertial.from_geometry(
        Box((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        mass=9.0,
        origin=Origin(xyz=(0.217, DOOR_THICKNESS * 0.5, DOOR_HEIGHT * 0.5)),
    )

    handle = model.part("door_handle")
    handle.visual(
        Cylinder(radius=0.022, length=0.010),
        origin=_y_axis_origin(xyz=(0.0, 0.0, 0.0)),
        material=handle_metal,
        name="hub",
    )
    handle.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=_y_axis_origin(xyz=(0.0, 0.009, 0.0)),
        material=handle_metal,
        name="spindle",
    )
    handle.visual(
        Box((0.075, 0.008, 0.020)),
        origin=Origin(xyz=(0.0, 0.015, 0.0)),
        material=handle_metal,
        name="turn_bar",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.075, 0.024, 0.030)),
        mass=0.5,
        origin=Origin(xyz=(0.0, 0.012, 0.0)),
    )

    model.articulation(
        "plinth_to_body",
        ArticulationType.FIXED,
        parent=plinth,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, PLINTH_HEIGHT)),
    )
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(HINGE_AXIS_X, HINGE_AXIS_Y, DOOR_BASE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=1.2,
            lower=0.0,
            upper=DOOR_SWING_MAX,
        ),
    )
    model.articulation(
        "door_to_handle",
        ArticulationType.REVOLUTE,
        parent=door,
        child=handle,
        origin=Origin(xyz=(0.366, 0.027, DOOR_HEIGHT * 0.5)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=-HANDLE_SWING_MAX,
            upper=HANDLE_SWING_MAX,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    plinth = object_model.get_part("plinth")
    body = object_model.get_part("mailbox_body")
    door = object_model.get_part("parcel_door")
    handle = object_model.get_part("door_handle")
    door_hinge = object_model.get_articulation("body_to_door")
    handle_pivot = object_model.get_articulation("door_to_handle")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "door_hinge_axis_is_vertical",
        tuple(door_hinge.axis) == (0.0, 0.0, 1.0),
        f"door hinge axis was {door_hinge.axis}",
    )
    ctx.check(
        "handle_axis_faces_forward",
        tuple(handle_pivot.axis) == (0.0, 1.0, 0.0),
        f"handle pivot axis was {handle_pivot.axis}",
    )
    ctx.check(
        "door_has_wide_swing_range",
        door_hinge.motion_limits is not None and door_hinge.motion_limits.upper is not None and door_hinge.motion_limits.upper >= math.radians(100.0),
        "door should open beyond 100 degrees",
    )

    ctx.expect_gap(body, plinth, axis="z", max_gap=0.001, max_penetration=0.0)
    ctx.expect_overlap(body, plinth, axes="xy", min_overlap=0.18)

    with ctx.pose({door_hinge: 0.0, handle_pivot: 0.0}):
        ctx.expect_gap(
            door,
            body,
            axis="y",
            positive_elem="door_skin",
            negative_elem="latch_stop",
            max_gap=0.001,
            max_penetration=0.0,
        )
        ctx.expect_overlap(door, body, axes="xz", min_overlap=0.20, elem_a="door_skin")
        ctx.expect_gap(
            body,
            door,
            axis="z",
            positive_elem="upper_knuckle",
            negative_elem="door_knuckle",
            max_gap=0.003,
            max_penetration=0.0,
        )
        ctx.expect_gap(
            door,
            body,
            axis="z",
            positive_elem="door_knuckle",
            negative_elem="lower_knuckle",
            max_gap=0.003,
            max_penetration=0.0,
        )
        ctx.expect_contact(handle, door, elem_a="hub", elem_b="door_skin")

    free_edge_closed = ctx.part_element_world_aabb(door, elem="free_edge_stile")
    with ctx.pose({door_hinge: math.radians(95.0)}):
        ctx.expect_gap(
            body,
            door,
            axis="z",
            positive_elem="upper_knuckle",
            negative_elem="door_knuckle",
            max_gap=0.003,
            max_penetration=0.0,
        )
        ctx.expect_gap(
            door,
            body,
            axis="z",
            positive_elem="door_knuckle",
            negative_elem="lower_knuckle",
            max_gap=0.003,
            max_penetration=0.0,
        )
        ctx.expect_gap(
            door,
            body,
            axis="y",
            positive_elem="free_edge_stile",
            min_gap=0.10,
        )
        free_edge_open = ctx.part_element_world_aabb(door, elem="free_edge_stile")
        moved_outward = False
        if free_edge_closed is not None and free_edge_open is not None:
            moved_outward = free_edge_open[1][1] > free_edge_closed[1][1] + 0.12
        ctx.check(
            "door_swings_outward",
            moved_outward,
            "door free edge did not move outward in front of the box",
        )

    turn_bar_rest = ctx.part_element_world_aabb(handle, elem="turn_bar")
    with ctx.pose({handle_pivot: math.radians(50.0)}):
        ctx.expect_contact(handle, door, elem_a="hub", elem_b="door_skin")
        turn_bar_turned = ctx.part_element_world_aabb(handle, elem="turn_bar")
        turned = False
        if turn_bar_rest is not None and turn_bar_turned is not None:
            rest_height = turn_bar_rest[1][2] - turn_bar_rest[0][2]
            turned_height = turn_bar_turned[1][2] - turn_bar_turned[0][2]
            turned = turned_height > rest_height + 0.02
        ctx.check(
            "handle_rotates_about_its_pivot",
            turned,
            "turn bar did not change orientation when the handle articulation moved",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
