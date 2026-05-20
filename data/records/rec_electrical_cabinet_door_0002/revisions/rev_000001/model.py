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

BODY_WIDTH = 0.60
BODY_DEPTH = 0.25
BODY_HEIGHT = 0.80
STEEL_THICKNESS = 0.0025
FRAME_WIDTH = 0.03

DOOR_CLEARANCE = 0.0015
HINGE_AXIS_OFFSET = 0.006
HINGE_AXIS_X = -BODY_WIDTH / 2.0 - HINGE_AXIS_OFFSET
HINGE_AXIS_Y = BODY_DEPTH / 2.0 + DOOR_CLEARANCE

HINGE_RADIUS = 0.0065
HINGE_GROUP_HEIGHT = 0.14
BODY_KNUCKLE_LENGTH = 0.035
DOOR_KNUCKLE_LENGTH = 0.045
HINGE_ZS = (0.26, 0.0, -0.26)

DOOR_WIDTH = BODY_WIDTH + 2.0 * HINGE_AXIS_OFFSET
DOOR_HEIGHT = BODY_HEIGHT - 0.004
DOOR_PANEL_DEPTH = 0.026
DOOR_SKIN_THICKNESS = 0.0025
DOOR_RETURN_WIDTH = 0.024
DOOR_LEFT_RETURN_WIDTH = 0.014
DOOR_LATCH_STILE_WIDTH = 0.034
DOOR_RAISED_PANEL_WIDTH = DOOR_WIDTH - 0.19
DOOR_RAISED_PANEL_HEIGHT = DOOR_HEIGHT - 0.18
DOOR_PANEL_CENTER_X = DOOR_WIDTH / 2.0 - HINGE_AXIS_OFFSET
DOOR_FRONT_SKIN_CENTER_Y = DOOR_PANEL_DEPTH - DOOR_SKIN_THICKNESS / 2.0
DOOR_LEFT_RETURN_START_X = HINGE_RADIUS
DOOR_LEFT_RETURN_CENTER_X = DOOR_LEFT_RETURN_START_X + DOOR_LEFT_RETURN_WIDTH / 2.0
DOOR_OUTER_RIGHT_X = DOOR_WIDTH - HINGE_AXIS_OFFSET
DOOR_INNER_LEFT_X = DOOR_LEFT_RETURN_START_X + DOOR_LEFT_RETURN_WIDTH
DOOR_INNER_RIGHT_X = DOOR_OUTER_RIGHT_X - DOOR_LATCH_STILE_WIDTH
DOOR_RIGHT_RETURN_CENTER_X = DOOR_OUTER_RIGHT_X - DOOR_LATCH_STILE_WIDTH / 2.0
DOOR_TOP_BOTTOM_CENTER_X = (DOOR_INNER_LEFT_X + DOOR_INNER_RIGHT_X) / 2.0
DOOR_TOP_BOTTOM_WIDTH = DOOR_INNER_RIGHT_X - DOOR_INNER_LEFT_X

HANDLE_X = DOOR_OUTER_RIGHT_X - 0.048
HANDLE_PLATE_SIZE = (0.055, 0.006, 0.19)
HANDLE_BOSS_RADIUS = 0.011
HANDLE_BOSS_LENGTH = 0.007
HANDLE_GRIP_SIZE = (0.018, 0.018, 0.13)
HANDLE_HUB_RADIUS = 0.010
HANDLE_HUB_LENGTH = 0.010
HANDLE_PLATE_CENTER_Y = DOOR_PANEL_DEPTH + HANDLE_PLATE_SIZE[1] / 2.0
HANDLE_BOSS_CENTER_Y = DOOR_PANEL_DEPTH + HANDLE_PLATE_SIZE[1] + HANDLE_BOSS_LENGTH / 2.0
HANDLE_PIVOT_Y = DOOR_PANEL_DEPTH + HANDLE_PLATE_SIZE[1] + HANDLE_BOSS_LENGTH


def _box_dims(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
    return tuple(aabb[1][index] - aabb[0][index] for index in range(3))


def _add_hinge_set(body, door, z_center: float, name_prefix: str, steel, hardware) -> None:
    body_leaf_depth = 0.022
    door_leaf_depth = 0.018

    body.visual(
        Box((0.014, body_leaf_depth, HINGE_GROUP_HEIGHT)),
        origin=Origin(
            xyz=(
                -BODY_WIDTH / 2.0 - 0.014 / 2.0,
                HINGE_AXIS_Y - HINGE_RADIUS - body_leaf_depth / 2.0,
                z_center,
            )
        ),
        material=steel,
        name=f"{name_prefix}_body_leaf",
    )
    door.visual(
        Box((0.01, door_leaf_depth, HINGE_GROUP_HEIGHT)),
        origin=Origin(
            xyz=(
                HINGE_RADIUS + 0.01 / 2.0,
                HINGE_RADIUS + door_leaf_depth / 2.0,
                z_center,
            )
        ),
        material=steel,
        name=f"{name_prefix}_door_leaf",
    )

    knuckle_offset = DOOR_KNUCKLE_LENGTH / 2.0 + BODY_KNUCKLE_LENGTH / 2.0
    for suffix, z_shift in (("top", knuckle_offset), ("bottom", -knuckle_offset)):
        body.visual(
            Cylinder(radius=HINGE_RADIUS, length=BODY_KNUCKLE_LENGTH),
            origin=Origin(xyz=(HINGE_AXIS_X, HINGE_AXIS_Y, z_center + z_shift)),
            material=hardware,
            name=f"{name_prefix}_body_knuckle_{suffix}",
        )

    door.visual(
        Cylinder(radius=HINGE_RADIUS, length=DOOR_KNUCKLE_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, z_center)),
        material=hardware,
        name=f"{name_prefix}_door_knuckle",
    )


def _build_panel_body(model: ArticulatedObject):
    steel = model.material("steel_gray", rgba=(0.47, 0.49, 0.52, 1.0))
    door_steel = model.material("door_gray", rgba=(0.67, 0.69, 0.72, 1.0))
    hardware = model.material("hardware", rgba=(0.74, 0.76, 0.79, 1.0))
    mount_plate = model.material("mount_plate", rgba=(0.82, 0.84, 0.86, 1.0))
    handle_black = model.material("handle_black", rgba=(0.1, 0.11, 0.12, 1.0))
    gasket = model.material("gasket", rgba=(0.18, 0.20, 0.21, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_WIDTH, STEEL_THICKNESS, BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, -BODY_DEPTH / 2.0 + STEEL_THICKNESS / 2.0, 0.0)),
        material=steel,
        name="back_panel",
    )
    body.visual(
        Box((STEEL_THICKNESS, BODY_DEPTH, BODY_HEIGHT)),
        origin=Origin(xyz=(-BODY_WIDTH / 2.0 + STEEL_THICKNESS / 2.0, 0.0, 0.0)),
        material=steel,
        name="left_wall",
    )
    body.visual(
        Box((STEEL_THICKNESS, BODY_DEPTH, BODY_HEIGHT)),
        origin=Origin(xyz=(BODY_WIDTH / 2.0 - STEEL_THICKNESS / 2.0, 0.0, 0.0)),
        material=steel,
        name="right_wall",
    )
    body.visual(
        Box((BODY_WIDTH - 2.0 * STEEL_THICKNESS, BODY_DEPTH, STEEL_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT / 2.0 - STEEL_THICKNESS / 2.0)),
        material=steel,
        name="top_wall",
    )
    body.visual(
        Box((BODY_WIDTH - 2.0 * STEEL_THICKNESS, BODY_DEPTH, STEEL_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, -BODY_HEIGHT / 2.0 + STEEL_THICKNESS / 2.0)),
        material=steel,
        name="bottom_wall",
    )

    frame_y = BODY_DEPTH / 2.0 - STEEL_THICKNESS / 2.0
    body.visual(
        Box((FRAME_WIDTH, STEEL_THICKNESS, BODY_HEIGHT)),
        origin=Origin(xyz=(-BODY_WIDTH / 2.0 + FRAME_WIDTH / 2.0, frame_y, 0.0)),
        material=steel,
        name="front_frame_left",
    )
    body.visual(
        Box((FRAME_WIDTH, STEEL_THICKNESS, BODY_HEIGHT)),
        origin=Origin(xyz=(BODY_WIDTH / 2.0 - FRAME_WIDTH / 2.0, frame_y, 0.0)),
        material=steel,
        name="front_frame_right",
    )
    body.visual(
        Box((BODY_WIDTH - 2.0 * FRAME_WIDTH, STEEL_THICKNESS, FRAME_WIDTH)),
        origin=Origin(xyz=(0.0, frame_y, BODY_HEIGHT / 2.0 - FRAME_WIDTH / 2.0)),
        material=steel,
        name="front_frame_top",
    )
    body.visual(
        Box((BODY_WIDTH - 2.0 * FRAME_WIDTH, STEEL_THICKNESS, FRAME_WIDTH)),
        origin=Origin(xyz=(0.0, frame_y, -BODY_HEIGHT / 2.0 + FRAME_WIDTH / 2.0)),
        material=steel,
        name="front_frame_bottom",
    )
    body.visual(
        Box((BODY_WIDTH - 0.14, 0.0015, BODY_HEIGHT - 0.16)),
        origin=Origin(
            xyz=(
                0.0,
                -BODY_DEPTH / 2.0 + STEEL_THICKNESS + 0.0015 / 2.0,
                0.0,
            )
        ),
        material=mount_plate,
        name="mounting_plate",
    )
    body.visual(
        Box((0.018, 0.014, 0.22)),
        origin=Origin(
            xyz=(
                BODY_WIDTH / 2.0 - FRAME_WIDTH / 2.0,
                frame_y - 0.007,
                0.0,
            )
        ),
        material=hardware,
        name="latch_strike",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)),
        mass=24.0,
        origin=Origin(),
    )

    door = model.part("door")
    door.visual(
        Box((DOOR_WIDTH, DOOR_SKIN_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(DOOR_PANEL_CENTER_X, DOOR_FRONT_SKIN_CENTER_Y, 0.0)),
        material=door_steel,
        name="front_skin",
    )
    door.visual(
        Box((DOOR_LEFT_RETURN_WIDTH, DOOR_PANEL_DEPTH, DOOR_HEIGHT - 2.0 * DOOR_RETURN_WIDTH)),
        origin=Origin(
            xyz=(
                DOOR_LEFT_RETURN_CENTER_X,
                DOOR_PANEL_DEPTH / 2.0,
                0.0,
            )
        ),
        material=door_steel,
        name="left_return",
    )
    door.visual(
        Box((DOOR_LATCH_STILE_WIDTH, DOOR_PANEL_DEPTH, DOOR_HEIGHT - 2.0 * DOOR_RETURN_WIDTH)),
        origin=Origin(
            xyz=(
                DOOR_RIGHT_RETURN_CENTER_X,
                DOOR_PANEL_DEPTH / 2.0,
                0.0,
            )
        ),
        material=door_steel,
        name="right_return",
    )
    door.visual(
        Box((DOOR_TOP_BOTTOM_WIDTH, DOOR_PANEL_DEPTH, DOOR_RETURN_WIDTH)),
        origin=Origin(
            xyz=(
                DOOR_TOP_BOTTOM_CENTER_X,
                DOOR_PANEL_DEPTH / 2.0,
                DOOR_HEIGHT / 2.0 - DOOR_RETURN_WIDTH / 2.0,
            )
        ),
        material=door_steel,
        name="top_return",
    )
    door.visual(
        Box((DOOR_TOP_BOTTOM_WIDTH, DOOR_PANEL_DEPTH, DOOR_RETURN_WIDTH)),
        origin=Origin(
            xyz=(
                DOOR_TOP_BOTTOM_CENTER_X,
                DOOR_PANEL_DEPTH / 2.0,
                -DOOR_HEIGHT / 2.0 + DOOR_RETURN_WIDTH / 2.0,
            )
        ),
        material=door_steel,
        name="bottom_return",
    )
    door.visual(
        Box((DOOR_RAISED_PANEL_WIDTH, 0.0014, DOOR_RAISED_PANEL_HEIGHT)),
        origin=Origin(
            xyz=(
                DOOR_PANEL_CENTER_X - 0.02,
                DOOR_PANEL_DEPTH + 0.0014 / 2.0,
                0.0,
            )
        ),
        material=steel,
        name="raised_stiffener",
    )
    door.inertial = Inertial.from_geometry(
        Box((DOOR_WIDTH, DOOR_PANEL_DEPTH, DOOR_HEIGHT)),
        mass=7.5,
        origin=Origin(xyz=(DOOR_PANEL_CENTER_X, DOOR_PANEL_DEPTH / 2.0, 0.0)),
    )

    for prefix, z_center in zip(("upper_hinge", "middle_hinge", "lower_hinge"), HINGE_ZS):
        _add_hinge_set(body, door, z_center, prefix, steel, hardware)

    door.visual(
        Box(HANDLE_PLATE_SIZE),
        origin=Origin(xyz=(HANDLE_X, HANDLE_PLATE_CENTER_Y, 0.0)),
        material=handle_black,
        name="escutcheon",
    )
    door.visual(
        Cylinder(radius=HANDLE_BOSS_RADIUS, length=HANDLE_BOSS_LENGTH),
        origin=Origin(
            xyz=(HANDLE_X, HANDLE_BOSS_CENTER_Y, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=handle_black,
        name="pivot_boss",
    )
    door.visual(
        Cylinder(radius=0.007, length=0.012),
        origin=Origin(
            xyz=(HANDLE_X - 0.013, DOOR_PANEL_DEPTH + HANDLE_PLATE_SIZE[1] + 0.006, 0.05),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=hardware,
        name="lock_core",
    )

    handle_lever = model.part("handle_lever")
    handle_lever.visual(
        Cylinder(radius=HANDLE_HUB_RADIUS, length=HANDLE_HUB_LENGTH),
        origin=Origin(
            xyz=(0.0, HANDLE_HUB_LENGTH / 2.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=handle_black,
        name="hub",
    )
    handle_lever.visual(
        Box(HANDLE_GRIP_SIZE),
        origin=Origin(xyz=(0.0, HANDLE_HUB_LENGTH + HANDLE_GRIP_SIZE[1] / 2.0, 0.0)),
        material=handle_black,
        name="grip",
    )
    handle_lever.visual(
        Box((0.016, 0.008, 0.05)),
        origin=Origin(xyz=(0.0, HANDLE_HUB_LENGTH + 0.004, -0.04)),
        material=handle_black,
        name="pull_lug",
    )
    handle_lever.inertial = Inertial.from_geometry(
        Box((0.03, 0.024, 0.14)),
        mass=0.16,
        origin=Origin(xyz=(0.0, 0.016, 0.0)),
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(HINGE_AXIS_X, HINGE_AXIS_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )
    model.articulation(
        "door_to_handle_lever",
        ArticulationType.REVOLUTE,
        parent=door,
        child=handle_lever,
        origin=Origin(xyz=(HANDLE_X, HANDLE_PIVOT_Y, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.5,
            lower=math.radians(-75.0),
            upper=0.0,
        ),
    )

    return body, door, handle_lever


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_electrical_panel")
    _build_panel_body(model)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    handle_lever = object_model.get_part("handle_lever")
    door_hinge = object_model.get_articulation("body_to_door")
    handle_joint = object_model.get_articulation("door_to_handle_lever")

    back_panel = body.get_visual("back_panel")
    front_frame_right = body.get_visual("front_frame_right")
    front_skin = door.get_visual("front_skin")
    right_return = door.get_visual("right_return")
    raised_stiffener = door.get_visual("raised_stiffener")
    handle_plate = door.get_visual("escutcheon")
    handle_boss = door.get_visual("pivot_boss")
    handle_grip = handle_lever.get_visual("grip")
    handle_hub = handle_lever.get_visual("hub")

    upper_body_knuckle_top = body.get_visual("upper_hinge_body_knuckle_top")
    upper_body_knuckle_bottom = body.get_visual("upper_hinge_body_knuckle_bottom")
    upper_door_knuckle = door.get_visual("upper_hinge_door_knuckle")
    middle_body_knuckle_top = body.get_visual("middle_hinge_body_knuckle_top")
    middle_body_knuckle_bottom = body.get_visual("middle_hinge_body_knuckle_bottom")
    middle_door_knuckle = door.get_visual("middle_hinge_door_knuckle")
    lower_body_knuckle_top = body.get_visual("lower_hinge_body_knuckle_top")
    lower_body_knuckle_bottom = body.get_visual("lower_hinge_body_knuckle_bottom")
    lower_door_knuckle = door.get_visual("lower_hinge_door_knuckle")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        door,
        handle_lever,
        elem_a=handle_boss,
        elem_b=handle_hub,
        reason="The handle lever rotates on a nested spindle seated inside the door-mounted pivot boss.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=48)

    body_aabb = ctx.part_world_aabb(body)
    back_panel_aabb = ctx.part_element_world_aabb(body, elem="back_panel")
    right_frame_aabb = ctx.part_element_world_aabb(body, elem="front_frame_right")
    ctx.check(
        "body_has_realistic_enclosure_size",
        back_panel_aabb is not None
        and right_frame_aabb is not None
        and 0.58 <= _box_dims(back_panel_aabb)[0] <= 0.62
        and 0.79 <= _box_dims(back_panel_aabb)[2] <= 0.81
        and 0.24 <= right_frame_aabb[1][1] - back_panel_aabb[0][1] <= 0.26,
        details=(
            f"Body AABB={body_aabb!r}, back_panel_aabb={back_panel_aabb!r}, "
            f"right_frame_aabb={right_frame_aabb!r}"
        ),
    )

    ctx.expect_overlap(door, body, axes="xz", min_overlap=0.55)
    ctx.expect_gap(
        door,
        body,
        axis="y",
        min_gap=0.24,
        positive_elem=right_return,
        negative_elem=back_panel,
        name="deep_steel_body_behind_door",
    )
    ctx.expect_gap(
        door,
        body,
        axis="y",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem=right_return,
        negative_elem=front_frame_right,
        name="door_pan_seats_just_ahead_of_front_frame",
    )
    handle_plate_aabb = ctx.part_element_world_aabb(door, elem="escutcheon")
    front_skin_aabb = ctx.part_element_world_aabb(door, elem="front_skin")
    ctx.check(
        "handle_base_mounts_flush_on_door_face",
        handle_plate_aabb is not None
        and front_skin_aabb is not None
        and abs(handle_plate_aabb[0][1] - front_skin_aabb[1][1]) <= 1e-6,
        details=f"handle_plate_aabb={handle_plate_aabb!r} front_skin_aabb={front_skin_aabb!r}",
    )
    ctx.expect_gap(
        handle_lever,
        door,
        axis="y",
        min_gap=0.008,
        positive_elem=handle_grip,
        negative_elem=front_skin,
        name="handle_grip_stands_proud_of_door_face",
    )
    ctx.expect_contact(
        handle_lever,
        door,
        elem_a=handle_hub,
        elem_b=handle_boss,
        name="handle_lever_contacts_pivot_boss",
    )
    ctx.expect_gap(
        door,
        door,
        axis="x",
        min_gap=0.03,
        positive_elem=handle_plate,
        negative_elem=raised_stiffener,
        name="locking_handle_sits_in_right_latch_zone",
    )
    ctx.expect_within(door, door, axes="xz", inner_elem=handle_plate, outer_elem=front_skin)
    ctx.expect_contact(
        body,
        door,
        elem_a=upper_body_knuckle_top,
        elem_b=upper_door_knuckle,
        name="upper_hinge_top_knuckle_contact",
    )
    ctx.expect_contact(
        body,
        door,
        elem_a=upper_body_knuckle_bottom,
        elem_b=upper_door_knuckle,
        name="upper_hinge_bottom_knuckle_contact",
    )
    ctx.expect_contact(
        body,
        door,
        elem_a=middle_body_knuckle_top,
        elem_b=middle_door_knuckle,
        name="middle_hinge_top_knuckle_contact",
    )
    ctx.expect_contact(
        body,
        door,
        elem_a=middle_body_knuckle_bottom,
        elem_b=middle_door_knuckle,
        name="middle_hinge_bottom_knuckle_contact",
    )
    ctx.expect_contact(
        body,
        door,
        elem_a=lower_body_knuckle_top,
        elem_b=lower_door_knuckle,
        name="lower_hinge_top_knuckle_contact",
    )
    ctx.expect_contact(
        body,
        door,
        elem_a=lower_body_knuckle_bottom,
        elem_b=lower_door_knuckle,
        name="lower_hinge_bottom_knuckle_contact",
    )

    door_closed_skin_aabb = ctx.part_element_world_aabb(door, elem="front_skin")
    handle_closed_grip_aabb = ctx.part_element_world_aabb(handle_lever, elem="grip")

    with ctx.pose({door_hinge: math.radians(100.0)}):
        ctx.fail_if_parts_overlap_in_current_pose(name="door_open_no_overlap")
        ctx.fail_if_isolated_parts(name="door_open_no_floating")
        ctx.expect_gap(
            door,
            body,
            axis="y",
            min_gap=0.50,
            positive_elem=right_return,
            negative_elem=front_frame_right,
            name="door_swings_clear_when_open",
        )
        if door_closed_skin_aabb is not None:
            open_skin_aabb = ctx.part_element_world_aabb(door, elem="front_skin")
            ctx.check(
                "door_moves_forward_when_open",
                open_skin_aabb is not None and open_skin_aabb[1][1] > door_closed_skin_aabb[1][1] + 0.48,
                details=f"Closed skin AABB={door_closed_skin_aabb!r}, open skin AABB={open_skin_aabb!r}",
            )

    with ctx.pose({handle_joint: math.radians(-75.0)}):
        ctx.fail_if_parts_overlap_in_current_pose(name="handle_rotated_no_overlap")
        ctx.fail_if_isolated_parts(name="handle_rotated_no_floating")
        ctx.expect_contact(
            handle_lever,
            door,
            elem_a=handle_hub,
            elem_b=handle_boss,
            name="handle_hub_stays_on_pivot_when_rotated",
        )
        if handle_closed_grip_aabb is not None:
            open_grip_aabb = ctx.part_element_world_aabb(handle_lever, elem="grip")
            closed_dx, _, closed_dz = _box_dims(handle_closed_grip_aabb)
            open_dx, _, open_dz = _box_dims(open_grip_aabb) if open_grip_aabb is not None else (0.0, 0.0, 0.0)
            ctx.check(
                "handle_lever_rotates_from_vertical_to_horizontal",
                open_grip_aabb is not None and open_dx > 0.10 and open_dz < 0.06,
                details=(
                    f"Closed grip AABB={handle_closed_grip_aabb!r}, "
                    f"open grip AABB={open_grip_aabb!r}"
                ),
            )

    with ctx.pose({door_hinge: math.radians(100.0), handle_joint: math.radians(-75.0)}):
        ctx.fail_if_parts_overlap_in_current_pose(name="door_open_handle_turned_no_overlap")
        ctx.fail_if_isolated_parts(name="door_open_handle_turned_no_floating")
        ctx.expect_gap(
            handle_lever,
            body,
            axis="y",
            min_gap=0.10,
            positive_elem=handle_grip,
            negative_elem=front_frame_right,
            name="turned_handle_moves_clear_with_open_door",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
