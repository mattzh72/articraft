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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _rect_profile(width: float, height: float, *, cx: float = 0.0, cy: float = 0.0):
    half_w = width * 0.5
    half_h = height * 0.5
    return [
        (cx - half_w, cy - half_h),
        (cx + half_w, cy - half_h),
        (cx + half_w, cy + half_h),
        (cx - half_w, cy + half_h),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="metal_locker_cabinet")

    cabinet_blue = model.material("cabinet_blue", rgba=(0.49, 0.63, 0.72, 1.0))
    cabinet_blue_dark = model.material("cabinet_blue_dark", rgba=(0.37, 0.50, 0.58, 1.0))
    steel = model.material("steel", rgba=(0.73, 0.75, 0.77, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.29, 0.31, 0.33, 1.0))
    black = model.material("black", rgba=(0.10, 0.11, 0.12, 1.0))

    width = 0.92
    depth = 0.46
    height = 1.85
    wall = 0.018
    frame_width = 0.045
    frame_depth = 0.028
    bottom_frame = 0.051
    top_frame = 0.051
    side_gap = 0.004
    center_gap = 0.004
    door_skin_t = 0.0016
    door_return = 0.020
    door_flange = 0.018
    door_face_center_local = -0.014
    door_shell_y = door_face_center_local - (door_skin_t * 0.5) - (door_return * 0.5)
    hinge_radius = 0.009
    pin_y = (depth * 0.5) + hinge_radius
    opening_width = width - (2.0 * frame_width)
    door_width = (opening_width - (2.0 * side_gap) - center_gap) * 0.5
    door_height = height - bottom_frame - top_frame - 0.008
    door_bottom_z = bottom_frame + 0.004
    door_mid_z = door_bottom_z + (door_height * 0.5)
    door_edge_offset = 0.012
    left_axis_x = -(opening_width * 0.5) + side_gap - door_edge_offset
    right_axis_x = -left_axis_x
    front_face_y = (depth * 0.5) - 0.006

    vent_slot_width = door_width * 0.54
    vent_slot_height = 0.012
    vent_centers = (
        door_height * 0.28,
        door_height * 0.245,
        door_height * 0.21,
        -(door_height * 0.21),
        -(door_height * 0.245),
        -(door_height * 0.28),
    )
    door_holes = [
        _rect_profile(vent_slot_width, vent_slot_height, cy=slot_center)
        for slot_center in vent_centers
    ]
    door_skin_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _rect_profile(door_width, door_height),
            door_holes,
            height=door_skin_t,
            center=True,
        ).rotate_x(math.pi * 0.5),
        "locker_door_skin",
    )

    body = model.part("cabinet_body")
    body.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, wall * 0.5)),
        material=cabinet_blue_dark,
        name="base_pan",
    )
    body.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, height - (wall * 0.5))),
        material=cabinet_blue_dark,
        name="top_pan",
    )
    body.visual(
        Box((wall, depth, height - (2.0 * wall))),
        origin=Origin(xyz=(-(width * 0.5) + (wall * 0.5), 0.0, height * 0.5)),
        material=cabinet_blue_dark,
        name="left_side_wall",
    )
    body.visual(
        Box((wall, depth, height - (2.0 * wall))),
        origin=Origin(xyz=((width * 0.5) - (wall * 0.5), 0.0, height * 0.5)),
        material=cabinet_blue_dark,
        name="right_side_wall",
    )
    body.visual(
        Box((width - (2.0 * wall), wall, height - (2.0 * wall))),
        origin=Origin(xyz=(0.0, -(depth * 0.5) + (wall * 0.5), height * 0.5)),
        material=cabinet_blue_dark,
        name="back_panel",
    )
    body.visual(
        Box((frame_width, frame_depth, height - bottom_frame - top_frame)),
        origin=Origin(
            xyz=(
                -(opening_width * 0.5) - (frame_width * 0.5),
                (depth * 0.5) - (frame_depth * 0.5),
                (bottom_frame + height - top_frame) * 0.5,
            )
        ),
        material=cabinet_blue,
        name="left_frame_stile",
    )
    body.visual(
        Box((frame_width, frame_depth, height - bottom_frame - top_frame)),
        origin=Origin(
            xyz=(
                (opening_width * 0.5) + (frame_width * 0.5),
                (depth * 0.5) - (frame_depth * 0.5),
                (bottom_frame + height - top_frame) * 0.5,
            )
        ),
        material=cabinet_blue,
        name="right_frame_stile",
    )
    body.visual(
        Box((opening_width, frame_depth, top_frame)),
        origin=Origin(
            xyz=(
                0.0,
                (depth * 0.5) - (frame_depth * 0.5),
                height - (top_frame * 0.5),
            )
        ),
        material=cabinet_blue,
        name="top_header",
    )
    body.visual(
        Box((opening_width, frame_depth, bottom_frame)),
        origin=Origin(
            xyz=(
                0.0,
                (depth * 0.5) - (frame_depth * 0.5),
                bottom_frame * 0.5,
            )
        ),
        material=cabinet_blue,
        name="bottom_sill",
    )
    body.visual(
        Box((width - 0.08, depth - 0.08, 0.05)),
        origin=Origin(xyz=(0.0, -0.01, 0.043)),
        material=dark_steel,
        name="recessed_plinth",
    )
    body.visual(
        Box((0.16, 0.004, 0.028)),
        origin=Origin(xyz=(0.0, (depth * 0.5) - 0.002, height - (top_frame * 0.5))),
        material=steel,
        name="name_plate",
    )

    hinge_centers = (height - 0.29, 0.29)
    hinge_leaf_width = 0.022
    hinge_leaf_depth = 0.028
    door_hinge_leaf_width = 0.018
    hinge_plate_height = 0.16
    hinge_segment = 0.024
    hinge_mid_segment = 0.036
    knuckle_offset_z = 0.037
    for side_sign, axis_x, side_name in (
        (-1.0, left_axis_x, "left"),
        (1.0, right_axis_x, "right"),
    ):
        for index, hinge_center_z in enumerate(hinge_centers):
            body.visual(
                Box((hinge_leaf_width, hinge_leaf_depth, hinge_plate_height)),
                origin=Origin(
                    xyz=(
                        axis_x + (side_sign * (hinge_radius + (hinge_leaf_width * 0.5))),
                        pin_y - (hinge_leaf_depth * 0.5),
                        hinge_center_z,
                    )
                ),
                material=steel,
                name=f"{side_name}_body_leaf_{index}",
            )
            body.visual(
                Cylinder(radius=hinge_radius, length=hinge_segment),
                origin=Origin(xyz=(axis_x, pin_y, hinge_center_z + knuckle_offset_z)),
                material=steel,
                name=f"{side_name}_body_knuckle_upper_{index}",
            )
            body.visual(
                Cylinder(radius=hinge_radius, length=hinge_segment),
                origin=Origin(xyz=(axis_x, pin_y, hinge_center_z - knuckle_offset_z)),
                material=steel,
                name=f"{side_name}_body_knuckle_lower_{index}",
            )

    body.inertial = Inertial.from_geometry(
        Box((width, depth, height)),
        mass=62.0,
        origin=Origin(xyz=(0.0, 0.0, height * 0.5)),
    )

    def add_door(part_name: str, *, direction: float, has_handle_mount: bool) -> None:
        door = model.part(part_name)
        skin_center_x = direction * (door_edge_offset + (door_width * 0.5))
        hinge_flange_x = direction * (door_edge_offset + (door_flange * 0.5))
        latch_flange_x = direction * (door_edge_offset + door_width - (door_flange * 0.5))
        reinforcement_x = direction * (door_edge_offset + door_width - 0.065)
        door.visual(
            door_skin_mesh,
            origin=Origin(xyz=(skin_center_x, door_face_center_local, 0.0)),
            material=cabinet_blue,
            name="door_outer_skin",
        )
        door.visual(
            Box((door_width - (2.0 * door_flange), door_return, door_flange)),
            origin=Origin(xyz=(skin_center_x, door_shell_y, (door_height * 0.5) - (door_flange * 0.5))),
            material=cabinet_blue_dark,
            name="top_return",
        )
        door.visual(
            Box((door_width - (2.0 * door_flange), door_return, door_flange)),
            origin=Origin(xyz=(skin_center_x, door_shell_y, -(door_height * 0.5) + (door_flange * 0.5))),
            material=cabinet_blue_dark,
            name="bottom_return",
        )
        door.visual(
            Box((door_flange, door_return, door_height)),
            origin=Origin(xyz=(hinge_flange_x, door_shell_y, 0.0)),
            material=cabinet_blue_dark,
            name="hinge_side_return",
        )
        door.visual(
            Box((door_flange, door_return, door_height)),
            origin=Origin(xyz=(latch_flange_x, door_shell_y, 0.0)),
            material=cabinet_blue_dark,
            name="meeting_side_return",
        )
        door.visual(
            Box((door_hinge_leaf_width, 0.026, hinge_plate_height)),
            origin=Origin(
                xyz=(
                    direction * (hinge_radius + (door_hinge_leaf_width * 0.5)),
                    -0.003,
                    hinge_centers[0] - door_mid_z,
                )
            ),
            material=steel,
            name="upper_hinge_leaf",
        )
        door.visual(
            Box((door_hinge_leaf_width, 0.026, hinge_plate_height)),
            origin=Origin(
                xyz=(
                    direction * (hinge_radius + (door_hinge_leaf_width * 0.5)),
                    -0.003,
                    hinge_centers[1] - door_mid_z,
                )
            ),
            material=steel,
            name="lower_hinge_leaf",
        )
        door.visual(
            Cylinder(radius=hinge_radius, length=hinge_mid_segment),
            origin=Origin(xyz=(0.0, 0.0, hinge_centers[0] - door_mid_z)),
            material=steel,
            name="upper_door_knuckle",
        )
        door.visual(
            Cylinder(radius=hinge_radius, length=hinge_mid_segment),
            origin=Origin(xyz=(0.0, 0.0, hinge_centers[1] - door_mid_z)),
            material=steel,
            name="lower_door_knuckle",
        )
        if has_handle_mount:
            door.visual(
                Box((0.11, 0.017, 0.30)),
                origin=Origin(xyz=(reinforcement_x, door_shell_y + 0.001, 0.0)),
                material=dark_steel,
                name="lock_box",
            )
        else:
            door.visual(
                Box((0.06, 0.013, 0.28)),
                origin=Origin(
                    xyz=(direction * (door_edge_offset + door_width - 0.030), door_shell_y + 0.001, 0.0)
                ),
                material=steel,
                name="strike_plate",
            )
        door.inertial = Inertial.from_geometry(
            Box((door_width + 0.03, 0.05, door_height)),
            mass=9.0,
            origin=Origin(xyz=(skin_center_x, -0.012, 0.0)),
        )

    add_door("left_door", direction=1.0, has_handle_mount=False)
    add_door("right_door", direction=-1.0, has_handle_mount=True)

    latch = model.part("center_latch")
    latch.visual(
        Box((0.060, 0.005, 0.205)),
        origin=Origin(xyz=(0.0, 0.0025, 0.0)),
        material=steel,
        name="latch_plate",
    )
    latch.visual(
        Cylinder(radius=0.019, length=0.020),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_hub",
    )
    latch.visual(
        Box((0.182, 0.024, 0.028)),
        origin=Origin(xyz=(0.0, 0.031, 0.0)),
        material=steel,
        name="latch_bar",
    )
    latch.visual(
        Box((0.036, 0.026, 0.100)),
        origin=Origin(xyz=(0.0, 0.022, 0.0)),
        material=dark_steel,
        name="grip_bridge",
    )
    latch.visual(
        Cylinder(radius=0.006, length=0.026),
        origin=Origin(xyz=(0.073, 0.031, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=black,
        name="right_end_cap",
    )
    latch.visual(
        Cylinder(radius=0.006, length=0.026),
        origin=Origin(xyz=(-0.073, 0.031, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=black,
        name="left_end_cap",
    )
    latch.inertial = Inertial.from_geometry(
        Box((0.19, 0.05, 0.21)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.018, 0.0)),
    )

    left_door = model.get_part("left_door")
    right_door = model.get_part("right_door")

    model.articulation(
        "body_to_left_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_door,
        origin=Origin(xyz=(left_axis_x, pin_y, door_mid_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=2.05),
    )
    model.articulation(
        "body_to_right_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_door,
        origin=Origin(xyz=(right_axis_x, pin_y, door_mid_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=2.05),
    )
    model.articulation(
        "right_door_to_latch",
        ArticulationType.REVOLUTE,
        parent=right_door,
        child=latch,
        origin=Origin(
            xyz=(
                -(door_edge_offset + door_width - 0.028),
                door_face_center_local + (door_skin_t * 0.5),
                0.0,
            )
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.5, lower=-1.10, upper=1.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    body = object_model.get_part("cabinet_body")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    latch = object_model.get_part("center_latch")
    left_hinge = object_model.get_articulation("body_to_left_door")
    right_hinge = object_model.get_articulation("body_to_right_door")
    latch_joint = object_model.get_articulation("right_door_to_latch")

    def _extent(aabb, axis: str) -> float | None:
        if aabb is None:
            return None
        axis_index = {"x": 0, "y": 1, "z": 2}[axis]
        return aabb[1][axis_index] - aabb[0][axis_index]

    ctx.check(
        "door hinges use vertical axes",
        left_hinge.axis == (0.0, 0.0, 1.0) and right_hinge.axis == (0.0, 0.0, -1.0),
        details=f"left_axis={left_hinge.axis}, right_axis={right_hinge.axis}",
    )
    ctx.check(
        "latch pivots about the door-normal axis",
        latch_joint.axis == (0.0, 1.0, 0.0),
        details=f"latch_axis={latch_joint.axis}",
    )

    with ctx.pose({left_hinge: 0.0, right_hinge: 0.0, latch_joint: 0.0}):
        ctx.expect_gap(
            right_door,
            left_door,
            axis="x",
            min_gap=0.003,
            max_gap=0.008,
            positive_elem="door_outer_skin",
            negative_elem="door_outer_skin",
            name="twin doors meet with a narrow center seam",
        )
        ctx.expect_contact(
            latch,
            right_door,
            elem_a="latch_plate",
            elem_b="door_outer_skin",
            contact_tol=0.0008,
            name="latch plate mounts flush on the right door",
        )

    with ctx.pose({left_hinge: 0.0, right_hinge: 0.0}):
        left_closed_aabb = ctx.part_element_world_aabb(left_door, elem="door_outer_skin")
        right_closed_aabb = ctx.part_element_world_aabb(right_door, elem="door_outer_skin")
        body_aabb = ctx.part_world_aabb(body)

    with ctx.pose({left_hinge: 1.20, right_hinge: 1.20}):
        left_open_aabb = ctx.part_element_world_aabb(left_door, elem="door_outer_skin")
        right_open_aabb = ctx.part_element_world_aabb(right_door, elem="door_outer_skin")

    left_closed_max_y = None if left_closed_aabb is None else left_closed_aabb[1][1]
    right_closed_max_y = None if right_closed_aabb is None else right_closed_aabb[1][1]
    left_open_max_y = None if left_open_aabb is None else left_open_aabb[1][1]
    right_open_max_y = None if right_open_aabb is None else right_open_aabb[1][1]
    body_front_y = None if body_aabb is None else body_aabb[1][1]
    ctx.check(
        "positive hinge motion swings both doors outward",
        (
            left_open_max_y is not None
            and right_open_max_y is not None
            and left_closed_max_y is not None
            and right_closed_max_y is not None
            and left_open_max_y > left_closed_max_y + 0.17
            and right_open_max_y > right_closed_max_y + 0.17
            and body_front_y is not None
            and left_open_max_y > body_front_y + 0.08
            and right_open_max_y > body_front_y + 0.08
        ),
        details=(
            f"left_closed_max_y={left_closed_max_y}, left_open_max_y={left_open_max_y}, "
            f"right_closed_max_y={right_closed_max_y}, right_open_max_y={right_open_max_y}, "
            f"body_front_y={body_front_y}"
        ),
    )

    with ctx.pose({right_hinge: 0.0, latch_joint: 0.0}):
        latch_bar_closed = ctx.part_element_world_aabb(latch, elem="latch_bar")
    with ctx.pose({right_hinge: 0.0, latch_joint: 1.00}):
        latch_bar_open = ctx.part_element_world_aabb(latch, elem="latch_bar")

    closed_x = _extent(latch_bar_closed, "x")
    closed_z = _extent(latch_bar_closed, "z")
    open_x = _extent(latch_bar_open, "x")
    open_z = _extent(latch_bar_open, "z")
    ctx.check(
        "central latch visibly rotates on its own pivot",
        (
            closed_x is not None
            and closed_z is not None
            and open_x is not None
            and open_z is not None
            and closed_x > 0.14
            and closed_z < 0.05
            and open_x < closed_x - 0.03
            and open_z > closed_z + 0.08
        ),
        details=(
            f"closed_x={closed_x}, closed_z={closed_z}, "
            f"open_x={open_x}, open_z={open_z}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
