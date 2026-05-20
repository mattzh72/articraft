from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="glass_front_display_cabinet")

    dark_wood = Material("dark_walnut", rgba=(0.23, 0.12, 0.055, 1.0))
    inner_wood = Material("finished_interior_wood", rgba=(0.34, 0.20, 0.10, 1.0))
    glass = Material("slightly_blue_clear_glass", rgba=(0.62, 0.86, 0.95, 0.32))
    brass = Material("brushed_brass", rgba=(0.86, 0.62, 0.24, 1.0))

    width = 0.90
    depth = 0.34
    height = 1.75
    wall = 0.035
    door_height = 1.56
    door_width = 0.405
    door_thickness = 0.028
    rail = 0.045

    case = model.part("case")
    # A rigid, open-front wooden cabinet body: side walls, top/bottom, back,
    # a shadowed interior opening, and two glass display shelves.
    case.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(-width / 2 + wall / 2, 0.0, height / 2)),
        material=dark_wood,
        name="side_wall_0",
    )
    case.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(width / 2 - wall / 2, 0.0, height / 2)),
        material=dark_wood,
        name="side_wall_1",
    )
    case.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, wall / 2)),
        material=dark_wood,
        name="bottom_rail",
    )
    case.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, height - wall / 2)),
        material=dark_wood,
        name="top_rail",
    )
    case.visual(
        Box((width, wall, height)),
        origin=Origin(xyz=(0.0, depth / 2 - wall / 2, height / 2)),
        material=inner_wood,
        name="back_panel",
    )
    for i, shelf_z in enumerate((0.62, 1.10)):
        case.visual(
            Box((width - 2 * wall + 0.010, depth - 0.070, 0.010)),
            origin=Origin(xyz=(0.0, 0.010, shelf_z)),
            material=glass,
            name=f"glass_shelf_{i}",
        )
    for side, x in enumerate((-width / 2 + 0.018, width / 2 - 0.018)):
        for i, zc in enumerate((height / 2 - 0.53, height / 2, height / 2 + 0.53)):
            case.visual(
                Box((0.030, 0.004, 0.070)),
                origin=Origin(xyz=(x, -depth / 2 - 0.001, zc)),
                material=brass,
                name=f"case_hinge_leaf_{side}_{i}",
            )

    def add_door(name: str, sign: float) -> object:
        """Create a narrow glazed door whose local X points toward the meeting stile."""
        door = model.part(name)
        half_z = door_height / 2
        # Four overlapping wooden frame members hold a slightly inset glass pane.
        door.visual(
            Box((rail, door_thickness, door_height)),
            origin=Origin(xyz=(sign * rail / 2, 0.0, 0.0)),
            material=dark_wood,
            name="hinge_stile",
        )
        door.visual(
            Box((rail, door_thickness, door_height)),
            origin=Origin(xyz=(sign * (door_width - rail / 2), 0.0, 0.0)),
            material=dark_wood,
            name="meeting_stile",
        )
        door.visual(
            Box((door_width, door_thickness, rail)),
            origin=Origin(xyz=(sign * door_width / 2, 0.0, half_z - rail / 2)),
            material=dark_wood,
            name="top_frame",
        )
        door.visual(
            Box((door_width, door_thickness, rail)),
            origin=Origin(xyz=(sign * door_width / 2, 0.0, -half_z + rail / 2)),
            material=dark_wood,
            name="bottom_frame",
        )
        door.visual(
            Box((door_width - 2 * rail + 0.010, 0.006, door_height - 2 * rail + 0.010)),
            origin=Origin(xyz=(sign * door_width / 2, 0.004, 0.0)),
            material=glass,
            name="glass_pane",
        )
        # Three short hinge knuckles, each tied into the hinge stile by a brass leaf.
        for i, zc in enumerate((-0.53, 0.0, 0.53)):
            door.visual(
                Cylinder(radius=0.008, length=0.115),
                origin=Origin(xyz=(0.0, 0.0, zc), rpy=(0.0, 0.0, 0.0)),
                material=brass,
                name=f"hinge_barrel_{i}",
            )
            door.visual(
                Box((0.040, 0.004, 0.070)),
                origin=Origin(xyz=(sign * 0.022, -door_thickness / 2 - 0.001, zc)),
                material=brass,
                name=f"hinge_leaf_{i}",
            )
        return door

    door_0 = add_door("door_0", 1.0)
    door_1 = add_door("door_1", -1.0)

    def add_knob(name: str) -> object:
        knob = model.part(name)
        # A small round brass pull: base flange, neck, rounded head, and an
        # off-centre raised tick that makes axial rotation visibly legible.
        align_to_front_axis = (math.pi / 2, 0.0, 0.0)
        knob.visual(
            Cylinder(radius=0.019, length=0.004),
            origin=Origin(xyz=(0.0, -0.002, 0.0), rpy=align_to_front_axis),
            material=brass,
            name="mount_flange",
        )
        knob.visual(
            Cylinder(radius=0.008, length=0.022),
            origin=Origin(xyz=(0.0, -0.011, 0.0), rpy=align_to_front_axis),
            material=brass,
            name="neck",
        )
        knob.visual(
            Sphere(radius=0.023),
            origin=Origin(xyz=(0.0, -0.031, 0.0)),
            material=brass,
            name="round_pull",
        )
        knob.visual(
            Box((0.004, 0.004, 0.018)),
            origin=Origin(xyz=(0.009, -0.053, 0.0)),
            material=Material("polished_highlight", rgba=(1.0, 0.86, 0.46, 1.0)),
            name="rotation_tick",
        )
        return knob

    knob_0 = add_knob("knob_0")
    knob_1 = add_knob("knob_1")

    hinge_y = -depth / 2 - door_thickness / 2
    hinge_z = height / 2
    hinge_x = width / 2 - wall

    model.articulation(
        "case_to_door_0",
        ArticulationType.REVOLUTE,
        parent=case,
        child=door_0,
        origin=Origin(xyz=(-hinge_x, hinge_y, hinge_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=15.0, velocity=1.5, lower=0.0, upper=1.75),
    )
    model.articulation(
        "case_to_door_1",
        ArticulationType.REVOLUTE,
        parent=case,
        child=door_1,
        origin=Origin(xyz=(hinge_x, hinge_y, hinge_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=15.0, velocity=1.5, lower=0.0, upper=1.75),
    )

    knob_offset = door_width - 0.030
    model.articulation(
        "door_0_to_knob_0",
        ArticulationType.REVOLUTE,
        parent=door_0,
        child=knob_0,
        origin=Origin(xyz=(knob_offset, -door_thickness / 2, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=5.0, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "door_1_to_knob_1",
        ArticulationType.REVOLUTE,
        parent=door_1,
        child=knob_1,
        origin=Origin(xyz=(-knob_offset, -door_thickness / 2, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=5.0, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    case = object_model.get_part("case")
    door_0 = object_model.get_part("door_0")
    door_1 = object_model.get_part("door_1")
    knob_0 = object_model.get_part("knob_0")
    knob_1 = object_model.get_part("knob_1")
    door_joint_0 = object_model.get_articulation("case_to_door_0")
    door_joint_1 = object_model.get_articulation("case_to_door_1")

    ctx.expect_gap(
        case,
        door_0,
        axis="y",
        min_gap=-0.0005,
        max_gap=0.001,
        positive_elem="side_wall_0",
        negative_elem="hinge_stile",
        name="door 0 closes flush to the front plane",
    )
    ctx.expect_gap(
        case,
        door_1,
        axis="y",
        min_gap=-0.0005,
        max_gap=0.001,
        positive_elem="side_wall_1",
        negative_elem="hinge_stile",
        name="door 1 closes flush to the front plane",
    )

    knob_pos_0 = ctx.part_world_position(knob_0)
    knob_pos_1 = ctx.part_world_position(knob_1)
    ctx.check(
        "pull knobs sit near the meeting stile",
        knob_pos_0 is not None
        and knob_pos_1 is not None
        and -0.08 < knob_pos_0[0] < 0.0
        and 0.0 < knob_pos_1[0] < 0.08
        and knob_pos_0[1] < -0.19
        and knob_pos_1[1] < -0.19,
        details=f"knob_0={knob_pos_0}, knob_1={knob_pos_1}",
    )

    rest_aabb_0 = ctx.part_world_aabb(door_0)
    rest_aabb_1 = ctx.part_world_aabb(door_1)
    with ctx.pose({door_joint_0: 0.9, door_joint_1: 0.9}):
        open_aabb_0 = ctx.part_world_aabb(door_0)
        open_aabb_1 = ctx.part_world_aabb(door_1)
    ctx.check(
        "both doors swing outward from side hinges",
        rest_aabb_0 is not None
        and rest_aabb_1 is not None
        and open_aabb_0 is not None
        and open_aabb_1 is not None
        and open_aabb_0[0][1] < rest_aabb_0[0][1] - 0.10
        and open_aabb_1[0][1] < rest_aabb_1[0][1] - 0.10,
        details=f"rest0={rest_aabb_0}, open0={open_aabb_0}, rest1={rest_aabb_1}, open1={open_aabb_1}",
    )

    ctx.expect_contact(knob_0, door_0, elem_a="mount_flange", elem_b="meeting_stile", contact_tol=0.001)
    ctx.expect_contact(knob_1, door_1, elem_a="mount_flange", elem_b="meeting_stile", contact_tol=0.001)

    return ctx.report()


object_model = build_object_model()
