from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="twin_door_locker")

    painted = model.material("powder_coated_blue_gray", rgba=(0.34, 0.43, 0.49, 1.0))
    dark_metal = model.material("dark_hinge_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    worn_steel = model.material("brushed_latch_steel", rgba=(0.72, 0.70, 0.64, 1.0))
    shadow = model.material("dark_recess_shadow", rgba=(0.02, 0.025, 0.03, 1.0))

    width = 0.90
    depth = 0.45
    height = 1.80
    wall = 0.025
    front_y = -depth / 2.0
    hinge_y = front_y - 0.020
    hinge_x = width / 2.0 + 0.025

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((width, wall, height)),
        origin=Origin(xyz=(0.0, depth / 2.0 - wall / 2.0, height / 2.0)),
        material=painted,
        name="back_panel",
    )
    cabinet.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(-width / 2.0 + wall / 2.0, 0.0, height / 2.0)),
        material=painted,
        name="side_wall_0",
    )
    cabinet.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(width / 2.0 - wall / 2.0, 0.0, height / 2.0)),
        material=painted,
        name="side_wall_1",
    )
    cabinet.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, height - wall / 2.0)),
        material=painted,
        name="top_panel",
    )
    cabinet.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, wall / 2.0)),
        material=painted,
        name="bottom_panel",
    )
    cabinet.visual(
        Box((width, 0.030, 0.040)),
        origin=Origin(xyz=(0.0, front_y - 0.004, height - 0.045)),
        material=painted,
        name="front_header_lip",
    )
    cabinet.visual(
        Box((width, 0.030, 0.045)),
        origin=Origin(xyz=(0.0, front_y - 0.004, 0.0475)),
        material=painted,
        name="front_sill_lip",
    )

    hinge_zs = (0.34, 0.90, 1.46)
    for side, sx in enumerate((-1.0, 1.0)):
        for i, z in enumerate(hinge_zs):
            cabinet.visual(
                Box((0.046, 0.026, 0.062)),
                origin=Origin(xyz=(sx * (width / 2.0 + 0.007), front_y - 0.005, z - 0.061)),
                material=dark_metal,
                name=f"hinge_leaf_{side}_{i}_low",
            )
            cabinet.visual(
                Box((0.046, 0.026, 0.062)),
                origin=Origin(xyz=(sx * (width / 2.0 + 0.007), front_y - 0.005, z + 0.061)),
                material=dark_metal,
                name=f"hinge_leaf_{side}_{i}_high",
            )
            cabinet.visual(
                Cylinder(radius=0.014, length=0.062),
                origin=Origin(xyz=(sx * hinge_x, hinge_y, z - 0.061)),
                material=dark_metal,
                name=f"hinge_barrel_{side}_{i}_low",
            )
            cabinet.visual(
                Cylinder(radius=0.014, length=0.062),
                origin=Origin(xyz=(sx * hinge_x, hinge_y, z + 0.061)),
                material=dark_metal,
                name=f"hinge_barrel_{side}_{i}_high",
            )

    door_height = 1.64
    door_width = 0.430
    door_center_z = 0.90
    door_y = -0.018
    door_thickness = 0.018

    def add_door_details(door, sign: float, prefix: str) -> None:
        panel_center_x = sign * 0.252
        door.visual(
            Box((door_width, door_thickness, door_height)),
            origin=Origin(xyz=(panel_center_x, door_y, door_center_z)),
            material=painted,
            name="door_panel",
        )
        door.visual(
            Box((0.012, 0.006, door_height - 0.10)),
            origin=Origin(xyz=(sign * 0.044, -0.030, door_center_z)),
            material=dark_metal,
            name="hinge_edge_fold",
        )
        for i, z in enumerate(hinge_zs):
            door.visual(
                Box((0.060, 0.008, 0.165)),
                origin=Origin(xyz=(sign * 0.045, -0.029, z)),
                material=dark_metal,
                name=f"hinge_strap_{i}",
            )
            door.visual(
                Box((0.050, 0.029, 0.060)),
                origin=Origin(xyz=(sign * 0.025, -0.0145, z)),
                material=dark_metal,
                name=f"hinge_web_{i}",
            )
            door.visual(
                Cylinder(radius=0.014, length=0.060),
                origin=Origin(xyz=(0.0, 0.0, z)),
                material=dark_metal,
                name=f"hinge_knuckle_{i}",
            )
            for dz in (-0.050, 0.050):
                door.visual(
                    Cylinder(radius=0.006, length=0.004),
                    origin=Origin(
                        xyz=(sign * 0.065, -0.034, z + dz),
                        rpy=(-math.pi / 2.0, 0.0, 0.0),
                    ),
                    material=worn_steel,
                    name=f"strap_screw_{i}_{0 if dz < 0 else 1}",
                )
        door.visual(
            Box((0.010, 0.006, door_height - 0.16)),
            origin=Origin(xyz=(sign * 0.461, -0.029, door_center_z)),
            material=dark_metal,
            name="meeting_edge_fold",
        )
        door.visual(
            Box((0.145, 0.006, 0.050)),
            origin=Origin(xyz=(panel_center_x, -0.030, 1.48)),
            material=worn_steel,
            name="label_frame",
        )
        door.visual(
            Box((0.126, 0.004, 0.030)),
            origin=Origin(xyz=(panel_center_x, -0.029, 1.48)),
            material=shadow,
            name="label_card_recess",
        )
        for i in range(5):
            door.visual(
                Box((0.165, 0.004, 0.014)),
                origin=Origin(xyz=(panel_center_x, -0.029, 1.240 - 0.040 * i)),
                material=shadow,
                name=f"upper_vent_{i}",
            )
        for i in range(5):
            door.visual(
                Box((0.165, 0.004, 0.014)),
                origin=Origin(xyz=(panel_center_x, -0.029, 0.500 - 0.040 * i)),
                material=shadow,
                name=f"lower_vent_{i}",
            )

    door_0 = model.part("door_0")
    add_door_details(door_0, 1.0, "door_0")

    door_1 = model.part("door_1")
    add_door_details(door_1, -1.0, "door_1")
    door_1.visual(
        Box((0.034, 0.008, 0.125)),
        origin=Origin(xyz=(-0.453, -0.031, 0.96)),
        material=dark_metal,
        name="latch_mount_plate",
    )

    latch_handle = model.part("latch_handle")
    latch_handle.visual(
        Cylinder(radius=0.034, length=0.026),
        origin=Origin(xyz=(0.0, -0.013, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=worn_steel,
        name="pivot_boss",
    )
    latch_handle.visual(
        Box((0.275, 0.014, 0.034)),
        origin=Origin(xyz=(0.0, -0.028, 0.0)),
        material=worn_steel,
        name="crossbar",
    )
    latch_handle.visual(
        Box((0.050, 0.020, 0.070)),
        origin=Origin(xyz=(0.0, -0.044, 0.0)),
        material=worn_steel,
        name="finger_grip",
    )
    latch_handle.visual(
        Cylinder(radius=0.007, length=0.030),
        origin=Origin(xyz=(-0.110, -0.028, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="left_latch_pin",
    )
    latch_handle.visual(
        Cylinder(radius=0.007, length=0.030),
        origin=Origin(xyz=(0.110, -0.028, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="right_latch_pin",
    )

    model.articulation(
        "cabinet_to_door_0",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door_0,
        origin=Origin(xyz=(-hinge_x, hinge_y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.2, lower=0.0, upper=1.75),
    )
    model.articulation(
        "cabinet_to_door_1",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door_1,
        origin=Origin(xyz=(hinge_x, hinge_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.2, lower=0.0, upper=1.75),
    )
    model.articulation(
        "door_1_to_latch_handle",
        ArticulationType.REVOLUTE,
        parent=door_1,
        child=latch_handle,
        origin=Origin(xyz=(-0.475, -0.035, 0.96)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=3.0, lower=0.0, upper=math.pi / 2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    door_0 = object_model.get_part("door_0")
    door_1 = object_model.get_part("door_1")
    latch = object_model.get_part("latch_handle")
    door_0_hinge = object_model.get_articulation("cabinet_to_door_0")
    door_1_hinge = object_model.get_articulation("cabinet_to_door_1")
    latch_pivot = object_model.get_articulation("door_1_to_latch_handle")

    ctx.expect_overlap(
        door_0,
        cabinet,
        axes="xz",
        min_overlap=0.35,
        elem_a="door_panel",
        name="door_0 covers the cabinet opening",
    )
    ctx.expect_overlap(
        door_1,
        cabinet,
        axes="xz",
        min_overlap=0.35,
        elem_a="door_panel",
        name="door_1 covers the cabinet opening",
    )
    ctx.expect_gap(
        cabinet,
        door_0,
        axis="y",
        min_gap=0.006,
        positive_elem="front_header_lip",
        negative_elem="door_panel",
        name="door_0 sits proud of the fixed cabinet",
    )
    ctx.expect_gap(
        cabinet,
        door_1,
        axis="y",
        min_gap=0.006,
        positive_elem="front_header_lip",
        negative_elem="door_panel",
        name="door_1 sits proud of the fixed cabinet",
    )
    ctx.expect_overlap(
        latch,
        door_0,
        axes="x",
        min_overlap=0.020,
        elem_a="crossbar",
        elem_b="door_panel",
        name="latch crossbar bridges across door_0",
    )
    ctx.expect_overlap(
        latch,
        door_1,
        axes="x",
        min_overlap=0.020,
        elem_a="crossbar",
        elem_b="door_panel",
        name="latch crossbar bridges across door_1",
    )
    ctx.expect_contact(
        latch,
        door_1,
        elem_a="pivot_boss",
        elem_b="latch_mount_plate",
        contact_tol=0.0015,
        name="latch boss seats on its pivot plate",
    )

    rest_0 = ctx.part_element_world_aabb(door_0, elem="door_panel")
    rest_1 = ctx.part_element_world_aabb(door_1, elem="door_panel")
    with ctx.pose({door_0_hinge: 1.20, door_1_hinge: 1.20}):
        open_0 = ctx.part_element_world_aabb(door_0, elem="door_panel")
        open_1 = ctx.part_element_world_aabb(door_1, elem="door_panel")
    ctx.check(
        "doors swing outward from the side hinges",
        rest_0 is not None
        and rest_1 is not None
        and open_0 is not None
        and open_1 is not None
        and open_0[0][1] < rest_0[0][1] - 0.10
        and open_1[0][1] < rest_1[0][1] - 0.10,
        details=f"rest_0={rest_0}, open_0={open_0}, rest_1={rest_1}, open_1={open_1}",
    )

    rest_bar = ctx.part_element_world_aabb(latch, elem="crossbar")
    with ctx.pose({latch_pivot: math.pi / 2.0}):
        turned_bar = ctx.part_element_world_aabb(latch, elem="crossbar")
    ctx.check(
        "latch handle rotates on the center pivot",
        rest_bar is not None
        and turned_bar is not None
        and (rest_bar[1][0] - rest_bar[0][0]) > (rest_bar[1][2] - rest_bar[0][2]) * 4.0
        and (turned_bar[1][2] - turned_bar[0][2]) > (turned_bar[1][0] - turned_bar[0][0]) * 4.0,
        details=f"rest_bar={rest_bar}, turned_bar={turned_bar}",
    )

    return ctx.report()


object_model = build_object_model()
