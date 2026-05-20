from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vented_locker_cabinet")

    powder_gray = model.material("powder_gray", rgba=(0.50, 0.54, 0.58, 1.0))
    blue_door = model.material("blue_sheet_metal", rgba=(0.20, 0.32, 0.46, 1.0))
    dark_metal = model.material("dark_hardware", rgba=(0.05, 0.055, 0.06, 1.0))
    bright_pin = model.material("worn_hinge_pin", rgba=(0.70, 0.68, 0.62, 1.0))

    width = 0.70
    depth = 0.48
    height = 1.90
    foot_h = 0.08
    wall = 0.032
    body_h = height - foot_h
    body_z = foot_h + body_h / 2.0
    front_y = -depth / 2.0
    hinge_x = -width / 2.0 - 0.010
    hinge_y = front_y - 0.015
    hinge_z0 = foot_h + 0.010

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((width, wall, body_h)),
        origin=Origin(xyz=(0.0, depth / 2.0 - wall / 2.0, body_z)),
        material=powder_gray,
        name="back_panel",
    )
    cabinet.visual(
        Box((wall, depth, body_h)),
        origin=Origin(xyz=(-width / 2.0 + wall / 2.0, 0.0, body_z)),
        material=powder_gray,
        name="side_panel_0",
    )
    cabinet.visual(
        Box((wall, depth, body_h)),
        origin=Origin(xyz=(width / 2.0 - wall / 2.0, 0.0, body_z)),
        material=powder_gray,
        name="side_panel_1",
    )
    cabinet.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, height - wall / 2.0)),
        material=powder_gray,
        name="top_panel",
    )
    cabinet.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, foot_h + wall / 2.0)),
        material=powder_gray,
        name="bottom_panel",
    )
    cabinet.visual(
        Box((0.060, 0.018, body_h)),
        origin=Origin(xyz=(width / 2.0 - 0.040, front_y - 0.007, body_z)),
        material=dark_metal,
        name="latch_strike",
    )
    cabinet.visual(
        Box((width, 0.018, 0.045)),
        origin=Origin(xyz=(0.0, front_y - 0.007, height - 0.058)),
        material=powder_gray,
        name="front_header",
    )
    cabinet.visual(
        Box((width, 0.018, 0.045)),
        origin=Origin(xyz=(0.0, front_y - 0.007, foot_h + 0.058)),
        material=powder_gray,
        name="front_sill",
    )
    for ix, x in enumerate((-width / 2.0 + 0.085, width / 2.0 - 0.085)):
        for iy, y in enumerate((front_y + 0.075, depth / 2.0 - 0.075)):
            cabinet.visual(
                Cylinder(radius=0.030, length=foot_h),
                origin=Origin(xyz=(x, y, foot_h / 2.0)),
                material=dark_metal,
                name=f"foot_{ix}_{iy}",
            )

    door = model.part("door")
    door_width = 0.64
    door_height = 1.72
    door_left = 0.030
    door_center_x = door_left + door_width / 2.0
    door_center_y = -0.010
    door_center_z = 0.910
    door_sheet = SlotPatternPanelGeometry(
        (door_width, door_height),
        0.018,
        slot_size=(0.42, 0.020),
        pitch=(0.50, 0.082),
        frame=0.070,
        corner_radius=0.006,
        stagger=False,
    )
    door.visual(
        mesh_from_geometry(door_sheet, "door_sheet"),
        origin=Origin(
            xyz=(door_center_x, door_center_y, door_center_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=blue_door,
        name="door_sheet",
    )
    door.visual(
        Box((0.026, 0.010, door_height - 0.10)),
        origin=Origin(xyz=(door_left + 0.035, -0.025, door_center_z)),
        material=blue_door,
        name="hinge_stiffener",
    )
    door.visual(
        Box((0.026, 0.010, door_height - 0.10)),
        origin=Origin(xyz=(door_left + door_width - 0.035, -0.025, door_center_z)),
        material=blue_door,
        name="latch_stiffener",
    )
    door.visual(
        Box((door_width - 0.08, 0.010, 0.024)),
        origin=Origin(xyz=(door_center_x, -0.025, door_center_z + door_height / 2.0 - 0.065)),
        material=blue_door,
        name="top_stiffener",
    )
    door.visual(
        Box((door_width - 0.08, 0.010, 0.024)),
        origin=Origin(xyz=(door_center_x, -0.025, door_center_z - door_height / 2.0 + 0.065)),
        material=blue_door,
        name="bottom_stiffener",
    )

    hinge_centers = (0.36, 0.91, 1.46)
    hinge_len = 0.22
    knuckle_len = 0.064
    hinge_radius = 0.018
    for i, zc in enumerate(hinge_centers):
        world_zc = hinge_z0 + zc
        # Fixed leaf and alternating fixed barrel knuckles on the cabinet jamb.
        cabinet.visual(
            Box((0.080, 0.008, hinge_len)),
            origin=Origin(xyz=(hinge_x + hinge_radius + 0.040, hinge_y + 0.011, world_zc)),
            material=bright_pin,
            name=f"hinge_leaf_fixed_{i}",
        )
        for j, dz in enumerate((-0.074, 0.074)):
            cabinet.visual(
                Cylinder(radius=hinge_radius, length=knuckle_len),
                origin=Origin(xyz=(hinge_x, hinge_y, world_zc + dz)),
                material=bright_pin,
                name=f"hinge_barrel_fixed_{i}_{j}",
            )
            cabinet.visual(
                Box((0.020, 0.008, 0.050)),
                origin=Origin(xyz=(hinge_x + hinge_radius + 0.005, hinge_y + 0.011, world_zc + dz)),
                material=bright_pin,
                name=f"hinge_web_fixed_{i}_{j}",
            )
        # Moving leaf and centered barrel knuckle attached to the door.
        door.visual(
            Box((0.088, 0.008, hinge_len)),
            origin=Origin(xyz=(0.044, -0.020, zc)),
            material=bright_pin,
            name=f"hinge_leaf_moving_{i}",
        )
        door.visual(
            Cylinder(radius=hinge_radius, length=knuckle_len),
            origin=Origin(xyz=(0.0, 0.0, zc)),
            material=bright_pin,
            name=f"hinge_barrel_moving_{i}",
        )

    latch_x = door_left + door_width - 0.110
    latch_z = door_center_z
    door.visual(
        Cylinder(radius=0.052, length=0.012),
        origin=Origin(xyz=(latch_x, -0.025, latch_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="latch_boss",
    )

    handle = model.part("latch_handle")
    handle.visual(
        Cylinder(radius=0.040, length=0.022),
        origin=Origin(xyz=(0.0, -0.011, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="handle_hub",
    )
    handle.visual(
        Cylinder(radius=0.018, length=0.008),
        origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="handle_stem",
    )
    handle.visual(
        Box((0.050, 0.018, 0.220)),
        origin=Origin(xyz=(0.0, -0.030, 0.0)),
        material=dark_metal,
        name="handle_bar",
    )
    handle.visual(
        Box((0.075, 0.014, 0.036)),
        origin=Origin(xyz=(0.0, -0.044, 0.0)),
        material=dark_metal,
        name="finger_pad",
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, hinge_z0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=0.0, upper=1.75),
    )
    model.articulation(
        "latch_rotation",
        ArticulationType.REVOLUTE,
        parent=door,
        child=handle,
        origin=Origin(xyz=(latch_x, -0.031, latch_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=5.0, lower=-1.57, upper=1.57),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    handle = object_model.get_part("latch_handle")
    door_hinge = object_model.get_articulation("door_hinge")
    latch = object_model.get_articulation("latch_rotation")

    with ctx.pose({door_hinge: 0.0, latch: 0.0}):
        ctx.expect_gap(
            cabinet,
            door,
            axis="y",
            max_gap=0.004,
            max_penetration=0.001,
            positive_elem="latch_strike",
            negative_elem="door_sheet",
            name="closed door sits close to latch strike",
        )
        ctx.expect_overlap(
            door,
            cabinet,
            axes="xy",
            min_overlap=0.020,
            elem_a="hinge_barrel_moving_1",
            elem_b="hinge_barrel_fixed_1_0",
            name="moving hinge knuckle stays coaxial with fixed barrels",
        )
        ctx.expect_contact(
            handle,
            door,
            elem_a="handle_stem",
            elem_b="latch_boss",
            contact_tol=0.002,
            name="rotary handle stem seats on door boss",
        )

    closed_aabb = ctx.part_element_world_aabb(door, elem="door_sheet")
    with ctx.pose({door_hinge: 1.20}):
        open_aabb = ctx.part_element_world_aabb(door, elem="door_sheet")
        ctx.expect_overlap(
            door,
            cabinet,
            axes="xy",
            min_overlap=0.020,
            elem_a="hinge_barrel_moving_1",
            elem_b="hinge_barrel_fixed_1_0",
            name="open door remains clipped on hinge axis",
        )
    ctx.check(
        "door swings outward on vertical hinge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][1] < closed_aabb[0][1] - 0.12,
        details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
    )

    with ctx.pose({latch: 0.0}):
        vertical_bar = ctx.part_element_world_aabb(handle, elem="handle_bar")
    with ctx.pose({latch: math.pi / 2.0}):
        turned_bar = ctx.part_element_world_aabb(handle, elem="handle_bar")
    ctx.check(
        "latch handle rotates from vertical to crosswise",
        vertical_bar is not None
        and turned_bar is not None
        and (vertical_bar[1][2] - vertical_bar[0][2]) > 0.18
        and (turned_bar[1][0] - turned_bar[0][0]) > 0.18,
        details=f"vertical_bar={vertical_bar}, turned_bar={turned_bar}",
    )

    return ctx.report()


object_model = build_object_model()
