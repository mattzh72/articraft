from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="high_voltage_busbar_chamber")

    painted_steel = model.material("painted_steel", rgba=(0.52, 0.57, 0.58, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.13, 0.15, 0.16, 1.0))
    hinge_steel = model.material("oiled_hinge_steel", rgba=(0.08, 0.09, 0.09, 1.0))
    latch_steel = model.material("brushed_latch_steel", rgba=(0.62, 0.63, 0.60, 1.0))
    rubber = model.material("black_gasket", rgba=(0.015, 0.016, 0.014, 1.0))
    warning_yellow = model.material("warning_yellow", rgba=(1.0, 0.78, 0.05, 1.0))
    copper = model.material("copper_busbar", rgba=(0.86, 0.37, 0.12, 1.0))
    porcelain = model.material("porcelain_insulator", rgba=(0.92, 0.88, 0.76, 1.0))

    # Overall envelope: a real floor-standing busbar cubicle, about two meters
    # tall, with the front door just proud of the chamber frame.
    body_w = 1.10
    body_d = 0.56
    body_h = 2.20
    wall = 0.045
    front_frame_y = body_d / 2.0 + 0.010
    front_frame_depth = 0.050

    hinge_x = -0.48
    hinge_y = body_d / 2.0 + 0.055
    door_bottom = 0.19
    door_h = 1.86
    panel_x0 = 0.070
    panel_w = 0.880
    door_right_x = panel_x0 + panel_w

    cubicle = model.part("cubicle")
    cubicle.visual(
        Box((body_w, wall, body_h)),
        origin=Origin(xyz=(0.0, -body_d / 2.0 + wall / 2.0, body_h / 2.0)),
        material=painted_steel,
        name="back_panel",
    )
    cubicle.visual(
        Box((wall, body_d, body_h)),
        origin=Origin(xyz=(-body_w / 2.0 + wall / 2.0, 0.0, body_h / 2.0)),
        material=painted_steel,
        name="side_panel_0",
    )
    cubicle.visual(
        Box((wall, body_d, body_h)),
        origin=Origin(xyz=(body_w / 2.0 - wall / 2.0, 0.0, body_h / 2.0)),
        material=painted_steel,
        name="side_panel_1",
    )
    cubicle.visual(
        Box((body_w, body_d, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=dark_steel,
        name="plinth",
    )
    cubicle.visual(
        Box((body_w, body_d, wall)),
        origin=Origin(xyz=(0.0, 0.0, body_h - wall / 2.0)),
        material=painted_steel,
        name="top_panel",
    )
    cubicle.visual(
        Box((0.120, front_frame_depth, body_h - 0.140)),
        origin=Origin(xyz=(-body_w / 2.0 + 0.060, front_frame_y, body_h / 2.0)),
        material=dark_steel,
        name="left_jamb",
    )
    cubicle.visual(
        Box((0.120, front_frame_depth, body_h - 0.140)),
        origin=Origin(xyz=(body_w / 2.0 - 0.060, front_frame_y, body_h / 2.0)),
        material=dark_steel,
        name="right_jamb",
    )
    cubicle.visual(
        Box((body_w, front_frame_depth, 0.115)),
        origin=Origin(xyz=(0.0, front_frame_y, body_h - 0.0575)),
        material=dark_steel,
        name="head_rail",
    )
    cubicle.visual(
        Box((body_w, front_frame_depth, 0.115)),
        origin=Origin(xyz=(0.0, front_frame_y, 0.0575)),
        material=dark_steel,
        name="sill_rail",
    )

    # A few copper busbar ends and porcelain standoff caps are visible above
    # the inspection opening, giving the cubicle its high-voltage chamber read.
    for i, x in enumerate((-0.22, 0.0, 0.22)):
        cubicle.visual(
            Cylinder(radius=0.026, length=0.070),
            origin=Origin(xyz=(x, body_d / 2.0 + 0.014, 2.065), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=copper,
            name=f"busbar_end_{i}",
        )
        cubicle.visual(
            Cylinder(radius=0.040, length=0.050),
            origin=Origin(xyz=(x, body_d / 2.0 - 0.015, 2.005), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=porcelain,
            name=f"insulator_cap_{i}",
        )

    hinge_radius = 0.040
    hinge_center_y = hinge_y + 0.028
    body_knuckle_ranges = (
        (0.12, 0.30),
        (0.68, 0.84),
        (1.22, 1.38),
        (1.74, 1.86),
    )
    for i, (z0, z1) in enumerate(body_knuckle_ranges):
        zc = door_bottom + (z0 + z1) / 2.0
        length = z1 - z0
        cubicle.visual(
            Box((0.100, 0.048, length)),
            origin=Origin(xyz=(hinge_x - 0.045, body_d / 2.0 + 0.059, zc)),
            material=hinge_steel,
            name=f"body_hinge_leaf_{i}",
        )
        cubicle.visual(
            Cylinder(radius=hinge_radius, length=length),
            origin=Origin(xyz=(hinge_x, hinge_center_y, zc)),
            material=hinge_steel,
            name="body_knuckle_1" if i == 1 else f"body_knuckle_{i}",
        )
    cubicle.visual(
        Cylinder(radius=0.018, length=door_h + 0.030),
        origin=Origin(xyz=(hinge_x, hinge_center_y, door_bottom + door_h / 2.0)),
        material=hinge_steel,
        name="hinge_pin",
    )

    bolt_levels = (0.48, 0.93, 1.38)
    for i, level in enumerate(bolt_levels):
        z = door_bottom + level
        # C-shaped keeper mouths on the fixed cubicle stile. The center slot is
        # open for the latch bolt; the back bridge keeps each keeper connected.
        cubicle.visual(
            Box((0.066, 0.110, 0.018)),
            origin=Origin(xyz=(0.535, body_d / 2.0 + 0.140, z + 0.036)),
            material=latch_steel,
            name=f"keeper_{i}_top",
        )
        cubicle.visual(
            Box((0.066, 0.110, 0.018)),
            origin=Origin(xyz=(0.535, body_d / 2.0 + 0.140, z - 0.036)),
            material=latch_steel,
            name=f"keeper_{i}_bottom",
        )
        cubicle.visual(
            Box((0.018, 0.110, 0.090)),
            origin=Origin(xyz=(0.577, body_d / 2.0 + 0.140, z)),
            material=latch_steel,
            name="keeper_1_bridge" if i == 1 else f"keeper_{i}_bridge",
        )
        cubicle.visual(
            Box((0.066, 0.050, 0.090)),
            origin=Origin(xyz=(0.535, body_d / 2.0 + 0.060, z)),
            material=latch_steel,
            name=f"keeper_mount_{i}",
        )

    door = model.part("door")
    door.visual(
        Box((panel_w, 0.055, door_h)),
        origin=Origin(xyz=(panel_x0 + panel_w / 2.0, 0.0275, door_h / 2.0)),
        material=painted_steel,
        name="door_panel",
    )
    door.visual(
        Box((panel_w - 0.140, 0.010, door_h - 0.260)),
        origin=Origin(xyz=(panel_x0 + panel_w / 2.0, 0.061, door_h / 2.0)),
        material=rubber,
        name="recess_gasket",
    )
    door.visual(
        Box((0.075, 0.030, door_h + 0.010)),
        origin=Origin(xyz=(panel_x0 + 0.0375, 0.070, door_h / 2.0)),
        material=dark_steel,
        name="left_stile",
    )
    door.visual(
        Box((0.090, 0.030, door_h + 0.010)),
        origin=Origin(xyz=(door_right_x - 0.045, 0.070, door_h / 2.0)),
        material=dark_steel,
        name="right_stile",
    )
    door.visual(
        Box((panel_w, 0.030, 0.080)),
        origin=Origin(xyz=(panel_x0 + panel_w / 2.0, 0.070, door_h - 0.040)),
        material=dark_steel,
        name="top_stile",
    )
    door.visual(
        Box((panel_w, 0.030, 0.080)),
        origin=Origin(xyz=(panel_x0 + panel_w / 2.0, 0.070, 0.040)),
        material=dark_steel,
        name="bottom_stile",
    )
    door.visual(
        Box((0.185, 0.006, 0.135)),
        origin=Origin(xyz=(panel_x0 + 0.305, 0.058, 1.360)),
        material=warning_yellow,
        name="warning_placard",
    )
    door.visual(
        Box((0.020, 0.008, 0.100)),
        origin=Origin(xyz=(panel_x0 + 0.305, 0.065, 1.360), rpy=(0.0, 0.0, 0.32)),
        material=dark_steel,
        name="lightning_mark",
    )

    door_knuckle_ranges = (
        (0.34, 0.64),
        (0.88, 1.18),
        (1.42, 1.70),
    )
    for i, (z0, z1) in enumerate(door_knuckle_ranges):
        zc = (z0 + z1) / 2.0
        length = z1 - z0
        door.visual(
            Box((0.120, 0.030, length)),
            origin=Origin(xyz=(0.035, 0.060, zc)),
            material=hinge_steel,
            name=f"door_hinge_leaf_{i}",
        )
        door.visual(
            Cylinder(radius=hinge_radius, length=length),
            origin=Origin(xyz=(0.0, 0.028, zc)),
            material=hinge_steel,
            name="door_knuckle_0" if i == 0 else ("door_knuckle_1" if i == 1 else "door_knuckle_2"),
        )

    latch_plate_x = door_right_x - 0.165
    handle_x = door_right_x - 0.180
    handle_z = 0.930
    handle_joint_y = 0.108
    door.visual(
        Box((0.185, 0.020, 1.120)),
        origin=Origin(xyz=(latch_plate_x, 0.087, door_h / 2.0)),
        material=latch_steel,
        name="latch_backplate",
    )
    door.visual(
        Cylinder(radius=0.077, length=0.034),
        origin=Origin(xyz=(handle_x, 0.091, handle_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=latch_steel,
        name="handle_boss",
    )
    for i, level in enumerate(bolt_levels):
        guide_x = door_right_x - 0.035 if i == 1 else door_right_x - 0.120
        guide_w = 0.130 if i == 1 else 0.285
        door.visual(
            Box((guide_w, 0.034, 0.014)),
            origin=Origin(xyz=(guide_x, 0.111, level + 0.020)),
            material=latch_steel,
            name=f"bolt_guide_{i}_top",
        )
        door.visual(
            Box((guide_w, 0.034, 0.014)),
            origin=Origin(xyz=(guide_x, 0.111, level - 0.020)),
            material=latch_steel,
            name=f"bolt_guide_{i}_bottom",
        )

    door_hinge = model.articulation(
        "cubicle_to_door",
        ArticulationType.REVOLUTE,
        parent=cubicle,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, door_bottom)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=260.0, velocity=0.65, lower=0.0, upper=1.75),
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.066, length=0.050),
        origin=Origin(xyz=(0.0, 0.025, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=hinge_steel,
        name="turning_hub",
    )
    handle.visual(
        Box((0.082, 0.038, 0.380)),
        origin=Origin(xyz=(0.0, 0.069, 0.0)),
        material=hinge_steel,
        name="turning_handle",
    )
    handle.visual(
        Cylinder(radius=0.027, length=0.090),
        origin=Origin(xyz=(0.0, 0.074, 0.205), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="grip_end_0",
    )
    handle.visual(
        Cylinder(radius=0.027, length=0.090),
        origin=Origin(xyz=(0.0, 0.074, -0.205), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="grip_end_1",
    )
    handle_turn = model.articulation(
        "door_to_handle",
        ArticulationType.REVOLUTE,
        parent=door,
        child=handle,
        origin=Origin(xyz=(handle_x, handle_joint_y, handle_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.4, lower=0.0, upper=math.pi / 2.0),
    )

    for i, level in enumerate(bolt_levels):
        bolt = model.part(f"bolt_{i}")
        bolt.visual(
            Box((0.190, 0.022, 0.026)),
            origin=Origin(xyz=(0.165, 0.0, 0.0)),
            material=latch_steel,
            name="bolt_bar",
        )
        bolt.visual(
            Box((0.035, 0.034, 0.020)),
            origin=Origin(xyz=(0.095, 0.0, 0.0)),
            material=hinge_steel,
            name="bolt_shoe",
        )
        model.articulation(
            f"door_to_bolt_{i}",
            ArticulationType.REVOLUTE,
            parent=door,
            child=bolt,
            origin=Origin(xyz=(door_right_x - 0.170, 0.128, level)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=90.0, velocity=0.9, lower=0.0, upper=math.pi / 2.0),
            mimic=Mimic(joint=handle_turn.name, multiplier=1.0, offset=0.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cubicle = object_model.get_part("cubicle")
    door = object_model.get_part("door")
    handle = object_model.get_part("handle")
    bolt_1 = object_model.get_part("bolt_1")
    door_hinge = object_model.get_articulation("cubicle_to_door")
    handle_turn = object_model.get_articulation("door_to_handle")

    ctx.allow_overlap(
        cubicle,
        door,
        elem_a="hinge_pin",
        elem_b="door_knuckle_0",
        reason="The fixed steel hinge pin intentionally passes through the door-side hinge knuckles.",
    )
    ctx.allow_overlap(
        cubicle,
        door,
        elem_a="hinge_pin",
        elem_b="door_knuckle_1",
        reason="The fixed steel hinge pin intentionally passes through the door-side hinge knuckles.",
    )
    ctx.allow_overlap(
        cubicle,
        door,
        elem_a="hinge_pin",
        elem_b="door_knuckle_2",
        reason="The fixed steel hinge pin intentionally passes through the door-side hinge knuckles.",
    )

    with ctx.pose({door_hinge: 0.0, handle_turn: 0.0}):
        ctx.expect_within(
            cubicle,
            door,
            axes="xy",
            inner_elem="hinge_pin",
            outer_elem="door_knuckle_0",
            margin=0.001,
            name="hinge pin is captured inside lower knuckle",
        )
        ctx.expect_within(
            cubicle,
            door,
            axes="xy",
            inner_elem="hinge_pin",
            outer_elem="door_knuckle_1",
            margin=0.001,
            name="hinge pin is captured inside a door knuckle",
        )
        ctx.expect_overlap(
            cubicle,
            door,
            axes="z",
            elem_a="hinge_pin",
            elem_b="door_knuckle_1",
            min_overlap=0.20,
            name="hinge pin spans the center knuckle",
        )
        ctx.expect_within(
            cubicle,
            door,
            axes="xy",
            inner_elem="hinge_pin",
            outer_elem="door_knuckle_2",
            margin=0.001,
            name="hinge pin is captured inside upper knuckle",
        )
        ctx.expect_gap(
            door,
            cubicle,
            axis="y",
            positive_elem="door_panel",
            negative_elem="right_jamb",
            min_gap=0.004,
            max_gap=0.030,
            name="closed door sits proud of front frame",
        )
        ctx.expect_overlap(
            door,
            cubicle,
            axes="xy",
            elem_a="door_knuckle_1",
            elem_b="body_knuckle_1",
            min_overlap=0.055,
            name="hinge knuckles share a vertical pin line",
        )
        ctx.expect_overlap(
            bolt_1,
            cubicle,
            axes="yz",
            elem_a="bolt_bar",
            elem_b="keeper_1_bridge",
            min_overlap=0.020,
            name="middle bolt aligns with keeper slot",
        )
        locked_bolt_aabb = ctx.part_element_world_aabb(bolt_1, elem="bolt_bar")
        closed_door_aabb = ctx.part_element_world_aabb(door, elem="door_panel")

    with ctx.pose({handle_turn: math.pi / 2.0}):
        retracted_bolt_aabb = ctx.part_element_world_aabb(bolt_1, elem="bolt_bar")

    ctx.check(
        "turning handle retracts draw bolts",
        locked_bolt_aabb is not None
        and retracted_bolt_aabb is not None
        and retracted_bolt_aabb[1][0] < locked_bolt_aabb[1][0] - 0.150,
        details=f"locked={locked_bolt_aabb}, retracted={retracted_bolt_aabb}",
    )

    with ctx.pose({door_hinge: 1.20, handle_turn: math.pi / 2.0}):
        opened_door_aabb = ctx.part_element_world_aabb(door, elem="door_panel")

    ctx.check(
        "inspection door swings outward from left hinge",
        closed_door_aabb is not None
        and opened_door_aabb is not None
        and opened_door_aabb[1][1] > closed_door_aabb[1][1] + 0.35,
        details=f"closed={closed_door_aabb}, opened={opened_door_aabb}",
    )

    ctx.expect_contact(
        handle,
        door,
        elem_a="turning_hub",
        elem_b="handle_boss",
        contact_tol=0.002,
        name="turning handle seats on the door boss",
    )

    return ctx.report()


object_model = build_object_model()
