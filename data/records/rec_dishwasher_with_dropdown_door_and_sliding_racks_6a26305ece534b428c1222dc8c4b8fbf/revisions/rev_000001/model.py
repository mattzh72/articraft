from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


WHITE = Material("warm_white_plastic", rgba=(0.88, 0.90, 0.88, 1.0))
STAINLESS = Material("brushed_stainless", rgba=(0.62, 0.66, 0.67, 1.0))
WIRE = Material("polished_wire", rgba=(0.78, 0.81, 0.82, 1.0))
DARK = Material("dark_gasket", rgba=(0.04, 0.045, 0.045, 1.0))
GLASS = Material("smoked_window", rgba=(0.18, 0.34, 0.46, 0.36))
BLACK = Material("black_control", rgba=(0.02, 0.02, 0.025, 1.0))
BLUE = Material("blue_button", rgba=(0.10, 0.28, 0.72, 1.0))
GREY = Material("soft_grey", rgba=(0.42, 0.45, 0.46, 1.0))


def _box(part, name: str, size, xyz, material):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _cyl(part, name: str, radius: float, length: float, xyz, material, rpy=(0.0, 0.0, 0.0)):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_dishwasher")

    for material in (WHITE, STAINLESS, WIRE, DARK, GLASS, BLACK, BLUE, GREY):
        model.material(material.name, rgba=material.rgba)

    cabinet = model.part("cabinet")

    # Compact countertop appliance shell: wide hollow wash bay with a narrow
    # right-side front control pod.
    _box(cabinet, "bottom_plinth", (0.50, 0.57, 0.035), (0.0, 0.015, 0.0175), WHITE)
    _box(cabinet, "top_cap", (0.50, 0.57, 0.025), (0.0, 0.015, 0.3875), WHITE)
    _box(cabinet, "back_wall", (0.025, 0.57, 0.35), (0.2375, 0.015, 0.2075), WHITE)
    _box(cabinet, "left_wall", (0.50, 0.025, 0.38), (0.0, -0.265, 0.200), WHITE)
    _box(cabinet, "right_wall", (0.50, 0.025, 0.38), (0.0, 0.295, 0.200), WHITE)
    _box(cabinet, "control_divider", (0.50, 0.018, 0.34), (0.0, 0.145, 0.205), WHITE)
    _box(cabinet, "pod_front_panel", (0.025, 0.130, 0.030), (-0.2375, 0.225, 0.365), WHITE)
    _box(cabinet, "pod_lower_panel", (0.025, 0.130, 0.040), (-0.2375, 0.225, 0.055), WHITE)
    _box(cabinet, "pod_side_frame_0", (0.025, 0.014, 0.340), (-0.2375, 0.160, 0.205), WHITE)
    _box(cabinet, "pod_side_frame_1", (0.025, 0.014, 0.340), (-0.2375, 0.290, 0.205), WHITE)
    _box(cabinet, "pod_separator_0", (0.025, 0.130, 0.014), (-0.2375, 0.225, 0.242), WHITE)
    _box(cabinet, "pod_separator_1", (0.025, 0.130, 0.012), (-0.2375, 0.225, 0.177), WHITE)
    _box(cabinet, "dial_socket_top", (0.025, 0.130, 0.012), (-0.2375, 0.225, 0.326), DARK)
    _box(cabinet, "dial_socket_bottom", (0.025, 0.130, 0.010), (-0.2375, 0.225, 0.247), DARK)
    _box(cabinet, "button_socket_0", (0.025, 0.130, 0.010), (-0.2375, 0.225, 0.226), DARK)
    _box(cabinet, "button_socket_1", (0.025, 0.130, 0.010), (-0.2375, 0.225, 0.166), DARK)

    # Front wash-chamber frame around the drop door; the center is intentionally
    # open so the stainless cavity, rack, and spray arm read as a hollow chamber.
    _box(cabinet, "front_top_lip", (0.025, 0.400, 0.035), (-0.2375, -0.055, 0.3575), WHITE)
    _box(cabinet, "front_bottom_sill", (0.025, 0.400, 0.035), (-0.2375, -0.055, 0.0525), WHITE)
    _box(cabinet, "front_side_lip_0", (0.025, 0.025, 0.320), (-0.2375, -0.255, 0.200), WHITE)
    _box(cabinet, "front_side_lip_1", (0.025, 0.025, 0.320), (-0.2375, 0.145, 0.200), WHITE)

    # Thin stainless liners are deliberately separate visible surfaces mounted
    # inside the shell, leaving real air space in the wash bay.
    _box(cabinet, "stainless_floor", (0.420, 0.380, 0.006), (-0.015, -0.055, 0.040), STAINLESS)
    _box(cabinet, "stainless_back", (0.006, 0.380, 0.305), (0.222, -0.055, 0.205), STAINLESS)
    _box(cabinet, "stainless_left", (0.430, 0.006, 0.305), (-0.010, -0.246, 0.205), STAINLESS)
    _box(cabinet, "stainless_right", (0.430, 0.006, 0.305), (-0.010, 0.136, 0.205), STAINLESS)
    _box(cabinet, "stainless_ceiling", (0.420, 0.380, 0.006), (-0.015, -0.055, 0.362), STAINLESS)
    _box(cabinet, "black_door_gasket", (0.006, 0.362, 0.270), (-0.239, -0.055, 0.205), DARK)

    # Fixed telescoping runner channels mounted to the chamber side walls.
    _box(cabinet, "fixed_runner_0", (0.340, 0.018, 0.012), (-0.020, -0.228, 0.097), STAINLESS)
    _box(cabinet, "fixed_runner_1", (0.340, 0.018, 0.012), (-0.020, 0.118, 0.097), STAINLESS)
    _box(cabinet, "runner_wall_brace_0", (0.060, 0.018, 0.012), (-0.020, -0.237, 0.097), STAINLESS)
    _box(cabinet, "runner_wall_brace_1", (0.060, 0.018, 0.012), (-0.020, 0.127, 0.097), STAINLESS)
    _box(cabinet, "runner_stop_0", (0.012, 0.020, 0.032), (-0.236, -0.228, 0.110), STAINLESS)
    _box(cabinet, "runner_stop_1", (0.012, 0.020, 0.032), (-0.236, 0.118, 0.110), STAINLESS)

    _cyl(cabinet, "spray_bearing", 0.024, 0.014, (-0.035, -0.055, 0.049), STAINLESS)
    _cyl(cabinet, "drain_screen", 0.048, 0.004, (0.070, -0.055, 0.044), DARK)
    _box(cabinet, "rear_water_tower", (0.028, 0.050, 0.195), (0.205, -0.055, 0.145), STAINLESS)
    _box(cabinet, "countertop_feet_0", (0.055, 0.055, 0.018), (-0.185, -0.210, -0.009), DARK)
    _box(cabinet, "countertop_feet_1", (0.055, 0.055, 0.018), (-0.185, 0.245, -0.009), DARK)
    _box(cabinet, "countertop_feet_2", (0.055, 0.055, 0.018), (0.185, -0.210, -0.009), DARK)
    _box(cabinet, "countertop_feet_3", (0.055, 0.055, 0.018), (0.185, 0.245, -0.009), DARK)

    # Wide lower drop door.  Its frame leaves a translucent window rather than a
    # solid slab, keeping the wash chamber readable in the closed pose.
    door = model.part("front_door")
    _box(door, "door_side_0", (0.018, 0.036, 0.310), (-0.009, -0.182, 0.155), WHITE)
    _box(door, "door_side_1", (0.018, 0.036, 0.310), (-0.009, 0.182, 0.155), WHITE)
    _box(door, "door_bottom_rail", (0.018, 0.400, 0.036), (-0.009, 0.0, 0.018), WHITE)
    _box(door, "door_top_rail", (0.018, 0.400, 0.036), (-0.009, 0.0, 0.292), WHITE)
    _box(door, "window_glass", (0.006, 0.328, 0.224), (-0.020, 0.0, 0.162), GLASS)
    _box(door, "inner_steel_skin", (0.004, 0.352, 0.252), (0.001, 0.0, 0.160), STAINLESS)
    _cyl(door, "door_hinge_barrel", 0.008, 0.145, (-0.004, 0.0, 0.000), WHITE, rpy=(-math.pi / 2, 0.0, 0.0))
    _box(door, "handle_post_0", (0.036, 0.014, 0.028), (-0.036, -0.095, 0.246), WHITE)
    _box(door, "handle_post_1", (0.036, 0.014, 0.028), (-0.036, 0.095, 0.246), WHITE)
    _cyl(door, "front_handle", 0.011, 0.220, (-0.055, 0.0, 0.258), WHITE, rpy=(-math.pi / 2, 0.0, 0.0))
    model.articulation(
        "cabinet_to_front_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(-0.250, -0.055, 0.065)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=1.65),
    )

    # Deep lower rack on moving runner members.
    rack = model.part("dish_rack")
    _box(rack, "moving_runner_0", (0.360, 0.013, 0.012), (0.0, -0.173, -0.016), WIRE)
    _box(rack, "moving_runner_1", (0.360, 0.013, 0.012), (0.0, 0.173, -0.016), WIRE)
    for i, y in enumerate((-0.160, -0.080, 0.000, 0.080, 0.160)):
        _box(rack, f"base_rail_x_{i}", (0.320, 0.005, 0.005), (0.0, y, 0.000), WIRE)
    for i, x in enumerate((-0.150, -0.075, 0.000, 0.075, 0.150)):
        _box(rack, f"base_rail_y_{i}", (0.005, 0.320, 0.005), (x, 0.0, 0.004), WIRE)
    for i, y in enumerate((-0.160, 0.160)):
        _box(rack, f"side_lower_rail_{i}", (0.325, 0.006, 0.006), (0.0, y, 0.045), WIRE)
        _box(rack, f"side_upper_rail_{i}", (0.325, 0.006, 0.006), (0.0, y, 0.095), WIRE)
    for i, x in enumerate((-0.162, 0.162)):
        _box(rack, f"end_lower_rail_{i}", (0.006, 0.340, 0.006), (x, 0.0, 0.045), WIRE)
        _box(rack, f"end_upper_rail_{i}", (0.006, 0.340, 0.006), (x, 0.0, 0.095), WIRE)
    for i, (x, y) in enumerate(((-0.162, -0.160), (-0.162, 0.160), (0.162, -0.160), (0.162, 0.160))):
        _box(rack, f"corner_post_{i}", (0.007, 0.007, 0.100), (x, y, 0.050), WIRE)
    _box(rack, "tine_anchor_front", (0.270, 0.006, 0.005), (0.0, -0.052, 0.001), WIRE)
    _box(rack, "tine_anchor_rear", (0.270, 0.006, 0.005), (0.0, 0.052, 0.001), WIRE)
    for i, x in enumerate((-0.120, -0.080, -0.040, 0.000, 0.040, 0.080, 0.120)):
        _cyl(rack, f"dish_tine_front_{i}", 0.003, 0.098, (x, -0.052, 0.049), WIRE)
        _cyl(rack, f"dish_tine_rear_{i}", 0.003, 0.098, (x, 0.052, 0.049), WIRE)
    for i, y in enumerate((-0.160, 0.160)):
        _box(rack, f"runner_bracket_{i}", (0.034, 0.026, 0.055), (-0.120, y, 0.015), WIRE)
        _box(rack, f"shelf_pivot_mount_{i}", (0.018, 0.026, 0.170), (-0.136, y, 0.085), WIRE)
    model.articulation(
        "cabinet_to_dish_rack",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=rack,
        origin=Origin(xyz=(-0.050, -0.055, 0.125)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.35, lower=0.0, upper=0.180),
    )

    # Smaller upper shelf folding on a pair of visible side pivots.
    shelf = model.part("cup_shelf")
    _box(shelf, "shelf_back_rail", (0.008, 0.292, 0.006), (0.000, 0.0, 0.000), WIRE)
    _box(shelf, "shelf_front_rail", (0.008, 0.292, 0.006), (0.175, 0.0, 0.000), WIRE)
    for i, y in enumerate((-0.145, 0.145)):
        _box(shelf, f"shelf_side_rail_{i}", (0.180, 0.006, 0.006), (0.088, y, 0.000), WIRE)
        _box(shelf, f"pivot_lug_{i}", (0.018, 0.018, 0.012), (0.000, y, 0.000), WIRE)
    for i, x in enumerate((0.035, 0.070, 0.105, 0.140)):
        _box(shelf, f"shelf_wire_{i}", (0.005, 0.292, 0.005), (x, 0.0, 0.003), WIRE)
    _cyl(shelf, "side_pivot_0", 0.007, 0.024, (0.0, -0.164, 0.000), WIRE, rpy=(-math.pi / 2, 0.0, 0.0))
    _cyl(shelf, "side_pivot_1", 0.007, 0.024, (0.0, 0.164, 0.000), WIRE, rpy=(-math.pi / 2, 0.0, 0.0))
    model.articulation(
        "rack_to_cup_shelf",
        ArticulationType.REVOLUTE,
        parent=rack,
        child=shelf,
        origin=Origin(xyz=(-0.120, 0.0, 0.160)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.0, lower=0.0, upper=1.25),
    )

    # Lower spray arm on a continuous central hub.
    spray = model.part("spray_arm")
    _cyl(spray, "spray_hub", 0.021, 0.016, (0.0, 0.0, 0.008), STAINLESS)
    _box(spray, "spray_blade", (0.044, 0.286, 0.012), (0.0, 0.0, 0.020), GREY)
    _box(spray, "spray_blade_tip_0", (0.058, 0.028, 0.010), (0.012, -0.142, 0.023), GREY)
    _box(spray, "spray_blade_tip_1", (0.058, 0.028, 0.010), (-0.012, 0.142, 0.023), GREY)
    for i, y in enumerate((-0.105, -0.055, 0.055, 0.105)):
        _box(spray, f"spray_nozzle_{i}", (0.010, 0.012, 0.004), (0.014 if y < 0 else -0.014, y, 0.028), DARK)
    model.articulation(
        "cabinet_to_spray_arm",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=spray,
        origin=Origin(xyz=(-0.035, -0.055, 0.056)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.2, velocity=8.0),
    )

    # Control pod: a continuously rotating timer dial and two separate
    # prismatic push buttons.
    timer = model.part("timer_dial")
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.066,
            0.022,
            body_style="skirted",
            top_diameter=0.052,
            skirt=KnobSkirt(0.074, 0.006, flare=0.05, chamfer=0.001),
            grip=KnobGrip(style="fluted", count=18, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=90.0),
        ),
        "timer_dial_cap",
    )
    timer.visual(
        knob_mesh,
        origin=Origin(xyz=(-0.011, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=BLACK,
        name="dial_cap",
    )
    _box(timer, "dial_mount_flange", (0.006, 0.082, 0.082), (-0.003, 0.0, 0.0), BLACK)
    _box(timer, "dial_pointer", (0.004, 0.006, 0.030), (-0.024, 0.0, 0.023), WHITE)
    model.articulation(
        "pod_to_timer_dial",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=timer,
        origin=Origin(xyz=(-0.250, 0.225, 0.285)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=3.0),
    )

    for i, z in enumerate((0.205, 0.150)):
        button = model.part(f"mode_button_{i}")
        _box(button, "button_cap", (0.018, 0.052, 0.032), (-0.009, 0.0, 0.0), BLUE if i == 0 else GREY)
        _box(button, "button_stem", (0.010, 0.032, 0.020), (0.005, 0.0, 0.0), DARK)
        model.articulation(
            f"pod_to_mode_button_{i}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=button,
            origin=Origin(xyz=(-0.250, 0.225, z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=0.08, lower=0.0, upper=0.008),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("front_door")
    rack = object_model.get_part("dish_rack")
    shelf = object_model.get_part("cup_shelf")
    spray = object_model.get_part("spray_arm")
    dial = object_model.get_part("timer_dial")
    button_0 = object_model.get_part("mode_button_0")
    button_1 = object_model.get_part("mode_button_1")

    door_joint = object_model.get_articulation("cabinet_to_front_door")
    rack_joint = object_model.get_articulation("cabinet_to_dish_rack")
    shelf_joint = object_model.get_articulation("rack_to_cup_shelf")
    spray_joint = object_model.get_articulation("cabinet_to_spray_arm")
    dial_joint = object_model.get_articulation("pod_to_timer_dial")
    button_joint_0 = object_model.get_articulation("pod_to_mode_button_0")
    button_joint_1 = object_model.get_articulation("pod_to_mode_button_1")

    # Closed door seats on the front frame without hiding the hollow chamber.
    frame_lip = ctx.part_element_world_aabb(cabinet, elem="front_top_lip")
    door_top = ctx.part_element_world_aabb(door, elem="door_top_rail")
    ctx.check(
        "drop door rests flush on front frame",
        frame_lip is not None
        and door_top is not None
        and abs(frame_lip[0][0] - door_top[1][0]) <= 0.0015,
        details=f"frame={frame_lip}, door={door_top}",
    )
    ctx.expect_within(
        rack,
        cabinet,
        axes="yz",
        margin=0.004,
        name="rack fits within hollow wash chamber",
    )
    ctx.expect_gap(
        rack,
        spray,
        axis="z",
        min_gap=0.012,
        name="rack clears rotating lower spray arm",
    )
    ctx.expect_contact(
        rack,
        cabinet,
        contact_tol=0.002,
        name="rack runner rides on fixed side channel",
    )

    rest_rack_pos = ctx.part_world_position(rack)
    with ctx.pose({rack_joint: 0.180}):
        extended_rack_pos = ctx.part_world_position(rack)
        ctx.expect_overlap(
            rack,
            cabinet,
            axes="x",
            min_overlap=0.100,
            name="rack runner remains retained when extended",
        )
    ctx.check(
        "rack slides outward on cabinet depth axis",
        rest_rack_pos is not None
        and extended_rack_pos is not None
        and extended_rack_pos[0] < rest_rack_pos[0] - 0.150,
        details=f"rest={rest_rack_pos}, extended={extended_rack_pos}",
    )

    closed_top = ctx.part_element_world_aabb(door, elem="door_top_rail")
    with ctx.pose({door_joint: 1.35}):
        open_top = ctx.part_element_world_aabb(door, elem="door_top_rail")
    ctx.check(
        "front door rotates downward from lower hinge",
        closed_top is not None
        and open_top is not None
        and open_top[1][2] < closed_top[0][2] - 0.120
        and open_top[0][0] < closed_top[0][0] - 0.120,
        details=f"closed={closed_top}, open={open_top}",
    )

    shelf_rest = ctx.part_element_world_aabb(shelf, elem="shelf_front_rail")
    with ctx.pose({shelf_joint: 1.0}):
        shelf_folded = ctx.part_element_world_aabb(shelf, elem="shelf_front_rail")
    ctx.check(
        "upper cup shelf folds upward on side pivots",
        shelf_rest is not None
        and shelf_folded is not None
        and shelf_folded[0][2] > shelf_rest[1][2] + 0.060,
        details=f"rest={shelf_rest}, folded={shelf_folded}",
    )

    blade_rest = ctx.part_element_world_aabb(spray, elem="spray_blade")
    with ctx.pose({spray_joint: math.pi / 2}):
        blade_turn = ctx.part_element_world_aabb(spray, elem="spray_blade")
    ctx.check(
        "spray arm spins continuously around center hub",
        blade_rest is not None
        and blade_turn is not None
        and (blade_rest[1][1] - blade_rest[0][1]) > 0.25
        and (blade_turn[1][0] - blade_turn[0][0]) > 0.25,
        details=f"rest={blade_rest}, turned={blade_turn}",
    )

    pointer_rest = ctx.part_element_world_aabb(dial, elem="dial_pointer")
    with ctx.pose({dial_joint: math.pi / 2}):
        pointer_turn = ctx.part_element_world_aabb(dial, elem="dial_pointer")
    ctx.check(
        "timer dial rotates as separate continuous control",
        pointer_rest is not None
        and pointer_turn is not None
        and (pointer_rest[1][2] - pointer_rest[0][2]) > 0.025
        and (pointer_turn[1][1] - pointer_turn[0][1]) > 0.025,
        details=f"rest={pointer_rest}, turned={pointer_turn}",
    )

    b0_rest = ctx.part_world_position(button_0)
    b1_rest = ctx.part_world_position(button_1)
    with ctx.pose({button_joint_0: 0.008, button_joint_1: 0.008}):
        b0_pushed = ctx.part_world_position(button_0)
        b1_pushed = ctx.part_world_position(button_1)
    ctx.check(
        "both mode buttons push inward independently",
        b0_rest is not None
        and b1_rest is not None
        and b0_pushed is not None
        and b1_pushed is not None
        and b0_pushed[0] > b0_rest[0] + 0.006
        and b1_pushed[0] > b1_rest[0] + 0.006,
        details=f"b0={b0_rest}->{b0_pushed}, b1={b1_rest}->{b1_pushed}",
    )

    return ctx.report()


object_model = build_object_model()
