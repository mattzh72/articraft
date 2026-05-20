from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="toy_tow_truck_car")

    red = model.material("molded_red_plastic", color=(0.86, 0.05, 0.03, 1.0))
    deck_gray = model.material("deck_gray_plastic", color=(0.35, 0.37, 0.38, 1.0))
    black = model.material("soft_black_rubber", color=(0.01, 0.01, 0.012, 1.0))
    dark_glass = model.material("smoky_blue_windows", color=(0.05, 0.13, 0.20, 0.82))
    silver = model.material("toy_silver_metal", color=(0.74, 0.72, 0.68, 1.0))
    yellow = model.material("amber_lights", color=(1.0, 0.67, 0.08, 1.0))
    white = model.material("white_headlights", color=(1.0, 0.94, 0.76, 1.0))

    chassis = model.part("chassis")

    # One connected toy body: low chassis, boxy cab, and molded recovery deck.
    chassis.visual(
        Box((0.64, 0.26, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=red,
        name="main_chassis",
    )
    chassis.visual(
        Box((0.23, 0.26, 0.10)),
        origin=Origin(xyz=(0.18, 0.0, 0.18)),
        material=red,
        name="cab_lower",
    )
    chassis.visual(
        Box((0.205, 0.25, 0.12)),
        origin=Origin(xyz=(0.18, 0.0, 0.29)),
        material=red,
        name="cab_upper",
    )
    chassis.visual(
        Box((0.34, 0.25, 0.05)),
        origin=Origin(xyz=(-0.13, 0.0, 0.155)),
        material=deck_gray,
        name="recovery_deck",
    )
    chassis.visual(
        Box((0.08, 0.25, 0.11)),
        origin=Origin(xyz=(0.015, 0.0, 0.215)),
        material=red,
        name="cab_bulkhead",
    )
    chassis.visual(
        Box((0.34, 0.032, 0.055)),
        origin=Origin(xyz=(-0.13, 0.126, 0.212)),
        material=red,
        name="deck_rail_side_0",
    )
    chassis.visual(
        Box((0.34, 0.032, 0.055)),
        origin=Origin(xyz=(-0.13, -0.126, 0.212)),
        material=red,
        name="deck_rail_side_1",
    )
    chassis.visual(
        Box((0.035, 0.28, 0.055)),
        origin=Origin(xyz=(0.337, 0.0, 0.105)),
        material=silver,
        name="front_bumper",
    )
    chassis.visual(
        Box((0.04, 0.28, 0.06)),
        origin=Origin(xyz=(-0.34, 0.0, 0.105)),
        material=silver,
        name="rear_bumper",
    )

    # Flush, slightly proud windows and toy details.
    chassis.visual(
        Box((0.012, 0.18, 0.065)),
        origin=Origin(xyz=(0.287, 0.0, 0.295)),
        material=dark_glass,
        name="windshield",
    )
    chassis.visual(
        Box((0.11, 0.006, 0.060)),
        origin=Origin(xyz=(0.19, -0.128, 0.300)),
        material=dark_glass,
        name="side_window",
    )
    chassis.visual(
        Box((0.075, 0.050, 0.025)),
        origin=Origin(xyz=(0.18, 0.0, 0.362)),
        material=yellow,
        name="roof_beacon",
    )
    chassis.visual(
        Box((0.012, 0.055, 0.024)),
        origin=Origin(xyz=(0.357, 0.070, 0.125)),
        material=white,
        name="headlight_0",
    )
    chassis.visual(
        Box((0.012, 0.055, 0.024)),
        origin=Origin(xyz=(0.357, -0.070, 0.125)),
        material=white,
        name="headlight_1",
    )

    # Axles and raised fenders give the four wheel parts visible support.
    for axle_x, axle_name in ((0.21, "front_axle"), (-0.21, "rear_axle")):
        chassis.visual(
            Cylinder(radius=0.014, length=0.296),
            origin=Origin(xyz=(axle_x, 0.0, 0.075), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=silver,
            name=axle_name,
        )
    for x_pos, x_name in ((0.21, "front"), (-0.21, "rear")):
        for y_pos, side in ((0.143, "left"), (-0.143, "right")):
            chassis.visual(
                Box((0.115, 0.050, 0.035)),
                origin=Origin(xyz=(x_pos, y_pos, 0.158)),
                material=red,
                name=f"{x_name}_{side}_fender",
            )

    # A small clevis at the back of the deck visually captures the folding boom.
    chassis.visual(
        Box((0.038, 0.018, 0.070)),
        origin=Origin(xyz=(-0.300, 0.099, 0.215)),
        material=silver,
        name="boom_bracket_0",
    )
    chassis.visual(
        Box((0.038, 0.018, 0.070)),
        origin=Origin(xyz=(-0.300, -0.099, 0.215)),
        material=silver,
        name="boom_bracket_1",
    )

    # Detailed tire/rim meshes, reused by all four continuously rotating wheels.
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.055,
            0.050,
            inner_radius=0.036,
            carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.06),
            tread=TireTread(style="block", depth=0.0035, count=18, land_ratio=0.58),
            grooves=(TireGroove(center_offset=0.0, width=0.004, depth=0.0018),),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.004, radius=0.002),
        ),
        "toy_truck_tire",
    )
    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.039,
            0.044,
            rim=WheelRim(inner_radius=0.025, flange_height=0.004, flange_thickness=0.002),
            hub=WheelHub(
                radius=0.014,
                width=0.034,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=5, circle_diameter=0.020, hole_diameter=0.0025),
            ),
            face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="split_y", count=5, thickness=0.0022, window_radius=0.005),
            bore=WheelBore(style="round", diameter=0.009),
        ),
        "toy_truck_rim",
    )

    wheel_specs = (
        ("front_left_wheel", 0.21, 0.168, math.pi / 2.0),
        ("front_right_wheel", 0.21, -0.168, -math.pi / 2.0),
        ("rear_left_wheel", -0.21, 0.168, math.pi / 2.0),
        ("rear_right_wheel", -0.21, -0.168, -math.pi / 2.0),
    )
    for part_name, x_pos, y_pos, yaw in wheel_specs:
        wheel = model.part(part_name)
        wheel.visual(
            tire_mesh,
            origin=Origin(rpy=(0.0, 0.0, yaw)),
            material=black,
            name="tire",
        )
        wheel.visual(
            wheel_mesh,
            origin=Origin(rpy=(0.0, 0.0, yaw)),
            material=silver,
            name="rim",
        )
        model.articulation(
            f"chassis_to_{part_name}",
            ArticulationType.CONTINUOUS,
            parent=chassis,
            child=wheel,
            origin=Origin(xyz=(x_pos, y_pos, 0.075)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.8, velocity=24.0),
        )

    boom = model.part("tow_boom")
    boom.visual(
        Cylinder(radius=0.016, length=0.180),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="hinge_tube",
    )
    boom.visual(
        Box((0.370, 0.042, 0.032)),
        origin=Origin(xyz=(-0.185, 0.0, 0.016)),
        material=yellow,
        name="boom_beam",
    )
    boom.visual(
        Box((0.035, 0.135, 0.026)),
        origin=Origin(xyz=(-0.345, 0.0, 0.014)),
        material=yellow,
        name="boom_crossbar",
    )
    boom.visual(
        Box((0.010, 0.010, 0.100)),
        origin=Origin(xyz=(-0.355, 0.0, -0.038)),
        material=silver,
        name="tow_cable",
    )
    boom.visual(
        Box((0.040, 0.012, 0.012)),
        origin=Origin(xyz=(-0.355, 0.0, -0.092)),
        material=silver,
        name="tow_hook_foot",
    )
    boom.visual(
        Box((0.012, 0.012, 0.045)),
        origin=Origin(xyz=(-0.378, 0.0, -0.076)),
        material=silver,
        name="tow_hook_tip",
    )
    model.articulation(
        "deck_to_boom",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=boom,
        origin=Origin(xyz=(-0.300, 0.0, 0.220)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=0.0, upper=1.25),
    )

    side_door = model.part("side_door")
    side_door.visual(
        Box((0.115, 0.012, 0.135)),
        origin=Origin(xyz=(0.0575, 0.006, 0.0)),
        material=red,
        name="door_panel",
    )
    side_door.visual(
        Box((0.063, 0.004, 0.044)),
        origin=Origin(xyz=(0.061, 0.014, 0.030)),
        material=dark_glass,
        name="door_window",
    )
    side_door.visual(
        Cylinder(radius=0.006, length=0.155),
        origin=Origin(xyz=(0.0, 0.016, 0.0)),
        material=silver,
        name="door_hinge",
    )
    side_door.visual(
        Box((0.010, 0.006, 0.018)),
        origin=Origin(xyz=(0.100, 0.0145, -0.020)),
        material=silver,
        name="door_handle",
    )
    model.articulation(
        "cab_to_side_door",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=side_door,
        origin=Origin(xyz=(0.102, 0.1300, 0.255)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=3.0, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    wheel_joint_names = (
        "chassis_to_front_left_wheel",
        "chassis_to_front_right_wheel",
        "chassis_to_rear_left_wheel",
        "chassis_to_rear_right_wheel",
    )
    wheel_joints = [object_model.get_articulation(name) for name in wheel_joint_names]
    ctx.check(
        "four wheels use continuous axle joints",
        all(joint.articulation_type == ArticulationType.CONTINUOUS for joint in wheel_joints),
        details=f"wheel joint types={[joint.articulation_type for joint in wheel_joints]}",
    )
    ctx.check(
        "wheel axles are lateral",
        all(tuple(joint.axis) == (0.0, 1.0, 0.0) for joint in wheel_joints),
        details=f"wheel axes={[joint.axis for joint in wheel_joints]}",
    )

    chassis = object_model.get_part("chassis")
    side_door = object_model.get_part("side_door")
    tow_boom = object_model.get_part("tow_boom")
    door_joint = object_model.get_articulation("cab_to_side_door")
    boom_joint = object_model.get_articulation("deck_to_boom")

    ctx.expect_gap(
        side_door,
        chassis,
        axis="y",
        positive_elem="door_panel",
        negative_elem="cab_lower",
        min_gap=0.0,
        max_gap=0.002,
        name="side access door sits just outside cab side",
    )

    rest_boom_aabb = ctx.part_element_world_aabb(tow_boom, elem="boom_beam")
    with ctx.pose({boom_joint: 1.25}):
        raised_boom_aabb = ctx.part_element_world_aabb(tow_boom, elem="boom_beam")
    rest_boom_top = rest_boom_aabb[1][2] if rest_boom_aabb is not None else None
    raised_boom_top = raised_boom_aabb[1][2] if raised_boom_aabb is not None else None
    ctx.check(
        "tow boom folds upward",
        rest_boom_top is not None and raised_boom_top is not None and raised_boom_top > rest_boom_top + 0.20,
        details=f"rest_top={rest_boom_top}, raised_top={raised_boom_top}",
    )

    closed_door_aabb = ctx.part_element_world_aabb(side_door, elem="door_panel")
    with ctx.pose({door_joint: 1.35}):
        open_door_aabb = ctx.part_element_world_aabb(side_door, elem="door_panel")
    closed_door_y = closed_door_aabb[1][1] if closed_door_aabb is not None else None
    open_door_y = open_door_aabb[1][1] if open_door_aabb is not None else None
    ctx.check(
        "side access door swings outward",
        closed_door_y is not None and open_door_y is not None and open_door_y > closed_door_y + 0.06,
        details=f"closed_max_y={closed_door_y}, open_max_y={open_door_y}",
    )

    return ctx.report()


object_model = build_object_model()
