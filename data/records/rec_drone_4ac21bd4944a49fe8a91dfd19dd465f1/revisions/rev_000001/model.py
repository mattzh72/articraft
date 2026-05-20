from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="micro_indoor_quadcopter")

    frame_plastic = model.material("satin_black_plastic", rgba=(0.025, 0.027, 0.030, 1.0))
    arm_plastic = model.material("dark_carbon_composite", rgba=(0.055, 0.057, 0.060, 1.0))
    motor_metal = model.material("brushed_motor_metal", rgba=(0.42, 0.43, 0.44, 1.0))
    rubber = model.material("soft_black_rubber", rgba=(0.010, 0.010, 0.012, 1.0))
    prop_plastic = model.material("translucent_propeller_blue", rgba=(0.36, 0.62, 0.95, 0.62))
    door_plastic = model.material("battery_door_gray", rgba=(0.18, 0.19, 0.20, 1.0))
    label_white = model.material("white_label_print", rgba=(0.85, 0.86, 0.82, 1.0))

    frame = model.part("frame")
    frame.visual(
        Cylinder(radius=0.035, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=frame_plastic,
        name="round_center_frame",
    )
    frame.visual(
        Cylinder(radius=0.024, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=Material("smoked_top_cover", rgba=(0.04, 0.055, 0.065, 0.72)),
        name="top_cover",
    )

    motor_radius = 0.075
    arm_center_radius = 0.045
    arm_length = 0.060
    motor_top_z = 0.016
    motor_angles = [math.radians(45.0 + i * 90.0) for i in range(4)]

    for i, theta in enumerate(motor_angles):
        c = math.cos(theta)
        s = math.sin(theta)
        frame.visual(
            Cylinder(radius=0.0036, length=arm_length),
            origin=Origin(
                xyz=(arm_center_radius * c, arm_center_radius * s, 0.000),
                rpy=(0.0, math.pi / 2.0, theta),
            ),
            material=arm_plastic,
            name=f"arm_{i}",
        )
        frame.visual(
            Cylinder(radius=0.0105, length=0.020),
            origin=Origin(xyz=(motor_radius * c, motor_radius * s, 0.003)),
            material=motor_metal,
            name=f"motor_can_{i}",
        )
        frame.visual(
            Cylinder(radius=0.0082, length=0.003),
            origin=Origin(xyz=(motor_radius * c, motor_radius * s, 0.0145)),
            material=frame_plastic,
            name=f"motor_cap_{i}",
        )
        frame.visual(
            Cylinder(radius=0.0060, length=0.004),
            origin=Origin(xyz=(motor_radius * c, motor_radius * s, -0.009)),
            material=rubber,
            name=f"foot_{i}",
        )

    # Fixed outer hinge knuckles and their small bosses are part of the frame;
    # the door carries the central knuckle and swings about the same line.
    hinge_y = -0.018
    hinge_z = -0.0115
    for side, x in enumerate((-0.030, 0.030)):
        frame.visual(
            Box((0.010, 0.010, 0.004)),
            origin=Origin(xyz=(x, hinge_y, -0.0095)),
            material=frame_plastic,
            name=f"hinge_boss_{side}",
        )
        frame.visual(
            Cylinder(radius=0.0022, length=0.010),
            origin=Origin(xyz=(x, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=frame_plastic,
            name=f"fixed_knuckle_{side}",
        )

    rotor = FanRotorGeometry(
        outer_radius=0.032,
        hub_radius=0.0065,
        blade_count=3,
        thickness=0.005,
        blade_pitch_deg=30.0,
        blade_sweep_deg=22.0,
        blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=12.0, camber=0.12),
        hub=FanRotorHub(style="spinner", rear_collar_height=0.0015, rear_collar_radius=0.0045),
    )
    rotor_mesh = mesh_from_geometry(rotor, "micro_propeller_rotor")

    for i, theta in enumerate(motor_angles):
        c = math.cos(theta)
        s = math.sin(theta)
        propeller = model.part(f"propeller_{i}")
        propeller.visual(
            rotor_mesh,
            origin=Origin(),
            material=prop_plastic,
            name="rotor",
        )
        model.articulation(
            f"motor_to_propeller_{i}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=propeller,
            origin=Origin(xyz=(motor_radius * c, motor_radius * s, motor_top_z + 0.00247)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.03, velocity=700.0),
        )

    door = model.part("battery_door")
    door.visual(
        Box((0.044, 0.032, 0.0024)),
        origin=Origin(xyz=(0.0, 0.018, 0.0018)),
        material=door_plastic,
        name="door_panel",
    )
    door.visual(
        Box((0.018, 0.0015, 0.0006)),
        origin=Origin(xyz=(0.0, 0.030, 0.0033)),
        material=label_white,
        name="polarity_mark",
    )
    door.visual(
        Box((0.030, 0.009, 0.0020)),
        origin=Origin(xyz=(0.0, 0.003, 0.0007)),
        material=door_plastic,
        name="hinge_leaf",
    )
    door.visual(
        Cylinder(radius=0.0022, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=door_plastic,
        name="door_knuckle",
    )
    door.visual(
        Box((0.012, 0.003, 0.0013)),
        origin=Origin(xyz=(0.0, 0.034, 0.0016)),
        material=rubber,
        name="latch_tab",
    )
    model.articulation(
        "frame_to_battery_door",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=door,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.12, velocity=3.0, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    door = object_model.get_part("battery_door")
    door_hinge = object_model.get_articulation("frame_to_battery_door")

    propeller_joints = [
        object_model.get_articulation(f"motor_to_propeller_{i}") for i in range(4)
    ]
    ctx.check(
        "four continuous propeller joints",
        len(propeller_joints) == 4
        and all(j.articulation_type == ArticulationType.CONTINUOUS for j in propeller_joints),
    )
    ctx.check(
        "battery door hinge opens from closed to service angle",
        door_hinge.articulation_type == ArticulationType.REVOLUTE
        and door_hinge.motion_limits is not None
        and door_hinge.motion_limits.lower == 0.0
        and door_hinge.motion_limits.upper is not None
        and 1.2 <= door_hinge.motion_limits.upper <= 1.5,
    )

    ctx.expect_gap(
        frame,
        door,
        axis="z",
        positive_elem="round_center_frame",
        negative_elem="door_panel",
        min_gap=0.0,
        max_gap=0.0015,
        name="closed battery door sits just below frame underside",
    )

    for i in range(4):
        propeller = object_model.get_part(f"propeller_{i}")
        ctx.expect_gap(
            propeller,
            frame,
            axis="z",
            positive_elem="rotor",
            negative_elem=f"motor_cap_{i}",
            max_gap=0.004,
            max_penetration=0.00001,
            name=f"propeller_{i} seats on its motor cap",
        )

    rest_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({door_hinge: 1.1}):
        open_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    ctx.check(
        "battery door swings downward when opened",
        rest_aabb is not None
        and open_aabb is not None
        and open_aabb[0][2] < rest_aabb[0][2] - 0.015,
        details=f"closed={rest_aabb}, opened={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
