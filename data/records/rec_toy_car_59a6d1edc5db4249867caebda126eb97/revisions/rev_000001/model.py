from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


def _wheel_visuals(part, *, tire_radius: float, tire_width: float, hub_radius: float, hub_width: float, tire_mat, hub_mat) -> None:
    spin_origin = Origin(rpy=(pi / 2.0, 0.0, 0.0))
    part.visual(
        Cylinder(radius=tire_radius, length=tire_width),
        origin=spin_origin,
        material=tire_mat,
        name="elem_tire",
    )
    part.visual(
        Cylinder(radius=hub_radius, length=tire_width * 0.92),
        origin=spin_origin,
        material=hub_mat,
        name="elem_rim",
    )
    part.visual(
        Cylinder(radius=hub_radius * 0.42, length=hub_width),
        origin=spin_origin,
        material=hub_mat,
        name="elem_hub_cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="toy_tow_truck")

    cab_red = model.material("cab_red", rgba=(0.80, 0.13, 0.12, 1.0))
    deck_grey = model.material("deck_grey", rgba=(0.30, 0.32, 0.35, 1.0))
    chassis_dark = model.material("chassis_dark", rgba=(0.14, 0.15, 0.16, 1.0))
    boom_yellow = model.material("boom_yellow", rgba=(0.95, 0.76, 0.16, 1.0))
    glass_blue = model.material("glass_blue", rgba=(0.54, 0.74, 0.86, 0.45))
    tire_black = model.material("tire_black", rgba=(0.05, 0.05, 0.05, 1.0))
    wheel_silver = model.material("wheel_silver", rgba=(0.72, 0.74, 0.78, 1.0))
    light_amber = model.material("light_amber", rgba=(0.96, 0.67, 0.16, 0.9))

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((0.28, 0.11, 0.13)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
    )

    # Chassis and running gear support
    body.visual(
        Box((0.24, 0.076, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=chassis_dark,
        name="elem_chassis_plate",
    )
    body.visual(
        Box((0.020, 0.090, 0.010)),
        origin=Origin(xyz=(0.130, 0.0, 0.009)),
        material=chassis_dark,
        name="elem_front_bumper",
    )
    body.visual(
        Box((0.020, 0.050, 0.017)),
        origin=Origin(xyz=(0.082, 0.0, 0.0225)),
        material=chassis_dark,
        name="elem_front_axle_hanger",
    )
    body.visual(
        Box((0.020, 0.050, 0.017)),
        origin=Origin(xyz=(-0.082, 0.0, 0.0225)),
        material=chassis_dark,
        name="elem_rear_axle_hanger",
    )
    body.visual(
        Cylinder(radius=0.0035, length=0.094),
        origin=Origin(xyz=(0.082, 0.0, 0.028), rpy=(pi / 2.0, 0.0, 0.0)),
        material=wheel_silver,
        name="elem_front_axle",
    )
    body.visual(
        Cylinder(radius=0.0035, length=0.094),
        origin=Origin(xyz=(-0.082, 0.0, 0.028), rpy=(pi / 2.0, 0.0, 0.0)),
        material=wheel_silver,
        name="elem_rear_axle",
    )

    # Recovery deck
    body.visual(
        Box((0.160, 0.082, 0.022)),
        origin=Origin(xyz=(-0.036, 0.0, 0.025)),
        material=deck_grey,
        name="elem_deck_support",
    )
    body.visual(
        Box((0.170, 0.102, 0.010)),
        origin=Origin(xyz=(-0.038, 0.0, 0.041)),
        material=deck_grey,
        name="elem_deck_plate",
    )
    body.visual(
        Box((0.158, 0.006, 0.014)),
        origin=Origin(xyz=(-0.038, 0.048, 0.048)),
        material=chassis_dark,
        name="elem_left_deck_rail",
    )
    body.visual(
        Box((0.158, 0.006, 0.014)),
        origin=Origin(xyz=(-0.038, -0.048, 0.048)),
        material=chassis_dark,
        name="elem_right_deck_rail",
    )
    body.visual(
        Box((0.012, 0.010, 0.018)),
        origin=Origin(xyz=(-0.109, 0.017, 0.055)),
        material=chassis_dark,
        name="elem_left_boom_post",
    )
    body.visual(
        Box((0.012, 0.010, 0.018)),
        origin=Origin(xyz=(-0.109, -0.017, 0.055)),
        material=chassis_dark,
        name="elem_right_boom_post",
    )
    body.visual(
        Box((0.018, 0.040, 0.008)),
        origin=Origin(xyz=(-0.109, 0.0, 0.046)),
        material=chassis_dark,
        name="elem_boom_mount_bridge",
    )

    # Cab base and shell
    body.visual(
        Box((0.098, 0.100, 0.010)),
        origin=Origin(xyz=(0.072, 0.0, 0.041)),
        material=cab_red,
        name="elem_cab_floor",
    )
    body.visual(
        Box((0.010, 0.100, 0.068)),
        origin=Origin(xyz=(0.027, 0.0, 0.080)),
        material=cab_red,
        name="elem_cab_back_wall",
    )
    body.visual(
        Box((0.098, 0.100, 0.010)),
        origin=Origin(xyz=(0.072, 0.0, 0.119)),
        material=cab_red,
        name="elem_cab_roof",
    )
    body.visual(
        Box((0.010, 0.100, 0.024)),
        origin=Origin(xyz=(0.118, 0.0, 0.058)),
        material=cab_red,
        name="elem_front_lower_panel",
    )
    body.visual(
        Box((0.010, 0.012, 0.044)),
        origin=Origin(xyz=(0.118, 0.044, 0.092)),
        material=cab_red,
        name="elem_front_left_pillar",
    )
    body.visual(
        Box((0.010, 0.012, 0.044)),
        origin=Origin(xyz=(0.118, -0.044, 0.092)),
        material=cab_red,
        name="elem_front_right_pillar",
    )
    body.visual(
        Box((0.010, 0.076, 0.010)),
        origin=Origin(xyz=(0.118, 0.0, 0.109)),
        material=cab_red,
        name="elem_front_brow",
    )
    body.visual(
        Box((0.004, 0.074, 0.034)),
        origin=Origin(xyz=(0.115, 0.0, 0.087)),
        material=glass_blue,
        name="elem_windshield",
    )
    body.visual(
        Box((0.086, 0.008, 0.024)),
        origin=Origin(xyz=(0.075, 0.046, 0.058)),
        material=cab_red,
        name="elem_left_side_lower",
    )
    body.visual(
        Box((0.086, 0.008, 0.010)),
        origin=Origin(xyz=(0.075, 0.046, 0.109)),
        material=cab_red,
        name="elem_left_side_upper",
    )
    body.visual(
        Box((0.078, 0.003, 0.035)),
        origin=Origin(xyz=(0.073, 0.0475, 0.0875)),
        material=glass_blue,
        name="elem_left_side_window",
    )
    body.visual(
        Box((0.063, 0.008, 0.024)),
        origin=Origin(xyz=(0.0865, -0.046, 0.058)),
        material=cab_red,
        name="elem_right_side_front_lower",
    )
    body.visual(
        Box((0.086, 0.008, 0.010)),
        origin=Origin(xyz=(0.075, -0.046, 0.109)),
        material=cab_red,
        name="elem_right_side_upper",
    )
    body.visual(
        Box((0.078, 0.003, 0.035)),
        origin=Origin(xyz=(0.073, -0.0475, 0.0875)),
        material=glass_blue,
        name="elem_right_side_window",
    )
    body.visual(
        Box((0.006, 0.050, 0.014)),
        origin=Origin(xyz=(0.121, 0.0, 0.058)),
        material=deck_grey,
        name="elem_grille",
    )
    body.visual(
        Box((0.004, 0.012, 0.012)),
        origin=Origin(xyz=(0.122, 0.031, 0.058)),
        material=light_amber,
        name="elem_left_headlight",
    )
    body.visual(
        Box((0.004, 0.012, 0.012)),
        origin=Origin(xyz=(0.122, -0.031, 0.058)),
        material=light_amber,
        name="elem_right_headlight",
    )
    body.visual(
        Box((0.040, 0.016, 0.008)),
        origin=Origin(xyz=(0.066, 0.0, 0.128)),
        material=light_amber,
        name="elem_lightbar",
    )

    front_left_wheel = model.part("front_left_wheel")
    front_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.028, length=0.018),
        mass=0.08,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _wheel_visuals(
        front_left_wheel,
        tire_radius=0.028,
        tire_width=0.018,
        hub_radius=0.017,
        hub_width=0.022,
        tire_mat=tire_black,
        hub_mat=wheel_silver,
    )

    front_right_wheel = model.part("front_right_wheel")
    front_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.028, length=0.018),
        mass=0.08,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _wheel_visuals(
        front_right_wheel,
        tire_radius=0.028,
        tire_width=0.018,
        hub_radius=0.017,
        hub_width=0.022,
        tire_mat=tire_black,
        hub_mat=wheel_silver,
    )

    rear_left_wheel = model.part("rear_left_wheel")
    rear_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.028, length=0.018),
        mass=0.08,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _wheel_visuals(
        rear_left_wheel,
        tire_radius=0.028,
        tire_width=0.018,
        hub_radius=0.017,
        hub_width=0.022,
        tire_mat=tire_black,
        hub_mat=wheel_silver,
    )

    rear_right_wheel = model.part("rear_right_wheel")
    rear_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.028, length=0.018),
        mass=0.08,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _wheel_visuals(
        rear_right_wheel,
        tire_radius=0.028,
        tire_width=0.018,
        hub_radius=0.017,
        hub_width=0.022,
        tire_mat=tire_black,
        hub_mat=wheel_silver,
    )

    tow_boom = model.part("tow_boom")
    tow_boom.inertial = Inertial.from_geometry(
        Box((0.130, 0.040, 0.040)),
        mass=0.15,
        origin=Origin(xyz=(-0.065, 0.0, 0.020)),
    )
    tow_boom.visual(
        Box((0.104, 0.024, 0.016)),
        origin=Origin(xyz=(-0.052, 0.0, 0.010)),
        material=boom_yellow,
        name="elem_boom_beam",
    )
    tow_boom.visual(
        Box((0.028, 0.030, 0.024)),
        origin=Origin(xyz=(-0.096, 0.0, 0.024)),
        material=boom_yellow,
        name="elem_boom_head",
    )
    tow_boom.visual(
        Box((0.016, 0.040, 0.010)),
        origin=Origin(xyz=(-0.108, 0.0, 0.010)),
        material=chassis_dark,
        name="elem_boom_crossbar",
    )
    tow_boom.visual(
        Box((0.022, 0.006, 0.010)),
        origin=Origin(xyz=(-0.120, 0.010, 0.002)),
        material=chassis_dark,
        name="elem_left_tow_prong",
    )
    tow_boom.visual(
        Box((0.022, 0.006, 0.010)),
        origin=Origin(xyz=(-0.120, -0.010, 0.002)),
        material=chassis_dark,
        name="elem_right_tow_prong",
    )

    side_access_door = model.part("side_access_door")
    side_access_door.inertial = Inertial.from_geometry(
        Box((0.023, 0.004, 0.022)),
        mass=0.02,
        origin=Origin(xyz=(-0.0115, 0.0, 0.011)),
    )
    side_access_door.visual(
        Box((0.023, 0.004, 0.022)),
        origin=Origin(xyz=(-0.0115, 0.0, 0.011)),
        material=deck_grey,
        name="elem_side_door_panel",
    )
    side_access_door.visual(
        Box((0.012, 0.0015, 0.004)),
        origin=Origin(xyz=(-0.0175, -0.00275, 0.011)),
        material=wheel_silver,
        name="elem_side_door_handle",
    )

    wheel_spin_limits = MotionLimits(effort=0.5, velocity=12.0)
    model.articulation(
        "front_left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=front_left_wheel,
        origin=Origin(xyz=(0.082, 0.056, 0.028)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=wheel_spin_limits,
    )
    model.articulation(
        "front_right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=front_right_wheel,
        origin=Origin(xyz=(0.082, -0.056, 0.028)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=wheel_spin_limits,
    )
    model.articulation(
        "rear_left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=rear_left_wheel,
        origin=Origin(xyz=(-0.082, 0.056, 0.028)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=wheel_spin_limits,
    )
    model.articulation(
        "rear_right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=rear_right_wheel,
        origin=Origin(xyz=(-0.082, -0.056, 0.028)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=wheel_spin_limits,
    )
    model.articulation(
        "body_to_tow_boom",
        ArticulationType.REVOLUTE,
        parent=body,
        child=tow_boom,
        origin=Origin(xyz=(-0.109, 0.0, 0.056)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=2.5, lower=0.0, upper=1.2),
    )
    model.articulation(
        "body_to_side_access_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=side_access_door,
        origin=Origin(xyz=(0.055, -0.046, 0.050)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=2.0, lower=0.0, upper=1.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    wheels = [
        object_model.get_part("front_left_wheel"),
        object_model.get_part("front_right_wheel"),
        object_model.get_part("rear_left_wheel"),
        object_model.get_part("rear_right_wheel"),
    ]
    boom = object_model.get_part("tow_boom")
    door = object_model.get_part("side_access_door")

    wheel_joints = [
        object_model.get_articulation("front_left_wheel_spin"),
        object_model.get_articulation("front_right_wheel_spin"),
        object_model.get_articulation("rear_left_wheel_spin"),
        object_model.get_articulation("rear_right_wheel_spin"),
    ]
    boom_joint = object_model.get_articulation("body_to_tow_boom")
    door_joint = object_model.get_articulation("body_to_side_access_door")

    for joint in wheel_joints:
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name} is continuous axle spin",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and joint.axis == (0.0, 1.0, 0.0)
            and limits is not None
            and limits.lower is None
            and limits.upper is None,
            details=f"type={joint.articulation_type}, axis={joint.axis}, limits={limits}",
        )

    ctx.check(
        "boom hinge pitches upward from rear deck",
        boom_joint.articulation_type == ArticulationType.REVOLUTE
        and boom_joint.axis == (0.0, 1.0, 0.0),
        details=f"type={boom_joint.articulation_type}, axis={boom_joint.axis}",
    )
    ctx.check(
        "side access door hinges vertically",
        door_joint.articulation_type == ArticulationType.REVOLUTE
        and door_joint.axis == (0.0, 0.0, 1.0),
        details=f"type={door_joint.articulation_type}, axis={door_joint.axis}",
    )

    ctx.expect_overlap(
        boom,
        body,
        axes="xy",
        elem_a="elem_boom_beam",
        elem_b="elem_deck_plate",
        min_overlap=0.010,
        name="boom sits over the recovery deck footprint",
    )

    with ctx.pose({boom_joint: 0.0}):
        boom_rest_aabb = ctx.part_world_aabb(boom)
    with ctx.pose({boom_joint: 1.0}):
        boom_up_aabb = ctx.part_world_aabb(boom)
    ctx.check(
        "tow boom lifts upward when opened",
        boom_rest_aabb is not None
        and boom_up_aabb is not None
        and boom_up_aabb[1][2] > boom_rest_aabb[1][2] + 0.055,
        details=f"rest={boom_rest_aabb}, raised={boom_up_aabb}",
    )

    with ctx.pose({door_joint: 0.0}):
        door_rest_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_joint: 1.0}):
        door_open_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "side access door swings outward from the cab side",
        door_rest_aabb is not None
        and door_open_aabb is not None
        and door_open_aabb[0][1] < door_rest_aabb[0][1] - 0.010,
        details=f"rest={door_rest_aabb}, open={door_open_aabb}",
    )

    for wheel in wheels:
        ctx.expect_origin_gap(
            wheel,
            body,
            axis="z",
            min_gap=0.0,
            max_gap=0.05,
            name=f"{wheel.name} stays below the truck body origin",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
