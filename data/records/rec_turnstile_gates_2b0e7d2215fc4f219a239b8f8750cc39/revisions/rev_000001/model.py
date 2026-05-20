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
    model = ArticulatedObject(name="three_arm_turnstile_gate")

    painted_steel = Material("dark_powder_coated_steel", color=(0.08, 0.09, 0.10, 1.0))
    brushed_steel = Material("brushed_stainless_steel", color=(0.62, 0.64, 0.62, 1.0))
    black_rubber = Material("black_rubber", color=(0.01, 0.01, 0.01, 1.0))
    safety_yellow = Material("safety_yellow_caps", color=(1.0, 0.74, 0.08, 1.0))

    frame = model.part("fixed_frame")
    frame.visual(
        Box((1.60, 1.20, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=painted_steel,
        name="floor_plate",
    )

    post_height = 1.70
    post_z = 0.06 + post_height / 2.0
    for ix, x in enumerate((-0.75, 0.75)):
        for iy, y in enumerate((-0.55, 0.55)):
            frame.visual(
                Box((0.06, 0.06, post_height)),
                origin=Origin(xyz=(x, y, post_z)),
                material=painted_steel,
                name=f"corner_post_{ix}_{iy}",
            )

    for iy, y in enumerate((-0.55, 0.55)):
        frame.visual(
            Box((1.56, 0.05, 0.05)),
            origin=Origin(xyz=(0.0, y, 1.76)),
            material=painted_steel,
            name=f"top_side_rail_{iy}",
        )
        frame.visual(
            Box((1.56, 0.035, 0.035)),
            origin=Origin(xyz=(0.0, y, 1.06)),
            material=painted_steel,
            name=f"waist_side_rail_{iy}",
        )

    for ix, x in enumerate((-0.75, 0.75)):
        frame.visual(
            Box((0.05, 1.16, 0.05)),
            origin=Origin(xyz=(x, 0.0, 1.76)),
            material=painted_steel,
            name=f"top_end_rail_{ix}",
        )

    frame.visual(
        Cylinder(radius=0.055, length=1.40),
        origin=Origin(xyz=(0.0, 0.0, 0.76)),
        material=brushed_steel,
        name="central_column",
    )
    frame.visual(
        Cylinder(radius=0.12, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 1.43)),
        material=black_rubber,
        name="bearing_cap",
    )

    rotor = model.part("rotor_hub")
    rotor.visual(
        Cylinder(radius=0.105, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=brushed_steel,
        name="hub_body",
    )
    rotor.visual(
        Cylinder(radius=0.075, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=black_rubber,
        name="top_plug",
    )

    arm_length = 0.64
    arm_radius = 0.022
    for i, yaw in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        direction = (math.cos(yaw), math.sin(yaw), 0.0)
        center = (
            direction[0] * arm_length / 2.0,
            direction[1] * arm_length / 2.0,
            0.0,
        )
        tip = (
            direction[0] * arm_length,
            direction[1] * arm_length,
            0.0,
        )
        rotor.visual(
            Cylinder(radius=arm_radius, length=arm_length),
            origin=Origin(xyz=center, rpy=(0.0, math.pi / 2.0, yaw)),
            material=brushed_steel,
            name=f"radial_arm_{i}",
        )
        rotor.visual(
            Sphere(radius=0.035),
            origin=Origin(xyz=tip),
            material=safety_yellow,
            name=f"arm_tip_{i}",
        )

    model.articulation(
        "frame_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 1.53)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.5),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("fixed_frame")
    rotor = object_model.get_part("rotor_hub")
    joint = object_model.get_articulation("frame_to_rotor")

    ctx.check(
        "rotor uses a continuous vertical joint",
        joint.articulation_type == ArticulationType.CONTINUOUS and tuple(joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={joint.articulation_type}, axis={joint.axis}",
    )
    ctx.expect_gap(
        rotor,
        frame,
        axis="z",
        positive_elem="hub_body",
        negative_elem="bearing_cap",
        max_gap=0.001,
        max_penetration=0.0,
        name="small hub sits on the top bearing cap",
    )
    ctx.expect_within(
        rotor,
        frame,
        axes="xy",
        inner_elem="radial_arm_0",
        outer_elem="floor_plate",
        margin=0.02,
        name="radial arm sweep fits inside the frame footprint",
    )

    arm_tip_rest = ctx.part_element_world_aabb(rotor, elem="arm_tip_0")
    with ctx.pose({joint: math.pi / 2.0}):
        arm_tip_turned = ctx.part_element_world_aabb(rotor, elem="arm_tip_0")
    ctx.check(
        "continuous rotor turns the arm around the central column",
        arm_tip_rest is not None
        and arm_tip_turned is not None
        and abs(arm_tip_rest[0][0] - arm_tip_turned[0][0]) > 0.45
        and abs(arm_tip_rest[0][1] - arm_tip_turned[0][1]) > 0.45,
        details=f"rest={arm_tip_rest}, turned={arm_tip_turned}",
    )

    return ctx.report()


object_model = build_object_model()
