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
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="boxy_toy_suv")

    body_plastic = Material("molded_blue_plastic", color=(0.05, 0.20, 0.85, 1.0))
    dark_plastic = Material("smoked_black_plastic", color=(0.02, 0.025, 0.03, 1.0))
    tire_rubber = Material("soft_black_tires", color=(0.005, 0.005, 0.004, 1.0))
    wheel_silver = Material("silver_hubcaps", color=(0.72, 0.72, 0.68, 1.0))
    axle_gray = Material("matte_axle_gray", color=(0.42, 0.42, 0.40, 1.0))
    lamp_red = Material("red_tail_lamp", color=(0.8, 0.03, 0.02, 1.0))
    lamp_clear = Material("cream_head_lamp", color=(1.0, 0.86, 0.45, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.310, 0.130, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=body_plastic,
        name="body_base",
    )
    body.visual(
        Box((0.185, 0.115, 0.055)),
        origin=Origin(xyz=(-0.010, 0.0, 0.135)),
        material=body_plastic,
        name="cabin_block",
    )
    body.visual(
        Box((0.205, 0.123, 0.012)),
        origin=Origin(xyz=(-0.012, 0.0, 0.166)),
        material=body_plastic,
        name="flat_roof",
    )
    body.visual(
        Box((0.084, 0.122, 0.020)),
        origin=Origin(xyz=(0.108, 0.0, 0.124)),
        material=body_plastic,
        name="square_hood",
    )
    body.visual(
        Box((0.018, 0.150, 0.022)),
        origin=Origin(xyz=(0.164, 0.0, 0.060)),
        material=dark_plastic,
        name="front_bumper",
    )
    body.visual(
        Box((0.018, 0.145, 0.020)),
        origin=Origin(xyz=(-0.164, 0.0, 0.060)),
        material=dark_plastic,
        name="rear_bumper",
    )
    body.visual(
        Box((0.004, 0.082, 0.024)),
        origin=Origin(xyz=(0.1565, 0.0, 0.091)),
        material=dark_plastic,
        name="front_grille",
    )
    body.visual(
        Box((0.004, 0.020, 0.012)),
        origin=Origin(xyz=(0.1565, 0.044, 0.095)),
        material=lamp_clear,
        name="headlamp_0",
    )
    body.visual(
        Box((0.004, 0.020, 0.012)),
        origin=Origin(xyz=(0.1565, -0.044, 0.095)),
        material=lamp_clear,
        name="headlamp_1",
    )
    body.visual(
        Box((0.004, 0.014, 0.020)),
        origin=Origin(xyz=(-0.157, 0.055, 0.088)),
        material=lamp_red,
        name="tail_lamp_0",
    )
    body.visual(
        Box((0.004, 0.014, 0.020)),
        origin=Origin(xyz=(-0.157, -0.055, 0.088)),
        material=lamp_red,
        name="tail_lamp_1",
    )

    # Dark glazing is inset as shallow plaques overlapping the cabin casting so
    # the shell still reads as one robust toy-vehicle body.
    body.visual(
        Box((0.140, 0.004, 0.027)),
        origin=Origin(xyz=(-0.020, 0.0585, 0.136)),
        material=dark_plastic,
        name="side_window_0",
    )
    body.visual(
        Box((0.140, 0.004, 0.027)),
        origin=Origin(xyz=(-0.020, -0.0585, 0.136)),
        material=dark_plastic,
        name="side_window_1",
    )
    body.visual(
        Box((0.004, 0.094, 0.030)),
        origin=Origin(xyz=(0.084, 0.0, 0.136)),
        material=dark_plastic,
        name="windshield",
    )

    # Four chunky U-shaped fender flares, built from connected bars that overlap
    # the body side casting but leave clear wheel openings.
    for side_index, side in enumerate((1.0, -1.0)):
        for axle_index, x_center in enumerate((0.100, -0.100)):
            suffix = f"{side_index}_{axle_index}"
            body.visual(
                Box((0.090, 0.016, 0.012)),
                origin=Origin(xyz=(x_center, side * 0.072, 0.083)),
                material=body_plastic,
                name=f"arch_top_{suffix}",
            )
            body.visual(
                Box((0.012, 0.016, 0.045)),
                origin=Origin(xyz=(x_center + 0.045, side * 0.072, 0.058)),
                material=body_plastic,
                name=f"arch_front_{suffix}",
            )
            body.visual(
                Box((0.012, 0.016, 0.045)),
                origin=Origin(xyz=(x_center - 0.045, side * 0.072, 0.058)),
                material=body_plastic,
                name=f"arch_rear_{suffix}",
            )

    # Body-side hinge lands give the moving doors visible support without
    # embedding the door panels into the shell.
    for side_index, side in enumerate((1.0, -1.0)):
        for door_index, hinge_x in enumerate((0.052, -0.005)):
            body.visual(
                Box((0.006, 0.004, 0.050)),
                origin=Origin(xyz=(hinge_x, side * 0.067, 0.096)),
                material=dark_plastic,
                name=f"hinge_land_{side_index}_{door_index}",
            )

    def add_side_door(
        name: str,
        hinge_x: float,
        side: float,
        length: float,
        axis_z: float,
    ) -> None:
        door = model.part(name)
        door.visual(
            Box((length, 0.006, 0.052)),
            origin=Origin(xyz=(-length / 2.0, side * 0.004, 0.0)),
            material=body_plastic,
            name="door_panel",
        )
        door.visual(
            Cylinder(radius=0.0025, length=0.052),
            origin=Origin(xyz=(0.0, side * 0.004, 0.0)),
            material=dark_plastic,
            name="hinge_barrel",
        )
        door.visual(
            Box((0.014, 0.003, 0.005)),
            origin=Origin(xyz=(-0.68 * length, side * 0.008, -0.004)),
            material=dark_plastic,
            name="pull_handle",
        )
        model.articulation(
            f"body_to_{name}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=door,
            origin=Origin(xyz=(hinge_x, side * 0.068, 0.096)),
            axis=(0.0, 0.0, axis_z),
            motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.05),
        )

    add_side_door("front_door_0", hinge_x=0.052, side=1.0, length=0.055, axis_z=-1.0)
    add_side_door("front_door_1", hinge_x=0.052, side=-1.0, length=0.055, axis_z=1.0)
    add_side_door("rear_door_0", hinge_x=-0.005, side=1.0, length=0.047, axis_z=-1.0)
    add_side_door("rear_door_1", hinge_x=-0.005, side=-1.0, length=0.047, axis_z=1.0)

    hatch = model.part("hatch")
    hatch.visual(
        Box((0.006, 0.120, 0.075)),
        origin=Origin(xyz=(-0.004, 0.0, -0.0375)),
        material=body_plastic,
        name="hatch_panel",
    )
    hatch.visual(
        Cylinder(radius=0.0035, length=0.124),
        origin=Origin(xyz=(-0.004, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="top_hinge_barrel",
    )
    hatch.visual(
        Box((0.002, 0.070, 0.026)),
        origin=Origin(xyz=(-0.008, 0.0, -0.026)),
        material=dark_plastic,
        name="rear_window",
    )
    hatch.visual(
        Box((0.003, 0.030, 0.006)),
        origin=Origin(xyz=(-0.008, 0.0, -0.070)),
        material=dark_plastic,
        name="hatch_handle",
    )
    model.articulation(
        "body_to_hatch",
        ArticulationType.REVOLUTE,
        parent=body,
        child=hatch,
        origin=Origin(xyz=(-0.158, 0.0, 0.150)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=1.5, lower=0.0, upper=1.20),
    )

    front_axle = model.part("front_axle")
    front_axle.visual(
        Cylinder(radius=0.005, length=0.176),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=axle_gray,
        name="front_axle_bar",
    )
    front_axle.visual(
        Cylinder(radius=0.006, length=0.015),
        origin=Origin(xyz=(0.0, 0.0, 0.0075)),
        material=axle_gray,
        name="steering_pivot_post",
    )
    front_axle.visual(
        Box((0.030, 0.030, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=axle_gray,
        name="steering_yoke_plate",
    )
    model.articulation(
        "body_to_front_axle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=front_axle,
        origin=Origin(xyz=(0.100, 0.0, 0.040)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=-0.30, upper=0.30),
    )

    rear_axle = model.part("rear_axle")
    rear_axle.visual(
        Cylinder(radius=0.005, length=0.176),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=axle_gray,
        name="rear_axle_bar",
    )
    rear_axle.visual(
        Cylinder(radius=0.006, length=0.015),
        origin=Origin(xyz=(0.0, 0.0, 0.0075)),
        material=axle_gray,
        name="rear_axle_post",
    )
    model.articulation(
        "body_to_rear_axle",
        ArticulationType.FIXED,
        parent=body,
        child=rear_axle,
        origin=Origin(xyz=(-0.100, 0.0, 0.040)),
    )

    def add_wheel(name: str, parent, joint_name: str, y: float, axle_bar_name: str) -> None:
        wheel = model.part(name)
        wheel.visual(
            Cylinder(radius=0.035, length=0.026),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=tire_rubber,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.018, length=0.026),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=wheel_silver,
            name="hub",
        )
        wheel.visual(
            Cylinder(radius=0.008, length=0.028),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_plastic,
            name="center_cap",
        )
        model.articulation(
            joint_name,
            ArticulationType.CONTINUOUS,
            parent=parent,
            child=wheel,
            origin=Origin(xyz=(0.0, y, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=20.0),
            meta={"axle_bar": axle_bar_name},
        )

    add_wheel("front_wheel_0", front_axle, "front_axle_to_wheel_0", 0.101, "front_axle_bar")
    add_wheel("front_wheel_1", front_axle, "front_axle_to_wheel_1", -0.101, "front_axle_bar")
    add_wheel("rear_wheel_0", rear_axle, "rear_axle_to_wheel_0", 0.101, "rear_axle_bar")
    add_wheel("rear_wheel_1", rear_axle, "rear_axle_to_wheel_1", -0.101, "rear_axle_bar")

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    front_axle = object_model.get_part("front_axle")
    rear_axle = object_model.get_part("rear_axle")
    hatch = object_model.get_part("hatch")
    steering = object_model.get_articulation("body_to_front_axle")

    ctx.expect_contact(
        front_axle,
        body,
        elem_a="steering_pivot_post",
        elem_b="body_base",
        contact_tol=0.001,
        name="front steering post reaches body casting",
    )
    ctx.expect_contact(
        rear_axle,
        body,
        elem_a="rear_axle_post",
        elem_b="body_base",
        contact_tol=0.001,
        name="rear axle post reaches body casting",
    )

    for wheel_name, axle, axle_elem in (
        ("front_wheel_0", front_axle, "front_axle_bar"),
        ("front_wheel_1", front_axle, "front_axle_bar"),
        ("rear_wheel_0", rear_axle, "rear_axle_bar"),
        ("rear_wheel_1", rear_axle, "rear_axle_bar"),
    ):
        wheel = object_model.get_part(wheel_name)
        ctx.expect_contact(
            axle,
            wheel,
            elem_a=axle_elem,
            elem_b="tire",
            contact_tol=0.001,
            name=f"{wheel_name} is seated on axle end",
        )

    # Closed panels sit just outside the robust toy casting instead of being
    # fused into it; their hinges provide the support path.
    ctx.expect_gap(
        "front_door_0",
        body,
        axis="y",
        positive_elem="door_panel",
        negative_elem="body_base",
        max_gap=0.006,
        max_penetration=0.0,
    )
    ctx.expect_gap(
        body,
        "front_door_1",
        axis="y",
        positive_elem="body_base",
        negative_elem="door_panel",
        max_gap=0.006,
        max_penetration=0.0,
    )
    ctx.expect_gap(
        "rear_door_0",
        body,
        axis="y",
        positive_elem="door_panel",
        negative_elem="body_base",
        max_gap=0.006,
        max_penetration=0.0,
    )
    ctx.expect_gap(
        body,
        "rear_door_1",
        axis="y",
        positive_elem="body_base",
        negative_elem="door_panel",
        max_gap=0.006,
        max_penetration=0.0,
    )
    ctx.expect_gap(
        body,
        hatch,
        axis="x",
        positive_elem="body_base",
        negative_elem="hatch_panel",
        max_gap=0.006,
        max_penetration=0.0,
    )

    rest_wheel_pos = ctx.part_world_position("front_wheel_0")
    with ctx.pose({steering: 0.30}):
        steered_wheel_pos = ctx.part_world_position("front_wheel_0")
    ctx.check(
        "front axle yaws about central pivot",
        rest_wheel_pos is not None
        and steered_wheel_pos is not None
        and steered_wheel_pos[0] < rest_wheel_pos[0] - 0.020,
        details=f"rest={rest_wheel_pos}, steered={steered_wheel_pos}",
    )

    rest_hatch_aabb = ctx.part_element_world_aabb(hatch, elem="hatch_panel")
    with ctx.pose({"body_to_hatch": 1.0}):
        raised_hatch_aabb = ctx.part_element_world_aabb(hatch, elem="hatch_panel")
    ctx.check(
        "raised hatch swings upward from top hinge",
        rest_hatch_aabb is not None
        and raised_hatch_aabb is not None
        and raised_hatch_aabb[0][2] > rest_hatch_aabb[0][2] + 0.020,
        details=f"rest={rest_hatch_aabb}, raised={raised_hatch_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
