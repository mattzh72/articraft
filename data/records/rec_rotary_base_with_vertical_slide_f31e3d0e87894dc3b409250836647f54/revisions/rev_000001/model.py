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
    model = ArticulatedObject(name="pedestal_rotary_lift")

    dark_steel = model.material("dark_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    blue_steel = model.material("blue_steel", rgba=(0.06, 0.20, 0.48, 1.0))
    zinc = model.material("zinc_plated", rgba=(0.62, 0.66, 0.68, 1.0))
    black = model.material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    orange = model.material("safety_orange", rgba=(0.92, 0.38, 0.06, 1.0))

    foot = model.part("foot")
    foot.visual(
        Cylinder(radius=0.38, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=dark_steel,
        name="floor_disk",
    )
    foot.visual(
        Cylinder(radius=0.112, length=0.160),
        origin=Origin(xyz=(0.0, 0.0, 0.140)),
        material=dark_steel,
        name="pedestal_column",
    )
    foot.visual(
        Cylinder(radius=0.185, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.2025)),
        material=zinc,
        name="bearing_housing",
    )
    for i, (x, y) in enumerate(
        ((0.24, 0.0), (-0.24, 0.0), (0.0, 0.24), (0.0, -0.24))
    ):
        foot.visual(
            Cylinder(radius=0.027, length=0.014),
            origin=Origin(xyz=(x, y, 0.061)),
            material=zinc,
            name=f"anchor_bolt_{i}",
        )

    platform = model.part("platform")
    platform.visual(
        Cylinder(radius=0.280, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=blue_steel,
        name="turntable_disk",
    )
    platform.visual(
        Cylinder(radius=0.105, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0875)),
        material=zinc,
        name="rotary_cap",
    )
    platform.visual(
        Box((0.300, 0.180, 0.060)),
        origin=Origin(xyz=(0.0, -0.010, 0.095)),
        material=blue_steel,
        name="guide_foot",
    )
    platform.visual(
        Box((0.035, 0.070, 0.780)),
        origin=Origin(xyz=(-0.110, -0.060, 0.500)),
        material=zinc,
        name="rail_0",
    )
    platform.visual(
        Box((0.035, 0.070, 0.780)),
        origin=Origin(xyz=(0.110, -0.060, 0.500)),
        material=zinc,
        name="rail_1",
    )
    platform.visual(
        Box((0.080, 0.055, 0.780)),
        origin=Origin(xyz=(0.0, 0.062, 0.500)),
        material=blue_steel,
        name="rear_spine",
    )
    platform.visual(
        Box((0.300, 0.150, 0.060)),
        origin=Origin(xyz=(0.0, -0.015, 0.900)),
        material=blue_steel,
        name="top_bridge",
    )
    platform.visual(
        Box((0.210, 0.018, 0.030)),
        origin=Origin(xyz=(0.0, -0.102, 0.135)),
        material=black,
        name="lower_stop",
    )
    platform.visual(
        Box((0.210, 0.018, 0.030)),
        origin=Origin(xyz=(0.0, -0.102, 0.835)),
        material=black,
        name="upper_stop",
    )

    front_plate = model.part("front_plate")
    front_plate.visual(
        Box((0.240, 0.035, 0.230)),
        origin=Origin(xyz=(0.0, -0.035, 0.0)),
        material=orange,
        name="plate_panel",
    )
    front_plate.visual(
        Box((0.050, 0.022, 0.260)),
        origin=Origin(xyz=(-0.110, -0.011, 0.0)),
        material=zinc,
        name="shoe_0",
    )
    front_plate.visual(
        Box((0.050, 0.022, 0.260)),
        origin=Origin(xyz=(0.110, -0.011, 0.0)),
        material=zinc,
        name="shoe_1",
    )
    front_plate.visual(
        Box((0.205, 0.012, 0.030)),
        origin=Origin(xyz=(0.0, -0.059, 0.085)),
        material=black,
        name="grip_bar",
    )
    for i, (x, z) in enumerate(
        ((-0.075, -0.070), (0.075, -0.070), (-0.075, 0.070), (0.075, 0.070))
    ):
        front_plate.visual(
            Cylinder(radius=0.018, length=0.010),
            origin=Origin(xyz=(x, -0.057, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=zinc,
            name=f"face_bolt_{i}",
        )

    model.articulation(
        "foot_to_platform",
        ArticulationType.REVOLUTE,
        parent=foot,
        child=platform,
        origin=Origin(xyz=(0.0, 0.0, 0.220)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-math.pi,
            upper=math.pi,
            effort=90.0,
            velocity=1.2,
        ),
    )
    model.articulation(
        "platform_to_front_plate",
        ArticulationType.PRISMATIC,
        parent=platform,
        child=front_plate,
        origin=Origin(xyz=(0.0, -0.095, 0.310)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=0.320,
            effort=120.0,
            velocity=0.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    foot = object_model.get_part("foot")
    platform = object_model.get_part("platform")
    front_plate = object_model.get_part("front_plate")
    rotary = object_model.get_articulation("foot_to_platform")
    lift = object_model.get_articulation("platform_to_front_plate")

    ctx.expect_gap(
        platform,
        foot,
        axis="z",
        max_gap=0.001,
        max_penetration=0.00001,
        positive_elem="turntable_disk",
        negative_elem="bearing_housing",
        name="turntable rests on bearing",
    )
    ctx.expect_overlap(
        platform,
        foot,
        axes="xy",
        min_overlap=0.18,
        elem_a="turntable_disk",
        elem_b="bearing_housing",
        name="platform centered over pedestal",
    )
    ctx.expect_contact(
        front_plate,
        platform,
        elem_a="shoe_0",
        elem_b="rail_0",
        name="first slide shoe bears on guide rail",
    )
    ctx.expect_contact(
        front_plate,
        platform,
        elem_a="shoe_1",
        elem_b="rail_1",
        name="second slide shoe bears on guide rail",
    )
    ctx.expect_overlap(
        front_plate,
        platform,
        axes="z",
        min_overlap=0.18,
        elem_a="shoe_0",
        elem_b="rail_0",
        name="lowered plate remains engaged on rail",
    )

    rest_pos = ctx.part_world_position(front_plate)
    with ctx.pose({lift: 0.320, rotary: math.pi / 2.0}):
        ctx.expect_overlap(
            front_plate,
            platform,
            axes="z",
            min_overlap=0.18,
            elem_a="shoe_0",
            elem_b="rail_0",
            name="raised plate remains engaged on rail",
        )
        raised_pos = ctx.part_world_position(front_plate)

    ctx.check(
        "front plate lifts vertically",
        rest_pos is not None
        and raised_pos is not None
        and raised_pos[2] > rest_pos[2] + 0.30,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


object_model = build_object_model()
