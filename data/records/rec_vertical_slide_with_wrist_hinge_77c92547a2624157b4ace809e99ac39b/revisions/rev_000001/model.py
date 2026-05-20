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
    model = ArticulatedObject(name="mast_lift_with_wrist")

    painted = model.material("warm_grey_paint", color=(0.46, 0.49, 0.50, 1.0))
    dark = model.material("dark_phosphate_steel", color=(0.05, 0.055, 0.06, 1.0))
    rail = model.material("polished_guide_steel", color=(0.78, 0.80, 0.78, 1.0))
    carriage_blue = model.material("blue_anodized_carriage", color=(0.05, 0.18, 0.36, 1.0))
    wear = model.material("black_polymer_wear_pad", color=(0.01, 0.012, 0.014, 1.0))
    yellow = model.material("safety_yellow_bracket", color=(0.95, 0.68, 0.08, 1.0))
    fastener = model.material("black_oxide_fasteners", color=(0.0, 0.0, 0.0, 1.0))

    mast = model.part("mast")
    mast.visual(
        Box((0.36, 0.28, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=dark,
        name="base_plate",
    )
    mast.visual(
        Box((0.105, 0.125, 1.32)),
        origin=Origin(xyz=(0.0, 0.0, 0.705)),
        material=painted,
        name="mast_tube",
    )
    mast.visual(
        Box((0.120, 0.145, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 1.385)),
        material=dark,
        name="top_stop",
    )
    mast.visual(
        Box((0.120, 0.145, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        material=dark,
        name="bottom_stop",
    )
    for i, y in enumerate((-0.043, 0.043)):
        mast.visual(
            Box((0.018, 0.014, 1.12)),
            origin=Origin(xyz=(0.060, y, 0.715)),
            material=rail,
            name=f"front_rail_{i}",
        )
    mast.visual(
        Box((0.006, 0.052, 1.04)),
        origin=Origin(xyz=(0.055, 0.0, 0.715)),
        material=dark,
        name="front_recess",
    )
    for i, z in enumerate((0.235, 1.195)):
        mast.visual(
            Box((0.025, 0.118, 0.018)),
            origin=Origin(xyz=(0.066, 0.0, z)),
            material=dark,
            name=f"rail_stop_{i}",
        )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.088, 0.178, 0.190)),
        origin=Origin(xyz=(0.119, 0.0, 0.0)),
        material=carriage_blue,
        name="carriage_body",
    )
    carriage.visual(
        Box((0.016, 0.156, 0.150)),
        origin=Origin(xyz=(0.170, 0.0, 0.0)),
        material=carriage_blue,
        name="front_wall",
    )
    for i, y in enumerate((-0.043, 0.043)):
        carriage.visual(
            Box((0.004, 0.024, 0.160)),
            origin=Origin(xyz=(0.073, y, 0.0)),
            material=wear,
            name=f"rear_wear_pad_{i}",
        )
    for i, y in enumerate((-0.077, 0.077)):
        carriage.visual(
            Box((0.026, 0.012, 0.170)),
            origin=Origin(xyz=(0.087, y, 0.0)),
            material=carriage_blue,
            name=f"side_guide_shoe_{i}",
        )

    # A real side-supported yoke: two cheek plates tied into a thick front wall.
    carriage.visual(
        Box((0.024, 0.128, 0.108)),
        origin=Origin(xyz=(0.168, 0.0, 0.010)),
        material=carriage_blue,
        name="yoke_bridge",
    )
    for i, y in enumerate((-0.058, 0.058)):
        carriage.visual(
            Box((0.088, 0.014, 0.106)),
            origin=Origin(xyz=(0.200, y, 0.010)),
            material=carriage_blue,
            name=f"yoke_plate_{i}",
        )

    # Socket plate cap screws and cheek screws are slightly seated in the metal.
    screw_x = 0.180
    for i, (y, z) in enumerate(((-0.050, -0.052), (0.050, -0.052), (-0.050, 0.070), (0.050, 0.070))):
        carriage.visual(
            Cylinder(radius=0.006, length=0.005),
            origin=Origin(xyz=(screw_x, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=fastener,
            name=f"front_screw_{i}",
        )
    for i, (y, sign) in enumerate(((-0.067, -1.0), (0.067, 1.0))):
        for j, (x, z) in enumerate(((0.184, -0.026), (0.216, 0.046))):
            carriage.visual(
                Cylinder(radius=0.0048, length=0.005),
                origin=Origin(xyz=(x, y, z), rpy=(sign * math.pi / 2.0, 0.0, 0.0)),
                material=fastener,
                name=f"cheek_screw_{i}_{j}",
            )

    nose = model.part("nose_bracket")
    nose.visual(
        Cylinder(radius=0.018, length=0.072),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=yellow,
        name="hinge_hub",
    )
    nose.visual(
        Cylinder(radius=0.0075, length=0.102),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="cross_pin",
    )
    nose.visual(
        Box((0.164, 0.044, 0.034)),
        origin=Origin(xyz=(0.092, 0.0, 0.0)),
        material=yellow,
        name="nose_arm",
    )
    nose.visual(
        Box((0.092, 0.010, 0.052)),
        origin=Origin(xyz=(0.074, -0.027, 0.001)),
        material=yellow,
        name="side_rib_0",
    )
    nose.visual(
        Box((0.092, 0.010, 0.052)),
        origin=Origin(xyz=(0.074, 0.027, 0.001)),
        material=yellow,
        name="side_rib_1",
    )
    nose.visual(
        Box((0.018, 0.078, 0.064)),
        origin=Origin(xyz=(0.180, 0.0, 0.0)),
        material=yellow,
        name="nose_face",
    )
    for i, (y, z) in enumerate(((-0.024, -0.020), (0.024, -0.020), (-0.024, 0.020), (0.024, 0.020))):
        nose.visual(
            Cylinder(radius=0.0045, length=0.005),
            origin=Origin(xyz=(0.191, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=fastener,
            name=f"nose_screw_{i}",
        )

    model.articulation(
        "mast_to_carriage",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.330)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.20, lower=0.0, upper=0.620),
    )
    model.articulation(
        "carriage_to_nose",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=nose,
        origin=Origin(xyz=(0.205, 0.0, 0.015)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=-0.52, upper=0.78),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    carriage = object_model.get_part("carriage")
    nose = object_model.get_part("nose_bracket")
    lift = object_model.get_articulation("mast_to_carriage")
    wrist = object_model.get_articulation("carriage_to_nose")

    ctx.expect_gap(
        carriage,
        mast,
        axis="x",
        positive_elem="rear_wear_pad_0",
        negative_elem="front_rail_0",
        min_gap=0.001,
        max_gap=0.006,
        name="lower carriage wear pad clears mast rail",
    )
    ctx.expect_gap(
        carriage,
        nose,
        axis="y",
        positive_elem="yoke_plate_1",
        negative_elem="hinge_hub",
        min_gap=0.008,
        max_gap=0.022,
        name="positive cheek clears hinge hub",
    )
    ctx.expect_gap(
        nose,
        carriage,
        axis="y",
        positive_elem="hinge_hub",
        negative_elem="yoke_plate_0",
        min_gap=0.008,
        max_gap=0.022,
        name="negative cheek clears hinge hub",
    )
    ctx.expect_gap(
        nose,
        carriage,
        axis="x",
        positive_elem="nose_arm",
        negative_elem="front_wall",
        min_gap=0.026,
        name="wrist arm starts ahead of carriage wall",
    )

    lower_pos = ctx.part_world_position(carriage)
    with ctx.pose({lift: 0.620}):
        ctx.expect_gap(
            carriage,
            mast,
            axis="x",
            positive_elem="rear_wear_pad_0",
            negative_elem="front_rail_0",
            min_gap=0.001,
            max_gap=0.006,
            name="raised carriage wear pad still clears rail",
        )
        ctx.expect_gap(
            carriage,
            mast,
            axis="z",
            positive_elem="carriage_body",
            negative_elem="bottom_stop",
            min_gap=0.65,
            name="raised carriage is above the bottom stop",
        )
        upper_pos = ctx.part_world_position(carriage)
    ctx.check(
        "lift joint moves carriage upward",
        lower_pos is not None and upper_pos is not None and upper_pos[2] > lower_pos[2] + 0.60,
        details=f"lower={lower_pos}, upper={upper_pos}",
    )

    rest_tip = ctx.part_element_world_aabb(nose, elem="nose_face")
    with ctx.pose({wrist: 0.72}):
        ctx.expect_gap(
            nose,
            carriage,
            axis="x",
            positive_elem="nose_arm",
            negative_elem="front_wall",
            min_gap=0.012,
            name="raised wrist remains ahead of carriage wall",
        )
        raised_tip = ctx.part_element_world_aabb(nose, elem="nose_face")
    with ctx.pose({wrist: -0.48}):
        ctx.expect_gap(
            nose,
            carriage,
            axis="x",
            positive_elem="nose_arm",
            negative_elem="front_wall",
            min_gap=0.012,
            name="lowered wrist remains ahead of carriage wall",
        )
    ctx.check(
        "wrist hinge lifts the nose tip",
        rest_tip is not None
        and raised_tip is not None
        and raised_tip[1][2] > rest_tip[1][2] + 0.05,
        details=f"rest_tip={rest_tip}, raised_tip={raised_tip}",
    )

    return ctx.report()


object_model = build_object_model()
