from __future__ import annotations

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
    model = ArticulatedObject(name="bridge_backed_radial_slide_arm")

    painted = Material("warm_grey_painted_steel", color=(0.55, 0.58, 0.58, 1.0))
    dark = Material("dark_burnished_steel", color=(0.10, 0.11, 0.12, 1.0))
    rail = Material("polished_linear_rail", color=(0.72, 0.74, 0.72, 1.0))
    truck_mat = Material("blue_anodized_truck", color=(0.08, 0.20, 0.42, 1.0))
    rubber = Material("black_rubber_stops", color=(0.02, 0.02, 0.018, 1.0))

    rear_support = model.part("rear_support")
    rear_support.visual(
        Box((0.36, 0.28, 0.030)),
        origin=Origin(xyz=(0.0, -0.020, 0.015)),
        material=painted,
        name="base_plate",
    )
    for x, name in ((-0.115, "bridge_post_0"), (0.115, "bridge_post_1")):
        rear_support.visual(
            Box((0.055, 0.052, 0.315)),
            origin=Origin(xyz=(x, -0.115, 0.182)),
            material=painted,
            name=name,
        )
    rear_support.visual(
        Box((0.285, 0.052, 0.060)),
        origin=Origin(xyz=(0.0, -0.115, 0.315)),
        material=painted,
        name="top_bridge",
    )
    rear_support.visual(
        Box((0.250, 0.045, 0.050)),
        origin=Origin(xyz=(0.0, -0.102, 0.082)),
        material=painted,
        name="lower_tie",
    )
    rear_support.visual(
        Box((0.100, 0.170, 0.050)),
        origin=Origin(xyz=(0.0, -0.058, 0.292)),
        material=painted,
        name="shoulder_bridge",
    )
    rear_support.visual(
        Cylinder(radius=0.045, length=0.285),
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        material=painted,
        name="front_pillar",
    )
    rear_support.visual(
        Cylinder(radius=0.078, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.305)),
        material=dark,
        name="shoulder_bearing",
    )

    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=0.062, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=dark,
        name="shoulder_hub",
    )
    arm.visual(
        Cylinder(radius=0.041, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        material=rail,
        name="hub_cap",
    )
    arm.visual(
        Box((0.780, 0.055, 0.035)),
        origin=Origin(xyz=(0.430, 0.0, 0.020)),
        material=painted,
        name="arm_beam",
    )
    arm.visual(
        Box((0.650, 0.012, 0.012)),
        origin=Origin(xyz=(0.475, -0.029, 0.0435)),
        material=rail,
        name="upper_rail_0",
    )
    arm.visual(
        Box((0.650, 0.012, 0.012)),
        origin=Origin(xyz=(0.475, 0.029, 0.0435)),
        material=rail,
        name="upper_rail_1",
    )
    arm.visual(
        Box((0.620, 0.018, 0.004)),
        origin=Origin(xyz=(0.490, 0.0, 0.039)),
        material=dark,
        name="center_slot",
    )
    arm.visual(
        Box((0.030, 0.090, 0.055)),
        origin=Origin(xyz=(0.835, 0.0, 0.026)),
        material=rubber,
        name="tip_stop",
    )

    truck = model.part("truck")
    truck.visual(
        Box((0.120, 0.110, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.0115)),
        material=truck_mat,
        name="top_plate",
    )
    for y, name in ((-0.046, "side_cheek_0"), (0.046, "side_cheek_1")):
        truck.visual(
            Box((0.120, 0.016, 0.070)),
            origin=Origin(xyz=(0.0, y, -0.020)),
            material=truck_mat,
            name=name,
        )
    truck.visual(
        Box((0.105, 0.092, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.056)),
        material=truck_mat,
        name="lower_bridge",
    )
    truck.visual(
        Cylinder(radius=0.021, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.0335)),
        material=dark,
        name="clamp_knob",
    )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=rear_support,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 0.320)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.2, lower=-0.55, upper=1.25),
    )
    model.articulation(
        "truck_slide",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=truck,
        origin=Origin(xyz=(0.700, 0.0, 0.050)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.35, lower=-0.200, upper=0.045),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rear_support = object_model.get_part("rear_support")
    arm = object_model.get_part("arm")
    truck = object_model.get_part("truck")
    shoulder = object_model.get_articulation("shoulder")
    truck_slide = object_model.get_articulation("truck_slide")

    ctx.check(
        "shoulder is revolute",
        shoulder.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={shoulder.articulation_type}",
    )
    ctx.check(
        "truck slide is prismatic",
        truck_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={truck_slide.articulation_type}",
    )
    ctx.expect_gap(
        arm,
        rear_support,
        axis="z",
        positive_elem="shoulder_hub",
        negative_elem="shoulder_bearing",
        min_gap=0.0,
        max_gap=0.001,
        name="rotary shoulder hub seats on bearing",
    )
    ctx.expect_overlap(
        arm,
        rear_support,
        axes="xy",
        elem_a="shoulder_hub",
        elem_b="shoulder_bearing",
        min_overlap=0.080,
        name="shoulder hub is centered over support bearing",
    )
    ctx.expect_gap(
        truck,
        arm,
        axis="z",
        positive_elem="top_plate",
        negative_elem="upper_rail_1",
        min_gap=0.0,
        max_gap=0.001,
        name="truck top plate rides just above the rail",
    )
    ctx.expect_within(
        truck,
        arm,
        axes="x",
        inner_elem="top_plate",
        outer_elem="arm_beam",
        margin=0.0,
        name="truck starts on the straight arm",
    )

    rest_pos = ctx.part_world_position(truck)
    with ctx.pose({truck_slide: truck_slide.motion_limits.upper}):
        ctx.expect_within(
            truck,
            arm,
            axes="x",
            inner_elem="top_plate",
            outer_elem="arm_beam",
            margin=0.0,
            name="truck remains captured near the tip",
        )
        extended_pos = ctx.part_world_position(truck)
    with ctx.pose({truck_slide: truck_slide.motion_limits.lower}):
        ctx.expect_within(
            truck,
            arm,
            axes="x",
            inner_elem="top_plate",
            outer_elem="arm_beam",
            margin=0.0,
            name="truck remains captured inward",
        )
        retracted_pos = ctx.part_world_position(truck)
    ctx.check(
        "prismatic truck moves along the arm",
        rest_pos is not None
        and extended_pos is not None
        and retracted_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.040
        and retracted_pos[0] < rest_pos[0] - 0.150,
        details=f"retracted={retracted_pos}, rest={rest_pos}, extended={extended_pos}",
    )

    arm_rest = ctx.part_world_position(arm)
    with ctx.pose({shoulder: 0.80}):
        arm_swept = ctx.part_world_position(arm)
        truck_swept = ctx.part_world_position(truck)
    ctx.check(
        "shoulder rotates the radial arm about the rear support",
        arm_rest is not None
        and arm_swept is not None
        and truck_swept is not None
        and abs(arm_swept[0] - arm_rest[0]) < 0.002
        and truck_swept[1] > 0.45,
        details=f"arm_rest={arm_rest}, arm_swept={arm_swept}, truck_swept={truck_swept}",
    )

    return ctx.report()


object_model = build_object_model()
