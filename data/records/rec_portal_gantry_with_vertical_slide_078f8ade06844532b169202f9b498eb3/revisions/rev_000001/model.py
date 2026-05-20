from __future__ import annotations

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
    model = ArticulatedObject(name="gantry_axis_hanging_z_stage")

    painted = model.material("painted_graphite", rgba=(0.08, 0.09, 0.10, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.72, 1.0))
    rail_steel = model.material("hardened_steel", rgba=(0.46, 0.48, 0.50, 1.0))
    bearing_black = model.material("black_bearing_blocks", rgba=(0.015, 0.015, 0.014, 1.0))
    blue = model.material("blue_anodized_carriage", rgba=(0.04, 0.22, 0.62, 1.0))
    yellow = model.material("yellow_safety_stops", rgba=(1.0, 0.76, 0.08, 1.0))

    bridge = model.part("bridge")
    bridge.visual(
        Box((1.75, 0.16, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 1.15)),
        material=aluminum,
        name="main_beam",
    )
    for x in (-0.82, 0.82):
        bridge.visual(
            Box((0.12, 0.18, 1.12)),
            origin=Origin(xyz=(x, 0.0, 0.56)),
            material=painted,
            name=f"upright_{0 if x < 0 else 1}",
        )
        bridge.visual(
            Box((0.34, 0.48, 0.05)),
            origin=Origin(xyz=(x, 0.0, 0.025)),
            material=painted,
            name=f"foot_{0 if x < 0 else 1}",
        )
    bridge.visual(
        Box((1.58, 0.07, 0.05)),
        origin=Origin(xyz=(0.0, 0.125, 0.075)),
        material=painted,
        name="ground_tie",
    )
    bridge.visual(
        Box((1.50, 0.024, 0.025)),
        origin=Origin(xyz=(0.0, -0.092, 1.19)),
        material=rail_steel,
        name="upper_beam_rail",
    )
    bridge.visual(
        Box((1.50, 0.024, 0.025)),
        origin=Origin(xyz=(0.0, -0.092, 1.11)),
        material=rail_steel,
        name="lower_beam_rail",
    )

    rider = model.part("rider")
    rider.visual(
        Box((0.24, 0.05, 0.26)),
        origin=Origin(xyz=(0.0, -0.04, 0.0)),
        material=painted,
        name="beam_rider_plate",
    )
    rider.visual(
        Box((0.065, 0.036, 0.035)),
        origin=Origin(xyz=(-0.075, 0.003, 0.04)),
        material=bearing_black,
        name="upper_beam_shoe_0",
    )
    rider.visual(
        Box((0.065, 0.036, 0.035)),
        origin=Origin(xyz=(-0.075, 0.003, -0.04)),
        material=bearing_black,
        name="lower_beam_shoe_0",
    )
    rider.visual(
        Box((0.065, 0.036, 0.035)),
        origin=Origin(xyz=(0.075, 0.003, 0.04)),
        material=bearing_black,
        name="upper_beam_shoe_1",
    )
    rider.visual(
        Box((0.065, 0.036, 0.035)),
        origin=Origin(xyz=(0.075, 0.003, -0.04)),
        material=bearing_black,
        name="lower_beam_shoe_1",
    )
    rider.visual(
        Box((0.20, 0.08, 0.54)),
        origin=Origin(xyz=(0.0, -0.085, -0.24)),
        material=painted,
        name="z_guide_backbone",
    )
    rider.visual(
        Box((0.025, 0.018, 0.66)),
        origin=Origin(xyz=(-0.07, -0.134, -0.24)),
        material=rail_steel,
        name="z_rail_0",
    )
    rider.visual(
        Box((0.025, 0.018, 0.66)),
        origin=Origin(xyz=(0.07, -0.134, -0.24)),
        material=rail_steel,
        name="z_rail_1",
    )
    for z, name in ((0.105, "upper_z_stop"), (-0.585, "lower_z_stop")):
        rider.visual(
            Box((0.19, 0.022, 0.035)),
            origin=Origin(xyz=(0.0, -0.145, z)),
            material=yellow,
            name=name,
        )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.15, 0.04, 0.24)),
        origin=Origin(xyz=(0.0, -0.025, -0.15)),
        material=blue,
        name="sliding_carriage_plate",
    )
    carriage.visual(
        Box((0.038, 0.036, 0.055)),
        origin=Origin(xyz=(-0.07, 0.009, -0.09)),
        material=bearing_black,
        name="upper_z_pad_0",
    )
    carriage.visual(
        Box((0.038, 0.036, 0.055)),
        origin=Origin(xyz=(-0.07, 0.009, -0.22)),
        material=bearing_black,
        name="lower_z_pad_0",
    )
    carriage.visual(
        Box((0.038, 0.036, 0.055)),
        origin=Origin(xyz=(0.07, 0.009, -0.09)),
        material=bearing_black,
        name="upper_z_pad_1",
    )
    carriage.visual(
        Box((0.038, 0.036, 0.055)),
        origin=Origin(xyz=(0.07, 0.009, -0.22)),
        material=bearing_black,
        name="lower_z_pad_1",
    )
    carriage.visual(
        Box((0.09, 0.07, 0.08)),
        origin=Origin(xyz=(0.0, -0.055, -0.31)),
        material=blue,
        name="tool_mount_block",
    )
    carriage.visual(
        Cylinder(radius=0.025, length=0.12),
        origin=Origin(xyz=(0.0, -0.055, -0.41)),
        material=rail_steel,
        name="hanging_tool_stub",
    )

    model.articulation(
        "bridge_to_rider",
        ArticulationType.PRISMATIC,
        parent=bridge,
        child=rider,
        origin=Origin(xyz=(-0.55, -0.125, 1.15)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=450.0, velocity=0.8, lower=0.0, upper=1.10),
    )
    model.articulation(
        "rider_to_carriage",
        ArticulationType.PRISMATIC,
        parent=rider,
        child=carriage,
        origin=Origin(xyz=(0.0, -0.170, 0.04)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.35, lower=0.0, upper=0.32),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bridge = object_model.get_part("bridge")
    rider = object_model.get_part("rider")
    carriage = object_model.get_part("carriage")
    horizontal = object_model.get_articulation("bridge_to_rider")
    vertical = object_model.get_articulation("rider_to_carriage")

    ctx.check(
        "two prismatic axes",
        horizontal.articulation_type == ArticulationType.PRISMATIC
        and vertical.articulation_type == ArticulationType.PRISMATIC,
        details=f"horizontal={horizontal.articulation_type}, vertical={vertical.articulation_type}",
    )
    ctx.check(
        "rider horizontal axis",
        tuple(horizontal.axis) == (1.0, 0.0, 0.0)
        and horizontal.motion_limits is not None
        and horizontal.motion_limits.upper >= 1.0,
        details=f"axis={horizontal.axis}, limits={horizontal.motion_limits}",
    )
    ctx.check(
        "carriage vertical axis",
        tuple(vertical.axis) == (0.0, 0.0, -1.0)
        and vertical.motion_limits is not None
        and vertical.motion_limits.upper >= 0.30,
        details=f"axis={vertical.axis}, limits={vertical.motion_limits}",
    )

    with ctx.pose({horizontal: 0.0, vertical: 0.0}):
        ctx.expect_contact(
            rider,
            bridge,
            elem_a="upper_beam_shoe_0",
            elem_b="upper_beam_rail",
            contact_tol=0.001,
            name="rider shoes sit on beam rail",
        )
        ctx.expect_contact(
            carriage,
            rider,
            elem_a="upper_z_pad_0",
            elem_b="z_rail_0",
            contact_tol=0.001,
            name="vertical carriage is nested on guide rail",
        )
        ctx.expect_within(
            carriage,
            rider,
            axes="x",
            inner_elem="sliding_carriage_plate",
            outer_elem="z_guide_backbone",
            margin=0.001,
            name="carriage centered within z guide width",
        )
        rest_rider = ctx.part_world_position(rider)
        rest_carriage = ctx.part_world_position(carriage)

    with ctx.pose({horizontal: 1.10, vertical: 0.32}):
        ctx.expect_contact(
            rider,
            bridge,
            elem_a="lower_beam_shoe_1",
            elem_b="lower_beam_rail",
            contact_tol=0.001,
            name="rider remains captured at far travel",
        )
        ctx.expect_overlap(
            carriage,
            rider,
            axes="z",
            elem_a="upper_z_pad_0",
            elem_b="z_rail_0",
            min_overlap=0.04,
            name="lowered z carriage still engages rail",
        )
        extended_rider = ctx.part_world_position(rider)
        lowered_carriage = ctx.part_world_position(carriage)

    ctx.check(
        "rider traverses along beam",
        rest_rider is not None
        and extended_rider is not None
        and extended_rider[0] > rest_rider[0] + 1.0,
        details=f"rest={rest_rider}, extended={extended_rider}",
    )
    ctx.check(
        "z carriage lowers from rider",
        rest_carriage is not None
        and lowered_carriage is not None
        and lowered_carriage[2] < rest_carriage[2] - 0.30,
        details=f"rest={rest_carriage}, lowered={lowered_carriage}",
    )

    return ctx.report()


object_model = build_object_model()
