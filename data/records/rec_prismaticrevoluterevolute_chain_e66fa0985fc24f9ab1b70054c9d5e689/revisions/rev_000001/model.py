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
    model = ArticulatedObject(name="prismatic_revolute_revolute_chain")

    dark_steel = Material("dark_blued_steel", color=(0.08, 0.09, 0.10, 1.0))
    rail_metal = Material("ground_steel", color=(0.48, 0.52, 0.54, 1.0))
    carriage_blue = Material("anodized_blue", color=(0.05, 0.20, 0.55, 1.0))
    link_orange = Material("safety_orange", color=(0.95, 0.38, 0.08, 1.0))
    pin_black = Material("black_pin", color=(0.02, 0.02, 0.018, 1.0))

    base_rail = model.part("base_rail")
    base_rail.visual(
        Box((0.45, 0.14, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=dark_steel,
        name="mounting_plate",
    )
    base_rail.visual(
        Box((0.39, 0.044, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=rail_metal,
        name="linear_rail",
    )
    base_rail.visual(
        Box((0.020, 0.070, 0.052)),
        origin=Origin(xyz=(-0.205, 0.0, 0.046)),
        material=dark_steel,
        name="rail_stop_0",
    )
    base_rail.visual(
        Box((0.020, 0.070, 0.052)),
        origin=Origin(xyz=(0.205, 0.0, 0.046)),
        material=dark_steel,
        name="rail_stop_1",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.090, 0.110, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, -0.090)),
        material=carriage_blue,
        name="slider_block",
    )
    carriage.visual(
        Box((0.035, 0.044, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        material=carriage_blue,
        name="hinge_pedestal",
    )
    carriage.visual(
        Box((0.036, 0.018, 0.105)),
        origin=Origin(xyz=(0.0, -0.034, -0.028)),
        material=carriage_blue,
        name="shoulder_yoke_0",
    )
    carriage.visual(
        Box((0.036, 0.018, 0.105)),
        origin=Origin(xyz=(0.0, 0.034, -0.028)),
        material=carriage_blue,
        name="shoulder_yoke_1",
    )

    shoulder_link = model.part("shoulder_link")
    shoulder_link.visual(
        Cylinder(radius=0.022, length=0.050),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=pin_black,
        name="shoulder_hub",
    )
    shoulder_link.visual(
        Box((0.160, 0.040, 0.030)),
        origin=Origin(xyz=(0.085, 0.0, 0.0)),
        material=link_orange,
        name="upper_bar",
    )
    shoulder_link.visual(
        Box((0.040, 0.016, 0.050)),
        origin=Origin(xyz=(0.180, -0.027, 0.0)),
        material=link_orange,
        name="elbow_fork_0",
    )
    shoulder_link.visual(
        Box((0.040, 0.016, 0.050)),
        origin=Origin(xyz=(0.180, 0.027, 0.0)),
        material=link_orange,
        name="elbow_fork_1",
    )

    forelink = model.part("forelink")
    forelink.visual(
        Cylinder(radius=0.018, length=0.038),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=pin_black,
        name="elbow_hub",
    )
    forelink.visual(
        Box((0.030, 0.030, 0.065)),
        origin=Origin(xyz=(0.0, 0.0, -0.038)),
        material=link_orange,
        name="lower_bar",
    )
    forelink.visual(
        Box((0.050, 0.038, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.079)),
        material=pin_black,
        name="tool_tab",
    )

    model.articulation(
        "rail_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base_rail,
        child=carriage,
        origin=Origin(xyz=(-0.075, 0.0, 0.160)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.30, lower=0.0, upper=0.150),
    )
    model.articulation(
        "carriage_to_shoulder",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=shoulder_link,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=2.0,
            lower=-math.pi / 2.0,
            upper=math.pi / 2.0,
        ),
    )
    model.articulation(
        "shoulder_to_forelink",
        ArticulationType.REVOLUTE,
        parent=shoulder_link,
        child=forelink,
        origin=Origin(xyz=(0.180, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=-math.radians(75.0),
            upper=math.radians(75.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_rail = object_model.get_part("base_rail")
    carriage = object_model.get_part("carriage")
    shoulder_link = object_model.get_part("shoulder_link")
    forelink = object_model.get_part("forelink")
    slide = object_model.get_articulation("rail_to_carriage")
    shoulder = object_model.get_articulation("carriage_to_shoulder")
    elbow = object_model.get_articulation("shoulder_to_forelink")

    ctx.expect_gap(
        carriage,
        base_rail,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="slider_block",
        negative_elem="linear_rail",
        name="carriage rides on rail",
    )
    ctx.expect_contact(
        carriage,
        shoulder_link,
        elem_a="shoulder_yoke_0",
        elem_b="shoulder_hub",
        contact_tol=0.001,
        name="shoulder hub is captured by yoke",
    )
    ctx.expect_contact(
        shoulder_link,
        forelink,
        elem_a="elbow_fork_0",
        elem_b="elbow_hub",
        contact_tol=0.001,
        name="forelink hub is captured by elbow fork",
    )

    ctx.check(
        "prismatic travel is 150 mm",
        slide.motion_limits is not None
        and abs(slide.motion_limits.upper - slide.motion_limits.lower - 0.150) < 1e-6,
        details=f"limits={slide.motion_limits}",
    )
    ctx.check(
        "shoulder bends about ninety degrees each way",
        shoulder.motion_limits is not None
        and abs(shoulder.motion_limits.lower + math.pi / 2.0) < 1e-6
        and abs(shoulder.motion_limits.upper - math.pi / 2.0) < 1e-6,
        details=f"limits={shoulder.motion_limits}",
    )
    ctx.check(
        "elbow bends about seventy five degrees each way",
        elbow.motion_limits is not None
        and abs(elbow.motion_limits.lower + math.radians(75.0)) < 1e-6
        and abs(elbow.motion_limits.upper - math.radians(75.0)) < 1e-6,
        details=f"limits={elbow.motion_limits}",
    )

    rest_position = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.150}):
        extended_position = ctx.part_world_position(carriage)
    ctx.check(
        "carriage translates along the rail",
        rest_position is not None
        and extended_position is not None
        and extended_position[0] > rest_position[0] + 0.145,
        details=f"rest={rest_position}, extended={extended_position}",
    )

    return ctx.report()


object_model = build_object_model()
