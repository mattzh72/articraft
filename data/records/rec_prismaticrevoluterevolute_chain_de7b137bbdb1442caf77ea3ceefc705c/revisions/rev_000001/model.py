from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="wall_transfer_axis")

    model.material("painted_wall", rgba=(0.30, 0.32, 0.34, 1.0))
    model.material("blackened_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    model.material("brushed_rail", rgba=(0.76, 0.78, 0.79, 1.0))
    model.material("carriage_blue", rgba=(0.10, 0.24, 0.38, 1.0))
    model.material("machined_aluminum", rgba=(0.66, 0.69, 0.70, 1.0))
    model.material("dark_cap", rgba=(0.03, 0.035, 0.04, 1.0))
    model.material("tool_black", rgba=(0.01, 0.012, 0.014, 1.0))

    wall_axis = model.part("wall_axis")
    wall_axis.visual(
        Box((1.10, 0.040, 0.42)),
        origin=Origin(xyz=(0.0, -0.030, 0.0)),
        material="painted_wall",
        name="wall_plate",
    )
    wall_axis.visual(
        Box((0.96, 0.030, 0.16)),
        origin=Origin(xyz=(0.0, -0.002, 0.0)),
        material="blackened_steel",
        name="rail_backbone",
    )
    wall_axis.visual(
        Cylinder(radius=0.012, length=0.92),
        origin=Origin(xyz=(0.0, 0.022, 0.055), rpy=(0.0, pi / 2.0, 0.0)),
        material="brushed_rail",
        name="upper_rail",
    )
    wall_axis.visual(
        Cylinder(radius=0.012, length=0.92),
        origin=Origin(xyz=(0.0, 0.022, -0.055), rpy=(0.0, pi / 2.0, 0.0)),
        material="brushed_rail",
        name="lower_rail",
    )
    for x_pos, stop_name in ((-0.49, "stop_0"), (0.49, "stop_1")):
        wall_axis.visual(
            Box((0.030, 0.050, 0.20)),
            origin=Origin(xyz=(x_pos, 0.015, 0.0)),
            material="blackened_steel",
            name=stop_name,
        )
    for x_pos in (-0.42, 0.42):
        for z_pos in (-0.145, 0.145):
            wall_axis.visual(
                Cylinder(radius=0.014, length=0.006),
                origin=Origin(xyz=(x_pos, -0.013, z_pos), rpy=(-pi / 2.0, 0.0, 0.0)),
                material="dark_cap",
                name=f"wall_bolt_{x_pos}_{z_pos}",
            )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.160, 0.045, 0.180)),
        origin=Origin(xyz=(0.0, -0.070, -0.115)),
        material="carriage_blue",
        name="saddle",
    )
    carriage.visual(
        Box((0.110, 0.050, 0.074)),
        origin=Origin(xyz=(0.0, -0.036, -0.080)),
        material="carriage_blue",
        name="shoulder_riser",
    )
    carriage.visual(
        Cylinder(radius=0.055, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, -0.0175)),
        material="machined_aluminum",
        name="shoulder_turntable",
    )
    carriage.visual(
        Cylinder(radius=0.067, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.039)),
        material="dark_cap",
        name="bearing_shadow",
    )

    arm_link = model.part("arm_link")
    arm_link.visual(
        Cylinder(radius=0.045, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material="machined_aluminum",
        name="shoulder_hub",
    )
    arm_link.visual(
        Box((0.045, 0.255, 0.026)),
        origin=Origin(xyz=(0.0, 0.155, 0.016)),
        material="machined_aluminum",
        name="link_web",
    )
    arm_link.visual(
        Cylinder(radius=0.038, length=0.032),
        origin=Origin(xyz=(0.0, 0.310, 0.016)),
        material="machined_aluminum",
        name="elbow_hub",
    )
    arm_link.visual(
        Cylinder(radius=0.023, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material="dark_cap",
        name="shoulder_cap",
    )
    arm_link.visual(
        Cylinder(radius=0.020, length=0.006),
        origin=Origin(xyz=(0.0, 0.310, -0.003)),
        material="dark_cap",
        name="elbow_cap",
    )

    tip_link = model.part("tip_link")
    tip_link.visual(
        Cylinder(radius=0.035, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        material="machined_aluminum",
        name="tip_hub",
    )
    tip_link.visual(
        Box((0.035, 0.190, 0.024)),
        origin=Origin(xyz=(0.0, 0.105, 0.046)),
        material="machined_aluminum",
        name="tip_web",
    )
    tip_link.visual(
        Cylinder(radius=0.025, length=0.030),
        origin=Origin(xyz=(0.0, 0.220, 0.046)),
        material="machined_aluminum",
        name="tool_hub",
    )
    tip_link.visual(
        Box((0.060, 0.040, 0.020)),
        origin=Origin(xyz=(0.0, 0.255, 0.046)),
        material="tool_black",
        name="tool_plate",
    )
    tip_link.visual(
        Cylinder(radius=0.008, length=0.060),
        origin=Origin(xyz=(0.0, 0.295, 0.046), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="tool_black",
        name="probe_tip",
    )

    model.articulation(
        "slide",
        ArticulationType.PRISMATIC,
        parent=wall_axis,
        child=carriage,
        origin=Origin(xyz=(-0.25, 0.1265, 0.115)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=500.0, velocity=0.55, lower=0.0, upper=0.50),
    )
    model.articulation(
        "shoulder_yaw",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=arm_link,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=70.0, velocity=1.8, lower=-1.10, upper=1.10),
    )
    model.articulation(
        "tip_yaw",
        ArticulationType.REVOLUTE,
        parent=arm_link,
        child=tip_link,
        origin=Origin(xyz=(0.0, 0.310, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=2.4, lower=-1.45, upper=1.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    wall_axis = object_model.get_part("wall_axis")
    carriage = object_model.get_part("carriage")
    arm_link = object_model.get_part("arm_link")
    tip_link = object_model.get_part("tip_link")
    slide = object_model.get_articulation("slide")
    shoulder = object_model.get_articulation("shoulder_yaw")
    tip = object_model.get_articulation("tip_yaw")

    ctx.check(
        "motion stack is prismatic then two revolutes",
        slide.articulation_type == ArticulationType.PRISMATIC
        and shoulder.articulation_type == ArticulationType.REVOLUTE
        and tip.articulation_type == ArticulationType.REVOLUTE,
        details=f"types={[slide.articulation_type, shoulder.articulation_type, tip.articulation_type]}",
    )
    ctx.expect_gap(
        carriage,
        wall_axis,
        axis="y",
        max_penetration=0.0005,
        max_gap=0.006,
        positive_elem="saddle",
        negative_elem="upper_rail",
        name="saddle rides just proud of the wall rail",
    )
    ctx.expect_within(
        carriage,
        wall_axis,
        axes="x",
        inner_elem="saddle",
        outer_elem="rail_backbone",
        margin=0.0,
        name="home carriage remains within rail span",
    )
    ctx.expect_gap(
        arm_link,
        carriage,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="shoulder_hub",
        negative_elem="shoulder_turntable",
        name="shoulder hub seats on turntable",
    )
    ctx.expect_overlap(
        arm_link,
        carriage,
        axes="xy",
        min_overlap=0.070,
        elem_a="shoulder_hub",
        elem_b="shoulder_turntable",
        name="shoulder bearing footprints overlap",
    )
    ctx.expect_gap(
        tip_link,
        arm_link,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0005,
        positive_elem="tip_hub",
        negative_elem="elbow_hub",
        name="tip hub seats on elbow hub",
    )
    ctx.expect_overlap(
        tip_link,
        arm_link,
        axes="xy",
        min_overlap=0.060,
        elem_a="tip_hub",
        elem_b="elbow_hub",
        name="elbow bearing footprints overlap",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.50}):
        extended_pos = ctx.part_world_position(carriage)
        ctx.expect_within(
            carriage,
            wall_axis,
            axes="x",
            inner_elem="saddle",
            outer_elem="rail_backbone",
            margin=0.0,
            name="extended carriage remains within rail span",
        )
    ctx.check(
        "slide translates carriage along the transfer axis",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.45,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
