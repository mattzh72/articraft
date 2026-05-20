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
    model = ArticulatedObject(name="shop_radial_arm_carriage")

    cast = model.material("painted_cast_iron", rgba=(0.18, 0.20, 0.21, 1.0))
    dark = model.material("dark_oxide_steel", rgba=(0.05, 0.055, 0.06, 1.0))
    steel = model.material("brushed_steel", rgba=(0.58, 0.60, 0.58, 1.0))
    red = model.material("red_stop_tag", rgba=(0.65, 0.05, 0.035, 1.0))

    # Grounded shop pedestal: a heavy floor foot, round column, and turntable
    # bearing surface.  The top of the bearing sits just below the yaw joint.
    pedestal = model.part("pedestal")
    pedestal.visual(
        Box((0.70, 0.50, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=cast,
        name="floor_foot",
    )
    pedestal.visual(
        Cylinder(radius=0.16, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=cast,
        name="lower_plinth",
    )
    pedestal.visual(
        Cylinder(radius=0.075, length=0.78),
        origin=Origin(xyz=(0.0, 0.0, 0.460)),
        material=cast,
        name="round_column",
    )
    pedestal.visual(
        Cylinder(radius=0.13, length=0.085),
        origin=Origin(xyz=(0.0, 0.0, 0.8825)),
        material=dark,
        name="upper_collar",
    )
    pedestal.visual(
        Cylinder(radius=0.155, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.9375)),
        material=steel,
        name="turntable_bearing",
    )
    for i, (x, y) in enumerate(((-0.27, -0.18), (-0.27, 0.18), (0.27, -0.18), (0.27, 0.18))):
        pedestal.visual(
            Cylinder(radius=0.018, length=0.010),
            origin=Origin(xyz=(x, y, 0.064)),
            material=dark,
            name=f"floor_bolt_{i}",
        )

    # Yawing arm assembly.  Its frame is on the vertical yaw axis at the
    # shoulder bearing; the rectangular beam projects in local +X.
    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=0.105, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=dark,
        name="yaw_boss",
    )
    arm.visual(
        Box((0.28, 0.32, 0.20)),
        origin=Origin(xyz=(0.080, 0.0, 0.135)),
        material=cast,
        name="shoulder_housing",
    )
    arm.visual(
        Box((1.24, 0.12, 0.12)),
        origin=Origin(xyz=(0.720, 0.0, 0.130)),
        material=cast,
        name="main_beam",
    )
    arm.visual(
        Box((1.18, 0.060, 0.028)),
        origin=Origin(xyz=(0.745, 0.0, 0.203)),
        material=steel,
        name="top_rail",
    )
    arm.visual(
        Box((1.16, 0.080, 0.026)),
        origin=Origin(xyz=(0.755, 0.0, 0.058)),
        material=steel,
        name="lower_rail",
    )
    arm.visual(
        Box((0.060, 0.24, 0.24)),
        origin=Origin(xyz=(1.365, 0.0, 0.130)),
        material=dark,
        name="end_plate",
    )
    arm.visual(
        Box((0.055, 0.18, 0.030)),
        origin=Origin(xyz=(1.400, 0.0, 0.008)),
        material=red,
        name="stop_label",
    )
    for i, (y, z) in enumerate(((-0.075, 0.205), (0.075, 0.205), (-0.075, 0.055), (0.075, 0.055))):
        arm.visual(
            Cylinder(radius=0.015, length=0.010),
            origin=Origin(xyz=(1.400, y, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=steel,
            name=f"end_bolt_{i}",
        )

    model.articulation(
        "pedestal_to_arm",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 0.950)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.45, lower=-0.75, upper=0.75),
    )

    # The carriage is a one-piece captured saddle around the rectangular beam:
    # top and bottom plates plus side cheeks leave a small running clearance
    # around the beam and rails, so it reads as retained but not fused.
    carriage = model.part("carriage")
    carriage.visual(
        Box((0.30, 0.24, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        material=dark,
        name="top_saddle",
    )
    carriage.visual(
        Box((0.30, 0.24, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, -0.110)),
        material=dark,
        name="bottom_saddle",
    )
    carriage.visual(
        Box((0.30, 0.035, 0.245)),
        origin=Origin(xyz=(0.0, 0.1025, 0.0)),
        material=dark,
        name="side_cheek_0",
    )
    carriage.visual(
        Box((0.30, 0.035, 0.245)),
        origin=Origin(xyz=(0.0, -0.1025, 0.0)),
        material=dark,
        name="side_cheek_1",
    )
    carriage.visual(
        Box((0.050, 0.255, 0.035)),
        origin=Origin(xyz=(-0.135, 0.0, 0.110)),
        material=cast,
        name="rear_wrap",
    )
    carriage.visual(
        Box((0.050, 0.255, 0.035)),
        origin=Origin(xyz=(0.135, 0.0, 0.110)),
        material=cast,
        name="front_wrap",
    )
    for i, x in enumerate((-0.075, 0.075)):
        carriage.visual(
            Box((0.070, 0.050, 0.0055)),
            origin=Origin(xyz=(x, 0.0, 0.08975)),
            material=steel,
            name=f"top_bearing_pad_{i}",
        )
    model.articulation(
        "arm_to_carriage",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=carriage,
        origin=Origin(xyz=(0.430, 0.0, 0.130)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.55),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    arm = object_model.get_part("arm")
    carriage = object_model.get_part("carriage")
    yaw = object_model.get_articulation("pedestal_to_arm")
    slide = object_model.get_articulation("arm_to_carriage")

    ctx.check("arm uses yaw joint", yaw.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("carriage uses beam slide", slide.articulation_type == ArticulationType.PRISMATIC)

    ctx.expect_within(
        arm,
        carriage,
        axes="yz",
        inner_elem="main_beam",
        margin=0.0,
        name="beam cross-section is captured by carriage",
    )
    ctx.expect_gap(
        carriage,
        arm,
        axis="z",
        positive_elem="top_saddle",
        negative_elem="top_rail",
        min_gap=0.002,
        max_gap=0.015,
        name="top running clearance",
    )
    ctx.expect_gap(
        arm,
        carriage,
        axis="z",
        positive_elem="lower_rail",
        negative_elem="bottom_saddle",
        min_gap=0.002,
        max_gap=0.015,
        name="bottom running clearance",
    )
    ctx.expect_gap(
        carriage,
        arm,
        axis="y",
        positive_elem="side_cheek_0",
        negative_elem="main_beam",
        min_gap=0.010,
        max_gap=0.035,
        name="positive side running clearance",
    )
    ctx.expect_gap(
        arm,
        carriage,
        axis="y",
        positive_elem="main_beam",
        negative_elem="side_cheek_1",
        min_gap=0.010,
        max_gap=0.035,
        name="negative side running clearance",
    )
    ctx.expect_overlap(
        carriage,
        arm,
        axes="x",
        elem_a="top_saddle",
        elem_b="main_beam",
        min_overlap=0.25,
        name="carriage remains on beam at rest",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.55}):
        ctx.expect_within(
            arm,
            carriage,
            axes="yz",
            inner_elem="main_beam",
            margin=0.0,
            name="extended carriage still captures beam section",
        )
        ctx.expect_overlap(
            carriage,
            arm,
            axes="x",
            elem_a="top_saddle",
            elem_b="main_beam",
            min_overlap=0.25,
            name="extended carriage remains on beam",
        )
        ctx.expect_gap(
            arm,
            carriage,
            axis="x",
            positive_elem="end_plate",
            negative_elem="front_wrap",
            min_gap=0.12,
            name="end plate remains a positive stop",
        )
        extended_pos = ctx.part_world_position(carriage)

    ctx.check(
        "slide moves carriage outward",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.50,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    with ctx.pose({yaw: 0.50}):
        yawed_pos = ctx.part_world_position(carriage)
    ctx.check(
        "yaw swings beam about pedestal",
        rest_pos is not None and yawed_pos is not None and yawed_pos[1] > rest_pos[1] + 0.15,
        details=f"rest={rest_pos}, yawed={yawed_pos}",
    )

    return ctx.report()


object_model = build_object_model()
