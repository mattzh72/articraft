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
    model = ArticulatedObject(name="side_wall_transfer_slide")

    wall_mat = Material("painted_side_plate", rgba=(0.70, 0.72, 0.74, 1.0))
    rail_mat = Material("hardened_rail", rgba=(0.13, 0.14, 0.15, 1.0))
    screw_mat = Material("blackened_screws", rgba=(0.03, 0.03, 0.035, 1.0))
    carriage_mat = Material("anodized_carriage", rgba=(0.95, 0.45, 0.12, 1.0))
    arm_mat = Material("blue_arm_links", rgba=(0.08, 0.22, 0.65, 1.0))
    joint_mat = Material("brushed_pivots", rgba=(0.55, 0.57, 0.58, 1.0))
    pad_mat = Material("black_rubber_pad", rgba=(0.01, 0.01, 0.012, 1.0))

    side_plate = model.part("side_plate")
    side_plate.visual(
        Box((1.30, 0.24, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=wall_mat,
        name="floor_foot",
    )
    side_plate.visual(
        Box((1.22, 0.060, 0.560)),
        origin=Origin(xyz=(0.0, 0.0, 0.325)),
        material=wall_mat,
        name="wall_plate",
    )
    side_plate.visual(
        Box((1.10, 0.032, 0.040)),
        origin=Origin(xyz=(0.0, 0.046, 0.425)),
        material=rail_mat,
        name="upper_rail",
    )
    side_plate.visual(
        Box((1.10, 0.032, 0.040)),
        origin=Origin(xyz=(0.0, 0.046, 0.235)),
        material=rail_mat,
        name="lower_rail",
    )
    side_plate.visual(
        Cylinder(radius=0.012, length=1.18),
        origin=Origin(xyz=(0.0, 0.072, 0.330), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=joint_mat,
        name="drive_screw",
    )
    for x, name in ((-0.575, "left_end_block"), (0.575, "right_end_block")):
        side_plate.visual(
            Box((0.075, 0.070, 0.250)),
            origin=Origin(xyz=(x, 0.060, 0.330)),
            material=rail_mat,
            name=name,
        )
    for x in (-0.46, -0.15, 0.15, 0.46):
        for z in (0.155, 0.505):
            side_plate.visual(
                Cylinder(radius=0.014, length=0.010),
                origin=Origin(xyz=(x, 0.035, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
                material=screw_mat,
                name=f"plate_bolt_{x}_{z}",
            )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.190, 0.042, 0.064)),
        origin=Origin(xyz=(0.0, -0.057, 0.095)),
        material=carriage_mat,
        name="upper_shoe",
    )
    carriage.visual(
        Box((0.190, 0.042, 0.064)),
        origin=Origin(xyz=(0.0, -0.057, -0.095)),
        material=carriage_mat,
        name="lower_shoe",
    )
    carriage.visual(
        Box((0.205, 0.056, 0.245)),
        origin=Origin(xyz=(0.0, -0.028, 0.0)),
        material=carriage_mat,
        name="carriage_face",
    )
    carriage.visual(
        Cylinder(radius=0.055, length=0.060),
        origin=Origin(xyz=(0.0, -0.030, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=joint_mat,
        name="shoulder_socket",
    )
    for z, name in ((0.070, "upper_socket_bolt"), (-0.070, "lower_socket_bolt")):
        carriage.visual(
            Cylinder(radius=0.010, length=0.012),
            origin=Origin(xyz=(0.073, 0.006, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=screw_mat,
            name=name,
        )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.049, length=0.060),
        origin=Origin(xyz=(0.0, 0.030, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=joint_mat,
        name="shoulder_boss",
    )
    upper_arm.visual(
        Box((0.405, 0.024, 0.024)),
        origin=Origin(xyz=(0.210, 0.030, 0.032)),
        material=arm_mat,
        name="upper_top_strut",
    )
    upper_arm.visual(
        Box((0.405, 0.024, 0.024)),
        origin=Origin(xyz=(0.210, 0.030, -0.032)),
        material=arm_mat,
        name="upper_bottom_strut",
    )
    upper_arm.visual(
        Box((0.300, 0.016, 0.046)),
        origin=Origin(xyz=(0.210, 0.030, 0.0)),
        material=arm_mat,
        name="upper_web",
    )
    upper_arm.visual(
        Cylinder(radius=0.047, length=0.060),
        origin=Origin(xyz=(0.420, 0.030, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=joint_mat,
        name="elbow_socket",
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.043, length=0.060),
        origin=Origin(xyz=(0.0, 0.090, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=joint_mat,
        name="elbow_boss",
    )
    forearm.visual(
        Box((0.350, 0.022, 0.024)),
        origin=Origin(xyz=(0.180, 0.090, 0.028)),
        material=arm_mat,
        name="forearm_top_strut",
    )
    forearm.visual(
        Box((0.350, 0.022, 0.024)),
        origin=Origin(xyz=(0.180, 0.090, -0.028)),
        material=arm_mat,
        name="forearm_bottom_strut",
    )
    forearm.visual(
        Cylinder(radius=0.034, length=0.055),
        origin=Origin(xyz=(0.365, 0.090, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=joint_mat,
        name="wrist_boss",
    )
    forearm.visual(
        Box((0.065, 0.052, 0.045)),
        origin=Origin(xyz=(0.405, 0.090, 0.0)),
        material=pad_mat,
        name="transfer_pad",
    )

    model.articulation(
        "slide_axis",
        ArticulationType.PRISMATIC,
        parent=side_plate,
        child=carriage,
        origin=Origin(xyz=(-0.350, 0.140, 0.330)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.650),
    )
    model.articulation(
        "shoulder_pivot",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=upper_arm,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.5, lower=0.0, upper=1.10),
    )
    model.articulation(
        "elbow_pivot",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.420, 0.0, 0.0), rpy=(0.0, 0.65, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.8, lower=0.0, upper=1.40),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    side_plate = object_model.get_part("side_plate")
    carriage = object_model.get_part("carriage")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    slide = object_model.get_articulation("slide_axis")
    shoulder = object_model.get_articulation("shoulder_pivot")
    elbow = object_model.get_articulation("elbow_pivot")

    ctx.expect_gap(
        carriage,
        side_plate,
        axis="y",
        positive_elem="upper_shoe",
        negative_elem="upper_rail",
        min_gap=-0.0005,
        max_gap=0.001,
        name="upper shoe rides on upper rail face",
    )
    ctx.expect_gap(
        carriage,
        side_plate,
        axis="y",
        positive_elem="lower_shoe",
        negative_elem="lower_rail",
        min_gap=-0.0005,
        max_gap=0.001,
        name="lower shoe rides on lower rail face",
    )
    ctx.expect_overlap(
        carriage,
        side_plate,
        axes="x",
        elem_a="upper_shoe",
        elem_b="upper_rail",
        min_overlap=0.16,
        name="carriage remains supported on upper rail",
    )
    ctx.expect_overlap(
        upper_arm,
        carriage,
        axes="xz",
        elem_a="shoulder_boss",
        elem_b="shoulder_socket",
        min_overlap=0.040,
        name="shoulder boss is coaxial with carriage socket",
    )
    ctx.expect_overlap(
        forearm,
        upper_arm,
        axes="xz",
        elem_a="elbow_boss",
        elem_b="elbow_socket",
        min_overlap=0.035,
        name="elbow boss is coaxial with upper-arm socket",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.650}):
        ctx.expect_overlap(
            carriage,
            side_plate,
            axes="x",
            elem_a="upper_shoe",
            elem_b="upper_rail",
            min_overlap=0.16,
            name="extended carriage is still on the rail",
        )
        extended_pos = ctx.part_world_position(carriage)
    ctx.check(
        "prismatic slide travels along the wall",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.60,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    rest_elbow = ctx.part_world_position(forearm)
    with ctx.pose({shoulder: 0.90}):
        raised_elbow = ctx.part_world_position(forearm)
    ctx.check(
        "shoulder pivot raises the elbow joint",
        rest_elbow is not None and raised_elbow is not None and raised_elbow[2] > rest_elbow[2] + 0.20,
        details=f"rest_elbow={rest_elbow}, raised_elbow={raised_elbow}",
    )

    rest_wrist = ctx.part_element_world_aabb(forearm, elem="transfer_pad")
    with ctx.pose({elbow: 0.95}):
        flexed_wrist = ctx.part_element_world_aabb(forearm, elem="transfer_pad")
    rest_wrist_z = None if rest_wrist is None else (rest_wrist[0][2] + rest_wrist[1][2]) * 0.5
    flexed_wrist_z = None if flexed_wrist is None else (flexed_wrist[0][2] + flexed_wrist[1][2]) * 0.5
    ctx.check(
        "elbow pivot swings the transfer pad",
        rest_wrist_z is not None and flexed_wrist_z is not None and flexed_wrist_z > rest_wrist_z + 0.12,
        details=f"rest_wrist_z={rest_wrist_z}, flexed_wrist_z={flexed_wrist_z}",
    )

    return ctx.report()


object_model = build_object_model()
