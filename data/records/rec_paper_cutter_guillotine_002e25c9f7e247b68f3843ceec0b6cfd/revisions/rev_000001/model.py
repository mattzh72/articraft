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
import cadquery as cq


BED_X = 0.62
BED_Y = 0.40
BED_THICKNESS = 0.028
HINGE_X = -0.28
HINGE_Y = 0.17
HINGE_Z = 0.088
ARM_YAW = math.atan2(-0.34, 0.58)
ARM_LIMIT = math.radians(80.0)


def _hinge_world(local_xyz: tuple[float, float, float]) -> tuple[float, float, float]:
    """Map coordinates in the cutter-arm hinge frame into the fixed bed frame."""
    x, y, z = local_xyz
    c = math.cos(ARM_YAW)
    s = math.sin(ARM_YAW)
    return (
        HINGE_X + c * x - s * y,
        HINGE_Y + s * x + c * y,
        HINGE_Z + z,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_guillotine_paper_cutter")

    bed_mat = Material("warm_laminate_bed", rgba=(0.82, 0.72, 0.50, 1.0))
    edge_mat = Material("dark_fence", rgba=(0.12, 0.13, 0.13, 1.0))
    steel_mat = Material("brushed_steel", rgba=(0.70, 0.72, 0.70, 1.0))
    blade_mat = Material("sharpened_blade", rgba=(0.86, 0.88, 0.86, 1.0))
    grip_mat = Material("black_plastic_grip", rgba=(0.03, 0.03, 0.035, 1.0))
    marking_mat = Material("printed_measure_marks", rgba=(0.05, 0.055, 0.05, 1.0))

    base = model.part("bed")
    base.visual(
        Box((BED_X, BED_Y, BED_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BED_THICKNESS / 2.0)),
        material=bed_mat,
        name="cutting_bed",
    )
    base.visual(
        Box((BED_X, 0.026, 0.032)),
        origin=Origin(xyz=(0.0, BED_Y / 2.0 - 0.013, BED_THICKNESS + 0.016)),
        material=edge_mat,
        name="rear_fence",
    )
    base.visual(
        Box((BED_X, 0.016, 0.020)),
        origin=Origin(xyz=(0.0, -BED_Y / 2.0 + 0.008, BED_THICKNESS + 0.010)),
        material=edge_mat,
        name="front_lip",
    )

    # A thin fixed cutting strip under the blade makes the diagonal cutting path legible.
    base.visual(
        Box((0.64, 0.018, 0.003)),
        origin=Origin(xyz=_hinge_world((0.33, 0.0, -0.0587)), rpy=(0.0, 0.0, ARM_YAW)),
        material=steel_mat,
        name="cutting_strip",
    )

    # Printed measuring guides sit slightly into the top laminate so they are mounted.
    for i, x in enumerate((-0.22, -0.12, -0.02, 0.08, 0.18)):
        base.visual(
            Box((0.003, 0.28, 0.0012)),
            origin=Origin(xyz=(x, -0.030, BED_THICKNESS + 0.0002)),
            material=marking_mat,
            name=f"measure_mark_{i}",
        )
    for i, y in enumerate((-0.14, -0.08, -0.02, 0.04, 0.10)):
        base.visual(
            Box((0.50, 0.0025, 0.0012)),
            origin=Origin(xyz=(0.02, y, BED_THICKNESS + 0.00025)),
            material=marking_mat,
            name=f"grid_line_{i}",
        )

    # Raised rear-corner hinge block: two clevis cheeks fixed to the bed and fence.
    for name, local_y in (("hinge_cheek_0", -0.040), ("hinge_cheek_1", 0.040)):
        base.visual(
            Box((0.064, 0.016, 0.072)),
            origin=Origin(xyz=_hinge_world((0.0, local_y, -0.026)), rpy=(0.0, 0.0, ARM_YAW)),
            material=steel_mat,
            name=name,
        )
    base.visual(
        Box((0.095, 0.078, 0.010)),
        origin=Origin(xyz=_hinge_world((-0.008, 0.0, -0.065)), rpy=(0.0, 0.0, ARM_YAW)),
        material=edge_mat,
        name="hinge_foot",
    )

    arm = model.part("cutter_arm")
    arm.visual(
        Cylinder(radius=0.018, length=0.054),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel_mat,
        name="hinge_barrel",
    )
    arm.visual(
        Box((0.630, 0.010, 0.052)),
        origin=Origin(xyz=(0.325, 0.0, -0.030)),
        material=blade_mat,
        name="blade_plate",
    )
    arm.visual(
        Box((0.625, 0.012, 0.006)),
        origin=Origin(xyz=(0.332, 0.0, -0.057)),
        material=steel_mat,
        name="sharp_edge",
    )
    arm.visual(
        Box((0.600, 0.038, 0.018)),
        origin=Origin(xyz=(0.335, 0.0, 0.004)),
        material=steel_mat,
        name="upper_spine",
    )
    arm.visual(
        Box((0.500, 0.030, 0.020)),
        origin=Origin(xyz=(0.360, 0.0, 0.021)),
        material=grip_mat,
        name="long_grip",
    )
    arm.visual(
        Box((0.070, 0.032, 0.020)),
        origin=Origin(xyz=(0.590, 0.0, 0.031)),
        material=grip_mat,
        name="grip_neck",
    )
    arm.visual(
        Cylinder(radius=0.025, length=0.110),
        origin=Origin(xyz=(0.635, 0.0, 0.043), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=grip_mat,
        name="end_grip",
    )

    model.articulation(
        "arm_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(HINGE_X, HINGE_Y, HINGE_Z), rpy=(0.0, 0.0, ARM_YAW)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=0.0, upper=ARM_LIMIT),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bed = object_model.get_part("bed")
    arm = object_model.get_part("cutter_arm")
    hinge = object_model.get_articulation("arm_hinge")

    limits = hinge.motion_limits
    ctx.check(
        "single rear-corner hinge has 0 to 80 degree travel",
        limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and abs(limits.upper - ARM_LIMIT) < 1e-6,
        details=f"limits={limits}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_overlap(
            arm,
            bed,
            axes="xy",
            elem_a="blade_plate",
            elem_b="cutting_bed",
            min_overlap=0.25,
            name="closed blade spans diagonally over the bed",
        )
        ctx.expect_gap(
            arm,
            bed,
            axis="z",
            positive_elem="blade_plate",
            negative_elem="cutting_strip",
            min_gap=0.0005,
            max_gap=0.006,
            name="closed blade rides just above the cutting strip",
        )

    with ctx.pose({hinge: ARM_LIMIT}):
        ctx.expect_gap(
            arm,
            bed,
            axis="z",
            positive_elem="end_grip",
            negative_elem="cutting_bed",
            min_gap=0.35,
            name="raised handle clears high above the bed",
        )

    return ctx.report()


object_model = build_object_model()
