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
    model = ArticulatedObject(name="upright_lift_slide_wrist_plate")

    powder_coat = Material("warm_gray_powder_coat", color=(0.46, 0.48, 0.48, 1.0))
    dark_metal = Material("dark_burnished_steel", color=(0.10, 0.11, 0.12, 1.0))
    carriage_blue = Material("satin_blue_carriage", color=(0.17, 0.29, 0.42, 1.0))
    black_plastic = Material("black_glide_plastic", color=(0.015, 0.017, 0.018, 1.0))
    wrist_plate_material = Material("plain_matte_wrist_plate", color=(0.82, 0.82, 0.78, 1.0))
    hinge_steel = Material("brushed_hinge_steel", color=(0.62, 0.63, 0.61, 1.0))

    mast = model.part("mast")
    mast.visual(
        Box((0.34, 0.30, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_metal,
        name="floor_base",
    )
    mast.visual(
        Box((0.16, 0.055, 1.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.595)),
        material=powder_coat,
        name="upright_column",
    )
    mast.visual(
        Box((0.025, 0.030, 0.92)),
        origin=Origin(xyz=(-0.055, -0.0415, 0.60)),
        material=dark_metal,
        name="guide_rail_0",
    )
    mast.visual(
        Box((0.025, 0.030, 0.92)),
        origin=Origin(xyz=(0.055, -0.0415, 0.60)),
        material=dark_metal,
        name="guide_rail_1",
    )
    mast.visual(
        Box((0.050, 0.006, 0.86)),
        origin=Origin(xyz=(0.0, -0.0295, 0.60)),
        material=dark_metal,
        name="front_recess",
    )
    mast.visual(
        Box((0.20, 0.070, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 1.1775)),
        material=dark_metal,
        name="top_cap",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.22, 0.050, 0.18)),
        origin=Origin(),
        material=carriage_blue,
        name="slide_block",
    )
    carriage.visual(
        Box((0.18, 0.018, 0.12)),
        origin=Origin(xyz=(0.0, -0.034, 0.0)),
        material=carriage_blue,
        name="front_boss",
    )
    carriage.visual(
        Box((0.036, 0.0035, 0.045)),
        origin=Origin(xyz=(-0.055, 0.02675, 0.055)),
        material=black_plastic,
        name="upper_glide_0",
    )
    carriage.visual(
        Box((0.036, 0.0035, 0.045)),
        origin=Origin(xyz=(-0.055, 0.02675, -0.055)),
        material=black_plastic,
        name="lower_glide_0",
    )
    carriage.visual(
        Box((0.036, 0.0035, 0.045)),
        origin=Origin(xyz=(0.055, 0.02675, 0.055)),
        material=black_plastic,
        name="upper_glide_1",
    )
    carriage.visual(
        Box((0.036, 0.0035, 0.045)),
        origin=Origin(xyz=(0.055, 0.02675, -0.055)),
        material=black_plastic,
        name="lower_glide_1",
    )
    carriage.visual(
        Box((0.030, 0.095, 0.090)),
        origin=Origin(xyz=(-0.105, -0.077, 0.015)),
        material=carriage_blue,
        name="hinge_cheek_0",
    )
    carriage.visual(
        Box((0.030, 0.095, 0.090)),
        origin=Origin(xyz=(0.105, -0.077, 0.015)),
        material=carriage_blue,
        name="hinge_cheek_1",
    )
    carriage.visual(
        Cylinder(radius=0.018, length=0.040),
        origin=Origin(xyz=(-0.110, -0.125, 0.015), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_steel,
        name="hinge_barrel_0",
    )
    carriage.visual(
        Cylinder(radius=0.018, length=0.040),
        origin=Origin(xyz=(0.110, -0.125, 0.015), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_steel,
        name="hinge_barrel_1",
    )

    plate = model.part("wrist_plate")
    plate.visual(
        Cylinder(radius=0.016, length=0.180),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_steel,
        name="hinge_sleeve",
    )
    plate.visual(
        Box((0.100, 0.035, 0.040)),
        origin=Origin(xyz=(0.0, -0.018, -0.025)),
        material=hinge_steel,
        name="hinge_tab",
    )
    plate.visual(
        Box((0.24, 0.22, 0.024)),
        origin=Origin(xyz=(0.0, -0.135, -0.050)),
        material=wrist_plate_material,
        name="plain_face",
    )

    model.articulation(
        "mast_to_carriage",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0.0, -0.085, 0.40)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=0.0, upper=0.45),
    )
    model.articulation(
        "carriage_to_plate",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=plate,
        origin=Origin(xyz=(0.0, -0.125, 0.015)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.0, lower=-0.35, upper=1.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    carriage = object_model.get_part("carriage")
    plate = object_model.get_part("wrist_plate")
    lift = object_model.get_articulation("mast_to_carriage")
    hinge = object_model.get_articulation("carriage_to_plate")

    ctx.expect_gap(
        mast,
        carriage,
        axis="y",
        positive_elem="guide_rail_0",
        negative_elem="upper_glide_0",
        min_gap=0.0,
        max_gap=0.002,
        name="carriage glides close to mast rail",
    )
    ctx.expect_overlap(
        mast,
        carriage,
        axes="xz",
        elem_a="guide_rail_0",
        elem_b="upper_glide_0",
        min_overlap=0.020,
        name="glide pad remains over the rail",
    )

    rest_position = ctx.part_world_position(carriage)
    with ctx.pose({lift: 0.45}):
        raised_position = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            mast,
            carriage,
            axes="xz",
            elem_a="guide_rail_0",
            elem_b="upper_glide_0",
            min_overlap=0.020,
            name="raised carriage remains guided by rail",
        )
    ctx.check(
        "prismatic joint raises carriage",
        rest_position is not None
        and raised_position is not None
        and raised_position[2] > rest_position[2] + 0.40,
        details=f"rest={rest_position}, raised={raised_position}",
    )

    ctx.expect_overlap(
        plate,
        carriage,
        axes="yz",
        elem_a="hinge_sleeve",
        elem_b="hinge_barrel_0",
        min_overlap=0.025,
        name="plate hinge is coaxial with supported barrel",
    )
    ctx.expect_contact(
        carriage,
        plate,
        elem_a="hinge_barrel_1",
        elem_b="hinge_sleeve",
        contact_tol=0.001,
        name="hinge barrel supports plate sleeve",
    )

    rest_plate = ctx.part_element_world_aabb(plate, elem="plain_face")
    with ctx.pose({hinge: 0.80}):
        tilted_plate = ctx.part_element_world_aabb(plate, elem="plain_face")
    ctx.check(
        "revolute hinge tips wrist plate upward",
        rest_plate is not None
        and tilted_plate is not None
        and tilted_plate[1][2] > rest_plate[1][2] + 0.05,
        details=f"rest={rest_plate}, tilted={tilted_plate}",
    )

    return ctx.report()


object_model = build_object_model()
