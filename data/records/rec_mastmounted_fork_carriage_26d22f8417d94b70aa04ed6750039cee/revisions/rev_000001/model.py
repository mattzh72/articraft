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
import math


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_backed_lift_mast")

    painted_steel = model.material("charcoal_painted_steel", color=(0.10, 0.12, 0.12, 1.0))
    guide_steel = model.material("bright_worn_guide_steel", color=(0.58, 0.60, 0.58, 1.0))
    wall_gray = model.material("powder_coated_wall_back", color=(0.20, 0.22, 0.23, 1.0))
    fork_steel = model.material("scraped_fork_steel", color=(0.30, 0.31, 0.30, 1.0))
    rubber = model.material("black_rubber", color=(0.015, 0.015, 0.014, 1.0))

    mast = model.part("mast")
    mast.visual(
        Box((0.92, 0.08, 2.16)),
        origin=Origin(xyz=(0.0, -0.085, 1.08)),
        material=wall_gray,
        name="wall_back_plate",
    )
    mast.visual(
        Box((0.96, 0.34, 0.08)),
        origin=Origin(xyz=(0.0, 0.015, 0.04)),
        material=painted_steel,
        name="floor_foot",
    )
    mast.visual(
        Box((0.86, 0.12, 0.12)),
        origin=Origin(xyz=(0.0, 0.015, 0.20)),
        material=painted_steel,
        name="lower_crossbar",
    )
    mast.visual(
        Box((0.86, 0.12, 0.12)),
        origin=Origin(xyz=(0.0, 0.015, 2.03)),
        material=painted_steel,
        name="upper_crossbar",
    )
    for x, name in ((-0.39, "upright_0"), (0.39, "upright_1")):
        mast.visual(
            Box((0.085, 0.115, 1.92)),
            origin=Origin(xyz=(x, 0.000, 1.11)),
            material=painted_steel,
            name=name,
        )
    mast.visual(
        Box((0.055, 0.060, 1.78)),
        origin=Origin(xyz=(-0.285, 0.075, 1.11)),
        material=guide_steel,
        name="guide_0",
    )
    mast.visual(
        Box((0.082, 0.030, 0.15)),
        origin=Origin(xyz=(-0.285, 0.055, 0.305)),
        material=painted_steel,
        name="guide_0_lower_mount",
    )
    mast.visual(
        Box((0.082, 0.030, 0.15)),
        origin=Origin(xyz=(-0.285, 0.055, 1.915)),
        material=painted_steel,
        name="guide_0_upper_mount",
    )
    mast.visual(
        Box((0.055, 0.060, 1.78)),
        origin=Origin(xyz=(0.285, 0.075, 1.11)),
        material=guide_steel,
        name="guide_1",
    )
    mast.visual(
        Box((0.082, 0.030, 0.15)),
        origin=Origin(xyz=(0.285, 0.055, 0.305)),
        material=painted_steel,
        name="guide_1_lower_mount",
    )
    mast.visual(
        Box((0.082, 0.030, 0.15)),
        origin=Origin(xyz=(0.285, 0.055, 1.915)),
        material=painted_steel,
        name="guide_1_upper_mount",
    )
    mast.visual(
        Box((0.090, 0.030, 1.75)),
        origin=Origin(xyz=(0.0, 0.070, 1.125)),
        material=rubber,
        name="center_chain_shadow",
    )
    for x in (-0.36, 0.36):
        for z in (0.32, 1.88):
            mast.visual(
                Cylinder(radius=0.026, length=0.014),
                origin=Origin(xyz=(x, -0.036, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=painted_steel,
                name=f"wall_bolt_{x:+.2f}_{z:.2f}",
            )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.64, 0.055, 0.52)),
        origin=Origin(xyz=(0.0, 0.0, 0.26)),
        material=painted_steel,
        name="front_plate",
    )
    carriage.visual(
        Box((0.64, 0.075, 0.10)),
        origin=Origin(xyz=(0.0, 0.040, 0.065)),
        material=fork_steel,
        name="fork_heel_bar",
    )
    for x, suffix in ((-0.19, "0"), (0.19, "1")):
        carriage.visual(
            Box((0.105, 0.92, 0.055)),
            origin=Origin(xyz=(x, 0.500, 0.025)),
            material=fork_steel,
            name=f"fork_tine_{suffix}",
        )
        carriage.visual(
            Box((0.105, 0.14, 0.025)),
            origin=Origin(xyz=(x, 1.000, 0.000)),
            material=fork_steel,
            name=f"fork_nose_{suffix}",
        )
    carriage.visual(
        Box((0.060, 0.030, 0.36)),
        origin=Origin(xyz=(-0.285, -0.040, 0.30)),
        material=rubber,
        name="slide_shoe_0",
    )
    carriage.visual(
        Cylinder(radius=0.040, length=0.035),
        origin=Origin(xyz=(-0.285, -0.045, 0.18), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=guide_steel,
        name="guide_roller_0_lower",
    )
    carriage.visual(
        Cylinder(radius=0.040, length=0.035),
        origin=Origin(xyz=(-0.285, -0.045, 0.42), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=guide_steel,
        name="guide_roller_0_upper",
    )
    carriage.visual(
        Box((0.060, 0.030, 0.36)),
        origin=Origin(xyz=(0.285, -0.040, 0.30)),
        material=rubber,
        name="slide_shoe_1",
    )
    carriage.visual(
        Cylinder(radius=0.040, length=0.035),
        origin=Origin(xyz=(0.285, -0.045, 0.18), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=guide_steel,
        name="guide_roller_1_lower",
    )
    carriage.visual(
        Cylinder(radius=0.040, length=0.035),
        origin=Origin(xyz=(0.285, -0.045, 0.42), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=guide_steel,
        name="guide_roller_1_upper",
    )
    for x, name in ((-0.24, "guard_post_0"), (0.24, "guard_post_1")):
        carriage.visual(
            Box((0.055, 0.045, 0.36)),
            origin=Origin(xyz=(x, 0.040, 0.68)),
            material=painted_steel,
            name=name,
        )
    carriage.visual(
        Box((0.56, 0.045, 0.055)),
        origin=Origin(xyz=(0.0, 0.040, 0.855)),
        material=painted_steel,
        name="guard_top_rail",
    )
    carriage.visual(
        Box((0.56, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, 0.045, 0.675)),
        material=painted_steel,
        name="guard_mid_rail",
    )

    model.articulation(
        "mast_to_carriage",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.190, 0.35)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=0.35, lower=0.0, upper=1.05),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    carriage = object_model.get_part("carriage")
    lift = object_model.get_articulation("mast_to_carriage")

    ctx.check(
        "single vertical lift joint",
        lift.articulation_type == ArticulationType.PRISMATIC and tuple(lift.axis) == (0.0, 0.0, 1.0),
        details=f"type={lift.articulation_type}, axis={lift.axis}",
    )
    ctx.expect_contact(
        carriage,
        mast,
        elem_a="guide_roller_0_lower",
        elem_b="guide_0",
        contact_tol=0.002,
        name="carriage rollers bear on the fixed guide",
    )
    ctx.expect_gap(
        carriage,
        mast,
        axis="y",
        min_gap=0.045,
        max_gap=0.075,
        positive_elem="front_plate",
        negative_elem="guide_0",
        name="front plate is clear of the guide rail face",
    )
    ctx.expect_overlap(
        carriage,
        mast,
        axes="xz",
        min_overlap=0.30,
        name="carriage remains captured within mast width and height",
    )

    rest_aabb = ctx.part_world_aabb(carriage)
    with ctx.pose({lift: 1.05}):
        raised_aabb = ctx.part_world_aabb(carriage)
        ctx.expect_contact(
            carriage,
            mast,
            elem_a="guide_roller_0_lower",
            elem_b="guide_0",
            contact_tol=0.002,
            name="raised carriage still rides the guide",
        )
        ctx.expect_overlap(
            carriage,
            mast,
            axes="x",
            min_overlap=0.30,
            name="raised carriage remains between the vertical guides",
        )

    ctx.check(
        "lift travel raises carriage and forks together",
        rest_aabb is not None
        and raised_aabb is not None
        and raised_aabb[0][2] > rest_aabb[0][2] + 1.0
        and raised_aabb[1][2] > rest_aabb[1][2] + 1.0,
        details=f"rest={rest_aabb}, raised={raised_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
