from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="mast_mounted_fork_carriage")

    mast_steel = Material("dark_powder_coated_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    rail_steel = Material("polished_guide_steel", rgba=(0.55, 0.57, 0.58, 1.0))
    carriage_yellow = Material("industrial_yellow_paint", rgba=(0.95, 0.63, 0.08, 1.0))
    fork_steel = Material("worn_fork_steel", rgba=(0.18, 0.18, 0.17, 1.0))
    rubber = Material("black_rubber_rollers", rgba=(0.01, 0.01, 0.012, 1.0))

    mast = model.part("mast")
    mast.visual(
        Box((0.36, 0.72, 0.04)),
        origin=Origin(xyz=(-0.04, 0.0, 0.02)),
        material=mast_steel,
        name="floor_plate",
    )
    mast.visual(
        Box((0.05, 0.58, 1.22)),
        origin=Origin(xyz=(-0.045, 0.0, 0.65)),
        material=mast_steel,
        name="rear_web",
    )
    for i, y in enumerate((-0.22, 0.22)):
        mast.visual(
            Box((0.05, 0.04, 1.16)),
            origin=Origin(xyz=(0.005, y, 0.64)),
            material=rail_steel,
            name=f"guide_rail_{i}",
        )
    mast.visual(
        Box((0.08, 0.66, 0.07)),
        origin=Origin(xyz=(-0.005, 0.0, 0.095)),
        material=mast_steel,
        name="lower_crosshead",
    )
    mast.visual(
        Box((0.08, 0.66, 0.07)),
        origin=Origin(xyz=(-0.005, 0.0, 1.225)),
        material=mast_steel,
        name="top_crosshead",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.08, 0.52, 0.38)),
        origin=Origin(xyz=(0.085, 0.0, 0.0)),
        material=carriage_yellow,
        name="back_plate",
    )
    carriage.visual(
        Box((0.10, 0.58, 0.07)),
        origin=Origin(xyz=(0.125, 0.0, 0.14)),
        material=carriage_yellow,
        name="upper_bar",
    )
    carriage.visual(
        Box((0.10, 0.58, 0.07)),
        origin=Origin(xyz=(0.125, 0.0, -0.14)),
        material=carriage_yellow,
        name="lower_bar",
    )
    for i, y in enumerate((-0.22, 0.22)):
        for j, z in enumerate((-0.13, 0.13)):
            carriage.visual(
                Cylinder(radius=0.024, length=0.030),
                origin=Origin(xyz=(0.054, y, z), rpy=(pi / 2.0, 0.0, 0.0)),
                material=rubber,
                name=f"guide_roller_{i}_{j}",
            )
    for i, y in enumerate((-0.15, 0.15)):
        carriage.visual(
            Box((0.70, 0.06, 0.055)),
            origin=Origin(xyz=(0.465, y, -0.15)),
            material=fork_steel,
            name=f"fork_tine_{i}",
        )
        carriage.visual(
            Box((0.11, 0.06, 0.035)),
            origin=Origin(xyz=(0.805, y, -0.135)),
            material=fork_steel,
            name=f"fork_tip_{i}",
        )

    model.articulation(
        "mast_to_carriage",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.42)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=0.35, lower=0.0, upper=0.30),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    carriage = object_model.get_part("carriage")
    lift = object_model.get_articulation("mast_to_carriage")

    ctx.check(
        "single vertical carriage slider",
        lift.axis == (0.0, 0.0, 1.0)
        and lift.motion_limits is not None
        and lift.motion_limits.lower == 0.0
        and abs(lift.motion_limits.upper - 0.30) < 1e-9,
        details=f"axis={lift.axis}, limits={lift.motion_limits}",
    )
    ctx.check(
        "fork tines are rigid carriage features",
        all(j.child != "fork_tine_0" and j.child != "fork_tine_1" for j in object_model.articulations),
        details="The tines should not have their own joints.",
    )

    ctx.expect_gap(
        carriage,
        mast,
        axis="x",
        positive_elem="back_plate",
        negative_elem="guide_rail_0",
        min_gap=0.010,
        max_gap=0.020,
        name="carriage plate clears first guide rail",
    )
    ctx.expect_gap(
        carriage,
        mast,
        axis="x",
        positive_elem="back_plate",
        negative_elem="guide_rail_1",
        min_gap=0.010,
        max_gap=0.020,
        name="carriage plate clears second guide rail",
    )
    ctx.expect_overlap(
        carriage,
        mast,
        axes="z",
        elem_a="back_plate",
        elem_b="guide_rail_0",
        min_overlap=0.30,
        name="lowered carriage remains engaged on mast",
    )

    lowered = ctx.part_world_position(carriage)
    with ctx.pose({lift: 0.30}):
        raised = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            carriage,
            mast,
            axes="z",
            elem_a="back_plate",
            elem_b="guide_rail_0",
            min_overlap=0.30,
            name="raised carriage remains engaged on mast",
        )

    ctx.check(
        "carriage lifts about 300 mm",
        lowered is not None and raised is not None and abs((raised[2] - lowered[2]) - 0.30) < 1e-6,
        details=f"lowered={lowered}, raised={raised}",
    )

    for tine_name in ("fork_tine_0", "fork_tine_1"):
        tine_aabb = ctx.part_element_world_aabb(carriage, elem=tine_name)
        plate_aabb = ctx.part_element_world_aabb(carriage, elem="back_plate")
        ctx.check(
            f"{tine_name} projects forward from carriage",
            tine_aabb is not None
            and plate_aabb is not None
            and tine_aabb[1][0] > plate_aabb[1][0] + 0.60
            and tine_aabb[0][0] < plate_aabb[1][0],
            details=f"tine_aabb={tine_aabb}, plate_aabb={plate_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
