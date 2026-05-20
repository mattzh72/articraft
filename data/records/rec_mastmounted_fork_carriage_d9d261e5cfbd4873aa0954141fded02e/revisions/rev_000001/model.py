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
    model = ArticulatedObject(name="twin_rail_forklift_mast")

    model.material("mast_dark_steel", rgba=(0.12, 0.13, 0.14, 1.0))
    model.material("worn_guide_steel", rgba=(0.68, 0.70, 0.72, 1.0))
    model.material("carriage_yellow", rgba=(0.95, 0.62, 0.08, 1.0))
    model.material("fork_steel", rgba=(0.34, 0.35, 0.36, 1.0))
    model.material("roller_black", rgba=(0.025, 0.026, 0.028, 1.0))

    rail_x = 0.32
    rail_w = 0.09
    rail_d = 0.12
    mast_h = 2.30

    mast = model.part("mast")
    mast.visual(
        Box((0.95, 0.28, 0.06)),
        origin=Origin(xyz=(0.0, 0.11, 0.03)),
        material="mast_dark_steel",
        name="rear_foot",
    )
    upright_names = ("upright_0", "upright_1")
    guide_names = ("guide_face_0", "guide_face_1")
    for i, x in enumerate((-rail_x, rail_x)):
        mast.visual(
            Box((rail_w, rail_d, mast_h)),
            origin=Origin(xyz=(x, 0.0, mast_h / 2.0 + 0.05)),
            material="mast_dark_steel",
            name=upright_names[i],
        )
        mast.visual(
            Box((0.040, 0.014, mast_h - 0.22)),
            origin=Origin(xyz=(x, -rail_d / 2.0 - 0.005, mast_h / 2.0 + 0.08)),
            material="worn_guide_steel",
            name=guide_names[i],
        )
    mast.visual(
        Box((0.82, 0.10, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material="mast_dark_steel",
        name="lower_crosshead",
    )
    mast.visual(
        Box((0.82, 0.10, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, mast_h + 0.02)),
        material="mast_dark_steel",
        name="upper_crosshead",
    )
    mast.visual(
        Box((0.06, 0.08, mast_h - 0.12)),
        origin=Origin(xyz=(0.0, 0.04, mast_h / 2.0 + 0.05)),
        material="mast_dark_steel",
        name="center_tie",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.72, 0.060, 0.62)),
        origin=Origin(xyz=(0.0, -0.160, 0.43)),
        material="carriage_yellow",
        name="apron_plate",
    )
    carriage.visual(
        Box((0.78, 0.080, 0.090)),
        origin=Origin(xyz=(0.0, -0.150, 0.15)),
        material="carriage_yellow",
        name="lower_bar",
    )
    carriage.visual(
        Box((0.78, 0.080, 0.090)),
        origin=Origin(xyz=(0.0, -0.150, 0.70)),
        material="carriage_yellow",
        name="upper_bar",
    )
    for i, x in enumerate((-rail_x, rail_x)):
        for j, z in enumerate((0.25, 0.61)):
            carriage.visual(
                Box((0.205, 0.065, 0.115)),
                origin=Origin(xyz=(x, -0.145, z)),
                material="carriage_yellow",
                name=f"roller_boss_{i}_{j}",
            )
            carriage.visual(
                Cylinder(radius=0.025, length=0.115),
                origin=Origin(xyz=(x, -0.112, z), rpy=(0.0, pi / 2.0, 0.0)),
                material="roller_black",
                name=f"front_roller_{i}_{j}",
            )
            for k, side in enumerate((-1.0, 1.0)):
                carriage.visual(
                    Box((0.025, 0.082, 0.115)),
                    origin=Origin(
                        xyz=(x + side * (rail_w / 2.0 + 0.0125), -0.080, z)
                    ),
                    material="carriage_yellow",
                    name=f"side_shoe_{i}_{j}_{k}",
                )

    for i, x in enumerate((-0.22, 0.22)):
        carriage.visual(
            Box((0.115, 0.080, 0.42)),
            origin=Origin(xyz=(x, -0.180, 0.235)),
            material="fork_steel",
            name=f"fork_shank_{i}",
        )
        carriage.visual(
            Box((0.105, 0.78, 0.060)),
            origin=Origin(xyz=(x, -0.560, 0.055)),
            material="fork_steel",
            name=f"fork_tine_{i}",
        )
        carriage.visual(
            Box((0.105, 0.16, 0.030)),
            origin=Origin(xyz=(x, -1.010, 0.040)),
            material="fork_steel",
            name=f"fork_tip_{i}",
        )

    model.articulation(
        "mast_to_carriage",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.10, effort=9000.0, velocity=0.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    carriage = object_model.get_part("carriage")
    lift = object_model.get_articulation("mast_to_carriage")

    ctx.expect_overlap(
        carriage,
        mast,
        axes="x",
        min_overlap=0.60,
        elem_a="apron_plate",
        elem_b="lower_crosshead",
        name="carriage spans the twin rails",
    )
    ctx.expect_overlap(
        carriage,
        mast,
        axes="z",
        min_overlap=0.50,
        elem_a="apron_plate",
        elem_b="upright_0",
        name="lowered carriage remains on the mast",
    )

    lowered = ctx.part_world_position(carriage)
    with ctx.pose({lift: 1.10}):
        raised = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            carriage,
            mast,
            axes="z",
            min_overlap=0.50,
            elem_a="apron_plate",
            elem_b="upright_0",
            name="raised carriage stays guided",
        )

    ctx.check(
        "single lift joint raises the carriage",
        lowered is not None and raised is not None and raised[2] > lowered[2] + 1.0,
        details=f"lowered={lowered}, raised={raised}",
    )

    return ctx.report()


object_model = build_object_model()
