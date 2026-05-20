from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="die_handling_lift_mast")

    safety_yellow = Material("safety_yellow", rgba=(0.95, 0.62, 0.08, 1.0))
    dark_steel = Material("dark_steel", rgba=(0.08, 0.085, 0.09, 1.0))
    warm_steel = Material("warm_steel", rgba=(0.28, 0.30, 0.31, 1.0))
    rubber_black = Material("rubber_black", rgba=(0.015, 0.015, 0.014, 1.0))
    polished_wear = Material("polished_wear", rgba=(0.68, 0.70, 0.68, 1.0))

    mast = model.part("mast")
    mast.visual(
        Box((1.38, 0.82, 0.12)),
        origin=Origin(xyz=(0.0, 0.12, 0.06)),
        material=dark_steel,
        name="base_plinth",
    )
    mast.visual(
        Box((1.22, 0.34, 0.32)),
        origin=Origin(xyz=(0.0, 0.43, 0.22)),
        material=dark_steel,
        name="rear_counterweight",
    )

    for side, x in enumerate((-0.53, 0.53)):
        mast.visual(
            Box((0.14, 0.18, 2.70)),
            origin=Origin(xyz=(x, 0.0, 1.41)),
            material=warm_steel,
            name=f"upright_{side}",
        )
        mast.visual(
            Box((0.075, 0.055, 2.42)),
            origin=Origin(xyz=(0.43 if x > 0.0 else -0.43, -0.105, 1.42)),
            material=polished_wear,
            name=f"guide_rail_{side}",
        )

    mast.visual(
        Box((1.22, 0.18, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 2.72)),
        material=warm_steel,
        name="top_header",
    )
    mast.visual(
        Box((1.16, 0.14, 0.12)),
        origin=Origin(xyz=(0.0, 0.03, 0.43)),
        material=warm_steel,
        name="lower_tie",
    )
    mast.visual(
        Box((1.10, 0.12, 0.10)),
        origin=Origin(xyz=(0.0, 0.07, 2.12)),
        material=warm_steel,
        name="upper_tie",
    )
    mast.visual(
        Box((1.04, 0.08, 0.07)),
        origin=Origin(xyz=(0.0, 0.12, 0.66)),
        material=dark_steel,
        name="guard_lower_rail",
    )
    mast.visual(
        Box((1.04, 0.08, 0.07)),
        origin=Origin(xyz=(0.0, 0.12, 2.43)),
        material=dark_steel,
        name="guard_upper_rail",
    )
    for i, x in enumerate((-0.32, -0.16, 0.0, 0.16, 0.32)):
        mast.visual(
            Box((0.035, 0.05, 1.82)),
            origin=Origin(xyz=(x, 0.13, 1.545)),
            material=dark_steel,
            name=f"guard_bar_{i}",
        )

    carriage = model.part("carriage")
    carriage.visual(
        Box((1.38, 0.085, 0.62)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=safety_yellow,
        name="faceplate",
    )
    carriage.visual(
        Box((1.55, 0.12, 0.12)),
        origin=Origin(xyz=(0.0, -0.005, 0.35)),
        material=safety_yellow,
        name="top_beam",
    )
    carriage.visual(
        Box((1.55, 0.12, 0.12)),
        origin=Origin(xyz=(0.0, -0.005, -0.35)),
        material=safety_yellow,
        name="bottom_beam",
    )
    for side, x in enumerate((-0.735, 0.735)):
        carriage.visual(
            Box((0.11, 0.14, 0.78)),
            origin=Origin(xyz=(x, -0.005, 0.0)),
            material=safety_yellow,
            name=f"side_cheek_{side}",
        )

    for i, x in enumerate((-0.46, 0.0, 0.46)):
        carriage.visual(
            Box((0.07, 0.105, 0.58)),
            origin=Origin(xyz=(x, -0.065, 0.0)),
            material=warm_steel,
            name=f"face_rib_{i}",
        )

    for side, x in enumerate((-0.42, 0.42)):
        carriage.visual(
            Box((0.28, 0.18, 0.24)),
            origin=Origin(xyz=(x, -0.08, -0.37)),
            material=safety_yellow,
            name=f"arm_socket_{side}",
        )
        carriage.visual(
            Box((0.20, 0.85, 0.12)),
            origin=Origin(xyz=(x, -0.465, -0.47)),
            material=warm_steel,
            name=f"load_arm_{side}",
        )
        carriage.visual(
            Box((0.20, 0.07, 0.09)),
            origin=Origin(xyz=(x, -0.925, -0.45)),
            material=dark_steel,
            name=f"arm_nose_{side}",
        )

    roller_rpy = (0.0, math.pi / 2.0, 0.0)
    for side, x in enumerate((-0.43, 0.43)):
        for level, z in (("lower", -0.24), ("upper", 0.24)):
            carriage.visual(
                Cylinder(radius=0.035, length=0.09),
                origin=Origin(xyz=(x, 0.0375, z), rpy=roller_rpy),
                material=rubber_black,
                name=f"{level}_roller_{side}",
            )
            carriage.visual(
                Box((0.13, 0.08, 0.10)),
                origin=Origin(xyz=(x, 0.005, z)),
                material=dark_steel,
                name=f"{level}_roller_boss_{side}",
            )

    model.articulation(
        "mast_to_carriage",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0.0, -0.205, 0.90)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18000.0, velocity=0.22, lower=0.0, upper=1.0),
        motion_properties=MotionProperties(damping=1200.0, friction=450.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    carriage = object_model.get_part("carriage")
    lift = object_model.get_articulation("mast_to_carriage")

    ctx.check(
        "single vertical prismatic carriage joint",
        lift.articulation_type == ArticulationType.PRISMATIC and lift.axis == (0.0, 0.0, 1.0),
        details=f"type={lift.articulation_type}, axis={lift.axis}",
    )

    carriage_aabb = ctx.part_world_aabb(carriage)
    if carriage_aabb is not None:
        (c_min, c_max) = carriage_aabb
        ctx.check(
            "carriage is broad and heavy",
            (c_max[0] - c_min[0]) > 1.45 and (c_max[2] - c_min[2]) > 0.80,
            details=f"carriage_aabb={carriage_aabb}",
        )

    for side in (0, 1):
        ctx.expect_contact(
            carriage,
            mast,
            elem_a=f"lower_roller_{side}",
            elem_b=f"guide_rail_{side}",
            contact_tol=0.003,
            name=f"lower roller {side} bears on mast guide",
        )
        ctx.expect_contact(
            carriage,
            mast,
            elem_a=f"upper_roller_{side}",
            elem_b=f"guide_rail_{side}",
            contact_tol=0.003,
            name=f"upper roller {side} bears on mast guide",
        )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({lift: 1.0}):
        raised_pos = ctx.part_world_position(carriage)
        for side in (0, 1):
            ctx.expect_within(
                carriage,
                mast,
                axes="z",
                inner_elem=f"upper_roller_{side}",
                outer_elem=f"guide_rail_{side}",
                margin=0.01,
                name=f"raised upper roller {side} remains in guide height",
            )
            ctx.expect_contact(
                carriage,
                mast,
                elem_a=f"upper_roller_{side}",
                elem_b=f"guide_rail_{side}",
                contact_tol=0.003,
                name=f"raised upper roller {side} still bears on guide",
            )

    ctx.check(
        "carriage travels upward along mast",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.95,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


object_model = build_object_model()
