from __future__ import annotations

import math

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
    model = ArticulatedObject(name="die_handling_lift_mast")

    safety_yellow = model.material("safety_yellow", rgba=(0.96, 0.67, 0.05, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.11, 0.12, 0.12, 1.0))
    worn_steel = model.material("worn_steel", rgba=(0.48, 0.50, 0.48, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    bolt_steel = model.material("bolt_steel", rgba=(0.22, 0.22, 0.20, 1.0))
    weld_dark = model.material("weld_dark", rgba=(0.07, 0.07, 0.065, 1.0))

    mast = model.part("mast")

    # Grounded skid base and a connected back frame.
    for x in (-0.43, 0.43):
        mast.visual(
            Box((0.18, 1.42, 0.08)),
            origin=Origin(xyz=(x, -0.23, 0.04)),
            material=dark_steel,
            name=f"base_skid_{0 if x < 0 else 1}",
        )
    for y, name in [(-0.90, "front_base_tie"), (0.42, "rear_base_tie")]:
        mast.visual(
            Box((1.08, 0.13, 0.10)),
            origin=Origin(xyz=(0.0, y, 0.05)),
            material=dark_steel,
            name=name,
        )
    mast.visual(
        Box((1.34, 0.18, 0.12)),
        origin=Origin(xyz=(0.0, 0.00, 0.10)),
        material=dark_steel,
        name="bottom_crossmember",
    )

    for x in (-0.58, 0.58):
        suffix = 0 if x < 0 else 1
        mast.visual(
            Box((0.14, 0.16, 2.72)),
            origin=Origin(xyz=(x, 0.00, 1.36)),
            material=dark_steel,
            name=f"box_upright_{suffix}",
        )
        mast.visual(
            Box((0.055, 0.055, 2.28)),
            origin=Origin(xyz=(x * 0.862, -0.095, 1.35)),
            material=worn_steel,
            name=f"guide_rail_{suffix}",
        )
        # Stop blocks sit outside the moving roller range but are physically tied
        # into each guide rail/upright.
        mast.visual(
            Box((0.13, 0.09, 0.10)),
            origin=Origin(xyz=(x * 0.862, -0.09, 0.19)),
            material=bolt_steel,
            name=f"bottom_stop_{suffix}",
        )
        mast.visual(
            Box((0.13, 0.09, 0.12)),
            origin=Origin(xyz=(x * 0.862, -0.09, 2.53)),
            material=bolt_steel,
            name=f"top_stop_{suffix}",
        )
        for z in (0.46, 1.20, 1.94, 2.34):
            mast.visual(
                Box((0.09, 0.06, 0.045)),
                origin=Origin(xyz=(x * 0.862, -0.075, z)),
                material=dark_steel,
                name=f"rail_boss_{suffix}_{int(z * 100)}",
            )
            mast.visual(
                Cylinder(radius=0.018, length=0.010),
                origin=Origin(
                    xyz=(x * 0.862, -0.123, z),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=bolt_steel,
                name=f"rail_bolt_{suffix}_{int(z * 100)}",
            )

    mast.visual(
        Box((1.28, 0.18, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 2.70)),
        material=dark_steel,
        name="top_header",
    )
    mast.visual(
        Box((1.14, 0.12, 0.09)),
        origin=Origin(xyz=(0.0, 0.035, 2.08)),
        material=dark_steel,
        name="rear_tie_bar",
    )
    for x, yaw in [(-0.50, -0.12), (0.50, 0.12)]:
        mast.visual(
            Box((0.08, 0.09, 2.40)),
            origin=Origin(xyz=(x, 0.055, 1.36), rpy=(0.0, yaw, 0.0)),
            material=dark_steel,
            name=f"rear_diagonal_{0 if x < 0 else 1}",
        )

    # Weld beads at the high-load joints are modeled as small connected strips.
    for x in (-0.58, 0.58):
        suffix = 0 if x < 0 else 1
        mast.visual(
            Box((0.18, 0.018, 0.026)),
            origin=Origin(xyz=(x, -0.089, 0.19)),
            material=weld_dark,
            name=f"lower_weld_front_{suffix}",
        )
        mast.visual(
            Box((0.18, 0.018, 0.026)),
            origin=Origin(xyz=(x, -0.089, 2.62)),
            material=weld_dark,
            name=f"upper_weld_front_{suffix}",
        )

    carriage = model.part("carriage")

    carriage.visual(
        Box((1.00, 0.085, 0.70)),
        origin=Origin(xyz=(0.0, -0.300, 0.380)),
        material=safety_yellow,
        name="faceplate",
    )
    carriage.visual(
        Box((1.08, 0.10, 0.12)),
        origin=Origin(xyz=(0.0, -0.305, 0.720)),
        material=safety_yellow,
        name="upper_carriage_channel",
    )
    carriage.visual(
        Box((1.08, 0.11, 0.12)),
        origin=Origin(xyz=(0.0, -0.305, 0.080)),
        material=safety_yellow,
        name="lower_carriage_channel",
    )
    for x in (-0.34, 0.34):
        suffix = 0 if x < 0 else 1
        carriage.visual(
            Box((0.13, 0.11, 0.78)),
            origin=Origin(xyz=(x, -0.295, 0.385)),
            material=safety_yellow,
            name=f"vertical_back_channel_{suffix}",
        )
        carriage.visual(
            Box((0.23, 0.22, 0.26)),
            origin=Origin(xyz=(x, -0.335, 0.145)),
            material=safety_yellow,
            name=f"fork_root_block_{suffix}",
        )
        carriage.visual(
            Box((0.18, 0.78, 0.085)),
            origin=Origin(xyz=(x, -0.705, 0.030)),
            material=safety_yellow,
            name=f"stub_load_arm_{suffix}",
        )
        carriage.visual(
            Box((0.032, 0.44, 0.19)),
            origin=Origin(xyz=(x - 0.075, -0.515, 0.135), rpy=(0.30, 0.0, 0.0)),
            material=safety_yellow,
            name=f"arm_gusset_inner_{suffix}",
        )
        carriage.visual(
            Box((0.032, 0.44, 0.19)),
            origin=Origin(xyz=(x + 0.075, -0.515, 0.135), rpy=(0.30, 0.0, 0.0)),
            material=safety_yellow,
            name=f"arm_gusset_outer_{suffix}",
        )
        carriage.visual(
            Box((0.13, 0.20, 0.10)),
            origin=Origin(xyz=(0.465 if x > 0 else -0.465, -0.240, 0.205)),
            material=safety_yellow,
            name=f"lower_roller_bracket_{suffix}",
        )
        carriage.visual(
            Box((0.13, 0.20, 0.10)),
            origin=Origin(xyz=(0.465 if x > 0 else -0.465, -0.240, 0.610)),
            material=safety_yellow,
            name=f"upper_roller_bracket_{suffix}",
        )
        for z, level in [(0.205, "lower"), (0.610, "upper")]:
            carriage.visual(
                Cylinder(radius=0.044, length=0.036),
                origin=Origin(
                    xyz=(0.500 if x > 0 else -0.500, -0.1665, z),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=black_rubber,
                name=f"{level}_guide_roller_{suffix}",
            )
            carriage.visual(
                Cylinder(radius=0.019, length=0.050),
                origin=Origin(
                    xyz=(0.500 if x > 0 else -0.500, -0.1665, z),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=bolt_steel,
                name=f"{level}_roller_hub_{suffix}",
            )

    # Heavy visible fasteners on the front face and fork-root doubler plates.
    for x in (-0.42, -0.20, 0.20, 0.42):
        for z in (0.18, 0.58):
            carriage.visual(
                Cylinder(radius=0.020, length=0.014),
                origin=Origin(xyz=(x, -0.347, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=bolt_steel,
                name=f"face_bolt_{int((x + 0.5) * 100)}_{int(z * 100)}",
            )
    for x in (-0.34, 0.34):
        suffix = 0 if x < 0 else 1
        carriage.visual(
            Box((0.30, 0.022, 0.30)),
            origin=Origin(xyz=(x, -0.392, 0.155)),
            material=worn_steel,
            name=f"root_doubler_{suffix}",
        )

    model.articulation(
        "mast_to_carriage",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.45)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18000.0, velocity=0.18, lower=0.0, upper=0.95),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    carriage = object_model.get_part("carriage")
    lift = object_model.get_articulation("mast_to_carriage")

    ctx.check(
        "single vertical lift joint",
        len(object_model.articulations) == 1,
        details=f"articulations={len(object_model.articulations)}",
    )

    for side in (0, 1):
        ctx.expect_gap(
            mast,
            carriage,
            axis="y",
            max_gap=0.002,
            max_penetration=0.001,
            positive_elem=f"guide_rail_{side}",
            negative_elem=f"lower_guide_roller_{side}",
            name=f"lower roller {side} runs just in front of guide rail",
        )
        ctx.expect_gap(
            mast,
            carriage,
            axis="y",
            max_gap=0.002,
            max_penetration=0.001,
            positive_elem=f"guide_rail_{side}",
            negative_elem=f"upper_guide_roller_{side}",
            name=f"upper roller {side} runs just in front of guide rail",
        )
        ctx.expect_gap(
            carriage,
            mast,
            axis="z",
            min_gap=0.10,
            positive_elem=f"lower_guide_roller_{side}",
            negative_elem=f"bottom_stop_{side}",
            name=f"lower roller {side} clears bottom stop at rest",
        )

    rest_position = ctx.part_world_position(carriage)
    with ctx.pose({lift: 0.95}):
        raised_position = ctx.part_world_position(carriage)
        for side in (0, 1):
            ctx.expect_gap(
                mast,
                carriage,
                axis="y",
                max_gap=0.002,
                max_penetration=0.001,
                positive_elem=f"guide_rail_{side}",
                negative_elem=f"upper_guide_roller_{side}",
                name=f"raised roller {side} remains clear of guide rail",
            )
            ctx.expect_gap(
                mast,
                carriage,
                axis="z",
                min_gap=0.20,
                positive_elem=f"top_stop_{side}",
                negative_elem=f"upper_guide_roller_{side}",
                name=f"raised roller {side} stops below top stop",
            )

    ctx.check(
        "carriage travels upward on the mast",
        rest_position is not None
        and raised_position is not None
        and raised_position[2] > rest_position[2] + 0.90,
        details=f"rest={rest_position}, raised={raised_position}",
    )

    return ctx.report()


object_model = build_object_model()
