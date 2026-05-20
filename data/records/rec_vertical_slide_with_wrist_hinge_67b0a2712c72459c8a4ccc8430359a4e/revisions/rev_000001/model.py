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
    model = ArticulatedObject(name="under_slung_lift_wrist")

    dark_steel = model.material("dark_steel", rgba=(0.16, 0.17, 0.18, 1.0))
    rail_wear = model.material("blackened_wear_strip", rgba=(0.03, 0.035, 0.04, 1.0))
    lift_yellow = model.material("safety_yellow", rgba=(0.95, 0.70, 0.12, 1.0))
    wrist_orange = model.material("wrist_orange", rgba=(0.95, 0.36, 0.08, 1.0))
    fastener = model.material("fastener_gunmetal", rgba=(0.08, 0.085, 0.09, 1.0))

    top_support = model.part("top_support")
    top_support.visual(
        Box((0.72, 0.28, 0.07)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=dark_steel,
        name="top_beam",
    )
    top_support.visual(
        Box((0.48, 0.20, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0825)),
        material=dark_steel,
        name="ceiling_plate",
    )
    top_support.visual(
        Box((0.035, 0.20, 0.425)),
        origin=Origin(xyz=(-0.095, 0.0, -0.2075)),
        material=dark_steel,
        name="guide_rail_0",
    )
    top_support.visual(
        Box((0.035, 0.20, 0.425)),
        origin=Origin(xyz=(0.095, 0.0, -0.2075)),
        material=dark_steel,
        name="guide_rail_1",
    )
    for index, y in enumerate((-0.085, 0.085)):
        top_support.visual(
            Box((0.25, 0.022, 0.035)),
            origin=Origin(xyz=(0.0, y, -0.405)),
            material=dark_steel,
            name=f"lower_tie_{index}",
        )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.116, 0.105, 0.31)),
        origin=Origin(xyz=(0.0, 0.0, -0.155)),
        material=lift_yellow,
        name="slider_body",
    )
    carriage.visual(
        Box((0.018, 0.13, 0.12)),
        origin=Origin(xyz=(-0.0535, 0.0, -0.12)),
        material=rail_wear,
        name="guide_pad_0",
    )
    carriage.visual(
        Box((0.018, 0.13, 0.12)),
        origin=Origin(xyz=(0.0535, 0.0, -0.12)),
        material=rail_wear,
        name="guide_pad_1",
    )
    carriage.visual(
        Box((0.12, 0.16, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, -0.326)),
        material=lift_yellow,
        name="clevis_bridge",
    )
    for index, y in enumerate((-0.0555, 0.0555)):
        carriage.visual(
            Box((0.08, 0.026, 0.12)),
            origin=Origin(xyz=(0.0, y, -0.39)),
            material=lift_yellow,
            name=f"hinge_ear_{index}",
        )

    wrist_bracket = model.part("wrist_bracket")
    wrist_bracket.visual(
        Cylinder(radius=0.026, length=0.085),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=fastener,
        name="hinge_barrel",
    )
    wrist_bracket.visual(
        Box((0.055, 0.07, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, -0.095)),
        material=wrist_orange,
        name="bracket_web",
    )
    wrist_bracket.visual(
        Box((0.16, 0.09, 0.028)),
        origin=Origin(xyz=(0.045, 0.0, -0.18)),
        material=wrist_orange,
        name="mount_plate",
    )
    wrist_bracket.visual(
        Box((0.025, 0.09, 0.06)),
        origin=Origin(xyz=(0.137, 0.0, -0.178)),
        material=wrist_orange,
        name="end_lip",
    )
    for index, y in enumerate((-0.027, 0.027)):
        wrist_bracket.visual(
            Cylinder(radius=0.011, length=0.016),
            origin=Origin(xyz=(0.06, y, -0.158)),
            material=fastener,
            name=f"bolt_boss_{index}",
        )

    model.articulation(
        "support_to_carriage",
        ArticulationType.PRISMATIC,
        parent=top_support,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.18, lower=0.0, upper=0.25),
    )
    model.articulation(
        "carriage_to_wrist",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=wrist_bracket,
        origin=Origin(xyz=(0.0, 0.0, -0.39)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=-0.75, upper=1.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    top_support = object_model.get_part("top_support")
    carriage = object_model.get_part("carriage")
    wrist_bracket = object_model.get_part("wrist_bracket")
    lift = object_model.get_articulation("support_to_carriage")
    wrist = object_model.get_articulation("carriage_to_wrist")

    ctx.check(
        "vertical carriage uses a prismatic joint",
        lift.articulation_type == ArticulationType.PRISMATIC,
        details=f"joint_type={lift.articulation_type}",
    )
    ctx.check(
        "underslung bracket uses a revolute joint",
        wrist.articulation_type == ArticulationType.REVOLUTE,
        details=f"joint_type={wrist.articulation_type}",
    )
    ctx.check(
        "lift axis is vertical downward",
        tuple(round(v, 6) for v in lift.axis) == (0.0, 0.0, -1.0),
        details=f"axis={lift.axis}",
    )
    ctx.expect_gap(
        top_support,
        carriage,
        axis="z",
        positive_elem="top_beam",
        negative_elem="slider_body",
        min_gap=0.0,
        max_gap=0.001,
        name="carriage hangs directly from underside of top support",
    )
    ctx.expect_origin_gap(
        carriage,
        wrist_bracket,
        axis="z",
        min_gap=0.34,
        max_gap=0.44,
        name="wrist bracket is below the hanging carriage",
    )

    rest_position = ctx.part_world_position(carriage)
    with ctx.pose({lift: 0.25}):
        lowered_position = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            carriage,
            top_support,
            axes="z",
            elem_a="guide_pad_0",
            elem_b="guide_rail_0",
            min_overlap=0.06,
            name="lowered carriage remains engaged in the left guide",
        )
        ctx.expect_overlap(
            carriage,
            top_support,
            axes="z",
            elem_a="guide_pad_1",
            elem_b="guide_rail_1",
            min_overlap=0.06,
            name="lowered carriage remains engaged in the right guide",
        )
    ctx.check(
        "prismatic carriage lowers along the support",
        rest_position is not None
        and lowered_position is not None
        and lowered_position[2] < rest_position[2] - 0.20,
        details=f"rest={rest_position}, lowered={lowered_position}",
    )

    def _center_x(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return 0.5 * (lower[0] + upper[0])

    rest_plate = ctx.part_element_world_aabb(wrist_bracket, elem="mount_plate")
    with ctx.pose({wrist: 0.9}):
        pitched_plate = ctx.part_element_world_aabb(wrist_bracket, elem="mount_plate")
    rest_x = _center_x(rest_plate)
    pitched_x = _center_x(pitched_plate)
    ctx.check(
        "revolute wrist pitches the bracket forward",
        rest_x is not None and pitched_x is not None and pitched_x > rest_x + 0.09,
        details=f"rest_x={rest_x}, pitched_x={pitched_x}",
    )

    return ctx.report()


object_model = build_object_model()
