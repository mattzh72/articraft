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
    model = ArticulatedObject(name="low_profile_vertical_slide")

    dark_anodized = Material("dark_anodized", rgba=(0.08, 0.085, 0.09, 1.0))
    satin_steel = Material("satin_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    graphite = Material("graphite_carriage", rgba=(0.12, 0.13, 0.14, 1.0))
    blue_plate = Material("blue_hinged_face", rgba=(0.08, 0.22, 0.72, 1.0))
    black_rubber = Material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.260, 0.160, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=dark_anodized,
        name="lower_mount",
    )
    base.visual(
        Box((0.165, 0.016, 0.390)),
        origin=Origin(xyz=(0.0, 0.043, 0.220)),
        material=dark_anodized,
        name="guide_backplate",
    )
    base.visual(
        Box((0.145, 0.030, 0.030)),
        origin=Origin(xyz=(0.0, 0.028, 0.040)),
        material=dark_anodized,
        name="lower_rail_clamp",
    )
    base.visual(
        Box((0.145, 0.030, 0.030)),
        origin=Origin(xyz=(0.0, 0.028, 0.400)),
        material=dark_anodized,
        name="upper_rail_clamp",
    )
    for rail_name, x in (("guide_rail_0", -0.045), ("guide_rail_1", 0.045)):
        base.visual(
            Cylinder(radius=0.008, length=0.350),
            origin=Origin(xyz=(x, 0.018, 0.220)),
            material=satin_steel,
            name=rail_name,
        )
    for idx, (x, y) in enumerate(
        ((-0.095, -0.055), (0.095, -0.055), (-0.095, 0.055), (0.095, 0.055))
    ):
        base.visual(
            Cylinder(radius=0.009, length=0.004),
            origin=Origin(xyz=(x, y, 0.027)),
            material=satin_steel,
            name=f"mount_screw_{idx}",
        )

    block = model.part("moving_block")
    block.visual(
        Box((0.125, 0.026, 0.080)),
        origin=Origin(xyz=(0.0, -0.022, 0.0)),
        material=graphite,
        name="front_plate",
    )
    for shoe_name, x in (("bearing_shoe_0", -0.045), ("bearing_shoe_1", 0.045)):
        block.visual(
            Box((0.030, 0.020, 0.070)),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=graphite,
            name=shoe_name,
        )
    for idx, x in enumerate((-0.040, 0.040)):
        block.visual(
            Box((0.024, 0.012, 0.026)),
            origin=Origin(xyz=(x, -0.034, 0.010)),
            material=graphite,
            name=f"hinge_ear_{idx}",
        )
        block.visual(
            Cylinder(radius=0.0065, length=0.024),
            origin=Origin(xyz=(x, -0.041, 0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_steel,
            name=f"outer_knuckle_{idx}",
        )
    block.visual(
        Cylinder(radius=0.0032, length=0.110),
        origin=Origin(xyz=(0.0, -0.041, 0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="hinge_pin",
    )
    block.visual(
        Box((0.080, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, -0.034, -0.030)),
        material=graphite,
        name="lower_boss",
    )

    bracket = model.part("face_bracket")
    bracket.visual(
        Cylinder(radius=0.0060, length=0.044),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="bracket_barrel",
    )
    bracket.visual(
        Box((0.052, 0.074, 0.008)),
        origin=Origin(xyz=(0.0, -0.043, -0.003)),
        material=blue_plate,
        name="bracket_face",
    )
    bracket.visual(
        Box((0.052, 0.008, 0.018)),
        origin=Origin(xyz=(0.0, -0.077, 0.002)),
        material=blue_plate,
        name="front_lip",
    )
    for idx, x in enumerate((-0.020, 0.020)):
        bracket.visual(
            Box((0.006, 0.050, 0.010)),
            origin=Origin(xyz=(x, -0.046, 0.003)),
            material=blue_plate,
            name=f"stiffening_rib_{idx}",
        )
    bracket.visual(
        Box((0.034, 0.010, 0.003)),
        origin=Origin(xyz=(0.0, -0.041, -0.008)),
        material=black_rubber,
        name="rubber_pad",
    )

    slide = model.articulation(
        "base_to_block",
        ArticulationType.PRISMATIC,
        parent=base,
        child=block,
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.25, lower=0.0, upper=0.200),
    )
    model.articulation(
        "block_to_bracket",
        ArticulationType.REVOLUTE,
        parent=block,
        child=bracket,
        origin=Origin(xyz=(0.0, -0.041, 0.010)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.8, lower=0.0, upper=1.20),
    )

    slide.meta["purpose"] = "vertical travel for the low-profile guide block"
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    block = object_model.get_part("moving_block")
    bracket = object_model.get_part("face_bracket")
    slide = object_model.get_articulation("base_to_block")
    hinge = object_model.get_articulation("block_to_bracket")

    ctx.allow_overlap(
        block,
        bracket,
        elem_a="hinge_pin",
        elem_b="bracket_barrel",
        reason="The bracket barrel is intentionally captured around the visible hinge pin.",
    )

    ctx.check(
        "block rides on a vertical prismatic joint",
        slide.articulation_type == ArticulationType.PRISMATIC
        and slide.axis == (0.0, 0.0, 1.0)
        and slide.motion_limits is not None
        and slide.motion_limits.upper == 0.200,
        details=f"type={slide.articulation_type}, axis={slide.axis}, limits={slide.motion_limits}",
    )
    ctx.check(
        "bracket is a limited revolute hinge",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and hinge.axis == (-1.0, 0.0, 0.0)
        and hinge.motion_limits is not None
        and hinge.motion_limits.lower == 0.0
        and hinge.motion_limits.upper >= 1.0,
        details=f"type={hinge.articulation_type}, axis={hinge.axis}, limits={hinge.motion_limits}",
    )

    ctx.expect_overlap(
        block,
        bracket,
        axes="x",
        elem_a="hinge_pin",
        elem_b="bracket_barrel",
        min_overlap=0.040,
        name="hinge barrel is retained on the pin span",
    )
    ctx.expect_within(
        bracket,
        block,
        axes="x",
        inner_elem="bracket_barrel",
        outer_elem="hinge_pin",
        margin=0.0,
        name="bracket barrel lies between the pin ends",
    )
    ctx.expect_gap(
        base,
        block,
        axis="y",
        positive_elem="guide_rail_0",
        negative_elem="bearing_shoe_0",
        max_gap=0.001,
        max_penetration=0.0001,
        name="bearing shoe runs against the guide rail",
    )
    ctx.expect_gap(
        block,
        bracket,
        axis="y",
        positive_elem="front_plate",
        negative_elem="bracket_face",
        min_gap=0.006,
        max_gap=0.020,
        name="hinged face projects forward of the carriage",
    )

    rest_pos = ctx.part_world_position(block)
    with ctx.pose({slide: 0.200}):
        extended_pos = ctx.part_world_position(block)
        ctx.expect_gap(
            base,
            block,
            axis="y",
            positive_elem="guide_rail_0",
            negative_elem="bearing_shoe_0",
            max_gap=0.001,
            max_penetration=0.0001,
            name="raised block remains guided by the rail",
        )

    ctx.check(
        "moving block translates upward",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[2] > rest_pos[2] + 0.180,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    def _zmax(aabb):
        return aabb[1][2]

    closed_face = ctx.part_element_world_aabb(bracket, elem="bracket_face")
    with ctx.pose({hinge: 1.0}):
        raised_face = ctx.part_element_world_aabb(bracket, elem="bracket_face")

    ctx.check(
        "hinged face swings upward from the block",
        closed_face is not None
        and raised_face is not None
        and _zmax(raised_face) > _zmax(closed_face) + 0.040,
        details=f"closed={closed_face}, raised={raised_face}",
    )

    return ctx.report()


object_model = build_object_model()
