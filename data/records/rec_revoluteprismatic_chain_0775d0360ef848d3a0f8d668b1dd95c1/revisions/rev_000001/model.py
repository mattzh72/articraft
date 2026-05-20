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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_telescoping_arm")

    cast_iron = model.material("dark_cast_iron", rgba=(0.08, 0.085, 0.09, 1.0))
    painted_blue = model.material("painted_blue", rgba=(0.06, 0.22, 0.48, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    chrome = model.material("brushed_slide", rgba=(0.74, 0.75, 0.72, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.015, 0.015, 0.014, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.30, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=cast_iron,
        name="floor_flange",
    )
    pedestal.visual(
        Cylinder(radius=0.095, length=0.66),
        origin=Origin(xyz=(0.0, 0.0, 0.37)),
        material=cast_iron,
        name="column",
    )
    pedestal.visual(
        Cylinder(radius=0.18, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.73)),
        material=dark_steel,
        name="top_bearing",
    )
    pedestal.visual(
        Cylinder(radius=0.125, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.685)),
        material=dark_steel,
        name="bearing_neck",
    )

    beam = model.part("beam")
    beam.visual(
        Cylinder(radius=0.17, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=painted_blue,
        name="turntable",
    )
    beam.visual(
        Box((0.86, 0.16, 0.035)),
        origin=Origin(xyz=(0.44, 0.0, 0.115)),
        material=painted_blue,
        name="upper_rail",
    )
    beam.visual(
        Box((0.86, 0.16, 0.030)),
        origin=Origin(xyz=(0.44, 0.0, 0.025)),
        material=painted_blue,
        name="lower_rail",
    )
    beam.visual(
        Box((0.86, 0.035, 0.11)),
        origin=Origin(xyz=(0.44, 0.0875, 0.070)),
        material=painted_blue,
        name="side_rail_0",
    )
    beam.visual(
        Box((0.86, 0.035, 0.11)),
        origin=Origin(xyz=(0.44, -0.0875, 0.070)),
        material=painted_blue,
        name="side_rail_1",
    )
    beam.visual(
        Box((0.18, 0.23, 0.026)),
        origin=Origin(xyz=(0.77, 0.0, 0.112)),
        material=dark_steel,
        name="collar_upper",
    )
    beam.visual(
        Box((0.18, 0.23, 0.022)),
        origin=Origin(xyz=(0.77, 0.0, 0.028)),
        material=dark_steel,
        name="collar_lower",
    )
    beam.visual(
        Box((0.18, 0.035, 0.085)),
        origin=Origin(xyz=(0.77, 0.105, 0.070)),
        material=dark_steel,
        name="collar_side_0",
    )
    beam.visual(
        Box((0.18, 0.035, 0.085)),
        origin=Origin(xyz=(0.77, -0.105, 0.070)),
        material=dark_steel,
        name="collar_side_1",
    )
    beam.visual(
        Box((0.18, 0.030, 0.09)),
        origin=Origin(xyz=(0.16, 0.0, 0.070)),
        material=painted_blue,
        name="root_web",
    )

    nose = model.part("nose")
    nose.visual(
        Box((0.56, 0.080, 0.040)),
        origin=Origin(xyz=(-0.060, 0.0, -0.010)),
        material=chrome,
        name="slide_tongue",
    )
    nose.visual(
        Box((0.040, 0.18, 0.10)),
        origin=Origin(xyz=(0.235, 0.0, -0.010)),
        material=dark_steel,
        name="tool_pad_plate",
    )
    nose.visual(
        Box((0.008, 0.165, 0.085)),
        origin=Origin(xyz=(0.259, 0.0, -0.010)),
        material=rubber,
        name="tool_pad_face",
    )

    model.articulation(
        "pedestal_to_beam",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, 0.76)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=-2.1, upper=2.1),
    )
    model.articulation(
        "beam_to_nose",
        ArticulationType.PRISMATIC,
        parent=beam,
        child=nose,
        origin=Origin(xyz=(0.75, 0.0, 0.070)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    pedestal = object_model.get_part("pedestal")
    beam = object_model.get_part("beam")
    nose = object_model.get_part("nose")
    root_joint = object_model.get_articulation("pedestal_to_beam")
    slide_joint = object_model.get_articulation("beam_to_nose")

    ctx.check(
        "root joint is revolute",
        root_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={root_joint.articulation_type}",
    )
    ctx.check(
        "nose joint is prismatic",
        slide_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={slide_joint.articulation_type}",
    )
    ctx.check(
        "only root rotation and nose slide are articulated",
        len(object_model.articulations) == 2,
        details=f"articulations={[joint.name for joint in object_model.articulations]}",
    )

    ctx.expect_gap(
        beam,
        pedestal,
        axis="z",
        positive_elem="turntable",
        negative_elem="top_bearing",
        min_gap=0.0,
        max_gap=0.001,
        name="turntable sits on pedestal bearing",
    )
    ctx.expect_within(
        nose,
        beam,
        axes="yz",
        inner_elem="slide_tongue",
        margin=0.0,
        name="slide tongue stays inside beam carrier envelope",
    )
    ctx.expect_gap(
        beam,
        nose,
        axis="z",
        positive_elem="upper_rail",
        negative_elem="slide_tongue",
        min_gap=0.014,
        max_gap=0.020,
        name="upper rail clears slide tongue",
    )
    ctx.expect_gap(
        nose,
        beam,
        axis="z",
        positive_elem="slide_tongue",
        negative_elem="lower_rail",
        min_gap=0.0,
        max_gap=0.001,
        name="slide tongue rides on lower rail",
    )
    ctx.expect_overlap(
        nose,
        beam,
        axes="x",
        elem_a="slide_tongue",
        elem_b="upper_rail",
        min_overlap=0.40,
        name="collapsed slide remains deeply inserted",
    )

    rest_pos = ctx.part_world_position(nose)
    with ctx.pose({slide_joint: 0.25}):
        ctx.expect_within(
            nose,
            beam,
            axes="yz",
            inner_elem="slide_tongue",
            margin=0.0,
            name="extended slide remains centered in carrier envelope",
        )
        ctx.expect_overlap(
            nose,
            beam,
            axes="x",
            elem_a="slide_tongue",
            elem_b="upper_rail",
            min_overlap=0.15,
            name="extended slide retains insertion",
        )
        extended_pos = ctx.part_world_position(nose)

    ctx.check(
        "prismatic nose extends along beam axis",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.20,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
