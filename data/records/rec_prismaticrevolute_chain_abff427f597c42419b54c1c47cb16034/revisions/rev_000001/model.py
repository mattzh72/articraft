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
    model = ArticulatedObject(name="low_profile_hinged_carriage")

    anodized = Material("dark_anodized_aluminum", rgba=(0.08, 0.09, 0.10, 1.0))
    rail_steel = Material("brushed_steel", rgba=(0.68, 0.70, 0.68, 1.0))
    carriage_blue = Material("blue_carriage_block", rgba=(0.05, 0.20, 0.48, 1.0))
    tab_orange = Material("safety_orange_tab", rgba=(0.95, 0.38, 0.08, 1.0))
    black = Material("black_fastener_finish", rgba=(0.015, 0.015, 0.014, 1.0))

    slide = model.part("slide")
    slide.visual(
        Box((0.58, 0.24, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=anodized,
        name="ground_plate",
    )
    slide.visual(
        Box((0.50, 0.035, 0.018)),
        origin=Origin(xyz=(0.0, -0.065, 0.027)),
        material=rail_steel,
        name="rail_0",
    )
    slide.visual(
        Box((0.50, 0.035, 0.018)),
        origin=Origin(xyz=(0.0, 0.065, 0.027)),
        material=rail_steel,
        name="rail_1",
    )
    for x, name in ((-0.272, "stop_0"), (0.272, "stop_1")):
        slide.visual(
            Box((0.028, 0.21, 0.040)),
            origin=Origin(xyz=(x, 0.0, 0.038)),
            material=anodized,
            name=name,
        )
    for x in (-0.21, 0.21):
        for y in (-0.095, 0.095):
            slide.visual(
                Cylinder(radius=0.009, length=0.003),
                origin=Origin(xyz=(x, y, 0.0195)),
                material=black,
                name=f"bolt_{x:+.2f}_{y:+.2f}",
            )

    block = model.part("block")
    block.visual(
        Box((0.115, 0.145, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=carriage_blue,
        name="slider_body",
    )
    block.visual(
        Box((0.100, 0.030, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=rail_steel,
        name="guide_tongue",
    )
    block.visual(
        Box((0.030, 0.016, 0.052)),
        origin=Origin(xyz=(-0.035, -0.040, 0.066)),
        material=carriage_blue,
        name="hinge_cheek_0",
    )
    block.visual(
        Box((0.030, 0.016, 0.052)),
        origin=Origin(xyz=(-0.035, 0.040, 0.066)),
        material=carriage_blue,
        name="hinge_cheek_1",
    )
    for y, name in ((-0.050, "pin_cap_0"), (0.050, "pin_cap_1")):
        block.visual(
            Cylinder(radius=0.010, length=0.004),
            origin=Origin(xyz=(-0.035, y, 0.070), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=black,
            name=name,
        )

    tab = model.part("tab")
    tab.visual(
        Cylinder(radius=0.004, length=0.096),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="hinge_pin",
    )
    tab.visual(
        Cylinder(radius=0.009, length=0.054),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="hinge_barrel",
    )
    tab.visual(
        Box((0.092, 0.046, 0.009)),
        origin=Origin(xyz=(0.049, 0.0, -0.008)),
        material=tab_orange,
        name="support_plate",
    )
    tab.visual(
        Box((0.018, 0.048, 0.014)),
        origin=Origin(xyz=(0.095, 0.0, -0.010)),
        material=black,
        name="rubber_tip",
    )

    model.articulation(
        "slide_to_block",
        ArticulationType.PRISMATIC,
        parent=slide,
        child=block,
        origin=Origin(xyz=(-0.160, 0.0, 0.036)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.240),
    )
    model.articulation(
        "block_to_tab",
        ArticulationType.REVOLUTE,
        parent=block,
        child=tab,
        origin=Origin(xyz=(-0.035, 0.0, 0.070)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.25),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    slide = object_model.get_part("slide")
    block = object_model.get_part("block")
    tab = object_model.get_part("tab")
    slide_joint = object_model.get_articulation("slide_to_block")
    tab_joint = object_model.get_articulation("block_to_tab")

    ctx.expect_gap(
        block,
        slide,
        axis="z",
        max_gap=0.001,
        max_penetration=0.000001,
        positive_elem="slider_body",
        negative_elem="rail_0",
        name="slider body rides on rail",
    )
    ctx.expect_within(
        block,
        slide,
        axes="y",
        inner_elem="guide_tongue",
        outer_elem="ground_plate",
        margin=0.0,
        name="guide tongue stays inside the broad slide width",
    )
    ctx.expect_gap(
        tab,
        block,
        axis="y",
        min_gap=0.003,
        positive_elem="hinge_barrel",
        negative_elem="hinge_cheek_0",
        name="tab barrel is clear of one hinge cheek",
    )
    ctx.expect_gap(
        block,
        tab,
        axis="y",
        min_gap=0.003,
        positive_elem="hinge_cheek_1",
        negative_elem="hinge_barrel",
        name="tab barrel is clear of the other hinge cheek",
    )
    ctx.allow_overlap(
        tab,
        block,
        elem_a="hinge_pin",
        elem_b="hinge_cheek_0",
        reason="The hinge pin is intentionally captured through the bracket cheek bore proxy.",
    )
    ctx.allow_overlap(
        tab,
        block,
        elem_a="hinge_pin",
        elem_b="hinge_cheek_1",
        reason="The hinge pin is intentionally captured through the opposite bracket cheek bore proxy.",
    )
    ctx.expect_within(
        tab,
        block,
        axes="xz",
        inner_elem="hinge_pin",
        outer_elem="hinge_cheek_0",
        margin=0.0,
        name="hinge pin is centered in one cheek bore",
    )
    ctx.expect_within(
        tab,
        block,
        axes="xz",
        inner_elem="hinge_pin",
        outer_elem="hinge_cheek_1",
        margin=0.0,
        name="hinge pin is centered in the other cheek bore",
    )
    ctx.expect_overlap(
        tab,
        block,
        axes="y",
        elem_a="hinge_pin",
        elem_b="hinge_cheek_0",
        min_overlap=0.012,
        name="hinge pin passes through one cheek",
    )
    ctx.expect_overlap(
        tab,
        block,
        axes="y",
        elem_a="hinge_pin",
        elem_b="hinge_cheek_1",
        min_overlap=0.012,
        name="hinge pin passes through the other cheek",
    )

    rest_block = ctx.part_world_position(block)
    with ctx.pose({slide_joint: 0.240}):
        ctx.expect_overlap(
            block,
            slide,
            axes="x",
            elem_a="slider_body",
            elem_b="rail_0",
            min_overlap=0.08,
            name="extended block remains supported on the rail",
        )
        extended_block = ctx.part_world_position(block)
    ctx.check(
        "prismatic joint translates the block along the slide",
        rest_block is not None
        and extended_block is not None
        and extended_block[0] > rest_block[0] + 0.20,
        details=f"rest={rest_block}, extended={extended_block}",
    )

    rest_tab_aabb = ctx.part_world_aabb(tab)
    with ctx.pose({tab_joint: 1.10}):
        raised_tab_aabb = ctx.part_world_aabb(tab)
    ctx.check(
        "hinged tab rotates upward from the block",
        rest_tab_aabb is not None
        and raised_tab_aabb is not None
        and raised_tab_aabb[1][2] > rest_tab_aabb[1][2] + 0.055,
        details=f"rest={rest_tab_aabb}, raised={raised_tab_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
