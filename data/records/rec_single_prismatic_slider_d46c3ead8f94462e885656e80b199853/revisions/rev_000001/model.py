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
    model = ArticulatedObject(name="compact_linear_adjustment_module")

    anodized_base = Material("dark_anodized_aluminum", rgba=(0.05, 0.055, 0.06, 1.0))
    steel = Material("brushed_hardened_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    bright_steel = Material("polished_rail_wear_strip", rgba=(0.86, 0.88, 0.84, 1.0))
    black = Material("blackened_screw_heads", rgba=(0.012, 0.012, 0.014, 1.0))
    clamp_blue = Material("blue_anodized_clamp", rgba=(0.08, 0.16, 0.30, 1.0))
    rubber = Material("matte_rubber_pad", rgba=(0.02, 0.022, 0.021, 1.0))
    tick_white = Material("etched_white_index_marks", rgba=(0.80, 0.82, 0.78, 1.0))

    base = model.part("base")

    # A compact but realistically thick benchtop base plate.
    base.visual(
        Box((0.300, 0.100, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=anodized_base,
        name="base_plate",
    )

    # Short central guide rail, with a lighter wear strip on its top face.
    base.visual(
        Box((0.238, 0.028, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material=steel,
        name="guide_rail",
    )
    base.visual(
        Box((0.230, 0.018, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
        material=bright_steel,
        name="rail_wear_strip",
    )

    # Low end stops keep the block from visually overrunning the short rail.
    for idx, x in enumerate((-0.126, 0.126)):
        base.visual(
            Box((0.018, 0.058, 0.025)),
            origin=Origin(xyz=(x, 0.0, 0.0305)),
            material=anodized_base,
            name=f"end_stop_{idx}",
        )

    # Counterbored mounting screws and fine index marks give the module a
    # precision adjustment-fixture character.
    for ix, x in enumerate((-0.125, 0.125)):
        for iy, y in enumerate((-0.036, 0.036)):
            base.visual(
                Cylinder(radius=0.0055, length=0.0012),
                origin=Origin(xyz=(x, y, 0.0186)),
                material=black,
                name=f"mount_screw_{ix}_{iy}",
            )

    for i, x in enumerate((-0.080, -0.040, 0.000, 0.040, 0.080)):
        base.visual(
            Box((0.002, 0.016 if i == 2 else 0.010, 0.0008)),
            origin=Origin(xyz=(x, 0.039, 0.0184)),
            material=tick_white,
            name=f"index_mark_{i}",
        )

    clamp = model.part("clamp_block")

    # The moving block is a saddle around the rail: two cheeks and a top bridge
    # leave real clearance around the rail instead of interpenetrating it.
    clamp.visual(
        Box((0.058, 0.062, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        material=clamp_blue,
        name="top_bridge",
    )
    clamp.visual(
        Box((0.058, 0.013, 0.017)),
        origin=Origin(xyz=(0.0, 0.0245, 0.0295)),
        material=clamp_blue,
        name="side_cheek_0",
    )
    clamp.visual(
        Box((0.058, 0.013, 0.017)),
        origin=Origin(xyz=(0.0, -0.0245, 0.0295)),
        material=clamp_blue,
        name="side_cheek_1",
    )
    clamp.visual(
        Box((0.044, 0.038, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.062)),
        material=rubber,
        name="top_pad",
    )

    for ix, x in enumerate((-0.017, 0.017)):
        for iy, y in enumerate((-0.032, 0.032)):
            clamp.visual(
                Cylinder(radius=0.004, length=0.0025),
                origin=Origin(xyz=(x, y, 0.031), rpy=(pi / 2.0, 0.0, 0.0)),
                material=black,
                name=f"clamp_screw_{ix}_{iy}",
            )

    model.articulation(
        "base_to_clamp",
        ArticulationType.PRISMATIC,
        parent=base,
        child=clamp,
        origin=Origin(xyz=(-0.070, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.18, lower=0.0, upper=0.140),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    clamp = object_model.get_part("clamp_block")
    slide = object_model.get_articulation("base_to_clamp")

    ctx.check(
        "single prismatic slide",
        len(object_model.articulations) == 1
        and slide.articulation_type == ArticulationType.PRISMATIC
        and slide.axis == (1.0, 0.0, 0.0),
        details=f"joints={len(object_model.articulations)}, type={slide.articulation_type}, axis={slide.axis}",
    )

    ctx.expect_overlap(
        clamp,
        base,
        axes="xy",
        min_overlap=0.025,
        elem_a="top_bridge",
        elem_b="guide_rail",
        name="saddle remains centered over rail at home",
    )
    ctx.expect_gap(
        clamp,
        base,
        axis="z",
        min_gap=0.002,
        max_gap=0.0045,
        positive_elem="top_bridge",
        negative_elem="guide_rail",
        name="top bridge clears rail",
    )
    ctx.expect_contact(
        clamp,
        base,
        elem_a="top_bridge",
        elem_b="rail_wear_strip",
        name="sliding bridge bears on rail wear strip",
    )
    ctx.expect_gap(
        clamp,
        base,
        axis="y",
        min_gap=0.003,
        max_gap=0.0055,
        positive_elem="side_cheek_0",
        negative_elem="guide_rail",
        name="positive cheek clears rail side",
    )
    ctx.expect_gap(
        base,
        clamp,
        axis="y",
        min_gap=0.003,
        max_gap=0.0055,
        positive_elem="guide_rail",
        negative_elem="side_cheek_1",
        name="negative cheek clears rail side",
    )
    ctx.expect_gap(
        clamp,
        base,
        axis="z",
        min_gap=0.002,
        max_gap=0.0045,
        positive_elem="side_cheek_0",
        negative_elem="base_plate",
        name="saddle clears base plate",
    )

    rest_pos = ctx.part_world_position(clamp)
    with ctx.pose({slide: 0.140}):
        ctx.expect_overlap(
            clamp,
            base,
            axes="xy",
            min_overlap=0.025,
            elem_a="top_bridge",
            elem_b="guide_rail",
            name="saddle remains centered over rail at travel limit",
        )
        extended_pos = ctx.part_world_position(clamp)

    moved_only_along_x = (
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.135
        and abs(extended_pos[1] - rest_pos[1]) < 1e-9
        and abs(extended_pos[2] - rest_pos[2]) < 1e-9
    )
    ctx.check(
        "clamp translates precisely along rail axis",
        moved_only_along_x,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
