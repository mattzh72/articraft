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
    mesh_from_cadquery,
)
import cadquery as cq


def _jaw_block_shape(sign: float) -> cq.Workplane:
    """Sliding bearing block with two rail bores and a stepped finger."""
    block_x = 0.052
    block_y = 0.160
    block_z = 0.050
    rail_y = 0.055
    # The rail bores are a hair smaller than the visual rails so the hidden
    # linear-bearing sleeve reads as captured rather than as a floating block.
    bore_r = 0.0076

    body = (
        cq.Workplane("XY")
        .box(block_x, block_y, block_z)
        .edges("|X")
        .fillet(0.004)
    )

    # Two through-bores run along local X, matching the fixed side rails.
    body = (
        body.faces(">X")
        .workplane()
        .pushPoints([(rail_y, 0.0), (-rail_y, 0.0)])
        .circle(bore_r)
        .cutThruAll()
    )

    # A low wide step bolts to the carriage, then a narrower upright carries the pad.
    foot = cq.Workplane("XY").box(0.044, 0.080, 0.026).translate((sign * 0.010, 0.0, 0.038))
    upright = cq.Workplane("XY").box(0.028, 0.060, 0.082).translate((sign * 0.024, 0.0, 0.092))
    nose = cq.Workplane("XY").box(0.020, 0.060, 0.034).translate((sign * 0.040, 0.0, 0.125))

    return body.union(foot).union(upright).union(nose).edges("|Z").fillet(0.002)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_rail_twin_slide_gripper")

    dark_anodized = model.material("dark_anodized", color=(0.08, 0.09, 0.10, 1.0))
    black = model.material("black_hardcoat", color=(0.015, 0.016, 0.018, 1.0))
    polished_steel = model.material("polished_steel", color=(0.72, 0.72, 0.68, 1.0))
    blue_jaw = model.material("blue_jaw_carriage", color=(0.05, 0.20, 0.65, 1.0))
    rubber = model.material("matte_rubber", color=(0.02, 0.02, 0.018, 1.0))

    rail_z = 0.125
    rail_y = 0.055

    housing = model.part("housing")
    housing.visual(
        Box((0.120, 0.160, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=dark_anodized,
        name="main_body",
    )
    housing.visual(
        Box((0.098, 0.132, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.083)),
        material=black,
        name="top_cover",
    )
    for y, name in ((rail_y, "side_support_0"), (-rail_y, "side_support_1")):
        housing.visual(
            Box((0.052, 0.026, 0.065)),
            origin=Origin(xyz=(0.0, y, 0.102)),
            material=dark_anodized,
            name=name,
        )
    for x, name in ((-0.185, "end_plate_0"), (0.185, "end_plate_1")):
        housing.visual(
            Box((0.024, 0.174, 0.100)),
            origin=Origin(xyz=(x, 0.0, 0.105)),
            material=dark_anodized,
            name=name,
        )
    for y, name in ((rail_y, "guide_rail_0"), (-rail_y, "guide_rail_1")):
        housing.visual(
            Cylinder(radius=0.008, length=0.372),
            origin=Origin(xyz=(0.0, y, rail_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=polished_steel,
            name=name,
        )
    for x in (-0.030, 0.030):
        for y in (-0.040, 0.040):
            housing.visual(
                Cylinder(radius=0.006, length=0.004),
                origin=Origin(xyz=(x, y, 0.098)),
                material=polished_steel,
                name=f"cover_screw_{x:+.2f}_{y:+.2f}",
            )

    jaw_specs = (
        ("jaw_block_0", -0.115, 1.0, (1.0, 0.0, 0.0), "housing_to_jaw_0"),
        ("jaw_block_1", 0.115, -1.0, (-1.0, 0.0, 0.0), "housing_to_jaw_1"),
    )
    for part_name, x0, sign, axis, joint_name in jaw_specs:
        jaw = model.part(part_name)
        jaw.visual(
            mesh_from_cadquery(_jaw_block_shape(sign), f"{part_name}_body", tolerance=0.0008),
            material=blue_jaw,
            name="carriage_body",
        )
        jaw.visual(
            Box((0.012, 0.064, 0.052)),
            origin=Origin(xyz=(sign * 0.050, 0.0, 0.096)),
            material=rubber,
            name="finger_pad",
        )

        model.articulation(
            joint_name,
            ArticulationType.PRISMATIC,
            parent=housing,
            child=jaw,
            origin=Origin(xyz=(x0, 0.0, rail_z)),
            axis=axis,
            motion_limits=MotionLimits(effort=90.0, velocity=0.18, lower=0.0, upper=0.052),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    jaw_0 = object_model.get_part("jaw_block_0")
    jaw_1 = object_model.get_part("jaw_block_1")
    slide_0 = object_model.get_articulation("housing_to_jaw_0")
    slide_1 = object_model.get_articulation("housing_to_jaw_1")

    ctx.check(
        "two independent prismatic jaw slides",
        slide_0.articulation_type == ArticulationType.PRISMATIC
        and slide_1.articulation_type == ArticulationType.PRISMATIC
        and slide_0.child == "jaw_block_0"
        and slide_1.child == "jaw_block_1",
        details=f"slide_0={slide_0}, slide_1={slide_1}",
    )

    for jaw, label in ((jaw_0, "jaw 0"), (jaw_1, "jaw 1")):
        for rail_elem in ("guide_rail_0", "guide_rail_1"):
            ctx.allow_overlap(
                housing,
                jaw,
                elem_a=rail_elem,
                elem_b="carriage_body",
                reason="Each polished rail is intentionally captured inside the jaw block linear-bearing bore.",
            )
            ctx.expect_overlap(
                jaw,
                housing,
                axes="yz",
                elem_a="carriage_body",
                elem_b=rail_elem,
                min_overlap=0.012,
                name=f"{label} carriage wraps {rail_elem}",
            )
            ctx.expect_within(
                jaw,
                housing,
                axes="x",
                inner_elem="carriage_body",
                outer_elem=rail_elem,
                margin=0.004,
                name=f"{label} carriage stays within {rail_elem} length at rest",
            )

    rest_0 = ctx.part_world_position(jaw_0)
    rest_1 = ctx.part_world_position(jaw_1)
    with ctx.pose({slide_0: 0.045, slide_1: 0.045}):
        moved_0 = ctx.part_world_position(jaw_0)
        moved_1 = ctx.part_world_position(jaw_1)
        for jaw, label in ((jaw_0, "jaw 0"), (jaw_1, "jaw 1")):
            ctx.expect_within(
                jaw,
                housing,
                axes="x",
                inner_elem="carriage_body",
                outer_elem="guide_rail_0",
                margin=0.004,
                name=f"{label} carriage remains captured on rail when actuated",
            )

    ctx.check(
        "jaw slides close toward the central grip gap",
        rest_0 is not None
        and rest_1 is not None
        and moved_0 is not None
        and moved_1 is not None
        and moved_0[0] > rest_0[0] + 0.035
        and moved_1[0] < rest_1[0] - 0.035,
        details=f"rest=({rest_0}, {rest_1}), moved=({moved_0}, {moved_1})",
    )

    return ctx.report()


object_model = build_object_model()
