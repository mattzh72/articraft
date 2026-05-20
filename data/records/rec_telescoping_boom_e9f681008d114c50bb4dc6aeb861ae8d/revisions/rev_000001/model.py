from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _hollow_rect_x(
    x_min: float,
    x_max: float,
    outer_y: float,
    outer_z: float,
    inner_y: float,
    inner_z: float,
) -> cq.Workplane:
    """Rectangular tube/ring running along local +X."""
    length = x_max - x_min
    x_mid = (x_min + x_max) * 0.5
    outer = cq.Workplane("XY").box(length, outer_y, outer_z).translate((x_mid, 0.0, 0.0))
    cutter = (
        cq.Workplane("XY")
        .box(length + 0.04, inner_y, inner_z)
        .translate((x_mid, 0.0, 0.0))
    )
    return outer.cut(cutter)


def _telescoping_tube(
    x_min: float,
    x_max: float,
    outer_y: float,
    outer_z: float,
    wall: float,
) -> cq.Workplane:
    return _hollow_rect_x(
        x_min,
        x_max,
        outer_y,
        outer_z,
        outer_y - 2.0 * wall,
        outer_z - 2.0 * wall,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_telescoping_boom")

    support_paint = model.material("support_powder_coat", color=(0.10, 0.12, 0.14, 1.0))
    blue_steel = model.material("blue_steel_sleeve", color=(0.05, 0.16, 0.30, 1.0))
    brushed_steel = model.material("brushed_inner_steel", color=(0.62, 0.66, 0.67, 1.0))
    dark_wear = model.material("black_wear_collars", color=(0.015, 0.017, 0.018, 1.0))
    zinc_bolts = model.material("zinc_bolt_heads", color=(0.55, 0.58, 0.56, 1.0))

    side_support = model.part("side_support")
    side_support.visual(
        Box((0.08, 0.72, 1.65)),
        origin=Origin(xyz=(-0.04, 0.0, 0.825)),
        material=support_paint,
        name="wall_plate",
    )
    side_support.visual(
        Box((0.30, 0.86, 0.08)),
        origin=Origin(xyz=(-0.02, 0.0, 0.04)),
        material=support_paint,
        name="floor_foot",
    )
    side_support.visual(
        Box((0.065, 0.36, 0.25)),
        origin=Origin(xyz=(0.0275, 0.0, 1.22)),
        material=support_paint,
        name="backing_pad",
    )
    side_support.visual(
        Box((0.46, 0.20, 0.035)),
        origin=Origin(xyz=(0.28, 0.0, 1.1425)),
        material=support_paint,
        name="lower_cradle",
    )
    side_support.visual(
        Box((0.34, 0.018, 0.18)),
        origin=Origin(xyz=(0.255, 0.092, 1.22)),
        material=support_paint,
        name="side_cheek_0",
    )
    side_support.visual(
        Box((0.34, 0.018, 0.18)),
        origin=Origin(xyz=(0.255, -0.092, 1.22)),
        material=support_paint,
        name="side_cheek_1",
    )
    side_support.visual(
        Box((0.085, 0.25, 0.035)),
        origin=Origin(xyz=(0.045, 0.0, 1.3125)),
        material=support_paint,
        name="upper_bridge",
    )
    side_support.visual(
        Box((0.38, 0.024, 0.055)),
        origin=Origin(xyz=(0.22, 0.145, 1.075)),
        material=support_paint,
        name="lower_gusset_0",
    )
    side_support.visual(
        Box((0.38, 0.024, 0.055)),
        origin=Origin(xyz=(0.22, -0.145, 1.075)),
        material=support_paint,
        name="lower_gusset_1",
    )
    for i, (y, z) in enumerate(
        (
            (-0.26, 0.24),
            (0.26, 0.24),
            (-0.26, 1.48),
            (0.26, 1.48),
        )
    ):
        side_support.visual(
            Cylinder(radius=0.025, length=0.012),
            origin=Origin(xyz=(0.004, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=zinc_bolts,
            name=f"bolt_head_{i}",
        )

    outer_sleeve = model.part("outer_sleeve")
    outer_sleeve.visual(
        mesh_from_cadquery(
            _telescoping_tube(0.0, 0.72, 0.160, 0.120, 0.016),
            "outer_tube",
            tolerance=0.0008,
            angular_tolerance=0.08,
        ),
        material=blue_steel,
        name="outer_tube",
    )
    outer_sleeve.visual(
        mesh_from_cadquery(
            _hollow_rect_x(0.66, 0.755, 0.186, 0.146, 0.124, 0.084),
            "outer_front_collar",
            tolerance=0.0008,
            angular_tolerance=0.08,
        ),
        material=dark_wear,
        name="front_collar",
    )

    middle_section = model.part("middle_section")
    middle_section.visual(
        mesh_from_cadquery(
            _telescoping_tube(-0.52, 0.36, 0.108, 0.074, 0.012),
            "middle_tube",
            tolerance=0.0008,
            angular_tolerance=0.08,
        ),
        material=brushed_steel,
        name="middle_tube",
    )
    middle_section.visual(
        mesh_from_cadquery(
            _hollow_rect_x(0.292, 0.375, 0.126, 0.092, 0.080, 0.052),
            "middle_front_collar",
            tolerance=0.0008,
            angular_tolerance=0.08,
        ),
        material=dark_wear,
        name="front_collar",
    )
    middle_section.visual(
        Box((0.16, 0.070, 0.007)),
        origin=Origin(xyz=(-0.44, 0.0, 0.0405)),
        material=dark_wear,
        name="upper_guide_pad",
    )
    middle_section.visual(
        Box((0.16, 0.070, 0.007)),
        origin=Origin(xyz=(-0.44, 0.0, -0.0405)),
        material=dark_wear,
        name="lower_guide_pad",
    )

    inner_section = model.part("inner_section")
    inner_section.visual(
        mesh_from_cadquery(
            _telescoping_tube(-0.42, 0.30, 0.074, 0.046, 0.008),
            "inner_tube",
            tolerance=0.0008,
            angular_tolerance=0.08,
        ),
        material=brushed_steel,
        name="inner_tube",
    )
    inner_section.visual(
        mesh_from_cadquery(
            _hollow_rect_x(0.238, 0.310, 0.090, 0.060, 0.052, 0.034),
            "inner_front_collar",
            tolerance=0.0008,
            angular_tolerance=0.08,
        ),
        material=dark_wear,
        name="front_collar",
    )
    inner_section.visual(
        Box((0.12, 0.052, 0.002)),
        origin=Origin(xyz=(-0.36, 0.0, 0.024)),
        material=dark_wear,
        name="upper_guide_pad",
    )
    inner_section.visual(
        Box((0.12, 0.052, 0.002)),
        origin=Origin(xyz=(-0.36, 0.0, -0.024)),
        material=dark_wear,
        name="lower_guide_pad",
    )

    tip_section = model.part("tip_section")
    tip_section.visual(
        Box((0.56, 0.046, 0.028)),
        origin=Origin(xyz=(-0.04, 0.0, 0.0)),
        material=brushed_steel,
        name="tip_bar",
    )
    tip_section.visual(
        Box((0.035, 0.064, 0.042)),
        origin=Origin(xyz=(0.2475, 0.0, 0.0)),
        material=dark_wear,
        name="front_stop",
    )
    tip_section.visual(
        Box((0.10, 0.006, 0.016)),
        origin=Origin(xyz=(-0.28, 0.026, 0.0)),
        material=dark_wear,
        name="side_guide_pad_0",
    )
    tip_section.visual(
        Box((0.10, 0.006, 0.016)),
        origin=Origin(xyz=(-0.28, -0.026, 0.0)),
        material=dark_wear,
        name="side_guide_pad_1",
    )

    model.articulation(
        "support_to_outer",
        ArticulationType.FIXED,
        parent=side_support,
        child=outer_sleeve,
        origin=Origin(xyz=(0.06, 0.0, 1.22)),
    )
    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer_sleeve,
        child=middle_section,
        origin=Origin(xyz=(0.70, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.22, lower=0.0, upper=0.34),
        motion_properties=MotionProperties(damping=8.0, friction=2.5),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle_section,
        child=inner_section,
        origin=Origin(xyz=(0.34, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.20, lower=0.0, upper=0.28),
        motion_properties=MotionProperties(damping=7.0, friction=2.0),
    )
    model.articulation(
        "inner_to_tip",
        ArticulationType.PRISMATIC,
        parent=inner_section,
        child=tip_section,
        origin=Origin(xyz=(0.28, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.18, lower=0.0, upper=0.22),
        motion_properties=MotionProperties(damping=6.0, friction=1.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("side_support")
    outer = object_model.get_part("outer_sleeve")
    middle = object_model.get_part("middle_section")
    inner = object_model.get_part("inner_section")
    tip = object_model.get_part("tip_section")
    outer_slide = object_model.get_articulation("outer_to_middle")
    middle_slide = object_model.get_articulation("middle_to_inner")
    tip_slide = object_model.get_articulation("inner_to_tip")

    ctx.allow_overlap(
        middle,
        outer,
        elem_a="upper_guide_pad",
        elem_b="outer_tube",
        reason=(
            "The plastic guide pad is intentionally modeled as a slightly compressed "
            "bearing strip against the inside of the outer sleeve."
        ),
    )
    ctx.allow_overlap(
        middle,
        outer,
        elem_a="lower_guide_pad",
        elem_b="outer_tube",
        reason=(
            "The lower guide pad is a hidden bearing strip that keeps the middle "
            "stage captured in the sleeve."
        ),
    )
    ctx.allow_overlap(
        inner,
        tip,
        elem_a="inner_tube",
        elem_b="side_guide_pad_0",
        reason=(
            "The side guide pad is intentionally represented with slight compression "
            "inside the inner tube so the final stage is supported."
        ),
    )
    ctx.allow_overlap(
        inner,
        tip,
        elem_a="inner_tube",
        elem_b="side_guide_pad_1",
        reason=(
            "The opposite side guide pad is intentionally represented with slight "
            "compression inside the inner tube so the final stage is supported."
        ),
    )

    ctx.expect_gap(
        outer,
        support,
        axis="z",
        max_gap=0.001,
        max_penetration=0.00001,
        positive_elem="outer_tube",
        negative_elem="lower_cradle",
        name="outer sleeve sits on the grounded cradle",
    )
    ctx.expect_overlap(
        outer,
        support,
        axes="xy",
        min_overlap=0.08,
        elem_a="outer_tube",
        elem_b="lower_cradle",
        name="cradle supports the sleeve footprint",
    )
    ctx.expect_contact(
        middle,
        outer,
        elem_a="upper_guide_pad",
        elem_b="outer_tube",
        name="middle stage upper guide bears on the outer sleeve",
    )
    ctx.expect_contact(
        middle,
        outer,
        elem_a="lower_guide_pad",
        elem_b="outer_tube",
        name="middle stage lower guide bears on the outer sleeve",
    )
    ctx.expect_contact(
        tip,
        inner,
        elem_a="side_guide_pad_0",
        elem_b="inner_tube",
        name="tip stage side guide bears on the inner tube",
    )
    ctx.expect_contact(
        tip,
        inner,
        elem_a="side_guide_pad_1",
        elem_b="inner_tube",
        name="opposite tip guide bears on the inner tube",
    )

    with ctx.pose({outer_slide: 0.0, middle_slide: 0.0, tip_slide: 0.0}):
        ctx.expect_within(
            middle,
            outer,
            axes="yz",
            inner_elem="middle_tube",
            outer_elem="outer_tube",
            margin=0.0,
            name="middle stage fits within outer sleeve section",
        )
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            elem_a="middle_tube",
            elem_b="outer_tube",
            min_overlap=0.40,
            name="middle stage has deep collapsed insertion",
        )
        ctx.expect_within(
            inner,
            middle,
            axes="yz",
            inner_elem="inner_tube",
            outer_elem="middle_tube",
            margin=0.0,
            name="inner stage fits within middle section",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            elem_a="inner_tube",
            elem_b="middle_tube",
            min_overlap=0.30,
            name="inner stage has collapsed insertion",
        )
        ctx.expect_within(
            tip,
            inner,
            axes="yz",
            inner_elem="tip_bar",
            outer_elem="inner_tube",
            margin=0.0,
            name="tip stage fits within inner section",
        )
        ctx.expect_overlap(
            tip,
            inner,
            axes="x",
            elem_a="tip_bar",
            elem_b="inner_tube",
            min_overlap=0.20,
            name="tip stage has collapsed insertion",
        )

    rest_tip = ctx.part_world_position(tip)
    with ctx.pose({outer_slide: 0.34, middle_slide: 0.28, tip_slide: 0.22}):
        ctx.expect_within(
            middle,
            outer,
            axes="yz",
            inner_elem="middle_tube",
            outer_elem="outer_tube",
            margin=0.0,
            name="extended middle stage remains centered in outer sleeve",
        )
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            elem_a="middle_tube",
            elem_b="outer_tube",
            min_overlap=0.12,
            name="extended middle stage retains insertion",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            elem_a="inner_tube",
            elem_b="middle_tube",
            min_overlap=0.10,
            name="extended inner stage retains insertion",
        )
        ctx.expect_overlap(
            tip,
            inner,
            axes="x",
            elem_a="tip_bar",
            elem_b="inner_tube",
            min_overlap=0.08,
            name="extended tip stage retains insertion",
        )
        extended_tip = ctx.part_world_position(tip)

    ctx.check(
        "serial prismatic stages extend forward",
        rest_tip is not None
        and extended_tip is not None
        and extended_tip[0] > rest_tip[0] + 0.75,
        details=f"rest_tip={rest_tip}, extended_tip={extended_tip}",
    )

    return ctx.report()


object_model = build_object_model()
