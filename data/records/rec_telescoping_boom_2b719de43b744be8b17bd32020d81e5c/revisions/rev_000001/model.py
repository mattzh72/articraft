from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _rectangular_tube(
    *,
    x_min: float,
    x_max: float,
    width: float,
    height: float,
    wall: float,
) -> cq.Workplane:
    """A straight open-ended rectangular telescoping tube, authored in meters."""
    length = x_max - x_min
    x_center = (x_min + x_max) / 2.0
    outer = cq.Workplane("XY").box(length, width, height).translate((x_center, 0.0, 0.0))
    bore = (
        cq.Workplane("XY")
        .box(length + 0.030, width - 2.0 * wall, height - 2.0 * wall)
        .translate((x_center, 0.0, 0.0))
    )
    return outer.cut(bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_backed_telescoping_reach_arm")

    dark_steel = model.material("dark_steel", color=(0.10, 0.11, 0.12, 1.0))
    black = model.material("black_rubber", color=(0.015, 0.015, 0.012, 1.0))
    outer_metal = model.material("brushed_outer", color=(0.62, 0.66, 0.68, 1.0))
    middle_metal = model.material("brushed_middle", color=(0.76, 0.78, 0.78, 1.0))
    inner_metal = model.material("brushed_inner", color=(0.86, 0.87, 0.86, 1.0))
    tip_metal = model.material("plain_tip", color=(0.70, 0.71, 0.70, 1.0))

    support = model.part("rear_support")
    # Object frame sits on the common telescoping axis through the guide bridge.
    support.visual(
        Box((0.44, 0.40, 0.060)),
        origin=Origin(xyz=(-0.10, 0.0, -0.65)),
        material=dark_steel,
        name="base_plate",
    )
    for y in (-0.135, 0.135):
        support.visual(
            Box((0.12, 0.040, 0.62)),
            origin=Origin(xyz=(-0.11, y, -0.34)),
            material=dark_steel,
            name=f"rear_post_{'negative' if y < 0.0 else 'positive'}",
        )
    support.visual(
        Box((0.14, 0.32, 0.050)),
        origin=Origin(xyz=(-0.10, 0.0, 0.080)),
        material=dark_steel,
        name="upper_bridge",
    )
    support.visual(
        Box((0.14, 0.32, 0.040)),
        origin=Origin(xyz=(-0.10, 0.0, -0.090)),
        material=dark_steel,
        name="lower_bridge",
    )
    # Forward rectangular guide bridge: it surrounds but clears the first stage.
    for y in (-0.105, 0.105):
        support.visual(
            Box((0.32, 0.025, 0.22)),
            origin=Origin(xyz=(0.10, y, 0.0)),
            material=dark_steel,
            name=f"guide_side_{'negative' if y < 0.0 else 'positive'}",
        )
    for z in (-0.085, 0.085):
        support.visual(
            Box((0.32, 0.24, 0.035)),
            origin=Origin(xyz=(0.10, 0.0, z)),
            material=dark_steel,
            name=f"guide_rail_{'lower' if z < 0.0 else 'upper'}",
        )
    support.visual(
        Box((0.030, 0.23, 0.030)),
        origin=Origin(xyz=(0.275, 0.0, -0.085)),
        material=black,
        name="front_wear_pad",
    )
    for z in (-0.06125, 0.06125):
        support.visual(
            Box((0.22, 0.17, 0.0125)),
            origin=Origin(xyz=(0.08, 0.0, z)),
            material=black,
            name=f"bridge_liner_{'lower' if z < 0.0 else 'upper'}",
        )
    for y in (-0.08625, 0.08625):
        support.visual(
            Box((0.22, 0.0125, 0.11)),
            origin=Origin(xyz=(0.08, y, 0.0)),
            material=black,
            name=f"bridge_liner_{'negative' if y < 0.0 else 'positive'}",
        )

    outer_section = model.part("outer_section")
    outer_section.visual(
        mesh_from_cadquery(
            _rectangular_tube(x_min=-0.18, x_max=0.70, width=0.160, height=0.110, wall=0.012),
            "outer_section_tube",
            tolerance=0.0008,
        ),
        material=outer_metal,
        name="outer_tube",
    )

    middle_section = model.part("middle_section")
    middle_section.visual(
        mesh_from_cadquery(
            _rectangular_tube(x_min=-0.45, x_max=0.32, width=0.122, height=0.078, wall=0.010),
            "middle_section_tube",
            tolerance=0.0008,
        ),
        material=middle_metal,
        name="middle_tube",
    )
    for z in (-0.041, 0.041):
        middle_section.visual(
            Box((0.10, 0.060, 0.004)),
            origin=Origin(xyz=(-0.36, 0.0, z)),
            material=black,
            name=f"middle_glide_{'lower' if z < 0.0 else 'upper'}",
        )

    inner_section = model.part("inner_section")
    inner_section.visual(
        mesh_from_cadquery(
            _rectangular_tube(x_min=-0.36, x_max=0.27, width=0.090, height=0.054, wall=0.008),
            "inner_section_tube",
            tolerance=0.0008,
        ),
        material=inner_metal,
        name="inner_tube",
    )
    for z in (-0.028, 0.028):
        inner_section.visual(
            Box((0.08, 0.046, 0.002)),
            origin=Origin(xyz=(-0.30, 0.0, z)),
            material=black,
            name=f"inner_glide_{'lower' if z < 0.0 else 'upper'}",
        )

    tip_section = model.part("tip_section")
    tip_section.visual(
        Box((0.540, 0.060, 0.034)),
        origin=Origin(xyz=(-0.030, 0.0, 0.0)),
        material=tip_metal,
        name="tip_bar",
    )
    tip_section.visual(
        Box((0.008, 0.064, 0.038)),
        origin=Origin(xyz=(0.244, 0.0, 0.0)),
        material=black,
        name="plain_tip_face",
    )
    for z in (-0.018, 0.018):
        tip_section.visual(
            Box((0.06, 0.034, 0.002)),
            origin=Origin(xyz=(-0.25, 0.0, z)),
            material=black,
            name=f"tip_glide_{'lower' if z < 0.0 else 'upper'}",
        )

    model.articulation(
        "support_to_outer",
        ArticulationType.PRISMATIC,
        parent=support,
        child=outer_section,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=130.0, velocity=0.30, lower=0.0, upper=0.18),
    )
    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer_section,
        child=middle_section,
        origin=Origin(xyz=(0.70, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=95.0, velocity=0.28, lower=0.0, upper=0.28),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle_section,
        child=inner_section,
        origin=Origin(xyz=(0.32, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.26, lower=0.0, upper=0.23),
    )
    model.articulation(
        "inner_to_tip",
        ArticulationType.PRISMATIC,
        parent=inner_section,
        child=tip_section,
        origin=Origin(xyz=(0.27, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.22, lower=0.0, upper=0.18),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    support = object_model.get_part("rear_support")
    outer = object_model.get_part("outer_section")
    middle = object_model.get_part("middle_section")
    inner = object_model.get_part("inner_section")
    tip = object_model.get_part("tip_section")
    support_to_outer = object_model.get_articulation("support_to_outer")
    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_inner = object_model.get_articulation("middle_to_inner")
    inner_to_tip = object_model.get_articulation("inner_to_tip")

    for joint in (support_to_outer, outer_to_middle, middle_to_inner, inner_to_tip):
        ctx.check(
            f"{joint.name} shares the reach axis",
            tuple(round(v, 6) for v in joint.axis) == (1.0, 0.0, 0.0),
            details=f"axis={joint.axis}",
        )

    ctx.expect_within(
        outer,
        support,
        axes="yz",
        margin=0.0,
        name="outer section is centered in the rear bridge opening",
    )
    ctx.expect_overlap(
        outer,
        support,
        axes="x",
        min_overlap=0.20,
        name="outer stage remains captured by the support bridge",
    )
    ctx.expect_within(middle, outer, axes="yz", margin=0.0, name="middle stage fits inside outer stage")
    ctx.expect_within(inner, middle, axes="yz", margin=0.0, name="inner stage fits inside middle stage")
    ctx.expect_within(tip, inner, axes="yz", margin=0.0, name="tip stage fits inside inner stage")
    ctx.expect_overlap(middle, outer, axes="x", min_overlap=0.40, name="middle stage retained at rest")
    ctx.expect_overlap(inner, middle, axes="x", min_overlap=0.34, name="inner stage retained at rest")
    ctx.expect_overlap(tip, inner, axes="x", min_overlap=0.28, name="tip stage retained at rest")

    rest_tip = ctx.part_world_position(tip)
    with ctx.pose(
        {
            support_to_outer: 0.18,
            outer_to_middle: 0.28,
            middle_to_inner: 0.23,
            inner_to_tip: 0.18,
        }
    ):
        ctx.expect_overlap(
            outer,
            support,
            axes="x",
            min_overlap=0.08,
            name="extended outer stage remains in bridge",
        )
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            min_overlap=0.13,
            name="extended middle stage remains inserted",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            min_overlap=0.12,
            name="extended inner stage remains inserted",
        )
        ctx.expect_overlap(
            tip,
            inner,
            axes="x",
            min_overlap=0.10,
            name="extended tip stage remains inserted",
        )
        extended_tip = ctx.part_world_position(tip)

    ctx.check(
        "serial prismatic joints extend the plain tip forward",
        rest_tip is not None and extended_tip is not None and extended_tip[0] > rest_tip[0] + 0.80,
        details=f"rest_tip={rest_tip}, extended_tip={extended_tip}",
    )

    return ctx.report()


object_model = build_object_model()
