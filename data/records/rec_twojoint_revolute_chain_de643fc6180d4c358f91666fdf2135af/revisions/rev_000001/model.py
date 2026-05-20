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
    model = ArticulatedObject(name="boxed_industrial_two_joint_chain")

    painted_base = model.material("painted_base", rgba=(0.18, 0.20, 0.22, 1.0))
    link_paint = model.material("link_paint", rgba=(0.08, 0.20, 0.32, 1.0))
    boss_steel = model.material("boss_steel", rgba=(0.58, 0.61, 0.62, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.05, 0.055, 0.06, 1.0))
    warning_yellow = model.material("end_tab_yellow", rgba=(0.95, 0.63, 0.08, 1.0))

    cyl_along_y = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))

    base = model.part("base")
    base.visual(
        Box((0.58, 0.46, 0.08)),
        origin=Origin(xyz=(0.08, 0.0, 0.04)),
        material=painted_base,
        name="base_plate",
    )
    base.visual(
        Box((0.25, 0.050, 0.300)),
        origin=Origin(xyz=(0.0, 0.155, 0.205)),
        material=painted_base,
        name="cheek_positive",
    )
    base.visual(
        Box((0.25, 0.050, 0.300)),
        origin=Origin(xyz=(0.0, -0.155, 0.205)),
        material=painted_base,
        name="cheek_negative",
    )
    base.visual(
        Cylinder(radius=0.032, length=0.420),
        origin=Origin(xyz=(0.0, 0.0, 0.300), rpy=cyl_along_y.rpy),
        material=dark_steel,
        name="base_pin",
    )
    for y, name in ((0.1975, "outer_pin_positive"), (-0.1975, "outer_pin_negative")):
        base.visual(
            Cylinder(radius=0.078, length=0.035),
            origin=Origin(xyz=(0.0, y, 0.300), rpy=cyl_along_y.rpy),
            material=boss_steel,
            name=name,
        )
    for x in (-0.090, 0.105):
        for y, name in ((0.118, "rib_positive"), (-0.118, "rib_negative")):
            base.visual(
                Box((0.060, 0.045, 0.210)),
                origin=Origin(xyz=(x, y, 0.155)),
                material=painted_base,
                name=f"{name}_{0 if x < 0 else 1}",
            )
    for x in (-0.145, 0.265):
        for y in (-0.160, 0.160):
            base.visual(
                Cylinder(radius=0.018, length=0.014),
                origin=Origin(xyz=(x, y, 0.083)),
                material=dark_steel,
                name=f"bolt_{'rear' if x < 0 else 'front'}_{'negative' if y < 0 else 'positive'}",
            )

    link_0 = model.part("link_0")
    link_0.visual(
        Box((0.450, 0.165, 0.130)),
        origin=Origin(xyz=(0.295, 0.0, 0.0)),
        material=link_paint,
        name="box_section",
    )
    link_0.visual(
        Box((0.420, 0.175, 0.026)),
        origin=Origin(xyz=(0.305, 0.0, 0.066)),
        material=link_paint,
        name="top_flange",
    )
    link_0.visual(
        Box((0.420, 0.175, 0.026)),
        origin=Origin(xyz=(0.305, 0.0, -0.066)),
        material=link_paint,
        name="bottom_flange",
    )
    link_0.visual(
        Cylinder(radius=0.095, length=0.190),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=cyl_along_y.rpy),
        material=boss_steel,
        name="proximal_boss",
    )
    link_0.visual(
        Cylinder(radius=0.058, length=0.196),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=cyl_along_y.rpy),
        material=dark_steel,
        name="proximal_pin_face",
    )
    link_0.visual(
        Box((0.155, 0.035, 0.120)),
        origin=Origin(xyz=(0.570, 0.095, 0.0)),
        material=link_paint,
        name="distal_strap_positive",
    )
    link_0.visual(
        Box((0.155, 0.035, 0.120)),
        origin=Origin(xyz=(0.570, -0.095, 0.0)),
        material=link_paint,
        name="distal_strap_negative",
    )
    for y, suffix in ((0.125, "positive"), (-0.125, "negative")):
        link_0.visual(
            Cylinder(radius=0.090, length=0.045),
            origin=Origin(xyz=(0.640, y, 0.0), rpy=cyl_along_y.rpy),
            material=boss_steel,
            name=f"distal_boss_{suffix}",
        )
    link_0.visual(
        Cylinder(radius=0.030, length=0.275),
        origin=Origin(xyz=(0.640, 0.0, 0.0), rpy=cyl_along_y.rpy),
        material=dark_steel,
        name="distal_pin",
    )

    link_1 = model.part("link_1")
    link_1.visual(
        Cylinder(radius=0.080, length=0.134),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=cyl_along_y.rpy),
        material=boss_steel,
        name="proximal_boss",
    )
    link_1.visual(
        Cylinder(radius=0.048, length=0.140),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=cyl_along_y.rpy),
        material=dark_steel,
        name="proximal_pin_face",
    )
    link_1.visual(
        Box((0.360, 0.115, 0.110)),
        origin=Origin(xyz=(0.240, 0.0, 0.0)),
        material=link_paint,
        name="box_section",
    )
    link_1.visual(
        Box((0.120, 0.105, 0.080)),
        origin=Origin(xyz=(0.460, 0.0, 0.0)),
        material=warning_yellow,
        name="end_tab",
    )
    link_1.visual(
        Cylinder(radius=0.050, length=0.112),
        origin=Origin(xyz=(0.520, 0.0, 0.0), rpy=cyl_along_y.rpy),
        material=warning_yellow,
        name="tab_round_end",
    )
    link_1.visual(
        Cylinder(radius=0.022, length=0.118),
        origin=Origin(xyz=(0.520, 0.0, 0.0), rpy=cyl_along_y.rpy),
        material=dark_steel,
        name="tab_pin_face",
    )

    model.articulation(
        "base_to_link_0",
        ArticulationType.REVOLUTE,
        parent=base,
        child=link_0,
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=1.4, lower=-1.20, upper=1.20),
    )
    model.articulation(
        "link_0_to_link_1",
        ArticulationType.REVOLUTE,
        parent=link_0,
        child=link_1,
        origin=Origin(xyz=(0.640, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.6, lower=-1.35, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    link_0 = object_model.get_part("link_0")
    link_1 = object_model.get_part("link_1")
    joint_0 = object_model.get_articulation("base_to_link_0")
    joint_1 = object_model.get_articulation("link_0_to_link_1")

    ctx.allow_overlap(
        base,
        link_0,
        elem_a="base_pin",
        elem_b="proximal_boss",
        reason="The first hinge pin is intentionally captured through the link boss.",
    )
    ctx.allow_overlap(
        base,
        link_0,
        elem_a="base_pin",
        elem_b="proximal_pin_face",
        reason="The visible first-joint pin face shares the same captured hinge shaft.",
    )
    ctx.allow_overlap(
        link_0,
        link_1,
        elem_a="distal_pin",
        elem_b="proximal_boss",
        reason="The second hinge pin is intentionally captured through the link boss.",
    )
    ctx.allow_overlap(
        link_0,
        link_1,
        elem_a="distal_pin",
        elem_b="proximal_pin_face",
        reason="The visible second-joint pin face shares the same captured hinge shaft.",
    )

    ctx.check(
        "serial chain has two revolute joints",
        len(object_model.articulations) == 2
        and joint_0.articulation_type == ArticulationType.REVOLUTE
        and joint_1.articulation_type == ArticulationType.REVOLUTE,
    )
    ctx.check(
        "hinge axes are parallel",
        joint_0.axis == joint_1.axis == (0.0, 1.0, 0.0),
        details=f"axis0={joint_0.axis}, axis1={joint_1.axis}",
    )

    ctx.expect_gap(
        base,
        link_0,
        axis="y",
        positive_elem="cheek_positive",
        negative_elem="proximal_boss",
        min_gap=0.020,
        max_gap=0.040,
        name="first boss clears positive base cheek",
    )
    ctx.expect_gap(
        link_0,
        base,
        axis="y",
        positive_elem="proximal_boss",
        negative_elem="cheek_negative",
        min_gap=0.020,
        max_gap=0.040,
        name="first boss clears negative base cheek",
    )
    ctx.expect_overlap(
        link_0,
        base,
        axes="xz",
        elem_a="proximal_boss",
        elem_b="cheek_positive",
        min_overlap=0.080,
        name="first hinge boss sits inside base cheek height",
    )
    ctx.expect_overlap(
        base,
        link_0,
        axes="y",
        elem_a="base_pin",
        elem_b="proximal_boss",
        min_overlap=0.180,
        name="first hinge pin passes through proximal boss",
    )
    ctx.expect_overlap(
        base,
        link_0,
        axes="y",
        elem_a="base_pin",
        elem_b="proximal_pin_face",
        min_overlap=0.180,
        name="first hinge shaft aligns with visible pin face",
    )
    ctx.expect_gap(
        link_0,
        link_1,
        axis="y",
        positive_elem="distal_strap_positive",
        negative_elem="proximal_boss",
        min_gap=0.006,
        max_gap=0.018,
        name="second boss clears positive fork strap",
    )
    ctx.expect_gap(
        link_1,
        link_0,
        axis="y",
        positive_elem="proximal_boss",
        negative_elem="distal_strap_negative",
        min_gap=0.006,
        max_gap=0.018,
        name="second boss clears negative fork strap",
    )
    ctx.expect_overlap(
        link_0,
        link_1,
        axes="y",
        elem_a="distal_pin",
        elem_b="proximal_boss",
        min_overlap=0.130,
        name="second hinge pin passes through proximal boss",
    )
    ctx.expect_overlap(
        link_0,
        link_1,
        axes="y",
        elem_a="distal_pin",
        elem_b="proximal_pin_face",
        min_overlap=0.130,
        name="second hinge shaft aligns with visible pin face",
    )

    rest_tip = ctx.part_element_world_aabb(link_1, elem="end_tab")
    with ctx.pose({joint_0: 0.45, joint_1: -0.70}):
        posed_tip = ctx.part_element_world_aabb(link_1, elem="end_tab")
    ctx.check(
        "end tab moves under revolute chain pose",
        rest_tip is not None
        and posed_tip is not None
        and abs(posed_tip[0][0] - rest_tip[0][0]) > 0.060
        and abs(posed_tip[0][2] - rest_tip[0][2]) > 0.060,
        details=f"rest={rest_tip}, posed={posed_tip}",
    )

    return ctx.report()


object_model = build_object_model()
