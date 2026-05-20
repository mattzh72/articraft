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
    model = ArticulatedObject(name="underslung_two_joint_chain")

    zinc = model.material("zinc_plated_steel", color=(0.68, 0.70, 0.69, 1.0))
    dark = model.material("dark_bushed_steel", color=(0.08, 0.085, 0.09, 1.0))
    link_paint = model.material("safety_yellow_link", color=(0.95, 0.65, 0.12, 1.0))
    tab_paint = model.material("worn_orange_tab", color=(0.90, 0.36, 0.10, 1.0))

    hinge_rpy = (math.pi / 2.0, 0.0, 0.0)

    support = model.part("top_support")
    support.visual(
        Box((0.240, 0.160, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.088)),
        material=zinc,
        name="mount_plate",
    )
    support.visual(
        Box((0.080, 0.132, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.0675)),
        material=zinc,
        name="cross_web",
    )
    for y, name in ((-0.060, "cheek_0"), (0.060, "cheek_1")):
        support.visual(
            Box((0.058, 0.020, 0.095)),
            origin=Origin(xyz=(0.0, y, 0.0275)),
            material=zinc,
            name=name,
        )
    for y, name in ((-0.074, "outer_boss_0"), (0.074, "outer_boss_1")):
        support.visual(
            Cylinder(radius=0.018, length=0.018),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=hinge_rpy),
            material=dark,
            name=name,
        )
    support.visual(
        Cylinder(radius=0.009, length=0.150),
        origin=Origin(rpy=hinge_rpy),
        material=dark,
        name="support_pin",
    )
    for x in (-0.075, 0.075):
        for y in (-0.050, 0.050):
            support.visual(
                Cylinder(radius=0.010, length=0.006),
                origin=Origin(xyz=(x, y, 0.102)),
                material=dark,
                name=f"bolt_{x:+.3f}_{y:+.3f}",
            )

    upper_link = model.part("upper_link")
    upper_link.visual(
        Cylinder(radius=0.022, length=0.070),
        origin=Origin(rpy=hinge_rpy),
        material=link_paint,
        name="upper_lug",
    )
    upper_link.visual(
        Box((0.028, 0.010, 0.250)),
        origin=Origin(xyz=(0.0, -0.034, -0.142)),
        material=link_paint,
        name="side_plate_0",
    )
    upper_link.visual(
        Box((0.028, 0.010, 0.250)),
        origin=Origin(xyz=(0.0, 0.034, -0.142)),
        material=link_paint,
        name="side_plate_1",
    )
    upper_link.visual(
        Cylinder(radius=0.024, length=0.012),
        origin=Origin(xyz=(0.0, -0.034, -0.285), rpy=hinge_rpy),
        material=link_paint,
        name="lower_boss_0",
    )
    upper_link.visual(
        Cylinder(radius=0.024, length=0.012),
        origin=Origin(xyz=(0.0, 0.034, -0.285), rpy=hinge_rpy),
        material=link_paint,
        name="lower_boss_1",
    )
    upper_link.visual(
        Cylinder(radius=0.009, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, -0.285), rpy=hinge_rpy),
        material=dark,
        name="middle_pin",
    )
    lower_link = model.part("lower_link")
    lower_link.visual(
        Cylinder(radius=0.021, length=0.046),
        origin=Origin(rpy=hinge_rpy),
        material=tab_paint,
        name="upper_lug",
    )
    lower_link.visual(
        Box((0.030, 0.026, 0.190)),
        origin=Origin(xyz=(0.0, 0.0, -0.110)),
        material=tab_paint,
        name="web_strap",
    )
    lower_link.visual(
        Box((0.050, 0.034, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, -0.225)),
        material=tab_paint,
        name="end_tab_plate",
    )
    lower_link.visual(
        Cylinder(radius=0.025, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, -0.245), rpy=hinge_rpy),
        material=tab_paint,
        name="end_tab_round",
    )
    for y, name in ((-0.0185, "tab_eye_0"), (0.0185, "tab_eye_1")):
        lower_link.visual(
            Cylinder(radius=0.008, length=0.005),
            origin=Origin(xyz=(0.0, y, -0.235), rpy=hinge_rpy),
            material=dark,
            name=name,
        )

    model.articulation(
        "support_hinge",
        ArticulationType.REVOLUTE,
        parent=support,
        child=upper_link,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-1.05, upper=1.05),
    )
    model.articulation(
        "middle_hinge",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=lower_link,
        origin=Origin(xyz=(0.0, 0.0, -0.285)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.2, lower=-1.25, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    support = object_model.get_part("top_support")
    upper_link = object_model.get_part("upper_link")
    lower_link = object_model.get_part("lower_link")
    support_hinge = object_model.get_articulation("support_hinge")
    middle_hinge = object_model.get_articulation("middle_hinge")

    ctx.allow_overlap(
        support,
        upper_link,
        elem_a="support_pin",
        elem_b="upper_lug",
        reason="The top bracket pin is intentionally captured through the first link lug.",
    )
    ctx.allow_overlap(
        upper_link,
        lower_link,
        elem_a="middle_pin",
        elem_b="upper_lug",
        reason="The second hinge pin is intentionally captured through the next link lug.",
    )

    hinges = [support_hinge, middle_hinge]
    ctx.check(
        "two parallel revolute hinges",
        len(hinges) == 2
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in hinges)
        and all(tuple(round(v, 6) for v in j.axis) == (0.0, 1.0, 0.0) for j in hinges),
        details=f"hinge axes={[j.axis for j in hinges]}",
    )

    ctx.expect_origin_gap(
        support,
        lower_link,
        axis="z",
        min_gap=0.270,
        max_gap=0.300,
        name="second hinge hangs below the top bracket",
    )
    ctx.expect_within(
        upper_link,
        support,
        axes="y",
        inner_elem="upper_lug",
        outer_elem="mount_plate",
        margin=0.0,
        name="first lug is captured under the support width",
    )
    ctx.expect_gap(
        lower_link,
        upper_link,
        axis="y",
        min_gap=0.004,
        max_gap=0.008,
        positive_elem="upper_lug",
        negative_elem="side_plate_0",
        name="second lug clears lower fork cheek 0",
    )
    ctx.expect_gap(
        upper_link,
        lower_link,
        axis="y",
        min_gap=0.004,
        max_gap=0.008,
        positive_elem="side_plate_1",
        negative_elem="upper_lug",
        name="second lug clears lower fork cheek 1",
    )
    ctx.expect_overlap(
        support,
        upper_link,
        axes="xyz",
        elem_a="support_pin",
        elem_b="upper_lug",
        min_overlap=0.015,
        name="support pin passes through first lug",
    )
    ctx.expect_overlap(
        upper_link,
        lower_link,
        axes="xyz",
        elem_a="middle_pin",
        elem_b="upper_lug",
        min_overlap=0.015,
        name="middle pin passes through second lug",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    rest_tip = _aabb_center(ctx.part_element_world_aabb(lower_link, elem="end_tab_round"))
    with ctx.pose({middle_hinge: 0.70}):
        swung_tip = _aabb_center(ctx.part_element_world_aabb(lower_link, elem="end_tab_round"))
    ctx.check(
        "lower hinge swings compact end tab",
        rest_tip is not None
        and swung_tip is not None
        and abs(swung_tip[0] - rest_tip[0]) > 0.10,
        details=f"rest_tip={rest_tip}, swung_tip={swung_tip}",
    )

    return ctx.report()


object_model = build_object_model()
