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
    model = ArticulatedObject(name="wall_hinged_support_arm")

    powder = model.material("powder_coated_graphite", color=(0.08, 0.09, 0.10, 1.0))
    dark = model.material("black_oxide_steel", color=(0.015, 0.016, 0.018, 1.0))
    zinc = model.material("brushed_zinc", color=(0.62, 0.65, 0.66, 1.0))
    brass = model.material("bronze_wear_strip", color=(0.72, 0.55, 0.26, 1.0))
    carriage_mat = model.material("anodized_carriage", color=(0.16, 0.18, 0.20, 1.0))

    wall_bracket = model.part("wall_bracket")
    wall_bracket.visual(
        Box((0.040, 0.200, 0.320)),
        origin=Origin(xyz=(-0.075, 0.0, 0.0)),
        material=powder,
        name="wall_plate",
    )
    wall_bracket.visual(
        Box((0.120, 0.120, 0.035)),
        origin=Origin(xyz=(0.005, 0.0, 0.065)),
        material=powder,
        name="upper_yoke_ear",
    )
    wall_bracket.visual(
        Box((0.120, 0.120, 0.035)),
        origin=Origin(xyz=(0.005, 0.0, -0.065)),
        material=powder,
        name="lower_yoke_ear",
    )
    wall_bracket.visual(
        Box((0.018, 0.120, 0.150)),
        origin=Origin(xyz=(-0.049, 0.0, 0.0)),
        material=powder,
        name="rear_web",
    )
    for z in (0.088, -0.088):
        wall_bracket.visual(
            Cylinder(radius=0.043, length=0.014),
            origin=Origin(xyz=(0.005, 0.0, z)),
            material=zinc,
            name=f"trunnion_cap_{'upper' if z > 0 else 'lower'}",
        )
    for i, (y, z) in enumerate(((-0.070, -0.115), (0.070, -0.115), (-0.070, 0.115), (0.070, 0.115))):
        wall_bracket.visual(
            Cylinder(radius=0.012, length=0.010),
            origin=Origin(xyz=(-0.051, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=zinc,
            name=f"wall_bolt_{i}",
        )
    wall_bracket.visual(
        Box((0.026, 0.030, 0.035)),
        origin=Origin(xyz=(0.060, 0.066, 0.065)),
        material=dark,
        name="swing_stop_0",
    )
    wall_bracket.visual(
        Box((0.026, 0.030, 0.035)),
        origin=Origin(xyz=(0.060, -0.066, -0.065)),
        material=dark,
        name="swing_stop_1",
    )

    main_link = model.part("main_link")
    main_link.visual(
        Cylinder(radius=0.034, length=0.074),
        origin=Origin(),
        material=dark,
        name="hinge_barrel",
    )
    main_link.visual(
        Cylinder(radius=0.041, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=zinc,
        name="upper_pivot_collar",
    )
    main_link.visual(
        Cylinder(radius=0.041, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
        material=zinc,
        name="lower_pivot_collar",
    )
    main_link.visual(
        Box((0.085, 0.114, 0.070)),
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
        material=powder,
        name="root_neck",
    )
    main_link.visual(
        Box((0.670, 0.114, 0.014)),
        origin=Origin(xyz=(0.390, 0.0, -0.034)),
        material=powder,
        name="guide_floor",
    )
    main_link.visual(
        Box((0.670, 0.016, 0.070)),
        origin=Origin(xyz=(0.390, 0.049, 0.0)),
        material=powder,
        name="guide_side_0",
    )
    main_link.visual(
        Box((0.670, 0.016, 0.070)),
        origin=Origin(xyz=(0.390, -0.049, 0.0)),
        material=powder,
        name="guide_side_1",
    )
    main_link.visual(
        Box((0.445, 0.016, 0.012)),
        origin=Origin(xyz=(0.500, 0.033, 0.041)),
        material=powder,
        name="top_lip_0",
    )
    main_link.visual(
        Box((0.445, 0.016, 0.012)),
        origin=Origin(xyz=(0.500, -0.033, 0.041)),
        material=powder,
        name="top_lip_1",
    )
    main_link.visual(
        Box((0.390, 0.006, 0.014)),
        origin=Origin(xyz=(0.495, 0.038, 0.0)),
        material=brass,
        name="wear_strip_0",
    )
    main_link.visual(
        Box((0.390, 0.006, 0.014)),
        origin=Origin(xyz=(0.495, -0.038, 0.0)),
        material=brass,
        name="wear_strip_1",
    )
    main_link.visual(
        Box((0.018, 0.114, 0.012)),
        origin=Origin(xyz=(0.711, 0.0, 0.041)),
        material=powder,
        name="guide_mouth_bridge",
    )
    main_link.visual(
        Box((0.020, 0.010, 0.052)),
        origin=Origin(xyz=(0.710, 0.055, 0.0)),
        material=dark,
        name="end_stop_0",
    )
    main_link.visual(
        Box((0.020, 0.010, 0.052)),
        origin=Origin(xyz=(0.710, -0.055, 0.0)),
        material=dark,
        name="end_stop_1",
    )

    tip_carriage = model.part("tip_carriage")
    tip_carriage.visual(
        Box((0.420, 0.070, 0.030)),
        origin=Origin(xyz=(0.100, 0.0, 0.004)),
        material=carriage_mat,
        name="slider_bar",
    )
    tip_carriage.visual(
        Box((0.026, 0.092, 0.056)),
        origin=Origin(xyz=(0.228, 0.0, 0.004)),
        material=zinc,
        name="stop_collar",
    )
    tip_carriage.visual(
        Box((0.120, 0.102, 0.060)),
        origin=Origin(xyz=(0.340, 0.0, 0.004)),
        material=carriage_mat,
        name="tip_block",
    )
    tip_carriage.visual(
        Box((0.028, 0.118, 0.074)),
        origin=Origin(xyz=(0.414, 0.0, 0.004)),
        material=dark,
        name="front_mount_plate",
    )
    tip_carriage.visual(
        Cylinder(radius=0.010, length=0.008),
        origin=Origin(xyz=(0.429, 0.030, 0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=zinc,
        name="face_bolt_0",
    )
    tip_carriage.visual(
        Cylinder(radius=0.010, length=0.008),
        origin=Origin(xyz=(0.429, -0.030, 0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=zinc,
        name="face_bolt_1",
    )
    tip_carriage.visual(
        Cylinder(radius=0.010, length=0.008),
        origin=Origin(xyz=(0.429, 0.030, -0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=zinc,
        name="face_bolt_2",
    )
    tip_carriage.visual(
        Cylinder(radius=0.010, length=0.008),
        origin=Origin(xyz=(0.429, -0.030, -0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=zinc,
        name="face_bolt_3",
    )

    model.articulation(
        "bracket_to_link",
        ArticulationType.REVOLUTE,
        parent=wall_bracket,
        child=main_link,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.5, lower=-0.95, upper=0.95),
    )
    model.articulation(
        "link_to_carriage",
        ArticulationType.PRISMATIC,
        parent=main_link,
        child=tip_carriage,
        origin=Origin(xyz=(0.520, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.30, lower=0.0, upper=0.180),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bracket = object_model.get_part("wall_bracket")
    link = object_model.get_part("main_link")
    carriage = object_model.get_part("tip_carriage")
    swing = object_model.get_articulation("bracket_to_link")
    slide = object_model.get_articulation("link_to_carriage")

    ctx.expect_overlap(
        link,
        bracket,
        axes="xy",
        min_overlap=0.045,
        elem_a="hinge_barrel",
        elem_b="upper_yoke_ear",
        name="hinge barrel is captured in the bracket planform",
    )
    ctx.expect_gap(
        bracket,
        link,
        axis="z",
        min_gap=0.003,
        max_gap=0.020,
        positive_elem="upper_yoke_ear",
        negative_elem="upper_pivot_collar",
        name="upper pivot collar clears the fixed yoke ear",
    )
    ctx.expect_gap(
        link,
        bracket,
        axis="z",
        min_gap=0.003,
        max_gap=0.020,
        positive_elem="lower_pivot_collar",
        negative_elem="lower_yoke_ear",
        name="lower pivot collar clears the fixed yoke ear",
    )
    ctx.expect_overlap(
        carriage,
        link,
        axes="x",
        min_overlap=0.240,
        elem_a="slider_bar",
        elem_b="guide_floor",
        name="collapsed carriage remains deeply inserted in the guide",
    )
    ctx.expect_gap(
        carriage,
        link,
        axis="z",
        min_gap=0.010,
        max_gap=0.030,
        positive_elem="slider_bar",
        negative_elem="guide_floor",
        name="slider bar rides above the guide floor with clearance",
    )
    ctx.expect_gap(
        link,
        carriage,
        axis="z",
        min_gap=0.010,
        max_gap=0.030,
        positive_elem="top_lip_0",
        negative_elem="slider_bar",
        name="upper guide lip clears the slider bar",
    )
    ctx.expect_contact(
        carriage,
        link,
        elem_a="slider_bar",
        elem_b="wear_strip_0",
        name="positive side wear strip supports the slider bar",
    )
    ctx.expect_contact(
        carriage,
        link,
        elem_a="slider_bar",
        elem_b="wear_strip_1",
        name="negative side wear strip supports the slider bar",
    )
    ctx.expect_gap(
        carriage,
        link,
        axis="x",
        min_gap=0.006,
        max_gap=0.040,
        positive_elem="stop_collar",
        negative_elem="guide_mouth_bridge",
        name="stop collar starts clear of the distal guide mouth",
    )

    rest_tip = ctx.part_element_world_aabb(carriage, elem="front_mount_plate")
    with ctx.pose({slide: 0.180}):
        ctx.expect_overlap(
            carriage,
            link,
            axes="x",
            min_overlap=0.110,
            elem_a="slider_bar",
            elem_b="guide_floor",
            name="extended carriage keeps retained insertion in the guide",
        )
        ctx.expect_gap(
            carriage,
            link,
            axis="x",
            min_gap=0.150,
            positive_elem="stop_collar",
            negative_elem="guide_mouth_bridge",
            name="stop collar moves outward from the distal guide mouth",
        )
        extended_tip = ctx.part_element_world_aabb(carriage, elem="front_mount_plate")
    ctx.check(
        "prismatic joint extends along the main link",
        rest_tip is not None
        and extended_tip is not None
        and extended_tip[0][0] > rest_tip[0][0] + 0.160,
        details=f"rest_tip={rest_tip}, extended_tip={extended_tip}",
    )

    with ctx.pose({swing: 0.70, slide: 0.120}):
        swung_tip = ctx.part_element_world_aabb(carriage, elem="front_mount_plate")
    ctx.check(
        "revolute swing moves the extended carriage sideways before extension stack",
        swung_tip is not None and swung_tip[1][1] > 0.55,
        details=f"swung_tip={swung_tip}",
    )

    return ctx.report()


object_model = build_object_model()
