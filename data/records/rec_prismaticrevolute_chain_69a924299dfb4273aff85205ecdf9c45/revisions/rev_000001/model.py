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
    model = ArticulatedObject(name="compact_transfer_axis")

    anodized_black = model.material("anodized_black", rgba=(0.025, 0.027, 0.030, 1.0))
    dark_rail = model.material("dark_hardened_rail", rgba=(0.11, 0.12, 0.13, 1.0))
    blue_carriage = model.material("blue_carriage", rgba=(0.05, 0.20, 0.48, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    orange_tab = model.material("orange_output_tab", rgba=(0.95, 0.38, 0.08, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))

    guide = model.part("guide_body")
    guide.visual(
        Box((0.340, 0.120, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=anodized_black,
        name="base_plate",
    )
    for y, side_name, lip_name, support_name in (
        (-0.052, "side_rail_0", "keeper_lip_0", "support_track_0"),
        (0.052, "side_rail_1", "keeper_lip_1", "support_track_1"),
    ):
        guide.visual(
            Box((0.320, 0.016, 0.058)),
            origin=Origin(xyz=(0.0, y, 0.043)),
            material=anodized_black,
            name=side_name,
        )
        guide.visual(
            Box((0.304, 0.020, 0.011)),
            origin=Origin(xyz=(0.0, y * 0.69, 0.074)),
            material=anodized_black,
            name=lip_name,
        )
        guide.visual(
            Box((0.300, 0.018, 0.009)),
            origin=Origin(xyz=(0.0, y * 0.46, 0.0185)),
            material=dark_rail,
            name=support_name,
        )
    guide.visual(
        Box((0.018, 0.120, 0.062)),
        origin=Origin(xyz=(-0.161, 0.0, 0.045)),
        material=anodized_black,
        name="rear_end_stop",
    )
    guide.visual(
        Box((0.018, 0.120, 0.034)),
        origin=Origin(xyz=(0.161, 0.0, 0.031)),
        material=anodized_black,
        name="front_low_stop",
    )
    for x, y, suffix in (
        (-0.118, -0.036, "0"),
        (-0.118, 0.036, "1"),
        (0.118, -0.036, "2"),
        (0.118, 0.036, "3"),
    ):
        guide.visual(
            Cylinder(radius=0.007, length=0.004),
            origin=Origin(xyz=(x, y, 0.0155)),
            material=brushed_steel,
            name=f"cap_screw_{suffix}",
        )

    carriage = model.part("carriage_head")
    carriage.visual(
        Box((0.090, 0.064, 0.044)),
        origin=Origin(xyz=(0.045, 0.0, 0.0)),
        material=blue_carriage,
        name="head_block",
    )
    carriage.visual(
        Box((0.012, 0.074, 0.052)),
        origin=Origin(xyz=(0.096, 0.0, 0.001)),
        material=blue_carriage,
        name="front_face",
    )
    carriage.visual(
        Box((0.030, 0.055, 0.010)),
        origin=Origin(xyz=(0.037, 0.0, 0.027)),
        material=brushed_steel,
        name="top_wear_plate",
    )
    for y, lug_name, web_name in (
        (-0.0315, "front_lug_0", "lug_web_0"),
        (0.0315, "front_lug_1", "lug_web_1"),
    ):
        carriage.visual(
            Cylinder(radius=0.010, length=0.019),
            origin=Origin(xyz=(0.105, y, 0.010), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=brushed_steel,
            name=lug_name,
        )
        carriage.visual(
            Box((0.018, 0.013, 0.024)),
            origin=Origin(xyz=(0.098, y, -0.003)),
            material=blue_carriage,
            name=web_name,
        )

    tab = model.part("output_tab")
    tab.visual(
        Cylinder(radius=0.007, length=0.044),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="hinge_barrel",
    )
    tab.visual(
        Box((0.074, 0.046, 0.007)),
        origin=Origin(xyz=(0.041, 0.0, -0.0095)),
        material=orange_tab,
        name="tab_plate",
    )
    tab.visual(
        Box((0.024, 0.040, 0.010)),
        origin=Origin(xyz=(0.013, 0.0, -0.0045)),
        material=orange_tab,
        name="hinge_strap",
    )
    tab.visual(
        Box((0.022, 0.040, 0.006)),
        origin=Origin(xyz=(0.073, 0.0, -0.0095)),
        material=rubber,
        name="contact_pad",
    )

    model.articulation(
        "guide_to_carriage",
        ArticulationType.PRISMATIC,
        parent=guide,
        child=carriage,
        origin=Origin(xyz=(-0.080, 0.0, 0.045)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=0.0, upper=0.120),
    )

    model.articulation(
        "carriage_to_tab",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=tab,
        origin=Origin(xyz=(0.105, 0.0, 0.010)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=4.0, lower=-0.65, upper=1.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    guide = object_model.get_part("guide_body")
    carriage = object_model.get_part("carriage_head")
    tab = object_model.get_part("output_tab")
    slide = object_model.get_articulation("guide_to_carriage")
    hinge = object_model.get_articulation("carriage_to_tab")

    ctx.expect_gap(
        carriage,
        guide,
        axis="z",
        positive_elem="head_block",
        negative_elem="support_track_0",
        max_gap=0.0005,
        max_penetration=0.0,
        name="carriage sits on the guide support track",
    )
    ctx.expect_overlap(
        carriage,
        guide,
        axes="x",
        elem_a="head_block",
        elem_b="side_rail_0",
        min_overlap=0.080,
        name="carriage remains captured by the slide rail at rest",
    )
    ctx.expect_contact(
        tab,
        carriage,
        elem_a="hinge_barrel",
        elem_b="front_lug_0",
        contact_tol=0.001,
        name="output tab hinge barrel is carried by the carriage lug",
    )

    rest_carriage = ctx.part_world_position(carriage)
    rest_tab = ctx.part_world_position(tab)
    with ctx.pose({slide: 0.120}):
        extended_carriage = ctx.part_world_position(carriage)
        extended_tab = ctx.part_world_position(tab)
        ctx.expect_overlap(
            carriage,
            guide,
            axes="x",
            elem_a="head_block",
            elem_b="side_rail_0",
            min_overlap=0.080,
            name="extended carriage still has retained rail engagement",
        )

    ctx.check(
        "prismatic joint moves the carriage along the transfer axis",
        rest_carriage is not None
        and extended_carriage is not None
        and extended_carriage[0] > rest_carriage[0] + 0.110
        and abs(extended_carriage[1] - rest_carriage[1]) < 1e-6
        and abs(extended_carriage[2] - rest_carriage[2]) < 1e-6,
        details=f"rest={rest_carriage}, extended={extended_carriage}",
    )
    ctx.check(
        "hinged tab rides with the moving carriage",
        rest_tab is not None
        and extended_tab is not None
        and extended_tab[0] > rest_tab[0] + 0.110,
        details=f"rest={rest_tab}, extended={extended_tab}",
    )

    rest_plate_aabb = ctx.part_element_world_aabb(tab, elem="tab_plate")
    with ctx.pose({hinge: 1.0}):
        raised_plate_aabb = ctx.part_element_world_aabb(tab, elem="tab_plate")

    rest_plate_center_z = (
        (rest_plate_aabb[0][2] + rest_plate_aabb[1][2]) * 0.5
        if rest_plate_aabb is not None
        else None
    )
    raised_plate_center_z = (
        (raised_plate_aabb[0][2] + raised_plate_aabb[1][2]) * 0.5
        if raised_plate_aabb is not None
        else None
    )
    ctx.check(
        "revolute joint lifts the flap-like output tab",
        rest_plate_center_z is not None
        and raised_plate_center_z is not None
        and raised_plate_center_z > rest_plate_center_z + 0.035,
        details=f"rest_z={rest_plate_center_z}, raised_z={raised_plate_center_z}",
    )

    return ctx.report()


object_model = build_object_model()
