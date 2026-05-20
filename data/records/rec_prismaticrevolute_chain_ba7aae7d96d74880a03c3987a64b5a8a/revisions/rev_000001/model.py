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
    model = ArticulatedObject(name="inspection_shuttle")

    steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.64, 1.0))
    dark_steel = model.material("darkened_steel", rgba=(0.12, 0.13, 0.14, 1.0))
    blue = model.material("anodized_blue", rgba=(0.10, 0.25, 0.50, 1.0))
    brass = model.material("oilite_bronze", rgba=(0.78, 0.58, 0.25, 1.0))
    black = model.material("black_oxide", rgba=(0.02, 0.02, 0.018, 1.0))
    orange = model.material("inspection_orange", rgba=(0.95, 0.36, 0.08, 1.0))

    guide = model.part("guide")
    guide.visual(
        Box((0.82, 0.30, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_steel,
        name="base_plate",
    )
    guide.visual(
        Box((0.72, 0.018, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.039)),
        material=black,
        name="center_way",
    )
    for y, rail_name, bolt_names in (
        (0.07, "rail_0", ("rail_bolt_0_0", "rail_bolt_0_1")),
        (-0.07, "rail_1", ("rail_bolt_1_0", "rail_bolt_1_1")),
    ):
        guide.visual(
            Box((0.70, 0.025, 0.045)),
            origin=Origin(xyz=(0.0, y, 0.0575)),
            material=steel,
            name=rail_name,
        )
        for x, bolt_name in zip((-0.29, 0.29), bolt_names):
            guide.visual(
                Cylinder(radius=0.008, length=0.006),
                origin=Origin(xyz=(x, y, 0.083)),
                material=black,
                name=bolt_name,
            )
    for x, stop_name, bolt_name in (
        (-0.38, "end_stop_0", "stop_bolt_0"),
        (0.38, "end_stop_1", "stop_bolt_1"),
    ):
        guide.visual(
            Box((0.030, 0.22, 0.075)),
            origin=Origin(xyz=(x, 0.0, 0.0725)),
            material=steel,
            name=stop_name,
        )
        guide.visual(
            Cylinder(radius=0.012, length=0.010),
            origin=Origin(xyz=(x, 0.095, 0.115), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=black,
            name=bolt_name,
        )

    shuttle = model.part("shuttle")
    shuttle.visual(
        Box((0.22, 0.18, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.1005)),
        material=blue,
        name="carriage_plate",
    )
    for y, pad_name in ((0.07, "wear_pad_0"), (-0.07, "wear_pad_1")):
        shuttle.visual(
            Box((0.18, 0.024, 0.008)),
            origin=Origin(xyz=(0.0, y, 0.084)),
            material=brass,
            name=pad_name,
        )
    for idx, y in enumerate((0.105, -0.105)):
        shuttle.visual(
            Box((0.18, 0.016, 0.048)),
            origin=Origin(xyz=(0.0, y, 0.064)),
            material=blue,
            name=f"carriage_flange_{idx}",
        )
        shuttle.visual(
            Box((0.18, 0.018, 0.020)),
            origin=Origin(xyz=(0.0, 0.093 if y > 0 else -0.093, 0.094)),
            material=blue,
            name=f"flange_web_{idx}",
        )
        for j, x in enumerate((-0.065, 0.065)):
            shuttle.visual(
                Cylinder(radius=0.007, length=0.007),
                origin=Origin(xyz=(x, y + (0.011 if y > 0 else -0.011), 0.060), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=black,
                name=f"flange_bolt_{idx}_{j}",
            )
    shuttle.visual(
        Box((0.095, 0.045, 0.025)),
        origin=Origin(xyz=(0.045, -0.105, 0.125)),
        material=blue,
        name="side_shelf",
    )
    shuttle.visual(
        Box((0.075, 0.050, 0.065)),
        origin=Origin(xyz=(0.045, -0.145, 0.1455)),
        material=blue,
        name="pivot_block",
    )
    for idx, x in enumerate((0.018, 0.072)):
        shuttle.visual(
            Cylinder(radius=0.010, length=0.008),
            origin=Origin(xyz=(x, -0.174, 0.146), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=black,
            name=f"block_bolt_{idx}",
        )
    shuttle.visual(
        Cylinder(radius=0.024, length=0.010),
        origin=Origin(xyz=(0.045, -0.145, 0.183)),
        material=steel,
        name="pivot_bearing",
    )

    link = model.part("link_paddle")
    link.visual(
        Cylinder(radius=0.028, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=steel,
        name="link_hub",
    )
    link.visual(
        Box((0.026, 0.135, 0.014)),
        origin=Origin(xyz=(0.0, -0.077, 0.007)),
        material=steel,
        name="arm_bar",
    )
    link.visual(
        Box((0.090, 0.055, 0.010)),
        origin=Origin(xyz=(0.0, -0.168, 0.007)),
        material=orange,
        name="paddle_plate",
    )
    for idx, x in enumerate((-0.045, 0.045)):
        link.visual(
            Cylinder(radius=0.0275, length=0.010),
            origin=Origin(xyz=(x, -0.168, 0.007)),
            material=orange,
            name=f"paddle_round_{idx}",
        )
    link.visual(
        Cylinder(radius=0.017, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=black,
        name="shoulder_cap",
    )
    link.visual(
        Cylinder(radius=0.004, length=0.020),
        origin=Origin(xyz=(0.0, -0.168, 0.015)),
        material=black,
        name="inspection_tip",
    )

    model.articulation(
        "guide_to_shuttle",
        ArticulationType.PRISMATIC,
        parent=guide,
        child=shuttle,
        origin=Origin(xyz=(-0.18, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.36),
    )
    model.articulation(
        "shuttle_to_link",
        ArticulationType.REVOLUTE,
        parent=shuttle,
        child=link,
        origin=Origin(xyz=(0.045, -0.145, 0.188)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.1),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    guide = object_model.get_part("guide")
    shuttle = object_model.get_part("shuttle")
    link = object_model.get_part("link_paddle")
    slide = object_model.get_articulation("guide_to_shuttle")
    hinge = object_model.get_articulation("shuttle_to_link")

    ctx.check(
        "shuttle uses a prismatic guide joint",
        slide.articulation_type == ArticulationType.PRISMATIC
        and slide.axis == (1.0, 0.0, 0.0)
        and slide.motion_limits is not None
        and slide.motion_limits.upper == 0.36,
        details=f"type={slide.articulation_type}, axis={slide.axis}, limits={slide.motion_limits}",
    )
    ctx.check(
        "paddle link uses a side revolute joint",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and hinge.axis == (0.0, 0.0, 1.0)
        and hinge.motion_limits is not None
        and hinge.motion_limits.upper == 1.1,
        details=f"type={hinge.articulation_type}, axis={hinge.axis}, limits={hinge.motion_limits}",
    )

    ctx.expect_contact(
        shuttle,
        guide,
        elem_a="wear_pad_0",
        elem_b="rail_0",
        contact_tol=0.0005,
        name="upper wear pad rides on its rail",
    )
    ctx.expect_contact(
        shuttle,
        guide,
        elem_a="wear_pad_1",
        elem_b="rail_1",
        contact_tol=0.0005,
        name="lower wear pad rides on its rail",
    )
    ctx.expect_gap(
        shuttle,
        guide,
        axis="x",
        positive_elem="carriage_plate",
        negative_elem="end_stop_0",
        min_gap=0.050,
        name="collapsed shuttle clears the rear stop",
    )
    ctx.expect_gap(
        link,
        shuttle,
        axis="z",
        positive_elem="arm_bar",
        negative_elem="pivot_block",
        min_gap=0.009,
        name="link arm clears the hinge block top",
    )

    rest_pos = ctx.part_world_position(shuttle)
    with ctx.pose({slide: 0.36}):
        ctx.expect_within(
            shuttle,
            guide,
            axes="x",
            inner_elem="wear_pad_0",
            outer_elem="rail_0",
            margin=0.001,
            name="extended upper pad remains on rail",
        )
        ctx.expect_within(
            shuttle,
            guide,
            axes="x",
            inner_elem="wear_pad_1",
            outer_elem="rail_1",
            margin=0.001,
            name="extended lower pad remains on rail",
        )
        ctx.expect_gap(
            guide,
            shuttle,
            axis="x",
            positive_elem="end_stop_1",
            negative_elem="carriage_plate",
            min_gap=0.050,
            name="extended shuttle clears the forward stop",
        )
        extended_pos = ctx.part_world_position(shuttle)
    ctx.check(
        "shuttle extends along the guide axis",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.34,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    paddle_rest = ctx.part_element_world_aabb(link, elem="paddle_plate")
    with ctx.pose({hinge: 1.1}):
        ctx.expect_gap(
            link,
            shuttle,
            axis="z",
            positive_elem="paddle_plate",
            negative_elem="pivot_block",
            min_gap=0.009,
            name="swung paddle remains above the pivot block",
        )
        paddle_swung = ctx.part_element_world_aabb(link, elem="paddle_plate")
    rest_center_x = None if paddle_rest is None else (paddle_rest[0][0] + paddle_rest[1][0]) * 0.5
    swung_center_x = None if paddle_swung is None else (paddle_swung[0][0] + paddle_swung[1][0]) * 0.5
    ctx.check(
        "paddle sweeps forward around the shoulder bolt",
        rest_center_x is not None and swung_center_x is not None and swung_center_x > rest_center_x + 0.11,
        details=f"rest_aabb={paddle_rest}, swung_aabb={paddle_swung}",
    )

    with ctx.pose({slide: 0.36, hinge: 1.1}):
        ctx.expect_gap(
            link,
            guide,
            axis="z",
            min_gap=0.060,
            name="combined travel keeps paddle link above the guide and stops",
        )

    return ctx.report()


object_model = build_object_model()
