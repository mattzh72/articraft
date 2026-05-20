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
    model = ArticulatedObject(name="vertical_carriage_tilting_nose")

    dark_cast = model.material("dark_cast_metal", rgba=(0.11, 0.12, 0.13, 1.0))
    machined = model.material("brushed_steel", rgba=(0.62, 0.66, 0.68, 1.0))
    carriage_blue = model.material("blue_carriage", rgba=(0.05, 0.18, 0.42, 1.0))
    nose_orange = model.material("orange_nose_link", rgba=(0.95, 0.38, 0.08, 1.0))
    rubber = model.material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    fastener = model.material("dark_fasteners", rgba=(0.025, 0.028, 0.03, 1.0))

    guide = model.part("upright_guide")
    guide.visual(
        Box((0.34, 0.28, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=dark_cast,
        name="ground_base",
    )
    guide.visual(
        Box((0.10, 0.22, 0.70)),
        origin=Origin(xyz=(0.0, 0.0, 0.39)),
        material=dark_cast,
        name="backbone",
    )
    guide.visual(
        Box((0.13, 0.25, 0.045)),
        origin=Origin(xyz=(0.018, 0.0, 0.075)),
        material=dark_cast,
        name="lower_stop",
    )
    guide.visual(
        Box((0.13, 0.25, 0.045)),
        origin=Origin(xyz=(0.018, 0.0, 0.705)),
        material=dark_cast,
        name="upper_stop",
    )
    guide.visual(
        Cylinder(radius=0.013, length=0.62),
        origin=Origin(xyz=(0.043, -0.075, 0.39)),
        material=machined,
        name="guide_rod_0",
    )
    guide.visual(
        Cylinder(radius=0.013, length=0.62),
        origin=Origin(xyz=(0.043, 0.075, 0.39)),
        material=machined,
        name="guide_rod_1",
    )
    guide.visual(
        Box((0.008, 0.055, 0.58)),
        origin=Origin(xyz=(0.052, 0.0, 0.39)),
        material=machined,
        name="center_way",
    )

    carriage = model.part("carriage_block")
    carriage.visual(
        Box((0.060, 0.200, 0.150)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=carriage_blue,
        name="front_block",
    )
    carriage.visual(
        Box((0.025, 0.050, 0.150)),
        origin=Origin(xyz=(-0.033, -0.075, 0.0)),
        material=carriage_blue,
        name="slide_shoe_0",
    )
    carriage.visual(
        Box((0.025, 0.050, 0.150)),
        origin=Origin(xyz=(-0.033, 0.075, 0.0)),
        material=carriage_blue,
        name="slide_shoe_1",
    )
    carriage.visual(
        Box((0.023, 0.070, 0.150)),
        origin=Origin(xyz=(-0.025, 0.0, 0.0)),
        material=carriage_blue,
        name="center_slider_pad",
    )
    carriage.visual(
        Cylinder(radius=0.018, length=0.106),
        origin=Origin(xyz=(0.046, 0.0, 0.015), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=machined,
        name="hinge_barrel",
    )
    for y in (-0.064, 0.064):
        for z in (-0.040, 0.050):
            carriage.visual(
                Cylinder(radius=0.008, length=0.008),
                origin=Origin(xyz=(0.034, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=fastener,
                name=f"face_bolt_{'n' if y < 0 else 'p'}_{'l' if z < 0 else 'u'}",
            )

    nose = model.part("nose_link")
    nose.visual(
        Box((0.165, 0.118, 0.044)),
        origin=Origin(xyz=(0.108, 0.0, 0.0)),
        material=nose_orange,
        name="nose_body",
    )
    for y, name in ((-0.068, "hinge_lug_0"), (0.068, "hinge_lug_1")):
        nose.visual(
            Box((0.052, 0.030, 0.050)),
            origin=Origin(xyz=(0.026, y, 0.0)),
            material=nose_orange,
            name=name,
        )
        nose.visual(
            Cylinder(radius=0.015, length=0.030),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=machined,
            name=f"side_knuckle_{0 if y < 0 else 1}",
        )
    nose.visual(
        Cylinder(radius=0.024, length=0.105),
        origin=Origin(xyz=(0.190, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="rounded_tip",
    )

    model.articulation(
        "guide_to_carriage",
        ArticulationType.PRISMATIC,
        parent=guide,
        child=carriage,
        origin=Origin(xyz=(0.1015, 0.0, 0.280)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.25, lower=0.0, upper=0.300),
    )
    model.articulation(
        "carriage_to_nose",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=nose,
        origin=Origin(xyz=(0.046, 0.0, 0.015)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=-0.60, upper=0.90),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    guide = object_model.get_part("upright_guide")
    carriage = object_model.get_part("carriage_block")
    nose = object_model.get_part("nose_link")
    slide = object_model.get_articulation("guide_to_carriage")
    tilt = object_model.get_articulation("carriage_to_nose")

    ctx.expect_gap(
        carriage,
        guide,
        axis="x",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem="slide_shoe_0",
        negative_elem="guide_rod_0",
        name="carriage shoe bears on first guide rod",
    )
    ctx.expect_contact(
        carriage,
        guide,
        elem_a="slide_shoe_1",
        elem_b="guide_rod_1",
        contact_tol=0.001,
        name="second carriage shoe bears on second guide rod",
    )
    ctx.expect_overlap(
        carriage,
        guide,
        axes="yz",
        min_overlap=0.080,
        name="carriage is visibly captured on upright guide",
    )
    ctx.expect_gap(
        nose,
        carriage,
        axis="x",
        min_gap=0.002,
        max_gap=0.020,
        positive_elem="nose_body",
        negative_elem="hinge_barrel",
        name="nose body clears the hinge barrel",
    )

    rest_carriage_pos = ctx.part_world_position(carriage)
    rest_nose_pos = ctx.part_world_position(nose)
    with ctx.pose({slide: 0.300}):
        raised_carriage_pos = ctx.part_world_position(carriage)
        raised_nose_pos = ctx.part_world_position(nose)
        ctx.expect_overlap(
            carriage,
            guide,
            axes="z",
            min_overlap=0.060,
            name="raised carriage remains engaged with guide",
        )
        ctx.expect_gap(
            carriage,
            guide,
            axis="x",
            min_gap=0.0,
            max_gap=0.001,
            positive_elem="slide_shoe_0",
            negative_elem="guide_rod_0",
            name="raised carriage keeps bearing contact",
        )

    ctx.check(
        "carriage prismatic joint raises block",
        rest_carriage_pos is not None
        and raised_carriage_pos is not None
        and raised_carriage_pos[2] > rest_carriage_pos[2] + 0.25,
        details=f"rest={rest_carriage_pos}, raised={raised_carriage_pos}",
    )
    ctx.check(
        "nose is carried upward by carriage",
        rest_nose_pos is not None
        and raised_nose_pos is not None
        and raised_nose_pos[2] > rest_nose_pos[2] + 0.25,
        details=f"rest={rest_nose_pos}, raised={raised_nose_pos}",
    )

    rest_nose_aabb = ctx.part_world_aabb(nose)
    with ctx.pose({tilt: 0.70}):
        tilted_nose_aabb = ctx.part_world_aabb(nose)
    ctx.check(
        "positive nose rotation tilts tip upward",
        rest_nose_aabb is not None
        and tilted_nose_aabb is not None
        and tilted_nose_aabb[1][2] > rest_nose_aabb[1][2] + 0.07,
        details=f"rest_aabb={rest_nose_aabb}, tilted_aabb={tilted_nose_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
