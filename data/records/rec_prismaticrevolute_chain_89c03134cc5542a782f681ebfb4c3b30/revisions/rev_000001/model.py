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
    model = ArticulatedObject(name="slider_tab_mechanism")

    satin_steel = model.material("satin_steel", rgba=(0.58, 0.60, 0.60, 1.0))
    dark_rail = model.material("dark_anodized_rail", rgba=(0.08, 0.09, 0.10, 1.0))
    carriage_blue = model.material("blue_carriage", rgba=(0.05, 0.24, 0.72, 1.0))
    tab_yellow = model.material("yellow_tab", rgba=(0.95, 0.68, 0.10, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.015, 0.015, 0.014, 1.0))

    frame = model.part("slide_frame")
    frame.visual(
        Box((0.70, 0.24, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=satin_steel,
        name="base_plate",
    )
    for y, name in ((0.085, "guide_rail_0"), (-0.085, "guide_rail_1")):
        frame.visual(
            Box((0.64, 0.025, 0.055)),
            origin=Origin(xyz=(0.0, y, 0.0625)),
            material=dark_rail,
            name=name,
        )
    frame.visual(
        Box((0.035, 0.24, 0.070)),
        origin=Origin(xyz=(-0.3325, 0.0, 0.070)),
        material=dark_rail,
        name="rear_stop",
    )
    frame.visual(
        Box((0.025, 0.24, 0.045)),
        origin=Origin(xyz=(0.3375, 0.0, 0.0575)),
        material=dark_rail,
        name="front_lip",
    )
    # Low black bearing strips make the central slot read as a real sliding way.
    for y, name in ((0.048, "wear_strip_0"), (-0.048, "wear_strip_1")):
        frame.visual(
            Box((0.57, 0.010, 0.004)),
            origin=Origin(xyz=(0.015, y, 0.037)),
            material=rubber_black,
            name=name,
        )
    for x in (-0.27, 0.27):
        for y in (-0.09, 0.09):
            frame.visual(
                Cylinder(radius=0.010, length=0.006),
                origin=Origin(xyz=(x, y, 0.038)),
                material=dark_rail,
                name=f"bolt_{x:+.2f}_{y:+.2f}",
            )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.280, 0.120, 0.040)),
        origin=Origin(xyz=(-0.140, 0.0, -0.033)),
        material=carriage_blue,
        name="carriage_bar",
    )
    carriage.visual(
        Box((0.230, 0.090, 0.008)),
        origin=Origin(xyz=(-0.150, 0.0, -0.049)),
        material=rubber_black,
        name="lower_glide",
    )
    for y, name in ((0.051, "hinge_knuckle_0"), (-0.051, "hinge_knuckle_1")):
        carriage.visual(
            Cylinder(radius=0.014, length=0.040),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=carriage_blue,
            name=name,
        )

    tab = model.part("tab")
    tab.visual(
        Cylinder(radius=0.011, length=0.062),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=tab_yellow,
        name="tab_barrel",
    )
    tab.visual(
        Box((0.112, 0.060, 0.014)),
        origin=Origin(xyz=(0.060, 0.0, -0.0165)),
        material=tab_yellow,
        name="tab_plate",
    )
    tab.visual(
        Box((0.050, 0.052, 0.004)),
        origin=Origin(xyz=(0.076, 0.0, -0.0075)),
        material=rubber_black,
        name="grip_pad",
    )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.080, 0.0, 0.088)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.35, lower=0.0, upper=0.220),
    )
    model.articulation(
        "carriage_to_tab",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=tab,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=1.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("slide_frame")
    carriage = object_model.get_part("carriage")
    tab = object_model.get_part("tab")
    slide = object_model.get_articulation("frame_to_carriage")
    hinge = object_model.get_articulation("carriage_to_tab")

    ctx.check(
        "mechanism has prismatic carriage and revolute tab",
        slide.articulation_type == ArticulationType.PRISMATIC
        and hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"slide={slide.articulation_type}, hinge={hinge.articulation_type}",
    )

    with ctx.pose({slide: 0.0, hinge: 0.0}):
        ctx.expect_gap(
            carriage,
            frame,
            axis="z",
            positive_elem="lower_glide",
            negative_elem="base_plate",
            max_gap=0.001,
            max_penetration=0.00001,
            name="carriage glide sits on the grounded frame",
        )
        ctx.expect_overlap(
            carriage,
            frame,
            axes="x",
            elem_a="carriage_bar",
            elem_b="base_plate",
            min_overlap=0.20,
            name="retracted carriage remains supported along the slide",
        )
        closed_tab_aabb = ctx.part_element_world_aabb(tab, elem="tab_plate")
        closed_tab_max_z = closed_tab_aabb[1][2] if closed_tab_aabb else None
        rest_pos = ctx.part_world_position(carriage)

    with ctx.pose({slide: 0.220, hinge: 0.0}):
        ctx.expect_gap(
            carriage,
            frame,
            axis="z",
            positive_elem="lower_glide",
            negative_elem="base_plate",
            max_gap=0.001,
            max_penetration=0.00001,
            name="extended carriage still rides on the frame",
        )
        ctx.expect_overlap(
            carriage,
            frame,
            axes="x",
            elem_a="carriage_bar",
            elem_b="base_plate",
            min_overlap=0.20,
            name="extended carriage retains insertion in the frame",
        )
        extended_pos = ctx.part_world_position(carriage)

    ctx.check(
        "prismatic joint moves the carriage forward",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.20,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    with ctx.pose({slide: 0.220, hinge: 1.35}):
        raised_tab_aabb = ctx.part_element_world_aabb(tab, elem="tab_plate")
        raised_tab_max_z = raised_tab_aabb[1][2] if raised_tab_aabb else None

    ctx.check(
        "revolute joint flips the carried tab upward",
        closed_tab_max_z is not None
        and raised_tab_max_z is not None
        and raised_tab_max_z > closed_tab_max_z + 0.055,
        details=f"closed_z={closed_tab_max_z}, raised_z={raised_tab_max_z}",
    )

    return ctx.report()


object_model = build_object_model()
