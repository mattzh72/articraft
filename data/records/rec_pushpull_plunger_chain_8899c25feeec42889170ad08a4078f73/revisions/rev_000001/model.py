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
    model = ArticulatedObject(name="plunger_tab_mechanism")

    dark_cast = model.material("dark_cast_housing", rgba=(0.10, 0.11, 0.12, 1.0))
    edge_wear = model.material("worn_edge_metal", rgba=(0.36, 0.38, 0.39, 1.0))
    plunger_metal = model.material("brushed_plunger", rgba=(0.72, 0.74, 0.70, 1.0))
    rubber = model.material("black_rubber_button", rgba=(0.02, 0.02, 0.018, 1.0))
    warning_tab = model.material("yellow_tab", rgba=(0.95, 0.72, 0.10, 1.0))
    pin_metal = model.material("hinge_pin_metal", rgba=(0.65, 0.66, 0.62, 1.0))

    housing = model.part("housing")

    # A grounded, open-ended rectangular guide: four walls form a clear tunnel
    # for the sliding plunger, with a base plate and a front hinge bracket.
    housing.visual(
        Box((0.250, 0.120, 0.014)),
        origin=Origin(xyz=(0.000, 0.000, 0.007)),
        material=dark_cast,
        name="base_plate",
    )
    housing.visual(
        Box((0.210, 0.068, 0.012)),
        origin=Origin(xyz=(0.000, 0.000, 0.020)),
        material=dark_cast,
        name="guide_floor",
    )
    housing.visual(
        Box((0.210, 0.068, 0.012)),
        origin=Origin(xyz=(0.000, 0.000, 0.090)),
        material=dark_cast,
        name="guide_roof",
    )
    for i, y in enumerate((-0.040, 0.040)):
        housing.visual(
            Box((0.210, 0.016, 0.070)),
            origin=Origin(xyz=(0.000, y, 0.055)),
            material=dark_cast,
            name=f"side_wall_{i}",
        )

    # Front face built as a frame around the plunger opening rather than a
    # solid block, so the nose visibly exits the guide.
    housing.visual(
        Box((0.014, 0.092, 0.014)),
        origin=Origin(xyz=(0.097, 0.000, 0.010)),
        material=edge_wear,
        name="front_sill",
    )
    housing.visual(
        Box((0.014, 0.092, 0.018)),
        origin=Origin(xyz=(0.097, 0.000, 0.096)),
        material=edge_wear,
        name="front_brow",
    )
    for i, y in enumerate((-0.050, 0.050)):
        housing.visual(
            Box((0.014, 0.016, 0.082)),
            origin=Origin(xyz=(0.097, y, 0.060)),
            material=edge_wear,
            name=f"front_post_{i}",
        )

    # Clevis-style hinge ears placed outside the moving tab knuckle.
    for i, y in enumerate((-0.036, 0.036)):
        housing.visual(
            Box((0.034, 0.012, 0.018)),
            origin=Origin(xyz=(0.113, y, 0.097)),
            material=edge_wear,
            name=f"hinge_ear_{i}",
        )
        housing.visual(
            Cylinder(radius=0.009, length=0.012),
            origin=Origin(xyz=(0.126, -0.034 if y < 0.0 else 0.034, 0.097), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=pin_metal,
            name=f"fixed_knuckle_{i}",
        )

    for i, y in enumerate((-0.042, 0.042)):
        for j, x in enumerate((-0.085, 0.065)):
            housing.visual(
                Cylinder(radius=0.008, length=0.004),
                origin=Origin(xyz=(x, y, 0.015)),
                material=edge_wear,
                name=f"bolt_head_{i}_{j}",
            )

    plunger = model.part("plunger")
    plunger.visual(
        Box((0.155, 0.064, 0.058)),
        origin=Origin(xyz=(-0.010, 0.000, 0.000)),
        material=plunger_metal,
        name="slider_bar",
    )
    plunger.visual(
        Cylinder(radius=0.013, length=0.050),
        origin=Origin(xyz=(0.092, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=plunger_metal,
        name="nose",
    )
    plunger.visual(
        Cylinder(radius=0.015, length=0.036),
        origin=Origin(xyz=(-0.105, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=plunger_metal,
        name="rear_stem",
    )
    plunger.visual(
        Cylinder(radius=0.026, length=0.010),
        origin=Origin(xyz=(-0.128, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="thumb_button",
    )

    tab = model.part("tab")
    tab.visual(
        Cylinder(radius=0.008, length=0.056),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=pin_metal,
        name="tab_knuckle",
    )
    tab.visual(
        Box((0.010, 0.045, 0.080)),
        origin=Origin(xyz=(0.007, 0.000, -0.045)),
        material=warning_tab,
        name="tab_plate",
    )
    tab.visual(
        Box((0.004, 0.026, 0.018)),
        origin=Origin(xyz=(0.000, 0.000, -0.045)),
        material=edge_wear,
        name="strike_pad",
    )
    tab.visual(
        Box((0.006, 0.040, 0.010)),
        origin=Origin(xyz=(0.012, 0.000, -0.082)),
        material=warning_tab,
        name="stiffened_lip",
    )

    model.articulation(
        "housing_to_plunger",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=plunger,
        origin=Origin(xyz=(0.000, 0.000, 0.055)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.35, lower=0.0, upper=0.025),
    )
    model.articulation(
        "housing_to_tab",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=tab,
        origin=Origin(xyz=(0.126, 0.000, 0.097)),
        # The tab hangs down in local -Z; a negative Y axis makes positive
        # rotation swing its free edge outward from the front face.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=0.0, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    plunger = object_model.get_part("plunger")
    tab = object_model.get_part("tab")
    slide = object_model.get_articulation("housing_to_plunger")
    hinge = object_model.get_articulation("housing_to_tab")

    ctx.check(
        "plunger uses a prismatic guide joint",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"got {slide.articulation_type}",
    )
    ctx.check(
        "tab uses a revolute hinge joint",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"got {hinge.articulation_type}",
    )

    ctx.expect_within(
        plunger,
        housing,
        axes="yz",
        inner_elem="slider_bar",
        margin=0.010,
        name="slider bar is centered in the guide tunnel",
    )
    ctx.expect_overlap(
        plunger,
        housing,
        axes="x",
        elem_a="slider_bar",
        elem_b="guide_roof",
        min_overlap=0.140,
        name="plunger remains retained in the guide at rest",
    )
    ctx.expect_gap(
        tab,
        plunger,
        axis="x",
        positive_elem="strike_pad",
        negative_elem="nose",
        min_gap=0.004,
        max_gap=0.010,
        name="nose starts just behind the hinged tab pad",
    )

    rest_pos = ctx.part_world_position(plunger)
    rest_pad_aabb = ctx.part_element_world_aabb(tab, elem="strike_pad")
    with ctx.pose({slide: 0.025, hinge: 0.55}):
        ctx.expect_overlap(
            plunger,
            housing,
            axes="x",
            elem_a="slider_bar",
            elem_b="guide_roof",
            min_overlap=0.130,
            name="extended plunger is still captured by the guide",
        )
        ctx.expect_gap(
            plunger,
            housing,
            axis="z",
            positive_elem="slider_bar",
            negative_elem="front_sill",
            min_gap=0.006,
            name="extended plunger clears the front face opening",
        )
        ctx.expect_gap(
            tab,
            plunger,
            axis="x",
            positive_elem="strike_pad",
            negative_elem="nose",
            max_gap=0.006,
            max_penetration=0.006,
            name="extended nose meets the swung tab pad",
        )
        extended_pos = ctx.part_world_position(plunger)
        open_pad_aabb = ctx.part_element_world_aabb(tab, elem="strike_pad")

    ctx.check(
        "positive prismatic travel moves the plunger toward the tab",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.020,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )
    ctx.check(
        "positive tab rotation swings the pad outward",
        rest_pad_aabb is not None
        and open_pad_aabb is not None
        and open_pad_aabb[1][0] > rest_pad_aabb[1][0] + 0.015,
        details=f"rest_pad={rest_pad_aabb}, open_pad={open_pad_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
