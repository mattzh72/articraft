from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="compact_service_slide_hinge")

    dark_steel = model.material("dark_steel", color=(0.10, 0.11, 0.12, 1.0))
    satin_aluminum = model.material("satin_aluminum", color=(0.68, 0.70, 0.72, 1.0))
    black_polymer = model.material("black_polymer", color=(0.015, 0.016, 0.018, 1.0))
    carriage_blue = model.material("blue_carriage", color=(0.12, 0.26, 0.45, 1.0))
    brass_pin = model.material("brass_pin", color=(0.82, 0.62, 0.25, 1.0))
    warning_tab = model.material("warning_tab", color=(0.95, 0.58, 0.12, 1.0))
    rubber = model.material("rubber", color=(0.02, 0.02, 0.018, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.240, 0.112, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=dark_steel,
        name="base_plate",
    )
    body.visual(
        Box((0.202, 0.008, 0.018)),
        origin=Origin(xyz=(0.0, -0.019, 0.021)),
        material=satin_aluminum,
        name="guide_rail_0",
    )
    body.visual(
        Box((0.202, 0.008, 0.018)),
        origin=Origin(xyz=(0.0, 0.019, 0.021)),
        material=satin_aluminum,
        name="guide_rail_1",
    )
    body.visual(
        Box((0.212, 0.010, 0.024)),
        origin=Origin(xyz=(0.0, -0.048, 0.024)),
        material=dark_steel,
        name="side_wall_0",
    )
    body.visual(
        Box((0.212, 0.010, 0.024)),
        origin=Origin(xyz=(0.0, 0.048, 0.024)),
        material=dark_steel,
        name="side_wall_1",
    )
    body.visual(
        Box((0.190, 0.028, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0135)),
        material=black_polymer,
        name="slide_bed",
    )
    body.visual(
        Box((0.012, 0.075, 0.025)),
        origin=Origin(xyz=(-0.106, 0.0, 0.0245)),
        material=dark_steel,
        name="end_stop_0",
    )
    body.visual(
        Box((0.012, 0.075, 0.025)),
        origin=Origin(xyz=(0.106, 0.0, 0.0245)),
        material=dark_steel,
        name="end_stop_1",
    )
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            body.visual(
                Cylinder(radius=0.009, length=0.004),
                origin=Origin(xyz=(sx * 0.086, sy * 0.038, 0.014), rpy=(0.0, 0.0, 0.0)),
                material=black_polymer,
                name=f"screw_recess_{0 if sx < 0 else 1}_{0 if sy < 0 else 1}",
            )
            body.visual(
                Cylinder(radius=0.006, length=0.006),
                origin=Origin(xyz=(sx * 0.086, sy * 0.038, 0.003), rpy=(0.0, 0.0, 0.0)),
                material=rubber,
                name=f"foot_{0 if sx < 0 else 1}_{0 if sy < 0 else 1}",
            )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.066, 0.052, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=carriage_blue,
        name="slider_block",
    )
    carriage.visual(
        Box((0.055, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, -0.019, -0.002)),
        material=black_polymer,
        name="wear_pad_0",
    )
    carriage.visual(
        Box((0.055, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, 0.019, -0.002)),
        material=black_polymer,
        name="wear_pad_1",
    )
    carriage.visual(
        Box((0.014, 0.012, 0.011)),
        origin=Origin(xyz=(0.030, -0.018, 0.0195)),
        material=carriage_blue,
        name="hinge_boss_0",
    )
    carriage.visual(
        Box((0.014, 0.012, 0.011)),
        origin=Origin(xyz=(0.030, 0.018, 0.0195)),
        material=carriage_blue,
        name="hinge_boss_1",
    )
    carriage.visual(
        Cylinder(radius=0.005, length=0.016),
        origin=Origin(xyz=(0.034, -0.018, 0.019), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brass_pin,
        name="outer_knuckle_0",
    )
    carriage.visual(
        Cylinder(radius=0.005, length=0.016),
        origin=Origin(xyz=(0.034, 0.018, 0.019), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brass_pin,
        name="outer_knuckle_1",
    )

    tab = model.part("tab")
    tab.visual(
        Cylinder(radius=0.0045, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brass_pin,
        name="center_knuckle",
    )
    tab.visual(
        Box((0.010, 0.019, 0.052)),
        origin=Origin(xyz=(0.008, 0.0, 0.025)),
        material=warning_tab,
        name="upright_leaf",
    )
    tab.visual(
        Box((0.004, 0.017, 0.018)),
        origin=Origin(xyz=(0.014, 0.0, 0.044)),
        material=black_polymer,
        name="grip_pad",
    )

    slide = model.articulation(
        "body_to_carriage",
        ArticulationType.PRISMATIC,
        parent=body,
        child=carriage,
        origin=Origin(xyz=(-0.045, 0.0, 0.030)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.35, lower=0.0, upper=0.090),
    )
    slide.meta["description"] = "Short service carriage translating along the body rails."

    hinge = model.articulation(
        "carriage_to_tab",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=tab,
        origin=Origin(xyz=(0.034, 0.0, 0.019)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=0.0, upper=1.25),
    )
    hinge.meta["description"] = "Small service tab flips outward around the carried hinge barrel."

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    carriage = object_model.get_part("carriage")
    tab = object_model.get_part("tab")
    slide = object_model.get_articulation("body_to_carriage")
    hinge = object_model.get_articulation("carriage_to_tab")

    ctx.expect_contact(
        carriage,
        body,
        elem_a="wear_pad_0",
        elem_b="guide_rail_0",
        contact_tol=0.0002,
        name="carriage pad rides on first rail",
    )
    ctx.expect_contact(
        carriage,
        body,
        elem_a="wear_pad_1",
        elem_b="guide_rail_1",
        contact_tol=0.0002,
        name="carriage pad rides on second rail",
    )
    ctx.expect_gap(
        body,
        carriage,
        axis="y",
        positive_elem="side_wall_1",
        negative_elem="slider_block",
        min_gap=0.014,
        max_gap=0.025,
        name="carriage clears positive guide wall",
    )
    ctx.expect_gap(
        carriage,
        body,
        axis="y",
        positive_elem="slider_block",
        negative_elem="side_wall_0",
        min_gap=0.014,
        max_gap=0.025,
        name="carriage clears negative guide wall",
    )

    rest_carriage = ctx.part_world_position(carriage)
    rest_tab = ctx.part_world_position(tab)
    with ctx.pose({slide: 0.090}):
        ctx.expect_contact(
            carriage,
            body,
            elem_a="wear_pad_0",
            elem_b="guide_rail_0",
            contact_tol=0.0002,
            name="extended carriage remains on first rail",
        )
        ctx.expect_contact(
            carriage,
            body,
            elem_a="wear_pad_1",
            elem_b="guide_rail_1",
            contact_tol=0.0002,
            name="extended carriage remains on second rail",
        )
        extended_carriage = ctx.part_world_position(carriage)
        extended_tab = ctx.part_world_position(tab)
    ctx.check(
        "prismatic carriage travels along body",
        rest_carriage is not None
        and extended_carriage is not None
        and extended_carriage[0] > rest_carriage[0] + 0.085,
        details=f"rest={rest_carriage}, extended={extended_carriage}",
    )
    ctx.check(
        "tab is carried by sliding carriage",
        rest_tab is not None and extended_tab is not None and extended_tab[0] > rest_tab[0] + 0.085,
        details=f"rest={rest_tab}, extended={extended_tab}",
    )

    closed_aabb = ctx.part_world_aabb(tab)
    with ctx.pose({hinge: 1.0}):
        folded_aabb = ctx.part_world_aabb(tab)
    ctx.check(
        "revolute tab folds outward",
        closed_aabb is not None
        and folded_aabb is not None
        and folded_aabb[1][0] > closed_aabb[1][0] + 0.025,
        details=f"upright={closed_aabb}, folded={folded_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
