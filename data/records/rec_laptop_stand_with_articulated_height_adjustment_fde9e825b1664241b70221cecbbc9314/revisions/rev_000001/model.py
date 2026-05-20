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
    model = ArticulatedObject(name="desktop_laptop_stand")

    powder_coat = Material("matte_black_powder_coat", color=(0.03, 0.032, 0.035, 1.0))
    dark_aluminum = Material("dark_anodized_aluminum", color=(0.09, 0.10, 0.105, 1.0))
    bare_edge = Material("brushed_edge_metal", color=(0.62, 0.64, 0.62, 1.0))
    rubber = Material("soft_black_rubber", color=(0.005, 0.005, 0.004, 1.0))

    stand = model.part("stand")
    stand.visual(
        Box((0.42, 0.32, 0.012)),
        origin=Origin(xyz=(0.0, -0.02, 0.006)),
        material=powder_coat,
        name="base_plate",
    )
    stand.visual(
        Box((0.062, 0.036, 0.46)),
        origin=Origin(xyz=(0.0, 0.12, 0.242)),
        material=dark_aluminum,
        name="mast",
    )
    stand.visual(
        Box((0.076, 0.050, 0.020)),
        origin=Origin(xyz=(0.0, 0.12, 0.472)),
        material=dark_aluminum,
        name="mast_cap",
    )
    stand.visual(
        Box((0.13, 0.030, 0.070)),
        origin=Origin(xyz=(0.0, 0.091, 0.047)),
        material=dark_aluminum,
        name="front_mast_foot",
    )
    stand.visual(
        Box((0.14, 0.026, 0.055)),
        origin=Origin(xyz=(0.0, 0.149, 0.039)),
        material=dark_aluminum,
        name="rear_mast_foot",
    )
    for index, x in enumerate((-0.16, 0.16)):
        stand.visual(
            Box((0.070, 0.048, 0.004)),
            origin=Origin(xyz=(x, -0.145, 0.002)),
            material=rubber,
            name=f"rubber_foot_{index}",
        )
    for index, x in enumerate((-0.16, 0.16), start=2):
        stand.visual(
            Box((0.070, 0.048, 0.004)),
            origin=Origin(xyz=(x, 0.105, 0.002)),
            material=rubber,
            name=f"rubber_foot_{index}",
        )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.012, 0.056, 0.110)),
        origin=Origin(xyz=(-0.037, 0.0, 0.0)),
        material=bare_edge,
        name="sleeve_side_0",
    )
    carriage.visual(
        Box((0.012, 0.056, 0.110)),
        origin=Origin(xyz=(0.037, 0.0, 0.0)),
        material=bare_edge,
        name="sleeve_side_1",
    )
    carriage.visual(
        Box((0.086, 0.020, 0.110)),
        origin=Origin(xyz=(0.0, -0.028, 0.0)),
        material=bare_edge,
        name="sleeve_front",
    )
    carriage.visual(
        Box((0.086, 0.020, 0.110)),
        origin=Origin(xyz=(0.0, 0.028, 0.0)),
        material=bare_edge,
        name="sleeve_rear",
    )
    carriage.visual(
        Box((0.120, 0.044, 0.030)),
        origin=Origin(xyz=(0.0, -0.050, 0.042)),
        material=bare_edge,
        name="carriage_head",
    )
    carriage.visual(
        Box((0.092, 0.020, 0.038)),
        origin=Origin(xyz=(0.0, -0.065, 0.064)),
        material=bare_edge,
        name="hinge_web",
    )
    carriage.visual(
        Cylinder(radius=0.012, length=0.080),
        origin=Origin(xyz=(0.0, -0.076, 0.082), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bare_edge,
        name="hinge_barrel",
    )

    tray = model.part("tray")
    tray.visual(
        Box((0.370, 0.260, 0.012)),
        origin=Origin(xyz=(0.0, -0.145, 0.000)),
        material=dark_aluminum,
        name="tray_panel",
    )
    tray.visual(
        Box((0.360, 0.016, 0.022)),
        origin=Origin(xyz=(0.0, -0.018, 0.008)),
        material=dark_aluminum,
        name="rear_stiffener",
    )
    tray.visual(
        Cylinder(radius=0.012, length=0.050),
        origin=Origin(xyz=(-0.070, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_aluminum,
        name="hinge_barrel_0",
    )
    tray.visual(
        Cylinder(radius=0.012, length=0.050),
        origin=Origin(xyz=(0.070, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_aluminum,
        name="hinge_barrel_1",
    )
    for index, x in enumerate((-0.070, 0.070)):
        tray.visual(
            Box((0.055, 0.030, 0.010)),
            origin=Origin(xyz=(x, -0.015, -0.004)),
            material=dark_aluminum,
            name=f"hinge_leaf_{index}",
        )
    tray.visual(
        Box((0.095, 0.014, 0.044)),
        origin=Origin(xyz=(-0.100, -0.268, 0.025)),
        material=dark_aluminum,
        name="front_lip_0",
    )
    tray.visual(
        Box((0.095, 0.014, 0.044)),
        origin=Origin(xyz=(0.100, -0.268, 0.025)),
        material=dark_aluminum,
        name="front_lip_1",
    )
    tray.visual(
        Box((0.280, 0.018, 0.004)),
        origin=Origin(xyz=(0.0, -0.090, 0.0075)),
        material=rubber,
        name="rubber_strip_0",
    )
    tray.visual(
        Box((0.280, 0.018, 0.004)),
        origin=Origin(xyz=(0.0, -0.195, 0.0075)),
        material=rubber,
        name="rubber_strip_1",
    )

    model.articulation(
        "mast_slide",
        ArticulationType.PRISMATIC,
        parent=stand,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.12, 0.340)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=0.0, upper=0.080),
    )
    model.articulation(
        "tray_tilt",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=tray,
        origin=Origin(xyz=(0.0, -0.076, 0.082)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.2, lower=-0.25, upper=0.70),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    carriage = object_model.get_part("carriage")
    tray = object_model.get_part("tray")
    slide = object_model.get_articulation("mast_slide")
    tilt = object_model.get_articulation("tray_tilt")

    ctx.expect_contact(
        carriage,
        stand,
        elem_a="sleeve_side_0",
        elem_b="mast",
        contact_tol=0.0001,
        name="sliding sleeve bears on mast side",
    )
    ctx.expect_overlap(
        carriage,
        stand,
        axes="z",
        elem_a="sleeve_front",
        elem_b="mast",
        min_overlap=0.100,
        name="carriage sleeve remains wrapped around mast",
    )
    ctx.expect_gap(
        tray,
        carriage,
        axis="x",
        positive_elem="hinge_barrel_1",
        negative_elem="hinge_barrel",
        min_gap=0.004,
        max_gap=0.008,
        name="right tray knuckle is spaced beside carriage hinge",
    )
    ctx.expect_gap(
        carriage,
        tray,
        axis="x",
        positive_elem="hinge_barrel",
        negative_elem="hinge_barrel_0",
        min_gap=0.004,
        max_gap=0.008,
        name="left tray knuckle is spaced beside carriage hinge",
    )

    rest_carriage_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.080}):
        raised_carriage_pos = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            carriage,
            stand,
            axes="z",
            elem_a="sleeve_front",
            elem_b="mast",
            min_overlap=0.095,
            name="raised carriage is still captured on mast",
        )

    ctx.check(
        "carriage slides upward along mast axis",
        rest_carriage_pos is not None
        and raised_carriage_pos is not None
        and raised_carriage_pos[2] > rest_carriage_pos[2] + 0.075,
        details=f"rest={rest_carriage_pos}, raised={raised_carriage_pos}",
    )

    rest_lip_aabb = ctx.part_element_world_aabb(tray, elem="front_lip_0")
    with ctx.pose({tilt: 0.70}):
        tilted_lip_aabb = ctx.part_element_world_aabb(tray, elem="front_lip_0")
    rest_lip_top = rest_lip_aabb[1][2] if rest_lip_aabb is not None else None
    tilted_lip_top = tilted_lip_aabb[1][2] if tilted_lip_aabb is not None else None
    ctx.check(
        "positive tray tilt raises the retaining lips",
        rest_lip_top is not None and tilted_lip_top is not None and tilted_lip_top > rest_lip_top + 0.10,
        details=f"rest_top={rest_lip_top}, tilted_top={tilted_lip_top}",
    )

    return ctx.report()


object_model = build_object_model()
