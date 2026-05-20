from __future__ import annotations

import cadquery as cq
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _center_plate_shape() -> cq.Workplane:
    length = 0.335
    root_x = 0.020
    root_half = 0.052
    tip_half = 0.030
    thickness = 0.026

    plate = (
        cq.Workplane("XY")
        .polyline(
            [
                (root_x, -root_half),
                (length, -tip_half),
                (length, tip_half),
                (root_x, root_half),
            ]
        )
        .close()
        .extrude(thickness)
        .translate((0.0, 0.0, -thickness / 2.0))
    )

    cutters = (
        cq.Workplane("XY")
        .pushPoints([(0.120, 0.0), (0.225, 0.0)])
        .circle(0.018)
        .extrude(0.080)
        .translate((0.0, 0.0, -0.040))
    )
    return plate.cut(cutters).edges("|Z").fillet(0.006)


def _tip_housing_shape() -> cq.Workplane:
    housing = (
        cq.Workplane("YZ")
        .rect(0.055, 0.050)
        .extrude(0.090)
        .edges("|X")
        .fillet(0.004)
    )
    bore = (
        cq.Workplane("YZ")
        .circle(0.0135)
        .extrude(0.120)
        .translate((-0.015, 0.0, 0.0))
    )
    return housing.cut(bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lift_slide_probe_arm")

    dark_metal = model.material("dark_burnished_steel", rgba=(0.06, 0.065, 0.07, 1.0))
    rail_metal = model.material("polished_linear_rail", rgba=(0.55, 0.58, 0.60, 1.0))
    carriage_blue = model.material("blue_anodized_carriage", rgba=(0.05, 0.18, 0.33, 1.0))
    plate_material = model.material("brushed_center_plate", rgba=(0.32, 0.34, 0.35, 1.0))
    housing_material = model.material("matte_tip_housing", rgba=(0.12, 0.13, 0.14, 1.0))
    probe_material = model.material("hardened_probe", rgba=(0.78, 0.76, 0.68, 1.0))

    mast = model.part("mast")
    mast.visual(
        Box((0.200, 0.160, 0.035)),
        origin=Origin(xyz=(0.000, 0.000, 0.0175)),
        material=dark_metal,
        name="base_foot",
    )
    mast.visual(
        Box((0.075, 0.078, 0.680)),
        origin=Origin(xyz=(0.000, 0.000, 0.365)),
        material=dark_metal,
        name="upright_mast",
    )
    mast.visual(
        Box((0.030, 0.100, 0.620)),
        origin=Origin(xyz=(0.0475, 0.000, 0.365)),
        material=rail_metal,
        name="guide_track",
    )
    mast.visual(
        Box((0.046, 0.116, 0.030)),
        origin=Origin(xyz=(0.039, 0.000, 0.670)),
        material=dark_metal,
        name="top_stop",
    )
    mast.visual(
        Box((0.046, 0.116, 0.030)),
        origin=Origin(xyz=(0.039, 0.000, 0.060)),
        material=dark_metal,
        name="bottom_stop",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.078, 0.130, 0.120)),
        origin=Origin(xyz=(0.000, 0.000, 0.000)),
        material=carriage_blue,
        name="slider_body",
    )
    carriage.visual(
        Box((0.006, 0.094, 0.092)),
        origin=Origin(xyz=(-0.041, 0.000, 0.000)),
        material=rail_metal,
        name="wear_pad",
    )
    for y, name in ((0.061, "hinge_ear_0"), (-0.061, "hinge_ear_1")):
        carriage.visual(
            Box((0.068, 0.014, 0.082)),
            origin=Origin(xyz=(0.067, y, 0.000)),
            material=carriage_blue,
            name=name,
        )
    for y, name in ((0.071, "pin_cap_0"), (-0.071, "pin_cap_1")):
        carriage.visual(
            Cylinder(radius=0.017, length=0.006),
            origin=Origin(xyz=(0.074, y, 0.000), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=rail_metal,
            name=name,
        )
    carriage.visual(
        Cylinder(radius=0.009, length=0.134),
        origin=Origin(xyz=(0.074, 0.000, 0.000), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=rail_metal,
        name="hinge_pin",
    )

    center_plate = model.part("center_plate")
    center_plate.visual(
        mesh_from_cadquery(_center_plate_shape(), "center_plate_web"),
        material=plate_material,
        name="plate_web",
    )
    center_plate.visual(
        Cylinder(radius=0.024, length=0.095),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=plate_material,
        name="hinge_barrel",
    )

    tip_housing = model.part("tip_housing")
    tip_housing.visual(
        mesh_from_cadquery(_tip_housing_shape(), "tip_housing_sleeve"),
        material=housing_material,
        name="housing_sleeve",
    )
    for y, name in ((0.031, "clamp_screw_0"), (-0.031, "clamp_screw_1")):
        tip_housing.visual(
            Cylinder(radius=0.0075, length=0.007),
            origin=Origin(xyz=(0.047, y, 0.014), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=rail_metal,
            name=name,
        )

    probe = model.part("probe")
    probe.visual(
        Cylinder(radius=0.009, length=0.180),
        origin=Origin(xyz=(0.090, 0.000, 0.000), rpy=(0.0, pi / 2.0, 0.0)),
        material=probe_material,
        name="probe_shaft",
    )
    probe.visual(
        Sphere(radius=0.011),
        origin=Origin(xyz=(0.190, 0.000, 0.000)),
        material=probe_material,
        name="probe_tip",
    )

    model.articulation(
        "mast_to_carriage",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0.106, 0.000, 0.170)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.24, lower=0.0, upper=0.260),
    )
    model.articulation(
        "carriage_to_plate",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=center_plate,
        origin=Origin(xyz=(0.074, 0.000, 0.000)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.6, lower=-0.35, upper=1.25),
    )
    model.articulation(
        "plate_to_tip_housing",
        ArticulationType.FIXED,
        parent=center_plate,
        child=tip_housing,
        origin=Origin(xyz=(0.335, 0.000, 0.000)),
    )
    model.articulation(
        "tip_housing_to_probe",
        ArticulationType.PRISMATIC,
        parent=tip_housing,
        child=probe,
        origin=Origin(xyz=(0.000, 0.000, 0.000)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.18, lower=0.0, upper=0.060),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    carriage = object_model.get_part("carriage")
    center_plate = object_model.get_part("center_plate")
    tip_housing = object_model.get_part("tip_housing")
    probe = object_model.get_part("probe")

    carriage_slide = object_model.get_articulation("mast_to_carriage")
    plate_hinge = object_model.get_articulation("carriage_to_plate")
    probe_slide = object_model.get_articulation("tip_housing_to_probe")

    ctx.allow_overlap(
        carriage,
        center_plate,
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        reason="The hinge pin is intentionally captured through the center plate barrel.",
    )
    ctx.expect_within(
        carriage,
        center_plate,
        axes="xz",
        inner_elem="hinge_pin",
        outer_elem="hinge_barrel",
        margin=0.001,
        name="hinge pin sits concentrically in barrel",
    )
    ctx.expect_overlap(
        carriage,
        center_plate,
        axes="y",
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        min_overlap=0.085,
        name="hinge pin spans the plate barrel",
    )

    ctx.expect_gap(
        carriage,
        mast,
        axis="x",
        positive_elem="wear_pad",
        negative_elem="guide_track",
        max_penetration=0.001,
        max_gap=0.002,
        name="carriage pad rides just off guide track",
    )
    ctx.expect_overlap(
        carriage,
        mast,
        axes="z",
        elem_a="slider_body",
        elem_b="guide_track",
        min_overlap=0.10,
        name="carriage remains seated on vertical guide",
    )

    ctx.expect_gap(
        tip_housing,
        center_plate,
        axis="x",
        positive_elem="housing_sleeve",
        negative_elem="plate_web",
        max_gap=0.003,
        max_penetration=0.0,
        name="tip housing mounts on the distal plate face",
    )
    ctx.expect_within(
        probe,
        tip_housing,
        axes="yz",
        inner_elem="probe_shaft",
        outer_elem="housing_sleeve",
        margin=0.0,
        name="probe shaft is centered through tip housing",
    )
    ctx.expect_overlap(
        probe,
        tip_housing,
        axes="x",
        elem_a="probe_shaft",
        elem_b="housing_sleeve",
        min_overlap=0.085,
        name="probe remains inserted at rest",
    )

    carriage_rest = ctx.part_world_position(carriage)
    with ctx.pose({carriage_slide: 0.260}):
        carriage_raised = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            carriage,
            mast,
            axes="z",
            elem_a="slider_body",
            elem_b="guide_track",
            min_overlap=0.10,
            name="raised carriage remains captured by guide",
        )
    ctx.check(
        "carriage prismatic axis raises along mast",
        carriage_rest is not None
        and carriage_raised is not None
        and carriage_raised[2] > carriage_rest[2] + 0.24,
        details=f"rest={carriage_rest}, raised={carriage_raised}",
    )

    tip_rest = ctx.part_world_position(tip_housing)
    with ctx.pose({plate_hinge: 1.0}):
        tip_lifted = ctx.part_world_position(tip_housing)
    ctx.check(
        "center plate hinge lifts distal housing",
        tip_rest is not None and tip_lifted is not None and tip_lifted[2] > tip_rest[2] + 0.25,
        details=f"rest={tip_rest}, lifted={tip_lifted}",
    )

    probe_rest = ctx.part_world_position(probe)
    with ctx.pose({probe_slide: 0.060}):
        probe_extended = ctx.part_world_position(probe)
        ctx.expect_overlap(
            probe,
            tip_housing,
            axes="x",
            elem_a="probe_shaft",
            elem_b="housing_sleeve",
            min_overlap=0.028,
            name="extended probe remains retained in sleeve",
        )
    ctx.check(
        "probe prismatic axis extends from tip housing",
        probe_rest is not None
        and probe_extended is not None
        and probe_extended[0] > probe_rest[0] + 0.055,
        details=f"rest={probe_rest}, extended={probe_extended}",
    )

    return ctx.report()


object_model = build_object_model()
