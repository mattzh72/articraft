from __future__ import annotations

import math

import cadquery as cq
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
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="large_format_engineering_scanner")

    warm_gray = Material("warm_gray", rgba=(0.72, 0.74, 0.72, 1.0))
    dark_gray = Material("dark_gray", rgba=(0.14, 0.15, 0.16, 1.0))
    black = Material("black_rubber", rgba=(0.01, 0.011, 0.012, 1.0))
    glass_blue = Material("blue_tinted_glass", rgba=(0.30, 0.55, 0.68, 0.45))
    white = Material("paper_white", rgba=(0.96, 0.96, 0.92, 1.0))
    metal = Material("brushed_metal", rgba=(0.68, 0.69, 0.66, 1.0))
    shadow = Material("shadow_black", rgba=(0.02, 0.022, 0.025, 1.0))

    # Root frame: X is the scan width, +Y is the rear hinge side, +Z is up.
    base = model.part("base")

    body_shell = (
        cq.Workplane("XY")
        .box(1.55, 0.72, 0.12)
        .edges("|Z")
        .fillet(0.035)
        .translate((0.0, 0.0, 0.060))
    )
    base.visual(
        mesh_from_cadquery(body_shell, "body_shell", tolerance=0.002),
        material=warm_gray,
        name="body_shell",
    )

    base.visual(
        Box((1.34, 0.40, 0.008)),
        origin=Origin(xyz=(0.0, 0.105, 0.124)),
        material=glass_blue,
        name="platen_glass",
    )
    base.visual(
        Box((1.18, 0.50, 0.002)),
        origin=Origin(xyz=(0.0, -0.005, 0.129)),
        material=white,
        name="media_sheet",
    )
    base.visual(
        Box((1.38, 0.026, 0.004)),
        origin=Origin(xyz=(0.0, 0.320, 0.126)),
        material=white,
        name="calibration_strip",
    )
    base.visual(
        Box((1.28, 0.008, 0.026)),
        origin=Origin(xyz=(0.0, -0.362, 0.070)),
        material=shadow,
        name="front_feed_slot",
    )
    base.visual(
        Box((1.18, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, -0.342, 0.124)),
        material=metal,
        name="front_hinge_bracket",
    )
    base.visual(
        Cylinder(radius=0.006, length=1.42),
        origin=Origin(xyz=(0.0, -0.350, 0.135), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="front_hinge_pin",
    )
    base.visual(
        Box((1.45, 0.030, 0.004)),
        origin=Origin(xyz=(0.0, 0.345, 0.122)),
        material=metal,
        name="rear_hinge_leaf",
    )
    base.visual(
        Box((1.45, 0.006, 0.020)),
        origin=Origin(xyz=(0.0, 0.359, 0.133)),
        material=metal,
        name="rear_hinge_curl",
    )
    base.visual(
        Cylinder(radius=0.008, length=1.46),
        origin=Origin(xyz=(0.0, 0.368, 0.145), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="rear_hinge_pin",
    )
    for i, x in enumerate((-0.64, 0.64)):
        for j, y in enumerate((-0.28, 0.28)):
            base.visual(
                Box((0.16, 0.10, 0.018)),
                origin=Origin(xyz=(x, y, -0.009)),
                material=dark_gray,
                name=f"foot_{i}_{j}",
            )

    lid = model.part("lid")
    lid_shell = (
        cq.Workplane("XY")
        .box(1.53, 0.400, 0.044)
        .edges("|Z")
        .fillet(0.028)
        .translate((0.0, -0.210, 0.027))
    )
    lid.visual(
        mesh_from_cadquery(lid_shell, "lid_shell", tolerance=0.002),
        material=dark_gray,
        name="lid_shell",
    )
    lid.visual(
        Box((1.31, 0.30, 0.006)),
        origin=Origin(xyz=(0.0, -0.205, 0.004)),
        material=white,
        name="lid_backing",
    )
    lid.visual(
        Box((1.42, 0.030, 0.004)),
        origin=Origin(xyz=(0.0, -0.050, 0.008)),
        material=metal,
        name="lid_hinge_leaf",
    )
    for i, x in enumerate((-0.735, 0.735)):
        lid.visual(
            Box((0.020, 0.330, 0.030)),
            origin=Origin(xyz=(x, -0.215, -0.010)),
            material=black,
            name=f"side_gasket_{i}",
        )

    roller_housing = model.part("roller_housing")
    roller_housing.visual(
        Box((1.44, 0.270, 0.032)),
        origin=Origin(xyz=(0.0, 0.145, 0.062)),
        material=dark_gray,
        name="top_cowl",
    )
    roller_housing.visual(
        Box((1.44, 0.022, 0.056)),
        origin=Origin(xyz=(0.0, 0.020, 0.028)),
        material=dark_gray,
        name="front_skirt",
    )
    roller_housing.visual(
        Box((1.44, 0.018, 0.040)),
        origin=Origin(xyz=(0.0, 0.272, 0.044)),
        material=dark_gray,
        name="rear_lip",
    )
    for i, x in enumerate((-0.705, 0.705)):
        roller_housing.visual(
            Box((0.035, 0.270, 0.064)),
            origin=Origin(xyz=(x, 0.145, 0.034)),
            material=dark_gray,
            name=f"end_cheek_{i}",
        )
    roller_housing.visual(
        Box((1.36, 0.018, 0.006)),
        origin=Origin(xyz=(0.0, 0.020, 0.004)),
        material=metal,
        name="front_hinge_leaf",
    )
    for i, x in enumerate((-0.695, 0.695)):
        roller_housing.visual(
            Box((0.045, 0.050, 0.040)),
            origin=Origin(xyz=(x, 0.240, 0.005)),
            material=black,
            name=f"rest_pad_{i}",
        )

    feed_roller = model.part("feed_roller")
    feed_roller.visual(
        Cylinder(radius=0.024, length=1.26),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="rubber_sleeve",
    )
    feed_roller.visual(
        Cylinder(radius=0.006, length=1.36),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="steel_shaft",
    )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, 0.360, 0.145)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=1.25),
    )
    model.articulation(
        "base_to_roller_housing",
        ArticulationType.REVOLUTE,
        parent=base,
        child=roller_housing,
        origin=Origin(xyz=(0.0, -0.345, 0.135)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.5, lower=0.0, upper=1.15),
    )
    model.articulation(
        "roller_housing_to_feed_roller",
        ArticulationType.CONTINUOUS,
        parent=roller_housing,
        child=feed_roller,
        origin=Origin(xyz=(0.0, 0.155, 0.024)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    roller_housing = object_model.get_part("roller_housing")
    feed_roller = object_model.get_part("feed_roller")
    lid_hinge = object_model.get_articulation("base_to_lid")
    housing_hinge = object_model.get_articulation("base_to_roller_housing")
    roller_spin = object_model.get_articulation("roller_housing_to_feed_roller")

    ctx.check(
        "feed roller has continuous spin",
        roller_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={roller_spin.articulation_type}",
    )
    ctx.check(
        "lid opens through service angle",
        lid_hinge.motion_limits is not None and lid_hinge.motion_limits.upper >= 1.1,
        details=f"limits={lid_hinge.motion_limits}",
    )
    ctx.check(
        "roller housing opens for loading",
        housing_hinge.motion_limits is not None and housing_hinge.motion_limits.upper >= 1.0,
        details=f"limits={housing_hinge.motion_limits}",
    )

    ctx.expect_overlap(
        lid,
        base,
        axes="xy",
        elem_a="lid_shell",
        elem_b="platen_glass",
        min_overlap=0.30,
        name="lid spans the platen footprint",
    )
    ctx.expect_gap(
        lid,
        base,
        axis="z",
        positive_elem="lid_shell",
        negative_elem="platen_glass",
        min_gap=0.010,
        max_gap=0.030,
        name="closed lid clears the glass platen",
    )
    ctx.expect_gap(
        feed_roller,
        base,
        axis="z",
        positive_elem="rubber_sleeve",
        negative_elem="media_sheet",
        min_gap=0.0,
        max_gap=0.008,
        name="feed roller sits just above media",
    )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    closed_housing_aabb = ctx.part_world_aabb(roller_housing)
    with ctx.pose({lid_hinge: 1.0}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({housing_hinge: 1.0}):
        open_housing_aabb = ctx.part_world_aabb(roller_housing)

    ctx.check(
        "rear piano hinge lifts the lid upward",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.12,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )
    ctx.check(
        "front hinge flips roller housing upward",
        closed_housing_aabb is not None
        and open_housing_aabb is not None
        and open_housing_aabb[1][2] > closed_housing_aabb[1][2] + 0.12,
        details=f"closed={closed_housing_aabb}, open={open_housing_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
