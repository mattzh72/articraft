from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelFace,
    BezelGeometry,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _vent_housing_mesh():
    """One connected galvanized curb and hollow low-wide duct body."""

    body_width = 0.92
    body_depth = 0.34
    body_height = 0.36
    body_center_z = 0.26

    outer = cq.Workplane("XY").box(body_width, body_depth, body_height).translate(
        (0.0, 0.0, body_center_z)
    )
    opening = cq.Workplane("XY").box(0.74, body_depth + 0.08, 0.22).translate(
        (0.0, 0.0, 0.255)
    )
    duct = outer.cut(opening)

    roof_flange = cq.Workplane("XY").box(1.12, 0.60, 0.055).translate(
        (0.0, 0.0, 0.0275)
    )
    raised_curb = cq.Workplane("XY").box(1.00, 0.48, 0.055).translate(
        (0.0, 0.0, 0.075)
    )

    return roof_flange.union(raised_curb).union(duct)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rooftop_vent_weather_flap")

    galvanized = model.material("weathered_galvanized", rgba=(0.55, 0.58, 0.55, 1.0))
    dark_metal = model.material("dark_hinge_metal", rgba=(0.16, 0.17, 0.16, 1.0))
    shadow = model.material("dark_outlet_shadow", rgba=(0.03, 0.035, 0.035, 1.0))

    body = model.part("housing")
    body.visual(
        mesh_from_cadquery(_vent_housing_mesh(), "low_wide_vent_housing"),
        material=galvanized,
        name="duct_shell",
    )

    outlet_frame = BezelGeometry(
        opening_size=(0.74, 0.22),
        outer_size=(0.88, 0.34),
        depth=0.035,
        opening_shape="rect",
        outer_shape="rect",
        face=BezelFace(style="flat", front_lip=0.004, fillet=0.0015),
    )
    body.visual(
        mesh_from_geometry(outlet_frame, "outlet_frame"),
        origin=Origin(xyz=(0.0, -0.185, 0.255), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="outlet_frame",
    )

    body.visual(
        Box((0.77, 0.006, 0.23)),
        origin=Origin(xyz=(0.0, 0.005, 0.255)),
        material=shadow,
        name="outlet_shadow",
    )

    for idx, x in enumerate((-0.4325, 0.4325)):
        body.visual(
            Box((0.045, 0.055, 0.075)),
            origin=Origin(xyz=(x, -0.226, 0.405)),
            material=dark_metal,
            name=f"hinge_cheek_{idx}",
        )

    flap = model.part("flap")
    flap.visual(
        Box((0.80, 0.025, 0.25)),
        origin=Origin(xyz=(0.0, -0.020, -0.140)),
        material=galvanized,
        name="flap_panel",
    )
    flap.visual(
        Cylinder(radius=0.018, length=0.82),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="hinge_tube",
    )
    flap.visual(
        Box((0.82, 0.040, 0.025)),
        origin=Origin(xyz=(0.0, -0.025, -0.275)),
        material=galvanized,
        name="bottom_drip_lip",
    )
    flap.visual(
        Box((0.82, 0.034, 0.035)),
        origin=Origin(xyz=(0.0, -0.024, -0.040)),
        material=galvanized,
        name="top_stiffener",
    )
    for idx, x in enumerate((-0.395, 0.395)):
        flap.visual(
            Box((0.026, 0.034, 0.245)),
            origin=Origin(xyz=(x, -0.030, -0.145)),
            material=galvanized,
            name=f"side_hem_{idx}",
        )

    model.articulation(
        "flap_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flap,
        origin=Origin(xyz=(0.0, -0.226, 0.405)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=0.0, upper=1.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("housing")
    flap = object_model.get_part("flap")
    hinge = object_model.get_articulation("flap_hinge")

    ctx.check(
        "single weather flap articulation",
        len(object_model.articulations) == 1 and hinge.child == flap.name,
        details=f"articulations={[joint.name for joint in object_model.articulations]}",
    )
    ctx.check(
        "flap hinge is horizontal revolute",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and abs(abs(hinge.axis[0]) - 1.0) < 1e-6
        and abs(hinge.axis[1]) < 1e-6
        and abs(hinge.axis[2]) < 1e-6,
        details=f"type={hinge.articulation_type}, axis={hinge.axis}",
    )

    ctx.expect_overlap(
        flap,
        body,
        axes="x",
        elem_a="flap_panel",
        elem_b="outlet_frame",
        min_overlap=0.74,
        name="flap spans most of framed outlet width",
    )
    ctx.expect_overlap(
        flap,
        body,
        axes="z",
        elem_a="flap_panel",
        elem_b="outlet_frame",
        min_overlap=0.24,
        name="closed flap covers the outlet height",
    )
    ctx.expect_gap(
        body,
        flap,
        axis="y",
        positive_elem="outlet_frame",
        negative_elem="flap_panel",
        min_gap=0.010,
        max_gap=0.060,
        name="closed flap sits just proud of outlet frame",
    )

    closed_panel_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")
    with ctx.pose({hinge: 1.05}):
        open_panel_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")

    ctx.check(
        "positive hinge pose lifts flap outward",
        closed_panel_aabb is not None
        and open_panel_aabb is not None
        and open_panel_aabb[0][2] > closed_panel_aabb[0][2] + 0.08
        and open_panel_aabb[0][1] < closed_panel_aabb[0][1] - 0.08,
        details=f"closed={closed_panel_aabb}, open={open_panel_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
