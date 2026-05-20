from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


NOSE_X = 0.405


def _fuselage_mesh() -> object:
    """Tapered oval model-airplane fuselage, authored along local +X."""
    return mesh_from_cadquery(
        (
            cq.Workplane("YZ")
            .workplane(offset=-0.405)
            .ellipse(0.006, 0.006)
            .workplane(offset=0.055)
            .ellipse(0.030, 0.034)
            .workplane(offset=0.115)
            .ellipse(0.052, 0.055)
            .workplane(offset=0.210)
            .ellipse(0.064, 0.066)
            .workplane(offset=0.215)
            .ellipse(0.057, 0.060)
            .workplane(offset=0.125)
            .ellipse(0.030, 0.034)
            .workplane(offset=0.090)
            .ellipse(0.007, 0.007)
            .loft(combine=True, ruled=False)
        ),
        "tapered_fuselage",
        tolerance=0.0015,
        angular_tolerance=0.08,
    )


def _flat_plate_mesh(outline: list[tuple[float, float]], thickness: float, name: str) -> object:
    return mesh_from_cadquery(
        cq.Workplane("XY").polyline(outline).close().extrude(thickness, both=True),
        name,
        tolerance=0.001,
        angular_tolerance=0.08,
    )


def _vertical_tail_mesh() -> object:
    outline = [
        (-0.385, 0.020),
        (-0.355, 0.128),
        (-0.280, 0.082),
        (-0.270, 0.026),
    ]
    return mesh_from_cadquery(
        cq.Workplane("XZ").polyline(outline).close().extrude(0.014, both=True),
        "vertical_tail",
        tolerance=0.001,
        angular_tolerance=0.08,
    )


def _canopy_mesh() -> object:
    return mesh_from_cadquery(
        (
            cq.Workplane("XY")
            .ellipse(0.060, 0.030)
            .extrude(0.028)
            .translate((0.085, 0.0, 0.038))
        ),
        "canopy_bubble",
        tolerance=0.001,
        angular_tolerance=0.08,
    )


def _propeller_mesh() -> object:
    """Broad, shallow two-blade propeller with an integral hub and spinner."""
    pos_blade = [
        (0.014, -0.020),
        (0.105, -0.043),
        (0.180, -0.034),
        (0.190, 0.002),
        (0.168, 0.039),
        (0.022, 0.020),
    ]
    neg_blade = [(-y, -z) for y, z in pos_blade]
    blade_pos = cq.Workplane("YZ").polyline(pos_blade).close().extrude(0.008)
    blade_neg = cq.Workplane("YZ").polyline(neg_blade).close().extrude(0.008)
    hub = cq.Workplane("YZ").circle(0.027).extrude(0.038)
    spinner = (
        cq.Workplane("YZ")
        .circle(0.024)
        .workplane(offset=0.035)
        .circle(0.006)
        .loft(combine=True)
    )
    return mesh_from_cadquery(
        hub.union(blade_pos).union(blade_neg).union(spinner),
        "broad_propeller",
        tolerance=0.0008,
        angular_tolerance=0.06,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="small_propeller_model_airplane")

    white = model.material("warm_white_plastic", rgba=(0.93, 0.93, 0.88, 1.0))
    red = model.material("red_trim", rgba=(0.78, 0.05, 0.04, 1.0))
    dark = model.material("dark_propeller", rgba=(0.06, 0.06, 0.055, 1.0))
    metal = model.material("brushed_metal", rgba=(0.55, 0.56, 0.58, 1.0))
    canopy = model.material("smoked_blue_canopy", rgba=(0.18, 0.35, 0.55, 0.62))

    airframe = model.part("airframe")
    airframe.visual(_fuselage_mesh(), material=white, name="fuselage")
    airframe.visual(
        _flat_plate_mesh(
            [
                (-0.165, -0.455),
                (0.055, -0.455),
                (0.088, -0.405),
                (0.095, 0.405),
                (0.055, 0.455),
                (-0.165, 0.455),
                (-0.190, 0.405),
                (-0.190, -0.405),
            ],
            0.018,
            "main_wing",
        ),
        material=white,
        name="main_wing",
    )
    airframe.visual(
        _flat_plate_mesh(
            [
                (-0.390, -0.175),
                (-0.265, -0.175),
                (-0.245, -0.145),
                (-0.250, 0.145),
                (-0.270, 0.175),
                (-0.390, 0.175),
            ],
            0.012,
            "horizontal_tail",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=white,
        name="horizontal_tail",
    )
    airframe.visual(_vertical_tail_mesh(), material=red, name="vertical_tail")
    airframe.visual(_canopy_mesh(), material=canopy, name="canopy")
    airframe.visual(
        Box((0.205, 0.030, 0.040)),
        origin=Origin(xyz=(-0.040, -0.325, 0.0)),
        material=red,
        name="wing_stripe_0",
    )
    airframe.visual(
        Box((0.205, 0.030, 0.040)),
        origin=Origin(xyz=(-0.040, 0.325, 0.0)),
        material=red,
        name="wing_stripe_1",
    )
    airframe.visual(
        Box((0.150, 0.006, 0.006)),
        origin=Origin(xyz=(-0.320, 0.0, 0.090)),
        material=white,
        name="tail_rudder_line",
    )
    airframe.visual(
        Cylinder(radius=0.010, length=0.210),
        origin=Origin(xyz=(-0.025, 0.0, -0.145)),
        material=metal,
        name="display_peg",
    )
    airframe.visual(
        Cylinder(radius=0.070, length=0.014),
        origin=Origin(xyz=(-0.025, 0.0, -0.255)),
        material=metal,
        name="display_base",
    )

    propeller = model.part("propeller")
    propeller.visual(_propeller_mesh(), material=dark, name="propeller_blades")

    model.articulation(
        "nose_to_propeller",
        ArticulationType.CONTINUOUS,
        parent=airframe,
        child=propeller,
        origin=Origin(xyz=(NOSE_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.15, velocity=80.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    airframe = object_model.get_part("airframe")
    propeller = object_model.get_part("propeller")
    spin = object_model.get_articulation("nose_to_propeller")

    ctx.expect_overlap(
        airframe,
        propeller,
        axes="yz",
        min_overlap=0.015,
        elem_a="fuselage",
        elem_b="propeller_blades",
        name="propeller is centered on the nose axis",
    )
    ctx.expect_gap(
        propeller,
        airframe,
        axis="x",
        min_gap=0.0,
        max_gap=0.004,
        positive_elem="propeller_blades",
        negative_elem="fuselage",
        name="propeller rear face sits at the nose",
    )
    with ctx.pose({spin: math.pi / 2.0}):
        ctx.expect_gap(
            propeller,
            airframe,
            axis="x",
            min_gap=0.0,
            max_gap=0.004,
            positive_elem="propeller_blades",
            negative_elem="fuselage",
            name="spinning propeller remains in front of fuselage",
        )

    return ctx.report()


object_model = build_object_model()
