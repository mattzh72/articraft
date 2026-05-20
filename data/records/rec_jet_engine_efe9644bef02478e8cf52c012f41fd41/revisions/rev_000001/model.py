from __future__ import annotations

import math
import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    FanRotorGeometry,
    FanRotorBlade,
    FanRotorHub,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="turbofan_engine")

    housing = model.part("housing")

    # Nacelle
    nacelle = (
        cq.Workplane("XY")
        .moveTo(-0.2, 0.13)
        .lineTo(0.2, 0.13)
        .lineTo(0.2, 0.15)
        .lineTo(-0.2, 0.15)
        .close()
        .revolve(360, (0, 0, 0), (1, 0, 0))
        .edges(">X or <X").fillet(0.009)
    )

    # Core
    core = (
        cq.Workplane("YZ")
        .cylinder(0.15, 0.04)
        .translate((0.025, 0, 0))
    )

    # Exhaust cone
    exhaust_cone = (
        cq.Workplane("YZ", origin=(-0.05, 0, 0))
        .circle(0.04)
        .workplane(offset=-0.15)
        .circle(0.01)
        .loft()
    )

    # Struts
    struts = (
        cq.Workplane("YZ", origin=(0.04, 0, 0))
        .rect(0.01, 0.26)
        .extrude(0.02)
        .union(
            cq.Workplane("YZ", origin=(0.04, 0, 0))
            .rect(0.26, 0.01)
            .extrude(0.02)
        )
    )

    # Stand
    stand = (
        cq.Workplane("XY")
        .box(0.3, 0.2, 0.02)
        .translate((0, 0, -0.25))
        .union(
            cq.Workplane("XY")
            .box(0.15, 0.04, 0.10)
            .translate((0, 0, -0.19))
        )
    )

    housing_cq = nacelle.union(core).union(exhaust_cone).union(struts).union(stand)
    housing.visual(mesh_from_cadquery(housing_cq, "housing_mesh"), name="housing")

    fan = model.part("fan")
    fan_geom = FanRotorGeometry(
        outer_radius=0.128,
        hub_radius=0.04,
        blade_count=24,
        thickness=0.02,
        blade_pitch_deg=45.0,
        blade_sweep_deg=10.0,
        blade=FanRotorBlade(shape="scimitar", camber=0.1),
        hub=FanRotorHub(style="spinner", rear_collar_height=0.01),
        center=True
    )
    fan.visual(mesh_from_geometry(fan_geom, "fan_mesh"), name="fan")

    model.articulation(
        "fan_spin",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=fan,
        origin=Origin(xyz=(0.11, 0, 0), rpy=(0, math.pi/2, 0)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=10.0, velocity=100.0)
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    ctx.allow_overlap(
        "fan",
        "housing",
        reason="The fan rotor hub is mounted flush against the core cylinder, which may cause slight precision overlap.",
    )

    ctx.expect_within(
        "fan", 
        "housing", 
        axes="yz", 
        margin=0.005, 
        name="fan spins inside the nacelle"
    )

    ctx.expect_contact(
        "fan",
        "housing",
        name="fan hub is seated against the core"
    )

    return ctx.report()


object_model = build_object_model()