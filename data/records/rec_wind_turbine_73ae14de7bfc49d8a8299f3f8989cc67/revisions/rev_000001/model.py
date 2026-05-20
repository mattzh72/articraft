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

def build_hub_mesh() -> cq.Workplane:
    return cq.Workplane("YZ").workplane(offset=0.0).circle(1.5).workplane(offset=2.0).circle(0.5).loft()

def build_blade_mesh() -> cq.Workplane:
    return (cq.Workplane("XY")
            .ellipse(0.8, 0.2)
            .workplane(offset=15.0).ellipse(0.6, 0.15).rotate((0,0,0),(0,0,1),5)
            .workplane(offset=30.0).ellipse(0.3, 0.05).rotate((0,0,0),(0,0,1),15)
            .loft())

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wind_turbine")

    tower = model.part("tower")
    tower.visual(
        Cylinder(radius=1.5, length=50.0),
        origin=Origin(xyz=(0.0, 0.0, 25.0)),
        name="tower_body",
    )

    nacelle = model.part("nacelle")
    nacelle.visual(
        Box((10.0, 3.0, 3.0)),
        origin=Origin(xyz=(-2.0, 0.0, 2.0)),
        name="nacelle_body",
    )
    nacelle.visual(
        Cylinder(radius=1.5, length=0.5),
        origin=Origin(xyz=(0.0, 0.0, 0.25)),
        name="yaw_bearing",
    )

    model.articulation(
        "tower_to_nacelle",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=nacelle,
        origin=Origin(xyz=(0.0, 0.0, 50.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=100.0, velocity=1.0),
    )

    hub = model.part("hub")
    hub.visual(
        mesh_from_cadquery(build_hub_mesh(), "hub_mesh"),
        origin=Origin(),
        name="hub_body",
    )

    model.articulation(
        "nacelle_to_hub",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=hub,
        origin=Origin(xyz=(3.0, 0.0, 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=100.0, velocity=5.0),
    )

    for i in range(3):
        angle_deg = i * 120.0
        angle_rad = math.radians(angle_deg)
        # Embed the blade root slightly into the hub (hub radius is 1.0 at X=1.0)
        y = -math.sin(angle_rad) * 0.9
        z = math.cos(angle_rad) * 0.9
        
        blade = model.part(f"blade_{i}")
        blade.visual(
            mesh_from_cadquery(build_blade_mesh(), f"blade_{i}_mesh"),
            name=f"blade_{i}_shell"
        )
        model.articulation(
            f"hub_to_blade_{i}",
            ArticulationType.FIXED,
            parent=hub,
            child=blade,
            origin=Origin(xyz=(1.0, y, z), rpy=(angle_rad, 0.0, 0.0))
        )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    # Yaw bearing rests on tower
    ctx.expect_contact(
        "nacelle", "tower",
        elem_a="yaw_bearing",
        elem_b="tower_body",
        name="yaw bearing rests on tower",
    )

    # Hub rests on nacelle front
    ctx.expect_contact(
        "hub", "nacelle",
        elem_a="hub_body",
        elem_b="nacelle_body",
        name="hub rests on nacelle front",
    )

    for i in range(3):
        ctx.allow_overlap(
            "hub", f"blade_{i}",
            elem_a="hub_body",
            elem_b=f"blade_{i}_shell",
            reason="Blades are intentionally embedded into the hub for a secure mounting representation."
        )
        ctx.expect_overlap(
            "hub", f"blade_{i}",
            elem_a="hub_body",
            elem_b=f"blade_{i}_shell",
            axes="x",
            name=f"blade_{i} remains inserted in hub along shaft axis"
        )

    return ctx.report()

object_model = build_object_model()