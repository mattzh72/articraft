from __future__ import annotations

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    Material,
    MotionLimits,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lazy_susan")

    # Base
    base = model.part("base")
    base_cq = (
        cq.Workplane("XY")
        .cylinder(height=0.01, radius=0.125, centered=(True, True, False))
        .faces(">Z")
        .workplane()
        .cylinder(height=0.02, radius=0.075, centered=(True, True, False))
    )
    base.visual(
        mesh_from_cadquery(base_cq, "base_mesh"),
        origin=Origin(),
        name="base_vis",
    )

    # Tray
    tray = model.part("tray")
    tray_cq = (
        cq.Workplane("XY")
        .cylinder(height=0.02, radius=0.15, centered=(True, True, False))
        .faces(">Z")
        .workplane()
        .hole(diameter=0.29, depth=0.015)
    )
    tray.visual(
        mesh_from_cadquery(tray_cq, "tray_mesh"),
        origin=Origin(),
        name="tray_vis",
        material=Material(name="wood", rgba=(0.8, 0.6, 0.4, 1.0)),
    )

    # Articulation
    model.articulation(
        "base_to_tray",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=tray,
        origin=Origin(xyz=(0, 0, 0.03)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base")
    tray = object_model.get_part("tray")
    
    ctx.expect_contact(tray, base, contact_tol=1e-4, name="tray rests on base")
    ctx.expect_within(base, tray, axes="xy", name="base is within tray footprint")
    
    with ctx.pose(base_to_tray=3.14):
        ctx.expect_contact(tray, base, contact_tol=1e-4, name="tray rests on base while rotated")
    
    return ctx.report()


object_model = build_object_model()
