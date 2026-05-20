from __future__ import annotations

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="globe_stand")

    # Base (Pedestal + Fork)
    pedestal = (
        cq.Workplane("XY")
        .circle(0.1).extrude(0.02)
        .faces(">Z").workplane().circle(0.02).extrude(0.1)
    )
    fork_arc = (
        cq.Workplane("XZ", origin=(0, 0, 0.30))
        .moveTo(0.195, 0)
        .threePointArc((0, -0.195), (-0.195, 0))
        .lineTo(-0.175, 0)
        .threePointArc((0, -0.175), (0.175, 0))
        .close()
        .extrude(0.01, both=True)
    )
    hole_east = cq.Workplane("YZ", origin=(0.175, 0, 0.30)).circle(0.0055).extrude(0.03)
    hole_west = cq.Workplane("YZ", origin=(-0.175, 0, 0.30)).circle(0.0055).extrude(-0.03)
    base_cq = pedestal.union(fork_arc).cut(hole_east).cut(hole_west)

    base = model.part("base")
    base.visual(mesh_from_cadquery(base_cq, "base_mesh"), name="base_mesh")

    # Meridian
    meridian_ring = (
        cq.Workplane("XZ")
        .circle(0.165).circle(0.155)
        .extrude(0.005, both=True)
    )
    trunnion_top = cq.Workplane("XY", origin=(0, 0, 0.155)).circle(0.005).extrude(-0.01)
    trunnion_bottom = cq.Workplane("XY", origin=(0, 0, -0.155)).circle(0.005).extrude(0.01)
    trunnion_east = cq.Workplane("YZ", origin=(0.165, 0, 0)).circle(0.005).extrude(0.02)
    trunnion_west = cq.Workplane("YZ", origin=(-0.165, 0, 0)).circle(0.005).extrude(-0.02)
    meridian_cq = meridian_ring.union(trunnion_top).union(trunnion_bottom).union(trunnion_east).union(trunnion_west)

    meridian = model.part("meridian")
    meridian.visual(mesh_from_cadquery(meridian_cq, "meridian_mesh"), name="meridian_mesh")

    # Globe
    globe = model.part("globe")
    globe.visual(Sphere(0.150), name="sphere")

    # Articulations
    # Meridian tilts on horizontal revolute axis at the fork (East-West axis is X-axis)
    model.articulation(
        "meridian_tilt",
        ArticulationType.REVOLUTE,
        parent=base,
        child=meridian,
        origin=Origin(xyz=(0, 0, 0.30)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=-1.57, upper=1.57),
    )

    # Globe rotates on north-south axis (Z-axis) relative to meridian
    model.articulation(
        "globe_spin",
        ArticulationType.CONTINUOUS,
        parent=meridian,
        child=globe,
        origin=Origin(xyz=(0, 0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=5.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    ctx.allow_overlap(
        "meridian", "base",
        elem_a="meridian_mesh", elem_b="base_mesh",
        reason="meridian trunnions are captured inside the fork holes"
    )
    ctx.allow_overlap(
        "globe", "meridian",
        elem_a="sphere", elem_b="meridian_mesh",
        reason="globe is captured by the top and bottom trunnions of the meridian"
    )

    ctx.expect_within("globe", "meridian", axes=("x", "z"), margin=0.01, name="globe stays inside meridian")
    ctx.expect_within("meridian", "base", axes="x", margin=0.01, name="meridian stays between fork arms")

    return ctx.report()


object_model = build_object_model()