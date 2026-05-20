from __future__ import annotations

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="revolving_door")

    # Base frame including floor plate, central post, and canopy
    base = model.part("base")
    base.visual(
        Cylinder(radius=0.9, height=0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        name="floor_plate",
    )
    base.visual(
        Cylinder(radius=0.025, height=2.18),
        origin=Origin(xyz=(0.0, 0.0, 1.11)),
        name="central_post",
    )
    base.visual(
        Cylinder(radius=0.04, height=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        name="bottom_collar",
    )
    base.visual(
        Cylinder(radius=0.9, height=0.1),
        origin=Origin(xyz=(0.0, 0.0, 2.25)),
        name="canopy",
    )

    # Curved enclosure walls to maintain the airlock
    enclosure_cq = cq.Workplane("XY").circle(0.85).circle(0.81).extrude(2.2)
    cutter1 = (
        cq.Workplane("XY")
        .moveTo(0, 0)
        .lineTo(0.6, 1.2)
        .lineTo(-0.6, 1.2)
        .close()
        .extrude(2.5)
    )
    cutter2 = (
        cq.Workplane("XY")
        .moveTo(0, 0)
        .lineTo(0.6, -1.2)
        .lineTo(-0.6, -1.2)
        .close()
        .extrude(2.5)
    )
    enclosure_cq = enclosure_cq.cut(cutter1).cut(cutter2)
    base.visual(
        mesh_from_cadquery(enclosure_cq, "enclosure_walls"),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        name="enclosure_walls",
    )

    # Rotating wing assembly
    rotor = model.part("rotor")
    
    # Hub: hollow cylinder that sleeves over the post
    # Inner radius 0.026 (0.001 clearance over 0.025 post)
    # Outer radius 0.04
    hub_cq = cq.Workplane("XY").circle(0.04).circle(0.026).extrude(2.1)
    
    # 3 wings
    for i in range(3):
        angle = i * 120
        wing = (
            cq.Workplane("XY")
            .transformed(rotate=cq.Vector(0, 0, angle))
            .center(0.04, 0)
            .rect(0.76, 0.02, centered=(False, True))
            .extrude(2.1)
        )
        hub_cq = hub_cq.union(wing)
        
    rotor.visual(
        mesh_from_cadquery(hub_cq, "rotor_mesh"),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        name="wings",
    )

    model.articulation(
        "rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    rotor = object_model.get_part("rotor")
    
    # The rotor sleeves over the central post.
    ctx.expect_within(
        rotor,
        base,
        axes="xy",
        inner_elem="wings",
        outer_elem="floor_plate",
        margin=0.0,
        name="rotor stays within the floor plate footprint",
    )
    
    ctx.expect_contact(
        rotor,
        base,
        elem_a="wings",
        elem_b="bottom_collar",
        name="rotor rests on the bottom collar",
    )
    
    ctx.expect_gap(
        rotor,
        base,
        axis="z",
        positive_elem="wings",
        negative_elem="floor_plate",
        min_gap=0.02,
        name="rotor clears the floor plate",
    )
    
    ctx.expect_gap(
        base,
        rotor,
        axis="z",
        positive_elem="canopy",
        negative_elem="wings",
        min_gap=0.04,
        name="rotor clears the canopy",
    )
    
    with ctx.pose(rotor_spin=1.0):
        ctx.expect_contact(
            rotor,
            base,
            elem_b="enclosure_walls",
            contact_tol=0.02,
            name="rotor clears the enclosure walls but stays close to maintain airlock",
        )

    return ctx.report()


object_model = build_object_model()
