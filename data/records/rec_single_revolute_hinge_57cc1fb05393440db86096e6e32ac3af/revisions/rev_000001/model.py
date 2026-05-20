from __future__ import annotations

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    place_on_face,
    place_on_surface,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_cabinet")

    # Cabinet Body
    cabinet = model.part("cabinet")
    cabinet_cq = (
        cq.Workplane("XY")
        .box(0.6, 0.3, 0.8)
        .faces("+Y")
        .shell(-0.02)
    )
    cabinet.visual(
        mesh_from_cadquery(cabinet_cq, "cabinet_mesh"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="cabinet_body",
        color=(0.8, 0.8, 0.8),
    )

    # Door
    door = model.part("door")
    door.visual(
        Box((0.6, 0.02, 0.8)),
        # The door origin will be at the hinge, let's say left side (-X)
        origin=Origin(xyz=(0.3, 0.01, 0.0)),
        name="door_panel",
        color=(0.9, 0.9, 0.9),
    )

    # Handle
    door.visual(
        Box((0.02, 0.03, 0.15)),
        origin=Origin(xyz=(0.55, 0.035, 0.0)),
        name="handle",
        color=(0.3, 0.3, 0.3),
    )

    # Articulation
    # Cabinet face is at Y=0.15. Hinge is at X=-0.3, Y=0.15, Z=0.0.
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(-0.3, 0.15, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=0.0, upper=2.0),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("door_hinge")

    # Closed pose: door should be flush with cabinet front
    ctx.expect_gap(door, cabinet, axis="y", positive_elem="door_panel", negative_elem="cabinet_body", min_gap=0.0, max_gap=0.001)
    
    # Door should cover the opening (overlap in XZ projection)
    ctx.expect_overlap(door, cabinet, axes="xz", elem_a="door_panel", elem_b="cabinet_body", min_overlap=0.1)

    # Open pose: door should move outward (+Y) and not collide
    with ctx.pose({hinge: 1.57}):
        ctx.expect_gap(door, cabinet, axis="y", positive_elem="door_panel", negative_elem="cabinet_body", min_gap=0.0)

    return ctx.report()

object_model = build_object_model()