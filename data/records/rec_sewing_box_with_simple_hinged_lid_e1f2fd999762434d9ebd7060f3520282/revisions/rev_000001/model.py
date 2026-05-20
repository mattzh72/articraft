from __future__ import annotations

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sewing_box")

    # Dimensions
    width = 0.25
    depth = 0.15
    height = 0.06
    wall_thickness = 0.005

    base = model.part("base")
    # Base: 25cm wide, 15cm deep, 6cm tall, 5mm wall thickness
    base_cq = (
        cq.Workplane("XY")
        .box(width, depth, height, centered=(True, True, False))
        .faces("+Z")
        .shell(-wall_thickness)
    )
    base.visual(
        mesh_from_cadquery(base_cq, "base_shell"),
        name="base_shell"
    )

    lid = model.part("lid")
    # Lid: 25cm wide, 15cm deep, 5mm thick
    lid_cq = (
        cq.Workplane("XY")
        .box(width, depth, wall_thickness, centered=(True, True, False))
    )
    # The lid part frame will be at the rear hinge (Y = depth/2, Z = height).
    # The lid itself spans from Y = -depth/2 to Y = depth/2 around its own center,
    # so we shift it by Y = -depth/2 so it spans Y = -depth to Y = 0 relative to the hinge.
    lid.visual(
        mesh_from_cadquery(lid_cq, "lid_shell"),
        origin=Origin(xyz=(0.0, -depth / 2, 0.0)),
        name="lid_shell"
    )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, depth / 2, height)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=0.0, upper=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("base_to_lid")

    # Check closed state
    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(lid, base, axis="z", max_gap=0.001, max_penetration=0.0, name="lid sits flush on base")
        ctx.expect_overlap(lid, base, axes="xy", min_overlap=0.14, name="lid covers base footprint")

    # Check open state
    with ctx.pose({hinge: 1.5}):
        ctx.expect_gap(lid, base, axis="z", max_penetration=0.001, name="lid does not penetrate base when open")
        lid_aabb = ctx.part_world_aabb(lid)
        base_aabb = ctx.part_world_aabb(base)
        if lid_aabb and base_aabb:
            ctx.check("lid opens upward", lid_aabb[1][2] > base_aabb[1][2] + 0.10, details=f"lid max z: {lid_aabb[1][2]}")

    return ctx.report()


object_model = build_object_model()