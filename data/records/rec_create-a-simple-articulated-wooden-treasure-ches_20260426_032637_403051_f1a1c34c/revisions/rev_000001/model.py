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
    model = ArticulatedObject(name="treasure_chest")

    # Base part
    base = model.part("base")
    
    # Base geometry: 0.6 x 0.4 x 0.3, hollowed out
    base_outer = cq.Workplane("XY").box(0.6, 0.4, 0.3, centered=(True, True, False))
    base_inner = cq.Workplane("XY").box(0.56, 0.36, 0.28, centered=(True, True, False)).translate((0, 0, 0.02))
    base_geom = base_outer.cut(base_inner)
    
    # Base straps and lock
    strap_base_box = cq.Workplane("XY").box(0.04, 0.42, 0.31, centered=(True, True, False)).translate((0, 0, -0.01))
    strap_base_geom = strap_base_box.translate((-0.2, 0, 0)).union(strap_base_box.translate((0.2, 0, 0)))
    strap_base_geom = strap_base_geom.cut(base_outer)
    
    lock_base = (
        cq.Workplane("XZ").workplane(offset=-0.2)
        .center(0, 0.26)
        .rect(0.06, 0.06)
        .extrude(-0.01)
    )
    
    base.visual(
        mesh_from_cadquery(base_geom, "base_mesh"),
        name="base_wood",
    )
    base.visual(
        mesh_from_cadquery(strap_base_geom, "base_straps"),
        name="base_straps",
    )
    base.visual(
        mesh_from_cadquery(lock_base, "base_lock"),
        name="base_lock",
    )

    # Lid part
    lid = model.part("lid")
    
    # Lid geometry: half-barrel, 0.6 x 0.4 x 0.2
    lid_outer = (
        cq.Workplane("YZ").workplane(offset=-0.3)
        .moveTo(-0.2, 0)
        .lineTo(-0.2, 0.05)
        .threePointArc((0, 0.2), (0.2, 0.05))
        .lineTo(0.2, 0)
        .close()
        .extrude(0.6)
    )
    lid_inner = (
        cq.Workplane("YZ").workplane(offset=-0.28)
        .moveTo(-0.18, 0)
        .lineTo(-0.18, 0.05)
        .threePointArc((0, 0.18), (0.18, 0.05))
        .lineTo(0.18, 0)
        .close()
        .extrude(0.56)
    )
    lid_geom = lid_outer.cut(lid_inner)
    
    # Lid straps and lock
    lid_strap_rim = cq.Workplane("XY").box(0.04, 0.42, 0.02, centered=(True, True, False))
    lid_strap_top = (
        cq.Workplane("YZ").workplane(offset=-0.02)
        .moveTo(-0.21, 0.02)
        .lineTo(-0.21, 0.05)
        .threePointArc((0, 0.21), (0.21, 0.05))
        .lineTo(0.21, 0.02)
        .close()
        .extrude(0.04)
    )
    lid_strap_single = lid_strap_rim.union(lid_strap_top)
    lid_strap_geom = lid_strap_single.translate((-0.2, 0, 0)).union(lid_strap_single.translate((0.2, 0, 0)))
    lid_strap_geom = lid_strap_geom.cut(lid_outer)
    
    lock_lid = (
        cq.Workplane("XZ").workplane(offset=-0.2)
        .center(0, 0.04)
        .rect(0.06, 0.06)
        .extrude(-0.01)
    )
    
    # The lid part frame is at the hinge.
    # The lid geometry has its back-most strap edge at (0, 0.21, 0).
    # To place this edge at the part frame origin, we offset the visual by (0, -0.21, 0).
    lid.visual(
        mesh_from_cadquery(lid_geom, "lid_mesh"),
        origin=Origin(xyz=(0.0, -0.21, 0.0)),
        name="lid_wood",
    )
    lid.visual(
        mesh_from_cadquery(lid_strap_geom, "lid_straps"),
        origin=Origin(xyz=(0.0, -0.21, 0.0)),
        name="lid_straps",
    )
    lid.visual(
        mesh_from_cadquery(lock_lid, "lid_lock"),
        origin=Origin(xyz=(0.0, -0.21, 0.0)),
        name="lid_lock",
    )

    # Articulation
    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        # Hinge is at the back-top edge of the base straps
        origin=Origin(xyz=(0.0, 0.21, 0.3)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=3.0, lower=0.0, upper=1.57),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("base_to_lid")

    # At rest, the lid should sit flush on the base
    ctx.expect_gap(lid, base, axis="z", max_gap=0.001, max_penetration=0.001)
    rest_aabb = ctx.part_world_aabb(lid)
    
    # Check that it opens correctly
    with ctx.pose({hinge: 1.5}):
        # When open, the lid should not penetrate the base
        ctx.expect_gap(lid, base, axis="z", max_gap=0.5, max_penetration=0.001)
        
        open_aabb = ctx.part_world_aabb(lid)
        if open_aabb and rest_aabb:
            ctx.check(
                "lid_opens_upward",
                open_aabb[1][2] > rest_aabb[1][2] + 0.1,
                details=f"Lid max Z should increase. Rest: {rest_aabb[1][2]:.3f}, Open: {open_aabb[1][2]:.3f}"
            )

    return ctx.report()


object_model = build_object_model()
