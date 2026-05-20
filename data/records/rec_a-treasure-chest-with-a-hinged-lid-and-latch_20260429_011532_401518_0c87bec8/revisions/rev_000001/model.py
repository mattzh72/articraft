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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="treasure_chest")

    # Base part: hollow chest body
    base = model.part("base")
    # Hollow base: outer 0.6x0.4x0.4m, 0.02m walls, sits on ground (z=0)
    base_geom = (
        cq.Workplane("XY")
        .box(0.6, 0.4, 0.4)
        .translate((0, 0, 0.2))  # Outer box: bottom at z=0, top at z=0.4
        .cut(
            cq.Workplane("XY")
            .box(0.56, 0.36, 0.38)
            .translate((0, 0, 0.21))  # Inner cavity: bottom at z=0.02, top at z=0.38 (0.02m walls)
        )
    )
    # Base part frame at world origin (0,0,0), visual already positioned to sit on ground
    base.visual(
        mesh_from_cadquery(base_geom, "base_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="base_shell",
    )

    # Lid part: hinged top panel
    lid = model.part("lid")
    lid_size = (0.62, 0.42, 0.02)  # Slightly larger than base top opening
    # Lid part frame is at the hinge line (back edge of the lid, aligned with base's back top edge)
    # Visual offset: lid extends 0.42m forward (y+) from hinge, so center is 0.21m forward, 0.01m up (z)
    lid.visual(
        Box(lid_size),
        origin=Origin(xyz=(0.0, 0.21, 0.01)),
        name="lid_panel",
    )

    # Hinge articulation: base to lid (revolute, opens upward)
    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        # Hinge line at back edge of base top: world (0, -0.2, 0.4) relative to base part frame (0,0,0)
        origin=Origin(xyz=(0.0, -0.2, 0.4)),
        axis=(1.0, 0.0, 0.0),  # Hinge along x-axis (left-right)
        motion_limits=MotionLimits(effort=5.0, velocity=3.0, lower=0.0, upper=1.57),  # 0 (closed) to ~90 degrees open
    )

    # Latch part: rotating catch to secure lid
    latch = model.part("latch")
    latch_size = (0.04, 0.02, 0.06)  # Small rectangular catch
    latch.visual(
        Box(latch_size),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="latch_catch",
    )

    # Latch articulation: base to latch (revolute, swings up to lock lid)
    model.articulation(
        "base_to_latch",
        ArticulationType.REVOLUTE,
        parent=base,
        child=latch,
        # Latch position: front center of base, just below top rim
        # World position: (0, 0.2, 0.37) relative to base part frame (0,0,0)
        # Latch is 0.06m tall: when down (q=0), top at 0.37+0.03=0.40 (matches lid bottom at 0.40)
        origin=Origin(xyz=(0.0, 0.2, 0.37)),
        axis=(1.0, 0.0, 0.0),  # Rotate around x-axis (up-down swing)
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=0.5),  # 0 (down) to ~28 degrees up
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    latch = object_model.get_part("latch")
    lid_hinge = object_model.get_articulation("base_to_lid")
    latch_hinge = object_model.get_articulation("base_to_latch")

    # Allow intentional overlap between base and lid (lid sits on base)
    ctx.allow_overlap("base", "lid", reason="Lid sits on top of base, intentional contact overlap")
    # Allow intentional overlap between base and latch (latch catch touches base top rim)
    ctx.allow_overlap("base", "latch", elem_a="base_shell", elem_b="latch_catch", reason="Latch catch touches base top rim when down")
    # Latch is connected via joint but has no physical contact in rest pose
    ctx.allow_isolated_part("latch", reason="Latch is connected to base via revolute joint, no physical contact in rest pose")

    # Check rest pose (lid closed, latch down)
    with ctx.pose({lid_hinge: 0.0, latch_hinge: 0.0}):
        # Lid should sit flush on base with minimal penetration
        ctx.expect_gap(lid, base, axis="z", max_penetration=0.005, name="lid closed seats on base")
        # Lid should cover most of the base opening
        ctx.expect_overlap(lid, base, axes="xy", min_overlap=0.4, name="lid covers base opening")
        # Latch down should clear the closed lid (lid is above latch)
        ctx.expect_gap(lid, latch, axis="z", max_penetration=0.01, name="latch down clears closed lid")

    # Check open lid pose
    with ctx.pose({lid_hinge: 1.57, latch_hinge: 0.0}):
        lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
        base_aabb = ctx.part_element_world_aabb(base, elem="base_shell")
        if lid_aabb and base_aabb:
            lid_min, lid_max = lid_aabb
            base_min, base_max = base_aabb
            # Handle both Vec3 objects and tuples
            lid_max_z = lid_max.z if hasattr(lid_max, 'z') else lid_max[2]
            base_max_z = base_max.z if hasattr(base_max, 'z') else base_max[2]
            ctx.check(
                "lid opens upward",
                lid_max_z > base_max_z + 0.3,
                details=f"lid max z={lid_max_z}, base max z={base_max_z}",
            )
        else:
            # Fallback to part AABB if element AABB fails
            lid_aabb_part = ctx.part_world_aabb(lid)
            base_aabb_part = ctx.part_world_aabb(base)
            if lid_aabb_part and base_aabb_part:
                lid_min_p, lid_max_p = lid_aabb_part
                base_min_p, base_max_p = base_aabb_part
                lid_max_z = lid_max_p.z if hasattr(lid_max_p, 'z') else lid_max_p[2]
                base_max_z = base_max_p.z if hasattr(base_max_p, 'z') else base_max_p[2]
                ctx.check(
                    "lid opens upward",
                    lid_max_z > base_max_z + 0.3,
                    details=f"lid max z={lid_max_z}, base max z={base_max_z} (fallback to part AABB)",
                )
            else:
                ctx.fail("lid opens upward", "Could not retrieve AABB for lid or base")

    # Check latch locked pose (swung up to catch lid)
    with ctx.pose({lid_hinge: 0.0, latch_hinge: 0.5}):
        ctx.expect_contact(latch, lid, contact_tol=0.02, name="latch catches lid when up")

    return ctx.report()


object_model = build_object_model()
