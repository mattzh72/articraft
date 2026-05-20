from __future__ import annotations

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hard_equipment_case")

    # Dimensions
    case_width = 0.4
    case_depth = 0.3
    base_height = 0.1
    lid_height = 0.05
    wall_thickness = 0.005
    fillet_radius = 0.02

    # Base shell
    base_cq = (
        cq.Workplane("XY")
        .box(case_width, case_depth, base_height)
        .edges("|Z")
        .fillet(fillet_radius)
        .faces("+Z")
        .shell(-wall_thickness)
    ).translate((0, 0, base_height / 2))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(base_cq, "base_shell_mesh"),
        name="base_shell",
    )

    # Base hinge knuckles
    base.visual(
        Cylinder(radius=0.008, length=0.04),
        origin=Origin(xyz=(-0.05, 0.16, 0.1), rpy=(0, 1.5708, 0)),
        name="base_hinge_knuckle_left",
    )
    base.visual(
        Box((0.04, 0.012, 0.01)),
        origin=Origin(xyz=(-0.05, 0.154, 0.095)),
        name="base_hinge_mount_left",
    )
    
    base.visual(
        Cylinder(radius=0.008, length=0.04),
        origin=Origin(xyz=(0.05, 0.16, 0.1), rpy=(0, 1.5708, 0)),
        name="base_hinge_knuckle_right",
    )
    base.visual(
        Box((0.04, 0.012, 0.01)),
        origin=Origin(xyz=(0.05, 0.154, 0.095)),
        name="base_hinge_mount_right",
    )

    # Base latches
    base.visual(
        Box((0.04, 0.01, 0.04)),
        origin=Origin(xyz=(-0.1, -0.155, 0.08)),
        name="base_latch_left",
    )
    base.visual(
        Box((0.04, 0.01, 0.04)),
        origin=Origin(xyz=(0.1, -0.155, 0.08)),
        name="base_latch_right",
    )

    # Lid shell
    lid_cq = (
        cq.Workplane("XY")
        .box(case_width, case_depth, lid_height)
        .edges("|Z")
        .fillet(fillet_radius)
        .faces("-Z")
        .shell(-wall_thickness)
    ).translate((0, 0, lid_height / 2))

    lid = model.part("lid")
    # Lid part frame is at the hinge
    lid.visual(
        mesh_from_cadquery(lid_cq, "lid_shell_mesh"),
        origin=Origin(xyz=(0, -0.16, 0)),
        name="lid_shell",
    )

    # Lid hinge knuckle
    lid.visual(
        Cylinder(radius=0.008, length=0.04),
        origin=Origin(xyz=(0, 0, 0), rpy=(0, 1.5708, 0)),
        name="lid_hinge_knuckle",
    )
    lid.visual(
        Box((0.04, 0.02, 0.01)),
        origin=Origin(xyz=(0, -0.01, 0.005)),
        name="lid_hinge_mount",
    )

    # Lid latch catches
    lid.visual(
        Box((0.04, 0.01, 0.02)),
        origin=Origin(xyz=(-0.1, -0.315, 0.01)),
        name="lid_latch_catch_left",
    )
    lid.visual(
        Box((0.04, 0.01, 0.02)),
        origin=Origin(xyz=(0.1, -0.315, 0.01)),
        name="lid_latch_catch_right",
    )

    # Articulation
    # The hinge axis is X. Positive rotation should open the lid.
    # The lid extends along -Y. To open upward (+Z), rotation around -X should be positive.
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0, 0.16, 0.1)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.74), # ~100 degrees
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("lid_hinge")

    # At rest, check gap between base and lid
    ctx.expect_gap(lid, base, axis="z", min_gap=0.0, max_gap=0.001, positive_elem="lid_shell", negative_elem="base_shell")
    ctx.expect_overlap(lid, base, axes="xy", min_overlap=0.1, elem_a="lid_shell", elem_b="base_shell")

    # Open pose
    with ctx.pose({hinge: 1.74}):
        # Check that the lid's front has moved up
        # We can check that the lid's world position (which is at the hinge) hasn't changed
        # but the lid shell's AABB should show that the front edge is higher.
        lid_aabb = ctx.part_world_aabb(lid)
        if lid_aabb is not None:
            ctx.check("lid_opens_upward", lid_aabb[1][2] > 0.3, details=f"max_z={lid_aabb[1][2]}")

    return ctx.report()


object_model = build_object_model()