from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_handwash_faucet")

    stainless = Material("brushed_stainless", rgba=(0.72, 0.74, 0.72, 1.0))
    dark = Material("dark_recess", rgba=(0.015, 0.017, 0.018, 1.0))
    blue = Material("blue_cold_marker", rgba=(0.05, 0.22, 0.78, 1.0))

    faucet = model.part("faucet")

    # Wall is the Y-Z plane; +X projects out from the wall toward the user.
    faucet.visual(
        Box((0.026, 0.220, 0.340)),
        origin=Origin(xyz=(0.000, 0.000, 0.000)),
        material=stainless,
        name="mounting_plate",
    )
    # Slightly proud screw heads make the flat wall plate read as mounted hardware.
    for z, name in ((0.125, "upper_screw"), (-0.125, "lower_screw")):
        faucet.visual(
            Cylinder(radius=0.014, length=0.006),
            origin=Origin(xyz=(0.016, 0.000, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark,
            name=name,
        )

    # Main valve body and fixed wall-to-spout plumbing.
    faucet.visual(
        Cylinder(radius=0.040, length=0.070),
        origin=Origin(xyz=(0.048, 0.000, -0.060), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="valve_body",
    )
    faucet.visual(
        Cylinder(radius=0.024, length=0.048),
        origin=Origin(xyz=(0.082, 0.000, -0.040)),
        material=stainless,
        name="riser_socket",
    )
    faucet.visual(
        Cylinder(radius=0.014, length=0.180),
        origin=Origin(xyz=(0.082, 0.000, 0.070)),
        material=stainless,
        name="vertical_spout_arm",
    )
    faucet.visual(
        Sphere(radius=0.017),
        origin=Origin(xyz=(0.082, 0.000, 0.160)),
        material=stainless,
        name="spout_elbow",
    )
    faucet.visual(
        Cylinder(radius=0.013, length=0.108),
        origin=Origin(xyz=(0.136, 0.000, 0.160), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="spout_tube",
    )
    faucet.visual(
        Sphere(radius=0.015),
        origin=Origin(xyz=(0.190, 0.000, 0.160)),
        material=stainless,
        name="outlet_elbow",
    )
    outlet_tube = cq.Workplane("XY").circle(0.012).circle(0.006).extrude(0.044)
    faucet.visual(
        mesh_from_cadquery(outlet_tube, "downturned_outlet"),
        origin=Origin(xyz=(0.190, 0.000, 0.116)),
        material=stainless,
        name="downturned_outlet",
    )
    faucet.visual(
        Cylinder(radius=0.008, length=0.003),
        origin=Origin(xyz=(0.190, 0.000, 0.115), rpy=(0.0, 0.0, 0.0)),
        material=dark,
        name="outlet_hole",
    )

    # The stem collar is fixed to the valve body.  Its outer face is the hinge
    # plane for the separate side lever.
    faucet.visual(
        Cylinder(radius=0.014, length=0.014),
        origin=Origin(xyz=(0.048, 0.045, -0.060), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="stem_collar",
    )

    side_lever = model.part("side_lever")
    side_lever.visual(
        Cylinder(radius=0.018, length=0.030),
        origin=Origin(xyz=(0.000, 0.015, 0.000), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="lever_hub",
    )
    side_lever.visual(
        Box((0.014, 0.014, 0.115)),
        origin=Origin(xyz=(0.000, 0.026, 0.064)),
        material=stainless,
        name="lever_arm",
    )
    side_lever.visual(
        Sphere(radius=0.017),
        origin=Origin(xyz=(0.000, 0.026, 0.126)),
        material=blue,
        name="lever_tip",
    )

    model.articulation(
        "valve_stem",
        ArticulationType.REVOLUTE,
        parent=faucet,
        child=side_lever,
        origin=Origin(xyz=(0.048, 0.052, -0.060)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=-0.45, upper=1.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    faucet = object_model.get_part("faucet")
    lever = object_model.get_part("side_lever")
    valve_stem = object_model.get_articulation("valve_stem")

    ctx.expect_contact(
        lever,
        faucet,
        elem_a="lever_hub",
        elem_b="stem_collar",
        contact_tol=0.001,
        name="lever hub seats on fixed valve stem collar",
    )
    ctx.expect_overlap(
        lever,
        faucet,
        axes="xz",
        elem_a="lever_hub",
        elem_b="stem_collar",
        min_overlap=0.010,
        name="lever hub is coaxial with horizontal valve stem",
    )

    closed_aabb = ctx.part_world_aabb(lever)
    with ctx.pose({valve_stem: 0.90}):
        rotated_aabb = ctx.part_world_aabb(lever)
    ctx.check(
        "side lever rotates forward about the horizontal stem",
        closed_aabb is not None
        and rotated_aabb is not None
        and rotated_aabb[1][0] > closed_aabb[1][0] + 0.045,
        details=f"closed_aabb={closed_aabb}, rotated_aabb={rotated_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
