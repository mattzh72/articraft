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


PIVOT_Z = 0.86


def _pillow_block_with_bore() -> cq.Workplane:
    """Weatherproof side bearing housing with a real through-bore."""

    block = cq.Workplane("XY").box(0.58, 0.34, 0.58)
    bore = cq.Workplane("XZ").circle(0.205).extrude(0.70, both=True)
    block = block.cut(bore)
    return block


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="weatherproof_singleleaf_drawbridge")

    concrete = model.material("sealed_concrete", rgba=(0.55, 0.56, 0.54, 1.0))
    galvanized = model.material("galvanized_steel", rgba=(0.42, 0.50, 0.55, 1.0))
    dark_steel = model.material("dark_coated_steel", rgba=(0.13, 0.17, 0.18, 1.0))
    stainless = model.material("stainless_hardware", rgba=(0.78, 0.80, 0.78, 1.0))
    rubber = model.material("black_epdm_seals", rgba=(0.01, 0.012, 0.010, 1.0))
    asphalt = model.material("grit_wearing_surface", rgba=(0.055, 0.060, 0.055, 1.0))
    safety_yellow = model.material("safety_yellow_edges", rgba=(0.95, 0.76, 0.08, 1.0))

    support_frame = model.part("support_frame")

    # Anchored fixed frame: sealed concrete plinth, galvanized sill, tied bearing
    # pedestals, and a drip hood protecting the hinge/bearing line.
    support_frame.visual(
        Box((1.80, 3.20, 0.30)),
        origin=Origin(xyz=(-0.50, 0.0, 0.15)),
        material=concrete,
        name="foundation",
    )
    support_frame.visual(
        Box((0.64, 2.36, 0.08)),
        origin=Origin(xyz=(-0.52, 0.0, 0.81)),
        material=dark_steel,
        name="approach_plate",
    )
    support_frame.visual(
        Box((0.92, 2.72, 0.16)),
        origin=Origin(xyz=(-0.28, 0.0, 0.40)),
        material=galvanized,
        name="anchored_sill",
    )
    support_frame.visual(
        Box((0.06, 2.80, 0.035)),
        origin=Origin(xyz=(0.230, 0.0, 1.015)),
        material=rubber,
        name="hinge_gasket",
    )
    support_frame.visual(
        Box((0.08, 2.56, 0.06)),
        origin=Origin(xyz=(-0.20, 0.0, 0.80)),
        material=galvanized,
        name="rain_gutter",
    )

    bearing_mesh = mesh_from_cadquery(_pillow_block_with_bore(), "bearing_block")
    for i, y in enumerate((-1.45, 1.45)):
        support_frame.visual(
            Box((0.62, 0.42, 0.28)),
            origin=Origin(xyz=(0.0, y, 0.44)),
            material=galvanized,
            name=f"bearing_pedestal_{i}",
        )
        support_frame.visual(
            bearing_mesh,
            origin=Origin(xyz=(0.0, y, PIVOT_Z)),
            material=galvanized,
            name=f"bearing_block_{i}",
        )
        support_frame.visual(
            Box((0.74, 0.46, 0.08)),
            origin=Origin(xyz=(-0.02, y, PIVOT_Z + 0.37)),
            material=galvanized,
            name=f"bearing_roof_{i}",
        )
        support_frame.visual(
            Box((0.06, 0.46, 0.15)),
            origin=Origin(xyz=(0.33, y, PIVOT_Z + 0.285)),
            material=galvanized,
            name=f"drip_lip_{i}",
        )
        support_frame.visual(
            Cylinder(radius=0.035, length=0.026),
            origin=Origin(xyz=(-0.20, y - 0.13, 0.535)),
            material=stainless,
            name=f"anchor_bolt_{i}_0",
        )
        support_frame.visual(
            Cylinder(radius=0.035, length=0.026),
            origin=Origin(xyz=(0.20, y + 0.13, 0.535)),
            material=stainless,
            name=f"anchor_bolt_{i}_1",
        )

    support_frame.visual(
        Box((0.26, 2.70, 0.16)),
        origin=Origin(xyz=(-0.35, 0.0, PIVOT_Z - 0.03)),
        material=galvanized,
        name="rear_cross_tie",
    )
    support_frame.visual(
        Box((0.48, 3.18, 0.07)),
        origin=Origin(xyz=(-0.22, 0.0, PIVOT_Z + 0.435)),
        material=galvanized,
        name="joint_hood",
    )
    support_frame.visual(
        Box((0.055, 3.18, 0.16)),
        origin=Origin(xyz=(0.055, 0.0, PIVOT_Z + 0.37)),
        material=galvanized,
        name="hood_drip_edge",
    )

    bridge_leaf = model.part("bridge_leaf")

    # Rotating single leaf.  The local part frame is the trunnion axis, so the
    # heavy side pivots remain coaxial with the revolute joint.
    bridge_leaf.visual(
        Box((4.82, 2.12, 0.12)),
        origin=Origin(xyz=(2.55, 0.0, -0.10)),
        material=dark_steel,
        name="deck_plate",
    )
    bridge_leaf.visual(
        Box((4.58, 1.94, 0.030)),
        origin=Origin(xyz=(2.66, 0.0, -0.025)),
        material=asphalt,
        name="wearing_surface",
    )
    bridge_leaf.visual(
        Box((0.10, 2.05, 0.18)),
        origin=Origin(xyz=(0.34, 0.0, 0.07)),
        material=galvanized,
        name="apron_carrier",
    )
    bridge_leaf.visual(
        Box((0.05, 2.06, 0.035)),
        origin=Origin(xyz=(0.29, 0.0, 0.155)),
        material=rubber,
        name="hinge_apron",
    )
    for i, y in enumerate((-1.12, 1.12)):
        bridge_leaf.visual(
            Box((4.88, 0.16, 0.36)),
            origin=Origin(xyz=(2.56, y, -0.20)),
            material=galvanized,
            name=f"side_girder_{i}",
        )
        bridge_leaf.visual(
            Box((4.62, 0.045, 0.07)),
            origin=Origin(xyz=(2.68, y * 0.985, -0.005)),
            material=safety_yellow,
            name=f"sealed_curb_{i}",
        )
        bridge_leaf.visual(
            Box((4.70, 0.055, 0.08)),
            origin=Origin(xyz=(2.60, y * 1.015, -0.42)),
            material=galvanized,
            name=f"drip_hem_{i}",
        )

    for i, x in enumerate((0.55, 1.55, 2.55, 3.55, 4.55)):
        bridge_leaf.visual(
            Box((0.12, 2.18, 0.20)),
            origin=Origin(xyz=(x, 0.0, -0.25)),
            material=galvanized,
            name=f"crossbeam_{i}",
        )

    bridge_leaf.visual(
        Cylinder(radius=0.14, length=3.34),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="trunnion_shaft",
    )
    for i, y in enumerate((-1.70, 1.70)):
        bridge_leaf.visual(
            Cylinder(radius=0.17, length=0.08),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=stainless,
            name=f"trunnion_cap_{i}",
        )

    for i, y in enumerate((-0.86, 0.0, 0.86)):
        bridge_leaf.visual(
            Box((0.58, 0.09, 0.42)),
            origin=Origin(xyz=(0.26, y, -0.11)),
            material=galvanized,
            name=f"torque_web_{i}",
        )

    bridge_leaf.visual(
        Box((0.10, 2.05, 0.20)),
        origin=Origin(xyz=(4.99, 0.0, -0.13)),
        material=galvanized,
        name="nose_bar",
    )
    bridge_leaf.visual(
        Box((0.055, 1.98, 0.09)),
        origin=Origin(xyz=(5.055, 0.0, -0.04)),
        material=rubber,
        name="nose_seal",
    )

    model.articulation(
        "frame_to_leaf",
        ArticulationType.REVOLUTE,
        parent=support_frame,
        child=bridge_leaf,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=250000.0, velocity=0.25, lower=0.0, upper=1.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("support_frame")
    leaf = object_model.get_part("bridge_leaf")
    hinge = object_model.get_articulation("frame_to_leaf")

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            leaf,
            frame,
            axis="x",
            positive_elem="hinge_apron",
            negative_elem="hinge_gasket",
            min_gap=0.0,
            max_gap=0.012,
            name="closed hinge gasket has a narrow sealed gap",
        )
        ctx.expect_overlap(
            leaf,
            frame,
            axes="y",
            elem_a="hinge_apron",
            elem_b="hinge_gasket",
            min_overlap=1.90,
            name="hinge seal spans the traffic width",
        )
        for i in (0, 1):
            ctx.expect_within(
                leaf,
                frame,
                axes="xz",
                inner_elem="trunnion_shaft",
                outer_elem=f"bearing_block_{i}",
                margin=0.0,
                name=f"trunnion centered inside bearing block {i}",
            )
            ctx.expect_overlap(
                leaf,
                frame,
                axes="y",
                elem_a="trunnion_shaft",
                elem_b=f"bearing_block_{i}",
                min_overlap=0.28,
                name=f"trunnion retained by bearing block {i}",
            )

    closed_tip = ctx.part_element_world_aabb(leaf, elem="nose_seal")
    with ctx.pose({hinge: 0.85}):
        raised_tip = ctx.part_element_world_aabb(leaf, elem="nose_seal")

    ctx.check(
        "leaf raises on side trunnions",
        closed_tip is not None
        and raised_tip is not None
        and raised_tip[1][2] > closed_tip[1][2] + 2.8,
        details=f"closed_tip={closed_tip}, raised_tip={raised_tip}",
    )

    return ctx.report()


object_model = build_object_model()
