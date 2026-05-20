from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_leaf_drawbridge")

    concrete = Material("weathered_concrete", rgba=(0.48, 0.48, 0.43, 1.0))
    asphalt = Material("dark_asphalt", rgba=(0.06, 0.065, 0.06, 1.0))
    structural_steel = Material("painted_steel", rgba=(0.12, 0.19, 0.23, 1.0))
    worn_steel = Material("worn_edge_steel", rgba=(0.36, 0.38, 0.36, 1.0))
    bearing_metal = Material("oiled_bearing_metal", rgba=(0.03, 0.035, 0.035, 1.0))
    rail_yellow = Material("safety_yellow", rgba=(0.95, 0.70, 0.10, 1.0))

    frame = model.part("shore_frame")

    # Fixed abutment and roadway approach.
    frame.visual(
        Box((2.10, 5.60, 0.76)),
        origin=Origin(xyz=(-1.45, 0.0, 0.38)),
        material=concrete,
        name="abutment_block",
    )
    frame.visual(
        Box((2.00, 3.50, 0.10)),
        origin=Origin(xyz=(-1.30, 0.0, 0.81)),
        material=asphalt,
        name="approach_road",
    )

    # A fixed U-shaped steel lip around the water/opening.  The side lips sit
    # just outside the bridge leaf; the end stop pad supports the free end when
    # the leaf is closed.
    for y, name in ((2.05, "side_lip_0"), (-2.05, "side_lip_1")):
        frame.visual(
            Box((7.90, 0.20, 0.30)),
            origin=Origin(xyz=(4.30, y, 0.60)),
            material=worn_steel,
            name=name,
        )
    for y, name in ((2.05, "cheek_anchor_0"), (-2.05, "cheek_anchor_1")):
        frame.visual(
            Box((0.84, 0.20, 0.30)),
            origin=Origin(xyz=(-0.02, y, 0.45)),
            material=concrete,
            name=name,
        )
    frame.visual(
        Box((0.50, 3.30, 0.12)),
        origin=Origin(xyz=(7.85, 0.0, 0.55)),
        material=worn_steel,
        name="stop_flange",
    )
    for y, name in ((1.80, "stop_return_0"), (-1.80, "stop_return_1")):
        frame.visual(
            Box((0.50, 0.30, 0.07)),
            origin=Origin(xyz=(7.85, y, 0.485)),
            material=worn_steel,
            name=name,
        )

    # Heavy bearing housings on both sides of the shore end.  These are fixed
    # side bearings; the moving leaf frame is the child of the revolute joint.
    for y, side in ((2.28, "upper"), (-2.28, "lower")):
        frame.visual(
            Box((0.80, 0.55, 0.08)),
            origin=Origin(xyz=(0.0, y, 0.42)),
            material=structural_steel,
            name=f"bearing_base_{side}",
        )
        frame.visual(
            Box((0.46, 0.42, 0.16)),
            origin=Origin(xyz=(0.0, y, 0.54)),
            material=structural_steel,
            name=f"bearing_stand_{side}",
        )
        frame.visual(
            Cylinder(radius=0.32, length=0.16),
            origin=Origin(xyz=(0.0, y, 0.80), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=bearing_metal,
            name=f"bearing_disk_{side}",
        )
        frame.visual(
            Cylinder(radius=0.17, length=0.20),
            origin=Origin(xyz=(0.0, y + math.copysign(0.18, y), 0.80), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=worn_steel,
            name=f"bearing_cap_{side}",
        )

    leaf = model.part("bridge_leaf")

    # The leaf part frame lies on the horizontal shore-side hinge axis.  The
    # closed bridge deck extends in local +X away from that axis.
    leaf.visual(
        Box((7.90, 3.55, 0.22)),
        origin=Origin(xyz=(4.05, 0.0, -0.08)),
        material=structural_steel,
        name="deck_plate",
    )
    leaf.visual(
        Box((7.70, 3.20, 0.06)),
        origin=Origin(xyz=(4.10, 0.0, 0.06)),
        material=asphalt,
        name="road_surface",
    )
    for y, name in ((1.78, "side_girder_0"), (-1.78, "side_girder_1")):
        leaf.visual(
            Box((7.90, 0.16, 0.46)),
            origin=Origin(xyz=(4.05, y, -0.02)),
            material=structural_steel,
            name=name,
        )
    for x in (1.0, 2.5, 4.0, 5.5, 7.0):
        leaf.visual(
            Box((0.14, 3.55, 0.12)),
            origin=Origin(xyz=(x, 0.0, -0.25)),
            material=worn_steel,
            name=f"cross_beam_{x:.1f}",
        )

    leaf.visual(
        Cylinder(radius=0.16, length=4.40),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bearing_metal,
        name="trunnion_shaft",
    )
    for y, name in ((1.68, "trunnion_collar_0"), (-1.68, "trunnion_collar_1")):
        leaf.visual(
            Cylinder(radius=0.24, length=0.18),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=worn_steel,
            name=name,
        )

    for y, suffix in ((1.52, "0"), (-1.52, "1")):
        for x in (0.9, 2.2, 3.5, 4.8, 6.1, 7.4):
            leaf.visual(
                Box((0.08, 0.08, 0.55)),
                origin=Origin(xyz=(x, y, 0.36)),
                material=rail_yellow,
                name=f"rail_post_{suffix}_{x:.1f}",
            )
        leaf.visual(
            Box((6.80, 0.08, 0.08)),
            origin=Origin(xyz=(4.15, y, 0.66)),
            material=rail_yellow,
            name=f"top_rail_{suffix}",
        )
        leaf.visual(
            Box((6.80, 0.06, 0.06)),
            origin=Origin(xyz=(4.15, y, 0.42)),
            material=rail_yellow,
            name=f"mid_rail_{suffix}",
        )

    model.articulation(
        "leaf_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=leaf,
        origin=Origin(xyz=(0.0, 0.0, 0.80)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=250000.0, velocity=0.35, lower=0.0, upper=1.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("shore_frame")
    leaf = object_model.get_part("bridge_leaf")
    hinge = object_model.get_articulation("leaf_hinge")

    ctx.check(
        "single horizontal revolute leaf",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 6) for v in hinge.axis) == (0.0, -1.0, 0.0),
        details=f"type={hinge.articulation_type}, axis={hinge.axis}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            leaf,
            frame,
            axis="z",
            positive_elem="deck_plate",
            negative_elem="stop_flange",
            max_gap=0.002,
            max_penetration=0.0,
            name="closed leaf rests on stop flange",
        )
        ctx.expect_overlap(
            leaf,
            frame,
            axes="xy",
            elem_a="deck_plate",
            elem_b="stop_flange",
            min_overlap=0.25,
            name="stop flange spans under free end",
        )
        ctx.expect_gap(
            frame,
            leaf,
            axis="y",
            positive_elem="side_lip_0",
            negative_elem="deck_plate",
            min_gap=0.05,
            max_gap=0.30,
            name="positive side lip clears leaf",
        )
        ctx.expect_gap(
            leaf,
            frame,
            axis="y",
            positive_elem="deck_plate",
            negative_elem="side_lip_1",
            min_gap=0.05,
            max_gap=0.30,
            name="negative side lip clears leaf",
        )
        closed_aabb = ctx.part_element_world_aabb(leaf, elem="deck_plate")

    with ctx.pose({hinge: 1.20}):
        raised_aabb = ctx.part_element_world_aabb(leaf, elem="deck_plate")

    closed_tip_height = closed_aabb[1][2] if closed_aabb is not None else None
    raised_tip_height = raised_aabb[1][2] if raised_aabb is not None else None
    ctx.check(
        "leaf raises upward from shore axis",
        closed_tip_height is not None
        and raised_tip_height is not None
        and raised_tip_height > closed_tip_height + 5.0,
        details=f"closed_max_z={closed_tip_height}, raised_max_z={raised_tip_height}",
    )

    return ctx.report()


object_model = build_object_model()
