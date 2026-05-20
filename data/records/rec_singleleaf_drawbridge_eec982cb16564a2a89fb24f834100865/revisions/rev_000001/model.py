from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="single_leaf_drawbridge")

    concrete = model.material("weathered_concrete", color=(0.45, 0.46, 0.43, 1.0))
    dark_steel = model.material("dark_blued_steel", color=(0.07, 0.09, 0.10, 1.0))
    worn_steel = model.material("worn_edge_steel", color=(0.23, 0.26, 0.27, 1.0))
    road = model.material("dark_road_deck", color=(0.035, 0.04, 0.04, 1.0))
    yellow = model.material("safety_yellow", color=(0.95, 0.72, 0.12, 1.0))

    bearing_block_shape = (
        cq.Workplane("XY")
        .box(0.42, 0.38, 0.50)
        .faces(">Y")
        .workplane()
        .circle(0.17)
        .cutThruAll()
        .edges("|Y")
        .fillet(0.025)
    )
    bearing_block_mesh = mesh_from_cadquery(
        bearing_block_shape,
        "bearing_block",
        tolerance=0.001,
        angular_tolerance=0.08,
    )

    shore_frame = model.part("shore_frame")
    shore_frame.visual(
        Box((1.75, 3.35, 0.22)),
        origin=Origin(xyz=(-0.55, 0.0, 0.11)),
        material=concrete,
        name="foundation_slab",
    )
    shore_frame.visual(
        Box((1.20, 0.32, 0.86)),
        origin=Origin(xyz=(-0.28, 1.36, 0.56)),
        material=concrete,
        name="side_wall_0",
    )
    shore_frame.visual(
        Box((1.20, 0.32, 0.86)),
        origin=Origin(xyz=(-0.28, -1.36, 0.56)),
        material=concrete,
        name="side_wall_1",
    )
    shore_frame.visual(
        Box((0.24, 3.00, 0.78)),
        origin=Origin(xyz=(-1.03, 0.0, 0.51)),
        material=concrete,
        name="rear_abutment",
    )
    shore_frame.visual(
        Box((0.34, 2.98, 0.22)),
        origin=Origin(xyz=(-0.16, 0.0, 0.96)),
        material=dark_steel,
        name="top_cross_tie",
    )
    shore_frame.visual(
        Box((0.30, 2.90, 0.22)),
        origin=Origin(xyz=(-0.96, 0.0, 0.86)),
        material=dark_steel,
        name="rear_cross_tie",
    )
    shore_frame.visual(
        Box((0.78, 1.65, 0.16)),
        origin=Origin(xyz=(-0.62, 0.0, 0.53)),
        material=road,
        name="shore_road_deck",
    )
    shore_frame.visual(
        Box((0.12, 1.78, 0.18)),
        origin=Origin(xyz=(-0.20, 0.0, 0.50)),
        material=worn_steel,
        name="hinge_sill",
    )
    shore_frame.visual(
        bearing_block_mesh,
        origin=Origin(xyz=(0.0, 1.10, 0.62)),
        material=dark_steel,
        name="bearing_0",
    )
    shore_frame.visual(
        bearing_block_mesh,
        origin=Origin(xyz=(0.0, -1.10, 0.62)),
        material=dark_steel,
        name="bearing_1",
    )
    shore_frame.visual(
        Box((0.48, 0.18, 0.24)),
        origin=Origin(xyz=(-0.03, 1.22, 0.30)),
        material=dark_steel,
        name="bearing_bracket_0",
    )
    shore_frame.visual(
        Box((0.48, 0.18, 0.24)),
        origin=Origin(xyz=(-0.03, -1.22, 0.30)),
        material=dark_steel,
        name="bearing_bracket_1",
    )
    shore_frame.visual(
        Box((0.10, 0.24, 0.08)),
        origin=Origin(xyz=(0.0, 1.10, 0.461)),
        material=worn_steel,
        name="bearing_pad_0",
    )
    shore_frame.visual(
        Box((0.10, 0.24, 0.08)),
        origin=Origin(xyz=(0.0, -1.10, 0.461)),
        material=worn_steel,
        name="bearing_pad_1",
    )

    bridge_leaf = model.part("bridge_leaf")
    bridge_leaf.visual(
        Cylinder(radius=0.12, length=2.28),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=worn_steel,
        name="trunnion_shaft",
    )
    bridge_leaf.visual(
        Box((3.72, 1.42, 0.10)),
        origin=Origin(xyz=(1.91, 0.0, -0.08)),
        material=worn_steel,
        name="deck_plate",
    )
    bridge_leaf.visual(
        Box((3.88, 0.18, 0.34)),
        origin=Origin(xyz=(1.95, 0.80, -0.04)),
        material=dark_steel,
        name="side_girder_0",
    )
    bridge_leaf.visual(
        Box((3.88, 0.18, 0.34)),
        origin=Origin(xyz=(1.95, -0.80, -0.04)),
        material=dark_steel,
        name="side_girder_1",
    )
    bridge_leaf.visual(
        Box((0.24, 1.78, 0.32)),
        origin=Origin(xyz=(0.12, 0.0, -0.04)),
        material=dark_steel,
        name="hinge_cross_girder",
    )
    bridge_leaf.visual(
        Box((0.24, 1.78, 0.30)),
        origin=Origin(xyz=(3.78, 0.0, -0.05)),
        material=dark_steel,
        name="nose_cross_girder",
    )
    for index, x_pos in enumerate((0.95, 1.78, 2.62)):
        bridge_leaf.visual(
            Box((0.16, 1.56, 0.18)),
            origin=Origin(xyz=(x_pos, 0.0, -0.16)),
            material=dark_steel,
            name=f"underside_rib_{index}",
        )
    bridge_leaf.visual(
        Box((3.20, 1.18, 0.022)),
        origin=Origin(xyz=(2.02, 0.0, -0.025)),
        material=road,
        name="road_wearing_panel",
    )
    bridge_leaf.visual(
        Box((0.18, 1.58, 0.024)),
        origin=Origin(xyz=(0.42, 0.0, 0.008)),
        material=yellow,
        name="shore_warning_stripe",
    )
    bridge_leaf.visual(
        Box((0.18, 1.58, 0.024)),
        origin=Origin(xyz=(3.52, 0.0, 0.008)),
        material=yellow,
        name="nose_warning_stripe",
    )

    model.articulation(
        "leaf_hinge",
        ArticulationType.REVOLUTE,
        parent=shore_frame,
        child=bridge_leaf,
        origin=Origin(xyz=(0.0, 0.0, 0.62)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=9000.0, velocity=0.35, lower=0.0, upper=1.25),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    shore_frame = object_model.get_part("shore_frame")
    bridge_leaf = object_model.get_part("bridge_leaf")
    hinge = object_model.get_articulation("leaf_hinge")

    ctx.allow_overlap(
        bridge_leaf,
        shore_frame,
        elem_a="trunnion_shaft",
        elem_b="bearing_pad_0",
        reason=(
            "The shaft is seated with a tiny compressive fit on the fixed bearing "
            "pad so the leaf is physically carried at the hinge line."
        ),
    )
    ctx.allow_overlap(
        bridge_leaf,
        shore_frame,
        elem_a="trunnion_shaft",
        elem_b="bearing_pad_1",
        reason=(
            "The shaft is seated with a tiny compressive fit on the fixed bearing "
            "pad so the leaf is physically carried at the hinge line."
        ),
    )

    ctx.check(
        "single revolute bridge leaf",
        len(object_model.articulations) == 1
        and hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"articulations={object_model.articulations}",
    )
    ctx.expect_within(
        bridge_leaf,
        shore_frame,
        axes="xz",
        inner_elem="trunnion_shaft",
        outer_elem="bearing_0",
        margin=0.09,
        name="trunnion centered in positive bearing",
    )
    ctx.expect_overlap(
        bridge_leaf,
        shore_frame,
        axes="y",
        elem_a="trunnion_shaft",
        elem_b="bearing_0",
        min_overlap=0.18,
        name="positive bearing captures the trunnion",
    )
    ctx.expect_within(
        bridge_leaf,
        shore_frame,
        axes="xz",
        inner_elem="trunnion_shaft",
        outer_elem="bearing_1",
        margin=0.09,
        name="trunnion centered in negative bearing",
    )
    ctx.expect_overlap(
        bridge_leaf,
        shore_frame,
        axes="y",
        elem_a="trunnion_shaft",
        elem_b="bearing_1",
        min_overlap=0.18,
        name="negative bearing captures the trunnion",
    )
    ctx.expect_gap(
        bridge_leaf,
        shore_frame,
        axis="z",
        positive_elem="trunnion_shaft",
        negative_elem="bearing_pad_0",
        max_penetration=0.002,
        name="positive bearing pad lightly supports the shaft",
    )
    ctx.expect_gap(
        bridge_leaf,
        shore_frame,
        axis="z",
        positive_elem="trunnion_shaft",
        negative_elem="bearing_pad_1",
        max_penetration=0.002,
        name="negative bearing pad lightly supports the shaft",
    )

    rest_tip = ctx.part_element_world_aabb(bridge_leaf, elem="nose_cross_girder")
    with ctx.pose({hinge: 1.25}):
        raised_tip = ctx.part_element_world_aabb(bridge_leaf, elem="nose_cross_girder")
    ctx.check(
        "leaf raises away from the waterway",
        rest_tip is not None
        and raised_tip is not None
        and raised_tip[0][2] > rest_tip[1][2] + 2.0,
        details=f"rest_tip={rest_tip}, raised_tip={raised_tip}",
    )

    return ctx.report()


object_model = build_object_model()
