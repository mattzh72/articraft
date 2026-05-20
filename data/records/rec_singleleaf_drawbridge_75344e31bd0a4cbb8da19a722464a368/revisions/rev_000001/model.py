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
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_leaf_drawbridge")

    concrete = Material("weathered_concrete", rgba=(0.55, 0.54, 0.50, 1.0))
    asphalt = Material("dark_asphalt", rgba=(0.06, 0.065, 0.07, 1.0))
    frame_steel = Material("dark_blued_steel", rgba=(0.12, 0.18, 0.23, 1.0))
    deck_steel = Material("painted_deck_plate", rgba=(0.34, 0.42, 0.44, 1.0))
    bearing_steel = Material("oiled_bearing_steel", rgba=(0.04, 0.045, 0.05, 1.0))
    stripe_yellow = Material("safety_yellow", rgba=(0.92, 0.72, 0.10, 1.0))

    frame = model.part("shore_frame")
    frame.visual(
        Box((1.35, 2.35, 0.22)),
        origin=Origin(xyz=(-0.50, 0.0, 0.11)),
        material=concrete,
        name="foundation",
    )
    frame.visual(
        Box((0.22, 1.82, 0.55)),
        origin=Origin(xyz=(-0.24, 0.0, 0.385)),
        material=concrete,
        name="abutment_wall",
    )
    frame.visual(
        Box((1.04, 1.48, 0.085)),
        origin=Origin(xyz=(-0.68, 0.0, 0.665)),
        material=concrete,
        name="approach_slab",
    )
    frame.visual(
        Box((1.00, 1.38, 0.025)),
        origin=Origin(xyz=(-0.69, 0.0, 0.720)),
        material=asphalt,
        name="approach_road",
    )

    bearing_ring = mesh_from_geometry(
        TorusGeometry(radius=0.095, tube=0.025, radial_segments=24, tubular_segments=32),
        "drawbridge_bearing_ring",
    )
    for index, y in enumerate((-0.93, 0.93)):
        frame.visual(
            Box((0.46, 0.24, 0.38)),
            origin=Origin(xyz=(0.0, y, 0.36)),
            material=concrete,
            name=f"bearing_plinth_{index}",
        )
        frame.visual(
            Box((0.30, 0.20, 0.18)),
            origin=Origin(xyz=(0.0, y, 0.608)),
            material=frame_steel,
            name=f"bearing_saddle_{index}",
        )
        frame.visual(
            Box((0.14, 0.20, 0.52)),
            origin=Origin(xyz=(-0.21, y, 0.64)),
            material=frame_steel,
            name=f"rear_upright_{index}",
        )
    frame.visual(
        bearing_ring,
        origin=Origin(xyz=(0.0, -0.93, 0.75), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bearing_steel,
        name="bearing_ring_0",
    )
    frame.visual(
        bearing_ring,
        origin=Origin(xyz=(0.0, 0.93, 0.75), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bearing_steel,
        name="bearing_ring_1",
    )
    frame.visual(
        Box((0.18, 2.08, 0.10)),
        origin=Origin(xyz=(-0.21, 0.0, 0.94)),
        material=frame_steel,
        name="overhead_tie",
    )

    leaf = model.part("bridge_leaf")
    leaf.visual(
        Cylinder(radius=0.052, length=1.93),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bearing_steel,
        name="trunnion_shaft",
    )
    leaf.visual(
        Box((2.65, 1.36, 0.035)),
        origin=Origin(xyz=(1.38, 0.0, -0.085)),
        material=deck_steel,
        name="deck_skin",
    )
    leaf.visual(
        Box((0.16, 1.55, 0.16)),
        origin=Origin(xyz=(0.08, 0.0, -0.065)),
        material=frame_steel,
        name="hinge_frame",
    )
    leaf.visual(
        Box((0.14, 1.55, 0.16)),
        origin=Origin(xyz=(2.72, 0.0, -0.065)),
        material=frame_steel,
        name="front_frame",
    )
    for index, y in enumerate((-0.705, 0.705)):
        leaf.visual(
            Box((2.72, 0.10, 0.16)),
            origin=Origin(xyz=(1.40, y, -0.065)),
            material=frame_steel,
            name=f"side_frame_{index}",
        )
        leaf.visual(
            Box((2.45, 0.045, 0.075)),
            origin=Origin(xyz=(1.45, y * 0.92, 0.045)),
            material=frame_steel,
            name=f"low_guard_{index}",
        )
    for index, x in enumerate((0.78, 1.48, 2.18)):
        leaf.visual(
            Box((0.075, 1.34, 0.070)),
            origin=Origin(xyz=(x, 0.0, -0.135)),
            material=frame_steel,
            name=f"cross_rib_{index}",
        )
    leaf.visual(
        Box((0.035, 1.32, 0.018)),
        origin=Origin(xyz=(2.805, 0.0, 0.015)),
        material=stripe_yellow,
        name="nose_stripe",
    )

    model.articulation(
        "frame_to_leaf",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=leaf,
        origin=Origin(xyz=(0.0, 0.0, 0.75)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=250000.0, velocity=0.35, lower=0.0, upper=1.22),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("shore_frame")
    leaf = object_model.get_part("bridge_leaf")
    hinge = object_model.get_articulation("frame_to_leaf")

    ctx.check(
        "single revolute bridge leaf",
        len(object_model.articulations) == 1 and hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"articulations={object_model.articulations}",
    )
    ctx.expect_overlap(
        leaf,
        frame,
        axes="y",
        elem_a="trunnion_shaft",
        elem_b="bearing_ring_0",
        min_overlap=0.015,
        name="shaft passes through one side bearing",
    )
    ctx.expect_overlap(
        leaf,
        frame,
        axes="y",
        elem_a="trunnion_shaft",
        elem_b="bearing_ring_1",
        min_overlap=0.015,
        name="shaft passes through opposite side bearing",
    )
    ctx.expect_gap(
        leaf,
        frame,
        axis="x",
        positive_elem="deck_skin",
        negative_elem="approach_slab",
        min_gap=0.04,
        max_gap=0.30,
        name="closed leaf starts beyond the shore slab",
    )

    closed_aabb = ctx.part_world_aabb(leaf)
    with ctx.pose({hinge: 1.22}):
        raised_aabb = ctx.part_world_aabb(leaf)

    closed_top = closed_aabb[1][2] if closed_aabb is not None else None
    raised_top = raised_aabb[1][2] if raised_aabb is not None else None
    ctx.check(
        "leaf raises about shore axis",
        closed_top is not None and raised_top is not None and raised_top > closed_top + 1.2,
        details=f"closed_top={closed_top}, raised_top={raised_top}",
    )

    return ctx.report()


object_model = build_object_model()
