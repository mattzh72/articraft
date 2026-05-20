from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_leaf_drawbridge")

    concrete = model.material("concrete", rgba=(0.63, 0.64, 0.66, 1.0))
    structural_steel = model.material("structural_steel", rgba=(0.30, 0.33, 0.36, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.21, 0.23, 0.26, 1.0))
    deck_wearing = model.material("deck_wearing", rgba=(0.17, 0.18, 0.19, 1.0))
    bearing_metal = model.material("bearing_metal", rgba=(0.40, 0.40, 0.42, 1.0))

    shore_frame = model.part("shore_frame")
    shore_frame.visual(
        Box((2.60, 6.40, 0.65)),
        origin=Origin(xyz=(-1.30, 0.00, 0.325)),
        material=concrete,
        name="base_pier",
    )
    shore_frame.visual(
        Box((0.60, 0.90, 0.65)),
        origin=Origin(xyz=(0.00, 2.70, 0.325)),
        material=concrete,
        name="left_bearing_pedestal",
    )
    shore_frame.visual(
        Box((0.60, 0.90, 0.65)),
        origin=Origin(xyz=(0.00, -2.70, 0.325)),
        material=concrete,
        name="right_bearing_pedestal",
    )
    shore_frame.visual(
        Box((2.15, 0.60, 1.10)),
        origin=Origin(xyz=(-0.82, 2.70, 1.20)),
        material=concrete,
        name="left_side_wall",
    )
    shore_frame.visual(
        Box((2.15, 0.60, 1.10)),
        origin=Origin(xyz=(-0.82, -2.70, 1.20)),
        material=concrete,
        name="right_side_wall",
    )
    shore_frame.visual(
        Box((0.50, 5.40, 1.10)),
        origin=Origin(xyz=(-1.75, 0.00, 1.20)),
        material=concrete,
        name="back_wall",
    )
    shore_frame.visual(
        Box((0.55, 5.40, 0.30)),
        origin=Origin(xyz=(-1.725, 0.00, 1.88)),
        material=structural_steel,
        name="crosshead",
    )
    shore_frame.visual(
        Box((1.60, 3.40, 0.18)),
        origin=Origin(xyz=(-0.80, 0.00, 1.17)),
        material=deck_wearing,
        name="approach_roadway",
    )
    shore_frame.visual(
        Box((0.25, 4.80, 0.28)),
        origin=Origin(xyz=(-0.125, 0.00, 0.93)),
        material=structural_steel,
        name="front_sill",
    )
    shore_frame.visual(
        Box((0.65, 0.66, 0.72)),
        origin=Origin(xyz=(0.025, 2.50, 1.11)),
        material=structural_steel,
        name="left_bearing_block",
    )
    shore_frame.visual(
        Box((0.65, 0.66, 0.72)),
        origin=Origin(xyz=(0.025, -2.50, 1.11)),
        material=structural_steel,
        name="right_bearing_block",
    )
    shore_frame.visual(
        Cylinder(radius=0.28, length=0.16),
        origin=Origin(xyz=(0.10, 2.93, 1.10), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bearing_metal,
        name="left_bearing_cap",
    )
    shore_frame.visual(
        Cylinder(radius=0.28, length=0.16),
        origin=Origin(xyz=(0.10, -2.93, 1.10), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bearing_metal,
        name="right_bearing_cap",
    )
    shore_frame.inertial = Inertial.from_geometry(
        Box((2.60, 6.40, 2.05)),
        mass=48000.0,
        origin=Origin(xyz=(-1.30, 0.00, 1.025)),
    )

    bridge_leaf = model.part("bridge_leaf")
    bridge_leaf.visual(
        Box((0.52, 4.10, 0.56)),
        origin=Origin(xyz=(0.14, 0.00, -0.10)),
        material=dark_steel,
        name="heel_beam",
    )
    bridge_leaf.visual(
        Box((3.25, 3.20, 0.12)),
        origin=Origin(xyz=(1.505, 0.00, 0.10)),
        material=structural_steel,
        name="deck_plate",
    )
    bridge_leaf.visual(
        Box((3.05, 2.95, 0.04)),
        origin=Origin(xyz=(1.58, 0.00, 0.14)),
        material=deck_wearing,
        name="wearing_surface",
    )
    bridge_leaf.visual(
        Box((2.85, 0.26, 0.86)),
        origin=Origin(xyz=(1.455, 1.93, -0.27)),
        material=dark_steel,
        name="left_girder",
    )
    bridge_leaf.visual(
        Box((2.85, 0.26, 0.86)),
        origin=Origin(xyz=(1.455, -1.93, -0.27)),
        material=dark_steel,
        name="right_girder",
    )
    bridge_leaf.visual(
        Box((0.12, 3.62, 0.18)),
        origin=Origin(xyz=(0.60, 0.00, -0.18)),
        material=dark_steel,
        name="cross_girder_a",
    )
    bridge_leaf.visual(
        Box((0.12, 3.62, 0.18)),
        origin=Origin(xyz=(1.50, 0.00, -0.18)),
        material=dark_steel,
        name="cross_girder_b",
    )
    bridge_leaf.visual(
        Box((0.12, 3.62, 0.18)),
        origin=Origin(xyz=(2.40, 0.00, -0.18)),
        material=dark_steel,
        name="cross_girder_c",
    )
    bridge_leaf.visual(
        Box((0.16, 3.82, 0.26)),
        origin=Origin(xyz=(3.05, 0.00, -0.03)),
        material=dark_steel,
        name="nose_beam",
    )
    bridge_leaf.visual(
        Cylinder(radius=0.18, length=0.12),
        origin=Origin(xyz=(0.06, 2.08, 0.00), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bearing_metal,
        name="left_trunnion",
    )
    bridge_leaf.visual(
        Cylinder(radius=0.18, length=0.12),
        origin=Origin(xyz=(0.06, -2.08, 0.00), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bearing_metal,
        name="right_trunnion",
    )
    bridge_leaf.inertial = Inertial.from_geometry(
        Box((3.25, 4.10, 0.86)),
        mass=14500.0,
        origin=Origin(xyz=(1.62, 0.00, -0.18)),
    )

    model.articulation(
        "shore_to_leaf",
        ArticulationType.REVOLUTE,
        parent=shore_frame,
        child=bridge_leaf,
        origin=Origin(xyz=(0.12, 0.00, 1.10)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=250000.0,
            velocity=0.35,
            lower=0.0,
            upper=1.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shore_frame = object_model.get_part("shore_frame")
    bridge_leaf = object_model.get_part("bridge_leaf")
    leaf_joint = object_model.get_articulation("shore_to_leaf")

    with ctx.pose({leaf_joint: 0.0}):
        ctx.expect_gap(
            bridge_leaf,
            shore_frame,
            axis="x",
            positive_elem="deck_plate",
            negative_elem="approach_roadway",
            max_gap=0.02,
            max_penetration=0.0,
            name="closed leaf starts at the shore sill",
        )
        ctx.expect_overlap(
            bridge_leaf,
            shore_frame,
            axes="y",
            elem_a="deck_plate",
            elem_b="approach_roadway",
            min_overlap=3.0,
            name="leaf roadway aligns with the shore opening width",
        )
        closed_leaf_surface = ctx.part_element_world_aabb(bridge_leaf, elem="wearing_surface")
        closed_approach = ctx.part_element_world_aabb(shore_frame, elem="approach_roadway")
        flush_top = (
            closed_leaf_surface is not None
            and closed_approach is not None
            and abs(closed_leaf_surface[1][2] - closed_approach[1][2]) <= 0.002
        )
        ctx.check(
            "closed leaf roadway sits flush with the approach deck",
            flush_top,
            details=f"leaf_surface={closed_leaf_surface}, approach={closed_approach}",
        )

    closed_nose = ctx.part_element_world_aabb(bridge_leaf, elem="nose_beam")
    with ctx.pose({leaf_joint: 1.05}):
        open_nose = ctx.part_element_world_aabb(bridge_leaf, elem="nose_beam")
        ctx.expect_gap(
            bridge_leaf,
            shore_frame,
            axis="z",
            positive_elem="nose_beam",
            negative_elem="approach_roadway",
            min_gap=1.5,
            name="opened leaf clears well above the shore deck",
        )

    opened_upward = (
        closed_nose is not None
        and open_nose is not None
        and open_nose[1][2] > closed_nose[1][2] + 2.5
    )
    ctx.check(
        "positive rotation lifts the free end upward",
        opened_upward,
        details=f"closed_nose={closed_nose}, open_nose={open_nose}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
