from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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

    painted_steel = model.material("painted_steel", rgba=(0.30, 0.34, 0.38, 1.0))
    darker_steel = model.material("darker_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    machinery_grey = model.material("machinery_grey", rgba=(0.43, 0.46, 0.49, 1.0))
    concrete = model.material("concrete", rgba=(0.58, 0.59, 0.60, 1.0))
    deck_surface = model.material("deck_surface", rgba=(0.18, 0.19, 0.20, 1.0))
    counterweight_color = model.material("counterweight_color", rgba=(0.54, 0.48, 0.42, 1.0))

    shore_frame = model.part("shore_frame")
    shore_frame.inertial = Inertial.from_geometry(
        Box((4.4, 2.4, 6.4)),
        mass=42000.0,
        origin=Origin(xyz=(0.0, -0.2, 3.2)),
    )
    shore_frame.visual(
        Box((4.4, 2.4, 0.6)),
        origin=Origin(xyz=(0.0, -0.2, 0.3)),
        material=concrete,
        name="foundation_block",
    )
    shore_frame.visual(
        Box((0.48, 2.2, 5.8)),
        origin=Origin(xyz=(-1.96, -0.2, 3.5)),
        material=painted_steel,
        name="left_frame_tower",
    )
    shore_frame.visual(
        Box((0.48, 2.2, 5.8)),
        origin=Origin(xyz=(1.96, -0.2, 3.5)),
        material=painted_steel,
        name="right_frame_tower",
    )
    shore_frame.visual(
        Box((0.72, 0.32, 2.55)),
        origin=Origin(xyz=(-1.72, -1.24, 1.875)),
        material=painted_steel,
        name="left_lower_rear_cheek",
    )
    shore_frame.visual(
        Box((0.72, 0.32, 2.55)),
        origin=Origin(xyz=(1.72, -1.24, 1.875)),
        material=painted_steel,
        name="right_lower_rear_cheek",
    )
    shore_frame.visual(
        Box((0.40, 1.20, 1.10)),
        origin=Origin(xyz=(-1.82, -0.56, 4.05)),
        material=painted_steel,
        name="left_bearing_web",
    )
    shore_frame.visual(
        Box((0.40, 1.20, 1.10)),
        origin=Origin(xyz=(1.82, -0.56, 4.05)),
        material=painted_steel,
        name="right_bearing_web",
    )
    shore_frame.visual(
        Box((4.4, 0.28, 0.80)),
        origin=Origin(xyz=(0.0, -1.26, 5.55)),
        material=painted_steel,
        name="upper_rear_wall",
    )
    shore_frame.visual(
        Box((4.4, 2.2, 0.45)),
        origin=Origin(xyz=(0.0, -0.2, 6.175)),
        material=painted_steel,
        name="top_crosshead",
    )
    shore_frame.visual(
        Box((4.4, 0.55, 0.55)),
        origin=Origin(xyz=(0.0, -0.10, 2.55)),
        material=painted_steel,
        name="lower_ring_beam",
    )
    shore_frame.visual(
        Box((3.44, 1.05, 0.25)),
        origin=Origin(xyz=(0.0, -0.605, 4.275)),
        material=deck_surface,
        name="approach_deck",
    )
    shore_frame.visual(
        Box((0.24, 0.88, 0.92)),
        origin=Origin(xyz=(-1.68, 0.15, 4.07)),
        material=machinery_grey,
        name="left_bearing_housing",
    )
    shore_frame.visual(
        Box((0.24, 0.88, 0.92)),
        origin=Origin(xyz=(1.68, 0.15, 4.07)),
        material=machinery_grey,
        name="right_bearing_housing",
    )
    shore_frame.visual(
        Box((0.18, 0.88, 0.24)),
        origin=Origin(xyz=(-1.68, 0.15, 4.49)),
        material=darker_steel,
        name="left_bearing_cap",
    )
    shore_frame.visual(
        Box((0.18, 0.88, 0.24)),
        origin=Origin(xyz=(1.68, 0.15, 4.49)),
        material=darker_steel,
        name="right_bearing_cap",
    )
    shore_frame.visual(
        Box((4.4, 0.22, 0.60)),
        origin=Origin(xyz=(0.0, 0.90, 0.85)),
        material=concrete,
        name="quay_edge",
    )

    bridge_leaf = model.part("bridge_leaf")
    bridge_leaf.inertial = Inertial.from_geometry(
        Box((2.95, 12.0, 2.2)),
        mass=22000.0,
        origin=Origin(xyz=(0.0, 4.9, -0.15)),
    )
    bridge_leaf.visual(
        Box((2.90, 10.45, 0.22)),
        origin=Origin(xyz=(0.0, 5.425, 0.13)),
        material=deck_surface,
        name="leaf_deck",
    )
    bridge_leaf.visual(
        Box((0.22, 10.25, 1.20)),
        origin=Origin(xyz=(-1.30, 5.325, -0.55)),
        material=painted_steel,
        name="left_girder",
    )
    bridge_leaf.visual(
        Box((0.22, 10.25, 1.20)),
        origin=Origin(xyz=(1.30, 5.325, -0.55)),
        material=painted_steel,
        name="right_girder",
    )
    bridge_leaf.visual(
        Box((2.46, 0.34, 0.55)),
        origin=Origin(xyz=(0.0, 10.58, -0.02)),
        material=painted_steel,
        name="toe_beam",
    )
    bridge_leaf.visual(
        Box((2.20, 0.60, 0.22)),
        origin=Origin(xyz=(0.0, 0.30, 0.13)),
        material=darker_steel,
        name="heel_plate",
    )
    bridge_leaf.visual(
        Cylinder(radius=0.18, length=3.12),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=darker_steel,
        name="trunnion_barrel",
    )
    bridge_leaf.visual(
        Box((2.10, 0.60, 1.05)),
        origin=Origin(xyz=(0.0, -0.40, -0.58)),
        material=painted_steel,
        name="counterweight_web",
    )
    bridge_leaf.visual(
        Box((2.70, 1.45, 1.70)),
        origin=Origin(xyz=(0.0, -1.28, -0.98)),
        material=counterweight_color,
        name="counterweight_block",
    )
    bridge_leaf.visual(
        Box((2.46, 0.45, 0.35)),
        origin=Origin(xyz=(0.0, 3.50, -0.02)),
        material=machinery_grey,
        name="mid_cross_tie",
    )

    model.articulation(
        "shore_to_leaf",
        ArticulationType.REVOLUTE,
        parent=shore_frame,
        child=bridge_leaf,
        origin=Origin(xyz=(0.0, 0.15, 4.15)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=400000.0, velocity=0.35, lower=0.0, upper=1.22),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shore_frame = object_model.get_part("shore_frame")
    bridge_leaf = object_model.get_part("bridge_leaf")
    leaf_joint = object_model.get_articulation("shore_to_leaf")

    ctx.expect_overlap(
        bridge_leaf,
        shore_frame,
        axes="x",
        elem_a="leaf_deck",
        elem_b="approach_deck",
        min_overlap=2.75,
        name="leaf deck aligns laterally with the shore approach deck",
    )
    ctx.expect_gap(
        bridge_leaf,
        shore_frame,
        axis="y",
        positive_elem="heel_plate",
        negative_elem="approach_deck",
        min_gap=0.05,
        max_gap=0.25,
        name="shore approach meets the leaf heel with a compact hinge gap",
    )

    rest_aabb = ctx.part_element_world_aabb(bridge_leaf, elem="toe_beam")
    with ctx.pose({leaf_joint: 1.05}):
        raised_aabb = ctx.part_element_world_aabb(bridge_leaf, elem="toe_beam")
        ctx.expect_gap(
            bridge_leaf,
            shore_frame,
            axis="z",
            positive_elem="counterweight_block",
            negative_elem="foundation_block",
            min_gap=0.15,
            name="raised counterweight clears the foundation block",
        )

    ctx.check(
        "leaf nose rises when opened",
        rest_aabb is not None
        and raised_aabb is not None
        and raised_aabb[0][2] > rest_aabb[0][2] + 5.0,
        details=f"rest_toe={rest_aabb}, raised_toe={raised_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
