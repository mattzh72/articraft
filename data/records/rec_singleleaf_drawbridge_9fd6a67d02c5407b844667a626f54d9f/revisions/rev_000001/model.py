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

    concrete = model.material("concrete", rgba=(0.67, 0.68, 0.66, 1.0))
    steel = model.material("steel", rgba=(0.30, 0.33, 0.36, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.20, 0.22, 0.24, 1.0))
    asphalt = model.material("asphalt", rgba=(0.12, 0.12, 0.13, 1.0))
    safety = model.material("safety_paint", rgba=(0.76, 0.73, 0.41, 1.0))

    frame = model.part("shore_frame")

    frame.visual(
        Box((1.10, 3.32, 0.18)),
        origin=Origin(xyz=(-0.55, 0.0, -0.09)),
        material=concrete,
        name="approach_slab",
    )
    frame.visual(
        Box((0.65, 0.38, 0.18)),
        origin=Origin(xyz=(-0.78, -1.82, -0.09)),
        material=concrete,
        name="left_approach_shoulder",
    )
    frame.visual(
        Box((0.65, 0.38, 0.18)),
        origin=Origin(xyz=(-0.78, 1.82, -0.09)),
        material=concrete,
        name="right_approach_shoulder",
    )
    frame.visual(
        Box((0.22, 4.00, 0.26)),
        origin=Origin(xyz=(-0.54, 0.0, -0.31)),
        material=concrete,
        name="shore_crossbeam",
    )
    frame.visual(
        Box((6.87, 0.40, 0.85)),
        origin=Origin(xyz=(2.985, -2.20, -0.275)),
        material=concrete,
        name="left_wall",
    )
    frame.visual(
        Box((6.87, 0.40, 0.85)),
        origin=Origin(xyz=(2.985, 2.20, -0.275)),
        material=concrete,
        name="right_wall",
    )
    frame.visual(
        Box((0.25, 4.00, 0.34)),
        origin=Origin(xyz=(6.30, 0.0, -0.17)),
        material=concrete,
        name="receiver_beam",
    )
    frame.visual(
        Box((0.70, 0.34, 0.16)),
        origin=Origin(xyz=(-0.16, -2.03, -0.63)),
        material=concrete,
        name="left_bearing_pedestal",
    )
    frame.visual(
        Box((0.70, 0.34, 0.16)),
        origin=Origin(xyz=(-0.16, 2.03, -0.63)),
        material=concrete,
        name="right_bearing_pedestal",
    )
    frame.visual(
        Box((0.28, 0.18, 0.08)),
        origin=Origin(xyz=(0.02, -1.98, -0.51)),
        material=dark_steel,
        name="left_bearing_cap",
    )
    frame.visual(
        Box((0.28, 0.18, 0.08)),
        origin=Origin(xyz=(0.02, 1.98, -0.51)),
        material=dark_steel,
        name="right_bearing_cap",
    )
    frame.inertial = Inertial.from_geometry(
        Box((6.90, 4.80, 0.90)),
        mass=14000.0,
        origin=Origin(xyz=(2.95, 0.0, -0.25)),
    )

    leaf = model.part("bridge_leaf")
    leaf.visual(
        Box((6.17, 3.32, 0.10)),
        origin=Origin(xyz=(3.085, 0.0, 0.28)),
        material=asphalt,
        name="roadway_deck",
    )
    leaf.visual(
        Box((6.00, 0.18, 0.08)),
        origin=Origin(xyz=(3.00, -1.72, 0.27)),
        material=steel,
        name="left_edge_plate",
    )
    leaf.visual(
        Box((6.00, 0.18, 0.08)),
        origin=Origin(xyz=(3.00, 1.72, 0.27)),
        material=steel,
        name="right_edge_plate",
    )
    leaf.visual(
        Box((5.90, 0.20, 0.60)),
        origin=Origin(xyz=(3.15, -1.76, -0.07)),
        material=steel,
        name="left_girder",
    )
    leaf.visual(
        Box((5.90, 0.20, 0.60)),
        origin=Origin(xyz=(3.15, 1.76, -0.07)),
        material=steel,
        name="right_girder",
    )
    leaf.visual(
        Box((0.40, 3.26, 0.46)),
        origin=Origin(xyz=(0.22, 0.0, 0.0)),
        material=steel,
        name="heel_beam",
    )
    leaf.visual(
        Box((0.75, 0.18, 0.56)),
        origin=Origin(xyz=(-0.05, -1.76, -0.02)),
        material=steel,
        name="left_trunnion_web",
    )
    leaf.visual(
        Box((0.75, 0.18, 0.56)),
        origin=Origin(xyz=(-0.05, 1.76, -0.02)),
        material=steel,
        name="right_trunnion_web",
    )
    leaf.visual(
        Cylinder(radius=0.14, length=0.22),
        origin=Origin(xyz=(0.0, -1.89, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_trunnion",
    )
    leaf.visual(
        Cylinder(radius=0.14, length=0.22),
        origin=Origin(xyz=(0.0, 1.89, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_trunnion",
    )
    leaf.visual(
        Box((0.20, 3.70, 0.26)),
        origin=Origin(xyz=(6.07, 0.0, 0.10)),
        material=steel,
        name="nose_beam",
    )
    leaf.visual(
        Box((6.00, 0.10, 0.14)),
        origin=Origin(xyz=(3.00, -1.80, 0.37)),
        material=safety,
        name="left_curb",
    )
    leaf.visual(
        Box((6.00, 0.10, 0.14)),
        origin=Origin(xyz=(3.00, 1.80, 0.37)),
        material=safety,
        name="right_curb",
    )
    leaf.inertial = Inertial.from_geometry(
        Box((6.20, 3.80, 0.75)),
        mass=5200.0,
        origin=Origin(xyz=(3.00, 0.0, 0.11)),
    )

    model.articulation(
        "shore_to_leaf",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=leaf,
        origin=Origin(xyz=(0.0, 0.0, -0.33)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=450000.0,
            velocity=0.35,
            lower=0.0,
            upper=1.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("shore_frame")
    leaf = object_model.get_part("bridge_leaf")
    hinge = object_model.get_articulation("shore_to_leaf")

    ctx.expect_overlap(
        leaf,
        frame,
        axes="y",
        elem_a="roadway_deck",
        elem_b="approach_slab",
        min_overlap=3.30,
        name="leaf deck matches approach width",
    )
    ctx.expect_overlap(
        leaf,
        frame,
        axes="xy",
        elem_a="left_trunnion",
        elem_b="left_bearing_cap",
        min_overlap=0.08,
        name="left trunnion stays centered over left bearing cap",
    )
    ctx.expect_overlap(
        leaf,
        frame,
        axes="xy",
        elem_a="right_trunnion",
        elem_b="right_bearing_cap",
        min_overlap=0.08,
        name="right trunnion stays centered over right bearing cap",
    )
    ctx.expect_gap(
        leaf,
        frame,
        axis="z",
        positive_elem="left_trunnion",
        negative_elem="left_bearing_cap",
        max_gap=0.002,
        max_penetration=0.0,
        name="left trunnion sits on its bearing cap",
    )
    ctx.expect_gap(
        leaf,
        frame,
        axis="z",
        positive_elem="right_trunnion",
        negative_elem="right_bearing_cap",
        max_gap=0.002,
        max_penetration=0.0,
        name="right trunnion sits on its bearing cap",
    )

    closed_deck = ctx.part_element_world_aabb(leaf, elem="roadway_deck")
    closed_approach = ctx.part_element_world_aabb(frame, elem="approach_slab")
    deck_level_ok = (
        closed_deck is not None
        and closed_approach is not None
        and abs(closed_deck[1][2] - closed_approach[1][2]) <= 0.002
        and abs(closed_deck[0][0] - closed_approach[1][0]) <= 0.002
    )
    ctx.check(
        "closed leaf sits flush with the approach",
        deck_level_ok,
        details=f"deck={closed_deck}, approach={closed_approach}",
    )

    closed_nose = ctx.part_element_world_aabb(leaf, elem="nose_beam")
    opened_nose = None
    with ctx.pose({hinge: 1.15}):
        opened_nose = ctx.part_element_world_aabb(leaf, elem="nose_beam")
    leaf_opens_up = (
        closed_nose is not None
        and opened_nose is not None
        and opened_nose[1][2] > closed_nose[1][2] + 4.5
        and opened_nose[1][0] < closed_nose[1][0] - 2.0
    )
    ctx.check(
        "leaf opens upward from the shore side hinge",
        leaf_opens_up,
        details=f"closed_nose={closed_nose}, opened_nose={opened_nose}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
