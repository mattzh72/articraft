from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vertical_storing_dock_leveler")

    concrete = model.material("formed_concrete", rgba=(0.50, 0.50, 0.47, 1.0))
    dark_rubber = model.material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    deck_steel = model.material("blue_gray_painted_steel", rgba=(0.18, 0.28, 0.34, 1.0))
    worn_steel = model.material("worn_steel_edges", rgba=(0.55, 0.57, 0.56, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(1.0, 0.72, 0.06, 1.0))

    dock_face = model.part("dock_face")
    dock_face.visual(
        Box((0.22, 2.90, 2.70)),
        origin=Origin(xyz=(-0.11, 0.0, 1.35)),
        material=concrete,
        name="concrete_wall",
    )
    dock_face.visual(
        Box((1.65, 2.90, 0.10)),
        origin=Origin(xyz=(0.825, 0.0, -0.05)),
        material=concrete,
        name="pit_floor",
    )
    dock_face.visual(
        Box((0.20, 2.46, 0.12)),
        origin=Origin(xyz=(0.02, 0.0, 0.06)),
        material=concrete,
        name="rear_sill",
    )
    for index, y in enumerate((-1.53, 1.53)):
        dock_face.visual(
            Box((1.65, 0.16, 0.18)),
            origin=Origin(xyz=(0.825, y, -0.01)),
            material=concrete,
            name=f"pit_curb_{index}",
        )
    for index, y in enumerate((-1.29, 1.29)):
        dock_face.visual(
            Box((0.075, 0.18, 1.18)),
            origin=Origin(xyz=(0.0375, y, 0.82)),
            material=dark_rubber,
            name=f"dock_bumper_{index}",
        )
    dock_face.visual(
        Box((0.012, 2.34, 0.08)),
        origin=Origin(xyz=(-0.006, 0.0, 0.205)),
        material=safety_yellow,
        name="yellow_hinge_stripe",
    )

    deck = model.part("deck")
    deck.visual(
        Box((0.10, 2.20, 2.32)),
        origin=Origin(xyz=(-0.01, 0.0, 1.20)),
        material=deck_steel,
        name="deck_plate",
    )
    deck.visual(
        Cylinder(radius=0.055, length=2.34),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=worn_steel,
        name="rear_hinge_tube",
    )
    for index, y in enumerate((-1.14, 1.14)):
        deck.visual(
            Box((0.08, 0.10, 2.24)),
            origin=Origin(xyz=(-0.020, y, 1.22)),
            material=deck_steel,
            name=f"side_beam_{index}",
        )
    for index, z in enumerate((0.46, 0.82, 1.18, 1.54, 1.90, 2.22)):
        deck.visual(
            Box((0.040, 1.36, 0.030)),
            origin=Origin(xyz=(0.055, 0.0, z)),
            material=worn_steel,
            name=f"traction_bar_{index}",
        )

    model.articulation(
        "dock_to_deck",
        ArticulationType.REVOLUTE,
        parent=dock_face,
        child=deck,
        origin=Origin(xyz=(0.06, 0.0, 0.18)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5000.0, velocity=0.35, lower=0.0, upper=math.pi / 2.0),
    )

    lip = model.part("lip")
    lip.visual(
        Cylinder(radius=0.040, length=2.12),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=worn_steel,
        name="lip_hinge_barrel",
    )
    lip.visual(
        Box((0.055, 1.42, 0.72)),
        origin=Origin(xyz=(0.045, 0.0, -0.36)),
        material=safety_yellow,
        name="lip_plate",
    )
    lip.visual(
        Box((0.070, 1.36, 0.045)),
        origin=Origin(xyz=(0.050, 0.0, -0.70)),
        material=worn_steel,
        name="lip_nose_edge",
    )

    model.articulation(
        "deck_to_lip",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=lip,
        origin=Origin(xyz=(0.08, 0.0, 2.36)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=1.0, lower=0.0, upper=2.95),
    )

    for name, side, lower, upper in (
        ("side_skirt_0", 1.0, 0.0, math.pi / 2.0),
        ("side_skirt_1", -1.0, -math.pi / 2.0, 0.0),
    ):
        skirt = model.part(name)
        skirt.visual(
            Cylinder(radius=0.0225, length=2.18),
            origin=Origin(xyz=(-0.01, 0.0, 1.08)),
            material=worn_steel,
            name="skirt_hinge_barrel",
        )
        skirt.visual(
            Box((0.045, 0.36, 2.10)),
            origin=Origin(xyz=(-0.01, -side * 0.18, 1.08)),
            material=safety_yellow,
            name="skirt_panel",
        )
        skirt.visual(
            Box((0.055, 0.045, 2.04)),
            origin=Origin(xyz=(0.01, -side * 0.36, 1.08)),
            material=worn_steel,
            name="skirt_wear_edge",
        )
        model.articulation(
            f"deck_to_{name}",
            ArticulationType.REVOLUTE,
            parent=deck,
            child=skirt,
            origin=Origin(xyz=(0.0725, side * 1.10, 0.12)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=400.0, velocity=1.0, lower=lower, upper=upper),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    dock = object_model.get_part("dock_face")
    deck = object_model.get_part("deck")
    lip = object_model.get_part("lip")
    skirt_0 = object_model.get_part("side_skirt_0")
    skirt_1 = object_model.get_part("side_skirt_1")
    deck_joint = object_model.get_articulation("dock_to_deck")
    lip_joint = object_model.get_articulation("deck_to_lip")
    skirt_0_joint = object_model.get_articulation("deck_to_side_skirt_0")
    skirt_1_joint = object_model.get_articulation("deck_to_side_skirt_1")

    ctx.expect_gap(
        deck,
        dock,
        axis="x",
        positive_elem="deck_plate",
        negative_elem="concrete_wall",
        max_gap=0.001,
        max_penetration=0.00001,
        name="deck stows flush with the dock face",
    )
    ctx.expect_overlap(
        deck,
        dock,
        axes="y",
        elem_a="deck_plate",
        elem_b="concrete_wall",
        min_overlap=2.0,
        name="wide deck spans the dock opening",
    )
    ctx.expect_gap(
        deck,
        dock,
        axis="z",
        positive_elem="rear_hinge_tube",
        negative_elem="rear_sill",
        min_gap=0.0,
        max_gap=0.010,
        name="deck rear hinge is carried by the pit sill",
    )
    ctx.expect_contact(
        lip,
        deck,
        elem_a="lip_hinge_barrel",
        elem_b="deck_plate",
        contact_tol=0.002,
        name="lip hinge sits on the deck front edge",
    )
    ctx.expect_contact(
        skirt_0,
        deck,
        elem_a="skirt_panel",
        elem_b="deck_plate",
        contact_tol=0.002,
        name="upper side skirt is hinged on the deck long edge",
    )
    ctx.expect_contact(
        skirt_1,
        deck,
        elem_a="skirt_panel",
        elem_b="deck_plate",
        contact_tol=0.002,
        name="lower side skirt is hinged on the deck long edge",
    )

    rest_deck_aabb = ctx.part_element_world_aabb(deck, elem="deck_plate")
    with ctx.pose({deck_joint: math.pi / 2.0}):
        deployed_deck_aabb = ctx.part_element_world_aabb(deck, elem="deck_plate")
    ctx.check(
        "deck rotates down from vertical stow to a horizontal bridge",
        rest_deck_aabb is not None
        and deployed_deck_aabb is not None
        and deployed_deck_aabb[1][0] > rest_deck_aabb[1][0] + 2.0
        and deployed_deck_aabb[1][2] < 0.30,
        details=f"rest={rest_deck_aabb}, deployed={deployed_deck_aabb}",
    )

    with ctx.pose(
        {
            deck_joint: math.pi / 2.0,
            lip_joint: 2.95,
            skirt_0_joint: math.pi / 2.0,
            skirt_1_joint: -math.pi / 2.0,
        }
    ):
        deck_aabb = ctx.part_element_world_aabb(deck, elem="deck_plate")
        lip_aabb = ctx.part_element_world_aabb(lip, elem="lip_plate")
        skirt_0_aabb = ctx.part_element_world_aabb(skirt_0, elem="skirt_panel")
        skirt_1_aabb = ctx.part_element_world_aabb(skirt_1, elem="skirt_panel")

        ctx.check(
            "lip swings past the deck front edge",
            deck_aabb is not None
            and lip_aabb is not None
            and lip_aabb[1][0] > deck_aabb[1][0] + 0.55,
            details=f"deck={deck_aabb}, lip={lip_aabb}",
        )
        ctx.check(
            "both side skirts fold downward beside the deployed deck",
            deck_aabb is not None
            and skirt_0_aabb is not None
            and skirt_1_aabb is not None
            and skirt_0_aabb[0][2] < deck_aabb[0][2] - 0.25
            and skirt_1_aabb[0][2] < deck_aabb[0][2] - 0.25,
            details=f"deck={deck_aabb}, skirt_0={skirt_0_aabb}, skirt_1={skirt_1_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
