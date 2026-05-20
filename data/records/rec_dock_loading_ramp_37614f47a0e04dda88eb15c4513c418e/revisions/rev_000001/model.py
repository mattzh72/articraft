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
    model = ArticulatedObject(name="edge_of_dock_lip_plate_leveler")

    steel_yellow = Material("worn_safety_yellow", rgba=(0.93, 0.63, 0.08, 1.0))
    dark_steel = Material("dark_burnished_steel", rgba=(0.18, 0.18, 0.17, 1.0))
    concrete_mat = Material("dock_concrete", rgba=(0.48, 0.48, 0.45, 1.0))
    hinge_steel = Material("oiled_hinge_steel", rgba=(0.10, 0.11, 0.11, 1.0))

    deck_len = 1.20
    deck_width = 1.80
    deck_thick = 0.060
    deck_center_z = 0.060
    deck_top_z = deck_center_z + deck_thick / 2.0
    cyl_to_y = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))

    dock = model.part("dock")
    dock.visual(
        Box((0.50, 2.10, 0.38)),
        origin=Origin(xyz=(-0.30, 0.0, -0.10)),
        material=concrete_mat,
        name="dock_face",
    )
    dock.visual(
        Box((0.035, 1.95, 0.24)),
        origin=Origin(xyz=(-0.055, 0.0, -0.030)),
        material=dark_steel,
        name="steel_curb",
    )
    dock.visual(
        Box((0.22, 1.90, 0.030)),
        origin=Origin(xyz=(-0.14, 0.0, deck_top_z + 0.015)),
        material=dark_steel,
        name="dock_wear_plate",
    )

    # Fixed hinge knuckles welded to the dock edge; the deck knuckles are
    # interleaved in the open gaps along Y.
    for i, y in enumerate((-0.60, 0.0, 0.60)):
        dock.visual(
            Cylinder(radius=0.022, length=0.30),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=cyl_to_y.rpy),
            material=hinge_steel,
            name=f"dock_hinge_barrel_{i}",
        )
        dock.visual(
            Box((0.18, 0.30, 0.018)),
            origin=Origin(xyz=(-0.09, y, 0.015)),
            material=dark_steel,
            name=f"dock_hinge_leaf_{i}",
        )
    dock.visual(
        Cylinder(radius=0.010, length=1.66),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=cyl_to_y.rpy),
        material=hinge_steel,
        name="rear_hinge_pin",
    )

    deck = model.part("deck")
    deck.visual(
        Box((1.18, deck_width, deck_thick)),
        origin=Origin(xyz=(0.61, 0.0, deck_center_z)),
        material=steel_yellow,
        name="deck_plate",
    )
    # Raised anti-slip bars and side weldments make the plate read as a heavy
    # dock-leveler deck rather than a bare cuboid.
    for i, x in enumerate((0.18, 0.34, 0.50, 0.66, 0.82, 0.98, 1.12)):
        deck.visual(
            Box((0.025, 1.58, 0.008)),
            origin=Origin(xyz=(x, 0.0, deck_top_z + 0.003)),
            material=dark_steel,
            name=f"deck_tread_{i}",
        )
    for i, y in enumerate((-0.895, 0.895)):
        deck.visual(
            Box((1.08, 0.045, 0.040)),
            origin=Origin(xyz=(0.64, y, deck_top_z + 0.018)),
            material=dark_steel,
            name=f"deck_edge_bar_{i}",
        )
    for i, y in enumerate((-0.52, 0.0, 0.52)):
        deck.visual(
            Box((0.92, 0.055, 0.10)),
            origin=Origin(xyz=(0.62, y, -0.019)),
            material=dark_steel,
            name=f"deck_under_rib_{i}",
        )

    for i, y in enumerate((-0.30, 0.30)):
        deck.visual(
            Cylinder(radius=0.021, length=0.24),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=cyl_to_y.rpy),
            material=hinge_steel,
            name=f"deck_rear_barrel_{i}",
        )
        deck.visual(
            Box((0.25, 0.24, 0.018)),
            origin=Origin(xyz=(0.125, y, 0.024)),
            material=dark_steel,
            name=f"deck_rear_leaf_{i}",
        )

    for i, y in enumerate((-0.63, 0.0, 0.63)):
        deck.visual(
            Cylinder(radius=0.020, length=0.24),
            origin=Origin(xyz=(deck_len, y, 0.0), rpy=cyl_to_y.rpy),
            material=hinge_steel,
            name=f"deck_front_barrel_{i}",
        )
        deck.visual(
            Box((0.19, 0.24, 0.022)),
            origin=Origin(xyz=(deck_len - 0.095, y, 0.029)),
            material=dark_steel,
            name=f"deck_front_leaf_{i}",
        )
    deck.visual(
        Cylinder(radius=0.009, length=1.56),
        origin=Origin(xyz=(deck_len, 0.0, 0.0), rpy=cyl_to_y.rpy),
        material=hinge_steel,
        name="front_hinge_pin",
    )

    lip = model.part("lip")
    lip.visual(
        Box((0.44, 1.70, 0.050)),
        origin=Origin(xyz=(0.25, 0.0, 0.065)),
        material=steel_yellow,
        name="lip_plate",
    )
    lip.visual(
        Cylinder(radius=0.018, length=1.66),
        origin=Origin(xyz=(0.48, 0.0, 0.052), rpy=cyl_to_y.rpy),
        material=dark_steel,
        name="rounded_nose",
    )
    for i, x in enumerate((0.14, 0.27, 0.40)):
        lip.visual(
            Box((0.022, 1.46, 0.007)),
            origin=Origin(xyz=(x, 0.0, deck_top_z + 0.0025)),
            material=dark_steel,
            name=f"lip_tread_{i}",
        )
    for i, y in enumerate((-0.31, 0.31)):
        lip.visual(
            Cylinder(radius=0.019, length=0.24),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=cyl_to_y.rpy),
            material=hinge_steel,
            name=f"lip_hinge_barrel_{i}",
        )
        lip.visual(
            Box((0.20, 0.24, 0.022)),
            origin=Origin(xyz=(0.10, y, 0.029)),
            material=dark_steel,
            name=f"lip_hinge_leaf_{i}",
        )

    model.articulation(
        "dock_to_deck",
        ArticulationType.REVOLUTE,
        parent=dock,
        child=deck,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25000.0, velocity=0.45, lower=-0.08, upper=0.45),
    )
    model.articulation(
        "deck_to_lip",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=lip,
        origin=Origin(xyz=(deck_len, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8000.0, velocity=0.8, lower=-1.05, upper=0.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    dock = object_model.get_part("dock")
    deck = object_model.get_part("deck")
    lip = object_model.get_part("lip")
    deck_hinge = object_model.get_articulation("dock_to_deck")
    lip_hinge = object_model.get_articulation("deck_to_lip")

    for barrel in ("deck_rear_barrel_0", "deck_rear_barrel_1"):
        ctx.allow_overlap(
            dock,
            deck,
            elem_a="rear_hinge_pin",
            elem_b=barrel,
            reason="The rear hinge pin is intentionally captured inside the deck hinge barrel.",
        )
        ctx.expect_within(
            dock,
            deck,
            axes="xz",
            inner_elem="rear_hinge_pin",
            outer_elem=barrel,
            margin=0.001,
            name=f"rear pin sits inside {barrel}",
        )
        ctx.expect_overlap(
            dock,
            deck,
            axes="y",
            elem_a="rear_hinge_pin",
            elem_b=barrel,
            min_overlap=0.20,
            name=f"rear pin passes through {barrel}",
        )

    for barrel in ("lip_hinge_barrel_0", "lip_hinge_barrel_1"):
        ctx.allow_overlap(
            deck,
            lip,
            elem_a="front_hinge_pin",
            elem_b=barrel,
            reason="The front hinge pin is intentionally captured inside the lip hinge barrel.",
        )
        ctx.expect_within(
            deck,
            lip,
            axes="xz",
            inner_elem="front_hinge_pin",
            outer_elem=barrel,
            margin=0.001,
            name=f"front pin sits inside {barrel}",
        )
        ctx.expect_overlap(
            deck,
            lip,
            axes="y",
            elem_a="front_hinge_pin",
            elem_b=barrel,
            min_overlap=0.20,
            name=f"front pin passes through {barrel}",
        )

    ctx.check(
        "deck hinge is revolute",
        deck_hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"deck hinge type={deck_hinge.articulation_type}",
    )
    ctx.check(
        "lip hinge is revolute",
        lip_hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"lip hinge type={lip_hinge.articulation_type}",
    )
    ctx.expect_gap(
        deck,
        dock,
        axis="x",
        positive_elem="deck_plate",
        negative_elem="steel_curb",
        min_gap=0.015,
        max_gap=0.070,
        name="deck sits just forward of dock curb",
    )
    ctx.expect_overlap(
        deck,
        dock,
        axes="y",
        elem_a="deck_plate",
        elem_b="dock_face",
        min_overlap=1.70,
        name="deck spans the dock opening width",
    )
    ctx.expect_gap(
        lip,
        deck,
        axis="x",
        positive_elem="lip_plate",
        negative_elem="deck_plate",
        min_gap=0.015,
        max_gap=0.040,
        name="lip plate begins just beyond deck front edge",
    )
    ctx.expect_overlap(
        lip,
        deck,
        axes="y",
        elem_a="lip_plate",
        elem_b="deck_plate",
        min_overlap=1.65,
        name="lip spans nearly the full deck width",
    )

    rest_lip_origin = ctx.part_world_position(lip)
    with ctx.pose({deck_hinge: 0.35}):
        raised_lip_origin = ctx.part_world_position(lip)
    ctx.check(
        "deck hinge raises front edge",
        rest_lip_origin is not None
        and raised_lip_origin is not None
        and raised_lip_origin[2] > rest_lip_origin[2] + 0.30,
        details=f"rest={rest_lip_origin}, raised={raised_lip_origin}",
    )

    rest_lip_aabb = ctx.part_element_world_aabb(lip, elem="lip_plate")
    with ctx.pose({lip_hinge: 0.35}):
        lifted_lip_aabb = ctx.part_element_world_aabb(lip, elem="lip_plate")
    with ctx.pose({lip_hinge: -0.85}):
        lowered_lip_aabb = ctx.part_element_world_aabb(lip, elem="lip_plate")
    ctx.check(
        "lip hinge lifts bridge plate",
        rest_lip_aabb is not None
        and lifted_lip_aabb is not None
        and lifted_lip_aabb[1][2] > rest_lip_aabb[1][2] + 0.12,
        details=f"rest={rest_lip_aabb}, lifted={lifted_lip_aabb}",
    )
    ctx.check(
        "lip hinge folds plate downward",
        rest_lip_aabb is not None
        and lowered_lip_aabb is not None
        and lowered_lip_aabb[0][2] < rest_lip_aabb[0][2] - 0.25,
        details=f"rest={rest_lip_aabb}, lowered={lowered_lip_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
