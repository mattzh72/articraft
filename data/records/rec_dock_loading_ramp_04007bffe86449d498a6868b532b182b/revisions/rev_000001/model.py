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
    model = ArticulatedObject(name="hydraulic_dock_leveler")

    concrete = model.material("pitted_concrete", rgba=(0.46, 0.46, 0.43, 1.0))
    dark_concrete = model.material("shadowed_pit_concrete", rgba=(0.23, 0.24, 0.24, 1.0))
    steel = model.material("painted_steel_deck", rgba=(0.31, 0.34, 0.36, 1.0))
    dark_steel = model.material("dark_structural_steel", rgba=(0.10, 0.12, 0.13, 1.0))
    safety_yellow = model.material("safety_yellow_edges", rgba=(0.95, 0.68, 0.05, 1.0))
    hydraulic_chrome = model.material("polished_hydraulic_rod", rgba=(0.78, 0.82, 0.82, 1.0))
    hose_black = model.material("black_rubber_hose", rgba=(0.015, 0.015, 0.012, 1.0))

    frame = model.part("dock_frame")
    frame.visual(
        Box((2.75, 2.62, 0.16)),
        origin=Origin(xyz=(1.22, 0.0, 0.08)),
        material=dark_concrete,
        name="pit_base",
    )
    frame.visual(
        Box((2.80, 0.16, 0.92)),
        origin=Origin(xyz=(1.22, 1.28, 0.50)),
        material=concrete,
        name="side_wall_0",
    )
    frame.visual(
        Box((2.80, 0.16, 0.92)),
        origin=Origin(xyz=(1.22, -1.28, 0.50)),
        material=concrete,
        name="side_wall_1",
    )
    frame.visual(
        Box((0.26, 2.78, 1.02)),
        origin=Origin(xyz=(-0.22, 0.0, 0.51)),
        material=concrete,
        name="rear_dock_wall",
    )
    frame.visual(
        Box((0.80, 2.82, 0.10)),
        origin=Origin(xyz=(-0.56, 0.0, 1.045)),
        material=concrete,
        name="dock_floor_slab",
    )
    frame.visual(
        Box((0.12, 2.38, 0.08)),
        origin=Origin(xyz=(-0.04, 0.0, 0.91)),
        material=dark_steel,
        name="rear_steel_header",
    )
    frame.visual(
        Box((0.14, 0.10, 0.20)),
        origin=Origin(xyz=(0.00, 1.13, 1.00)),
        material=dark_steel,
        name="rear_hinge_cheek_0",
    )
    frame.visual(
        Box((0.14, 0.10, 0.20)),
        origin=Origin(xyz=(0.00, -1.13, 1.00)),
        material=dark_steel,
        name="rear_hinge_cheek_1",
    )
    frame.visual(
        Box((0.10, 0.24, 0.14)),
        origin=Origin(xyz=(0.58, -0.45, 0.23)),
        material=dark_steel,
        name="ram_mount_pedestal",
    )
    frame.visual(
        Box((0.12, 0.38, 0.04)),
        origin=Origin(xyz=(0.58, -0.45, 0.30)),
        material=dark_steel,
        name="hydraulic_floor_mount",
    )
    frame.visual(
        Cylinder(radius=0.025, length=0.38),
        origin=Origin(xyz=(0.58, -0.45, 0.40), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="lower_pin",
    )
    frame.visual(
        Box((0.06, 0.03, 0.14)),
        origin=Origin(xyz=(0.58, -0.61, 0.38)),
        material=dark_steel,
        name="ram_base_lug_0",
    )
    frame.visual(
        Box((0.06, 0.03, 0.14)),
        origin=Origin(xyz=(0.58, -0.29, 0.38)),
        material=dark_steel,
        name="ram_base_lug_1",
    )

    deck = model.part("deck")
    deck.visual(
        Box((2.40, 2.10, 0.08)),
        origin=Origin(xyz=(1.20, 0.0, 0.04)),
        material=steel,
        name="deck_plate",
    )
    deck.visual(
        Cylinder(radius=0.045, length=1.92),
        origin=Origin(xyz=(-0.035, 0.0, 0.00), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="rear_hinge_tube",
    )
    deck.visual(
        Box((2.30, 0.08, 0.15)),
        origin=Origin(xyz=(1.17, 0.99, -0.065)),
        material=dark_steel,
        name="side_beam_0",
    )
    deck.visual(
        Box((2.30, 0.08, 0.15)),
        origin=Origin(xyz=(1.17, -0.99, -0.065)),
        material=dark_steel,
        name="side_beam_1",
    )
    for idx, x in enumerate((0.45, 0.90, 1.35, 1.80, 2.20)):
        deck.visual(
            Box((0.055, 1.82, 0.11)),
            origin=Origin(xyz=(x, 0.0, -0.045)),
            material=dark_steel,
            name=f"cross_stiffener_{idx}",
        )
    for idx, x in enumerate((0.28, 0.58, 0.88, 1.18, 1.48, 1.78, 2.08)):
        deck.visual(
            Box((0.035, 1.82, 0.012)),
            origin=Origin(xyz=(x, 0.0, 0.083)),
            material=dark_steel,
            name=f"traction_bar_{idx}",
        )
    deck.visual(
        Box((2.30, 0.035, 0.018)),
        origin=Origin(xyz=(1.18, 1.02, 0.087)),
        material=safety_yellow,
        name="edge_stripe_0",
    )
    deck.visual(
        Box((2.30, 0.035, 0.018)),
        origin=Origin(xyz=(1.18, -1.02, 0.087)),
        material=safety_yellow,
        name="edge_stripe_1",
    )
    deck.visual(
        Box((0.22, 0.24, 0.026)),
        origin=Origin(xyz=(1.50, -0.45, -0.013)),
        material=dark_steel,
        name="ram_upper_pad",
    )
    deck.visual(
        Box((0.045, 0.035, 0.28)),
        origin=Origin(xyz=(1.50, -0.58, -0.140)),
        material=dark_steel,
        name="ram_upper_lug_0",
    )
    deck.visual(
        Box((0.045, 0.035, 0.28)),
        origin=Origin(xyz=(1.50, -0.32, -0.140)),
        material=dark_steel,
        name="ram_upper_lug_1",
    )

    lip = model.part("lip")
    lip.visual(
        Box((0.62, 2.00, 0.070)),
        origin=Origin(xyz=(0.31, 0.0, 0.045)),
        material=steel,
        name="lip_plate",
    )
    lip.visual(
        Box((0.52, 1.90, 0.012)),
        origin=Origin(xyz=(0.34, 0.0, 0.083)),
        material=dark_steel,
        name="lip_grip_crest",
    )
    lip.visual(
        Box((0.080, 2.00, 0.032)),
        origin=Origin(xyz=(0.64, 0.0, 0.024)),
        material=safety_yellow,
        name="lip_toe_edge",
    )
    lip.visual(
        Box((0.14, 1.94, 0.026)),
        origin=Origin(xyz=(0.09, 0.0, 0.003)),
        material=dark_steel,
        name="front_hinge_leaf",
    )
    lip.visual(
        Cylinder(radius=0.026, length=1.88),
        origin=Origin(xyz=(0.06, 0.0, -0.026), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="front_hinge_tube",
    )

    ram = model.part("hydraulic_ram")
    ram_angle = math.atan2(0.92, 0.43)
    ram.visual(
        Cylinder(radius=0.072, length=0.50),
        origin=Origin(xyz=(0.335, 0.0, 0.157), rpy=(0.0, ram_angle, 0.0)),
        material=dark_steel,
        name="cylinder_body",
    )
    ram.visual(
        Cylinder(radius=0.034, length=0.50),
        origin=Origin(xyz=(0.680, 0.0, 0.318), rpy=(0.0, ram_angle, 0.0)),
        material=hydraulic_chrome,
        name="chrome_rod",
    )
    ram.visual(
        Cylinder(radius=0.024, length=0.18),
        origin=Origin(xyz=(0.136, 0.0, 0.064), rpy=(0.0, ram_angle, 0.0)),
        material=dark_steel,
        name="lower_neck",
    )
    ram.visual(
        Box((0.14, 0.12, 0.09)),
        origin=Origin(xyz=(0.11, 0.0, 0.045)),
        material=dark_steel,
        name="lower_clevis_web",
    )
    ram.visual(
        Cylinder(radius=0.060, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="lower_eye",
    )
    ram.visual(
        Cylinder(radius=0.045, length=0.18),
        origin=Origin(xyz=(0.92, 0.0, 0.43), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="upper_eye",
    )
    ram.visual(
        Cylinder(radius=0.014, length=0.36),
        origin=Origin(xyz=(0.335, 0.070, 0.157), rpy=(0.0, ram_angle, 0.0)),
        material=hose_black,
        name="hydraulic_hose",
    )

    model.articulation(
        "frame_to_deck",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=deck,
        origin=Origin(xyz=(0.0, 0.0, 1.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8000.0, velocity=0.35, lower=0.0, upper=0.35),
    )
    model.articulation(
        "deck_to_lip",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=lip,
        origin=Origin(xyz=(2.40, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1600.0, velocity=0.8, lower=0.0, upper=1.35),
    )
    model.articulation(
        "frame_to_hydraulic_ram",
        ArticulationType.FIXED,
        parent=frame,
        child=ram,
        origin=Origin(xyz=(0.58, -0.45, 0.40)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("dock_frame")
    deck = object_model.get_part("deck")
    lip = object_model.get_part("lip")
    ram = object_model.get_part("hydraulic_ram")
    deck_hinge = object_model.get_articulation("frame_to_deck")
    lip_hinge = object_model.get_articulation("deck_to_lip")

    ctx.allow_overlap(
        frame,
        ram,
        elem_a="lower_pin",
        elem_b="lower_eye",
        reason="The hydraulic cylinder base eye is intentionally captured around the fixed floor pin.",
    )
    ctx.expect_overlap(
        frame,
        ram,
        axes="xyz",
        elem_a="lower_pin",
        elem_b="lower_eye",
        min_overlap=0.020,
        name="hydraulic base eye is retained on the floor pin",
    )

    ctx.expect_within(
        deck,
        frame,
        axes="y",
        inner_elem="deck_plate",
        outer_elem="pit_base",
        name="deck sits between pit side walls",
    )
    ctx.expect_gap(
        lip,
        deck,
        axis="x",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="lip_plate",
        negative_elem="deck_plate",
        name="folding lip hinges at the front deck edge",
    )

    rest_lip_position = ctx.part_world_position(lip)
    with ctx.pose({deck_hinge: 0.35}):
        tilted_lip_position = ctx.part_world_position(lip)

    ctx.check(
        "rear hinge tilts the steel deck upward",
        rest_lip_position is not None
        and tilted_lip_position is not None
        and tilted_lip_position[2] > rest_lip_position[2] + 0.70,
        details=f"rest_lip_position={rest_lip_position}, tilted_lip_position={tilted_lip_position}",
    )

    rest_lip_aabb = ctx.part_world_aabb(lip)
    with ctx.pose({lip_hinge: 1.35}):
        folded_lip_aabb = ctx.part_world_aabb(lip)

    ctx.check(
        "front hinge pivots the lip downward",
        rest_lip_aabb is not None
        and folded_lip_aabb is not None
        and folded_lip_aabb[0][2] < rest_lip_aabb[0][2] - 0.45,
        details=f"rest_lip_aabb={rest_lip_aabb}, folded_lip_aabb={folded_lip_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
