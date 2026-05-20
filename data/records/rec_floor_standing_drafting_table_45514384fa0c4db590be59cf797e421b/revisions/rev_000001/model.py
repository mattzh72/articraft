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
    model = ArticulatedObject(name="floor_standing_drafting_table")

    powder_coat = Material("satin_black_powder_coat", color=(0.02, 0.024, 0.025, 1.0))
    dark_steel = Material("dark_burnished_steel", color=(0.09, 0.095, 0.10, 1.0))
    rubber = Material("matte_black_rubber", color=(0.008, 0.008, 0.007, 1.0))
    maple = Material("sealed_light_maple", color=(0.72, 0.55, 0.34, 1.0))
    maple_edge = Material("darker_maple_edge_banding", color=(0.48, 0.32, 0.17, 1.0))
    aluminum = Material("brushed_aluminum", color=(0.72, 0.74, 0.72, 1.0))
    graphite = Material("knurled_graphite_plastic", color=(0.03, 0.032, 0.035, 1.0))
    paper = Material("warm_white_drafting_surface", color=(0.86, 0.82, 0.73, 1.0))

    stand = model.part("stand")
    # Floor footprint: a rigid H base with rubber shoes.
    stand.visual(
        Box((0.10, 0.92, 0.055)),
        origin=Origin(xyz=(-0.46, 0.0, 0.0275)),
        material=powder_coat,
        name="base_rail_0",
    )
    stand.visual(
        Box((0.10, 0.92, 0.055)),
        origin=Origin(xyz=(0.46, 0.0, 0.0275)),
        material=powder_coat,
        name="base_rail_1",
    )
    stand.visual(
        Box((1.04, 0.075, 0.055)),
        origin=Origin(xyz=(0.0, -0.36, 0.0375)),
        material=powder_coat,
        name="front_floor_tie",
    )
    stand.visual(
        Box((1.04, 0.075, 0.055)),
        origin=Origin(xyz=(0.0, 0.36, 0.0375)),
        material=powder_coat,
        name="rear_floor_tie",
    )
    stand.visual(
        Box((0.14, 0.80, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0375)),
        material=powder_coat,
        name="base_spine",
    )
    for index, x in enumerate((-0.46, 0.46)):
        for y in (-0.43, 0.43):
            stand.visual(
                Box((0.14, 0.075, 0.018)),
                origin=Origin(xyz=(x, y, 0.009)),
                material=rubber,
                name=f"rubber_foot_{index}_{0 if y < 0 else 1}",
            )

    # Pedestal and upper cantilever that carry the board hinge.
    stand.visual(
        Box((0.16, 0.13, 0.70)),
        origin=Origin(xyz=(0.0, 0.0, 0.405)),
        material=powder_coat,
        name="outer_pedestal",
    )
    stand.visual(
        Box((0.115, 0.095, 0.34)),
        origin=Origin(xyz=(0.0, -0.035, 0.755)),
        material=dark_steel,
        name="inner_column",
    )
    stand.visual(
        Box((0.18, 0.42, 0.070)),
        origin=Origin(xyz=(0.0, -0.155, 0.745)),
        material=powder_coat,
        name="upper_cantilever",
    )
    stand.visual(
        Box((1.34, 0.095, 0.055)),
        origin=Origin(xyz=(0.0, -0.28, 0.825)),
        material=powder_coat,
        name="pivot_support_beam",
    )
    stand.visual(
        Box((0.18, 0.12, 0.080)),
        origin=Origin(xyz=(0.0, -0.28, 0.790)),
        material=powder_coat,
        name="cantilever_neck",
    )

    # Two fixed yoke frames surround the tilt axis; their central apertures
    # leave visible clearance for the rotating sleeve and side lock knobs.
    for index, x in enumerate((-0.64, 0.64)):
        stand.visual(
            Box((0.060, 0.185, 0.034)),
            origin=Origin(xyz=(x, -0.28, 1.012)),
            material=dark_steel,
            name=f"yoke_top_{index}",
        )
        stand.visual(
            Box((0.060, 0.185, 0.034)),
            origin=Origin(xyz=(x, -0.28, 0.835)),
            material=dark_steel,
            name=f"yoke_bottom_{index}",
        )
        stand.visual(
            Box((0.060, 0.034, 0.168)),
            origin=Origin(xyz=(x, -0.354, 0.93)),
            material=dark_steel,
            name=f"yoke_front_{index}",
        )
        stand.visual(
            Box((0.060, 0.034, 0.168)),
            origin=Origin(xyz=(x, -0.206, 0.93)),
            material=dark_steel,
            name=f"yoke_rear_{index}",
        )

    # An angle index plate and pointer make the tilt-adjustment hardware legible.
    stand.visual(
        Box((0.018, 0.26, 0.16)),
        origin=Origin(xyz=(0.678, -0.075, 0.845)),
        material=dark_steel,
        name="angle_index_plate",
    )
    for i, z in enumerate((0.790, 0.820, 0.850, 0.880, 0.910)):
        stand.visual(
            Box((0.022, 0.042 if i % 2 == 0 else 0.030, 0.006)),
            origin=Origin(xyz=(0.690, -0.034, z)),
            material=aluminum,
            name=f"angle_tick_{i}",
        )

    tabletop = model.part("tabletop")
    tabletop.visual(
        Cylinder(radius=0.025, length=1.16),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="hinge_sleeve",
    )
    tabletop.visual(
        Box((1.16, 0.050, 0.040)),
        origin=Origin(xyz=(0.0, 0.030, 0.025)),
        material=aluminum,
        name="front_hinge_apron",
    )
    tabletop.visual(
        Box((1.20, 0.82, 0.050)),
        origin=Origin(xyz=(0.0, 0.435, 0.060)),
        material=maple,
        name="drawing_board",
    )
    tabletop.visual(
        Box((1.22, 0.026, 0.074)),
        origin=Origin(xyz=(0.0, 0.025, 0.086)),
        material=maple_edge,
        name="front_pencil_lip",
    )
    tabletop.visual(
        Box((1.20, 0.030, 0.030)),
        origin=Origin(xyz=(0.0, 0.835, 0.065)),
        material=maple_edge,
        name="rear_edge_band",
    )
    tabletop.visual(
        Box((0.030, 0.67, 0.030)),
        origin=Origin(xyz=(-0.615, 0.500, 0.065)),
        material=maple_edge,
        name="side_edge_0",
    )
    tabletop.visual(
        Box((0.030, 0.67, 0.030)),
        origin=Origin(xyz=(0.615, 0.500, 0.065)),
        material=maple_edge,
        name="side_edge_1",
    )
    tabletop.visual(
        Box((1.08, 0.030, 0.018)),
        origin=Origin(xyz=(0.0, 0.135, 0.092)),
        material=paper,
        name="paper_stop_strip",
    )
    tabletop.visual(
        Box((1.02, 0.026, 0.024)),
        origin=Origin(xyz=(0.0, 0.655, 0.095)),
        material=aluminum,
        name="parallel_rule_bar",
    )
    tabletop.visual(
        Box((0.030, 0.68, 0.020)),
        origin=Origin(xyz=(-0.535, 0.475, 0.091)),
        material=aluminum,
        name="rule_track_0",
    )
    tabletop.visual(
        Box((0.030, 0.68, 0.020)),
        origin=Origin(xyz=(0.535, 0.475, 0.091)),
        material=aluminum,
        name="rule_track_1",
    )
    tabletop.visual(
        Box((0.090, 0.58, 0.035)),
        origin=Origin(xyz=(-0.36, 0.465, 0.025)),
        material=dark_steel,
        name="underside_rib_0",
    )
    tabletop.visual(
        Box((0.090, 0.58, 0.035)),
        origin=Origin(xyz=(0.36, 0.465, 0.025)),
        material=dark_steel,
        name="underside_rib_1",
    )

    tilt_joint = model.articulation(
        "stand_to_tabletop",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=tabletop,
        # The child frame is the front lower hinge axis; positive rotation
        # raises the rear of the drawing board.
        origin=Origin(xyz=(0.0, -0.28, 0.93)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.7, lower=0.0, upper=1.10),
    )

    for index, sign in enumerate((1.0, -1.0)):
        knob = model.part(f"lock_knob_{index}")
        knob.visual(
            Cylinder(radius=0.058, length=0.045),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=graphite,
            name="grip_disk",
        )
        knob.visual(
            Cylinder(radius=0.016, length=0.080),
            origin=Origin(
                xyz=(-0.060 * sign, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=dark_steel,
            name="short_stem",
        )
        # Shallow proud ribs make the control read as a knurled locking knob.
        for rib_i, angle in enumerate((0.0, math.pi / 3.0, 2.0 * math.pi / 3.0)):
            y = 0.045 * math.cos(angle)
            z = 0.045 * math.sin(angle)
            knob.visual(
                Box((0.048, 0.009, 0.014)),
                origin=Origin(xyz=(0.0, y, z), rpy=(0.0, 0.0, angle)),
                material=graphite,
                name=f"grip_rib_{rib_i}",
            )
        model.articulation(
            f"tabletop_to_lock_knob_{index}",
            ArticulationType.CONTINUOUS,
            parent=tabletop,
            child=knob,
            origin=Origin(xyz=(0.680 * sign, 0.0, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.5, velocity=8.0),
        )

    # Keep a handle to silence accidental lints while making the joint contract
    # explicit in the script above.
    _ = tilt_joint
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    tabletop = object_model.get_part("tabletop")
    knob_0 = object_model.get_part("lock_knob_0")
    knob_1 = object_model.get_part("lock_knob_1")
    tilt = object_model.get_articulation("stand_to_tabletop")

    # The table must be floor-standing, not a miniature desk object.
    stand_aabb = ctx.part_world_aabb(stand)
    board_aabb = ctx.part_element_world_aabb(tabletop, elem="drawing_board")
    ctx.check(
        "full size drafting table proportions",
        stand_aabb is not None
        and board_aabb is not None
        and stand_aabb[0][2] <= 0.002
        and board_aabb[1][2] > 0.95
        and (board_aabb[1][0] - board_aabb[0][0]) > 1.15
        and (board_aabb[1][1] - board_aabb[0][1]) > 0.78,
        details=f"stand_aabb={stand_aabb}, board_aabb={board_aabb}",
    )

    # Side lock knobs are separate rotating controls with stems seated against
    # the tabletop hinge sleeve.
    ctx.expect_gap(
        knob_0,
        tabletop,
        axis="x",
        positive_elem="short_stem",
        negative_elem="hinge_sleeve",
        min_gap=-0.001,
        max_gap=0.003,
        name="positive side knob stem meets hinge sleeve",
    )
    ctx.expect_gap(
        tabletop,
        knob_1,
        axis="x",
        positive_elem="hinge_sleeve",
        negative_elem="short_stem",
        min_gap=-0.001,
        max_gap=0.003,
        name="negative side knob stem meets hinge sleeve",
    )

    with ctx.pose({tilt: 0.0}):
        flat_board = ctx.part_element_world_aabb(tabletop, elem="drawing_board")
    with ctx.pose({tilt: 1.10}):
        tilted_board = ctx.part_element_world_aabb(tabletop, elem="drawing_board")
    ctx.check(
        "tilt joint raises rear of drawing surface",
        flat_board is not None
        and tilted_board is not None
        and tilted_board[1][2] > flat_board[1][2] + 0.55
        and tilted_board[0][2] > 0.90,
        details=f"flat_board={flat_board}, tilted_board={tilted_board}",
    )

    return ctx.report()


object_model = build_object_model()
