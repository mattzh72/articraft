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
    model = ArticulatedObject(name="hydraulic_truck_bed_ramp")

    galvanized = model.material("galvanized_steel", rgba=(0.48, 0.52, 0.54, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    rib_steel = model.material("raised_rib_steel", rgba=(0.34, 0.36, 0.37, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(1.0, 0.72, 0.05, 1.0))
    polished = model.material("polished_hydraulic_rod", rgba=(0.82, 0.86, 0.88, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))

    deck_length = 2.80
    deck_width = 1.10
    deck_thickness = 0.08
    deck_pitch = -math.radians(10.0)
    deck_center = (1.40, 0.0, 0.65)
    cp = math.cos(deck_pitch)
    sp = math.sin(deck_pitch)

    def deck_point(local_x: float, local_y: float, local_z: float) -> tuple[float, float, float]:
        """Map a point from the inclined deck's local frame into the root frame."""

        return (
            deck_center[0] + cp * local_x + sp * local_z,
            local_y,
            deck_center[2] - sp * local_x + cp * local_z,
        )

    def deck_origin(local_x: float, local_y: float, local_z: float) -> Origin:
        return Origin(xyz=deck_point(local_x, local_y, local_z), rpy=(0.0, deck_pitch, 0.0))

    deck = model.part("deck")
    deck.visual(
        Box((deck_length, deck_width, deck_thickness)),
        origin=deck_origin(0.0, 0.0, 0.0),
        material=galvanized,
        name="inclined_deck_slab",
    )

    # Raised steel ribs welded to the flat ramp plate.
    rib_height = 0.025
    for index, y in enumerate((-0.42, -0.28, -0.14, 0.0, 0.14, 0.28, 0.42)):
        deck.visual(
            Box((deck_length * 0.92, 0.035, rib_height)),
            origin=deck_origin(0.0, y, deck_thickness / 2.0 + rib_height / 2.0),
            material=rib_steel,
            name=f"long_rib_{index}",
        )
    for index, x in enumerate((-0.95, -0.35, 0.35, 0.95)):
        deck.visual(
            Box((0.045, deck_width * 0.92, 0.020)),
            origin=deck_origin(x, 0.0, deck_thickness / 2.0 + 0.010),
            material=rib_steel,
            name=f"cross_rib_{index}",
        )

    # Low yellow curb rails make the sides and front edge legible without turning
    # the flat deck into a boxed tray.
    for suffix, y in (("0", -deck_width / 2.0 - 0.028), ("1", deck_width / 2.0 + 0.028)):
        deck.visual(
            Box((deck_length, 0.055, 0.13)),
            origin=deck_origin(0.0, y, deck_thickness / 2.0 + 0.065),
            material=safety_yellow,
            name=f"side_curb_{suffix}",
        )

    # Underside crossmembers and the rear truck-bed lip are part of the welded
    # ramp frame.
    for index, x in enumerate((-1.05, -0.20, 0.65, 1.18)):
        deck.visual(
            Box((0.080, deck_width + 0.18, 0.085)),
            origin=deck_origin(x, 0.0, -deck_thickness / 2.0 - 0.035),
            material=dark_steel,
            name=f"underside_crossmember_{index}",
        )
    deck.visual(
        Box((0.11, deck_width + 0.12, 0.16)),
        origin=deck_origin(deck_length / 2.0 - 0.035, 0.0, deck_thickness / 2.0 + 0.080),
        material=safety_yellow,
        name="truck_bed_stop_lip",
    )

    # Telescoping square-tube sleeves fixed under the ramp.  Each sleeve is four
    # separate walls around a real central clearance for the moving chrome post.
    post_local_x = 0.58
    post_ys = (-0.36, 0.36)
    sleeve_inner = 0.135
    sleeve_wall = 0.025
    sleeve_outer = sleeve_inner + 2.0 * sleeve_wall
    sleeve_len = 0.52
    sleeve_bottoms: list[float] = []
    for post_index, post_y in enumerate(post_ys):
        post_x, _, underside_z = deck_point(post_local_x, post_y, -deck_thickness / 2.0)
        sleeve_top = underside_z + 0.015
        sleeve_bottom = sleeve_top - sleeve_len
        sleeve_bottoms.append(sleeve_bottom)
        sleeve_center_z = (sleeve_top + sleeve_bottom) / 2.0
        suffix = str(post_index)
        deck.visual(
            Box((sleeve_wall, sleeve_outer, sleeve_len)),
            origin=Origin(xyz=(post_x + sleeve_inner / 2.0 + sleeve_wall / 2.0, post_y, sleeve_center_z)),
            material=dark_steel,
            name=f"post_{suffix}_x_wall_pos",
        )
        deck.visual(
            Box((sleeve_wall, sleeve_outer, sleeve_len)),
            origin=Origin(xyz=(post_x - sleeve_inner / 2.0 - sleeve_wall / 2.0, post_y, sleeve_center_z)),
            material=dark_steel,
            name=f"post_{suffix}_x_wall_neg",
        )
        deck.visual(
            Box((sleeve_inner, sleeve_wall, sleeve_len)),
            origin=Origin(xyz=(post_x, post_y + sleeve_inner / 2.0 + sleeve_wall / 2.0, sleeve_center_z)),
            material=dark_steel,
            name=f"post_{suffix}_y_wall_pos",
        )
        deck.visual(
            Box((sleeve_inner, sleeve_wall, sleeve_len)),
            origin=Origin(xyz=(post_x, post_y - sleeve_inner / 2.0 - sleeve_wall / 2.0, sleeve_center_z)),
            material=dark_steel,
            name=f"post_{suffix}_y_wall_neg",
        )
        # Side saddle pads connect the vertical sleeve walls to the sloped deck
        # underside without blocking the central telescoping clearance.
        deck.visual(
            Box((0.30, 0.050, 0.050)),
            origin=Origin(xyz=(post_x, post_y + sleeve_outer / 2.0 + 0.025, sleeve_top - 0.020)),
            material=dark_steel,
            name=f"post_{suffix}_saddle_pos",
        )
        deck.visual(
            Box((0.30, 0.050, 0.050)),
            origin=Origin(xyz=(post_x, post_y - sleeve_outer / 2.0 - 0.025, sleeve_top - 0.020)),
            material=dark_steel,
            name=f"post_{suffix}_saddle_neg",
        )

    # Alternating hinge barrels mounted along the low/front edge of the deck.
    hinge_x, _, hinge_z = deck_point(-deck_length / 2.0 - 0.020, 0.0, deck_thickness / 2.0 + 0.035)
    for index, (y, length) in enumerate(((-0.45, 0.18), (0.0, 0.22), (0.45, 0.18))):
        deck.visual(
            Cylinder(radius=0.035, length=length),
            origin=Origin(xyz=(hinge_x, y, hinge_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"deck_hinge_barrel_{index}",
        )
        deck.visual(
            Box((0.18, length, 0.030)),
            origin=Origin(xyz=(hinge_x + 0.055, y, hinge_z - 0.030)),
            material=dark_steel,
            name=f"deck_hinge_strap_{index}",
        )

    leg_parts = []
    for post_index, post_y in enumerate(post_ys):
        post_x, _, _ = deck_point(post_local_x, post_y, -deck_thickness / 2.0)
        leg = model.part(f"leg_post_{post_index}")
        leg.visual(
            Cylinder(radius=0.055, length=1.00),
            origin=Origin(xyz=(0.0, 0.0, -0.080)),
            material=polished,
            name="inner_rod",
        )
        pad_thick = 0.014
        pad_span = 0.085
        pad_len = 0.070
        pad_offset = sleeve_inner / 2.0 - pad_thick / 2.0
        for level_index, z in enumerate((0.050, 0.320)):
            leg.visual(
                Box((pad_thick, pad_span, pad_len)),
                origin=Origin(xyz=(pad_offset, 0.0, z)),
                material=dark_steel,
                name=f"guide_pad_x_pos_{level_index}",
            )
            leg.visual(
                Box((pad_thick, pad_span, pad_len)),
                origin=Origin(xyz=(-pad_offset, 0.0, z)),
                material=dark_steel,
                name=f"guide_pad_x_neg_{level_index}",
            )
            leg.visual(
                Box((pad_span, pad_thick, pad_len)),
                origin=Origin(xyz=(0.0, pad_offset, z)),
                material=dark_steel,
                name=f"guide_pad_y_pos_{level_index}",
            )
            leg.visual(
                Box((pad_span, pad_thick, pad_len)),
                origin=Origin(xyz=(0.0, -pad_offset, z)),
                material=dark_steel,
                name=f"guide_pad_y_neg_{level_index}",
            )
        leg.visual(
            Box((0.36, 0.24, 0.045)),
            origin=Origin(xyz=(0.0, 0.0, -0.6025)),
            material=rubber,
            name="rubber_foot",
        )
        leg_parts.append((leg, post_x, post_y, sleeve_bottoms[post_index]))

    for post_index, (leg, post_x, post_y, sleeve_bottom) in enumerate(leg_parts):
        model.articulation(
            f"deck_to_leg_post_{post_index}",
            ArticulationType.PRISMATIC,
            parent=deck,
            child=leg,
            origin=Origin(xyz=(post_x, post_y, sleeve_bottom)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=2500.0, velocity=0.18, lower=0.0, upper=0.35),
        )

    approach = model.part("approach_plate")
    plate_len = 0.64
    plate_width = 1.02
    plate_thickness = 0.045
    approach_pitch = -math.radians(9.0)
    ca = math.cos(approach_pitch)
    sa = math.sin(approach_pitch)

    def approach_origin(local_x: float, local_y: float, local_z: float) -> Origin:
        return Origin(
            xyz=(ca * local_x + sa * local_z, local_y, -sa * local_x + ca * local_z),
            rpy=(0.0, approach_pitch, 0.0),
        )

    approach.visual(
        Box((plate_len, plate_width, plate_thickness)),
        origin=approach_origin(-plate_len / 2.0 - 0.060, 0.0, -plate_thickness / 2.0),
        material=galvanized,
        name="approach_deck",
    )
    for index, y in enumerate((-0.34, -0.17, 0.0, 0.17, 0.34)):
        approach.visual(
            Box((plate_len * 0.82, 0.028, 0.020)),
            origin=approach_origin(-plate_len / 2.0 - 0.060, y, 0.010),
            material=rib_steel,
            name=f"approach_rib_{index}",
        )
    approach.visual(
        Box((0.045, plate_width, 0.075)),
        origin=approach_origin(-plate_len - 0.060, 0.0, -0.038),
        material=safety_yellow,
        name="front_nose_lip",
    )
    for index, y in enumerate((-0.235, 0.235)):
        approach.visual(
            Box((0.15, 0.12, 0.026)),
            origin=Origin(xyz=(-0.055, y, -0.020)),
            material=dark_steel,
            name=f"approach_hinge_strap_{index}",
        )
    for index, (y, length) in enumerate(((-0.235, 0.25), (0.235, 0.25))):
        approach.visual(
            Cylinder(radius=0.033, length=length),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"approach_hinge_barrel_{index}",
        )

    model.articulation(
        "deck_to_approach_plate",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=approach,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=1.2, lower=0.0, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    approach = object_model.get_part("approach_plate")
    approach_joint = object_model.get_articulation("deck_to_approach_plate")

    ctx.check(
        "deck slab is an inclined plane",
        deck.get_visual("inclined_deck_slab").origin.rpy[1] < -0.10,
        details=f"deck pitch={deck.get_visual('inclined_deck_slab').origin.rpy[1]}",
    )
    ctx.check(
        "approach plate has a revolute hinge",
        approach_joint.articulation_type == ArticulationType.REVOLUTE
        and approach_joint.axis == (0.0, 1.0, 0.0)
        and approach_joint.motion_limits is not None
        and approach_joint.motion_limits.upper >= 1.0,
        details=f"joint={approach_joint}",
    )

    rest_plate_box = ctx.part_element_world_aabb(approach, elem="approach_deck")
    with ctx.pose({approach_joint: 1.0}):
        raised_plate_box = ctx.part_element_world_aabb(approach, elem="approach_deck")
    ctx.check(
        "approach plate folds upward about the front hinge",
        rest_plate_box is not None
        and raised_plate_box is not None
        and raised_plate_box[1][2] > rest_plate_box[1][2] + 0.25,
        details=f"rest={rest_plate_box}, raised={raised_plate_box}",
    )

    for post_index in (0, 1):
        leg = object_model.get_part(f"leg_post_{post_index}")
        joint = object_model.get_articulation(f"deck_to_leg_post_{post_index}")
        ctx.check(
            f"leg post {post_index} uses a downward prismatic joint",
            joint.articulation_type == ArticulationType.PRISMATIC
            and joint.axis == (0.0, 0.0, -1.0)
            and joint.motion_limits is not None
            and joint.motion_limits.upper >= 0.30,
            details=f"joint={joint}",
        )
        ctx.expect_gap(
            deck,
            leg,
            axis="x",
            min_gap=0.003,
            max_gap=0.020,
            positive_elem=f"post_{post_index}_x_wall_pos",
            negative_elem="inner_rod",
            name=f"leg post {post_index} clears positive x sleeve wall",
        )
        ctx.expect_gap(
            leg,
            deck,
            axis="x",
            min_gap=0.003,
            max_gap=0.020,
            positive_elem="inner_rod",
            negative_elem=f"post_{post_index}_x_wall_neg",
            name=f"leg post {post_index} clears negative x sleeve wall",
        )
        ctx.expect_gap(
            deck,
            leg,
            axis="y",
            min_gap=0.003,
            max_gap=0.020,
            positive_elem=f"post_{post_index}_y_wall_pos",
            negative_elem="inner_rod",
            name=f"leg post {post_index} clears positive y sleeve wall",
        )
        ctx.expect_gap(
            leg,
            deck,
            axis="y",
            min_gap=0.003,
            max_gap=0.020,
            positive_elem="inner_rod",
            negative_elem=f"post_{post_index}_y_wall_neg",
            name=f"leg post {post_index} clears negative y sleeve wall",
        )
        ctx.expect_overlap(
            leg,
            deck,
            axes="z",
            min_overlap=0.25,
            elem_a="inner_rod",
            elem_b=f"post_{post_index}_x_wall_pos",
            name=f"leg post {post_index} is retained inside the sleeve at rest",
        )
        rest_pos = ctx.part_world_position(leg)
        with ctx.pose({joint: joint.motion_limits.upper}):
            ctx.expect_overlap(
                leg,
                deck,
                axes="z",
                min_overlap=0.055,
                elem_a="inner_rod",
                elem_b=f"post_{post_index}_x_wall_pos",
                name=f"leg post {post_index} remains retained when extended",
            )
            extended_pos = ctx.part_world_position(leg)
        ctx.check(
            f"leg post {post_index} extends downward",
            rest_pos is not None
            and extended_pos is not None
            and extended_pos[2] < rest_pos[2] - 0.30,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    return ctx.report()


object_model = build_object_model()
