from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="mine_shaft_cage_elevator")

    dark_steel = model.material("dark_blued_steel", rgba=(0.07, 0.075, 0.075, 1.0))
    worn_steel = model.material("worn_rubbed_steel", rgba=(0.32, 0.31, 0.28, 1.0))
    safety_yellow = model.material("chipped_safety_yellow", rgba=(0.95, 0.66, 0.08, 1.0))
    rust_floor = model.material("rusty_checkered_floor", rgba=(0.42, 0.24, 0.13, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))

    shaft = model.part("shaft_frame")
    shaft.visual(Box((0.08, 0.08, 3.40)), origin=Origin(xyz=(-0.72, 0.0, 1.70)), material=dark_steel, name="guide_rail_0")
    shaft.visual(Box((0.08, 0.08, 3.40)), origin=Origin(xyz=(0.72, 0.0, 1.70)), material=dark_steel, name="guide_rail_1")
    shaft.visual(Box((1.58, 0.12, 0.12)), origin=Origin(xyz=(0.0, 0.0, 3.36)), material=dark_steel, name="top_crosshead")
    shaft.visual(Box((1.58, 0.12, 0.12)), origin=Origin(xyz=(0.0, 0.0, 0.06)), material=dark_steel, name="bottom_sill")
    shaft.visual(Box((1.42, 0.07, 0.07)), origin=Origin(xyz=(0.0, 0.62, 1.70)), material=worn_steel, name="rear_tie_bar")
    shaft.visual(Box((0.06, 0.55, 0.06)), origin=Origin(xyz=(-0.72, 0.31, 1.70)), material=worn_steel, name="tie_standoff_0")
    shaft.visual(Box((0.06, 0.55, 0.06)), origin=Origin(xyz=(0.72, 0.31, 1.70)), material=worn_steel, name="tie_standoff_1")
    shaft.visual(Box((0.16, 0.16, 0.10)), origin=Origin(xyz=(-0.72, 0.0, 3.47)), material=worn_steel, name="rail_cap_0")
    shaft.visual(Box((0.16, 0.16, 0.10)), origin=Origin(xyz=(0.72, 0.0, 3.47)), material=worn_steel, name="rail_cap_1")

    cage = model.part("cage")
    cage.visual(Box((1.12, 0.86, 0.08)), origin=Origin(xyz=(0.0, 0.0, 0.04)), material=rust_floor, name="floor_deck")
    cage.visual(Box((0.06, 0.06, 1.78)), origin=Origin(xyz=(-0.56, -0.43, 0.89)), material=safety_yellow, name="corner_post_0")
    cage.visual(Box((0.06, 0.06, 1.78)), origin=Origin(xyz=(0.56, -0.43, 0.89)), material=safety_yellow, name="corner_post_1")
    cage.visual(Box((0.06, 0.06, 1.78)), origin=Origin(xyz=(-0.56, 0.43, 0.89)), material=safety_yellow, name="corner_post_2")
    cage.visual(Box((0.06, 0.06, 1.78)), origin=Origin(xyz=(0.56, 0.43, 0.89)), material=safety_yellow, name="corner_post_3")
    cage.visual(Box((1.18, 0.06, 0.07)), origin=Origin(xyz=(0.0, -0.43, 1.78)), material=safety_yellow, name="front_top_rail")
    cage.visual(Box((1.18, 0.06, 0.07)), origin=Origin(xyz=(0.0, 0.43, 1.78)), material=safety_yellow, name="rear_top_rail")
    cage.visual(Box((0.06, 0.92, 0.07)), origin=Origin(xyz=(-0.56, 0.0, 1.78)), material=safety_yellow, name="side_top_rail_0")
    cage.visual(Box((0.06, 0.92, 0.07)), origin=Origin(xyz=(0.56, 0.0, 1.78)), material=safety_yellow, name="side_top_rail_1")
    cage.visual(Box((1.18, 0.06, 0.07)), origin=Origin(xyz=(0.0, -0.43, 0.11)), material=safety_yellow, name="front_bottom_rail")
    cage.visual(Box((1.18, 0.06, 0.07)), origin=Origin(xyz=(0.0, 0.43, 0.11)), material=safety_yellow, name="rear_bottom_rail")
    cage.visual(Box((0.06, 0.92, 0.07)), origin=Origin(xyz=(-0.56, 0.0, 0.11)), material=safety_yellow, name="side_bottom_rail_0")
    cage.visual(Box((0.06, 0.92, 0.07)), origin=Origin(xyz=(0.56, 0.0, 0.11)), material=safety_yellow, name="side_bottom_rail_1")
    cage.visual(Box((1.10, 0.045, 0.045)), origin=Origin(xyz=(0.0, 0.43, 0.68)), material=safety_yellow, name="rear_mid_rail_0")
    cage.visual(Box((1.10, 0.045, 0.045)), origin=Origin(xyz=(0.0, 0.43, 1.42)), material=safety_yellow, name="rear_mid_rail_1")

    for idx, x in enumerate((-0.36, -0.18, 0.0, 0.18, 0.36)):
        cage.visual(Box((0.028, 0.035, 1.42)), origin=Origin(xyz=(x, 0.43, 0.90)), material=worn_steel, name=f"rear_bar_{idx}")
    for side, x in enumerate((-0.56, 0.56)):
        for level, z in enumerate((0.68, 1.42)):
            cage.visual(Box((0.06, 0.90, 0.045)), origin=Origin(xyz=(x, 0.0, z)), material=safety_yellow, name=f"side_mid_rail_{side}_{level}")
        for idx, y in enumerate((-0.20, 0.05, 0.30)):
            cage.visual(Box((0.028, 0.035, 1.34)), origin=Origin(xyz=(x, y, 0.90)), material=worn_steel, name=f"side_bar_{side}_{idx}")

    cage.visual(Box((2.10, 0.04, 0.045)), origin=Origin(xyz=(0.0, -0.485, 1.55)), material=dark_steel, name="upper_gate_track")
    cage.visual(Box((2.10, 0.04, 0.045)), origin=Origin(xyz=(0.0, -0.485, 0.32)), material=dark_steel, name="lower_gate_track")
    for x in (-0.56, 0.56):
        cage.visual(Box((0.07, 0.09, 0.08)), origin=Origin(xyz=(x, -0.455, 1.55)), material=dark_steel, name=f"upper_track_bracket_{x:+.0f}")
        cage.visual(Box((0.07, 0.09, 0.08)), origin=Origin(xyz=(x, -0.455, 0.32)), material=dark_steel, name=f"lower_track_bracket_{x:+.0f}")

    cage.visual(Box((0.13, 0.08, 0.08)), origin=Origin(xyz=(-0.615, 0.0, 0.68)), material=safety_yellow, name="guide_arm_0_0")
    cage.visual(Box((0.05, 0.18, 0.11)), origin=Origin(xyz=(-0.655, 0.0, 0.68)), material=black_rubber, name="guide_shoe_0_0")
    cage.visual(Box((0.13, 0.08, 0.08)), origin=Origin(xyz=(-0.615, 0.0, 1.42)), material=safety_yellow, name="guide_arm_0_1")
    cage.visual(Box((0.05, 0.18, 0.11)), origin=Origin(xyz=(-0.655, 0.0, 1.42)), material=black_rubber, name="guide_shoe_0_1")
    cage.visual(Box((0.13, 0.08, 0.08)), origin=Origin(xyz=(0.615, 0.0, 0.68)), material=safety_yellow, name="guide_arm_1_0")
    cage.visual(Box((0.05, 0.18, 0.11)), origin=Origin(xyz=(0.655, 0.0, 0.68)), material=black_rubber, name="guide_shoe_1_0")
    cage.visual(Box((0.13, 0.08, 0.08)), origin=Origin(xyz=(0.615, 0.0, 1.42)), material=safety_yellow, name="guide_arm_1_1")
    cage.visual(Box((0.05, 0.18, 0.11)), origin=Origin(xyz=(0.655, 0.0, 1.42)), material=black_rubber, name="guide_shoe_1_1")

    gate = model.part("gate")
    gate.visual(Box((1.02, 0.035, 0.055)), origin=Origin(xyz=(0.0, 0.0, 1.38)), material=dark_steel, name="gate_top_bar")
    gate.visual(Box((1.02, 0.035, 0.055)), origin=Origin(xyz=(0.0, 0.0, 0.46)), material=dark_steel, name="gate_bottom_bar")
    gate.visual(Box((0.045, 0.035, 0.98)), origin=Origin(xyz=(-0.50, 0.0, 0.92)), material=dark_steel, name="gate_side_0")
    gate.visual(Box((0.045, 0.035, 0.98)), origin=Origin(xyz=(0.50, 0.0, 0.92)), material=dark_steel, name="gate_side_1")
    for idx, x in enumerate((-0.30, -0.15, 0.0, 0.15, 0.30)):
        gate.visual(Box((0.026, 0.032, 0.96)), origin=Origin(xyz=(x, 0.0, 0.92)), material=worn_steel, name=f"gate_bar_{idx}")
    gate.visual(Box((0.045, 0.035, 0.15)), origin=Origin(xyz=(-0.42, 0.0, 1.47)), material=dark_steel, name="top_hanger_0")
    gate.visual(Box((0.11, 0.04, 0.055)), origin=Origin(xyz=(-0.42, 0.010, 1.55)), material=black_rubber, name="top_shoe_0")
    gate.visual(Box((0.045, 0.035, 0.15)), origin=Origin(xyz=(-0.42, 0.0, 0.39)), material=dark_steel, name="bottom_hanger_0")
    gate.visual(Box((0.11, 0.04, 0.055)), origin=Origin(xyz=(-0.42, 0.010, 0.32)), material=black_rubber, name="bottom_shoe_0")
    gate.visual(Box((0.045, 0.035, 0.15)), origin=Origin(xyz=(0.42, 0.0, 1.47)), material=dark_steel, name="top_hanger_1")
    gate.visual(Box((0.11, 0.04, 0.055)), origin=Origin(xyz=(0.42, 0.010, 1.55)), material=black_rubber, name="top_shoe_1")
    gate.visual(Box((0.045, 0.035, 0.15)), origin=Origin(xyz=(0.42, 0.0, 0.39)), material=dark_steel, name="bottom_hanger_1")
    gate.visual(Box((0.11, 0.04, 0.055)), origin=Origin(xyz=(0.42, 0.010, 0.32)), material=black_rubber, name="bottom_shoe_1")
    gate.visual(Cylinder(radius=0.035, length=0.03), origin=Origin(xyz=(-0.42, -0.008, 1.55), rpy=(pi / 2.0, 0.0, 0.0)), material=worn_steel, name="top_roller_0")
    gate.visual(Cylinder(radius=0.035, length=0.03), origin=Origin(xyz=(-0.42, -0.008, 0.32), rpy=(pi / 2.0, 0.0, 0.0)), material=worn_steel, name="bottom_roller_0")
    gate.visual(Cylinder(radius=0.035, length=0.03), origin=Origin(xyz=(0.42, -0.008, 1.55), rpy=(pi / 2.0, 0.0, 0.0)), material=worn_steel, name="top_roller_1")
    gate.visual(Cylinder(radius=0.035, length=0.03), origin=Origin(xyz=(0.42, -0.008, 0.32), rpy=(pi / 2.0, 0.0, 0.0)), material=worn_steel, name="bottom_roller_1")

    model.articulation(
        "shaft_to_cage",
        ArticulationType.PRISMATIC,
        parent=shaft,
        child=cage,
        origin=Origin(xyz=(0.0, 0.0, 0.25)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4000.0, velocity=0.7, lower=0.0, upper=1.10),
    )
    model.articulation(
        "cage_to_gate",
        ArticulationType.PRISMATIC,
        parent=cage,
        child=gate,
        origin=Origin(xyz=(0.0, -0.535, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.45, lower=0.0, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shaft = object_model.get_part("shaft_frame")
    cage = object_model.get_part("cage")
    gate = object_model.get_part("gate")
    cage_slide = object_model.get_articulation("shaft_to_cage")
    gate_slide = object_model.get_articulation("cage_to_gate")

    ctx.expect_contact(cage, shaft, elem_a="guide_shoe_0_0", elem_b="guide_rail_0", name="cage shoe bears on guide rail")
    ctx.expect_contact(gate, cage, elem_a="top_shoe_0", elem_b="upper_gate_track", name="gate shoe rides upper track")
    ctx.expect_contact(gate, cage, elem_a="bottom_shoe_0", elem_b="lower_gate_track", name="gate shoe rides lower track")
    ctx.expect_within(gate, cage, axes="z", inner_elem="gate_side_0", outer_elem="front_top_rail", margin=1.75, name="gate remains in cage entrance height")

    cage_rest = ctx.part_world_position(cage)
    gate_rest = ctx.part_world_position(gate)
    with ctx.pose({cage_slide: 1.10}):
        ctx.expect_contact(cage, shaft, elem_a="guide_shoe_1_1", elem_b="guide_rail_1", name="raised cage remains guided")
        cage_raised = ctx.part_world_position(cage)
    with ctx.pose({gate_slide: 0.55}):
        ctx.expect_contact(gate, cage, elem_a="top_shoe_1", elem_b="upper_gate_track", name="opened gate remains on track")
        gate_open = ctx.part_world_position(gate)

    ctx.check(
        "cage slides upward on vertical rails",
        cage_rest is not None and cage_raised is not None and cage_raised[2] > cage_rest[2] + 1.0,
        details=f"rest={cage_rest}, raised={cage_raised}",
    )
    ctx.check(
        "gate slides horizontally across entrance",
        gate_rest is not None and gate_open is not None and gate_open[0] > gate_rest[0] + 0.50,
        details=f"rest={gate_rest}, open={gate_open}",
    )

    return ctx.report()


object_model = build_object_model()
