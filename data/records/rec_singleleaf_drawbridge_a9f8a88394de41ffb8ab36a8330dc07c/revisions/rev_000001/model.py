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
import math

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bascule_bridge")

    concrete = (0.65, 0.65, 0.65, 1.0)
    steel = (0.25, 0.3, 0.35, 1.0)
    road = (0.2, 0.2, 0.2, 1.0)
    sidewalk = (0.5, 0.5, 0.5, 1.0)
    railing = (0.8, 0.8, 0.8, 1.0)
    counterweight_color = (0.55, 0.55, 0.55, 1.0)
    tower_color = (0.7, 0.7, 0.75, 1.0)
    roof_color = (0.3, 0.3, 0.3, 1.0)

    base = model.part("base")

    water_color = (0.2, 0.4, 0.5, 1.0)
    base.visual(Box((100.0, 40.0, 1.0)), origin=Origin((4.0, 0.0, 0.5)), name="water", color=water_color)

    # Piers
    base.visual(Box((18.0, 24.0, 2.0)), origin=Origin((0.0, 0.0, 1.0)), name="pier_pad", color=concrete)
    
    base.visual(Box((10.0, 6.0, 6.0)), origin=Origin((0.0, 7.0, 5.0)), name="pier_left_base", color=concrete)
    base.visual(Box((8.0, 4.0, 8.0)), origin=Origin((0.0, 7.0, 12.0)), name="pier_left_mid", color=concrete)
    
    base.visual(Box((10.0, 6.0, 6.0)), origin=Origin((0.0, -7.0, 5.0)), name="pier_right_base", color=concrete)
    base.visual(Box((8.0, 4.0, 8.0)), origin=Origin((0.0, -7.0, 12.0)), name="pier_right_mid", color=concrete)

    # Approach side
    base.visual(Box((20.0, 10.0, 0.4)), origin=Origin((-21.6, 0.0, 16.2)), name="approach_deck", color=road)
    base.visual(Box((4.0, 10.0, 16.0)), origin=Origin((-21.6, 0.0, 8.0)), name="approach_pier", color=concrete)
    base.visual(Box((4.0, 10.0, 16.0)), origin=Origin((-31.6, 0.0, 8.0)), name="approach_abutment", color=concrete)
    base.visual(Box((20.0, 1.5, 0.2)), origin=Origin((-21.6, 4.25, 16.5)), name="approach_sw_l", color=sidewalk)
    base.visual(Box((20.0, 1.5, 0.2)), origin=Origin((-21.6, -4.25, 16.5)), name="approach_sw_r", color=sidewalk)
    base.visual(Box((20.0, 0.1, 1.2)), origin=Origin((-21.6, 4.9, 17.1)), name="approach_rail_l", color=railing)
    base.visual(Box((20.0, 0.1, 1.2)), origin=Origin((-21.6, -4.9, 17.1)), name="approach_rail_r", color=railing)

    # Receiving side
    base.visual(Box((20.0, 10.0, 0.4)), origin=Origin((40.1, 0.0, 16.2)), name="receiving_deck", color=road)
    base.visual(Box((4.0, 10.0, 16.0)), origin=Origin((32.1, 0.0, 8.0)), name="receiving_pier", color=concrete)
    base.visual(Box((4.0, 10.0, 16.0)), origin=Origin((48.1, 0.0, 8.0)), name="receiving_abutment", color=concrete)
    base.visual(Box((20.0, 1.5, 0.2)), origin=Origin((40.1, 4.25, 16.5)), name="receiving_sw_l", color=sidewalk)
    base.visual(Box((20.0, 1.5, 0.2)), origin=Origin((40.1, -4.25, 16.5)), name="receiving_sw_r", color=sidewalk)
    base.visual(Box((20.0, 0.1, 1.2)), origin=Origin((40.1, 4.9, 17.1)), name="receiving_rail_l", color=railing)
    base.visual(Box((20.0, 0.1, 1.2)), origin=Origin((40.1, -4.9, 17.1)), name="receiving_rail_r", color=railing)

    # Control Tower
    base.visual(Box((4.0, 4.0, 16.0)), origin=Origin((-14.0, 8.0, 8.0)), name="tower_base", color=concrete)
    base.visual(Box((5.0, 5.0, 4.0)), origin=Origin((-14.0, 8.0, 18.0)), name="tower_cabin", color=tower_color)
    base.visual(Box((5.5, 5.5, 1.0)), origin=Origin((-14.0, 8.0, 20.5)), name="tower_roof", color=roof_color)
    base.visual(Box((2.0, 1.0, 0.4)), origin=Origin((-14.0, 5.5, 16.2)), name="tower_walkway", color=sidewalk)

    leaf = model.part("leaf")

    # Trunnion
    leaf.visual(Cylinder(radius=0.5, length=16.0), origin=Origin((0.0, 0.0, 0.0), rpy=(math.pi/2, 0.0, 0.0)), name="trunnion", color=steel)

    # Girders
    leaf.visual(Box((41.5, 0.8, 2.0)), origin=Origin((9.25, 4.5, 0.0)), name="girder_left", color=steel)
    leaf.visual(Box((41.5, 0.8, 2.0)), origin=Origin((9.25, -4.5, 0.0)), name="girder_right", color=steel)

    # Cross bracing
    for i in range(8):
        x_pos = -6.0 + i * 5.0
        leaf.visual(Box((0.4, 8.2, 1.0)), origin=Origin((x_pos, 0.0, 0.0)), name=f"cross_brace_{i}", color=steel)

    # Counterweight
    leaf.visual(Box((5.0, 8.2, 6.0)), origin=Origin((-8.5, 0.0, -2.0)), name="counterweight", color=counterweight_color)

    # Deck
    leaf.visual(Box((41.5, 10.0, 0.4)), origin=Origin((9.25, 0.0, 1.2)), name="deck_plate", color=road)
    leaf.visual(Box((41.5, 1.5, 0.2)), origin=Origin((9.25, 4.25, 1.5)), name="deck_sw_l", color=sidewalk)
    leaf.visual(Box((41.5, 1.5, 0.2)), origin=Origin((9.25, -4.25, 1.5)), name="deck_sw_r", color=sidewalk)
    leaf.visual(Box((41.5, 0.1, 1.2)), origin=Origin((9.25, 4.9, 2.1)), name="deck_rail_l", color=railing)
    leaf.visual(Box((41.5, 0.1, 1.2)), origin=Origin((9.25, -4.9, 2.1)), name="deck_rail_r", color=railing)

    # Road lines
    for i in range(10):
        x_pos = -8.0 + i * 4.0
        leaf.visual(Box((2.0, 0.2, 0.05)), origin=Origin((x_pos, 0.0, 1.425)), name=f"road_line_{i}", color=(0.9, 0.9, 0.9, 1.0))

    model.articulation(
        "main_trunnion",
        ArticulationType.REVOLUTE,
        parent=base,
        child=leaf,
        origin=Origin((0.0, 0.0, 15.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=5000.0, velocity=0.2, lower=0.0, upper=1.3)
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    leaf = object_model.get_part("leaf")
    joint = object_model.get_articulation("main_trunnion")

    # Allow trunnion overlap with piers
    ctx.allow_overlap(leaf, base, elem_a="trunnion", elem_b="pier_left_mid", reason="Trunnion axle rests in the left pier bearing.")
    ctx.allow_overlap(leaf, base, elem_a="trunnion", elem_b="pier_right_mid", reason="Trunnion axle rests in the right pier bearing.")

    # Check closed state gaps
    ctx.expect_gap(base, leaf, axis="x", positive_elem="receiving_deck", negative_elem="deck_plate", min_gap=0.05, max_gap=0.15)
    ctx.expect_gap(leaf, base, axis="x", positive_elem="deck_plate", negative_elem="approach_deck", min_gap=0.05, max_gap=0.15)

    with ctx.pose({joint: 1.3}):
        # In open pose, the tip of the leaf should be high up
        tip_pos = ctx.part_element_world_aabb(leaf, elem="deck_plate")
        if tip_pos:
            ctx.check("leaf_opens_upward", tip_pos[1][2] > 25.0, details="Deck tip should be raised high in the open pose.")
            
    return ctx.report()

object_model = build_object_model()
