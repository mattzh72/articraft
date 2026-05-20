from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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

    concrete = model.material("concrete", rgba=(0.60, 0.61, 0.62, 1.0))
    deck_steel = model.material("deck_steel", rgba=(0.28, 0.33, 0.37, 1.0))
    painted_steel = model.material("painted_steel", rgba=(0.22, 0.26, 0.22, 1.0))
    bearing_steel = model.material("bearing_steel", rgba=(0.44, 0.45, 0.47, 1.0))
    pavement = model.material("pavement", rgba=(0.16, 0.16, 0.17, 1.0))
    stripe = model.material("stripe", rgba=(0.86, 0.79, 0.28, 1.0))
    rail_metal = model.material("rail_metal", rgba=(0.72, 0.74, 0.76, 1.0))

    shore_frame = model.part("shore_frame")
    shore_frame.inertial = Inertial.from_geometry(
        Box((5.2, 11.4, 4.8)),
        mass=85000.0,
        origin=Origin(xyz=(-1.1, 0.0, 2.4)),
    )
    shore_frame.visual(
        Box((5.2, 11.4, 1.2)),
        origin=Origin(xyz=(-1.1, 0.0, 0.6)),
        material=concrete,
        name="base_slab",
    )
    shore_frame.visual(
        Box((2.2, 7.1, 0.42)),
        origin=Origin(xyz=(-1.2, 0.0, 2.28)),
        material=pavement,
        name="shore_roadway",
    )
    shore_frame.visual(
        Box((1.8, 2.2, 1.08)),
        origin=Origin(xyz=(-1.45, 0.0, 1.53)),
        material=concrete,
        name="approach_plinth",
    )
    shore_frame.visual(
        Box((3.4, 1.30, 3.45)),
        origin=Origin(xyz=(-0.35, 5.03, 2.925)),
        material=concrete,
        name="left_side_wall",
    )
    shore_frame.visual(
        Box((3.4, 1.30, 3.45)),
        origin=Origin(xyz=(-0.35, -5.03, 2.925)),
        material=concrete,
        name="right_side_wall",
    )
    shore_frame.visual(
        Box((1.4, 8.9, 1.15)),
        origin=Origin(xyz=(-2.45, 0.0, 4.025)),
        material=concrete,
        name="rear_crossbeam",
    )
    shore_frame.visual(
        Box((1.9, 0.72, 1.15)),
        origin=Origin(xyz=(0.55, 4.95, 1.975)),
        material=concrete,
        name="left_front_cheek",
    )
    shore_frame.visual(
        Box((1.9, 0.72, 1.15)),
        origin=Origin(xyz=(0.55, -4.95, 1.975)),
        material=concrete,
        name="right_front_cheek",
    )
    shore_frame.visual(
        Box((1.20, 0.66, 1.10)),
        origin=Origin(xyz=(-0.10, 4.68, 2.20)),
        material=bearing_steel,
        name="left_bearing_block",
    )
    shore_frame.visual(
        Box((1.20, 0.66, 1.10)),
        origin=Origin(xyz=(-0.10, -4.68, 2.20)),
        material=bearing_steel,
        name="right_bearing_block",
    )
    shore_frame.visual(
        Cylinder(radius=0.24, length=0.78),
        origin=Origin(xyz=(-0.05, 4.93, 2.20), rpy=(1.57079632679, 0.0, 0.0)),
        material=rail_metal,
        name="left_bearing_cap",
    )
    shore_frame.visual(
        Cylinder(radius=0.24, length=0.78),
        origin=Origin(xyz=(-0.05, -4.93, 2.20), rpy=(1.57079632679, 0.0, 0.0)),
        material=rail_metal,
        name="right_bearing_cap",
    )

    bridge_leaf = model.part("bridge_leaf")
    bridge_leaf.inertial = Inertial.from_geometry(
        Box((12.0, 8.6, 0.95)),
        mass=42000.0,
        origin=Origin(xyz=(6.0, 0.0, -0.05)),
    )
    bridge_leaf.visual(
        Box((12.0, 7.60, 0.14)),
        origin=Origin(xyz=(6.0, 0.0, 0.22)),
        material=pavement,
        name="road_deck",
    )
    bridge_leaf.visual(
        Box((11.6, 0.56, 0.84)),
        origin=Origin(xyz=(5.8, 3.72, -0.12)),
        material=painted_steel,
        name="left_main_girder",
    )
    bridge_leaf.visual(
        Box((11.6, 0.56, 0.84)),
        origin=Origin(xyz=(5.8, -3.72, -0.12)),
        material=painted_steel,
        name="right_main_girder",
    )
    bridge_leaf.visual(
        Box((0.92, 8.0, 0.56)),
        origin=Origin(xyz=(0.46, 0.0, 0.0)),
        material=deck_steel,
        name="hinge_floorbeam",
    )
    bridge_leaf.visual(
        Box((0.52, 8.0, 0.56)),
        origin=Origin(xyz=(6.0, 0.0, -0.06)),
        material=deck_steel,
        name="mid_crossbeam",
    )
    bridge_leaf.visual(
        Box((0.56, 8.0, 0.56)),
        origin=Origin(xyz=(11.72, 0.0, 0.0)),
        material=deck_steel,
        name="nose_beam",
    )
    bridge_leaf.visual(
        Cylinder(radius=0.45, length=0.35),
        origin=Origin(xyz=(0.0, 4.175, 0.0), rpy=(1.57079632679, 0.0, 0.0)),
        material=bearing_steel,
        name="left_trunnion",
    )
    bridge_leaf.visual(
        Cylinder(radius=0.45, length=0.35),
        origin=Origin(xyz=(0.0, -4.175, 0.0), rpy=(1.57079632679, 0.0, 0.0)),
        material=bearing_steel,
        name="right_trunnion",
    )
    bridge_leaf.visual(
        Box((8.8, 0.16, 0.18)),
        origin=Origin(xyz=(7.1, 3.16, 0.38)),
        material=rail_metal,
        name="left_guardrail",
    )
    bridge_leaf.visual(
        Box((8.8, 0.16, 0.18)),
        origin=Origin(xyz=(7.1, -3.16, 0.38)),
        material=rail_metal,
        name="right_guardrail",
    )
    bridge_leaf.visual(
        Box((0.18, 4.8, 0.018)),
        origin=Origin(xyz=(6.2, 0.0, 0.299)),
        material=stripe,
        name="centerline_stripe",
    )

    model.articulation(
        "shore_to_leaf",
        ArticulationType.REVOLUTE,
        parent=shore_frame,
        child=bridge_leaf,
        origin=Origin(xyz=(0.0, 0.0, 2.20)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=300000.0,
            velocity=0.4,
            lower=0.0,
            upper=1.18,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shore_frame = object_model.get_part("shore_frame")
    bridge_leaf = object_model.get_part("bridge_leaf")
    leaf_joint = object_model.get_articulation("shore_to_leaf")

    ctx.expect_within(
        bridge_leaf,
        shore_frame,
        axes="xz",
        inner_elem="left_trunnion",
        outer_elem="left_bearing_block",
        margin=0.06,
        name="left trunnion stays seated in left bearing block envelope",
    )
    ctx.expect_within(
        bridge_leaf,
        shore_frame,
        axes="xz",
        inner_elem="right_trunnion",
        outer_elem="right_bearing_block",
        margin=0.06,
        name="right trunnion stays seated in right bearing block envelope",
    )

    with ctx.pose({leaf_joint: 0.0}):
        ctx.expect_gap(
            bridge_leaf,
            shore_frame,
            axis="x",
            positive_elem="road_deck",
            negative_elem="shore_roadway",
            min_gap=0.05,
            max_gap=0.20,
            name="closed leaf begins just beyond the shore approach slab",
        )
        closed_deck = ctx.part_element_world_aabb(bridge_leaf, elem="road_deck")
        closed_approach = ctx.part_element_world_aabb(shore_frame, elem="shore_roadway")
        deck_and_approach_ok = closed_deck is not None and closed_approach is not None
        if deck_and_approach_ok:
            deck_top = closed_deck[1][2]
            approach_top = closed_approach[1][2]
        else:
            deck_top = None
            approach_top = None
        ctx.check(
            "closed leaf deck aligns with the shore approach elevation",
            deck_and_approach_ok and abs(deck_top - approach_top) <= 0.03,
            details=f"deck_top={deck_top}, approach_top={approach_top}",
        )

    closed_tip = ctx.part_element_world_aabb(bridge_leaf, elem="nose_beam")
    with ctx.pose({leaf_joint: 1.0}):
        open_tip = ctx.part_element_world_aabb(bridge_leaf, elem="nose_beam")

    closed_ok = closed_tip is not None and open_tip is not None
    if closed_ok:
        closed_tip_center = (
            (closed_tip[0][0] + closed_tip[1][0]) * 0.5,
            (closed_tip[0][1] + closed_tip[1][1]) * 0.5,
            (closed_tip[0][2] + closed_tip[1][2]) * 0.5,
        )
        open_tip_center = (
            (open_tip[0][0] + open_tip[1][0]) * 0.5,
            (open_tip[0][1] + open_tip[1][1]) * 0.5,
            (open_tip[0][2] + open_tip[1][2]) * 0.5,
        )
    else:
        closed_tip_center = None
        open_tip_center = None
    ctx.check(
        "leaf opens upward from the shore hinge",
        closed_ok
        and open_tip_center[2] > closed_tip_center[2] + 8.0
        and open_tip_center[0] < closed_tip_center[0] - 4.0,
        details=f"closed_tip_center={closed_tip_center}, open_tip_center={open_tip_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
