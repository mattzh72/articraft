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

    concrete = model.material("concrete", rgba=(0.63, 0.65, 0.66, 1.0))
    steel = model.material("steel", rgba=(0.30, 0.33, 0.37, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.17, 0.18, 0.20, 1.0))
    deck_asphalt = model.material("deck_asphalt", rgba=(0.12, 0.12, 0.13, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.82, 0.74, 0.18, 1.0))

    shore_frame = model.part("shore_frame")
    shore_frame.visual(
        Box((4.00, 5.40, 1.70)),
        origin=Origin(xyz=(-2.45, 0.00, -0.85)),
        material=concrete,
        name="abutment",
    )
    shore_frame.visual(
        Box((1.40, 3.40, 0.20)),
        origin=Origin(xyz=(-0.90, 0.00, -0.10)),
        material=deck_asphalt,
        name="approach_deck",
    )
    shore_frame.visual(
        Box((1.10, 0.44, 1.45)),
        origin=Origin(xyz=(-0.05, 2.50, -0.08)),
        material=steel,
        name="left_bearing_pedestal",
    )
    shore_frame.visual(
        Box((1.10, 0.44, 1.45)),
        origin=Origin(xyz=(-0.05, -2.50, -0.08)),
        material=steel,
        name="right_bearing_pedestal",
    )
    shore_frame.visual(
        Box((0.90, 4.58, 0.26)),
        origin=Origin(xyz=(-0.18, 0.00, 0.45)),
        material=steel,
        name="portal_tie_beam",
    )
    shore_frame.visual(
        Cylinder(radius=0.34, length=0.24),
        origin=Origin(xyz=(0.00, 2.36, -0.20), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_bearing_housing",
    )
    shore_frame.visual(
        Cylinder(radius=0.34, length=0.24),
        origin=Origin(xyz=(0.00, -2.36, -0.20), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_bearing_housing",
    )
    shore_frame.visual(
        Box((0.32, 3.72, 0.18)),
        origin=Origin(xyz=(-0.34, 0.00, 0.05)),
        material=safety_yellow,
        name="hinge_walkway",
    )
    shore_frame.inertial = Inertial.from_geometry(
        Box((4.00, 5.40, 1.70)),
        mass=85000.0,
        origin=Origin(xyz=(-2.20, 0.00, -0.85)),
    )

    bridge_leaf = model.part("bridge_leaf")
    bridge_leaf.visual(
        Box((8.20, 3.80, 0.18)),
        origin=Origin(xyz=(4.00, 0.00, 0.11)),
        material=deck_asphalt,
        name="roadway",
    )
    bridge_leaf.visual(
        Box((0.70, 4.00, 0.64)),
        origin=Origin(xyz=(0.25, 0.00, -0.14)),
        material=steel,
        name="heel_box",
    )
    bridge_leaf.visual(
        Box((8.20, 0.24, 0.58)),
        origin=Origin(xyz=(4.00, 1.82, -0.11)),
        material=steel,
        name="left_girder",
    )
    bridge_leaf.visual(
        Box((8.20, 0.24, 0.58)),
        origin=Origin(xyz=(4.00, -1.82, -0.11)),
        material=steel,
        name="right_girder",
    )
    bridge_leaf.visual(
        Box((0.22, 3.76, 0.50)),
        origin=Origin(xyz=(8.09, 0.00, -0.07)),
        material=steel,
        name="toe_beam",
    )
    for index, x_pos in enumerate((1.70, 3.70, 5.70)):
        bridge_leaf.visual(
            Box((0.16, 3.34, 0.36)),
            origin=Origin(xyz=(x_pos, 0.00, -0.07)),
            material=steel,
            name=f"cross_floorbeam_{index}",
        )
    bridge_leaf.visual(
        Box((8.00, 0.16, 0.12)),
        origin=Origin(xyz=(4.00, 1.60, 0.23)),
        material=safety_yellow,
        name="left_curb",
    )
    bridge_leaf.visual(
        Box((8.00, 0.16, 0.12)),
        origin=Origin(xyz=(4.00, -1.60, 0.23)),
        material=safety_yellow,
        name="right_curb",
    )
    bridge_leaf.visual(
        Cylinder(radius=0.26, length=0.30),
        origin=Origin(xyz=(0.00, 2.09, 0.00), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_trunnion",
    )
    bridge_leaf.visual(
        Cylinder(radius=0.26, length=0.30),
        origin=Origin(xyz=(0.00, -2.09, 0.00), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_trunnion",
    )
    bridge_leaf.inertial = Inertial.from_geometry(
        Box((8.20, 4.00, 0.70)),
        mass=26000.0,
        origin=Origin(xyz=(4.00, 0.00, -0.08)),
    )

    model.articulation(
        "shore_to_leaf",
        ArticulationType.REVOLUTE,
        parent=shore_frame,
        child=bridge_leaf,
        origin=Origin(xyz=(0.00, 0.00, -0.20)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=600000.0,
            velocity=0.35,
            lower=0.0,
            upper=1.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    shore_frame = object_model.get_part("shore_frame")
    bridge_leaf = object_model.get_part("bridge_leaf")
    hinge = object_model.get_articulation("shore_to_leaf")

    closed_roadway = None
    approach_deck = None
    closed_toe = None
    open_toe = None

    with ctx.pose({hinge: 0.0}):
        ctx.expect_contact(
            bridge_leaf,
            shore_frame,
            elem_a="left_trunnion",
            elem_b="left_bearing_housing",
            name="left trunnion seats in left shore bearing",
        )
        ctx.expect_contact(
            bridge_leaf,
            shore_frame,
            elem_a="right_trunnion",
            elem_b="right_bearing_housing",
            name="right trunnion seats in right shore bearing",
        )
        ctx.expect_overlap(
            bridge_leaf,
            shore_frame,
            axes="y",
            elem_a="roadway",
            elem_b="approach_deck",
            min_overlap=3.20,
            name="leaf aligns with approach deck width",
        )
        closed_roadway = ctx.part_element_world_aabb(bridge_leaf, elem="roadway")
        approach_deck = ctx.part_element_world_aabb(shore_frame, elem="approach_deck")
        closed_toe = ctx.part_element_world_aabb(bridge_leaf, elem="toe_beam")

    with ctx.pose({hinge: 1.10}):
        open_toe = ctx.part_element_world_aabb(bridge_leaf, elem="toe_beam")

    deck_height_aligned = (
        closed_roadway is not None
        and approach_deck is not None
        and abs(closed_roadway[1][2] - approach_deck[1][2]) <= 0.01
    )
    ctx.check(
        "closed leaf deck sits level with shore deck",
        deck_height_aligned,
        details=f"leaf={closed_roadway}, approach={approach_deck}",
    )

    toe_rises = (
        closed_toe is not None
        and open_toe is not None
        and (open_toe[1][2] - closed_toe[1][2]) >= 6.0
    )
    ctx.check(
        "bridge leaf raises its free end upward",
        toe_rises,
        details=f"closed_toe={closed_toe}, open_toe={open_toe}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
