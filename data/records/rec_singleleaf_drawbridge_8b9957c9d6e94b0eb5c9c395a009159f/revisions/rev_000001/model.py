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

    concrete = model.material("concrete", rgba=(0.61, 0.62, 0.60, 1.0))
    dark_concrete = model.material("dark_concrete", rgba=(0.48, 0.49, 0.48, 1.0))
    bridge_steel = model.material("bridge_steel", rgba=(0.31, 0.36, 0.40, 1.0))
    deck_asphalt = model.material("deck_asphalt", rgba=(0.16, 0.17, 0.18, 1.0))
    safety_grey = model.material("safety_grey", rgba=(0.69, 0.71, 0.72, 1.0))

    shore_frame = model.part("shore_frame")
    shore_frame.visual(
        Box((5.60, 1.70, 1.00)),
        origin=Origin(xyz=(0.0, -0.85, 0.50)),
        material=concrete,
        name="foundation_block",
    )
    shore_frame.visual(
        Box((4.30, 1.80, 0.18)),
        origin=Origin(xyz=(0.0, -0.90, 1.09)),
        material=deck_asphalt,
        name="approach_deck",
    )
    shore_frame.visual(
        Box((0.20, 1.76, 0.30)),
        origin=Origin(xyz=(-1.98, -0.90, 1.33)),
        material=dark_concrete,
        name="left_approach_barrier",
    )
    shore_frame.visual(
        Box((0.20, 1.76, 0.30)),
        origin=Origin(xyz=(1.98, -0.90, 1.33)),
        material=dark_concrete,
        name="right_approach_barrier",
    )
    shore_frame.visual(
        Box((0.56, 1.52, 0.90)),
        origin=Origin(xyz=(-2.28, -0.06, 1.45)),
        material=dark_concrete,
        name="left_bearing_pedestal",
    )
    shore_frame.visual(
        Box((0.56, 1.52, 0.90)),
        origin=Origin(xyz=(2.28, -0.06, 1.45)),
        material=dark_concrete,
        name="right_bearing_pedestal",
    )
    shore_frame.visual(
        Box((3.10, 0.86, 0.50)),
        origin=Origin(xyz=(0.0, -0.74, 1.25)),
        material=dark_concrete,
        name="machinery_house_base",
    )
    shore_frame.visual(
        Box((3.70, 0.18, 0.18)),
        origin=Origin(xyz=(0.0, -0.10, 1.09)),
        material=bridge_steel,
        name="hinge_sill_beam",
    )
    shore_frame.visual(
        Cylinder(radius=0.22, length=0.20),
        origin=Origin(xyz=(-2.07, 0.10, 1.02), rpy=(0.0, pi / 2.0, 0.0)),
        material=bridge_steel,
        name="left_bearing_drum",
    )
    shore_frame.visual(
        Cylinder(radius=0.22, length=0.20),
        origin=Origin(xyz=(2.07, 0.10, 1.02), rpy=(0.0, pi / 2.0, 0.0)),
        material=bridge_steel,
        name="right_bearing_drum",
    )
    shore_frame.inertial = Inertial.from_geometry(
        Box((5.60, 1.70, 2.00)),
        mass=12000.0,
        origin=Origin(xyz=(0.0, -0.85, 1.00)),
    )

    bridge_leaf = model.part("bridge_leaf")
    bridge_leaf.visual(
        Box((3.80, 4.40, 0.16)),
        origin=Origin(xyz=(0.0, 2.20, 0.08)),
        material=deck_asphalt,
        name="roadway_panel",
    )
    bridge_leaf.visual(
        Box((0.28, 4.22, 0.60)),
        origin=Origin(xyz=(-1.76, 2.11, -0.14)),
        material=bridge_steel,
        name="left_main_girder",
    )
    bridge_leaf.visual(
        Box((0.28, 4.22, 0.60)),
        origin=Origin(xyz=(1.76, 2.11, -0.14)),
        material=bridge_steel,
        name="right_main_girder",
    )
    bridge_leaf.visual(
        Box((0.22, 4.00, 0.42)),
        origin=Origin(xyz=(0.0, 2.02, -0.11)),
        material=bridge_steel,
        name="center_stiffener",
    )
    bridge_leaf.visual(
        Box((3.80, 0.32, 0.56)),
        origin=Origin(xyz=(0.0, 0.16, -0.12)),
        material=bridge_steel,
        name="trunnion_beam",
    )
    bridge_leaf.visual(
        Box((3.80, 0.22, 0.30)),
        origin=Origin(xyz=(0.0, 4.29, -0.05)),
        material=bridge_steel,
        name="tip_beam",
    )
    bridge_leaf.visual(
        Box((0.12, 4.18, 0.20)),
        origin=Origin(xyz=(-1.84, 2.09, 0.25)),
        material=safety_grey,
        name="left_parapet",
    )
    bridge_leaf.visual(
        Box((0.12, 4.18, 0.20)),
        origin=Origin(xyz=(1.84, 2.09, 0.25)),
        material=safety_grey,
        name="right_parapet",
    )
    bridge_leaf.visual(
        Cylinder(radius=0.18, length=0.10),
        origin=Origin(xyz=(-1.88, 0.20, 0.02), rpy=(0.0, pi / 2.0, 0.0)),
        material=bridge_steel,
        name="left_trunnion_collar",
    )
    bridge_leaf.visual(
        Cylinder(radius=0.18, length=0.10),
        origin=Origin(xyz=(1.88, 0.20, 0.02), rpy=(0.0, pi / 2.0, 0.0)),
        material=bridge_steel,
        name="right_trunnion_collar",
    )
    bridge_leaf.inertial = Inertial.from_geometry(
        Box((3.95, 4.40, 0.90)),
        mass=4200.0,
        origin=Origin(xyz=(0.0, 2.15, -0.05)),
    )

    model.articulation(
        "shore_to_bridge_leaf",
        ArticulationType.REVOLUTE,
        parent=shore_frame,
        child=bridge_leaf,
        origin=Origin(xyz=(0.0, 0.0, 1.02)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=250000.0,
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
    hinge = object_model.get_articulation("shore_to_bridge_leaf")

    ctx.expect_gap(
        bridge_leaf,
        shore_frame,
        axis="y",
        positive_elem="roadway_panel",
        negative_elem="approach_deck",
        max_gap=0.02,
        max_penetration=0.0,
        name="closed leaf meets the shore deck at the hinge line",
    )
    ctx.expect_overlap(
        bridge_leaf,
        shore_frame,
        axes="x",
        elem_a="roadway_panel",
        elem_b="approach_deck",
        min_overlap=3.60,
        name="leaf roadway spans nearly the full shore deck width",
    )

    rest_tip = ctx.part_element_world_aabb(bridge_leaf, elem="tip_beam")
    with ctx.pose({hinge: 1.18}):
        open_tip = ctx.part_element_world_aabb(bridge_leaf, elem="tip_beam")
        ctx.expect_gap(
            bridge_leaf,
            shore_frame,
            axis="z",
            positive_elem="tip_beam",
            negative_elem="approach_deck",
            min_gap=2.5,
            name="opened bridge tip rises well above the shore deck",
        )

    ctx.check(
        "leaf tip rises when the bridge opens",
        rest_tip is not None
        and open_tip is not None
        and open_tip[0][2] > rest_tip[1][2] + 2.0,
        details=f"rest_tip={rest_tip}, open_tip={open_tip}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
