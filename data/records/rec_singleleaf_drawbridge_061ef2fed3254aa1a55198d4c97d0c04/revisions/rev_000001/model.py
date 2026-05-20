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
    model = ArticulatedObject(name="single_leaf_drawbridge")

    concrete = model.material("weathered_concrete", rgba=(0.46, 0.44, 0.40, 1.0))
    asphalt = model.material("dark_asphalt", rgba=(0.025, 0.027, 0.030, 1.0))
    painted_steel = model.material("oxide_red_steel", rgba=(0.46, 0.09, 0.045, 1.0))
    dark_steel = model.material("dark_gunmetal", rgba=(0.10, 0.11, 0.12, 1.0))
    polished = model.material("polished_pin_steel", rgba=(0.74, 0.72, 0.66, 1.0))
    yellow = model.material("safety_yellow", rgba=(0.95, 0.70, 0.06, 1.0))
    white = model.material("worn_white_paint", rgba=(0.86, 0.84, 0.76, 1.0))

    axis_z = 0.86

    frame = model.part("shore_frame")
    frame.visual(
        Box((1.45, 2.45, 0.78)),
        origin=Origin(xyz=(-0.73, 0.0, 0.39)),
        material=concrete,
        name="abutment_block",
    )
    frame.visual(
        Box((1.20, 1.42, 0.045)),
        origin=Origin(xyz=(-0.68, 0.0, 0.812)),
        material=asphalt,
        name="approach_slab",
    )
    frame.visual(
        Box((0.18, 2.28, 0.28)),
        origin=Origin(xyz=(-0.17, 0.0, 0.70)),
        material=concrete,
        name="shore_backwall",
    )
    frame.visual(
        Box((0.34, 0.38, 0.34)),
        origin=Origin(xyz=(0.0, 0.98, 0.61)),
        material=concrete,
        name="bearing_pedestal_0",
    )
    frame.visual(
        Box((0.34, 0.38, 0.34)),
        origin=Origin(xyz=(0.0, -0.98, 0.61)),
        material=concrete,
        name="bearing_pedestal_1",
    )
    frame.visual(
        Cylinder(radius=0.18, length=0.22),
        origin=Origin(xyz=(0.0, 0.97, axis_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="bearing_0",
    )
    frame.visual(
        Cylinder(radius=0.18, length=0.22),
        origin=Origin(xyz=(0.0, -0.97, axis_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="bearing_1",
    )
    frame.visual(
        Box((0.12, 0.42, 0.045)),
        origin=Origin(xyz=(0.0, 0.98, 0.725)),
        material=painted_steel,
        name="bearing_base_0",
    )
    frame.visual(
        Box((0.12, 0.42, 0.045)),
        origin=Origin(xyz=(0.0, -0.98, 0.725)),
        material=painted_steel,
        name="bearing_base_1",
    )
    for idx, y in enumerate((0.98, -0.98)):
        for x in (-0.105, 0.105):
            frame.visual(
                Cylinder(radius=0.018, length=0.012),
                origin=Origin(xyz=(x, y, 0.756)),
                material=polished,
                name=f"bearing_bolt_{idx}_{0 if x < 0 else 1}",
            )

    leaf = model.part("bridge_leaf")
    leaf.visual(
        Box((4.80, 1.50, 0.12)),
        origin=Origin(xyz=(2.40, 0.0, 0.0)),
        material=painted_steel,
        name="deck_panel",
    )
    leaf.visual(
        Box((4.72, 1.36, 0.026)),
        origin=Origin(xyz=(2.44, 0.0, 0.073)),
        material=asphalt,
        name="road_surface",
    )
    leaf.visual(
        Box((4.25, 0.035, 0.006)),
        origin=Origin(xyz=(2.62, 0.0, 0.089)),
        material=white,
        name="center_line",
    )
    leaf.visual(
        Box((4.70, 0.12, 0.26)),
        origin=Origin(xyz=(2.45, 0.66, -0.07)),
        material=dark_steel,
        name="side_girder_0",
    )
    leaf.visual(
        Box((4.70, 0.12, 0.26)),
        origin=Origin(xyz=(2.45, -0.66, -0.07)),
        material=dark_steel,
        name="side_girder_1",
    )
    leaf.visual(
        Box((0.18, 1.42, 0.22)),
        origin=Origin(xyz=(0.12, 0.0, -0.03)),
        material=dark_steel,
        name="heel_crossbeam",
    )
    leaf.visual(
        Box((0.16, 1.58, 0.18)),
        origin=Origin(xyz=(4.76, 0.0, -0.02)),
        material=dark_steel,
        name="tip_crossbeam",
    )
    leaf.visual(
        Box((0.34, 0.11, 0.14)),
        origin=Origin(xyz=(0.23, 0.77, 0.0)),
        material=painted_steel,
        name="trunnion_lug_0",
    )
    leaf.visual(
        Box((0.34, 0.11, 0.14)),
        origin=Origin(xyz=(0.23, -0.77, 0.0)),
        material=painted_steel,
        name="trunnion_lug_1",
    )
    leaf.visual(
        Cylinder(radius=0.070, length=0.46),
        origin=Origin(xyz=(0.0, 0.92, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=polished,
        name="trunnion_0",
    )
    leaf.visual(
        Cylinder(radius=0.070, length=0.46),
        origin=Origin(xyz=(0.0, -0.92, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=polished,
        name="trunnion_1",
    )
    leaf.visual(
        Box((0.18, 1.42, 0.008)),
        origin=Origin(xyz=(4.58, 0.0, 0.087)),
        material=yellow,
        name="tip_warning_band",
    )

    model.articulation(
        "leaf_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=leaf,
        origin=Origin(xyz=(0.0, 0.0, axis_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=250000.0, velocity=0.35, lower=0.0, upper=1.22),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("shore_frame")
    leaf = object_model.get_part("bridge_leaf")
    hinge = object_model.get_articulation("leaf_hinge")

    ctx.allow_overlap(
        leaf,
        frame,
        elem_a="trunnion_0",
        elem_b="bearing_0",
        reason="The leaf trunnion journal is intentionally captured inside the heavy side bearing proxy.",
    )
    ctx.allow_overlap(
        leaf,
        frame,
        elem_a="trunnion_1",
        elem_b="bearing_1",
        reason="The opposite trunnion journal is intentionally captured inside the heavy side bearing proxy.",
    )

    ctx.expect_within(
        leaf,
        frame,
        axes="xz",
        inner_elem="trunnion_0",
        outer_elem="bearing_0",
        margin=0.001,
        name="outer trunnion centered in bearing",
    )
    ctx.expect_within(
        leaf,
        frame,
        axes="xz",
        inner_elem="trunnion_1",
        outer_elem="bearing_1",
        margin=0.001,
        name="opposite trunnion centered in bearing",
    )
    ctx.expect_overlap(
        leaf,
        frame,
        axes="y",
        elem_a="trunnion_0",
        elem_b="bearing_0",
        min_overlap=0.12,
        name="outer trunnion inserted through bearing",
    )
    ctx.expect_overlap(
        leaf,
        frame,
        axes="y",
        elem_a="trunnion_1",
        elem_b="bearing_1",
        min_overlap=0.12,
        name="opposite trunnion inserted through bearing",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            leaf,
            frame,
            axis="x",
            positive_elem="deck_panel",
            negative_elem="approach_slab",
            min_gap=0.02,
            max_gap=0.15,
            name="closed leaf starts just beyond shore slab",
        )
        ctx.expect_overlap(
            leaf,
            frame,
            axes="y",
            elem_a="deck_panel",
            elem_b="approach_slab",
            min_overlap=1.20,
            name="closed leaf aligns with shore roadway",
        )

    def _z_top(aabb):
        return aabb[1][2]

    closed_tip = ctx.part_element_world_aabb(leaf, elem="tip_crossbeam")
    with ctx.pose({hinge: 1.22}):
        raised_tip = ctx.part_element_world_aabb(leaf, elem="tip_crossbeam")
    ctx.check(
        "hinge raises free end upward",
        closed_tip is not None
        and raised_tip is not None
        and _z_top(raised_tip) > _z_top(closed_tip) + 2.0,
        details=f"closed_tip={closed_tip}, raised_tip={raised_tip}",
    )

    return ctx.report()


object_model = build_object_model()
