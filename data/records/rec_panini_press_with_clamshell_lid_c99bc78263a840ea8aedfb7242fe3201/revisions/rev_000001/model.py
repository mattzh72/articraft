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
    model = ArticulatedObject(name="deli_counter_grill_press")

    stainless = model.material("stainless", rgba=(0.73, 0.75, 0.78, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.32, 0.34, 0.37, 1.0))
    cast_plate = model.material("cast_plate", rgba=(0.22, 0.22, 0.23, 1.0))
    phenolic = model.material("phenolic", rgba=(0.08, 0.08, 0.09, 1.0))
    marker_red = model.material("marker_red", rgba=(0.78, 0.18, 0.12, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.52, 0.42, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        material=dark_steel,
        name="bottom_plate",
    )
    base.visual(
        Box((0.48, 0.40, 0.022)),
        origin=Origin(xyz=(-0.01, 0.0, 0.106)),
        material=dark_steel,
        name="upper_deck",
    )
    base.visual(
        Box((0.48, 0.02, 0.15)),
        origin=Origin(xyz=(0.0, 0.20, 0.095)),
        material=stainless,
        name="left_wall",
    )
    base.visual(
        Box((0.48, 0.02, 0.15)),
        origin=Origin(xyz=(0.0, -0.20, 0.095)),
        material=stainless,
        name="right_wall",
    )
    base.visual(
        Box((0.02, 0.38, 0.22)),
        origin=Origin(xyz=(-0.25, 0.0, 0.12)),
        material=stainless,
        name="rear_wall",
    )
    base.visual(
        Box((0.02, 0.38, 0.05)),
        origin=Origin(xyz=(0.25, 0.0, 0.145)),
        material=stainless,
        name="front_beam",
    )
    base.visual(
        Box((0.17, 0.30, 0.016)),
        origin=Origin(xyz=(0.165, 0.0, 0.008)),
        material=dark_steel,
        name="drawer_channel",
    )
    base.visual(
        Box((0.34, 0.36, 0.028)),
        origin=Origin(xyz=(0.025, 0.0, 0.131)),
        material=cast_plate,
        name="lower_plate",
    )
    base.visual(
        Box((0.12, 0.04, 0.07)),
        origin=Origin(xyz=(0.10, 0.21, 0.105)),
        material=stainless,
        name="control_pod",
    )
    base.visual(
        Box((0.10, 0.14, 0.18)),
        origin=Origin(xyz=(-0.20, 0.0, 0.18)),
        material=stainless,
        name="hinge_tower",
    )
    base.visual(
        Cylinder(radius=0.018, length=0.16),
        origin=Origin(xyz=(-0.15, 0.0, 0.186), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_axle",
    )

    lid = model.part("lid")
    lid.visual(
        Box((0.34, 0.40, 0.06)),
        origin=Origin(xyz=(0.19, 0.0, 0.028)),
        material=stainless,
        name="lid_shell",
    )
    lid.visual(
        Box((0.33, 0.36, 0.024)),
        origin=Origin(xyz=(0.185, 0.0, -0.028)),
        material=cast_plate,
        name="lid_plate",
    )
    lid.visual(
        Box((0.33, 0.02, 0.06)),
        origin=Origin(xyz=(0.19, 0.19, 0.01)),
        material=stainless,
        name="left_skirt",
    )
    lid.visual(
        Box((0.33, 0.02, 0.06)),
        origin=Origin(xyz=(0.19, -0.19, 0.01)),
        material=stainless,
        name="right_skirt",
    )
    lid.visual(
        Box((0.028, 0.38, 0.052)),
        origin=Origin(xyz=(0.336, 0.0, 0.012)),
        material=stainless,
        name="front_skirt",
    )
    lid.visual(
        Box((0.03, 0.20, 0.07)),
        origin=Origin(xyz=(0.035, 0.0, 0.018)),
        material=dark_steel,
        name="rear_spine",
    )
    lid.visual(
        Box((0.032, 0.028, 0.046)),
        origin=Origin(xyz=(0.306, 0.125, 0.076)),
        material=dark_steel,
        name="handle_post_0",
    )
    lid.visual(
        Box((0.032, 0.028, 0.046)),
        origin=Origin(xyz=(0.306, -0.125, 0.076)),
        material=dark_steel,
        name="handle_post_1",
    )
    lid.visual(
        Cylinder(radius=0.019, length=0.30),
        origin=Origin(xyz=(0.286, 0.0, 0.107), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=phenolic,
        name="handle_bar",
    )

    grease_drawer = model.part("grease_drawer")
    grease_drawer.visual(
        Box((0.024, 0.30, 0.07)),
        origin=Origin(xyz=(0.012, 0.0, 0.035)),
        material=stainless,
        name="drawer_panel",
    )
    grease_drawer.visual(
        Box((0.18, 0.28, 0.006)),
        origin=Origin(xyz=(-0.09, 0.0, 0.003)),
        material=stainless,
        name="drawer_body",
    )
    grease_drawer.visual(
        Box((0.18, 0.006, 0.045)),
        origin=Origin(xyz=(-0.09, 0.137, 0.0285)),
        material=stainless,
        name="drawer_side_0",
    )
    grease_drawer.visual(
        Box((0.18, 0.006, 0.045)),
        origin=Origin(xyz=(-0.09, -0.137, 0.0285)),
        material=stainless,
        name="drawer_side_1",
    )
    grease_drawer.visual(
        Box((0.006, 0.28, 0.045)),
        origin=Origin(xyz=(-0.177, 0.0, 0.0285)),
        material=stainless,
        name="drawer_rear",
    )
    grease_drawer.visual(
        Box((0.012, 0.22, 0.022)),
        origin=Origin(xyz=(0.024, 0.0, 0.046)),
        material=phenolic,
        name="drawer_pull",
    )

    heat_knob = model.part("heat_knob")
    heat_knob.visual(
        Cylinder(radius=0.029, length=0.036),
        origin=Origin(xyz=(0.0, 0.018, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=phenolic,
        name="knob_body",
    )
    heat_knob.visual(
        Cylinder(radius=0.012, length=0.012),
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="knob_hub",
    )
    heat_knob.visual(
        Box((0.010, 0.006, 0.018)),
        origin=Origin(xyz=(0.014, 0.033, 0.015)),
        material=marker_red,
        name="pointer",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(-0.15, 0.0, 0.186)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.18, effort=80.0, velocity=1.0),
    )
    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=grease_drawer,
        origin=Origin(xyz=(0.248, 0.0, 0.02)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.11, effort=30.0, velocity=0.20),
    )
    model.articulation(
        "knob_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=heat_knob,
        origin=Origin(xyz=(0.10, 0.23, 0.105)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )

    return model


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    low, high = aabb
    return tuple((low[i] + high[i]) / 2.0 for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    grease_drawer = object_model.get_part("grease_drawer")
    heat_knob = object_model.get_part("heat_knob")

    lid_hinge = object_model.get_articulation("lid_hinge")
    drawer_slide = object_model.get_articulation("drawer_slide")
    knob_spin = object_model.get_articulation("knob_spin")

    ctx.expect_gap(
        lid,
        base,
        axis="z",
        positive_elem="lid_plate",
        negative_elem="lower_plate",
        min_gap=0.0005,
        max_gap=0.004,
        name="lid plate settles just above lower plate",
    )
    ctx.expect_overlap(
        lid,
        base,
        axes="xy",
        elem_a="lid_plate",
        elem_b="lower_plate",
        min_overlap=0.29,
        name="lid plate covers the cooking plate",
    )
    ctx.expect_within(
        grease_drawer,
        base,
        axes="y",
        inner_elem="drawer_body",
        outer_elem="drawer_channel",
        margin=0.011,
        name="drawer body stays centered in the channel",
    )
    ctx.expect_overlap(
        grease_drawer,
        base,
        axes="x",
        elem_a="drawer_body",
        elem_b="drawer_channel",
        min_overlap=0.165,
        name="closed grease drawer remains deeply inserted",
    )

    closed_handle = _aabb_center(ctx.part_element_world_aabb(lid, elem="handle_bar"))
    closed_drawer = ctx.part_world_position(grease_drawer)
    closed_pointer = _aabb_center(ctx.part_element_world_aabb(heat_knob, elem="pointer"))

    with ctx.pose({lid_hinge: 1.18}):
        open_handle = _aabb_center(ctx.part_element_world_aabb(lid, elem="handle_bar"))
    ctx.check(
        "lid opens upward on the rear hinge tower",
        closed_handle is not None
        and open_handle is not None
        and open_handle[2] > closed_handle[2] + 0.18,
        details=f"closed_handle={closed_handle}, open_handle={open_handle}",
    )

    with ctx.pose({drawer_slide: 0.11}):
        ctx.expect_within(
            grease_drawer,
            base,
            axes="y",
            inner_elem="drawer_body",
            outer_elem="drawer_channel",
            margin=0.011,
            name="extended drawer stays centered in the channel",
        )
        ctx.expect_overlap(
            grease_drawer,
            base,
            axes="x",
            elem_a="drawer_body",
            elem_b="drawer_channel",
            min_overlap=0.06,
            name="extended drawer still retains insertion",
        )
        open_drawer = ctx.part_world_position(grease_drawer)
    ctx.check(
        "grease drawer slides forward from the base front",
        closed_drawer is not None
        and open_drawer is not None
        and open_drawer[0] > closed_drawer[0] + 0.08,
        details=f"closed_drawer={closed_drawer}, open_drawer={open_drawer}",
    )

    with ctx.pose({knob_spin: math.pi / 2.0}):
        turned_pointer = _aabb_center(ctx.part_element_world_aabb(heat_knob, elem="pointer"))
    ctx.check(
        "heat-control knob visibly rotates",
        closed_pointer is not None
        and turned_pointer is not None
        and math.dist(closed_pointer, turned_pointer) > 0.015,
        details=f"closed_pointer={closed_pointer}, turned_pointer={turned_pointer}",
    )

    return ctx.report()


object_model = build_object_model()
