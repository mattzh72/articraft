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
    model = ArticulatedObject(name="tilting_drafting_table")

    wood = model.material("warm_beech", rgba=(0.72, 0.48, 0.25, 1.0))
    dark_wood = model.material("dark_edge_wood", rgba=(0.38, 0.22, 0.10, 1.0))
    black_metal = model.material("satin_black_steel", rgba=(0.03, 0.035, 0.04, 1.0))
    bright_metal = model.material("brushed_steel", rgba=(0.62, 0.65, 0.67, 1.0))
    drawer_mat = model.material("pale_drawer_wood", rgba=(0.64, 0.43, 0.24, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))

    frame = model.part("frame")
    # Four-legged welded steel base, sized like a real drafting table.
    for x in (-0.38, 0.38):
        for y in (-0.60, 0.60):
            frame.visual(
                Box((0.060, 0.060, 0.760)),
                origin=Origin(xyz=(x, y, 0.380)),
                material=black_metal,
                name=f"leg_{0 if x < 0 else 1}_{0 if y < 0 else 1}",
            )
            frame.visual(
                Box((0.105, 0.105, 0.024)),
                origin=Origin(xyz=(x, y, 0.012)),
                material=rubber,
                name=f"foot_{0 if x < 0 else 1}_{0 if y < 0 else 1}",
            )

    frame.visual(
        Box((0.090, 1.300, 0.070)),
        origin=Origin(xyz=(0.38, 0.0, 0.755)),
        material=black_metal,
        name="front_rail",
    )
    frame.visual(
        Box((0.090, 1.300, 0.070)),
        origin=Origin(xyz=(-0.38, 0.0, 0.755)),
        material=black_metal,
        name="rear_rail",
    )
    frame.visual(
        Box((0.860, 0.055, 0.070)),
        origin=Origin(xyz=(0.0, -0.60, 0.755)),
        material=black_metal,
        name="side_rail_0",
    )
    frame.visual(
        Box((0.810, 0.050, 0.045)),
        origin=Origin(xyz=(0.0, -0.60, 0.285)),
        material=black_metal,
        name="lower_side_rail_0",
    )
    frame.visual(
        Box((0.860, 0.055, 0.070)),
        origin=Origin(xyz=(0.0, 0.60, 0.755)),
        material=black_metal,
        name="side_rail_1",
    )
    frame.visual(
        Box((0.810, 0.050, 0.045)),
        origin=Origin(xyz=(0.0, 0.60, 0.285)),
        material=black_metal,
        name="lower_side_rail_1",
    )

    frame.visual(
        Box((0.090, 1.300, 0.045)),
        origin=Origin(xyz=(0.38, 0.0, 0.285)),
        material=black_metal,
        name="lower_front_rail",
    )
    frame.visual(
        Box((0.090, 1.300, 0.045)),
        origin=Origin(xyz=(-0.38, 0.0, 0.285)),
        material=black_metal,
        name="lower_rear_rail",
    )

    # Rear hinge support spine and small hinge leaves just behind the tabletop edge.
    frame.visual(
        Box((0.100, 1.300, 0.045)),
        origin=Origin(xyz=(-0.455, 0.0, 0.800)),
        material=black_metal,
        name="hinge_spine",
    )
    for y, suffix in ((-0.50, "0"), (0.0, "1"), (0.50, "2")):
        frame.visual(
            Box((0.035, 0.100, 0.090)),
            origin=Origin(xyz=(-0.490, y, 0.835)),
            material=black_metal,
            name=f"hinge_leaf_{suffix}",
        )

    # Static guide rails for the single front-access drawer.
    frame.visual(
        Box((0.680, 0.030, 0.040)),
        origin=Origin(xyz=(0.100, -0.390, 0.622)),
        material=bright_metal,
        name="guide_rail_0",
    )
    frame.visual(
        Box((0.055, 0.055, 0.205)),
        origin=Origin(xyz=(0.360, -0.431, 0.690)),
        material=black_metal,
        name="rail_hanger_0",
    )
    frame.visual(
        Box((0.680, 0.030, 0.040)),
        origin=Origin(xyz=(0.100, 0.390, 0.622)),
        material=bright_metal,
        name="guide_rail_1",
    )
    frame.visual(
        Box((0.055, 0.055, 0.205)),
        origin=Origin(xyz=(0.360, 0.431, 0.690)),
        material=black_metal,
        name="rail_hanger_1",
    )
    frame.visual(
        Box((0.060, 0.930, 0.035)),
        origin=Origin(xyz=(0.360, 0.0, 0.785)),
        material=black_metal,
        name="drawer_crossmember",
    )

    tabletop = model.part("tabletop")
    # Child frame is the rear top hinge line. The broad board extends forward
    # along local +X and tilts upward about local -Y.
    tabletop.visual(
        Box((0.900, 1.350, 0.040)),
        origin=Origin(xyz=(0.450, 0.0, -0.020)),
        material=wood,
        name="table_panel",
    )
    tabletop.visual(
        Box((0.045, 1.350, 0.045)),
        origin=Origin(xyz=(0.880, 0.0, 0.022)),
        material=dark_wood,
        name="front_lip",
    )
    for y, suffix in ((-0.665, "0"), (0.665, "1")):
        tabletop.visual(
            Box((0.900, 0.035, 0.045)),
            origin=Origin(xyz=(0.450, y, -0.004)),
            material=dark_wood,
            name=f"edge_strip_{suffix}",
        )
    tabletop.visual(
        Cylinder(radius=0.018, length=1.260),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bright_metal,
        name="hinge_barrel",
    )
    tabletop.visual(
        Box((0.055, 1.150, 0.030)),
        origin=Origin(xyz=(0.410, 0.0, -0.055)),
        material=dark_wood,
        name="underside_stiffener",
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.590, 0.660, 0.018)),
        origin=Origin(xyz=(0.015, 0.0, -0.065)),
        material=drawer_mat,
        name="drawer_bottom",
    )
    drawer.visual(
        Box((0.590, 0.030, 0.125)),
        origin=Origin(xyz=(0.015, -0.330, 0.000)),
        material=drawer_mat,
        name="drawer_side_0",
    )
    drawer.visual(
        Box((0.500, 0.034, 0.032)),
        origin=Origin(xyz=(-0.020, -0.358, -0.012)),
        material=bright_metal,
        name="runner_0",
    )
    drawer.visual(
        Box((0.590, 0.030, 0.125)),
        origin=Origin(xyz=(0.015, 0.330, 0.000)),
        material=drawer_mat,
        name="drawer_side_1",
    )
    drawer.visual(
        Box((0.500, 0.034, 0.032)),
        origin=Origin(xyz=(-0.020, 0.358, -0.012)),
        material=bright_metal,
        name="runner_1",
    )
    drawer.visual(
        Box((0.026, 0.660, 0.125)),
        origin=Origin(xyz=(-0.286, 0.0, 0.000)),
        material=drawer_mat,
        name="drawer_back",
    )
    drawer.visual(
        Box((0.044, 0.780, 0.180)),
        origin=Origin(xyz=(0.330, 0.0, 0.000)),
        material=dark_wood,
        name="drawer_front",
    )
    for y, suffix in ((-0.220, "0"), (0.220, "1")):
        drawer.visual(
            Box((0.090, 0.030, 0.030)),
            origin=Origin(xyz=(0.315, y, 0.018)),
            material=bright_metal,
            name=f"handle_post_{suffix}",
        )
    drawer.visual(
        Box((0.045, 0.500, 0.035)),
        origin=Origin(xyz=(0.365, 0.0, 0.018)),
        material=bright_metal,
        name="handle_bar",
    )

    model.articulation(
        "top_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=tabletop,
        origin=Origin(xyz=(-0.450, 0.0, 0.860)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.8, lower=0.0, upper=1.05),
    )
    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=drawer,
        origin=Origin(xyz=(0.140, 0.0, 0.620)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    tabletop = object_model.get_part("tabletop")
    drawer = object_model.get_part("drawer")
    top_hinge = object_model.get_articulation("top_hinge")
    drawer_slide = object_model.get_articulation("drawer_slide")

    ctx.expect_gap(
        tabletop,
        frame,
        axis="z",
        positive_elem="table_panel",
        negative_elem="side_rail_0",
        min_gap=0.005,
        max_gap=0.030,
        name="closed tabletop clears the support frame",
    )
    ctx.expect_overlap(
        tabletop,
        frame,
        axes="y",
        elem_a="hinge_barrel",
        elem_b="hinge_spine",
        min_overlap=1.0,
        name="wide rear hinge spans the table",
    )

    closed_front = ctx.part_element_world_aabb(tabletop, elem="front_lip")
    with ctx.pose({top_hinge: 0.95}):
        tilted_front = ctx.part_element_world_aabb(tabletop, elem="front_lip")
    ctx.check(
        "top tilts upward from rear hinge",
        closed_front is not None
        and tilted_front is not None
        and tilted_front[1][2] > closed_front[1][2] + 0.45,
        details=f"closed={closed_front}, tilted={tilted_front}",
    )

    ctx.expect_overlap(
        drawer,
        frame,
        axes="x",
        elem_a="runner_1",
        elem_b="guide_rail_1",
        min_overlap=0.40,
        name="closed drawer runners are captured in guide rails",
    )
    rest_drawer = ctx.part_world_position(drawer)
    with ctx.pose({drawer_slide: 0.35}):
        ctx.expect_overlap(
            drawer,
            frame,
            axes="x",
            elem_a="runner_1",
            elem_b="guide_rail_1",
            min_overlap=0.18,
            name="extended drawer keeps retained rail insertion",
        )
        extended_drawer = ctx.part_world_position(drawer)
    ctx.check(
        "drawer slides forward",
        rest_drawer is not None
        and extended_drawer is not None
        and extended_drawer[0] > rest_drawer[0] + 0.30,
        details=f"rest={rest_drawer}, extended={extended_drawer}",
    )

    return ctx.report()


object_model = build_object_model()
