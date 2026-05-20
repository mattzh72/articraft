from __future__ import annotations

from math import pi

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


BODY_DEPTH = 0.46
BODY_WIDTH = 0.39
BODY_HEIGHT = 0.33
SIDE_THICKNESS = 0.03
INNER_WIDTH = BODY_WIDTH - 2.0 * SIDE_THICKNESS

STEAM_SWING = 1.0
WATER_SWING = 1.0


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    lo, hi = aabb
    return tuple((a + b) * 0.5 for a, b in zip(lo, hi))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_boiler_espresso_machine")

    stainless = model.material("stainless", rgba=(0.80, 0.81, 0.82, 1.0))
    dark = model.material("dark", rgba=(0.14, 0.15, 0.16, 1.0))
    black = model.material("black", rgba=(0.08, 0.08, 0.08, 1.0))
    metal = model.material("metal", rgba=(0.70, 0.72, 0.74, 1.0))
    knob = model.material("knob", rgba=(0.24, 0.17, 0.11, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_DEPTH, INNER_WIDTH, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=stainless,
        name="base_plate",
    )
    body.visual(
        Box((0.22, INNER_WIDTH, 0.03)),
        origin=Origin(xyz=(0.09, 0.0, 0.045)),
        material=dark,
        name="tray_deck",
    )
    body.visual(
        Box((0.06, INNER_WIDTH, 0.08)),
        origin=Origin(xyz=(0.20, 0.0, 0.07)),
        material=stainless,
        name="lower_panel",
    )
    body.visual(
        Box((BODY_DEPTH, SIDE_THICKNESS, BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.18, BODY_HEIGHT * 0.5)),
        material=stainless,
        name="left_side",
    )
    body.visual(
        Box((BODY_DEPTH, SIDE_THICKNESS, BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.18, BODY_HEIGHT * 0.5)),
        material=stainless,
        name="right_side",
    )
    body.visual(
        Box((BODY_DEPTH, INNER_WIDTH, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.31)),
        material=stainless,
        name="roof",
    )
    body.visual(
        Box((0.06, INNER_WIDTH, 0.13)),
        origin=Origin(xyz=(0.20, 0.0, 0.245)),
        material=stainless,
        name="front_panel",
    )
    body.visual(
        Box((0.05, INNER_WIDTH, 0.28)),
        origin=Origin(xyz=(-0.205, 0.0, 0.17)),
        material=stainless,
        name="rear_wall",
    )
    body.visual(
        Box((0.05, 0.28, 0.02)),
        origin=Origin(xyz=(0.10, 0.0, 0.06)),
        material=black,
        name="tray_lip",
    )
    body.visual(
        Box((0.045, 0.09, 0.06)),
        origin=Origin(xyz=(0.2375, 0.0, 0.205)),
        material=metal,
        name="group_mount",
    )
    body.visual(
        Cylinder(radius=0.03, length=0.055),
        origin=Origin(xyz=(0.2675, 0.0, 0.205), rpy=(0.0, pi * 0.5, 0.0)),
        material=metal,
        name="group_head",
    )
    body.visual(
        Box((0.03, 0.03, 0.03)),
        origin=Origin(xyz=(0.215, 0.185, 0.25)),
        material=metal,
        name="steam_pivot_block",
    )
    body.visual(
        Box((0.03, 0.03, 0.03)),
        origin=Origin(xyz=(0.215, -0.185, 0.25)),
        material=metal,
        name="water_pivot_block",
    )

    rail_post_x = (-0.17, 0.13)
    rail_post_y = (-0.155, 0.155)
    for idx, x in enumerate(rail_post_x):
        for idy, y in enumerate(rail_post_y):
            body.visual(
                Cylinder(radius=0.005, length=0.03),
                origin=Origin(xyz=(x, y, 0.345)),
                material=metal,
                name=f"rail_post_{idx}_{idy}",
            )
    body.visual(
        Cylinder(radius=0.005, length=0.30),
        origin=Origin(xyz=(-0.02, 0.155, 0.36), rpy=(0.0, pi * 0.5, 0.0)),
        material=metal,
        name="rail_left",
    )
    body.visual(
        Cylinder(radius=0.005, length=0.30),
        origin=Origin(xyz=(-0.02, -0.155, 0.36), rpy=(0.0, pi * 0.5, 0.0)),
        material=metal,
        name="rail_right",
    )
    body.visual(
        Cylinder(radius=0.005, length=0.31),
        origin=Origin(xyz=(0.13, 0.0, 0.36), rpy=(-pi * 0.5, 0.0, 0.0)),
        material=metal,
        name="rail_front",
    )

    portafilter = model.part("portafilter")
    portafilter.visual(
        Cylinder(radius=0.029, length=0.03),
        origin=Origin(xyz=(-0.015, 0.0, -0.01), rpy=(0.0, pi * 0.5, 0.0)),
        material=metal,
        name="portafilter_head",
    )
    portafilter.visual(
        Cylinder(radius=0.03, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, -0.06)),
        material=metal,
        name="basket",
    )
    portafilter.visual(
        Box((0.13, 0.026, 0.022)),
        origin=Origin(xyz=(0.08, 0.0, -0.022)),
        material=knob,
        name="handle",
    )

    steam_wand = model.part("steam_wand")
    steam_wand.visual(
        Cylinder(radius=0.009, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=metal,
        name="pivot_collar",
    )
    steam_wand.visual(
        Box((0.012, 0.032, 0.012)),
        origin=Origin(xyz=(0.0, 0.016, -0.008)),
        material=metal,
        name="arm",
    )
    steam_wand.visual(
        Cylinder(radius=0.005, length=0.18),
        origin=Origin(xyz=(0.0, 0.032, -0.098)),
        material=metal,
        name="tube",
    )
    steam_wand.visual(
        Cylinder(radius=0.0065, length=0.02),
        origin=Origin(xyz=(0.0, 0.032, -0.19)),
        material=metal,
        name="tip",
    )

    water_wand = model.part("water_wand")
    water_wand.visual(
        Cylinder(radius=0.009, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=metal,
        name="pivot_collar",
    )
    water_wand.visual(
        Box((0.012, 0.032, 0.012)),
        origin=Origin(xyz=(0.0, -0.016, -0.008)),
        material=metal,
        name="arm",
    )
    water_wand.visual(
        Cylinder(radius=0.0045, length=0.15),
        origin=Origin(xyz=(0.0, -0.032, -0.083)),
        material=metal,
        name="tube",
    )
    water_wand.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(xyz=(0.0, -0.032, -0.161)),
        material=metal,
        name="tip",
    )

    shelf = model.part("shelf")
    shelf.visual(
        Box((0.015, 0.22, 0.10)),
        origin=Origin(xyz=(0.0075, 0.0, 0.05)),
        material=dark,
        name="shelf_panel",
    )
    shelf.visual(
        Box((0.02, 0.22, 0.012)),
        origin=Origin(xyz=(0.011, 0.0, 0.094)),
        material=metal,
        name="shelf_lip",
    )

    hatch = model.part("hatch")
    hatch.visual(
        Box((0.18, 0.22, 0.007)),
        origin=Origin(xyz=(0.09, 0.0, 0.0035)),
        material=stainless,
        name="hatch_panel",
    )
    hatch.visual(
        Box((0.03, 0.07, 0.01)),
        origin=Origin(xyz=(0.155, 0.0, 0.0085)),
        material=black,
        name="hatch_pull",
    )

    model.articulation(
        "brew_lock",
        ArticulationType.REVOLUTE,
        parent=body,
        child=portafilter,
        origin=Origin(xyz=(0.302, 0.0, 0.205)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=4.0, lower=-0.85, upper=0.85),
    )
    model.articulation(
        "steam_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=steam_wand,
        origin=Origin(xyz=(0.215, 0.204, 0.235)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=-STEAM_SWING, upper=0.25),
    )
    model.articulation(
        "water_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=water_wand,
        origin=Origin(xyz=(0.215, -0.204, 0.235)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=-0.25, upper=WATER_SWING),
    )
    model.articulation(
        "shelf_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=shelf,
        origin=Origin(xyz=(0.23, 0.0, 0.065)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.5),
    )
    model.articulation(
        "hatch_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=hatch,
        origin=Origin(xyz=(-0.16, 0.0, 0.33)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    portafilter = object_model.get_part("portafilter")
    steam_wand = object_model.get_part("steam_wand")
    water_wand = object_model.get_part("water_wand")
    shelf = object_model.get_part("shelf")
    hatch = object_model.get_part("hatch")

    brew_lock = object_model.get_articulation("brew_lock")
    steam_pivot = object_model.get_articulation("steam_pivot")
    water_pivot = object_model.get_articulation("water_pivot")
    shelf_hinge = object_model.get_articulation("shelf_hinge")
    hatch_hinge = object_model.get_articulation("hatch_hinge")

    ctx.allow_overlap(
        body,
        portafilter,
        elem_a="group_head",
        elem_b="portafilter_head",
        reason="The group head is simplified as a solid outer casting around the inserted portafilter head.",
    )

    ctx.expect_gap(
        shelf,
        body,
        axis="x",
        positive_elem="shelf_panel",
        negative_elem="lower_panel",
        min_gap=0.0,
        max_gap=0.02,
        name="shelf closes nearly flush to the lower front panel",
    )
    ctx.expect_gap(
        hatch,
        body,
        axis="z",
        positive_elem="hatch_panel",
        negative_elem="roof",
        min_gap=0.0,
        max_gap=0.01,
        name="rear hatch sits on the top deck when closed",
    )
    ctx.expect_gap(
        body,
        steam_wand,
        axis="z",
        positive_elem="steam_pivot_block",
        negative_elem="pivot_collar",
        min_gap=0.0,
        max_gap=0.001,
        name="steam wand collar hangs directly below its pivot block",
    )
    ctx.expect_gap(
        body,
        water_wand,
        axis="z",
        positive_elem="water_pivot_block",
        negative_elem="pivot_collar",
        min_gap=0.0,
        max_gap=0.001,
        name="hot water wand collar hangs directly below its pivot block",
    )

    portafilter_rest = _aabb_center(ctx.part_element_world_aabb(portafilter, elem="handle"))
    with ctx.pose({brew_lock: 0.75}):
        portafilter_rotated = _aabb_center(ctx.part_element_world_aabb(portafilter, elem="handle"))
    ctx.check(
        "portafilter handle rotates around the brew axis",
        portafilter_rest is not None
        and portafilter_rotated is not None
        and abs(portafilter_rotated[1] - portafilter_rest[1]) > 0.05,
        details=f"rest={portafilter_rest}, rotated={portafilter_rotated}",
    )

    steam_rest = _aabb_center(ctx.part_element_world_aabb(steam_wand, elem="tube"))
    with ctx.pose({steam_pivot: -STEAM_SWING}):
        steam_swung = _aabb_center(ctx.part_element_world_aabb(steam_wand, elem="tube"))
    ctx.check(
        "steam wand swings forward from its support pivot",
        steam_rest is not None and steam_swung is not None and steam_swung[0] > steam_rest[0] + 0.02,
        details=f"rest={steam_rest}, swung={steam_swung}",
    )

    water_rest = _aabb_center(ctx.part_element_world_aabb(water_wand, elem="tube"))
    with ctx.pose({water_pivot: WATER_SWING}):
        water_swung = _aabb_center(ctx.part_element_world_aabb(water_wand, elem="tube"))
    ctx.check(
        "hot water wand swings forward from its support pivot",
        water_rest is not None and water_swung is not None and water_swung[0] > water_rest[0] + 0.02,
        details=f"rest={water_rest}, swung={water_swung}",
    )

    shelf_rest = _aabb_center(ctx.part_element_world_aabb(shelf, elem="shelf_panel"))
    with ctx.pose({shelf_hinge: 1.35}):
        shelf_open = _aabb_center(ctx.part_element_world_aabb(shelf, elem="shelf_panel"))
    ctx.check(
        "accessory shelf folds down and outward",
        shelf_rest is not None
        and shelf_open is not None
        and shelf_open[0] > shelf_rest[0] + 0.04
        and shelf_open[2] < shelf_rest[2] - 0.02,
        details=f"rest={shelf_rest}, open={shelf_open}",
    )

    hatch_rest = _aabb_center(ctx.part_element_world_aabb(hatch, elem="hatch_panel"))
    with ctx.pose({hatch_hinge: 1.0}):
        hatch_open = _aabb_center(ctx.part_element_world_aabb(hatch, elem="hatch_panel"))
    ctx.check(
        "rear hatch lifts upward on its top hinge",
        hatch_rest is not None
        and hatch_open is not None
        and hatch_open[2] > hatch_rest[2] + 0.05,
        details=f"rest={hatch_rest}, open={hatch_open}",
    )

    return ctx.report()


object_model = build_object_model()
