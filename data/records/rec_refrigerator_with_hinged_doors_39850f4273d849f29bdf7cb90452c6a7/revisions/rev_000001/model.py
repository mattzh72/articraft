from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


CABINET_W = 0.92
CABINET_D = 0.62
CABINET_H = 1.82
WALL_T = 0.036
DOOR_T = 0.048
DOOR_H = 1.74
DOOR_BOTTOM = 0.045
CENTER_GAP = 0.012
SIDE_REVEAL = 0.008
DOOR_W = CABINET_W / 2.0 - CENTER_GAP / 2.0 - SIDE_REVEAL
HINGE_Y = -CABINET_D / 2.0


def _union_boxes(boxes: list[tuple[tuple[float, float, float], tuple[float, float, float]]]) -> cq.Workplane:
    """Return one fused CadQuery solid from (size, center) box descriptions."""
    solid: cq.Workplane | None = None
    for size, center in boxes:
        body = cq.Workplane("XY").box(*size).translate(center)
        solid = body if solid is None else solid.union(body)
    assert solid is not None
    return solid


def _cabinet_shell() -> cq.Workplane:
    w, d, h, t = CABINET_W, CABINET_D, CABINET_H, WALL_T
    boxes: list[tuple[tuple[float, float, float], tuple[float, float, float]]] = [
        ((w, d, t), (0.0, 0.0, t / 2.0)),
        ((w, d, t), (0.0, 0.0, h - t / 2.0)),
        ((t, d, h), (-w / 2.0 + t / 2.0, 0.0, h / 2.0)),
        ((t, d, h), (w / 2.0 - t / 2.0, 0.0, h / 2.0)),
        ((w, t, h), (0.0, d / 2.0 - t / 2.0, h / 2.0)),
        ((t * 0.82, d - t, h - 2.0 * t), (0.0, -t / 2.0, h / 2.0)),
        ((w - 2.0 * t, d * 0.70, t * 0.58), (0.0, 0.035, 0.72)),
        ((w - 2.0 * t, d * 0.70, t * 0.58), (0.0, 0.035, 1.20)),
    ]
    return _union_boxes(boxes)


def _door_panel(direction: float) -> cq.Workplane:
    panel = (
        cq.Workplane("XY")
        .box(DOOR_W, DOOR_T, DOOR_H)
        .translate((direction * DOOR_W / 2.0, -DOOR_T / 2.0, DOOR_BOTTOM + DOOR_H / 2.0))
    )
    return panel.edges("|Z").fillet(0.010)


def _inner_bin(direction: float) -> cq.Workplane:
    """A shallow molded lower door bin/ledge that supports the fold-out rail."""
    width = DOOR_W * 0.76
    x_center = direction * DOOR_W * 0.52
    boxes = [
        ((width, 0.074, 0.030), (x_center, 0.037, 0.255)),
        ((0.026, 0.074, 0.115), (x_center - direction * width / 2.0, 0.037, 0.300)),
        ((0.026, 0.074, 0.115), (x_center + direction * width / 2.0, 0.037, 0.300)),
        ((width, 0.018, 0.080), (x_center, 0.072, 0.332)),
    ]
    return _union_boxes(boxes)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="twin_door_pantry_refrigerator")

    enamel = model.material("warm_white_enamel", rgba=(0.90, 0.92, 0.91, 1.0))
    inner = model.material("matte_inner_liner", rgba=(0.78, 0.82, 0.80, 1.0))
    handle_mat = model.material("brushed_dark_handle", rgba=(0.13, 0.14, 0.15, 1.0))
    rail_mat = model.material("coated_wire_rail", rgba=(0.84, 0.88, 0.87, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(_cabinet_shell(), "divided_refrigerator_cabinet", tolerance=0.0015),
        material=inner,
        name="cabinet_shell",
    )

    left_door = model.part("left_door")
    left_door.visual(
        mesh_from_cadquery(_door_panel(1.0), "left_rounded_door", tolerance=0.0015),
        material=enamel,
        name="door_panel",
    )
    left_door.visual(
        Box((0.026, 0.016, 1.06)),
        origin=Origin(xyz=(DOOR_W - 0.052, -DOOR_T - 0.007, 0.92)),
        material=handle_mat,
        name="vertical_handle",
    )

    right_door = model.part("right_door")
    right_door.visual(
        mesh_from_cadquery(_door_panel(-1.0), "right_rounded_door", tolerance=0.0015),
        material=enamel,
        name="door_panel",
    )
    right_door.visual(
        Box((0.026, 0.016, 1.06)),
        origin=Origin(xyz=(-DOOR_W + 0.052, -DOOR_T - 0.007, 0.92)),
        material=handle_mat,
        name="vertical_handle",
    )
    right_door.visual(
        mesh_from_cadquery(_inner_bin(-1.0), "right_lower_door_bin", tolerance=0.0015),
        material=enamel,
        name="lower_bin",
    )
    right_door.visual(
        Box((0.050, 0.032, 0.030)),
        origin=Origin(xyz=(-DOOR_W * 0.84, 0.008, 0.400)),
        material=handle_mat,
        name="rail_hinge_block_0",
    )
    right_door.visual(
        Box((0.050, 0.032, 0.030)),
        origin=Origin(xyz=(-DOOR_W * 0.20, 0.008, 0.400)),
        material=handle_mat,
        name="rail_hinge_block_1",
    )

    bottle_rail = model.part("bottle_rail")
    rail_span = DOOR_W * 0.64
    bottle_rail.visual(
        Cylinder(radius=0.009, length=rail_span),
        origin=Origin(xyz=(0.0, 0.011, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rail_mat,
        name="hinge_tube",
    )
    bottle_rail.visual(
        Cylinder(radius=0.008, length=rail_span),
        origin=Origin(xyz=(0.0, 0.026, 0.160), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rail_mat,
        name="retainer_bar",
    )
    for i, x in enumerate((-rail_span / 2.0 + 0.020, rail_span / 2.0 - 0.020)):
        bottle_rail.visual(
            Cylinder(radius=0.007, length=0.160),
            origin=Origin(xyz=(x, 0.026, 0.080), rpy=(0.0, 0.0, 0.0)),
            material=rail_mat,
            name=f"side_arm_{i}",
        )

    model.articulation(
        "cabinet_to_left_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=left_door,
        origin=Origin(xyz=(-CABINET_W / 2.0 + SIDE_REVEAL, HINGE_Y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.85, effort=18.0, velocity=1.2),
    )
    model.articulation(
        "cabinet_to_right_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=right_door,
        origin=Origin(xyz=(CABINET_W / 2.0 - SIDE_REVEAL, HINGE_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.85, effort=18.0, velocity=1.2),
    )
    model.articulation(
        "right_door_to_bottle_rail",
        ArticulationType.REVOLUTE,
        parent=right_door,
        child=bottle_rail,
        origin=Origin(xyz=(-DOOR_W * 0.52, 0.018, 0.400)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.25, effort=2.0, velocity=1.5),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    bottle_rail = object_model.get_part("bottle_rail")
    left_hinge = object_model.get_articulation("cabinet_to_left_door")
    right_hinge = object_model.get_articulation("cabinet_to_right_door")
    rail_hinge = object_model.get_articulation("right_door_to_bottle_rail")

    ctx.allow_overlap(
        bottle_rail,
        right_door,
        elem_a="hinge_tube",
        elem_b="rail_hinge_block_0",
        reason="The folding bottle rail pin is intentionally captured inside the small hinge lug on the inner door.",
    )
    ctx.allow_overlap(
        bottle_rail,
        right_door,
        elem_a="hinge_tube",
        elem_b="rail_hinge_block_1",
        reason="The folding bottle rail pin is intentionally captured inside the second hinge lug on the inner door.",
    )
    ctx.expect_overlap(
        bottle_rail,
        right_door,
        axes="xz",
        min_overlap=0.010,
        elem_a="hinge_tube",
        elem_b="rail_hinge_block_0",
        name="rail pin is retained by first hinge lug",
    )
    ctx.expect_overlap(
        bottle_rail,
        right_door,
        axes="xz",
        min_overlap=0.010,
        elem_a="hinge_tube",
        elem_b="rail_hinge_block_1",
        name="rail pin is retained by second hinge lug",
    )

    ctx.expect_gap(
        cabinet,
        left_door,
        axis="y",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem="cabinet_shell",
        negative_elem="door_panel",
        name="left door closes just proud of cabinet opening",
    )
    ctx.expect_gap(
        cabinet,
        right_door,
        axis="y",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem="cabinet_shell",
        negative_elem="door_panel",
        name="right door closes just proud of cabinet opening",
    )
    ctx.expect_overlap(
        left_door,
        cabinet,
        axes="z",
        min_overlap=1.60,
        elem_a="door_panel",
        elem_b="cabinet_shell",
        name="left tall door covers the pantry height",
    )
    ctx.expect_overlap(
        right_door,
        cabinet,
        axes="z",
        min_overlap=1.60,
        elem_a="door_panel",
        elem_b="cabinet_shell",
        name="right tall door covers the pantry height",
    )

    left_closed = ctx.part_element_world_aabb(left_door, elem="door_panel")
    right_closed = ctx.part_element_world_aabb(right_door, elem="door_panel")
    with ctx.pose({left_hinge: 1.05, right_hinge: 1.05}):
        left_open = ctx.part_element_world_aabb(left_door, elem="door_panel")
        right_open = ctx.part_element_world_aabb(right_door, elem="door_panel")
    ctx.check(
        "both doors swing outward from side hinges",
        left_closed is not None
        and right_closed is not None
        and left_open is not None
        and right_open is not None
        and left_open[0][1] < left_closed[0][1] - 0.08
        and right_open[0][1] < right_closed[0][1] - 0.08,
        details=f"left_closed={left_closed}, left_open={left_open}, right_closed={right_closed}, right_open={right_open}",
    )

    rail_stowed = ctx.part_world_aabb(bottle_rail)
    with ctx.pose({rail_hinge: 1.10}):
        rail_folded = ctx.part_world_aabb(bottle_rail)
    ctx.check(
        "bottle rail folds out from the inner door panel",
        rail_stowed is not None
        and rail_folded is not None
        and rail_folded[1][1] > rail_stowed[1][1] + 0.07,
        details=f"stowed={rail_stowed}, folded={rail_folded}",
    )

    return ctx.report()


object_model = build_object_model()
