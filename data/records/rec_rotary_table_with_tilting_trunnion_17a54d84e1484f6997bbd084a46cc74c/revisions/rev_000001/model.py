from __future__ import annotations

from math import cos, pi, sin

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


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cylinder_x(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    # Workplane("YZ") extrudes along the world X axis.
    return (
        cq.Workplane("YZ")
        .circle(radius)
        .extrude(length)
        .translate((center[0] - length / 2.0, center[1], center[2]))
    )


def _side_frame_shape(x: float) -> cq.Workplane:
    # One connected side support with a bored trunnion bearing.  The lower foot
    # is intentionally modeled to sink slightly into the base slab so the root
    # part has a continuous support path.
    frame = _box((0.13, 0.62, 0.675), (x, 0.0, 0.4875))
    frame = frame.union(_box((0.21, 0.70, 0.10), (x, 0.0, 0.175)))

    inner_collar_x = x - 0.075 if x > 0.0 else x + 0.075
    frame = frame.union(_cylinder_x(0.145, 0.050, (inner_collar_x, 0.0, 0.54)))
    frame = frame.union(_cylinder_x(0.120, 0.040, (x, 0.0, 0.54)))

    for y in (-0.235, 0.235):
        frame = frame.union(_box((0.16, 0.070, 0.40), (x, y, 0.335)))

    frame = frame.cut(_cylinder_x(0.098, 0.40, (x, 0.0, 0.54)))
    return frame


def _rotary_table_shape() -> cq.Workplane:
    table = cq.Workplane("XY").circle(0.315).extrude(0.105).translate((0.0, 0.0, 0.022))
    table = table.edges(">Z").fillet(0.010).edges("<Z").fillet(0.006)

    # Center bore and bolt circle.
    table = table.cut(cq.Workplane("XY").circle(0.043).extrude(0.16).translate((0.0, 0.0, 0.0)))
    for i in range(6):
        angle = i * pi / 3.0
        x = 0.185 * cos(angle)
        y = 0.185 * sin(angle)
        hole = cq.Workplane("XY").circle(0.018).extrude(0.16).translate((x, y, 0.0))
        table = table.cut(hole)

    # Four radial T-slot grooves cut into the top face.
    for i in range(4):
        angle = i * pi / 2.0
        slot = (
            cq.Workplane("XY")
            .box(0.46, 0.042, 0.035)
            .translate((0.155, 0.0, 0.120))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle * 180.0 / pi)
        )
        table = table.cut(slot)

    return table


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="machine_tool_trunnion_table")

    cast_iron = model.material("dark_cast_iron", rgba=(0.12, 0.13, 0.14, 1.0))
    oiled_steel = model.material("oiled_steel", rgba=(0.36, 0.39, 0.40, 1.0))
    machined = model.material("machined_table_steel", rgba=(0.55, 0.58, 0.57, 1.0))
    blackened = model.material("blackened_hardware", rgba=(0.02, 0.02, 0.018, 1.0))

    base = model.part("base")
    base.visual(
        Box((1.45, 0.78, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=cast_iron,
        name="base_slab",
    )
    base.visual(
        mesh_from_cadquery(_side_frame_shape(-0.58), "side_frame_0", tolerance=0.0015),
        material=cast_iron,
        name="side_frame_0",
    )
    base.visual(
        mesh_from_cadquery(_side_frame_shape(0.58), "side_frame_1", tolerance=0.0015),
        material=cast_iron,
        name="side_frame_1",
    )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.085, length=1.20),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=oiled_steel,
        name="trunnion_shaft",
    )
    cradle.visual(
        Box((0.92, 0.16, 0.095)),
        origin=Origin(xyz=(0.0, 0.0, -0.050)),
        material=oiled_steel,
        name="cross_saddle",
    )
    cradle.visual(
        Cylinder(radius=0.185, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=oiled_steel,
        name="rotary_bearing",
    )
    for x in (-0.46, 0.46):
        cradle.visual(
            Cylinder(radius=0.092, length=0.070),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=oiled_steel,
            name=f"journal_{0 if x < 0 else 1}",
        )
    cradle.visual(
        Cylinder(radius=0.132, length=0.025),
        origin=Origin(xyz=(-0.4677, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=oiled_steel,
        name="thrust_washer_0",
    )
    cradle.visual(
        Cylinder(radius=0.132, length=0.025),
        origin=Origin(xyz=(0.4677, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=oiled_steel,
        name="thrust_washer_1",
    )

    rotary_table = model.part("rotary_table")
    rotary_table.visual(
        mesh_from_cadquery(_rotary_table_shape(), "slotted_rotary_table", tolerance=0.0012),
        material=machined,
        name="slotted_platter",
    )

    model.articulation(
        "base_to_cradle",
        ArticulationType.REVOLUTE,
        parent=base,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.54)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1800.0, velocity=0.35, lower=-1.05, upper=1.05),
    )

    model.articulation(
        "cradle_to_rotary_table",
        ArticulationType.CONTINUOUS,
        parent=cradle,
        child=rotary_table,
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    cradle = object_model.get_part("cradle")
    rotary_table = object_model.get_part("rotary_table")
    tilt = object_model.get_articulation("base_to_cradle")
    spin = object_model.get_articulation("cradle_to_rotary_table")

    # The trunnion shaft intentionally runs through the clear bearing bores in
    # the side frames, while the rotary table sits on the cradle bearing face.
    for side_frame, thrust_washer in (
        ("side_frame_0", "thrust_washer_0"),
        ("side_frame_1", "thrust_washer_1"),
    ):
        ctx.allow_overlap(
            base,
            cradle,
            elem_a=side_frame,
            elem_b=thrust_washer,
            reason=(
                "The trunnion thrust washer is intentionally seated a few "
                "millimeters into the side-frame bearing face to show the "
                "captured tilt support."
            ),
        )
    ctx.expect_gap(
        cradle,
        base,
        axis="x",
        positive_elem="thrust_washer_0",
        negative_elem="side_frame_0",
        max_penetration=0.006,
        name="first thrust washer is seated in its bearing face",
    )
    ctx.expect_gap(
        base,
        cradle,
        axis="x",
        positive_elem="side_frame_1",
        negative_elem="thrust_washer_1",
        max_penetration=0.006,
        name="second thrust washer is seated in its bearing face",
    )
    ctx.expect_within(
        cradle,
        base,
        axes="x",
        inner_elem="trunnion_shaft",
        margin=0.02,
        name="trunnion shaft spans the side supports",
    )
    ctx.expect_gap(
        rotary_table,
        cradle,
        axis="z",
        positive_elem="slotted_platter",
        negative_elem="rotary_bearing",
        max_gap=0.003,
        max_penetration=0.0,
        name="rotary platter seats on bearing",
    )

    rest_top = ctx.part_element_world_aabb(rotary_table, elem="slotted_platter")
    with ctx.pose({tilt: 0.65, spin: 1.2}):
        tilted_top = ctx.part_element_world_aabb(rotary_table, elem="slotted_platter")
        ctx.expect_origin_distance(
            cradle,
            base,
            axes="x",
            max_dist=0.001,
            name="tilt keeps the trunnion axis fixed in the side frames",
        )

    ctx.check(
        "tilt changes table height envelope",
        rest_top is not None
        and tilted_top is not None
        and abs((tilted_top[1][2] - tilted_top[0][2]) - (rest_top[1][2] - rest_top[0][2])) > 0.05,
        details=f"rest={rest_top}, tilted={tilted_top}",
    )

    return ctx.report()


object_model = build_object_model()
