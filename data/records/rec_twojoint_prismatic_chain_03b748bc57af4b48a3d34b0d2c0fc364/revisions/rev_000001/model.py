from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _tube_x(x0: float, x1: float, outer_r: float, inner_r: float) -> cq.Workplane:
    """Annular tube with its axis along local +X."""
    return (
        cq.Workplane("YZ")
        .circle(outer_r)
        .circle(inner_r)
        .extrude(x1 - x0)
        .translate((x0, 0.0, 0.0))
    )


def _cylinder_x(x0: float, x1: float, radius: float) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(x1 - x0).translate((x0, 0.0, 0.0))


def _box(cx: float, cy: float, cz: float, sx: float, sy: float, sz: float) -> cq.Workplane:
    return cq.Workplane("XY").box(sx, sy, sz).translate((cx, cy, cz))


def _cut_bolt_holes(plate: cq.Workplane, x0: float, x1: float, *, offset: float, radius: float) -> cq.Workplane:
    cutter = None
    for y in (-offset, offset):
        for z in (-offset, offset):
            hole = _cylinder_x(x0, x1, radius).translate((0.0, y, z))
            cutter = hole if cutter is None else cutter.union(hole)
    return plate.cut(cutter)


def _guide_pads_x(x0: float, x1: float, outer_r: float, tube_r: float, pad_width: float) -> cq.Workplane:
    """Four short bronze bearing pads that lightly touch a sliding round tube."""
    inner = tube_r - 0.0005
    radial = outer_r - inner
    mid = inner + radial / 2.0
    length = x1 - x0
    cx = x0 + length / 2.0
    top = _box(cx, 0.0, mid, length, pad_width, radial)
    bottom = _box(cx, 0.0, -mid, length, pad_width, radial)
    side_a = _box(cx, mid, 0.0, length, radial, pad_width)
    side_b = _box(cx, -mid, 0.0, length, radial, pad_width)
    return top.union(bottom).union(side_a).union(side_b).clean()


def _outer_sleeve_shape() -> cq.Workplane:
    sleeve = _tube_x(0.0, 1.02, 0.115, 0.088)
    rear_flange = _tube_x(-0.035, 0.075, 0.150, 0.088)
    front_collar = _tube_x(0.840, 1.020, 0.138, 0.088)
    rear_cap_lip = _tube_x(-0.060, -0.025, 0.132, 0.088)
    base = _box(0.420, 0.0, -0.165, 1.050, 0.360, 0.050)
    saddle_0 = _box(0.180, 0.0, -0.122, 0.170, 0.225, 0.070)
    saddle_1 = _box(0.650, 0.0, -0.122, 0.200, 0.225, 0.070)
    top_stop_pad = _box(0.965, 0.0, 0.152, 0.090, 0.105, 0.034)

    shape = (
        sleeve.union(rear_flange)
        .union(front_collar)
        .union(rear_cap_lip)
        .union(base)
        .union(saddle_0)
        .union(saddle_1)
        .union(top_stop_pad)
    )
    for x in (-0.020, 0.850):
        for y in (-0.125, 0.125):
            bolt_head = cq.Workplane("XY").circle(0.020).extrude(0.020).translate((x, y, -0.142))
            shape = shape.union(bolt_head)
    return shape.clean()


def _intermediate_shape() -> cq.Workplane:
    tube = _tube_x(-0.560, 0.680, 0.072, 0.054)
    rear_capture_stop = _tube_x(-0.540, -0.470, 0.084, 0.068)
    retraction_stop = _tube_x(0.125, 0.185, 0.095, 0.068)
    front_collar = _tube_x(0.570, 0.690, 0.092, 0.054)
    key_strip = _box(0.365, 0.0, 0.076, 0.360, 0.026, 0.016)
    stop_tab = _box(0.155, 0.0, 0.099, 0.060, 0.060, 0.022)
    return (
        tube.union(rear_capture_stop)
        .union(retraction_stop)
        .union(front_collar)
        .union(key_strip)
        .union(stop_tab)
        .clean()
    )


def _front_tube_shape() -> cq.Workplane:
    tube = _tube_x(-0.455, 0.575, 0.043, 0.027)
    rear_capture_stop = _tube_x(-0.445, -0.385, 0.052, 0.039)
    retraction_stop = _tube_x(0.090, 0.150, 0.063, 0.039)
    nose_hub = _tube_x(0.500, 0.590, 0.062, 0.039)
    return tube.union(rear_capture_stop).union(retraction_stop).union(nose_hub).clean()


def _output_plate_shape() -> cq.Workplane:
    plate = _box(0.605, 0.0, 0.0, 0.050, 0.250, 0.250)
    plate = _cut_bolt_holes(plate, 0.575, 0.635, offset=0.080, radius=0.014)
    boss = _tube_x(0.565, 0.625, 0.066, 0.039)
    return plate.union(boss).clean()


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="inline_telescoping_ram")

    dark_steel = model.material("dark_steel", rgba=(0.12, 0.13, 0.14, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.62, 0.65, 0.67, 1.0))
    polished_steel = model.material("polished_steel", rgba=(0.82, 0.84, 0.84, 1.0))
    bronze = model.material("oil_bronze", rgba=(0.72, 0.48, 0.20, 1.0))

    outer_sleeve = model.part("outer_sleeve")
    outer_sleeve.visual(
        mesh_from_cadquery(_outer_sleeve_shape(), "outer_sleeve_body"),
        material=dark_steel,
        name="sleeve_body",
    )
    outer_sleeve.visual(
        mesh_from_cadquery(_tube_x(0.920, 1.020, 0.090, 0.077), "outer_front_bushing"),
        material=bronze,
        name="front_bushing",
    )
    outer_sleeve.visual(
        mesh_from_cadquery(_guide_pads_x(0.945, 0.995, 0.089, 0.072, 0.018), "outer_front_guide_pads"),
        material=bronze,
        name="front_guide_pads",
    )
    outer_sleeve.visual(
        mesh_from_cadquery(_tube_x(0.060, 0.145, 0.090, 0.077), "outer_rear_bushing"),
        material=bronze,
        name="rear_bushing",
    )

    intermediate_tube = model.part("intermediate_tube")
    intermediate_tube.visual(
        mesh_from_cadquery(_intermediate_shape(), "intermediate_tube_body"),
        material=satin_steel,
        name="tube_body",
    )
    intermediate_tube.visual(
        mesh_from_cadquery(_tube_x(0.602, 0.690, 0.0555, 0.047), "intermediate_front_bushing"),
        material=bronze,
        name="front_bushing",
    )
    intermediate_tube.visual(
        mesh_from_cadquery(
            _guide_pads_x(0.625, 0.675, 0.055, 0.043, 0.014),
            "intermediate_front_guide_pads",
        ),
        material=bronze,
        name="front_guide_pads",
    )

    front_tube = model.part("front_tube")
    front_tube.visual(
        mesh_from_cadquery(_front_tube_shape(), "front_tube_body"),
        material=polished_steel,
        name="shaft_body",
    )
    front_tube.visual(
        mesh_from_cadquery(_output_plate_shape(), "front_output_plate"),
        material=dark_steel,
        name="output_plate",
    )

    model.articulation(
        "outer_to_intermediate",
        ArticulationType.PRISMATIC,
        parent=outer_sleeve,
        child=intermediate_tube,
        origin=Origin(xyz=(0.920, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2500.0, velocity=0.20, lower=0.0, upper=0.420),
        motion_properties=MotionProperties(damping=90.0, friction=18.0),
    )

    model.articulation(
        "intermediate_to_front",
        ArticulationType.PRISMATIC,
        parent=intermediate_tube,
        child=front_tube,
        origin=Origin(xyz=(0.600, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1800.0, velocity=0.23, lower=0.0, upper=0.350),
        motion_properties=MotionProperties(damping=70.0, friction=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_sleeve")
    intermediate = object_model.get_part("intermediate_tube")
    front = object_model.get_part("front_tube")
    stage_1 = object_model.get_articulation("outer_to_intermediate")
    stage_2 = object_model.get_articulation("intermediate_to_front")

    ctx.allow_overlap(
        intermediate,
        outer,
        elem_a="tube_body",
        elem_b="front_guide_pads",
        reason="The bronze guide pads intentionally use a sub-millimeter bearing interference to keep the sliding tube mechanically grounded.",
    )
    ctx.allow_overlap(
        front,
        intermediate,
        elem_a="shaft_body",
        elem_b="front_guide_pads",
        reason="The second-stage guide pads intentionally use a sub-millimeter bearing interference to keep the output tube mechanically grounded.",
    )

    ctx.check(
        "serial prismatic axes are inline",
        tuple(stage_1.axis) == (1.0, 0.0, 0.0) and tuple(stage_2.axis) == (1.0, 0.0, 0.0),
        details=f"stage_1={stage_1.axis}, stage_2={stage_2.axis}",
    )

    ctx.expect_within(
        intermediate,
        outer,
        axes="yz",
        inner_elem="tube_body",
        outer_elem="sleeve_body",
        margin=0.012,
        name="intermediate stage stays centered in outer sleeve bore",
    )
    ctx.expect_overlap(
        intermediate,
        outer,
        axes="x",
        elem_a="tube_body",
        elem_b="sleeve_body",
        min_overlap=0.520,
        name="collapsed intermediate stage remains guided",
    )
    ctx.expect_overlap(
        intermediate,
        outer,
        axes="x",
        elem_a="tube_body",
        elem_b="front_guide_pads",
        min_overlap=0.040,
        name="outer bearing pads engage intermediate tube",
    )
    ctx.expect_within(
        front,
        intermediate,
        axes="yz",
        inner_elem="shaft_body",
        outer_elem="tube_body",
        margin=0.012,
        name="front stage stays centered in intermediate bore",
    )
    ctx.expect_overlap(
        front,
        intermediate,
        axes="x",
        elem_a="shaft_body",
        elem_b="tube_body",
        min_overlap=0.440,
        name="collapsed front stage remains guided",
    )
    ctx.expect_overlap(
        front,
        intermediate,
        axes="x",
        elem_a="shaft_body",
        elem_b="front_guide_pads",
        min_overlap=0.040,
        name="intermediate bearing pads engage front tube",
    )

    rest_front_pos = ctx.part_world_position(front)
    with ctx.pose({stage_1: 0.420, stage_2: 0.350}):
        ctx.expect_overlap(
            intermediate,
            outer,
            axes="x",
            elem_a="tube_body",
            elem_b="sleeve_body",
            min_overlap=0.160,
            name="extended intermediate stage retains insertion",
        )
        ctx.expect_overlap(
            front,
            intermediate,
            axes="x",
            elem_a="shaft_body",
            elem_b="tube_body",
            min_overlap=0.160,
            name="extended front stage retains insertion",
        )
        ctx.expect_gap(
            intermediate,
            outer,
            axis="x",
            positive_elem="tube_body",
            negative_elem="sleeve_body",
            min_gap=-0.250,
            max_gap=0.900,
            name="intermediate stop stays within guided travel",
        )
        extended_front_pos = ctx.part_world_position(front)

    ctx.check(
        "front tube extends forward on shared axis",
        rest_front_pos is not None
        and extended_front_pos is not None
        and extended_front_pos[0] > rest_front_pos[0] + 0.700,
        details=f"rest={rest_front_pos}, extended={extended_front_pos}",
    )

    return ctx.report()


object_model = build_object_model()
