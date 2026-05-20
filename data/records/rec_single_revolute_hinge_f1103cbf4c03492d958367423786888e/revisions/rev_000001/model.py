from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


STACK_LENGTH = 0.110
KNUCKLE_CLEARANCE = 0.002
KNUCKLE_COUNT = 5
OUTER_RADIUS = 0.009
INNER_RADIUS = 0.0038
PIN_RADIUS = INNER_RADIUS
LEAF_WIDTH = 0.055
LEAF_THICKNESS = 0.003
LEAF_Y = -0.0105


def _intervals_for_knuckles(indices: tuple[int, ...]) -> list[tuple[float, float]]:
    segment = (STACK_LENGTH - KNUCKLE_CLEARANCE * (KNUCKLE_COUNT - 1)) / KNUCKLE_COUNT
    z0 = -STACK_LENGTH / 2.0
    intervals: list[tuple[float, float]] = []
    for i in indices:
        start = z0 + i * (segment + KNUCKLE_CLEARANCE)
        intervals.append((start, start + segment))
    return intervals


def _tube_segment(z0: float, z1: float) -> cq.Workplane:
    length = z1 - z0
    outer = (
        cq.Workplane("XY")
        .circle(OUTER_RADIUS)
        .extrude(length)
        .translate((0.0, 0.0, z0))
    )
    bore = (
        cq.Workplane("XY")
        .circle(INNER_RADIUS)
        .extrude(length + 0.004)
        .translate((0.0, 0.0, z0 - 0.002))
    )
    return outer.cut(bore)


def _round_hole(x: float, z: float, radius: float) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center(x, z)
        .circle(radius)
        .extrude(LEAF_THICKNESS * 8.0, both=True)
    )


def _leaf_plate(side: int) -> cq.Workplane:
    x_center = side * (OUTER_RADIUS + LEAF_WIDTH / 2.0 + 0.002)
    plate = (
        cq.Workplane("XY")
        .box(LEAF_WIDTH, LEAF_THICKNESS, STACK_LENGTH)
        .translate((x_center, LEAF_Y, 0.0))
        .edges("|Y")
        .fillet(0.0035)
    )
    for z in (-0.033, 0.033):
        plate = plate.cut(_round_hole(side * (OUTER_RADIUS + 0.030), z, 0.0050))
    return plate


def _hinge_half(side: int, indices: tuple[int, ...]) -> cq.Workplane:
    """Build one connected leaf with its assigned alternating hollow knuckles."""
    half = _leaf_plate(side)
    for z0, z1 in _intervals_for_knuckles(indices):
        zc = (z0 + z1) / 2.0
        tube = _tube_segment(z0, z1)
        web = (
            cq.Workplane("XY")
            .box(0.010, 0.007, z1 - z0)
            .translate((side * 0.011, -0.0075, zc))
        )
        half = half.union(tube).union(web)
    return half


def _pin() -> cq.Workplane:
    shaft_length = STACK_LENGTH + 0.010
    head_radius = 0.0056
    head_thickness = 0.0030
    shaft = (
        cq.Workplane("XY")
        .circle(PIN_RADIUS)
        .extrude(shaft_length)
        .translate((0.0, 0.0, -shaft_length / 2.0))
    )
    top_head = (
        cq.Workplane("XY")
        .circle(head_radius)
        .extrude(head_thickness)
        .translate((0.0, 0.0, shaft_length / 2.0 - 0.0004))
    )
    bottom_head = (
        cq.Workplane("XY")
        .circle(head_radius)
        .extrude(head_thickness)
        .translate((0.0, 0.0, -shaft_length / 2.0 - head_thickness + 0.0004))
    )
    return shaft.union(top_head).union(bottom_head)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="knuckle_stack_hinge")

    satin_steel = model.material("satin_steel", rgba=(0.60, 0.62, 0.60, 1.0))
    darker_leaf = model.material("brushed_ground_leaf", rgba=(0.42, 0.44, 0.43, 1.0))
    polished_pin = model.material("polished_pin", rgba=(0.78, 0.78, 0.74, 1.0))

    grounded = model.part("grounded_half")
    grounded.visual(
        mesh_from_cadquery(_hinge_half(-1, (0, 2, 4)), "grounded_half"),
        material=darker_leaf,
        name="grounded_leaf_knuckles",
    )

    carried = model.part("carried_half")
    carried.visual(
        mesh_from_cadquery(_hinge_half(1, (1, 3)), "carried_half"),
        material=satin_steel,
        name="carried_leaf_knuckles",
    )

    pin = model.part("pin")
    pin.visual(
        mesh_from_cadquery(_pin(), "central_pin"),
        material=polished_pin,
        name="pin_shaft_heads",
    )

    model.articulation(
        "grounded_to_carried",
        ArticulationType.REVOLUTE,
        parent=grounded,
        child=carried,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=0.0, upper=1.75),
    )
    model.articulation(
        "grounded_to_pin",
        ArticulationType.FIXED,
        parent=grounded,
        child=pin,
        origin=Origin(),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    grounded = object_model.get_part("grounded_half")
    carried = object_model.get_part("carried_half")
    pin = object_model.get_part("pin")
    hinge = object_model.get_articulation("grounded_to_carried")

    ctx.allow_overlap(
        grounded,
        pin,
        elem_a="grounded_leaf_knuckles",
        elem_b="pin_shaft_heads",
        reason=(
            "The central pin is intentionally captured through the grounded hollow "
            "knuckle barrels; the mesh proxy reports the seated pin/bore interface as overlap."
        ),
    )
    ctx.allow_overlap(
        carried,
        pin,
        elem_a="carried_leaf_knuckles",
        elem_b="pin_shaft_heads",
        reason=(
            "The carried knuckles rotate around the same captured pin, so the pin is "
            "intentionally nested through their bore."
        ),
    )

    ctx.check(
        "carried half has one pin-axis revolute joint",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 6) for v in hinge.axis) == (0.0, 0.0, 1.0),
        details=f"type={hinge.articulation_type}, axis={hinge.axis}",
    )
    ctx.check(
        "revolute joint has a practical hinge swing",
        hinge.motion_limits is not None
        and hinge.motion_limits.lower == 0.0
        and hinge.motion_limits.upper >= 1.5,
        details=f"limits={hinge.motion_limits}",
    )
    ctx.expect_overlap(
        grounded,
        carried,
        axes="z",
        min_overlap=0.100,
        name="both hinge halves share the same compact knuckle stack height",
    )
    ctx.expect_overlap(
        pin,
        grounded,
        axes="z",
        min_overlap=0.105,
        name="central pin spans the grounded knuckles",
    )
    ctx.expect_overlap(
        pin,
        carried,
        axes="z",
        min_overlap=0.075,
        name="central pin spans the carried knuckles",
    )
    ctx.expect_within(
        pin,
        grounded,
        axes="xy",
        inner_elem="pin_shaft_heads",
        outer_elem="grounded_leaf_knuckles",
        margin=0.0,
        name="pin is centered within the grounded barrel footprint",
    )
    ctx.expect_within(
        pin,
        carried,
        axes="xy",
        inner_elem="pin_shaft_heads",
        outer_elem="carried_leaf_knuckles",
        margin=0.0,
        name="pin is centered within the carried barrel footprint",
    )

    rest_aabb = ctx.part_world_aabb(carried)
    with ctx.pose({hinge: 1.2}):
        swung_aabb = ctx.part_world_aabb(carried)

    def mid_y(aabb):
        if aabb is None:
            return None
        return (aabb[0][1] + aabb[1][1]) / 2.0

    rest_y = mid_y(rest_aabb)
    swung_y = mid_y(swung_aabb)
    ctx.check(
        "carried half swings around the fixed pin",
        rest_y is not None and swung_y is not None and swung_y > rest_y + 0.025,
        details=f"rest_y={rest_y}, swung_y={swung_y}",
    )

    return ctx.report()


object_model = build_object_model()
