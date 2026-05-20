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
    mesh_from_cadquery,
)
import cadquery as cq


P0_TO_P1 = (0.220, 0.045)
P1_TO_P2 = (0.145, -0.050)

INPUT_Z = 0.056
COUPLER_Z = 0.081
OUTPUT_Z = 0.106

INPUT_THICKNESS = 0.012
COUPLER_THICKNESS = 0.010
OUTPUT_THICKNESS = 0.012

PIN_RADIUS = 0.0085
HOLE_RADIUS = 0.014
WASHER_OUTER_RADIUS = 0.024
WASHER_INNER_RADIUS = 0.0075
WASHER_THICKNESS = 0.004


def _normalize(v: tuple[float, float]) -> tuple[float, float]:
    length = math.hypot(v[0], v[1])
    if length <= 1.0e-9:
        return (0.0, 0.0)
    return (v[0] / length, v[1] / length)


def _smooth_polyline(points: list[tuple[float, float]], samples_per_span: int = 10) -> list[tuple[float, float]]:
    if len(points) == 2:
        return [points[0], points[1]]
    if len(points) == 3:
        p0, p1, p2 = points
        samples: list[tuple[float, float]] = []
        for i in range(samples_per_span + 1):
            t = i / samples_per_span
            u = 1.0 - t
            samples.append(
                (
                    u * u * p0[0] + 2.0 * u * t * p1[0] + t * t * p2[0],
                    u * u * p0[1] + 2.0 * u * t * p1[1] + t * t * p2[1],
                )
            )
        return samples
    return points


def _offset_strip_outline(
    centerline: list[tuple[float, float]],
    width: float,
) -> list[tuple[float, float]]:
    left: list[tuple[float, float]] = []
    right: list[tuple[float, float]] = []
    half_width = width * 0.5
    for i, p in enumerate(centerline):
        if i == 0:
            tangent = (centerline[1][0] - p[0], centerline[1][1] - p[1])
        elif i == len(centerline) - 1:
            tangent = (p[0] - centerline[i - 1][0], p[1] - centerline[i - 1][1])
        else:
            tangent = (
                centerline[i + 1][0] - centerline[i - 1][0],
                centerline[i + 1][1] - centerline[i - 1][1],
            )
        tangent = _normalize(tangent)
        normal = (-tangent[1], tangent[0])
        left.append((p[0] + normal[0] * half_width, p[1] + normal[1] * half_width))
        right.append((p[0] - normal[0] * half_width, p[1] - normal[1] * half_width))
    return left + list(reversed(right))


def _washer_shape(
    center: tuple[float, float],
    z_center: float,
    *,
    outer_radius: float = WASHER_OUTER_RADIUS,
    inner_radius: float = WASHER_INNER_RADIUS,
    thickness: float = WASHER_THICKNESS,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .center(center[0], center[1])
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(thickness)
        .translate((0.0, 0.0, z_center - thickness * 0.5))
    )


def _link_plate_shape(
    centerline_points: list[tuple[float, float]],
    *,
    width: float,
    thickness: float,
    z_center: float,
    boss_radius: float,
    holes: list[tuple[float, float]],
    edge_radius: float = 0.002,
) -> cq.Workplane:
    centerline = _smooth_polyline(centerline_points, samples_per_span=18)
    outline = _offset_strip_outline(centerline, width)
    solid = cq.Workplane("XY").polyline(outline).close().extrude(thickness)
    for point in holes:
        boss = cq.Workplane("XY").center(point[0], point[1]).circle(boss_radius).extrude(thickness)
        solid = solid.union(boss)

    cutters = cq.Workplane("XY").pushPoints(holes).circle(HOLE_RADIUS).extrude(thickness + 0.006)
    cutters = cutters.translate((0.0, 0.0, -0.003))
    solid = solid.cut(cutters)
    try:
        solid = solid.edges("|Z").fillet(edge_radius)
    except Exception:
        pass
    return solid.translate((0.0, 0.0, z_center - thickness * 0.5))


def _output_plate_shape() -> cq.Workplane:
    root = (0.0, 0.0)
    neck = (0.290, 0.035)
    tab_end = (0.380, 0.048)
    main = _link_plate_shape(
        [root, (0.120, 0.055), neck],
        width=0.034,
        thickness=OUTPUT_THICKNESS,
        z_center=OUTPUT_Z,
        boss_radius=0.036,
        holes=[root],
        edge_radius=0.002,
    )
    tab_centerline = _smooth_polyline([neck, tab_end], samples_per_span=1)
    tab_outline = _offset_strip_outline(tab_centerline, 0.018)
    tab = cq.Workplane("XY").polyline(tab_outline).close().extrude(OUTPUT_THICKNESS)
    tab = tab.union(cq.Workplane("XY").center(tab_end[0], tab_end[1]).circle(0.009).extrude(OUTPUT_THICKNESS))
    slot_center = (0.350, 0.044)
    slot = (
        cq.Workplane("XY")
        .center(slot_center[0], slot_center[1])
        .rect(0.036, 0.006)
        .extrude(OUTPUT_THICKNESS + 0.006)
    )
    slot = slot.union(
        cq.Workplane("XY")
        .pushPoints([(slot_center[0] - 0.018, slot_center[1]), (slot_center[0] + 0.018, slot_center[1])])
        .circle(0.003)
        .extrude(OUTPUT_THICKNESS + 0.006)
    )
    slot = slot.translate((0.0, 0.0, -0.003))
    tab = tab.cut(slot)
    try:
        tab = tab.edges("|Z").fillet(0.0015)
    except Exception:
        pass
    tab = tab.translate((0.0, 0.0, OUTPUT_Z - OUTPUT_THICKNESS * 0.5))
    return main.union(tab)


def _base_lug_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(0.210, 0.135, 0.016).translate((0.025, 0.0, 0.008))
    boss = cq.Workplane("XY").circle(0.043).extrude(0.030).translate((0.0, 0.0, 0.016))
    spine = cq.Workplane("XY").box(0.115, 0.030, 0.020).translate((0.055, 0.0, 0.026))
    rib_a = cq.Workplane("XY").box(0.090, 0.012, 0.018).translate((0.045, 0.036, 0.025))
    rib_b = cq.Workplane("XY").box(0.090, 0.012, 0.018).translate((0.045, -0.036, 0.025))
    shape = base.union(boss).union(spine).union(rib_a).union(rib_b)
    try:
        shape = shape.edges("|Z").fillet(0.004)
    except Exception:
        pass
    return shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_lever_linkage")

    dark_oxide = model.material("dark_oxide", rgba=(0.06, 0.065, 0.070, 1.0))
    blue_steel = model.material("blue_steel", rgba=(0.12, 0.18, 0.25, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.72, 0.72, 0.68, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))

    base = model.part("base_lug")
    base.visual(
        mesh_from_cadquery(_base_lug_shape(), "base_lug"),
        material=dark_oxide,
        name="base_lug",
    )
    base.visual(
        Cylinder(radius=PIN_RADIUS, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.059)),
        material=brushed_steel,
        name="root_pin",
    )
    base.visual(
        mesh_from_cadquery(_washer_shape((0.0, 0.0), INPUT_Z - INPUT_THICKNESS * 0.5 - WASHER_THICKNESS * 0.5), "root_lower_washer"),
        material=brushed_steel,
        name="root_lower_washer",
    )
    base.visual(
        mesh_from_cadquery(_washer_shape((0.0, 0.0), INPUT_Z + INPUT_THICKNESS * 0.5 + WASHER_THICKNESS * 0.5), "root_top_washer"),
        material=brushed_steel,
        name="root_top_washer",
    )
    for index, (x, y) in enumerate(((-0.045, -0.045), (-0.045, 0.045), (0.095, -0.045), (0.095, 0.045))):
        base.visual(
            Cylinder(radius=0.007, length=0.006),
            origin=Origin(xyz=(x, y, 0.018)),
            material=brushed_steel,
            name=f"anchor_bolt_{index}",
        )

    input_lever = model.part("input_lever")
    input_lever.visual(
        mesh_from_cadquery(
            _link_plate_shape(
                [(0.0, 0.0), (0.080, 0.105), P0_TO_P1],
                width=0.034,
                thickness=INPUT_THICKNESS,
                z_center=INPUT_Z,
                boss_radius=0.037,
                holes=[(0.0, 0.0), P0_TO_P1],
            ),
            "input_plate",
        ),
        material=blue_steel,
        name="input_plate",
    )
    input_lever.visual(
        Cylinder(radius=PIN_RADIUS, length=0.032),
        origin=Origin(xyz=(P0_TO_P1[0], P0_TO_P1[1], (INPUT_Z + INPUT_THICKNESS * 0.5 + COUPLER_Z + COUPLER_THICKNESS * 0.5) * 0.5)),
        material=brushed_steel,
        name="coupler_pin",
    )
    input_lever.visual(
        Cylinder(radius=0.016, length=0.006),
        origin=Origin(xyz=(P0_TO_P1[0], P0_TO_P1[1], INPUT_Z + INPUT_THICKNESS * 0.5 + 0.002)),
        material=brushed_steel,
        name="coupler_pin_shoulder",
    )
    input_lever.visual(
        mesh_from_cadquery(
            _washer_shape(P0_TO_P1, COUPLER_Z - COUPLER_THICKNESS * 0.5 - WASHER_THICKNESS * 0.5),
            "coupler_lower_washer",
        ),
        material=brushed_steel,
        name="coupler_lower_washer",
    )
    input_lever.visual(
        mesh_from_cadquery(
            _washer_shape(P0_TO_P1, COUPLER_Z + COUPLER_THICKNESS * 0.5 + WASHER_THICKNESS * 0.5),
            "coupler_top_washer",
        ),
        material=brushed_steel,
        name="coupler_top_washer",
    )
    input_lever.visual(
        mesh_from_cadquery(_washer_shape((0.0, 0.0), INPUT_Z, outer_radius=0.032, inner_radius=HOLE_RADIUS, thickness=0.003), "input_root_formed_edge"),
        material=brushed_steel,
        name="root_formed_edge",
    )

    coupler = model.part("coupler_lever")
    coupler.visual(
        mesh_from_cadquery(
            _link_plate_shape(
                [(0.0, 0.0), P1_TO_P2],
                width=0.030,
                thickness=COUPLER_THICKNESS,
                z_center=COUPLER_Z,
                boss_radius=0.032,
                holes=[(0.0, 0.0), P1_TO_P2],
                edge_radius=0.0018,
            ),
            "coupler_plate",
        ),
        material=blue_steel,
        name="coupler_plate",
    )
    coupler.visual(
        Cylinder(radius=PIN_RADIUS, length=0.036),
        origin=Origin(xyz=(P1_TO_P2[0], P1_TO_P2[1], (COUPLER_Z + COUPLER_THICKNESS * 0.5 + OUTPUT_Z + OUTPUT_THICKNESS * 0.5) * 0.5)),
        material=brushed_steel,
        name="output_pin",
    )
    coupler.visual(
        Cylinder(radius=0.016, length=0.006),
        origin=Origin(xyz=(P1_TO_P2[0], P1_TO_P2[1], COUPLER_Z + COUPLER_THICKNESS * 0.5 + 0.002)),
        material=brushed_steel,
        name="output_pin_shoulder",
    )
    coupler.visual(
        mesh_from_cadquery(
            _washer_shape(P1_TO_P2, OUTPUT_Z - OUTPUT_THICKNESS * 0.5 - WASHER_THICKNESS * 0.5),
            "output_lower_washer",
        ),
        material=brushed_steel,
        name="output_lower_washer",
    )
    coupler.visual(
        mesh_from_cadquery(
            _washer_shape(P1_TO_P2, OUTPUT_Z + OUTPUT_THICKNESS * 0.5 + WASHER_THICKNESS * 0.5),
            "output_top_washer",
        ),
        material=brushed_steel,
        name="output_top_washer",
    )

    output_lever = model.part("output_lever")
    output_lever.visual(
        mesh_from_cadquery(_output_plate_shape(), "output_plate"),
        material=blue_steel,
        name="output_plate",
    )
    output_lever.visual(
        Box((0.028, 0.006, 0.004)),
        origin=Origin(xyz=(0.352, 0.044, OUTPUT_Z + OUTPUT_THICKNESS * 0.5 + 0.002)),
        material=black_rubber,
        name="tab_grip_pad",
    )

    model.articulation(
        "base_to_input",
        ArticulationType.REVOLUTE,
        parent=base,
        child=input_lever,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.0, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "input_to_coupler",
        ArticulationType.REVOLUTE,
        parent=input_lever,
        child=coupler,
        origin=Origin(xyz=(P0_TO_P1[0], P0_TO_P1[1], 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=15.0, velocity=2.5, lower=-0.75, upper=0.75),
    )
    model.articulation(
        "coupler_to_output",
        ArticulationType.REVOLUTE,
        parent=coupler,
        child=output_lever,
        origin=Origin(xyz=(P1_TO_P2[0], P1_TO_P2[1], 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=15.0, velocity=2.5, lower=-0.80, upper=0.80),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_lug")
    input_lever = object_model.get_part("input_lever")
    coupler = object_model.get_part("coupler_lever")
    output = object_model.get_part("output_lever")
    base_joint = object_model.get_articulation("base_to_input")
    coupler_joint = object_model.get_articulation("input_to_coupler")
    output_joint = object_model.get_articulation("coupler_to_output")

    ctx.check(
        "three parallel revolute axes",
        all(
            joint.articulation_type == ArticulationType.REVOLUTE and tuple(joint.axis) == (0.0, 0.0, 1.0)
            for joint in (base_joint, coupler_joint, output_joint)
        ),
        details="Expected the base, coupler, and output pivots to be revolute joints about +Z.",
    )

    ctx.expect_gap(
        input_lever,
        base,
        axis="z",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem="input_plate",
        negative_elem="root_lower_washer",
        name="input plate captured by root lower washer",
    )
    ctx.expect_gap(
        coupler,
        input_lever,
        axis="z",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem="coupler_plate",
        negative_elem="coupler_lower_washer",
        name="coupler plate captured by lower washer",
    )
    ctx.expect_gap(
        output,
        coupler,
        axis="z",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem="output_plate",
        negative_elem="output_lower_washer",
        name="output plate captured by lower washer",
    )
    ctx.expect_gap(
        coupler,
        input_lever,
        axis="z",
        min_gap=0.008,
        positive_elem="coupler_plate",
        negative_elem="input_plate",
        name="coupler layer clears input layer",
    )
    ctx.expect_gap(
        output,
        coupler,
        axis="z",
        min_gap=0.008,
        positive_elem="output_plate",
        negative_elem="coupler_plate",
        name="output layer clears coupler layer",
    )
    ctx.expect_overlap(
        input_lever,
        base,
        axes="xy",
        min_overlap=0.020,
        elem_a="input_plate",
        elem_b="root_top_washer",
        name="root washer overlaps input boss footprint",
    )
    ctx.expect_overlap(
        output,
        coupler,
        axes="xy",
        min_overlap=0.020,
        elem_a="output_plate",
        elem_b="output_top_washer",
        name="output washer overlaps root boss footprint",
    )

    rest_output = ctx.part_world_position(output)
    with ctx.pose({base_joint: 0.50, coupler_joint: -0.60, output_joint: 0.65}):
        ctx.expect_gap(
            coupler,
            input_lever,
            axis="z",
            min_gap=0.008,
            positive_elem="coupler_plate",
            negative_elem="input_plate",
            name="swept coupler clears input layer",
        )
        ctx.expect_gap(
            output,
            coupler,
            axis="z",
            min_gap=0.008,
            positive_elem="output_plate",
            negative_elem="coupler_plate",
            name="swept output clears coupler layer",
        )
        swept_output = ctx.part_world_position(output)

    ctx.check(
        "output pivot moves through linkage arc",
        rest_output is not None
        and swept_output is not None
        and math.hypot(swept_output[0] - rest_output[0], swept_output[1] - rest_output[1]) > 0.050,
        details=f"rest={rest_output}, swept={swept_output}",
    )

    return ctx.report()


object_model = build_object_model()
