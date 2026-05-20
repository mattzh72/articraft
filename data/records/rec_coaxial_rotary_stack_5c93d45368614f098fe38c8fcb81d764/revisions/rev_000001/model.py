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


def _safe_chamfer(shape, amount: float):
    try:
        return shape.edges().chamfer(amount)
    except Exception:
        return shape


def _safe_fillet_vertical(shape, amount: float):
    try:
        return shape.edges("|Z").fillet(amount)
    except Exception:
        return shape


def _cut_hole_circle(shape, *, radius: float, hole_radius: float, count: int, height: float):
    cutter_height = height + 0.030
    for index in range(count):
        angle = 2.0 * math.pi * index / count
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        cutter = (
            cq.Workplane("XY")
            .center(x, y)
            .circle(hole_radius)
            .extrude(cutter_height)
            .translate((0.0, 0.0, -0.015))
        )
        shape = shape.cut(cutter)
    return shape


def _cut_counterbores(
    shape,
    *,
    radius: float,
    bore_radius: float,
    count: int,
    z_top: float,
    depth: float,
):
    for index in range(count):
        angle = 2.0 * math.pi * index / count
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        cutter = (
            cq.Workplane("XY")
            .center(x, y)
            .circle(bore_radius)
            .extrude(depth + 0.004)
            .translate((0.0, 0.0, z_top - depth))
        )
        shape = shape.cut(cutter)
    return shape


def _annulus(outer_radius: float, inner_radius: float, height: float, *, chamfer: float = 0.0015):
    shape = cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(height)
    return _safe_chamfer(shape, chamfer)


def _base_body():
    height = 0.060
    shape = cq.Workplane("XY").circle(0.295).extrude(0.030)
    shape = shape.union(cq.Workplane("XY").circle(0.250).extrude(height))
    shape = shape.union(cq.Workplane("XY").circle(0.082).extrude(0.073))
    shape = _cut_hole_circle(shape, radius=0.238, hole_radius=0.011, count=12, height=0.090)
    shape = _cut_counterbores(
        shape, radius=0.238, bore_radius=0.018, count=12, z_top=0.060, depth=0.010
    )
    shape = _safe_fillet_vertical(shape, 0.003)
    return _safe_chamfer(shape, 0.0015)


def _outer_carrier_body():
    height = 0.037
    shape = _annulus(0.255, 0.118, height, chamfer=0.0013)
    # One protruding service lug makes the yaw motion visually obvious and reads
    # as a bolted drive tab on the outer slew ring.
    lug = (
        cq.Workplane("XY")
        .box(0.090, 0.070, 0.020)
        .translate((0.278, 0.0, 0.021))
    )
    shape = shape.union(lug)
    shape = _cut_hole_circle(shape, radius=0.202, hole_radius=0.008, count=16, height=0.060)
    shape = _cut_counterbores(
        shape, radius=0.202, bore_radius=0.013, count=16, z_top=height, depth=0.006
    )
    shape = _safe_fillet_vertical(shape, 0.002)
    return _safe_chamfer(shape, 0.0012)


def _inner_ring_body():
    height = 0.030
    shape = _annulus(0.163, 0.066, height, chamfer=0.0012)
    lug = (
        cq.Workplane("XY")
        .box(0.058, 0.046, 0.016)
        .translate((0.174, 0.0, 0.020))
    )
    shape = shape.union(lug)
    shape = _cut_hole_circle(shape, radius=0.124, hole_radius=0.006, count=10, height=0.050)
    shape = _cut_counterbores(
        shape, radius=0.124, bore_radius=0.010, count=10, z_top=height, depth=0.005
    )
    shape = _safe_fillet_vertical(shape, 0.0016)
    return _safe_chamfer(shape, 0.001)


def _top_platter_body():
    shape = cq.Workplane("XY").circle(0.135).extrude(0.025)
    shape = shape.union(cq.Workplane("XY").circle(0.060).extrude(0.044))
    # Two low fixture rails are machined into the platter as connected raised
    # pads so the upper yaw joint is visibly non-axisymmetric.
    rail_0 = cq.Workplane("XY").box(0.150, 0.018, 0.009).translate((0.010, 0.040, 0.0295))
    rail_1 = cq.Workplane("XY").box(0.150, 0.018, 0.009).translate((0.010, -0.040, 0.0295))
    shape = shape.union(rail_0).union(rail_1)
    shape = _cut_hole_circle(shape, radius=0.095, hole_radius=0.006, count=8, height=0.060)
    shape = _cut_counterbores(
        shape, radius=0.095, bore_radius=0.010, count=8, z_top=0.025, depth=0.005
    )
    shape = _safe_fillet_vertical(shape, 0.0018)
    return _safe_chamfer(shape, 0.001)


def _aabb_center(aabb):
    if aabb is None:
        return None
    lo, hi = aabb

    def coord(vec, index: int):
        try:
            return vec[index]
        except TypeError:
            return (vec.x, vec.y, vec.z)[index]

    return tuple((coord(lo, i) + coord(hi, i)) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nested_rotary_turntable")

    model.material("cast_iron", rgba=(0.12, 0.13, 0.14, 1.0))
    model.material("dark_anodized", rgba=(0.08, 0.09, 0.10, 1.0))
    model.material("machined_aluminum", rgba=(0.70, 0.73, 0.76, 1.0))
    model.material("bearing_steel", rgba=(0.58, 0.61, 0.65, 1.0))
    model.material("safety_orange", rgba=(0.88, 0.38, 0.07, 1.0))
    model.material("index_yellow", rgba=(0.95, 0.78, 0.12, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_body(), "base_body", tolerance=0.0012),
        material="cast_iron",
        name="base_body",
    )
    base.visual(
        mesh_from_cadquery(_annulus(0.245, 0.160, 0.006), "base_bearing_race"),
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        material="bearing_steel",
        name="base_bearing_race",
    )
    for x, y, name in (
        (0.180, 0.180, "foot_0"),
        (-0.180, 0.180, "foot_1"),
        (-0.180, -0.180, "foot_2"),
        (0.180, -0.180, "foot_3"),
    ):
        base.visual(
            Cylinder(radius=0.030, length=0.014),
            origin=Origin(xyz=(x, y, -0.005)),
            material="dark_anodized",
            name=name,
        )

    outer = model.part("outer_carrier")
    outer.visual(
        mesh_from_cadquery(_outer_carrier_body(), "outer_carrier_body", tolerance=0.0012),
        material="machined_aluminum",
        name="outer_carrier_body",
    )
    outer.visual(
        mesh_from_cadquery(_annulus(0.238, 0.174, 0.006), "outer_lower_race"),
        material="bearing_steel",
        name="outer_lower_race",
    )
    outer.visual(
        mesh_from_cadquery(_annulus(0.154, 0.092, 0.006), "outer_upper_race"),
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
        material="bearing_steel",
        name="outer_upper_race",
    )
    outer.visual(
        Box((0.070, 0.014, 0.006)),
        origin=Origin(xyz=(0.252, 0.0, 0.0375)),
        material="index_yellow",
        name="outer_index_tab",
    )

    middle = model.part("middle_ring")
    middle.visual(
        mesh_from_cadquery(_inner_ring_body(), "middle_ring_body", tolerance=0.0012),
        material="dark_anodized",
        name="middle_ring_body",
    )
    middle.visual(
        mesh_from_cadquery(_annulus(0.150, 0.095, 0.005), "middle_lower_race"),
        material="bearing_steel",
        name="middle_lower_race",
    )
    middle.visual(
        mesh_from_cadquery(_annulus(0.112, 0.071, 0.005), "middle_upper_race"),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material="bearing_steel",
        name="middle_upper_race",
    )
    middle.visual(
        Box((0.054, 0.012, 0.006)),
        origin=Origin(xyz=(0.161, 0.0, 0.031)),
        material="index_yellow",
        name="middle_index_tab",
    )

    platter = model.part("top_platter")
    platter.visual(
        mesh_from_cadquery(_top_platter_body(), "top_platter_body", tolerance=0.0012),
        material="safety_orange",
        name="top_platter_body",
    )
    platter.visual(
        mesh_from_cadquery(_annulus(0.105, 0.070, 0.006), "platter_lower_race"),
        material="bearing_steel",
        name="platter_lower_race",
    )
    platter.visual(
        Box((0.115, 0.018, 0.006)),
        origin=Origin(xyz=(0.075, 0.0, 0.028)),
        material="dark_anodized",
        name="fixture_rail",
    )

    model.articulation(
        "base_to_outer",
        ArticulationType.REVOLUTE,
        parent=base,
        child=outer,
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-math.pi, upper=math.pi, effort=120.0, velocity=1.0),
    )
    model.articulation(
        "outer_to_middle",
        ArticulationType.REVOLUTE,
        parent=outer,
        child=middle,
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-2.35, upper=2.35, effort=70.0, velocity=1.6),
    )
    model.articulation(
        "middle_to_platter",
        ArticulationType.REVOLUTE,
        parent=middle,
        child=platter,
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-math.pi, upper=math.pi, effort=45.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    outer = object_model.get_part("outer_carrier")
    middle = object_model.get_part("middle_ring")
    platter = object_model.get_part("top_platter")
    base_to_outer = object_model.get_articulation("base_to_outer")
    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_platter = object_model.get_articulation("middle_to_platter")

    for joint in (base_to_outer, outer_to_middle, middle_to_platter):
        axis = tuple(round(v, 6) for v in joint.axis)
        xyz = joint.origin.xyz if joint.origin is not None else (0.0, 0.0, 0.0)
        ctx.check(
            f"{joint.name} is coaxial",
            axis == (0.0, 0.0, 1.0) and abs(xyz[0]) < 1e-9 and abs(xyz[1]) < 1e-9,
            details=f"axis={axis}, origin={xyz}",
        )

    ctx.expect_gap(
        outer,
        base,
        axis="z",
        positive_elem="outer_lower_race",
        negative_elem="base_bearing_race",
        max_gap=0.0005,
        max_penetration=0.000001,
        name="outer bearing races are seated",
    )
    ctx.expect_overlap(
        outer,
        base,
        axes="xy",
        elem_a="outer_lower_race",
        elem_b="base_bearing_race",
        min_overlap=0.055,
        name="outer race has compact radial engagement",
    )
    ctx.expect_gap(
        middle,
        outer,
        axis="z",
        positive_elem="middle_lower_race",
        negative_elem="outer_upper_race",
        max_gap=0.0005,
        max_penetration=0.000001,
        name="middle bearing races are seated",
    )
    ctx.expect_overlap(
        middle,
        outer,
        axes="xy",
        elem_a="middle_lower_race",
        elem_b="outer_upper_race",
        min_overlap=0.050,
        name="middle race has compact radial engagement",
    )
    ctx.expect_gap(
        platter,
        middle,
        axis="z",
        positive_elem="platter_lower_race",
        negative_elem="middle_upper_race",
        max_gap=0.0005,
        max_penetration=0.000001,
        name="platter bearing races are seated",
    )
    ctx.expect_overlap(
        platter,
        middle,
        axes="xy",
        elem_a="platter_lower_race",
        elem_b="middle_upper_race",
        min_overlap=0.035,
        name="platter race has compact radial engagement",
    )

    rest_outer = _aabb_center(ctx.part_element_world_aabb(outer, elem="outer_index_tab"))
    with ctx.pose({base_to_outer: 0.75}):
        turned_outer = _aabb_center(ctx.part_element_world_aabb(outer, elem="outer_index_tab"))
    ctx.check(
        "outer carrier visibly yaws",
        rest_outer is not None and turned_outer is not None and turned_outer[1] > rest_outer[1] + 0.12,
        details=f"rest={rest_outer}, turned={turned_outer}",
    )

    rest_middle = _aabb_center(ctx.part_element_world_aabb(middle, elem="middle_index_tab"))
    with ctx.pose({outer_to_middle: 0.90}):
        turned_middle = _aabb_center(ctx.part_element_world_aabb(middle, elem="middle_index_tab"))
    ctx.check(
        "middle ring visibly yaws",
        rest_middle is not None
        and turned_middle is not None
        and turned_middle[1] > rest_middle[1] + 0.11,
        details=f"rest={rest_middle}, turned={turned_middle}",
    )

    rest_platter = _aabb_center(ctx.part_element_world_aabb(platter, elem="fixture_rail"))
    with ctx.pose({middle_to_platter: 1.00}):
        turned_platter = _aabb_center(ctx.part_element_world_aabb(platter, elem="fixture_rail"))
    ctx.check(
        "top platter visibly yaws",
        rest_platter is not None
        and turned_platter is not None
        and turned_platter[1] > rest_platter[1] + 0.050,
        details=f"rest={rest_platter}, turned={turned_platter}",
    )

    return ctx.report()


object_model = build_object_model()
