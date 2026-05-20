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
    TorusGeometry,
    TrunnionYokeGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="small_trunnion_table")

    cast_iron = model.material("cast_iron", rgba=(0.18, 0.20, 0.22, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.55, 0.57, 0.56, 1.0))
    dark_slot = model.material("dark_slot", rgba=(0.04, 0.045, 0.05, 1.0))
    table_blue = model.material("table_blue", rgba=(0.10, 0.22, 0.34, 1.0))
    brass_mark = model.material("brass_mark", rgba=(0.95, 0.68, 0.24, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.31, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=cast_iron,
        name="floor_plate",
    )
    pedestal.visual(
        Cylinder(radius=0.235, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=cast_iron,
        name="fixed_bearing",
    )
    pedestal.visual(
        Box((0.025, 0.045, 0.018)),
        origin=Origin(xyz=(0.0, 0.285, 0.044)),
        material=dark_slot,
        name="fixed_index",
    )

    rotary_base = model.part("rotary_base")
    rotary_base.visual(
        Cylinder(radius=0.25, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=satin_steel,
        name="turntable_disk",
    )
    rotary_base.visual(
        Box((0.018, 0.045, 0.008)),
        origin=Origin(xyz=(0.0, 0.240, 0.054)),
        material=brass_mark,
        name="front_index",
    )

    yoke = TrunnionYokeGeometry(
        (0.44, 0.22, 0.40),
        span_width=0.32,
        trunnion_diameter=0.074,
        trunnion_center_z=0.315,
        base_thickness=0.040,
        corner_radius=0.012,
        center=False,
    )
    rotary_base.visual(
        mesh_from_geometry(yoke, "trunnion_yoke"),
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        material=cast_iron,
        name="trunnion_yoke",
    )

    bearing_ring = mesh_from_geometry(
        TorusGeometry(radius=0.047, tube=0.006, radial_segments=16, tubular_segments=32),
        "trunnion_bearing_ring",
    )
    for suffix, x in enumerate((-0.222, 0.222)):
        rotary_base.visual(
            bearing_ring,
            origin=Origin(xyz=(x, 0.0, 0.363), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_steel,
            name=f"bearing_ring_{suffix}",
        )

    table = model.part("table")
    plate = cq.Workplane("XY").box(0.270, 0.420, 0.055)
    top_z = 0.0275
    for x in (-0.080, 0.0, 0.080):
        wide_slot = cq.Workplane("XY").box(0.026, 0.365, 0.014).translate(
            (x, 0.0, top_z - 0.007)
        )
        deep_stem = cq.Workplane("XY").box(0.012, 0.385, 0.034).translate(
            (x, 0.0, top_z - 0.017)
        )
        plate = plate.cut(wide_slot).cut(deep_stem)
    table.visual(
        mesh_from_cadquery(plate, "slotted_table"),
        origin=Origin(),
        material=table_blue,
        name="table_slab",
    )
    table.visual(
        Cylinder(radius=0.037, length=0.426),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="trunnion_shaft",
    )
    for suffix, x in enumerate((-0.148, 0.148)):
        table.visual(
            Cylinder(radius=0.055, length=0.018),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_steel,
            name=f"table_hub_{suffix}",
        )

    model.articulation(
        "base_turn",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=rotary_base,
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=70.0, velocity=1.0, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "table_tilt",
        ArticulationType.REVOLUTE,
        parent=rotary_base,
        child=table,
        origin=Origin(xyz=(0.0, 0.0, 0.363)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.8, lower=-1.05, upper=1.05),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    rotary_base = object_model.get_part("rotary_base")
    table = object_model.get_part("table")
    base_turn = object_model.get_articulation("base_turn")
    table_tilt = object_model.get_articulation("table_tilt")

    ctx.allow_overlap(
        rotary_base,
        table,
        elem_a="trunnion_yoke",
        elem_b="trunnion_shaft",
        reason=(
            "The table's trunnion shaft is intentionally captured in the "
            "cheek bores so the cradle reads as bearing-supported."
        ),
    )

    ctx.expect_gap(
        rotary_base,
        pedestal,
        axis="z",
        positive_elem="turntable_disk",
        negative_elem="fixed_bearing",
        max_gap=0.001,
        max_penetration=0.0,
        name="rotary table disk sits on the fixed bearing",
    )
    ctx.expect_overlap(
        table,
        rotary_base,
        axes="x",
        elem_a="trunnion_shaft",
        elem_b="trunnion_yoke",
        min_overlap=0.40,
        name="trunnion shaft is retained through both cheeks",
    )
    ctx.expect_within(
        table,
        rotary_base,
        axes="yz",
        inner_elem="trunnion_shaft",
        outer_elem="trunnion_yoke",
        margin=0.0,
        name="trunnion shaft stays centered in the yoke bores",
    )
    ctx.expect_within(
        table,
        rotary_base,
        axes="x",
        inner_elem="table_slab",
        outer_elem="trunnion_yoke",
        margin=0.0,
        name="table slab is suspended between the side cheeks",
    )

    def aabb_center(aabb):
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    rest_table_aabb = ctx.part_element_world_aabb(table, elem="table_slab")
    rest_height = (
        rest_table_aabb[1][2] - rest_table_aabb[0][2]
        if rest_table_aabb is not None
        else None
    )
    with ctx.pose({table_tilt: 0.75}):
        tilted_table_aabb = ctx.part_element_world_aabb(table, elem="table_slab")
    tilted_height = (
        tilted_table_aabb[1][2] - tilted_table_aabb[0][2]
        if tilted_table_aabb is not None
        else None
    )
    ctx.check(
        "table tilts on the horizontal trunnion axis",
        rest_height is not None
        and tilted_height is not None
        and tilted_height > rest_height + 0.22,
        details=f"rest_height={rest_height}, tilted_height={tilted_height}",
    )

    rest_marker = ctx.part_element_world_aabb(rotary_base, elem="front_index")
    rest_marker_center = aabb_center(rest_marker) if rest_marker is not None else None
    with ctx.pose({base_turn: math.pi / 2.0}):
        turned_marker = ctx.part_element_world_aabb(rotary_base, elem="front_index")
    turned_marker_center = aabb_center(turned_marker) if turned_marker is not None else None
    ctx.check(
        "rotary base turns about the vertical axis",
        rest_marker_center is not None
        and turned_marker_center is not None
        and rest_marker_center[1] > 0.20
        and turned_marker_center[0] < -0.20
        and abs(turned_marker_center[1]) < 0.08,
        details=f"rest={rest_marker_center}, turned={turned_marker_center}",
    )

    return ctx.report()


object_model = build_object_model()
