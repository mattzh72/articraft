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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _x_cylinder_origin(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(0.0, pi / 2.0, 0.0))


def _y_cylinder_origin(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(-pi / 2.0, 0.0, 0.0))


def _add_rotary_handles(
    part,
    *,
    hub_length: float,
    hub_radius: float,
    rod_length: float,
    rod_radius: float,
    knob_radius: float,
    angles: list[float],
    stem_x: float,
    hub_name: str,
    material,
) -> None:
    part.visual(
        Cylinder(radius=hub_radius, length=hub_length),
        origin=_x_cylinder_origin((hub_length * 0.5, 0.0, 0.0)),
        material=material,
        name=hub_name,
    )
    for index, angle in enumerate(angles):
        y = 0.5 * rod_length * cos(angle)
        z = 0.5 * rod_length * sin(angle)
        part.visual(
            Cylinder(radius=rod_radius, length=rod_length),
            origin=Origin(
                xyz=(stem_x, y, z),
                rpy=(angle - pi / 2.0, 0.0, 0.0),
            ),
            material=material,
            name=f"spoke_{index}",
        )
        part.visual(
            Sphere(radius=knob_radius),
            origin=Origin(
                xyz=(
                    stem_x,
                    rod_length * cos(angle),
                    rod_length * sin(angle),
                )
            ),
            material=material,
            name=f"knob_{index}",
        )


def _make_carriage_shape() -> cq.Workplane:
    collar_outer = 0.106
    collar_inner = 0.062
    collar = cq.Workplane("XY").circle(collar_outer).extrude(0.11, both=True)
    collar_bore = cq.Workplane("XY").circle(collar_inner).extrude(0.13, both=True)
    collar = collar.cut(collar_bore)
    arm = cq.Workplane("XY").box(0.20, 0.09, 0.09).translate((0.0, 0.100, -0.01))
    web = cq.Workplane("XY").box(0.15, 0.06, 0.10).translate((0.0, 0.085, -0.06))
    ear_0 = cq.Workplane("XY").box(0.035, 0.028, 0.10).translate((0.068, 0.140, -0.02))
    ear_1 = cq.Workplane("XY").box(0.035, 0.028, 0.10).translate((-0.068, 0.140, -0.02))
    crank_boss = cq.Workplane("YZ").circle(0.026).extrude(0.044, both=True).translate((0.118, 0.02, -0.01))
    return collar.union(arm).union(web).union(ear_0).union(ear_1).union(crank_boss)


def _make_table_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(0.36, 0.30, 0.03).translate((0.0, 0.18, 0.0))
    slot = cq.Workplane("XY").rect(0.050, 0.22).extrude(0.05, both=True).translate((0.0, 0.18, 0.0))
    trunnion = cq.Workplane("XY").box(0.08, 0.05, 0.06).translate((0.0, 0.051, -0.02))
    arm = cq.Workplane("XY").box(0.10, 0.12, 0.05).translate((0.0, 0.10, -0.04))
    rib_0 = cq.Workplane("XY").box(0.03, 0.08, 0.08).translate((0.050, 0.10, -0.06))
    rib_1 = cq.Workplane("XY").box(0.03, 0.08, 0.08).translate((-0.050, 0.10, -0.06))
    cheek_0 = cq.Workplane("XY").box(0.018, 0.04, 0.06).translate((0.024, 0.04, -0.01))
    cheek_1 = cq.Workplane("XY").box(0.018, 0.04, 0.06).translate((-0.024, 0.04, -0.01))
    barrel = cq.Workplane("YZ").circle(0.026).extrude(0.06, both=True)
    return (
        barrel.union(trunnion)
        .union(arm)
        .union(rib_0)
        .union(rib_1)
        .union(cheek_0)
        .union(cheek_1)
        .union(plate.cut(slot))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="floor_drill_press")

    cast_iron = model.material("cast_iron", rgba=(0.20, 0.23, 0.24, 1.0))
    machine_green = model.material("machine_green", rgba=(0.28, 0.40, 0.30, 1.0))
    steel = model.material("steel", rgba=(0.62, 0.64, 0.67, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.34, 0.36, 0.38, 1.0))
    handle_black = model.material("handle_black", rgba=(0.08, 0.08, 0.08, 1.0))
    polished = model.material("polished", rgba=(0.76, 0.78, 0.80, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.72, 0.50, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=cast_iron,
        name="base_plate",
    )
    base.visual(
        Box((0.22, 0.20, 0.18)),
        origin=Origin(xyz=(0.0, -0.10, 0.13)),
        material=cast_iron,
        name="column_pedestal",
    )
    column = model.part("column")
    column.visual(
        Cylinder(radius=0.055, length=1.35),
        origin=Origin(xyz=(0.0, 0.0, 0.675)),
        material=steel,
        name="column",
    )

    head = model.part("head")
    head.visual(
        Box((0.20, 0.10, 0.30)),
        origin=Origin(xyz=(0.0, 0.105, 0.0)),
        material=machine_green,
        name="rear_mount",
    )
    head.visual(
        Box((0.34, 0.22, 0.26)),
        origin=Origin(xyz=(0.0, 0.165, 0.01)),
        material=machine_green,
        name="head_casting",
    )
    head.visual(
        Box((0.42, 0.24, 0.16)),
        origin=Origin(xyz=(0.0, 0.175, 0.22)),
        material=machine_green,
        name="belt_cover",
    )
    head.visual(
        Cylinder(radius=0.09, length=0.26),
        origin=_x_cylinder_origin((0.0, 0.17, 0.29)),
        material=dark_steel,
        name="motor",
    )
    head.visual(
        Cylinder(radius=0.083, length=0.22),
        origin=Origin(xyz=(0.0, 0.24, -0.03)),
        material=machine_green,
        name="quill_housing",
    )
    head.visual(
        Cylinder(radius=0.028, length=0.24),
        origin=Origin(xyz=(0.0, 0.24, -0.20)),
        material=polished,
        name="spindle",
    )
    head.visual(
        Cylinder(radius=0.042, length=0.09),
        origin=Origin(xyz=(0.0, 0.24, -0.12)),
        material=dark_steel,
        name="nose_cap",
    )
    head.visual(
        Cylinder(radius=0.030, length=0.06),
        origin=_x_cylinder_origin((0.165, 0.11, 0.0)),
        material=dark_steel,
        name="feed_boss",
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_make_carriage_shape(), "drill_press_carriage"),
        material=machine_green,
        name="carriage_body",
    )

    table = model.part("table")
    table.visual(
        mesh_from_cadquery(_make_table_shape(), "drill_press_table"),
        material=cast_iron,
        name="table_body",
    )

    feed_handle = model.part("feed_handle")
    _add_rotary_handles(
        feed_handle,
        hub_length=0.14,
        hub_radius=0.018,
        rod_length=0.20,
        rod_radius=0.010,
        knob_radius=0.022,
        angles=[-pi / 2.0, pi / 6.0, 5.0 * pi / 6.0],
        stem_x=0.10,
        hub_name="feed_hub",
        material=handle_black,
    )

    raise_crank = model.part("raise_crank")
    _add_rotary_handles(
        raise_crank,
        hub_length=0.11,
        hub_radius=0.014,
        rod_length=0.14,
        rod_radius=0.009,
        knob_radius=0.018,
        angles=[-pi / 2.0],
        stem_x=0.08,
        hub_name="crank_hub",
        material=handle_black,
    )

    model.articulation(
        "base_to_column",
        ArticulationType.FIXED,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, -0.10, 0.22)),
    )
    model.articulation(
        "column_to_head",
        ArticulationType.FIXED,
        parent=column,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 1.15)),
    )
    model.articulation(
        "table_lift",
        ArticulationType.PRISMATIC,
        parent=column,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.78)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.18, lower=-0.24, upper=0.34),
    )
    model.articulation(
        "table_tilt",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=table,
        origin=Origin(xyz=(0.0, 0.140, -0.02)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.8, lower=-0.78, upper=0.78),
    )
    model.articulation(
        "feed_spin",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=feed_handle,
        origin=Origin(xyz=(0.195, 0.11, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=10.0),
    )
    model.articulation(
        "crank_spin",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=raise_crank,
        origin=Origin(xyz=(0.140, 0.02, -0.01)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=8.0),
    )

    return model


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    low, high = aabb
    return (
        0.5 * (low[0] + high[0]),
        0.5 * (low[1] + high[1]),
        0.5 * (low[2] + high[2]),
    )


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    column = object_model.get_part("column")
    carriage = object_model.get_part("carriage")
    table = object_model.get_part("table")
    feed_handle = object_model.get_part("feed_handle")
    raise_crank = object_model.get_part("raise_crank")

    table_lift = object_model.get_articulation("table_lift")
    table_tilt = object_model.get_articulation("table_tilt")
    feed_spin = object_model.get_articulation("feed_spin")
    crank_spin = object_model.get_articulation("crank_spin")

    ctx.allow_overlap(
        carriage,
        table,
        elem_a="carriage_body",
        elem_b="table_body",
        reason="The tilting table trunnion barrel is intentionally captured inside the carriage hinge cheeks.",
    )
    ctx.allow_overlap(
        carriage,
        raise_crank,
        elem_a="carriage_body",
        elem_b="crank_hub",
        reason="The raise crank hub is intentionally seated into the collar's shaft boss as a simplified rotating mount.",
    )

    ctx.expect_gap(
        table,
        base,
        axis="z",
        min_gap=0.62,
        name="table stays well above the base at nominal height",
    )
    ctx.expect_within(
        column,
        carriage,
        axes="xy",
        inner_elem="column",
        outer_elem="carriage_body",
        margin=0.0,
        name="column remains centered inside the table collar",
    )

    carriage_rest = ctx.part_world_position(carriage)
    with ctx.pose({table_lift: 0.28}):
        ctx.expect_within(
            column,
            carriage,
            axes="xy",
            inner_elem="column",
            outer_elem="carriage_body",
            margin=0.0,
            name="raised collar still surrounds the column",
        )
        carriage_high = ctx.part_world_position(carriage)
    ctx.check(
        "table lift raises the carriage upward",
        carriage_rest is not None and carriage_high is not None and carriage_high[2] > carriage_rest[2] + 0.20,
        details=f"rest={carriage_rest}, high={carriage_high}",
    )

    table_rest = ctx.part_world_aabb(table)
    with ctx.pose({table_tilt: 0.60}):
        table_tilted = ctx.part_world_aabb(table)
    ctx.check(
        "table tilt lifts the front edge",
        table_rest is not None and table_tilted is not None and table_tilted[1][2] > table_rest[1][2] + 0.09,
        details=f"rest={table_rest}, tilted={table_tilted}",
    )

    feed_knob_rest = _aabb_center(ctx.part_element_world_aabb(feed_handle, elem="knob_0"))
    with ctx.pose({feed_spin: pi}):
        feed_knob_turned = _aabb_center(ctx.part_element_world_aabb(feed_handle, elem="knob_0"))
    ctx.check(
        "quill feed handle rotates around its side shaft",
        feed_knob_rest is not None
        and feed_knob_turned is not None
        and feed_knob_turned[2] > feed_knob_rest[2] + 0.30,
        details=f"rest={feed_knob_rest}, turned={feed_knob_turned}",
    )

    crank_knob_rest = _aabb_center(ctx.part_element_world_aabb(raise_crank, elem="knob_0"))
    with ctx.pose({crank_spin: pi}):
        crank_knob_turned = _aabb_center(ctx.part_element_world_aabb(raise_crank, elem="knob_0"))
    ctx.check(
        "table raise crank swings through a full rotation on the collar shaft",
        crank_knob_rest is not None
        and crank_knob_turned is not None
        and crank_knob_turned[2] > crank_knob_rest[2] + 0.20,
        details=f"rest={crank_knob_rest}, turned={crank_knob_turned}",
    )

    return ctx.report()


object_model = build_object_model()
