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
    TrunnionYokeGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


COLUMN_RADIUS = 0.022
COLUMN_HEIGHT = 0.410

HEAD_COLUMN_Z = 0.315
TABLE_SLIDE_Z = 0.065
TABLE_TILT_Y = 0.110
TABLE_TILT_Z = -0.020


def _make_head_casting() -> cq.Workplane:
    bore_radius = COLUMN_RADIUS + 0.0006

    sleeve = cq.Workplane("XY").circle(0.050).extrude(0.042, both=True)
    body = cq.Workplane("XY").box(0.112, 0.178, 0.096).translate((0.0, 0.074, 0.004))
    belt_cover = cq.Workplane("XY").box(0.124, 0.100, 0.056).translate((0.0, 0.012, 0.068))
    motor = cq.Workplane("YZ").circle(0.036).extrude(0.064, both=True).translate((0.0, -0.046, 0.072))
    nose = cq.Workplane("XY").box(0.056, 0.060, 0.086).translate((0.0, 0.128, -0.008))
    quill = cq.Workplane("XY").circle(0.021).extrude(0.070).translate((0.0, 0.128, -0.090))
    chuck = cq.Workplane("XY").circle(0.0135).extrude(0.055).translate((0.0, 0.128, -0.145))
    bore = cq.Workplane("XY").circle(bore_radius).extrude(0.260, both=True)

    return (
        sleeve.union(body)
        .union(belt_cover)
        .union(motor)
        .union(nose)
        .union(quill)
        .union(chuck)
        .cut(bore)
    )


def _make_table_carriage_body() -> cq.Workplane:
    bore_radius = COLUMN_RADIUS + 0.0025

    collar = cq.Workplane("XY").circle(0.048).extrude(0.035, both=True)
    arm = cq.Workplane("XY").box(0.050, 0.094, 0.040).translate((0.0, 0.051, -0.045))
    crank_boss = cq.Workplane("XY").box(0.020, 0.032, 0.024).translate((0.038, 0.008, -0.032))
    web = cq.Workplane("YZ").rect(0.060, 0.018).extrude(0.022, both=True).translate((0.0, 0.084, -0.040))
    bore = cq.Workplane("XY").circle(bore_radius).extrude(0.220, both=True)

    return collar.union(arm).union(crank_boss).union(web).cut(bore)


def _aabb_center(aabb):
    if aabb is None:
        return None
    return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_drill_press")

    cast_iron = model.material("cast_iron", rgba=(0.23, 0.23, 0.24, 1.0))
    machine_gray = model.material("machine_gray", rgba=(0.56, 0.58, 0.60, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.34, 0.36, 0.38, 1.0))
    steel = model.material("steel", rgba=(0.74, 0.75, 0.77, 1.0))
    handle_black = model.material("handle_black", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.280, 0.180, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=cast_iron,
        name="base_plate",
    )
    base.visual(
        Box((0.102, 0.118, 0.038)),
        origin=Origin(xyz=(0.0, -0.010, 0.041)),
        material=cast_iron,
        name="base_pedestal",
    )
    base.visual(
        Cylinder(radius=0.038, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.069)),
        material=machine_gray,
        name="column_boss",
    )

    column = model.part("column")
    column.visual(
        Cylinder(radius=COLUMN_RADIUS, length=COLUMN_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, COLUMN_HEIGHT * 0.5)),
        material=steel,
        name="column_tube",
    )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(_make_head_casting(), "drill_press_head"),
        material=machine_gray,
        name="head_casting",
    )
    head.visual(
        Cylinder(radius=0.012, length=0.016),
        origin=Origin(xyz=(0.056, 0.086, 0.008), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="feed_boss",
    )
    head.visual(
        Box((0.024, 0.028, 0.040)),
        origin=Origin(xyz=(0.0, -0.036, 0.010)),
        material=dark_steel,
        name="head_clamp_tab",
    )

    table_carriage = model.part("table_carriage")
    table_carriage.visual(
        mesh_from_cadquery(_make_table_carriage_body(), "table_carriage_body"),
        material=machine_gray,
        name="carriage_body",
    )
    table_carriage.visual(
        mesh_from_geometry(
            TrunnionYokeGeometry(
                (0.100, 0.030, 0.055),
                span_width=0.062,
                trunnion_diameter=0.022,
                trunnion_center_z=0.038,
                base_thickness=0.012,
                center=False,
            ),
            "table_carriage_yoke",
        ),
        origin=Origin(xyz=(0.0, TABLE_TILT_Y, TABLE_TILT_Z - 0.038)),
        material=dark_steel,
        name="table_yoke",
    )
    table_carriage.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(xyz=(0.043, 0.006, -0.034), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="crank_boss",
    )

    table = model.part("table")
    table.visual(
        Box((0.160, 0.160, 0.012)),
        origin=Origin(xyz=(0.0, 0.045, 0.026)),
        material=cast_iron,
        name="table_top",
    )
    table.visual(
        Box((0.040, 0.050, 0.018)),
        origin=Origin(xyz=(0.0, 0.000, 0.011)),
        material=machine_gray,
        name="table_neck",
    )
    table.visual(
        Box((0.050, 0.090, 0.020)),
        origin=Origin(xyz=(0.0, 0.067, 0.012)),
        material=machine_gray,
        name="table_rib",
    )
    table.visual(
        Cylinder(radius=0.0110, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="table_trunnion",
    )

    feed_wheel = model.part("feed_wheel")
    feed_wheel.visual(
        Cylinder(radius=0.013, length=0.018),
        origin=Origin(xyz=(0.009, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="feed_hub",
    )
    feed_wheel.visual(
        Cylinder(radius=0.045, length=0.012),
        origin=Origin(xyz=(0.011, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machine_gray,
        name="feed_disk",
    )
    feed_wheel.visual(
        Cylinder(radius=0.005, length=0.020),
        origin=Origin(xyz=(0.018, 0.028, 0.026), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=handle_black,
        name="feed_grip",
    )

    table_crank = model.part("table_crank")
    table_crank.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(xyz=(0.005, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="crank_hub",
    )
    table_crank.visual(
        Box((0.008, 0.040, 0.010)),
        origin=Origin(xyz=(0.009, 0.018, -0.008)),
        material=dark_steel,
        name="crank_arm",
    )
    table_crank.visual(
        Cylinder(radius=0.007, length=0.010),
        origin=Origin(xyz=(0.009, 0.036, -0.008), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machine_gray,
        name="crank_knuckle",
    )
    table_crank.visual(
        Cylinder(radius=0.0045, length=0.018),
        origin=Origin(xyz=(0.015, 0.046, -0.008), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=handle_black,
        name="crank_grip",
    )

    model.articulation(
        "base_to_column",
        ArticulationType.FIXED,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
    )
    model.articulation(
        "column_to_head",
        ArticulationType.FIXED,
        parent=column,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, HEAD_COLUMN_Z)),
    )
    model.articulation(
        "column_to_table_carriage",
        ArticulationType.PRISMATIC,
        parent=column,
        child=table_carriage,
        origin=Origin(xyz=(0.0, 0.0, TABLE_SLIDE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.25, lower=0.0, upper=0.055),
    )
    model.articulation(
        "carriage_to_table",
        ArticulationType.REVOLUTE,
        parent=table_carriage,
        child=table,
        origin=Origin(xyz=(0.0, TABLE_TILT_Y, TABLE_TILT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.4,
            lower=-0.60,
            upper=0.60,
        ),
    )
    model.articulation(
        "head_to_feed_wheel",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=feed_wheel,
        origin=Origin(xyz=(0.064, 0.086, 0.008)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=12.0),
    )
    model.articulation(
        "carriage_to_table_crank",
        ArticulationType.CONTINUOUS,
        parent=table_carriage,
        child=table_crank,
        origin=Origin(xyz=(0.050, 0.006, -0.034)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    head = object_model.get_part("head")
    table = object_model.get_part("table")
    table_carriage = object_model.get_part("table_carriage")
    feed_wheel = object_model.get_part("feed_wheel")
    table_crank = object_model.get_part("table_crank")

    table_slide = object_model.get_articulation("column_to_table_carriage")
    table_tilt = object_model.get_articulation("carriage_to_table")
    feed_joint = object_model.get_articulation("head_to_feed_wheel")
    crank_joint = object_model.get_articulation("carriage_to_table_crank")

    slide_upper = table_slide.motion_limits.upper or 0.0

    ctx.allow_overlap(
        table,
        table_carriage,
        elem_a="table_trunnion",
        elem_b="table_yoke",
        reason="The table trunnion is intentionally represented as a close-fit hinge journal nested into the yoke support.",
    )

    rest_table_pos = ctx.part_world_position(table)
    with ctx.pose({table_slide: slide_upper}):
        raised_table_pos = ctx.part_world_position(table)
        ctx.expect_gap(
            head,
            table,
            axis="z",
            min_gap=0.035,
            name="raised table stays below the spindle head",
        )

    ctx.check(
        "table carriage slides upward on the column",
        rest_table_pos is not None
        and raised_table_pos is not None
        and raised_table_pos[2] > rest_table_pos[2] + 0.045,
        details=f"rest={rest_table_pos}, raised={raised_table_pos}",
    )

    rest_table_aabb = ctx.part_world_aabb(table)
    with ctx.pose({table_tilt: 0.45}):
        tilted_table_aabb = ctx.part_world_aabb(table)

    rest_height = (
        (rest_table_aabb[1][2] - rest_table_aabb[0][2]) if rest_table_aabb is not None else None
    )
    tilted_height = (
        (tilted_table_aabb[1][2] - tilted_table_aabb[0][2]) if tilted_table_aabb is not None else None
    )
    ctx.check(
        "table tilt changes the table plane",
        rest_height is not None and tilted_height is not None and tilted_height > rest_height + 0.040,
        details=f"rest_height={rest_height}, tilted_height={tilted_height}",
    )

    feed_rest = _aabb_center(ctx.part_element_world_aabb(feed_wheel, elem="feed_grip"))
    with ctx.pose({feed_joint: math.pi / 2.0}):
        feed_rotated = _aabb_center(ctx.part_element_world_aabb(feed_wheel, elem="feed_grip"))
    feed_motion = (
        math.hypot(feed_rotated[1] - feed_rest[1], feed_rotated[2] - feed_rest[2])
        if feed_rest is not None and feed_rotated is not None
        else None
    )
    ctx.check(
        "feed wheel grip circles the quill feed axis",
        feed_motion is not None and feed_motion > 0.020,
        details=f"rest={feed_rest}, rotated={feed_rotated}, motion={feed_motion}",
    )

    crank_rest = _aabb_center(ctx.part_element_world_aabb(table_crank, elem="crank_grip"))
    with ctx.pose({crank_joint: math.pi / 2.0}):
        crank_rotated = _aabb_center(ctx.part_element_world_aabb(table_crank, elem="crank_grip"))
    crank_motion = (
        math.hypot(crank_rotated[1] - crank_rest[1], crank_rotated[2] - crank_rest[2])
        if crank_rest is not None and crank_rotated is not None
        else None
    )
    ctx.check(
        "table height crank grip rotates on its shaft",
        crank_motion is not None and crank_motion > 0.020,
        details=f"rest={crank_rest}, rotated={crank_rotated}, motion={crank_motion}",
    )

    return ctx.report()


object_model = build_object_model()
