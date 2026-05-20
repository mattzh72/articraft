from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Mesh,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _hollow_column_sleeve() -> Mesh:
    height = 0.120
    outer_radius = 0.065
    inner_radius = 0.045
    body = cq.Workplane("XY").circle(outer_radius).extrude(height)
    bore = (
        cq.Workplane("XY")
        .circle(inner_radius)
        .extrude(height + 0.004)
        .translate((0.0, 0.0, -0.002))
    )
    rack_clearance = (
        cq.Workplane("XY")
        .box(0.090, 0.032, height + 0.006)
        .translate((0.055, 0.0, height / 2.0))
    )
    return mesh_from_cadquery(
        body.cut(bore).cut(rack_clearance).translate((0.0, 0.0, -height / 2.0)),
        "column_sleeve",
        tolerance=0.001,
        angular_tolerance=0.08,
    )


def _slotted_round_table() -> Mesh:
    radius = 0.160
    thickness = 0.026
    table = cq.Workplane("XY").circle(radius).extrude(thickness)

    for y in (-0.046, 0.046):
        slot = (
            cq.Workplane("XY")
            .box(0.230, 0.018, thickness + 0.006)
            .translate((0.0, y, thickness / 2.0))
        )
        table = table.cut(slot)

    center_bore = (
        cq.Workplane("XY")
        .circle(0.018)
        .extrude(thickness + 0.006)
        .translate((0.0, 0.0, -0.003))
    )
    table = table.cut(center_bore)
    return mesh_from_cadquery(
        table.translate((0.0, 0.0, -thickness / 2.0)),
        "slotted_table",
        tolerance=0.001,
        angular_tolerance=0.08,
    )


def _pinion_gear() -> Mesh:
    teeth = 18
    root_radius = 0.027
    tip_radius = 0.032
    profile = []
    for i in range(teeth * 2):
        angle = 2.0 * math.pi * i / (teeth * 2)
        radius = tip_radius if i % 2 == 0 else root_radius
        profile.append((radius * math.cos(angle), radius * math.sin(angle)))
    return mesh_from_geometry(
        ExtrudeGeometry(profile, 0.026, center=True),
        "pinion_gear",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="laser_guide_bench_drill_press")

    enamel = model.material("deep_green_enamel", rgba=(0.035, 0.18, 0.145, 1.0))
    cast_iron = model.material("dark_cast_iron", rgba=(0.10, 0.105, 0.10, 1.0))
    steel = model.material("brushed_steel", rgba=(0.58, 0.61, 0.62, 1.0))
    black = model.material("matte_black", rgba=(0.01, 0.01, 0.012, 1.0))
    laser_red = model.material("laser_red", rgba=(1.0, 0.02, 0.0, 1.0))
    cover_red = model.material("clear_red_cover", rgba=(1.0, 0.08, 0.02, 0.38))

    column_x = -0.180

    frame = model.part("frame")
    frame.visual(
        Box((0.700, 0.420, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=enamel,
        name="base_plate",
    )
    frame.visual(
        Box((0.460, 0.300, 0.014)),
        origin=Origin(xyz=(0.040, 0.0, 0.061)),
        material=black,
        name="base_slot",
    )
    frame.visual(
        Cylinder(radius=0.065, length=0.036),
        origin=Origin(xyz=(column_x, 0.0, 0.073)),
        material=cast_iron,
        name="column_foot",
    )
    frame.visual(
        Cylinder(radius=0.040, length=1.050),
        origin=Origin(xyz=(column_x, 0.0, 0.580)),
        material=steel,
        name="column",
    )

    # A visible rack fixed to the front of the column. The small teeth are
    # slightly seated into the strip so the rack reads as one mounted member.
    frame.visual(
        Box((0.014, 0.018, 0.730)),
        origin=Origin(xyz=(column_x + 0.047, 0.0, 0.485)),
        material=steel,
        name="rack_backbone",
    )
    for index in range(22):
        frame.visual(
            Box((0.018, 0.020, 0.010)),
            origin=Origin(xyz=(column_x + 0.056, 0.0, 0.150 + index * 0.030)),
            material=steel,
            name=f"rack_tooth_{index}",
        )

    frame.visual(
        Box((0.460, 0.230, 0.200)),
        origin=Origin(xyz=(0.050, 0.0, 1.080)),
        material=enamel,
        name="head_body",
    )
    frame.visual(
        Cylinder(radius=0.085, length=0.240),
        origin=Origin(xyz=(-0.020, 0.0, 1.205), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=enamel,
        name="motor_housing",
    )
    frame.visual(
        Cylinder(radius=0.048, length=0.085),
        origin=Origin(xyz=(0.180, 0.0, 0.942)),
        material=cast_iron,
        name="spindle_nose",
    )
    frame.visual(
        Box((0.006, 0.038, 0.024)),
        origin=Origin(xyz=(0.283, -0.030, 1.080)),
        material=black,
        name="laser_window_0",
    )
    frame.visual(
        Box((0.006, 0.038, 0.024)),
        origin=Origin(xyz=(0.283, 0.030, 1.080)),
        material=black,
        name="laser_window_1",
    )
    frame.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(0.286, -0.030, 1.080), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=laser_red,
        name="laser_lens_0",
    )
    frame.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(0.286, 0.030, 1.080), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=laser_red,
        name="laser_lens_1",
    )

    table_carriage = model.part("table_carriage")
    table_carriage.visual(
        _hollow_column_sleeve(),
        origin=Origin(),
        material=cast_iron,
        name="column_sleeve",
    )
    table_carriage.visual(
        Box((0.114, 0.080, 0.035)),
        origin=Origin(xyz=(0.123, 0.0, 0.070)),
        material=cast_iron,
        name="tilt_support",
    )
    table_carriage.visual(
        Box((0.040, 0.014, 0.025)),
        origin=Origin(xyz=(0.066, -0.032, 0.070)),
        material=cast_iron,
        name="sleeve_lug_0",
    )
    table_carriage.visual(
        Box((0.040, 0.014, 0.025)),
        origin=Origin(xyz=(0.066, 0.032, 0.070)),
        material=cast_iron,
        name="sleeve_lug_1",
    )
    table_carriage.visual(
        Box((0.080, 0.112, 0.025)),
        origin=Origin(xyz=(0.200, 0.0, 0.060)),
        material=cast_iron,
        name="tilt_base",
    )
    table_carriage.visual(
        Box((0.080, 0.014, 0.082)),
        origin=Origin(xyz=(0.200, -0.048, 0.095)),
        material=cast_iron,
        name="tilt_cheek_0",
    )
    table_carriage.visual(
        Box((0.080, 0.014, 0.082)),
        origin=Origin(xyz=(0.200, 0.048, 0.095)),
        material=cast_iron,
        name="tilt_cheek_1",
    )
    table_carriage.visual(
        Box((0.040, 0.014, 0.018)),
        origin=Origin(xyz=(0.053, -0.030, 0.000)),
        material=cast_iron,
        name="bearing_bridge_0",
    )
    table_carriage.visual(
        Box((0.040, 0.014, 0.018)),
        origin=Origin(xyz=(0.053, 0.030, 0.000)),
        material=cast_iron,
        name="bearing_bridge_1",
    )
    table_carriage.visual(
        Box((0.050, 0.014, 0.050)),
        origin=Origin(xyz=(0.092, -0.030, 0.000)),
        material=cast_iron,
        name="pinion_bearing_0",
    )
    table_carriage.visual(
        Box((0.050, 0.014, 0.050)),
        origin=Origin(xyz=(0.092, 0.030, 0.000)),
        material=cast_iron,
        name="pinion_bearing_1",
    )

    table_crank = model.part("table_crank")
    table_crank.visual(
        _pinion_gear(),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pinion_gear",
    )
    table_crank.visual(
        Cylinder(radius=0.012, length=0.170),
        origin=Origin(xyz=(0.0, -0.072, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="crank_shaft",
    )
    table_crank.visual(
        Box((0.018, 0.090, 0.014)),
        origin=Origin(xyz=(0.0, -0.160, 0.0)),
        material=steel,
        name="crank_arm",
    )
    table_crank.visual(
        Cylinder(radius=0.013, length=0.060),
        origin=Origin(xyz=(0.0, -0.205, -0.030)),
        material=black,
        name="crank_handle",
    )

    work_table = model.part("work_table")
    work_table.visual(
        Cylinder(radius=0.017, length=0.118),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="table_trunnion",
    )
    work_table.visual(
        Box((0.160, 0.045, 0.035)),
        origin=Origin(xyz=(0.080, 0.0, 0.025)),
        material=cast_iron,
        name="table_arm",
    )
    work_table.visual(
        Cylinder(radius=0.055, length=0.040),
        origin=Origin(xyz=(0.160, 0.0, 0.050)),
        material=cast_iron,
        name="table_hub",
    )
    work_table.visual(
        _slotted_round_table(),
        origin=Origin(xyz=(0.160, 0.0, 0.075)),
        material=cast_iron,
        name="table_top",
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.018, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        material=steel,
        name="spindle_shaft",
    )
    spindle.visual(
        Cylinder(radius=0.032, length=0.066),
        origin=Origin(xyz=(0.0, 0.0, -0.103)),
        material=cast_iron,
        name="chuck_body",
    )
    spindle.visual(
        Cylinder(radius=0.017, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, -0.154)),
        material=steel,
        name="chuck_nose",
    )
    spindle.visual(
        Cylinder(radius=0.0055, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, -0.199)),
        material=steel,
        name="drill_bit",
    )
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        spindle.visual(
            Box((0.006, 0.020, 0.034)),
            origin=Origin(
                xyz=(0.018 * math.cos(angle), 0.018 * math.sin(angle), -0.139),
                rpy=(0.0, 0.0, angle),
            ),
            material=black,
            name=f"chuck_jaw_{index}",
        )

    laser_cover = model.part("laser_cover")
    laser_cover.visual(
        Box((0.012, 0.120, 0.102)),
        origin=Origin(xyz=(0.006, 0.060, 0.0)),
        material=cover_red,
        name="cover_panel",
    )
    laser_cover.visual(
        Cylinder(radius=0.008, length=0.112),
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
        material=cover_red,
        name="cover_hinge_barrel",
    )
    laser_cover.visual(
        Box((0.016, 0.024, 0.016)),
        origin=Origin(xyz=(0.012, 0.116, -0.035)),
        material=black,
        name="cover_latch",
    )

    model.articulation(
        "column_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=table_carriage,
        origin=Origin(xyz=(column_x, 0.0, 0.250)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.12, lower=0.0, upper=0.250),
    )
    model.articulation(
        "carriage_to_crank",
        ArticulationType.REVOLUTE,
        parent=table_carriage,
        child=table_crank,
        origin=Origin(xyz=(0.092, 0.0, 0.000)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "carriage_to_table",
        ArticulationType.REVOLUTE,
        parent=table_carriage,
        child=work_table,
        origin=Origin(xyz=(0.200, 0.0, 0.095)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.7, lower=-0.65, upper=0.65),
    )
    model.articulation(
        "head_to_spindle",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=spindle,
        origin=Origin(xyz=(0.180, 0.0, 0.900)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=30.0, lower=-2.0 * math.pi, upper=2.0 * math.pi),
    )
    model.articulation(
        "head_to_laser_cover",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=laser_cover,
        origin=Origin(xyz=(0.292, -0.066, 1.080)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.5, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("table_carriage")
    crank = object_model.get_part("table_crank")
    table = object_model.get_part("work_table")
    spindle = object_model.get_part("spindle")
    cover = object_model.get_part("laser_cover")
    slide = object_model.get_articulation("column_to_carriage")
    tilt = object_model.get_articulation("carriage_to_table")
    cover_hinge = object_model.get_articulation("head_to_laser_cover")

    ctx.allow_overlap(
        carriage,
        table,
        elem_a="tilt_cheek_0",
        elem_b="table_trunnion",
        reason="The table trunnion is intentionally captured inside the solid cheek proxy of the tilt hinge.",
    )
    ctx.allow_overlap(
        carriage,
        table,
        elem_a="tilt_cheek_1",
        elem_b="table_trunnion",
        reason="The table trunnion is intentionally captured inside the solid cheek proxy of the tilt hinge.",
    )
    ctx.allow_overlap(
        carriage,
        crank,
        elem_a="pinion_bearing_0",
        elem_b="crank_shaft",
        reason="The rack-and-pinion crank shaft is intentionally seated through the bearing block proxy.",
    )

    ctx.expect_overlap(
        carriage,
        frame,
        axes="z",
        elem_a="column_sleeve",
        elem_b="column",
        min_overlap=0.08,
        name="carriage sleeve stays engaged around column",
    )
    ctx.expect_overlap(
        table,
        carriage,
        axes="y",
        elem_a="table_trunnion",
        elem_b="tilt_cheek_0",
        min_overlap=0.006,
        name="trunnion enters first tilt cheek",
    )
    ctx.expect_overlap(
        table,
        carriage,
        axes="y",
        elem_a="table_trunnion",
        elem_b="tilt_cheek_1",
        min_overlap=0.006,
        name="trunnion enters second tilt cheek",
    )
    ctx.expect_overlap(
        crank,
        carriage,
        axes="y",
        elem_a="crank_shaft",
        elem_b="pinion_bearing_0",
        min_overlap=0.010,
        name="crank shaft passes through bearing block",
    )
    ctx.expect_contact(
        spindle,
        frame,
        elem_a="spindle_shaft",
        elem_b="spindle_nose",
        contact_tol=0.002,
        name="spindle shaft is seated in head nose",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.25}):
        raised_pos = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            carriage,
            frame,
            axes="z",
            elem_a="column_sleeve",
            elem_b="column",
            min_overlap=0.08,
            name="raised carriage sleeve remains on column",
        )
    ctx.check(
        "table carriage raises on column",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.20,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    flat_aabb = ctx.part_element_world_aabb(table, elem="table_top")
    with ctx.pose({tilt: 0.55}):
        tilted_aabb = ctx.part_element_world_aabb(table, elem="table_top")
    flat_span = flat_aabb[1][2] - flat_aabb[0][2] if flat_aabb is not None else 0.0
    tilted_span = tilted_aabb[1][2] - tilted_aabb[0][2] if tilted_aabb is not None else 0.0
    ctx.check(
        "round table tilts about bracket hinge",
        tilted_span > flat_span + 0.07,
        details=f"flat_span={flat_span}, tilted_span={tilted_span}",
    )

    closed_cover = ctx.part_element_world_aabb(cover, elem="cover_panel")
    with ctx.pose({cover_hinge: 1.0}):
        opened_cover = ctx.part_element_world_aabb(cover, elem="cover_panel")
    ctx.check(
        "laser cover opens forward from head",
        closed_cover is not None
        and opened_cover is not None
        and opened_cover[1][0] > closed_cover[1][0] + 0.05,
        details=f"closed={closed_cover}, opened={opened_cover}",
    )

    return ctx.report()


object_model = build_object_model()
