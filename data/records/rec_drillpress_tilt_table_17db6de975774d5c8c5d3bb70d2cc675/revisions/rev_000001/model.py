from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


CAST_IRON = Material("dark green-gray painted cast iron", color=(0.18, 0.24, 0.22, 1.0))
BASE_PAINT = Material("charcoal hammered paint", color=(0.08, 0.09, 0.09, 1.0))
BARE_STEEL = Material("brushed steel", color=(0.62, 0.64, 0.62, 1.0))
BLACK_STEEL = Material("blackened steel", color=(0.015, 0.016, 0.015, 1.0))
RUBBER = Material("black rubber", color=(0.01, 0.01, 0.01, 1.0))
SLOT_DARK = Material("dark recessed table slots", color=(0.02, 0.025, 0.025, 1.0))


def _tube_mesh(outer_radius: float, inner_radius: float, length: float, name: str):
    """Vertical hollow sleeve centered on the local origin, with a front rack window."""
    tube = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((0.0, 0.0, -0.5 * length))
    )
    rack_window = cq.Workplane("XY").box(0.045, 0.075, length + 0.006).translate(
        (0.0, -outer_radius, 0.0)
    )
    tube = tube.cut(rack_window)
    return mesh_from_cadquery(tube, name, tolerance=0.0007, angular_tolerance=0.08)


def _table_mesh():
    """Small square drill-press table with a through hole and shallow T-slot grooves."""
    table = cq.Workplane("XY").box(0.22, 0.20, 0.016)
    table = table.faces(">Z").workplane().circle(0.018).cutThruAll()
    for x in (-0.055, 0.055):
        table = (
            table.faces(">Z")
            .workplane(centerOption="CenterOfMass")
            .center(x, 0.0)
            .rect(0.018, 0.150)
            .cutBlind(-0.004)
        )
    table = table.edges("|Z").fillet(0.006)
    return mesh_from_cadquery(table, "square_tilting_table", tolerance=0.0008, angular_tolerance=0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_hobby_drill_press")

    frame = model.part("frame")
    frame.visual(
        Box((0.36, 0.26, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=BASE_PAINT,
        name="rectangular_base",
    )
    frame.visual(
        Box((0.24, 0.16, 0.018)),
        origin=Origin(xyz=(0.0, 0.015, 0.041)),
        material=CAST_IRON,
        name="raised_base_pad",
    )
    frame.visual(
        Cylinder(radius=0.036, length=0.022),
        origin=Origin(xyz=(0.0, 0.080, 0.049)),
        material=CAST_IRON,
        name="column_foot_flange",
    )
    frame.visual(
        Cylinder(radius=0.016, length=0.710),
        origin=Origin(xyz=(0.0, 0.080, 0.395)),
        material=BARE_STEEL,
        name="slim_column",
    )
    frame.visual(
        Box((0.010, 0.006, 0.430)),
        origin=Origin(xyz=(0.0, 0.061, 0.330)),
        material=BLACK_STEEL,
        name="front_rack",
    )
    for i in range(14):
        frame.visual(
            Box((0.030, 0.006, 0.006)),
            origin=Origin(xyz=(0.0, 0.056, 0.135 + i * 0.028)),
            material=BARE_STEEL,
            name=f"rack_tooth_{i}",
        )

    frame.visual(
        Box((0.110, 0.170, 0.082)),
        origin=Origin(xyz=(0.0, -0.010, 0.704)),
        material=CAST_IRON,
        name="head_casting",
    )
    frame.visual(
        Box((0.100, 0.130, 0.030)),
        origin=Origin(xyz=(0.0, -0.022, 0.758)),
        material=BASE_PAINT,
        name="belt_cover",
    )
    frame.visual(
        Cylinder(radius=0.035, length=0.105),
        origin=Origin(xyz=(0.0, 0.110, 0.716), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=BASE_PAINT,
        name="rear_motor",
    )
    frame.visual(
        Cylinder(radius=0.014, length=0.115),
        origin=Origin(xyz=(0.0, -0.062, 0.620)),
        material=BARE_STEEL,
        name="quill",
    )
    frame.visual(
        Cylinder(radius=0.020, length=0.040),
        origin=Origin(xyz=(0.0, -0.062, 0.544)),
        material=BLACK_STEEL,
        name="chuck_body",
    )
    frame.visual(
        Cylinder(radius=0.004, length=0.105),
        origin=Origin(xyz=(0.0, -0.062, 0.482)),
        material=BARE_STEEL,
        name="drill_bit",
    )

    saddle = model.part("saddle")
    saddle.visual(
        _tube_mesh(0.040, 0.0162, 0.082, "sliding_column_collar"),
        material=CAST_IRON,
        name="sliding_collar",
    )
    saddle.visual(
        Box((0.003, 0.014, 0.040)),
        origin=Origin(xyz=(-0.0175, 0.000, 0.000)),
        material=BARE_STEEL,
        name="gib_pad_0",
    )
    saddle.visual(
        Box((0.003, 0.014, 0.040)),
        origin=Origin(xyz=(0.0175, 0.000, 0.000)),
        material=BARE_STEEL,
        name="gib_pad_1",
    )
    saddle.visual(
        Box((0.058, 0.085, 0.026)),
        origin=Origin(xyz=(0.0, -0.076, -0.025)),
        material=CAST_IRON,
        name="table_arm",
    )
    saddle.visual(
        Box((0.012, 0.066, 0.018)),
        origin=Origin(xyz=(-0.033, -0.052, -0.025)),
        material=CAST_IRON,
        name="arm_side_rib_0",
    )
    saddle.visual(
        Box((0.012, 0.066, 0.018)),
        origin=Origin(xyz=(0.033, -0.052, -0.025)),
        material=CAST_IRON,
        name="arm_side_rib_1",
    )
    saddle.visual(
        Box((0.274, 0.032, 0.014)),
        origin=Origin(xyz=(0.0, -0.070, -0.021)),
        material=CAST_IRON,
        name="hinge_crossbar",
    )
    saddle.visual(
        Box((0.020, 0.036, 0.038)),
        origin=Origin(xyz=(-0.127, -0.070, 0.000)),
        material=CAST_IRON,
        name="hinge_cheek_0",
    )
    saddle.visual(
        Box((0.020, 0.036, 0.038)),
        origin=Origin(xyz=(0.127, -0.070, 0.000)),
        material=CAST_IRON,
        name="hinge_cheek_1",
    )
    saddle.visual(
        Cylinder(radius=0.014, length=0.044),
        origin=Origin(xyz=(0.048, 0.005, 0.006), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=CAST_IRON,
        name="crank_boss",
    )

    table = model.part("table")
    table.visual(
        _table_mesh(),
        origin=Origin(xyz=(0.0, -0.085, 0.011)),
        material=CAST_IRON,
        name="square_table_plate",
    )
    table.visual(
        Cylinder(radius=0.012, length=0.224),
        origin=Origin(xyz=(0.0, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=BARE_STEEL,
        name="tilt_hinge_barrel",
    )
    table.visual(
        Box((0.070, 0.038, 0.016)),
        origin=Origin(xyz=(0.0, -0.022, 0.002)),
        material=CAST_IRON,
        name="rear_table_lug",
    )
    for x in (-0.055, 0.055):
        table.visual(
            Box((0.012, 0.150, 0.0022)),
            origin=Origin(xyz=(x, -0.085, 0.0187)),
            material=SLOT_DARK,
            name=f"dark_t_slot_{0 if x < 0 else 1}",
        )

    crank = model.part("height_crank")
    crank.visual(
        Cylinder(radius=0.0055, length=0.035),
        origin=Origin(xyz=(0.0175, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=BARE_STEEL,
        name="short_shaft",
    )
    crank.visual(
        Cylinder(radius=0.024, length=0.010),
        origin=Origin(xyz=(0.039, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=BLACK_STEEL,
        name="crank_disc",
    )
    crank.visual(
        Box((0.006, 0.052, 0.006)),
        origin=Origin(xyz=(0.047, -0.020, 0.0)),
        material=BLACK_STEEL,
        name="crank_arm",
    )
    crank.visual(
        Cylinder(radius=0.006, length=0.026),
        origin=Origin(xyz=(0.056, -0.046, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=RUBBER,
        name="handle_grip",
    )

    model.articulation(
        "column_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=saddle,
        origin=Origin(xyz=(0.0, 0.080, 0.320)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.18, lower=-0.100, upper=0.140),
    )
    model.articulation(
        "table_tilt",
        ArticulationType.REVOLUTE,
        parent=saddle,
        child=table,
        origin=Origin(xyz=(0.0, -0.070, 0.000)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.2, lower=-0.70, upper=0.70),
    )
    model.articulation(
        "crank_spin",
        ArticulationType.CONTINUOUS,
        parent=saddle,
        child=crank,
        origin=Origin(xyz=(0.070, 0.005, 0.006)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    slide = object_model.get_articulation("column_slide")
    tilt = object_model.get_articulation("table_tilt")
    crank_spin = object_model.get_articulation("crank_spin")
    saddle = object_model.get_part("saddle")
    table = object_model.get_part("table")
    frame = object_model.get_part("frame")
    crank = object_model.get_part("height_crank")

    ctx.check(
        "primary mechanisms are articulated",
        slide.articulation_type == ArticulationType.PRISMATIC
        and tilt.articulation_type == ArticulationType.REVOLUTE
        and crank_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"types={[slide.articulation_type, tilt.articulation_type, crank_spin.articulation_type]}",
    )
    ctx.expect_gap(
        table,
        frame,
        axis="z",
        min_gap=0.040,
        positive_elem="square_table_plate",
        negative_elem="rectangular_base",
        name="table rides well above the base",
    )
    ctx.expect_overlap(
        table,
        frame,
        axes="xy",
        min_overlap=0.006,
        elem_a="square_table_plate",
        elem_b="drill_bit",
        name="table is centered under the drill line",
    )

    rest_saddle = ctx.part_world_position(saddle)
    with ctx.pose({slide: 0.120}):
        raised_saddle = ctx.part_world_position(saddle)
        ctx.expect_gap(
            table,
            frame,
            axis="z",
            min_gap=0.150,
            positive_elem="square_table_plate",
            negative_elem="rectangular_base",
            name="raised table still clears the base",
        )
    ctx.check(
        "table bracket slides upward on the column",
        rest_saddle is not None and raised_saddle is not None and raised_saddle[2] > rest_saddle[2] + 0.10,
        details=f"rest={rest_saddle}, raised={raised_saddle}",
    )

    with ctx.pose({tilt: 0.55}):
        tilted_table_aabb = ctx.part_element_world_aabb(table, elem="square_table_plate")
    with ctx.pose({tilt: 0.0}):
        level_table_aabb = ctx.part_element_world_aabb(table, elem="square_table_plate")
    ctx.check(
        "table tilts about a horizontal support hinge",
        tilted_table_aabb is not None
        and level_table_aabb is not None
        and (tilted_table_aabb[1][2] - tilted_table_aabb[0][2])
        > (level_table_aabb[1][2] - level_table_aabb[0][2]) + 0.040,
        details=f"level={level_table_aabb}, tilted={tilted_table_aabb}",
    )

    with ctx.pose({crank_spin: 0.0}):
        grip_aabb_0 = ctx.part_element_world_aabb(crank, elem="handle_grip")
    with ctx.pose({crank_spin: math.pi / 2.0}):
        grip_aabb_90 = ctx.part_element_world_aabb(crank, elem="handle_grip")
    if grip_aabb_0 is not None and grip_aabb_90 is not None:
        grip0_y = 0.5 * (grip_aabb_0[0][1] + grip_aabb_0[1][1])
        grip0_z = 0.5 * (grip_aabb_0[0][2] + grip_aabb_0[1][2])
        grip90_y = 0.5 * (grip_aabb_90[0][1] + grip_aabb_90[1][1])
        grip90_z = 0.5 * (grip_aabb_90[0][2] + grip_aabb_90[1][2])
    else:
        grip0_y = grip0_z = grip90_y = grip90_z = None
    ctx.check(
        "height crank handle orbits around its short shaft",
        grip0_y is not None
        and grip90_y is not None
        and abs(grip90_y - grip0_y) > 0.025
        and abs(grip90_z - grip0_z) > 0.025,
        details=f"q0=({grip0_y}, {grip0_z}), q90=({grip90_y}, {grip90_z})",
    )

    return ctx.report()


object_model = build_object_model()
