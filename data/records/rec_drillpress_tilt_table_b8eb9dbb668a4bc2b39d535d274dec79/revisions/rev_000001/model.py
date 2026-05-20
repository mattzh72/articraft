from __future__ import annotations

from math import pi

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


def make_carriage_mesh():
    sleeve = (
        cq.Workplane("XY")
        .circle(0.028)
        .extrude(0.095, both=True)
        .cut(cq.Workplane("XY").circle(0.015).extrude(0.100, both=True))
    )
    bridge = cq.Workplane("XY").box(0.046, 0.050, 0.072).translate((0.049, 0.0, 0.0))
    clamp_ring = (
        cq.Workplane("YZ")
        .circle(0.030)
        .extrude(0.028, both=True)
        .cut(cq.Workplane("YZ").circle(0.019).extrude(0.030, both=True))
        .translate((0.071, 0.0, 0.006))
    )
    clamp_bridge = cq.Workplane("XY").box(0.032, 0.042, 0.050).translate((0.060, 0.0, 0.004))
    return sleeve.union(bridge).union(clamp_ring).union(clamp_bridge)


def make_column_mesh():
    return cq.Workplane("XY").circle(0.015).extrude(0.170, both=True)


def make_table_bracket_mesh():
    collar = (
        cq.Workplane("XY")
        .circle(0.029)
        .extrude(0.025, both=True)
        .cut(cq.Workplane("XY").circle(0.015).extrude(0.028, both=True))
    )
    arm = cq.Workplane("XY").box(0.076, 0.048, 0.024).translate((0.049, 0.0, -0.004))
    ear_left = cq.Workplane("XY").box(0.018, 0.012, 0.036).translate((0.096, -0.021, 0.004))
    ear_right = cq.Workplane("XY").box(0.018, 0.012, 0.036).translate((0.096, 0.021, 0.004))
    boss_strut = cq.Workplane("XY").box(0.016, 0.028, 0.012).translate((0.090, 0.040, 0.002))
    lever_boss = (
        cq.Workplane("YZ")
        .circle(0.008)
        .extrude(0.007, both=True)
        .translate((0.083, 0.054, 0.002))
    )
    return collar.union(arm).union(ear_left).union(ear_right).union(boss_strut).union(lever_boss)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_drill_stand")

    cast_iron = model.material("cast_iron", rgba=(0.18, 0.18, 0.19, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.63, 0.65, 0.68, 1.0))
    black_paint = model.material("black_paint", rgba=(0.10, 0.10, 0.11, 1.0))
    table_paint = model.material("table_paint", rgba=(0.34, 0.35, 0.37, 1.0))
    lever_red = model.material("lever_red", rgba=(0.67, 0.12, 0.10, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.22, 0.15, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=cast_iron,
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=0.030, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=cast_iron,
        name="column_socket",
    )
    base.visual(
        Box((0.050, 0.060, 0.030)),
        origin=Origin(xyz=(-0.055, 0.0, 0.031)),
        material=cast_iron,
        name="rear_web",
    )

    column = model.part("column")
    column.visual(
        mesh_from_cadquery(make_column_mesh(), "column"),
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        material=satin_steel,
        name="column_tube",
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(make_carriage_mesh(), "carriage"),
        material=black_paint,
        name="carriage_body",
    )

    table_bracket = model.part("table_bracket")
    table_bracket.visual(
        mesh_from_cadquery(make_table_bracket_mesh(), "table_bracket"),
        material=black_paint,
        name="table_bracket_body",
    )

    table = model.part("table")
    table.visual(
        Cylinder(radius=0.055, length=0.010),
        origin=Origin(xyz=(0.067, 0.0, 0.016)),
        material=table_paint,
        name="table_surface",
    )
    table.visual(
        Box((0.036, 0.030, 0.022)),
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
        material=black_paint,
        name="table_rib",
    )
    table.visual(
        Box((0.012, 0.018, 0.016)),
        origin=Origin(xyz=(0.008, 0.0, 0.0)),
        material=black_paint,
        name="table_hinge_block",
    )
    table.visual(
        Cylinder(radius=0.005, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="table_trunnion",
    )

    lock_lever = model.part("lock_lever")
    lock_lever.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(0.014, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=black_paint,
        name="lever_hub",
    )
    lock_lever.visual(
        Box((0.008, 0.014, 0.050)),
        origin=Origin(xyz=(0.022, 0.0, 0.025)),
        material=lever_red,
        name="lever_handle",
    )
    lock_lever.visual(
        Cylinder(radius=0.007, length=0.020),
        origin=Origin(xyz=(0.022, 0.0, 0.055), rpy=(pi / 2.0, 0.0, 0.0)),
        material=black_paint,
        name="lever_grip",
    )

    model.articulation(
        "base_to_column",
        ArticulationType.FIXED,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
    )
    model.articulation(
        "column_to_carriage",
        ArticulationType.PRISMATIC,
        parent=column,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.190)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.20, lower=0.0, upper=0.115),
    )
    model.articulation(
        "column_to_table_bracket",
        ArticulationType.FIXED,
        parent=column,
        child=table_bracket,
        origin=Origin(xyz=(0.0, 0.0, 0.068)),
    )
    model.articulation(
        "table_bracket_to_table",
        ArticulationType.REVOLUTE,
        parent=table_bracket,
        child=table,
        origin=Origin(xyz=(0.096, 0.0, 0.004)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "table_bracket_to_lock_lever",
        ArticulationType.REVOLUTE,
        parent=table_bracket,
        child=lock_lever,
        origin=Origin(xyz=(0.083, 0.054, 0.002)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=-0.95, upper=0.70),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    column = object_model.get_part("column")
    carriage = object_model.get_part("carriage")
    table_bracket = object_model.get_part("table_bracket")
    table = object_model.get_part("table")
    lock_lever = object_model.get_part("lock_lever")
    slide = object_model.get_articulation("column_to_carriage")
    table_tilt = object_model.get_articulation("table_bracket_to_table")
    lever_joint = object_model.get_articulation("table_bracket_to_lock_lever")

    ctx.allow_overlap(
        carriage,
        column,
        elem_a="carriage_body",
        elem_b="column_tube",
        reason="The carriage is represented as a sleeve-style slide block that intentionally nests around the stand column.",
    )
    ctx.allow_overlap(
        table_bracket,
        column,
        elem_a="table_bracket_body",
        elem_b="column_tube",
        reason="The support bracket is represented as a collar clamp that intentionally nests around the stand column.",
    )
    ctx.allow_overlap(
        lock_lever,
        table_bracket,
        elem_a="lever_hub",
        elem_b="table_bracket_body",
        reason="The side lock lever is modeled as a simplified pivot hub seated in the bracket-side boss.",
    )
    ctx.allow_overlap(
        table,
        table_bracket,
        elem_a="table_trunnion",
        elem_b="table_bracket_body",
        reason="The tilting table uses a simplified trunnion proxy seated within the bracket yoke.",
    )
    ctx.expect_origin_distance(
        carriage,
        column,
        axes="xy",
        max_dist=0.001,
        name="carriage stays centered on the column",
    )
    ctx.expect_overlap(
        carriage,
        column,
        axes="z",
        min_overlap=0.080,
        name="carriage remains engaged on the column at rest",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.115}):
        ctx.expect_origin_distance(
            carriage,
            column,
            axes="xy",
            max_dist=0.001,
            name="carriage stays centered on the column when raised",
        )
        ctx.expect_overlap(
            carriage,
            column,
            axes="z",
            min_overlap=0.040,
            name="carriage keeps retained insertion at full lift",
        )
        raised_pos = ctx.part_world_position(carriage)

    ctx.check(
        "carriage raises upward",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.08,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    def visual_center(part_name: str, elem: str):
        bounds = ctx.part_element_world_aabb(part_name, elem=elem)
        if bounds is None:
            return None
        (mn_x, mn_y, mn_z), (mx_x, mx_y, mx_z) = bounds
        return ((mn_x + mx_x) * 0.5, (mn_y + mx_y) * 0.5, (mn_z + mx_z) * 0.5)

    ctx.expect_origin_distance(
        table_bracket,
        column,
        axes="xy",
        max_dist=0.001,
        name="table bracket stays concentric with the column",
    )
    ctx.expect_contact(
        table,
        table_bracket,
        elem_a="table_trunnion",
        elem_b="table_bracket_body",
        contact_tol=0.0015,
        name="table trunnion remains seated in the bracket yoke",
    )

    table_center_flat = visual_center("table", "table_surface")
    with ctx.pose({table_tilt: 0.50}):
        table_center_tilted = visual_center("table", "table_surface")
    ctx.check(
        "table tilts upward at positive travel",
        table_center_flat is not None
        and table_center_tilted is not None
        and table_center_tilted[2] > table_center_flat[2] + 0.020,
        details=f"flat={table_center_flat}, tilted={table_center_tilted}",
    )

    lever_center_rest = visual_center("lock_lever", "lever_handle")
    with ctx.pose({lever_joint: 0.60}):
        lever_center_rotated = visual_center("lock_lever", "lever_handle")
    ctx.check(
        "side lock lever rotates at the bracket",
        lever_center_rest is not None
        and lever_center_rotated is not None
        and (
            (lever_center_rotated[1] - lever_center_rest[1]) ** 2
            + (lever_center_rotated[2] - lever_center_rest[2]) ** 2
        )
        ** 0.5
        > 0.010,
        details=f"rest={lever_center_rest}, rotated={lever_center_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
