from __future__ import annotations

import math

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_drill_press")

    cast_iron = model.material("cast_iron", rgba=(0.27, 0.29, 0.31, 1.0))
    machine_gray = model.material("machine_gray", rgba=(0.60, 0.62, 0.65, 1.0))
    steel = model.material("steel", rgba=(0.76, 0.77, 0.79, 1.0))
    black = model.material("black", rgba=(0.12, 0.12, 0.13, 1.0))
    red = model.material("red", rgba=(0.66, 0.10, 0.08, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.40, 0.26, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=cast_iron,
        name="base",
    )
    frame.visual(
        Box((0.12, 0.12, 0.075)),
        origin=Origin(xyz=(0.0, 0.0, 0.0725)),
        material=cast_iron,
        name="column_pedestal",
    )
    frame.visual(
        Cylinder(radius=0.029, length=0.58),
        origin=Origin(xyz=(0.0, 0.0, 0.325)),
        material=steel,
        name="column",
    )
    frame.visual(
        Box((0.30, 0.18, 0.19)),
        origin=Origin(xyz=(0.065, 0.0, 0.700)),
        material=machine_gray,
        name="head",
    )
    frame.visual(
        Box((0.19, 0.18, 0.10)),
        origin=Origin(xyz=(0.005, 0.0, 0.805)),
        material=machine_gray,
        name="belt_cover",
    )
    frame.visual(
        Cylinder(radius=0.055, length=0.20),
        origin=Origin(xyz=(-0.090, 0.0, 0.775), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machine_gray,
        name="motor",
    )
    frame.visual(
        Cylinder(radius=0.040, length=0.11),
        origin=Origin(xyz=(0.130, 0.0, 0.582)),
        material=machine_gray,
        name="quill_nose",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.16),
        origin=Origin(xyz=(0.130, 0.0, 0.475)),
        material=steel,
        name="chuck",
    )
    frame.visual(
        Cylinder(radius=0.022, length=0.030),
        origin=Origin(xyz=(0.118, -0.080, 0.700), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machine_gray,
        name="feed_boss",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.050, 0.140, 0.120)),
        origin=Origin(xyz=(-0.055, 0.0, 0.0)),
        material=cast_iron,
        name="rear_clamp",
    )
    carriage.visual(
        Box((0.100, 0.030, 0.120)),
        origin=Origin(xyz=(0.015, 0.048, 0.0)),
        material=cast_iron,
        name="jaw_0",
    )
    carriage.visual(
        Box((0.100, 0.030, 0.120)),
        origin=Origin(xyz=(0.015, -0.048, 0.0)),
        material=cast_iron,
        name="jaw_1",
    )
    carriage.visual(
        Box((0.240, 0.080, 0.040)),
        origin=Origin(xyz=(0.150, 0.0, -0.010)),
        material=cast_iron,
        name="support_arm",
    )
    carriage.visual(
        Box((0.080, 0.070, 0.050)),
        origin=Origin(xyz=(0.080, 0.0, -0.030)),
        material=cast_iron,
        name="support_rib",
    )
    carriage.visual(
        Cylinder(radius=0.010, length=0.050),
        origin=Origin(xyz=(-0.039, 0.0, 0.0)),
        material=machine_gray,
        name="column_pad",
    )
    carriage.visual(
        Box((0.018, 0.100, 0.070)),
        origin=Origin(xyz=(0.177, 0.0, 0.035)),
        material=cast_iron,
        name="tilt_cheek_0",
    )
    carriage.visual(
        Box((0.018, 0.100, 0.070)),
        origin=Origin(xyz=(0.263, 0.0, 0.035)),
        material=cast_iron,
        name="tilt_cheek_1",
    )
    carriage.visual(
        Box((0.050, 0.025, 0.030)),
        origin=Origin(xyz=(0.050, -0.045, -0.005)),
        material=cast_iron,
        name="gearbox_neck",
    )
    carriage.visual(
        Box((0.055, 0.050, 0.055)),
        origin=Origin(xyz=(0.045, -0.078, -0.005)),
        material=cast_iron,
        name="gearbox",
    )

    table = model.part("table")
    table.visual(
        Cylinder(radius=0.024, length=0.060),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machine_gray,
        name="trunnion",
    )
    table.visual(
        Box((0.055, 0.070, 0.056)),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=machine_gray,
        name="table_bracket",
    )
    table.visual(
        Cylinder(radius=0.115, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.062)),
        material=machine_gray,
        name="table_top",
    )

    crank = model.part("crank")
    crank.visual(
        Cylinder(radius=0.008, length=0.024),
        origin=Origin(xyz=(0.0, -0.012, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="crank_shaft",
    )
    crank.visual(
        Cylinder(radius=0.016, length=0.020),
        origin=Origin(xyz=(0.0, -0.022, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="crank_hub",
    )
    crank.visual(
        Box((0.080, 0.012, 0.012)),
        origin=Origin(xyz=(0.040, -0.022, 0.0)),
        material=steel,
        name="crank_arm",
    )
    crank.visual(
        Sphere(radius=0.014),
        origin=Origin(xyz=(0.090, -0.022, 0.0)),
        material=black,
        name="crank_grip",
    )

    quill_handle = model.part("quill_handle")
    quill_handle.visual(
        Cylinder(radius=0.008, length=0.024),
        origin=Origin(xyz=(0.0, -0.012, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="feed_shaft",
    )
    quill_handle.visual(
        Cylinder(radius=0.018, length=0.022),
        origin=Origin(xyz=(0.0, -0.022, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="feed_hub",
    )

    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        quill_handle.visual(
            Box((0.140, 0.012, 0.012)),
            origin=Origin(
                xyz=(0.080 * math.cos(angle), -0.022, 0.080 * math.sin(angle)),
                rpy=(0.0, -angle, 0.0),
            ),
            material=steel,
            name=f"spoke_{index}",
        )
        quill_handle.visual(
            Sphere(radius=0.014),
            origin=Origin(xyz=(0.150 * math.cos(angle), -0.022, 0.150 * math.sin(angle))),
            material=red if index == 0 else black,
            name=f"grip_{index}",
        )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.20,
            lower=0.0,
            upper=0.100,
        ),
    )
    model.articulation(
        "carriage_to_table",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=table,
        origin=Origin(xyz=(0.220, 0.0, 0.030)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=50.0,
            velocity=1.5,
            lower=-math.radians(45.0),
            upper=math.radians(45.0),
        ),
    )
    model.articulation(
        "carriage_to_crank",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=crank,
        origin=Origin(xyz=(0.055, -0.103, -0.005)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=6.0),
    )
    model.articulation(
        "frame_to_quill_handle",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=quill_handle,
        origin=Origin(xyz=(0.118, -0.095, 0.700)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    table = object_model.get_part("table")
    crank = object_model.get_part("crank")
    quill_handle = object_model.get_part("quill_handle")

    table_lift = object_model.get_articulation("frame_to_carriage")
    table_tilt = object_model.get_articulation("carriage_to_table")
    crank_spin = object_model.get_articulation("carriage_to_crank")
    quill_spin = object_model.get_articulation("frame_to_quill_handle")

    def aabb_center(aabb):
        if aabb is None:
            return None
        low, high = aabb
        return tuple((low[i] + high[i]) * 0.5 for i in range(3))

    def aabb_size(aabb):
        if aabb is None:
            return None
        low, high = aabb
        return tuple(high[i] - low[i] for i in range(3))

    ctx.expect_origin_distance(
        carriage,
        frame,
        axes="xy",
        max_dist=0.001,
        name="carriage stays centered on the column axis",
    )
    ctx.expect_gap(
        frame,
        table,
        axis="z",
        positive_elem="chuck",
        negative_elem="table_top",
        min_gap=0.020,
        name="table clears the chuck at rest",
    )

    rest_table_top = ctx.part_element_world_aabb(table, elem="table_top")
    rest_table_center = aabb_center(rest_table_top)

    lift_upper = table_lift.motion_limits.upper or 0.0
    with ctx.pose({table_lift: lift_upper}):
        raised_table_top = ctx.part_element_world_aabb(table, elem="table_top")
        raised_table_center = aabb_center(raised_table_top)
        ctx.expect_gap(
            frame,
            table,
            axis="z",
            positive_elem="chuck",
            negative_elem="table_top",
            min_gap=0.010,
            name="raised table still clears the chuck",
        )

    ctx.check(
        "table lift raises the work surface",
        rest_table_center is not None
        and raised_table_center is not None
        and raised_table_center[2] > rest_table_center[2] + 0.075,
        details=f"rest={rest_table_center}, raised={raised_table_center}",
    )

    rest_table_size = aabb_size(rest_table_top)
    with ctx.pose({table_tilt: math.radians(30.0)}):
        tilted_table_top = ctx.part_element_world_aabb(table, elem="table_top")
        tilted_table_size = aabb_size(tilted_table_top)

    ctx.check(
        "table tilt changes the table plane",
        rest_table_size is not None
        and tilted_table_size is not None
        and tilted_table_size[2] > rest_table_size[2] + 0.050,
        details=f"rest={rest_table_size}, tilted={tilted_table_size}",
    )

    rest_crank_grip = aabb_center(ctx.part_element_world_aabb(crank, elem="crank_grip"))
    with ctx.pose({crank_spin: math.pi / 2.0}):
        turned_crank_grip = aabb_center(ctx.part_element_world_aabb(crank, elem="crank_grip"))

    ctx.check(
        "table lift crank rotates on its shaft",
        rest_crank_grip is not None
        and turned_crank_grip is not None
        and abs(turned_crank_grip[0] - rest_crank_grip[0]) > 0.050
        and abs(turned_crank_grip[2] - rest_crank_grip[2]) > 0.050,
        details=f"rest={rest_crank_grip}, turned={turned_crank_grip}",
    )

    rest_quill_grip = aabb_center(ctx.part_element_world_aabb(quill_handle, elem="grip_0"))
    with ctx.pose({quill_spin: math.pi / 2.0}):
        turned_quill_grip = aabb_center(ctx.part_element_world_aabb(quill_handle, elem="grip_0"))

    ctx.check(
        "quill handle rotates around the feed axis",
        rest_quill_grip is not None
        and turned_quill_grip is not None
        and abs(turned_quill_grip[0] - rest_quill_grip[0]) > 0.090
        and abs(turned_quill_grip[2] - rest_quill_grip[2]) > 0.090,
        details=f"rest={rest_quill_grip}, turned={turned_quill_grip}",
    )

    return ctx.report()


object_model = build_object_model()
