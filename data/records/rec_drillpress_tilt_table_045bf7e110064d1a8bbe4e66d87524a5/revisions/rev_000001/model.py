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


BASE_SIZE = (0.32, 0.22, 0.028)
COLUMN_X = -0.095
COLUMN_RADIUS = 0.031
COLUMN_HEIGHT = 0.59
COLUMN_BOTTOM_Z = 0.06
SUPPORT_REST_Z = 0.27


def _make_table_shape() -> cq.Workplane:
    top_center = (0.128, 0.0, 0.046)
    top = (
        cq.Workplane("XY")
        .circle(0.096)
        .extrude(0.007, both=True)
        .translate(top_center)
    )
    top = top.cut(
        cq.Workplane("XY").circle(0.018).extrude(0.010, both=True).translate(top_center)
    )
    top = top.cut(
        cq.Workplane("XY").box(0.030, 0.090, 0.020).translate(top_center)
    )

    trunnion = cq.Workplane("XZ").circle(0.014).extrude(0.029, both=True)
    web = cq.Workplane("XY").box(0.110, 0.032, 0.060).translate((0.055, 0.0, 0.021))
    return trunnion.union(web).union(top)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_drill_press")

    cast_iron = model.material("cast_iron", rgba=(0.28, 0.30, 0.32, 1.0))
    machine_gray = model.material("machine_gray", rgba=(0.53, 0.56, 0.60, 1.0))
    polished_steel = model.material("polished_steel", rgba=(0.78, 0.79, 0.80, 1.0))
    black_bakelite = model.material("black_bakelite", rgba=(0.10, 0.10, 0.11, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box(BASE_SIZE),
        origin=Origin(xyz=(0.0, 0.0, BASE_SIZE[2] / 2.0)),
        material=cast_iron,
        name="base_plate",
    )
    frame.visual(
        Box((0.11, 0.12, 0.060)),
        origin=Origin(xyz=(COLUMN_X, 0.0, 0.030)),
        material=cast_iron,
        name="column_pedestal",
    )
    frame.visual(
        Cylinder(radius=COLUMN_RADIUS, length=COLUMN_HEIGHT),
        origin=Origin(xyz=(COLUMN_X, 0.0, COLUMN_BOTTOM_Z + (COLUMN_HEIGHT / 2.0))),
        material=machine_gray,
        name="column",
    )
    rack_x = COLUMN_X + COLUMN_RADIUS - 0.003
    frame.visual(
        Box((0.010, 0.018, 0.42)),
        origin=Origin(xyz=(rack_x, 0.0, 0.31)),
        material=polished_steel,
        name="rack_bar",
    )
    for tooth_index in range(16):
        tooth_z = 0.13 + tooth_index * 0.022
        frame.visual(
            Box((0.006, 0.012, 0.010)),
            origin=Origin(xyz=(rack_x + 0.006, 0.0, tooth_z)),
            material=polished_steel,
            name=f"rack_tooth_{tooth_index}",
        )

    frame.visual(
        Box((0.24, 0.15, 0.11)),
        origin=Origin(xyz=(0.0, 0.0, 0.610)),
        material=cast_iron,
        name="head_housing",
    )
    frame.visual(
        Box((0.18, 0.15, 0.040)),
        origin=Origin(xyz=(0.020, 0.0, 0.685)),
        material=cast_iron,
        name="belt_cover",
    )
    frame.visual(
        Cylinder(radius=0.047, length=0.120),
        origin=Origin(xyz=(-0.135, 0.0, 0.610), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machine_gray,
        name="motor",
    )
    frame.visual(
        Cylinder(radius=0.030, length=0.120),
        origin=Origin(xyz=(0.092, 0.0, 0.505)),
        material=machine_gray,
        name="spindle_sleeve",
    )
    frame.visual(
        Cylinder(radius=0.017, length=0.060),
        origin=Origin(xyz=(0.092, 0.0, 0.415)),
        material=polished_steel,
        name="chuck",
    )
    frame.visual(
        Cylinder(radius=0.040, length=0.018),
        origin=Origin(xyz=(0.092, 0.0, 0.555)),
        material=cast_iron,
        name="head_nose",
    )

    table_support = model.part("table_support")
    table_support.visual(
        Box((0.074, 0.018, 0.085)),
        origin=Origin(xyz=(0.0, -0.040, 0.0)),
        material=cast_iron,
        name="support_side_0",
    )
    table_support.visual(
        Box((0.074, 0.018, 0.085)),
        origin=Origin(xyz=(0.0, 0.040, 0.0)),
        material=cast_iron,
        name="support_side_1",
    )
    table_support.visual(
        Box((0.030, 0.110, 0.030)),
        origin=Origin(xyz=(0.058, 0.0, 0.006)),
        material=cast_iron,
        name="support_bridge",
    )
    table_support.visual(
        Box((0.076, 0.012, 0.034)),
        origin=Origin(xyz=(0.073, -0.036, -0.018)),
        material=cast_iron,
        name="support_arm_0",
    )
    table_support.visual(
        Box((0.076, 0.012, 0.034)),
        origin=Origin(xyz=(0.073, 0.036, -0.018)),
        material=cast_iron,
        name="support_arm_1",
    )
    table_support.visual(
        Cylinder(radius=0.019, length=0.040),
        origin=Origin(xyz=(0.071, 0.0, -0.030), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machine_gray,
        name="pinion_boss",
    )
    table_support.visual(
        Box((0.038, 0.024, 0.020)),
        origin=Origin(xyz=(0.058, 0.0, -0.017)),
        material=cast_iron,
        name="pinion_mount",
    )
    table_support.visual(
        Box((0.024, 0.010, 0.052)),
        origin=Origin(xyz=(0.108, -0.034, -0.008)),
        material=cast_iron,
        name="trunnion_ear_0",
    )
    table_support.visual(
        Box((0.024, 0.010, 0.052)),
        origin=Origin(xyz=(0.108, 0.034, -0.008)),
        material=cast_iron,
        name="trunnion_ear_1",
    )

    table = model.part("table")
    table.visual(
        mesh_from_cadquery(_make_table_shape(), "table"),
        material=machine_gray,
        name="table_casting",
    )

    crank_handle = model.part("crank_handle")
    crank_handle.visual(
        Cylinder(radius=0.013, length=0.012),
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished_steel,
        name="handle_hub",
    )
    crank_handle.visual(
        Box((0.010, 0.068, 0.012)),
        origin=Origin(xyz=(0.016, 0.034, -0.018)),
        material=polished_steel,
        name="handle_arm",
    )
    crank_handle.visual(
        Box((0.014, 0.018, 0.024)),
        origin=Origin(xyz=(0.018, 0.064, -0.032)),
        material=polished_steel,
        name="grip_yoke",
    )

    crank_grip = model.part("crank_grip")
    crank_grip.visual(
        Cylinder(radius=0.008, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=black_bakelite,
        name="grip_body",
    )
    crank_grip.visual(
        Box((0.004, 0.020, 0.006)),
        origin=Origin(xyz=(0.0, 0.009, 0.0)),
        material=black_bakelite,
        name="grip_ridge",
    )

    model.articulation(
        "column_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=table_support,
        origin=Origin(xyz=(COLUMN_X, 0.0, SUPPORT_REST_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-0.06, upper=0.10, effort=120.0, velocity=0.15),
    )
    model.articulation(
        "table_tilt",
        ArticulationType.REVOLUTE,
        parent=table_support,
        child=table,
        origin=Origin(xyz=(0.108, 0.0, -0.008)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-math.radians(45.0),
            upper=math.radians(45.0),
            effort=40.0,
            velocity=1.2,
        ),
    )
    model.articulation(
        "crank_spin",
        ArticulationType.CONTINUOUS,
        parent=table_support,
        child=crank_handle,
        origin=Origin(xyz=(0.079, 0.0, -0.030)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=6.0),
    )
    model.articulation(
        "grip_spin",
        ArticulationType.CONTINUOUS,
        parent=crank_handle,
        child=crank_grip,
        origin=Origin(xyz=(0.018, 0.081, -0.032)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    def aabb_center(aabb):
        if aabb is None:
            return None
        (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
        return (
            0.5 * (min_x + max_x),
            0.5 * (min_y + max_y),
            0.5 * (min_z + max_z),
        )

    frame = object_model.get_part("frame")
    table_support = object_model.get_part("table_support")
    table = object_model.get_part("table")
    crank_handle = object_model.get_part("crank_handle")
    crank_grip = object_model.get_part("crank_grip")
    column_slide = object_model.get_articulation("column_slide")
    table_tilt = object_model.get_articulation("table_tilt")
    crank_spin = object_model.get_articulation("crank_spin")
    grip_spin = object_model.get_articulation("grip_spin")

    ctx.expect_overlap(
        table_support,
        frame,
        axes="z",
        elem_a="support_side_0",
        elem_b="column",
        min_overlap=0.08,
        name="left collar rail remains engaged on the column",
    )
    ctx.expect_overlap(
        table_support,
        frame,
        axes="z",
        elem_a="support_side_1",
        elem_b="column",
        min_overlap=0.08,
        name="right collar rail remains engaged on the column",
    )
    ctx.allow_overlap(
        crank_handle,
        table_support,
        elem_a="handle_hub",
        elem_b="pinion_boss",
        reason="The handle hub is intentionally represented as rotating within the simplified front pinion boss sleeve.",
    )
    ctx.allow_overlap(
        table_support,
        table,
        elem_a="trunnion_ear_0",
        elem_b="table_casting",
        reason="The table trunnion is simplified as a solid hub captured within the clevis ear.",
    )
    ctx.allow_overlap(
        table_support,
        table,
        elem_a="trunnion_ear_1",
        elem_b="table_casting",
        reason="The table trunnion is simplified as a solid hub captured within the clevis ear.",
    )
    ctx.expect_gap(
        frame,
        table,
        axis="z",
        positive_elem="chuck",
        negative_elem="table_casting",
        min_gap=0.05,
        max_gap=0.16,
        name="chuck clears the table in the rest pose",
    )

    rest_support_z = ctx.part_world_position(table_support)
    ctx.check(
        "table support stays centered on the column axis",
        rest_support_z is not None
        and abs(rest_support_z[0] - COLUMN_X) < 1e-6
        and abs(rest_support_z[1]) < 1e-6,
        details=f"support_origin={rest_support_z}",
    )
    with ctx.pose({column_slide: 0.05}):
        raised_support_z = ctx.part_world_position(table_support)
        ctx.expect_gap(
            frame,
            table,
            axis="z",
            positive_elem="chuck",
            negative_elem="table_casting",
            min_gap=0.005,
            max_gap=0.12,
            name="raising the table reduces spindle clearance",
        )

    ctx.check(
        "table support raises upward on the column",
        rest_support_z is not None
        and raised_support_z is not None
        and raised_support_z[2] > rest_support_z[2] + 0.04,
        details=f"rest={rest_support_z}, raised={raised_support_z}",
    )

    rest_aabb = ctx.part_world_aabb(table)
    with ctx.pose({table_tilt: math.radians(30.0)}):
        tilted_aabb = ctx.part_world_aabb(table)
    ctx.check(
        "table tilts about a horizontal trunnion",
        rest_aabb is not None
        and tilted_aabb is not None
        and tilted_aabb[0][2] < rest_aabb[0][2] - 0.08
        and tilted_aabb[0][0] < rest_aabb[0][0] - 0.003,
        details=f"rest={rest_aabb}, tilted={tilted_aabb}",
    )

    rest_arm_center = aabb_center(ctx.part_element_world_aabb(crank_handle, elem="handle_arm"))
    with ctx.pose({crank_spin: math.pi / 2.0}):
        turned_arm_center = aabb_center(ctx.part_element_world_aabb(crank_handle, elem="handle_arm"))
    ctx.check(
        "crank handle swings around the lift shaft",
        rest_arm_center is not None
        and turned_arm_center is not None
        and abs(turned_arm_center[1] - rest_arm_center[1]) > 0.012
        and abs(turned_arm_center[2] - rest_arm_center[2]) > 0.02,
        details=f"rest={rest_arm_center}, turned={turned_arm_center}",
    )

    rest_ridge_center = aabb_center(ctx.part_element_world_aabb(crank_grip, elem="grip_ridge"))
    with ctx.pose({grip_spin: math.pi / 2.0}):
        spun_ridge_center = aabb_center(ctx.part_element_world_aabb(crank_grip, elem="grip_ridge"))
    ctx.check(
        "grip spins on its local axle",
        rest_ridge_center is not None
        and spun_ridge_center is not None
        and abs(spun_ridge_center[0] - rest_ridge_center[0]) > 0.007
        and abs(spun_ridge_center[1] - rest_ridge_center[1]) > 0.007,
        details=f"rest={rest_ridge_center}, spun={spun_ridge_center}",
    )

    return ctx.report()


object_model = build_object_model()
