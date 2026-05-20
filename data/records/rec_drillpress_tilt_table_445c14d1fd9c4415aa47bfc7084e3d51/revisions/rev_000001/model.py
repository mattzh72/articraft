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
    MotionProperties,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _rod_rpy_for_yz(dy: float, dz: float) -> tuple[float, float, float]:
    """Rotate a local-Z cylinder so it lies along a vector in the YZ plane."""
    return (math.atan2(-dy, dz), 0.0, 0.0)


def _table_plate_mesh():
    """Small drill-press table plate with rounded corners and through slots."""
    plate = cq.Workplane("XY").box(0.220, 0.160, 0.018)
    plate = plate.edges("|Z").fillet(0.010)

    # Through slots in the cast table.  The central one gives the drill a
    # visible clearance opening; the side slots read as clamping slots.
    for x, w, h in ((0.0, 0.022, 0.070), (-0.062, 0.014, 0.082), (0.062, 0.014, 0.082)):
        cutter = (
            cq.Workplane("XY")
            .center(x, 0.0)
            .rect(w, h)
            .extrude(0.050)
            .translate((0.0, 0.0, -0.025))
        )
        plate = plate.cut(cutter)
    return plate


def _head_guide_ring_mesh():
    """Hollow quill guide collar fixed to the underside of the head."""
    return (
        cq.Workplane("XY")
        .circle(0.027)
        .circle(0.016)
        .extrude(0.035)
        .translate((0.0, 0.0, -0.0175))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sensitive_bench_drill_press")

    cast = model.material("dark_cast_iron", rgba=(0.075, 0.088, 0.100, 1.0))
    table_cast = model.material("oiled_table_casting", rgba=(0.105, 0.115, 0.120, 1.0))
    column_steel = model.material("brushed_column_steel", rgba=(0.55, 0.57, 0.58, 1.0))
    machined = model.material("machined_steel", rgba=(0.72, 0.73, 0.70, 1.0))
    black = model.material("blackened_controls", rgba=(0.020, 0.020, 0.025, 1.0))
    bit_steel = model.material("dark_tool_steel", rgba=(0.18, 0.19, 0.20, 1.0))

    # Static bench drill press frame: heavy foot, slim column, compact head and
    # a rear motor silhouette.
    frame = model.part("frame")
    frame.visual(
        Box((0.360, 0.280, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=cast,
        name="base",
    )
    frame.visual(
        Box((0.270, 0.190, 0.009)),
        origin=Origin(xyz=(0.0, 0.030, 0.0495)),
        material=Material("worn_table_slots", rgba=(0.030, 0.034, 0.038, 1.0)),
        name="base_raised_pad",
    )
    frame.visual(
        Cylinder(radius=0.022, length=0.690),
        origin=Origin(xyz=(0.0, -0.090, 0.390)),
        material=column_steel,
        name="column",
    )
    frame.visual(
        Box((0.205, 0.230, 0.105)),
        origin=Origin(xyz=(0.0, 0.010, 0.680)),
        material=cast,
        name="head_body",
    )
    frame.visual(
        Box((0.135, 0.080, 0.050)),
        origin=Origin(xyz=(0.0, 0.128, 0.645)),
        material=cast,
        name="front_quill_housing",
    )
    frame.visual(
        mesh_from_cadquery(_head_guide_ring_mesh(), "quill_guide_ring"),
        origin=Origin(xyz=(0.0, 0.095, 0.6025)),
        material=cast,
        name="quill_guide_ring",
    )
    frame.visual(
        Cylinder(radius=0.055, length=0.125),
        origin=Origin(xyz=(0.0, -0.175, 0.690), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="rear_motor",
    )
    frame.visual(
        Box((0.130, 0.045, 0.070)),
        origin=Origin(xyz=(0.0, -0.110, 0.685)),
        material=cast,
        name="motor_mount",
    )
    frame.visual(
        Box((0.165, 0.120, 0.035)),
        origin=Origin(xyz=(0.0, -0.020, 0.750)),
        material=cast,
        name="belt_cover",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.030),
        origin=Origin(xyz=(0.110, 0.070, 0.642), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined,
        name="lever_side_boss",
    )

    # Sliding table carriage.  A square clamp collar leaves real clearance
    # around the round column and carries the tilt hinge forward of the column.
    table_mount = model.part("table_mount")
    table_mount.visual(
        Box((0.017, 0.078, 0.090)),
        origin=Origin(xyz=(-0.0305, 0.0, 0.0)),
        material=cast,
        name="collar_jaw_0",
    )
    table_mount.visual(
        Box((0.017, 0.078, 0.090)),
        origin=Origin(xyz=(0.0305, 0.0, 0.0)),
        material=cast,
        name="collar_jaw_1",
    )
    table_mount.visual(
        Box((0.078, 0.017, 0.090)),
        origin=Origin(xyz=(0.0, 0.0305, 0.0)),
        material=cast,
        name="front_collar_bridge",
    )
    table_mount.visual(
        Box((0.078, 0.017, 0.090)),
        origin=Origin(xyz=(0.0, -0.0305, 0.0)),
        material=cast,
        name="rear_collar_bridge",
    )
    table_mount.visual(
        Box((0.150, 0.036, 0.025)),
        origin=Origin(xyz=(0.0, 0.057, 0.0)),
        material=cast,
        name="tilt_arm",
    )
    for x in (-0.068, 0.068):
        table_mount.visual(
            Box((0.020, 0.038, 0.030)),
            origin=Origin(xyz=(x, 0.090, 0.0)),
            material=cast,
            name=f"hinge_cheek_{0 if x < 0 else 1}",
        )

    # The table is intentionally small, with a slotted cast plate riding above a
    # trunnion hinge so it can tilt for angled drilling.
    table = model.part("table")
    table.visual(
        mesh_from_cadquery(_table_plate_mesh(), "slotted_tilting_table"),
        origin=Origin(xyz=(0.0, 0.095, 0.026)),
        material=table_cast,
        name="table_plate",
    )
    table.visual(
        Cylinder(radius=0.012, length=0.116),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined,
        name="table_trunnion",
    )
    for x in (-0.034, 0.034):
        table.visual(
            Box((0.012, 0.076, 0.012)),
            origin=Origin(xyz=(x, 0.045, 0.011)),
            material=table_cast,
            name=f"underside_rib_{0 if x < 0 else 1}",
        )

    # Sensitive quill assembly driven downward by the fine feed lever.
    quill = model.part("quill")
    quill.visual(
        Cylinder(radius=0.016, length=0.092),
        origin=Origin(xyz=(0.0, 0.0, -0.046)),
        material=machined,
        name="quill_sleeve",
    )
    quill.visual(
        Cylinder(radius=0.020, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.099)),
        material=machined,
        name="spindle_nose",
    )
    quill.visual(
        Cylinder(radius=0.014, length=0.043),
        origin=Origin(xyz=(0.0, 0.0, -0.1275)),
        material=black,
        name="drill_chuck",
    )
    quill.visual(
        Cylinder(radius=0.0035, length=0.118),
        origin=Origin(xyz=(0.0, 0.0, -0.208)),
        material=bit_steel,
        name="drill_bit",
    )

    feed_lever = model.part("feed_lever")
    lever_dy = 0.058
    lever_dz = -0.142
    lever_len = math.sqrt(lever_dy * lever_dy + lever_dz * lever_dz)
    feed_lever.visual(
        Cylinder(radius=0.0045, length=lever_len),
        origin=Origin(
            xyz=(0.0, lever_dy / 2.0, lever_dz / 2.0),
            rpy=_rod_rpy_for_yz(lever_dy, lever_dz),
        ),
        material=machined,
        name="fine_touch_rod",
    )
    feed_lever.visual(
        Sphere(radius=0.017),
        origin=Origin(xyz=(0.0, lever_dy, lever_dz)),
        material=black,
        name="finger_ball",
    )
    feed_lever.visual(
        Cylinder(radius=0.016, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined,
        name="lever_hub",
    )

    model.articulation(
        "table_height",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=table_mount,
        origin=Origin(xyz=(0.0, -0.090, 0.280)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.08, lower=-0.070, upper=0.220),
        motion_properties=MotionProperties(damping=1.0, friction=1.5),
    )
    model.articulation(
        "table_tilt",
        ArticulationType.REVOLUTE,
        parent=table_mount,
        child=table,
        origin=Origin(xyz=(0.0, 0.090, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.60, lower=-0.38, upper=0.38),
        motion_properties=MotionProperties(damping=0.25, friction=0.4),
    )
    model.articulation(
        "fine_feed",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=feed_lever,
        origin=Origin(xyz=(0.132, 0.070, 0.642)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=0.0, upper=0.82),
        motion_properties=MotionProperties(damping=0.12, friction=0.08),
    )
    model.articulation(
        "quill_feed",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=quill,
        origin=Origin(xyz=(0.0, 0.095, 0.590)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.05, lower=0.0, upper=0.034),
        motion_properties=MotionProperties(damping=0.6, friction=0.2),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    mount = object_model.get_part("table_mount")
    table = object_model.get_part("table")
    quill = object_model.get_part("quill")
    table_height = object_model.get_articulation("table_height")
    table_tilt = object_model.get_articulation("table_tilt")
    fine_feed = object_model.get_articulation("fine_feed")
    quill_feed = object_model.get_articulation("quill_feed")

    ctx.expect_within(
        frame,
        mount,
        axes="xy",
        inner_elem="column",
        margin=0.050,
        name="column lies inside sliding collar footprint",
    )
    ctx.expect_overlap(
        frame,
        mount,
        axes="z",
        elem_a="column",
        elem_b="collar_jaw_0",
        min_overlap=0.080,
        name="collar remains engaged around column at rest",
    )

    rest_mount_pos = ctx.part_world_position(mount)
    with ctx.pose({table_height: 0.180}):
        raised_mount_pos = ctx.part_world_position(mount)
        ctx.expect_overlap(
            frame,
            mount,
            axes="z",
            elem_a="column",
            elem_b="collar_jaw_1",
            min_overlap=0.080,
            name="raised collar remains on the column",
        )
    ctx.check(
        "table mount slides upward on column",
        rest_mount_pos is not None
        and raised_mount_pos is not None
        and raised_mount_pos[2] > rest_mount_pos[2] + 0.150,
        details=f"rest={rest_mount_pos}, raised={raised_mount_pos}",
    )

    rest_table_aabb = ctx.part_world_aabb(table)
    with ctx.pose({table_tilt: 0.32}):
        tilted_table_aabb = ctx.part_world_aabb(table)
    ctx.check(
        "positive table tilt raises the front of the small table",
        rest_table_aabb is not None
        and tilted_table_aabb is not None
        and tilted_table_aabb[1][2] > rest_table_aabb[1][2] + 0.015,
        details=f"rest={rest_table_aabb}, tilted={tilted_table_aabb}",
    )

    rest_quill_pos = ctx.part_world_position(quill)
    with ctx.pose({quill_feed: 0.030}):
        fed_quill_pos = ctx.part_world_position(quill)
    ctx.check(
        "quill feed slides the spindle downward",
        rest_quill_pos is not None
        and fed_quill_pos is not None
        and fed_quill_pos[2] < rest_quill_pos[2] - 0.020,
        details=f"rest={rest_quill_pos}, fed={fed_quill_pos}",
    )

    rest_lever_aabb = ctx.part_world_aabb(object_model.get_part("feed_lever"))
    with ctx.pose({fine_feed: 0.70}):
        stroked_lever_aabb = ctx.part_world_aabb(object_model.get_part("feed_lever"))
    ctx.check(
        "fine touch lever has a visible rotary stroke",
        rest_lever_aabb is not None
        and stroked_lever_aabb is not None
        and abs(stroked_lever_aabb[1][1] - rest_lever_aabb[1][1]) > 0.050,
        details=f"rest={rest_lever_aabb}, stroked={stroked_lever_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
