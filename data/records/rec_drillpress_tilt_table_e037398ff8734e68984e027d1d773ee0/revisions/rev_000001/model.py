from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BASE_TOP_Z = 0.103
COLUMN_RADIUS = 0.040
COLUMN_LENGTH = 1.350
HEAD_MOUNT_Z = 1.220
CARRIAGE_REST_Z = 0.650
CARRIAGE_TRAVEL = 0.180
TABLE_TILT_LIMIT = math.radians(50.0)
INSERT_COVER_LIMIT = math.radians(105.0)


def _base_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").rect(0.560, 0.380).extrude(0.035)
    pedestal = cq.Workplane("XY").rect(0.260, 0.180).extrude(0.050).translate((0.0, 0.0, 0.035))
    socket = cq.Workplane("XY").circle(0.074).extrude(0.018).translate((0.0, 0.0, 0.085))
    return plate.union(pedestal).union(socket)


def _carriage_shape() -> cq.Workplane:
    rear_pad = cq.Workplane("XY").box(0.170, 0.040, 0.180).translate((0.000, 0.060, 0.000))
    arm_a = cq.Workplane("XY").box(0.030, 0.140, 0.080).translate((-0.065, 0.150, -0.030))
    arm_b = cq.Workplane("XY").box(0.030, 0.140, 0.080).translate((0.065, 0.150, -0.030))
    ear_a = cq.Workplane("XY").box(0.020, 0.050, 0.080).translate((-0.060, 0.090, -0.030))
    ear_b = cq.Workplane("XY").box(0.020, 0.050, 0.080).translate((0.060, 0.090, -0.030))
    knob_boss = cq.Workplane("YZ").circle(0.018).extrude(0.022).translate((0.085, 0.070, 0.000))
    return rear_pad.union(arm_a).union(arm_b).union(ear_a).union(ear_b).union(knob_boss)


def _table_shape() -> cq.Workplane:
    slab = cq.Workplane("XY").rect(0.340, 0.350).extrude(0.030).translate((0.0, 0.175, 0.050))
    opening = cq.Workplane("XY").rect(0.094, 0.136).extrude(0.040).translate((0.0, 0.180, 0.050))
    trunnion = cq.Workplane("YZ").circle(0.023).extrude(0.050, both=True)
    rear_web = cq.Workplane("XY").box(0.050, 0.050, 0.050).translate((0.0, 0.030, 0.025))
    side_rib_a = cq.Workplane("XY").box(0.028, 0.220, 0.022).translate((-0.128, 0.185, 0.039))
    side_rib_b = cq.Workplane("XY").box(0.028, 0.220, 0.022).translate((0.128, 0.185, 0.039))
    front_rib = cq.Workplane("XY").box(0.140, 0.030, 0.016).translate((0.0, 0.315, 0.042))
    return slab.cut(opening).union(trunnion).union(rear_web).union(side_rib_a).union(side_rib_b).union(front_rib)


def _add_feed_handle(head_part, angle_deg: float, material: str, index: int) -> None:
    angle = math.radians(angle_deg)
    direction_y = math.cos(angle)
    direction_z = math.sin(angle)
    roll = angle - (math.pi / 2.0)
    rod_length = 0.200
    rod_center = (
        0.170,
        0.110 + direction_y * (rod_length * 0.5),
        0.130 + direction_z * (rod_length * 0.5),
    )
    ball_center = (
        0.170,
        0.110 + direction_y * rod_length,
        0.130 + direction_z * rod_length,
    )
    head_part.visual(
        Cylinder(radius=0.006, length=rod_length),
        origin=Origin(xyz=rod_center, rpy=(roll, 0.0, 0.0)),
        material=material,
        name=f"feed_handle_{index}",
    )
    head_part.visual(
        Sphere(radius=0.013),
        origin=Origin(xyz=ball_center),
        material=material,
        name=f"feed_ball_{index}",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="floor_drill_press")

    model.material("machine_green", rgba=(0.20, 0.36, 0.30, 1.0))
    model.material("cast_iron", rgba=(0.30, 0.32, 0.34, 1.0))
    model.material("dark_steel", rgba=(0.24, 0.25, 0.27, 1.0))
    model.material("bright_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    model.material("black_plastic", rgba=(0.10, 0.10, 0.11, 1.0))
    model.material("motor_gray", rgba=(0.42, 0.44, 0.47, 1.0))
    model.material("aluminum", rgba=(0.78, 0.79, 0.81, 1.0))

    base = model.part("base")
    base.visual(mesh_from_cadquery(_base_shape(), "drill_press_base"), material="cast_iron", name="base_casting")

    column = model.part("column")
    column.visual(
        Cylinder(radius=COLUMN_RADIUS, length=COLUMN_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, COLUMN_LENGTH * 0.5)),
        material="bright_steel",
        name="column_tube",
    )
    model.articulation(
        "base_to_column",
        ArticulationType.FIXED,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.170, 0.040, 0.180)),
        origin=Origin(xyz=(0.0, 0.060, 0.000)),
        material="cast_iron",
        name="rear_pad",
    )
    carriage.visual(
        Box((0.020, 0.050, 0.080)),
        origin=Origin(xyz=(-0.060, 0.090, -0.030)),
        material="cast_iron",
        name="ear_0",
    )
    carriage.visual(
        Box((0.020, 0.050, 0.080)),
        origin=Origin(xyz=(0.060, 0.090, -0.030)),
        material="cast_iron",
        name="ear_1",
    )
    carriage.visual(
        Cylinder(radius=0.018, length=0.022),
        origin=Origin(xyz=(0.096, 0.070, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="cast_iron",
        name="knob_boss",
    )
    model.articulation(
        "column_to_carriage",
        ArticulationType.PRISMATIC,
        parent=column,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_REST_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=700.0,
            velocity=0.120,
            lower=0.0,
            upper=CARRIAGE_TRAVEL,
        ),
    )

    table = model.part("table")
    table.visual(mesh_from_cadquery(_table_shape(), "drill_press_table"), material="cast_iron", name="table_shell")
    model.articulation(
        "carriage_to_table",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=table,
        origin=Origin(xyz=(0.0, 0.110, -0.030)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.700,
            lower=-TABLE_TILT_LIMIT,
            upper=TABLE_TILT_LIMIT,
        ),
    )

    insert_cover = model.part("insert_cover")
    insert_cover.visual(
        Cylinder(radius=0.004, length=0.078),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="dark_steel",
        name="hinge_barrel",
    )
    insert_cover.visual(
        Box((0.086, 0.128, 0.006)),
        origin=Origin(xyz=(0.0, 0.065, -0.003)),
        material="aluminum",
        name="cover_plate",
    )
    insert_cover.visual(
        Box((0.020, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, 0.121, 0.001)),
        material="aluminum",
        name="finger_tab",
    )
    model.articulation(
        "table_to_insert_cover",
        ArticulationType.REVOLUTE,
        parent=table,
        child=insert_cover,
        origin=Origin(xyz=(0.0, 0.112, 0.080)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.200,
            lower=0.0,
            upper=INSERT_COVER_LIMIT,
        ),
    )

    table_lock_knob = model.part("table_lock_knob")
    table_lock_knob.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(xyz=(0.009, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="bright_steel",
        name="knob_shaft",
    )
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.046,
            0.024,
            body_style="mushroom",
            top_diameter=0.040,
            grip=KnobGrip(style="fluted", count=12, depth=0.0016),
            center=False,
        ),
        "drill_press_table_lock_knob",
    )
    table_lock_knob.visual(
        knob_mesh,
        origin=Origin(xyz=(0.018, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="black_plastic",
        name="knob_body",
    )
    model.articulation(
        "carriage_to_table_lock_knob",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=table_lock_knob,
        origin=Origin(xyz=(0.107, 0.070, 0.000)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=6.0),
    )

    head = model.part("head")
    head.visual(
        Box((0.340, 0.200, 0.220)),
        origin=Origin(xyz=(0.0, 0.100, 0.140)),
        material="machine_green",
        name="head_body",
    )
    head.visual(
        Box((0.380, 0.220, 0.090)),
        origin=Origin(xyz=(0.0, 0.110, 0.295)),
        material="machine_green",
        name="belt_cover",
    )
    head.visual(
        Cylinder(radius=0.052, length=0.220),
        origin=Origin(xyz=(0.0, 0.190, -0.030)),
        material="dark_steel",
        name="spindle_housing",
    )
    head.visual(
        Cylinder(radius=0.034, length=0.160),
        origin=Origin(xyz=(0.0, 0.190, -0.120)),
        material="bright_steel",
        name="quill_body",
    )
    head.visual(
        Cylinder(radius=0.024, length=0.100),
        origin=Origin(xyz=(0.0, 0.190, -0.235)),
        material="dark_steel",
        name="chuck_body",
    )
    head.visual(
        Cylinder(radius=0.018, length=0.040),
        origin=Origin(xyz=(0.170, 0.110, 0.130), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="bright_steel",
        name="feed_hub",
    )
    _add_feed_handle(head, 0.0, "bright_steel", 0)
    _add_feed_handle(head, 120.0, "bright_steel", 1)
    _add_feed_handle(head, 240.0, "bright_steel", 2)
    model.articulation(
        "column_to_head",
        ArticulationType.FIXED,
        parent=column,
        child=head,
        origin=Origin(xyz=(0.0, COLUMN_RADIUS, HEAD_MOUNT_Z)),
    )

    motor = model.part("motor")
    motor.visual(
        Box((0.140, 0.024, 0.100)),
        origin=Origin(xyz=(0.0, -0.012, 0.110)),
        material="dark_steel",
        name="motor_mount",
    )
    motor.visual(
        Cylinder(radius=0.078, length=0.240),
        origin=Origin(xyz=(0.0, -0.132, 0.110), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="motor_gray",
        name="motor_body",
    )
    motor.visual(
        Cylinder(radius=0.042, length=0.020),
        origin=Origin(xyz=(0.0, -0.242, 0.110), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="dark_steel",
        name="motor_cap",
    )
    model.articulation(
        "head_to_motor",
        ArticulationType.FIXED,
        parent=head,
        child=motor,
        origin=Origin(xyz=(0.0, 0.000, 0.180)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carriage = object_model.get_part("carriage")
    table = object_model.get_part("table")
    insert_cover = object_model.get_part("insert_cover")
    table_lock_knob = object_model.get_part("table_lock_knob")

    table_lift = object_model.get_articulation("column_to_carriage")
    table_tilt = object_model.get_articulation("carriage_to_table")
    cover_hinge = object_model.get_articulation("table_to_insert_cover")
    knob_joint = object_model.get_articulation("carriage_to_table_lock_knob")

    ctx.allow_overlap(
        carriage,
        table,
        elem_a="ear_0",
        elem_b="table_shell",
        reason="The tabletop trunnion is intentionally represented as a captured hinge fit inside the carriage ear.",
    )
    ctx.allow_overlap(
        carriage,
        table,
        elem_a="ear_1",
        elem_b="table_shell",
        reason="The tabletop trunnion is intentionally represented as a captured hinge fit inside the carriage ear.",
    )

    ctx.expect_origin_gap(
        table_lock_knob,
        carriage,
        axis="x",
        min_gap=0.095,
        max_gap=0.120,
        name="table-lock knob sits off the carriage side",
    )
    ctx.expect_overlap(
        table,
        insert_cover,
        axes="xy",
        elem_a="table_shell",
        elem_b="cover_plate",
        min_overlap=0.080,
        name="insert cover stays centered in the tabletop footprint",
    )

    table_aabb = ctx.part_element_world_aabb(table, elem="table_shell")
    cover_plate_aabb = ctx.part_element_world_aabb(insert_cover, elem="cover_plate")
    flush_delta = None
    if table_aabb is not None and cover_plate_aabb is not None:
        flush_delta = abs(table_aabb[1][2] - cover_plate_aabb[1][2])
    ctx.check(
        "insert cover closes nearly flush with the tabletop",
        flush_delta is not None and flush_delta <= 0.003,
        details=f"top_surface_delta={flush_delta}",
    )

    lift_upper = 0.0
    if table_lift.motion_limits is not None and table_lift.motion_limits.upper is not None:
        lift_upper = table_lift.motion_limits.upper
    carriage_rest = ctx.part_world_position(carriage)
    with ctx.pose({table_lift: lift_upper}):
        carriage_upper = ctx.part_world_position(carriage)
    ctx.check(
        "table carriage slides upward on the column",
        carriage_rest is not None
        and carriage_upper is not None
        and carriage_upper[2] > carriage_rest[2] + 0.120,
        details=f"rest={carriage_rest}, upper={carriage_upper}",
    )

    cover_rest = ctx.part_world_position(insert_cover)
    with ctx.pose({table_tilt: math.radians(35.0)}):
        cover_tilted = ctx.part_world_position(insert_cover)
    ctx.check(
        "positive table tilt raises the table front",
        cover_rest is not None
        and cover_tilted is not None
        and cover_tilted[2] > cover_rest[2] + 0.045,
        details=f"rest={cover_rest}, tilted={cover_tilted}",
    )

    closed_cover_box = ctx.part_element_world_aabb(insert_cover, elem="cover_plate")
    with ctx.pose({cover_hinge: math.radians(85.0)}):
        open_cover_box = ctx.part_element_world_aabb(insert_cover, elem="cover_plate")
    ctx.check(
        "insert cover swings upward from the tabletop",
        closed_cover_box is not None
        and open_cover_box is not None
        and open_cover_box[1][2] > closed_cover_box[1][2] + 0.070,
        details=f"closed={closed_cover_box}, open={open_cover_box}",
    )

    limits = knob_joint.motion_limits
    ctx.check(
        "table-lock knob uses a continuous rotary joint",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS
        and limits is not None
        and limits.lower is None
        and limits.upper is None,
        details=f"type={knob_joint.articulation_type}, limits={limits}",
    )

    return ctx.report()


object_model = build_object_model()
