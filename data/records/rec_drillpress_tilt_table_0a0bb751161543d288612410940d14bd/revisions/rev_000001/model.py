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


COLUMN_RADIUS = 0.028
COLUMN_HEIGHT = 0.56
TABLE_OFFSET_X = 0.16
TABLE_RADIUS = 0.155
TABLE_THICKNESS = 0.022
TABLE_START_Z = 0.20
TABLE_TRAVEL = 0.23


def _build_collar_shape() -> cq.Workplane:
    sleeve = cq.Workplane("XY").cylinder(0.100, 0.056, centered=(True, True, True))
    bore = cq.Workplane("XY").cylinder(0.110, COLUMN_RADIUS + 0.0025, centered=(True, True, True))
    arm = cq.Workplane("XY").box(0.160, 0.065, 0.020).translate((0.080, 0.0, -0.030))
    gusset = cq.Workplane("XY").box(0.105, 0.026, 0.070).translate((0.050, 0.0, -0.050))
    pivot_block = cq.Workplane("XY").box(0.045, 0.090, 0.030).translate((TABLE_OFFSET_X, 0.0, -0.045))
    crank_boss = cq.Workplane("XY").box(0.032, 0.032, 0.035).translate((0.012, 0.068, -0.012))
    return sleeve.union(arm).union(gusset).union(pivot_block).union(crank_boss).cut(bore)


def _build_table_shape() -> cq.Workplane:
    disk = cq.Workplane("XY").cylinder(TABLE_THICKNESS, TABLE_RADIUS, centered=(True, True, True)).translate(
        (0.0, 0.0, 0.026)
    )
    boss = cq.Workplane("XY").cylinder(0.026, 0.050, centered=(True, True, True)).translate((0.0, 0.0, 0.008))
    barrel = (
        cq.Workplane("XY")
        .cylinder(0.050, 0.012, centered=(True, True, True))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate((0.0, 0.0, 0.002))
    )
    center_hole = cq.Workplane("XY").cylinder(0.060, 0.019, centered=(True, True, True)).translate((0.0, 0.0, 0.026))
    rear_notch = (
        cq.Workplane("XY")
        .cylinder(0.060, 0.042, centered=(True, True, True))
        .translate((-TABLE_OFFSET_X + 0.003, 0.0, 0.026))
    )
    rear_slot = cq.Workplane("XY").box(0.090, 0.120, 0.060).translate((-0.125, 0.0, 0.026))
    return disk.union(boss).union(barrel).cut(center_hole).cut(rear_notch).cut(rear_slot)


def _build_belt_cover_shape() -> cq.Workplane:
    wall = 0.003
    outer = cq.Workplane("XY").box(0.220, 0.210, 0.110).translate((0.110, 0.0, 0.055))
    inner = (
        cq.Workplane("XY")
        .box(0.220 - 2.0 * wall, 0.210 - 2.0 * wall, 0.110)
        .translate((0.110, 0.0, (0.110 / 2.0) - wall))
    )
    front_lip = cq.Workplane("XY").box(0.012, 0.210, 0.028).translate((0.214, 0.0, 0.018))
    hinge_flange = cq.Workplane("XY").box(0.018, 0.180, 0.010).translate((0.009, 0.0, 0.005))
    return outer.cut(inner).union(front_lip).union(hinge_flange)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_drill_press")

    frame_paint = model.material("frame_paint", rgba=(0.23, 0.40, 0.31, 1.0))
    cast_iron = model.material("cast_iron", rgba=(0.34, 0.36, 0.38, 1.0))
    column_steel = model.material("column_steel", rgba=(0.73, 0.75, 0.78, 1.0))
    spindle_dark = model.material("spindle_dark", rgba=(0.18, 0.19, 0.20, 1.0))
    handle_black = model.material("handle_black", rgba=(0.10, 0.10, 0.11, 1.0))
    pulley_dark = model.material("pulley_dark", rgba=(0.12, 0.13, 0.14, 1.0))
    clear_guard = model.material("clear_guard", rgba=(0.78, 0.90, 0.96, 0.35))

    frame = model.part("frame")
    frame.visual(
        Box((0.440, 0.300, 0.035)),
        origin=Origin(xyz=(0.060, 0.0, -0.0175)),
        material=frame_paint,
        name="base_plate",
    )
    frame.visual(
        Cylinder(radius=0.045, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=frame_paint,
        name="column_socket",
    )
    frame.visual(
        Cylinder(radius=COLUMN_RADIUS, length=COLUMN_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, COLUMN_HEIGHT / 2.0)),
        material=column_steel,
        name="column",
    )
    frame.visual(
        Box((0.014, 0.012, 0.250)),
        origin=Origin(xyz=(COLUMN_RADIUS + 0.007, 0.066, 0.3725)),
        material=spindle_dark,
        name="rack",
    )
    frame.visual(
        Box((0.010, 0.076, 0.012)),
        origin=Origin(xyz=(COLUMN_RADIUS + 0.005, 0.033, 0.445)),
        material=spindle_dark,
        name="rack_bridge",
    )
    frame.visual(
        Box((0.120, 0.180, 0.170)),
        origin=Origin(xyz=(0.020, 0.0, 0.570)),
        material=frame_paint,
        name="head_clamp",
    )
    frame.visual(
        Box((0.260, 0.200, 0.170)),
        origin=Origin(xyz=(0.180, 0.0, 0.600)),
        material=frame_paint,
        name="head_body",
    )
    frame.visual(
        Box((0.090, 0.110, 0.090)),
        origin=Origin(xyz=(0.195, 0.0, 0.470)),
        material=frame_paint,
        name="spindle_housing",
    )
    frame.visual(
        Box((0.100, 0.120, 0.080)),
        origin=Origin(xyz=(-0.070, 0.0, 0.600)),
        material=frame_paint,
        name="motor_mount",
    )
    frame.visual(
        Cylinder(radius=0.055, length=0.180),
        origin=Origin(xyz=(-0.150, 0.0, 0.600), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_paint,
        name="motor",
    )
    frame.visual(
        Cylinder(radius=0.050, length=0.080),
        origin=Origin(xyz=(0.085, 0.0, 0.635), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pulley_dark,
        name="motor_pulley",
    )
    frame.visual(
        Cylinder(radius=0.040, length=0.080),
        origin=Origin(xyz=(0.175, 0.0, 0.635), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pulley_dark,
        name="spindle_pulley",
    )
    frame.visual(
        Cylinder(radius=0.034, length=0.120),
        origin=Origin(xyz=(0.195, 0.0, 0.410)),
        material=column_steel,
        name="quill",
    )
    frame.visual(
        Cylinder(radius=0.020, length=0.060),
        origin=Origin(xyz=(0.195, 0.0, 0.320)),
        material=spindle_dark,
        name="chuck",
    )

    collar = model.part("collar")
    collar.visual(
        Box((0.040, 0.100, 0.100)),
        origin=Origin(xyz=(-0.055, 0.0, 0.0)),
        material=cast_iron,
        name="rear_block",
    )
    collar.visual(
        Box((0.090, 0.020, 0.100)),
        origin=Origin(xyz=(0.005, 0.046, 0.0)),
        material=cast_iron,
        name="side_clamp_0",
    )
    collar.visual(
        Box((0.090, 0.020, 0.100)),
        origin=Origin(xyz=(0.005, -0.046, 0.0)),
        material=cast_iron,
        name="side_clamp_1",
    )
    collar.visual(
        Box((0.060, 0.030, 0.020)),
        origin=Origin(xyz=(0.045, 0.040, -0.030)),
        material=cast_iron,
        name="front_connector_0",
    )
    collar.visual(
        Box((0.060, 0.030, 0.020)),
        origin=Origin(xyz=(0.045, -0.040, -0.030)),
        material=cast_iron,
        name="front_connector_1",
    )
    collar.visual(
        Box((0.130, 0.050, 0.020)),
        origin=Origin(xyz=(0.090, 0.0, -0.030)),
        material=cast_iron,
        name="support_arm",
    )
    collar.visual(
        Box((0.105, 0.026, 0.070)),
        origin=Origin(xyz=(0.085, 0.0, -0.050)),
        material=cast_iron,
        name="arm_gusset",
    )
    collar.visual(
        Box((0.045, 0.090, 0.030)),
        origin=Origin(xyz=(TABLE_OFFSET_X, 0.0, -0.045)),
        material=cast_iron,
        name="pivot_block",
    )
    collar.visual(
        Box((0.024, 0.080, 0.020)),
        origin=Origin(xyz=(TABLE_OFFSET_X, 0.0, -0.020)),
        material=cast_iron,
        name="pivot_saddle",
    )
    collar.visual(
        Box((0.028, 0.026, 0.032)),
        origin=Origin(xyz=(0.012, 0.065, -0.012)),
        material=cast_iron,
        name="crank_boss",
    )

    table = model.part("table")
    table.visual(
        mesh_from_cadquery(_build_table_shape(), "drill_table"),
        material=cast_iron,
        name="table_shell",
    )

    table_crank = model.part("table_crank")
    table_crank.visual(
        Cylinder(radius=0.008, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=spindle_dark,
        name="shaft",
    )
    table_crank.visual(
        Cylinder(radius=0.017, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=spindle_dark,
        name="hub",
    )
    table_crank.visual(
        Cylinder(radius=0.005, length=0.055),
        origin=Origin(xyz=(0.028, 0.0, -0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=spindle_dark,
        name="arm",
    )
    table_crank.visual(
        Cylinder(radius=0.005, length=0.040),
        origin=Origin(xyz=(0.055, 0.0, -0.030)),
        material=spindle_dark,
        name="grip_stem",
    )
    table_crank.visual(
        Cylinder(radius=0.008, length=0.026),
        origin=Origin(xyz=(0.055, 0.0, -0.050), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=handle_black,
        name="grip",
    )

    belt_cover = model.part("belt_cover")
    belt_cover.visual(
        Box((0.220, 0.230, 0.004)),
        origin=Origin(xyz=(0.110, 0.0, 0.108)),
        material=clear_guard,
        name="top_panel",
    )
    belt_cover.visual(
        Box((0.220, 0.004, 0.106)),
        origin=Origin(xyz=(0.110, 0.113, 0.055)),
        material=clear_guard,
        name="side_panel_0",
    )
    belt_cover.visual(
        Box((0.220, 0.004, 0.106)),
        origin=Origin(xyz=(0.110, -0.113, 0.055)),
        material=clear_guard,
        name="side_panel_1",
    )
    belt_cover.visual(
        Box((0.004, 0.230, 0.046)),
        origin=Origin(xyz=(0.218, 0.0, 0.023)),
        material=clear_guard,
        name="front_panel",
    )
    belt_cover.visual(
        Box((0.008, 0.230, 0.028)),
        origin=Origin(xyz=(0.004, 0.0, 0.094)),
        material=clear_guard,
        name="rear_strip",
    )

    table_lift = model.articulation(
        "table_lift",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=collar,
        origin=Origin(xyz=(0.0, 0.0, TABLE_START_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=TABLE_TRAVEL, effort=120.0, velocity=0.25),
    )
    table_tilt = model.articulation(
        "table_tilt",
        ArticulationType.REVOLUTE,
        parent=collar,
        child=table,
        origin=Origin(xyz=(TABLE_OFFSET_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.78, upper=0.78, effort=30.0, velocity=1.2),
    )
    table_crank_spin = model.articulation(
        "table_crank_spin",
        ArticulationType.CONTINUOUS,
        parent=collar,
        child=table_crank,
        origin=Origin(xyz=(0.012, 0.093, -0.012)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=8.0),
    )
    belt_cover_hinge = model.articulation(
        "belt_cover_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=belt_cover,
        origin=Origin(xyz=(0.030, 0.0, 0.590)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.35, effort=2.0, velocity=1.5),
    )

    frame.meta["articulations"] = {
        "table_lift": table_lift.name,
        "table_tilt": table_tilt.name,
        "table_crank_spin": table_crank_spin.name,
        "belt_cover_hinge": belt_cover_hinge.name,
    }
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    frame = object_model.get_part("frame")
    collar = object_model.get_part("collar")
    table = object_model.get_part("table")
    table_crank = object_model.get_part("table_crank")
    belt_cover = object_model.get_part("belt_cover")

    table_lift = object_model.get_articulation("table_lift")
    table_tilt = object_model.get_articulation("table_tilt")
    table_crank_spin = object_model.get_articulation("table_crank_spin")
    belt_cover_hinge = object_model.get_articulation("belt_cover_hinge")

    def _center(bounds):
        if bounds is None:
            return None
        lower, upper = bounds
        return tuple((lower[index] + upper[index]) * 0.5 for index in range(3))

    def _span_z(bounds):
        if bounds is None:
            return None
        lower, upper = bounds
        return upper[2] - lower[2]

    ctx.expect_origin_distance(
        collar,
        frame,
        axes="xy",
        max_dist=0.001,
        name="collar stays centered on the column axis",
    )
    ctx.expect_origin_distance(
        table,
        collar,
        axes="x",
        min_dist=TABLE_OFFSET_X - 0.001,
        max_dist=TABLE_OFFSET_X + 0.001,
        name="table hinge sits at the end of the support arm",
    )

    rest_collar_pos = ctx.part_world_position(collar)
    with ctx.pose({table_lift: TABLE_TRAVEL}):
        raised_collar_pos = ctx.part_world_position(collar)
        ctx.expect_origin_distance(
            collar,
            frame,
            axes="xy",
            max_dist=0.001,
            name="raised collar remains coaxial with the column",
        )
    ctx.check(
        "table lift raises the support collar",
        rest_collar_pos is not None
        and raised_collar_pos is not None
        and raised_collar_pos[2] > rest_collar_pos[2] + 0.15,
        details=f"rest={rest_collar_pos}, raised={raised_collar_pos}",
    )

    rest_table_aabb = ctx.part_world_aabb(table)
    with ctx.pose({table_tilt: math.radians(30.0)}):
        tilted_table_aabb = ctx.part_world_aabb(table)
    rest_span = _span_z(rest_table_aabb)
    tilted_span = _span_z(tilted_table_aabb)
    ctx.check(
        "table tilt changes the table plane",
        rest_span is not None and tilted_span is not None and tilted_span > rest_span + 0.08,
        details=f"rest_span={rest_span}, tilted_span={tilted_span}",
    )

    rest_grip_center = _center(ctx.part_element_world_aabb(table_crank, elem="grip"))
    with ctx.pose({table_crank_spin: math.pi / 2.0}):
        turned_grip_center = _center(ctx.part_element_world_aabb(table_crank, elem="grip"))
    ctx.check(
        "table crank rotates its handle around the shaft",
        rest_grip_center is not None
        and turned_grip_center is not None
        and (
            abs(turned_grip_center[0] - rest_grip_center[0]) > 0.080
            or abs(turned_grip_center[2] - rest_grip_center[2]) > 0.025
        ),
        details=f"rest={rest_grip_center}, turned={turned_grip_center}",
    )

    rest_cover_aabb = ctx.part_world_aabb(belt_cover)
    with ctx.pose({belt_cover_hinge: math.radians(75.0)}):
        open_cover_aabb = ctx.part_world_aabb(belt_cover)
    ctx.check(
        "belt cover opens upward over the head",
        rest_cover_aabb is not None
        and open_cover_aabb is not None
        and open_cover_aabb[1][2] > rest_cover_aabb[1][2] + 0.08,
        details=f"rest={rest_cover_aabb}, open={open_cover_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
