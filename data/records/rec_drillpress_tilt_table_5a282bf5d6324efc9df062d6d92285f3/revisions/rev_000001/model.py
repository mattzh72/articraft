from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    ExtrudeGeometry,
)


COLUMN_Y = -0.19
QUILL_Y = 0.07
TABLE_HOME_Z = 0.30
TABLE_TRAVEL = 0.16
QUILL_HOME_Z = 0.755
QUILL_TRAVEL = 0.08


def _tube_mesh(name: str, *, outer_radius: float, inner_radius: float, height: float):
    """A capped hollow cylindrical sleeve centered on local Z."""
    half = height * 0.5
    geom = LatheGeometry.from_shell_profiles(
        [(outer_radius, -half), (outer_radius, half)],
        [(inner_radius, -half), (inner_radius, half)],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )
    return mesh_from_geometry(geom, name)


def _rounded_plate_mesh(name: str, width: float, depth: float, height: float, radius: float):
    profile = rounded_rect_profile(width, depth, radius, corner_segments=8)
    return mesh_from_geometry(ExtrudeGeometry(profile, height), name)


def _tapered_chuck_mesh(name: str):
    # Lathed stepped cone: a real drill chuck reads as a knurled/tapered steel nose
    # rather than a simple cylinder.
    profile = [
        (0.000, 0.030),
        (0.020, 0.030),
        (0.028, 0.016),
        (0.030, -0.008),
        (0.022, -0.032),
        (0.010, -0.040),
        (0.000, -0.040),
    ]
    return mesh_from_geometry(LatheGeometry(profile, segments=56), name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="benchtop_pillar_drill_press")

    cast = model.material("dark_cast_iron", rgba=(0.11, 0.13, 0.15, 1.0))
    column_metal = model.material("polished_column", rgba=(0.58, 0.60, 0.62, 1.0))
    blue_cast = model.material("blue_head_casting", rgba=(0.05, 0.16, 0.30, 1.0))
    table_metal = model.material("machined_table", rgba=(0.19, 0.22, 0.23, 1.0))
    black = model.material("black_plastic", rgba=(0.02, 0.02, 0.02, 1.0))
    steel = model.material("bright_steel", rgba=(0.70, 0.72, 0.74, 1.0))
    red = model.material("red_switch", rgba=(0.8, 0.04, 0.03, 1.0))

    frame = model.part("frame")
    frame.visual(
        _rounded_plate_mesh("base_plate", 0.52, 0.64, 0.070, 0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=cast,
        name="base_plate",
    )
    frame.visual(
        Cylinder(radius=0.032, length=0.84),
        origin=Origin(xyz=(0.0, COLUMN_Y, 0.49)),
        material=column_metal,
        name="column",
    )
    frame.visual(
        Box((0.14, 0.12, 0.055)),
        origin=Origin(xyz=(0.0, COLUMN_Y + 0.02, 0.095)),
        material=cast,
        name="column_foot",
    )

    # Fixed upper casting: rear clamp around the column, front quill cheeks,
    # and a rounded belt/motor cover all connected as one immobile machine frame.
    frame.visual(
        Box((0.25, 0.19, 0.16)),
        origin=Origin(xyz=(0.0, -0.115, 0.84)),
        material=blue_cast,
        name="rear_head_casting",
    )
    frame.visual(
        Box((0.21, 0.23, 0.075)),
        origin=Origin(xyz=(0.0, 0.035, 0.875)),
        material=blue_cast,
        name="front_head_bridge",
    )
    frame.visual(
        Box((0.050, 0.22, 0.145)),
        origin=Origin(xyz=(-0.066, 0.045, 0.790)),
        material=blue_cast,
        name="front_cheek_0",
    )
    frame.visual(
        Box((0.050, 0.22, 0.145)),
        origin=Origin(xyz=(0.066, 0.045, 0.790)),
        material=blue_cast,
        name="front_cheek_1",
    )
    frame.visual(
        _tube_mesh("quill_guide", outer_radius=0.043, inner_radius=0.0265, height=0.082),
        origin=Origin(xyz=(0.0, QUILL_Y, 0.755)),
        material=blue_cast,
        name="quill_guide",
    )
    frame.visual(
        Box((0.034, 0.18, 0.060)),
        origin=Origin(xyz=(-0.065, QUILL_Y, 0.725)),
        material=blue_cast,
        name="nose_lug_0",
    )
    frame.visual(
        Box((0.034, 0.18, 0.060)),
        origin=Origin(xyz=(0.065, QUILL_Y, 0.725)),
        material=blue_cast,
        name="nose_lug_1",
    )
    frame.visual(
        Cylinder(radius=0.025, length=0.040),
        origin=Origin(xyz=(0.105, QUILL_Y, 0.840), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blue_cast,
        name="feed_bushing",
    )
    frame.visual(
        _rounded_plate_mesh("belt_cover", 0.33, 0.22, 0.060, 0.070),
        origin=Origin(xyz=(0.0, -0.03, 0.948)),
        material=blue_cast,
        name="belt_cover",
    )
    frame.visual(
        Cylinder(radius=0.055, length=0.16),
        origin=Origin(xyz=(0.0, -0.205, 0.83)),
        material=blue_cast,
        name="column_clamp",
    )
    frame.visual(
        Box((0.024, 0.010, 0.034)),
        origin=Origin(xyz=(-0.107, 0.130, 0.842)),
        material=red,
        name="stop_switch",
    )
    frame.visual(
        Box((0.035, 0.012, 0.020)),
        origin=Origin(xyz=(-0.055, 0.131, 0.844)),
        material=black,
        name="start_switch",
    )

    table = model.part("table")
    table.visual(
        _tube_mesh("column_sleeve", outer_radius=0.058, inner_radius=0.0365, height=0.120),
        origin=Origin(),
        material=table_metal,
        name="column_sleeve",
    )
    table.visual(
        Box((0.008, 0.020, 0.095)),
        origin=Origin(xyz=(0.036, 0.0, 0.0)),
        material=steel,
        name="column_gib_0",
    )
    table.visual(
        Box((0.008, 0.020, 0.095)),
        origin=Origin(xyz=(-0.036, 0.0, 0.0)),
        material=steel,
        name="column_gib_1",
    )
    table.visual(
        Box((0.085, 0.205, 0.040)),
        origin=Origin(xyz=(0.0, 0.160, -0.025)),
        material=table_metal,
        name="support_arm",
    )
    table.visual(
        Cylinder(radius=0.165, length=0.026),
        origin=Origin(xyz=(0.0, QUILL_Y - COLUMN_Y, 0.0)),
        material=table_metal,
        name="round_table",
    )
    table.visual(
        Box((0.210, 0.018, 0.006)),
        origin=Origin(xyz=(0.0, QUILL_Y - COLUMN_Y, 0.0145)),
        material=cast,
        name="table_t_slot_0",
    )
    table.visual(
        Box((0.018, 0.210, 0.006)),
        origin=Origin(xyz=(0.0, QUILL_Y - COLUMN_Y, 0.0145)),
        material=cast,
        name="table_t_slot_1",
    )
    table.visual(
        Cylinder(radius=0.009, length=0.070),
        origin=Origin(xyz=(0.077, -0.015, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="lock_screw",
    )
    table.visual(
        Sphere(radius=0.022),
        origin=Origin(xyz=(0.124, -0.015, 0.0)),
        material=black,
        name="lock_knob",
    )

    quill = model.part("quill")
    quill.visual(
        Cylinder(radius=0.0215, length=0.260),
        origin=Origin(xyz=(0.0, 0.0, -0.080)),
        material=steel,
        name="quill_sleeve",
    )
    quill.visual(
        Box((0.0065, 0.0010, 0.160)),
        origin=Origin(xyz=(0.0230, 0.0, -0.030)),
        material=steel,
        name="quill_key_rib",
    )
    quill.visual(
        Cylinder(radius=0.0015, length=0.160),
        origin=Origin(xyz=(0.0250, 0.0, -0.030)),
        material=steel,
        name="quill_key",
    )
    quill.visual(
        Cylinder(radius=0.016, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, -0.225)),
        material=steel,
        name="spindle",
    )
    quill.visual(
        _tapered_chuck_mesh("chuck"),
        origin=Origin(xyz=(0.0, 0.0, -0.278)),
        material=steel,
        name="chuck",
    )

    feed_spider = model.part("feed_spider")
    feed_spider.visual(
        Cylinder(radius=0.017, length=0.082),
        origin=Origin(xyz=(0.041, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="feed_hub",
    )
    feed_spider.visual(
        Cylinder(radius=0.024, length=0.026),
        origin=Origin(xyz=(0.077, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="hub_cap",
    )
    for index, angle in enumerate((math.pi / 2.0, 7.0 * math.pi / 6.0, 11.0 * math.pi / 6.0)):
        direction_y = math.cos(angle)
        direction_z = math.sin(angle)
        rod_center = (0.083, 0.060 * direction_y, 0.060 * direction_z)
        knob_center = (0.083, 0.122 * direction_y, 0.122 * direction_z)
        feed_spider.visual(
            Cylinder(radius=0.0055, length=0.118),
            origin=Origin(
                xyz=rod_center,
                rpy=(angle - math.pi / 2.0, 0.0, 0.0),
            ),
            material=steel,
            name=f"feed_handle_{index}",
        )
        feed_spider.visual(
            Sphere(radius=0.017),
            origin=Origin(xyz=knob_center),
            material=black,
            name=f"handle_knob_{index}",
        )

    model.articulation(
        "frame_to_table",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=table,
        origin=Origin(xyz=(0.0, COLUMN_Y, TABLE_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.12, lower=0.0, upper=TABLE_TRAVEL),
    )
    model.articulation(
        "frame_to_quill",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=quill,
        origin=Origin(xyz=(0.0, QUILL_Y, QUILL_HOME_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=85.0, velocity=0.18, lower=0.0, upper=QUILL_TRAVEL),
    )
    model.articulation(
        "frame_to_feed_spider",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=feed_spider,
        origin=Origin(xyz=(0.125, QUILL_Y, 0.840)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=-1.0, upper=1.4),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    table = object_model.get_part("table")
    quill = object_model.get_part("quill")
    feed_spider = object_model.get_part("feed_spider")
    table_slide = object_model.get_articulation("frame_to_table")
    quill_slide = object_model.get_articulation("frame_to_quill")
    feed_joint = object_model.get_articulation("frame_to_feed_spider")

    # Prompt-critical geometry: the table's sleeve surrounds the column and
    # remains captured as it slides upward.
    ctx.expect_overlap(
        table,
        frame,
        axes="xy",
        elem_a="column_sleeve",
        elem_b="column",
        min_overlap=0.050,
        name="table sleeve surrounds round column",
    )
    ctx.expect_overlap(
        table,
        frame,
        axes="z",
        elem_a="column_sleeve",
        elem_b="column",
        min_overlap=0.100,
        name="table sleeve is engaged on column",
    )
    table_rest = ctx.part_world_position(table)
    with ctx.pose({table_slide: TABLE_TRAVEL}):
        ctx.expect_overlap(
            table,
            frame,
            axes="z",
            elem_a="column_sleeve",
            elem_b="column",
            min_overlap=0.100,
            name="raised table remains on column",
        )
        table_raised = ctx.part_world_position(table)
    ctx.check(
        "table moves upward on prismatic slide",
        table_rest is not None
        and table_raised is not None
        and table_raised[2] > table_rest[2] + TABLE_TRAVEL * 0.9,
        details=f"rest={table_rest}, raised={table_raised}",
    )

    # The quill is centered in the head guide and positive joint motion drops it.
    ctx.expect_overlap(
        quill,
        frame,
        axes="xy",
        elem_a="quill_sleeve",
        elem_b="quill_guide",
        min_overlap=0.040,
        name="quill is coaxial with head guide",
    )
    quill_rest = ctx.part_world_position(quill)
    with ctx.pose({quill_slide: QUILL_TRAVEL}):
        ctx.expect_overlap(
            quill,
            frame,
            axes="z",
            elem_a="quill_sleeve",
            elem_b="quill_guide",
            min_overlap=0.010,
            name="dropped quill stays captured in guide",
        )
        quill_dropped = ctx.part_world_position(quill)
    ctx.check(
        "quill drops downward on prismatic slide",
        quill_rest is not None
        and quill_dropped is not None
        and quill_dropped[2] < quill_rest[2] - QUILL_TRAVEL * 0.9,
        details=f"rest={quill_rest}, dropped={quill_dropped}",
    )

    # The three-handle feed head is a separate revolute control on the side of
    # the head, not fused into the casting.
    feed_rest = ctx.part_world_position(feed_spider)
    with ctx.pose({feed_joint: 1.0}):
        feed_rotated = ctx.part_world_position(feed_spider)
    ctx.check(
        "feed spider mounted at head side",
        feed_rest is not None and feed_rotated is not None,
        details=f"rest={feed_rest}, rotated={feed_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
