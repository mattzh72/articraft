from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_OUTER_RADIUS = 0.23
BASE_INNER_RADIUS = 0.115
BASE_RING_HEIGHT = 0.032
BEARING_OUTER_RADIUS = 0.078
BEARING_INNER_RADIUS = 0.046
BEARING_HEIGHT = 0.054
SPOKE_WIDTH = 0.028
SPOKE_HEIGHT = 0.026
SPOKE_OVERLAP = 0.004
TABLE_SEATING_CLEARANCE = 0.001

TABLE_RADIUS = 0.125
TABLE_THICKNESS = 0.028
TABLE_EDGE_RADIUS = 0.009
SPINDLE_RADIUS = 0.038
SPINDLE_LENGTH = 0.036


def _build_ring_base() -> cq.Workplane:
    ring = (
        cq.Workplane("XY")
        .circle(BASE_OUTER_RADIUS)
        .circle(BASE_INNER_RADIUS)
        .extrude(BASE_RING_HEIGHT)
    )

    bearing = (
        cq.Workplane("XY")
        .circle(BEARING_OUTER_RADIUS)
        .circle(BEARING_INNER_RADIUS)
        .extrude(BEARING_HEIGHT)
    )

    spoke_length = BASE_INNER_RADIUS - BEARING_OUTER_RADIUS + 2.0 * SPOKE_OVERLAP
    spoke_center_x = (BEARING_OUTER_RADIUS + BASE_INNER_RADIUS) * 0.5
    spoke = (
        cq.Workplane("XY")
        .box(spoke_length, SPOKE_WIDTH, SPOKE_HEIGHT)
        .translate((spoke_center_x, 0.0, SPOKE_HEIGHT * 0.5))
    )

    spokes = spoke
    for angle_deg in (90.0, 180.0, 270.0):
        spokes = spokes.union(
            spoke.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
        )

    return ring.union(bearing).union(spokes)


def _build_center_table() -> cq.Workplane:
    platter = (
        cq.Workplane("XY")
        .circle(TABLE_RADIUS)
        .extrude(TABLE_THICKNESS)
        .faces(">Z")
        .edges()
        .fillet(TABLE_EDGE_RADIUS)
        .faces("<Z")
        .edges()
        .fillet(TABLE_EDGE_RADIUS)
    )

    spindle = (
        cq.Workplane("XY")
        .circle(SPINDLE_RADIUS)
        .extrude(-SPINDLE_LENGTH)
        .faces("<Z")
        .edges()
        .fillet(0.003)
    )

    return platter.union(spindle)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ring_base_yaw_platter")

    base_color = model.material("base_powdercoat", color=(0.18, 0.18, 0.20))
    table_color = model.material("platter_paint", color=(0.78, 0.79, 0.80))

    ring_base = model.part("ring_base")
    ring_base.visual(
        mesh_from_cadquery(_build_ring_base(), "ring_base"),
        material=base_color,
        name="base_shell",
    )

    center_table = model.part("center_table")
    center_table.visual(
        mesh_from_cadquery(_build_center_table(), "center_table"),
        material=table_color,
        name="table_shell",
    )

    model.articulation(
        "ring_base_to_center_table",
        ArticulationType.REVOLUTE,
        parent=ring_base,
        child=center_table,
        origin=Origin(xyz=(0.0, 0.0, BEARING_HEIGHT + TABLE_SEATING_CLEARANCE)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.5,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

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
    ring_base = object_model.get_part("ring_base")
    center_table = object_model.get_part("center_table")
    yaw_joint = object_model.get_articulation("ring_base_to_center_table")

    ctx.check(
        "yaw joint uses the shared vertical axis",
        yaw_joint.axis == (0.0, 0.0, 1.0),
        details=f"axis={yaw_joint.axis}",
    )

    limits = yaw_joint.motion_limits
    ctx.check(
        "yaw joint allows a full platter sweep",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower <= -3.0
        and limits.upper >= 3.0,
        details=f"limits={limits}",
    )

    with ctx.pose({yaw_joint: 0.0}):
        ctx.expect_overlap(
            center_table,
            ring_base,
            axes="xy",
            min_overlap=0.20,
            name="center table stays centered over the ring base footprint",
        )
        ctx.expect_origin_gap(
            center_table,
            ring_base,
            axis="z",
            min_gap=BEARING_HEIGHT + TABLE_SEATING_CLEARANCE - 0.001,
            max_gap=BEARING_HEIGHT + TABLE_SEATING_CLEARANCE + 0.001,
            name="center table axis sits at the bearing height",
        )
        rest_position = ctx.part_world_position(center_table)

    ctx.allow_isolated_part(
        center_table,
        reason=(
            "The rotating table is intentionally modeled with a 1 mm bearing clearance; "
            "the hidden rolling elements are omitted while the revolute joint preserves "
            "the shared-axis mount."
        ),
    )

    with ctx.pose({yaw_joint: 1.4}):
        ctx.expect_origin_distance(
            center_table,
            ring_base,
            axes="xy",
            max_dist=0.001,
            name="yaw motion keeps the table on the shared centerline",
        )
        turned_position = ctx.part_world_position(center_table)

    ctx.check(
        "yaw motion does not lift or drop the platter",
        rest_position is not None
        and turned_position is not None
        and abs(turned_position[2] - rest_position[2]) <= 1e-6,
        details=f"rest={rest_position}, turned={turned_position}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
