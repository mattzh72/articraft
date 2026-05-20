from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_WIDTH = 1.50
BASE_DEPTH = 1.10
BASE_HEIGHT = 0.22

PEDESTAL_WIDTH = 0.96
PEDESTAL_DEPTH = 0.82
PEDESTAL_HEIGHT = 0.28

SUPPORT_INNER_X = 0.46
SUPPORT_CAST_THICKNESS = 0.18
SUPPORT_DEPTH = 0.34
SUPPORT_HEIGHT = 0.62

PAD_LENGTH = 0.02
PAD_RADIUS = 0.10
PAD_CENTER_X = SUPPORT_INNER_X - (PAD_LENGTH / 2.0)

TILT_AXIS_Z = BASE_HEIGHT + PEDESTAL_HEIGHT + 0.28

JOURNAL_RADIUS = 0.075
JOURNAL_LENGTH = 0.125
JOURNAL_CENTER_X = PAD_CENTER_X - (PAD_LENGTH / 2.0) - (JOURNAL_LENGTH / 2.0)

TRUNNION_BODY_WIDTH = 0.40
TRUNNION_BODY_DEPTH = 0.30
TRUNNION_BODY_HEIGHT = 0.24
SPINDLE_NECK_WIDTH = 0.22
SPINDLE_NECK_DEPTH = 0.22
SPINDLE_NECK_HEIGHT = 0.14
SPINDLE_HOUSING_RADIUS = 0.18
SPINDLE_HOUSING_HEIGHT = 0.10
TABLE_CENTER_ABOVE_TILT = 0.0
TABLE_RADIUS = 0.31
TABLE_THICKNESS = 0.12
TABLE_BORE_RADIUS = 0.05
ROTARY_BEARING_RADIUS = 0.12
ROTARY_BEARING_THICKNESS = 0.02
ROTARY_BEARING_CENTER_Z = -(TABLE_THICKNESS / 2.0) - (ROTARY_BEARING_THICKNESS / 2.0)
ROTARY_HUB_RADIUS = 0.08
ROTARY_HUB_THICKNESS = 0.10
ROTARY_HUB_CENTER_Z = ROTARY_BEARING_CENTER_Z - (ROTARY_BEARING_THICKNESS / 2.0) - (ROTARY_HUB_THICKNESS / 2.0)
TRUNNION_LEG_X = 0.37
TRUNNION_LEG_WIDTH = 0.10
TRUNNION_LEG_DEPTH = 0.12
TRUNNION_LEG_HEIGHT = 0.22
TRUNNION_LEG_CENTER_Y = -0.12
TRUNNION_LEG_CENTER_Z = -0.17
TRUNNION_BEAM_DEPTH = 0.10
TRUNNION_BEAM_HEIGHT = 0.10
TRUNNION_BEAM_CENTER_Y = -0.16
TRUNNION_BEAM_CENTER_Z = -0.20
TRUNNION_CONNECTOR_WIDTH = 0.16
TRUNNION_CONNECTOR_DEPTH = 0.20
TRUNNION_CONNECTOR_HEIGHT = 0.08
TRUNNION_CONNECTOR_CENTER_Y = -0.10
TRUNNION_CONNECTOR_CENTER_Z = -0.15


def _make_base_casting() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .box(BASE_WIDTH, BASE_DEPTH, BASE_HEIGHT)
        .translate((0.0, 0.0, BASE_HEIGHT / 2.0))
        .edges("|Z")
        .fillet(0.03)
    )

    support_center_x = 0.60
    support_center_z = BASE_HEIGHT + 0.38

    left_frame = (
        cq.Workplane("XY")
        .box(0.28, 0.42, 0.76)
        .translate((-support_center_x, 0.0, support_center_z))
        .edges("|Z")
        .fillet(0.03)
    )
    right_frame = (
        cq.Workplane("XY")
        .box(0.28, 0.42, 0.76)
        .translate((support_center_x, 0.0, support_center_z))
        .edges("|Z")
        .fillet(0.03)
    )

    rear_bridge = (
        cq.Workplane("XY")
        .box(0.90, 0.18, 0.24)
        .translate((0.0, -0.24, BASE_HEIGHT + 0.12))
        .edges("|Z")
        .fillet(0.02)
    )

    low_saddle = (
        cq.Workplane("XY")
        .box(0.42, 0.28, 0.12)
        .translate((0.0, 0.12, BASE_HEIGHT + 0.06))
        .edges("|Z")
        .fillet(0.015)
    )

    swing_window = (
        cq.Workplane("XY")
        .box(0.78, 0.38, 0.46)
        .translate((0.0, 0.02, BASE_HEIGHT + 0.23))
    )

    casting = base.union(left_frame).union(right_frame).union(rear_bridge).union(low_saddle).cut(swing_window)
    return casting


def _make_trunnion_body() -> cq.Workplane:
    left_upright = (
        cq.Workplane("XY")
        .box(0.12, 0.18, 0.22)
        .translate((-0.38, -0.09, -0.11))
        .edges("|Z")
        .fillet(0.012)
    )
    right_upright = (
        cq.Workplane("XY")
        .box(0.12, 0.18, 0.22)
        .translate((0.38, -0.09, -0.11))
        .edges("|Z")
        .fillet(0.012)
    )
    lower_crossbeam = (
        cq.Workplane("XY")
        .box(0.76, 0.10, 0.08)
        .translate((0.0, -0.17, -0.20))
        .edges("|Z")
        .fillet(0.012)
    )
    center_bridge = (
        cq.Workplane("XY")
        .box(0.22, 0.14, 0.08)
        .translate((0.0, -0.08, -0.15))
        .edges("|Z")
        .fillet(0.01)
    )
    rotary_hub = (
        cq.Workplane("XY")
        .circle(ROTARY_HUB_RADIUS)
        .extrude(ROTARY_HUB_THICKNESS)
        .translate((0.0, 0.0, ROTARY_HUB_CENTER_Z - (ROTARY_HUB_THICKNESS / 2.0)))
    )
    bearing_face = (
        cq.Workplane("XY")
        .circle(ROTARY_BEARING_RADIUS)
        .extrude(ROTARY_BEARING_THICKNESS)
        .translate((0.0, 0.0, ROTARY_BEARING_CENTER_Z - (ROTARY_BEARING_THICKNESS / 2.0)))
    )
    left_journal = (
        cq.Workplane("XY")
        .circle(JOURNAL_RADIUS)
        .extrude(JOURNAL_LENGTH)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate((-JOURNAL_CENTER_X - (JOURNAL_LENGTH / 2.0), 0.0, 0.0))
    )
    right_journal = (
        cq.Workplane("XY")
        .circle(JOURNAL_RADIUS)
        .extrude(JOURNAL_LENGTH)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate((JOURNAL_CENTER_X - (JOURNAL_LENGTH / 2.0), 0.0, 0.0))
    )

    return (
        left_upright.union(right_upright)
        .union(lower_crossbeam)
        .union(center_bridge)
        .union(rotary_hub)
        .union(bearing_face)
        .union(left_journal)
        .union(right_journal)
    )


def _make_rotary_table() -> cq.Workplane:
    table = (
        cq.Workplane("XY")
        .circle(TABLE_RADIUS)
        .extrude(TABLE_THICKNESS)
        .translate((0.0, 0.0, -(TABLE_THICKNESS / 2.0)))
    )

    center_bore = cq.Workplane("XY").circle(TABLE_BORE_RADIUS).extrude(TABLE_THICKNESS + 0.02).translate(
        (0.0, 0.0, -(TABLE_THICKNESS / 2.0) - 0.01)
    )

    ring_groove = (
        cq.Workplane("XY")
        .circle(0.24)
        .circle(0.17)
        .extrude(0.012)
        .translate((0.0, 0.0, (TABLE_THICKNESS / 2.0) - 0.012))
    )

    slot_cutter = (
        cq.Workplane("XY")
        .box(0.034, 0.16, 0.016)
        .translate((0.0, 0.19, (TABLE_THICKNESS / 2.0) - 0.008))
    )

    table = table.cut(center_bore).cut(ring_groove)
    for angle_deg in (0.0, 90.0, 180.0, 270.0):
        table = table.cut(slot_cutter.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg))

    return table.edges("|Z").fillet(0.01)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="machine_tool_trunnion_table")

    cast_gray = model.material("cast_gray", color=(0.58, 0.60, 0.63))
    dark_cast = model.material("dark_cast", color=(0.34, 0.36, 0.39))
    machined_steel = model.material("machined_steel", color=(0.72, 0.74, 0.77))
    bearing_steel = model.material("bearing_steel", color=(0.65, 0.67, 0.70))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_casting(), "trunnion_base_casting"),
        material=cast_gray,
        name="base_casting",
    )
    base.visual(
        Cylinder(radius=PAD_RADIUS, length=PAD_LENGTH),
        origin=Origin(xyz=(-PAD_CENTER_X, 0.0, TILT_AXIS_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing_steel,
        name="left_bearing_pad",
    )
    base.visual(
        Cylinder(radius=PAD_RADIUS, length=PAD_LENGTH),
        origin=Origin(xyz=(PAD_CENTER_X, 0.0, TILT_AXIS_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing_steel,
        name="right_bearing_pad",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_WIDTH, BASE_DEPTH, BASE_HEIGHT + PEDESTAL_HEIGHT + SUPPORT_HEIGHT)),
        mass=2200.0,
        origin=Origin(xyz=(0.0, 0.0, (BASE_HEIGHT + PEDESTAL_HEIGHT + SUPPORT_HEIGHT) / 2.0)),
    )

    trunnion = model.part("trunnion_carriage")
    trunnion.visual(
        Box((0.12, 0.34, 0.26)),
        origin=Origin(xyz=(-JOURNAL_CENTER_X, -0.10, -0.07)),
        material=dark_cast,
        name="left_cheek",
    )
    trunnion.visual(
        Box((0.12, 0.34, 0.26)),
        origin=Origin(xyz=(JOURNAL_CENTER_X, -0.10, -0.07)),
        material=dark_cast,
        name="right_cheek",
    )
    trunnion.visual(
        Box((0.76, 0.10, 0.08)),
        origin=Origin(xyz=(0.0, -0.18, -0.18)),
        material=dark_cast,
        name="rear_tie_beam",
    )
    trunnion.visual(
        Box((0.14, 0.10, 0.12)),
        origin=Origin(xyz=(0.0, -0.10, -0.14)),
        material=dark_cast,
        name="hub_support",
    )
    trunnion.visual(
        Cylinder(radius=ROTARY_HUB_RADIUS, length=ROTARY_HUB_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, ROTARY_HUB_CENTER_Z)),
        material=machined_steel,
        name="rotary_hub",
    )
    trunnion.visual(
        Cylinder(radius=ROTARY_BEARING_RADIUS, length=ROTARY_BEARING_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, ROTARY_BEARING_CENTER_Z)),
        material=machined_steel,
        name="rotary_bearing_face",
    )
    trunnion.visual(
        Cylinder(radius=JOURNAL_RADIUS, length=JOURNAL_LENGTH),
        origin=Origin(xyz=(-JOURNAL_CENTER_X, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing_steel,
        name="left_journal",
    )
    trunnion.visual(
        Cylinder(radius=JOURNAL_RADIUS, length=JOURNAL_LENGTH),
        origin=Origin(xyz=(JOURNAL_CENTER_X, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing_steel,
        name="right_journal",
    )
    trunnion.inertial = Inertial.from_geometry(
        Box((2.0 * PAD_CENTER_X, 0.28, 0.26)),
        mass=680.0,
        origin=Origin(xyz=(0.0, -0.10, -0.06)),
    )

    rotary_table = model.part("rotary_table")
    rotary_table.visual(
        Cylinder(radius=TABLE_RADIUS, length=TABLE_THICKNESS),
        material=machined_steel,
        name="table_disc",
    )
    rotary_table.inertial = Inertial.from_geometry(
        Cylinder(radius=TABLE_RADIUS, length=TABLE_THICKNESS),
        mass=240.0,
    )

    model.articulation(
        "tilt_axis",
        ArticulationType.REVOLUTE,
        parent=base,
        child=trunnion,
        origin=Origin(xyz=(0.0, 0.0, TILT_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1800.0,
            velocity=0.7,
            lower=-1.2,
            upper=1.2,
        ),
    )
    model.articulation(
        "table_rotation",
        ArticulationType.REVOLUTE,
        parent=trunnion,
        child=rotary_table,
        origin=Origin(xyz=(0.0, 0.0, TABLE_CENTER_ABOVE_TILT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=2.5,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    trunnion = object_model.get_part("trunnion_carriage")
    rotary_table = object_model.get_part("rotary_table")
    tilt_axis = object_model.get_articulation("tilt_axis")
    table_rotation = object_model.get_articulation("table_rotation")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "tilt_axis_orientation",
        tuple(tilt_axis.axis) == (1.0, 0.0, 0.0),
        f"expected tilt axis (1, 0, 0), got {tilt_axis.axis}",
    )
    ctx.check(
        "table_rotation_axis_orientation",
        tuple(table_rotation.axis) == (0.0, 0.0, 1.0),
        f"expected table axis (0, 0, 1), got {table_rotation.axis}",
    )

    with ctx.pose({tilt_axis: 0.0, table_rotation: 0.0}):
        ctx.expect_contact(
            base,
            trunnion,
            elem_a="left_bearing_pad",
            name="left_trunnion_bearing_contact",
        )
        ctx.expect_contact(
            base,
            trunnion,
            elem_a="right_bearing_pad",
            name="right_trunnion_bearing_contact",
        )
        ctx.expect_contact(
            trunnion,
            rotary_table,
            elem_a="rotary_bearing_face",
            elem_b="table_disc",
            name="table_spindle_mount_contact",
        )
        ctx.expect_origin_gap(
            rotary_table,
            base,
            axis="z",
            min_gap=0.72,
            max_gap=0.84,
            name="table_center_height_above_base",
        )
        base_aabb = ctx.part_world_aabb(base)
        table_aabb = ctx.part_world_aabb(rotary_table)
        if base_aabb is not None and table_aabb is not None:
            base_width = base_aabb[1][0] - base_aabb[0][0]
            table_width = table_aabb[1][0] - table_aabb[0][0]
            ctx.check(
                "base_wider_than_rotary_table",
                base_width > (table_width * 2.0),
                f"base width {base_width:.3f} should be at least twice table width {table_width:.3f}",
            )

    tilt_limits = tilt_axis.motion_limits
    if tilt_limits is not None and tilt_limits.lower is not None and tilt_limits.upper is not None:
        for label, angle in (("lower", tilt_limits.lower), ("upper", tilt_limits.upper)):
            with ctx.pose({tilt_axis: angle, table_rotation: 0.0}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"tilt_{label}_no_overlap")
                ctx.fail_if_isolated_parts(name=f"tilt_{label}_no_floating")
                ctx.expect_contact(
                    base,
                    trunnion,
                    elem_a="left_bearing_pad",
                    name=f"tilt_{label}_left_bearing_contact",
                )
                ctx.expect_contact(
                    base,
                    trunnion,
                    elem_a="right_bearing_pad",
                    name=f"tilt_{label}_right_bearing_contact",
                )
                ctx.expect_contact(
                    trunnion,
                    rotary_table,
                    elem_a="rotary_bearing_face",
                    elem_b="table_disc",
                    name=f"tilt_{label}_table_mount_contact",
                )

    table_limits = table_rotation.motion_limits
    if table_limits is not None and table_limits.lower is not None and table_limits.upper is not None:
        for label, angle in (("lower", table_limits.lower), ("upper", table_limits.upper)):
            with ctx.pose({tilt_axis: 0.45, table_rotation: angle}):
                ctx.fail_if_isolated_parts(name=f"rotation_{label}_no_floating")
                ctx.expect_contact(
                    trunnion,
                    rotary_table,
                    elem_a="rotary_bearing_face",
                    elem_b="table_disc",
                    name=f"rotation_{label}_table_mount_contact",
                )

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
