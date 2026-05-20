from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


BASE_RADIUS = 0.125
BASE_HEIGHT = 0.036
BASE_HOUSING_RADIUS = 0.070
PLATTER_RADIUS = 0.170
PLATTER_THICKNESS = 0.024
SUPPORT_OFFSET_Y = 0.107
SUPPORT_THICKNESS = 0.024
SUPPORT_AXIS_HEIGHT = 0.102
TABLE_AXIS_Z = PLATTER_THICKNESS + SUPPORT_AXIS_HEIGHT

TABLE_SPAN_X = 0.190
TABLE_SPAN_Y = 0.120
TABLE_TOP_THICKNESS = 0.014
TABLE_TOP_Z = 0.018
TABLE_BRIDGE_X = 0.146
TABLE_BRIDGE_Y = 0.070
TABLE_BRIDGE_Z = 0.030
TABLE_SHAFT_RADIUS = 0.016
TABLE_SHAFT_LENGTH = 0.190
TABLE_FLANGE_RADIUS = 0.028
TABLE_FLANGE_THICKNESS = 0.012
TABLE_FLANGE_CENTER_Y = 0.089


def _cylinder_y(radius: float, length: float) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length / 2.0, both=True)


def _build_base_shape() -> cq.Workplane:
    base = cq.Workplane("XY").circle(BASE_RADIUS).extrude(0.016)
    base = base.union(cq.Workplane("XY").circle(BASE_HOUSING_RADIUS).extrude(BASE_HEIGHT))
    for x_pos, y_pos in ((0.080, 0.080), (0.080, -0.080), (-0.080, 0.080), (-0.080, -0.080)):
        foot = (
            cq.Workplane("XY")
            .center(x_pos, y_pos)
            .circle(0.018)
            .extrude(0.006)
        )
        base = base.union(foot)

    base = base.faces(">Z").workplane(centerOption="CenterOfMass").circle(0.040).cutBlind(-0.004)
    base = base.edges(">Z").chamfer(0.002)
    return base


def _build_platter_shape() -> cq.Workplane:
    platter = cq.Workplane("XY").circle(PLATTER_RADIUS).extrude(PLATTER_THICKNESS)
    platter = platter.edges(">Z").chamfer(0.0015)
    platter = platter.faces(">Z").workplane(centerOption="CenterOfMass").circle(0.080).cutBlind(-0.004)

    slot_cutter = (
        cq.Workplane("XY")
        .pushPoints([(0.0, -0.055), (0.0, 0.0), (0.0, 0.055)])
        .slot2D(0.140, 0.010, angle=0.0)
        .extrude(0.006)
        .translate((0.0, 0.0, PLATTER_THICKNESS - 0.006))
    )
    platter = platter.cut(slot_cutter)

    bolt_pattern = (
        cq.Workplane("XY")
        .polarArray(0.118, 0.0, 360.0, 8)
        .circle(0.005)
        .extrude(PLATTER_THICKNESS + 0.002)
    )
    platter = platter.cut(bolt_pattern.translate((0.0, 0.0, -0.001)))
    return platter


def _build_support_shape() -> cq.Workplane:
    outer = [
        (-0.046, 0.000),
        (0.046, 0.000),
        (0.046, 0.018),
        (0.031, 0.018),
        (0.031, 0.052),
        (0.019, SUPPORT_AXIS_HEIGHT - 0.006),
        (0.000, SUPPORT_AXIS_HEIGHT + 0.010),
        (-0.019, SUPPORT_AXIS_HEIGHT - 0.006),
        (-0.031, 0.052),
        (-0.031, 0.018),
        (-0.046, 0.018),
    ]
    support = cq.Workplane("XZ").polyline(outer).close().extrude(SUPPORT_THICKNESS / 2.0, both=True)

    boss = cq.Workplane("XZ").center(0.0, SUPPORT_AXIS_HEIGHT).circle(0.032).extrude(SUPPORT_THICKNESS / 2.0, both=True)
    support = support.union(boss)

    window = (
        cq.Workplane("XZ")
        .polyline([(-0.017, 0.022), (0.017, 0.022), (0.008, 0.055), (-0.008, 0.055)])
        .close()
        .extrude((SUPPORT_THICKNESS + 0.004) / 2.0, both=True)
    )
    support = support.cut(window)
    support = support.edges("|Y").fillet(0.0025)
    return support


def _build_table_shape() -> cq.Workplane:
    top_plate = (
        cq.Workplane("XY")
        .box(TABLE_SPAN_X, TABLE_SPAN_Y, TABLE_TOP_THICKNESS, centered=(True, True, False))
        .translate((0.0, 0.0, TABLE_TOP_Z))
        .edges("|Z")
        .fillet(0.004)
    )
    bridge = cq.Workplane("XY").box(TABLE_BRIDGE_X, TABLE_BRIDGE_Y, TABLE_BRIDGE_Z, centered=(True, True, False))

    shaft = _cylinder_y(TABLE_SHAFT_RADIUS, TABLE_SHAFT_LENGTH)
    left_flange = _cylinder_y(TABLE_FLANGE_RADIUS, TABLE_FLANGE_THICKNESS).translate((0.0, -TABLE_FLANGE_CENTER_Y, 0.0))
    right_flange = _cylinder_y(TABLE_FLANGE_RADIUS, TABLE_FLANGE_THICKNESS).translate((0.0, TABLE_FLANGE_CENTER_Y, 0.0))
    left_web = (
        cq.Workplane("XY")
        .box(0.030, 0.030, 0.022, centered=(True, True, False))
        .translate((0.0, -0.060, 0.006))
    )
    right_web = (
        cq.Workplane("XY")
        .box(0.030, 0.030, 0.022, centered=(True, True, False))
        .translate((0.0, 0.060, 0.006))
    )

    table = top_plate.union(bridge).union(shaft).union(left_flange).union(right_flange).union(left_web).union(right_web)

    for y_pos in (-0.032, 0.0, 0.032):
        groove = (
            cq.Workplane("XY")
            .box(0.160, 0.010, 0.006, centered=(True, True, False))
            .translate((0.0, y_pos, TABLE_TOP_Z + TABLE_TOP_THICKNESS - 0.006))
        )
        table = table.cut(groove)

    for x_pos in (-0.060, 0.060):
        hole = (
            cq.Workplane("XY")
            .center(x_pos, 0.0)
            .circle(0.0055)
            .extrude(TABLE_TOP_Z + TABLE_TOP_THICKNESS + 0.002)
            .translate((0.0, 0.0, -0.001))
        )
        table = table.cut(hole)

    return table


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="trunnion_workholding_stage")

    model.material("machine_dark", rgba=(0.20, 0.22, 0.24, 1.0))
    model.material("machined_steel", rgba=(0.63, 0.66, 0.70, 1.0))
    model.material("fixture_gray", rgba=(0.50, 0.53, 0.57, 1.0))

    base = model.part("base")
    base.visual(mesh_from_cadquery(_build_base_shape(), "base_body"), material="machine_dark", name="base_body")
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=BASE_RADIUS, length=BASE_HEIGHT),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT / 2.0)),
    )

    platter = model.part("platter")
    platter.visual(mesh_from_cadquery(_build_platter_shape(), "lower_platter"), material="machined_steel", name="lower_platter")
    platter.inertial = Inertial.from_geometry(
        Cylinder(radius=PLATTER_RADIUS, length=PLATTER_THICKNESS),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, PLATTER_THICKNESS / 2.0)),
    )

    left_support = model.part("left_support")
    left_support.visual(mesh_from_cadquery(_build_support_shape(), "left_support"), material="fixture_gray", name="left_support_body")
    left_support.inertial = Inertial.from_geometry(
        Box((0.092, SUPPORT_THICKNESS, 0.088)),
        mass=0.75,
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
    )

    right_support = model.part("right_support")
    right_support.visual(mesh_from_cadquery(_build_support_shape(), "right_support"), material="fixture_gray", name="right_support_body")
    right_support.inertial = Inertial.from_geometry(
        Box((0.092, SUPPORT_THICKNESS, 0.088)),
        mass=0.75,
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
    )

    work_table = model.part("work_table")
    work_table.visual(mesh_from_cadquery(_build_table_shape(), "work_table"), material="machined_steel", name="work_table_body")
    work_table.inertial = Inertial.from_geometry(
        Box((TABLE_SPAN_X, 0.190, 0.054)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
    )

    model.articulation(
        "base_to_platter",
        ArticulationType.REVOLUTE,
        parent=base,
        child=platter,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.6, lower=-pi, upper=pi),
    )
    model.articulation(
        "platter_to_left_support",
        ArticulationType.FIXED,
        parent=platter,
        child=left_support,
        origin=Origin(xyz=(0.0, -SUPPORT_OFFSET_Y, PLATTER_THICKNESS)),
    )
    model.articulation(
        "platter_to_right_support",
        ArticulationType.FIXED,
        parent=platter,
        child=right_support,
        origin=Origin(xyz=(0.0, SUPPORT_OFFSET_Y, PLATTER_THICKNESS)),
    )
    model.articulation(
        "platter_to_table",
        ArticulationType.REVOLUTE,
        parent=platter,
        child=work_table,
        origin=Origin(xyz=(0.0, 0.0, TABLE_AXIS_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=28.0, velocity=1.4, lower=-1.65, upper=1.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    platter = object_model.get_part("platter")
    left_support = object_model.get_part("left_support")
    right_support = object_model.get_part("right_support")
    work_table = object_model.get_part("work_table")

    platter_joint = object_model.get_articulation("base_to_platter")
    table_joint = object_model.get_articulation("platter_to_table")

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
        "parts_present",
        all(part is not None for part in (base, platter, left_support, right_support, work_table)),
        "Expected base, platter, both side supports, and work table parts.",
    )
    ctx.check(
        "platter_joint_axis_vertical",
        tuple(round(v, 6) for v in platter_joint.axis) == (0.0, 0.0, 1.0),
        f"Expected base_to_platter axis to be vertical, got {platter_joint.axis!r}.",
    )
    ctx.check(
        "table_joint_axis_horizontal",
        tuple(round(v, 6) for v in table_joint.axis) == (0.0, 1.0, 0.0),
        f"Expected platter_to_table axis to be horizontal, got {table_joint.axis!r}.",
    )

    ctx.expect_contact(platter, base, name="platter_bearing_contact")
    ctx.expect_overlap(platter, base, axes="xy", min_overlap=0.12, name="platter_over_base_footprint")
    ctx.expect_contact(left_support, platter, name="left_support_mounted_to_platter")
    ctx.expect_contact(right_support, platter, name="right_support_mounted_to_platter")
    ctx.expect_contact(work_table, left_support, name="left_trunnion_hub_supported")
    ctx.expect_contact(work_table, right_support, name="right_trunnion_hub_supported")
    ctx.expect_gap(work_table, platter, axis="z", min_gap=0.045, name="table_clears_platter")

    rest_aabb = ctx.part_world_aabb(work_table)
    if rest_aabb is not None:
        rest_z_span = rest_aabb[1][2] - rest_aabb[0][2]
    else:
        rest_z_span = None

    with ctx.pose({platter_joint: pi / 2.0}):
        left_pos = ctx.part_world_position(left_support)
        ctx.check(
            "platter_yaw_carries_supports",
            left_pos is not None
            and abs(left_pos[0] - SUPPORT_OFFSET_Y) <= 0.004
            and abs(left_pos[1]) <= 0.004,
            f"Expected left support to swing onto +X at 90 deg yaw, got {left_pos!r}.",
        )

    with ctx.pose({table_joint: 1.0}):
        ctx.expect_contact(work_table, left_support, name="left_hub_contact_tilted")
        ctx.expect_contact(work_table, right_support, name="right_hub_contact_tilted")
        ctx.expect_gap(work_table, platter, axis="z", min_gap=0.020, name="tilted_table_clears_platter")
        tilted_aabb = ctx.part_world_aabb(work_table)
        tilted_z_span = None if tilted_aabb is None else tilted_aabb[1][2] - tilted_aabb[0][2]
        ctx.check(
            "table_tilt_changes_vertical_envelope",
            rest_z_span is not None and tilted_z_span is not None and tilted_z_span > rest_z_span + 0.060,
            f"Expected tilted work table to show a taller Z envelope than rest; rest={rest_z_span!r}, tilted={tilted_z_span!r}.",
        )

    with ctx.pose({platter_joint: 0.7, table_joint: 0.9}):
        ctx.fail_if_parts_overlap_in_current_pose(name="posed_mechanism_overlap_check")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
