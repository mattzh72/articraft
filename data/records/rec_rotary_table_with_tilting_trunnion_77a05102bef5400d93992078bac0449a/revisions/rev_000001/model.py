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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.62
BASE_WIDTH = 0.46
BASE_PLATE_HEIGHT = 0.04
PEDESTAL_RADIUS = 0.145
PEDESTAL_HEIGHT = 0.092
BEARING_FLANGE_RADIUS = 0.17
BEARING_FLANGE_HEIGHT = 0.025

ROTARY_JOINT_Z = BASE_PLATE_HEIGHT + PEDESTAL_HEIGHT + BEARING_FLANGE_HEIGHT

ROTARY_DISC_RADIUS = 0.16
ROTARY_DISC_HEIGHT = 0.028
SADDLE_LENGTH = 0.42
SADDLE_WIDTH = 0.26
SADDLE_HEIGHT = 0.045
SADDLE_TOP_Z = ROTARY_DISC_HEIGHT + SADDLE_HEIGHT
LOW_BRACE_HEIGHT = 0.06
LOW_BRACE_WIDTH = 0.05
LOW_BRACE_Y = -0.085

CHEEK_THICKNESS = 0.035
CHEEK_SPAN_Y = 0.23
CHEEK_BASE_Z = SADDLE_TOP_Z
CHEEK_BOX_HEIGHT = 0.18
CHEEK_CAP_RADIUS = 0.078
CHEEK_INNER_X = 0.145
CHEEK_OUTER_X = CHEEK_INNER_X + CHEEK_THICKNESS
CHEEK_CENTER_X = CHEEK_INNER_X + (CHEEK_THICKNESS / 2.0)

TILT_AXIS_Z = 0.22
TABLE_RADIUS = 0.11
TABLE_THICKNESS = 0.02
TABLE_RECESS_RADIUS = 0.032
TABLE_RECESS_DEPTH = 0.004
TABLE_SLOT_LENGTH = 0.145
TABLE_SLOT_WIDTH = 0.014
TABLE_SLOT_DEPTH = 0.005
TABLE_HUB_RADIUS = 0.044
TABLE_HUB_LENGTH = 0.12
SHAFT_RADIUS = 0.018
JOURNAL_HALF_SPAN = CHEEK_INNER_X
TRUNNION_COLLAR_THICKNESS = 0.012
TRUNNION_COLLAR_RADIUS = 0.03


def _add_mesh_visual(part, shape, mesh_name: str, material: str, *, name: str) -> None:
    part.visual(mesh_from_cadquery(shape, mesh_name), material=material, name=name)


def _base_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(
        BASE_LENGTH,
        BASE_WIDTH,
        BASE_PLATE_HEIGHT,
        centered=(True, True, False),
    )
    base = base.edges("|Z").fillet(0.018)

    pedestal = (
        cq.Workplane("XY")
        .circle(PEDESTAL_RADIUS)
        .extrude(PEDESTAL_HEIGHT)
        .translate((0.0, 0.0, BASE_PLATE_HEIGHT))
    )
    flange = (
        cq.Workplane("XY")
        .circle(BEARING_FLANGE_RADIUS)
        .extrude(BEARING_FLANGE_HEIGHT)
        .translate((0.0, 0.0, BASE_PLATE_HEIGHT + PEDESTAL_HEIGHT))
    )

    return base.union(pedestal).union(flange)


def _cheek_shape(side: float) -> cq.Workplane:
    side_center_x = side * CHEEK_CENTER_X
    cheek_plate = (
        cq.Workplane("XY")
        .box(
            CHEEK_THICKNESS,
            CHEEK_SPAN_Y,
            CHEEK_BOX_HEIGHT,
            centered=(True, True, False),
        )
        .translate((side_center_x, 0.0, CHEEK_BASE_Z))
    )
    cheek_cap = (
        cq.Workplane("YZ")
        .circle(CHEEK_CAP_RADIUS)
        .extrude(CHEEK_THICKNESS)
        .translate((side_center_x - (CHEEK_THICKNESS / 2.0), 0.0, TILT_AXIS_Z))
    )
    return cheek_plate.union(cheek_cap)


def _carriage_shape() -> cq.Workplane:
    rotary_disc = cq.Workplane("XY").circle(ROTARY_DISC_RADIUS).extrude(ROTARY_DISC_HEIGHT)
    saddle = (
        cq.Workplane("XY")
        .box(
            SADDLE_LENGTH,
            SADDLE_WIDTH,
            SADDLE_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, ROTARY_DISC_HEIGHT))
    )
    low_brace = (
        cq.Workplane("XY")
        .box(
            SADDLE_LENGTH * 0.76,
            LOW_BRACE_WIDTH,
            LOW_BRACE_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, LOW_BRACE_Y, ROTARY_DISC_HEIGHT))
    )

    carriage = rotary_disc.union(saddle).union(low_brace)
    carriage = carriage.union(_cheek_shape(-1.0)).union(_cheek_shape(1.0))
    return carriage


def _table_plate_shape() -> cq.Workplane:
    table_plate = cq.Workplane("XY").circle(TABLE_RADIUS).extrude(TABLE_THICKNESS)
    table_plate = (
        table_plate.faces(">Z")
        .workplane()
        .circle(TABLE_RECESS_RADIUS)
        .cutBlind(-TABLE_RECESS_DEPTH)
    )

    slot_x = (
        cq.Workplane("XY")
        .slot2D(TABLE_SLOT_LENGTH, TABLE_SLOT_WIDTH)
        .extrude(TABLE_SLOT_DEPTH)
        .translate((0.0, 0.0, TABLE_THICKNESS - TABLE_SLOT_DEPTH))
    )
    slot_y = (
        cq.Workplane("XY")
        .slot2D(TABLE_SLOT_LENGTH, TABLE_SLOT_WIDTH, angle=90.0)
        .extrude(TABLE_SLOT_DEPTH)
        .translate((0.0, 0.0, TABLE_THICKNESS - TABLE_SLOT_DEPTH))
    )
    return table_plate.cut(slot_x).cut(slot_y).translate((0.0, 0.0, -(TABLE_THICKNESS / 2.0)))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rotary_table_with_trunnion")

    model.material("base_gray", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("frame_gray", rgba=(0.62, 0.65, 0.69, 1.0))
    model.material("table_steel", rgba=(0.76, 0.78, 0.80, 1.0))

    base = model.part("ground_base")
    _add_mesh_visual(base, _base_shape(), "ground_base", "base_gray", name="base_casting")

    carriage = model.part("rotary_carriage")
    carriage.visual(
        Cylinder(radius=ROTARY_DISC_RADIUS, length=ROTARY_DISC_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, ROTARY_DISC_HEIGHT / 2.0)),
        material="frame_gray",
        name="rotary_disc",
    )
    carriage.visual(
        Box((SADDLE_LENGTH, SADDLE_WIDTH, SADDLE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, ROTARY_DISC_HEIGHT + (SADDLE_HEIGHT / 2.0))),
        material="frame_gray",
        name="saddle_deck",
    )
    carriage.visual(
        Box((SADDLE_LENGTH * 0.76, LOW_BRACE_WIDTH, LOW_BRACE_HEIGHT)),
        origin=Origin(xyz=(0.0, LOW_BRACE_Y, ROTARY_DISC_HEIGHT + (LOW_BRACE_HEIGHT / 2.0))),
        material="frame_gray",
        name="lower_brace",
    )
    _add_mesh_visual(carriage, _cheek_shape(-1.0), "left_cheek", "frame_gray", name="left_cheek")
    _add_mesh_visual(carriage, _cheek_shape(1.0), "right_cheek", "frame_gray", name="right_cheek")

    work_table = model.part("work_table")
    _add_mesh_visual(work_table, _table_plate_shape(), "table_plate", "table_steel", name="table_plate")
    work_table.visual(
        Cylinder(radius=TABLE_HUB_RADIUS, length=TABLE_HUB_LENGTH),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material="table_steel",
        name="trunnion_hub",
    )
    work_table.visual(
        Cylinder(radius=SHAFT_RADIUS, length=2.0 * JOURNAL_HALF_SPAN),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material="table_steel",
        name="tilt_shaft",
    )
    work_table.visual(
        Cylinder(radius=TRUNNION_COLLAR_RADIUS, length=TRUNNION_COLLAR_THICKNESS),
        origin=Origin(
            xyz=(JOURNAL_HALF_SPAN - (TRUNNION_COLLAR_THICKNESS / 2.0), 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material="table_steel",
        name="right_collar",
    )
    work_table.visual(
        Cylinder(radius=TRUNNION_COLLAR_RADIUS, length=TRUNNION_COLLAR_THICKNESS),
        origin=Origin(
            xyz=(-(JOURNAL_HALF_SPAN - (TRUNNION_COLLAR_THICKNESS / 2.0)), 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material="table_steel",
        name="left_collar",
    )

    model.articulation(
        "base_rotation",
        ArticulationType.REVOLUTE,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, ROTARY_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-pi, upper=pi, effort=150.0, velocity=1.2),
    )
    model.articulation(
        "table_tilt",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=work_table,
        origin=Origin(xyz=(0.0, 0.0, TILT_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-1.0, upper=1.15, effort=90.0, velocity=0.9),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("ground_base")
    carriage = object_model.get_part("rotary_carriage")
    work_table = object_model.get_part("work_table")
    base_rotation = object_model.get_articulation("base_rotation")
    table_tilt = object_model.get_articulation("table_tilt")

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
        "joint_axes_match_machine_layout",
        tuple(base_rotation.axis) == (0.0, 0.0, 1.0) and tuple(table_tilt.axis) == (1.0, 0.0, 0.0),
        f"expected vertical base axis and transverse horizontal tilt axis, got {base_rotation.axis} and {table_tilt.axis}",
    )
    ctx.expect_contact(carriage, base, name="carriage_seats_on_rotary_base")
    ctx.expect_contact(work_table, carriage, name="table_is_carried_by_trunnion_cheeks")
    ctx.expect_gap(work_table, base, axis="z", min_gap=0.17, name="table_clears_grounded_base")

    rest_carriage_aabb = ctx.part_world_aabb(carriage)
    rest_table_plate_aabb = ctx.part_element_world_aabb(work_table, elem="table_plate")

    if rest_carriage_aabb is None or rest_table_plate_aabb is None:
        ctx.fail("world_aabbs_available", "expected carriage and work-table plate world AABBs in rest pose")
        return ctx.report()

    rest_carriage_x = rest_carriage_aabb[1][0] - rest_carriage_aabb[0][0]
    rest_carriage_y = rest_carriage_aabb[1][1] - rest_carriage_aabb[0][1]
    rest_table_plate_y = rest_table_plate_aabb[1][1] - rest_table_plate_aabb[0][1]
    rest_table_plate_z = rest_table_plate_aabb[1][2] - rest_table_plate_aabb[0][2]

    with ctx.pose({base_rotation: pi / 2.0}):
        turned_carriage_aabb = ctx.part_world_aabb(carriage)
    with ctx.pose({table_tilt: 0.9}):
        tilted_table_plate_aabb = ctx.part_element_world_aabb(work_table, elem="table_plate")

    if turned_carriage_aabb is None or tilted_table_plate_aabb is None:
        ctx.fail("posed_world_aabbs_available", "expected carriage and work-table plate world AABBs in posed checks")
        return ctx.report()

    turned_carriage_x = turned_carriage_aabb[1][0] - turned_carriage_aabb[0][0]
    turned_carriage_y = turned_carriage_aabb[1][1] - turned_carriage_aabb[0][1]
    tilted_table_plate_y = tilted_table_plate_aabb[1][1] - tilted_table_plate_aabb[0][1]
    tilted_table_plate_z = tilted_table_plate_aabb[1][2] - tilted_table_plate_aabb[0][2]

    ctx.check(
        "base_rotation_reorients_trunnion_frame",
        turned_carriage_x < (rest_carriage_x - 0.05) and turned_carriage_y > (rest_carriage_y + 0.05),
        (
            "expected quarter-turn base rotation to swing the long trunnion frame footprint from X into Y; "
            f"rest spans=({rest_carriage_x:.3f}, {rest_carriage_y:.3f}), "
            f"turned spans=({turned_carriage_x:.3f}, {turned_carriage_y:.3f})"
        ),
    )
    ctx.check(
        "table_tilt_changes_table_pose",
        tilted_table_plate_z > 0.17 and tilted_table_plate_y < (rest_table_plate_y - 0.02),
        (
            "expected the table plate to swing from a horizontal disk toward a near-vertical disk; "
            f"rest y/z spans=({rest_table_plate_y:.3f}, {rest_table_plate_z:.3f}), "
            f"tilted y/z spans=({tilted_table_plate_y:.3f}, {tilted_table_plate_z:.3f})"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
