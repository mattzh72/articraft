from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

BASE_W = 0.42
BASE_D = 0.34
BASE_H = 0.09
BASE_PAD_D = 0.19
BASE_PAD_H = 0.01

TURNTABLE_D = 0.24
TURNTABLE_H = 0.04
TURNTABLE_CAP_D = 0.18
TURNTABLE_CAP_H = 0.01

CRADLE_BASE_X = 0.16
CRADLE_BASE_Y = 0.14
CRADLE_BASE_T = 0.018
CRADLE_ARM_T = 0.018
CRADLE_ARM_Y = 0.15
CRADLE_ARM_H = 0.14
CRADLE_INNER_SPAN = 0.17
CRADLE_REAR_BEAM_Y = 0.02
CRADLE_REAR_BEAM_Z = 0.024
CRADLE_SIDE_RISER_X = 0.034
CRADLE_SIDE_RISER_Y = 0.044
CRADLE_SIDE_RISER_H = 0.038
TRUNNION_AXIS_Z = 0.098
TRUNNION_R = 0.012

TABLE_SIZE = 0.16
TABLE_T = 0.022
TABLE_BODY_X = 0.1
TABLE_BODY_Y = 0.09
TABLE_BODY_H = 0.052
TABLE_BODY_BOTTOM_Z = -0.006
TABLE_PLATE_AXIS_OFFSET = 0.045
TABLE_JOURNAL_SPAN = CRADLE_INNER_SPAN + CRADLE_ARM_T * 1.2
TABLE_JOURNAL_R = TRUNNION_R
TABLE_END_CAP_R = 0.016
TABLE_END_CAP_T = 0.005
TABLE_SLOT_W = 0.014
TABLE_SLOT_D = 0.005


def _base_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(BASE_W, BASE_D, BASE_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.01)
    )
    pad = (
        cq.Workplane("XY")
        .workplane(offset=BASE_H)
        .circle(BASE_PAD_D / 2.0)
        .extrude(BASE_PAD_H)
    )
    foot_relief = (
        cq.Workplane("XZ")
        .rect(BASE_W * 0.58, BASE_H * 0.42)
        .extrude(BASE_D * 0.58, both=True)
        .translate((0.0, 0.0, BASE_H * 0.32))
    )
    return body.cut(foot_relief).union(pad)


def _turntable_shape() -> cq.Workplane:
    disk = cq.Workplane("XY").circle(TURNTABLE_D / 2.0).extrude(TURNTABLE_H)
    disk = (
        disk.faces(">Z")
        .workplane()
        .circle(0.032)
        .cutBlind(-0.006)
    )
    top_cap = (
        cq.Workplane("XY")
        .workplane(offset=TURNTABLE_H)
        .circle(TURNTABLE_CAP_D / 2.0)
        .extrude(TURNTABLE_CAP_H)
    )
    return disk.union(top_cap)


def _cradle_shape() -> cq.Workplane:
    arm_center_x = CRADLE_INNER_SPAN / 2.0 + CRADLE_ARM_T / 2.0
    mount_plate = cq.Workplane("XY").box(
        CRADLE_BASE_X,
        CRADLE_BASE_Y,
        CRADLE_BASE_T,
        centered=(True, True, False),
    )
    side_riser = (
        cq.Workplane("XY")
        .box(
            CRADLE_SIDE_RISER_X,
            CRADLE_SIDE_RISER_Y,
            CRADLE_SIDE_RISER_H,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, CRADLE_BASE_T))
    )
    left_riser = side_riser.translate((arm_center_x, 0.0, 0.0))
    right_riser = side_riser.translate((-arm_center_x, 0.0, 0.0))
    cheek_profile = (
        cq.Workplane("YZ")
        .center(0.0, CRADLE_ARM_H / 2.0)
        .rect(CRADLE_ARM_Y, CRADLE_ARM_H)
        .extrude(CRADLE_ARM_T)
    )
    cheek_profile = (
        cheek_profile.faces(">X")
        .workplane()
        .center(0.0, TRUNNION_AXIS_Z - CRADLE_ARM_H / 2.0)
        .circle(TRUNNION_R + 0.0008)
        .cutThruAll()
    )
    left_arm = cheek_profile.translate((arm_center_x - CRADLE_ARM_T / 2.0, 0.0, CRADLE_BASE_T + CRADLE_SIDE_RISER_H))
    right_arm = cheek_profile.translate((-arm_center_x + CRADLE_ARM_T / 2.0, 0.0, CRADLE_BASE_T + CRADLE_SIDE_RISER_H))
    rear_beam = (
        cq.Workplane("XY")
        .box(
            CRADLE_INNER_SPAN + 2.0 * CRADLE_ARM_T,
            CRADLE_REAR_BEAM_Y,
            CRADLE_REAR_BEAM_Z,
            centered=(True, True, False),
        )
        .translate((0.0, -(CRADLE_ARM_Y - CRADLE_REAR_BEAM_Y) / 2.0, CRADLE_BASE_T + CRADLE_SIDE_RISER_H + CRADLE_ARM_H - CRADLE_REAR_BEAM_Z))
    )
    return mount_plate.union(left_riser).union(right_riser).union(left_arm).union(right_arm).union(rear_beam)


def _table_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(TABLE_SIZE, TABLE_SIZE, TABLE_T).translate((0.0, 0.0, TABLE_PLATE_AXIS_OFFSET))
    body = (
        cq.Workplane("XY")
        .box(TABLE_BODY_X, TABLE_BODY_Y, TABLE_BODY_H, centered=(True, True, False))
        .translate((0.0, 0.0, 0.0))
    )
    journal = cq.Workplane("YZ").circle(TABLE_JOURNAL_R).extrude(TABLE_JOURNAL_SPAN / 2.0, both=True)
    end_cap = cq.Workplane("YZ").circle(TABLE_END_CAP_R).extrude(TABLE_END_CAP_T / 2.0, both=True)
    left_cap = end_cap.translate((-(TABLE_JOURNAL_SPAN / 2.0 - TABLE_END_CAP_T / 2.0), 0.0, 0.0))
    right_cap = end_cap.translate(((TABLE_JOURNAL_SPAN / 2.0 - TABLE_END_CAP_T / 2.0), 0.0, 0.0))
    table = plate.union(body).union(journal).union(left_cap).union(right_cap)
    for x_pos in (-0.045, 0.0, 0.045):
        groove = (
            cq.Workplane("XZ")
            .center(x_pos, TABLE_PLATE_AXIS_OFFSET + TABLE_T / 2.0 - TABLE_SLOT_D / 2.0)
            .rect(TABLE_SLOT_W, TABLE_SLOT_D)
            .extrude(TABLE_SIZE * 0.62, both=True)
        )
        table = table.cut(groove)
    return table


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rotary_table_trunnion", assets=ASSETS)

    cast_iron = model.material("cast_iron", rgba=(0.42, 0.45, 0.48, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.62, 0.65, 0.68, 1.0))
    table_finish = model.material("table_finish", rgba=(0.52, 0.56, 0.60, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_shape(), "base.obj", assets=ASSETS),
        origin=Origin(),
        material=cast_iron,
        name="base_shell",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_W, BASE_D, BASE_H + BASE_PAD_H)),
        mass=26.0,
        origin=Origin(xyz=(0.0, 0.0, (BASE_H + BASE_PAD_H) / 2.0)),
    )

    turntable = model.part("turntable")
    turntable.visual(
        mesh_from_cadquery(_turntable_shape(), "turntable.obj", assets=ASSETS),
        origin=Origin(),
        material=machined_steel,
        name="turntable_shell",
    )
    turntable.inertial = Inertial.from_geometry(
        Cylinder(radius=TURNTABLE_D / 2.0, length=TURNTABLE_H + TURNTABLE_CAP_H),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, (TURNTABLE_H + TURNTABLE_CAP_H) / 2.0)),
    )

    arm_center_x = CRADLE_INNER_SPAN / 2.0 + CRADLE_ARM_T / 2.0
    cradle = model.part("cradle")
    cradle.visual(
        Box((CRADLE_BASE_X, CRADLE_BASE_Y, CRADLE_BASE_T)),
        origin=Origin(xyz=(0.0, 0.0, CRADLE_BASE_T / 2.0)),
        material=cast_iron,
        name="cradle_mount",
    )
    cradle.visual(
        Box((CRADLE_SIDE_RISER_X, CRADLE_SIDE_RISER_Y, CRADLE_SIDE_RISER_H)),
        origin=Origin(xyz=(arm_center_x, 0.0, CRADLE_BASE_T + CRADLE_SIDE_RISER_H / 2.0)),
        material=cast_iron,
        name="left_riser",
    )
    cradle.visual(
        Box((CRADLE_SIDE_RISER_X, CRADLE_SIDE_RISER_Y, CRADLE_SIDE_RISER_H)),
        origin=Origin(xyz=(-arm_center_x, 0.0, CRADLE_BASE_T + CRADLE_SIDE_RISER_H / 2.0)),
        material=cast_iron,
        name="right_riser",
    )
    cradle.visual(
        Box((CRADLE_ARM_T, CRADLE_ARM_Y, CRADLE_ARM_H)),
        origin=Origin(
            xyz=(arm_center_x, 0.0, CRADLE_BASE_T + CRADLE_SIDE_RISER_H + CRADLE_ARM_H / 2.0)
        ),
        material=cast_iron,
        name="left_arm",
    )
    cradle.visual(
        Box((CRADLE_ARM_T, CRADLE_ARM_Y, CRADLE_ARM_H)),
        origin=Origin(
            xyz=(-arm_center_x, 0.0, CRADLE_BASE_T + CRADLE_SIDE_RISER_H + CRADLE_ARM_H / 2.0)
        ),
        material=cast_iron,
        name="right_arm",
    )
    cradle.inertial = Inertial.from_geometry(
        Box((CRADLE_INNER_SPAN + 2.0 * CRADLE_ARM_T, CRADLE_ARM_Y, CRADLE_ARM_H + CRADLE_BASE_T)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, (CRADLE_ARM_H + CRADLE_BASE_T) / 2.0)),
    )

    table = model.part("table")
    table.visual(
        Box((TABLE_SIZE, TABLE_SIZE, TABLE_T)),
        origin=Origin(xyz=(0.0, 0.0, TABLE_PLATE_AXIS_OFFSET + TABLE_T / 2.0)),
        material=table_finish,
        name="table_plate",
    )
    table.visual(
        Box((TABLE_BODY_X, TABLE_BODY_Y, TABLE_BODY_H)),
        origin=Origin(xyz=(0.0, 0.0, TABLE_BODY_BOTTOM_Z + TABLE_BODY_H / 2.0)),
        material=machined_steel,
        name="table_body",
    )
    journal_len = arm_center_x - CRADLE_ARM_T / 2.0 - TABLE_BODY_X / 2.0
    journal_center_x = TABLE_BODY_X / 2.0 + journal_len / 2.0
    table.visual(
        Cylinder(radius=TABLE_JOURNAL_R, length=journal_len),
        origin=Origin(xyz=(journal_center_x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="left_journal",
    )
    table.visual(
        Cylinder(radius=TABLE_JOURNAL_R, length=journal_len),
        origin=Origin(xyz=(-journal_center_x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="right_journal",
    )
    table.inertial = Inertial.from_geometry(
        Box((TABLE_JOURNAL_SPAN, TABLE_SIZE, TABLE_PLATE_AXIS_OFFSET + TABLE_T + 0.003)),
        mass=5.0,
        origin=Origin(xyz=(0.0, 0.0, (TABLE_PLATE_AXIS_OFFSET + TABLE_T + 0.003) / 2.0 - 0.0015)),
    )

    model.articulation(
        "base_to_turntable",
        ArticulationType.REVOLUTE,
        parent=base,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, BASE_H + BASE_PAD_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.25,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "turntable_to_cradle",
        ArticulationType.FIXED,
        parent=turntable,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, TURNTABLE_H + TURNTABLE_CAP_H)),
    )
    model.articulation(
        "cradle_to_table",
        ArticulationType.REVOLUTE,
        parent=cradle,
        child=table,
        origin=Origin(xyz=(0.0, 0.0, TRUNNION_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.1,
            lower=-math.pi / 3.0,
            upper=math.pi / 3.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    turntable = object_model.get_part("turntable")
    cradle = object_model.get_part("cradle")
    table = object_model.get_part("table")
    rotary = object_model.get_articulation("base_to_turntable")
    tilt = object_model.get_articulation("cradle_to_table")

    base_shell = base.get_visual("base_shell")
    turntable_shell = turntable.get_visual("turntable_shell")
    cradle_mount = cradle.get_visual("cradle_mount")
    left_arm = cradle.get_visual("left_arm")
    right_arm = cradle.get_visual("right_arm")
    table_plate = table.get_visual("table_plate")
    left_journal = table.get_visual("left_journal")
    right_journal = table.get_visual("right_journal")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
        "part_visuals_present",
        all(v is not None for v in (base_shell, turntable_shell, cradle_mount, left_arm, right_arm, table_plate, left_journal, right_journal)),
        "Expected named visuals for base, turntable, cradle mount/arms, and table plate/journals.",
    )
    ctx.check(
        "rotary_range_matches_half_turn_each_way",
        rotary.motion_limits is not None
        and rotary.motion_limits.lower <= -3.10
        and rotary.motion_limits.upper >= 3.10,
        "Base rotation should allow about 180 degrees each way.",
    )
    ctx.check(
        "tilt_range_matches_sixty_deg_each_way",
        tilt.motion_limits is not None
        and tilt.motion_limits.lower <= -1.04
        and tilt.motion_limits.upper >= 1.04,
        "Trunnion tilt should allow about 60 degrees each way.",
    )

    ctx.expect_contact(turntable, base, elem_a=turntable_shell, elem_b=base_shell, name="turntable_seated_on_base")
    ctx.expect_overlap(turntable, base, axes="xy", min_overlap=0.18, name="turntable_centered_over_base")
    ctx.expect_origin_distance(turntable, base, axes="xy", max_dist=0.001, name="turntable_axis_centered_on_base")

    ctx.expect_contact(cradle, turntable, elem_a=cradle_mount, elem_b=turntable_shell, name="cradle_mounted_to_turntable")
    ctx.expect_overlap(cradle, turntable, axes="xy", min_overlap=0.10, name="cradle_footprint_on_turntable")
    ctx.expect_origin_gap(table, cradle, axis="z", min_gap=0.09, max_gap=0.12, name="table_axis_elevated_between_arms")
    ctx.expect_contact(table, cradle, elem_a=left_journal, elem_b=left_arm, name="left_journal_supported_by_left_arm")
    ctx.expect_contact(table, cradle, elem_a=right_journal, elem_b=right_arm, name="right_journal_supported_by_right_arm")
    ctx.expect_origin_distance(table, cradle, axes="y", max_dist=0.001, name="table_centered_between_cradle_arms_in_y")

    with ctx.pose({rotary: math.pi / 2.0}):
        ctx.expect_contact(turntable, base, elem_a=turntable_shell, elem_b=base_shell, name="turntable_keeps_contact_at_quarter_turn")
        ctx.expect_contact(cradle, turntable, elem_a=cradle_mount, elem_b=turntable_shell, name="cradle_keeps_contact_while_rotated")
        ctx.expect_contact(table, cradle, elem_a=left_journal, elem_b=left_arm, name="left_journal_keeps_support_while_rotary_moves")
        ctx.expect_contact(table, cradle, elem_a=right_journal, elem_b=right_arm, name="right_journal_keeps_support_while_rotary_moves")

    with ctx.pose({rotary: -math.pi}):
        ctx.expect_origin_distance(turntable, base, axes="xy", max_dist=0.001, name="turntable_stays_on_vertical_axis_at_limit")

    with ctx.pose({tilt: math.pi / 3.0}):
        ctx.expect_contact(table, cradle, elem_a=left_journal, elem_b=left_arm, name="left_journal_keeps_support_at_positive_tilt")
        ctx.expect_contact(table, cradle, elem_a=right_journal, elem_b=right_arm, name="right_journal_keeps_support_at_positive_tilt")
        ctx.expect_gap(table, turntable, axis="z", min_gap=0.02, name="table_clears_turntable_at_positive_tilt")

    with ctx.pose({tilt: -math.pi / 3.0}):
        ctx.expect_contact(table, cradle, elem_a=left_journal, elem_b=left_arm, name="left_journal_keeps_support_at_negative_tilt")
        ctx.expect_contact(table, cradle, elem_a=right_journal, elem_b=right_arm, name="right_journal_keeps_support_at_negative_tilt")
        ctx.expect_gap(table, turntable, axis="z", min_gap=0.02, name="table_clears_turntable_at_negative_tilt")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
