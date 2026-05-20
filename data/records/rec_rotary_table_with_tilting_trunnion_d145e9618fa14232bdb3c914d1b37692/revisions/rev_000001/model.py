from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BASE_X = 0.72
BASE_Y = 0.40
BASE_T = 0.04
BASE_X_CENTER = 0.14

WALL_X = 0.34
WALL_T = 0.04
WALL_H = 0.34
WALL_X_CENTER = 0.05
WALL_Y_CENTER = -0.18

PEDESTAL_R = 0.105
PEDESTAL_H = 0.12
ROTARY_Z = BASE_T + PEDESTAL_H

PLATTER_R = 0.10
PLATTER_T = 0.026
TRUNNION_X = 0.175
TRUNNION_Z = 0.17
CHEEK_Y = 0.11
CHEEK_T = 0.02

PIN_R = 0.019
PIN_LENGTH = 0.20
TABLE_BLOCK_X = 0.06
TABLE_BLOCK_Z = 0.045
TABLE_PLATE_X = 0.21
TABLE_PLATE_Z = 0.11


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_rotary_tilt_fixture")

    support_gray = model.material("support_gray", rgba=(0.25, 0.27, 0.30, 1.0))
    stage_gray = model.material("stage_gray", rgba=(0.48, 0.49, 0.50, 1.0))
    table_gray = model.material("table_gray", rgba=(0.66, 0.68, 0.70, 1.0))

    support = model.part("support")
    support.visual(
        Box((BASE_X, BASE_Y, BASE_T)),
        origin=Origin(xyz=(BASE_X_CENTER, 0.0, BASE_T / 2.0)),
        material=support_gray,
        name="base_plate",
    )
    support.visual(
        Cylinder(radius=PEDESTAL_R, length=PEDESTAL_H),
        origin=Origin(xyz=(0.0, 0.0, BASE_T + PEDESTAL_H / 2.0)),
        material=support_gray,
        name="rotary_pedestal",
    )
    support.visual(
        Box((WALL_X, WALL_T, WALL_H)),
        origin=Origin(xyz=(WALL_X_CENTER, WALL_Y_CENTER, BASE_T + WALL_H / 2.0)),
        material=support_gray,
        name="side_wall",
    )
    support.visual(
        Box((0.24, 0.10, 0.08)),
        origin=Origin(xyz=(0.00, -0.10, BASE_T + 0.04)),
        material=support_gray,
        name="rear_rib",
    )

    rotary_stage = model.part("rotary_stage")
    rotary_stage.visual(
        Cylinder(radius=PLATTER_R, length=PLATTER_T),
        origin=Origin(xyz=(0.0, 0.0, PLATTER_T / 2.0)),
        material=stage_gray,
        name="stage_platter",
    )
    rotary_stage.visual(
        Box((0.08, 0.08, 0.14)),
        origin=Origin(xyz=(0.05, 0.0, 0.096)),
        material=stage_gray,
        name="stage_column",
    )
    rotary_stage.visual(
        Box((0.16, 0.22, 0.04)),
        origin=Origin(xyz=(0.10, 0.0, 0.12)),
        material=stage_gray,
        name="trunnion_beam",
    )
    rotary_stage.visual(
        Box((0.03, CHEEK_T, 0.14)),
        origin=Origin(xyz=(TRUNNION_X, CHEEK_Y, TRUNNION_Z)),
        material=stage_gray,
        name="left_cheek",
    )
    rotary_stage.visual(
        Box((0.03, CHEEK_T, 0.14)),
        origin=Origin(xyz=(TRUNNION_X, -CHEEK_Y, TRUNNION_Z)),
        material=stage_gray,
        name="right_cheek",
    )

    tilt_table = model.part("tilt_table")
    tilt_table.visual(
        Cylinder(radius=PIN_R, length=PIN_LENGTH),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=table_gray,
        name="trunnion_pin",
    )
    tilt_table.visual(
        Box((0.12, 0.09, 0.09)),
        origin=Origin(xyz=(TABLE_BLOCK_X, 0.0, TABLE_BLOCK_Z)),
        material=table_gray,
        name="table_block",
    )
    tilt_table.visual(
        Box((0.34, 0.20, 0.045)),
        origin=Origin(xyz=(TABLE_PLATE_X, 0.0, TABLE_PLATE_Z)),
        material=table_gray,
        name="table_plate",
    )

    model.articulation(
        "support_to_rotary_stage",
        ArticulationType.REVOLUTE,
        parent=support,
        child=rotary_stage,
        origin=Origin(xyz=(0.0, 0.0, ROTARY_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.4,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    model.articulation(
        "rotary_stage_to_tilt_table",
        ArticulationType.REVOLUTE,
        parent=rotary_stage,
        child=tilt_table,
        origin=Origin(xyz=(TRUNNION_X, 0.0, TRUNNION_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.2,
            lower=-0.35,
            upper=1.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    rotary_stage = object_model.get_part("rotary_stage")
    tilt_table = object_model.get_part("tilt_table")
    c_axis = object_model.get_articulation("support_to_rotary_stage")
    a_axis = object_model.get_articulation("rotary_stage_to_tilt_table")

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

    ctx.expect_contact(
        rotary_stage,
        support,
        name="rotary_stage_is_seated_on_support",
    )
    ctx.expect_contact(
        tilt_table,
        rotary_stage,
        name="tilt_table_is_carried_by_trunnions",
    )
    ctx.expect_origin_gap(
        tilt_table,
        rotary_stage,
        axis="x",
        min_gap=0.15,
        name="table_sits_forward_of_rotary_stage",
    )

    with ctx.pose({a_axis: 0.75}):
        ctx.expect_contact(
            tilt_table,
            rotary_stage,
            name="tilted_table_remains_seated_in_trunnions",
        )

    rest_position = ctx.part_world_position(tilt_table)
    with ctx.pose({c_axis: math.pi / 2.0}):
        quarter_turn_position = ctx.part_world_position(tilt_table)

    c_axis_ok = (
        rest_position is not None
        and quarter_turn_position is not None
        and rest_position[0] > 0.15
        and abs(rest_position[1]) < 0.02
        and abs(quarter_turn_position[0]) < 0.03
        and quarter_turn_position[1] > rest_position[0] - 0.03
        and abs(quarter_turn_position[2] - rest_position[2]) < 0.01
    )
    ctx.check(
        "c_axis_positive_rotation_swings_fixture_sideways",
        c_axis_ok,
        details=(
            f"rest={rest_position}, quarter_turn={quarter_turn_position}; "
            "expected +90 deg C-axis motion to move the table center from +X toward +Y."
        ),
    )

    level_aabb = ctx.part_world_aabb(tilt_table)
    with ctx.pose({a_axis: 0.80}):
        tilted_aabb = ctx.part_world_aabb(tilt_table)

    a_axis_ok = (
        level_aabb is not None
        and tilted_aabb is not None
        and tilted_aabb[1][2] > level_aabb[1][2] + 0.08
    )
    ctx.check(
        "a_axis_positive_tilt_lifts_table_front",
        a_axis_ok,
        details=(
            f"level_aabb={level_aabb}, tilted_aabb={tilted_aabb}; "
            "expected positive tilt to raise the table envelope."
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
