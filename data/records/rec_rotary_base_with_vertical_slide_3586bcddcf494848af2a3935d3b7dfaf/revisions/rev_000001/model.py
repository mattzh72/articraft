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


BASE_RADIUS = 0.22
BASE_FOOT_H = 0.08
PEDESTAL_RADIUS = 0.12
PEDESTAL_H = 0.14

TURNTABLE_RADIUS = 0.17
TURNTABLE_T = 0.035

COLUMN_MOUNT_SIDE = 0.12
COLUMN_MOUNT_H = 0.08
COLUMN_SIDE = 0.09
COLUMN_SHAFT_H = 0.58
COLUMN_CAP_SIDE = 0.11
COLUMN_CAP_H = 0.04

CARRIAGE_WALL = 0.018
CARRIAGE_INNER = COLUMN_SIDE
CARRIAGE_OUTER = CARRIAGE_INNER + 2.0 * CARRIAGE_WALL
CARRIAGE_H = 0.12

TOOL_ARM_X = 0.055
TOOL_ARM_Y = 0.05
TOOL_ARM_H = 0.05
TOOL_ARM_CENTER_Z = 0.075

TOOL_PLATE_T = 0.012
TOOL_PLATE_W = 0.12
TOOL_PLATE_H = 0.12
TOOL_PLATE_CENTER_Z = 0.09

BASE_TOP_Z = BASE_FOOT_H + PEDESTAL_H
TURNTABLE_TOP_Z = TURNTABLE_T
COLUMN_TOTAL_H = COLUMN_MOUNT_H + COLUMN_SHAFT_H + COLUMN_CAP_H

BASE_SPIN_LOWER = -math.pi
BASE_SPIN_UPPER = math.pi
CARRIAGE_TRAVEL = 0.36


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rotary_pedestal_lifter")

    base_paint = model.material("base_paint", color=(0.24, 0.25, 0.27))
    machined_steel = model.material("machined_steel", color=(0.74, 0.75, 0.78))
    carriage_gray = model.material("carriage_gray", color=(0.58, 0.6, 0.62))
    tool_face = model.material("tool_face", color=(0.82, 0.83, 0.86))

    pedestal_base = model.part("pedestal_base")
    pedestal_base.visual(
        Cylinder(radius=BASE_RADIUS, length=BASE_FOOT_H),
        origin=Origin(xyz=(0.0, 0.0, BASE_FOOT_H / 2.0)),
        material=base_paint,
        name="base_foot",
    )
    pedestal_base.visual(
        Cylinder(radius=PEDESTAL_RADIUS, length=PEDESTAL_H),
        origin=Origin(xyz=(0.0, 0.0, BASE_FOOT_H + PEDESTAL_H / 2.0)),
        material=base_paint,
        name="pedestal_body",
    )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=TURNTABLE_RADIUS, length=TURNTABLE_T),
        origin=Origin(xyz=(0.0, 0.0, TURNTABLE_T / 2.0)),
        material=machined_steel,
        name="platter",
    )

    guide_column = model.part("guide_column")
    guide_column.visual(
        Box((COLUMN_MOUNT_SIDE, COLUMN_MOUNT_SIDE, COLUMN_MOUNT_H)),
        origin=Origin(xyz=(0.0, 0.0, COLUMN_MOUNT_H / 2.0)),
        material=machined_steel,
        name="column_mount",
    )
    guide_column.visual(
        Box((COLUMN_SIDE, COLUMN_SIDE, COLUMN_SHAFT_H)),
        origin=Origin(xyz=(0.0, 0.0, COLUMN_MOUNT_H + COLUMN_SHAFT_H / 2.0)),
        material=machined_steel,
        name="column_shaft",
    )
    guide_column.visual(
        Box((COLUMN_CAP_SIDE, COLUMN_CAP_SIDE, COLUMN_CAP_H)),
        origin=Origin(
            xyz=(0.0, 0.0, COLUMN_MOUNT_H + COLUMN_SHAFT_H + COLUMN_CAP_H / 2.0)
        ),
        material=machined_steel,
        name="column_cap",
    )

    carriage_head = model.part("carriage_head")
    carriage_head.visual(
        Box((CARRIAGE_WALL, CARRIAGE_OUTER, CARRIAGE_H)),
        origin=Origin(
            xyz=((CARRIAGE_INNER / 2.0) + (CARRIAGE_WALL / 2.0), 0.0, CARRIAGE_H / 2.0)
        ),
        material=carriage_gray,
        name="front_wall",
    )
    carriage_head.visual(
        Box((CARRIAGE_WALL, CARRIAGE_OUTER, CARRIAGE_H)),
        origin=Origin(
            xyz=(
                -((CARRIAGE_INNER / 2.0) + (CARRIAGE_WALL / 2.0)),
                0.0,
                CARRIAGE_H / 2.0,
            )
        ),
        material=carriage_gray,
        name="rear_wall",
    )
    carriage_head.visual(
        Box((CARRIAGE_OUTER, CARRIAGE_WALL, CARRIAGE_H)),
        origin=Origin(
            xyz=(0.0, (CARRIAGE_INNER / 2.0) + (CARRIAGE_WALL / 2.0), CARRIAGE_H / 2.0)
        ),
        material=carriage_gray,
        name="left_wall",
    )
    carriage_head.visual(
        Box((CARRIAGE_OUTER, CARRIAGE_WALL, CARRIAGE_H)),
        origin=Origin(
            xyz=(
                0.0,
                -((CARRIAGE_INNER / 2.0) + (CARRIAGE_WALL / 2.0)),
                CARRIAGE_H / 2.0,
            )
        ),
        material=carriage_gray,
        name="right_wall",
    )
    carriage_head.visual(
        Box((TOOL_ARM_X, TOOL_ARM_Y, TOOL_ARM_H)),
        origin=Origin(
            xyz=(CARRIAGE_OUTER / 2.0 + TOOL_ARM_X / 2.0, 0.0, TOOL_ARM_CENTER_Z)
        ),
        material=carriage_gray,
        name="tool_arm",
    )
    carriage_head.visual(
        Box((TOOL_PLATE_T, TOOL_PLATE_W, TOOL_PLATE_H)),
        origin=Origin(
            xyz=(
                CARRIAGE_OUTER / 2.0 + TOOL_ARM_X + TOOL_PLATE_T / 2.0,
                0.0,
                TOOL_PLATE_CENTER_Z,
            )
        ),
        material=tool_face,
        name="tool_plate",
    )

    model.articulation(
        "base_spin",
        ArticulationType.REVOLUTE,
        parent=pedestal_base,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.2,
            lower=BASE_SPIN_LOWER,
            upper=BASE_SPIN_UPPER,
        ),
    )
    model.articulation(
        "turntable_to_column",
        ArticulationType.FIXED,
        parent=turntable,
        child=guide_column,
        origin=Origin(xyz=(0.0, 0.0, TURNTABLE_TOP_Z)),
    )
    model.articulation(
        "carriage_lift",
        ArticulationType.PRISMATIC,
        parent=guide_column,
        child=carriage_head,
        origin=Origin(xyz=(0.0, 0.0, COLUMN_MOUNT_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.18,
            lower=0.0,
            upper=CARRIAGE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal_base = object_model.get_part("pedestal_base")
    turntable = object_model.get_part("turntable")
    guide_column = object_model.get_part("guide_column")
    carriage_head = object_model.get_part("carriage_head")

    base_spin = object_model.get_articulation("base_spin")
    carriage_lift = object_model.get_articulation("carriage_lift")
    tool_plate = carriage_head.get_visual("tool_plate")

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
        "base_spin_axis_vertical",
        base_spin.axis == (0.0, 0.0, 1.0),
        details=f"Expected vertical pedestal spin axis, got {base_spin.axis!r}.",
    )
    ctx.check(
        "carriage_lift_axis_vertical",
        carriage_lift.axis == (0.0, 0.0, 1.0),
        details=f"Expected vertical lift axis, got {carriage_lift.axis!r}.",
    )

    base_limits = base_spin.motion_limits
    lift_limits = carriage_lift.motion_limits
    ctx.check(
        "base_spin_limits_present",
        base_limits is not None
        and base_limits.lower is not None
        and base_limits.upper is not None
        and math.isclose(base_limits.lower, BASE_SPIN_LOWER, abs_tol=1e-9)
        and math.isclose(base_limits.upper, BASE_SPIN_UPPER, abs_tol=1e-9),
        details="Base turntable should rotate through a full bounded turn about the pedestal axis.",
    )
    ctx.check(
        "carriage_lift_limits_present",
        lift_limits is not None
        and lift_limits.lower is not None
        and lift_limits.upper is not None
        and math.isclose(lift_limits.lower, 0.0, abs_tol=1e-9)
        and math.isclose(lift_limits.upper, CARRIAGE_TRAVEL, abs_tol=1e-9),
        details="Carriage should start on the column shoulder and lift through the designed travel.",
    )

    ctx.expect_contact(
        turntable,
        pedestal_base,
        name="turntable_seated_on_pedestal",
    )
    ctx.expect_overlap(
        turntable,
        pedestal_base,
        axes="xy",
        min_overlap=0.30,
        name="turntable_centered_over_base",
    )
    ctx.expect_contact(
        guide_column,
        turntable,
        name="column_mounted_to_turntable",
    )
    ctx.expect_within(
        guide_column,
        turntable,
        axes="xy",
        margin=0.0,
        name="column_within_turntable_footprint",
    )
    ctx.expect_contact(
        carriage_head,
        guide_column,
        name="carriage_guided_on_column_rest",
    )
    ctx.expect_gap(
        carriage_head,
        turntable,
        axis="z",
        min_gap=COLUMN_MOUNT_H - 0.001,
        max_gap=COLUMN_MOUNT_H + 0.001,
        name="carriage_clears_turntable_at_rest",
    )
    ctx.expect_origin_gap(
        carriage_head,
        turntable,
        axis="z",
        min_gap=COLUMN_MOUNT_H + TURNTABLE_T - 0.001,
        max_gap=COLUMN_MOUNT_H + TURNTABLE_T + 0.001,
        name="carriage_origin_starts_above_turntable",
    )

    base_aabb = ctx.part_world_aabb(pedestal_base)
    column_aabb = ctx.part_world_aabb(guide_column)
    tool_plate_aabb = ctx.part_element_world_aabb(carriage_head, elem=tool_plate)
    if base_aabb is not None and column_aabb is not None and tool_plate_aabb is not None:
        base_width = base_aabb[1][0] - base_aabb[0][0]
        column_height = column_aabb[1][2] - column_aabb[0][2]
        tool_plate_height = tool_plate_aabb[1][2] - tool_plate_aabb[0][2]
        ctx.check(
            "realistic_pedestal_scale",
            base_width >= 0.43 and column_height >= 0.69 and tool_plate_height >= 0.11,
            details=(
                "Expected an industrial-scale pedestal with a substantial round base, "
                "a tall guide column, and a readable tooling plate."
            ),
        )

    if base_limits is not None and base_limits.lower is not None and base_limits.upper is not None:
        with ctx.pose({base_spin: base_limits.lower}):
            ctx.expect_contact(
                turntable,
                pedestal_base,
                name="turntable_contact_at_lower_spin_limit",
            )
            ctx.fail_if_parts_overlap_in_current_pose(name="base_spin_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="base_spin_lower_no_floating")
        with ctx.pose({base_spin: base_limits.upper}):
            ctx.expect_contact(
                turntable,
                pedestal_base,
                name="turntable_contact_at_upper_spin_limit",
            )
            ctx.fail_if_parts_overlap_in_current_pose(name="base_spin_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="base_spin_upper_no_floating")

    if lift_limits is not None and lift_limits.lower is not None and lift_limits.upper is not None:
        with ctx.pose({carriage_lift: lift_limits.lower}):
            ctx.expect_contact(
                carriage_head,
                guide_column,
                name="carriage_contact_at_lower_lift_limit",
            )
            ctx.expect_gap(
                carriage_head,
                turntable,
                axis="z",
                min_gap=COLUMN_MOUNT_H - 0.001,
                max_gap=COLUMN_MOUNT_H + 0.001,
                name="carriage_lower_pose_clearance",
            )
            ctx.fail_if_parts_overlap_in_current_pose(name="carriage_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="carriage_lower_no_floating")
        with ctx.pose({base_spin: math.pi / 2.0, carriage_lift: lift_limits.upper}):
            ctx.expect_contact(
                carriage_head,
                guide_column,
                name="carriage_contact_at_upper_lift_limit",
            )
            ctx.expect_gap(
                carriage_head,
                turntable,
                axis="z",
                min_gap=COLUMN_MOUNT_H + CARRIAGE_TRAVEL - 0.001,
                max_gap=COLUMN_MOUNT_H + CARRIAGE_TRAVEL + 0.001,
                name="carriage_upper_pose_clearance",
            )
            ctx.fail_if_parts_overlap_in_current_pose(name="carriage_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="carriage_upper_no_floating")

    ctx.fail_if_articulation_overlaps(
        max_pose_samples=24,
        name="articulations_clear_through_motion",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
