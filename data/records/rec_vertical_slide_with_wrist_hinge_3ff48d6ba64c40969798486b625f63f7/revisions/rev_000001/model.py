from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose

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


BASE_W = 0.18
BASE_D = 0.13
BASE_T = 0.018

PEDESTAL_W = 0.102
PEDESTAL_D = 0.082
PEDESTAL_T = 0.022

COLUMN_W = 0.084
COLUMN_D = 0.052
COLUMN_H = 0.318
COLUMN_Z0 = BASE_T + PEDESTAL_T

GUIDE_W = 0.056
GUIDE_D = 0.028
GUIDE_H = 0.314
GUIDE_Z0 = 0.082
GUIDE_Y = 0.028

TOP_W = 0.102
TOP_D = 0.082
TOP_T = 0.042
TOP_Z0 = COLUMN_Z0 + COLUMN_H

MOTOR_R = 0.018
MOTOR_H = 0.022

CARR_W = 0.112
CARR_D = 0.070
CARR_H = 0.084
CARR_HOME_Y = GUIDE_Y + (GUIDE_D / 2.0)
CARR_HOME_Z = 0.184
LIFT_TRAVEL = 0.160

HINGE_Y = 0.035
HINGE_Z = 0.010
TRUNNION_R = 0.010
HINGE_BORE_R = TRUNNION_R

WRIST_TRUNNION_L = 0.048
WRIST_BLOCK_W = 0.054
WRIST_BLOCK_D = 0.028
WRIST_BLOCK_H = 0.052
WRIST_BLOCK_Y = 0.035
WRIST_FACE_W = 0.094
WRIST_FACE_D = 0.010
WRIST_FACE_H = 0.074
WRIST_FACE_Y = 0.051
WRIST_TILT = 0.55


def _x_axis_cylinder(*, radius: float, length: float, y: float, z: float) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .center(y, z)
        .circle(radius)
        .extrude(length)
        .translate((-length / 2.0, 0.0, 0.0))
    )


def _make_column_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(BASE_W, BASE_D, BASE_T, centered=(True, True, False))
    pedestal = (
        cq.Workplane("XY")
        .box(PEDESTAL_W, PEDESTAL_D, PEDESTAL_T, centered=(True, True, False))
        .translate((0.0, 0.0, BASE_T))
    )
    mast = (
        cq.Workplane("XY")
        .box(COLUMN_W, COLUMN_D, COLUMN_H, centered=(True, True, False))
        .translate((0.0, 0.0, COLUMN_Z0))
    )
    guide = (
        cq.Workplane("XY")
        .box(GUIDE_W, GUIDE_D, GUIDE_H, centered=(True, True, False))
        .translate((0.0, GUIDE_Y, GUIDE_Z0))
    )
    top_cap = (
        cq.Workplane("XY")
        .box(TOP_W, TOP_D, TOP_T, centered=(True, True, False))
        .translate((0.0, 0.0, TOP_Z0))
    )
    motor = (
        cq.Workplane("XY")
        .circle(MOTOR_R)
        .extrude(MOTOR_H)
        .translate((0.0, 0.0, TOP_Z0 + TOP_T))
    )
    return base.union(pedestal).union(mast).union(guide).union(top_cap).union(motor)


def _make_carriage_shape() -> cq.Workplane:
    rear_slide_pad = cq.Workplane("XY").box(0.060, 0.006, 0.090).translate((0.0, 0.003, 0.0))
    main_block = cq.Workplane("XY").box(0.094, 0.034, 0.082).translate((0.0, 0.023, -0.002))
    lower_saddle = cq.Workplane("XY").box(0.056, 0.014, 0.024).translate((0.0, 0.034, -0.026))
    left_ear = cq.Workplane("XY").box(0.016, 0.016, 0.056).translate((-0.032, HINGE_Y, HINGE_Z))
    right_ear = cq.Workplane("XY").box(0.016, 0.016, 0.056).translate((0.032, HINGE_Y, HINGE_Z))
    return rear_slide_pad.union(main_block).union(lower_saddle).union(left_ear).union(right_ear)


def _make_wrist_shape() -> cq.Workplane:
    trunnion = _x_axis_cylinder(radius=TRUNNION_R, length=WRIST_TRUNNION_L, y=0.0, z=0.0)
    neck = cq.Workplane("XY").box(0.032, 0.020, 0.040).translate(
        (0.0, 0.016, 0.0)
    )
    rib = cq.Workplane("XY").box(WRIST_BLOCK_W, WRIST_BLOCK_D, WRIST_BLOCK_H).translate(
        (0.0, WRIST_BLOCK_Y, 0.0)
    )
    face = cq.Workplane("XY").box(WRIST_FACE_W, WRIST_FACE_D, WRIST_FACE_H).translate(
        (0.0, WRIST_FACE_Y, 0.0)
    )
    return trunnion.union(neck).union(rib).union(face)


def _aabb_center_z(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> float | None:
    if aabb is None:
        return None
    return 0.5 * (aabb[0][2] + aabb[1][2])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_lift_axis")

    model.material("frame_dark", rgba=(0.26, 0.28, 0.31, 1.0))
    model.material("carriage_silver", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("tool_gray", rgba=(0.50, 0.54, 0.58, 1.0))

    column = model.part("column")
    column.visual(
        Box((BASE_W, BASE_D, BASE_T)),
        origin=Origin(xyz=(0.0, 0.0, BASE_T / 2.0)),
        material="frame_dark",
        name="base_plate",
    )
    column.visual(
        Box((PEDESTAL_W, PEDESTAL_D, PEDESTAL_T)),
        origin=Origin(xyz=(0.0, 0.0, BASE_T + (PEDESTAL_T / 2.0))),
        material="frame_dark",
        name="pedestal",
    )
    column.visual(
        Box((0.070, 0.040, COLUMN_H)),
        origin=Origin(xyz=(0.0, 0.0, COLUMN_Z0 + (COLUMN_H / 2.0))),
        material="frame_dark",
        name="mast",
    )
    column.visual(
        Box((0.014, 0.012, GUIDE_H)),
        origin=Origin(xyz=(-0.022, 0.036, GUIDE_Z0 + (GUIDE_H / 2.0))),
        material="frame_dark",
        name="left_rail",
    )
    column.visual(
        Box((0.014, 0.012, GUIDE_H)),
        origin=Origin(xyz=(0.022, 0.036, GUIDE_Z0 + (GUIDE_H / 2.0))),
        material="frame_dark",
        name="right_rail",
    )
    column.visual(
        Box((TOP_W, TOP_D, TOP_T)),
        origin=Origin(xyz=(0.0, 0.0, TOP_Z0 + (TOP_T / 2.0))),
        material="frame_dark",
        name="top_cap",
    )
    column.visual(
        Cylinder(radius=MOTOR_R, length=MOTOR_H),
        origin=Origin(xyz=(0.0, 0.0, TOP_Z0 + TOP_T + (MOTOR_H / 2.0))),
        material="frame_dark",
        name="motor_housing",
    )
    column.inertial = Inertial.from_geometry(
        Box((BASE_W, BASE_D, TOP_Z0 + TOP_T + MOTOR_H)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, (TOP_Z0 + TOP_T + MOTOR_H) / 2.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.070, 0.006, 0.090)),
        origin=Origin(xyz=(0.0, 0.003, 0.0)),
        material="carriage_silver",
        name="rear_bridge",
    )
    carriage.visual(
        Box((0.094, 0.028, 0.070)),
        origin=Origin(xyz=(0.0, 0.020, -0.002)),
        material="carriage_silver",
        name="carriage_body",
    )
    carriage.visual(
        Box((0.040, 0.010, 0.028)),
        origin=Origin(xyz=(0.0, 0.028, -0.014)),
        material="carriage_silver",
        name="center_web",
    )
    carriage.visual(
        Box((0.016, 0.014, 0.042)),
        origin=Origin(xyz=(-0.030, 0.035, 0.010)),
        material="carriage_silver",
        name="left_ear",
    )
    carriage.visual(
        Box((0.016, 0.014, 0.042)),
        origin=Origin(xyz=(0.030, 0.035, 0.010)),
        material="carriage_silver",
        name="right_ear",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.094, 0.041, 0.090)),
        mass=2.1,
        origin=Origin(xyz=(0.0, 0.0205, 0.0)),
    )

    wrist = model.part("wrist_plate")
    wrist.visual(
        Box((0.044, 0.014, 0.040)),
        origin=Origin(xyz=(0.0, 0.007, 0.0)),
        material="tool_gray",
        name="hinge_block",
    )
    wrist.visual(
        Box((0.030, 0.018, 0.030)),
        origin=Origin(xyz=(0.0, 0.022, 0.0)),
        material="tool_gray",
        name="neck",
    )
    wrist.visual(
        Box((0.052, 0.010, 0.052)),
        origin=Origin(xyz=(0.0, 0.030, 0.0)),
        material="tool_gray",
        name="back_rib",
    )
    wrist.visual(
        Box((WRIST_FACE_W, 0.010, WRIST_FACE_H)),
        origin=Origin(xyz=(0.0, 0.036, 0.0)),
        material="tool_gray",
        name="wrist_plate",
    )
    wrist.inertial = Inertial.from_geometry(
        Box((WRIST_FACE_W, 0.046, WRIST_FACE_H)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.023, 0.0)),
    )

    model.articulation(
        "column_lift",
        ArticulationType.PRISMATIC,
        parent=column,
        child=carriage,
        origin=Origin(xyz=(0.0, CARR_HOME_Y, CARR_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=0.20,
            lower=0.0,
            upper=LIFT_TRAVEL,
        ),
    )
    model.articulation(
        "carriage_wrist",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=wrist,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=22.0,
            velocity=1.4,
            lower=-0.80,
            upper=0.95,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    column = object_model.get_part("column")
    carriage = object_model.get_part("carriage")
    wrist = object_model.get_part("wrist_plate")
    lift = object_model.get_articulation("column_lift")
    wrist_joint = object_model.get_articulation("carriage_wrist")

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
        "lift joint axis is vertical",
        lift.axis == (0.0, 0.0, 1.0),
        details=f"expected vertical prismatic axis, got {lift.axis}",
    )
    ctx.check(
        "wrist joint axis is horizontal",
        wrist_joint.axis == (1.0, 0.0, 0.0),
        details=f"expected horizontal wrist axis, got {wrist_joint.axis}",
    )

    ctx.expect_contact(
        carriage,
        column,
        contact_tol=1e-5,
        name="carriage rides on the fixed column guide",
    )
    ctx.expect_contact(
        wrist,
        carriage,
        contact_tol=1e-5,
        name="wrist trunnion is seated in the carriage clevis",
    )

    with ctx.pose({lift: LIFT_TRAVEL}):
        ctx.expect_contact(
            carriage,
            column,
            contact_tol=1e-5,
            name="carriage stays supported at full lift",
        )

    home_pos = ctx.part_world_position(carriage)
    with ctx.pose({lift: LIFT_TRAVEL}):
        raised_pos = ctx.part_world_position(carriage)
    ctx.check(
        "prismatic joint lifts upward",
        home_pos is not None
        and raised_pos is not None
        and isclose(raised_pos[0], home_pos[0], abs_tol=1e-6)
        and isclose(raised_pos[1], home_pos[1], abs_tol=1e-6)
        and raised_pos[2] > home_pos[2] + (LIFT_TRAVEL - 1e-4),
        details=f"home={home_pos}, raised={raised_pos}",
    )

    with ctx.pose({lift: LIFT_TRAVEL * 0.5, wrist_joint: 0.0}):
        neutral_aabb = ctx.part_world_aabb(wrist)
    with ctx.pose({lift: LIFT_TRAVEL * 0.5, wrist_joint: WRIST_TILT}):
        tilted_aabb = ctx.part_world_aabb(wrist)
        ctx.expect_contact(
            wrist,
            carriage,
            contact_tol=1e-5,
            name="wrist stays supported while tilted",
        )

    neutral_cz = _aabb_center_z(neutral_aabb)
    tilted_cz = _aabb_center_z(tilted_aabb)
    ctx.check(
        "positive wrist rotation lifts the tool face",
        neutral_cz is not None and tilted_cz is not None and tilted_cz > neutral_cz + 0.008,
        details=f"neutral_z={neutral_cz}, tilted_z={tilted_cz}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
