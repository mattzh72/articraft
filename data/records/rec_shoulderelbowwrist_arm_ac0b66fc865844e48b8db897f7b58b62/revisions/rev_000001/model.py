from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_PLATE_L = 0.22
BASE_PLATE_W = 0.16
BASE_PLATE_T = 0.018
BASE_COLUMN_L = 0.064
BASE_COLUMN_W = 0.106
BASE_COLUMN_H = 0.132

SHOULDER_Z = 0.18
FORK_GAP = 0.050
FORK_CHEEK_T = 0.014
FORK_CHEEK_L = 0.078
FORK_CHEEK_H = 0.094

UPPER_ARM_LENGTH = 0.235
UPPER_SHOULDER_RADIUS = 0.028
UPPER_ELBOW_GAP = 0.036
UPPER_ELBOW_CHEEK_T = 0.012

FOREARM_LENGTH = 0.185
FOREARM_ELBOW_RADIUS = 0.022
FOREARM_WRIST_GAP = 0.028
FOREARM_WRIST_CHEEK_T = 0.010

WRIST_BARREL_RADIUS = 0.017
WRIST_FLANGE_RADIUS = 0.036
WRIST_FLANGE_T = 0.012
WRIST_BODY_LENGTH = 0.068


def _joint_bosses(
    radius: float,
    thickness: float,
    y_offset: float,
    *,
    x_pos: float = 0.0,
    z_pos: float = 0.0,
) -> cq.Workplane:
    left = (
        cq.Workplane("XZ")
        .workplane(offset=y_offset)
        .circle(radius)
        .extrude(thickness)
    )
    right = (
        cq.Workplane("XZ")
        .workplane(offset=-(y_offset + thickness))
        .circle(radius)
        .extrude(thickness)
    )
    return left.union(right).translate((x_pos, 0.0, z_pos))


def _make_base_fork_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(
        BASE_PLATE_L,
        BASE_PLATE_W,
        BASE_PLATE_T,
        centered=(True, True, False),
    )
    column = (
        cq.Workplane("XY")
        .box(
            BASE_COLUMN_L,
            BASE_COLUMN_W,
            BASE_COLUMN_H,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, BASE_PLATE_T))
    )
    cheek_y = FORK_GAP / 2.0 + FORK_CHEEK_T / 2.0
    cheek_left = cq.Workplane("XY").box(0.058, FORK_CHEEK_T, FORK_CHEEK_H).translate(
        (0.014, cheek_y, SHOULDER_Z)
    )
    cheek_right = cq.Workplane("XY").box(0.058, FORK_CHEEK_T, FORK_CHEEK_H).translate(
        (0.014, -cheek_y, SHOULDER_Z)
    )
    base = plate.union(column).union(cheek_left).union(cheek_right)

    hole_positions = [
        (-0.074, -0.050),
        (-0.074, 0.050),
        (0.074, -0.050),
        (0.074, 0.050),
    ]
    for x_pos, y_pos in hole_positions:
        hole = (
            cq.Workplane("XY")
            .workplane(offset=-0.001)
            .center(x_pos, y_pos)
            .circle(0.009)
            .extrude(BASE_PLATE_T + 0.004)
        )
        base = base.cut(hole)

    return base


def _make_upper_arm_shape() -> cq.Workplane:
    shoulder_barrel = cq.Workplane("XZ").circle(UPPER_SHOULDER_RADIUS).extrude(FORK_GAP / 2.0, both=True)
    shoulder_block = cq.Workplane("XY").box(0.062, FORK_GAP, 0.052).translate((0.032, 0.0, 0.0))
    beam = (
        cq.Workplane("YZ")
        .workplane(offset=0.045)
        .rect(0.044, 0.056)
        .workplane(offset=0.149)
        .rect(0.032, 0.042)
        .loft(combine=True)
    )
    elbow_neck = cq.Workplane("XY").box(0.024, 0.036, 0.038).translate((0.196, 0.0, 0.0))
    elbow_head = cq.Workplane("XY").box(0.054, UPPER_ELBOW_GAP + 2.0 * UPPER_ELBOW_CHEEK_T, 0.046).translate(
        (UPPER_ARM_LENGTH - 0.003, 0.0, 0.0)
    )
    elbow_slot = cq.Workplane("XY").box(0.046, UPPER_ELBOW_GAP, 0.050).translate((UPPER_ARM_LENGTH + 0.001, 0.0, 0.0))
    return shoulder_barrel.union(shoulder_block).union(beam).union(elbow_neck).union(elbow_head).cut(elbow_slot)


def _make_forearm_shape() -> cq.Workplane:
    elbow_barrel = cq.Workplane("XZ").circle(FOREARM_ELBOW_RADIUS).extrude(UPPER_ELBOW_GAP / 2.0, both=True)
    elbow_block = cq.Workplane("XY").box(0.046, UPPER_ELBOW_GAP, 0.044).translate((0.022, 0.0, 0.0))
    beam = (
        cq.Workplane("YZ")
        .workplane(offset=0.034)
        .rect(0.034, 0.046)
        .workplane(offset=0.116)
        .rect(0.026, 0.034)
        .loft(combine=True)
    )
    wrist_neck = cq.Workplane("XY").box(0.022, 0.030, 0.030).translate((0.154, 0.0, 0.0))
    wrist_head = cq.Workplane("XY").box(0.046, FOREARM_WRIST_GAP + 2.0 * FOREARM_WRIST_CHEEK_T, 0.036).translate(
        (0.182, 0.0, 0.0)
    )
    wrist_slot = cq.Workplane("XY").box(0.037, FOREARM_WRIST_GAP, 0.040).translate((0.1865, 0.0, 0.0))
    return elbow_barrel.union(elbow_block).union(beam).union(wrist_neck).union(wrist_head).cut(wrist_slot)


def _make_wrist_flange_shape() -> cq.Workplane:
    barrel = cq.Workplane("XZ").circle(WRIST_BARREL_RADIUS).extrude(FOREARM_WRIST_GAP / 2.0, both=True)
    neck = (
        cq.Workplane("YZ")
        .circle(0.022)
        .extrude(0.058)
        .translate((0.0, 0.0, 0.0))
    )
    flange_disk = cq.Workplane("YZ").workplane(offset=WRIST_BODY_LENGTH - WRIST_FLANGE_T).circle(
        WRIST_FLANGE_RADIUS
    ).extrude(WRIST_FLANGE_T)
    flange = barrel.union(neck).union(flange_disk)

    hole_radius = 0.004
    bolt_circle = 0.022
    for angle_deg in (45.0, 135.0, 225.0, 315.0):
        angle = pi * angle_deg / 180.0
        y_pos = bolt_circle * cos(angle)
        z_pos = bolt_circle * sin(angle)
        bolt_hole = (
            cq.Workplane("YZ")
            .workplane(offset=WRIST_BODY_LENGTH - WRIST_FLANGE_T - 0.002)
            .center(y_pos, z_pos)
            .circle(hole_radius)
            .extrude(WRIST_FLANGE_T + 0.004)
        )
        flange = flange.cut(bolt_hole)

    center_bore = (
        cq.Workplane("YZ")
        .workplane(offset=WRIST_BODY_LENGTH - WRIST_FLANGE_T - 0.002)
        .circle(0.009)
        .extrude(WRIST_FLANGE_T + 0.004)
    )
    return flange.cut(center_bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_root_three_joint_arm")

    model.material("powder_charcoal", rgba=(0.20, 0.21, 0.24, 1.0))
    model.material("machined_aluminum", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("tool_steel", rgba=(0.55, 0.58, 0.62, 1.0))

    base_fork = model.part("base_fork")
    base_fork.visual(
        mesh_from_cadquery(_make_base_fork_shape(), "base_fork"),
        material="powder_charcoal",
        name="base_fork_body",
    )
    base_fork.inertial = Inertial.from_geometry(
        Box((BASE_PLATE_L, BASE_PLATE_W, SHOULDER_Z + 0.05)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, (SHOULDER_Z + 0.05) / 2.0)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        mesh_from_cadquery(_make_upper_arm_shape(), "upper_arm"),
        material="machined_aluminum",
        name="upper_arm_body",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((UPPER_ARM_LENGTH + 0.024, 0.060, 0.060)),
        mass=2.0,
        origin=Origin(xyz=((UPPER_ARM_LENGTH + 0.024) / 2.0, 0.0, 0.0)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        mesh_from_cadquery(_make_forearm_shape(), "forearm"),
        material="machined_aluminum",
        name="forearm_body",
    )
    forearm.inertial = Inertial.from_geometry(
        Box((FOREARM_LENGTH + 0.020, 0.046, 0.050)),
        mass=1.2,
        origin=Origin(xyz=((FOREARM_LENGTH + 0.020) / 2.0, 0.0, 0.0)),
    )

    wrist_flange = model.part("wrist_flange")
    wrist_flange.visual(
        mesh_from_cadquery(_make_wrist_flange_shape(), "wrist_flange"),
        material="tool_steel",
        name="wrist_assembly",
    )
    wrist_flange.inertial = Inertial.from_geometry(
        Box((WRIST_BODY_LENGTH, 0.074, 0.074)),
        mass=0.45,
        origin=Origin(xyz=(WRIST_BODY_LENGTH / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent=base_fork,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.70, upper=1.20, effort=60.0, velocity=1.2),
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.00, upper=1.55, effort=40.0, velocity=1.5),
    )
    model.articulation(
        "wrist_joint",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_flange,
        origin=Origin(xyz=(FOREARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.10, upper=1.10, effort=18.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_fork = object_model.get_part("base_fork")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist_flange = object_model.get_part("wrist_flange")
    shoulder_joint = object_model.get_articulation("shoulder_joint")
    elbow_joint = object_model.get_articulation("elbow_joint")
    wrist_joint = object_model.get_articulation("wrist_joint")

    def aabb_center_z(aabb):
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

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
    ctx.allow_overlap(
        base_fork,
        upper_arm,
        elem_a="base_fork_body",
        elem_b="upper_arm_body",
        reason="The shoulder yoke is represented as a simplified captured knuckle without explicit pin bores or split bearing hardware.",
    )
    ctx.allow_overlap(
        upper_arm,
        forearm,
        elem_a="upper_arm_body",
        elem_b="forearm_body",
        reason="The elbow clevis is modeled as a simplified interlocking knuckle so the joint reads mechanically without separate pin and bushing parts.",
    )
    ctx.allow_overlap(
        forearm,
        wrist_flange,
        elem_a="forearm_body",
        elem_b="wrist_assembly",
        reason="The wrist clevis uses the same simplified solid-knuckle representation instead of explicit pin and bearing cavities.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        base_fork,
        upper_arm,
        contact_tol=5e-4,
        name="shoulder barrel is captured by the grounded fork",
    )
    ctx.expect_contact(
        upper_arm,
        forearm,
        contact_tol=5e-4,
        name="elbow barrel is supported by the upper-arm clevis",
    )
    ctx.expect_contact(
        forearm,
        wrist_flange,
        contact_tol=5e-4,
        name="wrist barrel is supported by the forearm clevis",
    )

    ctx.expect_origin_gap(
        forearm,
        upper_arm,
        axis="x",
        min_gap=UPPER_ARM_LENGTH - 0.001,
        max_gap=UPPER_ARM_LENGTH + 0.001,
        name="elbow joint sits at the upper-arm tip",
    )
    ctx.expect_origin_gap(
        wrist_flange,
        forearm,
        axis="x",
        min_gap=FOREARM_LENGTH - 0.001,
        max_gap=FOREARM_LENGTH + 0.001,
        name="wrist joint sits at the forearm tip",
    )

    wrist_rest = ctx.part_element_world_aabb(wrist_flange, elem="wrist_assembly")
    wrist_rest_z = aabb_center_z(wrist_rest)

    with ctx.pose({shoulder_joint: 0.75, elbow_joint: 0.0, wrist_joint: 0.0}):
        wrist_raised = ctx.part_element_world_aabb(wrist_flange, elem="wrist_assembly")
        wrist_raised_z = aabb_center_z(wrist_raised)
        ctx.check(
            "positive shoulder rotation raises the arm",
            wrist_rest_z is not None and wrist_raised_z is not None and wrist_raised_z > wrist_rest_z + 0.12,
            details=f"rest_z={wrist_rest_z}, raised_z={wrist_raised_z}",
        )

    with ctx.pose({shoulder_joint: 0.0, elbow_joint: 0.90, wrist_joint: 0.0}):
        elbow_lifted = ctx.part_element_world_aabb(wrist_flange, elem="wrist_assembly")
        elbow_lifted_z = aabb_center_z(elbow_lifted)
        ctx.check(
            "positive elbow rotation lifts the distal chain",
            wrist_rest_z is not None and elbow_lifted_z is not None and elbow_lifted_z > wrist_rest_z + 0.05,
            details=f"rest_z={wrist_rest_z}, elbow_z={elbow_lifted_z}",
        )

    with ctx.pose({shoulder_joint: 0.0, elbow_joint: 0.0, wrist_joint: 0.80}):
        wrist_pitched = ctx.part_element_world_aabb(wrist_flange, elem="wrist_assembly")
        wrist_pitched_z = aabb_center_z(wrist_pitched)
        ctx.check(
            "positive wrist rotation tips the flange upward",
            wrist_rest_z is not None and wrist_pitched_z is not None and wrist_pitched_z > wrist_rest_z + 0.015,
            details=f"rest_z={wrist_rest_z}, wrist_z={wrist_pitched_z}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
