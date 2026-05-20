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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


SHOULDER_SHAFT_RADIUS = 0.0075
SHOULDER_HOLE_RADIUS = 0.0082
SHOULDER_FLANGE_RADIUS = 0.015

ELBOW_SHAFT_RADIUS = 0.0065
ELBOW_HOLE_RADIUS = 0.0072
ELBOW_FLANGE_RADIUS = 0.013

ROOT_FORK_GAP = 0.024
ROOT_FORK_CHEEK_THICKNESS = 0.008
ROOT_FORK_OUTER_WIDTH = ROOT_FORK_GAP + 2.0 * ROOT_FORK_CHEEK_THICKNESS

ELBOW_FORK_GAP = 0.022
ELBOW_FORK_CHEEK_THICKNESS = 0.008
ELBOW_FORK_OUTER_WIDTH = ELBOW_FORK_GAP + 2.0 * ELBOW_FORK_CHEEK_THICKNESS

UPPER_ARM_LENGTH = 0.280
FOREARM_LENGTH = 0.254


def _box_x(
    length: float,
    width_y: float,
    height_z: float,
    x0: float,
    *,
    y0: float = 0.0,
    z0: float = 0.0,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(length, width_y, height_z, centered=(False, True, True))
        .translate((x0, y0, z0))
    )


def _box_centered(
    length_x: float,
    width_y: float,
    height_z: float,
    *,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
) -> cq.Workplane:
    return cq.Workplane("XY").box(length_x, width_y, height_z).translate((x, y, z))


def _tube_x(
    length: float,
    outer_y: float,
    outer_z: float,
    wall: float,
    x0: float,
) -> cq.Workplane:
    outer = _box_x(length, outer_y, outer_z, x0)
    inner = _box_x(
        length + 0.002,
        outer_y - 2.0 * wall,
        outer_z - 2.0 * wall,
        x0 - 0.001,
    )
    return outer.cut(inner)


def _y_cylinder(
    radius: float,
    length: float,
    *,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center(x, z)
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .translate((0.0, y, 0.0))
    )


def _root_fork_shape() -> cq.Workplane:
    base_plate = _box_x(0.074, 0.108, 0.014, -0.094, z0=-0.055)
    rear_bridge = _box_x(0.028, ROOT_FORK_OUTER_WIDTH, 0.036, -0.064, z0=-0.040)
    left_cheek = _box_x(
        0.058,
        ROOT_FORK_CHEEK_THICKNESS,
        0.072,
        -0.056,
        y0=ROOT_FORK_GAP / 2.0 + ROOT_FORK_CHEEK_THICKNESS / 2.0,
    )
    right_cheek = _box_x(
        0.058,
        ROOT_FORK_CHEEK_THICKNESS,
        0.072,
        -0.056,
        y0=-(ROOT_FORK_GAP / 2.0 + ROOT_FORK_CHEEK_THICKNESS / 2.0),
    )
    shoulder_pin = _y_cylinder(
        SHOULDER_SHAFT_RADIUS,
        ROOT_FORK_OUTER_WIDTH,
        x=0.0,
        z=0.0,
    )

    return (
        base_plate.union(rear_bridge)
        .union(left_cheek)
        .union(right_cheek)
        .union(shoulder_pin)
    )


def _upper_arm_shape() -> cq.Workplane:
    shoulder_lug = _box_x(0.020, ROOT_FORK_GAP - 0.004, 0.030, -0.006)
    shoulder_neck = _box_x(0.018, 0.018, 0.030, 0.010)
    shoulder_bore = _y_cylinder(
        SHOULDER_SHAFT_RADIUS,
        ROOT_FORK_GAP + 0.004,
        x=0.0,
        z=0.0,
    )
    beam_outer = _box_x(0.196, 0.030, 0.044, 0.028)
    beam_inner = _box_x(0.178, 0.020, 0.028, 0.037)
    elbow_rear_bridge = _box_x(0.018, ELBOW_FORK_OUTER_WIDTH, 0.024, 0.232, z0=-0.020)
    left_elbow_cheek = _box_x(
        0.046,
        ELBOW_FORK_CHEEK_THICKNESS,
        0.046,
        UPPER_ARM_LENGTH - 0.046,
        y0=ELBOW_FORK_GAP / 2.0 + ELBOW_FORK_CHEEK_THICKNESS / 2.0,
    )
    right_elbow_cheek = _box_x(
        0.046,
        ELBOW_FORK_CHEEK_THICKNESS,
        0.046,
        UPPER_ARM_LENGTH - 0.046,
        y0=-(ELBOW_FORK_GAP / 2.0 + ELBOW_FORK_CHEEK_THICKNESS / 2.0),
    )
    elbow_pin = _y_cylinder(
        ELBOW_SHAFT_RADIUS,
        ELBOW_FORK_OUTER_WIDTH,
        x=UPPER_ARM_LENGTH,
        z=0.0,
    )

    return (
        shoulder_lug.union(shoulder_neck)
        .union(beam_outer)
        .cut(beam_inner)
        .cut(shoulder_bore)
        .union(elbow_rear_bridge)
        .union(left_elbow_cheek)
        .union(right_elbow_cheek)
        .union(elbow_pin)
    )


def _forearm_shape() -> cq.Workplane:
    elbow_lug = _box_x(0.020, ELBOW_FORK_GAP - 0.004, 0.028, -0.006)
    elbow_neck = _box_x(0.016, 0.016, 0.028, 0.010)
    elbow_bore = _y_cylinder(
        ELBOW_SHAFT_RADIUS,
        ELBOW_FORK_GAP + 0.004,
        x=0.0,
        z=0.0,
    )
    beam_outer = _box_x(0.192, 0.026, 0.038, 0.026)
    beam_inner = _box_x(0.174, 0.016, 0.022, 0.035)
    plate_stem = _box_x(0.028, 0.032, 0.048, 0.218)
    end_plate = _box_x(0.012, 0.090, 0.078, 0.242)

    return (
        elbow_lug.union(elbow_neck)
        .union(beam_outer)
        .cut(beam_inner)
        .cut(elbow_bore)
        .union(plate_stem)
        .union(end_plate)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_root_elbow_module")

    model.material("powder_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("anodized_aluminum", rgba=(0.72, 0.75, 0.78, 1.0))
    model.material("machined_aluminum", rgba=(0.80, 0.82, 0.84, 1.0))

    root_fork = model.part("root_fork")
    root_fork.visual(
        Box((0.090, 0.110, 0.014)),
        material="powder_steel",
        origin=Origin(xyz=(-0.045, 0.0, -0.055)),
        name="base_plate",
    )
    root_fork.visual(
        Box((0.024, ROOT_FORK_OUTER_WIDTH, 0.034)),
        material="powder_steel",
        origin=Origin(xyz=(-0.048, 0.0, -0.031)),
        name="rear_bridge",
    )
    root_fork.visual(
        Box((0.060, ROOT_FORK_CHEEK_THICKNESS, 0.074)),
        material="powder_steel",
        origin=Origin(
            xyz=(-0.030, ROOT_FORK_GAP / 2.0 + ROOT_FORK_CHEEK_THICKNESS / 2.0, 0.0)
        ),
        name="left_cheek",
    )
    root_fork.visual(
        Box((0.060, ROOT_FORK_CHEEK_THICKNESS, 0.074)),
        material="powder_steel",
        origin=Origin(
            xyz=(-0.030, -(ROOT_FORK_GAP / 2.0 + ROOT_FORK_CHEEK_THICKNESS / 2.0), 0.0)
        ),
        name="right_cheek",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Box((0.016, ROOT_FORK_GAP, 0.030)),
        material="anodized_aluminum",
        origin=Origin(xyz=(0.008, 0.0, 0.0)),
        name="shoulder_lug",
    )
    upper_arm.visual(
        Box((0.018, 0.020, 0.032)),
        material="anodized_aluminum",
        origin=Origin(xyz=(0.025, 0.0, 0.0)),
        name="shoulder_neck",
    )
    upper_arm.visual(
        Box((0.196, 0.030, 0.044)),
        material="anodized_aluminum",
        origin=Origin(xyz=(0.132, 0.0, 0.0)),
        name="main_beam",
    )
    upper_arm.visual(
        Box((0.018, ELBOW_FORK_OUTER_WIDTH, 0.024)),
        material="anodized_aluminum",
        origin=Origin(xyz=(0.239, 0.0, -0.014)),
        name="elbow_bridge",
    )
    upper_arm.visual(
        Box((0.050, ELBOW_FORK_CHEEK_THICKNESS, 0.046)),
        material="anodized_aluminum",
        origin=Origin(
            xyz=(
                UPPER_ARM_LENGTH - 0.025,
                ELBOW_FORK_GAP / 2.0 + ELBOW_FORK_CHEEK_THICKNESS / 2.0,
                0.0,
            )
        ),
        name="left_elbow_cheek",
    )
    upper_arm.visual(
        Box((0.050, ELBOW_FORK_CHEEK_THICKNESS, 0.046)),
        material="anodized_aluminum",
        origin=Origin(
            xyz=(
                UPPER_ARM_LENGTH - 0.025,
                -(ELBOW_FORK_GAP / 2.0 + ELBOW_FORK_CHEEK_THICKNESS / 2.0),
                0.0,
            )
        ),
        name="right_elbow_cheek",
    )

    forearm = model.part("forearm")
    forearm.visual(
        Box((0.014, ELBOW_FORK_GAP, 0.028)),
        material="machined_aluminum",
        origin=Origin(xyz=(0.007, 0.0, 0.0)),
        name="elbow_lug",
    )
    forearm.visual(
        Box((0.016, 0.018, 0.030)),
        material="machined_aluminum",
        origin=Origin(xyz=(0.022, 0.0, 0.0)),
        name="elbow_neck",
    )
    forearm.visual(
        Box((0.190, 0.026, 0.038)),
        material="machined_aluminum",
        origin=Origin(xyz=(0.125, 0.0, 0.0)),
        name="forearm_beam",
    )
    forearm.visual(
        Box((0.022, 0.036, 0.052)),
        material="machined_aluminum",
        origin=Origin(xyz=(0.231, 0.0, 0.0)),
        name="plate_stem",
    )
    forearm.visual(
        Box((0.012, 0.090, 0.078)),
        material="machined_aluminum",
        origin=Origin(xyz=(0.248, 0.0, 0.0)),
        name="end_plate",
    )

    model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent=root_fork,
        child=upper_arm,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.8,
            lower=-0.45,
            upper=1.20,
        ),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=28.0,
            velocity=2.0,
            lower=0.0,
            upper=2.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root_fork = object_model.get_part("root_fork")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    shoulder = object_model.get_articulation("shoulder_pitch")
    elbow = object_model.get_articulation("elbow_pitch")

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
        "joint_axes_match_pitch_module",
        tuple(shoulder.axis) == (0.0, -1.0, 0.0)
        and tuple(elbow.axis) == (0.0, -1.0, 0.0)
        and shoulder.motion_limits is not None
        and elbow.motion_limits is not None
        and isclose(shoulder.motion_limits.lower or 0.0, -0.45, abs_tol=1e-9)
        and isclose(shoulder.motion_limits.upper or 0.0, 1.20, abs_tol=1e-9)
        and isclose(elbow.motion_limits.lower or 0.0, 0.0, abs_tol=1e-9)
        and isclose(elbow.motion_limits.upper or 0.0, 2.15, abs_tol=1e-9),
        details="Shoulder and elbow should both pitch about local -Y with the authored limits.",
    )

    ctx.expect_contact(
        root_fork,
        upper_arm,
        contact_tol=1e-5,
        name="root_fork_physically_supports_upper_arm",
    )
    ctx.expect_contact(
        upper_arm,
        forearm,
        contact_tol=1e-5,
        name="upper_arm_physically_supports_forearm",
    )

    with ctx.pose({shoulder: 0.0, elbow: 0.0}):
        ctx.expect_overlap(
            root_fork,
            upper_arm,
            axes="yz",
            min_overlap=0.028,
            name="upper_arm_root_sits_inside_root_fork_envelope",
        )
        ctx.expect_overlap(
            upper_arm,
            forearm,
            axes="yz",
            min_overlap=0.024,
            name="forearm_root_sits_inside_upper_arm_fork_envelope",
        )
        upper_closed = ctx.part_world_aabb(upper_arm)
        forearm_closed = ctx.part_world_aabb(forearm)

    with ctx.pose({shoulder: 0.75, elbow: 0.0}):
        upper_lifted = ctx.part_world_aabb(upper_arm)

    with ctx.pose({shoulder: 0.0, elbow: 1.15}):
        forearm_bent = ctx.part_world_aabb(forearm)

    shoulder_ok = (
        upper_closed is not None
        and upper_lifted is not None
        and upper_lifted[1][2] > upper_closed[1][2] + 0.10
    )
    forearm_ok = (
        forearm_closed is not None
        and forearm_bent is not None
        and forearm_bent[1][2] > forearm_closed[1][2] + 0.10
    )

    ctx.check(
        "positive_shoulder_rotation_lifts_upper_arm",
        shoulder_ok,
        details="Positive shoulder q should rotate the upper arm upward around the fork-supported shoulder axis.",
    )
    ctx.check(
        "positive_elbow_rotation_lifts_forearm",
        forearm_ok,
        details="Positive elbow q should rotate the forearm upward around the upper-arm-supported elbow axis.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
