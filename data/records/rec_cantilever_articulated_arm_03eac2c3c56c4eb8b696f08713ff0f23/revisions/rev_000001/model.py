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


BASE_LENGTH = 0.180
BASE_WIDTH = 0.110
BASE_THICKNESS = 0.016

COLUMN_X = -0.045
COLUMN_WIDTH = 0.052
COLUMN_DEPTH = 0.050
COLUMN_HEIGHT = 0.086

SHOULDER_X = 0.002
SHOULDER_Z = BASE_THICKNESS + COLUMN_HEIGHT + 0.018

JOINT_RADIUS = 0.018
FORK_OUTER_WIDTH = 0.046
CHEEK_THICKNESS = 0.009
HUB_LENGTH = FORK_OUTER_WIDTH - 2.0 * CHEEK_THICKNESS

LINK_THICKNESS = 0.028
UPPER_ARM_LENGTH = 0.205
FOREARM_LENGTH = 0.175
WRIST_NECK_LENGTH = 0.034
WRIST_FACE_RADIUS = 0.033
WRIST_FACE_THICKNESS = 0.012

SHOULDER_LIMITS = (-0.35, 1.10)
ELBOW_LIMITS = (-1.35, 1.20)
WRIST_LIMITS = (-1.05, 1.10)


def _cylinder_y(radius: float, length: float) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length).translate((0.0, length / 2.0, 0.0))


def _plate_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)
        .edges("|Z")
        .fillet(0.008)
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-0.055, -0.032),
                (-0.055, 0.032),
                (0.055, -0.032),
                (0.055, 0.032),
            ]
        )
        .slot2D(0.024, 0.009, angle=90.0)
        .cutThruAll()
        .translate((0.0, 0.0, BASE_THICKNESS / 2.0))
    )


def _base_shape() -> cq.Workplane:
    plate = _plate_shape()
    column = (
        cq.Workplane("XY")
        .box(COLUMN_WIDTH, COLUMN_DEPTH, COLUMN_HEIGHT)
        .edges("|Z")
        .fillet(0.006)
        .translate((COLUMN_X, 0.0, BASE_THICKNESS + COLUMN_HEIGHT / 2.0))
    )
    lower_anchor = cq.Workplane("XY").box(0.030, FORK_OUTER_WIDTH, 0.018).translate((SHOULDER_X - 0.010, 0.0, SHOULDER_Z - 0.028))
    left_ear = cq.Workplane("XY").box(0.034, CHEEK_THICKNESS, 0.050).translate((SHOULDER_X + 0.006, -0.0185, SHOULDER_Z))
    right_ear = cq.Workplane("XY").box(0.034, CHEEK_THICKNESS, 0.050).translate((SHOULDER_X + 0.006, 0.0185, SHOULDER_Z))
    center_gusset = (
        cq.Workplane("XZ")
        .polyline(
            [
                (COLUMN_X - 0.010, BASE_THICKNESS + 0.050),
                (SHOULDER_X - 0.016, SHOULDER_Z - 0.036),
                (SHOULDER_X + 0.004, SHOULDER_Z - 0.036),
                (COLUMN_X + 0.012, BASE_THICKNESS + 0.060),
            ]
        )
        .close()
        .extrude(0.020)
        .translate((0.0, -0.010, 0.0))
    )
    return plate.union(column).union(lower_anchor).union(left_ear).union(right_ear).union(center_gusset)


def _upper_arm_shape() -> cq.Workplane:
    hub = _cylinder_y(JOINT_RADIUS, HUB_LENGTH)
    root_neck = cq.Workplane("XY").box(0.026, 0.024, 0.022).translate((0.022, 0.0, 0.0))
    beam = cq.Workplane("XY").box(0.132, 0.034, LINK_THICKNESS).translate((0.099, 0.0, 0.0))
    elbow_block = cq.Workplane("XY").box(0.060, FORK_OUTER_WIDTH, 0.030).translate((UPPER_ARM_LENGTH - 0.011, 0.0, 0.0))
    elbow_slot = cq.Workplane("XY").box(0.060, HUB_LENGTH, 0.054).translate((UPPER_ARM_LENGTH + 0.002, 0.0, 0.0))
    return hub.union(root_neck).union(beam).union(elbow_block).cut(elbow_slot)


def _forearm_shape() -> cq.Workplane:
    hub = _cylinder_y(JOINT_RADIUS, HUB_LENGTH)
    root_neck = cq.Workplane("XY").box(0.024, 0.024, 0.022).translate((0.020, 0.0, 0.0))
    beam = cq.Workplane("XY").box(0.110, 0.032, 0.024).translate((0.085, 0.0, 0.0))
    wrist_block = cq.Workplane("XY").box(0.056, FORK_OUTER_WIDTH, 0.030).translate((FOREARM_LENGTH - 0.010, 0.0, 0.0))
    wrist_slot = cq.Workplane("XY").box(0.062, HUB_LENGTH, 0.054).translate((FOREARM_LENGTH + 0.004, 0.0, 0.0))
    return hub.union(root_neck).union(beam).union(wrist_block).cut(wrist_slot)


def _wrist_body_shape() -> cq.Workplane:
    hub = _cylinder_y(JOINT_RADIUS * 0.95, HUB_LENGTH)
    neck = cq.Workplane("XY").box(0.028, 0.022, 0.022).translate((0.020, 0.0, 0.0))
    return hub.union(neck)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_cantilever_arm")

    base_finish = model.material("base_finish", rgba=(0.20, 0.22, 0.24, 1.0))
    arm_finish = model.material("arm_finish", rgba=(0.69, 0.72, 0.75, 1.0))
    wrist_finish = model.material("wrist_finish", rgba=(0.32, 0.35, 0.38, 1.0))
    face_finish = model.material("face_finish", rgba=(0.78, 0.80, 0.83, 1.0))

    base = model.part("base")
    base.visual(mesh_from_cadquery(_base_shape(), "base"), material=base_finish, name="base_body")
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, SHOULDER_Z + 0.040)),
        mass=6.0,
        origin=Origin(xyz=(COLUMN_X * 0.35, 0.0, (SHOULDER_Z + 0.040) / 2.0)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(mesh_from_cadquery(_upper_arm_shape(), "upper_arm"), material=arm_finish, name="upper_arm_body")
    upper_arm.inertial = Inertial.from_geometry(
        Box((UPPER_ARM_LENGTH, FORK_OUTER_WIDTH, 0.040)),
        mass=1.2,
        origin=Origin(xyz=(UPPER_ARM_LENGTH / 2.0, 0.0, 0.0)),
    )

    forearm = model.part("forearm")
    forearm.visual(mesh_from_cadquery(_forearm_shape(), "forearm"), material=arm_finish, name="forearm_body")
    forearm.inertial = Inertial.from_geometry(
        Box((FOREARM_LENGTH, FORK_OUTER_WIDTH, 0.036)),
        mass=0.9,
        origin=Origin(xyz=(FOREARM_LENGTH / 2.0, 0.0, 0.0)),
    )

    wrist_face = model.part("wrist_face")
    wrist_face.visual(mesh_from_cadquery(_wrist_body_shape(), "wrist_body"), material=wrist_finish, name="wrist_body")
    wrist_face.visual(
        Cylinder(radius=WRIST_FACE_RADIUS, length=WRIST_FACE_THICKNESS),
        origin=Origin(
            xyz=(WRIST_NECK_LENGTH + WRIST_FACE_THICKNESS / 2.0, 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=face_finish,
        name="face_plate",
    )
    wrist_face.inertial = Inertial.from_geometry(
        Box((0.080, 0.070, 0.070)),
        mass=0.45,
        origin=Origin(xyz=(0.040, 0.0, 0.0)),
    )

    model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_arm,
        origin=Origin(xyz=(SHOULDER_X, 0.0, SHOULDER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=SHOULDER_LIMITS[0],
            upper=SHOULDER_LIMITS[1],
            effort=42.0,
            velocity=1.2,
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
            lower=ELBOW_LIMITS[0],
            upper=ELBOW_LIMITS[1],
            effort=28.0,
            velocity=1.6,
        ),
    )
    model.articulation(
        "wrist_pitch",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_face,
        origin=Origin(xyz=(FOREARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=WRIST_LIMITS[0],
            upper=WRIST_LIMITS[1],
            effort=12.0,
            velocity=2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist_face = object_model.get_part("wrist_face")
    shoulder = object_model.get_articulation("shoulder_pitch")
    elbow = object_model.get_articulation("elbow_pitch")
    wrist = object_model.get_articulation("wrist_pitch")

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
        base,
        upper_arm,
        reason="The shoulder is represented as a captured trunnion fit with simplified solid cheek geometry around the upper-arm hub.",
    )
    ctx.allow_overlap(
        upper_arm,
        forearm,
        reason="The elbow uses simplified captured-fork solids rather than a fully hollowed clevis, so the hinge fit is intentionally proxied.",
    )
    ctx.allow_overlap(
        forearm,
        wrist_face,
        reason="The wrist pivot is simplified as a captured hinge fit with proxy solids standing in for the real bearing clearance.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    expected_parts = {"base", "upper_arm", "forearm", "wrist_face"}
    expected_joints = {"shoulder_pitch", "elbow_pitch", "wrist_pitch"}
    ctx.check(
        "expected parts present",
        expected_parts.issubset({part.name for part in object_model.parts}),
        details=f"parts={sorted(part.name for part in object_model.parts)}",
    )
    ctx.check(
        "expected joints present",
        expected_joints.issubset({joint.name for joint in object_model.articulations}),
        details=f"joints={sorted(joint.name for joint in object_model.articulations)}",
    )

    ctx.expect_contact(base, upper_arm, name="base supports shoulder hub")
    ctx.expect_contact(upper_arm, forearm, name="upper arm supports forearm hub")
    ctx.expect_contact(forearm, wrist_face, name="forearm supports wrist hub")
    ctx.expect_gap(
        wrist_face,
        base,
        axis="x",
        min_gap=0.230,
        positive_elem="face_plate",
        name="wrist face clearly overhangs the base",
    )

    rest_wrist_pos = ctx.part_world_position(wrist_face)
    with ctx.pose({shoulder: SHOULDER_LIMITS[1]}):
        shoulder_lifted_pos = ctx.part_world_position(wrist_face)
    ctx.check(
        "positive shoulder rotation lifts the arm",
        rest_wrist_pos is not None
        and shoulder_lifted_pos is not None
        and shoulder_lifted_pos[2] > rest_wrist_pos[2] + 0.20,
        details=f"rest={rest_wrist_pos}, lifted={shoulder_lifted_pos}",
    )

    with ctx.pose({elbow: ELBOW_LIMITS[1]}):
        elbow_lifted_pos = ctx.part_world_position(wrist_face)
    ctx.check(
        "positive elbow rotation folds the forearm upward",
        rest_wrist_pos is not None
        and elbow_lifted_pos is not None
        and elbow_lifted_pos[2] > rest_wrist_pos[2] + 0.10,
        details=f"rest={rest_wrist_pos}, elbow={elbow_lifted_pos}",
    )

    def _aabb_center_z(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> float | None:
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    rest_face_z = _aabb_center_z(ctx.part_element_world_aabb(wrist_face, elem="face_plate"))
    with ctx.pose({wrist: WRIST_LIMITS[1]}):
        lifted_face_z = _aabb_center_z(ctx.part_element_world_aabb(wrist_face, elem="face_plate"))
    ctx.check(
        "positive wrist rotation tips the face upward",
        rest_face_z is not None and lifted_face_z is not None and lifted_face_z > rest_face_z + 0.015,
        details=f"rest_z={rest_face_z}, tipped_z={lifted_face_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
