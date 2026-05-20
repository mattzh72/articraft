from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_DEPTH = 0.078
BODY_WIDTH = 0.115
BODY_REAR_HEIGHT = 0.0185
BODY_FRONT_HEIGHT = 0.0115

HINGE_X = -0.033
HINGE_Z = 0.025
HINGE_RADIUS = 0.0058
BODY_EAR_LENGTH = 0.018
ARM_KNUCKLE_LENGTH = 0.074
EAR_CENTER_Y = 0.046

HOLE_X = 0.008
HOLE_SPACING = 0.080
HOLE_RADIUS = 0.0038
COLLAR_RADIUS = 0.0062
COLLAR_HEIGHT = 0.0018
COLLAR_Z = 0.0142

TRAY_DEPTH = 0.050
TRAY_WIDTH = 0.102
TRAY_HEIGHT = 0.0075
TRAY_WALL = 0.0015
TRAY_FLOOR = 0.0015
TRAY_X = 0.006
TRAY_Z = 0.0045
TRAY_TRAVEL = 0.042
TRAY_OPENING_Y = (BODY_WIDTH / 2.0) - 0.006
CAVITY_DEPTH = TRAY_DEPTH + 0.004
CAVITY_WIDTH = TRAY_WIDTH + 0.004
CAVITY_HEIGHT = TRAY_HEIGHT + 0.002

ARM_LENGTH = 0.112
ARM_WIDTH = 0.109
ARM_REAR_WIDTH = 0.070
ARM_OPEN_LIMIT = 1.12
PUNCH_RADIUS = 0.0026
PUNCH_LENGTH = 0.0015
PUNCH_X = HOLE_X - HINGE_X


def _body_shell_shape() -> cq.Workplane:
    shell = (
        cq.Workplane("XZ")
        .moveTo(-BODY_DEPTH / 2.0, 0.0)
        .lineTo(BODY_DEPTH / 2.0, 0.0)
        .lineTo(BODY_DEPTH / 2.0, BODY_FRONT_HEIGHT)
        .lineTo(0.020, 0.0145)
        .lineTo(-0.004, 0.0162)
        .lineTo(-0.028, BODY_REAR_HEIGHT)
        .lineTo(-BODY_DEPTH / 2.0, BODY_REAR_HEIGHT)
        .close()
        .extrude(BODY_WIDTH / 2.0, both=True)
    )

    die_collars = (
        cq.Workplane("XY")
        .pushPoints([(HOLE_X, -HOLE_SPACING / 2.0), (HOLE_X, HOLE_SPACING / 2.0)])
        .circle(COLLAR_RADIUS)
        .extrude(COLLAR_HEIGHT)
        .translate((0.0, 0.0, COLLAR_Z))
    )

    tray_cavity = (
        cq.Workplane("XY")
        .center(TRAY_X, 0.0)
        .box(CAVITY_DEPTH, CAVITY_WIDTH, CAVITY_HEIGHT, centered=(True, True, False))
        .translate((0.0, 0.0, TRAY_Z - 0.001))
    )
    side_opening = (
        cq.Workplane("XY")
        .center(TRAY_X, BODY_WIDTH / 2.0 + 0.012)
        .box(CAVITY_DEPTH + 0.004, 0.036, CAVITY_HEIGHT + 0.004, centered=(True, True, False))
        .translate((0.0, 0.0, TRAY_Z - 0.002))
    )
    punch_holes = (
        cq.Workplane("XY")
        .pushPoints([(HOLE_X, -HOLE_SPACING / 2.0), (HOLE_X, HOLE_SPACING / 2.0)])
        .circle(HOLE_RADIUS)
        .extrude(BODY_REAR_HEIGHT + 0.010)
    )

    return shell.union(die_collars).cut(tray_cavity).cut(side_opening).cut(punch_holes)


def _body_hinge_shape() -> cq.Workplane:
    left_ear = (
        cq.Workplane("XZ")
        .center(HINGE_X, HINGE_Z)
        .circle(HINGE_RADIUS + 0.0008)
        .extrude(BODY_EAR_LENGTH / 2.0, both=True)
        .translate((0.0, -EAR_CENTER_Y, 0.0))
    )
    right_ear = (
        cq.Workplane("XZ")
        .center(HINGE_X, HINGE_Z)
        .circle(HINGE_RADIUS + 0.0008)
        .extrude(BODY_EAR_LENGTH / 2.0, both=True)
        .translate((0.0, EAR_CENTER_Y, 0.0))
    )
    return left_ear.union(right_ear)


def _arm_shell_shape() -> cq.Workplane:
    rear_shell = (
        cq.Workplane("XZ")
        .moveTo(-0.003, -0.001)
        .lineTo(0.006, 0.010)
        .lineTo(0.020, 0.017)
        .lineTo(0.034, 0.019)
        .lineTo(0.040, 0.013)
        .lineTo(0.040, -0.002)
        .lineTo(0.008, -0.0025)
        .close()
        .extrude(ARM_REAR_WIDTH / 2.0, both=True)
    )
    front_cover = (
        cq.Workplane("XZ")
        .moveTo(0.028, -0.0018)
        .lineTo(0.042, 0.011)
        .lineTo(0.060, 0.021)
        .lineTo(0.082, 0.018)
        .lineTo(ARM_LENGTH, 0.010)
        .lineTo(ARM_LENGTH, 0.0015)
        .lineTo(0.094, -0.0005)
        .lineTo(0.044, -0.0025)
        .close()
        .extrude(ARM_WIDTH / 2.0, both=True)
    )

    knuckle = (
        cq.Workplane("XZ")
        .center(0.0, 0.0)
        .circle(HINGE_RADIUS)
        .extrude(ARM_KNUCKLE_LENGTH / 2.0, both=True)
    )
    punch_pins = (
        cq.Workplane("XY")
        .workplane(offset=0.001)
        .pushPoints([(PUNCH_X, -HOLE_SPACING / 2.0), (PUNCH_X, HOLE_SPACING / 2.0)])
        .circle(PUNCH_RADIUS)
        .extrude(-PUNCH_LENGTH)
    )

    return rear_shell.union(front_cover).union(punch_pins)


def _arm_hinge_shape() -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center(0.0, 0.0)
        .circle(HINGE_RADIUS)
        .extrude(ARM_KNUCKLE_LENGTH / 2.0, both=True)
    )


def _tray_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .center(0.0, -TRAY_WIDTH / 2.0)
        .box(TRAY_DEPTH, TRAY_WIDTH, TRAY_HEIGHT, centered=(True, True, False))
    )
    inner = (
        cq.Workplane("XY")
        .center(0.0, -TRAY_WIDTH / 2.0)
        .box(
            TRAY_DEPTH - (2.0 * TRAY_WALL),
            TRAY_WIDTH - (2.0 * TRAY_WALL),
            TRAY_HEIGHT - TRAY_FLOOR,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, TRAY_FLOOR))
    )
    pull_tab = (
        cq.Workplane("XY")
        .center(0.0, 0.005)
        .box(0.020, 0.014, 0.006, centered=(True, True, False))
        .translate((0.0, 0.0, 0.001))
    )
    guide_rib = (
        cq.Workplane("XY")
        .center(0.0, -0.010)
        .box(0.034, 0.008, 0.0018, centered=(True, True, False))
        .translate((0.0, 0.0, TRAY_HEIGHT - 0.0008))
    )

    return outer.cut(inner).union(pull_tab).union(guide_rib)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_two_hole_punch")

    model.material("body_finish", rgba=(0.27, 0.29, 0.31, 1.0))
    model.material("arm_finish", rgba=(0.09, 0.10, 0.11, 1.0))
    model.material("tray_finish", rgba=(0.45, 0.47, 0.50, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_shape(), "body_shell"),
        material="body_finish",
        name="body_shell",
    )
    body.visual(
        mesh_from_cadquery(_body_hinge_shape(), "body_hinge"),
        material="body_finish",
        name="body_hinge",
    )

    arm = model.part("arm")
    arm.visual(
        mesh_from_cadquery(_arm_shell_shape(), "arm_shell"),
        material="arm_finish",
        name="arm_shell",
    )
    arm.visual(
        mesh_from_cadquery(_arm_hinge_shape(), "arm_hinge"),
        material="arm_finish",
        name="arm_hinge",
    )

    tray = model.part("tray")
    tray.visual(
        mesh_from_cadquery(_tray_shape(), "tray_shell"),
        material="tray_finish",
        name="tray_shell",
    )

    model.articulation(
        "arm_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=arm,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=ARM_OPEN_LIMIT, effort=25.0, velocity=2.4),
    )
    model.articulation(
        "tray_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(TRAY_X, TRAY_OPENING_Y, TRAY_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=TRAY_TRAVEL, effort=6.0, velocity=0.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    arm = object_model.get_part("arm")
    tray = object_model.get_part("tray")
    arm_hinge = object_model.get_articulation("arm_hinge")
    tray_slide = object_model.get_articulation("tray_slide")

    ctx.allow_overlap(
        arm,
        body,
        elem_a="arm_hinge",
        elem_b="body_hinge",
        reason="The rear hinge uses interleaved closed-solid barrels instead of modeling the internal hinge pin and clearance bore.",
    )

    with ctx.pose({arm_hinge: 0.0, tray_slide: 0.0}):
        ctx.expect_contact(
            arm,
            body,
            contact_tol=0.0005,
            elem_a="arm_hinge",
            elem_b="body_hinge",
            name="top arm stays carried by the rear hinge hardware",
        )
        ctx.expect_overlap(
            arm,
            body,
            axes="xy",
            elem_a="arm_shell",
            elem_b="body_shell",
            min_overlap=0.050,
            name="closed arm covers the punch body footprint",
        )
        ctx.expect_within(
            tray,
            body,
            axes="xz",
            inner_elem="tray_shell",
            outer_elem="body_shell",
            margin=0.004,
            name="closed tray stays under the punch head",
        )
        ctx.expect_overlap(
            tray,
            body,
            axes="y",
            elem_a="tray_shell",
            elem_b="body_shell",
            min_overlap=0.070,
            name="closed tray remains fully inserted in the body guide",
        )

    closed_arm_aabb = ctx.part_element_world_aabb(arm, elem="arm_shell")
    with ctx.pose({arm_hinge: ARM_OPEN_LIMIT}):
        open_arm_aabb = ctx.part_element_world_aabb(arm, elem="arm_shell")
    ctx.check(
        "top arm opens upward around the rear hinge",
        closed_arm_aabb is not None
        and open_arm_aabb is not None
        and open_arm_aabb[1][2] > closed_arm_aabb[1][2] + 0.040,
        details=f"closed={closed_arm_aabb}, open={open_arm_aabb}",
    )

    closed_tray_pos = ctx.part_world_position(tray)
    with ctx.pose({tray_slide: TRAY_TRAVEL}):
        ctx.expect_within(
            tray,
            body,
            axes="xz",
            inner_elem="tray_shell",
            outer_elem="body_shell",
            margin=0.004,
            name="extended tray stays aligned with the lateral guide",
        )
        ctx.expect_overlap(
            tray,
            body,
            axes="y",
            elem_a="tray_shell",
            elem_b="body_shell",
            min_overlap=0.050,
            name="extended tray retains insertion in the body guide",
        )
        extended_tray_pos = ctx.part_world_position(tray)
    ctx.check(
        "side tray pulls outward to the right",
        closed_tray_pos is not None
        and extended_tray_pos is not None
        and extended_tray_pos[1] > closed_tray_pos[1] + 0.030,
        details=f"closed={closed_tray_pos}, extended={extended_tray_pos}",
    )

    return ctx.report()


object_model = build_object_model()
