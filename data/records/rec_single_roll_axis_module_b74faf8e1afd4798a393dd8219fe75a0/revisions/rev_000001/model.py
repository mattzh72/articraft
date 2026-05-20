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


BASE_LENGTH = 0.19
BASE_DEPTH = 0.12
BASE_THICKNESS = 0.016

PEDESTAL_LENGTH = 0.084
PEDESTAL_DEPTH = 0.060
PEDESTAL_HEIGHT = 0.028
PEDESTAL_EMBED = 0.001

ARM_THICKNESS = 0.016
ARM_DEPTH = 0.040
ARM_HEIGHT = 0.074
ARM_EMBED = 0.001
ARM_CENTER_X = 0.031

SHAFT_RADIUS = 0.006
BEARING_BORE_RADIUS = SHAFT_RADIUS
SLOT_WIDTH = 0.018

SHAFT_Z = BASE_THICKNESS + PEDESTAL_HEIGHT + 0.045
ARM_BOTTOM_Z = BASE_THICKNESS + PEDESTAL_HEIGHT - ARM_EMBED

SHAFT_LENGTH = 0.089
SHAFT_CENTER_X = 0.0025

FLANGE_THICKNESS = 0.006
FLANGE_RADIUS = 0.018
FLANGE_CENTER_X = 0.045


def _fork_arm_shape(side: int) -> cq.Workplane:
    arm = cq.Workplane("XY").box(
        ARM_THICKNESS,
        ARM_DEPTH,
        ARM_HEIGHT,
        centered=(True, True, False),
    )
    slot_floor_z = SHAFT_Z - ARM_BOTTOM_Z - SHAFT_RADIUS
    slot_cut = (
        cq.Workplane("XY")
        .box(
            ARM_THICKNESS + 0.006,
            SLOT_WIDTH,
            ARM_HEIGHT - slot_floor_z + 0.004,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, slot_floor_z))
    )

    return arm.cut(slot_cut)


def _flange_shape() -> cq.Workplane:
    off_axis_hole_y = 0.0075
    hole_radius = 0.0022

    flange = (
        cq.Workplane("YZ")
        .circle(FLANGE_RADIUS)
        .extrude(FLANGE_THICKNESS)
        .translate((-FLANGE_THICKNESS / 2.0, 0.0, 0.0))
    )
    hole = (
        cq.Workplane("YZ")
        .center(off_axis_hole_y, 0.0)
        .circle(hole_radius)
        .extrude(FLANGE_THICKNESS + 0.002)
        .translate((-FLANGE_THICKNESS / 2.0 - 0.001, 0.0, 0.0))
    )
    return flange.cut(hole)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_supported_roll_spindle")

    model.material("frame_paint", rgba=(0.26, 0.28, 0.31, 1.0))
    model.material("steel", rgba=(0.70, 0.72, 0.75, 1.0))

    fork_frame = model.part("fork_frame")
    fork_frame.visual(
        Box((BASE_LENGTH, BASE_DEPTH, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material="frame_paint",
        name="base_plate",
    )
    fork_frame.visual(
        Box((PEDESTAL_LENGTH, PEDESTAL_DEPTH, PEDESTAL_HEIGHT + PEDESTAL_EMBED)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                BASE_THICKNESS + (PEDESTAL_HEIGHT + PEDESTAL_EMBED) / 2.0,
            )
        ),
        material="frame_paint",
        name="pedestal",
    )
    fork_frame.visual(
        mesh_from_cadquery(_fork_arm_shape(-1), "left_fork_arm"),
        origin=Origin(xyz=(-ARM_CENTER_X, 0.0, ARM_BOTTOM_Z)),
        material="frame_paint",
        name="left_arm",
    )
    fork_frame.visual(
        mesh_from_cadquery(_fork_arm_shape(1), "right_fork_arm"),
        origin=Origin(xyz=(ARM_CENTER_X, 0.0, ARM_BOTTOM_Z)),
        material="frame_paint",
        name="right_arm",
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=SHAFT_RADIUS, length=SHAFT_LENGTH),
        origin=Origin(
            xyz=(SHAFT_CENTER_X, 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material="steel",
        name="shaft",
    )
    spindle.visual(
        mesh_from_cadquery(_flange_shape(), "spindle_flange"),
        origin=Origin(xyz=(FLANGE_CENTER_X, 0.0, 0.0)),
        material="steel",
        name="flange",
    )

    model.articulation(
        "fork_frame_to_spindle",
        ArticulationType.REVOLUTE,
        parent=fork_frame,
        child=spindle,
        origin=Origin(xyz=(0.0, 0.0, SHAFT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-2.0 * pi,
            upper=2.0 * pi,
            effort=5.0,
            velocity=16.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    fork_frame = object_model.get_part("fork_frame")
    spindle = object_model.get_part("spindle")
    spin_joint = object_model.get_articulation("fork_frame_to_spindle")

    ctx.check(
        "fork spindle parts exist",
        fork_frame is not None and spindle is not None and spin_joint is not None,
        details="Expected fork_frame root, spindle child, and one spinning articulation.",
    )

    limits = spin_joint.motion_limits
    ctx.check(
        "spindle joint rotates about supported shaft axis",
        tuple(round(v, 4) for v in spin_joint.axis) == (1.0, 0.0, 0.0)
        and limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower < 0.0 < limits.upper,
        details=(
            f"axis={spin_joint.axis}, "
            f"limits={(None if limits is None else (limits.lower, limits.upper))}"
        ),
    )

    ctx.expect_gap(
        spindle,
        fork_frame,
        axis="x",
        positive_elem="flange",
        negative_elem="right_arm",
        min_gap=0.001,
        max_gap=0.004,
        name="flange sits just outside the right fork arm",
    )

    left_arm_aabb = ctx.part_element_world_aabb(fork_frame, elem="left_arm")
    right_arm_aabb = ctx.part_element_world_aabb(fork_frame, elem="right_arm")
    shaft_aabb = ctx.part_element_world_aabb(spindle, elem="shaft")

    supported_between_arms = False
    if left_arm_aabb is not None and right_arm_aabb is not None and shaft_aabb is not None:
        shaft_center_x = 0.5 * (shaft_aabb[0][0] + shaft_aabb[1][0])
        shaft_center_y = 0.5 * (shaft_aabb[0][1] + shaft_aabb[1][1])
        shaft_center_z = 0.5 * (shaft_aabb[0][2] + shaft_aabb[1][2])
        supported_between_arms = (
            left_arm_aabb[1][0] < shaft_center_x < right_arm_aabb[0][0]
            and shaft_aabb[0][0] <= left_arm_aabb[0][0] + 0.003
            and shaft_aabb[1][0] >= right_arm_aabb[1][0] + 0.003
            and left_arm_aabb[0][1] <= shaft_center_y <= left_arm_aabb[1][1]
            and right_arm_aabb[0][1] <= shaft_center_y <= right_arm_aabb[1][1]
            and left_arm_aabb[0][2] <= shaft_center_z <= left_arm_aabb[1][2]
            and right_arm_aabb[0][2] <= shaft_center_z <= right_arm_aabb[1][2]
        )

    ctx.check(
        "shaft axis is cradled between both fork arms",
        supported_between_arms,
        details=(
            f"left_arm_aabb={left_arm_aabb}, "
            f"right_arm_aabb={right_arm_aabb}, shaft_aabb={shaft_aabb}"
        ),
    )

    with ctx.pose({spin_joint: 1.3}):
        ctx.expect_gap(
            spindle,
            fork_frame,
            axis="x",
            positive_elem="flange",
            negative_elem="right_arm",
            min_gap=0.001,
            max_gap=0.004,
            name="rotated flange still clears the fork arm",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
