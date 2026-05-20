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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_DEPTH = 0.20
BASE_WIDTH = 0.24
BASE_THICKNESS = 0.02

ARM_DEPTH = 0.13
ARM_THICKNESS = 0.02
ARM_HEIGHT = 0.18
ARM_INNER_SPAN = 0.14
ARM_CENTER_OFFSET = ARM_INNER_SPAN / 2.0 + ARM_THICKNESS / 2.0
ARM_FUSE_OVERLAP = 0.0015

BRIDGE_DEPTH = 0.05
BRIDGE_HEIGHT = 0.05
BRIDGE_WIDTH = ARM_INNER_SPAN + 0.008
BRIDGE_X = -0.04

AXIS_HEIGHT = 0.145
ARM_HOLE_RADIUS = 0.009

ROLL_RADIUS = 0.045
ROLL_LENGTH = 0.10
END_CAP_RADIUS = 0.052
END_CAP_THICKNESS = 0.008
SPINDLE_RADIUS = 0.0075
SPINDLE_HALF_LENGTH = ARM_CENTER_OFFSET + ARM_THICKNESS / 2.0
COLLAR_RADIUS = 0.013
COLLAR_THICKNESS = 0.003

DRIVE_LUG_SIZE = (0.012, 0.006, 0.020)
DRIVE_LUG_Y = ROLL_LENGTH / 2.0 + END_CAP_THICKNESS + DRIVE_LUG_SIZE[1] / 2.0
DRIVE_LUG_Z = 0.03


def _fork_arm_shape() -> cq.Workplane:
    axis_z_local = AXIS_HEIGHT - (BASE_THICKNESS + ARM_HEIGHT / 2.0 - ARM_FUSE_OVERLAP)

    arm = cq.Workplane("XY").box(ARM_DEPTH, ARM_THICKNESS, ARM_HEIGHT)
    hole = (
        cq.Workplane("XZ")
        .center(0.0, axis_z_local)
        .circle(ARM_HOLE_RADIUS)
        .extrude(ARM_THICKNESS + 0.01, both=True)
    )
    return arm.cut(hole).edges("|Z").fillet(0.004)


def _roll_shell_shape() -> cq.Workplane:
    return cq.Workplane("XZ").circle(ROLL_RADIUS).extrude(ROLL_LENGTH / 2.0, both=True)


def _end_cap_shape() -> cq.Workplane:
    cap = cq.Workplane("XZ").circle(END_CAP_RADIUS).extrude(END_CAP_THICKNESS / 2.0, both=True)
    positive = cap.translate((0.0, ROLL_LENGTH / 2.0 + END_CAP_THICKNESS / 2.0, 0.0))
    negative = cap.translate((0.0, -(ROLL_LENGTH / 2.0 + END_CAP_THICKNESS / 2.0), 0.0))
    return positive.union(negative)


def _spindle_shape() -> cq.Workplane:
    return cq.Workplane("XZ").circle(SPINDLE_RADIUS).extrude(SPINDLE_HALF_LENGTH, both=True)


def _retaining_collar_shape() -> cq.Workplane:
    collar = (
        cq.Workplane("XZ")
        .circle(COLLAR_RADIUS)
        .extrude(COLLAR_THICKNESS / 2.0, both=True)
        .cut(cq.Workplane("XZ").circle(SPINDLE_RADIUS).extrude(COLLAR_THICKNESS, both=True))
    )
    positive = collar.translate((0.0, SPINDLE_HALF_LENGTH + COLLAR_THICKNESS / 2.0, 0.0))
    negative = collar.translate((0.0, -(SPINDLE_HALF_LENGTH + COLLAR_THICKNESS / 2.0), 0.0))
    return positive.union(negative)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_supported_roll_cartridge")

    model.material("frame_paint", rgba=(0.25, 0.28, 0.31, 1.0))
    model.material("bridge_paint", rgba=(0.31, 0.34, 0.37, 1.0))
    model.material("roll_paper", rgba=(0.77, 0.70, 0.55, 1.0))
    model.material("end_cap_black", rgba=(0.16, 0.17, 0.19, 1.0))
    model.material("spindle_steel", rgba=(0.73, 0.75, 0.78, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((BASE_DEPTH, BASE_WIDTH, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material="frame_paint",
        name="base_plate",
    )
    frame.visual(
        Box((BRIDGE_DEPTH, BRIDGE_WIDTH, BRIDGE_HEIGHT)),
        origin=Origin(
            xyz=(
                BRIDGE_X,
                0.0,
                BASE_THICKNESS + BRIDGE_HEIGHT / 2.0 - ARM_FUSE_OVERLAP,
            )
        ),
        material="bridge_paint",
        name="rear_bridge",
    )
    frame.visual(
        mesh_from_cadquery(_fork_arm_shape(), "left_fork_arm"),
        origin=Origin(
            xyz=(
                0.0,
                -ARM_CENTER_OFFSET,
                BASE_THICKNESS + ARM_HEIGHT / 2.0 - ARM_FUSE_OVERLAP,
            )
        ),
        material="frame_paint",
        name="left_arm",
    )
    frame.visual(
        mesh_from_cadquery(_fork_arm_shape(), "right_fork_arm"),
        origin=Origin(
            xyz=(
                0.0,
                ARM_CENTER_OFFSET,
                BASE_THICKNESS + ARM_HEIGHT / 2.0 - ARM_FUSE_OVERLAP,
            )
        ),
        material="frame_paint",
        name="right_arm",
    )
    frame.inertial = Inertial.from_geometry(
        Box((BASE_DEPTH, BASE_WIDTH, BASE_THICKNESS + ARM_HEIGHT)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, (BASE_THICKNESS + ARM_HEIGHT) / 2.0)),
    )

    cartridge = model.part("cartridge")
    cartridge.visual(
        mesh_from_cadquery(_roll_shell_shape(), "cartridge_roll_shell"),
        material="roll_paper",
        name="roll_shell",
    )
    cartridge.visual(
        mesh_from_cadquery(_end_cap_shape(), "cartridge_end_caps"),
        material="end_cap_black",
        name="end_caps",
    )
    cartridge.visual(
        mesh_from_cadquery(_spindle_shape(), "cartridge_spindle"),
        material="spindle_steel",
        name="spindle",
    )
    cartridge.visual(
        mesh_from_cadquery(_retaining_collar_shape(), "retaining_collars"),
        material="spindle_steel",
        name="retaining_collars",
    )
    cartridge.visual(
        Box(DRIVE_LUG_SIZE),
        origin=Origin(xyz=(0.0, DRIVE_LUG_Y, DRIVE_LUG_Z)),
        material="end_cap_black",
        name="drive_lug",
    )
    cartridge.inertial = Inertial.from_geometry(
        Box((END_CAP_RADIUS * 2.0, SPINDLE_HALF_LENGTH * 2.0, END_CAP_RADIUS * 2.0)),
        mass=0.8,
        origin=Origin(),
    )

    model.articulation(
        "frame_to_cartridge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=cartridge,
        origin=Origin(xyz=(0.0, 0.0, AXIS_HEIGHT)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=6.0, lower=-pi, upper=pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    cartridge = object_model.get_part("cartridge")
    roll_joint = object_model.get_articulation("frame_to_cartridge")

    ctx.expect_gap(
        cartridge,
        frame,
        axis="y",
        positive_elem="end_caps",
        negative_elem="left_arm",
        min_gap=0.010,
        max_gap=0.014,
        name="left fork arm clears the cartridge end cap",
    )
    ctx.expect_gap(
        frame,
        cartridge,
        axis="y",
        positive_elem="right_arm",
        negative_elem="end_caps",
        min_gap=0.010,
        max_gap=0.014,
        name="right fork arm clears the cartridge end cap",
    )
    ctx.expect_overlap(
        frame,
        cartridge,
        axes="y",
        elem_a="left_arm",
        elem_b="spindle",
        min_overlap=0.018,
        name="left fork arm envelopes the supported spindle length",
    )
    ctx.expect_overlap(
        frame,
        cartridge,
        axes="y",
        elem_a="right_arm",
        elem_b="spindle",
        min_overlap=0.018,
        name="right fork arm envelopes the supported spindle length",
    )
    ctx.expect_gap(
        cartridge,
        frame,
        axis="z",
        positive_elem="roll_shell",
        negative_elem="base_plate",
        min_gap=0.075,
        name="roll cartridge is lifted above the grounded base",
    )

    axis_ok = all(abs(a - b) < 1e-9 for a, b in zip(roll_joint.axis, (0.0, 1.0, 0.0)))
    origin_ok = roll_joint.origin is not None and abs(roll_joint.origin.xyz[2] - AXIS_HEIGHT) < 1e-9
    limits = roll_joint.motion_limits
    limit_ok = (
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower < 0.0 < limits.upper
    )
    ctx.check(
        "cartridge joint sits on the supported roll axis",
        axis_ok and origin_ok and limit_ok,
        details=(
            f"axis={roll_joint.axis}, "
            f"origin={None if roll_joint.origin is None else roll_joint.origin.xyz}, "
            f"limits={None if limits is None else (limits.lower, limits.upper)}"
        ),
    )

    def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) / 2.0 for i in range(3))

    rest_lug = _aabb_center(ctx.part_element_world_aabb(cartridge, elem="drive_lug"))
    with ctx.pose({roll_joint: 1.1}):
        turned_lug = _aabb_center(ctx.part_element_world_aabb(cartridge, elem="drive_lug"))
        ctx.expect_gap(
            cartridge,
            frame,
            axis="z",
            positive_elem="drive_lug",
            negative_elem="base_plate",
            min_gap=0.080,
            name="rotated drive lug still clears the grounded base",
        )
    ctx.check(
        "cartridge revolution visibly moves the drive lug",
        rest_lug is not None
        and turned_lug is not None
        and abs(turned_lug[0] - rest_lug[0]) > 0.015
        and abs(turned_lug[2] - rest_lug[2]) > 0.010,
        details=f"rest_lug={rest_lug}, turned_lug={turned_lug}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
