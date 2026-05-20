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


FOOT_SIZE = 0.18
FOOT_THICKNESS = 0.022
FOOT_CORNER_RADIUS = 0.014
MOUNT_HOLE_DIAMETER = 0.012
MOUNT_HOLE_OFFSET = 0.062

SHOULDER_RADIUS = 0.055
SHOULDER_HEIGHT = 0.010
PEDESTAL_RADIUS = 0.024
PEDESTAL_HEIGHT = 0.082
BEARING_COLLAR_RADIUS = 0.036
BEARING_COLLAR_HEIGHT = 0.012

JOINT_Z = FOOT_THICKNESS + SHOULDER_HEIGHT + PEDESTAL_HEIGHT + BEARING_COLLAR_HEIGHT

HEAD_BODY_RADIUS = 0.041
HEAD_BODY_HEIGHT = 0.024
HEAD_NECK_WIDTH = 0.050
HEAD_NECK_DEPTH = 0.046
HEAD_NECK_HEIGHT = 0.014
SENSOR_PLATE_SIZE = 0.110
SENSOR_PLATE_THICKNESS = 0.010

DRIVE_POD_SIZE = (0.030, 0.025, 0.022)
DRIVE_POD_CENTER = (0.047, 0.0, 0.017)
SENSOR_FACE_SIZE = (0.072, 0.072, 0.003)
SENSOR_FACE_CENTER_Z = HEAD_BODY_HEIGHT + HEAD_NECK_HEIGHT + SENSOR_PLATE_THICKNESS + SENSOR_FACE_SIZE[2] / 2.0


def _build_base_shape() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .box(FOOT_SIZE, FOOT_SIZE, FOOT_THICKNESS, centered=(True, True, False))
        .edges("|Z")
        .fillet(FOOT_CORNER_RADIUS)
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-MOUNT_HOLE_OFFSET, -MOUNT_HOLE_OFFSET),
                (-MOUNT_HOLE_OFFSET, MOUNT_HOLE_OFFSET),
                (MOUNT_HOLE_OFFSET, -MOUNT_HOLE_OFFSET),
                (MOUNT_HOLE_OFFSET, MOUNT_HOLE_OFFSET),
            ]
        )
        .hole(MOUNT_HOLE_DIAMETER)
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .circle(SHOULDER_RADIUS)
        .extrude(SHOULDER_HEIGHT)
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .circle(PEDESTAL_RADIUS)
        .extrude(PEDESTAL_HEIGHT)
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .circle(BEARING_COLLAR_RADIUS)
        .extrude(BEARING_COLLAR_HEIGHT)
    )
    return base


def _build_head_shape() -> cq.Workplane:
    head = (
        cq.Workplane("XY")
        .circle(HEAD_BODY_RADIUS)
        .extrude(HEAD_BODY_HEIGHT)
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .rect(HEAD_NECK_WIDTH, HEAD_NECK_DEPTH)
        .extrude(HEAD_NECK_HEIGHT)
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .rect(SENSOR_PLATE_SIZE, SENSOR_PLATE_SIZE)
        .extrude(SENSOR_PLATE_THICKNESS)
        .edges(">Z")
        .fillet(0.004)
    )
    return head


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    return tuple((lo + hi) / 2.0 for lo, hi in zip(aabb[0], aabb[1]))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mast_top_pan_module")

    model.material("foot_gray", rgba=(0.26, 0.28, 0.30, 1.0))
    model.material("machined_silver", rgba=(0.75, 0.77, 0.80, 1.0))
    model.material("actuator_dark", rgba=(0.17, 0.18, 0.20, 1.0))
    model.material("sensor_glass", rgba=(0.14, 0.22, 0.30, 0.90))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_shape(), "pan_module_base"),
        material="foot_gray",
        name="base_shell",
    )
    base.inertial = Inertial.from_geometry(
        Box((FOOT_SIZE, FOOT_SIZE, JOINT_Z)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, JOINT_Z / 2.0)),
    )

    rotary_head = model.part("rotary_head")
    rotary_head.visual(
        mesh_from_cadquery(_build_head_shape(), "pan_module_head"),
        material="machined_silver",
        name="head_shell",
    )
    rotary_head.visual(
        Box(DRIVE_POD_SIZE),
        origin=Origin(xyz=DRIVE_POD_CENTER),
        material="actuator_dark",
        name="drive_pod",
    )
    rotary_head.visual(
        Box(SENSOR_FACE_SIZE),
        origin=Origin(xyz=(0.0, 0.0, SENSOR_FACE_CENTER_Z)),
        material="sensor_glass",
        name="sensor_face",
    )
    rotary_head.inertial = Inertial.from_geometry(
        Box((0.122, 0.122, SENSOR_FACE_CENTER_Z + SENSOR_FACE_SIZE[2] / 2.0)),
        mass=0.92,
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
    )

    model.articulation(
        "pedestal_pan",
        ArticulationType.REVOLUTE,
        parent=base,
        child=rotary_head,
        origin=Origin(xyz=(0.0, 0.0, JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-pi, upper=pi, effort=14.0, velocity=2.6),
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

    base = object_model.get_part("base")
    rotary_head = object_model.get_part("rotary_head")
    pan = object_model.get_articulation("pedestal_pan")

    ctx.check(
        "pan articulation is vertical",
        pan.axis == (0.0, 0.0, 1.0)
        and pan.motion_limits is not None
        and pan.motion_limits.lower is not None
        and pan.motion_limits.upper is not None
        and pan.motion_limits.lower <= -3.0
        and pan.motion_limits.upper >= 3.0,
        details=f"axis={pan.axis}, limits={pan.motion_limits}",
    )

    with ctx.pose({pan: 0.0}):
        ctx.expect_gap(
            rotary_head,
            base,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            name="rotary head seats on the pedestal collar",
        )
        ctx.expect_overlap(
            rotary_head,
            base,
            axes="xy",
            min_overlap=0.10,
            name="rotary head stays centered over the heavy foot",
        )

        rest_pod_aabb = ctx.part_element_world_aabb(rotary_head, elem="drive_pod")

    with ctx.pose({pan: pi / 2.0}):
        turned_pod_aabb = ctx.part_element_world_aabb(rotary_head, elem="drive_pod")
        ctx.expect_gap(
            rotary_head,
            base,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            name="yaw motion keeps the head on the same bearing plane",
        )

    rest_center = _aabb_center(rest_pod_aabb)
    turned_center = _aabb_center(turned_pod_aabb)
    ctx.check(
        "drive pod swings around the pedestal axis",
        rest_center is not None
        and turned_center is not None
        and rest_center[0] > 0.03
        and abs(rest_center[1]) < 0.01
        and turned_center[1] > 0.03
        and abs(turned_center[0]) < 0.015,
        details=f"rest_center={rest_center}, turned_center={turned_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
