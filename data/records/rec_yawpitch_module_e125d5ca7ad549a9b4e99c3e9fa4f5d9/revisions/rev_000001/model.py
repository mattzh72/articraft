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


BASE_LENGTH = 0.260
BASE_WIDTH = 0.200
BASE_THICKNESS = 0.018

PEDESTAL_BASE_LENGTH = 0.132
PEDESTAL_BASE_WIDTH = 0.106
PEDESTAL_TOP_LENGTH = 0.104
PEDESTAL_TOP_WIDTH = 0.082
PEDESTAL_HEIGHT = 0.062

BEARING_RADIUS = 0.074
BEARING_HEIGHT = 0.016
CENTER_BOSS_HEIGHT = 0.008
BEARING_RING_TOP_Z = BASE_THICKNESS + PEDESTAL_HEIGHT + BEARING_HEIGHT
YAW_AXIS_Z = BEARING_RING_TOP_Z + CENTER_BOSS_HEIGHT

YAW_PLATTER_RADIUS = 0.068
YAW_PLATTER_BOTTOM_GAP = 0.0
YAW_PLATTER_HEIGHT = 0.012
YAW_HUB_RADIUS = 0.034
YAW_HUB_HEIGHT = 0.026

FORK_ARM_LENGTH = 0.100
FORK_ARM_THICKNESS = 0.016
FORK_ARM_BOTTOM_Z = 0.010
FORK_ARM_HEIGHT = 0.112
FORK_ARM_Y = 0.055
FORK_GUSSET_LENGTH = 0.044
FORK_GUSSET_HEIGHT = 0.040

PITCH_AXIS_X = 0.014
PITCH_AXIS_Z = 0.094
PITCH_BOSS_RADIUS = 0.020
PITCH_HOLE_RADIUS = 0.0115

CRADLE_BODY_LENGTH = 0.088
CRADLE_BODY_WIDTH = 0.072
CRADLE_BODY_HEIGHT = 0.052
CRADLE_BODY_X = 0.018
CRADLE_NOSE_LENGTH = 0.022
CRADLE_NOSE_WIDTH = 0.060
CRADLE_NOSE_HEIGHT = 0.042
CRADLE_NOSE_X = 0.060
CRADLE_KEEL_LENGTH = 0.052
CRADLE_KEEL_WIDTH = 0.048
CRADLE_KEEL_HEIGHT = 0.022
CRADLE_KEEL_X = 0.022
CRADLE_KEEL_Z = -0.018
TRUNNION_RADIUS = 0.010
TRUNNION_LENGTH = 0.112
TRUNNION_COLLAR_RADIUS = 0.016
TRUNNION_COLLAR_LENGTH = 0.004
TRUNNION_COLLAR_Y = FORK_ARM_Y - FORK_ARM_THICKNESS / 2.0 - TRUNNION_COLLAR_LENGTH / 2.0

YAW_LIMIT = 2.6
PITCH_LOWER = -0.75
PITCH_UPPER = 1.00


def _lower_stage_shape() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.015)
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .rect(0.180, 0.120, forConstruction=True)
        .vertices()
        .circle(0.007)
        .cutThruAll()
    )

    pedestal = (
        cq.Workplane("XY")
        .workplane(offset=BASE_THICKNESS)
        .rect(PEDESTAL_BASE_LENGTH, PEDESTAL_BASE_WIDTH)
        .workplane(offset=PEDESTAL_HEIGHT)
        .rect(PEDESTAL_TOP_LENGTH, PEDESTAL_TOP_WIDTH)
        .loft(combine=True)
    )

    bearing_ring = (
        cq.Workplane("XY")
        .workplane(offset=BASE_THICKNESS + PEDESTAL_HEIGHT)
        .circle(BEARING_RADIUS)
        .extrude(BEARING_HEIGHT)
    )

    center_boss = (
        cq.Workplane("XY")
        .workplane(offset=BASE_THICKNESS + PEDESTAL_HEIGHT)
        .circle(0.040)
        .extrude(BEARING_HEIGHT + CENTER_BOSS_HEIGHT)
    )

    shape = base.union(pedestal).union(bearing_ring).union(center_boss)
    shape = (
        shape.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .circle(0.028)
        .cutBlind(-0.010)
    )
    return shape


def _fork_shape() -> cq.Workplane:
    platter = (
        cq.Workplane("XY")
        .workplane(offset=YAW_PLATTER_BOTTOM_GAP)
        .circle(YAW_PLATTER_RADIUS)
        .extrude(YAW_PLATTER_HEIGHT)
    )

    hub = (
        cq.Workplane("XY")
        .workplane(offset=YAW_PLATTER_BOTTOM_GAP)
        .circle(YAW_HUB_RADIUS)
        .extrude(YAW_HUB_HEIGHT)
    )

    arm_left = (
        cq.Workplane("XY")
        .box(
            FORK_ARM_LENGTH,
            FORK_ARM_THICKNESS,
            FORK_ARM_HEIGHT,
            centered=(True, True, False),
        )
        .translate((PITCH_AXIS_X, FORK_ARM_Y, FORK_ARM_BOTTOM_Z))
    )
    arm_right = (
        cq.Workplane("XY")
        .box(
            FORK_ARM_LENGTH,
            FORK_ARM_THICKNESS,
            FORK_ARM_HEIGHT,
            centered=(True, True, False),
        )
        .translate((PITCH_AXIS_X, -FORK_ARM_Y, FORK_ARM_BOTTOM_Z))
    )

    gusset_left = (
        cq.Workplane("XY")
        .box(
            FORK_GUSSET_LENGTH,
            FORK_ARM_THICKNESS,
            FORK_GUSSET_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.030, FORK_ARM_Y, YAW_PLATTER_BOTTOM_GAP + 0.002))
    )
    gusset_right = (
        cq.Workplane("XY")
        .box(
            FORK_GUSSET_LENGTH,
            FORK_ARM_THICKNESS,
            FORK_GUSSET_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.030, -FORK_ARM_Y, YAW_PLATTER_BOTTOM_GAP + 0.002))
    )

    shape = (
        platter.union(hub)
        .union(arm_left)
        .union(arm_right)
        .union(gusset_left)
        .union(gusset_right)
    )

    return shape


def _cradle_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(
            CRADLE_BODY_LENGTH,
            CRADLE_BODY_WIDTH,
            CRADLE_BODY_HEIGHT,
            centered=(True, True, True),
        )
        .translate((CRADLE_BODY_X, 0.0, 0.0))
        .edges("|Z")
        .fillet(0.008)
    )

    nose = (
        cq.Workplane("XY")
        .box(
            CRADLE_NOSE_LENGTH,
            CRADLE_NOSE_WIDTH,
            CRADLE_NOSE_HEIGHT,
            centered=(True, True, True),
        )
        .translate((CRADLE_NOSE_X, 0.0, -0.002))
        .edges("|Z")
        .fillet(0.004)
    )

    keel = (
        cq.Workplane("XY")
        .box(
            CRADLE_KEEL_LENGTH,
            CRADLE_KEEL_WIDTH,
            CRADLE_KEEL_HEIGHT,
            centered=(True, True, True),
        )
        .translate((CRADLE_KEEL_X, 0.0, CRADLE_KEEL_Z))
    )

    shape = body.union(nose).union(keel)
    shape = (
        shape.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .center(0.008, 0.0)
        .rect(0.046, 0.034)
        .cutBlind(-0.010)
    )
    return shape


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None, axis: int) -> float | None:
    if aabb is None:
        return None
    return 0.5 * (aabb[0][axis] + aabb[1][axis])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="turntable_fork_yaw_pitch_head")

    model.material("base_finish", rgba=(0.20, 0.22, 0.24, 1.0))
    model.material("fork_finish", rgba=(0.58, 0.60, 0.64, 1.0))
    model.material("cradle_finish", rgba=(0.80, 0.82, 0.85, 1.0))

    lower_stage = model.part("lower_stage")
    lower_stage.visual(
        mesh_from_cadquery(_lower_stage_shape(), "lower_stage_shell"),
        material="base_finish",
        name="lower_stage_shell",
    )
    lower_stage.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, YAW_AXIS_Z)),
        mass=6.5,
        origin=Origin(xyz=(0.0, 0.0, YAW_AXIS_Z / 2.0)),
    )

    fork = model.part("fork")
    fork.visual(
        mesh_from_cadquery(_fork_shape(), "fork_shell"),
        material="fork_finish",
        name="fork_shell",
    )
    fork.visual(
        Box((0.018, 0.098, 0.016)),
        origin=Origin(xyz=(0.056, 0.0, 0.048)),
        material="fork_finish",
        name="fork_nose",
    )
    fork.inertial = Inertial.from_geometry(
        Box((0.110, 0.140, 0.120)),
        mass=2.2,
        origin=Origin(xyz=(0.010, 0.0, 0.060)),
    )

    cradle = model.part("cradle")
    cradle.visual(
        mesh_from_cadquery(_cradle_shape(), "cradle_shell"),
        material="cradle_finish",
        name="cradle_shell",
    )
    cradle.visual(
        Box((0.024, 0.012, 0.024)),
        origin=Origin(xyz=(0.000, 0.041, 0.0)),
        material="cradle_finish",
        name="left_trunnion_stub",
    )
    cradle.visual(
        Box((0.024, 0.012, 0.024)),
        origin=Origin(xyz=(0.000, -0.041, 0.0)),
        material="cradle_finish",
        name="right_trunnion_stub",
    )
    cradle.visual(
        Box((0.016, 0.052, 0.030)),
        origin=Origin(xyz=(0.078, 0.0, -0.001)),
        material="cradle_finish",
        name="cradle_face",
    )
    cradle.inertial = Inertial.from_geometry(
        Box((0.104, TRUNNION_LENGTH, 0.060)),
        mass=1.0,
        origin=Origin(xyz=(0.018, 0.0, 0.0)),
    )

    model.articulation(
        "lower_stage_to_fork_yaw",
        ArticulationType.REVOLUTE,
        parent=lower_stage,
        child=fork,
        origin=Origin(xyz=(0.0, 0.0, YAW_AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.6,
            lower=-YAW_LIMIT,
            upper=YAW_LIMIT,
        ),
    )

    model.articulation(
        "fork_to_cradle_pitch",
        ArticulationType.REVOLUTE,
        parent=fork,
        child=cradle,
        origin=Origin(xyz=(PITCH_AXIS_X, 0.0, PITCH_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=PITCH_LOWER,
            upper=PITCH_UPPER,
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

    lower_stage = object_model.get_part("lower_stage")
    fork = object_model.get_part("fork")
    cradle = object_model.get_part("cradle")
    yaw = object_model.get_articulation("lower_stage_to_fork_yaw")
    pitch = object_model.get_articulation("fork_to_cradle_pitch")

    ctx.check(
        "yaw joint is vertical revolute",
        yaw.axis == (0.0, 0.0, 1.0)
        and yaw.motion_limits is not None
        and yaw.motion_limits.lower is not None
        and yaw.motion_limits.upper is not None
        and yaw.motion_limits.lower < 0.0 < yaw.motion_limits.upper,
        details=f"axis={yaw.axis}, limits={yaw.motion_limits}",
    )
    ctx.check(
        "pitch joint is horizontal revolute",
        pitch.axis == (0.0, -1.0, 0.0)
        and pitch.motion_limits is not None
        and pitch.motion_limits.lower is not None
        and pitch.motion_limits.upper is not None
        and pitch.motion_limits.lower < 0.0 < pitch.motion_limits.upper,
        details=f"axis={pitch.axis}, limits={pitch.motion_limits}",
    )
    ctx.allow_overlap(
        cradle,
        fork,
        elem_a="left_trunnion_stub",
        elem_b="fork_shell",
        reason="The left pitch stub is modeled as a simplified bearing pad seating directly against the fork arm mesh.",
    )
    ctx.allow_overlap(
        cradle,
        fork,
        elem_a="right_trunnion_stub",
        elem_b="fork_shell",
        reason="The right pitch stub is modeled as a simplified bearing pad seating directly against the fork arm mesh.",
    )

    with ctx.pose({yaw: 0.0, pitch: 0.0}):
        ctx.expect_overlap(
            fork,
            lower_stage,
            axes="xy",
            elem_a="fork_shell",
            elem_b="lower_stage_shell",
            min_overlap=0.120,
            name="fork remains centered above the turntable footprint",
        )
        ctx.expect_gap(
            fork,
            lower_stage,
            axis="z",
            positive_elem="fork_shell",
            negative_elem="lower_stage_shell",
            min_gap=0.0,
            max_gap=0.0005,
            name="upper and lower turntable races stay seated without lift-off",
        )
        ctx.expect_within(
            cradle,
            fork,
            axes="y",
            inner_elem="cradle_shell",
            outer_elem="fork_shell",
            margin=0.0,
            name="cradle stays between the fork arm envelope",
        )
        fork_rest = ctx.part_element_world_aabb(fork, elem="fork_nose")
        cradle_rest = ctx.part_element_world_aabb(cradle, elem="cradle_face")

    with ctx.pose({yaw: 0.9, pitch: 0.0}):
        fork_yawed = ctx.part_element_world_aabb(fork, elem="fork_nose")

    with ctx.pose({yaw: 0.0, pitch: 0.65}):
        cradle_pitched = ctx.part_element_world_aabb(cradle, elem="cradle_face")

    fork_rest_y = _aabb_center(fork_rest, 1)
    fork_yawed_y = _aabb_center(fork_yawed, 1)
    cradle_rest_z = _aabb_center(cradle_rest, 2)
    cradle_pitched_z = _aabb_center(cradle_pitched, 2)

    ctx.check(
        "positive yaw sweeps the fork toward +y",
        fork_rest_y is not None
        and fork_yawed_y is not None
        and fork_yawed_y > fork_rest_y + 0.006,
        details=f"rest_y={fork_rest_y}, yawed_y={fork_yawed_y}",
    )
    ctx.check(
        "positive pitch lifts the cradle nose upward",
        cradle_rest_z is not None
        and cradle_pitched_z is not None
        and cradle_pitched_z > cradle_rest_z + 0.008,
        details=f"rest_z={cradle_rest_z}, pitched_z={cradle_pitched_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
