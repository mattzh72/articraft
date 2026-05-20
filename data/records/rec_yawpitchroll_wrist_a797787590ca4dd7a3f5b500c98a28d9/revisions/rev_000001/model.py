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


BASE_LEN = 0.180
BASE_WIDTH = 0.120
BASE_THICK = 0.018

YAW_AXIS_Z = 0.100
PITCH_OFFSET = (0.031, 0.000, 0.019)
ROLL_OFFSET = (0.044, 0.000, 0.000)

YAW_LIMIT = 1.50
PITCH_LOWER = -0.85
PITCH_UPPER = 1.15
ROLL_LIMIT = 1.70


def _support_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(
        BASE_LEN, BASE_WIDTH, BASE_THICK, centered=(True, True, False)
    )

    cheek = cq.Workplane("XY").box(
        0.082, 0.014, 0.076, centered=(True, True, False)
    ).translate((-0.008, -0.047, BASE_THICK))

    arm = cq.Workplane("XY").box(
        0.070, 0.042, 0.012, centered=(True, True, False)
    ).translate((0.000, -0.021, 0.082))

    seat = cq.Workplane("XY").circle(0.038).extrude(0.006).translate((0.000, 0.000, 0.094))

    gusset = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.042, BASE_THICK),
                (-0.042, 0.072),
                (0.010, 0.082),
                (0.018, BASE_THICK),
            ]
        )
        .close()
        .extrude(0.010)
        .translate((0.000, -0.041, 0.000))
    )

    rear_pad = cq.Workplane("XY").box(
        0.052, 0.060, 0.010, centered=(True, True, False)
    ).translate((-0.048, 0.000, BASE_THICK))

    return base.union(cheek).union(arm).union(seat).union(gusset).union(rear_pad)


def _yaw_cartridge_shape() -> cq.Workplane:
    base_flange = cq.Workplane("XY").circle(0.036).extrude(0.006)
    drum = cq.Workplane("XY").circle(0.028).extrude(0.026).translate((0.000, 0.000, 0.006))
    top_cap = cq.Workplane("XY").circle(0.022).extrude(0.005).translate((0.000, 0.000, 0.032))

    clevis_web = cq.Workplane("XY").box(
        0.014, 0.030, 0.018, centered=(False, True, False)
    ).translate((0.010, 0.000, 0.010))
    left_ear = cq.Workplane("XY").box(
        0.018, 0.010, 0.026, centered=(False, True, False)
    ).translate((0.013, 0.017, 0.006))
    right_ear = cq.Workplane("XY").box(
        0.018, 0.010, 0.026, centered=(False, True, False)
    ).translate((0.013, -0.017, 0.006))

    service_pod = cq.Workplane("YZ").circle(0.010).extrude(0.014).translate(
        (-0.030, 0.000, 0.020)
    )

    return (
        base_flange.union(drum)
        .union(top_cap)
        .union(clevis_web)
        .union(left_ear)
        .union(right_ear)
        .union(service_pod)
    )


def _pitch_yoke_shape() -> cq.Workplane:
    rear_trunnion = cq.Workplane("XY").box(
        0.012, 0.024, 0.014, centered=(True, True, True)
    )
    fork_blank = cq.Workplane("XY").box(
        0.050, 0.050, 0.018, centered=(False, True, True)
    ).translate((0.006, 0.000, 0.000))
    fork_slot = cq.Workplane("XY").box(
        0.038, 0.032, 0.020, centered=(False, True, True)
    ).translate((0.018, 0.000, 0.000))

    return rear_trunnion.union(fork_blank.cut(fork_slot))


def _roll_nose_shape() -> cq.Workplane:
    rear_mount = cq.Workplane("XY").box(
        0.020, 0.032, 0.014, centered=(True, True, True)
    )

    nose_body = cq.Workplane("YZ").circle(0.012).extrude(0.034).translate((0.004, 0.000, 0.000))
    taper = (
        cq.Workplane("YZ")
        .circle(0.012)
        .workplane(offset=0.044)
        .circle(0.004)
        .loft(combine=True)
        .translate((0.008, 0.000, 0.000))
    )

    dorsal_fin = cq.Workplane("XY").box(
        0.022, 0.006, 0.012, centered=(True, True, False)
    ).translate((0.028, 0.000, 0.007))

    chin_pad = cq.Workplane("XY").box(
        0.014, 0.010, 0.006, centered=(True, True, False)
    ).translate((0.018, 0.000, -0.010))

    return rear_mount.union(nose_body).union(taper).union(dorsal_fin).union(chin_pad)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_cheek_yaw_pitch_roll_module")

    model.material("support_gray", rgba=(0.36, 0.39, 0.42, 1.0))
    model.material("cartridge_silver", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("yoke_dark", rgba=(0.21, 0.23, 0.26, 1.0))
    model.material("nose_black", rgba=(0.13, 0.14, 0.16, 1.0))

    support = model.part("side_support")
    support.visual(
        mesh_from_cadquery(_support_shape(), "side_support"),
        material="support_gray",
        name="support_body",
    )
    support.inertial = Inertial.from_geometry(
        Box((BASE_LEN, BASE_WIDTH, 0.130)),
        mass=5.2,
        origin=Origin(xyz=(0.000, 0.000, 0.065)),
    )

    yaw_cartridge = model.part("yaw_cartridge")
    yaw_cartridge.visual(
        mesh_from_cadquery(_yaw_cartridge_shape(), "yaw_cartridge"),
        material="cartridge_silver",
        name="cartridge_body",
    )
    yaw_cartridge.inertial = Inertial.from_geometry(
        Cylinder(radius=0.036, length=0.040),
        mass=0.95,
        origin=Origin(xyz=(0.000, 0.000, 0.020)),
    )

    pitch_yoke = model.part("pitch_yoke")
    pitch_yoke.visual(
        mesh_from_cadquery(_pitch_yoke_shape(), "pitch_yoke"),
        material="yoke_dark",
        name="yoke_body",
    )
    pitch_yoke.inertial = Inertial.from_geometry(
        Box((0.052, 0.056, 0.024)),
        mass=0.42,
        origin=Origin(xyz=(0.024, 0.000, 0.000)),
    )

    roll_nose = model.part("roll_nose")
    roll_nose.visual(
        mesh_from_cadquery(_roll_nose_shape(), "roll_nose"),
        material="nose_black",
        name="nose_body",
    )
    roll_nose.inertial = Inertial.from_geometry(
        Cylinder(radius=0.015, length=0.082),
        mass=0.36,
        origin=Origin(xyz=(0.030, 0.000, 0.000), rpy=(0.000, pi / 2.0, 0.000)),
    )

    model.articulation(
        "support_to_yaw",
        ArticulationType.REVOLUTE,
        parent=support,
        child=yaw_cartridge,
        origin=Origin(xyz=(0.000, 0.000, YAW_AXIS_Z)),
        axis=(0.000, 0.000, 1.000),
        motion_limits=MotionLimits(
            lower=-YAW_LIMIT,
            upper=YAW_LIMIT,
            effort=24.0,
            velocity=1.6,
        ),
    )
    model.articulation(
        "yaw_to_pitch",
        ArticulationType.REVOLUTE,
        parent=yaw_cartridge,
        child=pitch_yoke,
        origin=Origin(xyz=PITCH_OFFSET),
        axis=(0.000, -1.000, 0.000),
        motion_limits=MotionLimits(
            lower=PITCH_LOWER,
            upper=PITCH_UPPER,
            effort=12.0,
            velocity=1.8,
        ),
    )
    model.articulation(
        "pitch_to_roll",
        ArticulationType.REVOLUTE,
        parent=pitch_yoke,
        child=roll_nose,
        origin=Origin(xyz=ROLL_OFFSET),
        axis=(1.000, 0.000, 0.000),
        motion_limits=MotionLimits(
            lower=-ROLL_LIMIT,
            upper=ROLL_LIMIT,
            effort=8.0,
            velocity=2.4,
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

    support = object_model.get_part("side_support")
    yaw_cartridge = object_model.get_part("yaw_cartridge")
    pitch_yoke = object_model.get_part("pitch_yoke")
    roll_nose = object_model.get_part("roll_nose")

    yaw_joint = object_model.get_articulation("support_to_yaw")
    pitch_joint = object_model.get_articulation("yaw_to_pitch")
    roll_joint = object_model.get_articulation("pitch_to_roll")

    ctx.check(
        "all module parts resolve",
        all(part is not None for part in (support, yaw_cartridge, pitch_yoke, roll_nose)),
        details="Expected side support, yaw cartridge, pitch yoke, and roll nose.",
    )
    ctx.check(
        "all revolute joints resolve",
        all(joint is not None for joint in (yaw_joint, pitch_joint, roll_joint)),
        details="Expected yaw, pitch, and roll revolute articulations.",
    )

    ctx.allow_overlap(
        yaw_cartridge,
        pitch_yoke,
        reason=(
            "The pitch stage is modeled as a compact captive trunnion seated inside the yaw-cartridge ears, "
            "so the hidden bearing engagement is represented by a slight proxy overlap."
        ),
    )
    ctx.allow_overlap(
        pitch_yoke,
        roll_nose,
        reason=(
            "The roll nose uses a simplified retained bearing block nested between the pitch-yoke arms, "
            "so the concealed bearing engagement is represented by a slight proxy overlap."
        ),
    )

    ctx.expect_origin_distance(
        yaw_cartridge,
        support,
        axes="xy",
        max_dist=0.002,
        name="yaw axis stays centered over the grounded support",
    )
    ctx.expect_origin_gap(
        yaw_cartridge,
        support,
        axis="z",
        min_gap=0.098,
        max_gap=0.102,
        name="yaw cartridge sits above the support base",
    )
    ctx.expect_origin_gap(
        pitch_yoke,
        yaw_cartridge,
        axis="x",
        min_gap=0.026,
        max_gap=0.032,
        name="pitch yoke is mounted forward of the yaw axis",
    )
    ctx.expect_origin_gap(
        roll_nose,
        pitch_yoke,
        axis="x",
        min_gap=0.042,
        max_gap=0.046,
        name="roll nose is mounted forward of the pitch axis",
    )

    rest_pitch_pos = ctx.part_world_position(pitch_yoke)
    rest_roll_pos = ctx.part_world_position(roll_nose)
    rest_roll_aabb = ctx.part_world_aabb(roll_nose)

    with ctx.pose({yaw_joint: 0.65}):
        yawed_pitch_pos = ctx.part_world_position(pitch_yoke)

    ctx.check(
        "yaw joint slews the forward stages laterally",
        rest_pitch_pos is not None
        and yawed_pitch_pos is not None
        and yawed_pitch_pos[1] > rest_pitch_pos[1] + 0.015
        and abs(yawed_pitch_pos[2] - rest_pitch_pos[2]) < 0.003,
        details=f"rest_pitch={rest_pitch_pos}, yawed_pitch={yawed_pitch_pos}",
    )

    with ctx.pose({pitch_joint: 0.75}):
        pitched_roll_pos = ctx.part_world_position(roll_nose)

    ctx.check(
        "pitch joint raises the roll nose",
        rest_roll_pos is not None
        and pitched_roll_pos is not None
        and pitched_roll_pos[2] > rest_roll_pos[2] + 0.020
        and pitched_roll_pos[0] < rest_roll_pos[0] - 0.005,
        details=f"rest_roll={rest_roll_pos}, pitched_roll={pitched_roll_pos}",
    )

    with ctx.pose({roll_joint: 1.35}):
        rolled_aabb = ctx.part_world_aabb(roll_nose)

    ctx.check(
        "roll joint rotates the asymmetric nose body",
        rest_roll_aabb is not None
        and rolled_aabb is not None
        and rolled_aabb[0][1] < rest_roll_aabb[0][1] - 0.004,
        details=f"rest_aabb={rest_roll_aabb}, rolled_aabb={rolled_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
