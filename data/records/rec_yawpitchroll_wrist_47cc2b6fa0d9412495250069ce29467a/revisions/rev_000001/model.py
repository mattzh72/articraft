from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


BASE_LENGTH = 0.140
BASE_WIDTH = 0.110
BASE_THICKNESS = 0.016
BASE_GUSSET_THICKNESS = 0.012
PEDESTAL_RADIUS = 0.034
PEDESTAL_HEIGHT = 0.042
STATIONARY_COLLAR_RADIUS = 0.047
STATIONARY_COLLAR_HEIGHT = 0.012
YAW_ORIGIN_Z = BASE_THICKNESS + PEDESTAL_HEIGHT

YAW_DISC_RADIUS = 0.045
YAW_DISC_THICKNESS = 0.014
PITCH_ORIGIN_X = 0.048
PITCH_ORIGIN_Z = 0.050
YAW_LUG_INNER_HALF_GAP = 0.029
YAW_LUG_THICKNESS = 0.014

TRUNNION_RADIUS = 0.010
TRUNNION_LENGTH = 2.0 * YAW_LUG_INNER_HALF_GAP
CRADLE_ARM_LENGTH = 0.056
CRADLE_ARM_WIDTH = 0.014
CRADLE_ARM_HEIGHT = 0.022
CRADLE_ARM_OFFSET_Y = 0.018
ROLL_ORIGIN_X = 0.072
ROLL_HOUSING_LENGTH = 0.024
ROLL_HOUSING_OUTER_RADIUS = 0.026
ROLL_HOUSING_INNER_RADIUS = 0.018

ROLL_SHOULDER_RADIUS = 0.024
ROLL_SHOULDER_LENGTH = 0.008
ROLL_PILOT_RADIUS = 0.0175
ROLL_PILOT_LENGTH = 0.018
ROLL_BODY_RADIUS = 0.020
ROLL_BODY_LENGTH = 0.056
ROLL_NOSE_RADIUS = 0.016
ROLL_NOSE_LENGTH = 0.014
ROLL_FLANGE_RADIUS = 0.028
ROLL_FLANGE_LENGTH = 0.006


def _mesh(shape: cq.Workplane, name: str):
    return mesh_from_cadquery(
        shape,
        name,
        tolerance=0.0007,
        angular_tolerance=0.08,
        unit_scale=1.0,
    )


def _make_base_yoke() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.006)
    )

    pedestal = (
        cq.Workplane("XY")
        .circle(PEDESTAL_RADIUS)
        .extrude(PEDESTAL_HEIGHT)
        .translate((0.0, 0.0, BASE_THICKNESS))
    )

    collar = (
        cq.Workplane("XY")
        .circle(STATIONARY_COLLAR_RADIUS)
        .extrude(STATIONARY_COLLAR_HEIGHT)
        .translate((0.0, 0.0, YAW_ORIGIN_Z - STATIONARY_COLLAR_HEIGHT))
    )

    gusset_profile = [
        (-0.050, BASE_THICKNESS),
        (-0.016, BASE_THICKNESS),
        (0.010, YAW_ORIGIN_Z - 0.010),
        (-0.050, YAW_ORIGIN_Z - 0.010),
    ]
    gusset_proto = (
        cq.Workplane("XZ")
        .polyline(gusset_profile)
        .close()
        .extrude(BASE_GUSSET_THICKNESS)
    )
    gusset_left = gusset_proto.translate(
        (0.0, 0.028 - BASE_GUSSET_THICKNESS / 2.0, 0.0)
    )
    gusset_right = gusset_proto.translate(
        (0.0, -0.028 - BASE_GUSSET_THICKNESS / 2.0, 0.0)
    )

    return (
        plate.union(pedestal)
        .union(collar)
        .union(gusset_left)
        .union(gusset_right)
        .clean()
    )


def _make_yaw_stage() -> cq.Workplane:
    disc = cq.Workplane("XY").circle(YAW_DISC_RADIUS).extrude(YAW_DISC_THICKNESS)

    column = (
        cq.Workplane("XY")
        .box(0.050, 0.040, 0.032)
        .translate((0.0, 0.0, 0.030))
    )
    forward_spine = (
        cq.Workplane("XY")
        .box(0.028, 0.024, 0.018)
        .translate((0.024, 0.0, 0.046))
    )

    bridge_width = 2.0 * (YAW_LUG_INNER_HALF_GAP + YAW_LUG_THICKNESS)
    top_bridge = (
        cq.Workplane("XY")
        .box(0.022, bridge_width, 0.008)
        .translate((0.038, 0.0, PITCH_ORIGIN_Z + 0.019))
    )
    bottom_bridge = (
        cq.Workplane("XY")
        .box(0.020, bridge_width, 0.008)
        .translate((0.032, 0.0, PITCH_ORIGIN_Z - 0.019))
    )

    lug_offset_y = YAW_LUG_INNER_HALF_GAP + YAW_LUG_THICKNESS / 2.0
    cheek_left = (
        cq.Workplane("XY")
        .box(0.012, YAW_LUG_THICKNESS, 0.042)
        .translate((PITCH_ORIGIN_X, lug_offset_y, PITCH_ORIGIN_Z))
    )
    cheek_right = (
        cq.Workplane("XY")
        .box(0.012, YAW_LUG_THICKNESS, 0.042)
        .translate((PITCH_ORIGIN_X, -lug_offset_y, PITCH_ORIGIN_Z))
    )

    return (
        disc.union(column)
        .union(forward_spine)
        .union(top_bridge)
        .union(bottom_bridge)
        .union(cheek_left)
        .union(cheek_right)
        .clean()
    )


def _make_pitch_cradle() -> cq.Workplane:
    rear_plate = (
        cq.Workplane("XY")
        .box(0.008, 0.030, 0.016)
        .translate((0.010, 0.0, 0.0))
    )
    left_hinge_pad = (
        cq.Workplane("XY")
        .box(0.008, 0.014, 0.016)
        .translate((0.010, 0.022, 0.0))
    )
    right_hinge_pad = (
        cq.Workplane("XY")
        .box(0.008, 0.014, 0.016)
        .translate((0.010, -0.022, 0.0))
    )

    spine = (
        cq.Workplane("XY")
        .box(0.040, 0.040, 0.016)
        .translate((0.034, 0.0, 0.0))
    )
    arm_left = (
        cq.Workplane("XY")
        .box(0.044, 0.010, 0.016)
        .translate((0.044, CRADLE_ARM_OFFSET_Y, 0.0))
    )
    arm_right = (
        cq.Workplane("XY")
        .box(0.044, 0.010, 0.016)
        .translate((0.044, -CRADLE_ARM_OFFSET_Y, 0.0))
    )

    housing_outer = (
        cq.Workplane("YZ")
        .circle(ROLL_HOUSING_OUTER_RADIUS)
        .extrude(ROLL_HOUSING_LENGTH)
        .translate((ROLL_ORIGIN_X - ROLL_HOUSING_LENGTH, 0.0, 0.0))
    )
    housing_inner = (
        cq.Workplane("YZ")
        .circle(ROLL_HOUSING_INNER_RADIUS)
        .extrude(ROLL_HOUSING_LENGTH)
        .translate((ROLL_ORIGIN_X - ROLL_HOUSING_LENGTH, 0.0, 0.0))
    )
    housing = housing_outer.cut(housing_inner)

    return (
        rear_plate.union(left_hinge_pad)
        .union(right_hinge_pad)
        .union(spine)
        .union(arm_left)
        .union(arm_right)
        .union(housing)
        .clean()
    )


def _make_roll_cartridge() -> cq.Workplane:
    shoulder = cq.Workplane("YZ").circle(ROLL_SHOULDER_RADIUS).extrude(ROLL_SHOULDER_LENGTH)
    pilot = cq.Workplane("YZ").circle(ROLL_PILOT_RADIUS).extrude(ROLL_PILOT_LENGTH)
    body = (
        cq.Workplane("YZ")
        .circle(ROLL_BODY_RADIUS)
        .extrude(ROLL_BODY_LENGTH)
        .translate((0.004, 0.0, 0.0))
    )
    nose = (
        cq.Workplane("YZ")
        .circle(ROLL_NOSE_RADIUS)
        .extrude(ROLL_NOSE_LENGTH)
        .translate((ROLL_BODY_LENGTH, 0.0, 0.0))
    )
    flange = (
        cq.Workplane("YZ")
        .circle(ROLL_FLANGE_RADIUS)
        .extrude(ROLL_FLANGE_LENGTH)
        .translate((ROLL_BODY_LENGTH + ROLL_NOSE_LENGTH - 0.002, 0.0, 0.0))
    )

    return shoulder.union(pilot).union(body).union(nose).union(flange).clean()


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="yaw_pitch_roll_wrist_module")

    model.material("powder_black", rgba=(0.16, 0.16, 0.18, 1.0))
    model.material("machined_silver", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("anodized_gray", rgba=(0.42, 0.45, 0.49, 1.0))
    model.material("tool_steel", rgba=(0.60, 0.62, 0.65, 1.0))

    base_yoke = model.part("base_yoke")
    base_yoke.visual(
        _mesh(_make_base_yoke(), "base_yoke"),
        material="powder_black",
        name="base_yoke_shell",
    )

    yaw_stage = model.part("yaw_stage")
    yaw_stage.visual(
        _mesh(_make_yaw_stage(), "yaw_stage"),
        material="machined_silver",
        name="yaw_stage_shell",
    )

    pitch_cradle = model.part("pitch_cradle")
    pitch_cradle.visual(
        _mesh(_make_pitch_cradle(), "pitch_cradle"),
        material="anodized_gray",
        name="pitch_cradle_shell",
    )

    roll_cartridge = model.part("roll_cartridge")
    roll_cartridge.visual(
        _mesh(_make_roll_cartridge(), "roll_cartridge"),
        material="tool_steel",
        name="roll_cartridge_shell",
    )

    model.articulation(
        "base_to_yaw_stage",
        ArticulationType.REVOLUTE,
        parent=base_yoke,
        child=yaw_stage,
        origin=Origin(xyz=(0.0, 0.0, YAW_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-2.6,
            upper=2.6,
            effort=22.0,
            velocity=2.2,
        ),
    )
    model.articulation(
        "yaw_stage_to_pitch_cradle",
        ArticulationType.REVOLUTE,
        parent=yaw_stage,
        child=pitch_cradle,
        origin=Origin(xyz=(PITCH_ORIGIN_X, 0.0, PITCH_ORIGIN_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.85,
            upper=1.25,
            effort=18.0,
            velocity=2.4,
        ),
    )
    model.articulation(
        "pitch_cradle_to_roll_cartridge",
        ArticulationType.REVOLUTE,
        parent=pitch_cradle,
        child=roll_cartridge,
        origin=Origin(xyz=(ROLL_ORIGIN_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-pi,
            upper=pi,
            effort=12.0,
            velocity=5.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_yoke = object_model.get_part("base_yoke")
    yaw_stage = object_model.get_part("yaw_stage")
    pitch_cradle = object_model.get_part("pitch_cradle")
    roll_cartridge = object_model.get_part("roll_cartridge")

    yaw = object_model.get_articulation("base_to_yaw_stage")
    pitch = object_model.get_articulation("yaw_stage_to_pitch_cradle")
    roll = object_model.get_articulation("pitch_cradle_to_roll_cartridge")

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

    ctx.check("yaw axis is vertical", yaw.axis == (0.0, 0.0, 1.0), f"axis={yaw.axis}")
    ctx.check(
        "pitch axis is horizontal cross-axis",
        pitch.axis == (0.0, -1.0, 0.0),
        f"axis={pitch.axis}",
    )
    ctx.check(
        "roll axis follows tool axis",
        roll.axis == (1.0, 0.0, 0.0),
        f"axis={roll.axis}",
    )

    ctx.expect_contact(base_yoke, yaw_stage, name="base yoke supports yaw stage")
    ctx.expect_contact(yaw_stage, pitch_cradle, name="yaw stage supports pitch cradle")
    ctx.expect_contact(
        pitch_cradle,
        roll_cartridge,
        name="pitch cradle supports roll cartridge",
    )

    home_pitch_origin = ctx.part_world_position(pitch_cradle)
    home_roll_origin = ctx.part_world_position(roll_cartridge)

    with ctx.pose({yaw: 0.75}):
        yawed_pitch_origin = ctx.part_world_position(pitch_cradle)
        ctx.check(
            "positive yaw swings cradle around +Z",
            yawed_pitch_origin is not None
            and home_pitch_origin is not None
            and yawed_pitch_origin[1] > home_pitch_origin[1] + 0.020,
            f"home={home_pitch_origin}, yawed={yawed_pitch_origin}",
        )

    with ctx.pose({pitch: 0.70}):
        pitched_roll_origin = ctx.part_world_position(roll_cartridge)
        ctx.check(
            "positive pitch lifts the roll cartridge",
            pitched_roll_origin is not None
            and home_roll_origin is not None
            and pitched_roll_origin[2] > home_roll_origin[2] + 0.030,
            f"home={home_roll_origin}, pitched={pitched_roll_origin}",
        )

    with ctx.pose({roll: pi / 2.0}):
        rolled_origin = ctx.part_world_position(roll_cartridge)
        ctx.check(
            "roll joint spins about the cartridge origin",
            rolled_origin is not None
            and home_roll_origin is not None
            and max(abs(a - b) for a, b in zip(rolled_origin, home_roll_origin)) < 1e-9,
            f"home={home_roll_origin}, rolled={rolled_origin}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
