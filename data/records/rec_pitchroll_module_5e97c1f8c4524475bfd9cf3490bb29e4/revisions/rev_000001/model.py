from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


PITCH_AXIS_Z = 0.110
FORK_INNER_GAP = 0.080
CHEEK_THICKNESS = 0.014
CHEEK_CENTER_Y = FORK_INNER_GAP / 2.0 + CHEEK_THICKNESS / 2.0
TRUNNION_SHAFT_RADIUS = 0.011
ROLL_SPINDLE_RADIUS = 0.021
ROLL_JOINT_X = 0.052


def along_y(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(pi / 2.0, 0.0, 0.0))


def along_x(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(0.0, pi / 2.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_spindle_sensor_bracket")

    model.material("frame_metal", rgba=(0.42, 0.45, 0.49, 1.0))
    model.material("cap_dark", rgba=(0.15, 0.16, 0.18, 1.0))
    model.material("yoke_metal", rgba=(0.58, 0.60, 0.63, 1.0))
    model.material("sensor_body", rgba=(0.32, 0.34, 0.36, 1.0))
    model.material("sensor_window", rgba=(0.20, 0.44, 0.66, 0.72))

    base = model.part("base_frame")
    base.visual(Box((0.140, 0.122, 0.012)), origin=Origin(xyz=(0.0, 0.0, 0.006)), material="frame_metal", name="mount_base")
    base.visual(Box((0.028, 0.080, 0.056)), origin=Origin(xyz=(-0.038, 0.0, 0.040)), material="frame_metal", name="rear_bridge")
    base.visual(Box((0.054, CHEEK_THICKNESS, 0.116)), origin=Origin(xyz=(-0.021, CHEEK_CENTER_Y, 0.070)), material="frame_metal", name="left_cheek")
    base.visual(Box((0.054, CHEEK_THICKNESS, 0.116)), origin=Origin(xyz=(-0.021, -CHEEK_CENTER_Y, 0.070)), material="frame_metal", name="right_cheek")
    base.visual(Box((0.030, CHEEK_THICKNESS, 0.032)), origin=Origin(xyz=(-0.018, CHEEK_CENTER_Y, 0.028)), material="frame_metal", name="left_pedestal")
    base.visual(Box((0.030, CHEEK_THICKNESS, 0.032)), origin=Origin(xyz=(-0.018, -CHEEK_CENTER_Y, 0.028)), material="frame_metal", name="right_pedestal")
    base.visual(Cylinder(radius=0.018, length=0.002), origin=along_y((0.0, 0.041, PITCH_AXIS_Z)), material="frame_metal", name="left_inner_pitch_collar")
    base.visual(Cylinder(radius=0.018, length=0.002), origin=along_y((0.0, -0.041, PITCH_AXIS_Z)), material="frame_metal", name="right_inner_pitch_collar")
    base.visual(Cylinder(radius=0.024, length=0.014), origin=along_y((0.0, CHEEK_CENTER_Y, PITCH_AXIS_Z)), material="frame_metal", name="left_pitch_boss")
    base.visual(Cylinder(radius=0.024, length=0.014), origin=along_y((0.0, -CHEEK_CENTER_Y, PITCH_AXIS_Z)), material="frame_metal", name="right_pitch_boss")
    base.visual(Cylinder(radius=0.028, length=0.004), origin=along_y((0.0, 0.056, PITCH_AXIS_Z)), material="cap_dark", name="left_pitch_cap")
    base.visual(Cylinder(radius=0.028, length=0.004), origin=along_y((0.0, -0.056, PITCH_AXIS_Z)), material="cap_dark", name="right_pitch_cap")
    for side_name, sign in (("left", 1.0), ("right", -1.0)):
        for idx, (dx, dz) in enumerate(((-0.010, -0.020), (-0.010, 0.020), (0.010, -0.020), (0.010, 0.020)), start=1):
            base.visual(
                Cylinder(radius=0.0028, length=0.003),
                origin=along_y((dx, sign * 0.0595, PITCH_AXIS_Z + dz)),
                material="cap_dark",
                name=f"{side_name}_pitch_bolt_{idx}",
            )

    yoke = model.part("pitch_yoke")
    yoke.visual(Cylinder(radius=TRUNNION_SHAFT_RADIUS, length=0.080), origin=along_y((0.0, 0.0, 0.0)), material="yoke_metal", name="trunnion_shaft")
    yoke.visual(Cylinder(radius=0.017, length=0.006), origin=along_y((0.0, 0.036, 0.0)), material="yoke_metal", name="left_trunnion_shoulder")
    yoke.visual(Cylinder(radius=0.017, length=0.006), origin=along_y((0.0, -0.036, 0.0)), material="yoke_metal", name="right_trunnion_shoulder")
    yoke.visual(Box((0.018, 0.020, 0.020)), origin=Origin(xyz=(0.016, 0.0, 0.0)), material="yoke_metal", name="hub_core")
    yoke.visual(Box((0.030, 0.010, 0.008)), origin=Origin(xyz=(0.026, 0.0, 0.015)), material="yoke_metal", name="top_rail")
    yoke.visual(Box((0.030, 0.010, 0.008)), origin=Origin(xyz=(0.026, 0.0, -0.015)), material="yoke_metal", name="bottom_rail")
    yoke.visual(Box((0.022, 0.012, 0.016)), origin=Origin(xyz=(0.030, 0.0, 0.0)), material="yoke_metal", name="center_spine")
    yoke.visual(Box((0.024, 0.010, 0.008)), origin=Origin(xyz=(ROLL_JOINT_X, 0.0, 0.028)), material="yoke_metal", name="upper_bearing_bridge")
    yoke.visual(Box((0.024, 0.010, 0.008)), origin=Origin(xyz=(ROLL_JOINT_X, 0.0, -0.028)), material="yoke_metal", name="lower_bearing_bridge")
    yoke.visual(Cylinder(radius=0.026, length=0.004), origin=along_x((ROLL_JOINT_X - 0.012, 0.0, 0.0)), material="cap_dark", name="rear_roll_bearing")
    yoke.visual(Cylinder(radius=0.026, length=0.004), origin=along_x((ROLL_JOINT_X + 0.012, 0.0, 0.0)), material="cap_dark", name="front_roll_bearing")
    for side_name, sign in (("top", 1.0), ("bottom", -1.0)):
        z = sign * 0.020
        for idx, dy in enumerate((-0.010, 0.010), start=1):
            yoke.visual(
                Cylinder(radius=0.0024, length=0.006),
                origin=along_x((ROLL_JOINT_X - 0.0145, dy, z)),
                material="cap_dark",
                name=f"{side_name}_rear_roll_bolt_{idx}",
            )
            yoke.visual(
                Cylinder(radius=0.0024, length=0.006),
                origin=along_x((ROLL_JOINT_X + 0.0145, dy, z)),
                material="cap_dark",
                name=f"{side_name}_front_roll_bolt_{idx}",
            )

    roll = model.part("roll_cartridge")
    roll.visual(Cylinder(radius=ROLL_SPINDLE_RADIUS, length=0.020), origin=along_x((0.0, 0.0, 0.0)), material="sensor_body", name="roll_spindle")
    roll.visual(Cylinder(radius=0.024, length=0.018), origin=along_x((0.017, 0.0, 0.0)), material="sensor_body", name="roll_neck")
    roll.visual(Cylinder(radius=0.035, length=0.048), origin=along_x((0.048, 0.0, 0.0)), material="sensor_body", name="cartridge_barrel")
    roll.visual(Cylinder(radius=0.038, length=0.012), origin=along_x((0.074, 0.0, 0.0)), material="sensor_body", name="front_bezel")
    roll.visual(Box((0.020, 0.016, 0.014)), origin=Origin(xyz=(0.048, 0.0, 0.033)), material="sensor_body", name="service_bump")
    roll.visual(Cylinder(radius=0.028, length=0.006), origin=along_x((0.074, 0.0, 0.0)), material="sensor_window", name="sensor_window")

    model.articulation(
        "pitch_joint",
        ArticulationType.REVOLUTE,
        parent=base,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, PITCH_AXIS_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-0.55, upper=0.57),
    )
    model.articulation(
        "roll_joint",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=roll,
        origin=Origin(xyz=(ROLL_JOINT_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=4.0, lower=-pi, upper=pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_frame")
    yoke = object_model.get_part("pitch_yoke")
    roll = object_model.get_part("roll_cartridge")
    pitch = object_model.get_articulation("pitch_joint")
    spindle = object_model.get_articulation("roll_joint")

    ctx.allow_overlap(
        yoke,
        roll,
        elem_a="front_roll_bearing",
        elem_b="roll_neck",
        reason="The forward roll support is a simplified solid stand-in for a bored coaxial bearing shell that intentionally captures the roll neck.",
    )

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check("base_frame_present", base is not None, "base frame part missing")
    ctx.check("pitch_yoke_present", yoke is not None, "pitch yoke part missing")
    ctx.check("roll_cartridge_present", roll is not None, "roll cartridge part missing")
    ctx.check("pitch_axis_is_lateral", tuple(pitch.axis) == (0.0, 1.0, 0.0), f"expected pitch axis (0,1,0), got {pitch.axis}")
    ctx.check("roll_axis_is_forward", tuple(spindle.axis) == (1.0, 0.0, 0.0), f"expected roll axis (1,0,0), got {spindle.axis}")
    ctx.check(
        "pitch_range_is_useful",
        pitch.motion_limits is not None
        and pitch.motion_limits.lower is not None
        and pitch.motion_limits.upper is not None
        and pitch.motion_limits.lower < -0.4
        and pitch.motion_limits.upper >= 0.55,
        "pitch limits should allow a meaningful gimbal sweep",
    )
    ctx.check(
        "roll_range_is_full",
        spindle.motion_limits is not None
        and spindle.motion_limits.lower is not None
        and spindle.motion_limits.upper is not None
        and spindle.motion_limits.upper - spindle.motion_limits.lower > 6.0,
        "roll cartridge should have nearly full-turn travel",
    )

    ctx.expect_contact(base, yoke, elem_a="left_inner_pitch_collar", elem_b="trunnion_shaft", name="left_pitch_trunnion_grounded")
    ctx.expect_contact(base, yoke, elem_a="right_inner_pitch_collar", elem_b="trunnion_shaft", name="right_pitch_trunnion_grounded")
    ctx.expect_contact(yoke, roll, elem_a="rear_roll_bearing", elem_b="roll_spindle", name="rear_roll_bearing_grounded")
    ctx.expect_contact(yoke, roll, elem_a="front_roll_bearing", elem_b="roll_spindle", name="front_roll_bearing_grounded")

    with ctx.pose({pitch: pitch.motion_limits.lower, spindle: spindle.motion_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_pitch_down_roll_max")
        ctx.expect_gap(yoke, base, axis="z", positive_elem="hub_core", negative_elem="mount_base", min_gap=0.010, name="yoke_clears_base_at_pitch_down")

    with ctx.pose({pitch: pitch.motion_limits.upper, spindle: spindle.motion_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_pitch_up_roll_min")
        ctx.expect_gap(roll, base, axis="z", positive_elem="cartridge_barrel", negative_elem="mount_base", min_gap=0.001, name="cartridge_clears_base_at_pitch_up")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
