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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _base_bracket_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(0.140, 0.100, 0.012, centered=(True, True, False))
        .faces(">Z")
        .workplane()
        .pushPoints(
            [
                (-0.048, -0.032),
                (-0.048, 0.032),
                (0.048, -0.032),
                (0.048, 0.032),
            ]
        )
        .hole(0.012)
    )

    pedestal = cq.Workplane("XY").circle(0.015).extrude(0.038).translate((0.0, 0.0, 0.012))
    rear_wall = (
        cq.Workplane("XY")
        .box(0.016, 0.074, 0.036, centered=(True, True, False))
        .translate((-0.034, 0.0, 0.012))
    )
    left_gusset = (
        cq.Workplane("XY")
        .box(0.042, 0.010, 0.012, centered=(True, True, False))
        .translate((-0.008, 0.022, 0.028))
    )
    right_gusset = (
        cq.Workplane("XY")
        .box(0.042, 0.010, 0.012, centered=(True, True, False))
        .translate((-0.008, -0.022, 0.028))
    )

    return (
        plate.union(pedestal)
        .union(rear_wall)
        .union(left_gusset)
        .union(right_gusset)
    )


def _yaw_cartridge_shape() -> cq.Workplane:
    lower_flange = cq.Workplane("XY").circle(0.020).extrude(0.004)
    drum = cq.Workplane("XY").circle(0.016).extrude(0.026).translate((0.0, 0.0, 0.004))
    upper_cap = cq.Workplane("XY").circle(0.018).extrude(0.004).translate((0.0, 0.0, 0.030))

    spine = (
        cq.Workplane("XY")
        .box(0.024, 0.012, 0.014, centered=(False, True, False))
        .translate((0.012, 0.0, 0.003))
    )
    left_ear = (
        cq.Workplane("XY")
        .box(0.012, 0.008, 0.018, centered=(False, True, False))
        .translate((0.036, 0.010, 0.001))
    )
    right_ear = (
        cq.Workplane("XY")
        .box(0.012, 0.008, 0.018, centered=(False, True, False))
        .translate((0.036, -0.010, 0.001))
    )
    upper_bridge = (
        cq.Workplane("XY")
        .box(0.018, 0.028, 0.006, centered=(False, True, False))
        .translate((0.030, 0.0, 0.014))
    )

    return (
        lower_flange.union(drum)
        .union(upper_cap)
        .union(spine)
        .union(left_ear)
        .union(right_ear)
        .union(upper_bridge)
    )


def _pitch_frame_shape() -> cq.Workplane:
    trunnion_block = cq.Workplane("XY").box(0.006, 0.012, 0.014, centered=(False, True, True))
    left_rail = (
        cq.Workplane("XY")
        .box(0.044, 0.006, 0.012, centered=(False, True, True))
        .translate((0.006, 0.009, 0.0))
    )
    right_rail = (
        cq.Workplane("XY")
        .box(0.044, 0.006, 0.012, centered=(False, True, True))
        .translate((0.006, -0.009, 0.0))
    )
    lower_bridge = (
        cq.Workplane("XY")
        .box(0.022, 0.024, 0.006, centered=(False, True, True))
        .translate((0.014, 0.0, -0.008))
    )
    upper_bridge = (
        cq.Workplane("XY")
        .box(0.016, 0.022, 0.006, centered=(False, True, True))
        .translate((0.030, 0.0, 0.008))
    )
    roll_housing = (
        cq.Workplane("YZ")
        .circle(0.018)
        .circle(0.0132)
        .extrude(0.014)
        .translate((0.046, 0.0, 0.0))
    )

    return (
        trunnion_block.union(left_rail)
        .union(right_rail)
        .union(lower_bridge)
        .union(upper_bridge)
        .union(roll_housing)
    )


def _roll_output_shape() -> cq.Workplane:
    rear_flange = (
        cq.Workplane("YZ")
        .circle(0.018)
        .extrude(0.002)
    )
    journal = (
        cq.Workplane("YZ")
        .circle(0.013)
        .extrude(0.010)
        .translate((0.002, 0.0, 0.0))
    )
    body = (
        cq.Workplane("YZ")
        .circle(0.011)
        .extrude(0.054)
        .translate((0.012, 0.0, 0.0))
    )
    nose_collar = (
        cq.Workplane("YZ")
        .circle(0.014)
        .extrude(0.010)
        .translate((0.066, 0.0, 0.0))
    )
    tool_stub = (
        cq.Workplane("YZ")
        .circle(0.0085)
        .extrude(0.014)
        .translate((0.076, 0.0, 0.0))
    )

    return rear_flange.union(journal).union(body).union(nose_collar).union(tool_stub)


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_tool_wrist")

    model.material("powder_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("machined_alloy", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("dark_anodized", rgba=(0.28, 0.30, 0.33, 1.0))
    model.material("signal_black", rgba=(0.10, 0.11, 0.12, 1.0))

    base_bracket = model.part("base_bracket")
    base_bracket.visual(
        mesh_from_cadquery(_base_bracket_shape(), "base_bracket"),
        material="powder_steel",
        name="bracket_shell",
    )

    yaw_cartridge = model.part("yaw_cartridge")
    yaw_cartridge.visual(
        mesh_from_cadquery(_yaw_cartridge_shape(), "yaw_cartridge"),
        material="machined_alloy",
        name="cartridge_shell",
    )

    pitch_frame = model.part("pitch_frame")
    pitch_frame.visual(
        mesh_from_cadquery(_pitch_frame_shape(), "pitch_frame"),
        material="machined_alloy",
        name="frame_shell",
    )

    roll_output = model.part("roll_output")
    roll_output.visual(
        mesh_from_cadquery(_roll_output_shape(), "roll_output"),
        material="dark_anodized",
        name="output_shell",
    )
    roll_output.visual(
        Box((0.022, 0.010, 0.012)),
        origin=Origin(xyz=(0.054, 0.0, 0.015)),
        material="signal_black",
        name="tool_key",
    )

    model.articulation(
        "yaw_joint",
        ArticulationType.REVOLUTE,
        parent=base_bracket,
        child=yaw_cartridge,
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.6, lower=-2.35, upper=2.35),
    )
    model.articulation(
        "pitch_joint",
        ArticulationType.REVOLUTE,
        parent=yaw_cartridge,
        child=pitch_frame,
        origin=Origin(xyz=(0.048, 0.0, 0.010)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=-0.85, upper=1.15),
    )
    model.articulation(
        "roll_joint",
        ArticulationType.REVOLUTE,
        parent=pitch_frame,
        child=roll_output,
        origin=Origin(xyz=(0.060, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.8, lower=-2.80, upper=2.80),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_bracket = object_model.get_part("base_bracket")
    yaw_cartridge = object_model.get_part("yaw_cartridge")
    pitch_frame = object_model.get_part("pitch_frame")
    roll_output = object_model.get_part("roll_output")
    yaw_joint = object_model.get_articulation("yaw_joint")
    pitch_joint = object_model.get_articulation("pitch_joint")
    roll_joint = object_model.get_articulation("roll_joint")
    tool_key = roll_output.get_visual("tool_key")

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

    ctx.expect_contact(yaw_cartridge, base_bracket, name="yaw_cartridge_is_seated")
    ctx.expect_contact(pitch_frame, yaw_cartridge, name="pitch_frame_is_supported")
    ctx.expect_contact(roll_output, pitch_frame, name="roll_output_is_supported")

    rest_roll_center = _aabb_center(ctx.part_world_aabb(roll_output))
    with ctx.pose({yaw_joint: 0.85}):
        yawed_roll_center = _aabb_center(ctx.part_world_aabb(roll_output))
    ctx.check(
        "yaw_positive_swings_output_toward_positive_y",
        rest_roll_center is not None
        and yawed_roll_center is not None
        and yawed_roll_center[1] > rest_roll_center[1] + 0.020,
        details=f"rest={rest_roll_center}, yawed={yawed_roll_center}",
    )

    with ctx.pose({pitch_joint: 0.75}):
        pitched_roll_center = _aabb_center(ctx.part_world_aabb(roll_output))
    ctx.check(
        "pitch_positive_raises_output",
        rest_roll_center is not None
        and pitched_roll_center is not None
        and pitched_roll_center[2] > rest_roll_center[2] + 0.020,
        details=f"rest={rest_roll_center}, pitched={pitched_roll_center}",
    )

    rest_key_center = _aabb_center(ctx.part_element_world_aabb(roll_output, elem=tool_key))
    with ctx.pose({roll_joint: 1.10}):
        rolled_key_center = _aabb_center(ctx.part_element_world_aabb(roll_output, elem=tool_key))
    ctx.check(
        "roll_positive_rotates_key_toward_negative_y",
        rest_key_center is not None
        and rolled_key_center is not None
        and rolled_key_center[1] < rest_key_center[1] - 0.008,
        details=f"rest={rest_key_center}, rolled={rolled_key_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
