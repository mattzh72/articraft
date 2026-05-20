from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


def _make_base_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(0.105, 0.082, 0.018)
        .translate((0.0, 0.0, 0.009))
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-0.032, -0.024),
                (-0.032, 0.024),
                (0.032, -0.024),
                (0.032, 0.024),
            ]
        )
        .hole(0.009)
    )
    journal = cq.Workplane("XY").circle(0.028).extrude(0.022).translate((0.0, 0.0, 0.018))
    thrust_face = cq.Workplane("XY").circle(0.040).extrude(0.004).translate((0.0, 0.0, 0.036))
    return plate.union(journal).union(thrust_face)


def _make_yaw_collar_shape() -> cq.Workplane:
    collar_body = cq.Workplane("XY").circle(0.046).extrude(0.044)
    collar_bore = cq.Workplane("XY").circle(0.031).extrude(0.044)
    ring = collar_body.cut(collar_bore)

    upper_gusset = cq.Workplane("XY").box(0.018, 0.014, 0.024).translate((0.036, 0.026, 0.022))
    lower_gusset = cq.Workplane("XY").box(0.018, 0.014, 0.024).translate((0.036, -0.026, 0.022))
    upper_ear = cq.Workplane("XY").box(0.026, 0.012, 0.032).translate((0.055, 0.030, 0.022))
    lower_ear = cq.Workplane("XY").box(0.026, 0.012, 0.032).translate((0.055, -0.030, 0.022))

    return ring.union(upper_gusset).union(lower_gusset).union(upper_ear).union(lower_ear)


def _make_pitch_yoke_shape() -> cq.Workplane:
    rear_bridge = cq.Workplane("XY").box(0.024, 0.060, 0.020).translate((0.012, 0.0, 0.0))

    arm_top = cq.Workplane("XY").box(0.060, 0.012, 0.040).translate((0.030, 0.030, 0.0))
    arm_bottom = cq.Workplane("XY").box(0.060, 0.012, 0.040).translate((0.030, -0.030, 0.0))

    top_trunnion = cq.Workplane("YZ").circle(0.016).extrude(0.006).translate((0.0, 0.030, 0.0))
    bottom_trunnion = cq.Workplane("YZ").circle(0.016).extrude(0.006).translate((0.0, -0.030, 0.0))

    top_link = cq.Workplane("XY").box(0.036, 0.008, 0.018).translate((0.042, 0.021, 0.0))
    bottom_link = cq.Workplane("XY").box(0.036, 0.008, 0.018).translate((0.042, -0.021, 0.0))
    roll_seat = cq.Workplane("YZ").circle(0.020).extrude(0.006).translate((0.054, 0.0, 0.0))

    return (
        rear_bridge.union(arm_top)
        .union(arm_bottom)
        .union(top_trunnion)
        .union(bottom_trunnion)
        .union(top_link)
        .union(bottom_link)
        .union(roll_seat)
    )


def _make_roll_spindle_shape() -> cq.Workplane:
    rear_shoulder = cq.Workplane("YZ").circle(0.018).extrude(0.006)
    shaft = cq.Workplane("YZ").circle(0.014).extrude(0.042).translate((0.006, 0.0, 0.0))
    flange = cq.Workplane("YZ").circle(0.030).extrude(0.010).translate((0.038, 0.0, 0.0))
    pilot = cq.Workplane("YZ").circle(0.009).extrude(0.008).translate((0.048, 0.0, 0.0))

    spindle = rear_shoulder.union(shaft).union(flange).union(pilot)

    bolt_holes = (
        cq.Workplane("YZ")
        .pushPoints([(0.018, 0.0), (-0.018, 0.0), (0.0, 0.018), (0.0, -0.018)])
        .circle(0.0032)
        .extrude(0.010)
        .translate((0.038, 0.0, 0.0))
    )
    center_bore = cq.Workplane("YZ").circle(0.005).extrude(0.018).translate((0.038, 0.0, 0.0))

    return spindle.cut(bolt_holes).cut(center_bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tool_wrist")

    base_paint = model.material("base_paint", color=(0.22, 0.22, 0.24, 1.0))
    collar_paint = model.material("collar_paint", color=(0.16, 0.17, 0.19, 1.0))
    yoke_finish = model.material("yoke_finish", color=(0.68, 0.71, 0.74, 1.0))
    spindle_finish = model.material("spindle_finish", color=(0.80, 0.82, 0.84, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_shape(), "tool_wrist_base"),
        material=base_paint,
        name="base_shell",
    )

    yaw_collar = model.part("yaw_collar")
    yaw_collar.visual(
        mesh_from_cadquery(_make_yaw_collar_shape(), "tool_wrist_yaw_collar"),
        material=collar_paint,
        name="yaw_collar_shell",
    )

    pitch_yoke = model.part("pitch_yoke")
    pitch_yoke.visual(
        mesh_from_cadquery(_make_pitch_yoke_shape(), "tool_wrist_pitch_yoke"),
        material=yoke_finish,
        name="pitch_yoke_shell",
    )

    roll_spindle = model.part("roll_spindle")
    roll_spindle.visual(
        mesh_from_cadquery(_make_roll_spindle_shape(), "tool_wrist_roll_spindle"),
        material=spindle_finish,
        name="roll_spindle_shell",
    )

    model.articulation(
        "base_to_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=yaw_collar,
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-2.4, upper=2.4),
    )
    model.articulation(
        "yaw_to_pitch",
        ArticulationType.REVOLUTE,
        parent=yaw_collar,
        child=pitch_yoke,
        origin=Origin(xyz=(0.068, 0.0, 0.022)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.3, lower=-1.1, upper=1.1),
    )
    model.articulation(
        "pitch_to_roll",
        ArticulationType.REVOLUTE,
        parent=pitch_yoke,
        child=roll_spindle,
        origin=Origin(xyz=(0.060, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=9.0, velocity=4.0, lower=-3.0, upper=3.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    yaw_collar = object_model.get_part("yaw_collar")
    pitch_yoke = object_model.get_part("pitch_yoke")
    roll_spindle = object_model.get_part("roll_spindle")

    yaw_joint = object_model.get_articulation("base_to_yaw")
    pitch_joint = object_model.get_articulation("yaw_to_pitch")
    roll_joint = object_model.get_articulation("pitch_to_roll")

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

    ctx.check(
        "all_expected_parts_present",
        {part.name for part in object_model.parts}
        == {"base", "yaw_collar", "pitch_yoke", "roll_spindle"},
        details="The wrist should contain the fixed base and the three articulated axis supports.",
    )
    ctx.check(
        "all_joints_are_revolute",
        yaw_joint.joint_type == ArticulationType.REVOLUTE
        and pitch_joint.joint_type == ArticulationType.REVOLUTE
        and roll_joint.joint_type == ArticulationType.REVOLUTE,
        details="Yaw, pitch, and roll must all be authored as revolute joints.",
    )
    ctx.check(
        "joint_axes_match_wrist_layout",
        yaw_joint.axis == (0.0, 0.0, 1.0)
        and pitch_joint.axis == (0.0, -1.0, 0.0)
        and roll_joint.axis == (1.0, 0.0, 0.0),
        details="Expected a vertical yaw axis, a horizontal pitch cross-axis, and a tool-axis roll joint.",
    )

    ctx.expect_contact(yaw_collar, base, contact_tol=0.0001, name="yaw_collar_seats_on_base")
    ctx.expect_contact(pitch_yoke, yaw_collar, contact_tol=0.0001, name="pitch_yoke_seats_in_yaw_ears")
    ctx.expect_contact(roll_spindle, pitch_yoke, contact_tol=0.0001, name="roll_spindle_seats_on_pitch_yoke")
    ctx.expect_overlap(yaw_collar, base, axes="xy", min_overlap=0.080, name="yaw_collar_over_base_footprint")
    ctx.expect_overlap(pitch_yoke, yaw_collar, axes="yz", min_overlap=0.030, name="pitch_yoke_supported_by_yaw_ears")
    ctx.expect_overlap(roll_spindle, pitch_yoke, axes="yz", min_overlap=0.036, name="roll_spindle_nested_in_yoke")

    ctx.expect_origin_gap(yaw_collar, base, axis="z", min_gap=0.039, max_gap=0.041, name="yaw_axis_height")
    ctx.expect_origin_gap(
        pitch_yoke,
        yaw_collar,
        axis="x",
        min_gap=0.067,
        max_gap=0.069,
        name="pitch_axis_forward_of_yaw_axis",
    )
    ctx.expect_origin_gap(
        pitch_yoke,
        yaw_collar,
        axis="z",
        min_gap=0.021,
        max_gap=0.023,
        name="pitch_axis_above_yaw_collar_base",
    )
    ctx.expect_origin_gap(
        roll_spindle,
        pitch_yoke,
        axis="x",
        min_gap=0.059,
        max_gap=0.061,
        name="roll_axis_forward_of_pitch_axis",
    )

    rest_roll_position = ctx.part_world_position(roll_spindle)
    with ctx.pose({yaw_joint: 0.8}):
        yawed_roll_position = ctx.part_world_position(roll_spindle)
        ctx.check(
            "yaw_swings_nose_sideways",
            rest_roll_position is not None
            and yawed_roll_position is not None
            and abs(yawed_roll_position[1] - rest_roll_position[1]) > 0.07,
            details="A nonzero yaw pose should swing the nose laterally around the vertical root axis.",
        )

    with ctx.pose({pitch_joint: 0.7}):
        pitched_roll_position = ctx.part_world_position(roll_spindle)
        ctx.check(
            "pitch_moves_nose_vertically",
            rest_roll_position is not None
            and pitched_roll_position is not None
            and abs(pitched_roll_position[2] - rest_roll_position[2]) > 0.03,
            details="A nonzero pitch pose should move the nose vertically about the cross-axis.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
