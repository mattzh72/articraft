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


def make_support_frame() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .box(0.16, 0.12, 0.012, centered=(True, True, False))
    )

    cheek_profile = [
        (-0.062, 0.012),
        (-0.062, 0.172),
        (-0.034, 0.172),
        (-0.008, 0.132),
        (0.010, 0.132),
        (0.010, 0.012),
    ]
    left_cheek = (
        cq.Workplane("XZ")
        .polyline(cheek_profile)
        .close()
        .extrude(0.012)
        .translate((0.0, 0.042, 0.0))
    )
    right_cheek = left_cheek.translate((0.0, -0.096, 0.0))

    rear_panel = (
        cq.Workplane("XY")
        .box(0.012, 0.092, 0.096, centered=(True, True, False))
        .translate((-0.060, 0.0, 0.024))
    )

    top_bridge = (
        cq.Workplane("XY")
        .box(0.028, 0.108, 0.018, centered=(True, True, False))
        .translate((-0.048, 0.0, 0.154))
    )

    bridge_spine = (
        cq.Workplane("XY")
        .box(0.044, 0.032, 0.016, centered=(True, True, False))
        .translate((-0.016, 0.0, 0.156))
    )

    column = cq.Workplane("XY").circle(0.030).extrude(0.160).translate((0.0, 0.0, 0.012))
    top_deck = cq.Workplane("XY").circle(0.048).extrude(0.012).translate((0.0, 0.0, 0.172))

    return (
        base.union(left_cheek)
        .union(right_cheek)
        .union(rear_panel)
        .union(top_bridge)
        .union(bridge_spine)
        .union(column)
        .union(top_deck)
    )


def make_yaw_base() -> cq.Workplane:
    rotor = cq.Workplane("XY").circle(0.042).extrude(0.020)
    shoulder = cq.Workplane("XY").circle(0.047).extrude(0.010).translate((0.0, 0.0, 0.020))

    mast = (
        cq.Workplane("XY")
        .box(0.024, 0.034, 0.020, centered=(False, True, False))
        .translate((0.0, 0.0, 0.028))
    )
    fork_block = (
        cq.Workplane("XY")
        .box(0.030, 0.050, 0.028, centered=(False, True, False))
        .translate((0.028, 0.0, 0.044))
    )
    connector = (
        cq.Workplane("XY")
        .box(0.010, 0.022, 0.014, centered=(False, True, False))
        .translate((0.020, 0.0, 0.042))
    )
    rear_bridge = (
        cq.Workplane("XY")
        .box(0.012, 0.044, 0.010, centered=(False, True, False))
        .translate((0.026, 0.0, 0.060))
    )
    center_slot = (
        cq.Workplane("XY")
        .box(0.024, 0.030, 0.020, centered=(False, True, False))
        .translate((0.034, 0.0, 0.046))
    )
    trunnion_bore = (
        cq.Workplane("XZ")
        .circle(0.004)
        .extrude(0.060, both=True)
        .translate((0.040, 0.0, 0.057))
    )

    return (
        rotor.union(shoulder)
        .union(mast)
        .union(connector)
        .union(fork_block)
        .union(rear_bridge)
        .cut(center_slot)
        .cut(trunnion_bore)
    )


def make_pitch_yoke() -> cq.Workplane:
    shaft = (
        cq.Workplane("XZ")
        .circle(0.004)
        .extrude(0.044, both=True)
    )
    rear_hub = (
        cq.Workplane("XY")
        .box(0.010, 0.016, 0.012, centered=(False, True, True))
        .translate((0.002, 0.0, 0.0))
    )
    spine = (
        cq.Workplane("XY")
        .box(0.028, 0.012, 0.008, centered=(False, True, True))
        .translate((0.010, 0.0, 0.0))
    )
    front_plate = (
        cq.Workplane("XY")
        .box(0.014, 0.022, 0.028, centered=(False, True, True))
        .translate((0.036, 0.0, 0.0))
    )
    output_pad = (
        cq.Workplane("XY")
        .box(0.008, 0.012, 0.014, centered=(False, True, True))
        .translate((0.050, 0.0, 0.0))
    )

    return shaft.union(rear_hub).union(spine).union(front_plate).union(output_pad)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_backed_yaw_pitch_module")

    frame_color = model.material("frame_gray", rgba=(0.24, 0.26, 0.29, 1.0))
    yaw_color = model.material("yaw_gray", rgba=(0.62, 0.65, 0.68, 1.0))
    pitch_color = model.material("pitch_gray", rgba=(0.76, 0.78, 0.80, 1.0))

    support_frame = model.part("support_frame")
    support_frame.visual(
        mesh_from_cadquery(make_support_frame(), "support_frame"),
        origin=Origin(),
        material=frame_color,
        name="frame_shell",
    )

    yaw_base = model.part("yaw_base")
    yaw_base.visual(
        mesh_from_cadquery(make_yaw_base(), "yaw_base"),
        origin=Origin(),
        material=yaw_color,
        name="yaw_shell",
    )

    pitch_yoke = model.part("pitch_yoke")
    pitch_yoke.visual(
        mesh_from_cadquery(make_pitch_yoke(), "pitch_yoke"),
        origin=Origin(),
        material=pitch_color,
        name="pitch_shell",
    )

    model.articulation(
        "frame_to_yaw",
        ArticulationType.REVOLUTE,
        parent=support_frame,
        child=yaw_base,
        origin=Origin(xyz=(0.0, 0.0, 0.184)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-2.2,
            upper=2.2,
        ),
    )

    model.articulation(
        "yaw_to_pitch",
        ArticulationType.REVOLUTE,
        parent=yaw_base,
        child=pitch_yoke,
        origin=Origin(xyz=(0.062, 0.0, 0.057)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=-0.35,
            upper=0.7,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support_frame = object_model.get_part("support_frame")
    yaw_base = object_model.get_part("yaw_base")
    pitch_yoke = object_model.get_part("pitch_yoke")
    yaw_joint = object_model.get_articulation("frame_to_yaw")
    pitch_joint = object_model.get_articulation("yaw_to_pitch")

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

    ctx.expect_contact(yaw_base, support_frame, name="yaw_base_is_seated_on_support_frame")
    ctx.expect_contact(pitch_yoke, yaw_base, name="pitch_yoke_is_supported_by_yaw_base")

    ctx.check(
        "yaw_joint_is_vertical",
        yaw_joint.axis == (0.0, 0.0, 1.0),
        details=f"expected vertical yaw axis, got {yaw_joint.axis}",
    )
    ctx.check(
        "pitch_joint_is_horizontal",
        pitch_joint.axis == (0.0, -1.0, 0.0),
        details=f"expected horizontal pitch axis lifting on positive motion, got {pitch_joint.axis}",
    )

    pitch_lower = -0.35
    pitch_upper = 0.7
    if pitch_joint.motion_limits is not None:
        if pitch_joint.motion_limits.lower is not None:
            pitch_lower = pitch_joint.motion_limits.lower
        if pitch_joint.motion_limits.upper is not None:
            pitch_upper = pitch_joint.motion_limits.upper

    neutral_pitch_pos = ctx.part_world_position(pitch_yoke)
    with ctx.pose({yaw_joint: 1.1}):
        yawed_pitch_pos = ctx.part_world_position(pitch_yoke)
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_yawed_pose")

    yaw_motion_ok = (
        neutral_pitch_pos is not None
        and yawed_pitch_pos is not None
        and abs(yawed_pitch_pos[1] - neutral_pitch_pos[1]) > 0.03
    )
    ctx.check(
        "yaw_motion_swings_pitch_module_sideways",
        yaw_motion_ok,
        details=f"neutral={neutral_pitch_pos}, yawed={yawed_pitch_pos}",
    )

    neutral_aabb = ctx.part_world_aabb(pitch_yoke)
    with ctx.pose({pitch_joint: pitch_upper}):
        raised_aabb = ctx.part_world_aabb(pitch_yoke)
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_raised_pitch_pose")

    pitch_motion_ok = (
        neutral_aabb is not None
        and raised_aabb is not None
        and raised_aabb[1][2] > neutral_aabb[1][2] + 0.02
    )
    ctx.check(
        "pitch_motion_lifts_output_face",
        pitch_motion_ok,
        details=f"neutral_aabb={neutral_aabb}, raised_aabb={raised_aabb}",
    )

    with ctx.pose({pitch_joint: pitch_lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_lowered_pitch_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
