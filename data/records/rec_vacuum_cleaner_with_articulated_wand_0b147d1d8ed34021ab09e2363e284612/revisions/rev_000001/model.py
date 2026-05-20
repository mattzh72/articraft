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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mass_market_vacuum_cleaner")

    body_gray = model.material("body_gray", rgba=(0.22, 0.24, 0.27, 1.0))
    collar_black = model.material("collar_black", rgba=(0.10, 0.10, 0.11, 1.0))
    tube_silver = model.material("tube_silver", rgba=(0.73, 0.75, 0.78, 1.0))
    clear_bin = model.material("clear_bin", rgba=(0.78, 0.82, 0.88, 0.40))
    accent_red = model.material("accent_red", rgba=(0.72, 0.15, 0.12, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.070, 0.080, 0.040)),
        origin=Origin(xyz=(-0.035, 0.0, 0.040)),
        material=body_gray,
        name="receiver_bridge",
    )
    body.visual(
        Box((0.020, 0.010, 0.050)),
        origin=Origin(xyz=(-0.010, 0.024, 0.000)),
        material=collar_black,
        name="receiver_left_ear",
    )
    body.visual(
        Box((0.020, 0.010, 0.050)),
        origin=Origin(xyz=(-0.010, -0.024, 0.000)),
        material=collar_black,
        name="receiver_right_ear",
    )
    body.visual(
        Box((0.130, 0.090, 0.100)),
        origin=Origin(xyz=(-0.010, 0.0, 0.095)),
        material=body_gray,
        name="intake_block",
    )
    body.visual(
        Box((0.120, 0.112, 0.190)),
        origin=Origin(xyz=(-0.110, 0.0, 0.165)),
        material=body_gray,
        name="motor_shell",
    )
    body.visual(
        Cylinder(radius=0.055, length=0.110),
        origin=Origin(xyz=(-0.162, 0.0, 0.195), rpy=(0.0, pi / 2.0, 0.0)),
        material=body_gray,
        name="rear_cap",
    )
    body.visual(
        Cylinder(radius=0.052, length=0.160),
        origin=Origin(xyz=(0.060, 0.0, 0.175), rpy=(0.0, pi / 2.0, 0.0)),
        material=clear_bin,
        name="dust_bin",
    )
    body.visual(
        Box((0.055, 0.092, 0.190)),
        origin=Origin(xyz=(-0.116, 0.0, 0.290)),
        material=body_gray,
        name="handle_post",
    )
    body.visual(
        Cylinder(radius=0.028, length=0.092),
        origin=Origin(xyz=(-0.116, 0.0, 0.385), rpy=(pi / 2.0, 0.0, 0.0)),
        material=body_gray,
        name="handle_cap",
    )
    body.visual(
        Box((0.040, 0.086, 0.020)),
        origin=Origin(xyz=(-0.040, 0.0, 0.245)),
        material=accent_red,
        name="release_latch",
    )

    upper_wand = model.part("upper_wand")
    upper_wand.visual(
        Box((0.028, 0.038, 0.028)),
        origin=Origin(xyz=(0.014, 0.0, 0.0)),
        material=collar_black,
        name="shoulder_lug",
    )
    upper_wand.visual(
        Cylinder(radius=0.018, length=0.252),
        origin=Origin(xyz=(0.154, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=tube_silver,
        name="upper_tube",
    )
    upper_wand.visual(
        Cylinder(radius=0.021, length=0.040),
        origin=Origin(xyz=(0.078, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=accent_red,
        name="upper_clamp_ring",
    )
    upper_wand.visual(
        Box((0.026, 0.038, 0.018)),
        origin=Origin(xyz=(0.267, 0.0, 0.0)),
        material=collar_black,
        name="elbow_bridge",
    )
    upper_wand.visual(
        Box((0.030, 0.010, 0.042)),
        origin=Origin(xyz=(0.295, 0.024, 0.0)),
        material=collar_black,
        name="elbow_left_ear",
    )
    upper_wand.visual(
        Box((0.030, 0.010, 0.042)),
        origin=Origin(xyz=(0.295, -0.024, 0.0)),
        material=collar_black,
        name="elbow_right_ear",
    )

    lower_wand = model.part("lower_wand")
    lower_wand.visual(
        Cylinder(radius=0.010, length=0.038),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=collar_black,
        name="elbow_lug",
    )
    lower_wand.visual(
        Cylinder(radius=0.018, length=0.330),
        origin=Origin(xyz=(0.175, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=tube_silver,
        name="lower_tube",
    )
    lower_wand.visual(
        Cylinder(radius=0.021, length=0.038),
        origin=Origin(xyz=(0.078, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=accent_red,
        name="lower_clamp_ring",
    )
    lower_wand.visual(
        Box((0.024, 0.038, 0.018)),
        origin=Origin(xyz=(0.338, 0.0, 0.0)),
        material=collar_black,
        name="neck_bridge",
    )
    lower_wand.visual(
        Box((0.022, 0.010, 0.040)),
        origin=Origin(xyz=(0.361, 0.024, 0.0)),
        material=collar_black,
        name="neck_left_ear",
    )
    lower_wand.visual(
        Box((0.022, 0.010, 0.040)),
        origin=Origin(xyz=(0.361, -0.024, 0.0)),
        material=collar_black,
        name="neck_right_ear",
    )

    floor_nozzle = model.part("floor_nozzle")
    floor_nozzle.visual(
        Cylinder(radius=0.010, length=0.038),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=collar_black,
        name="neck_lug",
    )
    floor_nozzle.visual(
        Box((0.060, 0.024, 0.022)),
        origin=Origin(xyz=(0.040, 0.0, -0.010)),
        material=collar_black,
        name="neck_stem",
    )
    floor_nozzle.visual(
        Box((0.060, 0.060, 0.022)),
        origin=Origin(xyz=(0.090, 0.0, -0.024)),
        material=collar_black,
        name="neck_transition",
    )
    floor_nozzle.visual(
        Box((0.280, 0.108, 0.026)),
        origin=Origin(xyz=(0.170, 0.0, -0.036)),
        material=body_gray,
        name="nozzle_shell",
    )
    floor_nozzle.visual(
        Box((0.220, 0.098, 0.006)),
        origin=Origin(xyz=(0.190, 0.0, -0.050)),
        material=collar_black,
        name="intake_lip",
    )

    model.articulation(
        "body_to_upper_wand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=upper_wand,
        origin=Origin(rpy=(0.0, 0.55, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=-0.55,
            upper=0.75,
        ),
    )
    model.articulation(
        "upper_to_lower_wand",
        ArticulationType.REVOLUTE,
        parent=upper_wand,
        child=lower_wand,
        origin=Origin(xyz=(0.316, 0.0, 0.0), rpy=(0.0, 0.20, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=-0.45,
            upper=0.95,
        ),
    )
    model.articulation(
        "lower_to_floor_nozzle",
        ArticulationType.REVOLUTE,
        parent=lower_wand,
        child=floor_nozzle,
        origin=Origin(xyz=(0.378, 0.0, 0.0), rpy=(0.0, -0.78, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=2.2,
            lower=-0.55,
            upper=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    upper_wand = object_model.get_part("upper_wand")
    lower_wand = object_model.get_part("lower_wand")
    floor_nozzle = object_model.get_part("floor_nozzle")
    shoulder = object_model.get_articulation("body_to_upper_wand")
    elbow = object_model.get_articulation("upper_to_lower_wand")
    neck = object_model.get_articulation("lower_to_floor_nozzle")

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
        "all_pitch_axes_point_through_collars",
        shoulder.axis == (0.0, -1.0, 0.0)
        and elbow.axis == (0.0, -1.0, 0.0)
        and neck.axis == (0.0, -1.0, 0.0),
        f"axes were shoulder={shoulder.axis}, elbow={elbow.axis}, neck={neck.axis}",
    )
    ctx.check(
        "joint_limits_are_ordered",
        all(
            joint.motion_limits is not None
            and joint.motion_limits.lower is not None
            and joint.motion_limits.upper is not None
            and joint.motion_limits.lower < joint.motion_limits.upper
            for joint in (shoulder, elbow, neck)
        ),
        "each revolute joint needs ordered lower and upper motion limits",
    )
    ctx.expect_contact(body, upper_wand, name="body_mount_contacts_upper_wand")
    ctx.expect_contact(upper_wand, lower_wand, name="elbow_mount_contacts_lower_wand")
    ctx.expect_contact(lower_wand, floor_nozzle, name="neck_mount_contacts_floor_nozzle")
    ctx.expect_origin_gap(body, floor_nozzle, axis="z", min_gap=0.40, name="body_sits_above_nozzle")
    ctx.expect_origin_gap(
        floor_nozzle,
        body,
        axis="x",
        min_gap=0.08,
        name="nozzle_projects_forward_of_body",
    )
    ctx.expect_origin_gap(
        upper_wand,
        lower_wand,
        axis="z",
        min_gap=0.10,
        name="lower_segment_hangs_below_upper_segment",
    )

    closed_nozzle_pos = ctx.part_world_position(floor_nozzle)
    with ctx.pose({shoulder: 0.45, elbow: 0.55, neck: 0.15}):
        opened_nozzle_pos = ctx.part_world_position(floor_nozzle)
        ctx.expect_origin_gap(
            floor_nozzle,
            body,
            axis="x",
            min_gap=0.28,
            name="opened_wand_reaches_forward",
        )
    ctx.check(
        "wand_opens_forward_when_joints_increase",
        closed_nozzle_pos is not None
        and opened_nozzle_pos is not None
        and opened_nozzle_pos[0] > closed_nozzle_pos[0] + 0.12,
        f"closed nozzle position={closed_nozzle_pos}, opened nozzle position={opened_nozzle_pos}",
    )

    ctx.warn_if_articulation_overlaps(max_pose_samples=24)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
