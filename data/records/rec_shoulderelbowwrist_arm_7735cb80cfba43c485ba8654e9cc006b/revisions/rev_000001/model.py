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


BASE_FOOT_RADIUS = 0.11
BASE_FOOT_HEIGHT = 0.02
BASE_PEDESTAL_RADIUS = 0.08
BASE_PEDESTAL_HEIGHT = 0.09
BASE_CAP_RADIUS = 0.062
BASE_CAP_HEIGHT = 0.012
BASE_TOTAL_HEIGHT = BASE_FOOT_HEIGHT + BASE_PEDESTAL_HEIGHT

SHOULDER_TURNTABLE_RADIUS = 0.055
SHOULDER_TURNTABLE_HEIGHT = 0.032
SHOULDER_MAST_SIZE = (0.08, 0.07, 0.09)
UPPER_BEAM_SIZE = (0.14, 0.06, 0.055)
UPPER_BEAM_CENTER = (0.115, 0.0, 0.09)
ELBOW_ORIGIN_X = 0.25
ELBOW_ORIGIN_Z = 0.09
ELBOW_YOKE_GAP = 0.034
ELBOW_YOKE_THICKNESS = 0.016
ELBOW_YOKE_SIZE = (0.03, ELBOW_YOKE_THICKNESS, 0.078)
ELBOW_YOKE_CENTER_X = 0.22
ELBOW_YOKE_CENTER_Z = ELBOW_ORIGIN_Z
ELBOW_YOKE_OFFSET_Y = (ELBOW_YOKE_GAP + ELBOW_YOKE_THICKNESS) / 2.0
ELBOW_MOTOR_SIZE = (0.04, ELBOW_YOKE_GAP + 2.0 * ELBOW_YOKE_THICKNESS, 0.05)
ELBOW_MOTOR_CENTER = (0.19, 0.0, ELBOW_ORIGIN_Z)

ELBOW_PIVOT_SIZE = (0.06, ELBOW_YOKE_GAP, 0.05)
FOREARM_NECK_SIZE = (0.05, 0.028, 0.03)
FOREARM_NECK_CENTER = (0.025, 0.0, 0.0)
FOREARM_BEAM_SIZE = (0.13, 0.05, 0.045)
FOREARM_BEAM_CENTER = (0.115, 0.0, 0.0)
WRIST_MOUNT_SIZE = (0.04, 0.065, 0.065)
WRIST_MOUNT_CENTER = (0.20, 0.0, 0.0)
WRIST_ORIGIN_X = 0.22

WRIST_ROTOR_RADIUS = 0.032
WRIST_ROTOR_LENGTH = 0.02
WRIST_HEAD_BOX_SIZE = (0.055, 0.055, 0.055)
WRIST_HEAD_BOX_CENTER = (0.0475, 0.0, 0.0)
WRIST_BODY_RADIUS = 0.028
WRIST_BODY_LENGTH = 0.055
WRIST_BODY_CENTER_X = 0.0625
TOOL_FLANGE_RADIUS = 0.04
TOOL_FLANGE_LENGTH = 0.008
TOOL_FLANGE_CENTER_X = 0.094
TOOL_SPIGOT_RADIUS = 0.014
TOOL_SPIGOT_LENGTH = 0.014
TOOL_SPIGOT_CENTER_X = 0.105


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_robotic_arm")

    dark_trim = model.material("dark_trim", rgba=(0.13, 0.14, 0.16, 1.0))
    base_gray = model.material("base_gray", rgba=(0.24, 0.26, 0.29, 1.0))
    arm_orange = model.material("arm_orange", rgba=(0.92, 0.46, 0.13, 1.0))
    metal = model.material("metal", rgba=(0.77, 0.79, 0.82, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=BASE_FOOT_RADIUS, length=BASE_FOOT_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_FOOT_HEIGHT / 2.0)),
        material=dark_trim,
        name="foot_ring",
    )
    base.visual(
        Cylinder(radius=BASE_PEDESTAL_RADIUS, length=BASE_PEDESTAL_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_FOOT_HEIGHT + BASE_PEDESTAL_HEIGHT / 2.0)),
        material=base_gray,
        name="pedestal_shell",
    )
    base.visual(
        Cylinder(radius=BASE_CAP_RADIUS, length=BASE_CAP_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_TOTAL_HEIGHT - BASE_CAP_HEIGHT / 2.0)),
        material=dark_trim,
        name="pedestal_cap",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=SHOULDER_TURNTABLE_RADIUS, length=SHOULDER_TURNTABLE_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_TURNTABLE_HEIGHT / 2.0)),
        material=dark_trim,
        name="shoulder_turntable",
    )
    upper_arm.visual(
        Box(SHOULDER_MAST_SIZE),
        origin=Origin(xyz=(0.02, 0.0, SHOULDER_MAST_SIZE[2] / 2.0)),
        material=base_gray,
        name="shoulder_mast",
    )
    upper_arm.visual(
        Box(UPPER_BEAM_SIZE),
        origin=Origin(xyz=UPPER_BEAM_CENTER),
        material=arm_orange,
        name="upper_beam",
    )
    upper_arm.visual(
        Box(ELBOW_MOTOR_SIZE),
        origin=Origin(xyz=ELBOW_MOTOR_CENTER),
        material=base_gray,
        name="elbow_motor",
    )
    upper_arm.visual(
        Box(ELBOW_YOKE_SIZE),
        origin=Origin(
            xyz=(ELBOW_YOKE_CENTER_X, ELBOW_YOKE_OFFSET_Y, ELBOW_YOKE_CENTER_Z)
        ),
        material=arm_orange,
        name="elbow_yoke_left",
    )
    upper_arm.visual(
        Box(ELBOW_YOKE_SIZE),
        origin=Origin(
            xyz=(ELBOW_YOKE_CENTER_X, -ELBOW_YOKE_OFFSET_Y, ELBOW_YOKE_CENTER_Z)
        ),
        material=arm_orange,
        name="elbow_yoke_right",
    )

    forearm = model.part("forearm")
    forearm.visual(
        Box(ELBOW_PIVOT_SIZE),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_trim,
        name="elbow_pivot",
    )
    forearm.visual(
        Box(FOREARM_NECK_SIZE),
        origin=Origin(xyz=FOREARM_NECK_CENTER),
        material=dark_trim,
        name="forearm_neck",
    )
    forearm.visual(
        Box(FOREARM_BEAM_SIZE),
        origin=Origin(xyz=FOREARM_BEAM_CENTER),
        material=arm_orange,
        name="forearm_beam",
    )
    forearm.visual(
        Box(WRIST_MOUNT_SIZE),
        origin=Origin(xyz=WRIST_MOUNT_CENTER),
        material=base_gray,
        name="wrist_mount",
    )

    wrist = model.part("wrist")
    wrist.visual(
        Cylinder(radius=WRIST_ROTOR_RADIUS, length=WRIST_ROTOR_LENGTH),
        origin=Origin(xyz=(WRIST_ROTOR_LENGTH / 2.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_trim,
        name="wrist_rotor",
    )
    wrist.visual(
        Box(WRIST_HEAD_BOX_SIZE),
        origin=Origin(xyz=WRIST_HEAD_BOX_CENTER),
        material=base_gray,
        name="wrist_head",
    )
    wrist.visual(
        Cylinder(radius=WRIST_BODY_RADIUS, length=WRIST_BODY_LENGTH),
        origin=Origin(
            xyz=(WRIST_BODY_CENTER_X, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)
        ),
        material=base_gray,
        name="wrist_body",
    )
    wrist.visual(
        Cylinder(radius=TOOL_FLANGE_RADIUS, length=TOOL_FLANGE_LENGTH),
        origin=Origin(
            xyz=(TOOL_FLANGE_CENTER_X, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)
        ),
        material=metal,
        name="tool_flange",
    )
    wrist.visual(
        Cylinder(radius=TOOL_SPIGOT_RADIUS, length=TOOL_SPIGOT_LENGTH),
        origin=Origin(
            xyz=(TOOL_SPIGOT_CENTER_X, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)
        ),
        material=metal,
        name="tool_spigot",
    )

    model.articulation(
        "shoulder_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, BASE_TOTAL_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.8,
            lower=-2.6,
            upper=2.6,
        ),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(ELBOW_ORIGIN_X, 0.0, ELBOW_ORIGIN_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=28.0,
            velocity=2.0,
            lower=-1.0,
            upper=1.15,
        ),
    )
    model.articulation(
        "wrist_roll",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist,
        origin=Origin(xyz=(WRIST_ORIGIN_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=3.5,
            lower=-3.0,
            upper=3.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist = object_model.get_part("wrist")

    shoulder_yaw = object_model.get_articulation("shoulder_yaw")
    elbow_pitch = object_model.get_articulation("elbow_pitch")
    wrist_roll = object_model.get_articulation("wrist_roll")

    pedestal_cap = base.get_visual("pedestal_cap")
    shoulder_turntable = upper_arm.get_visual("shoulder_turntable")
    elbow_yoke_left = upper_arm.get_visual("elbow_yoke_left")
    elbow_yoke_right = upper_arm.get_visual("elbow_yoke_right")
    elbow_pivot = forearm.get_visual("elbow_pivot")
    wrist_mount = forearm.get_visual("wrist_mount")
    wrist_rotor = wrist.get_visual("wrist_rotor")
    tool_flange = wrist.get_visual("tool_flange")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.allow_overlap(
        forearm,
        upper_arm,
        elem_a=elbow_pivot,
        elem_b=elbow_yoke_left,
        reason="elbow pivot block represents a concealed trunnion running inside the left knuckle cheek",
    )
    ctx.allow_overlap(
        forearm,
        upper_arm,
        elem_a=elbow_pivot,
        elem_b=elbow_yoke_right,
        reason="elbow pivot block represents a concealed trunnion running inside the right knuckle cheek",
    )

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
        "shoulder_axis_vertical",
        tuple(shoulder_yaw.axis) == (0.0, 0.0, 1.0),
        f"shoulder axis was {shoulder_yaw.axis}",
    )
    ctx.check(
        "elbow_axis_horizontal",
        tuple(elbow_pitch.axis) == (0.0, 1.0, 0.0),
        f"elbow axis was {elbow_pitch.axis}",
    )
    ctx.check(
        "wrist_axis_tool_aligned",
        tuple(wrist_roll.axis) == (1.0, 0.0, 0.0),
        f"wrist axis was {wrist_roll.axis}",
    )
    ctx.check(
        "joint_ranges_are_realistic",
        shoulder_yaw.motion_limits is not None
        and elbow_pitch.motion_limits is not None
        and wrist_roll.motion_limits is not None
        and shoulder_yaw.motion_limits.lower == -2.6
        and shoulder_yaw.motion_limits.upper == 2.6
        and elbow_pitch.motion_limits.lower == -1.0
        and elbow_pitch.motion_limits.upper == 1.15
        and wrist_roll.motion_limits.lower == -3.0
        and wrist_roll.motion_limits.upper == 3.0,
        "unexpected articulated range on one or more joints",
    )

    with ctx.pose({shoulder_yaw: 0.0, elbow_pitch: 0.0, wrist_roll: 0.0}):
        ctx.expect_gap(
            upper_arm,
            base,
            axis="z",
            positive_elem=shoulder_turntable,
            negative_elem=pedestal_cap,
            max_gap=0.0005,
            max_penetration=0.0,
            name="shoulder_turntable_seats_on_pedestal",
        )
        ctx.expect_overlap(
            upper_arm,
            base,
            axes="xy",
            elem_a=shoulder_turntable,
            elem_b=pedestal_cap,
            min_overlap=0.09,
            name="shoulder_bearing_has_xy_footprint",
        )
        ctx.expect_contact(
            upper_arm,
            forearm,
            contact_tol=0.0005,
            name="elbow_joint_stays_mounted",
        )
        ctx.expect_gap(
            upper_arm,
            forearm,
            axis="y",
            positive_elem=elbow_yoke_left,
            negative_elem=elbow_pivot,
            max_gap=0.0005,
            max_penetration=0.0,
            name="left_elbow_trunnion_face_contact",
        )
        ctx.expect_gap(
            forearm,
            upper_arm,
            axis="y",
            positive_elem=elbow_pivot,
            negative_elem=elbow_yoke_right,
            max_gap=0.0005,
            max_penetration=0.0,
            name="right_elbow_trunnion_face_contact",
        )
        ctx.expect_overlap(
            upper_arm,
            forearm,
            axes="x",
            elem_a=elbow_yoke_left,
            elem_b=elbow_pivot,
            min_overlap=0.012,
            name="elbow_knuckle_has_x_engagement",
        )
        ctx.expect_overlap(
            upper_arm,
            forearm,
            axes="z",
            elem_a=elbow_yoke_left,
            elem_b=elbow_pivot,
            min_overlap=0.045,
            name="elbow_knuckle_has_z_engagement",
        )
        ctx.expect_gap(
            wrist,
            forearm,
            axis="x",
            positive_elem=wrist_rotor,
            negative_elem=wrist_mount,
            max_gap=0.0005,
            max_penetration=1e-6,
            name="wrist_rotor_seats_on_mount",
        )
        ctx.expect_overlap(
            wrist,
            forearm,
            axes="yz",
            elem_a=wrist_rotor,
            elem_b=wrist_mount,
            min_overlap=0.05,
            name="wrist_mount_has_bearing_overlap",
        )
        ctx.expect_origin_gap(
            wrist,
            base,
            axis="x",
            min_gap=0.42,
            max_gap=0.49,
            name="rest_pose_reach_is_compact",
        )

        aabbs = [
            ctx.part_world_aabb(part)
            for part in (base, upper_arm, forearm, wrist)
        ]
        if all(aabb is not None for aabb in aabbs):
            mins = [min(aabb[0][i] for aabb in aabbs if aabb is not None) for i in range(3)]
            maxs = [max(aabb[1][i] for aabb in aabbs if aabb is not None) for i in range(3)]
            overall_dims = tuple(maxs[i] - mins[i] for i in range(3))
            ctx.check(
                "overall_envelope_reads_as_compact_arm",
                0.64 <= overall_dims[0] <= 0.74
                and 0.21 <= overall_dims[1] <= 0.24
                and 0.20 <= overall_dims[2] <= 0.245,
                f"overall envelope was {overall_dims}",
            )
        else:
            ctx.fail("overall_envelope_reads_as_compact_arm", "could not resolve one or more part AABBs")

        flange_aabb = ctx.part_element_world_aabb(wrist, elem=tool_flange)
        if flange_aabb is not None:
            flange_diameter_y = flange_aabb[1][1] - flange_aabb[0][1]
            flange_diameter_z = flange_aabb[1][2] - flange_aabb[0][2]
            ctx.check(
                "tool_flange_is_round_and_robotic",
                0.078 <= flange_diameter_y <= 0.082
                and 0.078 <= flange_diameter_z <= 0.082,
                f"tool flange projected diameters were {(flange_diameter_y, flange_diameter_z)}",
            )
        else:
            ctx.fail("tool_flange_is_round_and_robotic", "could not resolve tool flange AABB")

    for joint, parent, child in (
        (shoulder_yaw, base, upper_arm),
        (elbow_pitch, upper_arm, forearm),
        (wrist_roll, forearm, wrist),
    ):
        limits = joint.motion_limits
        if limits is None or limits.lower is None or limits.upper is None:
            ctx.fail(f"{joint.name}_limits_present", "bounded revolute joint was missing lower/upper limits")
            continue
        with ctx.pose({joint: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_lower_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{joint.name}_lower_no_floating")
            ctx.expect_contact(
                parent,
                child,
                contact_tol=0.0005,
                name=f"{joint.name}_lower_parent_child_contact",
            )
        with ctx.pose({joint: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_upper_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{joint.name}_upper_no_floating")
            ctx.expect_contact(
                parent,
                child,
                contact_tol=0.0005,
                name=f"{joint.name}_upper_parent_child_contact",
            )

    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=24,
        ignore_adjacent=False,
        ignore_fixed=True,
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
