from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _segment_capsule(name: str, radius: float, body_length: float):
    return mesh_from_geometry(CapsuleGeometry(radius, body_length), name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_robotic_leg")

    dark_frame = model.material("dark_frame", rgba=(0.16, 0.17, 0.19, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.33, 0.36, 0.39, 1.0))
    shell_blue = model.material("shell_blue", rgba=(0.27, 0.41, 0.54, 1.0))
    machined = model.material("machined", rgba=(0.67, 0.70, 0.73, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.09, 0.10, 1.0))

    hip_base = model.part("hip_base")
    hip_base.visual(
        Box((0.18, 0.14, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=dark_frame,
        name="base_plate",
    )
    hip_base.visual(
        Box((0.10, 0.09, 0.070)),
        origin=Origin(xyz=(-0.015, 0.0, 0.049)),
        material=gunmetal,
        name="dock_body",
    )
    hip_base.visual(
        Box((0.050, 0.072, 0.240)),
        origin=Origin(xyz=(-0.010, 0.0, 0.204)),
        material=dark_frame,
        name="spine_column",
    )
    hip_base.visual(
        Box((0.040, 0.074, 0.024)),
        origin=Origin(xyz=(0.000, 0.0, 0.336)),
        material=gunmetal,
        name="hip_bridge",
    )
    hip_base.visual(
        Box((0.034, 0.060, 0.060)),
        origin=Origin(xyz=(0.015, 0.0, 0.378)),
        material=dark_frame,
        name="upper_mast",
    )
    hip_base.visual(
        Box((0.050, 0.010, 0.020)),
        origin=Origin(xyz=(0.028, 0.026, 0.358)),
        material=dark_frame,
        name="hip_strut_left",
    )
    hip_base.visual(
        Box((0.050, 0.010, 0.020)),
        origin=Origin(xyz=(0.028, -0.026, 0.358)),
        material=dark_frame,
        name="hip_strut_right",
    )
    hip_base.visual(
        Box((0.018, 0.012, 0.052)),
        origin=Origin(xyz=(0.050, 0.036, 0.382)),
        material=machined,
        name="hip_yoke_left",
    )
    hip_base.visual(
        Box((0.018, 0.012, 0.052)),
        origin=Origin(xyz=(0.050, -0.036, 0.382)),
        material=machined,
        name="hip_yoke_right",
    )

    thigh = model.part("thigh")
    thigh.visual(
        Box((0.032, 0.038, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, -0.016)),
        material=gunmetal,
        name="hip_shoulder_block",
    )
    thigh.visual(
        Box((0.050, 0.044, 0.110)),
        origin=Origin(xyz=(0.0, 0.0, -0.087)),
        material=shell_blue,
        name="thigh_shell",
    )
    thigh.visual(
        Box((0.054, 0.010, 0.082)),
        origin=Origin(xyz=(0.0, 0.027, -0.083)),
        material=gunmetal,
        name="thigh_bay_left",
    )
    thigh.visual(
        Box((0.054, 0.010, 0.082)),
        origin=Origin(xyz=(0.0, -0.027, -0.083)),
        material=gunmetal,
        name="thigh_bay_right",
    )
    thigh.visual(
        Box((0.012, 0.046, 0.024)),
        origin=Origin(xyz=(-0.028, 0.0, -0.154)),
        material=dark_frame,
        name="thigh_lower_brace",
    )
    thigh.visual(
        Box((0.018, 0.010, 0.040)),
        origin=Origin(xyz=(0.0, 0.034, -0.162)),
        material=machined,
        name="knee_yoke_left",
    )
    thigh.visual(
        Box((0.018, 0.010, 0.040)),
        origin=Origin(xyz=(0.0, -0.034, -0.162)),
        material=machined,
        name="knee_yoke_right",
    )
    thigh.visual(
        Box((0.018, 0.058, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.142)),
        material=gunmetal,
        name="knee_mount_bridge",
    )

    shank = model.part("shank")
    shank.visual(
        Box((0.030, 0.058, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, -0.016)),
        material=gunmetal,
        name="knee_block",
    )
    shank.visual(
        Box((0.046, 0.042, 0.096)),
        origin=Origin(xyz=(0.0, 0.0, -0.080)),
        material=shell_blue,
        name="shank_shell",
    )
    shank.visual(
        Box((0.050, 0.010, 0.070)),
        origin=Origin(xyz=(0.0, 0.026, -0.078)),
        material=gunmetal,
        name="shank_bay_left",
    )
    shank.visual(
        Box((0.050, 0.010, 0.070)),
        origin=Origin(xyz=(0.0, -0.026, -0.078)),
        material=gunmetal,
        name="shank_bay_right",
    )
    shank.visual(
        Box((0.028, 0.056, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, -0.140)),
        material=dark_frame,
        name="ankle_fairing",
    )
    shank.visual(
        Box((0.018, 0.010, 0.036)),
        origin=Origin(xyz=(0.0, 0.033, -0.150)),
        material=machined,
        name="ankle_yoke_left",
    )
    shank.visual(
        Box((0.018, 0.010, 0.036)),
        origin=Origin(xyz=(0.0, -0.033, -0.150)),
        material=machined,
        name="ankle_yoke_right",
    )

    foot = model.part("foot")
    foot.visual(
        Box((0.028, 0.034, 0.028)),
        origin=Origin(xyz=(0.014, 0.0, -0.014)),
        material=gunmetal,
        name="ankle_block",
    )
    foot.visual(
        Box((0.024, 0.048, 0.016)),
        origin=Origin(xyz=(-0.002, 0.0, -0.036)),
        material=dark_frame,
        name="heel_counter",
    )
    foot.visual(
        Box((0.060, 0.032, 0.020)),
        origin=Origin(xyz=(0.038, 0.0, -0.038)),
        material=gunmetal,
        name="arch_web",
    )
    foot.visual(
        Box((0.110, 0.064, 0.010)),
        origin=Origin(xyz=(0.060, 0.0, -0.049)),
        material=shell_blue,
        name="sole_body",
    )
    foot.visual(
        Box((0.032, 0.052, 0.014)),
        origin=Origin(xyz=(0.102, 0.0, -0.037)),
        material=shell_blue,
        name="toe_shell",
    )
    foot.visual(
        Box((0.110, 0.064, 0.004)),
        origin=Origin(xyz=(0.060, 0.0, -0.056)),
        material=rubber,
        name="sole_pad",
    )

    model.articulation(
        "hip_pitch",
        ArticulationType.REVOLUTE,
        parent=hip_base,
        child=thigh,
        origin=Origin(xyz=(0.050, 0.0, 0.382)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=2.2,
            lower=-0.55,
            upper=1.75,
        ),
    )
    model.articulation(
        "knee_pitch",
        ArticulationType.REVOLUTE,
        parent=thigh,
        child=shank,
        origin=Origin(xyz=(0.0, 0.0, -0.166)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=38.0,
            velocity=2.6,
            lower=0.0,
            upper=2.30,
        ),
    )
    model.articulation(
        "ankle_pitch",
        ArticulationType.REVOLUTE,
        parent=shank,
        child=foot,
        origin=Origin(xyz=(0.0, 0.0, -0.152)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=24.0,
            velocity=3.0,
            lower=-0.85,
            upper=0.75,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    hip_base = object_model.get_part("hip_base")
    thigh = object_model.get_part("thigh")
    shank = object_model.get_part("shank")
    foot = object_model.get_part("foot")
    hip = object_model.get_articulation("hip_pitch")
    knee = object_model.get_articulation("knee_pitch")
    ankle = object_model.get_articulation("ankle_pitch")

    def _aabb_center(aabb):
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    def _union_aabb(part_aabbs):
        mins = [min(aabb[0][i] for aabb in part_aabbs) for i in range(3)]
        maxs = [max(aabb[1][i] for aabb in part_aabbs) for i in range(3)]
        return (tuple(mins), tuple(maxs))

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
        "serial_joint_axes_and_limits",
        hip.axis == (0.0, -1.0, 0.0)
        and knee.axis == (0.0, -1.0, 0.0)
        and ankle.axis == (0.0, -1.0, 0.0)
        and hip.motion_limits is not None
        and knee.motion_limits is not None
        and ankle.motion_limits is not None
        and hip.motion_limits.lower <= -0.5
        and hip.motion_limits.upper >= 1.7
        and knee.motion_limits.lower == 0.0
        and knee.motion_limits.upper >= 2.2
        and ankle.motion_limits.lower <= -0.8
        and ankle.motion_limits.upper >= 0.7,
        "hip-knee-ankle chain should be a serial pitch stack with plausible compact-folding ranges",
    )
    ctx.expect_contact(hip_base, thigh, name="hip_bearing_contact")
    ctx.expect_contact(thigh, shank, name="knee_bearing_contact")
    ctx.expect_contact(shank, foot, name="ankle_bearing_contact")

    with ctx.pose({hip: 0.0, knee: 0.0, ankle: 0.0}):
        sole_pad_aabb = ctx.part_element_world_aabb(foot, elem="sole_pad")
        thigh_origin = ctx.part_world_position(thigh)
        knee_origin = ctx.part_world_position(shank)
        ankle_origin = ctx.part_world_position(foot)
        neutral_chain_descends = (
            thigh_origin is not None
            and knee_origin is not None
            and ankle_origin is not None
            and thigh_origin[2] > knee_origin[2] > ankle_origin[2]
        )
        ctx.check(
            "default_link_orientations",
            neutral_chain_descends,
            (
                "neutral pose should hang downward from hip to knee to ankle; "
                f"thigh={thigh_origin}, knee={knee_origin}, ankle={ankle_origin}"
            ),
        )
        ctx.check(
            "neutral_foot_near_tabletop",
            sole_pad_aabb is not None and -0.002 <= sole_pad_aabb[0][2] <= 0.018,
            (
                "desktop neutral pose should place the rubber pad close to the support plane; "
                f"got min_z={None if sole_pad_aabb is None else sole_pad_aabb[0][2]:.4f}"
                if sole_pad_aabb is not None
                else "missing sole pad AABB"
            ),
        )

    with ctx.pose({hip: 1.10, knee: 0.0, ankle: 0.0}):
        hip_flex_knee_origin = ctx.part_world_position(shank)
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_hip_flex_pose")

    ctx.check(
        "hip_flex_moves_chain_forward_and_up",
        knee_origin is not None
        and hip_flex_knee_origin is not None
        and hip_flex_knee_origin[0] > knee_origin[0] + 0.10
        and hip_flex_knee_origin[2] > knee_origin[2] + 0.06,
        (
            "positive hip motion should swing the knee forward/up for stow behavior; "
            f"neutral={knee_origin}, flexed={hip_flex_knee_origin}"
        ),
    )

    with ctx.pose({hip: 0.0, knee: 1.45, ankle: 0.0}):
        knee_flex_ankle_origin = ctx.part_world_position(foot)
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_knee_flex_pose")

    ctx.check(
        "knee_flex_tucks_distal_chain",
        ankle_origin is not None
        and knee_flex_ankle_origin is not None
        and knee_flex_ankle_origin[0] > ankle_origin[0] + 0.09
        and knee_flex_ankle_origin[2] > ankle_origin[2] + 0.10,
        (
            "knee flexion should pull the ankle upward and inward for compact stow; "
            f"neutral={ankle_origin}, flexed={knee_flex_ankle_origin}"
        ),
    )

    with ctx.pose({hip: 0.2, knee: 0.7, ankle: -0.45}):
        plantar_toe = ctx.part_element_world_aabb(foot, elem="toe_shell")
    with ctx.pose({hip: 0.2, knee: 0.7, ankle: 0.45}):
        dorsi_toe = ctx.part_element_world_aabb(foot, elem="toe_shell")

    ctx.check(
        "ankle_reorients_foot",
        plantar_toe is not None
        and dorsi_toe is not None
        and _aabb_center(dorsi_toe)[2] > _aabb_center(plantar_toe)[2] + 0.020,
        (
            "ankle articulation should visibly lift the toe in dorsiflexion; "
            f"plantar={None if plantar_toe is None else _aabb_center(plantar_toe)}, "
            f"dorsi={None if dorsi_toe is None else _aabb_center(dorsi_toe)}"
        ),
    )

    with ctx.pose({hip: 1.20, knee: 1.55, ankle: -0.35}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_stowed_pose")
        base_plate_aabb = ctx.part_element_world_aabb(hip_base, elem="base_plate")
        foot_aabb = ctx.part_world_aabb(foot)
        model_aabb = _union_aabb(
            [
                ctx.part_world_aabb(hip_base),
                ctx.part_world_aabb(thigh),
                ctx.part_world_aabb(shank),
                ctx.part_world_aabb(foot),
            ]
        )
        compact_ok = (
            base_plate_aabb is not None
            and foot_aabb is not None
            and foot_aabb[0][2] > base_plate_aabb[1][2] + 0.020
            and (model_aabb[1][0] - model_aabb[0][0]) <= 0.40
        )
        ctx.check(
            "stowed_pose_is_compact",
            compact_ok,
            (
                "folded pose should lift the foot above the desktop and keep the x envelope compact; "
                f"base={base_plate_aabb}, foot={foot_aabb}, model_span_x={model_aabb[1][0] - model_aabb[0][0]:.4f}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
