from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
from pathlib import Path

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)
HERE = Path(__file__).resolve().parent


def _build_shade_shell():
    outer_profile = [
        (0.020, 0.000),
        (0.022, 0.008),
        (0.034, 0.036),
        (0.046, 0.086),
        (0.050, 0.118),
    ]
    inner_profile = [
        (0.012, 0.004),
        (0.014, 0.012),
        (0.028, 0.038),
        (0.041, 0.086),
        (0.046, 0.114),
    ]
    shell_geometry = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=56,
        start_cap="flat",
        end_cap="flat",
    )
    return mesh_from_geometry(shell_geometry, ASSETS.mesh_path("task_lamp_shade_shell.obj"))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clamp_task_lamp", assets=ASSETS)

    painted_steel = model.material("painted_steel", rgba=(0.17, 0.19, 0.21, 1.0))
    charcoal = model.material("charcoal", rgba=(0.10, 0.11, 0.12, 1.0))
    off_white = model.material("off_white", rgba=(0.90, 0.89, 0.84, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.71, 0.74, 0.77, 1.0))
    warm_bulb = model.material("warm_bulb", rgba=(0.96, 0.92, 0.74, 0.88))
    clamp_pad = model.material("clamp_pad", rgba=(0.30, 0.31, 0.33, 1.0))

    clamp_body = model.part("clamp_body")
    clamp_body.visual(
        Box((0.028, 0.058, 0.132)),
        origin=Origin(xyz=(-0.016, 0.0, 0.066)),
        material=painted_steel,
        name="rear_spine",
    )
    clamp_body.visual(
        Box((0.050, 0.060, 0.016)),
        origin=Origin(xyz=(0.000, 0.0, 0.018)),
        material=painted_steel,
        name="lower_jaw",
    )
    clamp_body.visual(
        Box((0.094, 0.056, 0.014)),
        origin=Origin(xyz=(0.028, 0.0, 0.108)),
        material=painted_steel,
        name="upper_jaw",
    )
    clamp_body.visual(
        Box((0.018, 0.048, 0.030)),
        origin=Origin(xyz=(-0.004, 0.0, 0.088)),
        material=painted_steel,
        name="jaw_brace",
    )
    clamp_body.visual(
        Box((0.040, 0.044, 0.005)),
        origin=Origin(xyz=(-0.002, 0.0, 0.0285)),
        material=clamp_pad,
        name="lower_pad",
    )
    clamp_body.visual(
        Box((0.042, 0.044, 0.005)),
        origin=Origin(xyz=(0.040, 0.0, 0.0985)),
        material=clamp_pad,
        name="upper_pad",
    )
    clamp_body.visual(
        Cylinder(radius=0.010, length=0.016),
        origin=Origin(xyz=(0.040, 0.0, 0.093)),
        material=brushed_metal,
        name="screw_boss",
    )
    clamp_body.visual(
        Cylinder(radius=0.010, length=0.118),
        origin=Origin(xyz=(-0.018, 0.0, 0.191)),
        material=painted_steel,
        name="post",
    )
    clamp_body.visual(
        Box((0.014, 0.028, 0.018)),
        origin=Origin(xyz=(-0.018, 0.0, 0.259)),
        material=painted_steel,
        name="shoulder_tower",
    )
    clamp_body.visual(
        Box((0.038, 0.050, 0.010)),
        origin=Origin(xyz=(-0.002, 0.0, 0.269)),
        material=painted_steel,
        name="clevis_bridge",
    )
    clamp_body.visual(
        Box((0.018, 0.008, 0.034)),
        origin=Origin(xyz=(0.008, -0.019, 0.249)),
        material=painted_steel,
        name="clevis_left",
    )
    clamp_body.visual(
        Box((0.018, 0.008, 0.034)),
        origin=Origin(xyz=(0.008, 0.019, 0.249)),
        material=painted_steel,
        name="clevis_right",
    )
    clamp_body.inertial = Inertial.from_geometry(
        Box((0.13, 0.08, 0.28)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
    )

    clamp_screw = model.part("clamp_screw")
    clamp_screw.visual(
        Cylinder(radius=0.010, length=0.008),
        origin=Origin(),
        material=brushed_metal,
        name="guide_collar",
    )
    clamp_screw.visual(
        Cylinder(radius=0.006, length=0.074),
        origin=Origin(xyz=(0.0, 0.0, -0.041)),
        material=brushed_metal,
        name="screw_shaft",
    )
    clamp_screw.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.051)),
        material=brushed_metal,
        name="pressure_pad",
    )
    clamp_screw.visual(
        Cylinder(radius=0.017, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.087)),
        material=charcoal,
        name="thumbwheel",
    )
    clamp_screw.inertial = Inertial.from_geometry(
        Box((0.040, 0.040, 0.110)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, -0.045)),
    )

    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=0.009, length=0.030),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=painted_steel,
        name="shoulder_hub",
    )
    arm.visual(
        Box((0.028, 0.018, 0.014)),
        origin=Origin(xyz=(0.014, 0.0, -0.006)),
        material=painted_steel,
        name="shoulder_block",
    )
    arm.visual(
        Box((0.260, 0.018, 0.018)),
        origin=Origin(xyz=(0.158, 0.0, 0.002)),
        material=painted_steel,
        name="main_beam",
    )
    arm.visual(
        Box((0.016, 0.016, 0.030)),
        origin=Origin(xyz=(0.080, 0.0, -0.015)),
        material=painted_steel,
        name="rear_spring_hanger",
    )
    arm.visual(
        Box((0.016, 0.016, 0.034)),
        origin=Origin(xyz=(0.220, 0.0, -0.013)),
        material=painted_steel,
        name="front_spring_hanger",
    )
    arm.visual(
        Cylinder(radius=0.005, length=0.140),
        origin=Origin(xyz=(0.150, 0.0, -0.026), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_metal,
        name="balance_spring",
    )
    arm.visual(
        Box((0.024, 0.046, 0.014)),
        origin=Origin(xyz=(0.282, 0.0, 0.016)),
        material=painted_steel,
        name="tip_bridge",
    )
    arm.visual(
        Box((0.018, 0.008, 0.032)),
        origin=Origin(xyz=(0.301, -0.019, 0.0)),
        material=painted_steel,
        name="fork_left",
    )
    arm.visual(
        Box((0.018, 0.008, 0.032)),
        origin=Origin(xyz=(0.301, 0.019, 0.0)),
        material=painted_steel,
        name="fork_right",
    )
    arm.inertial = Inertial.from_geometry(
        Box((0.32, 0.06, 0.08)),
        mass=0.85,
        origin=Origin(xyz=(0.160, 0.0, -0.005)),
    )

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=0.009, length=0.030),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=painted_steel,
        name="trunnion",
    )
    shade.visual(
        Box((0.012, 0.026, 0.018)),
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
        material=painted_steel,
        name="pivot_block",
    )
    shade.visual(
        Cylinder(radius=0.013, length=0.030),
        origin=Origin(xyz=(0.021, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="socket_neck",
    )
    shade.visual(
        _build_shade_shell(),
        origin=Origin(xyz=(0.024, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=off_white,
        name="shade_shell",
    )
    shade.visual(
        Cylinder(radius=0.005, length=0.036),
        origin=Origin(xyz=(0.048, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_metal,
        name="bulb_stem",
    )
    shade.visual(
        Sphere(radius=0.016),
        origin=Origin(xyz=(0.078, 0.0, 0.0)),
        material=warm_bulb,
        name="bulb",
    )
    shade.inertial = Inertial.from_geometry(
        Box((0.16, 0.10, 0.10)),
        mass=0.30,
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
    )

    model.articulation(
        "clamp_screw_spin",
        ArticulationType.CONTINUOUS,
        parent=clamp_body,
        child=clamp_screw,
        origin=Origin(xyz=(0.040, 0.0, 0.081)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )
    model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent=clamp_body,
        child=arm,
        origin=Origin(xyz=(0.008, 0.0, 0.249)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=1.8,
            lower=-0.45,
            upper=0.90,
        ),
    )
    model.articulation(
        "shade_tilt",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=shade,
        origin=Origin(xyz=(0.301, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.5,
            lower=-0.80,
            upper=0.95,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)

    clamp_body = object_model.get_part("clamp_body")
    clamp_screw = object_model.get_part("clamp_screw")
    arm = object_model.get_part("arm")
    shade = object_model.get_part("shade")

    clamp_screw_spin = object_model.get_articulation("clamp_screw_spin")
    shoulder_pitch = object_model.get_articulation("shoulder_pitch")
    shade_tilt = object_model.get_articulation("shade_tilt")

    upper_jaw = clamp_body.get_visual("upper_jaw")
    lower_jaw = clamp_body.get_visual("lower_jaw")
    upper_pad = clamp_body.get_visual("upper_pad")
    lower_pad = clamp_body.get_visual("lower_pad")
    screw_boss = clamp_body.get_visual("screw_boss")
    clevis_left = clamp_body.get_visual("clevis_left")
    clevis_right = clamp_body.get_visual("clevis_right")

    guide_collar = clamp_screw.get_visual("guide_collar")
    pressure_pad = clamp_screw.get_visual("pressure_pad")

    shoulder_hub = arm.get_visual("shoulder_hub")
    fork_left = arm.get_visual("fork_left")
    fork_right = arm.get_visual("fork_right")

    trunnion = shade.get_visual("trunnion")
    shade_shell = shade.get_visual("shade_shell")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=96)

    ctx.expect_gap(
        clamp_body,
        clamp_body,
        axis="z",
        min_gap=0.060,
        max_gap=0.070,
        positive_elem=upper_pad,
        negative_elem=lower_pad,
        name="clamp_mouth_gap",
    )
    ctx.expect_contact(
        clamp_screw,
        clamp_body,
        elem_a=guide_collar,
        elem_b=screw_boss,
        name="clamp_screw_seated_in_boss",
    )
    ctx.expect_gap(
        clamp_screw,
        clamp_body,
        axis="x",
        min_gap=0.0005,
        max_gap=0.006,
        positive_elem=pressure_pad,
        negative_elem=lower_jaw,
        name="pressure_pad_ahead_of_lower_jaw",
    )
    ctx.expect_contact(
        arm,
        clamp_body,
        elem_a=shoulder_hub,
        elem_b=clevis_left,
        name="left_clevis_contacts_hub",
    )
    ctx.expect_contact(
        arm,
        clamp_body,
        elem_a=shoulder_hub,
        elem_b=clevis_right,
        name="right_clevis_contacts_hub",
    )
    ctx.expect_contact(
        shade,
        arm,
        elem_a=trunnion,
        elem_b=fork_left,
        name="left_fork_contacts_trunnion",
    )
    ctx.expect_contact(
        shade,
        arm,
        elem_a=trunnion,
        elem_b=fork_right,
        name="right_fork_contacts_trunnion",
    )
    ctx.expect_gap(
        shade,
        clamp_body,
        axis="x",
        min_gap=0.20,
        positive_elem=shade_shell,
        negative_elem=upper_jaw,
        name="shade_projects_forward_of_clamp",
    )

    with ctx.pose({clamp_screw_spin: 1.4}):
        ctx.fail_if_parts_overlap_in_current_pose(name="clamp_screw_rotated_no_overlap")
        ctx.fail_if_isolated_parts(name="clamp_screw_rotated_no_floating")
        ctx.expect_contact(
            clamp_screw,
            clamp_body,
            elem_a=guide_collar,
            elem_b=screw_boss,
            name="clamp_screw_rotated_stays_seated",
        )

    shoulder_limits = shoulder_pitch.motion_limits
    if shoulder_limits is not None and shoulder_limits.lower is not None and shoulder_limits.upper is not None:
        with ctx.pose({shoulder_pitch: shoulder_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="shoulder_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="shoulder_lower_no_floating")
            ctx.expect_gap(
                shade,
                clamp_body,
                axis="x",
                min_gap=0.11,
                positive_elem=shade_shell,
                negative_elem=upper_jaw,
                name="shoulder_lower_head_stays_forward",
            )
        with ctx.pose({shoulder_pitch: shoulder_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="shoulder_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="shoulder_upper_no_floating")
            ctx.expect_gap(
                shade,
                clamp_body,
                axis="z",
                min_gap=0.16,
                positive_elem=shade_shell,
                negative_elem=upper_jaw,
                name="shoulder_upper_lifts_shade",
            )

    shade_limits = shade_tilt.motion_limits
    if shade_limits is not None and shade_limits.lower is not None and shade_limits.upper is not None:
        with ctx.pose({shade_tilt: shade_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="shade_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="shade_lower_no_floating")
        with ctx.pose({shade_tilt: shade_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="shade_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="shade_upper_no_floating")

    rest_pos = ctx.part_world_position(shade)
    with ctx.pose({shoulder_pitch: 0.85}):
        raised_pos = ctx.part_world_position(shade)
    shade_raise_ok = (
        rest_pos is not None
        and raised_pos is not None
        and raised_pos[2] > rest_pos[2] + 0.20
    )
    ctx.check(
        "shoulder_pitch_raises_lamp_head",
        shade_raise_ok,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    rest_shell_aabb = ctx.part_element_world_aabb(shade, elem="shade_shell")
    with ctx.pose({shade_tilt: 0.80}):
        tipped_shell_aabb = ctx.part_element_world_aabb(shade, elem="shade_shell")
        ctx.expect_gap(
            shade,
            clamp_body,
            axis="x",
            min_gap=0.16,
            positive_elem=shade_shell,
            negative_elem=upper_jaw,
            name="tilted_shade_stays_outboard",
        )
    shade_tilt_ok = (
        rest_shell_aabb is not None
        and tipped_shell_aabb is not None
        and tipped_shell_aabb[0][2] < rest_shell_aabb[0][2] - 0.020
    )
    ctx.check(
        "shade_tilt_changes_aim_downward",
        shade_tilt_ok,
        details=f"rest_aabb={rest_shell_aabb}, tipped_aabb={tipped_shell_aabb}",
    )

    with ctx.pose({shoulder_pitch: -0.18, shade_tilt: 0.72}):
        ctx.fail_if_parts_overlap_in_current_pose(name="working_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="working_pose_no_floating")
        ctx.expect_gap(
            shade,
            clamp_body,
            axis="x",
            min_gap=0.16,
            positive_elem=shade_shell,
            negative_elem=upper_jaw,
            name="working_pose_head_outboard",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
