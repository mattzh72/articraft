from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose

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
    model = ArticulatedObject(name="bench_slide_arm_module")

    rail_len = 0.90
    beam_w = 0.08
    beam_h = 0.05
    foot_len = 0.14
    foot_w = 0.10
    foot_h = 0.03
    beam_z = foot_h + beam_h / 2.0

    carriage_len = 0.34
    carriage_outer_w = 0.16
    cheek_w = 0.04
    carriage_h = 0.07
    bridge_t = 0.02
    stage_deck_len = 0.28
    stage_deck_w = 0.12
    stage_deck_t = 0.015

    turntable_r = 0.055
    turntable_t = 0.02
    shoulder_block_h = 0.10

    upper_arm_len = 0.32
    upper_arm_w = 0.05
    upper_arm_t = 0.025
    upper_arm_beam_len = 0.24
    upper_hub_r = 0.045
    upper_hub_t = 0.02

    forearm_len = 0.26
    forearm_w = 0.045
    forearm_t = 0.022
    forearm_hub_r = 0.04
    forearm_hub_t = 0.02
    wrist_mount_h = 0.028

    pad_backing_t = 0.008
    pad_t = 0.014

    rail_color = model.material("rail_color", rgba=(0.24, 0.26, 0.29, 1.0))
    carriage_color = model.material("carriage_color", rgba=(0.68, 0.70, 0.73, 1.0))
    shoulder_color = model.material("shoulder_color", rgba=(0.56, 0.58, 0.62, 1.0))
    upper_arm_color = model.material("upper_arm_color", rgba=(0.36, 0.46, 0.61, 1.0))
    forearm_color = model.material("forearm_color", rgba=(0.42, 0.52, 0.67, 1.0))
    pad_backing_color = model.material("pad_backing_color", rgba=(0.34, 0.34, 0.36, 1.0))
    pad_color = model.material("pad_color", rgba=(0.08, 0.08, 0.09, 1.0))

    base_rail = model.part("base_rail")
    base_rail.visual(
        Box((rail_len, beam_w, beam_h)),
        origin=Origin(xyz=(0.0, 0.0, beam_z)),
        material=rail_color,
        name="guide_beam",
    )
    base_rail.visual(
        Box((foot_len, foot_w, foot_h)),
        origin=Origin(xyz=(-rail_len * 0.33, 0.0, foot_h / 2.0)),
        material=rail_color,
        name="left_foot",
    )
    base_rail.visual(
        Box((foot_len, foot_w, foot_h)),
        origin=Origin(xyz=(rail_len * 0.33, 0.0, foot_h / 2.0)),
        material=rail_color,
        name="right_foot",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((carriage_len, cheek_w, carriage_h)),
        origin=Origin(xyz=(0.0, beam_w / 2.0 + cheek_w / 2.0 + 0.01, 0.015)),
        material=carriage_color,
        name="left_cheek",
    )
    carriage.visual(
        Box((carriage_len, cheek_w, carriage_h)),
        origin=Origin(xyz=(0.0, -(beam_w / 2.0 + cheek_w / 2.0 + 0.01), 0.015)),
        material=carriage_color,
        name="right_cheek",
    )
    carriage.visual(
        Box((carriage_len, carriage_outer_w, bridge_t)),
        origin=Origin(xyz=(0.0, 0.0, beam_h / 2.0 + bridge_t / 2.0)),
        material=carriage_color,
        name="top_bridge",
    )
    carriage.visual(
        Box((stage_deck_len, stage_deck_w, stage_deck_t)),
        origin=Origin(
            xyz=(0.0, 0.0, beam_h / 2.0 + bridge_t + stage_deck_t / 2.0)
        ),
        material=carriage_color,
        name="stage_deck",
    )

    shoulder_block = model.part("shoulder_block")
    shoulder_block.visual(
        Cylinder(radius=turntable_r, length=turntable_t),
        origin=Origin(xyz=(0.0, 0.0, turntable_t / 2.0)),
        material=shoulder_color,
        name="turntable",
    )
    shoulder_block.visual(
        Box((0.10, 0.10, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=shoulder_color,
        name="pedestal",
    )
    shoulder_block.visual(
        Box((0.14, 0.08, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, shoulder_block_h - 0.01)),
        material=shoulder_color,
        name="top_cap",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=upper_hub_r, length=upper_hub_t),
        origin=Origin(xyz=(0.0, 0.0, upper_hub_t / 2.0)),
        material=upper_arm_color,
        name="proximal_hub",
    )
    upper_arm.visual(
        Box((upper_arm_w, upper_arm_beam_len, upper_arm_t)),
        origin=Origin(xyz=(0.0, upper_arm_len / 2.0, upper_hub_t + upper_arm_t / 2.0)),
        material=upper_arm_color,
        name="main_beam",
    )
    upper_arm.visual(
        Cylinder(radius=upper_hub_r * 0.94, length=upper_hub_t),
        origin=Origin(xyz=(0.0, upper_arm_len, upper_hub_t / 2.0)),
        material=upper_arm_color,
        name="distal_hub",
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=forearm_hub_r, length=forearm_hub_t),
        origin=Origin(xyz=(0.0, 0.0, forearm_hub_t / 2.0)),
        material=forearm_color,
        name="proximal_hub",
    )
    forearm.visual(
        Box((forearm_w, forearm_len, forearm_t)),
        origin=Origin(xyz=(0.0, forearm_len / 2.0, forearm_hub_t + forearm_t / 2.0)),
        material=forearm_color,
        name="main_beam",
    )
    forearm.visual(
        Box((0.06, 0.05, wrist_mount_h)),
        origin=Origin(
            xyz=(0.0, forearm_len, forearm_hub_t + wrist_mount_h / 2.0)
        ),
        material=forearm_color,
        name="wrist_mount",
    )

    pad = model.part("pad")
    pad.visual(
        Box((0.11, 0.09, pad_backing_t)),
        origin=Origin(xyz=(0.0, 0.0, pad_backing_t / 2.0)),
        material=pad_backing_color,
        name="backing_plate",
    )
    pad.visual(
        Box((0.09, 0.07, pad_t)),
        origin=Origin(xyz=(0.0, 0.0, pad_backing_t + pad_t / 2.0)),
        material=pad_color,
        name="contact_pad",
    )

    model.articulation(
        "rail_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base_rail,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, beam_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.25,
            lower=-0.24,
            upper=0.24,
        ),
    )
    model.articulation(
        "carriage_to_shoulder",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=shoulder_block,
        origin=Origin(
            xyz=(0.0, 0.0, beam_h / 2.0 + bridge_t + stage_deck_t)
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.4,
            lower=-1.5,
            upper=1.5,
        ),
    )
    model.articulation(
        "shoulder_block_to_upper_arm",
        ArticulationType.FIXED,
        parent=shoulder_block,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, shoulder_block_h)),
    )
    model.articulation(
        "upper_arm_to_forearm",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.0, upper_arm_len, upper_hub_t)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.8,
            lower=-2.2,
            upper=2.2,
        ),
    )
    model.articulation(
        "forearm_to_pad",
        ArticulationType.FIXED,
        parent=forearm,
        child=pad,
        origin=Origin(
            xyz=(0.0, forearm_len, forearm_hub_t + wrist_mount_h)
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_rail = object_model.get_part("base_rail")
    carriage = object_model.get_part("carriage")
    shoulder_block = object_model.get_part("shoulder_block")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    pad = object_model.get_part("pad")

    slide = object_model.get_articulation("rail_to_carriage")
    shoulder = object_model.get_articulation("carriage_to_shoulder")
    elbow = object_model.get_articulation("upper_arm_to_forearm")

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

    for part_name in (
        "base_rail",
        "carriage",
        "shoulder_block",
        "upper_arm",
        "forearm",
        "pad",
    ):
        ctx.check(f"has_part_{part_name}", object_model.get_part(part_name) is not None)

    ctx.check(
        "slide_joint_is_prismatic_x",
        slide.articulation_type == ArticulationType.PRISMATIC
        and all(isclose(a, b, abs_tol=1e-9) for a, b in zip(slide.axis, (1.0, 0.0, 0.0))),
        details=f"joint type={slide.articulation_type}, axis={slide.axis}",
    )
    ctx.check(
        "shoulder_joint_is_revolute_z",
        shoulder.articulation_type == ArticulationType.REVOLUTE
        and all(
            isclose(a, b, abs_tol=1e-9) for a, b in zip(shoulder.axis, (0.0, 0.0, 1.0))
        ),
        details=f"joint type={shoulder.articulation_type}, axis={shoulder.axis}",
    )
    ctx.check(
        "elbow_joint_is_revolute_z",
        elbow.articulation_type == ArticulationType.REVOLUTE
        and all(isclose(a, b, abs_tol=1e-9) for a, b in zip(elbow.axis, (0.0, 0.0, 1.0))),
        details=f"joint type={elbow.articulation_type}, axis={elbow.axis}",
    )

    ctx.expect_contact(carriage, base_rail, name="carriage_contacts_guide_rail")
    ctx.expect_contact(shoulder_block, carriage, name="shoulder_block_seats_on_carriage")
    ctx.expect_contact(upper_arm, shoulder_block, name="upper_arm_mounts_on_shoulder_block")
    ctx.expect_contact(forearm, upper_arm, name="forearm_mounts_on_upper_arm")
    ctx.expect_contact(pad, forearm, name="pad_mounts_on_forearm")

    carriage_aabb = ctx.part_world_aabb(carriage)
    shoulder_aabb = ctx.part_world_aabb(shoulder_block)
    if carriage_aabb is not None and shoulder_aabb is not None:
        carriage_len = carriage_aabb[1][0] - carriage_aabb[0][0]
        shoulder_span = shoulder_aabb[1][0] - shoulder_aabb[0][0]
        ctx.check(
            "carriage_reads_as_stage",
            carriage_len >= shoulder_span * 2.2,
            details=f"carriage_len={carriage_len:.3f}, shoulder_span={shoulder_span:.3f}",
        )

    lower = slide.motion_limits.lower
    upper = slide.motion_limits.upper
    if lower is not None and upper is not None:
        with ctx.pose({slide: lower}):
            lower_pos = ctx.part_world_position(carriage)
        with ctx.pose({slide: upper}):
            upper_pos = ctx.part_world_position(carriage)
        if lower_pos is not None and upper_pos is not None:
            travel = upper_pos[0] - lower_pos[0]
            ctx.check(
                "carriage_travel_matches_prismatic_limits",
                isclose(travel, upper - lower, abs_tol=1e-6),
                details=f"travel={travel:.6f}, expected={upper - lower:.6f}",
            )
            ctx.check(
                "carriage_prismatic_motion_stays_on_axis",
                isclose(lower_pos[1], upper_pos[1], abs_tol=1e-6)
                and isclose(lower_pos[2], upper_pos[2], abs_tol=1e-6),
                details=f"lower={lower_pos}, upper={upper_pos}",
            )

    with ctx.pose({slide: 0.18, shoulder: 0.95, elbow: -0.85}):
        ctx.fail_if_parts_overlap_in_current_pose(name="articulated_pose_clearance")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
