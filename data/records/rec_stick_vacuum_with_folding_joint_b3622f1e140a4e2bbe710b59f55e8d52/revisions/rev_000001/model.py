from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_folding_stick_vacuum")

    graphite = model.material("graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    silver = model.material("silver", rgba=(0.72, 0.74, 0.77, 1.0))
    smoke_clear = model.material("smoke_clear", rgba=(0.60, 0.68, 0.74, 0.55))
    soft_black = model.material("soft_black", rgba=(0.10, 0.10, 0.11, 1.0))
    accent_red = model.material("accent_red", rgba=(0.70, 0.15, 0.14, 1.0))

    wand = model.part("wand")
    wand.visual(
        Cylinder(radius=0.016, length=0.50),
        origin=Origin(xyz=(0.0, 0.0, -0.27)),
        material=silver,
        name="main_tube",
    )
    wand.visual(
        Box((0.040, 0.042, 0.032)),
        origin=Origin(xyz=(0.010, 0.0, -0.031)),
        material=graphite,
        name="upper_joint_block",
    )
    wand.visual(
        Box((0.060, 0.005, 0.050)),
        origin=Origin(xyz=(0.030, 0.0225, 0.0)),
        material=graphite,
        name="upper_left_cheek",
    )
    wand.visual(
        Box((0.060, 0.005, 0.050)),
        origin=Origin(xyz=(0.030, -0.0225, 0.0)),
        material=graphite,
        name="upper_right_cheek",
    )
    wand.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(xyz=(0.060, 0.028, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=soft_black,
        name="upper_left_pin_cap",
    )
    wand.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(xyz=(0.060, -0.028, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=soft_black,
        name="upper_right_pin_cap",
    )
    wand.visual(
        Box((0.056, 0.042, 0.045)),
        origin=Origin(xyz=(0.009, 0.0, -0.542)),
        material=graphite,
        name="lower_joint_block",
    )
    wand.visual(
        Box((0.022, 0.005, 0.055)),
        origin=Origin(xyz=(0.018, 0.0225, -0.580)),
        material=graphite,
        name="lower_left_cheek",
    )
    wand.visual(
        Box((0.022, 0.005, 0.055)),
        origin=Origin(xyz=(0.018, -0.0225, -0.580)),
        material=graphite,
        name="lower_right_cheek",
    )
    wand.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(0.018, 0.028, -0.580), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=soft_black,
        name="lower_left_pin_cap",
    )
    wand.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(0.018, -0.028, -0.580), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=soft_black,
        name="lower_right_pin_cap",
    )
    wand.inertial = Inertial.from_geometry(
        Box((0.09, 0.08, 0.64)),
        mass=0.75,
        origin=Origin(xyz=(0.012, 0.0, -0.31)),
    )

    body = model.part("body")
    body.visual(
        Cylinder(radius=0.012, length=0.040),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="hinge_barrel",
    )
    body.visual(
        Box((0.090, 0.004, 0.024)),
        origin=Origin(xyz=(0.045, 0.019, 0.012)),
        material=graphite,
        name="hinge_left_yoke",
    )
    body.visual(
        Box((0.090, 0.004, 0.024)),
        origin=Origin(xyz=(0.045, -0.019, 0.012)),
        material=graphite,
        name="hinge_right_yoke",
    )
    body.visual(
        Box((0.034, 0.040, 0.032)),
        origin=Origin(xyz=(0.090, 0.0, 0.020)),
        material=graphite,
        name="hinge_bridge",
    )
    body.visual(
        Box((0.052, 0.040, 0.038)),
        origin=Origin(xyz=(0.122, 0.0, 0.040)),
        material=graphite,
        name="hinge_neck",
    )
    body.visual(
        Box((0.108, 0.070, 0.138)),
        origin=Origin(xyz=(0.168, 0.0, 0.112)),
        material=graphite,
        name="motor_housing",
    )
    body.visual(
        Box((0.070, 0.060, 0.082)),
        origin=Origin(xyz=(0.144, 0.0, 0.050)),
        material=soft_black,
        name="battery_pack",
    )
    body.visual(
        Cylinder(radius=0.035, length=0.160),
        origin=Origin(xyz=(0.236, 0.0, 0.126), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=smoke_clear,
        name="dust_cup",
    )
    body.visual(
        Box((0.055, 0.050, 0.060)),
        origin=Origin(xyz=(0.312, 0.0, 0.126)),
        material=graphite,
        name="cyclone_nose",
    )
    body.visual(
        Box((0.022, 0.042, 0.132)),
        origin=Origin(xyz=(0.122, 0.0, 0.140)),
        material=graphite,
        name="rear_grip_post",
    )
    body.visual(
        Box((0.075, 0.042, 0.022)),
        origin=Origin(xyz=(0.156, 0.0, 0.205)),
        material=graphite,
        name="top_handle_bridge",
    )
    body.visual(
        Box((0.045, 0.035, 0.020)),
        origin=Origin(xyz=(0.142, 0.0, 0.106)),
        material=graphite,
        name="lower_handle_bridge",
    )
    body.visual(
        Box((0.020, 0.045, 0.030)),
        origin=Origin(xyz=(0.158, 0.0, 0.126)),
        material=accent_red,
        name="trigger_paddle",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.31, 0.08, 0.33)),
        mass=2.05,
        origin=Origin(xyz=(0.180, 0.0, 0.115)),
    )

    floor_head = model.part("floor_head")
    floor_head.visual(
        Cylinder(radius=0.010, length=0.040),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="neck_barrel",
    )
    floor_head.visual(
        Box((0.034, 0.030, 0.030)),
        origin=Origin(xyz=(0.014, 0.0, -0.013)),
        material=graphite,
        name="neck_block",
    )
    floor_head.visual(
        Box((0.220, 0.090, 0.026)),
        origin=Origin(xyz=(0.128, 0.0, -0.031)),
        material=soft_black,
        name="head_shell",
    )
    floor_head.visual(
        Box((0.032, 0.090, 0.012)),
        origin=Origin(xyz=(0.234, 0.0, -0.040)),
        material=graphite,
        name="front_lip",
    )
    floor_head.visual(
        Box((0.100, 0.050, 0.012)),
        origin=Origin(xyz=(0.120, 0.0, -0.043)),
        material=graphite,
        name="suction_channel",
    )
    floor_head.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(0.050, 0.046, -0.047), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="left_wheel",
    )
    floor_head.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(0.050, -0.046, -0.047), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="right_wheel",
    )
    floor_head.inertial = Inertial.from_geometry(
        Box((0.26, 0.10, 0.08)),
        mass=0.85,
        origin=Origin(xyz=(0.13, 0.0, -0.04)),
    )

    model.articulation(
        "body_fold_joint",
        ArticulationType.REVOLUTE,
        parent=wand,
        child=body,
        origin=Origin(xyz=(0.060, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=3.0,
            lower=0.0,
            upper=2.75,
        ),
    )
    model.articulation(
        "nozzle_pitch_joint",
        ArticulationType.REVOLUTE,
        parent=wand,
        child=floor_head,
        origin=Origin(xyz=(0.018, 0.0, -0.580)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=3.5,
            lower=-0.55,
            upper=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    wand = object_model.get_part("wand")
    floor_head = object_model.get_part("floor_head")
    fold_joint = object_model.get_articulation("body_fold_joint")
    nozzle_joint = object_model.get_articulation("nozzle_pitch_joint")

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
    ctx.warn_if_articulation_overlaps(max_pose_samples=24)

    ctx.check(
        "fold_joint_has_flat_stow_range",
        fold_joint.motion_limits is not None
        and fold_joint.motion_limits.lower == 0.0
        and fold_joint.motion_limits.upper is not None
        and fold_joint.motion_limits.upper >= 2.65,
        "Body fold joint should reach a near-flat stow angle.",
    )
    ctx.check(
        "nozzle_joint_has_bidirectional_pitch",
        nozzle_joint.motion_limits is not None
        and nozzle_joint.motion_limits.lower is not None
        and nozzle_joint.motion_limits.upper is not None
        and nozzle_joint.motion_limits.lower < 0.0 < nozzle_joint.motion_limits.upper,
        "Floor head should pitch both slightly down and up around the neck pin.",
    )

    with ctx.pose({fold_joint: 0.0, nozzle_joint: 0.0}):
        ctx.expect_contact(
            body,
            wand,
            elem_a="hinge_barrel",
            elem_b="upper_left_cheek",
            name="fold_barrel_contacts_left_cheek",
        )
        ctx.expect_contact(
            body,
            wand,
            elem_a="hinge_barrel",
            elem_b="upper_right_cheek",
            name="fold_barrel_contacts_right_cheek",
        )
        ctx.expect_contact(
            floor_head,
            wand,
            elem_a="neck_barrel",
            elem_b="lower_left_cheek",
            name="nozzle_barrel_contacts_left_cheek",
        )
        ctx.expect_contact(
            floor_head,
            wand,
            elem_a="neck_barrel",
            elem_b="lower_right_cheek",
            name="nozzle_barrel_contacts_right_cheek",
        )
        ctx.expect_gap(
            body,
            floor_head,
            axis="z",
            min_gap=0.48,
            name="body_stays_well_above_floor_head_in_use",
        )

    def _assembly_extents():
        aabbs = [ctx.part_world_aabb(part) for part in (body, wand, floor_head)]
        if any(aabb is None for aabb in aabbs):
            return None
        min_corner = [min(aabb[0][i] for aabb in aabbs) for i in range(3)]
        max_corner = [max(aabb[1][i] for aabb in aabbs) for i in range(3)]
        return [max_corner[i] - min_corner[i] for i in range(3)]

    with ctx.pose({fold_joint: 0.0, nozzle_joint: 0.0}):
        deployed_extents = _assembly_extents()
    with ctx.pose({fold_joint: 2.75, nozzle_joint: 0.55}):
        stowed_extents = _assembly_extents()
        dust_cup_aabb = ctx.part_element_world_aabb(body, elem="dust_cup")
        head_shell_aabb = ctx.part_element_world_aabb(floor_head, elem="head_shell")
        stowed_front_gap = None
        if dust_cup_aabb is not None and head_shell_aabb is not None:
            stowed_front_gap = head_shell_aabb[0][0] - dust_cup_aabb[1][0]

    ctx.check(
        "compact_upright_height",
        deployed_extents is not None and 0.80 <= deployed_extents[2] <= 0.98,
        f"Expected a compact upright stick-vac envelope, got {deployed_extents}.",
    )
    ctx.check(
        "folding_reduces_storage_height",
        deployed_extents is not None
        and stowed_extents is not None
        and stowed_extents[2] <= deployed_extents[2] - 0.18,
        f"Expected folded height reduction, got deployed={deployed_extents}, stowed={stowed_extents}.",
    )
    ctx.check(
        "stowed_body_clears_head_shell",
        stowed_front_gap is not None and stowed_front_gap >= 0.01,
        f"Expected body and floor head to remain separated in stow pose, got front gap {stowed_front_gap}.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
