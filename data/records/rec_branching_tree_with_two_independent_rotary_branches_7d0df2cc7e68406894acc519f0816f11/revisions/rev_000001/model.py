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


BASE_X = 0.220
BASE_Y = 0.150
BASE_T = 0.018
POST_X = 0.038
POST_Y = 0.030
POST_H = 0.335

LOWER_AXIS_X = 0.046
LOWER_AXIS_Z = 0.118
LOWER_COLLAR_W = 0.030
LOWER_COLLAR_R = 0.018
LOWER_SHAFT_R = 0.0076
LOWER_HUB_W = 0.020
LOWER_ARM_LEN = 0.248
LOWER_LIMITS = (-0.55, 0.22)

UPPER_AXIS_X = 0.036
UPPER_AXIS_Y = 0.052
UPPER_AXIS_Z = 0.276
UPPER_COLLAR_W = 0.026
UPPER_COLLAR_R = 0.016
UPPER_SHAFT_R = 0.0070
UPPER_HUB_W = 0.018
UPPER_ARM_LEN = 0.178
UPPER_LIMITS = (-0.42, 0.52)

LOWER_ARM_Y = -0.066
LOWER_HUB_CENTER_Y = -0.025
UPPER_ARM_Y = 0.086
UPPER_HUB_CENTER_Y = 0.014


def _ring(width: float, outer_r: float, inner_r: float, flange_r: float, flange_t: float) -> cq.Workplane:
    core = cq.Workplane("XZ").circle(outer_r).circle(inner_r).extrude(width / 2.0, both=True)
    flange = cq.Workplane("XZ").circle(flange_r).circle(inner_r).extrude(flange_t)
    return (
        core.union(flange.translate((0.0, width / 2.0 - flange_t, 0.0)))
        .union(flange.translate((0.0, -width / 2.0, 0.0)))
    )


def _make_backbone_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(BASE_X, BASE_Y, BASE_T, centered=(True, True, False))
    post = (
        cq.Workplane("XY")
        .box(POST_X, POST_Y, POST_H, centered=(True, True, False))
        .translate((0.0, 0.0, BASE_T))
    )

    lower_cheek = (
        cq.Workplane("XY")
        .box(0.030, 0.008, 0.090, centered=(True, True, True))
        .translate((0.035, 0.018, LOWER_AXIS_Z))
    )
    lower_cheek_mirror = lower_cheek.translate((0.0, -0.036, 0.0))
    lower_mount = (
        cq.Workplane("XY")
        .box(0.016, 0.008, 0.062, centered=(True, True, True))
        .translate((0.014, 0.018, LOWER_AXIS_Z - 0.004))
    )
    lower_mount_mirror = lower_mount.translate((0.0, -0.036, 0.0))
    lower_shaft = (
        cq.Workplane("XZ")
        .center(LOWER_AXIS_X, LOWER_AXIS_Z)
        .circle(LOWER_SHAFT_R)
        .extrude(0.018, both=True)
    )
    lower_boss = (
        cq.Workplane("XZ")
        .center(LOWER_AXIS_X, LOWER_AXIS_Z)
        .circle(0.011)
        .extrude(0.004)
        .translate((0.0, 0.022, 0.0))
    )
    lower_boss_mirror = lower_boss.translate((0.0, -0.048, 0.0))
    lower_rib = (
        cq.Workplane("XZ")
        .moveTo(0.004, BASE_T + 0.008)
        .lineTo(0.004, LOWER_AXIS_Z + 0.030)
        .lineTo(0.022, LOWER_AXIS_Z - 0.004)
        .lineTo(0.018, BASE_T + 0.008)
        .close()
        .extrude(0.008)
    )
    lower_rib_neg = lower_rib.translate((0.0, -0.026, 0.0))
    lower_rib_pos = lower_rib.translate((0.0, 0.018, 0.0))

    base_gusset = (
        cq.Workplane("XZ")
        .moveTo(0.000, BASE_T)
        .lineTo(0.000, BASE_T + 0.138)
        .lineTo(0.045, BASE_T)
        .close()
        .extrude(0.010)
    )
    base_gusset_pos = base_gusset.translate((0.0, 0.018, 0.0))
    base_gusset_neg = base_gusset.translate((0.0, -0.028, 0.0))

    upper_side_plate = (
        cq.Workplane("XY")
        .box(0.014, 0.010, 0.120, centered=(True, True, True))
        .translate((0.000, POST_Y / 2.0 + 0.005, UPPER_AXIS_Z - 0.010))
    )
    upper_inner_cheek = (
        cq.Workplane("XY")
        .box(0.026, 0.008, 0.072, centered=(True, True, True))
        .translate((0.030, 0.043, UPPER_AXIS_Z))
    )
    upper_outer_cheek = (
        cq.Workplane("XY")
        .box(0.026, 0.008, 0.072, centered=(True, True, True))
        .translate((0.030, 0.061, UPPER_AXIS_Z))
    )
    upper_inner_mount = (
        cq.Workplane("XY")
        .box(0.016, 0.008, 0.056, centered=(True, True, True))
        .translate((0.012, 0.043, UPPER_AXIS_Z - 0.004))
    )
    upper_outer_mount = upper_inner_mount.translate((0.0, 0.018, 0.0))
    upper_shaft = (
        cq.Workplane("XZ")
        .center(UPPER_AXIS_X, UPPER_AXIS_Z)
        .circle(UPPER_SHAFT_R)
        .extrude(0.017, both=True)
        .translate((0.0, UPPER_AXIS_Y, 0.0))
    )
    upper_boss = (
        cq.Workplane("XZ")
        .center(UPPER_AXIS_X, UPPER_AXIS_Z)
        .circle(0.0105)
        .extrude(0.004)
        .translate((0.0, 0.065, 0.0))
    )
    upper_boss_mirror = upper_boss.translate((0.0, -0.026, 0.0))
    upper_rib = (
        cq.Workplane("XZ")
        .moveTo(0.000, UPPER_AXIS_Z - 0.050)
        .lineTo(0.000, UPPER_AXIS_Z + 0.022)
        .lineTo(0.018, UPPER_AXIS_Z - 0.006)
        .lineTo(0.014, UPPER_AXIS_Z - 0.050)
        .close()
        .extrude(0.006)
        .translate((0.0, 0.040, 0.0))
    )
    upper_rib_outer = upper_rib.translate((0.0, 0.018, 0.0))

    return (
        base.union(post)
        .union(base_gusset_pos)
        .union(base_gusset_neg)
        .union(lower_cheek)
        .union(lower_cheek_mirror)
        .union(lower_mount)
        .union(lower_mount_mirror)
        .union(lower_shaft)
        .union(lower_boss)
        .union(lower_boss_mirror)
        .union(lower_rib_pos)
        .union(lower_rib_neg)
        .union(upper_side_plate)
        .union(upper_inner_cheek)
        .union(upper_outer_cheek)
        .union(upper_inner_mount)
        .union(upper_outer_mount)
        .union(upper_shaft)
        .union(upper_boss)
        .union(upper_boss_mirror)
        .union(upper_rib)
        .union(upper_rib_outer)
    )


def _make_lower_arm_shape() -> cq.Workplane:
    hub = _make_lower_hub_shape()
    body = _make_lower_body_shape()
    return hub.union(body)


def _make_lower_hub_shape() -> cq.Workplane:
    sleeve = _ring(
        width=0.006,
        outer_r=0.011,
        inner_r=LOWER_SHAFT_R + 0.0006,
        flange_r=0.0135,
        flange_t=0.002,
    ).translate((0.0, LOWER_HUB_CENTER_Y, 0.0))
    outboard_pad = (
        cq.Workplane("XY")
        .box(0.024, 0.020, 0.020, centered=(False, True, True))
        .translate((0.004, -0.045, 0.0))
    )
    return sleeve.union(outboard_pad)


def _make_lower_body_shape() -> cq.Workplane:
    shoulder_block = (
        cq.Workplane("XY")
        .box(0.026, 0.014, 0.022, centered=(False, True, True))
        .translate((0.028, -0.058, 0.000))
    )
    beam = (
        cq.Workplane("XY")
        .box(0.184, 0.020, 0.018, centered=(False, True, True))
        .translate((0.046, LOWER_ARM_Y, -0.003))
    )
    reinforcement = (
        cq.Workplane("XZ")
        .moveTo(0.050, -0.010)
        .lineTo(0.126, -0.010)
        .lineTo(0.102, -0.026)
        .lineTo(0.056, -0.026)
        .close()
        .extrude(0.007, both=True)
        .translate((0.0, -0.060, 0.0))
    )
    tip_pad = (
        cq.Workplane("XY")
        .box(0.048, 0.036, 0.014, centered=(False, True, True))
        .translate((0.200, LOWER_ARM_Y, 0.001))
    )
    toe = (
        cq.Workplane("XZ")
        .center(0.248, -0.004)
        .circle(0.016)
        .extrude(0.010, both=True)
        .translate((0.0, LOWER_ARM_Y, 0.0))
    )
    locator = (
        cq.Workplane("XY")
        .center(0.234, 0.0)
        .circle(0.008)
        .extrude(0.012)
        .translate((0.0, LOWER_ARM_Y, 0.011))
    )
    return shoulder_block.union(beam).union(reinforcement).union(tip_pad).union(toe).union(locator)


def _make_upper_arm_shape() -> cq.Workplane:
    hub = _make_upper_hub_shape()
    body = _make_upper_body_shape()
    return hub.union(body)


def _make_upper_hub_shape() -> cq.Workplane:
    sleeve = _ring(
        width=0.006,
        outer_r=0.010,
        inner_r=UPPER_SHAFT_R + 0.0006,
        flange_r=0.0125,
        flange_t=0.002,
    ).translate((0.0, UPPER_HUB_CENTER_Y, 0.0))
    outboard_pad = (
        cq.Workplane("XY")
        .box(0.020, 0.018, 0.018, centered=(False, True, True))
        .translate((0.004, 0.073, 0.002))
    )
    return sleeve.union(outboard_pad)


def _make_upper_body_shape() -> cq.Workplane:
    shoulder_block = (
        cq.Workplane("XY")
        .box(0.020, 0.012, 0.020, centered=(False, True, True))
        .translate((0.024, 0.079, 0.002))
    )
    neck = (
        cq.Workplane("XY")
        .box(0.034, 0.016, 0.016, centered=(False, True, True))
        .translate((0.034, 0.083, 0.003))
    )
    beam = (
        cq.Workplane("XY")
        .box(0.120, 0.016, 0.014, centered=(False, True, True))
        .translate((0.048, UPPER_ARM_Y, 0.006))
    )
    web = (
        cq.Workplane("XZ")
        .moveTo(0.040, -0.004)
        .lineTo(0.092, -0.005)
        .lineTo(0.072, -0.017)
        .lineTo(0.046, -0.017)
        .close()
        .extrude(0.006, both=True)
        .translate((0.0, 0.084, 0.0))
    )
    fork_block = (
        cq.Workplane("XY")
        .box(0.042, 0.018, 0.040, centered=(False, True, True))
        .translate((0.142, UPPER_ARM_Y, 0.002))
    )
    fork_slot = (
        cq.Workplane("XY")
        .box(0.030, 0.010, 0.018, centered=(False, True, True))
        .translate((0.154, UPPER_ARM_Y, 0.002))
    )
    return shoulder_block.union(neck).union(beam).union(web).union(fork_block.cut(fork_slot))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_fixture_tree")

    model.material("painted_steel", rgba=(0.24, 0.26, 0.29, 1.0))
    model.material("machine_gray", rgba=(0.58, 0.60, 0.63, 1.0))
    model.material("black_oxide", rgba=(0.16, 0.17, 0.18, 1.0))

    backbone = model.part("backbone")
    backbone.visual(
        mesh_from_cadquery(_make_backbone_shape(), "fixture_backbone"),
        material="painted_steel",
        name="backbone_shell",
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        mesh_from_cadquery(_make_lower_hub_shape(), "fixture_lower_hub"),
        material="machine_gray",
        name="lower_hub",
    )
    lower_arm.visual(
        mesh_from_cadquery(_make_lower_body_shape(), "fixture_lower_body"),
        material="machine_gray",
        name="lower_arm_shell",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        mesh_from_cadquery(_make_upper_hub_shape(), "fixture_upper_hub"),
        material="black_oxide",
        name="upper_hub",
    )
    upper_arm.visual(
        mesh_from_cadquery(_make_upper_body_shape(), "fixture_upper_body"),
        material="black_oxide",
        name="upper_arm_shell",
    )

    model.articulation(
        "lower_pivot",
        ArticulationType.REVOLUTE,
        parent=backbone,
        child=lower_arm,
        origin=Origin(xyz=(LOWER_AXIS_X, 0.0, LOWER_AXIS_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=LOWER_LIMITS[0],
            upper=LOWER_LIMITS[1],
            effort=42.0,
            velocity=1.2,
        ),
    )
    model.articulation(
        "upper_pivot",
        ArticulationType.REVOLUTE,
        parent=backbone,
        child=upper_arm,
        origin=Origin(xyz=(UPPER_AXIS_X, UPPER_AXIS_Y, UPPER_AXIS_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=UPPER_LIMITS[0],
            upper=UPPER_LIMITS[1],
            effort=28.0,
            velocity=1.4,
        ),
    )

    return model


def _extent(aabb, axis_index: int) -> float:
    return float(aabb[1][axis_index] - aabb[0][axis_index])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    backbone = object_model.get_part("backbone")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    lower_pivot = object_model.get_articulation("lower_pivot")
    upper_pivot = object_model.get_articulation("upper_pivot")

    ctx.allow_overlap(
        backbone,
        lower_arm,
        elem_a="backbone_shell",
        elem_b="lower_hub",
        reason="lower arm hub sleeve intentionally wraps the supported pivot shaft and collar",
    )
    ctx.allow_overlap(
        backbone,
        upper_arm,
        elem_a="backbone_shell",
        elem_b="upper_hub",
        reason="upper arm hub sleeve intentionally wraps the side-mounted pivot shaft and collar",
    )

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
        "lower_joint_axis",
        tuple(round(value, 3) for value in lower_pivot.axis) == (0.0, 1.0, 0.0),
        details=f"unexpected lower joint axis: {lower_pivot.axis}",
    )
    ctx.check(
        "upper_joint_axis",
        tuple(round(value, 3) for value in upper_pivot.axis) == (0.0, 1.0, 0.0),
        details=f"unexpected upper joint axis: {upper_pivot.axis}",
    )
    ctx.expect_contact(
        lower_arm,
        backbone,
        contact_tol=0.0008,
        name="lower_arm_supported_by_lower_hub",
    )
    ctx.expect_contact(
        upper_arm,
        backbone,
        contact_tol=0.0008,
        name="upper_arm_supported_by_side_hub",
    )
    ctx.expect_origin_gap(
        positive_link=upper_arm,
        negative_link=lower_arm,
        axis="y",
        min_gap=0.045,
        max_gap=0.058,
        name="upper_arm_is_side_offset",
    )
    ctx.expect_origin_gap(
        positive_link=upper_arm,
        negative_link=lower_arm,
        axis="z",
        min_gap=0.145,
        max_gap=0.175,
        name="upper_arm_is_higher_than_lower_arm",
    )

    lower_aabb = ctx.part_world_aabb(lower_arm)
    upper_aabb = ctx.part_world_aabb(upper_arm)
    lower_dx = _extent(lower_aabb, 0) if lower_aabb is not None else 0.0
    upper_dx = _extent(upper_aabb, 0) if upper_aabb is not None else 0.0
    ctx.check(
        "arms_are_asymmetric_in_length",
        lower_dx > upper_dx + 0.055,
        details=f"lower dx={lower_dx:.4f}, upper dx={upper_dx:.4f}",
    )

    with ctx.pose({lower_pivot: LOWER_LIMITS[1], upper_pivot: UPPER_LIMITS[0]}):
        ctx.fail_if_parts_overlap_in_current_pose(name="clearance_at_crossed_extremes")
        ctx.expect_gap(
            positive_link=upper_arm,
            negative_link=lower_arm,
            axis="y",
            min_gap=0.012,
            name="arms_stay_laterally_separate_when_crossed",
        )

    with ctx.pose({lower_pivot: LOWER_LIMITS[0], upper_pivot: UPPER_LIMITS[1]}):
        ctx.fail_if_parts_overlap_in_current_pose(name="clearance_at_opposed_extremes")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
