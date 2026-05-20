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


BACKPLATE_W = 0.14
BACKPLATE_H = 0.56
BACKPLATE_T = 0.012

GUIDE_BASE_W = 0.048
GUIDE_BASE_H = 0.40
GUIDE_BASE_D = 0.012
GUIDE_TONGUE_W = 0.024
GUIDE_TONGUE_H = 0.34
GUIDE_TONGUE_D = 0.008
GUIDE_TOTAL_D = GUIDE_BASE_D + GUIDE_TONGUE_D

CARRIAGE_W = 0.078
CARRIAGE_H = 0.072
CARRIAGE_BODY_D = 0.024
RUNNER_W = 0.022
RUNNER_H = 0.058
RUNNER_D = GUIDE_TONGUE_D
RUNNER_X = (GUIDE_TONGUE_W / 2.0) + (RUNNER_W / 2.0)
PIVOT_BOSS_R = 0.020
PIVOT_BOSS_D = 0.008
PIVOT_Y = CARRIAGE_BODY_D + PIVOT_BOSS_D

ARM_HUB_R = 0.020
ARM_HUB_D = 0.010
ARM_BAR_W = 0.028
ARM_BAR_T = 0.014
ARM_BAR_L = 0.195
ARM_NECK_W = 0.020
ARM_NECK_T = 0.014
ARM_NECK_L = 0.040
PAD_SIZE = 0.060
PAD_T = 0.012

SLIDE_TRAVEL = 0.26
SLIDE_LOWER_Z = -(SLIDE_TRAVEL / 2.0)
SWING_LIMIT = 1.2


def _box_centered_on_back_face(size: tuple[float, float, float], *, y0: float, z0: float = 0.0) -> Origin:
    sx, sy, sz = size
    return Origin(xyz=(0.0, y0 + (sy / 2.0), z0))


def _rotated_y_cylinder_origin(center_xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=center_xyz, rpy=(pi / 2.0, 0.0, 0.0))


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((lo + hi) / 2.0 for lo, hi in zip(mins, maxs))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_backed_slide_and_swing_fixture")

    model.material("plate_gray", rgba=(0.30, 0.32, 0.35, 1.0))
    model.material("rail_steel", rgba=(0.58, 0.61, 0.66, 1.0))
    model.material("carriage_dark", rgba=(0.17, 0.19, 0.22, 1.0))
    model.material("arm_silver", rgba=(0.77, 0.79, 0.82, 1.0))
    model.material("pad_rubber", rgba=(0.15, 0.15, 0.16, 1.0))
    model.material("fastener_steel", rgba=(0.70, 0.72, 0.75, 1.0))

    backplate = model.part("backplate")
    backplate.visual(
        Box((BACKPLATE_W, BACKPLATE_T, BACKPLATE_H)),
        origin=Origin(xyz=(0.0, BACKPLATE_T / 2.0, 0.0)),
        material="plate_gray",
        name="plate",
    )

    screw_head_radius = 0.008
    screw_head_thickness = 0.003
    screw_x = 0.042
    screw_z = 0.18
    for index, (x, z) in enumerate(
        (
            (-screw_x, -screw_z),
            (screw_x, -screw_z),
            (-screw_x, screw_z),
            (screw_x, screw_z),
        ),
        start=1,
    ):
        backplate.visual(
            Cylinder(radius=screw_head_radius, length=screw_head_thickness),
            origin=_rotated_y_cylinder_origin((x, BACKPLATE_T + (screw_head_thickness / 2.0), z)),
            material="fastener_steel",
            name=f"mount_head_{index}",
        )

    guide = model.part("guide")
    guide.visual(
        Box((GUIDE_BASE_W, GUIDE_BASE_D, GUIDE_BASE_H)),
        origin=Origin(xyz=(0.0, -(GUIDE_TONGUE_D + (GUIDE_BASE_D / 2.0)), 0.0)),
        material="rail_steel",
        name="rail_base",
    )
    guide.visual(
        Box((GUIDE_TONGUE_W, GUIDE_TONGUE_D, GUIDE_TONGUE_H)),
        origin=Origin(xyz=(0.0, -(GUIDE_TONGUE_D / 2.0), 0.0)),
        material="rail_steel",
        name="rail_tongue",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((CARRIAGE_W, CARRIAGE_BODY_D, CARRIAGE_H)),
        origin=Origin(xyz=(0.0, CARRIAGE_BODY_D / 2.0, 0.0)),
        material="carriage_dark",
        name="body",
    )
    carriage.visual(
        Box((RUNNER_W, RUNNER_D, RUNNER_H)),
        origin=Origin(xyz=(RUNNER_X, -(RUNNER_D / 2.0), 0.0)),
        material="carriage_dark",
        name="runner_right",
    )
    carriage.visual(
        Box((RUNNER_W, RUNNER_D, RUNNER_H)),
        origin=Origin(xyz=(-RUNNER_X, -(RUNNER_D / 2.0), 0.0)),
        material="carriage_dark",
        name="runner_left",
    )
    carriage.visual(
        Cylinder(radius=PIVOT_BOSS_R, length=PIVOT_BOSS_D),
        origin=_rotated_y_cylinder_origin((0.0, CARRIAGE_BODY_D + (PIVOT_BOSS_D / 2.0), 0.0)),
        material="carriage_dark",
        name="pivot_boss",
    )

    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=ARM_HUB_R, length=ARM_HUB_D),
        origin=_rotated_y_cylinder_origin((0.0, ARM_HUB_D / 2.0, 0.0)),
        material="arm_silver",
        name="hub",
    )
    arm.visual(
        Box((ARM_BAR_W, ARM_BAR_L, ARM_BAR_T)),
        origin=Origin(xyz=(0.0, ARM_HUB_D + (ARM_BAR_L / 2.0), 0.0)),
        material="arm_silver",
        name="bar",
    )
    arm.visual(
        Box((ARM_NECK_W, ARM_NECK_L, ARM_NECK_T)),
        origin=Origin(
            xyz=(0.0, ARM_HUB_D + ARM_BAR_L + (ARM_NECK_L / 2.0), 0.0)
        ),
        material="arm_silver",
        name="neck",
    )
    arm.visual(
        Box((PAD_SIZE, PAD_T, PAD_SIZE)),
        origin=Origin(
            xyz=(
                0.0,
                ARM_HUB_D + ARM_BAR_L + ARM_NECK_L + (PAD_T / 2.0),
                0.0,
            )
        ),
        material="pad_rubber",
        name="end_pad",
    )

    model.articulation(
        "backplate_to_guide",
        ArticulationType.FIXED,
        parent=backplate,
        child=guide,
        origin=Origin(xyz=(0.0, BACKPLATE_T + GUIDE_TOTAL_D, 0.0)),
    )
    model.articulation(
        "guide_to_carriage",
        ArticulationType.PRISMATIC,
        parent=guide,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, SLIDE_LOWER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.18,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )
    model.articulation(
        "carriage_to_arm",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=arm,
        origin=Origin(xyz=(0.0, PIVOT_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.5,
            lower=-SWING_LIMIT,
            upper=SWING_LIMIT,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    backplate = object_model.get_part("backplate")
    guide = object_model.get_part("guide")
    carriage = object_model.get_part("carriage")
    arm = object_model.get_part("arm")

    guide_slide = object_model.get_articulation("guide_to_carriage")
    arm_swing = object_model.get_articulation("carriage_to_arm")

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
        "fixture_parts_present",
        all(part is not None for part in (backplate, guide, carriage, arm)),
        "Expected backplate, guide, carriage, and arm parts.",
    )
    ctx.check(
        "guide_stage_is_vertical_prismatic",
        guide_slide.articulation_type == ArticulationType.PRISMATIC and guide_slide.axis == (0.0, 0.0, 1.0),
        f"Expected a vertical prismatic slide, got type={guide_slide.articulation_type} axis={guide_slide.axis}.",
    )
    ctx.check(
        "arm_joint_is_vertical_revolute",
        arm_swing.articulation_type == ArticulationType.REVOLUTE and arm_swing.axis == (0.0, 0.0, 1.0),
        f"Expected a vertical revolute arm joint, got type={arm_swing.articulation_type} axis={arm_swing.axis}.",
    )

    ctx.expect_contact(guide, backplate, name="guide_is_mounted_to_backplate")
    ctx.expect_contact(carriage, guide, name="carriage_is_supported_by_guide")
    ctx.expect_contact(arm, carriage, name="arm_hub_contacts_carriage_boss")

    ctx.expect_gap(
        arm,
        backplate,
        axis="y",
        positive_elem="end_pad",
        min_gap=0.22,
        name="end_pad_projects_forward_from_wall_plate",
    )
    ctx.expect_overlap(
        carriage,
        guide,
        axes="x",
        min_overlap=GUIDE_TONGUE_W,
        name="carriage_tracks_rail_centerline",
    )

    with ctx.pose({guide_slide: guide_slide.motion_limits.lower}):
        low_carriage_z = ctx.part_world_position(carriage)[2]
        closed_pad_center = _aabb_center(ctx.part_element_world_aabb(arm, elem="end_pad"))

    with ctx.pose({guide_slide: guide_slide.motion_limits.upper}):
        high_carriage_z = ctx.part_world_position(carriage)[2]

    ctx.check(
        "positive_slide_moves_carriage_upward",
        high_carriage_z > low_carriage_z + 0.20,
        f"Expected the carriage to rise significantly, got low_z={low_carriage_z:.4f} high_z={high_carriage_z:.4f}.",
    )

    with ctx.pose({arm_swing: arm_swing.motion_limits.upper}):
        swung_pad_center = _aabb_center(ctx.part_element_world_aabb(arm, elem="end_pad"))

    ctx.check(
        "arm_swings_end_pad_sideways",
        closed_pad_center is not None
        and swung_pad_center is not None
        and abs(swung_pad_center[0] - closed_pad_center[0]) > 0.12
        and swung_pad_center[1] > 0.10,
        (
            "Expected the end pad to sweep laterally while staying forward of the wall. "
            f"closed={closed_pad_center}, swung={swung_pad_center}."
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
