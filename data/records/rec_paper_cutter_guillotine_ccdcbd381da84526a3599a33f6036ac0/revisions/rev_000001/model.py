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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


BED_SIZE = (0.48, 0.36, 0.016)
BED_TOP_Z = BED_SIZE[2]
PIVOT_ORIGIN = (-0.232, 0.172, 0.026)
ARM_CLOSED_YAW = -0.65
STOP_HOME = (0.09, 0.138, BED_TOP_Z)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_paper_cutter")

    model.material("bed_paint", color=(0.30, 0.33, 0.36))
    model.material("steel", color=(0.67, 0.69, 0.72))
    model.material("dark_frame", color=(0.16, 0.16, 0.17))
    model.material("blade_steel", color=(0.82, 0.84, 0.86))
    model.material("handle_grip", color=(0.08, 0.08, 0.08))
    model.material("stop_accent", color=(0.76, 0.16, 0.13))

    bed_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(BED_SIZE[0], BED_SIZE[1], 0.018, corner_segments=10),
            BED_SIZE[2],
        ),
        "paper_cutter_bed",
    )
    pivot_gusset_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            [
                (-0.032, -0.028),
                (0.020, -0.028),
                (0.032, -0.010),
                (0.032, 0.028),
                (-0.018, 0.028),
                (-0.032, 0.010),
            ],
            0.008,
        ),
        "paper_cutter_pivot_gusset",
    )
    arm_backbone_mesh = mesh_from_geometry(
        ExtrudeGeometry.centered(
            [
                (0.000, -0.016),
                (0.050, -0.023),
                (0.220, -0.020),
                (0.420, -0.016),
                (0.540, -0.012),
                (0.580, -0.009),
                (0.580, 0.009),
                (0.540, 0.012),
                (0.420, 0.016),
                (0.220, 0.020),
                (0.050, 0.023),
                (0.000, 0.016),
            ],
            0.008,
        ),
        "paper_cutter_arm_backbone",
    )

    base = model.part("base")
    base.visual(
        bed_mesh,
        material="bed_paint",
        name="bed",
    )
    base.visual(
        Box((0.42, 0.018, 0.024)),
        origin=Origin(xyz=(0.01, 0.171, 0.028)),
        material="dark_frame",
        name="fence",
    )
    base.visual(
        Cylinder(radius=0.008, length=0.36),
        origin=Origin(xyz=(0.03, 0.138, 0.024), rpy=(0.0, pi / 2.0, 0.0)),
        material="steel",
        name="guide_rail",
    )
    base.visual(
        pivot_gusset_mesh,
        origin=Origin(xyz=(-0.2075, 0.1475, 0.016)),
        material="dark_frame",
        name="pivot_block",
    )
    base.visual(
        Cylinder(radius=0.022, length=0.010),
        origin=Origin(xyz=(-0.232, 0.172, 0.021)),
        material="steel",
        name="pivot_post",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.54, 0.40, 0.05)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    blade_arm = model.part("blade_arm")
    blade_arm.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material="dark_frame",
        name="hub",
    )
    blade_arm.visual(
        arm_backbone_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material="dark_frame",
        name="backbone",
    )
    blade_arm.visual(
        Box((0.48, 0.004, 0.004)),
        origin=Origin(xyz=(0.24, -0.016, 0.012)),
        material="blade_steel",
        name="blade_bar",
    )
    blade_arm.visual(
        Cylinder(radius=0.012, length=0.14),
        origin=Origin(xyz=(0.50, 0.0, 0.032), rpy=(0.0, pi / 2.0, 0.0)),
        material="handle_grip",
        name="grip",
    )
    blade_arm.inertial = Inertial.from_geometry(
        Box((0.60, 0.05, 0.06)),
        mass=1.6,
        origin=Origin(xyz=(0.30, 0.0, 0.03)),
    )

    paper_stop = model.part("paper_stop")
    paper_stop.visual(
        Box((0.032, 0.006, 0.034)),
        origin=Origin(xyz=(0.0, -0.012, 0.017)),
        material="stop_accent",
        name="front_cheek",
    )
    paper_stop.visual(
        Box((0.032, 0.006, 0.034)),
        origin=Origin(xyz=(0.0, 0.012, 0.017)),
        material="stop_accent",
        name="rear_cheek",
    )
    paper_stop.visual(
        Box((0.032, 0.030, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material="stop_accent",
        name="bridge",
    )
    paper_stop.visual(
        Box((0.006, 0.030, 0.024)),
        origin=Origin(xyz=(-0.019, -0.021, 0.012)),
        material="stop_accent",
        name="stop_plate",
    )
    paper_stop.visual(
        Cylinder(radius=0.008, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material="dark_frame",
        name="lock_knob",
    )
    paper_stop.inertial = Inertial.from_geometry(
        Box((0.05, 0.05, 0.05)),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    model.articulation(
        "arm_swing",
        ArticulationType.REVOLUTE,
        parent=base,
        child=blade_arm,
        origin=Origin(xyz=PIVOT_ORIGIN, rpy=(0.0, 0.0, ARM_CLOSED_YAW)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=2.0,
            lower=0.0,
            upper=1.75,
        ),
    )
    model.articulation(
        "paper_stop_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=paper_stop,
        origin=Origin(xyz=STOP_HOME),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.25,
            lower=-0.16,
            upper=0.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    blade_arm = object_model.get_part("blade_arm")
    paper_stop = object_model.get_part("paper_stop")
    arm_swing = object_model.get_articulation("arm_swing")
    stop_slide = object_model.get_articulation("paper_stop_slide")

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
        "arm_joint_axis_vertical",
        tuple(float(v) for v in arm_swing.axis) == (0.0, 0.0, 1.0),
        f"expected vertical swing axis, got {arm_swing.axis}",
    )
    ctx.check(
        "stop_joint_axis_longitudinal",
        tuple(float(v) for v in stop_slide.axis) == (1.0, 0.0, 0.0),
        f"expected x-axis slide, got {stop_slide.axis}",
    )

    arm_limits = arm_swing.motion_limits
    stop_limits = stop_slide.motion_limits
    ctx.check(
        "arm_motion_range_realistic",
        arm_limits is not None
        and arm_limits.lower == 0.0
        and arm_limits.upper is not None
        and 1.2 <= arm_limits.upper <= 2.0,
        f"arm range should open to a realistic cutting angle, got {arm_limits}",
    )
    ctx.check(
        "stop_travel_realistic",
        stop_limits is not None
        and stop_limits.lower is not None
        and stop_limits.upper is not None
        and 0.20 <= stop_limits.upper - stop_limits.lower <= 0.40,
        f"paper stop travel should span the guide length, got {stop_limits}",
    )

    bed_aabb = ctx.part_element_world_aabb(base, elem="bed")
    fence_aabb = ctx.part_element_world_aabb(base, elem="fence")
    rail_aabb = ctx.part_element_world_aabb(base, elem="guide_rail")
    pivot_aabb = ctx.part_element_world_aabb(base, elem="pivot_post")

    if bed_aabb is not None and fence_aabb is not None:
        ctx.check(
            "fence_seated_on_bed",
            abs(fence_aabb[0][2] - bed_aabb[1][2]) <= 1e-6,
            f"fence should sit on the bed top, got bed top {bed_aabb[1][2]:.6f} and fence bottom {fence_aabb[0][2]:.6f}",
        )
    else:
        ctx.fail("fence_geometry_present", "bed or fence AABB was unavailable")

    if bed_aabb is not None and rail_aabb is not None:
        ctx.check(
            "guide_rail_seated_on_bed",
            abs(rail_aabb[0][2] - bed_aabb[1][2]) <= 1e-6,
            f"guide rail should sit on the bed top, got bed top {bed_aabb[1][2]:.6f} and rail bottom {rail_aabb[0][2]:.6f}",
        )
    else:
        ctx.fail("guide_rail_geometry_present", "bed or guide rail AABB was unavailable")

    if fence_aabb is not None and rail_aabb is not None:
        fence_to_rail_gap = fence_aabb[0][1] - rail_aabb[1][1]
        ctx.check(
            "guide_rail_runs_near_fence",
            0.010 <= fence_to_rail_gap <= 0.025,
            f"guide rail should ride just ahead of the fence, got y-gap {fence_to_rail_gap:.6f}",
        )
    else:
        ctx.fail("fence_rail_alignment_present", "fence or guide rail AABB was unavailable")

    if bed_aabb is not None and pivot_aabb is not None:
        ctx.check(
            "pivot_at_back_left_corner",
            pivot_aabb[0][0] < bed_aabb[0][0] + 0.03 and pivot_aabb[1][1] >= bed_aabb[1][1] - 0.03,
            f"pivot should read as a back-left corner mount, got pivot AABB {pivot_aabb}",
        )
    else:
        ctx.fail("pivot_geometry_present", "bed or pivot AABB was unavailable")

    ctx.expect_contact(
        blade_arm,
        base,
        elem_a="hub",
        elem_b="pivot_post",
        name="arm_hub_bears_on_pivot_post",
    )

    with ctx.pose({arm_swing: 0.0}):
        ctx.expect_gap(
            blade_arm,
            base,
            axis="z",
            positive_elem="blade_bar",
            negative_elem="bed",
            min_gap=0.012,
            max_gap=0.022,
            name="blade_bar_clear_of_bed_when_closed",
        )
        ctx.expect_overlap(
            blade_arm,
            base,
            axes="xy",
            elem_a="backbone",
            elem_b="bed",
            min_overlap=0.18,
            name="blade_arm_spans_the_bed",
        )

    with ctx.pose({stop_slide: 0.0}):
        ctx.expect_gap(
            paper_stop,
            base,
            axis="z",
            positive_elem="stop_plate",
            negative_elem="bed",
            max_gap=0.001,
            max_penetration=0.0,
            name="stop_plate_seats_on_bed",
        )
        ctx.expect_gap(
            paper_stop,
            base,
            axis="z",
            positive_elem="bridge",
            negative_elem="guide_rail",
            min_gap=0.001,
            max_gap=0.010,
            name="stop_bridge_clears_guide_rail",
        )
        ctx.expect_gap(
            base,
            paper_stop,
            axis="y",
            positive_elem="fence",
            negative_elem="rear_cheek",
            min_gap=0.006,
            max_gap=0.020,
            name="paper_stop_runs_just_ahead_of_fence",
        )
        ctx.expect_within(
            paper_stop,
            base,
            axes="x",
            inner_elem="bridge",
            outer_elem="guide_rail",
            margin=0.002,
            name="paper_stop_bridge_within_guide_at_home",
        )

    if arm_limits is not None and arm_limits.lower is not None and arm_limits.upper is not None:
        with ctx.pose({arm_swing: arm_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="arm_swing_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="arm_swing_lower_no_floating")
            ctx.expect_contact(
                blade_arm,
                base,
                elem_a="hub",
                elem_b="pivot_post",
                name="arm_hub_contact_at_closed_limit",
            )
        with ctx.pose({arm_swing: arm_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="arm_swing_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="arm_swing_upper_no_floating")
            ctx.expect_contact(
                blade_arm,
                base,
                elem_a="hub",
                elem_b="pivot_post",
                name="arm_hub_contact_at_open_limit",
            )

    if stop_limits is not None and stop_limits.lower is not None and stop_limits.upper is not None:
        with ctx.pose({stop_slide: stop_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="paper_stop_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="paper_stop_lower_no_floating")
            ctx.expect_gap(
                paper_stop,
                base,
                axis="z",
                positive_elem="stop_plate",
                negative_elem="bed",
                max_gap=0.001,
                max_penetration=0.0,
                name="paper_stop_lower_plate_on_bed",
            )
            ctx.expect_within(
                paper_stop,
                base,
                axes="x",
                inner_elem="bridge",
                outer_elem="guide_rail",
                margin=0.002,
                name="paper_stop_lower_within_guide",
            )
        with ctx.pose({stop_slide: stop_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="paper_stop_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="paper_stop_upper_no_floating")
            ctx.expect_gap(
                paper_stop,
                base,
                axis="z",
                positive_elem="stop_plate",
                negative_elem="bed",
                max_gap=0.001,
                max_penetration=0.0,
                name="paper_stop_upper_plate_on_bed",
            )
            ctx.expect_within(
                paper_stop,
                base,
                axes="x",
                inner_elem="bridge",
                outer_elem="guide_rail",
                margin=0.002,
                name="paper_stop_upper_within_guide",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
