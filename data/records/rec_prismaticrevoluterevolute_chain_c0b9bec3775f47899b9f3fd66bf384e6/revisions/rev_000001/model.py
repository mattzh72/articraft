from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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


WALL_T = 0.028
WALL_W = 0.260
WALL_H = 1.250

SPINE_D = 0.032
SPINE_W = 0.084
SPINE_H = 0.980

GUIDE_D = 0.026
GUIDE_W = 0.055
GUIDE_H = 0.900

TOP_CAP_H = 0.080

CARRIAGE_X = 0.109
CARRIAGE_START_Z = -0.320
SLIDE_TRAVEL = 0.580

SHOULDER_OFFSET = (0.060, 0.000, 0.020)
ELBOW_OFFSET_X = 0.360


def _base_shape() -> cq.Workplane:
    wall = cq.Workplane("XY").box(WALL_T, WALL_W, WALL_H).translate((WALL_T / 2.0, 0.0, 0.0))
    spine = cq.Workplane("XY").box(SPINE_D, SPINE_W, SPINE_H).translate(
        (WALL_T + SPINE_D / 2.0, 0.0, 0.0)
    )
    guide = cq.Workplane("XY").box(GUIDE_D, GUIDE_W, GUIDE_H).translate(
        (WALL_T + SPINE_D + GUIDE_D / 2.0, 0.0, 0.0)
    )
    top_cap = cq.Workplane("XY").box(0.060, 0.110, TOP_CAP_H).translate((0.058, 0.0, 0.490))
    bottom_cap = cq.Workplane("XY").box(0.060, 0.110, TOP_CAP_H).translate((0.058, 0.0, -0.490))
    top_shoulder = cq.Workplane("XY").box(0.030, 0.100, 0.040).translate((0.074, 0.0, 0.435))
    bottom_shoulder = cq.Workplane("XY").box(0.030, 0.100, 0.040).translate((0.074, 0.0, -0.435))

    holes = (
        cq.Workplane("YZ")
        .pushPoints([(-0.080, 0.430), (0.080, 0.430), (-0.080, -0.430), (0.080, -0.430)])
        .circle(0.007)
        .extrude(WALL_T + 0.002)
    )

    return (
        wall.union(spine)
        .union(guide)
        .union(top_cap)
        .union(bottom_cap)
        .union(top_shoulder)
        .union(bottom_shoulder)
        .cut(holes)
    )


def _carriage_shape() -> cq.Workplane:
    slider_block = cq.Workplane("XY").box(0.040, 0.050, 0.152).translate((0.000, 0.0, 0.0))
    rear_wear_pad = cq.Workplane("XY").box(0.008, 0.040, 0.120).translate((-0.019, 0.0, 0.0))
    front_plate = cq.Workplane("XY").box(0.028, 0.108, 0.132).translate((0.030, 0.0, 0.004))
    spine_web = cq.Workplane("XY").box(0.024, 0.040, 0.084).translate((0.044, 0.0, 0.020))
    upper_ear = cq.Workplane("XY").box(0.024, 0.012, 0.072).translate((0.050, 0.018, 0.020))
    lower_ear = cq.Workplane("XY").box(0.024, 0.012, 0.072).translate((0.050, -0.018, 0.020))
    lower_brace = cq.Workplane("XY").box(0.020, 0.064, 0.040).translate((0.028, 0.0, -0.050))
    pocket = cq.Workplane("XY").box(0.026, 0.022, 0.084).translate((0.042, 0.0, 0.020))

    return (
        slider_block.union(rear_wear_pad)
        .union(front_plate)
        .union(spine_web)
        .union(upper_ear)
        .union(lower_ear)
        .union(lower_brace)
        .cut(pocket)
    )


def _upper_arm_shape() -> cq.Workplane:
    shoulder_tongue = cq.Workplane("XY").box(0.024, 0.024, 0.052).translate((0.012, 0.0, 0.000))
    shoulder_cap = cq.Workplane("XY").box(0.030, 0.018, 0.022).translate((0.020, 0.0, 0.020))
    main_beam = cq.Workplane("XY").box(0.290, 0.026, 0.036).translate((0.157, 0.0, 0.000))
    top_rib = cq.Workplane("XY").box(0.180, 0.018, 0.014).translate((0.182, 0.0, 0.023))
    elbow_block = cq.Workplane("XY").box(0.036, 0.032, 0.040).translate((0.320, 0.0, 0.000))
    upper_fork = cq.Workplane("XY").box(0.032, 0.012, 0.054).translate((0.346, 0.016, 0.000))
    lower_fork = cq.Workplane("XY").box(0.032, 0.012, 0.054).translate((0.346, -0.016, 0.000))

    return (
        shoulder_tongue.union(shoulder_cap)
        .union(main_beam)
        .union(top_rib)
        .union(elbow_block)
        .union(upper_fork)
        .union(lower_fork)
    )


def _tip_link_shape() -> cq.Workplane:
    root_tongue = cq.Workplane("XY").box(0.022, 0.020, 0.046).translate((0.011, 0.0, 0.000))
    forearm = cq.Workplane("XY").box(0.168, 0.028, 0.036).translate((0.106, 0.0, 0.000))
    neck = cq.Workplane("XY").box(0.030, 0.022, 0.024).translate((0.205, 0.0, -0.002))
    tool_plate = cq.Workplane("XY").box(0.052, 0.066, 0.014).translate((0.244, 0.0, -0.010))
    tool_pad = cq.Workplane("YZ").circle(0.016).extrude(0.018, both=True).translate((0.270, 0.0, -0.010))
    top_pad = cq.Workplane("XY").box(0.026, 0.018, 0.014).translate((0.050, 0.0, 0.018))

    return root_tongue.union(forearm).union(neck).union(tool_plate).union(tool_pad).union(top_pad)


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lo, hi = aabb
    return tuple((a + b) * 0.5 for a, b in zip(lo, hi))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_transfer_axis")

    model.material("wall_graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("carriage_gray", rgba=(0.38, 0.40, 0.43, 1.0))
    model.material("arm_silver", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("tool_dark", rgba=(0.22, 0.23, 0.25, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_shape(), "base"),
        origin=Origin(),
        material="wall_graphite",
        name="base_shell",
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_shape(), "carriage"),
        origin=Origin(),
        material="carriage_gray",
        name="carriage_shell",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        mesh_from_cadquery(_upper_arm_shape(), "upper_arm"),
        origin=Origin(),
        material="arm_silver",
        name="upper_arm_shell",
    )

    tip_link = model.part("tip_link")
    tip_link.visual(
        mesh_from_cadquery(_tip_link_shape(), "tip_link"),
        origin=Origin(),
        material="tool_dark",
        name="tip_link_shell",
    )

    model.articulation(
        "base_to_carriage_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(CARRIAGE_X, 0.0, CARRIAGE_START_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=800.0,
            velocity=0.35,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )

    model.articulation(
        "carriage_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=upper_arm,
        origin=Origin(xyz=SHOULDER_OFFSET),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.4,
            lower=-0.45,
            upper=1.25,
        ),
    )

    model.articulation(
        "upper_arm_to_tip_link",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=tip_link,
        origin=Origin(xyz=(ELBOW_OFFSET_X, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.8,
            lower=-1.35,
            upper=1.20,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    upper_arm = object_model.get_part("upper_arm")
    tip_link = object_model.get_part("tip_link")

    slide = object_model.get_articulation("base_to_carriage_slide")
    shoulder = object_model.get_articulation("carriage_to_upper_arm")
    wrist = object_model.get_articulation("upper_arm_to_tip_link")

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

    ctx.expect_contact(carriage, base, name="carriage_contacts_guide")
    ctx.expect_contact(upper_arm, carriage, name="shoulder_hinge_is_supported")
    ctx.expect_contact(tip_link, upper_arm, name="tip_hinge_is_supported")

    ctx.check(
        "joint_stack_types",
        slide.articulation_type == ArticulationType.PRISMATIC
        and shoulder.articulation_type == ArticulationType.REVOLUTE
        and wrist.articulation_type == ArticulationType.REVOLUTE,
        "Expected one prismatic joint followed by two revolute joints.",
    )

    ctx.check(
        "joint_axes_match_intent",
        tuple(slide.axis) == (0.0, 0.0, 1.0)
        and tuple(shoulder.axis) == (0.0, -1.0, 0.0)
        and tuple(wrist.axis) == (0.0, -1.0, 0.0),
        f"Unexpected joint axes: slide={slide.axis}, shoulder={shoulder.axis}, wrist={wrist.axis}",
    )

    carriage_rest = ctx.part_world_position(carriage)
    with ctx.pose({slide: SLIDE_TRAVEL}):
        carriage_high = ctx.part_world_position(carriage)
    ctx.check(
        "slide_moves_carriage_up",
        carriage_rest is not None
        and carriage_high is not None
        and carriage_high[2] > carriage_rest[2] + 0.55 * SLIDE_TRAVEL,
        f"Expected carriage to rise along +Z; rest={carriage_rest}, high={carriage_high}",
    )

    with ctx.pose({slide: 0.20, shoulder: 0.0, wrist: 0.0}):
        tip_level = _aabb_center(ctx.part_world_aabb(tip_link))
    with ctx.pose({slide: 0.20, shoulder: 0.85, wrist: 0.0}):
        tip_shoulder_up = _aabb_center(ctx.part_world_aabb(tip_link))
    ctx.check(
        "shoulder_positive_pitch_lifts_tip",
        tip_level is not None
        and tip_shoulder_up is not None
        and tip_shoulder_up[2] > tip_level[2] + 0.10
        and tip_shoulder_up[0] < tip_level[0] - 0.03,
        f"Expected positive shoulder pitch to lift the arm; level={tip_level}, lifted={tip_shoulder_up}",
    )

    with ctx.pose({slide: 0.20, shoulder: 0.55, wrist: 0.0}):
        wrist_level = _aabb_center(ctx.part_world_aabb(tip_link))
    with ctx.pose({slide: 0.20, shoulder: 0.55, wrist: 0.85}):
        wrist_up = _aabb_center(ctx.part_world_aabb(tip_link))
    ctx.check(
        "wrist_positive_pitch_lifts_tool",
        wrist_level is not None
        and wrist_up is not None
        and wrist_up[2] > wrist_level[2] + 0.04
        and wrist_up[0] < wrist_level[0] - 0.01,
        f"Expected positive wrist pitch to lift the tool tip; level={wrist_level}, lifted={wrist_up}",
    )

    slide_limits = slide.motion_limits
    shoulder_limits = shoulder.motion_limits
    wrist_limits = wrist.motion_limits
    ctx.check(
        "motion_limits_are_reasonable",
        slide_limits is not None
        and shoulder_limits is not None
        and wrist_limits is not None
        and math.isclose(slide_limits.upper or 0.0, SLIDE_TRAVEL, rel_tol=0.0, abs_tol=1e-9)
        and (shoulder_limits.lower or 0.0) < 0.0 < (shoulder_limits.upper or 0.0)
        and (wrist_limits.lower or 0.0) < 0.0 < (wrist_limits.upper or 0.0),
        "Unexpected motion limits for the transfer-axis joints.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
