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


BASE_RADIUS = 0.17
BASE_HEIGHT = 0.03
CORE_RADIUS = 0.0195

COLLAR_BORE_RADIUS = 0.028
COLLAR_OUTER_RADIUS = 0.044
COLLAR_HEIGHT = 0.024
RETAINER_OUTER_RADIUS = 0.036
RETAINER_HEIGHT = 0.006

LOWER_COLLAR_CENTER_Z = 0.128
UPPER_COLLAR_CENTER_Z = 0.192

ARM_LENGTH = 0.215
ARM_ROOT_WIDTH = 0.044
ARM_TIP_WIDTH = 0.024
ARM_THICKNESS = 0.016
ARM_TIP_RADIUS = 0.014

LOWER_SWEEP = -3.05
UPPER_SWEEP = 3.05


def _frame_segment(z0: float, z1: float, radius: float = CORE_RADIUS) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(z1 - z0).translate((0.0, 0.0, z0))


def _retainer_at(z0: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(RETAINER_OUTER_RADIUS)
        .circle(CORE_RADIUS)
        .extrude(RETAINER_HEIGHT)
        .translate((0.0, 0.0, z0))
    )


def _stand_frame_shape() -> cq.Workplane:
    lower_seat_z0 = LOWER_COLLAR_CENTER_Z - (COLLAR_HEIGHT / 2.0) - RETAINER_HEIGHT
    upper_seat_z0 = UPPER_COLLAR_CENTER_Z - (COLLAR_HEIGHT / 2.0) - RETAINER_HEIGHT

    base = cq.Workplane("XY").circle(BASE_RADIUS).extrude(BASE_HEIGHT)
    foot_pad = cq.Workplane("XY").circle(0.078).extrude(0.015).translate((0.0, 0.0, BASE_HEIGHT))
    tower_housing = cq.Workplane("XY").circle(0.057).extrude(0.045).translate((0.0, 0.0, BASE_HEIGHT + 0.015))
    spine = _frame_segment(BASE_HEIGHT + 0.06, 0.252)
    top_button = (
        cq.Workplane("XY")
        .circle(0.022)
        .extrude(0.018)
        .translate((0.0, 0.0, 0.252))
    )

    return (
        base.union(foot_pad)
        .union(tower_housing)
        .union(spine)
        .union(top_button)
    )


def _seat_ring_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(RETAINER_OUTER_RADIUS)
        .circle(CORE_RADIUS)
        .extrude(RETAINER_HEIGHT)
    )


def _arm_shape() -> cq.Workplane:
    collar = cq.Workplane("XY").circle(COLLAR_OUTER_RADIUS).extrude(COLLAR_HEIGHT / 2.0, both=True)

    arm_plate = (
        cq.Workplane("XY")
        .moveTo(COLLAR_OUTER_RADIUS * 0.78, ARM_ROOT_WIDTH / 2.0)
        .lineTo(COLLAR_OUTER_RADIUS + ARM_LENGTH * 0.28, ARM_ROOT_WIDTH / 2.0)
        .lineTo(COLLAR_OUTER_RADIUS + ARM_LENGTH, ARM_TIP_WIDTH / 2.0)
        .lineTo(COLLAR_OUTER_RADIUS + ARM_LENGTH, -ARM_TIP_WIDTH / 2.0)
        .lineTo(COLLAR_OUTER_RADIUS + ARM_LENGTH * 0.28, -ARM_ROOT_WIDTH / 2.0)
        .lineTo(COLLAR_OUTER_RADIUS * 0.78, -ARM_ROOT_WIDTH / 2.0)
        .close()
        .extrude(ARM_THICKNESS / 2.0, both=True)
    )
    root_boss = (
        cq.Workplane("XY")
        .center(COLLAR_OUTER_RADIUS + 0.032, 0.0)
        .ellipse(0.038, ARM_ROOT_WIDTH * 0.76)
        .extrude(ARM_THICKNESS / 2.0, both=True)
    )
    tip_pad = (
        cq.Workplane("XY")
        .center(COLLAR_OUTER_RADIUS + ARM_LENGTH, 0.0)
        .circle(ARM_TIP_RADIUS)
        .extrude(ARM_THICKNESS / 2.0, both=True)
    )
    gusset = (
        cq.Workplane("XZ")
        .moveTo(COLLAR_OUTER_RADIUS * 0.8, -ARM_THICKNESS / 2.0)
        .lineTo(COLLAR_OUTER_RADIUS + ARM_LENGTH * 0.26, -ARM_THICKNESS / 2.0)
        .lineTo(COLLAR_OUTER_RADIUS + ARM_LENGTH * 0.16, -COLLAR_HEIGHT / 2.0 + 0.003)
        .lineTo(COLLAR_OUTER_RADIUS * 0.78, -COLLAR_HEIGHT / 2.0 + 0.003)
        .close()
        .extrude(ARM_ROOT_WIDTH * 0.56, both=True)
    )

    arm = collar.union(root_boss).union(arm_plate).union(tip_pad).union(gusset)
    bore = cq.Workplane("XY").circle(COLLAR_BORE_RADIUS).extrude((COLLAR_HEIGHT + 0.01) / 2.0, both=True)
    return arm.cut(bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_rotary_stand")

    base_finish = model.material("base_finish", rgba=(0.14, 0.14, 0.16, 1.0))
    arm_finish = model.material("arm_finish", rgba=(0.72, 0.74, 0.77, 1.0))

    stand = model.part("stand")
    stand.visual(
        mesh_from_cadquery(_stand_frame_shape(), "stand_frame"),
        material=base_finish,
        name="stand_frame",
    )
    stand.visual(
        mesh_from_cadquery(_seat_ring_shape(), "lower_seat"),
        origin=Origin(xyz=(0.0, 0.0, LOWER_COLLAR_CENTER_Z - (COLLAR_HEIGHT / 2.0) - RETAINER_HEIGHT)),
        material=base_finish,
        name="lower_seat",
    )
    stand.visual(
        mesh_from_cadquery(_seat_ring_shape(), "upper_seat"),
        origin=Origin(xyz=(0.0, 0.0, UPPER_COLLAR_CENTER_Z - (COLLAR_HEIGHT / 2.0) - RETAINER_HEIGHT)),
        material=base_finish,
        name="upper_seat",
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        mesh_from_cadquery(_arm_shape(), "lower_arm_body"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=arm_finish,
        name="lower_arm_body",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        mesh_from_cadquery(_arm_shape(), "upper_arm_body"),
        origin=Origin(rpy=(0.0, 0.0, math.pi)),
        material=arm_finish,
        name="upper_arm_body",
    )

    model.articulation(
        "stand_to_lower_arm",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=lower_arm,
        origin=Origin(xyz=(0.0, 0.0, LOWER_COLLAR_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.2,
            lower=LOWER_SWEEP,
            upper=-LOWER_SWEEP,
        ),
    )
    model.articulation(
        "stand_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, UPPER_COLLAR_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.2,
            lower=-UPPER_SWEEP,
            upper=UPPER_SWEEP,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    lower_joint = object_model.get_articulation("stand_to_lower_arm")
    upper_joint = object_model.get_articulation("stand_to_upper_arm")

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
        "stand_parts_present",
        all(part is not None for part in (stand, lower_arm, upper_arm)),
        "stand or arm parts could not be resolved",
    )
    ctx.check(
        "tower_axes_are_vertical",
        lower_joint.axis == (0.0, 0.0, 1.0) and upper_joint.axis == (0.0, 0.0, 1.0),
        "both arm collars should revolve about the tower's supported vertical axis",
    )
    ctx.check(
        "independent_rotary_ranges",
        lower_joint.motion_limits is not None
        and upper_joint.motion_limits is not None
        and lower_joint.motion_limits.lower is not None
        and lower_joint.motion_limits.upper is not None
        and upper_joint.motion_limits.lower is not None
        and upper_joint.motion_limits.upper is not None
        and lower_joint.motion_limits.upper - lower_joint.motion_limits.lower > 6.0
        and upper_joint.motion_limits.upper - upper_joint.motion_limits.lower > 6.0,
        "each collar should have a near-full-turn revolute sweep",
    )

    ctx.expect_contact(stand, lower_arm, elem_a="lower_seat", name="lower_collar_seats_on_stand")
    ctx.expect_contact(stand, upper_arm, elem_a="upper_seat", name="upper_collar_seats_on_stand")
    ctx.expect_origin_distance(
        lower_arm,
        stand,
        axes="xy",
        max_dist=0.001,
        name="lower_arm_is_concentric_with_tower",
    )
    ctx.expect_origin_distance(
        upper_arm,
        stand,
        axes="xy",
        max_dist=0.001,
        name="upper_arm_is_concentric_with_tower",
    )
    ctx.expect_origin_gap(
        upper_arm,
        lower_arm,
        axis="z",
        min_gap=0.055,
        max_gap=0.075,
        name="arm_collars_are_stacked_on_separate_levels",
    )
    ctx.expect_gap(
        upper_arm,
        lower_arm,
        axis="z",
        min_gap=0.025,
        name="arms_remain_vertically_clear",
    )

    with ctx.pose({lower_joint: 1.35, upper_joint: -1.6}):
        ctx.expect_contact(stand, lower_arm, elem_a="lower_seat", name="lower_arm_stays_supported_when_rotated")
        ctx.expect_contact(stand, upper_arm, elem_a="upper_seat", name="upper_arm_stays_supported_when_rotated")
        ctx.expect_gap(
            upper_arm,
            lower_arm,
            axis="z",
            min_gap=0.025,
            name="arms_clear_when_counter_rotated",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
