from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_FLANGE_RADIUS = 0.180
BASE_FLANGE_HEIGHT = 0.032
BASE_BODY_RADIUS = 0.126
BASE_BODY_HEIGHT = 0.110
BASE_CAP_RADIUS = 0.148
BASE_CAP_HEIGHT = 0.050
BASE_HEIGHT = BASE_FLANGE_HEIGHT + BASE_BODY_HEIGHT + BASE_CAP_HEIGHT

SHOULDER_RADIUS = 0.128
SHOULDER_STEP_RADIUS = 0.108
SHOULDER_HEIGHT = 0.110

UPPER_ARM_LENGTH = 0.420
UPPER_ARM_WIDTH = 0.150
UPPER_ARM_ELBOW_Z = 0.110
ELBOW_AXIS_X = 0.438
ELBOW_GAP = 0.078
ELBOW_OUTER_WIDTH = 0.162
ELBOW_CHEEK_THICKNESS = (ELBOW_OUTER_WIDTH - ELBOW_GAP) / 2.0
ELBOW_CHEEK_X = 0.092
ELBOW_CHEEK_Z = 0.148
ELBOW_CAP_RADIUS = 0.052
ELBOW_CAP_LENGTH = 0.012

FOREARM_LENGTH = 0.325
FOREARM_WIDTH = 0.104
FOREARM_HUB_RADIUS = 0.034
FOREARM_HUB_WIDTH = 0.070
FOREARM_THRUST_WASHER_RADIUS = 0.046
FOREARM_THRUST_WASHER_THICKNESS = 0.004
FOREARM_NECK_WIDTH = 0.058

WRIST_COLLAR_RADIUS = 0.050
WRIST_COLLAR_BORE = 0.032
WRIST_COLLAR_LENGTH = 0.054
WRIST_REAR_FLANGE_RADIUS = 0.042
WRIST_REAR_FLANGE_LENGTH = 0.008
WRIST_ROTOR_RADIUS = 0.040
WRIST_ROTOR_LENGTH = 0.046
WRIST_NOSE_RADIUS = 0.028
WRIST_NOSE_LENGTH = 0.036
WRIST_TRANSITION_START = 0.054
WRIST_TRANSITION_LENGTH = 0.016
WRIST_PLATE_SIZE = 0.072
WRIST_PLATE_THICKNESS = 0.014
WRIST_PLATE_START = 0.116


def _polar_points(radius: float, count: int, phase: float = 0.0) -> list[tuple[float, float]]:
    return [
        (
            radius * cos(phase + (2.0 * pi * index) / count),
            radius * sin(phase + (2.0 * pi * index) / count),
        )
        for index in range(count)
    ]


def _build_base_shape() -> cq.Workplane:
    flange = cq.Workplane("XY").circle(BASE_FLANGE_RADIUS).extrude(BASE_FLANGE_HEIGHT)
    body = (
        cq.Workplane("XY")
        .workplane(offset=BASE_FLANGE_HEIGHT)
        .circle(BASE_BODY_RADIUS)
        .extrude(BASE_BODY_HEIGHT)
    )
    cap = (
        cq.Workplane("XY")
        .workplane(offset=BASE_FLANGE_HEIGHT + BASE_BODY_HEIGHT)
        .circle(BASE_CAP_RADIUS)
        .extrude(BASE_CAP_HEIGHT)
    )
    conduit = (
        cq.Workplane("XY")
        .box(0.102, 0.082, 0.104)
        .translate((-0.090, 0.0, BASE_FLANGE_HEIGHT + 0.052))
    )

    mount_holes = (
        cq.Workplane("XY")
        .pushPoints(_polar_points(0.130, 4, pi / 4.0))
        .circle(0.013)
        .extrude(BASE_FLANGE_HEIGHT + 0.004)
    )
    service_slot = (
        cq.Workplane("XY")
        .box(0.026, 0.038, 0.052)
        .translate((-0.118, 0.0, BASE_FLANGE_HEIGHT + BASE_BODY_HEIGHT - 0.012))
    )
    top_service_flat = (
        cq.Workplane("XY")
        .box(0.072, 0.064, 0.012)
        .translate((-0.080, 0.0, BASE_HEIGHT - 0.006))
    )

    return flange.union(body).union(cap).union(conduit).cut(mount_holes).cut(service_slot).cut(top_service_flat)


def _build_upper_arm_shape() -> cq.Workplane:
    shoulder = cq.Workplane("XY").circle(SHOULDER_RADIUS).extrude(SHOULDER_HEIGHT)
    shoulder_step = (
        cq.Workplane("XY")
        .workplane(offset=SHOULDER_HEIGHT - 0.018)
        .circle(SHOULDER_STEP_RADIUS)
        .extrude(0.018)
    )

    beam_lower = (
        cq.Workplane("XY")
        .box(0.290, 0.128, 0.080)
        .translate((0.185, 0.0, 0.056))
        .edges("|Z")
        .fillet(0.012)
    )
    beam_upper = (
        cq.Workplane("XY")
        .box(0.165, 0.090, 0.026)
        .translate((0.155, 0.0, 0.100))
        .edges("|Z")
        .fillet(0.010)
    )

    cheek_pos = (
        cq.Workplane("XY")
        .box(0.060, 0.026, 0.100)
        .translate((0.398, 0.051, UPPER_ARM_ELBOW_Z))
    )
    cheek_neg = (
        cq.Workplane("XY")
        .box(0.060, 0.026, 0.100)
        .translate((0.398, -0.051, UPPER_ARM_ELBOW_Z))
    )
    cover_pos = (
        cq.Workplane("XZ")
        .circle(0.044)
        .extrude(0.010)
        .translate((0.422, 0.064, UPPER_ARM_ELBOW_Z))
    )
    cover_neg = (
        cq.Workplane("XZ")
        .circle(0.044)
        .extrude(0.010)
        .translate((0.422, -0.074, UPPER_ARM_ELBOW_Z))
    )

    top_channel = (
        cq.Workplane("XY")
        .workplane(offset=0.094)
        .center(0.188, 0.0)
        .slot2D(0.164, 0.024)
        .extrude(0.016)
    )
    side_window = (
        cq.Workplane("XZ")
        .center(0.214, 0.062)
        .slot2D(0.128, 0.034)
        .extrude(0.028, both=True)
    )

    return (
        shoulder.union(shoulder_step)
        .union(beam_lower)
        .union(beam_upper)
        .union(cheek_pos)
        .union(cheek_neg)
        .union(cover_pos)
        .union(cover_neg)
        .cut(top_channel)
        .cut(side_window)
    )


def _build_forearm_shape() -> cq.Workplane:
    root_tongue = cq.Workplane("XY").box(0.036, 0.076, 0.050).translate((0.006, 0.0, -0.014))
    beam_lower = (
        cq.Workplane("XY")
        .box(0.212, 0.090, 0.058)
        .translate((0.152, 0.0, -0.014))
        .edges("|Z")
        .fillet(0.009)
    )
    beam_upper = (
        cq.Workplane("XY")
        .box(0.120, 0.070, 0.022)
        .translate((0.186, 0.0, 0.018))
        .edges("|Z")
        .fillet(0.007)
    )
    root_boss = (
        cq.Workplane("YZ")
        .workplane(offset=-0.002)
        .circle(0.028)
        .extrude(0.014)
    )
    collar = (
        cq.Workplane("YZ")
        .workplane(offset=FOREARM_LENGTH - WRIST_COLLAR_LENGTH)
        .circle(0.044)
        .extrude(WRIST_COLLAR_LENGTH)
    )
    collar_face = (
        cq.Workplane("YZ")
        .workplane(offset=FOREARM_LENGTH - 0.012)
        .circle(0.038)
        .extrude(0.012)
    )
    top_channel = (
        cq.Workplane("XY")
        .workplane(offset=0.026)
        .center(0.184, 0.0)
        .slot2D(0.100, 0.016)
        .extrude(0.014)
    )
    side_window = (
        cq.Workplane("XZ")
        .center(0.184, -0.008)
        .slot2D(0.072, 0.020)
        .extrude(0.014, both=True)
    )
    wrist_bore = (
        cq.Workplane("YZ")
        .workplane(offset=FOREARM_LENGTH - WRIST_COLLAR_LENGTH - 0.001)
        .circle(0.028)
        .extrude(WRIST_COLLAR_LENGTH + 0.002)
    )

    return (
        root_tongue.union(root_boss)
        .union(beam_lower)
        .union(beam_upper)
        .union(collar)
        .union(collar_face)
        .cut(top_channel)
        .cut(side_window)
        .cut(wrist_bore)
    )


def _build_wrist_shape() -> cq.Workplane:
    rear_flange = (
        cq.Workplane("YZ")
        .circle(0.034)
        .extrude(WRIST_REAR_FLANGE_LENGTH)
    )
    rotor = (
        cq.Workplane("YZ")
        .workplane(offset=WRIST_REAR_FLANGE_LENGTH)
        .circle(0.034)
        .extrude(0.038)
    )
    transition = (
        cq.Workplane("YZ")
        .workplane(offset=0.046)
        .circle(0.030)
        .workplane(offset=0.014)
        .circle(0.024)
        .loft(combine=True)
    )
    nose = (
        cq.Workplane("YZ")
        .workplane(offset=0.060)
        .circle(0.024)
        .extrude(0.024)
    )
    tool_boss = cq.Workplane("YZ").workplane(offset=0.084).circle(0.015).extrude(0.010)
    tool_plate = (
        cq.Workplane("YZ")
        .workplane(offset=0.094)
        .rect(0.066, 0.066)
        .extrude(0.014)
    )
    plate_holes = (
        cq.Workplane("YZ")
        .workplane(offset=0.093)
        .pushPoints([(0.018, 0.018), (0.018, -0.018), (-0.018, 0.018), (-0.018, -0.018)])
        .circle(0.0045)
        .extrude(0.016)
    )

    return rear_flange.union(rotor).union(transition).union(nose).union(tool_boss).union(tool_plate).cut(plate_holes)


def _aabb_dims(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return (maxs[0] - mins[0], maxs[1] - mins[1], maxs[2] - mins[2])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="machine_tending_arm")

    model.material("base_charcoal", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("cast_aluminum", rgba=(0.70, 0.72, 0.75, 1.0))
    model.material("machined_steel", rgba=(0.57, 0.60, 0.64, 1.0))
    model.material("dark_tooling", rgba=(0.24, 0.25, 0.28, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_shape(), "machine_arm_base"),
        material="base_charcoal",
        name="base_shell",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.360, 0.360, BASE_HEIGHT)),
        mass=38.0,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT / 2.0)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        mesh_from_cadquery(_build_upper_arm_shape(), "machine_arm_upper_arm"),
        material="cast_aluminum",
        name="upper_arm_shell",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.455, 0.168, 0.160)),
        mass=18.0,
        origin=Origin(xyz=(0.225, 0.0, 0.080)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        mesh_from_cadquery(_build_forearm_shape(), "machine_arm_forearm"),
        material="machined_steel",
        name="forearm_shell",
    )
    forearm.inertial = Inertial.from_geometry(
        Box((0.340, 0.112, 0.118)),
        mass=10.5,
        origin=Origin(xyz=(0.170, 0.0, 0.004)),
    )

    wrist = model.part("wrist")
    wrist.visual(
        mesh_from_cadquery(_build_wrist_shape(), "machine_arm_wrist"),
        material="dark_tooling",
        name="wrist_shell",
    )
    wrist.inertial = Inertial.from_geometry(
        Box((0.160, 0.090, 0.090)),
        mass=4.2,
        origin=Origin(xyz=(0.080, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-2.60, upper=2.60, effort=520.0, velocity=1.4),
    )
    model.articulation(
        "upper_arm_to_forearm",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(ELBOW_AXIS_X, 0.0, UPPER_ARM_ELBOW_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.85, effort=340.0, velocity=1.6),
    )
    model.articulation(
        "forearm_to_wrist",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist,
        origin=Origin(xyz=(FOREARM_LENGTH, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-2.90, upper=2.90, effort=160.0, velocity=2.4),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist = object_model.get_part("wrist")

    shoulder = object_model.get_articulation("base_to_upper_arm")
    elbow = object_model.get_articulation("upper_arm_to_forearm")
    wrist_roll = object_model.get_articulation("forearm_to_wrist")

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
    ctx.allow_overlap(
        forearm,
        wrist,
        reason="Wrist roll cartridge is intentionally nested inside the forearm collar housing as a close-fit bearing package.",
    )
    ctx.allow_overlap(
        upper_arm,
        forearm,
        reason="Elbow trunnion is intentionally nested inside the upper-arm clevis as a compact bearing package; exact elbow contact checks cover the supported joint seat.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "all_primary_parts_present",
        all(part.name for part in (base, upper_arm, forearm, wrist)),
        "Base, upper arm, forearm, and wrist must all resolve.",
    )
    ctx.check(
        "joint_axes_match_robot_mechanism",
        shoulder.axis == (0.0, 0.0, 1.0)
        and elbow.axis == (0.0, 1.0, 0.0)
        and wrist_roll.axis == (1.0, 0.0, 0.0),
        (
            f"Expected yaw/pitch/roll axes, got shoulder={shoulder.axis}, "
            f"elbow={elbow.axis}, wrist={wrist_roll.axis}"
        ),
    )
    ctx.check(
        "joint_ranges_are_believable",
        shoulder.motion_limits is not None
        and elbow.motion_limits is not None
        and wrist_roll.motion_limits is not None
        and shoulder.motion_limits.lower is not None
        and shoulder.motion_limits.upper is not None
        and elbow.motion_limits.lower is not None
        and elbow.motion_limits.upper is not None
        and wrist_roll.motion_limits.lower is not None
        and wrist_roll.motion_limits.upper is not None
        and shoulder.motion_limits.lower < -2.0
        and shoulder.motion_limits.upper > 2.0
        and elbow.motion_limits.lower <= 0.0
        and elbow.motion_limits.upper >= 0.8
        and wrist_roll.motion_limits.lower < -2.5
        and wrist_roll.motion_limits.upper > 2.5,
        "Shoulder, elbow, and wrist need realistic industrial-arm travel limits.",
    )

    with ctx.pose({shoulder: 0.0, elbow: 0.0, wrist_roll: 0.0}):
        ctx.expect_contact(base, upper_arm, contact_tol=0.001, name="shoulder_bearing_contact")
        ctx.expect_overlap(base, upper_arm, axes="xy", min_overlap=0.180, name="shoulder_bearing_seat")

        ctx.expect_contact(upper_arm, forearm, contact_tol=0.001, name="elbow_bearing_contact")
        ctx.expect_overlap(upper_arm, forearm, axes="yz", min_overlap=0.070, name="elbow_hinge_overlap")

        ctx.expect_contact(forearm, wrist, contact_tol=0.001, name="wrist_roll_contact")
        ctx.expect_overlap(forearm, wrist, axes="yz", min_overlap=0.065, name="wrist_roll_overlap")

        base_dims = _aabb_dims(ctx.part_world_aabb(base))
        upper_dims = _aabb_dims(ctx.part_world_aabb(upper_arm))
        fore_dims = _aabb_dims(ctx.part_world_aabb(forearm))
        wrist_dims = _aabb_dims(ctx.part_world_aabb(wrist))
        hierarchy_ok = (
            base_dims is not None
            and upper_dims is not None
            and fore_dims is not None
            and wrist_dims is not None
            and base_dims[0] >= 0.34
            and upper_dims[1] > fore_dims[1] > wrist_dims[1]
            and upper_dims[2] > fore_dims[2] > 0.06
        )
        ctx.check(
            "heavy_to_light_hierarchy_reads_clearly",
            hierarchy_ok,
            (
                f"Expected base > upper arm > forearm > wrist bulk hierarchy, got "
                f"base={base_dims}, upper={upper_dims}, forearm={fore_dims}, wrist={wrist_dims}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
