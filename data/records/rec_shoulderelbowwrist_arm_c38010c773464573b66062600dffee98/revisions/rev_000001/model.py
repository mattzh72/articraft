from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


PEDESTAL_HEIGHT = 0.280

UPPER_ARM_LENGTH = 0.305
UPPER_ARM_CENTER_Z = 0.055

FOREARM_LENGTH = 0.255
WRIST_LENGTH = 0.104

ELBOW_HUB_LENGTH = 0.036
ELBOW_HUB_RADIUS = 0.020

WRIST_HUB_LENGTH = 0.028
WRIST_HUB_RADIUS = 0.016


def _pedestal_shape() -> cq.Workplane:
    base_plate = cq.Workplane("XY").circle(0.130).extrude(0.032)
    lower_skirt = cq.Workplane("XY").circle(0.105).extrude(0.018).translate((0.0, 0.0, 0.032))
    center_column = cq.Workplane("XY").circle(0.055).extrude(0.185).translate((0.0, 0.0, 0.032))
    shoulder_head = cq.Workplane("XY").circle(0.080).extrude(0.063).translate((0.0, 0.0, 0.217))

    pedestal = base_plate.union(lower_skirt).union(center_column).union(shoulder_head)

    anchor_cutters = (
        cq.Workplane("XY")
        .pushPoints(
            [
                (-0.082, -0.082),
                (-0.082, 0.082),
                (0.082, -0.082),
                (0.082, 0.082),
            ]
        )
        .circle(0.010)
        .extrude(0.045)
    )

    return pedestal.cut(anchor_cutters)


def _upper_arm_shape() -> cq.Workplane:
    shoulder_turret = cq.Workplane("XY").circle(0.060).extrude(0.060)
    body_profile = (
        cq.Workplane("XZ")
        .moveTo(0.000, 0.016)
        .lineTo(0.030, 0.014)
        .lineTo(0.120, 0.026)
        .lineTo(0.250, 0.030)
        .lineTo(UPPER_ARM_LENGTH, 0.034)
        .lineTo(UPPER_ARM_LENGTH, 0.076)
        .lineTo(0.250, 0.080)
        .lineTo(0.120, 0.086)
        .lineTo(0.030, 0.098)
        .lineTo(0.000, 0.094)
        .close()
        .extrude(0.039, both=True)
    )
    elbow_end = (
        cq.Workplane("YZ")
        .circle(0.024)
        .extrude(0.030)
        .translate((UPPER_ARM_LENGTH - 0.030, 0.0, UPPER_ARM_CENTER_Z))
    )

    return shoulder_turret.union(body_profile).union(elbow_end)


def _forearm_shape() -> cq.Workplane:
    body_profile = (
        cq.Workplane("XZ")
        .moveTo(0.000, -0.014)
        .lineTo(0.018, -0.016)
        .lineTo(0.090, -0.022)
        .lineTo(0.195, -0.026)
        .lineTo(FOREARM_LENGTH, -0.022)
        .lineTo(FOREARM_LENGTH, 0.022)
        .lineTo(0.195, 0.026)
        .lineTo(0.090, 0.022)
        .lineTo(0.018, 0.016)
        .lineTo(0.000, 0.014)
        .close()
        .extrude(0.027, both=True)
    )
    elbow_collar = cq.Workplane("YZ").circle(0.020).extrude(0.036)
    wrist_collar = (
        cq.Workplane("YZ")
        .circle(0.021)
        .extrude(0.036)
        .translate((FOREARM_LENGTH - 0.036, 0.0, 0.0))
    )

    return body_profile.union(elbow_collar).union(wrist_collar)


def _wrist_shape() -> cq.Workplane:
    body_profile = (
        cq.Workplane("XZ")
        .moveTo(0.000, -0.012)
        .lineTo(0.018, -0.014)
        .lineTo(0.060, -0.018)
        .lineTo(0.082, -0.018)
        .lineTo(0.082, 0.018)
        .lineTo(0.060, 0.018)
        .lineTo(0.018, 0.014)
        .lineTo(0.000, 0.012)
        .close()
        .extrude(0.020, both=True)
    )
    wrist_collar = cq.Workplane("YZ").circle(0.016).extrude(0.028)
    flange_adapter = cq.Workplane("YZ").circle(0.022).extrude(0.014).translate((0.072, 0.0, 0.0))
    flange_disk = cq.Workplane("YZ").circle(0.040).extrude(0.010).translate((0.086, 0.0, 0.0))
    pilot_ring = cq.Workplane("YZ").circle(0.017).extrude(0.008).translate((0.096, 0.0, 0.0))

    flange_bore = cq.Workplane("YZ").circle(0.010).extrude(0.024).translate((0.086, 0.0, 0.0))
    bolt_cuts = (
        cq.Workplane("YZ")
        .pushPoints([(0.022, 0.0), (-0.022, 0.0), (0.0, 0.022), (0.0, -0.022)])
        .circle(0.0032)
        .extrude(0.024)
        .translate((0.086, 0.0, 0.0))
    )

    flange = flange_adapter.union(flange_disk).union(pilot_ring).cut(flange_bore).cut(bolt_cuts)
    return body_profile.union(wrist_collar).union(flange)


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((lo + hi) / 2.0 for lo, hi in zip(mins, maxs))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="yaw_shoulder_wristed_arm")

    model.material("pedestal_graphite", rgba=(0.20, 0.22, 0.24, 1.0))
    model.material("arm_paint", rgba=(0.83, 0.84, 0.86, 1.0))
    model.material("tool_steel", rgba=(0.62, 0.65, 0.69, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        mesh_from_cadquery(_pedestal_shape(), "pedestal_shell"),
        material="pedestal_graphite",
        name="pedestal_shell",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        mesh_from_cadquery(_upper_arm_shape(), "upper_arm_shell"),
        material="arm_paint",
        name="upper_arm_shell",
    )

    forearm = model.part("forearm")
    forearm.visual(
        mesh_from_cadquery(_forearm_shape(), "forearm_shell"),
        material="arm_paint",
        name="forearm_shell",
    )

    wrist_link = model.part("wrist_link")
    wrist_link.visual(
        mesh_from_cadquery(_wrist_shape(), "wrist_shell"),
        material="tool_steel",
        name="wrist_shell",
    )

    model.articulation(
        "pedestal_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, PEDESTAL_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=1.2, lower=-3.0, upper=3.0),
    )
    model.articulation(
        "upper_arm_to_forearm",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, UPPER_ARM_CENTER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.4, lower=-1.35, upper=1.65),
    )
    model.articulation(
        "forearm_to_wrist",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_link,
        origin=Origin(xyz=(FOREARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=2.0, lower=-1.7, upper=1.7),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist_link = object_model.get_part("wrist_link")

    shoulder = object_model.get_articulation("pedestal_to_upper_arm")
    elbow = object_model.get_articulation("upper_arm_to_forearm")
    wrist = object_model.get_articulation("forearm_to_wrist")

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

    ctx.expect_contact(upper_arm, pedestal, name="shoulder_bearing_contact")
    ctx.expect_contact(forearm, upper_arm, name="elbow_bearing_contact")
    ctx.expect_contact(wrist_link, forearm, name="wrist_bearing_contact")

    rest_upper_center = _aabb_center(ctx.part_world_aabb(upper_arm))
    with ctx.pose({shoulder: pi / 2.0}):
        yawed_upper_center = _aabb_center(ctx.part_world_aabb(upper_arm))
    ctx.check(
        "shoulder_is_true_yaw",
        rest_upper_center is not None
        and yawed_upper_center is not None
        and abs(yawed_upper_center[0]) < 0.06
        and yawed_upper_center[1] > rest_upper_center[0] * 0.75,
        details=f"rest_center={rest_upper_center}, yawed_center={yawed_upper_center}",
    )

    rest_forearm_center = _aabb_center(ctx.part_world_aabb(forearm))
    with ctx.pose({elbow: 1.0}):
        lifted_forearm_center = _aabb_center(ctx.part_world_aabb(forearm))
    ctx.check(
        "elbow_lifts_forearm",
        rest_forearm_center is not None
        and lifted_forearm_center is not None
        and lifted_forearm_center[2] > rest_forearm_center[2] + 0.08,
        details=f"rest_center={rest_forearm_center}, lifted_center={lifted_forearm_center}",
    )

    with ctx.pose({elbow: 0.65, wrist: 0.0}):
        neutral_wrist_center = _aabb_center(ctx.part_world_aabb(wrist_link))
    with ctx.pose({elbow: 0.65, wrist: 0.9}):
        raised_wrist_center = _aabb_center(ctx.part_world_aabb(wrist_link))
    ctx.check(
        "wrist_pitches_tool_link",
        neutral_wrist_center is not None
        and raised_wrist_center is not None
        and (
            raised_wrist_center[2] > neutral_wrist_center[2] + 0.01
            or raised_wrist_center[0] < neutral_wrist_center[0] - 0.01
        ),
        details=f"neutral_center={neutral_wrist_center}, raised_center={raised_wrist_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
