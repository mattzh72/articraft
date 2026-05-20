from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

BASE_OUTER_R = 0.068
BASE_INNER_R = 0.040
BASE_BODY_H = 0.022
BASE_TRACK_R = 0.054
BASE_TRACK_INNER_R = 0.031
BASE_TRACK_H = 0.006
YAW_Z = BASE_BODY_H + BASE_TRACK_H

YOKE_TURNTABLE_R = 0.053
YOKE_TURNTABLE_H = 0.010
YOKE_PITCH_AXIS_Z = 0.082
YOKE_ARM_T = 0.010
YOKE_ARM_CENTER_Y = 0.040
YOKE_ARM_HOLE_R = 0.0072
YOKE_ARM_X = 0.014
YOKE_ARM_Z = 0.040
YOKE_ARM_Z_CENTER = 0.090
YOKE_STRUT_X = 0.014
YOKE_STRUT_Y = 0.010
YOKE_STRUT_Z = 0.050
YOKE_STRUT_Z_CENTER = 0.055
YOKE_STRUT_CENTER_Y = 0.024
YOKE_BRIDGE_X = 0.016
YOKE_BRIDGE_Y = 0.080
YOKE_BRIDGE_Z = 0.012
YOKE_BRIDGE_Z_CENTER = 0.110
YOKE_BODY_Y = (2.0 * YOKE_ARM_CENTER_Y) + YOKE_ARM_T

HOUSING_BODY_R = 0.016
HOUSING_BORE_R = 0.0102
HOUSING_BODY_L = 0.026
HOUSING_TRUNNION_R = 0.0072
HOUSING_TRUNNION_TIP_Y = YOKE_ARM_CENTER_Y - (YOKE_ARM_T / 2.0)
HOUSING_TRUNNION_L = HOUSING_TRUNNION_TIP_Y - HOUSING_BODY_R
HOUSING_FACE_RING_R = 0.019
HOUSING_FACE_RING_T = 0.002

SPINDLE_SHAFT_R = 0.0092
SPINDLE_SHAFT_X0 = -(HOUSING_BODY_L / 2.0) - 0.002
SPINDLE_SHAFT_L = 0.063
SPINDLE_RETAIN_R = 0.014
SPINDLE_RETAIN_T = 0.002
TOOL_FLANGE_R = 0.018
TOOL_FLANGE_T = 0.006
TOOL_FLANGE_X0 = 0.045


def _make_ring_mesh(outer_r: float, inner_r: float, height: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(outer_r).circle(inner_r).extrude(height)


def _make_yoke_body() -> cq.Workplane:
    turntable = cq.Workplane("XY").circle(YOKE_TURNTABLE_R).extrude(YOKE_TURNTABLE_H)
    turret = (
        cq.Workplane("XY")
        .circle(0.028)
        .extrude(0.022)
        .faces(">Z")
        .workplane()
        .circle(0.023)
        .extrude(0.008)
        .translate((0.0, 0.0, 0.010))
    )
    arm_blank = cq.Workplane("XY").box(YOKE_ARM_X, YOKE_ARM_T, YOKE_ARM_Z)
    pivot_hole = (
        cq.Workplane("XZ")
        .moveTo(0.0, YOKE_PITCH_AXIS_Z)
        .circle(YOKE_ARM_HOLE_R)
        .extrude(YOKE_ARM_T + 0.004, both=True)
    )
    left_arm = (
        arm_blank
        .translate((0.0, YOKE_ARM_CENTER_Y, YOKE_ARM_Z_CENTER))
        .cut(pivot_hole.translate((0.0, YOKE_ARM_CENTER_Y, 0.0)))
    )
    right_arm = (
        arm_blank
        .translate((0.0, -YOKE_ARM_CENTER_Y, YOKE_ARM_Z_CENTER))
        .cut(pivot_hole.translate((0.0, -YOKE_ARM_CENTER_Y, 0.0)))
    )
    left_strut = cq.Workplane("XY").box(YOKE_STRUT_X, YOKE_STRUT_Y, YOKE_STRUT_Z).translate((0.0, YOKE_STRUT_CENTER_Y, YOKE_STRUT_Z_CENTER))
    right_strut = cq.Workplane("XY").box(YOKE_STRUT_X, YOKE_STRUT_Y, YOKE_STRUT_Z).translate((0.0, -YOKE_STRUT_CENTER_Y, YOKE_STRUT_Z_CENTER))
    bridge = cq.Workplane("XY").box(YOKE_BRIDGE_X, YOKE_BRIDGE_Y, YOKE_BRIDGE_Z).translate((-0.004, 0.0, YOKE_BRIDGE_Z_CENTER))
    return turntable.union(turret).union(left_strut).union(right_strut).union(left_arm).union(right_arm).union(bridge)


def _make_housing_shell() -> cq.Workplane:
    body = cq.Workplane("YZ").circle(HOUSING_BODY_R).extrude(HOUSING_BODY_L / 2.0, both=True)
    face_ring = (
        cq.Workplane("YZ")
        .circle(HOUSING_FACE_RING_R)
        .circle(HOUSING_BORE_R)
        .extrude(HOUSING_FACE_RING_T)
        .translate((HOUSING_BODY_L / 2.0, 0.0, 0.0))
    )
    bore = cq.Workplane("YZ").circle(HOUSING_BORE_R).extrude((HOUSING_BODY_L / 2.0) + HOUSING_FACE_RING_T + 0.004, both=True)
    positive_trunnion = cq.Workplane("XY").cylinder(HOUSING_TRUNNION_L, HOUSING_TRUNNION_R).rotate((0, 0, 0), (1, 0, 0), 90).translate((0.0, HOUSING_BODY_R + (HOUSING_TRUNNION_L / 2.0), 0.0))
    negative_trunnion = cq.Workplane("XY").cylinder(HOUSING_TRUNNION_L, HOUSING_TRUNNION_R).rotate((0, 0, 0), (1, 0, 0), 90).translate((0.0, -(HOUSING_BODY_R + (HOUSING_TRUNNION_L / 2.0)), 0.0))
    return body.union(face_ring).cut(bore).union(positive_trunnion).union(negative_trunnion)


def _make_spindle_core() -> cq.Workplane:
    shaft = cq.Workplane("YZ").circle(SPINDLE_SHAFT_R).extrude(SPINDLE_SHAFT_L).translate((SPINDLE_SHAFT_X0, 0.0, 0.0))
    rear_retain = (
        cq.Workplane("YZ")
        .circle(SPINDLE_RETAIN_R)
        .extrude(SPINDLE_RETAIN_T)
        .translate((SPINDLE_SHAFT_X0, 0.0, 0.0))
    )
    front_retain = (
        cq.Workplane("YZ")
        .circle(SPINDLE_RETAIN_R)
        .extrude(SPINDLE_RETAIN_T)
        .translate((HOUSING_BODY_L / 2.0, 0.0, 0.0))
    )
    nose = cq.Workplane("YZ").circle(SPINDLE_SHAFT_R + 0.0015).extrude(TOOL_FLANGE_X0 - (HOUSING_BODY_L / 2.0) - SPINDLE_RETAIN_T).translate((HOUSING_BODY_L / 2.0 + SPINDLE_RETAIN_T, 0.0, 0.0))
    return shaft.union(rear_retain).union(front_retain).union(nose)


def _make_tool_flange() -> cq.Workplane:
    flange = cq.Workplane("YZ").circle(TOOL_FLANGE_R).extrude(TOOL_FLANGE_T).translate((TOOL_FLANGE_X0, 0.0, 0.0))
    bore = cq.Workplane("YZ").circle(0.005).extrude(TOOL_FLANGE_T).translate((TOOL_FLANGE_X0, 0.0, 0.0))
    bolt_holes = (
        cq.Workplane("YZ")
        .pushPoints([(0.010, 0.0), (-0.010, 0.0), (0.0, 0.010), (0.0, -0.010)])
        .circle(0.0022)
        .extrude(TOOL_FLANGE_T)
        .translate((TOOL_FLANGE_X0, 0.0, 0.0))
    )
    return flange.cut(bore).cut(bolt_holes)


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="yaw_pitch_roll_wrist", assets=ASSETS)

    steel = model.material("steel", rgba=(0.56, 0.58, 0.61, 1.0))
    dark = model.material("dark_paint", rgba=(0.18, 0.19, 0.22, 1.0))
    orange = model.material("safety_orange", rgba=(0.90, 0.42, 0.12, 1.0))
    black = model.material("blackened_steel", rgba=(0.12, 0.12, 0.13, 1.0))

    base_ring = model.part("base_ring")
    base_ring.visual(
        mesh_from_cadquery(
            _make_ring_mesh(BASE_OUTER_R, BASE_INNER_R, BASE_BODY_H),
            "base_ring_body.obj",
            assets=ASSETS,
        ),
        material=dark,
        name="ring_body",
    )
    base_ring.visual(
        mesh_from_cadquery(
            _make_ring_mesh(BASE_TRACK_R, BASE_TRACK_INNER_R, BASE_TRACK_H).translate((0.0, 0.0, BASE_BODY_H)),
            "base_slew_track.obj",
            assets=ASSETS,
        ),
        material=steel,
        name="slew_track",
    )
    base_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=BASE_OUTER_R, length=YAW_Z),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, YAW_Z / 2.0)),
    )

    pitch_yoke = model.part("pitch_yoke")
    pitch_yoke.visual(
        mesh_from_cadquery(_make_yoke_body(), "pitch_yoke_body.obj", assets=ASSETS),
        material=orange,
        name="yoke_body",
    )
    pitch_yoke.inertial = Inertial.from_geometry(
        Box((0.106, YOKE_BODY_Y, 0.112)),
        mass=0.95,
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
    )

    roll_housing = model.part("roll_housing")
    roll_housing.visual(
        mesh_from_cadquery(_make_housing_shell(), "roll_housing.obj", assets=ASSETS),
        material=steel,
        name="housing_shell",
    )
    roll_housing.inertial = Inertial.from_geometry(
        Box((0.046, (2.0 * HOUSING_TRUNNION_TIP_Y), 0.046)),
        mass=0.45,
        origin=Origin(),
    )

    roll_spindle = model.part("roll_spindle")
    roll_spindle.visual(
        mesh_from_cadquery(_make_spindle_core(), "roll_spindle_core.obj", assets=ASSETS),
        material=black,
        name="spindle_core",
    )
    roll_spindle.visual(
        mesh_from_cadquery(_make_tool_flange(), "tool_flange.obj", assets=ASSETS),
        material=steel,
        name="tool_flange",
    )
    roll_spindle.visual(
        Box((0.006, 0.010, 0.004)),
        origin=Origin(xyz=(TOOL_FLANGE_X0 + TOOL_FLANGE_T / 2.0, 0.0, TOOL_FLANGE_R - 0.001)),
        material=orange,
        name="index_tab",
    )
    roll_spindle.inertial = Inertial.from_geometry(
        Box((0.100, 0.036, 0.040)),
        mass=0.24,
        origin=Origin(xyz=(0.024, 0.0, 0.0)),
    )

    model.articulation(
        "yaw",
        ArticulationType.REVOLUTE,
        parent=base_ring,
        child=pitch_yoke,
        origin=Origin(xyz=(0.0, 0.0, YAW_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=-math.radians(150.0),
            upper=math.radians(150.0),
        ),
    )
    model.articulation(
        "pitch",
        ArticulationType.REVOLUTE,
        parent=pitch_yoke,
        child=roll_housing,
        origin=Origin(xyz=(0.0, 0.0, YOKE_PITCH_AXIS_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=-math.radians(60.0),
            upper=math.radians(60.0),
        ),
    )
    model.articulation(
        "roll",
        ArticulationType.REVOLUTE,
        parent=roll_housing,
        child=roll_spindle,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base_ring = object_model.get_part("base_ring")
    pitch_yoke = object_model.get_part("pitch_yoke")
    roll_housing = object_model.get_part("roll_housing")
    roll_spindle = object_model.get_part("roll_spindle")
    yaw = object_model.get_articulation("yaw")
    pitch = object_model.get_articulation("pitch")
    roll = object_model.get_articulation("roll")

    base_track = base_ring.get_visual("slew_track")
    yoke_body = pitch_yoke.get_visual("yoke_body")
    housing_shell = roll_housing.get_visual("housing_shell")
    spindle_flange = roll_spindle.get_visual("tool_flange")
    spindle_tab = roll_spindle.get_visual("index_tab")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    ctx.allow_overlap(
        pitch_yoke,
        roll_housing,
        reason="Pitch trunnions seat through the yoke side holes with a tight bearing-style fit.",
    )
    ctx.allow_overlap(
        roll_housing,
        roll_spindle,
        reason="The roll spindle is intentionally nested inside the housing bore and retaining features.",
    )

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
        "part presence",
        all(part is not None for part in (base_ring, pitch_yoke, roll_housing, roll_spindle)),
        "Expected base_ring, pitch_yoke, roll_housing, and roll_spindle.",
    )
    ctx.check(
        "articulation presence",
        all(joint is not None for joint in (yaw, pitch, roll)),
        "Expected yaw, pitch, and roll articulations.",
    )

    ctx.check(
        "yaw articulation metadata",
        yaw.articulation_type == ArticulationType.REVOLUTE
        and tuple(yaw.axis) == (0.0, 0.0, 1.0)
        and abs(yaw.motion_limits.lower + math.radians(150.0)) < 1e-6
        and abs(yaw.motion_limits.upper - math.radians(150.0)) < 1e-6,
        "Yaw should revolve about +Z with about ±150° travel.",
    )
    ctx.check(
        "pitch articulation metadata",
        pitch.articulation_type == ArticulationType.REVOLUTE
        and tuple(pitch.axis) == (0.0, 1.0, 0.0)
        and abs(pitch.motion_limits.lower + math.radians(60.0)) < 1e-6
        and abs(pitch.motion_limits.upper - math.radians(60.0)) < 1e-6,
        "Pitch should revolve about +Y with about ±60° travel.",
    )
    ctx.check(
        "roll articulation metadata",
        roll.articulation_type == ArticulationType.REVOLUTE
        and tuple(roll.axis) == (1.0, 0.0, 0.0)
        and abs(roll.motion_limits.lower + math.pi) < 1e-6
        and abs(roll.motion_limits.upper - math.pi) < 1e-6,
        "Roll should revolve about +X with about ±180° travel.",
    )

    ctx.expect_contact(
        base_ring,
        pitch_yoke,
        elem_a=base_track,
        elem_b=yoke_body,
        contact_tol=8e-4,
        name="base track supports yoke body",
    )
    ctx.expect_gap(
        pitch_yoke,
        base_ring,
        axis="z",
        positive_elem=yoke_body,
        negative_elem=base_track,
        max_penetration=1e-6,
        max_gap=8e-4,
        name="yoke base sits on slew track",
    )
    ctx.expect_overlap(
        pitch_yoke,
        base_ring,
        axes="xy",
        elem_a=yoke_body,
        elem_b=base_track,
        min_overlap=0.070,
        name="yoke overlaps slew track footprint",
    )
    ctx.expect_contact(
        pitch_yoke,
        roll_housing,
        contact_tol=8e-4,
        name="pitch yoke captures roll housing",
    )
    ctx.expect_origin_distance(
        roll_housing,
        roll_spindle,
        axes="yz",
        max_dist=1e-6,
        name="roll spindle stays centered in housing axis",
    )
    ctx.expect_within(
        roll_spindle,
        roll_housing,
        axes="yz",
        margin=0.003,
        name="roll spindle remains within housing envelope",
    )
    ctx.expect_gap(
        roll_spindle,
        base_ring,
        axis="z",
        min_gap=0.020,
        name="tool side clears base ring at rest",
    )

    rest_flange_center = _aabb_center(ctx.part_element_world_aabb(roll_spindle, elem=spindle_flange))
    rest_tab_center = _aabb_center(ctx.part_element_world_aabb(roll_spindle, elem=spindle_tab))

    with ctx.pose({yaw: math.radians(90.0)}):
        yaw_flange_center = _aabb_center(ctx.part_element_world_aabb(roll_spindle, elem=spindle_flange))
        yaw_ok = (
            rest_flange_center is not None
            and yaw_flange_center is not None
            and abs(math.hypot(rest_flange_center[0], rest_flange_center[1]) - math.hypot(yaw_flange_center[0], yaw_flange_center[1])) < 0.004
            and abs(yaw_flange_center[0]) < 0.006
            and abs(yaw_flange_center[1]) > 0.040
        )
        ctx.check(
            "yaw swings spindle around vertical axis",
            yaw_ok,
            f"Rest flange center={rest_flange_center}, yawed flange center={yaw_flange_center}",
        )
        ctx.expect_contact(
            base_ring,
            pitch_yoke,
            elem_a=base_track,
            elem_b=yoke_body,
            contact_tol=8e-4,
            name="yaw pose preserves yoke support",
        )

    with ctx.pose({pitch: -math.radians(45.0)}):
        pitch_flange_center = _aabb_center(ctx.part_element_world_aabb(roll_spindle, elem=spindle_flange))
        pitch_ok = (
            rest_flange_center is not None
            and pitch_flange_center is not None
            and pitch_flange_center[2] > rest_flange_center[2] + 0.025
            and pitch_flange_center[0] < rest_flange_center[0] - 0.010
        )
        ctx.check(
            "pitch raises tool flange",
            pitch_ok,
            f"Rest flange center={rest_flange_center}, pitched flange center={pitch_flange_center}",
        )
        ctx.expect_contact(
            pitch_yoke,
            roll_housing,
            contact_tol=8e-4,
            name="pitch pose preserves trunnion support",
        )

    with ctx.pose({roll: math.radians(90.0)}):
        rolled_tab_center = _aabb_center(ctx.part_element_world_aabb(roll_spindle, elem=spindle_tab))
        roll_ok = (
            rest_tab_center is not None
            and rolled_tab_center is not None
            and rest_tab_center[2] > rolled_tab_center[2] + 0.012
            and abs(rolled_tab_center[1]) > 0.012
        )
        ctx.check(
            "roll spins the index tab around tool axis",
            roll_ok,
            f"Rest tab center={rest_tab_center}, rolled tab center={rolled_tab_center}",
        )
        ctx.expect_origin_distance(
            roll_housing,
            roll_spindle,
            axes="yz",
            max_dist=1e-6,
            name="roll pose preserves spindle centering",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
