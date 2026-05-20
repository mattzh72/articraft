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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

MODEL_NAME = "rrp_manipulator"

BASE_LENGTH = 0.086
BASE_WIDTH = 0.054
BASE_THICKNESS = 0.010
BRACKET_CHEEK_LENGTH = 0.044
BRACKET_CHEEK_THICKNESS = 0.007
BRACKET_CHEEK_HEIGHT = 0.090
BRACKET_INNER_GAP = 0.022
BRACKET_BRACE_LENGTH = 0.018
BRACKET_BRACE_HEIGHT = 0.026
SHOULDER_PIVOT_Z = 0.090

SHOULDER_HUB_RADIUS = 0.010
SHOULDER_HUB_WIDTH = BRACKET_INNER_GAP
SHOULDER_ARM_LENGTH = 0.090
SHOULDER_ARM_WIDTH = 0.016
SHOULDER_ARM_HEIGHT = 0.012
SHOULDER_FORK_START = 0.078
SHOULDER_FORK_LENGTH = 0.024
SHOULDER_FORK_LUG_THICKNESS = 0.004
SHOULDER_FORK_INNER_GAP = 0.018
SHOULDER_FORK_HEIGHT = 0.024

ELBOW_HUB_RADIUS = 0.009
ELBOW_HUB_WIDTH = SHOULDER_FORK_INNER_GAP
ELBOW_ARM_LENGTH = 0.046
ELBOW_ARM_WIDTH = 0.014
ELBOW_ARM_HEIGHT = 0.011
SLIDER_SLEEVE_START = 0.042
SLIDER_SLEEVE_LENGTH = 0.118
SLIDER_SLEEVE_OUTER = 0.022
SLIDER_STAGE_SIZE = 0.016
SLIDER_STAGE_LENGTH = 0.115
SLIDER_NOSE_RADIUS = 0.0045
SLIDER_NOSE_LENGTH = 0.020
SLIDER_STROKE = 0.060

SHOULDER_LIMIT = math.radians(120.0)
ELBOW_LIMIT = math.radians(90.0)


def _box_x(length: float, width: float, height: float, x: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(length, width, height, centered=(False, True, True))
        .translate((x, 0.0, 0.0))
    )


def _box_z(length: float, width: float, height: float, z: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(length, width, height, centered=(True, True, False))
        .translate((0.0, 0.0, z))
    )


def _cylinder_y(radius: float, length: float, x: float = 0.0, z: float = 0.0) -> cq.Workplane:
    return cq.Workplane("XZ").center(x, z).circle(radius).extrude(length / 2.0, both=True)


def _cylinder_x(radius: float, length: float, x: float = 0.0) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length).translate((x, 0.0, 0.0))


def _make_base_bracket() -> cq.Workplane:
    cheek_offset = BRACKET_INNER_GAP / 2.0 + BRACKET_CHEEK_THICKNESS / 2.0

    base_plate = _box_z(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS, 0.0)
    cheek = _box_z(
        BRACKET_CHEEK_LENGTH,
        BRACKET_CHEEK_THICKNESS,
        BRACKET_CHEEK_HEIGHT,
        BASE_THICKNESS,
    )
    left_cheek = cheek.translate((0.0, cheek_offset, 0.0))
    right_cheek = cheek.translate((0.0, -cheek_offset, 0.0))
    rear_brace = (
        _box_z(BRACKET_BRACE_LENGTH, BRACKET_INNER_GAP, BRACKET_BRACE_HEIGHT, BASE_THICKNESS)
        .translate((-0.022, 0.0, 0.0))
    )

    return base_plate.union(left_cheek).union(right_cheek).union(rear_brace)


def _make_shoulder_link() -> cq.Workplane:
    hub = _cylinder_y(SHOULDER_HUB_RADIUS, SHOULDER_HUB_WIDTH)
    arm = _box_x(SHOULDER_ARM_LENGTH, SHOULDER_ARM_WIDTH, SHOULDER_ARM_HEIGHT, 0.0)

    lug_offset = SHOULDER_FORK_INNER_GAP / 2.0 + SHOULDER_FORK_LUG_THICKNESS / 2.0
    lug = _box_x(
        SHOULDER_FORK_LENGTH,
        SHOULDER_FORK_LUG_THICKNESS,
        SHOULDER_FORK_HEIGHT,
        SHOULDER_FORK_START,
    )
    left_lug = lug.translate((0.0, lug_offset, 0.0))
    right_lug = lug.translate((0.0, -lug_offset, 0.0))
    web = _box_x(0.018, SHOULDER_ARM_WIDTH, SHOULDER_FORK_HEIGHT * 0.85, SHOULDER_FORK_START - 0.012)

    return hub.union(arm).union(left_lug).union(right_lug).union(web)


def _make_elbow_link() -> cq.Workplane:
    hub = _cylinder_y(ELBOW_HUB_RADIUS, ELBOW_HUB_WIDTH)
    arm = _box_x(ELBOW_ARM_LENGTH, ELBOW_ARM_WIDTH, ELBOW_ARM_HEIGHT, 0.0)
    sleeve_outer = _box_x(
        SLIDER_SLEEVE_LENGTH,
        SLIDER_SLEEVE_OUTER,
        SLIDER_SLEEVE_OUTER,
        SLIDER_SLEEVE_START,
    )
    sleeve_inner = _box_x(
        SLIDER_SLEEVE_LENGTH + 0.004,
        SLIDER_STAGE_SIZE,
        SLIDER_STAGE_SIZE,
        SLIDER_SLEEVE_START - 0.002,
    )
    sleeve = sleeve_outer.cut(sleeve_inner)

    return hub.union(arm).union(sleeve)


def _make_tip_stage() -> cq.Workplane:
    rod = _box_x(SLIDER_STAGE_LENGTH, SLIDER_STAGE_SIZE, SLIDER_STAGE_SIZE, 0.0)
    nose = _cylinder_x(SLIDER_NOSE_RADIUS, SLIDER_NOSE_LENGTH, x=SLIDER_STAGE_LENGTH)
    return rod.union(nose)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name=MODEL_NAME, assets=ASSETS)

    dark_paint = model.material("dark_paint", rgba=(0.18, 0.20, 0.22, 1.0))
    light_paint = model.material("light_paint", rgba=(0.62, 0.66, 0.70, 1.0))
    steel = model.material("steel", rgba=(0.53, 0.56, 0.60, 1.0))
    black_oxide = model.material("black_oxide", rgba=(0.14, 0.15, 0.17, 1.0))

    base = model.part("base_bracket")
    base.visual(
        mesh_from_cadquery(_make_base_bracket(), "base_bracket.obj", assets=ASSETS),
        name="bracket_shell",
        material=dark_paint,
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS + BRACKET_CHEEK_HEIGHT)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, (BASE_THICKNESS + BRACKET_CHEEK_HEIGHT) / 2.0)),
    )

    shoulder = model.part("shoulder_link")
    shoulder.visual(
        mesh_from_cadquery(_make_shoulder_link(), "shoulder_link.obj", assets=ASSETS),
        name="shoulder_shell",
        material=light_paint,
    )
    shoulder.inertial = Inertial.from_geometry(
        Box((SHOULDER_FORK_START + SHOULDER_FORK_LENGTH, 0.026, SHOULDER_FORK_HEIGHT)),
        mass=0.55,
        origin=Origin(xyz=((SHOULDER_FORK_START + SHOULDER_FORK_LENGTH) / 2.0, 0.0, 0.0)),
    )

    elbow = model.part("elbow_link")
    elbow.visual(
        mesh_from_cadquery(_make_elbow_link(), "elbow_link.obj", assets=ASSETS),
        name="elbow_shell",
        material=steel,
    )
    elbow.inertial = Inertial.from_geometry(
        Box((SLIDER_SLEEVE_START + SLIDER_SLEEVE_LENGTH, SLIDER_SLEEVE_OUTER, SLIDER_SLEEVE_OUTER)),
        mass=0.42,
        origin=Origin(xyz=((SLIDER_SLEEVE_START + SLIDER_SLEEVE_LENGTH) / 2.0, 0.0, 0.0)),
    )

    tip = model.part("tip_stage")
    tip.visual(
        mesh_from_cadquery(_make_tip_stage(), "tip_stage.obj", assets=ASSETS),
        name="tip_shell",
        material=black_oxide,
    )
    tip.inertial = Inertial.from_geometry(
        Box((SLIDER_STAGE_LENGTH + SLIDER_NOSE_LENGTH, SLIDER_STAGE_SIZE, SLIDER_STAGE_SIZE)),
        mass=0.12,
        origin=Origin(xyz=((SLIDER_STAGE_LENGTH + SLIDER_NOSE_LENGTH) / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_shoulder",
        ArticulationType.REVOLUTE,
        parent=base,
        child=shoulder,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.5,
            lower=-SHOULDER_LIMIT,
            upper=SHOULDER_LIMIT,
        ),
    )
    model.articulation(
        "shoulder_to_elbow",
        ArticulationType.REVOLUTE,
        parent=shoulder,
        child=elbow,
        origin=Origin(xyz=(SHOULDER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=3.0,
            lower=-ELBOW_LIMIT,
            upper=ELBOW_LIMIT,
        ),
    )
    model.articulation(
        "elbow_to_tip",
        ArticulationType.PRISMATIC,
        parent=elbow,
        child=tip,
        origin=Origin(xyz=(SLIDER_SLEEVE_START, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.12,
            lower=0.0,
            upper=SLIDER_STROKE,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base_bracket")
    shoulder = object_model.get_part("shoulder_link")
    elbow = object_model.get_part("elbow_link")
    tip = object_model.get_part("tip_stage")
    base_shell = base.get_visual("bracket_shell")
    shoulder_shell = shoulder.get_visual("shoulder_shell")
    elbow_shell = elbow.get_visual("elbow_shell")
    tip_shell = tip.get_visual("tip_shell")
    shoulder_joint = object_model.get_articulation("base_to_shoulder")
    elbow_joint = object_model.get_articulation("shoulder_to_elbow")
    tip_joint = object_model.get_articulation("elbow_to_tip")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

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
    ctx.allow_overlap(
        base,
        shoulder,
        elem_a=base_shell,
        elem_b=shoulder_shell,
        reason="shoulder hub is intentionally captured between bracket cheeks as a hinge seat",
    )
    ctx.allow_overlap(
        shoulder,
        elbow,
        elem_a=shoulder_shell,
        elem_b=elbow_shell,
        reason="elbow hub is intentionally nested between the shoulder fork lugs",
    )
    ctx.allow_overlap(
        elbow,
        tip,
        elem_a=elbow_shell,
        elem_b=tip_shell,
        reason="tip stage is intentionally enclosed by the elbow sleeve for its prismatic guide",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check("base_exists", base is not None, "base bracket should exist")
    ctx.check("shoulder_exists", shoulder is not None, "shoulder link should exist")
    ctx.check("elbow_exists", elbow is not None, "elbow link should exist")
    ctx.check("tip_exists", tip is not None, "tip stage should exist")

    ctx.expect_contact(shoulder, base, name="shoulder_hub_seated_in_base_bracket")
    ctx.expect_contact(elbow, shoulder, name="elbow_hub_seated_in_shoulder_fork")
    ctx.expect_contact(tip, elbow, name="tip_stage_guided_in_elbow_sleeve")

    ctx.expect_origin_distance(elbow, shoulder, axes="yz", max_dist=1e-6, name="elbow_joint_stays_in_motion_plane")
    ctx.expect_origin_gap(elbow, shoulder, axis="x", min_gap=0.089, max_gap=0.091, name="shoulder_link_reaches_elbow_joint")
    ctx.expect_origin_distance(tip, elbow, axes="yz", max_dist=1e-6, name="tip_stage_is_coaxial_with_elbow_sleeve")
    ctx.expect_origin_gap(
        tip,
        elbow,
        axis="x",
        min_gap=SLIDER_SLEEVE_START - 1e-6,
        max_gap=SLIDER_SLEEVE_START + 1e-6,
        name="tip_stage_origin_starts_at_sleeve_entry",
    )
    ctx.expect_origin_distance(tip, elbow, axes="yz", max_dist=1e-6, name="tip_stage_starts_centered_in_sleeve")

    ctx.check(
        "shoulder_joint_axis_and_limits",
        tuple(shoulder_joint.axis) == (0.0, 1.0, 0.0)
        and math.isclose(shoulder_joint.motion_limits.lower, -SHOULDER_LIMIT, abs_tol=1e-9)
        and math.isclose(shoulder_joint.motion_limits.upper, SHOULDER_LIMIT, abs_tol=1e-9),
        "shoulder joint should be a y-axis revolute with ±120 degree travel",
    )
    ctx.check(
        "elbow_joint_axis_and_limits",
        tuple(elbow_joint.axis) == (0.0, 1.0, 0.0)
        and math.isclose(elbow_joint.motion_limits.lower, -ELBOW_LIMIT, abs_tol=1e-9)
        and math.isclose(elbow_joint.motion_limits.upper, ELBOW_LIMIT, abs_tol=1e-9),
        "elbow joint should be a y-axis revolute with ±90 degree travel",
    )
    ctx.check(
        "tip_joint_axis_and_limits",
        tuple(tip_joint.axis) == (1.0, 0.0, 0.0)
        and math.isclose(tip_joint.motion_limits.lower, 0.0, abs_tol=1e-9)
        and math.isclose(tip_joint.motion_limits.upper, SLIDER_STROKE, abs_tol=1e-9),
        "tip stage should slide 60 mm along its own x axis",
    )

    bend_angle = math.radians(50.0)
    with ctx.pose({shoulder_joint: bend_angle}):
        elbow_pos = ctx.part_world_position(elbow)
        expected_elbow = (
            SHOULDER_ARM_LENGTH * math.cos(bend_angle),
            0.0,
            SHOULDER_PIVOT_Z - SHOULDER_ARM_LENGTH * math.sin(bend_angle),
        )
        ctx.check(
            "shoulder_revolute_bends_in_xz_plane",
            elbow_pos is not None
            and all(abs(a - b) <= 1e-4 for a, b in zip(elbow_pos, expected_elbow)),
            f"expected elbow origin near {expected_elbow}, got {elbow_pos}",
        )
        ctx.expect_contact(elbow, shoulder, name="elbow_remains_supported_when_shoulder_bends")

    shoulder_pose = 0.70
    elbow_pose = -0.35
    with ctx.pose({shoulder_joint: shoulder_pose, elbow_joint: elbow_pose, tip_joint: 0.0}):
        tip_pos_retracted = ctx.part_world_position(tip)
    with ctx.pose({shoulder_joint: shoulder_pose, elbow_joint: elbow_pose, tip_joint: SLIDER_STROKE}):
        elbow_pos_extended = ctx.part_world_position(elbow)
        tip_pos_extended = ctx.part_world_position(tip)
        ctx.expect_contact(tip, elbow, name="tip_stage_remains_guided_at_full_extension")

    total_angle = shoulder_pose + elbow_pose
    expected_delta = (
        SLIDER_STROKE * math.cos(total_angle),
        0.0,
        -SLIDER_STROKE * math.sin(total_angle),
    )
    actual_delta = (
        tip_pos_extended[0] - tip_pos_retracted[0],
        tip_pos_extended[1] - tip_pos_retracted[1],
        tip_pos_extended[2] - tip_pos_retracted[2],
    )
    expected_tip_offset = (
        (SLIDER_SLEEVE_START + SLIDER_STROKE) * math.cos(total_angle),
        0.0,
        -(SLIDER_SLEEVE_START + SLIDER_STROKE) * math.sin(total_angle),
    )
    actual_tip_offset = (
        tip_pos_extended[0] - elbow_pos_extended[0],
        tip_pos_extended[1] - elbow_pos_extended[1],
        tip_pos_extended[2] - elbow_pos_extended[2],
    )
    ctx.check(
        "tip_stage_slides_along_elbow_axis",
        all(abs(a - b) <= 1e-4 for a, b in zip(actual_delta, expected_delta)),
        f"expected extension delta {expected_delta}, got {actual_delta}",
    )
    ctx.check(
        "tip_stage_origin_matches_rotated_slider_axis_when_extended",
        all(abs(a - b) <= 1e-4 for a, b in zip(actual_tip_offset, expected_tip_offset)),
        f"expected tip offset {expected_tip_offset}, got {actual_tip_offset}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
