from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi, sqrt

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BRIDGE_WIDTH = 0.24
BRIDGE_DEPTH = 0.085
BRIDGE_THICKNESS = 0.03
BEAM_BOTTOM_Z = -BRIDGE_THICKNESS / 2.0

LOWER_CHEEK_THICKNESS = 0.02
LOWER_CHEEK_DEPTH = 0.055
LOWER_CHEEK_DROP = 0.18
LOWER_CHEEK_X = BRIDGE_WIDTH / 2.0 - LOWER_CHEEK_THICKNESS / 2.0
LOWER_AXIS_Z = -0.17
LOWER_AXIS_SPAN = 2.0 * (LOWER_CHEEK_X - LOWER_CHEEK_THICKNESS / 2.0)
LOWER_STAGE_BODY_SPAN = LOWER_AXIS_SPAN - 0.016
LOWER_AXLE_RADIUS = 0.014
LOWER_STAGE_SLEEVE_RADIUS = 0.022
LOWER_STAGE_DRUM_RADIUS = 0.032
LOWER_STAGE_DRUM_LENGTH = 0.072
LOWER_STAGE_DROP = 0.085

UPPER_AXIS_Y = 0.068
UPPER_CHEEK_THICKNESS = 0.015
UPPER_CHEEK_DEPTH = 0.034
UPPER_CHEEK_DROP = 0.085
UPPER_CHEEK_X = 0.055
UPPER_AXIS_Z = -0.09
UPPER_AXIS_SPAN = 2.0 * (UPPER_CHEEK_X - UPPER_CHEEK_THICKNESS / 2.0)
UPPER_STAGE_BODY_SPAN = UPPER_AXIS_SPAN - 0.012
UPPER_AXLE_RADIUS = 0.01
UPPER_STAGE_SLEEVE_RADIUS = 0.016
UPPER_STAGE_DRUM_RADIUS = 0.022
UPPER_STAGE_DRUM_LENGTH = 0.04
UPPER_STAGE_DROP = 0.048


def _x_cylinder(length: float, radius: float, center: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> cq.Workplane:
    cx, cy, cz = center
    return (
        cq.Workplane("YZ")
        .circle(radius)
        .extrude(length)
        .translate((cx - length / 2.0, cy, cz))
    )


def _centered_box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    sx, sy, sz = size
    return cq.Workplane("XY").box(sx, sy, sz).translate(center)


def _build_bridge_shape() -> cq.Workplane:
    bridge = _centered_box((BRIDGE_WIDTH, BRIDGE_DEPTH, BRIDGE_THICKNESS), (0.0, 0.0, 0.0))

    for sign in (-1.0, 1.0):
        bridge = bridge.union(
            _centered_box(
                (LOWER_CHEEK_THICKNESS, LOWER_CHEEK_DEPTH, LOWER_CHEEK_DROP),
                (sign * LOWER_CHEEK_X, 0.0, BEAM_BOTTOM_Z - LOWER_CHEEK_DROP / 2.0),
            )
        )

    for sign in (-1.0, 1.0):
        bridge = bridge.union(
            _centered_box(
                (UPPER_CHEEK_THICKNESS, UPPER_CHEEK_DEPTH, UPPER_CHEEK_DROP),
                (
                    sign * UPPER_CHEEK_X,
                    UPPER_AXIS_Y,
                    BEAM_BOTTOM_Z - UPPER_CHEEK_DROP / 2.0,
                ),
            )
        )

    bridge = bridge.union(
        _centered_box(
            (0.06, 0.05, 0.05),
            (0.0, UPPER_AXIS_Y * 0.55, -0.04),
        )
    )
    bridge = bridge.union(
        _centered_box(
            (0.14, 0.018, 0.018),
            (0.0, UPPER_AXIS_Y - 0.005, -0.03),
        )
    )

    return bridge


def _build_hanging_rotary_stage(
    span: float,
    axle_radius: float,
    sleeve_radius: float,
    drum_radius: float,
    drum_length: float,
    drum_drop: float,
    web_width: float,
    web_depth: float,
    tab_width: float,
    tab_depth: float,
    tab_height: float,
) -> cq.Workplane:
    stage = _x_cylinder(span, sleeve_radius)
    web_height = max(drum_drop - drum_radius - sleeve_radius, 0.012)
    stage = stage.union(
        _centered_box(
            (web_width, web_depth, web_height),
            (0.0, 0.0, -(sleeve_radius + web_height / 2.0)),
        )
    )
    stage = stage.union(_x_cylinder(drum_length, drum_radius, center=(0.0, 0.0, -drum_drop)))
    stage = stage.union(
        _centered_box(
            (tab_width, tab_depth, tab_height),
            (0.0, 0.0, -(drum_drop + drum_radius + tab_height / 2.0 - 0.004)),
        )
    )
    return stage


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
    (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
    return (
        0.5 * (min_x + max_x),
        0.5 * (min_y + max_y),
        0.5 * (min_z + max_z),
    )


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return sqrt(sum((av - bv) ** 2 for av, bv in zip(a, b)))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offset_rotary_bridge")

    model.material("bridge_paint", rgba=(0.20, 0.22, 0.25, 1.0))
    model.material("stage_metal", rgba=(0.72, 0.75, 0.79, 1.0))
    model.material("accent_dark", rgba=(0.14, 0.15, 0.17, 1.0))

    bridge = model.part("bridge_support")
    bridge.visual(
        Box((BRIDGE_WIDTH, BRIDGE_DEPTH, BRIDGE_THICKNESS)),
        material="bridge_paint",
        name="top_beam",
    )
    bridge.visual(
        Box((LOWER_CHEEK_THICKNESS, LOWER_CHEEK_DEPTH, LOWER_CHEEK_DROP)),
        origin=Origin(xyz=(-LOWER_CHEEK_X, 0.0, BEAM_BOTTOM_Z - LOWER_CHEEK_DROP / 2.0)),
        material="bridge_paint",
        name="lower_left_cheek",
    )
    bridge.visual(
        Box((LOWER_CHEEK_THICKNESS, LOWER_CHEEK_DEPTH, LOWER_CHEEK_DROP)),
        origin=Origin(xyz=(LOWER_CHEEK_X, 0.0, BEAM_BOTTOM_Z - LOWER_CHEEK_DROP / 2.0)),
        material="bridge_paint",
        name="lower_right_cheek",
    )
    bridge.visual(
        Box((0.14, 0.024, 0.03)),
        origin=Origin(xyz=(0.0, 0.047, -0.03)),
        material="bridge_paint",
        name="upper_mount_block",
    )
    bridge.visual(
        Box((UPPER_CHEEK_THICKNESS, UPPER_CHEEK_DEPTH, UPPER_CHEEK_DROP)),
        origin=Origin(
            xyz=(-UPPER_CHEEK_X, UPPER_AXIS_Y, BEAM_BOTTOM_Z - UPPER_CHEEK_DROP / 2.0)
        ),
        material="bridge_paint",
        name="upper_left_cheek",
    )
    bridge.visual(
        Box((UPPER_CHEEK_THICKNESS, UPPER_CHEEK_DEPTH, UPPER_CHEEK_DROP)),
        origin=Origin(
            xyz=(UPPER_CHEEK_X, UPPER_AXIS_Y, BEAM_BOTTOM_Z - UPPER_CHEEK_DROP / 2.0)
        ),
        material="bridge_paint",
        name="upper_right_cheek",
    )

    lower_stage = model.part("lower_stage")
    lower_web_height = LOWER_STAGE_DROP - LOWER_STAGE_DRUM_RADIUS - LOWER_STAGE_SLEEVE_RADIUS
    lower_stage.visual(
        Cylinder(radius=LOWER_STAGE_SLEEVE_RADIUS, length=LOWER_AXIS_SPAN),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material="stage_metal",
        name="lower_sleeve",
    )
    lower_stage.visual(
        Box((0.028, 0.016, lower_web_height)),
        origin=Origin(
            xyz=(0.0, 0.0, -(LOWER_STAGE_SLEEVE_RADIUS + lower_web_height / 2.0))
        ),
        material="stage_metal",
        name="lower_web",
    )
    lower_stage.visual(
        Cylinder(radius=LOWER_STAGE_DRUM_RADIUS, length=LOWER_STAGE_DRUM_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, -LOWER_STAGE_DROP), rpy=(0.0, pi / 2.0, 0.0)),
        material="stage_metal",
        name="lower_body",
    )
    lower_stage.visual(
        Box((0.022, 0.012, 0.028)),
        origin=Origin(
            xyz=(0.0, 0.0, -(LOWER_STAGE_DROP + LOWER_STAGE_DRUM_RADIUS + 0.014))
        ),
        material="accent_dark",
        name="lower_tab",
    )

    upper_stage = model.part("upper_stage")
    upper_web_height = UPPER_STAGE_DROP - UPPER_STAGE_DRUM_RADIUS - UPPER_STAGE_SLEEVE_RADIUS
    upper_stage.visual(
        Cylinder(radius=UPPER_STAGE_SLEEVE_RADIUS, length=UPPER_AXIS_SPAN),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material="stage_metal",
        name="upper_sleeve",
    )
    upper_stage.visual(
        Box((0.02, 0.012, upper_web_height)),
        origin=Origin(
            xyz=(0.0, 0.0, -(UPPER_STAGE_SLEEVE_RADIUS + upper_web_height / 2.0))
        ),
        material="stage_metal",
        name="upper_web",
    )
    upper_stage.visual(
        Cylinder(radius=UPPER_STAGE_DRUM_RADIUS, length=UPPER_STAGE_DRUM_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, -UPPER_STAGE_DROP), rpy=(0.0, pi / 2.0, 0.0)),
        material="stage_metal",
        name="upper_body",
    )
    upper_stage.visual(
        Box((0.014, 0.008, 0.018)),
        origin=Origin(
            xyz=(0.0, 0.0, -(UPPER_STAGE_DROP + UPPER_STAGE_DRUM_RADIUS + 0.009))
        ),
        material="accent_dark",
        name="upper_tab",
    )

    model.articulation(
        "bridge_to_lower_stage",
        ArticulationType.REVOLUTE,
        parent=bridge,
        child=lower_stage,
        origin=Origin(xyz=(0.0, 0.0, LOWER_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-2.35, upper=2.35, effort=12.0, velocity=2.0),
    )
    model.articulation(
        "bridge_to_upper_stage",
        ArticulationType.REVOLUTE,
        parent=bridge,
        child=upper_stage,
        origin=Origin(xyz=(0.0, UPPER_AXIS_Y, UPPER_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-2.1, upper=2.1, effort=8.0, velocity=2.4),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bridge = object_model.get_part("bridge_support")
    lower_stage = object_model.get_part("lower_stage")
    upper_stage = object_model.get_part("upper_stage")
    lower_joint = object_model.get_articulation("bridge_to_lower_stage")
    upper_joint = object_model.get_articulation("bridge_to_upper_stage")

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
        "expected_parts_present",
        all(part is not None for part in (bridge, lower_stage, upper_stage)),
        "Bridge support, lower stage, and upper stage should all be present.",
    )
    ctx.expect_contact(
        lower_stage,
        bridge,
        contact_tol=5e-4,
        name="lower_stage_supported_by_bridge",
    )
    ctx.expect_contact(
        upper_stage,
        bridge,
        contact_tol=5e-4,
        name="upper_stage_supported_by_bridge",
    )
    ctx.check(
        "parallel_offset_axes",
        lower_joint.axis == (1.0, 0.0, 0.0)
        and upper_joint.axis == (1.0, 0.0, 0.0)
        and abs(lower_joint.origin.xyz[0] - upper_joint.origin.xyz[0]) < 1e-9
        and upper_joint.origin.xyz[1] > lower_joint.origin.xyz[1] + 0.03
        and upper_joint.origin.xyz[2] > lower_joint.origin.xyz[2] + 0.06,
        "The two revolute stages should use parallel X-axis pivots, with the upper axis offset forward and above the lower axis.",
    )
    ctx.expect_origin_gap(
        upper_stage,
        lower_stage,
        axis="y",
        min_gap=0.035,
        max_gap=0.08,
        name="upper_stage_is_forward_offset",
    )
    ctx.expect_origin_gap(
        upper_stage,
        lower_stage,
        axis="z",
        min_gap=0.07,
        max_gap=0.1,
        name="upper_stage_is_above_lower_stage",
    )

    lower_rest = _aabb_center(ctx.part_element_world_aabb(lower_stage, elem="lower_body"))
    upper_rest = _aabb_center(ctx.part_element_world_aabb(upper_stage, elem="upper_body"))
    with ctx.pose(bridge_to_lower_stage=0.9, bridge_to_upper_stage=0.0):
        lower_moved = _aabb_center(ctx.part_element_world_aabb(lower_stage, elem="lower_body"))
        upper_still = _aabb_center(ctx.part_element_world_aabb(upper_stage, elem="upper_body"))
    with ctx.pose(bridge_to_lower_stage=0.0, bridge_to_upper_stage=-0.8):
        lower_still = _aabb_center(ctx.part_element_world_aabb(lower_stage, elem="lower_body"))
        upper_moved = _aabb_center(ctx.part_element_world_aabb(upper_stage, elem="upper_body"))

    ctx.check(
        "lower_joint_moves_only_lower_stage",
        _distance(lower_rest, lower_moved) > 0.04 and _distance(upper_rest, upper_still) < 0.001,
        "Driving the lower revolute joint should move the lower paddle while leaving the upper stage in place.",
    )
    ctx.check(
        "upper_joint_moves_only_upper_stage",
        _distance(upper_rest, upper_moved) > 0.02 and _distance(lower_rest, lower_still) < 0.001,
        "Driving the upper revolute joint should move the upper paddle while leaving the lower stage in place.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
