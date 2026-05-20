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


BASE_RADIUS = 0.14
BASE_HEIGHT = 0.24

LOWER_SLEEVE_HEIGHT = 0.58
LOWER_SOCKET_HEIGHT = 0.055
LOWER_OUTER_RADIUS = 0.051
LOWER_INNER_RADIUS = 0.041
LOWER_SOCKET_RADIUS = 0.060
LOWER_TOP_LIP_RADIUS = 0.055
LOWER_TOP_LIP_HEIGHT = 0.020

MIDDLE_INSERT = 0.18
MIDDLE_EXPOSED = 0.54
MIDDLE_LENGTH = MIDDLE_INSERT + MIDDLE_EXPOSED
MIDDLE_OUTER_RADIUS = 0.039
MIDDLE_INNER_RADIUS = 0.034
MIDDLE_STOP_RADIUS = 0.047
MIDDLE_STOP_HEIGHT = 0.028
MIDDLE_TOP_LIP_RADIUS = 0.041
MIDDLE_TOP_LIP_INNER = 0.033
MIDDLE_TOP_LIP_HEIGHT = 0.024
MIDDLE_GUIDE_LENGTH = 0.40
MIDDLE_GUIDE_RADIUS = 0.012
MIDDLE_GUIDE_PLUG_HEIGHT = 0.014

UPPER_INSERT = 0.14
UPPER_EXPOSED = 0.42
UPPER_LENGTH = UPPER_INSERT + UPPER_EXPOSED
UPPER_OUTER_RADIUS = 0.031
UPPER_INNER_RADIUS = 0.027
UPPER_STOP_RADIUS = 0.037
UPPER_STOP_HEIGHT = 0.022
UPPER_TOP_RING_RADIUS = 0.036
UPPER_TOP_RING_INNER = 0.022
UPPER_TOP_RING_HEIGHT = 0.018
UPPER_SPINDLE_RADIUS = 0.024
UPPER_SPINDLE_HEIGHT = 0.018
UPPER_GUIDE_LENGTH = 0.34
UPPER_GUIDE_RADIUS = 0.010
UPPER_GUIDE_PLUG_HEIGHT = 0.012

PAN_COLLAR_RADIUS = 0.056
PAN_COLLAR_UPPER_RADIUS = 0.044
PAN_COLLAR_LOWER_HEIGHT = 0.038
PAN_COLLAR_UPPER_HEIGHT = 0.024
PAN_COLLAR_HEIGHT = PAN_COLLAR_LOWER_HEIGHT + PAN_COLLAR_UPPER_HEIGHT


def _ring(outer_radius: float, inner_radius: float, height: float, z0: float = 0.0) -> cq.Workplane:
    ring = cq.Workplane("XY").circle(outer_radius).extrude(height)
    if inner_radius > 0.0:
        ring = ring.cut(cq.Workplane("XY").circle(inner_radius).extrude(height))
    if z0:
        ring = ring.translate((0.0, 0.0, z0))
    return ring


def _cylinder(radius: float, height: float, z0: float = 0.0) -> cq.Workplane:
    solid = cq.Workplane("XY").circle(radius).extrude(height)
    if z0:
        solid = solid.translate((0.0, 0.0, z0))
    return solid


def make_base_can() -> cq.Workplane:
    body = _cylinder(BASE_RADIUS, 0.22)
    lid = _cylinder(0.132, 0.020, 0.22)
    foot_ring = _ring(BASE_RADIUS + 0.012, BASE_RADIUS - 0.014, 0.014)
    service_band = _ring(BASE_RADIUS + 0.005, BASE_RADIUS - 0.008, 0.012, 0.110)
    top_pad = _cylinder(0.066, 0.010, BASE_HEIGHT - 0.010)
    return body.union(lid).union(foot_ring).union(service_band).union(top_pad)


def make_lower_sleeve() -> cq.Workplane:
    socket = _ring(LOWER_SOCKET_RADIUS, LOWER_INNER_RADIUS, LOWER_SOCKET_HEIGHT)
    sleeve = _ring(
        LOWER_OUTER_RADIUS,
        LOWER_INNER_RADIUS,
        LOWER_SLEEVE_HEIGHT - LOWER_SOCKET_HEIGHT - LOWER_TOP_LIP_HEIGHT,
        LOWER_SOCKET_HEIGHT,
    )
    top_lip = _ring(
        LOWER_TOP_LIP_RADIUS,
        LOWER_INNER_RADIUS,
        LOWER_TOP_LIP_HEIGHT,
        LOWER_SLEEVE_HEIGHT - LOWER_TOP_LIP_HEIGHT,
    )
    clamp_band = _ring(LOWER_SOCKET_RADIUS + 0.004, LOWER_SOCKET_RADIUS - 0.003, 0.012, 0.038)
    return socket.union(sleeve).union(top_lip).union(clamp_band)


def make_middle_stage_shell() -> cq.Workplane:
    tube = _ring(MIDDLE_OUTER_RADIUS, MIDDLE_INNER_RADIUS, MIDDLE_EXPOSED)
    stop_collar = _ring(MIDDLE_STOP_RADIUS, MIDDLE_INNER_RADIUS, MIDDLE_STOP_HEIGHT, 0.0)
    top_lip = _ring(
        MIDDLE_TOP_LIP_RADIUS,
        MIDDLE_TOP_LIP_INNER,
        MIDDLE_TOP_LIP_HEIGHT,
        MIDDLE_EXPOSED - MIDDLE_TOP_LIP_HEIGHT,
    )
    return tube.union(stop_collar).union(top_lip)


def make_middle_stage_guide() -> cq.Workplane:
    guide = _cylinder(MIDDLE_GUIDE_RADIUS, MIDDLE_GUIDE_LENGTH + MIDDLE_GUIDE_PLUG_HEIGHT, -MIDDLE_GUIDE_LENGTH)
    plug = _cylinder(MIDDLE_INNER_RADIUS, MIDDLE_GUIDE_PLUG_HEIGHT, 0.0)
    return guide.union(plug)


def make_upper_stage_shell() -> cq.Workplane:
    tube = _ring(UPPER_OUTER_RADIUS, UPPER_INNER_RADIUS, UPPER_EXPOSED)
    stop_collar = _ring(UPPER_STOP_RADIUS, UPPER_INNER_RADIUS, UPPER_STOP_HEIGHT, 0.0)
    top_ring = _ring(
        UPPER_TOP_RING_RADIUS,
        UPPER_TOP_RING_INNER,
        UPPER_TOP_RING_HEIGHT,
        UPPER_EXPOSED - UPPER_TOP_RING_HEIGHT,
    )
    spindle = _cylinder(UPPER_SPINDLE_RADIUS, UPPER_SPINDLE_HEIGHT, UPPER_EXPOSED)
    return tube.union(stop_collar).union(top_ring).union(spindle)


def make_upper_stage_guide() -> cq.Workplane:
    guide = _cylinder(UPPER_GUIDE_RADIUS, UPPER_GUIDE_LENGTH + UPPER_GUIDE_PLUG_HEIGHT, -UPPER_GUIDE_LENGTH)
    plug = _cylinder(UPPER_INNER_RADIUS, UPPER_GUIDE_PLUG_HEIGHT, 0.0)
    return guide.union(plug)


def make_pan_collar() -> cq.Workplane:
    lower = _cylinder(PAN_COLLAR_RADIUS, PAN_COLLAR_LOWER_HEIGHT)
    upper = _cylinder(PAN_COLLAR_UPPER_RADIUS, PAN_COLLAR_UPPER_HEIGHT, PAN_COLLAR_LOWER_HEIGHT)
    return lower.union(upper)


def make_top_bracket() -> cq.Workplane:
    pad = _cylinder(0.032, 0.008)
    arm = cq.Workplane("XY").box(0.082, 0.022, 0.016).translate((0.041, 0.0, 0.016))
    deck = cq.Workplane("XY").box(0.040, 0.060, 0.008).translate((0.104, 0.0, 0.028))
    left_ear = cq.Workplane("XY").box(0.028, 0.008, 0.052).translate((0.104, 0.026, 0.058))
    right_ear = cq.Workplane("XY").box(0.028, 0.008, 0.052).translate((0.104, -0.026, 0.058))
    cross_pad = cq.Workplane("XY").box(0.012, 0.044, 0.006).translate((0.104, 0.0, 0.052))
    return pad.union(arm).union(deck).union(left_ear).union(right_ear).union(cross_pad)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rooftop_inspection_mast")

    base_gray = model.material("base_gray", color=(0.29, 0.31, 0.33))
    tube_silver = model.material("tube_silver", color=(0.77, 0.79, 0.80))
    collar_black = model.material("collar_black", color=(0.11, 0.11, 0.12))

    base_can = model.part("base_can")
    base_can.visual(
        mesh_from_cadquery(make_base_can(), "base_can"),
        origin=Origin(),
        material=base_gray,
        name="base_can_shell",
    )

    lower_sleeve = model.part("lower_sleeve")
    lower_sleeve.visual(
        mesh_from_cadquery(make_lower_sleeve(), "lower_sleeve"),
        origin=Origin(),
        material=base_gray,
        name="lower_sleeve_shell",
    )

    middle_stage = model.part("middle_stage")
    middle_stage.visual(
        mesh_from_cadquery(make_middle_stage_shell(), "middle_stage_shell"),
        origin=Origin(),
        material=tube_silver,
        name="middle_stage_shell",
    )
    middle_stage.visual(
        mesh_from_cadquery(make_middle_stage_guide(), "middle_stage_guide"),
        origin=Origin(),
        material=base_gray,
        name="middle_stage_guide",
    )

    upper_stage = model.part("upper_stage")
    upper_stage.visual(
        mesh_from_cadquery(make_upper_stage_shell(), "upper_stage_shell"),
        origin=Origin(),
        material=tube_silver,
        name="upper_stage_shell",
    )
    upper_stage.visual(
        mesh_from_cadquery(make_upper_stage_guide(), "upper_stage_guide"),
        origin=Origin(),
        material=base_gray,
        name="upper_stage_guide",
    )

    pan_collar = model.part("pan_collar")
    pan_collar.visual(
        mesh_from_cadquery(make_pan_collar(), "pan_collar"),
        origin=Origin(),
        material=collar_black,
        name="pan_collar_shell",
    )

    top_bracket = model.part("top_bracket")
    top_bracket.visual(
        mesh_from_cadquery(make_top_bracket(), "top_bracket"),
        origin=Origin(),
        material=collar_black,
        name="top_bracket_shell",
    )

    model.articulation(
        "base_to_lower_sleeve",
        ArticulationType.FIXED,
        parent=base_can,
        child=lower_sleeve,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT)),
    )
    model.articulation(
        "lower_to_middle_stage",
        ArticulationType.PRISMATIC,
        parent=lower_sleeve,
        child=middle_stage,
        origin=Origin(xyz=(0.0, 0.0, LOWER_SLEEVE_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.40, lower=0.0, upper=0.32),
    )
    model.articulation(
        "middle_to_upper_stage",
        ArticulationType.PRISMATIC,
        parent=middle_stage,
        child=upper_stage,
        origin=Origin(xyz=(0.0, 0.0, MIDDLE_EXPOSED)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.40, lower=0.0, upper=0.26),
    )
    model.articulation(
        "upper_to_pan_collar",
        ArticulationType.REVOLUTE,
        parent=upper_stage,
        child=pan_collar,
        origin=Origin(xyz=(0.0, 0.0, UPPER_EXPOSED + UPPER_SPINDLE_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=-2.5,
            upper=2.5,
        ),
    )
    model.articulation(
        "pan_collar_to_top_bracket",
        ArticulationType.FIXED,
        parent=pan_collar,
        child=top_bracket,
        origin=Origin(xyz=(0.0, 0.0, PAN_COLLAR_HEIGHT)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_can = object_model.get_part("base_can")
    lower_sleeve = object_model.get_part("lower_sleeve")
    middle_stage = object_model.get_part("middle_stage")
    upper_stage = object_model.get_part("upper_stage")
    pan_collar = object_model.get_part("pan_collar")
    top_bracket = object_model.get_part("top_bracket")

    lower_to_middle = object_model.get_articulation("lower_to_middle_stage")
    middle_to_upper = object_model.get_articulation("middle_to_upper_stage")
    upper_to_pan = object_model.get_articulation("upper_to_pan_collar")

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
        lower_sleeve,
        middle_stage,
        elem_a="lower_sleeve_shell",
        elem_b="middle_stage_guide",
        reason="Hidden guide spigot remains nested inside the lower receiver sleeve during mast extension.",
    )
    ctx.allow_overlap(
        middle_stage,
        upper_stage,
        elem_a="middle_stage_shell",
        elem_b="upper_stage_guide",
        reason="Hidden guide spigot remains nested inside the middle receiver sleeve during upper-stage extension.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    for part_name in (
        "base_can",
        "lower_sleeve",
        "middle_stage",
        "upper_stage",
        "pan_collar",
        "top_bracket",
    ):
        ctx.check(f"part_present_{part_name}", object_model.get_part(part_name) is not None)

    ctx.check(
        "mast_joint_types_match_prompt",
        lower_to_middle.articulation_type == ArticulationType.PRISMATIC
        and middle_to_upper.articulation_type == ArticulationType.PRISMATIC
        and upper_to_pan.articulation_type == ArticulationType.REVOLUTE,
        "Expected two prismatic mast stages and one revolute pan collar.",
    )
    ctx.check(
        "mast_joint_axes_are_vertical",
        tuple(lower_to_middle.axis) == (0.0, 0.0, 1.0)
        and tuple(middle_to_upper.axis) == (0.0, 0.0, 1.0)
        and tuple(upper_to_pan.axis) == (0.0, 0.0, 1.0),
        "All prompt-critical articulations should share the mast centerline axis.",
    )

    ctx.expect_contact(lower_sleeve, base_can, name="base_can_supports_lower_sleeve")
    ctx.expect_contact(
        middle_stage,
        lower_sleeve,
        elem_a="middle_stage_shell",
        elem_b="lower_sleeve_shell",
        name="middle_stage_seats_on_lower_sleeve",
    )
    ctx.expect_contact(
        upper_stage,
        middle_stage,
        elem_a="upper_stage_shell",
        elem_b="middle_stage_shell",
        name="upper_stage_seats_on_middle_stage",
    )
    ctx.expect_contact(pan_collar, upper_stage, name="pan_collar_sits_on_upper_stage")
    ctx.expect_contact(top_bracket, pan_collar, name="top_bracket_mounts_to_pan_collar")

    ctx.expect_overlap(middle_stage, lower_sleeve, axes="xy", min_overlap=0.070)
    ctx.expect_overlap(upper_stage, middle_stage, axes="xy", min_overlap=0.055)
    ctx.expect_overlap(pan_collar, upper_stage, axes="xy", min_overlap=0.045)
    ctx.expect_gap(
        top_bracket,
        upper_stage,
        axis="z",
        min_gap=0.055,
        name="top_bracket_reads_above_extension_stack",
    )

    middle_rest = ctx.part_world_position(middle_stage)
    with ctx.pose({lower_to_middle: 0.20}):
        middle_extended = ctx.part_world_position(middle_stage)
    ctx.check(
        "middle_stage_slides_along_mast_axis",
        middle_rest is not None
        and middle_extended is not None
        and abs(middle_extended[2] - middle_rest[2] - 0.20) < 1e-6
        and abs(middle_extended[0] - middle_rest[0]) < 1e-6
        and abs(middle_extended[1] - middle_rest[1]) < 1e-6,
        "Middle stage should extend only along +Z.",
    )

    upper_rest = ctx.part_world_position(upper_stage)
    with ctx.pose({middle_to_upper: 0.16}):
        upper_extended = ctx.part_world_position(upper_stage)
    ctx.check(
        "upper_stage_slides_along_mast_axis",
        upper_rest is not None
        and upper_extended is not None
        and abs(upper_extended[2] - upper_rest[2] - 0.16) < 1e-6
        and abs(upper_extended[0] - upper_rest[0]) < 1e-6
        and abs(upper_extended[1] - upper_rest[1]) < 1e-6,
        "Upper stage should extend only along +Z.",
    )

    def aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    bracket_aabb_rest = ctx.part_element_world_aabb(top_bracket, elem="top_bracket_shell")
    with ctx.pose({upper_to_pan: 1.2}):
        bracket_aabb_panned = ctx.part_element_world_aabb(top_bracket, elem="top_bracket_shell")

    bracket_center_rest = aabb_center(bracket_aabb_rest)
    bracket_center_panned = aabb_center(bracket_aabb_panned)
    ctx.check(
        "pan_collar_rotates_offset_bracket_about_centerline",
        bracket_center_rest is not None
        and bracket_center_panned is not None
        and math.hypot(
            bracket_center_panned[0] - bracket_center_rest[0],
            bracket_center_panned[1] - bracket_center_rest[1],
        )
        > 0.05
        and abs(bracket_center_panned[2] - bracket_center_rest[2]) < 0.01
        and abs(
            math.hypot(bracket_center_rest[0], bracket_center_rest[1])
            - math.hypot(bracket_center_panned[0], bracket_center_panned[1])
        )
        < 0.015,
        "The bracket should orbit in plan around the mast without climbing or dropping.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
