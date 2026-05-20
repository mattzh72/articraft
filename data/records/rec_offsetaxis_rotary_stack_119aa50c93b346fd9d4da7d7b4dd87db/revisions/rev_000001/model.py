from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import hypot, pi

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


BASE_LENGTH = 0.42
BASE_WIDTH = 0.28
BASE_THICKNESS = 0.036
PEDESTAL_RADIUS = 0.090
PEDESTAL_HEIGHT = 0.024

LOWER_PLATFORM_RADIUS = 0.145
LOWER_BEARING_RADIUS = 0.092
LOWER_BEARING_HEIGHT = 0.012
LOWER_PLATFORM_THICKNESS = 0.016

UPPER_AXIS_OFFSET = 0.186
UPPER_AXIS_HEIGHT = 0.159
UPPER_STAGE_RADIUS = 0.070
UPPER_COLLAR_RADIUS = 0.042


def _base_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(
        BASE_LENGTH,
        BASE_WIDTH,
        BASE_THICKNESS,
        centered=(True, True, False),
    )
    base = base.edges("|Z").fillet(0.018)
    base = (
        base.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .circle(PEDESTAL_RADIUS)
        .extrude(PEDESTAL_HEIGHT)
    )
    return base


def _lower_carrier_shape() -> cq.Workplane:
    carrier = cq.Workplane("XY").circle(LOWER_BEARING_RADIUS).extrude(LOWER_BEARING_HEIGHT)
    carrier = (
        carrier.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .circle(LOWER_PLATFORM_RADIUS)
        .extrude(LOWER_PLATFORM_THICKNESS)
    )
    carrier = (
        carrier.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .circle(0.118)
        .circle(0.052)
        .cutBlind(-0.004)
    )

    arm_base = (
        cq.Workplane("XY")
        .box(0.072, 0.094, 0.020, centered=(True, True, False))
        .translate((0.110, 0.0, LOWER_BEARING_HEIGHT + LOWER_PLATFORM_THICKNESS))
    )
    tower = (
        cq.Workplane("XY")
        .box(0.038, 0.094, 0.095, centered=(True, True, False))
        .translate((0.136, 0.0, LOWER_BEARING_HEIGHT + LOWER_PLATFORM_THICKNESS))
    )
    head = (
        cq.Workplane("XY")
        .box(0.086, 0.094, 0.024, centered=(True, True, False))
        .translate((UPPER_AXIS_OFFSET, 0.0, 0.123))
    )
    upper_bearing = (
        cq.Workplane("XY")
        .circle(0.038)
        .extrude(0.012)
        .translate((UPPER_AXIS_OFFSET, 0.0, UPPER_AXIS_HEIGHT - 0.012))
    )
    gusset = (
        cq.Workplane("XZ")
        .moveTo(0.075, LOWER_BEARING_HEIGHT + LOWER_PLATFORM_THICKNESS)
        .lineTo(0.132, LOWER_BEARING_HEIGHT + LOWER_PLATFORM_THICKNESS)
        .lineTo(0.184, 0.110)
        .lineTo(0.184, 0.123)
        .lineTo(0.154, 0.123)
        .lineTo(0.110, 0.062)
        .lineTo(0.075, 0.062)
        .close()
        .extrude(0.094)
        .translate((0.0, -0.047, 0.0))
    )

    carrier = carrier.union(arm_base).union(tower).union(head).union(upper_bearing).union(gusset)
    carrier = carrier.cut(
        cq.Workplane("XY")
        .box(0.110, 0.054, 0.062, centered=(True, True, False))
        .translate((0.158, 0.0, 0.048))
    )
    return carrier


def _upper_stage_shape() -> cq.Workplane:
    stage = cq.Workplane("XY").circle(UPPER_COLLAR_RADIUS).extrude(0.010)
    stage = stage.faces(">Z").workplane(centerOption="CenterOfMass").circle(UPPER_STAGE_RADIUS).extrude(0.014)
    stage = (
        stage.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .circle(0.056)
        .circle(0.022)
        .cutBlind(-0.003)
    )
    stage = (
        stage.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .circle(0.018)
        .extrude(0.018)
    )
    tooling_tab = (
        cq.Workplane("XY")
        .box(0.032, 0.050, 0.010, centered=(True, True, False))
        .translate((0.072, 0.0, 0.010))
    )
    stage = stage.union(tooling_tab)
    return stage


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="edge_arm_offset_rotary_stack")

    model.material("base_charcoal", rgba=(0.18, 0.19, 0.20, 1.0))
    model.material("carrier_alloy", rgba=(0.70, 0.72, 0.75, 1.0))
    model.material("stage_orange", rgba=(0.88, 0.45, 0.12, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_shape(), "base_mesh"),
        material="base_charcoal",
        name="base_mesh",
    )

    lower_carrier = model.part("lower_carrier")
    lower_carrier.visual(
        mesh_from_cadquery(_lower_carrier_shape(), "lower_carrier_mesh"),
        material="carrier_alloy",
        name="lower_carrier_mesh",
    )

    upper_stage = model.part("upper_stage")
    upper_stage.visual(
        mesh_from_cadquery(_upper_stage_shape(), "upper_stage_mesh"),
        material="stage_orange",
        name="upper_stage_mesh",
    )

    model.articulation(
        "base_to_lower",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_carrier,
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + PEDESTAL_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.6,
            lower=-2.7,
            upper=2.7,
        ),
    )
    model.articulation(
        "lower_to_upper",
        ArticulationType.REVOLUTE,
        parent=lower_carrier,
        child=upper_stage,
        origin=Origin(xyz=(UPPER_AXIS_OFFSET, 0.0, UPPER_AXIS_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=2.2,
            lower=-3.0,
            upper=3.0,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lower_carrier = object_model.get_part("lower_carrier")
    upper_stage = object_model.get_part("upper_stage")
    base_to_lower = object_model.get_articulation("base_to_lower")
    lower_to_upper = object_model.get_articulation("lower_to_upper")

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

    ctx.expect_contact(base, lower_carrier, name="base_bearing_contacts_lower_stage")
    ctx.expect_contact(lower_carrier, upper_stage, name="arm_head_contacts_upper_stage")
    ctx.expect_origin_distance(
        upper_stage,
        lower_carrier,
        axes="xy",
        min_dist=0.180,
        max_dist=0.192,
        name="upper_axis_is_offset_from_lower_axis",
    )
    ctx.expect_origin_gap(
        upper_stage,
        lower_carrier,
        axis="z",
        min_gap=0.156,
        max_gap=0.162,
        name="upper_axis_is_raised_above_lower_axis",
    )
    ctx.check(
        "parallel_vertical_revolute_axes",
        base_to_lower.axis == (0.0, 0.0, 1.0) and lower_to_upper.axis == (0.0, 0.0, 1.0),
        details=f"lower axis={base_to_lower.axis}, upper axis={lower_to_upper.axis}",
    )

    with ctx.pose({base_to_lower: 0.0, lower_to_upper: 0.0}):
        upper_rest = ctx.part_world_position(upper_stage)
    with ctx.pose({base_to_lower: 1.0, lower_to_upper: 0.0}):
        upper_swung = ctx.part_world_position(upper_stage)
    rest_radius = hypot(upper_rest[0], upper_rest[1])
    swung_radius = hypot(upper_swung[0], upper_swung[1])
    ctx.check(
        "lower_stage_orbits_upper_axis_support",
        hypot(upper_swung[0] - upper_rest[0], upper_swung[1] - upper_rest[1]) > 0.16
        and abs(rest_radius - swung_radius) < 0.003
        and abs(upper_swung[2] - upper_rest[2]) < 1e-6,
        details=f"rest={upper_rest}, swung={upper_swung}",
    )

    with ctx.pose({base_to_lower: 0.0, lower_to_upper: 0.0}):
        upper_aabb_0 = ctx.part_element_world_aabb(upper_stage, elem="upper_stage_mesh")
    with ctx.pose({base_to_lower: 0.0, lower_to_upper: pi / 2.0}):
        upper_aabb_90 = ctx.part_element_world_aabb(upper_stage, elem="upper_stage_mesh")
    span_x_0 = upper_aabb_0[1][0] - upper_aabb_0[0][0]
    span_y_0 = upper_aabb_0[1][1] - upper_aabb_0[0][1]
    span_x_90 = upper_aabb_90[1][0] - upper_aabb_90[0][0]
    span_y_90 = upper_aabb_90[1][1] - upper_aabb_90[0][1]
    ctx.check(
        "upper_stage_self_rotation_changes_tab_orientation",
        span_x_0 > span_y_0 + 0.010 and span_y_90 > span_x_90 + 0.010,
        details=(
            f"q0 spans=({span_x_0:.4f}, {span_y_0:.4f}), "
            f"q90 spans=({span_x_90:.4f}, {span_y_90:.4f})"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
