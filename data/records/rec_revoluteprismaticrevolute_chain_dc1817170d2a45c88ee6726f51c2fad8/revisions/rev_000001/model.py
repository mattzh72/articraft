from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


BASE_LENGTH = 0.24
BASE_WIDTH = 0.20
BASE_HEIGHT = 0.076
BASE_TOP = 0.116
YAW_AXIS_Z = 0.125

OUTER_BEAM_LENGTH = 0.30
OUTER_BEAM_WIDTH = 0.110
OUTER_BEAM_HEIGHT = 0.098
OUTER_BEAM_CENTER_X = 0.185
OUTER_BEAM_CENTER_Z = 0.058

STAGE_ORIGIN_X = 0.070
STAGE_ORIGIN_Z = 0.058
STAGE_TRAVEL = 0.16
STAGE_RAIL_LENGTH = 0.235
SPINDLE_AXIS_X = 0.334


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cyl_z(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(length).translate(
        (center[0], center[1], center[2] - length / 2.0)
    )


def _cyl_x(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length).translate(
        (center[0] - length / 2.0, center[1], center[2])
    )


def _hex_x(diameter: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("YZ").polygon(6, diameter).extrude(length).translate(
        (center[0] - length / 2.0, center[1], center[2])
    )


def _base_shape() -> cq.Workplane:
    lower_body = _box((BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT), (0.0, 0.0, BASE_HEIGHT / 2.0))
    lower_body = lower_body.edges("|Z").fillet(0.016)

    upper_plinth = _box((0.18, 0.14, 0.024), (0.0, 0.0, 0.088))
    turret = _cyl_z(0.085, 0.010, (0.0, 0.0, 0.105))
    bearing_land = _cyl_z(0.072, 0.006, (0.0, 0.0, 0.113))
    service_boss = _box((0.11, 0.055, 0.020), (-0.040, 0.0, 0.079))

    return lower_body.union(upper_plinth).union(turret).union(bearing_land).union(service_boss)


def _outer_turntable_shape() -> cq.Workplane:
    platter = _cyl_z(0.083, 0.018, (0.0, 0.0, 0.0))
    skirt = (
        cq.Workplane("XY")
        .circle(0.081)
        .circle(0.072)
        .extrude(0.014)
        .translate((0.0, 0.0, -0.013))
    )
    root_block = _box((0.095, 0.092, 0.062), (0.012, 0.0, 0.041))
    rear_cover = _box((0.060, 0.070, 0.046), (-0.040, 0.0, 0.035))
    return platter.union(skirt).union(root_block).union(rear_cover)


def _outer_sleeve_shape() -> cq.Workplane:
    pedestal = _box((0.080, 0.090, 0.064), (0.025, 0.0, 0.041))
    left_rail = _box((0.215, 0.012, 0.018), (0.1725, 0.033, 0.063))
    right_rail = _box((0.215, 0.012, 0.018), (0.1725, -0.033, 0.063))
    top_bridge = _box((0.145, 0.078, 0.010), (0.1375, 0.0, 0.077))
    front_stop = _box((0.012, 0.070, 0.018), (0.279, 0.0, 0.063))
    return (
        pedestal.union(left_rail)
        .union(right_rail)
        .union(top_bridge)
        .union(front_stop)
    )


def _stage_rail_shape() -> cq.Workplane:
    spine = _box((0.240, 0.024, 0.024), (0.120, 0.0, -0.026))
    left_shoe = _box((0.120, 0.012, 0.016), (0.060, 0.033, -0.012))
    right_shoe = _box((0.120, 0.012, 0.016), (0.060, -0.033, -0.012))
    rear_pad = _box((0.050, 0.020, 0.008), (0.040, 0.0, -0.034))
    return spine.union(left_shoe).union(right_shoe).union(rear_pad)


def _stage_carriage_shape() -> cq.Workplane:
    neck = _box((0.060, 0.024, 0.022), (0.250, 0.0, -0.014))
    bearing_housing = _box((0.056, 0.056, 0.052), (0.306, 0.0, 0.0))
    face_plate = _box((0.006, 0.050, 0.050), (0.331, 0.0, 0.0))
    housing_bore = _cyl_x(0.0205, 0.078, (0.304, 0.0, 0.0))
    nose_bore = _cyl_x(0.0200, 0.010, (0.331, 0.0, 0.0))
    return neck.union(bearing_housing).union(face_plate).cut(housing_bore).cut(nose_bore)


def _spindle_shape() -> cq.Workplane:
    rear_flange = _cyl_x(0.023, 0.006, (0.003, 0.0, 0.0))
    journal = _cyl_x(0.0195, 0.028, (0.020, 0.0, 0.0))
    nose_body = _cyl_x(0.016, 0.024, (0.046, 0.0, 0.0))
    nose_cap = _cyl_x(0.014, 0.016, (0.067, 0.0, 0.0))
    tool_stub = _cyl_x(0.008, 0.012, (0.081, 0.0, 0.0))
    return rear_flange.union(journal).union(nose_body).union(nose_cap).union(tool_stub)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="yaw_slide_spindle_arm")

    model.material("housing_charcoal", rgba=(0.20, 0.22, 0.24, 1.0))
    model.material("beam_graphite", rgba=(0.30, 0.32, 0.35, 1.0))
    model.material("machined_steel", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("spindle_black", rgba=(0.13, 0.14, 0.16, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_shape(), "base_housing"),
        material="housing_charcoal",
        name="housing",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_TOP)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP / 2.0)),
    )

    outer_arm = model.part("outer_arm")
    outer_arm.visual(
        mesh_from_cadquery(_outer_turntable_shape(), "outer_turntable"),
        material="housing_charcoal",
        name="turntable",
    )
    outer_arm.visual(
        mesh_from_cadquery(_outer_sleeve_shape(), "outer_sleeve"),
        material="beam_graphite",
        name="guide_sleeve",
    )
    outer_arm.inertial = Inertial.from_geometry(
        Box((0.42, 0.12, 0.12)),
        mass=11.5,
        origin=Origin(xyz=(0.14, 0.0, 0.055)),
    )

    forearm_stage = model.part("forearm_stage")
    forearm_stage.visual(
        mesh_from_cadquery(_stage_rail_shape(), "stage_rail"),
        material="machined_steel",
        name="rail_body",
    )
    forearm_stage.visual(
        mesh_from_cadquery(_stage_carriage_shape(), "stage_carriage"),
        material="machined_steel",
        name="bearing_carriage",
    )
    forearm_stage.inertial = Inertial.from_geometry(
        Box((0.34, 0.08, 0.08)),
        mass=5.2,
        origin=Origin(xyz=(0.18, 0.0, 0.0)),
    )

    spindle_head = model.part("spindle_head")
    spindle_head.visual(
        mesh_from_cadquery(_spindle_shape(), "spindle_head"),
        material="spindle_black",
        name="spindle_body",
    )
    spindle_head.inertial = Inertial.from_geometry(
        Box((0.21, 0.05, 0.05)),
        mass=1.6,
        origin=Origin(xyz=(-0.015, 0.0, 0.0)),
    )

    model.articulation(
        "base_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=outer_arm,
        origin=Origin(xyz=(0.0, 0.0, YAW_AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=1.2,
            lower=-2.4,
            upper=2.4,
        ),
    )
    model.articulation(
        "forearm_slide",
        ArticulationType.PRISMATIC,
        parent=outer_arm,
        child=forearm_stage,
        origin=Origin(xyz=(STAGE_ORIGIN_X, 0.0, STAGE_ORIGIN_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=0.28,
            lower=0.0,
            upper=STAGE_TRAVEL,
        ),
    )
    model.articulation(
        "spindle_roll",
        ArticulationType.REVOLUTE,
        parent=forearm_stage,
        child=spindle_head,
        origin=Origin(xyz=(SPINDLE_AXIS_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=4.0,
            lower=-pi,
            upper=pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    outer_arm = object_model.get_part("outer_arm")
    forearm_stage = object_model.get_part("forearm_stage")
    spindle_head = object_model.get_part("spindle_head")

    base_yaw = object_model.get_articulation("base_yaw")
    forearm_slide = object_model.get_articulation("forearm_slide")
    spindle_roll = object_model.get_articulation("spindle_roll")

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
        forearm_stage,
        spindle_head,
        elem_a="bearing_carriage",
        elem_b="spindle_body",
        reason="simplified rotating spindle journal is modeled as one rigid nose inside a machined bearing carrier bore, so mesh-backed nested bearing geometry can register pose-dependent self-nesting penetration",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "base_yaw_axis_is_vertical",
        tuple(base_yaw.axis) == (0.0, 0.0, 1.0),
        details=f"expected z-axis yaw, got {base_yaw.axis}",
    )
    ctx.check(
        "forearm_slide_axis_is_longitudinal",
        tuple(forearm_slide.axis) == (1.0, 0.0, 0.0),
        details=f"expected x-axis slide, got {forearm_slide.axis}",
    )
    ctx.check(
        "spindle_roll_axis_matches_tool_axis",
        tuple(spindle_roll.axis) == (1.0, 0.0, 0.0),
        details=f"expected x-axis spindle roll, got {spindle_roll.axis}",
    )

    ctx.expect_contact(
        outer_arm,
        base,
        elem_a="turntable",
        elem_b="housing",
        contact_tol=0.0015,
        name="turntable_supported_by_base_housing",
    )
    ctx.expect_overlap(
        outer_arm,
        base,
        axes="xy",
        elem_a="turntable",
        elem_b="housing",
        min_overlap=0.12,
        name="turntable_has_bearing_footprint_over_base",
    )
    ctx.expect_within(
        forearm_stage,
        outer_arm,
        axes="y",
        inner_elem="rail_body",
        outer_elem="guide_sleeve",
        margin=0.0,
        name="stage_rail_runs_between_outer_guides",
    )
    ctx.expect_overlap(
        outer_arm,
        forearm_stage,
        axes="x",
        elem_a="guide_sleeve",
        elem_b="rail_body",
        min_overlap=0.16,
        name="guide_sleeve_overhangs_stage_along_beam_axis",
    )
    ctx.expect_overlap(
        forearm_stage,
        spindle_head,
        axes="yz",
        elem_a="bearing_carriage",
        elem_b="spindle_body",
        min_overlap=0.038,
        name="spindle_sits_inside_bearing_carriage_envelope",
    )
    ctx.expect_within(
        spindle_head,
        forearm_stage,
        axes="yz",
        inner_elem="spindle_body",
        outer_elem="bearing_carriage",
        margin=0.012,
        name="spindle_body_stays_within_mount_width_and_height",
    )

    with ctx.pose({forearm_slide: STAGE_TRAVEL}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_part_overlap_at_full_extension")
        ctx.expect_within(
            forearm_stage,
            outer_arm,
            axes="y",
            inner_elem="rail_body",
            outer_elem="guide_sleeve",
            margin=0.0,
            name="stage_rail_runs_between_outer_guides_when_extended",
        )

    with ctx.pose({base_yaw: 1.25, forearm_slide: 0.10, spindle_roll: 2.2}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_part_overlap_in_offset_service_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
