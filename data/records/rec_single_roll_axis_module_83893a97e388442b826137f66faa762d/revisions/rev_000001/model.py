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
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.205
BASE_WIDTH = 0.116
BASE_THICKNESS = 0.012

PEDESTAL_LENGTH = 0.104
PEDESTAL_WIDTH = 0.076
PEDESTAL_HEIGHT = 0.036

BEAM_LENGTH = 0.170
BEAM_WIDTH = 0.052
BEAM_HEIGHT = 0.012
BEAM_TOP_Z = BASE_THICKNESS + PEDESTAL_HEIGHT + BEAM_HEIGHT

SUPPORT_CENTER_X = 0.072
SUPPORT_THICKNESS = 0.016
SUPPORT_DEPTH = 0.064
SUPPORT_COLUMN_HEIGHT = 0.068
SUPPORT_AXIS_LOCAL_Z = SUPPORT_COLUMN_HEIGHT

ROLL_AXIS_Z = BEAM_TOP_Z + SUPPORT_AXIS_LOCAL_Z

HOUSING_RADIUS = 0.025
BORE_RADIUS = 0.0188
CARTRIDGE_RADIUS = 0.0228
HOUSING_LENGTH = SUPPORT_THICKNESS
OUTER_FLANGE_RADIUS = 0.028
OUTER_FLANGE_LENGTH = 0.004

BARREL_RADIUS = 0.030
BARREL_BODY_LENGTH = 0.084
COLLAR_RADIUS = 0.034
COLLAR_LENGTH = 0.012
THRUST_RADIUS = 0.0226
THRUST_LENGTH = 0.004
JOURNAL_RADIUS = 0.0162
JOURNAL_LENGTH = 0.016

LENS_RECESS_RADIUS = 0.022
LENS_RECESS_LENGTH = 0.010
TRIM_GROOVE_OUTER_RADIUS = 0.0332
TRIM_GROOVE_INNER_RADIUS = 0.0316
TRIM_GROOVE_LENGTH = 0.006

POD_LENGTH = 0.042
POD_WIDTH = 0.022
POD_HEIGHT = 0.010

ROLL_LIMIT = 1.30


def _x_cylinder(radius: float, length: float, x_start: float) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length).translate((x_start, 0.0, 0.0))


def _x_ring(outer_radius: float, inner_radius: float, length: float, x_start: float) -> cq.Workplane:
    return _x_cylinder(outer_radius, length, x_start).cut(_x_cylinder(inner_radius, length, x_start))


def _build_base_frame_shape() -> cq.Workplane:
    base_plate = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)
        .edges("|Z")
        .fillet(0.004)
        .translate((0.0, 0.0, BASE_THICKNESS / 2.0))
    )
    pedestal = (
        cq.Workplane("XY")
        .box(PEDESTAL_LENGTH, PEDESTAL_WIDTH, PEDESTAL_HEIGHT)
        .edges("|Z")
        .fillet(0.003)
        .translate((0.0, 0.0, BASE_THICKNESS + PEDESTAL_HEIGHT / 2.0))
    )

    beam = (
        cq.Workplane("XY")
        .box(BEAM_LENGTH, BEAM_WIDTH, BEAM_HEIGHT)
        .edges("|Z")
        .fillet(0.0025)
        .translate((0.0, 0.0, BASE_THICKNESS + PEDESTAL_HEIGHT + BEAM_HEIGHT / 2.0))
    )

    left_rib = (
        cq.Workplane("XY")
        .box(0.032, 0.020, 0.028)
        .edges("|Z")
        .fillet(0.0018)
        .translate((-0.028, 0.0, BASE_THICKNESS + PEDESTAL_HEIGHT + 0.014))
    )
    right_rib = left_rib.translate((0.056, 0.0, 0.0))

    return base_plate.union(pedestal).union(beam).union(left_rib).union(right_rib)


def _build_support_shape(side: str) -> cq.Workplane:
    sign = -1.0 if side == "left" else 1.0

    column = (
        cq.Workplane("XY")
        .box(SUPPORT_THICKNESS, SUPPORT_DEPTH, SUPPORT_COLUMN_HEIGHT)
        .edges("|Z")
        .fillet(0.002)
        .translate((0.0, 0.0, SUPPORT_COLUMN_HEIGHT / 2.0))
    )
    housing = _x_ring(HOUSING_RADIUS, CARTRIDGE_RADIUS, HOUSING_LENGTH, -SUPPORT_THICKNESS / 2.0).translate(
        (0.0, 0.0, SUPPORT_AXIS_LOCAL_Z)
    )
    if side == "left":
        outer_flange_start = -SUPPORT_THICKNESS / 2.0 - OUTER_FLANGE_LENGTH
        inner_race_start = SUPPORT_THICKNESS / 2.0
        bore_start = outer_flange_start
    else:
        outer_flange_start = SUPPORT_THICKNESS / 2.0
        inner_race_start = -SUPPORT_THICKNESS / 2.0 - 0.006
        bore_start = -SUPPORT_THICKNESS / 2.0
    outer_flange = _x_ring(OUTER_FLANGE_RADIUS, BORE_RADIUS, OUTER_FLANGE_LENGTH, outer_flange_start).translate(
        (0.0, 0.0, SUPPORT_AXIS_LOCAL_Z)
    )
    inner_race = _x_ring(CARTRIDGE_RADIUS, BORE_RADIUS, 0.006, inner_race_start).translate(
        (0.0, 0.0, SUPPORT_AXIS_LOCAL_Z)
    )
    bore = _x_cylinder(BORE_RADIUS, SUPPORT_THICKNESS + OUTER_FLANGE_LENGTH + 0.006, bore_start).translate(
        (0.0, 0.0, SUPPORT_AXIS_LOCAL_Z)
    )
    gusset = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.0, SUPPORT_COLUMN_HEIGHT - 0.018),
                (0.0, SUPPORT_AXIS_LOCAL_Z - 0.010),
                (sign * 0.012, SUPPORT_AXIS_LOCAL_Z + 0.006),
                (sign * 0.012, SUPPORT_COLUMN_HEIGHT - 0.018),
            ]
        )
        .close()
        .extrude(0.018)
        .translate((0.0, -0.009, 0.0))
    )

    return column.union(housing).union(outer_flange).union(inner_race).union(gusset).cut(bore)


def _build_barrel_shape() -> cq.Workplane:
    body_start = -BARREL_BODY_LENGTH / 2.0
    left_collar_start = body_start - COLLAR_LENGTH
    right_collar_start = body_start + BARREL_BODY_LENGTH
    left_thrust_start = left_collar_start - THRUST_LENGTH
    right_thrust_start = right_collar_start + COLLAR_LENGTH
    left_journal_start = left_thrust_start - JOURNAL_LENGTH
    right_journal_start = right_thrust_start + THRUST_LENGTH

    barrel = (
        _x_cylinder(BARREL_RADIUS, BARREL_BODY_LENGTH, body_start)
        .union(_x_cylinder(COLLAR_RADIUS, COLLAR_LENGTH, left_collar_start))
        .union(_x_cylinder(COLLAR_RADIUS, COLLAR_LENGTH, right_collar_start))
        .union(_x_cylinder(THRUST_RADIUS, THRUST_LENGTH, left_thrust_start))
        .union(_x_cylinder(THRUST_RADIUS, THRUST_LENGTH, right_thrust_start))
        .union(_x_cylinder(JOURNAL_RADIUS, JOURNAL_LENGTH, left_journal_start))
        .union(_x_cylinder(JOURNAL_RADIUS, JOURNAL_LENGTH, right_journal_start))
    )

    pod = (
        cq.Workplane("XY")
        .box(POD_LENGTH, POD_WIDTH, POD_HEIGHT)
        .edges("|Z")
        .fillet(0.0015)
        .translate((0.006, 0.0, BARREL_RADIUS + POD_HEIGHT / 2.0 - 0.002))
    )

    lens_recess = _x_cylinder(LENS_RECESS_RADIUS, LENS_RECESS_LENGTH, body_start)
    left_trim_groove = _x_ring(
        TRIM_GROOVE_OUTER_RADIUS,
        TRIM_GROOVE_INNER_RADIUS,
        TRIM_GROOVE_LENGTH,
        body_start + 0.010,
    )
    right_trim_groove = _x_ring(
        TRIM_GROOVE_OUTER_RADIUS,
        TRIM_GROOVE_INNER_RADIUS,
        TRIM_GROOVE_LENGTH,
        body_start + BARREL_BODY_LENGTH - 0.016,
    )

    return barrel.union(pod).cut(lens_recess).cut(left_trim_groove).cut(right_trim_groove)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sensor_roll_module")

    model.material("frame_gray", rgba=(0.76, 0.78, 0.81, 1.0))
    model.material("trim_dark", rgba=(0.32, 0.34, 0.37, 1.0))
    model.material("barrel_black", rgba=(0.14, 0.15, 0.17, 1.0))

    base_frame = model.part("base_frame")
    base_frame.visual(
        mesh_from_cadquery(
            _build_base_frame_shape(),
            "sensor_roll_base_frame",
            tolerance=0.00025,
            angular_tolerance=0.04,
        ),
        material="frame_gray",
        name="frame",
    )
    base_frame.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, BEAM_TOP_Z)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, BEAM_TOP_Z / 2.0)),
    )

    left_support = model.part("left_support")
    left_support.visual(
        mesh_from_cadquery(
            _build_support_shape("left"),
            "sensor_roll_left_support",
            tolerance=0.0002,
            angular_tolerance=0.04,
        ),
        material="trim_dark",
        name="support",
    )
    left_support.inertial = Inertial.from_geometry(
        Box((SUPPORT_THICKNESS + OUTER_FLANGE_LENGTH, SUPPORT_DEPTH, SUPPORT_COLUMN_HEIGHT + HOUSING_RADIUS)),
        mass=0.24,
        origin=Origin(xyz=(0.0, 0.0, (SUPPORT_COLUMN_HEIGHT + HOUSING_RADIUS) / 2.0)),
    )

    right_support = model.part("right_support")
    right_support.visual(
        mesh_from_cadquery(
            _build_support_shape("right"),
            "sensor_roll_right_support",
            tolerance=0.0002,
            angular_tolerance=0.04,
        ),
        material="trim_dark",
        name="support",
    )
    right_support.inertial = Inertial.from_geometry(
        Box((SUPPORT_THICKNESS + OUTER_FLANGE_LENGTH, SUPPORT_DEPTH, SUPPORT_COLUMN_HEIGHT + HOUSING_RADIUS)),
        mass=0.24,
        origin=Origin(xyz=(0.0, 0.0, (SUPPORT_COLUMN_HEIGHT + HOUSING_RADIUS) / 2.0)),
    )

    model.articulation(
        "frame_to_left_support",
        ArticulationType.FIXED,
        parent=base_frame,
        child=left_support,
        origin=Origin(xyz=(-SUPPORT_CENTER_X, 0.0, BEAM_TOP_Z)),
    )
    model.articulation(
        "frame_to_right_support",
        ArticulationType.FIXED,
        parent=base_frame,
        child=right_support,
        origin=Origin(xyz=(SUPPORT_CENTER_X, 0.0, BEAM_TOP_Z)),
    )

    instrument_barrel = model.part("instrument_barrel")
    instrument_barrel.visual(
        mesh_from_cadquery(
            _build_barrel_shape(),
            "sensor_roll_barrel_body",
            tolerance=0.00025,
            angular_tolerance=0.04,
        ),
        material="barrel_black",
        name="barrel",
    )
    instrument_barrel.inertial = Inertial.from_geometry(
        Cylinder(
            radius=COLLAR_RADIUS,
            length=BARREL_BODY_LENGTH + 2.0 * (COLLAR_LENGTH + JOURNAL_LENGTH),
        ),
        mass=0.82,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    model.articulation(
        "barrel_roll",
        ArticulationType.REVOLUTE,
        parent=base_frame,
        child=instrument_barrel,
        origin=Origin(xyz=(0.0, 0.0, ROLL_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.6,
            lower=-ROLL_LIMIT,
            upper=ROLL_LIMIT,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_frame = object_model.get_part("base_frame")
    left_support = object_model.get_part("left_support")
    right_support = object_model.get_part("right_support")
    instrument_barrel = object_model.get_part("instrument_barrel")
    barrel_roll = object_model.get_articulation("barrel_roll")

    ctx.allow_overlap(
        base_frame,
        left_support,
        reason="Left support is modeled as a bolted cheek module with a seated mounting foot that intentionally shares the beam interface volume.",
    )
    ctx.allow_overlap(
        base_frame,
        right_support,
        reason="Right support is modeled as a bolted cheek module with a seated mounting foot that intentionally shares the beam interface volume.",
    )
    ctx.allow_overlap(
        instrument_barrel,
        left_support,
        reason="Left roll journal occupies the simplified bearing-cartridge envelope inside the left support housing.",
    )
    ctx.allow_overlap(
        instrument_barrel,
        right_support,
        reason="Right roll journal occupies the simplified bearing-cartridge envelope inside the right support housing.",
    )

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

    axis = tuple(round(value, 6) for value in barrel_roll.axis)
    limits = barrel_roll.motion_limits

    ctx.check("base_frame_present", base_frame is not None, "base_frame missing")
    ctx.check("left_support_present", left_support is not None, "left_support missing")
    ctx.check("right_support_present", right_support is not None, "right_support missing")
    ctx.check("instrument_barrel_present", instrument_barrel is not None, "instrument_barrel missing")
    ctx.check("barrel_roll_axis_x", axis == (1.0, 0.0, 0.0), f"expected x-axis roll, got {barrel_roll.axis}")
    ctx.check(
        "barrel_roll_limits_defined",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower < 0.0 < limits.upper
        and limits.upper - limits.lower >= 2.4,
        f"unexpected roll limits: {limits}",
    )

    ctx.expect_contact(
        left_support,
        base_frame,
        contact_tol=0.0008,
        name="left_support_grounded_on_frame",
    )
    ctx.expect_contact(
        right_support,
        base_frame,
        contact_tol=0.0008,
        name="right_support_grounded_on_frame",
    )
    ctx.expect_contact(
        instrument_barrel,
        left_support,
        contact_tol=0.0008,
        name="barrel_contacts_left_support_face",
    )
    ctx.expect_contact(
        instrument_barrel,
        right_support,
        contact_tol=0.0008,
        name="barrel_contacts_right_support_face",
    )
    ctx.expect_overlap(
        instrument_barrel,
        base_frame,
        axes="xy",
        min_overlap=0.042,
        name="barrel_reads_as_dense_centered_subassembly",
    )
    ctx.expect_origin_gap(
        right_support,
        left_support,
        axis="x",
        min_gap=0.143,
        max_gap=0.145,
        name="support_pair_stays_coaxial_and_compact",
    )
    ctx.expect_origin_gap(
        instrument_barrel,
        base_frame,
        axis="z",
        min_gap=0.126,
        max_gap=0.130,
        name="roll_axis_height_keeps_module_compact",
    )
    ctx.expect_gap(
        instrument_barrel,
        base_frame,
        axis="z",
        min_gap=0.016,
        max_gap=0.024,
        name="barrel_clears_lower_frame",
    )

    with ctx.pose({barrel_roll: limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_lower_roll_limit")
    with ctx.pose({barrel_roll: limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_upper_roll_limit")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
