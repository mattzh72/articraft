from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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


BASE_WIDTH = 0.74
BASE_DEPTH = 0.60
BASE_HEIGHT = 0.10
PLINTH_WIDTH = 0.50
PLINTH_DEPTH = 0.40
PLINTH_HEIGHT = 0.12

COLUMN_RADIUS = 0.12
COLUMN_HEIGHT = 1.38

PIVOT_Z = 1.28
SLEEVE_OUTER_RADIUS = 0.19
SLEEVE_INNER_RADIUS = COLUMN_RADIUS + 0.008
SLEEVE_HEIGHT = 0.24
SLEEVE_SPLIT_GAP = 0.035
SUPPORT_FLANGE_THICKNESS = 0.02

BEAM_LENGTH = 1.20
BEAM_WIDTH = 0.18
BEAM_HEIGHT = 0.16
ROOT_BLOCK_LENGTH = 0.30
ROOT_BLOCK_WIDTH = 0.24
ROOT_BLOCK_HEIGHT = 0.28
TAIL_LENGTH = 0.16
TAIL_WIDTH = 0.18
TAIL_HEIGHT = 0.12

SADDLE_LENGTH = 0.24
SADDLE_WIDTH = 0.30
SADDLE_HEIGHT = 0.30
SADDLE_CAP_LENGTH = 0.18
SADDLE_CAP_WIDTH = 0.24
SADDLE_CAP_HEIGHT = 0.06
SADDLE_BODY_WIDTH = 0.22
SADDLE_BODY_HEIGHT = 0.20
SADDLE_BODY_DROP = 0.26
GUIDE_PAD_LENGTH = 0.18
GUIDE_PAD_WIDTH = 0.08
GUIDE_PAD_THICKNESS = 0.02
SIDE_SHOE_LENGTH = 0.14
SIDE_SHOE_WIDTH = 0.03
SIDE_SHOE_HEIGHT = 0.14
SLIDE_START_X = 0.70
SLIDE_TRAVEL = 0.56


def _make_ring(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    outer = cq.Workplane("XY").circle(outer_radius).extrude(height).translate((0.0, 0.0, -height / 2.0))
    inner = (
        cq.Workplane("XY")
        .circle(inner_radius)
        .extrude(height + 0.01)
        .translate((0.0, 0.0, -(height + 0.01) / 2.0))
    )
    return outer.cut(inner)


def _make_column_assembly() -> cq.Workplane:
    base = cq.Workplane("XY").rect(BASE_WIDTH, BASE_DEPTH).extrude(BASE_HEIGHT)
    plinth = (
        cq.Workplane("XY")
        .rect(PLINTH_WIDTH, PLINTH_DEPTH)
        .extrude(PLINTH_HEIGHT)
        .translate((0.0, 0.0, BASE_HEIGHT))
    )
    column = (
        cq.Workplane("XY")
        .circle(COLUMN_RADIUS)
        .extrude(COLUMN_HEIGHT)
        .translate((0.0, 0.0, BASE_HEIGHT))
    )
    flange_z = PIVOT_Z - (SLEEVE_HEIGHT / 2.0) - SUPPORT_FLANGE_THICKNESS
    support_flange = (
        cq.Workplane("XY")
        .circle(SLEEVE_OUTER_RADIUS)
        .extrude(SUPPORT_FLANGE_THICKNESS)
        .translate((0.0, 0.0, flange_z))
    )

    return base.union(plinth).union(column).union(support_flange)


def _make_beam() -> cq.Workplane:
    beam_start_x = 0.22

    sleeve = _make_ring(SLEEVE_OUTER_RADIUS, SLEEVE_INNER_RADIUS, SLEEVE_HEIGHT)
    split_cut = cq.Workplane("XY").box(
        SLEEVE_OUTER_RADIUS * 2.2,
        SLEEVE_SPLIT_GAP,
        SLEEVE_HEIGHT + 0.02,
    )
    sleeve = sleeve.cut(split_cut)

    head_block = cq.Workplane("XY").box(ROOT_BLOCK_LENGTH, ROOT_BLOCK_WIDTH, ROOT_BLOCK_HEIGHT).translate(
        (beam_start_x + (ROOT_BLOCK_LENGTH / 2.0), 0.0, 0.0)
    )
    beam_body = cq.Workplane("XY").box(BEAM_LENGTH, BEAM_WIDTH, BEAM_HEIGHT).translate(
        (beam_start_x + (BEAM_LENGTH / 2.0), 0.0, 0.0)
    )
    tail = cq.Workplane("XY").box(TAIL_LENGTH, TAIL_WIDTH, TAIL_HEIGHT).translate((-0.27, 0.0, 0.0))

    clamp_ear_pos = SLEEVE_OUTER_RADIUS + 0.045
    clamp_ear_right = cq.Workplane("XY").box(0.11, 0.05, 0.11).translate((0.02, clamp_ear_pos, 0.0))
    clamp_ear_left = cq.Workplane("XY").box(0.11, 0.05, 0.11).translate((0.02, -clamp_ear_pos, 0.0))

    top_way = cq.Workplane("XY").box(BEAM_LENGTH - 0.08, 0.08, 0.04).translate(
        (beam_start_x + (BEAM_LENGTH / 2.0) + 0.04, 0.0, (BEAM_HEIGHT / 2.0) + 0.02)
    )

    def make_web(y_offset: float) -> cq.Workplane:
        return (
            cq.Workplane("XZ")
            .moveTo(beam_start_x, -(ROOT_BLOCK_HEIGHT / 2.0))
            .lineTo(0.56, -(BEAM_HEIGHT / 2.0))
            .lineTo(0.30, 0.01)
            .close()
            .extrude(0.028)
            .translate((0.0, y_offset, 0.0))
        )

    web_right = make_web((BEAM_WIDTH / 2.0) - 0.028)
    web_left = make_web(-(BEAM_WIDTH / 2.0))

    return (
        sleeve.union(head_block)
        .union(beam_body)
        .union(top_way)
        .union(tail)
        .union(clamp_ear_right)
        .union(clamp_ear_left)
        .union(web_right)
        .union(web_left)
    )


def _make_saddle() -> cq.Workplane:
    body = cq.Workplane("XY").box(SADDLE_LENGTH, SADDLE_BODY_WIDTH, SADDLE_BODY_HEIGHT).translate(
        (0.0, 0.0, -SADDLE_BODY_DROP)
    )

    roller_radius = 0.025
    roller_span = 0.08
    roller_z = -(BEAM_HEIGHT / 2.0) - roller_radius
    front_roller = (
        cq.Workplane("XZ")
        .circle(roller_radius)
        .extrude(roller_span)
        .translate((-0.05, -(roller_span / 2.0), roller_z))
    )
    rear_roller = (
        cq.Workplane("XZ")
        .circle(roller_radius)
        .extrude(roller_span)
        .translate((0.05, -(roller_span / 2.0), roller_z))
    )

    right_guide = cq.Workplane("XY").box(0.08, 0.02, 0.10).translate(
        (0.0, (BEAM_WIDTH / 2.0) + 0.03, -0.20)
    )
    left_guide = cq.Workplane("XY").box(0.08, 0.02, 0.10).translate(
        (0.0, -(BEAM_WIDTH / 2.0) - 0.03, -0.20)
    )
    cap = cq.Workplane("XY").box(SADDLE_CAP_LENGTH, SADDLE_CAP_WIDTH, SADDLE_CAP_HEIGHT).translate(
        (0.0, 0.0, -0.17)
    )
    front_nose = cq.Workplane("XY").box(0.14, 0.08, 0.12).translate((0.0, (SADDLE_BODY_WIDTH / 2.0) + 0.04, -0.18))

    return body.union(front_roller).union(rear_roller).union(right_guide).union(left_guide).union(cap).union(front_nose)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="radial_drill_arm_module")

    column_color = model.material("column_gray", rgba=(0.60, 0.62, 0.64, 1.0))
    beam_color = model.material("beam_graphite", rgba=(0.36, 0.38, 0.40, 1.0))
    saddle_color = model.material("saddle_blue", rgba=(0.20, 0.36, 0.56, 1.0))

    column = model.part("column_assembly")
    column.visual(
        mesh_from_cadquery(_make_column_assembly(), "column_assembly"),
        material=column_color,
        name="column_casting",
    )
    column.inertial = Inertial.from_geometry(
        Box((BASE_WIDTH, BASE_DEPTH, BASE_HEIGHT + COLUMN_HEIGHT)),
        mass=720.0,
        origin=Origin(xyz=(0.0, 0.0, (BASE_HEIGHT + COLUMN_HEIGHT) / 2.0)),
    )

    beam = model.part("radial_beam")
    beam.visual(
        mesh_from_cadquery(_make_beam(), "radial_beam"),
        material=beam_color,
        name="beam_casting",
    )
    beam.inertial = Inertial.from_geometry(
        Box((1.78, 0.50, 0.32)),
        mass=180.0,
        origin=Origin(xyz=(0.52, 0.0, 0.0)),
    )

    saddle = model.part("carriage_saddle")
    saddle.visual(
        mesh_from_cadquery(_make_saddle(), "carriage_saddle"),
        material=saddle_color,
        name="saddle_casting",
    )
    saddle.inertial = Inertial.from_geometry(
        Box((0.26, 0.38, 0.34)),
        mass=68.0,
        origin=Origin(),
    )

    model.articulation(
        "beam_swing",
        ArticulationType.CONTINUOUS,
        parent=column,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3200.0, velocity=0.8),
    )
    model.articulation(
        "saddle_slide",
        ArticulationType.PRISMATIC,
        parent=beam,
        child=saddle,
        origin=Origin(xyz=(SLIDE_START_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4200.0,
            velocity=0.30,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.allow_isolated_part(
        "carriage_saddle",
        reason=(
            "The saddle's roller-guided contact with the beam is mesh-backed and leaves an "
            "inspection-scale ~8e-06 m gap in the grounded-component sensor; exact guided-contact "
            "checks below validate the intended physical connection."
        ),
    )

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

    column = object_model.get_part("column_assembly")
    beam = object_model.get_part("radial_beam")
    saddle = object_model.get_part("carriage_saddle")
    beam_swing = object_model.get_articulation("beam_swing")
    saddle_slide = object_model.get_articulation("saddle_slide")

    ctx.check(
        "parts exist",
        all(part is not None for part in (column, beam, saddle)),
        "column, beam, and saddle must all be authored",
    )
    ctx.check(
        "beam swing uses a vertical axis",
        tuple(round(value, 6) for value in beam_swing.axis) == (0.0, 0.0, 1.0),
        f"beam_swing axis was {beam_swing.axis}",
    )
    ctx.check(
        "beam swing is continuous",
        beam_swing.articulation_type == ArticulationType.CONTINUOUS,
        f"beam_swing type was {beam_swing.articulation_type}",
    )
    ctx.check(
        "saddle slide follows beam axis",
        tuple(round(value, 6) for value in saddle_slide.axis) == (1.0, 0.0, 0.0),
        f"saddle_slide axis was {saddle_slide.axis}",
    )

    slide_limits = saddle_slide.motion_limits
    ctx.check(
        "saddle slide span is realistic",
        slide_limits is not None
        and slide_limits.lower is not None
        and slide_limits.upper is not None
        and abs(slide_limits.lower) < 1e-9
        and 0.55 <= slide_limits.upper <= 0.75,
        f"saddle_slide limits were {slide_limits}",
    )

    with ctx.pose(beam_swing=0.0, saddle_slide=0.0):
        ctx.expect_contact(
            beam,
            column,
            contact_tol=0.002,
            name="beam sleeve bears on the column",
        )
        ctx.expect_contact(
            saddle,
            beam,
            contact_tol=0.002,
            name="saddle is guided on the beam in the parked position",
        )
        ctx.expect_overlap(
            saddle,
            beam,
            axes="y",
            min_overlap=0.16,
            name="saddle straddles the beam width",
        )

    with ctx.pose(beam_swing=1.25, saddle_slide=0.20):
        ctx.expect_contact(
            beam,
            column,
            contact_tol=0.002,
            name="beam stays seated on the column while swung",
        )

    with ctx.pose(beam_swing=0.0, saddle_slide=SLIDE_TRAVEL - 0.04):
        ctx.expect_contact(
            saddle,
            beam,
            contact_tol=0.002,
            name="saddle remains guided at near-full extension",
        )
        ctx.expect_overlap(
            saddle,
            beam,
            axes="y",
            min_overlap=0.16,
            name="saddle still straddles the beam width at extension",
        )

    with ctx.pose(beam_swing=0.0, saddle_slide=0.0):
        retracted = ctx.part_world_position(saddle)
    with ctx.pose(beam_swing=0.0, saddle_slide=SLIDE_TRAVEL - 0.04):
        extended = ctx.part_world_position(saddle)

    slide_ok = False
    slide_details = "saddle world positions unavailable"
    if retracted is not None and extended is not None:
        dx = extended[0] - retracted[0]
        dy = extended[1] - retracted[1]
        dz = extended[2] - retracted[2]
        slide_ok = dx > 0.50 and abs(dy) < 0.01 and abs(dz) < 0.01
        slide_details = f"delta=({dx:.3f}, {dy:.3f}, {dz:.3f})"
    ctx.check("saddle translates outward along the beam", slide_ok, slide_details)

    with ctx.pose(beam_swing=0.0, saddle_slide=0.46):
        forward_pose = ctx.part_world_position(saddle)
    with ctx.pose(beam_swing=1.25, saddle_slide=0.46):
        swung_pose = ctx.part_world_position(saddle)

    swing_ok = False
    swing_details = "saddle world positions unavailable"
    if forward_pose is not None and swung_pose is not None:
        radius_forward = math.hypot(forward_pose[0], forward_pose[1])
        radius_swung = math.hypot(swung_pose[0], swung_pose[1])
        dy = swung_pose[1] - forward_pose[1]
        dz = swung_pose[2] - forward_pose[2]
        swing_ok = abs(radius_forward - radius_swung) < 0.02 and abs(dy) > 0.45 and abs(dz) < 0.01
        swing_details = (
            f"radius_forward={radius_forward:.3f}, radius_swung={radius_swung:.3f}, "
            f"dy={dy:.3f}, dz={dz:.3f}"
        )
    ctx.check("beam swing rotates the saddle around the column", swing_ok, swing_details)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
