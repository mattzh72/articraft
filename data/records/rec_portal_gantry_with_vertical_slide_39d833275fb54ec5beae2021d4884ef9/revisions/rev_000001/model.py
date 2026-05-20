from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


def _build_portal_frame_body() -> cq.Workplane:
    base_w = 0.56
    base_d = 0.34
    base_t = 0.03

    leg_t = 0.07
    leg_d = 0.20
    leg_h = 0.34
    inner_w = 0.38
    leg_x = inner_w / 2.0 + leg_t / 2.0

    beam_len = 0.44
    beam_d = 0.085
    beam_h = 0.09
    beam_z = 0.365

    body = cq.Workplane("XY").box(base_w, base_d, base_t)

    for sign in (-1.0, 1.0):
        leg = (
            cq.Workplane("XY")
            .box(leg_t, leg_d, leg_h)
            .translate((sign * leg_x, 0.0, base_t / 2.0 + leg_h / 2.0))
            .edges("|Z")
            .fillet(0.006)
        )
        body = body.union(leg)

    beam = (
        cq.Workplane("XY")
        .box(beam_len, beam_d, beam_h)
        .translate((0.0, 0.0, beam_z))
        .edges("|X")
        .fillet(0.008)
    )
    body = body.union(beam)

    for sign in (-1.0, 1.0):
        gusset = (
            cq.Workplane("XY")
            .box(0.018, 0.13, 0.19)
            .translate((sign * 0.200, 0.0, 0.105))
            .edges("|X")
            .fillet(0.01)
        )
        body = body.union(gusset)

    return body


def _build_crosshead_body() -> cq.Workplane:
    width = 0.14
    depth = 0.046
    height = 0.14

    body = (
        cq.Workplane("XY")
        .box(width, depth, height)
        .translate((0.0, 0.0145, 0.0))
        .edges("|Y")
        .fillet(0.006)
    )

    body = (
        body.faces(">Y")
        .workplane()
        .rect(0.084, 0.082)
        .cutBlind(0.018)
    )

    chin = (
        cq.Workplane("XY")
        .box(0.082, 0.032, 0.05)
        .translate((0.0, 0.0305, -0.043))
        .edges("|Y")
        .fillet(0.004)
    )
    return body.union(chin)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tabletop_portal_axis")

    frame_mat = model.material("frame_gray", rgba=(0.72, 0.74, 0.77, 1.0))
    rail_mat = model.material("rail_dark", rgba=(0.24, 0.26, 0.29, 1.0))
    carriage_mat = model.material("carriage_gray", rgba=(0.62, 0.64, 0.67, 1.0))
    quill_mat = model.material("quill_aluminum", rgba=(0.83, 0.84, 0.86, 1.0))
    spindle_mat = model.material("spindle_black", rgba=(0.10, 0.10, 0.12, 1.0))

    beam_z = 0.365
    beam_d = 0.085
    beam_front = beam_d / 2.0

    frame = model.part("portal_frame")
    frame.visual(
        mesh_from_cadquery(_build_portal_frame_body(), "portal_frame_body"),
        material=frame_mat,
        name="frame_body",
    )
    frame.visual(
        Box((0.36, 0.014, 0.018)),
        origin=Origin(xyz=(0.0, beam_front + 0.007, beam_z + 0.028)),
        material=rail_mat,
        name="beam_upper_rail",
    )
    frame.visual(
        Box((0.36, 0.014, 0.018)),
        origin=Origin(xyz=(0.0, beam_front + 0.007, beam_z - 0.028)),
        material=rail_mat,
        name="beam_lower_rail",
    )

    crosshead = model.part("crosshead")
    crosshead.visual(
        mesh_from_cadquery(_build_crosshead_body(), "crosshead_body"),
        material=carriage_mat,
        name="carriage_body",
    )
    crosshead.visual(
        Box((0.14, 0.015, 0.042)),
        origin=Origin(xyz=(0.0, -0.016, 0.028)),
        material=rail_mat,
        name="upper_truck",
    )
    crosshead.visual(
        Box((0.14, 0.015, 0.042)),
        origin=Origin(xyz=(0.0, -0.016, -0.028)),
        material=rail_mat,
        name="lower_truck",
    )
    crosshead.visual(
        Box((0.018, 0.012, 0.17)),
        origin=Origin(xyz=(-0.038, 0.0435, -0.02)),
        material=rail_mat,
        name="left_quill_rail",
    )
    crosshead.visual(
        Box((0.018, 0.012, 0.17)),
        origin=Origin(xyz=(0.038, 0.0435, -0.02)),
        material=rail_mat,
        name="right_quill_rail",
    )

    quill = model.part("quill")
    quill.visual(
        Box((0.11, 0.026, 0.16)),
        origin=Origin(xyz=(0.0, 0.0035, -0.08)),
        material=quill_mat,
        name="quill_plate",
    )
    quill.visual(
        Box((0.072, 0.032, 0.03)),
        origin=Origin(xyz=(0.0, 0.004, -0.015)),
        material=carriage_mat,
        name="quill_head",
    )
    quill.visual(
        Box((0.022, 0.015, 0.055)),
        origin=Origin(xyz=(-0.038, -0.017, -0.035)),
        material=rail_mat,
        name="left_slider",
    )
    quill.visual(
        Box((0.022, 0.015, 0.055)),
        origin=Origin(xyz=(0.038, -0.017, -0.035)),
        material=rail_mat,
        name="right_slider",
    )
    quill.visual(
        Box((0.05, 0.04, 0.10)),
        origin=Origin(xyz=(0.0, 0.004, -0.11)),
        material=spindle_mat,
        name="spindle_cartridge",
    )
    quill.visual(
        Cylinder(radius=0.018, length=0.05),
        origin=Origin(xyz=(0.0, 0.004, -0.185)),
        material=spindle_mat,
        name="spindle_nose",
    )

    model.articulation(
        "frame_to_crosshead",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=crosshead,
        origin=Origin(xyz=(0.0, 0.08, beam_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.30,
            lower=-0.11,
            upper=0.11,
        ),
    )
    model.articulation(
        "crosshead_to_quill",
        ArticulationType.PRISMATIC,
        parent=crosshead,
        child=quill,
        origin=Origin(xyz=(0.0, 0.074, 0.045)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.18,
            lower=0.0,
            upper=0.09,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("portal_frame")
    crosshead = object_model.get_part("crosshead")
    quill = object_model.get_part("quill")
    x_axis = object_model.get_articulation("frame_to_crosshead")
    z_axis = object_model.get_articulation("crosshead_to_quill")

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

    ctx.expect_gap(
        crosshead,
        frame,
        axis="y",
        positive_elem="upper_truck",
        negative_elem="beam_upper_rail",
        max_gap=0.0015,
        max_penetration=0.0,
        name="upper_truck_seats_on_upper_beam_rail",
    )
    ctx.expect_gap(
        crosshead,
        frame,
        axis="y",
        positive_elem="lower_truck",
        negative_elem="beam_lower_rail",
        max_gap=0.0015,
        max_penetration=0.0,
        name="lower_truck_seats_on_lower_beam_rail",
    )
    ctx.expect_gap(
        quill,
        crosshead,
        axis="y",
        positive_elem="left_slider",
        negative_elem="left_quill_rail",
        max_gap=0.0015,
        max_penetration=0.0,
        name="left_slider_seats_on_left_quill_rail",
    )
    ctx.expect_gap(
        quill,
        crosshead,
        axis="y",
        positive_elem="right_slider",
        negative_elem="right_quill_rail",
        max_gap=0.0015,
        max_penetration=0.0,
        name="right_slider_seats_on_right_quill_rail",
    )
    ctx.expect_overlap(
        quill,
        crosshead,
        axes="x",
        elem_a="quill_plate",
        elem_b="carriage_body",
        min_overlap=0.09,
        name="quill_stays_centered_under_crosshead",
    )

    parked_crosshead = ctx.part_world_position(crosshead)
    parked_quill = ctx.part_world_position(quill)
    with ctx.pose({x_axis: 0.10}):
        right_crosshead = ctx.part_world_position(crosshead)
    with ctx.pose({z_axis: 0.07}):
        lowered_quill = ctx.part_world_position(quill)

    ctx.check(
        "positive_x_travel_moves_crosshead_right",
        parked_crosshead is not None
        and right_crosshead is not None
        and right_crosshead[0] > parked_crosshead[0] + 0.09,
        details=(
            f"parked={parked_crosshead}, right={right_crosshead}; "
            "expected positive prismatic travel to increase X."
        ),
    )
    ctx.check(
        "positive_quill_travel_moves_down",
        parked_quill is not None
        and lowered_quill is not None
        and lowered_quill[2] < parked_quill[2] - 0.06,
        details=(
            f"parked={parked_quill}, lowered={lowered_quill}; "
            "expected positive quill travel to reduce Z."
        ),
    )

    with ctx.pose({x_axis: 0.11, z_axis: 0.09}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_full_right_and_down")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
