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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


FRAME_W = 0.20
FRAME_H = 0.58
PLATE_T = 0.010
RAIL_W = 0.028
RAIL_D = 0.045
RAIL_H = 0.36
RAIL_X = 0.058

CARRIAGE_Y = 0.076
CARRIAGE_Z_CLOSED = -0.08
CARRIAGE_TRAVEL = 0.22

BRACKET_PITCH_LOWER = -0.35
BRACKET_PITCH_UPPER = 0.80

OUTPUT_SLIDE_TRAVEL = 0.12


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cyl_x(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length, both=True).translate(center)


def _make_back_frame() -> cq.Workplane:
    frame = _box((FRAME_W, PLATE_T, FRAME_H), (0.0, 0.0, 0.0))

    for x_pos in (-RAIL_X, RAIL_X):
        frame = frame.union(
            _box(
                (RAIL_W, RAIL_D, RAIL_H),
                (x_pos, PLATE_T / 2.0 + RAIL_D / 2.0, 0.0),
            )
        )

    frame = frame.union(_box((0.12, 0.040, 0.050), (0.0, 0.020, -0.23)))
    frame = frame.union(_box((0.13, 0.050, 0.065), (0.0, 0.025, 0.225)))
    frame = frame.union(_box((0.11, 0.016, 0.030), (0.0, 0.013, 0.0)))
    frame = frame.union(_box((0.10, 0.022, 0.032), (0.0, 0.018, 0.145)))
    frame = frame.union(_box((0.10, 0.022, 0.032), (0.0, 0.018, -0.145)))

    slot_cutter = (
        cq.Workplane("XZ")
        .pushPoints(
            [
                (-0.070, -0.205),
                (0.070, -0.205),
                (-0.070, 0.205),
                (0.070, 0.205),
            ]
        )
        .slot2D(0.040, 0.010, angle=90)
        .extrude(0.030, both=True)
    )
    return frame.cut(slot_cutter)


def _make_slide_carriage() -> cq.Workplane:
    carriage = _box((0.140, 0.026, 0.090), (0.0, -0.013, 0.0))
    carriage = carriage.union(_box((0.050, 0.012, 0.036), (0.0, -0.004, -0.012)))
    carriage = carriage.union(_box((0.050, 0.040, 0.050), (0.0, 0.000, 0.0)))

    for x_pos in (-RAIL_X, RAIL_X):
        carriage = carriage.union(_box((0.032, 0.018, 0.125), (x_pos, -0.017, 0.0)))
        carriage = carriage.union(_box((0.020, 0.020, 0.054), (x_pos, 0.000, 0.0)))
        carriage = carriage.union(_cyl_x(0.018, 0.020, (x_pos, 0.000, 0.0)))

    carriage = carriage.union(_box((0.070, 0.014, 0.020), (0.0, -0.015, -0.045)))
    return carriage


def _make_pivot_bracket() -> cq.Workplane:
    bracket = _cyl_x(0.018, 0.036, (0.0, -0.002, 0.0))
    bracket = bracket.union(_box((0.044, 0.036, 0.032), (0.0, 0.000, 0.0)))
    bracket = bracket.union(_box((0.034, 0.170, 0.034), (0.0, 0.085, 0.0)))
    cavity = _box((0.018, 0.150, 0.020), (0.0, 0.095, -0.004))
    bracket = bracket.cut(cavity)
    return bracket


def _make_output_member() -> cq.Workplane:
    output = _box((0.016, 0.110, 0.012), (0.0, 0.055, -0.008))
    output = output.union(_box((0.022, 0.040, 0.022), (0.0, 0.132, 0.001)))
    output = output.union(_box((0.064, 0.024, 0.056), (0.0, 0.164, 0.028)))
    return output


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_transfer_unit")

    model.material("frame_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("machined_silver", rgba=(0.71, 0.74, 0.78, 1.0))
    model.material("graphite", rgba=(0.34, 0.36, 0.39, 1.0))
    model.material("anodized_black", rgba=(0.10, 0.11, 0.12, 1.0))

    back_frame = model.part("back_frame")
    back_frame.visual(
        mesh_from_cadquery(_make_back_frame(), "back_frame"),
        material="frame_steel",
        name="frame_body",
    )
    back_frame.inertial = Inertial.from_geometry(
        Box((FRAME_W, 0.080, FRAME_H)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.020, 0.0)),
    )

    slide_carriage = model.part("slide_carriage")
    slide_carriage.visual(
        mesh_from_cadquery(_make_slide_carriage(), "slide_carriage"),
        material="machined_silver",
        name="carriage_body",
    )
    slide_carriage.inertial = Inertial.from_geometry(
        Box((0.140, 0.052, 0.125)),
        mass=2.8,
        origin=Origin(xyz=(0.0, -0.010, 0.0)),
    )

    pivot_bracket = model.part("pivot_bracket")
    pivot_bracket.visual(
        Cylinder(radius=0.018, length=0.044),
        origin=Origin(rpy=(0.0, 1.5707963267948966, 0.0)),
        material="graphite",
        name="hinge_barrel",
    )
    pivot_bracket.visual(
        Box((0.044, 0.032, 0.032)),
        origin=Origin(xyz=(0.0, 0.000, 0.0)),
        material="graphite",
        name="root_block",
    )
    pivot_bracket.visual(
        Box((0.028, 0.028, 0.014)),
        origin=Origin(xyz=(0.0, 0.030, 0.009)),
        material="graphite",
        name="throat_bridge",
    )
    pivot_bracket.visual(
        Box((0.006, 0.150, 0.032)),
        origin=Origin(xyz=(-0.011, 0.105, 0.0)),
        material="graphite",
        name="left_cheek",
    )
    pivot_bracket.visual(
        Box((0.006, 0.150, 0.032)),
        origin=Origin(xyz=(0.011, 0.105, 0.0)),
        material="graphite",
        name="right_cheek",
    )
    pivot_bracket.visual(
        Box((0.028, 0.150, 0.006)),
        origin=Origin(xyz=(0.0, 0.105, 0.013)),
        material="graphite",
        name="top_rail",
    )
    pivot_bracket.inertial = Inertial.from_geometry(
        Box((0.070, 0.190, 0.060)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.095, -0.008)),
    )

    output_member = model.part("output_member")
    output_member.visual(
        Box((0.014, 0.260, 0.006)),
        origin=Origin(xyz=(0.0, 0.130, 0.010)),
        material="anodized_black",
        name="slide_bar",
    )
    output_member.visual(
        Box((0.022, 0.036, 0.020)),
        origin=Origin(xyz=(0.0, 0.230, 0.010)),
        material="anodized_black",
        name="nose_mount",
    )
    output_member.visual(
        Box((0.064, 0.028, 0.056)),
        origin=Origin(xyz=(0.0, 0.262, 0.032)),
        material="anodized_black",
        name="transfer_head",
    )
    output_member.inertial = Inertial.from_geometry(
        Box((0.070, 0.135, 0.070)),
        mass=1.0,
        origin=Origin(xyz=(0.0, 0.070, 0.006)),
    )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=back_frame,
        child=slide_carriage,
        origin=Origin(xyz=(0.0, CARRIAGE_Y, CARRIAGE_Z_CLOSED)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=CARRIAGE_TRAVEL,
            effort=900.0,
            velocity=0.30,
        ),
    )

    model.articulation(
        "carriage_to_bracket",
        ArticulationType.REVOLUTE,
        parent=slide_carriage,
        child=pivot_bracket,
        origin=Origin(xyz=(0.0, 0.036, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=BRACKET_PITCH_LOWER,
            upper=BRACKET_PITCH_UPPER,
            effort=55.0,
            velocity=1.8,
        ),
    )

    model.articulation(
        "bracket_to_output",
        ArticulationType.PRISMATIC,
        parent=pivot_bracket,
        child=output_member,
        origin=Origin(xyz=(0.0, 0.040, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=OUTPUT_SLIDE_TRAVEL,
            effort=180.0,
            velocity=0.20,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    back_frame = object_model.get_part("back_frame")
    slide_carriage = object_model.get_part("slide_carriage")
    pivot_bracket = object_model.get_part("pivot_bracket")
    output_member = object_model.get_part("output_member")

    frame_to_carriage = object_model.get_articulation("frame_to_carriage")
    carriage_to_bracket = object_model.get_articulation("carriage_to_bracket")
    bracket_to_output = object_model.get_articulation("bracket_to_output")

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
        "joint_stack_matches_prompt",
        frame_to_carriage.articulation_type == ArticulationType.PRISMATIC
        and carriage_to_bracket.articulation_type == ArticulationType.REVOLUTE
        and bracket_to_output.articulation_type == ArticulationType.PRISMATIC,
        "Expected prismatic -> revolute -> prismatic chain from frame to output.",
    )

    ctx.expect_contact(back_frame, slide_carriage, name="frame_carriage_supported_contact")
    ctx.expect_contact(slide_carriage, pivot_bracket, name="carriage_bracket_supported_contact")
    ctx.expect_contact(pivot_bracket, output_member, name="bracket_output_supported_contact")

    with ctx.pose({frame_to_carriage: CARRIAGE_TRAVEL}):
        ctx.expect_contact(
            back_frame,
            slide_carriage,
            name="frame_carriage_contact_at_top_of_travel",
        )

    with ctx.pose({carriage_to_bracket: 0.55, bracket_to_output: OUTPUT_SLIDE_TRAVEL}):
        ctx.expect_contact(
            pivot_bracket,
            output_member,
            name="bracket_output_contact_when_pitched_and_extended",
        )

    closed_carriage_pos = ctx.part_world_position(slide_carriage)
    with ctx.pose({frame_to_carriage: CARRIAGE_TRAVEL}):
        raised_carriage_pos = ctx.part_world_position(slide_carriage)

    if closed_carriage_pos is None or raised_carriage_pos is None:
        ctx.fail("carriage_lifts_upward", "Could not resolve slide carriage world positions.")
    else:
        ctx.check(
            "carriage_lifts_upward",
            raised_carriage_pos[2] > closed_carriage_pos[2] + 0.20,
            (
                f"Expected >0.20 m upward travel, got "
                f"{raised_carriage_pos[2] - closed_carriage_pos[2]:.4f} m."
            ),
        )

    closed_output_pos = ctx.part_world_position(output_member)
    with ctx.pose({carriage_to_bracket: 0.55}):
        pitched_output_pos = ctx.part_world_position(output_member)

    if closed_output_pos is None or pitched_output_pos is None:
        ctx.fail("nose_stage_pitches_upward", "Could not resolve output-member world positions.")
    else:
        ctx.check(
            "nose_stage_pitches_upward",
            pitched_output_pos[2] > closed_output_pos[2] + 0.02,
            (
                f"Expected the pitched nose stage to lift the distal slide by >0.02 m, got "
                f"{pitched_output_pos[2] - closed_output_pos[2]:.4f} m."
            ),
        )

    with ctx.pose({bracket_to_output: OUTPUT_SLIDE_TRAVEL}):
        extended_output_pos = ctx.part_world_position(output_member)

    if closed_output_pos is None or extended_output_pos is None:
        ctx.fail("output_member_extends_forward", "Could not resolve distal-slide world positions.")
    else:
        ctx.check(
            "output_member_extends_forward",
            extended_output_pos[1] > closed_output_pos[1] + 0.11,
            (
                f"Expected >0.11 m forward extension, got "
                f"{extended_output_pos[1] - closed_output_pos[1]:.4f} m."
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
