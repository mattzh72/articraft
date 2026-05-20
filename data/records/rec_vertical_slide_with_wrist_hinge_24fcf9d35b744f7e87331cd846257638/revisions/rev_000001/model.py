from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


GANTRY_W = 0.22
GANTRY_D = 0.075
GANTRY_H = 0.07

RAIL_SPACING = 0.062
RAIL_W = 0.012
RAIL_D = 0.016
RAIL_L = 0.42
RAIL_CENTER_Z = -(GANTRY_H / 2.0 + RAIL_L / 2.0)

LOWER_TIE_W = 0.11
LOWER_TIE_D = 0.028
LOWER_TIE_H = 0.018
LOWER_TIE_CENTER_Z = -(GANTRY_H / 2.0 + RAIL_L + LOWER_TIE_H / 2.0)

WRIST_JOINT_Z = -0.082

STAGE_FORWARD_Y = 0.018
STAGE_PAD_W = 0.02
STAGE_PAD_D = 0.012
STAGE_PAD_H = 0.11
STAGE_PAD_Z = 0.012
STAGE_TOP_BRIDGE_W = 0.078
STAGE_TOP_BRIDGE_D = 0.01
STAGE_TOP_BRIDGE_H = 0.022
STAGE_TOP_BRIDGE_Z = 0.066
STAGE_SPINE_W = 0.026
STAGE_SPINE_D = 0.016
STAGE_SPINE_H = 0.09
STAGE_SPINE_Z = -0.025
STAGE_MID_BRIDGE_W = 0.05
STAGE_MID_BRIDGE_D = 0.01
STAGE_MID_BRIDGE_H = 0.018
STAGE_MID_BRIDGE_Z = 0.006
STAGE_REST_Z = RAIL_CENTER_Z

CHEEK_GAP = 0.014
CHEEK_T = 0.01
CHEEK_D = 0.014
CHEEK_H = 0.038

WRIST_HUB_R = 0.008
WRIST_HUB_BORE_R = 0.0038
WRIST_HUB_HALF_LEN = CHEEK_GAP * 0.42
HINGE_PIN_R = WRIST_HUB_BORE_R
HINGE_PIN_HALF_LEN = CHEEK_GAP / 2.0 + CHEEK_T
WRIST_FORWARD_Y = STAGE_FORWARD_Y
WRIST_ARM_W = 0.01
WRIST_ARM_D = 0.008
WRIST_ARM_H = 0.052
WRIST_ARM_Z = -0.038
WRIST_PAD_W = 0.012
WRIST_PAD_D = 0.012
WRIST_PAD_H = 0.008

TOOL_PLATE_W = 0.06
TOOL_PLATE_D = 0.034
TOOL_PLATE_T = 0.006
TOOL_PLATE_Z = -0.11

Z_TRAVEL_LOWER = -0.09
Z_TRAVEL_UPPER = 0.09
WRIST_LOWER = -1.05
WRIST_UPPER = 0.95


def _gantry_beam_shape():
    beam = cq.Workplane("XY").box(GANTRY_W, GANTRY_D, GANTRY_H)
    return beam.edges("|Z").fillet(0.006)


def _lower_tie_shape():
    tie = cq.Workplane("XY").box(LOWER_TIE_W, LOWER_TIE_D, LOWER_TIE_H)
    return tie.edges("|Z").fillet(0.003)


def _stage_body_shape():
    left_pad = cq.Workplane("XY").transformed(
        offset=(-RAIL_SPACING / 2.0, STAGE_FORWARD_Y - 0.004, STAGE_PAD_Z)
    ).box(STAGE_PAD_W, STAGE_PAD_D, STAGE_PAD_H)
    right_pad = cq.Workplane("XY").transformed(
        offset=(RAIL_SPACING / 2.0, STAGE_FORWARD_Y - 0.004, STAGE_PAD_Z)
    ).box(STAGE_PAD_W, STAGE_PAD_D, STAGE_PAD_H)
    top_bridge = cq.Workplane("XY").transformed(
        offset=(0.0, STAGE_FORWARD_Y, STAGE_TOP_BRIDGE_Z)
    ).box(STAGE_TOP_BRIDGE_W, STAGE_TOP_BRIDGE_D, STAGE_TOP_BRIDGE_H)
    mid_bridge = cq.Workplane("XY").transformed(
        offset=(0.0, STAGE_FORWARD_Y, STAGE_MID_BRIDGE_Z)
    ).box(STAGE_MID_BRIDGE_W, STAGE_MID_BRIDGE_D, STAGE_MID_BRIDGE_H)
    cheek_offset = CHEEK_GAP / 2.0 + CHEEK_T / 2.0
    left_cheek = cq.Workplane("XY").transformed(
        offset=(-cheek_offset, STAGE_FORWARD_Y, WRIST_JOINT_Z)
    ).box(CHEEK_T, CHEEK_D, CHEEK_H)
    right_cheek = cq.Workplane("XY").transformed(
        offset=(cheek_offset, STAGE_FORWARD_Y, WRIST_JOINT_Z)
    ).box(CHEEK_T, CHEEK_D, CHEEK_H)
    hinge_pin = (
        cq.Workplane("YZ")
        .circle(HINGE_PIN_R)
        .extrude(HINGE_PIN_HALF_LEN, both=True)
        .translate((0.0, STAGE_FORWARD_Y, WRIST_JOINT_Z))
    )
    return (
        left_pad.union(right_pad)
        .union(top_bridge)
        .union(mid_bridge)
        .union(left_cheek)
        .union(right_cheek)
        .union(hinge_pin)
    )


def _wrist_bracket_shape():
    hub = (
        cq.Workplane("YZ")
        .circle(WRIST_HUB_R)
        .extrude(WRIST_HUB_HALF_LEN, both=True)
        .translate((0.0, WRIST_FORWARD_Y, 0.0))
    )
    bore = (
        cq.Workplane("YZ")
        .circle(WRIST_HUB_BORE_R)
        .extrude(WRIST_HUB_HALF_LEN + 0.002, both=True)
        .translate((0.0, WRIST_FORWARD_Y, 0.0))
    )
    hub = hub.cut(bore)
    arm = cq.Workplane("XY").transformed(offset=(0.0, WRIST_FORWARD_Y, WRIST_ARM_Z)).box(
        WRIST_ARM_W, WRIST_ARM_D, WRIST_ARM_H
    )
    pad_z = TOOL_PLATE_Z + TOOL_PLATE_T / 2.0 + WRIST_PAD_H / 2.0
    pad = cq.Workplane("XY").transformed(offset=(0.0, WRIST_FORWARD_Y, pad_z)).box(
        WRIST_PAD_W, WRIST_PAD_D, WRIST_PAD_H
    )
    return hub.union(arm).union(pad)


def _tool_plate_shape():
    plate = cq.Workplane("XY").box(TOOL_PLATE_W, TOOL_PLATE_D, TOOL_PLATE_T)
    plate = plate.edges("|Z").fillet(0.003)
    return (
        plate.faces(">Z")
        .workplane()
        .pushPoints([(-0.014, 0.0), (0.014, 0.0)])
        .hole(0.004)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gantry_z_wrist")

    beam_gray = model.material("beam_gray", rgba=(0.64, 0.66, 0.69, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.82, 0.84, 0.86, 1.0))
    carriage_dark = model.material("carriage_dark", rgba=(0.17, 0.18, 0.2, 1.0))
    wrist_blue = model.material("wrist_blue", rgba=(0.33, 0.4, 0.48, 1.0))
    tool_black = model.material("tool_black", rgba=(0.08, 0.09, 0.11, 1.0))

    gantry = model.part("gantry")
    gantry.visual(
        mesh_from_cadquery(_gantry_beam_shape(), "gantry_beam"),
        material=beam_gray,
        name="gantry_beam",
    )
    gantry.visual(
        Box((RAIL_W, RAIL_D, RAIL_L)),
        origin=Origin(xyz=(-RAIL_SPACING / 2.0, 0.0, RAIL_CENTER_Z)),
        material=rail_steel,
        name="left_rail",
    )
    gantry.visual(
        Box((RAIL_W, RAIL_D, RAIL_L)),
        origin=Origin(xyz=(RAIL_SPACING / 2.0, 0.0, RAIL_CENTER_Z)),
        material=rail_steel,
        name="right_rail",
    )
    gantry.visual(
        mesh_from_cadquery(_lower_tie_shape(), "lower_tie"),
        origin=Origin(xyz=(0.0, 0.0, LOWER_TIE_CENTER_Z)),
        material=beam_gray,
        name="lower_tie",
    )

    stage = model.part("z_stage")
    stage.visual(
        mesh_from_cadquery(_stage_body_shape(), "z_stage_body"),
        material=carriage_dark,
        name="stage_body",
    )

    wrist = model.part("wrist")
    wrist.visual(
        mesh_from_cadquery(_wrist_bracket_shape(), "wrist_bracket"),
        material=wrist_blue,
        name="wrist_bracket",
    )
    wrist.visual(
        mesh_from_cadquery(_tool_plate_shape(), "tool_plate"),
        origin=Origin(xyz=(0.0, 0.0, TOOL_PLATE_Z)),
        material=tool_black,
        name="tool_plate",
    )

    model.articulation(
        "gantry_to_stage",
        ArticulationType.PRISMATIC,
        parent=gantry,
        child=stage,
        origin=Origin(xyz=(0.0, 0.0, STAGE_REST_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=0.35,
            lower=Z_TRAVEL_LOWER,
            upper=Z_TRAVEL_UPPER,
        ),
    )
    model.articulation(
        "stage_to_wrist",
        ArticulationType.REVOLUTE,
        parent=stage,
        child=wrist,
        origin=Origin(xyz=(0.0, 0.0, WRIST_JOINT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.5,
            lower=WRIST_LOWER,
            upper=WRIST_UPPER,
        ),
    )

    return model


def _center_from_aabb(aabb):
    if aabb is None:
        return None
    min_corner, max_corner = aabb
    return tuple((lo + hi) / 2.0 for lo, hi in zip(min_corner, max_corner))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    gantry = object_model.get_part("gantry")
    stage = object_model.get_part("z_stage")
    wrist = object_model.get_part("wrist")
    z_slide = object_model.get_articulation("gantry_to_stage")
    pitch = object_model.get_articulation("stage_to_wrist")

    ctx.allow_overlap(
        stage,
        wrist,
        elem_a="stage_body",
        elem_b="wrist_bracket",
        reason="Captured pitch hinge pin and wrist hub intentionally interpenetrate at the rotational bearing.",
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

    ctx.check(
        "z_carriage_joint_is_vertical_prismatic",
        z_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(float(v) for v in z_slide.axis) == (0.0, 0.0, 1.0),
        f"type={z_slide.articulation_type}, axis={z_slide.axis}",
    )
    ctx.check(
        "wrist_joint_is_horizontal_pitch_axis",
        pitch.articulation_type == ArticulationType.REVOLUTE
        and tuple(float(v) for v in pitch.axis) == (1.0, 0.0, 0.0),
        f"type={pitch.articulation_type}, axis={pitch.axis}",
    )

    ctx.expect_contact(stage, gantry, elem_b="left_rail", name="stage_contacts_left_rail")
    ctx.expect_contact(stage, gantry, elem_b="right_rail", name="stage_contacts_right_rail")
    ctx.expect_contact(wrist, stage, name="wrist_captured_by_stage_clevis")

    with ctx.pose({z_slide: z_slide.motion_limits.upper}):
        ctx.expect_gap(
            gantry,
            stage,
            axis="z",
            min_gap=0.01,
            max_gap=0.05,
            positive_elem="gantry_beam",
            negative_elem="stage_body",
            name="stage_clears_beam_at_upper_limit",
        )

    with ctx.pose({z_slide: z_slide.motion_limits.lower}):
        ctx.expect_gap(
            stage,
            gantry,
            axis="z",
            min_gap=0.015,
            max_gap=0.05,
            positive_elem="stage_body",
            negative_elem="lower_tie",
            name="stage_stays_above_lower_tie",
        )

    rest_stage_pos = ctx.part_world_position(stage)
    with ctx.pose({z_slide: z_slide.motion_limits.upper}):
        upper_stage_pos = ctx.part_world_position(stage)
    with ctx.pose({z_slide: z_slide.motion_limits.lower}):
        lower_stage_pos = ctx.part_world_position(stage)

    ctx.check(
        "prismatic_stage_moves_only_along_z",
        rest_stage_pos is not None
        and upper_stage_pos is not None
        and lower_stage_pos is not None
        and upper_stage_pos[2] > rest_stage_pos[2] + 0.05
        and lower_stage_pos[2] < rest_stage_pos[2] - 0.05
        and abs(upper_stage_pos[0] - rest_stage_pos[0]) < 1e-5
        and abs(upper_stage_pos[1] - rest_stage_pos[1]) < 1e-5
        and abs(lower_stage_pos[0] - rest_stage_pos[0]) < 1e-5
        and abs(lower_stage_pos[1] - rest_stage_pos[1]) < 1e-5,
        (
            f"rest={rest_stage_pos}, upper={upper_stage_pos}, "
            f"lower={lower_stage_pos}"
        ),
    )

    rest_plate = _center_from_aabb(ctx.part_element_world_aabb(wrist, elem="tool_plate"))
    with ctx.pose({pitch: 0.65}):
        pitched_plate = _center_from_aabb(ctx.part_element_world_aabb(wrist, elem="tool_plate"))

    ctx.check(
        "pitch_wrist_swings_tool_plate_in_yz_plane",
        rest_plate is not None
        and pitched_plate is not None
        and abs(pitched_plate[0] - rest_plate[0]) < 1e-5
        and abs(pitched_plate[1] - rest_plate[1]) > 0.01
        and abs(pitched_plate[2] - rest_plate[2]) > 0.01,
        f"rest={rest_plate}, pitched={pitched_plate}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
