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


BASE_LENGTH = 1.00
BASE_WIDTH = 0.42
BASE_THICKNESS = 0.045

SIDE_PLATE_THICKNESS = 0.030
SIDE_PLATE_DEPTH = 0.115
SIDE_PLATE_HEIGHT = 0.310

BEAM_LENGTH = 0.780
BEAM_DEPTH = 0.094
BEAM_HEIGHT = 0.074
BEAM_CENTER_Z = 0.337

BEAM_RAIL_LENGTH = 0.700
BEAM_RAIL_DEPTH = 0.012
BEAM_RAIL_HEIGHT = 0.013
BEAM_RAIL_UPPER_Z = BEAM_CENTER_Z + 0.018
BEAM_RAIL_LOWER_Z = BEAM_CENTER_Z - 0.018
BEAM_RAIL_OUTER_Y = (BEAM_DEPTH / 2.0) + BEAM_RAIL_DEPTH

SIDE_PLATE_CENTER_X = (BEAM_LENGTH / 2.0) - (SIDE_PLATE_THICKNESS / 2.0)

BRIDGE_CARRIAGE_LENGTH = 0.170
BRIDGE_CARRIAGE_DEPTH = 0.060
BRIDGE_CARRIAGE_HEIGHT = 0.120
HANGER_WIDTH = 0.112
HANGER_DEPTH = 0.043
HANGER_HEIGHT = 0.142
HANGER_CENTER_Z = -0.111
Z_RAIL_WIDTH = 0.048
Z_RAIL_DEPTH = 0.012
Z_RAIL_HEIGHT = 0.130
Z_RAIL_CENTER_Y = 0.050
Z_RAIL_CENTER_Z = -0.065
Z_RAIL_FRONT_Y = Z_RAIL_CENTER_Y + (Z_RAIL_DEPTH / 2.0)

STAGE_SLIDER_WIDTH = 0.050
STAGE_SLIDER_DEPTH = 0.012
STAGE_SLIDER_HEIGHT = 0.120
STAGE_BODY_WIDTH = 0.078
STAGE_BODY_DEPTH = 0.044
STAGE_BODY_HEIGHT = 0.046
STAGE_NOSE_WIDTH = 0.048
STAGE_NOSE_DEPTH = 0.038
STAGE_NOSE_HEIGHT = 0.022
STAGE_PAD_WIDTH = 0.060
STAGE_PAD_DEPTH = 0.012
STAGE_PAD_HEIGHT = 0.010

BRIDGE_TRAVEL = 0.230
STAGE_TRAVEL = 0.085


def _portal_uprights_shape() -> cq.Workplane:
    left_plate = (
        cq.Workplane("XY")
        .box(
            SIDE_PLATE_THICKNESS,
            SIDE_PLATE_DEPTH,
            SIDE_PLATE_HEIGHT,
            centered=(True, True, False),
        )
        .translate((-SIDE_PLATE_CENTER_X, 0.0, BASE_THICKNESS))
    )
    right_plate = (
        cq.Workplane("XY")
        .box(
            SIDE_PLATE_THICKNESS,
            SIDE_PLATE_DEPTH,
            SIDE_PLATE_HEIGHT,
            centered=(True, True, False),
        )
        .translate((SIDE_PLATE_CENTER_X, 0.0, BASE_THICKNESS))
    )
    beam = (
        cq.Workplane("XY")
        .box(BEAM_LENGTH, BEAM_DEPTH, BEAM_HEIGHT)
        .translate((0.0, 0.0, BEAM_CENTER_Z))
    )

    portal = left_plate.union(right_plate).union(beam)

    window_height = 0.170
    window_depth = 0.060
    window_bottom_z = BASE_THICKNESS + 0.074
    for sign in (-1.0, 1.0):
        plate_window = (
            cq.Workplane("XY")
            .box(
                SIDE_PLATE_THICKNESS * 2.5,
                window_depth,
                window_height,
                centered=(True, True, False),
            )
            .translate((sign * SIDE_PLATE_CENTER_X, 0.0, window_bottom_z))
        )
        portal = portal.cut(plate_window)

    return portal


def _bridge_carriage_shape() -> cq.Workplane:
    back_plate = (
        cq.Workplane("XY")
        .box(0.170, 0.018, 0.150, centered=(True, False, True))
    )
    top_cap = (
        cq.Workplane("XY")
        .box(0.152, 0.040, 0.040, centered=(True, False, True))
        .translate((0.0, 0.010, 0.048))
    )
    left_cheek = (
        cq.Workplane("XY")
        .box(0.028, 0.032, 0.088, centered=(True, False, True))
        .translate((-0.038, 0.016, -0.082))
    )
    right_cheek = (
        cq.Workplane("XY")
        .box(0.028, 0.032, 0.088, centered=(True, False, True))
        .translate((0.038, 0.016, -0.082))
    )
    lower_bridge = (
        cq.Workplane("XY")
        .box(0.094, 0.020, 0.026, centered=(True, False, True))
        .translate((0.0, 0.010, -0.137))
    )
    return back_plate.union(top_cap).union(left_cheek).union(right_cheek).union(lower_bridge)


def _vertical_stage_shape() -> cq.Workplane:
    slider = (
        cq.Workplane("XY")
        .box(
            STAGE_SLIDER_WIDTH,
            STAGE_SLIDER_DEPTH,
            STAGE_SLIDER_HEIGHT,
            centered=(True, False, True),
        )
        .translate((0.0, 0.0, -(STAGE_SLIDER_HEIGHT / 2.0)))
    )
    body = (
        cq.Workplane("XY")
        .box(
            STAGE_BODY_WIDTH,
            STAGE_BODY_DEPTH,
            STAGE_BODY_HEIGHT,
            centered=(True, False, True),
        )
        .translate((0.0, 0.0, -0.141))
    )
    nose = (
        cq.Workplane("XY")
        .box(
            STAGE_NOSE_WIDTH,
            STAGE_NOSE_DEPTH,
            STAGE_NOSE_HEIGHT,
            centered=(True, False, True),
        )
        .translate((0.0, 0.0, -0.174))
    )
    tool_pad = (
        cq.Workplane("XY")
        .box(
            STAGE_PAD_WIDTH,
            STAGE_PAD_DEPTH,
            STAGE_PAD_HEIGHT,
            centered=(True, False, True),
        )
        .translate((0.0, 0.0, -0.189))
    )
    return slider.union(body).union(nose).union(tool_pad)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portal_module")

    model.material("frame_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("frame_gray", rgba=(0.69, 0.71, 0.74, 1.0))
    model.material("rail_steel", rgba=(0.55, 0.58, 0.62, 1.0))
    model.material("carriage_black", rgba=(0.12, 0.13, 0.15, 1.0))
    model.material("stage_gray", rgba=(0.80, 0.82, 0.84, 1.0))
    model.material("accent_blue", rgba=(0.18, 0.36, 0.69, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material="frame_dark",
        name="base_deck",
    )
    frame.visual(
        mesh_from_cadquery(_portal_uprights_shape(), "portal_uprights"),
        material="frame_gray",
        name="portal_uprights",
    )
    frame.visual(
        Box((BEAM_RAIL_LENGTH, BEAM_RAIL_DEPTH, BEAM_RAIL_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                (BEAM_DEPTH / 2.0) + (BEAM_RAIL_DEPTH / 2.0),
                BEAM_RAIL_UPPER_Z,
            )
        ),
        material="rail_steel",
        name="upper_beam_rail",
    )
    frame.visual(
        Box((BEAM_RAIL_LENGTH, BEAM_RAIL_DEPTH, BEAM_RAIL_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                (BEAM_DEPTH / 2.0) + (BEAM_RAIL_DEPTH / 2.0),
                BEAM_RAIL_LOWER_Z,
            )
        ),
        material="rail_steel",
        name="lower_beam_rail",
    )

    bridge_carriage = model.part("bridge_carriage")
    bridge_carriage.visual(
        mesh_from_cadquery(_bridge_carriage_shape(), "bridge_carriage"),
        material="carriage_black",
        name="carriage_body",
    )
    bridge_carriage.visual(
        Box((Z_RAIL_WIDTH, Z_RAIL_DEPTH, Z_RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, Z_RAIL_CENTER_Y, Z_RAIL_CENTER_Z)),
        material="rail_steel",
        name="z_rail",
    )

    vertical_stage = model.part("vertical_stage")
    vertical_stage.visual(
        mesh_from_cadquery(_vertical_stage_shape(), "vertical_stage"),
        material="stage_gray",
        name="stage_body",
    )
    vertical_stage.visual(
        Box((0.070, 0.016, 0.012)),
        origin=Origin(xyz=(0.0, 0.052, -0.170)),
        material="accent_blue",
        name="tool_edge",
    )

    model.articulation(
        "frame_to_bridge_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=bridge_carriage,
        origin=Origin(xyz=(0.0, BEAM_RAIL_OUTER_Y, BEAM_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-BRIDGE_TRAVEL,
            upper=BRIDGE_TRAVEL,
            effort=500.0,
            velocity=0.80,
        ),
    )
    model.articulation(
        "bridge_carriage_to_vertical_stage",
        ArticulationType.PRISMATIC,
        parent=bridge_carriage,
        child=vertical_stage,
        origin=Origin(xyz=(0.0, Z_RAIL_FRONT_Y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=STAGE_TRAVEL,
            effort=280.0,
            velocity=0.40,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    bridge_carriage = object_model.get_part("bridge_carriage")
    vertical_stage = object_model.get_part("vertical_stage")
    beam_slide = object_model.get_articulation("frame_to_bridge_carriage")
    z_slide = object_model.get_articulation("bridge_carriage_to_vertical_stage")

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

    with ctx.pose({beam_slide: 0.0, z_slide: 0.0}):
        ctx.expect_contact(
            bridge_carriage,
            frame,
            name="bridge carriage is supported by the beam rails",
        )
        ctx.expect_contact(
            vertical_stage,
            bridge_carriage,
            name="vertical stage is supported by the hanging carriage rail",
        )

    with ctx.pose({beam_slide: BRIDGE_TRAVEL, z_slide: STAGE_TRAVEL}):
        ctx.expect_contact(
            bridge_carriage,
            frame,
            name="bridge carriage stays engaged at full beam travel",
        )
        ctx.expect_contact(
            vertical_stage,
            bridge_carriage,
            name="vertical stage stays engaged at full drop",
        )
        ctx.expect_gap(
            vertical_stage,
            frame,
            axis="z",
            min_gap=0.010,
            negative_elem="base_deck",
            name="vertical stage clears the fixed base at full drop",
        )

    with ctx.pose({beam_slide: -BRIDGE_TRAVEL, z_slide: 0.0}):
        left_pos = ctx.part_world_position(bridge_carriage)
    with ctx.pose({beam_slide: BRIDGE_TRAVEL, z_slide: 0.0}):
        right_pos = ctx.part_world_position(bridge_carriage)
    carriage_motion_ok = (
        left_pos is not None
        and right_pos is not None
        and right_pos[0] > left_pos[0] + (2.0 * BRIDGE_TRAVEL * 0.95)
        and abs(right_pos[2] - left_pos[2]) < 1e-6
    )
    ctx.check(
        "bridge carriage translates horizontally on the beam",
        carriage_motion_ok,
        details=f"left={left_pos}, right={right_pos}",
    )

    with ctx.pose({beam_slide: 0.0, z_slide: 0.0}):
        stage_home = ctx.part_world_position(vertical_stage)
    with ctx.pose({beam_slide: 0.0, z_slide: STAGE_TRAVEL}):
        stage_drop = ctx.part_world_position(vertical_stage)
    stage_motion_ok = (
        stage_home is not None
        and stage_drop is not None
        and stage_drop[2] < stage_home[2] - (STAGE_TRAVEL * 0.95)
        and abs(stage_drop[0] - stage_home[0]) < 1e-6
    )
    ctx.check(
        "vertical stage drops on positive travel",
        stage_motion_ok,
        details=f"home={stage_home}, drop={stage_drop}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
