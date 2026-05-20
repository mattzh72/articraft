from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


FRAME_WIDTH = 0.78
FRAME_HEIGHT = 0.92
FRAME_BORDER = 0.055
WALL_THICK = 0.018

SIDE_BRACKET_WIDTH = 0.075
SIDE_BRACKET_DEPTH = 0.082
SIDE_BRACKET_HEIGHT = 0.24
FRAME_CROSS_BEAM_WIDTH = 0.68
FRAME_CROSS_BEAM_DEPTH = 0.032
FRAME_CROSS_BEAM_HEIGHT = 0.10

H_RAIL_LENGTH = 0.58
H_RAIL_RADIUS = 0.009
H_RAIL_Y = 0.082
H_RAIL_Z = 0.055
H_GUIDE_END_WIDTH = 0.085
H_GUIDE_END_DEPTH = 0.088
H_GUIDE_END_HEIGHT = 0.062
H_GUIDE_INNER_X = H_RAIL_LENGTH / 2.0 - H_GUIDE_END_WIDTH / 2.0 + 0.015

X_TRAVEL = 0.17
X_SLEEVE_LENGTH = 0.086
X_SLEEVE_RADIUS = 0.021
X_BRIDGE_WIDTH = 0.20
X_BRIDGE_THICK = 0.022
X_BRIDGE_HEIGHT = 0.18
X_CORE_WIDTH = 0.11
X_CORE_DEPTH = 0.032
X_CORE_HEIGHT = 0.085

MAST_PLATE_WIDTH = 0.18
MAST_PLATE_THICK = 0.016
MAST_HEIGHT = 0.60
MAST_PLATE_Y = 0.052
MAST_CAP_WIDTH = 0.14
MAST_CAP_DEPTH = 0.030
MAST_CAP_HEIGHT = 0.030
MAST_RISER_WIDTH = 0.070
MAST_RISER_DEPTH = 0.036
MAST_RISER_HEIGHT = 0.15
MAST_GUIDE_BACKPLATE_DEPTH = 0.018

V_RAIL_RADIUS = 0.008
V_RAIL_LENGTH = 0.48
V_RAIL_X = 0.045
V_RAIL_Y = 0.123
V_RAIL_CENTER_Z = 0.23

Z_TRAVEL = 0.18
Z_STAGE_LOW_CENTER_Z = 0.10
Z_SLEEVE_LENGTH = 0.10
Z_SLEEVE_RADIUS = 0.019
Z_PLATE_WIDTH = 0.16
Z_PLATE_THICK = 0.016
Z_PLATE_HEIGHT = 0.22
Z_CORE_WIDTH = 0.092
Z_CORE_DEPTH = 0.028
Z_CORE_HEIGHT = 0.115
TOOL_PAD_WIDTH = 0.132
TOOL_PAD_DEPTH = 0.040
TOOL_PAD_HEIGHT = 0.052


def _cx_box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _x_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length, both=True).translate(center)


def _z_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(length, both=True).translate(center)


def _back_frame_body() -> cq.Workplane:
    frame_ring = _cx_box((FRAME_WIDTH, WALL_THICK, FRAME_HEIGHT), (0.0, 0.0, 0.0)).cut(
        _cx_box(
            (FRAME_WIDTH - 2.0 * FRAME_BORDER, WALL_THICK + 0.004, FRAME_HEIGHT - 2.0 * FRAME_BORDER),
            (0.0, 0.0, 0.0),
        )
    )

    body = frame_ring.union(
        _cx_box(
            (FRAME_CROSS_BEAM_WIDTH, 0.038, 0.14),
            (0.0, 0.020, 0.0),
        )
    )

    body = body.union(_cx_box((0.080, 0.070, 0.16), (0.0, 0.040, 0.0)))
    body = body.union(_cx_box((H_RAIL_LENGTH, 0.016, 0.022), (0.0, H_RAIL_Y, H_RAIL_Z)))
    body = body.union(_cx_box((H_RAIL_LENGTH, 0.016, 0.022), (0.0, H_RAIL_Y, -H_RAIL_Z)))

    side_block_x = H_RAIL_LENGTH / 2.0 - 0.042
    for x_sign in (-1.0, 1.0):
        body = body.union(
            _cx_box(
                (0.084, 0.086, 0.132),
                (x_sign * side_block_x, 0.043, 0.0),
            )
        )

    return body


def _x_carriage_body() -> cq.Workplane:
    body = _cx_box((0.22, 0.022, 0.20), (0.0, 0.072, 0.0))
    body = body.union(_cx_box((0.08, 0.038, 0.16), (0.0, 0.085, 0.10)))
    body = body.union(_cx_box((MAST_PLATE_WIDTH, MAST_PLATE_THICK, MAST_HEIGHT), (0.0, 0.108, V_RAIL_CENTER_Z)))

    for z_center in (-H_RAIL_Z, H_RAIL_Z):
        body = body.union(_cx_box((0.14, 0.020, 0.034), (0.0, 0.018, z_center)))
        body = body.union(_cx_box((0.060, 0.034, 0.050), (0.0, 0.045, z_center)))

    for x_center in (-V_RAIL_X, V_RAIL_X):
        body = body.union(_cx_box((0.018, 0.014, V_RAIL_LENGTH), (x_center, V_RAIL_Y, V_RAIL_CENTER_Z)))

    for z_center in (
        V_RAIL_CENTER_Z - V_RAIL_LENGTH / 2.0 + MAST_CAP_HEIGHT / 2.0,
        V_RAIL_CENTER_Z + V_RAIL_LENGTH / 2.0 - MAST_CAP_HEIGHT / 2.0,
    ):
        body = body.union(_cx_box((MAST_CAP_WIDTH, 0.020, MAST_CAP_HEIGHT), (0.0, V_RAIL_Y, z_center)))

    return body


def _z_stage_body() -> cq.Workplane:
    body = _cx_box((Z_PLATE_WIDTH, Z_PLATE_THICK, Z_PLATE_HEIGHT), (0.0, 0.055, 0.060))
    body = body.union(_cx_box((Z_CORE_WIDTH, 0.032, 0.085), (0.0, 0.037, 0.055)))
    body = body.union(_cx_box((TOOL_PAD_WIDTH, TOOL_PAD_DEPTH, TOOL_PAD_HEIGHT), (0.0, 0.082, -0.060)))

    for x_center in (-V_RAIL_X, V_RAIL_X):
        body = body.union(_cx_box((0.032, 0.018, 0.120), (x_center, 0.016, 0.130)))
        body = body.union(_cx_box((0.050, 0.032, 0.100), (x_center, 0.035, 0.080)))

    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_backed_xz_positioning_module")

    model.material("frame_charcoal", rgba=(0.19, 0.21, 0.23, 1.0))
    model.material("rail_steel", rgba=(0.71, 0.73, 0.77, 1.0))
    model.material("carriage_graphite", rgba=(0.28, 0.31, 0.35, 1.0))
    model.material("stage_aluminum", rgba=(0.83, 0.85, 0.88, 1.0))

    back_frame = model.part("back_frame")
    back_frame.visual(
        mesh_from_cadquery(_back_frame_body(), "back_frame_body"),
        material="frame_charcoal",
        name="frame_body",
    )

    x_carriage = model.part("x_carriage")
    x_carriage.visual(
        mesh_from_cadquery(_x_carriage_body(), "x_carriage_body"),
        material="carriage_graphite",
        name="carriage_body",
    )

    z_stage = model.part("z_stage")
    z_stage.visual(
        mesh_from_cadquery(_z_stage_body(), "z_stage_body"),
        material="stage_aluminum",
        name="stage_body",
    )

    model.articulation(
        "frame_to_x_carriage",
        ArticulationType.PRISMATIC,
        parent=back_frame,
        child=x_carriage,
        origin=Origin(xyz=(0.0, H_RAIL_Y, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-X_TRAVEL,
            upper=X_TRAVEL,
            effort=350.0,
            velocity=0.40,
        ),
    )
    model.articulation(
        "x_carriage_to_z_stage",
        ArticulationType.PRISMATIC,
        parent=x_carriage,
        child=z_stage,
        origin=Origin(xyz=(0.0, V_RAIL_Y, Z_STAGE_LOW_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=Z_TRAVEL,
            effort=240.0,
            velocity=0.30,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    back_frame = object_model.get_part("back_frame")
    x_carriage = object_model.get_part("x_carriage")
    z_stage = object_model.get_part("z_stage")
    x_joint = object_model.get_articulation("frame_to_x_carriage")
    z_joint = object_model.get_articulation("x_carriage_to_z_stage")

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

    ctx.expect_contact(x_carriage, back_frame, name="x_carriage_contacts_horizontal_rails")
    ctx.expect_contact(z_stage, x_carriage, name="z_stage_contacts_mast_rails")
    ctx.expect_origin_gap(
        x_carriage,
        back_frame,
        axis="y",
        min_gap=0.08,
        max_gap=0.084,
        name="x_carriage_sits_forward_of_wall_frame",
    )
    ctx.expect_origin_gap(
        z_stage,
        x_carriage,
        axis="y",
        min_gap=0.12,
        max_gap=0.126,
        name="z_stage_sits_forward_of_x_carriage",
    )

    ctx.check(
        "x_joint_axis_is_lateral",
        tuple(x_joint.axis) == (1.0, 0.0, 0.0),
        details=f"expected lateral axis, got {x_joint.axis}",
    )
    ctx.check(
        "z_joint_axis_is_vertical",
        tuple(z_joint.axis) == (0.0, 0.0, 1.0),
        details=f"expected vertical axis, got {z_joint.axis}",
    )

    with ctx.pose({x_joint: -X_TRAVEL}):
        x_left = ctx.part_world_position(x_carriage)
    with ctx.pose({x_joint: X_TRAVEL}):
        x_right = ctx.part_world_position(x_carriage)
    ctx.check(
        "x_carriage_translates_side_to_side",
        x_left is not None and x_right is not None and x_left[0] < -0.16 and x_right[0] > 0.16,
        details=f"left={x_left}, right={x_right}",
    )

    with ctx.pose({z_joint: 0.0}):
        z_low = ctx.part_world_position(z_stage)
    with ctx.pose({z_joint: Z_TRAVEL}):
        z_high = ctx.part_world_position(z_stage)
    ctx.check(
        "z_stage_moves_upward",
        z_low is not None and z_high is not None and z_high[2] > z_low[2] + (Z_TRAVEL - 0.01),
        details=f"low={z_low}, high={z_high}",
    )

    with ctx.pose({x_joint: X_TRAVEL, z_joint: Z_TRAVEL}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlaps_at_maximum_travel_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
