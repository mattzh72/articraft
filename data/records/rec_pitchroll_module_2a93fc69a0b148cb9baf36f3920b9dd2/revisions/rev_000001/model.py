from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


FRAME_OUTER = 0.195
FRAME_OPEN = 0.151
FRAME_DEPTH = 0.018

ROLL_COLLAR_RADIUS = 0.016
ROLL_COLLAR_LENGTH = 0.004
ROLL_SHAFT_RADIUS = 0.0118
HOUSING_RADIUS = 0.022
HOUSING_LENGTH = 0.026
ROLL_SHAFT_LENGTH = HOUSING_LENGTH
ROLL_BORE_RADIUS = 0.0122
ROLL_COUNTERBORE_RADIUS = 0.0162
ROLL_COUNTERBORE_DEPTH = 0.004

SUPPORT_SIDE_CENTER_X = 0.145
HOUSING_CENTER_X = FRAME_OUTER / 2.0 + ROLL_COLLAR_LENGTH + HOUSING_LENGTH / 2.0 - 0.00025
SUPPORT_CHEEK_THICK = 0.018
SUPPORT_CHEEK_DEPTH = 0.016
SUPPORT_CHEEK_HEIGHT = 0.172
SUPPORT_CHEEK_Y = -0.032
SUPPORT_BAR_HEIGHT = 0.016
SUPPORT_BAR_DEPTH = 0.014
SUPPORT_BAR_Z = 0.118
SUPPORT_FOOT_WIDTH = 0.048
SUPPORT_FOOT_DEPTH = 0.034
SUPPORT_FOOT_HEIGHT = 0.018

PITCH_HOUSING_RADIUS = 0.018
PITCH_HOUSING_LENGTH = 0.014
PITCH_BORE_RADIUS = 0.0104
PITCH_COUNTERBORE_RADIUS = 0.0146
PITCH_COUNTERBORE_DEPTH = 0.004
PITCH_COLLAR_RADIUS = 0.0145
PITCH_COLLAR_LENGTH = 0.004
PITCH_SHAFT_RADIUS = 0.0098
PITCH_SHAFT_LENGTH = PITCH_HOUSING_LENGTH
PITCH_HOUSING_CENTER_Y = FRAME_DEPTH / 2.0 + PITCH_COLLAR_LENGTH + PITCH_HOUSING_LENGTH / 2.0

FLANGE_SIZE = 0.062
FLANGE_THICKNESS = 0.010
FLANGE_HOLE_RADIUS = 0.003
FLANGE_HOLE_SPAN = 0.036
FLANGE_CENTER_BORE_RADIUS = 0.012

CRADLE_BODY_THICK = 0.010
BARREL_RADIUS = 0.032
BARREL_INNER_RADIUS = 0.026
BARREL_LENGTH = 0.050
BARREL_WINDOW_HEIGHT = 0.048


def cyl_x(radius: float, length: float) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length / 2.0, both=True)


def cyl_y(radius: float, length: float) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length / 2.0, both=True)


def box_xyz(x: float, y: float, z: float) -> cq.Workplane:
    return cq.Workplane("XY").box(x, y, z)


def _make_support_shape() -> cq.Workplane:
    total_width = 2.0 * SUPPORT_SIDE_CENTER_X + SUPPORT_CHEEK_THICK

    support = (
        box_xyz(SUPPORT_CHEEK_THICK, SUPPORT_CHEEK_DEPTH, SUPPORT_CHEEK_HEIGHT)
        .translate((-SUPPORT_SIDE_CENTER_X, SUPPORT_CHEEK_Y, 0.0))
        .union(
            box_xyz(SUPPORT_CHEEK_THICK, SUPPORT_CHEEK_DEPTH, SUPPORT_CHEEK_HEIGHT).translate(
                (SUPPORT_SIDE_CENTER_X, SUPPORT_CHEEK_Y, 0.0)
            )
        )
        .union(box_xyz(total_width, SUPPORT_BAR_DEPTH, SUPPORT_BAR_HEIGHT).translate((0.0, SUPPORT_CHEEK_Y, SUPPORT_BAR_Z)))
        .union(box_xyz(total_width, SUPPORT_BAR_DEPTH, SUPPORT_BAR_HEIGHT).translate((0.0, SUPPORT_CHEEK_Y, -SUPPORT_BAR_Z)))
        .union(box_xyz(SUPPORT_FOOT_WIDTH, SUPPORT_FOOT_DEPTH, SUPPORT_FOOT_HEIGHT).translate((-SUPPORT_SIDE_CENTER_X, SUPPORT_CHEEK_Y, -0.123)))
        .union(box_xyz(SUPPORT_FOOT_WIDTH, SUPPORT_FOOT_DEPTH, SUPPORT_FOOT_HEIGHT).translate((SUPPORT_SIDE_CENTER_X, SUPPORT_CHEEK_Y, -0.123)))
    )

    cheek_window = box_xyz(
        SUPPORT_CHEEK_THICK + 0.002,
        SUPPORT_CHEEK_DEPTH + 0.004,
        SUPPORT_CHEEK_HEIGHT - 0.058,
    )
    bridge_width = abs(HOUSING_CENTER_X - SUPPORT_SIDE_CENTER_X) + 0.014
    bridge_center = (HOUSING_CENTER_X + SUPPORT_SIDE_CENTER_X) / 2.0

    for sign in (-1.0, 1.0):
        cheek_x = sign * SUPPORT_SIDE_CENTER_X
        housing_x = sign * HOUSING_CENTER_X
        bridge_x = sign * bridge_center

        support = (
            support.union(cyl_x(HOUSING_RADIUS, HOUSING_LENGTH).translate((housing_x, 0.0, 0.0)))
            .union(box_xyz(bridge_width, 0.008, 0.066).translate((bridge_x, -0.019, 0.0)))
            .union(box_xyz(0.016, 0.012, 0.048).translate((cheek_x, -0.022, 0.058)))
            .union(box_xyz(0.016, 0.012, 0.048).translate((cheek_x, -0.022, -0.058)))
            .cut(cheek_window.translate((cheek_x, SUPPORT_CHEEK_Y, 0.0)))
            .cut(box_xyz(SUPPORT_CHEEK_THICK + 0.002, SUPPORT_CHEEK_DEPTH + 0.004, 0.070).translate((cheek_x, SUPPORT_CHEEK_Y, 0.0)))
            .cut(cyl_x(ROLL_BORE_RADIUS, HOUSING_LENGTH + 0.006).translate((housing_x, 0.0, 0.0)))
        )

        counterbore_center = sign * (HOUSING_CENTER_X - HOUSING_LENGTH / 2.0 + ROLL_COUNTERBORE_DEPTH / 2.0)
        support = support.cut(
            cyl_x(ROLL_COUNTERBORE_RADIUS, ROLL_COUNTERBORE_DEPTH).translate((counterbore_center, 0.0, 0.0))
        )

    return support


def _make_outer_frame_shape() -> cq.Workplane:
    rail_width = (FRAME_OUTER - FRAME_OPEN) / 2.0
    rail_center = FRAME_OUTER / 2.0 - rail_width / 2.0

    frame = (
        box_xyz(rail_width, FRAME_DEPTH, FRAME_OUTER).translate((-rail_center, 0.0, 0.0))
        .union(box_xyz(rail_width, FRAME_DEPTH, FRAME_OUTER).translate((rail_center, 0.0, 0.0)))
        .union(box_xyz(FRAME_OPEN, FRAME_DEPTH, rail_width).translate((0.0, 0.0, rail_center)))
        .union(box_xyz(FRAME_OPEN, FRAME_DEPTH, rail_width).translate((0.0, 0.0, -rail_center)))
    )

    for x_pos, z_pos, angle in (
        (0.066, 0.066, 45.0),
        (-0.066, 0.066, -45.0),
        (0.066, -0.066, -45.0),
        (-0.066, -0.066, 45.0),
    ):
        frame = frame.union(
            box_xyz(0.036, 0.006, 0.006)
            .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), angle)
            .translate((x_pos, 0.0, z_pos))
        )

    frame = frame.cut(cyl_y(PITCH_BORE_RADIUS, 2.0 * PITCH_HOUSING_CENTER_Y + PITCH_HOUSING_LENGTH + 0.006))

    for sign in (-1.0, 1.0):
        housing_center = sign * PITCH_HOUSING_CENTER_Y
        counterbore_center = sign * (
            PITCH_HOUSING_CENTER_Y + PITCH_HOUSING_LENGTH / 2.0 - PITCH_COUNTERBORE_DEPTH / 2.0
        )
        collar_center_x = sign * (FRAME_OUTER / 2.0 + ROLL_COLLAR_LENGTH / 2.0)
        shaft_center_x = sign * (FRAME_OUTER / 2.0 + ROLL_COLLAR_LENGTH + ROLL_SHAFT_LENGTH / 2.0)

        frame = (
            frame.union(cyl_y(PITCH_HOUSING_RADIUS, PITCH_HOUSING_LENGTH).translate((0.0, housing_center, 0.0)))
            .union(box_xyz(0.010, 0.006, 0.040).translate((0.0, sign * 0.010, 0.054)))
            .union(box_xyz(0.010, 0.006, 0.040).translate((0.0, sign * 0.010, -0.054)))
            .union(cyl_x(ROLL_COLLAR_RADIUS, ROLL_COLLAR_LENGTH).translate((collar_center_x, 0.0, 0.0)))
            .union(cyl_x(ROLL_SHAFT_RADIUS, ROLL_SHAFT_LENGTH).translate((shaft_center_x, 0.0, 0.0)))
            .cut(cyl_y(PITCH_COUNTERBORE_RADIUS, PITCH_COUNTERBORE_DEPTH).translate((0.0, counterbore_center, 0.0)))
        )

    return frame


def _make_flange_block() -> cq.Workplane:
    flange = box_xyz(FLANGE_SIZE, FLANGE_THICKNESS, FLANGE_SIZE)
    flange = flange.cut(cyl_y(FLANGE_CENTER_BORE_RADIUS, FLANGE_THICKNESS + 0.006))
    hole_offset = FLANGE_HOLE_SPAN / 2.0
    for x_pos in (-hole_offset, hole_offset):
        for z_pos in (-hole_offset, hole_offset):
            flange = flange.cut(
                cyl_y(FLANGE_HOLE_RADIUS, FLANGE_THICKNESS + 0.006).translate((x_pos, 0.0, z_pos))
            )
    return flange


def _make_cradle_shape() -> cq.Workplane:
    outer_shell = (
        cq.Workplane("XZ")
        .slot2D(BARREL_LENGTH, 2.0 * BARREL_RADIUS, angle=0.0)
        .extrude(CRADLE_BODY_THICK / 2.0, both=True)
    )
    inner_shell = (
        cq.Workplane("XZ")
        .slot2D(BARREL_LENGTH - 0.014, 2.0 * BARREL_INNER_RADIUS, angle=0.0)
        .extrude((CRADLE_BODY_THICK + 0.004) / 2.0, both=True)
    )
    barrel = outer_shell.cut(inner_shell).cut(box_xyz(BARREL_LENGTH + 0.012, 0.040, BARREL_WINDOW_HEIGHT))

    cradle = barrel.union(_make_flange_block())
    cradle = (
        cradle.union(box_xyz(0.016, FLANGE_THICKNESS, 0.044).translate((-0.013, 0.0, 0.0)))
        .union(box_xyz(0.016, FLANGE_THICKNESS, 0.044).translate((0.013, 0.0, 0.0)))
        .union(box_xyz(0.048, 0.006, 0.010).translate((0.0, 0.0, 0.021)))
        .union(box_xyz(0.048, 0.006, 0.010).translate((0.0, 0.0, -0.021)))
    )

    for sign in (-1.0, 1.0):
        collar_center_y = sign * (FLANGE_THICKNESS / 2.0 + PITCH_COLLAR_LENGTH / 2.0)
        shaft_center_y = sign * (PITCH_HOUSING_CENTER_Y + PITCH_HOUSING_LENGTH / 2.0 - PITCH_SHAFT_LENGTH / 2.0)
        cradle = (
            cradle.union(cyl_y(PITCH_COLLAR_RADIUS, PITCH_COLLAR_LENGTH).translate((0.0, collar_center_y, 0.0)))
            .union(cyl_y(PITCH_SHAFT_RADIUS, PITCH_SHAFT_LENGTH).translate((0.0, shaft_center_y, 0.0)))
        )

    return cradle


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_gimbal_cube")

    carrier_finish = model.material("carrier_finish", rgba=(0.15, 0.16, 0.18, 1.0))
    frame_finish = model.material("frame_finish", rgba=(0.68, 0.70, 0.73, 1.0))
    cradle_finish = model.material("cradle_finish", rgba=(0.49, 0.51, 0.54, 1.0))

    support = model.part("support_carrier")
    support.visual(
        mesh_from_cadquery(_make_support_shape(), "support_carrier"),
        material=carrier_finish,
        name="support_body",
    )

    outer_frame = model.part("outer_frame")
    outer_frame.visual(
        mesh_from_cadquery(_make_outer_frame_shape(), "outer_frame"),
        material=frame_finish,
        name="frame_body",
    )

    cradle = model.part("inner_cradle")
    cradle.visual(
        mesh_from_cadquery(_make_cradle_shape(), "inner_cradle"),
        material=cradle_finish,
        name="cradle_body",
    )

    model.articulation(
        "carrier_to_outer_frame",
        ArticulationType.REVOLUTE,
        parent=support,
        child=outer_frame,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=2.5, lower=-1.25, upper=1.25),
    )
    model.articulation(
        "outer_frame_to_cradle",
        ArticulationType.REVOLUTE,
        parent=outer_frame,
        child=cradle,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=-0.82, upper=0.82),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support_carrier")
    outer_frame = object_model.get_part("outer_frame")
    cradle = object_model.get_part("inner_cradle")
    roll = object_model.get_articulation("carrier_to_outer_frame")
    pitch = object_model.get_articulation("outer_frame_to_cradle")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        outer_frame,
        cradle,
        reason=(
            "Nested pitch capture uses coaxial shafts and surrounding frame housings; "
            "exact centering/contact checks below protect the intended relationship."
        ),
    )

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts(contact_tol=0.0003)
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
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    ctx.expect_contact(
        support,
        outer_frame,
        contact_tol=0.0003,
        name="roll frame is captured in carrier bearings",
    )
    ctx.expect_contact(outer_frame, cradle, name="pitch cradle is captured by frame bores")
    ctx.expect_overlap(outer_frame, cradle, axes="xz", min_overlap=0.045, name="cradle stays centered in frame")
    ctx.expect_within(cradle, outer_frame, axes="xz", margin=0.012, name="rest cradle stays inside frame window")

    ctx.check(
        "roll articulation uses main shaft axis",
        tuple(round(v, 6) for v in roll.axis) == (1.0, 0.0, 0.0),
        details=f"axis={roll.axis}",
    )
    ctx.check(
        "pitch articulation uses cross axis",
        tuple(round(v, 6) for v in pitch.axis) == (0.0, 1.0, 0.0),
        details=f"axis={pitch.axis}",
    )

    with ctx.pose({roll: 1.0, pitch: 0.68}):
        ctx.expect_within(
            cradle,
            outer_frame,
            axes="xz",
            margin=0.030,
            name="cradle remains within frame silhouette near combined travel limit",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
