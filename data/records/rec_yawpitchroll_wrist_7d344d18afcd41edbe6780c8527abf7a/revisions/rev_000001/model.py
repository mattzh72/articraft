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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.26
BASE_WIDTH = 0.18
PLATE_THICKNESS = 0.014
BASE_HOLE_RADIUS = 0.006
BASE_HOLE_X = 0.095
BASE_HOLE_Y = 0.055

PEDESTAL_RADIUS = 0.033
PEDESTAL_HEIGHT = 0.028
YAW_JOINT_Z = PLATE_THICKNESS + PEDESTAL_HEIGHT

YAW_ROTOR_RADIUS = 0.036
YAW_ROTOR_HEIGHT = 0.012
PITCH_AXIS_X = 0.070
PITCH_AXIS_Z = 0.055
PITCH_EAR_THICKNESS = 0.012
PITCH_INNER_GAP = 0.050
PITCH_OUTER_SPAN = PITCH_INNER_GAP + (2.0 * PITCH_EAR_THICKNESS)
PITCH_STUB_RADIUS = 0.007
PITCH_BARREL_RADIUS = 0.012

ROLL_AXIS_X = 0.082
ROLL_SLEEVE_OUTER_RADIUS = 0.022
ROLL_JOURNAL_RADIUS = 0.009
ROLL_SLEEVE_LENGTH = 0.024


def _make_base_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, PLATE_THICKNESS)
        .translate((0.0, 0.0, PLATE_THICKNESS / 2.0))
    )
    pedestal = (
        cq.Workplane("XY")
        .cylinder(PEDESTAL_HEIGHT, PEDESTAL_RADIUS)
        .translate((0.0, 0.0, PLATE_THICKNESS + (PEDESTAL_HEIGHT / 2.0)))
    )
    front_rib = (
        cq.Workplane("XY")
        .box(0.090, 0.030, 0.020)
        .translate((0.028, 0.0, PLATE_THICKNESS + 0.010))
    )
    side_rib_left = (
        cq.Workplane("XY")
        .box(0.060, 0.016, 0.018)
        .translate((0.018, 0.036, PLATE_THICKNESS + 0.009))
    )
    side_rib_right = (
        cq.Workplane("XY")
        .box(0.060, 0.016, 0.018)
        .translate((0.018, -0.036, PLATE_THICKNESS + 0.009))
    )

    base = (
        plate.union(pedestal)
        .union(front_rib)
        .union(side_rib_left)
        .union(side_rib_right)
    )

    mounting_holes = (
        cq.Workplane("XY")
        .pushPoints(
            [
                (BASE_HOLE_X, BASE_HOLE_Y),
                (BASE_HOLE_X, -BASE_HOLE_Y),
                (-BASE_HOLE_X, BASE_HOLE_Y),
                (-BASE_HOLE_X, -BASE_HOLE_Y),
            ]
        )
        .circle(BASE_HOLE_RADIUS)
        .extrude(PLATE_THICKNESS + 0.004)
        .translate((0.0, 0.0, -0.002))
    )

    return base.cut(mounting_holes)


def _make_yaw_stage_shape() -> cq.Workplane:
    rotor = (
        cq.Workplane("XY")
        .cylinder(YAW_ROTOR_HEIGHT, YAW_ROTOR_RADIUS)
        .translate((0.0, 0.0, YAW_ROTOR_HEIGHT / 2.0))
    )
    mast = (
        cq.Workplane("XY")
        .box(0.028, 0.036, 0.050)
        .translate((0.0, 0.0, 0.037))
    )
    brace = (
        cq.Workplane("XY")
        .box(0.050, 0.024, 0.022)
        .translate((0.020, 0.0, 0.031))
    )
    forward_arm = (
        cq.Workplane("XY")
        .box(0.044, 0.020, 0.014)
        .translate((0.032, 0.0, 0.063))
    )
    ear_y = (PITCH_INNER_GAP / 2.0) + (PITCH_EAR_THICKNESS / 2.0)
    cross_bridge = (
        cq.Workplane("XY")
        .box(0.016, 0.070, 0.010)
        .translate((0.060, 0.0, 0.071))
    )
    left_link = (
        cq.Workplane("XY")
        .box(0.012, 0.008, 0.022)
        .translate((0.064, ear_y, 0.060))
    )
    right_link = (
        cq.Workplane("XY")
        .box(0.012, 0.008, 0.022)
        .translate((0.064, -ear_y, 0.060))
    )
    left_ear = (
        cq.Workplane("XY")
        .box(0.010, PITCH_EAR_THICKNESS, 0.028)
        .translate((PITCH_AXIS_X, ear_y, PITCH_AXIS_Z))
    )
    right_ear = (
        cq.Workplane("XY")
        .box(0.010, PITCH_EAR_THICKNESS, 0.028)
        .translate((PITCH_AXIS_X, -ear_y, PITCH_AXIS_Z))
    )

    stage = (
        rotor.union(mast)
        .union(brace)
        .union(forward_arm)
        .union(cross_bridge)
        .union(left_link)
        .union(right_link)
        .union(left_ear)
        .union(right_ear)
    )
    return stage


def _make_pitch_cradle_shape() -> cq.Workplane:
    left_cheek = (
        cq.Workplane("XY")
        .box(0.016, 0.014, 0.026)
        .translate((0.002, 0.018, 0.0))
    )
    right_cheek = (
        cq.Workplane("XY")
        .box(0.016, 0.014, 0.026)
        .translate((0.002, -0.018, 0.0))
    )
    rear_bridge = (
        cq.Workplane("XY")
        .box(0.010, 0.050, 0.012)
        .translate((-0.003, 0.0, 0.0))
    )
    left_rail = (
        cq.Workplane("XY")
        .box(0.082, 0.010, 0.014)
        .translate((0.045, 0.018, 0.0))
    )
    right_rail = (
        cq.Workplane("XY")
        .box(0.082, 0.010, 0.014)
        .translate((0.045, -0.018, 0.0))
    )
    roll_ring_outer = (
        cq.Workplane("YZ")
        .circle(0.016)
        .extrude(0.008, both=True)
        .translate((ROLL_AXIS_X, 0.0, 0.0))
    )
    roll_ring_bore = (
        cq.Workplane("YZ")
        .circle(0.0105)
        .extrude(0.009, both=True)
        .translate((ROLL_AXIS_X, 0.0, 0.0))
    )

    cradle = (
        left_cheek.union(right_cheek)
        .union(rear_bridge)
        .union(left_rail)
        .union(right_rail)
        .union(roll_ring_outer)
    )
    return cradle.cut(roll_ring_bore)


def _make_roll_spindle_shape() -> cq.Workplane:
    journal = (
        cq.Workplane("YZ")
        .circle(0.0075)
        .extrude(0.011, both=True)
    )
    rear_thrust_collar = (
        cq.Workplane("YZ")
        .circle(0.013)
        .extrude(0.0035, both=True)
        .translate((-0.0115, 0.0, 0.0))
    )
    front_thrust_collar = (
        cq.Workplane("YZ")
        .circle(0.013)
        .extrude(0.0035, both=True)
        .translate((0.0115, 0.0, 0.0))
    )
    spindle_body = (
        cq.Workplane("YZ")
        .circle(0.014)
        .extrude(0.020, both=True)
        .translate((0.046, 0.0, 0.0))
    )
    tool_flange = (
        cq.Workplane("YZ")
        .circle(0.024)
        .extrude(0.004, both=True)
        .translate((0.072, 0.0, 0.0))
    )
    tool_nose = (
        cq.Workplane("YZ")
        .circle(0.0085)
        .extrude(0.014, both=True)
        .translate((0.090, 0.0, 0.0))
    )

    return (
        journal.union(rear_thrust_collar)
        .union(front_thrust_collar)
        .union(spindle_body)
        .union(tool_flange)
        .union(tool_nose)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_frame_wrist")

    base_finish = model.material("base_finish", rgba=(0.24, 0.25, 0.27, 1.0))
    stage_finish = model.material("stage_finish", rgba=(0.18, 0.19, 0.21, 1.0))
    cradle_finish = model.material("cradle_finish", rgba=(0.73, 0.75, 0.78, 1.0))
    spindle_finish = model.material("spindle_finish", rgba=(0.84, 0.86, 0.88, 1.0))

    root_plate = model.part("root_plate")
    root_plate.visual(
        mesh_from_cadquery(_make_base_shape(), "root_plate"),
        material=base_finish,
        name="root_plate_shell",
    )

    yaw_stage = model.part("yaw_stage")
    yaw_stage.visual(
        Cylinder(radius=YAW_ROTOR_RADIUS, length=YAW_ROTOR_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, YAW_ROTOR_HEIGHT / 2.0)),
        material=stage_finish,
        name="yaw_rotor",
    )
    yaw_stage.visual(
        Box((0.026, 0.030, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=stage_finish,
        name="yaw_mast",
    )
    yaw_stage.visual(
        Box((0.064, 0.018, 0.014)),
        origin=Origin(xyz=(0.032, 0.0, 0.058)),
        material=stage_finish,
        name="yaw_arm",
    )
    yaw_stage.visual(
        Box((0.020, 0.060, 0.010)),
        origin=Origin(xyz=(0.060, 0.0, 0.070)),
        material=stage_finish,
        name="yaw_crossbeam",
    )
    yaw_stage.visual(
        Box((0.014, 0.008, 0.018)),
        origin=Origin(xyz=(0.064, 0.031, 0.056)),
        material=stage_finish,
        name="yaw_left_strut",
    )
    yaw_stage.visual(
        Box((0.014, 0.008, 0.018)),
        origin=Origin(xyz=(0.064, -0.031, 0.056)),
        material=stage_finish,
        name="yaw_right_strut",
    )
    yaw_stage.visual(
        Box((0.010, 0.012, 0.024)),
        origin=Origin(xyz=(PITCH_AXIS_X, 0.031, PITCH_AXIS_Z)),
        material=stage_finish,
        name="yaw_left_ear",
    )
    yaw_stage.visual(
        Box((0.010, 0.012, 0.024)),
        origin=Origin(xyz=(PITCH_AXIS_X, -0.031, PITCH_AXIS_Z)),
        material=stage_finish,
        name="yaw_right_ear",
    )

    pitch_cradle = model.part("pitch_cradle")
    pitch_cradle.visual(
        Cylinder(radius=0.0065, length=0.050),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=cradle_finish,
        name="pitch_trunnion",
    )
    pitch_cradle.visual(
        Cylinder(radius=0.011, length=0.030),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=cradle_finish,
        name="pitch_hub",
    )
    pitch_cradle.visual(
        Box((0.078, 0.010, 0.012)),
        origin=Origin(xyz=(0.045, 0.016, 0.0)),
        material=cradle_finish,
        name="pitch_left_rail",
    )
    pitch_cradle.visual(
        Box((0.078, 0.010, 0.012)),
        origin=Origin(xyz=(0.045, -0.016, 0.0)),
        material=cradle_finish,
        name="pitch_right_rail",
    )
    pitch_cradle.visual(
        Box((0.014, 0.040, 0.008)),
        origin=Origin(xyz=(ROLL_AXIS_X, 0.0, 0.014)),
        material=cradle_finish,
        name="roll_cage_top",
    )
    pitch_cradle.visual(
        Box((0.014, 0.040, 0.008)),
        origin=Origin(xyz=(ROLL_AXIS_X, 0.0, -0.014)),
        material=cradle_finish,
        name="roll_cage_bottom",
    )
    pitch_cradle.visual(
        Box((0.010, 0.008, 0.020)),
        origin=Origin(xyz=(ROLL_AXIS_X, 0.016, 0.0)),
        material=cradle_finish,
        name="roll_cage_left",
    )
    pitch_cradle.visual(
        Box((0.010, 0.008, 0.020)),
        origin=Origin(xyz=(ROLL_AXIS_X, -0.016, 0.0)),
        material=cradle_finish,
        name="roll_cage_right",
    )

    roll_spindle = model.part("roll_spindle")
    roll_spindle.visual(
        Cylinder(radius=0.008, length=0.018),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=spindle_finish,
        name="roll_journal",
    )
    roll_spindle.visual(
        Cylinder(radius=0.011, length=0.004),
        origin=Origin(xyz=(-0.011, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=spindle_finish,
        name="roll_rear_collar",
    )
    roll_spindle.visual(
        Cylinder(radius=0.0085, length=0.062),
        origin=Origin(xyz=(0.040, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=spindle_finish,
        name="roll_body",
    )
    roll_spindle.visual(
        Cylinder(radius=0.021, length=0.006),
        origin=Origin(xyz=(0.074, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=spindle_finish,
        name="tool_flange",
    )
    roll_spindle.visual(
        Cylinder(radius=0.007, length=0.020),
        origin=Origin(xyz=(0.087, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=spindle_finish,
        name="tool_nose",
    )

    model.articulation(
        "root_to_yaw",
        ArticulationType.REVOLUTE,
        parent=root_plate,
        child=yaw_stage,
        origin=Origin(xyz=(0.0, 0.0, YAW_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=24.0, velocity=2.0, lower=-2.5, upper=2.5),
    )
    model.articulation(
        "yaw_to_pitch",
        ArticulationType.REVOLUTE,
        parent=yaw_stage,
        child=pitch_cradle,
        origin=Origin(xyz=(PITCH_AXIS_X, 0.0, PITCH_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=2.2, lower=-1.15, upper=1.15),
    )
    model.articulation(
        "pitch_to_roll",
        ArticulationType.REVOLUTE,
        parent=pitch_cradle,
        child=roll_spindle,
        origin=Origin(xyz=(ROLL_AXIS_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=5.0, lower=-pi, upper=pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root_plate = object_model.get_part("root_plate")
    yaw_stage = object_model.get_part("yaw_stage")
    pitch_cradle = object_model.get_part("pitch_cradle")
    roll_spindle = object_model.get_part("roll_spindle")
    yaw_joint = object_model.get_articulation("root_to_yaw")
    pitch_joint = object_model.get_articulation("yaw_to_pitch")
    roll_joint = object_model.get_articulation("pitch_to_roll")

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
        "yaw axis is vertical",
        tuple(yaw_joint.axis) == (0.0, 0.0, 1.0),
        f"axis={yaw_joint.axis}",
    )
    ctx.check(
        "pitch axis is horizontal",
        tuple(pitch_joint.axis) == (0.0, -1.0, 0.0),
        f"axis={pitch_joint.axis}",
    )
    ctx.check(
        "roll axis follows spindle",
        tuple(roll_joint.axis) == (1.0, 0.0, 0.0),
        f"axis={roll_joint.axis}",
    )

    ctx.expect_contact(
        yaw_stage,
        root_plate,
        contact_tol=0.0008,
        name="yaw stage is seated on the fixed pedestal",
    )
    ctx.expect_origin_gap(
        pitch_cradle,
        yaw_stage,
        axis="x",
        min_gap=PITCH_AXIS_X - 0.001,
        max_gap=PITCH_AXIS_X + 0.001,
        name="pitch cradle origin sits at the forward trunnion line",
    )
    ctx.expect_origin_gap(
        pitch_cradle,
        yaw_stage,
        axis="z",
        min_gap=PITCH_AXIS_Z - 0.001,
        max_gap=PITCH_AXIS_Z + 0.001,
        name="pitch cradle origin sits at the yaw-stage ear height",
    )
    ctx.expect_origin_distance(
        pitch_cradle,
        yaw_stage,
        axes="y",
        min_dist=0.0,
        max_dist=0.001,
        name="pitch cradle stays centered between the yaw ears",
    )
    ctx.expect_origin_gap(
        roll_spindle,
        pitch_cradle,
        axis="x",
        min_gap=ROLL_AXIS_X - 0.001,
        max_gap=ROLL_AXIS_X + 0.001,
        name="roll spindle origin sits on the cradle tool axis",
    )
    ctx.expect_origin_distance(
        roll_spindle,
        pitch_cradle,
        axes="yz",
        min_dist=0.0,
        max_dist=0.001,
        name="roll spindle origin stays centered in the cradle cage",
    )

    neutral_pitch_origin = ctx.part_world_position(pitch_cradle)
    neutral_roll_origin = ctx.part_world_position(roll_spindle)

    with ctx.pose({yaw_joint: 0.60}):
        yawed_pitch_origin = ctx.part_world_position(pitch_cradle)
    ctx.check(
        "positive yaw swings the cradle toward +Y",
        (
            neutral_pitch_origin is not None
            and yawed_pitch_origin is not None
            and yawed_pitch_origin[1] > neutral_pitch_origin[1] + 0.030
        ),
        f"neutral={neutral_pitch_origin}, yawed={yawed_pitch_origin}",
    )

    with ctx.pose({pitch_joint: 0.60}):
        pitched_roll_origin = ctx.part_world_position(roll_spindle)
    ctx.check(
        "positive pitch lifts the roll spindle",
        (
            neutral_roll_origin is not None
            and pitched_roll_origin is not None
            and pitched_roll_origin[2] > neutral_roll_origin[2] + 0.035
        ),
        f"neutral={neutral_roll_origin}, pitched={pitched_roll_origin}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
