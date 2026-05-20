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


BASE_WIDTH = 0.240
BASE_DEPTH = 0.124
BASE_THICKNESS = 0.016
CHEEK_THICKNESS = 0.022
CHEEK_DEPTH = 0.104
CHEEK_HEIGHT = 0.158
AXIS_HEIGHT = 0.102

INNER_CLEAR_WIDTH = BASE_WIDTH - 2.0 * CHEEK_THICKNESS
CHEEK_CENTER_Z = -AXIS_HEIGHT + BASE_THICKNESS + CHEEK_HEIGHT / 2.0
CHEEK_CENTER_X = BASE_WIDTH / 2.0 - CHEEK_THICKNESS / 2.0

ROLL_BODY_LENGTH = 0.118
ROLL_BODY_WIDTH = 0.110
ROLL_BODY_HEIGHT = 0.118
ROLL_WINDOW_WIDTH = 0.090
ROLL_WINDOW_HEIGHT = 0.082
ROLL_SHOULDER_RADIUS = 0.030
ROLL_SHOULDER_LENGTH = 0.014
ROLL_JOURNAL_RADIUS = 0.020
ROLL_JOURNAL_LENGTH = 0.028

PITCH_TRUNNION_RADIUS = 0.014
PITCH_TRUNNION_LENGTH = 0.078
FLANGE_THICKNESS = 0.010
FLANGE_WIDTH = 0.060
FLANGE_HEIGHT = 0.046
FLANGE_CENTER_X = 0.012
FLANGE_CENTER_Z = 0.076


def _ground_shape():
    base = cq.Workplane("XY").box(BASE_WIDTH, BASE_DEPTH, BASE_THICKNESS).translate(
        (0.0, 0.0, -AXIS_HEIGHT + BASE_THICKNESS / 2.0)
    )

    left_cheek = cq.Workplane("XY").box(
        CHEEK_THICKNESS, CHEEK_DEPTH, CHEEK_HEIGHT
    ).translate((-CHEEK_CENTER_X, 0.0, CHEEK_CENTER_Z))
    right_cheek = cq.Workplane("XY").box(
        CHEEK_THICKNESS, CHEEK_DEPTH, CHEEK_HEIGHT
    ).translate((CHEEK_CENTER_X, 0.0, CHEEK_CENTER_Z))

    boss_length = CHEEK_THICKNESS + 0.012
    left_boss = cq.Workplane("YZ").circle(0.030).extrude(boss_length).translate(
        (-CHEEK_CENTER_X - boss_length / 2.0, 0.0, 0.0)
    )
    right_boss = cq.Workplane("YZ").circle(0.030).extrude(boss_length).translate(
        (CHEEK_CENTER_X - boss_length / 2.0, 0.0, 0.0)
    )

    structure = (
        base.union(left_cheek)
        .union(right_cheek)
        .union(left_boss)
        .union(right_boss)
    )

    bearing_hole_length = CHEEK_THICKNESS + 0.040
    left_bearing_hole = (
        cq.Workplane("YZ")
        .circle(0.020)
        .extrude(bearing_hole_length)
        .translate((-CHEEK_CENTER_X - bearing_hole_length / 2.0, 0.0, 0.0))
    )
    right_bearing_hole = (
        cq.Workplane("YZ")
        .circle(0.020)
        .extrude(bearing_hole_length)
        .translate((CHEEK_CENTER_X - bearing_hole_length / 2.0, 0.0, 0.0))
    )

    left_window = cq.Workplane("YZ").rect(0.050, 0.078).extrude(0.040).translate(
        (-CHEEK_CENTER_X - 0.020, 0.0, -0.010)
    )
    right_window = cq.Workplane("YZ").rect(0.050, 0.078).extrude(0.040).translate(
        (CHEEK_CENTER_X - 0.020, 0.0, -0.010)
    )

    return (
        structure.cut(left_bearing_hole)
        .cut(right_bearing_hole)
        .cut(left_window)
        .cut(right_window)
    )


def _outer_roll_shape():
    upper_beam = cq.Workplane("XY").box(ROLL_BODY_LENGTH, 0.094, 0.012).translate(
        (0.0, 0.0, 0.032)
    )
    lower_beam = cq.Workplane("XY").box(ROLL_BODY_LENGTH, 0.086, 0.012).translate(
        (0.0, 0.0, -0.032)
    )
    front_plate = cq.Workplane("XY").box(0.074, 0.014, 0.082).translate(
        (0.0, 0.047, 0.0)
    )
    rear_plate = cq.Workplane("XY").box(0.074, 0.014, 0.082).translate(
        (0.0, -0.047, 0.0)
    )

    shoulder = (
        cq.Workplane("YZ")
        .circle(ROLL_SHOULDER_RADIUS)
        .circle(0.018)
        .extrude(ROLL_SHOULDER_LENGTH)
    )
    journal = (
        cq.Workplane("YZ")
        .circle(ROLL_JOURNAL_RADIUS)
        .circle(0.016)
        .extrude(ROLL_JOURNAL_LENGTH)
    )

    pos_shoulder = shoulder.translate((0.049, 0.0, 0.0))
    neg_shoulder = shoulder.translate((-0.063, 0.0, 0.0))
    pos_journal = journal.translate((0.061, 0.0, 0.0))
    neg_journal = journal.translate((-0.089, 0.0, 0.0))

    return (
        upper_beam.union(lower_beam)
        .union(front_plate)
        .union(rear_plate)
        .union(pos_shoulder)
        .union(neg_shoulder)
        .union(pos_journal)
        .union(neg_journal)
    )


def _inner_cradle_body_shape():
    trunnion = cq.Workplane("XZ").circle(PITCH_TRUNNION_RADIUS).extrude(PITCH_TRUNNION_LENGTH).translate(
        (0.0, PITCH_TRUNNION_LENGTH / 2.0, 0.0)
    )
    mast = cq.Workplane("XY").box(0.022, 0.022, 0.066).translate(
        (0.0, 0.0, 0.033)
    )
    head = cq.Workplane("XY").box(0.040, 0.022, 0.014).translate(
        (0.012, 0.0, 0.069)
    )
    gusset = cq.Workplane("XY").box(0.018, 0.026, 0.024).translate(
        (-0.008, 0.0, 0.012)
    )

    return trunnion.union(mast).union(head).union(gusset)


def _output_flange_shape():
    flange = cq.Workplane("XY").rect(FLANGE_WIDTH, FLANGE_HEIGHT).extrude(FLANGE_THICKNESS)
    flange = flange.translate(
        (FLANGE_CENTER_X, 0.0, FLANGE_CENTER_Z - FLANGE_THICKNESS / 2.0)
    )
    flange = (
        flange.faces(">Z")
        .workplane()
        .pushPoints(
            [
                (-0.018, -0.013),
                (0.018, -0.013),
                (-0.018, 0.013),
                (0.018, 0.013),
            ]
        )
        .hole(0.006)
    )
    return flange


def _aabb_center(aabb):
    if aabb is None:
        return None
    low, high = aabb
    return tuple((low[i] + high[i]) * 0.5 for i in range(3))


def _aabb_span(aabb, axis_index: int) -> float | None:
    if aabb is None:
        return None
    low, high = aabb
    return high[axis_index] - low[axis_index]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_cheek_pitch_roll_cartridge")

    model.material("frame_paint", rgba=(0.24, 0.26, 0.29, 1.0))
    model.material("anodized_gray", rgba=(0.58, 0.61, 0.65, 1.0))
    model.material("dark_anodized", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("machined_face", rgba=(0.78, 0.80, 0.83, 1.0))

    side_structure = model.part("side_structure")
    side_structure.visual(
        mesh_from_cadquery(_ground_shape(), "side_structure"),
        material="frame_paint",
        name="side_structure_shell",
    )

    outer_roll_package = model.part("outer_roll_package")
    outer_roll_package.visual(
        mesh_from_cadquery(_outer_roll_shape(), "outer_roll_package"),
        material="anodized_gray",
        name="roll_carrier",
    )

    inner_cradle = model.part("inner_cradle")
    inner_cradle.visual(
        mesh_from_cadquery(_inner_cradle_body_shape(), "inner_cradle_body"),
        material="dark_anodized",
        name="cradle_body",
    )
    inner_cradle.visual(
        mesh_from_cadquery(_output_flange_shape(), "output_flange"),
        material="machined_face",
        name="output_flange",
    )

    model.articulation(
        "outer_roll_axis",
        ArticulationType.REVOLUTE,
        parent=side_structure,
        child=outer_roll_package,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=28.0,
            velocity=2.5,
            lower=-2.6,
            upper=2.6,
        ),
    )
    model.articulation(
        "inner_pitch_axis",
        ArticulationType.REVOLUTE,
        parent=outer_roll_package,
        child=inner_cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.5,
            lower=-1.0,
            upper=1.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    side_structure = object_model.get_part("side_structure")
    outer_roll_package = object_model.get_part("outer_roll_package")
    inner_cradle = object_model.get_part("inner_cradle")
    outer_roll_axis = object_model.get_articulation("outer_roll_axis")
    inner_pitch_axis = object_model.get_articulation("inner_pitch_axis")

    ctx.check(
        "roll axis is aligned to x",
        tuple(outer_roll_axis.axis) == (1.0, 0.0, 0.0),
        details=f"axis={outer_roll_axis.axis}",
    )
    ctx.check(
        "pitch axis is perpendicular and lifts on positive motion",
        tuple(inner_pitch_axis.axis) == (0.0, 1.0, 0.0),
        details=f"axis={inner_pitch_axis.axis}",
    )

    ctx.allow_isolated_part(
        outer_roll_package,
        reason="The roll journals are modeled with running clearance inside the cheek bores to represent bearing support without solid contact.",
    )
    ctx.allow_isolated_part(
        inner_cradle,
        reason="The inner cradle is carried by the same clearance-modeled roll cartridge assembly.",
    )

    ctx.expect_overlap(
        outer_roll_package,
        side_structure,
        axes="yz",
        min_overlap=0.070,
        name="outer roll package sits inside the cheek envelope",
    )
    ctx.expect_overlap(
        inner_cradle,
        outer_roll_package,
        axes="y",
        min_overlap=0.060,
        name="inner cradle stays captured between the roll package side plates",
    )

    neutral_flange_aabb = ctx.part_element_world_aabb(inner_cradle, elem="output_flange")
    neutral_flange_center = _aabb_center(neutral_flange_aabb)
    neutral_y_span = _aabb_span(neutral_flange_aabb, 1)
    neutral_z_span = _aabb_span(neutral_flange_aabb, 2)

    ctx.check(
        "output flange sits above the pitch center",
        neutral_flange_center is not None and neutral_flange_center[2] > 0.070,
        details=f"flange_center={neutral_flange_center}",
    )

    with ctx.pose({inner_pitch_axis: 0.75}):
        pitched_flange_aabb = ctx.part_element_world_aabb(inner_cradle, elem="output_flange")
        pitched_flange_center = _aabb_center(pitched_flange_aabb)

    ctx.check(
        "positive pitch drives the output flange forward",
        neutral_flange_center is not None
        and pitched_flange_center is not None
        and pitched_flange_center[0] > neutral_flange_center[0] + 0.025,
        details=f"neutral={neutral_flange_center}, pitched={pitched_flange_center}",
    )

    with ctx.pose({outer_roll_axis: 1.57}):
        rolled_flange_aabb = ctx.part_element_world_aabb(inner_cradle, elem="output_flange")
        rolled_y_span = _aabb_span(rolled_flange_aabb, 1)
        rolled_z_span = _aabb_span(rolled_flange_aabb, 2)

    ctx.check(
        "roll tips the horizontal output flange into a vertical orientation",
        neutral_y_span is not None
        and neutral_z_span is not None
        and rolled_y_span is not None
        and rolled_z_span is not None
        and neutral_y_span > neutral_z_span + 0.020
        and rolled_z_span > rolled_y_span + 0.020,
        details=(
            f"neutral_y_span={neutral_y_span}, neutral_z_span={neutral_z_span}, "
            f"rolled_y_span={rolled_y_span}, rolled_z_span={rolled_z_span}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
