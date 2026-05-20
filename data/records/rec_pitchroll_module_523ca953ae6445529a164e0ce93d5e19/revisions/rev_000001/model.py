from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


BASE_FOOT_X = 0.082
BASE_FOOT_Y = 0.092
BASE_FOOT_Z = 0.012

ROLL_AXIS_Z = 0.090
RING_OUTER_RADIUS = 0.050
RING_INNER_RADIUS = 0.036
RING_THICKNESS = 0.014

SUPPORT_PLATE_THICKNESS = 0.008
SUPPORT_SIDE_GAP = 0.0015
SUPPORT_PLATE_X = (
    RING_THICKNESS / 2.0 + SUPPORT_SIDE_GAP + SUPPORT_PLATE_THICKNESS / 2.0
)
SUPPORT_COLLAR_RADIUS = 0.064
SUPPORT_WINDOW_RADIUS = 0.052
SUPPORT_WEB_Y = 0.082
SUPPORT_WEB_Z = 0.090

PITCH_BOSS_RADIUS = 0.0065
PITCH_BOSS_HOLE_RADIUS = 0.0045
PITCH_BOSS_LENGTH = 0.010
PITCH_BOSS_CENTER_Y = 0.037

CRADLE_SHAFT_RADIUS = 0.0045
CRADLE_SHAFT_LENGTH = 0.078

ROLL_LIMIT = 1.4
PITCH_LOWER = -0.85
PITCH_UPPER = 0.85


def _make_base_bracket_shape() -> cq.Workplane:
    foot = (
        cq.Workplane("XY")
        .box(BASE_FOOT_X, BASE_FOOT_Y, BASE_FOOT_Z, centered=(True, True, False))
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(0.0, -0.024), (0.0, 0.024)])
        .slot2D(0.022, 0.007, angle=0.0)
        .cutThruAll()
    )

    support_web = cq.Workplane("XY").box(
        SUPPORT_PLATE_THICKNESS,
        SUPPORT_WEB_Y,
        SUPPORT_WEB_Z,
        centered=(True, True, False),
    )
    support_web = support_web.translate((0.0, 0.0, BASE_FOOT_Z))

    support_collar = (
        cq.Workplane("YZ")
        .circle(SUPPORT_COLLAR_RADIUS)
        .extrude(SUPPORT_PLATE_THICKNESS / 2.0, both=True)
        .translate((0.0, 0.0, ROLL_AXIS_Z))
    )
    support_window = (
        cq.Workplane("YZ")
        .circle(SUPPORT_WINDOW_RADIUS)
        .extrude((SUPPORT_PLATE_THICKNESS + 0.004) / 2.0, both=True)
        .translate((0.0, 0.0, ROLL_AXIS_Z))
    )
    lower_relief = (
        cq.Workplane("YZ")
        .rect(0.040, 0.044)
        .extrude((SUPPORT_PLATE_THICKNESS + 0.004) / 2.0, both=True)
        .translate((0.0, 0.0, BASE_FOOT_Z + 0.030))
    )

    support_core = support_web.union(support_collar).cut(support_window).cut(lower_relief)
    left_support = support_core.translate((-SUPPORT_PLATE_X, 0.0, 0.0))
    right_support = support_core.translate((SUPPORT_PLATE_X, 0.0, 0.0))

    rear_brace = cq.Workplane("XY").box(
        2.0 * SUPPORT_PLATE_X + SUPPORT_PLATE_THICKNESS,
        0.016,
        0.060,
        centered=(True, True, False),
    )
    rear_brace = rear_brace.translate((0.0, -0.024, BASE_FOOT_Z))

    front_rib = cq.Workplane("XY").box(
        2.0 * SUPPORT_PLATE_X + SUPPORT_PLATE_THICKNESS,
        0.012,
        0.030,
        centered=(True, True, False),
    )
    front_rib = front_rib.translate((0.0, 0.026, BASE_FOOT_Z))

    return foot.union(left_support).union(right_support).union(rear_brace).union(front_rib)


def _make_roll_frame_shape() -> cq.Workplane:
    ring = (
        cq.Workplane("YZ")
        .circle(RING_OUTER_RADIUS)
        .circle(RING_INNER_RADIUS)
        .extrude(RING_THICKNESS / 2.0, both=True)
    )

    boss_template = (
        cq.Workplane("XZ")
        .circle(PITCH_BOSS_RADIUS)
        .extrude(PITCH_BOSS_LENGTH / 2.0, both=True)
    )
    hole_template = (
        cq.Workplane("XZ")
        .circle(PITCH_BOSS_HOLE_RADIUS)
        .extrude((PITCH_BOSS_LENGTH + 0.004) / 2.0, both=True)
    )

    positive_boss = boss_template.translate((0.0, PITCH_BOSS_CENTER_Y, 0.0))
    negative_boss = boss_template.translate((0.0, -PITCH_BOSS_CENTER_Y, 0.0))
    positive_hole = hole_template.translate((0.0, PITCH_BOSS_CENTER_Y, 0.0))
    negative_hole = hole_template.translate((0.0, -PITCH_BOSS_CENTER_Y, 0.0))

    return (
        ring.union(positive_boss)
        .union(negative_boss)
        .cut(positive_hole)
        .cut(negative_hole)
    )


def _make_roll_marker_shape() -> cq.Workplane:
    return cq.Workplane("XY").box(0.012, 0.012, 0.006).translate(
        (0.0, 0.0, RING_OUTER_RADIUS + 0.002)
    )


def _make_pitch_cradle_body_shape() -> cq.Workplane:
    shaft = cq.Workplane("XZ").circle(CRADLE_SHAFT_RADIUS).extrude(
        CRADLE_SHAFT_LENGTH / 2.0, both=True
    )

    arm_left = cq.Workplane("XY").box(0.036, 0.020, 0.030).translate(
        (0.002, 0.015, 0.0)
    )
    arm_right = cq.Workplane("XY").box(0.036, 0.020, 0.030).translate(
        (0.002, -0.015, 0.0)
    )
    rear_bridge = cq.Workplane("XY").box(0.008, 0.034, 0.030).translate(
        (-0.012, 0.0, 0.0)
    )
    center_spine = cq.Workplane("XY").box(0.022, 0.016, 0.022).translate(
        (0.024, 0.0, 0.0)
    )

    return shaft.union(arm_left).union(arm_right).union(rear_bridge).union(center_spine)


def _make_output_face_shape() -> cq.Workplane:
    face = cq.Workplane("XY").box(0.006, 0.034, 0.034).translate((0.038, 0.0, 0.0))
    return (
        face.faces(">X")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [(-0.010, -0.010), (-0.010, 0.010), (0.010, -0.010), (0.010, 0.010)]
        )
        .hole(0.003)
    )


def _aabb_center(
    aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None,
):
    if aabb is None:
        return None
    minimum, maximum = aabb
    return tuple((minimum[index] + maximum[index]) / 2.0 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ring_cradle_pitch_roll_head")

    model.material("dark_frame", rgba=(0.20, 0.22, 0.25, 1.0))
    model.material("machined_metal", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("marker_red", rgba=(0.72, 0.16, 0.14, 1.0))

    base_bracket = model.part("base_bracket")
    base_bracket.visual(
        mesh_from_cadquery(_make_base_bracket_shape(), "base_bracket"),
        material="dark_frame",
        name="base_shell",
    )
    base_bracket.inertial = Inertial.from_geometry(
        Box((BASE_FOOT_X, SUPPORT_WEB_Y, 0.155)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, 0.0775)),
    )

    outer_roll_frame = model.part("outer_roll_frame")
    outer_roll_frame.visual(
        mesh_from_cadquery(_make_roll_frame_shape(), "outer_roll_frame"),
        material="machined_metal",
        name="roll_ring",
    )
    outer_roll_frame.visual(
        mesh_from_cadquery(_make_roll_marker_shape(), "roll_marker"),
        material="marker_red",
        name="roll_marker",
    )
    outer_roll_frame.inertial = Inertial.from_geometry(
        Box((0.016, 2.0 * RING_OUTER_RADIUS, 2.0 * RING_OUTER_RADIUS)),
        mass=0.9,
        origin=Origin(),
    )

    inner_pitch_cradle = model.part("inner_pitch_cradle")
    inner_pitch_cradle.visual(
        mesh_from_cadquery(_make_pitch_cradle_body_shape(), "inner_pitch_cradle"),
        material="dark_frame",
        name="cradle_body",
    )
    inner_pitch_cradle.visual(
        mesh_from_cadquery(_make_output_face_shape(), "output_face"),
        material="machined_metal",
        name="output_face",
    )
    inner_pitch_cradle.inertial = Inertial.from_geometry(
        Box((0.054, 0.050, 0.036)),
        mass=0.45,
        origin=Origin(xyz=(0.012, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_roll",
        ArticulationType.REVOLUTE,
        parent=base_bracket,
        child=outer_roll_frame,
        origin=Origin(xyz=(0.0, 0.0, ROLL_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-ROLL_LIMIT,
            upper=ROLL_LIMIT,
            effort=18.0,
            velocity=2.2,
        ),
    )
    model.articulation(
        "roll_to_pitch",
        ArticulationType.REVOLUTE,
        parent=outer_roll_frame,
        child=inner_pitch_cradle,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=PITCH_LOWER,
            upper=PITCH_UPPER,
            effort=12.0,
            velocity=2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base_bracket = object_model.get_part("base_bracket")
    outer_roll_frame = object_model.get_part("outer_roll_frame")
    inner_pitch_cradle = object_model.get_part("inner_pitch_cradle")

    base_to_roll = object_model.get_articulation("base_to_roll")
    roll_to_pitch = object_model.get_articulation("roll_to_pitch")

    ctx.check(
        "roll joint uses the main x-axis",
        tuple(base_to_roll.axis) == (1.0, 0.0, 0.0),
        details=f"axis={base_to_roll.axis}",
    )
    ctx.check(
        "pitch joint uses a perpendicular y-axis",
        tuple(roll_to_pitch.axis) == (0.0, -1.0, 0.0),
        details=f"axis={roll_to_pitch.axis}",
    )
    ctx.allow_overlap(
        base_bracket,
        outer_roll_frame,
        elem_a="base_shell",
        elem_b="roll_ring",
        reason="The grounded bracket intentionally captures the roll ring inside a close-fit bearing race representation.",
    )
    ctx.allow_overlap(
        inner_pitch_cradle,
        outer_roll_frame,
        elem_a="cradle_body",
        elem_b="roll_ring",
        reason="The pitch cradle is represented as a close-fit trunnion running inside the roll-frame bearing lugs.",
    )

    with ctx.pose({base_to_roll: 0.0, roll_to_pitch: 0.0}):
        ctx.expect_within(
            inner_pitch_cradle,
            outer_roll_frame,
            axes="yz",
            margin=0.002,
            name="cradle stays inside the roll frame aperture at rest",
        )
        ctx.expect_overlap(
            outer_roll_frame,
            base_bracket,
            axes="yz",
            min_overlap=0.090,
            name="roll frame remains nested inside the grounded bracket envelope",
        )

        roll_marker_rest = _aabb_center(
            ctx.part_element_world_aabb(outer_roll_frame, elem="roll_marker")
        )
        output_face_rest = _aabb_center(
            ctx.part_element_world_aabb(inner_pitch_cradle, elem="output_face")
        )

    with ctx.pose({base_to_roll: 0.75, roll_to_pitch: 0.0}):
        roll_marker_rolled = _aabb_center(
            ctx.part_element_world_aabb(outer_roll_frame, elem="roll_marker")
        )

    ctx.check(
        "positive roll carries the top marker toward negative y",
        roll_marker_rest is not None
        and roll_marker_rolled is not None
        and roll_marker_rolled[1] < roll_marker_rest[1] - 0.020,
        details=f"rest={roll_marker_rest}, rolled={roll_marker_rolled}",
    )

    with ctx.pose({base_to_roll: 0.0, roll_to_pitch: 0.65}):
        output_face_pitched = _aabb_center(
            ctx.part_element_world_aabb(inner_pitch_cradle, elem="output_face")
        )
        ctx.expect_within(
            inner_pitch_cradle,
            outer_roll_frame,
            axes="yz",
            margin=0.003,
            name="cradle remains captured by the roll frame at positive pitch",
        )

    ctx.check(
        "positive pitch lifts the output face upward",
        output_face_rest is not None
        and output_face_pitched is not None
        and output_face_pitched[2] > output_face_rest[2] + 0.020,
        details=f"rest={output_face_rest}, pitched={output_face_pitched}",
    )

    with ctx.pose({base_to_roll: -0.65, roll_to_pitch: 0.55}):
        ctx.expect_within(
            inner_pitch_cradle,
            outer_roll_frame,
            axes="yz",
            margin=0.003,
            name="combined roll and pitch keep the cradle inside the ring envelope",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
