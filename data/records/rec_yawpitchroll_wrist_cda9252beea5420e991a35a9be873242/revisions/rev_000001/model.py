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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


SUPPORT_WIDTH = 0.180
SUPPORT_DEPTH = 0.120
SUPPORT_THICKNESS = 0.018
SUPPORT_RING_OUTER_RADIUS = 0.046
SUPPORT_RING_INNER_RADIUS = 0.025
SUPPORT_RING_DEPTH = 0.018
SUPPORT_SLOT_LENGTH = 0.036
SUPPORT_SLOT_WIDTH = 0.012
SUPPORT_SLOT_SPACING = 0.110

YAW_NECK_RADIUS = 0.022
YAW_SHOULDER_RADIUS = 0.034
YAW_BODY_RADIUS = 0.031
YAW_BODY_LENGTH = 0.058
PITCH_AXIS_Z = -0.100
YAW_CHEEK_Y = 0.045
YAW_CHEEK_THICKNESS = 0.012

TRUNNION_RADIUS = 0.017
TRUNNION_LENGTH = 0.078
CRADLE_ARM_Y = 0.028
ROLL_AXIS_X = 0.060
ROLL_AXIS_Z = -0.040
BEARING_OUTER_RADIUS = 0.027
BEARING_INNER_RADIUS = 0.0135
BEARING_LENGTH = 0.024

TOOL_SHAFT_RADIUS = 0.0105
TOOL_SHAFT_LENGTH = 0.034


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="underslung_three_axis_wrist")

    model.material("painted_steel", rgba=(0.24, 0.27, 0.30, 1.0))
    model.material("machined_aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("anodized_dark", rgba=(0.20, 0.22, 0.25, 1.0))
    model.material("tool_steel", rgba=(0.58, 0.61, 0.66, 1.0))
    model.material("accent_orange", rgba=(0.86, 0.45, 0.13, 1.0))

    top_support = model.part("top_support")
    top_support.visual(
        mesh_from_cadquery(_make_top_support_shape(), "top_support"),
        material="painted_steel",
        name="support_frame",
    )
    top_support.inertial = Inertial.from_geometry(
        Box((SUPPORT_WIDTH, SUPPORT_DEPTH, SUPPORT_THICKNESS + 0.030)),
        mass=2.3,
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
    )

    yaw_stage = model.part("yaw_stage")
    yaw_stage.visual(
        Cylinder(radius=YAW_SHOULDER_RADIUS, length=0.008),
        material="machined_aluminum",
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        name="yaw_shoulder",
    )
    yaw_stage.visual(
        Cylinder(radius=YAW_BODY_RADIUS, length=0.059),
        material="machined_aluminum",
        origin=Origin(xyz=(0.0, 0.0, -0.0365)),
        name="yaw_body",
    )
    yaw_stage.visual(
        Box((0.050, 0.078, 0.012)),
        material="machined_aluminum",
        origin=Origin(xyz=(0.004, 0.0, -0.030)),
        name="clevis_bridge",
    )
    yaw_stage.visual(
        Box((0.056, YAW_CHEEK_THICKNESS, 0.090)),
        material="machined_aluminum",
        origin=Origin(xyz=(0.006, YAW_CHEEK_Y, -0.076)),
        name="left_cheek",
    )
    yaw_stage.visual(
        Box((0.056, YAW_CHEEK_THICKNESS, 0.090)),
        material="machined_aluminum",
        origin=Origin(xyz=(0.006, -YAW_CHEEK_Y, -0.076)),
        name="right_cheek",
    )
    yaw_stage.inertial = Inertial.from_geometry(
        Box((0.100, 0.100, 0.125)),
        mass=1.4,
        origin=Origin(xyz=(0.004, 0.0, -0.052)),
    )

    pitch_cradle = model.part("pitch_cradle")
    pitch_cradle.visual(
        Cylinder(radius=TRUNNION_RADIUS, length=0.070),
        material="anodized_dark",
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        name="trunnion_tube",
    )
    pitch_cradle.visual(
        Cylinder(radius=0.012, length=0.008),
        material="machined_aluminum",
        origin=Origin(xyz=(0.0, 0.035, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        name="left_journal",
    )
    pitch_cradle.visual(
        Cylinder(radius=0.012, length=0.008),
        material="machined_aluminum",
        origin=Origin(xyz=(0.0, -0.035, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        name="right_journal",
    )
    pitch_cradle.visual(
        Box((0.050, 0.010, 0.070)),
        material="anodized_dark",
        origin=Origin(xyz=(0.040, CRADLE_ARM_Y, -0.044)),
        name="left_arm",
    )
    pitch_cradle.visual(
        Box((0.050, 0.010, 0.070)),
        material="anodized_dark",
        origin=Origin(xyz=(0.040, -CRADLE_ARM_Y, -0.044)),
        name="right_arm",
    )
    pitch_cradle.visual(
        Box((0.018, 0.008, 0.020)),
        material="anodized_dark",
        origin=Origin(xyz=(0.020, 0.022, -0.015)),
        name="left_link",
    )
    pitch_cradle.visual(
        Box((0.018, 0.008, 0.020)),
        material="anodized_dark",
        origin=Origin(xyz=(0.020, -0.022, -0.015)),
        name="right_link",
    )
    pitch_cradle.visual(
        Box((0.020, 0.050, 0.014)),
        material="anodized_dark",
        origin=Origin(xyz=(0.049, 0.0, -0.069)),
        name="front_bridge",
    )
    pitch_cradle.visual(
        mesh_from_cadquery(_make_bearing_housing_shape(), "pitch_bearing_housing"),
        material="machined_aluminum",
        name="bearing_housing",
    )
    pitch_cradle.inertial = Inertial.from_geometry(
        Box((0.090, 0.075, 0.090)),
        mass=0.95,
        origin=Origin(xyz=(0.030, 0.0, -0.040)),
    )

    roll_output = model.part("roll_output")
    roll_output.visual(
        Cylinder(radius=TOOL_SHAFT_RADIUS, length=TOOL_SHAFT_LENGTH),
        origin=Origin(xyz=(0.001, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="tool_steel",
        name="tool_shaft",
    )
    roll_output.visual(
        mesh_from_cadquery(_make_roll_head_shape(), "roll_output_head"),
        material="tool_steel",
        name="tool_head",
    )
    roll_output.visual(
        Box((0.014, 0.010, 0.008)),
        origin=Origin(xyz=(0.022, 0.0, 0.019)),
        material="accent_orange",
        name="index_key",
    )
    roll_output.inertial = Inertial.from_geometry(
        Cylinder(radius=0.023, length=0.048),
        mass=0.38,
        origin=Origin(xyz=(0.018, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
    )

    model.articulation(
        "support_yaw",
        ArticulationType.REVOLUTE,
        parent=top_support,
        child=yaw_stage,
        origin=Origin(xyz=(0.0, 0.0, -SUPPORT_RING_DEPTH)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-2.6,
            upper=2.6,
            effort=28.0,
            velocity=1.8,
        ),
    )
    model.articulation(
        "yaw_to_pitch",
        ArticulationType.REVOLUTE,
        parent=yaw_stage,
        child=pitch_cradle,
        origin=Origin(xyz=(0.0, 0.0, PITCH_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-1.20,
            upper=1.15,
            effort=18.0,
            velocity=2.2,
        ),
    )
    model.articulation(
        "pitch_to_roll",
        ArticulationType.REVOLUTE,
        parent=pitch_cradle,
        child=roll_output,
        origin=Origin(xyz=(ROLL_AXIS_X, 0.0, ROLL_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-2.8,
            upper=2.8,
            effort=9.0,
            velocity=3.5,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    top_support = object_model.get_part("top_support")
    yaw_stage = object_model.get_part("yaw_stage")
    pitch_cradle = object_model.get_part("pitch_cradle")
    roll_output = object_model.get_part("roll_output")

    support_yaw = object_model.get_articulation("support_yaw")
    yaw_to_pitch = object_model.get_articulation("yaw_to_pitch")
    pitch_to_roll = object_model.get_articulation("pitch_to_roll")

    ctx.check(
        "all wrist parts are present",
        all(part is not None for part in (top_support, yaw_stage, pitch_cradle, roll_output)),
    )
    ctx.check(
        "all wrist joints are present",
        all(joint is not None for joint in (support_yaw, yaw_to_pitch, pitch_to_roll)),
    )
    ctx.check(
        "yaw pitch and roll axes follow the requested directions",
        support_yaw.axis == (0.0, 0.0, 1.0)
        and yaw_to_pitch.axis == (0.0, -1.0, 0.0)
        and pitch_to_roll.axis == (1.0, 0.0, 0.0),
        details=(
            f"yaw={support_yaw.axis}, pitch={yaw_to_pitch.axis}, roll={pitch_to_roll.axis}"
        ),
    )

    ctx.expect_within(
        roll_output,
        pitch_cradle,
        axes="yz",
        inner_elem="tool_shaft",
        outer_elem="bearing_housing",
        margin=0.0035,
        name="roll shaft stays centered inside the cradle bearing housing",
    )
    ctx.expect_overlap(
        roll_output,
        pitch_cradle,
        axes="x",
        elem_a="tool_shaft",
        elem_b="bearing_housing",
        min_overlap=0.022,
        name="roll shaft remains engaged across the housing length",
    )

    rest_output_pos = ctx.part_world_position(roll_output)
    with ctx.pose({support_yaw: 0.75}):
        yawed_output_pos = ctx.part_world_position(roll_output)
    ctx.check(
        "positive yaw swings the wrist toward +Y",
        rest_output_pos is not None
        and yawed_output_pos is not None
        and yawed_output_pos[1] > rest_output_pos[1] + 0.030,
        details=f"rest={rest_output_pos}, yawed={yawed_output_pos}",
    )

    with ctx.pose({yaw_to_pitch: 0.65}):
        pitched_output_pos = ctx.part_world_position(roll_output)
    ctx.check(
        "positive pitch lifts the roll output upward",
        rest_output_pos is not None
        and pitched_output_pos is not None
        and pitched_output_pos[2] > rest_output_pos[2] + 0.025,
        details=f"rest={rest_output_pos}, pitched={pitched_output_pos}",
    )

    rest_key_center = _aabb_center(ctx.part_element_world_aabb(roll_output, elem="index_key"))
    with ctx.pose({pitch_to_roll: pi / 2.0}):
        rolled_key_center = _aabb_center(
            ctx.part_element_world_aabb(roll_output, elem="index_key")
        )
    ctx.check(
        "roll joint spins an off-axis key around the tool axis",
        rest_key_center is not None
        and rolled_key_center is not None
        and abs(rolled_key_center[0] - rest_key_center[0]) < 0.004
        and rolled_key_center[1] < rest_key_center[1] - 0.012
        and rolled_key_center[2] < rest_key_center[2] - 0.012,
        details=f"rest_key={rest_key_center}, rolled_key={rolled_key_center}",
    )

    return ctx.report()


def _aabb_center(aabb):
    if aabb is None:
        return None
    min_corner, max_corner = aabb
    return tuple((min_corner[i] + max_corner[i]) * 0.5 for i in range(3))


def _make_top_support_shape():
    support = cq.Workplane("XY").box(
        SUPPORT_WIDTH,
        SUPPORT_DEPTH,
        SUPPORT_THICKNESS,
        centered=(True, True, False),
    )
    support = (
        support.faces(">Z")
        .workplane()
        .pushPoints(
            [
                (-SUPPORT_SLOT_SPACING * 0.5, 0.0),
                (SUPPORT_SLOT_SPACING * 0.5, 0.0),
            ]
        )
        .rect(0.026, 0.090)
        .extrude(0.016)
    )
    support = (
        support.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-SUPPORT_SLOT_SPACING * 0.5, 0.0),
                (SUPPORT_SLOT_SPACING * 0.5, 0.0),
            ]
        )
        .slot2D(SUPPORT_SLOT_LENGTH, SUPPORT_SLOT_WIDTH)
        .cutBlind(-(SUPPORT_THICKNESS + 0.020))
    )
    support = (
        support.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .circle(SUPPORT_RING_INNER_RADIUS)
        .cutThruAll()
    )

    bearing_ring = (
        cq.Workplane("XY")
        .circle(SUPPORT_RING_OUTER_RADIUS)
        .circle(SUPPORT_RING_INNER_RADIUS)
        .extrude(-SUPPORT_RING_DEPTH)
    )
    front_web = cq.Workplane("XY").box(0.062, 0.012, SUPPORT_RING_DEPTH).translate(
        (0.0, 0.030, -SUPPORT_RING_DEPTH * 0.5)
    )
    rear_web = cq.Workplane("XY").box(0.062, 0.012, SUPPORT_RING_DEPTH).translate(
        (0.0, -0.030, -SUPPORT_RING_DEPTH * 0.5)
    )
    return support.union(bearing_ring).union(front_web).union(rear_web)


def _make_yaw_stage_shape():
    shoulder = cq.Workplane("XY").circle(YAW_SHOULDER_RADIUS).extrude(-0.008)
    body = (
        cq.Workplane("XY")
        .workplane(offset=-0.008)
        .circle(YAW_BODY_RADIUS)
        .extrude(-YAW_BODY_LENGTH)
    )
    clevis_bridge = cq.Workplane("XY").box(0.050, 0.078, 0.012).translate(
        (0.004, 0.0, -0.030)
    )
    left_cheek = cq.Workplane("XY").box(0.056, YAW_CHEEK_THICKNESS, 0.090).translate(
        (0.006, YAW_CHEEK_Y, -0.076)
    )
    right_cheek = cq.Workplane("XY").box(0.056, YAW_CHEEK_THICKNESS, 0.090).translate(
        (0.006, -YAW_CHEEK_Y, -0.076)
    )
    return shoulder.union(body).union(clevis_bridge).union(left_cheek).union(right_cheek)


def _make_pitch_cradle_shape():
    trunnion_tube = cq.Workplane("XZ").circle(TRUNNION_RADIUS).extrude(
        TRUNNION_LENGTH * 0.5,
        both=True,
    )
    left_arm = cq.Workplane("XY").box(0.050, 0.010, 0.070).translate(
        (0.040, CRADLE_ARM_Y, -0.044)
    )
    right_arm = cq.Workplane("XY").box(0.050, 0.010, 0.070).translate(
        (0.040, -CRADLE_ARM_Y, -0.044)
    )
    left_link = cq.Workplane("XY").box(0.018, 0.008, 0.020).translate(
        (0.020, 0.022, -0.015)
    )
    right_link = cq.Workplane("XY").box(0.018, 0.008, 0.020).translate(
        (0.020, -0.022, -0.015)
    )
    front_bridge = cq.Workplane("XY").box(0.020, 0.050, 0.014).translate(
        (0.049, 0.0, -0.069)
    )
    return (
        trunnion_tube.union(left_arm)
        .union(right_arm)
        .union(left_link)
        .union(right_link)
        .union(front_bridge)
    )


def _make_bearing_housing_shape():
    return (
        cq.Workplane("YZ")
        .circle(BEARING_OUTER_RADIUS)
        .circle(BEARING_INNER_RADIUS)
        .extrude(BEARING_LENGTH * 0.5, both=True)
        .translate((ROLL_AXIS_X, 0.0, ROLL_AXIS_Z))
    )


def _make_roll_head_shape():
    flange = (
        cq.Workplane("YZ")
        .circle(0.023)
        .extrude(0.005, both=True)
        .translate((0.017, 0.0, 0.0))
    )
    pilot = (
        cq.Workplane("YZ")
        .circle(0.014)
        .extrude(0.007, both=True)
        .translate((0.029, 0.0, 0.0))
    )
    face_plate = (
        cq.Workplane("YZ")
        .circle(0.018)
        .circle(0.006)
        .extrude(0.003, both=True)
        .translate((0.0385, 0.0, 0.0))
    )
    return flange.union(pilot).union(face_plate)


# >>> USER_CODE_END

object_model = build_object_model()
