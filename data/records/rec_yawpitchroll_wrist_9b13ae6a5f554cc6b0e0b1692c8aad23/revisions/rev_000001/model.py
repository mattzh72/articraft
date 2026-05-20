from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


BODY_TOP_Z = 0.074
PITCH_AXIS_Z = 0.070
ROLL_AXIS_X = 0.062


def _rounded_box(size: tuple[float, float, float], center: tuple[float, float, float], fillet: float) -> cq.Workplane:
    solid = cq.Workplane("XY").box(*size)
    if fillet > 0.0:
        solid = solid.edges("|Z").fillet(fillet)
    return solid.translate(center)


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cyl_z(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    solid = cq.Solid.makeCylinder(
        radius,
        length,
        cq.Vector(center[0], center[1], center[2] - length / 2.0),
        cq.Vector(0.0, 0.0, 1.0),
    )
    return cq.Workplane(obj=solid)


def _cyl_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    solid = cq.Solid.makeCylinder(
        radius,
        length,
        cq.Vector(center[0], center[1] - length / 2.0, center[2]),
        cq.Vector(0.0, 1.0, 0.0),
    )
    return cq.Workplane(obj=solid)


def _cyl_x(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    solid = cq.Solid.makeCylinder(
        radius,
        length,
        cq.Vector(center[0] - length / 2.0, center[1], center[2]),
        cq.Vector(1.0, 0.0, 0.0),
    )
    return cq.Workplane(obj=solid)


def _body_shape() -> cq.Workplane:
    base = _rounded_box((0.190, 0.150, 0.028), (0.0, 0.0, 0.014), 0.010)
    shoulder = _rounded_box((0.128, 0.104, 0.020), (0.0, 0.0, 0.038), 0.007)
    neck = _rounded_box((0.088, 0.068, 0.014), (0.0, 0.0, 0.052), 0.004)
    tower = _cyl_z(0.030, 0.028, (0.0, 0.0, 0.060))

    body = base.union(shoulder).union(neck).union(tower)
    spindle_bore = _cyl_z(0.0172, 0.036, (0.0, 0.0, 0.058))

    return body.cut(spindle_bore)


def _yaw_stage_shape() -> cq.Workplane:
    platter = _cyl_z(0.054, 0.012, (0.0, 0.0, 0.009))
    hub = _cyl_z(0.028, 0.044, (0.0, 0.0, 0.022))
    rear_bridge = _rounded_box((0.022, 0.070, 0.026), (-0.018, 0.0, 0.028), 0.003)
    left_tower = _rounded_box((0.036, 0.016, 0.074), (0.0, 0.039, 0.046), 0.003)
    right_tower = _rounded_box((0.036, 0.016, 0.074), (0.0, -0.039, 0.046), 0.003)

    stage = platter.union(hub).union(rear_bridge).union(left_tower).union(right_tower)

    left_bore = _cyl_y(0.0122, 0.022, (0.0, 0.039, PITCH_AXIS_Z))
    right_bore = _cyl_y(0.0122, 0.022, (0.0, -0.039, PITCH_AXIS_Z))
    center_window = _box((0.052, 0.050, 0.038), (0.012, 0.0, 0.040))

    return stage.cut(left_bore).cut(right_bore).cut(center_window)


def _pitch_cradle_shape() -> cq.Workplane:
    trunnion = _cyl_y(0.0122, 0.070, (0.0, 0.0, 0.0))
    spine = _rounded_box((0.060, 0.052, 0.018), (0.032, 0.0, 0.0), 0.004)
    nose_housing = _cyl_x(0.037, 0.018, (ROLL_AXIS_X, 0.0, 0.0))

    cradle = trunnion.union(spine).union(nose_housing)
    roll_bore = _cyl_x(0.0186, 0.022, (ROLL_AXIS_X, 0.0, 0.0))
    center_window = _box((0.026, 0.054, 0.008), (0.026, 0.0, 0.0))

    return cradle.cut(roll_bore).cut(center_window)


def _roll_flange_shape() -> cq.Workplane:
    shaft = _cyl_x(0.0186, 0.024, (0.0, 0.0, 0.0))
    front_hub = _cyl_x(0.024, 0.022, (0.021, 0.0, 0.0))
    flange = _cyl_x(0.040, 0.010, (0.037, 0.0, 0.0))
    key_lug = _rounded_box((0.010, 0.014, 0.020), (0.037, 0.0, 0.044), 0.002)

    flange_shape = shaft.union(front_hub).union(flange).union(key_lug)

    center_socket = _cyl_x(0.0105, 0.014, (0.039, 0.0, 0.0))
    bolt_pattern = (
        cq.Workplane("YZ")
        .pushPoints(((0.026, 0.0), (-0.026, 0.0), (0.0, 0.026), (0.0, -0.026)))
        .circle(0.003)
        .extrude(0.012)
        .translate((0.032, 0.0, 0.0))
    )

    return flange_shape.cut(center_socket).cut(bolt_pattern)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_three_axis_wrist")

    model.material("body_paint", rgba=(0.20, 0.22, 0.25, 1.0))
    model.material("stage_paint", rgba=(0.15, 0.16, 0.18, 1.0))
    model.material("cradle_finish", rgba=(0.67, 0.70, 0.73, 1.0))
    model.material("flange_finish", rgba=(0.80, 0.82, 0.85, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shape(), "service_wrist_body"),
        material="body_paint",
        name="body_shell",
    )

    yaw_stage = model.part("yaw_stage")
    yaw_stage.visual(
        mesh_from_cadquery(_yaw_stage_shape(), "service_wrist_yaw_stage"),
        material="stage_paint",
        name="yaw_stage_shell",
    )

    pitch_cradle = model.part("pitch_cradle")
    pitch_cradle.visual(
        mesh_from_cadquery(_pitch_cradle_shape(), "service_wrist_pitch_cradle"),
        material="cradle_finish",
        name="pitch_cradle_shell",
    )

    roll_flange = model.part("roll_flange")
    roll_flange.visual(
        mesh_from_cadquery(_roll_flange_shape(), "service_wrist_roll_flange"),
        material="flange_finish",
        name="roll_flange_shell",
    )

    model.articulation(
        "body_to_yaw",
        ArticulationType.REVOLUTE,
        parent=body,
        child=yaw_stage,
        origin=Origin(xyz=(0.0, 0.0, BODY_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=2.0, lower=-pi, upper=pi),
    )
    model.articulation(
        "yaw_to_pitch",
        ArticulationType.REVOLUTE,
        parent=yaw_stage,
        child=pitch_cradle,
        origin=Origin(xyz=(0.0, 0.0, PITCH_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.5, lower=-1.45, upper=1.45),
    )
    model.articulation(
        "pitch_to_roll",
        ArticulationType.REVOLUTE,
        parent=pitch_cradle,
        child=roll_flange,
        origin=Origin(xyz=(ROLL_AXIS_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=4.0, lower=-pi, upper=pi),
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

    body = object_model.get_part("body")
    yaw_stage = object_model.get_part("yaw_stage")
    pitch_cradle = object_model.get_part("pitch_cradle")
    roll_flange = object_model.get_part("roll_flange")

    yaw_joint = object_model.get_articulation("body_to_yaw")
    pitch_joint = object_model.get_articulation("yaw_to_pitch")
    roll_joint = object_model.get_articulation("pitch_to_roll")

    roll_shell = roll_flange.get_visual("roll_flange_shell")

    ctx.allow_overlap(
        yaw_stage,
        pitch_cradle,
        reason="The pitch cradle rides in tight revolute trunnion bores inside the yaw-stage side supports.",
    )
    ctx.allow_overlap(
        pitch_cradle,
        roll_flange,
        reason="The compact roll flange is represented as a close journal fit inside the pitch-cradle nose housing.",
    )

    ctx.expect_origin_gap(
        yaw_stage,
        body,
        axis="z",
        min_gap=0.070,
        max_gap=0.078,
        name="yaw stage sits on top of the grounded body",
    )
    ctx.expect_origin_gap(
        pitch_cradle,
        yaw_stage,
        axis="z",
        min_gap=0.068,
        max_gap=0.072,
        name="pitch cradle axis rides above the yaw table",
    )
    ctx.expect_origin_gap(
        roll_flange,
        pitch_cradle,
        axis="x",
        min_gap=0.058,
        max_gap=0.066,
        name="roll flange is carried forward by the pitch cradle",
    )

    rest_roll_pos = ctx.part_world_position(roll_flange)
    with ctx.pose({yaw_joint: 0.65}):
        yawed_roll_pos = ctx.part_world_position(roll_flange)
    ctx.check(
        "positive yaw swings the wrist toward +Y",
        rest_roll_pos is not None
        and yawed_roll_pos is not None
        and yawed_roll_pos[1] > rest_roll_pos[1] + 0.03
        and yawed_roll_pos[0] < rest_roll_pos[0] - 0.01,
        details=f"rest={rest_roll_pos}, yawed={yawed_roll_pos}",
    )

    with ctx.pose({pitch_joint: 0.75}):
        pitched_roll_pos = ctx.part_world_position(roll_flange)
    ctx.check(
        "positive pitch raises the roll flange",
        rest_roll_pos is not None
        and pitched_roll_pos is not None
        and pitched_roll_pos[2] > rest_roll_pos[2] + 0.03
        and pitched_roll_pos[0] < rest_roll_pos[0] - 0.01,
        details=f"rest={rest_roll_pos}, pitched={pitched_roll_pos}",
    )

    rest_roll_aabb = ctx.part_element_world_aabb(roll_flange, elem=roll_shell)
    with ctx.pose({roll_joint: 1.15}):
        rolled_aabb = ctx.part_element_world_aabb(roll_flange, elem=roll_shell)
    ctx.check(
        "positive roll rotates the keyed flange about +X",
        rest_roll_aabb is not None
        and rolled_aabb is not None
        and rolled_aabb[0][1] < rest_roll_aabb[0][1] - 0.006
        and rolled_aabb[1][1] > rest_roll_aabb[1][1] + 0.006,
        details=f"rest_aabb={rest_roll_aabb}, rolled_aabb={rolled_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
