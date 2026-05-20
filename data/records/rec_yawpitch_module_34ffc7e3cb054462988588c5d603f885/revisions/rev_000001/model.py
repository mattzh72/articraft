from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PITCH_AXIS_Z = 0.185
BODY_TOP_Z = 0.105


def _cylinder_y(radius: float, length: float, *, y0: float, z: float, x: float = 0.0) -> cq.Workplane:
    """CadQuery cylinder whose axis runs along +Y from y0 to y0 + length."""
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -0.5 * length))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -90.0)
        .translate((x, y0 + 0.5 * length, z))
    )


def _body_geometry() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .box(0.46, 0.34, 0.070)
        .edges("|Z")
        .fillet(0.035)
        .translate((0.0, 0.0, 0.035))
    )
    bearing_plinth = cq.Workplane("XY").circle(0.140).extrude(0.036).translate((0.0, 0.0, 0.069))
    return base.union(bearing_plinth)


def _yaw_stage_geometry() -> cq.Workplane:
    turntable = cq.Workplane("XY").circle(0.106).extrude(0.041)
    rotor_cap = cq.Workplane("XY").circle(0.086).extrude(0.036).translate((0.0, 0.0, 0.039))
    saddle_deck = cq.Workplane("XY").box(0.170, 0.250, 0.033).translate((0.0, 0.0, 0.0915))

    cheek_blank = cq.Workplane("XY").box(0.130, 0.030, 0.158)
    pitch_bore = _cylinder_y(0.032, 0.080, y0=-0.040, z=0.0)
    cheek = cheek_blank.cut(pitch_bore)

    side_y = 0.112
    cheek_pos = cheek.translate((0.0, side_y, PITCH_AXIS_Z))
    cheek_neg = cheek.translate((0.0, -side_y, PITCH_AXIS_Z))

    outer_boss_pos = _cylinder_y(0.044, 0.012, y0=side_y + 0.013, z=PITCH_AXIS_Z)
    boss_bore_pos = _cylinder_y(0.026, 0.016, y0=side_y + 0.011, z=PITCH_AXIS_Z)
    outer_boss_pos = outer_boss_pos.cut(boss_bore_pos)

    outer_boss_neg = _cylinder_y(0.044, 0.012, y0=-(side_y + 0.025), z=PITCH_AXIS_Z)
    boss_bore_neg = _cylinder_y(0.026, 0.016, y0=-(side_y + 0.027), z=PITCH_AXIS_Z)
    outer_boss_neg = outer_boss_neg.cut(boss_bore_neg)

    stage = turntable.union(rotor_cap).union(saddle_deck)
    for feature in (cheek_pos, cheek_neg, outer_boss_pos, outer_boss_neg):
        stage = stage.union(feature)
    return stage


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_service_yaw_pitch_head")

    dark_body = Material("mat_dark_body", rgba=(0.07, 0.075, 0.08, 1.0))
    blue_metal = Material("mat_blue_metal", rgba=(0.12, 0.18, 0.24, 1.0))
    light_metal = Material("mat_light_metal", rgba=(0.62, 0.66, 0.68, 1.0))
    black = Material("mat_black_port", rgba=(0.005, 0.006, 0.007, 1.0))
    steel = Material("mat_burnished_steel", rgba=(0.45, 0.47, 0.47, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_geometry(), "grounded_body", tolerance=0.001),
        material=dark_body,
        name="grounded_body",
    )

    yaw_stage = model.part("yaw_stage")
    yaw_stage.visual(
        mesh_from_cadquery(_yaw_stage_geometry(), "yaw_stage_yoke", tolerance=0.001),
        material=blue_metal,
        name="yaw_stage_yoke",
    )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.021, length=0.276),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pitch_shaft",
    )
    cradle.visual(
        Box((0.125, 0.160, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=light_metal,
        name="cradle_block",
    )
    cradle.visual(
        Cylinder(radius=0.032, length=0.008),
        origin=Origin(xyz=(0.0, 0.141, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="shaft_cap_0",
    )
    cradle.visual(
        Cylinder(radius=0.032, length=0.008),
        origin=Origin(xyz=(0.0, -0.141, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="shaft_cap_1",
    )
    cradle.visual(
        Cylinder(radius=0.041, length=0.018),
        origin=Origin(xyz=(0.071, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=light_metal,
        name="front_face",
    )
    cradle.visual(
        Cylinder(radius=0.024, length=0.006),
        origin=Origin(xyz=(0.083, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="service_port",
    )

    model.articulation(
        "body_to_yaw",
        ArticulationType.REVOLUTE,
        parent=body,
        child=yaw_stage,
        origin=Origin(xyz=(0.0, 0.0, BODY_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.5, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "yaw_to_cradle",
        ArticulationType.REVOLUTE,
        parent=yaw_stage,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, PITCH_AXIS_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=-0.85, upper=0.85),
    )

    return model


def _axis_value(vec, index: int) -> float:
    try:
        return float(vec[index])
    except TypeError:
        return float((vec.x, vec.y, vec.z)[index])


def _aabb_center(aabb, index: int) -> float | None:
    if aabb is None:
        return None
    return 0.5 * (_axis_value(aabb[0], index) + _axis_value(aabb[1], index))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    yaw_stage = object_model.get_part("yaw_stage")
    cradle = object_model.get_part("cradle")
    yaw = object_model.get_articulation("body_to_yaw")
    pitch = object_model.get_articulation("yaw_to_cradle")

    ctx.check("yaw joint is vertical revolute", yaw.articulation_type == ArticulationType.REVOLUTE and tuple(yaw.axis) == (0.0, 0.0, 1.0))
    ctx.check("pitch joint is horizontal revolute", pitch.articulation_type == ArticulationType.REVOLUTE and tuple(pitch.axis) == (0.0, 1.0, 0.0))

    ctx.expect_gap(yaw_stage, body, axis="z", max_gap=0.002, max_penetration=0.0, name="yaw stage sits on grounded body")
    ctx.expect_overlap(yaw_stage, body, axes="xy", min_overlap=0.18, name="broad body supports compact rotary stage")
    ctx.expect_overlap(cradle, yaw_stage, axes="y", min_overlap=0.22, elem_a="pitch_shaft", elem_b="yaw_stage_yoke", name="pitch shaft spans the trunnion yoke")

    body_aabb = ctx.part_world_aabb(body)
    body_min_z = _axis_value(body_aabb[0], 2) if body_aabb is not None else None
    ctx.check("body is grounded", body_min_z is not None and abs(body_min_z) <= 0.001, details=f"body_min_z={body_min_z}")

    rest_port = ctx.part_element_world_aabb(cradle, elem="service_port")
    rest_y = _aabb_center(rest_port, 1)
    with ctx.pose({yaw: math.pi / 2.0}):
        yawed_port = ctx.part_element_world_aabb(cradle, elem="service_port")
        yawed_y = _aabb_center(yawed_port, 1)
    ctx.check(
        "positive yaw turns service face around vertical axis",
        rest_y is not None and yawed_y is not None and yawed_y > rest_y + 0.05,
        details=f"rest_y={rest_y}, yawed_y={yawed_y}",
    )

    rest_z = _aabb_center(rest_port, 2)
    with ctx.pose({pitch: 0.65}):
        pitched_port = ctx.part_element_world_aabb(cradle, elem="service_port")
        pitched_z = _aabb_center(pitched_port, 2)
    ctx.check(
        "positive pitch tips service face downward",
        rest_z is not None and pitched_z is not None and pitched_z < rest_z - 0.03,
        details=f"rest_z={rest_z}, pitched_z={pitched_z}",
    )

    return ctx.report()


object_model = build_object_model()
