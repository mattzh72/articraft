from __future__ import annotations

import math

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_stick_vacuum")

    graphite = Material("graphite", rgba=(0.05, 0.055, 0.06, 1.0))
    warm_gray = Material("warm_gray", rgba=(0.50, 0.52, 0.52, 1.0))
    satin_metal = Material("satin_metal", rgba=(0.76, 0.78, 0.76, 1.0))
    blue = Material("blue_accent", rgba=(0.04, 0.28, 0.80, 1.0))
    translucent_smoke = Material("translucent_smoke", rgba=(0.45, 0.70, 0.88, 0.42))
    rubber = Material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    brush_red = Material("brush_red", rgba=(0.85, 0.08, 0.035, 1.0))
    yellow = Material("yellow_trigger", rgba=(1.0, 0.68, 0.08, 1.0))

    motor_body = model.part("motor_body")
    motor_body.visual(
        Cylinder(radius=0.095, length=0.26),
        origin=Origin(xyz=(-0.18, 0.0, 0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="motor_pod",
    )
    motor_body.visual(
        Cylinder(radius=0.075, length=0.28),
        origin=Origin(xyz=(-0.29, 0.0, -0.060), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=translucent_smoke,
        name="dust_cup",
    )
    motor_body.visual(
        Cylinder(radius=0.070, length=0.055),
        origin=Origin(xyz=(-0.448, 0.0, 0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_gray,
        name="rear_filter",
    )
    motor_body.visual(
        Box((0.160, 0.090, 0.070)),
        origin=Origin(xyz=(-0.360, 0.0, -0.130)),
        material=graphite,
        name="battery_pack",
    )
    motor_body.visual(
        Cylinder(radius=0.018, length=0.270),
        origin=Origin(xyz=(-0.300, 0.0, 0.170), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="handle_grip",
    )
    motor_body.visual(
        Box((0.060, 0.040, 0.175)),
        origin=Origin(xyz=(-0.420, 0.0, 0.082)),
        material=rubber,
        name="rear_handle_strut",
    )
    motor_body.visual(
        Box((0.060, 0.040, 0.145)),
        origin=Origin(xyz=(-0.165, 0.0, 0.100)),
        material=rubber,
        name="front_handle_strut",
    )
    motor_body.visual(
        Box((0.052, 0.034, 0.032)),
        origin=Origin(xyz=(-0.235, 0.0, 0.138)),
        material=yellow,
        name="trigger_button",
    )
    motor_body.visual(
        Box((0.035, 0.125, 0.052)),
        origin=Origin(xyz=(-0.055, 0.0, 0.0)),
        material=graphite,
        name="fold_yoke_bridge",
    )
    for side, y in enumerate((-0.0475, 0.0475)):
        motor_body.visual(
            Box((0.060, 0.025, 0.066)),
            origin=Origin(xyz=(-0.015, y, 0.0)),
            material=graphite,
            name=f"fold_yoke_lug_{side}",
        )
    motor_body.visual(
        Cylinder(radius=0.012, length=0.140),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="fold_pin",
    )

    wand = model.part("wand")
    wand.visual(
        Cylinder(radius=0.028, length=0.058),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=blue,
        name="fold_barrel",
    )
    wand.visual(
        Cylinder(radius=0.021, length=0.780),
        origin=Origin(xyz=(0.415, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="straight_tube",
    )
    wand.visual(
        Cylinder(radius=0.028, length=0.080),
        origin=Origin(xyz=(0.780, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blue,
        name="end_collar",
    )
    wand.visual(
        Box((0.030, 0.120, 0.040)),
        origin=Origin(xyz=(0.815, 0.0, 0.0)),
        material=blue,
        name="end_yoke_bridge",
    )
    for side, y in enumerate((-0.0475, 0.0475)):
        wand.visual(
            Box((0.070, 0.023, 0.052)),
            origin=Origin(xyz=(0.855, y, 0.0)),
            material=blue,
            name=f"head_yoke_lug_{side}",
        )
    wand.visual(
        Cylinder(radius=0.009, length=0.120),
        origin=Origin(xyz=(0.860, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="head_pin",
    )

    floor_head = model.part("floor_head")
    floor_head.visual(
        Cylinder(radius=0.022, length=0.066),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="pitch_barrel",
    )
    floor_head.visual(
        Box((0.062, 0.050, 0.045)),
        origin=Origin(xyz=(0.049, 0.0, -0.015)),
        material=graphite,
        name="compact_neck",
    )
    floor_head.visual(
        Box((0.240, 0.300, 0.055)),
        origin=Origin(xyz=(0.190, 0.0, -0.055)),
        material=warm_gray,
        name="head_shell",
    )
    floor_head.visual(
        Box((0.030, 0.322, 0.040)),
        origin=Origin(xyz=(0.305, 0.0, -0.055)),
        material=rubber,
        name="front_bumper",
    )
    floor_head.visual(
        Box((0.150, 0.215, 0.012)),
        origin=Origin(xyz=(0.180, 0.0, -0.026)),
        material=graphite,
        name="suction_window",
    )
    floor_head.visual(
        Cylinder(radius=0.024, length=0.240),
        origin=Origin(xyz=(0.170, 0.0, -0.085), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brush_red,
        name="brush_roll",
    )
    for side, y in enumerate((-0.158, 0.158)):
        floor_head.visual(
            Cylinder(radius=0.022, length=0.018),
            origin=Origin(xyz=(0.075, y, -0.085), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name=f"side_wheel_{side}",
        )

    model.articulation(
        "fold_joint",
        ArticulationType.REVOLUTE,
        parent=motor_body,
        child=wand,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=0.0, upper=2.25),
    )
    model.articulation(
        "head_pitch",
        ArticulationType.REVOLUTE,
        parent=wand,
        child=floor_head,
        origin=Origin(xyz=(0.860, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=-0.80, upper=0.80),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    motor_body = object_model.get_part("motor_body")
    wand = object_model.get_part("wand")
    floor_head = object_model.get_part("floor_head")
    fold_joint = object_model.get_articulation("fold_joint")
    head_pitch = object_model.get_articulation("head_pitch")

    ctx.allow_overlap(
        motor_body,
        wand,
        elem_a="fold_pin",
        elem_b="fold_barrel",
        reason="The metal fold-hinge pin is intentionally captured through the wand barrel.",
    )
    ctx.expect_within(
        motor_body,
        wand,
        axes="xz",
        inner_elem="fold_pin",
        outer_elem="fold_barrel",
        margin=0.001,
        name="fold pin is centered inside fold barrel",
    )
    ctx.expect_overlap(
        wand,
        motor_body,
        axes="y",
        elem_a="fold_barrel",
        elem_b="fold_pin",
        min_overlap=0.055,
        name="fold barrel spans the fold pin",
    )

    ctx.allow_overlap(
        wand,
        floor_head,
        elem_a="head_pin",
        elem_b="pitch_barrel",
        reason="The compact head pitch pin is intentionally captured through the floor-head barrel.",
    )
    ctx.expect_within(
        wand,
        floor_head,
        axes="xz",
        inner_elem="head_pin",
        outer_elem="pitch_barrel",
        margin=0.001,
        name="head pin is centered inside pitch barrel",
    )
    ctx.expect_overlap(
        floor_head,
        wand,
        axes="y",
        elem_a="pitch_barrel",
        elem_b="head_pin",
        min_overlap=0.060,
        name="pitch barrel spans the head pin",
    )

    ctx.check(
        "fold joint is horizontal revolute",
        fold_joint.articulation_type == ArticulationType.REVOLUTE
        and abs(fold_joint.axis[1]) > 0.99
        and abs(fold_joint.axis[0]) < 1e-6
        and abs(fold_joint.axis[2]) < 1e-6,
        details=f"axis={fold_joint.axis}, type={fold_joint.articulation_type}",
    )
    ctx.check(
        "head joint is horizontal revolute",
        head_pitch.articulation_type == ArticulationType.REVOLUTE
        and abs(head_pitch.axis[1]) > 0.99
        and abs(head_pitch.axis[0]) < 1e-6
        and abs(head_pitch.axis[2]) < 1e-6,
        details=f"axis={head_pitch.axis}, type={head_pitch.articulation_type}",
    )

    neck_aabb = ctx.part_element_world_aabb(floor_head, elem="compact_neck")
    tube_aabb = ctx.part_element_world_aabb(wand, elem="straight_tube")
    if neck_aabb is not None and tube_aabb is not None:
        neck_length = neck_aabb[1][0] - neck_aabb[0][0]
        tube_length = tube_aabb[1][0] - tube_aabb[0][0]
    else:
        neck_length = tube_length = 0.0
    ctx.check(
        "compact wrist relative to wand",
        neck_length < 0.09 and tube_length > 0.70 and neck_length < tube_length * 0.14,
        details=f"neck_length={neck_length:.3f}, tube_length={tube_length:.3f}",
    )

    def _center_xz(aabb):
        return ((aabb[0][0] + aabb[1][0]) * 0.5, (aabb[0][2] + aabb[1][2]) * 0.5)

    rest_bridge = ctx.part_element_world_aabb(wand, elem="end_yoke_bridge")
    rest_center = _center_xz(rest_bridge) if rest_bridge is not None else None
    with ctx.pose({fold_joint: 1.35}):
        folded_bridge = ctx.part_element_world_aabb(wand, elem="end_yoke_bridge")
        folded_center = _center_xz(folded_bridge) if folded_bridge is not None else None
    ctx.check(
        "fold joint lifts wand end",
        rest_center is not None
        and folded_center is not None
        and folded_center[1] > rest_center[1] + 0.45
        and folded_center[0] < rest_center[0] - 0.45,
        details=f"rest={rest_center}, folded={folded_center}",
    )

    rest_head = ctx.part_element_world_aabb(floor_head, elem="head_shell")
    rest_head_center = _center_xz(rest_head) if rest_head is not None else None
    with ctx.pose({head_pitch: 0.55}):
        pitched_head = ctx.part_element_world_aabb(floor_head, elem="head_shell")
        pitched_center = _center_xz(pitched_head) if pitched_head is not None else None
    ctx.check(
        "floor head pitches about compact hinge",
        rest_head_center is not None
        and pitched_center is not None
        and abs(pitched_center[1] - rest_head_center[1]) > 0.05,
        details=f"rest={rest_head_center}, pitched={pitched_center}",
    )

    return ctx.report()


object_model = build_object_model()
