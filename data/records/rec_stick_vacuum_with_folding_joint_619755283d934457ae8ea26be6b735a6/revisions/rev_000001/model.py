from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_stick_vacuum")

    charcoal = model.material("charcoal_plastic", rgba=(0.025, 0.027, 0.030, 1.0))
    dark = model.material("dark_rubber", rgba=(0.005, 0.005, 0.006, 1.0))
    graphite = model.material("graphite_frame", rgba=(0.23, 0.25, 0.27, 1.0))
    warm_gray = model.material("warm_gray_shell", rgba=(0.66, 0.67, 0.66, 1.0))
    red = model.material("anodized_red", rgba=(0.78, 0.06, 0.035, 1.0))
    steel = model.material("brushed_steel", rgba=(0.72, 0.72, 0.68, 1.0))
    dust_tint = model.material("smoked_clear_bin", rgba=(0.54, 0.70, 0.78, 0.45))

    body = model.part("motor_body")
    # The root frame is the fold hinge centerline.  The mass and handle sit
    # above it, leaving the hinge visually exposed as a light fork.
    body.visual(
        Cylinder(radius=0.068, length=0.185),
        origin=Origin(xyz=(0.000, 0.000, 0.165)),
        material=dust_tint,
        name="dust_cup",
    )
    body.visual(
        Cylinder(radius=0.098, length=0.180),
        origin=Origin(xyz=(-0.050, 0.000, 0.285), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=warm_gray,
        name="motor_pod",
    )
    body.visual(
        Box((0.165, 0.095, 0.055)),
        origin=Origin(xyz=(-0.115, 0.000, 0.225)),
        material=charcoal,
        name="battery_pack",
    )
    body.visual(
        Box((0.210, 0.038, 0.035)),
        origin=Origin(xyz=(-0.170, 0.000, 0.455)),
        material=charcoal,
        name="handle_grip",
    )
    body.visual(
        Box((0.040, 0.038, 0.205)),
        origin=Origin(xyz=(-0.270, 0.000, 0.355)),
        material=charcoal,
        name="handle_rear_post",
    )
    body.visual(
        Box((0.175, 0.035, 0.032)),
        origin=Origin(xyz=(-0.176, 0.000, 0.285)),
        material=charcoal,
        name="handle_lower_strut",
    )
    body.visual(
        Box((0.034, 0.014, 0.150)),
        origin=Origin(xyz=(-0.012, 0.055, 0.095)),
        material=graphite,
        name="fold_side_strut_0",
    )
    body.visual(
        Box((0.034, 0.014, 0.150)),
        origin=Origin(xyz=(-0.012, -0.055, 0.095)),
        material=graphite,
        name="fold_side_strut_1",
    )
    body.visual(
        Box((0.060, 0.014, 0.090)),
        origin=Origin(xyz=(0.000, 0.055, 0.000)),
        material=graphite,
        name="fold_cheek_0",
    )
    body.visual(
        Box((0.060, 0.014, 0.090)),
        origin=Origin(xyz=(0.000, -0.055, 0.000)),
        material=graphite,
        name="fold_cheek_1",
    )
    body.visual(
        Cylinder(radius=0.028, length=0.012),
        origin=Origin(xyz=(0.000, 0.0665, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="fold_cap_0",
    )
    body.visual(
        Cylinder(radius=0.028, length=0.012),
        origin=Origin(xyz=(0.000, -0.0665, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="fold_cap_1",
    )

    wand = model.part("wand")
    wand.visual(
        Cylinder(radius=0.024, length=0.060),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="fold_barrel",
    )
    wand.visual(
        Cylinder(radius=0.006, length=0.112),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="fold_pin",
    )
    wand.visual(
        Box((0.034, 0.044, 0.090)),
        origin=Origin(xyz=(0.000, 0.000, -0.050)),
        material=charcoal,
        name="fold_lug",
    )
    wand.visual(
        Cylinder(radius=0.029, length=0.045),
        origin=Origin(xyz=(0.000, 0.000, -0.105)),
        material=graphite,
        name="top_collar",
    )
    wand.visual(
        Cylinder(radius=0.018, length=0.800),
        origin=Origin(xyz=(0.000, 0.000, -0.500)),
        material=red,
        name="straight_tube",
    )
    wand.visual(
        Cylinder(radius=0.025, length=0.070),
        origin=Origin(xyz=(0.000, 0.000, -0.905)),
        material=graphite,
        name="lower_collar",
    )
    wand.visual(
        Box((0.030, 0.035, 0.075)),
        origin=Origin(xyz=(0.018, 0.000, -0.935)),
        material=graphite,
        name="lower_neck",
    )
    wand.visual(
        Box((0.040, 0.100, 0.018)),
        origin=Origin(xyz=(0.018, 0.000, -0.960)),
        material=graphite,
        name="pitch_bridge",
    )
    wand.visual(
        Box((0.060, 0.012, 0.075)),
        origin=Origin(xyz=(0.018, 0.044, -1.000)),
        material=graphite,
        name="pitch_ear_0",
    )
    wand.visual(
        Box((0.060, 0.012, 0.075)),
        origin=Origin(xyz=(0.018, -0.044, -1.000)),
        material=graphite,
        name="pitch_ear_1",
    )

    head = model.part("floor_head")
    head.visual(
        Cylinder(radius=0.022, length=0.060),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="pitch_barrel",
    )
    head.visual(
        Cylinder(radius=0.005, length=0.098),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pitch_pin",
    )
    head.visual(
        Box((0.045, 0.045, 0.050)),
        origin=Origin(xyz=(0.020, 0.000, -0.035)),
        material=charcoal,
        name="pitch_neck",
    )
    head.visual(
        Box((0.340, 0.240, 0.055)),
        origin=Origin(xyz=(0.120, 0.000, -0.082)),
        material=warm_gray,
        name="nozzle_body",
    )
    head.visual(
        Box((0.035, 0.245, 0.035)),
        origin=Origin(xyz=(0.300, 0.000, -0.070)),
        material=dark,
        name="front_bumper",
    )
    head.visual(
        Cylinder(radius=0.022, length=0.220),
        origin=Origin(xyz=(0.175, 0.000, -0.052), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=red,
        name="brush_roller",
    )
    head.visual(
        Box((0.145, 0.170, 0.008)),
        origin=Origin(xyz=(0.140, 0.000, -0.113)),
        material=dark,
        name="suction_slot",
    )
    head.visual(
        Cylinder(radius=0.024, length=0.022),
        origin=Origin(xyz=(-0.035, 0.129, -0.088), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="rear_wheel_0",
    )
    head.visual(
        Cylinder(radius=0.024, length=0.022),
        origin=Origin(xyz=(-0.035, -0.129, -0.088), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="rear_wheel_1",
    )

    model.articulation(
        "fold_joint",
        ArticulationType.REVOLUTE,
        parent=body,
        child=wand,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=1.85),
    )
    model.articulation(
        "head_pitch",
        ArticulationType.REVOLUTE,
        parent=wand,
        child=head,
        origin=Origin(xyz=(0.000, 0.000, -1.000)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=-0.45, upper=0.40),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("motor_body")
    wand = object_model.get_part("wand")
    head = object_model.get_part("floor_head")
    fold = object_model.get_articulation("fold_joint")
    pitch = object_model.get_articulation("head_pitch")

    ctx.allow_overlap(
        body,
        wand,
        elem_a="fold_cheek_0",
        elem_b="fold_pin",
        reason="A narrow hinge pin intentionally passes through the open fork cheek.",
    )
    ctx.allow_overlap(
        body,
        wand,
        elem_a="fold_cheek_1",
        elem_b="fold_pin",
        reason="A narrow hinge pin intentionally passes through the open fork cheek.",
    )
    ctx.allow_overlap(
        wand,
        head,
        elem_a="pitch_ear_0",
        elem_b="pitch_pin",
        reason="The floor-head hinge pin is represented as captured through the fork ear.",
    )
    ctx.allow_overlap(
        wand,
        head,
        elem_a="pitch_ear_1",
        elem_b="pitch_pin",
        reason="The floor-head hinge pin is represented as captured through the fork ear.",
    )

    ctx.check(
        "fold joint is horizontal revolute",
        fold.articulation_type == ArticulationType.REVOLUTE and abs(fold.axis[1]) > 0.99,
        details=f"type={fold.articulation_type}, axis={fold.axis}",
    )
    ctx.check(
        "head pitch is horizontal revolute",
        pitch.articulation_type == ArticulationType.REVOLUTE and abs(pitch.axis[1]) > 0.99,
        details=f"type={pitch.articulation_type}, axis={pitch.axis}",
    )

    ctx.expect_gap(
        body,
        wand,
        axis="y",
        positive_elem="fold_cheek_0",
        negative_elem="fold_pin",
        max_penetration=0.010,
        name="fold pin capture is local in positive cheek",
    )
    ctx.expect_gap(
        wand,
        body,
        axis="y",
        positive_elem="fold_pin",
        negative_elem="fold_cheek_1",
        max_penetration=0.010,
        name="fold pin capture is local in negative cheek",
    )
    ctx.expect_gap(
        wand,
        head,
        axis="y",
        positive_elem="pitch_ear_0",
        negative_elem="pitch_pin",
        max_penetration=0.012,
        name="pitch pin capture is local in positive ear",
    )
    ctx.expect_gap(
        head,
        wand,
        axis="y",
        positive_elem="pitch_pin",
        negative_elem="pitch_ear_1",
        max_penetration=0.012,
        name="pitch pin capture is local in negative ear",
    )

    folded_z = None
    rest_aabb = ctx.part_element_world_aabb(wand, elem="straight_tube")
    rest_z = None if rest_aabb is None else rest_aabb[0][2]
    with ctx.pose({fold: 1.55}):
        folded_aabb = ctx.part_element_world_aabb(wand, elem="straight_tube")
        if folded_aabb is not None:
            folded_z = folded_aabb[0][2]
    ctx.check(
        "fold joint lifts the wand for storage",
        rest_z is not None and folded_z is not None and folded_z > rest_z + 0.40,
        details=f"rest tube bottom z={rest_z}, folded tube bottom z={folded_z}",
    )

    with ctx.pose({pitch: pitch.motion_limits.upper}):
        pitched_aabb = ctx.part_element_world_aabb(head, elem="front_bumper")
    with ctx.pose({pitch: pitch.motion_limits.lower}):
        lowered_aabb = ctx.part_element_world_aabb(head, elem="front_bumper")
    pitched_z = None if pitched_aabb is None else pitched_aabb[0][2]
    lowered_z = None if lowered_aabb is None else lowered_aabb[0][2]
    ctx.check(
        "floor head visibly pitches about its hinge",
        pitched_z is not None and lowered_z is not None and pitched_z < lowered_z - 0.10,
        details=f"upper-limit front bumper min z={pitched_z}, lower-limit min z={lowered_z}",
    )

    return ctx.report()


object_model = build_object_model()
