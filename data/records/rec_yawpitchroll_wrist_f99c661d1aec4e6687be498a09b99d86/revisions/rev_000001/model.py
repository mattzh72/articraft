from __future__ import annotations

import cadquery as cq
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _tube_z(outer_radius: float, inner_radius: float, length: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length * 0.5, both=True)
    )


def _tube_x(outer_radius: float, inner_radius: float, length: float) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length * 0.5, both=True)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_cheek_yaw_pitch_roll_module")

    cast_aluminum = model.material("cast_aluminum", color=(0.62, 0.65, 0.66, 1.0))
    dark_anodized = model.material("dark_anodized", color=(0.05, 0.055, 0.06, 1.0))
    bearing_black = model.material("bearing_black", color=(0.01, 0.012, 0.014, 1.0))
    brushed_steel = model.material("brushed_steel", color=(0.78, 0.76, 0.70, 1.0))
    blue_index = model.material("blue_index", color=(0.08, 0.20, 0.75, 1.0))

    side_support = model.part("side_support")
    side_support.visual(
        Box((0.240, 0.150, 0.016)),
        origin=Origin(xyz=(0.030, -0.050, 0.008)),
        material=cast_aluminum,
        name="base_foot",
    )
    side_support.visual(
        Box((0.200, 0.018, 0.185)),
        origin=Origin(xyz=(0.015, -0.115, 0.102)),
        material=cast_aluminum,
        name="side_cheek",
    )
    side_support.visual(
        Box((0.060, 0.066, 0.026)),
        origin=Origin(xyz=(0.000, -0.081, 0.180)),
        material=cast_aluminum,
        name="bearing_arm",
    )
    side_support.visual(
        Box((0.038, 0.045, 0.085)),
        origin=Origin(xyz=(-0.060, -0.088, 0.075)),
        material=cast_aluminum,
        name="front_gusset",
    )
    side_support.visual(
        Box((0.038, 0.045, 0.085)),
        origin=Origin(xyz=(0.080, -0.088, 0.075)),
        material=cast_aluminum,
        name="rear_gusset",
    )
    side_support.visual(
        mesh_from_cadquery(_tube_z(0.052, 0.040, 0.032), "yaw_bearing"),
        origin=Origin(xyz=(0.000, 0.000, 0.180)),
        material=brushed_steel,
        name="yaw_bearing",
    )
    side_support.visual(
        Cylinder(radius=0.005, length=0.004),
        origin=Origin(xyz=(-0.090, -0.050, 0.018)),
        material=bearing_black,
        name="mount_bolt_0",
    )
    side_support.visual(
        Cylinder(radius=0.005, length=0.004),
        origin=Origin(xyz=(0.130, -0.050, 0.018)),
        material=bearing_black,
        name="mount_bolt_1",
    )

    yaw_cartridge = model.part("yaw_cartridge")
    yaw_cartridge.visual(
        Cylinder(radius=0.032, length=0.056),
        origin=Origin(),
        material=dark_anodized,
        name="yaw_drum",
    )
    yaw_cartridge.visual(
        Cylinder(radius=0.047, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=brushed_steel,
        name="upper_thrust_washer",
    )
    yaw_cartridge.visual(
        Cylinder(radius=0.047, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.019)),
        material=brushed_steel,
        name="lower_thrust_washer",
    )
    yaw_cartridge.visual(
        Cylinder(radius=0.038, length=0.009),
        origin=Origin(xyz=(0.0, 0.0, 0.0315)),
        material=brushed_steel,
        name="upper_retainer",
    )
    yaw_cartridge.visual(
        Cylinder(radius=0.038, length=0.009),
        origin=Origin(xyz=(0.0, 0.0, -0.0315)),
        material=brushed_steel,
        name="lower_retainer",
    )
    yaw_cartridge.visual(
        Box((0.076, 0.070, 0.018)),
        origin=Origin(xyz=(0.060, 0.000, -0.032)),
        material=dark_anodized,
        name="pitch_saddle",
    )
    yaw_cartridge.visual(
        Box((0.028, 0.012, 0.070)),
        origin=Origin(xyz=(0.105, 0.036, 0.000)),
        material=dark_anodized,
        name="pitch_ear_0",
    )
    yaw_cartridge.visual(
        Box((0.028, 0.012, 0.070)),
        origin=Origin(xyz=(0.105, -0.036, 0.000)),
        material=dark_anodized,
        name="pitch_ear_1",
    )
    yaw_cartridge.visual(
        Cylinder(radius=0.016, length=0.006),
        origin=Origin(xyz=(0.105, 0.0445, 0.000), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="pitch_pin_cap_0",
    )
    yaw_cartridge.visual(
        Cylinder(radius=0.016, length=0.006),
        origin=Origin(xyz=(0.105, -0.0445, 0.000), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="pitch_pin_cap_1",
    )
    yaw_cartridge.visual(
        Box((0.006, 0.020, 0.040)),
        origin=Origin(xyz=(-0.032, 0.000, 0.000)),
        material=blue_index,
        name="yaw_index_mark",
    )

    pitch_yoke = model.part("pitch_yoke")
    pitch_yoke.visual(
        Cylinder(radius=0.014, length=0.045),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="pitch_trunnion",
    )
    pitch_yoke.visual(
        Box((0.083, 0.008, 0.038)),
        origin=Origin(xyz=(0.040, 0.027, 0.000)),
        material=cast_aluminum,
        name="yoke_cheek_0",
    )
    pitch_yoke.visual(
        Box((0.083, 0.008, 0.038)),
        origin=Origin(xyz=(0.040, -0.027, 0.000)),
        material=cast_aluminum,
        name="yoke_cheek_1",
    )
    pitch_yoke.visual(
        Box((0.014, 0.054, 0.022)),
        origin=Origin(xyz=(0.008, 0.000, 0.000)),
        material=cast_aluminum,
        name="rear_bridge",
    )
    pitch_yoke.visual(
        mesh_from_cadquery(_tube_x(0.026, 0.020, 0.018), "roll_bearing"),
        origin=Origin(xyz=(0.075, 0.000, 0.000)),
        material=brushed_steel,
        name="roll_bearing",
    )

    roll_nose = model.part("roll_nose")
    roll_nose.visual(
        Cylinder(radius=0.016, length=0.104),
        origin=Origin(xyz=(0.038, 0.000, 0.000), rpy=(0.0, pi / 2.0, 0.0)),
        material=bearing_black,
        name="nose_shaft",
    )
    roll_nose.visual(
        Cylinder(radius=0.023, length=0.004),
        origin=Origin(xyz=(-0.011, 0.000, 0.000), rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed_steel,
        name="rear_thrust_collar",
    )
    roll_nose.visual(
        Cylinder(radius=0.023, length=0.004),
        origin=Origin(xyz=(0.011, 0.000, 0.000), rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed_steel,
        name="front_thrust_collar",
    )
    roll_nose.visual(
        Sphere(radius=0.016),
        origin=Origin(xyz=(0.092, 0.000, 0.000)),
        material=bearing_black,
        name="rounded_tip",
    )
    roll_nose.visual(
        Cylinder(radius=0.011, length=0.004),
        origin=Origin(xyz=(0.109, 0.000, 0.000), rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed_steel,
        name="front_face",
    )
    roll_nose.visual(
        Box((0.030, 0.006, 0.008)),
        origin=Origin(xyz=(0.045, 0.000, 0.020)),
        material=blue_index,
        name="roll_index_fin",
    )

    model.articulation(
        "yaw_joint",
        ArticulationType.REVOLUTE,
        parent=side_support,
        child=yaw_cartridge,
        origin=Origin(xyz=(0.000, 0.000, 0.180)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=-0.75, upper=0.75),
    )
    model.articulation(
        "pitch_joint",
        ArticulationType.REVOLUTE,
        parent=yaw_cartridge,
        child=pitch_yoke,
        origin=Origin(xyz=(0.105, 0.000, 0.000)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=-0.55, upper=0.75),
    )
    model.articulation(
        "roll_joint",
        ArticulationType.REVOLUTE,
        parent=pitch_yoke,
        child=roll_nose,
        origin=Origin(xyz=(0.075, 0.000, 0.000)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=4.0, lower=-1.4, upper=1.4),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    yaw = object_model.get_articulation("yaw_joint")
    pitch = object_model.get_articulation("pitch_joint")
    roll = object_model.get_articulation("roll_joint")
    side_support = object_model.get_part("side_support")
    yaw_cartridge = object_model.get_part("yaw_cartridge")
    pitch_yoke = object_model.get_part("pitch_yoke")
    roll_nose = object_model.get_part("roll_nose")

    ctx.check(
        "three revolute joints",
        len(object_model.articulations) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in object_model.articulations),
        details=f"joints={[j.name for j in object_model.articulations]}",
    )
    ctx.check("yaw axis is vertical", tuple(round(v, 3) for v in yaw.axis) == (0.0, 0.0, 1.0))
    ctx.check("pitch axis is transverse", tuple(round(v, 3) for v in pitch.axis) == (0.0, -1.0, 0.0))
    ctx.check("roll axis is longitudinal", tuple(round(v, 3) for v in roll.axis) == (1.0, 0.0, 0.0))

    ctx.expect_within(
        yaw_cartridge,
        side_support,
        axes="xy",
        inner_elem="yaw_drum",
        outer_elem="yaw_bearing",
        margin=0.0,
        name="yaw cartridge sits inside side bearing footprint",
    )
    ctx.expect_overlap(
        yaw_cartridge,
        side_support,
        axes="z",
        elem_a="yaw_drum",
        elem_b="yaw_bearing",
        min_overlap=0.025,
        name="yaw cartridge passes through bearing height",
    )
    ctx.expect_within(
        pitch_yoke,
        yaw_cartridge,
        axes="y",
        inner_elem="pitch_trunnion",
        outer_elem="pitch_saddle",
        margin=0.0,
        name="pitch trunnion fits between yaw ears",
    )
    ctx.expect_within(
        roll_nose,
        pitch_yoke,
        axes="yz",
        inner_elem="nose_shaft",
        outer_elem="roll_bearing",
        margin=0.0,
        name="roll shaft is centered in pitch bearing",
    )
    ctx.expect_overlap(
        roll_nose,
        pitch_yoke,
        axes="x",
        elem_a="nose_shaft",
        elem_b="roll_bearing",
        min_overlap=0.015,
        name="roll shaft remains captured through bearing",
    )

    rest_pitch_pos = ctx.part_world_position(pitch_yoke)
    with ctx.pose({yaw: 0.60}):
        yawed_pitch_pos = ctx.part_world_position(pitch_yoke)
    ctx.check(
        "yaw motion swings pitch stage sideways",
        rest_pitch_pos is not None
        and yawed_pitch_pos is not None
        and yawed_pitch_pos[1] > rest_pitch_pos[1] + 0.045,
        details=f"rest={rest_pitch_pos}, yawed={yawed_pitch_pos}",
    )

    rest_roll_pos = ctx.part_world_position(roll_nose)
    with ctx.pose({pitch: 0.60}):
        pitched_roll_pos = ctx.part_world_position(roll_nose)
    ctx.check(
        "positive pitch raises roll nose",
        rest_roll_pos is not None
        and pitched_roll_pos is not None
        and pitched_roll_pos[2] > rest_roll_pos[2] + 0.035,
        details=f"rest={rest_roll_pos}, pitched={pitched_roll_pos}",
    )

    rest_fin_aabb = ctx.part_element_world_aabb(roll_nose, elem="roll_index_fin")
    with ctx.pose({roll: 0.90}):
        rolled_fin_aabb = ctx.part_element_world_aabb(roll_nose, elem="roll_index_fin")
    if rest_fin_aabb is not None and rolled_fin_aabb is not None:
        rest_fin_y = (rest_fin_aabb[0][1] + rest_fin_aabb[1][1]) * 0.5
        rolled_fin_y = (rolled_fin_aabb[0][1] + rolled_fin_aabb[1][1]) * 0.5
    else:
        rest_fin_y = rolled_fin_y = None
    ctx.check(
        "roll motion carries the index fin around the nose",
        rest_fin_y is not None and rolled_fin_y is not None and abs(rolled_fin_y - rest_fin_y) > 0.010,
        details=f"rest_y={rest_fin_y}, rolled_y={rolled_fin_y}",
    )

    return ctx.report()


object_model = build_object_model()
