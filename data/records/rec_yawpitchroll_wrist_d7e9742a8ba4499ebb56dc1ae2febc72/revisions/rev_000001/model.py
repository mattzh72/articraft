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


def _annulus(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
    )


def _tube_x(outer_radius: float, inner_radius: float, length: float, x0: float) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((x0, 0.0, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="yaw_pitch_roll_wrist")

    dark_steel = Material("dark_burnished_steel", color=(0.08, 0.085, 0.09, 1.0))
    blue_casting = Material("blue_cast_aluminum", color=(0.10, 0.22, 0.42, 1.0))
    satin_steel = Material("satin_machined_steel", color=(0.62, 0.64, 0.62, 1.0))
    black_bearing = Material("black_bearing_rubber", color=(0.015, 0.015, 0.014, 1.0))

    # Fixed annular base, scaled like a compact industrial robot wrist.
    base_ring = model.part("base_ring")
    base_ring.visual(
        mesh_from_cadquery(_annulus(0.22, 0.12, 0.03), "base_annular_ring"),
        material=dark_steel,
        name="annular_ring",
    )
    for index in range(8):
        angle = 2.0 * math.pi * index / 8.0
        base_ring.visual(
            Cylinder(radius=0.011, length=0.008),
            origin=Origin(
                xyz=(0.175 * math.cos(angle), 0.175 * math.sin(angle), 0.034),
            ),
            material=satin_steel,
            name=f"bolt_head_{index}",
        )

    # The yawing casting carries the pitch yoke.  Its frame is on the yaw axis
    # at the top of the fixed base ring.
    pitch_yoke = model.part("pitch_yoke")
    pitch_yoke.visual(
        Cylinder(radius=0.148, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=blue_casting,
        name="yaw_turntable",
    )
    pitch_yoke.visual(
        Cylinder(radius=0.070, length=0.085),
        origin=Origin(xyz=(0.0, 0.0, 0.072)),
        material=blue_casting,
        name="central_column",
    )
    pitch_yoke.visual(
        Box((0.18, 0.30, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.118)),
        material=blue_casting,
        name="yoke_bridge",
    )
    for y, suffix in ((0.14, "0"), (-0.14, "1")):
        pitch_yoke.visual(
            Box((0.080, 0.040, 0.340)),
            origin=Origin(xyz=(0.0, y, 0.285)),
            material=blue_casting,
            name=f"yoke_cheek_{suffix}",
        )
        pitch_yoke.visual(
            Cylinder(radius=0.056, length=0.025),
            origin=Origin(
                xyz=(0.0, 0.1075 if y > 0.0 else -0.1075, 0.300),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=black_bearing,
            name=f"pitch_bearing_{suffix}",
        )

    # The pitch carriage is the trunnion and non-rotating roll housing.
    roll_carriage = model.part("roll_carriage")
    for y, suffix in ((0.072, "0"), (-0.072, "1")):
        roll_carriage.visual(
            Cylinder(radius=0.032, length=0.046),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=satin_steel,
            name=f"pitch_trunnion_{suffix}",
        )
    roll_carriage.visual(
        mesh_from_cadquery(_tube_x(0.055, 0.031, 0.180, -0.040), "roll_housing_tube"),
        material=blue_casting,
        name="roll_housing_tube",
    )
    roll_carriage.visual(
        Box((0.060, 0.012, 0.012)),
        origin=Origin(xyz=(0.050, 0.0, 0.0275)),
        material=black_bearing,
        name="roll_bearing_pad",
    )

    # Coaxial spindle and tool flange; this is the roll axis.
    roll_spindle = model.part("roll_spindle")
    roll_spindle.visual(
        Cylinder(radius=0.022, length=0.285),
        origin=Origin(xyz=(0.1025, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="spindle_shaft",
    )
    roll_spindle.visual(
        Cylinder(radius=0.060, length=0.025),
        origin=Origin(xyz=(0.255, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="tool_flange",
    )
    for index in range(6):
        angle = 2.0 * math.pi * index / 6.0
        roll_spindle.visual(
            Cylinder(radius=0.0055, length=0.008),
            origin=Origin(
                xyz=(0.271, 0.042 * math.cos(angle), 0.042 * math.sin(angle)),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=dark_steel,
            name=f"flange_bolt_{index}",
        )

    model.articulation(
        "yaw",
        ArticulationType.REVOLUTE,
        parent=base_ring,
        child=pitch_yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.5,
            lower=-math.radians(150.0),
            upper=math.radians(150.0),
        ),
    )
    model.articulation(
        "pitch",
        ArticulationType.REVOLUTE,
        parent=pitch_yoke,
        child=roll_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.2,
            lower=-math.radians(60.0),
            upper=math.radians(60.0),
        ),
    )
    model.articulation(
        "roll",
        ArticulationType.REVOLUTE,
        parent=roll_carriage,
        child=roll_spindle,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base_ring = object_model.get_part("base_ring")
    pitch_yoke = object_model.get_part("pitch_yoke")
    roll_carriage = object_model.get_part("roll_carriage")
    roll_spindle = object_model.get_part("roll_spindle")
    yaw = object_model.get_articulation("yaw")
    pitch = object_model.get_articulation("pitch")
    roll = object_model.get_articulation("roll")

    ctx.check(
        "yaw limit is about 150 degrees each way",
        yaw.motion_limits is not None
        and abs(yaw.motion_limits.lower + math.radians(150.0)) < 1e-6
        and abs(yaw.motion_limits.upper - math.radians(150.0)) < 1e-6,
    )
    ctx.check(
        "pitch limit is about 60 degrees each way",
        pitch.motion_limits is not None
        and abs(pitch.motion_limits.lower + math.radians(60.0)) < 1e-6
        and abs(pitch.motion_limits.upper - math.radians(60.0)) < 1e-6,
    )
    ctx.check(
        "roll limit is about 180 degrees each way",
        roll.motion_limits is not None
        and abs(roll.motion_limits.lower + math.pi) < 1e-6
        and abs(roll.motion_limits.upper - math.pi) < 1e-6,
    )

    ctx.expect_gap(
        pitch_yoke,
        base_ring,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="yaw_turntable",
        negative_elem="annular_ring",
        name="yaw turntable is seated on the base ring",
    )
    ctx.expect_within(
        roll_spindle,
        roll_carriage,
        axes="yz",
        inner_elem="spindle_shaft",
        outer_elem="roll_housing_tube",
        margin=0.003,
        name="roll spindle is coaxial inside the roll housing",
    )
    ctx.expect_overlap(
        roll_spindle,
        roll_carriage,
        axes="x",
        min_overlap=0.12,
        elem_a="spindle_shaft",
        elem_b="roll_housing_tube",
        name="roll spindle remains captured through the housing",
    )

    flange_rest_aabb = ctx.part_element_world_aabb(roll_spindle, elem="tool_flange")
    flange_rest_z = (
        (flange_rest_aabb[0][2] + flange_rest_aabb[1][2]) * 0.5
        if flange_rest_aabb is not None
        else None
    )
    with ctx.pose({pitch: math.radians(60.0)}):
        flange_pitch_aabb = ctx.part_element_world_aabb(roll_spindle, elem="tool_flange")
        flange_pitch_z = (
            (flange_pitch_aabb[0][2] + flange_pitch_aabb[1][2]) * 0.5
            if flange_pitch_aabb is not None
            else None
        )
    with ctx.pose({roll: math.pi}):
        ctx.expect_overlap(
            roll_spindle,
            roll_carriage,
            axes="x",
            min_overlap=0.12,
            elem_a="spindle_shaft",
            elem_b="roll_housing_tube",
            name="roll spin preserves captured shaft fit",
        )
    ctx.check(
        "positive pitch moves the tool axis downward",
        flange_rest_z is not None
        and flange_pitch_z is not None
        and flange_pitch_z < flange_rest_z - 0.15,
        details=f"rest_z={flange_rest_z}, pitch_z={flange_pitch_z}",
    )

    return ctx.report()


object_model = build_object_model()
