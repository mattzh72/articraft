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
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="exposed_joystick_gimbal")

    pivot_z = 0.18

    painted_black = Material("satin_black_powdercoat", rgba=(0.015, 0.017, 0.020, 1.0))
    dark_plate = Material("dark_anodized_plate", rgba=(0.05, 0.055, 0.060, 1.0))
    machined_steel = Material("brushed_steel", rgba=(0.58, 0.58, 0.56, 1.0))
    bearing_bronze = Material("oilite_bronze_bushings", rgba=(0.75, 0.52, 0.24, 1.0))
    rubber = Material("matte_black_rubber", rgba=(0.02, 0.018, 0.016, 1.0))
    fastener = Material("black_oxide_fasteners", rgba=(0.005, 0.005, 0.006, 1.0))

    base_bearing_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.026, tube=0.007, radial_segments=28, tubular_segments=18),
        "base_bearing_race",
    )
    outer_bearing_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.023, tube=0.006, radial_segments=28, tubular_segments=18),
        "outer_pitch_bearing_race",
    )

    base = model.part("base")
    base.visual(
        Box((0.42, 0.32, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0325)),
        material=dark_plate,
        name="grounded_base_block",
    )
    for i, x in enumerate((-0.155, 0.155)):
        for j, y in enumerate((-0.115, 0.115)):
            base.visual(
                Cylinder(radius=0.020, length=0.010),
                origin=Origin(xyz=(x, y, 0.005)),
                material=rubber,
                name=f"rubber_foot_{i}_{j}",
            )
            base.visual(
                Cylinder(radius=0.008, length=0.006),
                origin=Origin(xyz=(x, y, 0.058)),
                material=fastener,
                name=f"hold_down_bolt_{i}_{j}",
            )
    for side, x, pedestal_name, saddle_name, bridge_name, bushing_name, bearing_name in (
        (
            "neg",
            -0.180,
            "neg_bearing_pedestal",
            "neg_bearing_saddle",
            "neg_bearing_bridge",
            "neg_roll_bushing",
            "neg_roll_bearing_race",
        ),
        (
            "pos",
            0.180,
            "pos_bearing_pedestal",
            "pos_bearing_saddle",
            "pos_bearing_bridge",
            "pos_roll_bushing",
            "pos_roll_bearing_race",
        ),
    ):
        base.visual(
            Box((0.058, 0.070, 0.088)),
            origin=Origin(xyz=(x, 0.0, 0.099)),
            material=painted_black,
            name=pedestal_name,
        )
        base.visual(
            Box((0.070, 0.082, 0.014)),
            origin=Origin(xyz=(x, 0.0, 0.137)),
            material=painted_black,
            name=saddle_name,
        )
        base.visual(
            Box((0.050, 0.020, 0.014)),
            origin=Origin(xyz=(x, 0.0, 0.148)),
            material=painted_black,
            name=bridge_name,
        )
        base.visual(
            Cylinder(radius=0.019, length=0.036),
            origin=Origin(xyz=(x, 0.0, pivot_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bearing_bronze,
            name=bushing_name,
        )
        base.visual(
            base_bearing_mesh,
            origin=Origin(xyz=(x, 0.0, pivot_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bearing_bronze,
            name=bearing_name,
        )

    outer = model.part("outer_gimbal")
    for side, x, trunnion_name, collar_name, web_name in (
        ("neg", -0.125, "neg_roll_trunnion", "neg_outer_shaft_collar", "neg_roll_yoke_web"),
        ("pos", 0.125, "pos_roll_trunnion", "pos_outer_shaft_collar", "pos_roll_yoke_web"),
    ):
        outer.visual(
            Cylinder(radius=0.010, length=0.190),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=machined_steel,
            name=trunnion_name,
        )
        outer.visual(
            Cylinder(radius=0.016, length=0.014),
            origin=Origin(xyz=(math.copysign(0.215, x), 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=machined_steel,
            name=collar_name,
        )
        outer.visual(
            Box((0.010, 0.020, 0.092)),
            origin=Origin(xyz=(math.copysign(0.029, x), 0.0, -0.042)),
            material=painted_black,
            name=web_name,
        )
    outer.visual(
        Box((0.074, 0.270, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, -0.097)),
        material=painted_black,
        name="lower_cross_member",
    )
    for side, y, upright_name, bushing_name, bearing_name in (
        ("front", -0.125, "front_pitch_upright", "front_pitch_bushing", "front_pitch_bearing_race"),
        ("rear", 0.125, "rear_pitch_upright", "rear_pitch_bushing", "rear_pitch_bearing_race"),
    ):
        for cheek_x, cheek_suffix in ((-0.028, "neg"), (0.028, "pos")):
            outer.visual(
                Box((0.016, 0.024, 0.100)),
                origin=Origin(xyz=(cheek_x, y, -0.047)),
                material=painted_black,
                name=f"{upright_name}_{cheek_suffix}_cheek",
            )
        outer.visual(
            Cylinder(radius=0.017, length=0.040),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=bearing_bronze,
            name=bushing_name,
        )
        outer.visual(
            outer_bearing_mesh,
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=bearing_bronze,
            name=bearing_name,
        )

    stick = model.part("stick")
    stick.visual(
        Cylinder(radius=0.007, length=0.310),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machined_steel,
        name="pitch_pin",
    )
    for side, y in (("front", -0.151), ("rear", 0.151)):
        stick.visual(
            Cylinder(radius=0.013, length=0.012),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=machined_steel,
            name=f"{side}_pitch_collar",
        )
    stick.visual(
        Sphere(radius=0.021),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=machined_steel,
        name="polished_pivot_ball",
    )
    stick.visual(
        Cylinder(radius=0.010, length=0.285),
        origin=Origin(xyz=(0.0, 0.0, 0.148)),
        material=machined_steel,
        name="straight_stick_shaft",
    )
    stick.visual(
        Cylinder(radius=0.022, length=0.095),
        origin=Origin(xyz=(0.0, 0.0, 0.323)),
        material=rubber,
        name="cylindrical_grip",
    )
    stick.visual(
        Sphere(radius=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.371)),
        material=rubber,
        name="rounded_grip_cap",
    )

    roll = model.articulation(
        "base_to_outer_gimbal",
        ArticulationType.REVOLUTE,
        parent=base,
        child=outer,
        origin=Origin(xyz=(0.0, 0.0, pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=3.0, lower=-0.42, upper=0.42),
    )
    pitch = model.articulation(
        "outer_gimbal_to_stick",
        ArticulationType.REVOLUTE,
        parent=outer,
        child=stick,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=3.5, lower=-0.42, upper=0.42),
    )
    roll.meta["mechanism_note"] = "Roll axis is the exposed base-supported trunnion line."
    pitch.meta["mechanism_note"] = "Pitch axis intersects the roll axis at the polished pivot ball."

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    outer = object_model.get_part("outer_gimbal")
    stick = object_model.get_part("stick")
    roll = object_model.get_articulation("base_to_outer_gimbal")
    pitch = object_model.get_articulation("outer_gimbal_to_stick")

    for bushing, trunnion in (
        ("neg_roll_bushing", "neg_roll_trunnion"),
        ("pos_roll_bushing", "pos_roll_trunnion"),
    ):
        ctx.allow_overlap(
            base,
            outer,
            elem_a=bushing,
            elem_b=trunnion,
            reason="The roll trunnion is intentionally captured inside the bronze base bushing.",
        )
        ctx.expect_within(
            outer,
            base,
            axes="yz",
            inner_elem=trunnion,
            outer_elem=bushing,
            margin=0.0,
            name=f"{trunnion} centered in roll bushing",
        )
        ctx.expect_overlap(
            outer,
            base,
            axes="x",
            elem_a=trunnion,
            elem_b=bushing,
            min_overlap=0.020,
            name=f"{trunnion} retained through roll bushing",
        )

    for bushing in ("front_pitch_bushing", "rear_pitch_bushing"):
        ctx.allow_overlap(
            outer,
            stick,
            elem_a=bushing,
            elem_b="pitch_pin",
            reason="The pitch pin is intentionally captured inside the outer gimbal bushing.",
        )
        ctx.expect_within(
            stick,
            outer,
            axes="xz",
            inner_elem="pitch_pin",
            outer_elem=bushing,
            margin=0.0,
            name=f"pitch pin centered in {bushing}",
        )
        ctx.expect_overlap(
            stick,
            outer,
            axes="y",
            elem_a="pitch_pin",
            elem_b=bushing,
            min_overlap=0.020,
            name=f"pitch pin retained through {bushing}",
        )

    ctx.check(
        "two exposed revolute joints",
        len(object_model.articulations) == 2
        and roll.articulation_type == ArticulationType.REVOLUTE
        and pitch.articulation_type == ArticulationType.REVOLUTE,
        details=f"articulations={[j.name for j in object_model.articulations]}",
    )
    dot = sum(a * b for a, b in zip(roll.axis, pitch.axis))
    ctx.check(
        "gimbal axes are orthogonal",
        abs(dot) < 1.0e-6,
        details=f"roll_axis={roll.axis}, pitch_axis={pitch.axis}, dot={dot}",
    )
    ctx.check(
        "gimbal axes intersect at pivot",
        roll.origin.xyz == (0.0, 0.0, 0.18) and pitch.origin.xyz == (0.0, 0.0, 0.0),
        details=f"roll_origin={roll.origin.xyz}, pitch_origin={pitch.origin.xyz}",
    )
    ctx.expect_within(
        stick,
        outer,
        axes="x",
        inner_elem="pitch_pin",
        outer_elem="front_pitch_bearing_race",
        margin=0.020,
        name="pitch pin centered in bearing races",
    )
    ctx.expect_overlap(
        outer,
        base,
        axes="x",
        elem_a="pos_roll_trunnion",
        elem_b="pos_roll_bearing_race",
        min_overlap=0.008,
        name="roll trunnions pass through base bearings",
    )
    ctx.expect_gap(
        outer,
        base,
        axis="z",
        positive_elem="lower_cross_member",
        negative_elem="grounded_base_block",
        min_gap=0.010,
        name="open yoke clears grounded base",
    )

    neutral_grip = ctx.part_element_world_aabb(stick, elem="cylindrical_grip")
    neutral_center = tuple((neutral_grip[0][i] + neutral_grip[1][i]) * 0.5 for i in range(3))
    with ctx.pose({roll: 0.32}):
        rolled_grip = ctx.part_element_world_aabb(stick, elem="cylindrical_grip")
        rolled_center = tuple((rolled_grip[0][i] + rolled_grip[1][i]) * 0.5 for i in range(3))
    with ctx.pose({pitch: 0.32}):
        pitched_grip = ctx.part_element_world_aabb(stick, elem="cylindrical_grip")
        pitched_center = tuple((pitched_grip[0][i] + pitched_grip[1][i]) * 0.5 for i in range(3))
    ctx.check(
        "roll deflects grip sideways",
        abs(rolled_center[1] - neutral_center[1]) > 0.030,
        details=f"neutral={neutral_center}, rolled={rolled_center}",
    )
    ctx.check(
        "pitch deflects grip fore-aft",
        abs(pitched_center[0] - neutral_center[0]) > 0.030,
        details=f"neutral={neutral_center}, pitched={pitched_center}",
    )

    return ctx.report()


object_model = build_object_model()
