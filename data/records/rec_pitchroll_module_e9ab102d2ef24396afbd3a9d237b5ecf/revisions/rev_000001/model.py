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
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_spindle_sensor_bracket")

    base_mat = Material("dark_powder_coated_steel", color=(0.06, 0.07, 0.08, 1.0))
    edge_mat = Material("brushed_edge_wear", color=(0.30, 0.32, 0.34, 1.0))
    yoke_mat = Material("orange_anodized_aluminum", color=(0.95, 0.38, 0.08, 1.0))
    bearing_mat = Material("black_bearing_rubber", color=(0.01, 0.012, 0.014, 1.0))
    spindle_mat = Material("satin_black_cartridge", color=(0.015, 0.017, 0.020, 1.0))
    glass_mat = Material("blue_sensor_glass", color=(0.08, 0.20, 0.42, 0.78))
    bolt_mat = Material("zinc_bolt_heads", color=(0.62, 0.64, 0.62, 1.0))

    base = model.part("base_frame")
    base.visual(
        Box((0.38, 0.34, 0.026)),
        origin=Origin(xyz=(0.06, 0.0, 0.013)),
        material=base_mat,
        name="base_plate",
    )
    base.visual(
        Box((0.075, 0.038, 0.174)),
        origin=Origin(xyz=(0.0, 0.151, 0.087)),
        material=base_mat,
        name="trunnion_tower_0",
    )
    base.visual(
        Box((0.075, 0.038, 0.174)),
        origin=Origin(xyz=(0.0, -0.151, 0.087)),
        material=base_mat,
        name="trunnion_tower_1",
    )
    base.visual(
        Box((0.039, 0.264, 0.028)),
        origin=Origin(xyz=(-0.057, 0.0, 0.160)),
        material=base_mat,
        name="rear_bridge",
    )
    base.visual(
        Box((0.060, 0.038, 0.095)),
        origin=Origin(xyz=(-0.050, 0.151, 0.071)),
        material=edge_mat,
        name="tower_rib_0",
    )
    base.visual(
        Box((0.060, 0.038, 0.095)),
        origin=Origin(xyz=(-0.050, -0.151, 0.071)),
        material=edge_mat,
        name="tower_rib_1",
    )

    base.visual(
        Cylinder(radius=0.040, length=0.026),
        origin=Origin(xyz=(0.0, 0.121, 0.145), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=edge_mat,
        name="pitch_bearing_0",
    )
    base.visual(
        mesh_from_geometry(TorusGeometry(radius=0.027, tube=0.005, radial_segments=18, tubular_segments=36), "bearing_bore_shadow_0"),
        origin=Origin(xyz=(0.0, 0.110, 0.145), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bearing_mat,
        name="bearing_bore_shadow_0",
    )
    base.visual(
        Cylinder(radius=0.040, length=0.026),
        origin=Origin(xyz=(0.0, -0.121, 0.145), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=edge_mat,
        name="pitch_bearing_1",
    )
    base.visual(
        mesh_from_geometry(TorusGeometry(radius=0.027, tube=0.005, radial_segments=18, tubular_segments=36), "bearing_bore_shadow_1"),
        origin=Origin(xyz=(0.0, -0.110, 0.145), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bearing_mat,
        name="bearing_bore_shadow_1",
    )

    for idx, (x, y) in enumerate(((-0.090, 0.115), (0.195, 0.115), (-0.090, -0.115), (0.195, -0.115))):
        base.visual(
            Cylinder(radius=0.014, length=0.006),
            origin=Origin(xyz=(x, y, 0.029)),
            material=bolt_mat,
            name=f"mount_bolt_{idx}",
        )

    yoke = model.part("pitch_yoke")
    yoke.visual(
        Cylinder(radius=0.018, length=0.250),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=edge_mat,
        name="trunnion_shaft",
    )
    yoke.visual(
        Cylinder(radius=0.030, length=0.115),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=yoke_mat,
        name="pitch_hub",
    )
    yoke.visual(
        Box((0.080, 0.136, 0.034)),
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
        material=yoke_mat,
        name="rear_yoke_bridge",
    )
    yoke.visual(
        Box((0.255, 0.026, 0.028)),
        origin=Origin(xyz=(0.135, 0.055, 0.0)),
        material=yoke_mat,
        name="side_arm_0",
    )
    yoke.visual(
        Box((0.255, 0.026, 0.028)),
        origin=Origin(xyz=(0.135, -0.055, 0.0)),
        material=yoke_mat,
        name="side_arm_1",
    )
    yoke.visual(
        Box((0.060, 0.026, 0.030)),
        origin=Origin(xyz=(0.240, 0.055, 0.0)),
        material=yoke_mat,
        name="nose_lug_0",
    )
    yoke.visual(
        Box((0.060, 0.026, 0.030)),
        origin=Origin(xyz=(0.240, -0.055, 0.0)),
        material=yoke_mat,
        name="nose_lug_1",
    )
    yoke.visual(
        Box((0.048, 0.018, 0.024)),
        origin=Origin(xyz=(0.202, 0.044, -0.026)),
        material=yoke_mat,
        name="saddle_0",
    )
    yoke.visual(
        Box((0.048, 0.018, 0.024)),
        origin=Origin(xyz=(0.202, -0.044, -0.026)),
        material=yoke_mat,
        name="saddle_1",
    )
    yoke.visual(
        mesh_from_geometry(TorusGeometry(radius=0.033, tube=0.010, radial_segments=18, tubular_segments=36), "rear_roll_bearing"),
        origin=Origin(xyz=(0.205, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing_mat,
        name="rear_roll_bearing",
    )
    yoke.visual(
        mesh_from_geometry(TorusGeometry(radius=0.033, tube=0.010, radial_segments=18, tubular_segments=36), "front_roll_bearing"),
        origin=Origin(xyz=(0.252, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing_mat,
        name="front_roll_bearing",
    )

    cartridge = model.part("roll_cartridge")
    cartridge.visual(
        Cylinder(radius=0.024, length=0.122),
        origin=Origin(xyz=(0.050, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=spindle_mat,
        name="spindle_shell",
    )
    cartridge.visual(
        Cylinder(radius=0.028, length=0.014),
        origin=Origin(xyz=(-0.018, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=spindle_mat,
        name="rear_retainer",
    )
    cartridge.visual(
        Cylinder(radius=0.030, length=0.016),
        origin=Origin(xyz=(0.066, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=spindle_mat,
        name="front_retainer",
    )
    cartridge.visual(
        Cylinder(radius=0.033, length=0.082),
        origin=Origin(xyz=(0.116, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=spindle_mat,
        name="sensor_can",
    )
    cartridge.visual(
        Cylinder(radius=0.027, length=0.006),
        origin=Origin(xyz=(0.160, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass_mat,
        name="sensor_window",
    )

    model.articulation(
        "pitch_trunnion",
        ArticulationType.REVOLUTE,
        parent=base,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5, lower=-0.25, upper=0.75),
    )
    model.articulation(
        "roll_spindle",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=cartridge,
        origin=Origin(xyz=(0.205, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=4.0, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_frame")
    yoke = object_model.get_part("pitch_yoke")
    cartridge = object_model.get_part("roll_cartridge")
    pitch = object_model.get_articulation("pitch_trunnion")
    roll = object_model.get_articulation("roll_spindle")

    ctx.allow_overlap(
        base,
        yoke,
        elem_a="pitch_bearing_0",
        elem_b="trunnion_shaft",
        reason="The yoke trunnion shaft is intentionally captured inside the simplified side bearing cup.",
    )
    ctx.allow_overlap(
        base,
        yoke,
        elem_a="pitch_bearing_1",
        elem_b="trunnion_shaft",
        reason="The opposite trunnion shaft end is intentionally seated in the matching side bearing cup.",
    )
    ctx.allow_overlap(
        yoke,
        cartridge,
        elem_a="rear_roll_bearing",
        elem_b="spindle_shell",
        reason="The roll cartridge journal is intentionally captured in the rear bearing race with a slight seated interference.",
    )
    ctx.allow_overlap(
        yoke,
        cartridge,
        elem_a="front_roll_bearing",
        elem_b="spindle_shell",
        reason="The roll cartridge journal is intentionally captured in the front bearing race with a slight seated interference.",
    )

    ctx.check(
        "pitch joint uses side trunnion axis",
        tuple(round(v, 6) for v in pitch.axis) == (0.0, -1.0, 0.0),
        details=f"pitch axis={pitch.axis}",
    )
    ctx.check(
        "roll joint uses forward spindle axis",
        tuple(round(v, 6) for v in roll.axis) == (1.0, 0.0, 0.0),
        details=f"roll axis={roll.axis}",
    )

    ctx.expect_overlap(
        base,
        yoke,
        axes="y",
        elem_a="pitch_bearing_0",
        elem_b="trunnion_shaft",
        min_overlap=0.006,
        name="first trunnion retained in side bearing",
    )
    ctx.expect_overlap(
        base,
        yoke,
        axes="y",
        elem_a="pitch_bearing_1",
        elem_b="trunnion_shaft",
        min_overlap=0.006,
        name="second trunnion retained in side bearing",
    )
    ctx.expect_within(
        cartridge,
        yoke,
        axes="yz",
        inner_elem="spindle_shell",
        outer_elem="front_roll_bearing",
        margin=0.004,
        name="spindle is coaxial within front bearing envelope",
    )
    ctx.expect_overlap(
        yoke,
        cartridge,
        axes="x",
        elem_a="front_roll_bearing",
        elem_b="spindle_shell",
        min_overlap=0.014,
        name="front bearing retains spindle journal",
    )
    ctx.expect_overlap(
        yoke,
        cartridge,
        axes="x",
        elem_a="rear_roll_bearing",
        elem_b="spindle_shell",
        min_overlap=0.014,
        name="rear bearing retains spindle journal",
    )

    with ctx.pose({roll: 1.20}):
        ctx.expect_within(
            cartridge,
            yoke,
            axes="yz",
            inner_elem="spindle_shell",
            outer_elem="rear_roll_bearing",
            margin=0.004,
            name="rolling spindle remains coaxial in rear bearing",
        )

    rest_can = ctx.part_element_world_aabb(cartridge, elem="sensor_can")
    with ctx.pose({pitch: 0.45}):
        pitched_can = ctx.part_element_world_aabb(cartridge, elem="sensor_can")

    with ctx.pose({pitch: -0.25}):
        ctx.expect_gap(
            yoke,
            base,
            axis="z",
            positive_elem="front_roll_bearing",
            negative_elem="base_plate",
            min_gap=0.010,
            name="downward pitch stop clears base plate",
        )

    ctx.check(
        "positive pitch lifts cartridge nose",
        rest_can is not None and pitched_can is not None and pitched_can[0][2] > rest_can[0][2] + 0.010,
        details=f"rest sensor AABB={rest_can}, pitched sensor AABB={pitched_can}",
    )

    return ctx.report()


object_model = build_object_model()
