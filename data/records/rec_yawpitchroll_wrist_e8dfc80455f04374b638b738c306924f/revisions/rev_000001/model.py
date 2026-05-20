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
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_roll_cartridge_wrist")

    tower_dark = model.material("tower_dark", rgba=(0.08, 0.09, 0.10, 1.0))
    tower_panel = model.material("tower_panel", rgba=(0.13, 0.15, 0.17, 1.0))
    yaw_orange = model.material("yaw_orange", rgba=(0.92, 0.48, 0.13, 1.0))
    pitch_blue = model.material("pitch_blue", rgba=(0.12, 0.28, 0.54, 1.0))
    cartridge_metal = model.material("cartridge_metal", rgba=(0.62, 0.66, 0.68, 1.0))
    bearing_steel = model.material("bearing_steel", rgba=(0.78, 0.78, 0.74, 1.0))
    black = model.material("black_rubber", rgba=(0.015, 0.015, 0.018, 1.0))

    pitch_bearing_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.026, tube=0.008, radial_segments=24, tubular_segments=40),
        "pitch_bearing_ring",
    )
    roll_bearing_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.041, tube=0.010, radial_segments=24, tubular_segments=40),
        "roll_bearing_ring",
    )

    tower = model.part("tower")
    tower.visual(
        Box((0.48, 0.38, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=tower_dark,
        name="floor_plate",
    )
    tower.visual(
        Box((0.14, 0.14, 0.58)),
        origin=Origin(xyz=(0.0, 0.0, 0.335)),
        material=tower_dark,
        name="upright_column",
    )
    tower.visual(
        Box((0.006, 0.096, 0.22)),
        origin=Origin(xyz=(0.073, 0.0, 0.34)),
        material=tower_panel,
        name="service_panel",
    )
    tower.visual(
        Cylinder(radius=0.105, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.6425)),
        material=tower_dark,
        name="top_cap",
    )
    tower.visual(
        Cylinder(radius=0.086, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.680)),
        material=bearing_steel,
        name="stationary_bearing",
    )
    for i, (x, y) in enumerate(((-0.18, -0.13), (-0.18, 0.13), (0.18, -0.13), (0.18, 0.13))):
        tower.visual(
            Cylinder(radius=0.013, length=0.012),
            origin=Origin(xyz=(x, y, 0.051)),
            material=bearing_steel,
            name=f"base_bolt_{i}",
        )

    yaw_stage = model.part("yaw_stage")
    yaw_stage.visual(
        Cylinder(radius=0.102, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=yaw_orange,
        name="turntable",
    )
    yaw_stage.visual(
        Box((0.22, 0.16, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0475)),
        material=yaw_orange,
        name="yoke_base",
    )
    yaw_stage.visual(
        Box((0.050, 0.044, 0.020)),
        origin=Origin(xyz=(0.125, 0.0, 0.070)),
        material=black,
        name="yaw_index",
    )
    yaw_stage.visual(
        Box((0.076, 0.032, 0.155)),
        origin=Origin(xyz=(0.0, -0.095, 0.1375)),
        material=yaw_orange,
        name="pitch_cheek_0",
    )
    yaw_stage.visual(
        pitch_bearing_mesh,
        origin=Origin(xyz=(0.0, -0.07885, 0.160), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bearing_steel,
        name="pitch_bearing_0",
    )
    yaw_stage.visual(
        Box((0.076, 0.032, 0.155)),
        origin=Origin(xyz=(0.0, 0.095, 0.1375)),
        material=yaw_orange,
        name="pitch_cheek_1",
    )
    yaw_stage.visual(
        pitch_bearing_mesh,
        origin=Origin(xyz=(0.0, 0.07885, 0.160), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bearing_steel,
        name="pitch_bearing_1",
    )

    pitch_cradle = model.part("pitch_cradle")
    pitch_cradle.visual(
        Cylinder(radius=0.020, length=0.034),
        origin=Origin(xyz=(0.0, -0.058, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bearing_steel,
        name="pitch_trunnion_0",
    )
    pitch_cradle.visual(
        Cylinder(radius=0.020, length=0.034),
        origin=Origin(xyz=(0.0, 0.058, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bearing_steel,
        name="pitch_trunnion_1",
    )
    pitch_cradle.visual(
        roll_bearing_mesh,
        origin=Origin(xyz=(-0.060, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing_steel,
        name="roll_bearing_0",
    )
    pitch_cradle.visual(
        roll_bearing_mesh,
        origin=Origin(xyz=(0.060, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing_steel,
        name="roll_bearing_1",
    )
    for suffix, z in (("top", 0.064), ("bottom", -0.064)):
        pitch_cradle.visual(
            Box((0.190, 0.126, 0.024)),
            origin=Origin(xyz=(0.0, 0.0, 0.056 if suffix == "top" else -0.056)),
            material=pitch_blue,
            name=f"{suffix}_bridge",
        )
    for suffix, y in (("0", -0.056), ("1", 0.056)):
        pitch_cradle.visual(
            Box((0.190, 0.016, 0.128)),
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=pitch_blue,
            name=f"side_rail_{suffix}",
        )

    roll_cartridge = model.part("roll_cartridge")
    roll_cartridge.visual(
        Cylinder(radius=0.032, length=0.154),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cartridge_metal,
        name="cartridge_body",
    )
    roll_cartridge.visual(
        Box((0.080, 0.012, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=black,
        name="index_key",
    )
    roll_cartridge.visual(
        Cylinder(radius=0.045, length=0.016),
        origin=Origin(xyz=(0.085, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cartridge_metal,
        name="output_flange",
    )
    roll_cartridge.visual(
        Cylinder(radius=0.018, length=0.050),
        origin=Origin(xyz=(0.118, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing_steel,
        name="tool_spigot",
    )
    roll_cartridge.visual(
        Cylinder(radius=0.036, length=0.018),
        origin=Origin(xyz=(-0.085, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cartridge_metal,
        name="rear_cap",
    )
    roll_cartridge.visual(
        Cylinder(radius=0.018, length=0.040),
        origin=Origin(xyz=(-0.114, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="cable_boot",
    )

    model.articulation(
        "tower_to_yaw",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=yaw_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.700)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=-2.8, upper=2.8),
    )
    model.articulation(
        "yaw_to_pitch",
        ArticulationType.REVOLUTE,
        parent=yaw_stage,
        child=pitch_cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.4, lower=-0.70, upper=0.70),
    )
    model.articulation(
        "pitch_to_roll",
        ArticulationType.REVOLUTE,
        parent=pitch_cradle,
        child=roll_cartridge,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.5, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    yaw_stage = object_model.get_part("yaw_stage")
    pitch_cradle = object_model.get_part("pitch_cradle")
    roll_cartridge = object_model.get_part("roll_cartridge")
    yaw_joint = object_model.get_articulation("tower_to_yaw")
    pitch_joint = object_model.get_articulation("yaw_to_pitch")
    roll_joint = object_model.get_articulation("pitch_to_roll")

    ctx.check(
        "serial revolute wrist joints",
        all(
            joint.articulation_type == ArticulationType.REVOLUTE
            for joint in (yaw_joint, pitch_joint, roll_joint)
        ),
        details="Yaw, pitch, and roll must all be revolute joints.",
    )
    ctx.check(
        "orthogonal yaw pitch roll axes",
        yaw_joint.axis == (0.0, 0.0, 1.0)
        and pitch_joint.axis == (0.0, 1.0, 0.0)
        and roll_joint.axis == (1.0, 0.0, 0.0),
        details=f"axes={yaw_joint.axis}, {pitch_joint.axis}, {roll_joint.axis}",
    )
    ctx.allow_overlap(
        yaw_stage,
        pitch_cradle,
        elem_a="pitch_bearing_0",
        elem_b="pitch_trunnion_0",
        reason="The pitch trunnion is intentionally captured inside the bearing ring proxy.",
    )
    ctx.allow_overlap(
        yaw_stage,
        pitch_cradle,
        elem_a="pitch_bearing_1",
        elem_b="pitch_trunnion_1",
        reason="The pitch trunnion is intentionally captured inside the opposite bearing ring proxy.",
    )
    ctx.allow_overlap(
        pitch_cradle,
        roll_cartridge,
        elem_a="roll_bearing_0",
        elem_b="cartridge_body",
        reason="The roll cartridge body is intentionally seated in the cradle bearing ring proxy.",
    )
    ctx.allow_overlap(
        pitch_cradle,
        roll_cartridge,
        elem_a="roll_bearing_1",
        elem_b="cartridge_body",
        reason="The roll cartridge body is intentionally seated in the opposite cradle bearing ring proxy.",
    )

    ctx.expect_gap(
        yaw_stage,
        tower,
        axis="z",
        positive_elem="turntable",
        negative_elem="stationary_bearing",
        max_gap=0.001,
        max_penetration=0.00001,
        name="yaw stage sits on tower bearing",
    )
    ctx.expect_overlap(
        yaw_stage,
        tower,
        axes="xy",
        elem_a="turntable",
        elem_b="stationary_bearing",
        min_overlap=0.14,
        name="yaw bearing is coaxially supported",
    )
    ctx.expect_overlap(
        pitch_cradle,
        yaw_stage,
        axes="xz",
        elem_a="pitch_trunnion_0",
        elem_b="pitch_bearing_0",
        min_overlap=0.030,
        name="pitch trunnion is carried by one yaw bearing",
    )
    ctx.expect_overlap(
        pitch_cradle,
        yaw_stage,
        axes="xz",
        elem_a="pitch_trunnion_1",
        elem_b="pitch_bearing_1",
        min_overlap=0.030,
        name="pitch trunnion is carried by opposite yaw bearing",
    )
    ctx.expect_within(
        roll_cartridge,
        pitch_cradle,
        axes="yz",
        inner_elem="cartridge_body",
        outer_elem="roll_bearing_0",
        margin=0.0,
        name="roll cartridge is nested inside one cradle bearing",
    )
    ctx.expect_within(
        roll_cartridge,
        pitch_cradle,
        axes="yz",
        inner_elem="cartridge_body",
        outer_elem="roll_bearing_1",
        margin=0.0,
        name="roll cartridge is nested inside opposite cradle bearing",
    )
    ctx.expect_overlap(
        roll_cartridge,
        pitch_cradle,
        axes="x",
        elem_a="cartridge_body",
        elem_b="roll_bearing_0",
        min_overlap=0.015,
        name="one roll bearing captures cartridge length",
    )
    ctx.expect_overlap(
        roll_cartridge,
        pitch_cradle,
        axes="x",
        elem_a="cartridge_body",
        elem_b="roll_bearing_1",
        min_overlap=0.015,
        name="opposite roll bearing captures cartridge length",
    )

    def _center_of_elem(part, elem_name):
        aabb = ctx.part_element_world_aabb(part, elem=elem_name)
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    rest_key = _center_of_elem(roll_cartridge, "index_key")
    with ctx.pose({roll_joint: 1.2}):
        rolled_key = _center_of_elem(roll_cartridge, "index_key")
    ctx.check(
        "roll visibly turns cartridge key",
        rest_key is not None
        and rolled_key is not None
        and (abs(rolled_key[1] - rest_key[1]) > 0.015 or abs(rolled_key[2] - rest_key[2]) > 0.015),
        details=f"rest={rest_key}, rolled={rolled_key}",
    )

    rest_pitch = _center_of_elem(pitch_cradle, "roll_bearing_1")
    with ctx.pose({pitch_joint: 0.55}):
        pitched = _center_of_elem(pitch_cradle, "roll_bearing_1")
    ctx.check(
        "pitch rotates cradle about trunnions",
        rest_pitch is not None and pitched is not None and abs(pitched[2] - rest_pitch[2]) > 0.025,
        details=f"rest={rest_pitch}, pitched={pitched}",
    )

    return ctx.report()


object_model = build_object_model()
