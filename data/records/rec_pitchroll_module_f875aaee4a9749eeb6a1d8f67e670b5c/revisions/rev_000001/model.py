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
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_axis_sensor_mount")

    dark_anodized = model.material("dark_anodized", rgba=(0.055, 0.060, 0.065, 1.0))
    black = model.material("matte_black", rgba=(0.010, 0.011, 0.012, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.62, 0.64, 0.64, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.23, 0.25, 0.27, 1.0))
    screw_steel = model.material("socket_head_steel", rgba=(0.46, 0.47, 0.47, 1.0))
    glass = model.material("smoked_lens_glass", rgba=(0.04, 0.08, 0.10, 0.72))

    pitch_ring_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.033, tube=0.0048, radial_segments=18, tubular_segments=40),
        "pitch_bearing_ring",
    )
    roll_ring_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.030, tube=0.0075, radial_segments=20, tubular_segments=48),
        "roll_bearing_ring",
    )

    base = model.part("base")
    base.visual(
        Box((0.340, 0.320, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=dark_anodized,
        name="base_plate",
    )
    base.visual(
        Box((0.260, 0.190, 0.020)),
        origin=Origin(xyz=(0.0, -0.006, 0.034)),
        material=gunmetal,
        name="raised_plinth",
    )
    base.visual(
        Box((0.240, 0.160, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.039)),
        material=dark_anodized,
        name="support_yoke_base",
    )
    base.visual(
        Box((0.035, 0.150, 0.170)),
        origin=Origin(xyz=(-0.0875, 0.0, 0.1385)),
        material=dark_anodized,
        name="support_cheek_0",
    )
    base.visual(
        Box((0.035, 0.150, 0.170)),
        origin=Origin(xyz=(0.0875, 0.0, 0.1385)),
        material=dark_anodized,
        name="support_cheek_1",
    )

    for i, (x, y) in enumerate(
        ((-0.125, -0.110), (0.125, -0.110), (-0.125, 0.110), (0.125, 0.110))
    ):
        base.visual(
            Cylinder(radius=0.012, length=0.006),
            origin=Origin(xyz=(x, y, 0.027)),
            material=screw_steel,
            name=f"base_screw_{i}",
        )
        base.visual(
            Box((0.018, 0.0026, 0.0018)),
            origin=Origin(xyz=(x, y, 0.0298)),
            material=black,
            name=f"base_screw_slot_{i}",
        )

    for i, x in enumerate((-0.109, 0.109)):
        base.visual(
            pitch_ring_mesh,
            origin=Origin(xyz=(0.1098 if x > 0.0 else -0.1098, 0.0, 0.169), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=aluminum,
            name=f"pitch_bearing_ring_{i}",
        )
        for j, (dy, dz) in enumerate(((-0.026, -0.020), (0.026, -0.020), (-0.026, 0.020), (0.026, 0.020))):
            base.visual(
                Cylinder(radius=0.0048, length=0.010),
                origin=Origin(
                    xyz=((0.108 if x > 0.0 else -0.108), dy, 0.169 + dz),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=screw_steel,
                name=f"pitch_cap_screw_{i}_{j}",
            )

    pitch_cradle = model.part("pitch_cradle")
    pitch_cradle.visual(
        Cylinder(radius=0.016, length=0.190),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="pitch_shaft",
    )
    for i, x in enumerate((-0.063, 0.063)):
        pitch_cradle.visual(
            Box((0.014, 0.172, 0.106)),
            origin=Origin(xyz=(x, 0.058, -0.045)),
            material=gunmetal,
            name=f"cradle_cheek_{i}",
        )
        pitch_cradle.visual(
            Cylinder(radius=0.025, length=0.016),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=aluminum,
            name=f"cradle_hub_{i}",
        )
        for j, (y, z) in enumerate(((0.014, -0.018), (0.095, -0.018), (0.014, -0.072), (0.095, -0.072))):
            pitch_cradle.visual(
                Cylinder(radius=0.0045, length=0.006),
                origin=Origin(
                    xyz=(x - (0.009 if x > 0.0 else -0.009), y, z),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=screw_steel,
                name=f"cradle_screw_{i}_{j}",
            )

    pitch_cradle.visual(
        Box((0.132, 0.018, 0.026)),
        origin=Origin(xyz=(0.0, -0.016, -0.030)),
        material=gunmetal,
        name="rear_tie_bar",
    )
    pitch_cradle.visual(
        Box((0.132, 0.020, 0.026)),
        origin=Origin(xyz=(0.0, 0.134, -0.090)),
        material=gunmetal,
        name="front_crossbar",
    )
    pitch_cradle.visual(
        roll_ring_mesh,
        origin=Origin(xyz=(0.0, 0.134, -0.035), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="roll_bearing_ring",
    )
    pitch_cradle.visual(
        Box((0.085, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, 0.126, -0.073)),
        material=gunmetal,
        name="lower_bearing_saddle",
    )
    for i, (x, z) in enumerate(((-0.026, -0.006), (0.026, -0.006), (-0.026, -0.067), (0.026, -0.067))):
        pitch_cradle.visual(
            Cylinder(radius=0.0042, length=0.008),
            origin=Origin(xyz=(x, 0.125, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=screw_steel,
            name=f"roll_cap_screw_{i}",
        )

    roll_spindle = model.part("roll_spindle")
    roll_spindle.visual(
        Cylinder(radius=0.030, length=0.210),
        origin=Origin(xyz=(0.0, 0.034, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="spindle_core",
    )
    roll_spindle.visual(
        Cylinder(radius=0.037, length=0.016),
        origin=Origin(xyz=(0.0, 0.077, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="sensor_flange",
    )
    for i, (x, z) in enumerate(((-0.023, -0.023), (0.023, -0.023), (-0.023, 0.023), (0.023, 0.023))):
        roll_spindle.visual(
            Cylinder(radius=0.0038, length=0.005),
            origin=Origin(xyz=(x, 0.086, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=screw_steel,
            name=f"flange_screw_{i}",
        )
    roll_spindle.visual(
        Box((0.110, 0.078, 0.078)),
        origin=Origin(xyz=(0.0, 0.136, 0.0)),
        material=black,
        name="sensor_body",
    )
    roll_spindle.visual(
        Box((0.086, 0.010, 0.054)),
        origin=Origin(xyz=(0.0, 0.1795, 0.0)),
        material=gunmetal,
        name="front_faceplate",
    )
    roll_spindle.visual(
        Cylinder(radius=0.027, length=0.018),
        origin=Origin(xyz=(0.0, 0.193, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=glass,
        name="sensor_lens",
    )
    for i, (x, z) in enumerate(((-0.041, -0.026), (0.041, -0.026), (-0.041, 0.026), (0.041, 0.026))):
        roll_spindle.visual(
            Cylinder(radius=0.0037, length=0.006),
            origin=Origin(xyz=(x, 0.183, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=screw_steel,
            name=f"faceplate_screw_{i}",
        )
    roll_spindle.visual(
        Cylinder(radius=0.010, length=0.030),
        origin=Origin(xyz=(0.0, -0.083, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="rear_cable_gland",
    )
    roll_spindle.visual(
        Cylinder(radius=0.005, length=0.035),
        origin=Origin(xyz=(0.0, -0.105, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="short_cable_tail",
    )

    model.articulation(
        "base_to_cradle",
        ArticulationType.REVOLUTE,
        parent=base,
        child=pitch_cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.169)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=-0.65, upper=0.65),
    )
    model.articulation(
        "cradle_to_spindle",
        ArticulationType.REVOLUTE,
        parent=pitch_cradle,
        child=roll_spindle,
        origin=Origin(xyz=(0.0, 0.134, -0.035)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.5, velocity=1.8, lower=-1.57, upper=1.57),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    pitch_cradle = object_model.get_part("pitch_cradle")
    roll_spindle = object_model.get_part("roll_spindle")
    pitch = object_model.get_articulation("base_to_cradle")
    roll = object_model.get_articulation("cradle_to_spindle")

    ctx.allow_overlap(
        base,
        pitch_cradle,
        elem_a="support_cheek_0",
        elem_b="pitch_shaft",
        reason="The pitch shaft is intentionally captured through the simplified solid yoke cheek bore.",
    )
    ctx.allow_overlap(
        base,
        pitch_cradle,
        elem_a="support_cheek_1",
        elem_b="pitch_shaft",
        reason="The pitch shaft is intentionally captured through the simplified solid yoke cheek bore.",
    )
    ctx.allow_overlap(
        pitch_cradle,
        roll_spindle,
        elem_a="roll_bearing_ring",
        elem_b="spindle_core",
        reason="The roll spindle is intentionally nested in the simplified bearing ring proxy.",
    )

    ctx.expect_within(
        pitch_cradle,
        base,
        axes="yz",
        inner_elem="pitch_shaft",
        outer_elem="support_cheek_0",
        margin=0.002,
        name="pitch shaft is centered in one yoke cheek",
    )
    ctx.expect_overlap(
        pitch_cradle,
        base,
        axes="x",
        elem_a="pitch_shaft",
        elem_b="support_cheek_0",
        min_overlap=0.006,
        name="pitch shaft enters one support cheek",
    )
    ctx.expect_overlap(
        pitch_cradle,
        base,
        axes="x",
        elem_a="pitch_shaft",
        elem_b="support_cheek_1",
        min_overlap=0.006,
        name="pitch shaft enters opposite support cheek",
    )
    ctx.expect_within(
        roll_spindle,
        pitch_cradle,
        axes="xz",
        inner_elem="spindle_core",
        outer_elem="roll_bearing_ring",
        margin=0.002,
        name="roll spindle is centered in bearing ring",
    )
    ctx.expect_overlap(
        roll_spindle,
        pitch_cradle,
        axes="y",
        elem_a="spindle_core",
        elem_b="roll_bearing_ring",
        min_overlap=0.010,
        name="roll spindle remains engaged in cradle bearing",
    )

    rest_position = ctx.part_world_position(roll_spindle)
    with ctx.pose({pitch: 0.45}):
        raised_position = ctx.part_world_position(roll_spindle)
    ctx.check(
        "positive pitch raises sensor end",
        rest_position is not None
        and raised_position is not None
        and raised_position[2] > rest_position[2] + 0.040,
        details=f"rest={rest_position}, raised={raised_position}",
    )

    with ctx.pose({roll: 1.20}):
        ctx.expect_within(
            roll_spindle,
            pitch_cradle,
            axes="xz",
            inner_elem="spindle_core",
            outer_elem="roll_bearing_ring",
            margin=0.002,
            name="rolled spindle stays coaxial in bearing",
        )

    return ctx.report()


object_model = build_object_model()
