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
    model = ArticulatedObject(name="linear_carriage_rotary_module")

    dark = model.material("black_anodized", rgba=(0.025, 0.028, 0.032, 1.0))
    blue = model.material("blue_anodized", rgba=(0.02, 0.16, 0.42, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    rail = model.material("ground_steel", rgba=(0.78, 0.80, 0.82, 1.0))
    bolt = model.material("black_oxide", rgba=(0.006, 0.006, 0.007, 1.0))

    base = model.part("base")
    base.visual(
        Box((1.05, 0.34, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=dark,
        name="base_plate",
    )
    for y, suffix in ((-0.105, "0"), (0.105, "1")):
        base.visual(
            Box((0.92, 0.040, 0.035)),
            origin=Origin(xyz=(0.0, y, 0.0725)),
            material=rail,
            name=f"linear_rail_{suffix}",
        )
        for x in (-0.36, -0.18, 0.0, 0.18, 0.36):
            base.visual(
                Cylinder(radius=0.0075, length=0.006),
                origin=Origin(xyz=(x, y, 0.087)),
                material=bolt,
                name=f"rail_bolt_{suffix}_{int((x + 0.36) * 100):02d}",
            )
    base.visual(
        Cylinder(radius=0.012, length=0.86),
        origin=Origin(xyz=(0.0, 0.0, 0.070), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="lead_screw",
    )
    for x, suffix in ((-0.455, "rear"), (0.455, "front")):
        base.visual(
            Box((0.060, 0.115, 0.090)),
            origin=Origin(xyz=(x, 0.0, 0.100)),
            material=dark,
            name=f"{suffix}_bearing_block",
        )
        base.visual(
            Cylinder(radius=0.023, length=0.006),
            origin=Origin(xyz=(x, 0.0, 0.070), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"{suffix}_bearing_face",
        )
    for x, suffix in ((-0.525, "rear"), (0.525, "front")):
        base.visual(
            Box((0.030, 0.34, 0.120)),
            origin=Origin(xyz=(x, 0.0, 0.060)),
            material=dark,
            name=f"{suffix}_end_plate",
        )

    carriage = model.part("carriage")
    for y, suffix in ((-0.105, "0"), (0.105, "1")):
        carriage.visual(
            Box((0.220, 0.055, 0.018)),
            origin=Origin(xyz=(0.0, y, 0.009)),
            material=blue,
            name=f"bearing_shoe_{suffix}",
        )
    carriage.visual(
        Box((0.285, 0.255, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=blue,
        name="carriage_plate",
    )
    carriage.visual(
        Box((0.085, 0.180, 0.080)),
        origin=Origin(xyz=(0.110, 0.0, 0.098)),
        material=blue,
        name="riser_block",
    )
    for y, suffix in ((-0.082, "0"), (0.082, "1")):
        carriage.visual(
            Box((0.100, 0.016, 0.080)),
            origin=Origin(xyz=(0.070, y, 0.098)),
            material=blue,
            name=f"riser_gusset_{suffix}",
        )
    for x in (-0.095, 0.095):
        for y in (-0.085, 0.085):
            carriage.visual(
                Cylinder(radius=0.009, length=0.007),
                origin=Origin(xyz=(x, y, 0.0615)),
                material=bolt,
                name=f"carriage_bolt_{'n' if x < 0 else 'p'}_{'n' if y < 0 else 'p'}",
            )

    rotary_stage = model.part("rotary_stage")
    rotary_stage.visual(
        Cylinder(radius=0.078, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=steel,
        name="turntable_disk",
    )
    rotary_stage.visual(
        Cylinder(radius=0.055, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
        material=dark,
        name="upper_hub",
    )
    for i, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        rotary_stage.visual(
            Cylinder(radius=0.0065, length=0.006),
            origin=Origin(xyz=(0.050 * math.cos(angle), 0.050 * math.sin(angle), 0.051)),
            material=bolt,
            name=f"turntable_bolt_{i}",
        )
    rotary_stage.visual(
        Box((0.130, 0.090, 0.035)),
        origin=Origin(xyz=(0.095, 0.0, 0.065)),
        material=dark,
        name="fork_base",
    )
    rotary_stage.visual(
        Box((0.058, 0.085, 0.090)),
        origin=Origin(xyz=(0.070, 0.0, 0.105)),
        material=dark,
        name="rear_web",
    )
    for y, suffix in ((-0.0475, "0"), (0.0475, "1")):
        rotary_stage.visual(
            Box((0.044, 0.015, 0.120)),
            origin=Origin(xyz=(0.170, y, 0.108)),
            material=dark,
            name=f"fork_cheek_{suffix}",
        )
        rotary_stage.visual(
            Cylinder(radius=0.026, length=0.010),
            origin=Origin(xyz=(0.170, y * 1.16, 0.125), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"bearing_boss_{suffix}",
        )
        for z, zs in ((0.088, "low"), (0.162, "high")):
            rotary_stage.visual(
                Cylinder(radius=0.0055, length=0.005),
                origin=Origin(xyz=(0.170, y * 1.21, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=bolt,
                name=f"cheek_bolt_{suffix}_{zs}",
            )

    output_wrist = model.part("output_wrist")
    output_wrist.visual(
        Cylinder(radius=0.032, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="cross_hub",
    )
    output_wrist.visual(
        Box((0.230, 0.054, 0.040)),
        origin=Origin(xyz=(0.115, 0.0, 0.0)),
        material=steel,
        name="output_arm",
    )
    output_wrist.visual(
        Box((0.052, 0.094, 0.084)),
        origin=Origin(xyz=(0.232, 0.0, 0.0)),
        material=steel,
        name="tool_flange",
    )
    for y in (-0.027, 0.027):
        for z in (-0.025, 0.025):
            output_wrist.visual(
                Cylinder(radius=0.0065, length=0.006),
                origin=Origin(xyz=(0.261, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=bolt,
                name=f"flange_bolt_{'n' if y < 0 else 'p'}_{'n' if z < 0 else 'p'}",
            )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(-0.280, 0.0, 0.090)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.35, lower=0.0, upper=0.45),
    )
    model.articulation(
        "carriage_to_rotary",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=rotary_stage,
        origin=Origin(xyz=(0.110, 0.0, 0.138)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.5, lower=-1.4, upper=1.4),
    )
    model.articulation(
        "rotary_to_wrist",
        ArticulationType.REVOLUTE,
        parent=rotary_stage,
        child=output_wrist,
        origin=Origin(xyz=(0.170, 0.0, 0.125)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=-0.95, upper=1.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    rotary = object_model.get_part("rotary_stage")
    wrist = object_model.get_part("output_wrist")
    slide = object_model.get_articulation("base_to_carriage")
    yaw = object_model.get_articulation("carriage_to_rotary")
    pitch = object_model.get_articulation("rotary_to_wrist")

    ctx.expect_contact(
        carriage,
        base,
        elem_a="bearing_shoe_0",
        elem_b="linear_rail_0",
        name="carriage shoe sits on rail",
    )
    ctx.expect_contact(
        rotary,
        carriage,
        elem_a="turntable_disk",
        elem_b="riser_block",
        name="rotary turntable seats on riser",
    )
    ctx.expect_within(
        wrist,
        rotary,
        axes="y",
        inner_elem="cross_hub",
        outer_elem="fork_cheek_0",
        margin=0.090,
        name="wrist hub is retained between fork cheeks",
    )

    rest_aabb = ctx.part_element_world_aabb(carriage, elem="carriage_plate")
    with ctx.pose({slide: 0.45}):
        ctx.expect_overlap(
            carriage,
            base,
            axes="x",
            elem_a="bearing_shoe_0",
            elem_b="linear_rail_0",
            min_overlap=0.12,
            name="extended carriage remains on rail",
        )
        extended_aabb = ctx.part_element_world_aabb(carriage, elem="carriage_plate")
    ctx.check(
        "linear stage travels forward",
        rest_aabb is not None
        and extended_aabb is not None
        and extended_aabb[0][0] > rest_aabb[0][0] + 0.40,
        details=f"rest={rest_aabb}, extended={extended_aabb}",
    )

    flange_rest = ctx.part_element_world_aabb(wrist, elem="tool_flange")
    with ctx.pose({yaw: 0.8}):
        flange_yaw = ctx.part_element_world_aabb(wrist, elem="tool_flange")
    ctx.check(
        "first rotary joint sweeps the wrist sideways",
        flange_rest is not None
        and flange_yaw is not None
        and abs(flange_yaw[0][1] - flange_rest[0][1]) > 0.08,
        details=f"rest={flange_rest}, yawed={flange_yaw}",
    )

    with ctx.pose({pitch: 0.75}):
        flange_pitch = ctx.part_element_world_aabb(wrist, elem="tool_flange")
    ctx.check(
        "second rotary joint pitches the output flange",
        flange_rest is not None
        and flange_pitch is not None
        and abs(flange_pitch[0][2] - flange_rest[0][2]) > 0.04,
        details=f"rest={flange_rest}, pitched={flange_pitch}",
    )

    return ctx.report()


object_model = build_object_model()
