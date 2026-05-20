from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="laboratory_automation_stage")

    dark = model.material("dark_anodized_aluminum", rgba=(0.08, 0.09, 0.10, 1.0))
    black = model.material("matte_black_hardware", rgba=(0.015, 0.015, 0.014, 1.0))
    steel = model.material("brushed_steel", rgba=(0.72, 0.74, 0.72, 1.0))
    satin = model.material("satin_aluminum", rgba=(0.58, 0.61, 0.63, 1.0))

    base = model.part("base")
    base.visual(
        Box((1.35, 0.62, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=dark,
        name="ground_plate",
    )
    base.visual(
        Box((1.15, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, -0.20, 0.0725)),
        material=steel,
        name="base_rail_0",
    )
    base.visual(
        Box((1.20, 0.075, 0.012)),
        origin=Origin(xyz=(0.0, -0.20, 0.061)),
        material=black,
        name="rail_seat_0",
    )
    base.visual(
        Box((1.15, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, 0.20, 0.0725)),
        material=steel,
        name="base_rail_1",
    )
    base.visual(
        Box((1.20, 0.075, 0.012)),
        origin=Origin(xyz=(0.0, 0.20, 0.061)),
        material=black,
        name="rail_seat_1",
    )
    for i, x in enumerate((-0.58, 0.58)):
        base.visual(
            Box((0.055, 0.50, 0.060)),
            origin=Origin(xyz=(x, 0.0, 0.085)),
            material=black,
            name=f"end_stop_{i}",
        )
    base.visual(
        Cylinder(radius=0.012, length=1.12),
        origin=Origin(xyz=(0.0, 0.0, 0.092), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="x_ballscrew",
    )
    for i, x in enumerate((-0.56, 0.56)):
        base.visual(
            Box((0.060, 0.095, 0.070)),
            origin=Origin(xyz=(x, 0.0, 0.090)),
            material=black,
            name=f"x_screw_bearing_{i}",
        )

    bridge = model.part("bridge_saddle")
    bridge.visual(
        Box((0.180, 0.080, 0.050)),
        origin=Origin(xyz=(0.0, -0.20, 0.115)),
        material=black,
        name="x_bearing_block_0",
    )
    bridge.visual(
        Box((0.180, 0.080, 0.050)),
        origin=Origin(xyz=(0.0, 0.20, 0.115)),
        material=black,
        name="x_bearing_block_1",
    )
    bridge.visual(
        Box((0.230, 0.520, 0.037)),
        origin=Origin(xyz=(0.0, 0.0, 0.158)),
        material=satin,
        name="saddle_cross_plate",
    )
    for i, y in enumerate((-0.245, 0.245)):
        bridge.visual(
            Box((0.125, 0.055, 0.325)),
            origin=Origin(xyz=(0.0, y, 0.334)),
            material=dark,
            name=f"bridge_cheek_{i}",
        )
    bridge.visual(
        Box((0.180, 0.600, 0.092)),
        origin=Origin(xyz=(0.0, 0.0, 0.541)),
        material=dark,
        name="cross_beam",
    )
    bridge.visual(
        Box((0.025, 0.535, 0.022)),
        origin=Origin(xyz=(-0.1025, 0.0, 0.510)),
        material=steel,
        name="y_rail_lower",
    )
    bridge.visual(
        Box((0.025, 0.535, 0.022)),
        origin=Origin(xyz=(-0.1025, 0.0, 0.573)),
        material=steel,
        name="y_rail_upper",
    )
    bridge.visual(
        Cylinder(radius=0.0085, length=0.505),
        origin=Origin(xyz=(-0.119, 0.0, 0.541), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="y_ballscrew",
    )
    for i, y in enumerate((-0.255, 0.255)):
        bridge.visual(
            Box((0.048, 0.030, 0.105)),
            origin=Origin(xyz=(-0.119, y, 0.541)),
            material=black,
            name=f"y_screw_bearing_{i}",
        )

    cross = model.part("cross_carriage")
    cross.visual(
        Box((0.040, 0.145, 0.035)),
        origin=Origin(xyz=(-0.135, 0.0, 0.510)),
        material=black,
        name="y_bearing_block_lower",
    )
    cross.visual(
        Box((0.040, 0.145, 0.035)),
        origin=Origin(xyz=(-0.135, 0.0, 0.573)),
        material=black,
        name="y_bearing_block_upper",
    )
    cross.visual(
        Box((0.035, 0.185, 0.205)),
        origin=Origin(xyz=(-0.1725, 0.0, 0.542)),
        material=satin,
        name="carriage_plate",
    )
    cross.visual(
        Box((0.070, 0.025, 0.315)),
        origin=Origin(xyz=(-0.225, -0.060, 0.492)),
        material=black,
        name="z_guide_bar_0",
    )
    cross.visual(
        Box((0.070, 0.025, 0.315)),
        origin=Origin(xyz=(-0.225, 0.060, 0.492)),
        material=black,
        name="z_guide_bar_1",
    )
    cross.visual(
        Cylinder(radius=0.0075, length=0.225),
        origin=Origin(xyz=(-0.269, 0.0, 0.492)),
        material=steel,
        name="z_ballscrew",
    )
    cross.visual(
        Box((0.025, 0.025, 0.040)),
        origin=Origin(xyz=(-0.285, 0.0, 0.492)),
        material=black,
        name="z_screw_collar",
    )
    cross.visual(
        Box((0.025, 0.085, 0.040)),
        origin=Origin(xyz=(-0.285, 0.0425, 0.492)),
        material=black,
        name="z_screw_side_web",
    )
    cross.visual(
        Box((0.097, 0.030, 0.040)),
        origin=Origin(xyz=(-0.2385, 0.090, 0.492)),
        material=black,
        name="z_screw_mount_arm",
    )

    z_ram = model.part("z_ram")
    z_ram.visual(
        Box((0.060, 0.070, 0.300)),
        origin=Origin(xyz=(0.0, 0.0, -0.150)),
        material=satin,
        name="ram_bar",
    )
    z_ram.visual(
        Box((0.045, 0.020, 0.260)),
        origin=Origin(xyz=(0.0, -0.0375, -0.125)),
        material=black,
        name="ram_slide_shoe_0",
    )
    z_ram.visual(
        Box((0.045, 0.020, 0.260)),
        origin=Origin(xyz=(0.0, 0.0375, -0.125)),
        material=black,
        name="ram_slide_shoe_1",
    )
    flange_shape = (
        cq.Workplane("XY")
        .box(0.140, 0.085, 0.025)
        .edges("|Z")
        .fillet(0.006)
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(-0.045, -0.025), (-0.045, 0.025), (0.045, -0.025), (0.045, 0.025)])
        .hole(0.011)
    )
    z_ram.visual(
        mesh_from_cadquery(flange_shape, "compact_mounting_flange", tolerance=0.0005),
        origin=Origin(xyz=(0.0, 0.0, -0.3125)),
        material=dark,
        name="mounting_flange",
    )
    z_ram.visual(
        Box((0.050, 0.052, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.291)),
        material=black,
        name="flange_clamp",
    )

    model.articulation(
        "x_axis",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bridge,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=450.0, velocity=0.55, lower=-0.32, upper=0.32),
    )
    model.articulation(
        "y_axis",
        ArticulationType.PRISMATIC,
        parent=bridge,
        child=cross,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=300.0, velocity=0.45, lower=-0.18, upper=0.18),
    )
    model.articulation(
        "z_axis",
        ArticulationType.PRISMATIC,
        parent=cross,
        child=z_ram,
        origin=Origin(xyz=(-0.225, 0.0, 0.640)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.25, lower=0.0, upper=0.22),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    bridge = object_model.get_part("bridge_saddle")
    cross = object_model.get_part("cross_carriage")
    z_ram = object_model.get_part("z_ram")
    x_axis = object_model.get_articulation("x_axis")
    y_axis = object_model.get_articulation("y_axis")
    z_axis = object_model.get_articulation("z_axis")

    ctx.expect_gap(
        bridge,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="x_bearing_block_0",
        negative_elem="base_rail_0",
        name="bridge saddle rides on the first base rail",
    )
    ctx.expect_gap(
        bridge,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="x_bearing_block_1",
        negative_elem="base_rail_1",
        name="bridge saddle rides on the second base rail",
    )
    ctx.expect_overlap(
        bridge,
        base,
        axes="xy",
        elem_a="x_bearing_block_0",
        elem_b="base_rail_0",
        min_overlap=0.030,
        name="first linear truck is seated over its rail",
    )
    ctx.expect_gap(
        bridge,
        cross,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="y_rail_lower",
        negative_elem="y_bearing_block_lower",
        name="cross carriage lower truck bears against the bridge rail",
    )
    ctx.expect_gap(
        bridge,
        cross,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="y_rail_upper",
        negative_elem="y_bearing_block_upper",
        name="cross carriage upper truck bears against the bridge rail",
    )
    ctx.expect_overlap(
        z_ram,
        cross,
        axes="z",
        elem_a="ram_bar",
        elem_b="z_guide_bar_0",
        min_overlap=0.10,
        name="z ram remains guided through the front carriage",
    )
    with ctx.pose({z_axis: 0.22}):
        ctx.expect_overlap(
            z_ram,
            cross,
            axes="z",
            elem_a="ram_slide_shoe_0",
            elem_b="z_guide_bar_0",
            min_overlap=0.08,
            name="lowered z ram keeps retained insertion in the guide",
        )

    rest_bridge = ctx.part_world_position(bridge)
    with ctx.pose({x_axis: 0.24}):
        advanced_bridge = ctx.part_world_position(bridge)
    ctx.check(
        "bridge saddle translates along X",
        rest_bridge is not None
        and advanced_bridge is not None
        and advanced_bridge[0] > rest_bridge[0] + 0.23
        and abs(advanced_bridge[1] - rest_bridge[1]) < 1e-6
        and abs(advanced_bridge[2] - rest_bridge[2]) < 1e-6,
        details=f"rest={rest_bridge}, advanced={advanced_bridge}",
    )

    rest_cross = ctx.part_world_position(cross)
    with ctx.pose({y_axis: 0.14}):
        advanced_cross = ctx.part_world_position(cross)
    ctx.check(
        "cross carriage translates along Y",
        rest_cross is not None
        and advanced_cross is not None
        and advanced_cross[1] > rest_cross[1] + 0.13
        and abs(advanced_cross[0] - rest_cross[0]) < 1e-6
        and abs(advanced_cross[2] - rest_cross[2]) < 1e-6,
        details=f"rest={rest_cross}, advanced={advanced_cross}",
    )

    rest_ram = ctx.part_world_position(z_ram)
    with ctx.pose({z_axis: 0.18}):
        lowered_ram = ctx.part_world_position(z_ram)
    ctx.check(
        "z ram descends on the Z axis",
        rest_ram is not None
        and lowered_ram is not None
        and lowered_ram[2] < rest_ram[2] - 0.17
        and abs(lowered_ram[0] - rest_ram[0]) < 1e-6
        and abs(lowered_ram[1] - rest_ram[1]) < 1e-6,
        details=f"rest={rest_ram}, lowered={lowered_ram}",
    )

    return ctx.report()


object_model = build_object_model()
