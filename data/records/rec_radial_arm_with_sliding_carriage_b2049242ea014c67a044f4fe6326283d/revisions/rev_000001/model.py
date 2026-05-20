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


def _hollow_cylinder(outer_radius: float, inner_radius: float, height: float):
    """CadQuery tube centered on the local XY plane."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
        .translate((0.0, 0.0, -height / 2.0))
    )


def _root_sleeve_mesh():
    """Flanged rotating sleeve with a true column bore."""
    main = _hollow_cylinder(0.300, 0.180, 0.520)
    upper = _hollow_cylinder(0.340, 0.180, 0.085).translate((0.0, 0.0, 0.220))
    lower = _hollow_cylinder(0.340, 0.180, 0.085).translate((0.0, 0.0, -0.220))
    return main.union(upper).union(lower)


def _saddle_body_mesh():
    """Compact carriage casting with a rectangular through-bore for the beam."""
    outer = cq.Workplane("XY").box(0.340, 0.480, 0.520)
    tunnel = cq.Workplane("XY").box(0.420, 0.340, 0.390)
    body = outer.cut(tunnel)
    return body.edges("|X").fillet(0.012)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="radial_drill_arm_module")

    cast_green = model.material("cast_green", rgba=(0.12, 0.28, 0.22, 1.0))
    dark_green = model.material("dark_green", rgba=(0.07, 0.16, 0.13, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.62, 0.64, 0.61, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.18, 1.0))
    bronze = model.material("bronze_wear", rgba=(0.70, 0.48, 0.20, 1.0))
    black = model.material("blackened_stops", rgba=(0.03, 0.035, 0.035, 1.0))

    column = model.part("column")
    column.visual(
        Cylinder(radius=0.46, length=0.105),
        origin=Origin(xyz=(0.0, 0.0, 0.0525)),
        material=dark_green,
        name="floor_plate",
    )
    column.visual(
        Cylinder(radius=0.155, length=1.92),
        origin=Origin(xyz=(0.0, 0.0, 0.985)),
        material=machined_steel,
        name="vertical_column",
    )
    column.visual(
        Cylinder(radius=0.232, length=0.160),
        origin=Origin(xyz=(0.0, 0.0, 0.9075)),
        material=machined_steel,
        name="lower_collar",
    )
    column.visual(
        Cylinder(radius=0.232, length=0.160),
        origin=Origin(xyz=(0.0, 0.0, 1.645)),
        material=machined_steel,
        name="upper_collar",
    )
    column.visual(
        Cylinder(radius=0.190, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, 0.165)),
        material=dark_steel,
        name="base_clamp_ring",
    )
    for i, angle in enumerate((math.radians(45), math.radians(135), math.radians(225), math.radians(315))):
        column.visual(
            Cylinder(radius=0.038, length=0.030),
            origin=Origin(xyz=(0.330 * math.cos(angle), 0.330 * math.sin(angle), 0.112)),
            material=dark_steel,
            name=f"anchor_bolt_{i}",
        )

    arm = model.part("arm")
    arm.visual(
        mesh_from_cadquery(_root_sleeve_mesh(), "rotating_root_sleeve", tolerance=0.001),
        material=cast_green,
        name="root_sleeve",
    )
    arm.visual(
        Box((0.300, 0.420, 0.370)),
        origin=Origin(xyz=(0.380, 0.0, 0.0)),
        material=cast_green,
        name="root_boss",
    )
    arm.visual(
        Box((1.430, 0.180, 0.240)),
        origin=Origin(xyz=(1.145, 0.0, 0.0)),
        material=cast_green,
        name="beam_core",
    )
    arm.visual(
        Box((1.340, 0.300, 0.050)),
        origin=Origin(xyz=(1.195, 0.0, 0.145)),
        material=dark_green,
        name="beam_top_flange",
    )
    arm.visual(
        Box((1.340, 0.300, 0.050)),
        origin=Origin(xyz=(1.195, 0.0, -0.145)),
        material=dark_green,
        name="beam_bottom_flange",
    )
    arm.visual(
        Box((1.340, 0.025, 0.285)),
        origin=Origin(xyz=(1.195, 0.1575, 0.0)),
        material=dark_green,
        name="front_side_rail",
    )
    arm.visual(
        Box((1.340, 0.025, 0.285)),
        origin=Origin(xyz=(1.195, -0.1575, 0.0)),
        material=dark_green,
        name="rear_side_rail",
    )
    for i, x in enumerate((0.400, 0.485, 1.835)):
        arm.visual(
            Box((0.050, 0.245, 0.150)),
            origin=Origin(xyz=(x, 0.0, -0.235)),
            material=cast_green,
            name=f"web_rib_{i}",
        )
    arm.visual(
        Box((0.110, 0.360, 0.105)),
        origin=Origin(xyz=(0.505, 0.0, 0.207)),
        material=black,
        name="inner_stop",
    )
    arm.visual(
        Box((0.110, 0.360, 0.105)),
        origin=Origin(xyz=(1.735, 0.0, 0.207)),
        material=black,
        name="outer_stop",
    )

    saddle = model.part("saddle")
    saddle.visual(
        Box((0.340, 0.480, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.230)),
        material=cast_green,
        name="saddle_body",
    )
    saddle.visual(
        Box((0.340, 0.480, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, -0.230)),
        material=cast_green,
        name="lower_bridge",
    )
    saddle.visual(
        Box((0.340, 0.070, 0.420)),
        origin=Origin(xyz=(0.0, 0.205, 0.0)),
        material=cast_green,
        name="front_jaw",
    )
    saddle.visual(
        Box((0.340, 0.070, 0.420)),
        origin=Origin(xyz=(0.0, -0.205, 0.0)),
        material=cast_green,
        name="rear_jaw",
    )
    saddle.visual(
        Box((0.360, 0.235, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.185)),
        material=bronze,
        name="upper_shoe",
    )
    saddle.visual(
        Box((0.360, 0.235, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, -0.185)),
        material=bronze,
        name="lower_shoe",
    )
    saddle.visual(
        Box((0.360, 0.010, 0.235)),
        origin=Origin(xyz=(0.0, 0.175, 0.0)),
        material=bronze,
        name="front_shoe",
    )
    saddle.visual(
        Box((0.360, 0.010, 0.235)),
        origin=Origin(xyz=(0.0, -0.175, 0.0)),
        material=bronze,
        name="rear_shoe",
    )
    saddle.visual(
        Box((0.200, 0.120, 0.240)),
        origin=Origin(xyz=(0.0, 0.290, 0.0)),
        material=dark_steel,
        name="side_clamp_pad",
    )

    model.articulation(
        "column_to_arm",
        ArticulationType.REVOLUTE,
        parent=column,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 1.250)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=650.0, velocity=0.55, lower=-2.75, upper=2.75),
    )
    model.articulation(
        "arm_to_saddle",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=saddle,
        origin=Origin(xyz=(0.760, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=0.18, lower=0.0, upper=0.700),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    column = object_model.get_part("column")
    arm = object_model.get_part("arm")
    saddle = object_model.get_part("saddle")
    swing = object_model.get_articulation("column_to_arm")
    slide = object_model.get_articulation("arm_to_saddle")

    ctx.expect_gap(
        arm,
        column,
        axis="z",
        positive_elem="root_sleeve",
        negative_elem="lower_collar",
        max_gap=0.004,
        max_penetration=0.001,
        name="rotating root seats on lower thrust collar",
    )
    ctx.expect_gap(
        column,
        arm,
        axis="z",
        positive_elem="upper_collar",
        negative_elem="root_sleeve",
        min_gap=0.035,
        name="rotating root clears upper collar",
    )

    with ctx.pose({slide: 0.0}):
        ctx.expect_gap(
            saddle,
            arm,
            axis="x",
            positive_elem="saddle_body",
            negative_elem="inner_stop",
            min_gap=0.025,
            max_gap=0.070,
            name="saddle parks clear of inner stop",
        )
        ctx.expect_overlap(
            saddle,
            arm,
            axes="x",
            elem_a="saddle_body",
            elem_b="beam_core",
            min_overlap=0.320,
            name="saddle remains guided at inner travel",
        )
        ctx.expect_contact(
            saddle,
            arm,
            elem_a="front_shoe",
            elem_b="front_side_rail",
            contact_tol=0.00001,
            name="front guide shoe bears at inner travel",
        )
        ctx.expect_gap(
            saddle,
            arm,
            axis="z",
            positive_elem="upper_shoe",
            negative_elem="beam_top_flange",
            min_gap=0.003,
            max_gap=0.008,
            name="upper shoe clears beam at inner travel",
        )
        ctx.expect_gap(
            arm,
            saddle,
            axis="z",
            positive_elem="beam_bottom_flange",
            negative_elem="lower_shoe",
            min_gap=0.003,
            max_gap=0.008,
            name="lower shoe clears beam at inner travel",
        )
        rest_pos = ctx.part_world_position(saddle)

    with ctx.pose({slide: 0.700}):
        ctx.expect_gap(
            arm,
            saddle,
            axis="x",
            positive_elem="outer_stop",
            negative_elem="saddle_body",
            min_gap=0.025,
            max_gap=0.070,
            name="saddle parks clear of outer stop",
        )
        ctx.expect_overlap(
            saddle,
            arm,
            axes="x",
            elem_a="saddle_body",
            elem_b="beam_core",
            min_overlap=0.320,
            name="saddle remains guided at outer travel",
        )
        ctx.expect_contact(
            saddle,
            arm,
            elem_a="front_shoe",
            elem_b="front_side_rail",
            contact_tol=0.00001,
            name="front guide shoe bears at outer travel",
        )
        ctx.expect_gap(
            saddle,
            arm,
            axis="z",
            positive_elem="upper_shoe",
            negative_elem="beam_top_flange",
            min_gap=0.003,
            max_gap=0.008,
            name="upper shoe clears beam at outer travel",
        )
        ctx.expect_gap(
            arm,
            saddle,
            axis="z",
            positive_elem="beam_bottom_flange",
            negative_elem="lower_shoe",
            min_gap=0.003,
            max_gap=0.008,
            name="lower shoe clears beam at outer travel",
        )
        extended_pos = ctx.part_world_position(saddle)

    ctx.check(
        "saddle translates outward along beam",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.650,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    with ctx.pose({swing: 1.0, slide: 0.350}):
        swung_pos = ctx.part_world_position(saddle)
    ctx.check(
        "arm swings around vertical column",
        swung_pos is not None and swung_pos[1] > 0.75,
        details=f"swung saddle origin={swung_pos}",
    )

    return ctx.report()


object_model = build_object_model()
