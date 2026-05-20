from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="two_axis_positioning_stage")

    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.70, 0.72, 0.70, 1.0))
    dark_anodized = model.material("dark_anodized", rgba=(0.05, 0.055, 0.060, 1.0))
    rail_steel = model.material("polished_rail_steel", rgba=(0.82, 0.84, 0.86, 1.0))
    bearing_black = model.material("black_bearing_blocks", rgba=(0.015, 0.017, 0.020, 1.0))
    hardware = model.material("dark_socket_screws", rgba=(0.0, 0.0, 0.0, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.56, 0.32, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=brushed_aluminum,
        name="base_plate",
    )

    for y, side in ((-0.10, "front"), (0.10, "rear")):
        base.visual(
            Box((0.50, 0.020, 0.031)),
            origin=Origin(xyz=(0.0, y, 0.0405)),
            material=dark_anodized,
            name=f"x_rail_support_{side}",
        )
        base.visual(
            Cylinder(radius=0.012, length=0.50),
            origin=Origin(xyz=(0.0, y, 0.067), rpy=(0.0, pi / 2.0, 0.0)),
            material=rail_steel,
            name=f"x_rail_{side}",
        )
        for x, end in ((-0.255, "low"), (0.255, "high")):
            base.visual(
                Box((0.022, 0.056, 0.045)),
                origin=Origin(xyz=(x, y, 0.0475)),
                material=dark_anodized,
                name=f"x_end_stop_{side}_{end}",
            )

    for x, y, corner in (
        (-0.24, -0.13, "0"),
        (-0.24, 0.13, "1"),
        (0.24, -0.13, "2"),
        (0.24, 0.13, "3"),
    ):
        base.visual(
            Cylinder(radius=0.012, length=0.004),
            origin=Origin(xyz=(x, y, 0.027)),
            material=hardware,
            name=f"base_screw_{corner}",
        )

    x_carriage = model.part("x_carriage")
    for x in (-0.055, 0.055):
        for y, side in ((-0.10, "front"), (0.10, "rear")):
            x_carriage.visual(
                Box((0.075, 0.045, 0.012)),
                origin=Origin(xyz=(x, y, 0.006)),
                material=bearing_black,
                name=f"x_bearing_{side}_{'neg' if x < 0.0 else 'pos'}",
            )

    x_carriage.visual(
        Box((0.19, 0.25, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=dark_anodized,
        name="x_saddle_plate",
    )
    x_carriage.visual(
        Box((0.17, 0.024, 0.320)),
        origin=Origin(xyz=(0.0, 0.0, 0.194)),
        material=brushed_aluminum,
        name="z_backplate",
    )
    x_carriage.visual(
        Box((0.18, 0.095, 0.032)),
        origin=Origin(xyz=(0.0, 0.010, 0.050)),
        material=dark_anodized,
        name="z_base_foot",
    )

    for z, end in ((0.055, "bottom"), (0.315, "top")):
        x_carriage.visual(
            Box((0.155, 0.040, 0.026)),
            origin=Origin(xyz=(0.0, -0.010, z)),
            material=dark_anodized,
            name=f"z_rail_clamp_{end}",
        )

    for x, side in ((-0.055, "neg"), (0.055, "pos")):
        x_carriage.visual(
            Cylinder(radius=0.008, length=0.250),
            origin=Origin(xyz=(x, -0.022, 0.185)),
            material=rail_steel,
            name=f"z_rail_{side}",
        )

    z_slider = model.part("z_slider")
    for x, side in ((-0.055, "neg"), (0.055, "pos")):
        z_slider.visual(
            Box((0.032, 0.018, 0.060)),
            origin=Origin(xyz=(x, -0.039, 0.0)),
            material=bearing_black,
            name=f"z_bearing_{side}",
        )

    z_slider.visual(
        Box((0.142, 0.020, 0.140)),
        origin=Origin(xyz=(0.0, -0.052, 0.0)),
        material=dark_anodized,
        name="z_cross_bridge",
    )
    z_slider.visual(
        Box((0.160, 0.012, 0.160)),
        origin=Origin(xyz=(0.0, -0.069, 0.0)),
        material=brushed_aluminum,
        name="payload_plate",
    )
    for x in (-0.050, 0.050):
        for z in (-0.050, 0.050):
            z_slider.visual(
                Box((0.018, 0.020, 0.018)),
                origin=Origin(xyz=(x, -0.061, z)),
                material=dark_anodized,
                name=f"payload_standoff_{'neg' if x < 0.0 else 'pos'}_{'low' if z < 0.0 else 'high'}",
            )
            z_slider.visual(
                Cylinder(radius=0.006, length=0.005),
                origin=Origin(xyz=(x, -0.077, z), rpy=(pi / 2.0, 0.0, 0.0)),
                material=hardware,
                name=f"payload_screw_{'neg' if x < 0.0 else 'pos'}_{'low' if z < 0.0 else 'high'}",
            )

    model.articulation(
        "x_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.079)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=150.0, velocity=0.35, lower=-0.10, upper=0.10),
    )
    model.articulation(
        "z_slide",
        ArticulationType.PRISMATIC,
        parent=x_carriage,
        child=z_slider,
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.25, lower=0.0, upper=0.16),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    x_carriage = object_model.get_part("x_carriage")
    z_slider = object_model.get_part("z_slider")
    x_slide = object_model.get_articulation("x_slide")
    z_slide = object_model.get_articulation("z_slide")

    ctx.expect_contact(
        x_carriage,
        base,
        elem_a="x_bearing_front_neg",
        elem_b="x_rail_front",
        contact_tol=0.001,
        name="x carriage bearing rides front rail",
    )
    ctx.expect_contact(
        z_slider,
        x_carriage,
        elem_a="z_bearing_neg",
        elem_b="z_rail_neg",
        contact_tol=0.001,
        name="z slider bearing rides vertical rail",
    )
    ctx.expect_overlap(
        z_slider,
        x_carriage,
        axes="xz",
        elem_a="payload_plate",
        elem_b="z_backplate",
        min_overlap=0.12,
        name="square payload plate aligns with z guide",
    )

    rest_x = ctx.part_world_position(x_carriage)
    rest_z = ctx.part_world_position(z_slider)
    with ctx.pose({x_slide: 0.10}):
        extended_x = ctx.part_world_position(x_carriage)
    with ctx.pose({z_slide: 0.16}):
        extended_z = ctx.part_world_position(z_slider)

    ctx.check(
        "x slide translates along x",
        rest_x is not None
        and extended_x is not None
        and extended_x[0] > rest_x[0] + 0.095
        and abs(extended_x[2] - rest_x[2]) < 0.002,
        details=f"rest={rest_x}, extended={extended_x}",
    )
    ctx.check(
        "z slide translates upward",
        rest_z is not None
        and extended_z is not None
        and extended_z[2] > rest_z[2] + 0.155
        and abs(extended_z[0] - rest_z[0]) < 0.002,
        details=f"rest={rest_z}, extended={extended_z}",
    )

    return ctx.report()


object_model = build_object_model()
