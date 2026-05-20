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


def _annular_disc(outer_radius: float, inner_radius: float, thickness: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(thickness)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="record_turntable")

    walnut = Material("satin_walnut", rgba=(0.42, 0.24, 0.12, 1.0))
    black = Material("matte_black", rgba=(0.005, 0.005, 0.004, 1.0))
    rubber = Material("charcoal_rubber", rgba=(0.015, 0.016, 0.015, 1.0))
    groove = Material("soft_groove_highlight", rgba=(0.09, 0.09, 0.085, 1.0))
    metal = Material("brushed_aluminum", rgba=(0.72, 0.70, 0.66, 1.0))
    dark_metal = Material("dark_anodized_metal", rgba=(0.06, 0.065, 0.07, 1.0))
    label = Material("cream_record_label", rgba=(0.88, 0.76, 0.52, 1.0))
    stylus = Material("ruby_stylus", rgba=(0.65, 0.02, 0.03, 1.0))

    plinth_shape = (
        cq.Workplane("XY")
        .box(0.44, 0.36, 0.060)
        .edges("|Z")
        .fillet(0.022)
        .edges(">Z")
        .fillet(0.006)
    )

    plinth = model.part("plinth")
    plinth.visual(
        mesh_from_cadquery(plinth_shape, "rounded_walnut_plinth"),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=walnut,
        name="plinth_shell",
    )
    plinth.visual(
        Cylinder(radius=0.041, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        material=metal,
        name="bearing_puck",
    )
    plinth.visual(
        Cylinder(radius=0.028, length=0.080),
        origin=Origin(xyz=(0.180, -0.105, 0.100)),
        material=dark_metal,
        name="tonearm_base",
    )
    plinth.visual(
        Cylinder(radius=0.043, length=0.009),
        origin=Origin(xyz=(0.180, -0.105, 0.0645)),
        material=metal,
        name="tonearm_base_flange",
    )
    for index, (x, y) in enumerate(
        ((-0.172, -0.132), (-0.172, 0.132), (0.172, -0.132), (0.172, 0.132))
    ):
        plinth.visual(
            Cylinder(radius=0.028, length=0.014),
            origin=Origin(xyz=(x, y, -0.006)),
            material=black,
            name=f"foot_{index}",
        )

    platter = model.part("platter")
    platter.visual(
        Cylinder(radius=0.158, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=metal,
        name="platter_disk",
    )
    platter.visual(
        Cylinder(radius=0.151, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=rubber,
        name="rubber_mat",
    )
    platter.visual(
        Cylinder(radius=0.146, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0375)),
        material=black,
        name="vinyl_record",
    )
    for index, radius in enumerate((0.055, 0.075, 0.095, 0.115, 0.132)):
        platter.visual(
            mesh_from_cadquery(
                _annular_disc(radius + 0.0012, radius - 0.0012, 0.00055),
                f"record_groove_{index}",
                tolerance=0.0005,
            ),
            origin=Origin(xyz=(0.0, 0.0, 0.03880)),
            material=groove,
            name=f"groove_{index}",
        )
    platter.visual(
        Cylinder(radius=0.034, length=0.0038),
        origin=Origin(xyz=(0.0, 0.0, 0.0400)),
        material=label,
        name="center_label",
    )
    platter.visual(
        Cylinder(radius=0.0042, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
        material=metal,
        name="spindle",
    )

    tonearm = model.part("tonearm")
    tonearm.visual(
        Cylinder(radius=0.024, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        material=dark_metal,
        name="pivot_cap",
    )
    tonearm.visual(
        Cylinder(radius=0.006, length=0.214),
        origin=Origin(xyz=(-0.107, 0.0, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="arm_tube",
    )
    tonearm.visual(
        Cylinder(radius=0.017, length=0.052),
        origin=Origin(xyz=(0.038, 0.0, 0.012), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="counterweight",
    )
    tonearm.visual(
        Box((0.040, 0.028, 0.007)),
        origin=Origin(xyz=(-0.226, 0.0, -0.004)),
        material=black,
        name="headshell",
    )
    tonearm.visual(
        Cylinder(radius=0.0022, length=0.024),
        origin=Origin(xyz=(-0.241, 0.0, -0.019)),
        material=stylus,
        name="stylus_tip",
    )

    model.articulation(
        "platter_spin",
        ArticulationType.CONTINUOUS,
        parent=plinth,
        child=platter,
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=6.0),
    )
    model.articulation(
        "tonearm_pivot",
        ArticulationType.REVOLUTE,
        parent=plinth,
        child=tonearm,
        origin=Origin(xyz=(0.180, -0.105, 0.150)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.35, velocity=1.2, lower=-0.48, upper=0.42),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    plinth = object_model.get_part("plinth")
    platter = object_model.get_part("platter")
    tonearm = object_model.get_part("tonearm")
    spin = object_model.get_articulation("platter_spin")
    pivot = object_model.get_articulation("tonearm_pivot")

    ctx.check(
        "platter has continuous vertical rotation",
        spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.check(
        "tonearm swings on limited vertical pivot",
        pivot.articulation_type == ArticulationType.REVOLUTE
        and tuple(pivot.axis) == (0.0, 0.0, 1.0)
        and pivot.motion_limits is not None
        and pivot.motion_limits.lower < 0.0
        and pivot.motion_limits.upper > 0.0,
        details=f"type={pivot.articulation_type}, axis={pivot.axis}, limits={pivot.motion_limits}",
    )

    ctx.expect_gap(
        platter,
        plinth,
        axis="z",
        positive_elem="platter_disk",
        negative_elem="bearing_puck",
        max_gap=0.001,
        max_penetration=0.0,
        name="platter rests on bearing puck",
    )
    ctx.expect_overlap(
        platter,
        plinth,
        axes="xy",
        elem_a="platter_disk",
        elem_b="bearing_puck",
        min_overlap=0.070,
        name="platter centered over bearing",
    )
    ctx.expect_gap(
        tonearm,
        plinth,
        axis="z",
        positive_elem="pivot_cap",
        negative_elem="tonearm_base",
        max_gap=0.001,
        max_penetration=0.0001,
        name="tonearm pivot cap seats on base",
    )

    with ctx.pose({spin: 1.2, pivot: -0.35}):
        ctx.expect_overlap(
            tonearm,
            platter,
            axes="xy",
            elem_a="headshell",
            elem_b="vinyl_record",
            min_overlap=0.004,
            name="swinging headshell reaches the record",
        )

    return ctx.report()


object_model = build_object_model()
