from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mast_top_pan_module")

    cast_black = model.material("cast_black", rgba=(0.025, 0.027, 0.030, 1.0))
    dark_metal = model.material("dark_anodized_metal", rgba=(0.09, 0.10, 0.11, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.55, 0.57, 0.58, 1.0))
    sensor_glass = model.material("dark_sensor_glass", rgba=(0.015, 0.050, 0.065, 1.0))
    status_white = model.material("white_index_mark", rgba=(0.85, 0.88, 0.86, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.155, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=cast_black,
        name="heavy_foot",
    )
    base.visual(
        Cylinder(radius=0.105, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=dark_metal,
        name="raised_boss",
    )
    base.visual(
        Cylinder(radius=0.032, length=0.150),
        origin=Origin(xyz=(0.0, 0.0, 0.143)),
        material=satin_steel,
        name="pedestal",
    )
    base.visual(
        Cylinder(radius=0.064, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
        material=dark_metal,
        name="top_bearing",
    )
    for idx, (x, y) in enumerate(
        (
            (0.105, 0.055),
            (-0.105, 0.055),
            (0.105, -0.055),
            (-0.105, -0.055),
        )
    ):
        base.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(x, y, 0.055)),
            material=satin_steel,
            name=f"foot_bolt_{idx}",
        )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.054, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=dark_metal,
        name="rotor_skirt",
    )

    housing_profile = rounded_rect_profile(0.118, 0.096, 0.014, corner_segments=8)
    housing_mesh = mesh_from_geometry(
        ExtrudeGeometry(housing_profile, 0.055, cap=True, center=True),
        "rounded_head_housing",
    )
    head.visual(
        housing_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0495)),
        material=dark_metal,
        name="head_housing",
    )
    head.visual(
        Box((0.126, 0.126, 0.007)),
        origin=Origin(xyz=(0.0, 0.0, 0.0805)),
        material=cast_black,
        name="sensor_frame",
    )
    head.visual(
        Box((0.104, 0.104, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.0870)),
        material=sensor_glass,
        name="sensor_plate",
    )
    head.visual(
        Box((0.010, 0.036, 0.002)),
        origin=Origin(xyz=(0.0, 0.035, 0.0910)),
        material=status_white,
        name="index_mark",
    )

    model.articulation(
        "pan_axis",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.232)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.4,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    head = object_model.get_part("head")
    pan_axis = object_model.get_articulation("pan_axis")

    ctx.check(
        "single vertical pan joint",
        len(object_model.articulations) == 1
        and tuple(round(v, 6) for v in pan_axis.axis) == (0.0, 0.0, 1.0)
        and pan_axis.articulation_type == ArticulationType.REVOLUTE,
        details=f"articulations={object_model.articulations}, axis={pan_axis.axis}",
    )
    ctx.expect_gap(
        head,
        base,
        axis="z",
        positive_elem="rotor_skirt",
        negative_elem="top_bearing",
        max_gap=0.001,
        max_penetration=0.0,
        name="rotor sits on pedestal bearing",
    )
    ctx.expect_within(
        head,
        base,
        axes="xy",
        inner_elem="rotor_skirt",
        outer_elem="top_bearing",
        margin=0.0,
        name="rotor is centered inside top bearing",
    )
    with ctx.pose({pan_axis: math.pi / 2.0}):
        ctx.expect_gap(
            head,
            base,
            axis="z",
            positive_elem="rotor_skirt",
            negative_elem="top_bearing",
            max_gap=0.001,
            max_penetration=0.0,
            name="pan joint preserves bearing support",
        )

    return ctx.report()


object_model = build_object_model()
