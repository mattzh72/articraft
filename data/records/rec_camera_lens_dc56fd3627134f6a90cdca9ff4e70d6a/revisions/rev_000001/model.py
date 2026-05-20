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


def _tube_x(
    outer_radius: float,
    inner_radius: float,
    length: float,
    center_x: float,
) -> cq.Workplane:
    """Return a hollow cylindrical sleeve with its optical axis along local X."""
    return (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((center_x - length / 2.0, 0.0, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="macro_1_1_prime_lens")

    satin_black = Material("satin_black", rgba=(0.01, 0.01, 0.012, 1.0))
    rubber_black = Material("ribbed_rubber", rgba=(0.005, 0.005, 0.006, 1.0))
    matte_black = Material("matte_black", rgba=(0.0, 0.0, 0.0, 1.0))
    anodized = Material("dark_anodized", rgba=(0.035, 0.035, 0.04, 1.0))
    mount_metal = Material("brushed_mount", rgba=(0.62, 0.61, 0.57, 1.0))
    glass = Material("coated_glass", rgba=(0.08, 0.16, 0.20, 0.46))
    white = Material("white_engraving", rgba=(0.92, 0.92, 0.86, 1.0))

    outer_barrel = model.part("outer_barrel")
    outer_barrel.visual(
        mesh_from_cadquery(_tube_x(0.054, 0.046, 0.100, -0.005), "outer_sleeve"),
        material=satin_black,
        name="outer_sleeve",
    )
    outer_barrel.visual(
        mesh_from_cadquery(_tube_x(0.057, 0.046, 0.010, 0.040), "front_lip"),
        material=anodized,
        name="front_lip",
    )
    outer_barrel.visual(
        mesh_from_cadquery(_tube_x(0.047, 0.031, 0.016, -0.062), "rear_mount"),
        material=mount_metal,
        name="rear_mount",
    )
    outer_barrel.visual(
        mesh_from_cadquery(_tube_x(0.060, 0.052, 0.004, -0.033), "rear_ring_stop"),
        material=anodized,
        name="rear_ring_stop",
    )
    outer_barrel.visual(
        mesh_from_cadquery(_tube_x(0.060, 0.052, 0.004, -0.003), "front_ring_stop"),
        material=anodized,
        name="front_ring_stop",
    )
    outer_barrel.visual(
        Cylinder(radius=0.032, length=0.003),
        origin=Origin(xyz=(-0.067, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="rear_glass",
    )
    outer_barrel.visual(
        Box((0.072, 0.0002, 0.00315)),
        origin=Origin(xyz=(0.006, 0.0, -0.044525)),
        material=matte_black,
        name="focus_guide_rail",
    )
    outer_barrel.visual(
        Box((0.008, 0.0035, 0.0015)),
        origin=Origin(xyz=(-0.002, 0.0, 0.0602)),
        material=white,
        name="aperture_index",
    )
    outer_barrel.visual(
        Box((0.040, 0.0025, 0.0012)),
        origin=Origin(xyz=(0.016, 0.0, 0.0578)),
        material=white,
        name="macro_label_bar",
    )

    aperture_ring = model.part("aperture_ring")
    aperture_ring.visual(
        mesh_from_cadquery(_tube_x(0.061, 0.056, 0.024, 0.0), "aperture_ring_body"),
        material=anodized,
        name="ring_body",
    )
    for i in range(24):
        angle = 2.0 * math.pi * i / 24.0
        aperture_ring.visual(
            Box((0.018, 0.0030, 0.0030)),
            origin=Origin(
                xyz=(0.0, 0.0618 * math.sin(angle), 0.0618 * math.cos(angle)),
                rpy=(angle, 0.0, 0.0),
            ),
            material=matte_black,
            name=f"grip_rail_{i}",
        )
    for i, angle_deg in enumerate((-42, -24, -8, 8, 24, 42)):
        angle = math.radians(angle_deg)
        aperture_ring.visual(
            Box((0.0020, 0.0080, 0.0010)),
            origin=Origin(
                xyz=(0.007, 0.0623 * math.sin(angle), 0.0623 * math.cos(angle)),
                rpy=(angle, 0.0, 0.0),
            ),
            material=white,
            name=f"f_stop_tick_{i}",
        )

    inner_barrel = model.part("inner_barrel")
    inner_barrel.visual(
        mesh_from_cadquery(_tube_x(0.043, 0.035, 0.165, 0.0325), "inner_sleeve"),
        material=satin_black,
        name="inner_sleeve",
    )
    inner_barrel.visual(
        mesh_from_cadquery(_tube_x(0.046, 0.034, 0.014, 0.119), "front_filter_ring"),
        material=anodized,
        name="front_filter_ring",
    )
    inner_barrel.visual(
        Cylinder(radius=0.0355, length=0.004),
        origin=Origin(xyz=(0.124, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="front_glass",
    )
    for i in range(32):
        angle = 2.0 * math.pi * i / 32.0
        inner_barrel.visual(
            Box((0.078, 0.0030, 0.0035)),
            origin=Origin(
                xyz=(0.040, 0.0442 * math.sin(angle), 0.0442 * math.cos(angle)),
                rpy=(angle, 0.0, 0.0),
            ),
            material=rubber_black,
            name=f"focus_rib_{i}",
        )
    inner_barrel.visual(
        Box((0.018, 0.0020, 0.0010)),
        origin=Origin(xyz=(0.070, 0.0, 0.0455)),
        material=white,
        name="focus_scale_mark",
    )

    model.articulation(
        "focus_slide",
        ArticulationType.PRISMATIC,
        parent=outer_barrel,
        child=inner_barrel,
        origin=Origin(xyz=(0.045, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.18, lower=0.0, upper=0.040),
    )
    model.articulation(
        "aperture_rotation",
        ArticulationType.CONTINUOUS,
        parent=outer_barrel,
        child=aperture_ring,
        origin=Origin(xyz=(-0.018, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=4.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_barrel")
    inner = object_model.get_part("inner_barrel")
    ring = object_model.get_part("aperture_ring")
    focus = object_model.get_articulation("focus_slide")
    aperture = object_model.get_articulation("aperture_rotation")

    ctx.expect_overlap(
        inner,
        outer,
        axes="x",
        elem_a="inner_sleeve",
        elem_b="outer_sleeve",
        min_overlap=0.045,
        name="collapsed focusing barrel remains inserted",
    )
    ctx.expect_within(
        inner,
        outer,
        axes="yz",
        inner_elem="inner_sleeve",
        outer_elem="outer_sleeve",
        margin=0.0,
        name="focusing barrel is coaxial with the outer sleeve",
    )
    ctx.expect_contact(
        outer,
        inner,
        elem_a="focus_guide_rail",
        elem_b="inner_sleeve",
        contact_tol=0.0001,
        name="hidden guide rail supports the focusing barrel",
    )
    ctx.expect_overlap(
        ring,
        outer,
        axes="x",
        elem_a="ring_body",
        elem_b="outer_sleeve",
        min_overlap=0.020,
        name="aperture ring wraps the outer barrel",
    )
    ctx.expect_origin_distance(
        ring,
        outer,
        axes="yz",
        max_dist=0.001,
        name="aperture ring rotates about the lens axis",
    )

    rest_pos = ctx.part_world_position(inner)
    with ctx.pose({focus: 0.040}):
        ctx.expect_overlap(
            inner,
            outer,
            axes="x",
            elem_a="inner_sleeve",
            elem_b="outer_sleeve",
            min_overlap=0.009,
            name="extended focusing barrel keeps retained insertion",
        )
        ctx.expect_within(
            inner,
            outer,
            axes="yz",
            inner_elem="inner_sleeve",
            outer_elem="outer_sleeve",
            margin=0.0,
            name="extended focusing barrel remains coaxial",
        )
        extended_pos = ctx.part_world_position(inner)
    ctx.check(
        "focus slide extends forward along the optical axis",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.035,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    with ctx.pose({aperture: 1.7}):
        ctx.expect_origin_distance(
            ring,
            outer,
            axes="yz",
            max_dist=0.001,
            name="rotated aperture ring stays coaxial",
        )

    return ctx.report()


object_model = build_object_model()
