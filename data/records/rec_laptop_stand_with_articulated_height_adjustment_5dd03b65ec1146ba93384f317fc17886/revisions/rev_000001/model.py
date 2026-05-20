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


ALUMINUM = Material("satin_aluminum", rgba=(0.72, 0.74, 0.73, 1.0))
DARK_ALUMINUM = Material("dark_anodized_aluminum", rgba=(0.30, 0.31, 0.31, 1.0))
RUBBER = Material("matte_black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))


def _oval_base_plate() -> object:
    """Low, filleted oval footprint typical of a cast aluminum stand base."""
    return (
        cq.Workplane("XY")
        .ellipse(0.18, 0.12)
        .extrude(0.018)
    )


def _hollow_sleeve() -> object:
    """Cylindrical outer sleeve with a real open bore for the sliding pedestal."""
    return (
        cq.Workplane("XY")
        .circle(0.040)
        .circle(0.021)
        .extrude(0.240)
        .translate((0.0, 0.0, 0.018))
    )


def _top_platform_plate() -> object:
    """Rounded aluminum platform with a broad rounded cooling cutout."""
    plate = (
        cq.Workplane("XY")
        .box(0.360, 0.250, 0.012)
    )
    cooling_cutout = (
        cq.Workplane("XY")
        .slot2D(0.205, 0.105)
        .extrude(0.040)
        .translate((0.005, 0.0, -0.020))
    )
    return plate.cut(cooling_cutout)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="aluminum_laptop_stand")

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_oval_base_plate(), "oval_base_plate", tolerance=0.0008),
        origin=Origin(),
        material=ALUMINUM,
        name="oval_base_plate",
    )
    base.visual(
        mesh_from_cadquery(_hollow_sleeve(), "pedestal_sleeve", tolerance=0.0008),
        origin=Origin(),
        material=ALUMINUM,
        name="pedestal_sleeve",
    )
    base.visual(
        Cylinder(radius=0.052, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=DARK_ALUMINUM,
        name="sleeve_collar",
    )

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.021, length=0.500),
        # The mast extends below the prismatic joint so it remains captured
        # inside the fixed sleeve at full height.
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=ALUMINUM,
        name="sliding_mast",
    )
    pedestal.visual(
        Cylinder(radius=0.032, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.315)),
        material=DARK_ALUMINUM,
        name="head_collar",
    )
    pedestal.visual(
        Box((0.075, 0.140, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.322)),
        material=ALUMINUM,
        name="head_bridge",
    )
    for y in (-0.058, 0.058):
        pedestal.visual(
            Box((0.060, 0.018, 0.100)),
            origin=Origin(xyz=(0.0, y, 0.380)),
            material=ALUMINUM,
            name=f"head_cheek_{0 if y < 0 else 1}",
        )
    pedestal.visual(
        Cylinder(radius=0.006, length=0.160),
        origin=Origin(xyz=(0.0, 0.0, 0.390), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=DARK_ALUMINUM,
        name="hinge_pin",
    )

    platform = model.part("platform")
    platform.visual(
        mesh_from_cadquery(_top_platform_plate(), "cooling_platform_plate", tolerance=0.0008),
        # The platform part frame is the hinge axis.  The broad plate extends
        # forward along +X and sits just below the hinge barrel.
        origin=Origin(xyz=(0.230, 0.0, -0.035)),
        material=ALUMINUM,
        name="cooling_platform_plate",
    )
    platform.visual(
        Cylinder(radius=0.016, length=0.080),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=ALUMINUM,
        name="hinge_barrel",
    )
    platform.visual(
        Box((0.090, 0.090, 0.018)),
        origin=Origin(xyz=(0.045, 0.0, -0.020)),
        material=ALUMINUM,
        name="barrel_strap",
    )
    platform.visual(
        Box((0.026, 0.235, 0.032)),
        origin=Origin(xyz=(0.402, 0.0, -0.015)),
        material=ALUMINUM,
        name="front_lip",
    )
    for index, (x, y) in enumerate(
        ((0.140, -0.092), (0.140, 0.092), (0.315, -0.092), (0.315, 0.092))
    ):
        platform.visual(
            Box((0.070, 0.018, 0.004)),
            origin=Origin(xyz=(x, y, -0.0275)),
            material=RUBBER,
            name=f"rubber_pad_{index}",
        )

    model.articulation(
        "base_to_pedestal",
        ArticulationType.PRISMATIC,
        parent=base,
        child=pedestal,
        origin=Origin(xyz=(0.0, 0.0, 0.258)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.18, lower=0.0, upper=0.120),
    )
    model.articulation(
        "pedestal_to_platform",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=platform,
        origin=Origin(xyz=(0.0, 0.0, 0.390)),
        # With the plate extending along local +X, negative Y makes positive
        # joint travel lift the front edge upward.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=-0.20, upper=0.90),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    pedestal = object_model.get_part("pedestal")
    platform = object_model.get_part("platform")
    height_slide = object_model.get_articulation("base_to_pedestal")
    pitch_hinge = object_model.get_articulation("pedestal_to_platform")

    ctx.allow_overlap(
        pedestal,
        platform,
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        reason="The dark hinge pin is intentionally captured inside the platform barrel.",
    )
    ctx.allow_overlap(
        base,
        pedestal,
        elem_a="pedestal_sleeve",
        elem_b="sliding_mast",
        reason="The sliding mast is intentionally nested in the fixed sleeve as a retained prismatic guide.",
    )
    ctx.expect_overlap(
        pedestal,
        platform,
        axes="y",
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        min_overlap=0.075,
        name="hinge pin spans the platform barrel",
    )
    ctx.expect_within(
        pedestal,
        base,
        axes="xy",
        inner_elem="sliding_mast",
        outer_elem="pedestal_sleeve",
        margin=0.002,
        name="sliding mast is centered in the sleeve bore",
    )
    ctx.expect_overlap(
        pedestal,
        base,
        axes="z",
        elem_a="sliding_mast",
        elem_b="pedestal_sleeve",
        min_overlap=0.10,
        name="mast remains inserted at low height",
    )

    rest_height = ctx.part_world_position(pedestal)
    with ctx.pose({height_slide: 0.120}):
        raised_height = ctx.part_world_position(pedestal)
        ctx.expect_overlap(
            pedestal,
            base,
            axes="z",
            elem_a="sliding_mast",
            elem_b="pedestal_sleeve",
            min_overlap=0.035,
            name="mast remains retained at full height",
        )
    ctx.check(
        "pedestal slides upward",
        rest_height is not None
        and raised_height is not None
        and raised_height[2] > rest_height[2] + 0.11,
        details=f"rest={rest_height}, raised={raised_height}",
    )

    rest_aabb = ctx.part_world_aabb(platform)
    with ctx.pose({pitch_hinge: 0.70}):
        pitched_aabb = ctx.part_world_aabb(platform)
    ctx.check(
        "platform pitches upward",
        rest_aabb is not None
        and pitched_aabb is not None
        and pitched_aabb[1][2] > rest_aabb[1][2] + 0.10,
        details=f"rest_aabb={rest_aabb}, pitched_aabb={pitched_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
