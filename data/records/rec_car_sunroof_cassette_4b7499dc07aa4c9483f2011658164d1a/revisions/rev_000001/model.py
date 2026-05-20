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
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


ALUMINIUM = Material("satin_anodized_aluminium", rgba=(0.72, 0.74, 0.73, 1.0))
BLACK_RUBBER = Material("black_epdm_gasket", rgba=(0.006, 0.006, 0.005, 1.0))
SHADOW_PLASTIC = Material("matte_black_plastic", rgba=(0.015, 0.014, 0.013, 1.0))
GLASS = Material("smoked_tempered_glass", rgba=(0.10, 0.18, 0.23, 0.42))
FRIT = Material("black_ceramic_frit", rgba=(0.0, 0.0, 0.0, 0.88))
FABRIC = Material("woven_grey_sunshade_fabric", rgba=(0.30, 0.31, 0.30, 1.0))


def _rounded_glass_panel() -> object:
    """A shallow tempered glass slab with radiused plan-view corners."""

    return (
        cq.Workplane("XY")
        .box(0.740, 0.560, 0.008)
        .edges("|Z")
        .fillet(0.025)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="electric_tilt_moonroof_cassette")
    for material in (ALUMINIUM, BLACK_RUBBER, SHADOW_PLASTIC, GLASS, FRIT, FABRIC):
        model.materials.append(material)

    frame = model.part("cassette_frame")

    # Vehicle-intrinsic frame: X is left-right, Y is front-to-rear, Z is upward.
    # The aluminium cassette is a continuous perimeter with overlapping miter
    # regions at the corners, leaving a clear central roof aperture.
    frame.visual(
        Box((0.090, 0.780, 0.035)),
        origin=Origin(xyz=(-0.455, 0.000, 0.000)),
        material=ALUMINIUM,
        name="side_rail_0",
    )
    frame.visual(
        Box((0.090, 0.780, 0.035)),
        origin=Origin(xyz=(0.455, 0.000, 0.000)),
        material=ALUMINIUM,
        name="side_rail_1",
    )
    frame.visual(
        Box((1.000, 0.100, 0.035)),
        origin=Origin(xyz=(0.000, -0.340, 0.000)),
        material=ALUMINIUM,
        name="front_rail",
    )
    frame.visual(
        Box((1.000, 0.100, 0.035)),
        origin=Origin(xyz=(0.000, 0.340, 0.000)),
        material=ALUMINIUM,
        name="rear_rail",
    )

    # Raised EPDM seals around the aperture and lower interior shade tracks.
    frame.visual(
        Box((0.018, 0.610, 0.010)),
        origin=Origin(xyz=(-0.392, 0.000, 0.022)),
        material=BLACK_RUBBER,
        name="side_gasket_0",
    )
    frame.visual(
        Box((0.018, 0.610, 0.010)),
        origin=Origin(xyz=(0.392, 0.000, 0.022)),
        material=BLACK_RUBBER,
        name="side_gasket_1",
    )
    frame.visual(
        Box((0.740, 0.018, 0.008)),
        origin=Origin(xyz=(0.000, -0.282, 0.016)),
        material=BLACK_RUBBER,
        name="front_gasket",
    )
    frame.visual(
        Box((0.740, 0.018, 0.010)),
        origin=Origin(xyz=(0.000, 0.292, 0.022)),
        material=BLACK_RUBBER,
        name="rear_gasket",
    )
    for x, name in ((-0.403, "shade_track_0"), (0.403, "shade_track_1")):
        frame.visual(
            Box((0.024, 0.620, 0.020)),
            origin=Origin(xyz=(x, 0.000, -0.024)),
            material=SHADOW_PLASTIC,
            name=name,
        )

    # Front hinge cross-shaft, two fixed bearing ears, and an electric tilt
    # drive pack on one cassette side.
    frame.visual(
        Cylinder(radius=0.006, length=0.760),
        origin=Origin(xyz=(0.000, -0.300, 0.022), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=ALUMINIUM,
        name="front_hinge_pin",
    )
    for x, name in ((-0.285, "hinge_ear_0"), (0.285, "hinge_ear_1")):
        frame.visual(
            Box((0.035, 0.020, 0.050)),
            origin=Origin(xyz=(x, -0.309, 0.004)),
            material=ALUMINIUM,
            name=name,
        )
    frame.visual(
        Box((0.070, 0.125, 0.052)),
        origin=Origin(xyz=(0.545, 0.120, -0.020)),
        material=SHADOW_PLASTIC,
        name="drive_motor",
    )
    frame.visual(
        Box((0.065, 0.040, 0.024)),
        origin=Origin(xyz=(0.500, 0.120, -0.012)),
        material=ALUMINIUM,
        name="motor_bracket",
    )
    frame.visual(
        Cylinder(radius=0.006, length=0.430),
        origin=Origin(xyz=(0.414, 0.075, -0.030), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=SHADOW_PLASTIC,
        name="drive_cable_tube",
    )

    # Roller shade bearing brackets are fixed to the front rail; the roller
    # itself is articulated so its spring-tension winding can follow the shade.
    for x, name in ((-0.360, "roller_bracket_0"), (0.360, "roller_bracket_1")):
        frame.visual(
            Box((0.014, 0.040, 0.052)),
            origin=Origin(xyz=(x, -0.335, -0.040)),
            material=ALUMINIUM,
            name=name,
        )

    glass = model.part("glass_panel")
    glass.visual(
        mesh_from_cadquery(_rounded_glass_panel(), "rounded_tempered_glass", tolerance=0.0008),
        origin=Origin(xyz=(0.000, 0.300, 0.006)),
        material=GLASS,
        name="glass_panel",
    )
    # Opaque ceramic frit border bonded to the glass perimeter.
    glass.visual(
        Box((0.740, 0.035, 0.0015)),
        origin=Origin(xyz=(0.000, 0.036, 0.010)),
        material=FRIT,
        name="front_frit",
    )
    glass.visual(
        Box((0.740, 0.035, 0.0015)),
        origin=Origin(xyz=(0.000, 0.564, 0.010)),
        material=FRIT,
        name="rear_frit",
    )
    glass.visual(
        Box((0.035, 0.560, 0.0015)),
        origin=Origin(xyz=(-0.352, 0.300, 0.010)),
        material=FRIT,
        name="side_frit_0",
    )
    glass.visual(
        Box((0.035, 0.560, 0.0015)),
        origin=Origin(xyz=(0.352, 0.300, 0.010)),
        material=FRIT,
        name="side_frit_1",
    )
    for x, name in ((-0.225, "glass_hinge_lug_0"), (0.225, "glass_hinge_lug_1")):
        glass.visual(
            Box((0.095, 0.040, 0.012)),
            origin=Origin(xyz=(x, 0.047, -0.002)),
            material=SHADOW_PLASTIC,
            name=name,
        )
    for x, name in ((-0.225, "hinge_knuckle_0"), (0.225, "hinge_knuckle_1")):
        glass.visual(
            Cylinder(radius=0.008, length=0.085),
            origin=Origin(xyz=(x, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=SHADOW_PLASTIC,
            name=name,
        )
    for x, name in ((-0.225, "hinge_strap_0"), (0.225, "hinge_strap_1")):
        glass.visual(
            Box((0.085, 0.024, 0.006)),
            origin=Origin(xyz=(x, 0.014, 0.005)),
            material=SHADOW_PLASTIC,
            name=name,
        )
    for x, name in ((-0.305, "rear_lift_pad_0"), (0.305, "rear_lift_pad_1")):
        glass.visual(
            Box((0.080, 0.035, 0.010)),
            origin=Origin(xyz=(x, 0.555, -0.001)),
            material=SHADOW_PLASTIC,
            name=name,
        )

    shade_roller = model.part("shade_roller")
    shade_roller.visual(
        Cylinder(radius=0.025, length=0.670),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=FABRIC,
        name="fabric_roll",
    )
    shade_roller.visual(
        Cylinder(radius=0.028, length=0.030),
        origin=Origin(xyz=(-0.350, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=SHADOW_PLASTIC,
        name="spring_end_cap",
    )
    shade_roller.visual(
        Cylinder(radius=0.028, length=0.030),
        origin=Origin(xyz=(0.350, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=SHADOW_PLASTIC,
        name="idle_end_cap",
    )

    shade = model.part("sunshade")
    shade.visual(
        Box((0.660, 0.610, 0.003)),
        origin=Origin(xyz=(0.000, -0.305, 0.000)),
        material=FABRIC,
        name="shade_cloth",
    )
    shade.visual(
        Box((0.700, 0.024, 0.025)),
        origin=Origin(xyz=(0.000, 0.000, 0.000)),
        material=SHADOW_PLASTIC,
        name="pull_bar",
    )
    for x, name in ((-0.392, "guide_shoe_0"), (0.392, "guide_shoe_1")):
        shade.visual(
            Box((0.035, 0.045, 0.012)),
            origin=Origin(xyz=(x, -0.004, -0.002)),
            material=SHADOW_PLASTIC,
            name=name,
        )
    for x, name in ((-0.365, "guide_stem_0"), (0.365, "guide_stem_1")):
        shade.visual(
            Box((0.030, 0.030, 0.010)),
            origin=Origin(xyz=(x, -0.004, -0.001)),
            material=SHADOW_PLASTIC,
            name=name,
        )

    glass_hinge = model.articulation(
        "front_glass_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=glass,
        origin=Origin(xyz=(0.000, -0.300, 0.022)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.23),
        motion_properties=MotionProperties(damping=1.5, friction=0.08),
    )
    glass_hinge.meta["description"] = "Tilt-only moonroof panel: positive rotation lifts the rear edge."

    shade_slide = model.articulation(
        "shade_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=shade,
        origin=Origin(xyz=(0.000, 0.290, -0.032)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.45, lower=0.0, upper=0.520),
        motion_properties=MotionProperties(damping=0.7, friction=0.15),
    )
    shade_slide.meta["description"] = "Positive travel pulls the shade bar forward toward the spring roller."

    model.articulation(
        "roller_wind",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=shade_roller,
        origin=Origin(xyz=(0.000, -0.335, -0.060)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=8.0, lower=-15.0, upper=0.0),
        motion_properties=MotionProperties(damping=0.25, friction=0.04),
        meta={"description": "Spring-tension roller winds as the interior shade retracts."},
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("cassette_frame")
    glass = object_model.get_part("glass_panel")
    shade = object_model.get_part("sunshade")
    roller = object_model.get_part("shade_roller")
    glass_hinge = object_model.get_articulation("front_glass_hinge")
    shade_slide = object_model.get_articulation("shade_slide")

    for knuckle in ("hinge_knuckle_0", "hinge_knuckle_1"):
        ctx.allow_overlap(
            frame,
            glass,
            elem_a="front_hinge_pin",
            elem_b=knuckle,
            reason="The glass hinge knuckle is intentionally modeled as a bushing captured around the front hinge pin.",
        )
        ctx.expect_overlap(
            frame,
            glass,
            axes="xyz",
            elem_a="front_hinge_pin",
            elem_b=knuckle,
            min_overlap=0.010,
            name=f"{knuckle} is captured on the front hinge pin",
        )

    for track, shoe in (("shade_track_0", "guide_shoe_0"), ("shade_track_1", "guide_shoe_1")):
        ctx.allow_overlap(
            frame,
            shade,
            elem_a=track,
            elem_b=shoe,
            reason="The sunshade guide shoe is intentionally captured inside the side track channel proxy.",
        )
        ctx.expect_overlap(
            frame,
            shade,
            axes="xyz",
            elem_a=track,
            elem_b=shoe,
            min_overlap=0.004,
            name=f"{shoe} remains captured in {track}",
        )

    for bracket, cap in (("roller_bracket_0", "spring_end_cap"), ("roller_bracket_1", "idle_end_cap")):
        ctx.allow_overlap(
            frame,
            roller,
            elem_a=bracket,
            elem_b=cap,
            reason="The roller end cap is intentionally seated through the simplified bearing bracket.",
        )
        ctx.expect_overlap(
            frame,
            roller,
            axes="x",
            elem_a=bracket,
            elem_b=cap,
            min_overlap=0.005,
            name=f"{cap} is retained by {bracket}",
        )

    ctx.expect_within(
        glass,
        frame,
        axes="xy",
        inner_elem="glass_panel",
        margin=0.0,
        name="glass panel remains within cassette footprint",
    )
    ctx.expect_gap(
        glass,
        frame,
        axis="z",
        min_gap=0.001,
        max_gap=0.020,
        positive_elem="glass_panel",
        negative_elem="rear_rail",
        name="closed glass sits just above aluminium rail plane",
    )
    ctx.expect_overlap(
        shade,
        frame,
        axes="x",
        elem_a="shade_cloth",
        elem_b="front_rail",
        min_overlap=0.60,
        name="sunshade cloth spans the cassette opening width",
    )

    closed_aabb = ctx.part_element_world_aabb(glass, elem="glass_panel")
    with ctx.pose({glass_hinge: 0.23}):
        tilted_aabb = ctx.part_element_world_aabb(glass, elem="glass_panel")
    ctx.check(
        "rear glass edge rises in tilt pose",
        closed_aabb is not None
        and tilted_aabb is not None
        and tilted_aabb[1][2] > closed_aabb[1][2] + 0.08,
        details=f"closed_aabb={closed_aabb}, tilted_aabb={tilted_aabb}",
    )

    shade_start = ctx.part_world_position(shade)
    with ctx.pose({shade_slide: 0.52}):
        shade_retracted = ctx.part_world_position(shade)
    ctx.check(
        "shade bar retracts forward toward roller",
        shade_start is not None
        and shade_retracted is not None
        and shade_retracted[1] < shade_start[1] - 0.45,
        details=f"start={shade_start}, retracted={shade_retracted}",
    )

    return ctx.report()


object_model = build_object_model()
