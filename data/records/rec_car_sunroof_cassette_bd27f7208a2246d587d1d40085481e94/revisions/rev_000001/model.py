from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilt_sunroof_cassette")

    satin_black = model.material("satin_black", rgba=(0.015, 0.016, 0.017, 1.0))
    rubber = model.material("rubber_black", rgba=(0.002, 0.002, 0.002, 1.0))
    dark_glass = model.material("smoked_glass", rgba=(0.05, 0.10, 0.13, 0.42))
    frit = model.material("black_frit", rgba=(0.0, 0.0, 0.0, 0.85))
    metal = model.material("dark_zinc", rgba=(0.28, 0.27, 0.25, 1.0))

    cassette = model.part("cassette")
    # A low-profile automotive cassette: a continuous rectangular frame with a
    # hollow center, side guide troughs, seals, and hinge bosses at the front.
    cassette.visual(
        Box((1.10, 0.080, 0.040)),
        origin=Origin(xyz=(0.0, 0.320, 0.020)),
        material=satin_black,
        name="side_rail_0",
    )
    cassette.visual(
        Box((1.10, 0.080, 0.040)),
        origin=Origin(xyz=(0.0, -0.320, 0.020)),
        material=satin_black,
        name="side_rail_1",
    )
    cassette.visual(
        Box((0.115, 0.720, 0.045)),
        origin=Origin(xyz=(-0.493, 0.0, 0.0225)),
        material=satin_black,
        name="front_crossmember",
    )
    cassette.visual(
        Box((0.115, 0.720, 0.035)),
        origin=Origin(xyz=(0.493, 0.0, 0.0175)),
        material=satin_black,
        name="rear_crossmember",
    )
    cassette.visual(
        Box((0.860, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.282, 0.047)),
        material=rubber,
        name="side_seal_0",
    )
    cassette.visual(
        Box((0.860, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, -0.282, 0.047)),
        material=rubber,
        name="side_seal_1",
    )
    cassette.visual(
        Box((0.018, 0.560, 0.010)),
        origin=Origin(xyz=(-0.431, 0.0, 0.047)),
        material=rubber,
        name="front_seal",
    )
    cassette.visual(
        Box((0.018, 0.560, 0.010)),
        origin=Origin(xyz=(0.431, 0.0, 0.047)),
        material=rubber,
        name="rear_seal",
    )
    cassette.visual(
        Box((0.880, 0.018, 0.006)),
        origin=Origin(xyz=(0.0, 0.326, 0.046)),
        material=metal,
        name="guide_track_0",
    )
    cassette.visual(
        Box((0.880, 0.018, 0.006)),
        origin=Origin(xyz=(0.0, -0.326, 0.046)),
        material=metal,
        name="guide_track_1",
    )
    for idx, y in enumerate((0.302, -0.302)):
        cassette.visual(
            Box((0.050, 0.034, 0.028)),
            origin=Origin(xyz=(-0.428, y, 0.059)),
            material=metal,
            name=f"glass_hinge_boss_{idx}",
        )
        cassette.visual(
            Cylinder(radius=0.007, length=0.050),
            origin=Origin(xyz=(-0.425, y, 0.052), rpy=(pi / 2.0, 0.0, 0.0)),
            material=metal,
            name=f"glass_hinge_pin_{idx}",
        )
    for idx, y in enumerate((0.315, -0.315)):
        cassette.visual(
            Box((0.040, 0.026, 0.026)),
            origin=Origin(xyz=(-0.476, y, 0.058)),
            material=metal,
            name=f"deflector_hinge_boss_{idx}",
        )

    glass = model.part("glass")
    # The glass part frame is on the front hinge line; the rectangular pane
    # extends rearward along local +X.
    glass.visual(
        Box((0.840, 0.520, 0.012)),
        origin=Origin(xyz=(0.420, 0.0, 0.006)),
        material=dark_glass,
        name="pane",
    )
    glass.visual(
        Box((0.800, 0.024, 0.002)),
        origin=Origin(xyz=(0.430, 0.248, 0.013)),
        material=frit,
        name="side_frit_0",
    )
    glass.visual(
        Box((0.800, 0.024, 0.002)),
        origin=Origin(xyz=(0.430, -0.248, 0.013)),
        material=frit,
        name="side_frit_1",
    )
    glass.visual(
        Box((0.035, 0.500, 0.002)),
        origin=Origin(xyz=(0.020, 0.0, 0.013)),
        material=frit,
        name="front_frit",
    )
    glass.visual(
        Box((0.050, 0.500, 0.002)),
        origin=Origin(xyz=(0.815, 0.0, 0.013)),
        material=frit,
        name="rear_frit",
    )
    for idx, y in enumerate((0.255, -0.255)):
        glass.visual(
            Box((0.060, 0.024, 0.008)),
            origin=Origin(xyz=(0.035, y, -0.004)),
            material=metal,
            name=f"hinge_tab_{idx}",
        )
    glass.visual(
        Box((0.050, 0.380, 0.012)),
        origin=Origin(xyz=(0.790, 0.0, -0.006)),
        material=metal,
        name="rear_lift_bracket",
    )

    deflector = model.part("deflector")
    # A narrow front wind strip, hinged at its leading edge and coupled to the
    # glass tilt so that it springs upward ahead of the opening.
    deflector.visual(
        Box((0.120, 0.555, 0.010)),
        origin=Origin(xyz=(0.060, 0.0, 0.005)),
        material=rubber,
        name="blade",
    )
    deflector.visual(
        Cylinder(radius=0.009, length=0.555),
        origin=Origin(xyz=(0.113, 0.0, 0.013), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="rounded_lip",
    )
    for idx, y in enumerate((0.260, -0.260)):
        deflector.visual(
            Box((0.050, 0.026, 0.008)),
            origin=Origin(xyz=(0.020, y, -0.004)),
            material=metal,
            name=f"hinge_leaf_{idx}",
        )

    model.articulation(
        "glass_hinge",
        ArticulationType.REVOLUTE,
        parent=cassette,
        child=glass,
        origin=Origin(xyz=(-0.425, 0.0, 0.052)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.8, lower=0.0, upper=0.18),
    )
    model.articulation(
        "deflector_hinge",
        ArticulationType.REVOLUTE,
        parent=cassette,
        child=deflector,
        origin=Origin(xyz=(-0.476, 0.0, 0.066)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=0.0, upper=0.35),
        mimic=Mimic(joint="glass_hinge", multiplier=1.8),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cassette = object_model.get_part("cassette")
    glass = object_model.get_part("glass")
    deflector = object_model.get_part("deflector")
    glass_hinge = object_model.get_articulation("glass_hinge")

    ctx.expect_within(
        glass,
        cassette,
        axes="xy",
        inner_elem="pane",
        margin=0.005,
        name="glass panel fits within cassette footprint",
    )
    ctx.expect_gap(
        glass,
        cassette,
        axis="z",
        positive_elem="pane",
        negative_elem="side_seal_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="closed glass seats on the side seal",
    )
    ctx.expect_overlap(
        deflector,
        cassette,
        axes="y",
        elem_a="blade",
        elem_b="front_crossmember",
        min_overlap=0.45,
        name="deflector spans the front crossmember",
    )

    rest_glass = ctx.part_element_world_aabb(glass, elem="pane")
    rest_deflector = ctx.part_element_world_aabb(deflector, elem="blade")
    with ctx.pose({glass_hinge: 0.18}):
        raised_glass = ctx.part_element_world_aabb(glass, elem="pane")
        raised_deflector = ctx.part_element_world_aabb(deflector, elem="blade")
        ctx.expect_gap(
            glass,
            cassette,
            axis="z",
            positive_elem="pane",
            negative_elem="rear_seal",
            max_penetration=0.0,
            name="tilted glass clears the rear cassette seal",
        )

    ctx.check(
        "rear edge tilts upward",
        rest_glass is not None
        and raised_glass is not None
        and raised_glass[1][2] > rest_glass[1][2] + 0.08,
        details=f"rest={rest_glass}, raised={raised_glass}",
    )
    ctx.check(
        "wind deflector lifts with glass",
        rest_deflector is not None
        and raised_deflector is not None
        and raised_deflector[1][2] > rest_deflector[1][2] + 0.025,
        details=f"rest={rest_deflector}, raised={raised_deflector}",
    )

    return ctx.report()


object_model = build_object_model()
