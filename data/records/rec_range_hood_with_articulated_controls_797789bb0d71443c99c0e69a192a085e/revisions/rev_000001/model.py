from __future__ import annotations

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

# Advanced; only use CadQuery if the native sdk is not enough to represent the shapes you want:
# import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_cabinet_range_hood")

    painted_metal = model.material("painted_metal", rgba=(0.92, 0.93, 0.95, 1.0))
    stainless = model.material("stainless", rgba=(0.72, 0.74, 0.77, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.19, 0.21, 1.0))

    hood = model.part("hood")
    hood.visual(
        Box((0.60, 0.30, 0.038)),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=painted_metal,
        name="canopy",
    )
    hood.visual(
        Box((0.60, 0.028, 0.016)),
        origin=Origin(xyz=(0.0, 0.164, 0.008)),
        material=painted_metal,
        name="front_lip",
    )
    hood.visual(
        Box((0.46, 0.17, 0.004)),
        origin=Origin(xyz=(0.0, -0.03, -0.002)),
        material=charcoal,
        name="filter_panel",
    )
    for index, x_pos in enumerate((-0.215, 0.215)):
        hood.visual(
            Box((0.022, 0.16, 0.003)),
            origin=Origin(xyz=(x_pos, 0.10, -0.0015)),
            material=stainless,
            name=f"track_{index}",
        )

    visor = model.part("visor")
    visor.visual(
        Box((0.56, 0.09, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.01)),
        material=stainless,
        name="visor_panel",
    )
    for index, x_pos in enumerate((-0.215, 0.215)):
        visor.visual(
            Box((0.012, 0.16, 0.002)),
            origin=Origin(xyz=(x_pos, -0.035, -0.005)),
            material=stainless,
            name=f"runner_{index}",
        )
    visor.visual(
        Box((0.30, 0.012, 0.006)),
        origin=Origin(xyz=(0.0, 0.051, -0.013)),
        material=stainless,
        name="grab_lip",
    )

    model.articulation(
        "hood_to_visor",
        ArticulationType.PRISMATIC,
        parent=hood,
        child=visor,
        origin=Origin(xyz=(0.0, 0.131, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.25,
            lower=0.0,
            upper=0.09,
        ),
    )

    for index, x_pos in enumerate((-0.075, 0.075)):
        knob = model.part(f"knob_{index}")
        knob.visual(
            Cylinder(radius=0.0035, length=0.01),
            origin=Origin(xyz=(0.0, 0.0, -0.005)),
            material=charcoal,
            name="shaft",
        )
        knob.visual(
            Cylinder(radius=0.011, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, -0.016)),
            material=charcoal,
            name="dial",
        )
        model.articulation(
            f"hood_to_knob_{index}",
            ArticulationType.CONTINUOUS,
            parent=hood,
            child=knob,
            origin=Origin(xyz=(x_pos, 0.156, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=1.0, velocity=8.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    hood = object_model.get_part("hood")
    visor = object_model.get_part("visor")
    knob_0 = object_model.get_part("knob_0")
    knob_1 = object_model.get_part("knob_1")
    visor_slide = object_model.get_articulation("hood_to_visor")

    ctx.expect_gap(
        hood,
        visor,
        axis="z",
        positive_elem="track_0",
        negative_elem="runner_0",
        min_gap=0.0005,
        max_gap=0.0020,
        name="left runner clears its guide track",
    )
    ctx.expect_gap(
        hood,
        visor,
        axis="z",
        positive_elem="track_1",
        negative_elem="runner_1",
        min_gap=0.0005,
        max_gap=0.0020,
        name="right runner clears its guide track",
    )
    ctx.expect_overlap(
        visor,
        hood,
        axes="x",
        elem_a="visor_panel",
        elem_b="canopy",
        min_overlap=0.50,
        name="visor stays broad under the canopy",
    )
    ctx.expect_contact(
        hood,
        knob_0,
        elem_a="front_lip",
        elem_b="shaft",
        name="left knob shaft meets the front lip",
    )
    ctx.expect_contact(
        hood,
        knob_1,
        elem_a="front_lip",
        elem_b="shaft",
        name="right knob shaft meets the front lip",
    )
    ctx.expect_origin_distance(
        knob_0,
        knob_1,
        axes="x",
        min_dist=0.12,
        max_dist=0.18,
        name="knobs keep compact galley spacing",
    )

    rest_position = ctx.part_world_position(visor)
    slide_upper = 0.09
    if visor_slide.motion_limits is not None and visor_slide.motion_limits.upper is not None:
        slide_upper = visor_slide.motion_limits.upper

    with ctx.pose({visor_slide: slide_upper}):
        ctx.expect_overlap(
            visor,
            hood,
            axes="y",
            elem_a="runner_0",
            elem_b="track_0",
            min_overlap=0.05,
            name="left runner stays inserted at full extension",
        )
        ctx.expect_overlap(
            visor,
            hood,
            axes="y",
            elem_a="runner_1",
            elem_b="track_1",
            min_overlap=0.05,
            name="right runner stays inserted at full extension",
        )
        extended_position = ctx.part_world_position(visor)

    ctx.check(
        "visor extends forward",
        rest_position is not None
        and extended_position is not None
        and extended_position[1] > rest_position[1] + 0.07,
        details=f"rest={rest_position}, extended={extended_position}",
    )

    return ctx.report()


object_model = build_object_model()
