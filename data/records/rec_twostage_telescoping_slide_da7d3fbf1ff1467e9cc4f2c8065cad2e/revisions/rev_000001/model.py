from __future__ import annotations

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
)


STEEL = "brushed_steel"
WALL_PAINT = "painted_wall_plate"
FASTENER = "dark_fastener"
RUNNER = "zinc_runner"
NYLON = "white_nylon_glide"


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_slide_extension")

    model.material(STEEL, rgba=(0.62, 0.64, 0.63, 1.0))
    model.material(WALL_PAINT, rgba=(0.72, 0.76, 0.78, 1.0))
    model.material(FASTENER, rgba=(0.06, 0.065, 0.07, 1.0))
    model.material(RUNNER, rgba=(0.82, 0.83, 0.78, 1.0))
    model.material(NYLON, rgba=(0.92, 0.90, 0.82, 1.0))

    side_plate = model.part("side_plate")
    side_plate.visual(
        Box((0.66, 0.075, 0.012)),
        origin=Origin(xyz=(0.0, -0.0175, 0.006)),
        material=WALL_PAINT,
        name="ground_foot",
    )
    side_plate.visual(
        Box((0.62, 0.012, 0.210)),
        origin=Origin(xyz=(0.0, -0.034, 0.110)),
        material=WALL_PAINT,
        name="wall_plate",
    )
    side_plate.visual(
        Box((0.500, 0.008, 0.070)),
        origin=Origin(xyz=(0.0, -0.024, 0.125)),
        material=STEEL,
        name="channel_back",
    )
    side_plate.visual(
        Box((0.500, 0.038, 0.009)),
        origin=Origin(xyz=(0.0, -0.005, 0.1535)),
        material=STEEL,
        name="top_flange",
    )
    side_plate.visual(
        Box((0.500, 0.038, 0.009)),
        origin=Origin(xyz=(0.0, -0.005, 0.0965)),
        material=STEEL,
        name="bottom_flange",
    )
    side_plate.visual(
        Box((0.014, 0.038, 0.054)),
        origin=Origin(xyz=(-0.257, -0.005, 0.125)),
        material=STEEL,
        name="rear_stop",
    )

    for i, x in enumerate((-0.225, -0.075, 0.075, 0.225)):
        side_plate.visual(
            Cylinder(radius=0.011, length=0.004),
            origin=Origin(xyz=(x, -0.026, 0.174), rpy=(-1.57079632679, 0.0, 0.0)),
            material=FASTENER,
            name=f"upper_screw_{i}",
        )
        side_plate.visual(
            Cylinder(radius=0.010, length=0.004),
            origin=Origin(xyz=(x, -0.026, 0.076), rpy=(-1.57079632679, 0.0, 0.0)),
            material=FASTENER,
            name=f"lower_screw_{i}",
        )

    inner_runner = model.part("inner_runner")
    inner_runner.visual(
        Box((0.550, 0.012, 0.028)),
        origin=Origin(xyz=(-0.190, 0.000, 0.125)),
        material=RUNNER,
        name="runner_body",
    )
    inner_runner.visual(
        Box((0.550, 0.024, 0.007)),
        origin=Origin(xyz=(-0.190, 0.000, 0.142)),
        material=RUNNER,
        name="runner_top_rib",
    )
    inner_runner.visual(
        Box((0.550, 0.024, 0.007)),
        origin=Origin(xyz=(-0.190, 0.000, 0.108)),
        material=RUNNER,
        name="runner_bottom_rib",
    )
    inner_runner.visual(
        Box((0.360, 0.016, 0.005)),
        origin=Origin(xyz=(-0.245, -0.001, 0.1465)),
        material=NYLON,
        name="top_glide",
    )
    inner_runner.visual(
        Box((0.360, 0.016, 0.005)),
        origin=Origin(xyz=(-0.245, -0.001, 0.1035)),
        material=NYLON,
        name="bottom_glide",
    )
    inner_runner.visual(
        Box((0.055, 0.018, 0.074)),
        origin=Origin(xyz=(0.1125, 0.003, 0.125)),
        material=RUNNER,
        name="front_tab",
    )
    inner_runner.visual(
        Cylinder(radius=0.008, length=0.003),
        origin=Origin(xyz=(0.118, 0.0135, 0.145), rpy=(-1.57079632679, 0.0, 0.0)),
        material=FASTENER,
        name="tab_screw_upper",
    )
    inner_runner.visual(
        Cylinder(radius=0.008, length=0.003),
        origin=Origin(xyz=(0.118, 0.0135, 0.105), rpy=(-1.57079632679, 0.0, 0.0)),
        material=FASTENER,
        name="tab_screw_lower",
    )

    model.articulation(
        "slide",
        ArticulationType.PRISMATIC,
        parent=side_plate,
        child=inner_runner,
        # The joint frame is the mouth of the fixed outer channel.
        origin=Origin(xyz=(0.240, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=0.0, upper=0.240),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    slide = object_model.get_articulation("slide")
    side_plate = object_model.get_part("side_plate")
    inner_runner = object_model.get_part("inner_runner")

    ctx.check(
        "single prismatic runner joint",
        len(object_model.articulations) == 1
        and slide.articulation_type == ArticulationType.PRISMATIC
        and slide.child == "inner_runner",
        details=f"joints={[joint.name for joint in object_model.articulations]}",
    )

    ctx.expect_gap(
        inner_runner,
        side_plate,
        axis="y",
        positive_elem="runner_body",
        negative_elem="channel_back",
        min_gap=0.006,
        max_gap=0.020,
        name="runner clears channel back",
    )
    ctx.expect_gap(
        side_plate,
        inner_runner,
        axis="z",
        positive_elem="top_flange",
        negative_elem="top_glide",
        max_gap=0.001,
        max_penetration=0.0,
        name="top glide is captured by upper flange",
    )
    ctx.expect_gap(
        inner_runner,
        side_plate,
        axis="z",
        positive_elem="bottom_glide",
        negative_elem="bottom_flange",
        max_gap=0.001,
        max_penetration=1e-6,
        name="bottom glide rides on lower flange",
    )
    ctx.expect_overlap(
        inner_runner,
        side_plate,
        axes="x",
        elem_a="runner_body",
        elem_b="channel_back",
        min_overlap=0.440,
        name="closed runner remains deeply inserted",
    )

    rest_position = ctx.part_world_position(inner_runner)
    with ctx.pose({slide: 0.240}):
        ctx.expect_overlap(
            inner_runner,
            side_plate,
            axes="x",
            elem_a="runner_body",
            elem_b="channel_back",
            min_overlap=0.200,
            name="extended runner is still retained",
        )
        ctx.expect_gap(
            side_plate,
            inner_runner,
            axis="z",
            positive_elem="top_flange",
            negative_elem="top_glide",
            max_gap=0.001,
            max_penetration=0.0,
            name="upper flange still captures extension",
        )
        extended_position = ctx.part_world_position(inner_runner)

    ctx.check(
        "runner extends outward along the slide",
        rest_position is not None
        and extended_position is not None
        and extended_position[0] > rest_position[0] + 0.20,
        details=f"rest={rest_position}, extended={extended_position}",
    )

    return ctx.report()


object_model = build_object_model()
