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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_service_telescoping_slide")

    zinc = model.material("satin_zinc", rgba=(0.62, 0.64, 0.62, 1.0))
    dark = model.material("dark_powder_coat", rgba=(0.05, 0.055, 0.06, 1.0))
    shadow = model.material("channel_shadow", rgba=(0.025, 0.027, 0.03, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    warning = model.material("orange_end_cap", rgba=(0.95, 0.34, 0.08, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.46, 0.16, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=dark,
        name="base_plate",
    )
    body.visual(
        Box((0.40, 0.096, 0.013)),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=shadow,
        name="channel_floor",
    )
    body.visual(
        Box((0.40, 0.022, 0.062)),
        origin=Origin(xyz=(0.0, 0.057, 0.052)),
        material=dark,
        name="side_wall_0",
    )
    body.visual(
        Box((0.40, 0.022, 0.062)),
        origin=Origin(xyz=(0.0, -0.057, 0.052)),
        material=dark,
        name="side_wall_1",
    )
    body.visual(
        Box((0.37, 0.026, 0.013)),
        origin=Origin(xyz=(0.0, 0.037, 0.079)),
        material=dark,
        name="retaining_lip_0",
    )
    body.visual(
        Box((0.37, 0.026, 0.013)),
        origin=Origin(xyz=(0.0, -0.037, 0.079)),
        material=dark,
        name="retaining_lip_1",
    )
    body.visual(
        Box((0.026, 0.104, 0.060)),
        origin=Origin(xyz=(-0.213, 0.0, 0.052)),
        material=dark,
        name="rear_stop",
    )
    for i, (x, y) in enumerate(
        ((-0.175, -0.061), (-0.175, 0.061), (0.175, -0.061), (0.175, 0.061))
    ):
        body.visual(
            Cylinder(radius=0.011, length=0.006),
            origin=Origin(xyz=(x, y, 0.024)),
            material=zinc,
            name=f"bolt_head_{i}",
        )
        body.visual(
            Cylinder(radius=0.015, length=0.006),
            origin=Origin(xyz=(x, y, 0.003)),
            material=rubber,
            name=f"foot_pad_{i}",
        )

    runner = model.part("runner")
    runner.visual(
        Box((0.420, 0.036, 0.028)),
        # Local -X is the length that remains hidden in the fixed channel.
        origin=Origin(xyz=(-0.110, 0.0, 0.048)),
        material=zinc,
        name="slider_bar",
    )
    runner.visual(
        Box((0.390, 0.024, 0.010)),
        origin=Origin(xyz=(-0.125, 0.0, 0.067)),
        material=zinc,
        name="top_rib",
    )
    runner.visual(
        Box((0.390, 0.008, 0.012)),
        origin=Origin(xyz=(-0.125, 0.022, 0.043)),
        material=zinc,
        name="side_bead_0",
    )
    runner.visual(
        Box((0.390, 0.008, 0.012)),
        origin=Origin(xyz=(-0.125, -0.022, 0.043)),
        material=zinc,
        name="side_bead_1",
    )
    runner.visual(
        Box((0.032, 0.108, 0.078)),
        origin=Origin(xyz=(0.107, 0.0, 0.051)),
        material=zinc,
        name="front_stop",
    )
    runner.visual(
        Box((0.020, 0.090, 0.046)),
        origin=Origin(xyz=(0.130, 0.0, 0.054)),
        material=warning,
        name="end_cap",
    )

    model.articulation(
        "body_to_runner",
        ArticulationType.PRISMATIC,
        parent=body,
        child=runner,
        # The joint frame is at the mouth of the fixed channel; positive travel
        # pulls the runner outward along the service-slide axis.
        origin=Origin(xyz=(0.200, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.18),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    runner = object_model.get_part("runner")
    slide = object_model.get_articulation("body_to_runner")

    ctx.check(
        "runner uses prismatic slide joint",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"joint_type={slide.articulation_type}",
    )
    ctx.check(
        "slide axis is outward x",
        tuple(round(v, 6) for v in slide.axis) == (1.0, 0.0, 0.0),
        details=f"axis={slide.axis}",
    )
    ctx.expect_contact(
        runner,
        body,
        elem_a="slider_bar",
        elem_b="channel_floor",
        name="slider rides on the channel floor",
    )
    ctx.expect_gap(
        body,
        runner,
        axis="y",
        min_gap=0.020,
        positive_elem="side_wall_0",
        negative_elem="side_bead_0",
        name="upper side bead clears side wall",
    )
    ctx.expect_gap(
        runner,
        body,
        axis="y",
        min_gap=0.020,
        positive_elem="side_bead_1",
        negative_elem="side_wall_1",
        name="lower side bead clears side wall",
    )
    ctx.expect_overlap(
        runner,
        body,
        axes="x",
        min_overlap=0.28,
        elem_a="slider_bar",
        elem_b="side_wall_0",
        name="collapsed runner has deep insertion",
    )

    rest_pos = ctx.part_world_position(runner)
    with ctx.pose({slide: 0.18}):
        ctx.expect_overlap(
            runner,
            body,
            axes="x",
            min_overlap=0.13,
            elem_a="slider_bar",
            elem_b="side_wall_0",
            name="extended runner still retained in channel",
        )
        extended_pos = ctx.part_world_position(runner)

    ctx.check(
        "upper limit extends runner outward",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.17,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
