import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

def add_hollow_tube(part, width: float, height: float, length: float, wall: float, prefix: str, color: tuple[float, float, float]):
    # Top wall
    part.visual(
        Box((width, wall, length)),
        origin=Origin(xyz=(0.0, height / 2 - wall / 2, length / 2)),
        name=f"{prefix}_top",
        color=color,
    )
    # Bottom wall
    part.visual(
        Box((width, wall, length)),
        origin=Origin(xyz=(0.0, -height / 2 + wall / 2, length / 2)),
        name=f"{prefix}_bottom",
        color=color,
    )
    # Left wall
    part.visual(
        Box((wall, height - 2 * wall, length)),
        origin=Origin(xyz=(-width / 2 + wall / 2, 0.0, length / 2)),
        name=f"{prefix}_left",
        color=color,
    )
    # Right wall
    part.visual(
        Box((wall, height - 2 * wall, length)),
        origin=Origin(xyz=(width / 2 - wall / 2, 0.0, length / 2)),
        name=f"{prefix}_right",
        color=color,
    )

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_slide")

    outer_w, outer_h = 0.040, 0.040
    mid_w, mid_h = 0.034, 0.034
    inner_w, inner_h = 0.028, 0.028
    length = 0.300
    wall = 0.002

    outer_section = model.part("outer_section")
    add_hollow_tube(outer_section, outer_w, outer_h, length, wall, "outer", color=(0.6, 0.6, 0.65))

    middle_runner = model.part("middle_runner")
    add_hollow_tube(middle_runner, mid_w, mid_h, length, wall, "mid", color=(0.7, 0.7, 0.75))

    inner_runner = model.part("inner_runner")
    add_hollow_tube(inner_runner, inner_w, inner_h, length, wall, "inner", color=(0.8, 0.8, 0.85))

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer_section,
        child=middle_runner,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=0.0, upper=0.25),
    )

    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle_runner,
        child=inner_runner,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=0.0, upper=0.25),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_section")
    mid = object_model.get_part("middle_runner")
    inner = object_model.get_part("inner_runner")

    ctx.allow_isolated_part(mid, reason="Telescoping runner proxy fit with clearance.")
    ctx.allow_isolated_part(inner, reason="Telescoping runner proxy fit with clearance.")

    ctx.expect_within(
        mid,
        outer,
        axes="xy",
        margin=0.002,
        name="middle runner is centered inside outer section"
    )
    ctx.expect_within(
        inner,
        mid,
        axes="xy",
        margin=0.002,
        name="inner runner is centered inside middle runner"
    )

    outer_to_mid = object_model.get_articulation("outer_to_middle")
    mid_to_inner = object_model.get_articulation("middle_to_inner")

    with ctx.pose({outer_to_mid: 0.25, mid_to_inner: 0.25}):
        ctx.expect_overlap(
            mid,
            outer,
            axes="z",
            min_overlap=0.04,
            name="middle runner remains inserted in outer section at full extension"
        )
        ctx.expect_overlap(
            inner,
            mid,
            axes="z",
            min_overlap=0.04,
            name="inner runner remains inserted in middle runner at full extension"
        )
        
        outer_pos = ctx.part_world_position(outer)
        mid_pos = ctx.part_world_position(mid)
        inner_pos = ctx.part_world_position(inner)
        if outer_pos and mid_pos and inner_pos:
            ctx.check(
                "middle extends from outer",
                mid_pos[2] > outer_pos[2] + 0.2,
            )
            ctx.check(
                "inner extends from middle",
                inner_pos[2] > mid_pos[2] + 0.2,
            )

    return ctx.report()

object_model = build_object_model()