from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


OUTER_LENGTH = 0.72
OUTER_WIDTH = 0.060
OUTER_HEIGHT = 0.042
OUTER_WALL = 0.003

MIDDLE_LENGTH = 0.64
MIDDLE_WIDTH = 0.048
MIDDLE_HEIGHT = 0.030
MIDDLE_WALL = 0.0025
MIDDLE_TRAVEL = 0.34

INNER_HOME = 0.04
INNER_LENGTH = 0.56
INNER_WIDTH = 0.038
INNER_HEIGHT = 0.021
INNER_WALL = 0.002
INNER_TRAVEL = 0.32

MIDDLE_PAD_THICKNESS = (OUTER_HEIGHT - 2.0 * OUTER_WALL - MIDDLE_HEIGHT) / 2.0
INNER_PAD_THICKNESS = (MIDDLE_HEIGHT - 2.0 * MIDDLE_WALL - INNER_HEIGHT) / 2.0


def _rectangular_tube(
    *,
    length: float,
    width: float,
    height: float,
    wall: float,
    bottom_slot_width: float,
) -> cq.Workplane:
    tube = (
        cq.Workplane("YZ")
        .rect(width, height)
        .rect(width - 2.0 * wall, height - 2.0 * wall)
        .extrude(length)
    )
    bottom_slot = (
        cq.Workplane("XY")
        .workplane(offset=-(height / 2.0 + 0.001))
        .center(length / 2.0, 0.0)
        .rect(length + 0.004, bottom_slot_width)
        .extrude(wall + 0.002)
    )
    return tube.cut(bottom_slot)


def _top_bottom_glides(
    *,
    start_x: float,
    length: float,
    width: float,
    section_height: float,
    thickness: float,
) -> cq.Workplane:
    top = (
        cq.Workplane("XY")
        .workplane(offset=section_height / 2.0)
        .center(start_x + length / 2.0, 0.0)
        .rect(length, width)
        .extrude(thickness)
    )
    bottom = (
        cq.Workplane("XY")
        .workplane(offset=-(section_height / 2.0 + thickness))
        .center(start_x + length / 2.0, 0.0)
        .rect(length, width)
        .extrude(thickness)
    )
    return top.union(bottom)


def _outer_section_shape() -> cq.Workplane:
    tube = _rectangular_tube(
        length=OUTER_LENGTH,
        width=OUTER_WIDTH,
        height=OUTER_HEIGHT,
        wall=OUTER_WALL,
        bottom_slot_width=0.016,
    )

    base_plate = (
        cq.Workplane("XY")
        .workplane(offset=-(OUTER_HEIGHT / 2.0 + 0.008))
        .center(0.18, 0.0)
        .rect(0.30, 0.082)
        .extrude(0.008)
    )

    return tube.union(base_plate)


def _middle_runner_shape() -> cq.Workplane:
    tube = _rectangular_tube(
        length=MIDDLE_LENGTH,
        width=MIDDLE_WIDTH,
        height=MIDDLE_HEIGHT,
        wall=MIDDLE_WALL,
        bottom_slot_width=0.012,
    )
    glides = _top_bottom_glides(
        start_x=0.06,
        length=0.48,
        width=0.030,
        section_height=MIDDLE_HEIGHT,
        thickness=MIDDLE_PAD_THICKNESS,
    )
    return tube.union(glides)


def _inner_runner_shape() -> cq.Workplane:
    tube = _rectangular_tube(
        length=INNER_LENGTH,
        width=INNER_WIDTH,
        height=INNER_HEIGHT,
        wall=INNER_WALL,
        bottom_slot_width=0.010,
    )
    glides = _top_bottom_glides(
        start_x=0.05,
        length=0.42,
        width=0.024,
        section_height=INNER_HEIGHT,
        thickness=INNER_PAD_THICKNESS,
    )
    return tube.union(glides)


def _add_mesh_visual(part, shape: cq.Workplane, mesh_name: str, material: str, visual_name: str) -> None:
    part.visual(
        mesh_from_cadquery(shape, mesh_name),
        material=material,
        name=visual_name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_stage_telescoping_slide")

    model.material("outer_steel", rgba=(0.23, 0.25, 0.28, 1.0))
    model.material("middle_steel", rgba=(0.55, 0.58, 0.61, 1.0))
    model.material("inner_steel", rgba=(0.77, 0.79, 0.82, 1.0))

    outer = model.part("outer_section")
    middle = model.part("middle_runner")
    inner = model.part("inner_runner")

    _add_mesh_visual(outer, _outer_section_shape(), "outer_section", "outer_steel", "outer_body")
    _add_mesh_visual(middle, _middle_runner_shape(), "middle_runner", "middle_steel", "middle_body")
    _add_mesh_visual(inner, _inner_runner_shape(), "inner_runner", "inner_steel", "inner_body")

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.50,
            lower=0.0,
            upper=MIDDLE_TRAVEL,
        ),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=Origin(xyz=(INNER_HOME, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.55,
            lower=0.0,
            upper=INNER_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    outer = object_model.get_part("outer_section")
    middle = object_model.get_part("middle_runner")
    inner = object_model.get_part("inner_runner")
    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_inner = object_model.get_articulation("middle_to_inner")

    ctx.allow_overlap(
        outer,
        middle,
        elem_a="outer_body",
        elem_b="middle_body",
        reason="The middle runner is intentionally represented as telescoping inside the outer box-section sleeve.",
    )
    ctx.allow_overlap(
        middle,
        inner,
        elem_a="middle_body",
        elem_b="inner_body",
        reason="The inner runner is intentionally represented as telescoping inside the middle box-section sleeve.",
    )
    ctx.expect_within(
        middle,
        outer,
        axes="yz",
        margin=0.001,
        name="middle runner stays centered inside the outer section",
    )
    ctx.expect_within(
        inner,
        middle,
        axes="yz",
        margin=0.001,
        name="inner runner stays centered inside the middle section",
    )
    ctx.expect_overlap(
        middle,
        outer,
        axes="x",
        min_overlap=0.60,
        name="middle runner remains inserted in the outer section at rest",
    )
    ctx.expect_overlap(
        inner,
        middle,
        axes="x",
        min_overlap=0.50,
        name="inner runner remains inserted in the middle section at rest",
    )

    middle_rest = ctx.part_world_position(middle)
    inner_rest = ctx.part_world_position(inner)
    with ctx.pose({outer_to_middle: MIDDLE_TRAVEL}):
        ctx.expect_within(
            middle,
            outer,
            axes="yz",
            margin=0.001,
            name="extended middle runner stays centered in the outer section",
        )
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            min_overlap=OUTER_LENGTH - MIDDLE_TRAVEL,
            name="extended middle runner still retains insertion in the outer section",
        )
        middle_extended = ctx.part_world_position(middle)

    with ctx.pose({outer_to_middle: MIDDLE_TRAVEL, middle_to_inner: INNER_TRAVEL}):
        ctx.expect_within(
            inner,
            middle,
            axes="yz",
            margin=0.001,
            name="extended inner runner stays centered in the middle section",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            min_overlap=0.24,
            name="extended inner runner still retains insertion in the middle section",
        )
        inner_extended = ctx.part_world_position(inner)

    ctx.check(
        "outer joint extends the middle runner along +X",
        middle_rest is not None
        and middle_extended is not None
        and middle_extended[0] > middle_rest[0] + 0.10,
        details=f"rest={middle_rest}, extended={middle_extended}",
    )
    ctx.check(
        "inner joint extends the inner runner along +X",
        inner_rest is not None
        and inner_extended is not None
        and inner_extended[0] > inner_rest[0] + 0.10,
        details=f"rest={inner_rest}, extended={inner_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
