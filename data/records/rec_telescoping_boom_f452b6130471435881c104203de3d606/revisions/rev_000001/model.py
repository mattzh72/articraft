from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


OUTER_LENGTH = 1.20
OUTER_WIDTH = 0.220
OUTER_HEIGHT = 0.160
OUTER_WALL = 0.018

MIDDLE_LENGTH = 1.05
MIDDLE_WIDTH = OUTER_WIDTH - 2.0 * OUTER_WALL
MIDDLE_HEIGHT = OUTER_HEIGHT - 2.0 * OUTER_WALL
MIDDLE_WALL = 0.014
MIDDLE_REST = 0.260
MIDDLE_TRAVEL = 0.500

INNER_LENGTH = 0.90
INNER_WIDTH = MIDDLE_WIDTH - 2.0 * MIDDLE_WALL
INNER_HEIGHT = MIDDLE_HEIGHT - 2.0 * MIDDLE_WALL
INNER_WALL = 0.010
INNER_REST = 0.260
INNER_TRAVEL = 0.420

TIP_LENGTH = 0.72
TIP_WIDTH = INNER_WIDTH - 2.0 * INNER_WALL
TIP_HEIGHT = INNER_HEIGHT - 2.0 * INNER_WALL
TIP_REST = 0.240
TIP_TRAVEL = 0.340


def _hollow_prism(
    length: float,
    outer_y: float,
    outer_z: float,
    inner_y: float,
    inner_z: float,
    *,
    x0: float = 0.0,
) -> cq.Workplane:
    """Rectangular tube solid extending from x0 to x0 + length."""
    outer = cq.Workplane("XY").box(length, outer_y, outer_z).translate(
        (x0 + length / 2.0, 0.0, 0.0)
    )
    cutter = cq.Workplane("XY").box(length + 0.04, inner_y, inner_z).translate(
        (x0 + length / 2.0, 0.0, 0.0)
    )
    return outer.cut(cutter)


def _telescoping_tube(
    length: float,
    width: float,
    height: float,
    wall: float,
    *,
    collar_length: float = 0.085,
    collar_growth: float = 0.026,
) -> cq.Workplane:
    """A hollow rectangular tube with a thicker front wear collar."""
    inner_y = width - 2.0 * wall
    inner_z = height - 2.0 * wall
    body = _hollow_prism(length, width, height, inner_y, inner_z)
    collar = _hollow_prism(
        collar_length,
        width + collar_growth,
        height + collar_growth,
        inner_y,
        inner_z,
        x0=length - collar_length,
    )
    shape = body.union(collar)
    try:
        shape = shape.edges("|X").fillet(0.003)
    except Exception:
        # The rectangular tube is still valid if a platform-specific fillet fails.
        pass
    return shape


def _tip_beam() -> cq.Workplane:
    beam = cq.Workplane("XY").box(TIP_LENGTH, TIP_WIDTH, TIP_HEIGHT).translate(
        (TIP_LENGTH / 2.0, 0.0, 0.0)
    )
    end_cap = cq.Workplane("XY").box(0.050, TIP_WIDTH + 0.018, TIP_HEIGHT + 0.018).translate(
        (TIP_LENGTH - 0.025, 0.0, 0.0)
    )
    shape = beam.union(end_cap)
    try:
        shape = shape.edges("|X").fillet(0.002)
    except Exception:
        pass
    return shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="underslung_telescoping_beam")

    support_blue = model.material("support_blue", rgba=(0.10, 0.18, 0.32, 1.0))
    dark_hardware = model.material("dark_hardware", rgba=(0.06, 0.07, 0.08, 1.0))
    outer_aluminum = model.material("outer_aluminum", rgba=(0.64, 0.67, 0.70, 1.0))
    middle_aluminum = model.material("middle_aluminum", rgba=(0.78, 0.80, 0.82, 1.0))
    inner_graphite = model.material("inner_graphite", rgba=(0.38, 0.40, 0.43, 1.0))
    safety_red = model.material("safety_red", rgba=(0.75, 0.05, 0.03, 1.0))

    top_support = model.part("top_support")
    top_support.visual(
        Box((1.38, 0.32, 0.030)),
        origin=Origin(xyz=(0.62, 0.0, 0.275)),
        material=support_blue,
        name="overhead_rail",
    )
    top_support.visual(
        Box((1.24, 0.075, 0.020)),
        origin=Origin(xyz=(0.62, -0.122, 0.250)),
        material=support_blue,
        name="edge_flange_0",
    )
    top_support.visual(
        Box((1.24, 0.075, 0.020)),
        origin=Origin(xyz=(0.62, 0.122, 0.250)),
        material=support_blue,
        name="edge_flange_1",
    )
    top_support.visual(
        Box((0.135, 0.275, 0.028)),
        origin=Origin(xyz=(0.32, 0.0, 0.094)),
        material=support_blue,
        name="saddle_top_0",
    )
    top_support.visual(
        Box((0.135, 0.275, 0.028)),
        origin=Origin(xyz=(0.32, 0.0, -0.125)),
        material=support_blue,
        name="saddle_bottom_0",
    )
    top_support.visual(
        Box((0.135, 0.025, 0.380)),
        origin=Origin(xyz=(0.32, -0.150, 0.065)),
        material=support_blue,
        name="hanger_plate_0_0",
    )
    top_support.visual(
        Box((0.135, 0.025, 0.380)),
        origin=Origin(xyz=(0.32, 0.150, 0.065)),
        material=support_blue,
        name="hanger_plate_0_1",
    )
    top_support.visual(
        Box((0.030, 0.030, 0.030)),
        origin=Origin(xyz=(0.282, -0.150, 0.145)),
        material=dark_hardware,
        name="bolt_0_0",
    )
    top_support.visual(
        Box((0.030, 0.030, 0.030)),
        origin=Origin(xyz=(0.358, 0.150, 0.145)),
        material=dark_hardware,
        name="bolt_0_1",
    )
    top_support.visual(
        Box((0.135, 0.275, 0.028)),
        origin=Origin(xyz=(0.88, 0.0, 0.094)),
        material=support_blue,
        name="saddle_top_1",
    )
    top_support.visual(
        Box((0.135, 0.275, 0.028)),
        origin=Origin(xyz=(0.88, 0.0, -0.125)),
        material=support_blue,
        name="saddle_bottom_1",
    )
    top_support.visual(
        Box((0.135, 0.025, 0.380)),
        origin=Origin(xyz=(0.88, -0.150, 0.065)),
        material=support_blue,
        name="hanger_plate_1_0",
    )
    top_support.visual(
        Box((0.135, 0.025, 0.380)),
        origin=Origin(xyz=(0.88, 0.150, 0.065)),
        material=support_blue,
        name="hanger_plate_1_1",
    )
    top_support.visual(
        Box((0.030, 0.030, 0.030)),
        origin=Origin(xyz=(0.842, -0.150, 0.145)),
        material=dark_hardware,
        name="bolt_1_0",
    )
    top_support.visual(
        Box((0.030, 0.030, 0.030)),
        origin=Origin(xyz=(0.918, 0.150, 0.145)),
        material=dark_hardware,
        name="bolt_1_1",
    )
    top_support.inertial = Inertial.from_geometry(
        Box((1.38, 0.34, 0.42)),
        mass=9.0,
        origin=Origin(xyz=(0.62, 0.0, 0.08)),
    )

    outer_section = model.part("outer_section")
    outer_section.visual(
        mesh_from_cadquery(
            _telescoping_tube(
                OUTER_LENGTH,
                OUTER_WIDTH,
                OUTER_HEIGHT,
                OUTER_WALL,
                collar_length=0.100,
                collar_growth=0.040,
            ),
            "outer_rectangular_section",
            tolerance=0.0007,
        ),
        material=outer_aluminum,
        name="outer_tube",
    )
    outer_section.visual(
        Box((0.030, 0.195, 0.012)),
        origin=Origin(xyz=(0.03, 0.0, 0.086)),
        material=dark_hardware,
        name="rear_stop_plate",
    )
    outer_section.inertial = Inertial.from_geometry(
        Box((OUTER_LENGTH, OUTER_WIDTH, OUTER_HEIGHT)),
        mass=5.5,
        origin=Origin(xyz=(OUTER_LENGTH / 2.0, 0.0, 0.0)),
    )

    middle_section = model.part("middle_section")
    middle_section.visual(
        mesh_from_cadquery(
            _telescoping_tube(
                MIDDLE_LENGTH,
                MIDDLE_WIDTH,
                MIDDLE_HEIGHT,
                MIDDLE_WALL,
                collar_length=0.080,
                collar_growth=0.024,
            ),
            "middle_rectangular_section",
            tolerance=0.0007,
        ),
        material=middle_aluminum,
        name="middle_tube",
    )
    middle_section.inertial = Inertial.from_geometry(
        Box((MIDDLE_LENGTH, MIDDLE_WIDTH, MIDDLE_HEIGHT)),
        mass=3.0,
        origin=Origin(xyz=(MIDDLE_LENGTH / 2.0, 0.0, 0.0)),
    )

    inner_section = model.part("inner_section")
    inner_section.visual(
        mesh_from_cadquery(
            _telescoping_tube(
                INNER_LENGTH,
                INNER_WIDTH,
                INNER_HEIGHT,
                INNER_WALL,
                collar_length=0.075,
                collar_growth=0.018,
            ),
            "inner_rectangular_section",
            tolerance=0.0007,
        ),
        material=inner_graphite,
        name="inner_tube",
    )
    inner_section.inertial = Inertial.from_geometry(
        Box((INNER_LENGTH, INNER_WIDTH, INNER_HEIGHT)),
        mass=1.8,
        origin=Origin(xyz=(INNER_LENGTH / 2.0, 0.0, 0.0)),
    )

    tip_section = model.part("tip_section")
    tip_section.visual(
        mesh_from_cadquery(_tip_beam(), "tip_rectangular_section", tolerance=0.0007),
        material=safety_red,
        name="tip_beam",
    )
    tip_section.visual(
        Box((0.075, 0.018, 0.018)),
        origin=Origin(xyz=(TIP_LENGTH - 0.025, 0.0, 0.0)),
        material=dark_hardware,
        name="pull_slot",
    )
    tip_section.inertial = Inertial.from_geometry(
        Box((TIP_LENGTH, TIP_WIDTH, TIP_HEIGHT)),
        mass=0.8,
        origin=Origin(xyz=(TIP_LENGTH / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "support_to_outer",
        ArticulationType.FIXED,
        parent=top_support,
        child=outer_section,
        origin=Origin(),
    )
    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer_section,
        child=middle_section,
        origin=Origin(xyz=(MIDDLE_REST, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.25, lower=0.0, upper=MIDDLE_TRAVEL),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle_section,
        child=inner_section,
        origin=Origin(xyz=(INNER_REST, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.28, lower=0.0, upper=INNER_TRAVEL),
    )
    model.articulation(
        "inner_to_tip",
        ArticulationType.PRISMATIC,
        parent=inner_section,
        child=tip_section,
        origin=Origin(xyz=(TIP_REST, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.30, lower=0.0, upper=TIP_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    top_support = object_model.get_part("top_support")
    outer = object_model.get_part("outer_section")
    middle = object_model.get_part("middle_section")
    inner = object_model.get_part("inner_section")
    tip = object_model.get_part("tip_section")
    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_inner = object_model.get_articulation("middle_to_inner")
    inner_to_tip = object_model.get_articulation("inner_to_tip")

    ctx.allow_overlap(
        outer,
        middle,
        elem_a="outer_tube",
        elem_b="middle_tube",
        reason=(
            "The middle rectangular section is intentionally nested inside the "
            "outer sleeve; the visible sleeve is hollow but the compiled proxy "
            "treats the retained insertion as interpenetration."
        ),
    )
    ctx.allow_overlap(
        middle,
        inner,
        elem_a="middle_tube",
        elem_b="inner_tube",
        reason=(
            "The inner rectangular section is intentionally retained inside the "
            "middle sleeve throughout its prismatic travel."
        ),
    )
    ctx.allow_overlap(
        inner,
        tip,
        elem_a="inner_tube",
        elem_b="tip_beam",
        reason=(
            "The solid red tip beam is intentionally captured inside the final "
            "hollow section so the last prismatic stage remains guided."
        ),
    )
    ctx.allow_overlap(
        outer,
        top_support,
        elem_a="outer_tube",
        elem_b="saddle_top_0",
        reason=(
            "The fixed top saddle locally clamps the upper face of the outer "
            "section; the small hidden embed represents a compressed clamp seat."
        ),
    )
    ctx.allow_overlap(
        outer,
        top_support,
        elem_a="outer_tube",
        elem_b="saddle_top_1",
        reason=(
            "The second fixed saddle uses the same local clamp seating to carry "
            "the outer telescoping sleeve from the overhead support."
        ),
    )

    ctx.expect_gap(
        top_support,
        outer,
        axis="z",
        positive_elem="overhead_rail",
        negative_elem="outer_tube",
        min_gap=0.10,
        name="overhead support sits above the under-slung beam",
    )
    ctx.expect_gap(
        top_support,
        outer,
        axis="z",
        positive_elem="saddle_top_0",
        negative_elem="outer_tube",
        max_penetration=0.022,
        name="first saddle bears on the outer section with local clamp seating",
    )
    ctx.expect_gap(
        top_support,
        outer,
        axis="z",
        positive_elem="saddle_top_1",
        negative_elem="outer_tube",
        max_penetration=0.022,
        name="second saddle bears on the outer section with local clamp seating",
    )
    ctx.expect_within(
        middle,
        outer,
        axes="yz",
        elem_a="middle_tube",
        elem_b="outer_tube",
        margin=0.0,
        name="middle section is centered inside outer bore at rest",
    )
    ctx.expect_within(
        inner,
        middle,
        axes="yz",
        elem_a="inner_tube",
        elem_b="middle_tube",
        margin=0.0,
        name="inner section is centered inside middle bore at rest",
    )
    ctx.expect_within(
        tip,
        inner,
        axes="yz",
        elem_a="tip_beam",
        elem_b="inner_tube",
        margin=0.0,
        name="tip section is centered inside inner bore at rest",
    )
    ctx.expect_overlap(
        middle,
        outer,
        axes="x",
        elem_a="middle_tube",
        elem_b="outer_tube",
        min_overlap=0.80,
        name="collapsed middle section remains deeply inserted",
    )
    ctx.expect_overlap(
        inner,
        middle,
        axes="x",
        elem_a="inner_tube",
        elem_b="middle_tube",
        min_overlap=0.70,
        name="collapsed inner section remains deeply inserted",
    )

    rest_tip = ctx.part_world_position(tip)
    with ctx.pose(
        {
            outer_to_middle: MIDDLE_TRAVEL,
            middle_to_inner: INNER_TRAVEL,
            inner_to_tip: TIP_TRAVEL,
        }
    ):
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            elem_a="middle_tube",
            elem_b="outer_tube",
            min_overlap=0.35,
            name="extended middle section retains insertion",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            elem_a="inner_tube",
            elem_b="middle_tube",
            min_overlap=0.30,
            name="extended inner section retains insertion",
        )
        ctx.expect_overlap(
            tip,
            inner,
            axes="x",
            elem_a="tip_beam",
            elem_b="inner_tube",
            min_overlap=0.25,
            name="extended tip section retains insertion",
        )
        extended_tip = ctx.part_world_position(tip)

    ctx.check(
        "serial prismatic stages extend along +X",
        rest_tip is not None
        and extended_tip is not None
        and extended_tip[0] > rest_tip[0] + MIDDLE_TRAVEL + INNER_TRAVEL + TIP_TRAVEL - 0.02,
        details=f"rest_tip={rest_tip}, extended_tip={extended_tip}",
    )

    return ctx.report()


object_model = build_object_model()
