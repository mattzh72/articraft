from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _box(size: tuple[float, float, float], xyz: tuple[float, float, float]) -> cq.Workplane:
    """CadQuery box centered at an explicit local coordinate."""

    return cq.Workplane("XY").box(*size).translate(xyz)


def _rect_tube(length: float, width: float, height: float, wall: float) -> cq.Workplane:
    """A rectangular tube open along local X, with real wall thickness."""

    outer = cq.Workplane("XY").box(length, width, height)
    # Cut a through opening slightly longer than the outside rail so the end faces
    # remain visibly open rather than capped by a placeholder solid.
    inner = cq.Workplane("XY").box(length + 0.020, width - 2.0 * wall, height - 2.0 * wall)
    return outer.cut(inner)


def _outer_rail_cq() -> cq.Workplane:
    rail = _rect_tube(0.800, 0.080, 0.060, 0.006)

    # A machined fixed-end mounting foot below the rail.  It is local to the
    # outer stage and clears the nested rail cavity.
    rail = rail.union(_box((0.220, 0.160, 0.014), (-0.300, 0.0, -0.0365)))
    rail = rail.union(_box((0.060, 0.090, 0.020), (-0.392, 0.0, -0.0195)))

    # End-stop bosses on the top wall bracket the travel of the moving members.
    rail = rail.union(_box((0.035, 0.030, 0.010), (0.355, 0.0, 0.035)))
    rail = rail.union(_box((0.030, 0.026, 0.008), (-0.365, 0.0, 0.034)))
    return rail


def _middle_rail_cq() -> cq.Workplane:
    rail = _rect_tube(0.680, 0.054, 0.034, 0.0045)

    # Low-profile stop shoes fit inside the outer rail clearance and become
    # visible when the stage is extended.
    rail = rail.union(_box((0.030, 0.016, 0.0020), (0.295, 0.0, 0.0176)))
    rail = rail.union(_box((0.030, 0.016, 0.0020), (-0.305, 0.0, 0.0176)))
    rail = rail.union(_box((0.030, 0.016, 0.0020), (0.295, 0.0, -0.0176)))
    rail = rail.union(_box((0.030, 0.016, 0.0020), (-0.305, 0.0, -0.0176)))
    return rail


def _inner_rail_cq() -> cq.Workplane:
    rail = _rect_tube(0.560, 0.034, 0.014, 0.003)

    # Small travel-stop buttons retained inside the middle-stage clearance.
    rail = rail.union(_box((0.024, 0.010, 0.0018), (0.220, 0.0, 0.0075)))
    rail = rail.union(_box((0.024, 0.010, 0.0018), (-0.245, 0.0, 0.0075)))
    rail = rail.union(_box((0.024, 0.010, 0.0018), (0.220, 0.0, -0.0075)))
    rail = rail.union(_box((0.024, 0.010, 0.0018), (-0.245, 0.0, -0.0075)))
    return rail


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_stage_telescoping_slide")

    satin_steel = model.material("satin_steel", rgba=(0.58, 0.60, 0.60, 1.0))
    dark_stop = model.material("dark_hardened_stops", rgba=(0.05, 0.055, 0.060, 1.0))
    black_bolt = model.material("black_socket_bolts", rgba=(0.01, 0.010, 0.012, 1.0))
    bronze = model.material("oilite_bronze_bearings", rgba=(0.78, 0.55, 0.24, 1.0))

    outer = model.part("outer_rail")
    outer.visual(
        mesh_from_cadquery(_outer_rail_cq(), "outer_rail_body", tolerance=0.0006),
        origin=Origin(),
        material=satin_steel,
        name="outer_rail_body",
    )
    for idx, (x, y) in enumerate(
        [(-0.350, -0.050), (-0.350, 0.050), (-0.245, -0.050), (-0.245, 0.050)]
    ):
        outer.visual(
            Cylinder(radius=0.010, length=0.004),
            origin=Origin(xyz=(x, y, -0.0275)),
            material=black_bolt,
            name=f"mount_bolt_{idx}",
        )
    outer.visual(
        Box((0.052, 0.038, 0.012)),
        origin=Origin(xyz=(0.355, 0.0, 0.036)),
        material=dark_stop,
        name="front_stop_block",
    )

    middle = model.part("middle_rail")
    middle.visual(
        mesh_from_cadquery(_middle_rail_cq(), "middle_rail_body", tolerance=0.0006),
        origin=Origin(),
        material=satin_steel,
        name="middle_rail_body",
    )
    for idx, y in enumerate((-0.03035, 0.03035)):
        middle.visual(
            Box((0.620, 0.0073, 0.014)),
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=bronze,
            name=f"outer_bearing_pad_{idx}",
        )

    inner = model.part("inner_rail")
    inner.visual(
        mesh_from_cadquery(_inner_rail_cq(), "inner_rail_body", tolerance=0.0006),
        origin=Origin(),
        material=satin_steel,
        name="inner_rail_body",
    )
    inner.visual(
        Box((0.220, 0.032, 0.012)),
        origin=Origin(xyz=(0.380, 0.0, 0.0)),
        material=satin_steel,
        name="end_tongue",
    )
    inner.visual(
        Box((0.024, 0.094, 0.062)),
        origin=Origin(xyz=(0.492, 0.0, 0.0)),
        material=satin_steel,
        name="front_bracket",
    )
    for idx, y in enumerate((-0.01960, 0.01960)):
        inner.visual(
            Box((0.500, 0.0058, 0.007)),
            origin=Origin(xyz=(-0.010, y, 0.0)),
            material=bronze,
            name=f"middle_bearing_pad_{idx}",
        )
    for idx, (y, z) in enumerate([(-0.026, 0.016), (0.026, 0.016), (-0.026, -0.016), (0.026, -0.016)]):
        inner.visual(
            Cylinder(radius=0.007, length=0.004),
            origin=Origin(xyz=(0.505, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black_bolt,
            name=f"end_bolt_{idx}",
        )

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.35, lower=0.0, upper=0.280),
    )

    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.260),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_rail")
    middle = object_model.get_part("middle_rail")
    inner = object_model.get_part("inner_rail")
    outer_slide = object_model.get_articulation("outer_to_middle")
    inner_slide = object_model.get_articulation("middle_to_inner")

    for pad in ("outer_bearing_pad_0", "outer_bearing_pad_1"):
        ctx.allow_overlap(
            middle,
            outer,
            elem_a=pad,
            elem_b="outer_rail_body",
            reason=(
                "The bronze bearing shoe is intentionally modeled as a very "
                "slightly compressed sliding contact inside the outer box rail."
            ),
        )
        ctx.expect_contact(
            middle,
            outer,
            elem_a=pad,
            elem_b="outer_rail_body",
            name=f"{pad} carries the middle stage against the outer rail",
        )

    for pad in ("middle_bearing_pad_0", "middle_bearing_pad_1"):
        ctx.allow_overlap(
            inner,
            middle,
            elem_a=pad,
            elem_b="middle_rail_body",
            reason=(
                "The inner-stage bearing shoe is intentionally modeled as a "
                "lightly compressed sliding contact inside the middle box rail."
            ),
        )
        ctx.expect_contact(
            inner,
            middle,
            elem_a=pad,
            elem_b="middle_rail_body",
            name=f"{pad} carries the inner stage against the middle rail",
        )

    ctx.expect_within(
        middle,
        outer,
        axes="yz",
        inner_elem="middle_rail_body",
        outer_elem="outer_rail_body",
        margin=0.001,
        name="middle rail is nested inside outer rail cross-section",
    )
    ctx.expect_within(
        inner,
        middle,
        axes="yz",
        inner_elem="inner_rail_body",
        outer_elem="middle_rail_body",
        margin=0.001,
        name="inner rail is nested inside middle rail cross-section",
    )
    ctx.expect_overlap(
        middle,
        outer,
        axes="x",
        elem_a="middle_rail_body",
        elem_b="outer_rail_body",
        min_overlap=0.50,
        name="middle rail retains insertion when collapsed",
    )
    ctx.expect_overlap(
        inner,
        middle,
        axes="x",
        elem_a="inner_rail_body",
        elem_b="middle_rail_body",
        min_overlap=0.45,
        name="inner rail retains insertion when collapsed",
    )

    rest_inner = ctx.part_world_position(inner)
    with ctx.pose({outer_slide: 0.280, inner_slide: 0.260}):
        ctx.expect_within(
            middle,
            outer,
            axes="yz",
            inner_elem="middle_rail_body",
            outer_elem="outer_rail_body",
            margin=0.001,
            name="extended middle rail stays guided in outer rail",
        )
        ctx.expect_within(
            inner,
            middle,
            axes="yz",
            inner_elem="inner_rail_body",
            outer_elem="middle_rail_body",
            margin=0.001,
            name="extended inner rail stays guided in middle rail",
        )
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            elem_a="middle_rail_body",
            elem_b="outer_rail_body",
            min_overlap=0.30,
            name="extended middle rail remains captured",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            elem_a="inner_rail_body",
            elem_b="middle_rail_body",
            min_overlap=0.18,
            name="extended inner rail remains captured",
        )
        extended_inner = ctx.part_world_position(inner)

    ctx.check(
        "serial extension moves the load bracket outward",
        rest_inner is not None
        and extended_inner is not None
        and extended_inner[0] > rest_inner[0] + 0.500,
        details=f"rest={rest_inner}, extended={extended_inner}",
    )

    return ctx.report()


object_model = build_object_model()
