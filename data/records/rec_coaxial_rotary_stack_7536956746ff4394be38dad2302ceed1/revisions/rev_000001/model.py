from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _cylinder_z(radius: float, height: float, z_min: float) -> cq.Workplane:
    """CadQuery cylinder whose lower face is at z_min."""
    return cq.Workplane("XY").circle(radius).extrude(height).translate((0.0, 0.0, z_min))


def _radial_slot(radius: float, z_min: float, z_max: float, *, width: float = 0.028) -> cq.Workplane:
    """A shallow rectangular cutter that makes a visible index flat on a round stage."""
    height = z_max - z_min
    depth = 0.014
    return (
        cq.Workplane("XY")
        .box(depth, width, height)
        .translate((radius - depth / 2.0 + 0.001, 0.0, z_min + height / 2.0))
    )


def _base_cup_geometry() -> cq.Workplane:
    foot = _cylinder_z(0.225, 0.030, 0.000)
    cup_wall = _cylinder_z(0.185, 0.090, 0.030)
    raised_lip = _cylinder_z(0.195, 0.018, 0.102)
    body = foot.union(cup_wall).union(raised_lip)

    # Open recess of the cup; the remaining annular lip carries the first rotor.
    body = body.cut(_cylinder_z(0.132, 0.090, 0.040))

    # A central bearing boss rises from the bottom of the cup but stays below the lip.
    body = body.union(_cylinder_z(0.064, 0.066, 0.040))

    # Four through holes in the grounded flange make the part read as a bolted base.
    for angle in (0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0):
        x = 0.165 * math.cos(angle)
        y = 0.165 * math.sin(angle)
        hole = _cylinder_z(0.014, 0.050, -0.010).translate((x, y, 0.0))
        body = body.cut(hole)

    return body


def _stage_geometry(
    *,
    bottom_radius: float,
    body_radius: float,
    top_radius: float,
    bottom_h: float,
    body_h: float,
    top_h: float,
    marker_angle: float,
) -> cq.Workplane:
    total_h = bottom_h + body_h + top_h
    bottom = _cylinder_z(bottom_radius, bottom_h, 0.0)
    barrel = _cylinder_z(body_radius, body_h, bottom_h)
    top = _cylinder_z(top_radius, top_h, bottom_h + body_h)
    stage = bottom.union(barrel).union(top)

    # A subtle machined index flat makes rotation visible without breaking the
    # coaxial silhouette or the plain tooling face.
    slot = _radial_slot(max(bottom_radius, body_radius, top_radius), 0.006, total_h - 0.006)
    slot = slot.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), math.degrees(marker_angle))
    return stage.cut(slot)


def _top_stage_geometry() -> cq.Workplane:
    bottom = _cylinder_z(0.116, 0.012, 0.000)
    barrel = _cylinder_z(0.096, 0.046, 0.012)
    tooling_face = _cylinder_z(0.108, 0.012, 0.058)
    stage = bottom.union(barrel).union(tooling_face)
    slot = _radial_slot(0.116, 0.007, 0.056, width=0.024).rotate(
        (0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 115.0
    )
    return stage.cut(slot)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="collared_coaxial_rotary_head")

    graphite = model.material("dark_graphite", rgba=(0.08, 0.085, 0.09, 1.0))
    gunmetal = model.material("brushed_gunmetal", rgba=(0.34, 0.36, 0.37, 1.0))
    satin = model.material("satin_aluminum", rgba=(0.62, 0.64, 0.62, 1.0))
    pale_steel = model.material("pale_tool_steel", rgba=(0.77, 0.78, 0.76, 1.0))

    base_cup = model.part("base_cup")
    base_cup.visual(
        mesh_from_cadquery(_base_cup_geometry(), "base_cup_shell", tolerance=0.0007),
        material=graphite,
        name="base_cup_shell",
    )

    lower_stage = model.part("lower_stage")
    lower_stage.visual(
        mesh_from_cadquery(
            _stage_geometry(
                bottom_radius=0.155,
                body_radius=0.135,
                top_radius=0.145,
                bottom_h=0.014,
                body_h=0.060,
                top_h=0.014,
                marker_angle=0.0,
            ),
            "lower_stage_shell",
            tolerance=0.0007,
        ),
        material=gunmetal,
        name="lower_stage_shell",
    )

    middle_stage = model.part("middle_stage")
    middle_stage.visual(
        mesh_from_cadquery(
            _stage_geometry(
                bottom_radius=0.132,
                body_radius=0.112,
                top_radius=0.122,
                bottom_h=0.012,
                body_h=0.052,
                top_h=0.014,
                marker_angle=math.radians(42.0),
            ),
            "middle_stage_shell",
            tolerance=0.0007,
        ),
        material=satin,
        name="middle_stage_shell",
    )

    top_stage = model.part("top_stage")
    top_stage.visual(
        mesh_from_cadquery(_top_stage_geometry(), "top_stage_shell", tolerance=0.0007),
        material=pale_steel,
        name="top_stage_shell",
    )

    full_turn = MotionLimits(effort=18.0, velocity=2.5, lower=-math.pi, upper=math.pi)
    model.articulation(
        "base_to_lower",
        ArticulationType.REVOLUTE,
        parent=base_cup,
        child=lower_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=full_turn,
    )
    model.articulation(
        "lower_to_middle",
        ArticulationType.REVOLUTE,
        parent=lower_stage,
        child=middle_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.088)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=full_turn,
    )
    model.articulation(
        "middle_to_top",
        ArticulationType.REVOLUTE,
        parent=middle_stage,
        child=top_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=full_turn,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_cup")
    lower = object_model.get_part("lower_stage")
    middle = object_model.get_part("middle_stage")
    top = object_model.get_part("top_stage")
    base_to_lower = object_model.get_articulation("base_to_lower")
    lower_to_middle = object_model.get_articulation("lower_to_middle")
    middle_to_top = object_model.get_articulation("middle_to_top")

    joints = (base_to_lower, lower_to_middle, middle_to_top)
    ctx.check(
        "three stacked revolute joints",
        all(j.articulation_type == ArticulationType.REVOLUTE for j in joints),
        details=f"joint types={[j.articulation_type for j in joints]}",
    )
    ctx.check(
        "all rotary axes share the centerline",
        all(tuple(round(v, 6) for v in j.axis) == (0.0, 0.0, 1.0) for j in joints),
        details=f"axes={[j.axis for j in joints]}",
    )

    ctx.expect_origin_distance(lower, base, axes="xy", max_dist=0.001, name="lower stage is coaxial with base")
    ctx.expect_origin_distance(middle, base, axes="xy", max_dist=0.001, name="middle stage is coaxial with base")
    ctx.expect_origin_distance(top, base, axes="xy", max_dist=0.001, name="top stage is coaxial with base")

    ctx.expect_gap(
        lower,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="lower collar sits on the base cup lip",
    )
    ctx.expect_gap(
        middle,
        lower,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="middle collar sits on lower stage",
    )
    ctx.expect_gap(
        top,
        middle,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="top collar sits on middle stage",
    )

    with ctx.pose({base_to_lower: 0.9, lower_to_middle: -0.7, middle_to_top: 1.1}):
        ctx.expect_origin_distance(top, base, axes="xy", max_dist=0.001, name="posed stack remains coaxial")
        ctx.expect_gap(
            top,
            middle,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            name="posed top collar keeps axial bearing contact",
        )

    return ctx.report()


object_model = build_object_model()
