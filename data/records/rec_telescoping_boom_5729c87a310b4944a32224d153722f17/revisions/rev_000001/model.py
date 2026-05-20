from __future__ import annotations

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


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _rect_tube(
    length: float,
    outer_y: float,
    outer_z: float,
    inner_y: float,
    inner_z: float,
    *,
    center_x: float,
) -> cq.Workplane:
    """Rectangular hollow section running along local X."""
    outer = _box((length, outer_y, outer_z), (center_x, 0.0, 0.0))
    cutter = _box((length + 0.040, inner_y, inner_z), (center_x, 0.0, 0.0))
    return outer.cut(cutter)


def _front_ring(
    length: float,
    outer_y: float,
    outer_z: float,
    inner_y: float,
    inner_z: float,
    *,
    center_x: float,
) -> cq.Workplane:
    return _rect_tube(length, outer_y, outer_z, inner_y, inner_z, center_x=center_x)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_extension_boom")

    dark_paint = model.material("dark_powder_coat", color=(0.08, 0.09, 0.10, 1.0))
    graphite = model.material("graphite_stage", color=(0.20, 0.23, 0.26, 1.0))
    machined = model.material("machined_aluminum", color=(0.62, 0.66, 0.68, 1.0))
    bronze = model.material("bronze_slide_lands", color=(0.72, 0.53, 0.24, 1.0))
    pad_black = model.material("black_tool_pad", color=(0.03, 0.03, 0.035, 1.0))
    bolt_steel = model.material("dark_bolt_heads", color=(0.12, 0.12, 0.13, 1.0))

    lower = model.part("lower_sleeve")
    lower_body = _rect_tube(1.000, 0.240, 0.180, 0.190, 0.130, center_x=0.500)
    lower_body = lower_body.union(_box((0.032, 0.240, 0.180), (0.016, 0.0, 0.0)))
    lower_body = lower_body.union(_front_ring(0.055, 0.275, 0.215, 0.182, 0.122, center_x=0.972))
    lower_body = lower_body.union(_box((0.400, 0.315, 0.035), (0.235, 0.0, -0.107)))
    lower.visual(
        mesh_from_cadquery(lower_body, "lower_sleeve_body", tolerance=0.0008),
        material=dark_paint,
        name="lower_body",
    )
    for y, land_name in ((-0.0905, "lower_side_land_0"), (0.0905, "lower_side_land_1")):
        lower.visual(
            Box((0.430, 0.009, 0.026)),
            origin=Origin(xyz=(0.765, y, 0.0)),
            material=bronze,
            name=land_name,
        )
    for z in (-0.0615, 0.0615):
        lower.visual(
            Box((0.430, 0.056, 0.007)),
            origin=Origin(xyz=(0.765, 0.0, z)),
            material=bronze,
            name=f"lower_flat_land_{0 if z < 0 else 1}",
        )
    for y in (-0.105, 0.105):
        lower.visual(
            Box((0.032, 0.032, 0.010)),
            origin=Origin(xyz=(0.155, y, -0.084)),
            material=bolt_steel,
            name=f"root_bolt_{0 if y < 0 else 1}",
        )

    middle = model.part("middle_stage")
    middle_body = _rect_tube(1.250, 0.150, 0.095, 0.120, 0.065, center_x=-0.155)
    middle_body = middle_body.union(_box((0.030, 0.150, 0.095), (-0.765, 0.0, 0.0)))
    middle_body = middle_body.union(_front_ring(0.045, 0.172, 0.117, 0.108, 0.055, center_x=0.447))
    middle.visual(
        mesh_from_cadquery(middle_body, "middle_stage_body", tolerance=0.0008),
        material=graphite,
        name="middle_body",
    )
    middle_stop = _front_ring(0.055, 0.212, 0.150, 0.146, 0.091, center_x=0.050)
    middle.visual(
        mesh_from_cadquery(middle_stop, "middle_stop_collar", tolerance=0.0008),
        material=dark_paint,
        name="middle_stop",
    )
    for y, land_name in ((-0.0805, "middle_side_land_0"), (0.0805, "middle_side_land_1")):
        middle.visual(
            Box((0.720, 0.011, 0.022)),
            origin=Origin(xyz=(-0.385, y, 0.0)),
            material=machined,
            name=land_name,
        )
    for z in (-0.0505, 0.0505):
        middle.visual(
            Box((0.720, 0.046, 0.006)),
            origin=Origin(xyz=(-0.385, 0.0, z)),
            material=machined,
            name=f"middle_flat_land_{0 if z < 0 else 1}",
        )
    for y, land_name in ((-0.057, "middle_inner_land_0"), (0.057, "middle_inner_land_1")):
        middle.visual(
            Box((0.330, 0.006, 0.019)),
            origin=Origin(xyz=(0.265, y, 0.0)),
            material=bronze,
            name=land_name,
        )

    top = model.part("top_stage")
    top_body = _rect_tube(1.065, 0.075, 0.043, 0.061, 0.029, center_x=-0.1275)
    top_body = top_body.union(_box((0.018, 0.075, 0.043), (0.396, 0.0, 0.0)))
    top.visual(
        mesh_from_cadquery(top_body, "top_stage_body", tolerance=0.0008),
        material=machined,
        name="top_body",
    )
    top_stop = _front_ring(0.035, 0.130, 0.078, 0.071, 0.039, center_x=0.030)
    top.visual(
        mesh_from_cadquery(top_stop, "top_stop_collar", tolerance=0.0008),
        material=graphite,
        name="top_stop",
    )
    for y, land_name in ((-0.04575, "top_side_land_0"), (0.04575, "top_side_land_1")):
        top.visual(
            Box((0.605, 0.0165, 0.014)),
            origin=Origin(xyz=(-0.3575, y, 0.0)),
            material=bronze,
            name=land_name,
        )
    top.visual(
        Box((0.050, 0.180, 0.180)),
        origin=Origin(xyz=(0.430, 0.0, 0.0)),
        material=pad_black,
        name="tool_pad",
    )
    top.visual(
        Box((0.014, 0.092, 0.092)),
        origin=Origin(xyz=(0.462, 0.0, 0.0)),
        material=machined,
        name="pad_boss",
    )
    bolt_index = 0
    for y in (-0.058, 0.058):
        for z in (-0.058, 0.058):
            top.visual(
                Cylinder(radius=0.009, length=0.014),
                origin=Origin(xyz=(0.462, y, z), rpy=(0.0, 1.57079632679, 0.0)),
                material=bolt_steel,
                name=f"pad_bolt_{bolt_index}",
            )
            bolt_index += 1

    model.articulation(
        "lower_to_middle",
        ArticulationType.PRISMATIC,
        parent=lower,
        child=middle,
        origin=Origin(xyz=(1.000, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.35, lower=0.0, upper=0.45),
    )
    model.articulation(
        "middle_to_top",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=top,
        origin=Origin(xyz=(0.470, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=0.0, upper=0.40),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower = object_model.get_part("lower_sleeve")
    middle = object_model.get_part("middle_stage")
    top = object_model.get_part("top_stage")
    lower_to_middle = object_model.get_articulation("lower_to_middle")
    middle_to_top = object_model.get_articulation("middle_to_top")

    ctx.check(
        "serial prismatic boom stages",
        lower_to_middle.articulation_type == ArticulationType.PRISMATIC
        and middle_to_top.articulation_type == ArticulationType.PRISMATIC
        and lower_to_middle.parent == "lower_sleeve"
        and lower_to_middle.child == "middle_stage"
        and middle_to_top.parent == "middle_stage"
        and middle_to_top.child == "top_stage",
    )

    ctx.expect_within(
        middle,
        lower,
        axes="yz",
        inner_elem="middle_body",
        outer_elem="lower_body",
        margin=0.0,
        name="middle stage stays centered in lower sleeve guides",
    )
    ctx.expect_within(
        top,
        middle,
        axes="yz",
        inner_elem="top_body",
        outer_elem="middle_body",
        margin=0.0,
        name="top stage stays centered in middle sleeve guides",
    )
    ctx.expect_overlap(
        middle,
        lower,
        axes="x",
        elem_a="middle_body",
        elem_b="lower_body",
        min_overlap=0.70,
        name="collapsed middle stage has long lower-sleeve engagement",
    )
    ctx.expect_overlap(
        top,
        middle,
        axes="x",
        elem_a="top_body",
        elem_b="middle_body",
        min_overlap=0.60,
        name="collapsed top stage has long middle-stage engagement",
    )
    ctx.expect_gap(
        middle,
        lower,
        axis="x",
        positive_elem="middle_stop",
        negative_elem="lower_body",
        min_gap=0.010,
        max_gap=0.030,
        name="middle stop shoulder starts outside lower sleeve",
    )
    ctx.expect_gap(
        top,
        middle,
        axis="x",
        positive_elem="top_stop",
        negative_elem="middle_body",
        min_gap=0.008,
        max_gap=0.025,
        name="top stop shoulder starts outside middle stage",
    )
    ctx.expect_contact(
        middle,
        lower,
        elem_a="middle_side_land_0",
        elem_b="lower_side_land_0",
        name="middle stage bears on lower sleeve slide land",
    )
    ctx.expect_contact(
        top,
        middle,
        elem_a="top_side_land_0",
        elem_b="middle_inner_land_0",
        name="top stage bears on middle stage slide land",
    )

    rest_middle = ctx.part_world_position(middle)
    rest_top = ctx.part_world_position(top)
    with ctx.pose({lower_to_middle: 0.45, middle_to_top: 0.40}):
        ctx.expect_overlap(
            middle,
            lower,
            axes="x",
            elem_a="middle_body",
            elem_b="lower_body",
            min_overlap=0.30,
            name="extended middle stage remains retained in lower sleeve",
        )
        ctx.expect_overlap(
            top,
            middle,
            axes="x",
            elem_a="top_body",
            elem_b="middle_body",
            min_overlap=0.24,
            name="extended top stage remains retained in middle stage",
        )
        ctx.expect_within(
            middle,
            lower,
            axes="yz",
            inner_elem="middle_body",
            outer_elem="lower_body",
            margin=0.0,
            name="extended middle stage stays aligned in lower guides",
        )
        ctx.expect_within(
            top,
            middle,
            axes="yz",
            inner_elem="top_body",
            outer_elem="middle_body",
            margin=0.0,
            name="extended top stage stays aligned in middle guides",
        )
        ctx.expect_contact(
            middle,
            lower,
            elem_a="middle_side_land_0",
            elem_b="lower_side_land_0",
            name="extended middle stage remains supported by lower slide land",
        )
        ctx.expect_contact(
            top,
            middle,
            elem_a="top_side_land_0",
            elem_b="middle_inner_land_0",
            name="extended top stage remains supported by middle slide land",
        )
        extended_middle = ctx.part_world_position(middle)
        extended_top = ctx.part_world_position(top)

    ctx.check(
        "moving stages extend along common boom axis",
        rest_middle is not None
        and rest_top is not None
        and extended_middle is not None
        and extended_top is not None
        and extended_middle[0] > rest_middle[0] + 0.44
        and extended_top[0] > rest_top[0] + 0.84,
        details=f"middle {rest_middle}->{extended_middle}, top {rest_top}->{extended_top}",
    )

    return ctx.report()


object_model = build_object_model()
