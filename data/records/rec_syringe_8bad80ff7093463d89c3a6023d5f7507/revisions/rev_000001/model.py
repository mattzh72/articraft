from __future__ import annotations

import math

import cadquery as cq

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
    mesh_from_cadquery,
)


def _annular_tube_x(x0: float, length: float, outer_radius: float, inner_radius: float) -> cq.Workplane:
    """Open cylindrical shell whose axis runs along +X."""
    return (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((x0, 0.0, 0.0))
    )


def _solid_tube_x(x0: float, length: float, radius: float) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length).translate((x0, 0.0, 0.0))


def _frustum_x(x0: float, length: float, radius_a: float, radius_b: float) -> cq.Workplane:
    """Solid tapered luer-nose frustum, wider at x0."""
    return (
        cq.Workplane("YZ")
        .circle(radius_a)
        .workplane(offset=length)
        .circle(radius_b)
        .loft()
        .translate((x0, 0.0, 0.0))
    )


def _clear_barrel_shape() -> cq.Workplane:
    """One connected clear syringe body: hollow barrel, collars, and luer nose."""
    barrel = _annular_tube_x(0.0, 0.092, 0.0080, 0.0066)
    rear_collar = _annular_tube_x(-0.006, 0.010, 0.0100, 0.0063)
    front_collar = _annular_tube_x(0.087, 0.012, 0.0087, 0.0030)
    luer_taper = _frustum_x(0.096, 0.013, 0.0058, 0.0032)
    luer_stem = _solid_tube_x(0.107, 0.010, 0.0032)
    return barrel.union(rear_collar).union(front_collar).union(luer_taper).union(luer_stem)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="detailed_clear_syringe")

    clear_polycarbonate = model.material(
        "clear_polycarbonate",
        rgba=(0.78, 0.94, 1.0, 0.34),
    )
    frosted_edge = model.material(
        "frosted_clear_edges",
        rgba=(0.86, 0.96, 1.0, 0.55),
    )
    ink = model.material("blue_black_print", rgba=(0.02, 0.04, 0.08, 1.0))
    plunger_plastic = model.material("plunger_white_plastic", rgba=(0.92, 0.94, 0.96, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    steel = model.material("stainless_steel", rgba=(0.80, 0.83, 0.84, 1.0))

    barrel = model.part("barrel")
    barrel.visual(
        mesh_from_cadquery(_clear_barrel_shape(), "clear_barrel", tolerance=0.00035),
        material=clear_polycarbonate,
        name="clear_barrel",
    )

    # Butterfly-style finger flanges molded around the rear collar.
    for sign, name in ((1.0, "finger_grip_0"), (-1.0, "finger_grip_1")):
        barrel.visual(
            Box((0.0060, 0.0290, 0.0105)),
            origin=Origin(xyz=(-0.0020, sign * 0.0215, 0.0)),
            material=frosted_edge,
            name=name,
        )
        barrel.visual(
            Cylinder(radius=0.00525, length=0.0060),
            origin=Origin(xyz=(-0.0020, sign * 0.0360, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=frosted_edge,
            name=f"{name}_round_end",
        )

    # Printed dose graduations: small tick marks plus long numbered-position ticks.
    for idx in range(11):
        x = 0.011 + idx * 0.0068
        major = idx % 2 == 0
        barrel.visual(
            Box((0.00075, 0.0090 if major else 0.0055, 0.00032)),
            origin=Origin(xyz=(x, 0.0, 0.00803)),
            material=ink,
            name=f"graduation_{idx}",
        )
    for idx, x in enumerate((0.0246, 0.0518, 0.0790)):
        barrel.visual(
            Box((0.0012, 0.0135, 0.00034)),
            origin=Origin(xyz=(x, -0.0005, -0.00803)),
            material=ink,
            name=f"index_mark_{idx}",
        )

    # Stainless needle fixed into the luer stem.
    barrel.visual(
        Cylinder(radius=0.00055, length=0.043),
        origin=Origin(xyz=(0.137, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="needle",
    )
    barrel.visual(
        Cylinder(radius=0.00105, length=0.006),
        origin=Origin(xyz=(0.116, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="needle_socket",
    )
    barrel.visual(
        mesh_from_cadquery(_frustum_x(0.158, 0.006, 0.00055, 0.00010), "beveled_needle_tip", tolerance=0.00012),
        material=steel,
        name="needle_tip",
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.00175, length=0.088),
        origin=Origin(xyz=(-0.0275, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=plunger_plastic,
        name="center_rod",
    )
    plunger.visual(
        Box((0.082, 0.0012, 0.0104)),
        origin=Origin(xyz=(-0.0290, 0.0, 0.0)),
        material=plunger_plastic,
        name="vertical_rib",
    )
    plunger.visual(
        Box((0.082, 0.0104, 0.0012)),
        origin=Origin(xyz=(-0.0290, 0.0, 0.0)),
        material=plunger_plastic,
        name="horizontal_rib",
    )
    plunger.visual(
        Cylinder(radius=0.0105, length=0.0055),
        origin=Origin(xyz=(-0.0750, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=plunger_plastic,
        name="thumb_pad",
    )
    plunger.visual(
        Cylinder(radius=0.0043, length=0.006),
        origin=Origin(xyz=(-0.0695, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=plunger_plastic,
        name="thumb_neck",
    )
    plunger.visual(
        Cylinder(radius=0.00605, length=0.0070),
        origin=Origin(xyz=(0.0180, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="rubber_piston",
    )
    plunger.visual(
        Cylinder(radius=0.00678, length=0.0014),
        origin=Origin(xyz=(0.0147, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="rear_seal_lip",
    )
    plunger.visual(
        Cylinder(radius=0.00678, length=0.0014),
        origin=Origin(xyz=(0.0213, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="front_seal_lip",
    )

    model.articulation(
        "barrel_to_plunger",
        ArticulationType.PRISMATIC,
        parent=barrel,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.18, lower=0.0, upper=0.055),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrel = object_model.get_part("barrel")
    plunger = object_model.get_part("plunger")
    slide = object_model.get_articulation("barrel_to_plunger")

    ctx.allow_overlap(
        barrel,
        plunger,
        elem_a="clear_barrel",
        elem_b="rear_seal_lip",
        reason="The rear rubber seal lip is intentionally modeled with slight compression against the clear barrel wall.",
    )
    ctx.allow_overlap(
        barrel,
        plunger,
        elem_a="clear_barrel",
        elem_b="front_seal_lip",
        reason="The front rubber seal lip is intentionally modeled with slight compression against the clear barrel wall.",
    )

    ctx.expect_within(
        plunger,
        barrel,
        axes="yz",
        inner_elem="rubber_piston",
        outer_elem="clear_barrel",
        margin=0.0001,
        name="rubber piston is centered inside clear barrel",
    )
    ctx.expect_overlap(
        plunger,
        barrel,
        axes="x",
        elem_a="rubber_piston",
        elem_b="clear_barrel",
        min_overlap=0.006,
        name="retracted piston remains inside the barrel",
    )
    ctx.expect_overlap(
        plunger,
        barrel,
        axes="x",
        elem_a="rear_seal_lip",
        elem_b="clear_barrel",
        min_overlap=0.001,
        name="rear seal lip is seated in the barrel",
    )
    ctx.expect_overlap(
        plunger,
        barrel,
        axes="x",
        elem_a="front_seal_lip",
        elem_b="clear_barrel",
        min_overlap=0.001,
        name="front seal lip is seated in the barrel",
    )

    rest_pos = ctx.part_world_position(plunger)
    with ctx.pose({slide: 0.055}):
        ctx.expect_within(
            plunger,
            barrel,
            axes="yz",
            inner_elem="rubber_piston",
            outer_elem="clear_barrel",
            margin=0.0001,
            name="pressed piston stays guided by the barrel",
        )
        ctx.expect_overlap(
            plunger,
            barrel,
            axes="x",
            elem_a="rubber_piston",
            elem_b="clear_barrel",
            min_overlap=0.006,
            name="pressed piston remains retained in the barrel",
        )
        ctx.expect_gap(
            barrel,
            plunger,
            axis="x",
            positive_elem="clear_barrel",
            negative_elem="thumb_pad",
            min_gap=0.006,
            name="thumb pad stops behind the finger flange",
        )
        pressed_pos = ctx.part_world_position(plunger)

    ctx.check(
        "plunger advances toward needle when pressed",
        rest_pos is not None and pressed_pos is not None and pressed_pos[0] > rest_pos[0] + 0.05,
        details=f"rest={rest_pos}, pressed={pressed_pos}",
    )

    return ctx.report()


object_model = build_object_model()
