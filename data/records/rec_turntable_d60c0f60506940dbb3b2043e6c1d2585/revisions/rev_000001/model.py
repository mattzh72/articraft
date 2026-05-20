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


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """A softly radiused rectangular solid, centered on its local origin."""

    return cq.Workplane("XY").box(*size).edges("|Z").fillet(radius)


def _ring(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    """Annular bearing/collar geometry, authored from z=0 to z=height."""

    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
    )


def _soft_disc(radius: float, height: float, fillet: float) -> cq.Workplane:
    """A low cylinder with small rounded upper/lower edges."""

    return cq.Workplane("XY").circle(radius).extrude(height).edges().fillet(fillet)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_turntable")

    painted_metal = model.material("satin_graphite_paint", rgba=(0.09, 0.095, 0.10, 1.0))
    dark_polymer = model.material("warm_black_polymer", rgba=(0.015, 0.016, 0.018, 1.0))
    anodized_aluminum = model.material("brushed_aluminum", rgba=(0.55, 0.56, 0.54, 1.0))
    black_rubber = model.material("matte_black_elastomer", rgba=(0.005, 0.005, 0.004, 1.0))
    soft_gray_rubber = model.material("dark_gray_elastomer", rgba=(0.045, 0.045, 0.043, 1.0))
    polished_steel = model.material("polished_steel", rgba=(0.82, 0.82, 0.78, 1.0))
    cartridge_black = model.material("cartridge_black", rgba=(0.02, 0.018, 0.016, 1.0))
    stylus_red = model.material("muted_red_stylus", rgba=(0.45, 0.035, 0.025, 1.0))

    deck_top_z = 0.076
    platter_pivot = (-0.075, 0.010, 0.089)
    tonearm_pivot = (0.168, 0.108, 0.096)

    plinth = model.part("plinth")
    plinth.visual(
        mesh_from_cadquery(_rounded_box((0.49, 0.39, 0.070), 0.014), "plinth_body"),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=dark_polymer,
        name="plinth_body",
    )
    plinth.visual(
        mesh_from_cadquery(_rounded_box((0.438, 0.338, 0.006), 0.010), "top_plate"),
        origin=Origin(xyz=(0.0, 0.0, 0.073)),
        material=painted_metal,
        name="top_plate",
    )
    plinth.visual(
        mesh_from_cadquery(_rounded_box((0.455, 0.355, 0.003), 0.012), "shadow_reveal"),
        origin=Origin(xyz=(0.0, 0.0, 0.0705)),
        material=dark_polymer,
        name="shadow_reveal",
    )
    for idx, (x, y) in enumerate(
        ((-0.185, -0.140), (0.185, -0.140), (-0.185, 0.140), (0.185, 0.140))
    ):
        plinth.visual(
            Cylinder(radius=0.030, length=0.020),
            origin=Origin(xyz=(x, y, -0.006)),
            material=soft_gray_rubber,
            name=f"foot_{idx}",
        )

    # Stationary, hollow bearing collars give each rotating stage a visible,
    # mechanically honest support without relying on a solid sleeve proxy.
    plinth.visual(
        mesh_from_cadquery(_ring(0.039, 0.013, 0.013), "platter_bearing_collar"),
        origin=Origin(xyz=(platter_pivot[0], platter_pivot[1], deck_top_z)),
        material=polished_steel,
        name="platter_bearing_collar",
    )
    plinth.visual(
        mesh_from_cadquery(_ring(0.026, 0.010, 0.020), "tonearm_bearing_collar"),
        origin=Origin(xyz=(tonearm_pivot[0], tonearm_pivot[1], deck_top_z)),
        material=polished_steel,
        name="tonearm_bearing_collar",
    )
    plinth.visual(
        Box((0.066, 0.018, 0.008)),
        origin=Origin(xyz=(0.095, 0.142, deck_top_z + 0.004)),
        material=dark_polymer,
        name="arm_rest_base",
    )
    plinth.visual(
        Cylinder(radius=0.007, length=0.038),
        origin=Origin(xyz=(0.095, 0.142, deck_top_z + 0.027)),
        material=polished_steel,
        name="arm_rest_post",
    )
    plinth.visual(
        Box((0.038, 0.010, 0.006)),
        origin=Origin(xyz=(0.095, 0.142, deck_top_z + 0.049)),
        material=black_rubber,
        name="arm_rest_cradle",
    )
    plinth.visual(
        Cylinder(radius=0.006, length=0.002),
        origin=Origin(xyz=(0.180, -0.140, deck_top_z + 0.001)),
        material=polished_steel,
        name="status_lens",
    )

    platter = model.part("platter")
    platter.visual(
        mesh_from_cadquery(_soft_disc(0.148, 0.024, 0.0022), "platter_disc"),
        origin=Origin(),
        material=anodized_aluminum,
        name="platter_disc",
    )
    platter.visual(
        Cylinder(radius=0.126, length=0.0032),
        origin=Origin(xyz=(0.0, 0.0, 0.0256)),
        material=black_rubber,
        name="rubber_mat",
    )
    platter.visual(
        Cylinder(radius=0.0060, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        material=polished_steel,
        name="spindle",
    )
    platter.visual(
        Box((0.018, 0.006, 0.0018)),
        origin=Origin(xyz=(0.128, 0.0, 0.0280)),
        material=polished_steel,
        name="rim_index",
    )

    tonearm = model.part("tonearm")
    tonearm.visual(
        Cylinder(radius=0.020, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=polished_steel,
        name="pivot_cap",
    )
    tonearm.visual(
        Cylinder(radius=0.012, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=anodized_aluminum,
        name="gimbal_tower",
    )
    tonearm.visual(
        Cylinder(radius=0.0048, length=0.220),
        origin=Origin(xyz=(-0.110, 0.0, 0.039), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=anodized_aluminum,
        name="arm_tube",
    )
    tonearm.visual(
        Cylinder(radius=0.018, length=0.050),
        origin=Origin(xyz=(0.023, 0.0, 0.039), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_rubber,
        name="counterweight",
    )
    tonearm.visual(
        Box((0.034, 0.016, 0.004)),
        origin=Origin(xyz=(-0.226, 0.0, 0.036)),
        material=anodized_aluminum,
        name="headshell",
    )
    tonearm.visual(
        Box((0.018, 0.012, 0.012)),
        origin=Origin(xyz=(-0.237, 0.0, 0.0285)),
        material=cartridge_black,
        name="cartridge",
    )
    tonearm.visual(
        Cylinder(radius=0.0015, length=0.006),
        origin=Origin(xyz=(-0.244, 0.0, 0.025)),
        material=stylus_red,
        name="stylus",
    )

    model.articulation(
        "platter_spin",
        ArticulationType.CONTINUOUS,
        parent=plinth,
        child=platter,
        origin=Origin(xyz=platter_pivot),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=33.0),
    )
    model.articulation(
        "tonearm_swing",
        ArticulationType.REVOLUTE,
        parent=plinth,
        child=tonearm,
        origin=Origin(xyz=tonearm_pivot),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=1.2, lower=-0.48, upper=0.36),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    plinth = object_model.get_part("plinth")
    platter = object_model.get_part("platter")
    tonearm = object_model.get_part("tonearm")
    platter_spin = object_model.get_articulation("platter_spin")
    tonearm_swing = object_model.get_articulation("tonearm_swing")

    ctx.expect_contact(
        platter,
        plinth,
        elem_a="platter_disc",
        elem_b="platter_bearing_collar",
        contact_tol=0.0015,
        name="platter sits on its bearing collar",
    )
    ctx.expect_contact(
        tonearm,
        plinth,
        elem_a="pivot_cap",
        elem_b="tonearm_bearing_collar",
        contact_tol=0.0015,
        name="tonearm pivot cap sits on its bearing collar",
    )
    ctx.expect_gap(
        tonearm,
        platter,
        axis="z",
        positive_elem="stylus",
        negative_elem="rubber_mat",
        min_gap=0.001,
        max_gap=0.012,
        name="stylus hovers just above the elastomer mat at rest",
    )

    index_rest = ctx.part_element_world_aabb(platter, elem="rim_index")
    with ctx.pose({platter_spin: math.pi / 2.0}):
        index_turned = ctx.part_element_world_aabb(platter, elem="rim_index")
    ctx.check(
        "platter articulation rotates the rim index",
        index_rest is not None
        and index_turned is not None
        and index_turned[0][1] > index_rest[0][1] + 0.08,
        details=f"rest={index_rest}, turned={index_turned}",
    )

    head_rest = ctx.part_element_world_aabb(tonearm, elem="headshell")
    with ctx.pose({tonearm_swing: 0.30}):
        head_swung = ctx.part_element_world_aabb(tonearm, elem="headshell")
    ctx.check(
        "tonearm stage pivots around its vertical bearing",
        head_rest is not None
        and head_swung is not None
        and head_swung[0][1] < head_rest[0][1] - 0.035,
        details=f"rest={head_rest}, swung={head_swung}",
    )

    return ctx.report()


object_model = build_object_model()
