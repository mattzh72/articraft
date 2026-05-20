from __future__ import annotations

import cadquery as cq
from math import pi

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


BODY_W = 0.34
BODY_D = 0.22
BODY_H = 0.14
WALL = 0.015
FLOOR = 0.014

HINGE_Y = BODY_D / 2.0 + 0.006
HINGE_Z = BODY_H + 0.006
SPINDLE_Z = 0.077


def _presentation_box_shell() -> cq.Workplane:
    """One-piece open presentation-box shell with a real hollow interior."""

    outer = cq.Workplane("XY").box(BODY_W, BODY_D, BODY_H).translate((0.0, 0.0, BODY_H / 2.0))
    inner_cut = (
        cq.Workplane("XY")
        .box(BODY_W - 2.0 * WALL, BODY_D - 2.0 * WALL, BODY_H)
        .translate((0.0, 0.0, FLOOR + BODY_H / 2.0))
    )
    shell = outer.cut(inner_cut)

    # Small outside rounds keep the box from reading as an unfinished block.
    try:
        shell = shell.edges("|Z").fillet(0.004)
    except Exception:
        pass
    return shell


def _cradle_ring() -> cq.Workplane:
    """Rotating winder cup/backing ring with a clearance bore for the spindle."""

    # The bore is slightly undersized versus the authored brass spindle.  That
    # creates a tiny hidden bearing-seat overlap so the supported cradle is not
    # visually floating while still reading as a spindle-mounted rotating cup.
    disk = cq.Workplane("XZ").circle(0.052).circle(0.0042).extrude(0.018, both=True)
    raised_hub = cq.Workplane("XZ").circle(0.017).circle(0.0042).extrude(0.030, both=True)
    retaining_rim = cq.Workplane("XZ").circle(0.056).circle(0.050).extrude(0.012, both=True)
    return disk.union(raised_hub).union(retaining_rim)


def _rounded_watch_cushion() -> cq.Workplane:
    pillow = cq.Workplane("XY").box(0.074, 0.026, 0.044)
    try:
        pillow = pillow.edges().fillet(0.006)
    except Exception:
        pass
    return pillow


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="watch_winder_presentation_box")

    walnut = model.material("dark_walnut_lacquer", rgba=(0.12, 0.055, 0.025, 1.0))
    black_velvet = model.material("black_velvet", rgba=(0.006, 0.007, 0.010, 1.0))
    brass = model.material("brushed_brass", rgba=(0.88, 0.64, 0.26, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.12, 0.17, 0.18, 0.35))
    rubber = model.material("matte_black_plastic", rgba=(0.015, 0.015, 0.018, 1.0))
    cream_suede = model.material("cream_suede_cushion", rgba=(0.76, 0.68, 0.55, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_presentation_box_shell(), "body_shell"),
        material=walnut,
        name="body_shell",
    )
    body.visual(
        Box((BODY_W - 2.0 * WALL - 0.006, BODY_D - 2.0 * WALL - 0.006, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, FLOOR + 0.0015)),
        material=black_velvet,
        name="velvet_liner",
    )
    body.visual(
        Box((0.040, 0.020, SPINDLE_Z - FLOOR + 0.010)),
        origin=Origin(xyz=(0.0, 0.062, FLOOR + (SPINDLE_Z - FLOOR + 0.010) / 2.0 - 0.004)),
        material=rubber,
        name="spindle_support",
    )
    body.visual(
        Cylinder(radius=0.005, length=0.052),
        origin=Origin(xyz=(0.0, 0.036, SPINDLE_Z), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="spindle_shaft",
    )
    body.visual(
        Cylinder(radius=0.006, length=BODY_W - 0.030),
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material=brass,
        name="rear_hinge_pin",
    )
    body.visual(
        Box((BODY_W - 0.070, 0.004, 0.018)),
        origin=Origin(xyz=(0.0, BODY_D / 2.0 + 0.001, BODY_H - 0.004)),
        material=brass,
        name="rear_hinge_leaf",
    )

    lid = model.part("lid")
    lid.visual(
        Box((BODY_W + 0.045, BODY_D + 0.025, 0.014)),
        # The child frame is on the hinge axis; the long thin panel reaches
        # forward from it and rests on the top rim at q=0.
        origin=Origin(xyz=(0.0, -0.1315, 0.001)),
        material=walnut,
        name="lid_panel",
    )
    lid.visual(
        Box((BODY_W - 0.080, BODY_D - 0.070, 0.002)),
        origin=Origin(xyz=(0.0, -0.132, 0.009)),
        material=smoked_glass,
        name="glass_inset",
    )
    lid.visual(
        Box((0.120, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, -0.244, 0.002)),
        material=brass,
        name="front_pull",
    )
    lid.visual(
        Box((BODY_W - 0.060, 0.006, 0.012)),
        origin=Origin(xyz=(0.0, -0.015, 0.000)),
        material=brass,
        name="hinge_leaf",
    )

    cradle = model.part("cradle")
    cradle.visual(
        mesh_from_cadquery(_cradle_ring(), "cradle_ring"),
        material=rubber,
        name="cradle_ring",
    )
    cradle.visual(
        mesh_from_cadquery(_rounded_watch_cushion(), "watch_cushion"),
        origin=Origin(xyz=(0.0, -0.030, 0.000)),
        material=cream_suede,
        name="watch_cushion",
    )
    cradle.visual(
        Box((0.010, 0.018, 0.056)),
        origin=Origin(xyz=(-0.046, -0.026, 0.000)),
        material=cream_suede,
        name="cushion_side_0",
    )
    cradle.visual(
        Box((0.010, 0.018, 0.056)),
        origin=Origin(xyz=(0.046, -0.026, 0.000)),
        material=cream_suede,
        name="cushion_side_1",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.65),
    )
    model.articulation(
        "body_to_cradle",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, SPINDLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    cradle = object_model.get_part("cradle")
    lid_hinge = object_model.get_articulation("body_to_lid")
    cradle_spin = object_model.get_articulation("body_to_cradle")

    ctx.allow_overlap(
        body,
        cradle,
        elem_a="spindle_shaft",
        elem_b="cradle_ring",
        reason="The small brass spindle is intentionally seated inside the cradle hub bearing.",
    )

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="body_shell",
        max_gap=0.001,
        max_penetration=0.0005,
        name="closed lid rests on top rim",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_panel",
        elem_b="body_shell",
        min_overlap=0.16,
        name="thin lid covers the presentation box",
    )
    ctx.expect_within(
        cradle,
        body,
        axes="xz",
        inner_elem="cradle_ring",
        outer_elem="body_shell",
        margin=0.002,
        name="cradle fits inside the box opening",
    )
    ctx.expect_within(
        body,
        cradle,
        axes="xz",
        inner_elem="spindle_shaft",
        outer_elem="cradle_ring",
        margin=0.001,
        name="spindle is centered in the cradle hub",
    )
    ctx.expect_overlap(
        body,
        cradle,
        axes="y",
        elem_a="spindle_shaft",
        elem_b="cradle_ring",
        min_overlap=0.003,
        name="spindle remains inserted in the cradle hub",
    )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({lid_hinge: 1.20}):
        opened_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    ctx.check(
        "rear hinge opens lid upward",
        closed_lid_aabb is not None
        and opened_lid_aabb is not None
        and opened_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.10,
        details=f"closed={closed_lid_aabb}, opened={opened_lid_aabb}",
    )

    with ctx.pose({cradle_spin: pi / 2.0}):
        ctx.expect_within(
            cradle,
            body,
            axes="xz",
            inner_elem="cradle_ring",
            outer_elem="body_shell",
            margin=0.002,
            name="spinning cradle remains within the box",
        )

    return ctx.report()


object_model = build_object_model()
