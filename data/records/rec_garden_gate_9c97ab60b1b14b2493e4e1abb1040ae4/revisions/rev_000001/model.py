from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _arched_rail_geometry(
    *,
    x0: float,
    x1: float,
    bottom_at_ends: float,
    rise: float,
    rail_height: float,
    depth: float,
    samples: int = 28,
):
    """Return a cedar board mesh: a constant-depth arch band in the X/Z view."""

    def arch_bottom(x: float) -> float:
        half = (x1 - x0) * 0.5
        mid = (x0 + x1) * 0.5
        t = (x - mid) / half
        return bottom_at_ends + rise * (1.0 - t * t)

    xs = [x0 + (x1 - x0) * i / (samples - 1) for i in range(samples)]
    top = [(x, arch_bottom(x) + rail_height) for x in xs]
    bottom = [(x, arch_bottom(x)) for x in reversed(xs)]
    geom = ExtrudeGeometry(top + bottom, depth, cap=True, center=True)
    # ExtrudeGeometry extrudes along local Z.  Rotate so the arch profile's
    # second coordinate becomes vertical Z and the extrusion becomes gate depth.
    geom.rotate_x(math.pi / 2.0)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cedar_garden_gate")

    cedar = Material("warm_cedar", color=(0.58, 0.31, 0.14, 1.0))
    cedar_dark = Material("dark_cedar_grain", color=(0.31, 0.16, 0.07, 1.0))
    cedar_light = Material("fresh_cedar_edge", color=(0.72, 0.43, 0.21, 1.0))
    black_iron = Material("blackened_iron", color=(0.015, 0.014, 0.013, 1.0))
    stone = Material("weathered_stone", color=(0.34, 0.32, 0.28, 1.0))

    # Fixed square timber posts, tied together by a low stone/soil threshold so
    # the freestanding post pair is one supported root assembly.
    posts = model.part("posts")
    posts.visual(
        Box((0.16, 0.16, 1.55)),
        origin=Origin(xyz=(0.0, 0.0, 0.775)),
        material=cedar,
        name="hinge_post",
    )
    posts.visual(
        Box((0.16, 0.16, 1.55)),
        origin=Origin(xyz=(1.42, 0.0, 0.775)),
        material=cedar,
        name="latch_post",
    )
    posts.visual(
        Box((1.62, 0.28, 0.06)),
        origin=Origin(xyz=(0.71, 0.0, 0.03)),
        material=stone,
        name="threshold",
    )
    for x in (0.0, 1.42):
        for y in (-0.083, 0.083):
            posts.visual(
                Box((0.010, 0.006, 1.36)),
                origin=Origin(xyz=(x - 0.045, y, 0.80)),
                material=cedar_dark,
                name=f"post_grain_{x:.1f}_{y:.1f}",
            )
            posts.visual(
                Box((0.010, 0.006, 1.32)),
                origin=Origin(xyz=(x + 0.035, y, 0.76)),
                material=cedar_dark,
                name=f"post_grain_b_{x:.1f}_{y:.1f}",
            )

    # Fixed post-side hinge plates and pins.
    for idx, z in enumerate((0.48, 1.05)):
        posts.visual(
            Box((0.035, 0.090, 0.22)),
            origin=Origin(xyz=(0.056, -0.055, z)),
            material=black_iron,
            name=f"post_hinge_plate_{idx}",
        )
        posts.visual(
            Cylinder(radius=0.018, length=0.235),
            origin=Origin(xyz=(0.100, -0.070, z)),
            material=black_iron,
            name=f"hinge_pin_{idx}",
        )
        for dz in (-0.085, 0.085):
            posts.visual(
                Box((0.054, 0.046, 0.030)),
                origin=Origin(xyz=(0.079, -0.070, z + dz)),
                material=black_iron,
                name=f"pin_block_{idx}_{dz:+.2f}",
            )
        for dz in (-0.070, 0.070):
            posts.visual(
                Cylinder(radius=0.010, length=0.006),
                origin=Origin(
                    xyz=(0.060, -0.098, z + dz),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=black_iron,
                name=f"post_hinge_screw_{idx}_{dz:+.2f}",
            )

    posts.visual(
        Box((0.020, 0.055, 0.115)),
        origin=Origin(xyz=(1.333, -0.053, 0.800)),
        material=black_iron,
        name="latch_keeper",
    )
    posts.visual(
        Box((0.055, 0.012, 0.030)),
        origin=Origin(xyz=(1.325, -0.080, 0.800)),
        material=black_iron,
        name="keeper_lip",
    )

    # Gate leaf.  Its part frame is exactly on the vertical hinge axis; the
    # closed leaf extends in local +X between the posts.
    leaf = model.part("leaf")
    leaf_y = 0.070
    leaf_depth = 0.055
    leaf.visual(
        Box((0.10, leaf_depth, 1.08)),
        origin=Origin(xyz=(0.090, leaf_y, 0.66)),
        material=cedar,
        name="hinge_stile",
    )
    leaf.visual(
        Box((0.10, leaf_depth, 1.08)),
        origin=Origin(xyz=(1.170, leaf_y, 0.66)),
        material=cedar,
        name="free_stile",
    )
    leaf.visual(
        Box((1.14, leaf_depth, 0.105)),
        origin=Origin(xyz=(0.630, leaf_y, 0.215)),
        material=cedar,
        name="bottom_rail",
    )
    leaf.visual(
        Box((1.08, leaf_depth, 0.090)),
        origin=Origin(xyz=(0.630, leaf_y, 0.660)),
        material=cedar,
        name="middle_rail",
    )
    leaf.visual(
        mesh_from_geometry(
            _arched_rail_geometry(
                x0=0.075,
                x1=1.185,
                bottom_at_ends=1.080,
                rise=0.165,
                rail_height=0.105,
                depth=0.070,
            ),
            "curved_top_rail",
        ),
        origin=Origin(xyz=(0.0, leaf_y, 0.0)),
        material=cedar_light,
        name="curved_top_rail",
    )

    def rail_underside(x: float) -> float:
        mid = (0.075 + 1.185) * 0.5
        half = (1.185 - 0.075) * 0.5
        t = (x - mid) / half
        return 1.080 + 0.165 * (1.0 - t * t)

    for idx, x in enumerate((0.235, 0.365, 0.495, 0.625, 0.755, 0.885, 1.015)):
        top = rail_underside(x) + 0.025
        bottom = 0.150
        leaf.visual(
            Box((0.070, 0.046, top - bottom)),
            origin=Origin(xyz=(x, leaf_y + 0.002, (top + bottom) * 0.5)),
            material=cedar,
            name=f"picket_{idx}",
        )
        leaf.visual(
            Box((0.007, 0.004, top - bottom - 0.090)),
            origin=Origin(xyz=(x - 0.018, leaf_y - 0.027, (top + bottom) * 0.5)),
            material=cedar_dark,
            name=f"picket_grain_{idx}",
        )

    # Long black strap hinge leaves fastened to the moving gate.
    for idx, z in enumerate((0.48, 1.05)):
        leaf.visual(
            Box((0.56, 0.010, 0.045)),
            origin=Origin(xyz=(0.340, 0.040, z)),
            material=black_iron,
            name=f"strap_hinge_{idx}",
        )
        leaf.visual(
            Box((0.060, 0.012, 0.070)),
            origin=Origin(xyz=(0.080, 0.040, z)),
            material=black_iron,
            name=f"strap_root_{idx}",
        )
        leaf.visual(
            Cylinder(radius=0.024, length=0.110),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=black_iron,
            name=f"hinge_knuckle_{idx}",
        )
        leaf.visual(
            Box((0.085, 0.046, 0.050)),
            origin=Origin(xyz=(0.0625, 0.015, z)),
            material=black_iron,
            name=f"knuckle_lug_{idx}",
        )
        for sx in (0.145, 0.330, 0.530):
            leaf.visual(
                Cylinder(radius=0.010, length=0.006),
                origin=Origin(
                    xyz=(sx, 0.034, z),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=black_iron,
                name=f"strap_screw_{idx}_{sx:.2f}",
            )

    # Fixed escutcheon plate behind the thumb latch on the free stile.
    leaf.visual(
        Box((0.115, 0.010, 0.145)),
        origin=Origin(xyz=(1.130, 0.040, 0.800)),
        material=black_iron,
        name="latch_plate",
    )

    # Rotating thumb latch/handle, with its own small pivot on the free stile.
    latch = model.part("thumb_latch")
    latch.visual(
        Cylinder(radius=0.035, length=0.014),
        origin=Origin(xyz=(0.0, -0.012, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_iron,
        name="pivot_disc",
    )
    latch.visual(
        Cylinder(radius=0.012, length=0.020),
        origin=Origin(xyz=(0.0, -0.003, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_iron,
        name="pivot_stem",
    )
    latch.visual(
        Box((0.260, 0.012, 0.030)),
        origin=Origin(xyz=(-0.105, -0.019, 0.0)),
        material=black_iron,
        name="thumb_bar",
    )
    latch.visual(
        Box((0.070, 0.012, 0.020)),
        origin=Origin(xyz=(0.055, -0.019, 0.0)),
        material=black_iron,
        name="latch_nose",
    )

    model.articulation(
        "gate_hinge",
        ArticulationType.REVOLUTE,
        parent=posts,
        child=leaf,
        origin=Origin(xyz=(0.100, -0.070, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=0.0, upper=1.55),
    )
    model.articulation(
        "latch_pivot",
        ArticulationType.REVOLUTE,
        parent=leaf,
        child=latch,
        origin=Origin(xyz=(1.130, 0.035, 0.800)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=3.0, lower=-0.45, upper=0.45),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    posts = object_model.get_part("posts")
    leaf = object_model.get_part("leaf")
    latch = object_model.get_part("thumb_latch")
    gate_hinge = object_model.get_articulation("gate_hinge")
    latch_pivot = object_model.get_articulation("latch_pivot")

    for idx in (0, 1):
        ctx.allow_overlap(
            posts,
            leaf,
            elem_a=f"hinge_pin_{idx}",
            elem_b=f"hinge_knuckle_{idx}",
            reason="Each moving hinge knuckle is intentionally wrapped around the fixed hinge pin.",
        )
    ctx.allow_overlap(
        leaf,
        latch,
        elem_a="latch_plate",
        elem_b="pivot_stem",
        reason="The thumb-latch pivot stem intentionally passes through the escutcheon plate.",
    )

    with ctx.pose({gate_hinge: 0.0, latch_pivot: 0.0}):
        for idx in (0, 1):
            ctx.expect_within(
                posts,
                leaf,
                axes="xy",
                inner_elem=f"hinge_pin_{idx}",
                outer_elem=f"hinge_knuckle_{idx}",
                margin=0.002,
                name=f"hinge pin {idx} is captured by its moving knuckle",
            )
            ctx.expect_overlap(
                posts,
                leaf,
                axes="z",
                elem_a=f"hinge_pin_{idx}",
                elem_b=f"hinge_knuckle_{idx}",
                min_overlap=0.090,
                name=f"hinge knuckle {idx} overlaps the pin along its length",
            )
        ctx.expect_gap(
            leaf,
            posts,
            axis="x",
            positive_elem="hinge_stile",
            negative_elem="hinge_post",
            min_gap=0.035,
            max_gap=0.070,
            name="hinge stile clears the square post",
        )
        ctx.expect_gap(
            posts,
            leaf,
            axis="x",
            positive_elem="latch_post",
            negative_elem="free_stile",
            min_gap=0.010,
            max_gap=0.040,
            name="free stile closes against latch post with a small reveal",
        )
        ctx.expect_overlap(
            latch,
            leaf,
            axes="xz",
            elem_a="pivot_disc",
            elem_b="latch_plate",
            min_overlap=0.030,
            name="thumb latch pivot sits on escutcheon plate",
        )
        ctx.expect_overlap(
            latch,
            leaf,
            axes="y",
            elem_a="pivot_stem",
            elem_b="latch_plate",
            min_overlap=0.006,
            name="thumb latch pivot stem penetrates the plate",
        )
        closed_stile_aabb = ctx.part_element_world_aabb(leaf, elem="free_stile")
        closed_latch_aabb = ctx.part_element_world_aabb(latch, elem="thumb_bar")

    with ctx.pose({gate_hinge: 1.05}):
        open_stile_aabb = ctx.part_element_world_aabb(leaf, elem="free_stile")
    ctx.check(
        "leaf swings inward on a vertical post hinge",
        closed_stile_aabb is not None
        and open_stile_aabb is not None
        and open_stile_aabb[0][1] > closed_stile_aabb[0][1] + 0.45,
        details=f"closed={closed_stile_aabb}, open={open_stile_aabb}",
    )

    with ctx.pose({latch_pivot: 0.40}):
        lifted_latch_aabb = ctx.part_element_world_aabb(latch, elem="thumb_bar")
    ctx.check(
        "thumb latch rotates upward about its local pivot",
        closed_latch_aabb is not None
        and lifted_latch_aabb is not None
        and lifted_latch_aabb[1][2] > closed_latch_aabb[1][2] + 0.035,
        details=f"closed={closed_latch_aabb}, lifted={lifted_latch_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
