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


def _annular_cylinder(
    outer_radius: float,
    inner_radius: float,
    height: float,
    *,
    z_min: float = 0.0,
    fillet: float = 0.0,
) -> cq.Workplane:
    """A watertight hollow cylinder in local Z, authored in meters."""
    body = cq.Workplane("XY").circle(outer_radius).extrude(height)
    cutter = (
        cq.Workplane("XY")
        .circle(inner_radius)
        .extrude(height + 0.006)
        .translate((0.0, 0.0, -0.003))
    )
    body = body.cut(cutter)
    if fillet > 0.0:
        body = body.edges().fillet(fillet)
    return body.translate((0.0, 0.0, z_min))


def _pavement_slab() -> cq.Workplane:
    slab = cq.Workplane("XY").box(1.20, 0.90, 0.08).translate((0.0, 0.0, -0.04))
    opening = (
        cq.Workplane("XY")
        .circle(0.155)
        .extrude(0.14)
        .translate((0.0, 0.0, -0.10))
    )
    return slab.cut(opening).edges("|Z").fillet(0.008)


def _flap_plate() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(0.34, 0.30, 0.018)
        .translate((0.19, 0.0, 0.0))
        .edges("|Z")
        .fillet(0.018)
    )
    post_clearance = (
        cq.Workplane("XY")
        .circle(0.098)
        .extrude(0.05)
        .translate((0.19, 0.0, -0.025))
    )
    return plate.cut(post_clearance)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stainless_retractable_bollard")

    stainless = model.material("brushed_stainless", rgba=(0.72, 0.74, 0.72, 1.0))
    dark_steel = model.material("shadowed_steel", rgba=(0.18, 0.19, 0.19, 1.0))
    concrete = model.material("pavement_concrete", rgba=(0.40, 0.40, 0.37, 1.0))
    black = model.material("black_keyway", rgba=(0.01, 0.01, 0.009, 1.0))

    sleeve = model.part("sleeve")
    sleeve.visual(
        mesh_from_cadquery(_pavement_slab(), "pavement_slab", tolerance=0.002),
        material=concrete,
        name="pavement_slab",
    )
    sleeve.visual(
        mesh_from_cadquery(
            _annular_cylinder(0.130, 0.075, 1.10, z_min=-1.10, fillet=0.002),
            "sleeve_wall",
            tolerance=0.0015,
        ),
        material=dark_steel,
        name="sleeve_wall",
    )
    sleeve.visual(
        mesh_from_cadquery(
            _annular_cylinder(0.200, 0.108, 0.035, z_min=0.0, fillet=0.003),
            "collar_ring",
            tolerance=0.0015,
        ),
        material=stainless,
        name="collar_ring",
    )
    for i in range(8):
        angle = i * math.tau / 8.0
        sleeve.visual(
            Cylinder(radius=0.009, length=0.006),
            origin=Origin(
                xyz=(0.162 * math.cos(angle), 0.162 * math.sin(angle), 0.037)
            ),
            material=dark_steel,
            name=f"collar_bolt_{i}",
        )

    # Expansion grooves make the ground read as street paving rather than a slab
    # of generic gray material.  They are slightly inset visual strips.
    for i, x in enumerate((-0.42, 0.42)):
        sleeve.visual(
            Box((0.012, 0.90, 0.003)),
            origin=Origin(xyz=(x, 0.0, 0.001)),
            material=dark_steel,
            name=f"paving_joint_{i}",
        )

    # Fixed hinge hardware beside the sleeve opening.  The cover's moving
    # knuckle interleaves with these outer knuckles about the same short pin
    # line, so the flap is visibly carried by a real support rather than
    # floating over the pavement.
    sleeve.visual(
        Box((0.055, 0.260, 0.034)),
        origin=Origin(xyz=(-0.220, 0.0, 0.027)),
        material=stainless,
        name="hinge_base",
    )
    for i, y in enumerate((-0.078, 0.078)):
        sleeve.visual(
            Cylinder(radius=0.012, length=0.052),
            origin=Origin(xyz=(-0.190, y, 0.052), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=stainless,
            name=f"fixed_knuckle_{i}",
        )
    for i, y in enumerate((-0.109, 0.109)):
        sleeve.visual(
            Cylinder(radius=0.0045, length=0.014),
            origin=Origin(xyz=(-0.190, y, 0.052), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"pin_end_{i}",
        )

    post = model.part("post")
    post.visual(
        mesh_from_cadquery(
            _annular_cylinder(0.075, 0.055, 1.08, z_min=-0.220, fillet=0.0015),
            "post_tube",
            tolerance=0.001,
        ),
        material=stainless,
        name="post_tube",
    )
    for z in (0.10, 0.46, 0.82):
        post.visual(
            Cylinder(radius=0.0765, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=dark_steel,
            name=f"brushed_band_{z:.2f}",
        )

    cap = model.part("cap")
    cap.visual(
        Cylinder(radius=0.085, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=stainless,
        name="cap_disk",
    )
    cap.visual(
        Cylinder(radius=0.045, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, -0.0175)),
        material=dark_steel,
        name="cap_stem",
    )
    cap.visual(
        Box((0.064, 0.014, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0565)),
        material=black,
        name="key_slot",
    )
    cap.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(-0.024, 0.0, 0.0570)),
        material=black,
        name="key_bow_round",
    )

    flap = model.part("flap")
    flap.visual(
        mesh_from_cadquery(_flap_plate(), "cover_plate", tolerance=0.001),
        material=stainless,
        name="cover_plate",
    )
    flap.visual(
        Box((0.045, 0.092, 0.012)),
        origin=Origin(xyz=(0.018, 0.0, 0.0)),
        material=stainless,
        name="hinge_leaf",
    )
    flap.visual(
        Cylinder(radius=0.0115, length=0.082),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="moving_knuckle",
    )

    model.articulation(
        "sleeve_to_post",
        ArticulationType.PRISMATIC,
        parent=sleeve,
        child=post,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1800.0, velocity=0.30, lower=-0.85, upper=0.0),
    )
    model.articulation(
        "post_to_cap",
        ArticulationType.CONTINUOUS,
        parent=post,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.860)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0),
    )
    model.articulation(
        "sleeve_to_flap",
        ArticulationType.REVOLUTE,
        parent=sleeve,
        child=flap,
        origin=Origin(xyz=(-0.190, 0.0, 0.052)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.5, lower=0.0, upper=1.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    sleeve = object_model.get_part("sleeve")
    post = object_model.get_part("post")
    cap = object_model.get_part("cap")
    flap = object_model.get_part("flap")
    slide = object_model.get_articulation("sleeve_to_post")
    cap_spin = object_model.get_articulation("post_to_cap")
    flap_hinge = object_model.get_articulation("sleeve_to_flap")

    ctx.allow_overlap(
        post,
        sleeve,
        elem_a="post_tube",
        elem_b="sleeve_wall",
        reason=(
            "The post is intentionally represented as a captured sliding tube "
            "inside the below-grade guide sleeve so the retractable bollard has "
            "a grounded support path."
        ),
    )

    ctx.expect_within(
        post,
        sleeve,
        axes="xy",
        inner_elem="post_tube",
        outer_elem="sleeve_wall",
        margin=0.0,
        name="post is centered within sleeve guide",
    )
    ctx.expect_overlap(
        post,
        sleeve,
        axes="z",
        elem_a="post_tube",
        elem_b="sleeve_wall",
        min_overlap=0.15,
        name="extended post remains engaged in sleeve",
    )
    ctx.expect_overlap(
        flap,
        sleeve,
        axes="xy",
        elem_a="cover_plate",
        elem_b="collar_ring",
        min_overlap=0.18,
        name="closed flap bridges sleeve collar opening",
    )
    ctx.expect_gap(
        flap,
        sleeve,
        axis="z",
        positive_elem="cover_plate",
        negative_elem="collar_ring",
        min_gap=0.004,
        max_gap=0.014,
        name="cover flap sits just proud of collar",
    )
    ctx.expect_gap(
        flap,
        sleeve,
        axis="z",
        positive_elem="hinge_leaf",
        negative_elem="hinge_base",
        min_gap=0.0,
        max_gap=0.006,
        name="flap leaf is supported by hinge base",
    )
    ctx.expect_gap(
        cap,
        post,
        axis="z",
        positive_elem="cap_disk",
        negative_elem="post_tube",
        max_gap=0.002,
        max_penetration=0.0,
        name="separate cap seats on post tube rim",
    )

    rest_post_aabb = ctx.part_world_aabb(post)
    rest_cap_pos = ctx.part_world_position(cap)
    if rest_post_aabb is not None:
        ctx.check(
            "street barrier height",
            rest_post_aabb[1][2] > 0.88,
            details=f"post_aabb={rest_post_aabb}",
        )

    with ctx.pose({slide: -0.85}):
        retracted_post_aabb = ctx.part_world_aabb(post)
        retracted_cap_pos = ctx.part_world_position(cap)
        ctx.expect_within(
            post,
            sleeve,
            axes="xy",
            inner_elem="post_tube",
            outer_elem="sleeve_wall",
            margin=0.0,
            name="retracted post remains centered in sleeve guide",
        )
        ctx.expect_overlap(
            post,
            sleeve,
            axes="z",
            elem_a="post_tube",
            elem_b="sleeve_wall",
            min_overlap=0.70,
            name="retracted post nests inside sleeve",
        )

    ctx.check(
        "post slides down along sleeve axis",
        rest_post_aabb is not None
        and retracted_post_aabb is not None
        and retracted_post_aabb[1][2] < rest_post_aabb[1][2] - 0.70,
        details=f"rest={rest_post_aabb}, retracted={retracted_post_aabb}",
    )
    ctx.check(
        "cap follows vertical post slide",
        rest_cap_pos is not None
        and retracted_cap_pos is not None
        and retracted_cap_pos[2] < rest_cap_pos[2] - 0.70,
        details=f"rest={rest_cap_pos}, retracted={retracted_cap_pos}",
    )

    closed_flap_aabb = ctx.part_world_aabb(flap)
    with ctx.pose({flap_hinge: 1.25}):
        open_flap_aabb = ctx.part_world_aabb(flap)
    ctx.check(
        "flap rotates upward on side hinge",
        closed_flap_aabb is not None
        and open_flap_aabb is not None
        and open_flap_aabb[1][2] > closed_flap_aabb[1][2] + 0.18,
        details=f"closed={closed_flap_aabb}, open={open_flap_aabb}",
    )

    cap_center = ctx.part_world_position(cap)
    with ctx.pose({cap_spin: math.pi / 2.0}):
        spun_cap_center = ctx.part_world_position(cap)
    ctx.check(
        "cap spins about its short local axis",
        cap_center is not None
        and spun_cap_center is not None
        and abs(spun_cap_center[0] - cap_center[0]) < 0.001
        and abs(spun_cap_center[1] - cap_center[1]) < 0.001
        and abs(spun_cap_center[2] - cap_center[2]) < 0.001,
        details=f"center={cap_center}, spun={spun_cap_center}",
    )

    return ctx.report()


object_model = build_object_model()
